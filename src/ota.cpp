#include "inc/ota.hpp"

#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <string.h>

#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "mbedtls/sha256.h"

#include "inc/lcd.hpp"
#include "inc/debug.hpp"   // 放最后：_DEBUG=0 时 #define Serial 只影响其后代码

// ══════════════════════════════════════════════════════════════════════════
//  OTA 状态（存 NVS 命名空间 "ota"，与 SN 的 "device_info"、配置的 "egg_cfg" 分开）
//    st    : 0=IDLE  1=PENDING(要下载)  2=VERIFYING(新固件首次启动待自检)
//    tgt   : 目标版本号
//    fail  : 当前目标的连续失败次数
//    aband : 已放弃的版本号（官方 Topic 再报同一个则忽略，等更新的版本）
// ══════════════════════════════════════════════════════════════════════════
#define OTA_NVS_NS   "ota"
enum { OTA_IDLE = 0, OTA_PENDING = 1, OTA_VERIFYING = 2 };

struct OtaSt { uint8_t st; uint32_t tgt; uint32_t fail; uint32_t aband; };

static OtaSt otaRead() {
    OtaSt s{OTA_IDLE, 0, 0, 0};
    Preferences p;
    if (p.begin(OTA_NVS_NS, true)) {
        s.st    = p.getUChar("st", OTA_IDLE);
        s.tgt   = p.getUInt("tgt", 0);
        s.fail  = p.getUInt("fail", 0);
        s.aband = p.getUInt("aband", 0);
        p.end();
    }
    return s;
}
static void otaWrite(const OtaSt& s) {
    Preferences p;
    if (p.begin(OTA_NVS_NS, false)) {
        p.putUChar("st", s.st);
        p.putUInt("tgt", s.tgt);
        p.putUInt("fail", s.fail);
        p.putUInt("aband", s.aband);
        p.end();
    }
}

static int hexNib(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// 官方 Topic payload → 版本号。支持纯数字 "10002" 或 JSON {"ver":10002}。失败返回 0。
static uint32_t otaParseVer(const char* p, unsigned int len) {
    if (!p || len == 0) return 0;
    char buf[64];
    unsigned n = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
    memcpy(buf, p, n);
    buf[n] = 0;
    if (buf[0] == '{') {
        StaticJsonDocument<128> d;
        if (deserializeJson(d, buf)) return 0;
        return d["ver"] | 0u;
    }
    return (uint32_t)strtoul(buf, nullptr, 10);
}

// ── 纯净环境：读 NVS 配置连 WiFi（不初始化 Config 类，直接读 egg_cfg blob）──
//   命名空间/键与 configSys.cpp 的 CFG_NVS_NS / CFG_NVS_KEY 严格一致。
static bool otaConnectWifi() {
    String ssid, pass;
    {
        Preferences p;
        if (!p.begin("egg_cfg", true)) { LOG("[OTA] 无配置命名空间，未配网？\n"); return false; }
        String js = p.getString("blob", "");
        p.end();
        if (js.isEmpty()) { LOG("[OTA] 配置为空，未配网\n"); return false; }
        StaticJsonDocument<1024> d;
        if (deserializeJson(d, js)) { LOG("[OTA] 配置解析失败\n"); return false; }
        ssid = (const char*)(d["ssid"] | "");
        pass = (const char*)(d["password"] | "");
    }
    if (ssid.isEmpty()) { LOG("[OTA] SSID 为空，无法联网\n"); return false; }

    LOG("[OTA] 连接 WiFi: %s\n", ssid.c_str());
    lcdOtaMessage("WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 25000) delay(200);
    if (WiFi.status() != WL_CONNECTED) { LOG("[OTA] WiFi 连接超时\n"); return false; }
    LOG("[OTA] ✓ WiFi 已连接 %s\n", WiFi.localIP().toString().c_str());
    return true;
}

// 拉取期望 SHA256（OSS 上 firmware.sha256，内容为 64 位 hex）→ out[32]。
static bool otaFetchSha256(const char* url, uint8_t out[32]) {
    esp_http_client_config_t cfg = {};
    cfg.url               = url;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;   // 用 Arduino 自带 CA bundle 验证 OSS 证书
    cfg.timeout_ms        = 10000;
    esp_http_client_handle_t h = esp_http_client_init(&cfg);
    if (!h) return false;

    bool ok = false;
    char hex[80] = {0};
    if (esp_http_client_open(h, 0) == ESP_OK) {
        esp_http_client_fetch_headers(h);
        int code = esp_http_client_get_status_code(h);
        if (code == 200) {
            int r = esp_http_client_read(h, hex, sizeof(hex) - 1);
            if (r >= 64) {
                ok = true;
                for (int i = 0; i < 32; i++) {
                    int hi = hexNib(hex[2 * i]), lo = hexNib(hex[2 * i + 1]);
                    if (hi < 0 || lo < 0) { ok = false; break; }
                    out[i] = (uint8_t)((hi << 4) | lo);
                }
            }
        } else {
            LOG("[OTA] SHA256 文件 HTTP 状态 %d\n", code);
        }
    }
    esp_http_client_close(h);
    esp_http_client_cleanup(h);
    return ok;
}

// 下载固件 → 写另一分区 → 边下边算 SHA256 → 【校验通过才切启动分区】。返回 true=成功。
static bool otaDownloadAndFlash(const char* url, const uint8_t expSha[32]) {
    const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
    if (!next) { LOG("[OTA] 无可用 OTA 目标分区\n"); return false; }
    LOG("[OTA] 目标分区 %s (offset 0x%06x, size %u)\n",
        next->label, (unsigned)next->address, (unsigned)next->size);

    esp_http_client_config_t cfg = {};
    cfg.url               = url;
    cfg.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.timeout_ms        = 20000;
    cfg.buffer_size       = 2048;
    esp_http_client_handle_t h = esp_http_client_init(&cfg);
    if (!h) return false;

    bool ok = false;
    esp_ota_handle_t ota = 0;
    do {
        if (esp_http_client_open(h, 0) != ESP_OK) { LOG("[OTA] HTTP open 失败\n"); break; }
        int total = esp_http_client_fetch_headers(h);
        int code  = esp_http_client_get_status_code(h);
        if (code != 200) { LOG("[OTA] 固件 HTTP 状态 %d（版本目录不存在？）\n", code); break; }
        if (total <= 0) total = (int)next->size;   // chunked 未知长度时用分区大小做进度上界

        if (esp_ota_begin(next, OTA_SIZE_UNKNOWN, &ota) != ESP_OK) {
            LOG("[OTA] esp_ota_begin 失败\n"); ota = 0; break;
        }

        mbedtls_sha256_context sha;
        mbedtls_sha256_init(&sha);
        mbedtls_sha256_starts(&sha, 0);   // 0 = SHA-256

        static uint8_t buf[2048];
        int  readTotal = 0, lastPct = -1;
        bool rdErr = false;
        while (true) {
            int r = esp_http_client_read(h, (char*)buf, sizeof(buf));
            if (r < 0) { LOG("[OTA] HTTP read 错误\n"); rdErr = true; break; }
            if (r == 0) {
                if (esp_http_client_is_complete_data_received(h)) break;   // 正常收完
                rdErr = true; break;                                       // 提前断开
            }
            if (esp_ota_write(ota, buf, r) != ESP_OK) { LOG("[OTA] esp_ota_write 失败\n"); rdErr = true; break; }
            mbedtls_sha256_update(&sha, buf, r);
            readTotal += r;
            int pct = total > 0 ? (int)((int64_t)readTotal * 100 / total) : 0;
            if (pct != lastPct) { lastPct = pct; lcdOtaProgress(pct); }
        }
        uint8_t got[32];
        mbedtls_sha256_finish(&sha, got);
        mbedtls_sha256_free(&sha);

        if (rdErr) { esp_ota_abort(ota); ota = 0; break; }
        if (esp_ota_end(ota) != ESP_OK) { LOG("[OTA] esp_ota_end 失败（镜像无效？）\n"); ota = 0; break; }
        ota = 0;   // end 之后 handle 失效

        if (memcmp(got, expSha, 32) != 0) { LOG("[OTA] ✗ SHA256 不符，拒绝切分区\n"); break; }
        LOG("[OTA] ✓ SHA256 校验通过（%d 字节）\n", readTotal);

        if (esp_ota_set_boot_partition(next) != ESP_OK) { LOG("[OTA] set_boot_partition 失败\n"); break; }
        ok = true;
    } while (0);

    if (ota) esp_ota_abort(ota);
    esp_http_client_close(h);
    esp_http_client_cleanup(h);
    return ok;
}

// 失败收尾：失败计数 +1；达上限则放弃该版本(退回旧固件)，否则保持 PENDING 重启重试。不返回。
static void otaFail(OtaSt s, const char* why) {
    s.fail++;
    LOG("[OTA] ✗ %s（失败 %u/%u）\n", why, (unsigned)s.fail, (unsigned)OTA_MAX_FAIL);
    if (s.fail >= OTA_MAX_FAIL) {
        s.aband = s.tgt; s.st = OTA_IDLE; s.fail = 0;
        otaWrite(s);
        LOG("[OTA] 达失败上限，放弃版本 %u，退回旧固件\n", (unsigned)s.tgt);
        lcdOtaMessage("Failed");
    } else {
        otaWrite(s);   // 保持 PENDING
        lcdOtaMessage("Retry");
    }
    Serial.flush();
    delay(2000);
    ESP.restart();     // 不返回
}

// 纯净 OTA 下载环境（otaBootCheck 检测到 PENDING 时进入）。不返回：成功切分区重启 / 失败重启。
static void runOtaDownload(OtaSt s) {
    LOG("[OTA] ═══ 纯净 OTA 环境：目标版本=%u（已失败 %u 次）═══\n",
        (unsigned)s.tgt, (unsigned)s.fail);
    lcdOtaBegin();
    lcdOtaProgress(0);

    if (!otaConnectWifi()) { otaFail(s, "WiFi 连接失败"); return; }

    char urlBin[192], urlSha[192];
    snprintf(urlBin, sizeof(urlBin), "%s/%u/firmware.bin",    OTA_URL_BASE, (unsigned)s.tgt);
    snprintf(urlSha, sizeof(urlSha), "%s/%u/firmware.sha256", OTA_URL_BASE, (unsigned)s.tgt);

    // 域名白名单（当前 URL 由固定 OTA_URL_BASE 拼出，天然满足；留作防御，若日后 URL 改由
    // Topic 下发则挡住指向非官方域名的地址）。
    if (!strstr(urlBin, OTA_URL_HOST)) { otaFail(s, "URL 域名不在白名单"); return; }

    LOG("[OTA] 固件: %s\n", urlBin);
    uint8_t expSha[32];
    if (!otaFetchSha256(urlSha, expSha)) { otaFail(s, "拉取 SHA256 失败"); return; }

    if (!otaDownloadAndFlash(urlBin, expSha)) { otaFail(s, "下载/校验失败"); return; }

    // 成功：置 VERIFYING，重启进入新固件（新固件跑到运行期 → otaMarkValidOnBoot 确认）。
    s.st = OTA_VERIFYING;
    otaWrite(s);
    LOG("[OTA] ✓ 已切启动分区，重启进入新固件 %u\n", (unsigned)s.tgt);
    lcdOtaProgress(100);
    lcdOtaMessage("Done, reboot");
    Serial.flush();
    delay(1000);
    ESP.restart();   // 不返回
}

// ┌──────────────────────────── 对外接口 ────────────────────────────┐

uint32_t otaCurrentVersion() { return (uint32_t)FW_VERSION; }

void otaOnOfficialVersion(const char* payload, unsigned int len) {
    uint32_t ver = otaParseVer(payload, len);
    if (ver == 0)              return;                 // 解析失败
    if (ver <= FW_VERSION)     return;                 // 已最新或更旧
    OtaSt s = otaRead();
    if (ver == s.aband)        return;                 // 已放弃该版本，等更新的
    // 触发升级
    s.st = OTA_PENDING;
    if (s.tgt != ver) s.fail = 0;                      // 新目标 → 失败计数清零
    s.tgt = ver;
    otaWrite(s);
    LOG("[OTA] 官方版本 %u > 本机 %u → PENDING，重启进入 OTA 环境\n",
        (unsigned)ver, (unsigned)FW_VERSION);
    Serial.flush();
    delay(300);
    ESP.restart();   // 不返回
}

void otaBootCheck() {
    OtaSt s = otaRead();

    if (s.st == OTA_VERIFYING) {
        if (FW_VERSION == s.tgt) {
            // 新固件正在跑，等 setup 末尾 otaMarkValidOnBoot() 确认，这里正常返回
            LOG("[OTA] 新固件 %u 首次启动（VERIFYING），待运行期确认\n", (unsigned)s.tgt);
            return;
        }
        // 运行的是旧固件 = 新固件启动失败被 bootloader 回滚了 → 记一次失败
        s.fail++;
        LOG("[OTA] 新固件 %u 启动失败被回滚（失败 %u/%u）\n",
            (unsigned)s.tgt, (unsigned)s.fail, (unsigned)OTA_MAX_FAIL);
        if (s.fail >= OTA_MAX_FAIL) {
            s.aband = s.tgt; s.st = OTA_IDLE; s.fail = 0;
            otaWrite(s);
            LOG("[OTA] 达失败上限，放弃版本 %u，留在旧固件\n", (unsigned)s.aband);
            return;
        }
        s.st = OTA_PENDING;   // 重新下载
        otaWrite(s);
        // 继续走下面的 PENDING 分支
    }

    if (s.st == OTA_PENDING) {
        runOtaDownload(s);   // 不返回
    }
    // OTA_IDLE → 正常启动
}

void otaMarkValidOnBoot() {
    OtaSt s = otaRead();
    if (s.st != OTA_VERIFYING) return;      // 不是新固件待自检
    if (s.tgt != FW_VERSION)   return;      // 版本不匹配（保险）
    // 跑到运行期 = 启动成功（rollback 已兜住"启动即崩"，这里确认取消回滚）。
    // ★ 这是"启动级"自检；"启动成功但功能坏"需功能级自检，属后续增强。
    esp_err_t e = esp_ota_mark_app_valid_cancel_rollback();
    s.st = OTA_IDLE; s.fail = 0;
    otaWrite(s);
    LOG("[OTA] ✓ 新固件 %u 启动成功，已 mark_valid 取消回滚（err=%d）\n",
        (unsigned)FW_VERSION, (int)e);
}
