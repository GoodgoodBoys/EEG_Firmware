#include <stdio.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <time.h>
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "mbedtls/md.h"

#include "inc/debug.hpp"
#include "inc/fs.hpp"
#include "inc/msg.hpp"
#include "inc/web.hpp"
#include "inc/lcd.hpp"
#include "inc/aud.hpp"          // 音频挂起/恢复/直收接口
#include "inc/configSys.hpp"
#include "inc/nvs_sn.h"
#include "inc/provision.hpp"
#include "inc/mic.hpp"          // VOICE_MAX / VOICE_PATH_FMT（语音存储）
#include "esp_task_wdt.h"       // 任务级看门狗：稳态纳入监控，配网/连WiFi 等长阻塞前退订

#define WIFI_WARTTING_TIME 30

// ==================== 参数配置 ====================
#define MQTT_BUFFER_SIZE          8192

#define AUDIO_FILE_PATH           "/output1.mp3"

// ── NTP 时间同步 ──
#define NTP_SERVER1               "pool.ntp.org"
#define NTP_SERVER2               "ntp.aliyun.com"
#define GMT_OFFSET_SEC            (8 * 3600)
#define DAYLIGHT_OFFSET_SEC       0

// 数据通道上 JSON 控制消息的最大长度（用于区分二进制数据与控制帧）
#define CTRL_JSON_MAX_LEN         240
// ==================================================

#define WEB_L2_POLL_MS  500   // L2 下 MQTT 轮询周期 ≈ 推送唤醒延迟上限。
                              // ↑省电 ↓延迟；保持 ≤700 以确保推送在 1s 内被处理。

#define XFER_RX_TIMEOUT_MS  30000  // 传输中超过这么久没再收到分片 → 判 App 掉线，
                                   // 自动回滚解锁，避免设备永久卡在锁定态耗电。

char clientId[48];
char pubAddr[64];
char subCmdAddr[64];
char subVideoDataAddr[64];
char subAudioDataAddr[64];   // dev/$sn/audio

char subPosHbAddr[64];   // dev/<sn>/posHeartbeat（订阅，App→设备心跳）
char posPubAddr[64];     // dev/<sn>/pos（发布，设备→App 姿态；也被伙伴订阅做舵机镜像）
char voicePubAddr[64];   // term/<sn>/voice（发布，设备→App 语音文件）
char peerPosSubAddr[64] = {0}; // dev/<peerSn>/pos（订阅，伙伴姿态→本机舵机镜像 P3）
char peerInfoSubAddr[64] = {0}; // devInfo/<peerSn>（订阅，伙伴在线状态→配对爱心图标）
char subVoicePlayAddr[64] = {0}; // dev/<sn>/voicePlay（订阅，伙伴推来的语音→即播即删 P4）

char MQTT_BROKER[64] = DEF_SERVER_IP;
int  MQTT_PORT       = DEF_SERVER_PORT;

extern QueueHandle_t qMainToWeb;
extern QueueHandle_t qWebToMain;

extern void mainWake();         // 唤醒主循环（V1_1.cpp）

extern volatile int g_pwrTier;   // 0/1/2 功耗档位（V1_1.ino 定义）
extern volatile int g_xferAnim;  // LCD 传输动画（lcd.cpp）：0=无 / 1=转圈 / 2=打勾

WiFiClientSecure espClient;
PubSubClient     client(espClient);

const char* videoPath = LCD_MOTION_VIDEOSHOW_PATH;

extern NvsSn g_sn;

// ── 设备信息 ──
static String   g_sn_str;
static String   g_topic_id;        // topicId(本机SN)：dev/ 主题加盐后的令牌（开机算一次）
static String   devInfoTopic;      // devInfo/<sn>
static String   g_willPayload;     // 遗嘱（offline）
static String   responseTopic;     // term/<sn>/response

// ── 状态机 ──
typedef enum {
    XFER_IDLE,
    XFER_RECEIVING_AUDIO,
    XFER_WAIT_VIDEO,
    XFER_RECEIVING_VIDEO,
} xferState_t;

static xferState_t xferState = XFER_IDLE;

// ══════════════════════════════════════════════════════════════
//  直收进 PSRAM（接收期不碰 flash）
//    NEW_* 时向 LCD/audio 要一块 PSRAM 基址；分片直接 memcpy 进去；
//    END 时校验 → commit（直接当播放缓存）或 abort（从 flash 回滚）；
//    整段成功后，把 PSRAM 内容后台写回 flash 做持久化。
// ══════════════════════════════════════════════════════════════

static volatile bool receiveError = false;
static uint8_t*      g_recvBase   = nullptr;   // 当前接收基址（指向 LCD/audio 的 PSRAM）
static size_t        g_recvMax    = 0;         // 可写上限
static size_t        g_recvLen    = 0;         // 已收字节

// 暂存音频接收信息，等 END_VIDEO 与视频一并持久化
static const uint8_t* g_audioSrc = nullptr;
static size_t         g_audioLen = 0;

// 后台持久化（PSRAM → flash）
typedef struct { const uint8_t* src; size_t len; char path[64]; } PersistJob_t;
static QueueHandle_t     qPersist        = nullptr;
static TaskHandle_t      xPersistTask     = nullptr;
volatile int             g_persistPending = 0;   // 在途持久化作业数（排空屏障；lcd.cpp 读取判"存盘中"→降载）
static volatile bool     g_persistAllOk   = true; // 本轮所有落盘是否都成功
static volatile bool     g_videoResultPend = false; // VIDEO 结果待落盘完成后补发
static size_t            g_videoResultBytes = 0;  // 补发 VIDEO 结果时用的字节数

// 最近一次收到"应用消息"（订阅主题真有 PUBLISH）的时刻。
// 只在 mqttCallback 里更新——MQTT 心跳 PINGRESP 不进回调，不会刷新它。
// 供：本文件 WiFi 睡眠管理 + 主循环 CPU 频率仲裁 共同读取。
volatile uint32_t g_lastMqttRxMs = 0;

volatile uint32_t g_lastHeartbeatMs = 0;   // 最近一次心跳（独立于 g_lastMqttRxMs）
volatile bool     g_posStreaming    = false;
// 设备→App 语音上行中：抬 CPU 满频 + WiFi 满功率加速上行（屏幕仍按空闲档休眠，
// 故不走 forceActive，只在 powerManagerLoop 里单独保 CPU 频率，见 V1_1.cpp）。
volatile bool     g_voiceSending    = false;
// 视频/音频下载传输中（enterTransferMode→true / exitTransferMode→false）。
// imu.cpp/lcd.cpp 读取：传输期 IMU 识别本被屏蔽 → 降采样减少 I2C 访问（降卡死概率）；
// LCD 降到 6fps 给 core0 松绑。
volatile bool     g_xfering         = false;
extern QueueHandle_t qPosStream;

// ── P3 摇摆镜像状态 ──
#define POS_BROADCAST_MS     4000   // 摇一摇后广播自身姿态给伙伴的时长
#define MIRROR_SEND_MIN_MS   100    // 舵机更新最快 ~10Hz（节流）
#define MIRROR_DEADZONE_DEG  5      // 角度变化 <5° 不动（去抖 + 减队列压力）
#define MIRROR_TIMEOUT_MS    1500   // 超时无姿态 → 舵机归中位
static uint32_t g_shakeBroadcastUntil = 0;   // A：广播自身姿态的截止时刻
static String   g_subPeer;                   // B：当前已订阅 pos 的伙伴 SN（跟踪绑定变化）
static int      g_mirrorLastAngle  = -1;     // B：上次发给舵机的角度
static uint32_t g_mirrorLastSendMs = 0;
static uint32_t g_mirrorLastRxMs   = 0;      // B：上次收到伙伴姿态的时刻
static bool     g_mirrorActive     = false;

// ── 配对爱心图标：已互绑 且 伙伴在线 ──
static bool   g_peerOnline = false;          // 伙伴 devInfo.status==online
volatile bool g_pairLinked = false;          // lcd.cpp 读取：已绑 且 伙伴在线
volatile bool g_wifiOnline  = false;         // lcd.cpp 读取：WiFi 是否已连接（WiFi 状态图标）
volatile bool g_needWifiHint = false;        // lcd.cpp 读取：有配置但连不上 → 显示"长按2秒配置"引导画面

// 性能统计
#define WEB_PERF_LOG  1
static uint32_t g_xferStartMs = 0;

// ══════════════════════════════════════════════════════════════
//  时间戳
// ══════════════════════════════════════════════════════════════

static String getTimestamp()
{
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &timeinfo);
        return String(buf);
    }
    return "uptime_" + String(millis());
}

static uint32_t getUnixTime()
{
    time_t now;
    time(&now);
    return (now > 1000000000) ? (uint32_t)now : 0;
}

// ══════════════════════════════════════════════════════════════
//  勿扰时段判断（配网时经 BLE 写入 Config 的 dndPeriods）
//    格式 "s-e,s-e"：半小时索引 s∈0..47(00:00~23:30) / e∈1..48(00:30~24:00)。
//    s<e  = 当天段 [s,e)；
//    s>=e = 跨 0 点段（晚上到次日凌晨）[s,48) ∪ [0,e)。
//    NTP 未同步 → 返回 false（不勿扰），避免时间不准误拦消息。
// ══════════════════════════════════════════════════════════════
static bool isInDndPeriod()
{
    String dnd = Config.getString("dndPeriods", "");
    if (dnd.isEmpty()) return false;

    struct tm t;
    if (!getLocalTime(&t, 50)) return false;              // 时间没同步 → 不勿扰
    int cur = t.tm_hour * 2 + (t.tm_min >= 30 ? 1 : 0);   // 当前半小时索引 0..47

    const char* p = dnd.c_str();
    while (*p) {
        int s = atoi(p);
        const char* dash = strchr(p, '-');
        if (!dash) break;
        int e = atoi(dash + 1);
        bool hit = (s < e) ? (cur >= s && cur < e)         // 当天段
                           : (cur >= s || cur < e);        // 跨 0 点段
        if (hit) return true;
        const char* comma = strchr(dash, ',');
        if (!comma) break;
        p = comma + 1;
    }
    return false;
}

// ══════════════════════════════════════════════════════════════
//  JSON 工具
// ══════════════════════════════════════════════════════════════

static bool tryParseControlMsg(const byte* payload, unsigned int length,
                               char* msgOut, size_t msgOutSize)
{
    if (length == 0 || length > CTRL_JSON_MAX_LEN) return false;
    if (payload[0] != '{') return false;

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, payload, length)) return false;

    String m = doc["msg"] | "";
    if (strlen(m.c_str()) == 0) return false;

    strncpy(msgOut, m.c_str(), msgOutSize - 1);
    msgOut[msgOutSize - 1] = '\0';
    return true;
}

// ══════════════════════════════════════════════════════════════
//  设备身份密钥：MQTT 密码 = HMAC-SHA256(盐, SN) 的前 128bit hex
//    取代"密码=SN"的弱认证。盐是编译期机密，必须与 EMQX 侧校验用的盐一致。
//    ★ 换 broker 认证方式前别烧此固件，否则连不上（EMQX 要能算同样的 HMAC 校验，
//      或把各设备 SN→key 导入内置认证；开机日志会打印本机 key 便于登记）。
// ══════════════════════════════════════════════════════════════
#define DEVICE_KEY_SALT   "egg-v1-2026-CHANGE-THIS-SECRET-SALT"   // ← 上线前改成你自己的机密盐

// ★ MQTT 认证开关：
//   1 = 用派生密钥 HMAC(盐,SN)（安全，但 EMQX 必须先能校验，否则连不上=错误码4）
//   0 = 临时退回"密码=SN"（弱认证，仅当 EMQX 还没配好新认证时先恢复联调用）
#define MQTT_AUTH_DERIVED_KEY   0

static String deriveDeviceKey(const String& sn)
{
    uint8_t mac[32];
    const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!info) return sn;   // 兜底：拿不到算法则退回旧行为（不应发生）
    mbedtls_md_hmac(info,
        (const uint8_t*)DEVICE_KEY_SALT, strlen(DEVICE_KEY_SALT),
        (const uint8_t*)sn.c_str(), sn.length(),
        mac);
    char hex[33];
    for (int i = 0; i < 16; i++) sprintf(hex + i * 2, "%02x", mac[i]);  // 前 128bit
    hex[32] = 0;
    return String(hex);
}

// ══════════════════════════════════════════════════════════════
//  主题加盐：把 dev/<SN>/* 里的 SN 换成不可猜的令牌，防外人往设备发消息。
//  令牌 = HMAC-SHA256(TOPIC_SALT, SN) 前 64bit hex（16 字符）。
//  ⚠ 必须与 App 的 topic_id.dart(_kTopicSalt/topicId) 完全一致，否则两端主题对不上。
//  这是"藏主题"级防护：配合"不能通配订阅"更稳；盐改了则所有设备+App 要一起更新。
// ══════════════════════════════════════════════════════════════
#define TOPIC_SALT   "egg-topic-2026-CHANGE-THIS-SALT"   // ← 上线前改成你自己的机密盐

static String topicId(const String& sn)
{
    uint8_t mac[32];
    const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (!info) return sn;
    mbedtls_md_hmac(info,
        (const uint8_t*)TOPIC_SALT, strlen(TOPIC_SALT),
        (const uint8_t*)sn.c_str(), sn.length(),
        mac);
    char hex[17];
    for (int i = 0; i < 8; i++) sprintf(hex + i * 2, "%02x", mac[i]);   // 前 64bit
    hex[16] = 0;
    return String(hex);
}

static void publishCmdToTerminal(const String& terminal, const char* msgVal)
{
    if (terminal.isEmpty()) {
        LOG("[TERM] ⚠ terminal 为空，跳过发送 %s\n", msgVal);
        return;
    }
    char topic[96];
    snprintf(topic, sizeof(topic), "term/%s/cmd", terminal.c_str());

    StaticJsonDocument<256> doc;
    doc["msg"]       = msgVal;
    doc["sn"]        = g_sn_str;   // 带上本机 SN：App 收 NEW_VOICE 可精确拉取该设备
    doc["timestamp"] = getTimestamp();

    char payload[192];
    serializeJson(doc, payload, sizeof(payload));

    bool ok = client.publish(topic, payload);
    LOG("[TERM] → %s : %s (%s)\n", topic, payload, ok ? "OK" : "FAIL");
}

// ══════════════════════════════════════════════════════════════
//  设备信息发布
// ══════════════════════════════════════════════════════════════

static void buildDevInfoJson(const char* status, char* out, size_t outSize)
{
    StaticJsonDocument<384> doc;
    doc["status"]    = status;
    doc["SN"]        = g_sn_str;
    doc["matchCode"] = Config.getString("matchCode", "");
    doc["peerSn"]    = Config.getString("peerSn", "");   // 互绑伙伴 SN（空=未绑），供 App 读绑定态
    doc["timestamp"] = getTimestamp();
    uint32_t ut = getUnixTime();
    if (ut > 0) doc["unixTime"] = ut;
    serializeJson(doc, out, outSize);
}

static void publishDevInfo(const char* status)
{
    char payload[384];
    buildDevInfoJson(status, payload, sizeof(payload));
    bool ok = client.publish(devInfoTopic.c_str(), payload, true);
    LOG("[DEV] 发布设备信息(%s) %s: %s\n", status, ok ? "成功" : "失败", payload);
}

// ══════════════════════════════════════════════════════════════
//  后台持久化任务（core 1）：传输结束后把 PSRAM 写回 flash
//    非关键路径，分块 + 每块让出，不会卡接收（接收已结束）。
// ══════════════════════════════════════════════════════════════

static void persistTask(void* p)
{
    LOG("[PST] 持久化任务启动 (core %d)\n", xPortGetCoreID());
    PersistJob_t job;
    while (true) {
        if (xQueueReceive(qPersist, &job, portMAX_DELAY) != pdTRUE) continue;

        if (job.src && job.len > 0) {
            String tmp = String(job.path) + ".tmp";
            File f = LittleFS.open(tmp.c_str(), FILE_WRITE);
            if (!f) {
                LOG("[PST] ✗ 打开失败 %s\n", tmp.c_str());
                g_persistAllOk = false;
            } else {
                uint32_t t0 = millis();
                size_t off = 0; bool ok = true;
                while (off < job.len) {
                    // ★ 摊薄落盘：flash 写/擦会经 IPC 冻结 core0 cache（spi_flash_op_block_func
                    //   自旋），单次擦大量 sector 冻结 >8s → 饿死 core0 触发 TWDT。
                    //   关键：不能只调 write 大小——LittleFS 会把多次小写攒在缓冲、最后一次性
                    //   flush 成大块擦写，仍长冻结。必须【每 4KB 写完立即 f.flush() 强制小步落盘】，
                    //   把每次 flash 操作压到 ~1 个 sector（冻结数十 ms），再 vTaskDelay 让 core0
                    //   恢复喂狗。总落盘时间被拉长（后台非关键路径，可接受）。
                    size_t n = job.len - off; if (n > 4096) n = 4096;
                    size_t w = f.write(job.src + off, n);
                    if (w != n) { ok = false; break; }
                    off += w;
                    f.flush();                       // ★ 逼 LittleFS 立即小步落盘，避免攒批大 flush
                    vTaskDelay(pdMS_TO_TICKS(15));    // 让 core0 从 flash 冻结中恢复、喂狗
                }
                f.flush(); f.close();

                if (ok) {
                    if (LittleFS.exists(job.path)) LittleFS.remove(job.path);
                    if (!LittleFS.rename(tmp.c_str(), job.path)) LittleFS.remove(tmp.c_str());
                    LOG("[PST] ✓ %s 持久化 %uKB / %lums\n", job.path,
                        (unsigned)(job.len / 1024), (unsigned long)(millis() - t0));
                } else {
                    LittleFS.remove(tmp.c_str());
                    g_persistAllOk = false;
                    LOG("[PST] ✗ %s 持久化失败（PSRAM 仍可播，下次开机用旧文件）\n", job.path);
                }
            }
        }

        if (g_persistPending > 0) g_persistPending--;
    }
}

static void enqueuePersist(const uint8_t* src, size_t len, const char* path)
{
    PersistJob_t job; job.src = src; job.len = len;
    strncpy(job.path, path, sizeof(job.path) - 1); job.path[sizeof(job.path) - 1] = 0;
    g_persistPending++;
    if (xQueueSend(qPersist, &job, 0) != pdTRUE) {
        g_persistPending--;
        g_persistAllOk = false;
        LOG("[PST] ⚠ 持久化队列满，跳过 %s\n", path);
    }
}

// 等所有在途持久化完成——新一轮接收会复用同一片 PSRAM，
// 必须先等旧持久化读完，避免边写 flash 边被覆盖。
static void waitPersistIdle()
{
    if (g_persistPending <= 0) return;
    LOG("[PST] 等待上轮持久化排空（%d 个在途）...\n", g_persistPending);
    while (g_persistPending > 0) { vTaskDelay(pdMS_TO_TICKS(20)); esp_task_wdt_reset(); }  // 喂狗（未订阅则空操作）
    LOG("[PST] 持久化已排空\n");
}

// ══════════════════════════════════════════════════════════════
//  接收缓冲：直写 PSRAM
// ══════════════════════════════════════════════════════════════

static void appendRecv(const uint8_t* payload, size_t length)
{
    if (!g_recvBase || receiveError) return;
    if (g_recvLen + length > g_recvMax) {
        LOG("[WEB] ✗ 接收溢出: %u+%u > %u\n",
            (unsigned)g_recvLen, (unsigned)length, (unsigned)g_recvMax);
        receiveError = true;
        return;
    }
    memcpy(g_recvBase + g_recvLen, payload, length);
    g_recvLen += length;
}

static void resetRecvPerf(const char* what)
{
    receiveError = false;
    g_recvLen = 0;
    g_xferStartMs = millis();
    if (WEB_PERF_LOG)
        LOG("[PERF] ▶ 开始接收 %s | RSSI=%d dBm | 堆=%uKB | PSRAM空=%uKB\n",
            what, WiFi.RSSI(), ESP.getFreeHeap() / 1024, ESP.getFreePsram() / 1024);
}

static void logXferPerf(const char* tag)
{
    if (!WEB_PERF_LOG) return;
    uint32_t totalMs = millis() - g_xferStartMs;
    uint32_t kb   = g_recvLen / 1024;
    uint32_t kbps = totalMs ? (uint32_t)((uint64_t)g_recvLen * 1000ULL / 1024ULL / totalMs) : 0;
    LOG("┌─[PERF] %s 接收完成（直收 PSRAM，零 flash 写）─\n", tag);
    LOG("│ 总量 %uKB | 总耗时 %lums | 吞吐 %lu KB/s | RSSI %d dBm\n",
        kb, (unsigned long)totalMs, (unsigned long)kbps, WiFi.RSSI());
    LOG("└──────────────────────────────────────────\n");
}

// ══════════════════════════════════════════════════════════════
//  传输锁定/解锁
// ══════════════════════════════════════════════════════════════

static void enterTransferMode()
{
    g_xfering = true;           // 传输期：IMU 降采样 + LCD 降 6fps（见 imu.cpp/lcd.cpp）
    waitPersistIdle();          // 复用 PSRAM 前，先等上轮持久化排空
    LOG("[XFER] ── 锁定：停止播放 ──\n");
    MessageToMain_t msg; strcpy(msg.strVal, "XFER_LOCK");
    xQueueSend(qWebToMain, &msg, 0);
    mainWake();
}

static void exitTransferMode()
{
    g_xfering = false;
    LOG("[XFER] ── 解锁：恢复播放 ──\n");
    MessageToMain_t msg; strcpy(msg.strVal, "XFER_UNLOCK");
    xQueueSend(qWebToMain, &msg, 0);
    mainWake();
}

static void publishResultBytes(const char* tag, bool success, size_t bytes)
{
    StaticJsonDocument<128> doc;
    char msgVal[16];
    snprintf(msgVal, sizeof(msgVal), "%s_%s", tag, success ? "OK" : "FAIL");
    doc["msg"]       = msgVal;
    doc["timestamp"] = getTimestamp();

    char payload[128];
    serializeJson(doc, payload, sizeof(payload));

    LOG("[%s] %s | %d 字节 | → %s : %s\n",
        tag, success ? "成功" : "失败", (int)bytes, responseTopic.c_str(), payload);
    client.publish(responseTopic.c_str(), payload);
}

static void publishResult(const char* tag, bool success)
{
    publishResultBytes(tag, success, g_recvLen);
}

// ══════════════════════════════════════════════════════════════
//  设备匹配
// ══════════════════════════════════════════════════════════════

static void handleMatchCommand(const char* msgBuffer)
{
    size_t len = strlen(msgBuffer);
    if (len < 11) return;
    const char* firstDash = msgBuffer + 5;
    if (*firstDash != '-') return;
    const char* lastDash = strrchr(msgBuffer, '-');
    if (!lastDash || lastDash == firstDash) return;
    size_t termIdLen = lastDash - (firstDash + 1);
    if (termIdLen == 0 || termIdLen > 128) return;
    char termId[129] = {0};
    memcpy(termId, firstDash + 1, termIdLen);
    const char* codeStr = lastDash + 1;
    size_t codeLen = strlen(codeStr);
    if (codeLen < 3 || codeLen > 8) return;                 // 兼容 3~8 位（新版 6 位）
    for (size_t i = 0; i < codeLen; i++) if (codeStr[i] < '0' || codeStr[i] > '9') return;
    String localCode = Config.getString("matchCode", "");
    if (localCode.isEmpty() || localCode != String(codeStr)) return;
    String deviceSn;
    SnError snRet = g_sn.read(deviceSn);
    if (snRet != SnError::OK) deviceSn = "UNKNOWN";
    char topic[192], payload[128];
    snprintf(topic, sizeof(topic), "term/%s/match/response", termId);
    snprintf(payload, sizeof(payload), "MATCHSUSS-%s", deviceSn.c_str());
    client.publish(topic, payload);
}

// ══════════════════════════════════════════════════════════════
//  命令分发
// ══════════════════════════════════════════════════════════════

// ══════════════════════════════════════════════════════════════
//  语音消息发送（设备→App，阶段D）
//    App 发 PULL_VOICE → 设备把 voice_N.mp3 逐条发到 term/<sn>/voice
//    （VOICE_BEGIN json → 二进制分片 → VOICE_END json）；
//    收到 App 的 VOICE_OK(带 id) 后删除该文件，再发下一条。
// ══════════════════════════════════════════════════════════════
static volatile bool g_pullVoiceReq  = false;
static volatile bool g_voiceAcked    = false;
static volatile int  g_voiceAckId    = -1;
static int      g_voiceSendingId     = -1;      // 正在发/等 ack 的槽，-1=空闲
static uint32_t g_voiceAckDeadline   = 0;

// 找第一个已存在的语音文件槽；没有返回 -1
static int findStoredVoiceSlot() {
    char p[24];
    for (int i = 0; i < VOICE_MAX; i++) {
        snprintf(p, sizeof(p), VOICE_PATH_FMT, i);
        if (LittleFS.exists(p)) return i;
    }
    return -1;
}

// 发送一条语音：VOICE_BEGIN(json) → 二进制分片 → VOICE_END(json)。
// 返回 true=整条发完；false=中途断链/publish 失败（未发 VOICE_END，调用方据此
// 不进等待态、保留请求下轮重试，免得白等 15s ack 超时。
static bool sendVoiceFileTo(int slot, const char* topic) {
    char path[24];
    snprintf(path, sizeof(path), VOICE_PATH_FMT, slot);
    File f = LittleFS.open(path, FILE_READ);
    if (!f) { LOG("[VOICE] ✗ 打不开 %s\n", path); return false; }
    size_t sz = f.size();

    StaticJsonDocument<160> b;
    b["msg"]  = "VOICE_BEGIN";
    b["sn"]   = g_sn_str;
    b["id"]   = slot;
    b["size"] = (uint32_t)sz;
    char jb[160]; serializeJson(b, jb, sizeof(jb));
    if (!client.publish(topic, jb)) {
        LOG("[VOICE] ✗ VOICE_BEGIN 发送失败（缓冲满/断链），中止槽 %d\n", slot);
        f.close();
        return false;
    }
    client.loop();

    static uint8_t buf[4096];
    size_t total = 0, n;
    while ((n = f.read(buf, sizeof(buf))) > 0) {
        // 分片发失败（缓冲满或断链）：不再发后续分片和 VOICE_END，让 App 收不齐、
        // 设备走重传逻辑。硬发完只会让 App 算出 size 不符再丢，白费一轮 15s。
        if (!client.connected() ||
            !client.publish(topic, buf, (unsigned int)n)) {
            LOG("[VOICE] ✗ 分片发送失败（已发 %u 字节），中止槽 %d 待重传\n",
                (unsigned)total, slot);
            f.close();
            return false;
        }
        total += n;
        client.loop();
        vTaskDelay(pdMS_TO_TICKS(6));   // 放慢一点，避免 Serverless broker 限速丢片
        esp_task_wdt_reset();           // 发大文件不回主循环，这里喂狗（未订阅则空操作）
    }
    f.close();

    StaticJsonDocument<64> e;
    e["msg"] = "VOICE_END";
    e["id"]  = slot;
    char je[64]; serializeJson(e, je, sizeof(je));
    if (!client.publish(topic, je)) {
        LOG("[VOICE] ✗ VOICE_END 发送失败，中止槽 %d 待重传\n", slot);
        return false;
    }
    LOG("[VOICE] 已发送槽 %d：%u 字节 → %s\n", slot, (unsigned)total, topic);
    return true;
}

// 发给 App 的一条语音（term/<sn>/voice，带 ack 后删文件）
static bool sendVoiceFile(int slot) { return sendVoiceFileTo(slot, voicePubAddr); }

// 找最新（编号最大）的已存语音槽 = 刚录的那条；没有返回 -1
static int findLatestVoiceSlot() {
    char p[24];
    for (int i = VOICE_MAX - 1; i >= 0; i--) {
        snprintf(p, sizeof(p), VOICE_PATH_FMT, i);
        if (LittleFS.exists(p)) return i;
    }
    return -1;
}

// P4：把某条语音推给已绑定伙伴的 dev/<peer>/voicePlay（即播即删，fire-and-forget 不等 ack）
static void pushVoiceToPeer(int slot, const String& peer) {
    char topic[64];
    snprintf(topic, sizeof(topic), "dev/%s/voicePlay", topicId(peer).c_str());
    LOG("[PVOICE] 推送语音槽 %d → %s\n", slot, topic);
    sendVoiceFileTo(slot, topic);
}

// webTask 每轮调用：驱动语音发送状态机
static void handleVoiceSend() {
    if (xferState != XFER_IDLE) return;   // 与 app→device 传输互斥
    if (!client.connected()) return;

    if (g_voiceSendingId >= 0) {          // 正在等某条的 ack
        if (g_voiceAcked && g_voiceAckId == g_voiceSendingId) {
            char p[24]; snprintf(p, sizeof(p), VOICE_PATH_FMT, g_voiceSendingId);
            LittleFS.remove(p);
            LOG("[VOICE] 槽 %d 已送达，删除\n", g_voiceSendingId);
            g_voiceSendingId = -1;
            g_voiceAcked = false;
            // 本条完成，继续往下尝试发下一条（不再 return，尽快清空队列）
        } else if ((int32_t)(millis() - g_voiceAckDeadline) >= 0) {
            LOG("[VOICE] 槽 %d 等 ack 超时，保留待重传\n", g_voiceSendingId);
            g_voiceSendingId = -1;        // 不删，下次 PULL 再发
        } else {
            g_voiceSending = true;        // 仍在等 ack，保持高性能
            return;
        }
    }

    if (!g_pullVoiceReq) { g_voiceSending = false; return; }

    int slot = findStoredVoiceSlot();
    if (slot < 0) {                       // 全部发完
        g_pullVoiceReq = false; g_voiceSending = false;
        if (g_xferAnim == 1) g_xferAnim = 2;   // ★ 语音全部上传成功 → LCD 打勾
        return;
    }

    g_voiceSending = true;                // ★ 发大文件前抬性能：CPU 满频 + WiFi 满功率
    if (g_xferAnim != 1) g_xferAnim = 1;  // ★ 开始/进行语音上传 → LCD 转圈
    if (!sendVoiceFile(slot)) {
        // 中途断链/缓冲满：不进等待态，保留 g_pullVoiceReq，下轮（连上后）重试本槽
        LOG("[VOICE] 槽 %d 发送中断，保留请求下轮重试\n", slot);
        return;                           // 保持 g_voiceSending=true，下轮重试
    }
    g_voiceSendingId   = slot;
    g_voiceAcked       = false;
    g_voiceAckDeadline = millis() + 15000;
}

// ══════════════════════════════════════════════════════════════
//  设备互绑（P1）：设备侧做 matchCode 授权 + CAS 互斥（防两端同时抢绑）
//    App 发 BIND{peer,matchCode} 到 dev/<sn>/cmd；本机校验 matchCode==自身、
//    且当前未绑或已绑同一 peer 才接受，存 peerSn 并重发 retained devInfo；
//    结果回 term/<sn>/response（BIND_OK / BIND_REJECTED+reason）。
// ══════════════════════════════════════════════════════════════
static void publishBindResult(const char* result, const char* reason, const char* peer)
{
    StaticJsonDocument<192> d;
    d["msg"]  = result;
    d["sn"]   = g_sn_str;
    d["peer"] = peer ? peer : "";
    if (reason) d["reason"] = reason;
    char payload[192];
    serializeJson(d, payload, sizeof(payload));
    client.publish(responseTopic.c_str(), payload);
    LOG("[BIND] → %s : %s\n", responseTopic.c_str(), payload);
}

static void handleBind(JsonDocument& doc)
{
    String peer = doc["peer"]      | "";
    String code = doc["matchCode"] | "";
    if (peer.isEmpty()) { publishBindResult("BIND_REJECTED", "no_peer", ""); return; }

    // ① 授权：必须带对本机 matchCode（防陌生人凭 SN 猜绑）
    String localCode = Config.getString("matchCode", "");
    if (localCode.isEmpty() || code != localCode) {
        publishBindResult("BIND_REJECTED", "bad_code", peer.c_str());
        LOG("[BIND] ✗ 拒绝 peer=%s：matchCode 不符\n", peer.c_str());
        return;
    }

    // ② CAS 互斥：未绑 或 已绑同一个 → 接受；已绑他人 → 拒（1:1 排他）
    String cur = Config.getString("peerSn", "");
    if (!cur.isEmpty() && cur != peer) {
        publishBindResult("BIND_REJECTED", "busy", peer.c_str());
        LOG("[BIND] ✗ 拒绝 peer=%s：已绑 %s\n", peer.c_str(), cur.c_str());
        return;
    }

    Config.setString("peerSn", peer.c_str());
    Config.save();
    publishDevInfo("online");            // 重发 retained devInfo（含新 peerSn）
    publishBindResult("BIND_OK", nullptr, peer.c_str());
    LOG("[BIND] ✓ 已绑定 peer=%s\n", peer.c_str());
}

static void handleUnbind(JsonDocument& doc)
{
    String peer = doc["peer"] | "";
    String cur  = Config.getString("peerSn", "");
    bool changed = false;
    if (!cur.isEmpty() && (peer.isEmpty() || cur == peer)) {   // peer 空=无条件解绑
        Config.setString("peerSn", "");
        Config.save();
        publishDevInfo("online");
        changed = true;
    }
    StaticJsonDocument<128> d;
    d["msg"] = "UNBIND_OK";
    d["sn"]  = g_sn_str;
    char payload[128];
    serializeJson(d, payload, sizeof(payload));
    client.publish(responseTopic.c_str(), payload);
    LOG("[BIND] 解绑%s（原 peer=%s）\n", changed ? "成功" : "(本就未绑)", cur.c_str());
}

// 送达回执：收到方 → 向发起方 dev/<toSn>/cmd 回一条 ackMsg（带本机 SN）
static void sendPeerAck(const String& toSn, const char* ackMsg)
{
    if (toSn.isEmpty()) return;
    char topic[64];
    snprintf(topic, sizeof(topic), "dev/%s/cmd", topicId(toSn).c_str());
    StaticJsonDocument<128> d;
    d["msg"]       = ackMsg;
    d["from"]      = g_sn_str;
    d["timestamp"] = getTimestamp();
    char pl[128];
    serializeJson(d, pl, sizeof(pl));
    client.publish(topic, pl);
    LOG("[ACK] → %s : %s\n", topic, ackMsg);
}

static void dispatchCommand(const char* msg, JsonDocument& doc)
{
    // ── 设备互绑（P1）──
    if (strcmp(msg, "BIND") == 0)   { handleBind(doc);   return; }
    if (strcmp(msg, "UNBIND") == 0) { handleUnbind(doc); return; }

    // ── 伙伴送达回执 → 转给 App 显示"已送达"（PEER_ACK 不在 EMQX 推送白名单，不触发推送）──
    if (strcmp(msg, "POKE_ACK") == 0 || strcmp(msg, "VOICE_ACK") == 0) {
        String from     = doc["from"] | "";
        String terminal = Config.getString("terminal", "");
        if (!terminal.isEmpty()) {
            char topic[96];
            snprintf(topic, sizeof(topic), "term/%s/cmd", terminal.c_str());
            StaticJsonDocument<192> d;
            d["msg"]       = "PEER_ACK";
            d["kind"]      = (strcmp(msg, "POKE_ACK") == 0) ? "poke" : "voice";
            d["peer"]      = from;                       // 哪台设备收到了
            d["timestamp"] = getTimestamp();
            char pl[192];
            serializeJson(d, pl, sizeof(pl));
            client.publish(topic, pl);
        }
        // 拍一拍戳伙伴收到送达回执 → 通知 LCD 播 wink（表示"发送成功"）
        if (strcmp(msg, "POKE_ACK") == 0) {
            char m[WEB_MSG_LEN]; strncpy(m, "WINK", sizeof(m) - 1); m[sizeof(m) - 1] = 0;
            xQueueSend(qWebToMain, &m, 0);
        }
        LOG("[ACK] 收到伙伴回执 %s from=%s → 转 App\n", msg, from.c_str());
        return;
    }

    // ── POKE（P2）：伙伴拍了它的蛋 → 本机播放拍一拍视频（复用 target 槽 + 音频）──
    if (strcmp(msg, "POKE") == 0) {
        if (xferState != XFER_IDLE) return;          // 传输中不打断（也不回执=发起方会超时提示忙）
        if (isInDndPeriod()) {                        // 勿扰时段：不播、不回 POKE_ACK（发起方超时）
            LOG("[DND] 勿扰时段，忽略伙伴 POKE\n");
            return;
        }
        MessageToMain_t toMain;
        strcpy(toMain.strVal, "VIDEO_SHOW");         // 复用 Main 的播放处理（target 视频+音频）
        xQueueSend(qWebToMain, &toMain, 0);
        mainWake();
        sendPeerAck(doc["from"] | "", "POKE_ACK");   // ★ 送达回执给发起方
        LOG("[POKE] 收到伙伴拍一拍，播放 target 视频\n");
        return;
    }

    // ── PULL_VOICE / VOICE_OK：语音消息拉取与确认（阶段D）──
    if (strcmp(msg, "PULL_VOICE") == 0) {
        g_pullVoiceReq = true;
        LOG("[VOICE] 收到 App 拉取请求\n");
        return;
    }
    if (strcmp(msg, "VOICE_OK") == 0) {
        g_voiceAckId = doc["id"] | -1;
        g_voiceAcked = true;
        LOG("[VOICE] 收到 App 确认 id=%d\n", g_voiceAckId);
        return;
    }

    // ── NEW_AUDIO：音频直收进 600KB 音频池 ──
    if (strcmp(msg, "NEW_AUDIO") == 0) {
        LOG("\n[AUDIO] 开始接收音频（直收 PSRAM）\n");
        if (xferState == XFER_IDLE) enterTransferMode();
        size_t maxB = 0;
        g_recvBase = audRecvBegin(&maxB);
        g_recvMax  = maxB;
        resetRecvPerf(AUDIO_FILE_PATH);
        if (!g_recvBase) { receiveError = true; LOG("[AUDIO] ✗ 音频池不可用\n"); }
        xferState = XFER_RECEIVING_AUDIO;
        return;
    }

    // ── NEW_VIDEO：视频直收进 LCD 池 target 槽 ──
    if (strcmp(msg, "NEW_VIDEO") == 0) {
        LOG("\n[VIDEO] 开始接收视频（直收 PSRAM）\n");
        if (xferState == XFER_IDLE) enterTransferMode();
        size_t maxB = 0;
        g_recvBase = lcdTargetRecvBegin(&maxB);
        g_recvMax  = maxB;
        resetRecvPerf(videoPath);
        if (!g_recvBase) { receiveError = true; LOG("[VIDEO] ✗ LCD 池无空间\n"); }
        xferState = XFER_RECEIVING_VIDEO;
        return;
    }

    // ── MATCH ──
    if (strncasecmp(msg, "MATCH-", 6) == 0) {
        handleMatchCommand(msg);
        return;
    }

    // ── VIDEO_SHOW（携带 term 字段）──
    if (strcmp(msg, "VIDEO_SHOW") == 0) {
        if (xferState != XFER_IDLE) {
            LOG("[WEB] ⚠ 传输中，屏蔽 VIDEO_SHOW\n");
            return;
        }
        if (isInDndPeriod()) {   // 勿扰时段：不显示、不回 DEV_RECEIVED（对方 App 会超时复位）
            LOG("[DND] 勿扰时段，忽略 VIDEO_SHOW\n");
            return;
        }

        String term = doc["terminal"] | "";
        if (strlen(term.c_str()) > 0) {
            String oldTerm = Config.getString("terminal", "");
            if (oldTerm != String(term.c_str())) {
                Config.setString("terminal", term.c_str());
                Config.save();
                LOG("[WEB] terminal 更新: %s → %s\n", oldTerm.c_str(), term.c_str());
            } else {
                LOG("[WEB] terminal 不变: %s\n", term.c_str());
            }
        } else {
            LOG("[WEB] ⚠ VIDEO_SHOW 未携带 term 字段\n");
        }

        String terminal = Config.getString("terminal", "");
        publishCmdToTerminal(terminal, "DEV_RECEIVED");

        MessageToMain_t toMain;
        strcpy(toMain.strVal, "VIDEO_SHOW");
        xQueueSend(qWebToMain, &toMain, 0);
        mainWake();
        return;
    }

    // ── 其它播放/控制指令：传输中屏蔽，否则转发 Main ──
    if (strcmp(msg, "RECORD") == 0 ||
        strncasecmp(msg, "SERVO", 5) == 0 ||
        strncasecmp(msg, "WBL", 3) == 0) {
        if (xferState != XFER_IDLE) {
            LOG("[WEB] ⚠ 传输中，屏蔽: %s\n", msg);
            return;
        }
        MessageToMain_t toMain;
        strncpy(toMain.strVal, msg, sizeof(toMain.strVal) - 1);
        toMain.strVal[sizeof(toMain.strVal) - 1] = '\0';
        xQueueSend(qWebToMain, &toMain, 0);
        mainWake();
        return;
    }

    LOG("[WEB] ⚠ 未知 msg: %s\n", msg);
}

// ══════════════════════════════════════════════════════════════
//  统一回滚（任意失败/中止时把音视频都恢复到 flash 旧状态）
// ══════════════════════════════════════════════════════════════

static void rollbackAll()
{
    if (xferState == XFER_RECEIVING_VIDEO) lcdTargetRecvAbort();  // 视频回 flash 旧文件
    audRecvAbort();                                              // 音频回 flash 旧文件
    g_audioSrc = nullptr;
}

// ══════════════════════════════════════════════════════════════
//  MQTT 回调
// ══════════════════════════════════════════════════════════════

// ── P4：伙伴语音接收（即播即删）──
static File     g_pvFile;
static bool     g_pvReceiving = false;
static String   g_pvFrom;               // 本条语音的发送方 SN（VOICE_BEGIN 带来，收齐后回执）
static uint32_t g_pvExpect = 0;         // VOICE_BEGIN 声明的字节数
static uint32_t g_pvGot    = 0;         // 实收字节数（收不全=丢片，播了是杂音）
#define PEER_VOICE_TMP   "/peer_voice.mp3.tmp"
#define PEER_VOICE_PATH  "/peer_voice.mp3"

// 收伙伴推来的语音：VOICE_BEGIN→写临时文件→VOICE_END→交 aud 即播即删（复用 App 语音协议）
static void handlePeerVoice(byte* payload, unsigned int length)
{
    char ctrl[32];
    if (tryParseControlMsg(payload, length, ctrl, sizeof(ctrl))) {
        if (strcmp(ctrl, "VOICE_BEGIN") == 0) {
            if (xferState != XFER_IDLE) return;      // 传输中不收（音频池/I2S 忙）
            if (isInDndPeriod()) {                   // 勿扰时段：拒收伙伴语音（不播）
                LOG("[DND] 勿扰时段，拒收伙伴语音\n");
                return;
            }
            StaticJsonDocument<160> b;               // 取发送方 SN + 声明大小
            bool ok = !deserializeJson(b, payload, length);
            g_pvFrom   = ok ? String((const char*)(b["sn"] | "")) : String();
            g_pvExpect = ok ? (uint32_t)(b["size"] | 0) : 0;
            g_pvGot    = 0;
            if (g_pvFile) g_pvFile.close();
            LittleFS.remove(PEER_VOICE_TMP);
            g_pvFile = LittleFS.open(PEER_VOICE_TMP, FILE_WRITE);
            g_pvReceiving = (bool)g_pvFile;
            LOG("[PVOICE] 开始接收伙伴语音（from=%s size=%u）\n",
                g_pvFrom.c_str(), (unsigned)g_pvExpect);
        } else if (strcmp(ctrl, "VOICE_END") == 0) {
            if (g_pvReceiving) {
                g_pvFile.close();
                g_pvReceiving = false;
                // ★ 收不全 = 分片丢了，播出来就是杂音，直接丢弃不播（日志暴露丢片问题）
                if (g_pvExpect > 0 && g_pvGot != g_pvExpect) {
                    LOG("[PVOICE] ✗ 收不全 %u/%u，丢弃不播（分片丢失）\n",
                        (unsigned)g_pvGot, (unsigned)g_pvExpect);
                    LittleFS.remove(PEER_VOICE_TMP);
                    return;
                }
                LittleFS.remove(PEER_VOICE_PATH);
                LittleFS.rename(PEER_VOICE_TMP, PEER_VOICE_PATH);
                audPlayFile(PEER_VOICE_PATH, true);  // 加载进池后删源，播完自动恢复原音频
                sendPeerAck(g_pvFrom, "VOICE_ACK");  // ★ 送达回执给发送方
                LOG("[PVOICE] 收齐 %u 字节 → 即播即删\n", (unsigned)g_pvGot);
            }
        }
        return;
    }
    if (g_pvReceiving) { g_pvFile.write(payload, length); g_pvGot += length; }
}

// ── P3：收到伙伴姿态 → 映射到本机舵机（经 Main 的 SERVO 处理，单任务驱动舵机）──
static void handlePeerPos(byte* payload, unsigned int length)
{
    StaticJsonDocument<96> d;
    if (deserializeJson(d, payload, length)) return;
    int ax = d["ax"] | 0;                       // 伙伴的横向加速度(mg)
    int angle = 87 + (ax * 35) / 1000;          // 中位87°，±1000mg→±35°(52~122)；单轴镜像
    if (angle < 52)  angle = 52;
    if (angle > 122) angle = 122;

    g_mirrorLastRxMs = millis();
    g_mirrorActive   = true;

    uint32_t now = millis();
    if (now - g_mirrorLastSendMs < MIRROR_SEND_MIN_MS) return;              // 节流
    if (g_mirrorLastAngle >= 0 &&
        abs(angle - g_mirrorLastAngle) < MIRROR_DEADZONE_DEG) return;        // 死区
    g_mirrorLastAngle  = angle;
    g_mirrorLastSendMs = now;

    // 无真实舵机也能据此确认镜像链路：收到伙伴 ax → 映射角度（已节流+死区，不刷屏）
    LOG("[MIRROR] 伙伴 ax=%d mg → 舵机 %d°\n", ax, angle);

    MessageToMain_t toMain;
    snprintf(toMain.strVal, sizeof(toMain.strVal), "SERVO%d", angle);
    xQueueSend(qWebToMain, &toMain, 0);
    mainWake();
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    // ★ posHeartbeat：只刷新心跳时间戳，绝不刷新 g_lastMqttRxMs（心跳不唤醒低功耗）
    if (strcmp(topic, subPosHbAddr) == 0) {
        g_lastHeartbeatMs = millis();
        return;
    }
    // ★ P3：伙伴姿态（高频）→ 舵机镜像。同样不刷 g_lastMqttRxMs，靠 Main 的 SERVO
    //    markActivity 保活；镜像结束自然降功耗。
    if (peerPosSubAddr[0] && strcmp(topic, peerPosSubAddr) == 0) {
        handlePeerPos(payload, length);
        return;
    }
    // ★ 伙伴在线状态（devInfo）→ 决定配对爱心亮不亮
    if (peerInfoSubAddr[0] && strcmp(topic, peerInfoSubAddr) == 0) {
        StaticJsonDocument<256> d;
        if (!deserializeJson(d, payload, length)) {
            const char* st = d["status"] | "";
            g_peerOnline = (strcmp(st, "online") == 0);
            LOG("[PAIR] 伙伴 %s\n", g_peerOnline ? "在线" : "离线");
        }
        return;
    }
    // 收到任何应用消息：刷新时间戳（WiFi 退出睡眠 + 主循环抬 CPU 都靠它）
    g_lastMqttRxMs = millis();

    // ── 命令主题：整包 JSON ──
    if (strcmp(topic, subCmdAddr) == 0) {
        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, payload, length);
        if (err) {
            LOG("[WEB] ✗ 命令 JSON 解析失败: %s\n", err.c_str());
            return;
        }
        String msg = doc["msg"] | "";
        String ts  = doc["timestamp"] | "";
        if (strlen(msg.c_str()) == 0) {
            LOG("[WEB] ✗ 命令缺少 msg 字段\n");
            return;
        }
        LOG("[WEB] 命令: %s (ts=%s)\n", msg.c_str(), ts.c_str());
        dispatchCommand(msg.c_str(), doc);
        return;
    }

    // ── 音频数据主题 dev/$sn/audio ──
    else if (strcmp(topic, subAudioDataAddr) == 0) {
        char ctrl[32];
        bool isCtrl = tryParseControlMsg(payload, length, ctrl, sizeof(ctrl));

        if (isCtrl && strcmp(ctrl, "END_AUDIO") == 0) {
            if (xferState != XFER_RECEIVING_AUDIO) return;
            LOG("\n[AUDIO] END_AUDIO\n");
            logXferPerf("AUDIO");
            bool ok = (!receiveError && g_recvLen > 4 && audRecvCommit(g_recvLen));
            if (ok) { g_audioSrc = g_recvBase; g_audioLen = g_recvLen; }  // 等 END_VIDEO 一并持久化
            else    { audRecvAbort(); g_audioSrc = nullptr; }
            publishResult("AUDIO", ok);
            xferState = XFER_WAIT_VIDEO;
            LOG("[XFER] 音频%s，等待 NEW_VIDEO...\n", ok ? "完成" : "失败");
            return;
        }
        if (isCtrl && strcmp(ctrl, "ABORT_TRANSFER") == 0) {
            LOG("[XFER] ⚠ ABORT（音频通道）\n");
            rollbackAll();
            xferState = XFER_IDLE; exitTransferMode();
            return;
        }

        if (xferState == XFER_RECEIVING_AUDIO) appendRecv(payload, length);
    }

    // ── 视频数据主题 ──
    else if (strcmp(topic, subVideoDataAddr) == 0) {
        char ctrl[32];
        bool isCtrl = tryParseControlMsg(payload, length, ctrl, sizeof(ctrl));

        if (isCtrl && strcmp(ctrl, "END_VIDEO") == 0) {
            if (xferState != XFER_RECEIVING_VIDEO) return;
            LOG("\n[VIDEO] END_VIDEO\n");
            logXferPerf("VIDEO");

            // 校验 MJPEG 头尾（直接在 PSRAM 上查）
            bool headTail = (g_recvLen > 4 &&
                             g_recvBase[0] == 0xFF && g_recvBase[1] == 0xD8 &&
                             g_recvBase[g_recvLen - 2] == 0xFF && g_recvBase[g_recvLen - 1] == 0xD9);

            bool ok = (!receiveError && headTail && lcdTargetRecvCommit(g_recvLen));

            if (ok) {
                // 提交成功：内容已可从 PSRAM 播放。落盘改为后台进行，
                // VIDEO_OK 推迟到落盘真正写入 flash 后，由 webTask 主循环补发，
                // 这样服务器收到 OK 即代表内容已持久化（掉电不丢）。
                g_persistAllOk     = true;
                g_videoResultBytes = g_recvLen;
                if (g_audioSrc) { enqueuePersist(g_audioSrc, g_audioLen, AUDIO_FILE_PATH); g_audioSrc = nullptr; }
                enqueuePersist(g_recvBase, g_recvLen, videoPath);
                g_videoResultPend = true;
                LOG("[VIDEO] 提交成功，待落盘完成后回 VIDEO_OK\n");
            } else {
                LOG("[VIDEO] ✗ 校验/提交失败 (headTail=%d recvErr=%d) → 回滚\n",
                    headTail, receiveError);
                rollbackAll();
                publishResult("VIDEO", false);   // 失败立即回
            }

            xferState = XFER_IDLE; exitTransferMode();
            return;
        }
        if (isCtrl && strcmp(ctrl, "ABORT_TRANSFER") == 0) {
            LOG("[XFER] ⚠ ABORT（视频通道）\n");
            rollbackAll();
            xferState = XFER_IDLE; exitTransferMode();
            return;
        }

        if (xferState == XFER_RECEIVING_VIDEO) appendRecv(payload, length);
    }

    // ── 伙伴语音（P4）dev/<sn>/voicePlay：即播即删 ──
    else if (subVoicePlayAddr[0] && strcmp(topic, subVoicePlayAddr) == 0) {
        handlePeerVoice(payload, length);
    }
}

// ══════════════════════════════════════════════════════════════
//  WiFi
// ══════════════════════════════════════════════════════════════

static bool connectWiFiFromConfig()
{
    String ssid = Config.getString("ssid",     "");
    String pass = Config.getString("password", "");
    if (ssid.isEmpty() || pass.isEmpty()) return false;
    WiFi.begin(ssid.c_str(), pass.c_str());
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 60) { vTaskDelay(500); retry++; }
    if (WiFi.status() == WL_CONNECTED) {
        LOG("[WiFi] ✓ %s\n", WiFi.localIP().toString().c_str());
        snprintf(clientId, sizeof(clientId), "ALIVE_%s", WiFi.macAddress().c_str());
        return true;
    }
    WiFi.disconnect(true);
    return false;
}

// ── WiFi 省电分级 ──
//   传输/姿态串流 → PS_NONE     满功率，保证下载吞吐与实时
//   其余（L0/L1/L2）→ MIN_MODEM  统一轻省、低延迟。原 L2 的 MAX_MODEM 已去掉
//                     （MAX 跳多个 DTIM 会让推送延迟到秒级，得不偿失）
static void applyWifiPowerSave()
{
    static wifi_ps_type_t cur = WIFI_PS_MIN_MODEM;
    bool xfering = (xferState != XFER_IDLE);

    wifi_ps_type_t want;
    if (xfering || g_posStreaming || g_voiceSending)
                                    want = WIFI_PS_NONE;        // 下载/串流/语音上行满功率
    else                            want = WIFI_PS_MIN_MODEM;   // ★ L0/L1/L2 统一 MIN_MODEM（原 L2 的 MAX 去掉）

    if (want != cur) {
        esp_wifi_set_ps(want);
        cur = want;
        LOG("[WIFI] PS → %s\n", want == WIFI_PS_NONE ? "NONE" : "MIN_MODEM");
    }
}


// ══════════════════════════════════════════════════════════════
//  webTask
// ══════════════════════════════════════════════════════════════

void webTask(void *webParameter)
{
    LOG("[WEB Task] 启动\n");

    // ── 后台持久化任务（core 1）。不再需要双缓冲，省下 1MB PSRAM。──
    qPersist = xQueueCreate(4, sizeof(PersistJob_t));
    xTaskCreatePinnedToCore(persistTask, "Persist", 4096, NULL, 1, &xPersistTask, 1);

    SnError ret = g_sn.read(g_sn_str);
    if (ret != SnError::OK) g_sn_str = "00000002";
    g_topic_id = topicId(g_sn_str);          // dev/ 主题加盐令牌（开机算一次）

    responseTopic = "term/" + g_sn_str + "/response";
    devInfoTopic  = "devInfo/" + g_sn_str;
    LOG("[SEC] topicId=%s（dev/ 主题令牌）\n", g_topic_id.c_str());
    LOG("[SEC] SN=%s  MQTT-KEY=%s  ← 登记到 EMQX 认证\n",
        g_sn_str.c_str(), deriveDeviceKey(g_sn_str).c_str());
    LOG("[WEB] 回复主题: %s\n", responseTopic.c_str());
    LOG("[WEB] 设备信息: %s\n", devInfoTopic.c_str());
    LOG("[WEB] 当前 terminal: %s\n", Config.getString("terminal", "(空)").c_str());
    LOG("[BIND] 本机 peerSn=%s（空=未绑，则不会推语音给伙伴）\n",
        Config.getString("peerSn", "(未绑)").c_str());

    // 清理可能残留的 .tmp（持久化中途掉电留下的）
    String tmpV = String(videoPath) + ".tmp";
    String tmpA = String(AUDIO_FILE_PATH) + ".tmp";
    if (LittleFS.exists(tmpV.c_str())) LittleFS.remove(tmpV.c_str());
    if (LittleFS.exists(tmpA.c_str())) LittleFS.remove(tmpA.c_str());

    espClient.setInsecure();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    client.setCallback(mqttCallback);
    client.setKeepAlive(30);
    client.setBufferSize(MQTT_BUFFER_SIZE);
    // ★ readByte() 内部是 while(!available()) yield() 忙等，上限=socketTimeout（默认 15s）。
    //   传输期 webTask 升到 P1，yield() 让不到 P0 的 IDLE0 → 收包卡顿时空转饿死 IDLE0，
    //   >5s 触发 TWDT panic（实测 backtrace 卡在 readByte→ssl_read→lwip）。收到 3s：
    //   忙等最多 3s < 5s，网络卡顿即返回触发重连，绝不再撑到看门狗。
    client.setSocketTimeout(3);

    // dev/ 系列主题用加盐令牌 g_topic_id（防外人往设备发消息）；term/、devInfo/ 仍用 SN
    snprintf(subCmdAddr, sizeof(subCmdAddr),
             "%s/%s/%s", TOPIC_SUB_HEADER, g_topic_id.c_str(), TOPIC_SUB_CMD);
    snprintf(subVideoDataAddr, sizeof(subVideoDataAddr),
             "%s/%s/%s", TOPIC_SUB_HEADER, g_topic_id.c_str(), TOPIC_SUB_VIDEO);
    snprintf(subAudioDataAddr, sizeof(subAudioDataAddr),
             "dev/%s/audio", g_topic_id.c_str());
    snprintf(pubAddr, sizeof(pubAddr),
             "%s/%s", TOPIC_PUB_HEADER, g_sn_str.c_str());
    snprintf(subPosHbAddr, sizeof(subPosHbAddr), "dev/%s/posHeartbeat", g_topic_id.c_str());
    snprintf(posPubAddr,   sizeof(posPubAddr),   "dev/%s/pos",          g_topic_id.c_str());
    snprintf(voicePubAddr, sizeof(voicePubAddr), "term/%s/voice",       g_sn_str.c_str());
    snprintf(subVoicePlayAddr, sizeof(subVoicePlayAddr), "dev/%s/voicePlay", g_topic_id.c_str());
    LOG("[PVOICE] 本机收语音主题: %s（伙伴推给我时应发到这个主题）\n", subVoicePlayAddr);

    bool ntpStarted = false;
    bool boosted    = false;     // 传输期是否已升优先级

    // ── TWDT 纳入/退订（幂等）：只在稳态 MQTT 处理段监控；配网 / 连 WiFi（最长 30s）/
    //    MQTT 连接这些用户触发或本就慢的长阻塞路径先退订，避免误 panic。──
    bool wdtSubscribed = false;
    auto wdtOn  = [&]{ if (!wdtSubscribed) { esp_task_wdt_add(NULL);    wdtSubscribed = true;  } };
    auto wdtOff = [&]{ if (wdtSubscribed)  { esp_task_wdt_delete(NULL); wdtSubscribed = false; } };

    while (true) {

        g_wifiOnline = (WiFi.status() == WL_CONNECTED);   // 供 LCD 的 WiFi 状态图标

        // ── 配网触发：按键 / 无凭据 / 连接失败 ──
        bool needProvision = false;

        if (g_enterProvisioning) {
            g_enterProvisioning = false;
            LOG("[WEB] 按键触发配网\n");
            needProvision = true;
        }
        else if (provisionNeeded()) {
            LOG("[WEB] 未配置 WiFi，自动进入配网\n");
            needProvision = true;
        }
        else if (WiFi.status() != WL_CONNECTED) {
            wdtOff();   // connectWiFiFromConfig 最长阻塞 30s > 5s，先退订
            if (connectWiFiFromConfig()) {
                g_needWifiHint = false;              // 连上 → 撤下配网引导
                Config.stopAdvertising();
                if (!ntpStarted) {
                    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
                    ntpStarted = true;
                    LOG("[WEB] NTP 同步已启动\n");
                }
            } else {
                // ★ 有配置但连不上（每次尝试 30s）：不再自动进配网，改为显示"长按2秒配置"引导，
                //   由用户长按按键触发配网（g_enterProvisioning）。后台继续重试，连上即撤引导。
                g_needWifiHint = true;
                LOG("[WEB] WiFi 连不上，显示配网引导，等长按 2s 配网\n");
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }
        }

        // ── 执行配网：挂起音频让出 SRAM → 懒加载 BLE → 配网 → 释放 BLE → 恢复音频 ──
        if (needProvision) {
            wdtOff();   // provisionButtonFlow 等用户操作，可阻塞数分钟，必须退订
            if (boosted) { vTaskPrioritySet(NULL, 0); boosted = false; }

            if (client.connected()) client.disconnect();
            espClient.stop();

            audSuspend();
            uint32_t suspT0 = millis();
            while (!audIsSuspended() && millis() - suspT0 < 2000)
                vTaskDelay(pdMS_TO_TICKS(10));

            Config.ensureBLEInit();
            provisionButtonFlow();
            Config.deinitBLE();
            audResume();

            if (WiFi.status() == WL_CONNECTED && !ntpStarted) {
                configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
                ntpStarted = true;
                LOG("[WEB] NTP 同步已启动\n");
            }
            continue;
        }

        // ── WiFi 已连接，处理 MQTT ──
        if (!client.connected()) {
            wdtOff();   // TLS 握手 + connect 可能阻塞数秒，先退订
            client.setServer(MQTT_BROKER, MQTT_PORT);

            char willBuf[384];
            buildDevInfoJson("offline", willBuf, sizeof(willBuf));
            g_willPayload = String(willBuf);

            LOG("[MQTT] 连接 %s:%d（含遗嘱）\n", MQTT_BROKER, MQTT_PORT);

            // 密码：派生密钥（安全）或临时退回 SN（见 MQTT_AUTH_DERIVED_KEY 开关）
#if MQTT_AUTH_DERIVED_KEY
            String devPass = deriveDeviceKey(g_sn_str);
#else
            String devPass = g_sn_str;   // 临时弱认证：EMQX 配好新认证前先恢复联调
#endif
            bool connected = client.connect(
                clientId,
                g_sn_str.c_str(), devPass.c_str(),
                devInfoTopic.c_str(), 1, true, g_willPayload.c_str());

            if (connected) {
                LOG("[MQTT] ✓ 已连接\n");
                client.subscribe(subCmdAddr, 1);
                client.subscribe(subVideoDataAddr, 1);   // 如需更快可改 QoS 0
                client.subscribe(subAudioDataAddr, 1);
                client.subscribe(subPosHbAddr, 0);   // 心跳，QoS0
                client.subscribe(subVoicePlayAddr, 1);   // P4：伙伴推来的语音
                publishDevInfo("online");
                client.publish(pubAddr, "device_online", true);
            } else {
                LOG("[MQTT] ✗ 错误码: %d\n", client.state());
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
        }

        // ── 稳态 MQTT 处理开始：纳入 TWDT（含传输期 client.loop 收包），循环末尾 reset ──
        wdtOn();

        // ── 来自 Main 的消息（CFG_UPDATE / TAP）──
        char webMsg[WEB_MSG_LEN];
        if (xQueueReceive(qMainToWeb, &webMsg, 0) == pdTRUE) {
            if (strcmp(webMsg, "CFG_UPDATE") == 0) {
                LOG("[WEB] 配置更新，重发设备信息\n");
                publishDevInfo("online");
            }
            else if (strcmp(webMsg, "TAP") == 0) {
                String terminal = Config.getString("terminal", "");
                LOG("[TAP] terminal=%s\n", terminal.c_str());
                publishCmdToTerminal(terminal, "TAP");
            }
            else if (strcmp(webMsg, "NEW_VOICE") == 0) {
                // 设备录了新语音 -> 发到 term/<账号>/cmd，触发离线推送（App 路径）
                String terminal = Config.getString("terminal", "");
                LOG("[VOICE] 新语音，terminal=%s\n", terminal.c_str());
                publishCmdToTerminal(terminal, "NEW_VOICE");
                // P4：若已绑定，把刚录的语音也推给伙伴即播（决策④ 都进）
                String peer = Config.getString("peerSn", "");
                if (!peer.isEmpty()) {
                    int slot = findLatestVoiceSlot();
                    if (slot >= 0) pushVoiceToPeer(slot, peer);
                }
            }
            else if (strcmp(webMsg, "POKE_PEER") == 0) {
                // 本机被拍一拍：若已互绑，"戳"伙伴让对方播放拍一拍视频（P2）
                String peer = Config.getString("peerSn", "");
                if (!peer.isEmpty()) {
                    char topic[64];
                    snprintf(topic, sizeof(topic), "dev/%s/cmd", topicId(peer).c_str());
                    StaticJsonDocument<160> d;
                    d["msg"]       = "POKE";
                    d["from"]      = g_sn_str;
                    d["timestamp"] = getTimestamp();
                    char payload[160];
                    serializeJson(d, payload, sizeof(payload));
                    client.publish(topic, payload);
                    LOG("[POKE] → %s : %s\n", topic, payload);
                }
            }
            else if (strcmp(webMsg, "SHAKE_PEER") == 0) {
                // 本机摇一摇：若已绑定，广播自身姿态一小段（伙伴舵机据此镜像 P3）
                String peer = Config.getString("peerSn", "");
                if (!peer.isEmpty()) {
                    g_shakeBroadcastUntil = millis() + POS_BROADCAST_MS;
                    LOG("[MIRROR] 摇一摇 → 广播自身姿态 %dms\n", POS_BROADCAST_MS);
                }
            }
        }

        // ── BLE 写入处理 ──
        Config.loop();

        // ── 传输期：升优先级到与 LCD 同级(1) + 收紧 loop；空闲：恢复(0) + 5ms ──
        bool xfering = (xferState != XFER_IDLE);
        if (xfering && !boosted) {
            vTaskPrioritySet(NULL, 1);
            boosted = true;
        } else if (!xfering && boosted) {
            vTaskPrioritySet(NULL, 0);
            boosted = false;
        }

        client.loop();

        // ── 传输接收看门狗：App 中途掉线（不再发分片/END/ABORT）时，避免设备永久
        //    卡在传输锁定态（xferLocked + forceActive 不休眠、屏幕锁待机、持续掉电）。
        //    g_lastMqttRxMs 每收到一片就刷新；停止刷新超过阈值即判 App 掉线。──
        if (xferState != XFER_IDLE &&
            (int32_t)(millis() - g_lastMqttRxMs) >= XFER_RX_TIMEOUT_MS) {
            LOG("[XFER] ✗ 接收超时 %ds 无分片，判 App 掉线 → 回滚解锁\n",
                XFER_RX_TIMEOUT_MS / 1000);
            rollbackAll();            // 依赖 xferState 判断是否回滚视频，必须先调
            xferState = XFER_IDLE;
            exitTransferMode();
        }

        // ── pos 串流态：App 心跳 3s 内活跃，或本机摇一摇后的广播窗口内（P3）──
        g_posStreaming = ((g_lastHeartbeatMs != 0) &&
                          (millis() - g_lastHeartbeatMs < 3000)) ||
                         ((int32_t)(millis() - g_shakeBroadcastUntil) < 0);

        // ── 取最新姿态并发布（QoS0）──
        if (g_posStreaming) {
            PosSample_t ps;
            if (xQueueReceive(qPosStream, &ps, 0) == pdTRUE) {
                char buf[48];
                snprintf(buf, sizeof(buf), "{\"ax\":%d,\"ay\":%d,\"az\":%d}",
                    ps.roll_cd, ps.pitch_cd, ps.yaw_cd);
                client.publish(posPubAddr, buf);   // PubSubClient = QoS0
            }
        }

        // ── 跟踪绑定变化：订阅/退订伙伴的 pos（P3 镜像）+ devInfo（在线→爱心图标）──
        {
            String peer = Config.getString("peerSn", "");
            if (peer != g_subPeer) {
                if (!g_subPeer.isEmpty()) {
                    char t[64];
                    snprintf(t, sizeof(t), "dev/%s/pos", topicId(g_subPeer).c_str()); client.unsubscribe(t);
                    snprintf(t, sizeof(t), "devInfo/%s", g_subPeer.c_str());     client.unsubscribe(t);
                }
                g_subPeer = peer;
                g_peerOnline = false;
                if (!peer.isEmpty()) {
                    snprintf(peerPosSubAddr, sizeof(peerPosSubAddr), "dev/%s/pos", topicId(peer).c_str());
                    client.subscribe(peerPosSubAddr, 0);
                    snprintf(peerInfoSubAddr, sizeof(peerInfoSubAddr), "devInfo/%s", peer.c_str());
                    client.subscribe(peerInfoSubAddr, 1);   // retained，一订阅即拿到当前在线态
                    LOG("[PAIR] 订阅伙伴 pos+devInfo: %s\n", peer.c_str());
                } else {
                    peerPosSubAddr[0] = 0;
                    peerInfoSubAddr[0] = 0;
                }
            }
            // 已绑 且 伙伴在线 → 点亮配对爱心（本机在线是隐含前提，此处正在跑 MQTT）
            g_pairLinked = (!peer.isEmpty()) && g_peerOnline;
        }

        // ── P3：镜像超时（伙伴停止摇动）→ 舵机归中位一次 ──
        if (g_mirrorActive &&
            (int32_t)(millis() - g_mirrorLastRxMs) >= MIRROR_TIMEOUT_MS) {
            g_mirrorActive    = false;
            g_mirrorLastAngle = -1;
            MessageToMain_t toMain;
            strcpy(toMain.strVal, "SERVO87");   // 归中位（87°）
            xQueueSend(qWebToMain, &toMain, 0);
            mainWake();
            LOG("[MIRROR] 伙伴停止，舵机归中位\n");
        }

        // ── 落盘完成后补发 VIDEO 结果 ──
        //   必须在 webTask（调 client.loop 的同一任务）里发，PubSubClient 非线程安全。
        //   g_persistPending==0 表示音视频都已写入 flash；据落盘成败回 OK/FAIL。
        if (g_videoResultPend && g_persistPending == 0) {
            g_videoResultPend = false;
            publishResultBytes("VIDEO", g_persistAllOk, g_videoResultBytes);
            LOG("[VIDEO] 落盘完成，已补发 VIDEO_%s\n", g_persistAllOk ? "OK" : "FAIL");
        }

        // ── 语音消息发送（App 拉取时逐条发 term/<sn>/voice，收 ack 后删）──
        handleVoiceSend();

        // ── WiFi 省电分级（传输满功率 / L1/L2 最省 / 仍按 DTIM 收下行）──
        applyWifiPowerSave();

        // ── 轮询节奏按状态调整：传输全速；WiFi 活跃保持低延迟；空闲慢轮询省电 ──
        // ── 轮询节奏：传输全速 / 串流低延迟 / L2 拉长喂 light-sleep / L0L1 低延迟 ──
        TickType_t pollTicks;
        if      (xfering)            pollTicks = 1;                          // 传输：全速
        else if (g_posStreaming)     pollTicks = pdMS_TO_TICKS(5);           // 串流：低延迟
        else if (g_pwrTier == 2)     pollTicks = pdMS_TO_TICKS(WEB_L2_POLL_MS); // L2：省电（推送仍≤1s）
        else                         pollTicks = pdMS_TO_TICKS(100);         // L0/L1：低延迟

        if (wdtSubscribed) esp_task_wdt_reset();   // 喂狗：稳态每轮一次（传输期 pollTicks=1，卡在 client.loop 内 >5s 才会点名）
        vTaskDelay(pollTicks);
    }
}