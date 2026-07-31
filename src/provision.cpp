#include "inc/provision.hpp"
#include "inc/configSys.hpp"
#include "inc/debug.hpp"

#include <WiFi.h>

// ── 配网触发标志（main 长按 IO0 置位；webTask 消费）──
volatile bool g_enterProvisioning = false;

// ── 配网阶段（lcd.cpp 读取，决定屏幕提示文字）──
//   0=未配网 / 1=等 BLE 连接（显示倒计时）/ 2=BLE 已连接等配置（显示"配置中"）
//   / 3=收到配置连 WiFi 中（整行显示"连接中"）
volatile int      g_provStage    = 0;
volatile uint32_t g_provDeadline = 0;   // 阶段1 等 BLE 的截止 millis（与自动关闭同一时刻）

extern volatile int g_xferAnim;         // lcd.cpp：0=无 1=转圈 2=打勾(带淡出)。配网成功时置2

// ── 参数 ──
#define PROV_WIFI_CONNECT_TIMEOUT_MS   15000     // 单次 WiFi 连接尝试超时
#define PROV_BLE_WAIT_LOG_INTERVAL     5000      // 等待日志间隔

// ★ 新增：自动关闭超时
#define PROV_NO_CONNECT_TIMEOUT_MS     120000    // 启动后 120s 无 BLE 连接 → 自动关闭
#define PROV_FAIL_TIMEOUT_MS           60000     // BLE 已连接后，配网失败持续 60s → 自动关闭

// ══════════════════════════════════════════════════════════════
//  是否需要配网（无有效 ssid）
// ══════════════════════════════════════════════════════════════
bool provisionNeeded()
{
    String ssid = Config.getString("ssid", "");
    return ssid.isEmpty();
}

// ══════════════════════════════════════════════════════════════
//  内部：用指定凭据尝试连接 WiFi
// ══════════════════════════════════════════════════════════════
static bool tryConnect(const String& ssid, const String& pass)
{
    LOG("[PROV] 尝试连接: %s\n", ssid.c_str());
    WiFi.disconnect(false);
    delay(100);
    WiFi.begin(ssid.c_str(), pass.c_str());

    uint32_t t0 = millis();
    while (millis() - t0 < PROV_WIFI_CONNECT_TIMEOUT_MS) {
        if (WiFi.status() == WL_CONNECTED) {
            LOG("[PROV] ✓ 连接成功: %s\n", WiFi.localIP().toString().c_str());
            return true;
        }
        if (Config.getDisconnectRequest()) {
            LOG("[PROV] 连接中 APP 请求断开\n");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    LOG("[PROV] ✗ 连接超时\n");
    return false;
}

// ══════════════════════════════════════════════════════════════
//  内部：关闭配网（退出配网模式 + 停 BLE 广播）
//   关闭后由 webTask 主循环的 connectWiFiFromConfig() 自动重连原配置档 WiFi，
//   故此处无需再写重连逻辑。
// ══════════════════════════════════════════════════════════════
static void provClose()
{
    g_provStage = 0;
    Config.setProvMode(false);
    Config.stopAdvertising();
}

// ══════════════════════════════════════════════════════════════
//  配网主流程（仅 BLE 凭据下发）
//
//  超时策略：
//    ① 等待 BLE 连接阶段：120s 内无连接 → 自动关闭
//    ② BLE 已连接、等待/尝试凭据阶段：失败持续 60s → 自动关闭
//    再次长按（g_enterProvisioning 被重新置位）→ 刷新当前阶段计时
//    自动关闭后 return，webTask 自动重连原配置档 WiFi
// ══════════════════════════════════════════════════════════════
void provisionButtonFlow()
{
    LOG("[PROV] ═══════════════════════════════════\n");
    LOG("[PROV]  进入配网模式（BLE）\n");
    LOG("[PROV] ═══════════════════════════════════\n");

    // 消费触发标志：此后再被置位即视为「再次长按」
    g_enterProvisioning = false;

    // ── ① 开启配网模式 + BLE 广播 ──
    Config.setProvMode(true);
    Config.startAdvertising();
    LOG("[PROV] BLE 广播已开启，等待 APP 连接...（≤%us 自动关闭）\n",
        PROV_NO_CONNECT_TIMEOUT_MS / 1000);

    // ── ② 等待 BLE 连接：120s 超时 / 可被再次长按刷新 ──
    uint32_t lastLog      = 0;
    uint32_t waitDeadline = millis() + PROV_NO_CONNECT_TIMEOUT_MS;
    g_provStage    = 1;                 // 阶段1：等 BLE 连接（屏幕显示倒计时）
    g_provDeadline = waitDeadline;      // 与自动关闭同一截止时刻

    while (!Config.isBLEConnected()) {

        // 再次长按 → 刷新 120s 等待计时
        if (g_enterProvisioning) {
            g_enterProvisioning = false;
            waitDeadline = millis() + PROV_NO_CONNECT_TIMEOUT_MS;
            g_provDeadline = waitDeadline;   // 屏幕倒计时同步刷新
            LOG("[PROV] 再次长按，刷新等待计时 → %us\n",
                PROV_NO_CONNECT_TIMEOUT_MS / 1000);
        }

        // APP 主动请求断开
        if (Config.getDisconnectRequest()) {
            LOG("[PROV] 等待 BLE 时收到断开，退出配网\n");
            provClose();
            return;
        }

        // 120s 内无 BLE 连接 → 自动关闭（webTask 随后重连原 WiFi）
        if ((int32_t)(millis() - waitDeadline) >= 0) {
            LOG("[PROV] %us 内无 BLE 连接，自动关闭配网，恢复原 WiFi\n",
                PROV_NO_CONNECT_TIMEOUT_MS / 1000);
            provClose();
            return;
        }

        if (millis() - lastLog > PROV_BLE_WAIT_LOG_INTERVAL) {
            lastLog = millis();
            LOG("[PROV] 等待 BLE 连接中...（剩余 %lds）\n",
                (long)((int32_t)(waitDeadline - millis()) / 1000));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    LOG("[PROV] ✓ BLE 已连接\n");
    g_provStage = 2;   // 阶段2：BLE 已连接，等 APP 下发配置（屏幕显示"配置中"）

    // BLE 连上后刷新 READ 特征为当前设备信息
    Config.refreshReadInfo();

    // ── ③ 断开当前 WiFi（保留旧凭据）──
    LOG("[PROV] 断开当前 WiFi，等待 APP 经 BLE 下发 ssid/password\n");
    WiFi.disconnect(false);
    delay(100);
    WiFi.mode(WIFI_STA);

    // ── ④ 等 BLE 凭据并尝试连接：失败持续 60s → 自动关闭 ──
    bool   provisioned = false;
    String newSsid, newPass;
    uint32_t failDeadline = millis() + PROV_FAIL_TIMEOUT_MS;   // 60s 失败窗口

    while (!provisioned) {

        // 等待 BLE 下发凭据（带 60s 超时 / 再次长按刷新 / 断开检查）
        while (!Config.getProvCreds(newSsid, newPass)) {

            // 再次长按 → 刷新 60s 配网计时
            if (g_enterProvisioning) {
                g_enterProvisioning = false;
                failDeadline = millis() + PROV_FAIL_TIMEOUT_MS;
                LOG("[PROV] 再次长按，刷新配网计时 → %us\n",
                    PROV_FAIL_TIMEOUT_MS / 1000);
            }

            // APP 请求断开
            if (Config.getDisconnectRequest()) {
                LOG("[PROV] APP 请求断开，退出配网\n");
                provClose();
                return;
            }

            // 配网失败持续 60s → 自动关闭（webTask 随后重连原 WiFi）
            if ((int32_t)(millis() - failDeadline) >= 0) {
                LOG("[PROV] 配网失败/无凭据持续 %us，自动关闭，恢复原 WiFi\n",
                    PROV_FAIL_TIMEOUT_MS / 1000);
                provClose();
                return;
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }

        LOG("[PROV] 收到 BLE 凭据: %s\n", newSsid.c_str());

        // 收到新凭据 → 给一次新的 60s 尝试窗口
        failDeadline = millis() + PROV_FAIL_TIMEOUT_MS;

        g_provStage = 3;   // 阶段3：收到配置，连 WiFi 中（整行显示"连接中"）
        // ★ 开始尝试前先推 connecting：一是让 App 明确"本轮已开始连接"，二是【覆盖掉
        //   上一轮失败残留在 READ 特征里的 {"status":"failed"}】。否则 App 重发凭据后，
        //   其 800ms 轮询会先读到旧的 failed 而误判本轮失败。App 只有先看到 connecting、
        //   再看到 failed 才认定失败，故这条是失败反馈能可靠工作的前提。
        Config.sendProvStatus("{\"status\":\"connecting\"}");
        if (tryConnect(newSsid, newPass)) {
            provisioned = true;
        } else {
            g_provStage = 2;   // 连接失败 → 回到等配置（"配置中"）
            Config.sendProvStatus("{\"status\":\"failed\"}");
            LOG("[PROV] 连接失败，继续等待新凭据...（剩余 %lds）\n",
                (long)((int32_t)(failDeadline - millis()) / 1000));
            if (Config.getDisconnectRequest()) {
                provClose();
                return;
            }
            // 不重置 failDeadline：失败后剩余时间继续倒数，直到收到新凭据或 60s 超时
        }
    }

    // ── ⑤ 连接成功：覆盖保存凭据 ──
    Config.setString("ssid",     newSsid.c_str());
    Config.setString("password", newPass.c_str());
    Config.save();
    LOG("[PROV] ✓ 新凭据已覆盖保存\n");

    // ── ⑥ 向 BLE READ 发送 {"status":"online"} ──
    Config.sendProvStatus("{\"status\":\"online\"}");
    LOG("[PROV] 已推送 online，等待 APP 确认断开...\n");

    // ── 配网成功：屏幕播打勾动画（带淡出）。阶段4 让 lcdTask 切到 drawXferAnim；
    //    等它播完（sweep+hold+fade≈1.6s）再继续等 disconnect，避免被过早 provClose 截断。──
    g_provStage = 4;
    g_xferAnim  = 2;
    {
        uint32_t ct0 = millis();
        while (g_xferAnim != 0 && millis() - ct0 < 2500) vTaskDelay(pdMS_TO_TICKS(50));
    }

    // ── ⑦ 等待 APP 回复 {"connection":"disconnect"} ──
    lastLog = 0;
    uint32_t waitStart = millis();
    const uint32_t DISCONNECT_WAIT_TIMEOUT_MS = 60000;
    while (!Config.getDisconnectRequest()) {
        if (!Config.isBLEConnected()) {
            LOG("[PROV] BLE 连接已断开，结束配网\n");
            break;
        }
        if (millis() - waitStart > DISCONNECT_WAIT_TIMEOUT_MS) {
            LOG("[PROV] 等待 disconnect 超时，强制结束配网\n");
            break;
        }
        if (millis() - lastLog > PROV_BLE_WAIT_LOG_INTERVAL) {
            lastLog = millis();
            LOG("[PROV] 等待 APP 发送 connection:disconnect...\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // ── ⑧ 关闭 BLE 和配网 ──
    LOG("[PROV] 关闭 BLE 与配网\n");
    provClose();

    LOG("[PROV] ═══════════════════════════════════\n");
    LOG("[PROV]  配网完成，恢复正常运行\n");
    LOG("[PROV] ═══════════════════════════════════\n");
}