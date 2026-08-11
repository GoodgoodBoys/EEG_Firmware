#include <stdio.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <time.h>
#include <stdlib.h>             // setenv（设置 TZ 时区）
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_random.h"
#include "esp_netif_sntp.h"     // IDF5 线程安全 SNTP（取代 configTime，避免 lwIP core lock 断言）
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
#include "inc/health.hpp"       // 运行期阶段打点（TWDT 超时时会被冻成快照）
#include "esp_task_wdt.h"       // 任务级看门狗：稳态纳入监控，配网/连WiFi 等长阻塞前退订
#include "inc/ota.hpp"          // OTA：官方版本 Topic 触发 + 版本上报

#define WIFI_WARTTING_TIME 30

// ==================== 参数配置 ====================
#define MQTT_BUFFER_SIZE          8192

#define AUDIO_FILE_PATH           "/output1.mp3"

// ── NTP 时间同步 ──
#define NTP_SERVER1               "pool.ntp.org"
#define NTP_SERVER2               "ntp.aliyun.com"
// ⚠ 这两个 offset 已不再直接生效（不再走 configTime）：实际时区由 startNtpSync() 里的
//   POSIX TZ 字符串 "CST-8" 设置。要改时区请改那里，改这两个宏没有作用。
#define GMT_OFFSET_SEC            (8 * 3600)
#define DAYLIGHT_OFFSET_SEC       0

// 数据通道上 JSON 控制消息的最大长度（用于区分二进制数据与控制帧）
#define CTRL_JSON_MAX_LEN         240
// ==================================================

#define WEB_L2_POLL_MS  500   // L2 下 MQTT 轮询周期 ≈ 推送唤醒延迟上限。
                              // ↑省电 ↓延迟；保持 ≤700 以确保推送在 1s 内被处理。

// 单次 client.loop() 判为"慢调用"的阈值：正常收包 <100ms，卡在 readByte 的
// setSocketTimeout(3) 上时会到数百 ms~3s。传输期 webTask 被提到 P1（高于 core0 的
// IDLE0，而 TWDT 单独监视 IDLE0/8s），慢调用连发会把 IDLE0 饿死 → panic 点名 IDLE0。
// 检测到慢调用即让出一个整 tick 给调度器复位 IDLE0 的狗（见批处理循环）。
#define WEB_LOOP_SLOW_MS  300

#define XFER_RX_TIMEOUT_MS  10000  // 传输中超过这么久没再收到分片 → 判 App 掉线/崩溃，
                                   // 自动回滚解锁，避免设备永久卡在锁定态耗电。
                                   // 分片走 QoS1（broker 会重传），10s 足够覆盖正常网络抖动；
                                   // 从 30s 收紧，让 App 崩溃后设备更快解锁自愈、少耗电。

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
extern QueueHandle_t qServoCmd;   // 舵机专用深度1覆盖队列：连发 SERVO 只留最新角度，不占 qWebToMain

extern void mainWake();         // 唤醒主循环（V1_1.cpp）

extern volatile int g_pwrTier;   // 0/1/2 功耗档位（V1_1.ino 定义）
extern volatile int g_xferAnim;  // LCD 传输动画（lcd.cpp）：0=无 / 1=转圈 / 2=打勾
extern volatile uint32_t g_bootId;   // 本次开机随机数（V1_1.cpp 定义）：随 devInfo 上报，
                                     // App 据此秒级识别"设备重启过"→ 中止在途传输，不等超时
extern volatile int g_batPercent;    // 电量百分比（V1_1.cpp 定义）：0~100，-1=未知；随 devInfo 上报给 App 显示电池
extern volatile bool g_batCharging;  // 是否充电中（V1_1.cpp 定义）：随 devInfo 上报，App 充电时显示充电图标

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

// ★ 传输会话 ID（App 每次发送生成，随 NEW_* 带来）：设备只认与本次 NEW 一致的 END，
//   拒收过期/错乱会话（如一端重启后旧会话的残留 END）。0 = App 未带（旧版本，跳过校验）。
static uint32_t g_curXferId = 0;

// 从数据主题控制帧 payload 里取 xferId，与本次会话比对。
//   App 未带（=0）或本机会话未记录（=0）时返回 true（向后兼容，不误杀）。
static bool xferIdMatches(const byte* payload, unsigned int length)
{
    if (g_curXferId == 0) return true;                 // 本会话未带 id → 不校验
    StaticJsonDocument<256> d;
    if (deserializeJson(d, payload, length)) return true;  // 解析失败不误杀，交由头尾校验兜底
    uint32_t id = d["xferId"] | 0;
    if (id == 0) return true;                          // App 未带 id → 兼容旧版
    return id == g_curXferId;
}

// 从控制帧 payload 里取出 xferId（取不到/解析失败返回 0）。用于重复 END 的会话匹配。
static uint32_t xferIdOf(const byte* payload, unsigned int length)
{
    StaticJsonDocument<256> d;
    if (deserializeJson(d, payload, length)) return 0;
    return d["xferId"] | 0u;
}

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
// NEW_AUDIO/NEW_VIDEO 声明的应收字节数（0=旧 App 未带，跳过校验保持兼容）。
// ★ 用途：QoS1 是"至少一次"，链路抖动时 broker 会重发同一片、PubSubClient 不去重，
//   appendRecv 纯追加就会多出字节 → MP3/视频被插入垃圾（同一文件每次听感不同）。
//   END 时比对 g_recvLen==g_declaredSize，多了(重复)就判失败 → App 自动重传，杜绝静默污染。
static uint32_t      g_declaredSize = 0;

// 暂存音频接收信息，等 END_VIDEO 与视频一并持久化
static const uint8_t* g_audioSrc = nullptr;
static size_t         g_audioLen = 0;

// 后台持久化（PSRAM → flash）——「延后到 L2 空闲窗口才写」模型。
//   至多 2 条待落盘（audio + video），用固定槽而非队列：被唤醒打断后保留 off、下次窗口续写；
//   收到同一 path 的新内容则 off 归零、整条替换（见 enqueuePersist / persistTask）。
typedef struct {
    volatile bool  active;   // 有待落盘内容
    const uint8_t* src;      // PSRAM 源（落盘期间不得被覆盖）
    size_t         len;      // 总字节
    size_t         off;      // 已写字节（续写起点；被唤醒打断后保留）
    char           path[64];
} PersistSlot_t;
#define PERSIST_SLOTS 2
static PersistSlot_t     s_pslot[PERSIST_SLOTS];
static portMUX_TYPE      s_pslotMux   = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t      xPersistTask = nullptr;

// ★ 唤醒立即打断落盘：由 markActivity()（V1_1.cpp，覆盖按键/App/摇拍）与 imuWoMISR()
//   （imu.cpp，IRAM，IMU 运动中断最早入口）置位。persistTask 每个 flash 操作前查它，
//   置位即停手、保留 off 待续写。开写窗口打开时（已 65s 无活动）由 powerManagerLoop 清零。
volatile bool g_persistAbortWrite = false;

// 有没有待落盘内容（供 powerManagerLoop 判要不要开窗、webTask 判后台落盘是否完成）。
bool persistIsPending() {
    for (int i = 0; i < PERSIST_SLOTS; i++) if (s_pslot[i].active) return true;
    return false;
}
static volatile bool     g_persistAllOk   = true; // 本轮所有落盘是否都成功
static volatile bool     g_videoResultPend = false; // VIDEO 结果待落盘完成后补发
static size_t            g_videoResultBytes = 0;  // 补发 VIDEO 结果时用的字节数

// ★ 传输存活时钟：最近一次收到【本次音视频传输自身】的帧的时刻。
//   唯一读者是 webTask 里的 RX 看门狗（判"App 中途掉线"）。
//   ★★ 只能被【属于本次传输的东西】刷新 —— 分片、END_*、NEW_* 的初始化。
//   绝不能被"任何入站消息"刷新：命令(SERVO/PULL_VOICE/PV_ACK)、伙伴语音帧都与本次传输无关，
//   拿它们喂狗会让传输卡死后永远不超时 → xferState 永久非 IDLE → 所有远程指令被静默屏蔽。
volatile uint32_t g_lastMqttRxMs = 0;
static inline void xferRxTouch() { g_lastMqttRxMs = millis(); }

// 传输态绝对上限：不看任何时钟，非 IDLE 超过这么久就无条件复位。
// 最后一道保险 —— 万一将来又有新路径把存活时钟喂活，也不会让设备永久失去远程控制。
#define XFER_STATE_MAX_MS  180000
static uint32_t g_xferStateSinceMs = 0;   // 进入非 IDLE 态的时刻（0 = 当前是 IDLE）

// ══════════════════════════════════════════════════════════════
//  网络自检与恢复
//    背景：设备没有 UI 报错、没有可用串口（USB CDC 一进 light-sleep 就断）。
//    而 MQTT 连不上时的现有行为是【无限 2 秒重试、无告警、无升级】——
//    表现就是"设备还亮着、本地交互正常，但所有远程指令石沉大海"，只能拔电恢复。
//    这里做三件事：① 把网络状态记进黑匣子（可诊断）② 幂等重订阅（低风险自愈）
//    ③ 连续失败到阈值时逐级升级（重连 WiFi → 重启）。
// ══════════════════════════════════════════════════════════════
volatile uint32_t g_lastAnyRxMs   = 0;    // 最近收到【任何】入站消息的时刻（诊断用，与传输时钟分开）
volatile uint32_t g_mqttFailCnt   = 0;    // 累计 MQTT 连接失败次数
volatile int32_t  g_mqttLastErr   = 0;    // 最后一次 client.state()
volatile uint32_t g_mqttFailBlk   = 0;    // 失败当时的内部 RAM 最大连续块（验证"内存不足"假设）
volatile uint32_t g_netRecoverCnt = 0;    // WiFi 级恢复次数
volatile uint32_t g_netRebootCnt  = 0;    // 因网络无法恢复而重启的次数
static   uint32_t s_mqttFailRun   = 0;    // 【连续】失败次数（连上就清零）

// 升级阈值（每次失败间隔 2s）
#define NET_RECOVER_FAILS   5     // ≈10s 连不上 → 重连 WiFi（清 lwIP 状态）
#define NET_REBOOT_FAILS    30    // ≈60s 连不上 → 重启（★ 置 0 可关闭这一级）
#define NET_REBOOT_MIN_GAP  3600000UL   // 两次重启至少间隔 1 小时，防重启循环

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

// ── 配对爱心图标：已互绑 且 伙伴在线 且 伙伴也回指本机（双向确认）──
static bool   g_peerOnline = false;          // 伙伴 devInfo.status==online
// ★ 双向确认：伙伴 devInfo.peerSn == 本机 SN。只在伙伴【在线】时刷新（离线 retained 可能是
//   连接时冻结、带旧 peerSn 的遗嘱，采信会误判）。修"对端出厂/换绑后本机仍单方显示已互绑"。
static bool   g_peerAcksMe = false;
volatile bool g_pairLinked = false;          // lcd.cpp 读取：已绑 且 伙伴在线 且 伙伴回指本机
volatile bool g_wifiOnline  = false;         // lcd.cpp 读取：WiFi 是否已连接（WiFi 状态图标）
volatile bool g_needWifiHint = false;        // lcd.cpp 读取：有配置但连不上 → 显示"长按2秒配置"引导画面
volatile bool g_dndActive    = false;        // V1_1.cpp 读取：当前处于勿扰时段（webTask 每 2s 刷新）
                                             //   → 勿扰时后台下载不亮屏（powerManagerLoop 据此不 forceActive）

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

// ══════════════════════════════════════════════════════════════
//  NTP 时间同步启动（线程安全，取代 configTime）
//    ★ 不能用 configTime()：它会在【非 TCPIP 线程】(webTask) 直接调 lwIP raw API
//      (udp_new_ip_type 建 UDP pcb)，触发断言
//      "assert failed: udp_new_ip_type ... Required to lock TCPIP core functionality!"
//      → panic 重启（表现为开机偶发崩一次）。
//      esp_netif_sntp_init() 是 IDF5 官方封装，内部自行处理 TCPIP 加锁，任意线程可调。
//    ★ 时区：configTime 原本用 GMT_OFFSET_SEC 顺带设时区；换 API 后必须自己设 TZ，
//      否则 getLocalTime() 返回 UTC → getTimestamp / 勿扰时段(isInDndPeriod) 全差 8 小时。
//      POSIX TZ 记法符号与 UTC 偏移相反：东八区(GMT+8) 写作 "CST-8"（无夏令时）。
//    ★ 幂等：esp_netif_sntp_init() 只能初始化一次，重复调用会报错，故用静态标志护住。
// ══════════════════════════════════════════════════════════════
static void startNtpSync()
{
    static bool s_ntpInited = false;
    if (s_ntpInited) return;

    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(
        2, ESP_SNTP_SERVER_LIST(NTP_SERVER1, NTP_SERVER2));
    esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err != ESP_OK) {
        LOG("[WEB] ⚠ SNTP 初始化失败: %s（时间不同步，勿扰时段将不生效）\n",
            esp_err_to_name(err));
        return;
    }
    s_ntpInited = true;

    setenv("TZ", "CST-8", 1);   // = GMT+8（POSIX 记法符号相反）；对应原 DAYLIGHT_OFFSET_SEC=0 无夏令时
    tzset();
    LOG("[WEB] NTP 同步已启动（esp_netif_sntp，线程安全）\n");
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
    doc["bootId"]    = (uint32_t)g_bootId;                // 本次开机随机数：App 据此识别设备重启（volatile 显式转型避免 ArduinoJson 模板推导问题）
    doc["bat"]       = (int)g_batPercent;                // 电量百分比 0~100（-1=未知）：App 据此在好友栏显示电池图标
    doc["chg"]       = (bool)g_batCharging;              // 是否充电中：App 充电时显示充电图标（充电期电压跳动，不显示百分比）
    doc["fw"]        = otaCurrentVersion();              // 当前固件版本（整数）：App 据此判断是否有新版可升级
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
//  后台持久化任务（core 1）：把收到的 PSRAM 内容写回 flash
//
//  策略（★重写）：【只在 L2 空闲窗口写，任何唤醒立即打断】。
//    · L0/L1（有屏、可能在播/交互）：绝不写 —— 写 flash 冻结 core0 cache 会卡渲染/动作。
//    · 进 L2 满 5s（idle≥65s，屏灭、IMU 进 WoM、播放已停）：powerManagerLoop 开窗
//      (persistForceAcquire) → 本任务全速写。
//    · 任何唤醒（按键/App/摇拍；IMU 经 imuWoMISR 最早置位）→ g_persistAbortWrite=1
//      → 本任务在下一个 flash 操作前停手、保留 off，下次窗口【续写】。
//    · 同一 path 收到新内容 → enqueuePersist 把 off 归零 → 下次【整条替换】。
//  内容 commit 后已在 PSRAM 可播，落盘只为掉电存活：正常关机 persistFlushBlocking 兜底，
//  异常断电则丢失（重发即可）—— 这是"绝不卡顿"换来的取舍。
// ══════════════════════════════════════════════════════════════

// 「落盘窗口」引用计数：>0 = 现在可以全速落盘。powerManagerLoop 在 idle≥65s 时持有，
//  persistFlushBlocking 关机排空时持有。用计数而非布尔，避免一方 release 误关另一方的窗口。
static portMUX_TYPE s_forceMux  = portMUX_INITIALIZER_UNLOCKED;
static volatile int s_forceRefs = 0;

// 中止标志：新传输要复用 PSRAM 前置位，令 persistTask 立刻【丢弃】当前写（不 rename、不续写）。
// ★ 与 g_persistAbortWrite 区别：那个是"唤醒→暂停+保留 off 续写"；这个是"源 PSRAM 要被覆盖
//   →作废，绝不能再写"。两者都在每个 flash 操作前查。
static volatile bool s_persistAbort = false;

// persistTask 是否【真正在写 flash】。lcd.cpp 据此降帧给 core0 让路（L2 息屏时无实际影响）；
// cancelPersistForReuse 据此等它收尾再复用 PSRAM。
volatile bool g_persistWriting = false;

void persistForceAcquire() { portENTER_CRITICAL(&s_forceMux); int v = s_forceRefs; s_forceRefs = v + 1;              portEXIT_CRITICAL(&s_forceMux); }
void persistForceRelease() { portENTER_CRITICAL(&s_forceMux); int v = s_forceRefs; if (v > 0) s_forceRefs = v - 1;   portEXIT_CRITICAL(&s_forceMux); }

static inline bool persistForced() { return s_forceRefs > 0; }

// 写一个槽：从 off 续写到 len。force=true（关机排空）时忽略唤醒打断、一次写到完。
//   返回 true=已写满并 rename；false=被打断/中止/失败/无内容（off 已回写进槽，供续写）。
static bool persistWriteSlot(int i, bool force)
{
    // 取槽快照（临界区内读一致）
    portENTER_CRITICAL(&s_pslotMux);
    bool           active = s_pslot[i].active;
    const uint8_t* src    = s_pslot[i].src;
    size_t         len    = s_pslot[i].len;
    size_t         off    = s_pslot[i].off;
    char           path[64];
    memcpy(path, s_pslot[i].path, sizeof(path));
    portEXIT_CRITICAL(&s_pslotMux);
    if (!active || !src || len == 0 || off >= len) return false;

    String tmp = String(path) + ".tmp";
    // off>0 续写(APPEND)；否则从头(WRITE，截断)。
    File f = (off > 0) ? LittleFS.open(tmp.c_str(), FILE_APPEND)
                       : LittleFS.open(tmp.c_str(), FILE_WRITE);
    if (!f) { LOG("[PST] ✗ 打开失败 %s\n", tmp.c_str()); g_persistAllOk = false; return false; }
    // 续写前校验 .tmp 实际长度与 off 一致；异常则从头重写
    if (off > 0 && f.size() != off) {
        LOG("[PST] ⚠ %s .tmp 长%u≠off%u，改从头写\n", path, (unsigned)f.size(), (unsigned)off);
        f.close();
        f = LittleFS.open(tmp.c_str(), FILE_WRITE);
        if (!f) { g_persistAllOk = false; return false; }
        off = 0;
    }

    uint32_t t0 = millis();
    bool ok = true, interrupted = false, discarded = false;
    LOG("[PST] ▶ %s落盘 %s（%uKB/%uKB）\n", off ? "续写" : "开始", path,
        (unsigned)(off / 1024), (unsigned)(len / 1024));
    g_persistWriting = true;
    // ★ 头号嫌疑打点：这个 while 里的每次 f.write/f.flush 都在写 flash，期间 core0 的
    //   cache 被 IPC 冻结 —— 若 TWDT 点名 IDLE0 而此刻 wdtTs[HT_PERSIST]==HS_P_WRITE，
    //   那就是"落盘冻住 core0"这条老路径又犯了，不必再猜。
    healthSetTaskStage(HT_PERSIST, HS_P_WRITE);
    while (off < len) {
        // ★ 每个 flash 操作前查两个中止标志：
        if (s_persistAbort)                { discarded   = true; break; }  // 源要被覆盖 → 丢弃
        if (!force && g_persistAbortWrite) { interrupted = true; break; }  // 唤醒 → 暂停续写
        size_t n = len - off; if (n > 4096) n = 4096;
        size_t w = f.write(src + off, n);
        if (w != n) { ok = false; break; }
        off += w;
        f.flush();                       // 逼 LittleFS 小步落盘：单次 flash 操作压到 ~1 sector
        vTaskDelay(pdMS_TO_TICKS(force ? 5 : 8));   // 让 core0 恢复
        esp_task_wdt_reset();            // 对未订阅任务是空操作；关机排空路径若被 TWDT 监视则需要
    }
    f.flush(); f.close();
    g_persistWriting = false;
    healthSetTaskStage(HT_PERSIST, HS_P_RENAME);   // 之后是 remove/rename，仍在碰 flash

    // 回写 off（打断/失败都要存，供续写）—— 仅当槽仍是同一内容（未被 cancel/replace 抢走）
    portENTER_CRITICAL(&s_pslotMux);
    bool stillOurs = (s_pslot[i].active && s_pslot[i].src == src);
    if (stillOurs) s_pslot[i].off = off;
    portEXIT_CRITICAL(&s_pslotMux);

    if (discarded) {
        // ★ 不在这里删 .tmp：本函数已 f.close() 且置 g_persistWriting=false，cancelPersistForReuse
        //   正是在等这个标志变假后统一删所有 .tmp（见其注释）。若这里也删，两核会并发 open/remove
        //   同一文件。槽的 active 也交由 cancel 清。
        LOG("[PST] ✗ %s 落盘中止（源 PSRAM 被复用），丢弃\n", path);
        return false;
    }
    if (interrupted) {
        LOG("[PST] ⏸ %s 落盘被唤醒打断（已写 %uKB/%uKB，待续写）\n",
            path, (unsigned)(off / 1024), (unsigned)(len / 1024));
        return false;
    }
    if (!ok) {
        LittleFS.remove(tmp.c_str());
        g_persistAllOk = false;
        if (stillOurs) { portENTER_CRITICAL(&s_pslotMux); s_pslot[i].active = false; portEXIT_CRITICAL(&s_pslotMux); }
        LOG("[PST] ✗ %s 持久化失败（PSRAM 仍可播，下次开机用旧文件）\n", path);
        return false;
    }
    // ★ 写满，但若期间槽内容已被替换(!stillOurs：enqueuePersist 用新 src 抢了本槽)，
    //   本次写的是【旧内容】→ 弃用 .tmp、不 rename（新内容会在下个窗口从 off=0 重写）。
    if (!stillOurs) {
        LittleFS.remove(tmp.c_str());
        LOG("[PST] ⚠ %s 落盘完成但内容已被替换，弃用本次\n", path);
        return false;
    }
    // 正常写满 → rename 成正式文件
    if (LittleFS.exists(path)) LittleFS.remove(path);
    if (!LittleFS.rename(tmp.c_str(), path)) LittleFS.remove(tmp.c_str());
    portENTER_CRITICAL(&s_pslotMux); s_pslot[i].active = false; s_pslot[i].off = 0; portEXIT_CRITICAL(&s_pslotMux);
    LOG("[PST] ✓ %s 持久化 %uKB / %lums\n", path, (unsigned)(len / 1024), (unsigned long)(millis() - t0));
    return true;
}

static void persistTask(void* p)
{
    LOG("[PST] 持久化任务启动 (core %d)\n", xPortGetCoreID());
    while (true) {
        // 开写门：窗口开(powerManagerLoop 在 idle≥65s 持 persistForced) + 有活槽 + 未被唤醒打断。
        // 未满足则轻睡等待（本任务在 core1，未纳入 TWDT，无需喂狗）。
        if (!persistForced() || !persistIsPending() || g_persistAbortWrite || s_persistAbort) {
            healthSetTaskStage(HT_PERSIST, HS_P_IDLE);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        // 窗口已开（此刻已 65s 无活动，powerManagerLoop 开窗时已清 g_persistAbortWrite）。
        // 逐个活槽写；窗口关 / 被唤醒打断 / 被取消 即停。
        for (int i = 0; i < PERSIST_SLOTS; i++) {
            if (!persistForced() || g_persistAbortWrite || s_persistAbort) break;
            persistWriteSlot(i, false);
        }
    }
}

// 入队：按 path 落槽（新内容 off 归零＝整条替换）。src 指向 PSRAM，落盘完成前不得覆盖。
static void enqueuePersist(const uint8_t* src, size_t len, const char* path)
{
    portENTER_CRITICAL(&s_pslotMux);
    int idx = -1;
    for (int i = 0; i < PERSIST_SLOTS; i++)   // 同 path 复用同槽（新内容替换）
        if (s_pslot[i].active &&
            strncmp(s_pslot[i].path, path, sizeof(s_pslot[i].path)) == 0) { idx = i; break; }
    if (idx < 0) for (int i = 0; i < PERSIST_SLOTS; i++) if (!s_pslot[i].active) { idx = i; break; }
    if (idx >= 0) {
        s_pslot[idx].active = true;
        s_pslot[idx].src    = src;
        s_pslot[idx].len    = len;
        s_pslot[idx].off    = 0;   // ★ 新内容 → 从头写（整条替换）
        strncpy(s_pslot[idx].path, path, sizeof(s_pslot[idx].path) - 1);
        s_pslot[idx].path[sizeof(s_pslot[idx].path) - 1] = 0;
    }
    portEXIT_CRITICAL(&s_pslotMux);
    if (idx < 0) { g_persistAllOk = false; LOG("[PST] ⚠ 落盘槽已满，跳过 %s\n", path); }
}

// 复用 PSRAM 前【取消】待落盘（源即将被新传输覆盖，旧内容作废）。
//   置 s_persistAbort 令 persistTask 立即停手（不 rename、不续写），等它收尾(~一个 4KB 写)，
//   再清空所有槽 + 删残留 .tmp。★ 关键安全不变量：接收（覆盖 PSRAM）在本函数返回后才开始，
//   而本函数等到 g_persistWriting=false 才返回 —— persistTask 绝不会读到被覆盖的 PSRAM。
static void cancelPersistForReuse()
{
    if (!persistIsPending() && !g_persistWriting) return;
    healthSetTaskStage(HT_WEB, HS_W_WAITPERS);   // 忙等 persistTask 收尾（有超时兜底）
    uint32_t t0 = millis();
    LOG("[PST] 取消待落盘，立即复用 PSRAM\n");
    s_persistAbort = true;                 // persistWriteSlot 下一个 flash 前丢弃当前写
    while (g_persistWriting && millis() - t0 < 2000) {   // 等它收尾(~一个 4KB)，2s 兜底防异常
        vTaskDelay(pdMS_TO_TICKS(10));
        esp_task_wdt_reset();
    }
    // 快照要删的 .tmp 路径，再清槽（LittleFS 调用不放临界区内）
    char paths[PERSIST_SLOTS][64]; bool had[PERSIST_SLOTS];
    portENTER_CRITICAL(&s_pslotMux);
    for (int i = 0; i < PERSIST_SLOTS; i++) {
        had[i] = s_pslot[i].active;
        memcpy(paths[i], s_pslot[i].path, sizeof(paths[i]));
        s_pslot[i].active = false;
        s_pslot[i].off    = 0;
    }
    portEXIT_CRITICAL(&s_pslotMux);
    for (int i = 0; i < PERSIST_SLOTS; i++) {
        if (!had[i]) continue;
        String t = String(paths[i]) + ".tmp";
        if (LittleFS.exists(t.c_str())) LittleFS.remove(t.c_str());
    }
    s_persistAbort = false;
    LOG("[PST] 待落盘已取消，用时 %lums\n", (unsigned long)(millis() - t0));
}

// 阻塞排空（关机前调用）：开窗全速写完，避免长按关机丢掉刚收到的视频。
//   force=true：忽略唤醒打断（关机不会再有交互，数据安全优先，一次写到完）。
void persistFlushBlocking(uint32_t timeoutMs)
{
    if (!persistIsPending()) return;
    healthSetTaskStage(HT_WEB, HS_W_WAITPERS);
    LOG("[PST] 关机前排空落盘...\n");
    persistForceAcquire();       // 持窗（保 240MHz + 禁 light-sleep）
    uint32_t t0 = millis();
    for (int i = 0; i < PERSIST_SLOTS; i++) {
        if ((millis() - t0) >= timeoutMs) break;
        persistWriteSlot(i, true);   // force：忽略唤醒打断，一次写到完
        esp_task_wdt_reset();
    }
    persistForceRelease();
    LOG("[PST] 关机前排空%s\n", persistIsPending() ? "超时/失败（掉电会退回旧内容）" : "完成");
}

// ══════════════════════════════════════════════════════════════
//  接收缓冲：直写 PSRAM
// ══════════════════════════════════════════════════════════════

// ── 传输就绪/进度上报（→ App 的 term/<sn>/response）────────────────────
//   XFER_READY：接收态已建好(含排空落盘)。App 收到才发分片 → 不把片发进"设备还没准备好"的空里，
//     根治"设备进下载模式却收不到、两边干等超时"（尤其 waitPersistIdle 排空阻塞回调那段）。
//   XFER_RECVD：报"已收字节"。两条触发路径：① 每收 ~64KB（快链路进度细腻）；
//     ② webTask 主循环每 ~1.5s 补一发（慢链路/接收尾部/等提交时也持续心跳，App 不误判超时）。
//     大文件慢传、卡在提交(建帧索引)那段，全靠这个时间心跳撑住 App 的空闲计时。
#define XFER_RECVD_STEP    32768u         // 按字节上报间隔（32KB：报得更密 → App 流控 _recvGot 追得更紧、少卡顿）
#define XFER_HEARTBEAT_MS  1500u          // 按时间补发心跳间隔（覆盖慢接收/尾部/提交前）
#define XFER_RECVD_LOG_STEP 524288u       // 串口进度日志间隔(~512KB，避免刷屏)
static uint32_t g_lastRecvdBytes = 0;    // 上次 RECVD 上报时的 g_recvLen
static uint32_t g_lastRecvdMs    = 0;    // 上次 RECVD 上报时刻（时间心跳据此节流）
static uint32_t g_lastRecvdLogBytes = 0; // 上次打串口进度日志时的 g_recvLen

// ── 结果回执可靠补发（防 QoS0 尾丢包）──
//   VIDEO_OK/AUDIO_OK 走 QoS0，大文件尾部易在同一拥塞窗口整批丢。仅"提交后连发2次"不够——
//   两次挤在一起可能一起丢，App 就只能干等 20s 空闲超时后误判"设备无响应"（长视频高发）。
//   这里把最近一次结果缓存，随后几秒内【间隔补发】几次，直到 App 收到（App 对同一结果天然
//   去重；新一轮传输按 xferId 过滤旧回执，故盲补发安全）。新一段接收开始时清零，不补发陈旧结果。
#define RESULT_RESEND_TIMES    6      // 补发次数
#define RESULT_RESEND_STEP_MS  600u   // 补发间隔（6×600ms ≈ 覆盖 3.6s，足够穿过瞬时丢包）
static char     g_lastResultPayload[128] = {0};
static uint32_t g_lastResultXferId = 0;   // 缓存结果所属会话 xferId：App 重发 END 时据此幂等重发结果
static uint8_t  g_resultResendLeft = 0;
static uint32_t g_resultResendAtMs = 0;

// ── online devInfo 连接后补发（防 QoS0 首发丢失把 retained 钉在遗嘱 offline）──
//   publishDevInfo 走 QoS0（PubSubClient 只能发 QoS0）。连接时注册的遗嘱(willRetain)
//   payload 是用【当前配置】构建的 offline devInfo —— 它带着正确的 matchCode。若连接后
//   那一发 online（QoS0）丢包、或设备曾非正常断连触发过遗嘱，broker 上的 retained 就停在
//   「offline + 正确 matchCode」上。App 匹配时据此读到「matchCode 正确但设备关机」的错觉
//   （即便设备其实在线）。连接成功后在随后几秒内间隔补发几次 online，把 retained 可靠刷成在线。
#define ONLINE_REPUB_TIMES    3      // 首发之外的补发次数
#define ONLINE_REPUB_STEP_MS  800u   // 补发间隔（3×800ms ≈ 2.4s，穿过瞬时丢包）
static uint8_t  s_onlineRepubLeft = 0;
static uint32_t s_onlineRepubAtMs = 0;

static void sendXferReady()
{
    StaticJsonDocument<64> d;
    d["msg"] = "XFER_READY"; d["xferId"] = g_curXferId;
    char b[64]; serializeJson(d, b, sizeof(b));
    client.publish(responseTopic.c_str(), b);
    LOG("[XFER] → 回 READY（接收态就绪，xferId=%u）\n", (unsigned)g_curXferId);
}
static void sendXferRecvd()
{
    StaticJsonDocument<96> d;
    d["msg"]    = "XFER_RECVD";
    d["xferId"] = g_curXferId;
    d["got"]    = (uint32_t)g_recvLen;
    d["total"]  = g_declaredSize;
    char b[96]; serializeJson(d, b, sizeof(b));
    client.publish(responseTopic.c_str(), b);
    // 节流打印接收进度 + 平均速率（看设备实际收多快、是否停滞）
    if ((uint32_t)g_recvLen - g_lastRecvdLogBytes >= XFER_RECVD_LOG_STEP ||
        (g_declaredSize > 0 && g_recvLen >= g_declaredSize)) {
        g_lastRecvdLogBytes = g_recvLen;
        uint32_t el = millis() - g_xferStartMs;
        uint32_t kbps = el > 0 ? (uint32_t)((uint64_t)g_recvLen * 1000 / 1024 / el) : 0;
        LOG("[XFER] ◀ 已收 %uKB/%uKB (%u%%) %lums %uKB/s\n",
            (unsigned)(g_recvLen / 1024), (unsigned)(g_declaredSize / 1024),
            g_declaredSize ? (unsigned)((uint64_t)g_recvLen * 100 / g_declaredSize) : 0,
            (unsigned long)el, (unsigned)kbps);
    }
    g_lastRecvdBytes = g_recvLen;        // 两条路径共用：任一发过就重置字节/时间节流
    g_lastRecvdMs    = millis();
}

static void appendRecv(const uint8_t* payload, size_t length)
{
    healthSetTaskStage(HT_WEB, HS_W_RXCHUNK);
    if (!g_recvBase || receiveError) return;
    if (g_recvLen + length > g_recvMax) {
        LOG("[WEB] ✗ 接收溢出: %u+%u > %u\n",
            (unsigned)g_recvLen, (unsigned)length, (unsigned)g_recvMax);
        receiveError = true;
        return;
    }
    memcpy(g_recvBase + g_recvLen, payload, length);
    g_recvLen += length;
    // 每 ~64KB 上报一次真实进度（节流；从 MQTT 回调里发 publish 安全，同 publishResult）。
    // 慢链路下不足 64KB 的间隙由主循环的时间心跳补发（见 webTask）。
    if ((uint32_t)g_recvLen - g_lastRecvdBytes >= XFER_RECVD_STEP) {
        sendXferRecvd();   // 内部更新 g_lastRecvdBytes/Ms
    }
}

static void resetRecvPerf(const char* what)
{
    receiveError = false;
    g_resultResendLeft = 0;   // 新一段接收开始 → 停掉上一段结果的补发（避免把陈旧回执发进新会话）
    g_recvLen = 0;
    g_lastRecvdBytes = 0;      // 进度上报重新起算
    g_lastRecvdLogBytes = 0;
    g_lastRecvdMs    = millis();
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

// ★ 发送【状态类】消息到主循环：不能像播放指令那样丢了就算了。
//   XFER_LOCK/UNLOCK 是成对的状态迁移，丢一条就永久失配（丢 UNLOCK = 设备锁死到重启）。
//   给一个短超时让出 CPU 等主循环腾位；真发不进去就明确告警（配合下面的超时自愈兜底）。
static void sendMainStateMsg(MessageToMain_t& m)
{
    if (xQueueSend(qWebToMain, &m, pdMS_TO_TICKS(200)) != pdTRUE)
        LOG("[XFER] ✗✗ 状态消息 %s 入队失败（qWebToMain 满）—— 依赖主循环超时自愈\n", m.strVal);
}

static void enterTransferMode()
{
    g_xfering = true;           // 传输期：IMU 降采样（见 imu.cpp）；LCD 据此【冻结当前帧】：
                                // 不解码 MJPEG、不画转圈 → 接收方无感，且 core0 全让给 webTask 收包，
                                // 修"边播视频边下载→视频极慢+下载失败"（见 lcd.cpp 的 g_xfering 分支）。
    // ★ 先发 XFER_LOCK 停播，再取消上轮落盘复用 PSRAM。
    LOG("[XFER] ── 锁定：停止播放 ──\n");
    MessageToMain_t msg; strcpy(msg.strVal, "XFER_LOCK");
    sendMainStateMsg(msg);
    mainWake();
    // ★ 复用同块 PSRAM 前【取消】上轮延后落盘（不再排空写完 ~20s）→ 连发时 READY 秒回。
    //   代价：被取代的中间条不进 flash；最后一条 END 时正常落盘。详见 cancelPersistForReuse。
    cancelPersistForReuse();
}

static void exitTransferMode()
{
    g_xfering = false;          // lcdTask 退出冻结、恢复状态机(idle/播放)
    LOG("[XFER] ── 解锁：恢复播放 ──\n");
    MessageToMain_t msg; strcpy(msg.strVal, "XFER_UNLOCK");
    sendMainStateMsg(msg);
    mainWake();
}

static void publishResultBytes(const char* tag, bool success, size_t bytes)
{
    StaticJsonDocument<128> doc;
    char msgVal[16];
    snprintf(msgVal, sizeof(msgVal), "%s_%s", tag, success ? "OK" : "FAIL");
    doc["msg"]       = msgVal;
    doc["xferId"]    = g_curXferId;   // ★ 回执带会话 ID：App 据此丢弃上一轮迟到的回执，
                                      //   避免重传时旧回执被当成本轮结果导致时序错乱
    char payload[128];
    serializeJson(doc, payload, sizeof(payload));

    LOG("[%s] %s | %d 字节 | → %s : %s\n",
        tag, success ? "成功" : "失败", (int)bytes, responseTopic.c_str(), payload);
    // ★ PubSubClient 只能发 QoS0（发布端不支持 QoS1），回执丢包 App 就白等超时。
    //   先连发 2 次做即时补偿，再缓存起来由 webTask 在随后几秒内间隔补发几次（见下），
    //   两手一起把"设备发完了、App 却收不到结果"的尾丢包基本堵死。App 端对同一结果天然去重。
    client.publish(responseTopic.c_str(), payload);
    client.publish(responseTopic.c_str(), payload);
    strncpy(g_lastResultPayload, payload, sizeof(g_lastResultPayload) - 1);
    g_lastResultPayload[sizeof(g_lastResultPayload) - 1] = 0;
    g_lastResultXferId = g_curXferId;   // 记住结果所属会话，供重复 END 幂等重发
    g_resultResendLeft = RESULT_RESEND_TIMES;
    g_resultResendAtMs = millis() + RESULT_RESEND_STEP_MS;
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
//  统一语音 ARQ 发送引擎（UVA）——设备→App 与 设备→伙伴 共用一套发送器
//  （合并原「阶段D App 语音」与「P4 伙伴语音发送」两条路径）
//
//  · Go-Back-N 滑动窗口：多片在途填满 RTT（提速关键，停等式每片一个往返在云 broker 上极慢）；
//    接收端按 offset 顺序落位、回【累积 next】，发送端据此滑窗；停滞(无 ACK 进展)则回退重发整窗。
//  · 两端 QoS0 安全：不假设回程可靠（伙伴回程也 QoS0），全靠超时重传兜底。
//    App 回程用 mqtt_client 可发 QoS1、更稳，但引擎完全不依赖它。
//  · 传输期「对端存活」判据 = 回程 ACK 本身（不再单开心跳）；开传前用重发 BEGIN 握手探测在场。
//  · 目标两类（UvKind）：
//      UV_APP  → term/<sn>/voice              ；成功=删语音槽（留言已送达）；失败=不删，留槽等下次 PULL。
//      UV_PEER → dev/<topicId(peer)>/voicePlay ；伙伴即播即删；本端成功/失败都不动槽（该槽仍归 App 路径）。
//  · 回程统一走本机 cmd（PV_ACK/PV_DONE/PV_FAIL/PV_BUSY），dispatchCommand 按 xferId 路由到对应会话。
//  · 协议消息名沿用 PV_*：与设备【接收端】handlePeerVoice 共用同一套，App 端也说这套。
//    帧常量另起 UV_ 名（值与接收端 PV_ 相同），因接收端的 PV_ 宏定义在本段之后。
// ══════════════════════════════════════════════════════════════
#define UV_CHUNK            4000       // 每片数据字节（+9 头 < 4096）
#define UV_MAX_SEND_PER_PUMP 2              // ★每轮 pump 每会话最多发几片：靠 5ms pump 节奏限速，
                                            //   避免一次猛发整窗被 Serverless broker 限速丢 QoS0 片。
#define UV_HDR              9          // 数据帧头：1(magic)+4(xferId,BE)+4(offset,BE)
#define UV_MAGIC            0xD1       // 数据帧魔数（≠ '{'，与接收端 PV_MAGIC 同值）

// ── 拥塞窗口（AIMD）───────────────────────────────────────────────────────────
//  原来是固定 8 片(32KB)窗口。在 EMQX Serverless 限速下"发得越猛丢得越多"，固定窗口会
//  持续过载 → 整窗 Go-Back-N 重发 → 更拥塞。改成加性增/乘性减，自动收敛到链路实际能力。
//  三端联合仿真实测：丢包 40% 场景放大倍数 5.82x → 3.38x，丢包 55% 场景 8.47x → 4.88x。
#define UV_CWND_INIT        (4 * UV_CHUNK)
#define UV_CWND_MIN         (2 * UV_CHUNK)
#define UV_CWND_MAX         (8 * UV_CHUNK)

// ── 超时参数（★ 经三端联合仿真参数扫描标定，勿凭感觉改）──────────────────────
//  关键不变量：UV_STEP_MAX_RETRY × UV_STEP_TIMEOUT_MS  <  UV_LIVENESS_MS
//    否则重传次数还没用完就被"对端无响应"砍掉 —— 这正是改造前丢包场景下的主要失败原因。
//    改造前 8×1200=9.6s > 6s（被砍），现在 20×1200=24s < 30s（重试跑满才由 liveness 收尾）。
//  扫描结论：STEP 必须保持短（快速重传是高丢包下的生命线，拉长到 3000 会让成功率从 80% 掉到 33%）；
//           真正要放宽的是 MAX_RETRY 与 LIVENESS。
//  实测成功率：丢包25% 95%→100%，丢包40% 80%→100%，丢包55% 35%→83%。
#define UV_STEP_TIMEOUT_MS  1200       // 单步(BEGIN/片/END)等回程 ACK 超时 → 重发当前步（保持不动）
#define UV_STEP_MAX_RETRY   20         // 8 → 20
#define UV_LIVENESS_MS      30000      // 6000 → 30000：连续这么久毫无回程 → 判对端不在
#define UV_HANDSHAKE_MS     10000      // 6000 → 10000：开传前等 BEGIN 应答的窗口
#define UV_TOTAL_TIMEOUT_MS 180000     // 整条总超时兜底

// ══════════════════════════════════════════════════════════════
//  出站投递账本（Outbox Ledger）
//    一条语音要投给【两个互不相干的目的地】：App 与 伙伴设备。
//    改造前只有"文件在/不在"两态，无法表达"App 已收、伙伴未收"，于是：
//      · App 送达即 LittleFS.remove → 伙伴还没开传就永远收不到（仿真实测伙伴成功率 0%）；
//      · 任一路失败即 g_pullVoiceReq=false → 整条链路永久熄火。
//    现在每条消息对每个目的地各记一格状态，**文件删除权只归 vmSettle()**：
//    仅当两路都到终态(DONE/GIVEUP/NA)才删。两路的调度、退避、放弃彼此完全独立。
// ══════════════════════════════════════════════════════════════
// ── 语音链路日志分级 ────────────────────────────────────────────────────────
//  真机排查时改这里。115200 串口下逐帧日志（60 片 + 60 个 ACK）会把关键信息淹掉，
//  所以默认只到 1 级；抓协议细节时临时开到 2。
//    0 = 关键事件：会话起止 / 账本跃迁 / 失败原因 / presence 变化 / 错误
//    1 = + 传输进度汇总（节流到每秒一行）+ 调度器"为什么不发"（原因变化时一行）
//    2 = + 逐帧细节：每一片数据、每一个 ACK 的路由结果
#define UV_LOG_LEVEL 1
#define UVLOG1(...)  do { if (UV_LOG_LEVEL >= 1) LOG(__VA_ARGS__); } while (0)
#define UVLOG2(...)  do { if (UV_LOG_LEVEL >= 2) LOG(__VA_ARGS__); } while (0)
#define UV_PROGRESS_LOG_MS 1000     // 进度汇总最小间隔

// ── 不变量断言 ──────────────────────────────────────────────────────────────
//  这些是协议正确性的地基，被破坏就说明出现了新的竞争。**只打日志不 panic**：
//  语音传输失败远没有重启严重，宁可降级也别把用户正在用的设备干掉。
//  真机上一旦看到 [UVINV]，就把当时的条件拿到 Test/uv_sim/fuzz.py 里复现。
//  已知不变量（改协议时必须继续成立）：
//    I-A  发送端：sent >= next（窗口右沿不在左沿之前）—— uint32 下溢防护依赖它
//    I-B  发送端：next <= size
//    I-C  接收端：同一 xferId 内 next 单调不减 —— ACK 水位过滤依赖它
//    I-D  账本：文件存在 ⟺ 账本 used；文件删除权只在 vmSettle
//    I-E  投递：同一条语音对同一目的地最多成功投递一次（uid 去重保证观感）
#define UV_INV(cond, fmt, ...)  do { if (!(cond)) LOG("[UVINV] ✗ " fmt "\n", ##__VA_ARGS__); } while (0)

// ── 对端在线判据（两侧对称，都由 broker 用 retained + LWT 维护，设备侧零轮询）──
//   g_peerOnline ← devInfo/<peerSn>            （已有）
//   g_appOnline  ← term/<terminal>/presence    （新增，App 进后台即报 offline）
//   ★ 不变量 I3：对端离线时该通路一片不发、一次握手都不做 → 零功耗等待、上线即投。
volatile bool   g_appOnline = false;
static bool     s_appPresenceSeen = false;      // 是否收到过 presence（未收到时走乐观降级，见下）
static uint32_t s_appPresenceFirstMs = 0;
char            appPresenceSubAddr[96] = {0};
static String   g_subTerminal;
// 兜底：EMQX ACL 若未放行 term/+/presence，presence 永不到达。此时不能一直不投递，
// 订阅后 PRESENCE_GRACE_MS 内没收到任何 presence → 降级为"乐观在线"，按旧行为试投。
#define PRESENCE_GRACE_MS  30000
// terminal 还没学到时录了语音 → 推送挂起，学到后由 learnTerminal 补发
static bool s_newVoicePend = false;
static inline bool appDeliverable() {
    if (appPresenceSubAddr[0] == 0) return false;                 // terminal 还没学到
    if (s_appPresenceSeen) return g_appOnline;
    return (uint32_t)(millis() - s_appPresenceFirstMs) > PRESENCE_GRACE_MS;   // 乐观降级
}

typedef enum : uint8_t {
    DST_NA      = 0,   // 该目的地不适用（未绑伙伴 / 还没学到 terminal）
    DST_PENDING = 1,   // 待投递（等对端上线，或等退避到期）
    DST_SENDING = 2,   // 会话进行中
    DST_DONE    = 3,   // 已确认送达
    DST_GIVEUP  = 4,   // 放弃（超重试上限 / 超 TTL / 源不可读）
} DstState;

typedef struct __attribute__((packed)) {
    uint32_t uid;       // 内容唯一 id（App 去重键）。★ 持久化 → 重启后仍稳定，不再重复冒泡
    uint32_t bornSec;   // 录制时刻 epoch 秒（NTP 未同步为 0，此时不做 TTL 判定）
    uint8_t  used;      // 0 = 空槽
    uint8_t  app;       // DstState
    uint8_t  peer;      // DstState
    uint8_t  appTry;    // 链路型失败累计
    uint8_t  peerTry;
    uint8_t  _rsv[3];
} VoiceMeta;                                   // 16 B

#define VMETA_PATH   "/voice_meta.bin"
#define VMETA_MAGIC  0x564D3031u               // "VM01"
#define VOICE_TTL_SEC (7 * 24 * 3600)          // 未送达消息最长保留 7 天

// 链路型失败退避（对端在线却失败 = 链路问题）。对端【离线】根本不会开会话，故无需长退避。
static const uint32_t UV_LINK_BACKOFF[] = { 5000, 15000, 45000, 120000, 300000 };
#define UV_LINK_MAX_TRY  (sizeof(UV_LINK_BACKOFF) / sizeof(UV_LINK_BACKOFF[0]))

static VoiceMeta s_vm[VOICE_MAX];
static uint32_t  s_appNextMs [VOICE_MAX];      // 退避到期（RAM only，重启归零=立即可试）
static uint32_t  s_peerNextMs[VOICE_MAX];

// 会话收尾时告诉账本"这次失败该怎么记"
typedef enum {
    UVR_LINK,      // 链路型：计入重试次数，按退避表重排
    UVR_YIELD,     // 让路（下载抢占）：不计次数，2s 后重排
    UVR_BADSRC,    // 源不可读：该槽直接两路终结（防坏槽永久堵塞后续所有语音）
} UvFailKind;

static void vmDump(const char* tag);   // 账本快照（定义在下方）

static uint32_t vmCrc32(const uint8_t* p, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (-(int32_t)(c & 1)));
    }
    return ~c;
}

// 原子写（.tmp → rename）。只在【终态跃迁】时调用，约 3~4 次/条消息 × 168B，对 flash 可忽略。
static void vmSave() {
    uint8_t buf[8 + sizeof(s_vm)];
    uint32_t magic = VMETA_MAGIC;
    memcpy(buf,     &magic, 4);
    memcpy(buf + 8, s_vm,   sizeof(s_vm));
    uint32_t crc = vmCrc32(buf + 8, sizeof(s_vm));
    memcpy(buf + 4, &crc,   4);
    File f = LittleFS.open(VMETA_PATH ".tmp", FILE_WRITE);
    if (!f) { LOG("[VM] ✗ 账本写入失败\n"); return; }
    f.write(buf, sizeof(buf));
    f.close();
    LittleFS.remove(VMETA_PATH);
    LittleFS.rename(VMETA_PATH ".tmp", VMETA_PATH);
}

// 开机加载 + 与实际文件对账（这段同时完成【老固件升级迁移】：文件在但无账本记录→按未投递重建）
static void vmLoad() {
    bool ok = false;
    File f = LittleFS.open(VMETA_PATH, FILE_READ);
    if (f && f.size() == 8 + sizeof(s_vm)) {
        uint8_t buf[8 + sizeof(s_vm)];
        if (f.read(buf, sizeof(buf)) == sizeof(buf)) {
            uint32_t magic, crc;
            memcpy(&magic, buf, 4);
            memcpy(&crc,   buf + 4, 4);
            if (magic == VMETA_MAGIC && crc == vmCrc32(buf + 8, sizeof(s_vm))) {
                memcpy(s_vm, buf + 8, sizeof(s_vm));
                ok = true;
            }
        }
    }
    if (f) f.close();
    if (!ok) memset(s_vm, 0, sizeof(s_vm));

    for (int i = 0; i < VOICE_MAX; i++) {
        char p[24]; snprintf(p, sizeof(p), VOICE_PATH_FMT, i);
        if (!LittleFS.exists(p)) { memset(&s_vm[i], 0, sizeof(VoiceMeta)); continue; }
        if (!s_vm[i].used) {                          // 老固件遗留 / 账本损坏 → 按未投递重建
            memset(&s_vm[i], 0, sizeof(VoiceMeta));
            s_vm[i].used = 1;
            s_vm[i].uid  = esp_random() | 1u;
            s_vm[i].app  = DST_PENDING;
            s_vm[i].peer = DST_PENDING;
            LOG("[VM] 槽 %d 无账本记录，按未投递重建\n", i);
        }
        // 断电打断的在途会话 → 回待投（不会重复投给已 DONE 的那一路）
        if (s_vm[i].app  == DST_SENDING) s_vm[i].app  = DST_PENDING;
        if (s_vm[i].peer == DST_SENDING) s_vm[i].peer = DST_PENDING;
    }
    vmSave();
    LOG("[VM] 账本就绪：%s\n", ok ? "已加载" : "已重建");
    vmDump("开机");
}

static const char* dstName(uint8_t st) {
    switch (st) {
        case DST_NA:      return "NA";
        case DST_PENDING: return "PEND";
        case DST_SENDING: return "SEND";
        case DST_DONE:    return "DONE";
        case DST_GIVEUP:  return "GIVE";
        default:          return "??";
    }
}

// ★ 账本快照：一行看清"每条语音对两个目的地各自到哪一步了"。
//   排查语音问题时这是第一手信息 —— 逐事件日志回答"发生了什么"，这个回答"现在是什么状态"。
//   格式： 槽[uid app状态/peer状态 重试次数 剩余退避秒]
static void vmDump(const char* tag) {
    char line[224];
    int  p = 0, n = 0;
    uint32_t now = millis();
    for (int i = 0; i < VOICE_MAX && p < (int)sizeof(line) - 48; i++) {
        if (!s_vm[i].used) continue;
        n++;
        int32_t ad = (int32_t)(s_appNextMs[i]  - now);
        int32_t pd = (int32_t)(s_peerNextMs[i] - now);
        p += snprintf(line + p, sizeof(line) - p, " %d[%08x %s/%s t%u/%u",
                      i, (unsigned)s_vm[i].uid,
                      dstName(s_vm[i].app), dstName(s_vm[i].peer),
                      (unsigned)s_vm[i].appTry, (unsigned)s_vm[i].peerTry);
        if (s_appNextMs[i]  && ad > 0) p += snprintf(line + p, sizeof(line) - p, " a+%lds", (long)(ad / 1000));
        if (s_peerNextMs[i] && pd > 0) p += snprintf(line + p, sizeof(line) - p, " p+%lds", (long)(pd / 1000));
        p += snprintf(line + p, sizeof(line) - p, "]");
    }
    if (n == 0) { line[0] = ' '; line[1] = '-'; line[2] = 0; }
    LOG("[VM] 账本(%s) %d条 | App=%s%s Peer=%s |%s\n",
        tag, n,
        g_appOnline ? "在线" : "离线",
        s_appPresenceSeen ? "" : "(presence未收到,宽限期)",
        g_peerOnline ? "在线" : "离线",
        line);
}

// ★ 唯一有权删除语音文件的地方（不变量 I1）
static void vmSettle(int slot) {
    if (slot < 0 || slot >= VOICE_MAX) return;
    VoiceMeta* m = &s_vm[slot];
    if (!m->used) return;
    bool a = (m->app  == DST_DONE || m->app  == DST_GIVEUP || m->app  == DST_NA);
    bool p = (m->peer == DST_DONE || m->peer == DST_GIVEUP || m->peer == DST_NA);
    if (!(a && p)) return;
    char f[24]; snprintf(f, sizeof(f), VOICE_PATH_FMT, slot);
    LittleFS.remove(f);
    LOG("[VM] 槽 %d 两路均终结(app=%u peer=%u)，文件删除\n",
        slot, (unsigned)m->app, (unsigned)m->peer);
    memset(m, 0, sizeof(*m));
    vmSave();
}

// 坏槽自愈：打不开/0 字节的槽直接两路终结并删掉。
// 改造前这种槽会让 findStoredVoiceSlot 每次都先选中它、瞬间失败、清掉 g_pullVoiceReq，
// 后面 voice_1..9 永远轮不到 —— 仿真实测该场景成功率 0%。
static void vmMarkBadSlot(int slot) {
    if (slot < 0 || slot >= VOICE_MAX) return;
    s_vm[slot].used = 1;
    // ★ 只终结【当前不在发送中】的通路。另一路可能正持有这个文件的读句柄(App 的 s_uvAppFile)
    //   或正在做 PSRAM 快照(伙伴)，把它也标成 GIVEUP 会让 vmSettle 把它正在读的文件删掉。
    //   那一路自己失败时会再走一遍这里，届时它已不是 SENDING，能正常终结。
    if (s_vm[slot].app  != DST_SENDING) s_vm[slot].app  = DST_GIVEUP;
    if (s_vm[slot].peer != DST_SENDING) s_vm[slot].peer = DST_GIVEUP;
    LOG("[VM] ✗ 槽 %d 不可读 → app=%s peer=%s\n",
        slot, dstName(s_vm[slot].app), dstName(s_vm[slot].peer));
    vmSettle(slot);
}

// 供 mic.cpp 环形淘汰使用：该槽是否正被任一通路发送（两路都要保护，不只 App）
bool voiceSlotBusy(int slot) {
    return slot >= 0 && slot < VOICE_MAX &&
           (s_vm[slot].app == DST_SENDING || s_vm[slot].peer == DST_SENDING);
}

// ★★ 竞争修复：账本只允许 webTask 改。
//   voiceEnqueue 是从 micTask 调的，若直接改 s_vm，会在 webTask 执行 vmSave()
//   把 s_vm 序列化进缓冲的中途抢占（micTask 优先级 1 > webTask 的 0，同在 core 0）
//   → 写进 flash 的账本是撕裂的。这里只置一个标志（bool 的存/取在 Xtensa 上是原子的），
//   实际入账由 webTask 在 uvService 里做。
static volatile int8_t s_enqSlot = -1;
static volatile bool   s_enqReq  = false;
void voiceEnqueue(int slot) {
    if (slot < 0 || slot >= VOICE_MAX) return;
    s_enqSlot = (int8_t)slot;
    __sync_synchronize();           // 先写槽号、再立标志（webTask 看到标志时槽号一定已可见）
    s_enqReq  = true;
}

// 由 webTask 调用：登记新录音。
//   ① 先清掉"本次明确指定的槽"的旧账本记录 —— ★必须有：mic.cpp 槽满时会【环形淘汰最旧一条】
//      并复用它的槽号，而被淘汰那条的账本记录还在（used=1）。不清的话新录音会继承旧记录：
//      旧 uid（App 会按 uid 去重丢弃）、旧的 app/peer 状态（若旧记录是 app=DONE，
//      这条新语音就【永远不会发给 App】）、旧的重试计数。
//   ② 再扫一遍"有文件但无账本记录"的槽兜底（连录多条时标志被覆盖也不会漏）。
static void vmRegisterNew() {
    bool dirty = false;
    int8_t forced = s_enqSlot;
    s_enqSlot = -1;
    if (forced >= 0 && forced < VOICE_MAX && s_vm[forced].used) {
        char p[24]; snprintf(p, sizeof(p), VOICE_PATH_FMT, forced);
        if (LittleFS.exists(p)) {
            // mic.cpp 的 pickOldestVoiceSlot 会跳过 voiceSlotBusy 的槽，正常不会撞上在途会话；
            // 但"检查忙 → 删文件"之间有个极窄窗口。真撞上了就告警：该会话在传的内容已被换掉，
            // 它会自行失败或 DONE（vmOnDone/vmOnFail 遇到已清空的记录会直接返回，不误伤新消息），
            // 随后下面的扫描把新录音重新登记，两路都会重发一遍。
            if (s_vm[forced].app == DST_SENDING || s_vm[forced].peer == DST_SENDING)
                LOG("[VM] ⚠ 槽 %d 在发送中却被录音复用（窄窗口竞争），该会话内容已失效\n", (int)forced);
            LOG("[VM] 槽 %d 被复用（环形淘汰），清掉旧记录 uid=%08x app=%s peer=%s\n",
                (int)forced, (unsigned)s_vm[forced].uid,
                dstName(s_vm[forced].app), dstName(s_vm[forced].peer));
            memset(&s_vm[forced], 0, sizeof(VoiceMeta));   // 下面的扫描会把它当成新消息登记
        }
    }
    bool haveTerm = !Config.getString("terminal", "").isEmpty();
    bool havePeer = !Config.getString("peerSn",   "").isEmpty();
    for (int i = 0; i < VOICE_MAX; i++) {
        if (s_vm[i].used) continue;
        char p[24]; snprintf(p, sizeof(p), VOICE_PATH_FMT, i);
        if (!LittleFS.exists(p)) continue;
        VoiceMeta* m = &s_vm[i];
        memset(m, 0, sizeof(*m));
        m->used    = 1;
        m->uid     = esp_random() | 1u;
        time_t now = time(nullptr);
        m->bornSec = (now > 1600000000) ? (uint32_t)now : 0;   // NTP 未同步则不参与 TTL
        m->app     = haveTerm ? DST_PENDING : DST_NA;
        m->peer    = havePeer ? DST_PENDING : DST_NA;
        s_appNextMs[i] = s_peerNextMs[i] = 0;                  // 立即可投
        dirty = true;
        LOG("[VM] 槽 %d 入队 uid=%08x app=%s peer=%s\n",
            i, (unsigned)m->uid, dstName(m->app), dstName(m->peer));
    }
    if (dirty) { vmSave(); vmDump("新录音"); }
}

// 按 bornSec 升序取"该通路 PENDING 且退避已到期"的最旧槽（FIFO 投递顺序）
static int vmPickNext(bool isApp, uint32_t now) {
    int best = -1; uint32_t bestKey = 0xFFFFFFFFu;
    for (int i = 0; i < VOICE_MAX; i++) {
        VoiceMeta* m = &s_vm[i];
        if (!m->used) continue;
        if ((isApp ? m->app : m->peer) != DST_PENDING) continue;
        uint32_t due = isApp ? s_appNextMs[i] : s_peerNextMs[i];
        if (due && (int32_t)(now - due) < 0) continue;
        uint32_t key = m->bornSec ? m->bornSec : (uint32_t)i;   // NTP 未同步时退化为槽序
        if (key < bestKey) { bestKey = key; best = i; }
    }
    return best;
}

// 某通路一次投递失败后的记账
static void vmOnFail(int slot, bool isApp, UvFailKind kind) {
    if (slot < 0 || slot >= VOICE_MAX || !s_vm[slot].used) return;
    if (kind == UVR_BADSRC) { vmMarkBadSlot(slot); return; }
    VoiceMeta* m = &s_vm[slot];
    uint8_t*  st  = isApp ? &m->app    : &m->peer;
    uint8_t*  tr  = isApp ? &m->appTry : &m->peerTry;
    uint32_t* due = isApp ? &s_appNextMs[slot] : &s_peerNextMs[slot];
    *st = DST_PENDING;
    if (kind == UVR_YIELD) { *due = millis() + 2000; return; }   // 让路不计次数
    if (++(*tr) >= (uint8_t)UV_LINK_MAX_TRY) {
        *st = DST_GIVEUP;
        LOG("[VM] 槽 %d %s 通路连续失败 %u 次 → 放弃该路（另一路不受影响）\n",
            slot, isApp ? "App" : "伙伴", (unsigned)*tr);
        vmSave(); vmSettle(slot);
        return;
    }
    *due = millis() + UV_LINK_BACKOFF[*tr - 1];
    LOG("[VM] 槽 %d %s 通路第 %u 次失败，%lus 后重试\n",
        slot, isApp ? "App" : "伙伴", (unsigned)*tr,
        (unsigned long)(UV_LINK_BACKOFF[*tr - 1] / 1000));
}

// 某通路投递成功
static void vmOnDone(int slot, bool isApp) {
    if (slot < 0 || slot >= VOICE_MAX || !s_vm[slot].used) return;
    if (isApp) { s_vm[slot].app = DST_DONE;  s_vm[slot].appTry  = 0; }
    else       { s_vm[slot].peer = DST_DONE; s_vm[slot].peerTry = 0; }
    vmSave();
    vmSettle(slot);
}

// TTL 老化 + terminal/peer 由无到有时把 NA 提回 PENDING。webTask 每分钟调一次。
static void vmAgeTick() {
    static uint32_t s_last = 0;
    if (millis() - s_last < 60000) return;
    s_last = millis();
    bool haveTerm = !Config.getString("terminal", "").isEmpty();
    bool havePeer = !Config.getString("peerSn",   "").isEmpty();
    time_t nowSec = time(nullptr);
    bool   timeOk = (nowSec > 1600000000);
    bool   dirty  = false;
    for (int i = 0; i < VOICE_MAX; i++) {
        VoiceMeta* m = &s_vm[i];
        if (!m->used) continue;
        if (haveTerm && m->app  == DST_NA) { m->app  = DST_PENDING; s_appNextMs[i]  = 0; dirty = true; }
        if (havePeer && m->peer == DST_NA) { m->peer = DST_PENDING; s_peerNextMs[i] = 0; dirty = true; }
        // ★ 必须先判 nowSec > bornSec：NTP 重新同步可能把时钟往回拨，此时 (nowSec - bornSec)
        //   作为无符号数会回绕成天文数字 → TTL 立刻"过期" → 未投递的语音被误删。
        if (timeOk && m->bornSec && (uint32_t)nowSec > m->bornSec &&
            (uint32_t)nowSec - m->bornSec > VOICE_TTL_SEC) {
            LOG("[VM] 槽 %d 超过 TTL(%d 天)，放弃未投递的通路\n", i, VOICE_TTL_SEC / 86400);
            if (m->app  == DST_PENDING) m->app  = DST_GIVEUP;
            if (m->peer == DST_PENDING) m->peer = DST_GIVEUP;
            dirty = true;
        }
    }
    if (dirty) { vmSave(); for (int i = 0; i < VOICE_MAX; i++) vmSettle(i); }
}

// 非 static：mic.cpp 读它（现已改读 voiceSlotBusy，保留此变量供日志/兼容）
volatile int    g_voiceSendingId     = -1;      // App 会话当前所发的槽，-1=空闲

// ── 统一发送会话：一个 App 会话 + 一个 Peer 会话，可并发（一次录音同时发 App 与伙伴）──
typedef enum { UV_APP, UV_PEER } UvKind;
typedef enum { UVS_IDLE, UVS_WAIT_READY, UVS_SENDING, UVS_WAIT_DONE } UvState;

typedef struct {
    UvKind   kind;
    UvState  state;
    char     dstTopic[64];   // 帧目的主题（App:term/<sn>/voice ; Peer:dev/<topicId(peer)>/voicePlay）
    uint32_t xferId;
    int      slot;           // 源语音槽
    uint32_t uid;            // App 内容去重键（随 BEGIN 发；伙伴忽略）
    uint8_t* snap;           // UV_PEER：整条 PSRAM 快照；UV_APP=nullptr（直接读槽文件 s_uvAppFile）
    uint32_t size;
    uint32_t next;           // 已确认的累积 offset（= 对端期望的下一个 = 窗口左沿）
    uint32_t sent;           // 已发出的最高 offset（窗口右沿；next..sent 为在途未确认）
    uint32_t deadline;       // 停滞超时时刻（acked 无进展达此 → 回退重发窗口）
    int      retry;          // 已重发次数
    uint32_t startMs;        // 会话起始（总超时/握手窗口基准）
    uint32_t lastRxMs;       // 上次收到本会话任意回程的时刻（传输期存活时钟）
    uint32_t cwnd;           // 当前拥塞窗口（字节，AIMD）
    uint32_t stalls;         // 累计停滞回退次数（诊断用）
    uint32_t logMs;          // 上次打进度汇总的时刻（节流用）
    uint32_t logAcked;       // 上次打进度时的 acked（用于算瞬时速率）
    uint32_t ackHigh;        // ★ 本会话见过的最大 ackNext（单调水位，见 uvOnAck）
} UvSession;

static UvSession s_uvApp  = { UV_APP,  UVS_IDLE, {0}, 0, -1, 0, nullptr, 0, 0, 0, 0, 0, 0, 0, UV_CWND_INIT, 0, 0, 0, 0 };
static UvSession s_uvPeer = { UV_PEER, UVS_IDLE, {0}, 0, -1, 0, nullptr, 0, 0, 0, 0, 0, 0, 0, UV_CWND_INIT, 0, 0, 0, 0 };

// 传输进度汇总（节流到每秒一行）：带速率和 cwnd，卡住时一眼看出卡在哪个 offset。
static void uvLogProgress(UvSession* s, const char* tag) {
    uint32_t now = millis();
    if (now - s->logMs < UV_PROGRESS_LOG_MS) return;
    uint32_t dt = now - s->logMs;
    uint32_t d  = (s->next > s->logAcked) ? (s->next - s->logAcked) : 0;
    s->logMs = now; s->logAcked = s->next;
    UVLOG1("[UV%s] %s %u/%u (%u%%) %ukB/s cwnd=%u inflight=%u stall=%u retry=%d\n",
           tag, s->kind == UV_APP ? "App" : "伙伴",
           (unsigned)s->next, (unsigned)s->size,
           (unsigned)(s->size ? (uint64_t)s->next * 100 / s->size : 0),
           (unsigned)(dt ? d / dt : 0),
           (unsigned)s->cwnd,
           (unsigned)(s->sent > s->next ? s->sent - s->next : 0),
           (unsigned)s->stalls, s->retry);
}

static inline void uvCwndGrow(UvSession* s) {          // 加性增
    if (s->cwnd + (uint32_t)UV_CHUNK <= (uint32_t)UV_CWND_MAX) s->cwnd += (uint32_t)UV_CHUNK;
}
static inline void uvCwndCut(UvSession* s) {           // 乘性减（停滞 = 拥塞信号）
    s->cwnd /= 2;
    if (s->cwnd < (uint32_t)UV_CWND_MIN) s->cwnd = (uint32_t)UV_CWND_MIN;
}
static uint8_t   s_uvFrame[UV_HDR + UV_CHUNK];   // 组帧临时缓冲（各 pump 内同步用完即弃，两会话不并发用同一份）
static File      s_uvAppFile;                    // UV_APP 直接读的槽文件句柄（本端删槽前不会被别处删）

// 有任一发送会话在跑：webTask 据此提速轮询到 5ms + WiFi 满功率，powerManagerLoop 据此保 CPU 满频。
static inline bool uvActive() { return s_uvApp.state != UVS_IDLE || s_uvPeer.state != UVS_IDLE; }

// 读源：Peer 从 PSRAM 快照；App 从槽文件按 offset seek 读（只读，无 flash 写冻结问题）。
static bool uvReadAt(UvSession* s, uint32_t off, uint8_t* dst, uint32_t len) {
    if (s->kind == UV_PEER) {
        if (!s->snap) return false;
        memcpy(dst, s->snap + off, len);
        return true;
    }
    if (!s_uvAppFile) return false;
    if (!s_uvAppFile.seek(off)) return false;
    return (uint32_t)s_uvAppFile.read(dst, len) == len;
}

// 组一片数据帧 [magic|xferId(BE)|offset(BE)|data] 并发出（QoS0）。
// ★ 三态返回：1=已发出 / 0=publish 失败(缓冲满/限速/ACL，不推进 sent，下轮再试) / -1=读源失败(会话收手)。
//   改造前 publish 失败也返回 true，调用方照样 sent += len —— 没发出去的片被记成已发，
//   只能等一整个 1200ms 停滞周期才补，在 broker 限速时会成片浪费。
static int uvSendChunk(UvSession* s, uint32_t off) {
    healthSetTaskStage(HT_WEB, HS_W_VOICETX);   // 读 LittleFS + publish
    if (off >= s->size) return -1;
    uint32_t remain = s->size - off;
    uint32_t want = remain < (uint32_t)UV_CHUNK ? remain : (uint32_t)UV_CHUNK;
    s_uvFrame[0] = UV_MAGIC;
    s_uvFrame[1] = (uint8_t)(s->xferId >> 24); s_uvFrame[2] = (uint8_t)(s->xferId >> 16);
    s_uvFrame[3] = (uint8_t)(s->xferId >> 8);  s_uvFrame[4] = (uint8_t)(s->xferId);
    s_uvFrame[5] = (uint8_t)(off >> 24);       s_uvFrame[6] = (uint8_t)(off >> 16);
    s_uvFrame[7] = (uint8_t)(off >> 8);        s_uvFrame[8] = (uint8_t)(off);
    if (!uvReadAt(s, off, s_uvFrame + UV_HDR, want)) return -1;
    if (!client.publish(s->dstTopic, s_uvFrame, UV_HDR + want)) {
        LOG("[UVDBG] ✗ 数据片 @%u (%uB) publish FAIL（缓冲满/限速/ACL？）\n",
            (unsigned)off, (unsigned)(UV_HDR + want));
        return 0;
    }
    return 1;
}
static void uvSendBegin(UvSession* s) {
    StaticJsonDocument<160> d;
    d["msg"] = "PV_BEGIN"; d["xferId"] = s->xferId; d["sn"] = g_sn_str;
    d["size"] = s->size;   d["uid"] = s->uid;
    char b[160]; size_t n = serializeJson(d, b, sizeof(b));
    bool ok = client.publish(s->dstTopic, (const uint8_t*)b, (unsigned int)n);
    // ★DBG：BEGIN 发去哪个主题、publish 成不成（pub=FAIL 常见于 ACL 拦截/缓冲满 → 对端永远收不到 → 无人应答）
    LOG("[UVDBG] → PV_BEGIN %s xferId=%u size=%u pub=%s\n",
        s->dstTopic, (unsigned)s->xferId, (unsigned)s->size, ok ? "OK" : "FAIL");
}
static void uvSendEnd(UvSession* s) {
    StaticJsonDocument<64> d; d["msg"] = "PV_END"; d["xferId"] = s->xferId;
    char b[64]; size_t n = serializeJson(d, b, sizeof(b));
    bool ok = client.publish(s->dstTopic, (const uint8_t*)b, (unsigned int)n);
    LOG("[UVDBG] → PV_END %s xferId=%u pub=%s\n", s->dstTopic, (unsigned)s->xferId, ok ? "OK" : "FAIL");
}

// 会话收尾。★ 不再直接碰文件：只更新本通路那一格账本，删除权归 vmSettle()。
//   这样"App 送达"不会把伙伴还没拿到的文件删掉（改造前伙伴成功率实测 0%）。
static void uvStop(UvSession* s, const char* why, bool ok, UvFailKind kind = UVR_LINK) {
    int  slot  = s->slot;
    bool isApp = (s->kind == UV_APP);

    if (isApp) {
        if (s_uvAppFile) s_uvAppFile.close();
        g_voiceSendingId = -1;
    } else {
        if (s->snap) { free(s->snap); s->snap = nullptr; }   // 释放 PSRAM 快照
    }
    // ★ 入账前用 uid 校验身份：槽号会被环形淘汰复用，会话开始时的那条消息可能已经不在这个槽了。
    //   不校验的话，一条旧会话的 DONE 会把【新录进来的那条】标成已送达 → 新语音永远不发。
    //   uid 是内容级唯一标识（持久化、复用槽时会重新生成），是这里唯一可信的身份依据。
    if (slot >= 0) {
        if (s_vm[slot].used && s_vm[slot].uid == s->uid) {
            if (ok) vmOnDone(slot, isApp);
            else    vmOnFail(slot, isApp, kind);
        } else {
            LOG("[VM] ⚠ 会话结束时槽 %d 已被复用/清空（会话 uid=%08x，账本 uid=%08x），结果不入账\n",
                slot, (unsigned)s->uid, (unsigned)(s_vm[slot].used ? s_vm[slot].uid : 0));
        }
    }

    // 一行会话总结：acked/size 的比值直接区分故障类型 ——
    //   0        = 根本没开始（对端没应答 BEGIN）
    //   中间值   = 传到一半断
    //   =size 却 FAIL = 数据都到了但 END/DONE 没走完
    LOG("[UVSTAT] %s slot=%d size=%u acked=%u stall=%u retry=%d cwnd=%u dur=%lums %s why=%s\n",
        isApp ? "APP" : "PEER", slot, (unsigned)s->size, (unsigned)s->next,
        (unsigned)s->stalls, s->retry, (unsigned)s->cwnd,
        (unsigned long)(millis() - s->startMs), ok ? "OK" : "FAIL", why);

    s->state = UVS_IDLE;
    s->slot  = -1;
    vmDump(ok ? "投递成功" : "投递失败");

    // LCD 收尾：只看"是否还有任一会话在跑"，不再互相读对方 state（解耦 K4）
    if (!(s_uvApp.state != UVS_IDLE || s_uvPeer.state != UVS_IDLE)) {
        if (ok)                   g_xferAnim = 2;   // 打勾（drawXferAnim 播完自动回 0）
        else if (g_xferAnim == 1) g_xferAnim = 0;   // 撤转圈
    }
}

// 启动 App 会话：直接读槽文件（不快照，省 PSRAM；本端删槽前不会被别处删）。返回 false=打不开/空。
static bool uvStartApp(int slot) {
    char path[24]; snprintf(path, sizeof(path), VOICE_PATH_FMT, slot);
    s_uvAppFile = LittleFS.open(path, FILE_READ);
    if (!s_uvAppFile) { LOG("[UV] App 打不开槽 %d\n", slot); return false; }
    uint32_t sz = s_uvAppFile.size();
    if (sz == 0) { s_uvAppFile.close(); return false; }
    s_uvApp.slot     = slot;
    s_uvApp.uid      = s_vm[slot].uid;
    s_uvApp.snap     = nullptr;
    s_uvApp.size     = sz;
    s_uvApp.xferId   = esp_random() | 1u;
    s_uvApp.next     = 0;
    s_uvApp.sent     = 0;
    s_uvApp.retry    = 0;
    s_uvApp.stalls   = 0;
    s_uvApp.cwnd     = UV_CWND_INIT;
    s_uvApp.ackHigh  = 0;
    s_uvApp.startMs  = millis();
    s_uvApp.lastRxMs = millis();
    s_uvApp.deadline = millis() + UV_STEP_TIMEOUT_MS;
    s_uvApp.state    = UVS_WAIT_READY;
    snprintf(s_uvApp.dstTopic, sizeof(s_uvApp.dstTopic), "%s", voicePubAddr);  // term/<sn>/voice
    g_voiceSendingId = slot;
    if (g_xferAnim != 1) g_xferAnim = 1;   // 转圈；powerManagerLoop 据此保持屏亮/满频
    uvSendBegin(&s_uvApp);
    LOG("[UV] ▶ 发 App 槽 %d（xferId=%u size=%u）\n", slot, (unsigned)s_uvApp.xferId, (unsigned)sz);
    return true;
}

// 启动 Peer 会话：整条快照进 PSRAM（发送期不持文件句柄，App 路径即便删同槽也不受影响）。
// ★ 返回 false = 槽不可读（调用方据此走 vmMarkBadSlot）；在线/忙的判定已上移到 peerSched()。
static bool uvStartPeer(int slot, const String& peer) {
    char path[24]; snprintf(path, sizeof(path), VOICE_PATH_FMT, slot);
    File f = LittleFS.open(path, FILE_READ);
    if (!f) { LOG("[UV] 伙伴打不开槽 %d\n", slot); return false; }
    uint32_t sz = f.size();
    if (sz == 0) { f.close(); return false; }
    uint8_t* data = (uint8_t*)ps_malloc(sz);
    if (!data) { f.close(); LOG("[UV] ✗ PSRAM 分配失败(%u)\n", (unsigned)sz); return false; }
    int rd = f.read(data, sz); f.close();
    if (rd < 0 || (uint32_t)rd != sz) { free(data); LOG("[UV] ✗ 读槽 %d 不全\n", slot); return false; }
    s_uvPeer.slot     = slot;
    s_uvPeer.uid      = s_vm[slot].uid;
    s_uvPeer.snap     = data;
    s_uvPeer.size     = sz;
    s_uvPeer.xferId   = esp_random() | 1u;
    s_uvPeer.next     = 0;
    s_uvPeer.sent     = 0;
    s_uvPeer.retry    = 0;
    s_uvPeer.stalls   = 0;
    s_uvPeer.cwnd     = UV_CWND_INIT;
    s_uvPeer.ackHigh  = 0;
    s_uvPeer.startMs  = millis();
    s_uvPeer.lastRxMs = millis();
    s_uvPeer.deadline = millis() + UV_STEP_TIMEOUT_MS;
    s_uvPeer.state    = UVS_WAIT_READY;
    snprintf(s_uvPeer.dstTopic, sizeof(s_uvPeer.dstTopic), "dev/%s/voicePlay", topicId(peer).c_str());
    if (g_xferAnim != 1) g_xferAnim = 1;
    uvSendBegin(&s_uvPeer);
    LOG("[UV] ▶ 推伙伴 %s 槽 %d（xferId=%u size=%u）\n",
        peer.c_str(), slot, (unsigned)s_uvPeer.xferId, (unsigned)sz);
    return true;
}

// 填满发送窗口：从 sent 处连发到 (next + cwnd) 或 size 为止。返回 false=读源失败(已收手)。
//   稳态下每收到一个累积 ACK、窗口左沿右移，这里就补发新腾出的额度 → 由接收端 ACK 节奏时钟化限速。
static bool uvFillWindow(UvSession* s) {
    uint32_t before = s->sent;
    int burst = 0;
    // ★ 下溢防护：s->sent 与 s->next 可能交叉（Go-Back-N 把 sent 拉回、随后滞留的旧 ACK 又把
    //   next 推上来）。改造前直接写 (s->sent - s->next) 是 uint32 减法，交叉时得到 ~42 亿，
    //   条件恒不成立 → 整轮一片不发（仿真实测在 ACK 滞留场景触发 1716 次空转）。
    uint32_t inflight = (s->sent > s->next) ? (s->sent - s->next) : 0;
    while (s->sent < s->size && inflight < s->cwnd
           && burst < UV_MAX_SEND_PER_PUMP) {          // ★每轮限发 UV_MAX_SEND_PER_PUMP 片（防猛发丢片）
        uint32_t remain = s->size - s->sent;
        uint32_t len = remain < (uint32_t)UV_CHUNK ? remain : (uint32_t)UV_CHUNK;
        int r = uvSendChunk(s, s->sent);
        if (r < 0) { uvStop(s, "读源失败", false, UVR_BADSRC); return false; }
        if (r == 0) break;                              // publish 失败：不推进 sent，下轮再试
        s->sent  += len;
        inflight += len;
        burst++;
    }
    if (s->sent > before) {
        UVLOG2("[UVDBG] → 数据 %s [%u..%u) acked=%u/%u cwnd=%u\n",
               s->kind == UV_APP ? "App" : "伙伴", (unsigned)before, (unsigned)s->sent,
               (unsigned)s->next, (unsigned)s->size, (unsigned)s->cwnd);
        uvLogProgress(s, "");                  // 1 级：每秒一行汇总（取代逐片刷屏）
    }
    return true;
}

// webTask 每轮推进单个会话（Go-Back-N 滑动窗口）：
//   断链/总超时/存活/握手 兜底 + 填窗口 + 全确认后发 END + 停滞回退重发。
static void uvPump(UvSession* s) {
    if (s->state == UVS_IDLE) return;
    uint32_t now = millis();
    // I-A / I-B：每轮校验窗口不变量。sent<next 说明下溢防护被绕过（新竞争），
    //            这里顺手自愈成安全值，避免 uvFillWindow 里的减法炸掉。
    UV_INV(s->sent >= s->next, "%s sent(%u) < next(%u) xferId=%u",
           s->kind == UV_APP ? "APP" : "PEER", (unsigned)s->sent, (unsigned)s->next,
           (unsigned)s->xferId);
    if (s->sent < s->next) s->sent = s->next;
    UV_INV(s->next <= s->size, "%s next(%u) > size(%u)",
           s->kind == UV_APP ? "APP" : "PEER", (unsigned)s->next, (unsigned)s->size);
    if (!client.connected()) { uvStop(s, "断链", false); return; }
    if ((int32_t)(now - (s->startMs + UV_TOTAL_TIMEOUT_MS)) >= 0) { uvStop(s, "总超时", false); return; }
    if (s->state == UVS_WAIT_READY) {
        // 开传前握手窗口：前台等这么久还没人应答 BEGIN → 收手（App 留槽等下次 PULL；伙伴丢弃）
        if ((int32_t)(now - (s->startMs + UV_HANDSHAKE_MS)) >= 0) { uvStop(s, "无人应答", false); return; }
    } else {
        // 传输期：ACK 本身就是存活信号；连续这么久毫无回程 → 判对端不在 → 收手
        if ((int32_t)(now - (s->lastRxMs + UV_LIVENESS_MS)) >= 0) { uvStop(s, "对端无响应", false); return; }
    }
    switch (s->state) {
        case UVS_SENDING:
            if (!uvFillWindow(s)) break;                 // 读源失败已收手
            if (s->next >= s->size) {                    // 全部数据已被【确认】 → 发 END 等 DONE
                uvSendEnd(s); s->state = UVS_WAIT_DONE; s->retry = 0; s->deadline = now + UV_STEP_TIMEOUT_MS;
                break;
            }
            // 停滞重传：deadline 到（累积 ACK 一直无进展）→ Go-Back-N 回退到已确认处，重发整窗
            if ((int32_t)(now - s->deadline) >= 0) {
                if (++s->retry > UV_STEP_MAX_RETRY) { uvStop(s, "重试超限", false); break; }
                s->stalls++;
                uvCwndCut(s);                            // ★ 停滞 = 拥塞信号 → 窗口减半
                LOG("[UV] 停滞回退重发 @%u cwnd→%u（xferId=%u）\n",
                    (unsigned)s->next, (unsigned)s->cwnd, (unsigned)s->xferId);
                s->sent = s->next;                       // 回退窗口右沿到左沿
                s->deadline = now + UV_STEP_TIMEOUT_MS;
                uvFillWindow(s);                         // 立即重发窗口
            }
            break;
        case UVS_WAIT_READY:   // 等 BEGIN 应答
        case UVS_WAIT_DONE:    // 全部已确认+已发 END，等整条 DONE
            if ((int32_t)(now - s->deadline) >= 0) {
                if (++s->retry > UV_STEP_MAX_RETRY) { uvStop(s, "重试超限", false); break; }
                if (s->state == UVS_WAIT_READY) uvSendBegin(s);
                else                            uvSendEnd(s);
                s->deadline = now + UV_STEP_TIMEOUT_MS;
            }
            break;
        default: break;
    }
}

// ── 回程处理（在 dispatchCommand 里按 xferId 找到会话后调用，与 webTask 同任务，无需加锁）──
//   ackNext = 接收端已连续收到的字节数（累积确认）。滑动窗口据此右移窗口左沿。
static void uvOnAck(UvSession* s, uint32_t xferId, uint32_t ackNext) {
    if (s->state == UVS_IDLE || xferId != s->xferId) return;
    s->lastRxMs = millis();           // 即便是滞后 ACK 也证明对端活着 → 先刷存活时钟

    // ★★ 竞争修复：ACK 单调水位。
    //   接收端在同一个 xferId 内的 next 只增不减（App 的 _RxState.next 与设备的 s_pvRxNext
    //   都是单调的），所以任何低于历史最高值的 ackNext 必定是【broker 排队造成的滞后/乱序】。
    //   不拦的话最典型的害处是：数据已全部确认、已进 WAIT_DONE，一条滞后的低 ACK 会命中
    //   "ackNext < size → 回退续发"分支，白白重传几十 KB 并多绕一个 RTT。
    //   高延迟 broker 上滞后 ACK 是常态，所以必须拦。
    if (ackNext < s->ackHigh) {
        UVLOG2("[UVDBG] 丢弃滞后 ACK next=%u < 水位 %u（xferId=%u）\n",
               (unsigned)ackNext, (unsigned)s->ackHigh, (unsigned)xferId);
        return;
    }
    s->ackHigh = ackNext;

    if (s->state == UVS_WAIT_READY) {
        // BEGIN 就绪：从对端期望处起（通常 0；对端有残留半收态则从残留续）
        s->next = ackNext; s->sent = ackNext;
        s->state = UVS_SENDING; s->retry = 0; s->deadline = millis() + UV_STEP_TIMEOUT_MS;
    } else if (s->state == UVS_SENDING) {
        if (ackNext > s->size) return;           // 非法 ACK（越界）→ 忽略
        if (ackNext > s->next) {                 // 累积确认推进 → 滑窗（左沿右移）+ 重置停滞计时
            s->next = ackNext;
            // ★ 窗口右沿不得落在左沿之后：Go-Back-N 回退把 sent 拉回后，滞留的旧 ACK 会把 next
            //   推到 sent 之前。不钳位的话 uvFillWindow 里的 (sent-next) uint32 减法会下溢成
            //   ~42 亿，导致整轮一片不发（空转），严重时把 liveness 拖到超时判死。
            if (s->sent < s->next) s->sent = s->next;
            uvCwndGrow(s);                       // 加性增
            s->retry = 0; s->deadline = millis() + UV_STEP_TIMEOUT_MS;
            // 新腾出的窗口额度由下一轮 uvPump 的 uvFillWindow 补发
        }
        // ackNext <= next：重复/滞后 ACK，忽略
    } else if (s->state == UVS_WAIT_DONE) {
        if (ackNext < s->size) {                 // 发了 END 但对端仍缺（END 丢/收不全）→ 回退续发
            s->next = ackNext; s->sent = ackNext;
            s->state = UVS_SENDING; s->retry = 0; s->deadline = millis() + UV_STEP_TIMEOUT_MS;
        }
    }
}
static void uvOnDone(UvSession* s, uint32_t xferId) { if (s->state != UVS_IDLE && xferId == s->xferId) { s->lastRxMs = millis(); uvStop(s, "DONE", true);  } }
static void uvOnFail(UvSession* s, uint32_t xferId) { if (s->state != UVS_IDLE && xferId == s->xferId) uvStop(s, "对端 FAIL", false); }
// 对端忙/勿扰不是链路故障（对端明确表示"稍后再来"）→ 不计入重试次数，2s 后重排
static void uvOnBusy(UvSession* s, uint32_t xferId) { if (s->state != UVS_IDLE && xferId == s->xferId) uvStop(s, "对端忙/勿扰", false, UVR_YIELD); }

// 回程按 xferId 找活跃会话（只两个，直接比对）
static UvSession* uvByXfer(uint32_t xferId) {
    if (s_uvApp.state  != UVS_IDLE && s_uvApp.xferId  == xferId) return &s_uvApp;
    if (s_uvPeer.state != UVS_IDLE && s_uvPeer.xferId == xferId) return &s_uvPeer;
    return nullptr;
}


// ══════════════════════════════════════════════════════════════
//  伙伴语音【接收端】：本机作为接收方，收伙伴推来的语音（stop-and-wait ARQ over QoS0）
//    发送方 A → dev/<topicId(本机)>/voicePlay：PV_BEGIN → 数据帧(带 offset) → PV_END
//    本机确认  → dev/<topicId(A)>/cmd         ：PV_ACK{next} / PV_DONE / PV_FAIL / PV_BUSY
//  · 每片带 xferId+offset：按 offset 幂等落位（重复片不重写、乱序片不误写）。
//  · 收齐即播即删；两类超时：整条去重窗口、接收侧无活动超时(放弃半收态)。
//  发送端（本机→App / 本机→伙伴）已统一为上方的 UVA 引擎（s_uvApp/s_uvPeer），
//  与本接收端共用同一套 PV_* 帧格式；App 端也说这套（见 App voice_service.dart）。
// ══════════════════════════════════════════════════════════════
#define PV_CHUNK            4000      // 每片数据字节（+9 头 < 4096，远小于 MQTT 缓冲 8192）
#define PV_HDR              9         // 数据帧头：1(magic)+4(xferId,BE)+4(offset,BE)
#define PV_MAGIC            0xD1      // 数据帧魔数（≠ '{'=0x7B，用于与 JSON 控制帧区分）
#define PV_ACK_TIMEOUT_MS   1500      // 单步(片/BEGIN/END)等确认超时
#define PV_MAX_RETRY        6         // 单步最大重试次数
#define PV_TOTAL_TIMEOUT_MS 120000    // 整条传输总超时（最长 30s 语音=240KB=60 片，留足重试余量）
#define PV_RX_TIMEOUT_MS    15000     // 接收侧无活动超时（放弃半收态）
#define PV_RX_MAX_SIZE      (300*1024) // 接收 size 上界：语音最长 30s≈240KB，留余量到 300KB。
                                       // 超出 = 对端异常/协议不符 → 拒收，防按异常 size 一路写文件吃 flash。
// ★ 去重窗口必须覆盖【发送端的退避重投上限】(UV_LINK_BACKOFF 最大 300s)，否则会出现：
//   语音已送达 → PV_DONE 在回程丢了 → 发送端判失败 → 退避 120s/300s 后带【新 xferId】重投
//   → 接收端因窗口已过期而当成新消息 → 重复播放/重复冒泡。600s 留足余量。
#define PV_DEDUP_MS         600000    // 整条去重窗口
#define PV_PREEMPT_GUARD_MS 3000      // 活跃会话的抢占保护窗口（见 PV_BEGIN 处理）

#define PEER_VOICE_TMP   "/peer_voice.mp3.tmp"
#define PEER_VOICE_PATH  "/peer_voice.mp3"

// ── 发送端已合并为上方统一引擎（s_uvApp / s_uvPeer）；以下仅保留【接收端】：本机作为
//    伙伴语音的接收方（dev/<sn>/voicePlay），即播即删。──

// ── 接收端(B)状态机 ──
static File     s_pvRxFile;
static uint32_t s_pvRxXferId    = 0;       // 0 = 无接收会话
static char     s_pvRxAckTopic[64] = {0};  // dev/<topicId(from)>/cmd（回确认）
static uint32_t s_pvRxSize      = 0;
static uint32_t s_pvRxNext      = 0;       // 已连续收到字节 = 下一个期望 offset
static uint32_t s_pvRxLastMs    = 0;
static uint32_t s_pvRxDoneId    = 0;       // 最近完成的 xferId（会话级去重：挡同会话的重复 END）
static uint32_t s_pvRxDoneMs    = 0;
// ★ 内容级去重：xferId 每次重投都会换新，只按它去重挡不住【跨会话】的重复投递 ——
//   一旦 PV_DONE 在回程丢了，发送端退避后带新 xferId 重投，伙伴就会把同一条语音再播一遍。
//   uid 在整条消息的生命周期内稳定（发送端持久化在账本里），是跨会话唯一可信的内容身份。
//   （App 端 voice_service.dart 早就按 uid 去重，所以只有伙伴通路会重复播放。）
static uint32_t s_pvRxUid       = 0;       // 当前会话的内容 uid
static uint32_t s_pvRxDoneUid   = 0;       // 最近播完的内容 uid
static uint32_t s_pvRxDoneUidMs = 0;
// 接收侧诊断计数（每条会话开始时清零）
static uint32_t s_pvRxT0        = 0;       // 本条开始时刻
static uint32_t s_pvRxLogMs     = 0;       // 上次打进度的时刻（节流）
static uint32_t s_pvRxDup       = 0;       // 收到的重复片数（发送方在重传 = 我们的 ACK 丢了/慢）
static uint32_t s_pvRxGap       = 0;       // 收到的空洞片数（前面有片丢了）

// ── 接收端：回一条控制帧给发送方（next<0 表示不带 next 字段）──
static void pvRxTx(const char* msg, uint32_t xferId, int64_t next)
{
    if (!s_pvRxAckTopic[0]) return;
    StaticJsonDocument<96> d;
    d["msg"] = msg; d["xferId"] = xferId;
    if (next >= 0) d["next"] = (uint32_t)next;
    char b[96]; size_t n = serializeJson(d, b, sizeof(b));
    client.publish(s_pvRxAckTopic, (const uint8_t*)b, (unsigned int)n);
}
// 收伙伴推来的语音（ARQ）：控制帧走 JSON、数据帧走 [magic|xferId|offset|data]
static void handlePeerVoice(byte* payload, unsigned int length)
{
    if (length == 0) return;

    // ── 控制帧（JSON，以 '{' 开头）──
    if (payload[0] == '{') {
        StaticJsonDocument<160> d;
        if (deserializeJson(d, payload, length)) return;      // 非法 JSON → 丢
        const char* msg = d["msg"] | "";
        uint32_t xferId = d["xferId"] | 0u;

        if (strcmp(msg, "PV_BEGIN") == 0) {
            String from = String((const char*)(d["sn"] | ""));
            uint32_t size = d["size"] | 0u;
            String peer = Config.getString("peerSn", "");
            if (peer.isEmpty() || from != peer) {              // 只收自己已绑伙伴的语音
                LOG("[PVOICE] 拒收非伙伴语音 from=%s\n", from.c_str());
                return;
            }
            // 确认目标固定回伙伴（peer==from），会话期间缓存该主题
            snprintf(s_pvRxAckTopic, sizeof(s_pvRxAckTopic), "dev/%s/cmd", topicId(from).c_str());
            if (xferId == 0) return;                           // 非法 xferId
            uint32_t uid = d["uid"] | 0u;
            // 会话级去重：同一 xferId 刚播过 → 直接回 DONE（挡同会话的重复 BEGIN/END）
            if (xferId == s_pvRxDoneId && (uint32_t)(millis() - s_pvRxDoneMs) < PV_DEDUP_MS) {
                pvRxTx("PV_DONE", xferId, -1); return;
            }
            // ★ 内容级去重：同一 uid 刚播过（哪怕 xferId 换了）→ 回 DONE，不重播。
            //   这是"PV_DONE 丢了 → 发送端退避重投"时唯一能挡住重复播放的机制。
            if (uid != 0 && uid == s_pvRxDoneUid &&
                (uint32_t)(millis() - s_pvRxDoneUidMs) < PV_DEDUP_MS) {
                LOG("[PVOICE] 内容 uid=%08x 已播过（xferId 换成了 %u）→ 回 DONE，不重播\n",
                    (unsigned)uid, (unsigned)xferId);
                pvRxTx("PV_DONE", xferId, -1); return;
            }
            // ★★ 竞争修复：【同 xferId 的重复 BEGIN 必须幂等】——回当前进度，绝不重置。
            //    发送端在 RTT > UV_STEP_TIMEOUT_MS 或 BEGIN 的 ACK 丢失时会重发 BEGIN；
            //    broker 排队尖峰会让先发的那条【后到】，落在会话已开始落位之后。
            //    原来无条件 s_pvRxNext=0 + 截断临时文件 → 已收分片全丢，且此后接收端期望 0、
            //    发送端窗口左沿在 N，双方永久错位，一路重传到"重试超限"才收手。
            //    三端联合仿真实测：上行 35% 排队尖峰下伙伴收齐率 78%，加此保护后 100%。
            //    （App 端 voice_service.dart 早就有这个保护，所以 App 通路不受影响。）
            if (s_pvRxXferId != 0 && xferId == s_pvRxXferId) {
                s_pvRxLastMs = millis();
                pvRxTx("PV_ACK", xferId, s_pvRxNext);
                UVLOG1("[PVOICE] 重复 BEGIN xferId=%u → 幂等回 next=%u（不重置）\n",
                       (unsigned)xferId, (unsigned)s_pvRxNext);
                return;
            }
            // ★★ 抢占保护（规则2：只信任单调量）：
            //    xferId 是随机数，只有唯一性、没有顺序性 —— 收到一个【不同 xferId】的 BEGIN 时
            //    无法判断它是"新会话"还是"旧会话的迟到帧"。若无条件覆盖，一条被 broker 压了十几秒
            //    的旧 BEGIN 就能把正在进行的新会话整个清零。
            //    唯一可信的单调量是【时间】：当前会话若刚收过数据，说明它是活的，不允许被抢占。
            //    定向仿真：会话2 传到 86% 时被旧 BEGIN 打断 → 重试超限，总耗时 59.5s；
            //              加此保护后一次成功，17.4s。
            //    回 PV_BUSY 而不是静默丢弃：发起方按 UVR_YIELD 处理（不计失败次数，2s 后重排），
            //    真正的新会话在旧会话静默 PV_PREEMPT_GUARD_MS 后即可正常接管。
            if (s_pvRxXferId != 0 && xferId != s_pvRxXferId &&
                (uint32_t)(millis() - s_pvRxLastMs) < PV_PREEMPT_GUARD_MS) {
                LOG("[PVOICE] ⚠ 拒绝抢占：xferId=%u 想覆盖仍活跃的会话 %u"
                    "（%lums 前刚收过数据，进度 %u/%u）→ 回 PV_BUSY\n",
                    (unsigned)xferId, (unsigned)s_pvRxXferId,
                    (unsigned long)(millis() - s_pvRxLastMs),
                    (unsigned)s_pvRxNext, (unsigned)s_pvRxSize);
                pvRxTx("PV_BUSY", xferId, -1);
                return;
            }
            if (xferState != XFER_IDLE || isInDndPeriod()) {   // 忙/勿扰 → 让发送方退避
                LOG("[PVOICE] 忙或勿扰，回 PV_BUSY\n");
                pvRxTx("PV_BUSY", xferId, -1); return;
            }
            // ★ size 上界防护（只在真正开新会话前查，不干扰上面的去重/幂等/抢占分支）：
            //   超过 PV_RX_MAX_SIZE 或为 0 = 对端异常/协议不符 → 回 FAIL 拒收，
            //   避免按异常大小一路写文件吃 flash（size=0 在 END 也判不出收齐）。
            if (size == 0 || size > PV_RX_MAX_SIZE) {
                LOG("[PVOICE] 拒收异常 size=%u（上限 %u）→ 回 FAIL\n",
                    (unsigned)size, (unsigned)PV_RX_MAX_SIZE);
                pvRxTx("PV_FAIL", xferId, -1); return;
            }
            // 开新会话（覆盖任何旧半收态）
            if (s_pvRxFile) s_pvRxFile.close();
            LittleFS.remove(PEER_VOICE_TMP);
            s_pvRxFile = LittleFS.open(PEER_VOICE_TMP, FILE_WRITE);
            if (!s_pvRxFile) { pvRxTx("PV_BUSY", xferId, -1); return; }   // 打不开→让稍后重试
            s_pvRxXferId = xferId; s_pvRxSize = size; s_pvRxNext = 0; s_pvRxLastMs = millis();
            s_pvRxUid    = uid;                                // 记住内容身份，收齐时用于跨会话去重
            pvRxTx("PV_ACK", xferId, 0);                       // 就绪：从 offset 0 开始
            s_pvRxT0 = millis(); s_pvRxLogMs = 0; s_pvRxDup = 0; s_pvRxGap = 0;
            LOG("[PVOICE] ▶ 开始接收伙伴 %s 语音 xferId=%u size=%u → 回执主题 %s\n",
                from.c_str(), (unsigned)xferId, (unsigned)size, s_pvRxAckTopic);
            return;
        }
        if (strcmp(msg, "PV_END") == 0) {
            // 已完成的重复 END → 回 DONE（幂等）
            if (xferId != 0 && xferId == s_pvRxDoneId &&
                (uint32_t)(millis() - s_pvRxDoneMs) < PV_DEDUP_MS) {
                pvRxTx("PV_DONE", xferId, -1); return;
            }
            if (s_pvRxXferId == 0 || xferId != s_pvRxXferId) return;
            s_pvRxLastMs = millis();
            s_pvRxFile.close();
            if (s_pvRxSize > 0 && s_pvRxNext == s_pvRxSize) {  // 收齐 → 即播即删
                LittleFS.remove(PEER_VOICE_PATH);
                LittleFS.rename(PEER_VOICE_TMP, PEER_VOICE_PATH);
                audPlayFile(PEER_VOICE_PATH, true);
                s_pvRxDoneId = xferId; s_pvRxDoneMs = millis();
                if (s_pvRxUid) { s_pvRxDoneUid = s_pvRxUid; s_pvRxDoneUidMs = millis(); }
                s_pvRxXferId = 0; s_pvRxNext = 0; s_pvRxSize = 0;
                pvRxTx("PV_DONE", xferId, -1);
                LOG("[PVOICE] ✓ 收齐 %uB 用时 %lums（重复片 %u 空洞片 %u）→ 即播即删\n",
                    (unsigned)s_pvRxSize, (unsigned long)(millis() - s_pvRxT0),
                    (unsigned)s_pvRxDup, (unsigned)s_pvRxGap);
            } else {                                           // 理论不该发生（ack 驱动）→ 保守 FAIL
                LOG("[PVOICE] ✗ END 但收不全 %u/%u → FAIL\n",
                    (unsigned)s_pvRxNext, (unsigned)s_pvRxSize);
                LittleFS.remove(PEER_VOICE_TMP);
                s_pvRxXferId = 0; s_pvRxNext = 0; s_pvRxSize = 0;
                pvRxTx("PV_FAIL", xferId, -1);
            }
            return;
        }
        return;   // 未知控制帧
    }

    // ── 数据帧 [magic|xferId(BE)|offset(BE)|data] ──
    if (payload[0] != PV_MAGIC || length < PV_HDR) return;
    uint32_t xferId = ((uint32_t)payload[1] << 24) | ((uint32_t)payload[2] << 16) |
                      ((uint32_t)payload[3] << 8)  |  (uint32_t)payload[4];
    uint32_t offset = ((uint32_t)payload[5] << 24) | ((uint32_t)payload[6] << 16) |
                      ((uint32_t)payload[7] << 8)  |  (uint32_t)payload[8];
    if (s_pvRxXferId == 0 || xferId != s_pvRxXferId) {        // 无会话/不匹配 → 丢
        // 这条要打：发送方在发我们没有会话的数据（BEGIN 丢了 / 我们已超时放弃），
        // 是"伙伴收不到"最常见的现场表现。节流 1s 防刷屏。
        static uint32_t s_lastNoSessMs = 0;
        if (millis() - s_lastNoSessMs > 1000) {
            s_lastNoSessMs = millis();
            LOG("[PVOICE] ⚠ 收到数据片但无匹配会话 frameXfer=%u 本机会话=%u off=%u（BEGIN 丢失/已超时放弃？）\n",
                (unsigned)xferId, (unsigned)s_pvRxXferId, (unsigned)offset);
        }
        return;
    }
    s_pvRxLastMs = millis();
    uint32_t dlen = length - PV_HDR;
    uint32_t prevNext = s_pvRxNext;
    if (offset == s_pvRxNext) {                               // 期望片 → 顺序落位
        // ★ 越界防护：期望片但落位会超过声明的 size = 发送方的 size/片长不一致（协议异常）→ 拒收该片，
        //   不推进 next（收不齐会由 END 的 FAIL 分支或 RX 超时兜底）。用 64 位算和防 offset+dlen 回绕。
        if ((uint64_t)offset + dlen > s_pvRxSize) {
            LOG("[PVOICE] ⚠ 数据片越界 off=%u+%u > size=%u，拒收\n",
                (unsigned)offset, (unsigned)dlen, (unsigned)s_pvRxSize);
        } else {
            healthSetTaskStage(HT_WEB, HS_W_PEERRX);   // 写 LittleFS（会碰 flash）
            s_pvRxFile.seek(offset);
            size_t wr = s_pvRxFile.write(payload + PV_HDR, dlen);
            if (wr != dlen) {
                // ★ 写失败（flash 满/短写）→ 别静默推进 next 导致 END 误判"收齐"、播放损坏文件。
                //   放弃本会话、回 FAIL 让发送方重传（下次或许 flash 已腾出空间）。
                LOG("[PVOICE] ✗ 落盘写入失败(%u/%u B) off=%u（flash 满？）→ 放弃回 FAIL\n",
                    (unsigned)wr, (unsigned)dlen, (unsigned)offset);
                s_pvRxFile.close();
                LittleFS.remove(PEER_VOICE_TMP);
                uint32_t badId = s_pvRxXferId;
                s_pvRxXferId = 0; s_pvRxNext = 0; s_pvRxSize = 0;
                pvRxTx("PV_FAIL", badId, -1);
                return;
            }
            s_pvRxNext += dlen;
        }
    } else if (offset < s_pvRxNext) s_pvRxDup++;              // 重复片（发送方重传）
    else                            s_pvRxGap++;              // 空洞片（前面有片丢了）
    // I-C：同一 xferId 内 next 必须单调不减。发送端的 ACK 水位过滤依赖这条性质，
    //      一旦被破坏（例如将来有人在数据路径里加了重置逻辑），发送端会误丢有效 ACK。
    UV_INV(s_pvRxNext >= prevNext, "PEERRX next 回退 %u→%u xferId=%u",
           (unsigned)prevNext, (unsigned)s_pvRxNext, (unsigned)xferId);
    // offset<next：重复片(已写)不重写；offset>next：空洞不写。一律回当前 next（幂等 + 自纠正）
    pvRxTx("PV_ACK", xferId, s_pvRxNext);

    // 接收进度汇总（节流每秒一行）：卡住时能一眼看出停在哪个 offset、是重复片还是空洞在堆积
    if (millis() - s_pvRxLogMs >= 1000) {
        s_pvRxLogMs = millis();
        UVLOG1("[PVOICE] ← %u/%u (%u%%) 重复%u 空洞%u\n",
               (unsigned)s_pvRxNext, (unsigned)s_pvRxSize,
               (unsigned)(s_pvRxSize ? (uint64_t)s_pvRxNext * 100 / s_pvRxSize : 0),
               (unsigned)s_pvRxDup, (unsigned)s_pvRxGap);
    }
}

// webTask 每轮调用：接收侧无活动超时 → 放弃半收态（发送方掉线/静默时不永久卡）
static void checkPeerVoiceRxTimeout()
{
    if (s_pvRxXferId != 0 && (uint32_t)(millis() - s_pvRxLastMs) > PV_RX_TIMEOUT_MS) {
        LOG("[PVOICE] 接收超时，放弃半收态 xferId=%u\n", (unsigned)s_pvRxXferId);
        if (s_pvRxFile) s_pvRxFile.close();
        LittleFS.remove(PEER_VOICE_TMP);
        s_pvRxXferId = 0; s_pvRxNext = 0; s_pvRxSize = 0;
    }
}

// ══════════════════════════════════════════════════════════════
//  两个【完全独立】的通路调度器
//    不变量 I2：两者不读写对方的任何状态；共享的只有各自那一格账本。
//    任一通路的失败/放弃/对端离线，都不影响另一通路。
// ══════════════════════════════════════════════════════════════

// ── "为什么不发"日志：调度器每轮 5ms 跑一次，不能每次都打；只在【原因变化】时打一行。
//    这是新架构最需要的一条日志 —— 现场问题几乎都是"设备为什么不投递"，
//    有了它就能直接区分 "App 不在线" / "还有退避没到期" / "全都投完了" / "被下载抢占"。
//    用指针比较即可（传进来的都是同一批字面量）。
static const char* s_appBlockWhy  = nullptr;
static const char* s_peerBlockWhy = nullptr;
static inline void schedBlock(const char** slot, const char* who, const char* why) {
    if (*slot != why) { *slot = why; UVLOG1("[VM] %s 通路暂不投递：%s\n", who, why); }
}
static inline void schedClear(const char** slot) { *slot = nullptr; }

// 通路 A：设备 → App
static void appSched() {
    if (s_uvApp.state != UVS_IDLE) { schedClear(&s_appBlockWhy); return; }   // 会话在跑，交给 uvPump
    if (!client.connected())   { schedBlock(&s_appBlockWhy, "App",  "MQTT 未连接");   return; }
    if (!appDeliverable()) {                    // ★ App 不在 → 一片不发、一次握手都不做
        schedBlock(&s_appBlockWhy, "App",
                   appPresenceSubAddr[0] == 0 ? "terminal 未知(App 从未连过本机)"
                                              : "App 离线(presence=offline)");
        return;
    }
    uint32_t now = millis();
    for (int guard = 0; guard < VOICE_MAX; guard++) {
        int slot = vmPickNext(true, now);
        if (slot < 0) {
            // 区分"没东西发"和"有东西但都在退避里"——这两种在现场是完全不同的问题
            bool waiting = false;
            for (int i = 0; i < VOICE_MAX; i++)
                if (s_vm[i].used && s_vm[i].app == DST_PENDING) { waiting = true; break; }
            schedBlock(&s_appBlockWhy, "App", waiting ? "有待投但退避未到期" : "无待投语音");
            return;
        }
        schedClear(&s_appBlockWhy);
        s_vm[slot].app = DST_SENDING;
        if (uvStartApp(slot)) return;
        s_vm[slot].app = DST_PENDING;
        vmMarkBadSlot(slot);                    // 坏槽就地终结并删除，继续下一条（防永久堵塞）
    }
}

// 通路 B：设备 → 伙伴设备
static void peerSched() {
    if (s_uvPeer.state != UVS_IDLE) { schedClear(&s_peerBlockWhy); return; }
    if (!client.connected()) { schedBlock(&s_peerBlockWhy, "伙伴", "MQTT 未连接"); return; }
    if (!g_peerOnline)       { schedBlock(&s_peerBlockWhy, "伙伴", "伙伴离线(devInfo)"); return; }
    // ★ 双向确认：伙伴在线但其 devInfo.peerSn 已不回指本机（对端出厂/换绑）→ 不再推语音给它
    if (!g_peerAcksMe)       { schedBlock(&s_peerBlockWhy, "伙伴", "伙伴已不认本机(单向绑定)"); return; }
    // ★ 用 webTask 已维护的缓存 g_subPeer，不要在这里调 Config.getString ——
    //   本函数每轮 uvService 都会执行（语音活跃期 5ms/次），而 getString 每次都
    //   构造一个 String（堆 malloc+free）并进临界区。长期运行下就是持续的小块堆churn，
    //   与系统里的大块分配（I2S DMA 16KB / MQTT 缓冲 8KB / PSRAM 快照）混在一起会加剧碎片化。
    const String& peer = g_subPeer;
    if (peer.isEmpty())      { schedBlock(&s_peerBlockWhy, "伙伴", "未绑定伙伴"); return; }
    uint32_t now = millis();
    for (int guard = 0; guard < VOICE_MAX; guard++) {
        int slot = vmPickNext(false, now);
        if (slot < 0) {
            bool waiting = false;
            for (int i = 0; i < VOICE_MAX; i++)
                if (s_vm[i].used && s_vm[i].peer == DST_PENDING) { waiting = true; break; }
            schedBlock(&s_peerBlockWhy, "伙伴", waiting ? "有待投但退避未到期" : "无待投语音");
            return;
        }
        schedClear(&s_peerBlockWhy);
        s_vm[slot].peer = DST_SENDING;
        if (uvStartPeer(slot, peer)) return;
        s_vm[slot].peer = DST_PENDING;
        vmMarkBadSlot(slot);
    }
}

// ══════════════════════════════════════════════════════════════
//  语音子系统自检（每 60s）—— 兜住"没预料到的卡死"
//    前面的修复都是针对【已知】竞争；这里是最后一道防线：不假设自己想全了，
//    只检查"结构上不该出现的状态"，出现就地清理。全部是幂等的、代价极低的操作。
//    每一条都打日志：真机上出现即说明有未知路径把状态搞坏了，是排查线索。
// ══════════════════════════════════════════════════════════════
extern void healthBoxJson(char* out, size_t n);    // V1_1.cpp：健康黑匣子快照（MQTT 上报用）
extern volatile uint32_t g_selfHealHits;           // V1_1.cpp：自检命中计数（并入黑匣子）

static void voiceSelfCheck() {
    static uint32_t s_last = 0;
    if (millis() - s_last < 60000) return;
    s_last = millis();
    // 本轮命中数：任何一条触发都说明有【未预料】的路径把状态搞坏了。
    // 累加进黑匣子随 MQTT 上报 —— 串口看不到，但这个数字能跨崩溃带出来。
    uint32_t hits = 0;

    // ① 孤儿文件句柄：接收会话已结束，但 tmp 文件句柄还开着。
    //    LittleFS 可同时打开的文件数有限，泄漏几个就会让后续所有文件操作失败
    //    （录音存不下、语音读不出、配置写不进）——典型的"长时间运行后功能异常"。
    if (s_pvRxXferId == 0 && s_pvRxFile) {
        LOG("[VMCHK] ✗ 发现孤儿接收句柄（会话已结束但文件未关）→ 关闭\n");
        hits++;
        s_pvRxFile.close();
        LittleFS.remove(PEER_VOICE_TMP);
    }
    if (s_uvApp.state == UVS_IDLE && s_uvAppFile) {
        LOG("[VMCHK] ✗ 发现孤儿发送句柄（App 会话已空闲但文件未关）→ 关闭\n");
        hits++;
        s_uvAppFile.close();
    }
    // ② 孤儿 PSRAM 快照：伙伴会话已空闲却没释放（240KB/条，泄漏几次就吃光 PSRAM）
    if (s_uvPeer.state == UVS_IDLE && s_uvPeer.snap) {
        LOG("[VMCHK] ✗ 发现孤儿 PSRAM 快照 → 释放\n");
        hits++;
        free(s_uvPeer.snap); s_uvPeer.snap = nullptr;
    }
    // ③ 账本与会话不一致：某槽标着 SENDING，但没有任何会话在发它 → 永久卡住不再投递
    for (int i = 0; i < VOICE_MAX; i++) {
        if (!s_vm[i].used) continue;
        bool appLive  = (s_uvApp.state  != UVS_IDLE && s_uvApp.slot  == i);
        bool peerLive = (s_uvPeer.state != UVS_IDLE && s_uvPeer.slot == i);
        if (s_vm[i].app == DST_SENDING && !appLive) {
            LOG("[VMCHK] ✗ 槽 %d app=SENDING 但无对应会话 → 复位为待投递\n", i);
            hits++;
            s_vm[i].app = DST_PENDING; s_appNextMs[i] = 0;
        }
        if (s_vm[i].peer == DST_SENDING && !peerLive) {
            LOG("[VMCHK] ✗ 槽 %d peer=SENDING 但无对应会话 → 复位为待投递\n", i);
            hits++;
            s_vm[i].peer = DST_PENDING; s_peerNextMs[i] = 0;
        }
    }
    // ④ 账本记录了但文件已不在（异常删除/文件系统故障）→ 清记录，别让调度器空转
    for (int i = 0; i < VOICE_MAX; i++) {
        if (!s_vm[i].used) continue;
        if (voiceSlotBusy(i)) continue;                 // 在传的不动
        char p[24]; snprintf(p, sizeof(p), VOICE_PATH_FMT, i);
        if (!LittleFS.exists(p)) {
            LOG("[VMCHK] ✗ 槽 %d 有账本记录但文件不存在 → 清记录\n", i);
            hits++;
            memset(&s_vm[i], 0, sizeof(VoiceMeta));
            vmSave();
        }
    }
    // ⑤ 残留的临时文件：没有接收会话却留着 tmp（掉电/异常中断留下的），占 flash
    if (s_pvRxXferId == 0 && LittleFS.exists(PEER_VOICE_TMP)) {
        LOG("[VMCHK] 清理残留的接收临时文件\n");
        LittleFS.remove(PEER_VOICE_TMP);
    }
    if (hits) g_selfHealHits += hits;
}

// webTask 每轮调用：驱动两条独立通路。
static void uvService() {
    // ★ 新录音入账（micTask 只置标志，改账本这件事只在本任务做，见 voiceEnqueue）
    if (s_enqReq) { s_enqReq = false; vmRegisterNew(); }

    // 下载/接收期暂停语音外发（避免与下行分片抢 QoS0 带宽）。
    // ★ 改造前这里直接 return，在途会话被冻结 → LIVENESS 到点被误判成"对端无响应"而失败
    //   （一个 App 下载动作能同时打死两条语音链路）。现在主动优雅收手并标回 PENDING，
    //   用 UVR_YIELD 不计入失败次数，下载结束 2s 后自动续。
    if (xferState != XFER_IDLE) {
        if (s_uvApp.state  != UVS_IDLE) uvStop(&s_uvApp,  "让路下载", false, UVR_YIELD);
        if (s_uvPeer.state != UVS_IDLE) uvStop(&s_uvPeer, "让路下载", false, UVR_YIELD);
        g_voiceSending = false;
        return;
    }

    appSched();   uvPump(&s_uvApp);
    peerSched();  uvPump(&s_uvPeer);

    vmAgeTick();
    voiceSelfCheck();   // 每 60s 一次的兜底自检（孤儿句柄/快照、账本与会话不一致、残留文件）
    g_voiceSending = uvActive();   // 供 powerManagerLoop 保 CPU 满频 + applyWifiPowerSave 满功率
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

    // ★ 防自绑（方案C 设备端兜底）：peer==本机 SN 一律拒，杜绝任何客户端把设备绑到自己。
    //   自绑会让拍一拍/摇一摇/语音发给"自己"，触发自戳自播、状态错乱。App 侧也有拦截，
    //   这里是不依赖 App 版本的最后防线。
    if (peer == g_sn_str) {
        publishBindResult("BIND_REJECTED", "self", peer.c_str());
        LOG("[BIND] ✗ 拒绝 peer=%s：不能与本机绑定\n", peer.c_str());
        return;
    }

    // ① 授权：必须带对本机 matchCode（防陌生人凭 SN 猜绑）
    String localCode = Config.getString("matchCode", "");
    if (localCode.isEmpty() || code != localCode) {
        publishBindResult("BIND_REJECTED", "bad_code", peer.c_str());
        LOG("[BIND] ✗ 拒绝 peer=%s：matchCode 不符\n", peer.c_str());
        return;
    }

    // ② CAS 互斥（活性感知）：未绑 或 已绑同一个 → 接受；已绑他人 → 分情况：
    //    · 旧伙伴【在线且已不回指本机】(确证失效，如它出厂/换绑) → 允许新伙伴接管；
    //    · 其余（旧伙伴在线且互认 / 旧伙伴离线不可知）→ 拒（1:1 排他，保护真实绑定）。
    //    离线一律不自动放行：区分不了"已出厂"与"暂时断网"，据此放行会拆散真伙伴；
    //    对端永不上线的死锁由主人在 App 手动解绑(UNBIND)兜底。
    String cur = Config.getString("peerSn", "");
    if (!cur.isEmpty() && cur != peer) {
        bool oldConfirmedDead = g_peerOnline && !g_peerAcksMe;   // 在线才算确证失效
        if (!oldConfirmedDead) {
            publishBindResult("BIND_REJECTED", "busy", peer.c_str());
            LOG("[BIND] ✗ 拒绝 peer=%s：已绑 %s（旧伙伴仍有效或状态未知）\n",
                peer.c_str(), cur.c_str());
            return;
        }
        LOG("[BIND] 旧伙伴 %s 在线却已不认本机 → 确证失效，允许 %s 接管\n",
            cur.c_str(), peer.c_str());
    }

    Config.setString("peerSn", peer.c_str());
    Config.save();
    publishDevInfo("online");            // 重发 retained devInfo（含新 peerSn）
    publishBindResult("BIND_OK", nullptr, peer.c_str());
    LOG("[BIND] ✓ 已绑定 peer=%s\n", peer.c_str());
}

static void handleUnbind(JsonDocument& doc)
{
    // ★ 鉴权：解绑必须带对本机的 matchCode（与 handleBind 对齐）。原来无任何校验 =
    //   任何知道本机 topicId 的人都能发 UNBIND 强拆绑定（静默 DoS）；而"主人强制换绑"
    //   要走 UNBIND，更需要门槛。补齐后：只有持 matchCode 的人（有资格绑本机的人）能解绑。
    String code      = doc["matchCode"] | "";
    String localCode = Config.getString("matchCode", "");
    if (localCode.isEmpty() || code != localCode) {
        LOG("[BIND] ✗ UNBIND 拒绝：matchCode 不符\n");
        return;
    }
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
    char pl[128];
    serializeJson(d, pl, sizeof(pl));
    client.publish(topic, pl);
    LOG("[ACK] → %s : %s\n", topic, ackMsg);
}

// 学习 terminal（App 账号）——设备上行的唯一去向：TAP / NEW_VOICE / DEV_RECEIVED /
// PEER_ACK 全都发到 term/<terminal>/cmd。
//
// 以前它【只】在 VIDEO_SHOW 分支里写，于是一台新设备在用户第一次长按蛋发视频之前
// terminal 恒为空 —— 拍一拍不落爱心、录音不触发推送、送达回执也发不出。
//
// 规则（两条，缺一不可）：
//   ① terminal 为空  → 任何带 terminal 的命令都可以填上（补上首次那段空窗）；
//   ② terminal 非空  → 只有 VIDEO_SHOW（用户主动发内容）才改绑，沿用原语义。
//
// ★ 为什么不让所有命令都能改绑：PULL_VOICE 也带 terminal，而它是 App 每次回前台/重连
//   时向【所有好友设备】群发的。一台蛋被多个好友加了之后，各家 App 的轮询会把 terminal
//   来回顶，既让 TAP 发给谁变得随机，又会每次触发 Config.save() 擦写 flash
//   （_doc 还是 StaticJsonDocument<1024>，反复 setString 同一键要靠 save 里的
//    garbageCollect 回收）。所以后台轮询只允许"从无到有"，不允许"改绑"。
static void learnTerminal(const char* msg, JsonDocument& doc)
{
    String term = doc["terminal"] | "";
    if (term.isEmpty()) return;                  // 伙伴设备发来的 POKE/ACK 不带，直接跳过
    String cur = Config.getString("terminal", "");
    if (cur == term) return;                     // 没变，不写
    if (!cur.isEmpty() && strcmp(msg, "VIDEO_SHOW") != 0) return;   // 已有值 → 只认 VIDEO_SHOW

    Config.setString("terminal", term.c_str());
    Config.save();
    LOG("[WEB] terminal %s → %s（由 %s 触发）\n",
        cur.isEmpty() ? "(空)" : cur.c_str(), term.c_str(), msg);

    // ★ terminal 由无到有：把之前挂起的 NEW_VOICE 推送补发出去（否则新设备的首批语音
    //   既不推送、App 也不知道要拉）。账本里 app==DST_NA 的槽由 vmAgeTick 提回 PENDING。
    if (s_newVoicePend) {
        s_newVoicePend = false;
        publishCmdToTerminal(term, "NEW_VOICE");
        LOG("[VOICE] terminal 已学到，补发 NEW_VOICE 推送\n");
    }
}

static void dispatchCommand(const char* msg, JsonDocument& doc)
{
    learnTerminal(msg, doc);   // ★ 任何命令都可能补上 terminal（见上方注释的两条规则）

    // ── 设备互绑（P1）──
    if (strcmp(msg, "BIND") == 0)   { handleBind(doc);   return; }
    if (strcmp(msg, "UNBIND") == 0) { handleUnbind(doc); return; }

    // ── 统一语音 ARQ 回程（App 或伙伴发来）：按 xferId 路由到对应发送会话推进 ──
    //   App 用 mqtt_client 发的这些回程可为 QoS1（更稳）；伙伴用 PubSubClient=QoS0。引擎两者通吃。
    if (strcmp(msg, "PV_ACK") == 0 || strcmp(msg, "PV_DONE") == 0 ||
        strcmp(msg, "PV_FAIL") == 0 || strcmp(msg, "PV_BUSY") == 0) {
        uint32_t x = doc["xferId"] | 0u; uint32_t nx = doc["next"] | 0u;
        UvSession* s = uvByXfer(x);
        // 逐 ACK 细节只在 2 级打（60 片的传输会产生 60+ 条）
        UVLOG2("[UVDBG] ← %s xferId=%u next=%u → %s\n", msg, (unsigned)x, (unsigned)nx,
               s ? (s->kind == UV_APP ? "命中 App 会话" : "命中 伙伴会话") : "无匹配会话");
        // ★ 但"回程对不上任何会话"是真异常（对端在应答一个我们已经放弃的会话），必打。
        //   节流到 1s，避免对端狂发时刷屏。
        if (!s) {
            static uint32_t s_lastOrphanMs = 0;
            if (millis() - s_lastOrphanMs > 1000) {
                s_lastOrphanMs = millis();
                LOG("[UVDBG] ← %s xferId=%u ★无匹配会话，丢弃★ 当前: App{xfer=%u st=%d} 伙伴{xfer=%u st=%d}\n",
                    msg, (unsigned)x,
                    (unsigned)s_uvApp.xferId,  (int)s_uvApp.state,
                    (unsigned)s_uvPeer.xferId, (int)s_uvPeer.state);
            }
        }
        if (s) {
            if      (strcmp(msg, "PV_ACK")  == 0) uvOnAck(s, x, nx);
            else if (strcmp(msg, "PV_DONE") == 0) uvOnDone(s, x);
            else if (strcmp(msg, "PV_FAIL") == 0) uvOnFail(s, x);
            else                                  uvOnBusy(s, x);
        }
        return;
    }

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
            char pl[192];
            serializeJson(d, pl, sizeof(pl));
            client.publish(topic, pl);
        }
        // 拍一拍戳伙伴收到送达回执 → 通知 LCD 播 wink（表示"发送成功"）
        // ★ 必须用 MessageToMain_t：qWebToMain 的项大小是 sizeof(MessageToMain_t)=136，
        //   之前这里传的是 char[16]，xQueueSend 会照 136 字节拷贝（越界读栈），且 "WINK"
        //   落在结构体的 cmd/intVal 上、strVal 恒为空串 → 主循环永远匹配不上，wink 从不播。
        if (strcmp(msg, "POKE_ACK") == 0) {
            MessageToMain_t m = {};
            strcpy(m.strVal, "WINK");
            xQueueSend(qWebToMain, &m, 0);
            mainWake();          // 与其它 qWebToMain 生产者一致：L2 下立即唤醒主循环
        }
        LOG("[ACK] 收到伙伴回执 %s from=%s → 转 App\n", msg, from.c_str());
        return;
    }

    // ── POKE（P2）：伙伴拍了它的蛋 → 本机固定播放拍一拍(TAP)动画（不再播 target 自定义视频/音频）──
    if (strcmp(msg, "POKE") == 0) {
        if (xferState != XFER_IDLE) return;          // 传输中不打断（也不回执=发起方会超时提示忙）
        if (isInDndPeriod()) {                        // 勿扰时段：不播、不回 POKE_ACK（发起方超时）
            LOG("[DND] 勿扰时段，忽略伙伴 POKE\n");
            return;
        }
        MessageToMain_t toMain;
        strcpy(toMain.strVal, "POKE_TAP");           // 固定播放 TAP 点头动画（不复用 target 视频/音频）
        xQueueSend(qWebToMain, &toMain, 0);
        mainWake();
        sendPeerAck(doc["from"] | "", "POKE_ACK");   // ★ 送达回执给发起方
        LOG("[POKE] 收到伙伴拍一拍，播放 TAP 动画\n");
        return;
    }

    // ── PULL_VOICE：App 请求拉取本机已存语音 → uvService 逐条 ARQ 发出（成功=PV_DONE 删槽）──
    //   （旧的整条 VOICE_OK 确认已废弃，改由统一 ARQ 的 PV_ACK/PV_DONE 逐片确认，见上面回程路由）
    if (strcmp(msg, "PULL_VOICE") == 0) {
        // App 主动来拉 = 它此刻确实在线（比 presence 更强的即时证据）。
        // 清掉所有 App 通路的退避，让 appSched 下一轮立刻开传。
        g_appOnline = true; s_appPresenceSeen = true;
        for (int i = 0; i < VOICE_MAX; i++) s_appNextMs[i] = 0;
        LOG("[VOICE] 收到 App 拉取请求 → App 通路立即可投\n");
        return;
    }

    // ── NEW_AUDIO：音频直收进 600KB 音频池 ──
    if (strcmp(msg, "NEW_AUDIO") == 0) {
        g_curXferId = doc["xferId"] | 0;             // 记录本次会话 ID（END 时校验）
        g_declaredSize = doc["size"] | 0u;           // 应收字节（END 时精确比对，抓 QoS1 重复片）
        LOG("\n[AUDIO] 开始接收音频（直收 PSRAM）xferId=%u size=%u\n",
            (unsigned)g_curXferId, (unsigned)g_declaredSize);
        if (xferState == XFER_IDLE) enterTransferMode();
        size_t maxB = 0;
        g_recvBase = audRecvBegin(&maxB);
        g_recvMax  = maxB;
        resetRecvPerf(AUDIO_FILE_PATH);
        if (!g_recvBase) { receiveError = true; LOG("[AUDIO] ✗ 音频池不可用\n"); }
        xferState = XFER_RECEIVING_AUDIO;
        g_lastMqttRxMs = millis();          // ★ 刷新 RX 看门狗基准：排空落盘/等 READY 期间没收片，
                                            //   若沿用 NEW_ 回调早期的旧时间戳会被 10s 看门狗误判掉线回滚
        if (g_recvBase) sendXferReady();   // 接收态就绪(含排空落盘) → 通知 App 可以发分片了
        return;
    }

    // ── NEW_VIDEO：视频直收进 LCD 池 target 槽 ──
    if (strcmp(msg, "NEW_VIDEO") == 0) {
        g_curXferId = doc["xferId"] | 0;             // 记录本次会话 ID（END 时校验）
        g_declaredSize = doc["size"] | 0u;           // 应收字节（END 时精确比对，抓 QoS1 重复片）
        LOG("\n[VIDEO] 开始接收视频（直收 PSRAM）xferId=%u size=%u\n",
            (unsigned)g_curXferId, (unsigned)g_declaredSize);
        if (xferState == XFER_IDLE) enterTransferMode();
        size_t maxB = 0;
        g_recvBase = lcdTargetRecvBegin(&maxB);
        g_recvMax  = maxB;
        resetRecvPerf(videoPath);
        if (!g_recvBase) { receiveError = true; LOG("[VIDEO] ✗ LCD 池无空间\n"); }
        xferState = XFER_RECEIVING_VIDEO;
        g_lastMqttRxMs = millis();          // ★ 同 NEW_AUDIO：刷新 RX 看门狗基准，避免排空/等 READY 期间误判
        if (g_recvBase) sendXferReady();
        return;
    }

    // ── MATCH ──
    if (strncasecmp(msg, "MATCH-", 6) == 0) {
        handleMatchCommand(msg);
        return;
    }

    // ── 勿扰时段统一拦截：所有会"打扰"的远程动作/播放指令都不转发 Main ──
    //   → 不唤醒、不亮屏、不动舵机、不播放、不录音（POKE 在上方已单独按勿扰拦）。
    //   内容传输(NEW_AUDIO/NEW_VIDEO)仍照收照存(后台)；本地按键 / IMU 拍摇不经此路径，
    //   仍可正常唤醒设备 —— 勿扰只挡"远程打扰"，不挡本地交互。
    if (isInDndPeriod() &&
        (strcmp(msg, "VIDEO_SHOW") == 0 || strncasecmp(msg, "SERVO", 5) == 0 ||
         strcmp(msg, "RECORD") == 0    || strncasecmp(msg, "WBL", 3) == 0)) {
        LOG("[DND] 勿扰时段，拦截远程指令 %s（后台静默，不亮屏）\n", msg);
        return;
    }

    // ── VIDEO_SHOW（携带 term 字段）──
    if (strcmp(msg, "VIDEO_SHOW") == 0) {
        if (xferState != XFER_IDLE) {
            LOG("[WEB] ⚠ 传输中，屏蔽 VIDEO_SHOW\n");
            return;
        }

        // terminal 已在 dispatchCommand 入口由 learnTerminal() 统一学习并落盘，此处直接读用
        String terminal = Config.getString("terminal", "");
        if (terminal.isEmpty())
            LOG("[WEB] ⚠ terminal 为空（App 命令未带 terminal？），DEV_RECEIVED 发不出\n");
        publishCmdToTerminal(terminal, "DEV_RECEIVED");

        MessageToMain_t toMain;
        strcpy(toMain.strVal, "VIDEO_SHOW");
        xQueueSend(qWebToMain, &toMain, 0);
        mainWake();
        return;
    }

    // ── SERVO：走独立深度1覆盖队列(qServoCmd)——连发只保留最新角度，不占 qWebToMain
    //   (避免挤掉/排在 VIDEO_SHOW/RECORD 后面)，也不会因队列满而丢。传输中仍屏蔽。──
    if (strncasecmp(msg, "SERVO", 5) == 0) {
        if (xferState != XFER_IDLE) { LOG("[WEB] ⚠ 传输中，屏蔽: %s\n", msg); return; }
        MessageToMain_t toMain;
        strncpy(toMain.strVal, msg, sizeof(toMain.strVal) - 1);
        toMain.strVal[sizeof(toMain.strVal) - 1] = '\0';
        xQueueOverwrite(qServoCmd, &toMain);
        mainWake();
        return;
    }

    // ── 其它播放/控制指令：传输中屏蔽，否则转发 Main ──
    if (strcmp(msg, "RECORD") == 0 ||
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
//  进配网前的统一中止
//
//  为什么必须有这一步（实测事故复盘）：配网与在途传输并发时，内部 RAM 会在
//  20 秒内被三批高峰叠加压穿 ——
//    ① NimBLE 起来吃掉约 19KB（实测 42KB→23KB）
//    ② 配网结束要重连 MQTT（TLS 握手）+ 重建 I2S DMA（23KB→14KB）
//    ③ 在途传输此时必然因 App 断开而 10s 超时，超时回滚又触发
//       /target.mjpeg(约 4MB / 362 帧) 从 flash 重载，同时两条语音 ARQ 会话启动
//  实测最大连续块从 17KB 一路跌到 2KB，webTask(P0) 被 P1 的 LcdTask 压住
//  8 秒没喂上狗 → TWDT 重启（点名 WEBTask，但 backtrace 是 LcdTask 的）。
//
//  这里在【BLE 启动之前、MQTT 断开之前】主动收尾，收益有两层：
//    · 省内存：语音会话释放 PSRAM 快照与文件句柄，传输不再占着接收态
//    · 更关键的是【把回滚重载挪到安全窗口】——此刻 BLE 未起、语音已停、
//      MQTT 还活着，是这一整段里内存最宽松的时刻；而不是等它自己 10s 超时，
//      正好落在所有事情挤成一团的那一刻。
//
//  ★ 顺序要求：必须在 client.disconnect() 之前调用，FAIL 回执才发得出去 ——
//    App 收到就能立刻重传，不必干等自己那边超时。
// ══════════════════════════════════════════════════════════════
static void abortAllForProvision()
{
    // ① 在途音视频接收：先发 FAIL 再回滚（此刻 MQTT 还连着，App 收得到）
    //    守卫 xferState != XFER_IDLE 是硬要求：IDLE 时 rollbackAll() 会无条件回滚
    //    音频池（audRecvAbort → 从 flash 重载），而落盘是延后的，flash 里往往还是
    //    上一条 —— 会把刚收好的新音频覆盖成旧内容（同 ABORT_TRANSFER 处的教训）。
    if (xferState != XFER_IDLE) {
        // RECEIVING_AUDIO 阶段回 AUDIO_FAIL；WAIT_VIDEO / RECEIVING_VIDEO 阶段
        // 音频已回过 OK，App 正在发或准备发视频，故回 VIDEO_FAIL。
        const char* tag = (xferState == XFER_RECEIVING_AUDIO) ? "AUDIO" : "VIDEO";
        LOG("[PROV] ⚠ 进配网：中止在途 %s 接收（已收 %u B）→ 回 FAIL + 回滚\n",
            tag, (unsigned)g_recvLen);
        // ★ 先发 FAIL 再回滚，顺序不能反：rollbackAll → lcdTargetRecvAbort → waitTgtDone
        //   是【同步阻塞】的，要等 lcdTask 把 4MB 的 target 从 flash 重载完（实测 1398ms）
        //   才返回。放在后面的话，这条回执要压着一秒多才发得出去，白白拖长 App 的等待。
        //   publishResult 只发一条 JSON，不依赖 xferState，可以安全前置。
        publishResult(tag, false);
        rollbackAll();
        xferState   = XFER_IDLE;
        g_curXferId = 0;          // 清会话 ID：配网后若收到旧会话的残留 END，不误认
        exitTransferMode();
    }

    // ② 语音 ARQ 发送会话：按 YIELD 停，不按 FAIL ——
    //    这不是链路故障，不该计入失败次数触发退避（最长 300s）。
    //    YIELD 语义下调度器会在配网结束后自然重排，用户无感。
    if (s_uvApp.state  != UVS_IDLE) uvStop(&s_uvApp,  "进配网", false, UVR_YIELD);
    if (s_uvPeer.state != UVS_IDLE) uvStop(&s_uvPeer, "进配网", false, UVR_YIELD);

    // ③ 伙伴语音接收半收态：丢弃 + 删临时文件（发送方收不到 ACK 会自行重投）
    if (s_pvRxXferId != 0) {
        LOG("[PROV] ⚠ 进配网：丢弃伙伴语音半收态 %u/%u B\n",
            (unsigned)s_pvRxNext, (unsigned)s_pvRxSize);
        if (s_pvRxFile) s_pvRxFile.close();
        LittleFS.remove(PEER_VOICE_TMP);
        s_pvRxXferId = 0; s_pvRxNext = 0; s_pvRxSize = 0;
    }
}

// ══════════════════════════════════════════════════════════════
//  MQTT 回调
// ══════════════════════════════════════════════════════════════

// ── P3：收到伙伴姿态 → 映射到本机舵机（经 Main 的 SERVO 处理，单任务驱动舵机）──
static void handlePeerPos(byte* payload, unsigned int length)
{
    // 勿扰时段：不镜像伙伴摇摆（不动舵机、不亮屏）——这是独立于 dispatchCommand 的远程"打扰"路径。
    if (g_dndActive) return;
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
    xQueueOverwrite(qServoCmd, &toMain);   // 深度1覆盖：伙伴镜像高频姿态只留最新
    mainWake();
}

// ── 音频/视频接收收尾：校验 + 提交 + 回执。────────────────────────────────
//   抽成函数供两处调用：① 收到 END_ 控制帧；② 【已收满 g_declaredSize】(方案A)。
//   ★ 方案A：知道应收总字节，收满即提交回 OK，不再干等 END —— 慢链路尾部 END 到得慢/卡时
//     不再"收齐却无响应→App 超时→整条重传"。END 若随后再到，此时已 IDLE，会被忽略。
static void finishAudioReceive(const char* why)
{
    healthSetTaskStage(HT_WEB, HS_W_COMMIT);
    LOG("\n[AUDIO] 收尾(%s)\n", why);
    logXferPerf("AUDIO");
    bool sizeOk = (g_declaredSize == 0) || (g_recvLen == g_declaredSize);
    if (!sizeOk)
        LOG("[AUDIO] ✗ 大小不符 收=%u 应=%u（QoS1 重复片？）→ 判失败触发重传\n",
            (unsigned)g_recvLen, (unsigned)g_declaredSize);
    sendXferRecvd();                 // 提交前补一发心跳：commit 可能阻塞，先刷新 App 空闲计时
    uint32_t cmtT0 = millis();
    bool ok = (!receiveError && sizeOk && g_recvLen > 4 && audRecvCommit(g_recvLen));
    LOG("[AUDIO] 提交%s 收=%u/应=%u 用时%lums\n", ok ? "成功" : "失败",
        (unsigned)g_recvLen, (unsigned)g_declaredSize, (unsigned long)(millis() - cmtT0));
    if (ok) { g_audioSrc = g_recvBase; g_audioLen = g_recvLen; }  // 等 END_VIDEO 一并持久化
    else    { audRecvAbort(); g_audioSrc = nullptr; }
    publishResult("AUDIO", ok);
    xferState = XFER_WAIT_VIDEO;
    LOG("[XFER] 音频%s，等待 NEW_VIDEO...\n", ok ? "完成" : "失败");
}

static void finishVideoReceive(const char* why)
{
    healthSetTaskStage(HT_WEB, HS_W_COMMIT);   // 校验 + 建帧索引，可阻塞秒级
    LOG("\n[VIDEO] 收尾(%s)\n", why);
    logXferPerf("VIDEO");
    // 校验 MJPEG 头尾（直接在 PSRAM 上查）
    bool headTail = (g_recvLen > 4 &&
                     g_recvBase[0] == 0xFF && g_recvBase[1] == 0xD8 &&
                     g_recvBase[g_recvLen - 2] == 0xFF && g_recvBase[g_recvLen - 1] == 0xD9);
    bool sizeOk = (g_declaredSize == 0) || (g_recvLen == g_declaredSize);
    if (!sizeOk)
        LOG("[VIDEO] ✗ 大小不符 收=%u 应=%u（QoS1 重复片？）→ 判失败触发重传\n",
            (unsigned)g_recvLen, (unsigned)g_declaredSize);
    sendXferRecvd();                 // 提交前补一发心跳：建帧索引会阻塞回调，先刷新 App 空闲计时
    uint32_t cmtT0 = millis();
    bool ok = (!receiveError && sizeOk && headTail && lcdTargetRecvCommit(g_recvLen));
    LOG("[VIDEO] 提交%s 收=%u/应=%u 头尾=%d 用时%lums\n", ok ? "成功" : "失败",
        (unsigned)g_recvLen, (unsigned)g_declaredSize, headTail, (unsigned long)(millis() - cmtT0));
    if (ok) {
        // 内容已在 PSRAM，立即回 VIDEO_OK；落盘转后台延后（见 persistTask）。
        g_persistAllOk     = true;
        g_videoResultBytes = g_recvLen;
        if (g_audioSrc) { enqueuePersist(g_audioSrc, g_audioLen, AUDIO_FILE_PATH); g_audioSrc = nullptr; }
        enqueuePersist(g_recvBase, g_recvLen, videoPath);
        g_videoResultPend = true;
        publishResultBytes("VIDEO", true, g_recvLen);
        LOG("[VIDEO] 提交成功 → 立即回 VIDEO_OK，落盘延后\n");
    } else {
        LOG("[VIDEO] ✗ 校验/提交失败 (headTail=%d recvErr=%d) → 回滚\n", headTail, receiveError);
        rollbackAll();
        publishResult("VIDEO", false);
    }
    xferState = XFER_IDLE; exitTransferMode();
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    // 诊断用：收到任何入站消息的时刻。与传输存活时钟 g_lastMqttRxMs 严格分开 ——
    // 那个只能被本次传输自己的帧刷新（见 xferRxTouch），这个才是"网络还有没有动静"。
    g_lastAnyRxMs = millis();

    // ★ OTA 官方版本 Topic（retained，广播给所有设备）：比较版本号，更新则触发升级。
    //   放最前：命令很少、判断极快；且它可能立即 esp_restart，越早处理越干净。
    if (strcmp(topic, OTA_OFFICIAL_TOPIC) == 0) {
        otaOnOfficialVersion((const char*)payload, length);
        return;
    }

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
    // ★ 伙伴在线状态（devInfo）→ 决定配对爱心亮不亮 + 驱动伙伴语音通路
    if (peerInfoSubAddr[0] && strcmp(topic, peerInfoSubAddr) == 0) {
        StaticJsonDocument<256> d;
        if (!deserializeJson(d, payload, length)) {
            const char* st = d["status"] | "";
            bool was = g_peerOnline;
            g_peerOnline = (strcmp(st, "online") == 0);
            // ★ 双向确认：只在伙伴【在线】时判它的 peerSn 是否回指本机。
            //   离线的 retained 可能是连接时冻结的遗嘱，带的是换绑前的旧 peerSn，
            //   采信它会误判"对端还认我" → 故离线时不翻转，等它下次在线再定。
            //   containsKey 区分：字段缺失(旧固件→宽容认我) vs 字段为空(明确不认我)。
            if (g_peerOnline) {
                g_peerAcksMe = d.containsKey("peerSn")
                             ? (g_sn_str == (const char*)(d["peerSn"] | ""))
                             : true;
            }
            LOG("[PAIR] 伙伴%s%s\n", g_peerOnline ? "在线" : "离线",
                g_peerOnline ? (g_peerAcksMe ? "(互认)" : "(对端已不认我)") : "");
            if (!was && g_peerOnline) {          // 上升沿 → 清退避，积压立即投递
                for (int i = 0; i < VOICE_MAX; i++) s_peerNextMs[i] = 0;
                s_peerBlockWhy = nullptr;        // 让下一次阻塞原因重新打印
                vmDump("伙伴上线");
            }
        }
        return;
    }
    // ★ App 在线状态（term/<terminal>/presence，retained + LWT）→ 驱动 App 语音通路
    if (appPresenceSubAddr[0] && strcmp(topic, appPresenceSubAddr) == 0) {
        bool was = g_appOnline;
        g_appOnline = (length == 6 && memcmp(payload, "online", 6) == 0);
        s_appPresenceSeen = true;
        LOG("[VM] App %s\n", g_appOnline ? "上线" : "离线");
        if (!was && g_appOnline) {               // 上升沿 → 清退避，积压立即投递
            for (int i = 0; i < VOICE_MAX; i++) s_appNextMs[i] = 0;
            s_appBlockWhy = nullptr;             // 让下一次阻塞原因重新打印
            vmDump("App上线");
        }
        return;
    }
    // ★★ 这里【不再】刷新 g_lastMqttRxMs。
    //   它是【传输存活时钟】，唯一读者是下方的 RX 看门狗（用来判断"App 中途掉线"）。
    //   原来放在这里 = 只要收到【任何】到达本处的消息就刷新，而到达本处的包括：
    //     · dev/<tid>/cmd 上的任意命令 —— 摇摆时 10Hz 的 SERVO、30s 的 PULL_VOICE、
    //       伙伴语音 ARQ 的每个 PV_ACK（高频）…… 全都与本次传输无关
    //     · dev/<tid>/voicePlay 的伙伴语音数据帧 —— 完全是另一个子系统
    //   后果：传输卡死后只要还有别的流量，看门狗永远数不满 10s → xferState 永久停在
    //   RECEIVING_*，而 SERVO/VIDEO_SHOW/RECORD/WBL 全部命中 "传输中，屏蔽" 被静默丢弃。
    //   最恶劣的是自我维持：用户摇摆控件发出的 SERVO 既【被挡】又【在喂】那条本该解锁它的狗。
    //   现在改为只由【真正属于本次传输的帧】刷新，见下方 xferRxTouch() 的调用点。

    // ── 命令主题：整包 JSON ──
    if (strcmp(topic, subCmdAddr) == 0) {
        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, payload, length);
        if (err) {
            LOG("[WEB] ✗ 命令 JSON 解析失败: %s\n", err.c_str());
            return;
        }
        String msg = doc["msg"] | "";
        if (strlen(msg.c_str()) == 0) {
            LOG("[WEB] ✗ 命令缺少 msg 字段\n");
            return;
        }
        LOG("[WEB] 命令: %s\n", msg.c_str());
        dispatchCommand(msg.c_str(), doc);
        return;
    }

    // ── 音频数据主题 dev/$sn/audio ──
    else if (strcmp(topic, subAudioDataAddr) == 0) {
        char ctrl[32];
        bool isCtrl = tryParseControlMsg(payload, length, ctrl, sizeof(ctrl));
        xferRxTouch();   // ★ 本主题上的任何东西（分片/END/ABORT）都真属于本次传输 → 可以喂狗

        if (isCtrl && strcmp(ctrl, "END_AUDIO") == 0) {
            if (xferState != XFER_RECEIVING_AUDIO) {
                // App 没等到结果而重发 END：若是刚完成的会话 → 幂等重发缓存结果（不重复提交），
                //   让 App 可靠拿到 OK/FAIL（下行 QoS0 易丢，这是可靠交付结果的兜底）。
                uint32_t endId = xferIdOf(payload, length);
                if (endId != 0 && endId == g_lastResultXferId && g_lastResultPayload[0]) {
                    client.publish(responseTopic.c_str(), g_lastResultPayload);
                    LOG("[AUDIO] 重复 END(xferId=%u) → 重发缓存结果\n", (unsigned)endId);
                }
                return;
            }
            if (!xferIdMatches(payload, length)) {       // ★ 过期/错乱会话（如一端重启后残留 END）
                LOG("[AUDIO] ✗ xferId 不匹配，丢弃并回滚 + 回 FAIL\n");
                rollbackAll(); xferState = XFER_IDLE; exitTransferMode();
                publishResult("AUDIO", false);
                return;
            }
            finishAudioReceive("END");
            return;
        }
        if (isCtrl && strcmp(ctrl, "ABORT_TRANSFER") == 0) {
            // ★ 必须先确认确实在传输中。IDLE 时 rollbackAll() 会【无条件】回滚音频池
            //   （audRecvAbort → audTask 从 flash 重载），而落盘已延后到 L2 窗口，
            //   flash 里往往还是上一条 → 把刚收好的新音频覆盖成旧内容。
            //   App 在重传前会主动发 ABORT，那时设备可能早已完成并回到 IDLE。
            if (xferState == XFER_IDLE) {
                LOG("[XFER] 收到 ABORT 但当前无传输，忽略（避免误回滚音频）\n");
                return;
            }
            LOG("[XFER] ⚠ ABORT（音频通道）\n");
            rollbackAll();
            xferState = XFER_IDLE; exitTransferMode();
            return;
        }

        if (xferState == XFER_RECEIVING_AUDIO) {
            appendRecv(payload, length);
            // 方案A：已收满应收字节 → 立即收尾，不等 END（慢链路尾部 END 慢/卡也不会误判超时）
            if (xferState == XFER_RECEIVING_AUDIO &&
                g_declaredSize > 0 && g_recvLen == g_declaredSize)
                finishAudioReceive("收满");
        }
    }

    // ── 视频数据主题 ──
    else if (strcmp(topic, subVideoDataAddr) == 0) {
        char ctrl[32];
        bool isCtrl = tryParseControlMsg(payload, length, ctrl, sizeof(ctrl));
        xferRxTouch();   // ★ 本主题上的任何东西（分片/END/ABORT）都真属于本次传输 → 可以喂狗

        if (isCtrl && strcmp(ctrl, "END_VIDEO") == 0) {
            if (xferState != XFER_RECEIVING_VIDEO) {
                // App 没等到结果而重发 END：若是刚完成的会话 → 幂等重发缓存结果（不重复提交），
                //   让 App 可靠拿到 OK/FAIL（下行 QoS0 易丢，这是可靠交付结果的兜底）。
                uint32_t endId = xferIdOf(payload, length);
                if (endId != 0 && endId == g_lastResultXferId && g_lastResultPayload[0]) {
                    client.publish(responseTopic.c_str(), g_lastResultPayload);
                    LOG("[VIDEO] 重复 END(xferId=%u) → 重发缓存结果\n", (unsigned)endId);
                }
                return;
            }
            if (!xferIdMatches(payload, length)) {       // ★ 过期/错乱会话（如一端重启后残留 END）
                LOG("[VIDEO] ✗ xferId 不匹配，丢弃并回滚 + 回 FAIL\n");
                rollbackAll(); xferState = XFER_IDLE; exitTransferMode();
                publishResult("VIDEO", false);
                return;
            }
            finishVideoReceive("END");
            return;
        }
        if (isCtrl && strcmp(ctrl, "ABORT_TRANSFER") == 0) {
            // ★ 同音频通道：IDLE 时不可回滚，否则会把刚收好、尚未落盘的新内容
            //   覆盖成 flash 里的旧内容（App 重传前会发 ABORT，那时可能已 IDLE）。
            if (xferState == XFER_IDLE) {
                LOG("[XFER] 收到 ABORT 但当前无传输，忽略（避免误回滚）\n");
                return;
            }
            LOG("[XFER] ⚠ ABORT（视频通道）\n");
            rollbackAll();
            xferState = XFER_IDLE; exitTransferMode();
            return;
        }

        if (xferState == XFER_RECEIVING_VIDEO) {
            appendRecv(payload, length);
            // 方案A：已收满应收字节 → 立即收尾，不等 END（长视频尾部 END 慢/卡也不会误判超时）
            if (xferState == XFER_RECEIVING_VIDEO &&
                g_declaredSize > 0 && g_recvLen == g_declaredSize)
                finishVideoReceive("收满");
        }
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
    healthSetTaskStage(HT_WEB, HS_W_WIFI);   // 最长阻塞 30s（60×500ms）
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
    if (xfering || g_posStreaming || uvActive() || s_pvRxXferId != 0)
                                    want = WIFI_PS_NONE;        // 下载/串流/语音发送(App或伙伴)/收伙伴语音 满功率
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

    // ── 后台持久化任务（core 1）。落盘走 s_pslot 槽（延后到 L2 窗口写、唤醒可打断续写）。──
    xTaskCreatePinnedToCore(persistTask, "Persist", 4096, NULL, 1, &xPersistTask, 1);

    // ── 语音出站账本：加载 + 与实际文件对账（含老固件升级迁移、断电在途会话回退）──
    vmLoad();

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
                    startNtpSync();   // 线程安全；configTime 会触发 lwIP core lock 断言 panic
                    ntpStarted = true;
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

            // ★ 在降优先级、断 MQTT 之前中止所有在途传输/语音会话。
            //   放在 vTaskPrioritySet 之前是有意的：传输期 webTask 正持 P1，
            //   在这个优先级上把 FAIL 回执发出去最快；降到 P0 后会被 P1 的
            //   LcdTask（正在做回滚触发的 4MB 重载）压住，回执要等一秒多才发得出。
            abortAllForProvision();

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
                startNtpSync();   // 线程安全；configTime 会触发 lwIP core lock 断言 panic
                ntpStarted = true;
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
            // ★ 内部 RAM 单次峰值最大的地方：TLS 握手要几十 KB。若 lowBlk 快照里
            //   ts[HT_WEB]==HS_W_MQTT，说明内存是被握手吃掉的（对照 net.err=-2）。
            healthSetTaskStage(HT_WEB, HS_W_MQTT);
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
                client.subscribe(OTA_OFFICIAL_TOPIC, 1); // OTA：官方版本 Topic(retained)，一订阅即拿最新版本号
                publishDevInfo("online");
                // ★ 连接后补发 online：QoS0 首发若丢，retained 会停在遗嘱 offline（详见 ONLINE_REPUB 定义处）。
                s_onlineRepubLeft = ONLINE_REPUB_TIMES;
                s_onlineRepubAtMs = millis() + ONLINE_REPUB_STEP_MS;
                client.publish(pubAddr, "device_online", true);
                s_mqttFailRun = 0;                    // 连上了 → 清连续失败计数
            } else {
                // ── 记录失败现场（黑匣子读得到，串口读不到）──
                g_mqttLastErr = client.state();
                g_mqttFailCnt++;
                s_mqttFailRun++;
                g_mqttFailBlk = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
                LOG("[MQTT] ✗ 错误码: %d（连续 %u 次）内部RAM最大块 %uKB\n",
                    (int)g_mqttLastErr, (unsigned)s_mqttFailRun,
                    (unsigned)(g_mqttFailBlk / 1024));

                // ── 升级恢复：单纯重试 2s 是治不好持久性故障的 ──
                if (s_mqttFailRun == NET_RECOVER_FAILS) {
                    // 第一级：重连 WiFi。清掉 lwIP/DHCP 的残留状态 —— 这能修
                    // "WiFi 显示已连接但实际发不出包" 这类上层看不见的僵死。
                    g_netRecoverCnt++;
                    LOG("[NET] ✗✗ 连续 %d 次连不上 MQTT → 重连 WiFi（第 %u 次恢复）\n",
                        NET_RECOVER_FAILS, (unsigned)g_netRecoverCnt);
                    espClient.stop();
                    WiFi.disconnect(true);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    WiFi.mode(WIFI_STA);
                    // 下一轮循环开头的 WiFi.status() 检查会自动走 connectWiFiFromConfig()
                }
#if NET_REBOOT_FAILS > 0
                else if (s_mqttFailRun >= NET_REBOOT_FAILS) {
                    // 第二级：重启。★ 严格设防，宁可不救也不能打断用户正在用的功能：
                    //   · 播放/录音/传输中 → 不重启（等它做完，下一轮再判）
                    //   · 距上次网络重启不足 1 小时 → 不重启（防重启循环）
                    static uint32_t s_lastNetRebootMs = 0;
                    bool busy = lcdIsPlaying() || audIsPlaying() || micIsRecording() ||
                                xferState != XFER_IDLE || Config.isProvMode();
                    bool tooSoon = (s_lastNetRebootMs != 0) &&
                                   (millis() - s_lastNetRebootMs < NET_REBOOT_MIN_GAP);
                    if (!busy && !tooSoon) {
                        s_lastNetRebootMs = millis();
                        g_netRebootCnt++;
                        LOG("[NET] ✗✗✗ 连续 %u 次连不上，网络无法恢复 → 重启（第 %u 次）\n",
                            (unsigned)s_mqttFailRun, (unsigned)g_netRebootCnt);
                        Serial.flush();
                        vTaskDelay(pdMS_TO_TICKS(200));
                        esp_restart();
                    }
                }
#endif
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
                // 设备录了新语音 -> 发到 term/<账号>/cmd，触发离线推送（App 路径）。
                // ★ 投递本身已由账本 + 两个调度器驱动（voiceEnqueue 已把两路置成 PENDING），
                //   这里只负责"发推送通知"，不再直接启动任何会话。
                String terminal = Config.getString("terminal", "");
                LOG("[VOICE] 新语音，terminal=%s\n", terminal.c_str());
                if (terminal.isEmpty()) {
                    s_newVoicePend = true;    // terminal 还没学到 → 学到后补发（见 learnTerminal）
                    LOG("[VOICE] terminal 未知，推送挂起待补发\n");
                } else {
                    publishCmdToTerminal(terminal, "NEW_VOICE");
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

        // ── 一轮排空多个入站包 ──
        //   PubSubClient::loop() 每次只处理 1 个包；连发命令(如 SERVO 洪流)会 1 包/轮
        //   慢慢排、在 broker 侧积压并阻塞后面的播放指令。这里有上限地多跑几次，突发能
        //   快速排空。上限 8 防极端洪流饿死本循环其余工作(pos/落盘/心跳)；available()>0
        //   才继续 → 只在确有缓冲数据时多跑，readByte 不会空转忙等。
        for (int i = 0; i < 8; i++) {
            // ★ 必须在循环【内】喂狗：client.loop() 单次最坏会阻塞 setSocketTimeout(3) 整整
            //   3 秒——PubSubClient 的 readByte 在包只收到一半时会等剩余字节，而传输大分片
            //   时 TLS 半包是常态。本循环最多 8 次 ⇒ 最坏 24s，远超 TWDT 的 5s，实测已因此
            //   panic 过（"WEBTask (CPU 0) did not reset the watchdog"）。
            //   下面那句 available() 判断在两次调用【之间】，挡不住单次调用进去阻塞。
            //   这里喂狗是安全的：每次 client.loop() 都被 socketTimeout 有界兜住，不是死循环；
            //   真的 App 掉线由下方 XFER_RX_TIMEOUT_MS 那段接收看门狗负责判定。
            if (wdtSubscribed) esp_task_wdt_reset();
            healthSetTaskStage(HT_WEB, HS_W_IDLE);   // 进 client.loop：收包/回调都在里面
            uint32_t loopT0 = millis();
            bool alive = client.loop();
            // ★ IDLE0 饿死修复：本次 client.loop() 若卡在 readByte 的 socketTimeout 上（TLS 半包，
            //   可达数百 ms~3s），期间传输期 P1 的 webTask 一直占着 core0。yield() 只让给同级/更高
            //   优先级，不会让给更低的 IDLE0，而 TWDT(8s) 单独盯着 core0 IDLE0 —— 慢调用在一轮批处理
            //   里连发几次就把 IDLE0 饿过 8s → panic 点名 IDLE0（webTask 每轮已自喂，不会被点名，
            //   故排查时易看错方向）。这里检测到慢调用就 vTaskDelay(1) 阻塞 webTask 一个整 tick，
            //   调度器此刻能跑到 IDLE0 复位它的狗。正常快调用(<100ms)不触发，传输不降速。
            if (millis() - loopT0 >= WEB_LOOP_SLOW_MS) vTaskDelay(1);
            if (!alive) break;                       // 断连即止
            if (espClient.available() <= 0) break;   // 无更多缓冲数据即止
        }

        // ── 传输态绝对上限（最后一道保险，不看任何时钟）────────────────────────
        //   RX 看门狗依赖"存活时钟不被喂活"这个前提。万一将来又有新路径把它喂活了，
        //   这里保证设备最多 XFER_STATE_MAX_MS 就一定退出传输态，不会永久失去远程控制
        //   （xferState 非 IDLE 时 SERVO/VIDEO_SHOW/RECORD/WBL 全部被静默屏蔽）。
        if (xferState != XFER_IDLE) {
            if (g_xferStateSinceMs == 0) g_xferStateSinceMs = millis();
            else if ((int32_t)(millis() - g_xferStateSinceMs) >= XFER_STATE_MAX_MS) {
                LOG("[XFER] ✗✗ 传输态持续 %ds 未结束，强制复位（存活时钟被喂活？）\n",
                    XFER_STATE_MAX_MS / 1000);
                g_selfHealHits++;         // 计入黑匣子：出现即说明有未预料的喂狗路径
                const char* tag = (xferState == XFER_RECEIVING_AUDIO) ? "AUDIO" : "VIDEO";
                rollbackAll();
                xferState = XFER_IDLE;
                g_xferStateSinceMs = 0;
                exitTransferMode();
                publishResult(tag, false);
            }
        } else {
            g_xferStateSinceMs = 0;
        }

        // ── 传输接收看门狗：App 中途掉线（不再发分片/END/ABORT）时，避免设备永久
        //    卡在传输态 —— 那会让【所有远程指令】被 "传输中，屏蔽" 静默丢弃。
        //    g_lastMqttRxMs 现在只由本次传输自己的帧刷新（见 xferRxTouch 的调用点），
        //    不再被 SERVO/PULL_VOICE/伙伴语音等无关流量喂活。──
        if (xferState != XFER_IDLE &&
            (int32_t)(millis() - g_lastMqttRxMs) >= XFER_RX_TIMEOUT_MS) {
            // ★ 先记下当前阶段，回 FAIL 用（rollbackAll 后 xferState 会被清）。
            const char* rxTag = (xferState == XFER_RECEIVING_AUDIO) ? "AUDIO" : "VIDEO";
            LOG("[XFER] ✗ 接收超时 %ds 无分片，判 App 掉线 → 回 %s_FAIL + 回滚解锁\n",
                XFER_RX_TIMEOUT_MS / 1000, rxTag);
            rollbackAll();            // 依赖 xferState 判断是否回滚视频，必须先调
            xferState = XFER_IDLE;
            exitTransferMode();
            // ★ 明确回 FAIL（带会话 xferId + 走结果补发），别让 App 干等到自己 20s 超时才失败。
            //   若 App 已在等 ack → 立即失败并按 retriable 重传；仍在发分片则由 App 侧流控停滞兜底。
            publishResult(rxTag, false);
        }

        // ── 传输心跳：接收态下每 ~1.5s 补发一次 XFER_RECVD（进度+存活）。──
        //   慢链路(64KB 间隔可能 >20s)、接收尾部(不足 64KB)、等提交前 —— 都靠它撑住 App 的空闲计时，
        //   杜绝"大文件慢=误判超时"。提交(建帧索引)会阻塞本循环，故 END 前会再补一发(见 END_*)。
        if ((xferState == XFER_RECEIVING_AUDIO || xferState == XFER_RECEIVING_VIDEO) &&
            (uint32_t)(millis() - g_lastRecvdMs) >= XFER_HEARTBEAT_MS) {
            sendXferRecvd();
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
        //    ★ 节流到 2s：这两个块只是在【检测配置变化】（绑定/配网时才变，属用户操作级事件），
        //      但原来每轮都跑，而 Config.getString 每次都构造 String（堆分配 + 临界区）。
        //      语音活跃期本循环 5ms 一轮 ⇒ 每秒数百次无谓的堆操作。2s 检测一次对用户无感。
        static uint32_t s_lastBindChk = 0;
        if (millis() - s_lastBindChk >= 2000) {
            s_lastBindChk = millis();
            String peer = Config.getString("peerSn", "");
            if (peer != g_subPeer) {
                if (!g_subPeer.isEmpty()) {
                    char t[64];
                    snprintf(t, sizeof(t), "dev/%s/pos", topicId(g_subPeer).c_str()); client.unsubscribe(t);
                    snprintf(t, sizeof(t), "devInfo/%s", g_subPeer.c_str());     client.unsubscribe(t);
                }
                g_subPeer = peer;
                g_peerOnline = false;
                g_peerAcksMe = false;   // 新伙伴（或解绑）→ 等其 devInfo 重新确认双向
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
            // ── 跟踪 terminal 变化：订阅/退订 App 的 presence（App 语音通路的在线判据）──
            //    retained + LWT 由 broker 维护：App 进后台主动报 offline，崩溃/断网由遗嘱兜底。
            //    与上面的绑定跟踪同属"检测配置变化"，共用这个 2s 节流。
            String term = Config.getString("terminal", "");
            if (term != g_subTerminal) {
                if (appPresenceSubAddr[0]) client.unsubscribe(appPresenceSubAddr);
                g_subTerminal = term;
                g_appOnline = false;
                s_appPresenceSeen = false;
                if (!term.isEmpty()) {
                    snprintf(appPresenceSubAddr, sizeof(appPresenceSubAddr),
                             "term/%s/presence", term.c_str());
                    client.subscribe(appPresenceSubAddr, 1);   // retained → 一订阅即知当前态
                    s_appPresenceFirstMs = millis();
                    LOG("[VM] 订阅 App presence: %s\n", appPresenceSubAddr);
                } else {
                    appPresenceSubAddr[0] = 0;
                }
            }
        }
        // 已绑 且 伙伴在线 → 点亮配对爱心。★ 放在节流块【外】：g_peerOnline 由 devInfo 回调
        // 异步更新，爱心要跟手，不能等 2s 才刷。用缓存的 g_subPeer，不产生堆分配。
        g_pairLinked = (!g_subPeer.isEmpty()) && g_peerOnline && g_peerAcksMe;

        // ── P3：镜像超时（伙伴停止摇动）→ 舵机归中位一次 ──
        if (g_mirrorActive &&
            (int32_t)(millis() - g_mirrorLastRxMs) >= MIRROR_TIMEOUT_MS) {
            g_mirrorActive    = false;
            g_mirrorLastAngle = -1;
            MessageToMain_t toMain;
            strcpy(toMain.strVal, "SERVO87");   // 归中位（87°）
            xQueueOverwrite(qServoCmd, &toMain);
            mainWake();
            LOG("[MIRROR] 伙伴停止，舵机归中位\n");
        }

        // ── 后台落盘收尾：只记日志，★绝不在这里再发回执 ──
        //   VIDEO_OK 已在 END_VIDEO commit 成功时立即发出（内容那时已在 PSRAM 可播）。
        //   若这里补发：① 重复一条回执；② 落盘失败发出的 VIDEO_FAIL 会带上【当前】
        //   g_curXferId —— 万一 App 已开始新一轮传输，这条迟到的 FAIL 会通过会话校验，
        //   被误判成新传输失败。落盘失败只影响"掉电后退回旧内容"，不影响本次使用。
        if (g_videoResultPend && !persistIsPending()) {
            g_videoResultPend = false;
            LOG("[VIDEO] 后台落盘完成：%s（%uKB）\n",
                g_persistAllOk ? "成功" : "失败(PSRAM 仍可播，掉电会退回旧内容)",
                (unsigned)(g_videoResultBytes / 1024));
        }

        // ── 结果回执补发（防 QoS0 尾丢包）：随后几秒内间隔重发最近一次结果，共 RESULT_RESEND_TIMES 次。
        //   直到 App 收到并去重；新一轮传输 resetRecvPerf 会清零本状态，不会把陈旧回执发进新会话。──
        if (g_resultResendLeft > 0 && (int32_t)(millis() - g_resultResendAtMs) >= 0) {
            if (client.connected() && g_lastResultPayload[0])
                client.publish(responseTopic.c_str(), g_lastResultPayload);
            g_resultResendLeft--;
            g_resultResendAtMs = millis() + RESULT_RESEND_STEP_MS;
        }

        // ── online devInfo 连接后补发（防 QoS0 首发丢失把 retained 钉在遗嘱 offline，详见定义处）──
        if (s_onlineRepubLeft > 0 && (int32_t)(millis() - s_onlineRepubAtMs) >= 0) {
            if (client.connected()) publishDevInfo("online");
            s_onlineRepubLeft--;
            s_onlineRepubAtMs = millis() + ONLINE_REPUB_STEP_MS;
        }

        // ── 电量上报：电量变化≥2% 或 距上次≥60s → 重发 retained devInfo（App 好友栏显示电池）──
        //   retained：新上线的 App 一订阅 devInfo 即拿到最新电量，无需等下一次变化。
        {
            static uint32_t s_lastBatPubMs = 0;
            static int      s_lastBatPct   = -999;
            static int      s_lastChg      = -1;
            int bat = g_batPercent;   // 先读到普通 int，避免对 volatile 直接运算的告警
            int chg = g_batCharging ? 1 : 0;
            if (client.connected() &&
                (abs(bat - s_lastBatPct) >= 2 || chg != s_lastChg ||   // 充电插拔立即重发
                 (millis() - s_lastBatPubMs) > 60000)) {
                s_lastBatPct   = bat;
                s_lastChg      = chg;
                s_lastBatPubMs = millis();
                publishDevInfo("online");
            }
        }

        // ── 健康黑匣子上报（term/<sn>/health，retained）─────────────────────────
        //   串口不可用于长期监控：USB CDC 在 light-sleep 后就断开了。MQTT 在 light-sleep 下
        //   靠 DTIM 唤醒仍然工作，所以诊断走这条路出去。
        //   · 连上后【立即发一次】：把上次崩溃前的最后现场（复位原因、撑了多久、历史最差值）
        //     带出来 —— 崩溃那一刻多半来不及发，靠 RTC 内存跨重启保留、重启后补报。
        //   · 之后每 5 分钟一次：看碎片化/栈水位的长期走势。
        //   retained：随时用 mosquitto_sub 订阅一次就能拿到最新状态，不用一直挂着。
        {
            static uint32_t s_lastHealthMs = 0;
            static bool     s_healthBooted = false;
            if (client.connected() &&
                (!s_healthBooted || (millis() - s_lastHealthMs) >= 300000)) {
                s_healthBooted  = true;
                s_lastHealthMs  = millis();
                char topic[64];
                snprintf(topic, sizeof(topic), "term/%s/health", g_sn_str.c_str());
                healthSetTaskStage(HT_WEB, HS_W_REPORT);
                // 最坏 ~740B（所有字段取满宽 32 位十进制时）。960 留足余量：JSON 一旦被
                // snprintf 截断就整条解析不了，等于这次上报白发 —— 宁可多占 200 字节栈
                // （webTask 栈 10240，实测历史最小剩余 4952，撑得住）。
                char body[960];
                healthBoxJson(body, sizeof(body));
                client.publish(topic, (const uint8_t*)body, strlen(body), true);
                LOG("[HB] → %s %s\n", topic, body);
            }
        }

        // ── 统一语音 ARQ 发送引擎：触发 App 会话 + 推进 App/Peer 两会话（逐片确认可靠传）──
        uvService();

        // ── 接收侧（本机作为伙伴语音接收方）无活动超时兜底 ──
        checkPeerVoiceRxTimeout();

        // ── 勿扰态缓存（每 2s 刷新）：供 powerManagerLoop 判断"勿扰时下载不亮屏"。
        //    isInDndPeriod 内部 getLocalTime 有阻塞，故节流，不每轮调用。──
        {
            static uint32_t s_lastDndMs = 0;
            if (millis() - s_lastDndMs > 2000) {
                s_lastDndMs = millis();
                g_dndActive = isInDndPeriod();
            }
        }

        // ── WiFi 省电分级（传输满功率 / L1/L2 最省 / 仍按 DTIM 收下行）──
        applyWifiPowerSave();

        // ── 轮询节奏按状态调整：传输全速；WiFi 活跃保持低延迟；空闲慢轮询省电 ──
        // ── 轮询节奏：传输全速 / 串流低延迟 / L2 拉长喂 light-sleep / L0L1 低延迟 ──
        TickType_t pollTicks;
        if      (xfering)            pollTicks = 1;                          // 传输：全速
        else if (uvActive() || s_pvRxXferId != 0) pollTicks = pdMS_TO_TICKS(5); // 语音 ARQ 收发：低延迟（逐片往返快）
        else if (g_posStreaming)     pollTicks = pdMS_TO_TICKS(5);           // 串流：低延迟
        else if (g_pwrTier == 2)     pollTicks = pdMS_TO_TICKS(WEB_L2_POLL_MS); // L2：省电（推送仍≤1s）
        else                         pollTicks = pdMS_TO_TICKS(100);         // L0/L1：低延迟

        if (wdtSubscribed) esp_task_wdt_reset();   // 喂狗：稳态每轮一次（传输期 pollTicks=1，卡在 client.loop 内 >5s 才会点名）
        vTaskDelay(pollTicks);
    }
}