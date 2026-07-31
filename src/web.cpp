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
#include "esp_task_wdt.h"       // 任务级看门狗：稳态纳入监控，配网/连WiFi 等长阻塞前退订

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

// 后台持久化（PSRAM → flash）
typedef struct { const uint8_t* src; size_t len; char path[64]; } PersistJob_t;
static QueueHandle_t     qPersist        = nullptr;
static TaskHandle_t      xPersistTask     = nullptr;
volatile int             g_persistPending = 0;   // 在途持久化作业数（含仍在排队等窗口的）。
                                                 // 用途：排空屏障 + 判断要不要开 L2 落盘窗口。
                                                 // ★ 不代表"正在写 flash"——降载判据请用 g_persistWriting。
// ★ 计数跨任务读改写：webTask 加、persistTask 减。volatile int 的 ++/-- 非原子，
//   两核并发可能算错 → waitPersistIdle 永久等待。统一走临界区封装保证原子。
static portMUX_TYPE      s_pendMux = portMUX_INITIALIZER_UNLOCKED;
// 读到普通 int 再写回，避免对 volatile 直接 ++/--（-Wvolatile 弃用告警）；原子性由临界区保证。
static inline void pendInc()   { portENTER_CRITICAL(&s_pendMux); int v = g_persistPending; g_persistPending = v + 1;              portEXIT_CRITICAL(&s_pendMux); }
static inline void pendDec()   { portENTER_CRITICAL(&s_pendMux); int v = g_persistPending; if (v > 0) g_persistPending = v - 1;    portEXIT_CRITICAL(&s_pendMux); }
static inline void pendReset() { portENTER_CRITICAL(&s_pendMux); g_persistPending = 0;                                             portEXIT_CRITICAL(&s_pendMux); }
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
//  调度策略：默认【延后到「落盘窗口」再写】，而不是收完就写。
//    原因：flash 写会经 IPC 冻结 cache，与播放并发会掉帧 + 断音。而内容 commit 后
//    已在 PSRAM 里可直接播放，落盘只为"掉电不丢"，属非关键后台路径 —— 拖长用户无感，
//    卡顿用户却立刻能感知。所以先回 VIDEO_OK 让用户马上能播，落盘挑没人看的时候做。
//
//  窗口何时开（persistForceAcquire，引用计数，可并存）：
//    ① 进 L2 放锁前 —— 主路径。屏幕已休眠、IMU 已进 WoM、播放已停、各任务已降频，
//       但还没进 light-sleep，是"最空闲却仍全速"的黄金时机（见 V1_1.cpp powerManagerLoop）
//    ② 新传输前（waitPersistIdle）—— PSRAM 要被复用，旧数据必须先写完
//    ③ 长按关机前（persistFlushBlocking）—— 否则关机丢掉刚收到的视频
//    ④ 兜底超时 PERSIST_DEFER_MAX_MS —— 防"用户一直交互→永不进 L2→永不落盘"
//
//  ★ 忙闲判据 persistBusy() 故意不含 g_xfering：传输期本就已停播，且新传输前要靠
//    开窗强制排空；若含进来会造成"落盘等传输结束、传输的 waitPersistIdle 等落盘"死锁。
// ══════════════════════════════════════════════════════════════
#define PERSIST_DEFER_MAX_MS  90000   // 兜底：单个作业最多延后这么久，之后无条件写完。
                                      // 防"用户一直交互→永不进 L2→永不落盘"，给掉电丢失窗口封顶。

// 「落盘窗口」引用计数：>0 = 现在可以全速落盘。
// 多个调用方可并存（进 L2 前落盘 / 新传输前排空 / 关机前排空），故用计数而非布尔——
// 用布尔时一方 release 会误关另一方的窗口（例：waitPersistIdle 正在排空，
// powerManagerLoop 恰好因不在 L2 而清标志 → 落盘停摆 → 排空超时 → PSRAM 被覆盖写坏文件）。
static portMUX_TYPE s_forceMux  = portMUX_INITIALIZER_UNLOCKED;
static volatile int s_forceRefs = 0;

// 中止标志：排空超时后置位，通知 persistTask 立刻放弃当前作业。
// ★ 必须有：排空超时会 pendReset() 放行新传输去覆盖 PSRAM，而 persistTask 可能仍在读
//   同一块 PSRAM 往 flash 写 → 写出新旧混合的损坏文件，下次开机加载就出问题。
//   置位后 persistTask 走失败路径删掉 .tmp、保留 flash 里的原文件（宁可不更新，不可写坏）。
static volatile bool s_persistAbort = false;

// persistTask 是否【真正在写 flash】。lcd.cpp 据此降帧给 core0 让路。
// ★ 不能用 g_persistPending > 0 代替：落盘改成延后到 L2 窗口后，作业会在队列里排队
//   好几分钟，pending 全程 >0 却根本没在写 —— 用它判会让 LCD 从传输完成起一直低帧率。
volatile bool g_persistWriting = false;

void persistForceAcquire() { portENTER_CRITICAL(&s_forceMux); int v = s_forceRefs; s_forceRefs = v + 1;              portEXIT_CRITICAL(&s_forceMux); }
void persistForceRelease() { portENTER_CRITICAL(&s_forceMux); int v = s_forceRefs; if (v > 0) s_forceRefs = v - 1;   portEXIT_CRITICAL(&s_forceMux); }
bool persistIsPending()    { return g_persistPending > 0; }

static inline bool persistForced() { return s_forceRefs > 0; }

// 设备是否正忙（用户可感知的活动）。仅在「窗口未开」时用来让路，避免万一在 L0/L1
// 被兜底超时唤醒落盘时撞上播放。★不含 g_xfering：否则会与新传输前的排空互等成死锁。
static inline bool persistBusy()
{
    return lcdIsPlaying() || audIsPlaying() || micIsRecording() ||
           g_posStreaming || g_voiceSending;
}

static void persistTask(void* p)
{
    LOG("[PST] 持久化任务启动 (core %d)\n", xPortGetCoreID());
    PersistJob_t job;
    while (true) {
        if (xQueueReceive(qPersist, &job, portMAX_DELAY) != pdTRUE) continue;

        uint32_t jobT0 = millis();   // 本作业入手时刻（兜底超时基准）
        bool abandon  = false;       // true = 源 PSRAM 即将/已被覆盖，放弃本作业

        // ★ 等「落盘窗口」：默认按兵不动，直到以下任一条件成立——
        //   ① 窗口打开（进 L2 放锁前 / 新传输前 / 关机前）
        //   ② 兜底超时（防用户一直交互导致永不进 L2、掉电窗口无上限）
        //   ③ 收到中止（排空超时已放行新传输覆盖 PSRAM）→ 放弃，别写坏文件
        //   等待期间设备可自由播放，完全不受落盘干扰。
        if (job.src && job.len > 0) {
            bool waited   = false;
            bool byWindow = true;    // 退出原因在循环内定夺，避免事后再读 persistForced() 时
            while (true) {           // 窗口恰好已关而误报"兜底超时"
                if (s_persistAbort)                               { abandon  = true;  break; }
                if (persistForced())                              { byWindow = true;  break; }
                // ★ 兜底超时到点，但若仍在播动画则继续等——不在渲染时写 flash，杜绝卡顿（成因A）。
                //   持续播放会把落盘一直推后（掉电窗口相应变大，属可接受取舍）；一旦停播或窗口开
                //   (进 L2 / 强排空)就照常放行。
                if ((millis() - jobT0) >= PERSIST_DEFER_MAX_MS &&
                    !lcdIsPlaying())                              { byWindow = false; break; }
                if (!waited) { waited = true; LOG("[PST] ⏳ 等落盘窗口（进 L2 / 新传输 / 关机时开）\n"); }
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            if (abandon)
                LOG("[PST] ✗ 等窗口时收到中止，放弃 %s（源即将被覆盖）\n", job.path);
            else if (waited)
                LOG("[PST] ▶ 开始落盘（%s）\n", byWindow ? "窗口已开" : "兜底超时");
        }

        if (!abandon && job.src && job.len > 0) {
            String tmp = String(job.path) + ".tmp";
            File f = LittleFS.open(tmp.c_str(), FILE_WRITE);
            if (!f) {
                LOG("[PST] ✗ 打开失败 %s\n", tmp.c_str());
                g_persistAllOk = false;
            } else {
                uint32_t t0 = millis();
                size_t off = 0; bool ok = true;
                bool deferred = false;
                g_persistWriting = true;    // ★ 从这里开始真正碰 flash（LCD 据此降帧让路）
                while (off < job.len) {
                    // ★ 写到一半窗口关了（如落盘期间用户拍一拍唤醒设备回 L0 并开始播放）：暂停让路。
                    // ★★ 防卡顿关键（成因A/B）：可延后的落盘只要在播动画(lcdIsPlaying)就【一律让路】，
                    //    连"兜底超时"也不越过——写 flash 冻结 core0 cache 会卡住渲染。这覆盖用户主诉的
                    //    "快速切换时卡顿"（此时落盘多是 90s 兜底/常态触发，persistForced()=false）。
                    //  但【强制窗口】(persistForced：进 L2 / 关机)不受此让路约束、照写：
                    //    · L2 窗口——已息屏，无动画可卡；
                    //    · 关机排空(persistFlushBlocking)——走 persistForceAcquire()，数据安全优先，照写。
                    //  新传输不再走这里排空——改为 cancelPersistForReuse() 直接取消上轮落盘。
                    //  ★ s_persistAbort：被取消时立刻跳出让路、去下面的中止分支收尾（cancel 更跟手）。
                    //  其余忙态(音频/录音/串流)沿用原判据：窗口没开且没到兜底才让路。
                    while ( !s_persistAbort &&
                            ( (lcdIsPlaying() && !persistForced()) ||
                              (!persistForced() && persistBusy() &&
                               (millis() - jobT0) < PERSIST_DEFER_MAX_MS) ) ) {
                        if (!deferred) {
                            deferred = true;
                            g_persistWriting = false;   // 让路期间没在写，别让 LCD 白降帧
                            LOG("[PST] ⏸ 设备转忙（播放中），落盘让路...\n");
                        }
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    if (deferred) {
                        deferred = false;
                        g_persistWriting = true;        // 恢复写 → 重新降帧让路
                        LOG("[PST] ▶ 继续落盘（已写 %uKB/%uKB）\n",
                            (unsigned)(off / 1024), (unsigned)(job.len / 1024));
                    }
                    // ★ 摊薄落盘：flash 写/擦会经 IPC 冻结 core0 cache（spi_flash_op_block_func
                    //   自旋），单次擦大量 sector 冻结 >8s → 饿死 core0 触发 TWDT。
                    //   关键：不能只调 write 大小——LittleFS 会把多次小写攒在缓冲、最后一次性
                    //   flush 成大块擦写，仍长冻结。必须【每 4KB 写完立即 f.flush() 强制小步落盘】，
                    //   把每次 flash 操作压到 ~1 个 sector（冻结数十 ms），再 vTaskDelay 让 core0
                    //   恢复喂狗。总落盘时间被拉长（后台非关键路径，可接受）。
                    // ★ 排空超时已放行新传输覆盖 PSRAM → 立刻中止，别把混合数据写进去
                    if (s_persistAbort) {
                        LOG("[PST] ✗ 收到中止（源 PSRAM 即将被覆盖），放弃本次落盘\n");
                        ok = false; break;
                    }
                    size_t n = job.len - off; if (n > 4096) n = 4096;
                    size_t w = f.write(job.src + off, n);
                    if (w != n) { ok = false; break; }
                    off += w;
                    f.flush();                       // ★ 逼 LittleFS 立即小步落盘，避免攒批大 flush
                    // 让路延时：强制窗口(排空/L2/关机)时播放已停、屏已冻结/息屏，无渲染要保护 →
                    // 降到 5ms 加速排空(缩短下一条传输的 READY 等待)；非强制(90s 兜底,可能在播)仍 15ms 保渲染。
                    vTaskDelay(pdMS_TO_TICKS(persistForced() ? 5 : 15));
                }
                f.flush(); f.close();
                g_persistWriting = false;   // ★ 写完（含失败/中止路径），解除 LCD 降帧

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

        pendDec();
    }
}

static void enqueuePersist(const uint8_t* src, size_t len, const char* path)
{
    PersistJob_t job; job.src = src; job.len = len;
    strncpy(job.path, path, sizeof(job.path) - 1); job.path[sizeof(job.path) - 1] = 0;
    pendInc();
    if (xQueueSend(qPersist, &job, 0) != pdTRUE) {
        pendDec();
        g_persistAllOk = false;
        LOG("[PST] ⚠ 持久化队列满，跳过 %s\n", path);
    }
}

// 复用同一片 PSRAM 前，【取消】上轮延后的落盘（不再排空写完——那要 ~20s，正是连发时"等 READY"的元凶）。
//   依据："后一条覆盖前一条"，被取代的旧内容无需持久化；只要确保 persistTask 停手、并清掉队列里
//   尚未取出的作业，避免它读到已被新数据覆盖的 PSRAM、把损坏内容写进 flash。
//   代价：被取代的【中间那条】不进 flash（掉电/重启不留）；最后一条会在自己 END 时正常落盘。
//   耗时：仅等 persistTask 结束当前那 4KB 写并收尾（~50-100ms），而非整条排空。
static void cancelPersistForReuse()
{
    if (g_persistPending <= 0) return;
    uint32_t t0 = millis();
    LOG("[PST] 取消上轮在途落盘（%d 个），立即复用 PSRAM\n", g_persistPending);
    s_persistAbort = true;                 // 令 persistTask 放弃当前作业：不 rename .tmp，不写坏旧文件
    while (g_persistPending > 0 && millis() - t0 < 2000) {   // 等它收尾(~100ms)，2s 兜底防异常
        vTaskDelay(pdMS_TO_TICKS(10));
        esp_task_wdt_reset();
    }
    // ★ 清掉队列里【尚未取出】的作业：音视频是两个作业，取消了正在写的那个，另一个仍排队；
    //   新传输开始后它会被取出、其 src 指向已被覆盖的 PSRAM → 照样写坏文件。只 pendReset 清计数不够。
    {
        PersistJob_t drop;
        while (xQueueReceive(qPersist, &drop, 0) == pdTRUE)
            LOG("[PST] 丢弃排队落盘 %s（源即将被覆盖）\n", drop.path);
    }
    s_persistAbort = false;
    pendReset();
    LOG("[PST] 上轮落盘已取消，用时 %lums\n", (unsigned long)(millis() - t0));
}

// 阻塞排空（关机前调用）：开窗全速写完，避免长按关机丢掉刚收到的视频。
void persistFlushBlocking(uint32_t timeoutMs)
{
    if (g_persistPending <= 0) return;
    LOG("[PST] 关机前排空落盘（%d 个在途）...\n", g_persistPending);
    persistForceAcquire();       // persistForced()=true → persistTask 不受"播放让路"约束，照写（数据安全优先）
    uint32_t t0 = millis();
    while (g_persistPending > 0 && (millis() - t0) < timeoutMs) {
        vTaskDelay(pdMS_TO_TICKS(20));
        esp_task_wdt_reset();
    }
    persistForceRelease();
    LOG("[PST] 关机前排空%s\n", g_persistPending > 0 ? "超时（未写完，掉电会退回旧内容）" : "完成");
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

static void enterTransferMode()
{
    g_xfering = true;           // 传输期：IMU 降采样（见 imu.cpp）；LCD 据此【冻结当前帧】：
                                // 不解码 MJPEG、不画转圈 → 接收方无感，且 core0 全让给 webTask 收包，
                                // 修"边播视频边下载→视频极慢+下载失败"（见 lcd.cpp 的 g_xfering 分支）。
    // ★ 先发 XFER_LOCK 停播，再取消上轮落盘复用 PSRAM。
    LOG("[XFER] ── 锁定：停止播放 ──\n");
    MessageToMain_t msg; strcpy(msg.strVal, "XFER_LOCK");
    xQueueSend(qWebToMain, &msg, 0);
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
    xQueueSend(qWebToMain, &msg, 0);
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
//  语音消息发送（设备→App，阶段D）
//    App 发 PULL_VOICE → 设备把 voice_N.mp3 逐条发到 term/<sn>/voice
//    （VOICE_BEGIN json → 二进制分片 → VOICE_END json）；
//    收到 App 的 VOICE_OK(带 id) 后删除该文件，再发下一条。
// ══════════════════════════════════════════════════════════════
static volatile bool g_pullVoiceReq  = false;
static volatile bool g_voiceAcked    = false;
static volatile int  g_voiceAckId    = -1;
// 非 static：mic.cpp 环形淘汰最旧槽时要读它，跳过"当前正被 App 发送"的槽（volatile：跨任务读）
volatile int    g_voiceSendingId     = -1;      // 正在发/等 ack 的槽，-1=空闲
static uint32_t g_voiceAckDeadline   = 0;

// 每个语音槽的唯一 id（0=未分配）。槽号(voice_0..)会被复用，无法区分"新语音"和"同一条重发"；
// 这个 uid 随 VOICE_BEGIN 发给 App 做去重键，槽被 ack 删除时清零 → 下条新语音拿到全新 uid。
static uint32_t s_slotUid[VOICE_MAX] = {0};
static uint32_t slotUid(int slot) {
    if (slot < 0 || slot >= VOICE_MAX) return 0;
    if (s_slotUid[slot] == 0) s_slotUid[slot] = esp_random() | 1u;   // |1 确保非 0
    return s_slotUid[slot];
}

// 供 mic.cpp 环形淘汰复用槽时调用：清掉该槽残留的旧 uid，确保新录语音拿到全新 uid，
// 避免 App 端按 uid 去重把新语音误当成旧语音的重发而丢弃（去重窗口 60s 内才会撞，但要根治）。
void voiceSlotResetUid(int slot) {
    if (slot >= 0 && slot < VOICE_MAX) s_slotUid[slot] = 0;
}

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
    b["uid"]  = slotUid(slot);          // 不复用的唯一 id：App 用它去重（取代复用的槽号）
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

// ══════════════════════════════════════════════════════════════
//  P4 伙伴语音：应用层逐片确认可靠传输（stop-and-wait ARQ over QoS0）
//    发送 A → dev/<topicId(B)>/voicePlay：PV_BEGIN → 数据帧(带 offset) → PV_END
//    确认 B → dev/<topicId(A)>/cmd       ：PV_ACK{next} / PV_DONE / PV_FAIL / PV_BUSY
//  · A 收到 B 对 offset O 的 PV_ACK{next=O+L} 才发下一片 → 天然限速到 B 的处理速度，
//    从源头避免"慢订阅→broker 丢 QoS0"；残余丢片靠单片超时重发兜住。
//  · 每片带 xferId+offset：B 按 offset 幂等落位（重复片不重写、乱序片不误写）。
//  · 三类超时：单步 ack 超时(重发)、整条总超时、接收侧无活动超时(放弃半收态)。
//  App 路径(term/<sn>/voice, sendVoiceFileTo)完全不动，此处是独立的设备↔设备协议。
// ══════════════════════════════════════════════════════════════
#define PV_CHUNK            4000      // 每片数据字节（+9 头 < 4096，远小于 MQTT 缓冲 8192）
#define PV_HDR              9         // 数据帧头：1(magic)+4(xferId,BE)+4(offset,BE)
#define PV_MAGIC            0xD1      // 数据帧魔数（≠ '{'=0x7B，用于与 JSON 控制帧区分）
#define PV_ACK_TIMEOUT_MS   1500      // 单步(片/BEGIN/END)等确认超时
#define PV_MAX_RETRY        6         // 单步最大重试次数
#define PV_TOTAL_TIMEOUT_MS 120000    // 整条传输总超时（最长 30s 语音=240KB=60 片，留足重试余量）
#define PV_RX_TIMEOUT_MS    15000     // 接收侧无活动超时（放弃半收态）
#define PV_DEDUP_MS         60000     // 整条去重窗口（同 xferId 已播过则不重播）

#define PEER_VOICE_TMP   "/peer_voice.mp3.tmp"
#define PEER_VOICE_PATH  "/peer_voice.mp3"

// ── 发送端(A)状态机 ──
enum PvSendState { PVS_IDLE, PVS_WAIT_READY, PVS_SEND, PVS_WAIT_ACK, PVS_WAIT_DONE };
static PvSendState s_pvsState   = PVS_IDLE;
static String   s_pvsPeer;                 // 伙伴 SN（仅日志）
static char     s_pvsTopic[64]  = {0};     // dev/<topicId(peer)>/voicePlay
static uint32_t s_pvsXferId     = 0;
static uint8_t* s_pvsData       = nullptr; // PSRAM：整条语音快照（发送期不依赖文件句柄，App 路径可随时删该槽）
static uint32_t s_pvsSize       = 0;
static uint32_t s_pvsNext       = 0;       // 当前在途片 offset（= B 期望的下一个 offset）
static uint8_t* s_pvsFrame      = nullptr; // PSRAM：[magic|xferId|offset|data]
static uint32_t s_pvsFrameLen   = 0;       // 当前 frame 总字节（含头）——超时原样重发
static uint32_t s_pvsDeadline   = 0;
static int      s_pvsRetry      = 0;
static uint32_t s_pvsStartMs    = 0;

// ── 接收端(B)状态机 ──
static File     s_pvRxFile;
static uint32_t s_pvRxXferId    = 0;       // 0 = 无接收会话
static char     s_pvRxAckTopic[64] = {0};  // dev/<topicId(from)>/cmd（回确认）
static uint32_t s_pvRxSize      = 0;
static uint32_t s_pvRxNext      = 0;       // 已连续收到字节 = 下一个期望 offset
static uint32_t s_pvRxLastMs    = 0;
static uint32_t s_pvRxDoneId    = 0;       // 最近完成的 xferId（整条去重）
static uint32_t s_pvRxDoneMs    = 0;

// P4 传输进行中（发送或接收）——webTask 据此把轮询提速到 5ms、WiFi 拉满功率，
// 否则 L2(500ms 轮询 + WiFi 省电 DTIM)下逐片往返会很慢，长语音甚至撞总超时。
static inline bool peerVoiceActive() { return s_pvsState != PVS_IDLE || s_pvRxXferId != 0; }

// App 语音上传路径是否在忙（拉取中或正发某条等 ack）——与 P4 协调 g_xferAnim：
// 录音时两路(发 App + 推伙伴)常并发，只有对方也空闲才打勾，否则留转圈给对方收尾，避免闪烁。
static inline bool appVoiceBusy() { return g_pullVoiceReq || g_voiceSendingId >= 0; }

// ── 发送端：组一片数据帧并发出（offset 处读 PV_CHUNK 字节）。返回 false=读文件失败 ──
static bool pvsSendChunk(uint32_t offset)
{
    if (!s_pvsData || !s_pvsFrame || offset > s_pvsSize) return false;
    uint32_t remain = s_pvsSize - offset;
    uint32_t want   = (remain < (uint32_t)PV_CHUNK) ? remain : (uint32_t)PV_CHUNK;
    s_pvsFrame[0] = PV_MAGIC;
    s_pvsFrame[1] = (uint8_t)(s_pvsXferId >> 24); s_pvsFrame[2] = (uint8_t)(s_pvsXferId >> 16);
    s_pvsFrame[3] = (uint8_t)(s_pvsXferId >> 8);  s_pvsFrame[4] = (uint8_t)(s_pvsXferId);
    s_pvsFrame[5] = (uint8_t)(offset >> 24);      s_pvsFrame[6] = (uint8_t)(offset >> 16);
    s_pvsFrame[7] = (uint8_t)(offset >> 8);       s_pvsFrame[8] = (uint8_t)(offset);
    memcpy(s_pvsFrame + PV_HDR, s_pvsData + offset, want);
    s_pvsFrameLen = PV_HDR + want;
    client.publish(s_pvsTopic, s_pvsFrame, s_pvsFrameLen);
    return true;
}
static void pvsSendBegin()
{
    StaticJsonDocument<128> d;
    d["msg"] = "PV_BEGIN"; d["xferId"] = s_pvsXferId; d["sn"] = g_sn_str; d["size"] = s_pvsSize;
    char b[128]; size_t n = serializeJson(d, b, sizeof(b));
    client.publish(s_pvsTopic, (const uint8_t*)b, (unsigned int)n);
}
static void pvsSendEnd()
{
    StaticJsonDocument<64> d; d["msg"] = "PV_END"; d["xferId"] = s_pvsXferId;
    char b[64]; size_t n = serializeJson(d, b, sizeof(b));
    client.publish(s_pvsTopic, (const uint8_t*)b, (unsigned int)n);
}
static void pvsStop(const char* why, bool ok)
{
    if (s_pvsData) { free(s_pvsData); s_pvsData = nullptr; }
    s_pvsState = PVS_IDLE;
    // LCD 收尾（与 App 上传路径协调）：只有 App 也不忙才由本路径定妆——
    //   成功→打勾；失败→撤掉转圈。若 App 仍在忙则保持现状(转圈)，等它收尾统一打勾，避免闪烁。
    if (!appVoiceBusy()) {
        if (ok)                       g_xferAnim = 2;   // 打勾（drawXferAnim 播完自动回 0）
        else if (g_xferAnim == 1)     g_xferAnim = 0;   // 失败且当前是本路径的转圈 → 撤掉
    }
    LOG("[PVOICE] 发送%s（%s）xferId=%u\n", ok ? "完成" : "中止", why, (unsigned)s_pvsXferId);
}

// NEW_VOICE 触发：向已绑伙伴可靠推送某槽语音。忙/伙伴离线/打不开则跳过（不影响 App 路径）。
static void startPeerVoiceSend(int slot, const String& peer)
{
    if (s_pvsState != PVS_IDLE) { LOG("[PVOICE] 上一条仍在发，跳过本条\n"); return; }
    if (xferState != XFER_IDLE) { LOG("[PVOICE] 下载/传输中，暂不推送伙伴语音\n"); return; } // 避免与下载抢资源
    if (peer.isEmpty()) return;
    if (!g_peerOnline) { LOG("[PVOICE] 伙伴离线，不推送\n"); return; }   // 预检，省一轮 BEGIN 重试
    char path[24]; snprintf(path, sizeof(path), VOICE_PATH_FMT, slot);
    File f = LittleFS.open(path, FILE_READ);
    if (!f) { LOG("[PVOICE] 打不开槽 %d\n", slot); return; }
    uint32_t sz = f.size();
    if (sz == 0) { f.close(); return; }
    // 一次性快照整条语音进 PSRAM，随即关文件：发送期(数秒)不再持有文件句柄，
    // App 路径 handleVoiceSend 收 ack 后删同一槽也不会读到已删的打开文件（同由 NEW_VOICE 触发，会重叠）。
    uint8_t* data = (uint8_t*)ps_malloc(sz);
    if (!data) { f.close(); LOG("[PVOICE] ✗ PSRAM 分配失败(%u)\n", (unsigned)sz); return; }
    int rd = f.read(data, sz);
    f.close();
    if (rd < 0 || (uint32_t)rd != sz) { free(data); LOG("[PVOICE] ✗ 读槽 %d 不全\n", slot); return; }
    if (!s_pvsFrame) {
        s_pvsFrame = (uint8_t*)ps_malloc(PV_HDR + PV_CHUNK);
        if (!s_pvsFrame) { free(data); LOG("[PVOICE] ✗ PSRAM 分配失败\n"); return; }
    }
    s_pvsData   = data;
    s_pvsPeer   = peer;
    snprintf(s_pvsTopic, sizeof(s_pvsTopic), "dev/%s/voicePlay", topicId(peer).c_str());
    s_pvsXferId = esp_random() | 1u;   // 非 0
    s_pvsSize   = sz;
    s_pvsNext   = 0;
    s_pvsRetry  = 0;
    s_pvsStartMs = millis();
    s_pvsDeadline = millis() + PV_ACK_TIMEOUT_MS;
    s_pvsState  = PVS_WAIT_READY;
    g_xferAnim  = 1;                    // LCD 转圈（推伙伴中）；powerManagerLoop 据此保持亮屏
    pvsSendBegin();
    LOG("[PVOICE] ▶ 向伙伴 %s 发送槽 %d（xferId=%u size=%u）\n",
        peer.c_str(), slot, (unsigned)s_pvsXferId, (unsigned)sz);
}

// webTask 每轮调用：推进发送状态机（超时重发 / 发下一片 / 总超时兜底）
static void pumpPeerVoiceSend()
{
    if (s_pvsState == PVS_IDLE) return;
    uint32_t now = millis();
    if (!client.connected())                                  { pvsStop("断链", false); return; }
    if ((int32_t)(now - (s_pvsStartMs + PV_TOTAL_TIMEOUT_MS)) >= 0) { pvsStop("总超时", false); return; }

    switch (s_pvsState) {
        case PVS_SEND:                       // ack 推进后由回调置此态：发下一片 / 或收尾发 END
            if (s_pvsNext >= s_pvsSize) {
                pvsSendEnd();
                s_pvsState = PVS_WAIT_DONE; s_pvsRetry = 0; s_pvsDeadline = now + PV_ACK_TIMEOUT_MS;
            } else if (!pvsSendChunk(s_pvsNext)) {
                pvsStop("读文件失败", false);
            } else {
                s_pvsState = PVS_WAIT_ACK; s_pvsRetry = 0; s_pvsDeadline = now + PV_ACK_TIMEOUT_MS;
            }
            break;
        case PVS_WAIT_READY:                 // 等 BEGIN 的就绪确认
        case PVS_WAIT_ACK:                   // 等当前片确认
        case PVS_WAIT_DONE:                  // 等整条 DONE/FAIL
            if ((int32_t)(now - s_pvsDeadline) >= 0) {
                if (++s_pvsRetry > PV_MAX_RETRY) { pvsStop("重试超限", false); break; }
                if      (s_pvsState == PVS_WAIT_READY) pvsSendBegin();
                else if (s_pvsState == PVS_WAIT_ACK)   client.publish(s_pvsTopic, s_pvsFrame, s_pvsFrameLen);
                else                                    pvsSendEnd();
                s_pvsDeadline = now + PV_ACK_TIMEOUT_MS;
            }
            break;
        default: break;
    }
}

// ── 发送端：收到 B 的确认（在 dispatchCommand 里调用，webTask 同任务，无需加锁）──
static void pvOnAck(uint32_t xferId, uint32_t next)
{
    if (s_pvsState == PVS_IDLE || xferId != s_pvsXferId) return;
    if (s_pvsState == PVS_WAIT_READY) {          // BEGIN 就绪：从 B 期望的 next 开始发
        s_pvsNext = next; s_pvsState = PVS_SEND;
    } else if (s_pvsState == PVS_WAIT_ACK) {
        if (next > s_pvsNext) { s_pvsNext = next; s_pvsState = PVS_SEND; }  // 有进展→发下一片
        // next <= s_pvsNext：重复/滞后 ack，忽略（超时会重发当前片）
    } else if (s_pvsState == PVS_WAIT_DONE) {
        if (next < s_pvsSize) { s_pvsNext = next; s_pvsState = PVS_SEND; }  // B 仍缺数据→回退续发
    }
}
static void pvOnDone(uint32_t xferId) { if (s_pvsState != PVS_IDLE && xferId == s_pvsXferId) pvsStop("DONE", true);  }
static void pvOnFail(uint32_t xferId) { if (s_pvsState != PVS_IDLE && xferId == s_pvsXferId) pvsStop("FAIL", false); }
static void pvOnBusy(uint32_t xferId) { if (s_pvsState != PVS_IDLE && xferId == s_pvsXferId) pvsStop("伙伴忙/勿扰", false); }

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
            // 整条去重：刚播过同一条 → 直接回 DONE（不重播）
            if (xferId == s_pvRxDoneId && (uint32_t)(millis() - s_pvRxDoneMs) < PV_DEDUP_MS) {
                pvRxTx("PV_DONE", xferId, -1); return;
            }
            if (xferState != XFER_IDLE || isInDndPeriod()) {   // 忙/勿扰 → 让发送方退避
                LOG("[PVOICE] 忙或勿扰，回 PV_BUSY\n");
                pvRxTx("PV_BUSY", xferId, -1); return;
            }
            // 开新会话（覆盖任何旧半收态）
            if (s_pvRxFile) s_pvRxFile.close();
            LittleFS.remove(PEER_VOICE_TMP);
            s_pvRxFile = LittleFS.open(PEER_VOICE_TMP, FILE_WRITE);
            if (!s_pvRxFile) { pvRxTx("PV_BUSY", xferId, -1); return; }   // 打不开→让稍后重试
            s_pvRxXferId = xferId; s_pvRxSize = size; s_pvRxNext = 0; s_pvRxLastMs = millis();
            pvRxTx("PV_ACK", xferId, 0);                       // 就绪：从 offset 0 开始
            LOG("[PVOICE] 开始接收伙伴 %s 语音 xferId=%u size=%u\n",
                from.c_str(), (unsigned)xferId, (unsigned)size);
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
                s_pvRxXferId = 0; s_pvRxNext = 0; s_pvRxSize = 0;
                pvRxTx("PV_DONE", xferId, -1);
                LOG("[PVOICE] ✓ 收齐 → 即播即删\n");
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
    if (s_pvRxXferId == 0 || xferId != s_pvRxXferId) return;  // 无会话/不匹配 → 丢
    s_pvRxLastMs = millis();
    uint32_t dlen = length - PV_HDR;
    if (offset == s_pvRxNext) {                               // 期望片 → 顺序落位
        s_pvRxFile.seek(offset);
        s_pvRxFile.write(payload + PV_HDR, dlen);
        s_pvRxNext += dlen;
    }
    // offset<next：重复片(已写)不重写；offset>next：空洞不写。一律回当前 next（幂等 + 自纠正）
    pvRxTx("PV_ACK", xferId, s_pvRxNext);
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

// webTask 每轮调用：驱动语音发送状态机
static void handleVoiceSend() {
    if (xferState != XFER_IDLE) return;   // 与 app→device 传输互斥
    if (!client.connected()) return;

    if (g_voiceSendingId >= 0) {          // 正在等某条的 ack
        if (g_voiceAcked && g_voiceAckId == g_voiceSendingId) {
            char p[24]; snprintf(p, sizeof(p), VOICE_PATH_FMT, g_voiceSendingId);
            LittleFS.remove(p);
            if (g_voiceSendingId >= 0 && g_voiceSendingId < VOICE_MAX)
                s_slotUid[g_voiceSendingId] = 0;   // 释放 uid → 该槽下条新语音拿全新 uid
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
        // 打勾：仅当 P4 推伙伴也不在传时才定妆，否则留转圈给 P4 收尾（避免一方未完就打勾闪烁）
        if (g_xferAnim == 1 && !peerVoiceActive()) g_xferAnim = 2;
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
}

static void dispatchCommand(const char* msg, JsonDocument& doc)
{
    learnTerminal(msg, doc);   // ★ 任何命令都可能补上 terminal（见上方注释的两条规则）

    // ── 设备互绑（P1）──
    if (strcmp(msg, "BIND") == 0)   { handleBind(doc);   return; }
    if (strcmp(msg, "UNBIND") == 0) { handleUnbind(doc); return; }

    // ── P4 伙伴语音 ARQ：伙伴回来的确认帧 → 推进本机发送状态机 ──
    if (strcmp(msg, "PV_ACK")  == 0) { pvOnAck (doc["xferId"] | 0u, doc["next"] | 0u); return; }
    if (strcmp(msg, "PV_DONE") == 0) { pvOnDone(doc["xferId"] | 0u); return; }
    if (strcmp(msg, "PV_FAIL") == 0) { pvOnFail(doc["xferId"] | 0u); return; }
    if (strcmp(msg, "PV_BUSY") == 0) { pvOnBusy(doc["xferId"] | 0u); return; }

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
    if (xfering || g_posStreaming || g_voiceSending || peerVoiceActive())
                                    want = WIFI_PS_NONE;        // 下载/串流/语音上行/伙伴语音 满功率
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
                    if (slot >= 0) startPeerVoiceSend(slot, peer);
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
            if (!client.loop()) break;               // 断连即止
            if (espClient.available() <= 0) break;   // 无更多缓冲数据即止
        }

        // ── 传输接收看门狗：App 中途掉线（不再发分片/END/ABORT）时，避免设备永久
        //    卡在传输锁定态（xferLocked + forceActive 不休眠、屏幕锁待机、持续掉电）。
        //    g_lastMqttRxMs 每收到一片就刷新；停止刷新超过阈值即判 App 掉线。──
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
            xQueueOverwrite(qServoCmd, &toMain);
            mainWake();
            LOG("[MIRROR] 伙伴停止，舵机归中位\n");
        }

        // ── 后台落盘收尾：只记日志，★绝不在这里再发回执 ──
        //   VIDEO_OK 已在 END_VIDEO commit 成功时立即发出（内容那时已在 PSRAM 可播）。
        //   若这里补发：① 重复一条回执；② 落盘失败发出的 VIDEO_FAIL 会带上【当前】
        //   g_curXferId —— 万一 App 已开始新一轮传输，这条迟到的 FAIL 会通过会话校验，
        //   被误判成新传输失败。落盘失败只影响"掉电后退回旧内容"，不影响本次使用。
        if (g_videoResultPend && g_persistPending == 0) {
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

        // ── 语音消息发送（App 拉取时逐条发 term/<sn>/voice，收 ack 后删）──
        handleVoiceSend();

        // ── P4 伙伴语音 ARQ：推进发送状态机 + 接收侧无活动超时兜底 ──
        pumpPeerVoiceSend();
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
        else if (peerVoiceActive())  pollTicks = pdMS_TO_TICKS(5);           // P4 伙伴语音：低延迟（逐片往返快）
        else if (g_posStreaming)     pollTicks = pdMS_TO_TICKS(5);           // 串流：低延迟
        else if (g_pwrTier == 2)     pollTicks = pdMS_TO_TICKS(WEB_L2_POLL_MS); // L2：省电（推送仍≤1s）
        else                         pollTicks = pdMS_TO_TICKS(100);         // L0/L1：低延迟

        if (wdtSubscribed) esp_task_wdt_reset();   // 喂狗：稳态每轮一次（传输期 pollTicks=1，卡在 client.loop 内 >5s 才会点名）
        vTaskDelay(pollTicks);
    }
}