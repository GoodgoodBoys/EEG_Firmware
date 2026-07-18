#include <Arduino.h>

#include <QMI8658.h>

#include "inc/debug.hpp"
#include "inc/fs.hpp"
#include "inc/lcd.hpp"
#include "inc/msg.hpp"
#include "inc/imu.hpp"
#include "inc/web.hpp"
#include "inc/aud.hpp"
#include "inc/servo.hpp"
#include "inc/mic.hpp"
#include "inc/configSys.hpp"
#include "inc/nvs_sn.h"
#include "inc/provision.hpp"
#include "esp_pm.h"
#include <WiFi.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_random.h"

#define VERSION "0.1"
#define RESOURCE_MONITOR_INTERVAL_MS 5000
#define TASK_STACK_SIZE 16384

// PCB v3.0 引脚（原理图 alivePCB-v3.0, 2026-07-06）
// v3.0 新增独立按键 Button1，配网长按检测从 BOOT(IO0) 迁移到这里，
// 避免用户长按误触发 BOOT 启动选择脚。
#define PROV_BUTTON_PIN     42       // Button1（IO42）微动开关
#define PROV_LONG_PRESS_MS  2000     // 长按 2 秒

#define BAT_ADC_PIN         5
#define BAT_DIVIDER         3.128f      // (100k+47k)/47k
#define BAT_SAMPLES         16

// v3.0 新增 Charge_Status 硬件充电状态脚（IO4）：HIGH=正在充电，LOW=未充电。
// 取代旧的电压阈值软件判断。
#define CHARGE_STATUS_PIN   4

volatile int      g_batPercent = -1;    // -1=未知 -2=充电中 0~100=电量（LCD 叠加读取，勿加 static！）
volatile uint32_t g_batMv      = 0;

volatile bool g_batCharging = false;
volatile int g_pwrTier = 0;

// 本次开机随机数：随 devInfo 上报，App 据此秒级识别"设备重启过"→ 中止在途传输，不等超时。
// 每次上电/重启都不同；同一次开机内恒定（bind/配置更新重发 devInfo 时不变，故不误判）。
volatile uint32_t g_bootId = 0;

// NVS SN
NvsSn g_sn;

// 传输锁标志：传输期间屏蔽 IMU 等触发的播放请求
static bool xferLocked = false;

static esp_pm_lock_handle_t s_freqLock = nullptr;
static bool s_freqLockHeld = false;

// 独立禁睡锁：仅信封态用——释放频率锁降到 40MHz，但持此锁禁 light-sleep（不闪、不打断推屏）
static esp_pm_lock_handle_t s_sleepLock = nullptr;
static bool s_sleepHeld = false;
static inline void setSleepLock(bool on)
{
    if (!s_sleepLock) return;
    if (on && !s_sleepHeld)  { esp_pm_lock_acquire(s_sleepLock); s_sleepHeld = true;  }
    if (!on &&  s_sleepHeld) { esp_pm_lock_release(s_sleepLock); s_sleepHeld = false; }
}

// 消息队列
QueueHandle_t qMainToLcd;
QueueHandle_t qMainToWeb;
QueueHandle_t qMainToAud;
QueueHandle_t qMainToMic;

QueueHandle_t qLcdToMain;
QueueHandle_t qWebToMain;
QueueHandle_t qImuToMain;
QueueHandle_t qMicToMain;

QueueHandle_t qConfigUpdate;
QueueHandle_t qPosStream;

// ── 低功耗唤醒信号量（L2 阻塞 + 事件瞬时唤醒）──
SemaphoreHandle_t xMainWake = nullptr;   // 主循环唤醒（计数型，事件不丢）
SemaphoreHandle_t xLcdWake  = nullptr;   // LCD 任务唤醒（二值）
SemaphoreHandle_t xAudWake  = nullptr;   // AUD 任务唤醒（二值）

#define PWR_L2_BLOCK_MS  500   // L2 各任务最长阻塞 = light-sleep 单次时长上限

// 任务句柄
TaskHandle_t  xLcdTaskHandle;
TaskHandle_t  xImuTaskHandle;
TaskHandle_t  xWebTaskHandle;
TaskHandle_t  xAudTaskHandle;
TaskHandle_t  xMicTaskHandle;


// ══════════════════════════════════════════════════════════════
//  功耗管理：活动分级（L0/L1/L2）+ CPU 频率单一仲裁
//
//  活动 = 舵机动作 / 摇一摇 / 拍一拍 / 视频切换或播放（重置 g_lastActivityMs）
//    L0  <30s ：LCD 全亮            ，持锁 240MHz，禁止 light-sleep
//    L1  ≥30s ：LCD 亮度 60%        ，持锁 240MHz，禁止 light-sleep（屏亮防闪）
//    L2  ≥60s ：LCD 面板休眠+停渲染 ，放锁 → 自动 light-sleep（40MHz + DTIM 唤醒）
//
//  配网 / 姿态串流 / 文件传输：forceActive() 全程满电，与低功耗互斥。
// ══════════════════════════════════════════════════════════════
extern volatile bool g_posStreaming;   // web.cpp（pos 串流期为 true）
extern volatile bool g_voiceSending;   // web.cpp（设备→App 语音上行期为 true）
extern volatile bool g_needWifiHint;   // web.cpp（连不上显示配网引导期间保屏幕亮）
extern volatile int  g_xferAnim;       // lcd.cpp（传输动画：转圈/打勾期间保屏幕亮）
extern volatile bool g_lcdSleeping;    // lcd.cpp（睡眠动画中：不算活动）

#define PWR_IDLE_L1_MS      30000
#define PWR_IDLE_L2_MS      60000

static uint32_t g_lastActivityMs = 0;
static int      g_curTier        = 0;

// 消息待回复态：收到视频消息(VIDEO_SHOW)置未读；拍一拍(TAP)回复清除；
// tier2 且仍未读 → 进"信封待回复态"（屏60%+信封弹跳，功耗同 L2：light-sleep+降频+WoM）。
static bool g_hasUnreadMsg = false;
static bool g_envelopeMode = false;

static inline void markActivity() { g_lastActivityMs = millis(); }

// 轻量抬频:持锁=240MHz；放锁=自动 40MHz + light-sleep。绝不调用 esp_pm_configure。
static inline void cpuBoost(bool on)
{
    if (!s_freqLock) return;
    if (on && !s_freqLockHeld)  { esp_pm_lock_acquire(s_freqLock); s_freqLockHeld = true;  }
    if (!on && s_freqLockHeld)  { esp_pm_lock_release(s_freqLock); s_freqLockHeld = false; }
}

// 供 IMU/WEB 任务跨文件调用：有事件 → 唤醒主循环（瞬时恢复心跳）
void mainWake() { if (xMainWake) xSemaphoreGive(xMainWake); }

// Button1 按下中断：瞬时唤醒主循环（L2/light-sleep 下按键立即生效，不用等 500ms 轮询）
void IRAM_ATTR buttonISR() {
    if (xMainWake) {
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(xMainWake, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

// 离开 L2 → 唤醒被信号量阻塞的非唤醒任务（LCD/AUD），瞬时恢复心跳
static void wakeRenderTasks()
{
    if (xLcdWake) xSemaphoreGive(xLcdWake);
    if (xAudWake) xSemaphoreGive(xAudWake);
}

// 配网 / 姿态串流 / 文件传输：全程满电，与低功耗互斥
static void forceActive(uint32_t now)
{
    g_lastActivityMs = now;
    if (g_curTier != 0) {            // 从低功耗档拉回 → 唤醒渲染/音频任务
        g_curTier = 0;
        g_pwrTier = 0;
        lcdSetPowerMode(0);
        wakeRenderTasks();
        if (g_lcdSleeping) {         // 从睡眠动画醒来 → 回 idle
            g_lcdSleeping = false;
            MessageToLCD_t m = {}; m.cmd = LCDMSG_PLAY; m.playTimes = 0;   // playTimes=0 → switchToFree(idle)
            xQueueSend(qMainToLcd, &m, 0);
        }
    }
    g_pwrTier = 0;
    cpuBoost(true);
}

static void powerManagerLoop()
{
    uint32_t now = millis();

    // 配网 / 姿态串流 / 文件传输（xferLocked）/ 录音中：满电不睡
    if (g_enterProvisioning || Config.isProvMode() ||
        g_posStreaming || xferLocked || micIsRecording() || g_xferAnim ||
        g_needWifiHint) {
        forceActive(now);   // 语音上传动画/配网引导期间也保持亮，才看得到提示
        return;
    }

    // 播放中（视频/音频）持续视为活动
    if (lcdIsPlaying() || audIsPlaying()) g_lastActivityMs = now;

    uint32_t idle = now - g_lastActivityMs;
    int tier = (idle >= PWR_IDLE_L2_MS) ? 2 : (idle >= PWR_IDLE_L1_MS) ? 1 : 0;
    bool envelope = (tier == 2 && g_hasUnreadMsg);   // 信封待回复态（L2 功耗 + 屏60%信封弹跳）

    if (tier != g_curTier || envelope != g_envelopeMode) {
        int prev = g_curTier;
        g_curTier = tier;
        g_pwrTier = tier;
        g_envelopeMode = envelope;
        lcdSetPowerMode(envelope ? 3 : tier);           // 3=信封态；否则常规 L0/L1/L2
        if (prev == 2 || envelope) wakeRenderTasks();   // 离开 L2 或 进信封 → 唤醒阻塞的 lcdTask
        LOG("[PWR] → %s (空闲 %lus)\n",
            envelope ? "信封待回复" : (tier == 2 ? "L2" : tier == 1 ? "L1" : "L0"),
            (unsigned long)(idle / 1000));
        // ★ 进入 L1(0→1)：播 startSleep 一次（信封态不播 startSleep）
        if (prev == 0 && tier == 1 && !envelope) {
            g_lcdSleeping = true;
            MessageToLCD_t m = {}; m.cmd = LCDMSG_PLAY; m.motion = MOTION_STARTSLEEP;
            m.playTimes = 1; m.interruptAble = true;
            xQueueSend(qMainToLcd, &m, 0);
        }
    }
    g_pwrTier = tier;

    // ★ CPU 频率唯一仲裁：
    //   L0/L1 → 持锁 240MHz，禁止 light-sleep（屏亮防闪）
    //   L2    → 放锁，允许自动 light-sleep（40MHz + DTIM 唤醒）
    //   语音上行 → 即使 L2（屏已休眠）也持锁 240MHz 保上行吞吐，屏幕照常灭
    // 信封态：降频到 40MHz（省电）+ 持禁睡锁禁 light-sleep（否则背光闪、推屏被打断）。
    if (envelope) {
        cpuBoost(false);        // 释放频率锁 → 40MHz
        setSleepLock(true);     // 但禁 light-sleep
    } else {
        cpuBoost(tier <= 1 || g_voiceSending);
        setSleepLock(false);    // 其他态不额外禁睡（保持原行为）
    }
}

static void batInit() {
    analogReadResolution(12);
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
    analogRead(BAT_ADC_PIN);

    // 充电状态：硬件电平判断（HIGH=正在充电，LOW=未充电）
    pinMode(CHARGE_STATUS_PIN, INPUT);
}
static uint32_t batReadAdcMv() {        // 分压后实测 mV（eFuse 校准）
    uint32_t sum = 0;
    for (int i = 0; i < BAT_SAMPLES; i++) sum += analogReadMilliVolts(BAT_ADC_PIN);
    return sum / BAT_SAMPLES;
}
static int batMvToPercent(uint32_t mv) {     // 2S 锂电曲线（8.4V→6.6V）
    static const struct { uint16_t mv; uint8_t pct; } C[] = {
        {8400,100},{8200,90},{8000,80},{7800,70},{7600,62},
        {7400,52},{7300,42},{7200,30},{7000,15},{6800,6},{6600,0}
    };
    const int N = sizeof(C)/sizeof(C[0]);
    if (mv >= C[0].mv)   return 100;
    if (mv <= C[N-1].mv) return 0;
    for (int i = 0; i < N-1; i++)
        if (mv <= C[i].mv && mv > C[i+1].mv) {
            int dMv = C[i].mv - C[i+1].mv, dPct = C[i].pct - C[i+1].pct;
            return C[i+1].pct + (int)((long)(mv - C[i+1].mv) * dPct / dMv);
        }
    return 0;
}

static bool batIsCharging() {
    return digitalRead(CHARGE_STATUS_PIN) == HIGH;
}

static void batUpdate() {                 // 每 5s 一次
    static uint32_t last = 0;
    if (millis() - last < 5000) return;
    last = millis();

    uint32_t adcMv = batReadAdcMv();
    uint32_t vbat  = (uint32_t)(adcMv * BAT_DIVIDER + 0.5f);
    g_batMv = vbat;
    g_batPercent  = batMvToPercent(vbat);
    g_batCharging = batIsCharging();

    LOG("[BAT] %s | %u.%02uV | %d%% | ADC %umV\n",
        g_batCharging ? "充电中" : "电池",
        vbat/1000, (vbat%1000)/10, g_batPercent, adcMv);
}

#if _DEBUG
// 仅 _DEBUG 版编译：无 log 版整段移除（连栈水位采集一并省掉）
static void printStackHWM(const char* name, TaskHandle_t h, uint32_t cfgBytes)
{
    if (h == nullptr) {                       // 未创建的任务（如被注释的 MicTask）
        LOG("  %-9s | 未创建\n", name);
        return;
    }
    uint32_t freeMin = uxTaskGetStackHighWaterMark(h) * sizeof(StackType_t);
    uint32_t usedMax = (cfgBytes > freeMin) ? (cfgBytes - freeMin) : 0;
    float    pct     = cfgBytes ? (100.0f * usedMax / cfgBytes) : 0.0f;
    LOG("  %-9s | 总=%5u | 峰值用=%5u | 剩余=%5u | %4.1f%%\n",
        name, cfgBytes, usedMax, freeMin, pct);
}
#endif


// ══════════════════════════════════════════════════════════════
//  关机 → 深度睡眠（长按 Button1 8 秒触发；按 BOOT 键(GPIO0) ext0 唤醒开机）
// ══════════════════════════════════════════════════════════════
#define SHUTDOWN_LONG_PRESS_MS   8000
#define SHUTDOWN_DEBUG_TIMER_S   60     // >0 = 兜底 timer 秒数（调试防睡死）；0 = 真关机，只靠 BOOT 键唤醒

static void enterDeepSleep()
{
    LOG("[PWR] ★ 长按 %ds → 关机，进入深度睡眠\n", SHUTDOWN_LONG_PRESS_MS / 1000);

    // 挂起任务，避免配 IMU 时 I2C 并发
    if (xImuTaskHandle) vTaskSuspend(xImuTaskHandle);
    if (xLcdTaskHandle) vTaskSuspend(xLcdTaskHandle);
    if (xAudTaskHandle) vTaskSuspend(xAudTaskHandle);
    if (xMicTaskHandle) vTaskSuspend(xMicTaskHandle);

    // 关外设
    backLightOFF();
    servoPowerOff();
    WiFi.mode(WIFI_OFF);

    // 麦克风省电（复现 micSleepTest 最优待机：CLK=IO48 拉 HIGH+hold，DATA=IO47 输入下拉）
    gpio_reset_pin(GPIO_NUM_48);
    gpio_set_direction(GPIO_NUM_48, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_48, 1);
    gpio_hold_en(GPIO_NUM_48);
    gpio_reset_pin(GPIO_NUM_47);
    gpio_set_direction(GPIO_NUM_47, GPIO_MODE_INPUT);
    gpio_pulldown_en(GPIO_NUM_47);

    // 关 IMU 省电（唤醒改用 BOOT 键，不用 IMU）
    imuPrepareDeepSleep();

    // ext0：BOOT 键(GPIO0) 唤醒。BOOT 空闲高、按下接地为低 → 低电平唤醒 + 上拉保持。
    // ⚠ 唤醒时按一下即松开：芯片 reset 时若仍按住 GPIO0(低) 可能进下载模式、不启动固件。
    rtc_gpio_pulldown_dis(GPIO_NUM_0);
    rtc_gpio_pullup_en(GPIO_NUM_0);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);   // 0 = 低电平唤醒（BOOT 按下）

    // 调试兜底 timer（防 WoM 没配通变砖；调通后 SHUTDOWN_DEBUG_TIMER_S 设 0）
    if (SHUTDOWN_DEBUG_TIMER_S > 0)
        esp_sleep_enable_timer_wakeup((uint64_t)SHUTDOWN_DEBUG_TIMER_S * 1000000ULL);

    gpio_deep_sleep_hold_en();
    LOG("[PWR] 进入深睡（拍一下开机）...\n");
    Serial.flush();
    delay(50);
    esp_deep_sleep_start();          // 不返回；唤醒 = 芯片 reset 重启 = 开机
}

static bool snSetup()
{
    LOG("[SN] 开始 SN 校验...\n");
    SnError ret = g_sn.initDefault();
    if (ret != SnError::OK) {
        LOG("[SN] ★ SN 初始化失败: %s\n", snErrorToStr(ret));
        return false;
    }
    String currentSn;
    ret = g_sn.read(currentSn);
    if (ret != SnError::OK) {
        LOG("[SN] ★ SN 读取失败: %s\n", snErrorToStr(ret));
        return false;
    }
    LOG("[SN] ✓ 设备 SN: [%s]\n", currentSn.c_str());
    return true;
}

void setup() {
    LOG_BEGIN(115200);
    LOG("\n");
    LOG("╔══════════════════════════════════════╗\n");
    LOG("║   EGG ESP32-S3   固件版本 v%-10s║\n", VERSION);
    LOG("╚══════════════════════════════════════╝\n");
    LOG("[BOOT] 系统启动...\n");

    // 本次开机随机数（供 App 识别设备重启）。esp_random 在 RF 未启用时熵偏弱但足够区分开机。
    g_bootId = esp_random();
    LOG("[BOOT] bootId=%u\n", (unsigned)g_bootId);

    // 唤醒原因：区分正常上电 / 从关机(深睡)被拍醒
    esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
    if (wc == ESP_SLEEP_WAKEUP_EXT0)       LOG("[BOOT] 从关机唤醒（BOOT 键开机）\n");
    else if (wc == ESP_SLEEP_WAKEUP_TIMER) LOG("[BOOT] 调试 timer 唤醒开机\n");

    LOG("[BOOT] === 启动时内存 ===\n");
    LOG("[BOOT] 内部RAM剩余: %u KB\n", ESP.getFreeHeap()  / 1024);
    LOG("[BOOT] PSRAM总量:   %u KB\n", ESP.getPsramSize() / 1024);
    LOG("[BOOT] PSRAM剩余:   %u KB\n", ESP.getFreePsram() / 1024);

    /* Flash 文件系统初始化 */
    LOG("[BOOT] (1/9) 挂载 LittleFS...\n");
    if (!fsInit()) {
        LOG("[BOOT] ✗ LittleFS 挂载失败，系统停止启动！\n");
        while (true) {}
    }
    LOG("[BOOT] ✓ LittleFS 就绪\n");

    /* SN 校验 */
    LOG("[BOOT] (2/9) SN 序列号校验...\n");
    if (!snSetup()) {
        LOG("[BOOT] ✗ SN 校验失败，系统停止启动！\n");
        while (true) {}
    }
    LOG("[BOOT] ✓ SN 校验通过\n");

    /* 电源管理 / 自动 light-sleep */
    LOG("[BOOT] (3/9) 配置电源管理（自动 light-sleep）...\n");
    {
        esp_pm_config_t pm = { .max_freq_mhz = 240, .min_freq_mhz = 40, .light_sleep_enable = true };
        if (esp_pm_configure(&pm) != ESP_OK) {
            LOG("[BOOT] ⚠ esp_pm_configure 失败（PM/Tickless 未启用）\n");
        } else {
            LOG("[BOOT] ✓ 自动 light-sleep 已开启（40~240MHz）\n");
            // 抬频锁:持有=锁定到 max_freq(240)；释放=系统自动降到 40 并可 light-sleep
            esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "freqmax", &s_freqLock);
            esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "nosleep", &s_sleepLock);   // 信封态禁睡用
        }
    }

    /* 配置 + BLE */
    LOG("[BOOT] (4/9) 加载配置 + 初始化 BLE...\n");
    // 先读 SN，用它拼出唯一 BLE 名 EGG_<SN>（修"所有设备同名、App 分不清"的 Bug）。
    // ★ 静态缓冲：Config.begin 只存名字指针供后续 BLE 初始化用，不能传临时 String。
    static char s_bleName[24] = "EGG_00000000";
    {
        String snForCfg;
        if (g_sn.read(snForCfg) == SnError::OK && !snForCfg.isEmpty())
            snprintf(s_bleName, sizeof(s_bleName), "EGG_%s", snForCfg.c_str());
        Config.begin(s_bleName);
        // 把 SN 镜像进配置(仅内存不落盘)，让 BLE READ 能读到——App 连上显示 SN + 记住本机 SN
        if (!snForCfg.isEmpty()) Config.setString("SN", snForCfg.c_str());
    }
    LOG("[BOOT] BLE 名: %s\n", s_bleName);
    Config.printConfig();
    LOG("[BOOT] ✓ 配置就绪\n");

    /* LCD */
    LOG("[BOOT] (5/9) 初始化 LCD...\n");
    lcdInit();
    LOG("[BOOT] ✓ LCD 就绪\n");

    /* 音频 */
    LOG("[BOOT] (6/9) 初始化音频（I2S/MAX98357A 功放）...\n");
    if (audInit() != AUD_OK) {
        LOG("[BOOT] ⚠ 音频初始化异常（继续启动，播放可能不可用）\n");
    } else {
        LOG("[BOOT] ✓ 音频就绪\n");
    }

    /* 麦克风（PDM，I2S0；Button1 单击录音）*/
    LOG("[BOOT] 初始化麦克风（PDM，I2S0）...\n");
    if (!initMIC()) {
        LOG("[BOOT] ⚠ 麦克风初始化失败（继续启动，录音功能不可用）\n");
    } else {
        LOG("[BOOT] ✓ 麦克风就绪\n");
    }

    /* IMU */
    LOG("[BOOT] (7/9) 初始化 IMU（QMI8658）...\n");
    imuInit();
    LOG("[BOOT] ✓ IMU 就绪\n");

    /* 舵机 */
    LOG("[BOOT] (8/9) 初始化舵机...\n");
    servoInit();
    servoRotate(SERVO_REST_ANGLE);
    LOG("[BOOT] ✓ 舵机就绪（归中位 %d°）\n", SERVO_REST_ANGLE);

    /* 电量 ADC */
    LOG("[BOOT] (9/9) 初始化电量检测...\n");
    batInit();
    LOG("[BOOT] ✓ 电量检测就绪\n");

    pinMode(PROV_BUTTON_PIN, INPUT_PULLUP);                 // 配网/录音按键
    // 按下中断：L2/light-sleep 下按键立即唤醒主循环（ISR 里 xMainWake 空指针安全）
    attachInterrupt(digitalPinToInterrupt(PROV_BUTTON_PIN), buttonISR, FALLING);

    /* 消息队列 */
    LOG("[BOOT] 创建消息队列...\n");
    qMainToLcd    = xQueueCreate(5, sizeof(MessageToLCD_t));
    qMainToWeb    = xQueueCreate(5, WEB_MSG_LEN);
    qMainToAud    = xQueueCreate(5, sizeof(MessageToAud_t));
    qMainToMic    = xQueueCreate(5, sizeof(MessageToMic_t));
    qLcdToMain    = xQueueCreate(5, sizeof(MessageToMain_t));
    qWebToMain    = xQueueCreate(5, sizeof(MessageToMain_t));
    qImuToMain    = xQueueCreate(5, sizeof(MessageToMain_t));
    qConfigUpdate = xQueueCreate(3, sizeof(uint8_t));
    qPosStream    = xQueueCreate(1, sizeof(PosSample_t));

    // ── 低功耗唤醒信号量（务必在 imu/web/lcd/aud 任务创建之前）──
    xMainWake = xSemaphoreCreateCounting(20, 0);   // 计数型：多事件不丢
    xLcdWake  = xSemaphoreCreateBinary();
    xAudWake  = xSemaphoreCreateBinary();
    if (!qMainToLcd || !qMainToWeb || !qMainToAud || !qMainToMic ||
        !qLcdToMain || !qWebToMain || !qImuToMain || !qConfigUpdate ||
        !qPosStream || !xMainWake || !xLcdWake || !xAudWake) {
        LOG("[BOOT] ✗ 队列/信号量创建失败！\n");
        while (true) {}
    }
    LOG("[BOOT] ✓ 队列 + 唤醒信号量就绪\n");

    /* 任务创建 */
    LOG("[BOOT] 创建任务...\n");
    BaseType_t ok = pdPASS;

    /* IMU 任务（内部 SRAM 栈 — Flash 操作期间 cache 禁用，栈不能在 PSRAM）
       优先级 2（高于 LCD/MIC 的 1）：保证 250Hz 采样能及时抢占，拍击冲击不被渲染饿掉。
       采样极快且每轮 delay 让出，不会饿死同核渲染任务。*/
    ok &= xTaskCreatePinnedToCore(imuTask, "IMUTask", 3072, NULL, 2, &xImuTaskHandle, 0);

    /* WEB 任务 */
    ok &= xTaskCreatePinnedToCore(webTask, "WEBTask", 10240, NULL, 0, &xWebTaskHandle, 0);

    /* LCD 任务（内部 SRAM 栈 — JPEG 解码 + LittleFS 文件读取需要）*/
    ok &= xTaskCreatePinnedToCore(lcdTask, "LcdTask", 3072, NULL, 1, &xLcdTaskHandle, 0);

    /* AUD 任务（内部 SRAM 栈 — minimp3 解码 ~4KB + 开销；大缓冲在 PSRAM）*/
    ok &= xTaskCreatePinnedToCore(audTask, "AudTask", 19456, NULL, 0, &xAudTaskHandle, 1);

    /* MIC 任务（栈含 Shine MP3 编码开销，从 4096 提到 8192）*/
    ok &= xTaskCreatePinnedToCore(micTask, "MicTask", 8192, NULL, 1, &xMicTaskHandle, 0);

    if (ok != pdPASS) {
        LOG("[BOOT] ✗ 任务创建失败（内存不足？）\n");
    } else {
        LOG("[BOOT] ✓ 任务创建完成（IMU/WEB/LCD/AUD/MIC）\n");
    }

    // ★ TWDT 超时 5s→8s：flash 密集写（persist 落盘）经 IPC 冻结 core0 cache
    //   （spi_flash_op_block_func），单次擦除+搬迁偶发逼近 5s。8s 给足余量，
    //   仍保留 core0 IDLE 监控 + panic（reconfigure 只改全局配置，不动已 add 的任务）。
    {
        esp_task_wdt_config_t twdt_cfg = {
            .timeout_ms     = 8000,
            .idle_core_mask = (1 << 0),   // 保持监控 core0 IDLE0（与原 sdkconfig 一致）
            .trigger_panic  = true,
        };
        if (esp_task_wdt_reconfigure(&twdt_cfg) == ESP_OK)
            LOG("[BOOT] ✓ TWDT 超时已设为 8s（flash 落盘冻结留余量）\n");
        else
            LOG("[BOOT] ⚠ TWDT reconfigure 失败，沿用默认 5s\n");
    }

    LOG("[BOOT] ════════ 系统初始化完成 ════════\n\n");

    g_lastActivityMs = millis();   // 开机视为活动，避免立即进省电
}

void loop() {
    MessageToLCD_t  msgToLcd;
    char  msgToWeb[WEB_MSG_LEN];
    MessageToAud_t  msgToAud;
    MessageToMic_t  msgToMic;
    MessageToMain_t msgFromWeb;
    MessageToMain_t msgFromImu;
    uint8_t         cfgMsg;

    {
        static uint32_t btnPressStart = 0;
        static bool     btnWasDown    = false;
        static bool     provFired     = false;
 
        bool btnDown = (digitalRead(PROV_BUTTON_PIN) == LOW);

        if (btnDown) {
            // 按住期间保持活跃（脱离 L2、快速轮询），松开才能被及时检测到
            markActivity();
            if (!btnWasDown) {
                btnWasDown    = true;
                btnPressStart = millis();
                provFired     = false;
            } else {
                uint32_t hold = millis() - btnPressStart;
                if (!provFired && hold >= PROV_LONG_PRESS_MS) {
                    provFired = true;
                    LOG("[BTN] Button1 长按 %d ms，触发配网\n", PROV_LONG_PRESS_MS);
                    g_enterProvisioning = true;     // 通知 webTask
                }
                if (hold >= SHUTDOWN_LONG_PRESS_MS) {
                    enterDeepSleep();               // 长按 8 秒 → 关机（不返回）
                }
            }
        } else {
            if (btnWasDown && !provFired) {
                // 短按松开（非长按）= 单击
                if (Config.isProvMode() || g_enterProvisioning) {
                    // 配网中：忽略单击，不录音（避免与 BLE 配网重叠）
                    LOG("[BTN] 配网中，忽略单击（不录音）\n");
                } else {
                    // 单击切换录音开始/结束
                    MessageToMic_t micMsg;
                    micMsg.cmd = micIsRecording() ? MIC_STOP : MIC_START;
                    xQueueSend(qMainToMic, &micMsg, 0);
                    markActivity();      // 录音 = 活动
                    LOG("[BTN] Button1 单击 -> %s\n",
                        micMsg.cmd == MIC_START ? "开始录音" : "停止录音");
                }
            }
            btnWasDown = false;
        }
    }
 
    // ── Web 指令处理 ─────────────────────────────────────
    if (xQueueReceive(qWebToMain, &msgFromWeb, 0) == pdTRUE) {
        LOG("[Main] 收到Web指令: %s\n", msgFromWeb.strVal);

        // ── 传输锁定 ──
        if (strcmp(msgFromWeb.strVal, "XFER_LOCK") == 0) {
            xferLocked = true;
            LOG("[Main] ★ 传输锁定：停止视频和音频播放\n");

            // 停止音频
            msgToAud.cmd = AUD_STOP;
            xQueueSend(qMainToAud, &msgToAud, 0);

            // LCD 切到待机（不播放 target 视频，避免 SD 冲突）
            msgToLcd.cmd         = LCDMSG_PLAY;
            msgToLcd.motion      = MOTION_TAP;
            msgToLcd.playTimes   = 0;
            msgToLcd.interruptAble = true;
            xQueueSend(qMainToLcd, &msgToLcd, 0);
        }
        // ── 传输解锁 ──
        else if (strcmp(msgFromWeb.strVal, "XFER_UNLOCK") == 0) {
            xferLocked = false;
            LOG("[Main] ★ 传输解锁：恢复正常\n");
        }
        // ── 视频 & 音频播放 ──
        if (strcmp(msgFromWeb.strVal, "VIDEO_SHOW") == 0) {
            if (xferLocked) {
                LOG("[Main] ⚠ 传输中，屏蔽\n");
            } else {
                // 发送播放指令
                msgToLcd.cmd           = LCDMSG_PLAY_NONINT;
                msgToLcd.motion        = MOTION_VIDEOSHOW;
                msgToLcd.playTimes     = 1;
                msgToLcd.interruptAble = false;
                xQueueSend(qMainToLcd, &msgToLcd, 0);

                msgToAud.cmd = AUD_PLAY;
                xQueueSend(qMainToAud, &msgToAud, 0);

                markActivity();          // 视频播放 = 活动
                g_hasUnreadMsg = true;   // ★ 收到消息，待拍一拍回复；1min 无回复进信封态
            }
        }
        // ── 录音 ──
        else if (strcmp(msgFromWeb.strVal, "RECORD") == 0) {
            if (xferLocked) {
                LOG("[Main] ⚠ 传输中，屏蔽 RECORD\n");
            } else {
                msgToMic.cmd = MIC_START;
                xQueueSend(qMainToMic, &msgToMic, 0);
                markActivity();          // 录音 = 活动
            }
        }
        // ── 舵机（传输中也允许，不涉及 SD）──
        else if (strncasecmp(msgFromWeb.strVal, "SERVO", 5) == 0) {
            int angle, angularVelocity;
            int parsedCount = sscanf(msgFromWeb.strVal, "SERVO%d-%d", &angle, &angularVelocity);

            if (parsedCount == 2) {
                LOG("[SERVO] 目标角度=%d度，角速度=%d度/秒\n", angle, angularVelocity);
                angle           = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
                angularVelocity = constrain(angularVelocity, 0, SERVO_MAX_SPEED);
                servoRotateAV(angle, angularVelocity);
            } else {
                parsedCount = sscanf(msgFromWeb.strVal, "SERVO%d", &angle);
                if (parsedCount == 1) {
                    LOG("[SERVO] 目标角度=%d度\n", angle);
                    angle = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
                    servoRotate(angle);
                }
            }
            markActivity();              // 舵机动作 = 活动
        }
        // ── WBL 摇摆 ──
        else if (strncasecmp(msgFromWeb.strVal, "WBL", 3) == 0) {
            int angle1, angle2, angularVelocity, swingTimes, delayTime;
            sscanf(msgFromWeb.strVal, "WBL%d-%d-%d-%d-%d",
                   &swingTimes, &angle1, &angle2, &angularVelocity, &delayTime);
            for (int count = swingTimes; count > 0; count--) {
                servoRotateAV(angle1, angularVelocity);
                delay(delayTime);
                servoRotateAV(angle2, angularVelocity);
                delay(delayTime);
            }
            delay(500);
            servoRotate(SERVO_REST_ANGLE);
            markActivity();              // 摇摆 = 活动
        }
        // ── WINK：拍一拍送达成功 → 播 wink ──
        else if (strcmp(msgFromWeb.strVal, "WINK") == 0) {
            MessageToLCD_t m = {}; m.cmd = LCDMSG_PLAY; m.motion = MOTION_WINK;
            m.playTimes = 1; m.interruptAble = true;
            xQueueSend(qMainToLcd, &m, 0);
            markActivity();
        }
    }

    // ── IMU 指令处理（传输中屏蔽）────────────────────────
    if (xQueueReceive(qImuToMain, &msgFromImu, 0) == pdTRUE) {
 
        if (xferLocked) {
            LOG("[Main] ⚠ 传输中，屏蔽 IMU 事件 (CMD=%d)\n", msgFromImu.intVal);
        } else {
            LOG("[Main] 收到Imu指令，CMD=%d\n", msgFromImu.intVal);
 
            markActivity();              // 拍一拍/摇一摇 = 活动

            if (msgFromImu.intVal == 1) {
                // 拍一拍 → LCD 播放 TAP 动画
                msgToLcd.cmd           = LCDMSG_PLAY;
                msgToLcd.motion        = MOTION_TAP;
                msgToLcd.playTimes     = 1;
                msgToLcd.interruptAble = true;
                xQueueSend(qMainToLcd, &msgToLcd, 0);
 
                // ★ 转发 TAP 给 Web，由 Web 向 term/<terminal>/cmd 上报
                strcpy(msgToWeb, "TAP");
                xQueueSend(qMainToWeb, &msgToWeb, 0);

                // ★ P2：拍一拍同时"戳"已互绑的伙伴，让对方设备播放拍一拍视频
                strcpy(msgToWeb, "POKE_PEER");
                xQueueSend(qMainToWeb, &msgToWeb, 0);

                g_hasUnreadMsg = false;   // ★ 拍一拍=回复，清未读（取消信封态计时/退出信封态）

            } else if (msgFromImu.intVal == 2) {
                if (g_envelopeMode) {
                    // ★ 信封待回复态摇一摇：重播上一条(target 视频+音频) + 发 TAP 回执 + 退出
                    msgToLcd.cmd           = LCDMSG_PLAY_NONINT;
                    msgToLcd.motion        = MOTION_VIDEOSHOW;
                    msgToLcd.playTimes     = 1;
                    msgToLcd.interruptAble = false;
                    xQueueSend(qMainToLcd, &msgToLcd, 0);
                    msgToAud.cmd = AUD_PLAY;
                    xQueueSend(qMainToAud, &msgToAud, 0);
                    strcpy(msgToWeb, "TAP");           // 回执：与正常回复相同
                    xQueueSend(qMainToWeb, &msgToWeb, 0);
                    g_hasUnreadMsg = false;
                    g_envelopeMode = false;            // 退出信封态（本轮已 markActivity 拉回 L0）
                    LOG("[MSG] 信封态摇一摇 → 重播上一条 + 回执\n");
                } else {
                    // 摇一摇 → LCD 播放 SHAKE 动画
                    msgToLcd.cmd           = LCDMSG_PLAY;
                    msgToLcd.motion        = MOTION_SHAKE;
                    msgToLcd.playTimes     = 1;
                    msgToLcd.interruptAble = true;
                    xQueueSend(qMainToLcd, &msgToLcd, 0);
                    // ★ P3：摇一摇同时让已绑定伙伴的舵机镜像本机姿态（web 广播姿态一小段）
                    strcpy(msgToWeb, "SHAKE_PEER");
                    xQueueSend(qMainToWeb, &msgToWeb, 0);
                }
            }
        }
    }

    // ── 配置更新 ─────────────────────────────────────────
    if (xQueueReceive(qConfigUpdate, &cfgMsg, 0) == pdTRUE) {
        LOG("[Main] 配置已更新\n");
        msgToLcd.cmd = LCDMSG_CFG_UPDATE;
        xQueueSend(qMainToLcd, &msgToLcd, 0);
        msgToAud.cmd = AUD_CFG_UPDATE;
        xQueueSend(qMainToAud, &msgToAud, 0);
        strcpy(msgToWeb, "CFG_UPDATE");
        xQueueSend(qMainToWeb, &msgToWeb, 0);
    }

    Config.loop();

#if _DEBUG
    // ── 系统资源监控（仅 _DEBUG 版编译；无 log 版整段移除，省 CPU + 不唤醒）──
    static unsigned long lastMonitorTime = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastMonitorTime >= RESOURCE_MONITOR_INTERVAL_MS) {
        lastMonitorTime = currentTime;

        UBaseType_t   stackFreeWords    = uxTaskGetStackHighWaterMark(NULL);
        unsigned long stackFreeBytes    = stackFreeWords * sizeof(StackType_t);
        unsigned long stackUsedBytes    = TASK_STACK_SIZE - stackFreeBytes;
        float         stackUsagePercent = (float)stackUsedBytes / TASK_STACK_SIZE * 100;

        LOG("\n==================== 系统资源监控 ====================\n");
        LOG("运行时间: %lu 秒 | 传输锁: %s\n",
            currentTime / 1000, xferLocked ? "锁定" : "正常");
        LOG("------------------------------------------------------\n");
        LOG("堆内存: 剩余=%lu 字节 | 历史最小=%lu 字节\n",
            ESP.getFreeHeap(), ESP.getMinFreeHeap());
        LOG("主任务栈: 总=%d | 已用=%lu | 剩余=%lu | 占%.1f%%\n",
            TASK_STACK_SIZE, stackUsedBytes, stackFreeBytes, stackUsagePercent);
        LOG("内部RAM剩余: %u KB\n", ESP.getFreeHeap()  / 1024);
        LOG("PSRAM总量:   %u KB\n", ESP.getPsramSize() / 1024);
        LOG("PSRAM剩余:   %u KB\n", ESP.getFreePsram() / 1024);

        const lcdStats_t* ls = lcdGetStats();
        LOG("LCD: FPS=%.1f(max=%.1f) avg=%dms max=%dms %s 缓存=%d个/%d帧 池=%dKB/%dKB\n",
            ls->fps, ls->fpsMax, ls->avgDecodeMs, ls->maxDecodeMs,
            ls->isPreloaded ? "PSRAM" : "SD",
            ls->cachedVideos, ls->totalCachedFrames,
            (int)(ls->poolUsed / 1024), (int)(ls->poolTotal / 1024));
        LOG("---------------- 任务栈水位 ----------------\n");
        printStackHWM("loopTask", xTaskGetCurrentTaskHandle(), 8192);
        printStackHWM("IMUTask",  xImuTaskHandle, 3072);
        printStackHWM("WEBTask",  xWebTaskHandle, 10240);
        printStackHWM("LcdTask",  xLcdTaskHandle, 3072);
        printStackHWM("AudTask",  xAudTaskHandle, 19456);
        printStackHWM("MicTask",  xMicTaskHandle, 4096);   // 未创建会自动显示"未创建"
        LOG("======================================================\n\n");
    }
#endif

    batUpdate();

    servoLoop();          // 舵机：90°静止1s则 detach
    powerManagerLoop();   // 功耗分级 + CPU 频率仲裁

    // ── 心跳调度：L2 阻塞等事件（喂 light-sleep），事件 give 信号量瞬时唤醒；
    //    L0/L1 维持 100ms（按键长按检测/巡检）。──
    uint32_t napMs = (g_pwrTier == 2) ? PWR_L2_BLOCK_MS : 100;
    xSemaphoreTake(xMainWake, pdMS_TO_TICKS(napMs));
}