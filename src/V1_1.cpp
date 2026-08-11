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
#include "inc/health.hpp"
#include "inc/ota.hpp"
#include "esp_pm.h"
#include <WiFi.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_random.h"

#define VERSION "0.1"
#define RESOURCE_MONITOR_INTERVAL_MS 5000

// ── 各任务栈大小（字节）──
// ★ 必须用宏统一：以前建任务和 printStackHWM 各写一份字面量，改了一处忘另一处，
//   监控就会按错误的总量算占比（MicTask 早已是 8192，监控却仍按 4096 算 → 显示 0.0%）。
#define STACK_IMU   4096    // 3072→4096：WoM 唤醒那轮 imuExitWoM + analyzeTapWindow(大量 float
                            //   + %f 格式化) 同时展开，实测峰值 2568/3072 = 83.6%，余量仅 504 字节
#define STACK_WEB  10240
#define STACK_LCD   4096    // 3072→4096：实测峰值 2608/3072 = 84.9%，余量仅 464 字节（JPEG 解码路径）
#define STACK_AUD  24576    // 19456→24576：minimp3 的 mp3dec_decode_frame 解码 scratch 全在栈上，
                            //   实测峰值 17496/19456 = 89.9%，余量仅 1960 字节。而 Xtensa 的
                            //   低级中断复用【被打断任务的栈】，audTask 提优先级后跑得更频繁、
                            //   承接中断的概率也更高，这点余量不够安全。
#define STACK_MIC   8192

// PCB v3.0 引脚（原理图 alivePCB-v3.0, 2026-07-06）
// v3.0 新增独立按键 Button1，配网长按检测从 BOOT(IO0) 迁移到这里，
// 避免用户长按误触发 BOOT 启动选择脚。
#define PROV_BUTTON_PIN     2        // Button1（IO2）微动开关
#define PROV_LONG_PRESS_MS  2000     // 长按 2 秒

// 主循环每轮从 qWebToMain 排空的上限（条数 + 时间预算，见 loop() 里的说明）
#define QWEB_DRAIN_PER_LOOP   4
#define QWEB_DRAIN_BUDGET_MS  50

// ══════════════════════════════════════════════════════════════
//  舵机「首拍保留」：只针对把舵机从断电态唤醒的那一次摆动
//
//  成因：servoLoop 会在"中位静止 1 秒"后 detach + 断开供电轨，所以隔一会儿再摇，
//  舵机是从【断电态】起步的。而供电轨稳定只等了 SERVO_POWER_SETTLE_MS(20ms)，
//  舵机内部 MCU 启动 + 初始定位还要上百毫秒 —— 这期间写进去的 PWM 等于打空。
//  若此时 App 松手的回中位紧跟着到（qServoCmd 是深度 1 的覆盖队列，取的永远是
//  最新一条），舵机就直接归中位了，用户【完全看不到这次摆动】。
//
//  做法：这一拍执行后开一个保留窗，窗内主循环【暂不从 qServoCmd 取新指令】，
//  让舵机把这一拍走完；窗口结束再取——那时队列里自然是最新的那条（通常就是
//  回中位），语义和原来完全一致，只是晚了这一小段。
//
//  ★ 只对首拍生效：窗口关闭后 g_servoSwingHoldUntil 归零，后续摆动一律走原路径，
//    不引入任何延迟——连续快速摆动时舵机本就在工作态，再加延迟只会拖沓。
//  ★ 不阻塞主循环：只是跳过"取舵机队列"这一步，按键/电量/功耗状态机照常运行。
// ══════════════════════════════════════════════════════════════
#define SERVO_FIRST_SWING_HOLD_MS  250   // 覆盖"内部就绪 + 转过 35°"所需时间
static uint32_t g_servoSwingHoldUntil = 0;   // 保留窗截止时刻（0 = 无保留）

#define BAT_ADC_PIN         5
#define BAT_DIVIDER         3.128f      // (100k+47k)/47k
#define BAT_SAMPLES         16

// v3.0 新增 Charge_Status 硬件充电状态脚（IO4）：HIGH=正在充电，LOW=未充电。
// 取代旧的电压阈值软件判断。
#define CHARGE_STATUS_PIN   4

// 低电自动关机 / 开机复检阈值（分压后实测电压 mV）
#define BAT_SHUTDOWN_MV       6400   // ≤6.4V 视为没电
#define BAT_LOW_CONFIRM_CNT   3      // 运行中连续 N 次采样(每5s)≤阈值才关机，防舵机瞬时压降误关

// 关机休眠态下的【开机】阈值：必须高于关机阈值，形成迟滞(hysteresis)。
//   关机后负载从运行态(WiFi/LCD 几十~几百 mA)骤降到休眠 2.2mA，内阻压降消失、端电压
//   回弹几十 mV。若开机/关机同用 6400，会陷入"回弹→放行开机→开机后负载压回→又低电
//   关机"的反复循环（约 40s 一轮）。留 200mV 余量确保开机后撑得住。
#define BAT_WAKE_MV           6600   // >6.6V 才允许开机（正在充电时另行无条件放行）

volatile int      g_batPercent = -1;    // -1=未知 -2=充电中 0~100=电量（LCD 叠加读取，勿加 static！）
volatile uint32_t g_batMv      = 0;

volatile bool g_batCharging = false;
volatile int g_pwrTier = 0;

// 本次开机随机数：随 devInfo 上报，App 据此秒级识别"设备重启过"→ 中止在途传输，不等超时。
// 每次上电/重启都不同；同一次开机内恒定（bind/配置更新重发 devInfo 时不变，故不误判）。
volatile uint32_t g_bootId = 0;

// NVS SN
NvsSn g_sn;

// ★ 传输锁：直接读 webTask 的 g_xfering，不再维护本地 xferLocked 副本。
//   原来是 webTask 经 qWebToMain 发 XFER_LOCK/UNLOCK、主循环据此翻转一个本地 bool。
//   但那是【跨任务的成对状态迁移经过一条会丢消息的队列】——UNLOCK 丢一次就永久锁死
//   （forceActive 永不休眠 2.2mA→几十mA + VIDEO_SHOW/RECORD/SERVO 全被屏蔽，只能重启）。
//   而 xferLocked 的全部 6 处用途都只是在问"是否正在传输"，g_xfering 正是这个语义、
//   且是 webTask 直接写的变量、不经队列 —— 那个本地副本纯属冗余。
//   直接用它以后失配在结构上不可能发生，也顺带覆盖了反向失配（LOCK 丢失 → 传输期没停播）。
//   XFER_LOCK/UNLOCK 消息保留：它们还要触发【副作用】(AUD_STOP、LCD 切待机)，只是不再管状态。
//   附带好处：起止都比消息往返早一拍，屏蔽窗口更准。
extern volatile bool g_xfering;          // web.cpp：传输进行中（enterTransferMode/exitTransferMode 直接写）
#define xferLocked  (g_xfering)

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

QueueHandle_t qWebToMain;
QueueHandle_t qServoCmd;   // 舵机专用深度1覆盖队列：连发 SERVO 只应用最新角度，不占 qWebToMain、不溢出丢播放指令
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
extern volatile bool g_dndActive;      // web.cpp（当前勿扰时段：下载照收但不亮屏）
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

// ★ 任何可感知活动 → 记录时刻 + 立即打断在写的落盘（唤醒时不让 flash 冻结 core0 卡渲染/动作）。
//   IMU 运动更早的打断在 imuWoMISR（硬件中断）里，这里覆盖按键/App/摇拍分类后等所有活动。
static inline void markActivity() { g_lastActivityMs = millis(); g_persistAbortWrite = true; }

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

// 离开 L2 → 唤醒被信号量阻塞的非唤醒任务（LCD/AUD/IMU），瞬时恢复心跳
static void wakeRenderTasks()
{
    if (xLcdWake) xSemaphoreGive(xLcdWake);
    if (xAudWake) xSemaphoreGive(xAudWake);
    // ★ IMU 同理：它在 L2 会阻塞在 WoM 待机上等中断，自己不轮询功耗档位。
    //   不叫醒它的话，按键/App 把设备拉回 L0 后，IMU 仍停在 21Hz WoM 模式装死，
    //   拍一拍摇一摇全部失灵 —— 而待机是死等通知、没有超时兜底，这是唯一的救命通路。
    imuWake();
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
    // ★ 必须同时持"禁 light-sleep"锁：ESP_PM_CPU_FREQ_MAX 只锁定运行频率，
    //   并不阻止系统在任务空闲时自动进 light-sleep。此前传输/配网/串流期仍会
    //   偷偷进 light-sleep，导致 IMU 退 WoM 时的 I2C 写撞上 → ESP_ERR_INVALID_STATE(259)
    //   触发总线恢复，也会拖慢收包。真正做到注释所述"全程满电、与低功耗互斥"。
    setSleepLock(true);
}

// ── 落盘窗口的持有状态（引用计数式，必须严格配对）──
//   ★ 提成函数是因为 powerManagerLoop 有多个 return / 分支出口（forceActive 提前 return、
//     信封态分支……），任何一个出口漏掉 release 都会让计数泄漏、窗口永久开着、
//     延后落盘策略失效。所有出口统一调用本函数，杜绝遗漏。
static bool s_pmHoldsPersist = false;
static void persistWindowUpdate(bool want)
{
    if (want && !s_pmHoldsPersist) {
        // ★ 开窗即清打断标志：此刻已 idle≥65s（65s 无任何活动），之前那次唤醒留下的
        //   g_persistAbortWrite 已是陈旧信号，清掉让 persistTask 能开写/续写。
        g_persistAbortWrite = false;
        persistForceAcquire();
        s_pmHoldsPersist = true;
        LOG("[PWR] ⏸ 进 L2 满 5s，开窗落盘（保持 240MHz + 禁 light-sleep）\n");
    } else if (!want && s_pmHoldsPersist) {
        persistForceRelease();
        s_pmHoldsPersist = false;
        LOG("[PWR] ▶ 落盘窗口关闭，放行\n");
    }
}

static void powerManagerLoop()
{
    uint32_t now = millis();

    // ── 勿扰时段的"后台下载"：保持满电收包（不睡/满频），但【不亮屏、不重置活动计时】──
    //   勿扰时 App 发来的视频照收照存（后台），设备不点亮、不打扰。屏幕维持原省电态（多为 L2 息屏）。
    //   仅针对纯下载(xferLocked)——配网/录音/串流/连不上引导等本地或需可视的态仍走下面的 forceActive。
    //   本地按键 / IMU 拍摇经各自路径 markActivity 唤醒，不受此影响（勿扰只挡远程打扰）。
    if (g_dndActive && xferLocked &&
        !g_enterProvisioning && !Config.isProvMode() &&
        !micIsRecording() && !g_needWifiHint && !g_posStreaming) {
        persistWindowUpdate(false);
        cpuBoost(true);       // 满频，收得快
        setSleepLock(true);   // 禁 light-sleep（收包可靠 + 避免 IMU 退 WoM 的 I2C 撞睡眠 259）
        // ★ 不调 forceActive：不 lcdSetPowerMode(0)、不唤醒渲染、不重置 g_lastActivityMs
        //   → LCD 维持当前态，后台静默收，用户无感（勿扰下载完也不会亮 60s）。
        return;
    }

    // 配网 / 姿态串流 / 文件传输（xferLocked）/ 录音中：满电不睡
    if (g_enterProvisioning || Config.isProvMode() ||
        g_posStreaming || xferLocked || micIsRecording() || g_xferAnim ||
        g_needWifiHint) {
        persistWindowUpdate(false);   // ★ 这些状态不该落盘，且此路径提前 return，必须先关窗
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
    // ══════════════════════════════════════════════════════════════
    //  「落盘黄金窗口」= 进 L2 满 5s（idle≥65s）
    //    此刻屏幕已休眠停渲染、IMU 已进 WoM、播放早已停止（L2 = 60s 无活动，再等 5s 稳态），
    //    各任务也都已降到最低轮询频率——但【还没放锁进 light-sleep】。
    //    这是全系统最空闲、却仍全速(240MHz)的唯一时机：落盘对用户零感知、最快写完，
    //    且不会与 light-sleep 冲突（flash 撞 light-sleep 的坑同 IMU 的 I2C 那个 259）。
    //    落盘未完成前不放锁；写完后下一轮自然进 light-sleep，无需额外状态机。
    //    ★ 多等 5s：刚进 L2 就写、用户马上又醒会白打断一次；等 5s 稳态再写，抖动更少。
    //      被唤醒打断的落盘保留 off，下次进这个窗口续写（见 web.cpp persistWriteSlot）。
    //
    //  ★ 窗口管理走统一的 persistWindowUpdate()：本函数有多个出口（上面的 forceActive
    //    提前 return、下面的信封态分支），任一出口漏 release 都会让计数泄漏、窗口永久开着。
    //  ★ 信封态不开窗：那时屏幕 60% 亮着播信封动画，落盘会掉帧；等退出信封再写。
    // ══════════════════════════════════════════════════════════════
    bool persistWindow = (idle >= PWR_IDLE_L2_MS + 5000 &&
                          !envelope && !g_voiceSending && persistIsPending());
    persistWindowUpdate(persistWindow);

    if (envelope) {
        cpuBoost(false);        // 释放频率锁 → 40MHz
        setSleepLock(true);     // 但禁 light-sleep
    } else if (persistWindow) {
        cpuBoost(true);         // 保持满频，落盘最快
        setSleepLock(true);     // ★ 必须：CPU_FREQ_MAX 只锁频率，并不禁 light-sleep
    } else {
        cpuBoost(tier <= 1 || g_voiceSending);
        setSleepLock(false);    // 其他态不额外禁睡（保持原行为）
    }
}

// ★ ADC 配置顺序不能反（开机日志里那条 "Pin is not configured as analog channel" 的成因）：
//   analogSetPinAttenuation(pin,...) 走的是 esp32-hal-adc.c 的「重配已有通道」分支，
//   要求该脚已被注册为 ADC 通道(perimanGetPinBusType == ADC_ONESHOT)；而注册发生在
//   第一次 analogRead() 内部的 __analogInit()。先设衰减 → 报错 + 衰减【根本没设上】。
//   本例后果无害：Arduino 全局默认恰好就是 ADC_11db(esp32-hal-adc.c:62)，通道按默认值
//   初始化，读数一直是对的，只是每次开机白报一条错误。但不该依赖这个默认值——
//   哪天库改了默认档，分压后 ~2.4V 会被 0db(~950mV 量程) 削顶，电量直接读废。
static void batInit() {
    analogReadResolution(12);
    analogRead(BAT_ADC_PIN);                          // ① 先读一次 → 注册 ADC 通道
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);   // ② 通道已存在，这次才真正生效
    analogRead(BAT_ADC_PIN);                          // ③ 丢弃切换衰减后的首个样本

    // 充电状态：硬件电平判断（HIGH=正在充电，LOW=未充电）
    pinMode(CHARGE_STATUS_PIN, INPUT);
}
static uint32_t batReadAdcMv() {        // 分压后实测 mV（eFuse 校准）
    uint32_t sum = 0;
    for (int i = 0; i < BAT_SAMPLES; i++) sum += analogReadMilliVolts(BAT_ADC_PIN);
    return sum / BAT_SAMPLES;
}

// 纯净环境（batInit 未跑，如 runShutdownSleep）下的电压裸读：自带 ADC 配置。
static uint32_t readBatteryMvRaw() {
    analogReadResolution(12);
    analogRead(BAT_ADC_PIN);                          // 先注册通道，理由同 batInit()
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
    analogRead(BAT_ADC_PIN);                          // 丢弃切换衰减后的首个样本
    uint32_t sum = 0;
    for (int i = 0; i < BAT_SAMPLES; i++) sum += analogReadMilliVolts(BAT_ADC_PIN);
    uint32_t adcMv = sum / BAT_SAMPLES;
    return (uint32_t)(adcMv * BAT_DIVIDER + 0.5f);
}

// enterShutdown 定义在文件靠后（runShutdownSleep 之后）；batUpdate 低电检测需提前调用它。
static void enterShutdown();
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

// 充电插拔要立即反应：置位后 batUpdate 跳过 5s 节流、当轮立即重算一次。
static volatile bool s_batForce = false;

// 充电状态检测 + 时间去抖（loop 里每轮调用，与 batUpdate 的 5s 周期解耦）：
//   充电脚是数字电平，但插拔瞬间/接触不良会抖动，单次跳变就触发会误判。这里做时间去抖——
//   读到与当前稳定态不同的电平后，需【持续 CHG_DEBOUNCE_MS 一致】才确认切换、触发 batUpdate
//   重算；中途抖回原电平则重新计时。噪声/短暂跳变被过滤，用 ~1s 延迟换取稳定不乱跳。
#define CHG_DEBOUNCE_MS  1000    // 充电态切换去抖时长（约 1s）
static void batCheckChargeFast() {
    static int      stableChg = -1;   // 已确认的稳定充电态（-1=未定）
    static int      cand      = -1;   // 当前候选态
    static uint32_t candSince = 0;    // 候选态起始时刻

    int      chg = batIsCharging() ? 1 : 0;
    uint32_t now = millis();

    if (chg == stableChg) { cand = chg; return; }                 // 与稳定态相同 → 无变化
    if (chg != cand)      { cand = chg; candSince = now; return; } // 新候选 → 记起点，重新计时
    if (now - candSince >= CHG_DEBOUNCE_MS) {                      // 候选持续够久 → 确认切换
        stableChg  = chg;
        s_batForce = true;                                        // 触发 batUpdate 当轮重算
    }
}

static void batUpdate() {                 // 每 5s 一次（充电插拔时经 s_batForce 当轮立即触发）
    static uint32_t last = 0;
    if (!s_batForce && millis() - last < 5000) return;
    s_batForce = false;
    last = millis();

    uint32_t adcMv   = batReadAdcMv();
    uint32_t vbatRaw = (uint32_t)(adcMv * BAT_DIVIDER + 0.5f);
    bool     charging = batIsCharging();

    // ── 充电状态切换 → 清空电量记录，重新计算 ──
    //   充电期间读数虚高，不显示电量（App 显示"充电中"）。插上/拔下都清空平滑值，
    //   拔下后从新采样直接重新起算、立即显示（不做沉降等待，简单直接）。
    static bool  s_wasCharging = false;
    static float s_vbatEma     = 0;
    static bool  s_emaInit     = false;

    if (charging != s_wasCharging) { s_emaInit = false; }   // 插上/拔下都清空平滑记录
    s_wasCharging = charging;
    g_batCharging = charging;
    g_batMv       = vbatRaw;

    uint32_t vbatSmooth = 0;   // 有效平滑电压（仅放电态有值，供低电检测用）

    if (charging) {
        g_batPercent = -1;                      // 充电中：不显示（App 靠 chg 显示"充电中"）
    } else {
        // 放电（含刚拔下）：EMA 平滑（0.2 新 + 0.8 旧）后换算百分比，直接显示真实电量。
        //   不做单调锁定——电压回弹时百分比允许回升，跟随真实电压。平滑本身已抑制跳动。
        //   刚拔下时 emaInit=false → 用当前采样初始化，立即显示。
        if (!s_emaInit) { s_vbatEma = vbatRaw; s_emaInit = true; }
        else            { s_vbatEma = 0.2f * vbatRaw + 0.8f * s_vbatEma; }
        vbatSmooth   = (uint32_t)(s_vbatEma + 0.5f);
        g_batPercent = batMvToPercent(vbatSmooth);
    }

    LOG("[BAT] %s | %u.%02uV | %d%% | ADC %umV\n",
        charging ? "充电中" : "电池",
        vbatRaw/1000, (vbatRaw%1000)/10, g_batPercent, adcMv);

    // ── 低电自动关机：仅放电态用平滑电压判据（充电态不检测、计数清零）──
    //   连续计数防瞬时压降误关（舵机摆动等大电流会短暂拉低电压，下次采样即恢复）。
    static int lowCnt = 0;
    if (vbatSmooth > 0 && vbatSmooth <= BAT_SHUTDOWN_MV) {
        if (++lowCnt >= BAT_LOW_CONFIRM_CNT) {
            LOG("[BAT] ★ 连续 %d 次 ≤%u mV → 低电自动关机\n", lowCnt, (unsigned)BAT_SHUTDOWN_MV);
            enterShutdown();   // 置标志 + 硬复位进关机休眠，不返回
        }
    } else {
        lowCnt = 0;
    }
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
//  关机（复位到纯净环境 + light-sleep 模拟）
//    · 长按 Button1(IO2) 8 秒 → 关机：置 RTC 关机标志 → esp_restart 硬复位。
//    · 复位后 setup【最早期、任何外设初始化之前】检测到标志 → 进入关机休眠(runShutdownSleep)：
//      定时唤醒轮询按键（本板 GPIO 中断唤醒不可靠），纯净环境下平均电流≈纯 light-sleep。
//    · 休眠态下长按 Button1 3 秒 → esp_restart 正常开机。
//
//  为什么走"复位再休眠"而不是原地休眠：
//    原地在 loopTask 里挂任务 + 关 BLE 会崩——关机前 2s 触发的配网让 NimBLE 在 Core 0 活跃，
//    从 loopTask 去 deinit 它会跳到已释放的空回调(PC=0, InstrFetchProhibited)。
//    esp_restart 把 BLE/WiFi 硬件一并干净复位，复位后的纯净环境既不会崩、又天然低功耗。
// ══════════════════════════════════════════════════════════════
#define SHUTDOWN_LONG_PRESS_MS   8000   // 长按 Button1 8s → 关机（关机态下改单击开机，无长按阈值）

// 关机标志：放 RTC 内存 + 配合复位原因判定，跨 esp_restart 传递"请进入关机休眠"。
// 用 NOINIT + 复位原因门控：上电(POWERON)时其值是垃圾、但被复位原因挡掉；只有
// esp_restart(SW 复位) 且值==MAGIC 才认。
RTC_NOINIT_ATTR static uint32_t s_shutdownFlag;
#define SHUTDOWN_MAGIC   0x5D0117AAu

// ══════════════════════════════════════════════════════════════
//  健康黑匣子（RTC 慢速内存）
//
//  为什么放 RTC 内存：串口 LOG 在这个板子上不可用于长期监控 ——
//  ARDUINO_USB_CDC_ON_BOOT=1 时 Serial 走 USB CDC，设备一进 light-sleep
//  USB 外设掉电、主机端连接断开，之后所有日志都收不到。而 RTC_NOINIT 段在
//  light-sleep / deep-sleep / 软复位 / TWDT panic 重启 之后【全部保留】
//  （只有真正掉电才丢），是唯一能跨越崩溃把现场带出来的存储。
//
//  记录的是【历史最差值】而不是当前值：崩溃前最后一刻的采样多半发不出去，
//  但"曾经跌到过多少"能跨重启累积，重启后由 webTask 通过 MQTT 上报。
//  碎片化就是靠 minLargestBlk 的长期走势判断的。
// ══════════════════════════════════════════════════════════════
// 内部 RAM"最大连续块"低水位告警线。与资源监控里那条 ⚠⚠ 日志同值：
// I2S DMA 环(16KB)、MQTT 缓冲(8KB) 都在这个量级，跌破即意味着大块分配随时会失败。
#define HEALTH_LOWBLK_WARN  (20 * 1024)

#define HEALTH_MAGIC  0x484C5404u      // "HL\4"，结构变更时改这个数使旧数据失效
                                       // （\2→\3：新增 lastStage/curStage）
                                       // （\3→\4：新增 TWDT 现场 + 运行期阶段 + 低水位快照）

typedef struct {
    uint32_t magic;
    uint32_t bootCount;        // 累计启动次数
    uint32_t crashCount;       // 非正常复位次数（panic / 各种看门狗 / 欠压）
    uint32_t lastReset;        // 上次复位原因 esp_reset_reason()
    uint32_t lastUptimeSec;    // 本次已运行多久（healthBoxSample 每 5s 刷新）
    // ★ 上次【崩溃前】撑了多久。必须单独存一格：lastUptimeSec 会被本次运行每 5s 覆盖，
    //   等到 MQTT 连上去上报时它早就等于本次的 up 了 —— 直接报它等于报了个废值。
    //   healthBoxInit 在本次第一次采样之前把它转存到这里，语义才成立。
    uint32_t prevUptimeSec;
    uint32_t maxUptimeSec;     // 历史最长连续运行
    uint32_t minFreeHeap;      // 历史最小空闲内部堆
    uint32_t minLargestBlk;    // ★ 历史最小"最大连续块" —— 碎片化的关键指标
    uint32_t minFreePsram;
    uint32_t minPsramBlk;
    uint16_t stkMinFree[6];    // 各任务栈历史最小剩余（loop/IMU/WEB/LCD/AUD/MIC）
    uint32_t selfHealHits;     // 语音子系统自检命中次数（>0 = 有未预料的卡死路径）
    // ── 启动阶段标记（见 health.hpp 的成因说明）──
    uint32_t curStage;         // 本次进行到哪一步（setup 边走边写）
    uint32_t lastStage;        // 上次【停在】哪一步 —— 崩溃现场的唯一线索

    // ── 运行期阶段：每任务一格，见 health.hpp 的 HealthTaskSlot ──
    uint8_t  taskStage[HT_SLOTS];   // 本次运行的实时值（被 TWDT 的 ISR 读取）
    uint8_t  wdtStage[HT_SLOTS];    // ★ 看门狗超时【那一刻】的快照

    uint32_t wdtCount;         // TWDT 累计触发次数（跨重启）

    // ── 内部 RAM 低水位快照 ──
    //   原来只记 minLargestBlk 一个孤立数字，看得到"曾经跌到 1908"，却不知道
    //   何时跌的、当时在干什么。这里在【首次跌破阈值】那一刻抓一份现场。
    uint32_t lowBlkAt;         // 发生时刻（uptime 秒；0 = 从未发生）
    uint32_t lowBlkBoot;       // 发生在第几次启动 —— 快照跨重启保留，缺了它就分不清
                               //   "3000 秒" 是本次的还是三次重启之前的
    uint32_t lowBlkVal;        // 当时的最大连续块
    uint32_t lowBlkHeap;       // 当时的空闲内部堆
    uint8_t  lowBlkStage[HT_SLOTS];  // 当时各任务在干什么
} HealthBox;
RTC_NOINIT_ATTR static HealthBox g_hb;

// 打一个启动阶段点。★ 必须是真实写入 RTC 内存的一条 store：g_hb 在 RTC_NOINIT 段，
// panic / 看门狗 / 软复位都不清它，所以赋值语句一执行完，现场就已经保住了。
// ★ 编译屏障：这个值【只写不读】，正常控制流下没有任何代码依赖它，理论上允许编译器
//   把连续几次赋值合并成最后一次。而本机制的全部价值恰恰在于"崩溃那一刻它是多少"。
//   屏障成本为零（不生成指令），只是禁止跨过它做重排/合并。不用 volatile 是因为
//   那会让 healthBoxInit 里的 memset(&g_hb,...) 触碰 volatile 成员，属于未定义行为。
void healthSetStage(uint32_t stage)
{
    g_hb.curStage = stage;
    asm volatile("" ::: "memory");
}

// 运行期阶段打点：每任务一格，见 health.hpp。成本 = 一条 store，可放热路径。
// 越界静默忽略（宁可少一条诊断信息，也不能让诊断代码本身写坏内存）。
void healthSetTaskStage(uint8_t slot, uint8_t stage)
{
    if (slot >= HT_SLOTS) return;
    g_hb.taskStage[slot] = stage;
    asm volatile("" ::: "memory");
}

// ══════════════════════════════════════════════════════════════
//  TWDT 超时现场（覆盖 IDF 的 weak 钩子，超时时在中断上下文被调用）
//
//  解决黑匣子最大的盲区：reset="任务看门狗" 只说明"有人没喂狗"，答不出是谁、
//  更答不出当时在干什么。这里把 taskStage 冻成快照，重启后随健康 JSON 带出来。
//
//  ★★ 本函数【只做 RTC 内存读写】，一条外部函数都不调 —— 这是刻意的：
//     我们最怀疑的故障场景正是 persistTask 写 flash 导致 core0 cache 冻结，
//     那种时刻任何落在 flash 里的代码（pcTaskGetName / xTaskGetCurrentTaskHandleForCore
//     都在 flash）一旦被取指就会二次崩溃 —— 诊断代码反而把原始现场掩盖成 panic。
//     RTC slow memory 不经 flash cache，读写在任何上下文都成立。
//     代价是拿不到任务名，但 taskStage 已经能回答"每个任务卡在哪一步"，够用。
//  ★ 同理不取内存水位：heap_caps_* 要拿堆锁，在 ISR 里可能死锁。
// ══════════════════════════════════════════════════════════════
extern "C" void esp_task_wdt_isr_user_handler(void)
{
    g_hb.wdtCount++;
    for (int i = 0; i < HT_SLOTS; i++) g_hb.wdtStage[i] = g_hb.taskStage[i];
}

const char* healthStageName(uint32_t s)
{
    if (s >= BS_LCD_PRELOAD && s < BS_LCD_PRELOAD + 8) {
        // 与 lcd.cpp 的 preloadList 顺序严格一致（改那边记得同步改这里）
        static const char* kVid[8] = { "idle", "nod", "swingSwing", "showUp",
                                       "wink", "startSleep", "sleeping", "target" };
        static char buf[40];
        snprintf(buf, sizeof(buf), "预加载视频[%u] %s",
                 (unsigned)(s - BS_LCD_PRELOAD), kVid[s - BS_LCD_PRELOAD]);
        return buf;
    }
    switch (s) {
        case BS_NONE:     return "无记录";
        case BS_FS:       return "挂载LittleFS";
        case BS_SN:       return "SN校验";
        case BS_PM:       return "电源管理";
        case BS_CFG_BLE:  return "配置+BLE";
        case BS_LCD_INIT: return "LCD初始化";
        case BS_LCD_DONE: return "预加载收尾";
        case BS_AUD:      return "音频初始化";
        case BS_MIC:      return "麦克风初始化";
        case BS_IMU:      return "IMU初始化";
        case BS_SERVO:    return "舵机上电归中位";
        case BS_BAT:      return "电量ADC";
        case BS_QUEUE:    return "创建队列";
        case BS_TASKS:    return "创建任务";
        case BS_RUNNING:  return "运行期";
        case BS_SHUTDOWN: return "正常关机";
        default:          return "未知";
    }
}

volatile uint32_t g_selfHealHits = 0;   // web.cpp 的 voiceSelfCheck 命中时 ++

// web.cpp / lcd.cpp 的诊断量（定义在各自文件，这里只读）
extern volatile uint32_t g_lastAnyRxMs, g_mqttFailCnt, g_mqttFailBlk;
extern volatile int32_t  g_mqttLastErr;
extern volatile uint32_t g_netRecoverCnt, g_netRebootCnt;
// g_sprFail 原定义在 lcd.cpp 的 sprite 预分配逻辑里，该逻辑已回退。这里就地占位（恒为 0），
// 保持健康上报 JSON 的字段结构不变；日后重做 sprite 预分配时再把定义移回 lcd.cpp。
volatile uint32_t g_sprFail = 0;

// 内部 RAM 的空闲块数量 —— 碎片化最直观的指标（块多且小 = 碎）
static uint32_t healthFreeBlocks() {
    multi_heap_info_t hi;
    heap_caps_get_info(&hi, MALLOC_CAP_INTERNAL);
    return (uint32_t)hi.free_blocks;
}

// 哪些算「异常复位」（计入 crashCount）。
//   ★ PWR_GLITCH / CPU_LOCKUP 必须计入：前者是芯片的电源毛刺检测电路触发的
//     （插拔 Type-C 那一瞬间的电压突变最容易命中），后者是双重异常把 CPU 锁死，
//     两者都是真故障。原来漏了它们，故障会被静默计成"正常"。
//   ★ USB / JTAG 不计入：那是把板子插到电脑上时，主机枚举 USB-Serial-JTAG 并
//     toggle DTR/RTS 触发的复位（S3 内置的自动下载电路），属于正常开发操作，
//     算进 crashCount 只会把调试噪声混进故障统计。
static bool healthIsCrash(uint32_t r) {
    return r == ESP_RST_PANIC      || r == ESP_RST_INT_WDT  || r == ESP_RST_TASK_WDT ||
           r == ESP_RST_WDT        || r == ESP_RST_BROWNOUT ||
           r == ESP_RST_PWR_GLITCH || r == ESP_RST_CPU_LOCKUP;
}
// ★ 必须覆盖 esp_reset_reason_t 的全部取值：漏掉的一律落到 "未知"，
//   而"未知"对排查毫无帮助 —— 实测就吃过亏：插 Type-C 复位查不出原因，
//   正是因为 PWR_GLITCH / USB 这两种没列出来，全被归进了"未知"。
static const char* healthResetName(uint32_t r) {
    switch (r) {
        case ESP_RST_POWERON:    return "上电";
        case ESP_RST_EXT:        return "外部复位脚";
        case ESP_RST_SW:         return "软复位";
        case ESP_RST_PANIC:      return "崩溃(panic)";
        case ESP_RST_INT_WDT:    return "中断看门狗";
        case ESP_RST_TASK_WDT:   return "任务看门狗";
        case ESP_RST_WDT:        return "其它看门狗";
        case ESP_RST_DEEPSLEEP:  return "深睡唤醒";
        case ESP_RST_BROWNOUT:   return "欠压";
        case ESP_RST_SDIO:       return "SDIO";
        case ESP_RST_USB:        return "USB外设复位";   // 插电脑时主机 DTR/RTS 触发，正常
        case ESP_RST_JTAG:       return "JTAG";
        case ESP_RST_EFUSE:      return "eFuse错误";
        case ESP_RST_PWR_GLITCH: return "电源毛刺";       // 插拔电源瞬间的电压突变
        case ESP_RST_CPU_LOCKUP: return "CPU死锁";        // 双重异常
        default:                 return "未知";
    }
}

// setup() 里尽早调用（在关机休眠判定之后、外设初始化之前）
static void healthBoxInit() {
    uint32_t reason = esp_reset_reason();
    if (g_hb.magic != HEALTH_MAGIC) {          // 首次上电 / 结构变更 → 全新初始化
        memset(&g_hb, 0, sizeof(g_hb));
        g_hb.magic         = HEALTH_MAGIC;
        g_hb.minFreeHeap   = 0xFFFFFFFFu;
        g_hb.minLargestBlk = 0xFFFFFFFFu;
        g_hb.minFreePsram  = 0xFFFFFFFFu;
        g_hb.minPsramBlk   = 0xFFFFFFFFu;
        for (int i = 0; i < 6; i++) g_hb.stkMinFree[i] = 0xFFFF;
    }
    // ★ 转存启动阶段：此刻 curStage 还是【上次停下时】的值（RTC 不被复位清除），
    //   必须在本次 setup 打第一个点之前取走，否则就被覆盖了。
    g_hb.lastStage = g_hb.curStage;
    g_hb.curStage  = BS_NONE;
    // 同理转存"上次撑了多久"（本次的第一次 healthBoxSample 还没跑，此刻它仍是上次的值）
    g_hb.prevUptimeSec = g_hb.lastUptimeSec;
    // 运行期阶段清零：上次的值已由 wdtStage 冻在快照里，这里不该残留干扰本次判断
    for (int i = 0; i < HT_SLOTS; i++) g_hb.taskStage[i] = 0;

    g_hb.bootCount++;
    g_hb.lastReset = reason;
    if (healthIsCrash(reason)) g_hb.crashCount++;
    LOG("[HB] 第 %u 次启动 | 本次复位=%s | 累计异常复位=%u | 上次运行=%us(最长 %us)\n",
        (unsigned)g_hb.bootCount, healthResetName(reason),
        (unsigned)g_hb.crashCount, (unsigned)g_hb.lastUptimeSec, (unsigned)g_hb.maxUptimeSec);
    // ★ 这一行才是启动期崩溃的现场：告诉你上次停在哪一步，不需要串口。
    //   lastStage < BS_RUNNING = 上次没跑完 setup（启动期崩溃）；
    //   == BS_RUNNING = 崩在运行期；== BS_SHUTDOWN = 正常关机，不是故障。
    if (healthIsCrash(reason) || g_hb.lastStage != BS_RUNNING) {
        LOG("[HB] ★ 上次停在: %s (stage=%u)%s\n",
            healthStageName(g_hb.lastStage), (unsigned)g_hb.lastStage,
            (g_hb.lastStage != BS_RUNNING && g_hb.lastStage != BS_SHUTDOWN &&
             g_hb.lastStage != BS_NONE) ? "  ← 启动期崩溃！" : "");
    }
}

// 资源监控里每 5s 调用：把当前值并入历史最差值
static void healthBoxSample() {
    uint32_t up = millis() / 1000;
    g_hb.lastUptimeSec = up;                       // 崩溃后这就是"撑了多久"
    if (up > g_hb.maxUptimeSec) g_hb.maxUptimeSec = up;

    size_t v;
    size_t iFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    if (iFree < g_hb.minFreeHeap)   g_hb.minFreeHeap   = iFree;
    size_t iBlk  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    if (iBlk < g_hb.minLargestBlk)  g_hb.minLargestBlk = iBlk;

    // ★ 低水位快照：只在【首次】跌破阈值时抓，记的是"第一次出事时系统在干什么"。
    //   原来只有 minLargestBlk 一个孤立数字 —— 看得到曾经跌到 1908B，却不知道何时跌的、
    //   当时哪个任务在做什么，等于知道有病却查不出病灶。
    //   不重复抓：第一次跌破往往就是根因现场，后面的多是被它拖出来的连锁反应。
    if (iBlk < HEALTH_LOWBLK_WARN && g_hb.lowBlkAt == 0) {
        g_hb.lowBlkAt   = up ? up : 1;    // 0 有"从未发生"的语义，故最小记 1
        g_hb.lowBlkBoot = g_hb.bootCount;
        g_hb.lowBlkVal  = iBlk;
        g_hb.lowBlkHeap = iFree;
        for (int i = 0; i < HT_SLOTS; i++) g_hb.lowBlkStage[i] = g_hb.taskStage[i];
        LOG("[HB] ⚠⚠ 内部RAM 最大连续块跌破 %uKB（现 %u B，空闲堆 %u B）→ 已抓低水位快照\n",
            (unsigned)(HEALTH_LOWBLK_WARN / 1024), (unsigned)iBlk, (unsigned)iFree);
    }

    v = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    if (v < g_hb.minFreePsram)  g_hb.minFreePsram  = v;
    v = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    if (v < g_hb.minPsramBlk)   g_hb.minPsramBlk   = v;

    TaskHandle_t hs[6] = { xTaskGetCurrentTaskHandle(), xImuTaskHandle, xWebTaskHandle,
                           xLcdTaskHandle, xAudTaskHandle, xMicTaskHandle };
    for (int i = 0; i < 6; i++) {
        if (!hs[i]) continue;
        uint32_t f = uxTaskGetStackHighWaterMark(hs[i]) * sizeof(StackType_t);
        if (f > 0xFFFF) f = 0xFFFF;
        if (f < g_hb.stkMinFree[i]) g_hb.stkMinFree[i] = (uint16_t)f;
    }
    g_hb.selfHealHits = g_selfHealHits;
}

// 供 web.cpp 组 MQTT 上报用。键名短，一条 MQTT 消息装得下。
// 语义：boot/crash/reset/up/maxUp 是跨重启累积；min* 是历史最差值；stk 是各任务栈历史最小剩余。
void healthBoxJson(char* out, size_t n) {
    snprintf(out, n,
        "{\"boot\":%u,\"crash\":%u,\"reset\":\"%s\","
        // ★ stage 紧跟 reset：这两个字段合起来才是完整现场（"为什么复位" + "停在哪"）。
        //   放最前面也是防截断——万一将来字段加多了超出缓冲，先保住的是诊断价值最高的。
        "\"stage\":%u,\"stageAt\":\"%s\","
        // ★ TWDT 现场：wdt=累计触发次数，wdtTs=超时【那一刻】各任务卡在哪一步
        //   （下标 web/lcd/imu/mic/persist，码值见 health.hpp）。
        //   reset="任务看门狗" 时先看这个数组，直接指认是谁没喂狗、卡在哪。
        "\"wdt\":%u,\"wdtTs\":[%u,%u,%u,%u,%u],"
        // lastUp = 上次【崩溃前撑了多久】。几分钟 vs 十几小时是完全不同的故障模式，
        // 原来结构里存了却没上报，等于白记。
        "\"up\":%u,\"lastUp\":%u,\"maxUp\":%u,"
        "\"heapNow\":%u,\"heapMin\":%u,\"blkNow\":%u,\"blkMin\":%u,"
        // ★ 低水位快照：at=发生时刻(uptime 秒，0=从未发生)，blk/heap=当时的值，
        //   ts=当时各任务在干什么。回答"内存是什么时候、被谁吃掉的"。
        "\"lowBlk\":{\"boot\":%u,\"at\":%u,\"blk\":%u,\"heap\":%u,\"ts\":[%u,%u,%u,%u,%u]},"
        "\"psMin\":%u,\"psBlkMin\":%u,\"heal\":%u,"
        "\"blkN\":%u,\"fsUsed\":%u,\"fsTotal\":%u,"
        "\"net\":{\"up\":%d,\"noRx\":%u,\"fail\":%u,\"err\":%d,\"failBlk\":%u,"
        "\"recov\":%u,\"reboot\":%u},\"sprFail\":%u,"
        "\"stk\":[%u,%u,%u,%u,%u,%u]}",
        (unsigned)g_hb.bootCount, (unsigned)g_hb.crashCount,
        healthResetName(g_hb.lastReset),
        (unsigned)g_hb.lastStage, healthStageName(g_hb.lastStage),
        (unsigned)g_hb.wdtCount,
        g_hb.wdtStage[0], g_hb.wdtStage[1], g_hb.wdtStage[2],
        g_hb.wdtStage[3], g_hb.wdtStage[4],
        (unsigned)(millis() / 1000), (unsigned)g_hb.prevUptimeSec,
        (unsigned)g_hb.maxUptimeSec,
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)g_hb.minFreeHeap,
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
        (unsigned)g_hb.minLargestBlk,
        (unsigned)g_hb.lowBlkBoot, (unsigned)g_hb.lowBlkAt,
        (unsigned)g_hb.lowBlkVal, (unsigned)g_hb.lowBlkHeap,
        g_hb.lowBlkStage[0], g_hb.lowBlkStage[1], g_hb.lowBlkStage[2],
        g_hb.lowBlkStage[3], g_hb.lowBlkStage[4],
        (unsigned)g_hb.minFreePsram, (unsigned)g_hb.minPsramBlk,
        (unsigned)g_hb.selfHealHits,
        // ★ blkN = 内部 RAM 的空闲块【数量】—— 比"碎片率"更能说明问题：
        //   3 块平均 20KB = 健康；80 块平均 750B = 严重碎片（剩余总量可能一样）。
        (unsigned)healthFreeBlocks(),
        // LittleFS 用量：语音(最多10条×240KB)+下载的视频/音频都写在这里。
        // 剩余空间不足时 LittleFS 写入会急剧变慢甚至失败，而写 flash 会冻结 core0 cache。
        (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes(),
        // ── 网络诊断 ──
        //   noRx 几百秒 = 设备根本没收到东西（网络断了）；只有几秒 = 网络正常，问题在别处。
        //   err=-2 且 failBlk 很小 = TLS 握手时内存不够；err=4/5 = 认证/ACL，与内存无关。
        (int)(WiFi.status() == WL_CONNECTED),
        (unsigned)(g_lastAnyRxMs ? (millis() - g_lastAnyRxMs) / 1000 : 9999),
        (unsigned)g_mqttFailCnt, (int)g_mqttLastErr, (unsigned)g_mqttFailBlk,
        (unsigned)g_netRecoverCnt, (unsigned)g_netRebootCnt,
        (unsigned)g_sprFail,
        g_hb.stkMinFree[0], g_hb.stkMinFree[1], g_hb.stkMinFree[2],
        g_hb.stkMinFree[3], g_hb.stkMinFree[4], g_hb.stkMinFree[5]);
}

// 等 Button1 松开（arm 唤醒前调用）：触发关机/校验失败时按钮仍被按住(LOW)，
// 不等松手就 arm 低电平唤醒会立即再次自唤醒。
// ★ 要求"持续高电平 150ms"才算真松开：只看一次高电平就 arm 的话，机械抖动的下一个
//   低电平尖峰会正好自触发唤醒。
static void waitButtonRelease()
{
    uint32_t highStart = 0;
    while (true) {
        if (digitalRead(PROV_BUTTON_PIN) == HIGH) {
            if (highStart == 0) highStart = millis();
            if (millis() - highStart >= 150) break;   // 连续高 150ms = 真正松开
        } else {
            highStart = 0;                            // 见到低电平就重新计时（抖动未停）
        }
        delay(10);
    }
}

// 关机休眠期按键唤醒：ISR 通知本任务从（自动）light-sleep 醒来。
static TaskHandle_t s_shutdownWakeTask = nullptr;
static void IRAM_ATTR shutdownBtnISR() {
    if (s_shutdownWakeTask) {
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(s_shutdownWakeTask, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

// 关机休眠态：在 setup() 早期、纯净环境（BLE/WiFi/各任务均未初始化）下调用。
//   关屏 + 麦克风省电，然后【阻塞等按键中断】——睡到被按下为止（中间零唤醒，最低功耗）；
//   单击 → 复检电量：够电则开机，仍没电则显示没电图标 3s 后回休眠。不返回（唯一出口是 esp_restart）。
static void runShutdownSleep()
{
    LOG("[PWR] ════ 关机休眠态（单击开机）════\n");

    // ── 关外设：这些外挂芯片默认态/上一轮 init 后的状态不会被 esp_restart 复位，
    //    不显式关就会留几 mA 静态耗电。全是纯 GPIO 操作，无需各自的 init。──
    // 背光 LED（主耗电）：拉低关灯
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, LOW);
    // LCD 面板：复位脚拉低保持复位，停止驱动像素（面板逻辑再省几 mA）
    pinMode(LCD_RST, OUTPUT);
    digitalWrite(LCD_RST, LOW);
    // 音频功放 MAX98357A：SD 脚拉低 = 关闭（默认高阻可能被使能，静态耗几 mA）
    pinMode(AUD_SHDN, OUTPUT);
    digitalWrite(AUD_SHDN, LOW);
    // 舵机供电轨高边开关：拉低 = 断电（默认可能常通）
    pinMode(SERVO_POWER_PIN, OUTPUT);
    digitalWrite(SERVO_POWER_PIN, LOW);
    // IMU(QMI8658)：外挂 I2C 芯片，esp_restart 不复位它，会保留上一轮工作模式(~1.5mA)。
    // 自起最小 I2C 把它关进 Power-Down(~6µA)。只写一次，之后不再碰 I2C（避免撞 light-sleep）。
    imuShutdownStandalone();

    // 麦克风省电（CLK=IO48 拉 HIGH，DATA=IO47 输入下拉）——纯 GPIO 操作，无需 initMIC
    gpio_reset_pin(GPIO_NUM_48);
    gpio_set_direction(GPIO_NUM_48, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_48, 1);
    gpio_reset_pin(GPIO_NUM_47);
    gpio_set_direction(GPIO_NUM_47, GPIO_MODE_INPUT);
    gpio_pulldown_en(GPIO_NUM_47);

    // 按钮：输入上拉 + 注册为 light-sleep 唤醒源（低电平=按下）+ 边沿中断通知本任务。
    //   ★ 中断唤醒机制与 L2/IMU 相同（自动 light-sleep 下 gpio_wakeup 能把 CPU 唤醒）。
    //   ★★ 依赖 IO2 有稳定高电平（本板 IO2→3V3 外部上拉），否则悬空脚会被 LOW_LEVEL 唤醒源
    //      反复误触发、电流跳变。
    pinMode(PROV_BUTTON_PIN, INPUT_PULLUP);
    // 显式设定 light-sleep 期该脚为 输入+上拉（IDF 给睡眠期唤醒 GPIO 配上拉的正规接口）。
    gpio_sleep_set_direction((gpio_num_t)PROV_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_sleep_set_pull_mode((gpio_num_t)PROV_BUTTON_PIN, GPIO_PULLUP_ONLY);
    waitButtonRelease();     // 先等松手，否则按下电平会立刻自唤醒

    // GPIO 中断唤醒（电平 LOW_LEVEL 把 CPU 从 light-sleep 唤醒 + FALLING 中断 notify 任务）。
    //   ★ 单击开机，不长按：长按(持续低电平)会让 LOW_LEVEL 电平中断被【持续触发】→ core1
    //     中断风暴 → Interrupt WDT panic；单击松手快，电平中断只闪一下(远小于 WDT 超时)，安全。
    //   ★ 唤醒后第一件事就是 detach + 关唤醒源，杜绝后续风暴，然后直接按"单击=开机意图"处理。
    s_shutdownWakeTask = xTaskGetCurrentTaskHandle();
    attachInterrupt(digitalPinToInterrupt(PROV_BUTTON_PIN), shutdownBtnISR, FALLING);
    gpio_wakeup_enable((gpio_num_t)PROV_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    // 开自动 light-sleep（与 L2 同一低功耗机制，睡时地板实测 2.2mA）
    {
        esp_pm_config_t pm = { .max_freq_mhz = 240, .min_freq_mhz = 40, .light_sleep_enable = true };
        esp_pm_configure(&pm);
    }

    LOG("[PWR] 进入关机 light-sleep（单击按键开机）...\n");
    Serial.flush();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // 睡到被按键唤醒（单击即触发）

        // ★ 唤醒后立即拆中断 + 关唤醒源：单击松手快、电平中断只短暂闪动不会到 WDT 超时；
        //   拆掉后即便偶尔按稍久也不再风暴。之后按"单击 = 开机意图"处理，不再长按判断。
        detachInterrupt(digitalPinToInterrupt(PROV_BUTTON_PIN));
        gpio_wakeup_disable((gpio_num_t)PROV_BUTTON_PIN);

        delay(30);   // 简单去抖（等抖动稳定；单击此时可能已松手，无需再确认电平）

        // 单击唤醒 = 开机意图 → 复检电量：够电（或正在充电）则开机；仍没电则亮屏提示 3s 后回休眠。
        uint32_t vbat = readBatteryMvRaw();
        // ★ 纯净环境下 batInit() 尚未执行，充电脚还没 pinMode，必须先配置再读，
        //   否则 digitalRead 拿到的是未使能输入缓冲的不确定值（同 readBatteryMvRaw 自带 ADC 配置）。
        pinMode(CHARGE_STATUS_PIN, INPUT);
        bool charging = batIsCharging();
        LOG("[PWR] ✓ 单击唤醒，复检电压 %u.%02uV%s\n",
            vbat/1000, (vbat%1000)/10, charging ? "（充电中）" : "");
        // ★ 用高于关机阈值的 BAT_WAKE_MV 判定，形成迟滞，避免"回弹开机→负载压回→再关机"循环。
        // ★ 正在充电则无条件放行：深度亏电时涓流预充电压半天到不了阈值，若不放行，用户插着
        //   充电器也开不了机、只看到"没电"图标，会误判设备损坏。充电中开机是安全的——
        //   batUpdate 在充电时不做低电关机判定（vbatSmooth 保持 0，判据恒不成立）。
        if (vbat > BAT_WAKE_MV || charging) {
            LOG("[PWR] %s → 正常开机\n", charging ? "充电中" : "电量足够");
            Serial.flush();
            delay(50);
            esp_restart();               // 关机标志已清，正常启动。不返回。
        }
        // 仍 ≤6.4V：先关 light-sleep（否则显示期 delay 会进睡、背光 PWM 停），
        //   亮屏（亮度 50）显示没电图标 3s，再重新置标志回关机休眠。
        LOG("[PWR] 仍 ≤%u mV → 显示没电提示 3s 后回休眠\n", (unsigned)BAT_SHUTDOWN_MV);
        {
            esp_pm_config_t pmAwake = { .max_freq_mhz = 240, .min_freq_mhz = 240, .light_sleep_enable = false };
            esp_pm_configure(&pmAwake);
        }
        lcdShowLowBatteryScreen(3000);
        s_shutdownFlag = SHUTDOWN_MAGIC;   // 复位后 setup 早期再次进入关机休眠
        Serial.flush();
        delay(50);
        esp_restart();                     // 不返回
    }
}

// 长按 8s 触发：不在此处关外设/关 BLE（跨核 deinit 会崩），改为置标志 + 硬复位，
// 复位后由 setup 早期的 runShutdownSleep 在纯净环境处理。
static void enterShutdown()
{
    LOG("[PWR] ★ 进入关机（长按 8s 或低电触发）→ 置标志 + 硬复位\n");

    // ★ 标记"这是正常关机"：否则下次开机会看到 lastStage=运行期 + reset=软复位，
    //   与真实故障混在一起。关机休眠期间 healthBoxInit 不执行（runShutdownSleep 不返回），
    //   所以这个值会一直保留到用户单击开机后的那次 setup 才被读走。
    healthSetStage(BS_SHUTDOWN);

    // ★ 先把延后的落盘写完（可能还有刚收到的视频只在 PSRAM 里，复位会丢）。
    persistFlushBlocking(15000);

    s_shutdownFlag = SHUTDOWN_MAGIC;   // RTC 标志：复位后 setup 据此进关机休眠
    LOG("[PWR] 置关机标志 → 硬复位进入关机休眠\n");
    Serial.flush();
    delay(50);
    esp_restart();          // 硬复位（BLE/WiFi 硬件一并干净复位，不会崩）。不返回。
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

    // ★ 关机休眠：若本次是关机触发的复位（esp_restart(SW) 且 RTC 标志==MAGIC），在【任何外设
    //   初始化之前】就进入关机休眠——LCD/音频/舵机/IMU/BLE/WiFi/各任务全都不启动，电流最低。
    //   上电(POWERON) 时 RTC 值是垃圾、但被复位原因挡掉。runShutdownSleep 不返回（长按开机会 esp_restart）。
    if (esp_reset_reason() == ESP_RST_SW && s_shutdownFlag == SHUTDOWN_MAGIC) {
        s_shutdownFlag = 0;   // 消费标志：本次若长按开机，esp_restart 后即正常启动
        LOG("[BOOT] 检测到关机标志 → 进入关机休眠态\n");
        runShutdownSleep();   // 不返回
    }
    s_shutdownFlag = 0;       // 正常启动路径也清掉，避免残留误判

    // ★ 健康黑匣子：必须在这里初始化（关机休眠判定之后、外设初始化之前），
    //   这样"上次为什么挂的"在任何可能再次崩溃的代码跑起来之前就已经记下。
    healthBoxInit();

    // ★ OTA：若 NVS 标记要求升级 → 进入纯净下载环境（不返回，内部 esp_restart）；
    //   或处理"新固件启动失败被回滚"的残留。放在任何常规外设初始化【之前】——
    //   OTA 环境只用 NVS/WiFi/LCD(轻量进度屏)/HTTPS，内存最宽裕、无 flash/PM/看门狗冲突。
    otaBootCheck();

    // ★ 尽早压灭背光：从上电到 LCD 首帧画好之前保持全灭。否则 GPIO17 上电浮空可能微亮、
    //   且 lcdInit 的 tft.init() 一结束就点亮背光，而那时面板刚复位、8 个视频还没预加载完，
    //   显示的是花屏——这是开机"闪几遍"的根因之一。背光由 lcdTask 开机进度条从黑渐亮起来。
    pinMode(LCD_BL, OUTPUT);
    digitalWrite(LCD_BL, LOW);

    LOG("[BOOT] === 启动时内存 ===\n");
    LOG("[BOOT] 内部RAM剩余: %u KB\n", ESP.getFreeHeap()  / 1024);
    LOG("[BOOT] PSRAM总量:   %u KB\n", ESP.getPsramSize() / 1024);
    LOG("[BOOT] PSRAM剩余:   %u KB\n", ESP.getFreePsram() / 1024);

    /* Flash 文件系统初始化 */
    healthSetStage(BS_FS);
    LOG("[BOOT] (1/9) 挂载 LittleFS...\n");
    if (!fsInit()) {
        LOG("[BOOT] ✗ LittleFS 挂载失败，系统停止启动！\n");
        while (true) {}
    }
    LOG("[BOOT] ✓ LittleFS 就绪\n");

    /* SN 校验 */
    healthSetStage(BS_SN);
    LOG("[BOOT] (2/9) SN 序列号校验...\n");
    if (!snSetup()) {
        LOG("[BOOT] ✗ SN 校验失败，系统停止启动！\n");
        while (true) {}
    }
    LOG("[BOOT] ✓ SN 校验通过\n");

    /* 电源管理 / 自动 light-sleep */
    healthSetStage(BS_PM);
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

            // ★★ 启动期全程持锁（满频 + 禁 light-sleep）——修"开机转圈背光闪"。
            //   背光是 LovyanGFX 的 Light_PWM，走 LEDC、44.1kHz、时钟源 APB。
            //   light-sleep 一进 APB 就停，PWM 输出随之停摆，唤醒后再恢复 → 肉眼可见的闪。
            //   而 powerManagerLoop 要到 loop() 首轮才跑，【本行到 setup 结束是一段
            //   没有任何人持锁的真空期】，偏偏这段的空闲窗口极密集：bootSpinnerTask 每帧
            //   vTaskDelay(30ms)、预加载每 32KB 让出一次、audInit 还有两个 delay(200)，
            //   几乎每帧都能睡进去，于是整个转圈过程一直在闪。
            //   （同一个坑在 runShutdownSleep 显示低电图标前已经处理过一次：那里是直接
            //     esp_pm_configure 关掉 light-sleep，注释写的就是"否则背光 PWM 停"。）
            //   ★ 衔接：进 loop 后 powerManagerLoop 的 L0 分支会 setSleepLock(false)
            //     把它放掉，稳态功耗与改动前完全一致；此处只覆盖启动这十几秒。
            cpuBoost(true);
            setSleepLock(true);
        }
    }

    /* 配置 + BLE */
    healthSetStage(BS_CFG_BLE);
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
    healthSetStage(BS_LCD_INIT);   // lcdInit 内部会把预加载细分到 BS_LCD_PRELOAD+slot
    LOG("[BOOT] (5/9) 初始化 LCD...\n");
    lcdInit();
    LOG("[BOOT] ✓ LCD 就绪\n");

    /* 音频 */
    healthSetStage(BS_AUD);
    LOG("[BOOT] (6/9) 初始化音频（I2S/MAX98357A 功放）...\n");
    if (audInit() != AUD_OK) {
        LOG("[BOOT] ⚠ 音频初始化异常（继续启动，播放可能不可用）\n");
    } else {
        LOG("[BOOT] ✓ 音频就绪\n");
    }

    /* 麦克风（PDM，I2S0；Button1 单击录音）*/
    healthSetStage(BS_MIC);
    LOG("[BOOT] 初始化麦克风（PDM，I2S0）...\n");
    if (!initMIC()) {
        LOG("[BOOT] ⚠ 麦克风初始化失败（继续启动，录音功能不可用）\n");
    } else {
        LOG("[BOOT] ✓ 麦克风就绪\n");
    }

    /* IMU */
    healthSetStage(BS_IMU);
    LOG("[BOOT] (7/9) 初始化 IMU（QMI8658）...\n");
    imuInit();
    LOG("[BOOT] ✓ IMU 就绪\n");

    /* 舵机 */
    healthSetStage(BS_SERVO);
    LOG("[BOOT] (8/9) 初始化舵机...\n");
    servoInit();
    servoRotate(SERVO_REST_ANGLE);
    LOG("[BOOT] ✓ 舵机就绪（归中位 %d°）\n", SERVO_REST_ANGLE);

    /* 电量 ADC */
    healthSetStage(BS_BAT);
    LOG("[BOOT] (9/9) 初始化电量检测...\n");
    batInit();
    LOG("[BOOT] ✓ 电量检测就绪\n");

    pinMode(PROV_BUTTON_PIN, INPUT_PULLUP);                 // 配网/录音按键
    // 按下中断：L2/light-sleep 下按键立即唤醒主循环（ISR 里 xMainWake 空指针安全）
    attachInterrupt(digitalPinToInterrupt(PROV_BUTTON_PIN), buttonISR, FALLING);

    /* 消息队列 */
    healthSetStage(BS_QUEUE);
    LOG("[BOOT] 创建消息队列...\n");
    qMainToLcd    = xQueueCreate(5, sizeof(MessageToLCD_t));
    qMainToWeb    = xQueueCreate(5, WEB_MSG_LEN);
    qMainToAud    = xQueueCreate(5, sizeof(MessageToAud_t));
    qMainToMic    = xQueueCreate(5, sizeof(MessageToMic_t));
    qWebToMain    = xQueueCreate(5, sizeof(MessageToMain_t));
    qImuToMain    = xQueueCreate(5, sizeof(MessageToMain_t));
    qConfigUpdate = xQueueCreate(3, sizeof(uint8_t));
    qPosStream    = xQueueCreate(1, sizeof(PosSample_t));
    qServoCmd     = xQueueCreate(1, sizeof(MessageToMain_t));   // 深度1：只保留最新舵机角度

    // ── 低功耗唤醒信号量（务必在 imu/web/lcd/aud 任务创建之前）──
    xMainWake = xSemaphoreCreateCounting(20, 0);   // 计数型：多事件不丢
    xLcdWake  = xSemaphoreCreateBinary();
    xAudWake  = xSemaphoreCreateBinary();
    if (!qMainToLcd || !qMainToWeb || !qMainToAud || !qMainToMic ||
        !qWebToMain || !qServoCmd || !qImuToMain || !qConfigUpdate ||
        !qPosStream || !xMainWake || !xLcdWake || !xAudWake) {
        LOG("[BOOT] ✗ 队列/信号量创建失败！\n");
        while (true) {}
    }
    LOG("[BOOT] ✓ 队列 + 唤醒信号量就绪\n");

    /* 任务创建 */
    healthSetStage(BS_TASKS);
    LOG("[BOOT] 创建任务...\n");

    // 交接屏幕所有权：停"开机转圈"并等它退出，之后再建 lcdTask 接管 tft（杜绝双写屏幕）。
    // 放在建任务前：转圈全程覆盖预加载+音频+麦克风+IMU+舵机+队列这段黑屏，到此刻才收尾。
    lcdStopBootSpinner();

    BaseType_t ok = pdPASS;

    /* IMU 任务（内部 SRAM 栈 — Flash 操作期间 cache 禁用，栈不能在 PSRAM）
       优先级 2（高于 LCD/MIC 的 1）：保证 250Hz 采样能及时抢占，拍击冲击不被渲染饿掉。
       采样极快且每轮 delay 让出，不会饿死同核渲染任务。*/
    ok &= xTaskCreatePinnedToCore(imuTask, "IMUTask", STACK_IMU, NULL, 2, &xImuTaskHandle, 0);

    /* WEB 任务 */
    ok &= xTaskCreatePinnedToCore(webTask, "WEBTask", STACK_WEB, NULL, 0, &xWebTaskHandle, 0);

    /* LCD 任务（内部 SRAM 栈 — JPEG 解码 + LittleFS 文件读取需要）*/
    ok &= xTaskCreatePinnedToCore(lcdTask, "LcdTask", STACK_LCD, NULL, 1, &xLcdTaskHandle, 0);

    /* AUD 任务（内部 SRAM 栈 — minimp3 解码 ~4KB + 开销；大缓冲在 PSRAM）
     * ★ 优先级 0→5：音频是硬实时的，喂不上 I2S 的 DMA 就会重播旧缓冲发出嗡嗡电流声。
     *   原来的 0 与 IDLE 同级，还被同在 core 1 的 loopTask / persistTask(都是 1) 抢占，
     *   欠载探针实测 4.6s 里亏空 ~40ms。这里放到全系统最高，安全性来自 audTask 全程
     *   阻塞式等待（播放中阻塞在 i2s_channel_write 的 DMA 信号量，空闲时 vTaskDelay /
     *   信号量），没有任何忙等，不会饿死别人；且它独占 core 1，core 0 的
     *   LCD/IMU/WEB 完全不受影响。 */
    ok &= xTaskCreatePinnedToCore(audTask, "AudTask", STACK_AUD, NULL, 5, &xAudTaskHandle, 1);

    /* MIC 任务（栈含 Shine MP3 编码开销，从 4096 提到 8192）*/
    ok &= xTaskCreatePinnedToCore(micTask, "MicTask", STACK_MIC, NULL, 1, &xMicTaskHandle, 0);

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

    // ★ setup 全部走完 → 标记进入运行期。此后再崩，lastStage 就是 BS_RUNNING，
    //   一眼区分"启动期崩溃"（stage < 99，本次事故的形态）和"运行期崩溃"。
    healthSetStage(BS_RUNNING);

    // ★ OTA 自检：本次若是新固件首次启动(VERIFYING)，跑到运行期 = 启动成功 →
    //   esp_ota_mark_app_valid_cancel_rollback()，确认新固件、取消 bootloader 回滚。
    otaMarkValidOnBoot();

    g_lastActivityMs = millis();   // 开机视为活动，避免立即进省电
}

// 舵机指令处理（由 qServoCmd 深度1覆盖队列驱动；连发只应用最新角度）。
// 支持 SERVO<angle> 或 SERVO<angle>-<angularVelocity>；偏离中位则同步播"摇一摇"动画。
static void processServoCmd(const char* strVal)
{
    int angle, angularVelocity;
    int parsedCount = sscanf(strVal, "SERVO%d-%d", &angle, &angularVelocity);

    if (parsedCount == 2) {
        LOG("[SERVO] 目标角度=%d度，角速度=%d度/秒\n", angle, angularVelocity);
        angle           = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
        angularVelocity = constrain(angularVelocity, 0, SERVO_MAX_SPEED);
        servoRotateAV(angle, angularVelocity);
    } else {
        parsedCount = sscanf(strVal, "SERVO%d", &angle);
        if (parsedCount == 1) {
            LOG("[SERVO] 目标角度=%d度\n", angle);
            angle = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
            servoRotate(angle);
        }
    }
    // ★ 首拍保留：本次若把舵机从断电态唤醒（servoJustPoweredOn）且是摆动指令，
    //   开一个保留窗，让这一拍走完再接受下一条 —— 否则紧随其后的回中位会把它
    //   顶掉，用户看不到动作。回中位本身不开窗（它就是要回去的）。
    if (parsedCount >= 1 && angle != SERVO_REST_ANGLE && servoJustPoweredOn) {
        g_servoSwingHoldUntil = millis() + SERVO_FIRST_SWING_HOLD_MS;
        LOG("[SERVO] 舵机刚上电 → 首拍保留 %dms\n", SERVO_FIRST_SWING_HOLD_MS);
    }

    // ★ 物理摇摆(舵机偏离中位)时同步播"摇一摇"动画。App 摇摆控件发的就是 SERVO52/122。
    //   回中位 87 = 静止，不触发。此路径也覆盖 P3 伙伴镜像摇摆。
    //   NONINT + !lcdIsPlaying() 守卫：正在播摇摆时忽略后续摇摆、既不打断也不重播。
    if (parsedCount >= 1 && angle != SERVO_REST_ANGLE && !lcdIsPlaying()) {
        MessageToLCD_t sm = {};
        sm.cmd = LCDMSG_PLAY_NONINT; sm.motion = MOTION_SHAKE;
        sm.playTimes = 1; sm.interruptAble = false;
        xQueueSend(qMainToLcd, &sm, 0);
    }
    markActivity();              // 舵机动作 = 活动
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
                // 按住 2s → 触发配网（给用户即时反馈）。若继续按到 8s 进关机，
                // enterShutdown 走硬复位，BLE/配网随复位一并干净清掉（不在原地 deinit，避免崩溃）。
                if (!provFired && hold >= PROV_LONG_PRESS_MS) {
                    provFired = true;
                    LOG("[BTN] Button1 长按 %d ms，触发配网\n", PROV_LONG_PRESS_MS);
                    g_enterProvisioning = true;     // 通知 webTask
                }
                if (hold >= SHUTDOWN_LONG_PRESS_MS) {
                    enterShutdown();                // 长按 8 秒 → 关机（不返回）
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
 
    // ── 舵机指令：独立深度1覆盖队列(qServoCmd)。连发 SERVO 只应用最新角度，
    //   不占 qWebToMain(不会挤掉/排在播放指令后)，深度1覆盖也不会因满而丢。──
    {
        // ★ 首拍保留窗内不取新指令：让刚上电的舵机把这一拍走完。
        //   窗内到达的指令仍在 qServoCmd 里覆盖累积（深度1，留最新一条），
        //   窗口结束后照常取走——与原语义一致，只是晚一小段。
        //   用有符号差值比较，millis() 溢出时也不会误判。
        bool holding = (g_servoSwingHoldUntil != 0) &&
                       ((int32_t)(millis() - g_servoSwingHoldUntil) < 0);
        if (!holding) {
            g_servoSwingHoldUntil = 0;        // 窗口到期（或本就没开），恢复常规节奏
            MessageToMain_t servoCmd;
            if (xQueueReceive(qServoCmd, &servoCmd, 0) == pdTRUE)
                processServoCmd(servoCmd.strVal);
        }
    }


    // ── Web 指令处理 ─────────────────────────────────────
    //  ★ 每轮排空至多 QWEB_DRAIN_PER_LOOP 条（原来每轮只取 1 条）。
    //    qWebToMain 深度只有 5，而所有生产者都用超时 0（满了静默丢弃）。主循环一旦被
    //    某条消息的阻塞处理拖住（WBL 分支里 servoRotateAV 是逐度 delay 的，可达数秒），
    //    队列就会积压 → 后续消息被丢。丢到 XFER_UNLOCK 就是永久锁死：xferLocked 恒为真
    //    → forceActive 永不休眠(电流从 2.2mA 涨到几十 mA) + VIDEO_SHOW/RECORD/SERVO
    //    全被"传输中，屏蔽"，且只能重启恢复。
    //    上限而不是全排空：单条消息可能阻塞很久，一次处理太多会让本轮 loop 过长，
    //    影响按键/电量/功耗状态机的响应。
    //  ★ 同时设时间预算：本循环里有【阻塞式】分支（WBL 的 servoRotateAV 是逐度 delay，
    //    可达数秒）。只按条数排空的话，连着 4 条 WBL 会把主循环一口气堵住十几秒，
    //    期间不检查按键（含 8 秒长按关机！）、不采电量、不跑功耗状态机。
    //    时间检查放在【每次取消息之前】：处理完一条耗时的就立刻收手、让本轮 loop 走完，
    //    剩下的下一轮再排 —— 既排得快，又不牺牲响应性。
    uint32_t drainT0 = millis();
    for (int drained = 0;
         drained < QWEB_DRAIN_PER_LOOP &&
         (millis() - drainT0) < QWEB_DRAIN_BUDGET_MS &&
         xQueueReceive(qWebToMain, &msgFromWeb, 0) == pdTRUE;
         drained++) {
        LOG("[Main] 收到Web指令: %s\n", msgFromWeb.strVal);

        // ── 传输锁定 ──
        if (strcmp(msgFromWeb.strVal, "XFER_LOCK") == 0) {
            // 锁状态本身由 g_xfering 直接反映，这里只负责【副作用】：停音频 + LCD 切待机。
            // 所以这条消息即便丢了也不会让设备锁死，最多是没及时停播。
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
            // 解锁已随 g_xfering 自动生效（就算这条消息丢了也不会锁死），此处仅留日志标记
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
        // ── POKE_TAP：伙伴拍一拍(P2) → 固定播放 TAP 点头动画（不播 target 自定义视频/音频）──
        //   与本机拍一拍(IMU intVal==1)一致用 NONINT：紧随其后的 wink/其它动画会排队，
        //   点头不会被拦腰打断。不设 g_hasUnreadMsg（拍一拍是实时互动，无待回复内容），
        //   也不转发 POKE_PEER（避免与发起方来回互戳成环）。
        else if (strcmp(msgFromWeb.strVal, "POKE_TAP") == 0) {
            MessageToLCD_t m = {}; m.cmd = LCDMSG_PLAY_NONINT; m.motion = MOTION_TAP;
            m.playTimes = 1; m.interruptAble = false;
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

            // ★ 信封待回复态：拍一拍 / 摇一摇 行为一致 —— 都视为"查看并回复这条消息"：
            //   重播上一条(target 视频+音频) + 发 TAP 回执 + 退出信封态。
            if (g_envelopeMode &&
                (msgFromImu.intVal == 1 || msgFromImu.intVal == 2)) {
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
                LOG("[MSG] 信封态%s → 重播上一条 + 回执\n",
                    msgFromImu.intVal == 1 ? "拍一拍" : "摇一摇");
            }
            else if (msgFromImu.intVal == 1) {
                // 拍一拍（非信封态）→ LCD 播放 TAP 点头动画
                // ★ 不可打断（NONINT）：紧接着就会向伙伴发 POKE，伙伴的 POKE_ACK 往往在
                //   几百毫秒内回来并触发 WINK。若点头是可打断的，wink 会把它拦腰截断，
                //   看起来就是"点头点到一半突然眨眼"。NONINT 让 lcdTask 把后到的消息
                //   压在 msgPending 里、等点头播完(playTimes=1)再处理，于是顺序恒为
                //   点头 → 眨眼。代价：点头这一秒内到达的其它动画（好友视频/摇一摇）
                //   也会顺延到点头之后播，不会丢，只是晚一拍。
                msgToLcd.cmd           = LCDMSG_PLAY_NONINT;
                msgToLcd.motion        = MOTION_TAP;
                msgToLcd.playTimes     = 1;
                msgToLcd.interruptAble = false;
                xQueueSend(qMainToLcd, &msgToLcd, 0);

                // ★ 转发 TAP 给 Web，由 Web 向 term/<terminal>/cmd 上报
                strcpy(msgToWeb, "TAP");
                xQueueSend(qMainToWeb, &msgToWeb, 0);

                // ★ P2：拍一拍同时"戳"已互绑的伙伴，让对方设备播放拍一拍视频
                strcpy(msgToWeb, "POKE_PEER");
                xQueueSend(qMainToWeb, &msgToWeb, 0);

                g_hasUnreadMsg = false;   // ★ 拍一拍=回复，清未读

            } else if (msgFromImu.intVal == 2) {
                // 摇一摇（非信封态）→ LCD 播放 SHAKE 动画
                // ★ NONINT：摇摆一旦起播必整段播完；播放期间到达的其它摇摆由 lcdTask 侧忽略、不打断。
                msgToLcd.cmd           = LCDMSG_PLAY_NONINT;
                msgToLcd.motion        = MOTION_SHAKE;
                msgToLcd.playTimes     = 1;
                msgToLcd.interruptAble = false;
                xQueueSend(qMainToLcd, &msgToLcd, 0);
                // ★ P3：摇一摇同时让已绑定伙伴的舵机镜像本机姿态（web 广播姿态一小段）
                strcpy(msgToWeb, "SHAKE_PEER");
                xQueueSend(qMainToWeb, &msgToWeb, 0);
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

        // ★ 先并入黑匣子（RTC 内存，跨崩溃保留），再打日志。
        //   串口在 light-sleep 后就断了，真正能带出现场的是这一行不是下面的 LOG。
        healthBoxSample();

        LOG("\n==================== 系统资源监控 ====================\n");
        LOG("运行时间: %lu 秒 | 传输锁: %s\n",
            currentTime / 1000, xferLocked ? "锁定" : "正常");
        LOG("------------------------------------------------------\n");
        LOG("堆内存: 剩余=%lu 字节 | 历史最小=%lu 字节\n",
            ESP.getFreeHeap(), ESP.getMinFreeHeap());

        // ★ 碎片化指标：只看"剩余总量"看不出碎片化 —— 剩余可能还有 100KB，但最大连续块
        //   已经掉到 8KB，此时任何一次 16KB+ 的分配（I2S DMA / MQTT 缓冲 / JPEG 解码 /
        //   PSRAM 快照）都会失败 → 表现为"长时间运行后功能异常"，而堆监控一切正常。
        //   碎片率 = 1 - 最大块/剩余总量。持续上升就是碎片化在恶化。
        {
            size_t iFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            size_t iBig  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
            size_t pFree = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            size_t pBig  = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
            LOG("碎片化: 内部RAM 剩余=%uKB 最大块=%uKB 碎片率=%u%% | "
                "PSRAM 剩余=%uKB 最大块=%uKB 碎片率=%u%%\n",
                (unsigned)(iFree / 1024), (unsigned)(iBig / 1024),
                (unsigned)(iFree ? 100 - (uint32_t)((uint64_t)iBig * 100 / iFree) : 0),
                (unsigned)(pFree / 1024), (unsigned)(pBig / 1024),
                (unsigned)(pFree ? 100 - (uint32_t)((uint64_t)pBig * 100 / pFree) : 0));
            // 最大块跌破这个线就该警觉：I2S DMA 环(16KB)、MQTT 缓冲(8KB)都在这个量级
            if (iBig < 20 * 1024)
                LOG("⚠⚠ 内部RAM 最大连续块仅 %uKB —— 大块分配即将开始失败（碎片化）\n",
                    (unsigned)(iBig / 1024));
        }
        // ★ 原来这里另算了一份"主任务栈"，按 TASK_STACK_SIZE(16384) 折算 → 73.6%，
        //   而下面 printStackHWM("loopTask", 8192) 同一时刻算出 47.3%：同一个任务两个数字，
        //   且 16384 是错的（Arduino loopTask 默认 8192）。删掉重复实现，统一走 printStackHWM。
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
        printStackHWM("loopTask", xTaskGetCurrentTaskHandle(), 8192);   // Arduino loopTask，非本文件创建
        printStackHWM("IMUTask",  xImuTaskHandle, STACK_IMU);
        printStackHWM("WEBTask",  xWebTaskHandle, STACK_WEB);
        printStackHWM("LcdTask",  xLcdTaskHandle, STACK_LCD);
        printStackHWM("AudTask",  xAudTaskHandle, STACK_AUD);
        printStackHWM("MicTask",  xMicTaskHandle, STACK_MIC);   // 未创建会自动显示"未创建"
        LOG("======================================================\n\n");
    }
#endif

    batCheckChargeFast();   // 充电插拔快速检测（每轮，秒级反应；电量数值仍每 5s 平滑更新）
    batUpdate();

    servoLoop();          // 舵机：90°静止1s则 detach
    powerManagerLoop();   // 功耗分级 + CPU 频率仲裁

    // ── 心跳调度：L2 阻塞等事件（喂 light-sleep），事件 give 信号量瞬时唤醒；
    //    L0/L1 维持 100ms（按键长按检测/巡检）。──
    uint32_t napMs = (g_pwrTier == 2) ? PWR_L2_BLOCK_MS : 100;
    // ★ 首拍保留窗内：只睡到窗口到期，别多睡。否则窗口结束后还要再等一整轮
    //   轮询(100ms)才去取那条积压的回中位，回中位会明显拖慢。
    if (g_servoSwingHoldUntil) {
        int32_t left = (int32_t)(g_servoSwingHoldUntil - millis());
        if (left > 0 && (uint32_t)left < napMs) napMs = (uint32_t)left;
    }
    xSemaphoreTake(xMainWake, pdMS_TO_TICKS(napMs));
}