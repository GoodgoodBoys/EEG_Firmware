#include "inc/imu.hpp"
#include "inc/msg.hpp"
#include <QMI8658.h>
#include <Wire.h>
#include <math.h>
#include "esp_pm.h"        // 仅在 I2C 事务期间短暂禁 light-sleep，防事务撞 sleep 报 INVALID_STATE
#include "esp_task_wdt.h"  // 任务级看门狗：卡死时 panic backtrace 直接点名本任务
#include "esp_sleep.h"     // light-sleep GPIO 唤醒源（WoM 运动中断唤醒深睡的 L2）
#include "driver/gpio.h"
#include "soc/gpio_struct.h"   // ISR 里直接写 GPIO.pin[n].int_ena（IRAM 安全，见 imuWoMISR）
#include "inc/debug.hpp"   // ★ 放在库头之后，使 LOG / LOG_FATAL / _DEBUG 生效
#include "inc/health.hpp"  // 运行期阶段打点（TWDT 超时时会被冻成快照）

static QMI8658 imu;

extern QueueHandle_t qImuToMain; // message queue |  Imu -> main
extern QueueHandle_t qPosStream;
extern volatile bool g_posStreaming;
extern volatile int  g_pwrTier;   // 0/1/2 功耗档位（V1_1.cpp 定义）
extern volatile bool g_xfering;   // web.cpp：视频/音频下载传输中（传输期识别被屏蔽，降采样减 I2C 卡死概率）
extern volatile bool g_persistAbortWrite;  // web.cpp：唤醒立即打断落盘写入（IMU 运动最早在本 WoM ISR 置位）
extern void mainWake();           // 唤醒主循环（V1_1.cpp）

// ═══════════════════════════════════════════════════════════════
//  纯 IMU 动作识别（无震动开关）：拍=加速度冲击 / 摇=陀螺
//    醒着(L0/L1/串流/唤醒窗口)：加速度 250Hz 全采做识别。
//                              L1 只是屏幕降到 60%、CPU 仍满频，IMU 照常工作，
//                              保证这一档的拍/摇零延迟、不丢第一下。
//    睡眠(L2)                ：IMU 进 Wake-on-Motion 低功耗，运动经 IMU_INT1(GPIO21)
//                              硬件中断唤醒本任务；醒来退 WoM，恢复全采做识别。
//
//  ★ 锁时机（关键）：本平台 light-sleep 期间做 I2C 事务会报 INVALID_STATE，且 WoM
//    退出若恢复 ODR 的写失败，accel 会卡在 21Hz(采样~39Hz)。所以只在【配置 WoM /
//    唤醒恢复 / 采样】这几段 I2C 期间短暂持 NO_LIGHT_SLEEP 锁保事务成功；
//    【待机阻塞等 WoM 中断】时放锁，正常 light-sleep 省电（那时不碰 I2C）。
//
//  拍一拍判据来自 40拍/30摇/20碰 真机标定（见 Test/motionCapture 分析）。
// ═══════════════════════════════════════════════════════════════

#define POS_RATE_DIV       12     // 250Hz ÷ 12 ≈ 20Hz（姿态串流上报率）
#define POS_ACC_DEADZONE   15.0f  // 三轴变化都 <15mg 视为没动，不发

// ── 拍一拍判据（标定值，可调）──
#define TAP_LINV_LO     700.0f    // 垂直冲击下界(mg)：滤噪声
#define TAP_LINV_HI     3000.0f   // 垂直冲击上界(mg)：砍硬磕
// 垂直/水平比。★ 1.2 → 0.8：40 条真机拍样本里这一项的 p10 恰好就是 1.2，等于设计时就
//   把最边缘的 10% 切掉了；被它拒的样本全是 vh≈1.0 的【拍侧面】（linV 与 horiz 相当），
//   而拍侧面是这个蛋最自然的动作。离线消融证明它对区分"摇"毫无增量贡献——摇的判别力
//   全在陀螺(gMax 差 10 倍)和时长(active 差 4 倍)上。放宽后 tap 召回 90%→100%，
//   shake/bump 误判一个不增。保留一道 0.8 的弱门限只为挡纯水平滑动。
#define TAP_VH_MIN      0.8f
#define TAP_JERK_MAX    1.0e6f    // jerk 上界(mg/s)：软接触，排除硬磕（按真实 dt 归一化）
#define TAP_ACTIVE_MAX  100       // 活跃时长上限(ms)：短冲击，排除持续运动
#define ACTIVE_DEV_TH   300.0f    // 算 active_ms 用的 |a|-1000 门限
#define TAP_GMAX_MAX    400.0f    // 陀螺峰上限(dps)：加强排除摇（仅陀螺 settle 后生效）
#define TAP_REV_MAX     2         // 陀螺反转上限：加强排除摇
#define GYRO_REV_TH     250.0f    // 陀螺方向反转判定(dps)
#define TRIG_TH_MG      500.0f    // 触发门限 |aMag-1000|：拍/摇/碰都进来分类
#define TAP_COOLDOWN_MS 500
#define IMU_POLL_MS     2         // 采样轮询间隔(ms)，须 < 4ms(250Hz样本周期) 才不丢样本
#define IMU_XFER_POLL_MS 100      // ★ 下载传输期轮询间隔(ms)：识别本被屏蔽，降到 10Hz 采样，
                                  //   I2C 访问减少 ~50 倍 → 大幅降低总线 hang 卡死概率

// ── 环形缓冲（触发前基线 + 触发后冲击）──
#define PRE_SAMPLES   50          // 触发前 ≈200ms：算重力基线
// 触发后窗口。★ 100(400ms) → 60(240ms)：真机 40 条拍样本的 active 时长 p90 才 59ms，
//   400ms 里绝大部分是白等，而这段时间【不接受新触发】。缩短直接换来响应性。
//   离线用截断到 60 的窗口复算过：tap 40/40、shake 0/30、bump 1/20，抗摇能力不变。
#define POST_SAMPLES  60          // 触发后 ≈240ms：含冲击 + 冲击后静止
#define RING_SIZE  (PRE_SAMPLES + POST_SAMPLES + 8)

struct Sample { uint32_t t; int16_t ax, ay, az, gx, gy, gz; };   // t = micros()，特征按真实 dt 算
static Sample   s_ring[RING_SIZE];
static int      s_head = 0, s_filled = 0;
static bool     s_capturing = false;
static int      s_postLeft = 0, s_trigHead = 0;
static uint32_t s_tapCooldownUntil = 0;

static TaskHandle_t s_imuTaskHandle = nullptr;
#define IMU_AWAKE_WINDOW_MS  1500   // 被 WoM 唤醒后保持全采样的窗口时长（这段持锁禁 sleep）

static uint32_t s_gyroReadyAt = 0;      // 陀螺起振 settle 截止时刻

// ── I2C 事务期：禁 light-sleep + 锁最高频率 ──
//  ★ 为什么还要锁频率（原来只有禁睡锁，是本次排查揪出来的坑）：
//    实测 I2C 在 L0/L1 连续跑 60 秒毫无问题，一进 L2 就【每一条事务】都返回
//    ESP_ERR_INVALID_STATE(259)。而 L0/L1 与 L2 的唯一差别就是 powerManagerLoop 里
//    cpuBoost(tier<=1)：L2 放掉 CPU_FREQ_MAX 锁 → DFS 把 CPU/APB 从 240MHz 降到 40MHz。
//    IDF 的 I2C 驱动只在时钟源是 APB 时才自己取 APB_FREQ_MAX 锁（i2c_common.c:296），
//    而 S3 的默认源不是 APB，所以它只取了 NO_LIGHT_SLEEP —— 没有任何东西挡住降频，
//    而 I2C 的 FIFO/寄存器接口挂在 APB 上（驱动注释原话）。
//    代价几乎为零：这把锁只在采样/配置期间持有，而那本来就是 tier<=1、已经满频的时候；
//    L2 下也只在 enterWoM/exitWoM 的那几毫秒内多持一下。
static esp_pm_lock_handle_t s_imuNoSleep = nullptr;
static esp_pm_lock_handle_t s_imuMaxFreq = nullptr;
static void imuI2cLock(bool on) {
    static bool held = false;
    if (on == held) return;
    // ★ 两把锁必须【同时存在】才记账为已持有：若只有一把创建成功（另一把 OOM），
    //   acquire 时跳过 nullptr、release 时同样跳过，本身不会失衡；但 held 记成 true 会
    //   掩盖"频率没锁住"的事实。这里不额外容错，保持与创建失败时的告警一致即可。
    if (on) {
        if (s_imuMaxFreq) esp_pm_lock_acquire(s_imuMaxFreq);   // 先锁频率
        if (s_imuNoSleep) esp_pm_lock_acquire(s_imuNoSleep);
    } else {
        if (s_imuNoSleep) esp_pm_lock_release(s_imuNoSleep);   // 逆序释放
        if (s_imuMaxFreq) esp_pm_lock_release(s_imuMaxFreq);
    }
    held = on;
}

// ═══════════════════════════════════════════════════════════════
//  Wake-on-Motion（裸 I2C 直接配 QMI8658 寄存器；均在持锁状态下调用）
//    ⚠ 库自带 enableWakeOnMotion() 残缺（没发 CTRL9 命令 / 没配 INT 路由），故自己写。
//      以下值可能需按实测微调（见文末排查）：WOM_INT_SEL / WOM_THRESH_MG / INT 极性。
// ═══════════════════════════════════════════════════════════════
// ★★★ I2C 从机地址必须【运行时探测】，不能硬编码。
//   QMI8658 的地址由 SA0/AD0 引脚决定：0x6A(低) 或 0x6B(高)。库的 begin() 会自己探测——
//   先试 0x6A，若 WHO_AM_I != 0x05 就切到 0x6B，之后一直用探测到的 _address（私有，无 getter）。
//   而本文件的裸 I2C 曾把地址写死成 0x6A，本板实际是另一个地址，于是【每一条 womWrite/womRead
//   都 NACK】→ 事务不完成 → ESP_ERR_INVALID_STATE(259)。
//   后果极隐蔽：库走 _address 一切正常（采样 250Hz 稳定），只有 WoM 配置和 CTRL1(INT1_EN)
//   这些走裸 I2C 的写全部静默失败 —— 排查时一度误判成极性、INT 路由、DFS 降频。
//   ★ 库的 writeRegister/readRegister 与本文件的 womWrite/womRead 实现逐字节相同，
//     唯一差别就是地址，这是定位到此的决定性证据。
#define QMI_ADDR_LOW    0x6A
#define QMI_ADDR_HIGH   0x6B
#define QMI_REG_WHOAMI  0x00
#define QMI_WHOAMI_VAL  0x05
static uint8_t QMI_ADDR = QMI_ADDR_LOW;   // 由 imuDetectAddr() 在 init 时确定
#define REG_CTRL1       0x02      // ★ 串口/中断使能：bit3=INT1_EN, bit4=INT2_EN
#define REG_CTRL2       0x03
#define REG_CTRL7       0x08
#define REG_CTRL9       0x0A
#define REG_CAL1_L      0x0B
#define REG_CAL1_H      0x0C
#define REG_STATUSINT   0x2D
#define REG_STATUS1     0x2F      // bit2 = WoM 事件标志（读它才会清标志并复位 INT 线，见手册 §12.4）
#define CMD_WRITE_WOM   0x08
#define CMD_ACK         0x00
#define ACC_ODR_LP_21HZ 0x0D      // CTRL2：±2g + 低功耗 21Hz

#define WOM_THRESH_MG   100       // WoM 唤醒阈值(mg)：偏低确保拍/碰都能唤醒（CAL1_L，1mg/LSB）
// ★ CAL1_H[7:6] 是【引脚 + 空闲电平】的组合，不是单纯的引脚选择（数据手册 Table 39）：
//     00 = INT1，空闲 0（触发拉高）    10 = INT1，空闲 1（触发拉低）
//     01 = INT2，空闲 0                11 = INT2，空闲 1
//   本板 GPIO21 接的是 INT1，ESP32 侧用 INPUT_PULLDOWN + GPIO_INTR_HIGH_LEVEL 唤醒，
//   对应"空闲低、触发拉高" → 取 0b00。这也与手册 CTRL9 握手描述的
//   "raise INT1 ... INT1 is pulled low upon ACK"（高有效）一致。
//   （最早的值 1 = 0b01 会把中断路由到 INT2，而 INT2 根本没接线，怎么调都不可能醒；
//     之后试过 0b10 = INT1 空闲高/触发低，实测摇动也无反应，故翻到 0b00。）
// CAL1_H[7:6]（手册表 39「WoM Interrupt Initial Value select」）：
//   bit6 = 选哪根 INT 脚（0=INT1, 1=INT2）；bit7 = 空闲(初始)电平。
//     00 = INT1 空闲低 → 触发拉高   ← 现用（手册 CTRL9 握手描述的 "raise INT1" 也是高有效）
//     10 = INT1 空闲高 → 触发拉低   ← 曾用，实测摇动无反应
//   ★ 改这里必须同步改 ESP32 侧两处，三者不一致就永远不触发：
//     ① pinMode(IMU_INT1, ...)          空闲低 → INPUT_PULLDOWN
//     ② gpio_wakeup_enable(..., ...)    空闲低 → GPIO_INTR_HIGH_LEVEL
#define WOM_INT_SEL     0x00      // 0b00：INT1 + 空闲低电平（触发时拉高）
#define WOM_BLANK       0x02      // CAL1_H[5:0] 消隐样本数
#define WOM_DISABLE     0x00      // CAL1_L 写 0 = 关闭 WoM，INT 引脚恢复常规功能
// （无定时兜底：待机时死等 WoM 中断或 imuWake()，见 imuTask 里的 ulTaskNotifyTake）

static volatile bool s_womActive = false;

// ★ 返回是否成功。原来这里丢掉了 endTransmission() 的返回值，于是 WoM 的每一条寄存器
//   写全部失败（259）也毫无察觉，照样打印"→ WoM 待机"，害得极性/INT路由/INT1_EN 排查了
//   好几轮 —— 实际上一个字节都没写进芯片。
static bool womWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(QMI_ADDR);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}
static uint8_t womRead(uint8_t reg) {
    Wire.beginTransmission(QMI_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)QMI_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}
// 用指定地址读 WHO_AM_I，命中 0x05 即该地址有效（复刻库 begin() 的探测逻辑）
static bool imuProbeAddr(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(QMI_REG_WHOAMI);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
    return Wire.available() && Wire.read() == QMI_WHOAMI_VAL;
}

// 探测本板 QMI8658 的真实地址，供裸 I2C（WoM / CTRL1）使用。
// 必须在 Wire 起来之后、任何 womWrite/womRead 之前调用。
static void imuDetectAddr() {
    if (imuProbeAddr(QMI_ADDR_LOW))  { QMI_ADDR = QMI_ADDR_LOW;  LOG("[IMU] I2C 地址探测: 0x6A\n"); return; }
    if (imuProbeAddr(QMI_ADDR_HIGH)) { QMI_ADDR = QMI_ADDR_HIGH; LOG("[IMU] I2C 地址探测: 0x6B\n"); return; }
    LOG("[IMU] ⚠ 0x6A/0x6B 都读不到 WHO_AM_I=0x05，裸 I2C（WoM/INT1_EN）将失败\n");
}

static void ctrl9Cmd(uint8_t cmd) {   // CTRL9 命令握手：发命令→等 CmdDone→ACK→等清除
    womWrite(REG_CTRL9, cmd);
    uint32_t t0 = millis();
    while (!(womRead(REG_STATUSINT) & 0x80) && millis() - t0 < 100) delay(1);
    womWrite(REG_CTRL9, CMD_ACK);
    t0 = millis();
    while ((womRead(REG_STATUSINT) & 0x80) && millis() - t0 < 100) delay(1);
}

static void imuBusRecover();   // 前置声明（配置失败时调用）

// 返回 true = WoM 配置成功写进芯片。★ 返回 false 时【调用方绝不能进待机】：
//   芯片没被配置成 WoM，就永远不会产生唤醒中断，睡下去只能靠按键救。
static bool imuEnterWoM() {   // ★ 调用前须持锁（保 I2C 事务成功、WoM 真正配上）
    if (s_womActive) return true;

    for (int attempt = 0; attempt < 2; attempt++) {
        // 第一条写当探针：总线不通就先恢复再重试一轮
        if (!womWrite(REG_CTRL7, 0x00)) {
            if (attempt == 0) {
                LOG("[IMU] ✗ 配 WoM 时 I2C 写失败，恢复总线后重试\n");
                imuBusRecover();
                continue;
            }
            LOG("[IMU] ✗ 恢复后仍失败，放弃进 WoM（本轮继续采样，不待机）\n");
            return false;
        }
        delay(2);
        bool ok = true;
        ok &= womWrite(REG_CTRL2, ACC_ODR_LP_21HZ);             // ±2g + 21Hz 低功耗
        ok &= womWrite(REG_CAL1_L, WOM_THRESH_MG);
        ok &= womWrite(REG_CAL1_H, (uint8_t)((WOM_INT_SEL << 6) | WOM_BLANK));
        ctrl9Cmd(CMD_WRITE_WOM);
        ok &= womWrite(REG_CTRL7, 0x01);                        // 只开加速度
        if (!ok) {
            if (attempt == 0) { LOG("[IMU] ✗ WoM 寄存器写入不完整，恢复总线后重试\n"); imuBusRecover(); continue; }
            LOG("[IMU] ✗ 重试后仍写不全，放弃进 WoM\n");
            return false;
        }

        // ★ 回读【校验】：不能只打印。I2C 写返回成功只代表从机 ACK 了地址与数据，
        //   不保证寄存器真的生效（如芯片处在拒绝配置的状态）。若 CTRL7 没变成 0x01
        //   或阈值没写进去，WoM 引擎就没跑起来 —— 此时进待机会永远等不到中断，
        //   只能靠按键救。故校验不过就当失败处理，退回继续采样。
        uint8_t rbCtrl7 = womRead(REG_CTRL7);
        uint8_t rbThr   = womRead(REG_CAL1_L);
        if (rbCtrl7 != 0x01 || rbThr != WOM_THRESH_MG) {
            LOG("[IMU] ✗ WoM 回读校验失败（CTRL7=0x%02X 期望0x01，CAL1_L=0x%02X 期望0x%02X）\n",
                rbCtrl7, rbThr, WOM_THRESH_MG);
            if (attempt == 0) { imuBusRecover(); continue; }
            return false;
        }
        s_womActive = true;
        LOG("[IMU] → WoM 待机（阈值%dmg，等 GPIO%d 中断 | 回读 CTRL7=0x%02X CAL1_L=0x%02X）\n",
            WOM_THRESH_MG, IMU_INT1, rbCtrl7, rbThr);
        return true;
    }
    return false;
}

static void imuExitWoM() {    // ★ 调用前须持锁（保恢复 250Hz 的写成功，不卡 21Hz→39Hz）
    if (!s_womActive) return;

    // ★★ STATUS1 必须【第一时间】读：手册 §12.4 读它才清 WoM 标志并复位 INT 线，
    //   而 §12.5 说重配 WoM 阈值也会清掉该标志 —— 若放到下面的关闭序列之后再读，
    //   标志早被我们自己抹掉，永远读出 0x00，看上去像"不是 WoM 唤醒"（实测踩过）。
    //   bit2=1 → 确实是运动事件唤醒；bit2=0 → 系统 imuWake() 或杂散 GPIO 中断。
    uint8_t st1 = womRead(REG_STATUS1);
    LOG("[IMU] 唤醒来源: STATUS1=0x%02X（bit2=WoM → %s）\n",
        st1, (st1 & 0x04) ? "运动事件" : "非WoM(系统唤醒/杂散)");

    // ★★ 必须按手册 §12.6 的三步正式退出，只写 CTRL7=0 是不够的：
    //     ① 清 CTRL7[1:0] 关传感器 ② CAL1_L 写 0（阈值 0 = 关闭 WoM）③ 重发 CTRL9 WoM 命令
    //   漏掉②③的后果：WoM 引擎在正常 250Hz 采样期间继续运行，而 §12.4 规定
    //   "For each WoM event, the state of the selected interrupt line is toggled" ——
    //   INT1 被不停翻转，下次 gpio_intr_enable() 一武装就立刻自触发，
    //   表现为 进WoM→退WoM→进WoM 的无限循环（本次实测现象）。
    womWrite(REG_CTRL7, 0x00); delay(2);   // ① 关传感器
    womWrite(REG_CAL1_L, WOM_DISABLE);     // ② 阈值 0 = 关闭 WoM，INT 脚恢复常规功能
    womWrite(REG_CAL1_H, 0x00);
    ctrl9Cmd(CMD_WRITE_WOM);               // ③ 重发 CTRL9 命令让①②生效
    imu.setAccelRange(QMI8658_ACCEL_RANGE_8G);
    imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
    imu.setAccelODR(QMI8658_ACCEL_ODR_250HZ);
    imu.setGyroODR(QMI8658_GYRO_ODR_250HZ);
    imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
    s_gyroReadyAt = millis() + 170;
    s_womActive = false;
    LOG("[IMU] ← 退出 WoM，恢复 250Hz 全采（陀螺 settle 中）\n");
}

// 系统侧唤醒：功耗状态机离开 L2 时（按键 / App 指令 / 任何 markActivity）调用，
// 把可能正阻塞在 WoM 待机里的 imuTask 立刻叫醒去恢复全采。
// ★ 必须有：imuTask 在 WoM 待机时【只等中断通知】，自己不轮询 g_pwrTier。
//   而待机是【死等通知】没有超时兜底，中断不通时这就是唯一的救命通路 ——
//   期间按键把设备唤到 L0，IMU 却还在 21Hz WoM 模式下装死，拍/摇全部失灵。
void imuWake() {
    if (s_imuTaskHandle) xTaskNotifyGive(s_imuTaskHandle);
}

// WoM 运动中断：把阻塞的 IMU 任务唤醒去采样。
// ⚠ 触发方式实际是【高电平】而非双沿：imuInit 里的 gpio_wakeup_enable(HIGH_LEVEL) 会覆盖
//   attachInterrupt 设的类型（IDF 的 gpio_wakeup_enable 内部调 gpio_hal_set_intr_type
//   改写同一个寄存器字段，见 esp_driver_gpio/src/gpio.c:670）。这是设计如此——light-sleep
//   的 GPIO 唤醒源只支持电平触发。
void IRAM_ATTR imuWoMISR() {
    // ★ 电平触发必须在 ISR 里立刻关掉本脚中断：INT1 保持高电平期间 ISR 会被无限重入，
    //   把 core0 上的所有任务饿死 → Interrupt WDT panic（关机按键那段栽过同样的坑，
    //   见 runShutdownSleep 的注释）。醒来的任务退 WoM 后会重新武装。
    //   直接写寄存器而不调 gpio_intr_disable()：后者不在 IRAM
    //   （sdkconfig 里 CONFIG_GPIO_CTRL_FUNC_IN_IRAM 未开），flash 操作期间调用会崩。
    //   这一行等价于 gpio_ll_intr_disable()，是 static inline 的同款实现。
    GPIO.pin[IMU_INT1].int_ena = 0;
    // ★ IMU 运动的最早打断点：置位后 persistTask（core1）在下一个 flash 操作前就停手，
    //   把 core0 立刻还给醒来的 imuTask 做高频采样，避免落盘冻结吃掉动作唤醒的头几拍。
    //   g_persistAbortWrite 是普通 DRAM volatile，IRAM ISR 里写它安全（不碰 flash）。
    g_persistAbortWrite = true;
    if (s_imuTaskHandle) {
        BaseType_t hpw = pdFALSE;
        vTaskNotifyGiveFromISR(s_imuTaskHandle, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

// ───────────────────────────────────────────────────────────────
//  拍一拍：加速度冲击波形识别（环形缓冲 + 触发 + 窗口分析）
// ───────────────────────────────────────────────────────────────
static void sendTapEvent() {
    MessageToMain_t msgToMain = { .cmd = MESSAGE_IMU, .intVal = 1 };
    xQueueSend(qImuToMain, &msgToMain, 0);
    mainWake();          // 瞬时唤醒主循环（L2 下拍一拍立即响应）
}

static void resetTapDetection() {
    s_capturing = false; s_postLeft = 0;
    // ★ 必须连环形缓冲一起清（唯一调用点是 WoM 唤醒后）。
    //   否则缓冲里还留着休眠【之前】的样本，而所有特征都按真实时间戳算：
    //     dtUs = r.t - prevT  → 跨越整个休眠期，可达数秒
    //     if (dev > 300) activeUs += dtUs
    //   唤醒瞬间设备正在被拍/被动（dev 必然 >300），于是 activeMs 一下冲到数千，
    //   isShake(activeMs>=150) 成立 → 唤醒后的第一拍被误判成"摇一摇"。
    //   这个毒样本要 150 个样本(≈600ms)才轮出窗口，而唤醒后全采窗只有 1500ms。
    //   清零后需重新攒够 PRE_SAMPLES(200ms) 才允许触发，换来干净的重力基线。
    s_filled = 0; s_head = 0;
}

static inline Sample& ringAt(int off) {   // off: -(PRE-1)..+POST，触发点=0
    return s_ring[(s_trigHead + off + RING_SIZE * 4) % RING_SIZE];
}

// 分析整窗，判定是否拍一拍。返回 true=TAP。
static bool analyzeTapWindow() {
    // 1) 重力方向：触发前基线(off<0)估计
    double sx = 0, sy = 0, sz = 0; int nb = 0;
    for (int off = -(PRE_SAMPLES - 1); off < 0; off++) {
        Sample& r = ringAt(off); sx += r.ax; sy += r.ay; sz += r.az; nb++;
    }
    if (nb == 0) return false;
    float gmx = sx / nb, gmy = sy / nb, gmz = sz / nb;
    float gmag = sqrtf(gmx * gmx + gmy * gmy + gmz * gmz);
    if (gmag < 1.0f) gmag = 1.0f;
    float hx = gmx / gmag, hy = gmy / gmag, hz = gmz / gmag;

    // 2) 遍历整窗提取特征（用真实时间戳算 jerk / 时长，免受采样率波动影响）
    float peakLinV = 0, peakHoriz = 0, maxJerk = 0, gMax = 0;
    float activeUs = 0;
    int   rev = 0, lastDir[3] = {0, 0, 0}, nSamp = 0;
    bool  havePrev = false; float pax = 0, pay = 0, paz = 0;
    uint32_t prevT = 0, firstT = 0, lastT = 0;

    for (int off = -(PRE_SAMPLES - 1); off <= POST_SAMPLES; off++) {
        Sample& r = ringAt(off);
        float ax = r.ax, ay = r.ay, az = r.az;
        float vert   = ax * hx + ay * hy + az * hz;
        float linV   = fabsf(vert - gmag);
        float amag2  = ax * ax + ay * ay + az * az;
        float horiz2 = amag2 - vert * vert; if (horiz2 < 0) horiz2 = 0;
        float horiz  = sqrtf(horiz2);
        float dev    = fabsf(sqrtf(amag2) - 1000.0f);

        if (linV  > peakLinV)  peakLinV  = linV;
        if (horiz > peakHoriz) peakHoriz = horiz;

        if (nSamp == 0) firstT = r.t;
        lastT = r.t; nSamp++;

        if (havePrev) {
            float dtUs = (float)(uint32_t)(r.t - prevT); if (dtUs < 1) dtUs = 1;
            float da   = sqrtf((ax-pax)*(ax-pax) + (ay-pay)*(ay-pay) + (az-paz)*(az-paz));
            float jerk = da / (dtUs * 1e-6f);              // mg/s，按真实 dt 归一化
            if (jerk > maxJerk) maxJerk = jerk;
            if (dev > ACTIVE_DEV_TH) activeUs += dtUs;      // 活跃时长按真实时间累加
        }
        pax = ax; pay = ay; paz = az; prevT = r.t; havePrev = true;

        float g[3] = { (float)r.gx, (float)r.gy, (float)r.gz };
        for (int k = 0; k < 3; k++) {
            if (fabsf(g[k]) > gMax) gMax = fabsf(g[k]);
            int dir = (g[k] > GYRO_REV_TH) ? 1 : (g[k] < -GYRO_REV_TH ? -1 : 0);
            if (dir != 0 && lastDir[k] != 0 && dir != lastDir[k]) rev++;
            if (dir != 0) lastDir[k] = dir;
        }
    }

    float vh       = peakLinV / (peakHoriz > 1 ? peakHoriz : 1);
    int   activeMs = (int)(activeUs / 1000.0f);
    float rateHz   = (nSamp > 1 && lastT > firstT) ? (nSamp - 1) * 1e6f / (float)(lastT - firstT) : 0;
    bool  gyroGate = (int32_t)(millis() - s_gyroReadyAt) >= 0;   // 陀螺 settle 后才用陀螺判据

    // 3) 判定（加速度为主；陀螺 settle 后作加强）
    // ★ 删掉原来的 "(vh < 1.0 && peakHoriz > 1200)" 一条：它和放宽后的 TAP_VH_MIN 冲突，
    //   会把拍侧面（vh≈0.9、horiz 偏大）重新判成摇，使放宽 vh 的改动失效。
    //   摇的判别交给 activeMs 和陀螺反转就够了（离线验证：shake 仍 0/30 误判）。
    bool isShake = (activeMs >= 150) || (gyroGate && rev >= 3);
    bool tap = !isShake &&
               peakLinV >= TAP_LINV_LO && peakLinV <= TAP_LINV_HI &&
               vh >= TAP_VH_MIN && maxJerk < TAP_JERK_MAX && activeMs < TAP_ACTIVE_MAX &&
               (!gyroGate || (rev < TAP_REV_MAX && gMax < TAP_GMAX_MAX));

    // 采样率(rateHz)应接近 250；若明显偏低 = IMU 被抢占/采样不足，冲击可能采漏
    LOG("[IMU] %s linV=%.0f horiz=%.0f vh=%.1f jerk=%.0f act=%dms gMax=%.0f rev=%d | 采样%.0fHz/%dpts%s\n",
        tap ? "拍一拍✓" : (isShake ? "忽略(摇/运动)" : "忽略(弱/硬磕)"),
        peakLinV, peakHoriz, vh, maxJerk, activeMs, gMax, rev, rateHz, nSamp,
        gyroGate ? "" : " [陀螺未settle]");
    return tap;
}

static void stepTapAccel(const QMI8658_Data& d) {
    Sample s;
    s.t  = micros();
    s.ax = (int16_t)lroundf(d.accelX); s.ay = (int16_t)lroundf(d.accelY); s.az = (int16_t)lroundf(d.accelZ);
    s.gx = (int16_t)lroundf(d.gyroX);  s.gy = (int16_t)lroundf(d.gyroY);  s.gz = (int16_t)lroundf(d.gyroZ);
    int cur = s_head;
    s_ring[s_head] = s;
    s_head = (s_head + 1) % RING_SIZE;
    if (s_filled < RING_SIZE) s_filled++;

    float dev = fabsf(sqrtf((float)s.ax * s.ax + (float)s.ay * s.ay + (float)s.az * s.az) - 1000.0f);
    uint32_t now = millis();

    if (!s_capturing) {
        if (dev > TRIG_TH_MG && now > s_tapCooldownUntil && s_filled >= PRE_SAMPLES) {
            s_capturing = true; s_postLeft = POST_SAMPLES; s_trigHead = cur;
        }
    } else if (--s_postLeft <= 0) {
        bool tap = analyzeTapWindow();
        s_capturing = false;
        // ★ 只有【判定成功】才进冷却。原来无论成败都冷却 500ms，加上 400ms 采集窗
        //   等于任何一次触发后失聪 900ms —— 用户拍一下没反应、本能地补一拍，
        //   第二拍正好落在失聪窗里被吞掉，体感就是"要拍好几下才有一次反应"。
        //   失败不冷却后，失聪窗只剩采集窗本身(240ms)，补拍能立刻被受理。
        //   摇动期间会因此反复触发分析，但陀螺门限(gMax 400 vs 摇的 1600+)稳稳挡住，
        //   且单次分析只是遍历 110 个样本，开销可忽略。
        if (tap) {
            s_tapCooldownUntil = now + TAP_COOLDOWN_MS;   // 防同一拍重复上报
            sendTapEvent();
        }
    }
}

// ═══════════════════════════════════════════════════════════════
//  摇一摇：IMU 陀螺仪方案（保持不变）
// ═══════════════════════════════════════════════════════════════
#define SHAKE_GYRO_TH_DPS    200.0f   // 250→200：轻摇的陀螺也算一次反转
#define SHAKE_MIN_REVERSALS  2        // 3→2：约 1.5 个来回即可（更易触发；也更易被拍误判，实测调）
#define SHAKE_WINDOW_MS      800      // 600→800：放宽累积窗口（配合 L2 唤醒后陀螺 settle 的短窗）
#define SHAKE_ACCEL_RMS_MIN  300.0f   // 400→300：加速度门槛降低，慢/轻摇也能过
#define SHAKE_COOLDOWN_MS    800

struct AxisTracker {
    int8_t  lastDir   = 0;
    int     reversals = 0;
    void update(float val, float threshold) {
        int8_t dir = 0;
        if (val >  threshold) dir =  1;
        if (val < -threshold) dir = -1;
        if (dir != 0 && lastDir != 0 && dir != lastDir) reversals++;
        if (dir != 0) lastDir = dir;
    }
    void reset() { lastDir = 0; reversals = 0; }
};

static AxisTracker  gxTrack, gyTrack, gzTrack;
static uint32_t     shakeWindowStart = 0;
static float        accelRmsAcc      = 0;
static int          accelRmsCount    = 0;
static uint32_t     lastShakeMs      = 0;

static void stepShake(float ax, float ay, float az,
                      float gx, float gy, float gz)
{
    uint32_t now = millis();

    if (now - shakeWindowStart > SHAKE_WINDOW_MS) {
        gxTrack.reset(); gyTrack.reset(); gzTrack.reset();
        accelRmsAcc = 0; accelRmsCount = 0;
        shakeWindowStart = now;
    }

    gxTrack.update(gx, SHAKE_GYRO_TH_DPS);
    gyTrack.update(gy, SHAKE_GYRO_TH_DPS);
    gzTrack.update(gz, SHAKE_GYRO_TH_DPS);

    accelRmsAcc += ax*ax + ay*ay + az*az;
    accelRmsCount++;

    int totalReversals = gxTrack.reversals + gyTrack.reversals + gzTrack.reversals;
    float accelRms = sqrtf(accelRmsAcc / accelRmsCount);

    if (totalReversals >= SHAKE_MIN_REVERSALS &&
        accelRms       >= SHAKE_ACCEL_RMS_MIN &&
        now - lastShakeMs > SHAKE_COOLDOWN_MS)
    {
        lastShakeMs = now;
        LOG("[IMU] 摇一摇！rev=%d accelRms=%.1f\n", totalReversals, accelRms);

        MessageToMain_t msgToMain = { .cmd = MESSAGE_IMU, .intVal = 2 };
        xQueueSend(qImuToMain, &msgToMain, 0);
        mainWake();

        gxTrack.reset(); gyTrack.reset(); gzTrack.reset();
        accelRmsAcc = 0; accelRmsCount = 0;
        shakeWindowStart = now;
    }
}

static bool  s_posPrev = false;
static int   s_posDiv  = 0;
static float s_lx = 0, s_ly = 0, s_lz = 0;

static void stepPosStream(float ax, float ay, float az)
{
    if (!g_posStreaming) { s_posPrev = false; return; }
    bool justStarted = !s_posPrev;
    s_posPrev = true;

    if (++s_posDiv < POS_RATE_DIV) return;   // 降到 ~20Hz
    s_posDiv = 0;

    if (!justStarted &&
        fabsf(ax - s_lx) < POS_ACC_DEADZONE &&
        fabsf(ay - s_ly) < POS_ACC_DEADZONE &&
        fabsf(az - s_lz) < POS_ACC_DEADZONE) return;
    s_lx = ax; s_ly = ay; s_lz = az;

    PosSample_t s;
    s.roll_cd  = (int16_t)roundf(ax);
    s.pitch_cd = (int16_t)roundf(ay);
    s.yaw_cd   = (int16_t)roundf(az);
    xQueueOverwrite(qPosStream, &s);
}

// ═══════════════════════════════════════════════════════════════
//  对外接口
// ═══════════════════════════════════════════════════════════════
void imuInit()
{
    delay(10);
    if (!imu.begin(IMU_SDA, IMU_SCL)) {
        LOG_FATAL("[IMU] 初始化失败，系统停止\n");
        while (1) delay(10);
    }
    delay(100);
    Wire.setClock(400000);                          // 250Hz 双传感器读需要 400k I2C
    // ★ I2C 事务超时收到 30ms：总线被 QMI8658 电气 hang 住时，读操作 30ms 内失败返回，
    //   而不是卡在底层 s_i2c_send_commands 里死等（默认可达 1s，8 次累计 >5s 触发 TWDT
    //   panic → 重启，backtrace 实测卡在 QMI8658::isDataReady→Wire→i2c_master）。
    Wire.setTimeOut(30);
    imu.setAccelRange(QMI8658_ACCEL_RANGE_8G);      // 识别用 ±8g（拍峰值≈2g）
    imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
    imu.setAccelODR(QMI8658_ACCEL_ODR_250HZ);
    imu.setGyroODR(QMI8658_GYRO_ODR_250HZ);
    imu.setAccelUnit_mg(true);
    imu.setGyroUnit_dps(true);
    imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
    s_gyroReadyAt = millis() + 170;

    // I2C 事务期禁 light-sleep 锁（只在 I2C 期间短暂持有，待机放锁正常 light-sleep）
    esp_err_t e1 = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "imu_i2c",  &s_imuNoSleep);
    esp_err_t e2 = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX,   0, "imu_freq", &s_imuMaxFreq);
    if (e1 != ESP_OK || e2 != ESP_OK)
        LOG("[IMU] ⚠ PM 锁创建失败 (nosleep=%d freq=%d)，I2C 在低功耗档可能失败\n", e1, e2);

    // WoM 唤醒中断脚（IMU_INT1=GPIO21）。★ 与 WOM_INT_SEL=0x00 配套：INT1 空闲低、触发拉高，
    //   故用【下拉】—— 在 CTRL1.INT1_EN 打开前引脚是高阻，下拉保证它稳定在低位、不误触发。
    pinMode(IMU_INT1, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(IMU_INT1), imuWoMISR, CHANGE);

    // ★ 必须在任何裸 I2C 之前探测真实从机地址（库用的是它自己探测到的私有 _address，
    //   本文件的 womWrite/womRead 走的是这里探测出来的 QMI_ADDR）。
    imuDetectAddr();

    // ★★★ 打开 INT1 的输出驱动 —— 这是 WoM 中断从来没工作过的根因。
    //   CTRL1.bit3(INT1_EN) 默认 0 = 高阻，而库的 begin() 写死 CTRL1=0x60 也没置它，
    //   于是芯片内部无论触发什么事件都驱动不了引脚，GPIO21 恒为上拉的高电平。
    //   0x68 = 0x60 | (1<<3)：保留库设的 ADDR_AI(地址自增，连读要用) 和 BE(大端)，加上 INT1_EN。
    //   必须放在 imu.begin() 之后（它会写 0x60 覆盖）。
    womWrite(REG_CTRL1, 0x68);
    LOG("[IMU] CTRL1=0x%02X（INT1_EN 已开，回读 0x%02X）\n", 0x68, womRead(REG_CTRL1));

    // light-sleep GPIO 唤醒源。极性与 WOM_INT_SEL=0x00 选的"空闲低、触发拉高"配套：
    // 空闲时 INT1 被芯片驱动为低，运动时拉高 → 高电平唤醒 + 高电平触发 ISR。
    // ⚠ 这一行还会把 attachInterrupt 设的 CHANGE 覆盖成 HIGH_LEVEL（IDF 内部调
    //   gpio_hal_set_intr_type，见 esp_driver_gpio/src/gpio.c:670），属预期行为：
    //   light-sleep 的 GPIO 唤醒源只支持电平触发。
    gpio_wakeup_enable((gpio_num_t)IMU_INT1, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    LOG("[IMU] 配置完成（纯 IMU：加速度拍一拍 + 陀螺摇一摇 + WoM 睡眠唤醒）\n");
}

// ★ I2C 总线恢复：读操作持续超时（总线被从机拉住 / 电气 hang）时调用。
//   Wire.setTimeOut 只让单次读快速失败、不 panic，但若总线真 hang，后续读会一直失败、
//   IMU 功能实质挂掉直到重启。这里重建 I2C 总线并重配传感器，让 IMU 自愈无需重启。
//   调用时 imuTask 正持 s_imuNoSleep 锁（禁 light-sleep），I2C 重初始化安全。
static void imuBusRecover()
{
    LOG("[IMU] ⚠ I2C 疑似卡死，恢复总线并重配传感器\n");
    Wire.end();
    delay(5);
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(30);
    imu.setAccelRange(QMI8658_ACCEL_RANGE_8G);
    imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
    imu.setAccelODR(QMI8658_ACCEL_ODR_250HZ);
    imu.setGyroODR(QMI8658_GYRO_ODR_250HZ);
    imu.setAccelUnit_mg(true);
    imu.setGyroUnit_dps(true);
    imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
    // ★ 必须重新打开 INT1 输出：上面这串库调用最终会走到库自己的初始化路径，
    //   而库把 CTRL1 写成 0x60（INT1_EN=0，引脚回到高阻）。不补这一行的话，
    //   总线恢复之后 WoM 中断就再也出不来了 —— 表现为"某次 I2C 抖动后摇不醒"，
    //   且因为 WoM 寄存器本身写得进去、回读也正常，极难定位。
    womWrite(REG_CTRL1, 0x68);
    s_gyroReadyAt = millis() + 170;
    s_womActive = false;   // 强制退出 WoM 记账，避免恢复后状态错位
}

// 关机进深睡前调用（imuTask 已挂起）：关闭 IMU 传感器省电。深睡靠 BOOT 键唤醒，不用 IMU。
void imuPrepareDeepSleep()
{
    imu.enableSensors(QMI8658_DISABLE_ALL);   // 关 accel+gyro，~6µA Power-Down
    LOG("[IMU] 深睡：IMU 已关闭省电\n");
}

// 关机休眠前的独立 IMU 关断：在【未经 imuInit】的纯净环境(runShutdownSleep)调用。
//   自己起一次最小 I2C，把 QMI8658 写 CTRL7=0 关闭 accel+gyro → Power-Down(~6µA)。
//   ★ 只做一次 I2C 写，随后不再碰总线（避免 light-sleep 期 I2C 事务报 INVALID_STATE）。
void imuShutdownStandalone()
{
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(30);
    imuDetectAddr();             // ★ 纯净环境，QMI_ADDR 还是默认值，必须先探测
    womWrite(REG_CTRL7, 0x00);   // 关 accel+gyro → Power-Down
    LOG("[IMU] 关机：已关闭传感器省电（独立 I2C）\n");
}

void imuTask(void *)
{
    s_imuTaskHandle = xTaskGetCurrentTaskHandle();
    uint32_t awakeUntil = millis() + IMU_AWAKE_WINDOW_MS;
    uint32_t lastGoodMs = millis();   // 上次成功读到数据的时刻（I2C hang 检测用）
    imuI2cLock(true);        // 开机采样期持锁（保 I2C；进 WoM 待机前才放）
    esp_task_wdt_add(NULL);  // 纳入 TWDT：本任务(core0 P2 最高) 卡住会饿死整核，最需点名

    while (true)
    {
        esp_task_wdt_reset();   // 喂狗（WoM 待机期已退订 TWDT，见下方阻塞点）
        healthSetTaskStage(HT_IMU, HS_I_SAMPLE);
        // ★ L0 和 L1 都按"醒着"处理：250Hz 全采、不进 WoM。
        //   L1 只是把屏幕压到 60% 亮度，CPU 仍持 240MHz 锁、屏幕也还亮着，本就不是深度
        //   省电档；让 IMU 在这一档进 WoM 反而有个坏处：拍/摇要先触发 WoM 中断、退 WoM、
        //   等陀螺 settle 才开始识别，头一下容易被吃掉。
        //   真正需要省电的是 L2（屏灭 + light-sleep），WoM 留给它。
        bool active = g_posStreaming || (g_pwrTier <= 1);
        if (active) awakeUntil = millis() + IMU_AWAKE_WINDOW_MS;

        if (!active && (int32_t)(millis() - awakeUntil) >= 0) {
            // ★ 配 WoM 失败 → 不进待机，退回继续采样，1.5s 后再试。
            //   芯片没配成 WoM 就不会有唤醒中断，此时睡下去等于把 IMU 关掉。
            //   宁可多耗点电继续采样（拍/摇仍可用），也不要变成"睡死只能按键救"。
            if (!imuEnterWoM()) {
                awakeUntil = millis() + IMU_AWAKE_WINDOW_MS;
                continue;
            }
            // ★ 重新武装 INT1 中断：上一次唤醒时 ISR 把 int_ena 清零防风暴了，不重开就再也不会响。
            //   顺带打印引脚电平：WOM_INT_SEL=0x00 下配好 WoM 后应为【低】（空闲态）。
            //   若这里是高，说明极性又反了 → 把 WOM_INT_SEL 改回 0x02，同时 pinMode 改
            //   INPUT_PULLUP、gpio_wakeup_enable 改 GPIO_INTR_LOW_LEVEL（三处必须同步）。
            // ★ 武装前先清掉残留的唤醒通知，让待机从干净状态开始。
            //   任务通知是【计数型】：采样那 1.5s 里若 imuWake() 被调用过、或 ISR 曾触发过，
            //   计数会一直挂着，随后的 ulTaskNotifyTake 会【立刻返回】→ 表现为
            //   进WoM→秒退→再进 的死循环。清在武装之前，这样武装之后收到的都是真中断。
            uint32_t stale = ulTaskNotifyTake(pdTRUE, 0);   // 非阻塞取，>0 = 有残留
            if (stale) LOG("[IMU] ⚠ 待机前清掉 %u 条残留唤醒通知\n", (unsigned)stale);

            LOG("[IMU] 武装 INT1（当前电平=%d，期望 0=空闲）\n", digitalRead(IMU_INT1));
            gpio_intr_enable((gpio_num_t)IMU_INT1);
            imuI2cLock(false);                          // ★ 待机放锁：正常 light-sleep 省电
            // ★ 死等唤醒通知，无定时兜底 —— 让"WoM 中断到底通没通"成为可观测的二值结果：
            //   摇一下立刻醒 = 通了；毫无反应 = 没通。有定时兜底的话两种情况都表现为
            //   "有时能醒"（3s 睡 + 1.5s 采 ≈ 1/3 命中率），根本分不清。
            //   ★ 与上一次删兜底的关键区别：那时没有任何系统侧唤醒通路，中断不通 = IMU
            //     永久停摆、按键也救不回来。现在 wakeRenderTasks() 会调 imuWake()，
            //     按键/App 把设备拉回 L0 时能把这里叫醒，最坏情况也只是"摇不醒、但按键能救"。
            //   ⚠ 必须先退订 TWDT：无限阻塞期间无人喂狗，8s 看门狗会 panic 重启。
            esp_task_wdt_delete(NULL);
            healthSetTaskStage(HT_IMU, HS_I_STANDBY);   // 无限阻塞（已退订 TWDT，不会被点名）
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            healthSetTaskStage(HT_IMU, HS_I_WOM);       // 醒来：持锁 + 退 WoM（I2C 事务）
            esp_task_wdt_add(NULL);
            imuI2cLock(true);                           // ★ 醒来先持锁，保恢复/采样 I2C
            imuExitWoM();                               // 恢复 250Hz（持锁中，不会卡 21Hz→39Hz）
            awakeUntil = millis() + IMU_AWAKE_WINDOW_MS;
            resetTapDetection();
            gxTrack.reset(); gyTrack.reset(); gzTrack.reset();
            continue;
        }

        // ── 采样 + 识别（持锁中，250Hz 全采）──
        QMI8658_Data d; int guard = 0;
        bool gotData = false;
        while (imu.isDataReady() && imu.readSensorData(d) && guard++ < 8) {
            gotData = true;
            bool gyroValid = (int32_t)(millis() - s_gyroReadyAt) >= 0;
            stepTapAccel(d);
            if (gyroValid)
                stepShake(d.accelX, d.accelY, d.accelZ,
                          d.gyroX,  d.gyroY,  d.gyroZ);
            stepPosStream(d.accelX, d.accelY, d.accelZ);
        }

        // ── I2C hang 自愈：active（应持续 250Hz 出数据）却连续 1s 读不到 → 恢复总线 ──
        if (gotData) {
            lastGoodMs = millis();
        } else if (active && (int32_t)(millis() - lastGoodMs) > 1000) {
            imuBusRecover();
            lastGoodMs = millis();
        }

        // ★ 下载传输期：识别被屏蔽（V1_1.cpp 传输中丢弃 IMU 事件），降到 10Hz 采样，
        //   把频繁的 I2C 读缩到最少，降低撞上总线 hang 卡死的概率。
        delay(g_xfering ? IMU_XFER_POLL_MS : IMU_POLL_MS);
    }
}
