#include "inc/imu.hpp"
#include "inc/msg.hpp"
#include <QMI8658.h>
#include <Wire.h>
#include <math.h>
#include "esp_pm.h"        // 仅在 I2C 事务期间短暂禁 light-sleep，防事务撞 sleep 报 INVALID_STATE
#include "esp_task_wdt.h"  // 任务级看门狗：卡死时 panic backtrace 直接点名本任务
#include "esp_sleep.h"     // light-sleep GPIO 唤醒源（WoM 运动中断唤醒深睡的 L2）
#include "driver/gpio.h"
#include "inc/debug.hpp"   // ★ 放在库头之后，使 LOG / LOG_FATAL / _DEBUG 生效

static QMI8658 imu;

extern QueueHandle_t qImuToMain; // message queue |  Imu -> main
extern QueueHandle_t qPosStream;
extern volatile bool g_posStreaming;
extern volatile int  g_pwrTier;   // 0/1/2 功耗档位（V1_1.cpp 定义）
extern volatile bool g_xfering;   // web.cpp：视频/音频下载传输中（传输期识别被屏蔽，降采样减 I2C 卡死概率）
extern void mainWake();           // 唤醒主循环（V1_1.cpp）

// ═══════════════════════════════════════════════════════════════
//  纯 IMU 动作识别（无震动开关）：拍=加速度冲击 / 摇=陀螺
//    醒着(L0/串流/唤醒窗口)：加速度 250Hz 全采做识别。
//    睡眠(L1/L2)          ：IMU 进 Wake-on-Motion 低功耗，运动经 IMU_INT1(GPIO21)
//                           硬件中断唤醒本任务；醒来退 WoM，恢复全采做识别。
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
#define TAP_VH_MIN      1.2f      // 垂直/水平比：排除摇（水平主导）
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
#define POST_SAMPLES  100         // 触发后 ≈400ms：含冲击 + 冲击后静止
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

// ── I2C 事务期禁 light-sleep 锁 ──
static esp_pm_lock_handle_t s_imuNoSleep = nullptr;
static void imuI2cLock(bool on) {
    static bool held = false;
    if (!s_imuNoSleep) return;
    if (on && !held)  { esp_pm_lock_acquire(s_imuNoSleep); held = true;  }
    if (!on &&  held) { esp_pm_lock_release(s_imuNoSleep); held = false; }
}

// ═══════════════════════════════════════════════════════════════
//  Wake-on-Motion（裸 I2C 直接配 QMI8658 寄存器；均在持锁状态下调用）
//    ⚠ 库自带 enableWakeOnMotion() 残缺（没发 CTRL9 命令 / 没配 INT 路由），故自己写。
//      以下值可能需按实测微调（见文末排查）：WOM_INT_SEL / WOM_THRESH_MG / INT 极性。
// ═══════════════════════════════════════════════════════════════
#define QMI_ADDR        0x6A      // = QMI8658_ADDRESS_LOW
#define REG_CTRL2       0x03
#define REG_CTRL7       0x08
#define REG_CTRL9       0x0A
#define REG_CAL1_L      0x0B
#define REG_CAL1_H      0x0C
#define REG_STATUSINT   0x2D
#define CMD_WRITE_WOM   0x08
#define CMD_ACK         0x00
#define ACC_ODR_LP_21HZ 0x0D      // CTRL2：±2g + 低功耗 21Hz

#define WOM_THRESH_MG   100       // WoM 唤醒阈值(mg)：偏低确保拍/碰都能唤醒
#define WOM_INT_SEL     1         // CAL1_H[7:6] 选 INT 引脚：先试 1(INT1)，没唤醒改 0
#define WOM_BLANK       0x02      // CAL1_H[5:0] 消隐样本数
#define WOM_FALLBACK_MS 3000      // 兜底超时：WoM 没配通也不睡死；调通后可加大省电

static volatile bool s_womActive = false;

static void womWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(QMI_ADDR);
    Wire.write(reg); Wire.write(val);
    Wire.endTransmission();
}
static uint8_t womRead(uint8_t reg) {
    Wire.beginTransmission(QMI_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)QMI_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}
static void ctrl9Cmd(uint8_t cmd) {   // CTRL9 命令握手：发命令→等 CmdDone→ACK→等清除
    womWrite(REG_CTRL9, cmd);
    uint32_t t0 = millis();
    while (!(womRead(REG_STATUSINT) & 0x80) && millis() - t0 < 100) delay(1);
    womWrite(REG_CTRL9, CMD_ACK);
    t0 = millis();
    while ((womRead(REG_STATUSINT) & 0x80) && millis() - t0 < 100) delay(1);
}

static void imuEnterWoM() {   // ★ 调用前须持锁（保 I2C 事务成功、WoM 真正配上）
    if (s_womActive) return;
    womWrite(REG_CTRL7, 0x00); delay(2);                    // 关所有传感器
    womWrite(REG_CTRL2, ACC_ODR_LP_21HZ);                   // ±2g + 21Hz 低功耗
    womWrite(REG_CAL1_L, WOM_THRESH_MG);
    womWrite(REG_CAL1_H, (uint8_t)((WOM_INT_SEL << 6) | WOM_BLANK));
    ctrl9Cmd(CMD_WRITE_WOM);
    womWrite(REG_CTRL7, 0x01);                              // 只开加速度
    s_womActive = true;
    LOG("[IMU] → WoM 待机（阈值%dmg，等运动中断 GPIO%d 唤醒）\n", WOM_THRESH_MG, IMU_INT1);
}

static void imuExitWoM() {    // ★ 调用前须持锁（保恢复 250Hz 的写成功，不卡 21Hz→39Hz）
    if (!s_womActive) return;
    womWrite(REG_CTRL7, 0x00); delay(2);
    imu.setAccelRange(QMI8658_ACCEL_RANGE_8G);
    imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
    imu.setAccelODR(QMI8658_ACCEL_ODR_250HZ);
    imu.setGyroODR(QMI8658_GYRO_ODR_250HZ);
    imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
    s_gyroReadyAt = millis() + 170;
    s_womActive = false;
    LOG("[IMU] ← 退出 WoM，恢复 250Hz 全采（陀螺 settle 中）\n");
}

// WoM 运动中断：把阻塞的 IMU 任务唤醒去采样（极性不确定 → CHANGE 双沿最稳）
void IRAM_ATTR imuWoMISR() {
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
    bool isShake = (activeMs >= 150) || (vh < 1.0f && peakHoriz > 1200) ||
                   (gyroGate && rev >= 3);
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
        s_tapCooldownUntil = now + TAP_COOLDOWN_MS;
        if (tap) sendTapEvent();
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
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "imu_i2c", &s_imuNoSleep);

    // WoM 唤醒中断脚（IMU_INT1=GPIO21）。极性/驱动方式不确定 → CHANGE 双沿 + 上拉最稳。
    pinMode(IMU_INT1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IMU_INT1), imuWoMISR, CHANGE);

    // ★ 把 IMU_INT1 注册为 light-sleep 唤醒源：L2 深睡时运动中断能立即唤醒 CPU（否则只靠
    //   WOM_FALLBACK_MS 定时醒来采样，瞬间的拍一拍容易错过采样窗口 → L2 难唤醒）。
    //   极性推测：INPUT_PULLUP 空闲高、运动拉低 → 低电平唤醒。若真机发现 L2 一直不省电
    //   （被误唤醒），说明极性反，改成 GPIO_INTR_HIGH_LEVEL。
    gpio_wakeup_enable((gpio_num_t)IMU_INT1, GPIO_INTR_LOW_LEVEL);
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
    s_gyroReadyAt = millis() + 170;
    s_womActive = false;   // 强制退出 WoM 记账，避免恢复后状态错位
}

// 关机进深睡前调用（imuTask 已挂起）：关闭 IMU 传感器省电。深睡靠 BOOT 键唤醒，不用 IMU。
void imuPrepareDeepSleep()
{
    imu.enableSensors(QMI8658_DISABLE_ALL);   // 关 accel+gyro，~6µA Power-Down
    LOG("[IMU] 深睡：IMU 已关闭省电\n");
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
        esp_task_wdt_reset();   // 喂狗（WoM 待机阻塞 3s < 5s，唤醒后回到这里）
        bool active = g_posStreaming || (g_pwrTier == 0);
        if (active) awakeUntil = millis() + IMU_AWAKE_WINDOW_MS;

        if (!active && (int32_t)(millis() - awakeUntil) >= 0) {
            imuEnterWoM();                              // 配 WoM（持锁中，I2C 稳，真正配上）
            imuI2cLock(false);                          // ★ 待机放锁：正常 light-sleep 省电
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(WOM_FALLBACK_MS)); // 等 WoM 中断（兜底防睡死）
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
