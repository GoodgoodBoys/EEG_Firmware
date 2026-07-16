/*
 * motionCapture.ino — 动作原始数据采集固件（拍一拍 / 摇一摇 / 误碰 特征分析用）
 *
 * 硬件: ESP32-S3 + QMI8658 (PCB v3.0, I2C SDA=GPIO13 SCL=GPIO14)
 * 输出: USB 串口 CSV，带 "# EVENT / # END" 事件分隔，配套 capture.py 落盘。
 *
 * 采集策略: 环形缓冲 + 宽松触发
 *   - IMU 250Hz 连续采样进环形缓冲；
 *   - 合加速度去重力后的偏移 |aMag-1000mg| > TRIG_TH 即触发（只切事件，不判决动作）；
 *   - 输出「触发前 PRE 个 + 触发后 POST 个」样本（含冲击前基线 / 冲击 / 冲击后静止）；
 *   - 事件后冷却 COOLDOWN_MS，避免余震/持续摇晃被切成碎块。
 *
 *  ★ 用最大量程采集（accel ±16g / gyro ±2048dps），避免拍击冲击削顶。
 *    拿到真实峰值后，再决定识别用多大量程。
 *
 * Arduino IDE 烧录:
 *   开发板    = ESP32S3 Dev Module
 *   USB CDC On Boot = Enabled   （否则 Serial 无输出）
 *   依赖库    = QMI8658 (lahavg/QMI8658-Arduino-Library)，与主工程同一个库
 */
#include <Wire.h>
#include <QMI8658.h>
#include <math.h>

#define IMU_SDA 13          // PCB v3.0
#define IMU_SCL 14

// ── 采集参数（可调）─────────────────────────────────────────────
#define SAMPLE_ODR_HZ   250   // 采样率；若 t_ms 间隔明显 >4ms 说明 I2C 跟不上，降载或查线
#define PRE_SAMPLES     50    // 触发前保留（≈200ms @250Hz）：含重力基线
#define POST_SAMPLES    100   // 触发后继续采（≈400ms）：含冲击 + 冲击后静止
#define TRIG_TH_MG      700   // 触发阈值 |aMag-1000mg|，宽松，只滤掉静止微动（拍/摇/碰都要采到）
#define COOLDOWN_MS     800   // 事件冷却：防余震 / 持续摇晃被切碎
// ───────────────────────────────────────────────────────────────

#define RING_SIZE  (PRE_SAMPLES + POST_SAMPLES + 8)

struct Sample { uint32_t t; int16_t ax, ay, az, gx, gy, gz; };

static QMI8658 imu;
static Sample   ring[RING_SIZE];
static int      head = 0, filled = 0;
static bool     capturing = false;
static int      postLeft = 0, trigHead = 0;
static uint32_t cooldownUntil = 0, eventId = 0;

static void dumpEvent();

void setup()
{
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) { }
    delay(200);

    if (!imu.begin(IMU_SDA, IMU_SCL)) {
        Serial.println("# ERROR imu.begin 失败，检查 I2C 接线/地址");
        while (1) delay(100);
    }
    Wire.setClock(400000);                          // 250Hz 双传感器读，需要 400k I2C

    imu.setAccelRange(QMI8658_ACCEL_RANGE_16G);     // ★ 最大量程，防拍击冲击削顶
    imu.setGyroRange(QMI8658_GYRO_RANGE_2048DPS);
    imu.setAccelODR(QMI8658_ACCEL_ODR_250HZ);
    imu.setGyroODR(QMI8658_GYRO_ODR_250HZ);
    imu.setAccelUnit_mg(true);
    imu.setGyroUnit_dps(true);
    imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
    delay(300);

    // ── 静止自检：采 500ms，报告三轴均值 → 确认 IMU 正常 + 哪个轴是垂直（重力）轴 ──
    double sx = 0, sy = 0, sz = 0; int n = 0;
    uint32_t s0 = millis();
    while (millis() - s0 < 500) {
        QMI8658_Data d;
        if (imu.isDataReady() && imu.readSensorData(d)) {
            sx += d.accelX; sy += d.accelY; sz += d.accelZ; n++;
        }
    }
    Serial.printf("# READY odr=%dHz arange=16g grange=2048dps pre=%d post=%d trig=%dmg cooldown=%dms\n",
                  SAMPLE_ODR_HZ, PRE_SAMPLES, POST_SAMPLES, TRIG_TH_MG, COOLDOWN_MS);
    if (n > 0)
        Serial.printf("# REST accelX=%.0f accelY=%.0f accelZ=%.0f (mg) n=%d  <- 看哪轴≈±1000 = 垂直轴\n",
                      sx / n, sy / n, sz / n, n);
    Serial.println("# 开始采集：每次拍/摇/碰会输出一段 # EVENT .. # END");
}

void loop()
{
    QMI8658_Data d;
    if (!(imu.isDataReady() && imu.readSensorData(d))) return;

    Sample s;
    s.t  = millis();
    s.ax = (int16_t)lroundf(d.accelX);
    s.ay = (int16_t)lroundf(d.accelY);
    s.az = (int16_t)lroundf(d.accelZ);
    s.gx = (int16_t)lroundf(d.gyroX);
    s.gy = (int16_t)lroundf(d.gyroY);
    s.gz = (int16_t)lroundf(d.gyroZ);

    int cur = head;
    ring[head] = s;
    head = (head + 1) % RING_SIZE;
    if (filled < RING_SIZE) filled++;

    float mag = sqrtf((float)s.ax * s.ax + (float)s.ay * s.ay + (float)s.az * s.az);
    float dev = fabsf(mag - 1000.0f);

    // ── 诊断心跳：非采集时每 500ms 报「这段时间采了多少样本 + 最大冲击」──
    //   静止时约 n=125 maxDev=个位~几十mg；拍下去 maxDev 应跳到几千。
    //   n=0 → IMU 没出数据(查 I2C/接线)；完全看不到 # LIVE → 串口/CDC 问题(见排查)。
    static uint32_t dbgLast = 0; static float dbgMax = 0; static uint32_t dbgN = 0;
    dbgN++; if (dev > dbgMax) dbgMax = dev;
    if (!capturing && millis() - dbgLast > 500) {
        Serial.printf("# LIVE n=%lu maxDev=%.0fmg\n", (unsigned long)dbgN, dbgMax);
        dbgLast = millis(); dbgMax = 0; dbgN = 0;
    }

    if (!capturing) {
        if (dev > TRIG_TH_MG && millis() > cooldownUntil && filled >= PRE_SAMPLES) {
            capturing = true;
            postLeft  = POST_SAMPLES;
            trigHead  = cur;                        // 触发样本 → seq=0
        }
    } else {
        if (--postLeft <= 0) {
            dumpEvent();                            // dump 期间不采（在冷却区，无所谓）
            capturing     = false;
            cooldownUntil = millis() + COOLDOWN_MS;
        }
    }
}

// 输出 ring 中 [触发前 PRE-1 .. 触发点 .. 触发后 POST]，触发点 seq=0
static void dumpEvent()
{
    eventId++;
    int total = PRE_SAMPLES + POST_SAMPLES;
    Serial.printf("# EVENT %lu samples=%d\n", (unsigned long)eventId, total);
    Serial.println("seq,t_ms,ax,ay,az,gx,gy,gz");

    int firstIdx = (trigHead - (PRE_SAMPLES - 1) + RING_SIZE * 4) % RING_SIZE;
    uint32_t base = ring[firstIdx].t;               // t_ms 相对窗口首样本

    for (int off = -(PRE_SAMPLES - 1); off <= POST_SAMPLES; off++) {
        int idx = (trigHead + off + RING_SIZE * 4) % RING_SIZE;
        Sample &r = ring[idx];
        Serial.printf("%d,%lu,%d,%d,%d,%d,%d,%d\n",
                      off, (unsigned long)(r.t - base),
                      r.ax, r.ay, r.az, r.gx, r.gy, r.gz);
    }
    Serial.printf("# END %lu\n", (unsigned long)eventId);
}
