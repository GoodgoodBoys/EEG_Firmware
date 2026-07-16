/*
 * motionDetectTest.ino — IMU 动作检测实时验证固件（拍一拍 / 摇一摇 / 硬磕）
 *
 * 把 events/ 数据标定出的判据直接跑在设备上：每次动作在串口打印一行判定 + 特征，
 * 用 Arduino 串口监视器(115200)直接看，不需要 capture.py。
 *
 * 判据（来自 40拍/30摇/20碰 标定，纯加速度主判 + 陀螺加强）：
 *   TAP  =  700 ≤ linV ≤ 3000mg     (下界滤噪 / 上界砍硬磕)
 *        &&  vh_ratio ≥ 1.2          (垂直主导 → 排除摇)
 *        &&  maxΔa < 4000mg          (软接触 → 排除硬磕，等价 jerk<1e6)
 *        &&  active_ms < 100         (短冲击 → 再排除持续运动/摇)
 *        &&  reversals < 2 && gMax<400 (陀螺加强，排除摇)
 *   实测: tap 40/40, shake 误0, bump 误1/20
 *
 * 硬件: ESP32-S3 + QMI8658 (SDA=13 SCL=14)。Arduino: USB CDC On Boot = Enabled。
 */
#include <Wire.h>
#include <QMI8658.h>
#include <math.h>

#define IMU_SDA 13
#define IMU_SCL 14

// ── 采集/触发参数 ──
#define ODR_HZ         250
#define DT_MS          4              // 1000/250
#define PRE_SAMPLES    50             // 触发前(≈200ms)：算重力基线
#define POST_SAMPLES   100            // 触发后(≈400ms)：含冲击+静止
#define TRIG_TH_MG     500            // 触发门限 |aMag-1000|，宽松，拍/摇/碰都进来分类
#define COOLDOWN_MS    500

// ── 判据阈值（现场可调）──
#define TAP_LINV_LO    700.0f
#define TAP_LINV_HI    3000.0f
#define TAP_VH_MIN     1.2f
#define TAP_DA_MAX     4000.0f        // 单样本相邻加速度差上限(mg)
#define TAP_ACTIVE_MAX 100            // ms
#define ACTIVE_DEV_TH  300.0f         // 算 active_ms 用的 |a|-1000 门限
#define GYRO_REV_TH    250.0f         // 陀螺方向反转判定(dps)
#define TAP_GMAX_MAX   400.0f
#define TAP_REV_MAX    2

#define RING_SIZE  (PRE_SAMPLES + POST_SAMPLES + 8)

struct Sample { int16_t ax, ay, az, gx, gy, gz; };

static QMI8658 imu;
static Sample   ring[RING_SIZE];
static int      head = 0, filled = 0;
static bool     capturing = false;
static int      postLeft = 0, trigHead = 0;
static uint32_t cooldownUntil = 0, evId = 0;

static void analyzeWindow();

void setup()
{
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) {}
    delay(200);

    if (!imu.begin(IMU_SDA, IMU_SCL)) {
        Serial.println("# ERROR imu.begin 失败");
        while (1) delay(100);
    }
    Wire.setClock(400000);
    imu.setAccelRange(QMI8658_ACCEL_RANGE_8G);      // 识别用 ±8g 足够（拍峰值≈2g）
    imu.setGyroRange(QMI8658_GYRO_RANGE_512DPS);
    imu.setAccelODR(QMI8658_ACCEL_ODR_250HZ);
    imu.setGyroODR(QMI8658_GYRO_ODR_250HZ);
    imu.setAccelUnit_mg(true);
    imu.setGyroUnit_dps(true);
    imu.enableSensors(QMI8658_ENABLE_ACCEL | QMI8658_ENABLE_GYRO);
    delay(300);

    Serial.println("# 动作检测测试就绪：拍/摇/碰，看下方判定");
    Serial.println("# 判定  linV(垂直冲击) horiz(水平) vh(比) dA(锐度) act(时长) gMax(陀螺) rev(反转)");
}

void loop()
{
    QMI8658_Data d;
    if (!(imu.isDataReady() && imu.readSensorData(d))) return;

    Sample s = {
        (int16_t)lroundf(d.accelX), (int16_t)lroundf(d.accelY), (int16_t)lroundf(d.accelZ),
        (int16_t)lroundf(d.gyroX),  (int16_t)lroundf(d.gyroY),  (int16_t)lroundf(d.gyroZ)
    };
    int cur = head;
    ring[head] = s;
    head = (head + 1) % RING_SIZE;
    if (filled < RING_SIZE) filled++;

    float dev = fabsf(sqrtf((float)s.ax * s.ax + (float)s.ay * s.ay + (float)s.az * s.az) - 1000.0f);

    if (!capturing) {
        if (dev > TRIG_TH_MG && millis() > cooldownUntil && filled >= PRE_SAMPLES) {
            capturing = true; postLeft = POST_SAMPLES; trigHead = cur;
        }
    } else if (--postLeft <= 0) {
        analyzeWindow();
        capturing = false;
        cooldownUntil = millis() + COOLDOWN_MS;
    }
}

// 取 ring 中第 off 个样本（off: -(PRE-1)..+POST，触发点=0）
static inline Sample& at(int off) {
    return ring[(trigHead + off + RING_SIZE * 4) % RING_SIZE];
}

static void analyzeWindow()
{
    // ── 1) 重力方向：用触发前基线(off<0，纯静止段)估计 ──
    double sx = 0, sy = 0, sz = 0; int nb = 0;
    for (int off = -(PRE_SAMPLES - 1); off < 0; off++) {
        Sample& r = at(off); sx += r.ax; sy += r.ay; sz += r.az; nb++;
    }
    float gmx = sx / nb, gmy = sy / nb, gmz = sz / nb;
    float gmag = sqrtf(gmx * gmx + gmy * gmy + gmz * gmz);
    if (gmag < 1.0f) gmag = 1.0f;
    float hx = gmx / gmag, hy = gmy / gmag, hz = gmz / gmag;   // 重力单位向量

    // ── 2) 遍历整窗提取特征 ──
    float peakLinV = 0, peakHoriz = 0, maxDA = 0, gMax = 0;
    int   activeCnt = 0;
    bool  havePrev = false; float pax = 0, pay = 0, paz = 0;
    int   lastDir[3] = {0, 0, 0}, rev = 0;

    for (int off = -(PRE_SAMPLES - 1); off <= POST_SAMPLES; off++) {
        Sample& r = at(off);
        float ax = r.ax, ay = r.ay, az = r.az;

        float vert  = ax * hx + ay * hy + az * hz;     // 沿重力轴分量
        float linV  = fabsf(vert - gmag);              // 去重力后的垂直冲击
        float amag2 = ax * ax + ay * ay + az * az;
        float horiz2 = amag2 - vert * vert; if (horiz2 < 0) horiz2 = 0;
        float horiz = sqrtf(horiz2);

        if (linV  > peakLinV)  peakLinV  = linV;
        if (horiz > peakHoriz) peakHoriz = horiz;
        if (fabsf(sqrtf(amag2) - 1000.0f) > ACTIVE_DEV_TH) activeCnt++;

        if (havePrev) {
            float da = sqrtf((ax - pax) * (ax - pax) + (ay - pay) * (ay - pay) + (az - paz) * (az - paz));
            if (da > maxDA) maxDA = da;
        }
        pax = ax; pay = ay; paz = az; havePrev = true;

        // 陀螺：峰值 + 三轴方向反转次数
        float g[3] = { (float)r.gx, (float)r.gy, (float)r.gz };
        for (int k = 0; k < 3; k++) {
            if (fabsf(g[k]) > gMax) gMax = fabsf(g[k]);
            int dir = (g[k] > GYRO_REV_TH) ? 1 : (g[k] < -GYRO_REV_TH ? -1 : 0);
            if (dir != 0 && lastDir[k] != 0 && dir != lastDir[k]) rev++;
            if (dir != 0) lastDir[k] = dir;
        }
    }

    float vh = peakLinV / (peakHoriz > 1 ? peakHoriz : 1);
    int   activeMs = activeCnt * DT_MS;

    // ── 3) 判定 ──
    const char* verdict;
    bool isShake = (rev >= 3) || (activeMs >= 150) || (vh < 1.0f && peakHoriz > 1200);
    if      (isShake)                     verdict = "SHAKE   ";
    else if (peakLinV < TAP_LINV_LO)      verdict = "IGN(弱) ";
    else if (peakLinV > TAP_LINV_HI ||
             maxDA    > TAP_DA_MAX)       verdict = "IGN(硬磕)";
    else if (vh >= TAP_VH_MIN && activeMs < TAP_ACTIVE_MAX &&
             rev < TAP_REV_MAX && gMax < TAP_GMAX_MAX)
                                          verdict = "TAP  ✓  ";
    else                                  verdict = "IGN(不明)";

    evId++;
    Serial.printf("[%lu] %s linV=%4.0f horiz=%4.0f vh=%.1f dA=%4.0f act=%3dms gMax=%4.0f rev=%d\n",
                  (unsigned long)evId, verdict,
                  peakLinV, peakHoriz, vh, maxDA, activeMs, gMax, rev);
}
