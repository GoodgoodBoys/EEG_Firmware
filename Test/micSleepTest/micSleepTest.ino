/**
 * micSleepTest.ino  —  测 PDM 麦克风(U5 LMD2718T261)在深睡下的耗电
 * =============================================================
 *  背景：麦克风 VDD 常挂 3V3，深睡时 ESP 的 PDM_CLK(IO48) 悬空，很多 PDM MEMS
 *  麦克风检测不到"停时钟"就不进睡眠，会一直吃几百 µA。本 sketch 把 CLK 拉到
 *  定电平(默认 LOW)让它进睡眠，其余脚全保持高阻(不驱动，避免对着上拉漏电)。
 *
 *  测法：烧本 sketch → 电池供电、拔 USB → 测 3V3 电流，和"裸睡 0.34mA"对比：
 *    · 电流明显下降(比如掉 ~200µA) → 麦克风原来没睡，就是它；这就是省下的量。
 *    · 电流没变 → 麦克风本来就睡着/或不是靠 CLK 电平睡。把下面 MIC_CLK_SLEEP_LEVEL
 *      改成 1(HIGH) 再烧再测一次(有的麦克风是 CLK 保持高才睡)。两个都试过没变，
 *      那这颗麦克风的静态电流就是它本身，软件压不下去(要靠给它 VDD 加门控关电)。
 *
 *  ⚠ 电池供电、拔 USB、电流表串电池/3V3 回路。IMU 也一并关掉(排除变量)。
 * =============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include <WiFi.h>

#define PDM_CLK_PIN   48        // 麦克风时钟
#define PDM_DATA_PIN  47        // 麦克风数据(mic→ESP)
#define IMU_SDA_PIN   13
#define IMU_SCL_PIN   14
#define QMI_WHO_AM_I  0x00
#define QMI_CTRL1     0x02
#define QMI_CTRL7     0x08

// ★★★ 麦克风睡眠时 CLK 保持的电平：先试 0(LOW)；若电流没降，改 1(HIGH) 再测 ★★★
#define MIC_CLK_SLEEP_LEVEL   1

// —— IMU 关断(排除它的变量，同 deepSleepTest) ——
static void imuWrite(uint8_t a, uint8_t r, uint8_t v) {
  Wire.beginTransmission(a); Wire.write(r); Wire.write(v); Wire.endTransmission();
}
static bool imuRead(uint8_t a, uint8_t r, uint8_t& o) {
  Wire.beginTransmission(a); Wire.write(r);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)a, 1) != 1) return false;
  o = Wire.read(); return true;
}
static void imuPowerDown() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN); Wire.setClock(100000); delay(5);
  for (uint8_t a : { (uint8_t)0x6A, (uint8_t)0x6B }) {
    uint8_t who;
    if (!imuRead(a, QMI_WHO_AM_I, who)) { Serial.printf("  IMU@0x%02X 无应答\n", a); continue; }
    imuWrite(a, QMI_CTRL7, 0x00);
    imuWrite(a, QMI_CTRL1, 0x01);
    Serial.printf("  IMU@0x%02X WHO_AM_I=0x%02X → 已 power-down\n", a, who);
  }
  Wire.end();
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n===== 麦克风低功耗测试 =====");

  WiFi.mode(WIFI_OFF);
  imuPowerDown();                       // 关 IMU，排除变量

  // 麦克风：CLK 拉到定电平 + hold，让它进睡眠；DATA 设输入下拉(定电平不悬空)
  gpio_reset_pin((gpio_num_t)PDM_CLK_PIN);
  gpio_set_direction((gpio_num_t)PDM_CLK_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)PDM_CLK_PIN, MIC_CLK_SLEEP_LEVEL);

  gpio_reset_pin((gpio_num_t)PDM_DATA_PIN);
  gpio_set_direction((gpio_num_t)PDM_DATA_PIN, GPIO_MODE_INPUT);
  gpio_pulldown_en((gpio_num_t)PDM_DATA_PIN);

  gpio_hold_en((gpio_num_t)PDM_CLK_PIN);
  gpio_deep_sleep_hold_en();
  Serial.printf("  PDM_CLK(IO%d) → %s + hold；PDM_DATA(IO%d) → 输入下拉\n",
                PDM_CLK_PIN, MIC_CLK_SLEEP_LEVEL ? "HIGH" : "LOW", PDM_DATA_PIN);

  // 其余脚(LCD/功放/舵机等)全部保持高阻，不驱动 → 靠板上上下拉定电平，不漏电

  Serial.println("  >>> 进入深睡，测 3V3 电流，和裸睡 0.34mA 对比：");
  Serial.println("      掉了=麦克风原来没睡(省下的就是它)；没掉=改 MIC_CLK_SLEEP_LEVEL=1 再试");
  Serial.flush();
  delay(50);
  esp_deep_sleep_start();
}

void loop() {}
