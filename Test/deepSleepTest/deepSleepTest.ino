/**
 * deepSleepTest.ino  —  按固件的外设关断态进入深度睡眠，测深睡电流
 * =============================================================
 *  关键认识：IMU(QMI8658) 和 LCD 面板是【独立芯片】，ESP32 复位不会复位它们！
 *  如果你之前跑的正式固件把 IMU 使能了、LCD 点亮了，直接烧本 sketch 也不会关掉它们
 *  —— 它们仍保持之前状态在耗电，测出来偏高。所以睡前必须主动关：
 *    · IMU：I2C 写 CTRL7=0 关掉 accel+gyro（进 standby ~µA）
 *    · LCD：拉低 RST 把面板保持在复位态（低功耗）+ 背光关
 *    · 功放/舵机电源轨/I2S/舵机信号：拉到关断电平（全 LOW，取自固件）
 *  再把这些电平锁住(hold)，然后深睡。
 *
 *  各引脚关断电平来源（alivePCB-v3.0 + 固件代码，均为 LOW，除 CS）：
 *    IO38 功放SHDN=LOW(aud.cpp)  IO40/39/41 I2S=LOW(aud.cpp)
 *    IO15 舵机电源轨=LOW(servo.hpp)  IO16 舵机信号=LOW(servo.hpp)
 *    IO17 LCD背光=LOW(LovyanGFX invert=false,亮度0)  IO12 LCD_RST=LOW(保持复位)
 *    IO8  LCD_CS=HIGH(不选中)  IO18 LCD_DC=LOW
 *
 *  ⚠ 测量：必须【电池供电、拔 USB】，电流表串电池正极回路。
 *    仍有软件关不掉的硬件漏电会计入：电池分压器 100k+47k ≈ 8.4V/147kΩ ≈ 57µA。
 *    ESP32-S3 深睡本体 ~10µA，所以底电流主要就是那个分压器 + 各芯片 standby。
 * =============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include <WiFi.h>

// —— 引脚（与固件一致）——
#define IMU_SDA_PIN   13
#define IMU_SCL_PIN   14
// QMI8658 寄存器
#define QMI_WHO_AM_I  0x00     // 应返回 0x05
#define QMI_CTRL1     0x02     // bit0 SensorDisable=1 → 停内部振荡器(真·power-down)
#define QMI_CTRL7     0x08     // bit0 aEN, bit1 gEN；写 0 = 关加速度+陀螺

// 睡前要设成关断电平并锁住的脚：{引脚, 电平, 名字}
struct PinState { gpio_num_t pin; int level; const char* name; };
static const PinState kPins[] = {
  // { GPIO_NUM_38, 0, "功放SHDN" },
  // { GPIO_NUM_40, 0, "I2S_BCK" },
  // { GPIO_NUM_39, 0, "I2S_LRCK" },
  // { GPIO_NUM_41, 0, "I2S_DIN" },
  // { GPIO_NUM_15, 0, "舵机电源轨" },
  // { GPIO_NUM_16, 0, "舵机信号" },
  // { GPIO_NUM_17, 0, "LCD背光" },
  // { GPIO_NUM_12, 0, "LCD_RST(保持复位)" },   // 拉低=面板 held-in-reset=低功耗
  // { GPIO_NUM_8,  1, "LCD_CS(不选中)" },
  // { GPIO_NUM_18, 0, "LCD_DC" },
};

static void imuWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
static bool imuRead(uint8_t addr, uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  out = Wire.read();
  return true;
}

// 彻底关掉 IMU（QMI8658）：先回读 WHO_AM_I 确认能通信，再 CTRL7=0 关传感器 +
// CTRL1 SensorDisable=1 停内部振荡器（真·power-down，~µA）。两个地址都试。
static void imuPowerDown() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(100000);
  delay(5);
  bool found = false;
  for (uint8_t addr : { (uint8_t)0x6A, (uint8_t)0x6B }) {
    uint8_t who;
    if (!imuRead(addr, QMI_WHO_AM_I, who)) {
      Serial.printf("  IMU@0x%02X 无应答\n", addr);
      continue;
    }
    found = true;
    imuWrite(addr, QMI_CTRL7, 0x00);   // 关 accel+gyro
    imuWrite(addr, QMI_CTRL1, 0x01);   // SensorDisable=1 → 停振荡器
    Serial.printf("  IMU@0x%02X WHO_AM_I=0x%02X → 已 power-down\n", addr, who);
  }
  if (!found)
    Serial.println("  ⚠ 没找到 IMU！(接线/地址不对) → 它可能仍在耗电，就是这 1.4mA 的嫌疑");
  Wire.end();
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n===== 深睡电流测试（按固件关断态）=====");

  // ① WiFi 关
  WiFi.mode(WIFI_OFF);

  // ② IMU 主动关断（独立芯片，必须 I2C 关，否则保持之前状态耗电）
  imuPowerDown();

  // ③ 各 GPIO 设成关断电平
  for (auto& p : kPins) {
    gpio_reset_pin(p.pin);
    gpio_set_direction(p.pin, GPIO_MODE_OUTPUT);
    gpio_set_level(p.pin, p.level);
    Serial.printf("  %-16s (IO%2d) → %s\n", p.name, p.pin, p.level ? "HIGH" : "LOW");
  }
  delay(80);   // 等 LCD 面板进入复位稳定

  // ④ 锁住这些电平，让深睡期间保持（不锁的话数字脚深睡会悬空/复位）
  for (auto& p : kPins) gpio_hold_en(p.pin);
  gpio_deep_sleep_hold_en();
  Serial.println("  已锁存(deep-sleep hold)");

  // ⑤ 不设唤醒源 → 睡到底，按复位键重跑。
  //   （要按键唤醒：解注释；IO0=BOOT 是 RTC 脚，按下变低唤醒）
  // esp_sleep_enable_ext1_wakeup(BIT64(0), ESP_EXT1_WAKEUP_ANY_LOW);

  Serial.println("  >>> 进入深度睡眠，现在读电流（电池供电、拔 USB）");
  Serial.flush();
  delay(50);
  esp_deep_sleep_start();
}

void loop() { }
