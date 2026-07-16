#ifndef _SERVO
#define _SERVO

#include <Arduino.h>
#include "debug.hpp"

// PCB v3.0 引脚（原理图 alivePCB-v3.0, 2026-07-06）
#define SERVO_PIN          16

// v3.0 新增：舵机供电轨高边开关（旧板舵机直接接电池、常通电）。
#define SERVO_POWER_PIN    15

// 舵机限位：中位 87°，正负 35° → 52~122°。所有舵机动作都会被 constrain 到这个范围
// （servoRotate / servoRotateAV 都钳制），任何摇摆系数都不会超出。
#define SERVO_ANGLE_MIN    52
#define SERVO_ANGLE_MAX    122
#define SERVO_MAX_SPEED    500

// ── LEDC 资源（避开背光 channel0/timer0）──
#define SERVO_LEDC_CHANNEL 1
#define SERVO_LEDC_TIMER   1
#define SERVO_LEDC_FREQ    50          // 50Hz 舵机
#define SERVO_LEDC_RES     14          // 14位分辨率 = 16383
#define SERVO_LEDC_MAXDUTY ((1 << SERVO_LEDC_RES) - 1)

// 脉宽范围（微秒）
#define SERVO_US_MIN       500
#define SERVO_US_MAX       2500
#define SERVO_PERIOD_US    20000       // 50Hz → 20000us

// 省电
#define SERVO_REST_ANGLE      87
#define SERVO_REST_TOL        5
#define SERVO_IDLE_MS         1000
#define SERVO_POWER_SETTLE_MS 20     // 供电轨重新上电后的稳定时间（电容充电+舵机内部复位）

int currentAngle = 0;
static bool          servoAttached   = false;
static bool          servoPowered    = false;
static unsigned long lastServoMoveMs = 0;

// 供电轨开关：断电即彻底省电（无 PWM 时也不再耗电），用前需重新上电
static inline void servoPowerOn()
{
  if (servoPowered) return;
  digitalWrite(SERVO_POWER_PIN, HIGH);
  servoPowered = true;
  delay(SERVO_POWER_SETTLE_MS);      // 等供电轨稳定，避免舵机上电瞬间乱抖
  LOG("[SERVO] 供电轨已开启\n");
}

static inline void servoPowerOff()
{
  if (!servoPowered) return;
  digitalWrite(SERVO_POWER_PIN, LOW);
  servoPowered = false;
  LOG("[SERVO] 供电轨已断开，省电\n");
}

// 角度 → LEDC duty
static inline uint32_t servoAngleToDuty(int angle)
{
  uint32_t us = SERVO_US_MIN +
                (uint32_t)((SERVO_US_MAX - SERVO_US_MIN) * (long)angle / 180);
  return (uint32_t)((uint64_t)us * SERVO_LEDC_MAXDUTY / SERVO_PERIOD_US);
}

static inline void servoWriteAngle(int angle)
{
  ledcWrite(SERVO_PIN, servoAngleToDuty(angle));
}

static inline void servoEnsureAttached()
{
  servoPowerOn();   // 用前先确保供电轨已开启（可能被 servoLoop 断过电）
  if (!servoAttached) {
    // 用裸 LEDC 接管引脚（arduino-esp32 3.x 新 ledcAttach API）
    ledcAttachChannel(SERVO_PIN, SERVO_LEDC_FREQ, SERVO_LEDC_RES, SERVO_LEDC_CHANNEL);
    servoAttached = true;
    LOG("[SERVO] 重新 attach (LEDC ch%d)\n", SERVO_LEDC_CHANNEL);
  }
}

void servoInit()
{
  LOG("[SERVO] 舵机初始化(LEDC)...\n");

  pinMode(SERVO_POWER_PIN, OUTPUT);
  digitalWrite(SERVO_POWER_PIN, LOW);
  servoPowered = false;
  servoPowerOn();   // 开机先上电一次，归中位

  ledcAttachChannel(SERVO_PIN, SERVO_LEDC_FREQ, SERVO_LEDC_RES, SERVO_LEDC_CHANNEL);
  servoAttached   = true;
  lastServoMoveMs = millis();
  servoWriteAngle(SERVO_REST_ANGLE);   // 上电归中位
  currentAngle = SERVO_REST_ANGLE;
  LOG("[SERVO] ✓ 初始化完成，归中位 %d°\n", SERVO_REST_ANGLE);
}

void servoRotate(int angle)
{
  angle = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
  servoEnsureAttached();
  currentAngle = angle;
  servoWriteAngle(angle);
  lastServoMoveMs = millis();
  LOG("[SERVO] 转动到：%d度\n", angle);
}

void servoRotateAV(int angle, int angularVelocity)
{
  angle = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);   // ★ 强制限位，任何摇摆都不超范围
  if (angularVelocity <= 0 || angle == currentAngle) return;
  if (angularVelocity > SERVO_MAX_SPEED) { servoRotate(angle); return; }

  servoEnsureAttached();
  int direction = (angle > currentAngle) ? 1 : -1;
  int total = abs(angle - currentAngle);
  unsigned long stepUs = (unsigned long)(1000000.0 / angularVelocity);
  unsigned long stepMs = stepUs / 1000;
  unsigned long remUs  = stepUs % 1000;

  for (int i = 0; i < total; i++) {
    currentAngle += direction;
    servoWriteAngle(currentAngle);
    if (stepMs > 0) delay(stepMs);
    if (remUs  > 0) delayMicroseconds(remUs);
  }
  lastServoMoveMs = millis();
}

// 省电：回中位静止超时 → detach（停 PWM）+ 断开供电轨（彻底断电）
void servoLoop()
{
  if (servoAttached &&
      abs(currentAngle - SERVO_REST_ANGLE) <= SERVO_REST_TOL &&
      (millis() - lastServoMoveMs) > SERVO_IDLE_MS) {
    ledcDetach(SERVO_PIN);           // 停 LEDC 输出
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);    // 强拉低，确保无脉冲
    servoAttached = false;
    LOG("[SERVO] 中位静止%dms，detach 省电\n", SERVO_IDLE_MS);

    servoPowerOff();                 // 此刻已静止在中位，断开供电轨彻底省电
  }
}

#endif