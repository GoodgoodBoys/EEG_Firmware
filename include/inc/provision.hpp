#ifndef _PROVISION_HPP
#define _PROVISION_HPP

#include <Arduino.h>

// ══════════════════════════════════════════════════════════════
//  配网模块（BLE 凭据下发）
//
//  流程：
//    触发源：长按 IO0 → g_enterProvisioning；或无凭据/连接失败
//    webTask 调用 provisionButtonFlow()：
//      ① 开始 BLE 广播，等 BLE 连接
//      ② BLE 连上 → 断开当前 WiFi（保留旧凭据）
//      ③ 等 APP 经 BLE 下发 {ssid, password}
//      ④ 连接成功 → 覆盖保存凭据 → BLE read 发 {"status":"online"}
//      ⑤ 等 APP 回 {"connection":"disconnect"} → 关闭 BLE+配网
// ══════════════════════════════════════════════════════════════

extern volatile bool g_enterProvisioning;

// 配网阶段（lcd.cpp 读取显示提示文字）：0=未配网 1=等BLE 2=BLE已连待配置 3=连WiFi中
extern volatile int      g_provStage;
extern volatile uint32_t g_provDeadline;   // 阶段1 倒计时截止 millis

bool provisionNeeded();
void provisionButtonFlow();

#endif