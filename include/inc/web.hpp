#ifndef _WEB
#define _WEB

#include <WiFi.h>

#define TEST_SSID "HUAWEI-R1G96U"
#define TEST_PASSWORD "HW13924654101"

#define DEF_SERVER_IP       "j515e510.ala.cn-shenzhen.emqxsl.cn"
#define DEF_SERVER_PORT     8883
#define USER_NAME           "USER001"
#define USER_PASSWORD       "USER001"
#define TOPIC_SUB_HEADER    "dev"
#define TOPIC_PUB_HEADER    "term"

#define TOPIC_SUB_CMD       "cmd"
#define TOPIC_SUB_VIDEO     "video"



void webTask(void *webParameter);

// ══════════════════════════════════════════════════════════════
//  落盘调度（persistTask）对外接口
//    策略：【只在 L2 空闲窗口写，任何唤醒立即打断、下次续写】。L0/L1 绝不写。
//    powerManagerLoop 在 idle≥65s（进 L2 满 5s）开窗(persistForceAcquire) → 全速写；
//    任何唤醒置 g_persistAbortWrite → persistTask 下一个 flash 前停手、保留 off 续写。
// ══════════════════════════════════════════════════════════════
void persistForceAcquire();                      // 开窗：允许 persistTask 全速落盘（idle≥65s）
void persistForceRelease();                      // 关窗（与 acquire 成对）
bool persistIsPending();                         // 是否仍有待落盘内容
void persistFlushBlocking(uint32_t timeoutMs);   // 阻塞排空（关机前用，忽略打断写到完）

// ★ 唤醒立即打断落盘写入：markActivity()（V1_1.cpp）与 imuWoMISR()（imu.cpp，IRAM）置位。
//   persistTask 每个 flash 操作前查它，置位即停手保留 off；开窗时由 powerManagerLoop 清零。
extern volatile bool g_persistAbortWrite;

#endif
