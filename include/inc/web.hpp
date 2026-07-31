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
//    策略：作业入队后按兵不动，等「落盘窗口」再写——进 L2 放锁前 / 新传输前 / 关机前。
//    进 L2 那一刻屏幕已休眠、IMU 已进 WoM、播放已停，但还没放锁进 light-sleep，
//    是 flash 落盘的黄金窗口：对用户零感知、全速、且不与 light-sleep 冲突。
//    开窗用引用计数（多个调用方可并存，如"新传输前排空"与"进 L2 前落盘"重叠）。
// ══════════════════════════════════════════════════════════════
void persistForceAcquire();                      // 开窗：让 persistTask 立即全速落盘
void persistForceRelease();                      // 关窗（与 acquire 成对）
bool persistIsPending();                         // 是否仍有在途落盘作业
void persistFlushBlocking(uint32_t timeoutMs);   // 阻塞排空（关机前用）

#endif
