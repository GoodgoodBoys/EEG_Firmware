#ifndef _IMU
#define _IMU

#include <functional>

// PCB v3.0 引脚（原理图 alivePCB-v3.0, 2026-07-06）
#define IMU_SDA GPIO_NUM_13
#define IMU_SCL GPIO_NUM_14

#define IMU_INT1 GPIO_NUM_21

enum ImuEvent {
    IMU_EVENT_SHAKE,
    IMU_EVENT_TAP,
};

using ImuEventCb = std::function<void(ImuEvent)>;

void imuInit();
void imuSetEventCallback(ImuEventCb cb);
void imuTask(void *);

// 关机进深睡前调用（imuTask 已挂起）：关闭 IMU 传感器省电。深睡靠 BOOT 键唤醒，不用 IMU。
void imuPrepareDeepSleep();

#endif
