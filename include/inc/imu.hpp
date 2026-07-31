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

// 系统侧唤醒 IMU 任务：功耗状态机离开 L2 时调用（见 V1_1.cpp wakeRenderTasks）。
// imuTask 在 WoM 待机时只等中断通知、不轮询功耗档位，没有这条通路的话，
// 按键/App 把设备唤到 L0 后 IMU 仍会停在 21Hz WoM 模式里不干活。
void imuWake();

// 关机进深睡前调用（imuTask 已挂起）：关闭 IMU 传感器省电。深睡靠 BOOT 键唤醒，不用 IMU。
void imuPrepareDeepSleep();

// 关机休眠前（未经 imuInit 的纯净环境，见 runShutdownSleep）调用：自起最小 I2C，
// 把 QMI8658 关进 Power-Down 省电。esp_restart 不复位这颗外挂芯片，否则它保留上一轮
// imuInit 的工作模式(~1.5mA)。
void imuShutdownStandalone();

#endif
