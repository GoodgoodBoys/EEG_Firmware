#ifndef _HEALTH
#define _HEALTH

#include <stdint.h>

// ══════════════════════════════════════════════════════════════
//  启动阶段标记（BOOT_STAGE）
//
//  为什么需要：黑匣子原本只能回答"上次是不是崩了"，回答不了"崩在哪"——
//  minFreeHeap / stkMinFree / maxUptime 全部由 healthBoxSample() 采集，而它只在
//  loop() 的资源监控里每 5s 跑一次。【启动期崩溃时 loop 一次都没执行过】，那些字段
//  要么停在初始值、要么还是上一次运行留下的旧值，对现场毫无参考价值。
//  实测踩过：一台设备连崩 6 次（boot=7/crash=6），读到的栈水位和堆余量却全部来自
//  第 7 次成功启动之后 —— 拿它去推断崩溃原因会被直接带偏。
//
//  做法：setup 每走一步就往 RTC 内存写一个步号。panic / 看门狗 / 软复位都不清 RTC，
//  下次启动 healthBoxInit() 把它转存进 lastStage 随健康快照上报 ——
//  "上次停在第几步"直接变成一个可读数字，不需要串口也能定位启动期崩溃。
//  代价：每步一条 store 指令，零开销。
//
//  ★ 只有真正掉电才会丢（那时 magic 也对不上，lastStage 归 0 = 无记录）。
// ══════════════════════════════════════════════════════════════
enum BootStage {
    BS_NONE        = 0,    // 首次上电 / 无记录（RTC 被清过）
    BS_FS          = 1,    // 挂载 LittleFS
    BS_SN          = 2,    // SN 校验（NVS）
    BS_PM          = 3,    // 电源管理配置 + PM 锁创建
    BS_CFG_BLE     = 4,    // 加载配置 + BLE 名
    BS_LCD_INIT    = 5,    // LCD 面板初始化（tft.init + 起开机转圈）

    // 10..17 = 预加载第 (s - BS_LCD_PRELOAD) 个视频。这一段是启动期最长的环节，
    // 单独细分到"哪个文件"——半截/损坏的 target.mjpeg 是最容易怀疑的对象。
    BS_LCD_PRELOAD = 10,
    BS_LCD_DONE    = 18,   // 预加载收尾（统计 + 排开机 showUp）

    BS_AUD         = 20,   // 音频：I2S 探测 + 600KB 池 + 预加载 /output1.mp3
    BS_MIC         = 21,   // 麦克风 PDM（I2S0）
    BS_IMU         = 22,   // IMU QMI8658（I2C）
    BS_SERVO       = 23,   // 舵机上电 + 归中位（启动期最大电流事件）
    BS_BAT         = 24,   // 电量 ADC
    BS_QUEUE       = 25,   // 队列 / 信号量创建
    BS_TASKS       = 26,   // 任务创建 + TWDT reconfigure

    BS_RUNNING     = 99,   // setup 完成、已进 loop：此后崩溃 = 运行期问题，不是启动期
    BS_SHUTDOWN    = 200,  // 正常关机（enterShutdown）——与崩溃明确区分开
};

// 打一个启动阶段点。定义在 V1_1.cpp（g_hb 所在处），供 lcd.cpp 等模块细分内部步骤。
void healthSetStage(uint32_t stage);

// 步号 → 可读名字（日志 + 健康上报共用）。
// ★ 非线程安全：预加载分支用了 static 缓冲。只在 setup 和上报路径调用，均为单线程。
const char* healthStageName(uint32_t stage);


// ══════════════════════════════════════════════════════════════
//  运行期阶段标记（每任务一格）
//
//  为什么要按任务分格而不是复用 curStage：curStage 是单一变量，5 个任务并发写会
//  互相覆盖，崩溃时读到的是"最后一个写的人"，毫无意义。每任务独占一格才能回答
//  "看门狗超时那一刻，每个任务分别卡在哪"。
//
//  为什么只存数字、不带名字表：Flash 已经用到 95%+，一张字符串表不划算。
//  数字直接进健康 JSON 的 "ts" 数组，对照表就在下面。
//
//  ★ 写入成本 = 一条 store 指令，可以放在热路径里（每帧/每包）。
//  ★ persistTask 虽然不在 TWDT 监控名单里，但它写 flash 会 IPC 冻结 core0 cache，
//    是"IDLE0 被饿死"的头号嫌疑 —— 必须占一格，否则看门狗点名 IDLE0 时无从对证。
// ══════════════════════════════════════════════════════════════
enum HealthTaskSlot {
    HT_WEB     = 0,
    HT_LCD     = 1,
    HT_IMU     = 2,
    HT_MIC     = 3,
    HT_PERSIST = 4,
    HT_SLOTS   = 5,
};

// ── 阶段码（0 = 未设置/未运行）。同一格内的数字含义见各行注释。──
enum {
    // HT_WEB
    HS_W_IDLE      = 1,   // 空闲轮询 client.loop()
    HS_W_WIFI      = 2,   // 连 WiFi（阻塞最长 30s）
    HS_W_MQTT      = 3,   // 连 MQTT / TLS 握手  ★ 单次内部 RAM 峰值最大的地方
    HS_W_RXCHUNK   = 4,   // 收音视频分片（appendRecv）
    HS_W_COMMIT    = 5,   // 收尾提交：校验 + 建帧索引（可能阻塞秒级）
    HS_W_WAITPERS  = 6,   // 等落盘让路（cancelPersistForReuse / persistFlushBlocking）
    HS_W_VOICETX   = 7,   // 语音上行：读文件 + publish
    HS_W_PEERRX    = 8,   // 伙伴语音落盘（写 LittleFS）
    HS_W_REPORT    = 9,   // devInfo / health 上报

    // HT_LCD
    HS_L_IDLE      = 1,   // 待机 / 阻塞等唤醒
    HS_L_DECODE    = 2,   // JPEG 解码 + 推屏
    HS_L_FILE      = 3,   // 读文件（流式回退路径）
    HS_L_RELOAD    = 4,   // 重载 target 视频
    HS_L_WAITPERS  = 5,   // 等 persist 空闲

    // HT_IMU
    HS_I_SAMPLE    = 1,   // 正常采样
    HS_I_I2C       = 2,   // I2C 读写（历史卡死点）
    HS_I_WOM       = 3,   // 进/出 WoM 模式切换
    HS_I_STANDBY   = 4,   // WoM 待机（已退订 TWDT）

    // HT_MIC
    HS_M_IDLE      = 1,
    HS_M_RECORD    = 2,   // 采集
    HS_M_ENCODE    = 3,   // Shine MP3 编码
    HS_M_WRITE     = 4,   // 写文件

    // HT_PERSIST
    HS_P_IDLE      = 1,   // 等窗口
    HS_P_WRITE     = 2,   // ★ 正在写 flash（此刻 core0 cache 被冻结）
    HS_P_RENAME    = 3,   // 收尾 rename
};

// 打一个运行期阶段点。slot 取 HealthTaskSlot，stage 取上面对应段的 HS_*。
// ★ 必须是 ISR 安全 + 无锁 + 无阻塞：它会被放进热路径，也会被 TWDT 的 ISR 读取。
//
// ★ 语义是【最后进入的阶段】，函数出口不做恢复 —— 恢复得处理每一条 return 分支，
//   改动面大、容易漏，为一个诊断字段不值得冒这个险。各任务的主循环每轮会把自己
//   重置回 IDLE 档（webTask 在 client.loop() 前、lcdTask 每帧、micTask 每轮），
//   所以"阶段已退出但值还留着"的误读窗口不超过一轮，对定位卡死足够了。
void healthSetTaskStage(uint8_t slot, uint8_t stage);

#endif  // _HEALTH
