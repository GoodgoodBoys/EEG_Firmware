#ifndef _LCD
#define _LCD

#include <SPI.h>
#include <LovyanGFX.hpp>

#include "inc/debug.hpp"
#include "inc/spi.hpp"
#include <LittleFS.h>
#include "inc/msg.hpp"

// ==================== 硬件引脚 ====================
// PCB v3.0 引脚（原理图 alivePCB-v3.0, 2026-07-06）
#define LCD_CS        GPIO_NUM_8
#define LCD_WIDTH     240
#define LCD_HEIGHT    320
#define LCD_BL        GPIO_NUM_17
#define LCD_RST       GPIO_NUM_12
#define LCD_DC        GPIO_NUM_18
#define LCD_ROTATION  3

// ==================== PSRAM 预加载池 ====================
//
// 架构：
//   启动时将所有视频从 SD 卡整体读入 PSRAM，同时构建帧索引表。
//   播放时零拷贝：直接将 PSRAM 指针传给 drawJpg()，
//   不经过任何中间缓冲、不读 SD 卡、不做 memcpy。
//
//   唯一瓶颈 = JPEG 解码 + SPI DMA 传输，帧率可达硬件极限。
//
#define PSRAM_LCD_POOL_SIZE     (6 * 1024 * 1024)  // 6MB 专用池
#define VIDEO_CACHE_SLOTS       8                   // 最多缓存 8 个视频（7 段动画 + target）
#define MAX_FRAMES_PER_VIDEO    4000                // 单个视频最大帧数

// 流式回退用（PSRAM 装不下时）
#define FRAME_BUF_SIZE          (100 * 1024)
#define READ_BLOCK_SIZE         (32 * 1024)

// ==================== 帧率 ====================
#define TARGET_FPS    12     // 0 = 不限速，跑满硬件极限
#define FRAME_MS      (1000 / TARGET_FPS)     // 0 = 无帧间延迟

// ==================== 视频路径 ====================
#define LCD_FREE_VIDEO              "/def/idle.mjpeg"          // 空闲循环
#define LCD_MOTION_TAP_PATH         "/def/nod.mjpeg"          // 拍一拍
#define LCD_MOTION_SHAKE_PATH       "/def/swingSwing.mjpeg"   // 摇一摇
#define LCD_MOTION_VIDEOSHOW_PATH   "/target.mjpeg"           // 拍一拍上传视频
#define LCD_SHOWUP_PATH             "/def/showUp.mjpeg"       // 开机一次
#define LCD_WINK_PATH               "/def/wink.mjpeg"         // 拍一拍送达成功
#define LCD_STARTSLEEP_PATH         "/def/startSleep.mjpeg"   // 进 L1 一次
#define LCD_SLEEPING_PATH           "/def/sleeping.mjpeg"     // startSleep 后循环

// ==================== 状态机 ====================
typedef enum {
    LCD_FREE,
    LCD_PLAYING,
    LCD_PLAYING_NONINT,
    LCD_OFF,
    LCD_ERROR,
} lcdState_t;

typedef struct {
    char    videoPath[256];
    int     playTimes;
    bool    interruptAble;
} playerInfo_t;

// ==================== 帧索引条目 ====================
typedef struct {
    uint32_t offset;    // 帧在 PSRAM 数据中的起始偏移
    uint32_t size;      // 帧大小（含 FFD8~FFD9）
} frameEntry_t;

// ==================== 运行时统计 ====================
typedef struct {
    float    fps;               // 实时帧率
    float    fpsMax;            // 历史最高帧率
    uint32_t totalFrames;       // 累计帧数
    uint32_t errorCount;        // 错误数
    uint32_t decodeMs;          // 最近一帧：JPEG 解码 + SPI 传输耗时
    uint32_t avgDecodeMs;       // 平均解码耗时
    uint32_t maxDecodeMs;       // 峰值解码耗时
    size_t   poolUsed;          // PSRAM 池已用
    size_t   poolTotal;         // PSRAM 池总量
    int      cachedVideos;      // 已缓存视频数
    int      totalCachedFrames; // 已缓存总帧数
    int      fileSize;
    bool     isPreloaded;       // 当前视频是否从 PSRAM 零拷贝播放
    char     currentFile[64];
} lcdStats_t;

// ==================== 公开 API ====================

void lcdInit();
void lcdTask(void *lcdParameter);

// 停止开机转圈并等其退出（创建 lcdTask 前调用，交接 tft 所有权）。见 lcd.cpp 启动转圈段。
void lcdStopBootSpinner();

// 全屏显示"没电"图标 holdMs 毫秒（亮度 50）。关机休眠态长按开机、复检仍没电时用。
// 纯净环境可调用（内部自行 tft.init，不预加载视频）；显示完关背光，调用方随后 esp_restart。
void lcdShowLowBatteryScreen(uint32_t holdMs);

// OTA 升级进度屏（纯净 OTA 环境调用，内部自行 tft.init、不预加载视频）。
// 文字用英文——LovyanGFX 默认字库无中文。
void lcdOtaBegin();                    // 初始化 + 画标题 + 空进度条框
void lcdOtaProgress(int pct);          // 更新进度条 + 百分比（0~100）
void lcdOtaMessage(const char* msg);   // 底部一行英文状态（WiFi.../Retry/Failed/...）

const lcdStats_t* lcdGetStats();

/** web 任务下载完新视频后调用，通知 LCD 重新加载 target 视频 */
void lcdNotifyVideoUpdated();

void backLightON();
void backLightOFF();

uint8_t* lcdTargetRecvBegin(size_t* maxBytes);
bool     lcdTargetRecvCommit(size_t size);
void     lcdTargetRecvAbort(void);

void lcdSetBatteryOverlay(bool on);

// 功耗模式：0=正常全亮  1=亮度60%  2=面板休眠+停渲染（由主循环功耗状态机调用）
void lcdSetPowerMode(int mode);

// 是否正在播放（TAP/SHAKE/target）；待机循环不算。供功耗状态机判断"播放中=活动"
bool lcdIsPlaying();

#endif