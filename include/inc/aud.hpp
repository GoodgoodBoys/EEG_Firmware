#ifndef _AUD
#define _AUD

// ─── MAX98357A 引脚定义（PCB v3.0，替换旧板 PCM5102+PCM8403 二合一）──────────
// 原理图 alivePCB-v3.0 (2026-07-06)：DAC+功放合并为一颗芯片，只剩一个使能脚。
#define AUD_SHDN      GPIO_NUM_38   // 使能/静音（经三极管接 SD_MODE#，LOW=关闭，HIGH=使能）
#define DAC_LRCK      GPIO_NUM_39   // I2S 字时钟
#define DAC_BCK       GPIO_NUM_40   // I2S 位时钟
#define DAC_DIN       GPIO_NUM_41   // I2S 数据

// ─── 默认参数 ─────────────────────────────────────────────────────────────────
#define AUD_FILE_HDR_LEN    10      // 文件头校验字节数

// ─── 错误码 ───────────────────────────────────────────────────────────────────
typedef enum {
  AUD_OK              = 0,
  AUD_ERR_I2S_INIT    = -1,
  AUD_ERR_FILE_OPEN   = -2,
  AUD_ERR_FILE_READ   = -3,
  AUD_ERR_BAD_HEADER  = -4,
  AUD_ERR_MP3_BEGIN   = -5,
} AudError_t;

// ─── 对外接口 ─────────────────────────────────────────────────────────────────
AudError_t audInit();
void       audTask(void *param);
void       audSetVolume(uint8_t vol);
void audNotifyFileUpdated();   // 文件更新后重缓存
void audSuspend();             // 配网前挂起音频
void audResume();              // 配网后恢复音频
bool audIsSuspended();         // 查询是否已挂起
bool audIsPlaying();           // 是否正在播放（供功耗状态机判断"播放中=活动"）

// P4：播放指定 mp3 文件（伙伴语音即播即删）。deleteAfterLoad=true 则加载进池后删源文件；
// 播完自动恢复 /output1.mp3 到池，不影响后续 DIY 音频。
void audPlayFile(const char* path, bool deleteAfterLoad);

uint8_t* audRecvBegin(size_t* maxBytes);
bool     audRecvCommit(size_t size);
void     audRecvAbort(void);

#endif // _AUD