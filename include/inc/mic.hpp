#ifndef _MIC
#define _MIC

#include <Arduino.h>

// PCB v3.0 引脚（原理图 alivePCB-v3.0, 2026-07-06）
#define PDM_CLK_PIN     47
#define PDM_DATA_PIN    48

#define SAMPLE_RATE     16000
#define DMA_BUF_LEN     256         // 帧数
#define DMA_BUF_COUNT   8           // 缓冲数量

// Button1 单击开始/再单击结束录音；超过此时长未结束则自动停止
#define RECORD_MAX_MS      30000

// 语音消息存储：最多存 VOICE_MAX 条，文件名 /voice_0.mp3 ... /voice_9.mp3。
// 录一条占一个空槽，存满则拒绝新录音（等传给 App 后删除释放槽位，见阶段D）。
#define VOICE_MAX          10
#define VOICE_PATH_FMT     "/voice_%d.mp3"

// 初始化麦克风
bool initMIC();
void micTask(void *micParameter);
bool micIsRecording();   // 供 Button1 单击逻辑判断这一下是"开始"还是"停止"


#endif
