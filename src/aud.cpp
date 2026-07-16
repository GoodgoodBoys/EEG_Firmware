#include <Arduino.h>
#include <LittleFS.h>
#include <math.h>
#include <driver/i2s_std.h>

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_STDIO
#include "inc/minimp3.h"

#include "inc/aud.hpp"
#include "inc/msg.hpp"
#include "inc/configSys.hpp"
#include "inc/mic.hpp"      // 复用 SAMPLE_RATE(16000)，与录音回放保持同一采样率
#include "inc/debug.hpp"   // ★ 放在库头/其它 inc 之后，使 LOG / _DEBUG 生效

extern QueueHandle_t qMainToAud;

// ── 低功耗联动（V1_1.cpp 定义）───────────────────────────────
extern volatile int      g_pwrTier;   // 0/1/2 功耗档位
extern SemaphoreHandle_t xAudWake;    // L2 唤醒信号量（powerManagerLoop 离开 L2 时 give）
#define AUD_L2_BLOCK_MS   500         // L2 最长阻塞 = light-sleep 单次时长上限

// ─── I2S 配置 ─────────────────────────────────────────────
#define I2S_DMA_BUF_CNT   4
#define I2S_DMA_BUF_LEN   512
#define I2S_DEFAULT_RATE  44100

// ─── 解码用 PCM 缓冲（PSRAM）─────────────────────────────
#define PCM_BUF_SAMPLES    MINIMP3_MAX_SAMPLES_PER_FRAME

// ─── 音频文件常驻缓存池（PSRAM）──────────────────────────
//  ★ 8MB PSRAM，这里固定预留 600KB 给 MP3 文件（不是 600MB！）
//    文件超过此上限会拒绝缓存并回退到即时加载。
#define AUDIO_POOL_SIZE    (600 * 1024)
#define AUDIO_FILE_PATH    "/output1.mp3"

// ─── 音量增益（0~255）+ 软限幅 ─────────────────────────────
//  gain = volume/255 * AUD_GAIN_MAX。volume 越大越响；峰值超过拐点用 tanh 平滑压住，
//  绝不硬削波/int16 回绕（这就是"拉大音量不破音"的关键，取代原来的裸乘法）。
//  录音源偏小 → 把 volume 往上调即可拉响，安静段被压、响段不破。
#define AUD_GAIN_MAX       2.2f     // volume=255 时的最大增益（太高会一直触发限幅→发闷/杂，2~2.5 较稳）
#define AUD_LIMIT_KNEE     0.80f    // 软限幅拐点：低于此透明直通，高于此 tanh 压峰

// 软限幅：|v|≤拐点 透明；>拐点 平滑压向满幅（响而不破，不回绕）
static inline int16_t audSoftLimit(float v) {
    const float FS   = 32767.0f;
    const float knee = AUD_LIMIT_KNEE * FS;
    float a = v < 0 ? -v : v;
    if (a <= knee) return (int16_t)v;
    float over = (a - knee) / (FS - knee);
    float comp = knee + (FS - knee) * tanhf(over);
    return (int16_t)(v < 0 ? -comp : comp);
}

// ─── 内部状态 ─────────────────────────────────────────────
static mp3dec_t          s_dec;
static bool              g_isPlaying   = false;
static i2s_chan_handle_t s_i2sTxHandle = nullptr;

// 600KB 常驻池（开机一次性分配，永不释放，避免反复 malloc/free 造成 PSRAM 碎片）
static uint8_t*  s_audioPool = nullptr;    // 池起始地址
static size_t    s_fileSize  = 0;          // 已加载的文件大小（0 = 池内无有效数据）
static size_t    s_filePos   = 0;          // 当前解码读取位置

// PCM 输出缓冲（PSRAM，常驻）
static int16_t*  s_pcmBuf    = nullptr;

static uint8_t   s_volume    = 0;

// ─── 控制标志（跨核：webTask 置位，audTask 处理）──────────
static volatile bool s_reloadReq    = false;   // 文件已更新，请求重缓存
static volatile bool s_suspendReq   = false;   // 请求挂起（配网前释放 I2S）
static volatile bool s_resumeReq    = false;   // 请求恢复
static volatile bool g_audSuspended = false;   // 当前是否已挂起

// ----------------------------------------------------------
// I2S 初始化（重建总线：startPlayback / audReinitHW 调用）
//   会重新接管 BCK/LRCK/DIN 三脚（即便它们之前被 audIdleShutdown 拉低）
// ----------------------------------------------------------
static bool initI2S(int sampleRate, int channels) {
    if (s_i2sTxHandle) {
        i2s_channel_disable(s_i2sTxHandle);
        i2s_del_channel(s_i2sTxHandle);
        s_i2sTxHandle = nullptr;
    }

    // 固定用 I2S1（TX）：I2S0 留给麦克风 PDM RX（PDM 模式 ESP32-S3 上只有
    // I2S0 支持，见 mic.cpp 注释），播放走普通 I2S 模式，两个控制器随便选，
    // 用 I2S1 就不会跟麦克风抢。
    i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_NUM_1, I2S_ROLE_MASTER);
    chanCfg.dma_desc_num  = I2S_DMA_BUF_CNT;
    chanCfg.dma_frame_num = I2S_DMA_BUF_LEN;

    if (i2s_new_channel(&chanCfg, &s_i2sTxHandle, nullptr) != ESP_OK) {
        LOG("[AUD] i2s_new_channel 失败\n");
        return false;
    }

    i2s_std_config_t stdCfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)sampleRate),
        // ★ 始终立体声 slot：mono 源在 audWriteFrame 里复制成左右两路后喂进来，
        //   避免 MAX98357A 单声道 slot 出杂音（channels 参数只用来定采样率那边）。
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)DAC_BCK,
            .ws   = (gpio_num_t)DAC_LRCK,
            .dout = (gpio_num_t)DAC_DIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { false, false, false },
        },
    };

    if (i2s_channel_init_std_mode(s_i2sTxHandle, &stdCfg) != ESP_OK) {
        LOG("[AUD] i2s_channel_init_std_mode 失败\n");
        i2s_del_channel(s_i2sTxHandle); s_i2sTxHandle = nullptr;
        return false;
    }
    if (i2s_channel_enable(s_i2sTxHandle) != ESP_OK) {
        LOG("[AUD] i2s_channel_enable 失败\n");
        i2s_del_channel(s_i2sTxHandle); s_i2sTxHandle = nullptr;
        return false;
    }

    LOG("[AUD] I2S: rate=%d ch=%d\n", sampleRate, channels);
    return true;
}

// ----------------------------------------------------------
// ★ 空闲最省态（实测 4.8mA）——播放结束 / 停止 / 开机探测后调用
//   ① 关闭功放（MAX98357A 单脚 SD_MODE#）→ ② 删 I2S channel（归还外设）
//   → ③ BCK/LRCK/DIN 主动拉低（切断对 DAC 的喂时钟，这步是省电关键）
//   播放前由 startPlayback→initI2S 重建总线、重新接管这三脚并使能功放。
// ----------------------------------------------------------
static void audIdleShutdown() {
    // ① 先关闭功放，避免爆音
    digitalWrite(AUD_SHDN, LOW);

    // ② 删除 I2S channel（不仅 disable）
    if (s_i2sTxHandle) {
        i2s_channel_disable(s_i2sTxHandle);
        i2s_del_channel(s_i2sTxHandle);
        s_i2sTxHandle = nullptr;
    }

    // ③ 三脚主动拉低，切断对 DAC 的时钟/数据驱动
    pinMode(DAC_BCK,  OUTPUT); digitalWrite(DAC_BCK,  LOW);
    pinMode(DAC_LRCK, OUTPUT); digitalWrite(DAC_LRCK, LOW);
    pinMode(DAC_DIN,  OUTPUT); digitalWrite(DAC_DIN,  LOW);

    LOG("[AUD] 空闲最省态：I2S 释放 + BCK/LRCK/DIN 拉低 + 功放关闭\n");
}

// ----------------------------------------------------------
// 把 AUDIO_FILE_PATH 读入常驻池（唯一会碰 Flash 的地方）
//   - 开机 audInit() 调一次
//   - 文件更新后由 audTask 在「不播放时」调一次
// ----------------------------------------------------------
// 把指定 mp3 文件读入常驻池（AUDIO_FILE_PATH 或伙伴语音临时文件）
static bool audLoadFileToPool(const char* path) {
    s_fileSize = 0;
    s_filePos  = 0;

    if (!s_audioPool) {
        LOG("[AUD] 池未分配，无法加载\n");
        return false;
    }
    if (!LittleFS.exists(path)) {
        LOG("[AUD] 加载跳过：文件不存在 %s\n", path);
        return false;
    }

    File f = LittleFS.open(path, FILE_READ);
    if (!f) {
        LOG("[AUD] 加载失败：无法打开 %s\n", path);
        return false;
    }

    size_t totalSize = f.size();
    if (totalSize == 0) {
        f.close();
        LOG("[AUD] 预加载失败：文件为空\n");
        return false;
    }
    if (totalSize > AUDIO_POOL_SIZE) {
        f.close();
        LOG("[AUD] 预加载失败：文件 %uKB 超过池上限 %uKB\n",
            (unsigned)(totalSize / 1024), (unsigned)(AUDIO_POOL_SIZE / 1024));
        return false;
    }

    uint32_t t0 = millis();
    size_t bytesRead = 0;
    while (bytesRead < totalSize) {
        size_t chunk = totalSize - bytesRead;
        if (chunk > 8192) chunk = 8192;
        size_t r = f.read(s_audioPool + bytesRead, chunk);
        if (r == 0) break;
        bytesRead += r;
        vTaskDelay(1);                // 让出，避免长读饿死同核任务
    }
    f.close();

    if (bytesRead != totalSize) {
        LOG("[AUD] 预加载读取不完整: %u/%u\n",
            (unsigned)bytesRead, (unsigned)totalSize);
        return false;
    }

    s_fileSize = totalSize;
    LOG("[AUD] ✓ 音频已缓存到 PSRAM: %s (%uKB, %lums)\n",
        path, (unsigned)(totalSize / 1024), (unsigned long)(millis() - t0));
    return true;
}

// 预加载默认音频（/output1.mp3）——薄封装
static bool audPreloadFile() { return audLoadFileToPool(AUDIO_FILE_PATH); }

// ── P4：播放指定文件（伙伴语音即播即删）控制标志 ──
static volatile bool s_playFileReq     = false;   // 请求播放 s_playFilePath
static char          s_playFilePath[64] = {0};
static bool          s_playFileDelete  = false;   // 加载进池后删源文件
static bool          s_restoreAfterPlay = false;  // 播完后恢复 /output1.mp3

// ----------------------------------------------------------
// 停止播放：进入空闲最省态（删 I2S + 拉低引脚 + 静音）
//   池常驻不释放；s_fileSize 保留，下次可直接重播
// ----------------------------------------------------------
static void stopPlayback() {
    LOG("[AUD] 停止播放\n");
    g_isPlaying = false;
    s_filePos   = 0;
    audIdleShutdown();          // ★ 每次播放完/停止 → 最省态
}

// 输出一帧到 I2S：应用增益+软限幅；★单声道复制成左右两路（始终立体声输出）。
// 录音是 mono，MAX98357A 用 I2S 单声道 slot 常出杂音/乱码——这就是"设备播放杂乱、
// 但同一文件在手机上很干净"的根因。展开：[m0,m1,...] → [m0,m0,m1,m1,...]。
static void audWriteFrame(int samples, int channels) {
    const float gain = (s_volume / 255.0f) * AUD_GAIN_MAX;
    int outSamples;
    if (channels == 1) {
        // 反向原地展开成立体声（s_pcmBuf 容量=最大立体声帧，恰好放得下 2N）
        for (int i = samples - 1; i >= 0; i--) {
            int16_t s = audSoftLimit(s_pcmBuf[i] * gain);
            s_pcmBuf[2 * i]     = s;
            s_pcmBuf[2 * i + 1] = s;
        }
        outSamples = samples * 2;
    } else {
        int n = samples * channels;
        for (int i = 0; i < n; i++) s_pcmBuf[i] = audSoftLimit(s_pcmBuf[i] * gain);
        outSamples = n;
    }
    size_t written = 0;
    i2s_channel_write(s_i2sTxHandle, s_pcmBuf, outSamples * sizeof(int16_t),
                      &written, pdMS_TO_TICKS(200));
}

// ----------------------------------------------------------
// 启动播放：重建 I2S 总线 + 解除静音 + 用常驻池零 Flash 访问
// ----------------------------------------------------------
static AudError_t startPlayback() {
    if (g_isPlaying) stopPlayback();

    // 池里没有有效数据 → 兜底即时加载一次（首启动还没下载过等场景）
    if (s_fileSize == 0) {
        if (!audPreloadFile()) {
            LOG("[AUD] 池为空且即时加载失败\n");
            return AUD_ERR_FILE_OPEN;
        }
    }

    s_filePos = 0;

    // ── ① 解码第一帧，获取采样率/声道数（纯 CPU，不需要 I2S）──
    mp3dec_init(&s_dec);

    mp3dec_frame_info_t info;
    int samples = mp3dec_decode_frame(
        &s_dec, s_audioPool + s_filePos,
        (int)(s_fileSize - s_filePos), s_pcmBuf, &info);

    if (samples <= 0 || info.hz == 0) {
        LOG("[AUD] 第一帧解码失败\n");
        return AUD_ERR_MP3_BEGIN;
    }

    // ── ② 重建 I2S 总线（接管 BCK/LRCK/DIN，覆盖空闲时的拉低态）──
    if (!initI2S(info.hz, info.channels)) {
        return AUD_ERR_MP3_BEGIN;
    }

    // ── ③ 打开声音：使能功放（MAX98357A 单脚 SD_MODE#）──
    digitalWrite(AUD_SHDN, HIGH);
    delay(40);                          // 功放上电稳定

    // ── ④ 输出第一帧（增益+软限幅+mono→stereo，见 audWriteFrame）──
    audWriteFrame(samples, info.channels);

    s_filePos  += info.frame_bytes;
    g_isPlaying = true;

    LOG("[AUD] 开始播放(池内): rate=%d ch=%d  缓存=%uKB\n",
        info.hz, info.channels, (unsigned)(s_fileSize / 1024));
    return AUD_OK;
}

// ----------------------------------------------------------
// 解码一帧（纯 PSRAM 操作，不碰 Flash）
// ----------------------------------------------------------
static bool decodeAndOutputFrame() {
    if (s_filePos >= s_fileSize) return false;

    mp3dec_frame_info_t info;
    int samples = mp3dec_decode_frame(
        &s_dec, s_audioPool + s_filePos,
        (int)(s_fileSize - s_filePos), s_pcmBuf, &info);

    if (info.frame_bytes > 0) {
        s_filePos += info.frame_bytes;
    } else if (samples <= 0) {
        s_filePos++;   // 跳过无效字节
    }

    if (samples > 0) {
        audWriteFrame(samples, info.channels);   // 增益+软限幅+mono→stereo
    }

    return (s_filePos < s_fileSize);
}

// ----------------------------------------------------------
// 硬件挂起 / 恢复（与 BLE 配网互斥用）
//   挂起：进入空闲最省态（删 I2S，归还 DMA 占用的内部 SRAM + 拉低引脚）
//   恢复：重建 I2S
// ----------------------------------------------------------
static void audDeinitHW() {
    audIdleShutdown();
    LOG("[AUD] I2S 已释放（SRAM 归还）\n");
}

static bool audReinitHW() {
    bool ok = initI2S(I2S_DEFAULT_RATE, 2);   // 重建总线，接管引脚
    LOG("[AUD] I2S 已重建: %s\n", ok ? "OK" : "FAIL");
    return ok;
}

// ----------------------------------------------------------
// 初始化（GPIO + I2S 探测 + PCM 缓冲 + 600KB 池 + 预加载）
// ----------------------------------------------------------
AudError_t audInit() {
    LOG("[AUD] 功放初始化\n");
    s_volume = Config.getInt("volume", 200);   // 0~255（越大越响，峰值软限幅不破音）

    pinMode(AUD_SHDN, OUTPUT);
    digitalWrite(AUD_SHDN, LOW);
    delay(200);
    digitalWrite(AUD_SHDN, HIGH);
    delay(200);

    // I2S 探测（确认硬件可用），随后立即进入空闲最省态
    if (!initI2S(I2S_DEFAULT_RATE, 2)) {
        LOG("[AUD] I2S 初始化失败！\n");
        return AUD_ERR_I2S_INIT;
    }
    // ★ 开机空闲：删 channel + 拉低 BCK/LRCK/DIN + 静音（4.8mA）
    audIdleShutdown();

    // PCM 输出缓冲（常驻 PSRAM）
    s_pcmBuf = (int16_t*)heap_caps_malloc(
        PCM_BUF_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (!s_pcmBuf) {
        LOG("[AUD] PCM 缓冲分配失败！\n");
        return AUD_ERR_I2S_INIT;
    }

    // ★ 600KB 音频文件常驻池（PSRAM，一次性分配）
    s_audioPool = (uint8_t*)heap_caps_malloc(AUDIO_POOL_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_audioPool) {
        LOG("[AUD] 音频池分配失败！需要 %uKB，剩余 PSRAM %uKB\n",
            (unsigned)(AUDIO_POOL_SIZE / 1024),
            (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        return AUD_ERR_I2S_INIT;
    }
    LOG("[AUD] ✓ 音频池: %uKB @ %p\n", (unsigned)(AUDIO_POOL_SIZE / 1024), s_audioPool);

    // 开机预加载一次（文件不存在则跳过，等下载后由 audNotifyFileUpdated 触发）
    audPreloadFile();

    LOG("[AUD] 初始化完成，堆=%u KB\n", ESP.getFreeHeap() / 1024);
    return AUD_OK;
}

// ----------------------------------------------------------
// audTask
// ----------------------------------------------------------
void audTask(void* param) {
    LOG("[AUD] Task 启动，prio=%d\n", (int)uxTaskPriorityGet(nullptr));

    bool           newPlayReq = false;
    MessageToAud_t rxMsg;

    while (true) {
        // ── 队列指令 ──
        if (xQueueReceive(qMainToAud, &rxMsg, 0) == pdTRUE) {
            switch (rxMsg.cmd) {
                case AUD_PLAY:
                    LOG("[AUD] 收到 AUD_PLAY\n");
                    newPlayReq = true;
                    break;
                case AUD_CFG_UPDATE:
                    s_volume = Config.getInt("volume", 200);
                    LOG("[AUD] 音量更新: %d\n", s_volume);
                    break;
                case AUD_STOP:
                    stopPlayback();
                    LOG("[AUD] 收到 AUD_STOP\n");
                    break;
                default:
                    break;
            }
        }

        // ── 挂起请求（配网前，释放 I2S 的 SRAM）──
        if (s_suspendReq) {
            s_suspendReq = false;
            stopPlayback();
            audDeinitHW();
            g_audSuspended = true;
            LOG("[AUD] ★ 已挂起（等待 BLE 配网结束）\n");
        }

        // ── 恢复请求（配网后）──
        if (s_resumeReq) {
            s_resumeReq = false;
            audReinitHW();
            g_audSuspended = false;
            LOG("[AUD] ★ 已恢复\n");
        }

        // 挂起期间：不解码、不碰 I2S，只低频自旋
        if (g_audSuspended) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // ── 文件更新重缓存（仅在不播放时执行，防止 use-after-free）──
        if (s_reloadReq && !g_isPlaying) {
            s_reloadReq = false;
            LOG("[AUD] 文件已更新，重新缓存...\n");
            audPreloadFile();
        }

        // ── P4：播放指定文件（伙伴语音即播即删）──
        if (s_playFileReq && !g_audSuspended) {
            s_playFileReq = false;
            stopPlayback();
            if (audLoadFileToPool(s_playFilePath)) {
                if (s_playFileDelete) LittleFS.remove(s_playFilePath);
                s_restoreAfterPlay = true;
                newPlayReq = true;              // 触发下面 startPlayback 播池内新内容
            } else {
                LOG("[AUD] 播放指定文件失败: %s\n", s_playFilePath);
                s_reloadReq = true;             // 加载失败 → 恢复原音频
            }
        }

        // ── 启动播放（内部 initI2S 重建总线 + 解除静音）──
        if (newPlayReq) {
            newPlayReq = false;
            AudError_t err = startPlayback();
            if (err != AUD_OK) {
                LOG("[AUD] 播放失败 err=%d\n", (int)err);
                stopPlayback();                 // 失败也回到最省态
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }

        // ── 逐帧解码输出 ──
        if (g_isPlaying) {
            if (!decodeAndOutputFrame()) {
                LOG("[AUD] 播放结束\n");
                stopPlayback();                 // ★ 播放完毕 → 删 I2S + 静音（最省态）
                if (s_restoreAfterPlay) {       // P4：伙伴语音播完 → 恢复 /output1.mp3 到池
                    s_restoreAfterPlay = false;
                    s_reloadReq = true;
                    LOG("[AUD] 伙伴语音播完，恢复原音频\n");
                }
            }
        } else {
            // ── 空闲心跳：I2S 已在 stopPlayback 中彻底释放（4.8mA）──
            //   L0=20ms(响应快) / L1=100ms / L2=阻塞等唤醒（喂 light-sleep）
            //   L2 由 powerManagerLoop 离开 L2 时 give xAudWake 瞬时唤醒。
            if (g_pwrTier >= 2) {
                xSemaphoreTake(xAudWake, pdMS_TO_TICKS(AUD_L2_BLOCK_MS));
            } else {
                vTaskDelay(pdMS_TO_TICKS(g_pwrTier == 1 ? 100 : 20));
            }
        }
    }
}

// ══════════════════════════════════════════════════════════
//  对外接口（需在 aud.hpp 声明）
// ══════════════════════════════════════════════════════════

void audSetVolume(uint8_t vol) {
    s_volume = vol;   // 0~255（uint8 天然封顶；增益+软限幅在播放时处理）
}

// 文件下载更新后调用：请求 audTask 重新缓存
void audNotifyFileUpdated() {
    s_reloadReq = true;
}

// P4：播放指定 mp3 文件（伙伴语音即播即删）。由 webTask 收齐伙伴语音后调用；
// audTask 会停当前播放→加载进池→(可选删源)→播放→播完恢复 /output1.mp3。
void audPlayFile(const char* path, bool deleteAfterLoad) {
    strncpy(s_playFilePath, path, sizeof(s_playFilePath) - 1);
    s_playFilePath[sizeof(s_playFilePath) - 1] = 0;
    s_playFileDelete = deleteAfterLoad;
    s_playFileReq    = true;
}

// 配网前调用：请求挂起音频，释放 I2S 占用的内部 SRAM
void audSuspend() {
    s_suspendReq = true;
}

// 配网后调用：请求恢复音频
void audResume() {
    s_resumeReq = true;
}

// 供 webTask 轮询：音频是否已完成挂起
bool audIsSuspended() {
    return g_audSuspended;
}

// 是否正在播放音频，供主循环把"播放中"持续计为活动
bool audIsPlaying() {
    return g_isPlaying;
}

// ══════════════════════════════════════════════════════════════
//  MQTT 直收进 PSRAM（接收期不碰 flash）
//    web 把音频分片直接 memcpy 进 s_audioPool；
//    END 时 commit → 池内即新音频，可直接播；
//    失败 abort → audTask 空闲时从 flash 重载旧文件。
//    传输期音频已 AUD_STOP（未播放），故 web 直写池是安全的。
// ══════════════════════════════════════════════════════════════

uint8_t* audRecvBegin(size_t* maxBytes) {
    s_fileSize = 0;                 // 标记池内暂无有效数据
    if (maxBytes) *maxBytes = AUDIO_POOL_SIZE;
    LOG("[AUD] 直收准备：基址 %p 上限 %uKB\n", s_audioPool, (unsigned)(AUDIO_POOL_SIZE / 1024));
    return s_audioPool;
}

bool audRecvCommit(size_t size) {
    if (size == 0 || size > AUDIO_POOL_SIZE) {
        s_reloadReq = true;         // 异常 → 从 flash 重载旧文件
        return false;
    }
    s_fileSize = size;              // 池内即新音频，下次 AUD_PLAY 直接播
    LOG("[AUD] 直收提交：%uKB 已在 PSRAM\n", (unsigned)(size / 1024));
    return true;
}

void audRecvAbort() {
    s_reloadReq = true;             // audTask 空闲时从 flash 重载旧文件
}