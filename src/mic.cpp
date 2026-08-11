#include <Arduino.h>
#include <LittleFS.h>
#include <SPI.h>
#include "driver/i2s_pdm.h"

#include "inc/mic.hpp"
#include "inc/msg.hpp"
#include "inc/mp3enc.hpp"   // Shine MP3 编码器封装（隔离其全局枚举，避免与 msg.hpp 冲突）
#include "inc/debug.hpp"
#include "esp_task_wdt.h"   // 任务级看门狗：卡死时 panic backtrace 直接点名本任务
#include "inc/health.hpp"   // 运行期阶段打点（TWDT 超时时会被冻成快照）

// 16kHz 单声道语音，64kbps 音质更干净；30s ≈ 240KB（10 条 2.4MB 够存）
// ⚠ 试过 128kbps：录音完全没声音（实测）。原因见下方 shine 缓冲区分析——
//   在改码率前必须先确认 shine 内部帧缓冲放得下该码率的一帧，否则会静默出错。
#define MP3_BITRATE_KBPS   64

// 最短有效录音时长（ms）：短于此（或编码为空）视为误触/碎录音 → 丢弃，不占槽、不推送。
// 从源头挡住"快速连按侧键"产生的碎语音刷屏（推送轰炸 + 伙伴蛋碎播 + 占槽）。
#define MIN_RECORD_MS      1000

// 录音临时文件：先录到这里，录完【有效】才提交(rename)到正式语音槽 voice_N。
// 好处：无效录音不占槽/不淘汰；满槽时也只有"确认要保存"才去环形淘汰最旧一条。
#define VOICE_TMP_PATH     "/voice_rec.tmp"

// 麦克风软件数字增益：S3 的 PDM 驱动没有硬件 amplify_num，靠软件乘倍数提升
// 录音电平（加在录音上，发给 App 的 MP3 也是正常音量）。带限幅防溢出。
// 太小=录音轻，太大=大声说话会过压变浑浊（有软限幅兜底不硬破，但过高会闷/杂），实测调（8~24）。
// 换新麦克风(SNR 更好、底噪低)后增益空间更大，提到 22 让录音电平达到正常音频水平。
#define MIC_GAIN           20

// ── 语音带通软件滤波 + 噪声门（S3 PDM 无硬件高通，靠软件去底噪）──
//   MIC_HP_R：一阶高通，去直流/低频隆隆声，~120Hz。越接近 1 截止越低（去得越少）。
//   MIC_LP_A：一阶低通，压高频嘶声，~3kHz。越小滤得越狠（也越闷）。设 1.0 = 不低通。
//   软噪声门（下行扩展器）：信号包络低于 MIC_GATE_TH 时，不是直接静音，而是
//     衰减到 MIC_GATE_FLOOR（下限），这样小声说话短暂掉到门限下也只是变轻、
//     不会被整段切掉。
//     MIC_GATE_TH   ：门限（增益后幅度，0~32767）。调大=压得更狠但易吃掉小声语音。
//                     设 0 = 关闭噪声门。
//     MIC_GATE_FLOOR：门关时的衰减下限（0=全静音，0.3=降到30%，1=不压）。
//                     调小=静音更干净但小声语音也更容易被压轻。
//     MIC_ENV_A     ：包络跟踪速度（越大越快跟上）。
//     MIC_GATE_SMOOTH：门开关的平滑（越大越干脆，太大会有咔哒声）。
// 经 micTest 实测调好：150Hz 高通去低频隆隆 + 软噪声门清静音段。
#define MIC_HP_R           0.94f     // ~150Hz 高通（去 47Hz 隆隆等 <200Hz 低频噪声）
#define MIC_LP_A           1.00f     // 1.0 = 不低通（残留是宽带白噪，低通没用还损音质）
#define MIC_GATE_TH        600.0f    // 噪声门门限（增益后幅度）；0=关。按设备实测可调
// ★ FLOOR 0.25→0.08：0.25 只把静音段压 12dB，底噪照样透出来——伙伴蛋的小喇叭在
//   2~6kHz 有谐振峰，正好放大这段嘶声（手机喇叭频响宽 + 有 DSP，所以同一文件在
//   App 里听不出来）。0.08 ≈ -22dB，静音段基本听不到，仍不是全静音、不会把
//   小声说话整段切掉。
#define MIC_GATE_FLOOR     0.30f     // 门关时降到 8%（≈-22dB）
#define MIC_ENV_A          0.02f
// ★ SMOOTH 0.03→0.008：门压得更深后，开关速度必须放慢，否则语气停顿处会听到
//   底噪一下一下地"抽气"（pumping）。0.008 @16kHz ≈ 8ms 时间常数，跟得上语句
//   间隙又不会在音节间抖动。
#define MIC_GATE_SMOOTH    0.008f
#define MIC_LIMIT_KNEE     0.80f     // 软限幅拐点：低于此透明，高于此平滑压峰不硬削波（增益提高后抬高拐点，正常说话不被压闷）

struct MicFilt { float hpX1, hpY1, lpS, env, gate; };

// 软限幅：低于拐点透明直通，高于拐点 tanh 平滑压向满幅（绝不硬削波=不破音）
static inline float micSoftLimit(float v) {
    const float FS   = 32767.0f;
    const float knee = MIC_LIMIT_KNEE * FS;
    float a = v < 0 ? -v : v;
    if (a <= knee) return v;
    float over = (a - knee) / (FS - knee);
    float comp = knee + (FS - knee) * tanhf(over);
    return v < 0 ? -comp : comp;
}

// ★调试开关：1=录音原始直通（不做高通/低通/噪声门/软限幅，只保留增益 + 硬限幅防 int16 回绕），
//            0=正常滤波链。用于对比"滤波 vs 不滤波"的录音音质。测完想恢复就改回 0。
#define MIC_RAW_PASSTHROUGH  0

// 单个采样：高通 → 低通 → 增益 → 噪声门 → 软限幅。状态由调用方持有（每次录音重置）。
static inline int16_t micProcess(int16_t raw, MicFilt* s) {
#if MIC_RAW_PASSTHROUGH
    (void)s;                                          // 直通不用滤波状态
    float v = (float)raw * MIC_GAIN;                  // 只加增益（增益非滤波）；想更"裸"可把 MIC_GAIN 调到 1
    if (v >  32767.0f) v =  32767.0f;                 // 硬限幅仅防 int16 回绕（不是压缩，也不是滤波）
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
#else
    float xf = (float)raw;
    float hp = xf - s->hpX1 + MIC_HP_R * s->hpY1;   // 一阶高通
    s->hpX1 = xf;
    s->hpY1 = hp;
    s->lpS += MIC_LP_A * (hp - s->lpS);             // 一阶低通
    float v = s->lpS * MIC_GAIN;                    // 增益

    if (MIC_GATE_TH > 0.0f) {                        // 软噪声门（下行扩展）
        float mag = v < 0 ? -v : v;
        s->env += MIC_ENV_A * (mag - s->env);        // 包络跟踪
        float target = (s->env > MIC_GATE_TH) ? 1.0f : MIC_GATE_FLOOR;
        s->gate += MIC_GATE_SMOOTH * (target - s->gate);
        v *= s->gate;
    }

    return (int16_t)micSoftLimit(v);                 // 软限幅（响而不破）
#endif
}

extern QueueHandle_t qMainToMic; // message queue |  main -> microphone
extern QueueHandle_t qMicToMain; // message queue |  microphone -> main
extern QueueHandle_t qMainToWeb; // 录完发 NEW_VOICE 给 webTask（触发推送）

// web.cpp：该槽是否正被【任一通路】(App / 伙伴)发送。环形淘汰要跳过它，避免删到正被读的文件。
// ★ 取代原来的 g_voiceSendingId 单值判断——那个只保护 App 通路，伙伴通路的槽会被误淘汰。
extern bool voiceSlotBusy(int slot);
// web.cpp：录音提交后登记到出站账本（分配 uid、两个目的地各置为待投递）。
extern void voiceEnqueue(int slot);

// 录音音量表（lcd.cpp 定义并绘制）：录音时置位并写实时电平，LCD 据此画音量动画
extern volatile bool g_micMeterActive;
extern volatile int  g_micLevel;

i2s_chan_handle_t rx_handle = NULL;

static volatile bool s_recording = false;

bool micIsRecording() { return s_recording; }

// 找一个空的语音槽（voice_0..voice_{VOICE_MAX-1} 里第一个不存在的）；满了返回 -1
static int findFreeVoiceSlot() {
    char path[24];
    for (int i = 0; i < VOICE_MAX; i++) {
        snprintf(path, sizeof(path), VOICE_PATH_FMT, i);
        if (!LittleFS.exists(path)) return i;
    }
    return -1;
}

// 满槽时挑"最旧"的一条淘汰（环形队列：丢队头、腾位给新录音，保留最近 VOICE_MAX 条）。
// 判据：getLastWrite() 最小者最旧；时间未同步/相等时天然退化为最小编号（先扫到先选中）。
// ★ 跳过当前正被【任一通路】发送的槽（voiceSlotBusy），避免删掉 webTask 正在读的文件。
// 返回 -1 = 没有可淘汰的槽（极端：全部都在发送中，几乎不可能，因为一次只发一条）。
static int pickOldestVoiceSlot() {
    int    bestSlot = -1;
    time_t bestTime = 0;
    char   path[24];
    for (int i = 0; i < VOICE_MAX; i++) {
        if (voiceSlotBusy(i)) continue;        // 别删任一通路正在发送的那条
        snprintf(path, sizeof(path), VOICE_PATH_FMT, i);
        File f = LittleFS.open(path, FILE_READ);
        if (!f) continue;
        time_t t = f.getLastWrite();
        f.close();
        if (bestSlot < 0 || t < bestTime) { bestSlot = i; bestTime = t; }
    }
    return bestSlot;
}

// 通知 webTask 发 NEW_VOICE 推送
static void notifyNewVoice() {
    char m[WEB_MSG_LEN];
    strncpy(m, "NEW_VOICE", sizeof(m) - 1);
    m[sizeof(m) - 1] = '\0';
    xQueueSend(qMainToWeb, &m, 0);
    LOG("[MIC] 已通知 web 发 NEW_VOICE 推送\n");
}

// 初始化麦克风
bool initMIC() {
    // 创建 RX channel（固定用 I2S0：ESP32-S3 的 PDM 模式只有 I2S0 支持 IDF
    // 硬性限制，见 i2s_pdm.c 里 "PDM is only supported on I2S0"；因此播放那边
    // 的 TX 通道改用 I2S1，见 aud.cpp）
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_frame_num = DMA_BUF_LEN;
    chan_cfg.dma_desc_num  = DMA_BUF_COUNT;

    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (err != ESP_OK) {
        LOG("[MIC ERROR] i2s_new_channel: %s\n", esp_err_to_name(err));
        return false;
    }

    // ★ PDM 时钟：DSR_16S 把时钟从 ~1.024MHz 提到 ~2.048MHz，接近本 mic
    //   (LMD2718T261) 手册典型 2.4MHz，信噪比明显更好（实测底噪降 ~14dB）。
    i2s_pdm_rx_clk_config_t clkc = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE);
    clkc.dn_sample_mode = I2S_PDM_DSR_16S;

    // 配置 PDM RX
    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg  = clkc,
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                    I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk  = (gpio_num_t)PDM_CLK_PIN,
            .din  = (gpio_num_t)PDM_DATA_PIN,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    err = i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_cfg);
    if (err != ESP_OK) {
        LOG("[MIC ERROR] i2s_channel_init_pdm_rx_mode: %s\n", esp_err_to_name(err));
        return false;
    }

    // ★ 省电：初始化后不 enable，让 PDM 时钟保持关闭；只在录音时 enable、录完 disable。
    LOG("[MIC] 初始化完成（待机，录音时才启用）\n");
    return true;
}

// 录制：Button1 单击开始，再单击（收到 MIC_STOP）或满 RECORD_MAX_MS 结束。
// 先边录边 Shine 编码写到【临时文件】；录完再判定：
//   · 空 / 时长 < MIN_RECORD_MS → 丢弃（不占槽、不淘汰、不推送，挡碎语音刷屏）；
//   · 有效 → 分配正式槽：有空位用空位，满 VOICE_MAX 条则环形淘汰最旧一条，
//            把临时文件 rename 过去，再发 NEW_VOICE。
void recordAndSave() {
    // ── ① 初始化 Shine 编码器（mono / SAMPLE_RATE / MP3_BITRATE_KBPS）+ 打开临时文件 ──
    Mp3Enc* enc = mp3encOpen(SAMPLE_RATE, MP3_BITRATE_KBPS);
    if (!enc) {
        LOG("[MIC ERROR] MP3 编码器初始化失败（参数非法或内存不足）\n");
        return;
    }
    const int samplesPerPass = mp3encSamplesPerPass(enc);   // 每次编码需要的采样数

    File f = LittleFS.open(VOICE_TMP_PATH, FILE_WRITE);
    if (!f) { LOG("[MIC ERROR] 无法创建临时文件 %s\n", VOICE_TMP_PATH); mp3encClose(enc); return; }

    // ── ② 启用麦克风（平时禁用省电，录音才开）──
    if (i2s_channel_enable(rx_handle) != ESP_OK) {
        LOG("[MIC ERROR] 启用麦克风失败\n");
        f.close(); LittleFS.remove(VOICE_TMP_PATH); mp3encClose(enc); return;
    }

    static int16_t rdBuf[DMA_BUF_LEN];            // I2S 读缓冲
    static int16_t frame[MP3ENC_MAX_SAMPLES];     // 攒够一帧再编码
    int      frameFill = 0;
    uint32_t rawSamples = 0, mp3Bytes = 0;
    size_t   bytesRead = 0;
    uint32_t startMs = millis(), lastLog = 0;
    MicFilt  mf = {0, 0, 0, 0, 0};          // 滤波 + 噪声门状态

    s_recording = true;
    g_micMeterActive = true;   // ★ 通知 LCD 切到录音音量表（全屏）
    g_micLevel = 0;
    LOG("[MIC] 开始录音 -> %s（MP3 %dkbps，最长 %us，再单击 Button1 停止）\n",
        VOICE_TMP_PATH, MP3_BITRATE_KBPS, RECORD_MAX_MS / 1000);

    bool stop = false;
    while (!stop) {
        esp_task_wdt_reset();   // 录音最长连续 30s，须在此喂狗（主循环的 reset 期间不触发）
        healthSetTaskStage(HT_MIC, HS_M_RECORD);   // 采集 + 编码 + 写文件，都在这一轮里
        if (millis() - startMs >= RECORD_MAX_MS) {
            LOG("[MIC] 达到最长时长 %us，自动停止\n", RECORD_MAX_MS / 1000);
            break;
        }
        // 非阻塞检查停止指令：只认 MIC_STOP，其余丢弃
        MessageToMic_t msg;
        if (xQueueReceive(qMainToMic, &msg, 0) == pdTRUE && msg.cmd == MIC_STOP) {
            LOG("[MIC] 收到停止指令\n");
            break;
        }

        esp_err_t err = i2s_channel_read(rx_handle, rdBuf, sizeof(rdBuf),
                                         &bytesRead, pdMS_TO_TICKS(200));
        if (err == ESP_ERR_TIMEOUT) continue;
        if (err != ESP_OK || bytesRead == 0) break;

        int n = bytesRead / 2;
        rawSamples += n;

        // 实时音量电平（给 LCD 音量表）：raw RMS → dBFS → noise gate → 平滑 → 0..100
        {
            double sq = 0;
            for (int i = 0; i < n; i++) { float v = rdBuf[i]; sq += v * v; }
            float rms = n > 0 ? sqrtf((float)(sq / n)) : 0.0f;
            float db  = 20.0f * log10f((rms + 1.0f) / 32768.0f);   // -90..0 dBFS
            float lv  = (db + 50.0f) * (100.0f / 44.0f);           // -50dB→0，-6dB→100（noise gate 在 -50dB）
            if (lv < 0) lv = 0; if (lv > 100) lv = 100;
            static float sm = 0;
            sm += (lv - sm) * (lv > sm ? 0.55f : 0.15f);           // attack 快 / release 慢
            g_micLevel = (int)(sm + 0.5f);
        }

        for (int i = 0; i < n; i++) {
            frame[frameFill++] = micProcess(rdBuf[i], &mf);
            if (frameFill == samplesPerPass) {
                int written = 0;
                const uint8_t* mp3 = mp3encFrame(enc, frame, &written);
                if (written > 0) { f.write(mp3, written); mp3Bytes += written; }
                frameFill = 0;
            }
        }

        if (millis() - lastLog >= 1000) {
            lastLog = millis();
            LOG("[MIC] 录制中... %us\n", (millis() - startMs) / 1000);
        }
    }

    uint32_t recMs = millis() - startMs;   // 实际录音时长（最短时长判定用；编码收尾很快，忽略）

    // ── ③ 关闭麦克风省电（录音已结束，后面只是编码收尾）──
    i2s_channel_disable(rx_handle);

    // ── ④ 收尾：最后不满一帧用静音补齐后编码，再 flush 冲刷缓冲 ──
    if (frameFill > 0) {
        for (int i = frameFill; i < samplesPerPass; i++) frame[i] = 0;
        int written = 0;
        const uint8_t* mp3 = mp3encFrame(enc, frame, &written);
        if (written > 0) { f.write(mp3, written); mp3Bytes += written; }
    }
    {
        int written = 0;
        const uint8_t* mp3 = mp3encFlush(enc, &written);
        if (written > 0) { f.write(mp3, written); mp3Bytes += written; }
    }
    mp3encClose(enc);
    f.close();
    s_recording = false;
    g_micMeterActive = false;   // ★ 录音结束：LCD 恢复 free 待机动画

    // ── ⑤ 有效性判定：空 或 太短(<门槛) → 丢弃临时文件，不占槽、不淘汰、不推送 ──
    //   放在提交之前：无效录音绝不会误删已有的旧语音（避免"误触一下把一条旧消息挤掉却没录进东西"）。
    if (mp3Bytes == 0 || recMs < MIN_RECORD_MS) {
        LOG("[MIC] ✗ 录音无效（时长 %ums < %dms 或空 %uB），丢弃不推送\n",
            (unsigned)recMs, MIN_RECORD_MS, (unsigned)mp3Bytes);
        LittleFS.remove(VOICE_TMP_PATH);
        return;
    }

    // ── ⑥ 分配正式槽：有空位用空位；满 VOICE_MAX 条则环形淘汰最旧一条（保留最近 N 条）──
    int slot = findFreeVoiceSlot();
    if (slot < 0) {
        slot = pickOldestVoiceSlot();
        if (slot < 0) {   // 极端：所有槽都在发送中（一次只发一条，几乎不可能）
            LOG("[MIC] ✗ 所有语音槽都在发送中，暂丢弃本条\n");
            LittleFS.remove(VOICE_TMP_PATH);
            return;
        }
        char oldPath[24];
        snprintf(oldPath, sizeof(oldPath), VOICE_PATH_FMT, slot);
        LittleFS.remove(oldPath);
        LOG("[MIC] 语音已满 %d 条，环形淘汰最旧槽 %d 腾位\n", VOICE_MAX, slot);
    }
    char voicePath[24];
    snprintf(voicePath, sizeof(voicePath), VOICE_PATH_FMT, slot);

    // ── ⑦ 提交：临时文件 rename 到正式槽 + 清该槽旧 uid（复用槽时防 App 按 uid 误去重）──
    if (LittleFS.exists(voicePath)) LittleFS.remove(voicePath);
    if (!LittleFS.rename(VOICE_TMP_PATH, voicePath)) {
        LOG("[MIC ERROR] 提交失败（rename %s -> %s）\n", VOICE_TMP_PATH, voicePath);
        LittleFS.remove(VOICE_TMP_PATH);
        return;
    }
    voiceEnqueue(slot);   // 登记出站账本：分配 uid、App/伙伴两路各置为待投递

    LOG("[MIC] ✓ 保存完成: %s  原始 %u 采样 -> MP3 %u 字节 时长 %ums（槽 %d）\n",
        voicePath, (unsigned)rawSamples, (unsigned)mp3Bytes, (unsigned)recMs, slot);

    notifyNewVoice();                 // 通知 web 发 NEW_VOICE 推送（App 会来拉取）
}

void micTask(void *micParameter)
{
    LOG("[MIC] MIC Task start\n");
    // 清理上次录音中途崩溃/掉电残留的临时文件（否则占 flash；正常录音会覆盖它，但崩溃时不会）
    if (LittleFS.exists(VOICE_TMP_PATH)) LittleFS.remove(VOICE_TMP_PATH);
    esp_task_wdt_add(NULL);   // 纳入 TWDT（录音循环内另有 reset，见 recordAndSave）
    MessageToMic_t  rxMsg;

    while (true) {
        esp_task_wdt_reset();   // 空闲每 50ms 喂狗
        healthSetTaskStage(HT_MIC, HS_M_IDLE);
        if (xQueueReceive(qMainToMic, &rxMsg, 0) == pdTRUE) {
            LOG("[MIC] 收到指令，CMD=%d\n", rxMsg.cmd);
            if (rxMsg.cmd == MIC_START && !s_recording) {
                recordAndSave();
            }
            // 空闲时收到 MIC_STOP：忽略（按钮逻辑按 micIsRecording() 判断，正常不会发生）
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
