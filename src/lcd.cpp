#include "inc/msg.hpp"
#include "inc/fs.hpp"
#include "inc/lcd.hpp"
#include "inc/configSys.hpp"
#include "inc/health.hpp"   // 启动阶段打点：把预加载细分到"哪个视频文件"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"   // 任务级看门狗：卡死时 panic backtrace 直接点名本任务

// ══════════════════════════════════════════════════════════════
//  LovyanGFX 硬件配置
// ══════════════════════════════════════════════════════════════

class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ST7789  _panel_instance;
    lgfx::Bus_SPI       _bus_instance;
    lgfx::Light_PWM     _light_instance;
public:
    LGFX(void) {
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host    = SPI2_HOST;
            cfg.freq_write  = 80000000;
            cfg.freq_read   = 40000000;
            cfg.pin_sclk    = SPI_SCK;
            cfg.pin_mosi    = SPI_MOSI;
            cfg.pin_miso    = SPI_MISO;
            cfg.pin_dc      = LCD_DC;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs       = LCD_CS;
            cfg.pin_rst      = LCD_RST;
            cfg.panel_width  = LCD_WIDTH;
            cfg.panel_height = LCD_HEIGHT;
            cfg.invert       = true;
            cfg.bus_shared   = true;
            _panel_instance.config(cfg);
        }
        {
            auto cfg = _light_instance.config();
            cfg.pin_bl      = LCD_BL;
            cfg.invert      = false;
            cfg.freq        = 44100;
            cfg.pwm_channel = 7;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }
        setPanel(&_panel_instance);
    }
};

// ══════════════════════════════════════════════════════════════
//  全局对象
// ══════════════════════════════════════════════════════════════

static LGFX        tft;
static LGFX_Sprite frameCanvas(&tft);   // 全屏后台缓冲（PSRAM），用于合成电量叠加再整块推屏
static lcdState_t  lcdState = LCD_OFF;
static lcdStats_t  stats;

extern QueueHandle_t qMainToLcd;
extern QueueHandle_t qLcdToMain;

// 视频更新通知标志（web 任务 → LCD 任务）
static volatile bool videoUpdatePending = false;

// ══════════════════════════════════════════════════════════════
//  PSRAM 池 + 视频缓存
// ══════════════════════════════════════════════════════════════
//
//  内存布局：
//
//  ┌─── PSRAM 4MB 池 ──────────────────────────────────────┐
//  │ [lcdFree.mjpeg 数据] [tap.mjpeg] [shake.mjpeg] [target│.mjpeg] [空闲]
//  └───────────────────────────────────────────────────────────┘
//           ↑                  ↑
//      slot[0].data       slot[1].data
//
//  每个 slot 还有一个帧索引数组（也在 PSRAM）：
//    frameIndex[0] = { offset: 0,    size: 8432 }  ← 第1帧
//    frameIndex[1] = { offset: 8432, size: 9120 }  ← 第2帧
//    ...
//
//  播放时：tft.drawJpg(slot.data + frameIndex[i].offset,
//                       frameIndex[i].size, 0, 0)
//  → 零拷贝，指针直接指向 PSRAM，无任何中间缓冲。
//

typedef struct {
    uint8_t*      data;         // PSRAM 中的视频数据指针
    size_t        dataSize;     // 文件总字节数
    frameEntry_t* frameIndex;   // 帧索引数组（PSRAM）
    int           frameCount;   // 总帧数
    bool          loaded;       // 是否已加载
    char          path[256];    // 源文件路径
} videoCacheSlot_t;

static uint8_t*          psramPool     = nullptr;  // 4MB 池起始地址
static size_t            poolUsed      = 0;        // 池已用字节
static videoCacheSlot_t  cache[VIDEO_CACHE_SLOTS];
static playerInfo_t*     playerInfo    = nullptr;

// 当前正在播放的缓存槽 + 帧号
static int currentSlot  = -1;
static int currentFrame = 0;

// ── 流式回退用缓冲（PSRAM 装不下时）──
static uint8_t* streamFrameBuf = nullptr;
static uint8_t* streamReadBlk  = nullptr;
static File     streamFile;
static bool     isStreaming     = false;
static size_t   stBlockLen      = 0;
static size_t   stBlockPos      = 0;

// ── 统计辅助 ──
static uint32_t decodeAccum = 0;
static uint32_t decodeCount = 0;

// ══════════════════════════════════════════════════════════════
//  PSRAM 池分配器（简单 bump allocator）
// ══════════════════════════════════════════════════════════════

static uint8_t* poolAlloc(size_t bytes)
{
    // 4 字节对齐
    size_t aligned = (bytes + 3) & ~3;
    if (poolUsed + aligned > PSRAM_LCD_POOL_SIZE) return nullptr;
    uint8_t* ptr = psramPool + poolUsed;
    poolUsed += aligned;
    return ptr;
}

// 释放指定 slot 及其之后的所有内存（用于 target 重新加载）
static void poolFreeFrom(int slotIdx)
{
    if (slotIdx < 0 || slotIdx >= VIDEO_CACHE_SLOTS) return;

    // 计算该 slot 之前占用的总量
    size_t keepUsed = 0;
    for (int i = 0; i < slotIdx; i++) {
        if (cache[i].loaded) {
            keepUsed = (cache[i].data + cache[i].dataSize) - psramPool;
            // 加上帧索引
            size_t idxSize = cache[i].frameCount * sizeof(frameEntry_t);
            size_t idxEnd = ((uint8_t*)cache[i].frameIndex + idxSize) - psramPool;
            if (idxEnd > keepUsed) keepUsed = idxEnd;
        }
    }

    // 4 字节对齐
    keepUsed = (keepUsed + 3) & ~3;
    poolUsed = keepUsed;

    // 清除该 slot 及之后的所有 slot
    for (int i = slotIdx; i < VIDEO_CACHE_SLOTS; i++) {
        cache[i].loaded     = false;
        cache[i].data       = nullptr;
        cache[i].frameIndex = nullptr;
        cache[i].frameCount = 0;
        cache[i].dataSize   = 0;
    }
}

// ══════════════════════════════════════════════════════════════
//  在已位于 PSRAM 的数据上扫描帧边界、建索引、填充 slot
//  （flash 加载 与 MQTT 直收 两条路径共用此函数）
// ══════════════════════════════════════════════════════════════
static bool finalizeSlotFromData(int slotIdx, uint8_t* data, size_t totalRead,
                                 const char* path, uint32_t loadMs)
{
    if (slotIdx < 0 || slotIdx >= VIDEO_CACHE_SLOTS) return false;

    videoCacheSlot_t& slot = cache[slotIdx];
    slot.loaded = false;

    // 校验 JPEG 头
    if (totalRead < 4 || data[0] != 0xFF || data[1] != 0xD8) {
        LOG("[LCD] 缓存[%d] 非 MJPEG 格式\n", slotIdx);
        return false;
    }

    // ── 第一遍：计数帧数 ──
    int frameCount = 0;
    size_t pos = 0;
    while (pos + 1 < totalRead) {
        uint8_t* p = (uint8_t*)memchr(data + pos, 0xFF, totalRead - pos);
        if (!p || (size_t)(p - data) + 1 >= totalRead) break;
        size_t idx = p - data;
        if (data[idx + 1] == 0xD8) { frameCount++; pos = idx + 2; }
        else                       { pos = idx + 1; }
    }

    if (frameCount == 0) {
        LOG("[LCD] 缓存[%d] 未发现 JPEG 帧\n", slotIdx);
        return false;
    }
    if (frameCount > MAX_FRAMES_PER_VIDEO) {
        LOG("[LCD] 缓存[%d] 帧数超限: %d (上限 %d)\n",
            slotIdx, frameCount, MAX_FRAMES_PER_VIDEO);
        frameCount = MAX_FRAMES_PER_VIDEO;
    }

    // 分配帧索引数组
    frameEntry_t* idx = (frameEntry_t*)poolAlloc(frameCount * sizeof(frameEntry_t));
    if (!idx) {
        LOG("[LCD] 缓存[%d] 帧索引分配失败\n", slotIdx);
        return false;
    }

    // ── 第二遍：记录每帧 offset / size ──
    int fi = 0;
    pos = 0;
    while (pos + 1 < totalRead && fi < frameCount) {
        uint8_t* soi = nullptr;
        for (size_t s = pos; s + 1 < totalRead; s++) {
            if (data[s] == 0xFF && data[s + 1] == 0xD8) { soi = data + s; break; }
        }
        if (!soi) break;
        size_t frameStart = soi - data;

        size_t frameEnd = 0; bool found = false;
        for (size_t e = frameStart + 2; e + 1 < totalRead; e++) {
            if (data[e] == 0xFF && data[e + 1] == 0xD9) { frameEnd = e + 2; found = true; break; }
        }
        if (!found) break;   // 最后一帧不完整，跳过

        idx[fi].offset = (uint32_t)frameStart;
        idx[fi].size   = (uint32_t)(frameEnd - frameStart);
        fi++;
        pos = frameEnd;
    }
    frameCount = fi;

    slot.data       = data;
    slot.dataSize   = totalRead;
    slot.frameIndex = idx;
    slot.frameCount = frameCount;
    slot.loaded     = true;
    strncpy(slot.path, path, sizeof(slot.path) - 1);
    slot.path[sizeof(slot.path) - 1] = '\0';   // strncpy 不保证补零，显式终止

    LOG("[LCD] 缓存[%d] ✓ %s → PSRAM\n", slotIdx, path);
    LOG("[LCD]   大小: %dKB | 帧数: %d | 加载: %dms\n",
        (int)(totalRead / 1024), frameCount, (int)loadMs);
    LOG("[LCD]   池使用: %dKB / %dKB\n",
        (int)(poolUsed / 1024), (int)(PSRAM_LCD_POOL_SIZE / 1024));
    return true;
}

// ══════════════════════════════════════════════════════════════
//  视频加载：SD → PSRAM + 帧索引构建
// ══════════════════════════════════════════════════════════════

/**
 * @brief 将视频文件加载到 PSRAM 池中，并构建帧索引
 *
 * 步骤：
 *   1. 检查文件存在性和大小
 *   2. 从池中分配空间，整体读入
 *   3. 扫描 PSRAM 数据，找出所有 0xFFD8...FFD9 帧边界
 *   4. 构建 frameIndex[] 数组
 *
 * @param slotIdx  缓存槽编号 (0~3)
 * @param path     Flash文件路径
 * @return true = 加载成功
 */
static bool loadVideoToCache(int slotIdx, const char* path)
{
    if (slotIdx < 0 || slotIdx >= VIDEO_CACHE_SLOTS) return false;

    videoCacheSlot_t& slot = cache[slotIdx];
    slot.loaded = false;

    // ── 打开文件 ──
    if (!LittleFS.exists(path)) {
        LOG("[LCD] 缓存[%d] 文件不存在: %s\n", slotIdx, path);
        return false;
    }

    File f = LittleFS.open(path, FILE_READ);
    if (!f) {
        LOG("[LCD] 缓存[%d] 无法打开: %s\n", slotIdx, path);
        return false;
    }

    size_t fileSize = f.size();
    if (fileSize < 4) {
        LOG("[LCD] 缓存[%d] 文件过小: %d bytes\n", slotIdx, (int)fileSize);
        f.close();
        return false;
    }

    // ── 分配池空间 ──
    uint8_t* data = poolAlloc(fileSize);
    if (!data) {
        LOG("[LCD] 缓存[%d] PSRAM 空间不足: 需要 %dKB, 剩余 %dKB\n",
            slotIdx, (int)(fileSize / 1024),
            (int)((PSRAM_LCD_POOL_SIZE - poolUsed) / 1024));
        f.close();
        return false;
    }

    // ── 整体读入 PSRAM ──
    uint32_t t0 = millis();
    size_t totalRead = 0;
    while (totalRead < fileSize) {
        size_t chunk = fileSize - totalRead;
        if (chunk > 32768) chunk = 32768;
        size_t r = f.read(data + totalRead, chunk);
        if (r == 0) break;
        totalRead += r;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    f.close();

    uint32_t loadMs = millis() - t0;

    if (totalRead != fileSize) {
        LOG("[LCD] 缓存[%d] 读取不完整: %d/%d\n",
            slotIdx, (int)totalRead, (int)fileSize);
        return false;
    }

    // 共用：扫描帧边界、建索引、填充 slot
    return finalizeSlotFromData(slotIdx, data, totalRead, path, loadMs);
}

// ══════════════════════════════════════════════════════════════
//  查找缓存槽
// ══════════════════════════════════════════════════════════════

static int findCacheSlot(const char* path)
{
    for (int i = 0; i < VIDEO_CACHE_SLOTS; i++) {
        if (cache[i].loaded && strcmp(cache[i].path, path) == 0) {
            return i;
        }
    }
    return -1;
}

// ══════════════════════════════════════════════════════════════
//  流式回退（文件太大装不进 PSRAM 时使用）
// ══════════════════════════════════════════════════════════════

static void stResetBlock() { stBlockLen = 0; stBlockPos = 0; }

static inline bool stRefill()
{
    if (stBlockPos < stBlockLen) return true;
    stBlockLen = streamFile.read(streamReadBlk, READ_BLOCK_SIZE);
    stBlockPos = 0;
    return (stBlockLen > 0);
}

static uint32_t stReadNextFrame(uint8_t* buf, size_t bufSize)
{
    if (!stRefill()) return 0;

    // 搜索帧头
    while (true) {
        uint8_t* p = (uint8_t*)memchr(
            streamReadBlk + stBlockPos, 0xFF, stBlockLen - stBlockPos);
        if (!p) { if (!stRefill()) return 0; continue; }
        size_t idx = p - streamReadBlk;
        if (idx + 1 < stBlockLen) {
            if (streamReadBlk[idx + 1] == 0xD8) { stBlockPos = idx + 2; break; }
            stBlockPos = idx + 1; continue;
        }
        if (!stRefill()) return 0;
        if (stBlockLen > 0 && streamReadBlk[0] == 0xD8) { stBlockPos = 1; break; }
    }

    buf[0] = 0xFF; buf[1] = 0xD8;
    size_t pos = 2;

    while (true) {
        if (!stRefill()) break;
        size_t rem = stBlockLen - stBlockPos;
        uint8_t* end = (uint8_t*)memchr(streamReadBlk + stBlockPos, 0xFF, rem);
        if (end) {
            size_t cl = end - (streamReadBlk + stBlockPos);
            if (pos + cl + 2 > bufSize) { stats.errorCount++; return 0; }
            memcpy(buf + pos, streamReadBlk + stBlockPos, cl);
            pos += cl; stBlockPos += cl;
            if (stBlockPos + 1 < stBlockLen && streamReadBlk[stBlockPos + 1] == 0xD9) {
                buf[pos++] = 0xFF; buf[pos++] = 0xD9;
                stBlockPos += 2;
                return (uint32_t)pos;
            }
            buf[pos++] = streamReadBlk[stBlockPos++];
        } else {
            if (pos + rem > bufSize) { stats.errorCount++; return 0; }
            memcpy(buf + pos, streamReadBlk + stBlockPos, rem);
            pos += rem; stBlockPos = stBlockLen;
        }
    }
    return 0;
}

// ══════════════════════════════════════════════════════════════
//  播放器核心
// ══════════════════════════════════════════════════════════════

// 电量叠加相关：定义在文件靠后，这里前向声明，供 player() 调用
static bool        batOverlayActive();
static void        drawBatteryOn(lgfx::LovyanGFX& g);
static void        drawOverlaysOn(lgfx::LovyanGFX& g);   // 电量 + 配对爱心

/**
 * @brief 播放一帧
 *
 * 预加载模式（零拷贝）：
 *   PSRAM 指针 → drawJpg() → SPI DMA → 屏幕
 *   无 memcpy，无 SD 读取，无中间缓冲。
 *
 * 流式模式（回退）：
 *   SD → readBlock → frameBuf → drawJpg()
 *
 * @return 0 = 正常, 1 = 播完一轮
 */
static int player()
{
    uint32_t t0 = micros();

    if (!isStreaming && currentSlot >= 0) {
        // ═══ 预加载模式：零拷贝 ═══
        videoCacheSlot_t& slot = cache[currentSlot];

        if (currentFrame >= slot.frameCount) {
            // 播完一轮 → 重头
            currentFrame = 0;
            return 1;
        }

        frameEntry_t& fe = slot.frameIndex[currentFrame];

        healthSetTaskStage(HT_LCD, HS_L_DECODE);   // JPEG 解码 + SPI DMA 推屏
        if (batOverlayActive()) {
            frameCanvas.drawJpg(slot.data + fe.offset, fe.size, 0, 0); // 解码进 PSRAM 缓冲
            drawOverlaysOn(frameCanvas);                               // 缓冲上合成电量 + 配对爱心
            frameCanvas.pushSprite(0, 0);                              // 整块推屏（消除闪烁）
        } else {
            // ★ 无叠加：直接把 PSRAM 指针给 drawJpg，零拷贝最快
            tft.drawJpg(slot.data + fe.offset, fe.size, 0, 0);
        }

        currentFrame++;

    } else if (isStreaming && streamFile) {
        // ═══ 流式模式 ═══
        uint32_t fSize = stReadNextFrame(streamFrameBuf, FRAME_BUF_SIZE);
        if (fSize == 0) {
            streamFile.seek(0);
            stResetBlock();
            return 1;
        }
        if (batOverlayActive()) {
            frameCanvas.drawJpg(streamFrameBuf, fSize, 0, 0);
            drawOverlaysOn(frameCanvas);
            frameCanvas.pushSprite(0, 0);
        } else {
            tft.drawJpg(streamFrameBuf, fSize, 0, 0);
        }

    } else {
        return 1;
    }

    // 统计
    uint32_t elapsed = (micros() - t0);  // 微秒
    uint32_t elapsedMs = elapsed / 1000;
    stats.decodeMs = elapsedMs;
    stats.totalFrames++;

    decodeAccum += elapsedMs;
    decodeCount++;
    if (elapsedMs > stats.maxDecodeMs) stats.maxDecodeMs = elapsedMs;
    if (decodeCount >= 120) {
        stats.avgDecodeMs = decodeAccum / decodeCount;
        decodeAccum = 0;
        decodeCount = 0;
    }

    return 0;
}

// ══════════════════════════════════════════════════════════════
//  视频切换
// ══════════════════════════════════════════════════════════════

/**
 * @brief 切换到指定路径的视频
 *
 * 优先从缓存播放（零拷贝），缓存未命中则流式回退。
 */
static bool switchToVideo(const char* path)
{
    // 关闭流式文件（如果有）
    if (streamFile) streamFile.close();
    isStreaming  = false;
    currentSlot  = -1;
    currentFrame = 0;

    // 查找缓存
    int slot = findCacheSlot(path);
    if (slot >= 0 && cache[slot].frameCount > 0) {
        currentSlot = slot;
        stats.isPreloaded = true;

        const char* name = strrchr(path, '/');
        strncpy(stats.currentFile, name ? name + 1 : path, sizeof(stats.currentFile) - 1);
        stats.currentFile[sizeof(stats.currentFile) - 1] = '\0';
        stats.fileSize = cache[slot].dataSize;

        LOG("[LCD] ▶ 播放(PSRAM): %s [%d帧]\n", stats.currentFile, cache[slot].frameCount);
        return true;
    }

    // 缓存未命中 → 流式回退
    if (!LittleFS.exists(path)) {
        LOG("[LCD] ✗ 文件不存在: %s\n", path);
        return false;
    }

    streamFile = LittleFS.open(path, FILE_READ);
    if (!streamFile) {
        LOG("[LCD] ✗ 打开失败: %s\n", path);
        return false;
    }

    isStreaming = true;
    stResetBlock();
    stats.isPreloaded = false;
    stats.fileSize = streamFile.size();

    const char* name = strrchr(path, '/');
    strncpy(stats.currentFile, name ? name + 1 : path, sizeof(stats.currentFile) - 1);
    stats.currentFile[sizeof(stats.currentFile) - 1] = '\0';

    LOG("[LCD] ▶ 播放(SD流式): %s [%dKB]\n",
        stats.currentFile, (int)(stats.fileSize / 1024));
    return true;
}

// 切到指定视频并循环播放（LCD_FREE 状态循环当前 slot，不限 idle）
static void switchToLoop(const char* path)
{
    strcpy(playerInfo->videoPath, path);
    playerInfo->playTimes    = -1;
    playerInfo->interruptAble = true;

    if (switchToVideo(path)) lcdState = LCD_FREE;
    else                     lcdState = LCD_ERROR;
}

static void switchToFree() { switchToLoop(LCD_FREE_VIDEO); }   // idle 循环

// 是否存有自定义视频(target)：优先看缓存（常态已预加载，纯内存查询），缓存未命中再查文件
// （PSRAM 不足改流式播放的兜底）。两者皆无 = 未存自定义视频（App 从未上传过）。
static bool hasTargetVideo()
{
    int slot = findCacheSlot(LCD_MOTION_VIDEOSHOW_PATH);
    if (slot >= 0 && cache[slot].frameCount > 0) return true;
    return LittleFS.exists(LCD_MOTION_VIDEOSHOW_PATH);
}

static const char* motionToPath(MOTION_t motion)
{
    switch (motion) {
        case MOTION_TAP:       return LCD_MOTION_TAP_PATH;        // nod
        case MOTION_SHAKE:     return LCD_MOTION_SHAKE_PATH;      // swingSwing
        case MOTION_VIDEOSHOW: return LCD_MOTION_VIDEOSHOW_PATH;  // target
        case MOTION_SHOWUP:    return LCD_SHOWUP_PATH;
        case MOTION_WINK:      return LCD_WINK_PATH;
        case MOTION_STARTSLEEP:return LCD_STARTSLEEP_PATH;
        case MOTION_SLEEPING:  return LCD_SLEEPING_PATH;
        default:               return LCD_FREE_VIDEO;
    }
}

static void processMessage(MessageToLCD_t& rxMsg, bool& msgPending, int& playCounter)
{
    if (rxMsg.playTimes == 0) {
        switchToFree();
        playCounter = 0;
        msgPending  = false;
        return;
    }

    MOTION_t motion = rxMsg.motion;
    // ★ 收到"播自定义视频(target)"请求（好友发视频 VIDEO_SHOW）、但设备没存自定义视频时 →
    //   退回播点头(nod)，而不是待机 idle。（伙伴拍一拍 POKE 现固定走 MOTION_TAP，不经此分支）
    if (motion == MOTION_VIDEOSHOW && !hasTargetVideo()) {
        LOG("[LCD] 无自定义视频 → 拍一拍改播点头(nod)\n");
        motion = MOTION_TAP;
    }

    const char* path = motionToPath(motion);
    strcpy(playerInfo->videoPath, path);
    playerInfo->playTimes    = rxMsg.playTimes;
    playerInfo->interruptAble = rxMsg.interruptAble;

    if (switchToVideo(path)) {
        lcdState    = playerInfo->interruptAble ? LCD_PLAYING : LCD_PLAYING_NONINT;
        playCounter = 0;
        msgPending  = false;
    } else {
        stats.errorCount++;
        msgPending = false;
        switchToFree();
    }
}

// ══════════════════════════════════════════════════════════════
//  target 视频热重载
// ══════════════════════════════════════════════════════════════

// ★ 打点在函数体第一行（见下）：重载要读整个 target.mjpeg 并重建帧索引，是 lcdTask
//   少数会长时间不返回的路径之一。
static void reloadTargetVideo()
{
    healthSetTaskStage(HT_LCD, HS_L_RELOAD);
    LOG("[LCD] target 视频热重载...\n");

    // target 固定在 slot 7（最后一个）
    const int TARGET_SLOT = 7;

    // 如果当前正在播放 target，先切走
    bool wasPlayingTarget = (currentSlot == TARGET_SLOT);
    if (wasPlayingTarget) {
        switchToFree();
    }

    // 释放 slot 3 及之后的内存
    poolFreeFrom(TARGET_SLOT);

    // 重新加载
    if (loadVideoToCache(TARGET_SLOT, LCD_MOTION_VIDEOSHOW_PATH)) {
        LOG("[LCD] ✓ target 热重载成功\n");
    } else {
        LOG("[LCD] ⚠ target 热重载失败（文件不存在或空间不足）\n");
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    // 更新统计
    stats.cachedVideos = 0;
    stats.totalCachedFrames = 0;
    for (int i = 0; i < VIDEO_CACHE_SLOTS; i++) {
        if (cache[i].loaded) {
            stats.cachedVideos++;
            stats.totalCachedFrames += cache[i].frameCount;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    stats.poolUsed = poolUsed;
}

// ══════════════════════════════════════════════════════════════
//  target 视频：MQTT 直收进 PSRAM（接收期不碰 flash）
//    web 任务把分片直接 memcpy 进池的空闲尾部；
//    END 时 commit → 就地建帧索引、直接当 slot3 播；
//    失败 abort → 从 flash 重载旧 target。
//  commit/abort 的重活（扫描建索引）放在 LCD 任务里执行（池属主）。
// ══════════════════════════════════════════════════════════════

#define LCD_TARGET_SLOT   7

#define TGT_REQ_NONE    0
#define TGT_REQ_ABORT  (-1)
#define TGT_REQ_BEGIN  (-2)

static volatile int  g_tgtReq  = TGT_REQ_NONE;  // 0=无, >0=commit(字节数), -1=abort, -2=begin
static volatile bool g_tgtDone = false;
static volatile bool g_tgtOk   = false;

// begin 的结果（lcdTask 填写，webTask 在 g_tgtDone 置位后读取）。
// volatile 限定的是【指针变量本身】(uint8_t* volatile)，不是它指向的数据。
static uint8_t* volatile g_tgtBase  = nullptr;
static volatile size_t   g_tgtAvail = 0;

// 请求被 webTask 放弃（等超时且宽限期内也没完成）。lcdTask 完成时据此不再置 g_tgtDone，
// 避免迟到的置位串扰到下一次请求。每次新请求发起时清零。
static volatile bool s_tgtAbandoned = false;

// ★ 忙等 lcdTask 处理有 5s 超时兜底：这三个接口都跑在 mqttCallback→webTask 上下文。
//   若 lcdTask 因任何原因未及时响应（渲染卡顿/解码损坏帧/被挂起），死等会把整个
//   MQTT 收包线程钉住。超时改为返回失败，上层走 rollback + 回 VIDEO_FAIL，最坏退化成
//   "本次传输失败可重试"。（等待期间由 waitTgtDone 负责喂 TWDT，见下。）
//   实测各阻塞点均远小于此：L2 休眠 500ms / 信封弹跳 ≤1.3s / 配网 60ms / 传输期 166ms。
#define LCD_RECV_WAIT_TIMEOUT_MS  5000

// 等 lcdTask 处理完一次请求；返回 false = 超时（调用方按失败处理）
// ★ 循环内必须喂狗：本函数跑在 webTask，而 webTask 稳态是订阅了 TWDT(8s) 的。
//   begin/commit 各自最长等 5s，若同一次 client.loop() 里连着处理两条控制帧（NEW_VIDEO
//   与 END_VIDEO 都可能在一次 available() 突发里到达），累计就会越过 8s 触发 panic。
//   esp_task_wdt_reset() 对未订阅的任务是空操作，其它路径（waitPersistIdle）也是这么用的。
static bool waitTgtDone(const char* what)
{
    uint32_t t0 = millis();
    while (!g_tgtDone) {
        if (millis() - t0 > LCD_RECV_WAIT_TIMEOUT_MS) {
            // ★ 撤请求只对【还没被取走】的请求有效。lcdTask 取 req 时是先读后清零
            //   （req = g_tgtReq; g_tgtReq = NONE;），所以这里写 NONE 可能已经晚了——
            //   它或许正在执行池操作。故超时后再宽限一小段，看它是否马上完成；
            //   真完成了就按成功返回，避免"webTask 判失败回滚、lcdTask 却已提交"的状态撕裂。
            g_tgtReq = TGT_REQ_NONE;
            for (int i = 0; i < 20 && !g_tgtDone; i++) {   // 再等 100ms
                vTaskDelay(pdMS_TO_TICKS(5));
                esp_task_wdt_reset();
            }
            if (g_tgtDone) {
                LOG("[LCD] ⚠ %s 超时后宽限期内完成，按成功处理\n", what);
                return true;
            }
            // ★ 宽限期也没完成 → 只能判失败，但 lcdTask 可能【此后】才完成并置 g_tgtDone。
            //   那个迟到的置位会被下一次请求的等待循环误当成"本次已完成"而立即返回
            //   （下一次请求开头虽然清了 g_tgtDone，但清零与 lcdTask 的迟到写入存在竞争）。
            //   置弃用标记：lcdTask 完成时若发现请求已被放弃，就不再置 g_tgtDone。
            s_tgtAbandoned = true;
            LOG("[LCD] ✗ %s 等 lcdTask 超时(%dms)\n", what, LCD_RECV_WAIT_TIMEOUT_MS);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
        esp_task_wdt_reset();             // 未订阅则空操作
    }
    return true;
}

// ★ 本函数运行在 webTask(mqttCallback) 上下文，但【切走播放 + 释放池】必须由 lcdTask 做：
//   这两件事会把 cache[TARGET].frameIndex/data 置 nullptr，而 lcdTask 的 player() 里
//   `frameEntry_t& fe = slot.frameIndex[currentFrame];` 紧跟在 frameCount 检查之后——
//   跨任务清空正好插在这两行之间时就是空指针解引用（LoadProhibited 崩溃）。
//   触发路径真实存在：无音轨的视频不发 NEW_AUDIO，NEW_VIDEO 的 enterTransferMode() 与本函数
//   背靠背执行，而停播用的 XFER_LOCK 是绕队列到主循环再到 LCD 的异步消息，来不及生效。
//   故改成与 commit/abort 相同的请求-应答握手，池操作全部收敛到属主任务 lcdTask。
uint8_t* lcdTargetRecvBegin(size_t* maxBytes)
{
    g_tgtDone      = false;
    s_tgtAbandoned = false;   // 新请求，清掉上一次可能留下的放弃标记
    g_tgtBase  = nullptr;
    g_tgtAvail = 0;
    g_tgtReq   = TGT_REQ_BEGIN;
    if (!waitTgtDone("begin")) { if (maxBytes) *maxBytes = 0; return nullptr; }
    if (maxBytes) *maxBytes = g_tgtAvail;
    return g_tgtBase;
}

bool lcdTargetRecvCommit(size_t size)
{
    g_tgtDone = false; g_tgtOk = false;
    s_tgtAbandoned = false;
    g_tgtReq  = (int)size;
    if (!waitTgtDone("commit")) return false;
    return g_tgtOk;
}

void lcdTargetRecvAbort()
{
    g_tgtDone = false;
    s_tgtAbandoned = false;
    g_tgtReq  = TGT_REQ_ABORT;
    waitTgtDone("abort");
}

// 在 lcdTask 循环里调用：真正执行 commit/abort（池操作的属主在此）
static void lcdProcessTargetRecv()
{
    int req = g_tgtReq;
    if (req == TGT_REQ_NONE) return;
    g_tgtReq = TGT_REQ_NONE;

    if (req == TGT_REQ_BEGIN) {
        // 直收准备：切走 target 播放 + 释放旧 target，腾出池的空闲尾部当接收缓冲。
        // 在 lcdTask 里做 → 与 player() 天然互斥，不会在它解引用 frameIndex 时被抽走。
        if (currentSlot == LCD_TARGET_SLOT) switchToFree();
        poolFreeFrom(LCD_TARGET_SLOT);

        size_t reserve = (size_t)MAX_FRAMES_PER_VIDEO * sizeof(frameEntry_t) + 4096;
        if (poolUsed + reserve >= PSRAM_LCD_POOL_SIZE) {
            g_tgtBase = nullptr; g_tgtAvail = 0;
            LOG("[LCD] ✗ target 直收准备失败：池无空间\n");
        } else {
            g_tgtAvail = PSRAM_LCD_POOL_SIZE - poolUsed - reserve;
            g_tgtBase  = psramPool + poolUsed;   // commit 时 poolAlloc 会返回同一地址
            LOG("[LCD] target 直收准备：基址 %p 上限 %dKB\n",
                g_tgtBase, (int)(g_tgtAvail / 1024));
        }
        stats.poolUsed = poolUsed;
        // ★ 请求已被 webTask 放弃（等超时）→ 不置 g_tgtDone，避免这个迟到的置位
        //   串扰到下一次请求。池操作本身已经做完且状态自洽（旧 target 已释放）。
        if (!s_tgtAbandoned) g_tgtDone = true;
        return;
    }

    if (req > 0) {
        uint8_t* data = poolAlloc((size_t)req);    // 应当 == begin 返回的接收基址
        // ★ 不变量校验：webTask 是照 begin 给的基址把分片写进池的，commit 这次 poolAlloc
        //   必须落在同一地址。若两者之间有别的 poolAlloc 插进来（如 LCD_ERROR 恢复时
        //   重载 idle），地址会错位，那样建出的帧索引指向的根本不是收到的数据 → 花屏/崩溃。
        //   宁可判失败回滚、让 App 重传，也不要把错位数据当成有效视频。
        bool addrOk = (data == g_tgtBase);
        if (data && !addrOk)
            LOG("[LCD] ✗ commit 基址错位 %p != %p（期间池被别处分配过）\n", data, g_tgtBase);
        bool ok = data && addrOk &&
                  finalizeSlotFromData(LCD_TARGET_SLOT, data, (size_t)req,
                                       LCD_MOTION_VIDEOSHOW_PATH, 0);
        if (!ok) {                                  // 建索引失败 → 回退到 flash 旧文件
            poolFreeFrom(LCD_TARGET_SLOT);
            loadVideoToCache(LCD_TARGET_SLOT, LCD_MOTION_VIDEOSHOW_PATH);
        }
        g_tgtOk = ok;
        LOG("[LCD] target 直收提交: %s\n", ok ? "OK" : "FAIL(已回退flash)");
    } else {
        poolFreeFrom(LCD_TARGET_SLOT);
        loadVideoToCache(LCD_TARGET_SLOT, LCD_MOTION_VIDEOSHOW_PATH);
        g_tgtOk = false;
        LOG("[LCD] target 直收放弃，已从 flash 重载旧文件\n");
    }

    stats.cachedVideos = 0; stats.totalCachedFrames = 0;
    for (int i = 0; i < VIDEO_CACHE_SLOTS; i++) {
        if (cache[i].loaded) { stats.cachedVideos++; stats.totalCachedFrames += cache[i].frameCount; }
    }
    stats.poolUsed = poolUsed;
    if (!s_tgtAbandoned) g_tgtDone = true;   // 理由同上：被放弃的请求不再置完成标志
}

// ══════════════════════════════════════════════════════════════
//  电量叠加：每帧在右上角重绘一个小电池图标（帧会覆盖该角，故须每帧画）
//  数值由主循环更新到 g_batPercent（-1=未知则不画）
// ══════════════════════════════════════════════════════════════

extern volatile int  g_batPercent;     // 在 V1_1.ino 定义：0~100 电量，-1=未知
extern volatile bool g_batCharging;    // 在 V1_1.ino 定义：true=充电中（按硬件充电信号置位）

extern SemaphoreHandle_t xLcdWake;       // L2 唤醒信号量（V1_1.cpp）
#define LCD_L2_BLOCK_MS  500

// ── 功耗模式（主循环设置，lcdTask 应用）──
//   0 = L0 正常全亮
//   1 = L1 亮度降为基准 60%
//   2 = L2 面板休眠 + 背光关 + 停止解码/推屏（仍处理收图与消息）
static volatile int g_lcdPowerMode   = 0;   // 目标模式（主循环写）
volatile bool g_lcdSleeping = false;         // 睡眠动画中（startSleep/sleeping）：不算"活动"，供 lcdIsPlaying 排除
static int          g_lcdAppliedMode = -1;  // lcdTask 已应用的模式

// ── 背光渐变：curBri 每帧朝 tgtBri 逼近，消除睡眠/唤醒/亮度分级的瞬间跳变 ──
static int  s_curBri     = -1;     // 当前实际背光（-1=未初始化）
static int  s_tgtBri     = 255;    // 目标背光
static bool s_panelSlept = false;  // 面板是否已 tft.sleep()（L2 渐暗到 0 后才真休眠）
#define BRI_FADE_STEP  20          // 每帧背光步进（255/20≈13 帧 ≈ 0.5~0.7s）
#define LCD_L1_BRIGHT        0.4f  // L1 空闲亮度系数（相对配网基础亮度）
#define LCD_ENVELOPE_BRIGHT  0.1f  // ★ 信封待回复态亮度系数（调这个改信封态屏幕亮度）

// 每帧把背光朝目标逼近一步；返回 true = 仍在渐变中
static bool stepBrightness()
{
    if (s_curBri < 0) s_curBri = s_tgtBri;              // 首次：直接对齐
    if (s_curBri == s_tgtBri) return false;
    if (s_curBri < s_tgtBri) { s_curBri += BRI_FADE_STEP; if (s_curBri > s_tgtBri) s_curBri = s_tgtBri; }
    else                     { s_curBri -= BRI_FADE_STEP; if (s_curBri < s_tgtBri) s_curBri = s_tgtBri; }
    tft.setBrightness((uint8_t)s_curBri);
    return s_curBri != s_tgtBri;
}
void lcdSetPowerMode(int mode) { g_lcdPowerMode = mode; }

static bool g_batOverlayOn = true;     // 总开关（可用 lcdSetBatteryOverlay 关）
static bool g_canvasReady  = false;    // 全屏帧缓冲是否创建成功

void lcdSetBatteryOverlay(bool on) { g_batOverlayOn = on; }

// 是否正在播放（TAP/SHAKE/target），供主循环把"播放中"持续计为活动。
// LCD_FREE(待机循环) 不算，避免待机也被当成活动而不降频。
bool lcdIsPlaying()
{
    // 睡眠动画(startSleep/sleeping)不算"活动"，否则一进 L1 就被判活动立刻醒
    return (lcdState == LCD_PLAYING || lcdState == LCD_PLAYING_NONINT) && !g_lcdSleeping;
}

// 把电量电池画到指定画布 g（可为全屏 sprite，也可为 tft 直绘回退）
static void drawBatteryOn(lgfx::LovyanGFX& g)
{
    const int W = 26, H = 12, NUB = 2, M = 4;   // 电池体宽高、正极头、边距
    int x = g.width() - W - NUB - M;             // 右上角
    int y = M;

    g.drawRect(x, y, W, H, TFT_WHITE);                       // 外壳
    g.fillRect(x + W, y + (H - 5) / 2, NUB, 5, TFT_WHITE);   // 正极头
    g.fillRect(x + 2, y + 2, W - 4, H - 4, TFT_BLACK);       // 内部清底

    if (g_batCharging) {               // 充电中：绿色填充条 0→满 循环动画
        static uint8_t anim = 0;
        anim = (anim + 6) % 108;
        int aw = (W - 4) * (anim > 100 ? 100 : anim) / 100;
        if (aw > 0) g.fillRect(x + 2, y + 2, aw, H - 4, TFT_GREEN);
        return;
    }

    int pct = g_batPercent;
    if (pct < 0) return;               // 未知：只留空壳
    if (pct > 100) pct = 100;
    uint16_t col = (pct <= 20) ? TFT_RED : TFT_WHITE;        // ≤20% 红，否则白
    int fillW = (W - 4) * pct / 100;                         // 填充条
    if (fillW > 0) g.fillRect(x + 2, y + 2, fillW, H - 4, col);
}

// ══════════════════════════════════════════════════════════════
//  没电大图标（居中）：横向电池轮廓 + 内部 ~10% 红色残量 + 红色感叹号。
//  低电关机后长按开机、复检仍 ≤6.4V 时全屏显示 3 秒。风格延续 drawBatteryOn。
// ══════════════════════════════════════════════════════════════
static void drawLowBatteryIcon(lgfx::LovyanGFX& g)
{
    const int cx = g.width() / 2, cy = g.height() / 2;
    const int W = 120, H = 60, NUB = 7, NUBH = 20, TH = 5, R = 12;  // 电池体宽高、正极头宽高、描边粗、圆角半径
    const int x = cx - (W + NUB) / 2;
    const int y = cy - H / 2;

    g.fillScreen(TFT_BLACK);

    // 圆角外壳：白圆角实心 → 内部挖黑，留 TH 宽干净白边框。
    //   （多层 drawRoundRect 各层半径 R-i 不同、圆角弧不连续，四角会露黑点；
    //     "填充-挖空"两步的圆角是连续的，边框干净无锯齿。）
    g.fillRoundRect(x, y, W, H, R, TFT_WHITE);
    g.fillRoundRect(x + TH, y + TH, W - 2 * TH, H - 2 * TH, R - TH, TFT_BLACK);

    // 正极头：贴电池体右侧直边、竖向居中，内插 2px 与体咬合（不留缝、不凸太多）
    g.fillRoundRect(x + W - 2, cy - NUBH / 2, NUB + 2, NUBH, 3, TFT_WHITE);

    // 内部 ~10% 红色残量条（靠左）
    const int inX = x + TH + 5, inY = y + TH + 5;
    const int inH = H - 2 * (TH + 5);
    const int fillW = (W - 2 * (TH + 5)) * 10 / 100;
    g.fillRect(inX, inY, fillW, inH, TFT_RED);

    // 红色感叹号（在电池体水平 + 竖直正中心）：粗竖条 + 下圆点
    const int exX = x + W / 2;                       // 电池体水平中心
    const int barW = 10, barTop = y + 12, barH = 22; // 竖条：整体竖向居中于 H=60
    g.fillRect(exX - barW / 2, barTop, barW, barH, TFT_RED);
    g.fillCircle(exX, barTop + barH + 9, 5, TFT_RED); // 竖条下方间隙 4 + 圆点半径 5
}

// 全屏显示没电图标 holdMs 毫秒（亮度 50），用于关机休眠态复检仍没电时的提示。
// 纯净环境可调用：内部自行 tft.init（不预加载视频）；显示完关背光，调用方随后 esp_restart。
void lcdShowLowBatteryScreen(uint32_t holdMs)
{
    tft.init();
    tft.setRotation(LCD_ROTATION);
    tft.setBrightness(0);            // 先灭再画，避免点亮瞬间花屏
    drawLowBatteryIcon(tft);
    tft.setBrightness(50);           // 需求指定亮度 50
    delay(holdMs);
    tft.setBrightness(0);            // 关背光（随后 esp_restart 复位面板）
}

// ══════════════════════════════════════════════════════════════
//  OTA 升级进度屏（纯净环境：内部自行 tft.init，不预加载视频）
//  文字用英文——LovyanGFX 默认字库无中文，中文会画不出来。
// ══════════════════════════════════════════════════════════════
static int s_otaBarX = 30, s_otaBarY = 150, s_otaBarW = 260, s_otaBarH = 24;

void lcdOtaBegin() {
    tft.init();
    tft.setRotation(LCD_ROTATION);
    tft.setBrightness(0);                       // 先灭再画，避免点亮瞬间花屏
    tft.fillScreen(0x0000);                     // 黑底
    const uint16_t white = tft.color565(255, 255, 255);
    const int W = tft.width(), H = tft.height();
    tft.setTextColor(white);
    tft.setTextSize(2);
    tft.setCursor(W / 2 - 54, H / 2 - 48);
    tft.print("FW UPDATE");
    s_otaBarW = W - 60; s_otaBarH = 24;
    s_otaBarX = 30;     s_otaBarY = H / 2 - 6;
    tft.drawRect(s_otaBarX, s_otaBarY, s_otaBarW, s_otaBarH, white);
    tft.setBrightness(90);
}

void lcdOtaProgress(int pct) {
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    const uint16_t green = tft.color565(0, 220, 90);
    const uint16_t white = tft.color565(255, 255, 255);
    const int fillW = (s_otaBarW - 4) * pct / 100;
    tft.fillRect(s_otaBarX + 2, s_otaBarY + 2, fillW, s_otaBarH - 4, green);
    tft.fillRect(s_otaBarX, s_otaBarY + s_otaBarH + 8, 80, 20, 0x0000);   // 清旧数字
    tft.setTextColor(white);
    tft.setTextSize(2);
    tft.setCursor(s_otaBarX, s_otaBarY + s_otaBarH + 8);
    tft.printf("%d%%", pct);
}

void lcdOtaMessage(const char* msg) {
    const uint16_t white = tft.color565(255, 255, 255);
    const int W = tft.width(), H = tft.height();
    tft.fillRect(0, H - 34, W, 34, 0x0000);
    tft.setTextColor(white);
    tft.setTextSize(1);
    tft.setCursor(10, H - 26);
    tft.print(msg);
}

// ══════════════════════════════════════════════════════════════
//  配对爱心叠加：两颗粉色爱心（一大一小重叠）——已互绑且伙伴在线时显示
// ══════════════════════════════════════════════════════════════
extern volatile bool g_pairLinked;     // web.cpp：已互绑 且 伙伴在线

// 实心爱心 = 两瓣圆(左右) + 下三角尖角。(cx,apexY)=底部尖角，r=瓣半径。
// 瓣心间距 0.9r(<r → 两瓣交叠、顶部平、侧凸小) + 长三角(高≈4r>宽≈3.8r → 瘦长不鼓)。
// 以底部尖角锚定，方便多颗心"底部齐平"。
static void drawHeart(lgfx::LovyanGFX& g, int cx, int apexY, int r, uint16_t color)
{
    const int dx = (r * 17) / 20;         // 瓣心间距 0.85r：交叠多→顶部圆润饱满、不露两个圈
    const int ly = apexY - (r * 27) / 10; // 瓣心 y：2.7r 三角(比之前短)→更圆更饱满、不尖
    g.fillCircle(cx - dx, ly, r, color);                          // 左瓣
    g.fillCircle(cx + dx, ly, r, color);                          // 右瓣
    g.fillTriangle(cx - dx - r, ly, cx + dx + r, ly, cx, apexY, color); // 下尖角
}

// 两颗心叠在电池左侧、底部齐平（想微调就改这几个常量）
static void drawPairHeartsOn(lgfx::LovyanGFX& g)
{
    const uint16_t pinkBig   = g.color565(255,  78, 142);    // 深粉（大，在后）
    const uint16_t pinkSmall = g.color565(255, 158, 196);    // 浅粉（小，在前）

    const int rBig = 4, rSmall = 3;                          // 加大 → 更宽更饱满
    const int batW = 26, batNub = 2, batM = 4;
    const int batX = g.width() - batW - batNub - batM;       // 电池左边缘

    const int apexY = batM + 12;    // 两心尖角同一水平线 → 底部齐平（与电池底对齐）
    const int sx    = batX - 35;    // 小心中心：放在 WiFi 图标左侧，各图标间留 ~8px
    const int bx    = sx - 5;       // 大心中心（在后、靠左，与小心重叠）

    drawHeart(g, bx, apexY, rBig,   pinkBig);        // ① 大心在后
    drawHeart(g, sx, apexY, rSmall, pinkSmall);      // ② 小心在前（浅粉压在上）
}

// ── WiFi 状态图标：连上=白色，未连=红色 + 右下角叉。三段弧 + 底部圆点。──
extern volatile bool g_wifiOnline;     // web.cpp：WiFi 是否已连接
extern volatile bool g_xfering;        // web.cpp：视频/音频下载传输中
extern volatile bool g_persistWriting; // web.cpp：persistTask 是否【真正在写 flash】。
                                       // ★ 不用 g_persistPending：落盘已改为延后到 L2 窗口，
                                       //   作业会排队好几分钟、pending 全程 >0 却没在写，
                                       //   用它判会让 LCD 从传输完成起一直卡在低帧率。
extern volatile int      g_provStage;    // provision.cpp：配网阶段 1=等BLE 2=BLE已连 3=连WiFi中
extern volatile uint32_t g_provDeadline; // provision.cpp：等BLE倒计时截止 millis
extern volatile bool     g_needWifiHint; // web.cpp：有配置但连不上 → 显示"长按2秒配置"引导

#define LCD_XFER_DELAY_MS  166         // ★ 下载/存盘期每帧固定让出延迟（ms）：不补偿解码，
                                       //   无条件让出，把 core0 稳定让给 webTask 收包 / persistTask 落盘

static void drawWifiOn(lgfx::LovyanGFX& g)
{
    const uint16_t col = g_wifiOnline ? TFT_WHITE : TFT_RED;
    const int batW = 26, batNub = 2, batM = 4;
    const int batX = g.width() - batW - batNub - batM;   // 电池左边缘
    const int ox = batX - 15;      // WiFi 原点(底部圆点) x：电池左侧 ~8px 间距
    const int oy = batM + 11;      // y：底部与电池带对齐

    // 三段弧(内→外) + 底部圆点。fillArc 角度：0=右 90=下 270=上；225~315 = 顶部扇面。
    // （若烧上去发现扇面朝下，把 225/315 改成 45/135 即可翻转到朝上。）
    g.fillArc(ox, oy, 3,  4,  225, 315, col);
    g.fillArc(ox, oy, 6,  7,  225, 315, col);
    g.fillArc(ox, oy, 9, 10, 225, 315, col);
    g.fillCircle(ox, oy, 1, col);

    if (!g_wifiOnline) {           // 未连：右下角画个红叉（画两遍加粗）
        const int xr = ox + 6, yr = oy - 1;
        for (int t = 0; t <= 1; t++) {
            g.drawLine(xr - 3, yr - 3 + t, xr + 3, yr + 3 + t, TFT_RED);
            g.drawLine(xr + 3, yr - 3 + t, xr - 3, yr + 3 + t, TFT_RED);
        }
    }
}

// 叠加合成：WiFi(常显) + 电量(有内容才显) + 配对爱心(已配对且在线才显)
static void drawOverlaysOn(lgfx::LovyanGFX& g)
{
    drawWifiOn(g);
    if (g_batOverlayOn && (g_batCharging || g_batPercent >= 0)) drawBatteryOn(g);
    if (g_pairLinked) drawPairHeartsOn(g);
}

// 当前帧是否需要走"合成缓冲"路径：
//   叠加开 + 缓冲就绪 + 有内容可显示(充电或已知电量) + 仅待机/TAP/SHAKE(槽0/1/2，非 target)
static inline bool batOverlayActive()
{
    // 待机/TAP/SHAKE(非 target 视频)时始终走合成路径——WiFi 图标常显，
    // 电量/爱心按各自条件叠加。
    return g_canvasReady
        && currentSlot >= 0 && currentSlot != LCD_TARGET_SLOT;
}

// ══════════════════════════════════════════════════════════════
//  错误恢复
// ══════════════════════════════════════════════════════════════

static void handleErrorState()
{
    static uint32_t lastRetry = 0;
    if (millis() - lastRetry < 3000) {
        vTaskDelay(pdMS_TO_TICKS(100));
        return;
    }
    lastRetry = millis();

    LOG("[LCD] 错误恢复尝试...\n");

    // 尝试重新加载待机视频
    if (!cache[0].loaded) {
        loadVideoToCache(0, LCD_FREE_VIDEO);
    }

    if (cache[0].loaded || LittleFS.exists(LCD_FREE_VIDEO)) {
        switchToFree();
        if (lcdState == LCD_FREE) {
            LOG("[LCD] ✓ 恢复成功\n");
            return;
        }
    }

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(20, 140);
    tft.print("Waiting for video...");
}

// ══════════════════════════════════════════════════════════════
//  初始化
// ══════════════════════════════════════════════════════════════

// 前置声明：lcdInit 里 tft.init() 之后立即后台起转（定义在 drawXferAnim 之后）。
static void lcdStartBootSpinner();

void lcdInit()
{
    LOG("[LCD] ═══════════════════════════════════\n");
    LOG("[LCD]  初始化（4MB PSRAM 预加载模式）\n");
    LOG("[LCD] ═══════════════════════════════════\n");

    // ── 硬件 ──
    tft.init();
    tft.setRotation(LCD_ROTATION);
    {
        int initBri = Config.getInt("brightness", 255);
        // ★ init 后先灭：面板刚复位、8 个视频尚未预加载完，此时点亮只会显示花屏。
        //   背光保持 0，由 lcdTask 开机进度条每帧渐亮到目标（转圈"从黑淡入"，消除开机花屏闪）。
        tft.setBrightness(0);
        s_curBri = 0;                    // 从全灭开始
        s_tgtBri = initBri;              // 目标 = 配置亮度，开机转圈期间渐亮到此
        s_panelSlept = false;
    }

    // ★ 面板已就绪 → 立刻后台起"开机转圈"，覆盖下面预加载 + 各外设初始化的黑屏。
    //   本任务在 core0 独立渲染，setup 主线程(core1)继续预加载/初始化，两者并行不冲突
    //   （预加载/音频/IMU/舵机均不碰 tft）。创建 lcdTask 前由 lcdStopBootSpinner() 交接。
    lcdStartBootSpinner();

    // ── 全屏帧缓冲（PSRAM）：用于"视频帧 + 电量叠加"合成后整块推屏，消除叠加闪烁 ──
    frameCanvas.setColorDepth(16);
    frameCanvas.setPsram(true);
    if (frameCanvas.createSprite(tft.width(), tft.height())) {
        g_canvasReady = true;
        LOG("[LCD] ✓ 帧缓冲: %dx%d @PSRAM (%dKB)\n",
            tft.width(), tft.height(), tft.width() * tft.height() * 2 / 1024);
    } else {
        LOG("[LCD] ⚠ 帧缓冲创建失败，电量叠加退回直绘（可能会闪）\n");
    }

    // ── 清零统计 ──
    memset(&stats, 0, sizeof(stats));
    memset(cache, 0, sizeof(cache));

    // ── 分配 4MB PSRAM 池 ──
    psramPool = (uint8_t*)heap_caps_malloc(
        PSRAM_LCD_POOL_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (!psramPool) {
        LOG("[LCD] ✗ 4MB PSRAM 池分配失败！\n");
        LOG("[LCD]   可用 PSRAM: %d KB\n",
            (int)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
        lcdState = LCD_ERROR;
        return;
    }
    poolUsed = 0;
    stats.poolTotal = PSRAM_LCD_POOL_SIZE;

    LOG("[LCD] ✓ PSRAM 池: %d KB @ %p\n",
        (int)(PSRAM_LCD_POOL_SIZE / 1024), psramPool);

    // ── 分配 playerInfo ──
    playerInfo = (playerInfo_t*)poolAlloc(sizeof(playerInfo_t));
    if (playerInfo == NULL) {
        LOG("[LCD] ✗ playerInfo 分配失败！PSRAM池已满\n");
        lcdState = LCD_ERROR;
        return;
    }

    // ── 分配流式回退缓冲（以防万一）──
    streamFrameBuf = (uint8_t*)poolAlloc(FRAME_BUF_SIZE);
    streamReadBlk  = (uint8_t*)poolAlloc(READ_BLOCK_SIZE);

    LOG("[LCD] 基础分配: %d KB（含回退缓冲）\n", (int)(poolUsed / 1024));

    // ── 预加载所有视频 ──
    LOG("[LCD] ── 开始预加载 ──\n");

    struct { int slot; const char* path; } preloadList[] = {
        { 0, LCD_FREE_VIDEO },              // idle
        { 1, LCD_MOTION_TAP_PATH },         // nod
        { 2, LCD_MOTION_SHAKE_PATH },       // swingSwing
        { 3, LCD_SHOWUP_PATH },
        { 4, LCD_WINK_PATH },
        { 5, LCD_STARTSLEEP_PATH },
        { 6, LCD_SLEEPING_PATH },
        { 7, LCD_MOTION_VIDEOSHOW_PATH },   // target（可热重载，放最后）
    };

    for (auto& item : preloadList) {
        // ★ 打点到【具体哪个文件】：预加载是启动期耗时最长的环节，读的又是 flash 上
        //   可能被上次崩溃写坏的内容（/target.mjpeg 由延后落盘写入，写一半掉电就是半截）。
        //   崩在这里时 stage 直接指出是哪一个视频，不必接串口逐个试。
        healthSetStage(BS_LCD_PRELOAD + item.slot);
        if (!loadVideoToCache(item.slot, item.path)) {
            LOG("[LCD] ⚠ 跳过: %s\n", item.path);
        }
    }
    healthSetStage(BS_LCD_DONE);

    // ── 汇总统计 ──
    stats.poolUsed = poolUsed;
    stats.cachedVideos = 0;
    stats.totalCachedFrames = 0;
    for (int i = 0; i < VIDEO_CACHE_SLOTS; i++) {
        if (cache[i].loaded) {
            stats.cachedVideos++;
            stats.totalCachedFrames += cache[i].frameCount;
        }
    }

    LOG("[LCD] ── 预加载完成 ──\n");
    LOG("[LCD]   缓存视频: %d 个 | 总帧数: %d\n",
        stats.cachedVideos, stats.totalCachedFrames);
    LOG("[LCD]   PSRAM 池: %dKB / %dKB (%.1f%%)\n",
        (int)(poolUsed / 1024), (int)(PSRAM_LCD_POOL_SIZE / 1024),
        100.0f * poolUsed / PSRAM_LCD_POOL_SIZE);

    // ── 开机播放 showUp 一次，播完自动接 idle 循环 ──
    delay(50);
    strcpy(playerInfo->videoPath, LCD_SHOWUP_PATH);
    playerInfo->playTimes    = 1;
    playerInfo->interruptAble = true;

    if (switchToVideo(LCD_SHOWUP_PATH)) {
        lcdState = LCD_PLAYING;                 // 播完 → switchToFree(idle)
    } else if (switchToVideo(LCD_FREE_VIDEO)) {
        lcdState = LCD_FREE;                    // 无 showUp 则直接 idle
    } else {
        lcdState = LCD_ERROR;
    }

    LOG("[LCD] ✓ 初始化完成\n");
}

// ══════════════════════════════════════════════════════════════
//  主任务
// ══════════════════════════════════════════════════════════════

// 应用功耗模式到面板（只在 lcdTask 内调用，确保 tft 单线程访问）
static void lcdApplyPowerMode(int mode)
{
    int base = Config.getInt("brightness", 255);
    if (mode == 2) {
        // L2：目标渐暗到 0（渐暗完成后由 lcdTask 才 tft.sleep()），不再瞬间关背光
        s_tgtBri = 0;
        LOG("[LCD] ▼ L2 渐暗休眠\n");
    } else {
        if (s_panelSlept) {                 // 仅当面板真休眠过才唤醒（渐暗中途拉回不冗余 wakeup）
            tft.wakeup();
            s_panelSlept = false;
            LOG("[LCD] ▲ 面板唤醒（渐亮）\n");
        }
        if (mode == 3) tft.fillScreen(0x0000);   // 进信封态先全屏黑一次，之后信封只局部刷新
        // 信封态(mode3) 和 L1(mode1) 各用独立系数（改上面的宏）；L0(mode0) 满亮
        if      (mode == 3) s_tgtBri = (int)(base * LCD_ENVELOPE_BRIGHT);
        else if (mode == 1) s_tgtBri = (int)(base * LCD_L1_BRIGHT);
        else                s_tgtBri = base;
        LOG("[LCD] 亮度→%d (L%d，渐变)\n", s_tgtBri, mode);
    }
    g_lcdAppliedMode = mode;
}

// ══════════════════════════════════════════════════════════════
//  录音音量表（矢量绘制，录音时替换全屏；由 mic.cpp 置位/写电平）
//    话筒图标固定，胶囊内部绿色由下至上随音量涨，内缩留白边。
// ══════════════════════════════════════════════════════════════
volatile bool g_micMeterActive = false;   // mic.cpp：录音开始 true / 停止 false
volatile int  g_micLevel       = 0;       // mic.cpp：0~100 实时音量
static bool   s_meterInited     = false;
static int    s_lastMeterLevel  = -1;

static LGFX_Sprite meterSpr(&tft);        // 内胶囊填充双缓冲（消闪烁）
static int mInX, mInY, mInW, mInH, mInR;  // 内胶囊坐标（drawMicIconStatic 里按屏幕算）

// 按实际屏幕尺寸(tft.width()/height())居中画话筒图标；内胶囊内缩 7px 留白边
static void drawMicIconStatic()
{
    const uint16_t W = 0xFFFF;
    int cx = tft.width() / 2, cy = tft.height() / 2;
    const int capW = 46, capH = 86, capR = 23, inset = 7;
    int capX = cx - capW / 2, capY = cy - 60;

    mInX = capX + inset; mInY = capY + inset;
    mInW = capW - 2 * inset; mInH = capH - 2 * inset; mInR = capR - inset;

    tft.fillScreen(0x0000);
    tft.fillRoundRect(capX, capY, capW, capH, capR, W);              // 胶囊主体
    // 拾音弧：开口向上的碗，两端翘到胶囊下部两侧、弧底探到胶囊下方（贴合胶囊曲边）
    tft.fillArc(cx, capY + capH - 20, capW / 2 + 2, capW / 2 + 8, 32, 148, W);
    tft.fillRoundRect(cx - 4, capY + capH + 8, 8, 18, 4, W);         // 竖线（顶端插入弧底，无缝连接）

    if (meterSpr.width() == 0) { meterSpr.setColorDepth(16); meterSpr.createSprite(mInW, mInH); }
}

static void drawMicMeter(int level)
{
    if (level < 0) level = 0;
    if (level > 100) level = 100;
    if (level == s_lastMeterLevel) return;       // 没变不重画
    s_lastMeterLevel = level;
    if (meterSpr.width() == 0) return;

    const uint16_t W  = 0xFFFF;
    const uint16_t G  = tft.color565(34, 197, 94);    // #22c55e
    const uint16_t LG = tft.color565(134, 239, 172);  // 峰值高亮

    int h   = level * mInH / 100;                 // 绿色高度
    int top = mInH - h;                           // 上部未填充

    // 全在 sprite 里合成，最后一次性 pushSprite → 无闪烁
    meterSpr.fillSprite(W);
    if (h > 0) {
        meterSpr.fillRoundRect(0, 0, mInW, mInH, mInR, G);        // 整块绿
        if (top > 0) meterSpr.fillRect(0, 0, mInW, top, W);       // 盖上部为白 → 顶边平（进度线）
        if (h > 3)   meterSpr.fillRect(0, top, mInW, 3, LG);      // 峰值高亮线
    }
    meterSpr.pushSprite(mInX, mInY);
}

// ══════════════════════════════════════════════════════════════
//  BLE 配网动画：黑底 + 白蓝牙图标 + 右侧三波，波逐个变绿再全白，循环。
//    配网中（Config.isProvMode()）由 lcdTask 每帧调用。借 frameCanvas 合成防闪。
// ══════════════════════════════════════════════════════════════
static void drawBluetoothOn(lgfx::LovyanGFX& g, int cx, int cy, int h, uint16_t col)
{
    const int w  = h * 55 / 100;
    const int y0 = cy - h, y1 = cy + h, qU = cy - h / 2, qD = cy + h / 2;
    const int lx = cx - w, rx = cx + w;
    const int th = 3;                               // 线宽
    g.drawWideLine(cx, y0, cx, y1, th, col);        // 中竖轴
    g.drawWideLine(cx, y0, rx, qU, th, col);        // 顶 → 右上肩
    g.drawWideLine(rx, qU, lx, qD, th, col);        // 右上肩 → 左下（穿轴）
    g.drawWideLine(cx, y1, rx, qD, th, col);        // 底 → 右下肩
    g.drawWideLine(rx, qD, lx, qU, th, col);        // 右下肩 → 左上（穿轴）
}

static void drawProvAnim()
{
    const uint16_t WHITE = TFT_WHITE;
    const uint16_t GREEN = tft.color565(34, 197, 94);   // #22c55e
    const int W = tft.width(), H = tft.height();
    const int cx = W / 2, cy = H / 2;
    const int bth  = 30;                    // 蓝牙半高
    const int bw   = bth * 55 / 100;        // 蓝牙半宽 ≈16
    const int gap  = 18;                    // 蓝牙中心 → 波圆心间距（蓝牙与波拉开一点，不要太远）
    const int rMin = 16, rStep = 10;        // 波半径 16/26/36，厚 3
    const int rMax = rMin + 2 * rStep + 3;  // 最外波外缘半径
    // 整体水平居中：图形范围 [btx-bw, wx+rMax]（wx=btx+gap），令其中点 = cx
    const int btx = cx - (gap + rMax - bw) / 2;
    const int wx  = btx + gap;              // 波圆心（在蓝牙右侧）

    // 每 1s 多亮一个波：0→1→2→3(全绿)→0(全白)… 循环
    const int nGreen = (int)((millis() / 1000) % 4);

    const bool useCanvas = g_canvasReady && frameCanvas.width() > 0;
    lgfx::LovyanGFX& g =
        useCanvas ? *static_cast<lgfx::LovyanGFX*>(&frameCanvas)
                  : *static_cast<lgfx::LovyanGFX*>(&tft);

    g.fillScreen(TFT_BLACK);
    drawBluetoothOn(g, btx, cy, bth, WHITE);

    // 右侧三波：以 wx 为圆心的右向弧（0°=右，顺时针；-52..52 = 右向扇区），半径递增
    for (int i = 0; i < 3; i++) {
        const uint16_t col = (i < nGreen) ? GREEN : WHITE;
        const int r = rMin + i * rStep;
        g.fillArc(wx, cy, r, r + 3, -52, 52, col);
    }

    // ── 底部提示文字（白）：按配网阶段切换 ──
    //   阶段3 连 WiFi 中 → 整行"连接中"；阶段2 BLE 已连 → "请在APP配置（配置中）"；
    //   阶段1 等 BLE → "请在APP配置（剩余/120）"，倒计时与 provision 的自动关闭同一截止。
    char buf[48];
    const int stage = g_provStage;
    if (stage == 3) {
        snprintf(buf, sizeof(buf), "连接中");
    } else if (stage == 2) {
        snprintf(buf, sizeof(buf), "请在APP配置（配置中）");
    } else {
        // 阶段1 等 BLE：无 WiFi 配置（首次配网，无原配置可回退）→ 不显示倒计时；
        //   有配置（长按重配，超时会回退原网）→ 显示与自动关闭同步的倒计时。
        if (Config.getString("ssid", "").isEmpty()) {
            snprintf(buf, sizeof(buf), "请在APP配置");
        } else {
            const uint32_t dl = g_provDeadline, t = millis();
            int remain = (dl > t) ? (int)((dl - t) / 1000) : 0;
            if (remain > 120) remain = 120;
            snprintf(buf, sizeof(buf), "请在APP配置（%d/120）", remain);
        }
    }
    g.setFont(&fonts::efontCN_16);                       // 中文字库（efont，UTF-8）
    g.setTextColor(WHITE, TFT_BLACK);
    g.setTextSize(1);
    g.setTextDatum(textdatum_t::middle_center);
    g.drawString(buf, cx, cy + 70);

    // ── 设备 SN（白，配网全程显示，供用户与 App 弹窗核对是否为本机）──
    //   SN 在开机时已镜像进 Config（V1_1.cpp），配网期间恒定可读。
    char snbuf[32];
    snprintf(snbuf, sizeof(snbuf), "SN：%s", Config.getString("SN", "").c_str());
    g.drawString(snbuf, cx, cy + 94);

    g.setTextDatum(textdatum_t::top_left);               // 复位，避免影响别处绘制
    g.setFont(&fonts::Font0);

    if (useCanvas) frameCanvas.pushSprite(0, 0);
}

// ══════════════════════════════════════════════════════════════
//  配网引导画面：有 WiFi 配置但连不上时显示。左向粗白箭头（指向配网按键方向）+
//    右侧"长按2秒 / 配置设备"文字，提示用户长按 2 秒进入配网。
// ══════════════════════════════════════════════════════════════
static void drawWifiHint()
{
    const uint16_t WHITE = TFT_WHITE;
    const int W = tft.width(), H = tft.height();
    const int cy = H / 2 - 32;   // 整体上移（调这个值控制高低，越大越靠上）

    const bool useCanvas = g_canvasReady && frameCanvas.width() > 0;
    lgfx::LovyanGFX& g =
        useCanvas ? *static_cast<lgfx::LovyanGFX*>(&frameCanvas)
                  : *static_cast<lgfx::LovyanGFX*>(&tft);

    g.fillScreen(TFT_BLACK);

    // ── 左向棱角分明箭头（实心三角头 + 矩形杆，无圆角）；整体左右平滑晃动 ──
    const int cxBase = W * 26 / 100;               // 箭头基准中心 x
    const int head = 18, halfH = 17, gun = 22, thick = 12;  // 三角头长/半高、杆长/粗
    const float ph = (float)(millis() % 1200) / 1200.0f * 6.2832f;  // 1.2s 一个来回
    const int dx = (int)roundf(sinf(ph) * 7.0f);   // ±7px 正弦晃动
    const int cx0 = cxBase + dx;
    const int xTip = cx0 - (head + gun) / 2;       // 三角尖端 x（整体绕 cxBase 晃）
    // 三角头（尖朝左）：三顶点，棱角分明
    g.fillTriangle(xTip, cy, xTip + head, cy - halfH, xTip + head, cy + halfH, WHITE);
    // 矩形杆
    g.fillRect(xTip + head, cy - thick / 2, gun, thick, WHITE);

    // ── 右侧文字（两行，固定不随箭头晃）──
    g.setFont(&fonts::efontCN_16);
    g.setTextColor(WHITE, TFT_BLACK);
    g.setTextSize(1);
    g.setTextDatum(textdatum_t::middle_left);
    const int tx = cxBase + (head + gun) / 2 + 14;
    g.drawString("侧键长按2秒", tx, cy - 12);
    g.drawString("配置网络",   tx, cy + 12);
    g.setTextDatum(textdatum_t::top_left);
    g.setFont(&fonts::Font0);

    if (useCanvas) frameCanvas.pushSprite(0, 0);
}

// ══════════════════════════════════════════════════════════════
//  消息待回复：信封弹跳动画（LCD 模式3）。白身黑边+翻盖+右上红点，
//    以左下角为支点小幅跳起再落回（半正弦缓动，非线性）。每轮 ~1s，
//    之后由 lcdTask 停 2s（CPU light-sleep）再来。sprite 旋转推屏。
// ══════════════════════════════════════════════════════════════
static LGFX_Sprite envSpr(&tft);             // 信封图形
static LGFX_Sprite envArea(&tft);           // 信封活动区（局部合成，只推这一小块，避免全屏刷屏）
static int s_envPivX = 0, s_envPivY = 0;    // envSpr 内左下角支点
static int s_envDstX = 0, s_envDstY = 0;    // （诊断日志用）
static int s_envPivAX = 0, s_envPivAY = 0;  // 支点在 envArea 内坐标
static int s_areaX = 0, s_areaY = 0;        // envArea 推屏左上位置

static void envInitSprite()
{
    const int W = tft.width(), H = tft.height();
    const int ew = W * 38 / 100;             // 信封身宽（缩小；也让 sprite 更小、旋转更快）
    const int eh = ew * 62 / 100;            // 信封身高
    const int bd = (ew >= 120) ? 8 : 6;      // 黑边宽
    const int r  = eh / 5;                   // 圆角
    const int rr = eh / 5;                   // 红点半径
    const int mg = rr + 8;                   // sprite 四周余量（容红点 + 旋转）
    const int sw = ew + 2 * mg, sh = eh + 2 * mg;

    if (envSpr.width() == 0) {
        envSpr.setColorDepth(16);
        bool ok = envSpr.createSprite(sw, sh);        // 先试内部 RAM：旋转比 PSRAM 快很多
        bool psram = false;
        if (!ok) { envSpr.setPsram(true); ok = envSpr.createSprite(sw, sh); psram = true; }  // 装不下→PSRAM
        LOG("[ENV] sprite %dx%d 创建%s(%s) | 内部RAM空 %uKB | PSRAM空 %uKB\n",
            sw, sh, ok ? "成功" : "★失败", psram ? "PSRAM" : "内部RAM",
            (unsigned)(ESP.getFreeHeap()/1024), (unsigned)(ESP.getFreePsram()/1024));
    }

    const uint16_t BK = 0x0000, WT = 0xFFFF, RD = tft.color565(229, 57, 53);
    const int ex = mg, ey = mg;              // 信封身左上（sprite 内）

    envSpr.fillSprite(BK);
    envSpr.fillRoundRect(ex, ey, ew, eh, r, BK);                        // 黑（外=粗边）
    envSpr.fillRoundRect(ex+bd, ey+bd, ew-2*bd, eh-2*bd, r-bd, WT);     // 白身（内）
    const int mx = ex + ew/2, topY = ey + bd + 2, midY = ey + eh*46/100;
    envSpr.drawWideLine(ex+bd+2, topY, mx, midY, bd, BK);              // 翻盖 V 左
    envSpr.drawWideLine(mx, midY, ex+ew-bd-2, topY, bd, BK);          // 翻盖 V 右
    envSpr.fillCircle(ex+ew, ey, rr+bd/2, BK);                        // 红点黑圈
    envSpr.fillCircle(ex+ew, ey, rr, RD);                            // 红点

    s_envPivX = ex; s_envPivY = ey + eh;     // 支点=信封身左下角
    s_envDstX = W/2 - ew/2; s_envDstY = H/2 + eh/2;
    envSpr.setPivot(s_envPivX, s_envPivY);

    // 活动区合成 sprite：容纳信封旋转+跳起的小块（四周留余量），每帧只推这块，避免全屏刷屏
    const int padX = 8, padTop = 18, padBot = 8;
    const int aw = sw + 2 * padX, ah = sh + padTop + padBot;
    if (envArea.width() == 0) {
        envArea.setColorDepth(16);
        envArea.setPsram(true);
        envArea.createSprite(aw, ah);
    }
    s_envPivAX = padX + s_envPivX;            // 支点在 area 内（envSpr 在 area 内偏移 padX,padTop）
    s_envPivAY = padTop + s_envPivY;
    s_areaX = W/2 - padX - ex - ew/2;         // 让信封身居中于屏幕
    s_areaY = H/2 - padTop - ey - eh/2;
}

// 画一帧信封：借 frameCanvas 后台合成再整块推屏，消除 fillScreen 直接画屏的闪烁
static void envDrawFrame(float angle, float dy)
{
    // 只在"活动区"小 sprite 里合成（抗锯齿旋转），再把这一小块推屏——不碰全屏，无刷屏感
    if (envArea.width() == 0) return;
    envArea.fillScreen(0x0000);
    envSpr.pushRotateZoomWithAA(&envArea, (float)s_envPivAX, (float)s_envPivAY - dy, angle, 1.0f, 1.0f);
    envArea.pushSprite(s_areaX, s_areaY);
}

// 画一轮弹跳（约 1s）。angle 负=绕左下角逆时针（右边抬起）；真机若方向反改正负。
static void drawEnvelopeBounce()
{
    static uint32_t s_envLog = 0;
    if (millis() - s_envLog > 3000) {
        s_envLog = millis();
        LOG("[ENV] 信封动画循环中 (mode=%d, sprite=%dx%d, dst=%d,%d)\n",
            g_lcdPowerMode, envSpr.width(), envSpr.height(), s_envDstX, s_envDstY);
    }
    if (envSpr.width() == 0) envInitSprite();
    const uint32_t DUR = 1300;   // 一轮弹跳时长（越大越慢越柔）
    const uint32_t t0 = millis();
    while (true) {
        const uint32_t el = millis() - t0;
        if (el >= DUR) break;
        const float t = (float)el / DUR;
        float angle = 0, dy = 0;
        if (t < 0.5f) {                        // 主弹跳 0→顶→0（跳低一点）
            const float e = sinf((t / 0.5f) * PI);
            angle = -6.0f * e; dy = 8.0f * e;
        } else if (t < 0.8f) {                 // 小回弹（不变）
            const float e = sinf(((t - 0.5f) / 0.3f) * PI);
            angle = -3.0f * e; dy = 4.0f * e;
        }
        stepBrightness();                      // 弹跳期把背光渐到 60%
        envDrawFrame(angle, dy);
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(80));         // ~12fps 弹跳期
    }
    envDrawFrame(0, 0);                        // 落回静止一帧
}

// ══════════════════════════════════════════════════════════════
//  传输动画（居中）：转圈=等待(白，渐隐尾) / 打勾=完成(白勾+绿色从左扫到右)
//    由 web.cpp 写 g_xferAnim：0=无 / 1=转圈 / 2=打勾(播完自动回 0→回待机)
// ══════════════════════════════════════════════════════════════
volatile int g_xferAnim = 0;
static int   s_xferMode  = 0;          // 已初始化的模式
static float s_spinAng   = 0;          // 转圈头部角度
static uint32_t s_checkT0 = 0;
static LGFX_Sprite checkSpr(&tft);     // 打勾双缓冲（每帧从干净底重画，消涂抹/残留）
static int   chkX, chkY;               // sprite 屏幕位置
static int   lcP[3][2];                // 勾三点（sprite 内坐标）
static float cLen1, cLen2, cTotal;

#define SPIN_SEGS       48             // 弧分段数（密→连续渐变，非点状）
#define SPIN_R0         30             // 环带内半径
#define SPIN_R1         40             // 环带外半径
#define SPIN_STEP       12             // 度/帧（转速）
#define CHECK_SWEEP_MS  650
#define CHECK_HOLD_MS   500
#define CHECK_FADE_MS   450   // 停留后整体向黑渐变消失的时长

static void checkPtAt(float s, int& x, int& y) {
    if (s <= cLen1) { float t = cLen1 > 0 ? s / cLen1 : 0;
        x = lcP[0][0] + (int)((lcP[1][0]-lcP[0][0])*t); y = lcP[0][1] + (int)((lcP[1][1]-lcP[0][1])*t);
    } else { float t = cLen2 > 0 ? (s-cLen1)/cLen2 : 0;
        x = lcP[1][0] + (int)((lcP[2][0]-lcP[1][0])*t); y = lcP[1][1] + (int)((lcP[2][1]-lcP[1][1])*t); }
}

static void drawXferAnim()
{
    int cx = tft.width() / 2, cy = tft.height() / 2;

    if (g_xferAnim != s_xferMode) {          // 切换模式：初始化
        s_xferMode = g_xferAnim;
        tft.fillScreen(0x0000);
        if (g_xferAnim == 1) s_spinAng = 0;
        else if (g_xferAnim == 2) {
            const int margin = 12;
            int minx = cx - 30, miny = cy - 24;
            chkX = minx - margin; chkY = miny - margin;
            int sw = 64 + 2 * margin, sh = 50 + 2 * margin;
            lcP[0][0]=(cx-30)-chkX; lcP[0][1]=(cy+4)-chkY;   // 折线三点（sprite 内）
            lcP[1][0]=(cx-8)-chkX;  lcP[1][1]=(cy+26)-chkY;
            lcP[2][0]=(cx+34)-chkX; lcP[2][1]=(cy-24)-chkY;
            cLen1 = hypotf(lcP[1][0]-lcP[0][0], lcP[1][1]-lcP[0][1]);
            cLen2 = hypotf(lcP[2][0]-lcP[1][0], lcP[2][1]-lcP[1][1]);
            cTotal = cLen1 + cLen2;
            if (checkSpr.width() == 0) { checkSpr.setColorDepth(16); checkSpr.createSprite(sw, sh); }
            s_checkT0 = millis();
        }
    }

    if (g_xferAnim == 1) {                    // 转圈：整圈 48 段短弧，头亮尾黑渐变（顺时针，连续不点状）
        float seg = 360.0f / SPIN_SEGS;
        for (int i = 0; i < SPIN_SEGS; i++) {
            // 尾巴沿逆时针方向渐暗（拖在顺时针前进的后方）：a0 = 头部 - i*seg
            float a0 = fmodf(s_spinAng - i * seg + 360.0f, 360.0f), a1 = a0 + seg + 0.6f;
            int lv = 255 - 255 * i / SPIN_SEGS;
            tft.fillArc(cx, cy, SPIN_R0, SPIN_R1, a0, a1, tft.color565(lv, lv, lv));
        }
        s_spinAng += SPIN_STEP; if (s_spinAng >= 360) s_spinAng -= 360;   // 头部顺时针推进
    } else if (g_xferAnim == 2 && checkSpr.width() > 0) {
        uint32_t el = millis() - s_checkT0;
        float p = el < CHECK_SWEEP_MS ? (float)el / CHECK_SWEEP_MS : 1.0f;
        float L = p * cTotal;
        int ex, ey; checkPtAt(L, ex, ey);

        // ── 停留结束后淡出：颜色乘系数 f(1→0) 向黑渐变（背景是黑，等效整体淡出）──
        const uint32_t holdEnd = CHECK_SWEEP_MS + CHECK_HOLD_MS;
        float f = 1.0f;
        if (el > holdEnd) f = 1.0f - (float)(el - holdEnd) / CHECK_FADE_MS;
        if (f < 0) f = 0;
        const uint16_t Wf = tft.color565((int)(255*f), (int)(255*f), (int)(255*f));
        const uint16_t Gf = tft.color565((int)(34*f),  (int)(197*f), (int)(94*f));

        // sprite 里从干净黑底整块重画：白勾底 + 绿线到进度 L → 一次 pushSprite（无累积残留）
        checkSpr.fillSprite(0x0000);
        checkSpr.drawWideLine(lcP[0][0],lcP[0][1], lcP[1][0],lcP[1][1], 10, Wf);
        checkSpr.drawWideLine(lcP[1][0],lcP[1][1], lcP[2][0],lcP[2][1], 10, Wf);
        checkSpr.fillCircle(lcP[0][0],lcP[0][1], 5, Wf);
        checkSpr.fillCircle(lcP[1][0],lcP[1][1], 5, Wf);
        checkSpr.fillCircle(lcP[2][0],lcP[2][1], 5, Wf);
        if (L <= cLen1) {
            checkSpr.drawWideLine(lcP[0][0],lcP[0][1], ex,ey, 10, Gf);
        } else {
            checkSpr.drawWideLine(lcP[0][0],lcP[0][1], lcP[1][0],lcP[1][1], 10, Gf);
            checkSpr.drawWideLine(lcP[1][0],lcP[1][1], ex,ey, 10, Gf);
            checkSpr.fillCircle(lcP[1][0],lcP[1][1], 5, Gf);
        }
        checkSpr.fillCircle(lcP[0][0],lcP[0][1], 5, Gf);
        checkSpr.fillCircle(ex, ey, 5, Gf);
        checkSpr.pushSprite(chkX, chkY);

        if (el >= holdEnd + CHECK_FADE_MS) { g_xferAnim = 0; s_xferMode = 0; }
    }
}

// ══════════════════════════════════════════════════════════════
//  开机启动转圈（提前到 tft.init() 之后即起转，覆盖预加载 + 外设初始化黑屏）
//    独立任务 pin 到 core0：启动期该核空闲、屏幕 SPI 无其它用户，与 setup 主线程
//    (core1) 的预加载/音频/IMU/舵机初始化并行——它们都不碰 tft，无并发冲突。
//    ★ 交接：创建 lcdTask 前必须 lcdStopBootSpinner()，阻塞等本任务退出后再让
//      lcdTask 接管 tft，杜绝两任务同时写屏。
// ══════════════════════════════════════════════════════════════
static volatile bool s_bootSpinRun  = false;
static TaskHandle_t  s_bootSpinTask = nullptr;

static void bootSpinnerTask(void*)
{
    tft.fillScreen(0x0000);          // 先刷黑：点亮时不显示面板复位后的残留像素（消花屏）
    g_xferAnim = 1;                  // 复用转圈动画（drawXferAnim 的转圈分支）
    while (s_bootSpinRun) {
        stepBrightness();            // 每帧朝 s_tgtBri 渐亮 —— 转圈"从黑淡入"
        tft.startWrite();
        drawXferAnim();
        tft.endWrite();
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    g_xferAnim = 0; s_xferMode = 0;  // 交接前清转圈态，lcdTask 接管后直接进待机
    s_bootSpinTask = nullptr;        // ★ 置空=已退出、不再碰 tft，lcdStopBootSpinner 据此放行
    vTaskDelete(NULL);
}

static void lcdStartBootSpinner()
{
    if (s_bootSpinTask) return;
    s_bootSpinRun = true;
    // core0（启动期空闲）与 setup 主线程(core1)并行，转圈才持续流畅、不被阻塞初始化冻结
    xTaskCreatePinnedToCore(bootSpinnerTask, "BootSpin", 3072, NULL, 1, &s_bootSpinTask, 0);
}

void lcdStopBootSpinner()
{
    s_bootSpinRun = false;
    if (!s_bootSpinTask) return;
    // 阻塞等任务真正退出（handle 置空）后再返回——确保它已不再碰 tft，lcdTask 才安全接管
    uint32_t t0 = millis();
    while (s_bootSpinTask && millis() - t0 < 1000) vTaskDelay(pdMS_TO_TICKS(5));
}

void lcdTask(void *lcdParameter)
{
    LOG("[LCD] Task 启动\n");
    esp_task_wdt_add(NULL);   // 纳入 TWDT：本任务 5s 内不 reset 即 panic 点名（各阻塞点 <5s）

    // ── 开机转圈已由 bootSpinnerTask 提前（tft.init 后即起转）覆盖整个启动黑屏，并把
    //    背光从全灭渐亮到 s_tgtBri。此处不再重复转圈，直接进主循环接待机；此时
    //    s_curBri 已 == s_tgtBri，主循环首帧 stepBrightness 不跳变，无暗闪。──

    MessageToLCD_t rxMsg;
    bool msgPending  = false;
    int  playCounter = 0;

    uint32_t frameCount    = 0;
    uint32_t fpsWindowStart = millis();

    // 持有 SPI 总线：连续帧之间不释放 CS，减少开销
    uint32_t frameStartTime;
    
    while (true) {
        esp_task_wdt_reset();   // 喂狗：每帧一次（含 L2 阻塞 500ms 唤醒后回到这里）
        healthSetTaskStage(HT_LCD, HS_L_IDLE);   // 每帧回到这里重置，下面各阶段再覆盖
        frameStartTime = millis();

        // ── 接收消息 ──
        // ★ 先收进临时变量，确认要采纳了才写进 rxMsg：rxMsg 只有一个槽位，直接收进去
        //   会把尚未处理的上一条无声顶掉（见下面 wink 的优先级判断）。
        MessageToLCD_t inMsg;
        if (xQueueReceive(qMainToLcd, &inMsg, 0) == pdTRUE) {
            switch (inMsg.cmd) {
                case LCDMSG_PLAY:
                case LCDMSG_PLAY_NONINT:
                    // ★ wink（拍一拍已送达）只是锦上添花的反馈，优先级低于点头动画本身。
                    //   场景：正播好友视频(NONINT)时用户拍了一下 → 点头挂在 msgPending 里
                    //   等视频播完；此时伙伴的 POKE_ACK 回来触发 wink，若直接收进 rxMsg
                    //   就会把还没轮到播的点头顶掉，变成"只眨眼、不点头"。
                    //   宁可这次不眨眼，也不能吞掉点头。
                    if (inMsg.motion == MOTION_WINK && msgPending) {
                        LOG("[LCD] 已有待播动画，丢弃 wink（点头优先）\n");
                        break;
                    }
                    // ★ 摇一摇去重：正在播放摇摆动画期间，忽略后续摇摆——既不打断当前、也不排队重播。
                    //   摇一摇会由 SERVO/IMU/P3 镜像连发多条触发；在池/播放状态的属主任务 lcdTask 里
                    //   原子判定丢弃，兜住主循环侧 !lcdIsPlaying() 守卫因跨任务状态滞后而漏进来的那几条。
                    if (inMsg.motion == MOTION_SHAKE &&
                        (lcdState == LCD_PLAYING || lcdState == LCD_PLAYING_NONINT) &&
                        strcmp(playerInfo->videoPath, LCD_MOTION_SHAKE_PATH) == 0) {
                        LOG("[LCD] 摇摆播放中，忽略新的摇摆\n");
                        break;
                    }
                    rxMsg      = inMsg;
                    msgPending = true;
                    break;
                case LCDMSG_CFG_UPDATE: {
                    // 亮度基准变了：强制按"当前功耗模式×系数"重算（下一轮应用）
                    g_lcdAppliedMode = -1;
                    LOG("[LCD] 配置更新：亮度将按当前档位重算\n");
                    break;
                }
                default: break;
            }
        }

        // ── 检查 target 视频热重载（旧的 flash 重载路径，保留兼容）──
        if (videoUpdatePending) {
            videoUpdatePending = false;
            reloadTargetVideo();
        }

        // ── 处理 MQTT 直收的 commit/abort（必须始终运行，否则传输 commit 会卡死）──
        lcdProcessTargetRecv();

        // ── 应用功耗模式（仅变化时）──
        if (g_lcdPowerMode != g_lcdAppliedMode) lcdApplyPowerMode(g_lcdPowerMode);

        // ── 背光渐变：每帧朝目标逼近（睡眠渐暗 / 唤醒渐亮 / 亮度分级平滑）──
        stepBrightness();

        // ── L2 且背光已渐暗到 0 → 真正面板休眠+阻塞；渐暗过程中仍渲染(画睡眠动画) ──
        if (g_lcdPowerMode == 2 && s_curBri == 0 && !s_panelSlept) {
            tft.sleep();
            s_panelSlept = true;
        }
        bool renderPaused = (g_lcdPowerMode == 2 && s_panelSlept);

        if (!renderPaused) {
            // ── BLE 配网中：最高优先，全屏。阶段4=配网成功 → 播打勾(带淡出)，否则蓝牙+三波 ──
            if (Config.isProvMode()) {
                if (g_provStage == 4) drawXferAnim();   // 成功：复用打勾动画（g_xferAnim=2 由 provision 触发）
                else                  drawProvAnim();
                vTaskDelay(pdMS_TO_TICKS(60));
                continue;
            }

            // ── 有配置但连不上：显示配网引导画面（箭头左右平滑晃动，~20fps）──
            if (g_needWifiHint) {
                drawWifiHint();
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            // ── 消息待回复（模式3）：信封弹跳一轮 → 停 2s 浅睡（xLcdWake 唤醒可提前退出）──
            if (g_lcdPowerMode == 3) {
                drawEnvelopeBounce();
                xSemaphoreTake(xLcdWake, pdMS_TO_TICKS(2000));
                continue;
            }

            // ── 录音音量表：优先，全屏替换（矢量绘制，不走 MJPEG）──
            if (g_micMeterActive) {
                tft.startWrite();
                if (!s_meterInited) { drawMicIconStatic(); s_meterInited = true; s_lastMeterLevel = -1; }
                drawMicMeter(g_micLevel);
                tft.endWrite();
                vTaskDelay(pdMS_TO_TICKS(30));   // ~33Hz，跟手；矢量绘制很快
                continue;
            }
            s_meterInited = false;   // 已退出音量表：下一帧 player() 画待机覆盖恢复

            // ── 传输动画：转圈(等待)/打勾(完成)，全屏居中 ──
            if (g_xferAnim) {
                tft.startWrite();
                drawXferAnim();
                tft.endWrite();
                vTaskDelay(pdMS_TO_TICKS(45));
                continue;
            }
            s_xferMode = 0;   // 已退出传输动画：下次进入重新 init

            // ── 下载期(App→设备)：冻结当前帧，接收方【无感】──
            //   不解码 MJPEG、不改屏：屏幕停在原样，看不出在下载；把 core0 全让给 webTask 收包，
            //   修"边播视频边下载→视频极慢+下载失败"。lcdProcessTargetRecv() 在循环顶部照常跑，
            //   视频收齐照常提交；下载结束(g_xfering=0)自动恢复状态机继续播放/待机。
            if (g_xfering) {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            tft.startWrite();
            // ── 状态机 ──
            switch (lcdState) {

                case LCD_FREE:
                    player();
                    frameCount++;
                    if (msgPending) processMessage(rxMsg, msgPending, playCounter);
                    break;

                case LCD_PLAYING:
                    playCounter += player();
                    frameCount++;
                    if (msgPending) processMessage(rxMsg, msgPending, playCounter);
                    if (playerInfo->playTimes > 0 && playCounter >= playerInfo->playTimes) {
                        if (strcmp(playerInfo->videoPath, LCD_STARTSLEEP_PATH) == 0)
                            switchToLoop(LCD_SLEEPING_PATH);   // startSleep 播完 → sleeping 循环
                        else
                            switchToFree();                    // 其余 → idle
                        playCounter = 0;
                    }
                    break;

                case LCD_PLAYING_NONINT:
                    playCounter += player();
                    frameCount++;
                    if (playerInfo->playTimes > 0 && playCounter >= playerInfo->playTimes) {
                        if (msgPending) processMessage(rxMsg, msgPending, playCounter);
                        else { switchToFree(); playCounter = 0; }
                    }
                    break;

                case LCD_ERROR:
                    tft.endWrite();
                    handleErrorState();
                    if (lcdState != LCD_ERROR) tft.startWrite();
                    break;

                case LCD_OFF:
                    tft.endWrite();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    tft.startWrite();
                    break;
            }

            // ── 回退：仅当帧缓冲创建失败时，才在屏上直接叠加（可能会闪）；
            //    正常情况下电量已在 player() 内合成进帧缓冲，这里跳过 ──
            if (!g_canvasReady &&
                currentSlot >= 0 && currentSlot != LCD_TARGET_SLOT)
                drawOverlaysOn(tft);   // 电量 + 配对爱心（降级直绘，可能会闪）

            tft.endWrite();

            // ── FPS 统计（每秒）──
            uint32_t now = millis();
            uint32_t elapsed = now - fpsWindowStart;
            if (elapsed >= 1000) {
                stats.fps = frameCount * 1000.0f / elapsed;
                if (stats.fps > stats.fpsMax) stats.fpsMax = stats.fps;
                frameCount    = 0;
                fpsWindowStart = now;

                LOG("[LCD] FPS: %5.1f | avg: %2dms | max: %2dms | %s | %s\n",
                    stats.fps,
                    stats.avgDecodeMs,
                    stats.maxDecodeMs,
                    stats.isPreloaded ? "PSRAM" : "SD",
                    stats.currentFile);
            }

            // ── 帧间调度 ──
            //   正常：固定帧率——补偿解码用时，凑够 FRAME_MS 维持流畅（慢帧只让 1 tick）。
            //   下载/存盘：固定延迟——不看解码用了多久，每帧后无条件让出 LCD_XFER_DELAY_MS，
            //   保证 core0 稳定分给 webTask 收包 / persistTask 落盘，不被 LCD 慢帧连续解码饿死。
            if (g_xfering || g_persistWriting) {
                vTaskDelay(pdMS_TO_TICKS(LCD_XFER_DELAY_MS));   // 固定延迟，与解码用时无关
            } else {
                uint32_t frameUsedTime = millis() - frameStartTime;
                if (frameUsedTime < FRAME_MS) {
                    vTaskDelay(pdMS_TO_TICKS(FRAME_MS - frameUsedTime));
                } else {
                    vTaskDelay(1);   // 慢帧保底让出 ≥1 tick：喂 IDLE0 + 不饿死同核任务
                }
            }
        } else {
            // ── L2：屏幕休眠，停止解码/推屏；阻塞等唤醒（喂 light-sleep）──
            //    离开 L2 时被 give xLcdWake 瞬时唤醒；带超时兜底，超时回循环顶部
            //    仍会跑一次 lcdProcessTargetRecv()（极端情况下的直收兜底）。
            fpsWindowStart = millis();
            frameCount     = 0;
            stats.fps      = 0;
            xSemaphoreTake(xLcdWake, pdMS_TO_TICKS(LCD_L2_BLOCK_MS));
        }
    }
}

// ══════════════════════════════════════════════════════════════
//  公开 API
// ══════════════════════════════════════════════════════════════

const lcdStats_t* lcdGetStats()
{
    return &stats;
}

void lcdNotifyVideoUpdated()
{
    videoUpdatePending = true;
    LOG("[LCD] 收到视频更新通知\n");
}

void backLightON()
{
    int b = Config.getInt("brightness", 255);
    tft.setBrightness((uint8_t)b);
    s_curBri = s_tgtBri = b;   // 同步渐变状态，避免 lcdTask 恢复后又跳变
}

void backLightOFF()
{
    tft.setBrightness(0);
    s_curBri = s_tgtBri = 0;
}