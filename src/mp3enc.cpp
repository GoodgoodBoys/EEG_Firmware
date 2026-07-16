// Shine MP3 编码器封装的实现。
// 本文件是唯一 include layer3.h 的地方——把它那些会污染全局命名空间的枚举
// （NONE/STEREO/MONO...）关在这里，不外泄给 msg.hpp 等其它模块。
#include <new>            // std::nothrow

extern "C" {
#include "layer3.h"       // vendored 在 lib/shine（LGPL v2）
}

#include "inc/mp3enc.hpp"

struct Mp3Enc {
    shine_t s;
};

Mp3Enc* mp3encOpen(int sampleRate, int bitrateKbps) {
    if (shine_check_config(sampleRate, bitrateKbps) < 0) return nullptr;

    shine_config_t cfg;
    shine_set_config_mpeg_defaults(&cfg.mpeg);
    cfg.mpeg.mode       = MONO;
    cfg.mpeg.bitr       = bitrateKbps;
    cfg.wave.channels   = PCM_MONO;
    cfg.wave.samplerate = sampleRate;

    shine_t s = shine_initialise(&cfg);
    if (!s) return nullptr;

    Mp3Enc* e = new (std::nothrow) Mp3Enc;
    if (!e) { shine_close(s); return nullptr; }
    e->s = s;
    return e;
}

int mp3encSamplesPerPass(Mp3Enc* e) {
    return e ? shine_samples_per_pass(e->s) : 0;
}

const uint8_t* mp3encFrame(Mp3Enc* e, int16_t* frame, int* written) {
    return shine_encode_buffer_interleaved(e->s, frame, written);
}

const uint8_t* mp3encFlush(Mp3Enc* e, int* written) {
    return shine_flush(e->s, written);
}

void mp3encClose(Mp3Enc* e) {
    if (!e) return;
    shine_close(e->s);
    delete e;
}
