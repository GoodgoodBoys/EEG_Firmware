#ifndef _MP3ENC
#define _MP3ENC

#include <stdint.h>
#include <stddef.h>

// Shine MP3 编码器的薄封装。
// 目的：把 Shine（layer3.h）的 C 枚举（NONE/STEREO/... 会泄漏到全局命名空间，
// 跟 msg.hpp 里的 NONE 冲突）隔离在 mp3enc.cpp 内部，对外只暴露不透明句柄。
// 只做单声道语音编码这一种用途。

// 一帧最多的采样数（对应 Shine 的 SHINE_MAX_SAMPLES=1152），调用方按此开缓冲
#define MP3ENC_MAX_SAMPLES 1152

struct Mp3Enc;   // 不透明句柄

// 打开单声道编码器；参数非法或内存不足返回 nullptr
Mp3Enc* mp3encOpen(int sampleRate, int bitrateKbps);

// 每次编码需要喂入的采样数
int mp3encSamplesPerPass(Mp3Enc* e);

// 编码一帧（mp3encSamplesPerPass 个 int16 单声道采样）。
// 返回内部缓冲指针 + 本次产出的 MP3 字节数（*written，可能为 0）。
const uint8_t* mp3encFrame(Mp3Enc* e, int16_t* frame, int* written);

// 冲刷编码器缓冲，返回最后残留的 MP3 字节（关闭前调用一次）
const uint8_t* mp3encFlush(Mp3Enc* e, int* written);

// 关闭并释放
void mp3encClose(Mp3Enc* e);

#endif // _MP3ENC
