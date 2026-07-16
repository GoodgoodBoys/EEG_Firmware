/**
 * @file nvs_sn.h
 * @brief NVS Serial Number (SN) 读写与校验模块 — Arduino 版
 *        适用于 ESP32-S3-WROOM（Arduino ESP32 Core >= 2.0）
 *
 * 底层使用 Arduino Preferences 库（对 ESP-IDF NVS 的封装）
 *
 * SN 格式规范：
 *   - 固定 8 位纯数字字符串
 *   - 示例：'00000001'
 *   - NVS 命名空间："device_info"，Key："serial_num"
 */

#pragma once

#include <Arduino.h>
#include <Preferences.h>

/* ─── 配置常量 ─────────────────────────────────────────────── */

/** NVS 命名空间（最长 15 字节） */
#define NVS_SN_NAMESPACE  "device_info"

/** NVS Key（最长 15 字节） */
#define NVS_SN_KEY        "serial_num"

/** SN 固定长度（不含 '\0'） */
#define SN_LENGTH         8

/** 出厂初始 SN */
#define SN_DEFAULT        "00000001"

/* ─── 错误码 ─────────────────────────────────────────────────*/

enum class SnError : uint8_t {
    OK              = 0,  ///< 操作成功
    NULL_PTR        = 1,  ///< 指针为空
    WRONG_LENGTH    = 2,  ///< 长度不为 SN_LENGTH
    INVALID_CHAR    = 3,  ///< 含非数字字符
    NVS_OPEN_FAIL   = 4,  ///< NVS 打开失败
    NVS_WRITE_FAIL  = 5,  ///< NVS 写入失败
    NVS_READ_FAIL   = 6,  ///< NVS 读取失败
    NOT_FOUND       = 7,  ///< NVS 中无该 Key
    VERIFY_FAIL     = 8,  ///< 写回校验不一致
};

/** 将 SnError 转为可读字符串 */
const char* snErrorToStr(SnError err);

/* ─── NvsSn 类 ───────────────────────────────────────────────*/

class NvsSn {
public:
    /**
     * @brief 构造函数
     * @param ns   NVS 命名空间（默认 NVS_SN_NAMESPACE）
     * @param key  NVS Key（默认 NVS_SN_KEY）
     */
    NvsSn(const char* ns = NVS_SN_NAMESPACE,
          const char* key = NVS_SN_KEY);

    /**
     * @brief 校验 SN 字符串格式（不操作 NVS）
     *
     * 规则：非空 → 长度 == SN_LENGTH → 每位 '0'~'9'
     *
     * @param sn  待校验字符串
     * @return SnError::OK 或具体错误码
     */
    SnError validate(const String& sn) const;

    /**
     * @brief 将 SN 写入 NVS
     *
     * 流程：写前校验 → NVS 写入 → 写回校验
     *
     * @param sn  待写入 SN（8 位纯数字）
     * @return SnError::OK 或具体错误码
     */
    SnError write(const String& sn);

    /**
     * @brief 从 NVS 读取 SN
     *
     * 读出后执行格式校验；若 Key 不存在返回 SnError::NOT_FOUND
     *
     * @param[out] outSn  存放读取结果的 String
     * @return SnError::OK 或具体错误码
     */
    SnError read(String& outSn) const;

    /**
     * @brief 开机初始化：无记录则写入默认 SN，有记录则校验
     *
     * @param defaultSn  出厂默认 SN（默认 SN_DEFAULT "00000001"）
     * @return SnError::OK 或具体错误码
     */
    SnError initDefault(const String& defaultSn = SN_DEFAULT);

private:
    const char* _ns;   ///< NVS 命名空间
    const char* _key;  ///< NVS Key
};
