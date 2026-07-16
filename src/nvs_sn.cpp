/**
 * @file nvs_sn.cpp
 * @brief NVS SN 读写与校验实现 — Arduino 版
 *        适用于 ESP32-S3-WROOM（Arduino ESP32 Core >= 2.0）
 */

#include "inc/nvs_sn.h"
#include "inc/debug.hpp"   // ★ 放在 nvs_sn.h(含 Preferences.h) 之后，使 LOG / _DEBUG 生效

/* ─────────────────────────────────────────────────────────────
 * 错误码字符串映射
 * ───────────────────────────────────────────────────────────── */
const char* snErrorToStr(SnError err)
{
    switch (err) {
        case SnError::OK:            return "OK";
        case SnError::NULL_PTR:      return "NULL_PTR";
        case SnError::WRONG_LENGTH:  return "WRONG_LENGTH";
        case SnError::INVALID_CHAR:  return "INVALID_CHAR";
        case SnError::NVS_OPEN_FAIL: return "NVS_OPEN_FAIL";
        case SnError::NVS_WRITE_FAIL:return "NVS_WRITE_FAIL";
        case SnError::NVS_READ_FAIL: return "NVS_READ_FAIL";
        case SnError::NOT_FOUND:     return "NOT_FOUND";
        case SnError::VERIFY_FAIL:   return "VERIFY_FAIL";
        default:                     return "UNKNOWN";
    }
}

/* ─────────────────────────────────────────────────────────────
 * 内部日志助手
 * ───────────────────────────────────────────────────────────── */
static void logBanner(const char* title)
{
    LOG("─────────────────────────────────────\n");
    LOG(" %s\n", title);
    LOG("─────────────────────────────────────\n");
}

/* ─────────────────────────────────────────────────────────────
 * NvsSn 构造
 * ───────────────────────────────────────────────────────────── */
NvsSn::NvsSn(const char* ns, const char* key)
    : _ns(ns), _key(key)
{}

/* ─────────────────────────────────────────────────────────────
 * validate()
 * ───────────────────────────────────────────────────────────── */
SnError NvsSn::validate(const String& sn) const
{
    /* 1. 空字符串（Arduino String 无 NULL 指针概念，检查 isEmpty） */
    if (sn.isEmpty()) {
        LOG("[NVS_SN][校验] SN 为空字符串\n");
        return SnError::NULL_PTR;
    }

    /* 2. 长度必须恰好为 SN_LENGTH */
    if ((int)sn.length() != SN_LENGTH) {
        LOG("[NVS_SN][校验] 长度错误: 实际 %d 位，要求 %d 位\n",
            sn.length(), SN_LENGTH);
        return SnError::WRONG_LENGTH;
    }

    /* 3. 每位字符必须在 '0'~'9' */
    for (int i = 0; i < SN_LENGTH; i++) {
        if (sn[i] < '0' || sn[i] > '9') {
            LOG("[NVS_SN][校验] 非法字符: 位置[%d]='%c'(0x%02X)\n",
                i, sn[i], (uint8_t)sn[i]);
            return SnError::INVALID_CHAR;
        }
    }

    LOG("[NVS_SN][校验] SN='%s' → PASS\n", sn.c_str());
    return SnError::OK;
}

/* ─────────────────────────────────────────────────────────────
 * write()
 * ───────────────────────────────────────────────────────────── */
SnError NvsSn::write(const String& sn)
{
    logBanner("SN 写入");

    /* ① 写前格式校验 */
    SnError ret = validate(sn);
    if (ret != SnError::OK) {
        LOG("[NVS_SN][写入] 预校验失败，中止写入\n");
        return ret;
    }

    /* ② 打开 NVS（读写模式，false = 非只读） */
    Preferences prefs;
    if (!prefs.begin(_ns, false)) {
        LOG("[NVS_SN][写入] 打开命名空间 '%s' 失败\n", _ns);
        return SnError::NVS_OPEN_FAIL;
    }

    /* ③ 写入字符串 */
    size_t written = prefs.putString(_key, sn);
    prefs.end();

    if (written == 0) {
        LOG("[NVS_SN][写入] putString 返回 0，写入失败\n");
        return SnError::NVS_WRITE_FAIL;
    }
    LOG("[NVS_SN][写入] Flash 写入完成（%d bytes）\n", written);

    /* ④ 写回校验：读出并逐字节比对 */
    String verify;
    ret = read(verify);
    if (ret != SnError::OK) {
        LOG("[NVS_SN][写入] 写回读取失败: %s\n", snErrorToStr(ret));
        return ret;
    }

    if (verify != sn) {
        LOG("[NVS_SN][写入] 写回校验 FAIL: 期望='%s'，实际='%s'\n",
            sn.c_str(), verify.c_str());
        return SnError::VERIFY_FAIL;
    }

    LOG("[NVS_SN][写入] 写回校验 PASS: '%s'\n", verify.c_str());
    return SnError::OK;
}

/* ─────────────────────────────────────────────────────────────
 * read()
 * ───────────────────────────────────────────────────────────── */
SnError NvsSn::read(String& outSn) const
{
    logBanner("SN 读取");

    /* ① 打开 NVS（只读模式，true = 只读） */
    Preferences prefs;
    if (!prefs.begin(_ns, true)) {
        /*
         * 只读模式下，命名空间不存在时 begin() 返回 false。
         * 这等价于 NVS_NOT_FOUND。
         */
        LOG("[NVS_SN][读取] 命名空间 '%s' 不存在（首次上电？）\n", _ns);
        prefs.end();
        return SnError::NOT_FOUND;
    }

    /* ② 检查 Key 是否存在 */
    if (!prefs.isKey(_key)) {
        LOG("[NVS_SN][读取] Key '%s' 不存在\n", _key);
        prefs.end();
        return SnError::NOT_FOUND;
    }

    /* ③ 读取字符串 */
    outSn = prefs.getString(_key, "");
    prefs.end();

    if (outSn.isEmpty()) {
        LOG("[NVS_SN][读取] 读出空字符串，可能存储异常\n");
        return SnError::NVS_READ_FAIL;
    }

    LOG("[NVS_SN][读取] 原始数据='%s'\n", outSn.c_str());

    /* ④ 读出后格式校验 */
    return validate(outSn);
}

/* ─────────────────────────────────────────────────────────────
 * initDefault()
 * ───────────────────────────────────────────────────────────── */
SnError NvsSn::initDefault(const String& defaultSn)
{
    logBanner("SN 初始化");
    String existing;
    SnError ret = read(existing);

    if (ret == SnError::NOT_FOUND) {
        /* 首次上电，无记录 → 写入出厂默认值 */
        LOG("[NVS_SN][初始化] 首次写入出厂 SN: '%s'\n", defaultSn.c_str());
        return write(defaultSn);
    }

    if (ret != SnError::OK) {
        /* 有记录但格式损坏 → 强制覆写 */
        LOG("[NVS_SN][初始化] SN 数据异常 (%s)，强制覆写为 '%s'\n",
            snErrorToStr(ret), defaultSn.c_str());
        return write(defaultSn);
    }

    /* 有记录且合法 → 直接使用 */
    LOG("[NVS_SN][初始化] 现有 SN='%s'，无需覆写\n", existing.c_str());
    return SnError::OK;
}