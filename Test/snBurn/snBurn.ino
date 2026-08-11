/**
 * snBurn.ino  —  给 EGG 设备预烧录 SN（独立 Arduino sketch）
 * =============================================================
 *  作用：把一个 8 位纯数字 SN 写进 NVS，位置/格式与正式固件
 *        (inc/nvs_sn.h + src/nvs_sn.cpp) 完全一致，写完即可烧正式固件使用。
 *
 *  NVS 存储约定（务必与固件一致，别改）：
 *    命名空间 = "device_info"
 *    Key      = "serial_num"
 *    值        = 8 位纯数字字符串，如 "00000002"
 *
 *  —— 用法 ——
 *   1. 改下面的 TARGET_SN 为这台设备要烧的 SN（8 位数字）。
 *   2. Arduino IDE 里选：
 *        开发板   : ESP32S3 Dev Module
 *        Flash Size: 16MB (128Mb)
 *        Partition : 任意含默认 nvs 的方案即可（默认 nvs 都在 0x9000，
 *                    与固件 partitions.csv 一致，SN 不会错位）
 *        USB CDC On Boot: Enabled（否则串口看不到日志）
 *   3. 烧录本 sketch，打开串口监视器(115200)，看到 “✓ SN 烧录成功” 即可。
 *   4. 之后用 PlatformIO 正常烧正式固件（普通 upload 不会擦除 NVS，SN 保留）。
 *      ★ 不要执行 “erase_flash / pio run -t erase”，那会连 NVS 一起抹掉。
 *      顺序也可反过来：先烧正式固件、再烧本 sketch 写 SN，同样可行。
 * =============================================================
 */

#include <Arduino.h>
#include <Preferences.h>

// ★★★ 改这里：这台设备要烧的 SN（必须 8 位纯数字）★★★
#define TARGET_SN "00000005"

// —— 下面与固件保持一致，别动 ——
#define NVS_SN_NAMESPACE "device_info"
#define NVS_SN_KEY "serial_num"
#define SN_LENGTH 8

// 校验：非空、长度==8、每位 '0'~'9'（与固件 NvsSn::validate 同规则）
static bool validSn(const String& sn) {
  if (sn.length() != SN_LENGTH) return false;
  for (int i = 0; i < SN_LENGTH; i++)
    if (sn[i] < '0' || sn[i] > '9') return false;
  return true;
}

static void line() {
  Serial.println("-------------------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(600);  // 等 USB CDC 就绪
  Serial.println();
  line();
  Serial.println(" EGG  SN 预烧录工具");
  line();

  String target = String(TARGET_SN);

  // ① 目标 SN 合法性
  if (!validSn(target)) {
    Serial.printf(" ✗ TARGET_SN 非法: '%s'（必须 8 位纯数字）\n", target.c_str());
    Serial.println(" 请修改 sketch 顶部 TARGET_SN 后重新烧录。");
    line();
    return;
  }

  // ② 打开 NVS（读写），先读旧值给个提示
  Preferences prefs;
  if (!prefs.begin(NVS_SN_NAMESPACE, false)) {
    Serial.println(" ✗ 打开 NVS 命名空间失败");
    line();
    return;
  }
  if (prefs.isKey(NVS_SN_KEY)) {
    String old = prefs.getString(NVS_SN_KEY, "");
    Serial.printf(" 已有 SN: '%s' → 将覆写为 '%s'\n", old.c_str(), target.c_str());
  } else {
    Serial.printf(" 无旧 SN → 首次写入 '%s'\n", target.c_str());
  }

  // ③ 写入
  size_t n = prefs.putString(NVS_SN_KEY, target);
  prefs.end();
  if (n == 0) {
    Serial.println(" ✗ 写入失败（putString 返回 0）");
    line();
    return;
  }
  Serial.printf(" 已写入 %u 字节\n", (unsigned)n);

  // ④ 重新只读打开，回读校验
  Preferences rp;
  if (!rp.begin(NVS_SN_NAMESPACE, true)) {
    Serial.println(" ✗ 回读打开失败");
    line();
    return;
  }
  String back = rp.getString(NVS_SN_KEY, "");
  rp.end();

  line();
  if (back == target && validSn(back)) {
    Serial.printf(" ✓ SN 烧录成功: '%s'\n", back.c_str());
    Serial.println(" 现在可以烧正式固件了（普通 upload，勿 erase）。");
  } else {
    Serial.printf(" ✗ 回读校验失败: 期望 '%s'，实际 '%s'\n",
                  target.c_str(), back.c_str());
  }
  line();
}

void loop() {
  // 烧录是一次性的，这里空转即可。
  delay(2000);
}
