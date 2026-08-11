/**
 * clearCfg.ino  —  清除 EGG 设备的【配置】，保留 SN（独立 Arduino sketch）
 * =============================================================
 *  作用：清空正式固件存配置用的 NVS 命名空间 "egg_cfg"
 *        （ssid/password/matchCode/peerSn/terminal/亮度/音量/勿扰），
 *        把设备恢复到「有 SN、无配置」的全新态，方便反复测试。
 *
 *  ★ 只清配置，不动 SN：
 *      配置在 NVS 命名空间 "egg_cfg"，SN 在 "device_info"（见 nvs_sn）。
 *      本 sketch 只 clear "egg_cfg"，SN 原样保留，无需重烧。
 *
 *  ★ 还要清 LittleFS 的旧配置文件！
 *      正式固件带一次性迁移逻辑：若 NVS 无配置、LittleFS 有旧 /Config/config.json，
 *      开机会把旧配置搬回 NVS（复活）。所以只烧本 sketch 还不够，必须再跑一次
 *      `pio run -t uploadfs` 把 LittleFS 重刷掉（data/ 里没有 /Config，等于删除）。
 *      —— 本 sketch 不删 LittleFS，是因为 Arduino IDE 默认分区表的 spiffs 偏移/大小
 *         与本工程自定义分区(0x710000, 9MB)不一致，在这里删会删错区域。交给 uploadfs 最稳。
 *
 *  —— 用法 ——
 *   1. Arduino IDE 选：ESP32S3 Dev Module / 16MB / 含默认 nvs(0x9000) 的分区方案
 *      / USB CDC On Boot: Enabled。
 *   2. 烧录本 sketch，串口(115200)看到 “✓ 配置已清除（SN 保留）”。
 *   3. 回 PlatformIO：`pio run -t uploadfs`（清 LittleFS 旧配置）→ `pio run -t upload`
 *      （烧回正式固件）。开机即全新态，会自动进配网。
 *   ★ 全程不要 erase_flash / pio run -t erase（那会连 SN 一起抹）。
 * =============================================================
 */

#include <Arduino.h>
#include <Preferences.h>

// 必须与正式固件 configSys.cpp 的 CFG_NVS_NS 完全一致
#define CFG_NVS_NAMESPACE "egg_cfg"

static void line() { Serial.println("-------------------------------------"); }

void setup() {
  Serial.begin(115200);
  delay(600);  // 等 USB CDC 就绪
  Serial.println();
  line();
  Serial.println(" EGG  配置清除工具（保留 SN）");
  line();

  Preferences p;
  if (!p.begin(CFG_NVS_NAMESPACE, false)) {
    // 命名空间不存在（从没存过配置）也算已是干净态
    Serial.println(" 命名空间 'egg_cfg' 不存在 → 本就无配置，无需清除");
    line();
    return;
  }
  bool ok = p.clear();   // 清空整个 egg_cfg 命名空间；不影响 device_info(SN)
  p.end();

  line();
  if (ok) {
    Serial.println(" ✓ 配置已清除（SN 保留）");
    Serial.println(" 下一步：pio run -t uploadfs（清 LittleFS 旧配置）→ pio run -t upload");
  } else {
    Serial.println(" ✗ 清除失败（p.clear() 返回 false）");
  }
  line();
}

void loop() { delay(2000); }
