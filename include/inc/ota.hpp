#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════════════
//  OTA（空中升级）—— A/B 双分区 + 回滚 + 纯净下载环境
//
//  设计要点（与讨论一致）：
//   · 触发：设备订阅【官方版本 Topic】(retained)，版本号 > 本机 FW_VERSION 且未被放弃 → 升级。
//   · 纯净环境：置 NVS 标记 → esp_restart → setup 最早期检测标记 → 只初始化
//     NVS/WiFi/LCD(轻量)/HTTPS 下载固件，不启动音频/IMU/BLE/低功耗等，内存最宽裕。
//   · 传输：HTTPS 从阿里云 OSS 拉固件，边下边算 SHA256；【校验通过才切启动分区】。
//   · A/B + 回滚：写另一分区 → set_boot → 重启进新固件 → 跑到运行期=启动成功→mark_valid；
//     新固件启动即崩 → bootloader 自动回滚旧分区（需 sdkconfig ROLLBACK_ENABLE）。
//   · 失败上限：连续 OTA_MAX_FAIL 次失败 → 放弃该版本、退回旧固件，等更新的版本再试。
//   · 安全（基础）：HTTPS(crt_bundle 验证) + SHA256(完整性) + 官方 Topic ACL + URL 域名白名单。
// ══════════════════════════════════════════════════════════════════════════

// ┌────────────────────────────────────────────────────────────────────────┐
// │  ★★★ 待你填写 / 确认的配置（现在留占位；建好 OSS + 定好 Topic 后改这里）★★★  │
// └────────────────────────────────────────────────────────────────────────┘

// 当前固件版本：递增整数（发布新固件时 +1，或用 YYYYMMDDNN）。OTA 靠它与官方 Topic
// 上的版本号比较决定是否升级。大小可比 = 单调递增是硬要求。
#define FW_VERSION            10001

// 官方版本 Topic（retained）。设备订阅它拿最新版本号。
// ★ EMQX 上务必配 ACL：只有【你的官方账号】能 publish 到它，设备只能 subscribe。
//   否则任何人往这个 Topic 发个假版本 + 恶意 URL，所有设备就会去下恶意固件。
#define OTA_OFFICIAL_TOPIC    "ota/release"

// OSS 固件基址（末尾【不带】斜杠）。完整地址由版本号拼出：
//   固件  = OTA_URL_BASE "/" <ver> "/firmware.bin"
//   校验  = OTA_URL_BASE "/" <ver> "/firmware.sha256"（内容为 64 位小写 hex）
// ★ 待你建好华南深圳、公共读的 OSS bucket 后填，形如：
//   https://your-bucket.oss-cn-shenzhen.aliyuncs.com/fw
#define OTA_URL_BASE          "https://egg-ota-test.oss-cn-beijing.aliyuncs.com/fw"

// URL 域名白名单：设备只接受该 host 开头的地址（防官方 Topic 被篡改指向别处）。
// ★ 必须与上面 bucket 的域名一致（只填 host，不含协议/路径）。
#define OTA_URL_HOST          "egg-ota-test.oss-cn-beijing.aliyuncs.com"

// 连续失败上限：达到即放弃该版本、退回旧固件，直到官方发布更新的版本再试。
#define OTA_MAX_FAIL          5

// ┌────────────────────────────────────────────────────────────────────────┐
// │  对外接口                                                                  │
// └────────────────────────────────────────────────────────────────────────┘

// setup() 最早期调用（关机休眠判定之后、常规外设初始化之前）。
//  · NVS 标记为 PENDING → 进入纯净 OTA 下载环境（不返回，内部 esp_restart）。
//  · 上次新固件启动失败被回滚（VERIFYING 残留在旧固件）→ 记一次失败并重排下载 / 放弃。
//  · 否则立即返回，继续正常启动。
void otaBootCheck();

// setup() 末尾（healthSetStage(BS_RUNNING) 之后）调用。
// 若本次是新固件首次启动(VERIFYING)且已跑到运行期 = 视为「启动成功」→
// esp_ota_mark_app_valid_cancel_rollback()，确认新固件、取消 bootloader 回滚。
void otaMarkValidOnBoot();

// 收到官方 Topic 的版本号 payload（纯数字，或 {"ver":N} JSON）：比较后决定是否触发升级。
// 触发 = 存 NVS(PENDING + targetVer) + esp_restart 进入 OTA 环境。由 mqttCallback 路由进来。
void otaOnOfficialVersion(const char* payload, unsigned int len);

// 当前固件版本（devInfo 上报 "fw" 字段用）。
uint32_t otaCurrentVersion();
