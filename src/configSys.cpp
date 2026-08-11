#include "inc/configSys.hpp"
#include "esp_random.h"
#include "inc/debug.hpp"
#include <Preferences.h>

// ============================================================
//  配置持久化：存 NVS（不是 LittleFS）
//  · 独立命名空间 "egg_cfg"（与 SN 的 "device_info" 分开）。
//  · 整份配置以一个 JSON blob 存一个键：一次 save = 一次 putString，写入量最小、最省磨损。
//  · NVS 分区(0x9000, 20KB)不随 `uploadfs` / 文件系统 OTA 重刷而丢 —— 配置(含 peerSn 互绑、
//    WiFi 凭据、matchCode)从此持久，只有显式 `erase` 才清（与 SN 同级保护）。
//  · 旧固件把配置存在 LittleFS /Config/config.json；首次跑新固件时 migrateLegacyFromLittleFS()
//    自动搬进 NVS，现有设备升级不丢配置。
// ============================================================
#define CFG_NVS_NS   "egg_cfg"
#define CFG_NVS_KEY  "blob"

// ============================================================
// BLE 服务端连接回调
// ============================================================
class BLEServerCB : public NimBLEServerCallbacks {
public:
    BLEServerCB(ConfigManager* mgr) : _mgr(mgr) {}

    // ★ 新版签名：NimBLEConnInfo& 而不是 NimBLEServer*
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        _mgr->_bleConnected = true;
        LOG("[BLE] 客户端已连接\n");
        NimBLEDevice::stopAdvertising();
    }

    void onDisconnect(NimBLEServer* pServer,
                      NimBLEConnInfo& connInfo,
                      int reason) override {
        _mgr->_bleConnected = false;
        if (_mgr->_provMode) {
            LOG("[BLE] 客户端已断开，配网中，重新广播...\n");
            NimBLEDevice::startAdvertising();
        } else {
            LOG("[BLE] 客户端已断开，非配网模式，停止广播\n");
        }
    }
private:
    ConfigManager* _mgr;
};

// ============================================================
// BLE 写入特征回调
// ============================================================
class BLEWriteCB : public NimBLECharacteristicCallbacks {   // ← Nim 前缀
public:
    BLEWriteCB(ConfigManager* mgr) : _mgr(mgr) {}

    // NimBLE 签名多一个 NimBLEConnInfo& 参数
    void onWrite(NimBLECharacteristic* pChar,
                 NimBLEConnInfo& connInfo) override {
        // NimBLE getValue() 返回 std::string，显式转成 Arduino String
        String val = String(pChar->getValue().c_str());
        if (val.length() > 0) {
            _mgr->_blePendingPayload = val;
            _mgr->_bleHasNewData     = true;
        }
    }
private:
    ConfigManager* _mgr;
};

// ============================================================
// ConfigManager 实现
// ============================================================

ConfigManager::ConfigManager() {}

bool ConfigManager::begin(const char* bleName) {
    LOG("[CFG] 初始化配置管理器...\n");

    // LittleFS 仍需挂载：视频/语音/target 存这里；旧配置迁移也从这里读。
    if (!mountFS()) return false;

    if (loadFromNvs()) {
        LOG("[CFG] ✓ 从 NVS 加载配置\n");
    } else if (migrateLegacyFromLittleFS()) {
        LOG("[CFG] ✓ 检测到旧 LittleFS 配置 → 已迁移进 NVS\n");
    } else {
        LOG("[CFG] 首次开机（NVS 无配置、无旧文件），初始化默认配置...\n");
        initDefaultConfig();
        save();
    }

    _bleName = bleName;   // 记录名字，配网时再懒加载 BLE
    LOG("[CFG] 初始化完成（配置存 NVS；BLE 延迟到配网时初始化）\n");
    return true;
}

bool ConfigManager::mountFS() {
    if (!LittleFS.begin(true)) {
        LOG("[CFG] LittleFS 挂载失败！\n");
        return false;
    }
    LOG("[CFG] LittleFS 已挂载，总空间: %u KB，已用: %u KB\n",
        LittleFS.totalBytes() / 1024, LittleFS.usedBytes() / 1024);
    return true;
}

bool ConfigManager::validateJson(const char* path) {
    if (!LittleFS.exists(path)) { LOG("[CFG] 文件不存在: %s\n", path); return false; }
    String content = readFile(path);
    if (content.isEmpty()) { LOG("[CFG] 文件为空: %s\n", path); return false; }
    StaticJsonDocument<CONFIG_DOC_SIZE> testDoc;
    DeserializationError err = deserializeJson(testDoc, content);
    if (err) { LOG("[CFG] JSON解析失败 [%s]: %s\n", path, err.c_str()); return false; }
    if (!testDoc.containsKey("ssid")     || !testDoc.containsKey("password") ||
        !testDoc.containsKey("matchCode")|| !testDoc.containsKey("brightness")||
        !testDoc.containsKey("volume")) {
        LOG("[CFG] 配置文件缺少必要字段: %s\n", path);
        return false;
    }
    return true;
}

bool ConfigManager::loadFromFile(const char* path) {
    String content = readFile(path);
    if (content.isEmpty()) return false;
    DeserializationError err = deserializeJson(_doc, content);
    return !err;
}

void ConfigManager::initDefaultConfig() {
    _doc.clear();
    _doc["ssid"]       = CFG_DEF_SSID;
    _doc["password"]   = CFG_DEF_PASSWORD;
    // matchCode：首次开机随机生成 6 位（取代弱默认 "000"，防默认值撞车 + 加大暴力空间）
    char mc[8];
    snprintf(mc, sizeof(mc), "%06u", (unsigned)(esp_random() % 1000000u));
    _doc["matchCode"]  = mc;
    _doc["brightness"] = CFG_DEF_BRIGHTNESS;
    _doc["volume"]     = CFG_DEF_VOLUME;
    LOG("[CFG] 默认配置已初始化（matchCode=%s）\n", mc);
}

// 从 NVS 读整份配置 JSON blob 进 _doc。命名空间/键不存在或解析失败返回 false。
// ★ 在 begin() 里调用，此时任务尚未创建（单线程），故 deserialize 无需加锁（同 loadFromFile）。
bool ConfigManager::loadFromNvs() {
    Preferences p;
    if (!p.begin(CFG_NVS_NS, true)) return false;   // 只读；命名空间不存在 → false（首次开机）
    String js = p.getString(CFG_NVS_KEY, "");
    p.end();
    if (js.isEmpty()) return false;
    DeserializationError err = deserializeJson(_doc, js);
    if (err) { LOG("[CFG] NVS 配置解析失败: %s\n", err.c_str()); return false; }
    // 最小字段校验：缺关键字段视为无效 → 回退到迁移/默认（防写坏的残留）
    if (!_doc.containsKey("ssid") || !_doc.containsKey("matchCode")) {
        LOG("[CFG] NVS 配置缺关键字段，判为无效\n");
        return false;
    }
    return true;
}

// 一次性迁移：旧固件把配置存在 LittleFS /Config/config.json。首次跑新固件（NVS 尚无配置）时
// 把它读出来并 save() 进 NVS，现有设备升级不丢 WiFi/matchCode/互绑。之后 NVS 优先，不再触发。
// 旧文件保留不删（无害；万一回退旧固件仍能读到）。
bool ConfigManager::migrateLegacyFromLittleFS() {
    bool ok = false;
    if (validateJson(CONFIG_FILE)        && loadFromFile(CONFIG_FILE))        ok = true;
    else if (validateJson(CONFIG_BACKUP_FILE) && loadFromFile(CONFIG_BACKUP_FILE)) ok = true;
    if (!ok) return false;
    if (!save()) { LOG("[CFG] ⚠ 旧配置迁移写 NVS 失败\n"); return false; }
    LOG("[CFG] 旧配置已迁移进 NVS：%s\n", toJsonString().c_str());
    return true;
}

bool ConfigManager::save() {
    // 回收 pool 中被覆盖的旧字符串（反复 setString 同一键会累积），防止 pool 撑满
    taskENTER_CRITICAL(&_mux);
    _doc.garbageCollect();
    taskEXIT_CRITICAL(&_mux);

    String js = toJsonString();   // 内部持锁，序列化进栈缓冲后在临界区外构造 String
    if (js.isEmpty()) return false;

    // ★ 写 NVS（不是 LittleFS）：整份 JSON 存一个键。NVS 自带磨损均衡 + 每条 CRC，
    //   分区不随 uploadfs / 文件系统 OTA 重刷而丢。写 flash 有短暂 cache 冻结，但配置写
    //   低频（配网/绑定/设置），且比原来的 atomicWrite(临时文件+rename 多次 flash 操作)更省。
    Preferences p;
    if (!p.begin(CFG_NVS_NS, false)) { LOG("[CFG] ✗ 打开 NVS 命名空间失败\n"); return false; }
    size_t n = p.putString(CFG_NVS_KEY, js);
    p.end();
    if (n == 0) { LOG("[CFG] ✗ NVS 写入失败（putString 返回 0，NVS 满？）\n"); return false; }
    LOG("[CFG] 配置已写入 NVS（%u B）\n", (unsigned)js.length());
    return true;
}

// 恢复出厂：清配置(NVS egg_cfg + LittleFS 旧 /Config)，保留 SN 与视频文件，然后重启。
// 由 handleBLEWrite 收到 {"factoryReset":true} 时调用（配网态经 BLE 触发，不依赖网络）。
void ConfigManager::factoryReset() {
    LOG("[CFG] ★★ 恢复出厂：清配置(NVS egg_cfg + LittleFS /Config)，保留 SN 与视频，重启\n");
    // 1. 清 NVS 配置命名空间（不碰 SN 的 device_info 命名空间，也不碰视频文件）
    Preferences p;
    if (p.begin(CFG_NVS_NS, false)) { p.clear(); p.end(); }
    // 2. 删 LittleFS 旧配置文件：否则开机迁移逻辑会把它复活。只删 /Config，不动 /def /target 视频。
    LittleFS.remove(CONFIG_FILE);
    LittleFS.remove(CONFIG_BACKUP_FILE);
    LittleFS.remove(CONFIG_TEMP_FILE);
    LOG("[CFG] 配置已清除，0.5s 后重启进入全新态（无配置 → 自动进配网）\n");
    Serial.flush();
    delay(500);
    ESP.restart();   // 不返回
}

String ConfigManager::toJsonString() const {
    // 序列化进固定缓冲（临界区内不分配内存），再在临界区外构造 String
    char buf[CONFIG_DOC_SIZE * 2];
    taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    serializeJsonPretty(_doc, buf, sizeof(buf));
    taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    return String(buf);
}

String ConfigManager::getString(const char* key, const char* defaultVal) const {
    // 在临界区内只 strncpy 到栈缓冲（无 malloc），String 在临界区外构造
    char buf[128];
    taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    const char* v = _doc[key] | defaultVal;
    strncpy(buf, v ? v : "", sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    return String(buf);
}
int ConfigManager::getInt(const char* key, int defaultVal) const {
    taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    int v = _doc[key] | defaultVal;
    taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    return v;
}
float ConfigManager::getFloat(const char* key, float defaultVal) const {
    taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    float v = _doc[key] | defaultVal;
    taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    return v;
}
bool ConfigManager::getBool(const char* key, bool defaultVal) const {
    taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    bool v = _doc[key] | defaultVal;
    taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    return v;
}

void ConfigManager::setString(const char* key, const char* value) {
    // ★ 关键：用 Arduino String 赋值，强制 ArduinoJson 复制内容进 pool；
    //   直接传 const char* 只会保存指针，源失效后变悬空指针（terminal 乱码的根因）。
    //   String 在临界区外构造，避免在临界区内 malloc。
    String v(value);
    taskENTER_CRITICAL(&_mux);
    _doc[key] = v;
    taskEXIT_CRITICAL(&_mux);
}
void ConfigManager::setInt(const char* key, int value) {
    taskENTER_CRITICAL(&_mux); _doc[key] = value; taskEXIT_CRITICAL(&_mux);
}
void ConfigManager::setFloat(const char* key, float value) {
    taskENTER_CRITICAL(&_mux); _doc[key] = value; taskEXIT_CRITICAL(&_mux);
}
void ConfigManager::setBool(const char* key, bool value) {
    taskENTER_CRITICAL(&_mux); _doc[key] = value; taskEXIT_CRITICAL(&_mux);
}

String ConfigManager::readFile(const char* path) {
    File f = LittleFS.open(path, FILE_READ);
    if (!f) { LOG("[CFG] 无法打开文件读取: %s\n", path); return ""; }
    String content = f.readString();
    f.close();
    return content;
}

void ConfigManager::startAdvertising() { NimBLEDevice::startAdvertising(); }
void ConfigManager::stopAdvertising() {
    if (_bleInited) NimBLEDevice::stopAdvertising();
}

void ConfigManager::ensureBLEInit() {
    if (_bleInited) return;
    setupBLE(_bleName ? _bleName : "ESP32-Device");
    _bleInited = true;
}

void ConfigManager::deinitBLE() {
    if (!_bleInited) return;
    NimBLEDevice::stopAdvertising();
    NimBLEDevice::deinit(true);   // true = 释放全部，归还内部 SRAM
    _bleServer = nullptr; _charRead = nullptr;
    _charWrite = nullptr; _charNotify = nullptr;
    _bleConnected = false;
    _bleInited = false;
    LOG("[BLE] 已 deinit，内存归还\n");
}

// ----------------------------------------------------------
// BLE 初始化
// ----------------------------------------------------------
void ConfigManager::setupBLE(const char* name) {
    // ★ 回调对象用函数内 static，不要 new：
    //   本函数每次 ensureBLEInit()（每次进配网）都会执行一遍。查 NimBLE 2.5.0 源码：
    //     · NimBLEServer::setCallbacks(cb, deleteCallbacks=true) —— server 析构时会 delete ✓
    //     · NimBLECharacteristic::setCallbacks(cb) —— 无所有权参数，析构函数只删 descriptor，
    //       【不删回调】 → 每次配网 new 一个就永久泄漏一个。
    //   两者都改成 static：只构造一次、地址恒定，NimBLE 无论删不删都不会出问题
    //   （server 那个显式传 false，避免它去 delete 一个非 new 出来的对象）。
    //   this 恒为唯一的 Config 单例，static 复用是安全的。
    static BLEServerCB s_serverCb(this);
    static BLEWriteCB  s_writeCb(this);

    NimBLEDevice::init(name);

    _bleServer = NimBLEDevice::createServer();
    _bleServer->setCallbacks(&s_serverCb, false);   // false = 不要 delete（不是 new 出来的）

    NimBLEService* service = _bleServer->createService(BLE_SERVICE_UUID);

    _charRead = service->createCharacteristic(
        BLE_CHAR_READ_UUID, NIMBLE_PROPERTY::READ);
    _charRead->setValue(toJsonString().c_str());

    _charWrite = service->createCharacteristic(
        BLE_CHAR_WRITE_UUID, NIMBLE_PROPERTY::WRITE);
    _charWrite->setCallbacks(&s_writeCb);

    _charNotify = service->createCharacteristic(
        BLE_CHAR_NOTIFY_UUID, NIMBLE_PROPERTY::NOTIFY);

    // ★ service->start() 已废弃，删除
    // ★ Server start 会自动启动所有 service

    _bleServer->start();   // ← 改为启动 server

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    // ★ 把设备名 EGG_<SN> 放进【扫描响应】：主广播包 31 字节已被 128-bit Service UUID
    //   占满（flags 3B + UUID 18B = 21B，放不下 12+ 字节的名字），故开启扫描响应承载名字。
    //   这样 App 扫描到设备即可从广播名解析出 SN，在"查询到设备"弹窗里显示供用户核对。
    //   注意顺序：enableScanResponse(true) 必须先于 setName，否则 setName 会写进主包而放不下。
    adv->enableScanResponse(true);
    adv->setName(name);

    // 不调用 startAdvertising()，等配网时手动触发
    LOG("[BLE] 蓝牙已初始化(未广播)，设备名(扫描响应): %s\n", name);
}

// ----------------------------------------------------------
// 处理 BLE 写入（区分正常模式 / 配网模式）
// ----------------------------------------------------------
void ConfigManager::handleBLEWrite(const String& payload) {
    LOG("[BLE] 收到写入: %s\n", payload.c_str());

    StaticJsonDocument<CONFIG_DOC_SIZE> incoming;
    DeserializationError err = deserializeJson(incoming, payload);
    if (err) {
        notifyBLEResult("{\"status\":\"error\",\"msg\":\"JSON解析失败\"}");
        return;
    }

    // ═══ 配网模式 ═══
    if (_provMode) {
        // ⓿ 恢复出厂：清配置(NVS egg_cfg + LittleFS /Config)，保留 SN 与视频后重启。
        //    走 BLE 触发（配网态设备可能还没联网，不依赖网络）；最高优先级，先于其它处理。
        if (incoming.containsKey("factoryReset")) {
            if (incoming["factoryReset"].as<bool>()) {
                notifyBLEResult("{\"status\":\"ok\",\"msg\":\"factory reset\"}");
                factoryReset();   // 内部 ESP.restart()，不返回
            }
            return;
        }

        // ① APP 请求断开
        if (incoming.containsKey("connection")) {
            const char* conn = incoming["connection"] | "";
            if (strcmp(conn, "disconnect") == 0) {
                _provDisconnectReq = true;
                notifyBLEResult("{\"status\":\"ok\",\"msg\":\"disconnecting\"}");
                return;
            }
        }

        // ② 设置字段（brightness/volume/matchCode/dndPeriods）可【不带 ssid 单独更新】，
        //    ssid+password 则额外触发联网。这样「我的设备」的设置表单（不发 ssid）也能改
        //    勿扰时段等设置，不再被"in provisioning"忽略。
        bool hasCreds = incoming.containsKey("ssid") && incoming.containsKey("password");
        bool changed  = false;

        // ★ matchCode/dndPeriods 是字符串：.as<String>() 会 malloc，必须在临界区【外】先取好，
        //   不能在临界区里构造临时 String。brightness/volume 是整型，pool 赋值不 malloc，可留在里面。
        bool   hasMatch = incoming.containsKey("matchCode");
        bool   hasDnd   = incoming.containsKey("dndPeriods");   // 紧凑串 "s-e,s-e"（半小时索引，s>e=跨0点）；空串=关闭
        String matchV   = hasMatch ? incoming["matchCode"].as<String>()  : String();
        String dndV     = hasDnd   ? incoming["dndPeriods"].as<String>() : String();

        taskENTER_CRITICAL(&_mux);
        if (incoming.containsKey("brightness")) { _doc["brightness"] = incoming["brightness"]; changed = true; }
        if (incoming.containsKey("volume"))     { _doc["volume"]     = incoming["volume"];     changed = true; }
        if (hasMatch)  { _doc["matchCode"]  = matchV; changed = true; }
        if (hasDnd)    { _doc["dndPeriods"] = dndV;   changed = true; }
        if (hasCreds) {
            const char* s = incoming["ssid"]     | "";
            const char* p = incoming["password"] | "";
            strncpy(_provSsidBuf, s, sizeof(_provSsidBuf) - 1); _provSsidBuf[sizeof(_provSsidBuf) - 1] = 0;
            strncpy(_provPassBuf, p, sizeof(_provPassBuf) - 1); _provPassBuf[sizeof(_provPassBuf) - 1] = 0;
            _provHasCreds = true;
        }
        taskEXIT_CRITICAL(&_mux);

        if (hasCreds || changed) {
            save();
            if (qConfigUpdate != nullptr) {
                uint8_t msg = 1;
                xQueueSend(qConfigUpdate, &msg, 0);
                LOG("[CFG] 已发送配置更新通知\n");
            }
            LOG("[PROV] 配网写入：creds=%d 设置更新=%d dnd=%s\n",
                hasCreds, changed, (incoming.containsKey("dndPeriods")
                    ? (const char*)(incoming["dndPeriods"] | "") : "(未变)"));
            notifyBLEResult(hasCreds ? "{\"status\":\"ok\",\"msg\":\"creds received\"}"
                                     : "{\"status\":\"ok\",\"msg\":\"settings saved\"}");
        } else {
            notifyBLEResult("{\"status\":\"ignored\",\"msg\":\"in provisioning\"}");
        }
        return;
    }

    // ═══ 正常模式：增量更新 ═══
    // ★ key 必须用 String 复制：直接用 incoming 里的 const char* 作 key，ArduinoJson 只存指针，
    //   incoming 出栈后 key 悬空（同 setString 注释里的坑）。String 在临界区【外】构造，
    //   既复制了 key、又不在临界区里 malloc。
    if (incoming.containsKey("key") && incoming.containsKey("value")) {
        String       key = incoming["key"].as<String>();
        JsonVariant  val = incoming["value"];
        taskENTER_CRITICAL(&_mux);
        _doc[key] = val;               // String key → 复制进 pool；value 亦复制
        taskEXIT_CRITICAL(&_mux);
    } else {
        for (JsonPair kv : incoming.as<JsonObject>()) {
            String      skey = kv.key().c_str();   // 复制 key（临界区外）
            JsonVariant sval = kv.value();
            taskENTER_CRITICAL(&_mux);
            _doc[skey] = sval;
            taskEXIT_CRITICAL(&_mux);
        }
    }

    if (save()) {
        _charRead->setValue(toJsonString().c_str());
        notifyBLEResult("{\"status\":\"ok\",\"msg\":\"配置已保存\"}");
        if (qConfigUpdate != nullptr) {
            uint8_t msg = 1;
            xQueueSend(qConfigUpdate, &msg, 0);
        }
    } else {
        notifyBLEResult("{\"status\":\"error\",\"msg\":\"保存失败\"}");
    }
}

void ConfigManager::notifyBLEResult(const char* result) {
    if (_bleConnected && _charNotify) {
        _charNotify->setValue((uint8_t*)result, strlen(result));
        _charNotify->notify();
    }
    LOG("[BLE] 通知: %s\n", result);
}

// ----------------------------------------------------------
// 配网接口
// ----------------------------------------------------------
void ConfigManager::setProvMode(bool on) {
    _provMode = on;
    if (on) {
        // 进入配网，清空残留标志
        _provHasCreds      = false;
        _provDisconnectReq = false;
    }
    LOG("[CFG] 配网模式: %s\n", on ? "开启" : "关闭");
}

bool ConfigManager::getProvCreds(String& ssid, String& pass) {
    if (!_provHasCreds) return false;
    char s[64], p[64];
    taskENTER_CRITICAL(&_mux);
    strcpy(s, _provSsidBuf);
    strcpy(p, _provPassBuf);
    _provHasCreds = false;
    taskEXIT_CRITICAL(&_mux);
    ssid = String(s);     // String 分配在临界区外
    pass = String(p);
    return true;
}

bool ConfigManager::getDisconnectRequest() {
    if (!_provDisconnectReq) return false;
    _provDisconnectReq = false;
    return true;
}

void ConfigManager::sendProvStatus(const char* json) {
    if (_charRead) _charRead->setValue((uint8_t*)json, strlen(json));
    if (_bleConnected && _charNotify) {
        _charNotify->setValue((uint8_t*)json, strlen(json));
        _charNotify->notify();
    }
    LOG("[BLE] 配网状态推送: %s\n", json);
}
// ----------------------------------------------------------
// 刷新 BLE READ 特征为最新设备信息
// toJsonString() 即完整配置（含 ssid/password/matchCode/brightness/volume/terminal）
// ----------------------------------------------------------
void ConfigManager::refreshReadInfo() {
    if (!_charRead) return;
    String js = toJsonString();
    _charRead->setValue((uint8_t*)js.c_str(), js.length());
    LOG("[BLE] READ 特征已刷新为设备信息: %s\n", js.c_str());
}
// ----------------------------------------------------------
// loop()
// ----------------------------------------------------------
void ConfigManager::loop() {
    if (_bleHasNewData) {
        _bleHasNewData = false;
        String payload = _blePendingPayload;
        handleBLEWrite(payload);
    }
}

void ConfigManager::printConfig() const {
    LOG("[CFG] ===== 当前配置 =====\n");
    // ★ 用固定栈缓冲序列化：临界区内禁 malloc（原来的 String output 会在临界区里 realloc）。
    //   与 toJsonString() 保持一致的安全写法。
    char buf[CONFIG_DOC_SIZE * 2];
    taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    serializeJsonPretty(_doc, buf, sizeof(buf));
    taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    LOG("%s\n", buf);
    LOG("[CFG] ====================\n");
}

ConfigManager Config;