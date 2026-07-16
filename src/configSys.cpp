#include "inc/configSys.hpp"
#include "esp_random.h"
#include "inc/debug.hpp"

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

    if (!mountFS()) return false;

    bool mainOk   = validateJson(CONFIG_FILE);
    bool backupOk = validateJson(CONFIG_BACKUP_FILE);

    if (mainOk) {
        loadFromFile(CONFIG_FILE);
        LOG("[CFG] 主配置加载成功\n");
        if (!backupOk) {
            String js = toJsonString();
            writeFile(CONFIG_BACKUP_FILE, js);
            LOG("[CFG] 备份已刷新\n");
        }
    } else if (backupOk) {
        LOG("[CFG] 主配置损坏，尝试从备份恢复...\n");
        if (restoreFromBackup()) LOG("[CFG] 从备份恢复成功\n");
        else { LOG("[CFG] 备份恢复失败，初始化默认配置\n"); initDefaultConfig(); save(); }
    } else {
        LOG("[CFG] 首次开机，初始化默认配置...\n");
        initDefaultConfig();
        save();
    }

    _bleName = bleName;   // 记录名字，配网时再懒加载 BLE
    LOG("[CFG] 初始化完成（BLE 延迟到配网时初始化）\n");
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

bool ConfigManager::atomicWrite(const String& jsonStr) {
    if (!writeFile(CONFIG_TEMP_FILE, jsonStr)) { LOG("[CFG] 写入临时文件失败\n"); return false; }
    if (!validateJson(CONFIG_TEMP_FILE)) {
        LOG("[CFG] 临时文件校验失败，中止写入\n");
        LittleFS.remove(CONFIG_TEMP_FILE);
        return false;
    }
    if (LittleFS.exists(CONFIG_FILE)) {
        if (LittleFS.exists(CONFIG_BACKUP_FILE)) LittleFS.remove(CONFIG_BACKUP_FILE);
        if (!LittleFS.rename(CONFIG_FILE, CONFIG_BACKUP_FILE))
            LOG("[CFG] 备份旧配置失败，继续尝试覆盖...\n");
    }
    if (!writeFile(CONFIG_FILE, jsonStr)) {
        LOG("[CFG] 写入正式配置文件失败！\n");
        restoreFromBackup();
        return false;
    }
    LittleFS.remove(CONFIG_TEMP_FILE);
    LOG("[CFG] 配置已原子写入\n");
    return true;
}

bool ConfigManager::restoreFromBackup() {
    if (!validateJson(CONFIG_BACKUP_FILE)) { LOG("[CFG] 备份文件无效，无法恢复\n"); return false; }
    String content = readFile(CONFIG_BACKUP_FILE);
    if (content.isEmpty()) return false;
    if (!writeFile(CONFIG_FILE, content)) { LOG("[CFG] 恢复备份到主配置失败\n"); return false; }
    return loadFromFile(CONFIG_FILE);
}

bool ConfigManager::save() {
    // 回收 pool 中被覆盖的旧字符串（反复 setString 同一键会累积），防止 pool 撑满
    taskENTER_CRITICAL(&_mux);
    _doc.garbageCollect();
    taskEXIT_CRITICAL(&_mux);

    String js = toJsonString();
    if (js.isEmpty()) return false;
    return atomicWrite(js);
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

bool ConfigManager::writeFile(const char* path, const String& content) {
    File f = LittleFS.open(path, FILE_WRITE);
    if (!f) { LOG("[CFG] 无法打开文件写入: %s\n", path); return false; }
    size_t written = f.print(content);
    f.close();
    return (written == content.length());
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
    NimBLEDevice::init(name);

    _bleServer = NimBLEDevice::createServer();
    _bleServer->setCallbacks(new BLEServerCB(this));

    NimBLEService* service = _bleServer->createService(BLE_SERVICE_UUID);

    _charRead = service->createCharacteristic(
        BLE_CHAR_READ_UUID, NIMBLE_PROPERTY::READ);
    _charRead->setValue(toJsonString().c_str());

    _charWrite = service->createCharacteristic(
        BLE_CHAR_WRITE_UUID, NIMBLE_PROPERTY::WRITE);
    _charWrite->setCallbacks(new BLEWriteCB(this));

    _charNotify = service->createCharacteristic(
        BLE_CHAR_NOTIFY_UUID, NIMBLE_PROPERTY::NOTIFY);

    // ★ service->start() 已废弃，删除
    // ★ Server start 会自动启动所有 service

    _bleServer->start();   // ← 改为启动 server

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    // ★ setScanResponse → setScanResponseData(空) 或直接不调用
    // adv->setScanResponseData(NimBLEAdvertisementData());  // 可选

    // 不调用 startAdvertising()，等配网时手动触发
    LOG("[BLE] 蓝牙已初始化(未广播)，设备名: %s\n", name);
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

        taskENTER_CRITICAL(&_mux);
        if (incoming.containsKey("brightness")) { _doc["brightness"] = incoming["brightness"];              changed = true; }
        if (incoming.containsKey("volume"))     { _doc["volume"]     = incoming["volume"];                  changed = true; }
        if (incoming.containsKey("matchCode"))  { _doc["matchCode"]  = incoming["matchCode"].as<String>();  changed = true; }
        // 勿扰时段：紧凑字符串 "s-e,s-e"（半小时索引 s∈0..47 / e∈1..48，s>e=跨0点）；空串=关闭
        if (incoming.containsKey("dndPeriods")) { _doc["dndPeriods"] = incoming["dndPeriods"].as<String>(); changed = true; }
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

    // ═══ 正常模式：原逻辑不变（也是增量更新）═══
    if (incoming.containsKey("key") && incoming.containsKey("value")) {
        const char* key = incoming["key"];
        JsonVariant  val = incoming["value"];
        taskENTER_CRITICAL(&_mux);
        _doc[key] = val;
        taskEXIT_CRITICAL(&_mux);
    } else {
        taskENTER_CRITICAL(&_mux);
        for (JsonPair kv : incoming.as<JsonObject>()) {
            _doc[kv.key()] = kv.value();
        }
        taskEXIT_CRITICAL(&_mux);
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
    taskENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    String output;
    serializeJsonPretty(_doc, output);
    taskEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
    LOG("%s\n", output.c_str());
    LOG("[CFG] ====================\n");
}

ConfigManager Config;