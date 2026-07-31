#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
// #include <BLEDevice.h>
// #include <BLEServer.h>
// #include <BLEUtils.h>
// #include <BLE2902.h>

#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLECharacteristic.h>
#include <NimBLEAdvertising.h>

// ============================================================
// 配置文件路径
// ============================================================
#define CONFIG_FILE           "/Config/config.json"
#define CONFIG_BACKUP_FILE    "/Config/config.bak.json"
#define CONFIG_TEMP_FILE      "/Config/config.temp.json"

// ============================================================
// 默认配置值
// ============================================================
#define CFG_DEF_SSID          ""
#define CFG_DEF_PASSWORD      ""
#define CFG_DEF_MATCHCODE     "000000"
#define CFG_DEF_BRIGHTNESS    128
// 音量默认 50%（0~255 量程 ⇒ 128）。与 App 配网页音量条的默认值 50 一致。
// ★ 这是全工程唯一的音量默认值来源，aud.cpp 的 Config.getInt 兜底也引用它，别再各写一份。
#define CFG_DEF_VOLUME        128

// ============================================================
// JSON 文档大小
// ============================================================
#define CONFIG_DOC_SIZE       1024

// ============================================================
// BLE UUID
// ============================================================
#define BLE_SERVICE_UUID        "69f4119b-a201-4481-bd31-9ba6c42ec40e"
#define BLE_CHAR_READ_UUID      "936fa05e-a0b4-45ac-b864-0f85da1f9718"
#define BLE_CHAR_WRITE_UUID     "7cd2459b-8016-4930-bfec-343c921cd594"
#define BLE_CHAR_NOTIFY_UUID    "02b68376-9001-4494-8ac3-a5065d23297b"

class ConfigManager {
public:
    ConfigManager();

    bool begin(const char* bleName = "EGG_ESP32S3_0");

    String getString(const char* key, const char* defaultVal = "") const;
    int    getInt   (const char* key, int     defaultVal = 0)      const;
    float  getFloat (const char* key, float   defaultVal = 0.0f)   const;
    bool   getBool  (const char* key, bool    defaultVal = false)  const;

    void setString(const char* key, const char* value);
    void setInt   (const char* key, int         value);
    void setFloat (const char* key, float       value);
    void setBool  (const char* key, bool        value);

    bool   save();
    String toJsonString() const;
    void   loop();
    void   startAdvertising();
    void   stopAdvertising();

    bool isBLEConnected() const { return _bleConnected; }

    void printConfig() const;

    // ========================================================
    //  配网支持（由 provision 模块调用）
    // ========================================================
    // 设置/退出配网模式（影响 BLE 写入的路由）
    void setProvMode(bool on);
    bool isProvMode() const { return _provMode; }

    // 取 BLE 收到的配网凭据（AP 配网），有则返回 true 并清标志
    bool getProvCreds(String& ssid, String& pass);

    // APP 是否请求断开（{"connection":"disconnect"}），有则返回 true 并清标志
    bool getDisconnectRequest();

    // 向 BLE read 特征写入状态 JSON（如 {"status":"online"}），并通过 notify 推送
    void sendProvStatus(const char* json);

    // 刷新 BLE READ 特征为最新设备信息（含 ssid/password/brightness/volume 等）
    void refreshReadInfo();

    void ensureBLEInit();   // 懒加载：仅配网前初始化 BLE
    void deinitBLE();       // 配网后释放 BLE，归还 SRAM

private:
    // ---------- 文件操作 ----------
    bool   mountFS();
    bool   loadFromFile(const char* path);
    bool   validateJson(const char* path);
    bool   atomicWrite(const String& jsonStr);
    bool   restoreFromBackup();
    void   initDefaultConfig();
    bool   writeFile(const char* path, const String& content);
    String readFile(const char* path);

    // ---------- 内部状态 ----------
    StaticJsonDocument<CONFIG_DOC_SIZE> _doc;
    mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    // ---------- BLE（官方库 Bluedroid）----------
    void setupBLE(const char* name);
    void handleBLEWrite(const String& payload);
    void notifyBLEResult(const char* result);

    NimBLEServer*          _bleServer    = nullptr;
    NimBLECharacteristic*  _charRead     = nullptr;
    NimBLECharacteristic*  _charWrite    = nullptr;
    NimBLECharacteristic*  _charNotify   = nullptr;
    volatile bool       _bleConnected = false;

    volatile bool _bleHasNewData     = false;
    String        _blePendingPayload;

    // ---------- 配网状态 ----------
    volatile bool _provMode          = false;
    volatile bool _provHasCreds      = false;
    volatile bool _provDisconnectReq = false;
    char          _provSsidBuf[64]   = {0};
    char          _provPassBuf[64]   = {0};

    friend class BLEServerCB;
    friend class BLEWriteCB;

    bool        _bleInited = false;
    const char* _bleName   = nullptr;
};

extern ConfigManager Config;
extern QueueHandle_t qConfigUpdate;