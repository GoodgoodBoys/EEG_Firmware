# EEG Firmware

面向 **EGG 智能交互终端** 的 ESP32-S3 固件。项目基于 Arduino Framework 与 FreeRTOS，集成屏幕动画、音频播放与录音、IMU 手势识别、舵机控制、BLE 配网、MQTT 通信、OTA 升级和分级低功耗管理。

> [!IMPORTANT]
> 本工程按自研 PCB v3.0 和 ESP32-S3-WROOM-1-N16R8 配置，不是通用开发板示例。直接烧录到其他硬件前，请先核对 Flash、PSRAM、外设型号和全部引脚定义。

## 主要功能

- ST7789 240 × 320 LCD，支持 MJPEG 动画、PSRAM 预加载和流式回退
- MAX98357A I2S 音频播放，支持 MP3 解码、音量控制和语音消息播放
- PDM 麦克风录音，设备端编码 MP3，最多保留 10 条待发送语音
- QMI8658 IMU，支持拍一拍、摇一摇、运动唤醒和姿态同步
- 舵机动作与自动断电，减少空闲功耗
- BLE 下发 Wi-Fi 凭据，设备名按 SN 生成为 `EGG_<SN>`
- MQTT over TLS，支持命令、音视频分片传输、设备绑定和健康状态上报
- LittleFS 资源管理，以及视频、音频和表情缓存
- A/B 分区 OTA、SHA-256 校验、失败重试和 bootloader 回滚
- L0/L1/L2 分级功耗管理，支持动态频率、自动 light-sleep 和低电关机
- RTC 健康“黑匣子”、任务看门狗和运行资源监控

## 硬件与构建环境

| 项目 | 当前配置 |
| --- | --- |
| MCU | ESP32-S3-WROOM-1-N16R8 |
| Flash / PSRAM | 16 MB QIO Flash / 8 MB OPI PSRAM |
| 构建系统 | PlatformIO |
| 框架 | Arduino（pioarduino 社区平台，Arduino-ESP32 3.x） |
| 屏幕 | ST7789，240 × 320，SPI |
| IMU | QMI8658，I²C |
| 音频输出 | MAX98357A，I2S |
| 麦克风 | PDM，16 kHz |
| 文件系统 | LittleFS |
| 串口波特率 | 115200 |

核心依赖已在 [`platformio.ini`](platformio.ini) 中声明，包括 ArduinoJson、ESP32Servo、LovyanGFX、NimBLE-Arduino、PubSubClient 和 QMI8658 驱动。

## 快速开始

### 1. 准备环境

安装以下任一种环境：

- VS Code + PlatformIO IDE 扩展
- PlatformIO Core 6.x 命令行工具

首次完整构建会下载平台、工具链和依赖，并根据 `sdkconfig` 重新编译部分 ESP-IDF 组件，耗时会明显长于后续增量构建。

### 2. 获取代码

```bash
git clone https://github.com/GoodgoodBoys/EEG_Firmware.git
cd EEG_Firmware
```

### 3. 编译

```bash
pio run -e esp32s3
```

### 4. 烧录固件

```bash
pio run -e esp32s3 -t upload
```

如 PlatformIO 未自动识别串口，可追加 `--upload-port COMx`。

### 5. 烧录 LittleFS 资源

```bash
pio run -e esp32s3 -t uploadfs
```

`data/` 下的出厂动画不会随普通固件烧录自动写入，首次部署、整片擦除或修改分区表后必须单独执行此步骤。

### 6. 查看日志

```bash
pio device monitor -b 115200
```

正常启动时应完成 LittleFS、SN、电源管理、LCD、音频、麦克风、IMU、舵机和电量检测初始化。电源管理配置生效后，日志中会出现自动 light-sleep 已开启的信息。

## 首次部署与配置

### 设备 SN

每台量产设备应写入唯一的 8 位数字 SN。仓库提供了独立工具 [`Test/snBurn/snBurn.ino`](Test/snBurn/snBurn.ino)：

1. 修改其中的 `TARGET_SN`。
2. 使用 Arduino IDE 将该 sketch 烧入设备。
3. 在 115200 波特率下确认回读校验成功。
4. 再使用 PlatformIO 烧录正式固件。

普通 `upload` 不会清除 NVS 中的 SN；`pio run -t erase` 会清除整片 Flash，执行后需要重新写入 SN、固件和 LittleFS 资源。未预烧录时，正式固件会使用 [`include/inc/nvs_sn.h`](include/inc/nvs_sn.h) 中的默认 SN，此行为只适合开发调试。

### BLE 配网

没有有效 Wi-Fi 配置时，设备进入配网流程；也可以长按 Button1 约 2 秒主动触发。App 通过 BLE 向设备下发 `ssid` 和 `password`，连接成功后配置保存到 NVS。

BLE Service 与 Characteristic UUID 定义在 [`include/inc/configSys.hpp`](include/inc/configSys.hpp)。

### MQTT 与 OTA

部署到真实环境前，请检查以下位置：

- [`include/inc/web.hpp`](include/inc/web.hpp)：MQTT 服务器、端口、账号和 Topic 前缀
- [`src/web.cpp`](src/web.cpp)：设备密钥盐、Topic 盐及认证开关
- [`include/inc/ota.hpp`](include/inc/ota.hpp)：`FW_VERSION`、发布 Topic、固件地址和域名白名单

发布 OTA 时应同时上传：

```text
<OTA_URL_BASE>/<version>/firmware.bin
<OTA_URL_BASE>/<version>/firmware.sha256
```

其中 `firmware.sha256` 为 `firmware.bin` 的 64 位小写 SHA-256 十六进制摘要。OTA 版本号必须单调递增。

> [!WARNING]
> 仓库中的服务地址、测试凭据和盐值只适合开发联调。生产发布前必须替换，并在服务端配置最小权限 ACL。不要把真实 Wi-Fi、MQTT 或云存储密钥提交到 Git。

## 文件系统与分区

16 MB Flash 使用双 OTA 分区布局，详见 [`partitions.csv`](partitions.csv)：

| 分区 | 大小 | 用途 |
| --- | ---: | --- |
| `nvs` | 20 KB | SN、设备配置和 OTA 状态 |
| `otadata` | 8 KB | OTA 启动选择信息 |
| `app0` | 3.5 MB | 固件槽 A |
| `app1` | 3.5 MB | 固件槽 B |
| `spiffs` | 8.94 MB | LittleFS 资源与运行时缓存 |

虽然分区 subtype 名为 `spiffs`，工程通过 `board_build.filesystem = littlefs` 使用 LittleFS。

出厂资源位于 [`data/def`](data/def)：

```text
data/
└── def/
    ├── idle.mjpeg
    ├── nod.mjpeg
    ├── showUp.mjpeg
    ├── sleeping.mjpeg
    ├── startSleep.mjpeg
    ├── swingSwing.mjpeg
    └── wink.mjpeg
```

运行时还可能生成 `/target.mjpeg`、`/output1.mp3`、`/voice_*.mp3` 和 `/stk/` 表情缓存。请勿随意修改文件名；播放状态机依赖这些固定路径。

## Button1 操作

| 操作 | 行为 |
| --- | --- |
| 单击 | 开始或结束一次录音 |
| 长按约 2 秒 | 进入 BLE 配网 |
| 长按约 8 秒 | 关机并进入低功耗休眠 |
| 关机状态单击 | 复检电量，电量足够或正在充电时开机 |

配网期间会忽略单击录音操作。

## 代码结构

```text
.
├── data/                 # LittleFS 出厂资源
├── include/inc/          # 模块头文件、硬件引脚与公共接口
├── lib/shine/            # Shine MP3 编码器源码
├── src/
│   ├── V1_1.cpp          # 启动流程、任务编排、功耗与主状态机
│   ├── web.cpp           # Wi-Fi、MQTT、传输协议与运行时持久化
│   ├── lcd.cpp           # ST7789、MJPEG 播放与屏幕状态机
│   ├── aud.cpp           # MP3 解码与 I2S 播放
│   ├── mic.cpp           # PDM 录音与 MP3 编码
│   ├── imu.cpp           # QMI8658、拍/摇检测与运动唤醒
│   ├── provision.cpp     # BLE 配网流程
│   ├── ota.cpp           # A/B OTA、校验与回滚
│   ├── configSys.cpp     # NVS 配置与 BLE GATT
│   ├── sticker.cpp       # 表情下载与 LRU 缓存
│   ├── fs.cpp            # LittleFS 初始化
│   └── nvs_sn.cpp        # SN 读写与校验
├── Test/                 # SN、功耗、运动识别和协议仿真工具
├── partitions.csv        # 16 MB Flash 分区表
├── platformio.ini        # PlatformIO 环境、依赖与 sdkconfig 覆盖项
└── sdkconfig.defaults    # ESP-IDF 默认配置
```

核心任务通过 FreeRTOS 队列与信号量通信：IMU、Web、LCD、音频和麦克风任务分别处理实时工作，主循环负责用户输入、舵机、功耗状态和跨模块调度。

## 常见问题

### 烧录后循环重启或 PSRAM 初始化失败

先核对模组是否确为 N16R8。当前配置为：

```ini
board_build.arduino.memory_type = qio_opi
board_build.flash_mode = qio
board_upload.flash_size = 16MB
```

如果实际硬件使用 QSPI PSRAM，需要相应调整 `memory_type`。

### 修改 `custom_sdkconfig` 后没有生效

清理后重新完整构建：

```bash
pio run -e esp32s3 -t clean
pio run -e esp32s3
```

### 能启动但没有动画

确认执行过 `pio run -e esp32s3 -t uploadfs`，并检查 `data/def/` 中的资源是否齐全。

### 无法连接 MQTT

依次检查 Wi-Fi 配置、系统时间、TLS 服务地址、认证方式、服务端账号/派生密钥和 Topic ACL。设备的串口日志会输出连接状态码，便于定位认证或网络问题。

## 开发注意事项

- 调整引脚前先检查 [`include/inc`](include/inc) 下各硬件模块的定义，当前映射与 PCB v3.0 绑定。
- 修改分区表后应执行整片擦除，并重新烧录 SN、固件和文件系统。
- 资源传输和 Flash 持久化与播放、低功耗状态机存在时序约束；改动任务优先级、队列深度或写盘策略后应做长时间稳定性测试。
- OTA 发布前至少验证正常升级、摘要错误、断网重试、新固件启动失败回滚四条路径。
- `Test/` 中包含硬件验证 sketch、运动样本/分析脚本和传输协议仿真脚本，可用于回归测试。

## License

本仓库目前未包含开源许可证。在许可证明确之前，请勿将代码视为已授权的开源软件；如需使用或分发，请联系项目维护者。
