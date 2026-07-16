# EGG 固件 —— 从 Arduino IDE 迁移到 PlatformIO 操作手册

目标：把工程搬到 PlatformIO，打开 ESP-IDF 的电源管理（PM）+ Tickless Idle，
让自动 light-sleep 生效——也就是让你开机日志里那行
`⚠ esp_pm_configure 失败(PM/Tickless 未启用)` 变成 `✓ 自动 light-sleep 已开启`。
这一步做完，你 L2 待机那 40mA 里 CPU 一直 240MHz 空转的那块会塌下去。

---

## 0. 先理解这次迁移做了什么（30秒）

你的代码**一行没动**（除了 V1_1.ino 改名加一句）。变的只是「构建方式」：

- Arduino IDE 用的是**预编译内核**，sdkconfig 被锁死，PM 关着且你改不了——这是你那行 ⚠ 的根源。
- PlatformIO + pioarduino 平台支持一个叫 `custom_sdkconfig` 的功能，能在**仍然写 Arduino 代码**的前提下，重新编译 ESP-IDF 底层、把 `CONFIG_PM_ENABLE` 打开。
- 我已经在 `platformio.ini` 里把那几个开关写好了。你照着下面做就行。

> 为什么用 pioarduino 而不是官方平台：官方 PlatformIO 的 espressif32 平台目前只支持
> arduino-esp32 **2.0.17**，跑不了你的 3.x 内核和 NimBLE 2.5.0。3.x 由社区 pioarduino
> 分支维护，`platformio.ini` 里 `platform=` 那行指向的就是它。

---

## 1. 安装 VS Code + PlatformIO（约 10 分钟）

1. 装 **VS Code**：https://code.visualstudio.com/ 下载安装。
2. 打开 VS Code → 左侧竖排图标里点**扩展**（四个方块那个）。
3. 搜索框输入 `PlatformIO IDE` → 点 **Install**。
4. 装完它会自动下载工具链（几百 MB，右下角有进度条），可能要 5~10 分钟，
   完成后提示重启 VS Code，重启。
5. 重启后左侧出现一个**蚂蚁头/外星人图标**（PlatformIO 主页入口），看到它就装好了。

> 如果后面 `platform=`（pioarduino）那步报 penv / 安装错误，是标准 PlatformIO 和
> pioarduino 偶发的环境冲突。解决办法：卸载 PlatformIO IDE 扩展，改装
> **pioarduino IDE** 扩展（扩展商店搜 pioarduino），用法完全一样。先用标准的，
> 出问题再换。

---

## 2. 把工程放好（约 5 分钟）

1. 把我给你的 `egg-firmware` 整个文件夹解压到一个**路径不含中文、不含空格**的位置
   （比如 `D:\egg-firmware`，别放桌面/中文目录，PlatformIO 对中文路径偶尔抽风）。

2. 把你现有工程里的 **10 个源文件**复制进 `egg-firmware/src/`：
   ```
   V1_1.ino  → 改名 V1_1.cpp（看下面第 3 步必改项）
   aud.cpp   configSys.cpp   fs.cpp   imu.cpp   lcd.cpp
   mic.cpp   nvs_sn.cpp   provision.cpp   web.cpp
   ```
   `src/` 里那个占位 txt 放好后删掉。

3. 头文件**已经帮你放好**在 `include/inc/` 里了（包括 minimp3.h），不用动。
   你代码里的 `#include "inc/xxx.hpp"` 会自动从这里找到。

4. 把你原来烧进文件系统的视频/音频按 `data/` 里的说明放进 `data/`（见该目录 txt）。

---

## 3. ★ 唯一要改的代码：V1_1.cpp

Arduino IDE 会偷偷帮 `.ino` 自动加 `#include <Arduino.h>` 和函数声明。改成 `.cpp` 后没了，所以：

1. 文件名 `V1_1.ino` → `V1_1.cpp`
2. 在文件**最顶部**（所有 include 之前）加一行：
   ```cpp
   #include <Arduino.h>
   ```
3. 编译若报 `'xxx' was not declared in this scope`：说明某函数在定义前被调用了。
   去那个函数定义处，把它的签名复制一份放到 `setup()` 之前当前向声明。
   例：报 `powerManagerLoop`，就在前面加 `static void powerManagerLoop();`
   （你这份代码基本自上而下，大概率不用加，编译器会精确告诉你哪行。）

---

## 4. 第一次编译（Build）

1. VS Code 底部有一排蓝色状态栏，最左边有个 **✓ 对勾图标**（Build / 编译）。点它。
   - 第一次会下载 pioarduino 平台 + arduino-esp32 3.x + 所有库，**很慢（10~20 分钟）**，
     而且因为开了 `custom_sdkconfig`，它要**重新编译 ESP-IDF**，更慢。耐心等，只有第一次这样。
2. 编译成功底部会显示绿色 `SUCCESS`。
3. 如果报错，翻到本文最后第 7 节「常见报错」。

> 小贴士：状态栏那几个图标从左到右依次是
> ✓ 编译、→ 烧录(Upload)、🗑 清理、🔌 串口监视器(Monitor)、🏠 主页。

---

## 5. 烧录固件 + 文件系统

**先烧固件：**
- 用数据线接好板子 → 点状态栏 **→（Upload）** 图标。
- 认不到串口的话，点状态栏那个「插头/COMx」位置手动选端口。

**再烧 LittleFS（视频音频）：**
这个没有图标，要用命令。点状态栏左边那个**蚂蚁图标**展开 PlatformIO 面板，
或按 `Ctrl+Shift+P` 输入 `PlatformIO: New Terminal` 打开终端，输入：
```
pio run -t uploadfs
```
这会把 `data/` 打包成 LittleFS 镜像烧进 spiffs 分区。
（如果你 target.mjpeg / output1.mp3 是运行时下载的，data/ 里可以只放 def/ 那三个待机视频。）

---

## 6. 看日志，确认 PM 真的开了 ✓

点状态栏 **🔌（Serial Monitor）** 图标，波特率已设 115200。复位板子，看开机日志：

- **成功**：你会看到代码里那行
  `[PWR] ✓ 自动 light-sleep 已开启 (40~240MHz)`
  —— 说明 `CONFIG_PM_ENABLE` 生效了，CPU 在放锁时会自动降到 40MHz + light-sleep。
- **还是 ⚠**：说明 `custom_sdkconfig` 没吃进去。检查 `platformio.ini` 那几行有没有
  写对、有没有真的用 pioarduino 平台（不是官方 espressif32），然后**清理重编**
  （状态栏 🗑 Clean → 再 Build）。`custom_sdkconfig` 改动有时要 Clean 才重新编 IDF。

确认 ✓ 之后，**重新测 L2 待机电流**。CPU 那块应该明显塌下去。

---

## 7. 核对硬件设置（重要！对照你的 Arduino IDE 工具菜单）

`platformio.ini` 里的 Flash/PSRAM 设置我按 **WROOM-1 N16R8（16MB Flash + 8MB OPI PSRAM）**
填的——这是你分区表 16MB 布局推出来的最可能配置。但**请务必核对一遍**，错了会不断重启或 PSRAM 失败：

打开你的 **Arduino IDE → 工具** 菜单，读这几项，和 `platformio.ini` 对照：

| Arduino IDE 工具菜单         | 对应 platformio.ini                          | 我填的值        |
|------------------------------|----------------------------------------------|-----------------|
| Flash Size                   | `board_upload.flash_size`                    | 16MB            |
| PSRAM                        | `board_build.arduino.memory_type` 的后半     | OPI → `qio_opi` |
| Flash Mode                   | `board_build.flash_mode`                     | QIO             |
| Partition Scheme             | `board_build.partitions`                     | partitions.csv  |

**最容易错的是 PSRAM 那项**：
- 如果你 Arduino IDE 里 PSRAM 选的是 **OPI PSRAM** → 保持 `qio_opi`（已填）。
- 如果选的是 **QSPI PSRAM** → 改成 `board_build.arduino.memory_type = qio_qspi`。
- 如果你那项是灰的/没选过但 PSRAM 能用，多半是 OPI，先用 `qio_opi`。

烧完如果串口疯狂打印重启或 `PSRAM 池分配失败`，99% 是这里，回来改 memory_type。

---

## 8. 开了 PM 之后要注意的副作用（别慌）

light-sleep 不是免费的，开完观察这几点：

1. **背光 / 舵机 PWM 可能受降频影响**。LEDC 频率若跟着 APB 走，降频时背光可能轻微闪、
   舵机可能抖。你播放时本来就持锁 240MHz 不睡，所以主要看 L1/L2 待机时。若有问题，
   说明 LEDC 没用独立时钟源，可单独处理。
2. **串口日志降频时可能偶发乱码**，正常现象，调试想稳就临时持锁。
3. **持锁期间不睡是对的**：你播放视频/音频、配网、pos 串流都 `cpuBoost(true)` 持
   `ESP_PM_CPU_FREQ_MAX` 锁，这期间芯片保持 240MHz 不 light-sleep，符合预期。
   只有 L1/L2 待机放锁后才真正自动睡。
4. **WiFi 不会断**：你要求待机不断网。PM + tickless + WiFi modem-sleep(DTIM) 三者配合，
   CPU 在 beacon 间隙睡、DTIM 到了醒来收下行，关联不丢——这正是"待机不断网"能省电的唯一正解。
   配合把 `applyWifiPowerSave()` 里 L1/L2 改回 `WIFI_PS_MAX_MODEM`（你之前砍成 MIN 了）效果更好。

---

## 9. 常见报错速查

- **下载平台/库超时**：国内网络常见。多试几次，或给 PlatformIO 配代理
  （`Ctrl+Shift+P` → `PlatformIO: ...` 设置里加代理）。

- **`'xxx' was not declared`**：见第 3 节，加前向声明。

- **NimBLE / LovyanGFX 编译报一堆类型错**：检查 `lib_deps` 版本是否被改动；
  必须是 NimBLE-Arduino@2.5.0、LovyanGFX@1.2.21、ArduinoJson@6.21.6，
  和你 Arduino IDE 里一致。版本一漂 API 就对不上。

- **QMI8658 找不到 / API 对不上**：`lib_deps` 里我用的是 git 地址
  `https://github.com/lahavg/QMI8658-Arduino-Library`。如果它拉到的版本和你
  Arduino IDE 的 1.0.1 不一致导致报错，把这行改成指定 tag（若仓库有），
  或把你 Arduino 库目录里那个 QMI8658 文件夹整个复制到工程的 `lib/` 下（PlatformIO 的
  `lib/` 目录里的库优先于 lib_deps）。

- **烧录后不断重启 / PSRAM 失败**：见第 7 节，改 `memory_type`。

- **`custom_sdkconfig` 没生效（还是 ⚠）**：Clean（🗑）后重 Build；确认 platform 是 pioarduino。

---

## 10. 这步做完之后的下一步（省电收尾）

PM 开了、L2 的 CPU 塌下去之后，按之前聊的，还能再榨：
1. `applyWifiPowerSave()` 里 L1/L2 用 `WIFI_PS_MAX_MODEM`（你之前统一成 MIN 了，最省的那档被砍了）。
2. `audTask` 空闲时别 20ms 空转，改成阻塞等队列（参考 imuTask 的写法），把睡眠窗口拉长。
3. L2 主循环 / web 空闲轮询从 100ms 拉到 200~500ms。

待机不断网的物理下限大概在十几 mA（WiFi 连着的代价）。要再低就只能断网/deep-sleep，
但那和你"待机不断网"的约束冲突，所以十几 mA 基本就是你的目标地板。
