# 语音链路三端联合仿真（回归测试）

把三端代码忠实移植成可执行模型，接一个可注入 **单向延迟 / 抖动 / 丢包 / 滞留乱序** 的虚拟
MQTT broker，用离散事件时钟跑完整协议。改动 `web.cpp` 的 UVA 引擎或 `voice_service.dart`
之后跑一遍，确认没有回归。

| 模型 | 移植自 |
|---|---|
| `Device` | `src/web.cpp` — `uvStartApp/uvFillWindow/uvPump/uvOnAck/uvStop/uvService`（含 uint32 减法语义） |
| `App` | `APP/V1.0/v1_0/lib/services/voice_service.dart` — `_onControl/_onData/_finish/_ack` + uid/xferId 双键去重 + async 存盘延迟 |
| `Peer` | `src/web.cpp` — `handlePeerVoice` + `checkPeerVoiceRxTimeout` |

## 跑法

```bash
python uv_joint_test2.py   # 全场景 × 三个档位(CURR/P123/FULL)
python sweep.py            # 超时参数扫描 + 失败原因归因 + 解耦专项
python final.py            # 最终回归表（发版前跑这个）
python race.py             # ACK 竞争：RTT 递增下的 BEGIN 重传竞争
python race2.py            # ACK 竞争：上行乱序（broker 排队尖峰）
```

## ACK 竞争/冒险（已修复，勿回退）

高延迟 broker 会把下面几类竞争放大。`race2.py` 能复现第 1 条：

| # | 竞争 | 触发条件 | 后果 | 修复 |
|---|---|---|---|---|
| 1 | **伙伴接收端 `PV_BEGIN` 非幂等** | 上行乱序，先发的 BEGIN 后到，落在数据已开始落位之后 | 接收端 `next` 归零 + 截断临时文件，与发送端窗口左沿永久错位，一路重传到"重试超限" | 同 xferId 的 BEGIN 幂等回当前 `next`，绝不重置 |
| 2 | 滞后 ACK 把 `WAIT_DONE` 拉回 | 任何 RTT 抖动 | 数据已全确认却回退重传几十 KB + 多绕一个 RTT | `ackHigh` 单调水位，丢弃低于水位的 ACK |
| 3 | `vmMarkBadSlot` 终结正在发送的另一路 | 一路读源失败时另一路在传 | 删掉对方正在读的文件 | 只终结非 `DST_SENDING` 的通路 |
| 4 | 账本跨任务改写 | micTask 在 webTask `vmSave()` 序列化途中抢占 | 写进 flash 的账本撕裂 | micTask 只置标志，入账只在 webTask 做 |
| 5 | App `_finish` 的 async 窗口 | 存盘期间设备重发 `PV_END` | `_rx` 已删、`_doneAt` 未写 → 一个回应都发不出，设备存活时钟不刷新 | `_finishing` 表，窗口期回 `PV_ACK{next:size}` |

实测（上行 35% 排队尖峰 +3s，40 个种子）：竞争 1 修复前伙伴收齐率 **78%**，修复后 **100%**；
加 30% 丢包时 **85% → 100%**。

| 6 | **旧会话的 `PV_BEGIN` 抢占新会话** | 会话1 失败后会话2 开始，会话1 被 broker 压了十几秒的 BEGIN 才到 | `xferId` 是随机数没有顺序性，接收端无法区分"新会话"和"迟到帧" → 无条件覆盖，把已传 86% 的会话2 清零 | 抢占保护：当前会话 `PV_PREEMPT_GUARD_MS`(3s) 内收过数据就拒绝抢占，回 `PV_BUSY` |

竞争 6 定向实测（`stale_begin2.py`）：修复前会话2 传到 `208000/240000` 被打断 → `重试超限`，
总耗时 **59.5s**；修复后一次成功，**17.4s**。

---

# 预防：四条结构性规则

逐个打补丁治标不治本。下面四条规则覆盖了上述全部 6 个竞争，**新增协议逻辑时按这四条自检**。

### 规则 1：入站帧处理必须幂等
同一条消息重复处理 N 次，结果必须与处理 1 次相同。MQTT 本身就允许重复投递，broker 排队还会乱序。
- 数据帧：按 offset 幂等落位（重复片不重写、空洞片不写）
- 控制帧：`PV_BEGIN` 同 xferId 幂等回当前进度；`PV_END`/`PV_DONE` 用去重窗口幂等
- **自检工具**：`fuzz.py` 的复制注入直接测这条

### 规则 2：只信任单调量，不信任瞬时量
需要判断"新 / 旧"的地方，必须依赖单调量。`xferId` 是随机数——**只有唯一性，没有顺序性**。

| 判断 | 错误做法 | 正确做法 |
|---|---|---|
| 这个 ACK 是不是旧的 | 比 `next` | `ackHigh` 单调水位 |
| 这个 BEGIN 是不是旧的 | 比 `xferId` | `lastRxMs` 活跃度（时间单调） |
| 这条内容收过没有 | 比 `xferId`（会话级，会变） | `uid`（内容级，持久化） |

### 规则 3：单一所有者
每块可变状态只有一个写者，其他人只能读或发请求。

| 状态 | 唯一写者 |
|---|---|
| 账本 `s_vm` | webTask（micTask 只置 `s_enqReq` 标志） |
| 语音文件删除 | `vmSettle()` |
| 会话 `UvSession` | 拥有它的那条通路 |
| 接收态 `s_pvRx*` | webTask |

### 规则 4：不留悬空态
异步状态转移必须有**显式**的中间态，并定义该态下所有入站帧的响应。
- App 存盘期 → `_finishing` 表（回 `PV_ACK{next:size}`）
- 设备投递中 → `DST_SENDING`

---

# 自动化：让竞争被工具发现，而不是靠人审查

```bash
python fuzz.py 400      # 对抗性 broker：25% 复制 + 25% 乱序 + 每步不变量断言
```

基线：400 个种子、28350 个注入的重复帧，**零不变量违反、两条通路 100% 收齐**。
改协议后跑一遍；出现 `★ 首次违反` 就说明引入了新竞争，输出里带完整轨迹。

固件里也埋了同一组不变量（`UV_INV` 宏，**只打日志不 panic**）。真机日志出现 `[UVINV]` 时，
把当时的链路条件搬到 `fuzz.py` 里复现。

## 不变量清单（改协议时必须继续成立）

| # | 不变量 | 谁依赖它 |
|---|---|---|
| I-A | 发送端 `sent >= next` | `uvFillWindow` 的 uint32 减法 |
| I-B | 发送端 `next <= size` | 边界安全 |
| I-C | 同一 xferId 内接收端 `next` 单调不减 | 竞争 2 的 ACK 水位过滤 |
| I-D | 文件存在 ⟺ 账本 `used`；删除权只在 `vmSettle` | 双通路解耦 |
| I-E | 同一条语音对同一目的地最多成功投递一次 | uid 去重 |
| I-F | 会话结束入账前，`session.uid == 账本[slot].uid` | 槽位会被环形淘汰复用 |

## 静态验证发现的槽位复用问题（仿真覆盖不到，靠代码走查）

`mic.cpp` 槽满时会**环形淘汰最旧一条并复用它的槽号**。账本是按槽号索引的，于是有两处要防：

1. **新录音继承旧账本记录**：淘汰只删文件、不清账本，`vmRegisterNew` 的扫描又跳过 `used==1` 的槽
   → 新语音拿到旧 uid（App 按 uid 去重丢弃）和旧状态（旧记录若是 `app=DONE`，新语音**永远不发给 App**）。
   修复：`voiceEnqueue` 把槽号传给 webTask，登记前先清该槽旧记录。
2. **旧会话的结果记到新消息头上**：会话开始到结束期间槽位可能已被复用，`uvStop` 若直接按槽号入账，
   一条旧会话的 DONE 会把新录的那条标成已送达。修复：入账前校验 `uid`（不匹配则不入账，见 I-F）。

只依赖标准库，无需 pip 安装。

## 基线（当前实现，`final.py` 输出）

```
场景                  CURR App  FINAL App   CURR放大  FINAL放大
理想 RTT120               100%      100%     1.00x     1.00x
云端 RTT600               100%      100%     1.00x     1.00x
ACK滞留乱序30%@1.5s        100%      100%     1.02x     1.10x
丢包25%                    95%      100%     3.55x     2.53x
丢包40%+RTT400             80%      100%     5.82x     3.38x
丢包55%+RTT600             35%       83%     8.47x     4.88x
坏槽 voice_0=0B             0%      100%     0.00x     1.00x
双通路并发                 100%      100%     2.01x     2.01x

解耦专项
App离线录音 → 伙伴收 / App 30s后上线补投 :  CURR App 0%   → FULL 100%
伙伴离线录音 → App收 / 伙伴 20s后上线补投 :  CURR 伙伴 0%  → FULL 100%
```

## 改超时参数前必看

关键不变量：`UV_STEP_MAX_RETRY × UV_STEP_TIMEOUT_MS < UV_LIVENESS_MS`
否则重传次数还没用完就被"对端无响应"砍掉。

参数扫描实测（丢包 40% + RTT400，60 个种子）：

| STEP / LIVENESS / MAX_RETRY | 成功率 |
|---|---|
| 1200 / 6000 / 8（改造前） | 80% |
| **3000 / 15000 / 6** | **33%** ← 拉长 STEP 是灾难 |
| 1200 / 30000 / 20（当前） | **100%** |
| 800 / 30000 / 30 | 100%（更快但 RTT 余量薄） |

**STEP 必须保持短**（快速重传是高丢包下的生命线），要放宽的是 MAX_RETRY 和 LIVENESS。

## 不覆盖的部分

- EMQX Serverless 限速的正反馈（模型丢包率固定，不随发送速率变化 → cwnd 的真实收益应更大）
- PubSubClient 的 publish 阻塞/失败、TLS 半包、`setSocketTimeout(3)` 的 3s 阻塞
- ESP32 的 flash 写冻结 cache、TWDT、任务抢占
- iOS 后台冻结的真实时序（`_finish` 用固定 100ms 建模）
- EMQX ACL / presence 主题的真实配置

这些只能靠真机验证。
