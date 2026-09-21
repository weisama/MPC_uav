# 参考仓库清单（本地只读副本）

> **地点：`~/refs/`，在仓库之外，不进 git。**（Q18=a 决策：仓库外 + 持久 + 钉 commit）
>
> **为什么不在 `/tmp`**：2026-09-20 那一轮发现 `/tmp/repo-survey/` 等**全部被清空**，
> 导致当时**无法复核任何第一手结论** —— 而 Q3 定的纪律正是「结论必须第一手可复核」。
> `~/refs/` 就是为堵这个洞而建的。
>
> **重建方式**：按本表的仓库地址与 commit 重新克隆即可。核验用 `git log -1 --format=%H`。

## 1. 清单与核对结果（2026-09-20 实测）

| 本地目录 | 仓库 | 钉住的 commit | 日期 | 与项目文档记录比对 |
|---|---|---|---|---|
| `Fast-Drone-XI35/` | `git@github.com:Longer95479/Fast-Drone-XI35.git`（分支 `board`） | **`8c80ddb511c814e42576d0b8f3532e9b776e1224`** | 2026-02-02 | ✅ **一字不差**（`code-inventory.md` §A.1 记 `8c80ddb`）。2026-09-20 经 `git ls-remote` 复核**上游未漂移** |
| `xi35-sparse/` | 同上（稀疏克隆，只含 `src/realflight_modules/px4ctrl`） | `8c80ddb` | 2026-02-02 | 便于快速读取，与上面同 commit |
| `px4-mpc/` | `git@github.com:DISCOWER/px4-mpc.git` | **`3e541afb05350e30a0867420a5df40dbdfa92c13`** | 2026-06-15 | ✅ commit / 日期 / **40 个文件** / BSD 3-Clause（`Copyright (c) 2022, JaeyoungLim`）全部吻合。⚠️ **原文档记的 URL `PX4/px4-mpc` 不存在**，已在本轮修正 |
| `px4_msgs/` | `git@github.com:PX4/px4_msgs.git`（分支 `release/1.14`） | **`ffb6e80e1c17e5714395611a020c282a87af8fa4`** | 2023-11-11 | ✅ 与文档记的 `ffb6e80e` 吻合 |
| `OmniNxt/` | `git@github.com:HKUST-Aerial-Robotics/OmniNxt.git` | `79ce1d4b69a5a8c861417716adaac00c2bc6f594` | 2025-03-09 | 文档未记 commit |
| `agilicious/` | `git@github.com:uzh-rpg/agilicious.git` | `50bba456156f6f438bea212c643b02e162442fa7` | 2023-03-07 | 文档未记 commit。⚠️ **许可证禁止再分发，只读不抄** |
| `Fast-Drone-250/` | `git@github.com:ZJU-FAST-Lab/Fast-Drone-250.git` | `8ff64274b161bb829653faf5946656658e7e2c9f` | 2022-11-09 | 文档未记 commit。**FSM 设计的真正源头** |
| `Sim2real-UAV-Params-/` | `git@github.com:Huan-qun/Sim2real-UAV-Params-.git` | `b7aeebb71cc708445fbac70a4863c736d4674630` | 2026-08-30 | `z0.params` 的出处；`S2R/assets/vrpn_listener.zip` 见 `mocap-setup.md` §6.2 |
| `Nxt-FC/` | `git@github.com:HKUST-Aerial-Robotics/Nxt-FC.git`（分支 `main`） | `153b7d13609e870a8abc6ff909461a13ea2d69f8` | 2024-08-13 | ⚠️ **不是固件源码仓库**（仅 51 个文件：硬件 + 预览版二进制 + submodule） |
| **`px4-z0/`** | `git@github.com:Peize-Liu/PX4-Autopilot.git` | **`5ba2e91ff227fe4da0133371e88dc81b038bb81e`** | 2024-07-02 | ★ **z0 固件的真身**。以 **fetch-only** 方式保存（无工作树）；读文件用 `git show FETCH_HEAD:<path>` |
| `px4-v1.14.3/` `px4-v1.15.0/` `px4-v1.16.0/` | `git@github.com:PX4/PX4-Autopilot.git`（三个 tag） | `1dacb4c` / `30e763b` / `6ea3539` | — | 作对照用（如 ADC 通道的版本边界）。**fetch-only，无工作树** |

## 2. ★ z0 固件真身是怎么找到的

`HKUST-Aerial-Robotics/Nxt-FC` 的 `.gitmodules`：

```
[submodule "PX4-Autopilot"]
	path = PX4-Autopilot
	url = https://github.com/Peize-Liu/PX4-Autopilot.git
	branch = develop
[submodule "Nxt-FC-Hardware"]
	path = Nxt-FC-Hardware
	url = https://github.com/Peize-Liu/NxtPX4-Hardware.git
```

`Nxt-FC/main` 指向 `PX4-Autopilot` @ `5ba2e91f`，该 commit 的 `boards/hkust/` 下**确实有
`nxt-dual` 与 `nxt-v1`**。而 PX4 官方上游 **v1.14.3 无 `boards/hkust`，v1.15.0 起才有** ——
这解释了为什么 z0 的固件只能来自 fork。

> ⛔ **2026-09-20 订正一处推理（~~结论不变，理由换掉~~）**：本文档曾由「`nxt-dual` 只在 PX4 ≥ 1.15.0 存在」
> **推出**「所以 z0 固件是 v1.15 世代」。**这个推理是错的** —— 上游和 fork 是两回事：
> fork 的 `feat_deprecated_v1.14.x_high_imu` 分支（2024-03-20）**就带 `nxt-dual`**。
> 「板级目标存在」**不能**推出世代。
> ✅ ~~**真正扛住结论的是直接证据**：fork **四条分支**的 `msg/OffboardControlMode.msg`
> **全部**含 `thrust_and_torque` + `direct_actuator`，且**全部**有 `srv/VehicleCommand.srv`
> （见 §2.7）。**从"看消息定义"取代"看板子存在与否"。**~~
>
> ☠️ **2026-09-21 再订正：块里那句「结论不变」也错了 —— 结论变了。**
> 上面这条推理**确实**证出了「fork 的 `msg/` 是 v1.15 世代」，**但它证的是 fork，不是 z0**。
> 而 z0 的飞控**不跑 fork**（真身 `4a0e65f2`，`msg/` 与上游 v1.14.3 全同）⟹
> **「z0 是 v1.15 世代」这个结论本身就是错的**（见 §2.5(d)）。
> 教训叠了一层：**「换了理由」也要再问一次「那结论还是同一个对象吗」** ——
> 理由从"板子存在"换成"消息定义"，看着更硬，**但两次说的都是 fork**，
> 而问题从来是**飞控里烧的是不是 fork**（这一问直到 09-21 拿到 `flight_custom_version` 才回答）。

> ✅ **2026-09-21 了结 —— 这条反而是佐证（当时以为的疑点，实为同一个数）。**
> `z0.params` 头声明 `Git Revision: 4a0e65f233000000`，当时判「与 `5ba2e91f` 不一致」是个疑点。
> 真身查明后（§2.5(d)）它**逐字对上**：飞控实测 8 字节 `00 00 00 33 f2 65 0e 4a` 小端还原
> = `4a0e65f233`，而 `4a0e65f233000000` 正是把该 hash 前 16 个十六进制字符按
> `& 0xFFFFFFFFFF000000` 截断后的值 —— **两处是同一个 8 字节值的两种写法**。
> 「首飞前必须在 QGC 里记录飞控实际版本号」这件事**也已经做了**：就是 §2.5(c)(d) 的 MAVLink 实测。
>
> ⚠️ 但要留一手：`z0.params` 自标「**尚未在 z0 的飞控上核实**」，其出处是上游
> `Sim2real-UAV-Params-` 的存档。所以它是**旁证**（说明同源的那台机器也跑这份固件），
> **不是** z0 飞控的读数。z0 的读数来自 §2.5(d)——那份才是第一手。

## 2.5 ★★ ~~z0 固件是「v1.15 世代」，不是 1.14~~ → **2026-09-21 定案：本节 (a)(b) 结论作废，实为 v1.14.3 系**（2026-09-20）

> ⚠️⚠️ **2026-09-21 定案 —— 读 (a)(b) 之前先读 §2.5(d)。**
> 飞控报出的 git hash 是 **`4a0e65f2`**（commit `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`，
> `Minderring/PX4-Autopilot`，**"add micoair743-aio to PX4-Autopilot v1.14.3"**，2024-11-18）。
> 它的 `msg/`（185 个，含 `msg/CMakeLists.txt`）与**上游 v1.14.3 逐字节相同**（185/185）；
> 与 **`px4_msgs release/1.14`** 则是在**共有的 184 个文件**上逐字节相同
> （唯一差异是文件**集合**——`px4_msgs` 不带 `msg/CMakeLists.txt`，那是 PX4 自己的构建文件）。
> ⟹ (a) 那张「`release/1.14` 不兼容」表、(b) 的「v1.15 世代」判定，**都拿错了参照物（fork），作废**；
> §2.6 的决策与 Phase 2 的选版**都要按 §2.5(d) 改**。
> **本节 (a)(b) 仅作留存，不要再引用它们做判断。**

这一节记录 Q24 的验收步骤**执行结果**。结论是**验收失败**，并连带推翻项目的一个前提标签。

### (a) `px4_msgs` `release/1.14` 与 z0 固件**不兼容**

按 Q24=a 的验收步骤，把 `px4_msgs` `release/1.14` @ `ffb6e80e` 的 `msg/*.msg` 与
`Peize-Liu/PX4-Autopilot @ 5ba2e91f` 的 `msg/` **逐字节比对**（firmware 侧 216 个 blob 全部用
`git hash-object` 对过 tree SHA，**100% 覆盖，非抽样**）：

| 结果 | 数量 |
|---|---|
| 两边共有 | 184 |
| **逐字节相同** | **143** |
| 有差异 | 41 —— 其中 **28 个是字段布局差异**、3 个仅常量/队列长度、10 个仅注释 |

- **只在固件里、`px4_msgs` 没有**：**32 个**（含 `srv/VehicleCommand.srv` 与
  `ArmingCheckRequest/Reply`、`ParameterSetValueRequest/Response` 等**服务**定义 ——
  `release/1.14` 里**连 `srv/` 目录都没有**）。
- **只在 `px4_msgs` 里、固件没有**：**0 个**。

**关键消息**：

| 消息 | 判定 |
|---|---|
| `VehicleRatesSetpoint` | ✅ **逐字节相同**（仅注释 `NED frame`→`FRD frame`） |
| `VehicleAttitudeSetpoint` | ✅ **逐字节相同**（仅注释） |
| `VehicleOdometry` / `TimesyncStatus` / `ActuatorMotors` | ✅ 相同 |
| `BatteryStatus` | ✅ **wire 相同**（仅注释） |
| **`OffboardControlMode`** | ❌ **破坏性**：`bool actuator` → `bool thrust_and_torque` + `bool direct_actuator` |
| **`VehicleCommand`** | ❌ 破坏性：`source_component` `uint8` → `uint16`（`VehicleCommandAck` 同） |
| **`VehicleStatus`** | ❌ 破坏性：字段增加 + `ARMING_STATE_*` / `NAVIGATION_STATE_MAX` **重新编号** |
| **`VehicleLocalPosition`** | ❌ 破坏性：**中间插入**字段（`heading_var` / `unaided_heading` / `tilt_var`） |
| **`VehicleControlMode`** | ❌ 破坏性：标志位**重排** + 新增 `flag_control_allocation_enabled` / `source_id` |
| `SensorGps` | ❌ 硬类型变更：`int32 lat/lon/alt` → `float64 latitude_deg/...` |

> ✅ **一条对我们有利的巧合**：Phase 2/3 真正要发的 `VehicleRatesSetpoint` **恰好是逐字节相同的**。
> 但**周围的配套消息**（`OffboardControlMode` —— offboard 心跳必需！、`VehicleStatus`、`VehicleLocalPosition`）
> **全是破坏性的**。所以**不能用** release/1.14。

⚠️ **`px4_msgs` 那边没有版本号机制能救你**：`release/1.14` 与 fork **都没有** `msg/versioned/`、
`*_v1.msg` 或任何版本标记（该机制 **v1.16 才有**）。两边必须**原始 `.msg` 文本一致**，而它们不一致。
DDS 话题名**完全一样**（`/fmu/in/offboard_control_mode`、`/fmu/out/vehicle_status` …都在 fork 的
`src/modules/uxrce_dds_client/dds_topics.yaml` 里逐字存在）→ **正是那种"不报错、只是消息没人收"的静默失败**。

### (b) 因此：**"z0 是 PX4 1.14.x" 这个标签是错的**

`5ba2e91f` 的 `msg/OffboardControlMode.msg` 有 `thrust_and_torque` + `direct_actuator`。
**上游对照（第一手）**：

| 版本 | `OffboardControlMode.msg` |
|---|---|
| PX4 **v1.14.3**（tag `1dacb4c`） | `bool actuator` |
| PX4 **v1.15.0**（tag `30e763b`） | **`bool thrust_and_torque` + `bool direct_actuator`** |
| **z0 fork `5ba2e91f`** | **`thrust_and_torque` + `direct_actuator`** ← **与 v1.15.0 一致** |

旁证：`VehicleLocalPosition.heading_var`、`VehicleCommand.source_component` 为 `uint16`、
存在 `srv/` 目录、**无** `msg/versioned/`（v1.16 才有）—— 全部指向**v1.15 世代**。

> 📌 **`5ba2e91f` 是 2024-07-02 的 `main`**，早于 v1.15.0 的发布，所以它**既不是 v1.14 也不是 v1.15.0 正式版**，
> 而是走向 v1.15 的一个中间 commit。**→ `px4_msgs` 的 `release/1.15` 也未必逐字节相同。**
> **最稳的做法：从 fork 自身的 `msg/` 生成 `px4_msgs`**（或把它 `msg/` 下 216 个定义 +
> `srv/VehicleCommand.srv` 直接打包成包）。**这需要在 Phase 2 的 S1 步骤里定案。**
>
> ⚠️ **一处仍然悬空**：`z0.params` 头写 `Version: 1.14.3 dev`。但该头**来自上游参考机的参数文件**
> （`Huan-qun/Sim2real-UAV-Params-` 的 `uav1.params`），**未必描述 z0 自己**。
> **→ 首飞前在 QGC 读飞控实际报出的版本号，这是唯一的终审。**

### (c) ★★★ 2026-09-21 **终审读数已拿到：飞控自报 1.14.3 —— 与 (b) 的结论冲突，此事未决**
> ☠️ **2026-09-21 后续定案：本节「未决」已结，且本节下半部分（`flight_custom_version` 解不出 hash
> 那段）有**两处错误结论**，见 §2.5(d)①。本节仅留存，**不要引用**。

上一段要的那个「唯一终审」，2026-09-21 通过 TELEM2 ↔ Jetson 串口1 拿到了（**第一手读数，
不是 QGC 截屏，是 MAVLink `AUTOPILOT_VERSION` 的原始字段**）：

| 字段 | 读数 |
|---|---|
| `HEARTBEAT` | `autopilot=12`（PX4）、`type=2`（四旋翼）、`system=1 component=1` |
| **`flight_sw_version`** | **`17695488`（`0x010e0300`）** → 按 PX4 编码 `(major<<24)\|(minor<<16)\|(patch<<8)` 解出 **`1.14.3`** |
| `flight_custom_version` | 原始 8 字节 = `00 00 00 33 f2 65 0e 4a` → ~~**无法解出 git hash**~~ **☠️ 错判；实为 `4a0e65f233`，见 §2.5(d)①** |
| `vendor_id` / `product_id` | `7052` / `54` |
| `capabilities` | `0xecff` |

#### ☠️ ~~`flight_custom_version` 这一次**帮不上忙** —— 且工具原本会给出一份假证据~~
> **2026-09-21 定案：本小节整段作废，两处都错。** (1) 该字段**帮得上忙**，解出的就是 git hash；
> (2) 所谓「工具会给出假证据 / 已补第二道闸」—— 那道闸门**本身就是错的、已删除**，
> 且它会把**任何**真固件都挡掉。完整更正见 §2.5(d)①，此处保留原文仅为记录这段弯路。

按上游源码（v1.15.0 `mavlink_main.cpp:1082-1087`，本仓库工具已第一手核实并抄在 docstring 里），
`flight_custom_version[3:8]` 应当是 git hash 的**前 5 个 ASCII 字符**（只可能是 `0-9a-f`）。

而 z0 报出的 `[3:8]` = `33 f2 65 0e 4a`，其中 **`0x0e` 与 `0xf2` 不可打印** ——
**不可能是任何 git hash 的 ASCII 表示**。

> ⟹ **该字段在 z0 的固件上不携带可读的 git hash。** 无法用它核对 `5ba2e91f`，
> 也**不能**据此说"与 `5ba2e91f` 不一致"。
>
> ⚠️ 本节初稿曾把这一串硬按 16 进制打成 `4a0e65f233` 并判"与 5ba2e91f 不一致，§2.5 要重开" ——
> ~~**那是错的，已作废**~~。**☠️ 2026-09-21 更正：这一段自己也错了两处** ——
> (1) 那个"硬凑"出来的 `4a0e65f233` **才是对的**（它确实是 hash）；
> (2) 「已给 [`mavlink_probe.py`](../../tools/prereq-evidence/mavlink_probe.py) 补上第二道闸
> （`b[3:8]` 必须全在 `0-9a-f` 内）」—— **那道闸门是错的，已删除**，它会把任何真固件都挡掉。
> 真正的教训反了过来：**错在拿"我以为的编码"去否证"实测的字节"**。详见 §2.5(d)①。

#### ★ 冲突本身：`1.14.3` vs (b) 的「v1.15 世代」

> ☠️ **2026-09-21 已解**：不是「飞控版本号没随上游升版」这类假设中的任何一条 ——
> 而是**飞控根本没跑 fork**。(b) 的判据（fork 的 `msg/`）从一开始就不是 z0 的东西。
> 破案手段就是下面「能立刻做的判别」的第 2 项。结论见 §2.5(d)。

- (b) 的判据是 fork `5ba2e91f` 的**源码** `msg/` 定义（`OffboardControlMode` 带
  `thrust_and_torque` ⟹ 与 v1.15.0 一致）
- 而**飞控自己报的是 `1.14.3`**
- §2.5 上一段把「读飞控实际报出的版本号」定为**唯一终审** ⟹ **终审与 (b) 不符**

**可能的解释（都未验证，不要挑一个当结论）**：

| # | 假设 | 怎么验 |
|---|---|---|
| 1 | z0 的飞控**跑的不是** `5ba2e91f`，而是另一支（如厂商 nxt-dual 的出厂固件，基于 1.14.3） | 读参数表 / 看它是否带 fork 特有分支的痕迹 |
| 2 | 厂商分支的版本号**没有随上游升版**（源码是 v1.15 世代但 `PX4_VERSION` 仍是 1.14.3） | 需要拿到该分支源码 |
| 3 | `flight_sw_version` 被厂商机制覆写（`flight_custom_version` 那 3 个被覆盖的字节正是同一套机制） | 看上游 `px4_firmware_vendor_version()` 的语义 |

#### ⚠️ 后果：**§2.6 的决策与 Phase 2 的 `px4_msgs` 选版，前提现在是被挑战的**

- (a) 那张"`release/1.14` 不兼容"的表是拿 **fork 源码**比对的 —— 若飞控实际跑的是 1.14.3 系固件，
  该结论**可能反过来**。
- ⟹ **在 Phase 2 的 S1 定案前必须解决这一条**。在此之前，**不要把 (b) 当既成事实往下游引**。

#### ✅ 能立刻做的判别（~~尚未做~~ **2026-09-21 已做** —— 第 2 项就是破案手段）

> 结果见 §2.5(d)：**第 2 项**（拿厂商分支源码直接看）成了关键；第 1 项（读参数表）
> **没用上** —— 在没读参数表之前就定案了。第 3 项（uXRCE-DDS）留给 Phase 2。

1. **用 MAVLink 读飞控的完整参数表**，与 [`docs/assets/z0.params`](../assets/z0.params) 对照。
   该文件头自标「尚未在 z0 的飞控上核实」—— 这一步能同时**第一手核实 `MAV_1_CONFIG` /
   `SER_TEL2_BAUD`**（此前只有"强证据、非第一手"）。
2. 拿该分支源码直接看 `version.h` / `px4_firmware_vendor_version()`。
3. 等 DDS 通了以后，直接看运行固件实际的话题字段布局（最硬，但要先搭 uXRCE-DDS）。

### (d) ★★★ 2026-09-21 定案：**z0 固件真身 = `4a0e65f2`，v1.14.3 系的厂商构建** —— (a)(b) 作废

#### ① 先结掉 (c) 里那个「取不到 hash」—— **那是我判错了，不是飞控没给**

(c) 说 `flight_custom_version[3:8]` 应当是 git hash 的 **ASCII** 字符、z0 的字节含
不可打印值所以「取不到」。**这个说法是错的。** 源码链（**出自运行固件自己的那棵树**，
commit `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`，不是拿别的版本代跑的）:

```python
# src/lib/version/px_update_git_header.py:86-93
git_version = subprocess.check_output('git rev-parse --verify HEAD'.split(), ...).strip()
git_version_short = git_version[0:16]          # ← 40 位 sha 的前 16 个十六进制字符
#   →  header 里写死：  #define PX4_GIT_VERSION_BINARY 0x{git_version_short}
```
```c
// src/lib/version/version.c:343
uint64_t px4_firmware_version_binary(void) { return PX4_GIT_VERSION_BINARY; }
// src/modules/mavlink/mavlink_main.cpp:1073-1078
/* use only first 5 bytes of git hash for firmware version */
const uint64_t fw_git_version_binary = px4_firmware_version_binary() & 0xFFFFFFFFFF000000;
const uint64_t fw_vendor_version = px4_firmware_vendor_version() >> 8;
memcpy(&msg.flight_custom_version, &fw_git_version_binary, sizeof(msg.flight_custom_version));
memcpy(&msg.flight_custom_version, &fw_vendor_version, fw_vendor_version_length);  // = 3
```

⟹ 塞进去的是 **uint64 的二进制镜像（小端）**，掩码只留高 5 字节 = hash 的**前 10 个字符**，
前 3 字节再被厂商版本号覆盖。**`[3:8]` 本来就不该是 ASCII** —— 那道「必须是 `0-9a-f`」的闸门
会把**任何**真固件都挡掉，是一道**反向的**错闸。已从
[`mavlink_probe.py`](../../tools/prereq-evidence/mavlink_probe.py) 移除，并补了
`--self-test`（用下面的真值对做交叉验证，见 ②）。

**解码（第一手，飞控 2026-09-21 报出）**：

| | 值 |
|---|---|
| 原始 8 字节 | `00 00 00 33 f2 65 0e 4a` |
| 前 3 字节 = 厂商版本号 | `00 00 00` ⟹ 本次构建**没有**厂商版本号 |
| `[3:8]` 小端还原 | **`4a0e65f233`** = git hash 前 10 位 |

**反向验证（这一步才是它可信的理由）**：由 sha 前 16 字符 `4a0e65f2331b23b9`
按上面的规则**正向推出**的 8 字节，是 `00 00 00 33 f2 65 0e 4a` —— **与飞控实测逐字相同**。
不是我拿实测字节去凑一个"看起来像"的字符串。

**另有两份独立旁证（都不依赖读源码）**：

1. [`docs/assets/z0.params`](../assets/z0.params) 头部自 commit `04c5991` 起就写着
   `Git Revision: 4a0e65f233000000`。把它与源码链推出的值并排看：
   `int("4a0e65f2331b23b9", 16) & 0xFFFFFFFFFF000000` = `0x4a0e65f233000000` —— **逐字相同**。
   ⚠️ 该文件自标「尚未在 z0 的飞控上核实」、出处是上游 `Sim2real-UAV-Params-` 的存档，
   所以它是「**同源的那台机器也跑这份固件**」的旁证，**不能替代**上面那份来自 z0 飞控自己的读数。
   （§2.5 头部那段「`z0.params` 与 `5ba2e91f` 不一致」的疑点，到这一步才闭合。）
2. **同一条消息里的另一个字段**：(c) 的 `flight_sw_version = 0x010e0300` ⟹ **`1.14.3`**。
   它与 hash 走的是**完全不同的机制**（编译期 `PX4_VERSION` 宏 vs git 头文件），
   却指向同一代固件 —— 互不依赖的两条线自洽。

#### ② 这个 commit 是什么

| 查询 | 结果 |
|---|---|
| GitHub API `/commits/4a0e65f233` | `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c` |
| 日期 / 作者 | **2024-11-18** / Minderring |
| 信息 | **`add micoair743-aio to PX4-Autopilot v1.14.3`** |
| 所属厂商分支 | `Minderring/PX4-Autopilot` 有 `micoair743-v1.14.3` 等分支（`git ls-remote --heads` 第一手） |
| **父 commit** | `71fd55d7816c355f7095e0c5105ff031e4c97871`（GitHub API 第一手）—— **不是** v1.14.3 的 tag `1dacb4c` |

> ⚠️ 所以「基于 v1.14.3」这句的依据是 **commit 标题自述 + `msg/` 逐字节全同**，
> **不是祖先关系**（它的父是 `71fd55d`）。`71fd55d` 与 tag `1dacb4c` 是否同一支系**未核**——
> 本地克隆是 `--depth 1`，走不了祖先链，所以**不猜**。这不影响我们要的结论
> （消息定义才是判据），但**别把它说成「v1.14.3 + 一个 patch」**。

把它**那个 commit 的 tree 精确取下来**（`git fetch --depth 1 --filter=blob:none <sha>`，
只取 commit+tree、不取 blob），做两件事：

**（i）`msg/` 逐字节比对（全部用 blob SHA 对，零抽样）**

| 对比 | 共有文件中 blob 不同的数量 |
|---|---|
| 固件 commit `msg/`（185 个） vs **上游 v1.14.3**（tag `1dacb4c`） | **0**（连文件名差异都没有） |
| 固件 commit `msg/` vs **`px4_msgs release/1.14`**（@ `ffb6e80e`） | **0**（唯一差异是 `msg/CMakeLists.txt`，那是 PX4 的构建文件，`px4_msgs` 本就不带） |

⟹ **飞控实际的消息定义 = 上游 v1.14.3 = `px4_msgs release/1.14`。**

**（ii）`boards/` 里有谁**

固件那棵树里是 `micoair/{h743, h743-v2, h743-aio}`，
**没有 `hkust/`**（上游 `hkust/` 要到 **v1.15.0 才出现**，v1.14.3 里根本没有 —— 第一手核对过
`git ls-tree v1.14.3:boards/`，且这两个克隆是 `blob:none` 而 **tree 是本地齐的**，故该否定结论成立）。

#### ③ 后果（**都要改**）

- **§2.5(a) 作废**：那张表是拿 **fork** 比的。对**真身在用的**基准（v1.14.3），`release/1.14`
  在**共有的 184 个 `msg/` 文件上 blob 全同**（差异只在文件集合：`msg/CMakeLists.txt`）。
- **§2.5(b) 作废**：「v1.15 世代」是对 fork 说的，而飞控不跑 fork。
- **§2.6 的决策前提被推翻**：原则「以 z0 固件真身为准」仍成立，但真身**不是 `5ba2e91f`**。
  Phase 2 的 `px4_msgs` 基准回到 **[`release/1.14` @ `ffb6e80e`]**（已实测逐字节相同）——
  **这正是 Q24=a 最初的表述**。
- **§2.7 的 ADC 缺陷与 z0 的固件无关**：那个 `ADC_BATTERY_CURRENT_CHANNEL = ADC12_CH(5)`
  在 `boards/hkust/nxt-dual/src/board_config.h` 里，而运行固件的树里没有这块板；
  它的 `boards/micoair/h743-aio` 写的是 `/* PC1 */ ADC1_CH(11)`，**与引脚相符、无此缺陷**。
  ⟹ ④「未中招」**有了解释**（不是运气），Phase 3「重编重刷 `pr_fix_hkust_nxt_dual`」的修法
  **对这块固件不适用，要重定**。
- **`CLAUDE.md` 里「PX4 1.14.x」这个标签反而一直是对的** —— 09-20/21 一度把它判成错的，是误判。

#### ④ 仍未定（**要人拍板，别自动往下改**）

1. **硬件到底是哪块板**：固件树里只有 `micoair/h743-*`，而本项目文档一直写「飞控 = MicoAir NxtPX4v2」
   （NxtPX4v2 的 PX4 目标名是 `hkust_nxt-dual`）。飞控报出的 `vendor_id=7052 / product_id=54`
   对不上上游任何板（上游 5 块 `hkust/*`、`micoair/*` 板的 `default.px4board` 都不写这两个 id，
   它们由厂商分支的 Kconfig 决定，上游看不到）。
   → **看实物/订单**，或读参数表（`SER_*` 端口的集合可区分板型）。
2. **§2.6 怎么改**：见 §2.6 的 ⚠️ 块 —— 属于用户决策，**签字前不要按旧表动手**。

> 📌 **方法教训（与 §0 那两条同源）**：09-21 那次错，错在**拿"我以为的编码"去否证"实测的字节"**，
> 而不是去查源码。结果是把一份**正确**的读数（`4a0e65f233`）当成假证据作废，
> 还顺手加了一道会把真数据全挡掉的闸门。**先读源码，再判读数。**

## 2.6 ★★ 已定决策（用户 Q31=a，2026-09-20）：**版本基准统一到 fork `5ba2e91f`**

> ⚠️⚠️ **2026-09-21：本决策的前提已被推翻（见 §2.5(d)），下表不可再照着执行。**
> 原则「以 z0 固件真身为准」**仍然成立**；但**真身不是 fork `5ba2e91f`**，而是
> **`4a0e65f2`（commit `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`，v1.14.3 系的 MicoAir 构建）**。
> ⟹ 下表「从 fork 构建 SITL」「从 fork 生成 `px4_msgs`」两条**都不再成立**；
> 正确基准是 **`px4_msgs release/1.14` @ `ffb6e80e`**（已实测：与真身**共有的 184 个 `msg/`
> 文件 blob 全同**，唯一集合差异是 `msg/CMakeLists.txt`）。
> **改法待用户签字** —— 见 ticket [11](../../.scratch/phase1/issues/11-firmware-version-conflict.md)
> （`Status: needs-triage`）。**签字前不要按本表动手。**

> 这一条同时解决了 Q12（SITL 被控对象）与 Q24（`px4_msgs` 选版）的**共同上游问题** ——
> 它们的争执根源就是"以哪个 PX4 版本为准"。现在的答案是：**以 z0 固件真身为准。**

| 项目 | 决定 |
|---|---|
| **SITL 被控对象** | **从 fork `5ba2e91f` 构建**（不是上游 v1.14.3）。Q12 的 **gazebo-classic 选择照旧可用** |
| **`px4_msgs`** | **从 fork 的 `msg/` + `srv/` 生成**，逐字节一致。**`release/1.14` 已否决**；`release/1.15` 也不必试 |
| **Q24=a 的原表述** | 「锁定 `release/1.14` @ `ffb6e80e`，验收时 diff 若不同则改为匹配 fork」→ **验收失败，现在直接执行"改为匹配 fork"这半句** |
| **Q12「留在 1.14.x」** | 决策本身（**不刷固件**）不变；**「1.14」这个标签作废**，改为"留在 z0 固件真身 `5ba2e91f`" |

**可行性已核实（第一手）**：fork 支持两套 SITL ——

| | fork `5ba2e91f` | 上游 v1.14.3 |
|---|---|---|
| `4001_gz_x500`（gz-sim） | ✅ 有 | ✅ 有 |
| `Tools/simulation/gazebo-classic` | ✅ 有 | ✅ 有 |
| `ROMFS/.../init.d-posix/airframes/` 条目数 | **63 个**（两套仿真都在） | 较少 |

**构建代价**：`~/refs/px4-z0` 是 **fetch-only**（无工作树 / 无 submodule），**不能直接构建** ——
需要**带 `--recursive` 的完整 clone**（fork 有 **23 个 submodule**）。
其中对本项目要紧的三个都在：`src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client`、
`Tools/simulation/gazebo-classic/sitl_gazebo-classic`、`Tools/simulation/gz`。
**建议新目录 `~/refs/px4-z0-build/`，与本表的 fetch-only 副本并存**（后者继续承担"读源码"职责，不需要 submodule）。

## 2.7 ★ fork 分支拓扑 + **ADC bug 的一行修复就藏在 fork 里**（2026-09-20 新增）

> ⚠️⚠️ **2026-09-21：本节讲的全是 fork —— 而 z0 的飞控不跑 fork（见 §2.5(d)）。**
> 飞控跑的是**基于 v1.14.3 的 MicoAir 构建**，其树里**没有 `boards/hkust/`**，
> 所以本节 (b) 那两个「修复途径」**对 z0 的固件都不适用** —— 要修也得针对
> `boards/micoair/h743-*`（而它的电流通道 `/* PC1 */ ADC1_CH(11)` **本身就是对的**）。
> 本节作为**fork 的拓扑记录**仍然有效，但**不要拿它当 z0 固件的修复方案**。

### (a) 分支拓扑（`git ls-remote --heads` 第一手）

fork 只有**四条**分支（**没有 `.gitmodules` 里写的 `develop`** —— 那条是陈旧的）：

| 分支 | commit | 日期 | `boards/hkust/` | `OffboardControlMode` | ADC 电流通道 |
|---|---|---|---|---|---|
| `feat_deprecated_v1.14.x_high_imu` | `c0ccb2f4` | 2024-03-20 | `nxt-dual` + `nxt-v1` | `thrust_and_torque`+`direct_actuator` | `CH(5)` |
| **`feat_fast_lab_stable`** ← **Nxt-FC 钉的就是它** | **`5ba2e91f`** | 2024-07-02 | `nxt-dual` + `nxt-v1` | 同上 | `CH(5)` |
| `pr_fix_hkust_nxt_dual` | `8161c4cd` | 2024-11-19 | `nxt-dual` + `nxt-v1` | 同上 | ✅ **`CH(8)`** |
| `main` | `5b66a1e` | 2024-11-19 | `nxt-dual` + `nxt-v1` | 同上 | `CH(5)` |

> ⚠️ **`feat_deprecated_v1.14.x_high_imu` 这个分支名是误导性的** —— 名字里的 "v1.14.x" 
> 不描述它的消息定义（它是 v1.15 世代）。**不要按分支名做版本推断。**
>
> ⚠️ **`5ba2e91f` 不是 `main` 的祖先**（`merge-base --is-ancestor` 为假）—— `feat_fast_lab_stable`
> 与 `main` **已分叉**。但两者 `boards/hkust/nxt-dual/src/board_config.h` 的 blob 是**同一个**
> （`2b8aba12`），所以对本文档的结论无影响。

### (b) ★★ ADC bug 的修复 = **一行 diff**，且**就在 fork 自己身上**

`pr_fix_hkust_nxt_dual` 相对 `main` 的**全部**差异：

```
 boards/hkust/nxt-dual/src/board_config.h | 2 +-
 1 file changed, 1 insertion(+), 1 deletion(-)
```

其 commit message 就是 `hkust_nxt-dual: fix ADC_BATTERY_CURRENT channel mismatch`。

| 修复途径 | 代价 | 出处 |
|---|---|---|
| ★ **apply `pr_fix_hkust_nxt_dual` 的那一行** | **1 行改动**，vendor 自己写的 | fork `8161c4cd`（2024-11-19） |
| 跳到上游 v1.16.0 | 整个版本跳变 | 上游 v1.16.0 已改 `CH(8)`（第一手核实） |

> 📌 注意 `pr_fix_` 前缀 + 它**没有**被合进 `main`（`main` 仍是 `CH(5)`）→ 这是一个**未合并的
> 提案分支**。**用它 = 我们自己做一次 fork 上的补丁**，不是"上游已修好，跟着走"。
> **→ 这正是 Q12「不刷固件」与 A4「必须重编」正面相撞的地方 —— 决策见第 8 轮问答（Q37）。**

## 3. 使用纪律

1. **只读。** 这些是参考物，不是本项目代码。**不要把任何文件复制进本仓库**（Q10）。
2. **不要 `git pull`。** 钉住的 commit 是可复现性的基础；上游漂移会让本文档的结论失效。
   需要看新版时，**另建目录**并在此表登记。
3. **引用时带 commit。** 任何「XI35 是这么做的」这类结论，都要能指回本表的某一行。
4. **PX4 上游三个 tag 是 fetch-only 的**（`--no-checkout`），这是刻意的：只为读少数几个文件，
   不需要检出整套 PX4。用 `git ls-tree` / `git show` 访问。
