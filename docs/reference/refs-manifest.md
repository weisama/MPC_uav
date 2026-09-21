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

> ⛔ **2026-09-20 订正一处推理（结论不变，理由换掉）**：本文档曾由「`nxt-dual` 只在 PX4 ≥ 1.15.0 存在」
> **推出**「所以 z0 固件是 v1.15 世代」。**这个推理是错的** —— 上游和 fork 是两回事：
> fork 的 `feat_deprecated_v1.14.x_high_imu` 分支（2024-03-20）**就带 `nxt-dual`**。
> 「板级目标存在」**不能**推出世代。
> ✅ **真正扛住结论的是直接证据**：fork **四条分支**的 `msg/OffboardControlMode.msg`
> **全部**含 `thrust_and_torque` + `direct_actuator`，且**全部**有 `srv/VehicleCommand.srv`
> （见 §2.7）。**从"看消息定义"取代"看板子存在与否"。**

> ⚠️ **仍未完全等于「飞控里烧的那一份」**：`z0.params` 头声明 `Git Revision: 4a0e65f233000000`，
> **与 `5ba2e91f` 不一致**。这可能是不同的构建、或 param 头写的是上游基线。
> **→ 首飞前必须在 QGC 里记录飞控实际报出的版本号，以它为准。**

## 2.5 ★★ z0 固件是「v1.15 世代」，不是 1.14 —— 且 `px4_msgs` 必须换（2026-09-20）

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

## 2.6 ★★ 已定决策（用户 Q31=a，2026-09-20）：**版本基准统一到 fork `5ba2e91f`**

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
