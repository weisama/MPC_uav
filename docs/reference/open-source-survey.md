# 开源项目包级取舍调查

> 用户指定要调研的 5 个项目 + 调查中发现的相关项目，以及在**本项目**里各自怎么用。
> 相关：[`../PROJECT.md`](../PROJECT.md) 第 5.2 节

---

## 0. ✅ 调查方法与置信度（已升级：全部第一手）

> **2026-09-18 更新**：本文档早先版本的方法论一栏写着"多数项目只靠搜索摘要、未经核对"。**那已经过时——所有项目都已 `git clone` 到本地并逐目录核对。** 下面第 1–9 节的每一条结论现在都是第一手证据。

| 手段 | 状态 | 覆盖 |
|---|---|---|
| `git clone` 到 `/tmp/repo-survey/` | ✅ **可用** | 下表全部 |
| SSH 直取私有仓库 | ✅ 可用 | `weisama/MPC_uav`、`Huan-qun/Sim2real-UAV-Params-` |
| `WebFetch` 打开网页 | ❌ **被网络策略拦截**（`github.com`、`uzh-rpg.github.io`、`oddityrc.com`） | 只能用 clone 替代 |
| `WebSearch` 摘要 | ⚠️ 二手，**本文档已不再依赖** | — |

**本地 clone 清单**（`/tmp/repo-survey/`）：

| 目录 | 非 `.git` 文件数 | 结论 |
|---|---|---|
| `agilicious/` | **38** | ❌ **零代码**，见 §2 |
| `Fast-Drone-XI35/` | **1501** | ✅ 真实代码，见 §3 |
| `Fast-Drone-250/` | 1132 | XI35 的上游，见 §3.4 |
| `OmniNxt/` | **4** | ❌ **零代码**，见 §4 |
| `px4-mpc/` | 40 | ✅ (C) 方案蓝本，见 §5 |
| `nxtfc/` | 49 | ★ 见 §9 |
| `d2slam/`、`omninxt-ghpages/`、`px4_ugv_exp/` | — | 旁证 |

> ⚠️ **两个真实踩过的坑，值得记住**：
> 1. 一个代理用 HTTPS/GitHub API 探测 private 仓库，得到 **404 就断言"仓库不存在"**。GitHub 对未授权 private 仓库**故意返回 404**。有 SSH key 时必须用 `git ls-remote`。**教训见 [`../LOCAL-NOTES.md`](../LOCAL-NOTES.md) 第 3 节。**
> 2. 一个代理在只有 README/LICENSE/PDF 二进制的情况下，声称"已检索论文全文"并给出构型结论。**该结论事后被本地 PDF 的 `pdftotext` 推翻**（见 §4.1）。**教训：结论必须能指回一个具体的、可复现的证据来源。**

---

## 1. `acados/acados` —— ✅ **核心依赖**

| 项 | 内容 |
|---|---|
| 定位 | 基于 CasADi 的**嵌入式最优控制求解器**，**自动生成 C 代码** |
| 本项目角色 | **MPC 求解器由它生成**。这是唯一一个"必用"的项目 |
| 获取方式 | 源码编译安装（aarch64） |

**组成部分**（本项目会用到的）：

| 组件 | 作用 |
|---|---|
| `acados_template` | Python 接口，用于定义 OCP 并触发生成 |
| **HPIPM** | 默认 QP 求解器（内点法），利用 OCP 带状结构 |
| **BLASFEO** | 高性能线性代数库，**按 CPU 微架构编译** |
| **Tera / `t_renderer`** | 代码生成的模板引擎（Rust 二进制） |

**★ Jetson (aarch64) 上的已知要点**：

| 问题 | 处理 |
|---|---|
| **无 `nvcc` / TensorRT**（已实测） | acados 的 CPU 路线不需要；但**不要走 CUDA 后端** |
| Orin NX 是 **Cortex-A78AE**，**acados/BLASFEO 没有 A78 目标** | `BLASFEO_TARGET = ARMV8A_ARM_CORTEX_A76` |
| HPIPM 目标 | `HPIPM_TARGET = GENERIC` |
| **缺 Tera 渲染器** | ☠️ 旧话「报 **`EOFError`**」**在 v0.6.0 上不成立**（v0.5.x 及以前交互式 `input()` 的内建异常）。★ **但「必须装」这件事不变**：**真身是三种落点，其中「在、但渲染不出东西」完全静默**，静默两层后才炸成 `.so: cannot open shared object file`（`utils.py:396-397` 用 `os.system` 只查退出码、不校验产物）。详见 [`../control/acados-orin-build.md`](../control/acados-orin-build.md) §4 |
| 运行时路径 | 必须设置 `ACADOS_SOURCE_DIR` 与 `LD_LIBRARY_PATH` |

**求解器配置（参考 `px4-mpc` 的做法）**：

```
N  = 50          预测步数
Tf = 1.0 s       预测时域
cost:  NONLINEAR_LS
qp:    PARTIAL_CONDENSING_HPIPM
hess:  GAUSS_NEWTON
int:   ERK, 4 stages, 3 steps
nlp:   SQP_RTI   ← 实时迭代，每周期一次 SQP，延迟最低
```

---

## 2. `uzh-rpg/agilicious` —— ⚠️ **只借架构思想，拿不到代码**

> ★ **本节已改为第一手数据**（`git clone` 实测，HEAD `50bba45`，最后提交 **2023-03-07** —— 仓库已三年未更新）。

| 项 | 内容 |
|---|---|
| 定位 | ETH/UZH 的敏捷飞行框架（Science Robotics 2022） |
| **仓库实际内容** | ★ **38 个文件，零代码**：`docs/*.rst`（24 个 Sphinx 文档）+ `README.md` + `launch_container.sh` + 8 个 `.bib` + 3 张图 |
| 代码文件计数 | `.cpp` **0** / `.hpp` **0** / `.h` **0** / `.yaml` **0** / `.launch` **0** / `.params` **0** / `.py` **1**（`docs/conf.py`，Sphinx 配置） |
| **源码可得性** | ❌ **公开仓库没有源码，且不是"忘了传"——是许可证锁的。** 代码需向 UZH 申请 |

**★ 许可证是硬约束，必须知道**（`docs/license.rst`）：

- 只有 **UZH 明确授权的研究者**才能使用，申请入口是 JotForm `230332943688361`
- 授权范围 **仅限学术非商业**
- ⚠️ **§2.3 明文禁止**：不得再许可、分发、转让、披露给 **"另一研究组"或任何第三方** —— 需 PI 书面许可
- §6.3：协议终止时须卸载并销毁所有副本

→ **结论：即使申请到了，也不能把它的代码放进 `MPC_uav`**（除非拿到 PI 书面许可）。**本项目对 agilicious 只能"读文档、引论文"，一行代码都不搬。**

**★ 关键词命中数（说明它不是本项目能用的代码源）**：

| 关键词 | 命中文件数 |
|---|---|
| `PX4` / `px4` | **0** |
| `mavros` / `MAVROS` | **0** |
| `uXRCE` | **0** |
| `ROS2` / `ros2` | **0** |
| `Offboard`（大写，PX4 模式） | **0** |
| `offboard`（小写） | **1** ⚠️ **假阳性**——见下 |

> ⚠️ **那唯一的 `offboard` 命中是个陷阱**：`docs/modules/bridge.rst:9` 说的是 `laird` 桥接"**从 offboard controller 取指令**再经 SBUS 发给底层控制器"——这里 offboard 是"机外控制器"的意思，**和 PX4 的 Offboard 模式毫无关系**。关键词搜索容易在这里骗人。

**★★ 对本项目真正有价值的三条**：

1. **`guard` 模块 = 本项目"安全网"的现成范本。** 原文（`docs/modules/guard.rst`）：
   > *"运行一条**完全独立的 pipeline**（独立的 sampler / estimator / controller），与主 pipeline **并行**。当主控制器崩溃、主估计发散、或无人机飞出预定 3D 空间时，guard **自动接管**并把飞机带回安全。用户观察到异常时也可**手动触发**。"*

   → 这正好是 [`px4-interface.md`](px4-interface.md) 第 1 节那张图里"安全网链路 / 控制链路分开"的**同构设计**。**z0 的安全网应该照这个思路做：一条不依赖 MPC 的独立链路 + 一个 3D 边界条件。**

2. **★ 它的 MPC 求解器栈和本项目完全一致。** `docs/modules/controller.rst` 原文：
   > *":code:`MPC`: … It uses **`acados`** with **`HPIPM`** solver to build an optimization problem…"*

   → 这是本项目 §1 里 acados + HPIPM 选型的**独立佐证**——不是我拍脑袋选的。

3. **`INDI` 在其架构里明确是内环。** `controller.rst` 原文：
   > *":code:`INDI`: … It's primarily used for **inner loop control (tracking body rates)**."*

   → 与本项目演进路线 `(C) → (C)+INDI → (D)` **字面吻合**。**内环用 INDI 是这条线上被验证过的做法。**

4. **它的 controllers 列表同时有 MPC / 几何控制 / PID / INDI**，且 `Pipelines` 支持**运行时切换**（*"create multiple pipelines and switch between them at runtime"*）。
   → 这给本项目的 `(A) → (C)` 分阶段切换提供了架构上的正当性。

**Bridge 层（说明它不走 PX4）**：`sbus` / `ctrl`（自家 `AgiNuttx` 底层控制器）/ `laird`（LAIRD RF 模块）/ `RotorS`（仿真）/ BetaFlight。
→ **完全没有 PX4 / mavros 这条路。** 本项目不能照抄它的接口层，只能借分层。

**结论**：`agilicious` 对本项目是**理论来源 + 架构范本**，**不是代码来源**，且**受许可证限制连代码都不能引用**。

---

## 3. `Longer95479/Fast-Drone-XI35` —— ✅ **本项目最有价值的参考（同硬件！）**

> ★ **本节已改为第一手数据**（`git clone` 实测，HEAD `8c80ddb`，2026-02-02，默认分支 **`board`**，只有一个分支）。

| 项 | 内容 |
|---|---|
| 定位 | **哈工大 HIT-K1928-LAB** 的同机架自主飞行栈 |
| 硬件 | **与本项目高度重叠**：OddityRC XI35 + NxtPX4v2/Nxt-FC + Jetson Orin NX + 6S |
| 规模 | **1501 个文件**（`.cpp` 237 / `.h` 202 / `.hpp` 145 / `.py` 62 / `.launch` 72 / `.msg` 43） |
| 控制器 | **PID，不是 MPC** |
| ROS 版本 | **ROS1**（`rospy` / `catkin` / `mavros_msgs`） |

**★ 为什么它最有价值**：**同一台机架、同一个飞控、同一个机载电脑**上已经有人跑通了完整自主飞行栈。**它的 `(B)` 级接口和状态机是"在 z0 这块硬件上被验证过"的。**

### 3.1 目录结构（第一手）

```
src/
├── realflight_modules/   ← ★ 与飞控/传感器对接的全部内容
│   ├── px4ctrl/          ← ★★ 飞控接口 + 状态机（本项目主要参考对象）
│   ├── motion_capture/   ← ★ 动捕 → 里程计（含时延对齐）
│   ├── kf_fusion/        ← 动捕+VINS 融合
│   ├── VINS-Fusion-gpu/  ← 视觉里程计
│   ├── realsense-ros/ 、jetsoncsi-ros/ 、det-reg-pnp/   ← 感知，本项目不需要
├── planner/              ← ego-planner 系（bspline_opt / plan_env / plan_manage / …）
├── uav_simulator/        ← 仿真
└── utils/                ← quadrotor_msgs / uav_utils / pose_utils / imu_filter / …
```

### 3.2 ★★ `px4ctrl` —— 真正要抄的地方

**规模**：1932 行（`PX4CtrlFSM.cpp` 685 / `controller.cpp` 300 / `input.cpp` 287 / `px4ctrl_node.cpp` 134 / 其余为头与参数）。

**★ 接口事实（第一手 grep 确认）**：

| 项 | 实际用的东西 | 对 z0 的含义 |
|---|---|---|
| 指令话题 | `/mavros/setpoint_raw/attitude`（**`AttitudeTarget`**） | ⚠️ 这是 **(B) 级**，不是本项目 Phase 1 的 (A) 级 |
| 模式切换 | `/mavros/set_mode`（`custom_mode = "OFFBOARD"`） | ✅ 通用，Phase 1 照用 |
| 解锁 | `/mavros/cmd/arming` | ✅ 通用 |
| 重启 | `/mavros/cmd/command`，`command=246`（`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN`） | 🔶 可选 |
| 里程计源 | `use_motion_capture_odom` 开关：`true` → `/motion_capture/motion_capture_odom`；`false` → `odom` | ✅ 思路可借 |

**★ 状态机（`PX4CtrlFSM.h:63`）—— 本项目直接借这个骨架**：

```cpp
enum State_t {
    MANUAL_CTRL = 1,  // px4ctrl 未接管，飞控只听遥控器
    AUTO_HOVER,       // 已接管，按里程计保持悬停
    CMD_CTRL,         // 已接管，正在跟踪指令
    AUTO_TAKEOFF,
    AUTO_LAND
};
```

> 📌 **这是本项目的关键收获**：状态机把"**谁在控制飞机**"（遥控器 / 自动）和"**自动在干什么**"（悬停 / 跟踪 / 起飞 / 降落）**正交地分开了**。本项目 Phase 1 的状态机应照此设计，并把 `MANUAL_CTRL → AUTO_TAKEOFF → AUTO_HOVER → CMD_CTRL → AUTO_LAND` 的**转移条件**照抄过来（它们包含超时、电压、RC 打杆接管等实飞才会想到的判据）。

**★ 控制器的取舍**：`controller.cpp` 是 **PID + 推力模型反解**（(B) 级姿态输出）。
→ ❌ **不要抄它的控制器**（本项目要做 MPC）；✅ **但要抄它"从期望加速度反解推力 + 电压补偿"的那段**——`thrust_model` 的公式正是 MPC 也需要的东西。

### 3.3 ⚠️ `config/ctrl_param_fpv.yaml` 的两个真实的坑

**坑 1（★ 会直接导致炸机）**：这个文件名叫 `_fpv`，但第 13 行是

```yaml
use_motion_capture_odom: true
```

→ **它是 XI35 的动捕配置，不是 FPV 配置。** 而第 5 行 `mass: 0.645`（XI35 实重）**不是 z0 的 750 g**，第 12 行 `low_voltage: 18.0` 是 6S 值。
→ **绝不能整份拷到 z0。** 质量差 16% 会直接体现在悬停推力和 PID 增益上。

**坑 2**：第 15–19 行与第 20–24 行的 **`rc_reverse:` 重复定义了两次**（内容完全相同）。

```yaml
rc_reverse:        # ← 15-19 行
    roll: true
    ...
rc_reverse:        # ← 20-24 行，完全重复
    roll: true
    ...
```

→ YAML 解析器**静默取后者**，功能上无害，但说明**这个文件被反复覆盖编辑过，不要假设它内部自洽**。

### 3.4 ★ `thrust_calibrate.py` —— 只是**数据记录器**，不是求解器

**⚠️ 更正早先的说法**：本文档以前写它"可直接拿来用于推力曲线标定"。**不准确。**

**它实际做的事**（120 行，逐行读过）：

1. 订阅 `/mavros/battery` 和 `/mavros/setpoint_raw/attitude`，**按 `time_interval`（默认 1 s）分段取均值**
2. 用 `/traj_start_trigger` 按下/再按**开始与停止**记录
3. 电压低于 `vbat_min` 时自动停止
4. **把 `(thrust, voltage)` 序列追加写入 `px4ctrl/thrust_calibrate_scrips/data.csv`**

**它不做的事**：**不拟合 K1 / K2 / K3**。拟合要你自己拿 CSV 离线做。

**移植成本（比想像中高）**：

| 问题 | 说明 |
|---|---|
| `#!/usr/bin/env python2` | **Python 2**，ROS2 下必须重写 |
| `import rospy` / `rospkg` | ROS1，需改 `rclpy` |
| 订阅 `/mavros/setpoint_raw/attitude` | ⚠️ **本项目 Phase 1 走 (A) 级 `TrajectorySetpoint`，根本不发 attitude target** —— 这个话题要换 |

**★ 但它藏着两条本项目直接要用的知识**：

```
# 推力模型（config/ctrl_param_fpv.yaml:33）
F = K1 * Voltage^K2 * (K3*u^2 + (1-K3)*u)
```

- **K3 就是 PX4 的 `THR_MDL_FAC`** —— 文件里自己写了注释：*"K3 equals THR_MDL_FAC"*。z0 的 `THR_MDL_FAC=0.45`，**可以直接代入作为 K3 的先验**。
- **`F = K1*V^K2*(...)` 这个形式本身就是结论**：推力**随电池电压变化**（`V^K2`，K2≈1.69）。**这就是"电池是瓶颈"的定量表达**，也是 MPC 里 `T_max` 必须按电压缩放的依据。

→ **结论**：`thrust_calibrate.py` 是 **🔶 参考（借公式与流程，代码重写）**，不是 ✅ 直接可用。

### 3.5 ★ `motion_capture` 包 —— 动捕时延对齐的现成做法

**规模**：236 行（`Process.cpp` 134 / `motion_capture_node.cpp` 37 / 其余为参数）。

**它解决的问题**（`ReadMe.md` 原文）：

> *"将动捕输出的 3 个 Topic 整合为一个动捕里程计话题进行集中输出，本 ROS 包通过**固定 Pose 的数据和时间**，将 **Twist 和 Accel 数据进行线性插值**计算出和 Pose 时间戳对齐的数据，最后打包整合发布在 `/motion_capture/motion_capture_odom`"*

**★ 对本项目的价值**：

| 点 | 说明 |
|---|---|
| **插值对齐时间戳** | 这是一个**本项目尚未考虑的技巧**。`mocap-setup.md` 的方案是"直接转发 `vrpn_listener` 的位姿"，没有做 pose/twist/accel 的**互相对齐**。若将来要用动捕的速度/加速度（MPC 里很有用），**这段插值逻辑是现成的蓝本** |
| **发布成 `nav_msgs::Odometry`** | 一个话题同时给位置+姿态+速度+角速度，**比多个 topic 更好用** |
| ⚠️ 依赖 `/vrpn_client_node/...` | 它用的是 `vrpn_client_ros` 的命名，**不是**本项目选定的自写 `vrpn_listener`。**对接时要改话题名** |

→ **结论**：**🔶 重点参考（借算法，重写代码）**。

### 3.6 包级取舍总表

| 包 / 文件 | 取舍 | 理由 |
|---|---|---|
| **`px4ctrl/PX4CtrlFSM.*`** | ✅ **重点参考** | ★ **状态机骨架**，实飞验证过，本项目直接借 |
| **`px4ctrl/controller.cpp` 的推力模型段** | ✅ **重点参考** | 期望加速度 → 推力 + 电压补偿 |
| **`px4ctrl/controller.cpp` 的 PID 段** | ❌ **不用** | 本项目要做的是 MPC |
| **`px4ctrl/config/ctrl_param_fpv.yaml`** | ❌ **不要拷贝** | ⚠️ 质量 0.645 是 XI35 的，不是 z0 的 750 g；且 `rc_reverse` 重复定义 |
| **`px4ctrl/thrust_calibrate_scrips/`** | 🔶 **参考公式与流程** | ⚠️ Python2 + ROS1 + 话题是 (B) 级的，必须重写 |
| **`motion_capture`** | 🔶 **重点参考** | ★ pose/twist/accel **插值对齐**；话题名要换成 `vrpn_listener` 的 |
| `kf_fusion` | 🔶 第二阶段再看 | 动捕 + VINS 融合；Phase 1 只用动捕 |
| `planner/`（ego-planner 系） | 🔶 **第二阶段再说** | 第一阶段不做避障规划，**先能稳定跟踪给定轨迹** |
| `VINS-Fusion-gpu` / `realsense-ros` / `det-reg-pnp` / `jetsoncsi-ros` | ❌ **不用** | 纯视觉/识别，本项目 Phase 1 不涉及 |
| `uav_simulator/` | ❌ 不用 | 本项目用 PX4 SITL |
| `catkin_*` / ROS1 构建体系 | ❌ 不直接搬 | 目标是 ROS2 Humble，**移植而非搬运** |

> ⚠️ **再强调一次**：该项目的控制器是 **PID**，**不能把它的控制部分当 MPC 参考**。用户说的"参考其控制以外的功能包"正是这个意思。

### 3.7 上游 `Fast-Drone-250`

`ZJU-FAST-Lab/Fast-Drone-250`（1132 文件）是 XI35 的上游骨架 —— **XI35 的 `px4ctrl` / `motion_capture` 均源自它**。两者结论一致，**看 XI35 即可**（XI35 更新、且硬件与本项目相同）。

---

## 4. `HKUST-Aerial-Robotics/OmniNxt` —— ❌ **零可复用代码；★ 构型已更正**

> ⚠️ **本节更正了本文档早先版本的两处错误**：(1) 说它是"全向倾转旋翼平台"；(2) 说"唯一可取的产出是 `OmniNxt-03.params`"。**两条都不成立。**

| 项 | 内容（第一手 `git clone` 实测） |
|---|---|
| 定位 | **全向视觉感知**的常规四旋翼（IROS 2024）—— ★ **不是倾转旋翼** |
| 仓库实际内容 | ★ **只有 4 个文件**：`README.md`(11.7 KB) + `LICENSE` + `misc/OmniNxt.pdf`(15 MB) + `misc/system_overview.png`。**`.cpp`/`.h`/`.py`/`.yaml`/`.launch`/`.params` 命中数全部为 0** |
| 源码可得性 | ❌ **仓库只是个论文 landing page**。README 声称提供控制/规划的**两个外部仓库已 404**（`D2SLAM-Fusion/controller-PX4Control`、`planner-EgoPlanner`） |
| 共用硬件 | ✅ **Nxt-FC 飞控**（PX4 目标板名 `hkust_nxt-dual`）—— 与 z0 的 NxtPX4v2 同系列 |

### 4.1 ★ 构型更正：是"全向视觉"，不是"全向倾转"

论文原文标题（本地 PDF `assets/papers/Liu2024_IROS_OmniNxt.pdf` 提取）：

> **OmniNxt: A Fully Open-source and Compact Aerial Robot with Omnidirectional Visual Perception**

全文关键词计数（36 KB 文本）：`omnidirectional` **39 次**、`tilt` **0 次**、`tilting` **0 次**、`servo` **0 次**、`fully-actuated` **0 次**。

**"Omnidirectional" 指的是多鱼眼相机的全向视野（感知），不是全向倾转动力。** 构型是**常规四旋翼**。

**连带失效的结论**：早先版本写的"❌ 不要参考它的控制分配——它的构型有倾转舵机（`CA_SV*`）"——**这个理由不成立**（它没有倾转舵机）。不过结论（不参考其控制分配）依然成立，理由变成：**它压根没有可参考的代码**。

### 4.2 ❌ `OmniNxt-03.params` 实际取不到

**该文件不在仓库里。** `README.md:102` 指向一个 **OneDrive/SharePoint 链接**，实测返回的是 275 KB 的 **HTML 登录页**，需浏览器登录才能下载。

→ **早先版本"唯一可取的产出"这条也不成立。** z0 的参数比对基准改由 §9 的 `Nxt-FC` 提供（那是**真正的板级默认参数**，比一份同平台参数文件更有权威性）。

### 4.3 ✅ 真正的价值：它的硬件线

| 仓库 | 状态 | 价值 |
|---|---|---|
| **`HKUST-Aerial-Robotics/Nxt-FC`** | ✅ 200 | z0 那块飞控的官方开源仓库（含硬件 KiCad 设计） |
| **`Peize-Liu/PX4-Autopilot`** | ✅ 200 | Nxt-FC 配套的 PX4 fork。☠️ **2026-09-21：它与 z0 的固件无关**（真身是 `Minderring/PX4-Autopilot @ 4a0e65f2`）—— 见 §9 的 ☠️ 与 [`refs-manifest.md`](refs-manifest.md) §2.5(d) |
| `HKUST-Aerial-Robotics/D2SLAM` | ✅ 200 | VIO + 深度（本项目第一阶段用不上） |

**详见 §9。**

### 4.4 ⚠️ 明确的"不要"

- ❌ **不要指望能跑它的代码** —— 仓库里没有代码，外部依赖已 404
- ❌ **不要参考它的规划/感知** —— 它是全向视觉，本项目首要定位源是动捕
- ❌ **不要把 Nxt-FC 仓库里的预编译固件刷到 z0** —— 那是给 `nxt-v1`（单 IMU + BMP388）的，**不是给 NxtPX4v2（双 BMI088 + SPL06）的**，见 §9.4

---

## 5. `Huan-qun/Sim2real-UAV-Params-` —— ✅ **已采用其飞控参数**

| 项 | 内容 |
|---|---|
| 定位 | **同配件的动捕实飞资料库** |
| 获取 | ✅ SSH 直取成功 |

**取用的产出**：

| 上游文件 | 状态 | 整理到哪 |
|---|---|---|
| **`S2R/assets/param/uav1.params`** | ✅ **已采用** | 原文件 → [`../assets/z0.params`](../assets/z0.params)；逐项分析 → [`params-analysis.md`](params-analysis.md) |
| **`S2R/assets/doc/mocap.md`**（594 行） | ✅ **已整理** | ★ **动捕链路搭建手册** → [`mocap-setup.md`](mocap-setup.md)（依赖安装、坐标系、刚体创建、mavros、速度路由、`vrpn_relative_pose` 源码、开机自启、z0 适配清单） |
| **`BOM.md`** | ✅ **已整理** | 完整物料清单 → [`hardware.md`](hardware.md) 第 11 节 |
| `S2R/README.md` | ✅ **已整理** | 物理参数表 → 本节下方；Genesis/rsl_rl → 见下 |

### ★ 从 `BOM.md` 里读出的三条新信息

1. **遥控器包含接收机**：条目原文是"【ELRS版本 带蓝牙】黑色+电池+ **XF接收机** 左手油" → **RadioMaster XF 接收机是随遥控器一起的**，不是另配。（这解决了 [`hardware.md`](hardware.md) 第 7 节里"接收机需另配"的不确定性。）

2. **电池厂商标的是 30 A**：条目原文"伪神电池18650 VTC6 3000mAh 3S 4S 6S **30A** XT30 60 FPV远航电池6S XT60"。
   → 与 [`hardware.md`](hardware.md) 第 6 节一致：**厂商标称 30 A，而 Mooch 独立实测持续放电只有 15 A。** ⚠️ **本项目按 15 A 做保守设计。**

3. **★ 动捕球是用铝柱架起来的**：条目"**铝柱(装动捕球)**"。
   → **直接关系到 `EKF2_EV_POS_X/Y/Z`**：架高的刚体原点与飞控 IMU 之间**几乎必然存在 Z 向偏移**（铝柱高度）。所以 [`params-analysis.md`](params-analysis.md) 3.6 里"`EKF2_EV_POS_Z = 0` 可能是漏标"这个怀疑**得到了佐证**。

> ⚠️ **同时注意**：upstream BOM 里还有 **D435i 相机**、碳板机臂、保护圈等本用户清单里没有的东西。**说明上游那台机器与本项目并非完全相同**（多了一套深度相机）。这也提醒：**`z0.params` 是"近似同构"而非"完全相同"的参考。**

### ★ 从 `S2R/README.md` 里读出的第四条信息：上游还做 **RL sim-to-real**

上游 README 除了机架和参数表，还有一节 **"genesis 环境配置"**：

| 项 | 内容 |
|---|---|
| 仿真器 | **Genesis**（`Huan-qun/Genesis`，一个 fast physics simulation 平台） |
| Python | **3.11** |
| 强化学习库 | **`rsl_rl` v1.0.2**（leggedrobotics 的 RL 库） |

**含义**：仓库名里的 **"Sim2real"** 是**字面意思** —— 上游不只做动捕实飞，还在做**仿真训练 → 真机部署**的强化学习路线。`S2R` = Sim2Real。

**对本项目的启示**：

- ✅ **说明 MPC 不是这台机器的唯一路线**，上游/同平台还有 RL 路线在跑。**本项目选择 MPC 是有意识地选了另一条路**，不是因为 RL 不可行。
- 🔶 **可复用**：如果后续要做"MPC 与 RL 对比"或 sim-to-real 实验，这套 **Genesis + rsl_rl** 环境是现成的起点。
- ⚠️ **但要注意**：z0 的机载电脑 **没有 `nvcc`、没有 TensorRT**（见 [`hardware.md`](hardware.md) 第 8 节），**GPU 训练在当前环境跑不了** —— 训练得在别的机器上做，Jetson 只负责推理。

### 关于 `S2R/assets/param/` 里的物理参数表

上游 README 里有一张物理参数表，**但它的标题就是"单机物理参数**(**待更新**)"** —— 上游作者自己标注了未更新。表中相关数值：

| 项 | 上游值 | 对比本项目 |
|---|---|---|
| 质量 | 0.3 kg | ⚠️ 与本项目的 **750 g** 差 2.5 倍（那是另一台更轻的机器） |
| **臂长** | **0.08 m** | ✅ **与本项目的 76 mm 接近** —— 反过来佐证 `CA_ROTOR = 0.15` 是默认值 |
| Ixx / Iyy / Izz | 5.6e-4 / 5.6e-4 / 8.6e-4 kg·m² | 🔶 可作**量级参考**，不可直接用 |
| KF | 3.16e-10 | 🔶 同上 |
| KM | 7.94e-12 | 🔶 同上 |
| 推重比 | 2.90 | ⚠️ 与 [`hardware.md`](hardware.md) 第 6.2 节的电池受限 T/W（1.60–2.54）**量级吻合** |
| 桨半径 | 0.0231 m | ⚠️ 45 mm 半径 → 90 mm 直径，与本项目的 DT90MMX3 一致 ✅ |

> 📌 **"臂长 0.08 m"这一条很重要**：上游自己写的臂长是 80 mm（≈ 本项目的 76 mm），而参数文件里却是 150 mm。**两个数字在同一份资料里打架**，进一步证明 `CA_ROTOR = 0.15` **继承的是 PX4 默认值，不是实测值**。

---

## 6. `weisama/MPC_uav` —— 📚 **本项目仓库（目前只有资料）**

| 项 | 内容 |
|---|---|
| 定位 | **本项目自己的 GitHub 仓库** |
| 当前内容 | README + `papers/` 下 **6 篇 PDF**，**无代码** |
| 获取 | ✅ SSH 直取成功 |

**6 篇论文与本项目的关系**（**只需理解关系，不必细读公式** —— 用户明确要求）：

| 文件 | 与本项目的关系 |
|---|---|
| **`Foehn2022_SciRob_agilicious.pdf`** | ★ **`agilicious` 的论文**。本项目 `(C)` 方案（CTBR）与 INDI 内环的**理论来源** |
| **`Sun2022_TRO_NMPC_vs_DFBC.pdf`** | ★ **NMPC vs 微分平坦控制的实证对比**。结论：*"两者加内环 INDI + 气动阻力模型后跟踪误差降低 78 % 以上"* —— **直接支撑本项目的 INDI 演进路线** |
| **`Torrente2021_RAL_DataDrivenMPC.pdf`** | **数据驱动 MPC**：用高斯过程建模气动效应嵌入 MPC，高速跟踪误差最多降 70 %，实测至 14 m/s / 4 g。→ **本项目第二阶段（气动补偿）的参考** |
| **`Liu2024_IROS_OmniNxt.pdf`** | 同平台（**Nxt-FC + Jetson Orin NX，与本项目完全同款**）的全向平台论文。**硬件参考价值 > 算法价值** |
| **`Houska2011_OCAM_ACADOToolkit.pdf`** | ACADO toolkit 综述。**acados 的前身**，帮助理解 acados 的设计取舍 |
| **`Houska2011_Automatica_RTI.pdf`** | ACADO 的**实时迭代（RTI）**算法原始论文。**`SQP_RTI` 的出处** —— 解释了为什么 acados 能做到微秒级 |

> 📌 **这 6 篇的分工**：**控制方法** 3 篇（`Foehn2022` / `Sun2022` / `Torrente2021`）、**工具链** 2 篇（`Houska2011` × 2）、**硬件平台** 1 篇（`Liu2024`）。**本项目应沿着这条线索走，而不是另起炉灶。**
>
> 📖 **每篇的详细中文标注**见 [`../assets/papers/README.md`](../assets/papers/README.md)。

---

## 7. 调查中发现的**额外**参考项目

用户没有指定，但调研中发现对本项目有直接价值的：

| 项目 | 价值 | 优先级 |
|---|---|---|
| **`DISCOWER/px4-mpc`** | ★★ **`(C)` 方案的现成蓝本**：用 acados 生成 MPC，**发布 `/fmu/in/vehicle_rates_setpoint`** —— 与本项目方案完全一致。约 210 ★ | ★★★ **最高** |
| `tkkim-robot/px4_ugv_exp` | PX4 + ROS2 Humble + Vicon 的闭环最小可运行模板 | ★★ |
| `SaxionMechatronics/mocap_px4_bridge` | OptiTrack → PX4（ROS2 版） | ★ 备用参考 |

> 📌 **`px4-mpc` 应该被 clone 下来当作起点。** 它同时解决了两个问题：**acados 怎么用** + **MPC 怎么接 PX4**。本项目的增量工作是**把控制器换成自己的 + 接上动捕**，而不是从零搭建。

---

## 8. ⚠️ 本节已废弃，汇总见 §10

> 早先版本这里的汇总表把 `thrust_calibrate.py` 写成"拿来即用"、把 `OmniNxt-03.params` 写成"可用的第二份对照"——**两条都已在 §3.4 与 §4.2 被第一手证据推翻**。**请直接看 [§10 汇总](#10-汇总本项目实际要做什么2026-09-18-重写)。**

---

## 9. ★ Nxt-FC / PX4 fork —— z0 飞控的官方固件线（2026-09-18 新增）

> ☠️ **2026-09-21 定案：本节的框架错了一处 —— "Nxt-FC 的 submodule 钉的那支 PX4" ≠ "z0 飞控里烧的固件"。**
> 真身是 **`Minderring/PX4-Autopilot` @ `4a0e65f2`**（v1.14.3 系的 **MicoAir** 厂商构建，
> 由飞控自报的 `flight_custom_version` 反查出来），**不是 `Peize-Liu/PX4-Autopilot @ 5ba2e91f`**。
> 后果：☠️ ~~**§9.3 的「权威比对基准」路径要换**（真身树里没有 `boards/hkust/`）~~、
> **§9.5.3 的 ADC 推论整段作废**（☠️ ~~那条缺陷不在这块固件里~~ → **2026-10-09 更正：缺陷在，但那份配置本来就带修好的值**）、**§9.2 的"必然是 HKUST fork 的构建"要改成"MicoAir 厂商构建"**。
> 逐处见各小节内的 ☠️ 注；完整证据链见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)。
> **本节对 Nxt-FC 仓库本身的描述、以及 §9.4「不要刷错固件」仍然有效。**
>
> ☠️ **2026-10-09 再更正两处（上面这条 ☠️ 自己也有半句说错了）**：
> ① ~~"§9.3 的路径要换"~~ —— **不用换，`boards/hkust/nxt-dual/init/rc.board_defaults` 就是
> `z0.params` 的权威比对基准**（见 §9.3 的新更正）；
> ② ~~"那条缺陷不在这块固件里"~~ —— 缺陷**在** z0 的板级目标里，但 z0 那份配置**本来就带修好的值**。
> 根因是**把 `4a0e65f2` 一棵树当成了"构建用的那棵树"**：飞控自报 `ver_hw = HKUST_NXT_DUAL`，
> 而板级目录来自它的**子提交 `33bdd21f`**（版本头只记裸 `HEAD`、不带 dirty 标记）。

> **来源**：第一手 `git clone` + `git ls-remote` + `strings` 验证 —— 这句仍成立（它描述的是 **Nxt-FC 这个仓库**）。
> ☠️ **原写「这是本项目对"z0 参数该跟谁比"这个悬而未决问题的权威解答」—— 已作废**，按上面的 ☠️ 重读。

### 9.1 仓库构成

`HKUST-Aerial-Robotics/Nxt-FC` —— **整仓仅 49 个文件**（`find -type f -not -path './.git/*' | wc -l` 实测）：

| 内容 | 数量 | 明细 |
|---|---|---|
| **预编译固件二进制** | **21** | `firmware/{firmware_1.2.x_px4_1.14.x, firwmware_v1.1.x, pre_firmware_v1.2.x}/` 各含 `*_default` / `*_bootloader` 的 `.bin` / `.elf` / `.px4`（★ 注意 `firwmware` 是**上游的拼写错误**） |
| README 插图 | 17 | `image/README/*.png` |
| 顶层脚本与元数据 | 11 | `docker_build.sh`、`init_all_repo.sh`、`update_all_repo.sh`、`.gitmodules`、`.gitignore`、`LICENSE`、`README.md`、`prepare-commit-msg`、`.github/workflows/{manual_build,px4-autopilot}.yml`、`.vscode/c_cpp_properties.json` |

- **两个 submodule 是空的，必须 `git submodule update --init`**：
  - `PX4-Autopilot` → `Peize-Liu/PX4-Autopilot`（develop）
  - `Nxt-FC-Hardware` → `Peize-Liu/NxtPX4-Hardware`（main）
- **硬件设计不在 Nxt-FC 内**，在 `NxtPX4-Hardware/`：`NxtFC-DUAL/NxtPX4.kicad_sch` / `.kicad_pcb` / `.kicad_pro`、`NxtPX4.pdf`、`NXTVersion2.0/STM32H743VIH_pinout_v2.0.xlsx`、`project_lib/3D_Model/*.step`。**无 BOM CSV、无 Gerber**
- ❌ **仓库内没有任何 `.params` 文件**，**也没有 PX4 机型定义**（无 `ROMFS/`、无 init.d diff）

### 9.2 ★★ `hkust_nxt-dual` 的版本可用性（对本项目最关键）

`hkust_nxt-dual` 是 **NxtPX4v2 在 PX4 里的板级目标**（微空官网 `micoair.com` 明确这么写）。它 **2024-04-02 才由 PR #22961 进入官方上游**。

| PX4 ref | `boards/hkust/nxt-dual/` 存在吗 |
|---|---|
| v1.14.0 / **v1.14.3** / release/1.14 | ❌ **不存在** |
| **v1.15.0** / v1.16.0 / main | ✅ 存在 |

**→ z0 跑的 PX4 1.14.3 不是官方支持组合**，必然是厂商构建。（z0 的 1.14 是**飞控出厂自带**的，见 9.5。）

> ☠️ **2026-09-21 更正措辞**：原写「必然是 **MicoAir / HKUST fork** 的构建」—— 这半句把**两件事混了**。
> 实测真身 = **`Minderring/PX4-Autopilot @ 4a0e65f2`**，它的 `boards/` 里是
> `micoair/{h743, h743-v2, h743-aio}`、**没有 `hkust/`** ⟹ 是 **MicoAir 的厂商构建**，
> 与 `Peize-Liu` 那支 **HKUST fork 无关**。见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)。
>
> ☠️ **2026-10-09 再更正**：上面这段的**结论一半错**。真相是 —— **z0 跑的就是 `hkust_nxt-dual`**
> （飞控自报 `ver_hw = HKUST_NXT_DUAL`，8 份飞行日志一致），只是那个板级目录**不在 `4a0e65f2` 这棵树上**，
> 而在它的**子提交 `33bdd21f`** 上（Minderring 自己加的，2024-11-19）。
> 所以正确说法是：**既是 MicoAir 的厂商构建，也跑 `hkust_nxt-dual` 这个板级目标** ——
> 这两件事**不互斥**，正是 §9.2 上半段那句"`hkust_nxt-dual` 是 NxtPX4v2 在 PX4 里的板级目标"的意思。

### 9.3 ★★ **恢复有效（2026-10-09）**：「板级默认参数 —— 权威的比对基准」**

> ☠️ **2026-09-21 曾把本节整节作废**（理由：真身树里没有 `hkust/` 这块板）——
> **那个理由 2026-10-09 被推翻了**：z0 跑的板级目标**就是 `hkust_nxt-dual`**
> （飞控自报 `ver_hw = HKUST_NXT_DUAL`），板级目录来自 `4a0e65f2` 的**子提交 `33bdd21f`**。
> ⟹ **本节录的 `boards/hkust/nxt-dual/init/rc.board_defaults` 就是 `z0.params` 的权威比对基准。**
>
> ✅ **而且它对得上**：把该文件的 **18 条**与 `z0.params` 逐条比，**15 条数值相同**、
> **3 条被改过**（`BAT1_N_CELLS` 4→6 = 6S 电池、`BAT1_V_DIV` 10.1→10.02 = 分压标定、
> `RC_INPUT_PROTO` -1→6）。全文与比对方法见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)④ 的 2026-10-09 块。
>
> ☠️ **作废当时用来替换它的那张表**（`micoair/{h743,h743-v2,h743-aio}` 的默认值）——
> 那三块板**不在候选里**，表里的 `IMU_GYRO_CUTOFF 80/80/100` 等值对 z0 **没有意义**：
>
> | 板 | `BAT1_A_PER_V` | `BAT1_V_DIV` | `IMU_GYRO_CUTOFF` | `SYS_HAS_MAG` |
> |---|---|---|---|---|
> | ~~`micoair/h743`~~ | ~~40~~ | ~~21.2~~ | ~~80~~ | ~~1~~ |
> | ~~`micoair/h743-v2`~~ | ~~40~~ | ~~21.12~~ | ~~80~~ | ~~1~~ |
> | ~~`micoair/h743-aio`~~ | ~~14.14~~ | ~~21.12~~ | ~~100~~ | ~~0~~ |
>
> ☠️ ~~⚠️ **该拿哪一块当基准，取决于"板名 NxtPX4v2 ≠ 固件目标名"这个未决项**…… **定案前不要拿本节的任何一组值当"权威基准"**~~
> —— **已定案**：基准就是本节下面这份 `nxt-dual` 的值。

`boards/hkust/nxt-dual/init/rc.board_defaults`：

```
BAT1_A_PER_V 17     BAT1_N_CELLS 4     BAT1_V_CHARGED 4.2    BAT1_V_DIV 10.1
BAT1_V_EMPTY 3.2    SYS_HAS_MAG 0      PWM_MAIN_TIM0 -4      RC_INPUT_PROTO -1
IMU_GYRO_RATEMAX 2000    CBRK_SUPPLY_CHK 894281
SYS_AUTOSTART 4001  MAV_TYPE 2  CA_AIRFRAME 0  CA_ROTOR_COUNT 4
MC_ROLLRATE_K 0.35  MC_PITCHRATE_K 0.4  MC_YAWRATE_K 1.2  MC_YAWRATE_MAX 360
```

**怎么用**：

- **硬件绑定值**（与板子出厂一致才对）：`BAT1_A_PER_V`、`BAT1_V_DIV`、`PWM_MAIN_TIM0`、`CBRK_SUPPLY_CHK`、`SYS_HAS_MAG`、`RC_INPUT_PROTO`
- **机型值**（可按 z0 实际改）：`SYS_AUTOSTART 4001`、`MC_*RATE_K`、`BAT1_N_CELLS`（板子是 4，**z0 是 6S**）
- **板级没有覆盖 `PWM_MAIN_FUNC*` 和 `MOT_POLE_COUNT`** → 走 PX4 内置默认。所以 `z0.params` 里的电机序 **`3-2-1-4` 是上游手改的**，不是板级默认值
- `rc.board_sensors`：双 BMI088（SPI1 + SPI4，`-R 2`）+ `spl06 -X -a 0x77`

> ⚠️ **注意**：**fork 的 `feat_deprecated_v1.14.x_high_imu` 分支的 `rc.board_defaults` 只有 2 行**（`SYS_HAS_MAG 0`、`IMU_GYRO_RATEMAX 2000`），**没有电池/电机/autostart 参数**。
> ☠️ **2026-09-21：这说的是 fork 的分支，不是 z0 的固件。** 由此推出的
> 「**1.14.x 构建不会自动带这些默认值，必须全部写进 `z0.params`**」**对真身不成立** ——
> 真身三块 `micoair` 板的 `rc.board_defaults` 都自带 `BAT1_*` / `MC_*RATE_K` / `CBRK_SUPPLY_CHK` / `SYS_AUTOSTART` 等（见上表）。

### 9.4 ⚠️⚠️ 不要刷错固件

`Nxt-FC/firmware/firmware_1.2.x_px4_1.14.x/hkust_nxt-v1_default.elf` 对**z0 是错的**。`strings` 证据：

| 项 | 该 bin | z0 需要的 |
|---|---|---|
| build path | `build/hkust_nxt-v1_default` | `hkust_nxt-dual` |
| 板名 | `HKUST_NXT_V1` | dual |
| 版本串 | `v1.14.0-beta1-1676-g4dbcf92a7d-dirty`（`develop_v1.14.x`） | — |
| 传感器脚本 | `bmi088 -A -R 0 -s`（**单 IMU**）+ `bmp388` | **双 BMI088 + spl06** |

**那是给 NxtPX4 v1.2.x 硬件的。刷到 NxtPX4v2 上传感器直接不工作。**

### 9.5 ✅ 对 z0 的结论

1. **固件策略**：**用飞控出厂自带的 PX4 1.14，不自己刷。** 那份是 MicoAir 的构建，是本平台上唯一被验证过的 1.14 组合。
   ✅ **2026-09-21 定案：这份构建的身份已查明** = `Minderring/PX4-Autopilot` @ `4a0e65f2`
   （v1.14.3 系，2024-11-18），其 `msg/` 与上游 v1.14.3 逐字节相同。见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)。
2. **排查动作**（原写「在 QGC 里记录当前固件版本号」，已改走 MAVLink）—— ✅ **已做**：`AUTOPILOT_VERSION` 实测
   `flight_sw_version = 0x010e0300`（⟹ `1.14.3`），`flight_custom_version` ⟹ hash `4a0e65f233`。
3. **⚠️ 一个真实的坑（已用上游 git 历史逐条核实）**：**`ADC_BATTERY_CURRENT_CHANNEL = ADC12_CH(5)` 对 `nxt-dual` 是错的。**
   ☠️ **2026-09-21 定案：实测判为「未中招」，本项已结。**
   ★ **2026-10-09 更正解释**：☠️ ~~这条缺陷不在 z0 的固件里~~ —— **缺陷就在 z0 的板级目标里**
   （`hkust/nxt-dual`，飞控自报 `ver_hw` 证实），但 **z0 那份 `board_config.h:98` 早就是 `ADC12_CH(8)`**
   （厂商分支 `33bdd21f`，与 fork 修复提交 `8161c4cd` 同一 blob）⟹ 「未中招」= **这份配置本来就修好了**。

   上游修复 commit：**`129c0f188a`，2024-11-19，*"hkust_nxt-dual: fix ADC_BATTERY_CURRENT channel mismatch"***

   | PX4 版本 | `nxt-dual` | `nxt-v1` |
   |---|---|---|
   | v1.15.0（2024-08-23） | `ADC12_CH(5)` ❌ | `ADC12_CH(5)` |
   | v1.16.0 / main | **`ADC12_CH(8)`** ✅ | `ADC12_CH(5)` |

   **★ 三点推论（前两条仍成立，第三条 ☠️ 已证伪 —— 但证伪它的理由 2026-10-09 又换了一次）**：
   - 修复**只针对 `nxt-dual`**；`nxt-v1` 在 main 上**仍是 `CH(5)`**（对它是对的）→ **不要照抄 `nxt-v1` 的写法**
   - 版本边界：**上游 v1.15.0 仍带、v1.16.0 才修** ✅（第一手核实，**指上游官方树**）
   - ☠️ **原写「z0 是 1.14.x，连 `hkust/nxt-dual` 这个板级目标都还不存在（v1.15.0 才有）→ 1.14 的构建几乎必然带 `CH(5)`」—— 已作废，这是本节的错误推论。**
     ☠️ 2026-09-21 给的作废理由是「真身树里根本没有 `hkust/`，是 `boards/micoair/h743-aio`」——
     **这个理由 2026-10-09 也作废了**：**z0 的固件恰恰就是含 `hkust/nxt-dual` 的构建**
     （它不在上游 1.14 里，而在厂商分支 `33bdd21f` 里）。**真正的作废理由是**：
     那句话假定"1.14 的构建必然沿用上游那份 `CH(5)`"，而厂商**加这块板时给的就是修好的 `CH(8)`**
     ⟹ **「板级目标在 1.14 里不存在」推不出「必然带 `CH(5)`」**（这条推理错误本身仍然成立）。

   → ☠️ **原写「若中招，电池电流采样读的是错误通道，飞行中的电流/电量保护全是假的」—— 对 z0 不成立。**
   → ✅ **排查方法已执行（2026-09-21）：上电实测未解锁电流读数 ≈0 且非恒定怪值 ⟹ 未中招**
     （读数与判据见 [`params-changes.md`](params-changes.md) §A4）。
     ★ **2026-10-09 更正解释**：☠️ ~~"不是运气 —— 这条缺陷不在这块固件里"~~ ⟹
     **缺陷在 z0 的板级目标里，但这份配置从厂商加这块板起就是修好的 `ADC12_CH(8)`**
     （`33bdd21f:boards/hkust/nxt-dual/src/board_config.h:98`）。**是"本来就修好了"，不是"运气"。**
4. **`MC_*RATE_K` 是 PX4 角速率环增益** —— Phase 2 的 MPC 要架在这一层之上，这几条值最终决定了内环带宽。
5. **用不上的部分**：fork 的 `feat_fast_lab_stable`（UART DMA）、Nxt-FC 的 OmniNxt docker CI，对 z0 无增量价值。
   ☠️ **原写「`pr_fix_hkust_nxt_dual` 的修复已进上游（可用）」—— 2026-09-21 作废**（理由当时写的是"它修的是 `hkust/nxt-dual`，而 z0 的板级目标不是它"）。
   ⚠️ **2026-10-09 再更正**：**z0 的板级目标就是 `hkust/nxt-dual`**，所以那条修法**本来是对口的** ——
   只是 **z0 那份 `board_config.h` 里那一行早就是 `CH(8)`**，**没有东西要打** ⟹
   ~~"Phase 4 重编固件时不能照搬这一行的修法"~~ 这个告诫随之失效（Phase 4 且裁定**不重刷固件**）。
   详见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)④ 的 2026-10-09 块 / §2.7 的 ☠️ 注。

---

## 10. 汇总：本项目实际要做什么（2026-09-18 重写）

> ★ **本节取代了旧的 §8。** 每条都对应上面某个 § 的第一手证据，**没有一条是搜来的**。

### 10.1 一句话结论

**三个被指定参考的项目，只有一个（Fast-Drone-XI35）真的有可抄的东西 —— 而且可抄的只有"状态机 + 动捕对齐"，不是它的控制器。真正决定本项目成败的两条线，一个是 `px4-mpc`（代码骨架），一个是 `Nxt-FC`（飞控固件与板级参数基准）。**

### 10.2 按"项目"分类

```
需要自己写的：
  ├─ 动捕接入节点（相对位姿 + 原点点零）                 ← 蓝本：vrpn_listener.cpp（已验证）
  ├─ Phase 1 定点飞行节点（(A) 级，TrajectorySetpoint）
  │                                                      ← 蓝本：px4ctrl 的状态机（§3.2）
  ├─ 飞行状态机（MANUAL/HOVER/CMD/TAKEOFF/LAND）         ← 骨架：px4ctrl（§3.2）
  ├─ 模型预测控制的 OCP 定义（CasADi / acados_template） ← 蓝本：px4-mpc（§7）
  └─ 模型预测控制 → PX4 的发布节点                       ← 蓝本：px4-mpc（§7）

拿来即用的：
  ├─ acados（源码编译，aarch64）                ← 唯一"必用"依赖（§1）
  ├─ z0.params（★ 行数现算、别抄；2026-10-10 已从飞控重导出） ← 与 **`boards/hkust/nxt-dual/init/rc.board_defaults`** diff（★ 2026-10-09 定案：这一份就是基准；已比过 18 条：15 同 / 3 改）
  ├─ vrpn_listener（用户已验证的 zip）           ← 动捕链路，不改 stamp（mocap-setup 6.2）
  └─ 6 篇论文                                   ← 理论依据，已在仓库

参考但不照搬的：
  ├─ Fast-Drone-XI35 的 px4ctrl 状态机与推力模型  ← §3.2 / §3.6（★ 最有价值的参考）
  ├─ Fast-Drone-XI35 的 motion_capture 插值对齐   ← §3.5
  ├─ Fast-Drone-XI35 的 thrust_calibrate 公式     ← §3.4（⚠️ 只是记录器，要重写）
  ├─ agilicious 的 guard 概念与 acados 佐证       ← §2（⚠️ 许可证禁止转载代码）
  └─ px4-mpc 作为 (C) 级的代码骨架                ← §7

明确不用的：
  ├─ OmniNxt 的一切代码                         ← §4（仓库只有 4 个文件，外部依赖 404）
  ├─ OmniNxt-03.params                          ← §4.2（OneDrive 需登录，取不到）
  ├─ Fast-Drone-XI35 的 PID 控制器与 ctrl_param_fpv.yaml ← §3.3（质量是 0.645 不是 z0 的 0.75）
  ├─ Fast-Drone-XI35 的视觉/规划链（VINS/realsense/det-reg-pnp/planner）← §3.6
  ├─ agilicious 的代码                          ← §2（不存在，且许可证明文禁止转发）
  ├─ Nxt-FC 的预编译固件                        ← §9.4（是 nxt-v1 单 IMU 的）
  └─ px4_ros_com                                ← v1.14+ 已废弃
```

### 10.3 ★ 三条最重要的结论

1. **⚠️ 本项目不在 PX4 官方支持矩阵内。** `hkust_nxt-dual` 板级目标 **2024-04-02 才进上游（≥ v1.15.0）**，而 z0 是 **1.14.x**（飞控出厂自带）。→ **不刷固件、不追上游，把 1.14 当作既定事实。** 见 §9.2。
   ✅ **真身已查明**（2026-09-21）：`Minderring/PX4-Autopilot` @ `4a0e65f2`，v1.14.3 系 MicoAir 厂商构建。
   ☠️ ~~⚠️ **别把它读成"z0 跑 `hkust_nxt-dual`"** —— 真身树里没有 `hkust/`，是 `boards/micoair/*`。~~
   —— **2026-10-09 更正：恰恰相反，z0 跑的就是 `hkust_nxt-dual`**（飞控自报 `ver_hw = HKUST_NXT_DUAL`）。
   `4a0e65f2` 那棵树里没有 `boards/hkust/`，是因为板级目录来自它的**子提交 `33bdd21f`**。
   ⚠️ 该结论**不派生动作**：同日操作者裁定**不重刷固件**。

2. ☠️ **原写「电池电流采样可能是假的（上游把 `ADC_BATTERY_CURRENT_CHANNEL` 从 `CH(5)` 改 `CH(8)`）⟹ 电流/电量保护都是空转」—— 2026-09-23 改为：这条对 z0 不成立。**
   **实测已判「未中招」**（2026-09-21：未解锁时电流读数 ≈0.37 A 且**有变化**，非恒 0 / 非恒定怪值，见 [`params-changes.md`](params-changes.md) §A4）。
   ★ **2026-10-09 更正解释（观测不变）**：☠️ ~~**且真身里根本没有那块板**（无 `boards/hkust/`，`boards/micoair/h743-aio` 写 `/* PC1 */ ADC1_CH(11)`，与引脚相符）~~ ——
   **缺陷就在 z0 的板级目标里，但 z0 那份配置从厂商加这块板起就是修好的 `ADC12_CH(8)`**。
   **⟹ 缺陷描述对上游 `boards/hkust/nxt-dual` 成立（上游 v1.16.0 才修），对 z0 不适用（已修好）** —— 不是"运气好没中招"。见 §9.5。

3. **✅ 架构解耦是对的。** `(C)` 级 `VehicleRatesSetpoint` 在 PX4 文档里标注"**所需状态估计：none**"——MPC 链路完全不依赖飞控定位，与"动捕→EKF2"安全网链路可**独立调试、独立失败**。agilicious 的 `guard` 模块用同样思路（独立 pipeline + 3D 边界）做了三年。见 §2 与 [`px4-interface.md`](px4-interface.md) 3.1。
   ☠️ **2026-10-13（票 13 收口轮）**：这条结论**一个字都不变**（"none"这一格是本项目最重要的架构洞察）。
   只有一处措辞要跟上载体：**"`(C)` 级"指的是输出形态（角速率 + 推力），不是某个消息名** ——
   本项目 2026-10-12 起的载体是 mavros 的 `/mavros/setpoint_raw/attitude`（`AttitudeTarget`），
   `VehicleRatesSetpoint` 只是 PX4 那边的同类消息（[`px4-interface.md`](px4-interface.md) §4.3）。

### 10.4 下一步

**代码层面的取舍已全部落地**：逐文件清单与取舍理由见 [`code-inventory.md`](code-inventory.md)；实施方案见 [`../control/phase1-implementation.md`](../control/phase1-implementation.md)（Phase 1）与 [`../control/phase2-implementation.md`](../control/phase2-implementation.md)（Phase 2）。
