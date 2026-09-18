# 开源项目包级取舍调查

> 用户指定要调研的 5 个项目 + 调查中发现的相关项目，以及在**本项目**里各自怎么用。
> 相关：[`../PROJECT.md`](../PROJECT.md) 第 5.2 节

---

## 0. ⚠️ 调查方法与置信度

| 手段 | 状态 |
|---|---|
| SSH 克隆 / `git ls-remote` | ✅ **可用**（`weisama/MPC_uav`、`Huan-qun/Sim2real-UAV-Params-` 已直取） |
| `WebFetch` 打开网页 | ❌ **被网络策略全线拦截**（`github.com`、`uzh-rpg.github.io`、`oddityrc.com` 等） |
| `WebSearch` 摘要 | ✅ 可用，但**是二手信息** |

**因此**：凡是**没有通过 SSH 直接拿到源码**的项目（`agilicious`、`acados`、`Fast-Drone-XI35`、`OmniNxt`），其**包级结构判断来自搜索摘要与既有知识，未经逐目录核对**。用之前应自己在 Jetson 上 clone 一次确认。

> ⚠️ **已纠正的错误**：本次调查中一个代理用 HTTPS/API 探测 `weisama/MPC_uav` 和 `Huan-qun/Sim2real-UAV-Params-`，得到 404 后**错误地得出"仓库不存在"的结论**。改用 SSH 后立刻成功。**教训见 [`../LOCAL-NOTES.md`](../LOCAL-NOTES.md) 第 3 节。**

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
| **缺 Tera 渲染器** | 报 **`EOFError`** —— 这是 acados 最常见的坑 |
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

## 2. `uzh-rpg/agilicious` —— ⚠️ **只参考架构与论文，拿不到代码**

| 项 | 内容 |
|---|---|
| 定位 | ETH/UZH 的敏捷飞行框架（Science Robotics 2022） |
| **源码可得性** | ❌ **公开仓库没有任何源码** —— 只有 README + 文档 + bib。**代码需向 UZH 申请** |
| 本项目怎么用 | **参考其文档里的架构分层 + 论文里的 MPC 理论** |

**★ 对本项目最有价值的三条信息**：

1. **它的 MPC 输出就是 CTBR（Collective Thrust + Body Rate）**
   → 这是本项目 `(C)` 方案的**最有力佐证**。

2. **它的 Bridge 层只有 `sbus` / `ctrl` / `laird` / `RotorS` —— 不走 PX4、不走 mavros。**
   → 也就是说 agilicious **没有"PX4 Offboard"这条路径**。**本项目不能照抄它的代码，只能借它的分层思想。**

3. **它的 controllers 列表里同时有 MPC、几何控制、PID 和 INDI。**
   → INDI 在其架构里就是**内环**。这与本项目的演进路线 `(C) → (C)+INDI → (D)` 一致。

**结论**：`agilicious` 对本项目是**理论来源**，不是**代码来源**。

---

## 3. `Longer95479/Fast-Drone-XI35` —— ✅ **重点参考（同硬件！）**

| 项 | 内容 |
|---|---|
| 定位 | **哈工大 HIT-K1928-LAB** 的同机架自主飞行栈 |
| 硬件 | **与本项目高度重叠**：OddityRC XI35 + NxtPX4v2/Nxt-FC + Jetson Orin NX + 6S |
| 控制器 | **PID，不是 MPC** |
| ROS 版本 | **ROS1** |

**★ 这是本项目最有价值的参考项目** —— 因为**同一台机器、同一个飞控、同一个机载电脑**上已经有人跑通了完整的自主飞行栈。

### 包级取舍（用户要求"参考其控制以外的功能包"）

| 包 / 文件 | 本项目怎么用 | 理由 |
|---|---|---|
| **`thrust_calibrate.py`** | ✅ **直接拿来用** | ★ **推力曲线标定**工具，正是本项目 [`hardware.md`](hardware.md) 第 10 节待实测清单第 2 项的解法 |
| **`motion_capture`**（动捕包） | ✅ **重点参考** | 动捕接入 + **时延对齐**，与本项目的动捕链路高度重合 |
| **`px4ctrl`** | ✅ **作为基线对照** | ⚠️ 它是**控制器**，本项目要做的是 MPC。但它的**有限状态机**（解锁 → 起飞 → 悬停 → 降落）是现成的、经过实飞验证的**运行骨架**，值得复用其状态机而替换其控制器 |
| **`ego-planner`** 规划链 | 🔶 **第二阶段再说** | 本项目第一阶段不做复杂避障规划。**先能稳定跟踪给定轨迹，再谈规划** |
| `catkin_*` / ROS1 构建体系 | ❌ 不直接搬 | 本项目目标是 ROS2 Humble。**需要移植而非搬运** |

> 📌 **一个重要的架构建议**：`px4ctrl` 的存在意味着你可以**先用它飞通"z0 动捕定点"这个里程碑**（[`../PROJECT.md`](../PROJECT.md) 里程碑 1），再逐步把控制器换成 MPC。**这比从零写 MPC 再一起调试要稳得多**——因为这样 MPC 出问题时你能确定"是 MPC 的问题"而不是"是整个栈的问题"。

> ⚠️ **注意**：该项目的控制器是 PID，**不能把它的控制部分当 MPC 参考**。用户说的"参考其控制以外的功能包"正是这个意思。

---

## 4. `HKUST-Aerial-Robotics/OmniNxt` —— ⚠️ **构型不同，唯一可取的是参数文件**

| 项 | 内容 |
|---|---|
| 定位 | 全向倾转旋翼平台（IROS 2024） |
| **源码可得性** | ⚠️ **主仓库只有 README + misc，引用的外部仓库已 404** |
| **构型** | **全向倾转旋翼 —— 与本项目的固定推力四旋翼完全不同** |
| 共用硬件 | **Nxt-FC 飞控**（PX4 目标板名 `hkust_nxt-dual`） |

**★ 唯一可直接取的产出**：

| 产出 | 价值 |
|---|---|
| **`OmniNxt-03.params`**（公开的同平台 PX4 参数文件） | 🔶 **可作为 `z0.params` 的第二份对照** —— 同飞控、同平台、不同构型，差异部分能帮助判断"哪些参数是飞控硬件绑定的、哪些是构型绑定的" |

**⚠️ 明确的"不要"**：

- ❌ **不要参考它的控制分配** —— 它的构型有倾转舵机（`CA_SV*`），本项目是纯四旋翼
- ❌ **不要参考它的规划/感知** —— 它是全向视觉，本项目首要定位源是动捕
- ❌ **不要指望能跑它的代码** —— 外部依赖已 404

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

## 8. 汇总：本项目实际要做什么

```
需要自己写的：
  ├─ MPC 的 OCP 定义（CasADi / acados_template）      ← 照 px4-mpc 改
  ├─ 动捕 → MPC 的状态输入节点                        ← 照旧机经验
  ├─ MPC → PX4 的 uXRCE-DDS 发布节点                  ← 照 px4-mpc
  └─ 状态机（解锁/起飞/悬停/降落）                     ← 照 px4ctrl 改

拿来即用的：
  ├─ acados（编译安装）                               ← 核心依赖
  ├─ z0.params                                      ← 已归档，待 diff
  ├─ thrust_calibrate.py                              ← 从 Fast-Drone-XI35
  ├─ px4-mpc（作为代码骨架）                           ← ⭐ 最有价值的发现
  └─ 6 篇论文（理论依据）                              ← 已在 MPC_uav 仓库

参考但不照搬的：
  ├─ agilicious（架构分层 + CTBR 佐证，无代码）
  ├─ Fast-Drone-XI35（动捕包 + 状态机 + 标定工具）
  └─ OmniNxt（仅 OmniNxt-03.params 作第二份对照）

明确不用的：
  ├─ OmniNxt 的控制/感知（构型完全不同）
  ├─ Fast-Drone-XI35 的控制器（PID，非 MPC）
  ├─ agilicious 的代码（不存在）
  └─ px4_ros_com（v1.14+ 已废弃）
```
