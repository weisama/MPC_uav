# 第 4 章 参考项目与资料来源

本章回答三个问题：本项目的哪些部分站在外部项目与论文之上；每个来源借了什么、没借什么；以及哪些结论能回到源码或数据。

参考仓库的只读副本放在 `~/refs/`，不进 git。本章凡写版本号，都以 [`refs-manifest.md`](../../reference/refs-manifest.md) 为准；系统的分层、代码结构与公式推导分别留给第 5、6、8 章。

---

## 4.1 证据分层与引用纪律

本章把证据分成四类，让读者知道一句话该信到什么程度。

| 等级 | 定义 | 典型锚点 |
|---|---|---|
| **第一手 · 源码** | 克隆到钉住的 commit，直接读文件或看许可证 | 仓库结构、状态机枚举、板级默认值 |
| **第一手 · 运行或全文** | 在本机构建、实测，或从 PDF 全文抽取 | acados 构建、飞控自报版本、论文摘要 |
| **推断** | 由第一手事实外推，但没有直接验证 | “这套方法在 z0 上也会有效” |
| **二手** | 搜索摘要、旧索引、第三方转述 | 只用于定位线索，不作为结论判据 |

写作时遵守四条：版本只认钉住的 commit 或飞控自报字段；论文数字写成“论文报告了什么”，不写成“本项目验证了什么”；推断必须显式标注；本仓库的状态以 git 提交和票面 `Status` 为准。

<big>**一句话总结：本章把证据分为源码、运行/全文、推断与二手四级；版本只认锚点，推断必须写明。**</big>

---

## 4.2 参考的开源项目

下表给出八个项目的来源、锚点与角色。“是否复用代码”只回答文件是否进入本仓库，不评价借鉴深度。

| 项目 | 来源实验室 | 钉住的 commit / 版本 | 在本项目中的角色 | 是否复用代码 |
|---|---|---|---|---|
| [`acados/acados`](https://github.com/acados/acados) | 弗莱堡大学 syscop（Diehl 组） | `00947ddc`（`v0.6.0-41-g00947ddc3`） | 求解器与代码生成 | 否，外部依赖 |
| [`uzh-rpg/agilicious`](https://github.com/uzh-rpg/agilicious) | UZH-RPG | `50bba45` | 架构与选型佐证 | 否 |
| [`Longer95479/Fast-Drone-XI35`](https://github.com/Longer95479/Fast-Drone-XI35) | 哈工大 HIT-K1928-LAB | `8c80ddb`（分支 `board`） | 同硬件整机参考，代码框架和接口 | 否，状态机重写 |
| [`HKUST-Aerial-Robotics/OmniNxt`](https://github.com/HKUST-Aerial-Robotics/OmniNxt) | 港科大 Aerial Robotics Group | `79ce1d4` | 同系列硬件佐证，代码框架和接口 | 否 |
| [`Huan-qun/Sim2real-UAV-Params-`](https://github.com/Huan-qun/Sim2real-UAV-Params-) | 个人仓库，未注明实验室 | `b7aeebb` | 参数、动捕文档与监听器 | 是，`vrpn_listener` vendored |
| [`weisama/MPC_uav`](https://github.com/weisama/MPC_uav) | 本项目 | `21eb4a7`（Phase 4 收口） | 本报告描述的仓库本身 | 是，本仓库 |
| [`DISCOWER/px4-mpc`](https://github.com/DISCOWER/px4-mpc) | KTH / ETH（ATMOS 团队） | `3e541af` | 10 状态 OCP 与节点骨架 | 是，模型与 OCP 改写 |


### 4.2.1 acados —— 求解器与代码生成

`acados/acados` 是基于 [CasADi](https://web.casadi.org/) 的嵌入式最优控制求解器，能把最优控制问题生成成 C 代码。它用 [HPIPM](https://github.com/giaf/hpipm) 解 QP、[BLASFEO](https://github.com/giaf/blasfeo) 做线性代数、`t_renderer` 生成 C 代码，许可证为 BSD 2-Clause。

**借了什么。** 求解器与代码生成整条链路都作为外部依赖使用，不重新实现：`acados_template` 搭问题，HPIPM、BLASFEO 与 `t_renderer` 在生成和求解时工作。求解器配置沿用成熟选项，具体参数留给第 8 章。机载电脑上只需两处适配：BLASFEO 目标取 `ARMV8A_ARM_CORTEX_A76`（上游没有 A78 目标），HPIPM 取 `GENERIC`。

**没借什么。** 不走 CUDA，也不用预编译二进制：z0 的机载电脑没有 `nvcc` 与 TensorRT，acados 在本机按源码编译，安装到 `~/acados`，构建目录在仓库之外。

**锚点。** `00947ddc394b30c956d9a7794d0bbcbc44c7c050`，2026-09-25。构建细节见 [`acados-orin-build.md`](../../control/acados-orin-build.md)。

<big>**一句话总结：acados 是一个强大的广泛被引用的 MPC 求解器，极大地加速了 MPC 求解，提供 MPC 代码范式与代码生成。**</big>

### 4.2.2 agilicious —— 架构范本，不是代码来源

`uzh-rpg/agilicious` 是 UZH-RPG 的敏捷四旋翼框架，对应论文 `Foehn2022`。仓库钉在 `50bba45`（2023-03-07），只有 38 个文件，全是文档、容器脚本与图片，没有任何 `.cpp`、`.hpp`、`.yaml` 或 `.launch`。

**借了什么。** 借的是四个概念：

- guard：一条与主链路并行的备份链路，主链路异常时接管。z0 没有实现独立的 guard pipeline，实际安全网是 PX4 的 EKF2、遥控接管与 `AUTO.LAND`。
- 求解器选型：它的 MPC 使用 acados + HPIPM，为本项目的选型提供旁证。
- INDI 内环：它的控制器列表把 INDI 放在内环跟踪角速率。本项目把它列为 `(C) → (C)+INDI` 的下一步，**尚未实现**。
- 多 pipeline：它支持多条 pipeline 与运行时切换；本项目只按阶段选择启动配置。

**没借什么。** 一行代码都没有借：仓库里没有代码，许可证也只授权 UZH 批准的研究者学术非商业使用，并限制再分发。接口层同样不能照抄：它走 SBUS、自家控制板与 RotorS，不走 PX4 或 mavros。

**锚点与证据。** `50bba456156f6f438bea212c643b02e162442fa7`；细节见 [`open-source-survey.md`](../../reference/open-source-survey.md) §2。

<big>**一句话总结：agilicious 只借 guard、求解器选型与内环演进思路；代码和接口都没有借用。**</big>

### 4.2.3 Fast-Drone-XI35 —— 同硬件的整机参考

`Longer95479/Fast-Drone-XI35` 是哈工大 HIT-K1928-LAB 的自主飞行栈，钉在 `8c80ddb`（2026-02-02，分支 `board`）。它与 z0 的硬件高度重叠：XI35 机架、NxtPX4v2/Nxt-FC 飞控、Jetson Orin NX、6S 电池；控制器是 PID，框架是 ROS 1。

**借了什么。** 主要是 `px4ctrl` 的状态机：五个状态 `MANUAL_CTRL`、`AUTO_HOVER`、`CMD_CTRL`、`AUTO_TAKEOFF`、`AUTO_LAND` 的划分，以及消息超时、电池电压、遥控打杆接管、offboard 掉线这些实飞判据。z0 的 `traj` 节点从 `WAIT_START` 进 `TRACK` 时接收一次开始边沿，也来自这里。

**改了什么。** 没有搬运 C++，而是在 [`fsm.hpp`](../../../src/control/include/control/fsm.hpp) 里用 ROS 2 重写，并把 XI35 扁平枚举里的两个维度拆开：控制权（`MANUAL` / `AUTO`）与任务态（`INIT` → `STANDBY` → `TAKEOFF` → `HOVER` → `WAIT_START` → `TRACK` → `LAND`）。状态名与判据沿用，结构是本项目自己的。

**没借什么。** PID 控制器、`ctrl_param_fpv.yaml` 的参数值、视觉与规划链、ROS 1 构建体系都没有采用。推力模型只在 XI35 的参数文件注释里出现，`controller.cpp` 实际用的是线性映射加电压补偿；本项目也没有实现它。动捕插值对齐同样只是读过，没有落地。

**许可证边界。** `px4ctrl` 目录是 GPL-3.0；本项目没有把它的源文件或构建依赖引入仓库，只按判据重写。

<big>**一句话总结：XI35 参考了其 FSM （状态划分与实飞判据）；PID、参数与推力模型都没有落地。**</big>

### 4.2.4 OmniNxt —— 同系列硬件的平台佐证

`HKUST-Aerial-Robotics/OmniNxt` 是港科大 Aerial Robotics Group 的全向视觉平台，对应论文 `Liu2024`，仓库钉在 `79ce1d4`（2025-03-09）。它只有 4 个文件（README、许可证、论文 PDF 与一张总览图），README 指向的控制与规划仓库都已 404，参数文件也取不到。

**借了什么。** 只借硬件事实：它用的飞控是 **Nxt-FC**，机载电脑是 **NVIDIA Jetson Orin NX**，与 z0 的飞控同系列、机载电脑同款。论文里的多鱼眼相机与全向感知算法本项目不用。

**没借什么。** 没有代码或参数可借；也不能把它的预编译固件刷到 z0 上：Nxt-FC 仓库里的固件是给 `nxt-v1` 板的，z0 是双 BMI088 + SPL06 的 NxtPX4v2。

**一处更正。** 旧索引把 OmniNxt 写成“全向倾转旋翼、带舵机”。PDF 全文里 `tilt`、`tilting`、`servo` 各 0 次，构型是常规四旋翼；“omnidirectional”指全向视觉。

<big>**一句话总结：OmniNxt 只用于佐证 Nxt-FC 与 Orin NX 这条硬件线；参考其硬件（无人机动力套件）选项。**</big>

### 4.2.5 Sim2real-UAV-Params —— 参数与动捕链路的资料库

`Huan-qun/Sim2real-UAV-Params-` 是同一套配件的动捕实飞资料库，钉在 `b7aeebb`（2026-08-30）。它提供参数、文档与一个可用的 VRPN 监听器，不是控制算法；仓库没有许可证文件。

**借了什么。** `uav1.params` 作为 z0 参数文件的基线，归档为 [`docs/assets/z0.params`](../../assets/z0.params) 后逐项对账；`mocap.md` 整理成 [`mocap-setup.md`](../../reference/mocap-setup.md)；`BOM.md` 的物料信息整理进 [`hardware.md`](../../reference/hardware.md)；`vrpn_listener.zip` 原样 vendored 到 [`src/vrpn_listener/`](../../../src/vrpn_listener/VENDORED.md)，9 个文件与本仓库逐文件 sha256 一致，一行未改。它的 `stamp = now()` 是刻意设计：动捕主机与机载电脑约有 4 秒时钟差，用 `msg_time` 会毁掉 EKF2 融合。

**没借什么。** 上游物理参数表标注“待更新”，其 0.3 kg 是另一台更轻的机器，只能作量级参考；Genesis + `rsl_rl` 的强化学习路线也没有采用。

**边界。** `uav1.params` 与 z0 是“近似同构”，不是同一台机器的实测快照。

<big>**一句话总结：Sim2real 提供参数基线、动捕手册与 VRPN 监听器；参数表和强化学习路线未采用。**</big>

### 4.2.6 MPC_uav —— 本报告描述的仓库本身

`weisama/MPC_uav` 是**本项目自己的仓库**，不是外部参考项目。把它列进本章，是为了说明“哪些东西来自上游、哪些是本项目新增”，而不是把它包装成第三方参考实现。

本章描述的实现状态以 Phase 4 收口提交 **`21eb4a7`**（2026-10-17）为准。四个版本分支分别钉在阶段性落点：`v1.0` → `5ff9709`（Phase 1 收口）、`v2.0` → `175e298`（Phase 2 票面全部 `done`，阶段 spec 未收口）、`v3.0` → `cdddeb5`（Phase 3 收口）、`v4.0` → `f137472`（Phase 4 之后的在制品快照）。

**从上游借来的部分。** 求解器来自 acados；10 状态模型、OCP 与节点骨架来自 `px4-mpc`；状态划分与实飞判据来自 XI35；参数基线、动捕文档与监听器来自 Sim2real；飞控板级事实来自 Nxt-FC 与厂商固件。

**属于本项目的部分。** 七个 ROS 2 包的组织与实现；`control` 把控制权与任务态拆开；`nmpc` 走 mavros 下行；`bench` 的假 PX4 夹具与 SITL 同构机体；三条解析轨迹；报告的数据管线、图与结果表。包与节点见第五、六章，报告管线见第九、十章。

**边界声明。** `model.py` 保留 `px4-mpc` 的 BSD 版权头，`vrpn_listener` 的字节与上游 zip 一致；其余重写部分在代码注释里标注出处。飞行结果只属于 z0。

<big>**一句话总结：MPC_uav 是本项目自己的仓库；上游内容与原创部分在此划清。**</big>

### 4.2.7 px4-mpc —— 10 状态 OCP 的直接蓝本

`DISCOWER/px4-mpc` 是 KTH / ETH ATMOS 团队把 acados 与 PX4 连起来的示例，钉在 `3e541af`（2026-06-15），BSD 3-Clause。它是为 x500 写的，价值在于“怎么把模型、OCP 与节点接起来”。

**借了什么。** 10 状态模型：状态取位置、速度、四元数共 10 维，控制量取总推力与机体三轴角速率共 4 维；角速率不作状态，等价于假设内环完美跟踪。世界系 ENU、机体系 FLU，模型方程与约定可以原样复用。求解器配置与节点骨架也来自它。输出侧的符号约定以 [`sign-map.md`](../../control/sign-map.md) 为唯一权威。

**改了什么。** 参数、上限、代价与载体都按 z0 改过：质量与悬停推力换成 z0 的值；删掉硬编码的 `0.07` 推力标度；角速率上限换成 3.84 / 3.84 / 6.28 rad/s；推力上限按电池取 11.8 N；求解失败与 NaN 分级兜底；控制代价改罚 `u − u_ref`；状态源改成 `/mocap/pose_enu`；删掉 NED→ENU 转换与 `_v1` 双订阅；下游从 `/fmu/in/vehicle_rates_setpoint` 改成 mavros 的 `/mavros/setpoint_raw/attitude`。

**没借什么。** x500 的参数、`0.07` 魔数、占位的 `max_rate = 0.5`、`MPC_HORIZON = 15` 死变量都没有沿用。截至 2026-10-17，上游 23 条远端分支里没有 `VehicleAttitudeSetpoint` 的参考实现，四旋翼分支都发角速率设定点；要做 (B) 级姿态回退，是全新工作。

<big>**一句话总结：px4-mpc 是本项目的 MPC 动力学代码框架的主要参考，参考了 10 状态模型、求解器配置与节点骨架；参数与下行载体按 z0 改过。**</big>

### 4.2.8 Nxt-FC 与 z0 固件 —— 飞控硬件与固件身份

`HKUST-Aerial-Robotics/Nxt-FC` 是港科大 Aerial Robotics Group 的飞控仓库，钉在 `153b7d1`（2024-08-13），GPL-3.0。它包含预编译固件、README 与两个未初始化的 submodule 指针；硬件设计在 `NxtPX4-Hardware` 里，**它本身不是飞控固件的源码仓库**。

**借了什么。** 借的是板级事实：板级目标名 `hkust_nxt-dual`；`rc.board_defaults` 的 18 条默认值中，15 条与 `z0.params` 相同，3 条被改过（`BAT1_N_CELLS`、`BAT1_V_DIV`、`RC_INPUT_PROTO`）；电池电流通道在厂商加入这块板时就已经是 `ADC12_CH(8)`，不需要打补丁。

**没借什么。** 不刷它的预编译固件：那是给 `nxt-v1` 板的（启动脚本启用 BMI088、ICM42688P 与 BMP388），而 z0 是 NxtPX4v2。也不把 submodule 指针当成 z0 的固件身份：gitlink 指向 `Peize-Liu/PX4-Autopilot` @ `5ba2e91f`，z0 不跑这支 fork。

**z0 固件的真身。** 飞控自报 `flight_sw_version = 0x010e0300`，即 `1.14.3`；`flight_custom_version` 的 8 字节还原出 `4a0e65f233`，对应 `Minderring/PX4-Autopilot` @ `4a0e65f2`（2024-11-18），是一份 v1.14.3 系的 MicoAir 厂商构建。其 `msg/` 与上游 v1.14.3 逐字节相同。2026-10-09 操作者已裁定不重刷固件，本章只作事实存档。

<big>**一句话总结：Nxt-FC 本项目飞控的开源仓库**</big>

---

## 4.3 论文

[`docs/assets/papers/`](../../assets/papers) 里有 6 篇 PDF。英文原题按 PDF 首页保留，中文译名放在括号里；引用的数字只写“论文报告了什么”。

### 4.3.1 `Sun2022_TRO_NMPC_vs_DFBC.pdf`

**原题：** *A Comparative Study of Nonlinear MPC and Differential-Flatness-based Control for Quadrotor Agile Flight*（中文译名：《非线性 MPC 与基于微分平坦的控制用于四旋翼敏捷飞行的对比研究》）

**出处与实验室：** IEEE Transactions on Robotics (T-RO), 2022；苏黎世大学机器人与感知组（UZH-RPG），与 4.2.2 的 agilicious 同一研究组。

**链接：** [DOI: 10.1109/TRO.2022.3177279](https://doi.org/10.1109/TRO.2022.3177279) · [arXiv:2109.01365](https://arxiv.org/abs/2109.01365)

**内容。** 在同一台四旋翼上对比非线性 MPC 与微分平坦控制，速度到 20 m/s；两者加上 INDI 内环与气动阻力模型后，论文报告跟踪误差降低 78% 以上。

**用途与对应关系。** 这是 `(C) → (C)+INDI` 路线的直接依据。它没有单独的开源仓库；同组工作见 agilicious（4.2.2）。本项目没有复现 INDI，也没有在 z0 上验证这个数字。

<big>**一句话总结：Sun2022 对比论证 MPC 加 INDI 内环的收益，是本项目控制器内（PID角度环）/外环（MPC）的选型依据，对应同组 agilicious。**</big>

### 4.3.2 `Foehn2022_SciRob_agilicious.pdf`

**原题：** *Agilicious: Open-Source and Open-Hardware Agile Quadrotor for Vision-Based Flight*（中文译名：《Agilicious：面向视觉飞行的开源开放硬件敏捷四旋翼》）

**出处与实验室：** Science Robotics, Vol. 7, Issue 67, 2022；UZH-RPG。

**链接：** [DOI: 10.1126/scirobotics.abl6259](https://doi.org/10.1126/scirobotics.abl6259) · [arXiv:2307.06100](https://arxiv.org/abs/2307.06100)

**内容。** 一套软硬件协同设计的敏捷四旋翼平台，MPC 输出总推力与机体三轴角速率（CTBR），内环用 INDI；论文报告 5 g 加速度、70 km/h 的轨迹跟踪，并描述了 guard 模块。

**用途与对应关系。** 它直接对应 4.2.2 的 agilicious 仓库，是 (C) 级控制与 acados + HPIPM 选型的理论来源。仓库没有可借用的代码，本项目只借分层与控制层级的判断。

<big>**一句话总结：Foehn2022 是 agilicious 平台对应的论文，为 (C) 级控制与 acados 选型提供了理论依据。**</big>

### 4.3.3 `Torrente2021_RAL_DataDrivenMPC.pdf`

**原题：** *Data-Driven MPC for Quadrotors*（中文译名：《面向四旋翼的数据驱动 MPC》）

**出处与实验室：** IEEE Robotics and Automation Letters (RA-L), 2021；UZH-RPG。

**链接：** [DOI: 10.1109/LRA.2021.3061307](https://doi.org/10.1109/LRA.2021.3061307) · [arXiv:2102.05773](https://arxiv.org/abs/2102.05773)

**内容。** 用高斯过程学习气动效应并嵌入 MPC，论文报告在 14 m/s、加速度超过 4 g 时，高速跟踪误差最多降低 70%。

**用途与对应关系。** 这是第二阶段气动补偿的参考；与 agilicious（4.2.2）同组，但没有单独的开源仓库。本项目当前没有气动项，70% 不是 z0 的读数。

<big>**一句话总结：Torrente2021 给出用高斯过程补偿气动效应的路线；与 agilicious 同组，本项目暂未参考。**</big>

### 4.3.4 `Houska2011_Automatica_RTI.pdf`

**原题：** *An auto-generated real-time iteration algorithm for nonlinear MPC in the microsecond range*（中文译名：《一种把非线性 MPC 压到微秒级的自动生成实时迭代算法》）

**出处与实验室：** Automatica, 47 (2011) 2279–2285；K.U. Leuven（Diehl 组）。

**链接：** [DOI: 10.1016/j.automatica.2011.08.020](https://doi.org/10.1016/j.automatica.2011.08.020)

**内容。** 提出实时迭代（RTI）：把 SQP 的准备放在两次采样之间，采样时刻只做一次廉价反馈迭代，使非线性 MPC 的单周期计算压到微秒级。

**用途与对应关系。** 它是 acados（4.2.1）里 `SQP_RTI` 求解器的理论出处，也解释了机载 100 Hz 求解为什么可行。算法由 acados 实现，本项目没有自己写 RTI。

<big>**一句话总结：Houska2011 提出的实时迭代算法，是 acados 中 `SQP_RTI` 求解器的直接理论出处。**</big>

### 4.3.5 `Houska2011_OCAM_ACADOToolkit.pdf`

**原题：** *ACADO toolkit—An open-source framework for automatic control and dynamic optimization*（中文译名：《ACADO Toolkit：一个用于自动控制与动态优化的开源框架》）

**出处与实验室：** Optimal Control Applications and Methods (OCAM), 32:298–312, 2011；K.U. Leuven（Diehl 组）。

**链接：** [DOI: 10.1002/oca.939](https://doi.org/10.1002/oca.939)

**内容。** 介绍 ACADO Toolkit：一个 C++ 的最优控制与动态优化框架，覆盖直接最优控制（含 MPC）以及状态与参数估计，重点讲软件设计与符号表达式处理。

**用途与对应关系。** ACADO Toolkit 是 acados（4.2.1）的前身；读它能理解 acados 为什么强调符号建模与代码生成。本项目没有使用 ACADO Toolkit 的代码。

<big>**一句话总结：Houska2011 介绍的是 acados 的前身 ACADO Toolkit；本项目只借它来理解设计取舍。**</big>

### 4.3.6 `Liu2024_IROS_OmniNxt.pdf`

**原题：** *OmniNxt: A Fully Open-source and Compact Aerial Robot with Omnidirectional Visual Perception*（中文译名：《OmniNxt：一款全开源、紧凑、带全向视觉感知的空中机器人》）

**出处与实验室：** IEEE/RSJ IROS 2024；港科大 Aerial Robotics Group。

**链接：** [DOI: 10.1109/IROS58592.2024.10802134](https://doi.org/10.1109/IROS58592.2024.10802134) · [arXiv:2403.20085](https://arxiv.org/abs/2403.20085)

**内容。** 面向全向视野的空中机器人平台：多鱼眼相机组、飞控 Nxt-FC、Jetson Orin NX，软件提供 Omni-VINS 与 Omni-Depth。

**用途与对应关系。** 它对应 4.2.4 的 OmniNxt 仓库，也佐证 4.2.8 的 Nxt-FC 硬件线。它的感知与规划面向视觉，本项目不采用；构型是常规四旋翼，不是旧索引写的倾转旋翼。

<big>**一句话总结：Liu2024 是 OmniNxt （港科大 Aerial Robotics Group） 对应的论文，也佐证了 Nxt-FC 与 Orin NX 这条硬件线有工程先例。**</big>

---

## 4.4 移植边界与原创性

### 4.4.1 已落地的复用与改写

| 上游 | 落点 | 方式 | 本项目改动 |
|---|---|---|---|
| `acados` @ `00947ddc` | `~/acados` | 外部依赖 | 机载电脑适配：BLASFEO A76、HPIPM GENERIC、不走 CUDA |
| `px4-mpc` @ `3e541af` 的模型与 OCP | [`src/nmpc/nmpc/`](../../../src/nmpc/nmpc) | 复制后改写 | 参数、上限、代价、兜底、状态源、下行载体 |
| `px4-mpc` 的节点骨架 | [`src/nmpc/nmpc/node_core.py`](../../../src/nmpc/nmpc/node_core.py) | 结构参考后重写 | ROS 2 节点、mavros 下行、健康度与兜底 |
| XI35 `px4ctrl` 的状态机 | [`src/control/include/control/fsm.hpp`](../../../src/control/include/control/fsm.hpp) | 按判据重写 | 控制权与任务态显式拆开 |
| Sim2real `vrpn_listener.zip` | [`src/vrpn_listener/`](../../../src/vrpn_listener) | 原样 vendored | 无；新增 `VENDORED.md` 说明来源 |
| Sim2real `uav1.params` 与文档 | [`docs/assets/z0.params`](../../assets/z0.params)、[`docs/reference/`](../../reference) | 基线后整理 | 按 z0 板级默认值与日志改值 |

真正逐字节进入仓库的上游代码只有 `vrpn_listener`；`px4-mpc` 是改写，XI35 是重写，acados 始终是外部依赖。

<big>**一句话总结：真正进入仓库的上游代码只有 VRPN 监听器；px4-mpc 是改写，XI35 是重写。**</big>

### 4.4.2 只有概念、没有代码

- **agilicious 的 guard 与 INDI**：只有架构思路；z0 的安全网是 PX4 的 EKF2、遥控接管与 `AUTO.LAND`，内环仍是 PX4 角速率环。
- **XI35 的推力模型与动捕插值**：读过、记录过，当前代码没有实现。
- **Nxt-FC 的预编译固件**：没有刷；z0 跑的是 Minderring 的 v1.14.3 厂商构建。
- **论文里的方法**：只引用结论，不复现实验；78%、70%、14 m/s、4 g 都是论文读数，不是 z0 读数。

报告里写“参考了某项目”，不等于“某项目替我们飞了”；飞行结果、参数现值与失效判据都来自 z0 自己的数据与代码。

<big>**一句话总结：guard、INDI、XI35 推力模型与固件都没有落地；论文只提供参考结论。**</big>

### 4.4.3 本项目的原创部分

- **ROS 2 包组织与实现**：七个包的分工、节点边界、话题与参数；`control` 把控制权与任务态拆成两个维度。
- **mavros 下行**：MPC 节点走 `/mavros/setpoint_raw/attitude`，把总推力与机体角速率交给 PX4。
- **轨迹与台架**：悬停、正方形、8 字三条解析轨迹；假 PX4 夹具、脚本化动捕、故障注入与同构 SITL 机体。
- **数据与报告**：四源对齐的数据管线、图与结果表、文档与票面记录。

控制与节点部分以 Phase 4 收口提交 `21eb4a7` 为代码锚点，见第五、六章；报告管线属于 Phase 5，见第九、十章。

<big>**一句话总结：包组织、mavros 下行、轨迹、台架与报告管线是本项目原创。**</big>

---

## 4.5 资料来源与复现

本章的依据集中在五份文档与一类目录：

- [`docs/reference/refs-manifest.md`](../../reference/refs-manifest.md)：仓库清单、钉住的 commit、z0 固件身份。**版本以它为准。**
- [`docs/reference/open-source-survey.md`](../../reference/open-source-survey.md)：项目级调查与源码事实。
- [`docs/reference/code-inventory.md`](../../reference/code-inventory.md)：逐文件的“可复用 / 参考 / 不用”取舍。
- [`docs/assets/papers/README.md`](../../assets/papers/README.md)：6 篇论文的中文标注；标题与译名以 PDF 首页复核为准。
- [`src/vrpn_listener/VENDORED.md`](../../../src/vrpn_listener/VENDORED.md)：vendored 监听器的来源、md5、逐文件 sha256 与复核命令。
- `~/refs/`：参考仓库的本地只读副本，在仓库之外、不进 git。

复核命令：

```bash
# 任一参考仓库的钉住 commit
git -C ~/refs/px4-mpc log -1 --format=%H
git -C ~/refs/Nxt-FC  log -1 --format=%H

# acados 的版本串
git -C ~/refs/acados describe --tags

# 论文标题与摘要（第一手全文抽取）
pdftotext docs/assets/papers/Sun2022_TRO_NMPC_vs_DFBC.pdf -
```

使用纪律与 `refs-manifest.md` §3 一致：只读、不 `git pull`、需要新版时另建目录并登记。例外是 acados：上游在 cmake configure 时会往自己的 `lib/` 写文件，所以构建走 out-of-source，产物落在 `~/acados-build*`。

<big>**一句话总结：来源、版本与复现命令集中在五份文档与只读参考副本。**</big>

---

## 第 4 章引用清单

| 主题 | 文件 |
|---|---|
| 参考仓库清单与钉住的 commit | [`docs/reference/refs-manifest.md`](../../reference/refs-manifest.md) |
| 开源项目逐项调查 | [`docs/reference/open-source-survey.md`](../../reference/open-source-survey.md) |
| 逐文件取舍 | [`docs/reference/code-inventory.md`](../../reference/code-inventory.md) |
| 6 篇论文的中文标注 | [`docs/assets/papers/README.md`](../../assets/papers/README.md) |
| 原仓库 README（归档） | [`docs/assets/MPC_uav-README.md`](../../assets/MPC_uav-README.md) |
| 固件身份与 `px4_msgs` 选版 | [`refs-manifest.md §2.5(d)、§2.6`](../../reference/refs-manifest.md) |
| acados 在机载电脑上的构建 | [`docs/control/acados-orin-build.md`](../../control/acados-orin-build.md) |
| px4-mpc 的逐行解读与改动 | [`docs/control/phase2-implementation.md`](../../control/phase2-implementation.md) |
| 输出侧符号映射的唯一权威 | [`docs/control/sign-map.md`](../../control/sign-map.md) |
| 状态机实现与出处注释 | [`src/control/include/control/fsm.hpp`](../../../src/control/include/control/fsm.hpp) |
| 模型实现与 BSD 版权头 | [`src/nmpc/nmpc/model.py`](../../../src/nmpc/nmpc/model.py) |
| vendored 监听器的来源与字节核对 | [`src/vrpn_listener/VENDORED.md`](../../../src/vrpn_listener/VENDORED.md) |

<big>**一句话总结：本章每条来源都能回到仓库内文档、源码或钉住的只读副本。**</big>
