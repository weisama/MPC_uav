# 第 4 章 参考项目与资料来源

本章回答三个问题：本项目的哪些部分站在外部项目与论文之上；每个来源具体借了什么、又明确没有借什么；以及哪些结论可以直接回到源码或数据，哪些只是推断。

本项目不是从零开始。求解器、模型骨架、状态机、动捕参数与飞控板级事实，都能指到一个钉住的 commit 或一份本地文件。但报告也不把外部工作写成自己的实现：凡是别人写的代码、提出的方法、给出的结论，都标出来源；凡是我们改过的，写明改动；凡是只在 z0 上成立的话，才归到本项目名下。

参考仓库的只读副本放在 `~/refs/`，在仓库之外、不进 git。本章凡写版本号，都以 [`refs-manifest.md`](../../reference/refs-manifest.md) 的表为准；凡写“第一手”，都能按文中给出的 commit、路径或命令重跑一遍。系统的分层、代码结构与公式推导分别留给第 5、6、8 章，本章只讲来源与边界。

---

## 4.1 证据分层与引用纪律

### 4.1.1 四类证据

本章把证据分成四类。区分它们不是为了给结论排座次，而是为了让读者知道一句话该信到什么程度。

| 等级 | 定义 | 本章的典型锚点 |
|---|---|---|
| **第一手 · 源码** | 把仓库克隆到钉住的 commit，直接读文件、数文件或看许可证 | 仓库文件数、许可证条款、状态机枚举、10 状态模型、`rc.board_defaults` |
| **第一手 · 运行或全文** | 在本机构建、实测，或从 PDF 全文抽取 | acados 构建与求解返回码、飞控自报版本、`pdftotext` 抽出的论文摘要 |
| **推断** | 由第一手事实外推，但没有直接验证 | “这套方法在 z0 上也会带来同样的收益”“移植成本大约是多少” |
| **二手** | 搜索摘要、旧索引、第三方转述 | 只用于定位线索，不作为结论判据；钉住 commit 的仓库内文档不属此类，它对“该仓库如何设计”是第一手 |

写作时遵守四条：

1. **版本只认锚点。** 仓库版本以 `refs-manifest.md` 的 commit 为准，飞控版本以飞控自报的字段为准；网页、README 与分支名都不算版本证据。
2. **复述与复现分开。** 论文说“误差降低 78%”，正文写“论文报告了 78%”，不写“本项目验证了 78%”。
3. **推断必须显式。** 凡是“对 z0 也会如此”的句子，标注“推断”，并写出它依赖哪一个第一手事实。
4. **本仓库的状态以 git 提交和票面 `Status` 为准。** 不把上游 README 的自我描述当成我们自己的进度。

<big>**一句话总结：本章的证据分为源码、运行、推断与二手四级；版本只认钉住的锚点，推断必须写明。**</big>

### 4.1.2 三次被证据推翻的结论

项目早期有三次结论站不住，后来都被第一手证据推翻。它们直接决定了本章的写法。

**第一次：把 404 当成“仓库不存在”。** 一个代理用 HTTPS 与 GitHub API 探测 `weisama/MPC_uav` 和 `Huan-qun/Sim2real-UAV-Params-`，两者都返回 404，于是判定仓库不存在。GitHub 对未授权的私有仓库**故意返回 404**；换成 SSH 后 `git ls-remote` 立刻成功。教训是：探测失败只说明这条路径没有权限，不说明对象不存在；有 SSH 凭据时就用 `git ls-remote`，并把命令一起记下来（[open-source-survey.md §0](../../reference/open-source-survey.md)）。

**第二次：把 README 与文件名当成全文。** 旧的论文索引把 OmniNxt 写成“全向倾转旋翼、带舵机”。把本地 PDF 抽成文本后，36 KB 正文里 `omnidirectional` 出现 39 次，而 `tilt`、`tilting`、`servo` 各出现 **0 次**；“omnidirectional”指的是多鱼眼相机的全向视野，机体仍是常规四旋翼。同一轮里，README 指向的 `OmniNxt-03.params` 下载链接实际返回的是一张 275 KB 的登录页。教训是：README、landing page 与文件名都不是全文证据，结论要能指回一次可复现的抽取（[open-source-survey.md §0](../../reference/open-source-survey.md)）。

**第三次：拿“我以为的编码”去否证实测的字节。** 飞控报出的 `flight_custom_version` 是 8 个字节，早期检查假定它应当是 git hash 的 ASCII 前 5 个字符，于是给工具加了一道“第 3 到第 8 字节必须落在 `0-9a-f`”的闸门。查源码后才明白：这个字段是 **uint64 的小端二进制镜像**，高 5 字节是 hash 的前 10 个十六进制字符，前 3 字节可被厂商版本号覆盖。z0 实测的 `00 00 00 33 f2 65 0e 4a` 还原出 `4a0e65f233`，与 `z0.params` 头部的 `4a0e65f233000000` 逐字对上。那道闸门会把**任何**真固件都挡掉，已经删除。教训是：先读**产生这个读数的那段源码**，再判断读数真假（[refs-manifest.md §2.5(d)](../../reference/refs-manifest.md)）。

<big>**一句话总结：私有仓库的 404、过时的索引和假定的编码都曾把结论带偏；判据必须落到源码、全文或实测字节。**</big>

---

## 4.2 开源项目

下表先给出八个项目的锚点与角色。后面每个项目单独说明“借了什么、没借什么”；表中“代码是否进入本仓库”一列只回答文件是否落在本仓库，不评价借鉴深度。

| 项目 | 钉住的 commit / 版本 | 在本项目中的角色 | 代码是否进入本仓库 |
|---|---|---|---|
| `acados/acados` | `00947ddc`（`v0.6.0-41-g00947ddc3`） | 模型预测控制求解器与 C 代码生成 | 否，源码构建为外部依赖 |
| `uzh-rpg/agilicious` | `50bba45` | 架构与选型佐证 | 否 |
| `Longer95479/Fast-Drone-XI35` | `8c80ddb`（分支 `board`） | 同硬件整机参考 | 否，状态机按判据重写 |
| `HKUST-Aerial-Robotics/OmniNxt` | `79ce1d4` | 同系列硬件佐证 | 否 |
| `Huan-qun/Sim2real-UAV-Params-` | `b7aeebb` | 参数、动捕文档与监听器来源 | 是，`vrpn_listener` 原样 vendored |
| `weisama/MPC_uav` | `21eb4a7`（Phase 4 收口） | 本报告描述的仓库本身 | 是，本仓库 |
| `DISCOWER/px4-mpc` | `3e541af` | 10 状态 OCP 与节点骨架蓝本 | 是，模型与 OCP 改写 |
| `HKUST-Aerial-Robotics/Nxt-FC` | `153b7d1` | 飞控硬件与板级参数来源 | 否，未刷其固件 |

### 4.2.1 acados —— 求解器与代码生成

`acados/acados` 是基于 CasADi 的嵌入式最优控制求解器，能把最优控制问题生成成 C 代码，再由机载程序调用。它的默认二次规划求解器是 HPIPM，线性代数后端是 BLASFEO，代码生成由 Tera 模板引擎驱动。本项目用的是 2-Clause BSD 许可的版本。

**借了什么。** 整个求解器与代码生成链路都是外部依赖，不重新实现：`acados_template` 负责在 Python 侧搭问题，HPIPM 解 QP，BLASFEO 做线性代数，`t_renderer` 生成 C 代码。求解器选项沿用成熟配置，具体步数、时域与积分器见第 8 章。在 Orin NX 上，BLASFEO 目标取 `ARMV8A_ARM_CORTEX_A76`（上游没有 A78 目标），HPIPM 取 `GENERIC`。

**没借什么。** 不走 CUDA 后端。z0 的 Jetson 没有 `nvcc` 与 TensorRT，CPU 路线已经够用；这条限制是实测出来的，不是偏好。也没有使用预编译二进制，acados 在本机按源码编译，安装目录是 `~/acados`，源码副本在 `~/refs/acados`，构建目录在仓库之外。

**锚点与证据。** `00947ddc394b30c956d9a7794d0bbcbc44c7c050`，2026-09-25，`git describe` 为 `v0.6.0-41-g00947ddc3`。构建配置、`t_renderer` 版本与求解读数见 [`acados-orin-build.md`](../../control/acados-orin-build.md)；本章不复述那里的耗时数字，因为它们会随机器负载变化。

<big>**一句话总结：acados 提供求解器与代码生成，本项目只做源码构建和 Orin 适配，不走 CUDA、不用预编译包。**</big>

### 4.2.2 agilicious —— 架构范本，不是代码来源

`uzh-rpg/agilicious` 是 UZH-RPG（苏黎世大学机器人与感知组）的敏捷四旋翼框架，对应论文 `Foehn2022`。仓库钉在 `50bba456156f6f438bea212c643b02e162442fa7`（2023-03-07）。克隆下来只有 **38 个文件**：Sphinx 文档、README、容器脚本、`.bib` 与图片，`.cpp`、`.hpp`、`.h`、`.yaml`、`.launch`、`.params` 的数量都是 **0**，唯一的 `.py` 是 Sphinx 配置。它不是一个可以直接读代码的上游项目。

**借了什么，以及落地到什么程度。** 借的是四个概念，不是文件：

- **guard。** 它的 guard 是一条与主控制链路并行的独立 pipeline，带独立估计、独立控制器与三维空间边界，主链路异常时接管。这个思路影响了本项目“安全网链路 / 控制链路分开”的架构判断。但必须说清楚：z0 **没有**实现一条 agilicious 式的独立 guard pipeline；实际的安全网是 PX4 自己的 EKF2、遥控打杆接管与 `AUTO.LAND`，控制器节点在失效时停发 setpoint 并请求降落。
- **`acados` + HPIPM 的选型佐证。** agilicious 的文档明确写它的模型预测控制使用 acados 与 HPIPM，这为本项目选择同一套求解器栈提供了独立旁证。
- **INDI 作为内环。** 它的控制器列表里有模型预测控制、几何控制、PID 与 INDI，INDI 被明确放在内环、跟踪机体角速率。这与本项目 `(C) → (C)+INDI` 的演进方向一致；但 INDI 在本项目中**尚未实现**，只是路线图上的下一步。
- **多 pipeline 与运行时切换。** 它支持创建多条 pipeline 并在运行时切换，这为本项目按阶段从 (A) 到 (C) 的分层推进提供了架构上的正当性；本项目的“切换”靠启动菜单与 launch 选择，不是运行时 pipeline 切换。

**没借什么。** 一行代码都没有借，也借不了：许可证只授权 UZH 明确批准的研究者用于学术非商业目的，并明文限制再分发、转让或披露给其他研究组。接口层也不能照抄：它的 Bridge 走 SBUS、自家 `AgiNuttx` 控制板、LAIRD 射频与 RotorS 仿真，完全没有 PX4 或 mavros 这条路。

**证据。** 文件数与许可证条款来自本地克隆与 `docs/license.rst`；`acados` + HPIPM、INDI 内环、guard、pipeline 切换四条来自仓库文档原文。详见 [`open-source-survey.md`](../../reference/open-source-survey.md) §2。

<big>**一句话总结：agilicious 只提供 guard、求解器选型与内环演进的思路，代码与接口都没有借用，guard 本身也未在 z0 落地。**</big>

### 4.2.3 Fast-Drone-XI35 —— 同硬件的整机参考

`Longer95479/Fast-Drone-XI35` 是哈工大 HIT-K1928-LAB 的自主飞行栈，钉在 `8c80ddb511c814e42576d0b8f3532e9b776e1224`（2026-02-02，默认分支 `board`），共 1501 个文件。它与 z0 的硬件高度重叠：OddityRC XI35 机架、NxtPX4v2/Nxt-FC 飞控、Jetson Orin NX、6S 电池。控制器是 PID，框架是 ROS 1，所以它是**同硬件上跑通过整条自主飞行链**的参考，不是控制方法的参考。

**借了什么，并且已经落地。** 主要是 `px4ctrl` 的状态机骨架：

- 五个状态 `MANUAL_CTRL`、`AUTO_HOVER`、`CMD_CTRL`、`AUTO_TAKEOFF`、`AUTO_LAND` 的划分。
- 状态转移里的实飞判据：消息超时、电池电压、遥控打杆接管、offboard 掉线。这些判据比“能不能飞”更接近现场会遇到的失效。
- 开始键的边沿语义。z0 的 `traj` 节点在 `WAIT_START → TRACK` 时接收一次开始边沿，这个先例来自 XI35。

本项目没有逐行搬运这份 C++，而是在 [`fsm.hpp`](../../../src/control/include/control/fsm.hpp) 里用 ROS 2 C++ 重写，并把 XI35 压在一个扁平枚举里的两个维度**显式拆开**：控制权（`MANUAL` / `AUTO`）与任务态（`INIT` → `STANDBY` → `TAKEOFF` → `HOVER` → `WAIT_START` → `TRACK` → `LAND`）。状态名与转移判据沿用，结构是本项目自己的。

**读过但没有落地的部分。** 有两处值得记录，因为它们常被误读成“已经用了”：

- **推力模型。** XI35 的参数文件用注释记录了非线性模型 `F = K1 · V^K2 · (K3·u² + (1−K3)·u)`，并注明 `K3` 对应 PX4 的 `THR_MDL_FAC`；但 `controller.cpp` 实际做的是 `u = a_z / thr2acc_` 的线性映射，加上按电压分段标定的 `volt2HoverPerOverM0` 补偿，`K1`–`K3` 读进结构体后没有被使用。本项目读过这两条思路，但当前代码都没有实现：(A) 级只发位置设定点，(C) 级发的是归一化推力，由 PX4 按自己的推力曲线解释；电压补偿在本轮没有落地。
- **动捕插值对齐。** XI35 的 `motion_capture` 包把 pose 固定，再对 twist 与 accel 做线性插值，合成一条里程计。本项目的动捕节点只转发位姿，没有做这件事；它仍是一条后续参考，不是现状。

**没借什么。** PID 控制器、`ctrl_param_fpv.yaml` 的任何参数值（它的 `mass: 0.645` 是 XI35 的实重，z0 是 0.75 kg；`low_voltage: 18.0` 是竞速口径；同一份 YAML 里 `rc_reverse` 还重复定义了两次）、Python 2 的 `thrust_calibrate.py`（它只记录数据，不拟合）、视觉与规划链、`uav_simulator`，以及整个 ROS 1 构建体系。

**许可证边界。** `px4ctrl` 目录带 GPL-3.0 许可证。本项目没有把它的源文件或构建依赖引入仓库，只按状态划分与转移判据重写；如果以后要直接复制它的代码，需要重新评估许可证兼容性。

<big>**一句话总结：XI35 借出状态划分与实飞判据，PID、参数值和推力模型都没有落地；状态机是本项目按 ROS 2 重写的。**</big>

### 4.2.4 OmniNxt —— 同系列硬件的平台佐证

`HKUST-Aerial-Robotics/OmniNxt` 对应论文 `Liu2024`，仓库钉在 `79ce1d4b69a5a8c861417716adaac00c2bc6f594`（2025-03-09）。它只有 **4 个文件**：`README.md`、`LICENSE`、`misc/OmniNxt.pdf` 与 `misc/system_overview.png`。README 声称提供的控制与规划仓库都已 404；`OmniNxt-03.params` 不在仓库里，链接指向需要登录的 OneDrive。仓库许可证是 GPL-3.0，但没有可复用的代码。

**借了什么。** 只借硬件事实：OmniNxt 用的飞控是 **Nxt-FC**，机载电脑是 **NVIDIA Jetson Orin NX**，与 z0 的飞控同系列、机载电脑同款。这条事实把 z0 的硬件从“自己拼的一台机器”放回一个已知平台上，也为飞控板级参数的比对提供了线索。论文里的多鱼眼相机、Omni-VINS 与 Omni-Depth 属于全向视觉感知，本项目不用。

**没借什么。** 没有代码可借，也没有参数文件可借；它的规划与感知链路面向视觉定位，而本项目的首要定位源是动捕。更重要的是，**不能把它的预编译固件或板级配置刷到 z0 上**：Nxt-FC 仓库里的固件是给 `nxt-v1` 板的，z0 是双 BMI088 + SPL06 的 NxtPX4v2。

**一处更正。** 旧索引曾把 OmniNxt 写成“全向倾转旋翼、带舵机”，并据此说“不要参考它的控制分配”。本地 PDF 全文核实后，这两个前提都不成立：正文里 `tilt`、`tilting`、`servo` 各 0 次，构型是常规四旋翼；“omnidirectional”指的是全向视觉。结论（不参考它的控制分配）仍然成立，但理由换成了“它没有可参考的代码”。这就是 §4.1.2 第二次教训的具体来源。

<big>**一句话总结：OmniNxt 只用于佐证 Nxt-FC 与 Orin NX 这条硬件线；它没有可复用代码，也不是倾转旋翼。**</big>

### 4.2.5 Sim2real-UAV-Params —— 参数与动捕链路的资料库

`Huan-qun/Sim2real-UAV-Params-` 是同一套配件的动捕实飞资料库，钉在 `b7aeebb71cc708445fbac70a4863c736d4674630`（2026-08-30）。它提供的是参数、文档与一个可用的 VRPN 监听器，不是控制算法。本地克隆里没有找到许可证文件；这一点在 §4.4 的边界表里单独说明。

**借了什么，并且已经落地。**

- **`S2R/assets/param/uav1.params`**：z0 参数文件的基线。它被归档为 [`docs/assets/z0.params`](../../assets/z0.params)，再与飞控板级默认值、飞行日志逐项对账。它是“近似同构”的参考，不是 z0 的实测快照；这一点在报告里必须一直带着。
- **`S2R/assets/doc/mocap.md`**：动捕链路搭建手册，整理成 [`mocap-setup.md`](../../reference/mocap-setup.md)，覆盖依赖安装、坐标系、刚体创建、mavros、速度路由、`vrpn_relative_pose` 源码与开机自启。
- **`BOM.md`**：物料清单，整理进 [`hardware.md`](../../reference/hardware.md) 第 11 节。它解决了三个具体问题：遥控器随附 XF 接收机；电池厂商标称 30 A（VTC6 的 Mooch 独立实测 15 A 记在 `hardware.md` §6，不是 BOM 的读数）；动捕球用铝柱架高，因此刚体原点与 IMU 之间几乎必然有 z 向偏移。
- **`S2R/assets/vrpn_listener.zip`**：原样 vendored 到 [`src/vrpn_listener/`](../../../src/vrpn_listener/VENDORED.md)。zip 的 md5 是 `9f43d98fb1e2d03b69067ef43babeda9`，包内 9 个文件与本仓库逐文件 sha256 一致，一行未改。它的 `stamp = now()` 是**刻意设计**：动捕主机与 Jetson 之间约有 4 秒时钟差，用 VRPN 的 `msg_time` 会把消息送进另一个时钟域，毁掉 EKF2 融合。

**没借什么。** 上游 README 里的物理参数表标注“待更新”，其质量 0.3 kg 是另一台更轻的机器，转动惯量与 `KF` / `KM` 只能作量级参考；臂长 0.08 m 反而佐证了 PX4 的 `CA_ROTOR = 0.15` 是默认值、不是实测值。上游的 Genesis + `rsl_rl` 强化学习路线也没有采用：本项目有意识地选择模型预测控制这条路；而且 z0 的 Jetson 没有 `nvcc` 与 TensorRT，GPU 训练得在别的机器上做。上游清单里的 D435i、碳板机臂、保护圈也不在本项目配置里。

**证据边界。** `uav1.params` 与 z0 的关系是“近似同构”，不是“同一台机器”。z0 的固件身份、板级默认值与参数现值分别由飞控自报、`rc.board_defaults` 与 `z0.params` 导出核对；本章不把上游参数表当成 z0 的读数。

<big>**一句话总结：Sim2real 仓库提供参数基线、动捕手册与逐字节 vendored 的 VRPN 监听器；它的物理参数表和强化学习路线没有采用。**</big>

### 4.2.6 MPC_uav —— 本报告描述的仓库本身

`weisama/MPC_uav` 是**本项目自己的仓库**，不是外部参考项目。它最初只有 README 与 6 篇 PDF，后来才逐步长出代码、数据与文档。把它列进本章，是为了说明“哪些东西来自上游、哪些是本项目新增”，而不是把它包装成第三方参考实现。

本章描述的实现状态以 Phase 4 收口提交 **`21eb4a7`**（2026-10-17）为准。仓库的四个版本分支分别钉在阶段性落点：`v1.0` → `5ff9709`（Phase 1 收口）、`v2.0` → `175e298`（Phase 2 票面全部 `done`，阶段 spec 未收口）、`v3.0` → `cdddeb5`（Phase 3 收口）、`v4.0` → `f137472`（Phase 4 之后的在制品快照）；它们都是分支而不是标签。

**从上游借来的部分。** 求解器来自 acados；10 状态模型、OCP 结构与节点骨架来自 `px4-mpc`；状态划分与实飞判据来自 XI35；参数基线、动捕文档与监听器来自 Sim2real；飞控板级事实来自 Nxt-FC 与厂商固件。每一项的落点与改动见 §4.4。

**属于本项目的部分。** 七个 ROS 2 包的组织与实现：`bench`（台架夹具）、`bringup`（launch 与参数）、`control`（状态机与 offboard）、`mocap`（相对位姿）、`nmpc`（模型预测控制核心与节点）、`traj`（三条解析轨迹）、`vrpn_listener`（vendored）。其中 `control` 把控制权与任务态显式拆开，`nmpc` 走 mavros 的 `/mavros/setpoint_raw/attitude` 而不是上游的 `/fmu/in/*`，`bench` 的假 PX4 夹具与 SITL 同构机体也都是本项目自己搭的。包与节点的实现见第五、六章；报告的数据管线、图与结果表属于 Phase 5，见第九、十章。

**边界声明。** 本报告不把上游代码说成我们的实现：`src/nmpc/nmpc/model.py` 保留了 `px4-mpc` 的 BSD 3-Clause 版权头，`src/vrpn_listener/` 的字节与上游 zip 一致；其余按判据重写的部分，在代码注释里标注了出处。反过来，报告里出现的飞行结果只属于 z0，不属于任何上游项目。

<big>**一句话总结：MPC_uav 是本项目自己的仓库，不是外部参考；上游内容与原创部分在这里划清，代码状态锚在 Phase 4 收口提交。**</big>

### 4.2.7 px4-mpc —— 10 状态 OCP 的直接蓝本

`DISCOWER/px4-mpc` 是一个把 acados 与 PX4 连起来的可运行示例，钉在 `3e541afb05350e30a0867420a5df40dbdfa92c13`（2026-06-15），共 40 个文件，BSD 3-Clause 许可；LICENSE 署名为 JaeyoungLim，但源文件头普遍写 PX4 Development Team，复制时按文件头保留。它不属于 PX4 组织。它是为 x500 写的；对本项目的价值在于“怎么把模型、OCP 与节点接起来”，不在于它的参数。

**借了什么，并且已经落地。** 核心是 10 状态模型：状态取位置、速度、四元数共 10 维，控制量取总推力与机体三轴角速率共 4 维；角速率**不作状态**，姿态运动学直接用机体角速率驱动四元数，等价于假设内环完美跟踪。世界系取 ENU、机体系取 FLU，`g = [0, 0, −9.81]`、推力沿机体 z 轴，因此模型方程与 ENU/FLU 约定可以原样复用，输入侧不需要做 NED/FRD 转换；文件本身经过参数化与缓存改造，不是逐字节复制。求解器选项（50 步、1.0 s、`NONLINEAR_LS`、`PARTIAL_CONDENSING_HPIPM`、`GAUSS_NEWTON`、4 级 3 步 ERK、`SQP_RTI`）与节点骨架也来自它。输出侧的符号约定以 [`sign-map.md`](../../control/sign-map.md) 为唯一权威，该表与 `px4-mpc` 的实际源码行做过交叉核对；本章不复制映射数值，避免与权威表漂移。

**改了什么。** 相对参考实现的改动有明确编号，落在 `params.py`、`ocp.py`、`node_core.py` 与 `sign_map.py` 里：

- 质量与悬停推力换成 z0 的值，不再用 x500 的 1 kg / 0.73。
- 删掉硬编码的 `0.07` 推力标度，改成 `1/t_max`；参考实现自己的模型反算约 `0.0744`，`0.07` 本来就低约 6%。
- 角速率上限换成 z0 的 3.84 / 3.84 / 6.28 rad/s，偏航上限不再取一半。
- 推力上限按电池可用能力取 11.8 N，与归一化常数 `t_max` 解耦。
- 求解失败与 NaN 走分级兜底，并把“退化求解照样发”与“映射前拒绝不发”分开记录。
- 控制代价改罚 `u − u_ref`，状态源改成 `/mocap/pose_enu`。
- 删掉 NED→ENU 的输入侧转换与 `_v1` 后缀话题的双订阅 hack：前者在本项目的状态源下是死代码，后者是 PX4 v1.16 才有的机制。
- 下游载体从 `/fmu/in/vehicle_rates_setpoint` 改成 mavros 的 `/mavros/setpoint_raw/attitude`；控制层级仍是 (C) 级，变的只是消息与链路。

**没借什么。** x500 的参数、`0.07` 魔数、占位用的 `max_rate = 0.5`、输入侧的帧变换、`_v1` 兼容代码与 `MPC_HORIZON = 15` 这个定义了却没用的变量都没有沿用。还有一条“没有找到”的事实值得记下：本地克隆是 2026-09 的快照，含 22 条远端分支；2026-10-17 用 `git ls-remote --heads` 复核上游已有 23 条（新增 `dev-elias_branch`），补查后仍没有 `VehicleAttitudeSetpoint` 的参考实现。四旋翼相关分支都发角速率设定点，属于 (C) 级（仓库另有 spacecraft 等非四旋翼分支，不在比较范围内）。如果以后要做 (B) 级姿态接口的回退，那是全新工作，不是从它这里抄一段。

**许可证边界。** BSD 3-Clause，允许修改与再分发，但保留版权声明。`model.py` 的文件头保留了原版权与许可条款，并写明改动点。

<big>**一句话总结：px4-mpc 借出 10 状态模型、求解器配置与节点骨架；参数、魔数、帧变换与下游载体都按 z0 改过。**</big>

### 4.2.8 Nxt-FC 与 z0 固件 —— 飞控硬件与固件身份

`HKUST-Aerial-Robotics/Nxt-FC` 是 z0 那块飞控的官方开源仓库，钉在 `153b7d13609e870a8abc6ff909461a13ea2d69f8`（2024-08-13），GPL-3.0；`git ls-tree -r` 有 51 个条目，其中 49 个普通文件、2 个未初始化的 submodule gitlink（2026-10-17 现算；`refs-manifest.md` 记的 51 即条目数）。它包含三部分：预编译固件二进制、README 插图与顶层脚本，以及两个未初始化的 submodule 指针：`Peize-Liu/PX4-Autopilot` 与 `Peize-Liu/NxtPX4-Hardware`。硬件设计在后者里，是 KiCad 工程；**它本身不是飞控固件的源码仓库**，仓库里没有 `.params`、没有机型定义、也没有 `ROMFS/`。

**借了什么。** 借的是板级事实，不是固件：

- 板级目标名 `hkust_nxt-dual`，以及它对应的传感器配置：双 BMI088 在 SPI1 与 SPI4（rotation 2），气压计 `spl06`。
- `boards/hkust/nxt-dual/init/rc.board_defaults` 的 18 条板级默认值。与 `z0.params` 逐条比，15 条数值相同，3 条被改过：`BAT1_N_CELLS` 4 → 6、`BAT1_V_DIV` 10.1 → 10.02、`RC_INPUT_PROTO` −1 → 6。
- 电池电流通道的事实：`hkust/nxt-dual` 的 `board_config.h` 在厂商加入这块板时就是 `ADC12_CH(8)`，与 fork 的修复提交同一 blob。因此 z0 不需要打那一行补丁。

**没借什么。** 不刷它的预编译固件。`firmware/firmware_1.2.x_px4_1.14.x/` 里的是 `hkust_nxt-v1_default`，对应 `nxt-v1` 板（启动脚本启用 BMI088、ICM42688P 与 BMP388）；z0 是 NxtPX4v2，刷上去传感器直接不工作。也不把 Nxt-FC 的 submodule 指针当成 z0 的固件身份：树里的 gitlink 指向 `Peize-Liu/PX4-Autopilot` @ `5ba2e91f`（该 commit 在 `feat_fast_lab_stable` 上；`.gitmodules` 只声明 URL 与 `branch = develop`），而 z0 **不跑这支 fork**。

**z0 固件的真身。** 飞控自报的 `flight_sw_version = 0x010e0300` 解出 `1.14.3`；`flight_custom_version` 的 8 个字节还原出 `4a0e65f233`，对应 `Minderring/PX4-Autopilot` @ `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`（2024-11-18），是一份 v1.14.3 系的 MicoAir 厂商构建。它那棵树的 `msg/` 与上游 v1.14.3 逐字节相同（185 个文件），而板级目录来自它的子提交 `33bdd21f`（2024-11-19，“add NxtPX4v2 to PX4-Autopilot v1.14.3”）。版本头只记裸 `HEAD`、不带 dirty 标记，所以“树里没有 `boards/hkust/`”与“飞控跑 `hkust_nxt-dual`”并不矛盾。

**两条容易踩的版本陷阱。** 第一，submodule 钉的 commit 不等于飞控里烧的固件；第二，分支名不能当版本推断——`feat_deprecated_v1.14.x_high_imu` 这个名字里写着 1.14，但它的消息定义属于 v1.15 世代。z0 的固件身份只认飞控自报字段与 `msg/` 的逐字节比对。2026-10-09 操作者已裁定**不重刷固件**，所以本章的固件结论只作事实存档，不派生刷机动作。

<big>**一句话总结：Nxt-FC 提供板级目标与默认参数，不提供 z0 的固件；z0 跑的是 Minderring 的 v1.14.3 厂商构建，不刷固件。**</big>

---

## 4.3 论文

`docs/assets/papers/` 里有 6 篇 PDF。按主题分：3 篇控制方法（`Sun2022`、`Foehn2022`、`Torrente2021`）、2 篇工具链（`Houska2011` 的两篇）、1 篇平台（`Liu2024`）。本章的原题均按 PDF 首页核对，中文译名放在括号里；引用的结论只写“论文报告了什么”，不写成“本项目复现了什么”。

### 4.3.1 `Sun2022_TRO_NMPC_vs_DFBC.pdf`

**原题：** *A Comparative Study of Nonlinear MPC and Differential-Flatness-based Control for Quadrotor Agile Flight*（中文译名：《非线性模型预测控制与基于微分平坦的控制用于四旋翼敏捷飞行的对比研究》）

**出处：** IEEE Transactions on Robotics (T-RO), 2022。作者 Sihao Sun、Angel Romero、Philipp Foehn、Elia Kaufmann、Davide Scaramuzza。

**内容。** 把非线性模型预测控制（NMPC）与基于微分平坦的控制（DFBC）放在同一台四旋翼、同一批轨迹上做实证对比，速度到 20 m/s。结论是各有取舍：NMPC 在动力学不可行的轨迹上明显更优，代价是计算耗时更高、存在数值不收敛的风险；DFBC 计算便宜，但激进机动时跟踪变差。两者加上 INDI 内环与气动阻力模型后，论文报告跟踪误差降低 **78% 以上**。

**在本项目中的用途。** 这是 `(C) → (C)+INDI` 演进路线的直接依据，也解释了为什么第一版采用 10 状态、把角速率当作“内环完美跟踪”的输入：它是路线图规划的第一步，不是论文的标准做法。论文同时说明外环用模型预测控制是合理的，内环用 INDI 还能再降低误差。**边界：** 78% 是论文在其实验平台上的结果；本项目没有复现 INDI，也没有在 z0 上验证这个数字。

<big>**一句话总结：Sun2022 用实证说明模型预测控制加 INDI 内环的收益，为本项目的内环演进提供了依据，但 z0 尚未实现 INDI。**</big>

### 4.3.2 `Foehn2022_SciRob_agilicious.pdf`

**原题：** *Agilicious: Open-Source and Open-Hardware Agile Quadrotor for Vision-Based Flight*（中文译名：《Agilicious：面向视觉飞行的开源开放硬件敏捷四旋翼》）

**出处：** Science Robotics, Vol. 7, Issue 67, 2022。作者 Philipp Foehn 等（苏黎世大学机器人与感知组，UZH-RPG）。

**内容。** 一套软硬件协同设计的敏捷四旋翼平台。硬件给出完整机体设计与选型，软件提供统一接口，同时支持模型控制器与神经网络控制器，便于在同一平台上公平对比。论文报告了 5 g 加速度、70 km/h 的轨迹跟踪能力。它的模型预测控制输出是**总推力 + 机体角速率**（CTBR），内环用 INDI 跟踪角速率；文档里还描述了 guard 模块。

**在本项目中的用途。** 这是 `(C)` 控制层级的理论来源：总推力 + 三轴角速率既能保留模型对动力学的作用，又能把 1 kHz 的角速率环与混控留给 PX4。它同时为本项目选择 acados + HPIPM 提供了独立佐证，并给出了 guard 的架构想象。**边界：** 论文与它的公开仓库都不提供可借用给本项目的代码；仓库只有文档，许可证也限制再分发。本项目只借分层与控制层级的判断，不借实现。

<big>**一句话总结：Foehn2022 为 (C) 级控制与 acados 选型提供理论依据；它没有可借用的代码，z0 只借了分层判断。**</big>

### 4.3.3 `Torrente2021_RAL_DataDrivenMPC.pdf`

**原题：** *Data-Driven MPC for Quadrotors*（中文译名：《面向四旋翼的数据驱动模型预测控制》）

**出处：** IEEE Robotics and Automation Letters (RA-L), 2021。作者 G. Torrente、E. Kaufmann、P. Foehn、D. Scaramuzza。

**内容。** 用高斯过程学习气动效应，把学到的气动模型嵌入模型预测控制。动机是标准刚体模型在高速下会因气动效应失准。论文报告在 14 m/s、加速度超过 4 g 的飞行中，高速跟踪误差最多降低 70%。

**在本项目中的用途。** 这是第二阶段（气动补偿）的参考：等基础模型预测控制跑通、要追求更高速度时，再考虑数据驱动方法。**边界：** 本项目当前没有高斯过程模型，没有气动项，也没有在 z0 上做过这项对比；70% 是论文的结果，不是 z0 的读数。

<big>**一句话总结：Torrente2021 给出用高斯过程补偿气动效应的路线，属于本项目的后续参考；z0 当前没有实现气动项。**</big>

### 4.3.4 `Houska2011_Automatica_RTI.pdf`

**原题：** *An auto-generated real-time iteration algorithm for nonlinear MPC in the microsecond range*（中文译名：《一种把非线性模型预测控制压到微秒级的自动生成实时迭代算法》）

**出处：** Automatica, 47 (2011) 2279–2285。作者 Boris Houska、Hans Joachim Ferreau、Moritz Diehl（K.U. Leuven）。

**内容。** 提出自动生成的实时迭代（Real-Time Iteration, RTI）算法。核心是把每个采样周期内的 SQP 迭代拆成两部分：耗时的准备在两次采样之间完成，采样时刻只做一次廉价的反馈迭代。这样即使是非线性模型预测控制，单周期计算也能压到微秒级。

**在本项目中的用途。** 这是 acados 里 `SQP_RTI` 求解器的理论出处，也解释了“在 Jetson 上按 100 Hz 跑模型预测控制”为什么可行：每个控制周期的预算是 10 ms，而 RTI 把大部分计算挪到了周期之外。**边界：** 算法由 acados 实现，本项目没有自己写 RTI；本章只用它解释求解器选项的来源。

<big>**一句话总结：Houska2011 的 RTI 把 SQP 的准备与反馈拆开，是 SQP_RTI 的理论出处，也解释了机载实时求解为何可行。**</big>

### 4.3.5 `Houska2011_OCAM_ACADOToolkit.pdf`

**原题：** *ACADO toolkit—An open-source framework for automatic control and dynamic optimization*（中文译名：《ACADO Toolkit：一个用于自动控制与动态优化的开源框架》）

**出处：** Optimal Control Applications and Methods (OCAM), 32:298–312, 2011。作者 Boris Houska、Hans Joachim Ferreau、Moritz Diehl（K.U. Leuven）。

**内容。** 介绍 ACADO Toolkit：一个用 C++ 实现的最优控制与动态优化框架，覆盖直接最优控制（含模型预测控制）以及状态与参数估计。文章重点讲软件设计、符号表达式处理与可扩展性，并用最优控制与参数估计两个教程例子演示如何搭问题。

**在本项目中的用途。** ACADO Toolkit 是 acados 的前身。读它能理解 acados 的设计取舍从哪来：为什么强调符号建模、为什么用 `acados_template`、为什么生成 C 代码。**边界：** 本项目直接使用 acados，没有使用 ACADO Toolkit 的代码。

<big>**一句话总结：Houska2011 介绍的是 acados 的前身 ACADO Toolkit；本项目只借它理解设计取舍，没有使用其代码。**</big>

### 4.3.6 `Liu2024_IROS_OmniNxt.pdf`

**原题：** *OmniNxt: A Fully Open-source and Compact Aerial Robot with Omnidirectional Visual Perception*（中文译名：《OmniNxt：一款全开源、紧凑、带全向视觉感知的空中机器人》）

**出处：** IEEE/RSJ IROS 2024。作者 Peize Liu、Chen Feng、Yang Xu、Yan Ning、Hao Xu、Shaojie Shen（香港科技大学）。

**内容。** 面向全向视野的空中机器人平台。硬件上自制了多鱼眼相机组与飞控 **Nxt-FC**，机载电脑是 **NVIDIA Jetson Orin NX**；软件上给出 Omni-VINS 与 Omni-Depth，目标是在有限算力下实现精确定位与实时稠密建图。

**在本项目中的用途。** 硬件平台佐证：它用的飞控与 z0 同系列，机载电脑同款，说明 z0 的硬件组合有已知的工程先例。**边界：** 它的感知与规划面向视觉，本项目首要定位源是动捕；它的仓库没有可复用代码，参数文件也取不到。**一处更正：** 旧索引曾写它是“倾转旋翼”，本地 PDF 全文核实为常规四旋翼；“omnidirectional”指全向视觉，不指全向动力。

<big>**一句话总结：Liu2024 佐证 Nxt-FC 与 Orin NX 这条硬件线；它是常规四旋翼，感知与规划部分本项目不采用。**</big>

---

## 4.4 移植边界与原创性

### 4.4.1 已落地的复用与改写

下表只列真正进入本仓库或本机构建的东西。`acados` 是外部依赖，其余按落点列出；“复用方式”一列区分原样、改写与重写。

| 上游 | 落点 | 复用方式 | 本项目的改动 | 许可与边界 |
|---|---|---|---|---|
| `acados/acados` @ `00947ddc` | `~/acados`、`~/refs/acados` | 源码构建为外部依赖 | Orin 适配：BLASFEO A76、HPIPM GENERIC、不走 CUDA | BSD 2-Clause |
| `px4-mpc` @ `3e541af` 的模型与 OCP | `src/nmpc/nmpc/model.py`、`ocp.py` | 复制后改写 | 参数、上限、代价、兜底、状态源、下游载体共九类改动 | BSD 3-Clause，保留版权头 |
| `px4-mpc` 的节点骨架 | `src/nmpc/nmpc/node_core.py`、`mpc_node.py` | 结构参考后重写为 ROS 2 节点 | mavros 下行、健康度与兜底、参考前馈 | BSD 3-Clause |
| XI35 `px4ctrl` 的状态机 | `src/control/include/control/fsm.hpp` | 按状态划分与判据重写 | 控制权与任务态显式拆开；不引入源文件 | 上游 GPL-3.0 |
| Sim2real `vrpn_listener.zip` | `src/vrpn_listener/` | **原样 vendored** | 无；新增 `VENDORED.md` 说明来源 | 上游未附许可证文件，`package.xml` 的许可证字段仍为 `TODO` |
| Sim2real `uav1.params` | `docs/assets/z0.params` | 基线后逐项对账 | 按 z0 板级默认值与飞行日志改值 | 数据参考，不是 z0 实测快照 |
| Sim2real `mocap.md`、`BOM.md` | `docs/reference/mocap-setup.md`、`hardware.md` | 整理与改写 | 补 z0 适配清单与实测读数 | 文档参考 |

`vrpn_listener` 是唯一逐字节原样进入仓库的上游代码。它的 9 个文件与 zip 的 sha256 一致，`VENDORED.md` 是新增文件、不属于 vendored 集合。`model.py` 不是逐字节复制：它保留了原版权头，动力学与四元数工具来自 `px4-mpc`，参数与缓存策略按本项目改过。XI35 的状态机则是按判据重写，没有把 GPL 源文件放进本仓库。

<big>**一句话总结：真正进入仓库的上游代码只有 vendored 的 VRPN 监听器；px4-mpc 是改写，XI35 是重写，acados 是外部依赖。**</big>

### 4.4.2 只有概念、没有代码

有四类内容只以概念或事实的形式进入本项目，容易被误读成“已经实现”，这里逐条说清：

- **agilicious 的 guard。** 借的是“独立链路 + 三维边界”的思路；z0 的安全网仍是 PX4 的 EKF2、遥控接管与 `AUTO.LAND`，没有独立 guard pipeline。
- **agilicious 的 INDI 内环。** 只是演进方向；当前 (C) 级的角速率由 PX4 的 1 kHz 角速率环跟踪，没有 INDI。
- **XI35 的推力模型与动捕插值。** 读过、记录过，当前代码没有实现；`(A)` 级不发送推力，动捕节点只转发位姿。
- **Nxt-FC 的预编译固件。** 没有刷。z0 的固件是 Minderring 的 v1.14.3 厂商构建，操作者已裁定不重刷。
- **论文里的方法。** 只引用结论，不复现实验；78%、70%、14 m/s、4 g 都是论文的读数，不是 z0 的读数。

反过来也要说清：报告里写“参考了某项目”，不等于“某项目替我们飞了”。飞行结果、参数现值与失效判据都来自 z0 自己的数据与代码。

<big>**一句话总结：guard、INDI、XI35 的推力模型与固件都没有落地；论文只提供结论，不提供 z0 的读数。**</big>

### 4.4.3 本项目的原创部分

把上游的部分划出去之后，剩下的才是本项目的增量：

- **ROS 2 的包组织与实现。** 七个包的分工、节点边界、话题与参数；`control` 把控制权与任务态拆成两个维度，而不是沿用上游的扁平枚举。
- **mavros 下行。** 模型预测控制节点走 `/mavros/setpoint_raw/attitude`，把总推力与机体角速率交给 PX4；这与 `px4-mpc` 的 uXRCE-DDS 载体不同。
- **轨迹生成。** 悬停、正方形、8 字三条解析轨迹，以及开始边沿、轨迹原点与完成判据。
- **台架与仿真。** 假 PX4 夹具、脚本化动捕、故障注入、z0 同构 SITL 机体，以及 seam 判据。
- **数据与报告。** 四源对齐的数据管线、图与结果表、文档与票面记录。

控制与节点部分以 Phase 4 收口提交 `21eb4a7` 为代码锚点，分工见第五、六章；报告的数据管线、图与结果表属于 Phase 5，见第九、十章。本章只做归属判断：上游的名字出现在“来源”里，不出现“我们的实现”里。

<big>**一句话总结：包组织、mavros 下行、轨迹、台架与数据管线是本项目原创；上游只出现在来源与改动说明里。**</big>

---

## 4.5 资料来源与复现

本章的依据集中在五份文档与一类目录：

- [`docs/reference/refs-manifest.md`](../../reference/refs-manifest.md)：仓库清单、钉住的 commit、z0 固件身份的完整证据链。**版本以它为准。**
- [`docs/reference/open-source-survey.md`](../../reference/open-source-survey.md)：项目级调查，含 §0 的两条教训与各项目的源码事实；版本字段编码那条见 `refs-manifest.md` §2.5(d)。
- [`docs/reference/code-inventory.md`](../../reference/code-inventory.md)：逐文件的“可复用 / 参考 / 不用”取舍。
- [`docs/assets/papers/README.md`](../../assets/papers/README.md)：6 篇论文的中文标注；本章的标题与译名以 PDF 首页复核为准。
- `~/refs/`：参考仓库的本地只读副本，在仓库之外、不进 git。
- [`src/vrpn_listener/VENDORED.md`](../../../src/vrpn_listener/VENDORED.md)：vendored 监听器的来源、md5、逐文件 sha256 与复核命令。

复核命令如下，读者可在本机直接重跑：

```bash
# 任一参考仓库的钉住 commit
git -C ~/refs/Fast-Drone-XI35 log -1 --format=%H
git -C ~/refs/px4-mpc        log -1 --format=%H
git -C ~/refs/agilicious     log -1 --format=%H
git -C ~/refs/OmniNxt        log -1 --format=%H
git -C ~/refs/Nxt-FC         log -1 --format=%H
git -C ~/refs/Sim2real-UAV-Params- log -1 --format=%H

# acados 的版本串
git -C ~/refs/acados describe --tags

# 论文标题与摘要（第一手全文抽取）
pdftotext docs/assets/papers/Sun2022_TRO_NMPC_vs_DFBC.pdf -
```

使用纪律与 `refs-manifest.md` §3 一致：只读、不 `git pull`、需要新版时另建目录并登记；引用任何“某项目是这么做的”结论时，带上本表的 commit。例外是 acados：上游在 cmake configure 时会往自己的 `lib/` 写两个文件，所以构建走 out-of-source，产物落在 `~/acados-build*`；除此之外按只读纪律使用。`docs/LOCAL-NOTES.md` 含本机凭据，本章不引用其中的任何内容。

<big>**一句话总结：来源、版本与复现命令集中在 refs-manifest、调查文档、论文索引与 VENDORED.md；参考副本只读、不进仓库。**</big>

---

## 本章引用清单

| 主题 | 文件 |
|---|---|
| 参考仓库清单与钉住的 commit | [`docs/reference/refs-manifest.md`](../../reference/refs-manifest.md) |
| 开源项目逐项调查与 §0 教训 | [`docs/reference/open-source-survey.md`](../../reference/open-source-survey.md) |
| 逐文件取舍 | [`docs/reference/code-inventory.md`](../../reference/code-inventory.md) |
| 6 篇论文的中文标注 | [`docs/assets/papers/README.md`](../../assets/papers/README.md) |
| 原仓库 README（归档） | [`docs/assets/MPC_uav-README.md`](../../assets/MPC_uav-README.md) |
| z0 固件身份与 `px4_msgs` 选版 | [`refs-manifest.md §2.5(d)、§2.6`](../../reference/refs-manifest.md) |
| acados 在 Orin 上的构建 | [`docs/control/acados-orin-build.md`](../../control/acados-orin-build.md) |
| px4-mpc 的逐行解读与改动清单 | [`docs/control/phase2-implementation.md`](../../control/phase2-implementation.md) §2、§2.5 |
| 输出侧符号映射的唯一权威 | [`docs/control/sign-map.md`](../../control/sign-map.md) |
| 状态机实现与出处注释 | [`src/control/include/control/fsm.hpp`](../../../src/control/include/control/fsm.hpp) |
| 模型实现与 BSD 版权头 | [`src/nmpc/nmpc/model.py`](../../../src/nmpc/nmpc/model.py) |
| vendored 监听器的来源与字节核对 | [`src/vrpn_listener/VENDORED.md`](../../../src/vrpn_listener/VENDORED.md) |

<big>**一句话总结：本章的每条来源都能回到仓库内的文档、源码或钉住 commit 的只读副本，可按上表逐项复核。**</big>
