# MPC_uav

四旋翼无人机模型预测控制（MPC）相关文献与代码。

**平台**：`z0` —— OddityRC XI35 Pro 3.5" 涵道机架（152 mm 轴距）+ 微空 NxtPX4v2 飞控（PX4 1.14.x）+ Jetson Orin NX 8GB，Nokov 动捕定位。
**目标**：用 **acados** 生成 MPC 求解器，跑在 Jetson 上，经 **PX4 Offboard**（`VehicleRatesSetpoint`，即 CTBR 形态）控制飞机。

---

## 文档（`docs/`）

| 文件 | 内容 |
| --- | --- |
| [`docs/PROJECT.md`](docs/PROJECT.md) | ★ **主蓝图**。项目定位、硬件构型、系统架构、控制方案、飞控参数、里程碑、风险与开放问题 |
| [`docs/CONTEXT.md`](docs/CONTEXT.md) | **术语表（先读它）**。开头澄清 `MPC` 与 PX4 `MPC_*` 参数前缀的命名冲突 |
| [`docs/control/mpc-roadmap.md`](docs/control/mpc-roadmap.md) | MPC 路线图：(A)/(B)/(C)/(D) 四方案对比、`(C) → (C)+INDI → (D)` 演进、模型与约束、acados 在 Jetson 上的部署要点 |
| [`docs/reference/hardware.md`](docs/reference/hardware.md) | 硬件规格调查；★ 第 6.2 节 **电池功率瓶颈**、第 9 节 MPC 建模参数、第 10 节待测清单 |
| [`docs/reference/params-analysis.md`](docs/reference/params-analysis.md) | `z0.params` 逐项分析（哪些可复用 / 哪些必须在飞控上核实） |
| [`docs/reference/px4-interface.md`](docs/reference/px4-interface.md) | 动捕 / PX4 / Offboard 接口事实；第 7 节 **已纠正的错误认知**（真实踩过的坑） |
| [`docs/reference/mocap-setup.md`](docs/reference/mocap-setup.md) | ★ **动捕链路搭建手册**：依赖安装、坐标系标定、刚体创建、mavros 启动、`vrpn_relative_pose` 源码、开机自启、z0 适配清单 |
| [`docs/reference/open-source-survey.md`](docs/reference/open-source-survey.md) | 开源项目包级取舍（acados / agilicious / Fast-Drone-XI35 / OmniNxt / Sim2real-UAV-Params- / px4-mpc） |
| [`docs/LOCAL-NOTES.md`](docs/LOCAL-NOTES.md) | ⚠️ **本机凭据与网络信息**（含机载电脑密码）—— **private 仓库专用，勿 fork 外传** |

> ⚠️ **`docs/` 是 private 资料**：含机器密码、内网 IP、GitHub 身份信息。**不要让这个仓库变成 public。**

## 参考论文（`docs/assets/papers/`）

| 文件 | 标题 | 出处 | 年份 |
| --- | --- | --- | --- |
| `Sun2022_TRO_NMPC_vs_DFBC.pdf` | A Comparative Study of Nonlinear MPC and Differential-Flatness-based Control for Quadrotor Agile Flight | IEEE Transactions on Robotics (T-RO) | 2022 |
| `Torrente2021_RAL_DataDrivenMPC.pdf` | Data-Driven MPC for Quadrotors | IEEE Robotics and Automation Letters (RA-L) | 2021 |
| `Foehn2022_SciRob_agilicious.pdf` | Agilicious: Open-Source and Open-Hardware Agile Quadrotor for Vision-Based Flight | Science Robotics, Vol. 7, Issue 67 | 2022 |
| `Liu2024_IROS_OmniNxt.pdf` | OmniNxt: A Fully Open-source and Compact Aerial Robot with Omnidirectional Visual Perception | IEEE/RSJ IROS | 2024 |
| `Houska2011_OCAM_ACADOToolkit.pdf` | ACADO toolkit — An open-source framework for automatic control and dynamic optimization | Optimal Control Applications and Methods, 32:298–312 | 2011 |
| `Houska2011_Automatica_RTI.pdf` | An auto-generated real-time iteration algorithm for nonlinear MPC in the microsecond range | Automatica, 47:2279–2285 | 2011 |

> 📖 **每篇的中文标注（研究内容 + 对本项目的用处）见 [`docs/assets/papers/README.md`](docs/assets/papers/README.md)。**
> 文件命名规则：`<第一作者><年份>_<出处简称>_<主题关键词>.pdf`。

### 论文速览

- **`Sun2022_TRO_NMPC_vs_DFBC`** — NMPC 与微分平坦控制（DFBC）的实证对比，速度至 20 m/s。结论：NMPC 在动力学不可行轨迹上更优，但计算耗时高、有数值收敛风险；两者加内环 INDI + 气动阻力模型后跟踪误差降低 78% 以上。
- **`Torrente2021_RAL_DataDrivenMPC`** — 用高斯过程建模气动效应并嵌入 MPC，高速跟踪误差最多降低 70%，实测速度至 14 m/s、加速度超 4g。
- **`Foehn2022_SciRob_agilicious`** — Agilicious 软硬件协同设计的开源敏捷四旋翼平台，支持模型与神经网络控制器，5g / 70 km/h 轨迹跟踪。
- **`Liu2024_IROS_OmniNxt`** — 全向视觉感知的开源紧凑型空中机器人平台，含 Nxt-FC 飞控与多鱼眼相机组。
- **`Houska2011_OCAM_ACADOToolkit`** — 自动控制与动态优化的开源 C++ 框架，支持 MPC、状态/参数估计与符号表达式，可自动生成 C 代码。
- **`Houska2011_Automatica_RTI`** — ACADO 的自动生成实时迭代算法，把非线性 MPC 求解压到微秒级。

## 本机参数（`docs/assets/z0.params`）

`z0` 的飞控参数初始配置，928 行 / 914 条参数 / PX4 1.14.3 dev。

内容逐字来自上游 [`Huan-qun/Sim2real-UAV-Params-`](https://github.com/Huan-qun/Sim2real-UAV-Params-) 的 `S2R/assets/param/uav1.params`（上游那台机器叫 "Vehicle 1"），仅文件头加了 6 行出处说明。**尚未在 z0 的飞控上核实** —— 逐项分析见 [`docs/reference/params-analysis.md`](docs/reference/params-analysis.md)。

---

## 状态

📚 **目前只有资料与文档，尚无代码。** 里程碑计划见 [`docs/PROJECT.md`](docs/PROJECT.md) 第 6 节。
