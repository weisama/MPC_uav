# 参考文献索引

> 本项目 `MPC_uav` 仓库的参考文献，**6 篇**。
> 论文来源与关系见 [`../MPC_uav-README.md`](../MPC_uav-README.md) 与 [`../../reference/open-source-survey.md`](../../reference/open-source-survey.md) 第 6 节。

---

## 命名规则

```
<第一作者><年份>_<出处简称>_<主题关键词>.pdf

作者    姓氏，ASCII
年份     正式发表年份
出处     TRO / RAL / SciRob / IROS / Automatica / OCAM
主题     英文短关键词，能一眼区分即可
```

**为什么用 ASCII 文件名**：跨平台、跨工具链最稳，脚本批处理不必操心编码。

---

## A. 理论主线 —— 本项目控制方案的直接依据

### `Sun2022_TRO_NMPC_vs_DFBC.pdf` ★★★

| 项 | 内容 |
|---|---|
| **中文标题** | 非线性 MPC 与微分平坦控制用于四旋翼敏捷飞行的对比研究 |
| 原题 | A Comparative Study of Nonlinear MPC and Differential-Flatness-based Control for Quadrotor Agile Flight |
| 作者 | Sihao Sun, Angel Romero, Philipp Foehn, Elia Kaufmann, Davide Scaramuzza |
| 出处 | IEEE Transactions on Robotics (T-RO), 2022 |

**主要研究内容**：把当时最主流的两种敏捷飞行控制方法——**非线性 MPC（NMPC）**和**基于微分平坦的控制（DFBC）**——放在同一台四旋翼、同一批轨迹上做**实证对比**，飞行速度到 20 m/s。结论是各有取舍：NMPC 在**动力学不可行的轨迹**上明显更优（因为它能显式处理约束），但**计算耗时高、存在数值不收敛的风险**；DFBC 计算便宜但在激进机动时跟踪变差。

**★ 对本项目最关键的一句**：两者**加上内环 INDI + 气动阻力模型**之后，**跟踪误差降低 78 % 以上**。

**用在哪**：这是本项目 `(C) → (C)+INDI` 演进路线的**直接依据**。它同时说明了两件事：(1) 外环用 MPC（模型预测控制）是合理的；(2) **内环用 INDI 能把整体误差再降一大截**。

---

### `Foehn2022_SciRob_agilicious.pdf` ★★★

| 项 | 内容 |
|---|---|
| **中文标题** | Agilicious：面向视觉飞行的开源开放硬件敏捷四旋翼 |
| 原题 | Agilicious: Open-Source and Open-Hardware Agile Quadrotor for Vision-Based Flight |
| 作者 | Philipp Foehn 等（ETH Zürich / UZH-RPG） |
| 出处 | Science Robotics, Vol. 7, Issue 67, 2022 |

**主要研究内容**：一套**软硬件协同设计**的开源敏捷四旋翼平台。硬件上给出完整的机体设计与选型；软件上提供统一接口，**同时支持模型控制器（模型预测控制、几何控制、PID、INDI）和神经网络控制器**，以便在同一平台上公平对比不同方法。实测可达 **5 g 加速度 / 70 km/h** 的轨迹跟踪。

**★ 对本项目最关键的两点**：
1. 它的 **模型预测控制输出就是 CTBR（Collective Thrust + Body Rate）** —— 本项目 `(C)` 方案的最有力佐证。
2. 它的 **controllers 列表里模型预测控制、几何控制、PID、INDI 并列**，INDI 在其架构里担任**内环**。

**用在哪**：`(C)` 方案的**理论来源**。⚠️ 注意它的公开仓库**不含任何源码**（只有 README + 文档），Bridge 层也不走 PX4 —— **只能借它的分层思想，不能抄代码**。

---

### `Torrente2021_RAL_DataDrivenMPC.pdf` ★★

| 项 | 内容 |
|---|---|
| **中文标题** | 面向四旋翼的数据驱动模型预测控制 |
| 原题 | Data-Driven MPC for Quadrotors |
| 作者 | G. Torrente, E. Kaufmann, P. Foehn, D. Scaramuzza |
| 出处 | IEEE Robotics and Automation Letters (RA-L), 2021 |

**主要研究内容**：用**高斯过程（Gaussian Process）学习气动效应**，把学到的气动模型**嵌入模型预测控制** 里。核心动机是：标准模型预测控制用的刚体模型在高速下会因气动效应而失准。实测在 **14 m/s、加速度超 4 g** 的飞行中，**高速跟踪误差最多降低 70 %**。关键词里明确列了 Model Predictive Control / Gaussian Processes / Autonomous Agile Flight。

**用在哪**：本项目**第二阶段**的参考 —— 当基础模型预测控制跑通、要追求更高速度时，用数据驱动方法补气动项。

---

## B. 工具链来源 —— acados 的前身与理论根基

### `Houska2011_OCAM_ACADOToolkit.pdf` ★

| 项 | 内容 |
|---|---|
| **中文标题** | ACADO Toolkit：一个用于自动控制与动态优化的开源框架 |
| 原题 | ACADO toolkit — An open-source framework for automatic control and dynamic optimization |
| 作者 | Boris Houska, Hans Joachim Ferreau, Moritz Diehl（K.U. Leuven） |
| 出处 | Optimal Control Applications and Methods (OCAM), 32:298–312, 2011 |

**主要研究内容**：介绍 **ACADO Toolkit** 这个 C++ 软件环境与算法集合，覆盖**直接最优控制**（含模型预测控制）以及**状态与参数估计**。重点讲了它的软件设计（面向对象、可扩展、可耦合已有优化包）和**符号表达式处理能力**，并用两个教程例子演示如何搭建最优控制问题与参数估计问题。

**用在哪**：**acados 的前身**。读完能理解 acados 的设计取舍从哪来（为什么是 C 代码生成、为什么强调符号建模与 `acados_template`）。

---

### `Houska2011_Automatica_RTI.pdf` ★★

| 项 | 内容 |
|---|---|
| **中文标题** | 一种把非线性 MPC 压到微秒级的自动生成实时迭代算法 |
| 原题 | An auto-generated real-time iteration algorithm for nonlinear MPC in the microsecond range |
| 作者 | Boris Houska 等 |
| 出处 | Automatica, 47 (2011) 2279–2285 |

**主要研究内容**：提出**自动生成的实时迭代（Real-Time Iteration, RTI）算法**。RTI 的核心思想是：把每个采样周期内的 SQP 迭代**拆成两部分**——耗时的部分（准备）在两次采样之间完成，采样时刻只做**一次廉价的反馈迭代**。这样即使非线性模型预测控制，单周期的计算也能压到**微秒级**。

**★ 用在哪**：**acados 里 `SQP_RTI` 求解器的理论出处**。本项目的模型预测控制要在 Jetson 上跑 100 Hz（每周期 10 ms 预算），**读这篇就能理解为什么这件事可行**，以及 `SQP_RTI` 为什么适合硬实时场景。

---

## C. 平台参考

### `Liu2024_IROS_OmniNxt.pdf` ★

| 项 | 内容 |
|---|---|
| **中文标题** | OmniNxt：一款全开源、紧凑、带全向视觉感知的空中机器人 |
| 原题 | OmniNxt: A Fully Open-source and Compact Aerial Robot with Omnidirectional Visual Perception |
| 作者 | **Peize Liu**, Chen Feng, Yang Xu, Yan Ning, Hao Xu, Shaojie Shen（香港科技大学） |
| 出处 | IEEE/RSJ IROS 2024 |

**主要研究内容**：面向**全向视野（omnidirectional FoV）**的空中机器人平台。硬件上自制了**多鱼眼相机组**和**飞控 Nxt-FC**；软件上给出 **Omni-VINS**（定位）与 **Omni-Depth**（稠密建图），目标是在**有限算力下**实现精确定位与实时稠密建图。全部软硬件开源并提供各模块的 Docker 镜像。

**★ 对本项目的价值**：作者列表里明确写着它用的机载电脑是 **Nvidia Jetson Orin NX**，飞控是 **Nxt-FC** —— **与本项目完全同款**。这是"本项目硬件是一个已知平台"这一判断的又一个佐证。

**⚠️ 明确不要参考的部分**：它的**构型是倾转旋翼**（有舵机），与本项目的固定推力四旋翼**完全不同**；它的**感知是视觉**，而本项目首要定位源是动捕。**⇒ 硬件参考价值 > 算法参考价值。**

---

## 速查表

| 文件 | 年份 | 出处 | 一句话 |
|---|---|---|---|
| `Houska2011_OCAM_ACADOToolkit.pdf` | 2011 | OCAM | acados 的前身，C++ 最优控制框架 |
| `Houska2011_Automatica_RTI.pdf` | 2011 | Automatica | `SQP_RTI` 的理论出处，微秒级非线性模型预测控制 |
| `Torrente2021_RAL_DataDrivenMPC.pdf` | 2021 | RA-L | 高斯过程学气动 + 模型预测控制，提速降误差 70 % |
| `Sun2022_TRO_NMPC_vs_DFBC.pdf` | 2022 | T-RO | NMPC vs 微分平坦，**加 INDI 后误差降 78 %** |
| `Foehn2022_SciRob_agilicious.pdf` | 2022 | Science Robotics | 敏捷平台，**模型预测控制输出 CTBR**，含 INDI 内环 |
| `Liu2024_IROS_OmniNxt.pdf` | 2024 | IROS | 同款硬件（Nxt-FC + Orin NX），全向视觉平台 |

**按主题分**：`Sun2022` / `Foehn2022` / `Torrente2021` 是**控制方法**；`Houska2011` × 2 是**工具链**；`Liu2024` 是**硬件平台**。

**阅读优先级**：`Sun2022` → `Foehn2022` → `Houska2011_Automatica_RTI` → 其余。
