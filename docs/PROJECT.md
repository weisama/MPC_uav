# MPC 控制无人机 —— 项目技术蓝图

> **文档定位**：个人技术蓝图 + 决策记录（ADR 风格），中文为主，术语保留英文原词。
> **读者**：作者本人、未来协作者、半年后回看的自己。
> **配套文件**：[`../CONTEXT.md`](../CONTEXT.md)（术语表，**先读它**）、[`reference/code-inventory.md`](reference/code-inventory.md)（★ 代码取舍）、[`control/phase1-implementation.md`](control/phase1-implementation.md)（★★ **Phase 1 实施清单**）、[`control/phase2-implementation.md`](control/phase2-implementation.md)（★★ **Phase 2 实施清单**）、[`reference/`](reference/)（调查原始记录）、[`control/mpc-roadmap.md`](control/mpc-roadmap.md)（Phase 2 资料汇总）
> **后续去向**：本目录对应 [`weisama/MPC_uav`](https://github.com/weisama/MPC_uav) 仓库的 `docs/` 子目录。
> **最后更新**：2026-09-18

---

## 0. 一句话概括

本项目这台无人机命名为 **`z0`**，全部文档一律用这个代号，不用"新机/本机/这台机"等代称。

**在 `z0` 上**（一台**已经组装完成、已经实飞验证过**的 3.5 英寸 6S 涵道四旋翼），用 **acados 生成 MPC（模型预测控制）求解器**，跑在 **Jetson Orin NX** 上，通过 **PX4 飞控**控制飞机，以 **Nokov 动捕**作为定位源。目标不是"造一台无人机"，而是**建成一个可复用的 MPC 飞行平台**。

**核心判断**：这个项目**不是从零开始**。硬件、动捕链路、仿真环境、飞控参数都有现成资产（详见 1.2）。真正全新的只有 **MPC 这一层**。

---

## 1. 项目定位与现状

### 1.1 目标（Q1 答案：平台建设型）

建成一台**可长期复用的自主无人机平台**，MPC 是第一块而非唯一目的。副产物是发表论文。

**这意味着**：
- 验收标准不应只写"MPC 能飞"，而应包含**可复用性**（模块边界清晰、接口稳定、能被后续任务复用）
- 每一个技术选择都要问"这对后续扩展是加分还是负债"

### 1.2 已有资产 —— 你不是从零开始

| 资产 | 状态 | 证据位置 |
|---|---|---|
| **整机组装 + 遥控器实飞** | ✅ 完成 | 用户确认；**z0 目前只遥控器飞过** |
| **动捕 → PX4 定点/Offboard 飞行** | ✅ **已在旧机上实现** | `weisama-log/25.09.30_s2-f290.md` 标题即「已实现动捕定点飞行」 |
| **动捕延迟实测 = 50 ms** | ✅ 已测（**旧机**） | `weisama-log/25.06.28_PX4_log.md`（对 `vehicle_visual_odometry_0` 测出）。⚠️ 只有一句结论、**没有方法/相关峰/原始数据** ⟹ 对 z0 是**参照、不是结论** |
| **★ 动捕延迟在 z0 上实测 ≈ 50 ms** | ✅ **2026-09-28 已测（z0 自己的数）** | 从飞控 ulog log 210 交叉相关 IMU × 动捕（**同一 hrt 时钟**，不含跨机对时误差）。方法/出处/不确定性见 [`reference/mocap-setup.md`](reference/mocap-setup.md) **§6.3**，工具在 [`../tools/tau-analysis/`](../tools/tau-analysis/) |
| **动捕漂移问题已解决** | ✅ 已解决 | 通过调 `EKF2_EVP_NOISE`；见 `25.06.26_mocap.md` |
| **动捕上电返回 (0,0,0) 问题** | ✅ 已解决 | 自写 `vrpn_relative_pose` 记录初始位姿做相对转发 |
| **PX4 SITL 仿真环境** | ✅ 已搭通 | `osrf/ros:humble-desktop-full` 容器 + PX4 1.13.3 + gazebo-classic + mavros |
| **卡尔曼滤波理论** | ✅ 系统自学 | `weisama-log/25.09.25_kalman/`：KF/EKF/**ESKF**/ESIKF 符号化推导 |
| **电机辨识基础** | ✅ 做过 | `weisama-log/homework/`：电机阶跃响应两点法 |
| **C++ / ROS1+ROS2 / mavros / Docker / QGC / 刷机 / systemd** | ✅ 熟练 | 多个自写节点：`offboard_node.cpp`、`vrpn_relative_pose_node.cpp`、`pose_send_node.cpp` |

**关键区分**：

| | 旧机 | **z0**（本项目这台） |
|---|---|---|
| 机架 | 超维空间 S2-F290 | **OddityRC XI35 Pro，152 mm 轴距** |
| 飞控 | Pixhawk + PX4 **1.13.3** | **微空 NxtPX4v2 + PX4 1.14.3 系**（✅ 2026-09-21 定案：真身 = `Minderring/PX4-Autopilot` @ `4a0e65f2`，飞控自报 `flight_sw_version = 1.14.3`。☠️ 09-20/21 一度判为「**v1.15 世代**」—— 那是**拿 fork 当参照物**的误判，已作废，见下方风险表与 [`reference/refs-manifest.md`](reference/refs-manifest.md) §2.5(d)） |
| 机载电脑 | 树莓派 4B (8GB?) | **Jetson Orin NX 8GB** |
| 状态 | 动捕定点飞行已验证 | **只遥控器飞过** |
| 飞控参数 | — | [`assets/z0.params`](assets/z0.params)（源自上游 `uav1.params`，见 5.1） |

→ **动捕链路的所有经验都在旧机上，z0 需要重做一遍**（换机 → 换 EKF2 参数 → 重标定外参）。这是里程碑 1。

### 1.3 全新的是什么（gap 分析）

对 `agilicious`、`acados`、`OmniNxt`、`Fast-Drone-XI35`、`Sim2real-UAV-Params-` 五个关键词做全盘 grep，**用户笔记中命中数全部为 0**。

→ **MPC（模型预测控制）这条线是唯一真正全新的部分。**

⚠️ 但有一个陷阱必须先破除：笔记里出现的 3 处 "MPC"（`MPC_Z_VEL_MAX_UP`、`MPC_XY_VEL_MAX`、"更新其中PID（MPC_xxx）参数"）**全部是 PX4 的 PID 控制器参数，不是模型预测控制**。详见 [`../CONTEXT.md`](../CONTEXT.md) 开头的术语澄清。

### 1.4 值得记录的复用线索：你的硬件是一个**已知平台**

调查发现，**同一套硬件**（OddityRC XI35 涵道机架 + NxtPX4v2/Nxt-FC + Jetson Orin NX + 6S）被至少两个团队公开使用：

| 团队 | 项目 | 产出 |
|---|---|---|
| **哈工大 HIT-K1928-LAB** | [`Fast-Drone-XI35`](https://github.com/Longer95479/Fast-Drone-XI35) | 完整的 ROS1 自主飞行栈（**控制器是 PID，不是 MPC**） |
| **HKUST 航空机器人组** | [`OmniNxt`](https://github.com/HKUST-Aerial-Robotics/OmniNxt) | 论文 + 一份公开的**同平台 PX4 参数文件** `OmniNxt-03.params` |
| **个人 `Huan-qun`** | [`Sim2real-UAV-Params-`](https://github.com/Huan-qun/Sim2real-UAV-Params-) | **同配件的动捕实飞 PX4 参数**（本项目采用，见第 5 节） |

**结论**：你不是第一个做这个平台的人，**不要闭门造车**。

---

## 2. 硬件平台与构型

### 2.1 配件清单

| 类别 | 型号 | 关键规格 |
|---|---|---|
| **机架** | 怪象 OddityRC **XI35 Pro** | 3.5" 涵道机架，**轴距 152 mm**，机架重 118 g（含螺丝/打印件） |
| **飞控** | 微空科技 **NxtPX4v2** | STM32H743VIH6@480MHz、**双 BMI088**、SPL06 气压计、**无磁罗盘**、7 路 UART、**无以太网**、8 路 DShot/BDShot、27×32×8 mm / 6.5 g、20×20 mm 安装孔 |
| **电调** | 微空科技 **33A 四合一**（蓝鸟 48K） | 4-in-1 ESC |
| **电机** | OddityRC **Spinnybois 2006-2150KV**（升级款） | 定子 20×6 mm，**12 槽 14 极**，KV=2150，6S 定位 |
| **螺旋桨** | HQProp **DT90MMX3**（=T90MMX3） | 直径 **90 mm**（3.5" 级），螺距 **3"**，三叶，PC 材质，**2.35 g/支**，1.5 mm T-mount 轴孔 |
| **电池** | 6S1P **Sony/Murata US18650VTC6** 3000 mAh | 标称 22.2 V / 满电 25.2 V / 实用截止 18.0 V，**15 A 持续（Mooch 实测）**，内阻 13 mΩ/节 |
| **遥控器** | RadioMaster **Pocket** | ELRS 2.4 GHz，EdgeTX，**CRSF over UART @ 420000 baud**，不标配接收机 |
| **机载电脑** | **Jetson Orin NX 8GB** + [ORIN NX V2 底板](https://gitee.com/kit-miao/orin-board) | 见 2.5 |

> ⚠️ **一处用词纠正**：HQProp `T90MMX3` 的正式型号是 **`DT90MMX3`**。你说的"翼飞 ProTek35 桨叶"——ProTek35 的原厂配桨其实是 **Nazgul 3535**，DT90 是被卖家标注为 **ProTek35 兼容**的第三方桨。不影响使用，但文档里应写对。

### 2.2 构型定标（为 MPC 建模铺路）

> **这是本项目 MPC 模型的几何基础。符号约定不写死，后面会花几天找"为什么横滚是反的"。**

| 项 | 值 | 来源 |
|---|---|---|
| 构型 | **四旋翼 X 型**（Quad X，无正前方电机） | 用户确认 |
| PX4 机型 | `SYS_AUTOSTART = 4001`（Generic Quad X） | `z0.params` |
| `MAV_TYPE` | 2（四旋翼） | `z0.params` |
| **轴距** | **152 mm** | 机架规格 |
| **臂长** | **76 mm** | 152 / 2 |
| 机体坐标系 | PX4 约定 **FRD**（x 前、y 右、z 下） | PX4 标准 |
| **电机序** | **`3-2-1-4`** | `z0.params`：`PWM_MAIN_FUNC1..4 = 103, 102, 101, 104`；另有独立调研的实测值吻合 |
| 螺旋桨转向 | ⚠️ **props-in / props-out 待现场确认** | 见第 7 节开放问题 |
| 电调协议 | DShot（`DSHOT_MIN = 0.055`，`DSHOT_3D_ENABLE = 0`） | `z0.params` |
| 电机极对数 | `MOT_POLE_COUNT = 14` | `z0.params` |

**电机序解读**：PX4 的 actuator function 编号中 `101=Motor1`、`102=Motor2`、`103=Motor3`、`104=Motor4`。`PWM_MAIN_FUNCx` 是"输出口 x 分配哪个功能"：

```
物理输出口 1 → Motor 3
物理输出口 2 → Motor 2
物理输出口 3 → Motor 1
物理输出口 4 → Motor 4
```

即 **`3-2-1-4`**。`z0.params` 里的这个值是**刻意改过的非默认值**，两个独立来源互相印证。

> ⚠️ 注意：控制分配的另一半是**几何**（`CA_ROTORx_PX/PY`）。`z0.params` 里这一项是 PX4 模板默认值（算出来轴距 424 mm，与你机架不符），**必须在飞控上核实**。见 [`reference/params-analysis.md`](reference/params-analysis.md) 第 3 节。

### 2.3 建模参数表（MPC 模型的输入）

标注规则：**✅ 已有实测** / 🔶 推算（有依据） / ❌ **需实测或系统辨识**

| 参数 | 值 | 状态 | 备注 |
|---|---|---|---|
| 轴距 | 152 mm | ✅ | 官方规格 |
| 臂长 | 76 mm | ✅ | 由轴距推得 |
| **整机起飞重量** | **750 g** | ✅ | **用户实测** |
| **单电机最大推力** | 1020 g | 🔶 | ⚠️ 这是 **GFD90-3 桨** 的数据，**你的桨是 DT90MMX3，不是同一个桨** |
| 4 电机最大总推力 | 4080 g | 🔶 | 同上 |
| **T/W（电机能力上限）** | **6.29**（实测）/ 5.44（外推） | 🔶 | 实测：`hover_thrust = 0.1589` ⟹ 满量程 46.30 N；外推：4080 / 750 |
| **T/W（电池能供出的上限）** | **1.60 ~ 2.54** | 🔶 | ★ 见 2.4 —— 这是 OCP `F` 上限（`f_max` = 11.8 N）的口径 |
| **悬停油门** | 理论 **42.9%** / PX4 参数写 **50%** | 🔶 | 见下方说明 |
| **悬停电流（总）** | **≈ 7.4 A** | 🔶 | 按 P ∝ T^1.5 推算 |
| 悬停效率 | ≈ 4.5 g/W | 🔶 | 600–750 g 级 3.5" 机的典型区间 |
| 电池电压范围 | 25.2 V 满 / 22.2 V 标称 / 18.0 V 实用截止 | ✅ | 6S |
| 电池最大持续电流 | 15 A（Mooch 实测）/ 30 A（厂商标） | ✅ | 见 2.4 |
| 电机满油门转速 | 41553 RPM（24 V） | ✅ | 实测数据 |
| 电机满油门电流 | 21.81 A / 523 W | ✅ | 实测数据 |
| 满油门力效 | 1.95 g/W | ✅ | 实测数据 |
| **转动惯量 Ixx/Iyy/Izz** | — | ❌ | **需系统辨识或 CAD 估算** |
| **推力系数 kt** | — | ❌ | **需系统辨识** |
| **扭矩系数 kq** | — | ❌ | **需系统辨识** |
| **电机时间常数** | — | ❌ | **需系统辨识** |
| **推力曲线 `THR_MDL_FAC`** | 0.45（PX4 参数） | 🔶 | 可用 `thrust_calibrate.py` 实测 |
| 气动阻力系数 | — | ❌ | 高速飞行才需要，第一版可忽略 |

**关于悬停油门的两个数**：理论推算 42.9%（`√(187.5/1020)`），但 PX4 参数里写的是 `MPC_THR_HOVER = 0.5`。**`0.5` 是 PX4 的默认值**，不一定是实测值。MPC 的**线性化工作点**取哪个，会影响控制效果——建议用 `Fast-Drone-XI35` 的 `thrust_calibrate.py` 实测后回填。

**关于 1020 g 这个数**：这是 GFD90-3 桨的数据，**不是你的 DT90MMX3**。它是**电机能力的参考上限**，不能直接当模型参数用。真实值必须用 `thrust_calibrate.py` 实测。

### 2.4 ★ 电池是全机最大的瓶颈（重要发现）

把 2.3 的数据代进去算，得到一条**对 MPC 设计有决定性影响**的结论：

| 依据 | 持续电流 | 可用总功率 | **反推可达 T/W** |
|---|---|---|---|
| **电池 Mooch 实测（保守）** | **15 A** | 333 W | **仅 1.60** |
| 电池厂商标称（乐观） | 30 A | 666 W | **仅 2.54** |
| 电机 + 桨本身潜力 | 87.2 A | 2092 W | 5.44（外推）/ **6.29（实测）** |

> 推算方式：由实测点（523 W / 1020 g）按 `P ∝ T^1.5` 外推，即 `T ∝ P^(2/3)`。
>
> ★ **2026-09-29 实测对账**（ticket 10）：悬停日志反推 `hover_thrust = 0.1589`（满电口径）
> ⟹ 满量程 **46.30 N** ⟹ T/W **6.29**。与上表最后一行的外推 5.44 同量级（高 16 %，方向合理）。
> ⚠️ 但上表**前两行（15 / 30 A）仍然是外推**，而它们才是 OCP `F` 上限该用的那一层。

**含义（对 MPC 的直接影响）**：

1. **悬停与温和机动没问题** —— 悬停只吃 ≈7.4 A，是 15 A 限额的一半。
2. **敏捷飞行做不了。** 文献里 `agilicious` 那类 20 m/s 机动飞行，通常需要 T/W 有 4–6 的裕度。**按电池真实能力算只有 1.60–2.54。**
3. **模型预测控制的推力可行域必须按电池设，不能按电机设。** 否则它会算出物理上供不出来的推力指令——轻则跟踪变差，重则求解器给出"数学最优但物理不可行"的解。★ **2026-09-29 订正**：这条说的是 `F` 的**上限**（`f_max` = 11.8 N，15 A 口径）。而把 `F` 换算成 PX4 `thrust_body` 的**归一化常数**是**另一个**数（`t_max` = 实测满量程 46.30 N，按电机）—— 两者 2026-09-29 之前是同一个 `T_max`，回填 `hover_thrust` 时把 `F` 上限顺手抬了 **3.9 倍**且测试全绿（[`control/phase2-implementation.md`](control/phase2-implementation.md) §3.3.3）。**改任何一个之前先问：我在改的是"归一化"还是"上限"。**
4. **电压跌落（sag）不可忽略。** VTC6 内阻 13 mΩ/节 → 6S1P 整包 ≈78 mΩ：
   - 30 A 时压降 **2.34 V** → 满电 25.2 V 掉到 22.9 V
   - 87 A 时压降 **6.8 V** → 掉到 18.4 V
   → **"同样的油门指令，不同时刻给出的推力不一样"**。这是 sim-to-real 最典型的坑之一，也是一个**值得做的辨识实验**：测推力-油门曲线在不同 SOC / 电压下的漂移。
5. `z0.params` 里 `BAT_AVRG_CURRENT = 15.0` **恰好等于 VTC6 的实测持续放电能力**——说明这个值被认真设过，是可信的。

> ⚠️ **本节的数字是推算，不是实测。** 精度受三点限制：(a) 1020 g 是另一个桨的数据；(b) 实测点在 24 V 而 6S 是 22.2–25.2 V；(c) `P ∝ T^1.5` 是经验近似。**请用 `thrust_calibrate.py` 实测后替换。**

### 2.5 机载电脑（Jetson Orin NX 8GB）

**环境侦察实测结果**（2026-09-18）：

| 项 | 值 |
|---|---|
| 系统 | Ubuntu 22.04 / **L4T R36.5.0（JetPack 6.2）** / 内核 5.15.185-tegra |
| CPU | **6 核** |
| 内存 | **7.4 GB（可用 5.0 GB）**，Swap 3.7 GB |
| 磁盘 | 233 GB，已用 36 GB（17%） |
| CUDA | `nvidia-l4t-cuda 36.5.0`（runtime） |
| **CUDA Toolkit** | ❌ **宿主无 `nvcc`** |
| **TensorRT** | ❌ **未安装** |
| ROS2 | **Humble 已 source**（299 个包，全是官方核心包） |
| Docker | `chen` **不在 docker 组**，需 `sudo` |

**⚠️ 内存是硬约束**：7.4 GB 要同时跑 Gazebo 仿真 + PX4 SITL + ROS2 + MPC 是**紧张的**。真机飞行时 MPC + 动捕 + mavros 没问题，但**仿真时要留神**。

**⚠️ 没有 `nvcc` / TensorRT** —— 如果有任何基于 GPU 或 TensorRT 的感知/学习组件计划，需要先补装。当前 MPC 路线不需要。

---

## 3. 系统架构

### 3.1 三层结构

```
┌─────────────────────────────────────────────────────────────┐
│  动捕层（Nokov）                                              │
│  10.1.1.198:3883 (VRPN)                                      │
│  输出：刚体位姿 PoseStamped                                    │
└───────────────┬─────────────────────────────────────────────┘
                │ 双路分发 ★
        ┌───────┴────────┐
        │                │
        ▼                ▼
┌───────────────┐  ┌──────────────────────────────────────────┐
│ 安全网链路     │  │ 控制链路                                   │
│ mavros         │  │ MPC 节点（Jetson, 100 Hz）                 │
│ /mavros/       │  │  · 状态输入：动捕位姿（直出，低延迟）        │
│ vision_pose/   │  │  · 求解器：acados 生成的 C 代码             │
│ pose           │  │  · 输出：ω_des (rad/s) + T_des             │
│ → VISION_      │  └──────────────┬───────────────────────────┘
│ POSITION_      │                 │ uXRCE-DDS
│ ESTIMATE       │                 │ /fmu/in/vehicle_rates_setpoint
│ → EKF2 融合    │                 │ /fmu/in/offboard_control_mode (≥2 Hz)
└───────┬────────┘                 │
        │                          │
        ▼                          ▼
┌─────────────────────────────────────────────────────────────┐
│  PX4 飞控（NxtPX4v2, PX4 1.14.3系）                          │
│   · EKF2：融合动捕位姿 → 提供位置/速度估计（安全网用）          │
│   · 角速率环 1 kHz：跟踪 ω_des，做控制分配                     │
│   · 安全逻辑：失控保护、RC 接管、低电、kill switch              │
└───────────────┬─────────────────────────────────────────────┘
                │ DShot
                ▼
          电机 + 电调（33A 四合一）
```

### 3.2 ★ 架构关键点：两条链路可以解耦

PX4 官方 Offboard 文档明确标注了每种 setpoint 所需的**状态估计**：

| 控制量 | 所需估计 | 消息 |
|---|---|---|
| position / velocity / acceleration | **需要** position/velocity | `TrajectorySetpoint` |
| attitude | **none** | `VehicleAttitudeSetpoint` |
| **body_rate** | **none** | **`VehicleRatesSetpoint`** ← 本项目方案 |
| thrust_and_torque | **none** | `VehicleThrustSetpoint` + `VehicleTorqueSetpoint` |
| direct motors | **none** | `ActuatorMotors` |

**含义（这是本项目最重要的架构洞察）**：选 `(C)` 方案时，**MPC 这条链完全不依赖 PX4 的位置估计**。动捕位姿直给 MPC，MPC 出 ω+T 给 PX4，PX4 自己有没有位置估计都不影响 MPC 工作。

→ **"动捕 → PX4 EKF2" 从"必需"降级为"安全网"**（失控保护、RC 接管、自动降落仍需要它）。这让两条链路可以独立调试、独立失败。

### 3.3 接口选择

| 链路 | 方案 | 理由 |
|---|---|---|
| **动捕 → PX4** | **mavros**（`/mavros/vision_pose/pose` → `VISION_POSITION_ESTIMATE` → EKF2） | 已在旧机上跑通，有 50 ms 延迟实测数据、EKF2 调参经验全部可复用 |
| **MPC → PX4** | **uXRCE-DDS**（`/fmu/in/vehicle_rates_setpoint`） | 有现成蓝本 [`DISCOWER/px4-mpc`](https://github.com/DISCOWER/px4-mpc) 可直接对照，省掉自己设计接口 |

**为什么"混合"而不是统一**：mavros 与 uXRCE-DDS Agent 是两个独立进程，各连 PX4 的**不同 UART 口**，在同一个 ROS2 图里共存无冲突。**只在新的一条链路上冒险，另一条保持不变**，出问题时能快速二分定位。

**前提**：NxtPX4v2 需要两个空闲 UART。它有 7 路，`MAV_0 → TELEM1(UART?)`、`MAV_1 → TELEM2(UART4)`，`TELEM4/UART8` 空着。**前提成立。**

> ⚠️ 备选：全走 mavros（用 `setpoint_raw/attitude` 的 `body_rate` 字段）。如果觉得"单一中间件"比"少写代码"更重要，选这条。**这仍是开放决策。**

### 3.4 运行环境

**原则：尽量都跑在 ROS2 Humble。**

| 组件 | 运行环境 |
|---|---|
| MPC 节点、mavros、uXRCE-DDS Agent | **ROS2 Humble 原生**（宿主 Ubuntu 22.04） |
| acados | **宿主原生编译**（aarch64），`ACADOS_SOURCE_DIR` + `LD_LIBRARY_PATH` |
| PX4 SITL 仿真 | 已有的 humble 容器（`osrf/ros:humble-desktop-full` + PX4 1.13.3 + gazebo-classic） |
| noetic 容器 | **仅**用于确实没有 ROS2 版本的 ROS1 包 |

**为什么不用 `ros1_bridge`**：两个 ROS 版本共存的桥接是最脆弱的环节。用户已明确"尽量都运行在 humble 里"——这与"把 `Fast-Drone-XI35` 的少量包移植到 ROS2"的策略一致。

---

## 4. 控制方案

> ⚠️ **本节是资料汇总，不是最终决策。** 架构决策在另一个对话中进行。本节把四个选项的证据摆齐，供那个对话取用。

**建议主线：`(C)`（MPC 出角速率 + 推力），演进方向 `(C) + INDI → (D)`。**

### 4.1 四个选项

| | MPC 输出 | PX4 承担 | 评价 |
|---|---|---|---|
| **(A)** | 位置/速度（`TrajectorySetpoint`） | 完整内环 | ❌ **MPC 被 PX4 位置环抹掉，做了等于没做** |
| **(B)** | 姿态 + 推力（`VehicleAttitudeSetpoint`） | 角速率环 | ⚠️ 姿态环带宽与 MPC 预测会打架；四元数 setpoint 的 yaw 处理麻烦 |
| **(C)** ★ | **角速率 + 推力（`VehicleRatesSetpoint`）** | **仅 1 kHz rate loop** | ✅ **推荐** |
| **(D)** | 电机指令（`ActuatorMotors`） | 无（绕过控制分配） | ⚠️ 第二阶段的终点，**但会禁用 PX4 的 `control_allocator` 与安全网** |

> ⚠️ 纠正一个常见误解：**(B) 不能用 `TrajectorySetpoint` 实现**——该消息**根本没有 thrust 字段**（只有 position/velocity/acceleration/jerk/yaw/yawspeed）。**(B) 的正确消息是 `VehicleAttitudeSetpoint`。**
>
> ⚠️ 另一条：**推力是 `[-1,1]` 归一化，不是 0–1，也不是"相对悬停归一化"。** 数值约定错了，飞机不会飞。

### 4.2 为什么推荐 (C) —— 四条独立论据

1. **模型最简** —— 只需建到"刚体动力学 + 推力/力矩"这一层，**不需要电机模型、不需要混装矩阵的精确标定**。转向约定、桨的推力系数这些坑全部留给 PX4。
2. **带宽分层合理** —— MPC 100 Hz 出 ω_des/T_des，PX4 1 kHz rate loop 做底层跟踪。这是学术界主流。
3. **架构解耦**（见 3.2）—— **`(C)` 不需要 PX4 有位置估计**，两条链路可独立失败。
4. **两份独立佐证**：
   - `agilicious` 的 MPC 输出就是 **CTBR（collective thrust + body rate）**，Bridge 只有 sbus/ctrl/laird/RotorS —— **不走 PX4、不走 mavros**
   - [`DISCOWER/px4-mpc`](https://github.com/DISCOWER/px4-mpc)（210★）用 acados 生成 MPC，发布 `/fmu/in/vehicle_rates_setpoint` —— **与 `(C)` 完全一致，是现成蓝本**

### 4.3 C → D 的三步演进（每步保留前一步的模型与接口）

```
步骤 1:  (C)                          MPC(ω_des, T_des) ──uXRCE-DDS──▶ PX4 rate loop ──DShot──▶ 电机
步骤 2:  (C) + INDI                   MPC(ω_des, T_des) ─────────────▶ INDI ────────▶ PX4 rate loop ──▶ 电机
步骤 3:  (D)                          MPC(ω_des, T_des) ─────────────▶ INDI ────────────────────▶ 电机
```

**为什么 INDI 是这个项目的自然选择**：

- **不依赖精确模型** —— INDI 是增量式的，只用当前角加速度与"操纵效能矩阵"（control effectiveness matrix）。对一台**模型参数全靠辨识、还有电压 sag** 的机器特别合适。
- **`agilicious` 的 controllers 列表里同时有 MPC、几何控制、PID 和 INDI**，INDI 在其架构里就是内环。
- **你自己仓库里的论文支持这条路**：`MPC_uav/papers/Sun2022_TRO_NMPC_vs_DFBC.pdf` 的结论明确写着"**两者加内环 INDI + 气动阻力模型后跟踪误差降低 78% 以上**"。

### 4.4 模型结构（待架构对话确认后展开）

`(C)` 方案下，MPC 的状态与输入大致为：

- **状态**：位置 `p`、速度 `v`、姿态四元数 `q`、机体角速率 `ω`
- **输入**：总推力 `T`、机体角速率期望 `ω_des`（三轴）
- **必须的模型参数**：质量、转动惯量、推力/力矩映射、阻力系数（可后加）
- **约束**：推力上下限（**按电池设，不按电机设**——见 2.4）、姿态倾角上限、角速率上限

`px4-mpc` 的 `multirotor_rate_model.py` 可直接作为起点（`Tf=1.0 s`、`N=50`、`NONLINEAR_LS`、`PARTIAL_CONDENSING_HPIPM`、`GAUSS_NEWTON`、`ERK` 4 stages/3 steps、`SQP_RTI`）。

---

## 5. 飞控参数与开源项目取舍

### 5.1 飞控参数：`z0.params`

**决策**：`z0` 的飞控初始参数直接采用 [`Huan-qun/Sim2real-UAV-Params-`](https://github.com/Huan-qun/Sim2real-UAV-Params-) 的 `S2R/assets/param/uav1.params`。

**依据**：那份文件来自**另一台用同一套配件搭建、且用动捕实飞验证过的无人机**（用户确认）。已归档为本仓库的 [`assets/z0.params`](assets/z0.params)（928 行 / 914 条参数 / PX4 1.14.3 dev；**比上游原件多 6 行出处说明**）。

> ⚠️ **命名上要分清两件事**：
> - **`uav1.params`** = 上游那个文件的原名。**"uav1" 是上游那台机器的代号**（上游是多机集群，刚体叫 `r11`/`r22`），不是 z0 的名字。
> - **`z0.params`** = 我们归档的这份。★ **2026-09-21 已与 z0 飞控真身逐条对账：914/915 条名字全对上，其中 21 条值不同、其余 893 条一致** ⟹
>   **飞控上跑的就是这份配置**（它不再是"上游参考值"）。
>
> ☠️ **但"一致"恰恰意味着下面那些可疑值真的在飞机上** —— 不是文件过时。逐条对账、21 条差异与待改项见
> [`.scratch/phase1/issues/15-fcu-param-reconciliation.md`](../.scratch/phase1/issues/15-fcu-param-reconciliation.md)。
> ☠️ **且不要整份烧 `z0.params`**：它含 6 条加速度计标定 + 12 条遥控器校准，而**飞控上那两组才是 z0 自己的**；
> 整份烧会把它们覆盖掉。要改就**只挑配置类参数**写。

**可直接复用的部分**（动捕飞行验证过的真实配置）：

| 参数 | 值 | 含义 |
|---|---|---|
| `EKF2_EV_CTRL` | **11** | 水平位置 + 垂直位置 + yaw（**不含 3D 速度**） |
| `EKF2_HGT_REF` | **3** | 高度参考 = Vision |
| `EKF2_MAG_TYPE` | **5** | 无磁罗盘 |
| `EKF2_EV_DELAY` | **50.0 ms** | ★ **与用户实测的 50 ms 完全一致** |
| `COM_ARM_WO_GPS` | **1** | 允许无 GPS 解锁（室内飞行必需） |
| `SYS_AUTOSTART` | **4001** | Generic Quad X |
| `MAV_1_CONFIG` / `MAV_1_MODE` | 102 (TELEM2) / 2 (Onboard) | 机载电脑链路 |
| `SER_TEL1_BAUD` / `SER_TEL2_BAUD` | 921600 / 921600 | |
| `PWM_MAIN_FUNC1..4` | 103, 102, 101, 104 | **电机序 `3-2-1-4`** |
| `THR_MDL_FAC` | 0.45 | 推力曲线因子 |
| `BAT_AVRG_CURRENT` | 15.0 | 与 VTC6 实测放电能力一致 |
| `BAT1_N_CELLS` / `MOT_POLE_COUNT` | 6 / 14 | 6S / 14 极 |

**必须在飞控上核实或修改的部分** —— 详见 [`reference/params-analysis.md`](reference/params-analysis.md)：

| 参数 | 问题 |
|---|---|
| `CA_ROTORx_PX/PY = 0.15` | 算得臂长 212 mm、轴距 **424 mm**，与 152 mm 机架**不符**，疑为 PX4 模板默认值 |
| `BAT1_R_INTERNAL = 0.005 Ω` | VTC6 6S1P 应有 **≈78 mΩ**，差 15 倍 |
| `EKF2_EV_POS_X/Y/Z = 0` | 动捕刚体 → IMU 的**外参未标定** |
| `EKF2_GPS_CTRL = 7` | GPS 融合仍开启，室内动捕飞行通常应设 **0** |
| `MPC_THR_HOVER = 0.5` | 疑为 PX4 默认值，非实测 |
| `COM_RC_IN_MODE = 3` | 含义需对着 PX4 1.14 文档核实（影响遥控器能否接管） |
| `UXRCE_DDS_CFG = 0` | uXRCE-DDS 当前**禁用** —— 走 `(C)` 方案时必须改 |

> ★ **2026-09-22 人裁定**：上表 8 条待处理项里**现在就改 3 条** —— `EKF2_GPS_CTRL = 0`、`SYS_HAS_GPS = 0`、
> `MPC_XY_VEL_MAX = 2.0`；其余分别留到**系统辨识 / 推力标定 / 尺子量 / 飞行阶段**。
> 逐条处置（含"谁改、怎么验"）见 [`reference/params-changes.md`](reference/params-changes.md) **§I**。
> ⏸ 那 3 条是「**已裁定、尚未写入**」（裁定当天飞控没上电）。§7.2 的 **O3 / O4 / O5 / O8** 已一并更新。

> ✅ **推荐动作已执行（2026-09-21）**：从 z0 的飞控导出全量参数（915 条）与 `z0.params` 逐条 diff ——
> **结论：914/915 条名字全对上；21 条值差异全部是设备标定类（加速度计 6 条 + 遥控器校准 12 条）与飞行计数器（3 条）**。
> ⟹ 上表那些"必须核实"的项**不是文件过时，而是飞控上真实存在的待改项**；且飞控**从无飞行记录**
> （`COM_FLIGHT_UUID = 0`、`LND_FLIGHT_T_* = 0`）。完整对账见
> [`.scratch/phase1/issues/15-fcu-param-reconciliation.md`](../.scratch/phase1/issues/15-fcu-param-reconciliation.md)。
> ```bash
> # 再次对账（★ 读参数别用 /mavros/param/get —— ROS2 版 mavros 没这个服务，见 ticket 15 §五）：
> source ~/mavros_ws/install/setup.bash && ros2 param dump /mavros/param > /tmp/fcu_params.yaml
> ```

### 5.2 开源项目取舍（详细版见 [`reference/open-source-survey.md`](reference/open-source-survey.md)）

| 项目 | 定位 | 本项目怎么用 |
|---|---|---|
| **`acados`** | 基于 CasADi 的嵌入式 MPC 求解器，生成 C 代码 | ✅ **核心依赖**。注意 Jetson 上的 `BLASFEO_TARGET=ARMV8A_ARM_CORTEX_A76`（Orin NX 是 A78AE，**acados 无 A78 目标**）、`HPIPM_TARGET=GENERIC`、☠️ Tera 渲染器缺失**不再报** `EOFError`（v0.6.0 起）—— ★ **但「必须装」这件事不变**；真身是三种落点，其中「在、但渲染不出东西」**完全静默**，见 [`control/acados-orin-build.md`](control/acados-orin-build.md) §4） |
| **`agilicious`** | ETH/UZH 敏捷飞行框架 | ⚠️ **公开仓库无任何源码**（只有 README + docs + bib），代码需向 UZH 申请。**只参考其文档架构 + 论文理论** |
| **`Fast-Drone-XI35`** | ★ **同硬件**的 ROS1 自主飞行栈（控制器是 PID） | ✅ **重点参考**。取 `thrust_calibrate.py`（推力标定）、`px4ctrl` 的 FSM 状态机、`motion_capture` 的时延对齐；**ego-planner 规划链留到第二阶段** |
| **`OmniNxt`** | **全向视觉感知**的常规四旋翼（★ 构型已更正，见下） | ❌ **零可复用代码**：仓库里**只有 4 个文件**（README + LICENSE + 论文 PDF + 一张图），16 MB 里 15 MB 是 PDF。**且 `OmniNxt-03.params` 根本不在仓库里**——README 指向 OneDrive 链接，需浏览器登录才能下。**真正的产出是它的硬件线**：`Nxt-FC` 与配套 PX4 fork |
| **`Sim2real-UAV-Params-`** | 同配件动捕实飞资料 | ✅ **已采用其 `uav1.params`**；另有 `mocap.md`（594 行动捕/PX4/ROS2 配置手册）与 `BOM.md` 值得对照 |
| **`MPC_uav`**（本项目仓库） | 论文资料库 | 📚 目前只有 README + 6 篇 PDF，**无代码**。论文见 [`assets/MPC_uav-README.md`](assets/MPC_uav-README.md) |
| `DISCOWER/px4-mpc` | acados MPC + PX4 (ROS2) | ✅ **`(C)` 方案的现成蓝本**（210★，发布 `vehicle_rates_setpoint`） |
| `tkkim-robot/px4_ugv_exp` | PX4 + ROS2 Humble + Vicon 闭环 | 🔍 动捕闭环最小可运行模板 |
| `SaxionMechatronics/mocap_px4_bridge` | OptiTrack → PX4 (ROS2) | 🔍 备用参考 |

---

## 6. 里程碑

| # | 里程碑 | 内容 | 验收标准 | 依赖 |
|---|---|---|---|---|
| **0** | **环境就绪** | 装 acados（含 Tera）、CasADi、uXRCE-DDS Agent；★ **`px4_msgs` 钉 `release/1.14` @ `ffb6e80e`** 并与真身 `4a0e65f2` 的 `msg/` 逐字节比对通过（共有的 184 个文件 blob 全同）。☠️ 原写「**从 fork `5ba2e91f` 的 `msg/` 生成**」—— **作废**（真身不跑 fork，见 §7.1 与 [`reference/refs-manifest.md`](reference/refs-manifest.md) §2.6 执行版） | MPC 求解器能编译并跑通官方 `quadrotor_nav` 示例；**且 `px4_msgs` 与固件消息定义逐字节一致** | — |
| **1** | **z0 动捕定点** | 移植旧机的动捕链路到 z0：VRPN 驱动 → `/mavros/vision_pose/pose` → EKF2；重标 `EKF2_EV_POS_*` 外参；重调 `EKF2_EVP_NOISE` | z0 能用动捕自主定点悬停 | 动捕场地 + 网段打通（见第 7 节） |
| **2** | **系统辨识** | 用 `thrust_calibrate.py` 测推力曲线；辨识 kt/kq/惯量；测推力在不同电压下的漂移 | 得到完整的建模参数表（替换 2.3 里的 ❌ 项） | 里程碑 1 |
| **3** | **仿真闭环** | 先在 Python/CasADi 对自写模型验 MPC 数学；再进 PX4 SITL 验接口与闭环 | SITL 里轨迹跟踪不发散 | 里程碑 0 |
| **4** | **实机 `(C)` 闭环** | MPC → `VehicleRatesSetpoint` → PX4 rate loop，动捕定位 | 实机完成给定轨迹跟踪 | 里程碑 1、2、3 |
| **5** | **INDI 内环** | 用 INDI 替代 PX4 rate loop（步骤 2） | 跟踪精度优于步骤 1 | 里程碑 4 |
| **6** | **基线对比与论文** | 与 PID 基线（`px4ctrl`）对比，出数据 | 论文素材 | 里程碑 4/5 |
| **7**（可选） | **(D) 电机直发** | INDI 直接输出电机指令，绕过 `control_allocator` | — | 里程碑 5 |

**顺序上的一个判断**：**里程碑 1（z0 动捕定点）应该在 MPC 之前完成。** 理由：MPC 的地基是状态估计。动捕链路没在 z0 上跑通，MPC 调参时会无法区分"是 MPC 的问题"还是"是定位的问题"。

---

## 7. 风险与开放问题

### 7.1 已识别的风险

| 风险 | 影响 | 缓解 |
|---|---|---|
| **电池功率瓶颈**（见 2.4） | T/W 实际只有 1.60–2.54，敏捷飞行受限 | MPC 推力约束按电池设；考虑换更高 C 的电池 |
| **电压 sag** | 同样油门给出不同推力，sim-to-real gap | 辨识推力-电压关系；或模型里显式建电压状态 |
| ~~**★★ 动捕时间戳抖动 10–100 ms**~~ | ✅ **【已撤销】该风险不存在** —— 抖动实际只有 ~2–5 ms，且 `stamp = now()` 是刻意设计（动捕主机与 Jetson 时钟差 ~4 秒，改成 `msg_time` 反而会毁掉 EKF2）。真实最坏延迟 ≈ 55 ms 而非 150 ms | **不需要任何缓解措施。** 详见 [`reference/mocap-setup.md`](reference/mocap-setup.md) **6.2** |
| ~~**飞控固件来源不明**~~ | ✅ **【已结案，2026-09-21 第一手终审】** 真身是 **`Minderring/PX4-Autopilot` @ `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`**（2024-11-18，commit message `add micoair743-aio to PX4-Autopilot v1.14.3`）= **v1.14.3 系的 MicoAir 厂商构建**。判据是**飞控自报**的 `flight_custom_version` 8 字节 `00 00 00 33 f2 65 0e 4a` ⟹ hash `4a0e65f233`（由 sha 正向推出的字节与实测**逐字相同**），且 `flight_sw_version = 0x010e0300` ⟹ `1.14.3`；其 `msg/` 与**上游 v1.14.3 逐字节相同**。☠️ **09-20 那一版「真身 = `Peize-Liu/PX4-Autopilot @ 5ba2e91f`（经 Nxt-FC 的 submodule 指认）」作废** —— 那条 submodule 线索**不等于飞控里烧的那一份**；`5ba2e91f` 是 **v1.15 世代**的 fork，**与 z0 无关** | ✅ **该"以 QGC 读数为准"的终审已做**（MAVLink `AUTOPILOT_VERSION`，2026-09-21）—— 见 [`reference/refs-manifest.md`](reference/refs-manifest.md) §2.5(c)(d)。⚠️ `z0.params` 头的 `Git Revision: 4a0e65f233000000` 与它**逐字对得上**（同一个 8 字节值的两种写法），该参数文件是**旁证**而非 z0 的读数。⚠️ 遗留张力：板名 **NxtPX4v2** 与真身树里的板级目标（只有 `micoair/{h743,h743-v2,h743-aio}`、**无 `hkust/`**）对不上 ⟹ **Phase 4 重编该编哪个目标未定** |
| **电池电流采样读错通道（`boards/hkust/nxt-dual` 的 ADC 缺陷）—— ★ 实测：z0 未中招** | `nxt-dual` 的 `board_config.h` 里 `ADC_BATTERY_CURRENT_CHANNEL = ADC12_CH(5)` 是错的（引脚表写 `/* PC5 */ GPIO_ADC12_INP8`，即电流接在 **PC5 = 通道 8**，宏却让电流去读**通道 5** —— 注释与通道号自相矛盾）。版本边界第一手确认：**v1.15.0 仍是 `CH(5)`，v1.16.0 才改成 `CH(8)`**（电压通道全程 `CH(4)` 未动）。⚠️ 修复**只针对 `nxt-dual`**：`nxt-v1` 的电流也是 `CH(5)`，但其**电压通道是 `CH(9)` 而非 `nxt-dual` 的 `CH(4)`** —— 两板不可互相照抄。☠️ **但这块板不在 z0 的固件里**：真身（`4a0e65f2`）树中**没有 `hkust/`**，它的 `boards/micoair/h743-aio` 写的是 `/* PC1 */ ADC1_CH(11)`，**与引脚相符、无此缺陷** | ✅ **2026-09-21 上电实测判为"未中招"**（未解锁时电流读数 ≈0 且非恒定怪值；读数与判据见 [`reference/params-changes.md`](reference/params-changes.md) **§A4**）。⟹ **不是运气，是这条缺陷不在这块固件里**。⚠️ **后果**：Phase 4 原计划的「重编重刷 `pr_fix_hkust_nxt_dual`（`CH(5)`→`CH(8)`）」**对 z0 的固件不适用，修法要重定**。❌ 通用结论仍成立：采样通道来自**编译期宏** `ADC_CHANNELS`（`ADC.hpp:58-67`，有 `#ifndef`/`#error` 守卫），**参数无法绕过**，只能重编重刷。⚠️ 逐项证据见 [`reference/params-analysis.md`](reference/params-analysis.md) **§3.5.1** |
| **Jetson 内存 7.4 GB** | 仿真时 Gazebo + SITL + ROS2 可能吃紧 | 仿真时关闭不必要组件；真机飞行无此问题 |
| **PX4 版本迁移** | 旧机 1.13.3 → z0 1.14.x，EKF2 参数名全变（`EKF2_AID_MASK` v1.15 已删除、`EKF2_HGT_MODE` → `EKF2_HGT_REF`） | 用户已完成过一次该迁移（笔记里有成对对照），成本可控 |
| **`px4_msgs` 版本不匹配** ★★ **已经踩到，不是假设** | v1.15 及以前**静默失败**（topic 名或字段错位不报错；`msg/versioned/` 机制 v1.16 才有）→ **DDS 话题名一致但线格式不符 = 无报错的静默失败**。**2026-09-21 定案的基准**：与真身 `4a0e65f2` 共用 184 个 `msg/` 文件上，`px4_msgs` `release/1.14` @ `ffb6e80e` **blob 全同**（唯一集合差异 `msg/CMakeLists.txt`，那是 PX4 自己的构建文件）。☠️ **原判「`release/1.14` 与 z0 固件不兼容（184 里 28 个字段不同）」作废** —— 那是拿 **fork `5ba2e91f`** 比出来的 | ★ **`px4_msgs` = `release/1.14` @ `ffb6e80e`**（**用户已签字**）；**不要从任何 fork 生成**。核对工具用 `tools/px4-sitl/check_msg_compat.py`（跑法见 [`control/px4-sitl-baseline.md`](control/px4-sitl-baseline.md) §4）。☠️ ~~官方脚本 `./scripts/check-message-compatibility.py -v <px4_msgs> <PX4-Autopilot>`~~ —— **这个路径是错的**：该脚本**不在 PX4-Autopilot 里**，真身在 `auterion/px4-ros2-interface-lib`，**直接跑会给假 FAILED**（见 [`control/px4-sitl-baseline.md`](control/px4-sitl-baseline.md) §2.3）。详见 [`reference/refs-manifest.md`](reference/refs-manifest.md) §2.5(d) 与 §2.6 执行版 |
| ~~★★ **固件版本标签错误（"1.14.x"）**~~ | ✅ **【已撤销】该风险不存在** —— 「z0 是 PX4 1.14.x」**这个标签一直是对的**（真身 `4a0e65f2` 属 **v1.14.3 系**，飞控自报 `flight_sw_version = 1.14.3`）。☠️ **2026-09-20 那条「`5ba2e91f` 是 v1.15 世代 ⟹ 1.14 标签是错的」判定本身才是错的** —— 它拿 **fork** 当了 z0 的固件，而**飞控不跑 fork**（`5ba2e91f` 的 `OffboardControlMode` 带 `thrust_and_torque` 是事实，但那描述的是 fork）。⟹ **因这条误判而"打折"的下游结论可以恢复**：`msg/` 已实测与上游 v1.14.3 逐字节相同（185/185），凡依赖"1.14 消息布局/默认值"的判断**按上游 v1.14.3 走即可** | **不需要缓解措施。** 详见 [`reference/refs-manifest.md`](reference/refs-manifest.md) §2.5(d) / §2.6 执行版，以及 [`reference/params-changes.md`](reference/params-changes.md) §H |
| **noetic 容器镜像版本不匹配** | 镜像 `l4t-r35.4.1`（JetPack 5）vs 宿主 `R36.5.0`（JetPack 6） | 使用前实测验证；优先用 Humble 原生环境 |
| **acados 在 Jetson 上的编译** | 无官方 Jetson 文档；☠️ `t_renderer` 缺失**不再报** `EOFError`（v0.6.0 起）—— ★ **但「必须装」这件事不变**；真身是三种落点，其中「在、但渲染不出东西」**完全静默**，见 [`control/acados-orin-build.md`](control/acados-orin-build.md) §4） | 见 5.2 的要点；`BLASFEO_TARGET` 必须手动指定 |

### 7.2 开放问题（待确认）

| # | 问题 | 为什么要紧 |
|---|---|---|
| **O1** | **螺旋桨转向是 props-in 还是 props-out？** | 直接决定 MPC 模型里力矩 τ 的符号 |
| **O2** | ~~**动捕怎么接到 Jetson？**~~ ✅ **已收敛**：VRPN 就是 UDP over IP，**把 Jetson 连到动捕主机所在的同一个 WiFi 即可，不需要额外网卡**。实测：z0 连 `SUSTech-802.1x`（`10.29.12.59/14`）时，到 `10.1.1.198` 的路由经校园网网关（`10.31.255.254`）绕出，ping 与 TCP 3883 **均不通**；换到动捕所在网络后即通 | **剩一个动作**：连对 WiFi。不是硬件问题 |
| **O3** | **`CA_ROTORx_PX/PY` 实际值是多少？** 🔶 **一半已答（2026-09-21 对账）**：**飞控真身读到的就是 `±0.15`** —— 所以"文件是不是过时"这个问题**结了**。但**"实测这台飞机的真实几何"仍未做**。★ **2026-09-22 裁定：`CA_ROTOR*` 不改**（分配器按几何自身范数归一化，均匀缩放会被完全约掉 ⟹ 改了没效果），真正要它的是 **(D) 方案**与力矩饱和估算 | 控制分配的几何基础 |
| **O4** | `EKF2_EV_POS_X/Y/Z` 外参（动捕刚体 → IMU 偏移） | ⬜ **仍未做**。★ **2026-09-22 裁定：留到"尺子量"**（`hardware.md` §6）—— 量完即写。外参错会导致位置估计有常值偏移 |
| **O5** | `MPC_THR_HOVER` 是默认值还是实测值？ | ★ **一半已答（2026-09-21 对账）**：真身就是 `0.5`，**且 `0.5` 恰好是 PX4 出厂默认值** ⟹ 看不出来被改过。★ **2026-09-22 裁定：留到推力标定**（里程碑 2，与 `THR_MDL_FAC` 一并定）；它决定 **MPC（模型预测控制）** 的线性化工作点对不对 |
| **O6** | ~~**通信中间件最终选"混合"还是"全 mavros"？**~~ ✅ **已定（分阶段）**：**Phase 1 全走 mavros2**（`/mavros/vision_pose/pose` + `/mavros/setpoint_raw/local`），**完全不碰 uXRCE-DDS**（`UXRCE_DDS_CFG` 都不用改）；**uXRCE-DDS 推迟到 Phase 2** 引入 MPC 时再上。理由：每个阶段只引入一个新变量 —— Phase 1 验动捕链路（中间件零新增），Phase 2 才引入新中间件（此时动捕已验证）。同时避开 `px4_msgs` 版本不匹配的静默失败陷阱 | 见 3.3 与本项目 Phase 1 决策记录 |
| **O7** | 里程碑 5（INDI）是第一版就做，还是留到第二阶段？ | 影响模型复杂度 |
| **O8** | `EKF2_GPS_CTRL = 7` 是否需要改成 0？ | ✅ **已裁定（2026-09-22）：改** —— `EKF2_GPS_CTRL = 0` + `SYS_HAS_GPS = 0`（配套）。**状态：已裁定、待写入**（裁定当天飞控没上电）。命令见 [`reference/params-changes.md`](reference/params-changes.md) **§I.1** |

---

## 8. 附录

### 8.1 调查方法记录

本次文档的事实来源，按可靠性排序：

1. **z0 机载电脑实测**（环境侦察、全盘搜索参数文件）—— 最高可靠性
2. **用户笔记仓库 `/home/chen/weisama-log`** —— 一手记录
3. **SSH 直取的两个 GitHub 仓库**（`weisama/MPC_uav`、`Huan-qun/Sim2real-UAV-Params-`）—— 一手资料
4. **PX4 官方文档**（Offboard、External Position Estimation、参数表）
5. **WebSearch 摘要** —— ⚠️ **本次调查中 WebFetch 被网络策略全线拦截**（github.com、oddityrc.com、micoair.cn 等均返回 "Unable to verify if domain is safe to fetch"），所有网页数据来自搜索摘要，**可靠性较低**。硬件规格中标注"需实测"的项目主要受此影响。

### 8.2 ★ SSH 访问 private 仓库的经验教训

**用 HTTPS / GitHub API 访问 private 仓库会返回 404，而不是 403。**

本次调查中，一个代理用 `api.github.com/repos/weisama/MPC_uav` 探测得到 **404**，据此得出"仓库不存在"的**错误结论**。换用 SSH 后立刻成功。

- GitHub **故意对未授权的 private 仓库返回 404**，以避免泄露仓库是否存在
- `api.github.com/users/weisama` 会显示 `public_repos: 1`，进一步误导
- GitHub **全站搜索也不索引 private 仓库**（搜 `Sim2real-UAV-Params` 返回 `total_count: 0`）

→ **结论：有 SSH key 时，判断仓库能否访问一律用 `git ls-remote git@github.com:<owner>/<repo>.git`，不要用 HTTPS 或 API。**

完整验证命令见 [`LOCAL-NOTES.md`](LOCAL-NOTES.md) 第 3 节。

### 8.3 文件索引

| 路径 | 内容 |
|---|---|
| [`../CONTEXT.md`](../CONTEXT.md) | **术语表（先读）**，含 `MPC` 歧义澄清 |
| [`control/mpc-roadmap.md`](control/mpc-roadmap.md) | MPC 路线图详细版（**资料汇总**，含 D1–D10 待决清单） |
| [`control/phase2-implementation.md`](control/phase2-implementation.md) | ★★ **Phase 2 实施清单（MPC 接管 PID）** —— D1–D10 决议、`px4-mpc` 第一手解读、S1–S8 分步步骤、待实测参数——**做 Phase 2 时先读它** |
| [`control/sign-map.md`](control/sign-map.md) | ★★ **符号映射表的唯一权威** —— ENU/FLU ↔ PX4 FRD 的推导、与参考实现的逐条对照、推力项的量纲与 `T_max` 口径；**代码一律回链它、不复写数值** |
| [`reference/hardware.md`](reference/hardware.md) | 硬件规格调查原始记录 |
| [`reference/params-analysis.md`](reference/params-analysis.md) | `z0.params` 逐项分析 |
| [`reference/px4-interface.md`](reference/px4-interface.md) | 动捕 / PX4 / Offboard 接口事实 |
| [`reference/mocap-setup.md`](reference/mocap-setup.md) | ★ **动捕链路搭建手册**（操作规程 + `vrpn_relative_pose` 源码 + z0 适配清单） |
| [`reference/open-source-survey.md`](reference/open-source-survey.md) | 开源项目包级取舍（**全部第一手 `git clone` 核实**；§9 = Nxt-FC / 固件线） |
| [`reference/code-inventory.md`](reference/code-inventory.md) | ★★ **逐文件盘点（✅/🔶/❌ + 理由）+ 落到哪个包**（代码取舍的权威来源） |
| [`control/phase1-implementation.md`](control/phase1-implementation.md) | ★★ **Phase 1 实施清单**（包与文件、话题接口、坐标系、FSM、轨迹与判据、参数核对、验证顺序、避坑清单、4 件物理操作）——**写 Phase 1 代码前先读它** |
| [`control/bench-runbook.md`](control/bench-runbook.md) | ★★ **拆桨台架清单**（Phase 1 里**必须由人做**的那一薄层：前置参数、逐项步骤、判据、记录表、这些框各关哪张票）——**做台架时照它走**（ticket 08/09 都已落地；§7 的顺序表里涉及台架的那两步仍以本清单为准） |
| [`assets/z0.params`](assets/z0.params) | **z0 的飞控参数文件**（928 行 / 914 条参数） |
| [`assets/papers/`](assets/papers/) | 6 篇参考文献 PDF + [中文索引](assets/papers/README.md)（命名规则、研究内容、用处） |
| [`LOCAL-NOTES.md`](LOCAL-NOTES.md) | ⚠️ **含本机凭据（系统密码等）。作者 2026-09-18 决定随 `docs/` 入库**（private 仓库）。见该文件头部的风险须知 |
