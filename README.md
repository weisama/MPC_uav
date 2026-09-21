# MPC_uav — z0 无人机 MPC 控制

> **本仓库是 z0（OddityRC XI35 Pro 152 mm 涵道四旋翼）自主飞行栈的工作副本。**
> 飞控 MicoAir **NxtPX4v2**（固件 = `Peize-Liu/PX4-Autopilot` @ `5ba2e91f`，**v1.15 世代** —— 见下方约束表）
> \+ 机载 Jetson Orin NX 8GB + Nokov 动捕（VRPN）。
> 最终目标是 **MPC 接管飞控内置 PID**；**分三阶段落地**，见下。

---

## 三阶段路线

| 阶段 | 做什么 | 控制接口 | 状态 |
|---|---|---|---|
| **Phase 1** | **offboard 位置定点飞行** —— 用动捕让 z0 稳定悬停并跟踪简单轨迹，**PX4 跑全部内环 PID** | **(A) 级**：`/mavros/setpoint_raw/local`（位置 + yaw） | ⬜ 未开始 · [`spec`](.scratch/phase1/spec.md) `ready-for-agent` |
| **Phase 2** | **MPC 接管 PID —— SITL 验证**：MPC 输出角速率 + 推力，PX4 只跑角速率环；被控对象**从 fork 构建的 z0 同构机**（**不是 `gz_x500`**） | **(C) 级**：`/fmu/in/vehicle_rates_setpoint` | ⬜ 未开始 · [`spec`](.scratch/phase2/spec.md) `ready-for-agent` |
| **Phase 3** | **MPC 上实机**：四项前置测量 + 一次固件修复（ADC 一行），然后拆桨 → 系绳 → 自由飞四级阶梯 | 同 Phase 2（**(C) 级**） | ⬜ 未开始 · [`spec`](.scratch/phase3/spec.md) `ready-for-human` |

**指导思想：每个阶段只引入一个新变量。** Phase 1 全走 mavros2（**完全不碰 uXRCE-DDS**），把"动捕链路 + 状态机 + offboard"先跑通；Phase 2 再加 MPC + acados + uXRCE-DDS，**但只在 SITL 里验证，不碰飞机**；Phase 3 才上实机，且**先在离地之前把每个能摔机的未知量量出来**。

---

## 目录

```
├── CONTEXT.md                        术语表（含 MPC 歧义澄清）
├── .scratch/                         ★ 本地 issue tracker（随 git 入库，不是临时目录）
│   ├── phase1/spec.md                Phase 1 spec —— offboard 位置定点飞行
│   ├── phase2/spec.md                Phase 2 spec —— MPC 接管 PID：SITL 验证
│   └── phase3/spec.md                Phase 3 spec —— MPC 实机飞行（含固件一行修复与前置测量）
├── docs/    资料与文档
│   ├── PROJECT.md                    ★ 总纲（先读它）
│   ├── LOCAL-NOTES.md                ⚠️ 含本机凭据，作者决定随 docs/ 入库 —— 见文件头部须知
│   ├── agents/                       skill 配置（issue tracker / triage / 域文档规则）
│   ├── control/
│   │   ├── phase1-implementation.md  ★★ Phase 1 实施清单
│   │   ├── phase2-implementation.md  ★★ Phase 2/3 实施清单（Phase 3 spec 引用它）
│   │   └── mpc-roadmap.md            Phase 2 资料汇总（含 D1–D10）
│   ├── reference/
│   │   ├── code-inventory.md         ★★ 代码盘点与取舍
│   │   ├── open-source-survey.md     开源项目调查（第一手 clone 核实）
│   │   ├── mocap-setup.md            ★ 动捕链路搭建手册
│   │   ├── px4-interface.md          动捕 / PX4 / Offboard 接口事实
│   │   ├── params-analysis.md        z0.params 逐项分析（取证正文）
│   │   ├── params-changes.md         ★★ **参数改动表**（现值/建议值/理由/是否阻塞）→ ticket 化时引用
│   │   ├── xi35-fsm.md               ★★ **XI35 状态机转换表**（17 行）→ ticket 化时引用
│   │   ├── refs-manifest.md          ★★ 参考仓库清单 + 钉住的 commit + §2.5 z0 固件版本判定
│   │   └── hardware.md               硬件规格
│   └── assets/
│       ├── z0.params                 z0 飞控参数（928 行，待上机核实）
│       └── papers/                   6 篇参考文献 + 中文索引
├── tools/prereq-evidence/            ★ 前置条件取证脚本（只读）—— 服务 ticket 01
└── src/                              ★ ROS2 工作区（**colcon 根 = 仓库根**，构建产物已被 .gitignore 挡住）
    ├── vrpn_listener/                ✅ 动捕链路，vendored 原样 —— 读它的 VENDORED.md，**别读它的 README.md**
    └── mocap/                        ✅ 相对位姿（两阶段原点 + 仅归零 yaw）→ `/mavros/vision_pose/pose` → EKF2
```

> **构建**（`colcon build` 在**仓库根**跑，不是 `src/`）：
> ```bash
> source /opt/ros/humble/setup.bash && cd ~/ros_ws && colcon build
> ```
> 依赖与运行方式见 [`src/vrpn_listener/VENDORED.md`](src/vrpn_listener/VENDORED.md) §5；
> `mocap` 与它的测试见 [`src/mocap/README.md`](src/mocap/README.md)（`colcon test --packages-select mocap`）。

**先读哪个**：`docs/PROJECT.md` → `docs/reference/code-inventory.md`（代码取舍）→ `docs/control/phase1-implementation.md`（动手写代码前必读）→ `CONTEXT.md`（查术语）。
**要动手做哪一阶段**：先读对应的 `.scratch/phase{1,2,3}/spec.md` —— 它定义该阶段的范围与验收判据。

> **三份 spec 的 `Status:`（2026-09-20 已分诊）**：Phase 1 与 Phase 2 → **`ready-for-agent`**
> （代码 + SITL 可 AFK 推进）；Phase 3 → **`ready-for-human`**（量电机位置、刷固件、系绳试飞，
> AFK agent 做不了）。Q35=a 的暂时退回**已解除**。

### 📁 参考仓库在**仓库外**

所有上游参考仓库（XI35 / OmniNxt / agilicious / Fast-Drone-250 / px4-mpc / px4_msgs /
Sim2real-UAV-Params- / Nxt-FC / z0 固件真身 / PX4 上游对照 tag）都就地放在 **`~/refs/`**，
**不进本仓库**（Q10 决策），按文档记录的 commit 钉住。**完整清单、钉住的 commit、逐项核对结果见
[`docs/reference/refs-manifest.md`](docs/reference/refs-manifest.md)。**

> ⚠️ **不要再依赖 `/tmp`** —— 2026-09-20 那一轮里 `/tmp/repo-survey/` 等**全部被清空过**，
> 导致当时无法复核任何第一手结论。`~/refs/` 就是为此建立的。

---

## 关键约束（摘要，每条都有第一手证据）

| 约束 | 后果 |
|---|---|
| ⚠️ **z0 不在 PX4 官方支持矩阵内** —— `hkust_nxt-dual` 板级目标只在 **PX4 ≥ 1.15.0** 存在（v1.14.3 无 `boards/hkust` ✅ 已核实）。★ **固件真身 = `Peize-Liu/PX4-Autopilot`** @ `5ba2e91f`（2024-07-02），经 `HKUST-Aerial-Robotics/Nxt-FC` 的 submodule 指认。★★ **2026-09-20 更正：它其实是「v1.15 世代」，不是 1.14** —— 见下一行 | **不刷固件、不追上游**（Q12 不变）。但⚠️ **"1.14" 这个标签是错的**：`z0.params` 头声明的 `Version: 1.14.3 dev` / `Git Revision 4a0e65f2` 来自**上游参考机的参数文件**，未必描述 z0 → **首飞前必须在 QGC 记录飞控实际版本号**，以它为准 |
| ★★ **固件是 v1.15 世代（本轮第一手判定，推翻"1.14.x"）** —— `5ba2e91f` 的 `msg/OffboardControlMode.msg` 有 `bool thrust_and_torque` + `bool direct_actuator`；**上游 v1.15.0 同款，v1.14.3 是 `bool actuator`**。另有 `VehicleLocalPosition.heading_var`、`VehicleCommand.source_component` 为 `uint16`、存在 `srv/` 目录、**无** `msg/versioned/`（该机制 v1.16 才有） | 这条**直接推翻**了"按 1.14 选 `px4_msgs`"的方案 —— 见下一行 |
| ⚠️ **电池电流采样读错通道（★ 已第一手确认为"中招"）** —— `nxt-dual` 的 `ADC12_CH(5)` 是错的。**已在 z0 固件真身里逐字读到该错值**，并找到硬件层根因：引脚表写 `/* PC5 */ GPIO_ADC12_INP8`（电流接在 PC5 = 通道 8），宏却读**通道 5**。**v1.15.0 仍带 bug，v1.16.0 才改成 `ADC12_CH(8)`** | 上电后**在 QGC 核对电流读数**（恒为 0 或怪值 = 中招）；中招则电流/电量保护全是假的。❌ **参数无法绕过（已第一手排除）**：采样通道来自**编译期宏** `ADC_CHANNELS` → **只能重编重刷固件**。★ **2026-09-20：修复代价已查明 = 一行** —— fork 自带 `pr_fix_hkust_nxt_dual` @ `8161c4cd`，相对 `main` 仅 `CH(5)`→`CH(8)` 一行（**未合并的提案分支**）。详见 `docs/reference/refs-manifest.md` §2.7(b) |
| ⚠️ **动捕时间戳 `stamp = now()` 是刻意设计，不要改** | 动捕主机与 Jetson 有 **~4 秒**时钟差；改成 `msg_time` 会毁掉 EKF2 融合 |
| ⚠️ **`TrajectorySetpoint` 没有 `thrust` 字段** | 要发姿态+推力得用 `VehicleAttitudeSetpoint` |
| ⚠️ **推力是 `[-1, 1]` 归一化，不是 `0–1`、不是牛顿** | 数值约定错了，MPC 算得再对飞机也不会飞 |
| ⚠️ **`px4_msgs` 版本必须与固件严格匹配** | v1.15 及以前**不匹配会静默失败**（不报错，只是消息没人收） |
| ★★ **Q24 的验收步骤已执行，结论：`px4_msgs` `release/1.14` @ `ffb6e80e` 与 z0 固件不兼容，不可用** —— 184 个共用消息里 **28 个字段布局不同**，含 11 个关键消息中的 6 个。`VehicleRatesSetpoint` / `VehicleAttitudeSetpoint` / `VehicleOdometry` / `ActuatorMotors` 恰好**逐字节相同**（Phase 2/3 的 `(C)` 级幸免），但 `OffboardControlMode`（`actuator` → `thrust_and_torque`）、`VehicleCommand`、`VehicleStatus`、`VehicleLocalPosition`、`VehicleControlMode` **均为破坏性差异** | ★ **已定（Q31=a）：版本基准统一到 fork `5ba2e91f`** —— **`px4_msgs` 从 fork 的 `msg/` + `srv/` 生成**；`release/1.14` 否决，`release/1.15` 也不必试。**SITL 被控对象同样从 fork 构建**（gazebo-classic 照旧可用，fork 支持两套仿真）。详见 `docs/reference/refs-manifest.md` §2.5–§2.6 |

> ⚠️ **待清扫（Q36=b：已定交给后续的 ticket 化步骤一并处理）**：`docs/` 里仍残留**约 24 处**把 z0 说成
> "PX4 1.14.x"、或引用 `release/1.14` 的陈述（分布在 `PROJECT.md`、`phase1-implementation.md`、
> `px4-interface.md`、`code-inventory.md`、`open-source-survey.md` 等）。其中**一部分是合法的**
> ——指上游 v1.14.3 作版本对照——所以需要**逐处甄别**，不能一律替换。
> **权威判定与分支对照表见 [`refs-manifest.md`](docs/reference/refs-manifest.md) §2.5 / §2.7。**

---

## 许可证与致谢

- 本仓库引用的第三方项目各自的许可证见 `docs/reference/open-source-survey.md`。
  ★ **注意**：`uzh-rpg/agilicious` 的许可证**禁止**再分发其代码，本项目**只引其论文与设计思想，不引用任何代码**。
- 参考论文索引见 [`docs/assets/papers/README.md`](docs/assets/papers/README.md)。
