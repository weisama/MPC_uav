# MPC_uav — z0 无人机 MPC 控制

> **本仓库是 z0（OddityRC XI35 Pro 152 mm 涵道四旋翼）自主飞行栈的工作副本。**
> 飞控 MicoAir **NxtPX4v2**（固件真身 = `Minderring/PX4-Autopilot` @ `4a0e65f2`，**v1.14.3 系**的
> MicoAir 厂商构建 —— 见下方约束表。⚠️ fork `Peize-Liu/PX4-Autopilot @ 5ba2e91f` **与 z0 无关**）
> \+ 机载 Jetson Orin NX 8GB + Nokov 动捕（VRPN）。
> 最终目标是 **MPC 接管飞控内置 PID**；**分四阶段落地**，见下。

---

## 版本信息

| | |
|---|---|
| **当前版本** | **v2.0** —— **Phase 2 的代码 + 真机录制产物**的快照（2026-10-08）。⚠️ **这个标签不表示"Phase 2 收口"**：MPC（模型预测控制）在 SITL 里的闭环已经跑起来，但 seam 3 的判据**没有跑满**，未决的那些事以 [`.scratch/phase2/`](.scratch/phase2/spec.md) 的 `Status:` 行为准，别拿本行当进度表 |
| **日期** | 2026-10-08 |
| **分支 / 标签** | `main`（开发主线）· **`v2.0`（标签）= 本版快照**（代码 + `bag/` 的六场景录制产物一起）· **`v1.0`（分支）= Phase 1 收口**那一版的快照，指着 `5ff9709`（2026-09-28）—— ⚠️ 它是**分支**、不是标签，别改成标签：同一个名字既当分支又当标签会让 `git rev-parse v1.0` 歧义 |
| **本版对应提交** | ☠️ **本页无法自引自己的 hash** —— 用 `git rev-parse v2.0`（或 `git rev-parse main`）取 |
| **飞控固件** | `Minderring/PX4-Autopilot` @ `4a0e65f2` —— v1.14.3 系的 MicoAir 厂商构建，**已在机上、不刷不改** |
| **`px4_msgs`** | `release/1.14` @ `ffb6e80`（仓库外 `~/refs/px4_msgs`；**Phase 2 起用** —— ⚠️ 现已是**构建期依赖**：`nmpc` 与 `bench` 的 `package.xml` 里各有一条 `<depend>`） |
| **mavros** | **2.15.1 @ `22ae5b7`** —— 源码构建在 `~/mavros_ws`，**不在本仓库**（Phase 1 全靠它） |
| **ROS 2** | **Humble** |
| **运行环境** | Ubuntu 22.04.5 LTS · 内核 5.15.185-tegra · aarch64（Jetson Orin NX 8GB） |
| **阶段状态** | Phase 1 🟨 **收口**（真机自由飞成 2026-09-28；★ 残余项是**裁定不再补做 / 留待台架日**的那几条，见下） · Phase 2 🟨 **进行中**（见下） · Phase 3 🟨 **进行中**（整理 + 收尾，见下） · Phase 4 ⬜ 未开始 —— ★ **这几个词是粗判、不是读数**：真实进度只认各 spec 与各票的 `Status:` 行 |

★ 上表每一行的证据落点：固件与 `px4_msgs` 见 [`refs-manifest.md`](docs/reference/refs-manifest.md)
§2.5/§2.6，mavros 版本见 [`src/control/README.md`](src/control/README.md)。

## 项目基础信息

| | |
|---|---|
| **项目** | **MPC_uav** —— z0 无人机自主飞行栈的工作副本（★ **文档优先**：本仓库就是现场那台 Jetson 上的工作树） |
| **机体** | **z0** = OddityRC XI35 Pro，**152 mm 涵道四旋翼** |
| **飞控** | MicoAir **NxtPX4v2**（PX4 1.14.x，⚠️ 板名 ≠ 固件目标名，见"关键约束"） |
| **机载电脑** | Jetson Orin NX 8GB —— ★ **本仓库就在它上面**（`~/ros_ws`），文档里的"现场"指这台 |
| **定位** | **Nokov 动捕** → VRPN → `/mavros/vision_pose/pose` → EKF2（**室内飞行，不用 GPS**） |
| **中间件** | ROS 2 Humble + mavros2 —— Phase 1 **完全不碰 uXRCE-DDS**，Phase 2 才引入 |
| **工作区形态** | ROS 2 工作区，★ **colcon 根 = 仓库根**（不是 `src/`）；6 个包见下面目录树 |
| **目标** | 让 **MPC 接管飞控内置 PID**；分四阶段，**每阶段只引入一个新变量** |
| **issue tracker** | **`.scratch/<phase>/issues/*.md`** —— 本地 markdown，**随 git 入库**，**不用 GitHub Issues**（规则见 [`docs/agents/issue-tracker.md`](docs/agents/issue-tracker.md)） |
| **术语表** | [`CONTEXT.md`](CONTEXT.md) —— 查词先看它（含"MPC"这个词的歧义澄清） |
| **文档优先** | 结论大多带**第一手证据**（源码路径 + 行号 + commit）⟹ **改代码前先确认文档里的结论有没有被推翻**，不要凭记忆。教训见 [`open-source-survey.md`](docs/reference/open-source-survey.md) §0 |

**怎么跑起来**：构建命令见下面[目录](#目录)末尾的引用块；起链与现场操作看
[`run/README.md`](run/README.md)（菜单）与 [`operation_manual.md`](operation_manual.md)（★ 真机飞之前必读）。
**怎么跑测试**：`python3 -m pytest run/tests/test_operation_manual.py -q`（文档守卫，**逐文件跑** ——
★ **没 source 时 `pytest run/tests` 整个目录一条都收不到**，这不是"全绿"；
每个文件的用例数与跑法见 [`run/README.md`](run/README.md) 的「测试」一节 —— ★ **本页不抄用例数**，
抄了就会过期）；ROS 侧的
`colcon test` 用法与已知抖动见 [`src/bench/README.md`](src/bench/README.md)。

---

## 四阶段路线

| 阶段 | 做什么 | 控制接口 | 状态 |
|---|---|---|---|
| **Phase 1** | **offboard 位置定点飞行** —— 用动捕让 z0 稳定悬停并跟踪简单轨迹，**PX4 跑全部内环 PID** | **(A) 级**：`/mavros/setpoint_raw/local`（位置 + yaw） | 🟨 **代码与台架测试已就绪** + **2026-09-28 真机自由飞一次成**（定点定高 + 各按键 + 三条轨迹，**目测一切正常、数值未量** ⟹ ticket 09 收 `done`）· **阶梯 1–3 级（拆桨 / 系绳 20 cm / 系绳悬停 30 s）未做** —— ★ 2026-09-28 操作者裁定**不再补做**（自由飞已覆盖其风险；**是裁定不是做过了**，那几级的读数仍然是空的）· **台架清单 T1–T8 待下一次台架日一次性全跑** · [`spec`](.scratch/phase1/spec.md) `ready-for-agent` |
| **Phase 2** | **MPC 接管 PID —— SITL 验证**：MPC 输出角速率 + 推力，PX4 只跑角速率环；被控对象**从上游 v1.14.3 构建的 z0 同构机**（**不是 `gz_x500`**；☠️ 原写"从 fork 构建"，该 fork 与 z0 无关） | **(C) 级**：`/fmu/in/vehicle_rates_setpoint` | 🟨 **进行中** —— MPC（模型预测控制）节点 + acados + SITL 同构机都已落地，**闭环跑得起来**；⚠️ **还没收口**（seam 3 的判据未跑满、被控对象待按真机实测重搭）。★ **本格不抄读数**（抄了就会过期）—— 进度以 [`spec`](.scratch/phase2/spec.md) 与 `.scratch/phase2/issues/` 里各票的 `Status:` 行为准 |
| **Phase 3** | **整理 Phase 2 的产出 + 收尾它没做完的票**：把入口层与 `docs/` 裁到读得完，并结掉 Phase 2 未结的票（它们是 Phase 4 的前置）—— **不改控制律**，实机飞行不在这里 | ——（**不引入新的控制接口**） | 📋 **进度以 [`spec`](.scratch/phase3/spec.md) 与 `.scratch/phase3/issues/` 各票的 `Status:` 行为准** |
| **Phase 4** | **MPC 上实机**：四项前置测量 + 一次固件重编（~~ADC 一行修复~~ ☠️ **修法要重定 —— 那条缺陷不在 z0 的固件里**），然后拆桨 → 系绳 → 自由飞四级阶梯 | 同 Phase 2（**(C) 级**） | ⬜ 未开始 · [`spec`](.scratch/phase4/spec.md) `ready-for-human` |

**指导思想：每个阶段只引入一个新变量**（★ **Phase 3 是例外** —— 它不引入新变量，只把前面摊开的产出收敛、把 Phase 2 的账结清）。Phase 1 全走 mavros2（**完全不碰 uXRCE-DDS**），把"动捕链路 + 状态机 + offboard"先跑通；Phase 2 再加 MPC + acados + uXRCE-DDS，**但只在 SITL 里验证，不碰飞机**；Phase 4 才上实机，且**先在离地之前把每个能摔机的未知量量出来**。

---

## 目录

```
├── CONTEXT.md                        术语表（含 MPC 歧义澄清）
├── operation_manual.md               ★★ **现场那一页**：遥控各通道 + FSM + 起链命令（人工飞一次）
├── .scratch/                         ★ 本地 issue tracker（随 git 入库，不是临时目录）
│   ├── phase1/spec.md                Phase 1 spec —— offboard 位置定点飞行
│   ├── phase2/spec.md                Phase 2 spec —— MPC 接管 PID：SITL 验证
│   ├── phase3/spec.md                Phase 3 spec —— 整理 Phase 2 的产出 + 收尾未结票
│   └── phase4/spec.md                Phase 4 spec —— MPC 实机飞行（含固件重编与前置测量）
├── docs/    资料与文档
│   ├── PROJECT.md                    ★ 总纲（先读它）
│   ├── LOCAL-NOTES.md                ⚠️ 含本机凭据，作者决定随 docs/ 入库 —— 见文件头部须知
│   ├── agents/                       skill 配置（issue tracker / triage / 域文档规则）
│   ├── control/
│   │   ├── phase1-implementation.md  ★★ Phase 1 实施清单
│   │   ├── bench-runbook.md          ★★ 拆桨台架清单（Phase 1 必须由人做的那一薄层）
│   │   ├── phase2-implementation.md  ★★ Phase 2 实施清单（Phase 4 spec 引用它）
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
│       ├── z0.params                 z0 飞控参数（928 行；★ 已与飞控真身逐条对账，见 issues/15）
│       └── papers/                   6 篇参考文献 + 中文索引
├── run/                              ★ **人工启动层**：`z0.py` 交互菜单（起链 + 每步核对 +
│                                          录制，含两条「一键」——全部程序 `up` / `up.sh`（起完把终端
│                                          让给操作台选轨迹）与手动遥控飞 `up_manual`——
│                                          以及它们的另一头「清除后台节点」`down` / `down.sh`）
│                                          —— 不含飞行逻辑，也不替代台架清单 —— 见 [`run/README.md`](run/README.md)
│   └── rec.py                        ★ **录制层**（菜单 11，ticket 19）：`ros2 bag record`
│                                          六个场景，**前台倒计时、到点自动收口**（场景 6
│                                          任意录制不倒计时，Ctrl-C 收口）—— **只订阅**，
│                                          不解锁/不起飞/不发帧
├── bag/                              ★ **录制产物**（数据，不是代码；**随 git 入库**）：`<场景>/`
│                                          的功能包 + `fcu/` 里的飞控日志 + 自动生成的 `MANIFEST.md`
│                                          （扁平一层，**没有日期层**；重录落 `<场景>__<UTC 时间戳>/`）
│                                          —— 布局与坑见 [`bag/README.md`](bag/README.md)
├── tools/prereq-evidence/            ★ 前置条件取证脚本（只读）—— 服务 ticket 01
└── src/                              ★ ROS2 工作区（**colcon 根 = 仓库根**，构建产物已被 .gitignore 挡住）
    ├── vrpn_listener/                ✅ 动捕链路，vendored 原样 —— 读它的 VENDORED.md，**别读它的 README.md**
    ├── mocap/                        ✅ 相对位姿（两阶段原点 + 仅归零 yaw）→ `/mavros/vision_pose/pose` → EKF2
    ├── traj/                         ✅ 参考轨迹（三条解析轨迹，50 Hz，**相对轨迹原点**；不发 setpoint）
    ├── bench/                        ✅ 台架夹具（**测试基础设施，不是飞行代码**）：假 mavros + **假 PX4（`/fmu/*`，Phase 2）** + 脚本化动捕 + 假 traj + 故障注入
    ├── control/                      ✅ 自主飞行控制节点（ticket 05/06/08/09：起飞 → 悬停判稳 → **等开始键** → 跟踪轨迹 → 自动降落；**2026-09-28 真机自由飞一次成**，阶梯 1–3 级待做）
    ├── nmpc/                         ✅ Phase 2 的 MPC（模型预测控制）节点：`(C)` 级 CTBR（只发总推力 + 机体角速率）+ 零 ROS 的 OCP 与符号映射 + **装桨前的静态自检**（`ros2 run nmpc preflight_check.py`）
    └── bringup/                      ✅ launch + 操作台（ticket 09）：`z0.launch.py` 一键起 mocap+traj+control(+mavros)、`fly.py` 选轨迹看状态
```

> **构建**（`colcon build` 在**仓库根**跑，不是 `src/`）：
> ```bash
> source /opt/ros/humble/setup.bash && cd ~/ros_ws && colcon build
> ```
> ⚠️ **裸 `colcon build` 之前要先 source `px4_msgs`** —— 本仓库的 `nmpc` 与 `bench` 都依赖它
> （两个包的 `package.xml` 里各有 `<depend>px4_msgs</depend>`），而 `px4_msgs` **在仓库外**
> （下位工作区 `~/px4_msgs_ws`，`release/1.14`）⟹ 完整的一行是
> `source /opt/ros/humble/setup.bash && source ~/px4_msgs_ws/install/setup.bash && cd ~/ros_ws && colcon build`。
> 依赖与运行方式见 [`src/vrpn_listener/VENDORED.md`](src/vrpn_listener/VENDORED.md) §5；
> `mocap` 与它的测试见 [`src/mocap/README.md`](src/mocap/README.md)（`colcon test --packages-select mocap`）；
> `traj` 见 [`src/traj/README.md`](src/traj/README.md)（`--packages-up-to traj`，**它的 seam 1 也依赖 `bench`**）；
> `control` 见 [`src/control/README.md`](src/control/README.md)、夹具见 [`src/bench/README.md`](src/bench/README.md)；
> **一键启动与操作台**见 [`src/bringup/README.md`](src/bringup/README.md)（`ros2 launch bringup z0.launch.py` + `ros2 run bringup fly.py`）；
> **现场那几条命令与每步核对**收在 [`run/z0.py`](run/z0.py) 的菜单里（`run/z0.py --list` 看有哪些，见 [`run/README.md`](run/README.md)）；
> **人工怎么按遥控器飞一次轨迹**（通道作用 → FSM → 起链 → 每步判读 → 排障）看
> [`operation_manual.md`](operation_manual.md)（在**仓库根**）。
> ⚠️ `control` 的测试依赖 `bench` ⟹ 用 **`colcon build --packages-up-to control`**，
> `--packages-select control` 会因缺 `test_depend` 失败。
> ★ **全仓库的 lint 失败全部来自 vendored 的 `vrpn_listener`**（`mocap`/`traj`/`bench`/`control`/`bringup`
> 刻意不含 `ament_lint_auto`，见 [`src/control/README.md`](src/control/README.md)）⟹ **新增任何包不得引入 lint 失败**。
> ★ **本行不抄条数**（抄了就会过期）—— 现算：
>
> ```bash
> colcon test --packages-select vrpn_listener && colcon test-result --test-result-base build/vrpn_listener | tail -1
> ```

**先读哪个**：`docs/PROJECT.md` → `docs/reference/code-inventory.md`（代码取舍）→ `docs/control/phase1-implementation.md`（动手写代码前必读）→ `CONTEXT.md`（查术语）。
**要动手做哪一阶段**：先读对应的 `.scratch/phase{1,2,3,4}/spec.md` —— 它定义该阶段的范围与验收判据。

> **进度只认 `Status:` 行** —— 四份 spec 与每一张票各自的那一行才算数，**本页不抄结论**
> （抄了第二天就过期）。**未结票清单也是现算的**（`done` / `wontfix` 之外都算未结）：
>
> ```bash
> grep -L -E '^Status: *(done|wontfix)' .scratch/phase*/issues/*.md
> ```

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
| ⚠️ **z0 不在 PX4 官方支持矩阵内，且"板名 ≠ 固件目标名"** —— 上游 `hkust_nxt-dual` 板级目标只在 **PX4 ≥ 1.15.0** 存在（v1.14.3 无 `boards/hkust` ✅ 已核实）；而**真身固件的 `boards/` 里只有 `micoair/{h743, h743-v2, h743-aio}`，没有 `hkust/`** | **不刷固件、不追上游**（Q12 不变）。⚠️ **张力没消**：板名 = **MicoAir NxtPX4v2**（用户看实物确认，`hkust_nxt-dual` 只是**上游目标名**）⟹ **Phase 4 重编固件该编哪个板级目标仍未定**。详见 [`refs-manifest.md`](docs/reference/refs-manifest.md) §2.5(d)④ |
| ★★ **固件真身 = `Minderring/PX4-Autopilot` @ `4a0e65f2`（v1.14.3 系）** —— commit `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`，`add micoair743-aio to PX4-Autopilot v1.14.3`（2024-11-18）。**飞控自报的读数**（MAVLink `AUTOPILOT_VERSION`）：`flight_sw_version = 0x010e0300`（⟹ **`1.14.3`**）、`flight_custom_version` 8 字节 `00 00 00 33 f2 65 0e 4a` ⟹ hash **`4a0e65f233`**。其 `msg/`（185 个）与**上游 v1.14.3（tag `1dacb4c`）逐字节相同** | ☠️ **fork `Peize-Liu/PX4-Autopilot @ 5ba2e91f` 与 z0 无关**（它是 **v1.15 世代**；`Nxt-FC` 的 submodule 指认是一条**错的线索**）。**"z0 是 1.14.x" 这个标签是对的** —— 09-20/21 一度判它错，是误判。详见 [`refs-manifest.md`](docs/reference/refs-manifest.md) §2.5(d) |
| ⚠️ **电池电流采样通道缺陷（`ADC12_CH(5)`）—— ★ 实测判为"未中招"** —— 那个错值在 `boards/hkust/nxt-dual/src/board_config.h` 里（引脚表写 `/* PC5 */ GPIO_ADC12_INP8` = 通道 8，宏却读**通道 5**）；而 **z0 真身那棵树没有 `hkust/`**，它的 `boards/micoair/h743-aio` 写的是 `/* PC1 */ ADC1_CH(11)`，**与引脚相符、无此缺陷**。版本边界（第一手）：v1.15.0 仍带、v1.16.0 才改成 `ADC12_CH(8)` | ✅ **2026-09-21 实测判为"未中招"**（读数与判据见 [`params-changes.md`](docs/reference/params-changes.md) §A4）—— **不是运气，是这条缺陷不在这块固件里**。⚠️ 因此 Phase 4「重编重刷 `pr_fix_hkust_nxt_dual`」这个修法**对 z0 的固件不适用，要重定**。~~详见 `refs-manifest.md` §2.7(b)~~ ☠️ 那个指针**本身就指错了**（§2.7(b) 讲分支表，不讲 ADC）；逐项证据见 [`params-analysis.md`](docs/reference/params-analysis.md) **§3.5.1**（与 [`PROJECT.md`](docs/PROJECT.md) 同一行的指向保持一致） |
| ⚠️ **动捕时间戳 `stamp = now()` 是刻意设计，不要改** | 动捕主机与 Jetson 有 **~4 秒**时钟差；改成 `msg_time` 会毁掉 EKF2 融合 |
| ⚠️ **`TrajectorySetpoint` 没有 `thrust` 字段** | 要发姿态+推力得用 `VehicleAttitudeSetpoint` |
| ⚠️ **推力是 `[-1, 1]` 归一化，不是 `0–1`、不是牛顿** | 数值约定错了，MPC 算得再对飞机也不会飞 |
| ⚠️ **`px4_msgs` 版本必须与固件严格匹配** | v1.15 及以前**不匹配会静默失败**（不报错，只是消息没人收） |
| ★★ **`px4_msgs` = `release/1.14` @ `ffb6e80e`**（✅ **用户已签字**）—— 与真身 `4a0e65f2` 在**共有的 184 个 `msg/` 文件上 blob 全同**（唯一集合差异是 `msg/CMakeLists.txt`，那是 PX4 自己的构建文件，`px4_msgs` 本就不带） | ☠️ **原判「`release/1.14` 与 z0 固件不兼容（184 里 28 个字段不同）」作废** —— 那是拿 **fork** 的 `msg/` 比出来的（[`refs-manifest.md`](docs/reference/refs-manifest.md) §2.5(a)）。**不要从任何 fork 生成 `px4_msgs`**。**SITL 被控对象 = 上游 v1.14.3（tag `1dacb4c`）**（默认取法，用户未单独表态）。详见 §2.6 执行版 |

> ✅ **「1.14 表述清扫」已完成（Q36=b）**：跨文档把 z0 与 **fork `5ba2e91f`** 绑定、或说成
> "v1.15 世代"的表述已逐处甄别清扫（保留的**上游 v1.14.3 对照**是合法的，未动）。
> 逐处处置表见 [ticket 10](.scratch/phase1/issues/10-docs-1-14-sweep.md) §处置表。
> **权威判定与分支对照表见 [`refs-manifest.md`](docs/reference/refs-manifest.md) §2.5(d) / §2.6 / §2.7。**

---

## 许可证与致谢

- 本仓库引用的第三方项目各自的许可证见 `docs/reference/open-source-survey.md`。
  ★ **注意**：`uzh-rpg/agilicious` 的许可证**禁止**再分发其代码，本项目**只引其论文与设计思想，不引用任何代码**。
- 参考论文索引见 [`docs/assets/papers/README.md`](docs/assets/papers/README.md)。
