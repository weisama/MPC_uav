# 代码盘点与取舍

> **本文档回答两个问题**：
> 1. 三个指定项目 + 调查中发现的项目里，**哪些文件能用、哪些只是参考、哪些不要**（第一部分）
> 2. 这些东西**落到 z0 的哪个包里**（第二部分）
>
> **实施清单已拆分为独立文件**：
> - Phase 1（offboard 位置定点，`(A)` 级）→ [`../control/phase1-implementation.md`](../control/phase1-implementation.md)
> - Phase 2（MPC 接管 PID，`(C)` 级）→ [`../control/phase2-implementation.md`](../control/phase2-implementation.md)
>
> 相关：[`open-source-survey.md`](open-source-survey.md)（项目级调查与第一手证据）、[`mocap-setup.md`](mocap-setup.md)（动捕链路）、[`px4-interface.md`](px4-interface.md)（接口事实）、[`params-analysis.md`](params-analysis.md)（`z0.params` 逐项）

---

## 判定标准

| 标记 | 含义 |
|---|---|
| ✅ **可复用** | 可以直接拿进来（原样或小改），**已有实机验证** |
| 🔶 **参考** | 借**算法 / 结构 / 参数**，代码必须重写（语言、ROS 版本、话题不匹配） |
| ❌ **不用** | 与本项目无关，或用了会出错 |

**一条贯穿全文的原则**：判断"能不能直接抄"的**唯一硬标准是"它在 z0 这块硬件上被验证过吗"**，而不是"它写得好不好"。所以 XI35 的 PID 控制器虽然写得比我们好，也依然是 ❌ —— 因为它跑在 **(B) 级姿态接口**上，和 Phase 1 的 **(A) 级位置接口**不是一回事。

---

# 第一部分：逐项目盘点

## A.1 `Longer95479/Fast-Drone-XI35`（同硬件，默认分支 `board`，HEAD `8c80ddb`）

### A.1.1 `realflight_modules/px4ctrl/` —— ★ 最有价值的参考

| 文件 | 规模 | 取舍 | 理由 |
|---|---|---|---|
| `src/PX4CtrlFSM.h` | 131 行 | ✅ **重点参考** | ★ **状态机骨架**，五个状态划分得正好（见 A.1.2） |
| `src/PX4CtrlFSM.cpp` | 685 行 | ✅ **重点参考** | 状态转移条件（超时、电压、RC 接管、offboard 掉线）是**实飞才会想到的判据** |
| `src/PX4CtrlParam.{h,cpp}` | 175 行 | 🔶 参考 | 参数结构设计可借；参数值**不可借**（见 A.1.3） |
| `src/input.{h,cpp}` | 426 行 | 🔶 参考 | RC/指令/里程计的超时检查与 `triggered` 边沿检测写法可借 |
| `src/controller.cpp` **的推力模型段** | ~1/3 | ✅ **重点参考** | 期望加速度 → 推力 + **电压补偿**。★ MPC 也要这段 |
| `src/controller.cpp` **的 PID 段** | ~2/3 | ❌ **不用** | 本项目要做的是 MPC |
| `src/px4ctrl_node.cpp` | 134 行 | 🔶 参考 | 订阅/发布话题的接线方式 |
| `launch/run_ctrl.launch` | — | 🔶 参考 | 启动流程可对照；ROS1 → ROS2 要重写 |
| `config/ctrl_param_fpv.yaml` | 72 行 | ❌ **禁止拷贝** | ⚠️ 见 A.1.3 |
| `thrust_calibrate_scrips/thrust_calibrate.py` | 120 行 | 🔶 **参考公式与流程** | ⚠️ Python2 + ROS1 + 话题是 (B) 级的，必须重写；**且它只记录数据，不做拟合** |
| `thrust_calibrate_scrips/{record,stop_cali}.sh` | 18 行 | 🔶 参考 | `rosbag record` 话题列表可借 |
| `LICENSE` | — | — | GPL（移植代码前需确认许可兼容性） |

### A.1.2 ★ 要抄的东西：`PX4CtrlFSM` 的状态划分

```
MANUAL_CTRL   —— 程序未接管，飞控只听遥控器
AUTO_HOVER    —— 已接管，按里程计保持悬停
CMD_CTRL      —— 已接管，正在跟踪外部指令
AUTO_TAKEOFF
AUTO_LAND
```

**★ 关键设计洞察**：这个枚举把**两件正交的事**合并成了一个扁平的枚举——
**(1) 谁在控制飞机**（遥控器 / 程序）与 **(2) 程序在干什么**（悬停 / 跟踪 / 起降）。

本项目要把这个正交性**显式化**（见 [`../control/phase1-implementation.md`](../control/phase1-implementation.md) §4），但**状态名和转移条件照抄**。理由是：它已经在"同一块飞控 + 同一个机载电脑"上跑过实飞，那些看起来多余的判断（消息超时、电池电压、RC 打杆）**每一条都是用事故换来的**。

### A.1.3 ⚠️⚠️ `config/ctrl_param_fpv.yaml` —— 两个真实的坑

**坑 1（会直接导致炸机）**：文件名是 `_fpv`，但第 13 行是 `use_motion_capture_odom: true`。

```yaml
mass        : 0.645   # ← XI35 的实重（kg）
use_motion_capture_odom: true   # ← 这是动捕配置，不是 FPV 配置
low_voltage: 18.0     # ← 6S 电池
```

- **`mass: 0.645` 是 XI35 的，z0 实测 750 g，差 16%。**
- 这个质量直接进入"期望加速度 → 推力"的换算，也影响 PID 增益。整份拷贝到 z0 = 悬停推力系统性偏低 16%。
- `use_motion_capture_odom` 的作用（`px4ctrl_node.cpp:32`）：`true` 订阅 `/motion_capture/motion_capture_odom`，`false` 订阅 `odom`。**它是"选数据源"的开关，不是"启用动捕"的开关。**

**坑 2**：`rc_reverse:` 在**第 15–19 行和第 20–24 行重复定义了两次**，内容完全相同。

→ YAML 解析器**静默取后者**，功能上无害，但**说明这个文件被反复覆盖编辑过**。**不要假设它内部自洽**，任何值都要回源码确认用途。

### A.1.4 ★ `motion_capture/` 包 —— 动捕时延对齐的现成做法

| 文件 | 规模 | 取舍 | 理由 |
|---|---|---|---|
| `src/Process.cpp` | 134 行 | ✅ **重点参考** | ★ **pose / twist / accel 插值对齐**（见下） |
| `src/Process.h` | 30 行 | 🔶 参考 | 三个队列 + 各自 `mutex` 的结构 |
| `src/motion_capture_node.cpp` | 37 行 | 🔶 参考 | 接线 |
| `config/motion_capture.yaml` | 3 行 | ❌ 话题名不匹配 | 它用 `vrpn_client_ros` 的 `/vrpn_client_node/<刚体>/...`，本项目用自写 `vrpn_listener` 的 `/vrpn/<刚体>/...` |
| `ReadMe.md` | — | ✅ 参考 | 把这个包**做什么、为什么**说清楚了 |

**它解决的问题**（`ReadMe.md` 原文）：

> *"将动捕输出的 3 个 Topic 整合为一个动捕里程计话题进行集中输出，本 ROS 包通过**固定 Pose 的数据和时间**，将 **Twist 和 Accel 数据进行线性插值**计算出和 Pose 时间戳对齐的数据，最后打包整合发布在 `/motion_capture/motion_capture_odom`"*

**★ 这是本项目目前没做的事**：`mocap-setup.md` 的方案是**直接转发 pose**，没有做 pose/twist/accel 的互相对齐。**Phase 1 用不到速度，但 Phase 2 的 MPC 一定想用动捕速度**——到那时这段插值逻辑就是现成蓝本。

### A.1.5 其余包

| 包 | 取舍 | 理由 |
|---|---|---|
| `kf_fusion` | 🔶 第二阶段再看 | 动捕 + VINS 融合；Phase 1 只用动捕 |
| `planner/`（`bspline_opt` / `plan_env` / `plan_manage` / `path_searching` / `traj_utils` / `wall_follower` / `drone_detect` / `rosmsg_tcp_bridge`） | ❌ Phase 1 不用 | 避障规划与"跟踪给定轨迹"是两件事。**先能稳定跟踪，再谈规划** |
| `VINS-Fusion-gpu` / `realsense-ros` / `det-reg-pnp` / `jetsoncsi-ros` | ❌ 不用 | 纯视觉/识别 |
| `uav_simulator` / `auto_search` | ❌ 不用 | 本项目用 PX4 SITL |
| `utils/{quadrotor_msgs,uav_utils,pose_utils,imu_filter,catkin_simple,...}` | ❌ 不直接搬 | ROS1 工具链；需要哪些就自己写 |
| 顶层 `Docker/` `shfiles/` `terminator/` `config_script/` | 🔶 参考 | 环境搭建脚本可对照 |
| **上游 `ZJU-FAST-Lab/Fast-Drone-250`** | ❌ **不必看** | XI35 的 `px4ctrl`/`motion_capture` 均源自它，但 XI35 更新且硬件与 z0 相同 |

---

## A.2 `Huan-qun/Sim2real-UAV-Params-` 的 `vrpn_listener.zip` —— ✅ **直接复用（vendored）**

> 用户明确说明：*"是经过验证的可以直接使用的"*。**实测确认它是 ROS2/ament 包**（`ament_cmake` + `rclcpp` + ROS2 XML launch），不是 ROS1。

> ### ★ vendoring 边界（Q10 + Q16=b + Q28=a —— 三处决策合起来才完整，不要再重开）
>
> 本节曾被误读为与 Q10「不要复制」冲突。**2026-09-20 经用户 Q28=a 明确，三者是自洽的**：
>
> | 包 | 来源 | 决定 |
> |---|---|---|
> | **`vrpn_listener`** | **上游 `Huan-qun/Sim2real-UAV-Params-` 的 zip（Q16=b 重新获取的那份）** | ✅ **vendor 进仓库，原样不改** —— 即本节的做法 |
> | `vrpn_relative_pose` | 同上 zip | ❌ **不 vendor** —— 由 z0 自己的 `mocap` 包**重写**（见 `.scratch/phase1/spec.md:32`） |
> | `offboard` | 同上 zip | ❌ **不 vendor** —— 由 z0 自己的 `control` 包**重写** |
>
> **Q10「不要复制」的确切范围**：指**不要复制 2026-09-20 之前留在 `/tmp` 的那几份陈旧副本**
> （当时 `/tmp` 被清空，来源与新鲜度都不可靠）。**不是**说 `vrpn_listener` 不该 vendor。
> **→ 所以：vendor 的对象必须是 Q16=b 从上游 zip 重新取的那一份，且要与
> `refs-manifest.md` 记录的 commit（`b7aeebb`）对得上。**

| 文件 | 规模 | 取舍 | 说明 |
|---|---|---|---|
| `src/vrpn_listener.cpp` | 8088 B | ✅ **原样使用** | ★ 见下：**时间戳是刻意设计，不要改** |
| `include/vrpn_listener/vrpn_listener.hpp` | 1982 B | ✅ 原样 | 含 `Synchronizer` 结构（每刚体一套三发布者） |
| `CMakeLists.txt` / `package.xml` | — | ✅ 原样 | 依赖 `VRPN`（系统库）+ `rclcpp` + `geometry_msgs` + `tf2` |
| `cmake/FindVRPN.cmake` / `Findquatlib.cmake` | — | ✅ 原样 | ★ **系统没装 VRPN 时需要这两个**（已实测本机无 `libvrpn`） |
| `config/params.yaml` | 7 行 | ✅ 原样（值已对） | `server: 10.1.1.198` / `port: 3883` / `frame_id: "world"` / `mainloop_frequency: 400.0` |
| `launch/sync_entity_state.launch` | — | ✅ 原样 | ROS2 XML launch |
| `README.md` | — | ❌ **不要信** | ⚠️ **已过时**：它仍叫你 clone 公开的 `efc-robot/vrpn_client_ros2`。**源码已经定制过了，README 没跟上** |

### ★ 发布的话题（源码 `createSynchronizer`，实测）

```
/vrpn/<刚体名>/pose     geometry_msgs/PoseStamped    ← 本项目只用这个
/vrpn/<刚体名>/twist    geometry_msgs/TwistStamped
/vrpn/<刚体名>/accel    geometry_msgs/AccelStamped
```

刚体名从 VRPN 连接自动发现（`refresh_trackers`，1 Hz）；名字里的空格会被 `replaceSpace` 替换。**不需要在配置里写刚体名。**

### ⚠️ 唯一一条"不要改"的硬规矩

`vrpn_listener.cpp:40`：

```cpp
    // Use ROS receive time, not VRPN msg_time, because the mocap PC clock is
    // not NTP-synced with the Jetson (~4s offset observed). Adds ~2-5ms jitter
    // but keeps the stamp in the same clock domain as IMU/MAVROS for fusion.
    pose_msg.header.stamp = listener->now();
```

**这是刻意的，不是 bug。** 换成 `msg_time` 会因动捕主机与 Jetson 有 **~4 秒**时钟差而灾难性出错。**vendored 时连这一行一起原样保留。**（完整论证见 [`mocap-setup.md`](mocap-setup.md) 6.2。）

---

## A.3 `DISCOWER/px4-mpc` —— ✅ **Phase 2 的代码骨架**

| 项 | 内容 |
|---|---|
| 规模 | 40 个文件 |
| 价值 | ★ **唯一一个把 `acados` 和 `PX4` 连起来的可运行蓝本** |
| 关键事实 | 它发布 **`/fmu/in/vehicle_rates_setpoint`** —— **与本项目 Phase 2 的 (C) 方案完全一致** |

**取舍**：🔶 **Phase 2 再引入**。Phase 1 **完全不需要它**（全 mavros2，(A) 级），现在 clone 下来读一遍可以，但不要进 Phase 1 的构建。

**理由**：Phase 1 的目标是"**让一条链先跑通**"。同时引入 uXRCE-DDS + acados + MPC 是三个新变量一起上，出问题无法归因（这正是"每个阶段只加一个新变量"的原则）。

---

## A.4 `uzh-rpg/agilicious` —— ❌ **一行代码都不能拿**

| 项 | 第一手事实 |
|---|---|
| 规模 | **38 个文件，零代码**（24 个 `.rst` + README + 8 个 `.bib` + 1 个 `docs/conf.py`） |
| 相关度 | `PX4` / `mavros` / `uXRCE` / `ROS2` / `Offboard` **命中数全部为 0** |
| 最后提交 | **2023-03-07**（已三年未更新） |
| 许可证 | ★ **只能由 UZH 明确授权的研究者使用**；**§2.3 明文禁止**再许可/分发/披露给任何第三方 |

**→ 结论：❌ 代码不用（拿不到），且即使拿到也不能进仓库。**

**但它有 3 条概念上的价值**（详见 [`open-source-survey.md`](open-source-survey.md) §2）：

1. `guard` 模块 = **本项目"安全网"的现成范本**：独立 pipeline + 独立估计 + 3D 边界，主链路异常时自动接管
2. 它的 MPC 用 **`acados` + `HPIPM`** —— 本项目求解器选型的**独立佐证**
3. `INDI` 在其架构里明确是**内环（body rate 跟踪）** —— 与 `(C) → (C)+INDI → (D)` 吻合

---

## A.5 `HKUST-Aerial-Robotics/OmniNxt` —— ❌ **没有代码**

| 项 | 第一手事实 |
|---|---|
| 规模 | **4 个文件**：`README.md` + `LICENSE` + `misc/OmniNxt.pdf`(15 MB) + `misc/system_overview.png` |
| 声称的外部代码 | ❌ **两个都 404**（`D2SLAM-Fusion/controller-PX4Control`、`planner-EgoPlanner`） |
| `OmniNxt-03.params` | ❌ **不在仓库**；README 指向 OneDrive，实测返回 275 KB HTML **登录页** |
| 构型 | ⚠️ **是全向视觉感知的常规四旋翼，不是全向倾转**（已用本地 PDF 的 `pdftotext` 核实） |

**→ 结论：❌ 代码与参数都不用。** 它唯一的用处是**引出了 A.6**。

---

## A.6 `Minderring/PX4-Autopilot`（**z0 固件真身**）+ `Nxt-FC` / `Peize-Liu/PX4-Autopilot`（☠️ 已降格，见下）—— ★★ **飞控侧的权威来源**

> ☠️ **2026-09-21 定案：标题里的 `Peize-Liu/PX4-Autopilot` 已降格。** 真身是
> **`Minderring/PX4-Autopilot` @ `4a0e65f2`**（v1.14.3 系的 MicoAir 厂商构建，飞控自报的读数；
> 见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)）。`Peize-Liu` 那支 fork（`5ba2e91f`）**与 z0 无关**，
> 它的 `.gitmodules` 指认**不等于飞控里烧的那一份**。**下表凡提到 `hkust/nxt-dual` 的行都受此影响。**

**这不是"代码参考"，而是"事实来源"**——它决定了 z0 的固件与参数的比对基准。

| 关键事实 | 对 z0 的影响 |
|---|---|
| ★ **`hkust_nxt-dual` 板级目标 2024-04-02 才进上游，只在 PX4 ≥ v1.15.0 存在** | ⚠️ **z0 不在官方支持矩阵内**，必然是厂商构建 ✅（已证实：真身 `boards/` 里只有 `micoair/{h743, h743-v2, h743-aio}`）。⚠️ **但别把「厂商构建」读成「跑 `hkust_nxt-dual`」** —— 真身树里**没有 `hkust/`**，用的是 `micoair` 板级目标 |
| ☠️ **原写「`boards/hkust/nxt-dual/init/rc.board_defaults` 给出板级默认参数」—— 该路径不在真身树里** | ⚠️ **本行原称它是 `z0.params` 的"权威 diff 基准"，作废。** 真身树里的对应文件是 `boards/micoair/{h743,h743-v2,h743-aio}/init/rc.board_defaults`，**三块板的默认值互不相同**（`BAT1_A_PER_V` = 40 / 40 / 14.14，`BAT1_V_DIV` = 21.2 / 21.12 / 21.12，`IMU_GYRO_CUTOFF` = 80 / 80 / 100，`SYS_HAS_MAG` = 1 / 1 / 0）。**到底该拿哪一块当基准 = "板名 NxtPX4v2 ≠ 固件目标名"这个未决项的一部分**，见 [`refs-manifest.md`](refs-manifest.md) §2.5(d)④ —— **在它定案前不要引本行当基准** |
| ⚠️ **`ADC_BATTERY_CURRENT_CHANNEL` 曾为 `ADC12_CH(5)`，上游已修为 `ADC12_CH(8)`** | ✅ **但这条缺陷（在 `boards/hkust/nxt-dual`）不在真身树里** —— 真身的 `boards/micoair/h743-aio` 写的是 `/* PC1 */ ADC1_CH(11)`，**与引脚相符、无此缺陷**。**2026-09-21 上电实测亦判为「未中招」**（见 [`params-changes.md`](params-changes.md) §A4） |
| `firmware/` 里的预编译固件是 **`nxt-v1`（单 BMI088 + BMP388）** | ❌ **不要刷到 NxtPX4v2（双 BMI088 + SPL06）上**，传感器直接不工作 |
| ☠️ **原写「fork 的 `feat_deprecated_v1.14.x_high_imu` 分支的 `rc.board_defaults` 只有 2 行」—— 已作废** | ⚠️ **本行描述的是 fork 的分支，不是 z0 的固件** —— 由此推出的「**1.14.x 构建不会自动带电池/电机/autostart 默认值，必须全部写进 `z0.params`**」**对真身不成立**：真身三块 `micoair` 板的 `rc.board_defaults` 都自带 `BAT1_*` / `MC_ROLLRATE_K` / `CBRK_SUPPLY_CHK` / `SYS_AUTOSTART` 等 |

**取舍**：🔶 **读、不抄**。用来比对参数、判断固件风险。完整证据见 [`open-source-survey.md`](open-source-survey.md) §9
与 [`refs-manifest.md`](refs-manifest.md) §2.5(d)。

---

## A.7 `acados/acados` —— ✅ **Phase 2 的必用依赖**

| 项 | 内容 |
|---|---|
| 取舍 | ✅ **Phase 2 必用**（Phase 1 不需要） |
| 安装方式 | 源码编译（aarch64） |
| ★ Orin NX 要点 | `BLASFEO_TARGET = ARMV8A_ARM_CORTEX_A76`（无 A78 目标）、`HPIPM_TARGET = GENERIC`、☠️ 缺 Tera 渲染器**不再报** `EOFError`（v0.6.0 起）—— ★ **但「必须装」这件事不变**；真身是三种落点，其中「在、但渲染不出东西」**完全静默**，见 [`../control/acados-orin-build.md`](../control/acados-orin-build.md) §4）、必须设 `ACADOS_SOURCE_DIR` 与 `LD_LIBRARY_PATH` |

详细配置与求解器参数见 [`../control/mpc-roadmap.md`](../control/mpc-roadmap.md)。

---

# 第二部分：取舍结论 → 目标包结构

## B.1 仓库与目录（已定）

```
ros_ws/  = weisama/MPC_uav   （private）
├── docs/     资料（已有，本文档所在）
└── src/      ROS2 代码 —— ★ **六个包都已落地**（本节的"待建/新写"是当初的取舍结论，不是现状）
    ├── vrpn_listener/   ← vendored：定制版原样放进，★ 写明"勿替换为公开版"
    ├── mocap/           ← 重写：相对位姿（解锁边沿锁原点 + 仅 yaw 归零）
    ├── traj/            ← 新写：悬停 / 正方形 / 八字
    ├── bench/           ← 新写（当初没列入）：台架夹具，**测试基础设施、不是飞行代码**
    ├── control/         ← 新写：FSM + offboard setpoint（Phase 1）→ MPC（Phase 2）
    └── bringup/         ← 新写：launch + 参数

★ `bench` 是后加的：`control` 的验收要"假 mavros + 脚本化动捕 + 故障注入 + DUT 真子进程"，
  这件事本身够一个包，塞进 `control/test/` 会逼每个测试自己造一遍世界。
★ `bringup` 落地时**推翻了本文档给它的蓝本**：`config/mavros_z0.yaml` 没有被任何 launch 读取 ——
  上游 `px4.launch` 把 `config_yaml` 写死了，见 [`../../src/bringup/README.md`](../../src/bringup/README.md)。
```

## B.2 每个包的取舍链条（"为什么是它"）

| 包 | 主要来源 | 关键取舍 |
|---|---|---|
| `vrpn_listener` | S2R zip | ✅ **原样 vendored**。**唯一改动是把它放进 `src/` 并保留原 `params.yaml`**（值已对）。**不要"顺手升级"成公开版** |
| `mocap` | 重写；蓝本 = `mocap-setup.md` 7.3 的旧节点 | ❌ **不直接复用旧节点**：它的原点是"启动后 10 帧"，本项目要"**解锁上升沿**"；且它**不归零姿态**，本项目要**仅归零 yaw**。🔶 可借 `motion_capture/Process.cpp` 的**插值对齐**思路（Phase 2 用） |
| `traj` | 新写 | ❌ **不抄 ego-planner**（那是避障规划）。Phase 1 只需要三个**解析式**轨迹 |
| `control` | 🔶 **`px4ctrl` 的 FSM 骨架 + 推力模型段** | ❌ **不抄 `controller.cpp` 的 PID**；❌ **不抄 `ctrl_param_fpv.yaml` 的任何值**（质量是 0.645）。Phase 2 时这里的控制器换成 MPC（🔶 蓝本 = `px4-mpc`） |
| `bringup` | 新写 | 🔶 参照 `px4ctrl/launch/run_ctrl.launch` 的启动顺序 |

## B.3 一句话总结

> **三个指定项目里，只有 Fast-Drone-XI35 有可抄的东西，而且可抄的只有"状态机 + 动捕对齐 + 推力模型公式"，不是它的控制器。真正决定成败的两条线，一条是 `px4-mpc`（Phase 2 骨架），一条是 `Nxt-FC`（固件与参数基准）。**

---

## B.4 下一步

取舍到此结束。**动手前先读实施清单**：

| 阶段 | 文件 | 内容 |
|---|---|---|
| **Phase 1** | [`../control/phase1-implementation.md`](../control/phase1-implementation.md) | 包与文件、话题接口、坐标系、FSM、判据与轨迹、参数核对、验证顺序、避坑清单、需要你物理操作的 4 件事 |
| **Phase 2** | [`../control/phase2-implementation.md`](../control/phase2-implementation.md) | D1–D10 决议、`px4-mpc` 第一手解读、S1–S8 步骤、待实测参数 |
| **Phase 2** | [`../control/sign-map.md`](../control/sign-map.md) | ★ **符号映射表的唯一权威** —— 推导、与参考实现的对照、推力归一化的量纲；代码回链它 |
