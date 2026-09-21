# 05: `control` 骨架 —— mavros 接口 + OFFBOARD 心跳流 + `INIT→STANDBY`，含假 mavros 台架

Status: ready-for-agent
Type: task

**What to build:** 机载电脑第一次真正"接管"飞机：节点把 PX4 拉进 OFFBOARD，并以 **50 Hz** 持续流「位置 + yaw」setpoint —— 但**先不飞任何轨迹**，状态只走到 `STANDBY`（未解锁、已在流）。

**同时建立可复用的测试夹具**（假 mavros + 脚本化动捕输入），它是本阶段以及 Phase 2/3 所有节点级测试的**公共底座**。

**这是刻意提前的前置工作，不是副产品：** 假 mavros 必须能在**指定时刻注入故障**（让 `/mavros/state` 静默 0.5 s、注入时间戳倒退、注入电压跌落）。**真 SITL 做不到这件事**，而状态机里那些"实飞才会想到"的判据全靠它才可测。所以它在第一张 control ticket 里就先做出来，而不是等需要时现搭。**SITL 因此不进自动化测试**，它只是上机前手动跑一次的集成关卡。

**状态机形状** —— 两个正交维度显式分开，**不压成扁平枚举**：

```
控制权：   MANUAL ──(开始信号)──▶ AUTO ──(RC 打杆 / 超时 / 异常)──▶ MANUAL
任务态：   IDLE ─▶ TAKEOFF ─▶ HOVER ─▶ TRACK ─▶ LAND
```

本张只实现：

- `INIT` —— 等 mavros 连接、动捕有数据、`local_position` 有效；**不发 setpoint**
- `STANDBY` —— 未解锁；**已开始以 50 Hz 持续发布 setpoint**（位置 = 当前，yaw = 0）

**两件必须做对的事：**

1. `type_mask` 用 **2552**（忽略速度 / 加速度 / yaw_rate，即"位置 + yaw"），`coordinate_frame = FRAME_LOCAL_NED`，`position.z` 向下为正，`yaw` 用弧度。
2. **切进 OFFBOARD 之前** setpoint 必须已以 ≥2 Hz 持续发布 **≥1 s**（本项目 50 Hz、`STANDBY` 阶段就在发）。**断流 PX4 会自动退出 OFFBOARD**（`COM_OF_LOSS_T = 1.0 s`）。

**Blocked by:** 04（代码已就绪）、[13](13-mavros-bench-vision-pose.md)（★ 本张整条建在 mavros 上，而**本机还没装 mavros**）

- [ ] `INIT→STANDBY` 的守卫逐条生效：`connected` ∧ 动捕有数据 ∧ `local_position` 有效 —— 缺任一条就**不发 setpoint**
- [ ] 台架上节点把 PX4 拉进 OFFBOARD，setpoint 以 **50 Hz** 持续流出，`type_mask` / 坐标系 / 时间戳单位（**微秒**）字段正确
- [ ] **断流实验**：杀掉节点 → PX4 在 `COM_OF_LOSS_T` 之后**自行退出 OFFBOARD**（证明心跳是唯一的维持机制）
- [ ] 夹具可复用：假 mavros 能收 setpoint 话题、能发 `/mavros/state` 等入向话题，且**能在指定时刻注入故障**
- [ ] 脚本化 `/vrpn/<刚体>/pose` 发布者可用（或 rosbag 回放）
- [ ] 夹具跑得进 `colcon test`，至少一个冒烟场景通过
- [ ] FSM 的两个维度在代码里是**显式分开**的，不是扁平枚举

---

## Comments

### 2026-09-21 —— 串口权限：一个已实测的阻塞前提

在 ticket 01 的侦察中实测到，**本张会撞上这个**：

- 用户 `chen` **不在 `dialout` 组**（`dialout:x:20:` 成员为空）。
- `/dev/ttyTHS1`、`/dev/ttyTHS2` 权限是 `root:dialout 660`（实测 `ls -l`）。
- 实测打开失败：`SerialException: [Errno 13] could not open port /dev/ttyTHS1: Permission denied`。

**为什么与本张有关**：本张要让节点把 PX4 拉进 OFFBOARD，而**连上飞控这一条就依赖串口能打开** ——
不论是走 mavros 还是直接开串口。这是**夹具之外的、环境层面的前置**。

**修法**（需人执行，agent 没有免密 sudo）：

```bash
sudo usermod -aG dialout "$(id -un)"
# ⚠️ 必须**重新登录**（或 newgrp dialout）才生效 —— 仅加入组不重登不生效
```

> ⚠️ 注意：这条**不是**本张的验收项，只是环境前提。它也不解决 ticket 01 的 ②
> （飞控到底接 THS1 还是 THS2），那一问仍需现场认定，或用
> [`tools/prereq-evidence/mavlink_probe.py --scan`](../../../tools/prereq-evidence/mavlink_probe.py) 自动判定。

### 2026-09-21 —— 一条与本张有关的边界：动捕"数据中断"的告警已被 ticket 04 做掉一半

spec 用户故事 10（"动捕数据中断超过阈值时看到明确告警"）有**两半**，别重复实现、也别以为没人管：

| 哪一半 | 归谁 | 状态 |
|---|---|---|
| **数据到没到**（动捕/VRPN 侧断了） | [`mocap` 节点](04-mocap-relative-pose.md) | ✅ **已实现并实测**（看门狗，中断超 `no_data_warn_s` 报 ERROR，恢复报 WARN；见 04 §八） |
| **消费是否超时**（`/mavros/local_position`、`/mavros/state` 静默） | **本张**（`phase1-implementation.md` §4.3-3 的 `msg_timeout`） | ⏳ 待做 |

⟹ 本张仍需要自己的超时判据：`mocap` 那一侧正常不代表 mavros 那一侧正常（两个话题来自不同进程）。

### 2026-09-21 —— ★ 台架实测送来一条**夹具硬要求**：mavros 的话题是 sensor QoS（BEST_EFFORT）

真 mavros 上实测（飞控已上电，证据见 [ticket 13](13-mavros-bench-vision-pose.md) §实测 ⑤）：

- `/mavros/local_position/pose` 与 `/mavros/local_position/pose_cov` 都是 **BEST_EFFORT / VOLATILE**。
  用默认的 **RELIABLE** 订阅**一帧都收不到** —— 只在日志里留一句
  `New publisher discovered ... offering incompatible QoS ... Last incompatible policy: RELIABILITY`，
  **不报错、不退出**（正是"看起来像逻辑错"的失败方式）。
- ⟹ **假 mavros 必须用同样的 QoS 发这些入向话题**，否则被测节点收不到、测试会以误导人的方式失败。
- 反过来也顺带确认了：本节点发出的 `/mavros/vision_pose/pose` 是 **RELIABLE**，与真 mavros 的订阅
  （`vision_pose` 插件，`ros2 topic info -v` 实读 RELIABLE）**兼容** ⟹ 夹具照抄即可。

另两条与夹具直接相关的实测事实：

1. **`/mavros/state` 实测 `1.00 Hz`**（30 条 / 30 s；间隔 0.998–1.002 s）—— 夹具注入 `armed` 边沿时要按这个
   **真实节拍**来（否则会测出一个现实中不存在的敏捷边沿）；也意味着 `msg_timeout` 的阈值不能小于 1 s。
2. **`/mavros/local_position/pose_cov` 默认没有数据** —— 源码 `local_position.cpp:257` 只在飞控推
   `LOCAL_POSITION_NED_COV` 时才有输出，PX4 默认不推。所以**别在断言里依赖协方差**。
