# 05: `control` 骨架 —— mavros 接口 + OFFBOARD 心跳流 + `INIT→STANDBY`，含假 mavros 台架

Status: ready-for-human
Type: task

> ⚠️ **为什么是 `ready-for-human` 而不是 `done`**：代码与台架夹具**已完成并全绿**，
> 但**两条验收项判的完全是 PX4 侧行为**（真 PX4 被拉进 OFFBOARD；杀节点后 PX4 在 `COM_OF_LOSS_T`
> 后自行退出）—— 夹具**不模拟 PX4**（那是 SITL / 真机的职责），而且当前 `/dev/ttyTHS1`
> 被另一会话的 mavros 占着。剩下的事需要**人 + 空闲的飞控 + 拆桨台架**。详见下方 Comments。

**What to build:** 机载电脑第一次真正"接管"飞机：节点把 PX4 拉进 OFFBOARD，并以 **50 Hz** 持续流「位置 + yaw」setpoint —— 但**先不飞任何轨迹**，状态只走到 `STANDBY`（未解锁、已在流）。

**同时建立可复用的测试夹具**（假 mavros + 脚本化动捕输入），它是本阶段以及 Phase 2/3 所有节点级测试的**公共底座**。

**这是刻意提前的前置工作，不是副产品：** 假 mavros 必须能在**指定时刻注入故障**（让 `/mavros/state` 静默 0.5 s、注入时间戳倒退、注入电压跌落）。**真 SITL 做不到这件事**，而状态机里那些"实飞才会想到"的判据全靠它才可测。所以它在第一张 control ticket 里就先做出来，而不是等需要时现搭。**SITL 因此不进自动化测试**，它只是上机前手动跑一次的集成关卡。

**状态机形状** —— 两个正交维度显式分开，**不压成扁平枚举**：

```
控制权：   MANUAL ──(开始信号)──▶ AUTO ──(RC 打杆 / 超时 / 异常)──▶ MANUAL
任务态：   IDLE ─▶ TAKEOFF ─▶ HOVER ─▶ TRACK ─▶ LAND
```

> ☠️ **这段是设计讨论期的原型，不是最终口径（2026-09-21，实现后被审查发现）**，两处不一致：
> ①**任务态缺 `INIT` 与 `STANDBY`**（见下方"本张只实现"那两行——那才是本张实现的），
> 本段里的 `IDLE` 大致对应 `INIT`；②**"开始信号"不是进入 `AUTO` 的边沿**：
> `AUTO` = "本项目正在发会驱动飞机的指令（已解锁的 offboard）"，而**切 OFFBOARD 不等于交出控制权**
> （那时飞机还没解锁）⟹ 进入 `AUTO` 的边沿是**解锁**（归 ticket 08）。
> 现口径下 `MANUAL/STANDBY` 是**正常的稳定态**，不是过渡态。

本张只实现：

- `INIT` —— 等 mavros 连接、动捕有数据、`local_position` 有效；**不发 setpoint**
- `STANDBY` —— 未解锁；**已开始以 50 Hz 持续发布 setpoint**（位置 = 当前，yaw = 0）

**两件必须做对的事：**

1. `type_mask` 用 **2552**（忽略速度 / 加速度 / yaw_rate，即"位置 + yaw"），`coordinate_frame = FRAME_LOCAL_NED`，`position.z` 向下为正，`yaw` 用弧度。
   > ☠️☠️ **更正（2026-09-21，本张实现后被审查发现）：`position.z` 向下为正是错的。**
   > mavros 对入向的 `FRAME_LOCAL_NED` setpoint 做 **ENU→NED** 转换（`setpoint_raw.cpp:231`），
   > 所以**我们填进去的是 ROS/ENU 口径 —— `z` 向上为正**；`FRAME_LOCAL_NED` 只是**协议枚举名**，
   > 不代表填写口径。同理 **`yaw = 0` 到了 PX4 是 +90°（机头朝东），不是"保持当前机头"**。
   > 完整论证与行号见下方 Comments 的 ☠️☠️ 条与
   > [`docs/control/phase1-implementation.md`](../../../docs/control/phase1-implementation.md) §3.1/§3.3。
   > **照 NED 去翻 z 的符号会得到一架往地里钻的飞机。**
2. **切进 OFFBOARD 之前** setpoint 必须已以 ≥2 Hz 持续发布 **≥1 s**（本项目 50 Hz、`STANDBY` 阶段就在发）。**断流 PX4 会自动退出 OFFBOARD**（`COM_OF_LOSS_T = 1.0 s`）。

**Blocked by:** 04（代码已就绪）、[13](13-mavros-bench-vision-pose.md)（★ 本张整条建在 mavros 上，而**本机还没装 mavros**）

- [x] `INIT→STANDBY` 的守卫逐条生效：`connected` ∧ 动捕有数据 ∧ `local_position` 有效 —— 缺任一条就**不发 setpoint**
      （`test_control_standby.py`：动捕暂停 / `local_position` 静默 / `/mavros/state` 静默**逐条**测，每条都带正控）
- [ ] 台架上节点把 PX4 拉进 OFFBOARD，setpoint 以 **50 Hz** 持续流出，`type_mask` / 坐标系 / 时间戳单位（**微秒**）字段正确
      ⟶ ⏳ **只完成夹具那一半**：字段与 50 Hz 已逐条断言（`type_mask=2552`、`coordinate_frame=1`、
      三轴位置跟随假 EKF、`yaw=0`、stamp 非零、帧率 ≥0.8×50）。**真 PX4 那一半没做**（飞控被占）。
      ⚠️ 且「时间戳单位（**微秒**）」这半句**已被推翻**，见下方 Comments 第 2 条
- [ ] **断流实验**：杀掉节点 → PX4 在 `COM_OF_LOSS_T` 之后**自行退出 OFFBOARD**（证明心跳是唯一的维持机制）
      ⟶ ⏳ **夹具侧已做**（`test_杀掉_DUT_之后_setpoint_流停止`），**PX4 侧没做**（同上）
- [x] 夹具可复用：假 mavros 能收 setpoint 话题、能发 `/mavros/state` 等入向话题，且**能在指定时刻注入故障**
- [x] 脚本化 `/vrpn/<刚体>/pose` 发布者可用（或 rosbag 回放）—— 两种喂法都实现了
      （`VrpnSource(target='vrpn')` 发上游、`target='mocap'` 直接发 `/mocap/pose_enu`）
- [x] 夹具跑得进 `colcon test`，至少一个冒烟场景通过
      （`control` 话题级 9 条 + `control` 纯函数 12 条 + 夹具自检 9 条 + 时序纯逻辑 6 条 = **36 条全绿**；
      审查后补了 seam 2 那一档，见下方 Comments）
- [x] FSM 的两个维度在代码里是**显式分开**的，不是扁平枚举
      （`Authority{MANUAL,AUTO}` × `Phase{INIT,STANDBY,TAKEOFF,HOVER,TRACK,LAND}`，
      `/control/state` 发 `"MANUAL/STANDBY"`）

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

### 2026-09-21 —— ✅ 实现完成：`src/control/` + `src/bench/`（代码全绿，两条真机验收项留待飞控空闲）

**产出**（`colcon build --packages-up-to control` → `colcon test` 全绿）：

| 包 | 内容 | 测试 |
|---|---|---|
| [`src/control/`](../../../src/control/) | 控制节点：`fsm.hpp`（纯逻辑、零 ROS）+ `px4_interface.{hpp,cpp}`（mavros 边界）+ `control_node.cpp`（只做接线） | **9 条**节点级（seam 1，DUT 是子进程） |
| [`src/bench/`](../../../src/bench/) | 可复用夹具：假 mavros + 脚本化动捕 + 故障注入时序 + `Scenario` | **9 条**夹具自检 + **6 条**时序纯逻辑 |

两个包的 `README.md` 就是使用说明（`control` 的参数表与两个维度的语义、`bench` 的"它假的是什么、
能注入什么、怎么写一个场景"）。★ **先读 `src/bench/README.md` 再写别的节点的测试** ——
Phase 2/3 的所有节点级测试都建在它上面。

**三条已定决策**（用户 2026-09-21 裁定）：① `STANDBY` **自动切 OFFBOARD、参数可关**（`enter_offboard`，
默认 true）——因为切模式≠交出控制权（未解锁 ⟹ 仍是 `MANUAL`）；② 夹具落 **新包 `src/bench/`**；
③ 与既有文档冲突的事实**就地标注更正**（带源码路径与行号）。

#### ★ 本次新增的三条第一手事实（都改了文档）

1. **`/mavros/state` 实测 1.000 Hz，超时判据必须是 2.5 s，不能照抄 `px4ctrl` 的 0.5 s。**
   根源读源码确认：mavros **只在收到 MAVLink HEARTBEAT 时才发这个话题**
   （`sys_status.cpp` 的 `handle_heartbeat` → `state_pub->publish`），而 HEARTBEAT 在 PX4 里是
   **定频流**（`mavlink/streams/HEARTBEAT.hpp` 的 `const_rate() == true`）⟹ **不是 mavros 的定时器，改不了**。
   **这同时结掉了 [ticket 13](13-mavros-bench-vision-pose.md) 坑 4 与 [ticket 04](04-mocap-relative-pose.md) §七 第 4 条的悬案。**
   ⚠️ 代价：**解锁上升沿最多晚 1 s 被检出** —— ticket 08 的起飞逻辑必须把这一秒算进去。
   已落：`control.yaml` 的 `state_timeout_s: 2.5`、夹具 `bench.yaml` 的 `state: 1.0 Hz`（★ 夹具不许比真机更灵敏）、
   实施清单 §4.3-3 的超时口径表。
2. **spec 的「所有发往 PX4 的消息用微秒时间戳」对 Phase 1 不成立**（☠️ **更正**）。
   该条只对 Phase 2 的 uXRCE-DDS 成立。Phase 1 走 mavros：`header.stamp` → MAVLink `time_boot_ms`
   （**毫秒**、截断 32 位，`mavros/include/mavros/plugin.hpp:264-267`），而 **PX4 根本不读它** ——
   `mavlink_receiver.cpp:960` 的 handler 只读 `type_mask`/`coordinate_frame`/各分量，
   时间戳在发布处用 `hrt_absolute_time()` 自己打（**1067 行**）。第一手读的是 `~/refs/px4-v1.15.0`
   （与 z0 固件同代）。实现口径 = `header.stamp = now()`。已落：实施清单新增 §3.7、`spec.md` 就地加 ☠️ 注。
3. **`type_mask = 2552` 复核通过，且顺带读到一个重要机制**：该 handler **只在
   `nav_state == OFFBOARD` 时才把 setpoint 发进 ORB**（`mavlink_receiver.cpp:1066` 注释
   "only publish setpoint once in OFFBOARD"）⟹ **切模式之前流的那些 setpoint 是被丢弃的**；
   真正起作用的是它们每条都触发的 **`offboard_control_mode` 发布**（`Commander::offboardControlCheck()`）。
   这就是"前置流"能生效的机制。另外 `FORCE_SET` 那条 `mavlink_log_critical` 分支**不会**被 2552 触发。

#### ★ 一条对既有文档的更正：lint 失败基线是 **172**，不是 176

跑完全仓库测试后逐条数 XML：**172** 条失败用例，**全部**来自 vendored 的 `vrpn_listener`
（copyright 4 + cpplint 20 + lint_cmake 146 + uncrustify 2）。文档里写的 176 高估了 4。
`control` / `bench` **刻意不加 `ament_lint_auto`**（沿用 `mocap` 的先例）⟹ 本次不改变这个数字。

> ⚠️ 但 `colcon test-result --all` 打印的仍然是 **176**：它还会把
> `build/vrpn_listener/Testing/*/Test.xml`（CTest 层的汇总日志）里那 4 条**重复记一遍**。
> 两个数字指的是同一批失败，不是"又多了 4 条" —— 复核时请看 per-package 的 xunit 分解
> （`colcon test-result --all | grep '^build/'`），别只看最后那行 Summary。

#### 台架夹具的两条"静默失败"陷阱（已写成回归测试）

- **QoS 不兼容不报错**：真 mavros 的 `/mavros/local_position/pose` 与 `/mavros/battery` 是
  **best_effort**（`SensorDataQoS()`），`/mavros/state` 是**可靠 + `transient_local`**。
  按默认（可靠）订阅前者会**一帧都收不到**，日志里只有一句 `offering incompatible QoS`。
  夹具逐条复刻了这些 QoS，并有**专门的回归测试**盯着"可靠订阅者收不到 best_effort 数据"。
- **夹具与 DUT 的 `ROS_LOCALHOST_ONLY` 必须一致**：本机环境里该变量是 `0`（有人设过），
  夹具若 `setdefault`、DUT 却强设 `1`，两边会用不同的 DDS 传输配置 ——
  现象是**两个进程都活着但彼此发现不了，且没有任何报错**（本次实现时真踩到这个，
  9 条 control 测试全挂在 `wait_for_subscribers("state")`）。现在两边都用常量硬写。

**安全边界**：夹具默认 `ROS_DOMAIN_ID=73`，且**拒绝 domain 0**（那里跑着真 mavros + 真飞控，
而夹具提供 `/mavros/cmd/arming` ⟹ 测试里的"假解锁"可能打到真飞机上）。

⏳ **遗留（需人 + 空闲飞控 + 拆桨台架）**：真 PX4 被拉进 OFFBOARD；杀节点后 PX4 在
`COM_OF_LOSS_T`（1.0 s）后自行退出 OFFBOARD。**夹具不模拟 PX4**，这两条的判据全在 PX4 侧。
当前 `/dev/ttyTHS1` 由另一会话的 mavros 占着（[ticket 14](14-mavros-from-source.md)），**本张不抢**。
★ **步骤、判据、记录表见 [`docs/control/bench-runbook.md`](../../../docs/control/bench-runbook.md) 的 T3 与 T5**
（2026-09-22 起）：一次台架会话就能把动捕链、坐标轴、这两条、以及 ticket 06 的打杆/`mode_lost` 一起做完。

### 2026-09-21 —— ★★ `/code-review` 双轴审查（fixpoint `HEAD = 0964172`）与随后的修正

审查只覆盖本张自己的产出（`src/control/`、`src/bench/` 未入库 + 5 份文档改动），
**不含**另一会话未提交的 `13-*.md` 与 `docs/reference/mocap-setup.md`。

#### ☠️☠️ 最重要的发现：整份文档的 ENU/NED 口径是**反的**（Spec 轴挖出，已第一手复核）

原文档（本张自己写的 `control/README.md`、代码注释、`control.yaml`，**以及既有的**
实施清单 §3.1/§3.3/§5.x 与 Phase 2 附录）都写着"位置是 EKF2 的 local NED，z 向下为正"。
**这是错的。** 读 mavros 源码：

- 出向：`local_position.cpp:157` 先 `transform_frame_ned_enu` 再发 ⟹ `/mavros/local_position/pose` 是 **ENU**；
- 入向：`setpoint_raw.cpp:208` 的 `local_cb` 把 `FRAME_LOCAL_NED` 的 `PositionTarget` **当 ENU 收**，
  在 **`:231`**（位置）与 **`:235`**（yaw）做 ENU→NED 再发 PX4。

⟹ **我们填进 `PositionTarget` 的是 ENU 口径**（z 向上为正），**`FRAME_LOCAL_NED` 只是协议枚举名**。
yaw 的换算按 `frame_tf.hpp:180` + `ftf_frame_conversions.cpp:33/40` **逐字复算**（不是手推四元数，
第一次手推算错了）：**`yaw_ned = π/2 − yaw_enu`** ⟹ 本项目填的 `yaw = 0`，PX4 收到的是 **+90°（机头朝东）**。
顺带得到一个安心结论：把 `local_position` 原样转给 setpoint **是对的** —— 一进一出恰好抵消。

⚠️ **对 ticket 08 的直接影响**：定"起飞朝向"之前必须处理 `yaw`，**不能留着默认 0 当"保持当前机头"**。
已按决策 ③**就地标注更正**（带源码路径 + 行号），未删改原结论。

#### 已修的代码问题

1. **守卫判据曾有两份**（Standards 轴：Duplicated Code + Feature Envy）：`fsm.hpp` 里的
   `&&` 级联与 `control_node.cpp::describe_missing()` 的 `if` 级联各写一遍 ⟹ 加一条守卫时
   极容易只改一处，症状是"状态机不进 STANDBY，但日志说前置条件都成立"。
   现改为 `fsm.hpp` 的 **`enum class Guard` + `all_guards()` + `guard_holds()` + `missing_guards()`**
   单一出处，节点侧只剩一份 `switch` 文案，且**故意不写 `default:`**（漏一条 ⟹ `-Wswitch` 编译失败）。
2. **`mode_request_in_flight_` 可能永远卡住**（Spec 轴）：服务端在请求悬空时消失，
   rclcpp **不会超时这个请求**，于是节点永久不再重试，而日志还在说"每 1.0 s 重试"。
   现加悬空请求丢弃（阈值 `max(2.0, 2×offboard_retry_s)`，比正常往返宽松得多）。
   **这一支没有 seam-1 覆盖**（要构造"服务端收下请求后消失"，夹具目前不做），
   已在 `control/README.md` 与 `control_node.cpp` 头注释里点名。
3. **删掉死代码**：`local_velocity` 的订阅（空 lambda，README 却声称"记录用"）与其参数
   `local_velocity_topic`、以及无人调用的 `Fsm::ever_streamed()`。
   夹具**仍然**发这条话题（它假的是 mavros，mavros 确实发），只是节点不再订阅。
4. **补上缺失的 seam 2 单测**（Spec 轴）：新增 `test/test_fsm.cpp`（12 例，零 ROS 依赖）——
   逐条掀掉守卫时**断言缺项清单恰好是那一条**（只断言"不放行"的话，"某条守卫恒假"也会通过）、
   缺项顺序、断流后前置流**重新计时**、`STANDBY` 下控制权仍是 `MANUAL`。
   还顺带钉住一条默认值的纪律：**全零 `Guards` 里"未解锁"是成立的**（`/mavros/state` 一帧没到时
   按未解锁处理，否则启动瞬间会伪造"异常解锁"告警）。

#### 判定为"记录了但本张不改"的

- **文档与实现的措辞不一致**：`/mocap/pose_enu` 曾写"只用于记录"，实际已是**守卫输入**；
  `rc_timeout_s` / `battery_timeout_s` 与对应订阅曾写"ticket 05 实现"，实际**只记账、不做判据**
  （判据分别归 ticket 06 / 09）。三处已按"订阅与参数先就位、判据后接"的口径改清。
- **`conftest.py` 在 `control` 与 `bench` 各有一份**：接受。两份的内容与演进理由不同，
  抽公共包会让"测试辅助"多出一个要发版的包，收益不抵复杂度。
- **spec 的两处过时口径已就地标注**（不改结论）：故事 12"按开始键前不发任何控制指令"
  与"切 OFFBOARD 需要先流 setpoint"互斥（决策 ① 采用后者，理由是飞机未解锁）；
  原型状态机图缺 `INIT`/`STANDBY` 且把"开始信号"当成进入 `AUTO` 的边沿。

#### 复核时的数字

`colcon test --packages-up-to control`：**36 例全绿** ——
`control` seam 1 九例 + seam 2 十二例，`bench` 夹具自检九例 + `timeline` 纯逻辑六例。
lint 失败数**不增**（见上文 172/176 两条口径）。
