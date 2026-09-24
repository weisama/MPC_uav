// control 的状态机 —— ★ 两个正交维度显式分开，不压成扁平枚举。
//
//   控制权：  MANUAL ──(本项目开始发会驱动飞机的指令)──▶ AUTO ──(RC 打杆 / 超时 / 异常)──▶ MANUAL
//   任务态：  INIT ─▶ STANDBY ─▶ TAKEOFF ─▶ HOVER ─▶ WAIT_START ─▶ TRACK ─▶ LAND
//                                                                    └──▶ RC_OVERRIDE（控制权被外部夺走）
//
// ★ 本文件**零 ROS 依赖**（自带 POD 类型，不 include 任何 ROS 头）—— 与 `mocap` 的
//   `mocap_rel_pose.hpp` 同一个理由：状态转移条件是"飞起来才发现错"的重灾区，
//   而它在话题层面没有诊断分辨率。ROS 边界（话题进出、服务调用）全在 `control_node.cpp`，
//   本文件只回答"给定这组守卫 + 这组失效判据，现在该处于哪个状态、该不该发指令"。
//
// ★★ 本张（ticket 06）的全部分量：**失效路径**。
//   · `Health` —— 边界层算好的失效判据（"坏消息"，与 `Guards` 的"放行条件"极性相反）
//   · `Degrade` —— 降级原因，名字直接进日志（`degrade=<reason>`，可 grep）
//   · 决策：任何失效 ⟹ **停发 setpoint**；**已解锁**时并请求 `AUTO.LAND`；
//     **打杆/模式被夺走** ⟹ 停发 + 交还控制权（`RC_OVERRIDE`）且**不抢回 OFFBOARD**
//   · 闩锁**可复位**（不是单向 latch）：见 `update()` 里 ① 的复位条件
//   ★★ "模式被夺走"要看**丢的是哪个模式**（2026-09-22 复核时定，理由见 `first_degrade`）：
//      OFFBOARD 只有在**我们正在流**时丢了才算夺权；`AUTO.LAND` 一旦丢就是夺权。
//      少了这条区分，"我们自己停发 ⟹ PX4 因 `COM_OF_LOSS_T` 自己退出 OFFBOARD"会被
//      误报成"有人接管"——在真机上那**必然**发生（判据 0.5 s、`COM_OF_LOSS_T` 1.0 s），
//      而夹具永远看不见（假 mavros 不假扮 PX4 的这一半，见 `CONTEXT.md`）。
//
// ★ 未解锁时"只是停发、不请求 LAND"是刻意的（用户 2026-09-22 裁定），两条第一手依据：
//   ① PX4 自己的打杆接管就**要求已解锁**（`Commander.cpp:2740` 是
//      `if (_arm_state_machine.isArmed())` 门，`:2743-2745` 才是打杆条件；
//      2026-09-22 复核时更正过一处 2729-2731 的错引）
//   ② `AUTO.LAND` 在 `mode_req_prevent_arming` 名单里（`mode_requirements.cpp:142`）
//      ⟹ 未解锁就切进去，台架上会**解不了锁**
//
// ★★ 本张（ticket 08）的全部分量：**第一次真正自主离地并稳定悬停**。
//   `STANDBY → TAKEOFF → HOVER → WAIT_START → LAND`，全部判据都是程序可判定的量：
//   · **起飞命令**只从 `request_takeoff()` 进来（ROS 侧是服务 `/control/takeoff`，
//     ticket 09 会用遥控器 ch9 的上升沿去调它）。**本文件不读遥控器**。
//   · **解锁**走 `/mavros/cmd/arming`（H.6 的 D3 = 程序解锁）；前置 = 已在 OFFBOARD
//     + setpoint 已在流（H.6 原文的两条）+ **遥控在线**（H.7.1，见 `update()` ⑦）。
//     ⚠️ **不重试**：应答被拒或无应答 ⟹ 放弃本次起飞、退回 STANDBY（用户 2026-09-22 裁定）。
//   · **爬升**：目标 z 从"进入 TAKEOFF 那一刻的 EKF 位置"线性上升 `takeoff_altitude_m`，
//     速率 `climb_speed_mps`。★ 起点是**锁一次**的，不是每帧跟着当前位姿跑 ——
//     否则"目标"会跟着飞机漂，"爬到 1 m"这句话就不成立了。
//   · **稳定**：`位置误差 < hover_pos_tol_m ∧ |v| < hover_vel_tol_mps` 持续 `hover_settle_s`。
//     ★ 位置误差相对的是**悬停点**（= 起飞点正上方 `takeoff_altitude_m`），
//     不是轨迹原点 —— 轨迹原点恰恰是**判稳那一刻**才记下来的（`traj_origin()`），
//     拿它当判据的基准是循环定义。判稳时飞机必然在悬停点 `hover_pos_tol_m` 之内，
//     所以记下来的原点与悬停点的差不超过一个判据宽度。
//   · **轨迹原点**记录后进 `WAIT_START`，目标切到原点本身（"停在原地"，见 `update()` ⑦）。
//   · **任务结束**：`WAIT_START` 保持 `wait_start_timeout_s` ⟹ `LAND`（请求 `AUTO.LAND`）。
//     **上锁不是我们发的**：PX4 的 `COM_DISARM_LAND = 2.0`（z0 实测值，ticket 15 对账）
//     会在它自己的着陆检测判定落地 2 s 后上锁 ⟹ 我们只等 `armed` 的降沿。
//   · **机头**：setpoint 的 yaw 一律 = **起飞机头**（STANDBY 时是当前机头，起飞后是锁定值）。
//     ⚠️ **这条推翻了 ticket 05 的 `yaw` 参数（已删）**：那个参数默认 0，而 0 在
//     ROS/ENU 口径下经 mavros 换算后是 PX4 的 +90°（机头朝东）⟹ 一旦在 OFFBOARD 里解锁，
//     PX4 会**真的去转机头**。本张按 §3.4 的精神改成"不转向"。
//
// ★★ 本张（ticket 09）加的部分：**`WAIT_START → TRACK` 与轨迹跟踪**。
//   · **开始键**只从 `note_start_edge()` 进来（ROS 侧是遥控器 ch9 的**上升沿**，
//     判据在 `rc_switch.hpp`）。**本文件不读遥控器、不数边沿** —— 它只回答
//     "这条边沿在当前任务态下算什么"。
//   · **边沿的用法按任务态判定，不按计数器**（`note_start_edge` 里有完整理由）：
//     `kIdle` ⟹ 受理起飞（§H.5.1 的"第 1 次"，D3 = 程序解锁）；`kWaitStart` ⟹ 开始轨迹
//     （"第 2 次"）；其余任务态 ⟹ **当场丢掉并记日志**（不排队 —— 与 `request_takeoff`
//     同一条纪律：排队会在操作员不看的某一刻自己飞起来）。
//   · **`TRACK` 的目标 = 轨迹原点 + `/traj/reference`**（参考是**相对轨迹原点**的，
//     ticket 07 的落地；漏掉原点会让飞机朝地面俯冲 —— ticket 09 的 Comments 逐字警告过）。
//   · **参考的 yaw 不转发**：`traj` 的 `yaw_rad` 默认 0，而 0 在 ENU 口径下是"机头朝东"
//     ⟹ 转发它会让飞机在半空中自己转 90°（与 ticket 05 的 `yaw` 参数同一个坑）。
//     TRACK 的 yaw 仍是**起飞机头**。
//   · **参考静默**（在 `TRACK` 里已经跑起来之后又停了）⟹ `Degrade::kTrajTimeout`：
//     停发 + 请求降落。理由见 `TaskLimits::wait_start_timeout_s` 与 `TrackInput`。
//   · **任务结束的两条路**：`progress ≥ 1.0`（轨迹真的飞完）与 `WAIT_START` 兜底超时
//     （操作员一直没按开始键）—— 动作完全相同（停发 + 降落），但**必须在日志里分开说**
//     （用 `EndReason`）：把后者说成"任务飞完了"是误导。

#ifndef CONTROL__FSM_HPP_
#define CONTROL__FSM_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace control
{

/// 控制权的定义再强调一次（README 里有完整版）：
/// **AUTO = 本项目正在发"会驱动飞机"的指令，即飞机已解锁、由我们通过 offboard 驱动。**
/// STANDBY 虽然已经切进 OFFBOARD 模式，但飞机**未解锁** ⟹ 谁都没在飞它 ⟹ 控制权仍是 MANUAL。
/// 这条定义让"切模式"与"交出控制权"两件事不被混为一谈。
enum class Authority
{
  kManual,
  kAuto,
};

enum class Phase
{
  kInit,        ///< 上电：等前置（mavros 连接 / 动捕有数据 / local_position 有效）；**不发 setpoint**
  /// 已开始以 `setpoint_rate_hz` 持续发布 setpoint（目标 = 当前位置、yaw = 当前机头）。
  /// ★ **收到起飞命令但还没解锁的期间也是它** —— 那时飞机仍未被我们驱动。
  kStandby,
  kTakeoff,     ///< 已解锁且在 OFFBOARD，目标 z 沿斜坡上升（ticket 08）
  kHover,       ///< 到高后定点保持，同时判稳（ticket 08）
  /// 已判稳、**轨迹原点已记录**，目标 = 原点本身，等操作员按遥控器开始键。
  /// ★ ticket 09：**这个态的出口有两条** —— 开始键的上升沿 ⟹ `TRACK`；
  ///   兜底超时 `wait_start_timeout_s` ⟹ 任务结束进 `LAND`（`EndReason::kWaitStartTimeout`）。
  ///   两条出口的动作完全不同（一个继续飞、一个降落），所以日志必须说清是哪一条。
  kWaitStart,
  /// 跟着 `/traj/reference` 飞（ticket 09）：目标 = **轨迹原点 + 参考**，
  /// `progress ≥ 1.0` ⟹ 任务飞完（`EndReason::kTrajDone`）⟹ `LAND`。
  /// ⚠️ 参考的 `yaw` **不转发**（理由见文件头 ticket 09 那一节）。
  kTrack,
  /// 停发 + 请求 `AUTO.LAND`。三个来源：**失效降级**（ticket 06）、**轨迹飞完**（ticket 09）、
  /// **`WAIT_START` 兜底超时**（ticket 09）。处置逐字相同，靠 `Decision::end_reason` +
  /// `Decision::degrade` 区分（日志要说清是哪一种）。
  kLand,
  kRcOverride,  ///< 控制权被外部夺走（ticket 06）：停发 + 交还控制权 + **不抢回 OFFBOARD**
};

const char * to_string(Authority a);
const char * to_string(Phase p);

/// 守卫 —— 由 ROS 边界（`Px4Interface`）算好"当前是否成立"，本文件不碰话题、不碰时钟。
struct Guards
{
  bool connected{false};        ///< `/mavros/state` 在 `state_timeout_s` 内且 `connected == true`
  bool mocap_fresh{false};      ///< 动捕话题在 `mocap_timeout_s` 内到过一帧
  bool local_pos_fresh{false};  ///< `/mavros/local_position/pose` 在 `local_pos_timeout_s` 内到过一帧
  bool armed{false};            ///< `/mavros/state.armed`
  bool in_offboard{false};      ///< `/mavros/state.mode == "OFFBOARD"`
  /// `/mavros/state.mode` == **我们请求的中止任务模式**（`AUTO.LAND`）。
  /// 与 `in_offboard` 一起构成"当前模式是不是我们要的"（见 `mode_is_ours`）——
  /// 少了它，我们自己请求 `AUTO.LAND` 成功之后模式的变化会被误判成"被外部夺走"。
  bool in_land{false};
  /// ★ 本次解锁是**本节点发起的**（ticket 08 起由 `Px4Interface` 真实追踪，不再是参数）。
  ///   追踪方式见 `px4_interface.hpp` 的 `request_arming()`：只有拿到 `/mavros/cmd/arming`
  ///   的**肯定应答**（`MAV_RESULT::ACCEPTED`）之后观察到的第一个 `armed` 上升沿才算我们的。
  ///   ⚠️ 中间可能夹几帧 `armed == false`（`/mavros/state` 只有 1 Hz，而应答是即时的）
  ///   ⟹ "看到 false 就清掉请求标记"会把**我们自己的**解锁误报成外部解锁（§H.6 的告警）。
  bool own_arming{false};
};

/// 守卫的**名字**。★ 这是"哪些条件算前置"的唯一出处：
/// `ready_to_stream()` 的放行判据与节点的告警清单都从 `missing_guards()` 取。
/// 为什么要这样：这两处曾经各写一遍 `&&`/`if` 级联，**加一条守卫时极容易只改一处** ——
/// 结果是"状态机不进 STANDBY，但日志说前置条件都成立"，那种 bug 在实飞现场是查不下去的。
enum class Guard
{
  kConnected,     ///< 飞控已连接（`/mavros/state` 新鲜且 `connected`）
  kMocapFresh,    ///< 动捕有数据
  kLocalPosFresh, ///< `local_position` 有效
  /// 飞机的解锁（如果有）来自本节点 —— **未解锁时恒成立**。
  /// 语义 = "不许出现『别人解锁了』这种情况"：§H.6 要求外部解锁 ⟹ 停发 + 告警。
  /// 由 `Guards::own_arming` 提供（ticket 08 起是真实的解锁追踪，不再是参数）。
  /// ⚠️ 它**不在** `ready_to_stream()` 里当真值用（未解锁恒成立，所以放行判据不受影响），
  /// 但它是 `AUTO` 的必要条件：`guards.armed` 为真而这条不成立 ⟹ 停发 + 退回 INIT。
  kArmingByUs,
};

/// 当前模式是不是**我们要的那个**（OFFBOARD，或我们请求过的 `AUTO.LAND`）。
/// ★ 这条区分是有代价才换来的：我们自己请求 `AUTO.LAND` 成功的那一刻，`in_offboard` 也
///   会从真变假 —— 只看 `in_offboard` 的话，**我们自己的降落请求会被当成"被外部夺走"**，
///   于是状态机一边降落一边把控制权"交还"给不存在的飞行员，还会停止重试降落请求。
inline bool mode_is_ours(const Guards & g)
{
  return g.in_offboard || g.in_land;
}

/// 守卫的**全部成员**。★ 只有这一处列举 ⟹ 加一条守卫时不必去找散落的 `&&` 级联。
inline const std::vector<Guard> & all_guards()
{
  static const std::vector<Guard> kAll{
    Guard::kConnected, Guard::kMocapFresh, Guard::kLocalPosFresh, Guard::kArmingByUs};
  return kAll;
}

/// 单条守卫当前是否成立。★ **故意不写 `default:`** —— 新增一条 `Guard` 而这里没跟上时，
/// `-Wswitch` 会把它变成编译错误（这正是本条"唯一出处"想买到的保险）。
inline bool guard_holds(const Guards & g, Guard which)
{
  switch (which) {
    case Guard::kConnected:     return g.connected;
    case Guard::kMocapFresh:    return g.mocap_fresh;
    case Guard::kLocalPosFresh: return g.local_pos_fresh;
    case Guard::kArmingByUs:    return !g.armed || g.own_arming;
  }
  return false;  // 不可达：上面的 switch 已覆盖全部枚举
}

/// 当前**不成立**的守卫（顺序同 `all_guards()`）。放行判据与告警文案共用它。
inline std::vector<Guard> missing_guards(const Guards & g)
{
  std::vector<Guard> missing;
  for (Guard which : all_guards()) {
    if (!guard_holds(g, which)) {
      missing.push_back(which);
    }
  }
  return missing;
}

/// 放行判据 = 「四条守卫全成立」。
/// 判据与 `missing_guards()` 同源（同一个 `all_guards()` + `guard_holds()`，只是不收集清单）——
/// 这条谓词每帧都跑（50 Hz），所以不给它一次堆分配。
inline bool ready_to_stream(const Guards & g)
{
  for (Guard which : all_guards()) {
    if (!guard_holds(g, which)) {
      return false;
    }
  }
  return true;
}

/// 失效判据 —— 边界层算好的"坏消息"。**极性与 `Guards` 相反**（true = 出事了），
/// 所以刻意分成两个结构体：混在一起会让"守卫成立"与"判据成立"的读法互相打架。
/// 阈值全部在边界层（`control.yaml`）里，本文件只看布尔量。
struct Health
{
  bool rc_fresh{false};             ///< `/mavros/rc/in` 在 `rc_timeout_s` 内到过一帧
  /// 摇杆变化率超阈值（与固件的 `COM_RC_STICK_OV` 同源，见 `rc_stick.hpp`）。
  /// ★ 只在**已解锁**时才当失效用（PX4 亦然，`Commander.cpp:2740` 的解锁门）。
  bool rc_sticks_moving{false};
  /// 电压低于 `low_voltage_v`。★ **"没收到过电池"必须是 false** —— 把"没数据"当成
  /// "低压"会让没接电池的台架上永远在降落（与"没收到 state 就当成未解锁"同一条纪律）。
  bool battery_low{false};
  /// `local_position` 出现 NaN/Inf（EKF2 位置失效）。
  bool position_invalid{false};
  /// 动捕**有帧但时间戳不前进**（冻住/倒退）：mavros 会丢弃这类 vision_pose
  /// （`send_vision_estimate()` 里戳相等即 return，§3.5），EKF2 也会丢过旧的样本
  /// ⟹ 话题层面"有数据"，EKF2 眼里"没数据"。
  bool mocap_stamp_stalled{false};
  /// ★★ ticket 16：**EKF2 自己说"我的水平位置没有绝对参考"** —— 载体是
  /// `/mavros/estimator_status.pos_horiz_abs_status_flag`，边界层要求它**连续为假**若干帧
  /// 才算成立（消抖，帧数在 `control.yaml`）。
  ///
  /// 这是验收标准 ④「位置失效 ⟹ LAND」在**本链条上真能兑现**的那一条：
  /// `position_invalid` 那条（NaN）实测恒假 —— PX4 的 `PublishLocalPosition` **不合成 NaN**
  /// （`EKF2.cpp` 只发状态位），`LOCAL_POSITION_NED` 又把 `xy_valid` 丢掉 ⟹ 失效信息
  /// 到不了 mavros。而本字段在 PX4 侧就是 `(cs.gps || cs.ev_pos) && _fault_status.value == 0`
  /// （`EKF/ekf_helper.cpp:761`），**每次发布现算**（⟹ 构造上就不锁存，会自清）。
  ///
  /// ⚠️ 它**只在已解锁时**才当失效用（同 `rc_sticks_moving` 的解锁门，理由见 `first_degrade`）。
  /// ⚠️ 它不是 ④ 的兜底 —— 真兜底是动捕那两条（`mocap_timeout` / `mocap_stamp_stalled`）：
  ///    EKF2 自己清 `ev_pos` 是 **400 ms**（样本停了，`EKF/ev_control.cpp:75`）或 **1 s**
  ///    （样本照到、但没被融合，`EKF/ev_pos_control.cpp:256`），再加上**发布相位（1 Hz）**
  ///    与本判据的 **2 帧消抖** ⟹ 到我们手上 ~2.5–3 s，而动捕那两条是 0.5 s。
  ///    它值钱的地方是**另一类**故障 —— 帧正常到达、EKF2 却拒绝用（融合失败、`_fault_status`
  ///    非零）：那种场面动捕那两条**完全看不见**。两条路径的源码出处见
  ///    `docs/control/phase1-implementation.md` §4.4。
  bool ekf2_pos_lost{false};
  /// ★★ ticket 09：在 `TRACK` 里**等参考等太久**（自"进 `TRACK` / 上一次收到参考"起超过
  /// `TaskLimits::traj_timeout_s`）⟹ 停发 + 请求降落。
  ///
  /// ⚠️ **本字段由 `Fsm::update()` 填，不由边界层填** —— 这是本结构体里唯一的例外，
  ///   理由必须写清楚：判据要的不是"话题静默了多久"，而是"**我们向 `traj` 要了流之后，
  ///   它多久没给**"。两者的计时起点不同：
  ///     · 刚进 `TRACK` 时参考**本来就还没开始发** —— `traj` 要等我们的 `/traj/start`
  ///       才发第一帧（`traj_node.cpp:289`：`running_` 为假就一帧都不发）。拿"话题静默"
  ///       直接当判据，会在进 `TRACK` 的**第二帧**就判失效（`fail-fast` 在 20 ms 的
  ///       正常等待窗里就是误判）。
  ///     · 边界层也不知道"重发 `/traj/start`"这件事（运行中重选轨迹之后要再发一次，
  ///       发出去之后同样要给一个新窗口）。
  ///   ⟹ 计时起点（进 `TRACK`、每次重发、每次收到参考）只有任务层知道，字段就在这里填。
  ///   ★ 起点与边界层的 `traj_reference` 新鲜度**同时**到期（两者都从"最后一帧参考"起算），
  ///     所以不会叠加成两个超时。
  bool traj_stale{false};
};

/// 本帧的**导航读数**（ticket 08）—— 起飞与悬停判据要的量的最小集合。
///
/// ★ 与 `Guards`/`Health` 同一条纪律：**边界层算好，本文件只做几何**。
/// ★★ 位置/速度/机头**全都已经在 EKF2 的 local 系里**（ROS 侧 ENU，z 向上为正），
///    与 setpoint 同口径 ⟹ 生成目标**不需要任何坐标转换**。这是本张能这么薄的原因。
///    两个来源分别是 `/mavros/local_position/pose` 与
///    `/mavros/local_position/velocity_local`；**后者也是 ENU**（已核对 mavros 源码：
///    `local_position.cpp:210` 在**同一个回调**里发它，速度经 `transform_frame_ned_enu`）。
///
/// ⚠️ `position_valid` 一条管**位置与姿态两样**：它们来自同一条 `PoseStamped`
///    ⟹ 不会一个有一个没有。速度是**另一条话题**（另一条订阅），启动时可能还没到，
///    所以 `velocity_valid` 单列。判据一律要求"量本身有效"才用，见 `update()`。
struct Odometry
{
  bool position_valid{false};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  bool velocity_valid{false};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  /// 机头朝向（ROS/ENU：从 +x 逆时针，与 `PoseStamped` 的四元数同系）。
  double yaw{0.0};

  /// 三维位置偏差（相对给定点；ENU 口径 ⟹ 就是朴素的欧氏距离）。
  double distance_to(double ox, double oy, double oz) const
  {
    const double dx = x - ox, dy = y - oy, dz = z - oz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  /// 三维速率。⚠️ 判稳用的是**三维**（`|v| < 0.10 m/s`），不是只看水平分量。
  double speed() const
  {
    return std::sqrt(vx * vx + vy * vy + vz * vz);
  }
};

/// 本帧的**轨迹参考**（ticket 09）—— `/traj/reference` + `/traj/progress` 的最小集合。
///
/// ★★ **口径**：参考是**相对轨迹原点**的（ENU，单位 m），**1 m 高度在原点里、不在轨迹里**
///    （悬停轨迹的参考恒为 `(0,0,0)`，ticket 07 的落地）。所以 `TRACK` 的目标 = 轨迹原点 + 参考，
///    **绝不能**把它直接当 setpoint 发出去 —— 那是把原点丢掉，飞机会先往地面附近扎
///    （ticket 09 的 Comments 第 1 条逐字警告过）。
///
/// ★ 与 `Odometry` 同一条纪律：**边界层算好，本文件只做几何**。这里刻意**不**带新鲜度窗口，
///   只报"上一帧参考是多久以前到的"（`age_s`）—— 因为"多久算失效"取决于"我们什么时候开始要流"
///   （见 `Health::traj_stale`），那是任务层的事，边界层算不出来。
struct TrajReference
{
  /// 距上一帧 `/traj/reference` 的时长（秒）。**从没收到过时是 `+inf`**。
  /// ★ 用"年龄"而不是"是否新鲜"：本文件要拿它同时判两件事（流断没断、要不要重发
  ///   `/traj/start`），两者阈值不同（`traj_stream_gap_s` vs `traj_timeout_s`），
  ///   而边界层一旦替我折成布尔量就再也拆不开了。
  double age_s{std::numeric_limits<double>::infinity()};
  /// 参考位置（**相对轨迹原点**，ROS/ENU）。
  double x{0.0};
  double y{0.0};
  double z{0.0};
  /// 上一次收到的 `/traj/progress`（0–1）。**跑完之后 `traj` 会一直保持 1.0**。
  /// ⚠️ 它是"最后一次收到的值"，流断的时候**不会**自己归零 ⟹ 判"跑完了"必须配合
  ///   `progress_seen` 与 `track_started_` 两个闩锁（否则上一轮任务留下的 1.0 会让
  ///   刚进 `TRACK` 的这一轮**当场**判成飞完，见 `update()` 的 `kTrack` 分支）。
  double progress{0.0};
  /// 收到过至少一帧 `/traj/progress`。
  bool progress_seen{false};

  /// 参考流"还活着"（年龄在 `gap_s` 之内）。★ 是本文件的小工具，不是边界层的判据。
  bool alive(double gap_s) const {return age_s <= gap_s;}
};


/// 从四元数取 **ROS/ENU 口径**的偏航角（从 +x 逆时针，绕 +z）。
/// ★ 放这里而不是边界层：它是纯函数（seam 2 可测），而 yaw 的正确性在实机上
///   代价很高（ticket 05 的 `yaw` 参数就是这么错的 —— 见文件头 ★）。
/// ⚠️ 返回的是 **ENU** 口径。mavros 对 `FRAME_LOCAL_NED` 的 setpoint 做 ENU→NED 换算，
///   所以**我们填 ENU、mavros 换**，口径与本函数一致，不要在这里预先 `π/2 − yaw`。
inline double yaw_from_quaternion(double qw, double qx, double qy, double qz)
{
  // 标准 yaw（ZYX 序）：atan2(2(qw·qz + qx·qy), 1 − 2(qy² + qz²))。
  return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

/// 任务判据的阈值（ticket 08）。
///
/// ★ **与失效判据的 `Limits`（在 `px4_interface.hpp`）刻意分开**：那一组管"什么时候停"，
///   这一组管"飞到什么算到"。合在一起之后，调悬停精度会顺手把低压阈值也改了。
/// ★ 放在本文件（零 ROS）而不是边界层：它们全是纯几何/时长，边界层只负责把 yaml 里的数搬进来。
struct TaskLimits
{
  /// 爬升目标高度（相对**进入 TAKEOFF 那一刻的 EKF 位置**，m）。工单：1 m。
  double takeoff_altitude_m{1.0};
  /// 爬升速率上限（m/s）。工单：≤0.5 m/s。
  double climb_speed_mps{0.5};
  /// `TAKEOFF → HOVER` 的高度容差（m，工单 0.10）。
  double takeoff_alt_tol_m{0.10};
  /// `TAKEOFF → HOVER` 的垂速容差（m/s，工单 0.10）。
  double takeoff_vel_tol_mps{0.10};
  /// 上面两条同时成立需持续多久才算"到高"（工单 1 s）。
  double takeoff_settle_s{1.0};
  /// 稳定判据：位置误差上限（m，工单 `< 0.10 m`）。
  double hover_pos_tol_m{0.10};
  /// 稳定判据：速率上限（m/s，工单 `< 0.10 m/s`）。
  double hover_vel_tol_mps{0.10};
  /// 稳定判据需持续多久（工单 2 s）。
  double hover_settle_s{2.0};
  /// `WAIT_START` 的**兜底时限**：这么久还没等到开始键就结束任务、自动降落。
  /// ★★ ticket 09 取代了 ticket 08 的 `hover_hold_s`（那个数**就是这张票要重新审的**，
  ///   见它删掉前的注释）：`WAIT_START` 是一个**等操作员**的态，60 s 按不到就降落等于
  ///   "手慢就白飞一趟"。300 s 是给"起飞后还要切图/看 RViz/确认轨迹"留的时间。
  /// ⚠️ 它的**动作是降落**，不是"继续等" —— 与 §5.3 的"悬停 60 s"不是一回事：
  ///   后者由 **1 m 悬停轨迹**（`traj` 的 `hover_duration_s = 60 s`）在 `TRACK` 里兑现。
  /// ★ `<= 0` = **不超时**（一直等操作员）。台架 pytest 用的就是这一档 ——
  ///   否则"起飞 → 判稳 → 60 s 后降落"的既有用例要等 5 分钟。
  double wait_start_timeout_s{300.0};
  /// `/traj/reference` **断流**的判据（秒）：超过它没有新参考
  ///   · 目标**不再更新**（保持上一个目标 —— 飞机停在原地，而不是回落到轨迹原点）；
  ///   · 并且**重发一次 `/traj/start`**（运行中重选轨迹后 `traj` 会自己停流，
  ///     见 `update()` 的 `kTrack` 分支）。
  /// ★ 取 0.5 s 而不是"几个周期"：它是**重发 `/traj/start`** 的闸门，而重发这个动作
  ///   在"其实还在跑"的情况下是一次**真实的指令跳变**（`traj` 会把参考时钟归零，
  ///   飞机会掉头飞回轨迹起点）⟹ 必须大到"不可能是调度抖动"。
  ///   `traj` 发参考是 50 Hz（周期 20 ms）⟹ 0.5 s = **连丢 25 帧**，台架上不会发生。
  ///   ⚠️ **不要把它调小**：调到两三个周期，任何一次调度延迟都会让飞机掉头。
  double traj_stream_gap_s{0.5};
  /// 进 `TRACK` 之后（或上一次收到参考之后）这么久还没参考 ⟹ **放弃并降落**
  ///   （`Degrade::kTrajTimeout`）。取 3.0 的理由：正常一次 `/traj/start` 的往返是
  ///   **20 ms 级**（`traj` 下一个 50 Hz tick 就发），给到 3 s 已经是两个数量级的余量；
  ///   而"操作员按了开始键但没选轨迹"（`traj` 只打一条 WARN、一帧都不发）必须有个头
  ///   —— 否则飞机会一直悬在那里，而**程序没有任何出口**（那时飞机仍在我们手上，
  ///   §H.7 的打杆接管还在，但那要操作员自己发现问题）。
  double traj_timeout_s{3.0};
  /// 起飞命令的**有效期**：受理之后这么久还没解锁成功就作废、退回 STANDBY（不自动重试）。
  /// 取 5.0 的理由：`/mavros/state` 只有 1 Hz（解锁成功最多晚 1 s 才知道）+
  /// `/mavros/cmd/arming` 的服务往返 + 余量。⚠️ 它**也是**"前置不满足"的兜底时限
  /// （前置齐了才会发解锁请求，见 `update()` ⑦）。
  double takeoff_timeout_s{5.0};
};

/// 降级原因。★ `to_string()` 的字符串**直接进日志**（`degrade=<reason>`）⟹ 稳定、可 grep、
/// 不要随手改（测试也断言它们）。
enum class Degrade
{
  kNone,
  kRcOverride,         ///< 摇杆在动（已解锁）⟹ 飞行员接管
  /// 我们**正在驱动**的模式被外部切走了（OFFBOARD 且当时在流，或我们请求过的 `AUTO.LAND`）。
  /// ⚠️ 名字是 `mode_lost` 而不是 `offboard_lost`：**我们自己请求降落**造成的 OFFBOARD→AUTO.LAND
  ///    切换不算夺权（见 `mode_is_ours`），所以这条判据管的是"我们持有的模式"，不是一个模式名。
  /// ⚠️ 也**不包括**"我们自己停发之后 OFFBOARD 掉了"—— 那是 PX4 自己的断流保护，见 `first_degrade`。
  kModeLost,
  kStateTimeout,       ///< `/mavros/state` 静默 ⟹ 飞控失联
  kMocapTimeout,       ///< 动捕静默
  kLocalPosTimeout,    ///< `local_position` 静默
  kRcTimeout,          ///< 遥控器静默（**只在已解锁时算失效**，见 `first_degrade`）
  kBatteryLow,         ///< 电压低于阈值
  kPositionInvalid,    ///< `local_position` 出现 NaN/Inf
  kMocapStampStalled,  ///< 动捕时间戳不前进
  /// EKF2 自报"水平位置没有绝对参考"（`pos_horiz_abs` 连续为假，见 `Health::ekf2_pos_lost`）。
  /// ⚠️ 排在 `mocap_stamp_stalled` **之后**：戳冻住是原因、本判据是它的后果
  ///   （戳不前进 ⟹ EKF2 自己清 `ev_pos`（400 ms / 1 s，见 `Health::ekf2_pos_lost`）
  ///   ⟹ `pos_horiz_abs` 变假），
  ///   报后者会把排查方向指到估计器上，而问题在上游的戳。
  /// ⚠️ 名字里的 `ekf2_` 是**故意的**：日志要靠它把"EKF2 说没位置"与"话题上没数据"
  ///   （`local_pos_timeout`）分开 —— 两者的排查方向完全不同。
  kEkf2PosLost,
  /// ★★ ticket 09：`TRACK` 里**参考流没了**（`traj` 节点挂了 / 没选轨迹 / `/traj/start`
  /// 被拒），且**重发 `/traj/start` 也没救回来**（判据 = `Health::traj_stale`）。
  /// ⚠️ 排在最后：它只在 `TRACK` 里成立，而进 `TRACK` 本身就要求前面那些判据都是好的
  ///   —— 真要同帧共现（比如动捕和参考同时断），钱都花在"先查动捕"上更对。
  kTrajTimeout,
};

const char * to_string(Degrade d);

/// 降级的**全部成员**（唯一出处，同 `all_guards()`）：日志文案与测试的完备性断言都取它。
inline const std::vector<Degrade> & all_degrades()
{
  static const std::vector<Degrade> kAll{
    Degrade::kNone, Degrade::kRcOverride, Degrade::kModeLost, Degrade::kStateTimeout,
    Degrade::kMocapTimeout, Degrade::kLocalPosTimeout, Degrade::kRcTimeout, Degrade::kBatteryLow,
    Degrade::kPositionInvalid, Degrade::kMocapStampStalled, Degrade::kEkf2PosLost,
    Degrade::kTrajTimeout};
  return kAll;
}

/// "控制权被夺走"这一类降级（与"输入坏了"分开：它们的处置完全不同 —— 前者交还控制权、
/// 不抢回 OFFBOARD；后者停发 + 请求 LAND）。
inline bool is_takeover(Degrade d)
{
  return (d == Degrade::kRcOverride) || (d == Degrade::kModeLost);
}

/// 上一帧的三个事实（判"夺权"要的是**下降沿**，而下降沿的原因取决于上一帧我们在干什么）。
/// 打包成一个结构体而不是三个 `bool` 参数：它们总是一起出现，散着传极易漏传或传错序。
struct PrevFrame
{
  bool mode_ours{false};   ///< 上一帧的模式是不是我们要的（`mode_is_ours`）
  bool streaming{false};   ///< 上一帧我们是否**正在流 setpoint**（⟹ 我们正在驱动这个模式）
  bool in_land{false};     ///< 上一帧是不是已在我们请求的 `AUTO.LAND` 里
};

/// 本轮该报哪个降级原因（同时成立时报**第一个**）。顺序不随意 —— 见实现里的逐条理由。
inline Degrade first_degrade(const Guards & g, const Health & h, const PrevFrame & prev)
{
  // ① 夺权类最优先：它们说明"控制权已经不在我们手上"，比任何输入故障都更根本。
  if (h.rc_sticks_moving && g.armed) {return Degrade::kRcOverride;}
  // ★ 用 `mode_is_ours` 而不是 `in_offboard`：我们**自己**请求 AUTO.LAND 成功时的模式切换
  //   不是"被夺走"（否则降落中途会把自己判成夺权）。
  // ★★ 还要看**丢的是哪个模式**（2026-09-22 复核补）：
  //   · OFFBOARD + 上一帧在流 ⟹ 我们正驱动着它，掉了只可能是外部 ⟹ 夺权。
  //   · OFFBOARD + 上一帧**没在流** ⟹ 那多半是**我们自己停发**的后果：PX4 断流超
  //     `COM_OF_LOSS_T`（1.0 s）就自行退出 OFFBOARD。把它报成"有人接管"有两个具体后果：
  //     ① 日志把排查方向指向不存在的飞行员（现场最贵的东西）；
  //     ② `is_takeover` 会把已闩锁的 `kArmedDegrade` **升级**成夺权 ⟹ 停掉 `AUTO.LAND` 重试，
  //        而"输入坏了就让它降落"正是 ticket 06 要的；未解锁时还会把"判据恢复即自动复位"
  //        变成"等有人把模式放回来"（真机上必然发生：判据 0.5 s < `COM_OF_LOSS_T` 1.0 s）。
  //   · `AUTO.LAND` 丢了 ⟹ 永远算夺权：那是我们主动要来的模式，断流不会让它退出，
  //     所以在我们没在流时它掉出来只可能是人干的（这条保住"降落中别跟人抢"）。
  //   ⚠️ 代价（刻意接受）：人在我们降级停发期间拨走了 OFFBOARD，我们不会立刻知道 ——
  //     要靠**打杆**那条判据（飞行员真要接管飞机，手一定在杆上；PX4 的 `COM_RC_OVERRIDE`
  //     同样要求打杆）。两条都成立时才升级，见 `update()` ③。
  if (prev.mode_ours && !mode_is_ours(g) && (prev.streaming || prev.in_land)) {
    return Degrade::kModeLost;
  }

  // ② ticket 06 点名的那三条输入超时。
  if (!g.connected) {return Degrade::kStateTimeout;}
  if (!g.mocap_fresh) {return Degrade::kMocapTimeout;}

  // ③ `position_invalid` 排在 `local_pos_timeout` **之前**：NaN 的那一帧就已经是失效
  //    （边界层对 NaN 帧不刷新新鲜度），0.5 s 之后两者会同时成立 —— 报"超时"就丢了根因。
  if (h.position_invalid) {return Degrade::kPositionInvalid;}
  if (!g.local_pos_fresh) {return Degrade::kLocalPosTimeout;}

  // ④ 遥控器静默只在**已解锁**时算失效：未解锁时链路断着不影响任何事，而一旦解锁
  //    （ticket 08 的起飞前），它就是"起飞前遥控必须开机且链路已建立"（§H.7.1）的执行者。
  if (g.armed && !h.rc_fresh) {return Degrade::kRcTimeout;}

  // ⑤ 低压是"还能飞但该停"，动捕戳不前进是"上游数据坏了但还没丢帧" —— 都排在最后。
  if (h.battery_low) {return Degrade::kBatteryLow;}
  if (h.mocap_stamp_stalled) {return Degrade::kMocapStampStalled;}

  // ⑥ ticket 16：EKF2 自报水平位置没有绝对参考（`pos_horiz_abs` 连续为假）。
  //   ★ 排在 ⑤ 之后是**因果顺序**：戳冻住是因、这条是果（戳停 ⟹ EKF2 自己清 `ev_pos`），
  //     同帧两条都成立时报上游那条。
  //   ★★ 与 `rc_timeout` 同款的**解锁门**（`g.armed`）——理由与它不同，必须写清楚：
  //     · 未解锁时这条判据的代价是**停发 setpoint**，而冷启动时"EKF2 还没把位置锚到动捕上"
  //       是**常态**（`ev_pos` 要等 vision 真的被融合起来，`pos_horiz_abs` 从假变真）⟹
  //       不加门会在这个正常窗口里停流，把"准备起飞"变成"节点不发东西了"。
  //       （用户 2026-09-22 裁定：不做。台架上每次冷启动都会撞上这段窗口。）
  //     · 已解锁时它才是"④ 位置失效 ⟹ LAND"的执行者：那时停下比继续飞安全。
  //   ⚠️ 代价（刻意接受，两道门都要说清）：
  //     1) 未解锁时 EKF2 真坏了我们**不报** —— 那时没有任务可中止，报的代价（停发）比收益大。
  //     2) 于是"解锁前就坏了"的情形只能等**解锁之后**才被抓住（第一帧起就成立，
  //        再等消抖的 2 帧 ≈ 2 s）。这段时间里飞机是解锁状态而位置参考已经没了 ——
  //        所以真兜底仍然是动捕那两条（0.5 s 就报，且不看解锁状态）。
  if (g.armed && h.ekf2_pos_lost) {return Degrade::kEkf2PosLost;}

  // ⑦ ticket 09：`TRACK` 里参考流没了（`traj` 挂了 / 没选轨迹 / `/traj/start` 被拒）。
  //   ★★ 只有这一条判据**不带 `g.armed` 门**，反而要看 `g.armed` 的另一面 —— 它只在
  //     `TRACK` 里成立，而进 `TRACK` 必须先判稳、先解锁（`kWaitStart` 是解锁之后才到的），
  //     所以"未解锁时误报"在这条上构造不出来。判据本身（含计时起点）见 `Health::traj_stale`。
  //   ★ 排在最后而不是最前：它对"飞机是不是安全"最不紧急（参考没了，飞机还悬在原位、
  //     动捕和 EKF2 都还好好的），而同帧共现时前面的每一条都更需要先查。
  if (h.traj_stale) {return Degrade::kTrajTimeout;}
  return Degrade::kNone;
}

/// 任务**正常结束**的原因（ticket 09，取代 ticket 08 的 `Decision::mission_complete`）。
///
/// ★ 为什么值得单开一个枚举而不是一个 bool：`WAIT_START` 的兜底超时与"轨迹真的飞完了"
///   **动作逐字相同**（停发 + 请求降落），但两句日志的**意思完全相反** —— 把"操作员一直
///   没按开始键"说成"任务飞完了"是误导，而现场的判断几乎全靠这条日志。
/// ⚠️ 它**只覆盖"没出故障的结束"**。失效降级（ticket 06 那一家子）结束任务时这一栏是
///   `kNone`，因为那时该看的是 `degrades`（`Decision::degrade`）—— 两条信息刻意不混。
enum class EndReason
{
  kNone,             ///< 没有结束（在飞，或者结束的原因不是"正常走完"，见上）
  kTrajDone,         ///< `TRACK`：`progress ≥ 1.0`，轨迹真的跑完了
  kWaitStartTimeout, ///< `WAIT_START`：一直没等到开始键，兜底时限到了
  /// ★★ ticket 07 增补（2026-09-24 用户裁定）：**长按开始键 ≥ `start_land_hold_s`** ⟹
  ///   操作员要求**立即**降落。它与上面两条**动作逐字相同**（停发 + 请求 `AUTO.LAND`），
  ///   所以走同一个出口；差别只在日志措辞 —— 现场看到的"飞机自己降落了"必须能一眼分成
  ///   "飞完了""等不到键""操作员按的"三件事。
  kLandPressed,
};

const char * to_string(EndReason r);

/// 决策 —— 由 ROS 边界执行：边界层不判断，只按这几个字段行事。
struct Decision
{
  Phase phase{Phase::kInit};
  Authority authority{Authority::kManual};
  bool stream_setpoint{false};   ///< 是否发布 setpoint（INIT 与 STANDBY 的唯一区别）
  bool request_offboard{false};  ///< 是否该请求 OFFBOARD（已连续流够时长、且当前不在 OFFBOARD）
  /// 是否该请求 `AUTO.LAND`（主动中止任务）。★ 只在**已解锁且非夺权**的降级里为真。
  bool request_land{false};
  /// 本轮的降级原因（`kNone` = 没有降级）。★ 直接进日志：`degrade=<to_string(...)>`。
  Degrade degrade{Degrade::kNone};

  // ────────── ticket 08 ──────────

  /// 本帧该发**一次**解锁命令（`/mavros/cmd/arming`）。★ 一次性脉冲：只有决定解锁的
  /// 那一帧为真（边界层照旧"不判断"，只管发；重发/重试的语义在 FSM 里，本张刻意**不重试**）。
  bool request_arming{false};
  /// 本帧 setpoint 的**目标位置**（ROS/ENU，z 向上为正）。`stream_setpoint` 为真时才有意义。
  /// ★ 目标由状态机算（爬升斜坡、悬停点、轨迹原点），**不再由边界层拿"当前位姿"顶替** ——
  /// 那是 ticket 05 的写法，而"目标"这个概念在 ticket 08 才第一次与"当前"分家。
  double target_x{0.0};
  double target_y{0.0};
  double target_z{0.0};
  /// 本帧 setpoint 的 yaw（ROS/ENU）。★ 起飞前 = 当前机头（不转向），起飞后 = 锁定值。
  double target_yaw{0.0};
  /// 进 `LAND` 的原因是"任务正常走完"而不是失效降级（日志用它区分两种降落）。
  /// ★ 与 `degrade == kNone` 是**两回事**：正常走完时 `degrade` 也是 `kNone`
  ///   ⟹ 只用 `degrade` 判断会把"任务飞完了"漏成"什么都没发生"。
  EndReason end_reason{EndReason::kNone};

  // ────────── ticket 09 ──────────

  /// 本帧该发**一次** `/traj/start`（`std_msgs/Empty`，`traj` 收到就把参考时钟归零）。
  /// ★ 一次性脉冲，且**只可能出现在 `TRACK`**：
  ///   · 进 `TRACK` 的第一帧必发（"开始飞"与"参考时钟零点"必须是同一时刻）；
  ///   · 之后**只在参考流断了**（`traj_stream_gap_s`）才重发一次 —— 那是"运行中重选轨迹"
  ///     的处置（ticket 09 Comments 第 2 条）：重选会让 `traj` 自己停流，
  ///     要有人再发一次 start 它才会按新轨迹从零开始。
  /// ⚠️ 重发在"其实还在跑"的情况下是**真实的指令跳变**（参考时钟归零 ⟹ 掉头飞回起点），
  ///   所以闸门用的是 0.5 s（连丢 25 帧）而不是几个周期，理由见 `TaskLimits::traj_stream_gap_s`。
  bool request_traj_start{false};
};

/// `/control/takeoff` 的受理结论（ticket 08）。★ 让"为什么没受理"在**服务应答**里就说清，
/// 而不是只落进日志 —— 操作员按了键没反应时，第一个被问的问题永远是"什么缺了"。
enum class TakeoffReply
{
  kAccepted,       ///< 受理了；随后由 `update()` 推进（**不等于**已解锁，见 `Fsm::request_takeoff`）
  kAlreadyFlying,  ///< 已经在执行一次任务了（含"落地上锁还没等到降沿"的收尾期）
};

const char * to_string(TakeoffReply r);

/// 遥控器**开始键**一条上升沿的受理结论（ticket 09）。
/// ★ 与 `TakeoffReply` 分开而不是合并：两者词表不同（这条多一个"丢掉"），
///   合并之后"为什么没反应"在**两种**输入上会共用一句含糊的话。
enum class StartEdgeReply
{
  kTakeoff,   ///< `kIdle`：这是 §H.5.1 的"第 1 次" ⟹ 受理起飞（D3 = 程序解锁）
  /// `kWaitStart`：§H.5.1 的"第 2 次"。★★ ticket 07 增补起它**不当场进 `TRACK`** 了：
  /// 进 `TRACK` 的动作挪到**松手**那一刻（`note_start_release`），因为"按住不动 ≥ 1 s"
  /// 在同一次按下里还要表达另一件事（降落）——按下就开跑的话那条命令根本无从表达。
  kTrackOnRelease,
  /// ★★ 已在飞（`kClimb`/`kHover`/`kTrack`）：**这不是"丢掉"** —— 它给这次按下锁存了
  /// "有资格成为一次降落请求"，按住够久（`start_land_hold_s`）就会降落。
  /// ★ 但它**不改变任何任务**：光按一下不松（不到时长）什么都不发生。
  kLandHold,
  /// 其余任务态（`kRequested`/`kDone`）⟹ **当场丢掉**，连降落资格都不给
  /// （`kRequested` 是"已受理起飞、还没解锁"⟹ 那一刻飞机还在地上，
  /// 若把这里算成有资格，操作员按住不放会让飞机**刚起飞就降落**，见声明处的反例）。
  /// ★ 刻意**不排队**（与 `request_takeoff` 同一条纪律）：排队意味着"操作员在起飞过程中
  ///   多按了一下，程序在几十秒后自己开跑轨迹"——那个时刻没有人会预期它发生。
  kIgnored,
};

const char * to_string(StartEdgeReply r);

/// 一条**长按**事件（`StartSwitchDetector::take_long_press()`）的受理结论（ticket 07 增补）。
/// ★ 与 `StartEdgeReply` 分开：词表不同（这条多一个"已有别的原因在收口"），
///   合并之后"为什么没反应"在两种输入上会共用一句含糊的话。
enum class StartLandReply
{
  /// 受理：任务就此**正常收口**（停发 + 请求 `AUTO.LAND`），`end_reason = kLandPressed`。
  kLand,
  /// 这次按下的**那一刻**不在飞行态（未解锁的 `INIT`/`STANDBY`，或已受理起飞但还没解锁）
  /// ⟹ 丢掉。★ 判据锁存在边沿那一帧，**不是**看 1 s 到点那一刻的任务态。
  kNotEligible,
  /// 控制权已被夺走（`RC_OVERRIDE` / 模式被外部切走）⟹ **什么模式都不请求** ——
  /// 与 `update()` ④ 同一条纪律：不跟正在接管飞机的人抢。
  kTakeover,
  /// 程序**已经在收口**了（任务已 `kDone`，或已因失效降级而在降落）⟹ 不用再管这一次。
  kAlreadyClosing,
};

const char * to_string(StartLandReply r);

class Fsm
{
public:
  /// @param offboard_prestream_s 切进 OFFBOARD 之前 setpoint 必须先连续流多久。
  ///        见 `phase1-implementation.md` §4.3-1：**断流 PX4 会自动退出 OFFBOARD**
  ///        （`COM_OF_LOSS_T`，z0 现值 1.0 s），所以"先流够了再切"不是优化，是前提。
  /// @param limits 任务判据（ticket 08）。默认值就是工单里的数。
  Fsm(double offboard_prestream_s, TaskLimits limits);

  /// 受理一条**起飞命令** —— `/control/takeoff` 的唯一入口，ticket 09 会让遥控器 ch9 的
  /// 上升沿来调它。★ 语义与 `request_set_mode` 一样是**异步**的：受理 ≠ 起飞。
  ///
  /// 不受理的情形只有一种：**已经有一次任务在跑**（`kAlreadyFlying`）。刻意**不排队**：
  /// 排队等待前置满足 = 会在操作员不看的某一刻自己起飞 —— 那是本项目最不该有的失效模式。
  /// 所以受理之后，前置（守卫 + OFFBOARD + 遥控在线）**当帧**不满足就不发解锁请求，
  /// 由 `takeoff_timeout_s` 兜底作废（见 `TaskLimits::takeoff_timeout_s`）。
  ///
  /// @param now_s 单调递增的秒（同 `update()`）
  TakeoffReply request_takeoff(double now_s);

  /// 解锁请求**被拒**（`/mavros/cmd/arming` 应答 `success == false` 或超时无应答）。
  /// 由边界层在拿到应答时调用；原因由调用方打印（本文件不碰字符串）。
  /// ★ 处置 = **放弃本次起飞、退回 STANDBY**，**不自动重试**（用户 2026-09-22 裁定）：
  ///   静默重试会把"参数没配对"这类确定性的拒绝变成一串没人看的日志。
  void note_arming_failed();

  /// 受理遥控器**开始键的一条上升沿**（ticket 09）。ROS 侧是 `/mavros/rc/in` 上
  /// `start_channel` 的上升沿（判据 = `StartSwitchDetector`，200 ms 去抖、上电即高位不算）。
  ///
  /// ★★ **按任务态判定，不数边沿的次数**：§H.5.1 说的"第 1 次 / 第 2 次"是**同一把开关
  ///   在同一个任务里的两次动作**，而它们之间的区别**不在次数上、在"当时飞机在干什么"上**：
  ///     · `kIdle` ⟹ 受理起飞（`request_takeoff`，§H.5.1 的 D3 = 程序解锁）；
  ///     · `kWaitStart` ⟹ 开始轨迹（进 `TRACK`）；
  ///     · 其余 ⟹ 丢掉。
  ///   用一个计数器去数"这是第几次按键"是**错的**，而且错得很具体：任务落地后计数器不会
  ///   自己归零（除非每个退出路径都记得减），于是**第二次任务的第一次按键会被当成
  ///   "第 2 次"**，飞机在操作员只想起飞的时候直接开跑轨迹。用任务态判则天然没有这个
  ///   状态要同步 —— `kIdle` 就是 `kIdle`，不管之前飞过几轮。
  ///   （"上电即在高位不算边沿"同理不在次数上，在 `StartSwitchDetector::released_` 上。）
  ///
  /// @param now_s 单调递增的秒（同 `update()`）
  StartEdgeReply note_start_edge(double now_s);

  /// 开始键**松手**了（`StartSwitchDetector::take_release()`，确认低位那一刻）。
  /// ★ 唯一的作用是兑现 §H.5.1 的"第 2 次按键"：`kWaitStart` 里按下时只**记下**要开轨迹
  ///   （见 `note_start_edge`），到松手这一帧才真的进 `TRACK`。
  /// ⚠️ 它**不判断**"这一次按下算不算短按"—— 时长全由 `StartSwitchDetector` 说了算：
  ///   真按够长的那些，松手之前就已经走 `note_start_long_press()` 收口了（那条路会把
  ///   "要开轨迹"这个待办清掉）⟹ 走到这里的必然是不足时长的那些。
  /// @return 这一帧是否真的进了 `TRACK`（边界层据此打日志；没人等着的松手返回 false）
  bool note_start_release(double now_s);

  /// 受理**长按**开始键（`StartSwitchDetector::take_long_press()`）：按住够久了 ⟹ 立即降落。
  ///
  /// ★★ **资格看"按下那一刻"**：这一位在 `note_start_edge()` 里就锁存了 —— 只有落在
  ///   *已解锁并在飞* 的任务态（`kClimb`/`kHover`/`kWaitStart`/`kTrack`）上的边沿才算数。
  ///   反例见 `note_start_edge()` 的注释：用"1 s 到点那一刻"的任务态判，操作员按完起飞
  ///   手还停在开关上时，到点那一刻飞机**已经在爬升**，于是它刚离地就自己降落。
  /// @return 受理结论（本方**不打印**任何东西，文案由边界层按它出）
  StartLandReply note_start_long_press();

  /// @param now_s 单调递增的秒（边界层给 `rclcpp::Node::now()`，本文件只用差值）
  /// @param traj 本帧的轨迹参考（ticket 09）。★ 带默认值是**刻意的**：本张之前的所有
  ///        调用点（ticket 05–08 的 ~130 处 + 单测）与 `TRACK` 无关，加一个必填参数会
  ///        让它们全部改写一遍，而改写本身不带来任何信息（它们传的必然是"没有参考"）。
  Decision update(const Guards & guards, const Health & health, const Odometry & odom,
    double now_s, const TrajReference & traj = {});

  /// 观测口（日志/测试用）：当前这轮连续流了多久（没在流时为 0）。
  double streaming_seconds(double now_s) const;

  /// 观测口：当前闩锁住的降级原因（没闩锁时为 `kNone`）。日志用它说明"为什么还没复位"。
  Degrade latched_reason() const {return (latch_ != Latch::kNone) ? latched_reason_
                                                                  : Degrade::kNone;}

  /// 观测口：是否在闩锁里（⟹ 这次失效**不会**自己好）。
  bool latched() const {return latch_ != Latch::kNone;}

  /// 观测口：**起飞时锁定的原点**（进入 TAKEOFF 那一刻的 EKF 位置，ENU）。
  /// ⚠️ 不是轨迹原点 —— 那个只在判稳那一刻才存在，见 `traj_origin()`。
  bool takeoff_origin(double * x, double * y, double * z) const;

  /// 观测口：**轨迹原点** —— 稳定判据成立那一刻的 EKF 位置（工单的 ★ 核心设计），
  /// 之后 `WAIT_START`/`TRACK` 的一切都以它为 0。返回 false 表示**还没判稳过**。
  /// ★ 日志里以 `traj_origin=(x, y, z)` 打出来，可 grep、可与动捕原点对账。
  bool traj_origin(double * x, double * y, double * z) const;

  /// 观测口：**起飞机头**（进入 `TAKEOFF` 那一刻锁定的 ENU 偏航角；没起飞过时是 0）。
  /// 日志用它说清"这轮是按哪个机头飞的"—— 排查"飞机自己在转"这类问题时这是第一个要看的数。
  double takeoff_yaw() const {return takeoff_yaw_;}

  /// 观测口：本帧任务子状态的名字（日志用，`"idle"` / `"requested"` / `"climb"` / …）。
  const char * task_name() const;

  /// 观测口：起飞命令已受理但**还没解锁**（`STANDBY` 里等解锁的那一段）。
  /// 诊断日志用它说明"命令收到了、在等什么"。
  bool awaiting_arming() const {return task_ == Task::kRequested;}

  /// 观测口：这一轮**已经发过**解锁请求（⟹ 再不发第二条）。诊断日志用它区分
  /// "还没到解锁的前置"与"请求发出去了、在等 PX4 生效"—— 两者该查的东西完全不同。
  bool arming_requested() const {return arming_sent_;}

private:
  /// 任务子状态（ticket 08）。★ 单独一层而不是把 `Task` 塞进 `Phase`：
  /// `Phase` 是对外契约（`/control/state` 的 `"<AUTHORITY>/<PHASE>"`，测试也在断言它），
  /// 而这一层管的是"内部飞到哪一步了"。`update()` ⑦ 末尾是**唯一**的翻译处。
  /// ★★ 它同时是"起飞命令的生命周期"：受理 ⟹ `kRequested`，落地上锁 ⟹ 回 `kIdle`，
  ///    中间任何一步都不接受第二条命令 —— 这就把 `kAlreadyFlying` 落到了一处状态上。
  enum class Task
  {
    kIdle,       ///< 没有任务 → `STANDBY`（或未放行时的 `INIT`）
    kRequested,  ///< 收到起飞命令，正在发/等解锁 → 仍是 `STANDBY`（飞机还没被我们驱动）
    kClimb,      ///< 爬升 → `TAKEOFF`
    kHover,      ///< 到高后定点保持并判稳 → `HOVER`
    kWaitStart,  ///< 已判稳、轨迹原点已记录，等开始键 → `WAIT_START`
    kTrack,      ///< 跟着参考飞 → `TRACK`（ticket 09）
    kDone,       ///< 任务走完，等落地上锁 → `LAND`
  };

  /// 闩锁的**种类** —— 它决定"什么条件下才允许复位"。★ 用种类而不是两个 bool：
  /// 两个 bool 会出现"闩锁还在、原因已经被清掉"的退化状态（`latched()` 为真而
  /// `latched_reason()` 为 `kNone`），那时 `active == kNone` 会让状态机**带着失效继续流**。
  enum class Latch
  {
    kNone,         ///< 没闩锁：未解锁的失效走这条（判据一恢复就自动回 STANDBY）
    kTakeover,     ///< 控制权被夺走：对外把模式放回 OFFBOARD / 落地上锁才复位
    kArmedDegrade, ///< 已解锁时的输入失效：任务已中止，**落地上锁**才复位
  };

  double offboard_prestream_s_;
  TaskLimits limits_;
  bool streaming_{false};
  double stream_started_at_s_{0.0};
  /// 曾经流起来过 —— 只有"曾经就绪、之后失去"才算**失效**，否则启动时"还没起完"
  /// （动捕节点还没起、mavros 还没连上）会被记成 degrade，而那是 INIT 的常态。
  bool ever_streamed_{false};
  bool was_armed_{false};
  /// **上一帧**的三个事实（**帧末**刷新 —— 见 `update()` 里 `record()` 的说明；判据与复位只看它，不看本帧）。
  PrevFrame prev_{};
  Latch latch_{Latch::kNone};
  Degrade latched_reason_{Degrade::kNone};

  // ────────── ticket 08 ──────────
  Task task_{Task::kIdle};
  /// 起飞命令的受理时刻（`takeoff_timeout_s` 从它起算）。
  double requested_at_s_{0.0};
  /// 解锁请求**已经发出去过一次**（不重试 ⟹ 这一位进去就不再下来，直到任务复位）。
  /// ★ 它与 `task_ == kRequested` 的区别：`armed` 一到就进 `kClimb`，而这一位保证
  ///   "一次任务只请求解锁一次"，即使中途守卫抖了一下也不会补发第二条。
  bool arming_sent_{false};
  /// 起飞机头（进入 `TAKEOFF` 那一刻锁定，之后全程用它当 setpoint 的 yaw）。
  /// ★ "飞机原地不动"只在 yaw 也被固定时才成立 —— 一个会转向的飞机不算悬停成功。
  double takeoff_yaw_{0.0};
  /// 起飞原点（进入 `TAKEOFF` 那一刻的 EKF 位置）。爬升目标与悬停点都从它推出来。
  double origin_x_{0.0}, origin_y_{0.0}, origin_z_{0.0};
  bool origin_valid_{false};
  /// 爬升起点时刻（斜坡 = `climb_speed_mps * (now - climb_started_at_s_)` 封顶到目标高度）。
  double climb_started_at_s_{0.0};
  /// **当前这一级判据**开始连续成立的时刻；负值表示"还没开始计时"。
  /// ★ 两处判据（到高 / 判稳）共用它：它们**互斥**（同一帧只可能在一个任务态里），
  ///   各存一个反而会出现"离开这个态时忘了清那一个"。每次进新的一级都重置。
  /// ⚠️ 哨兵值必须是**负数**：夹具/测试的 `now_s` 从 ~0 起算，用 `0.0` 当哨兵会在
  ///    开机后那几帧把"还没计时"和"从 0 秒起就在计时"混为一谈。
  double settle_started_at_s_{-1.0};
  /// `WAIT_START` 的起点（`wait_start_timeout_s` 从它起算）。
  double wait_started_at_s_{0.0};
  /// **轨迹原点**（工单的 ★ 核心设计）：稳定判据成立那一刻记下的 EKF 位置。
  /// 之后 `WAIT_START`/`TRACK` 的目标就是它 —— 于是 EKF2 局部原点与动捕原点之间的
  /// 常值偏移被**自动吸收**（我们是"以当前点为 0"发的指令）。
  double traj_x_{0.0}, traj_y_{0.0}, traj_z_{0.0};
  bool traj_valid_{false};

  // ────────── ticket 09（`TRACK`） ──────────

  /// 任务正常结束的原因（只在转进 `kDone` 的那一刻落定，见 `Decision::end_reason`）。
  EndReason end_reason_{EndReason::kNone};
  /// 本轮 `TRACK` 里**发出过** `/traj/start`（⟹ 之后的"没有参考"是"要了没给"）。
  bool traj_start_sent_{false};
  /// 上一次发出 `/traj/start` 的时刻（重发的闸门，见 `TaskLimits::traj_stream_gap_s`）。
  double last_traj_start_at_s_{0.0};
  /// "等参考"的计时起点：进 `TRACK` 那一刻 / 上一次收到参考那一刻。
  /// ★ 只用于 `Health::traj_stale`（`TaskLimits::traj_timeout_s` 从它起算）。
  double traj_waited_since_s_{0.0};
  /// 本轮 `TRACK` 里**收到过至少一帧参考**（诊断 + 说明"参考到底来没来过"）。
  bool track_seen_reference_{false};
  /// ★★ 本轮 `TRACK` 里**见过 `progress < 1.0`** ⟹ 这条轨迹真的在跑。
  ///   为什么需要它（**不是防御性编程，是一个具体的坑**）：`traj` 跑完之后
  ///   **会一直保持 `progress = 1.0` 并继续按 50 Hz 发参考**（`traj_node.cpp:302-308`）。
  ///   于是**同一个节点进程里的第二轮任务**进 `TRACK` 时，手上那个"上一轮的 1.0"是
  ///   **新鲜的**（不是陈旧数据）—— 只看"参考新鲜 + progress ≥ 1.0"会让飞机
  ///   **一进 `TRACK` 就判成飞完、当场降落**。要求"先见它从 1.0 以下跑起来"就把这条堵死了。
  ///   ⚠️ 前提：三条轨迹的时长都 > 0（`trajectories.hpp:148` 把非正时长夹到 1e-3，
  ///     正方形/八字按周长÷限速算）⟹ 不存在"第一帧就是 1.0"的轨迹。
  bool track_started_{false};
  /// `TRACK` 的**最后一个目标**（ENU）。参考断流时用它继续发（"停在那儿别动"）。
  /// ★ 进 `TRACK` 时初值是**轨迹原点**（= 判稳那一刻的位置），所以"还没收到第一帧参考"
  ///   那几十毫秒里飞机就是原地悬停 —— 与 `WAIT_START` 的最后一段逐字连续。
  double track_target_x_{0.0}, track_target_y_{0.0}, track_target_z_{0.0};

  // ────────── ticket 07 增补（长按开始键 ⟹ 降落） ──────────

  /// ★★ **这一条边沿有没有资格成为一次降落请求** —— 在**边沿那一帧**就定下来
  ///   （`note_start_edge`），长按事件到点时只核对它，**不再看当时的任务态**。
  ///   看当时的任务态会让"按完起飞、手还停在开关上"变成"刚离地就降落"（反例见声明处）。
  /// 松手时清掉（这一次按下结束了）。
  bool land_hold_armed_{false};
  /// `WAIT_START` 里按下之后"**欠一次开始**"：松手那一帧兑现（进 `TRACK`）。
  /// ★ 它就是"第 2 次按键的动作挪到松手"这件事的全部状态。
  bool track_on_release_{false};

  /// 进 `TRACK` 的那一套记账（★ 一处做齐、**全清**，不是"只置一个"：这一轮的 `TRACK`
  /// 与上一轮必须看不出区别）。ticket 07 增补起由 `note_start_release()` 调用。
  void start_tracking(double now_s)
  {
    task_ = Task::kTrack;
    traj_start_sent_ = false;        // ⟹ `update()` 的第一帧必发 `/traj/start`
    last_traj_start_at_s_ = now_s;
    traj_waited_since_s_ = now_s;    // 等参考从这一刻起算
    track_seen_reference_ = false;
    track_started_ = false;
    // ★ 断流时的保持目标从**轨迹原点**起（不是上一轮的最后一个目标 —— 那可能离这儿
    //   半条轨迹远，飞机的第一帧就会朝它冲过去）。
    track_target_x_ = traj_x_;
    track_target_y_ = traj_y_;
    track_target_z_ = traj_z_;
  }

  /// `armed` 下降沿的复位动作（落地上锁 ⟹ 一切重来）。★ 抽出来是因为 `update()` 的
  /// ① 复位与任务层必须**同时**归零：漏一个就会出现"飞机已上锁，任务还记着上一轮的轨迹原点"。
  void reset_mission()
  {
    task_ = Task::kIdle;
    arming_sent_ = false;
    origin_valid_ = false;
    traj_valid_ = false;
    settle_started_at_s_ = -1.0;
    climb_started_at_s_ = 0.0;
    wait_started_at_s_ = 0.0;
    // ★ ticket 09：`TRACK` 的那一组闩锁**必须一起清** —— 留着 `track_started_` 会让
    //   下一轮的第一帧就"见过 progress < 1.0"，而那正是它要挡的（见成员注释）。
    end_reason_ = EndReason::kNone;
    traj_start_sent_ = false;
    last_traj_start_at_s_ = 0.0;
    traj_waited_since_s_ = 0.0;
    track_seen_reference_ = false;
    track_started_ = false;
    // ★ ticket 07 增补：开始键那两位也一起清 —— 留着"要开轨迹"的待办会让**下一轮**的
    //   第一次松手就把飞机开进 `TRACK`（而那一轮操作员可能只想起飞）。
    land_hold_armed_ = false;
    track_on_release_ = false;
  }

  /// 停发 + 交出控制权的**公共动作**（④ 夺权 / ⑤ 失效 / ⑥ 守卫不成立 —— 三处的这一段
  /// 必须逐字一致，否则会出现"停发了却还在请求某个模式"这种自相矛盾的输出）。
  /// ★ `phase` 先归 `INIT`，调用方按自己的情形覆盖（`RC_OVERRIDE` / `LAND`）。
  void halt(Decision & d)
  {
    d.authority = Authority::kManual;
    d.stream_setpoint = false;
    d.request_offboard = false;
    d.request_land = false;
    d.phase = Phase::kInit;
    streaming_ = false;
  }
};

// ─────────────────────────── 实现（header-only，仍零 ROS 依赖） ───────────────────────────

inline const char * to_string(Authority a)
{
  // 字符串直接进 `/control/state`（`"<AUTHORITY>/<PHASE>"`），也是测试的断言面 ⟹ 不要随手改。
  return (a == Authority::kAuto) ? "AUTO" : "MANUAL";
}

inline const char * to_string(Phase p)
{
  switch (p) {
    case Phase::kStandby:    return "STANDBY";
    case Phase::kTakeoff:    return "TAKEOFF";
    case Phase::kHover:      return "HOVER";
    case Phase::kWaitStart:  return "WAIT_START";
    case Phase::kTrack:      return "TRACK";
    case Phase::kLand:       return "LAND";
    case Phase::kRcOverride: return "RC_OVERRIDE";
    case Phase::kInit:       break;
  }
  return "INIT";
}

/// `/control/takeoff` 应答的**词表**（`accepted` / `already_flying`）。
/// ★ 由 `control_node.cpp::on_takeoff_request` 放在应答 `message` 的**开头** ⟹ 现场脚本可 grep。
/// ⚠️ 曾经有一个版本把这里晾着不用、文档却按它写词表 —— 两处出处必然走散。
inline const char * to_string(TakeoffReply r)
{
  switch (r) {
    case TakeoffReply::kAccepted:      return "accepted";
    case TakeoffReply::kAlreadyFlying: return "already_flying";
  }
  return "accepted";
}

/// 任务正常结束原因的**词表**（`traj_done` / `wait_start_timeout`）。★ 一样直接进日志。
/// ⚠️ 这里**不许出现"任务完成"这种笼统的词**：这个枚举存在的唯一理由就是让
///   "轨迹飞完了"与"没等到开始键"在日志里长得不一样。
inline const char * to_string(EndReason r)
{
  switch (r) {
    case EndReason::kTrajDone:         return "traj_done";
    case EndReason::kWaitStartTimeout: return "wait_start_timeout";
    case EndReason::kLandPressed:      return "land_pressed";
    case EndReason::kNone:             break;
  }
  return "none";
}

/// 开始键受理结论的**词表**（`takeoff` / `track_on_release` / `land_hold` / `ignored`）——
/// 现场脚本按它 grep "按键到底被当成了什么"。
inline const char * to_string(StartEdgeReply r)
{
  switch (r) {
    case StartEdgeReply::kTakeoff:         return "takeoff";
    case StartEdgeReply::kTrackOnRelease:  return "track_on_release";
    case StartEdgeReply::kLandHold:        return "land_hold";
    case StartEdgeReply::kIgnored:         return "ignored";
  }
  return "ignored";
}

/// 长按受理结论的**词表**（`land` / `not_eligible` / `takeover` / `already_closing`）。
inline const char * to_string(StartLandReply r)
{
  switch (r) {
    case StartLandReply::kLand:           return "land";
    case StartLandReply::kNotEligible:    return "not_eligible";
    case StartLandReply::kTakeover:       return "takeover";
    case StartLandReply::kAlreadyClosing: return "already_closing";
  }
  return "not_eligible";
}

/// ★ 故意不写 `default:`：新增一条 `Degrade` 而这里没跟上时，`-Wswitch` 直接编译失败。
inline const char * to_string(Degrade d)
{
  switch (d) {
    case Degrade::kRcOverride:        return "rc_override";
    case Degrade::kModeLost:          return "mode_lost";
    case Degrade::kStateTimeout:      return "state_timeout";
    case Degrade::kMocapTimeout:      return "mocap_timeout";
    case Degrade::kLocalPosTimeout:   return "local_pos_timeout";
    case Degrade::kRcTimeout:         return "rc_timeout";
    case Degrade::kBatteryLow:        return "battery_low";
    case Degrade::kPositionInvalid:   return "position_invalid";
    case Degrade::kMocapStampStalled: return "mocap_stamp_stalled";
    case Degrade::kEkf2PosLost:       return "ekf2_pos_lost";
    case Degrade::kTrajTimeout:       return "traj_timeout";
    case Degrade::kNone:              break;
  }
  return "none";
}

inline Fsm::Fsm(double offboard_prestream_s, TaskLimits limits)
: offboard_prestream_s_(offboard_prestream_s), limits_(limits) {}

inline TakeoffReply Fsm::request_takeoff(double now_s)
{
  // ★ 只拦"已经有一次任务在跑"。**不拦前置** —— 前置在这条命令进来的时候可能正好缺一条
  //   （动捕刚断了一帧），当场拒绝会让操作员必须掐着时机按键。受理之后前置每帧都查，
  //   不满足就不发解锁请求，`takeoff_timeout_s` 到点作废并在日志里说明（见 `update()` ⑦）。
  if (task_ != Task::kIdle) {
    return TakeoffReply::kAlreadyFlying;
  }
  task_ = Task::kRequested;
  requested_at_s_ = now_s;
  arming_sent_ = false;
  // 上一轮的轨迹原点**现在**就清掉：它已经没意义了（飞机还没起飞），留着只会让日志
  // 在"还没判稳"的时候打出一个上一轮的原点。
  traj_valid_ = false;
  origin_valid_ = false;
  settle_started_at_s_ = -1.0;
  return TakeoffReply::kAccepted;
}

inline StartEdgeReply Fsm::note_start_edge(double now_s)
{
  // ★★ 判据是**当前任务态**，不是"这是第几次按键"（完整理由见声明处）。
  switch (task_) {
    case Task::kIdle:
      // §H.5.1 的"第 1 次"：受理起飞。⚠️ 走 `request_takeoff()` 而不是自己置 `kRequested`
      //   —— 受理的全部语义（不排队、前置当帧查、`takeoff_timeout_s` 兜底）都在那里，
      //   复制一遍迟早走散（尤其是"已受理还在等解锁时再来一条"的拒绝）。
      //   `request_takeoff` 在 `kIdle` 上必然受理 ⟹ 这里也必然返回 `kTakeoff`。
      // ★ 这一次按下**没有降落资格**（那一刻飞机还没被我们驱动）⟹ 显式清掉那一位：
      //   资格是"这一条边沿带来的"，不能靠"上一条边沿没留下它"活着。
      land_hold_armed_ = false;
      return (request_takeoff(now_s) == TakeoffReply::kAccepted) ? StartEdgeReply::kTakeoff
                                                                 : StartEdgeReply::kIgnored;

    case Task::kWaitStart:
      // §H.5.1 的"第 2 次" —— ★★ 但**不当场进 `TRACK`**（ticket 07 增补）：
      //   同一次按下还可能是"长按降落"的起点（用户裁定 `WAIT_START` 里也算），
      //   而按下就开跑会让那条命令**根本无从表达**（而且那 1 s 里飞机已经在跟参考飞了）。
      //   ⟹ 这里只记下"欠一次开始"，由**松手**（`note_start_release`）兑现；
      //     按够时长则由长按事件收口（它会把这个待办清掉）。
      track_on_release_ = true;
      land_hold_armed_ = true;
      return StartEdgeReply::kTrackOnRelease;

    case Task::kClimb:
    case Task::kHover:
    case Task::kTrack:
      // ★★ 已解锁并在飞 ⟹ 这一次按下**有降落资格**（ticket 07 增补要的就是这条路）。
      //   除此之外**什么都不做**：按一下不松（不到时长）不改变任何任务。
      land_hold_armed_ = true;
      return StartEdgeReply::kLandHold;

    case Task::kRequested:
    case Task::kDone:
      // 丢掉，**连降落资格都不给**。★ `kRequested` 是"已受理起飞、还没解锁"——那一刻飞机
      //   还在地上，若把它算成有资格，操作员按完起飞不松手会让飞机**刚离地就降落**
      //   （反例的完整链条见声明处）。一处也不能"帮操作员记住"：排队 = 在自己不看的某一刻开跑。
      land_hold_armed_ = false;
      return StartEdgeReply::kIgnored;
  }
  return StartEdgeReply::kIgnored;
}

inline bool Fsm::note_start_release(double now_s)
{
  const bool pending = track_on_release_;
  // ★ 无论走哪条路，**这一次按下都结束了**：待办与降落资格一起清（长按那条路已经清过一遍）。
  track_on_release_ = false;
  land_hold_armed_ = false;
  // 只有"在 `WAIT_START` 里按下、还没被长按接管"的那一次才兑现。别的松手一律不管：
  // 起飞的松手、`TRACK` 里按了一下又松开的松手、任务已经在收口时的松手。
  if (!pending || task_ != Task::kWaitStart) {
    return false;
  }
  start_tracking(now_s);
  return true;
}

inline StartLandReply Fsm::note_start_long_press()
{
  // ① 控制权已经被夺走 ⟹ **什么模式都不请求**（同 `update()` ④ 的纪律）。
  if (latch_ == Latch::kTakeover) {
    land_hold_armed_ = false;
    return StartLandReply::kTakeover;
  }
  // ② 这次按下的那一刻没有资格 —— 判据是**边沿那一帧**锁存的那一位，
  //    **不是**现在这个任务态（反例见 `note_start_edge` 的注释）。
  if (!land_hold_armed_) {
    return StartLandReply::kNotEligible;
  }
  land_hold_armed_ = false;
  track_on_release_ = false;   // 已经要降落了，松手不许再把飞机开进 `TRACK`
  // ③ 程序**已经在收口**了：任务自己走完（`kDone`），或已因失效降级在降落（闩锁）。
  //    这一条只影响日志措辞 —— 那两种情况本来就会降落，这里不做第二件事。
  if (task_ == Task::kDone || latch_ != Latch::kNone) {
    return StartLandReply::kAlreadyClosing;
  }
  // ④ 受理：**正常收口**。★ 这里只置状态，真正的动作（停发 + 请求 `AUTO.LAND`）留给本帧的
  //    `update()` ⑤ —— "降落只有一个出口"这条纪律不许在这里破。
  end_reason_ = EndReason::kLandPressed;
  task_ = Task::kDone;
  return StartLandReply::kLand;
}

inline void Fsm::note_arming_failed()
{
  // 只对"我们正在等解锁"这一态有意义；别的态收到这条说明边界层多调了，忽略即可
  // （不 assert：这不值得让飞控边上跑着的节点崩掉）。
  if (task_ == Task::kRequested) {
    reset_mission();
  }
}

inline bool Fsm::takeoff_origin(double * x, double * y, double * z) const
{
  if (!origin_valid_) {
    return false;
  }
  *x = origin_x_;
  *y = origin_y_;
  *z = origin_z_;
  return true;
}

inline bool Fsm::traj_origin(double * x, double * y, double * z) const
{
  if (!traj_valid_) {
    return false;
  }
  *x = traj_x_;
  *y = traj_y_;
  *z = traj_z_;
  return true;
}

inline const char * Fsm::task_name() const
{
  // ★ 日志用的小写词，与 `/control/state` 里的大写 `Phase` **刻意不同名**：
  //   两者接近但不等价（`kRequested` 的阶段名是 `STANDBY`），同名会让人以为是同一个东西。
  switch (task_) {
    case Task::kIdle:      return "idle";
    case Task::kRequested: return "requested";
    case Task::kClimb:     return "climb";
    case Task::kHover:     return "hover";
    case Task::kWaitStart: return "wait_start";
    case Task::kTrack:     return "track";
    case Task::kDone:      return "done";
  }
  return "idle";
}

inline Decision Fsm::update(const Guards & guards, const Health & health, const Odometry & odom,
  double now_s, const TrajReference & traj)
{
  Decision d;

  const bool prev_armed = was_armed_;
  const PrevFrame prev = prev_;          // ★ 本帧的判据看的是**上一帧**（本帧的事实见 `record`）
  // ★★ `was_armed_` **只在可信帧上刷新**（ticket 18 的第二半，票面没写、读代码才发现）：
  //   复位判据是 `armed` 的**降沿**，而 mavros 那帧合成断连帧（`connected=false`）里的
  //   `armed=false` 是**占位值**而不是读数。让它把降沿吃掉，链路恢复时"真的落地上锁"
  //   （`connected=true ∧ armed=false`）就再也对不上边 ⟹ 闩锁永远不复位 = **单向 latch**
  //   （ticket 06 验收项 ⑤ 明令禁止的形状，只能重启节点）。
  //   ⟹ 这里与 ① 的 `guards.connected` 是**同一条语义的两半，缺一不可**，各有一条用例钉着。
  if (guards.connected) {
    was_armed_ = guards.armed;
  }

  // ★★ 取的是**本帧结束时**的台账，因为 `streaming_` 正是**本帧**被改动的
  //   （⑥ 置真、`halt()` 置假，两者都在 `record` 之前落定）——在函数开头抄一份会差一帧，
  //   那正好把"我们正驱动着 OFFBOARD"判成否 ⟹ 夺权判据漏报（第一版就是这么错的，测试抓住的）。
  //   `mode_is_ours(guards)` / `guards.in_land` 由入参 `guards` 派生、`update()` 全程只读
  //   ⟹ 帧首帧末同值；三者放在一处落账只是为了让"上一帧的三个事实"同一时刻取值，语义一致。
  auto record = [&](Decision d) {
    prev_ = PrevFrame{mode_is_ours(guards), streaming_, guards.in_land};
    return d;
  };

  // ── ① 复位（先复位、再判据 ⟹ 判据在同一帧恢复就立刻生效）──
  //   (a) **可信的**落地上锁：`connected` 且 `armed` true→false ⟹ 飞机落地/上锁，任务结束、一切重来。
  //       **这是"可复位"的主路径** —— 不需要重启节点，也不需要任何"清故障"命令，操作员落地上锁即可。
  //       ★★ `guards.connected` 这一半是 ticket 18 加的（2026-09-22）：mavros 在链路超时后会
  //       **自己发一帧** `connected=false, armed=false, guided=false, manual_input=false, mode=""`
  //       （`sys_status.cpp:903-911` 的 `publish_disconnection()`，`:1411` 触发；第一手核实）。
  //       那帧里的 `armed=false` **不是读数**而是占位值（它只说明"现在不知道飞控怎么样"），
  //       凭它复位 = 把闩锁**在空中**清掉，而链路恢复时若 PX4 仍 armed，本节点就会走上"重新准备接管"
  //       那条路（⑦ 的 `request_offboard`）。⚠️ 当前 `kArmingByUs` **恰好也**挡着它 —— 那帧的
  //       `armed=false` 顺手清了归属标记（`px4_interface.cpp:160-164`）⟹ 恢复时 `own_arming=false`
  //       ⟹ 守卫不成立、停在 `MANUAL/INIT`。但那是**偶然的安全网**（它依赖"合成帧恰好也清了归属"），
  //       判据必须自己挡住这件事 —— 且 `arm_request_pending_` 陈旧时那层网就不在了。
  //       ⚠️ 代价（刻意接受）：真出现"链路断了之后飞机被人工落地上锁"时，我们也**看不到**那次上锁
  //       —— 闩锁要等链路恢复、重新看到 `connected=true ∧ armed=false` 才复位（实测就是几秒）。
  //   (b) 模式**被外部**交回我们手里（升到 OFFBOARD）：闩锁期间我们从不自己请求 OFFBOARD
  //       ⟹ 这条只可能是外部动作。
  if (guards.connected && prev_armed && !guards.armed) {
    latch_ = Latch::kNone;
    latched_reason_ = Degrade::kNone;
    // ★★ 任务层**同时**归零（ticket 08）：落地上锁 = 这一轮飞完了。不清的话，
    //    `takeoff_origin` / `traj_origin` 会带着上一轮的值活到下一轮，而它们正是
    //    "目标"的基准 ⟹ 下一轮会在**上一轮的原点**上悬停。
    reset_mission();
  } else if (!prev.mode_ours && guards.in_offboard) {
    // 只清**夺权**闩锁：已解锁的输入失效要等落地上锁（电池不会因为有人切了模式就变满）。
    if (latch_ == Latch::kTakeover) {
      latch_ = Latch::kNone;
      latched_reason_ = Degrade::kNone;
    }
  }

  // ── ② 本轮判据 ──
  // ★★ ticket 09：`TRACK` 里"参考没了"这一条**在任务层算**（理由见 `Health::traj_stale`）：
  //   判据要的计时起点是"进 TRACK / 上次收到参考 / 上次重发 `/traj/start`"，
  //   全是边界层看不到的事。这里把算好的结果填进 `Health` 的一份副本再喂给 `first_degrade`，
  //   于是"降级判据"仍然只有 `first_degrade` 一个出处。
  Health h = health;
  if (task_ == Task::kTrack && traj.alive(limits_.traj_stream_gap_s)) {
    traj_waited_since_s_ = now_s;   // 参考到了 ⟹ 重新起算
  }
  h.traj_stale = (task_ == Task::kTrack) &&
    (now_s - traj_waited_since_s_) >= limits_.traj_timeout_s;
  const Degrade reason = ever_streamed_ ? first_degrade(guards, h, prev) : Degrade::kNone;

  // ── ③ 闩锁：记住**首因**（最接近根因），闩锁期间不被后续判据改写 ──
  if (reason != Degrade::kNone) {
    if (latch_ == Latch::kNone) {
      latched_reason_ = reason;
      if (is_takeover(reason)) {
        latch_ = Latch::kTakeover;         // 夺权：任何解锁状态都闩锁（不许抢回控制权）
      } else if (guards.armed) {
        latch_ = Latch::kArmedDegrade;     // 已解锁的失效：闩到落地上锁
      }
      // ★ 未解锁的失效**不闩锁** ⟹ 判据一恢复就自动回 STANDBY（"可复位、不是单向 latch"的
      //   台架证据就在这条上）。
    } else if (latch_ == Latch::kArmedDegrade && is_takeover(reason)) {
      // ★ **夺权可以升级一个已经闩锁的降级** —— 飞行员永远优先。
      //   不升级的后果很具体：电池低压已经闩锁在 LAND 上，飞行员却打了杆想接管，而我们还在
      //   每 offboard_retry_s 请求一次 `AUTO.LAND` —— 与"抢回 OFFBOARD"同一类拉锯，
      //   只是这次是在跟一个正在接管飞机的人抢。首因保持只在**同类或更弱**的新原因上生效。
      latch_ = Latch::kTakeover;
      latched_reason_ = reason;   // 换成夺权的名字：日志要指向正确的排查方向
    }
  }
  const bool latched = (latch_ != Latch::kNone);
  const Degrade active = latched ? latched_reason_ : reason;
  d.degrade = active;

  // ── ④ 控制权被夺走 ⟹ 停发 + 交还，**不请求任何模式** ──
  if (latch_ == Latch::kTakeover) {
    halt(d);
    // ★ 这是本张最关键的一条"不做"：不抢回 OFFBOARD（`halt()` 已经把 request_offboard
    //   与 request_land 都清掉了），连 `AUTO.LAND` 也不请求 —— 控制权在飞行员手上。
    d.phase = Phase::kRcOverride;
    return record(d);
  }

  // ── ⑤ 有失效 ⟹ 停发；已解锁时主动中止任务（请求 AUTO.LAND）──
  //   ★ ticket 08 把"任务**正常**走完"接进同一个出口：两者的**动作逐字相同**（停发 + 降落），
  //     分开写迟早会出现两处不一致（比如只给降级那条重试 `AUTO.LAND`）。差别只在日志里
  //     怎么称呼它 ⟹ 用 `d.end_reason` 传出去，由边界层决定措辞
  //     （ticket 09 起它**两条正常出口**：轨迹飞完、`WAIT_START` 兜底超时 —— 两者动作
  //      逐字相同、意思相反，措辞必须分得开，这正是把它从 bool 升成枚举的原因）。
  if (active != Degrade::kNone || task_ == Task::kDone) {
    halt(d);
    // ★ 只有"没出故障"的结束才报原因：那时 `degrade` 也是 `kNone`，两条信息互补。
    d.end_reason = (active == Degrade::kNone) ? end_reason_ : EndReason::kNone;
    if (guards.armed) {
      // ★ 尽力而为：`AUTO.LAND` 需要 EKF2 的 local position（`mode_requirements.cpp:141`），
      //   位置真失效时 PX4 会**拒绝**它 —— 那时唯一兜底是 PX4 自己的失效保护
      //   （`NAV_RCL_ACT` / `COM_OBL_RC_ACT`，注意 z0 现值会让 RC 丢失去**返航**，见 A1 阻塞项）。
      d.phase = Phase::kLand;
      // ★ 已经在我们请求的降落模式里就**不再重发**：`mode == AUTO.LAND` 说明 PX4 接受了，
      //   此时再按重试间隔请求一遍是纯粹的噪声（真正需要重试的是"被拒了"，而那种情况下
      //   `in_land` 一直是 false）。
      // ⚠️ 模式被别人切走时的处置分两种，见 `first_degrade` 里那条 ★★：从 `AUTO.LAND`
      //   被切走 = 夺权（走 ④，连 LAND 也不再请求，不跟人抢）；从 OFFBOARD 被切走而我们
      //   已经停发 = 多半是 PX4 自己的断流保护，于是**继续按本分支重试降落**。
      d.request_land = !guards.in_land;
    }
    return record(d);
  }

  // ── ⑥ 没有失效：ticket 05 的放行判据 ──
  if (!ready_to_stream(guards)) {
    halt(d);   // 守卫不成立 ⟹ 停发并退回 INIT（三处的公共动作，见 `halt()`）
    return record(d);
  }

  // ★ 发 setpoint 需要**可用的位置与姿态**（ticket 08 起目标由本文件算，不再拿"当前位姿"顶替）。
  //   位置失效的那一帧边界层不刷新新鲜度 ⟹ `position_invalid` 会先降级（上面的 ⑤），
  //   这条在正常路径上**不会触发**；留着是为了让"绝不带着无效位置发 setpoint"成为一条
  //   接缝 2 可测的**不变式**（而不是"靠上游不会发生"）。
  if (!odom.position_valid) {
    halt(d);
    return record(d);
  }

  // ── ⑦ 无失效、守卫成立：流 setpoint + 推进任务 ──
  d.authority = guards.armed ? Authority::kAuto : Authority::kManual;
  d.stream_setpoint = true;
  ever_streamed_ = true;

  if (!streaming_) {
    streaming_ = true;
    stream_started_at_s_ = now_s;
  }

  d.request_offboard =
    !guards.in_offboard && (now_s - stream_started_at_s_) >= offboard_prestream_s_;

  // 默认目标 = **当前位置、当前机头**（STANDBY 的语义就是"原地保持"）。
  d.target_x = odom.x;
  d.target_y = odom.y;
  d.target_z = odom.z;
  d.target_yaw = odom.yaw;

  switch (task_) {
    case Task::kIdle:
      d.phase = Phase::kStandby;
      break;

    case Task::kRequested:
      // 收到起飞命令、正在发/等解锁。★ 对外**仍是 STANDBY**：飞机还没被我们驱动，
      //   `AUTO` 那条定义（"正在发会驱动飞机的指令"）必须等 `armed` 为真才成立
      //   —— 不过 `d.authority` 上面已经按 `guards.armed` 算了，正好。
      d.phase = Phase::kStandby;
      if (guards.armed) {
        // 解锁成功（不管是哪一帧成功的）⟹ **就在这一帧锁定原点与机头**，开始爬升。
        // ★ 锁在"看到 armed 的那一帧"而不是"发解锁请求的那一帧"：请求与生效之间
        //   飞机可能已经在地上被风吹着挪了一点，而我们要的基准是**起飞时的**位置。
        origin_x_ = odom.x;
        origin_y_ = odom.y;
        origin_z_ = odom.z;
        origin_valid_ = true;
        takeoff_yaw_ = odom.yaw;
        task_ = Task::kClimb;
        climb_started_at_s_ = now_s;
        settle_started_at_s_ = -1.0;
      } else if (!arming_sent_ && guards.in_offboard && health.rc_fresh) {
        // ★ 解锁的前置（§H.6 原文两条）= **已在 OFFBOARD** + **setpoint 已在流**。
        //   后者由 `streaming_`/`offboard_prestream_s_` 保证（切进 OFFBOARD 之前就得流够），
        //   这里补前者的显式判断 —— 少了它，在 STANDBY 还没切模式的那一小段时间里
        //   发了解锁请求，PX4 会因为"没在 OFFBOARD"而拒绝（H.6 的 D3）。
        // ★★ 再加一条 `rc_fresh`（§H.7.1「解锁前遥控必须开机且链路已建立」）：把那条
        //   **人肉清单项变成机器判据**。不这么做的后果是具体的 —— 遥控静默时解锁，
        //   紧接着 ④ 的 `rc_timeout`（只在已解锁时算失效）就会判失效并请求降落，
        //   于是飞机**起飞后立刻降落**，比一开始就不解锁难查得多。
        //   ⚠️ 代价：台上没有遥控接收机（`/mavros/rc/in` 从未到帧）时**永远解不了锁**。
        //   这是刻意的：ticket 09 的起飞键就在遥控器上，遥控不在场本来就不该起飞。
        d.request_arming = true;
        arming_sent_ = true;
      } else if (now_s - requested_at_s_ > limits_.takeoff_timeout_s) {
        // 前置迟迟不齐 / 解锁始终没生效 ⟹ **作废**，退回 STANDBY（不自动重试）。
        // ⚠️ 这里**不打印**原因：诊断日志每 `diag_period_s` 已经把这轮的缺失项列出来了。
        reset_mission();
      }
      break;

    case Task::kClimb: {
      d.phase = Phase::kTakeoff;
      // 机头保持起飞机头（不转向）。
      d.target_yaw = takeoff_yaw_;
      // 目标高度 = 原点 + 高度上限；目标 z 沿斜坡上升，速率 ≤ climb_speed_mps。
      // ★ 水平两轴**钉在原点**：工单要的是"垂直起飞"，不是"斜着飞上去"。
      const double altitude = limits_.takeoff_altitude_m;
      const double climbed =
        limits_.climb_speed_mps * std::max(0.0, now_s - climb_started_at_s_);
      d.target_x = origin_x_;
      d.target_y = origin_y_;
      d.target_z = origin_z_ + std::min(altitude, climbed);

      // 到高判据：`|z − (原点+高度)| < tol ∧ |v_z| < tol` 持续 `takeoff_settle_s`。
      // ⚠️ 用**原点+高度**而不是 `target_z`：斜坡还没走完时目标就在当前高度上，
      //    拿它当基准会"发出去的一瞬间就满足"，斜坡形同虚设。
      // ⚠️ `velocity_valid` 是必需的：速度话题还没到过一帧时 `vz` 是 0（"看起来静止"），
      //    用它判稳会**立刻**通过 —— 这是"没数据当成好消息"的老毛病，必须显式挡掉。
      const bool at_altitude = odom.velocity_valid &&
        std::fabs(odom.z - (origin_z_ + altitude)) < limits_.takeoff_alt_tol_m &&
        std::fabs(odom.vz) < limits_.takeoff_vel_tol_mps;
      if (at_altitude) {
        if (settle_started_at_s_ < 0.0) {
          settle_started_at_s_ = now_s;
        } else if (now_s - settle_started_at_s_ >= limits_.takeoff_settle_s) {
          task_ = Task::kHover;
          settle_started_at_s_ = -1.0;   // 判稳重新计时（判据不同，不能沿用）
        }
      } else {
        settle_started_at_s_ = -1.0;     // 连续成立的语义：一帧不成立就重新计时
      }
      break;
    }

    case Task::kHover: {
      d.phase = Phase::kHover;
      d.target_yaw = takeoff_yaw_;
      // 悬停点 = 起飞原点正上方 `takeoff_altitude_m`（与爬升的目标一致 ⟹ 到高后无跳变）。
      d.target_x = origin_x_;
      d.target_y = origin_y_;
      d.target_z = origin_z_ + limits_.takeoff_altitude_m;

      // 稳定判据（工单原话）：`位置误差 < 0.10 m ∧ |v| < 0.10 m/s 持续 2 s`。
      // ★ 位置误差相对**悬停点**（见文件头 ★ 的说明：轨迹原点此刻还不存在）。
      // ⚠️ `velocity_valid` 同上，必须显式要求。
      const bool stable = odom.velocity_valid &&
        odom.distance_to(d.target_x, d.target_y, d.target_z) < limits_.hover_pos_tol_m &&
        odom.speed() < limits_.hover_vel_tol_mps;
      if (stable) {
        if (settle_started_at_s_ < 0.0) {
          settle_started_at_s_ = now_s;
        } else if (now_s - settle_started_at_s_ >= limits_.hover_settle_s) {
          // ★★ **轨迹原点**：稳定判据成立那一刻的 EKF 位置（工单的 ★ 核心设计）。
          //   记的是**当前位置**，不是悬停点 —— "以当前点为 0"正是这条设计的意思，
          //   EKF2 局部原点与动捕原点之间的那个常值偏移就此被吸收。
          //   它与悬停点的差不超过 `hover_pos_tol_m`（判据保证），所以跳变 ≤ 10 cm。
          traj_x_ = odom.x;
          traj_y_ = odom.y;
          traj_z_ = odom.z;
          traj_valid_ = true;
          task_ = Task::kWaitStart;
          wait_started_at_s_ = now_s;
          settle_started_at_s_ = -1.0;
        }
      } else {
        settle_started_at_s_ = -1.0;
      }
      break;
    }

    case Task::kWaitStart:
      d.phase = Phase::kWaitStart;
      d.target_yaw = takeoff_yaw_;
      // 目标 = **轨迹原点本身**："停在原地"。★ 若沿用悬停点，飞机就会慢慢朝悬停点
      // 挪回到最开始的 10 cm 内 —— 那会让"轨迹原点"变成一个飞机其实没去过的点。
      d.target_x = traj_x_;
      d.target_y = traj_y_;
      d.target_z = traj_z_;
      // ★★ 出口①（正常出口）是**开始键**：`note_start_edge()` 已经把 `task_` 改成 `kTrack`
      //   ⟹ 那一帧根本走不到这里。所以本分支只剩出口②：兜底超时。
      // ⚠️ 超时**不是"再等一会儿"，是结束任务 + 降落** —— 它存在的理由是"程序不能有一个
      //   自己走不出去的态"（操作员忘了按键、遥控器没电、……），见 `wait_start_timeout_s`。
      if (limits_.wait_start_timeout_s > 0.0 &&
        now_s - wait_started_at_s_ >= limits_.wait_start_timeout_s)
      {
        end_reason_ = EndReason::kWaitStartTimeout;
        task_ = Task::kDone;
      }
      break;

    case Task::kTrack: {
      d.phase = Phase::kTrack;
      // ★ yaw 仍然是**起飞机头**，**不转发参考的 `yaw`**（理由见文件头 ticket 09 那一节：
      //   `traj` 的 `yaw_rad` 恒 0，而在 ENU 口径下 0 就是"机头朝东"）。
      d.target_yaw = takeoff_yaw_;

      // ── 目标 = 轨迹原点 + 参考（**加原点**，绝不直接转发参考）──
      const bool ref_alive = traj.alive(limits_.traj_stream_gap_s);
      if (ref_alive) {
        track_seen_reference_ = true;
        // ★★ 加原点 —— ticket 09 Comments 第 1 条的**唯一**执行处。
        d.target_x = traj_x_ + traj.x;
        d.target_y = traj_y_ + traj.y;
        d.target_z = traj_z_ + traj.z;
        track_target_x_ = d.target_x;
        track_target_y_ = d.target_y;
        track_target_z_ = d.target_z;
      } else {
        // 参考断了：**保持最后一个目标**（飞机悬停不动）。
        // ★ 为什么不是"回落到轨迹原点"：那是一道肉眼看不见的位移指令（原点与当前点可能
        //   差着半条轨迹的距离）——参考断掉的那一刻正是最不该主动加一次机动的时候。
        //   这一支只在进 TRACK 后的头 0.5 s（还没收到第一帧）真正落到"原点"上，
        //   那时 `d.target_*` 的默认值就是当前位姿，本来也不动。
        d.target_x = track_target_x_;
        d.target_y = track_target_y_;
        d.target_z = track_target_z_;
      }

      // ── 要不要（重）发 `/traj/start` ──
      //   · 进 TRACK 的第一帧必发：参考时钟的零点必须与"开始飞"同一时刻；
      //   · 之后只在**参考流断了**且**距上次发也过了同样久**（节流）时重发一次 ——
      //     那是"运行中重选轨迹"（`traj` 自己停流了）唯一的救法。
      // ⚠️ 重发的闸门（`traj_stream_gap_s`）必须远大于 50 Hz 的周期，理由见那个成员的注释。
      const bool due = (now_s - last_traj_start_at_s_) >= limits_.traj_stream_gap_s;
      if (!traj_start_sent_ || (!ref_alive && due)) {
        d.request_traj_start = true;
        traj_start_sent_ = true;
        last_traj_start_at_s_ = now_s;
      }

      // ── 任务结束：轨迹真的跑完 ──
      // ⚠️ 两个条件都不能少（见 `track_started_` 的成员注释）：`progress` 是**最后一次
      //   收到的值**，上一轮任务留下的 1.0 在流断时不会归零 —— 只看数值会让第二轮任务
      //   一进 TRACK 就判飞完。要求"亲眼见过它从 1.0 以下跑起来"才算。
      if (ref_alive && traj.progress_seen && !track_started_ && traj.progress < 1.0) {
        track_started_ = true;
      }
      if (track_started_ && traj.progress >= 1.0) {
        end_reason_ = EndReason::kTrajDone;
        task_ = Task::kDone;
      }
      break;
    }

    case Task::kDone:
      // 不可达（⑤ 已经把它拦在降落分支里了）。写出来是为了让 `-Wswitch` 保持完整。
      d.phase = Phase::kLand;
      break;
  }
  return record(d);
}

inline double Fsm::streaming_seconds(double now_s) const
{
  return streaming_ ? (now_s - stream_started_at_s_) : 0.0;
}

}  // namespace control

#endif  // CONTROL__FSM_HPP_
