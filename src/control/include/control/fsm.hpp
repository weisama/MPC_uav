// control 的状态机 —— ★ 两个正交维度显式分开，不压成扁平枚举。
//
//   控制权：  MANUAL ──(本项目开始发会驱动飞机的指令)──▶ AUTO ──(RC 打杆 / 超时 / 异常)──▶ MANUAL
//   任务态：  INIT ─▶ STANDBY ─▶ TAKEOFF ─▶ HOVER ─▶ TRACK ─▶ LAND
//                                                        └──▶ RC_OVERRIDE（控制权被外部夺走）
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

#ifndef CONTROL__FSM_HPP_
#define CONTROL__FSM_HPP_

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
  kStandby,     ///< **已开始以 setpoint_rate_hz 持续发布 setpoint**（位置 = 当前，yaw = 0）
  kTakeoff,     ///< 占位（ticket 08）
  kHover,       ///< 占位（ticket 08）
  kTrack,       ///< 占位（ticket 07）
  kLand,        ///< 主动中止任务（ticket 06）：停发 + 请求 `AUTO.LAND`（仅**已解锁**时）
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
  /// ★ 本次解锁是**本节点发起的**（ticket 06/08 之间的缝，见 `own_arming` 参数）。
  ///   本张没有解锁路径 ⟹ 默认 false ⟹ 任何 `armed` 上升沿都是"外部解锁"（异常，§H.6）。
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
  /// 由 `Guards::own_arming` 提供（ticket 08 会把它变成真实解锁状态而不是参数）。
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
};

const char * to_string(Degrade d);

/// 降级的**全部成员**（唯一出处，同 `all_guards()`）：日志文案与测试的完备性断言都取它。
inline const std::vector<Degrade> & all_degrades()
{
  static const std::vector<Degrade> kAll{
    Degrade::kNone, Degrade::kRcOverride, Degrade::kModeLost, Degrade::kStateTimeout,
    Degrade::kMocapTimeout, Degrade::kLocalPosTimeout, Degrade::kRcTimeout, Degrade::kBatteryLow,
    Degrade::kPositionInvalid, Degrade::kMocapStampStalled};
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
  return Degrade::kNone;
}

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
};

class Fsm
{
public:
  /// @param offboard_prestream_s 切进 OFFBOARD 之前 setpoint 必须先连续流多久。
  ///        见 `phase1-implementation.md` §4.3-1：**断流 PX4 会自动退出 OFFBOARD**
  ///        （`COM_OF_LOSS_T`，z0 现值 1.0 s），所以"先流够了再切"不是优化，是前提。
  explicit Fsm(double offboard_prestream_s);

  /// @param now_s 单调递增的秒（边界层给 `rclcpp::Node::now()`，本文件只用差值）
  Decision update(const Guards & guards, const Health & health, double now_s);

  /// 观测口（日志/测试用）：当前这轮连续流了多久（没在流时为 0）。
  double streaming_seconds(double now_s) const;

  /// 观测口：当前闩锁住的降级原因（没闩锁时为 `kNone`）。日志用它说明"为什么还没复位"。
  Degrade latched_reason() const {return (latch_ != Latch::kNone) ? latched_reason_
                                                                  : Degrade::kNone;}

  /// 观测口：是否在闩锁里（⟹ 这次失效**不会**自己好）。
  bool latched() const {return latch_ != Latch::kNone;}

private:
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
  bool streaming_{false};
  double stream_started_at_s_{0.0};
  /// 曾经流起来过 —— 只有"曾经就绪、之后失去"才算**失效**，否则启动时"还没起完"
  /// （动捕节点还没起、mavros 还没连上）会被记成 degrade，而那是 INIT 的常态。
  bool ever_streamed_{false};
  bool was_armed_{false};
  /// **上一帧**的三个事实（每帧开头刷新；判据与复位都只看它，不看本帧）。
  PrevFrame prev_{};
  Latch latch_{Latch::kNone};
  Degrade latched_reason_{Degrade::kNone};

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
    case Phase::kTrack:      return "TRACK";
    case Phase::kLand:       return "LAND";
    case Phase::kRcOverride: return "RC_OVERRIDE";
    case Phase::kInit:       break;
  }
  return "INIT";
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
    case Degrade::kNone:              break;
  }
  return "none";
}

inline Fsm::Fsm(double offboard_prestream_s)
: offboard_prestream_s_(offboard_prestream_s) {}

inline Decision Fsm::update(const Guards & guards, const Health & health, double now_s)
{
  Decision d;

  const bool prev_armed = was_armed_;
  const PrevFrame prev = prev_;          // ★ 本帧的判据看的是**上一帧**（本帧的事实见 `record`）
  was_armed_ = guards.armed;

  // ★★ 帧末才刷新"上一帧的事实"：`streaming_` 往往是**本帧**才被置真/置假的
  //   （⑥ 置真、`halt()` 置假），在函数开头抄一份会差一帧 —— 那正好把
  //   "我们正驱动着 OFFBOARD" 判成否，于是夺权判据漏报（第一版就是这么错的，测试抓住的）。
  auto record = [&](Decision d) {
    prev_ = PrevFrame{mode_is_ours(guards), streaming_, guards.in_land};
    return d;
  };

  // ── ① 复位（先复位、再判据 ⟹ 判据在同一帧恢复就立刻生效）──
  //   (a) `armed` true→false：飞机落地/上锁 ⟹ 任务结束，一切重来。**这是"可复位"的主路径** ——
  //       不需要重启节点，也不需要任何"清故障"命令，操作员落地上锁即可。
  //   (b) 模式**被外部**交回我们手里（升到 OFFBOARD）：闩锁期间我们从不自己请求 OFFBOARD
  //       ⟹ 这条只可能是外部动作。
  if (prev_armed && !guards.armed) {
    latch_ = Latch::kNone;
    latched_reason_ = Degrade::kNone;
  } else if (!prev.mode_ours && guards.in_offboard) {
    // 只清**夺权**闩锁：已解锁的输入失效要等落地上锁（电池不会因为有人切了模式就变满）。
    if (latch_ == Latch::kTakeover) {
      latch_ = Latch::kNone;
      latched_reason_ = Degrade::kNone;
    }
  }

  // ── ② 本轮判据 ──
  const Degrade reason = ever_streamed_ ? first_degrade(guards, health, prev) : Degrade::kNone;

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
  if (active != Degrade::kNone) {
    halt(d);
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

  d.phase = Phase::kStandby;
  // ★ 未解锁 ⟹ MANUAL；解锁且守卫仍成立（只可能来自 `own_arming` 那条缝，ticket 08 会做成真的）
  //   ⟹ AUTO：我们正在发会驱动飞机（已解锁的 offboard）的指令。
  d.authority = guards.armed ? Authority::kAuto : Authority::kManual;
  d.stream_setpoint = true;
  ever_streamed_ = true;

  if (!streaming_) {
    streaming_ = true;
    stream_started_at_s_ = now_s;
  }

  d.request_offboard =
    !guards.in_offboard && (now_s - stream_started_at_s_) >= offboard_prestream_s_;
  return record(d);
}

inline double Fsm::streaming_seconds(double now_s) const
{
  return streaming_ ? (now_s - stream_started_at_s_) : 0.0;
}

}  // namespace control

#endif  // CONTROL__FSM_HPP_
