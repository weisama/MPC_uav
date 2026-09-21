// control 的状态机 —— ★ 两个正交维度显式分开，不压成扁平枚举。
//
//   控制权：  MANUAL ──(本项目开始发会驱动飞机的指令)──▶ AUTO ──(RC 打杆 / 超时 / 异常)──▶ MANUAL
//   任务态：  INIT ─▶ STANDBY ─▶ TAKEOFF ─▶ HOVER ─▶ TRACK ─▶ LAND
//
// ★ 本文件**零 ROS 依赖**（自带 POD 类型，不 include 任何 ROS 头）—— 与 `mocap` 的
//   `mocap_rel_pose.hpp` 同一个理由：状态转移条件是"飞起来才发现错"的重灾区，
//   而它在话题层面没有诊断分辨率。ROS 边界（话题进出、服务调用）全在 `control_node.cpp`，
//   本文件只回答"给定这组守卫，现在该处于哪个状态、该不该发指令"。
//
// ★ 本张（ticket 05）只实现 INIT 与 STANDBY 两个任务态；TAKEOFF/HOVER/TRACK/LAND 先占位，
//   由 ticket 08（起飞/悬停/降落）、07（轨迹）、06（失效路径）填。占位态**不在转移表里**，
//   本张任何输入组合都到不了它们。

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
  kInit,      ///< 上电：等前置（mavros 连接 / 动捕有数据 / local_position 有效）；**不发 setpoint**
  kStandby,   ///< 未解锁；**已开始以 setpoint_rate_hz 持续发布 setpoint**（位置 = 当前，yaw = 0）
  kTakeoff,   ///< 占位（ticket 08）
  kHover,     ///< 占位（ticket 08）
  kTrack,     ///< 占位（ticket 07）
  kLand,      ///< 占位（ticket 06/08）
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
  kNotArmed,      ///< 未解锁（本张没有解锁路径 ⟹ 解锁一律是外部解锁）
};

/// 守卫的**全部成员**。★ 只有这一处列举 ⟹ 加一条守卫时不必去找散落的 `&&` 级联。
inline const std::vector<Guard> & all_guards()
{
  static const std::vector<Guard> kAll{
    Guard::kConnected, Guard::kMocapFresh, Guard::kLocalPosFresh, Guard::kNotArmed};
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
    case Guard::kNotArmed:      return !g.armed;
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

/// 本张（ticket 05）的放行判据 = 「四条守卫全成立」。
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

/// 决策 —— 由 ROS 边界执行：边界层不判断，只按这四个字段行事。
struct Decision
{
  Phase phase{Phase::kInit};
  Authority authority{Authority::kManual};
  bool stream_setpoint{false};   ///< 是否发布 setpoint（这就是 INIT 与 STANDBY 的唯一区别）
  bool request_offboard{false};  ///< 是否该请求 OFFBOARD 模式（已连续流够时长、且当前不在 OFFBOARD）
};

class Fsm
{
public:
  /// @param offboard_prestream_s 切进 OFFBOARD 之前 setpoint 必须先连续流多久。
  ///        见 `phase1-implementation.md` §4.3-1：**断流 PX4 会自动退出 OFFBOARD**
  ///        （`COM_OF_LOSS_T`，z0 现值 1.0 s），所以"先流够了再切"不是优化，是前提。
  explicit Fsm(double offboard_prestream_s);

  /// @param now_s 单调递增的秒（边界层给 `rclcpp::Node::now()`，本文件只用差值）
  Decision update(const Guards & guards, double now_s);

  /// 观测口（日志/测试用）：当前这轮连续流了多久（没在流时为 0）。
  double streaming_seconds(double now_s) const;

private:
  double offboard_prestream_s_;
  bool streaming_{false};
  double stream_started_at_s_{0.0};
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
    case Phase::kStandby: return "STANDBY";
    case Phase::kTakeoff: return "TAKEOFF";
    case Phase::kHover:   return "HOVER";
    case Phase::kTrack:   return "TRACK";
    case Phase::kLand:    return "LAND";
    case Phase::kInit:    break;
  }
  return "INIT";
}

inline Fsm::Fsm(double offboard_prestream_s)
: offboard_prestream_s_(offboard_prestream_s) {}

inline Decision Fsm::update(const Guards & guards, double now_s)
{
  Decision d;

  // ★ 本张（ticket 05）的守卫 = 「四条全成立才进 STANDBY」，缺一条就退回 INIT 并且**不发 setpoint**。
  //   为什么"不发"是对的：STANDBY 的 setpoint 是"原地保持"，一旦我们能观测到的输入不可信，
  //   继续发就等于拿可能已经失效的状态去驱动飞机。停发之后由 PX4 自己的失效保护兜底
  //   （断流 `COM_OF_LOSS_T` 后退出 OFFBOARD，按 `COM_OBL_RC_ACT = 0` 回 Position 模式）。
  //
  // ★ `!guards.armed` 这一条来自 `params-changes.md` §H.6（第一手）：
  //   **"若 `armed` 的变化不是我们造成的 ⟹ 停发 setpoint + 告警"**。本张没有解锁命令
  //   （解锁归 ticket 08），所以本张看到 `armed` 上升沿一律是"不是我们造成的"。
  //   ⚠️ ticket 08 会把这一条改掉（那时 `armed` 是 TAKEOFF 的触发条件，不再是异常）—— 改这里时
  //   请连同 §H.6 一起读。
  const bool ready = ready_to_stream(guards);

  if (!ready) {
    d.phase = Phase::kInit;
    d.authority = Authority::kManual;
    d.stream_setpoint = false;
    // 断流即"这一轮连续流"结束：重新流够时长之后才允许再切 OFFBOARD。
    streaming_ = false;
    return d;
  }

  d.phase = Phase::kStandby;
  d.authority = Authority::kManual;  // 未解锁 ⟹ 还没到 AUTO，见本文件开头对 Authority 的定义
  d.stream_setpoint = true;

  if (!streaming_) {
    streaming_ = true;
    stream_started_at_s_ = now_s;
  }

  d.request_offboard =
    !guards.in_offboard && (now_s - stream_started_at_s_) >= offboard_prestream_s_;
  return d;
}

inline double Fsm::streaming_seconds(double now_s) const
{
  return streaming_ ? (now_s - stream_started_at_s_) : 0.0;
}

}  // namespace control

#endif  // CONTROL__FSM_HPP_
