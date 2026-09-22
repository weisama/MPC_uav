// control 节点：本项目的**第一个自主飞行节点**。ticket 05 走到 `STANDBY`，ticket 06 加**失效路径**。
//
// 数据流：
//   /mavros/state ─┐
//   /mavros/local_position/pose ─┤
//   /mocap/pose_enu ─────────────┼─▶ 守卫 + 失效判据 ─▶ 状态机 ─▶ /mavros/setpoint_raw/local
//   /mavros/rc/in、/mavros/battery ┘                            ├─▶ /control/state
//                                                               └─▶ /mavros/set_mode（OFFBOARD / AUTO.LAND）
//
// ★ 本节点的职责分工（改代码前先读这条）：
//   · **判断**（该不该发 setpoint、该不该切模式）只在 `fsm.hpp`，那里零 ROS 依赖。
//   · **翻译**（话题 → 守卫/判据、决策 → 话题/服务）只在 `px4_interface.cpp`。
//   · 本文件只做**接线**：参数、定时器、日志。不要在这里加"顺手判断"——判据进了这里就没人测得到了。
//     **唯一的例外**是 `request_mode()` 里的重试节流（in-flight / 重试间隔 / 悬空请求丢弃）：
//     那是"输出侧的发送节奏"，不是"输入侧的判据"，天然属于调用方。它没有 seam-1 测试覆盖，
//     改动时请连同 `src/control/README.md` 的覆盖清单一起看。
//
// ★ 本张（ticket 06）**新增的全是"往安全方向"的动作**：
//   · 任何失效 ⟹ 立即停发 setpoint；**已解锁**时并请求 `AUTO.LAND`（未解锁时只停发 ——
//     `AUTO.LAND` 会阻止解锁，台架上切进去就解不了锁了）。
//   · 控制权被夺走（打杆 / 模式被切走）⟹ 停发 + **不抢回 OFFBOARD**（`Phase::kRcOverride`）。
//   · 本张仍**不发解锁指令**（解锁 = ticket 08），也没有起飞/跟踪（08 / 07）。
//
// ★ 运行纪律：本节点与 `mocap` 节点共用"从启动到解锁之间不要搬动飞机"的纪律。

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "control/fsm.hpp"
#include "control/px4_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

/// 降级原因的**重报**间隔（s）：degrade 持续期间每隔这么久再打一条。
/// 取 10 s 的理由与 INIT 的诊断日志一致：太密会刷屏（失效可能持续几分钟），
/// 太疏则"最后一条日志是十分钟前的"，现场翻日志时判断不出"还在不在降级"。
constexpr double kDegradeRepeatS = 10.0;

/// 把 QoS 剖面的两个关键位翻成人话 —— 判"永远收不到"这类静默故障时，这两个位就是全部信息。
std::string qos_summary(const rclcpp::QoS & q)
{
  std::string s = (q.reliability() == rclcpp::ReliabilityPolicy::BestEffort)
    ? "best_effort" : "reliable";
  if (q.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
    s += "+transient_local";
  }
  return s;
}

}  // namespace

class ControlNode : public rclcpp::Node
{
public:
  ControlNode()
  : Node("control")
  {
    // —— 控制循环 ——
    setpoint_rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 50.0);
    // ★ 切 OFFBOARD 之前必须先流多久。默认 1.0 s 来自 phase1-implementation.md §4.3-1：
    //   断流 PX4 会自动退出 OFFBOARD（COM_OF_LOSS_T，z0 现值 1.0 s）⟹ "先流够了再切"是前提不是优化。
    //   PX4 侧另有一条"进入 OFFBOARD 前 setpoint 需 ≥2 Hz"的检查，50 Hz 远高于它。
    offboard_prestream_s_ = declare_parameter<double>("offboard_prestream_s", 1.0);
    offboard_retry_s_ = declare_parameter<double>("offboard_retry_s", 1.0);
    enter_offboard_ = declare_parameter<bool>("enter_offboard", true);
    yaw_ = declare_parameter<double>("yaw", 0.0);
    diag_period_s_ = declare_parameter<double>("diag_period_s", 1.0);

    // —— 话题/服务名（多机、台架复用不改代码）——
    topics_.state = declare_parameter<std::string>("state_topic", topics_.state);
    topics_.local_position = declare_parameter<std::string>(
      "local_position_topic", topics_.local_position);
    topics_.rc = declare_parameter<std::string>("rc_topic", topics_.rc);
    topics_.battery = declare_parameter<std::string>("battery_topic", topics_.battery);
    topics_.mocap = declare_parameter<std::string>("mocap_topic", topics_.mocap);
    topics_.setpoint = declare_parameter<std::string>("setpoint_topic", topics_.setpoint);
    topics_.set_mode_service = declare_parameter<std::string>(
      "set_mode_service", topics_.set_mode_service);
    const std::string state_pub_topic = declare_parameter<std::string>(
      "control_state_topic", "/control/state");

    // —— 每条输入的超时判据（★ 逐条参数化：上游频率差两个数量级）——
    control::Timeouts timeouts;
    timeouts.state_s = declare_parameter<double>("state_timeout_s", timeouts.state_s);
    timeouts.local_pos_s = declare_parameter<double>(
      "local_pos_timeout_s", timeouts.local_pos_s);
    timeouts.mocap_s = declare_parameter<double>("mocap_timeout_s", timeouts.mocap_s);
    timeouts.rc_s = declare_parameter<double>("rc_timeout_s", timeouts.rc_s);
    timeouts.battery_s = declare_parameter<double>("battery_timeout_s", timeouts.battery_s);
    timeouts_ = timeouts;

    // —— 失效判据的阈值（ticket 06，逐条出处见 `px4_interface.hpp` 的 `Limits`）——
    control::Limits limits;
    limits.low_voltage_v = declare_parameter<double>("low_voltage_v", limits.low_voltage_v);
    limits.rc_stick_ov_us_per_s = declare_parameter<double>(
      "rc_stick_ov_us_per_s", limits.rc_stick_ov_us_per_s);
    limits.mocap_stamp_stall_s = declare_parameter<double>(
      "mocap_stamp_stall_s", limits.mocap_stamp_stall_s);
    limits.rc_map_roll = declare_parameter<int>("rc_map_roll", limits.rc_map_roll);
    limits.rc_map_pitch = declare_parameter<int>("rc_map_pitch", limits.rc_map_pitch);
    limits.rc_map_throttle = declare_parameter<int>("rc_map_throttle", limits.rc_map_throttle);
    limits.rc_map_yaw = declare_parameter<int>("rc_map_yaw", limits.rc_map_yaw);
    limits_ = limits;

    // ★★ **ticket 06 / 08 之间的那条缝**：本张没有解锁路径（解锁 = ticket 08），
    //    但"已解锁时的失效该怎么处置"（请求 AUTO.LAND）与"允许本节点解锁"这两件事
    //    在守卫里是同一个条件（`Guard::kArmingByUs`）。所以这里用一个参数把"这次解锁
    //    是（当作）本节点发起的"告诉状态机：台架注入 armed 时置 true 才能走到 LAND 分支。
    //    ticket 08 落地时**删掉这个参数**，改成由解锁服务的结果驱动。
    own_arming_ = declare_parameter<bool>("own_arming", false);
    // 主动中止任务用的模式名（mavros `set_mode` 的 `custom_mode`，PX4 的 `AUTO.LAND`）。
    land_mode_ = declare_parameter<std::string>("land_mode", "AUTO.LAND");

    fsm_ = std::make_unique<control::Fsm>(offboard_prestream_s_);
    // ★ 边界层的配置一次交齐：话题/超时/阈值/**模式名**/解锁来源 ——
    //   后两项也在这里，是为了让 `Guards` 的每一条都在边界层算完（`guards()` 的注释）。
    //   `modes.offboard` 保持默认（"OFFBOARD" 是本栈固定的口径，本张没有为它开参数；
    //   而 `land_mode` 是可换的，ticket 06 的验收项点名要它可配）。
    control::Px4Config px4_config;
    px4_config.topics = topics_;
    px4_config.timeouts = timeouts_;
    px4_config.limits = limits_;
    px4_config.modes.land = land_mode_;
    px4_config.own_arming = own_arming_;
    px4_ = std::make_unique<control::Px4Interface>(*this, px4_config);

    // 状态串用 transient_local：晚订阅的 `ros2 topic echo`、晚起的测试都能立刻拿到当前值，
    // 不必"等它变化"。（另外每 diag_period_s 还会心跳重发一次，见 publish_state。）
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_pub_topic, rclcpp::QoS(10).transient_local());

    RCLCPP_INFO(
      get_logger(),
      "control 节点启动：setpoint %.1f Hz → %s；/control/state → %s\n"
      "  前置流 %.1f s 后%s切 OFFBOARD（重试间隔 %.1f s）；setpoint：位置 = 当前位姿、yaw = %.3f rad\n"
      "  超时判据：state %.1f s（★ 该话题实测 1 Hz）、local_position %.1f s、mocap %.1f s、rc %.1f s、battery %.1f s\n"
      "  失效判据：低压 < %.1f V、摇杆变化率 > %.1f µs/s（≈ COM_RC_STICK_OV = 20）、"
      "动捕时间戳不前进 > %.1f s\n"
      "  ★ 任何失效 ⟹ 立即停发 setpoint；**已解锁**时并请求 %s；打杆/模式被夺走 ⟹ 停发且不抢回 OFFBOARD\n"
      "  ★ 本张不发解锁指令（解锁 = ticket 08）；own_arming = %s（06/08 之间的缝，见参数注释）",
      setpoint_rate_hz_, topics_.setpoint.c_str(), state_pub_topic.c_str(),
      offboard_prestream_s_, enter_offboard_ ? "自动" : "**不**（enter_offboard = false）",
      offboard_retry_s_, yaw_,
      timeouts_.state_s, timeouts_.local_pos_s, timeouts_.mocap_s,
      timeouts_.rc_s, timeouts_.battery_s,
      limits_.low_voltage_v, limits_.rc_stick_ov_us_per_s, limits_.mocap_stamp_stall_s,
      land_mode_.c_str(), own_arming_ ? "true" : "false");

    if (!enter_offboard_) {
      RCLCPP_WARN(
        get_logger(),
        "enter_offboard = false ⟹ 只流 setpoint、不切 OFFBOARD。"
        "此时真 PX4 永远不会接受这些 setpoint（它只在 OFFBOARD 模式下读 setpoint_raw）。");
    }

    if (own_arming_) {
      RCLCPP_WARN(
        get_logger(),
        "own_arming = true ⟹ 任何 `armed` 上升沿都被当作**本节点发起的解锁**（不再报『外部解锁』）。\n"
        "  ★ 这是 ticket 06 与 08 之间的临时缝，**只用于台架**（节点自己并不发解锁指令）。\n"
        "  实飞前请保持 false，直到 ticket 08 用真实的解锁结果替换它。");
    }

    const double period_s = 1.0 / std::max(1.0, setpoint_rate_hz_);
    tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(period_s), [this]() {tick();});
    diag_timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.1, diag_period_s_)), [this]() {diagnose();});
  }

private:
  // ───────────────────────────── 控制循环（每次 setpoint_rate_hz 一次） ─────────────────────────────

  void tick()
  {
    const double now_s = now().seconds();
    mode_before_ = px4_->mode();   // 只在日志里用：说清"模式从哪儿变到哪儿"
    const control::Guards guards = px4_->guards(now_s);
    const control::Health health = px4_->health(now_s);
    const control::Decision d = fsm_->update(guards, health, now_s);

    detect_external_arming(guards);

    if (d.stream_setpoint) {
      const auto current = px4_->local_position();
      if (current) {
        px4_->publish_setpoint(*current, static_cast<float>(yaw_));
      }
      // current 为空只可能发生在"本帧守卫刚成立、但守卫要求 local_position 新鲜"的竞态里 ——
      // 实际上 guards.local_pos_fresh ⟹ 收到过至少一帧。留这个分支是为了不把"漏发一帧"变成段错误。
    }

    if (d.request_offboard && enter_offboard_) {
      request_mode("OFFBOARD", "前置流已满足", now_s);
    }
    if (d.request_land) {
      // 主动中止任务。★ `d.request_land` 为真 ⟹ 失效已经发生（见 `fsm.hpp` ⑤），
      //   这里只负责发请求与记账，不做任何判断。
      request_mode(land_mode_, "失效降级", now_s);
    }

    publish_state(d, now_s);
    maybe_log_degrade(d);
    last_phase_ = d.phase;
  }

  // ★ 这里曾经有一个 `current_guards()`：`Guards` 由边界层算，节点再往上补 `own_arming` 与
  //   `in_land`。它是个**活生生的 bug 源** —— 诊断文案那条路（`missing_guards`）忘了走它，
  //   于是 own_arming = true 且已解锁时，日志报的守卫与状态机看见的不是同一份。
  //   2026-09-22 复核时把两个字段都搬进了 `Px4Interface::guards()`（连同模式名配置），
  //   本函数随之删除：**节点不再自己翻译任何守卫**。

  /// 「外部解锁」告警 —— 上升沿只报一次。
  /// ★ 依据 `params-changes.md` §H.6（第一手）：**若 `armed` 的变化不是我们造成的 ⟹ 停发 setpoint
  ///   + 告警**。本张没有任何解锁路径（解锁归 ticket 08）⟹ 本张看到的**任何** `armed` 上升沿
  ///   都不是我们造成的。停发由守卫（`!guards.armed`）自动完成，这里只负责"让人看见"。
  /// ⚠️ 注意检出最多晚 1 s（`/mavros/state` 实测 1 Hz）—— 这是上游限制，不是本节点的缺陷。
  void detect_external_arming(const control::Guards & guards)
  {
    if (guards.armed && !last_armed_ && !own_arming_) {
      RCLCPP_ERROR(
        get_logger(),
        "检测到**外部解锁**（%s 报 armed = true，而本节点不发送任何解锁指令）⟹ 立即停发 setpoint。\n"
        "  请确认是谁解锁的（遥控器 / QGC / 别的进程）。恢复过程：先查明原因，再重新满足前置条件。\n"
        "  ★ 检出滞后最多 1 s：/mavros/state 只在收到 MAVLink HEARTBEAT 时才发，实测 1.000 Hz。",
        topics_.state.c_str());
    }
    last_armed_ = guards.armed;
  }

  /// 请求切换飞行模式（`OFFBOARD` / `AUTO.LAND`），并按 `offboard_retry_s` 重试。
  /// ⚠️ 服务回调的 ok **不代表 PX4 接受了**（`mode_sent` 只说消息发出去了）⟹ 真正的判据是
  ///    `/mavros/state.mode` 变成期望值（1 Hz ⟹ 最多晚 1 s）。所以这里不看回调结果，
  ///    而是每 `offboard_retry_s` 重发一次，直到状态机那边看见模式已到位（`in_offboard` / `in_land`）
  ///    —— **决定"还要不要重试"的是状态机**，本函数只负责发与节流。
  /// @param why 人话说明"为什么现在切"（进日志；两种来源：前置流满足 / 失效降级）
  void request_mode(const std::string & mode, const char * why, double now_s)
  {
    if (mode_request_in_flight_) {
      if (pending_mode_ != mode) {
        // ★ 上一次请求的模式已经过时（新的决策要另一个模式）⟹ 不等它了。
        //   典型场面：刚发出 OFFBOARD 请求，紧接着判出失效要 `AUTO.LAND` ——
        //   等一个不再需要的应答只会推迟降落。
        RCLCPP_WARN(
          get_logger(), "上一次对 %s 的切模式请求尚无应答，但新的决策要切 %s ⟹ 不等它了",
          pending_mode_.c_str(), mode.c_str());
        mode_request_in_flight_ = false;
      } else {
        // ⚠️ 上一次请求**永远没回来**（mavros 挂了/被重启/服务端消失）时，rclcpp 不会超时它
        //   （`rclcpp/client.hpp`：没有 response 就永远悬着）⟹ 没有这个分支就会永久卡住，
        //   同时日志还在说"每 offboard_retry_s 重试"，是最难查的那种"说到做不到"。
        //   判据取 2×重试间隔（下限 2 s），比正常往返（毫秒级）宽松得多，不会误杀。
        const double stale_after_s = std::max(2.0, 2.0 * offboard_retry_s_);
        if ((now_s - last_mode_request_s_) < stale_after_s) {
          return;  // 上一次还在合理的往返时间内，别叠加
        }
        RCLCPP_WARN(
          get_logger(),
          "上一次切模式请求过了 %.1f s 仍无回应（mavros 挂了或重启了？）⟹ 丢弃它，本次重新发起",
          now_s - last_mode_request_s_);
        mode_request_in_flight_ = false;
      }
    }
    if (mode_requested_once_ && mode == pending_mode_ &&
      (now_s - last_mode_request_s_) < offboard_retry_s_) {
      return;
    }

    const bool first = !mode_requested_once_ || mode != pending_mode_;
    mode_request_in_flight_ = true;
    mode_requested_once_ = true;
    pending_mode_ = mode;
    last_mode_request_s_ = now_s;

    px4_->request_set_mode(
      mode,
      [this, first, mode, why](bool ok, const std::string & detail) {
        mode_request_in_flight_ = false;
        if (ok) {
          if (first) {
            RCLCPP_INFO(
              get_logger(),
              "已请求切 %s（%s；流了 %.2f s）；%s。★ 等 /mavros/state.mode 变成 %s 才算切成功"
              "（该话题 1 Hz ⟹ 最多晚 1 s），未成则每 %.1f s 重试。",
              mode.c_str(), why, fsm_->streaming_seconds(now().seconds()), detail.c_str(),
              mode.c_str(), offboard_retry_s_);
          }
        } else {
          // 服务不可用/被拒：按重试间隔再来。真机上最常见的原因是 mavros 还没起完；
          // `AUTO.LAND` 被拒还有一个具体原因：EKF2 没有 local position（它进不去）。
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "切 %s 失败：%s（每 %.1f s 重试）", mode.c_str(), detail.c_str(), offboard_retry_s_);
        }
      });
  }

  // ───────────────────────────── 状态发布 ─────────────────────────────

  /// `/control/state` = `"<控制权>/<任务态>"`（如 `MANUAL/STANDBY`）。
  /// 变化时立刻发；另外每 diag_period_s 心跳重发一次（话题本身是 transient_local，
  /// 心跳是为了让"用默认 QoS 订阅"的工具（例如随手 `ros2 topic hz`）也能看到）。
  void publish_state(const control::Decision & d, double now_s)
  {
    const std::string s = std::string(control::to_string(d.authority)) + "/" +
      control::to_string(d.phase);
    const bool changed = (s != last_state_str_);
    if (!changed && (now_s - last_state_pub_s_) < diag_period_s_) {
      return;
    }
    last_state_pub_s_ = now_s;

    std_msgs::msg::String msg;
    msg.data = s;
    state_pub_->publish(msg);

    if (changed) {
      log_transition(d, s);
      last_state_str_ = s;
    }
  }

  /// 状态**变化**时的日志。★ 每个任务态各说各的"这意味着飞机现在怎样了" ——
  /// 一句"前置条件未满足"套在所有回退上是查不下去的（2026-09-22 之前的写法）。
  /// ⚠️ 故意不写 `default:`：新增一个 `Phase` 而漏掉文案时 `-Wswitch` 直接编译失败。
  void log_transition(const control::Decision & d, const std::string & s)
  {
    last_init_log_s_ = 0.0;  // 下次退回 INIT 时立刻打一次诊断
    switch (d.phase) {
      case control::Phase::kStandby:
        RCLCPP_INFO(
          get_logger(),
          "→ %s：开始以 %.1f Hz 发布 setpoint（位置 = 当前位姿、yaw = %.3f）。"
          "前置条件全成立（飞控已连接 ∧ 动捕有数据 ∧ local_position 有效 ∧ arm 守卫成立）。",
          s.c_str(), setpoint_rate_hz_, yaw_);
        return;
      case control::Phase::kLand:
        // 这条路径的完整说明（含原因与复位条件）由上一条 `degrade=` 日志给出，这里只说状态含义。
        RCLCPP_ERROR(
          get_logger(),
          "→ %s：**已停止发布 setpoint**，并请求 %s 主动中止任务（原因见上一行 degrade=）。\n"
          "  交给 PX4 自己兜底的部分：断流超过 COM_OF_LOSS_T（z0 现值 1.0 s）后它自行退出 OFFBOARD。\n"
          "  ★ 复位条件：**落地上锁**（armed → false）后自动复位，不需要重启节点。",
          s.c_str(), land_mode_.c_str());
        return;
      case control::Phase::kRcOverride:
        RCLCPP_WARN(
          get_logger(),
          "→ %s：**控制权被夺走** —— 已停止发布 setpoint、已停止请求任何模式（原因见上一行 degrade=）。\n"
          "  ★ 本节点**不会主动抢回** OFFBOARD（否则与飞行员一秒一次地拉锯）。\n"
          "  ★ 复位条件：有人把模式放回 OFFBOARD，或落地上锁；对打杆夺权则是摇杆回中且模式被交回。",
          s.c_str());
        return;
      case control::Phase::kInit:
        RCLCPP_WARN(
          get_logger(),
          "→ %s：**已停止发布 setpoint**（缺：%s）\n"
          "  交给 PX4 自己的失效保护兜底：断流超过 COM_OF_LOSS_T（z0 现值 1.0 s）后它自行退出 OFFBOARD。",
          s.c_str(), describe_missing(now().seconds()).c_str());
        return;
      case control::Phase::kTakeoff:
      case control::Phase::kHover:
      case control::Phase::kTrack:
        // 本张（ticket 06）到不了这里：起飞 / 悬停 = ticket 08，跟踪 = ticket 07。
        RCLCPP_WARN(get_logger(), "→ %s：该任务态本张（ticket 06）还未实现", s.c_str());
        return;
    }
  }

  /// 降级/夺权原因的日志（ticket 06 要求"降级原因在日志里**可检索**"）。
  /// ★ 唯一载体是这一行里的 `degrade=<token>`（token 由 `fsm.hpp::to_string(Degrade)` 给，
  ///   与状态机同源 ⟹ 日志里搜到的原因与代码里的判据永远对得上）。三种时机各打一次：
  ///   · 原因**变了**（含某原因 ⟹ none，即"已复位"）；
  ///   · 持续期间每 `kDegradeRepeatS` 重报一次 —— 别让"还在降级"随时间从日志里消失
  ///     （现场最常见的问题是"翻了半天日志，最后一条是十分钟前的"）。
  void maybe_log_degrade(const control::Decision & d)
  {
    const std::string token = control::to_string(d.degrade);
    const double now_s = now().seconds();

    if (token == last_degrade_token_) {
      if (d.degrade != control::Degrade::kNone &&
        (now_s - last_degrade_log_s_) >= kDegradeRepeatS) {
        log_degrade(d.degrade, token, now_s);
      }
      return;
    }

    const std::string previous = last_degrade_token_;
    last_degrade_token_ = token;
    if (d.degrade == control::Degrade::kNone) {
      RCLCPP_INFO(
        get_logger(),
        "degrade=none：降级已复位（此前 degrade=%s，持续 %.1f s）—— %s",
        previous.empty() ? "（无）" : previous.c_str(), now_s - degrade_started_s_,
        fsm_->latched() ? "但闩锁仍在（等落地上锁，或模式被交回）" : "判据已恢复");
      return;
    }
    degrade_started_s_ = now_s;
    log_degrade(d.degrade, token, now_s);
  }

  void log_degrade(control::Degrade reason, const std::string & token, double now_s)
  {
    last_degrade_log_s_ = now_s;
    RCLCPP_ERROR(
      get_logger(),
      "degrade=%s（持续 %.1f s%s）—— %s\n  · 证据：%s",
      token.c_str(), now_s - degrade_started_s_,
      fsm_->latched() ? "，**已闩锁**：不会自己恢复" : "，未闩锁：判据一恢复就自动复位",
      describe_degrade(reason).c_str(), describe_evidence(reason, now_s).c_str());
  }

  /// 判据的**证据**（读数，而不是标签）—— 告警里写"动捕超时"没用，
  /// 要写"静默 3.2 s、话题上有 1 个发布者"。这是"日志能自己站住"的最小要求。
  /// ⚠️ 故意不写 `default:`：新增一条 `Degrade` 而漏掉证据时 `-Wswitch` 直接编译失败。
  std::string describe_evidence(control::Degrade d, double now_s) const
  {
    switch (d) {
      case control::Degrade::kRcOverride:
        return "遥控器新鲜（" + age_str(topics_.rc, now_s) + "）且摇杆变化率超过 " +
               std::to_string(limits_.rc_stick_ov_us_per_s) + " µs/s";
      case control::Degrade::kModeLost:
        return "模式从「" + mode_before_ + "」变成了「" + px4_->mode() + "」";
      case control::Degrade::kStateTimeout:
        return topics_.state + " 静默 " + age_str(topics_.state, now_s) + "（超时判据 " +
               std::to_string(timeouts_.state_s) + " s）；发布者：" + publisher_summary(topics_.state);
      case control::Degrade::kMocapTimeout:
        return topics_.mocap + " 静默 " + age_str(topics_.mocap, now_s) + "；发布者：" +
               publisher_summary(topics_.mocap);
      case control::Degrade::kLocalPosTimeout:
        return topics_.local_position + " 静默 " + age_str(topics_.local_position, now_s) +
               "；发布者：" + publisher_summary(topics_.local_position);
      case control::Degrade::kRcTimeout:
        return topics_.rc + " 静默 " + age_str(topics_.rc, now_s) + "；发布者：" +
               publisher_summary(topics_.rc);
      case control::Degrade::kBatteryLow: {
        const auto v = px4_->battery_voltage();
        return topics_.battery + " 报电压 " +
               (v ? std::to_string(*v) : std::string("（无可用读数）")) + " V（阈值 " +
               std::to_string(limits_.low_voltage_v) + " V）；该通道静默 " +
               age_str(topics_.battery, now_s);
      }
      case control::Degrade::kPositionInvalid:
        return topics_.local_position + " 最近一帧含 NaN/Inf（EKF2 位置失效）";
      case control::Degrade::kMocapStampStalled:
        return topics_.mocap + " 有帧（静默 " + age_str(topics_.mocap, now_s) +
               "）但时间戳连续不前进超过 " + std::to_string(limits_.mocap_stamp_stall_s) + " s";
      case control::Degrade::kNone:
        break;
    }
    return "（无）";
  }


  /// 每条降级原因：**它意味着什么 + 怎么复位**。★ 这张表就是"日志可检索"的正文。
  /// ⚠️ 故意不写 `default:`：新增一条 `Degrade` 而漏掉文案时 `-Wswitch` 直接编译失败。
  std::string describe_degrade(control::Degrade d) const
  {
    switch (d) {
      case control::Degrade::kRcOverride:
        return "遥控器在打杆（已解锁）⟹ 飞行员接管。已停发且**不请求任何模式**；"
               "复位：摇杆回中且模式被交回 OFFBOARD，或落地上锁";
      case control::Degrade::kModeLost:
        return "本节点持有的模式（OFFBOARD / 已请求的 " + land_mode_ + "）被外部切走。"
               "已停发且**不抢回**；复位：有人把模式放回 OFFBOARD，或落地上锁";
      case control::Degrade::kStateTimeout:
        return "飞控失联（" + topics_.state + " 超时）⟹ 已停发；已解锁时会请求 " + land_mode_ +
               "（但飞控失联时它多半收不到，此时靠 PX4 自己的失效保护）";
      case control::Degrade::kMocapTimeout:
        return "动捕无数据 ⟹ EKF2 的视觉位置会随时间失效；已停发；已解锁时请求 " + land_mode_ +
               "（EKF2 位置失效时 PX4 会**拒绝**它，见 ticket 06 的保留说明）";
      case control::Degrade::kLocalPosTimeout:
        return "local_position 静默 ⟹ 没有可用的位姿可发；已停发；已解锁时请求 " + land_mode_;
      case control::Degrade::kRcTimeout:
        return "遥控器静默（**只在已解锁时算失效**）⟹ 已停发；已解锁时请求 " + land_mode_ +
               "。★ 注意 PX4 侧 `NAV_RCL_ACT` 现值 2 = 返航（A1 阻塞项）";
      case control::Degrade::kBatteryLow:
        return "电压低于阈值 ⟹ 主动中止任务：已停发并请求 " + land_mode_ +
               "；复位：落地上锁（换电池 / 充电后即可）";
      case control::Degrade::kPositionInvalid:
        return "EKF2 给出了 NaN/Inf 位置（位置估计失效）⟹ 已停发；已解锁时请求 " + land_mode_ +
               "（很可能被拒 —— 它自己也需要位置）";
      case control::Degrade::kMocapStampStalled:
        return "动捕有帧但**时间戳不前进** ⟹ mavros/EKF2 都会丢弃这类位姿（话题层看不出来）；"
               "已停发；已解锁时请求 " + land_mode_ + "；复位：时间戳恢复前进";
      case control::Degrade::kNone:
        break;
    }
    return "（无）";
  }

  // ───────────────────────────── 诊断 ─────────────────────────────

  /// 50 Hz 的 tick 不发日志，诊断走这个低频定时器 ——
  /// 判据可查（谁断了、断多久、话题图上有没有发布者、QoS 对不对）比"日志里什么都没有"重要得多。
  void diagnose()
  {
    const double now_s = now().seconds();
    if (last_phase_ == control::Phase::kInit) {
      if (last_init_log_s_ <= 0.0 || (now_s - last_init_log_s_) >= 10.0) {
        last_init_log_s_ = now_s;
        RCLCPP_WARN(
          get_logger(),
          "仍在 INIT，前置条件未满足：\n%s\n  （守卫：%s；每 10 s 重报一次）",
          describe_inputs(now_s).c_str(), describe_missing(now_s).c_str());
      }
    }
    // 运行中"某条输入静默"由 tick 的状态回退 + `degrade=` 日志负责，这里不重复。
    // ★ 但**闩锁**要在这里说一声：闩锁期间状态机不会自己恢复，操作员需要知道"怎么退出去"
    //   （否则会以为是节点卡住了，去重启它 —— 而那正好是 §4.4 想避免的）。
    if (fsm_->latched() && last_phase_ != control::Phase::kInit) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "失效已闩锁（degrade=%s，任务态 %s）⟹ **不会自己恢复**。复位方式：%s",
        control::to_string(fsm_->latched_reason()), control::to_string(last_phase_),
        describe_reset(fsm_->latched_reason()).c_str());
    }
  }

  /// 每条降级原因的**复位条件**（与 `docs/control/phase1-implementation.md` §4.4 同源）。
  /// ⚠️ 故意不写 `default:`：新增一条 `Degrade` 而漏掉复位说明时 `-Wswitch` 直接编译失败。
  static std::string describe_reset(control::Degrade d)
  {
    switch (d) {
      case control::Degrade::kRcOverride:
      case control::Degrade::kModeLost:
        // 夺权类：外部把模式放回 OFFBOARD，或落地上锁。
        return "有人把模式放回 OFFBOARD（外部动作），或落地上锁（armed → false）";
      case control::Degrade::kStateTimeout:
      case control::Degrade::kMocapTimeout:
      case control::Degrade::kLocalPosTimeout:
      case control::Degrade::kRcTimeout:
      case control::Degrade::kBatteryLow:
      case control::Degrade::kPositionInvalid:
      case control::Degrade::kMocapStampStalled:
        // 输入类：落地上锁才复位（未解锁时则是"判据恢复即自动复位"，不会闩锁）。
        return "落地上锁（armed → false）；未解锁时的降级不闩锁，判据恢复即自动复位";
      case control::Degrade::kNone:
        break;
    }
    return "（无）";
  }

  /// 缺了哪些前置条件 —— 告警正文就是判据本身，不写"前置条件未满足"这种查不下去的话。
  /// ★ **"缺哪几条"由 `missing_guards()` 说了算**，本函数只负责把每条翻成带诊断信息的中文。
  ///   这样"状态机为什么没进 STANDBY"与"日志说缺什么"永远是同一份判据。
  std::string describe_missing(double now_s) const
  {
    const auto missing = control::missing_guards(px4_->guards(now_s));
    if (missing.empty()) {
      return "无（前置条件成立）";
    }
    std::string s;
    for (size_t i = 0; i < missing.size(); ++i) {
      s += (i ? "；" : "") + describe_guard(missing[i], now_s);
    }
    return s;
  }

  /// ⚠️ 这里**故意不写 `default:`** —— 新增一条 `Guard` 而漏掉这份文案时，`-Wswitch` 直接编译失败。
  std::string describe_guard(control::Guard which, double now_s) const
  {
    switch (which) {
      case control::Guard::kConnected:
        return "飞控未连接（" + topics_.state + " 静默 " + age_str(topics_.state, now_s) +
               " 或 connected = false）";
      case control::Guard::kMocapFresh:
        return "动捕无数据（" + topics_.mocap + " 静默 " + age_str(topics_.mocap, now_s) + "）";
      case control::Guard::kLocalPosFresh:
        return "local_position 无效（" + topics_.local_position + " 静默 " +
               age_str(topics_.local_position, now_s) + "）";
      case control::Guard::kArmingByUs:
        // ★ 语义是"**未解锁** ∨ 解锁是本节点发起的"，所以缺项的读法是"别人解锁了"。
        return own_arming_ ? "（不可达：own_arming = true 时这条守卫恒成立）"
                           : "**已解锁**（本张没有解锁路径 ⟹ 这是外部解锁，见 ERROR 日志）";
    }
    return "?";  // 不可达：上面的 switch 已覆盖全部枚举
  }

  /// 每条输入一行：话题、静默多久、超时判据、**话题图上有什么发布者（含 QoS）**。
  /// ★ QoS 那一列是刻意加的：`best_effort` 发布方配 `reliable` 订阅方**不报错**，只是永远收不到，
  ///   而这一列能让那类故障一眼看穿（local_position / battery 在真 mavros 上是 best_effort）。
  std::string describe_inputs(double now_s) const
  {
    struct Row
    {
      const char * label;
      const std::string * topic;
    };
    const Row rows[] = {
      {"state", &topics_.state},
      {"local_position", &topics_.local_position},
      {"mocap", &topics_.mocap},
      {"rc", &topics_.rc},
      {"battery", &topics_.battery},
    };
    std::string s;
    for (const auto & row : rows) {
      s += "    · " + std::string(row.label) + "  " + *row.topic + "\n";
      s += "        静默 " + age_str(*row.topic, now_s) + "；发布者：" +
        publisher_summary(*row.topic) + "\n";
    }
    return s;
  }

  std::string age_str(const std::string & topic, double now_s) const
  {
    const double age = px4_->age_seconds(topic, now_s);
    if (age < 0.0) {
      return "从未收到过";
    }
    return std::to_string(age) + " s";
  }

  std::string publisher_summary(const std::string & topic) const
  {
    const auto infos = get_publishers_info_by_topic(topic);
    if (infos.empty()) {
      return "**0 个**（这个话题上没有任何发布者）";
    }
    std::string s = std::to_string(infos.size()) + " 个";
    for (size_t i = 0; i < infos.size(); ++i) {
      s += (i ? ", " : "（") + qos_summary(infos[i].qos_profile());
    }
    return s + "）";
  }

  // ───────────────────────────── 成员 ─────────────────────────────

  double setpoint_rate_hz_{50.0};
  double offboard_prestream_s_{1.0};
  double offboard_retry_s_{1.0};
  bool enter_offboard_{true};
  double yaw_{0.0};
  double diag_period_s_{1.0};
  /// ★ 06/08 之间的缝：把"这次解锁当作本节点发起的"告诉状态机（见构造里的注释）。
  bool own_arming_{false};
  std::string land_mode_{"AUTO.LAND"};

  control::Px4Topics topics_;
  control::Timeouts timeouts_;
  control::Limits limits_;

  std::unique_ptr<control::Fsm> fsm_;
  std::unique_ptr<control::Px4Interface> px4_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  std::string last_state_str_;
  control::Phase last_phase_{control::Phase::kInit};
  double last_state_pub_s_{0.0};
  double last_init_log_s_{0.0};
  bool last_armed_{false};
  /// 上一拍的 `/mavros/state.mode`（只给日志用：说清"模式从哪儿变到哪儿"）。
  std::string mode_before_;

  // —— 模式请求的节流状态（`request_mode()`；输出侧的发送节奏，不是判据）——
  bool mode_request_in_flight_{false};
  bool mode_requested_once_{false};
  /// 最近一次请求的模式 —— 与 `in_flight` 一起决定"还要不要等这个应答"。
  std::string pending_mode_;
  double last_mode_request_s_{0.0};

  // —— 降级日志（`maybe_log_degrade()`）——
  /// 上一次报出来的 `degrade=` token（与 `fsm.hpp::to_string(Degrade)` 同源）。
  /// ★ 初值就是 `none`（同一处 `to_string` 给的字面量）而不是空串：否则**启动后第一拍**会因为
  ///   `token != ""` 而走进"已复位"分支，打出一条莫名其妙的 `degrade=none：降级已复位
  ///   （此前 degrade=（无））` —— 现场翻日志时会以为刚刚真的复位过什么。
  std::string last_degrade_token_{control::to_string(control::Degrade::kNone)};
  double last_degrade_log_s_{0.0};
  double degrade_started_s_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
