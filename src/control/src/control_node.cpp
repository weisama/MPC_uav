// control 节点：本项目的**第一个自主飞行节点**。本张（ticket 05）只走到 `STANDBY`。
//
// 数据流：
//   /mavros/state ─┐
//   /mavros/local_position/pose ─┤
//   /mocap/pose_enu ─────────────┼─▶ 守卫 ─▶ 状态机 ─▶ /mavros/setpoint_raw/local（位置 + yaw）
//   /mavros/rc/in、/mavros/battery ┘        └─▶ /control/state（"<控制权>/<任务态>"）
//
// ★ 本节点的职责分工（改代码前先读这条）：
//   · **判断**（该不该发 setpoint、该不该切 OFFBOARD）只在 `fsm.hpp`，那里零 ROS 依赖。
//   · **翻译**（话题 → 守卫、决策 → 话题/服务）只在 `px4_interface.cpp`。
//   · 本文件只做**接线**：参数、定时器、日志。不要在这里加"顺手判断"——判据进了这里就没人测得到了。
//     **唯一的例外**是 `request_offboard()` 里的重试节流（in-flight / 重试间隔 / 悬空请求丢弃）：
//     那是"输出侧的发送节奏"，不是"输入侧的判据"，天然属于调用方。它没有 seam-1 测试覆盖，
//     改动时请连同 `src/control/README.md` 的覆盖清单一起看。
//
// ★ 本张**不发任何会驱动飞机的指令**：
//   · 不发解锁（解锁 = ticket 08）；setpoint 的内容是"位置 = 当前位姿" ⟹ 即便 PX4 真在 OFFBOARD
//     收到它，含义也是"原地保持"，不会让飞机动。
//   · 切 OFFBOARD 模式**不算**交出控制权：飞机未解锁 ⟹ `/control/state` 仍是 `MANUAL/*`。
//     这条语义见 README.md 与 CONTEXT.md 对"控制权 / 任务态"的定义。
//
// ★ 运行纪律：本节点与 `mocap` 节点共用"从启动到解锁之间不要搬动飞机"的纪律 —— 本张虽然不解锁，
//   但 `local_position` 的起点就是这段时间的位姿。

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

    fsm_ = std::make_unique<control::Fsm>(offboard_prestream_s_);
    px4_ = std::make_unique<control::Px4Interface>(*this, topics_, timeouts_);

    // 状态串用 transient_local：晚订阅的 `ros2 topic echo`、晚起的测试都能立刻拿到当前值，
    // 不必"等它变化"。（另外每 diag_period_s 还会心跳重发一次，见 publish_state。）
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_pub_topic, rclcpp::QoS(10).transient_local());

    RCLCPP_INFO(
      get_logger(),
      "control 节点启动：setpoint %.1f Hz → %s；/control/state → %s\n"
      "  前置流 %.1f s 后%s切 OFFBOARD（重试间隔 %.1f s）；setpoint：位置 = 当前位姿、yaw = %.3f rad\n"
      "  超时判据：state %.1f s（★ 该话题实测 1 Hz）、local_position %.1f s、mocap %.1f s、rc %.1f s、battery %.1f s\n"
      "  ★ 本张只到 STANDBY：不发解锁指令、setpoint 也不含任何位移（原地保持）",
      setpoint_rate_hz_, topics_.setpoint.c_str(), state_pub_topic.c_str(),
      offboard_prestream_s_, enter_offboard_ ? "自动" : "**不**（enter_offboard = false）",
      offboard_retry_s_, yaw_,
      timeouts_.state_s, timeouts_.local_pos_s, timeouts_.mocap_s,
      timeouts_.rc_s, timeouts_.battery_s);

    if (!enter_offboard_) {
      RCLCPP_WARN(
        get_logger(),
        "enter_offboard = false ⟹ 只流 setpoint、不切 OFFBOARD。"
        "此时真 PX4 永远不会接受这些 setpoint（它只在 OFFBOARD 模式下读 setpoint_raw）。");
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
    const control::Guards guards = px4_->guards(now_s);
    const control::Decision d = fsm_->update(guards, now_s);

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
      request_offboard(now_s);
    }

    publish_state(d, now_s);
    last_phase_ = d.phase;
  }

  /// 「外部解锁」告警 —— 上升沿只报一次。
  /// ★ 依据 `params-changes.md` §H.6（第一手）：**若 `armed` 的变化不是我们造成的 ⟹ 停发 setpoint
  ///   + 告警**。本张没有任何解锁路径（解锁归 ticket 08）⟹ 本张看到的**任何** `armed` 上升沿
  ///   都不是我们造成的。停发由守卫（`!guards.armed`）自动完成，这里只负责"让人看见"。
  /// ⚠️ 注意检出最多晚 1 s（`/mavros/state` 实测 1 Hz）—— 这是上游限制，不是本节点的缺陷。
  void detect_external_arming(const control::Guards & guards)
  {
    if (guards.armed && !last_armed_) {
      RCLCPP_ERROR(
        get_logger(),
        "检测到**外部解锁**（%s 报 armed = true，而本节点不发送任何解锁指令）⟹ 立即停发 setpoint。\n"
        "  请确认是谁解锁的（遥控器 / QGC / 别的进程）。恢复过程：先查明原因，再重新满足前置条件。\n"
        "  ★ 检出滞后最多 1 s：/mavros/state 只在收到 MAVLink HEARTBEAT 时才发，实测 1.000 Hz。",
        topics_.state.c_str());
    }
    last_armed_ = guards.armed;
  }

  /// 请求 OFFBOARD，并按 offboard_retry_s 重试。
  /// ⚠️ 服务回调的 ok **不代表 PX4 接受了**（`mode_sent` 只说消息发出去了）⟹ 真正的判据是
  ///    `/mavros/state.mode` 变成 "OFFBOARD"（1 Hz ⟹ 最多晚 1 s）。所以这里不看回调结果，
  ///    而是每 offboard_retry_s 重发一次，直到 `guards.in_offboard` 变真为止。
  void request_offboard(double now_s)
  {
    if (mode_request_in_flight_) {
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
    if (offboard_requested_once_ && (now_s - last_mode_request_s_) < offboard_retry_s_) {
      return;
    }

    mode_request_in_flight_ = true;
    last_mode_request_s_ = now_s;
    const bool first = !offboard_requested_once_;
    offboard_requested_once_ = true;

    px4_->request_set_mode(
      "OFFBOARD",
      [this, first](bool ok, const std::string & detail) {
        mode_request_in_flight_ = false;
        if (ok) {
          if (first) {
            RCLCPP_INFO(
              get_logger(),
              "已请求 OFFBOARD（流了 %.2f s）；%s。★ 等 /mavros/state.mode 变成 OFFBOARD 才算切成功"
              "（该话题 1 Hz ⟹ 最多晚 1 s），未成则每 %.1f s 重试。",
              fsm_->streaming_seconds(now().seconds()), detail.c_str(), offboard_retry_s_);
          }
        } else {
          // 服务不可用/被拒：按重试间隔再来。真机上最常见的原因是 mavros 还没起完。
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "切 OFFBOARD 失败：%s（每 %.1f s 重试）", detail.c_str(), offboard_retry_s_);
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

  void log_transition(const control::Decision & d, const std::string & s)
  {
    if (d.phase == control::Phase::kStandby) {
      RCLCPP_INFO(
        get_logger(),
        "→ %s：开始以 %.1f Hz 发布 setpoint（位置 = 当前位姿、yaw = %.3f）。"
        "前置条件全成立（飞控已连接 ∧ 动捕有数据 ∧ local_position 有效 ∧ 未解锁）。",
        s.c_str(), setpoint_rate_hz_, yaw_);
      last_init_log_s_ = 0.0;  // 下次退回 INIT 时立刻打一次诊断
    } else {
      RCLCPP_WARN(
        get_logger(),
        "→ %s：**已停止发布 setpoint**（缺：%s）\n"
        "  交给 PX4 自己的失效保护兜底：断流超过 COM_OF_LOSS_T（z0 现值 1.0 s）后它自行退出 OFFBOARD。",
        s.c_str(), describe_missing(now().seconds()).c_str());
      last_init_log_s_ = 0.0;
    }
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
    // 运行中的"某条输入静默"由 tick 的状态回退 + log_transition 的缺项清单负责，这里不重复。
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
      case control::Guard::kNotArmed:
        return "**已解锁**（本张没有解锁路径 ⟹ 这是外部解锁，见 ERROR 日志）";
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

  control::Px4Topics topics_;
  control::Timeouts timeouts_;

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
  bool mode_request_in_flight_{false};
  bool offboard_requested_once_{false};
  double last_mode_request_s_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
