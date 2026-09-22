// control 节点：本项目的**第一个自主飞行节点**。ticket 05 走到 `STANDBY`，ticket 06 加**失效路径**，
// ticket 08 加**起飞 / 悬停 / 自动降落**（`TAKEOFF → HOVER → WAIT_START → LAND`）。
//
// 数据流：
//   /mavros/state ─┐
//   /mavros/local_position/pose ─┤
//   /mavros/local_position/velocity_local ─┤
//   /mocap/pose_enu ─────────────┼─▶ 守卫 + 失效判据 + 导航读数 ─▶ 状态机 ─▶ /mavros/setpoint_raw/local
//   /mavros/rc/in、/mavros/battery ┘                                    ├─▶ /control/state
//                                                                       ├─▶ /control/takeoff（服务：受理起飞命令）
//                                                                       ├─▶ /mavros/set_mode（OFFBOARD / AUTO.LAND）
//                                                                       └─▶ /mavros/cmd/arming（解锁，仅一次）
//
// ★ 本节点的职责分工（改代码前先读这条）：
//   · **判断**（该不该发 setpoint、该不该切模式、该不该解锁、飞到哪一步）只在 `fsm.hpp`，那里零 ROS 依赖。
//   · **翻译**（话题 → 守卫/判据/读数、决策 → 话题/服务）只在 `px4_interface.cpp`。
//   · 本文件只做**接线**：参数、定时器、服务、日志。不要在这里加"顺手判断"——判据进了这里就没人测得到了。
//     **唯一的例外**是 `request_mode()` 里的重试节流（in-flight / 重试间隔 / 悬空请求丢弃）：
//     那是"输出侧的发送节奏"，不是"输入侧的判据"，天然属于调用方。它没有 seam-1 测试覆盖，
//     改动时请连同 `src/control/README.md` 的覆盖清单一起看。
//
// ★ 本张（ticket 08）**新增的三条对外接口/动作**：
//   · `/control/takeoff`（`std_srvs/Trigger`）—— **唯一**的起飞入口。ticket 09 会让遥控器
//     ch9 的上升沿来调它（A1 项），本张先用服务把它跑通、测到。
//     ★ 受理 ≠ 起飞：真正的判据是随后 `/mavros/state.armed`（1 Hz）。
//   · `/mavros/cmd/arming` —— **只用来解锁、只发一次、被拒不重试**（用户 2026-09-22 裁定）。
//   · 自动降落：`WAIT_START` 保持 `hover_hold_s` 后状态机自己请求 `AUTO.LAND`；
//     **上锁不由我们发**，交给 PX4 的 `COM_DISARM_LAND = 2.0`（z0 实测值），我们只等 `armed` 降沿。
//   ⚠️ 本张**删掉了 `yaw` 参数**：setpoint 的 yaw 现在由状态机给（起飞前 = 当前机头，
//     起飞后 = 锁定的起飞机头）。旧参数默认 0 在 ENU 口径下是"机头朝东"，
//     一在 OFFBOARD 里解锁 PX4 就会真的去转机头 —— 见 `fsm.hpp` 文件头 ★。
//
// ★ 运行纪律：本节点与 `mocap` 节点共用"从启动到解锁之间不要搬动飞机"的纪律。

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "control/fsm.hpp"
#include "control/px4_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

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

/// `namespace/node`（默认命名空间不写成 `//node`）—— 发布者的**身份**，见 `publisher_summary`。
std::string node_id(const rclcpp::TopicEndpointInfo & info)
{
  const std::string ns = info.node_namespace();
  return (ns.empty() || ns == "/") ? ("/" + info.node_name()) : (ns + "/" + info.node_name());
}

/// `(x, y, z)` —— **固定形状**是为了让日志可 grep（`traj_origin=`、`takeoff_origin=`）。
/// 理由与 `degrade=<token>` 那条一样：现场要的是"能搜到并直接喂给脚本"，
/// 而不是"读起来顺"（台架对账时要把这些数与动捕报的位置做差）。
std::string format_vec3(double x, double y, double z)
{
  char buf[96];
  std::snprintf(buf, sizeof(buf), "(%.3f, %.3f, %.3f)", x, y, z);
  return std::string(buf);
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
    diag_period_s_ = declare_parameter<double>("diag_period_s", 1.0);
    // ⚠️ 这里**曾经有**一个 `yaw` 参数（ticket 05），ticket 08 删掉了 —— 见文件头 ★。
    //   删参数而不是改默认值，是因为"0"这个数本身在这一栈里就是错的（ENU 的 0 = 机头朝东）：
    //   留着它，下一个人只会再踩一次。

    // —— 话题/服务名（多机、台架复用不改代码）——
    topics_.state = declare_parameter<std::string>("state_topic", topics_.state);
    topics_.local_position = declare_parameter<std::string>(
      "local_position_topic", topics_.local_position);
    topics_.local_velocity = declare_parameter<std::string>(
      "local_velocity_topic", topics_.local_velocity);
    topics_.rc = declare_parameter<std::string>("rc_topic", topics_.rc);
    topics_.battery = declare_parameter<std::string>("battery_topic", topics_.battery);
    topics_.mocap = declare_parameter<std::string>("mocap_topic", topics_.mocap);
    topics_.setpoint = declare_parameter<std::string>("setpoint_topic", topics_.setpoint);
    topics_.set_mode_service = declare_parameter<std::string>(
      "set_mode_service", topics_.set_mode_service);
    topics_.arming_service = declare_parameter<std::string>(
      "arming_service", topics_.arming_service);
    const std::string state_pub_topic = declare_parameter<std::string>(
      "control_state_topic", "/control/state");
    takeoff_service_name_ = declare_parameter<std::string>(
      "takeoff_service", "/control/takeoff");

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

    // ★★ **ticket 06 / 08 之间的那条缝已经被真实实现取代**（ticket 08）：`own_arming` 参数
    //    删掉了，`Guard::kArmingByUs` 现在由 `/mavros/cmd/arming` 的应答 + `armed` 的上升沿
    //    决定（见 `px4_interface.cpp::on_state`）。
    //    ⚠️ 留着参数会让"外部解锁告警"（§H.6）整条失效：置 true 之后**任何人**拨的
    //    解锁开关都会被记成我们做的。

    // 主动中止任务用的模式名（mavros `set_mode` 的 `custom_mode`，PX4 的 `AUTO.LAND`）。
    land_mode_ = declare_parameter<std::string>("land_mode", "AUTO.LAND");

    // —— 任务判据的阈值（ticket 08；逐条出处见 `fsm.hpp` 的 `TaskLimits`）——
    // ★ 与上面的失效判据 `Limits` 分开：那一组管"什么时候停"，这一组管"飞到什么算到"。
    control::TaskLimits task;
    task.takeoff_altitude_m = declare_parameter<double>(
      "takeoff_altitude_m", task.takeoff_altitude_m);
    task.climb_speed_mps = declare_parameter<double>("climb_speed_mps", task.climb_speed_mps);
    task.takeoff_alt_tol_m = declare_parameter<double>(
      "takeoff_alt_tol_m", task.takeoff_alt_tol_m);
    task.takeoff_vel_tol_mps = declare_parameter<double>(
      "takeoff_vel_tol_mps", task.takeoff_vel_tol_mps);
    task.takeoff_settle_s = declare_parameter<double>(
      "takeoff_settle_s", task.takeoff_settle_s);
    task.hover_pos_tol_m = declare_parameter<double>("hover_pos_tol_m", task.hover_pos_tol_m);
    task.hover_vel_tol_mps = declare_parameter<double>(
      "hover_vel_tol_mps", task.hover_vel_tol_mps);
    task.hover_settle_s = declare_parameter<double>("hover_settle_s", task.hover_settle_s);
    task.hover_hold_s = declare_parameter<double>("hover_hold_s", task.hover_hold_s);
    task.takeoff_timeout_s = declare_parameter<double>(
      "takeoff_timeout_s", task.takeoff_timeout_s);
    task_limits_ = task;

    fsm_ = std::make_unique<control::Fsm>(offboard_prestream_s_, task_limits_);
    // ★ 边界层的配置一次交齐：话题/超时/阈值/**模式名** ——
    //   后一项也在这里，是为了让 `Guards` 的每一条都在边界层算完（`guards()` 的注释）。
    //   `modes.offboard` 保持默认（"OFFBOARD" 是本栈固定的口径，本张刻意不为它开参数）；
    //   `land_mode` 走参数是 ticket 06 自己的选择（多机/换固件时不用改代码）—— ticket 06
    //   点名的是"已解锁的失效要**请求 `AUTO.LAND`**"这个动作，没有要求它可配。
    control::Px4Config px4_config;
    px4_config.topics = topics_;
    px4_config.timeouts = timeouts_;
    px4_config.limits = limits_;
    px4_config.modes.land = land_mode_;
    px4_ = std::make_unique<control::Px4Interface>(*this, px4_config);

    // 状态串用 transient_local：晚订阅的 `ros2 topic echo`、晚起的测试都能立刻拿到当前值，
    // 不必"等它变化"。（另外每 diag_period_s 还会心跳重发一次，见 publish_state。）
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_pub_topic, rclcpp::QoS(10).transient_local());

    // ★★ 起飞命令的**唯一**入口（ticket 08）。用 `Trigger` 而不是自定义 srv：
    //   本张只需要"受理/拒绝 + 一句话原因"，`success` + `message` 正好装得下。
    //   ⚠️ **服务回调只投递命令**（`request_takeoff` 记一笔就返回），不做任何推进 ——
    //   推进在 50 Hz 的 `tick()` 里，用**那一刻**的守卫/读数。这样"受理时的世界"
    //   与"执行时的世界"永远是同一帧的，不需要把守卫缓存到节点里。
    takeoff_srv_ = create_service<std_srvs::srv::Trigger>(
      takeoff_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        on_takeoff_request(*res);
      });

    RCLCPP_INFO(
      get_logger(),
      "control 节点启动：setpoint %.1f Hz → %s；/control/state → %s；起飞命令 → %s（std_srvs/Trigger）\n"
      "  前置流 %.1f s 后%s切 OFFBOARD（重试间隔 %.1f s）；setpoint：位置/机头**全部由状态机给**（无 yaw 参数）\n"
      "  超时判据：state %.1f s（★ 该话题实测 1 Hz）、local_position/velocity %.1f s、mocap %.1f s、rc %.1f s、battery %.1f s\n"
      "  失效判据：低压 < %.1f V、摇杆变化率 > %.1f µs/s（≈ COM_RC_STICK_OV = 20）、"
      "动捕时间戳不前进 > %.1f s\n"
      "  ★ 任何失效 ⟹ 立即停发 setpoint；**已解锁**时并请求 %s；打杆/模式被夺走 ⟹ 停发且不抢回 OFFBOARD\n"
      "  ★ 任务判据：起飞高度 %.2f m（≤ %.2f m/s）、到高容差 ±%.2f m / ±%.2f m/s 持续 %.1f s、"
      "稳定 < %.2f m ∧ < %.2f m/s 持续 %.1f s、保持 %.1f s 后自动降落\n"
      "  ★ 解锁：调用 %s **一次**，被拒不重试（登记 %s；上锁交给 PX4 自己的 COM_DISARM_LAND）",
      setpoint_rate_hz_, topics_.setpoint.c_str(), state_pub_topic.c_str(),
      takeoff_service_name_.c_str(),
      offboard_prestream_s_, enter_offboard_ ? "自动" : "**不**（enter_offboard = false）",
      offboard_retry_s_,
      timeouts_.state_s, timeouts_.local_pos_s, timeouts_.mocap_s,
      timeouts_.rc_s, timeouts_.battery_s,
      limits_.low_voltage_v, limits_.rc_stick_ov_us_per_s, limits_.mocap_stamp_stall_s,
      land_mode_.c_str(),
      task_limits_.takeoff_altitude_m, task_limits_.climb_speed_mps,
      task_limits_.takeoff_alt_tol_m, task_limits_.takeoff_vel_tol_mps,
      task_limits_.takeoff_settle_s,
      task_limits_.hover_pos_tol_m, task_limits_.hover_vel_tol_mps, task_limits_.hover_settle_s,
      task_limits_.hover_hold_s,
      topics_.arming_service.c_str(), takeoff_service_name_.c_str());

    if (!enter_offboard_) {
      RCLCPP_WARN(
        get_logger(),
        "enter_offboard = false ⟹ 只流 setpoint、不切 OFFBOARD。"
        "此时真 PX4 永远不会接受这些 setpoint（它只在 OFFBOARD 模式下读 setpoint_raw），"
        "起飞也不会解锁（解锁前置之一就是 OFFBOARD）。");
    }
    if (task_limits_.hover_hold_s > 0.0 && task_limits_.hover_hold_s < 30.0) {
      RCLCPP_WARN(
        get_logger(),
        "hover_hold_s = %.1f s 偏短：判稳后最多这么久就会自动降落。"
        "自由飞（阶梯第 4 级）要求持续 60 s，取 < 30 s 会飞不到验收时长就下来了。",
        task_limits_.hover_hold_s);
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
    const control::Odometry odom = px4_->odometry();
    const control::Decision d = fsm_->update(guards, health, odom, now_s);

    detect_external_arming(guards);

    if (d.stream_setpoint) {
      // ★ ticket 08：目标来自状态机（`d.target_*`），**不再**拿"当前位姿"顶替 ——
      //   爬升斜坡 / 悬停点 / 轨迹原点都是它算的。边界层只管发。
      px4_->publish_setpoint(d.target_x, d.target_y, d.target_z, static_cast<float>(d.target_yaw));
    }

    if (d.request_arming) {
      request_arming_once(now_s);
    }
    if (d.request_offboard && enter_offboard_) {
      request_mode("OFFBOARD", "前置流已满足", now_s);
    }
    if (d.request_land) {
      // 主动中止任务。两个来源：失效降级（ticket 06）与任务正常走完（ticket 08）——
      // 处置逐字相同，这里只说清是哪一个（`d.mission_complete`）。
      request_mode(land_mode_, d.mission_complete ? "任务走完" : "失效降级", now_s);
    }

    publish_state(d, now_s);
    maybe_log_degrade(d);
    last_phase_ = d.phase;
  }

  /// `/control/takeoff` 的回调 —— **只投递命令**，推进留给下一帧 `tick()`。
  /// ★ 应答里把"当前前置缺什么"一并报出去：操作员按了键没反应时，第一个问题永远是"什么缺了"，
  ///   而这个答案此刻就在手上（`describe_missing`）。让它只出现在日志里是白扔。
  void on_takeoff_request(std_srvs::srv::Trigger::Response & res)
  {
    const double now_s = now().seconds();
    const auto reply = fsm_->request_takeoff(now_s);
    res.success = (reply == control::TakeoffReply::kAccepted);

    // ★★ 应答的**词表**由 `to_string(TakeoffReply)` 定（`accepted` / `already_flying`），
    //   放在 `message` 的**开头** ⟹ 现场脚本抓得到固定形状，操作员也读得到"缺什么"的人话。
    //   ⚠️ 这里曾经把 `to_string` 晾着不用、文档却按它写词表（复核指出）——
    //   两处出处必然走散，所以**真的把它接上**，而不是把文档改成描述一串自由文本。
    const std::string verdict = control::to_string(reply);

    if (reply == control::TakeoffReply::kAlreadyFlying) {
      res.message = verdict + "：已经有一次任务在跑（任务态 " + std::string(fsm_->task_name()) +
        "）。等它走完并**落地上锁**（armed → false）之后再来 —— 复位不需要重启节点。";
      RCLCPP_WARN(get_logger(), "起飞命令被拒：%s", res.message.c_str());
      return;
    }

    const control::Guards guards = px4_->guards(now_s);
    res.message = verdict + "：已受理（★ 受理 ≠ 起飞）：状态机会在 [已在 OFFBOARD ∧ 遥控在线] 时发**一次**"
      "解锁请求，最多等 " + std::to_string(task_limits_.takeoff_timeout_s) + " s；"
      "前置缺项：" + describe_missing(now_s) +
      "；OFFBOARD=" + (guards.in_offboard ? "是" : "否") +
      "；遥控=" + (px4_->health(now_s).rc_fresh ? "在线" : "**不在线**") +
      "。真正的判据是随后的 /mavros/state.armed（1 Hz ⟹ 最多晚 1 s）。";
    RCLCPP_INFO(
      get_logger(),
      "收到起飞命令（受理）⟹ 任务态 %s。%s",
      fsm_->task_name(), res.message.c_str());
  }

  /// 解锁请求 —— **只发一次、被拒不重试**（用户 2026-09-22 裁定）。
  /// ★ 重试属于**输出侧的发送节奏**，与 `request_mode()` 同一类；但这里刻意**不做**重试：
  ///   静默重试会把"解锁参数没配对"这类确定性的拒绝变成一串没人看的日志，
  ///   而操作员看到的现象只是"按了键，飞机不动"。
  /// ⚠️ 悬空请求要丢弃：rclcpp 不会超时一个没有回应的请求（`rclcpp/client.hpp`），
  ///   没有这条就会永久卡住，同时日志还在说"等 /mavros/state.armed 确认"。
  void request_arming_once(double now_s)
  {
    if (arming_request_in_flight_) {
      const double stale_after_s = 2.0;   // 正常往返是毫秒级，2 s 只可能是丢了
      if ((now_s - last_arming_request_s_) < stale_after_s) {
        return;
      }
      RCLCPP_WARN(
        get_logger(),
        "上一次解锁请求过了 %.1f s 仍无回应（mavros 挂了或重启了？）⟹ 丢弃它，不再重发",
        now_s - last_arming_request_s_);
      // ★ 丢弃就当作失败：让起飞退回 STANDBY，而不是等满 takeoff_timeout_s。
      arming_request_in_flight_ = false;
      fsm_->note_arming_failed();
      return;
    }

    arming_request_in_flight_ = true;
    last_arming_request_s_ = now_s;
    RCLCPP_INFO(
      get_logger(),
      "请求解锁（%s，只发这一次）：前置已成立（已在 OFFBOARD ∧ setpoint 已流 %.2f s ∧ 遥控在线）",
      topics_.arming_service.c_str(), fsm_->streaming_seconds(now_s));

    px4_->request_arming(
      true,
      [this](bool ok, const std::string & detail) {
        arming_request_in_flight_ = false;
        if (ok) {
          // ⚠️ 只表示飞控接受了这条命令。真正的判据是随后的 `/mavros/state.armed`。
          RCLCPP_INFO(get_logger(), "解锁请求已受理：%s", detail.c_str());
          return;
        }
        RCLCPP_ERROR(
          get_logger(),
          "解锁请求失败：%s\n"
          "  ⟹ 本次起飞**作废、退回 STANDBY**，不自动重试（用户 2026-09-22 裁定）。\n"
          "  常见原因：解锁前置没满足（PX4 要求 OFFBOARD / 遥控在线 / 姿态估计有效）、"
          "`COM_ARM_WO_GPS = 1` 之外还有别的解锁门、飞控在拒绝状态。\n"
          "  查明原因后重新调用 %s。",
          detail.c_str(), takeoff_service_name_.c_str());
        fsm_->note_arming_failed();
      });
  }

  // ★ 这里曾经有一个 `current_guards()`：`Guards` 由边界层算，节点再往上补 `own_arming` 与
  //   `in_land`。它是个**活生生的 bug 源** —— 诊断文案那条路（`missing_guards`）忘了走它，
  //   于是 own_arming = true 且已解锁时，日志报的守卫与状态机看见的不是同一份。
  //   2026-09-22 复核时把两个字段都搬进了 `Px4Interface::guards()`（连同模式名配置），
  //   本函数随之删除：**节点不再自己翻译任何守卫**。

  /// 「外部解锁」告警 —— 上升沿只报一次。
  /// ★ 依据 `params-changes.md` §H.6（第一手）：**若 `armed` 的变化不是我们造成的 ⟹ 停发 setpoint
  ///   + 告警**。判据就是 `guards.own_arming`（ticket 08 起由解锁服务的应答 + `armed`
  ///   的上升沿给出，见 `px4_interface.cpp::on_state`）—— 本函数**不再**自己判断归属。
  ///   停发由守卫（`Guard::kArmingByUs`）自动完成，这里只负责"让人看见"。
  /// ⚠️ 注意检出最多晚 1 s（`/mavros/state` 实测 1 Hz）—— 这是上游限制，不是本节点的缺陷。
  void detect_external_arming(const control::Guards & guards)
  {
    if (guards.armed && !last_armed_ && !guards.own_arming) {
      RCLCPP_ERROR(
        get_logger(),
        "检测到**外部解锁**（%s 报 armed = true，而本节点这次没有发出过解锁请求）⟹ 立即停发 setpoint"
        "（守卫 kArmingByUs 不再成立，状态机退回 INIT）。\n"
        "  请确认是谁解锁的（遥控器 / QGC / 别的进程），**在查明之前不要让它起飞**。\n"
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
          "→ %s：开始以 %.1f Hz 发布 setpoint（目标 = 当前位置、yaw = 当前机头，即“原地保持”）。"
          "前置条件全成立（飞控已连接 ∧ 动捕有数据 ∧ local_position 有效 ∧ arm 守卫成立）；"
          "任务态 %s。",
          s.c_str(), setpoint_rate_hz_, fsm_->task_name());
        return;
      case control::Phase::kTakeoff: {
        double ox = 0.0, oy = 0.0, oz = 0.0, tx = 0.0, ty = 0.0, tz = 0.0;
        const bool has_origin = fsm_->takeoff_origin(&ox, &oy, &oz);
        const bool has_traj = fsm_->traj_origin(&tx, &ty, &tz);
        RCLCPP_WARN(
          get_logger(),
          "→ %s：**已解锁**，开始爬升。目标 z 从起飞原点（起飞瞬间的 EKF 位置）沿 %.2f m/s 的"
          "斜坡上升到 +%.2f m，水平两轴钉在原点（垂直起飞，不斜着走）。\n"
          "  takeoff_origin=%s；机头锁定在 %.3f rad（不转向）\n"
          "  ★ 到高判据：|z − (原点+%.2f)| < %.2f m ∧ |v_z| < %.2f m/s 持续 %.1f s\n"
          "  ★ 轨迹原点尚未记录（要等稳定判据成立才记）；此前 traj_origin=%s",
          s.c_str(), task_limits_.climb_speed_mps, task_limits_.takeoff_altitude_m,
          has_origin ? format_vec3(ox, oy, oz).c_str() : "（无）",
          fsm_->takeoff_yaw(),
          task_limits_.takeoff_altitude_m, task_limits_.takeoff_alt_tol_m,
          task_limits_.takeoff_vel_tol_mps, task_limits_.takeoff_settle_s,
          has_traj ? format_vec3(tx, ty, tz).c_str() : "（无）");
        return;
      }
      case control::Phase::kHover: {
        double ox = 0.0, oy = 0.0, oz = 0.0;
        const bool has_origin = fsm_->takeoff_origin(&ox, &oy, &oz);
        RCLCPP_WARN(
          get_logger(),
          "→ %s：已到高，定点保持并判稳。悬停点 = 起飞原点 (%.3f, %.3f, %.3f) 正上方 %.2f m。\n"
          "  ★ 稳定判据：位置误差 < %.2f m ∧ |v| < %.2f m/s（三维速率，来源 %s）**持续 %.1f s**；"
          "成立那一刻记录**轨迹原点**。",
          s.c_str(), ox, oy, oz, task_limits_.takeoff_altitude_m,
          task_limits_.hover_pos_tol_m, task_limits_.hover_vel_tol_mps,
          topics_.local_velocity.c_str(), task_limits_.hover_settle_s);
        if (!has_origin) {
          // 不可达（HOVER 只可能从 CLIMB 进来，而 CLIMB 一定锁过原点）—— 留着是为了让
          // "原点没锁就悬停"这件事在日志里显形，而不是安静地悬停在一个未定义的点上。
          RCLCPP_ERROR(get_logger(), "  ⚠️ 异常：进入 HOVER 但起飞原点未记录");
        }
        return;
      }
      case control::Phase::kWaitStart: {
        double tx = 0.0, ty = 0.0, tz = 0.0;
        const bool has_traj = fsm_->traj_origin(&tx, &ty, &tz);
        // ★★ 「轨迹原点在日志里可复核」是本张的验收项之一 ⟹ 用固定的
        //    `traj_origin=(x, y, z)` 形状打出来，可 grep、可与动捕原点对账。
        RCLCPP_WARN(
          get_logger(),
          "→ %s：**稳定判据成立**。traj_origin=%s（= 判稳那一刻 EKF2 报的位置，ENU）。\n"
          "  ★ 这条设计让 Phase 1 与 Phase 2 的原点问题解耦：之后一切目标都以它为 0，"
          "EKF2 局部原点与动捕原点之间的常值偏移被自动吸收。\n"
          "  ★ 目标已切到轨迹原点本身（停在原地）；保持 %.1f s 后**自动请求 %s 降落**，"
          "上锁由 PX4 的 COM_DISARM_LAND = 2.0 完成。\n"
          "  ★ ticket 09 会在这里插入“遥控器开始键 ⟹ TRACK”。",
          s.c_str(), has_traj ? format_vec3(tx, ty, tz).c_str() : "（异常：未记录）",
          task_limits_.hover_hold_s, land_mode_.c_str());
        return;
      }
      case control::Phase::kLand:
        // 两种情况（失效降级 / 任务走完）在这里分开说 —— 动作逐字相同，只有原因不同。
        // 降级那条的完整说明（含证据与复位条件）由上一条 `degrade=` 日志给出。
        if (d.mission_complete) {
          RCLCPP_WARN(
            get_logger(),
            "→ %s：**任务正常走完**（WAIT_START 保持满 %.1f s）⟹ 已停止发布 setpoint，并请求 %s。\n"
            "  ★ 上锁不由我们发：PX4 的 COM_DISARM_LAND = 2.0（z0 实测值）在它自己的着陆检测"
            "判定落地 2 s 后上锁，我们只等 armed 的降沿。\n"
            "  ★ 复位条件：**落地上锁**（armed → false）后自动复位，不需要重启节点。",
            s.c_str(), task_limits_.hover_hold_s, land_mode_.c_str());
          return;
        }
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
      case control::Phase::kTrack:
        // 本张（ticket 08）到不了这里：跟踪 = ticket 09（入口是"遥控器开始键"，本张没有）。
        RCLCPP_WARN(get_logger(), "→ %s：该任务态本张（ticket 08）还未实现", s.c_str());
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
        // ★ 判据是"新鲜 **且** `connected`"两条合起来的（`px4_interface.cpp::guards()`）
        //   ⟹ 证据必须两条都写：只写"静默 X s"会在 `connected=false` 而话题**新鲜**时报出
        //   "静默 0.0 s"，把排查方向指错。这种"新鲜但 disconnected"的帧是真会出现的 ——
        //   mavros 在链路超时后自己发一帧合成状态（`sys_status.cpp:903-911`，见 ticket 06 的 Comments）。
        return topics_.state + " 静默 " + age_str(topics_.state, now_s) + "（超时判据 " +
               std::to_string(timeouts_.state_s) + " s）、connected=" +
               (px4_->state_connected() ? "true" : "false") + "；发布者：" +
               publisher_summary(topics_.state);
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
    // ★★ 「已收到起飞命令、但还没解锁」—— ticket 08 唯一的"按了键没反应"路径。
    //   受理 ≠ 起飞（见 `Fsm::request_takeoff`），所以必须有一处**持续**把"在等什么"说出来，
    //   否则操作员只能看到 `/control/takeoff` 回了 ok 然后什么都没有。
    if (fsm_->awaiting_arming()) {
      const control::Guards guards = px4_->guards(now_s);
      const control::Health health = px4_->health(now_s);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "已收到起飞命令，正在等解锁：解锁请求已发出 = %s；OFFBOARD = %s；遥控 = %s；守卫缺项：%s\n"
        "  ★ 受理后最多等 %.1f s（takeoff_timeout_s），到点作废并退回 STANDBY —— **不自动重试**。\n"
        "  若解锁被飞控拒绝，会另有一条 ERROR 写明 MAV_RESULT 码。",
        fsm_->arming_requested() ? "是" : "否（解锁前置未齐）",
        guards.in_offboard ? "是" : "否", health.rc_fresh ? "在线" : "**不在线**",
        describe_missing(now_s).c_str(), task_limits_.takeoff_timeout_s);
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
        // ★ 语义是"**未解锁** ∨ 解锁是本节点发起的"，所以缺项的读法只有一种：**别人解锁了**。
        //   ticket 08 起归属由 `/mavros/cmd/arming` 的应答决定，节点这里不再有二义。
        return "**已解锁**，且这次解锁**不是本节点请求的**（⟹ 外部解锁：遥控器 / QGC / "
               "别的进程），见 ERROR 日志";
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
      {"local_velocity", &topics_.local_velocity},
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

  /// `N 个（/节点名 [QoS], /另一个 [QoS]）`。
  /// ★ **节点名是刻意加的（2026-09-22）**：台架上抓到过一次"话题上有帧、而它唯一的合法发布者
  ///   当时没有任何发布路径"的事件，当时这一列只写"发布者：N 个（QoS）"⟹ **事后无法指认是谁发的**。
  ///   数量能回答"有没有"，**节点名才能回答"是谁"** —— 指认不出来才会变成悬案。
  ///   夹具里每个场景自带一个节点名，所以这一列在 seam 1 上是可断言的
  ///   （见 `test/test_control_standby.py::test_日志里能读出发布者的节点名`）。
  std::string publisher_summary(const std::string & topic) const
  {
    const auto infos = get_publishers_info_by_topic(topic);
    if (infos.empty()) {
      return "**0 个**（这个话题上没有任何发布者）";
    }
    std::string s = std::to_string(infos.size()) + " 个";
    for (size_t i = 0; i < infos.size(); ++i) {
      s += (i ? ", " : "（") + node_id(infos[i]) + " [" +
           qos_summary(infos[i].qos_profile()) + "]";
    }
    return s + "）";
  }

  // ───────────────────────────── 成员 ─────────────────────────────

  double setpoint_rate_hz_{50.0};
  double offboard_prestream_s_{1.0};
  double offboard_retry_s_{1.0};
  bool enter_offboard_{true};
  double diag_period_s_{1.0};
  std::string land_mode_{"AUTO.LAND"};
  /// 起飞命令的服务名（ticket 09 会由遥控器 ch9 的上升沿去调它）。
  std::string takeoff_service_name_{"/control/takeoff"};

  control::Px4Topics topics_;
  control::Timeouts timeouts_;
  control::Limits limits_;
  /// 任务判据（`fsm.hpp::TaskLimits`）—— 日志里要引用，所以节点留一份。
  control::TaskLimits task_limits_;

  std::unique_ptr<control::Fsm> fsm_;
  std::unique_ptr<control::Px4Interface> px4_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_;
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

  // —— 解锁请求的节流状态（`request_arming_once()`，同一类：输出侧的节奏，不是判据）——
  /// ★ 状态机一次任务只脉冲一次 `request_arming`，所以正常情况下这里最多有一次在途；
  ///   留 `in_flight` 是为了处理"悬空请求"（rclcpp 不会超时没有回应的服务调用）。
  bool arming_request_in_flight_{false};
  double last_arming_request_s_{0.0};

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
