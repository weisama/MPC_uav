// control 节点：本项目的**第一个自主飞行节点**。ticket 05 走到 `STANDBY`，ticket 06 加**失效路径**，
// ticket 08 加**起飞 / 悬停 / 自动降落**（`TAKEOFF → HOVER → WAIT_START → LAND`），
// ticket 09 把 `WAIT_START → TRACK → LAND` 这一段接上（**开始键** + 轨迹跟踪）。
//
// 数据流：
//   /mavros/state ─┐
//   /mavros/local_position/pose ─┤
//   /mavros/local_position/velocity_local ─┤
//   /mocap/pose_enu ─────────────┼─▶ 守卫 + 失效判据 + 导航读数 ─▶ 状态机 ─▶ /mavros/setpoint_raw/local
//   /mavros/rc/in、/mavros/battery ┘                                    ├─▶ /control/state
//   /traj/reference、/traj/progress ────────────────────────────────────┤  （ticket 09：参考 + 完成度）
//                                                                       ├─▶ /control/takeoff（服务：受理起飞命令）
//                                                                       ├─▶ /control/traj_progress（转发完成度）
//                                                                       ├─▶ /traj/start（让 traj 归零参考时钟）
//                                                                       ├─▶ /mavros/set_mode（OFFBOARD / AUTO.LAND）
//                                                                       └─▶ /mavros/cmd/arming（解锁，仅一次）
//   ⚠️ **本节点是 `/traj/*` 的唯一消费者**：`traj` 只管发，什么时候要它、要不要重发开始，
//      全部由这里（`fsm.hpp`）决定 —— 所以 `traj` 侧本张一行代码都没改。
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
//   · `/control/takeoff`（`std_srvs/Trigger`）—— 本张里它是**唯一**的起飞入口。
//     ★ ticket 09 起**不再是唯一入口**：现场走的是遥控器开始键（ch9）的第 1 次上升沿，
//     两条路径走**同一个** `Fsm::request_takeoff`，判据逐条相同；服务那条留给调试与**拆桨台架**
//     （台上没有遥控接收机 ⟹ "RC 在线"这个起飞前置条件本身就过不了）。
//     ★ 受理 ≠ 起飞：真正的判据是随后 `/mavros/state.armed`（1 Hz）。
//   · `/mavros/cmd/arming` —— **只用来解锁、只发一次、被拒不重试**（用户 2026-09-22 裁定）。
//   · 自动降落：状态机在任务走完时自己请求 `AUTO.LAND`
//     （⚠️ ticket 09 起出口**不是**"`WAIT_START` 停够 `hover_hold_s`"——那条参数已删除，
//     见下方 ticket 09 一段与 `fsm.hpp` 的 `EndReason`）；
//     **上锁不由我们发**，交给 PX4 的 `COM_DISARM_LAND = 2.0`（z0 实测值），我们只等 `armed` 降沿。
// ★ 本张（ticket 09）**新增的两条对外接口/动作**：
//   · `/traj/start`（`Empty`，**我们发**）—— 让 `traj` 把参考时钟归零。发的时机由 `fsm.hpp` 算
//     （`Decision::request_traj_start` 是脉冲）：进 `TRACK` 的那一帧发一次，之后**只在参考流断了**
//     才重发（那是"运行中重选轨迹"的补救，见 `TaskLimits::traj_stream_gap_s`）。
//   · `/control/traj_progress`（`Float32`，**我们发**）—— 逐字转发 `/traj/progress`，给操作员看进度。
//   ⚠️ 遥控器开始键（ch9 上升沿）**不在这里判**：`rc_switch.hpp` 判边沿，`fsm.hpp::note_start_edge`
//     判"这条边沿该干什么"（按任务态，不按计数器）。节点只把取到的边沿投进去、把结论写进日志。
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
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{

/// 降级原因的**重报**间隔（s）：degrade 持续期间每隔这么久再打一条。
/// 取 10 s 的理由与 INIT 的诊断日志一致：太密会刷屏（失效可能持续几分钟），
/// 太疏则"最后一条日志是十分钟前的"，现场翻日志时判断不出"还在不在降级"。
constexpr double kDegradeRepeatS = 10.0;

/// `TRACK` 进行中"进度心跳"的间隔（s）。★ 与 `diag_period_s` 分开是因为它的**用途不同**：
/// 那一份是"判据可查"（缺什么、谁断了），这一份是给操作员一个"轨迹还在走、走到几成"的读数。
/// 取 5 s：三条轨迹分别是 60 s（悬停）、几十秒（方形/8 字）的量级 ⟹ 一次飞行十来条，不刷屏。
constexpr double kTrajProgressLogS = 5.0;

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
    // ★ ticket 16：EKF2 位置失效的载体（唯一一个在本链条上真能读到的，见 `Px4Topics`）。
    topics_.estimator_status = declare_parameter<std::string>(
      "estimator_status_topic", topics_.estimator_status);
    topics_.mocap = declare_parameter<std::string>("mocap_topic", topics_.mocap);
    topics_.setpoint = declare_parameter<std::string>("setpoint_topic", topics_.setpoint);
    // ★ ticket 09：`traj` 的三条接口 + 我们自己的完成度观测面。全部走参数的理由同上面那几条
    //   （台架夹具要用私有话题名；多机时每条前缀不同）。
    topics_.traj_reference = declare_parameter<std::string>(
      "traj_reference_topic", topics_.traj_reference);
    topics_.traj_progress = declare_parameter<std::string>(
      "traj_progress_topic", topics_.traj_progress);
    topics_.traj_start = declare_parameter<std::string>("traj_start_topic", topics_.traj_start);
    topics_.control_traj_progress = declare_parameter<std::string>(
      "control_traj_progress_topic", topics_.control_traj_progress);
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
    // ★ ticket 16：**不是判据、只是诊断/新鲜度门**（见 `Timeouts::estimator_s`）。
    timeouts.estimator_s = declare_parameter<double>("estimator_timeout_s", timeouts.estimator_s);
    timeouts_ = timeouts;

    // —— 失效判据的阈值（ticket 06，逐条出处见 `px4_interface.hpp` 的 `Limits`）——
    control::Limits limits;
    limits.low_voltage_v = declare_parameter<double>("low_voltage_v", limits.low_voltage_v);
    limits.rc_stick_ov_us_per_s = declare_parameter<double>(
      "rc_stick_ov_us_per_s", limits.rc_stick_ov_us_per_s);
    limits.mocap_stamp_stall_s = declare_parameter<double>(
      "mocap_stamp_stall_s", limits.mocap_stamp_stall_s);
    // ★ ticket 16：消抖帧数。**与上游频率强相关**（1 Hz ⟹ 2 帧 ≈ 2 s），换固件要重估。
    limits.ekf2_pos_lost_samples = declare_parameter<int>(
      "ekf2_pos_lost_samples", limits.ekf2_pos_lost_samples);
    limits.rc_map_roll = declare_parameter<int>("rc_map_roll", limits.rc_map_roll);
    limits.rc_map_pitch = declare_parameter<int>("rc_map_pitch", limits.rc_map_pitch);
    limits.rc_map_throttle = declare_parameter<int>("rc_map_throttle", limits.rc_map_throttle);
    limits.rc_map_yaw = declare_parameter<int>("rc_map_yaw", limits.rc_map_yaw);
    limits_ = limits;

    // —— 遥控器**开始键**的判据（ticket 09；§H.1 的那一组数，值本身不要就地改）——
    // ★ 与 `Limits`（失效判据）分开：这一组管的是"操作员做了一个动作没有"，
    //   与"什么时候停"混在一起之后，调去抖会顺手改了低压阈值。
    // ★ 通道号与阈值的裁定过程（含 §H.4 的 EdgeTX 分配、§H.2 的"ch9 未被任何 `RC_MAP_*` 占用"）
    //   都在 `docs/reference/params-changes.md` §H。
    control::RcStart rc_start;
    rc_start.channel = declare_parameter<int>("start_channel", rc_start.channel);
    rc_start.start_threshold = declare_parameter<double>(
      "start_threshold", rc_start.start_threshold);
    rc_start.release_threshold = declare_parameter<double>(
      "release_threshold", rc_start.release_threshold);
    // ★ 参数用**毫秒**、内部用秒：§H.1 的裁定就是按毫秒说的（200 ms），
    //   让人在 `control.yaml` 里写 `200` 比写 `0.2` 少一层折算错误。
    rc_start.debounce_s = declare_parameter<double>("debounce_ms", rc_start.debounce_s * 1e3) * 1e-3;
    rc_start.low_us = declare_parameter<double>("start_low_us", rc_start.low_us);
    rc_start.high_us = declare_parameter<double>("start_high_us", rc_start.high_us);
    // ★ ticket 07 增补（2026-09-24 用户裁定）：**长按开始键 ≥ 这么久 ⟹ 立即自动降落**。
    //   `<= 0` = 关掉这条功能（那是"回到 ticket 09 那一版行为"的配置方式）。
    //   ⚠️ 计时从**电平被确认**那一刻起算（`debounce_ms` 之后）⟹ 现场"按 1 s"要按 ≈ 1.2 s。
    rc_start.land_hold_s = declare_parameter<double>("start_land_hold_s", rc_start.land_hold_s);
    rc_start_ = rc_start;

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
    // ★★ ticket 09 **换掉**了 `hover_hold_s`（ticket 08 的"悬停保持 N s 后自动降落"）：
    //   降落不再由"时间到了"触发，而是由**开始键**（进 `TRACK`）或**轨迹跑完**触发。
    //   但"操作员一直不按键"这条自己走不出去的态必须有个兜底 ⟹ 那个数现在叫
    //   `wait_start_timeout_s`（`<= 0` = 永不超时）。见 `fsm.hpp::TaskLimits`。
    task.wait_start_timeout_s = declare_parameter<double>(
      "wait_start_timeout_s", task.wait_start_timeout_s);
    // ★ ticket 09：`TRACK` 的两条时限（重发 `/traj/start` 的闸门 / 判"轨迹超时"降级）。
    //   ⚠️ 这两个都是**与 `traj` 的发布行为强耦合**的数（50 Hz 的流、`select` 会静默重置时钟），
    //   改之前先读 `TaskLimits` 上那两段说明。
    task.traj_stream_gap_s = declare_parameter<double>(
      "traj_stream_gap_s", task.traj_stream_gap_s);
    task.traj_timeout_s = declare_parameter<double>("traj_timeout_s", task.traj_timeout_s);
    task.takeoff_timeout_s = declare_parameter<double>(
      "takeoff_timeout_s", task.takeoff_timeout_s);
    task_limits_ = task;

    fsm_ = std::make_unique<control::Fsm>(offboard_prestream_s_, task_limits_);
    // ★ 边界层的配置一次交齐：话题/超时/阈值/**模式名** ——
    //   后一项也在这里，是为了让 `Guards` 的每一条都在边界层算完（`guards()` 的注释）。
    //   `modes.offboard` 保持默认（"OFFBOARD" 是本栈固定的口径，本张刻意不为它开参数）；
    //   `land_mode` 走参数是 ticket 06 自己的选择（多机/换固件时不用改代码）—— ticket 06
    //   点名的是"已解锁的失效要**请求 `AUTO.LAND`**"这个动作，没有要求它可配。
    // ★ ticket 09：完成度观测面。**在边界层之前建**，因为它的转发回调要捕获这个发布者
    //   （回调只会在订阅收到帧时才跑，那时构造函数早就结束了 —— 但先建能让"谁依赖谁"一眼看清）。
    traj_progress_pub_ = create_publisher<std_msgs::msg::Float32>(
      topics_.control_traj_progress, 10);

    control::Px4Config px4_config;
    px4_config.topics = topics_;
    px4_config.timeouts = timeouts_;
    px4_config.limits = limits_;
    px4_config.modes.land = land_mode_;
    px4_config.rc_start = rc_start_;
    // ★ ticket 09：完成度的**观测面**（`/control/traj_progress`）挂在边界层那条订阅上（回调），
    //   而不是让节点自己再订阅一遍 —— 理由见 `Px4Config::on_traj_progress`：
    //   两处订阅会让"状态机用的那个数"与"转发出去的那个数"来自两条不同的队列。
    px4_config.on_traj_progress = [this](float p) {
      std_msgs::msg::Float32 msg;
      msg.data = p;
      traj_progress_pub_->publish(msg);
    };
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
      "动捕时间戳不前进 > %.1f s、%s 报 pos_horiz_abs 连续 %d 帧为假（★ 只在已解锁时算失效）\n"
      "  ★ 任何失效 ⟹ 立即停发 setpoint；**已解锁**时并请求 %s；打杆/模式被夺走 ⟹ 停发且不抢回 OFFBOARD\n"
      "  ★ 「EKF2 位置失效」的载体 = %s（%.1f s 内的帧才算数；该话题实测 1 Hz，**不作超时判据**）\n"
      "  ★ 任务判据：起飞高度 %.2f m（≤ %.2f m/s）、到高容差 ±%.2f m / ±%.2f m/s 持续 %.1f s、"
      "稳定 < %.2f m ∧ < %.2f m/s 持续 %.1f s\n"
      "  ★ 开始键（ch%d 上升沿）：归一化 > %.2f 起算、< %.2f 复位、去抖 %.0f ms"
      "（越界读数按 %.0f→0.0 / %.0f→1.0 折算）⟹ 第 1 次解锁起飞、第 2 次开始轨迹\n"
      "  ★ WAIT_START 兜底超时 %.1f s（%s）；轨迹参考 %s、完成度 %s → 转发到 %s；重发 /traj/start 的闸门 %.1f s、"
      "参考断流 %.1f s 判 traj_timeout\n"
      "  ★ 解锁：调用 %s **一次**，被拒不重试（登记 %s；上锁交给 PX4 自己的 COM_DISARM_LAND）",
      setpoint_rate_hz_, topics_.setpoint.c_str(), state_pub_topic.c_str(),
      takeoff_service_name_.c_str(),
      offboard_prestream_s_, enter_offboard_ ? "自动" : "**不**（enter_offboard = false）",
      offboard_retry_s_,
      timeouts_.state_s, timeouts_.local_pos_s, timeouts_.mocap_s,
      timeouts_.rc_s, timeouts_.battery_s,
      limits_.low_voltage_v, limits_.rc_stick_ov_us_per_s, limits_.mocap_stamp_stall_s,
      topics_.estimator_status.c_str(), limits_.ekf2_pos_lost_samples,
      land_mode_.c_str(),
      topics_.estimator_status.c_str(), timeouts_.estimator_s,
      task_limits_.takeoff_altitude_m, task_limits_.climb_speed_mps,
      task_limits_.takeoff_alt_tol_m, task_limits_.takeoff_vel_tol_mps,
      task_limits_.takeoff_settle_s,
      task_limits_.hover_pos_tol_m, task_limits_.hover_vel_tol_mps, task_limits_.hover_settle_s,
      rc_start_.channel, rc_start_.start_threshold, rc_start_.release_threshold,
      rc_start_.debounce_s * 1e3, rc_start_.low_us, rc_start_.high_us,
      task_limits_.wait_start_timeout_s,
      task_limits_.wait_start_timeout_s > 0.0 ? "≤0 表示永不超时" : "**已关闭**：不按键就永远停在 WAIT_START",
      topics_.traj_reference.c_str(), topics_.traj_progress.c_str(),
      topics_.control_traj_progress.c_str(),
      task_limits_.traj_stream_gap_s, task_limits_.traj_timeout_s,
      topics_.arming_service.c_str(), takeoff_service_name_.c_str());

    if (!enter_offboard_) {
      RCLCPP_WARN(
        get_logger(),
        "enter_offboard = false ⟹ 只流 setpoint、不切 OFFBOARD。"
        "此时真 PX4 永远不会接受这些 setpoint（它只在 OFFBOARD 模式下读 setpoint_raw），"
        "起飞也不会解锁（解锁前置之一就是 OFFBOARD）。");
    }
    // ★★ ticket 09：这条告警的**理由整个换了**，别照 ticket 08 的说法理解它。
    //   ticket 08 时 `WAIT_START` 保持 `hover_hold_s`（60 s）后自动降落 ⟹ 那个 60 s **就是**
    //   §5.3 的"持续 60 s"，所以"配短了 = 验收时长飞不够"成立。
    //   ticket 09 起不是了：§5.3 的那 60 s 现在来自**轨迹 1 自己的时长**
    //   （`traj` 的 `hover_duration_s: 60.0`，在 `TRACK` 里跑），与 `WAIT_START` 停多久无关。
    //   ⟹ 这个数配短了**不会**让验收飞不够，它只会让"操作员还没准备好、飞机就自己落了"。
    //   所以阈值是**人的准备时间**的量级，取 60 s（一个月以内的等待都算"手感正常"），
    //   不是从 §5.3 推出来的硬约束。
    if (task_limits_.wait_start_timeout_s > 0.0 && task_limits_.wait_start_timeout_s < 60.0) {
      RCLCPP_WARN(
        get_logger(),
        "wait_start_timeout_s = %.1f s 偏短：判稳进 WAIT_START 后最多这么久**就会自己降落**"
        "（与操作员按不按开始键无关）。它**不是** §5.3 的时长约束（那 60 s 归轨迹 1 的"
        "`hover_duration_s`），而是「操作员从看到稳了到按下开始键的准备时间」—— 取 < 60 s "
        "等于替操作员掐表，现场大概率会撞上「还没准备好飞机就自己落了」。"
        "要「一直等」就把它配成 0 或负数（= 永不超时）。",
        task_limits_.wait_start_timeout_s);
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

    // ★★ ticket 09：先**取走**开始键的上升沿（消费语义 —— 一次按键只会被取走一次），
    //   再把它投给状态机。顺序不能反：`note_start_edge()` 会改任务态（第 1 次受理起飞 /
    //   第 2 次进 TRACK），而下面那一帧的决策要反映**改完之后**的任务态。
    //   ⚠️ 边沿与"这一帧的守卫/读数"一起用，而不是"按下那一刻的" —— 50 Hz ⟹ 最多差 20 ms，
    //     而这样"受理时的世界"与"执行时的世界"永远是同一帧（与 `/control/takeoff` 服务同一条纪律）。
    if (px4_->take_start_edge()) {
      const control::StartEdgeReply reply = fsm_->note_start_edge(now_s);
      log_start_edge(reply);
    }
    // ★★ ticket 07 增补：**长按**与**松手**两条事件。顺序不能反 —— 长按那条会把"欠下的开始"
    //   清掉（`track_on_release_`），松手那条再见它时就不该再把飞机开进 `TRACK`。
    //   （两条事件物理上不可能同帧：上升沿到"够久"之间隔着 `land_hold_s`，而松手更在之后。）
    if (px4_->take_start_long_press()) {
      log_start_long_press(fsm_->note_start_long_press());
    }
    if (px4_->take_start_release()) {
      log_start_release(fsm_->note_start_release(now_s));
    }

    const control::Guards guards = px4_->guards(now_s);
    const control::Health health = px4_->health(now_s);
    const control::Odometry odom = px4_->odometry();
    const control::TrajReference traj = px4_->traj(now_s);
    const control::Decision d = fsm_->update(guards, health, odom, now_s, traj);

    detect_external_arming(guards);
    detect_traj_stream(d, traj, now_s);

    if (d.stream_setpoint) {
      // ★ ticket 08：目标来自状态机（`d.target_*`），**不再**拿"当前位姿"顶替 ——
      //   爬升斜坡 / 悬停点 / 轨迹原点都是它算的。边界层只管发。
      px4_->publish_setpoint(d.target_x, d.target_y, d.target_z, static_cast<float>(d.target_yaw));
    }

    // ★ 进 `TRACK` 的那一帧先把两个"诊断计数"清零（顺序在下面那条脉冲之前：那一帧发出去的
    //   就是第 1 次，> 1 才是"断流后重发"）。只给日志用，不影响任何判据。
    if (d.phase == control::Phase::kTrack && last_phase_ != control::Phase::kTrack) {
      track_entered_s_ = now_s;
      traj_start_sent_count_ = 0;
    }
    if (d.request_traj_start) {
      // ★ 脉冲（只在 `TRACK` 里给）：进 TRACK 的第一帧发一次，之后只在参考流断了才重发。
      px4_->publish_traj_start();
      ++traj_start_sent_count_;
    }

    if (d.request_arming) {
      request_arming_once(now_s);
    }
    if (d.request_offboard && enter_offboard_) {
      request_mode("OFFBOARD", "前置流已满足", now_s);
    }
    if (d.request_land) {
      // 主动中止任务。两个来源：失效降级（ticket 06）与任务正常走完（ticket 08/09）——
      // 处置逐字相同，这里只说清是哪一个（`d.end_reason`）。
      // ★ token 用 `to_string(EndReason)`（与 `fsm.hpp` 的判据同源，同 `TakeoffReply` 那条纪律），
      //   人话用 `land_why()` —— 两者都进日志，可 grep 也可读。
      request_mode(land_mode_, land_why(d.end_reason), now_s);
    }

    publish_state(d, now_s);
    maybe_log_degrade(d);
    last_phase_ = d.phase;
  }

  /// `request_mode()` 的 `why` 文案 —— ★ 必须返回**静态生存期**的字面量：这个字符串会被
  /// **异步**的服务回调捕获（`request_mode` 里的 lambda），传 `std::string::c_str()` 会悬空。
  /// ⚠️ 故意不写 `default:`：新增一条 `EndReason` 而漏掉文案时 `-Wswitch` 直接编译失败。
  static const char * land_why(control::EndReason r)
  {
    switch (r) {
      case control::EndReason::kTrajDone:
        return "任务走完：轨迹完成度到 1";
      case control::EndReason::kWaitStartTimeout:
        return "任务走完：等开始键超时";
      case control::EndReason::kLandPressed:
        return "任务走完：操作员长按开始键";
      case control::EndReason::kNone:
        break;
    }
    // `kNone` = 不是"任务自己走完"那一类出口 ⟹ 就是失效降级（完整原因见同一拍的 `degrade=`）。
    return "失效降级";
  }

  /// `TRACK` 里**参考流的进出**都留一条日志（ticket 09）。
  /// ★ 为什么值得单独一条：`/traj/reference` 只在 `traj` 的 `running_` 为真时才发
  ///   （`tick()` 开头 `if (!running_) return;`）⟹ **"参考静默"与"traj 没在跑"是同一件事**。
  ///   现场看到飞机停在原地不动时，这一条能立刻分开两种原因：
  ///   "我们从没要过这个流"（本节点只发了一次 start，而 traj 因为没选轨迹忽略了它）
  ///   与"跑到一半流断了"（那时 `traj` 正在跑 ⟹ 只可能是它被重选/挂了）。
  /// ⚠️ 用 `d.phase` 而不是内部任务态：`TRACK` 是本张唯一会要这个流的态。
  void detect_traj_stream(const control::Decision & d, const control::TrajReference & traj,
    double now_s)
  {
    const bool in_track = (d.phase == control::Phase::kTrack);
    if (!in_track) {
      // ★★ 离开 TRACK **不是**"参考断了"（ticket 09 的台架实测抓到的第一条日志缺陷）：
      //   离开之后我们**不再需要**这个流（进 LAND 时目标已经不再跟着它走），
      //   而 `alive` 里含着 `in_track` ⟹ 照原先的写法，TRACK → LAND 那一拍必然出现
      //   alive 由真变假，于是**每一次正常收口**都会打出一条"**轨迹参考断了**"，
      //   同一行里的年龄还是 0.016 s（流好得很）。实测证据（`test_control_track.py`
      //   的「完成度到 1.0」那条）：它紧挨着「任务正常走完（end_reason=traj_done）」，
      //   现场翻日志只会读成"飞完的那一刻参考流挂了"。
      //   ⟹ 只在 TRACK 里判进出；离开时把闩锁复位，好让**下一个任务**重新播报"参考已经来了"。
      traj_stream_seen_ = false;
      return;
    }
    const bool alive = traj.alive(task_limits_.traj_stream_gap_s);
    if (alive == traj_stream_seen_) {
      // 仍在同一状态：只在"流还在"时按 diag 周期说一次进度（给现场一个"它还在动"的读数）。
      if (alive && (now_s - last_traj_progress_log_s_) >= kTrajProgressLogS) {
        last_traj_progress_log_s_ = now_s;
        RCLCPP_INFO(
          get_logger(),
          "TRACK 进行中：/traj/reference 的年龄 %.3f s，完成度 %.3f（%s 上转发的是同一个数）",
          traj.age_s, traj.progress, topics_.control_traj_progress.c_str());
      }
      return;
    }
    traj_stream_seen_ = alive;
    if (alive) {
      RCLCPP_WARN(
        get_logger(),
        "TRACK：**轨迹参考已经来了**（%s 上有帧，年龄 %.3f s）⟹ 目标开始跟着它走。",
        topics_.traj_reference.c_str(), traj.age_s);
      return;
    }
    // 流断了。★ 两种处置都由状态机定（重发 `/traj/start` 的闸门是 `traj_stream_gap_s`，
    //   到点还没回来就是 `traj_timeout` 降级）—— 这里只说清"什么时候断的"，不改任何行为。
    RCLCPP_WARN(
      get_logger(),
      "TRACK：**轨迹参考断了**（%s 上的最后一帧距今 %.3f s，判据 = 流的闸门 %.1f s；"
      "再撑 %.1f s 没回来就是 degrade=traj_timeout）。\n"
      "  ★ 最可能的原因：有人在运行中重选了轨迹（`traj` 的 select 会停掉流并**静默**把时钟归零，"
      "它不发任何进度）⟹ 状态机会在闸门（%.1f s）到点后重发一次 %s。\n"
      "  ⚠️ 这一条也说明飞机在**原地保持**（目标停在最后一个参考上，**不会**掉头回轨迹原点）。",
      topics_.traj_reference.c_str(), traj.age_s, task_limits_.traj_stream_gap_s,
      task_limits_.traj_timeout_s,
      task_limits_.traj_stream_gap_s, topics_.traj_start.c_str());
  }

  /// 开始键判定的**反应**（ticket 09 的核心可观测面）：每一次确认的上升沿都要留下一条日志，
  /// 包括**被丢掉的那一次** —— "按了没反应"是现场最难查的一类问题，而它只有两种可能：
  /// 边沿没产生（去抖/通道号/断链 ⟹ 见 `rc_switch.hpp` 的三个观测口），
  /// 或者产生了但**任务态不接受**（比如已经在飞了）。
  /// ★ 后者的证据就在这里：`ignored` 那一条会把当前任务态写出来。
  void log_start_edge(control::StartEdgeReply reply)
  {
    const auto & sw = px4_->start_switch();
    const std::string tail = "（按键计数 " + std::to_string(sw.edge_count()) +
      "，折算值 " + std::to_string(sw.normalized()) + "，已确认过低电平 = " +
      (sw.released() ? "是" : "否") + "）";
    // ⚠️ 故意不写 `default:`：新增一条 `StartEdgeReply` 而漏掉文案时 `-Wswitch` 直接编译失败。
    switch (reply) {
      case control::StartEdgeReply::kTakeoff:
        RCLCPP_WARN(
          get_logger(),
          "遥控器开始键：**第 1 次上升沿** ⟹ 已受理起飞（★ 受理 ≠ 起飞：真正起飞要等解锁）。%s",
          tail.c_str());
        return;
      case control::StartEdgeReply::kTrackOnRelease:
        RCLCPP_WARN(
          get_logger(),
          "遥控器开始键：**第 2 次上升沿** ⟹ 已记下「开始轨迹」—— ★ **松手**那一刻才真的进 "
          "TRACK（按住 ≥ %.1f s 则改为降落，见 ticket 07 的增补）。%s",
          rc_start_.land_hold_s, tail.c_str());
        return;
      case control::StartEdgeReply::kLandHold:
        RCLCPP_WARN(
          get_logger(),
          "遥控器开始键：已**锁存降落资格**（当前在飞）。按住 ≥ %.1f s ⟹ 立即自动降落；"
          "按住不到这个时长就松手则什么都不发生。%s",
          rc_start_.land_hold_s, tail.c_str());
        return;
      case control::StartEdgeReply::kIgnored:
        RCLCPP_WARN(
          get_logger(),
          "遥控器开始键：上升沿**被丢掉**了 —— 当前任务态是 %s，这一步不接受开始键。%s\n"
          "  ★ 任务态的推进与**按键次数无关**（不数第几次），只看当前在哪一步："
          "只有 IDLE 与 WAIT_START 接受这条边沿（见 `Fsm::note_start_edge`）。\n"
          "  ★ 但已受理起飞、**还没解锁**的 `kRequested` 也走这一条：那一刻飞机还在地上，"
          "连「长按降落」的资格都不给（给了它就会「刚离地就降落」）。",
          fsm_->task_name(), tail.c_str());
        return;
    }
  }

  /// **长按**开始键的受理结论（ticket 07 增补）。★ 与 `log_start_edge` 一样：
  /// 每一次都要留下一条日志，**包括被忽略的那一次** —— "按了没反应"只有两种可能
  /// （事件没产生 / 产生了但当时没有资格），而这里的 `not_eligible` 就是后者的证据。
  void log_start_long_press(control::StartLandReply reply)
  {
    // ⚠️ 故意不写 `default:`：新增一条 `StartLandReply` 而漏掉文案时 `-Wswitch` 编译失败。
    switch (reply) {
      case control::StartLandReply::kLand:
        RCLCPP_WARN(
          get_logger(),
          "遥控器开始键：**长按 %.1f s** ⟹ 立即自动降落（任务就此正常收口，"
          "`end_reason=%s`）。",
          rc_start_.land_hold_s, control::to_string(control::EndReason::kLandPressed));
        return;
      case control::StartLandReply::kNotEligible:
        RCLCPP_WARN(
          get_logger(),
          "遥控器开始键：长按**被忽略**了 —— 这一次按下发生在**未解锁**的时候"
          "（当前任务态 %s）。★ 判据看的是**按下那一刻**，不是按满的这一刻："
          "按完起飞再一直按着，不会让飞机刚离地就降落。",
          fsm_->task_name());
        return;
      case control::StartLandReply::kTakeover:
        RCLCPP_WARN(
          get_logger(),
          "遥控器开始键：长按**被忽略**了 —— **控制权不在我们手上**（`degrade=%s`，"
          "见同一拍的诊断块）。★ 我们此时不请求任何模式，连 `AUTO.LAND` 也不请求："
          "不跟正在接管飞机的人抢。",
          control::to_string(fsm_->latched_reason()));
        return;
      case control::StartLandReply::kAlreadyClosing:
        RCLCPP_WARN(
          get_logger(),
          "遥控器开始键：长按**不用管了** —— 程序已经在收口（当前任务态 %s，`degrade=%s`）。",
          fsm_->task_name(), control::to_string(fsm_->latched_reason()));
        return;
    }
  }

  /// **松手**（确认低位）那一帧 (ticket 07 增补)。★ 只有"兑现了欠下的开始"才打日志 ——
  /// 每一次松手都打的话，"上电时开关本来就在低位"那条也会冒出来。
  void log_start_release(bool entered_track)
  {
    if (!entered_track) {
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "遥控器开始键：**松手** ⟹ 进入 TRACK（目标 = 轨迹原点 + %s），下一帧发 %s 让 traj "
      "把参考时钟归零。",
      topics_.traj_reference.c_str(), topics_.traj_start.c_str());
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
        // ★ 兜底时限的两种配法要**分开写**（而不是把 0 打成一个时间）：`<= 0` 在这个栈里
        //   意思是"永不超时"，打成 "0.0 s" 会让现场以为"马上就要自己降落了"。
        const std::string deadline = (task_limits_.wait_start_timeout_s > 0.0)
          ? std::to_string(task_limits_.wait_start_timeout_s) + " s 后自动降落"
          : std::string("**永不超时**（wait_start_timeout_s <= 0）");
        // ★★ 「轨迹原点在日志里可复核」是本张的验收项之一 ⟹ 用固定的
        //    `traj_origin=(x, y, z)` 形状打出来，可 grep、可与动捕原点对账。
        RCLCPP_WARN(
          get_logger(),
          "→ %s：**稳定判据成立**。traj_origin=%s（= 判稳那一刻 EKF2 报的位置，ENU）。\n"
          "  ★ 这条设计让 Phase 1 与 Phase 2 的原点问题解耦：之后一切目标都以它为 0，"
          "EKF2 局部原点与动捕原点之间的常值偏移被自动吸收。\n"
          "  ★ 目标已切到轨迹原点本身（**停在原地等**）——从这里开始飞机只认遥控器的开始键：\n"
          "      第 2 次上升沿 ⟹ 进 TRACK（开始飞轨迹）；一直不按 ⟹ 最多等 %s\n"
          "  ⚠️ **轨迹还没被 `traj` 播出来**：%s 只在收到 %s 之后才有帧，"
          "而那条消息要等进 TRACK 才由本节点发出。\n"
          "  ★ 上锁由 PX4 的 COM_DISARM_LAND = 2.0 完成，我们不发上锁。",
          s.c_str(), has_traj ? format_vec3(tx, ty, tz).c_str() : "（异常：未记录）",
          deadline.c_str(), topics_.traj_reference.c_str(), topics_.traj_start.c_str());
        return;
      }
      case control::Phase::kTrack: {
        double tx = 0.0, ty = 0.0, tz = 0.0;
        const bool has_traj = fsm_->traj_origin(&tx, &ty, &tz);
        RCLCPP_WARN(
          get_logger(),
          "→ %s：**开始轨迹**（遥控器开始键的第 2 次上升沿；★ 受理 ≠ 已动 —— 第一帧的参考"
          "还没到，见紧接着的那一条「轨迹参考已经来了」）。\n"
          "  ★ 目标 = traj_origin %s **+** %s 上的位置 —— traj 发出来的是**相对轨迹原点**的参考，"
          "把原点丢掉（直接拿参考当目标）会让飞机当场俯冲扎向地面（它的高度是 0 起步的）。\n"
          "  ★ 完成度：%s 逐字转发到 %s；到 1.0 ⟹ 任务飞完 ⟹ 请求 %s。\n"
          "  ★ 参考的年龄超过 %.1f s 就先保持上一个目标并重发 %s"
          "（那是「运行中重选轨迹」的补救）；超过 %.1f s 仍然没有 ⟹ degrade=traj_timeout。",
          s.c_str(), has_traj ? format_vec3(tx, ty, tz).c_str() : "（异常：未记录）",
          topics_.traj_reference.c_str(),
          topics_.traj_progress.c_str(), topics_.control_traj_progress.c_str(),
          land_mode_.c_str(),
          task_limits_.traj_stream_gap_s, topics_.traj_start.c_str(),
          task_limits_.traj_timeout_s);
        return;
      }
      case control::Phase::kLand:
        // 两种情况（失效降级 / 任务正常走完）在这里分开说 —— 动作逐字相同，只有原因不同。
        // 降级那条的完整说明（含证据与复位条件）由上一条 `degrade=` 日志给出。
        // ★★ ticket 09：判据从「`mission_complete` 布尔」换成「`end_reason` 枚举」——
        //   本张有了**两条**正常出口（轨迹跑完 / 等开始键超时），一个布尔量装不下了。
        if (d.end_reason != control::EndReason::kNone) {
          RCLCPP_WARN(
            get_logger(),
            "→ %s：**任务正常走完**（end_reason=%s）⟹ 已停止发布 setpoint，并请求 %s。\n"
            "  ★ 上锁不由我们发：PX4 的 COM_DISARM_LAND = 2.0（z0 实测值）在它自己的着陆检测"
            "判定落地 2 s 后上锁，我们只等 armed 的降沿。\n"
            "  ★ 复位条件：**落地上锁**（armed → false）后自动复位，不需要重启节点。",
            s.c_str(), control::to_string(d.end_reason), land_mode_.c_str());
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
        // ★★ 措辞刻意**不写**"EKF2 位置失效"：那句话归 `kEkf2PosLost`（ticket 16）。
        //   本判据在真机上**永不触发**（PX4 的 `PublishLocalPosition` 不合成 NaN、MAVLink 又丢掉
        //   `xy_valid`）—— 它只在夹具注入 NaN 时成立，是**防御性**的。日志若打出这一条，
        //   说明读数真的坏了，但**别**据此去找 EKF2 的问题。
        return topics_.local_position + " 最近一帧含 NaN/Inf（**防御性判据**，真机上 EKF2 不发 NaN；"
                                         "「EKF2 位置失效」请看 ekf2_pos_lost）";
      case control::Degrade::kMocapStampStalled:
        return topics_.mocap + " 有帧（静默 " + age_str(topics_.mocap, now_s) +
               "）但时间戳连续不前进超过 " + std::to_string(limits_.mocap_stamp_stall_s) + " s";
      case control::Degrade::kEkf2PosLost:
        // ★ 证据是**读数本身**（EKF2 自报的状态位），不是我们的推断 —— 且必须带上"这条输入是
        //   什么时候的"：本判据有一道新鲜度门（见 `px4_interface.cpp::health()`），
        //   陈旧的话题会让判据**不成立**，所以"静默多久"是排查的第一个数。
        return topics_.estimator_status + " 报 pos_horiz_abs=false 连续 ≥ " +
               std::to_string(limits_.ekf2_pos_lost_samples) + " 帧（1 Hz ⟹ 约 " +
               std::to_string(limits_.ekf2_pos_lost_samples) + " s；该话题静默 " +
               age_str(topics_.estimator_status, now_s) + "）⟹ EKF2 自报水平位置没有绝对参考";
      case control::Degrade::kTrajTimeout:
        // ★ 证据要把**两种断流原因**都摆在台面上（读数本身分不开它们）：一是"我们从来没要过
        //   这个流"（`/traj/start` 发了但 traj 没选轨迹 ⟹ 它只打一条 WARN 就忽略），
        //   二是"要过，但运行中被重选了"。前者看 `起点至今`，后者看"这条话题曾经有过帧"。
        return topics_.traj_reference + " 静默 " + age_str(topics_.traj_reference, now_s) +
               "（判据 " + std::to_string(task_limits_.traj_timeout_s) + " s；进 TRACK 至今 " +
               std::to_string(now_s - track_entered_s_) + " s，期间发出过 " +
               std::to_string(traj_start_sent_count_) + " 次 " + topics_.traj_start +
               "（第 1 次是进场那次；> 1 就是断流后重发过））" +
               "；发布者：" + publisher_summary(topics_.traj_reference) +
               "；★ 最常见的原因：traj 侧**没有选中任何轨迹**（`/traj/start` 被它忽略并打了 "
               "WARN），或运行中有人重选了轨迹";
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
      case control::Degrade::kEkf2PosLost:
        // ★ 文案要把**边界**说清楚：这条判据只看 EKF2 的状态位，不等于"话题上没数据"，
        //   也不等于动捕真的坏了 —— 它是"帧可能还在到，但 EKF2 不肯用"。
        // ★ 复位**只有一条路**：落地上锁。别在这里写"状态位恢复即复位" —— 判据本身带解锁门
        //   （`first_degrade()` ⑥）⟹ 它一旦成立就必然是 `Latch::kArmedDegrade`（`fsm.hpp`），
        //   而在空中把闩锁解开等于**未经确认就恢复发 setpoint**。与 `describe_reset()` 同源。
        return "EKF2 自报水平位置**没有绝对参考**（" + topics_.estimator_status +
               " 的 pos_horiz_abs 连续为假，**只在已解锁时**算失效）⟹ 已停发；已解锁时请求 " +
               land_mode_ + "（它自己也要求位置，很可能被拒 —— 那时靠 PX4 的失效保护）；"
               "复位：**落地上锁**（状态位恢复**不会**在空中把它解掉）";
      case control::Degrade::kTrajTimeout:
        // ★ 与上面那条 `kEkf2PosLost` 同一个形状：判据只在 `TRACK` 里成立，
        //   而 `TRACK` 只在已解锁时进得去 ⟹ 它一旦成立就必然是 `kArmedDegrade` 闩锁，
        //   复位**只有**落地上锁一条路（在空中把闩锁解开 = 未经确认就恢复发 setpoint）。
        return "轨迹参考断流 ⟹ 已停发；已解锁时请求 " + land_mode_ +
               "（这一条**不是**\"轨迹跑完了\"—— 跑完走的是正常出口，见 end_reason）；"
               "复位：**落地上锁**（参考恢复**不会**在空中把它解掉）";
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

    // ★★ ticket 09：「在 `TRACK` 里、但参考还没来」——这是本张唯一的"按键有效果但飞机不动"路径。
    //   受理（进 TRACK）≠ 飞机开始动：目标要等 `/traj/reference` 的第一帧才有变化，而那一帧
    //   要等 `traj` 收到 `/traj/start` 之后才发。所以必须有一处持续把"在等什么"说出来，
    //   否则操作员只看到 `/control/state` 变成 AUTO/TRACK 然后飞机稳稳停着。
    //   ⚠️ 与 `degrade=traj_timeout` 是**先后**关系（那个到点就停发了），这里说的是"还没到点"。
    if (last_phase_ == control::Phase::kTrack) {
      const control::TrajReference traj = px4_->traj(now_s);
      if (!traj.alive(task_limits_.traj_stream_gap_s)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "已在 TRACK，但 %s 上还没有参考（最后一次 %.3f s 前；判 traj_timeout 的门槛 %.1f s）"
          "⟹ 飞机**原地保持**（目标 = 轨迹原点），不会自己开始飞。\n"
          "  已发出 %d 次 %s（第 1 次是进场那次；> 1 就是断流后重发过）。\n"
          "  ★ 若一直不来，先看 `traj` 那边有没有**选中轨迹** —— 没选的话 /traj/start 会被它"
          "忽略（它只打一条 WARN），于是本节点永远收不到参考。\n"
          "  ★ 发布者：%s",
          topics_.traj_reference.c_str(), traj.age_s, task_limits_.traj_timeout_s,
          traj_start_sent_count_, topics_.traj_start.c_str(),
          publisher_summary(topics_.traj_reference).c_str());
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
      case control::Degrade::kEkf2PosLost:
      case control::Degrade::kTrajTimeout:
        // 输入类：落地上锁才复位（未解锁时则是"判据恢复即自动复位"，不会闩锁）。
        // ★ `kEkf2PosLost` 走这条是因为它**本来就在已解锁时才成立**（`first_degrade` 的解锁门）
        //   ⟹ 它一旦成立就必然是 `kArmedDegrade` 闩锁，只能等落地上锁。
        //   `kTrajTimeout` 同理：它只在 `TRACK` 里可能为真，而 `TRACK` 只在已解锁时进得去。
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
      // ★ ticket 16：它**不进任何超时判据**（见 `Timeouts::estimator_s`），但每一条
      //   "EKF2 位置失效"的日志都要靠这一行的"静默多久"来判断"我们到底知不知道现在的情况"。
      {"estimator_status", &topics_.estimator_status},
      // ★ ticket 09：`traj` 的参考。⚠️ "静默"这一列在这里**要按任务态读**：`traj` 在收到
      //   `/traj/start` 之前**根本不发**参考（`tick()` 开头 `if (!running_) return;`）
      //   ⟹ 没进 TRACK 时"从未收到过"是**正常**的，不是故障。判据侧只看一个数：
      //   `TaskLimits::traj_timeout_s`，而它的计时起点是**进 TRACK 的那一刻**。
      //   （完成度那一行同理：它连"跑完保持 1.0"都只在跑过之后才发生。）
      {"traj_reference", &topics_.traj_reference},
      {"traj_progress", &topics_.traj_progress},
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
  /// 遥控器开始键的判据（`px4_interface.hpp::RcStart`，§H.1）—— 日志里要引用，所以节点留一份。
  /// ★ 边界层自己也算了一份（`Px4Config::rc_start` 交过去的正是它）⟹ 两处**同源**，
  ///   这里只看不改。
  control::RcStart rc_start_;
  /// 任务判据（`fsm.hpp::TaskLimits`）—— 日志里要引用，所以节点留一份。
  control::TaskLimits task_limits_;

  std::unique_ptr<control::Fsm> fsm_;
  std::unique_ptr<control::Px4Interface> px4_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  /// ★ ticket 09：`/control/traj_progress` —— 逐字转发 `/traj/progress`（转发在边界层那条
  /// 订阅的回调里，见 `px4_config.on_traj_progress`）。
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr traj_progress_pub_;
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

  // —— 轨迹流（ticket 09 的诊断；**不进任何判据**，判据全在 `fsm.hpp`）——
  /// 上一拍"参考流是否可用"（`detect_traj_stream()` 用它做进出边沿）。
  bool traj_stream_seen_{false};
  double last_traj_progress_log_s_{0.0};
  /// 进 `TRACK` 的时刻与"进 TRACK 之后发出过几次 `/traj/start`"。
  /// ★ 只在 `degrade=traj_timeout` 的证据里用：那一条必须能分开
  ///   "我们从来没要过这个流"（计数恒为 1）与"要过，但运行中被打断了"（计数 > 1）。
  double track_entered_s_{0.0};
  int traj_start_sent_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
