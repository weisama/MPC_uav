#include "control/px4_interface.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "mavros_msgs/msg/position_target.hpp"

namespace control
{

namespace
{
/// ★ 位置 + yaw：忽略速度(8/16/32)、加速度(64/128/256)、yaw_rate(2048)。
///   注意**没有** FORCE(512) —— PX4 只在"加速度分量有效"时才检查 FORCE，这里加速度被忽略 ⟹ 那条
///   错误分支不会触发。逐位含义见 `/opt/ros/humble/share/mavros_msgs/msg/PositionTarget.msg`
///   与 PX4 v1.14.3 `mavlink_receiver.cpp::handle_message_set_position_target_local_ned`。
constexpr uint16_t kTypeMaskPositionAndYaw =
  mavros_msgs::msg::PositionTarget::IGNORE_VX |
  mavros_msgs::msg::PositionTarget::IGNORE_VY |
  mavros_msgs::msg::PositionTarget::IGNORE_VZ |
  mavros_msgs::msg::PositionTarget::IGNORE_AFX |
  mavros_msgs::msg::PositionTarget::IGNORE_AFY |
  mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
  mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

static_assert(kTypeMaskPositionAndYaw == 2552, "type_mask 必须是 2552（ticket 05 的验收项）");

/// ★★ 票 15：`/control/target` 的掩码 —— 位置 + **速度** + yaw 三路都有效，
///   只忽略加速度(64/128/256)与 yaw_rate(2048) ⟹ 2496。
///   ⚠️ 与上面那条**不是**同一个数，也**不能**互换：给外部控制器的参考少了速度，
///   模型预测控制的时域外推就退化成"原地保持"，而话题面上每帧都正常。
constexpr uint16_t kTypeMaskPositionVelocityAndYaw =
  mavros_msgs::msg::PositionTarget::IGNORE_AFX |
  mavros_msgs::msg::PositionTarget::IGNORE_AFY |
  mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
  mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

static_assert(kTypeMaskPositionVelocityAndYaw == 2496, "type_mask 必须是 2496（票 15）");

/// 摇杆检测器**首帧**用的名义周期（s）。只在"没有上一帧"时用得上（那一路不会去算变化率），
/// 取 20 Hz 只是为了让 `update()` 收到一个正数。
constexpr double kNominalRcDtS = 0.05;

/// 位置/姿态是否有 NaN/Inf（EKF2 位置失效时 mavros 会把 NaN 原样转发过来）。
bool pose_is_finite(const geometry_msgs::msg::Pose & p)
{
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(p.orientation.x) &&
         std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) &&
         std::isfinite(p.orientation.w);
}

/// 速度三分量是否有限（同一条 MAVLink 消息的 NaN 会同时打到位置与速度上，见 `odometry()`）。
bool vector_is_finite(const geometry_msgs::msg::Vector3 & v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
}  // namespace

const char * to_string(OutputMode m)
{
  switch (m) {
    case OutputMode::kPosition:
      return "position";
    case OutputMode::kExternal:
      return "external";
  }
  return "?";
}

OutputMode parse_output_mode(const std::string & name)
{
  if (name == "position") {
    return OutputMode::kPosition;
  }
  if (name == "external") {
    return OutputMode::kExternal;
  }
  throw std::invalid_argument(
          "output_mode=" + name + " 不认识，只能是 position（本节点直接发 "
          "/mavros/setpoint_raw/local）或 external（发 /control/target，由外部控制器发执行器指令）");
}

Px4Interface::Px4Interface(rclcpp::Node & node, const Px4Config & config)
: node_(node), topics_(config.topics), timeouts_(config.timeouts), limits_(config.limits),
  modes_(config.modes), rc_start_(config.rc_start), traj_progress_cb_(config.on_traj_progress),
  // ★ 开始键的判据（§H.1 的那一组数）—— 有状态，所以在这里构造而不是每帧现算。
  start_(rc_start_.start_threshold, rc_start_.release_threshold, rc_start_.debounce_s,
    rc_start_.low_us, rc_start_.high_us, rc_start_.land_hold_s)
{
  // 摇杆阈值来自参数（`COM_RC_STICK_OV` 的 µs 等价物）⟹ 检测器在这里才构造。
  for (StickMoveDetector & det : stick_) {
    det = StickMoveDetector(limits_.rc_stick_ov_us_per_s);
  }
  // ★ QoS 逐条对齐真实 mavros 2.15.1 的发布方 —— 这一层**错了不会报错，只会"永远收不到"**
  //   （与"时间戳单位错了只会没人收"同一类静默失败）。已读源码确认：
  //     · `/mavros/state`        ：`QoS(10).transient_local()`（可靠 + 瞬态本地）
  //                                `sys_status.cpp:579-584` `mavros::StateQoS()`
  //     · `/mavros/local_position/*`、`/mavros/battery`：`SensorDataQoS()` = **best_effort**
  //                                `local_position.cpp:88-107`、`sys_status.cpp:595`
  //     · `/mavros/rc/in`        ：默认 `10`（可靠）`rc_io.cpp:48`
  //     · `/mavros/setpoint_raw/local`（mavros 是**订阅方**）：`SensorDataQoS()` best_effort
  //                                `setpoint_raw.cpp:67` ⟹ 我们**发布**用默认（可靠）即可兼容
  //                                （offered ≥ requested）；反过来发布 best_effort 会让 mavros 收不到。
  //   ⚠️ `local_position` 若按默认（可靠）订阅，与 mavros 的 best_effort 发布方**不兼容** ⟹ 静默无数据。
  inputs_[topics_.state] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false, timeouts_.state_s};
  inputs_[topics_.local_position] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false,
    timeouts_.local_pos_s};
  // ★ 速度与位置**共用 `local_pos_s`**：它们出自 mavros 的**同一个回调**、同一条 MAVLink
  //   消息（见 `Px4Topics::local_velocity` 的说明）⟹ 一方静默另一方必然也静默，
  //   给两个超时参数只会多出一个"调了一个另一个没跟上"的坑。
  inputs_[topics_.local_velocity] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false,
    timeouts_.local_pos_s};
  inputs_[topics_.rc] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false, timeouts_.rc_s};
  inputs_[topics_.battery] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false, timeouts_.battery_s};
  inputs_[topics_.mocap] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false, timeouts_.mocap_s};
  // ★ ticket 16：登记进来才有"多久没到"可看（诊断），但它**不进任何判据**（见 `Timeouts::estimator_s`）。
  inputs_[topics_.estimator_status] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false,
    timeouts_.estimator_s};
  // ★★ ticket 09：`traj` 的两条输入。⚠️ 这里的**超时字段刻意留 0.0，它没有含义** ——
  //   "参考多久没来算失效"是**任务层**的事（`TaskLimits::traj_stream_gap_s` / `traj_timeout_s`）：
  //   那个时限必须与"我们什么时候开始要这个流"一起看（进 `TRACK` 才生效），而边界层不知道
  //   任务在做什么。登记进来只为一件既有的事：诊断面能报"这条输入静默多久"（`describe_inputs`）。
  //   ★ 所以别在这里调 `fresh()`（恒为假）——要年龄就调 `age_seconds()` / `traj()`。
  inputs_[topics_.traj_reference] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false, 0.0};
  inputs_[topics_.traj_progress] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false, 0.0};

  state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
    topics_.state, rclcpp::QoS(10).transient_local(),
    [this](mavros_msgs::msg::State::ConstSharedPtr msg) {on_state(*msg);});

  local_position_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    topics_.local_position, rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_local_position(*msg);});

  // ★ 速度的 QoS 与 `local_position` 逐字相同（同为 `SensorDataQoS` = best_effort）——
  //   已核对 mavros 源码：两者在 `local_position.cpp` 里由**同一个回调**发布，
  //   用的也是同一个 publisher QoS。⚠️ 这一层错了不会报错，只会"永远收不到"。
  local_velocity_sub_ = node_.create_subscription<geometry_msgs::msg::TwistStamped>(
    topics_.local_velocity, rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
      // ★ 不做"新鲜度"以外的判断：NaN 的帧**照常刷新新鲜度**（帧确实在到），
      //   "这一帧能不能用"由 `odometry()` 每次现算 —— 与 `on_local_position` 的做法不同，
      //   因为那里 NaN 是**降级判据**（`position_invalid`）而这里不是。
      local_velocity_ = *msg;
      slot(topics_.local_velocity) = {node_.now(), true, timeouts_.local_pos_s};
    });

  rc_sub_ = node_.create_subscription<mavros_msgs::msg::RCIn>(
    topics_.rc, rclcpp::QoS(10),
    [this](mavros_msgs::msg::RCIn::ConstSharedPtr msg) {on_rc(*msg);});

  battery_sub_ = node_.create_subscription<sensor_msgs::msg::BatteryState>(
    topics_.battery, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
      // ★ 低电压判据只看**电压**，不看电流（ticket 06 明确要求）。
      // ★ `voltage > 0` 这一条是必须的：mavros 在没有电池读数时发 0.0
      //   （z0.params 里 `SENS_EN_BATT = 0`、`BAT1_SOURCE = 0`，飞控可能根本不上报），
      //   把 0 V 当"低压"会让没接电池的台架**永远在降落**。NaN 同理（`std::isfinite`）。
      const float v = msg->voltage;
      const bool usable = std::isfinite(v) && (v > 0.0f);
      battery_voltage_ = usable ? std::optional<double>(static_cast<double>(v)) : std::nullopt;
      battery_low_ = usable && (static_cast<double>(v) < limits_.low_voltage_v);
      slot(topics_.battery) = {node_.now(), true, timeouts_.battery_s};
    });

  mocap_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    topics_.mocap, rclcpp::QoS(10),
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_mocap(*msg);});

  // ★★ ticket 16：QoS **故意用易失**（默认 `QoS(10)`），而不是真实 mavros 发布方的
  //   `QoS(10).transient_local()`（`mavros/qos.hpp:60` `StateQoS()`，`sys_status.cpp:579` 用它发
  //   `estimator_status`）。两种 QoS 都兼容（可靠发布方 + 可靠订阅方，durability 只要求
  //   offered ≥ requested），差别在**启动时会不会回放历史**：
  //     · transient_local 订阅 ⟹ 复现 2026-09-22 现场实测到的现象：一订阅上来就"唰"地回放
  //       上一会话缓存的 ~10 帧。若那批旧帧里 `pos_horiz_abs` 是假（上一轮的失效状态），
  //       本判据的**连续计数**会当场被喂满 ⟹ 节点一启动就报 `ekf2_pos_lost`。
  //     · 易失订阅 ⟹ 只看得见"从现在起"的帧（1 Hz，最多晚 1 s 才有第一帧）。
  //   ⟹ 这里要的是后者：**判据必须建立在本次会话收到的样本上**，"上一轮的旧读数"不是证据。
  estimator_status_sub_ = node_.create_subscription<mavros_msgs::msg::EstimatorStatus>(
    topics_.estimator_status, rclcpp::QoS(10),
    [this](mavros_msgs::msg::EstimatorStatus::ConstSharedPtr msg) {on_estimator_status(*msg);});

  // ★★ ticket 09：`traj` 的两条输入。QoS 用默认（`QoS(10)` = 可靠 + 易失），与**发布方逐字一致**：
  //   已读源码确认 `traj_node.cpp` 的 `reference_pub_` / `progress_pub_` 都是 `rclcpp::QoS(10)`
  //   （没有 transient_local、没有 best_effort）⟹ 这里的 durability/reliability 对得上。
  //   ⚠️ 这一层错了不会报错，只会"永远收不到"（同下面那段 QoS 说明）。
  //   ★ **易失是对的**，不只是"够用"：`traj` 在 `/traj/start` 之前**根本不发**参考
  //   （`tick()` 开头 `if (!running_) return;`）⟹ 节点启动时本来就该"没有参考"，
  //   回放上一会话的最后一条参考会让"进 TRACK 第一帧就有一个陈旧目标"。
  traj_reference_sub_ = node_.create_subscription<mavros_msgs::msg::PositionTarget>(
    topics_.traj_reference, rclcpp::QoS(10),
    [this](mavros_msgs::msg::PositionTarget::ConstSharedPtr msg) {on_traj_reference(*msg);});

  traj_progress_sub_ = node_.create_subscription<std_msgs::msg::Float32>(
    topics_.traj_progress, rclcpp::QoS(10),
    [this](std_msgs::msg::Float32::ConstSharedPtr msg) {on_traj_progress(*msg);});

  // 唯一的控制输出。发布方用默认（可靠）QoS：mavros 以 best_effort 订阅，可靠发布方可兼容；
  // 反过来（best_effort 发布）会让 mavros 收不到。
  // ★★ 票 15：**两档各建一条，绝不两条都建** —— "同一时刻只有一条 PX4 setpoint 流在写"
  //   这件事因此在**结构上**成立（不是靠调用方记得别调用另一个）。
  //   position ⟹ `/mavros/setpoint_raw/local`；external ⟹ `/control/target`（外部控制器再发 PX4）。
  if (config.output_mode == OutputMode::kPosition) {
    setpoint_pub_ = node_.create_publisher<mavros_msgs::msg::PositionTarget>(topics_.setpoint, 10);
  } else {
    control_target_pub_ = node_.create_publisher<mavros_msgs::msg::PositionTarget>(
      topics_.control_target, 10);
  }

  // ★ ticket 09：两条**我们对外提供**的话题。用默认 QoS（可靠 + 易失）：
  //   · `/traj/start` —— `traj_node.cpp` 的订阅者也是 `rclcpp::QoS(10)`；
  //   · `/control/traj_progress` —— 观测面，订阅方是操作员/工具（`ros2 topic echo` 默认可靠）。
  //   ⚠️ 它**不是**状态话题：`traj` 没在跑的时候我们什么都不发（"没有进度"≠"进度是 0"，
  //     与 `/control/state` 那种 transient_local 的心跳不同）。
  traj_start_pub_ = node_.create_publisher<std_msgs::msg::Empty>(topics_.traj_start, 10);
  control_traj_progress_pub_ = node_.create_publisher<std_msgs::msg::Float32>(
    topics_.control_traj_progress, 10);
  set_mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>(topics_.set_mode_service);
  arming_client_ = node_.create_client<mavros_msgs::srv::CommandBool>(topics_.arming_service);
}

/// `/mavros/state` 一帧 —— 除记录之外，**唯一**的额外职责是在 `armed` 的上升沿上兑现
/// "这次解锁是谁发起的"（ticket 08）。
/// ★ 为什么在这里而不是在 `guards()` 里：`guards()` 是**只读**的（`const`），
///   而上升沿是个**事件** —— 放在每帧都跑、可能被调用多次的只读路径里，
///   要么改不掉 `const`，要么同一帧被兑现两次。
void Px4Interface::on_state(const mavros_msgs::msg::State & msg)
{
  const bool was_armed = state_.armed;
  state_ = msg;

  if (state_.armed && !was_armed) {
    // ★ 上升沿：把"我们请求过"兑现成"这次是我们做的"。**消费掉** pending：
    //   一次请求只认一次上升沿（否则后续每一次外部解锁都会被记成我们的）。
    arming_by_us_ = arm_request_pending_;
    arm_request_pending_ = false;
  } else if (!state_.armed) {
    // 上锁：本次解锁的归属作废（下次解锁重新判定）。
    // ⚠️ **只清归属、不清 `arm_request_pending_`** —— 见它的成员注释：应答与上升沿之间
    //   必然夹着几帧 `armed == false`，在这里清掉就等于"我们自己的解锁从来不是我们做的"。
    arming_by_us_ = false;
  }

  slot(topics_.state) = {node_.now(), true, timeouts_.state_s};
}

Px4Interface::Input & Px4Interface::slot(const std::string & topic)
{
  return inputs_.at(topic);
}

/// 遥控器帧 —— 两条职责：刷新新鲜度（ticket 06 的第三条超时判据），以及把四个摇杆通道
/// 喂给变化率检测器（打杆夺权）。
void Px4Interface::on_rc(const mavros_msgs::msg::RCIn & msg)
{
  const rclcpp::Time now = node_.now();
  const double dt = rc_timed_ ? (now - last_rc_time_).seconds() : 0.0;

  // ★ 链路断过一次（超过它自己的超时）之后，检测器的历史就作废了：那时"上一帧的值"是
  //   断链前的读数，拿它算变化率会把"链路恢复 + 摇杆本来就在偏位"当成一次打杆。
  const bool gap = !rc_timed_ || (dt > timeouts_.rc_s);
  if (gap) {
    for (StickMoveDetector & det : stick_) {
      det.reset();
    }
    // ★★ ticket 09：开始键也在这一份复位里。理由与摇杆**不同但同源**：断链期间这档开关
    //   动没动过我们不知道，而"已确认过低电平"（§H.5.2 判据的那一位）在那之前是真的 ⟹
    //   "链路断了、开关恰好停在上位"会在链路恢复的第一帧凭空造出一条边沿，而那条边沿
    //   会被当成"起飞"或"开始轨迹"。见 `StartSwitchDetector::reset()`。
    start_.reset();
  }
  const double dt_use = (dt > 1e-3) ? dt : kNominalRcDtS;

  bool moving = false;
  const std::array<int, 4> channels = limits_.rc_channels();
  const size_t count = msg.channels.size();
  for (size_t i = 0; i < stick_.size(); ++i) {
    // 1-based 参数 → 0-based 下标；通道数不足时**跳过而不是抛异常**
    // （遥控器通道数取决于接收机；本张的验收项 7 要求"任何失效下节点不崩"）。
    const int idx = channels[i] - 1;
    if (idx < 0 || static_cast<size_t>(idx) >= count) {
      continue;
    }
    if (stick_[i].update(static_cast<double>(msg.channels[static_cast<size_t>(idx)]), dt_use)) {
      moving = true;
    }
  }
  sticks_moving_ = moving;

  // ★ ticket 09：同一帧还要喂**开始键**（1-based 通道号 → 0-based 下标）。
  //   ⚠️ 越界时**跳过而不是抛异常**（通道数取决于接收机，同上面那个循环）；
  //     更要紧的是：跳过之后检测器**一个样本都没收到** ⟹ `sample_valid()` 为假 ⟹
  //     `take_start_edge()` 永远返回 false。这正是"通道号配错了"在现场的表现
  //     （按了没反应），诊断里靠 `sample_valid()` 认出来。
  {
    const int idx = rc_start_.channel - 1;
    if (idx >= 0 && static_cast<size_t>(idx) < count) {
      start_.update(static_cast<double>(msg.channels[static_cast<size_t>(idx)]), now.seconds());
    }
  }

  last_rc_time_ = now;
  rc_timed_ = true;
  slot(topics_.rc) = {now, true, timeouts_.rc_s};
}

/// 动捕帧 —— 除新鲜度之外还要盯**时间戳是否前进**（ticket 06 验收项 6 的"注入时间戳倒退"）。
/// 理由：话题层面"有数据"不等于 EKF2 眼里"有数据" —— mavros 会丢弃戳不前进的 vision_pose
/// （`send_vision_estimate()` 里戳相等即 return），EKF2 也会丢过旧的样本。
void Px4Interface::on_mocap(const geometry_msgs::msg::PoseStamped & msg)
{
  const rclcpp::Time now = node_.now();
  const rclcpp::Time stamp(msg.header.stamp);

  // ★★ 基准是**历史最大戳**，不是"上一帧的戳"（2026-09-22 复核时改，第一手依据）：
  //   EKF2 收外部视觉样本的判据就是"必须比**已缓冲的最新样本**还新"
  //     `EKF/estimator_interface.cpp:393`：`if (time_us >= …get_newest().time_us + _min_obs_interval_us)`
  //     else 分支 `:401-403` 只打一条 `ECL_WARN("EV data too fast …")` 就**把这一帧丢掉**。
  //   拿"上一帧"当基准的话，**倒退到旧纪元之后继续正常前进**（夹具 `VrpnSource::rewind` 的形状）
  //   每一帧戳都比上一帧大 ⟹ 判据永远说"健康"，而 PX4 那边每一帧都在丢 —— 那是真盲区。
  //   改成最大戳之后，判据的字面意思变成"已见过的最新时刻不再被超过"，与 EKF2 同一把尺子。
  if (mocap_stamp_valid_ && stamp <= mocap_stamp_max_) {
    // 没超过历史最大戳（相等 = 冻住；更小 = 倒退/旧纪元）：开始计时。
    if (!mocap_stall_counting_) {
      mocap_stall_counting_ = true;
      mocap_stall_start_ = now;
    }
    mocap_stalled_ = (now - mocap_stall_start_).seconds() >= limits_.mocap_stamp_stall_s;
  } else {
    mocap_stamp_max_ = stamp;
    mocap_stall_counting_ = false;
    mocap_stalled_ = false;
  }
  mocap_stamp_valid_ = true;

  // ★ 新鲜度**照常刷新**：帧确实在到。这正是这条判据必须独立存在的原因 ——
  //   它管的失效，新鲜度判据完全看不见。
  slot(topics_.mocap) = {now, true, timeouts_.mocap_s};
}

/// `local_position` 帧 —— NaN/Inf 是 EKF2 位置失效在话题层的表现。
void Px4Interface::on_local_position(const geometry_msgs::msg::PoseStamped & msg)
{
  if (!pose_is_finite(msg.pose)) {
    // ★ **不刷新新鲜度**：NaN 帧不是"一帧可用的位置"，把它算作新鲜会让 0.5 s 的超时判据
    //   一直不成立（于是节点停止流 setpoint 的理由变成"EKF2 没数据"，而日志里说不出
    //   真正的原因）。这样处理之后，NaN 那一帧就立刻降级，超时判据在 0.5 s 后跟着成立
    //   —— 两条判据的优先级里 `position_invalid` 排在前面（见 `fsm.hpp::first_degrade`）。
    position_invalid_ = true;
    return;
  }
  position_invalid_ = false;
  local_position_ = msg;
  slot(topics_.local_position) = {node_.now(), true, timeouts_.local_pos_s};
}

/// `estimator_status` 帧 —— ticket 16 的载体：数 `pos_horiz_abs` **连续**为假的帧数。
///
/// `pos_horiz_abs_status_flag` 是 MAVLink `ESTIMATOR_STATUS.flags` 的一位，到了 mavros 侧被拆成
/// 结构体成员（`EstimatorStatus.msg` / `sys_status.cpp:1296` 的解析）。PX4 侧它就是
/// `(cs.gps || cs.ev_pos) && _fault_status.value == 0`（`EKF/ekf_helper.cpp:761`）：
///   · `cs.ev_pos` = "外部视觉的位置正在被融合" ⟹ 动捕断了/戳冻住/融合失败都会让这一位落下来；
///   · `_fault_status.value == 0` = 没有任何传感器故障位（`bad_acc_vertical` 等）—— 是个**全有或全无**
///     的门，z0 的 mag 已禁用（`EKF2_MAG_TYPE=5` / `SYS_HAS_MAG=0`）所以那不是风险点，
///     但 `BADACC_PROBATION = 10 s` 期间它会保持置位（见 §4.4 的丢包风险一节）。
void Px4Interface::on_estimator_status(const mavros_msgs::msg::EstimatorStatus & msg)
{
  // ★ **对称**计数：真 ⟹ 立刻归零；假 ⟹ 累加。没有"半清"（不清也不加）的第三条路 ——
  //   别的方式（滑动窗口、计数衰减）会让"连续"这个词在代码里失去字面意思，而判据的
  //   全部理由就是"连续"（见 `Limits::ekf2_pos_lost_samples`）。
  if (msg.pos_horiz_abs_status_flag) {
    pos_lost_streak_ = 0;
  } else if (pos_lost_streak_ < limits_.ekf2_pos_lost_samples) {
    // 够数就不再涨（避免长期失效时把 int 涨上去；判据只看是否 ≥ 阈值）。
    ++pos_lost_streak_;
  }
  ekf2_pos_lost_ = (pos_lost_streak_ >= limits_.ekf2_pos_lost_samples);

  slot(topics_.estimator_status) = {node_.now(), true, timeouts_.estimator_s};
}

/// `/traj/reference` 一帧（ticket 09）—— **只记录，不做任何判断**。
///
/// ★ 这一层刻意不回答"这个参考还算不算新鲜"：那取决于"我们什么时候开始要这个流"（进 `TRACK`
///   才要），而边界层不知道任务在做什么（见 `traj()` 与 `inputs_` 里那两条的说明）。
/// ★ 位置**不在这里加轨迹原点**：`traj` 发出来的是**相对轨迹原点**的参考，
///   相加是 `fsm.hpp` 的 `TRACK` 分支做的事（`setpoint = traj_origin + reference`）。
///   在这里加会让"原点"这个概念出现在两个文件里，而原点只有状态机知道。
void Px4Interface::on_traj_reference(const mavros_msgs::msg::PositionTarget & msg)
{
  traj_reference_ = msg;
  slot(topics_.traj_reference) = {node_.now(), true, 0.0};
}

/// `/traj/progress` 一帧（ticket 09）—— 记录 + **逐字转发**给配置里的回调。
///
/// ★ 转发**逐字**（不重算、不钳位、不滤波）是本条接口的全部内容：它是给操作员看的那个数，
///   与 `fsm.hpp` 判"跑完了"用的是**同一帧**（转出去的值与判据不同源是台架上最难查的一类不一致）。
/// ★ 非有限值**不特殊处理**：`progress` 来自 `elapsed_ / duration_`，NaN 在两处比较里都为假
///   ⟹ 既不会被判成"跑完了"、也不会污染别的判据（安全侧），而在这里加一条分支反而会制造
///   "判据看到 NaN、转发出去的不是"这种两份事实。
void Px4Interface::on_traj_progress(const std_msgs::msg::Float32 & msg)
{
  traj_progress_ = static_cast<double>(msg.data);
  traj_progress_ever_ = true;
  if (traj_progress_cb_) {
    // ⚠️ 必须先判空：空 `std::function` 直接调用会抛 `std::bad_function_call`，
    //   而在订阅回调里抛异常会让节点崩 —— 那是本张"任何失效下节点不崩"的直接落地。
    traj_progress_cb_(msg.data);
  }
  slot(topics_.traj_progress) = {node_.now(), true, 0.0};
}

const Px4Interface::Input & Px4Interface::slot(const std::string & topic) const
{
  return inputs_.at(topic);
}

bool Px4Interface::fresh(const std::string & topic, double now_s) const
{
  const auto & in = slot(topic);
  if (!in.ever) {
    return false;
  }
  return (now_s - in.last_rx.seconds()) <= in.timeout_s;
}

bool Px4Interface::ever_received(const std::string & topic) const
{
  return slot(topic).ever;
}

double Px4Interface::age_seconds(const std::string & topic, double now_s) const
{
  const auto & in = slot(topic);
  return in.ever ? (now_s - in.last_rx.seconds()) : -1.0;
}

Guards Px4Interface::guards(double now_s) const
{
  Guards g;
  g.connected = fresh(topics_.state, now_s) && state_.connected;
  g.mocap_fresh = fresh(topics_.mocap, now_s);
  g.local_pos_fresh = fresh(topics_.local_position, now_s);
  // ★ 未收到过 `/mavros/state` 时 `state_` 是全零 ⟹ `armed == false` ⟹ 当"未解锁"处理。
  //   与 `mocap` 节点同一条纪律：把"没收到"当成"已解锁"会在启动瞬间伪造出一个"异常解锁"。
  g.armed = state_.armed;
  // 模式名的**翻译**只在这里（本包的一条纪律）：两处 `==` 字符串比较都在这一行区间里。
  g.in_offboard = (state_.mode == modes_.offboard);
  // ★ `in_land` 必须在这里算：它进的是 `mode_is_ours()`，而"我们请求的降落模式生效"被
  //   判成"被夺权"会让飞机一边降落一边把控制权交还给不存在的飞行员（见 `fsm.hpp`）。
  g.in_land = (state_.mode == modes_.land);
  // ticket 08：解锁的**归属**来自解锁服务的应答 + `armed` 的上升沿（见 `on_state`），
  // 不再是参数。未解锁时恒为 false ⟹ `guard_holds(kArmingByUs)` 恒成立（那条守卫的语义）。
  g.own_arming = arming_by_us_;
  return g;
}

Health Px4Interface::health(double now_s) const
{
  Health h;
  h.rc_fresh = fresh(topics_.rc, now_s);
  // ★ 摇杆判据只在**遥控器新鲜**时才成立：链路断了就报"超时"（更根本的原因），
  //   而不是拿着断链前的旧读数说"飞行员在打杆"（两者在日志里指向完全不同的排查方向）。
  h.rc_sticks_moving = h.rc_fresh && sticks_moving_;
  // ★ "没收到过电池" 必须是 false：`battery_low_` 只会被电池回调置位 ⟹ 天然满足。
  //   没接电池的台架上这条判据不会成立（否则永远在降落）。
  h.battery_low = battery_low_;
  h.position_invalid = position_invalid_;
  h.mocap_stamp_stalled = mocap_stalled_;
  // ★★ ticket 16：**必须**加这道新鲜度门，理由是这条判据的"坏消息"来自一个**能自清**的读数
  //   （`pos_horiz_abs` 每次发布现算）—— 但它只在**帧还在到**的时候才自清。话题一旦静默，
  //   `pos_lost_streak_` 就冻在最后一帧上：那时我们**不知道** EKF2 现在怎么样，
  //   让一个陈旧的真值一直挂着，等于把"不知道"说成"知道它坏了"（与"没收到电池不是低压"
  //   同一条纪律，见 `Health::battery_low`）。
  //   ⚠️ 代价（刻意接受）：`estimator_status` 单独静默（`local_position` 还在）时这条判据**失明**。
  //     没有更好的信号可用（这就是载体本身的边界），而真兜底是动捕那两条判据（不依赖本话题）。
  h.ekf2_pos_lost = fresh(topics_.estimator_status, now_s) && ekf2_pos_lost_;
  return h;
}

std::optional<geometry_msgs::msg::PoseStamped> Px4Interface::local_position() const
{
  // ★ 没有可用位置（从未收到 / 最近一帧是 NaN）⟹ nullopt。调用方据此不发 setpoint。
  if (!ever_received(topics_.local_position) || position_invalid_) {
    return std::nullopt;
  }
  return local_position_;
}

Odometry Px4Interface::odometry() const
{
  Odometry o;

  // ── 位置与姿态：同一条 `/mavros/local_position/pose`（出向已 NED→ENU）──
  // ★ "最近一帧是 NaN ⟹ 不可用"与 `local_position()` 同一条判据（都看 `position_invalid_`），
  //   两处必须一致：否则会出现"节点认为位置可用、状态机认为不可用"这种两份事实。
  o.position_valid = ever_received(topics_.local_position) && !position_invalid_;
  if (o.position_valid) {
    o.x = local_position_.pose.position.x;
    o.y = local_position_.pose.position.y;
    o.z = local_position_.pose.position.z;
    const auto & q = local_position_.pose.orientation;
    // ★ yaw 用 `fsm.hpp` 的纯函数算（seam 2 可测），这里**不做任何加减 π/2 的换算** ——
    //   ENU 口径进、ENU 口径出，mavros 负责换到 NED。见 `px4_interface.hpp` 文件头第 2 条。
    o.yaw = yaw_from_quaternion(q.w, q.x, q.y, q.z);
  }

  // ── 速度：另一条话题（`velocity_local`），但**同一条 MAVLink 消息**、同样是 ENU ──
  // ★ 这里**现算** NaN，不存标志位：速度的 NaN 不是独立的降级判据（它必然伴随位置 NaN，
  //   那条已经由 `position_invalid` 管了）⟹ 多存一个标志只是多一处会不同步的状态。
  if (ever_received(topics_.local_velocity)) {
    const auto & v = local_velocity_.twist.linear;
    o.velocity_valid = vector_is_finite(v);
    if (o.velocity_valid) {
      o.vx = v.x;
      o.vy = v.y;
      o.vz = v.z;
    }
  }
  return o;
}

TrajReference Px4Interface::traj(double now_s) const
{
  TrajReference t;
  const double age = age_seconds(topics_.traj_reference, now_s);
  // ★★ 从未收到过 ⟹ **`inf`**，而不是 `age_seconds()` 那个 −1：`TrajReference::age_s` 的语义是
  //   "距上一次参考多久了"，任何 `<=` 的比较都该判它"不新鲜"。−1 会让 `alive(gap)` 恒真 ⟹
  //   任务在**一个参考都没收到**的时候以为流是好的（那是最贵的一类错：目标冻在原点还继续飞）。
  t.age_s = (age < 0.0) ? std::numeric_limits<double>::infinity() : age;
  // ★ 值只在**这一帧有没有参考**上取用：调用方先看 `age_s`（`alive()`），不新鲜时**不该**用这三个数
  //   —— 它们那时是"最后一帧参考"，而 `fsm.hpp` 的 `TRACK` 分支在断流时保持**上一个目标**、
  //   不回退到这些残留值（见那里的注释）。放在这里只是让"最后一帧"可读，不是让它可用。
  t.x = traj_reference_.position.x;
  t.y = traj_reference_.position.y;
  t.z = traj_reference_.position.z;
  // ★★ 票 15：参考速度。`traj_node.cpp` 的参考掩码是 2496（速度有效），所以这三格是**真值**；
  //   位置那档不消费它，external 档（`/control/target`）拿它当前馈。
  //   与位置同一条纪律：`age_s` 不新鲜时调用方**不该**用这些残留值（`TRACK` 那一支会把
  //   断流时的前馈一起归零，见 `fsm.hpp`）。
  t.vx = traj_reference_.velocity.x;
  t.vy = traj_reference_.velocity.y;
  t.vz = traj_reference_.velocity.z;
  t.progress = traj_progress_;
  t.progress_seen = traj_progress_ever_;
  return t;
}

void Px4Interface::publish_traj_start()
{
  // ★ `Empty`：这条消息没有载荷 —— `traj` 收到就把参考时钟归零（`traj_node.cpp::on_start`）。
  // ★ **不做重试、不做去抖、不看应答**：什么时候该发是任务层算好的（`Decision::request_traj_start`
  //   是**脉冲**），"发几次、隔多久"的策略在 `fsm.hpp`（`track_started_` / `traj_stream_gap_s`）。
  //   这里多发一次就是一次真实的指令跳变（参考时钟归零 ⟹ 掉头飞回起点）。
  traj_start_pub_->publish(std_msgs::msg::Empty());
}

void Px4Interface::publish_setpoint(double x, double y, double z, float yaw)
{
  // ★ 出口再挡一次非有限值：目标由状态机从 `odometry()` 算出来，而那里已经挡过 NaN，
  //   但"绝不把 NaN 发给飞控"这条保证值钱到可以在这里重复一次
  //   （PX4 收到 NaN 位置的行为没有明确定义）。
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(yaw)) {
    return;
  }

  mavros_msgs::msg::PositionTarget msg;
  msg.header.stamp = node_.now();
  // ★ `frame_id` 沿用 `local_position` 的（没收到过时是空串）：mavros 对
  //   `setpoint_raw/local` **只读 `coordinate_frame`**，`frame_id` 不参与换算
  //   （`setpoint_raw.cpp` 的 `local_cb`）—— 与 ticket 05 的行为逐字一致，改它没有意义。
  msg.header.frame_id = local_position_.header.frame_id;
  msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = kTypeMaskPositionAndYaw;
  // ★ 坐标是 **ROS/ENU 口径**（z 向上为正），不是 NED —— mavros 会在 `local_cb` 里把它
  //   ENU→NED 再发给 PX4（`setpoint_raw.cpp:231`）。与 `/mavros/local_position/pose`
  //   （出向 NED→ENU，`local_position.cpp:157`）恰好互逆 ⟹ "目标 = 报回来的那个点"
  //   这句话在数值上成立。见头文件注释第 2 条。
  msg.position.x = x;
  msg.position.y = y;
  msg.position.z = z;
  msg.yaw = yaw;
  // ★ 票 15：external 档**故意不建**这条发布者；走到这里说明调用方分错了档。
  //   静默什么都不发（而不是解引用空指针）—— 但它本不该发生，故不加任何"补救"。
  if (setpoint_pub_) {
    setpoint_pub_->publish(msg);
  }
}

void Px4Interface::publish_control_target(double x, double y, double z,
  double vx, double vy, double vz, float yaw)
{
  // 同一个出口纪律：绝不把 NaN/Inf 交给外部控制器（它会把它们喂进 QP）。
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
    !std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz) || !std::isfinite(yaw))
  {
    return;
  }
  if (!control_target_pub_) {
    return;
  }

  mavros_msgs::msg::PositionTarget msg;
  msg.header.stamp = node_.now();
  // `frame_id` 与 setpoint 保持一致（`local_position` 的换算系）；内部话题的消费者按 ENU/FLU 读。
  msg.header.frame_id = local_position_.header.frame_id;
  // ⚠️ 枚举名有误导性（见文件头第 2 条）：值**仍是 ROS/ENU**，这里只是沿用同一套占位。
  msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = kTypeMaskPositionVelocityAndYaw;
  msg.position.x = x;
  msg.position.y = y;
  msg.position.z = z;
  msg.velocity.x = vx;
  msg.velocity.y = vy;
  msg.velocity.z = vz;
  msg.yaw = yaw;
  control_target_pub_->publish(msg);
}

void Px4Interface::publish_target(double x, double y, double z,
  double vx, double vy, double vz, float yaw)
{
  // 构造时**只建一条**发布者（见构造函数）⟹ 这里用它判断后端，不可能两路同时出帧。
  if (control_target_pub_) {
    publish_control_target(x, y, z, vx, vy, vz, yaw);
  } else {
    publish_setpoint(x, y, z, yaw);
  }
}

void Px4Interface::request_set_mode(const std::string & custom_mode, ResultCallback done)
{
  if (!set_mode_client_->service_is_ready()) {
    done(false, "服务不可用（" + topics_.set_mode_service + " 上没有 mavros 的 set_mode 服务）");
    return;
  }

  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->base_mode = 0;  // custom_mode 非空时 mavros 会自己补 CUSTOM_MODE_ENABLED / 解锁位
  req->custom_mode = custom_mode;

  set_mode_client_->async_send_request(
    req,
    [done, custom_mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      auto res = future.get();
      if (res->mode_sent) {
        // ⚠️ 只表示 SET_MODE 报文发出去了。真正的判据是稍后 `/mavros/state.mode` 变成期望值。
        done(true, "SET_MODE(" + custom_mode + ") 已发出；等待 /mavros/state.mode 确认");
      } else {
        done(false, "mavros 拒绝了这个 custom_mode（无法解析？）");
      }
    });
}

void Px4Interface::request_arming(bool arm, ResultCallback done)
{
  if (!arming_client_->service_is_ready()) {
    done(false, "服务不可用（" + topics_.arming_service + " 上没有 mavros 的 cmd/arming 服务）");
    return;
  }

  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = arm;

  // ⚠️ 回调里捕获 `this`：`Px4Interface` 是节点的成员、活到 `spin` 结束之后才析构
  //   （客户端的待处理回调不会在它之后触发）。
  arming_client_->async_send_request(
    req,
    [this, done, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      const auto res = future.get();
      const std::string what = arm ? "arming(true)" : "arming(false)";
      // ★★ `success` 与 `result` 是**两件事**，都要报出来：
      //   `success` = "命令送到了且没超时"，`result` = MAV_RESULT 码
      //   （`command.cpp:246`：`success = is_not_timeout && result == enum_value(ACCEPTED)`）。
      //   MAV_RESULT 是 mavros 唯一的"为什么被拒"载体 ⟹ 只写"被拒了"会让现场只能靠猜。
      //   ⚠️ 码值**按 MAVLink 定义数**，别凭印象：`ACCEPTED = 0` / `TEMPORARILY_REJECTED = 1`
      //     / `DENIED = 2` / `UNSUPPORTED = 3` / `FAILED = 4`
      //     （第一手：`/opt/ros/humble/include/mavlink/v2.0/common/common.h:1135-1136`）。
      //     ← 这里原本写的是"4 = TEMPORARILY_REJECTED"，**是错的**（4 是 FAILED）。
      // ★ 用 `MAV_RESULT=` 这个**固定形状**（与 `degrade=` / `traj_origin=` 同一惯例）：
      //   现场要的是"能 grep 到并读到码"，台架测试也断言这个令牌（见 test_control_takeoff.py）。
      if (res->success) {
        if (arm) {
          // ★ 只在这里置位：**肯定应答**才产生归属。真正的兑现要等 `armed` 的上升沿。
          arm_request_pending_ = true;
        }
        done(true, what + " 已被飞控接受（MAV_RESULT=" + std::to_string(res->result) +
          "）；等 /mavros/state.armed 确认");
      } else {
        if (arm) {
          // ⚠️ 被拒了就把 pending 清掉：留着它会让**之后**任何一次外部解锁被误认成我们的。
          arm_request_pending_ = false;
        }
        done(false, what + " 被飞控拒绝：MAV_RESULT=" + std::to_string(res->result));
      }
    });
}

}  // namespace control
