#include "control/px4_interface.hpp"

#include <cmath>
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
}  // namespace

Px4Interface::Px4Interface(rclcpp::Node & node, const Px4Config & config)
: node_(node), topics_(config.topics), timeouts_(config.timeouts), limits_(config.limits),
  modes_(config.modes), own_arming_(config.own_arming)
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
  inputs_[topics_.rc] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false, timeouts_.rc_s};
  inputs_[topics_.battery] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false, timeouts_.battery_s};
  inputs_[topics_.mocap] = {rclcpp::Time(0, 0, RCL_ROS_TIME), false, timeouts_.mocap_s};

  state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
    topics_.state, rclcpp::QoS(10).transient_local(),
    [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
      state_ = *msg;
      slot(topics_.state) = {node_.now(), true, timeouts_.state_s};
    });

  local_position_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    topics_.local_position, rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_local_position(*msg);});

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

  // 唯一的控制输出。发布方用默认（可靠）QoS：mavros 以 best_effort 订阅，可靠发布方可兼容；
  // 反过来（best_effort 发布）会让 mavros 收不到。
  setpoint_pub_ = node_.create_publisher<mavros_msgs::msg::PositionTarget>(topics_.setpoint, 10);
  set_mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>(topics_.set_mode_service);
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
  // ticket 06/08 之间的缝：本张没有解锁路径 ⟹ 由参数告知（ticket 08 改成解锁服务的结果）。
  g.own_arming = own_arming_;
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

void Px4Interface::publish_setpoint(
  const geometry_msgs::msg::PoseStamped & current, float yaw)
{
  // ★ 出口再挡一次 NaN：`local_position()` 已经会把 NaN 帧判成"不可用"，但"绝不把 NaN
  //   发给飞控"这条保证值钱到可以在这里重复一次（PX4 收到 NaN 位置的行为没有明确定义）。
  if (!pose_is_finite(current.pose)) {
    return;
  }

  mavros_msgs::msg::PositionTarget msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = current.header.frame_id;
  msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  msg.type_mask = kTypeMaskPositionAndYaw;
  // 「位置 = 当前」⟹ 原地保持。⚠️ 坐标是 **ROS/ENU 口径**（z 向上为正），不是 NED ——
  // mavros 会在 `local_cb` 里把它 ENU→NED 再发给 PX4（`setpoint_raw.cpp:231`）。
  // 与入参同源（`/mavros/local_position/pose` 出向已是 ENU，`local_position.cpp:157`）
  // ⟹ 这一进一出恰好抵消，PX4 收到的是它自己报的那个位置。见头文件注释第 2 条。
  msg.position = current.pose.position;
  msg.yaw = yaw;
  setpoint_pub_->publish(msg);
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

}  // namespace control
