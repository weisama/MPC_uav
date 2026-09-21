#include "control/px4_interface.hpp"

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
}  // namespace

Px4Interface::Px4Interface(
  rclcpp::Node & node, const Px4Topics & topics, const Timeouts & timeouts)
: node_(node), topics_(topics), timeouts_(timeouts)
{
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
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      local_position_ = *msg;
      slot(topics_.local_position) = {node_.now(), true, timeouts_.local_pos_s};
    });

  rc_sub_ = node_.create_subscription<mavros_msgs::msg::RCIn>(
    topics_.rc, rclcpp::QoS(10),
    [this](mavros_msgs::msg::RCIn::ConstSharedPtr) {
      slot(topics_.rc) = {node_.now(), true, timeouts_.rc_s};
    });

  battery_sub_ = node_.create_subscription<sensor_msgs::msg::BatteryState>(
    topics_.battery, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::BatteryState::ConstSharedPtr) {
      slot(topics_.battery) = {node_.now(), true, timeouts_.battery_s};
    });

  mocap_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    topics_.mocap, rclcpp::QoS(10),
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr) {
      slot(topics_.mocap) = {node_.now(), true, timeouts_.mocap_s};
    });

  // 唯一的控制输出。发布方用默认（可靠）QoS：mavros 以 best_effort 订阅，可靠发布方可兼容；
  // 反过来（best_effort 发布）会让 mavros 收不到。
  setpoint_pub_ = node_.create_publisher<mavros_msgs::msg::PositionTarget>(topics_.setpoint, 10);
  set_mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>(topics_.set_mode_service);
}

Px4Interface::Input & Px4Interface::slot(const std::string & topic)
{
  return inputs_.at(topic);
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
  g.in_offboard = (state_.mode == "OFFBOARD");
  return g;
}

std::optional<geometry_msgs::msg::PoseStamped> Px4Interface::local_position() const
{
  if (!ever_received(topics_.local_position)) {
    return std::nullopt;
  }
  return local_position_;
}

void Px4Interface::publish_setpoint(
  const geometry_msgs::msg::PoseStamped & current, float yaw)
{
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
