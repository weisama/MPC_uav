// `control` 的 mavros 边界 —— 所有话题/服务的收发都只在这里发生。
//
// ★ 本类的职责是"把 mavros 的话翻译成守卫"，**不做任何状态判断**：新鲜度（收到过没有、多久没收到）
//   在这里算，状态往哪走由 `fsm.hpp` 决定。这样"实飞才会想到"的判据（消息超时）与"状态语义"
//   各自只有一个家，改一个不会悄悄影响另一个。
//
// ★ 三个必须做对的字段（ticket 05 验收项，都有第一手依据）：
//   1. `type_mask = 2552` = 忽略速度/加速度/yaw_rate，保留**位置 + yaw**。
//      （2552 = 8+16+32 + 64+128+256 + 2048；不含 FORCE(512) 与 IGNORE_YAW(1024)。
//        PX4 v1.14.3 `mavlink_receiver.cpp::handle_message_set_position_target_local_ned`
//        逐位读这个掩码，忽略位对应的分量在 PX4 侧变成 NaN。）
//   2. `coordinate_frame = FRAME_LOCAL_NED (1)` —— ⚠️ **这个枚举名有误导性，别按字面理解**：
//      mavros 对入向的 `FRAME_LOCAL_NED` setpoint 做的是 **ENU→NED** 转换
//      （`setpoint_raw.cpp:231` `transform_frame_enu_ned(position)`），
//      所以**我们填进去的是 ROS/ENU 口径**（x 东、y 北、**z 向上为正**；yaw 从 +x 逆时针）。
//      实测换算（2026-09-21，按 `frame_tf.hpp:180` + `ftf_frame_conversions.cpp:33/40/86/90`
//      逐字复算）：`yaw_ned = π/2 − yaw_enu` ⟹ **这里填 `yaw = 0`，PX4 收到的是 +90°**。
//      ★ 也正因如此，把 `/mavros/local_position/pose`（mavros 出向已做过 NED→ENU，
//        `local_position.cpp:157`）**原样转回来是正确的** —— 位置与姿态恰好回到 PX4 自己报的值。
//   3. `header.stamp`：mavros 会把它换算成 MAVLink 的 `time_boot_ms`（**毫秒**，
//      `plugin.hpp:264-267`），而 **PX4 根本不读这个字段**（它自己用 `hrt_absolute_time()`
//      给 setpoint 打戳）。所以 spec 里"发往 PX4 的消息用微秒时间戳"那条**不适用于 Phase 1 的
//      mavros 路径**（那是 Phase 2 uXRCE-DDS 的约定）。这里的口径与 mavros 自带的 setpoint 插件
//      一致：`header.stamp = now()`。

#ifndef CONTROL__PX4_INTERFACE_HPP_
#define CONTROL__PX4_INTERFACE_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "control/fsm.hpp"

namespace control
{

/// 话题/服务名全部走参数（多机、台架复用都不用改代码）。
struct Px4Topics
{
  std::string state{"/mavros/state"};
  std::string local_position{"/mavros/local_position/pose"};
  std::string rc{"/mavros/rc/in"};
  std::string battery{"/mavros/battery"};
  std::string mocap{"/mocap/pose_enu"};
  std::string setpoint{"/mavros/setpoint_raw/local"};
  std::string set_mode_service{"/mavros/set_mode"};
  std::string arming_service{"/mavros/cmd/arming"};
};

/// 每条输入的独立超时。**逐条参数化是刻意的** —— 它们的上游频率差两个数量级：
///   · 动捕 ~196 Hz、`local_position` 实测 ~30 Hz ⟹ 0.5 s 是"明显漏了"的量级
///   · ⚠️ **`/mavros/state` 实测 1.000 Hz**（2026-09-21 本机读数；根源是 mavros
///     **只在收到 MAVLink HEARTBEAT 时**才发这个话题，而 HEARTBEAT 是 PX4 的 1 Hz 定频流
///     ⟹ mavros 侧改不了）。**照抄 `px4ctrl` 的 0.5 s 会让这条判据恒真、每秒都在报超时** ——
///     所以它的默认值是 2.5 s（2.5 个周期）。
struct Timeouts
{
  double state_s{2.5};      ///< ★ 见上：1 Hz 的上游，不能用 0.5 s
  double local_pos_s{0.5};
  double mocap_s{0.5};
  double rc_s{0.5};
  double battery_s{2.0};
};

class Px4Interface
{
public:
  using ResultCallback = std::function<void(bool ok, const std::string & detail)>;

  Px4Interface(rclcpp::Node & node, const Px4Topics & topics, const Timeouts & timeouts);

  /// 由守卫算出"当前哪些输入可信"。`armed` / `in_offboard` 直接取自最近一帧 `/mavros/state`
  /// （**未收到过该话题时按未解锁处理** —— 见 `mocap` 的同一条纪律：把"没收到"当成
  /// "已解锁"会在启动瞬间伪造出一个"异常解锁"）。
  Guards guards(double now_s) const;

  /// 一帧 `local_position` 都没有时返回 nullopt（STANDBY 的 setpoint 位置就取自它）。
  std::optional<geometry_msgs::msg::PoseStamped> local_position() const;

  /// 诊断用：某条输入距今多久（从未收到返回 `-1.0`）。`key` 取 `Px4Topics` 的成员。
  double age_seconds(const std::string & topic, double now_s) const;

  /// 发布唯一控制输出。`current` 是 EKF2 当前位姿（**ROS/ENU 口径**，见文件头第 2 条），
  /// 位置原样转发 ⟹ "原地保持"。`yaw` 也是 ENU 口径（⟹ `0` 表示机头朝 ENU 的 +x，不是"NED 的 0"）。
  void publish_setpoint(const geometry_msgs::msg::PoseStamped & current, float yaw);

  /// 请求切换飞行模式（`custom_mode`，如 `"OFFBOARD"`）。
  /// ⚠️ **`mode_sent == true` 只表示 SET_MODE 消息发出去了，不代表 PX4 接受** ——
  /// 真正的判据是稍后 `/mavros/state.mode` 变成期望值（`/mavros/state` 是 1 Hz ⟹ 最多晚 1 s）。
  /// 调用方必须按这个语义重试，不要以为回调返回 ok 就切成功了。
  void request_set_mode(const std::string & custom_mode, ResultCallback done);

  bool ever_received(const std::string & topic) const;

private:
  struct Input
  {
    rclcpp::Time last_rx{0, 0, RCL_ROS_TIME};
    bool ever{false};
    double timeout_s{0.0};
  };

  Input & slot(const std::string & topic);
  const Input & slot(const std::string & topic) const;
  bool fresh(const std::string & topic, double now_s) const;

  rclcpp::Node & node_;
  Px4Topics topics_;
  Timeouts timeouts_;

  std::map<std::string, Input> inputs_;

  mavros_msgs::msg::State state_;
  geometry_msgs::msg::PoseStamped local_position_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_sub_;

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  // 📌 解锁服务（`/mavros/cmd/arming`）本张**不建**：本张没有解锁路径（解锁 = ticket 08）。
  //    台架夹具**会**提供这个服务，好让 ticket 08 直接接上。
};

}  // namespace control

#endif  // CONTROL__PX4_INTERFACE_HPP_
