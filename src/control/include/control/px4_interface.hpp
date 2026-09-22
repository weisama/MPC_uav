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

#include <array>
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
#include "control/rc_stick.hpp"

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

/// PX4 的**模式名**（`/mavros/state.mode` 与 `set_mode` 的 `custom_mode` 用的是同一套字符串）。
/// ★ 与 `Px4Topics` 分开：这些不是话题名，而是"模式叫什么"；但同样是换固件/多机时可能要改的口径。
/// ★ 放在这里的原因：`Guards::in_offboard` / `in_land` 是**从模式名翻译出来的**，
///   而"翻译只在边界层"是本包的一条纪律（见 README 的运行纪律）⟹ 节点不再自己比字符串。
struct ModeNames
{
  std::string offboard{"OFFBOARD"};   ///< 自主飞行模式
  std::string land{"AUTO.LAND"};      ///< 主动中止任务时请求的模式
};

/// 失效判据的**阈值**（ticket 06）。全部做成参数、全部在这里（`fsm.hpp` 只看布尔量）——
/// 每个默认值都必须有出处，理由逐条写在成员上。
struct Limits
{
  /// 低电压阈值（V）。⚠️ **这条判据用的是电压、不是电流** —— ticket 06 明确要求
  /// "不依赖电流读数"（[`01`](../../../.scratch/phase1/issues/01-hardware-prereqs.md) 的
  /// ADC 判定：电流通道有已知缺陷，虽然本机未中招）。
  ///
  /// 取 **20.0 V**，逐条出处：
  ///   · 电芯数 = **6S**：`BAT1_N_CELLS = 6` 已被 ticket 15 与飞控实测对账（6/14 ✅）；
  ///     ticket 01 记录的"未解锁 20.159 V / 剩余 16%"与 6S 自洽（3.36 V/cell；
  ///     若按 5S 则是 4.03 V/cell ≈ 满电，与飞控自报的 16% 矛盾）。
  ///   · 阈值对齐 **PX4 自己的低电阈值**（`BAT_LOW_THR = 0.15`，电压模型端点
  ///     `BAT1_V_CHARGED = 4.2 / BAT1_V_EMPTY = 3.2`）⟹ 3.2 + 0.15 ≈ 3.35 V/cell ≈ 20.1 V。
  ///     **必须我们来做这件事**：z0 的 `COM_LOW_BAT_ACT = 0` ⟹ PX4 只警告、不动作。
  ///   · 对照：开源参考实现 px4ctrl 的 XI35 6S 配置用 18.0 V（3.0 V/cell，竞速口径，偏损伤）。
  /// ★ **换电池必须改这里（4S/5S 都不合用）**：4S 满电才 16.8 V ⟹ 本阈值会**一直误报**；
  ///   5S 满电 21.0 V ⟹ 才掉到 4.0 V/cell 就会报低压（过早）。取小了则保护太晚。
  double low_voltage_v{20.0};

  /// 摇杆"在动"的变化率阈值（µs/s）。**必须与固件的 `COM_RC_STICK_OV` 同源**，
  /// 换算见 `rc_stick.hpp` 文件头（0.2/s ≈ 102 µs/s，按 z0 实测 `RC1_MIN/MAX`）。
  double rc_stick_ov_us_per_s{100.0};

  /// 动捕**时间戳**连续不前进多久算"冻住"（s）。动捕 ~196 Hz ⟹ 0.5 s 是 ~100 帧，
  /// 不是抖动而是真冻住（`vrpn_listener` 的 `stamp = now()`，夹具 `hold_stamp` 复现）。
  double mocap_stamp_stall_s{0.5};

  /// 四个摇杆通道号（**1-based**，与 PX4 `RC_MAP_ROLL/PITCH/THROTTLE/YAW` 同一约定）。
  int rc_map_roll{1};
  int rc_map_pitch{2};
  int rc_map_throttle{3};
  int rc_map_yaw{4};

  std::array<int, 4> rc_channels() const
  {
    return {rc_map_roll, rc_map_pitch, rc_map_throttle, rc_map_yaw};
  }
};

/// `Px4Interface` 的**全部配置**。★ 打包成一个结构体而不是继续往构造函数上加参数：
/// 这些字段总是一起来（节点把参数读齐了一次性交给边界层），而位置参数一旦超过三四个，
/// "传错顺序"就变成编译期抓不到的静默 bug（字段全是 POD / 字符串）。
struct Px4Config
{
  Px4Topics topics;
  Timeouts timeouts;
  Limits limits;
  ModeNames modes;
  /// 本节点发起的解锁（ticket 06/08 之间的缝，参数 `own_arming`）。
  /// ★ 语义：`Guards::own_arming` 直接取它；ticket 08 会把它从参数变成"解锁服务的结果"，
  ///   那时改的也只是这个字段的来源（`fsm.hpp` 一行不用动）。
  bool own_arming{false};
};

class Px4Interface
{
public:
  using ResultCallback = std::function<void(bool ok, const std::string & detail)>;

  Px4Interface(rclcpp::Node & node, const Px4Config & config);

  /// 由守卫算出"当前哪些输入可信"。★ **全部字段都在这里算完**，节点不再自己补任何一条
  /// （`in_offboard` / `in_land` 是模式名的**翻译**，`own_arming` 是解锁来源 —— 都不该由
  /// 调用方各写一遍：那样"状态机看见的守卫"与"日志说的守卫"会变成两份，实飞现场查不下去）。
  /// `armed` 取自最近一帧 `/mavros/state`（**未收到过该话题时按未解锁处理** —— 见 `mocap`
  /// 的同一条纪律：把"没收到"当成"已解锁"会在启动瞬间伪造出一个"异常解锁"）。
  Guards guards(double now_s) const;

  /// 失效判据（ticket 06）。**极性与 `Guards` 相反**（true = 出事了），所以分开两个函数名。
  /// 每一条的"没数据时是什么"都在实现里逐条写死 —— 全部是"不算失效"，除了
  /// `rc_sticks_moving`（它本身就要求有新鲜数据才成立）。
  Health health(double now_s) const;

  /// 一帧**可用**的 `local_position` 都没有时返回 nullopt（STANDBY 的 setpoint 位置就取自它）。
  /// ★ 出现 NaN/Inf 的那一帧算"不可用"：宁可不发 setpoint，也不把 NaN 转发给 PX4。
  std::optional<geometry_msgs::msg::PoseStamped> local_position() const;

  /// 最近一帧 `/mavros/state` 报的模式（**一帧都没收到过时是空串**）。
  /// 节点用它算 `Guards::in_land`（"当前是不是我们请求的那个降落模式"）。
  const std::string & mode() const {return state_.mode;}

  /// 最近一帧电池电压（V）；**没数据时返回 nullopt**（NaN、0 V 也算没数据，见回调里的说明）。
  /// 诊断用：告警里写"报电压 20.3 V"比只写"低压"有用得多。
  std::optional<double> battery_voltage() const {return battery_voltage_;}

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

  void on_rc(const mavros_msgs::msg::RCIn & msg);
  void on_mocap(const geometry_msgs::msg::PoseStamped & msg);
  void on_local_position(const geometry_msgs::msg::PoseStamped & msg);

  rclcpp::Node & node_;
  Px4Topics topics_;
  Timeouts timeouts_;
  Limits limits_;
  ModeNames modes_;
  /// 本节点发起的解锁 —— 进 `Guards::own_arming`（ticket 06/08 之间的缝）。
  bool own_arming_{false};

  std::map<std::string, Input> inputs_;

  mavros_msgs::msg::State state_;
  geometry_msgs::msg::PoseStamped local_position_;

  // ── 失效判据的**记忆**（判据本身在 `health()`，这里只存"最近一帧告诉我们什么"）──
  /// 四个摇杆通道的变化率检测器（顺序同 `Limits::rc_channels()`）。
  std::array<StickMoveDetector, 4> stick_{
    StickMoveDetector(), StickMoveDetector(), StickMoveDetector(), StickMoveDetector()};
  bool sticks_moving_{false};
  rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
  bool rc_timed_{false};
  /// ★ 只有**真的收到过**电池帧才可能为 true（"没收到"不是"低压"，见 `health()`）。
  bool battery_low_{false};
  /// 最近一帧可用的电压读数（诊断用；nullopt = 没有可信读数）。
  std::optional<double> battery_voltage_;
  /// `local_position` 出现过 NaN/Inf。下一帧**有限**的帧会清掉它。
  bool position_invalid_{false};
  /// 见过的**最大**动捕戳 —— 判据的基准，与 EKF2 收外部视觉样本的尺子一致
  /// （`EKF/estimator_interface.cpp:393` 比的是"已缓冲的最新样本"，见 `on_mocap`）。
  rclcpp::Time mocap_stamp_max_{0, 0, RCL_ROS_TIME};
  bool mocap_stamp_valid_{false};
  bool mocap_stall_counting_{false};
  rclcpp::Time mocap_stall_start_{0, 0, RCL_ROS_TIME};
  bool mocap_stalled_{false};

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
