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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/estimator_status.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"

#include "control/fsm.hpp"
#include "control/rc_stick.hpp"
#include "control/rc_switch.hpp"

namespace control
{

/// 话题/服务名全部走参数（多机、台架复用都不用改代码）。
struct Px4Topics
{
  std::string state{"/mavros/state"};
  std::string local_position{"/mavros/local_position/pose"};
  /// ★ **速度**（ticket 08）：悬停稳定判据的 `|v|` 取自它，而不是拿动捕位姿差分出来
  /// 的"我们自己的速度" —— 后者带差分噪声，会把一个真在悬停的飞机判成"还在动"。
  /// ⚠️ 口径与 `local_position` **完全一致**：mavros 在**同一个回调**里从同一条
  /// `LOCAL_POSITION_NED` 发出这两个话题（`local_position.cpp:181` 与 `:210`），
  /// 位置走 `transform_frame_ned_enu(position)`、速度走
  /// `transform_frame_ned_enu(vx, vy, vz)` ⟹ **两者都是 ROS/ENU**，与 setpoint 同系。
  std::string local_velocity{"/mavros/local_position/velocity_local"};
  std::string rc{"/mavros/rc/in"};
  std::string battery{"/mavros/battery"};
  /// ★★ ticket 16：EKF2 的**估计器状态**（MAVLink `ESTIMATOR_STATUS`）。
  /// 用它是因为它是 ④「EKF2 位置失效」在本链条上**唯一真能读到的载体**：
  /// `pos_horiz_abs_status_flag` 直接来自 `estimator_status.solution_status_flags`
  /// （`mavlink/streams/ESTIMATOR_STATUS.hpp:88`，z0 固件 commit `4a0e65f2` 已核对），
  /// 而它在 PX4 侧**每次发布现算**（`EKF/ekf_helper.cpp:761`）⟹ **能自清**。
  /// ⚠️ 上游只有 **1 Hz**（2026-09-22 z0 实测 1.000 Hz）—— 与 `/mavros/state` 同量级，
  ///   所以它的超时不能照抄 0.5 s（见 `Timeouts::estimator_s`）。
  std::string estimator_status{"/mavros/estimator_status"};
  std::string mocap{"/mocap/pose_enu"};
  std::string setpoint{"/mavros/setpoint_raw/local"};
  /// ★ ticket 09：**别的包**的话题（不属于 mavros）。放在同一个结构体里是因为
  /// "本节点订阅/发布了哪些话题"必须只有一处 —— 诊断清单、QoS 表、README 都从它派生。
  /// `traj` 的三条接口（ticket 07 定稿）：参考（50 Hz，**相对轨迹原点**）、完成度（0–1）、
  /// 开始（`Empty`，收到就把参考时钟归零）。
  std::string traj_reference{"/traj/reference"};
  std::string traj_progress{"/traj/progress"};
  std::string traj_start{"/traj/start"};
  /// ★ ticket 09：**我们发**的完成度转发（`/control/traj_progress`）。
  /// ⚠️ 与 `/traj/progress` 同名不同域：这条是**我们**对外提供的观测面，值逐字转发
  ///   （见 `Px4Config::on_traj_progress`）。自己重算弧长等于把同一个数算两遍。
  std::string control_traj_progress{"/control/traj_progress"};
  /// ★★ 票 15：`output_mode=external` 时**我们对外发**的绝对目标（`PositionTarget`，ENU/FLU，
  ///   位置 + 速度 + yaw，50 Hz）。它**不是**给 PX4 的 —— 消费方是外部控制器
  ///   （`mpc_node` 的 `reference_source=control`，将来还会有别的后端）。
  /// ★ 口径：数值与 `/mavros/setpoint_raw/local` **完全同系**（ROS/ENU，z 向上为正，yaw 从 +x
  ///   逆时针），`coordinate_frame` 只是占位（与 `/traj/reference` 的既有做法逐字一致）。
  std::string control_target{"/control/target"};
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
  /// ★ ticket 16：`/mavros/estimator_status` 实测 **1.000 Hz**（2026-09-22 z0 实测），
  /// 取 2.5 s = 2.5 个周期（与 `state_s` 同一个理由、同一个数）。
  /// ⚠️ 它**不作为判据用**（没有"估计器状态静默"这条降级）—— 只用于诊断里报"这条输入多久没到"
  ///   与"敢不敢拿最后一帧读数说事"（见 `health()` 里那条新鲜度门）。
  ///   理由：这条话题静默时我们**什么也不知道**（既不能说 EKF2 坏了，也不能说它好），
  ///   而它上面的信息没有别的替代品 —— 拿它当判据只会多出一条"含义不明"的降级。
  double estimator_s{2.5};
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

  /// ★★ ticket 16：`/mavros/estimator_status.pos_horiz_abs` **连续为假**多少帧才算"EKF2 位置失效"。
  ///
  /// 取 **2**（用户 2026-09-22 裁定：连续 2 帧）的理由是**代价不对称**：
  ///   · 上游 1 Hz ⟹ 单帧为假可能是发布瞬间的状态（EKF2 在两次融合之间、`_fault_status` 抖了
  ///     一下），一帧就降级 = 每次抖动都停发 setpoint + 请求降落。
  ///   · 等 2 帧的代价 = 多 1 s 才认出失效，而这**不改变 ④ 的兜底能力**：
  ///     真正的兜底是动捕那两条（0.5 s），它们不依赖这个话题（见 `docs/.../§4.4`）。
  /// ⚠️ 换固件/改 `EKF2_EV_*` 之后这个数要重估 —— 它与上游频率强相关（1 Hz 时 2 帧 = 2 s）。
  /// ⚠️ 端到端延迟 = **上游两道**（EKF2 自己清 `cs.ev_pos`：样本停了是 400 ms，
  ///   `EKF/ev_control.cpp:75`；样本照到但没被融合是 1 s，`EKF/ev_pos_control.cpp:256`）
  ///   **+ 发布相位（1 Hz）+ 这里这两帧** ≈ 2.5–3 s。这两个上游阈值的出处见 `docs/.../§4.4`。
  /// ⚠️ **取值必须 ≥ 1**：判据是 `pos_lost_streak_ >= ekf2_pos_lost_samples`，而 streak 在读到**真**帧时
  ///   也是 0 ⟹ 填 0 或负数会让判据**恒成立**（一解锁就停发 + 请求降落，飞机就飞不了）。
  ///   **不做钳位**是刻意的：本仓所有数值参数都只在 `control.yaml` 受控、没有一处做运行时校验，
  ///   给这一个加钳位既不一致、又会让"参数写错了"这件事**在日志里静默消失**（`control.yaml` 是该值的唯一出处）。
  int ekf2_pos_lost_samples{2};

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

/// 遥控器**开始键**（ticket 09）—— §H.1 定案的那一组数（用户 2026-09-21 裁定）。
///
/// ★ 单开一个结构体而不是塞进 `Limits`：`Limits` 是**失效判据的阈值**（"什么时候停"），
///   这一组是**一个操作动作的判据**（"操作员按了没有"）。混在一起之后，调去抖会顺手改了低压阈值。
/// ★ 值全部来自 §H.1，**不要就地改**：通道号与阈值的裁定过程（含 §H.4 的 EdgeTX 分配、
///   §H.2 的"ch9 没有被任何 `RC_MAP_*` 占用"）都在 `docs/reference/params-changes.md`。
struct RcStart
{
  /// 开始键的通道号（**1-based**，与 PX4 `RC_MAP_*` 同一约定；⟹ `/mavros/rc/in` 的 `channels[8]`）。
  /// ⚠️ 通道数取决于接收机：不够时**跳过**（不判断、不抛异常）—— 与摇杆那条同样处理。
  int channel{9};
  /// 判"高位"的折算阈值（H.1：0.75 ⟹ 1750 µs）。
  double start_threshold{0.75};
  /// 判"低位"的折算阈值（H.1：0.25 ⟹ 1250 µs）。
  double release_threshold{0.25};
  /// 电平要**连续**保持多久才算数（H.1：200 ms）。
  double debounce_s{0.2};
  /// 折算到 0.0 / 1.0 的两个端点（µs）。★ 方向由这两个数说了算，不由名字里的 min/max
  /// 说了算 —— 换一个方向相反的接收机通道只要改这两个数（见 `rc_switch.hpp`）。
  double low_us{1000.0};
  double high_us{2000.0};
  /// **长按**判据：一条边沿之后电平持续为高这么久 ⟹ `take_start_long_press()` 给一条
  /// （ticket 07 增补；`<= 0` = 关掉这条功能）。⚠️ 计时从**电平被确认**那一刻起算
  /// ⟹ 现场"按 1 s"实际要按 ≈ 1.2 s（`debounce_s` 之后才开始计时）。
  double land_hold_s{1.0};
};

/// ★★ 票 15：谁把最终指令交给 PX4 —— 是**本节点自己**，还是**外部控制器**。
///
/// · `kPosition`（默认）：本节点发 `/mavros/setpoint_raw/local`（位置 + yaw）⟹ 老行为逐字不变。
/// · `kExternal`：本节点**不发任何 PX4 setpoint**，只把**绝对目标**（位置 + 速度 + yaw）
///   发到 `/control/target`；由订阅它的外部控制器（如 `mpc_node` 的 `reference_source=control`）
///   求解并发 `/mavros/setpoint_raw/attitude`。
///
/// ★ 为什么是"输出后端"而不是"任务分叉"：开始键、解锁、切 OFFBOARD、起飞、判稳、轨迹原点、
///   `/traj/start`、降落**全部留在本节点**（`fsm.hpp`）⟹ 加一种新控制方式只需要写一个
///   "订 `/control/target`、发执行器 setpoint"的节点，本文件与 `fsm.hpp` 都不用改。
/// ⚠️ 同一时刻**只有一条** PX4 setpoint 话题在被写：两档各自独占一条，绝不同时发。
enum class OutputMode
{
  kPosition,
  kExternal,
};

const char * to_string(OutputMode m);

/// 把参数串翻成 `OutputMode`；不认识时抛 `std::invalid_argument`（入口处一声响，
/// 而不是默默退回默认档 —— 那会让"我明明配了 external"变成现场最难查的一类静默失败）。
OutputMode parse_output_mode(const std::string & name);

/// `Px4Interface` 的**全部配置**。★ 打包成一个结构体而不是继续往构造函数上加参数：
/// 这些字段总是一起来（节点把参数读齐了一次性交给边界层），而位置参数一旦超过三四个，
/// "传错顺序"就变成编译期抓不到的静默 bug（字段全是 POD / 字符串）。
struct Px4Config
{
  Px4Topics topics;
  Timeouts timeouts;
  Limits limits;
  ModeNames modes;
  RcStart rc_start;
  /// ★★ 票 15：输出后端。默认 `kPosition` = 不改变任何现有行为。
  OutputMode output_mode{OutputMode::kPosition};
  /// `/traj/progress` 每来一帧调一次（**逐字转发**，不做任何加工）。
  /// ★ 为什么用回调而不是让节点自己再订阅一遍：**新鲜度台账只能有一个出处**
  ///   （`inputs_` 里的 `traj_progress` 也是这份订阅在刷）。两处订阅会出现
  ///   "状态机看的那个年龄"与"转发出去的那个值"来自两条不同队列的帧。
  /// ★ 名字带 `on_` 前缀是**故意**的：同一个标识符 `traj_progress` 在 `Px4Topics` 里是
  ///   **话题名**，在这里是**回调** —— 少了前缀，`config.traj_progress` 与
  ///   `config.topics.traj_progress` 读起来是同一个东西的两种写法（复核时被点出来）。
  std::function<void(float)> on_traj_progress;
  // ★ ticket 08 删掉了这里的 `own_arming` 参数：那条"缝"现在是**真的**
  //   （`request_arming()` 的应答 ⟹ `Guards::own_arming`），不再需要人工告知。
  //   ⚠️ 删参数不是洁癖：留着它，`own_arming: true` 会让**任何**人拨的解锁开关都被当成
  //   我们做的 ⟹ §H.6 要的"外部解锁告警"整条失效（那正是它存在的理由）。
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

  /// 一帧**可用**的 `local_position` 都没有时返回 nullopt（诊断用；位置本身见 `odometry()`）。
  /// ★ 出现 NaN/Inf 的那一帧算"不可用"：宁可不发 setpoint，也不把 NaN 转发给 PX4。
  std::optional<geometry_msgs::msg::PoseStamped> local_position() const;

  /// 本帧的**导航读数**（ticket 08）：位置、速度、机头，**全都是 ROS/ENU 口径**
  /// ⟹ 与 setpoint 同系，`fsm.hpp` 拿来直接算目标，**不需要任何坐标转换**。
  /// ★ "哪一路能信"的翻译只在这里（`position_valid` / `velocity_valid`）——
  ///   与 `guards()` 同一条纪律：`fsm.hpp` 只看布尔量，不碰 `std::isfinite`。
  Odometry odometry() const;

  /// 本帧的**轨迹参考**（ticket 09）：`/traj/reference` 的位置 + `/traj/progress` 的完成度，
  /// 外加"上一帧参考是多久以前到的"。
  /// ★ 口径：位置是**相对轨迹原点**的（ENU）⟹ `TRACK` 的目标 = 轨迹原点 + 它
  ///   （见 `fsm.hpp` 的 `TrajReference`）。
  /// ★ **只报年龄、不报"新鲜与否"**：多久算失效取决于"我们什么时候开始要流"，
  ///   那是任务层的事（`Health::traj_stale`）—— 在这里折成布尔量就再也拆不开了。
  TrajReference traj(double now_s) const;

  /// 取走遥控器开始键的**一条上升沿**（§H.5.1 的"第 1 次 = 起飞 / 第 2 次 = 开始轨迹"）。
  /// ★ 消费语义：一次按键只可能被取走一次（判据与理由见 `rc_switch.hpp`）。
  bool take_start_edge() {return start_.take_edge();}

  /// 取走**一条长按**事件（ticket 07 增补）：一次边沿之后，电平持续为高满
  /// `RcStart::land_hold_s` 的那一刻给出一次。消费语义同 `take_start_edge()`。
  bool take_start_long_press() {return start_.take_long_press();}

  /// 取走**一条松手**事件（确认低位那一刻）。★ 只有 `WAIT_START` 里"第 2 次按键"
  /// 需要它：那一次的动作现在发生在**松手**（见 `Fsm::note_start_release`）。
  bool take_start_release() {return start_.take_release();}

  /// 诊断用：开始键的判据现在怎么看这档开关（`high` / `released` / 边沿总数）。
  /// ★ 这三个是"按键没反应"这类现场问题的**唯一**分辨手段：是没收到帧、
  ///   还是没确认过低电平（H.5.2）、还是通道号配错了（`sample_valid` 为假）。
  const StartSwitchDetector & start_switch() const {return start_;}

  /// 发布 `/traj/start`（`Empty`）—— 让 `traj` 把参考时钟归零（`running_ = true`）。
  /// ★ 只有 `TRACK` 会用到它（`Decision::request_traj_start` 是脉冲）。
  /// ⚠️ 在"参考其实还在跑"的时候发它是一次**真实的指令跳变**（参考时钟归零 ⟹ 掉头飞回起点），
  ///   所以调用方必须守住闸门（见 `TaskLimits::traj_stream_gap_s`）。
  void publish_traj_start();

  /// 最近一帧 `/mavros/state` 报的模式（**一帧都没收到过时是空串**）。
  /// ★ 节点**不**再自己拿它比字符串（`in_offboard` / `in_land` 在 `guards()` 里算完）；这里留着是给诊断用
  /// （告警里写"模式从「OFFBOARD」变成了「POSCTL」"）。
  const std::string & mode() const {return state_.mode;}

  /// 最近一帧 `/mavros/state` 报的 `connected`（**一帧都没收到过时是 `false`**）。
  /// ★ 诊断用，且**必须能单独读到**：`kStateTimeout` 的判据是"新鲜 **且** `connected`"，
  ///   而 mavros 在链路超时后会发一帧"话题新鲜但 `connected=false`"的合成状态
  ///   （`sys_status.cpp:903-911`）⟹ 只写"静默 X s"的告警会把排查方向指错。
  bool state_connected() const {return state_.connected;}

  /// 最近一帧电池电压（V）；**没数据时返回 nullopt**（NaN、0 V 也算没数据，见回调里的说明）。
  /// 诊断用：告警里写"报电压 20.3 V"比只写"低压"有用得多。
  std::optional<double> battery_voltage() const {return battery_voltage_;}

  /// 诊断用：某条输入距今多久（从未收到返回 `-1.0`）。`key` 取 `Px4Topics` 的成员。
  double age_seconds(const std::string & topic, double now_s) const;

  /// 发布唯一控制输出。三个位置分量是 **ROS/ENU 口径**（见文件头第 2 条，z 向上为正），
  /// `yaw` 也是 ENU 口径（⟹ `0` 表示机头朝 ENU 的 +x，不是"NED 的 0"；见文件头第 2 条那条 ★）。
  /// ★★ ticket 08 起这是**目标**而不再是"当前位姿"：STANDBY 时两者恰好相等（原地保持），
  ///    起飞后目标由状态机给出（爬升斜坡 / 悬停点 / 轨迹原点）。
  ///    ⚠️ 目标由**调用方**（`fsm.hpp` 的 `Decision`）算好 —— 这里只管发，不做任何几何。
  void publish_setpoint(double x, double y, double z, float yaw);

  /// ★★ 票 15：把**绝对目标**发给外部控制器（`/control/target`，`output_mode=external`）。
  ///
  /// ★ 值与 `/mavros/setpoint_raw/local` **同系**：ROS/ENU（z 向上为正），yaw 从 +x 逆时针。
  ///   `coordinate_frame` 填 `FRAME_LOCAL_NED` 只是**占位**（与 `/traj/reference` 的既有做法
  ///   逐字一致）—— 内部话题的消费者按 ENU/FLU 读，不要按枚举名去换算。
  /// ★ 掩码 = 2496（忽略加速度三轴 + yaw_rate），⟹ 位置、速度、yaw **三路都有效**。
  ///   外部控制器拿到的是一条"位置 + 速度"的参考，正好是 MPC 需要的形状。
  /// ⚠️ 只在 `output_mode=external` 的链上调用；`position` 档不能碰它（否则会多出一个
  ///   没人消费的发布者，"飞机不动"时现场会先怀疑那个后端）。
  void publish_control_target(double x, double y, double z,
    double vx, double vy, double vz, float yaw);

  /// ★★ 票 15：按构造时选定的 `output_mode` 把目标发到**唯一**一条后端通路。
  ///
  /// 调用方（`control_node.cpp`）只表达"现在该有一个目标"，**不判断后端** ——
  /// 分支收在这里，新增后端时只动本文件，不会在状态机旁边再长出一个 switch。
  void publish_target(double x, double y, double z,
    double vx, double vy, double vz, float yaw);

  /// 请求切换飞行模式（`custom_mode`，如 `"OFFBOARD"`）。
  /// ⚠️ **`mode_sent == true` 只表示 SET_MODE 消息发出去了，不代表 PX4 接受** ——
  /// 真正的判据是稍后 `/mavros/state.mode` 变成期望值（`/mavros/state` 是 1 Hz ⟹ 最多晚 1 s）。
  /// 调用方必须按这个语义重试，不要以为回调返回 ok 就切成功了。
  void request_set_mode(const std::string & custom_mode, ResultCallback done);

  /// 请求解锁 / 上锁（`/mavros/cmd/arming`，`CommandBool`）。
  ///
  /// ★★ **本节点只用它来解锁**：上锁交给 PX4 自己的 `COM_DISARM_LAND = 2.0`（z0 实测值，
  ///    `docs/assets/z0.params:234`），我们只等 `armed` 的降沿（工单裁定）。
  /// ⚠️ 语义与 `request_set_mode` 一样是**异步**的：`done(true, …)` 只表示**飞控接受了这条命令**
  ///    （`success = is_not_timeout && result == MAV_RESULT::ACCEPTED`，见 `command.cpp:246`），
  ///    真正的判据是随后 `/mavros/state.armed`（1 Hz ⟹ 最多晚 1 s）。
  /// ★ 肯定应答会**记下**"这次解锁是我们发起的"，在之后观察到的第一个 `armed` 上升沿上
  ///    兑现成 `Guards::own_arming` —— 那是区分"我们解锁"与"别人解锁"（§H.6 的告警）的唯一依据。
  /// ⚠️ 本函数**不重试**（那是调用方的策略）：被拒就把 `arm_request_pending_` 清掉。
  void request_arming(bool arm, ResultCallback done);

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
  /// `/mavros/estimator_status` 一帧 —— 只做一件事：数 `pos_horiz_abs` **连续**为假的帧数
  /// （见 `Limits::ekf2_pos_lost_samples` 与 `health()`）。
  void on_estimator_status(const mavros_msgs::msg::EstimatorStatus & msg);
  /// `/mavros/state` 一帧 —— 顺带在 `armed` 的**上升沿**上兑现"这次解锁是谁发起的"。
  void on_state(const mavros_msgs::msg::State & msg);
  /// `/traj/reference` 一帧（ticket 09）—— 只记录，判据在 `fsm.hpp`。
  void on_traj_reference(const mavros_msgs::msg::PositionTarget & msg);
  /// `/traj/progress` 一帧（ticket 09）—— 记录 + **逐字转发**给配置里的回调。
  void on_traj_progress(const std_msgs::msg::Float32 & msg);

  rclcpp::Node & node_;
  Px4Topics topics_;
  Timeouts timeouts_;
  Limits limits_;
  ModeNames modes_;
  RcStart rc_start_;
  std::function<void(float)> traj_progress_cb_;

  // ── ticket 08：解锁的**归属**追踪（进 `Guards::own_arming`）──
  /// 解锁请求已被飞控接受、但**还没在 `/mavros/state` 里看到 `armed` 的上升沿**。
  /// ⚠️⚠️ 它**不能**在任何 `armed == false` 的帧上被清掉：`/mavros/state` 只有 1 Hz，
  ///   而服务应答是即时的 ⟹ 应答与上升沿之间**必然**夹着若干帧 `armed == false`，
  ///   清掉就会把**我们自己**的解锁误报成外部解锁（§H.6 的告警，现场最贵的那种误报）。
  ///   只在"被拒"（`request_arming` 的失败回调）或"上升沿已兑现"时清。
  bool arm_request_pending_{false};
  /// 已经兑现的归属：本次解锁是不是我们做的。`armed == false` 时恒为 false。
  bool arming_by_us_{false};

  std::map<std::string, Input> inputs_;

  mavros_msgs::msg::State state_;
  geometry_msgs::msg::PoseStamped local_position_;
  geometry_msgs::msg::TwistStamped local_velocity_;

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
  /// ticket 16：`pos_horiz_abs` **连续**为假的帧数（任何一帧为真立刻归零 —— 对称自清）。
  /// ★ 它数的是**样本**不是时间：上游 1 Hz ⟹ 2 帧 ≈ 2 s，改频率时这个含义跟着变（见 `Limits`）。
  int pos_lost_streak_{0};
  /// 连续假帧数够不够 `Limits::ekf2_pos_lost_samples`。
  /// ★ 它只回答"读数说没位置"，**不回答"这个读数还算不算数"** —— 后者是 `health()` 里那条
  ///   新鲜度门（话题静默时最后一帧的"真"会永远挂着，那是"我们不知道"，不是"我们知道它坏"）。
  bool ekf2_pos_lost_{false};

  // ── ticket 09：开始键与轨迹参考 ──
  /// 开始键的上升沿检测器（§H.1 的参数在 `RcStart`）。★ 在构造函数里用配置起，
  /// 因为它是**有状态**的（电平、去抖窗、已确认过的边沿），不能每帧现算。
  StartSwitchDetector start_;
  /// 最近一帧 `/traj/reference`（**相对轨迹原点**的 ENU 目标）。判据不看它，只当"最后的值"。
  mavros_msgs::msg::PositionTarget traj_reference_;
  /// 最近一帧 `/traj/progress` 的值（0–1；`traj` 跑完之后会一直保持 1.0）。
  /// ⚠️ 它**不会**因为流断了而自己归零 —— "跑完了"的判据必须配合闩锁，见 `fsm.hpp`。
  double traj_progress_{0.0};
  bool traj_progress_ever_{false};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr local_velocity_sub_;
  rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_sub_;
  rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr estimator_status_sub_;
  rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr traj_reference_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr traj_progress_sub_;

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  /// `/traj/start`（`Empty`）—— 我们不"发布状态"，只是让 `traj` 归零它的时钟。
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr traj_start_pub_;
  /// `/control/traj_progress`（ticket 09 的观测面）：逐字转发 `/traj/progress`。
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_traj_progress_pub_;
  /// ★★ 票 15：`/control/target` —— **只在 `output_mode=external` 时创建**。
  /// `position` 档下为 `nullptr`（话题面上也就没有这个发布者）。
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr control_target_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  /// 解锁服务（ticket 08 起真的建起来了；台架夹具 `fake_mavros` 早就提供了它）。
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
};

}  // namespace control

#endif  // CONTROL__PX4_INTERFACE_HPP_
