// traj 节点：把四条解析轨迹按 50 Hz 采成**参考流**发出去。
//
// 职责边界（这条边界是刻意的，改之前先读 docs/control/phase1-implementation.md §2）：
//   本节点 **只发参考**：不发 setpoint、不解锁、不进 OFFBOARD、不碰飞控。
//   把参考加上轨迹原点、变成 `/mavros/setpoint_raw/local` 上的 setpoint，是 `control` 的事。
//   ⟹ 本节点可以在飞机没上电时单独起来，配合 RViz 离线核对轨迹形状与尺寸
//      （ticket 07 的验收判据之一，不需要动捕、不需要飞控、不需要解锁）。
//
// 话题：
//   ← `/traj/select`  (std_msgs/UInt8)   1 = 悬停 / 2 = 正方形 / 3 = 8 字 / 4 = 起飞 / **0 = 清空**
//   ← `/traj/start`   (std_msgs/Empty)   ★ **参考时钟的零点**
//   → `/traj/reference` (mavros_msgs/PositionTarget) 50 Hz，**相对轨迹原点**
//   → `/traj/progress`  (std_msgs/Float32)           完成度 0–1
//   → `/traj/marker`    (visualization_msgs/Marker)  RViz：整条折线 + 当前参考点
//
// ★★ 为什么「选」与「开始」是两条消息（这是本节点最容易被改错的地方）：
//   参考是按时间参数化的 —— 一旦开始，t 就在真实时间里前进。若把"选中"当作零点，
//   那么从选中到 control 真正进 TRACK 之间的那几秒，参考已经跑到轨迹中段了，
//   control 一进 TRACK 就要把飞机**猛地拽过去**（这正是 Phase 1 最怕的那种跳变）。
//   ⟹ 选轨迹只让节点"准备好并画出折线"，**参考流要等 `/traj/start` 才开**，
//      而那一条由 control 在 WAIT_START → TRACK 时发（第一手先例：Fast-Drone-XI35 的
//      px4ctrl 在 AUTO_HOVER 里发 `/traj_start_trigger` 允许规划器启动，
//      见 realflight_modules/px4ctrl/src/PX4CtrlFSM.cpp:202 与 :598-603）。
//
// ★ 跑完之后不停发、也不消失：参考停在轨迹终点、完成度保持 1.0（与先例 traj_server
//   "到点后原地悬停"一致），由 control 自己按 progress ≥ 1.0 走 TRACK → LAND（§4.2）。
//   本节点**不主动降落**：谁有控制权谁负责收尾。
//
// ⚠️ 坐标系口径（别记错）：参考是**相对轨迹原点**的，且是 ROS/ENU 口径（z 向上为正）。
//   轨迹原点 ≠ 动捕原点，它是 control 在 HOVER 稳定那一刻记下的 EKF2 位置（§3.3）。
//   ⟹ **绝不要把 `/traj/reference` 直接 remap 到 mavros 的 setpoint 上**：
//      那样位置会被当成 map 系里的绝对坐标，飞机会朝**动捕原点**附近去，而不是悬停点。

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "traj/trajectories.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace
{

/// `type_mask`（置 1 = 忽略）：位置 + 速度 + yaw 有效 ⟹ 只忽略加速度与 yaw_rate。
///   448 = IGNORE_AFX|IGNORE_AFY|IGNORE_AFZ = 64|128|256，2048 = IGNORE_YAW_RATE
///   ⟹ 448 + 2048 = **2496**。（含速度；只发位置+yaw 的话是 2552，见 §8.6。）
/// ★ 参考消息带速度**不等于**打开了速度前馈：§8.6 里"先不做"的是 **control → PX4** 那条
///   setpoint 上的前馈，那由 control 决定用哪个掩码。这里带速度是因为轨迹发生器本来就有
///   解析导数（限速判据也要它），而且 §8.6 说的"下一步调优"正好需要它 —— 备着，不用不花钱。
constexpr uint16_t kTypeMask = 2496;

/// marker 的两条 id（同一 ns 下）：整条轨迹的折线、当前的参考点。
constexpr int kMarkerPathId = 0;
constexpr int kMarkerPointId = 1;
constexpr char kMarkerNs[] = "traj";
/// 折线线宽 [m]：1 cm 在这个棚的尺度下看得清，又不会把飞机盖住。
constexpr double kPathLineWidthM = 0.01;
/// 参考点球半径 [m]：3 cm 左右，能看出"现在参考飞到哪了"。
constexpr double kPointDiameterM = 0.06;

}  // namespace

class TrajNode : public rclcpp::Node
{
public:
  TrajNode()
  : Node("traj")
  {
    // —— 话题名（全部可参数化：多机时换一份 yaml 即可）——
    reference_topic_ = declare_parameter<std::string>("reference_topic", "/traj/reference");
    progress_topic_ = declare_parameter<std::string>("progress_topic", "/traj/progress");
    select_topic_ = declare_parameter<std::string>("select_topic", "/traj/select");
    start_topic_ = declare_parameter<std::string>("start_topic", "/traj/start");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/traj/marker");

    // ★ 参考的 frame 是纯标注：mavros 的 setpoint_raw 只读 coordinate_frame 与 type_mask，
    //   从不读 frame_id（第一手：mavros/src/plugins/setpoint_raw.cpp:209-249）。
    reference_frame_id_ = declare_parameter<std::string>("reference_frame_id", "traj_origin");
    // ⚠️ marker 的 frame 必须是 RViz 认得的那个（有 TF 的），默认 map =
    //    /mavros/local_position/pose 的 frame。见 yaml 里的说明：形状与尺寸是真的，落点不是。
    marker_frame_id_ = declare_parameter<std::string>("marker_frame_id", "map");

    // —— 轨迹参数（原样交给纯函数层；参数到行为的映射只在 traj::make 一处）——
    params_.max_speed_ms = declare_parameter<double>("max_speed_ms", 0.5);
    params_.hover_duration_s = declare_parameter<double>("hover_duration_s", 60.0);
    params_.square_side_m = declare_parameter<double>("square_side_m", 1.0);
    params_.lemniscate_a_m = declare_parameter<double>("lemniscate_a_m", 1.0);
    params_.height_offset_m = declare_parameter<double>("height_offset_m", 0.0);
    // ★ 轨迹 4（起飞）的两个参数。⚠️ 它们是**轨迹参数**，不是"运行时可改的输入" ——
    //   要改就改 yaml / 重启本节点（`/traj/select 4` 只负责选它，不负责改它）。
    params_.takeoff_height_m = declare_parameter<double>("takeoff_height_m", 0.2);
    params_.takeoff_speed_ms = declare_parameter<double>("takeoff_speed_ms", 0.2);

    const double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    marker_period_s_ = declare_parameter<double>("marker_period_s", 1.0);
    marker_points_ = declare_parameter<int>("marker_points", 201);
    yaw_rad_ = declare_parameter<double>("yaw_rad", 0.0);
    const int startup_kind = declare_parameter<int>("startup_kind", 0);

    warn_about_parameters(rate_hz);

    // —— 接口 ——
    // QoS 用默认的 reliable + volatile：参考是"最新值就是全部"的流，不需要历史。
    ref_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(reference_topic_, rclcpp::QoS(10));
    progress_pub_ = create_publisher<std_msgs::msg::Float32>(progress_topic_, rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, rclcpp::QoS(10));

    select_sub_ = create_subscription<std_msgs::msg::UInt8>(
      select_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::UInt8::SharedPtr msg) {on_select(msg);});
    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      start_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Empty::SharedPtr msg) {on_start(msg);});

    // 定时器**一直**在跑，但 tick() 在"没选"或"选了没开始"时直接返回 ⟹ 一帧都不发。
    // 这样就不存在"定时器启停"与"时钟零点"两个状态机互相同步的问题。
    timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period_s_), [this]() {tick();});

    RCLCPP_INFO(
      get_logger(),
      "traj 节点启动：\n"
      "  发布 %s（%.1f Hz，**相对轨迹原点**、限速 %.2f m/s）\n"
      "  发布 %s（完成度 0–1）、%s（RViz）\n"
      "  订阅 %s（1/2/3/4 选轨迹，0 清空）、%s（开始）\n"
      "  ★ 本节点只发参考：不发 setpoint、不解锁、不进 OFFBOARD —— 那是 control 的事\n"
      "  ★⚠ 参考是**相对轨迹原点**的（轨迹原点 ≈ HOVER 稳定时的 EKF2 位置，≠ 动捕原点）。\n"
      "      千万不要把 %s 直接 remap 到 mavros 的 setpoint 上 —— 那会让飞机朝动捕原点附近去。\n"
      "  手敲命令（消遣/离线核对用）：\n"
      "    ros2 topic pub -1 %s std_msgs/msg/UInt8 \"{data: 2}\"\n"
      "    ros2 topic pub -1 %s std_msgs/msg/Empty \"{}\"",
      reference_topic_.c_str(), 1.0 / timer_period_s_, params_.max_speed_ms,
      progress_topic_.c_str(), marker_topic_.c_str(),
      select_topic_.c_str(), start_topic_.c_str(),
      reference_topic_.c_str(), select_topic_.c_str(), start_topic_.c_str());

    // ★ 启动即选：只用于**离线**在 RViz 里核对形状（不需要动捕/飞控/解锁）。
    //   飞的时候 yaml 里保持 0 —— 轨迹选择是操作者的动作，不该藏在参数里。
    //   注意它同样**不会**开始：参考流仍要等 /traj/start。
    traj::Kind kind{};
    if (traj::parse_kind(static_cast<unsigned char>(startup_kind), kind)) {
      select_kind(kind, /*by_parameter=*/true);
    } else if (startup_kind != 0) {
      RCLCPP_WARN(
        get_logger(), "startup_kind = %d 无效（只有 1/2/3/4 有意义，0 = 不自动选）—— 忽略",
        startup_kind);
    }
  }

private:
  // ——————————————————————————————————————————————————————————————
  // 参数自检：只告警，不改用户填的值（改了就成了"悄悄替人做决定"）
  // ——————————————————————————————————————————————————————————————

  void warn_about_parameters(double rate_hz)
  {
    // 定时器周期为 0 会抛异常 ⟹ 这一条必须兜住，其它只告警。
    timer_period_s_ = rate_hz > 0.0 ? 1.0 / rate_hz : 1.0;
    if (rate_hz <= 0.0) {
      RCLCPP_ERROR(get_logger(), "rate_hz = %.3f 非法（必须 > 0）—— 暂按 1.0 Hz 起定时器", rate_hz);
    }
    // §5.2 的硬要求是 ≤0.5 m/s。不拦，但必须让人看见。
    if (params_.max_speed_ms > kDesignMaxSpeedMs) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ max_speed_ms = %.2f 超过 §5.2 的设计值 %.2f m/s —— "
        "轨迹尺寸与动捕棚的捕捉范围是配套的，速度也决定了失控时的后果。确认是有意的再往下走。",
        params_.max_speed_ms, kDesignMaxSpeedMs);
    }
    if (params_.max_speed_ms <= 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "max_speed_ms = %.3f 非法（必须 > 0）—— 纯函数层会夹住它，但时长会变得没有物理意义。",
        params_.max_speed_ms);
    }
    // —— 轨迹 4（起飞）——
    // ★ 爬升速率也受 §5.2 那条限速的约束（它是一条"速度上限"，不限于平面轨迹）：
    //   不拦，但超了必须让人看见。
    if (params_.takeoff_speed_ms > params_.max_speed_ms) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ takeoff_speed_ms = %.2f 超过 max_speed_ms = %.2f —— 起飞的爬升速率也受"
        "§5.2 那条限速约束（见 traj.yaml 里 takeoff_speed_ms 的注释）。",
        params_.takeoff_speed_ms, params_.max_speed_ms);
    }
    if (params_.takeoff_speed_ms <= 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "takeoff_speed_ms = %.3f 非法（必须 > 0）—— 纯函数层会夹住它，但爬升时长会变得没有物理意义。",
        params_.takeoff_speed_ms);
    }
    if (params_.takeoff_height_m <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ takeoff_height_m = %.3f 非正 —— 轨迹 4（起飞）会**原地不动**（z 恒为该值）。"
        "要「起飞 20 cm」就该是 0.2（默认值）。",
        params_.takeoff_height_m);
    }
    if (params_.hover_duration_s <= 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "hover_duration_s = %.3f 非法（必须 > 0）—— 悬停的完成度全靠它，见 §5.3（默认 60 s）。",
        params_.hover_duration_s);
    }
    if (marker_points_ < 2) {
      RCLCPP_WARN(get_logger(), "marker_points = %d 太小 —— 按 2 处理", marker_points_);
      marker_points_ = 2;
    }
  }

  // ——————————————————————————————————————————————————————————————
  // 输入
  // ——————————————————————————————————————————————————————————————

  void on_select(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (msg->data == 0) {
      clear();
      return;
    }
    traj::Kind kind{};
    if (!traj::parse_kind(msg->data, kind)) {
      // ★ 无效值**保持当前轨迹不动**：突然清空比"没反应"更危险（操作者多半是敲错了）。
      RCLCPP_WARN(
        get_logger(), "忽略无效的 %s = %u（只有 1 = 悬停 / 2 = 正方形 / 3 = 8 字 / 4 = 起飞，"
        "0 = 清空）",
        select_topic_.c_str(), msg->data);
      return;
    }
    select_kind(kind, /*by_parameter=*/false);
  }

  void select_kind(traj::Kind kind, bool by_parameter)
  {
    trajectory_ = traj::make(kind, params_);
    if (!trajectory_) {  // 不可达：traj::make 覆盖了全部编号
      RCLCPP_ERROR(get_logger(), "内部错误：无法构造轨迹 %s", traj::to_string(kind));
      return;
    }
    // ★ 重新选择 = 回到"待开始"。否则在飞行中途换轨迹会让参考**瞬间跳到新轨迹的当前 t**
    //   （旧轨迹已经跑了 3 s，新轨迹也从 3 s 开始）—— 那是一次真实的指令跳变。
    running_ = false;
    elapsed_ = 0.0;
    logged_done_ = false;
    publish_path_marker(now());  // 折线立刻可看：满足"RViz 核对形状"的判据，不必等开始

    // ★ 报出去的"速度"要跟着轨迹走：起飞用的是 takeoff_speed_ms，不是平面轨迹的限速。
    const double shown_speed_ms =
      kind == traj::Kind::kTakeoff ? params_.takeoff_speed_ms : params_.max_speed_ms;
    RCLCPP_INFO(
      get_logger(),
      "%s %s：时长 %.2f s（速度 %.2f m/s）%s\n"
      "  ★ 现在**还没有开始**：参考流要等 %s 才发 —— 若选中的瞬间就开跑，"
      "control 进 TRACK 时参考已经到中段了，飞机会被猛地拽一下。",
      by_parameter ? "（来自 startup_kind 参数）已选" : "已选", traj::to_string(kind),
      trajectory_->duration(), shown_speed_ms, describe_shape(kind).c_str(),
      start_topic_.c_str());
  }

  void clear()
  {
    trajectory_.reset();
    running_ = false;
    elapsed_ = 0.0;
    logged_done_ = false;
    delete_markers();
    // 发一帧 0.0 再停：让下游明确看到"回到零了"，而不是靠"话题静默"去猜。
    // （control 的 TRACK → LAND 是 progress ≥ 1.0 触发的，0.0 不会误触发任何东西。）
    publish_progress(0.0);
    RCLCPP_INFO(
      get_logger(), "已清空轨迹：停发参考与完成度，marker 已删除（要再跑就重新 %s）",
      select_topic_.c_str());
  }

  void on_start(const std_msgs::msg::Empty::SharedPtr)
  {
    if (!trajectory_) {
      RCLCPP_WARN(
        get_logger(), "收到 %s 但**还没有选轨迹**（先发 %s = 1/2/3/4）—— 忽略",
        start_topic_.c_str(), select_topic_.c_str());
      return;
    }
    running_ = true;
    start_time_ = now();
    elapsed_ = 0.0;
    logged_done_ = false;
    RCLCPP_INFO(
      get_logger(), "★ 开始 %s：参考时钟零点 = 现在，时长 %.2f s（跑完停在终点，等 control 收尾）",
      traj::to_string(trajectory_->kind()), trajectory_->duration());
  }

  // ——————————————————————————————————————————————————————————————
  // 输出：定时器一响就按"真实流逝的时间"取参考
  // ——————————————————————————————————————————————————————————————

  void tick()
  {
    if (!trajectory_) {
      return;  // 没选轨迹 ⟹ 一帧都不发
    }
    const rclcpp::Time stamp = now();

    // ★★ 折线的周期重发**必须在 `!running_` 之前** —— 这是实测踩出来的：
    //   Marker 是 volatile 的（后起的 RViz 看不到之前发过的），所以"选中那一刻发一次"
    //   根本不够：晚 1 秒起来的 RViz、或者订阅刚匹配上的下游，都会永远看不到折线。
    //   而"选中但还没开始"恰恰是**离线核对形状**的用法（不需要动捕/飞控/起飞）
    //   ⟹ 重发必须与"开始没开始"无关，只与"选没选"有关（yaml 里 marker_period_s
    //   写的就是"整条轨迹折线的重发周期"，这里才是它的兑现处）。
    maybe_republish_path(stamp);

    if (!running_) {
      return;  // 选了但还没 /traj/start ⟹ 参考与完成度一帧都不发（此时 control 自己保持悬停）
    }

    // ★ 用真实流逝的时间而不是"定时器计数 × 周期"：定时器会抖动、会被调度延迟，
    //   而参考的时间轴必须与真实时间一致（control 侧的前馈是按真实时间算的）。
    elapsed_ = (stamp - start_time_).seconds();

    const traj::Reference ref = trajectory_->at(elapsed_);
    publish_reference(ref, stamp);
    publish_progress(ref.progress);
    publish_point_marker(ref, stamp);

    if (ref.progress >= 1.0 && !logged_done_) {
      logged_done_ = true;
      RCLCPP_INFO(
        get_logger(),
        "★ %s 已跑完（%.2f s）：参考停在终点、完成度保持 1.0 —— 现在等 control 按 §4.2 走 TRACK → LAND",
        traj::to_string(trajectory_->kind()), trajectory_->duration());
    }
  }

  void publish_reference(const traj::Reference & ref, const rclcpp::Time & stamp)
  {
    mavros_msgs::msg::PositionTarget msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = reference_frame_id_;  // 纯标注：mavros 不读 frame_id
    msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = kTypeMask;
    msg.position.x = ref.p.x;
    msg.position.y = ref.p.y;
    msg.position.z = ref.p.z;
    msg.velocity.x = ref.v.x;
    msg.velocity.y = ref.v.y;
    msg.velocity.z = ref.v.z;
    msg.yaw = static_cast<float>(yaw_rad_);
    // acceleration_or_force 与 yaw_rate 留 0：掩码里已置"忽略"，填了也不会被解释。
    ref_pub_->publish(msg);
  }

  /// 完成度（`std_msgs/Float32`）。**它没有 header** —— 所以这里没有 stamp 参数，
  /// 与参考/两条 marker 的签名不同是**故意的**（不摆一个用不上的参数装整齐）。
  void publish_progress(double progress)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(progress);
    progress_pub_->publish(msg);
  }

  // —— RViz 可视化 ——

  /// 整条轨迹的折线（几何采样，不含时间）。选中时发一次，之后每 marker_period_s 重发一次
  /// （Marker 是 volatile 的：后起的 RViz 看不到之前发过的，所以必须周期重发）。
  void publish_path_marker(const rclcpp::Time & stamp)
  {
    if (!trajectory_) {
      return;
    }
    const std::vector<traj::Vec3> points = trajectory_->path(marker_points_);
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = marker_frame_id_;
    m.ns = kMarkerNs;
    m.id = kMarkerPathId;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;  // 未旋转；RL 会把它归一化
    m.scale.x = kPathLineWidthM;
    m.color.r = 0.10f;
    m.color.g = 0.85f;
    m.color.b = 0.20f;
    m.color.a = 0.90f;
    m.points.reserve(points.size());
    for (const traj::Vec3 & p : points) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = p.z;
      m.points.push_back(gp);
    }
    marker_pub_->publish(m);
    last_path_time_ = stamp;
  }

  /// 当前参考点（一个球）。悬停时折线只有 1 个点、画不出东西 ⟹ 靠这个球看它在哪。
  void publish_point_marker(const traj::Reference & ref, const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = marker_frame_id_;
    m.ns = kMarkerNs;
    m.id = kMarkerPointId;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = ref.p.x;
    m.pose.position.y = ref.p.y;
    m.pose.position.z = ref.p.z;
    m.pose.orientation.w = 1.0;
    m.scale.x = kPointDiameterM;
    m.scale.y = kPointDiameterM;
    m.scale.z = kPointDiameterM;
    m.color.r = 0.95f;
    m.color.g = 0.45f;
    m.color.b = 0.00f;
    m.color.a = 1.00f;
    marker_pub_->publish(m);
  }

  void delete_markers()
  {
    const rclcpp::Time stamp = now();
    const int ids[] = {kMarkerPathId, kMarkerPointId};
    const int types[] = {
      visualization_msgs::msg::Marker::LINE_STRIP,
      visualization_msgs::msg::Marker::SPHERE};
    for (int i = 0; i < 2; ++i) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = marker_frame_id_;
      m.ns = kMarkerNs;
      m.id = ids[i];
      m.type = types[i];
      m.action = visualization_msgs::msg::Marker::DELETE;  // 只需 ns/id/action
      marker_pub_->publish(m);
    }
  }

  void maybe_republish_path(const rclcpp::Time & stamp)
  {
    if ((stamp - last_path_time_).seconds() >= marker_period_s_) {
      publish_path_marker(stamp);
    }
  }

  /// 日志里那半句"它的形状长什么样" —— 四条轨迹的尺寸差别不看一眼是记不住的。
  /// ★ 周长与时长**一律从轨迹对象自己算**（`duration() · max_speed`），不写死：
  ///   写死的意思是"改了 `lemniscate_a_m` 之后日志还在说 3.05 m"（Spec 审查指出过）。
  std::string describe_shape(traj::Kind kind) const
  {
    const double perimeter_m = trajectory_ ? trajectory_->duration() * params_.max_speed_ms : 0.0;
    switch (kind) {
      case traj::Kind::kHover:
        return "  形状：一个点（原地不动）；完成度按 hover_duration_s 走完，没有终点";
      case traj::Kind::kSquare:
        return "  形状：边长 " + num(params_.square_side_m) + " m 的正方形（周长 " +
               num(perimeter_m) + " m，闭合，跑完回到起点）";
      case traj::Kind::kLemniscate:
        return "  形状：8 字，宽 " + num(2.0 * params_.lemniscate_a_m) + " m、高 " +
               num(params_.lemniscate_a_m) + " m（★ y 幅度只有 x 的一半 = Gerono 双纽线天然偏扁）；周长 " +
               num(perimeter_m) + " m";
      case traj::Kind::kTakeoff:
        // ★ 这条**不能**套上面那个 perimeter_m（= 时长 × max_speed_ms）：起飞的时长沿用
        //   hover_duration_s、速度用 takeoff_speed_ms，两者都不是"周长 ÷ 限速"那个口径。
        return "  形状：从**轨迹原点（地面）**匀速直上 " + num(params_.takeoff_height_m) +
               " m（爬升 " + num(params_.takeoff_speed_ms) + " m/s ⟹ " +
               num(std::abs(params_.takeoff_height_m) / params_.takeoff_speed_ms) +
               " s 到顶，到顶后停在那儿）；★ 不参与 height_offset_m";
    }
    return {};
  }

  static std::string num(double v)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3f", v);
    return buf;
  }

  static constexpr double kDesignMaxSpeedMs = 0.5;  ///< §5.2 的设计值（超出就告警）

  // —— 配置 ——
  std::string reference_topic_;
  std::string progress_topic_;
  std::string select_topic_;
  std::string start_topic_;
  std::string marker_topic_;
  std::string reference_frame_id_;
  std::string marker_frame_id_;
  traj::Params params_{};
  double yaw_rad_{0.0};
  double marker_period_s_{1.0};
  double timer_period_s_{0.02};
  int marker_points_{201};

  // —— 状态 ——
  std::unique_ptr<traj::Trajectory> trajectory_;  ///< 空 = 没选轨迹（一帧都不发）
  bool running_{false};                           ///< false = 选了但还没 /traj/start
  bool logged_done_{false};
  double elapsed_{0.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_path_time_{0, 0, RCL_ROS_TIME};

  // —— 接口 ——
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr ref_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr select_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajNode>());
  rclcpp::shutdown();
  return 0;
}
