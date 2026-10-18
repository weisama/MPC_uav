// mocap 节点：动捕刚体位姿 → **相对位姿**（两阶段原点 + 仅归零 yaw）→ 喂给 EKF2。
//
// 数据流（全部为**动捕原生 ENU / 机体 FLU**，本节点不做任何坐标系转换）：
//   /vrpn/<刚体>/pose  ──▶ 本节点 ──▶ /mavros/vision_pose/pose   （mavros 负责 ENU→NED、FLU→FRD）
//                                  └─▶ /mocap/pose_enu           （Phase 1 只用于记录；Phase 2 的 MPC 状态源）
//   /vrpn/<刚体>/twist ──▶ 本节点 ──▶ /mocap/velocity_enu         （Phase 2 的 MPC 速度来源，见 .scratch/phase2/issues/15）
//
// ★ 速度是**原样 republish**（`TwistStamped`，frame 与 stamp 都从 `msg->header` 带走）：
//   `vrpn_listener` 的 twist 与本节点转发给 MPC 的 pose **同一个 frame**（都是 `"world"`）
//   ⟹ 同 frame 直接转发就是**零变换**，与「输入侧不做坐标转换」那条约定一致。
//
// ★ 四条不许碰的约定（改任何一条都会毁掉 EKF2 融合，见 docs/control/phase1-implementation.md）：
//   1. **stamp 原样转发** —— 不二次打戳（`vrpn_listener` 的 `stamp = now()` 是刻意设计）。
//      理由有二：时间戳不前进的消息会被 mavros 静默丢弃（§3.5）；动捕主机与本机有 ~4 s 时钟差，
//      换成 VRPN 的 msg_time 会直接毁掉融合。
//   2. **不做坐标转换** —— 位置 ENU、姿态 FLU 原样交给 mavros 转。
//      （下面的外参平移**不算**转换：同一个刚体上两点的平移，输入输出仍是 ENU/FLU。）
//   3. **姿态只归零 yaw** —— roll/pitch 保持绝对（理由见 mocap_rel_pose.hpp 的 yaw_zeroed）。
//   4. **外参成对** —— 本节点做了刚体原点→IMU 的平移 ⟹ 飞控 `EKF2_EV_POS_X/Y/Z` 保持 `0/0/0`（见下）。
//
// ★ 动捕外参（2026-10-09 加）：本节点在**读原始数据这一层**把位姿从「动捕刚体原点」平移到
//   「飞控 IMU 中心」（参数 `body_to_imu_x/y/z`，机体系 FLU），**用该帧姿态旋转**施加。
//   ① 施加点在 `on_pose` 的**最前面**（`to_pod` 之后、`selector_->observe` 之前）⟹ 两阶段原点
//      也采样在 IMU 点上，输出的相对位姿里**不含**这个常值偏移；
//   ② ★ 成对纪律：**本节点做了平移 ⟹ 飞控上 `EKF2_EV_POS_X/Y/Z` 必须保持 `0/0/0`**，
//      两边都做 = **双计**（错一倍），而且两边都"看起来对"；
//   ③ ⚠️ 没做平移的那一半：`/mocap/velocity_enu` 仍是**刚体原点**的速度（原样转发）。
//      两者的差是 `ω × d_机体系`，其中竖直偏移只与 roll/pitch 角速率耦合（≈ 0.08·ω_xy）。
//      影响面有限：EKF2 **不融合** EV 速度（`EKF2_EV_CTRL = 11`，bit2 关），只有 Phase 2 的
//      MPC 速度输入会带上它；且动捕 twist 的 angular 实测恒为 0 ⟹ 想补偿也补不了。见 README。
//
// ★ 运行纪律：**从本节点启动到解锁之间，不要把飞机搬来搬去、不要转它。**
//   （两阶段原点切换的跳变量 = 这段时间里飞机被移动的量。朝向不重要，静止才重要。）

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mocap/mocap_rel_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace
{

mocap::Pose to_pod(const geometry_msgs::msg::Pose & p)
{
  return mocap::Pose{
    mocap::Vec3{p.position.x, p.position.y, p.position.z},
    mocap::Quat{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}};
}

bool ends_with(const std::string & s, const std::string & suffix)
{
  return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string to_lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

}  // namespace

class MocapRelPoseNode : public rclcpp::Node
{
public:
  MocapRelPoseNode()
  : Node("mocap_rel_pose")
  {
    // —— 输入 ——
    // ★ 刚体名是「变量」不是「常量」：2026-09-21 一天之内实测到 `Z0`（大写）与 `z0`（小写）两个值
    //   （Nokov 侧重命名一次就变，话题名原样取自 VRPN 的 sender 名，只把空格换成 `_`）。
    //   ⟹ 不要把这个名字当常量用；**每次配置前现场核对**：ros2 topic list | grep vrpn
    body_name_ = declare_parameter<std::string>("body_name", "z0");
    input_topic_ = declare_parameter<std::string>("input_topic", "");
    if (input_topic_.empty()) {
      input_topic_ = "/vrpn/" + body_name_ + "/pose";
    }
    // 速度输入：`vrpn_listener` 对每个 sender 同时发 /pose、/twist、/accel
    // （`vrpn_listener.cpp:139-144`），话题名规则与 pose 完全相同（刚体名 + `/{pose,twist}`）。
    twist_topic_ = declare_parameter<std::string>("twist_topic", "");
    if (twist_topic_.empty()) {
      twist_topic_ = "/vrpn/" + body_name_ + "/twist";
    }
    vision_pose_topic_ = declare_parameter<std::string>(
      "vision_pose_topic", "/mavros/vision_pose/pose");
    pose_enu_topic_ = declare_parameter<std::string>("pose_enu_topic", "/mocap/pose_enu");
    velocity_enu_topic_ = declare_parameter<std::string>(
      "velocity_enu_topic", "/mocap/velocity_enu");
    state_topic_ = declare_parameter<std::string>("state_topic", "/mavros/state");

    // —— 动捕外参：刚体原点 → IMU 中心（机体系 FLU：x 前 / y 左 / z 上，m）——
    // ★ IMU 在刚体原点**下方** ⟹ z 分量为**负**（默认 −0.080 = 80 mm，2026-10-09 操作者给定）。
    // ⚠️ 这个量**每场次/每装法都要复核**：2026-09-21 与 2026-09-29 两次静止读数差 20.2 mm。
    // ⚠️ 与本参数成对：本节点施加了平移 ⟹ 飞控 `EKF2_EV_POS_X/Y/Z` 必须保持 0/0/0（双计 = 错一倍）。
    r_imu_in_body_ = mocap::Vec3{
      declare_parameter<double>("body_to_imu_x", 0.0),
      declare_parameter<double>("body_to_imu_y", 0.0),
      declare_parameter<double>("body_to_imu_z", -0.080)};

    // —— 两阶段原点的采样帧数（动捕 ~200 Hz ⟹ 30 帧 ≈ 0.15 s，60 帧 ≈ 0.3 s）——
    boot_samples_ = declare_parameter<int>("boot_samples", 30);
    const int arm_samples = declare_parameter<int>("arm_samples", 60);

    // 两阶段原点切换量超过这个值就告警：说明"启动到解锁之间飞机被移动了"，纪律没守住。
    jump_warn_m_ = declare_parameter<double>("jump_warn_m", 0.05);
    const double no_data_warn_s = declare_parameter<double>("no_data_warn_s", 5.0);

    selector_ = std::make_unique<mocap::TwoStageOrigin>(boot_samples_, arm_samples);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&MocapRelPoseNode::on_pose, this, std::placeholders::_1));
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_, rclcpp::QoS(10),
      std::bind(&MocapRelPoseNode::on_twist, this, std::placeholders::_1));
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      state_topic_, rclcpp::QoS(10),
      std::bind(&MocapRelPoseNode::on_state, this, std::placeholders::_1));

    vision_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      vision_pose_topic_, rclcpp::QoS(10));
    pose_enu_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_enu_topic_, rclcpp::QoS(10));
    velocity_enu_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      velocity_enu_topic_, rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "mocap 节点启动：订阅 %s、%s（body_name = \"%s\"）与 %s；发布 %s、%s、%s\n"
      "  两阶段原点：启动采样 %d 帧 → 解锁上升沿后采样 %d 帧；切换量 > %.3f m 时告警\n"
      "  ★ 纪律：从本节点启动到解锁之间，不要把飞机搬来搬去、不要转它（朝向不重要，静止才重要）",
      input_topic_.c_str(), twist_topic_.c_str(), body_name_.c_str(), state_topic_.c_str(),
      vision_pose_topic_.c_str(), pose_enu_topic_.c_str(), velocity_enu_topic_.c_str(),
      boot_samples_, arm_samples, jump_warn_m_);

    // ★ 外参必须**每次启动都打在日志里** —— 它是静默的（填 0 / 填错 / 符号反，静止与水平飞行都看不出来），
    //   现场唯一能核对的就是这一行。同时把「飞控那边必须保持 0」这条成对纪律一起打出来。
    const bool extrinsic_active = mocap::norm(r_imu_in_body_) > 1e-9;
    RCLCPP_INFO(
      get_logger(),
      "  动捕外参（刚体原点 → IMU 中心，机体系 FLU）= (%.4f, %.4f, %.4f) m —— %s\n"
      "    · 施加方式：p_IMU = p_刚体 + R(该帧姿态)·外参（倾斜时 IMU 不在刚体原点正下方，"
      "别改成固定减 z）\n"
      "    · ⚠️ 每场次/每装法都要复核（两次静止读数曾差 20.2 mm）："
      "ros2 param get /mocap_rel_pose body_to_imu_z\n"
      "    · ★ 成对纪律：本节点做了平移 ⟹ 飞控 `EKF2_EV_POS_X/Y/Z` 必须保持 0/0/0（两边都做 = 双计）",
      r_imu_in_body_.x, r_imu_in_body_.y, r_imu_in_body_.z,
      extrinsic_active ? "已施加" : "⚠️ 全 0 ⟹ 相当于没做外参（此时飞控侧才需要填 EKF2_EV_POS_*）");

    // 动捕数据看门狗（每 no_data_warn_s/5 查一次，≥0.2 s ⟹ 检出滞后远小于阈值本身）：
    //   · 启动后一直没数据 → ERROR，并把 graph 里实际存在的 /vrpn/*/pose 列出来供"现场核对"
    //     （名字对不上、大小写不同、vrpn_listener 没起，三种都能一眼看出）
    //   · 运行中数据中断 → ERROR（★ 每次中断只报一次，不刷屏），恢复时再报一次
    no_data_warn_s_ = no_data_warn_s;
    last_pose_time_ = now();  // 起点：本节点启动时刻
    watchdog_ = create_wall_timer(
      std::chrono::duration<double>(std::max(0.2, no_data_warn_s / 5.0)),
      [this]() {check_mocap_health();});
  }

private:
  /// 解锁状态的三态：**没收到过 state 与"收到且未解锁"是两件事** —— 前者若被当成"已解锁"，
  /// 启动瞬间就会伪造出一个解锁上升沿（用错误的起点锁"起飞前原点"）。见 mocap_rel_pose.hpp 的说明。
  mocap::ArmedState armed_state() const
  {
    if (!has_state_) {
      return mocap::ArmedState::Unknown;
    }
    return armed_ ? mocap::ArmedState::Armed : mocap::ArmedState::Disarmed;
  }

  void on_state(const mavros_msgs::msg::State::SharedPtr msg)
  {
    has_state_ = true;
    armed_ = msg->armed;
  }

  /// 动捕速度（ENU，m/s）→ **原样 republish** 成 `/mocap/velocity_enu`（`TwistStamped`）。
  ///
  /// ★ 三点与 pose 路径**刻意不同**，理由都在这：
  ///   1. **零变换**：`vrpn_listener` 的 twist 与 pose 是**同一个 frame**（`"world"`，
  ///      `vrpn_listener.cpp:61` 的 `listener->_frame_id`）⟹ 同 frame 直接转发即可，
  ///      与「输入侧不做坐标转换」那条约定一致。`frame_id` 与 `stamp` 都从 `msg->header`
  ///      **原样带走**（不二次打戳 —— 同 pose 的「约定 1」，理由见文首）。
  ///   2. **不经过两阶段原点选择器**：速度与原点无关（原点只平移位置）⟹ 不需要
  ///      `selector_->ready()` 那道闸门。位置在原点锁定前发出去会喂错数据，速度不会。
  ///   3. **不参与动捕看门狗**：看门狗判的是「位置这条链还在不在」（`last_pose_time_`）；
  ///      twist 与 pose 由**同一台** `vrpn_listener` 发出，位置断了 twist 也就断了。
  void on_twist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (!received_twist_) {
      received_twist_ = true;
      RCLCPP_INFO(
        get_logger(), "收到第一帧 %s（frame_id = \"%s\"）；开始转发到 %s（★ 原样，零变换）",
        twist_topic_.c_str(), msg->header.frame_id.c_str(), velocity_enu_topic_.c_str());
    }
    geometry_msgs::msg::TwistStamped out;
    out.header = msg->header;   // ★ 原样转发 frame_id 与 stamp（零变换 / 不二次打戳）
    out.twist = msg->twist;     // ★ 三分量原样带走（angular 在 vrpn_listener 侧刻意留零）
    velocity_enu_pub_->publish(out);
  }

  /// 动捕数据看门狗：数据停超过 no_data_warn_s 就 ERROR，恢复时报一声。
  /// ★ 与"解锁边沿"无关，纯看数据到没到 —— 定位失效时飞控那边（EKF2）看到的是"没有新的
  ///   vision_pose"，操作者看到的必须是**明确的一句告警**，而不是"日志里什么都没有"。
  void check_mocap_health()
  {
    const double quiet_s = (now() - last_pose_time_).seconds();
    if (quiet_s < no_data_warn_s_) {
      return;
    }
    if (pose_count_ == 0) {
      if (!logged_no_data_) {
        logged_no_data_ = true;
        RCLCPP_ERROR(
          get_logger(), "启动 %.1f s 内 %s 一帧数据都没有。%s",
          no_data_warn_s_, input_topic_.c_str(), describe_vrpn_candidates().c_str());
      }
      return;
    }
    if (!logged_stale_) {
      logged_stale_ = true;
      RCLCPP_ERROR(
        get_logger(),
        "★ 动捕数据已中断 %.1f s（共收到 %zu 帧，最后一帧在本机时刻 %.3f）"
        " —— EKF2 的视觉输入已失效，**不要再起飞 / 应立即切回手动**。检查 vrpn_listener 与动捕主机。",
        quiet_s, pose_count_, last_pose_time_.seconds());
    }
  }

  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    ++pose_count_;
    if (logged_stale_) {
      logged_stale_ = false;
      RCLCPP_WARN(
        get_logger(), "动捕数据已恢复（中断了约 %.1f s）—— 期间 EKF2 没有视觉输入。",
        (now() - last_pose_time_).seconds());
    }
    last_pose_time_ = now();
    const mocap::Pose raw = to_pod(msg->pose);
    // ★ 外参在这一步施加：**在原点采样之前**，⟹ 两阶段原点与后续每一帧都在同一个点上，输出里不含常值偏移。
    const mocap::Pose in = mocap::rigid_body_to_imu(raw, r_imu_in_body_);
    selector_->observe(in, armed_state());

    if (pose_count_ == 1) {
      RCLCPP_INFO(
        get_logger(),
        "收到第一帧 %s：刚体原点 (%.4f, %.4f, %.4f) → IMU 点 (%.4f, %.4f, %.4f)，frame_id = \"%s\"",
        input_topic_.c_str(), raw.p.x, raw.p.y, raw.p.z, in.p.x, in.p.y, in.p.z,
        msg->header.frame_id.c_str());
    }

    if (!selector_->ready()) {
      return;  // 启动原点还没锁好，一帧都不发（拿默认原点糊出去 = 给 EKF2 喂错数据）
    }

    if (!logged_boot_lock_) {
      logged_boot_lock_ = true;
      RCLCPP_INFO(
        get_logger(), "★ 阶段 A：启动原点已锁定（%d 帧均值）p_boot = (%.4f, %.4f, %.4f)；姿态保持绝对",
        boot_samples_, selector_->origin().p.x, selector_->origin().p.y, selector_->origin().p.z);
    }

    if (selector_->switched() && !logged_switch_) {
      logged_switch_ = true;
      log_arm_lock(msg->header.stamp);
    }

    const mocap::Pose out = selector_->apply(in);
    publish(vision_pose_pub_, msg->header, out);
    publish(pose_enu_pub_, msg->header, out);
  }

  void log_arm_lock(const rclcpp::Time & stamp)
  {
    const mocap::Origin & boot = selector_->boot_origin();
    const mocap::Origin & arm = selector_->origin();
    RCLCPP_INFO(
      get_logger(),
      "★ 阶段 B：解锁上升沿后的起飞前原点已锁定 —— 切换发生在此刻（帧 stamp = %.9f，"
      "本机 %.9f）\n"
      "  p_arm   = (%.4f, %.4f, %.4f)   p_boot = (%.4f, %.4f, %.4f)\n"
      "  归零的 yaw = %.2f°；切换瞬间位置跳变 = %.4f m；姿态起 roll/pitch 保持绝对",
      stamp.seconds(), now().seconds(),
      arm.p.x, arm.p.y, arm.p.z, boot.p.x, boot.p.y, boot.p.z,
      arm.yaw * 180.0 / M_PI, selector_->switch_jump());
    if (selector_->switch_jump() > jump_warn_m_) {
      RCLCPP_WARN(
        get_logger(),
        "⚠️ 两阶段原点切换量 %.4f m 超过阈值 %.4f m —— 说明「启动到解锁之间飞机被移动/转动了」。"
        "本次飞行位置会有一个同量的跳变（EKF2 会把它当成真实运动）。"
        "下次请严格遵守纪律：摆好后到解锁前别动它。",
        selector_->switch_jump(), jump_warn_m_);
    }
  }

  void publish(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub,
    const std_msgs::msg::Header & header, const mocap::Pose & pose) const
  {
    geometry_msgs::msg::PoseStamped out;
    out.header = header;  // ★ 原样转发 vrpn_listener 的 stamp 与 frame_id，不二次打戳
    out.pose.position.x = pose.p.x;
    out.pose.position.y = pose.p.y;
    out.pose.position.z = pose.p.z;
    out.pose.orientation.x = pose.q.x;
    out.pose.orientation.y = pose.q.y;
    out.pose.orientation.z = pose.q.z;
    out.pose.orientation.w = pose.q.w;
    pub->publish(out);
  }

  /// 列出 graph 里**有发布者**的 /vrpn/*/pose，并标出与 body_name 大小写不同的那些。
  /// ★ 必须按 count_publishers 过滤：本节点自己的订阅也会让话题出现在 graph 里，
  ///   把它当"候选"列出来会造成"名字没错"的假象（实测踩到过，见 ticket 04 的 Comments）。
  std::string describe_vrpn_candidates() const
  {
    const std::string want = to_lower(body_name_);
    std::string body_list;
    std::string marker_list;
    std::string publisher_less_list;
    for (const auto & topic : get_topic_names_and_types()) {
      const std::string & name = topic.first;
      if (name.rfind("/vrpn/", 0) != 0 || !ends_with(name, "/pose")) {
        continue;
      }
      if (count_publishers(name) == 0) {
        publisher_less_list += "    " + name + "\n";  // 多半就是本节点自己的订阅
        continue;
      }
      const bool is_marker = name.find("_Marker") != std::string::npos;
      // 只在"本体的、且与当前订阅不同"的话题上提示大小写差异（marker 话题与自己的订阅话题不提示）。
      const bool case_insensitive_hit =
        !is_marker && name != input_topic_ && to_lower(name).find(want) != std::string::npos;
      std::string & dst = is_marker ? marker_list : body_list;
      dst += "    " + name + (case_insensitive_hit ? "   ←★ 与 body_name 只差大小写？\n" : "\n");
    }
    if (body_list.empty() && marker_list.empty()) {
      return "graph 里**没有任何有发布者**的 /vrpn/*/pose" +
             (publisher_less_list.empty() ? std::string{} :
             "（下列话题没有发布者，多半就是本节点自己的订阅，**不算候选**）：\n" +
             publisher_less_list) +
             "  ⟹ vrpn_listener 起了吗？（ros2 topic list | grep vrpn；"
             "ros2 topic info <话题> 看 Publisher count。★ 刚起的发布者 DDS 发现还要 1–3 s，"
             "本机实测约 2.5 s，不是协议问题）";
    }
    return "graph 里现有的 /vrpn/*/pose：\n" + body_list +
           (marker_list.empty() ? std::string{} :
           "  （单颗反光球各自是独立 sender，下游不需要）\n" + marker_list) +
           "  ⟹ 用 ros2 topic list | grep vrpn 现场核对，把 body_name 改成实际名字";
  }

  std::string body_name_;
  mocap::Vec3 r_imu_in_body_{};   ///< 刚体原点 → IMU 中心（机体系 FLU，m）
  std::string input_topic_;
  std::string twist_topic_;
  std::string vision_pose_topic_;
  std::string pose_enu_topic_;
  std::string velocity_enu_topic_;
  std::string state_topic_;
  int boot_samples_{30};
  double jump_warn_m_{0.05};
  double no_data_warn_s_{5.0};

  bool has_state_{false};
  bool armed_{false};
  std::size_t pose_count_{0};
  bool logged_boot_lock_{false};
  bool logged_switch_{false};
  bool logged_no_data_{false};
  bool logged_stale_{false};
  bool received_twist_{false};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<mocap::TwoStageOrigin> selector_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_enu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_enu_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapRelPoseNode>());
  rclcpp::shutdown();
  return 0;
}
