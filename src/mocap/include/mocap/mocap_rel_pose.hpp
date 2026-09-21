// 相对位姿的纯转换部分 —— ★ 本文件零 ROS 依赖（不 include rclcpp / geometry_msgs / tf2）。
//
// 为什么单独拆出来（这是本项目全项目唯一的「刻意测试性设计」，见
// docs/control/phase1-implementation.md §3 与 .scratch/phase1/spec.md「seam 2」）：
//   位姿处理错误**在话题层面没有诊断分辨率** —— 飞机往反方向飞时，话题上分不出是
//   原点锁错、yaw 归零错，还是 mavros 的边界转换不如预期。所以把「减原点 + 仅归零 yaw +
//   两阶段原点切换」做成 (位姿, 原点) → 位姿 的纯函数，直接单测，成本≈0。
//
// ★ 本文件**不做任何坐标系转换**：输入输出都是**动捕原生 ENU / 机体 FLU**。
//   ENU→NED 与 FLU→FRD 全由 mavros 在它自己的边界上完成（已读 mavros 源码确认，§3.5）。
//   本节点是全栈唯一一个「不做坐标转换」的节点 —— 不要在这里加转换。

#ifndef MOCAP__MOCAP_REL_POSE_HPP_
#define MOCAP__MOCAP_REL_POSE_HPP_

#include <cmath>

namespace mocap
{

// —— 最小 POD 类型：只为把几何量搬进/搬出纯函数，与 ROS 消息解耦 ——

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// 四元数 (x, y, z, w)，与 geometry_msgs/Quaternion 字段同名同序。
struct Quat
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose
{
  Vec3 p{};
  Quat q{};
};

/// `/mavros/state` 给到的解锁状态。**三态而不是两个 bool**：
/// 「还没收到过 state」与「收到且未解锁」是两件事 —— 前者若被当作"未解锁"，是安全的；
/// 但若被当作"已解锁"，启动瞬间就会伪造出一个解锁上升沿。这个区别用类型表达出来。
enum class ArmedState
{
  Unknown,   ///< 还没收到过 /mavros/state
  Disarmed,
  Armed
};

/// 锁定下来的原点 = 位置平移量 + 被归零掉的 yaw。
/// ⚠️ 是否归零 yaw 由 `TwoStageOrigin` 的**阶段**决定（阶段 A 不归零），不在这里另存一份，
/// 免得两处状态要手工保持同步。
struct Origin
{
  Vec3 p{};          ///< 位置原点（动捕 ENU 原生，单位 m）
  double yaw{0.0};   ///< 被归零掉的 yaw [rad]
  bool valid{false}; ///< 原点是否已锁定
};

/// 两阶段的阶段名。
enum class OriginStage
{
  Boot,  ///< 阶段 A：用「启动原点」，姿态**不**归零 yaw
  Arm    ///< 阶段 B：用「起飞前静止帧原点」，姿态归零 yaw
};

// —— 四元数 / 向量小工具（就地实现，避免为了几个算子引入 tf2）——

inline Quat normalize(const Quat & q)
{
  const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!(n > 1e-12)) {
    return Quat{};  // 退化输入 → 单位四元数，宁可不动也不要产生 NaN
  }
  return Quat{q.x / n, q.y / n, q.z / n, q.w / n};
}

inline Quat quat_mul(const Quat & a, const Quat & b)
{
  return Quat{
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

/// 绕世界 z 轴的纯 yaw 旋转。
inline Quat quat_from_yaw(double yaw)
{
  return Quat{0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5)};
}

/// ZYX（yaw-pitch-roll）约定下取 yaw [rad]。
inline double yaw_of(const Quat & q_in)
{
  const Quat q = normalize(q_in);
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline double norm(const Vec3 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// ——————————————————————————————————————————————————————————————
// ① 原点减法：位置相对化，**逐轴独立**
// ——————————————————————————————————————————————————————————————

/// 输出 = 当前位置 − 锁定原点，逐轴相减。
/// ★ 位置与姿态**完全解耦** —— 本函数的签名里根本没有姿态，姿态处理错也不会影响位置。
inline Vec3 relative_position(const Vec3 & p, const Vec3 & origin)
{
  return Vec3{p.x - origin.x, p.y - origin.y, p.z - origin.z};
}

// ——————————————————————————————————————————————————————————————
// ② 仅归零 yaw：roll / pitch 保持绝对
// ——————————————————————————————————————————————————————————————

/// q_rel = Rz(−yaw0) ⊗ q —— **左乘**一个绕世界 z 的纯 yaw 旋转。
///
/// ★ 为什么只归零 yaw、不整个姿态做相对：解锁时飞机停在地面，roll/pitch 通常不是精确的 0
///   （地面不平、起落架软）。整个姿态做相对之后，"水平"的定义变成了"解锁时那个姿态"——
///   若解锁时有 5° 横滚，PX4 认为的竖直方向就偏 5°，飞机会持续往一个方向漂，
///   **而 EKF2 认为误差是零，控制器看不出来**。
///
/// 左乘纯 yaw 旋转在 ZYX 分解下等价于"只减 yaw"：R = Rz(ψ)Ry(θ)Rx(φ) ⟹ Rz(−ψ0)R = Rz(ψ−ψ0)Ry(θ)Rx(φ)。
/// ⟹ roll / pitch **精确不变**，机体 z 轴与竖直方向的夹角**精确不变**（推力方向不被人为改变）。
inline Quat yaw_zeroed(const Quat & q_in, double yaw0)
{
  return normalize(quat_mul(quat_from_yaw(-yaw0), normalize(q_in)));
}

// ——————————————————————————————————————————————————————————————
// ③ 两阶段原点切换
// ——————————————————————————————————————————————————————————————

namespace detail
{

/// 位置窗口：累加 → 计数 → 满窗取均值（两阶段共用，不写两遍）。
struct Vec3Window
{
  void add(const Vec3 & v)
  {
    sum_.x += v.x;
    sum_.y += v.y;
    sum_.z += v.z;
    ++count_;
  }

  void reset()
  {
    sum_ = Vec3{};
    count_ = 0;
  }

  bool full(int samples) const {return count_ >= samples;}
  Vec3 mean() const {return Vec3{sum_.x / count_, sum_.y / count_, sum_.z / count_};}

private:
  Vec3 sum_{};
  int count_{0};
};

/// 四元数窗口：取均值前先对参考帧做**半球对齐**（否则 w 变号的两帧会互相抵消），
/// 静止时等价于对角度取均值。
struct QuatWindow
{
  void reset(const Quat & reference)
  {
    reference_ = normalize(reference);
    sum_ = Quat{0.0, 0.0, 0.0, 0.0};
  }

  void add(const Quat & q_in)
  {
    const Quat q = normalize(q_in);
    const double dot = q.x * reference_.x + q.y * reference_.y + q.z * reference_.z +
      q.w * reference_.w;
    const double sign = (dot < 0.0) ? -1.0 : 1.0;
    sum_.x += sign * q.x;
    sum_.y += sign * q.y;
    sum_.z += sign * q.z;
    sum_.w += sign * q.w;
  }

  double mean_yaw() const {return yaw_of(sum_);}

private:
  Quat reference_{};
  Quat sum_{0.0, 0.0, 0.0, 0.0};
};

}  // namespace detail

/// 「启动原点 → 起飞前静止帧原点」的两阶段原点选择器。**纯逻辑，零 ROS 依赖。**
///
///   阶段 A（节点启动 → 解锁前）：锁定启动原点并**持续输出**相对位姿，姿态保持绝对。
///     ⟹ 目的：让 EKF2 在**解锁之前**就已经吃到 vision_pose 并收敛。
///        若实现成"解锁前什么都不发"，等到解锁那一刻才第一次喂，EKF2 能否干净初始化
///        是个**未验证**的问题，而 PX4 解锁与进入 OFFBOARD 都要求已收敛的估计。
///   阶段 B（解锁上升沿）：采一小段静止帧，切换为"起飞前静止帧原点"，姿态开始归零 yaw。
///     ⟹ 切换瞬间的位置跳变 = |p_arm − p_boot|。**只要"启动到解锁之间没有搬动飞机"，
///        这个跳变 ≈ 0**（这就是本项目新增的那条纪律，替代了老的"机头必须朝假定东"仪式）。
class TwoStageOrigin
{
public:
  /// boot_samples / arm_samples：两阶段的采样帧数（< 1 会被夹到 1，避免死锁）。
  TwoStageOrigin(int boot_samples, int arm_samples)
  : boot_samples_(boot_samples < 1 ? 1 : boot_samples),
    arm_samples_(arm_samples < 1 ? 1 : arm_samples)
  {
  }

  /// 喂入一帧动捕位姿，并告知当前的解锁状态。
  void observe(const Pose & pose, ArmedState armed_state)
  {
    // —— 1) 识别解锁上升沿。三条纪律：★ 上电即在高位不算边沿（同 RC 通道的边缘语义）；
    //       ★ 只认第一次上升沿；★ 切换完成后**永不重锁**（飞行中重锁原点 = 给 EKF2 一记位置跳变）。
    if (armed_state != ArmedState::Unknown) {
      const bool armed = armed_state == ArmedState::Armed;
      if (baseline_seen_ && armed && !baseline_armed_ && !arm_window_open_ && !switched_) {
        arm_window_open_ = true;
        arm_p_.reset();
        arm_q_.reset(pose.q);
      }
      baseline_seen_ = true;
      baseline_armed_ = armed;
    }

    // —— 2) 采样 ——
    if (!origin_.valid) {
      boot_p_.add(pose.p);
      if (boot_p_.full(boot_samples_)) {
        origin_ = Origin{boot_p_.mean(), 0.0, true};
        boot_origin_ = origin_;   // 阶段 A：姿态保持绝对（是否归零 yaw 由阶段决定）
      }
    } else if (arm_window_open_) {
      arm_p_.add(pose.p);
      arm_q_.add(pose.q);
      if (arm_p_.full(arm_samples_)) {
        const Vec3 p_arm = arm_p_.mean();
        origin_ = Origin{p_arm, arm_q_.mean_yaw(), true};
        switch_jump_ = norm(relative_position(p_arm, boot_origin_.p));
        switched_ = true;
        arm_window_open_ = false;
      }
    }
  }

  /// 原点是否已锁定（未锁定前**不要发布** —— 一帧都不能拿默认原点去糊）。
  bool ready() const {return origin_.valid;}

  /// 当前生效的原点。
  const Origin & origin() const {return origin_;}

  /// 阶段 A 的原点（切换后在日志里和 arm 原点对照，用来量"启动到解锁之间动了多少"）。
  const Origin & boot_origin() const {return boot_origin_;}

  OriginStage stage() const {return switched_ ? OriginStage::Arm : OriginStage::Boot;}

  /// 是否已完成阶段 A→B 切换。
  bool switched() const {return switched_;}

  /// 切换瞬间的位置跳变量 [m] = |p_arm − p_boot|。**接近 0 才符合纪律。**
  double switch_jump() const {return switch_jump_;}

  /// 相对位姿 = 减原点 + （阶段 B 才）归零 yaw。
  /// 原点未锁定时原样返回（节点此时不会发布，这里只是让行为可预期）。
  Pose apply(const Pose & pose) const
  {
    if (!origin_.valid) {
      return pose;
    }
    Pose out;
    out.p = relative_position(pose.p, origin_.p);
    out.q = (stage() == OriginStage::Arm) ? yaw_zeroed(pose.q, origin_.yaw) : pose.q;
    return out;
  }

private:
  int boot_samples_;
  int arm_samples_;

  // 阶段 A
  detail::Vec3Window boot_p_;

  // 解锁上升沿 / 阶段 B 采样窗
  bool baseline_seen_{false};
  bool baseline_armed_{false};
  bool arm_window_open_{false};
  detail::Vec3Window arm_p_;
  detail::QuatWindow arm_q_;

  // 结果
  Origin origin_{};
  Origin boot_origin_{};
  bool switched_{false};
  double switch_jump_{0.0};
};

}  // namespace mocap

#endif  // MOCAP__MOCAP_REL_POSE_HPP_
