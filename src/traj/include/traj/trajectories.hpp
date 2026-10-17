// traj 的纯函数部分 —— ★ 本文件零 ROS 依赖（不 include rclcpp / geometry_msgs / visualization_msgs）。
//
// 为什么单独拆出来（沿用 `mocap` / `control` 的先例，见 .scratch/phase1/spec.md 的「接缝 2」）：
//   几何与限速的错误**在话题层面没有诊断分辨率** —— 飞机画出个偏扁的 8 字、或某一段超速，
//   在 /traj/reference 的数值流里分不出"公式错了"还是"参数错了"。做成纯函数 ⟹ 直接单测，
//   成本≈0，挡住的却是"飞起来才发现"的一整类错误。
//
// ★★ 坐标系与口径（写死，不靠记忆）：
//   本文件里所有量都在**轨迹原点系**里，且是 **ROS/ENU 口径**：x 东、y 北、**z 向上为正**。
//   · **轨迹原点 ≠ 动捕原点**：它是 `control` 在 HOVER 判定稳定那一刻记录的 EKF2 位置
//     （docs/control/phase1-implementation.md §3.3）。两者之间可能有常值偏移。
//   · 三条**平面**轨迹（悬停/正方形/8 字）都从 (0, 0, height) 出发 —— 也就是"从当前悬停点开始"，
//     **不加任何绝对偏移**；第 4 条（起飞）从 **(0, 0, 0)** 出发 = 从轨迹原点往上爬（见 `Takeoff`）。
//     ★ 把相对量变成绝对位置（加上轨迹原点）是**持有原点那一方**的事，`control` 与
//       `mpc_node`（`traj_reference` 的锚点 + 参考）各算各的，但**只在自己那一处**：
//       本文件只吐相对量，免得三处各存一份原点而慢慢不一致。
//   · `height` 默认 0 = 轨迹平面就是悬停高度。§5.2 那句"高度 1.0 m"由**轨迹原点自带**
//     （那个点本来就是起飞到 1 m 之后的悬停点），**不是在这里再抬 1 m** ——
//      照"绝对高度"理解而在这里加 1.0，飞机会爬到 2 m。要显式偏移就用 height_offset_m
//      （⚠️ 第 4 条不参与它，理由见 `Takeoff`）。
//
// ★ 生成逻辑不依赖任何"当前状态"：给定 (轨迹编号, 参数) 与时刻 t，输出唯一确定。

#ifndef TRAJ__TRAJECTORIES_HPP_
#define TRAJ__TRAJECTORIES_HPP_

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace traj
{

/// 内部小工具。★ 必须定义在下面那些类**之前** —— 类内定义的成员函数体是在
/// "类定义结束处"查找名字的，写在文件末尾的 namespace 对它不可见（会编不过）。
namespace detail
{

inline double clamp(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

inline double clamp01(double v) {return clamp(v, 0.0, 1.0);}

}  // namespace detail

// —— 最小 POD 类型：只为把几何量搬进/搬出纯函数，与 ROS 消息解耦 ——

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// 某一时刻的参考状态。
struct Reference
{
  Vec3 p{};              ///< 位置 [m]，轨迹原点系（ENU：x 东 / y 北 / z 上）
  Vec3 v{};              ///< 速度 [m/s]，同一坐标系
  double progress{0.0};  ///< 完成度 0–1（1 = 跑完；跑完之后**保持 1**）
};

/// 四条轨迹的编号。★ 数值就是 `/traj/select` 上发的那个数字，**不要重排**。
/// ★ 第 4 条为什么不叫 0：`/traj/select` 的 **0 = 清空**（见 `parse_kind` 与节点侧 `on_select`），
///   那个号一个字节都不动 ⟹ 新轨迹取下一个空号（2026-10-16 操作者裁定）。
enum class Kind : unsigned char
{
  kHover = 1,        ///< 原地悬停：目标恒为 (0,0,z)
  kSquare = 2,       ///< 正方形：边长 square_side_m
  kLemniscate = 3,   ///< 8 字：Gerono 双纽线，x 幅值 = lemniscate_a_m
  kTakeoff = 4,      ///< 起飞：从轨迹原点匀速升到 takeoff_height_m，然后停在那儿
};

const char * to_string(Kind k);

/// `/traj/select` 的取值 → 编号。**0 与其余值都返回 false**（0 在节点侧另有含义：清空）。
bool parse_kind(unsigned char value, Kind & out);

/// 轨迹发生器的参数（与 `config/traj.yaml` 一一对应）。纯 POD，零 ROS。
struct Params
{
  /// 沿轨迹的**速度上限** [m/s]。t 的推进速率按它缩放 ⟹ 弧长匀速。
  /// ⚠️ Phase 1 的设计值是 **0.5 m/s**（ticket 07 的验收判据）；这是"任一时刻都不超过"的上限，
  /// 不是平均速度。改大之前先读 §5.2（轨迹尺寸与动捕棚的捕捉范围是配套的）。
  double max_speed_ms{0.5};

  /// 轨迹 1 的时长 [s]。★ 悬停没有"终点"，天生跑不完，而完成度必须是 0–1 ⟹ 用时长兜。
  /// 默认 60.0 = §5.3 的验收值（"定点水平误差 ≤15 cm、持续 60 s"）。
  double hover_duration_s{60.0};

  double square_side_m{1.0};       ///< 轨迹 2 的边长 [m]
  double lemniscate_a_m{1.0};      ///< 轨迹 3 的幅度 [m]（x 的半幅；y 的半幅只有它的一半）。
                                   ///< ★ 2026-10-08 由 0.5 放大到 1.0（原本的 2 倍），见 traj.yaml。
  double height_offset_m{0.0};     ///< 轨迹平面相对轨迹原点的高度偏移 [m]，默认 0（见文件头）。
                                   ///< ⚠️ **轨迹 4（起飞）不参与这一条** —— 它是从轨迹原点往上爬，
                                   ///<   再加偏移没有意义（见 `Takeoff` 的注释与 traj.yaml）。

  /// 轨迹 4 升到多高 [m]，**相对轨迹原点**（不是绝对高度）。默认 0.2 = "起飞 20 cm"。
  /// ★ 2026-10-16 操作者裁定**写死 0.2**（起链时**不问**操作者）：要改就改这里的默认值或 traj.yaml。
  double takeoff_height_m{0.2};

  /// 轨迹 4 的**爬升速率** [m/s]。默认 0.2 ⟹ 20 cm 用 1.0 s。
  /// ★ 单独一条而不是复用 `max_speed_ms`（0.5）：那是三条"平面轨迹"的限速上限，
  ///   按它爬 20 cm 只要 0.4 s —— 起飞要的是"看得见的匀速上升"，不是"冲到顶"。
  ///   ⚠️ 节点侧会核对它 ≤ `max_speed_ms`（超了告警，见 `warn_about_parameters`）。
  double takeoff_speed_ms{0.2};
};

// ——————————————————————————————————————————————————————————————
// 轨迹基类：at(t) 是唯一的生成入口
// ——————————————————————————————————————————————————————————————

class Trajectory
{
public:
  virtual ~Trajectory() = default;

  /// 轨迹开始后 `t_s` 秒的参考状态。`t_s < 0` 视作 0；`t_s > duration()` 视作终点
  /// （**跑完就停在终点，不是停发** —— 停不停发是节点侧的决定）。
  virtual Reference at(double t_s) const = 0;

  /// 跑完整条轨迹的时长 [s]。恒 > 0（构造时把非正参数夹住，见各实现的注释）。
  virtual double duration() const = 0;

  virtual Kind kind() const = 0;

  /// 整条轨迹的**几何采样**（给 RViz 画形状用）。★ 按几何采样而不是按时间采样：
  /// 画出来的是"这条曲线的形状与尺寸"，与限速、路径长度都无关 —— 这正是 ticket 07
  /// 要在 RViz 里核对的东西。返回的点带上 height。
  virtual std::vector<Vec3> path(int points) const = 0;
};

// ——————————————————————————————————————————————————————————————
// ① 原地悬停
// ——————————————————————————————————————————————————————————————

class Hover : public Trajectory
{
public:
  /// @param duration_s 完成度从 0 走到 1 的时长（悬停没有终点，这是唯一能让它有完成度的办法）
  /// @param height 轨迹平面高度（默认 0 = 轨迹原点所在高度）
  explicit Hover(double duration_s, double height = 0.0)
  : duration_(duration_s > 0.0 ? duration_s : kMinDuration),
    height_(height)
  {
  }

  Reference at(double t_s) const override
  {
    Reference r;
    r.p = Vec3{0.0, 0.0, height_};
    r.v = Vec3{};  // 原地悬停：速度恒为 0（前馈没有东西可前馈）
    r.progress = detail::clamp01(t_s / duration_);
    return r;
  }

  double duration() const override {return duration_;}
  Kind kind() const override {return Kind::kHover;}

  /// 一个点。LINE_STRIP 画不出东西（本来就只是一点）—— RViz 里靠参考点球体看它在哪。
  std::vector<Vec3> path(int) const override {return {Vec3{0.0, 0.0, height_}};}

private:
  static constexpr double kMinDuration = 1e-3;  // 非正时长会让 progress 除零；夹住而不是崩
  double duration_;
  double height_;
};

// ——————————————————————————————————————————————————————————————
// ② 正方形：(0,0) → (side,0) → (side,side) → (0,side) → (0,0)
// ——————————————————————————————————————————————————————————————

class Square : public Trajectory
{
public:
  /// @param side_m 边长 [m]
  /// @param max_speed_ms 沿边的速度 [m/s]（恒定，四个角上只换方向不换大小）
  Square(double side_m, double max_speed_ms, double height = 0.0)
  : side_(side_m > 0.0 ? side_m : kMinSide),
    speed_(max_speed_ms > 0.0 ? max_speed_ms : kMinSpeed),
    height_(height),
    perimeter_(4.0 * side_)
  {
  }

  /// 周长 4·side / 限速。1 m 边长 @0.5 m/s = **8.0 s**（§5.2）。
  double duration() const override {return perimeter_ / speed_;}

  Reference at(double t_s) const override
  {
    Reference r;
    r.p.z = height_;
    // 沿弧长匀速推进：s = 限速 × t（t 被夹到 [0, duration] ⟹ s 被夹到 [0, 周长]）
    const double s = detail::clamp(t_s, 0.0, duration()) * speed_;
    if (s >= perimeter_) {          // 跑完：正方形是闭合的 ⟹ 停在起点
      r.progress = 1.0;
      return r;                     // v 保持 0
    }
    r.progress = s / perimeter_;
    const int edge = static_cast<int>(s / side_);   // 0..3（s < 周长 ⟹ 不会越界）
    const double u = s - edge * side_;              // 该条边上已经走了多少
    switch (edge) {
      case 0:  r.p.x = u;           r.p.y = 0.0;         r.v.x = speed_;  break;
      case 1:  r.p.x = side_;       r.p.y = u;           r.v.y = speed_;  break;
      case 2:  r.p.x = side_ - u;   r.p.y = side_;       r.v.x = -speed_; break;
      default: r.p.x = 0.0;         r.p.y = side_ - u;   r.v.y = -speed_; break;
    }
    return r;
  }

  Kind kind() const override {return Kind::kSquare;}

  /// 五个顶点（首尾重合 = 闭合）。★ 按几何采样而不是按时间采样：`points` 参数对它无意义
  /// （形状就是这五个角），这是刻意的 —— 否则边长为 1 m 的直角会被时间采样磨圆。
  std::vector<Vec3> path(int) const override
  {
    return {
      Vec3{0.0, 0.0, height_},
      Vec3{side_, 0.0, height_},
      Vec3{side_, side_, height_},
      Vec3{0.0, side_, height_},
      Vec3{0.0, 0.0, height_},
    };
  }

private:
  static constexpr double kMinSide = 1e-6;
  static constexpr double kMinSpeed = 1e-6;
  double side_;
  double speed_;
  double height_;
  double perimeter_;
};

// ——————————————————————————————————————————————————————————————
// ③ 8 字：Gerono 双纽线
// ——————————————————————————————————————————————————————————————

/// x(θ) = a·sin θ，y(θ) = a·sin θ·cos θ，θ: 0→2π。
///
/// ★ **它的实际尺寸是不对称的**（数值积分/求极值算出来的，不是估的）：
///   x ∈ [−a, +a]（宽 2a），**y ∈ [−a/2, +a/2]（高只有 a）** ——
///   y 的幅度恰好是 x 的一半，所以 8 字**偏扁**。a = 1.0 m（2026-10-08 起的默认）⟹ 宽 2.0 m、高 1.0 m；
///   放大前 a = 0.5 m ⟹ 宽 1.0 m、高 0.5 m。
///   想要两个方向都是 ±a 的对称 8 字要换 Bernoulli 双纽线（§5.2）——那是**显示偏好**，
///   不影响控制逻辑，所以先按 Gerono 做。
///
/// ★ **弧长匀速**：θ 不是弧长参数，直接让 θ 匀速会让飞机在"过中心"那一段冲得最快
///   （`|dr/dθ|` = a·√(cos²θ + cos²2θ) 实测 ∈ [0.661, 1.414]·a、均值 0.970·a ⟹
///   **峰值是均值的 1.46 倍、谷值是 0.68 倍** —— 按 θ 匀速时把"平均"压到 0.5 m/s，
///   峰值就是 0.73 m/s，**直接违反限速**；而报告的 |v| 仍然是 0.5，只有位置差分能看出这个错）。
///   所以这里**预计算一张弧长表**再按弧长反查 θ —— 限速判据（"任一时刻 ≤ max_speed"）
///   因此是按弧长成立的，而不是按参数。
///
/// ⚠️ 残余误差（写在明处，别当成 0）：反查是**分段线性**的 ⟹ 每段内 dθ/ds 是常数，
///   而真值在该段内是变化的 ⟹ 位置的真实速度有一个量级为 (h/2)·|f′/f| 的**波纹**
///   （h = 2π/table_size）。**实测**（中心差分 dt = 1e-5 s、步长 0.01 s 扫全曲线，
///   与 `test/test_trajectories.cpp` 那条波纹测试同一把尺）：
///     表 2000 点 ⟹ 波纹峰值 **1.09e-3 m/s**（相对 2.2e-3）   ← 5e-4 的相对阈值**不够**
///     表 20000 点（默认）⟹ **9.8e-5 m/s**（相对 2.0e-4）
///     表 200000 点 ⟹ 3.7e-6 m/s
///   ⟹ 线性收敛；默认值下比跟踪误差判据（§5.3 的 15 cm）小四个数量级，可以忽略。
///   ★ 更正（2026-09-22，Spec 审查时独立复算发现）：这里原本写"默认 20000 点时 ±1e-4 相对、
///   0.5 m/s ⟹ 5e-5 m/s"，**低估了约 2 倍**；`README` 里那句"2000 点 5.3e-4"同样低估 2 倍。
///   阈值本身没问题（gtest 用 1e-3 与 5e-4 两条，各留 5× 与 2.5× 余量），错的是注释里的数。
class Lemniscate : public Trajectory
{
public:
  /// @param a_m x 的半幅 [m]（默认 1.0 ⟹ 宽 2.0 m、高 1.0 m、周长 ≈6.10 m）
  /// @param max_speed_ms 沿弧长的速度 [m/s]
  /// @param table_size 弧长表的采样点数（梯形法积分；2000 点周长相对误差已 < 1e-6，
  ///   这里给 20000 是为了压住上面说的**速度波纹**，不是为了积分精度）
  Lemniscate(double a_m, double max_speed_ms, double height = 0.0, int table_size = 20000)
  : a_(a_m > 0.0 ? a_m : kMinA),
    speed_(max_speed_ms > 0.0 ? max_speed_ms : kMinSpeed),
    height_(height),
    n_(table_size < 8 ? 8 : table_size)
  {
    build_arc_table();
  }

  /// 周长 / 限速。a = 1.0 m 时周长 ≈6.10 m ⟹ @0.5 m/s ≈ **12.2 s**（§5.2）。
  double duration() const override {return perimeter_ / speed_;}

  /// 当前生效的周长 [m]（测试与日志用；也顺带说明"时长是算出来的、不是写死的"）。
  double perimeter() const {return perimeter_;}

  Reference at(double t_s) const override
  {
    Reference r;
    const double s = detail::clamp(t_s, 0.0, duration()) * speed_;  // 沿弧长的行进量
    if (s >= perimeter_) {          // 跑完：曲线闭合（θ=2π 与 θ=0 同一点）⟹ 停在起点
      r.p = point_at(0.0);
      r.progress = 1.0;
      return r;
    }
    r.progress = s / perimeter_;
    const double theta = theta_of_arc(s);
    r.p = point_at(theta);
    // 方向取 dr/dθ 的单位向量 ⟹ |v| 恒 = 限速（这也是"报告的 v"唯一该有的样子）
    const Vec3 dir = unit_tangent(theta);
    r.v = Vec3{dir.x * speed_, dir.y * speed_, dir.z * speed_};
    return r;
  }

  Kind kind() const override {return Kind::kLemniscate;}

  /// 按**参数 θ 均匀**采样的闭合折线。θ 均匀 ≠ 弧长均匀 ⟹ 点会往"曲率大/弧长跑得快"的
  /// 地方稍微聚一点，但**画形状与量尺寸足够准**（顶点位置是解析解，不含插值误差）。
  std::vector<Vec3> path(int points) const override
  {
    const int n = points < 2 ? 2 : points;
    std::vector<Vec3> out;
    out.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
      out.push_back(point_at(2.0 * M_PI * i / n));
    }
    return out;
  }

private:
  static constexpr double kMinA = 1e-6;
  static constexpr double kMinSpeed = 1e-6;

  /// |dr/dθ| 中与 a 无关的那一半：√(cos²θ + cos²2θ)。
  /// ★ 它**不会同时为 0**（cosθ = 0 时 cos2θ = −1；反之亦然）⟹ 下面不会除到 0。
  static double speed_factor(double theta)
  {
    const double c1 = std::cos(theta);
    const double c2 = std::cos(2.0 * theta);
    return std::sqrt(c1 * c1 + c2 * c2);
  }

  Vec3 point_at(double theta) const
  {
    const double s = std::sin(theta);
    return Vec3{a_ * s, a_ * s * std::cos(theta), height_};
  }

  /// dr/dθ 的单位向量（切线方向）。
  Vec3 unit_tangent(double theta) const
  {
    const double f = speed_factor(theta);
    return Vec3{std::cos(theta) / f, std::cos(2.0 * theta) / f, 0.0};
  }

  /// 预计算 θ → 弧长 的梯形积分表（θ 均匀，弧长严格单调递增 ⟹ 可反查）。
  void build_arc_table()
  {
    theta_.resize(static_cast<std::size_t>(n_) + 1);
    arc_.resize(static_cast<std::size_t>(n_) + 1);
    const double dtheta = 2.0 * M_PI / n_;
    theta_[0] = 0.0;
    arc_[0] = 0.0;
    double prev = a_ * speed_factor(0.0);
    for (int i = 1; i <= n_; ++i) {
      const double theta = dtheta * i;
      theta_[static_cast<std::size_t>(i)] = theta;
      const double cur = a_ * speed_factor(theta);
      arc_[static_cast<std::size_t>(i)] = arc_[static_cast<std::size_t>(i - 1)] +
        0.5 * (prev + cur) * dtheta;
      prev = cur;
    }
    perimeter_ = arc_.back();
  }

  /// 弧长 → θ：在表里二分 + 线性插值。
  double theta_of_arc(double s) const
  {
    // arc_ 严格递增 ⟹ 找第一个 > s 的表项
    const auto it = std::upper_bound(arc_.begin(), arc_.end(), s);
    const std::size_t hi = static_cast<std::size_t>(it - arc_.begin());
    if (hi == 0) {
      return theta_[0];
    }
    if (hi >= arc_.size()) {
      return theta_.back();
    }
    const std::size_t lo = hi - 1;
    const double span = arc_[hi] - arc_[lo];
    const double frac = span > 0.0 ? (s - arc_[lo]) / span : 0.0;
    return theta_[lo] + frac * (theta_[hi] - theta_[lo]);
  }

  double a_;
  double speed_;
  double height_;
  int n_;
  std::vector<double> theta_;
  std::vector<double> arc_;
  double perimeter_{0.0};
};

// ——————————————————————————————————————————————————————————————
// ④ 起飞：从轨迹原点匀速升到 takeoff_height_m，到顶后停在那儿
// ——————————————————————————————————————————————————————————————

/// `z(t) = h · clamp01(t / (|h|/v))`，`v = takeoff_speed_ms`；到顶后 `ż = 0`、位置恒为 `h`。
///
/// ★★ **零点就是轨迹原点**（与本文件其它三条同一条纪律）：`z(0) = 0`。对"起飞"这条轨迹来说，
///   轨迹原点在**地面上**（对模型预测控制档 = 闸门第一次打开那一拍的锚点）⟹ `z` 就是**离地高度**，
///   `h = 0.2` 的物理含义正是"起飞 20 cm"。⛔ 别把它读成"绝对高度"或"悬停点之上的偏移"。
///
/// ★ **斜坡而不是一步到位**（2026-10-16 操作者在这两条里选的）：升速**有界**（= `takeoff_speed_ms`），
///   与 §5.2 那条限速同取向；一步到位等于把爬升速率交给闭环自己定，而"起飞"这一步
///   恰恰是最不该让控制器即兴发挥的那一步。
///
/// ★ **不参与 `height_offset_m`**：与 `Hover` / `Square` / `Lemniscate` 的差别就在这一处 ——
///   那三条是"某个高度的平面轨迹"（偏移有意义），这一条是"从原点往上爬"（偏移只会让人困惑）。
///
/// ★ **到顶之后就是悬停** ⟹ 时长沿用 `hover_duration_s`（不另开一条参数），`progress` 同悬停：
///   按时长线性走到 1（**不是**按斜坡走完就算 1）—— 这样"跑完"仍然等于"这一档的时长到了"，
///   与其它三条的语义一致，`control` 那条 `progress ≥ 1 ⟹ LAND` 也不会在爬升刚结束时触发。
class Takeoff : public Trajectory
{
public:
  /// @param duration_s 完成度从 0 走到 1 的时长 [s]（节点侧传 `hover_duration_s`）
  /// @param height_m 升到多高 [m]，**相对轨迹原点**；负值 = 降到地平面以下（允许，符号跟着走）
  /// @param climb_speed_ms 爬升速率 [m/s]（非正值被夹住，不让进度除法出现 0）
  explicit Takeoff(double duration_s, double height_m, double climb_speed_ms)
  : duration_(duration_s > 0.0 ? duration_s : kMinDuration),
    height_(height_m),
    speed_(climb_speed_ms > 0.0 ? climb_speed_ms : kMinSpeed),
    ramp_s_(std::abs(height_m) / speed_)
  {
  }

  Reference at(double t_s) const override
  {
    Reference r;
    const double t = t_s < 0.0 ? 0.0 : t_s;   // 与其它三条同一条口径：t < 0 视作 0
    if (t >= ramp_s_) {
      r.p = Vec3{0.0, 0.0, height_};          // 到顶：停在那儿（v 保持 0）
    } else {
      r.p = Vec3{0.0, 0.0, height_ * (t / ramp_s_)};
      r.v = Vec3{0.0, 0.0, height_ >= 0.0 ? speed_ : -speed_};
    }
    r.progress = detail::clamp01(t / duration_);
    return r;
  }

  double duration() const override {return duration_;}
  Kind kind() const override {return Kind::kTakeoff;}

  /// 爬升用了多久 [s]（`|h|/v`；`h = 0` 时是 0）。测试与日志用 —— 与 `Lemniscate::perimeter()`
  /// 同一个理由：把"算出来的"亮出来，免得有人以为它写死在别处。
  double ramp_duration() const {return ramp_s_;}

  /// 起止两点（LINE_STRIP 画竖线）。★ 按几何采样：`points` 对它无意义（形状就是这一段）。
  std::vector<Vec3> path(int) const override
  {
    return {Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, height_}};
  }

private:
  static constexpr double kMinDuration = 1e-3;
  static constexpr double kMinSpeed = 1e-6;
  double duration_;
  double height_;
  double speed_;
  double ramp_s_;   // = |height_| / speed_（构造时算一次；h = 0 ⟹ 0 ⟹ at() 走"到顶"那一支）
};

// ——————————————————————————————————————————————————————————————
// 编号 → 轨迹：参数到行为的映射**只在这一处**
// ——————————————————————————————————————————————————————————————

inline std::unique_ptr<Trajectory> make(Kind kind, const Params & params)
{
  switch (kind) {
    case Kind::kHover:
      return std::make_unique<Hover>(params.hover_duration_s, params.height_offset_m);
    case Kind::kSquare:
      return std::make_unique<Square>(
        params.square_side_m, params.max_speed_ms, params.height_offset_m);
    case Kind::kLemniscate:
      return std::make_unique<Lemniscate>(
        params.lemniscate_a_m, params.max_speed_ms, params.height_offset_m);
    case Kind::kTakeoff:
      // ★ 时长沿用 hover_duration_s（到顶之后就是悬停）；height_offset_m 这条**不传**（见 Takeoff 的注释）
      return std::make_unique<Takeoff>(
        params.hover_duration_s, params.takeoff_height_m, params.takeoff_speed_ms);
  }
  return nullptr;  // 不可达：上面的 switch 覆盖了全部枚举
}

inline const char * to_string(Kind k)
{
  switch (k) {
    case Kind::kHover:      return "hover";
    case Kind::kSquare:     return "square";
    case Kind::kLemniscate: return "lemniscate";
    case Kind::kTakeoff:    return "takeoff";
  }
  return "?";
}

inline bool parse_kind(unsigned char value, Kind & out)
{
  switch (value) {
    case 1: out = Kind::kHover;      return true;
    case 2: out = Kind::kSquare;     return true;
    case 3: out = Kind::kLemniscate; return true;
    case 4: out = Kind::kTakeoff;    return true;
    default: return false;   // ★ 0 落在这里 = 清空，节点侧另有分支（`on_select`）
  }
}

}  // namespace traj

#endif  // TRAJ__TRAJECTORIES_HPP_
