// 接缝 2（纯函数）单测：四条轨迹的**几何、尺寸、限速、完成度**。
//
// 为什么这些断言值得写（而不是"飞起来看看"）：
//   ① 形状与尺寸有**精确解**可对 —— 边长 1 m 的正方形周长必然是 4.0 m、8 字（a = 1.0，
//      2026-10-08 起的默认）必然是宽 2.0 m 高 1.0 m。这类错误一旦漏到天上，表现只是"飞机画得有点怪"，
//      而"有点怪"和"动捕/控制有毛病"在 /traj/reference 的数值流里分不开。
//   ② 限速是 §5.2 的硬要求（≤0.5 m/s）。★ 这里不只断言"我输出的 v 不大于上限"
//      （那是自证），还用**有限差分**独立地量一遍"位置实际走多快" —— 后者才拦得住
//      "θ 匀速而不是弧长匀速"这个真会发生的错误（见 Lemniscate 的注释）。
//
// ★ 本文件零 ROS 依赖，与 `control/test/test_fsm.cpp` 同款：只 include 纯头文件。

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "traj/trajectories.hpp"

namespace
{

using traj::Kind;
using traj::Lemniscate;
using traj::Params;
using traj::Reference;
using traj::Square;
using traj::Vec3;

constexpr double kPi = 3.14159265358979323846;

double norm(const Vec3 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/// 独立实现的弧长积分（Simpson 法，与被测代码的梯形表**不是**同一套算法）。
/// 用它来交叉验证 Lemniscate 内部的弧长表与反查插值。
double lemniscate_perimeter_simpson(double a, int n = 20000)
{
  const auto f = [a](double th) {
      const double c1 = std::cos(th);
      const double c2 = std::cos(2.0 * th);
      return a * std::sqrt(c1 * c1 + c2 * c2);
    };
  const double h = 2.0 * kPi / n;  // n 取偶数
  double sum = f(0.0) + f(2.0 * kPi);
  for (int i = 1; i < n; ++i) {
    sum += (i % 2 == 0 ? 2.0 : 4.0) * f(h * i);
  }
  return sum * h / 3.0;
}

// ——————————————————————————————————————————————————————————————
// ① 悬停
// ——————————————————————————————————————————————————————————————

TEST(Hover, 位置恒在原点_速度恒为零)
{
  const traj::Hover h(60.0);
  for (double t = 0.0; t <= 60.0; t += 0.5) {
    const Reference r = h.at(t);
    EXPECT_DOUBLE_EQ(r.p.x, 0.0);
    EXPECT_DOUBLE_EQ(r.p.y, 0.0);
    EXPECT_DOUBLE_EQ(r.p.z, 0.0);
    EXPECT_DOUBLE_EQ(norm(r.v), 0.0);
  }
}

TEST(Hover, 完成度随时长线性走到_1)
{
  const traj::Hover h(10.0);
  EXPECT_DOUBLE_EQ(h.duration(), 10.0);
  EXPECT_DOUBLE_EQ(h.at(0.0).progress, 0.0);
  EXPECT_DOUBLE_EQ(h.at(2.5).progress, 0.25);
  EXPECT_NEAR(h.at(5.0).progress, 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(h.at(10.0).progress, 1.0);
  // 跑完之后保持 1，不回落也不越界
  EXPECT_DOUBLE_EQ(h.at(99.0).progress, 1.0);
  // t < 0 夹到 0
  EXPECT_DOUBLE_EQ(h.at(-1.0).progress, 0.0);
}

TEST(Hover, 高度偏移参数决定悬停平面)
{
  const traj::Hover h(5.0, 0.3);
  EXPECT_DOUBLE_EQ(h.at(1.0).p.z, 0.3);
  // ★ 默认值必须是 0：§5.2 那句"高度 1.0 m"由**轨迹原点自带**，
  //   在这里再加 1.0 会让飞机爬到 2 m（见 trajectories.hpp 文件头）。
  const traj::Hover plain(5.0);
  EXPECT_DOUBLE_EQ(plain.at(1.0).p.z, 0.0);
}

// ——————————————————————————————————————————————————————————————
// ② 正方形
// ——————————————————————————————————————————————————————————————

TEST(Square, 周长与时长_1m边长限速_0_5_是_8_秒)
{
  const Square sq(1.0, 0.5);
  EXPECT_NEAR(sq.duration(), 8.0, 1e-12);  // 4 m / 0.5 m/s
  // 时长按限速缩放：限速加倍 ⟹ 时长减半
  const Square fast(1.0, 1.0);
  EXPECT_NEAR(fast.duration(), 4.0, 1e-12);
}

TEST(Square, 四条边的端点与走向)
{
  const double side = 1.0;
  const double speed = 0.5;
  const Square sq(side, speed);
  const double d = sq.duration();

  // 每条边取中点 + 只差一点点的两端，核对"在正确的边上、朝正确的方向"
  struct Case
  {
    double frac;    // 沿整条轨迹的完成度
    double x, y;    // 期望位置
    double vx, vy;  // 期望速度方向
  };
  const Case cases[] = {
    {0.125, 0.5, 0.0, 1.0, 0.0},   // 边 0：+x
    {0.375, 1.0, 0.5, 0.0, 1.0},   // 边 1：+y
    {0.625, 0.5, 1.0, -1.0, 0.0},  // 边 2：−x
    {0.875, 0.0, 0.5, 0.0, -1.0},  // 边 3：−y
  };
  for (const Case & c : cases) {
    const Reference r = sq.at(c.frac * d);
    EXPECT_NEAR(r.p.x, c.x, 1e-12);
    EXPECT_NEAR(r.p.y, c.y, 1e-12);
    EXPECT_NEAR(r.v.x, c.vx * speed, 1e-12);
    EXPECT_NEAR(r.v.y, c.vy * speed, 1e-12);
    EXPECT_NEAR(norm(r.v), speed, 1e-12);
  }
}

TEST(Square, 起点在原点_每个角都能精确到达)
{
  const Square sq(1.0, 0.5);
  const double d = sq.duration();
  const Reference start = sq.at(0.0);
  EXPECT_DOUBLE_EQ(start.p.x, 0.0);
  EXPECT_DOUBLE_EQ(start.p.y, 0.0);
  EXPECT_DOUBLE_EQ(start.progress, 0.0);

  // 四个角分别出现在 t = k·(边长/限速) 处（弧长 = 边长 ⟹ 时间 = 边长/限速）
  const double per_edge = 1.0 / 0.5;
  const Vec3 corners[] = {
    {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}};
  for (int k = 1; k <= 4; ++k) {
    const Reference r = sq.at(per_edge * k);
    EXPECT_NEAR(r.p.x, corners[k - 1].x, 1e-12) << "角 " << k;
    EXPECT_NEAR(r.p.y, corners[k - 1].y, 1e-12) << "角 " << k;
  }
  EXPECT_NEAR(d, per_edge * 4, 1e-12);
}

TEST(Square, 跑完停在起点且速度为_0)
{
  const Square sq(1.0, 0.5);
  const Reference end = sq.at(sq.duration());
  EXPECT_DOUBLE_EQ(end.progress, 1.0);
  EXPECT_DOUBLE_EQ(end.p.x, 0.0);
  EXPECT_DOUBLE_EQ(end.p.y, 0.0);
  EXPECT_DOUBLE_EQ(norm(end.v), 0.0);  // ★ 停在终点，而不是继续飞出去
  // 超时也是同一个状态
  const Reference late = sq.at(1e6);
  EXPECT_DOUBLE_EQ(late.progress, 1.0);
  EXPECT_DOUBLE_EQ(norm(late.v), 0.0);
}

TEST(Square, 几何采样是五个顶点_四条_1m_边_闭合)
{
  const Square sq(1.0, 0.5);
  const std::vector<Vec3> path = sq.path(100);
  ASSERT_EQ(path.size(), 5u);
  EXPECT_EQ(path.front().x, path.back().x);  // 闭合
  EXPECT_EQ(path.front().y, path.back().y);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double edgelen = std::hypot(
      path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    EXPECT_NEAR(edgelen, 1.0, 1e-12) << "边 " << i - 1;  // ★ 1 m 边长，RViz 要量的就是它
  }
}

// ——————————————————————————————————————————————————————————————
// ③ 8 字
// ——————————————————————————————————————————————————————————————

TEST(Lemniscate, 尺寸_x_半幅_0_5_y_半幅只有_0_25)
{
  const Lemniscate lem(0.5, 0.5);
  double x_max = 0.0, x_min = 0.0, y_max = 0.0, y_min = 0.0;
  for (int i = 0; i <= 100000; ++i) {
    const Vec3 p = lem.at(lem.duration() * i / 100000.0).p;
    x_max = std::max(x_max, p.x);
    x_min = std::min(x_min, p.x);
    y_max = std::max(y_max, p.y);
    y_min = std::min(y_min, p.y);
  }
  // ★ Gerono 双纽线的 y 幅度**恰是 x 的一半** ⟹ 8 字是偏扁的。
  //   这里把"偏扁"当成**规格**钉住：若哪天换成 Bernoulli，这条测试必须先改。
  EXPECT_NEAR(x_max, +0.5, 1e-3);
  EXPECT_NEAR(x_min, -0.5, 1e-3);
  EXPECT_NEAR(y_max, +0.25, 1e-3);
  EXPECT_NEAR(y_min, -0.25, 1e-3);
}

TEST(Lemniscate, 周长与时长的交叉验证)
{
  const double a = 0.5;
  const Lemniscate lem(a, 0.5);
  // ① 与**独立实现**的 Simpson 积分对齐（相对误差 1e-6 以内 = 内部梯形表够密）
  const double expected = lemniscate_perimeter_simpson(a);
  EXPECT_NEAR(lem.perimeter(), expected, expected * 1e-6);
  // ② ticket / §5.2 给的解析值 ≈3.05 m
  EXPECT_NEAR(lem.perimeter(), 3.05, 0.01);
  // ③ 时长 = 周长 / 限速，@0.5 m/s ≈ 6.1 s
  EXPECT_NEAR(lem.duration(), lem.perimeter() / 0.5, 1e-12);
  EXPECT_NEAR(lem.duration(), 6.1, 0.02);
}

TEST(Lemniscate, 放大到原本的_2_倍_后尺寸与时长相乘而不是相加)
{
  // ★ 2026-10-08 操作者把轨迹 3 的幅度从 0.5 放大到 1.0（**原本的 2 倍**）。
  //   放大是一条**相似变换** ⟹ 长度量都乘 2、时间也乘 2，而**限速一个字不改**（仍 0.5 m/s）。
  //   在纯函数层把这三件事钉住；话题层（seam 1）另有一条按新规格 ±1.0 / ±0.5 的断言。
  const Lemniscate before(0.5, 0.5);
  const Lemniscate after(1.0, 0.5);

  // ① 周长与时长都恰好翻倍（相似变换，不是"相近的另一个数"）
  EXPECT_NEAR(after.perimeter(), 2.0 * before.perimeter(), before.perimeter() * 1e-9);
  EXPECT_NEAR(after.duration(), 2.0 * before.duration(), before.duration() * 1e-9);

  // ② a = 1.0 的周长与**独立 Simpson 积分**互校 ≈6.0972 m
  const double expected = lemniscate_perimeter_simpson(1.0);
  EXPECT_NEAR(after.perimeter(), expected, expected * 1e-6);
  EXPECT_NEAR(after.perimeter(), 6.10, 0.02);

  // ③ 时长 ≈12.19 s @0.5 m/s —— 放大的是尺寸，**不是速度**
  EXPECT_NEAR(after.duration(), 12.2, 0.05);

  // ④ 采样一遍位置，钉住新规格的幅度：x 半幅 1.0、y 半幅 0.5（仍是 x 的一半 = 偏扁）
  double x_max = 0.0, y_max = 0.0;
  for (int i = 0; i <= 100000; ++i) {
    const Vec3 p = after.at(after.duration() * i / 100000.0).p;
    x_max = std::max(x_max, p.x);
    y_max = std::max(y_max, p.y);
  }
  EXPECT_NEAR(x_max, 1.0, 1e-3);
  EXPECT_NEAR(y_max, 0.5, 1e-3);
  EXPECT_NEAR(x_max / y_max, 2.0, 1e-2);
}

TEST(Lemniscate, 闭合_起点在原点_跑完停在原点)
{
  const Lemniscate lem(0.5, 0.5);
  const Reference start = lem.at(0.0);
  EXPECT_NEAR(start.p.x, 0.0, 1e-12);
  EXPECT_NEAR(start.p.y, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(start.progress, 0.0);

  const Reference end = lem.at(lem.duration());
  EXPECT_DOUBLE_EQ(end.progress, 1.0);
  EXPECT_NEAR(end.p.x, 0.0, 1e-12);  // θ=2π 与 θ=0 同一点 ⟹ 曲线闭合
  EXPECT_NEAR(end.p.y, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(norm(end.v), 0.0);

  const std::vector<Vec3> path = lem.path(200);
  EXPECT_EQ(path.size(), 201u);
  EXPECT_NEAR(path.front().x, path.back().x, 1e-12);
  EXPECT_NEAR(path.front().y, path.back().y, 1e-12);
}

/// ★★ 这条是 8 字最关键的一条测试：**位置实际走多快**（有限差分量的），
///    而不是"我报告的 v 有多大"。θ 匀速的错误会让过中心那一段冲到 ~1.4×限速（40% 偏差），
///    而报告的 |v| 仍然写着 0.5 —— 只有差分能抓到。
///    容差 1e-3（相对）比残余波纹（~1e-4，见 trajectories.hpp 的说明）宽一个数量级，
///    却比"θ 匀速"那个错（~4e-1）严 400 倍 ⟹ 分辨率足够。
TEST(Lemniscate, 有限差分证实沿弧长匀速_这就是弧长表存在的理由)
{
  const double speed = 0.5;
  const Lemniscate lem(0.5, speed);
  const double dt = 1e-5;
  double worst = 0.0;
  for (double t = 0.05; t < lem.duration() - 0.05; t += 0.01) {
    const Vec3 ahead = lem.at(t + dt).p;
    const Vec3 behind = lem.at(t - dt).p;
    const double fd_speed = std::hypot(ahead.x - behind.x, ahead.y - behind.y) / (2.0 * dt);
    worst = std::max(worst, std::fabs(fd_speed - speed));
    EXPECT_NEAR(fd_speed, speed, speed * 1e-3) << "t = " << t;
  }
  // 顺带把残余波纹的**上界**钉住：它应该远小于 1e-3 量级，加密表就该变小。
  // （万一哪天有人把 table_size 调回去，这条会立刻说话。）
  EXPECT_LT(worst, speed * 5e-4) << "速度波纹 " << worst << " m/s，比预期大 —— 弧长表太稀了？";
}

TEST(Lemniscate, 报告的_速度大小恒等于限速_方向与差分一致)
{
  const double speed = 0.5;
  const Lemniscate lem(0.5, speed);
  const double dt = 1e-5;
  for (double t = 0.05; t < lem.duration() - 0.05; t += 0.05) {
    const Reference r = lem.at(t);
    EXPECT_NEAR(norm(r.v), speed, speed * 1e-9);
    // 方向也要对：否则"位置对了、前馈方向错了"会在控制侧变成持续的抗风
    const Vec3 ahead = lem.at(t + dt).p;
    const Vec3 behind = lem.at(t - dt).p;
    const double fx = (ahead.x - behind.x) / (2.0 * dt);
    const double fy = (ahead.y - behind.y) / (2.0 * dt);
    const double fn = std::hypot(fx, fy);
    EXPECT_NEAR(r.v.x / speed, fx / fn, 1e-3) << "t = " << t;
    EXPECT_NEAR(r.v.y / speed, fy / fn, 1e-3) << "t = " << t;
  }
}

TEST(Lemniscate, 完成度单调不减且首尾为_0_与_1)
{
  const Lemniscate lem(0.5, 0.5);
  double prev = -1.0;
  for (int i = 0; i <= 5000; ++i) {
    const double p = lem.at(lem.duration() * i / 5000.0).progress;
    EXPECT_GE(p, prev);
    EXPECT_GE(p, 0.0);
    EXPECT_LE(p, 1.0);
    prev = p;
  }
  EXPECT_DOUBLE_EQ(prev, 1.0);
}

// ——————————————————————————————————————————————————————————————
// ④ 起飞：从轨迹原点匀速升到 takeoff_height_m，到顶后停在那儿
// ——————————————————————————————————————————————————————————————

/// 斜坡的**形状**：z(0) = 0、到顶那一刻正好是 h、之后恒为 h；v 在斜坡内是 +v、到顶后归 0。
TEST(Takeoff, 斜坡_从原点升到高度_到顶后停住)
{
  const traj::Takeoff to(60.0, 0.2, 0.2);
  EXPECT_DOUBLE_EQ(to.ramp_duration(), 1.0);       // 0.2 m @ 0.2 m/s
  EXPECT_DOUBLE_EQ(to.duration(), 60.0);           // 时长沿用 hover_duration_s

  EXPECT_DOUBLE_EQ(to.at(0.0).p.z, 0.0);           // ★ 零点 = 轨迹原点 —— "起飞"的物理含义
  EXPECT_DOUBLE_EQ(to.at(0.0).p.x, 0.0);
  EXPECT_DOUBLE_EQ(to.at(0.0).p.y, 0.0);
  EXPECT_NEAR(to.at(0.5).p.z, 0.1, 1e-12);         // 斜坡中点 = 一半
  EXPECT_DOUBLE_EQ(to.at(1.0).p.z, 0.2);           // 到顶
  EXPECT_DOUBLE_EQ(to.at(30.0).p.z, 0.2);          // 到顶之后停在那儿
  EXPECT_DOUBLE_EQ(to.at(600.0).p.z, 0.2);

  EXPECT_DOUBLE_EQ(to.at(0.5).v.z, 0.2);           // 斜坡内 = 爬升速率
  EXPECT_DOUBLE_EQ(to.at(1.0).v.z, 0.0);           // 到顶：速度归零
  EXPECT_DOUBLE_EQ(to.at(30.0).v.z, 0.0);
  EXPECT_DOUBLE_EQ(to.at(0.5).v.x, 0.0);           // 水平两轴恒为 0（起飞不横着走）
  EXPECT_DOUBLE_EQ(to.at(30.0).v.y, 0.0);
}

/// ★ 用**有限差分**独立量一遍"位置实际走多快"（不是读它自己报告的 v）—— 与 Lemniscate
/// 那条同一把尺：报告的 v 与位置差分不一致时，只有差分能看出来。
TEST(Takeoff, 有限差分证实斜坡内匀速_到顶后完全停住)
{
  const traj::Takeoff to(60.0, 0.2, 0.2);
  const double dt = 1e-5;
  for (double t = 1e-3; t < 0.999; t += 0.01) {
    const double dz = (to.at(t + dt).p.z - to.at(t).p.z) / dt;
    EXPECT_NEAR(dz, 0.2, 1e-9) << "t = " << t;     // 斜坡内：实际爬升速率 = takeoff_speed_ms
  }
  for (double t = 1.0; t <= 60.0; t += 1.0) {
    const double dz = (to.at(t + dt).p.z - to.at(t).p.z) / dt;
    EXPECT_DOUBLE_EQ(dz, 0.0) << "t = " << t;      // 到顶之后：一点都没动
  }
}

/// 完成度**按时长**走（不是"爬到顶就算 1"）。★ 这条不是形式主义：`control` 那条
/// `progress ≥ 1 ⟹ LAND` 就压在这上面 —— 若做成"斜坡走完 = 1"，20 cm 一起飞就被判成跑完了。
TEST(Takeoff, 完成度按时长走而不是按斜坡走)
{
  const traj::Takeoff to(60.0, 0.2, 0.2);
  EXPECT_DOUBLE_EQ(to.at(0.0).progress, 0.0);
  EXPECT_NEAR(to.at(30.0).progress, 0.5, 1e-12);
  EXPECT_NEAR(to.at(1.0).progress, 1.0 / 60.0, 1e-12);   // 到顶那一刻：才走了 1/60
  EXPECT_DOUBLE_EQ(to.at(60.0).progress, 1.0);
  EXPECT_DOUBLE_EQ(to.at(600.0).progress, 1.0);          // 跑完保持 1
  EXPECT_DOUBLE_EQ(to.at(-1.0).progress, 0.0);           // t < 0 夹到 0
  EXPECT_DOUBLE_EQ(to.at(-1.0).p.z, 0.0);
}

/// 几何采样 = 起止两点（RViz 画一条竖线）。按几何采样，`points` 对它无意义。
TEST(Takeoff, 几何采样是起止两点)
{
  const traj::Takeoff to(60.0, 0.2, 0.2);
  const std::vector<Vec3> path = to.path(201);
  ASSERT_EQ(path.size(), 2u);
  EXPECT_DOUBLE_EQ(path[0].x, 0.0);
  EXPECT_DOUBLE_EQ(path[0].y, 0.0);
  EXPECT_DOUBLE_EQ(path[0].z, 0.0);
  EXPECT_DOUBLE_EQ(path[1].z, 0.2);
}

/// 退化与负值：h = 0（原地不动）、h < 0（往下走，符号跟着走）、时长/速度非正（夹住）
/// —— 都不许崩、不许 NaN、`duration()` 必须 > 0。
TEST(Takeoff, 零高度与负高度与非法速度都不崩)
{
  const traj::Takeoff flat(10.0, 0.0, 0.2);
  EXPECT_DOUBLE_EQ(flat.ramp_duration(), 0.0);
  EXPECT_DOUBLE_EQ(flat.at(5.0).p.z, 0.0);
  EXPECT_DOUBLE_EQ(flat.at(5.0).v.z, 0.0);

  const traj::Takeoff down(10.0, -0.2, 0.2);       // 负高度 = 往下
  EXPECT_DOUBLE_EQ(down.ramp_duration(), 1.0);
  EXPECT_DOUBLE_EQ(down.at(0.5).v.z, -0.2);
  EXPECT_NEAR(down.at(1.0).p.z, -0.2, 1e-12);

  const traj::Takeoff bad(0.0, 0.2, 0.0);
  EXPECT_GT(bad.duration(), 0.0);
  EXPECT_GT(bad.ramp_duration(), 0.0);
  EXPECT_TRUE(std::isfinite(bad.at(0.5).p.z));
  EXPECT_TRUE(std::isfinite(bad.at(0.5).v.z));
  EXPECT_TRUE(std::isfinite(bad.at(0.5).progress));
}

// ——————————————————————————————————————————————————————————————
// 四条轨迹共有的不变量
// ——————————————————————————————————————————————————————————————

/// §5.2 的硬要求：任一时刻速度都不超过限速。四条轨迹一起扫。
/// ★ 轨迹 4（起飞）在这里靠的是它**自己的** `takeoff_speed_ms`（0.2 ≤ 0.5）：爬升速率也是一条速度，
///   同样受这条约束 —— 节点侧对 "takeoff_speed_ms > max_speed_ms" 会告警（`warn_about_parameters`）。
TEST(限速, 四条轨迹全程不超过限速)
{
  const double speed = 0.5;
  const Params params;  // 默认参数：0.5 m/s
  ASSERT_DOUBLE_EQ(params.max_speed_ms, speed);

  for (const Kind kind : {Kind::kHover, Kind::kSquare, Kind::kLemniscate, Kind::kTakeoff}) {
    const std::unique_ptr<traj::Trajectory> tr = traj::make(kind, params);
    ASSERT_NE(tr, nullptr);
    const int n = 20000;
    for (int i = 0; i <= n; ++i) {
      const double t = tr->duration() * i / n;
      const Reference r = tr->at(t);
      EXPECT_LE(norm(r.v), speed + 1e-12)
        << traj::to_string(kind) << " 在 t = " << t << " 超速";
      EXPECT_GE(norm(r.v), 0.0);
      EXPECT_FALSE(std::isnan(r.p.x) || std::isnan(r.p.y) || std::isnan(r.p.z));
      EXPECT_FALSE(std::isnan(r.v.x) || std::isnan(r.progress));
      EXPECT_GE(r.progress, 0.0);
      EXPECT_LE(r.progress, 1.0);
    }
  }
}

/// 时长与限速成反比（"t 的推进速率按 max_speed 缩放"这条验收判据的直接形式）。
TEST(限速, 时长与限速成反比)
{
  for (const Kind kind : {Kind::kHover, Kind::kSquare, Kind::kLemniscate}) {
    Params slow;
    slow.max_speed_ms = 0.25;
    Params fast;
    fast.max_speed_ms = 0.5;
    if (kind == Kind::kHover) {
      continue;  // ★ 悬停的时长由 hover_duration_s 决定，与限速无关（它没有弧长）
    }
    const std::unique_ptr<traj::Trajectory> a = traj::make(kind, slow);
    const std::unique_ptr<traj::Trajectory> b = traj::make(kind, fast);
    EXPECT_NEAR(a->duration(), 2.0 * b->duration(), a->duration() * 1e-9)
      << traj::to_string(kind);
  }
}

TEST(参数, 非法参数被夹住而不是产生_nan_或零时长)
{
  // 非正边长/幅度/限速/时长，全都不能崩、不能 NaN、duration 必须 > 0
  const Params bad{
    /*max_speed_ms=*/0.0,
    /*hover_duration_s=*/0.0,
    /*square_side_m=*/-1.0,
    /*lemniscate_a_m=*/0.0,
    /*height_offset_m=*/0.0};
  for (const Kind kind : {Kind::kHover, Kind::kSquare, Kind::kLemniscate, Kind::kTakeoff}) {
    const std::unique_ptr<traj::Trajectory> tr = traj::make(kind, bad);
    ASSERT_NE(tr, nullptr);
    EXPECT_GT(tr->duration(), 0.0) << traj::to_string(kind);
    const Reference r = tr->at(tr->duration() * 0.5);
    EXPECT_TRUE(std::isfinite(r.p.x) && std::isfinite(r.p.y) && std::isfinite(r.p.z));
    EXPECT_TRUE(std::isfinite(r.v.x) && std::isfinite(r.progress));
    EXPECT_GE(r.progress, 0.0);
    EXPECT_LE(r.progress, 1.0);
  }
  // 负的限速取绝对值意义上的"夹住"后仍须有限
  const Lemniscate neg(-1.0, -1.0);
  EXPECT_TRUE(std::isfinite(neg.duration()));
  EXPECT_GT(neg.duration(), 0.0);
  EXPECT_TRUE(std::isfinite(neg.at(0.1).p.x));
}

// ——————————————————————————————————————————————————————————————
// 编号（`/traj/select` 的取值口径）
// ——————————————————————————————————————————————————————————————

TEST(编号, 1_2_3_4_对应四条轨迹)
{
  Kind k{};
  ASSERT_TRUE(traj::parse_kind(1, k));
  EXPECT_EQ(k, Kind::kHover);
  ASSERT_TRUE(traj::parse_kind(2, k));
  EXPECT_EQ(k, Kind::kSquare);
  ASSERT_TRUE(traj::parse_kind(3, k));
  EXPECT_EQ(k, Kind::kLemniscate);
  ASSERT_TRUE(traj::parse_kind(4, k));      // ★ 2026-10-16：起飞
  EXPECT_EQ(k, Kind::kTakeoff);

  EXPECT_STREQ(traj::to_string(Kind::kHover), "hover");
  EXPECT_STREQ(traj::to_string(Kind::kSquare), "square");
  EXPECT_STREQ(traj::to_string(Kind::kLemniscate), "lemniscate");
  EXPECT_STREQ(traj::to_string(Kind::kTakeoff), "takeoff");
  EXPECT_EQ(static_cast<unsigned char>(Kind::kHover), 1u);  // ★ 数值就是话题上的数字
  EXPECT_EQ(static_cast<unsigned char>(Kind::kSquare), 2u);
  EXPECT_EQ(static_cast<unsigned char>(Kind::kLemniscate), 3u);
  EXPECT_EQ(static_cast<unsigned char>(Kind::kTakeoff), 4u);  // ★ 不是 0：0 是"清空"
}

TEST(编号, 0_与越界值被拒绝)
{
  Kind k{};
  EXPECT_FALSE(traj::parse_kind(0, k));  // 0 在节点侧另有含义：清空（★ 起飞因此取 4）
  EXPECT_FALSE(traj::parse_kind(5, k));
  EXPECT_FALSE(traj::parse_kind(255, k));
}

TEST(工厂, make_对每个编号都返回对应类型与高度偏移)
{
  Params p;
  p.height_offset_m = 0.4;
  for (const Kind kind : {Kind::kHover, Kind::kSquare, Kind::kLemniscate}) {
    const std::unique_ptr<traj::Trajectory> tr = traj::make(kind, p);
    ASSERT_NE(tr, nullptr);
    EXPECT_EQ(tr->kind(), kind);
    EXPECT_DOUBLE_EQ(tr->at(0.0).p.z, 0.4) << traj::to_string(kind);
    EXPECT_DOUBLE_EQ(tr->at(tr->duration()).progress, 1.0);
  }
  // ★★ 轨迹 4（起飞）**不参与** height_offset_m：它是"从轨迹原点往上爬"，
  //   零点必须是 0（否则"起飞 20 cm"这句话就不成立了 —— 会变成 20 cm + 偏移）。
  const std::unique_ptr<traj::Trajectory> to = traj::make(Kind::kTakeoff, p);
  ASSERT_NE(to, nullptr);
  EXPECT_EQ(to->kind(), Kind::kTakeoff);
  EXPECT_DOUBLE_EQ(to->at(0.0).p.z, 0.0) << "起飞必须从轨迹原点出发，不能带 height_offset_m";
  EXPECT_DOUBLE_EQ(to->at(to->duration()).p.z, p.takeoff_height_m);
  EXPECT_DOUBLE_EQ(to->at(to->duration()).progress, 1.0);
}

}  // namespace
