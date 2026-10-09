// seam 2 —— 相对位姿四个纯函数的单测。★ 零 ROS 依赖（本文件不 include 任何 ROS 头）。
//
// 为什么值得有这组测试：位姿处理错误**在话题层面没有诊断分辨率** —— 飞机往反方向飞时，
// 话题上分不出是原点锁错、yaw 归零错，还是 mavros 的边界转换不如预期。
//
// ★ 测试侧走**完全独立的另一条路**：构造输入与断言都走旋转矩阵（欧拉角 → 矩阵连乘 →
//   Shepperd 法转四元数 → 被测函数 → 四元数转矩阵 → ZYX 反解），**一次都不调用被测的
//   `normalize` / `quat_mul` / `yaw_of`**（只有待测函数本身是黑盒）。这样四元数乘法写反序、
//   归一化写错、yaw 符号写反这三类错误都会被抓住，而不是"用同一段错代码验证自己"。

#include <cmath>

#include "gtest/gtest.h"
#include "mocap/mocap_rel_pose.hpp"

namespace
{

constexpr double kDeg = M_PI / 180.0;

using mocap::Quat;
using mocap::Vec3;

struct Mat3
{
  double m[3][3];
};

Mat3 mat_mul(const Mat3 & A, const Mat3 & B)
{
  Mat3 out{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out.m[i][j] = A.m[i][0] * B.m[0][j] + A.m[i][1] * B.m[1][j] + A.m[i][2] * B.m[2][j];
    }
  }
  return out;
}

/// 四元数 → 旋转矩阵（测试侧独立实现；自带 1/|q|² 归一化，不调被测的 `normalize`）。
Mat3 rot(const Quat & q)
{
  const double x = q.x, y = q.y, z = q.z, w = q.w;
  const double n2 = x * x + y * y + z * z + w * w;
  return Mat3{{
    {1 - 2 * (y * y + z * z) / n2, 2 * (x * y - w * z) / n2, 2 * (x * z + w * y) / n2},
    {2 * (x * y + w * z) / n2, 1 - 2 * (x * x + z * z) / n2, 2 * (y * z - w * x) / n2},
    {2 * (x * z - w * y) / n2, 2 * (y * z + w * x) / n2, 1 - 2 * (x * x + y * y) / n2}}};
}

/// 旋转矩阵 → 四元数（Shepperd 法，测试侧独立实现）。
Quat quat_of(const Mat3 & R)
{
  const double t = R.m[0][0] + R.m[1][1] + R.m[2][2];
  if (t > 0.0) {
    const double s = std::sqrt(t + 1.0) * 2.0;  // = 4w
    return Quat{
      (R.m[2][1] - R.m[1][2]) / s, (R.m[0][2] - R.m[2][0]) / s,
      (R.m[1][0] - R.m[0][1]) / s, 0.25 * s};
  }
  if (R.m[0][0] > R.m[1][1] && R.m[0][0] > R.m[2][2]) {
    const double s = std::sqrt(1.0 + R.m[0][0] - R.m[1][1] - R.m[2][2]) * 2.0;  // = 4x
    return Quat{
      0.25 * s, (R.m[0][1] + R.m[1][0]) / s, (R.m[0][2] + R.m[2][0]) / s,
      (R.m[2][1] - R.m[1][2]) / s};
  }
  if (R.m[1][1] > R.m[2][2]) {
    const double s = std::sqrt(1.0 + R.m[1][1] - R.m[0][0] - R.m[2][2]) * 2.0;  // = 4y
    return Quat{
      (R.m[0][1] + R.m[1][0]) / s, 0.25 * s, (R.m[1][2] + R.m[2][1]) / s,
      (R.m[0][2] - R.m[2][0]) / s};
  }
  const double s = std::sqrt(1.0 + R.m[2][2] - R.m[0][0] - R.m[1][1]) * 2.0;  // = 4z
  return Quat{
    (R.m[0][2] + R.m[2][0]) / s, (R.m[1][2] + R.m[2][1]) / s, 0.25 * s,
    (R.m[1][0] - R.m[0][1]) / s};
}

struct Rpy
{
  double roll, pitch, yaw;
};

/// 旋转矩阵 → ZYX 欧拉角（测试侧独立实现）。
Rpy rpy_of(const Mat3 & R)
{
  Rpy out{};
  out.roll = std::atan2(R.m[2][1], R.m[2][2]);
  out.pitch = std::atan2(-R.m[2][0], std::sqrt(R.m[0][0] * R.m[0][0] + R.m[1][0] * R.m[1][0]));
  out.yaw = std::atan2(R.m[1][0], R.m[0][0]);
  return out;
}

/// 由 ZYX 欧拉角构造四元数：R = Rz(yaw)·Ry(pitch)·Rx(roll)，再转四元数。
/// ★ 走矩阵连乘，不调被测的 `quat_mul` —— 所以被测的乘法若反序/符号错，这里会立刻对不上。
Quat quat_zyx(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const Mat3 rz{{cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0}};
  const Mat3 ry{{cp, 0.0, sp, 0.0, 1.0, 0.0, -sp, 0.0, cp}};
  const Mat3 rx{{1.0, 0.0, 0.0, 0.0, cr, -sr, 0.0, sr, cr}};
  return quat_of(mat_mul(mat_mul(rz, ry), rx));
}

/// 机体系 z 轴（推力方向）在世界系中的表示。
Vec3 body_z_in_world(const Quat & q) {return Vec3{rot(q).m[0][2], rot(q).m[1][2], rot(q).m[2][2]};}

// ———————————————————————————————————————————————————————————————
// ⓪ 刚体原点 → IMU 中心（动捕外参）
//    ★ 这一节挡的是一类**静默**错误：外参填 0 / 符号反 / 写成"固定减 z"，在静止与水平飞行里
//      完全看不出来（EKF2 的原点初始化会把常值部分吸收掉），只有倾斜飞行时才变成位置误差。
// ———————————————————————————————————————————————————————————————

namespace
{

/// z0 的外参：IMU 在刚体原点**下方** 80 mm（机体系 FLU：x 前 / y 左 / z **上** ⟹ z 为负）。
/// 出处：2026-10-09 操作者给定；推导与"每场次要复测"见 .scratch/phase4/issues/01-…。
constexpr double kImuBelow = 0.080;
const Vec3 kRImuInBody{0.0, 0.0, -kImuBelow};

}  // namespace

TEST(Rotate, 与独立实现的旋转矩阵逐元素一致)
{
  // `mocap::rotate` 是新加的底层算子（四元数转矢量）。写反序 / 共轭——这类错会静默地
  // 把偏移转到镜像方向，所以这里拿测试侧**独立的** `rot()` 矩阵逐元素对。
  const Vec3 vs[] = {
    Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0},
    Vec3{0.3, -0.7, 0.25}, Vec3{0.0, 0.0, -kImuBelow}};
  const Quat qs[] = {
    quat_zyx(0.0, 0.0, 0.0), quat_zyx(35 * kDeg, 20 * kDeg, -110 * kDeg),
    quat_zyx(-25 * kDeg, -40 * kDeg, 120 * kDeg), quat_zyx(0.0, 90 * kDeg, 0.0)};
  for (const Quat & q : qs) {
    for (const Vec3 & v : vs) {
      const Mat3 R = rot(q);
      const Vec3 got = mocap::rotate(q, v);
      const double want[3] = {
        R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z,
        R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z,
        R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z};
      EXPECT_NEAR(got.x, want[0], 1e-12);
      EXPECT_NEAR(got.y, want[1], 1e-12);
      EXPECT_NEAR(got.z, want[2], 1e-12);
    }
  }
}

TEST(RigidBodyToImu, 外参全零时逐位恒等_保住改动前的行为)
{
  // 逃生口：把三个参数都填 0 ⟹ 输出必须**逐位**等于输入（否则"关掉外参"这句话不成立）。
  const mocap::Pose in{Vec3{-1.28261, -0.13292, 0.10850}, quat_zyx(3 * kDeg, 4 * kDeg, 5 * kDeg)};
  const mocap::Pose out = mocap::rigid_body_to_imu(in, Vec3{0.0, 0.0, 0.0});
  EXPECT_DOUBLE_EQ(out.p.x, in.p.x);
  EXPECT_DOUBLE_EQ(out.p.y, in.p.y);
  EXPECT_DOUBLE_EQ(out.p.z, in.p.z);
  EXPECT_DOUBLE_EQ(out.q.x, in.q.x);
  EXPECT_DOUBLE_EQ(out.q.w, in.q.w);
}

TEST(RigidBodyToImu, 水平时只动z且朝下)
{
  // 用 bag/1_static 那场的实测静止读数（刚体原点 = 地面上方 108.5 mm）当输入。
  const Vec3 p{-1.28261, -0.13292, 0.10850};
  const Quat q = quat_zyx(0.0, 0.0, 0.0);   // 平放（实测 roll/pitch < 1°）
  const mocap::Pose out = mocap::rigid_body_to_imu(mocap::Pose{p, q}, kRImuInBody);

  EXPECT_DOUBLE_EQ(out.p.x, p.x);
  EXPECT_DOUBLE_EQ(out.p.y, p.y);
  EXPECT_LT(out.p.z - p.z, 0.0) << "★ IMU 在刚体原点**下方** ⟹ 差值是负的（符号反了就在这里红）";
  EXPECT_NEAR(out.p.z - p.z, -0.080, 1e-12);
  EXPECT_NEAR(out.p.z, 0.02850, 1e-12);
  // 姿态**逐位**不变（平移不改变朝向）
  EXPECT_DOUBLE_EQ(out.q.x, q.x);
  EXPECT_DOUBLE_EQ(out.q.y, q.y);
  EXPECT_DOUBLE_EQ(out.q.z, q.z);
  EXPECT_DOUBLE_EQ(out.q.w, q.w);
}

TEST(RigidBodyToImu, 倾斜时按姿态旋转_而不是固定减_z)
{
  const Vec3 p{0.4, -0.2, 0.5};
  const Quat qs[] = {
    quat_zyx(35 * kDeg, 20 * kDeg, 0.0),
    quat_zyx(-25 * kDeg, -40 * kDeg, 120 * kDeg)};

  for (const Quat & q : qs) {
    const mocap::Pose out = mocap::rigid_body_to_imu(mocap::Pose{p, q}, kRImuInBody);
    const Mat3 R = rot(q);
    // 期望 = R·(0,0,−0.080)，用测试侧独立矩阵算（第 3 列 = 机体 z 轴在世界系中的表示）
    const double want[3] = {R.m[0][2] * -kImuBelow, R.m[1][2] * -kImuBelow, R.m[2][2] * -kImuBelow};
    EXPECT_NEAR(out.p.x - p.x, want[0], 1e-12);
    EXPECT_NEAR(out.p.y - p.y, want[1], 1e-12);
    EXPECT_NEAR(out.p.z - p.z, want[2], 1e-12);

    // ★ 鉴别力：上面三条只有在"带姿态"时才可能同时成立；这条额外钉住**这个测试能区分两种实现**——
    //   若实现退化成"固定减 z"（水平分量恒为 0），下面这条立刻红。
    EXPECT_GT(std::hypot(out.p.x - p.x, out.p.y - p.y), 0.02)
      << "倾斜 ≥ 25° 时水平补偿应有 80·sinθ ≈ 34 mm；为 0 说明实现退化成固定减 z";
    EXPECT_GT(std::fabs((out.p.z - p.z) + kImuBelow), 1e-3)
      << "竖直分量也不再是 −0.080（固定减 z 会给出恰好 −0.080 ⟹ 这条就是它的判据）";
  }
}

TEST(RigidBodyToImu, 与两阶段原点串起来_输出是IMU点的相对位移)
{
  // 把「外参 → 原点采样 → 相对位姿」按节点里的真实顺序串起来跑一遍：
  // 解锁时平放并锁原点，之后飞机抬起并倾斜 ⟹ 输出的水平分量应当等于**IMU 点的**位移，
  // 而不是刚体原点的位移（两者只差 R(q1)·r − R(q0)·r）。
  const Vec3 p0{-1.2826, -0.1329, 0.1085};
  const Quat q0 = quat_zyx(0.0, 0.0, 0.0);
  const Vec3 p1{p0.x + 0.10, p0.y - 0.05, p0.z + 0.30};
  const Quat q1 = quat_zyx(20 * kDeg, 15 * kDeg, 0.0);

  mocap::TwoStageOrigin sel(3, 3);
  for (int i = 0; i < 3; ++i) {
    sel.observe(mocap::rigid_body_to_imu(mocap::Pose{p0, q0}, kRImuInBody),
      mocap::ArmedState::Disarmed);
  }
  ASSERT_TRUE(sel.ready());
  for (int i = 0; i < 3; ++i) {
    sel.observe(mocap::rigid_body_to_imu(mocap::Pose{p0, q0}, kRImuInBody),
      mocap::ArmedState::Armed);
  }
  ASSERT_TRUE(sel.switched());

  const mocap::Pose out = sel.apply(mocap::rigid_body_to_imu(mocap::Pose{p1, q1}, kRImuInBody));
  const Mat3 R1 = rot(q1);
  EXPECT_NEAR(out.p.x, (p1.x - p0.x) + R1.m[0][2] * -kImuBelow, 1e-12);
  EXPECT_NEAR(out.p.y, (p1.y - p0.y) + R1.m[1][2] * -kImuBelow, 1e-12);
  EXPECT_NEAR(out.p.z, (p1.z - p0.z) + (R1.m[2][2] - 1.0) * -kImuBelow, 1e-12);
  // 不施加外参时，倾斜带来的这部分（≈ 26 mm 水平）会**完全没有** —— 这条就是它的鉴别力。
  const mocap::Pose naive = sel.apply(mocap::Pose{p1, q1});
  EXPECT_GT(std::hypot(out.p.x - naive.p.x, out.p.y - naive.p.y), 0.02);
}

// ———————————————————————————————————————————————————————————————
// ① 原点减法：逐轴独立，且与姿态处理无关
// ———————————————————————————————————————————————————————————————

TEST(RelativePosition, 减去锁定原点)
{
  // 用 ticket 12 的实测静止读数当原点：x = −1.28 m，动捕原点在棚里别处。
  const Vec3 origin{-1.2804, -0.1397, 0.0883};
  const Vec3 p{-0.7804, -0.4397, 0.3883};  // 从原点往 +x 0.5 m、往 −y 0.3 m、往 +z 0.3 m
  const Vec3 rel = mocap::relative_position(p, origin);
  EXPECT_NEAR(rel.x, 0.5, 1e-12);
  EXPECT_NEAR(rel.y, -0.3, 1e-12);
  EXPECT_NEAR(rel.z, 0.3, 1e-12);
}

TEST(RelativePosition, 静止在原点时输出零)
{
  const Vec3 origin{-1.2804, -0.1397, 0.0883};
  const Vec3 rel = mocap::relative_position(origin, origin);
  EXPECT_DOUBLE_EQ(rel.x, 0.0);
  EXPECT_DOUBLE_EQ(rel.y, 0.0);
  EXPECT_DOUBLE_EQ(rel.z, 0.0);
}

TEST(RelativePosition, 逐轴独立_只动原点的一轴只影响对应输出轴)
{
  const Vec3 p{1.0, 2.0, 3.0};
  const Vec3 base{-1.0, -2.0, -3.0};
  const Vec3 a = mocap::relative_position(p, base);
  const Vec3 b = mocap::relative_position(p, Vec3{base.x + 0.7, base.y, base.z});
  EXPECT_NEAR(b.x - a.x, -0.7, 1e-12);
  EXPECT_DOUBLE_EQ(b.y, a.y);
  EXPECT_DOUBLE_EQ(b.z, a.z);
}

TEST(RelativePosition, 位置输出不受姿态影响)
{
  // 同一位置配三种差别很大的姿态 ⟹ 输出位置必须**逐位相同**。
  mocap::TwoStageOrigin sel(3, 3);
  const Vec3 p{-0.5, 0.25, 0.1};
  for (int i = 0; i < 3; ++i) {
    sel.observe(mocap::Pose{p, quat_zyx(0.0, 0.0, 0.0)}, mocap::ArmedState::Disarmed);
  }
  ASSERT_TRUE(sel.ready());

  const mocap::Pose flat{p, quat_zyx(0.0, 0.0, 0.0)};
  const mocap::Pose tilted{p, quat_zyx(35 * kDeg, -20 * kDeg, 120 * kDeg)};
  const mocap::Pose wild{p, quat_zyx(-170 * kDeg, 80 * kDeg, -95 * kDeg)};
  const Vec3 r0 = sel.apply(flat).p;
  const Vec3 r1 = sel.apply(tilted).p;
  const Vec3 r2 = sel.apply(wild).p;
  EXPECT_DOUBLE_EQ(r0.x, r1.x);
  EXPECT_DOUBLE_EQ(r0.x, r2.x);
  EXPECT_DOUBLE_EQ(r0.y, r1.y);
  EXPECT_DOUBLE_EQ(r0.y, r2.y);
  EXPECT_DOUBLE_EQ(r0.z, r1.z);
  EXPECT_DOUBLE_EQ(r0.z, r2.z);
}

// ———————————————————————————————————————————————————————————————
// ② 仅归零 yaw：roll / pitch 保持绝对
// ———————————————————————————————————————————————————————————————

TEST(YawZeroed, 只减掉_yaw_roll_pitch_原样)
{
  const double roll = 5 * kDeg, pitch = 10 * kDeg, yaw = 30 * kDeg;
  const Quat q = quat_zyx(roll, pitch, yaw);
  const Quat q_rel = mocap::yaw_zeroed(q, yaw);
  const Rpy out = rpy_of(rot(q_rel));
  EXPECT_NEAR(out.roll, roll, 1e-12);
  EXPECT_NEAR(out.pitch, pitch, 1e-12);
  EXPECT_NEAR(out.yaw, 0.0, 1e-12);
}

TEST(YawZeroed, 归零后机体z轴的竖直分量不变)
{
  // "机体 z 轴方向不变"的准确含义：z 轴绕世界 z 转了一下，但**与竖直方向的夹角完全不变**
  // ⟹ 推力方向的竖直分量不被人为改变。若整个姿态做相对，这一条会立刻被破坏。
  const double yaw = -72 * kDeg;
  const Quat q = quat_zyx(12 * kDeg, -7 * kDeg, yaw);
  const Quat q_rel = mocap::yaw_zeroed(q, yaw);
  const Vec3 z_before = body_z_in_world(q);
  const Vec3 z_after = body_z_in_world(q_rel);
  EXPECT_NEAR(z_after.z, z_before.z, 1e-12);
  // 水平分量：恰好被绕世界 z 转过 −yaw
  const double dz = std::atan2(z_after.y, z_after.x) - std::atan2(z_before.y, z_before.x);
  EXPECT_NEAR(std::remainder(dz, 2 * M_PI), -yaw, 1e-12);
}

TEST(YawZeroed, 零偏航归零是恒等变换)
{
  const Quat q = quat_zyx(3 * kDeg, 4 * kDeg, 5 * kDeg);
  const Quat q_rel = mocap::yaw_zeroed(q, 0.0);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(rot(q_rel).m[i][j], rot(q).m[i][j], 1e-12);
    }
  }
}

TEST(YawZeroed, 归零的量就是该帧的_yaw_与角度回卷无关)
{
  const double yaw = 179 * kDeg;  // ★ 贴近 ±180°，容易踩回卷的坑
  const Quat q = quat_zyx(0.0, 0.0, yaw);
  const Quat q_rel = mocap::yaw_zeroed(q, mocap::yaw_of(q));
  EXPECT_NEAR(mocap::yaw_of(q_rel), 0.0, 1e-12);
}

TEST(YawZeroed, 单测里也钉一下_yaw_of_本身)
{
  EXPECT_NEAR(mocap::yaw_of(quat_zyx(0.0, 0.0, 90 * kDeg)), 90 * kDeg, 1e-12);
  EXPECT_NEAR(mocap::yaw_of(quat_zyx(0.0, 0.0, -150 * kDeg)), -150 * kDeg, 1e-12);
  // yaw_of 不受 roll/pitch 影响
  EXPECT_NEAR(mocap::yaw_of(quat_zyx(40 * kDeg, -25 * kDeg, 60 * kDeg)), 60 * kDeg, 1e-12);
}

// ———————————————————————————————————————————————————————————————
// ③ 两阶段原点切换
// ———————————————————————————————————————————————————————————————

namespace
{

const Vec3 kPBoot{-1.2804, -0.1397, 0.0883};             ///< 节点启动时的静止读数（ticket 12 实测）
const Quat kQBoot = quat_zyx(1 * kDeg, -2 * kDeg, 37 * kDeg);

void feed(
  mocap::TwoStageOrigin & sel, const Vec3 & p, const Quat & q,
  mocap::ArmedState armed_state)
{
  sel.observe(mocap::Pose{p, q}, armed_state);
}

/// 阶段 A：启动采样，未解锁。
void feed_boot(mocap::TwoStageOrigin & sel, int n, const Vec3 & p = kPBoot, const Quat & q = kQBoot)
{
  for (int i = 0; i < n; ++i) {
    feed(sel, p, q, mocap::ArmedState::Disarmed);
  }
}

/// 阶段 B：解锁上升沿后的静止采样窗。
void feed_arm(mocap::TwoStageOrigin & sel, int n, const Vec3 & p, const Quat & q)
{
  for (int i = 0; i < n; ++i) {
    feed(sel, p, q, mocap::ArmedState::Armed);
  }
}

}  // namespace

TEST(TwoStageOrigin, 阶段A锁定启动原点且不归零yaw)
{
  mocap::TwoStageOrigin sel(5, 5);
  EXPECT_FALSE(sel.ready());
  feed_boot(sel, 4);
  EXPECT_FALSE(sel.ready()) << "采样没满就不该锁定（宁可不发，也不要拿默认原点糊）";
  feed_boot(sel, 1);
  ASSERT_TRUE(sel.ready());
  EXPECT_EQ(sel.stage(), mocap::OriginStage::Boot);
  EXPECT_FALSE(sel.switched());
  EXPECT_NEAR(sel.origin().p.x, kPBoot.x, 1e-12);

  // 阶段 A：位置相对化，姿态**原样**（连 yaw 都不减）
  const mocap::Pose out = sel.apply(mocap::Pose{kPBoot, kQBoot});
  EXPECT_NEAR(out.p.x, 0.0, 1e-12);
  EXPECT_NEAR(out.q.x, kQBoot.x, 1e-12);
  EXPECT_NEAR(out.q.z, kQBoot.z, 1e-12);
  EXPECT_NEAR(mocap::yaw_of(out.q), mocap::yaw_of(kQBoot), 1e-12);
}

TEST(TwoStageOrigin, 启动原点是采样窗的均值而不是第一帧)
{
  mocap::TwoStageOrigin sel(3, 3);
  feed(sel, Vec3{0.0, 0.0, 0.0}, kQBoot, mocap::ArmedState::Disarmed);
  feed(sel, Vec3{0.2, 0.0, 0.0}, kQBoot, mocap::ArmedState::Disarmed);
  feed(sel, Vec3{0.4, 0.0, 0.0}, kQBoot, mocap::ArmedState::Disarmed);
  ASSERT_TRUE(sel.ready());
  EXPECT_NEAR(sel.origin().p.x, 0.2, 1e-12);
}

TEST(TwoStageOrigin, 未解锁时绝不提前锁起飞前原点)
{
  mocap::TwoStageOrigin sel(3, 3);
  feed_boot(sel, 3);
  feed_boot(sel, 50, Vec3{kPBoot.x + 0.3, kPBoot.y, kPBoot.z});  // 有人把它挪了 0.3 m，但还没解锁
  EXPECT_EQ(sel.stage(), mocap::OriginStage::Boot);
  EXPECT_FALSE(sel.switched());
  EXPECT_NEAR(sel.origin().p.x, kPBoot.x, 1e-12) << "解锁前原点不许动";
}

TEST(TwoStageOrigin, 每收到state就直接当已解锁是不允许的)
{
  // 未收到 /mavros/state 时，has_state=false、armed 传 false：不许把它当成"已解锁"。
  mocap::TwoStageOrigin sel(3, 3);
  for (int i = 0; i < 20; ++i) {
    feed(sel, kPBoot, kQBoot, mocap::ArmedState::Unknown);
  }
  EXPECT_EQ(sel.stage(), mocap::OriginStage::Boot);
}

TEST(TwoStageOrigin, 解锁上升沿后才开始采起飞前原点)
{
  mocap::TwoStageOrigin sel(4, 6);
  feed_boot(sel, 4);
  ASSERT_TRUE(sel.ready());

  // 上升沿这一帧：识别到边沿，但采样窗还没满 ⟹ 仍然用启动原点
  feed(sel, kPBoot, kQBoot, mocap::ArmedState::Armed);
  EXPECT_EQ(sel.stage(), mocap::OriginStage::Boot);
  EXPECT_FALSE(sel.switched());
  EXPECT_NEAR(sel.origin().p.x, kPBoot.x, 1e-12);

  // 采满 → 切换
  feed_arm(sel, 5, kPBoot, kQBoot);
  ASSERT_TRUE(sel.switched());
  EXPECT_EQ(sel.stage(), mocap::OriginStage::Arm);
  EXPECT_NEAR(sel.origin().yaw, mocap::yaw_of(kQBoot), 1e-12);
}

TEST(TwoStageOrigin, 切换瞬间位置无跳变_前提是启动到解锁之间没动过)
{
  // 纪律成立（p_boot == p_arm）时的行为：对**任意**探测位姿，切换前后的输出完全相同。
  mocap::TwoStageOrigin sel(4, 6);
  feed_boot(sel, 4);
  ASSERT_TRUE(sel.ready());
  const mocap::TwoStageOrigin stage_a = sel;  // 切换前的快照

  feed_arm(sel, 6, kPBoot, kQBoot);  // 静止不动地解锁
  ASSERT_TRUE(sel.switched());
  const mocap::TwoStageOrigin & stage_b = sel;

  EXPECT_NEAR(sel.switch_jump(), 0.0, 1e-12) << "没动过 ⟹ 切换量必须是 0";

  const mocap::Pose probe{Vec3{kPBoot.x + 1.3, kPBoot.y - 0.7, kPBoot.z + 0.9}, kQBoot};
  const mocap::Pose a = stage_a.apply(probe);
  const mocap::Pose b = stage_b.apply(probe);
  EXPECT_NEAR(b.p.x, a.p.x, 1e-12);
  EXPECT_NEAR(b.p.y, a.p.y, 1e-12);
  EXPECT_NEAR(b.p.z, a.p.z, 1e-12);
}

TEST(TwoStageOrigin, 切换瞬间的跳变就等于动过的量_可被日志看见)
{
  // 纪律没守住的情形：启动后在解锁前把飞机挪了 d ⟹ 切换瞬间位置恰好跳 d（EKF2 会当成真实运动）。
  // 本测试钉的是"这个跳变是可测量、可告警的"，不是"它无害"。
  mocap::TwoStageOrigin sel(4, 6);
  feed_boot(sel, 4);
  const mocap::TwoStageOrigin stage_a = sel;
  const Vec3 d{0.12, -0.05, 0.02};
  const Vec3 p_arm{kPBoot.x + d.x, kPBoot.y + d.y, kPBoot.z + d.z};
  feed_arm(sel, 6, p_arm, kQBoot);
  ASSERT_TRUE(sel.switched());

  const double expected_jump = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  EXPECT_NEAR(sel.switch_jump(), expected_jump, 1e-12);

  const mocap::Pose probe{p_arm, kQBoot};
  const mocap::Pose a = stage_a.apply(probe);
  const mocap::Pose b = sel.apply(probe);
  EXPECT_NEAR(b.p.x - a.p.x, -d.x, 1e-12);
  EXPECT_NEAR(b.p.y - a.p.y, -d.y, 1e-12);
  EXPECT_NEAR(b.p.z - a.p.z, -d.z, 1e-12);
}

TEST(TwoStageOrigin, 阶段B输出以解锁点为原点且yaw已归零)
{
  mocap::TwoStageOrigin sel(4, 6);
  const Vec3 p_arm{kPBoot.x + 0.05, kPBoot.y, kPBoot.z};
  feed_boot(sel, 4);
  feed_arm(sel, 6, p_arm, kQBoot);
  ASSERT_TRUE(sel.switched());

  const mocap::Pose out = sel.apply(mocap::Pose{p_arm, kQBoot});
  EXPECT_NEAR(out.p.x, 0.0, 1e-12);
  EXPECT_NEAR(out.p.y, 0.0, 1e-12);
  EXPECT_NEAR(out.p.z, 0.0, 1e-12);
  EXPECT_NEAR(mocap::yaw_of(out.q), 0.0, 1e-12);
}

TEST(TwoStageOrigin, 阶段B姿态处理不污染位置输出)
{
  // ★ 阶段 A 那条「位置与姿态解耦」是在**不归零 yaw** 的情况下验的；真正做姿态变换的是阶段 B。
  //   这里在**已切换之后**用三种差别很大的姿态探测：输出位置必须与姿态无关（逐位相同），
  //   且都等于 p − p_arm（姿态若以任何方式混进位置，这里立刻不等）。
  mocap::TwoStageOrigin sel(4, 6);
  const Vec3 p_arm{kPBoot.x + 0.05, kPBoot.y - 0.02, kPBoot.z};
  feed_boot(sel, 4);
  feed_arm(sel, 6, p_arm, kQBoot);
  ASSERT_TRUE(sel.switched());

  const Vec3 p{kPBoot.x - 0.4, kPBoot.y + 0.9, kPBoot.z + 0.3};
  const mocap::Pose flat{p, quat_zyx(0.0, 0.0, 0.0)};
  const mocap::Pose tilted{p, quat_zyx(35 * kDeg, -20 * kDeg, 120 * kDeg)};
  const mocap::Pose wild{p, quat_zyx(-170 * kDeg, 80 * kDeg, -95 * kDeg)};
  const Vec3 r0 = sel.apply(flat).p;
  const Vec3 r1 = sel.apply(tilted).p;
  const Vec3 r2 = sel.apply(wild).p;
  for (const Vec3 & r : {r0, r1, r2}) {
    EXPECT_NEAR(r.x, -0.45, 1e-12) << "位置 = p − p_arm，与姿态无关";
    EXPECT_NEAR(r.y, 0.92, 1e-12);
    EXPECT_NEAR(r.z, 0.3, 1e-12);
  }
  EXPECT_DOUBLE_EQ(r0.x, r1.x);
  EXPECT_DOUBLE_EQ(r1.y, r2.y);
  EXPECT_DOUBLE_EQ(r0.z, r2.z);
  // 而姿态确实**变了**（否则这条测试会因为"姿态被整体忽略"而假通过）
  EXPECT_GT(std::fabs(mocap::yaw_of(sel.apply(wild).q)), 1.0);
}

TEST(TwoStageOrigin, 上电即在高位不算边沿)
{
  // 同 RC 通道的边缘语义：节点启动时看到的第一条 state 若是 armed=true，**不当作上升沿**
  // （我们不知道它是什么时候解的锁，拿这一刻当"起飞前静止帧"是没有依据的）。
  mocap::TwoStageOrigin sel(3, 3);
  for (int i = 0; i < 50; ++i) {
    feed(sel, kPBoot, kQBoot, mocap::ArmedState::Armed);
  }
  EXPECT_EQ(sel.stage(), mocap::OriginStage::Boot);
  EXPECT_FALSE(sel.switched());
}

TEST(TwoStageOrigin, 只认第一次上升沿_切换后永不重锁)
{
  // ★ 这条是安全性质：飞行中再锁一次原点 = 给 EKF2 一记位置跳变。
  mocap::TwoStageOrigin sel(3, 3);
  feed_boot(sel, 3);
  feed_arm(sel, 3, kPBoot, kQBoot);
  ASSERT_TRUE(sel.switched());
  const mocap::Origin locked = sel.origin();

  // 上锁 → 再解锁（还在原地），原点不许变
  feed(sel, kPBoot, kQBoot, mocap::ArmedState::Disarmed);
  feed(sel, kPBoot, kQBoot, mocap::ArmedState::Armed);
  feed_arm(sel, 20, Vec3{kPBoot.x + 2.0, kPBoot.y, kPBoot.z}, kQBoot);
  EXPECT_EQ(sel.stage(), mocap::OriginStage::Arm);
  EXPECT_NEAR(sel.origin().p.x, locked.p.x, 1e-12);
  EXPECT_NEAR(sel.origin().p.y, locked.p.y, 1e-12);
  EXPECT_NEAR(sel.origin().p.z, locked.p.z, 1e-12);
  EXPECT_NEAR(sel.origin().yaw, locked.yaw, 1e-12);
}

TEST(TwoStageOrigin, 四个采样数下界都夹到1_不会死锁)
{
  mocap::TwoStageOrigin sel(0, 0);
  feed(sel, kPBoot, kQBoot, mocap::ArmedState::Disarmed);
  EXPECT_TRUE(sel.ready()) << "boot_samples=0 应被夹到 1";
  feed(sel, kPBoot, kQBoot, mocap::ArmedState::Armed);
  EXPECT_TRUE(sel.switched()) << "arm_samples=0 应被夹到 1";
}

}  // namespace
