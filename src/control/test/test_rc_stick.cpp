// seam 2 —— `rc_stick.hpp`（打杆判据）的纯函数单测，零 ROS 依赖。
//
// ★ 这一份存在的**唯一理由**是把一个极易读错的语义钉死：`COM_RC_STICK_OV` 比的是
//   **摇杆的每秒变化率**，不是**摇杆偏移量**。本项目早期文档（`params-changes.md` D2、
//   ticket 06 正文）都写成了"打杆 20% 即接管" —— 那是误读。
//   两种读法在实验室里表现不同，但**都不会报错**：
//     · 真语义（变化率）：摇杆推到头再**保持不动** ⟹ 不再算接管
//     · 误读（偏移量）：只要摇杆不在中位就一直算接管
//   所以下面每条"不算在动"的用例都是一次**反向验证**：它会在误读的实现上失败。
//
// 算法的第一手出处见 `rc_stick.hpp` 文件头（`ManualControl.cpp:163-168` +
// `MovingDiff.hpp:40-58` + `AlphaFilter.hpp:62-77`）。

#include <cmath>
#include <limits>

#include "control/rc_stick.hpp"
#include "gtest/gtest.h"

namespace
{

/// z0 实测校准的中位/行程（`RC1_MIN = 989 / RC1_MAX = 2012`）—— 用例里的 µs 都按它取值，
/// 这样"半行程"之类的说法有实际尺度。
constexpr double kCenter = 1500.0;
constexpr double kHalfStroke = 511.0;

/// 以固定周期喂一串值，返回**最后一帧**的判定。
bool feed(control::StickMoveDetector & det, double from_us, double to_us, int steps, double dt_s)
{
  bool moving = false;
  for (int i = 1; i <= steps; ++i) {
    const double v = from_us + (to_us - from_us) * static_cast<double>(i) / steps;
    moving = det.update(v, dt_s);
  }
  return moving;
}

}  // namespace

TEST(StickMoveDetector, 第一帧不算在动)
{
  control::StickMoveDetector det;
  // 没有上一帧 ⟹ 变化率无从算起（`MovingDiff::_last_value` 初值 NaN 的等价行为）。
  // 否则节点刚起来收到的第一帧就会被当成"飞行员在打杆"（假夺权，直接把控制权交出去）。
  EXPECT_FALSE(det.update(kCenter, 0.1));
  EXPECT_DOUBLE_EQ(det.rate_us_per_s(), 0.0);
}

TEST(StickMoveDetector, 快速推杆算在动)
{
  control::StickMoveDetector det;
  det.update(kCenter, 0.1);
  // 0.1 s 走了 500 µs ⟹ 5000 µs/s，比阈值大 50 倍。
  EXPECT_TRUE(det.update(kCenter + 500.0, 0.1));
}

TEST(StickMoveDetector, 反向推杆也算在动)
{
  control::StickMoveDetector det;
  det.update(kCenter, 0.1);
  EXPECT_TRUE(det.update(kCenter - 500.0, 0.1));
  EXPECT_LT(det.rate_us_per_s(), 0.0) << "符号应保留（反向）";
}

/// ★★ 本文件的核心用例：**慢慢推满半行程 ⟹ 不算在动**。
/// 20 s 推 511 µs（≈ 满行程的一半）⟹ 25 µs/s，只有阈值的 1/4。
/// 误读成"偏移量"的实现会在这里返回 true。
TEST(StickMoveDetector, 慢速大行程漂移不算在动)
{
  control::StickMoveDetector det;
  det.update(kCenter, 0.1);
  const bool moving = feed(det, kCenter, kCenter + kHalfStroke, 200, 0.1);   // 20 s

  EXPECT_FALSE(moving) << "比的是变化率，不是偏移量 —— 慢漂移不该判成夺权";
  EXPECT_LT(std::fabs(det.rate_us_per_s()), det.threshold_us_per_s());
}

/// ★ 同一件事的另一面：**推到头再保持不动**，判据会衰减下去。
/// 这是"变化率"语义的直接推论，也是最容易在实飞中观察到的一条。
TEST(StickMoveDetector, 推到头保持不动会衰减到阈值以下)
{
  control::StickMoveDetector det;
  det.update(kCenter, 0.1);
  EXPECT_TRUE(det.update(kCenter + kHalfStroke, 0.1)) << "推到头那一瞬间确实在动";

  bool moving = true;
  int frames_until_quiet = 0;
  for (int i = 0; i < 200 && moving; ++i) {
    moving = det.update(kCenter + kHalfStroke, 0.1);   // 保持不动
    ++frames_until_quiet;
  }
  EXPECT_FALSE(moving) << "摇杆保持不动时不该一直算接管";
  // 一阶低通（α = dt/(0.1+dt) = 0.5 @ 10 Hz）⟹ 每帧减半，从 5110 µs/s 衰减到 100 只要几帧。
  EXPECT_LT(frames_until_quiet, 20) << "衰减太快/太慢都说明滤波器常数被改过："
                                    << frames_until_quiet << " 帧";
}

/// dt 必须**真的**进滤波器：同一个物理速度，10 Hz 与 20 Hz 喂出来应有一致的判定。
/// （若实现里把 dt 忽略掉，`(v2-v1)/dt` 那一项在两个周期下会差 2 倍，下面的阈值就会暴露。）
TEST(StickMoveDetector, 不同周期下判定一致)
{
  control::StickMoveDetector slow;   // 10 Hz
  control::StickMoveDetector fast;   // 20 Hz
  slow.update(kCenter, 0.1);
  fast.update(kCenter, 0.05);

  // 两者都是 **1500 µs/s** 的匀速（500 µs/s 一个周期 vs 周期减半时步长减半）⟹ 判定必须一致。
  const bool a = feed(slow, kCenter, kCenter + 150.0, 10, 0.1);    // 150 µs / 0.1 s
  const bool b = feed(fast, kCenter, kCenter + 75.0, 10, 0.05);    // 75 µs / 0.05 s
  EXPECT_EQ(a, b) << "同一物理速度在两个采样周期下判定不一致";
  EXPECT_TRUE(a);
}

TEST(StickMoveDetector, 非正周期不作判断)
{
  control::StickMoveDetector det;
  det.update(kCenter, 0.1);
  EXPECT_FALSE(det.update(kCenter + 900.0, 0.0)) << "dt = 0 时变化率是无穷大，不能据此夺权";
  EXPECT_FALSE(det.update(kCenter + 900.0, -0.1));
  // 而且不能污染状态：下一帧用正常 dt 仍然能正常判。
  EXPECT_TRUE(det.update(kCenter + 1800.0, 0.1)) << "被空帧影响之后判据失灵了";
}

TEST(StickMoveDetector, NaN值不作判断且不污染状态)
{
  control::StickMoveDetector det;
  det.update(kCenter, 0.1);
  const double nan = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(det.update(nan, 0.1));
  EXPECT_TRUE(std::isfinite(det.rate_us_per_s())) << "NaN 不能灌进滤波器状态（会永久失灵）";
  EXPECT_FALSE(det.update(std::numeric_limits<double>::infinity(), 0.1));
  EXPECT_TRUE(std::isfinite(det.rate_us_per_s()));
  // 之后正常的一帧仍然工作。
  EXPECT_TRUE(det.update(kCenter + 500.0, 0.1));
}

TEST(StickMoveDetector, reset之后第一帧不算在动)
{
  control::StickMoveDetector det;
  det.update(kCenter, 0.1);
  EXPECT_TRUE(det.update(kCenter + 900.0, 0.1));

  det.reset();
  EXPECT_DOUBLE_EQ(det.rate_us_per_s(), 0.0);
  // 丢帧/长时间无数据之后重新开始：绝不能让"从旧值跳过来"算成一次打杆。
  EXPECT_FALSE(det.update(kCenter, 0.1));
}

TEST(StickMoveDetector, 阈值可配且与固件同源)
{
  control::StickMoveDetector loose(20.0);    // COM_RC_STICK_OV 调很小
  control::StickMoveDetector tight(1000.0);  // 调很大
  loose.update(kCenter, 0.1);
  tight.update(kCenter, 0.1);

  // 30 µs / 0.1 s = 30 µs/s，匀速喂 10 帧 ⟹ 滤波后的变化率收敛到 30 µs/s。
  EXPECT_TRUE(feed(loose, kCenter, kCenter + 30.0, 10, 0.1));
  EXPECT_FALSE(feed(tight, kCenter, kCenter + 30.0, 10, 0.1));
  EXPECT_DOUBLE_EQ(tight.threshold_us_per_s(), 1000.0);
}

/// ★ 低通让"变化率"**滞后一帧**：单独一帧刚好越过阈值的跳变**不算在动**，
/// 得持续才有判定。这是 `AlphaFilter` 的直接后果，也是想要的性质 ——
/// 单帧抖动（/mavros/rc/in 的量化噪声、一帧 1 µs 的抖动）不该把控制权交出去。
TEST(StickMoveDetector, 单帧越过阈值不算在动)
{
  control::StickMoveDetector det;   // 阈值 100 µs/s
  det.update(kCenter, 0.1);
  // 一帧内跳 30 µs ⟹ 瞬时 300 µs/s（3 倍阈值），但滤波后只有 150 µs/s…
  EXPECT_TRUE(det.update(kCenter + 30.0, 0.1)) << "3 倍阈值的单帧跳变应当算在动";

  // 反例：一帧内跳 15 µs ⟹ 瞬时 150 µs/s（越过阈值），滤波后 75 µs/s ⟹ 不判。
  control::StickMoveDetector jitter;
  jitter.update(kCenter, 0.1);
  EXPECT_FALSE(jitter.update(kCenter + 15.0, 0.1)) << "单帧刚好越过阀值时不该判成夺权";
  EXPECT_LT(std::fabs(jitter.rate_us_per_s()), jitter.threshold_us_per_s());
}

/// 默认阈值 = `COM_RC_STICK_OV = 20` × z0 实测校准（见 `rc_stick.hpp` 文件头）。
/// ★ 这条断言的作用是**提醒改固件参数的人同步改这里**：两者不同源的话，节点眼里的
/// "夺权"与 PX4 眼里的"打杆接管"会发生在不同时刻。
TEST(StickMoveDetector, 默认阈值等于固件口径)
{
  control::StickMoveDetector det;
  EXPECT_DOUBLE_EQ(det.threshold_us_per_s(), 100.0);
}

/// 观测口收敛到真实速度（钉住单位是 µs/s，不是归一化摇杆）。
TEST(StickMoveDetector, 观测口收敛到真实速度)
{
  control::StickMoveDetector det;
  det.update(1000.0, 0.1);
  feed(det, 1000.0, 1500.0, 10, 0.1);   // 每帧 50 µs / 0.1 s = 500 µs/s 匀速
  EXPECT_NEAR(det.rate_us_per_s(), 500.0, 25.0);
}
