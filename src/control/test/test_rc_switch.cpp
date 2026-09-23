// seam 2 —— `rc_switch.hpp`（"开始键"上升沿判据，ticket 09）的纯函数单测，零 ROS 依赖。
//
// ★ 这一份存在的理由：这条判据的三个坑在**话题层面完全同形**（都表现为"飞机进了 TRACK"
//   或"没进"），只能在这里钉：
//     ① 上电即在高位被当成一次边沿 ⟹ 操作员什么都没做，飞机自己受理了起飞（§H.5.2 明令禁止）；
//     ② 抖动没有去抖 ⟹ 一次按键被算成两条边沿（第 1 条解锁，第 2 条直接把轨迹开起来
//        —— 而那一刻操作员的手还停在开关上）；
//     ③ 边沿"可读"而不是"取走" ⟹ 调用方每帧问一次，同一次按键在下一帧被再取一次，
//        于是"第 1 次解锁、第 2 次开轨迹"这条语义整个错位。
//
// 判据的**唯一出处**是 `docs/reference/params-changes.md` §H.1 / §H.5（用户 2026-09-21 裁定），
// 逐字引在 `rc_switch.hpp` 文件头。下面的阈值取 §H.1 定案的那一组（0.75 / 0.25 / 200 ms）。

#include <limits>

#include "control/rc_switch.hpp"
#include "gtest/gtest.h"

namespace
{

/// §H.1 定案的那一组参数（`control.yaml` 的默认值）。
/// 通道刻度取 PX4 的标准 PWM 量程（1000/2000 µs）—— §H.1 记的 `1750 / 1250 µs` 就是
/// 按它折出来的（0.75 / 0.25）。
constexpr double kLowUs = 1000.0;
constexpr double kHighUs = 2000.0;
constexpr double kLow = 1000.0;    ///< 低位读数
constexpr double kHigh = 2000.0;   ///< 高位读数
constexpr double kBand = 1500.0;   ///< 死区中间的读数

/// 真 mavros 上 `/mavros/rc/in` 的实测帧间隔（10 Hz，见 `bench/config/bench.yaml` 的 `rates_hz.rc`）。
constexpr double kDt = 0.1;

/// 去抖 200 ms ⟹ 10 Hz 下要连喂几帧同一电平（第 1 帧只负责起算）。
constexpr int kDebounceFrames = 3;

control::StartSwitchDetector make_detector()
{
  return control::StartSwitchDetector(0.75, 0.25, 0.2, kLowUs, kHighUs);
}

/// 以 10 Hz 从 `t_s` 起喂 `frames` 帧同一读数；返回**喂完之后**的时刻。
/// ★ 用"连喂 n 帧"而不是散着写时间，是因为这条判据的时间语义就是"连续保持多久"，
///   散写时辰极易把去抖窗算错一帧（而错一帧的后果是"按了没反应"，现场看不出来）。
double feed(control::StartSwitchDetector & det, double us, double t_s, int frames)
{
  double t = t_s;
  for (int i = 0; i < frames; ++i) {
    det.update(us, t);
    t += kDt;
  }
  return t;
}

}  // namespace

/// ★★ §H.5.2 逐字要求：**上电即在高位不算边沿**（必须先见低位）。
/// 少了这条，开机时开关恰好停在上位就会让飞机在操作员什么都不知道的时候被受理起飞。
TEST(StartSwitch, 上电即在高位不算边沿)
{
  control::StartSwitchDetector det = make_detector();
  for (int i = 0; i < 50; ++i) {   // 5 s 高位
    det.update(kHigh, kDt * i);
    EXPECT_FALSE(det.take_edge()) << "第 " << i << " 帧凭空产生了边沿";
  }
  EXPECT_TRUE(det.high()) << "电平本身要认得出来（只是不算边沿）";
  EXPECT_FALSE(det.released()) << "没确认过低位 ⟹ H.5.2 的那条判据不成立";
  EXPECT_EQ(det.edge_count(), 0);
}

/// 正常一次按键：**先低位、再高位、去抖满足** ⟹ 恰好一条边沿。
///
/// ⚠️ 去抖是"候选电平**连续**保持多久"（`rc_switch.hpp`）⟹ 10 Hz 下要 3 帧（≈0.2 s）。
///   这条用例把这个帧数钉住 —— "按下去但底下那 0.2 s 还没走完"在现场就是"按了没反应"，
///   必须能一眼看出是在去抖、而不是坏了或没接上。
TEST(StartSwitch, 先低位再高位且去抖满足时产生一条边沿)
{
  control::StartSwitchDetector det = make_detector();
  double t = feed(det, kLow, 0.0, kDebounceFrames - 1);
  EXPECT_FALSE(det.released()) << "低位还没确认（去抖窗没走完）";
  t = feed(det, kLow, t, 1);        // 低位确认
  EXPECT_TRUE(det.released());
  EXPECT_FALSE(det.take_edge());

  t = feed(det, kHigh, t, kDebounceFrames - 1);
  EXPECT_FALSE(det.take_edge()) << "去抖窗还没走完就算数 ⟹ 抖动会被当成按键";
  feed(det, kHigh, t, 1);
  EXPECT_TRUE(det.take_edge());
  EXPECT_TRUE(det.high());
  EXPECT_EQ(det.edge_count(), 1);
}

/// 边沿是**事件**：取走一次就没了（同一次按键不许被消费两次）。
/// ★ 反例（第一版实现就是这个形状）会把一次按键算成"解锁 + 开始轨迹"两步 ——
///   飞机刚离地就进 TRACK，而操作员的手还停在开关上。
TEST(StartSwitch, 边沿取出一次之后不再重复)
{
  control::StartSwitchDetector det = make_detector();
  double t = feed(det, kLow, 0.0, kDebounceFrames);
  t = feed(det, kHigh, t, kDebounceFrames);
  ASSERT_TRUE(det.take_edge());

  for (int i = 0; i < 30; ++i) {   // 开关一直停在高位（H.5.1：电平本身什么都不做）
    det.update(kHigh, t);
    EXPECT_FALSE(det.take_edge()) << "第 " << i << " 帧又把上一次的边沿取出来了";
    t += kDt;
  }
}

/// 抖动不去抖 —— 一个**短于去抖窗**的高位脉冲不算按键。
TEST(StartSwitch, 短于去抖窗的高位脉冲不算)
{
  control::StartSwitchDetector det = make_detector();
  double t = feed(det, kLow, 0.0, kDebounceFrames);
  t = feed(det, kHigh, t, kDebounceFrames - 1);   // 抖动：只维持两帧（0.1 s < 0.2 s）
  feed(det, kLow, t, 5);
  EXPECT_FALSE(det.take_edge());
  EXPECT_EQ(det.edge_count(), 0);
  EXPECT_FALSE(det.high()) << "电平没有确认，不能报「按着」";
}

/// ★ 死区（`release_threshold < x < start_threshold`）里的读数是"**没有信息**"：
/// 不是低位、也不是高位。它既不产生边沿，**也不打断**正在走的去抖窗 ——
/// 后半句错了（把死区当低位或当成新的候选）会让"开关从低位拨到高位、路上经过中位"
/// 多花一个去抖窗，而现场只看到"这次按下去比以前迟钝"。
TEST(StartSwitch, 死区读数不打断去抖计时)
{
  control::StartSwitchDetector det = make_detector();
  const double t0 = feed(det, kLow, 0.0, kDebounceFrames);   // 低位已确认

  det.update(kHigh, t0);            // 高位第 1 帧 ⟹ 起算
  det.update(kBand, t0 + kDt);      // 途经中位（死区）
  det.update(kHigh, t0 + 2 * kDt);  // 距高位第 1 帧正好 0.2 s
  EXPECT_TRUE(det.take_edge()) << "死区读数把去抖计时打断了（应当保持候选与它的起点）";
}

/// 从没越界过的读数（一直停在带内）永远不产生边沿、也不确认任何电平。
TEST(StartSwitch, 一直停在阈值带内不产生边沿)
{
  control::StartSwitchDetector det = make_detector();
  for (int i = 0; i < 50; ++i) {
    det.update(kBand, kDt * i);
    EXPECT_FALSE(det.take_edge());
  }
  EXPECT_FALSE(det.released());
  EXPECT_FALSE(det.high());
  EXPECT_TRUE(det.sample_valid()) << "收到过帧，只是它什么也没说明";
}

/// 两次按键 = 两条边沿（§H.5：第 1 次解锁起飞、第 2 次开始轨迹）。
/// ★ 中间必须**确认过低位**（放下开关）才算第二次 —— 只回到带内不算。
TEST(StartSwitch, 两次按键产生两条边沿)
{
  control::StartSwitchDetector det = make_detector();
  double t = feed(det, kLow, 0.0, kDebounceFrames);
  t = feed(det, kHigh, t, kDebounceFrames);
  EXPECT_TRUE(det.take_edge());

  // 放下开关（回低位并确认）
  t = feed(det, kLow, t, kDebounceFrames);
  EXPECT_FALSE(det.take_edge());

  // 再按一次
  feed(det, kHigh, t, kDebounceFrames);
  EXPECT_TRUE(det.take_edge());
  EXPECT_EQ(det.edge_count(), 2);
  EXPECT_FALSE(det.take_edge()) << "第 2 条取走之后就没有了";
}

/// ★ 非有限读数（NaN/Inf）= "这一帧没有信息"：不动状态、**不能当成低位**。
/// 当成低位的后果很具体：链路上出现一帧 NaN 就等于"操作员把开关放下来了"，
/// 于是下一次真实的高位又变成一条新边沿（凭空多出一条边）。
TEST(StartSwitch, 非有限读数不动状态)
{
  control::StartSwitchDetector det = make_detector();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  det.update(nan, 0.0);
  EXPECT_FALSE(det.released()) << "NaN 被当成低位了";
  EXPECT_FALSE(det.sample_valid());

  // 高位序列里插一帧 NaN：不打断去抖窗（它没有任何信息）。
  const double t0 = feed(det, kLow, 0.1, kDebounceFrames);
  det.update(kHigh, t0);
  det.update(nan, t0 + kDt);
  det.update(kHigh, t0 + 2 * kDt);
  EXPECT_TRUE(det.take_edge()) << "NaN 打断了去抖计时";
}

/// ★★ 链路断过之后的复位：**必须重新看见一次低位**，才承认下一次高位是边沿。
/// 这是 §H.5.2 那条判据在断链场景下的复现 —— "已确认过低电平"这一位在断链前是真的，
/// 而断链期间开关动没动过我们不知道（`px4_interface.cpp::on_rc` 在 `gap` 上复位它）。
/// ⚠️ 不复位的后果很具体：链路抖一下、开关恰好停在上位 ⟹ 恢复的第一帧凭空产生一条边沿，
///   而那条边沿要么被当成"起飞"、要么被当成"开始轨迹"。
TEST(StartSwitch, 断链复位后必须重新看见低位才算边沿)
{
  control::StartSwitchDetector det = make_detector();
  double t = feed(det, kLow, 0.0, kDebounceFrames);
  t = feed(det, kHigh, t, kDebounceFrames);
  ASSERT_TRUE(det.take_edge());

  det.reset();
  EXPECT_FALSE(det.released()) << "复位之后「已确认过低电平」必须作废";
  EXPECT_FALSE(det.high()) << "复位之后电平回到未知";
  EXPECT_EQ(det.edge_count(), 1) << "★ 边沿总数是「上电以来确认过几条」的诊断计数，只增不减";

  // 开关一直停在高位（断链期间没有动过）—— 恢复之后**不许**凭空产生边沿。
  t = feed(det, kHigh, t, 5);
  EXPECT_FALSE(det.take_edge()) << "断链 + 开关停在上位 ⟹ 恢复后凭空造出了一条边沿";

  // 必须重新走一遍"低位 → 高位"。
  t = feed(det, kLow, t, kDebounceFrames);
  EXPECT_TRUE(det.released());
  t = feed(det, kHigh, t, kDebounceFrames);
  EXPECT_TRUE(det.take_edge()) << "重新确认过低位之后，下一次高位应当照常是边沿";
  EXPECT_EQ(det.edge_count(), 2);
}

/// 折算的方向由 `low_us` / `high_us` **两个参数**定义，不由名字里的 min/max 定义 ——
/// 现场换一个方向相反的接收机通道，改这两个数即可，**不改代码**。
TEST(StartSwitch, 折算方向由参数决定)
{
  // 反着配：2000 µs 折算到 0（低位）、1000 µs 折算到 1（高位）。
  control::StartSwitchDetector det(0.75, 0.25, 0.2, 2000.0, 1000.0);
  double t = feed(det, 2000.0, 0.0, kDebounceFrames);
  t = feed(det, 1000.0, t, kDebounceFrames);
  EXPECT_TRUE(det.take_edge()) << "两个端点的方向应当由参数说了算";
  EXPECT_NEAR(det.normalized(), 1.0, 1e-12);
}

/// 两个端点**配成相等**才是退化配置：方向无从判定 ⟹ 折算恒为 0 ⟹ 永远产生不了边沿。
/// ★ 这是"配错了也不会动"的安全侧退化（与"反着配"**不是**同一件事，见上一条用例）。
TEST(StartSwitch, 端点配成相等时永远不产生边沿)
{
  control::StartSwitchDetector det(0.75, 0.25, 0.2, 1500.0, 1500.0);
  double t = feed(det, 1000.0, 0.0, 20);
  feed(det, 2000.0, t, 20);
  EXPECT_FALSE(det.take_edge());
  EXPECT_EQ(det.edge_count(), 0);
  EXPECT_NEAR(det.normalized(), 0.0, 1e-12);
}

/// 折算**不钳位**：越界的读数要从 `normalized()` 里看得出来（配错刻度时唯一的线索）。
TEST(StartSwitch, 越界读数不钳位)
{
  control::StartSwitchDetector det = make_detector();
  det.update(2200.0, 0.0);
  EXPECT_NEAR(det.normalized(), 1.2, 1e-12);
  EXPECT_FALSE(det.take_edge()) << "上电即高位（越界也一样）不算边沿（H.5.2）";
  det.update(900.0, kDt);
  EXPECT_NEAR(det.normalized(), -0.1, 1e-12);
}
