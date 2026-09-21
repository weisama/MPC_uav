// seam 2 —— `fsm.hpp` 的**纯函数单测**，零 ROS 依赖（不链接 rclcpp、不调 ament_target_dependencies）。
//
// 为什么除了 seam 1 的话题级测试之外还要这一份：
//   · 话题级测试能回答"飞机最后发了什么"，但**回答不了"是哪条守卫放行的"** —— 四条守卫
//     全部失效时，话题上的表现都是"没有 setpoint"，症状完全一样，诊断分辨率到顶了。
//   · 这里的判据（`missing_guards` 的成员与顺序、前置流计时器的重置）是**纯逻辑**，
//     在纯函数层测，失败信息直接指着那条守卫，不用再去翻日志。
// 与 `mocap/test/test_mocap_rel_pose.cpp` 同一个思路：把"飞起来才发现错"的判据钉在零依赖层。

#include <string>
#include <vector>

#include "control/fsm.hpp"
#include "gtest/gtest.h"

namespace
{

/// 四条守卫全成立 —— 每个用例都以它为基线，再**只**掀掉一条，这样"是它导致的"才成立。
control::Guards all_ok()
{
  control::Guards g;
  g.connected = true;
  g.mocap_fresh = true;
  g.local_pos_fresh = true;
  g.armed = false;
  return g;
}

std::vector<std::string> names(const std::vector<control::Guard> & guards)
{
  std::vector<std::string> out;
  for (control::Guard which : guards) {
    switch (which) {
      case control::Guard::kConnected:     out.push_back("connected"); break;
      case control::Guard::kMocapFresh:    out.push_back("mocap_fresh"); break;
      case control::Guard::kLocalPosFresh: out.push_back("local_pos_fresh"); break;
      case control::Guard::kNotArmed:      out.push_back("not_armed"); break;
    }
  }
  return out;
}

}  // namespace

// ─────────────────────────── 守卫清单 ───────────────────────────

TEST(GuardList, 四条全成立时没有缺项且放行)
{
  EXPECT_TRUE(control::missing_guards(all_ok()).empty());
  EXPECT_TRUE(control::ready_to_stream(all_ok()));
}

/// ★ 逐条掀掉 —— 每条都断言"只少了它这一条"。只断言"不放行"是不够的：
/// 那样"某条守卫永远为假"（例如把手写成了 `g.armed` 而不是 `!g.armed`）也照样通过。
TEST(GuardList, 逐条掀掉时缺项恰好是那一条)
{
  struct Case
  {
    const char * name;
    control::Guard which;
    void (*break_it)(control::Guards &);
  };
  const Case cases[] = {
    {"connected", control::Guard::kConnected, [](control::Guards & g) {g.connected = false;}},
    {"mocap_fresh", control::Guard::kMocapFresh, [](control::Guards & g) {g.mocap_fresh = false;}},
    {"local_pos_fresh", control::Guard::kLocalPosFresh,
      [](control::Guards & g) {g.local_pos_fresh = false;}},
    {"not_armed", control::Guard::kNotArmed, [](control::Guards & g) {g.armed = true;}},
  };

  for (const Case & c : cases) {
    control::Guards g = all_ok();
    c.break_it(g);
    EXPECT_FALSE(control::ready_to_stream(g)) << "掀掉 " << c.name << " 之后仍然放行了";
    EXPECT_EQ(names(control::missing_guards(g)), std::vector<std::string>{c.name})
      << "掀掉 " << c.name << " 时缺项清单不对";
  }
}

/// 顺序固定（= `all_guards()` 的顺序）—— 日志里的缺项清单照着它排，改了会让人误读。
TEST(GuardList, 缺项顺序固定)
{
  control::Guards g;  // 全零
  g.armed = true;     // 补上最后一条，"四条全不成立"才是真全不成立（见下一条用例）
  EXPECT_EQ(
    names(control::missing_guards(g)),
    (std::vector<std::string>{"connected", "mocap_fresh", "local_pos_fresh", "not_armed"}));
}

/// ★ 全零的 `Guards` 里 **`not_armed` 是成立的**（`armed` 默认 false ⟹ 按"未解锁"处理）。
/// 这是刻意选的默认值，与 `Px4Interface::guards()` 同一条纪律：`/mavros/state` 一帧都没到时
/// `armed` 取 false，否则启动瞬间会伪造出一个"异常解锁"告警。
/// 它不会造成"没数据也放行"—— 那种情况下 `connected` 已经把门关上了。
TEST(GuardList, 全零守卫里未解锁是成立的)
{
  control::Guards g;
  EXPECT_TRUE(control::guard_holds(g, control::Guard::kNotArmed));
  EXPECT_EQ(
    names(control::missing_guards(g)),
    (std::vector<std::string>{"connected", "mocap_fresh", "local_pos_fresh"}));
  EXPECT_FALSE(control::ready_to_stream(g));
}

/// `kNotArmed` 是 `armed` 的**取反**（本张唯一一条"成立条件是假值"的守卫）。
TEST(GuardList, 未解锁是armed的取反)
{
  control::Guards g = all_ok();
  EXPECT_TRUE(control::guard_holds(g, control::Guard::kNotArmed));
  g.armed = true;
  EXPECT_FALSE(control::guard_holds(g, control::Guard::kNotArmed));
}

TEST(GuardList, 状态名进control_state话题)
{
  EXPECT_STREQ(control::to_string(control::Authority::kManual), "MANUAL");
  EXPECT_STREQ(control::to_string(control::Authority::kAuto), "AUTO");
  EXPECT_STREQ(control::to_string(control::Phase::kInit), "INIT");
  EXPECT_STREQ(control::to_string(control::Phase::kStandby), "STANDBY");
}

// ─────────────────────────── 状态机 ───────────────────────────

TEST(Fsm, 守卫不成立时退回INIT且停发setpoint)
{
  control::Fsm fsm(1.0);
  control::Guards g = all_ok();
  g.mocap_fresh = false;

  const control::Decision d = fsm.update(g, 100.0);
  EXPECT_EQ(d.phase, control::Phase::kInit);
  EXPECT_EQ(d.authority, control::Authority::kManual);
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_FALSE(d.request_offboard);
}

TEST(Fsm, 守卫成立即进STANDBY并开始流但控制权仍是MANUAL)
{
  control::Fsm fsm(1.0);
  const control::Decision d = fsm.update(all_ok(), 100.0);

  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_TRUE(d.stream_setpoint);
  // ★ 未解锁 ⟹ 不算交出控制权（README「两个正交维度」的定义）。
  EXPECT_EQ(d.authority, control::Authority::kManual);
  // 刚流了 0 s，前置流时长没到 ⟹ 不许切。
  EXPECT_FALSE(d.request_offboard);
}

/// 前置流时长的**两侧**都断言：差一点点不许切、到了才许切。
TEST(Fsm, 前置流时长两侧)
{
  control::Fsm fsm(3.0);
  fsm.update(all_ok(), 100.0);

  EXPECT_FALSE(fsm.update(all_ok(), 102.999).request_offboard) << "还没流够就切了";
  EXPECT_TRUE(fsm.update(all_ok(), 103.0).request_offboard) << "流够了却没切";
}

TEST(Fsm, 已经在OFFBOARD里就不再请求切模式)
{
  control::Fsm fsm(1.0);
  fsm.update(all_ok(), 100.0);

  control::Guards g = all_ok();
  g.in_offboard = true;
  EXPECT_FALSE(fsm.update(g, 200.0).request_offboard);
}

/// ★ 断流之后**重新计时**：守卫一断一续，前置流必须从头算。
/// 不重置的话，"断流前的旧时长"会让切模式发生在流刚恢复的下一帧 —— 而 PX4 那边
/// 看到的是一条刚续上的流，`offboard_control_mode` 还没重新连上。
TEST(Fsm, 断流后前置流重新计时)
{
  control::Fsm fsm(1.0);
  fsm.update(all_ok(), 100.0);
  EXPECT_TRUE(fsm.update(all_ok(), 150.0).request_offboard);  // 流够 50 s

  control::Guards bad = all_ok();
  bad.connected = false;
  fsm.update(bad, 151.0);                    // 断流
  EXPECT_DOUBLE_EQ(fsm.streaming_seconds(151.0), 0.0);

  fsm.update(all_ok(), 152.0);               // 恢复
  EXPECT_FALSE(fsm.update(all_ok(), 152.5).request_offboard) << "恢复后没重新计时";
  EXPECT_TRUE(fsm.update(all_ok(), 153.0).request_offboard);
}

TEST(Fsm, 观测口在没流时报零)
{
  control::Fsm fsm(1.0);
  EXPECT_DOUBLE_EQ(fsm.streaming_seconds(100.0), 0.0);

  control::Guards bad = all_ok();
  bad.local_pos_fresh = false;
  fsm.update(bad, 100.0);
  EXPECT_DOUBLE_EQ(fsm.streaming_seconds(130.0), 0.0);
}
