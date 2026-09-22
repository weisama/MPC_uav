// seam 2 —— `fsm.hpp` 的**纯函数单测**，零 ROS 依赖（不链接 rclcpp、不调 ament_target_dependencies）。
//
// 为什么除了 seam 1 的话题级测试之外还要这一份：
//   · 话题级测试能回答"飞机最后发了什么"，但**回答不了"是哪条守卫/哪条判据放行或拦下的"** ——
//     四条守卫全部失效、四种失效判据全部成立时，话题上的表现都是"没有 setpoint"，症状完全一样，
//     诊断分辨率到顶了。
//   · 这里的判据（`missing_guards` 的成员与顺序、失效判据的**优先级**、闩锁与复位）是**纯逻辑**，
//     在纯函数层测，失败信息直接指着那条判据，不用再去翻日志。
// 与 `mocap/test/test_mocap_rel_pose.cpp` 同一个思路：把"飞起来才发现错"的判据钉在零依赖层。
//
// ★ ticket 06 加的部分：失效判据（`Health`）、降级原因（`Degrade`）、闩锁与**复位**。
//   其中"复位"这一类断言是本张的重点 —— XI35 的三处死代码里有两处就是**单向 latch**
//   （见 `docs/reference/xi35-fsm.md` §4），所以每条闩锁都必须配一条"怎么退出去"的用例。

#include <string>
#include <vector>

#include "control/fsm.hpp"
#include "gtest/gtest.h"

namespace
{

/// 四条守卫全成立（未解锁）—— 每个用例都以它为基线，再**只**掀掉一条/换一条判据，
/// 这样"是它导致的"才成立。
control::Guards all_ok()
{
  control::Guards g;
  g.connected = true;
  g.mocap_fresh = true;
  g.local_pos_fresh = true;
  g.armed = false;
  return g;
}

/// 已解锁的场面（`own_arming` = 本张的台架缝，见 `control.yaml`）。
control::Guards armed_ok()
{
  control::Guards g = all_ok();
  g.armed = true;
  g.own_arming = true;
  return g;
}

/// 一切输入都健康（`Health` 的默认值就是"没有坏消息"）。
control::Health health_ok()
{
  control::Health h;
  h.rc_fresh = true;
  return h;
}

std::vector<std::string> names(const std::vector<control::Guard> & guards)
{
  std::vector<std::string> out;
  for (control::Guard which : guards) {
    switch (which) {
      case control::Guard::kConnected:     out.push_back("connected"); break;
      case control::Guard::kMocapFresh:    out.push_back("mocap_fresh"); break;
      case control::Guard::kLocalPosFresh: out.push_back("local_pos_fresh"); break;
      case control::Guard::kArmingByUs:    out.push_back("arming_by_us"); break;
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
    {"arming_by_us", control::Guard::kArmingByUs, [](control::Guards & g) {g.armed = true;}},
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
    (std::vector<std::string>{"connected", "mocap_fresh", "local_pos_fresh", "arming_by_us"}));
}

/// ★ 全零的 `Guards` 里 **`arming_by_us` 是成立的**（`armed` 默认 false ⟹ 按"未解锁"处理）。
/// 这是刻意选的默认值，与 `Px4Interface::guards()` 同一条纪律：`/mavros/state` 一帧都没到时
/// `armed` 取 false，否则启动瞬间会伪造出一个"异常解锁"告警。
/// 它不会造成"没数据也放行"—— 那种情况下 `connected` 已经把门关上了。
TEST(GuardList, 全零守卫里未解锁是成立的)
{
  control::Guards g;
  EXPECT_TRUE(control::guard_holds(g, control::Guard::kArmingByUs));
  EXPECT_EQ(
    names(control::missing_guards(g)),
    (std::vector<std::string>{"connected", "mocap_fresh", "local_pos_fresh"}));
  EXPECT_FALSE(control::ready_to_stream(g));
}

/// `kArmingByUs` = 「未解锁 ∨ 本节点解锁的」——
/// 这条守卫的意义是堵住"别人解锁了"（§H.6），不是"禁止解锁"（ticket 08 要自己解锁）。
TEST(GuardList, 解锁来自本节点时守卫成立)
{
  control::Guards g = all_ok();
  EXPECT_TRUE(control::guard_holds(g, control::Guard::kArmingByUs));   // 未解锁
  g.armed = true;
  EXPECT_FALSE(control::guard_holds(g, control::Guard::kArmingByUs));  // 外部解锁
  g.own_arming = true;
  EXPECT_TRUE(control::guard_holds(g, control::Guard::kArmingByUs));   // 本节点解锁
}

TEST(GuardList, 状态名进control_state话题)
{
  EXPECT_STREQ(control::to_string(control::Authority::kManual), "MANUAL");
  EXPECT_STREQ(control::to_string(control::Authority::kAuto), "AUTO");
  EXPECT_STREQ(control::to_string(control::Phase::kInit), "INIT");
  EXPECT_STREQ(control::to_string(control::Phase::kStandby), "STANDBY");
  EXPECT_STREQ(control::to_string(control::Phase::kLand), "LAND");
  EXPECT_STREQ(control::to_string(control::Phase::kRcOverride), "RC_OVERRIDE");
}

/// 降级原因的名字要**进日志**（`degrade=<reason>`）⟹ 每个成员都必须有稳定、互不相同的名字。
/// ★ 用 `all_degrades()` 遍历（唯一出处）⟹ 新增一个 `Degrade` 而漏了 `to_string` 会在
/// `-Wswitch` 上编译失败，漏了 `all_degrades()` 会因为这条断言少了一项而红。
TEST(DegradeNames, 每个降级原因都有可grep的名字)
{
  std::vector<std::string> tokens;
  for (control::Degrade d : control::all_degrades()) {
    const std::string token = control::to_string(d);
    EXPECT_FALSE(token.empty()) << "有成员没有名字";
    EXPECT_NE(token, "?") << "`to_string` 没覆盖这个成员";
    tokens.push_back(token);
  }
  EXPECT_EQ(tokens.size(), control::all_degrades().size());
  for (size_t i = 1; i < tokens.size(); ++i) {
    for (size_t j = 0; j < i; ++j) {
      EXPECT_NE(tokens[i], tokens[j]) << "名字重复：" << tokens[i];
    }
  }
  EXPECT_EQ(control::to_string(control::Degrade::kNone), "none");
  EXPECT_EQ(control::to_string(control::Degrade::kBatteryLow), "battery_low");
}

// ─────────────────────────── 状态机：放行与前置流（ticket 05） ───────────────────────────

TEST(Fsm, 守卫不成立时退回INIT且停发setpoint)
{
  control::Fsm fsm(1.0);
  control::Guards g = all_ok();
  g.mocap_fresh = false;

  const control::Decision d = fsm.update(g, health_ok(), 100.0);
  EXPECT_EQ(d.phase, control::Phase::kInit);
  EXPECT_EQ(d.authority, control::Authority::kManual);
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_FALSE(d.request_offboard);
  EXPECT_FALSE(d.request_land);
  EXPECT_EQ(d.degrade, control::Degrade::kNone);
}

/// ★ 「还没起完」不是「失效」：一帧都没流过时，动捕没数据只是 INIT 的常态。
/// 不区分这两件事的话，节点每次启动都会在日志里报一条降级原因（那是假警报，
/// 会让人对真警报麻木）。
TEST(Fsm, 没流过之前不报降级原因)
{
  control::Fsm fsm(1.0);
  control::Guards g = all_ok();
  g.mocap_fresh = false;   // 动捕节点还没起来

  EXPECT_EQ(fsm.update(g, health_ok(), 100.0).degrade, control::Degrade::kNone);
}

TEST(Fsm, 守卫成立即进STANDBY并开始流但控制权仍是MANUAL)
{
  control::Fsm fsm(1.0);
  const control::Decision d = fsm.update(all_ok(), health_ok(), 100.0);

  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_TRUE(d.stream_setpoint);
  // ★ 未解锁 ⟹ 不算交出控制权（README「两个正交维度」的定义）。
  EXPECT_EQ(d.authority, control::Authority::kManual);
  // 刚流了 0 s，前置流时长没到 ⟹ 不许切。
  EXPECT_FALSE(d.request_offboard);
}

TEST(Fsm, 解锁且守卫成立时控制权是AUTO)
{
  control::Fsm fsm(0.0);
  const control::Decision d = fsm.update(armed_ok(), health_ok(), 100.0);

  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_EQ(d.authority, control::Authority::kAuto);
  EXPECT_TRUE(d.stream_setpoint);
}

/// 外部解锁（`own_arming = false`）⟹ 守卫不成立 ⟹ 停发（§H.6），且**不是**降级。
TEST(Fsm, 外部解锁停发但不记降级原因)
{
  control::Fsm fsm(0.0);
  fsm.update(all_ok(), health_ok(), 100.0);   // 先流起来
  control::Guards g = all_ok();
  g.armed = true;                             // 解锁不是我们干的
  const control::Decision d = fsm.update(g, health_ok(), 101.0);

  EXPECT_EQ(d.phase, control::Phase::kInit);
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_EQ(d.degrade, control::Degrade::kNone) << "外部解锁不是『输入失效』，原因由 ERROR 日志说";
  EXPECT_FALSE(d.request_land) << "未解锁 + 没有失效 ⟹ 不该请求 AUTO.LAND";
}

/// 前置流时长的**两侧**都断言：差一点点不许切、到了才许切。
TEST(Fsm, 前置流时长两侧)
{
  control::Fsm fsm(3.0);
  fsm.update(all_ok(), health_ok(), 100.0);

  EXPECT_FALSE(fsm.update(all_ok(), health_ok(), 102.999).request_offboard) << "还没流够就切了";
  EXPECT_TRUE(fsm.update(all_ok(), health_ok(), 103.0).request_offboard) << "流够了却没切";
}

TEST(Fsm, 已经在OFFBOARD里就不再请求切模式)
{
  control::Fsm fsm(1.0);
  fsm.update(all_ok(), health_ok(), 100.0);

  control::Guards g = all_ok();
  g.in_offboard = true;
  EXPECT_FALSE(fsm.update(g, health_ok(), 200.0).request_offboard);
}

/// ★ 断流之后**重新计时**：守卫一断一续，前置流必须从头算。
/// 不重置的话，"断流前的旧时长"会让切模式发生在流刚恢复的下一帧 —— 而 PX4 那边
/// 看到的是一条刚续上的流，`offboard_control_mode` 还没重新连上。
TEST(Fsm, 断流后前置流重新计时)
{
  control::Fsm fsm(1.0);
  fsm.update(all_ok(), health_ok(), 100.0);
  EXPECT_TRUE(fsm.update(all_ok(), health_ok(), 150.0).request_offboard);  // 流够 50 s

  control::Guards bad = all_ok();
  bad.connected = false;
  fsm.update(bad, health_ok(), 151.0);       // 断流
  EXPECT_DOUBLE_EQ(fsm.streaming_seconds(151.0), 0.0);

  fsm.update(all_ok(), health_ok(), 152.0);  // 恢复
  EXPECT_FALSE(fsm.update(all_ok(), health_ok(), 152.5).request_offboard) << "恢复后没重新计时";
  EXPECT_TRUE(fsm.update(all_ok(), health_ok(), 153.0).request_offboard);
}

TEST(Fsm, 观测口在没流时报零)
{
  control::Fsm fsm(1.0);
  EXPECT_DOUBLE_EQ(fsm.streaming_seconds(100.0), 0.0);

  control::Guards bad = all_ok();
  bad.local_pos_fresh = false;
  fsm.update(bad, health_ok(), 100.0);
  EXPECT_DOUBLE_EQ(fsm.streaming_seconds(130.0), 0.0);
}

// ─────────────────────────── 失效判据的优先级 ───────────────────────────

TEST(FirstDegrade, 一切正常时没有降级原因)
{
  EXPECT_EQ(
    control::first_degrade(all_ok(), health_ok(), control::PrevFrame{}), control::Degrade::kNone);
}

TEST(FirstDegrade, 三条输入超时各有各的原因)
{
  control::Guards g = all_ok();
  g.connected = false;
  EXPECT_EQ(
    control::first_degrade(g, health_ok(), control::PrevFrame{}), control::Degrade::kStateTimeout);

  g = all_ok();
  g.mocap_fresh = false;
  EXPECT_EQ(
    control::first_degrade(g, health_ok(), control::PrevFrame{}), control::Degrade::kMocapTimeout);

  g = armed_ok();
  control::Health h = health_ok();
  h.rc_fresh = false;
  EXPECT_EQ(control::first_degrade(g, h, control::PrevFrame{}), control::Degrade::kRcTimeout);
}

/// ★ 遥控器静默**只在已解锁时**算失效：未解锁时链路断着不影响任何事
/// （而一旦解锁它就是"首飞前遥控必须开机且链路已建立"的执行者，§H.7.1）。
TEST(FirstDegrade, 遥控器静默在未解锁时不算失效)
{
  control::Health h = health_ok();
  h.rc_fresh = false;
  EXPECT_EQ(control::first_degrade(all_ok(), h, control::PrevFrame{}), control::Degrade::kNone);
}

/// 同时成立时报**第一个**（顺序即优先级，改了会让日志里的原因变得不可比）。
TEST(FirstDegrade, 同时成立时报优先级最高的那个)
{
  control::Guards g = all_ok();
  g.connected = false;
  g.mocap_fresh = false;
  EXPECT_EQ(
    control::first_degrade(g, health_ok(), control::PrevFrame{}), control::Degrade::kStateTimeout);

  // 低压 + 动捕超时：报超时（低压还能飞，超时已经瞎了）。
  g = all_ok();
  g.mocap_fresh = false;
  control::Health h = health_ok();
  h.battery_low = true;
  EXPECT_EQ(control::first_degrade(g, h, control::PrevFrame{}), control::Degrade::kMocapTimeout);
}

/// ★ NaN 的位置排在"位置超时"之前：NaN 那一帧就已经是失效（边界层对 NaN 帧不刷新新鲜度），
/// 0.5 s 后两者同时成立 —— 报"超时"就把真正的根因（EKF2 给出了 NaN）丢掉了。
TEST(FirstDegrade, 位置NaN优先于位置超时)
{
  control::Guards g = all_ok();
  g.local_pos_fresh = false;      // 0.5 s 之后的必然结果
  control::Health h = health_ok();
  h.position_invalid = true;      // 真正的原因
  EXPECT_EQ(control::first_degrade(g, h, control::PrevFrame{}), control::Degrade::kPositionInvalid);
}

/// 打杆只在已解锁时算夺权（PX4 亦然，`Commander.cpp:2740` 的解锁门）。
TEST(FirstDegrade, 未解锁时打杆不算夺权)
{
  control::Health h = health_ok();
  h.rc_sticks_moving = true;
  EXPECT_EQ(control::first_degrade(all_ok(), h, control::PrevFrame{}), control::Degrade::kNone);
  EXPECT_EQ(
    control::first_degrade(armed_ok(), h, control::PrevFrame{}), control::Degrade::kRcOverride);
}

/// ★★ OFFBOARD 的下降沿**只在"上一帧我们在流"时**才算夺权（`PrevFrame` 字段序：
/// `mode_ours, streaming, in_land`）。2026-09-22 复核时补的 —— 少了它，真机上每一次输入失效
/// 都会在 1 s 后被误报成"有人接管"（判据 0.5 s < `COM_OF_LOSS_T` 1.0 s，见 `first_degrade`）。
TEST(FirstDegrade, OFFBOARD的下降沿只在我们在流时才算夺权)
{
  control::Guards kicked_out = all_ok();          // mode 已经不是我们的了

  const control::PrevFrame streaming{true, true, false};
  EXPECT_EQ(
    control::first_degrade(kicked_out, health_ok(), streaming), control::Degrade::kModeLost);

  const control::PrevFrame stopped{true, false, false};   // 我们自己停发之后
  EXPECT_EQ(
    control::first_degrade(kicked_out, health_ok(), stopped), control::Degrade::kNone)
    << "PX4 因断流自己退出 OFFBOARD 不是「有人接管」，报它会把排查方向带偏";

  // 从 `AUTO.LAND` 被切走：断流不会让它退出 ⟹ 只可能是人干的 ⟹ 永远是夺权。
  const control::PrevFrame was_land{true, false, true};
  EXPECT_EQ(
    control::first_degrade(kicked_out, health_ok(), was_land), control::Degrade::kModeLost);
}

// ─────────────────────────── 失效处置：未解锁 vs 已解锁 ───────────────────────────

/// ★ 未解锁时只是"暂时不能发"：回 INIT、不请求 LAND。
/// 依据（第一手）：`AUTO.LAND` 在 `mode_req_prevent_arming` 名单里（`mode_requirements.cpp:142`）
/// ⟹ 未解锁就切进去，台架上会**解不了锁**。
TEST(Fsm, 未解锁时的失效只停发不请求降落)
{
  control::Fsm fsm(0.0);
  fsm.update(all_ok(), health_ok(), 100.0);

  control::Guards g = all_ok();
  g.mocap_fresh = false;
  const control::Decision d = fsm.update(g, health_ok(), 101.0);

  EXPECT_EQ(d.phase, control::Phase::kInit);
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_FALSE(d.request_land) << "未解锁时不该请求 AUTO.LAND";
  EXPECT_EQ(d.degrade, control::Degrade::kMocapTimeout);
}

/// ★★ "可复位，不是单向 latch"的**主证据**：未解锁时的降级不闩锁 ⟹ 判据恢复即回 STANDBY。
TEST(Fsm, 未解锁时的失效在判据恢复后自动复位)
{
  control::Fsm fsm(0.0);
  fsm.update(all_ok(), health_ok(), 100.0);

  control::Guards bad = all_ok();
  bad.mocap_fresh = false;
  fsm.update(bad, health_ok(), 101.0);
  EXPECT_FALSE(fsm.latched()) << "未解锁的失效不该闩锁";

  const control::Decision d = fsm.update(all_ok(), health_ok(), 102.0);
  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_TRUE(d.stream_setpoint);
  EXPECT_EQ(d.degrade, control::Degrade::kNone);
}

TEST(Fsm, 已解锁时的失效停发并请求降落)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health h = health_ok();
  h.battery_low = true;
  const control::Decision d = fsm.update(armed_ok(), h, 101.0);

  EXPECT_EQ(d.phase, control::Phase::kLand);
  EXPECT_EQ(d.authority, control::Authority::kManual) << "我们已经停发 ⟹ 不是我们在驱动飞机";
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_TRUE(d.request_land);
  EXPECT_FALSE(d.request_offboard);
  EXPECT_EQ(d.degrade, control::Degrade::kBatteryLow);
  EXPECT_TRUE(fsm.latched());
}

/// ★ 已解锁的失效**不允许空中自动恢复**：任务已经中止（请求了 AUTO.LAND），
/// 输入自己好了就悄悄把控制权抢回来是危险的（飞行员不期待、降落中断在半空）。
TEST(Fsm, 已解锁时的失效不因输入恢复而空中自动接管)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health h = health_ok();
  h.battery_low = true;
  fsm.update(armed_ok(), h, 101.0);

  const control::Decision d = fsm.update(armed_ok(), health_ok(), 110.0);   // 电压"回来了"
  EXPECT_EQ(d.phase, control::Phase::kLand);
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_TRUE(d.request_land);
  EXPECT_EQ(d.degrade, control::Degrade::kBatteryLow) << "闩锁期间保持**首因**（最接近根因）";
}

/// ★ 复位主路径：**落地上锁**（`armed` true→false）⟹ 闩锁解除，一切重来。
TEST(Fsm, 已解锁的失效在落地上锁之后复位)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health h = health_ok();
  h.battery_low = true;
  fsm.update(armed_ok(), h, 101.0);

  control::Guards landed = all_ok();               // armed = false（落地上锁）
  EXPECT_FALSE(fsm.update(landed, h, 200.0).request_land) << "落地之后不该再请求降落";
  EXPECT_FALSE(fsm.latched());

  // 而且电压仍然低 ⟹ 未解锁下只是"暂时不能发"（回 INIT），不是单向 latch。
  const control::Decision d = fsm.update(landed, h, 201.0);
  EXPECT_EQ(d.phase, control::Phase::kInit);
  EXPECT_EQ(d.degrade, control::Degrade::kBatteryLow);

  // 电压恢复 ⟹ 自动回 STANDBY（可复位）。
  const control::Decision back = fsm.update(landed, health_ok(), 202.0);
  EXPECT_EQ(back.phase, control::Phase::kStandby);
  EXPECT_TRUE(back.stream_setpoint);
}

/// ★ 已在 LAND 里（低压闩锁）时飞行员打杆 ⟹ **升级成夺权**：交还控制权、**停发
/// `AUTO.LAND` 请求**。飞行员永远优先，不能跟他抢。
TEST(Fsm, 低压闩锁之后打杆会升级成夺权)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health low = health_ok();
  low.battery_low = true;
  EXPECT_EQ(fsm.update(armed_ok(), low, 101.0).phase, control::Phase::kLand);

  control::Health both = low;          // 电压仍然低，而且飞行员在打杆
  both.rc_sticks_moving = true;
  const control::Decision d = fsm.update(armed_ok(), both, 102.0);

  EXPECT_EQ(d.phase, control::Phase::kRcOverride) << "飞行员接管时不该还停在 LAND";
  EXPECT_FALSE(d.request_land) << "★ 已经在请求降落，飞行员一打杆就必须停止请求（否则每秒一次的模式拉锯）";
  EXPECT_EQ(d.degrade, control::Degrade::kRcOverride) << "日志要给飞行员优先的**当前**原因";
  EXPECT_FALSE(d.stream_setpoint);
}

/// 反向不成立：夺权闩锁**不能**被普通的输入失效"降级"（飞机的控制权不在我们手上，
/// 报一个输入超时会把排查方向带偏）。
TEST(Fsm, 夺权闩锁不会被输入超时改写)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health moving = health_ok();
  moving.rc_sticks_moving = true;
  fsm.update(armed_ok(), moving, 101.0);

  control::Health muted = health_ok();
  muted.rc_fresh = false;                       // 连遥控器也断了
  const control::Decision d = fsm.update(armed_ok(), muted, 102.0);
  EXPECT_EQ(d.degrade, control::Degrade::kRcOverride);
  EXPECT_EQ(d.phase, control::Phase::kRcOverride);
  EXPECT_TRUE(fsm.latched());
}

/// ★★ 我们自己请求的 `AUTO.LAND` 生效**不是**"被外部夺走"。
/// 少了这条区分，降落会变成一场自导自演的闹剧：请求被接受 ⟹ `in_offboard` 掉 ⟹ 状态机
/// 判"模式被夺走" ⟹ 交还控制权给不存在的飞行员 + **停止重试降落请求**。
TEST(Fsm, 自己请求的降落模式生效不算夺权)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health low = health_ok();
  low.battery_low = true;
  const control::Decision asked = fsm.update(armed_ok(), low, 101.0);
  EXPECT_TRUE(asked.request_land);

  control::Guards landed_mode = armed_ok();   // PX4 接受了：mode == AUTO.LAND
  landed_mode.in_land = true;
  const control::Decision d = fsm.update(landed_mode, low, 101.2);

  EXPECT_EQ(d.degrade, control::Degrade::kBatteryLow) << "报的还该是最初的原因，不是 mode_lost";
  EXPECT_EQ(d.phase, control::Phase::kLand);
  EXPECT_FALSE(d.request_land) << "已经在降落模式里 ⟹ 不再重发请求（重试是留给『被拒』的）";
  EXPECT_EQ(d.authority, control::Authority::kManual);
}

/// 反过来：已经在我们请求的降落模式里，模式又被别人切走 ⟹ 夺权（停止请求、交还控制权）。
TEST(Fsm, 降落中模式被切走则交还控制权)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health low = health_ok();
  low.battery_low = true;
  fsm.update(armed_ok(), low, 101.0);
  control::Guards landed_mode = armed_ok();
  landed_mode.in_land = true;
  fsm.update(landed_mode, low, 101.2);

  const control::Decision d = fsm.update(armed_ok(), low, 102.0);   // POSCTL（别人切的）
  EXPECT_EQ(d.phase, control::Phase::kRcOverride) << "模式被切走 ⟹ 交还控制权";
  EXPECT_FALSE(d.request_land) << "★ 不许再抢：已经在降落，别人接管了就不再请求任何模式";
  EXPECT_EQ(d.degrade, control::Degrade::kModeLost);
  EXPECT_TRUE(fsm.latched()) << "夺权要闩锁（复位靠外部把模式放回来，或落地上锁）";
}

/// ★★ 与上一条成对，也是**真机上必然发生**的那个场面：我们自己停发之后，PX4 因断流
/// （`COM_OF_LOSS_T` 1.0 s）**自己**退出了 OFFBOARD。这时：
///   · **不能**报成夺权 —— 没有飞行员，报错了日志会把排查方向指向一个不存在的人，
///     而且 `is_takeover` 会升级闩锁、把 `AUTO.LAND` 的重试停掉（ticket 06 要的正是"输入坏了
///     就让它降落"，请求被拒时**重试**才是拿到降落的方式）；
///   · 且要**继续**请求降落（`in_land` 仍为假 ⟹ `request_land` 为真）。
/// ⚠️ 夹具看不见这个场面（假 mavros 不假扮 PX4 的断流退出，见 `CONTEXT.md`）⟹ 证据只能在这里。
TEST(Fsm, 停发之后PX4自己退出OFFBOARD不算夺权_继续请求降落)
{
  control::Fsm fsm(0.0);
  control::Guards inside = armed_ok();
  inside.in_offboard = true;
  fsm.update(inside, health_ok(), 100.0);           // 已解锁、在 OFFBOARD 里流
  EXPECT_GT(fsm.streaming_seconds(100.5), 0.0)
    << "正控：第 100 帧我们确实在流 —— 下面这个下降沿的定性全靠它（见 `PrevFrame::streaming`）";

  control::Health low = health_ok();
  low.battery_low = true;
  const control::Decision first = fsm.update(inside, low, 101.0);
  EXPECT_EQ(first.phase, control::Phase::kLand);
  EXPECT_TRUE(first.request_land);

  // 1 s 之后 PX4 自己退出 OFFBOARD（我们的流早就停了 ⟹ `in_offboard` 变假）
  const control::Guards out = armed_ok();
  const control::Decision d = fsm.update(out, low, 102.0);
  EXPECT_EQ(d.degrade, control::Degrade::kBatteryLow)
    << "报的该是最初的原因（低压），不是 mode_lost —— 没人接管，是我们自己停发的结果";
  EXPECT_EQ(d.phase, control::Phase::kLand) << "任务已中止 ⟹ 仍在降落任务态";
  EXPECT_TRUE(d.request_land) << "★ 还要继续请求降落（被拒的话重试是唯一拿到降落的方式）";
  EXPECT_EQ(fsm.latched_reason(), control::Degrade::kBatteryLow) << "闩锁原因不许被改名";
  EXPECT_FALSE(d.stream_setpoint);
}

/// ★★ 同一条规则的**未解锁**版本，守的是 ticket 06 验收项 ⑤ 的"可复位"：
/// 未解锁时停发 ⟹ PX4 自己退出 OFFBOARD ⟹ 若误判成夺权，闩锁就变成"等有人把模式放回来"，
/// 而本张承诺的是"判据一恢复就自动回 STANDBY"（台架上这条一直是绿的，因为夹具不会自退）。
TEST(Fsm, 未解锁时停发后PX4自己退出OFFBOARD_判据恢复即自动复位)
{
  control::Fsm fsm(0.0);
  control::Guards inside = all_ok();                // 未解锁但已在 OFFBOARD 里流
  inside.in_offboard = true;
  fsm.update(inside, health_ok(), 100.0);
  EXPECT_GT(fsm.streaming_seconds(100.5), 0.0) << "正控：第 100 帧确实在流";

  control::Guards g = inside;
  g.mocap_fresh = false;                            // 动捕失效 ⟹ 停发（未解锁 ⟹ 不请求 LAND）
  const control::Decision d = fsm.update(g, health_ok(), 101.0);
  EXPECT_EQ(d.phase, control::Phase::kInit);
  EXPECT_FALSE(d.request_land);
  EXPECT_FALSE(fsm.latched()) << "未解锁的失效不闩锁";

  // PX4 自己退出 OFFBOARD（我们的流停了 1 s 之后）—— 这一帧不许改变上面任何一条。
  control::Guards out = g;                          // 动捕仍然失效
  out.in_offboard = false;                          // ★ 关键：mode 已经不是我们持有的了
  const control::Decision after = fsm.update(out, health_ok(), 102.0);
  EXPECT_EQ(after.degrade, control::Degrade::kMocapTimeout) << "还是最初的原因";
  EXPECT_FALSE(fsm.latched()) << "★ 不许因为 PX4 自退就闩锁成夺权（那样就只能靠人把模式放回来）";

  // 判据恢复 ⟹ 自动回 STANDBY 并重新流（"可复位"的正面）。
  const control::Decision back = fsm.update(inside, health_ok(), 103.0);
  EXPECT_EQ(back.phase, control::Phase::kStandby);
  EXPECT_TRUE(back.stream_setpoint);
  EXPECT_EQ(back.degrade, control::Degrade::kNone);
}

// ─────────────────────────── 控制权被夺走（打杆 / 模式） ───────────────────────────

/// ★★ ticket 06 验收项 1：打杆 ⟹ **立刻停发** + 不请求 LAND + 不请求 OFFBOARD。
TEST(Fsm, 打杆时进RC_OVERRIDE并停发)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health h = health_ok();
  h.rc_sticks_moving = true;
  const control::Decision d = fsm.update(armed_ok(), h, 101.0);

  EXPECT_EQ(d.phase, control::Phase::kRcOverride);
  EXPECT_EQ(d.authority, control::Authority::kManual);
  EXPECT_FALSE(d.stream_setpoint) << "只是『不更新目标』不够 —— 必须真的停发，否则操作员拉不回来";
  EXPECT_FALSE(d.request_offboard);
  EXPECT_FALSE(d.request_land) << "控制权已交还飞行员，不该再请求任何模式";
  EXPECT_EQ(d.degrade, control::Degrade::kRcOverride);
}

/// ★★ 本张最关键的一条"不做"：夺权之后**不许抢回 OFFBOARD**。
/// 摇杆回中、一切输入健康、飞机也没进 OFFBOARD —— 但那是飞行员的地盘，不是我们的。
TEST(Fsm, 夺权之后不抢回OFFBOARD)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health moving = health_ok();
  moving.rc_sticks_moving = true;
  fsm.update(armed_ok(), moving, 101.0);

  // 摇杆回中（不再在动），模式是 POSCTL（假 PX4 被夺权后的样子）
  control::Guards g = armed_ok();
  g.in_offboard = false;
  for (double t = 102.0; t < 130.0; t += 0.5) {
    const control::Decision d = fsm.update(g, health_ok(), t);
    EXPECT_FALSE(d.request_offboard) << "t=" << t << "：夺权之后又去抢 OFFBOARD 了";
    EXPECT_FALSE(d.stream_setpoint);
    EXPECT_EQ(d.phase, control::Phase::kRcOverride);
  }
}

/// ★ 复位路径：**有人把模式放回 OFFBOARD**（闩锁期间我们从不自己请求 ⟹ 只可能是外部动作）。
TEST(Fsm, 外部把模式放回OFFBOARD即复位)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health moving = health_ok();
  moving.rc_sticks_moving = true;
  control::Guards out = armed_ok();
  out.in_offboard = false;
  fsm.update(out, moving, 101.0);
  EXPECT_TRUE(fsm.latched());

  control::Guards back = armed_ok();
  back.in_offboard = true;                    // 有人把它放回去了
  const control::Decision d = fsm.update(back, health_ok(), 102.0);
  EXPECT_FALSE(fsm.latched());
  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_TRUE(d.stream_setpoint);
  EXPECT_EQ(d.degrade, control::Degrade::kNone);
}

/// 模式被夺走的**权威路径**：曾在 OFFBOARD，现在不是了（`state.mode != "OFFBOARD"`）。
TEST(Fsm, 模式掉出OFFBOARD时进RC_OVERRIDE)
{
  control::Fsm fsm(0.0);
  control::Guards inside = all_ok();
  inside.in_offboard = true;
  fsm.update(inside, health_ok(), 100.0);     // 已经切进去了

  control::Guards kicked_out = all_ok();      // mode 变了（POSCTL）
  const control::Decision d = fsm.update(kicked_out, health_ok(), 101.0);

  EXPECT_EQ(d.phase, control::Phase::kRcOverride);
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_FALSE(d.request_offboard) << "模式被夺走之后不许抢回来";
  EXPECT_EQ(d.degrade, control::Degrade::kModeLost);
}

/// ★ 首因优先：打杆（先）→ 模式随后掉出 OFFBOARD（`/mavros/state` 1 Hz，最多晚 1 s）⟹
/// 报"打杆"而不是"模式被夺走"。两个原因指向不同的排查方向，报错了会浪费现场时间。
TEST(Fsm, 打杆之后模式才掉出来仍然报打杆)
{
  control::Fsm fsm(0.0);
  control::Guards inside = armed_ok();
  inside.in_offboard = true;
  fsm.update(inside, health_ok(), 100.0);

  control::Health moving = health_ok();
  moving.rc_sticks_moving = true;
  fsm.update(inside, moving, 100.05);         // 我们先从摇杆上判出夺权

  control::Guards kicked_out = armed_ok();    // 1 s 后 state 才反映出来
  const control::Decision d = fsm.update(kicked_out, health_ok(), 101.0);
  EXPECT_EQ(d.degrade, control::Degrade::kRcOverride);
  EXPECT_EQ(d.phase, control::Phase::kRcOverride);
}

/// 未解锁时模式被夺走（台架上有人拨了模式开关）⟹ 一样"不抢回来"，
/// 直到有人把模式放回去（或节点重启）。
TEST(Fsm, 未解锁时模式被夺走也不抢回来)
{
  control::Fsm fsm(0.0);
  control::Guards inside = all_ok();
  inside.in_offboard = true;
  fsm.update(inside, health_ok(), 100.0);

  control::Guards kicked_out = all_ok();
  const control::Decision d = fsm.update(kicked_out, health_ok(), 101.0);
  EXPECT_EQ(d.phase, control::Phase::kRcOverride);
  EXPECT_FALSE(d.request_offboard);
  EXPECT_EQ(d.degrade, control::Degrade::kModeLost);
}

/// 我们自己请求 OFFBOARD 成功的那一帧**不是**"模式被夺走"（`in_offboard` 是上升沿）。
TEST(Fsm, 模式进入OFFBOARD不算被夺走)
{
  control::Fsm fsm(0.0);
  fsm.update(all_ok(), health_ok(), 100.0);

  control::Guards inside = all_ok();
  inside.in_offboard = true;
  const control::Decision d = fsm.update(inside, health_ok(), 100.1);
  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_EQ(d.degrade, control::Degrade::kNone);
}

// ─────────────────────────── 动捕时间戳不前进 ───────────────────────────

/// "有帧但时间戳不前进"是**话题层面看不见**的失效：mavros 会丢弃这类 vision_pose
/// （`send_vision_estimate()` 里戳相等即 return），EKF2 也会丢过旧的样本。
/// ⟹ 我们的新鲜度判据（看到达时刻）对它无感，必须单独判。
TEST(Fsm, 动捕时间戳不前进时降级)
{
  control::Fsm fsm(0.0);
  fsm.update(armed_ok(), health_ok(), 100.0);

  control::Health stalled = health_ok();
  stalled.mocap_stamp_stalled = true;
  const control::Decision d = fsm.update(armed_ok(), stalled, 101.0);

  EXPECT_EQ(d.degrade, control::Degrade::kMocapStampStalled);
  EXPECT_EQ(d.phase, control::Phase::kLand);
  EXPECT_TRUE(d.request_land);
}
