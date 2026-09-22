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

/// 已解锁的场面 —— ★ **且这次解锁是本节点请求的**（ticket 08 起 `own_arming` 由
/// `/mavros/cmd/arming` 的应答决定；这里直接构造那个结果，`own_arming=false` 的用例见
/// `外部解锁` 一条）。
control::Guards armed_ok()
{
  control::Guards g = all_ok();
  g.armed = true;
  g.own_arming = true;
  return g;
}

/// 已解锁 **∧ 这次解锁是我们的（`own_arming`）∧ 已在 OFFBOARD`** —— ticket 08 的爬升/悬停场面，
/// ticket 18 的"链路静默"场面也用它（真飞起来时我们必然在 OFFBOARD 里：切进 OFFBOARD 本来就是
/// 解锁的前置）⟹ 少了这一位，用例就少测了一个真实存在的输入。
/// ★ ticket 18 时从 ticket 08 那段**上移**到这里：它现在两张票的用例都在用，
///   而 C++ 的 free function 必须先声明后使用（原位置在文件下半部分）。
control::Guards armed_offboard()
{
  control::Guards g = armed_ok();
  g.in_offboard = true;
  return g;
}

/// **mavros 那帧合成断连状态**的逐字形状（ticket 18）—— `publish_disconnection()` 的那几个字段
/// 全是**常量**、与夹具/真机的实际状态无关（真 mavros 这时也联系不上飞控）。
/// ★ 与 `synthetic_disconnect` 相关的场景一律用它构造，别各写一遍：这一帧的要点正是
///   "字段是占位的"，散着构造迟早有人顺手写成"当前状态的拷贝"。
control::Guards disconnect_frame()
{
  control::Guards g = all_ok();
  g.connected = false;
  g.armed = false;
  g.in_offboard = false;   // `mode = ""` ⟹ 既不是 OFFBOARD 也不是我们请求的 AUTO.LAND
  return g;
}

/// 一切输入都健康（`Health` 的默认值就是"没有坏消息"）。
control::Health health_ok()
{
  control::Health h;
  h.rc_fresh = true;
  return h;
}

/// 一帧"什么都正常"的导航读数：位置在原点、速度为零、机头朝 +x。
/// ★ `Odometry` 的默认值是 **`position_valid = false`**（与 `Guards`/`Health` 同一条纪律：
///   "没数据"绝不能当成好消息）⟹ 每个用例都必须**显式**给出可用的读数，没有隐式默认。
control::Odometry odom_ok()
{
  control::Odometry o;
  o.position_valid = true;
  o.velocity_valid = true;
  return o;
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
  // ★★ token 必须落在**文档里那张 grep 词表**内：`README.md` 与
  //   `docs/control/phase1-implementation.md` §4.4 给现场的方子是
  //   `grep -oE "degrade=[a-z0-9_]+"`。两处必须同一个词表 ——
  //   ticket 16 的 `ekf2_pos_lost` 是第一个**带数字**的 token，而当时那份方子写的是
  //   `[a-z_]+`：照它捞只会捞到 `ekf`，"日志里明明有这条降级、方子却捞不出来"。
  //   这条断言就是防它再走散（改词表要同时改这里与那两处文档）。
  for (const std::string & token : tokens) {
    for (char c : token) {
      const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || (c == '_');
      EXPECT_TRUE(ok) << "token 跑到文档的词表 [a-z0-9_] 之外了：" << token;
    }
  }
  EXPECT_EQ(control::to_string(control::Degrade::kNone), "none");
  EXPECT_EQ(control::to_string(control::Degrade::kBatteryLow), "battery_low");
  EXPECT_EQ(control::to_string(control::Degrade::kEkf2PosLost), "ekf2_pos_lost");
}

// ─────────────────────────── 状态机：放行与前置流（ticket 05） ───────────────────────────

TEST(Fsm, 守卫不成立时退回INIT且停发setpoint)
{
  control::Fsm fsm(1.0, control::TaskLimits{});
  control::Guards g = all_ok();
  g.mocap_fresh = false;

  const control::Decision d = fsm.update(g,  health_ok(), odom_ok(),  100.0);
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
  control::Fsm fsm(1.0, control::TaskLimits{});
  control::Guards g = all_ok();
  g.mocap_fresh = false;   // 动捕节点还没起来

  EXPECT_EQ(fsm.update(g,  health_ok(), odom_ok(),  100.0).degrade, control::Degrade::kNone);
}

TEST(Fsm, 守卫成立即进STANDBY并开始流但控制权仍是MANUAL)
{
  control::Fsm fsm(1.0, control::TaskLimits{});
  const control::Decision d = fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);

  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_TRUE(d.stream_setpoint);
  // ★ 未解锁 ⟹ 不算交出控制权（README「两个正交维度」的定义）。
  EXPECT_EQ(d.authority, control::Authority::kManual);
  // 刚流了 0 s，前置流时长没到 ⟹ 不许切。
  EXPECT_FALSE(d.request_offboard);
}

TEST(Fsm, 解锁且守卫成立时控制权是AUTO)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  const control::Decision d = fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_EQ(d.authority, control::Authority::kAuto);
  EXPECT_TRUE(d.stream_setpoint);
}

/// 外部解锁（`own_arming = false`）⟹ 守卫不成立 ⟹ 停发（§H.6），且**不是**降级。
TEST(Fsm, 外部解锁停发但不记降级原因)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);   // 先流起来
  control::Guards g = all_ok();
  g.armed = true;                             // 解锁不是我们干的
  const control::Decision d = fsm.update(g,  health_ok(), odom_ok(),  101.0);

  EXPECT_EQ(d.phase, control::Phase::kInit);
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_EQ(d.degrade, control::Degrade::kNone) << "外部解锁不是『输入失效』，原因由 ERROR 日志说";
  EXPECT_FALSE(d.request_land) << "未解锁 + 没有失效 ⟹ 不该请求 AUTO.LAND";
}

/// 前置流时长的**两侧**都断言：差一点点不许切、到了才许切。
TEST(Fsm, 前置流时长两侧)
{
  control::Fsm fsm(3.0, control::TaskLimits{});
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);

  EXPECT_FALSE(fsm.update(all_ok(),  health_ok(), odom_ok(),  102.999).request_offboard) << "还没流够就切了";
  EXPECT_TRUE(fsm.update(all_ok(),  health_ok(), odom_ok(),  103.0).request_offboard) << "流够了却没切";
}

TEST(Fsm, 已经在OFFBOARD里就不再请求切模式)
{
  control::Fsm fsm(1.0, control::TaskLimits{});
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);

  control::Guards g = all_ok();
  g.in_offboard = true;
  EXPECT_FALSE(fsm.update(g,  health_ok(), odom_ok(),  200.0).request_offboard);
}

/// ★ 断流之后**重新计时**：守卫一断一续，前置流必须从头算。
/// 不重置的话，"断流前的旧时长"会让切模式发生在流刚恢复的下一帧 —— 而 PX4 那边
/// 看到的是一条刚续上的流，`offboard_control_mode` 还没重新连上。
TEST(Fsm, 断流后前置流重新计时)
{
  control::Fsm fsm(1.0, control::TaskLimits{});
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);
  EXPECT_TRUE(fsm.update(all_ok(),  health_ok(), odom_ok(),  150.0).request_offboard);  // 流够 50 s

  control::Guards bad = all_ok();
  bad.connected = false;
  fsm.update(bad,  health_ok(), odom_ok(),  151.0);       // 断流
  EXPECT_DOUBLE_EQ(fsm.streaming_seconds(151.0), 0.0);

  fsm.update(all_ok(),  health_ok(), odom_ok(),  152.0);  // 恢复
  EXPECT_FALSE(fsm.update(all_ok(),  health_ok(), odom_ok(),  152.5).request_offboard) << "恢复后没重新计时";
  EXPECT_TRUE(fsm.update(all_ok(),  health_ok(), odom_ok(),  153.0).request_offboard);
}

TEST(Fsm, 观测口在没流时报零)
{
  control::Fsm fsm(1.0, control::TaskLimits{});
  EXPECT_DOUBLE_EQ(fsm.streaming_seconds(100.0), 0.0);

  control::Guards bad = all_ok();
  bad.local_pos_fresh = false;
  fsm.update(bad,  health_ok(), odom_ok(),  100.0);
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

/// ★★ ticket 16：「EKF2 位置失效」的**真载体**（`/mavros/estimator_status.pos_horiz_abs`）。
///
/// 判据**只在已解锁时**成立（与 `rc_timeout` 同款门）：
///   · 未解锁时没有任务可中止，而这条判据的代价是**停发** ⟹ 会在台架上把"冷启动时 EKF2
///     还没把位置锚到动捕上"这段正常窗口变成"不流 setpoint"（用户 2026-09-22 裁定：不做）。
///   · 已解锁时它就是 ④「位置失效 ⟹ LAND」在**真机链条上真能兑现**的那一条
///     （`position_invalid` 那条在本链条上恒假，见 §4.4）。
TEST(FirstDegrade, EKF2位置失效只在已解锁时算失效)
{
  control::Health h = health_ok();
  h.ekf2_pos_lost = true;
  EXPECT_EQ(control::first_degrade(all_ok(), h, control::PrevFrame{}), control::Degrade::kNone)
    << "未解锁时不算失效（同 rc_timeout 的解锁门）";
  EXPECT_EQ(
    control::first_degrade(armed_ok(), h, control::PrevFrame{}), control::Degrade::kEkf2PosLost);
}

/// ★ 同一帧里两条都成立时报**动捕戳冻住** —— 它是原因，`ekf2_pos_lost` 是它的后果：
/// 戳不前进 ⟹ EKF2 拿不到新样本 ⟹ 自己清掉 `ev_pos`（**400 ms**：`EKF/ev_control.cpp:75-81`
/// 的 `isTimedOut(_ev_sample_prev.time_us, 2 * EV_MAX_INTERVAL)`）⟹ `pos_horiz_abs` 变假。
/// 报后者会把排查方向指到估计器上，而问题在上游的戳。
TEST(FirstDegrade, 动捕戳冻住优先于EKF2位置失效)
{
  control::Health h = health_ok();
  h.mocap_stamp_stalled = true;
  h.ekf2_pos_lost = true;
  EXPECT_EQ(
    control::first_degrade(armed_ok(), h, control::PrevFrame{}),
    control::Degrade::kMocapStampStalled);
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
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);

  control::Guards g = all_ok();
  g.mocap_fresh = false;
  const control::Decision d = fsm.update(g,  health_ok(), odom_ok(),  101.0);

  EXPECT_EQ(d.phase, control::Phase::kInit);
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_FALSE(d.request_land) << "未解锁时不该请求 AUTO.LAND";
  EXPECT_EQ(d.degrade, control::Degrade::kMocapTimeout);
}

/// ★★ "可复位，不是单向 latch"的**主证据**：未解锁时的降级不闩锁 ⟹ 判据恢复即回 STANDBY。
TEST(Fsm, 未解锁时的失效在判据恢复后自动复位)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);

  control::Guards bad = all_ok();
  bad.mocap_fresh = false;
  fsm.update(bad,  health_ok(), odom_ok(),  101.0);
  EXPECT_FALSE(fsm.latched()) << "未解锁的失效不该闩锁";

  const control::Decision d = fsm.update(all_ok(),  health_ok(), odom_ok(),  102.0);
  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_TRUE(d.stream_setpoint);
  EXPECT_EQ(d.degrade, control::Degrade::kNone);
}

TEST(Fsm, 已解锁时的失效停发并请求降落)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health h = health_ok();
  h.battery_low = true;
  const control::Decision d = fsm.update(armed_ok(),  h, odom_ok(),  101.0);

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
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health h = health_ok();
  h.battery_low = true;
  fsm.update(armed_ok(),  h, odom_ok(),  101.0);

  const control::Decision d = fsm.update(armed_ok(),  health_ok(), odom_ok(),  110.0);   // 电压"回来了"
  EXPECT_EQ(d.phase, control::Phase::kLand);
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_TRUE(d.request_land);
  EXPECT_EQ(d.degrade, control::Degrade::kBatteryLow) << "闩锁期间保持**首因**（最接近根因）";
}

/// ★ 复位主路径：**落地上锁**（`armed` true→false）⟹ 闩锁解除，一切重来。
TEST(Fsm, 已解锁的失效在落地上锁之后复位)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health h = health_ok();
  h.battery_low = true;
  fsm.update(armed_ok(),  h, odom_ok(),  101.0);

  control::Guards landed = all_ok();               // armed = false（落地上锁）
  EXPECT_FALSE(fsm.update(landed,  h, odom_ok(),  200.0).request_land) << "落地之后不该再请求降落";
  EXPECT_FALSE(fsm.latched());

  // 而且电压仍然低 ⟹ 未解锁下只是"暂时不能发"（回 INIT），不是单向 latch。
  const control::Decision d = fsm.update(landed,  h, odom_ok(),  201.0);
  EXPECT_EQ(d.phase, control::Phase::kInit);
  EXPECT_EQ(d.degrade, control::Degrade::kBatteryLow);

  // 电压恢复 ⟹ 自动回 STANDBY（可复位）。
  const control::Decision back = fsm.update(landed,  health_ok(), odom_ok(),  202.0);
  EXPECT_EQ(back.phase, control::Phase::kStandby);
  EXPECT_TRUE(back.stream_setpoint);
}

/// ★★ ticket 18：mavros 的**合成断连帧**不是"落地上锁"。
///
/// 真 mavros 在链路超时（`conn_timeout` 默认 10 s）后会**自己发一帧**
/// `connected=false, armed=false, guided=false, manual_input=false, mode=""`（`sys_status.cpp:903-911`
/// 的 `publish_disconnection()`，由 `:1411` 的 `connection_cb(false)` 触发；
/// mavros 2.15.1 @ `22ae5b7`，第一手核实）。那一帧里的 `armed=false` **不是读数**，
/// 是占位值 —— 它只说明"我们现在不知道飞控怎么样"。
/// 把它当成落地上锁的降沿，就会把闩锁**在空中**清掉；链路恢复时若 PX4 仍 armed
/// （很可能：我们停发之后它自己退出了 OFFBOARD，但没落地），闩锁要拦的正是那一刻。
TEST(Fsm, 合成断连帧不算落地上锁_闩锁不被它清掉)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  const control::Guards armed = armed_offboard();   // 已解锁 ∧ 已在 OFFBOARD
  fsm.update(armed,  health_ok(), odom_ok(),  100.0);

  // 链路静默（`/mavros/state` 超时）⟹ 已解锁时的失效 ⟹ 闩锁。
  control::Guards silent = armed;
  silent.connected = false;
  fsm.update(silent,  health_ok(), odom_ok(),  103.0);
  ASSERT_TRUE(fsm.latched()) << "正控：这一帧必须已经闩锁，否则下面的「没被清掉」不成立";
  ASSERT_EQ(fsm.latched_reason(), control::Degrade::kStateTimeout);

  // mavros 的合成断连帧到达（字段逐字照 `publish_disconnection()`）。
  const control::Decision d = fsm.update(disconnect_frame(),  health_ok(), odom_ok(),  113.0);

  EXPECT_TRUE(fsm.latched()) << "★ 合成帧的 armed=false 不是落地上锁 ⟹ 闩锁必须还在";
  EXPECT_EQ(fsm.latched_reason(), control::Degrade::kStateTimeout) << "首因不被改写";
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_FALSE(d.request_offboard) << "失联期间不抢回任何模式";
  EXPECT_EQ(d.degrade, control::Degrade::kStateTimeout) << "日志里的原因仍是「飞控失联」";
  // ⚠️ 代价（刻意接受，见 ticket 18 的 Comments）：合成帧之后**对外任务态**是 `INIT` ——
  //    ⑤ 那句「已解锁就请求 AUTO.LAND」看的是 `guards.armed`，而这一帧把它报成了 false。
  //    闩锁与"任务已中止"都在，链路一恢复就回到 `LAND`（下一条用例钉住）。
  //    要让它这期间也显示 `LAND`，得让 ⑤ 把「已闩锁的 kArmedDegrade」也算作已解锁 ——
  //    那是另一条语义改动，本票**未采纳**（选的是 (a)：只改复位判据）。
  EXPECT_EQ(d.phase, control::Phase::kInit);
}

/// ★★ ticket 18 的本体：合成帧之后**链路恢复**，而那期间 PX4 **没有落地**
/// （它多半只是按 `COM_OF_LOSS_T` 自己退出了 OFFBOARD）。
/// ⟹ 闩锁必须仍然在：不许从 INIT 重新来过，而是回到 LAND 并**重发 `AUTO.LAND`**。
///   本条的判别力就在这两句上（改回原状 ⟹ 节点停在 `INIT`）。
///
/// ⚠️ **`request_offboard` 为假这一句没有判别力**（三种实现——全修复 / 只修一半 / 全撤——都成立）：
///   拦住"自主重新接管"的是 `kArmingByUs` 守卫（合成帧那个假的 `armed=false` 顺手把 `own_arming`
///   清掉了，链路回来那次**真的**上升沿于是被记成"别人解锁"），那是**撞上的**安全网、不是本票
///   设计的。留着这条断言是为了钉住**性质**（以后别让它漏出去），别把它当成"本票修好了"的证据。
TEST(Fsm, 合成帧之后链路恢复且PX4仍armed_继续走LAND且不接管)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  const control::Guards armed = armed_offboard();
  fsm.update(armed,  health_ok(), odom_ok(),  100.0);

  control::Guards silent = armed;
  silent.connected = false;
  fsm.update(silent,  health_ok(), odom_ok(),  103.0);

  fsm.update(disconnect_frame(),  health_ok(), odom_ok(),  113.0);
  ASSERT_TRUE(fsm.latched());

  // 链路恢复、PX4 仍 armed：模式是它自己的（`COM_OBL_RC_ACT` / `NAV_RCL_ACT` 决定的），
  // 而这次解锁**不是**我们做的（假降沿已把归属清掉）—— 与真机逐字一致。
  control::Guards back = armed;
  back.in_offboard = false;
  back.own_arming = false;
  const control::Decision d = fsm.update(back,  health_ok(), odom_ok(),  130.0);

  EXPECT_TRUE(fsm.latched()) << "闩锁撑过整段失联";
  EXPECT_EQ(d.phase, control::Phase::kLand) << "任务已中止（不是回 INIT 重新来过）";
  EXPECT_TRUE(d.request_land) << "链路回来了 ⟹ 重发 AUTO.LAND（中止任务这件事不放弃）";
  EXPECT_FALSE(d.request_offboard) << "不许自主重新接管（见文件里的 ⚠️：这条不是本票的判别力所在）";
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_EQ(d.degrade, control::Degrade::kStateTimeout) << "闩锁保持**首因**，日志指向飞控失联";
}

/// ★★ ticket 18 的第二半（票面没写、第一手读代码才发现）：合成帧**不能把 `armed` 的降沿吃掉**。
///
/// 复位是**降沿**（`prev_armed && !guards.armed`），而 `was_armed_` 若在 `connected=false`
/// 的合成帧上也被刷新，那么链路恢复时（`connected=true ∧ armed=false`，**真的**落地上锁）
/// 上一帧的 `armed` 已经是 false ⟹ 降沿早就被吃掉了 ⟹ **闩锁永远不复位**。
/// 那就成了 ticket 06 验收项 ⑤ 明令禁止的单向 latch（只能重启节点）。
TEST(Fsm, 合成帧之后链路恢复且已落地上锁_仍然复位)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  const control::Guards armed = armed_offboard();
  fsm.update(armed,  health_ok(), odom_ok(),  100.0);

  control::Guards silent = armed;
  silent.connected = false;
  fsm.update(silent,  health_ok(), odom_ok(),  103.0);

  fsm.update(disconnect_frame(),  health_ok(), odom_ok(),  113.0);
  ASSERT_TRUE(fsm.latched());

  // 链路恢复：PX4 在失联期间已经落地并上锁 ⟹ 这是**可信的**落地上锁，必须复位。
  const control::Decision d = fsm.update(all_ok(),  health_ok(), odom_ok(),  130.0);
  EXPECT_FALSE(fsm.latched()) << "★ 不许变成单向 latch：只能重启节点才能退出去的降级是 ticket 06 的禁忌";
  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_TRUE(d.stream_setpoint) << "复位之后重新开始流 setpoint";
  EXPECT_EQ(d.degrade, control::Degrade::kNone);
}

/// ★ 已在 LAND 里（低压闩锁）时飞行员打杆 ⟹ **升级成夺权**：交还控制权、**停发
/// `AUTO.LAND` 请求**。飞行员永远优先，不能跟他抢。
TEST(Fsm, 低压闩锁之后打杆会升级成夺权)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health low = health_ok();
  low.battery_low = true;
  EXPECT_EQ(fsm.update(armed_ok(),  low, odom_ok(),  101.0).phase, control::Phase::kLand);

  control::Health both = low;          // 电压仍然低，而且飞行员在打杆
  both.rc_sticks_moving = true;
  const control::Decision d = fsm.update(armed_ok(),  both, odom_ok(),  102.0);

  EXPECT_EQ(d.phase, control::Phase::kRcOverride) << "飞行员接管时不该还停在 LAND";
  EXPECT_FALSE(d.request_land) << "★ 已经在请求降落，飞行员一打杆就必须停止请求（否则每秒一次的模式拉锯）";
  EXPECT_EQ(d.degrade, control::Degrade::kRcOverride) << "日志要给飞行员优先的**当前**原因";
  EXPECT_FALSE(d.stream_setpoint);
}

/// 反向不成立：夺权闩锁**不能**被普通的输入失效"降级"（飞机的控制权不在我们手上，
/// 报一个输入超时会把排查方向带偏）。
TEST(Fsm, 夺权闩锁不会被输入超时改写)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health moving = health_ok();
  moving.rc_sticks_moving = true;
  fsm.update(armed_ok(),  moving, odom_ok(),  101.0);

  control::Health muted = health_ok();
  muted.rc_fresh = false;                       // 连遥控器也断了
  const control::Decision d = fsm.update(armed_ok(),  muted, odom_ok(),  102.0);
  EXPECT_EQ(d.degrade, control::Degrade::kRcOverride);
  EXPECT_EQ(d.phase, control::Phase::kRcOverride);
  EXPECT_TRUE(fsm.latched());
}

/// ★★ 我们自己请求的 `AUTO.LAND` 生效**不是**"被外部夺走"。
/// 少了这条区分，降落会变成一场自导自演的闹剧：请求被接受 ⟹ `in_offboard` 掉 ⟹ 状态机
/// 判"模式被夺走" ⟹ 交还控制权给不存在的飞行员 + **停止重试降落请求**。
TEST(Fsm, 自己请求的降落模式生效不算夺权)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health low = health_ok();
  low.battery_low = true;
  const control::Decision asked = fsm.update(armed_ok(),  low, odom_ok(),  101.0);
  EXPECT_TRUE(asked.request_land);

  control::Guards landed_mode = armed_ok();   // PX4 接受了：mode == AUTO.LAND
  landed_mode.in_land = true;
  const control::Decision d = fsm.update(landed_mode,  low, odom_ok(),  101.2);

  EXPECT_EQ(d.degrade, control::Degrade::kBatteryLow) << "报的还该是最初的原因，不是 mode_lost";
  EXPECT_EQ(d.phase, control::Phase::kLand);
  EXPECT_FALSE(d.request_land) << "已经在降落模式里 ⟹ 不再重发请求（重试是留给『被拒』的）";
  EXPECT_EQ(d.authority, control::Authority::kManual);
}

/// 反过来：已经在我们请求的降落模式里，模式又被别人切走 ⟹ 夺权（停止请求、交还控制权）。
TEST(Fsm, 降落中模式被切走则交还控制权)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health low = health_ok();
  low.battery_low = true;
  fsm.update(armed_ok(),  low, odom_ok(),  101.0);
  control::Guards landed_mode = armed_ok();
  landed_mode.in_land = true;
  fsm.update(landed_mode,  low, odom_ok(),  101.2);

  const control::Decision d = fsm.update(armed_ok(),  low, odom_ok(),  102.0);   // POSCTL（别人切的）
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
  control::Fsm fsm(0.0, control::TaskLimits{});
  control::Guards inside = armed_ok();
  inside.in_offboard = true;
  fsm.update(inside,  health_ok(), odom_ok(),  100.0);           // 已解锁、在 OFFBOARD 里流
  EXPECT_GT(fsm.streaming_seconds(100.5), 0.0)
    << "正控：第 100 帧我们确实在流 —— 下面这个下降沿的定性全靠它（见 `PrevFrame::streaming`）";

  control::Health low = health_ok();
  low.battery_low = true;
  const control::Decision first = fsm.update(inside,  low, odom_ok(),  101.0);
  EXPECT_EQ(first.phase, control::Phase::kLand);
  EXPECT_TRUE(first.request_land);

  // 1 s 之后 PX4 自己退出 OFFBOARD（我们的流早就停了 ⟹ `in_offboard` 变假）
  const control::Guards out = armed_ok();
  const control::Decision d = fsm.update(out,  low, odom_ok(),  102.0);
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
  control::Fsm fsm(0.0, control::TaskLimits{});
  control::Guards inside = all_ok();                // 未解锁但已在 OFFBOARD 里流
  inside.in_offboard = true;
  fsm.update(inside,  health_ok(), odom_ok(),  100.0);
  EXPECT_GT(fsm.streaming_seconds(100.5), 0.0) << "正控：第 100 帧确实在流";

  control::Guards g = inside;
  g.mocap_fresh = false;                            // 动捕失效 ⟹ 停发（未解锁 ⟹ 不请求 LAND）
  const control::Decision d = fsm.update(g,  health_ok(), odom_ok(),  101.0);
  EXPECT_EQ(d.phase, control::Phase::kInit);
  EXPECT_FALSE(d.request_land);
  EXPECT_FALSE(fsm.latched()) << "未解锁的失效不闩锁";

  // PX4 自己退出 OFFBOARD（我们的流停了 1 s 之后）—— 这一帧不许改变上面任何一条。
  control::Guards out = g;                          // 动捕仍然失效
  out.in_offboard = false;                          // ★ 关键：mode 已经不是我们持有的了
  const control::Decision after = fsm.update(out,  health_ok(), odom_ok(),  102.0);
  EXPECT_EQ(after.degrade, control::Degrade::kMocapTimeout) << "还是最初的原因";
  EXPECT_FALSE(fsm.latched()) << "★ 不许因为 PX4 自退就闩锁成夺权（那样就只能靠人把模式放回来）";

  // 判据恢复 ⟹ 自动回 STANDBY 并重新流（"可复位"的正面）。
  const control::Decision back = fsm.update(inside,  health_ok(), odom_ok(),  103.0);
  EXPECT_EQ(back.phase, control::Phase::kStandby);
  EXPECT_TRUE(back.stream_setpoint);
  EXPECT_EQ(back.degrade, control::Degrade::kNone);
}

// ─────────────────────────── 控制权被夺走（打杆 / 模式） ───────────────────────────

/// ★★ ticket 06 验收项 1：打杆 ⟹ **立刻停发** + 不请求 LAND + 不请求 OFFBOARD。
TEST(Fsm, 打杆时进RC_OVERRIDE并停发)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health h = health_ok();
  h.rc_sticks_moving = true;
  const control::Decision d = fsm.update(armed_ok(),  h, odom_ok(),  101.0);

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
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health moving = health_ok();
  moving.rc_sticks_moving = true;
  fsm.update(armed_ok(),  moving, odom_ok(),  101.0);

  // 摇杆回中（不再在动），模式是 POSCTL（假 PX4 被夺权后的样子）
  control::Guards g = armed_ok();
  g.in_offboard = false;
  for (double t = 102.0; t < 130.0; t += 0.5) {
    const control::Decision d = fsm.update(g,  health_ok(), odom_ok(),  t);
    EXPECT_FALSE(d.request_offboard) << "t=" << t << "：夺权之后又去抢 OFFBOARD 了";
    EXPECT_FALSE(d.stream_setpoint);
    EXPECT_EQ(d.phase, control::Phase::kRcOverride);
  }
}

/// ★ 复位路径：**有人把模式放回 OFFBOARD**（闩锁期间我们从不自己请求 ⟹ 只可能是外部动作）。
TEST(Fsm, 外部把模式放回OFFBOARD即复位)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health moving = health_ok();
  moving.rc_sticks_moving = true;
  control::Guards out = armed_ok();
  out.in_offboard = false;
  fsm.update(out,  moving, odom_ok(),  101.0);
  EXPECT_TRUE(fsm.latched());

  control::Guards back = armed_ok();
  back.in_offboard = true;                    // 有人把它放回去了
  const control::Decision d = fsm.update(back,  health_ok(), odom_ok(),  102.0);
  EXPECT_FALSE(fsm.latched());
  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_TRUE(d.stream_setpoint);
  EXPECT_EQ(d.degrade, control::Degrade::kNone);
}

/// 模式被夺走的**权威路径**：曾在 OFFBOARD，现在不是了（`state.mode != "OFFBOARD"`）。
TEST(Fsm, 模式掉出OFFBOARD时进RC_OVERRIDE)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  control::Guards inside = all_ok();
  inside.in_offboard = true;
  fsm.update(inside,  health_ok(), odom_ok(),  100.0);     // 已经切进去了

  control::Guards kicked_out = all_ok();      // mode 变了（POSCTL）
  const control::Decision d = fsm.update(kicked_out,  health_ok(), odom_ok(),  101.0);

  EXPECT_EQ(d.phase, control::Phase::kRcOverride);
  EXPECT_FALSE(d.stream_setpoint);
  EXPECT_FALSE(d.request_offboard) << "模式被夺走之后不许抢回来";
  EXPECT_EQ(d.degrade, control::Degrade::kModeLost);
}

/// ★ 首因优先：打杆（先）→ 模式随后掉出 OFFBOARD（`/mavros/state` 1 Hz，最多晚 1 s）⟹
/// 报"打杆"而不是"模式被夺走"。两个原因指向不同的排查方向，报错了会浪费现场时间。
TEST(Fsm, 打杆之后模式才掉出来仍然报打杆)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  control::Guards inside = armed_ok();
  inside.in_offboard = true;
  fsm.update(inside,  health_ok(), odom_ok(),  100.0);

  control::Health moving = health_ok();
  moving.rc_sticks_moving = true;
  fsm.update(inside,  moving, odom_ok(),  100.05);         // 我们先从摇杆上判出夺权

  control::Guards kicked_out = armed_ok();    // 1 s 后 state 才反映出来
  const control::Decision d = fsm.update(kicked_out,  health_ok(), odom_ok(),  101.0);
  EXPECT_EQ(d.degrade, control::Degrade::kRcOverride);
  EXPECT_EQ(d.phase, control::Phase::kRcOverride);
}

/// 未解锁时模式被夺走（台架上有人拨了模式开关）⟹ 一样"不抢回来"，
/// 直到有人把模式放回去（或节点重启）。
TEST(Fsm, 未解锁时模式被夺走也不抢回来)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  control::Guards inside = all_ok();
  inside.in_offboard = true;
  fsm.update(inside,  health_ok(), odom_ok(),  100.0);

  control::Guards kicked_out = all_ok();
  const control::Decision d = fsm.update(kicked_out,  health_ok(), odom_ok(),  101.0);
  EXPECT_EQ(d.phase, control::Phase::kRcOverride);
  EXPECT_FALSE(d.request_offboard);
  EXPECT_EQ(d.degrade, control::Degrade::kModeLost);
}

/// 我们自己请求 OFFBOARD 成功的那一帧**不是**"模式被夺走"（`in_offboard` 是上升沿）。
TEST(Fsm, 模式进入OFFBOARD不算被夺走)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);

  control::Guards inside = all_ok();
  inside.in_offboard = true;
  const control::Decision d = fsm.update(inside,  health_ok(), odom_ok(),  100.1);
  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_EQ(d.degrade, control::Degrade::kNone);
}

// ─────────────────────────── 动捕时间戳不前进 ───────────────────────────

/// "有帧但时间戳不前进"是**话题层面看不见**的失效：mavros 会丢弃这类 vision_pose
/// （`send_vision_estimate()` 里戳相等即 return），EKF2 也会丢过旧的样本。
/// ⟹ 我们的新鲜度判据（看到达时刻）对它无感，必须单独判。
TEST(Fsm, 动捕时间戳不前进时降级)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(armed_ok(),  health_ok(), odom_ok(),  100.0);

  control::Health stalled = health_ok();
  stalled.mocap_stamp_stalled = true;
  const control::Decision d = fsm.update(armed_ok(),  stalled, odom_ok(),  101.0);

  EXPECT_EQ(d.degrade, control::Degrade::kMocapStampStalled);
  EXPECT_EQ(d.phase, control::Phase::kLand);
  EXPECT_TRUE(d.request_land);
}

// ─────────────────────────── 起飞 / 悬停判据（ticket 08） ───────────────────────────
//
// ★ 为什么这些判据还要在**这一层**再测一遍（`test_control_takeoff.py` 已经从头到尾飞了一轮）：
//   · "持续成立"的语义（一帧不成立就**重新计时**，不是累计）在话题层要靠卡时刻去撞，
//     撞不准就变成一条时不时红一次的用例；在纯函数层是"喂一帧不合格的读数，看计时有没有清零"。
//   · "判据的基准是什么"是这一张最容易写错的两处，而写错之后话题层的表现只是"飞机没进 HOVER"——
//     看不出原因。这里的两条：到高看**原点+高度**（不是当帧目标）、判稳看**悬停点**（不是轨迹原点）。
//   · `request_takeoff` 的生命周期（受理 / 已经在飞 / 超时作废 / 被拒 / 落地上锁复位）全是纯状态。
//
// ★ 场面约定：`now_s` 从 100 s 起算（不是 0）—— 判据里的哨兵值是负数（`settle_started_at_s_ = -1`），
//   从 0 起算的用例撞不出"哨兵被当成了真实时刻"这类错。

namespace
{

/// 一帧带指定位置 / 速度 / 机头的读数。
control::Odometry odom_at(double x, double y, double z, double yaw = 0.0,
  double vx = 0.0, double vy = 0.0, double vz = 0.0)
{
  control::Odometry o = odom_ok();
  o.x = x;
  o.y = y;
  o.z = z;
  o.vx = vx;
  o.vy = vy;
  o.vz = vz;
  o.yaw = yaw;
  return o;
}

/// 秒 → 帧（50 Hz = 0.02 s/帧，与 `setpoint_rate_hz` 对齐）。
int frames_of(double seconds) {return static_cast<int>(seconds / 0.02 + 1e-9);}

/// 第 `i` 帧对应的**时刻偏移**（从 `Climbing::now` 的起点算起）。
double dt_of(int frames) {return 0.02 * frames;}

/// ticket 08 的基线场面：受理起飞 ⟹ 发解锁请求 ⟹ 解锁 ⟹ 停在 `TAKEOFF`。
///
/// ★ 原点与机头取**三轴各不同、且机头非零**的值：轴装反、或把 yaw 当成常量 0 都会显形
///   （ticket 05 的 `yaw` 参数就是这么错的）。
struct Climbing
{
  control::Fsm fsm{0.0, control::TaskLimits{}};
  control::TaskLimits limits{};
  /// 当前时刻。★ 逐帧 `+= 0.02` 累加，**不用 `起点 + 帧数×0.02`** —— 判据里全是
  ///   "now − 锁存的时刻"，累加出来的差值与探针完全一致，不会有边界上的差一帧抖动。
  double now{0.0};
  double x{0.20}, y{-0.30}, z{0.05};   ///< 起飞原点（三轴各不相同）
  double yaw{0.70};                    ///< 起飞机头（非零 ⟹ "锁没锁"看得出来）
};

Climbing reach_climb()
{
  Climbing c;
  c.fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);
  EXPECT_EQ(c.fsm.request_takeoff(100.0), control::TakeoffReply::kAccepted);

  control::Guards offboard = all_ok();
  offboard.in_offboard = true;
  c.fsm.update(offboard,  health_ok(), odom_ok(),  100.02);   // 前置齐 ⟹ 发出解锁请求

  // ★ 受理与解锁之间飞机**已经被挪了一点**：起飞原点锁的必须是"看到 armed 那一帧"的位置，
  //   不是受理那一刻的 —— 前者才是"起飞时的位置"。
  c.now = 100.04;
  c.fsm.update(armed_offboard(),  health_ok(), odom_at(c.x, c.y, c.z, c.yaw),  c.now);
  return c;
}

/// 以 0.02 s 的步长喂 `frames` 帧，每帧的读数都由调用方给（同一帧重复 `frames` 次）。
control::Decision advance(Climbing & c, int frames, const control::Odometry & odom)
{
  control::Decision d;
  for (int i = 0; i < frames; ++i) {
    c.now += 0.02;
    d = c.fsm.update(armed_offboard(),  health_ok(), odom,  c.now);
  }
  return d;
}

/// 在 `c` 的原点上方 `z` 处、速度为零、机头不变，停 `frames` 帧。
control::Decision stand_at(Climbing & c, int frames, double z)
{
  return advance(c, frames, odom_at(c.x, c.y, z, c.yaw));
}

}  // namespace

TEST(Takeoff, 受理之后仍停在STANDBY且前置不齐时不请求解锁)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);

  EXPECT_EQ(fsm.request_takeoff(100.0), control::TakeoffReply::kAccepted);
  EXPECT_TRUE(fsm.awaiting_arming());

  // 前置不齐（不在 OFFBOARD）⟹ 一帧都不该请求解锁。
  //   ★ 少了这条判断，收命令的那一小会儿（还没切进 OFFBOARD）就会发出一条注定被拒的请求 ——
  //     而被拒会走那条**不重试**的失败路径，整个任务当场作废。
  const control::Decision d = fsm.update(all_ok(),  health_ok(), odom_ok(),  100.02);
  EXPECT_FALSE(d.request_arming);
  EXPECT_EQ(d.phase, control::Phase::kStandby) << "受理 ≠ 起飞：飞机还没被我们驱动";
  EXPECT_EQ(d.authority, control::Authority::kManual);
  EXPECT_TRUE(d.stream_setpoint) << "但 setpoint 照流（停机坪上也在流，这是切 OFFBOARD 的前提）";
}

TEST(Takeoff, 前置齐时解锁请求只发一次)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);
  fsm.request_takeoff(100.0);

  control::Guards offboard = all_ok();
  offboard.in_offboard = true;
  int requests = 0;
  for (int i = 0; i < 50; ++i) {
    if (fsm.update(offboard,  health_ok(), odom_ok(),  100.02 + 0.02 * i).request_arming) {
      ++requests;
    }
  }
  EXPECT_EQ(requests, 1) << "解锁请求是一次性脉冲：重发会让飞控收到一串 ARM 命令";
  EXPECT_TRUE(fsm.arming_requested()) << "请求发出去过 ⟹ 诊断日志要能区分「还没到前置」与「在等生效」";
}

TEST(Takeoff, 已经在飞的时候第二次起飞被拒)
{
  Climbing c = reach_climb();
  EXPECT_EQ(c.fsm.request_takeoff(c.now + 1.0), control::TakeoffReply::kAlreadyFlying);
  EXPECT_STREQ(c.fsm.task_name(), "climb");
}

/// ★ §H.7.1「起飞前遥控必须开机且链路已建立」—— 把它做成**机器判据**，不是人肉清单。
/// 不这么做的后果是具体的：遥控静默时解锁，紧接着的 `rc_timeout` 降级（只在已解锁时算失效）
/// 会判失效并请求降落 ⟹ 飞机**起飞后立刻降落**，比一开始就不解锁难查得多。
TEST(Takeoff, 遥控不在线时不请求解锁且命令超时作废)
{
  const control::TaskLimits limits;
  control::Fsm fsm(0.0, limits);
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);
  fsm.request_takeoff(100.0);

  control::Guards offboard = all_ok();
  offboard.in_offboard = true;
  control::Health no_rc = health_ok();
  no_rc.rc_fresh = false;

  const control::Decision d = fsm.update(offboard,  no_rc, odom_ok(),  100.02);
  EXPECT_FALSE(d.request_arming) << "遥控不在线时不许解锁（§H.7.1）";
  EXPECT_EQ(d.phase, control::Phase::kStandby);

  // 超时 ⟹ 命令**作废**（不自动重试），退回可以重新受理的状态。
  const control::Decision late = fsm.update(
    offboard,  no_rc, odom_ok(),  100.02 + limits.takeoff_timeout_s + 0.5);
  EXPECT_FALSE(late.request_arming) << "作废的时候也不许补发解锁请求（不自动重试）";
  EXPECT_FALSE(fsm.awaiting_arming()) << "超时之后命令应当没了";
  EXPECT_STREQ(fsm.task_name(), "idle");
  EXPECT_EQ(fsm.request_takeoff(200.0), control::TakeoffReply::kAccepted) << (
    "作废不需要重启节点：操作员按第二下就该被受理");
}

TEST(Takeoff, 解锁被拒时当场退回STANDBY且不自动重试)
{
  control::Fsm fsm(0.0, control::TaskLimits{});
  fsm.update(all_ok(),  health_ok(), odom_ok(),  100.0);
  fsm.request_takeoff(100.0);

  control::Guards offboard = all_ok();
  offboard.in_offboard = true;
  ASSERT_TRUE(fsm.update(offboard,  health_ok(), odom_ok(),  100.02).request_arming);
  ASSERT_TRUE(fsm.awaiting_arming());

  // 边界层拿到"被拒"的应答 ⟹ 通知状态机 ⟹ 任务层**当场**归零（不必等 takeoff_timeout_s）。
  //   ★ 静默重试会把"参数没配对"这种确定性拒绝变成一串没人看的日志（用户 2026-09-22 裁定）。
  fsm.note_arming_failed();
  EXPECT_STREQ(fsm.task_name(), "idle");
  EXPECT_FALSE(fsm.awaiting_arming());
  EXPECT_FALSE(fsm.arming_requested()) << "这一轮的「发过请求」也随之作废，否则下一轮不再请求解锁";

  int retries = 0;
  for (int i = 0; i < 250; ++i) {   // 250 帧 = 5 s ≫ takeoff_timeout_s
    if (fsm.update(offboard,  health_ok(), odom_ok(),  100.04 + 0.02 * i).request_arming) {
      ++retries;
    }
  }
  EXPECT_EQ(retries, 0) << "被拒之后**一帧都不许**补发（不自动重试）";
  EXPECT_EQ(fsm.request_takeoff(200.0), control::TakeoffReply::kAccepted);
}

TEST(Takeoff, 解锁那一帧锁定原点与机头并进入TAKEOFF)
{
  Climbing c = reach_climb();

  EXPECT_STREQ(c.fsm.task_name(), "climb");
  double ox = 0.0, oy = 0.0, oz = 0.0;
  ASSERT_TRUE(c.fsm.takeoff_origin(&ox, &oy, &oz));
  EXPECT_DOUBLE_EQ(ox, c.x) << "原点必须是**解锁那一帧**的位置，不是受理那一帧的";
  EXPECT_DOUBLE_EQ(oy, c.y);
  EXPECT_DOUBLE_EQ(oz, c.z);
  EXPECT_DOUBLE_EQ(c.fsm.takeoff_yaw(), c.yaw) << "起飞不转向：机头锁一次，全程用它";

  // 第一帧的斜坡刚走了 0.02 s（不是一上来就跳到 +1 m）。
  const double dt = 0.02;
  const control::Decision d = stand_at(c, 1, c.z);
  EXPECT_EQ(d.phase, control::Phase::kTakeoff);
  EXPECT_EQ(d.authority, control::Authority::kAuto);
  EXPECT_NEAR(d.target_z, c.z + c.limits.climb_speed_mps * dt, 1e-9);
  EXPECT_DOUBLE_EQ(d.target_x, c.x) << "水平两轴钉在原点（垂直起飞，不斜着走）";
  EXPECT_DOUBLE_EQ(d.target_y, c.y);
}

TEST(Takeoff, 爬升目标按速率线性上升且封顶在目标高度)
{
  Climbing c = reach_climb();
  const double top = c.limits.takeoff_altitude_m;
  const double rate = c.limits.climb_speed_mps;

  // 斜坡期间**逐帧**核对（不是只看终点）：目标 z = 原点 + rate × 已爬时间。
  // ★ 飞机一直待在地面（z 给 0），所以到高判据不会成立、不会半路跳进 HOVER。
  const int ramp = frames_of(top / rate);
  for (int i = 1; i < ramp; ++i) {
    const control::Decision d = stand_at(c, 1, 0.0);
    EXPECT_NEAR(d.target_z, c.z + rate * (dt_of(i)), 1e-9) << "第 " << i << " 帧不在斜坡上";
    EXPECT_EQ(d.phase, control::Phase::kTakeoff);
  }

  // 爬满之后封顶，不再上升（否则"爬到 1 m"这句话不成立）。
  const control::Decision d = stand_at(c, frames_of(3.0), 0.0);
  EXPECT_NEAR(d.target_z, c.z + top, 1e-9) << "斜坡必须在目标高度上封顶";
}

/// ★★ 本条钉的是 ticket 08 最容易写错的一处：**到高判据的基准是「原点 + 高度」，
/// 不是当帧的 `target_z`**。
///
/// 场面取**最容易骗过它的那一个**：让飞机严格跟着斜坡走（目标走到哪它就到哪）。
/// 此时"飞机当前位置"与"当帧目标 z"只差一个**原点高度**（本例 0.05 m < tol 0.10）
/// ⟹ 判据写成 `target_z` 的话**第一帧就满足**，斜坡形同虚设，
/// 飞机会在爬升的半路上（本例第 51 帧 = 走到 1.0 s 的斜坡中点）被判成"到高"。
TEST(Takeoff, 到高判据看的是原点加高度而不是当前目标值)
{
  Climbing c = reach_climb();
  const double top = c.limits.takeoff_altitude_m;
  const double rate = c.limits.climb_speed_mps;
  const int ramp = frames_of(top / rate);

  control::Decision mid, last;
  for (int i = 1; i <= ramp; ++i) {
    last = advance(c, 1, odom_at(c.x, c.y, rate * dt_of(i), c.yaw));
    if (i == ramp / 2) {mid = last;}   // 斜坡中点 ≈ 0.5 m，正好是目标高度的一半
  }
  EXPECT_EQ(mid.phase, control::Phase::kTakeoff) << (
    "斜坡才走到一半（0.5 m）就被判成到高 ⟹ 判据的基准被写成了当帧目标值");
  EXPECT_EQ(last.phase, control::Phase::kTakeoff) << (
    "判据的基准错了的话，整段斜坡里判据恒成立");
}

TEST(Takeoff, 到高需要连续成立_一帧不合格就重新计时)
{
  Climbing c = reach_climb();
  const double top = c.limits.takeoff_altitude_m;
  const int nearly = frames_of(0.9 * c.limits.takeoff_settle_s);   // 45 帧 = 0.90 s

  ASSERT_EQ(nearly, 45);
  EXPECT_EQ(stand_at(c, nearly, top).phase, control::Phase::kTakeoff);

  // 差一点点就到 settle 时长时给一帧不合格的读数。
  EXPECT_EQ(advance(c, 1, odom_at(c.x, c.y, top - 0.2, c.yaw)).phase,
    control::Phase::kTakeoff);

  // 再凑满 0.90 s（累计 1.80 s > takeoff_settle_s，但**连续**只有 0.90 s）⟹ 仍不该到高。
  stand_at(c, nearly, top);
  EXPECT_STREQ(c.fsm.task_name(), "climb") << "「持续」的语义是连续的，不是累计的";

  // 再多喂几帧 ⟹ 这才到高（顺带证明上面那条不是"永远到不了高"的假阳性）。
  //   ⚠️ 计时是从**判据开始成立的那一帧**起算的 ⟹ 那一帧本身不贡献时长，
  //   所以"连续 0.90 s + 0.10 s"并不等于 1.00 s（差一帧），这里给 0.30 s 的余量。
  EXPECT_STREQ(c.fsm.task_name(), "climb");
  const control::Decision d = stand_at(c, frames_of(0.30), top);
  EXPECT_EQ(d.phase, control::Phase::kHover);
  EXPECT_STREQ(c.fsm.task_name(), "hover");
}

/// ★★ `velocity_valid` 是**必需**的：速度话题一帧都没到过时 `vz` 恰好是 0（"看起来静止"），
/// 拿它判稳会**立刻**通过 —— 这是"没数据当成好消息"的老毛病，必须显式挡掉。
TEST(Takeoff, 速度读数无效时到高判据不成立)
{
  Climbing c = reach_climb();
  control::Odometry blind = odom_at(c.x, c.y, c.limits.takeoff_altitude_m, c.yaw);
  blind.velocity_valid = false;

  const control::Decision d = advance(c, frames_of(5.0), blind);
  EXPECT_EQ(d.phase, control::Phase::kTakeoff) << (
    "没有速度读数时 `vz` 恰好是 0，看起来「静止」—— 不能当成到高");
}

TEST(Takeoff, 判稳需要位置与速率两条同时成立)
{
  const double top = control::TaskLimits{}.takeoff_altitude_m;
  double ox = 0.0, oy = 0.0, oz = 0.0;

  // ① 位置对、速率大 ⟹ 不判稳（这一半错了会"飘着也算稳"）。
  //   ★ 速率必须给在**水平**分量上：垂速给大了连"到高"都过不去
  //     （`takeoff_vel_tol_mps` 与 `hover_vel_tol_mps` 恰好都是 0.10），那测的就不是判稳这一级了。
  Climbing c = reach_climb();
  const control::Decision d = advance(c, frames_of(10.0),
    odom_at(c.x, c.y, top, c.yaw, control::TaskLimits{}.hover_vel_tol_mps * 3.0, 0.0, 0.0));
  EXPECT_EQ(d.phase, control::Phase::kHover) << "位置是对的，所以这一条只该卡在判稳上";
  EXPECT_STREQ(c.fsm.task_name(), "hover") << "速率超容差时不该判稳（判据的另一半）";
  EXPECT_FALSE(c.fsm.traj_origin(&ox, &oy, &oz)) << "没判稳过 ⟹ 轨迹原点也不该存在";

  // ② 速率对、位置偏 ⟹ 也不判稳。
  Climbing p = reach_climb();
  const control::Decision dp = advance(p, frames_of(10.0),
    odom_at(p.x + control::TaskLimits{}.hover_pos_tol_m * 3.0, p.y, top, p.yaw));
  EXPECT_EQ(dp.phase, control::Phase::kHover);
  EXPECT_STREQ(p.fsm.task_name(), "hover") << "离悬停点太远时不该判稳（位置那一半）";
  EXPECT_FALSE(p.fsm.traj_origin(&ox, &oy, &oz));
}

/// ★★ ticket 08 的 ★ 核心设计：**轨迹原点 = 判稳那一刻 EKF 报的位置**，不是悬停点。
/// 工单的原话是"这一刻记下来"，理由是 EKF2 局部原点与动捕原点之间的那个常值偏移
/// 就此被自动吸收（此后一切目标都以它为 0）。
TEST(Takeoff, 判稳成立时轨迹原点记的是判稳那一刻的位置而不是悬停点)
{
  Climbing c = reach_climb();
  const double top = c.limits.takeoff_altitude_m;

  // 让飞机**偏着**停稳：横向偏 0.06 m、高度偏 0.06 m。
  // ★ 两个偏移合起来 0.085 m，必须仍在 `hover_pos_tol_m`(0.10) 之内 —— 否则测的是"没判稳"。
  const double dx = 0.06, dz = 0.06;
  ASSERT_LT(std::sqrt(dx * dx + dz * dz), c.limits.hover_pos_tol_m);

  const control::Decision d = advance(c, frames_of(10.0),
    odom_at(c.x + dx, c.y, c.z + top + dz, c.yaw));
  ASSERT_EQ(d.phase, control::Phase::kWaitStart);

  double tx = 0.0, ty = 0.0, tz = 0.0;
  ASSERT_TRUE(c.fsm.traj_origin(&tx, &ty, &tz));
  EXPECT_DOUBLE_EQ(tx, c.x + dx) << "轨迹原点必须是**判稳那一刻的位置**，不是悬停点";
  EXPECT_DOUBLE_EQ(ty, c.y);
  EXPECT_DOUBLE_EQ(tz, c.z + top + dz) << (
    "记成悬停点的话，WAIT_START 会把飞机往一个它此刻并不在的点上拽 "
    "—— 那个「原点」就成了飞机没去过的地方");
  // WAIT_START 的目标就是它（停在原地），机头仍是起飞机头。
  EXPECT_DOUBLE_EQ(d.target_x, tx);
  EXPECT_DOUBLE_EQ(d.target_y, ty);
  EXPECT_DOUBLE_EQ(d.target_z, tz);
  EXPECT_DOUBLE_EQ(d.target_yaw, c.yaw);
}

TEST(Takeoff, 保持满时长后任务走完并请求降落)
{
  Climbing c = reach_climb();
  const double top = c.limits.takeoff_altitude_m;

  // 判稳 ⟹ WAIT_START（判稳前后各判据的时长加起来 1.0 + 2.0 s，给 5 s 足够）。
  control::Decision d = stand_at(c, frames_of(5.0), top);
  ASSERT_STREQ(c.fsm.task_name(), "wait_start");
  EXPECT_TRUE(d.stream_setpoint) << "保持期间照流（目标 = 轨迹原点）";
  EXPECT_FALSE(d.request_land) << "还没到保持时长，不该请求降落";

  // 保持满 ⟹ 任务走完。★ 动作与**降级**完全一样（停发 + 降落出口），差别只在措辞
  //   ⟹ 用 `mission_complete` 传出去，由边界层决定日志怎么说。
  d = stand_at(c, frames_of(c.limits.hover_hold_s + 1.0), top);
  EXPECT_EQ(d.phase, control::Phase::kLand);
  EXPECT_TRUE(d.mission_complete) << "任务走完 ≠ 降级：动作一样，措辞不同";
  EXPECT_EQ(d.degrade, control::Degrade::kNone) << "正常走完不该有降级原因";
  EXPECT_TRUE(d.request_land) << "降落必须是我们请求的（上锁才是 PX4 的事）";
  EXPECT_FALSE(d.stream_setpoint) << "请求降落之后必须停发 setpoint";
  // ★ 控制权这时候是 MANUAL：`AUTO` 的定义是"**我们正在发会驱动飞机的指令**"，
  //   而请求 `AUTO.LAND` 之后我们停发、由 PX4 自己落 ⟹ 与 ticket 06 的降级出口逐字一致
  //   （`halt()` 是两处共用的那一段）。`/control/state` 于是打出 `MANUAL/LAND`。
  EXPECT_EQ(d.authority, control::Authority::kManual);
}

TEST(Takeoff, 落地上锁之后原点被清掉_不会锚在上一轮的原点上)
{
  Climbing c = reach_climb();
  const double top = c.limits.takeoff_altitude_m;
  stand_at(c, frames_of(5.0), top);
  ASSERT_STREQ(c.fsm.task_name(), "wait_start");

  // 上锁（armed 的降沿）⟹ 任务层归零。
  //   ★ 不清的话，下一轮会在**上一轮的原点**上悬停 —— 而那是飞机上一次起飞的地方。
  control::Guards down = armed_offboard();
  down.armed = false;
  c.now += 0.02;
  const control::Decision d =
    c.fsm.update(down,  health_ok(), odom_at(c.x, c.y, 0.0, c.yaw),  c.now);

  double x = 0.0, y = 0.0, z = 0.0;
  EXPECT_FALSE(c.fsm.takeoff_origin(&x, &y, &z)) << "落地上锁之后起飞原点必须失效";
  EXPECT_FALSE(c.fsm.traj_origin(&x, &y, &z)) << "轨迹原点同理";
  EXPECT_STREQ(c.fsm.task_name(), "idle");
  EXPECT_EQ(d.phase, control::Phase::kStandby);
  EXPECT_EQ(d.authority, control::Authority::kManual) << "上锁 ⟹ 控制权交还";
  EXPECT_FALSE(d.request_land) << "已经上锁了，不必再请求降落";

  // 而且下一轮能重新起飞（复位是可用的，不是单向闩锁）。
  EXPECT_EQ(c.fsm.request_takeoff(c.now + 1.0), control::TakeoffReply::kAccepted);
  EXPECT_TRUE(c.fsm.awaiting_arming());
}

TEST(Takeoff, 起飞后机头锁定不跟随当前机头)
{
  Climbing c = reach_climb();
  const double top = c.limits.takeoff_altitude_m;

  // 爬升途中假 EKF 把机头转到别处 ⟹ 目标 yaw 必须仍是**起飞机头**（§3.4 起飞不转向）。
  const control::Decision d = advance(c, frames_of(5.0),
    odom_at(c.x, c.y, top, c.yaw + 1.0));
  EXPECT_EQ(d.phase, control::Phase::kWaitStart);
  EXPECT_DOUBLE_EQ(d.target_yaw, c.yaw) << "起飞之后不转向";

  // ★ 而 STANDBY 段是**跟随**当前机头的（不然停在停机坪上就会自己扭）。
  control::Fsm fsm(0.0, control::TaskLimits{});
  const control::Decision standby =
    fsm.update(all_ok(),  health_ok(), odom_at(0.0, 0.0, 0.0, -1.1),  100.0);
  EXPECT_EQ(standby.phase, control::Phase::kStandby);
  EXPECT_DOUBLE_EQ(standby.target_yaw, -1.1);
}
