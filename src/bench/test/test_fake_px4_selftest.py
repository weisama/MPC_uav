"""夹具自检 —— **假 PX4 假得像不像**（与 `test_bench_selftest.py` 同一个理由，见那个文件头）。

★ 这个文件里的每条断言都在回答同一个问题：**如果夹具本身是错的，被测节点的失败会长什么样？**
  答案几乎都是"静默"——QoS 不兼容不报错、时间戳单位错不报错、2 Hz 被写成 10 Hz 也不报错。
  所以这里把它们钉成会红的一行。

★ 全部**不起 DUT**（`start_dut=False`）：要验的是夹具自己的行为。需要"有个谁在发/在收"的地方，
  由**本文件自己**发（`_publish_rates` 等）—— 这样夹具的量具（`publisher_qos` /
  `stamp_unit_guess` / `matched_*`）也被一起验了，而不是靠 DUT 恰好正确。
"""

from __future__ import annotations

import math
import time

import pytest
from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint, VehicleStatus
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from bench import Scenario
from bench.fake_px4 import (ARMING_STATE_ARMED, NAV_STATE_OFFBOARD, NAV_STATE_POSCTL,
                            px4_in_qos, px4_out_qos)


def _fixture_only(**kw) -> Scenario:
    """只起夹具（含假 PX4）、不起 DUT；`mocap=False` 因为这里不测动捕那条链。"""
    return Scenario(dut=None, start_dut=False, px4=True, mocap=False, **kw)


# ───────────────────────────── QoS：与 PX4 的端点逐字对齐 ─────────────────────────────

def test_QoS_常量与_PX4_的_writer_reader_逐字一致():
    """夹具的两个 QoS 是**照抄** PX4 的（`utilities.hpp:59-63` 的 writer、`:114-119` 的 reader）。

    ★ 抄错的后果是**夹具独有的失败**：DUT 在夹具上接不上，在真 PX4 上却是好的
      （或者反过来）⟹ 一切结论都作废。所以这里逐字段对，不看"能不能跑"。
    """
    out = px4_out_qos()
    assert out.reliability == ReliabilityPolicy.BEST_EFFORT, "PX4 的 writer 是 best_effort"
    assert out.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert out.history == HistoryPolicy.KEEP_LAST and out.depth == 1

    into = px4_in_qos()
    assert into.reliability == ReliabilityPolicy.BEST_EFFORT, "PX4 的 reader 是 best_effort"
    # ★ 一定是 VOLATILE：夹具若"顺手"写成 TRANSIENT_LOCAL，DUT 少写一个字段也可能看起来正常
    assert into.durability == DurabilityPolicy.VOLATILE
    assert into.depth == 1


def test_QoS_不兼容的订阅者永远匹配不上_也不报错():
    """★ 本文件最重要的一条：ROS 2 的 QoS 不兼容**不抛异常、不打日志**，只是永不匹配。

    这条性质是整套台架测试的第一性事实（"没收到"必须先问"接上了吗"）。这里用两种订阅者
    把两个方向都钉住：**兼容的**必须数得到，**不兼容的**必须数不到 —— 且都不报错。
    """
    with _fixture_only() as sc:
        assert sc.px4.matched_subscribers("status") == 0, "刚起来不该有订阅者"

        # 正控先做：兼容的订阅者**必须**被数到（否则下面那条可能只是因为"夹具没在发"）
        good = sc.node.create_subscription(
            VehicleStatus, sc.px4.topics["status"], lambda _m: None,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        assert sc.px4.wait_for_subscribers("status", timeout_s=5.0), \
            "兼容 QoS 的订阅者都没匹配上 ⟹ 夹具的发布者本身有问题"

        # 不兼容：夹具的 status 是 BEST_EFFORT，而这个是 RELIABLE
        bad = sc.node.create_subscription(
            VehicleStatus, sc.px4.topics["status"], lambda _m: None,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        sc.hold(0.5)
        assert sc.px4.matched_subscribers("status") == 1, \
            "只该数到那一个兼容的订阅者；RELIABLE 的那个接不上（而且不会有任何报错）"

        # 收不到 ≠ 没接上：不兼容的订阅者确实一帧都收不到 —— 这条与上面那条合起来才是完整判据
        got: list = []
        sc.node.destroy_subscription(bad)
        sc.node.create_subscription(
            VehicleStatus, sc.px4.topics["status"], got.append,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        sc.hold(1.2)
        assert got == [], "RELIABLE 的订阅者收到了帧 ⟹ 夹具的 status 不是 BEST_EFFORT"
        del good


# ───────────────────────────── 状态帧：2 Hz + 变化即刻发 ─────────────────────────────

def test_vehicle_status_是_2_Hz_定频():
    """`Commander.cpp:1790` 逐字："at 2 Hz or immediately when changed"。

    ★ 频率只在**窗口**上算（`status_rate_hz` 的两端都是记录的 `at_s`），
      不断言相邻两帧的间隔：定时器是从回调开始时刻重新计时的，单帧的间隔毫无意义。
    """
    with _fixture_only() as sc:
        sc.advance_to(4.2)                       # 不要卡边界：从 0.2 s 起算 4 个周期
        frames = sc.px4.status_frames
        assert len(frames) >= 6, f"4.2 s 里只发了 {len(frames)} 帧状态"
        rate = sc.px4.status_rate_hz(from_s=frames[1].at_s, to_s=frames[-1].at_s)
        assert 1.5 <= rate <= 2.5, f"状态频率 {rate:.2f} Hz 不是 2 Hz（真机是 2 Hz 定频）"


def test_注入的变化不超过一拍就发出去():
    """★ "变化即刻发"这一半**必须**有，否则注入 arm 要等最多 500 ms 才反映到话题上 ——
    那是夹具的假象（真机是 commander 主循环的 10 ms 量级），而 DUT 里任何"等状态变化"
    的判据都会被这个假象拖慢，甚至被判成"状态超时"。
    """
    with _fixture_only() as sc:
        sc.hold(1.2)                              # 先跑一段，确保 2 Hz 那条路已经工作
        n_before = len(sc.px4.status_frames)
        assert n_before > 0, "正控：静默注入之前，状态帧本来就在流动"
        assert sc.px4.status_frames[-1].msg.arming_state != ARMING_STATE_ARMED

        t_inject = sc.t()
        sc.px4.set_armed(True)
        sc.px4.set_nav_state(NAV_STATE_OFFBOARD)
        sc.advance_to(t_inject + 0.5)
        landed = [f for f in sc.px4.status_frames[n_before:]
                  if int(f.msg.arming_state) == ARMING_STATE_ARMED
                  and int(f.msg.nav_state) == NAV_STATE_OFFBOARD]
        assert landed, "注入之后 0.5 s 内没有任何一帧带上新状态"
        latency = landed[0].at_s - t_inject
        # 0.1 s 是"即刻发"的量级（10 ms 主循环 + DDS 抖动），与 0.5 s 的定频周期分得开
        assert latency < 0.1, f"注入到落地花了 {latency * 1e3:.0f} ms ⟹ 只在按 2 Hz 发，没做即刻发"


def test_静默真的掐断了那条话题_恢复后重新流动():
    """`mute` 是故障注入的基本手段（"这一段时间夹具装作没听见"）。它必须**真的**有效。

    ★ 负断言配正控：掐之前有帧、掐的窗口里一帧都没有、恢复之后又有了。
      只断言"窗口里没有"，无法区分"掐成功了"与"夹具压根没在发"。
      窗口取 1.2 s（= 2.4 个周期）—— 太短的话"恰好没到点"也能凑出 0 帧。
    """
    with _fixture_only() as sc:
        sc.hold(1.0)
        assert sc.px4.status_frames, "正控：掐之前状态帧在流动"
        t0 = sc.t()
        sc.px4.mute("status")
        sc.advance_to(t0 + 1.2)
        in_window = [f for f in sc.px4.status_frames if t0 < f.at_s <= t0 + 1.2]
        assert in_window == [], f"静默窗口里还收到了 {len(in_window)} 帧"

        sc.px4.unmute("status")
        n = len(sc.px4.status_frames)
        sc.hold(0.8)
        assert len(sc.px4.status_frames) > n, "恢复之后状态帧没有回来"


# ───────────────────────────── 四条注入面 ─────────────────────────────

def test_四条注入面都能落到话题上():
    """`set_armed` / `set_nav_state` / `set_pre_flight_checks_pass` / `mute` —— 逐个验证
    "改的属性真的进了发出的帧"（改属性这件事本身太容易自欺：回调可能读的是别的地方）。"""
    with _fixture_only() as sc:
        sc.hold(0.6)
        sc.px4.set_pre_flight_checks_pass(False)
        sc.px4.set_nav_state(NAV_STATE_OFFBOARD)
        sc.px4.set_armed(True)
        sc.hold(0.6)
        last = sc.px4.status_frames[-1].msg
        assert int(last.nav_state) == NAV_STATE_OFFBOARD
        assert int(last.arming_state) == ARMING_STATE_ARMED
        assert last.pre_flight_checks_pass is False
        assert int(last.nav_state_user_intention) == NAV_STATE_OFFBOARD, \
            "DUT 若读 nav_state_user_intention（而不是 nav_state），夹具也得给出同一个值"


def test_假的_PX4_时钟与墙钟是两个纪元():
    """★ 夹具照抄真机的一条**反直觉性质**：`VehicleStatus.timestamp` 是 PX4 上电起算的 µs，
    而 DUT 发出去的时间戳是**墙钟** µs ⟹ 两者相减没有意义（差的是"飞控开了多久"）。

    夹具照抄它，是为了让"拿两个纪元做减法"的写法**在这里就该露馅**（真机上它只会静默地
    算出几百万秒的假年龄）。
    """
    with _fixture_only() as sc:
        sc.hold(0.6)
        frame = sc.px4.status_frames[-1]
        assert frame.timestamp_us < 10 * 1e6, \
            f"夹具的 hrt 应该只走了不到 10 s，实际 {frame.timestamp_us / 1e6:.1f} s"
        assert abs(time.time() * 1e6 - frame.timestamp_us) > 1e9, \
            "夹具的时钟与墙钟差了不止 1000 s ⟹ 这就是要照抄的那条性质"


# ───────────────────────────── 量具：时间戳取证 ─────────────────────────────

def _publish_rates(sc: Scenario, timestamp: int) -> None:
    """以"DUT"的身份在 `/fmu/in/vehicle_rates_setpoint` 上发几帧（QoS 照票面要求）。

    ★ 顺序是 **建发布者 → 等发现完成 → 才发**：DDS 的发现是异步的，发现完成之前发的帧
      会被直接丢掉（不报错）。本文件第一版把 `wait_for_publishers` 写在了建发布者**之前**，
      于是它等的是一个还不存在的发布者 —— 5 s 超时，而现象看起来像"夹具收不到"。
    """
    pub = sc.node.create_publisher(
        VehicleRatesSetpoint, sc.px4.topics["rates"],
        QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                   reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    assert sc.px4.wait_for_publishers("rates", timeout_s=5.0), "夹具没发现本测试的发布者"
    msg = VehicleRatesSetpoint()
    msg.timestamp = int(timestamp)
    msg.roll, msg.pitch, msg.yaw = 0.0, 0.0, 0.0
    msg.thrust_body[2] = -0.62
    for _ in range(6):                      # 发几帧，让中位数稳一点
        pub.publish(msg)
        time.sleep(0.03)


@pytest.mark.parametrize("unit,divisor,expected", [
    # divisor = 把 `time.time_ns()` 除成**那个单位**的数：ns→µs 除以 1e3、ns→ms 除以 1e6、
    # 而"发成 ns"就是原样发出（除 1）—— 三者相差三个数量级，猜错的余地见 `stamp_unit_guess`。
    ("us", 1_000, "us"),
    ("ms", 1_000_000, "ms"),
    ("ns", 1, "ns"),
])
def test_时间戳取证能把三种单位分开(unit, divisor, expected):
    """★ 单位错是**静默**的：`min(timestamp - time_offset, hrt)` 会把它钳回"新鲜"，
    所以话题上看不出任何异常。夹具不能替 DUT 发时间戳，但必须能**说清它发了什么**。

    ⚠️ 判据是"`msg.timestamp` 相对**墙钟**的比值"，所以夹具这一侧的参照也必须是**墙钟**。
       （本文件第一版就是在这里红的：拿 `Scenario.t()` 当参照 —— 那是"相对夹具就位的秒数"，
       与墙钟差着 1.7e9 的量级，于是**正确的 µs 会被报成 `ns`**。）
    """
    with _fixture_only() as sc:
        _publish_rates(sc, time.time_ns() // divisor)
        sc.hold(0.3)
        got, age = sc.px4.stamp_unit_guess("rates")
        assert got == expected, f"单位被判成 {got!r}（中位年龄 {age:.3g} s），期望 {expected!r}"
        if expected == "us":
            assert abs(age) < 0.5, f"µs 的年龄应当接近 0，实际 {age:.3g} s"


def test_时间戳为_0_会被判成_zero():
    """`timestamp == 0` 在 PX4 的反序列化里是**一条分支**（`topic.timestamp = hrt_absolute_time()`）
    —— DUT 若忘了填，PX4 不会报错，只会悄悄用上自己的时钟。夹具要能指出这一点。"""
    with _fixture_only() as sc:
        _publish_rates(sc, 0)
        sc.hold(0.3)
        assert sc.px4.stamp_unit_guess("rates")[0] == "zero"


def test_时间戳往过去挪_1_秒会被判成_stale():
    """夹具**不模拟**时钟同步的滞后 Δ（见 `stamp_age_s` 的说明），但它得认得出
    "这个 µs 是对的、只是挪到过去了"—— 05 实测 1 s 贴边、2 s 就不新鲜了。"""
    with _fixture_only() as sc:
        _publish_rates(sc, time.time_ns() // 1000 - int(1.0e6))
        sc.hold(0.3)
        unit, age = sc.px4.stamp_unit_guess("rates")
        assert unit == "stale", f"被判成 {unit!r}（年龄 {age:.3g} s）"
        assert 0.5 < age < 5.0, f"挪了 1 s 的年龄应当是 1 s 上下，实际 {age:.3g} s"


def test_没收到帧时的量具给出_none_而不是假数():
    """`none` 与 `zero` 是两件事：前者是"夹具没收到任何东西"（可能根本没接上），
    后者是"DUT 发了，但时间戳是 0"。混在一起会让排查从第一步就走错方向。"""
    with _fixture_only() as sc:
        unit, age = sc.px4.stamp_unit_guess("rates")
        assert unit == "none", f"一帧都没收到时应当是 'none'，实际 {unit!r}"
        assert math.isnan(age), f"（年龄应当是 nan，实际 {age!r}）"


def test_心跳话题的发布者_QoS_是票面要求的那一组():
    """★ 这一条演示**怎么验"票面要求的 QoS"**：夹具自己的订阅是 VOLATILE（照抄 PX4），
    与 DUT 发 TRANSIENT_LOCAL 是**兼容**的 ⟹ 在"收得到吗"上永远看不出来。
    唯一的量法是读发布方的 info（`publisher_qos`）。

    ⚠️ **只断言两项**（`reliability` / `durability`）—— 第三项 `depth` 为什么断不了，
      见下一条用例（一条专门钉住"量不回来"这个事实的用例）。
    """
    with _fixture_only() as sc:
        pub = sc.node.create_publisher(
            OffboardControlMode, sc.px4.topics["mode"],
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        pub.publish(OffboardControlMode())
        assert sc.px4.wait_for_publishers("mode", timeout_s=5.0)
        got = sc.px4.publisher_qos("mode")
        assert got is not None, "拿不到发布方 QoS（DDS 发现还没完成？先过 wait_for_publishers）"
        assert got.reliability == ReliabilityPolicy.BEST_EFFORT
        assert got.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_发现信息里读不回来的那两项_history_与_depth():
    """★ 钉住一个**会造成恒红测试**的栈事实（本机实测 2026-09-28，Humble + rmw_fastrtps）：

      · 发现信息里的 `history` 回来是 `UNKNOWN(3)`，`depth` **恒为 0**
      · 且与发布者实际写的值**无关** —— `depth=1` 与 `depth=10` 读回来都是 0

    ⟹ 票面「pub ... `depth 1`」这一项**在这套栈上黑盒不可观测**。夹具不许拿它下断言
      （那会是"夹具在说谎"的另一种形式：永远失败，却与 DUT 无关）；DUT 侧只能靠
      "读自己那份 QoS 常量"级别的测试 + 代码审查兜住。

    ★ 为什么这条算**夹具自检**：夹具的 `publisher_qos` 提供三个字段，其中两个可用；
      不把第三个钉死，下一个写测试的人会理所当然地写上 `assert got.depth == 1`
      —— 然后花半天去查一个根本不存在的 DUT 缺陷。
    """
    with _fixture_only() as sc:
        for depth in (1, 10):
            topic = f"/qosprobe/depth{depth}"
            sc.node.create_publisher(
                OffboardControlMode, topic,
                QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                           reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            sc.hold(0.1)                     # 让发现完成
        infos = {d: sc.node.get_publishers_info_by_topic(f"/qosprobe/depth{d}")
                 for d in (1, 10)}
        assert all(infos.values()), "正控：两个发布者都该被发现（否则下面的断言只是「没接上」）"
        q1, q10 = (infos[d][0].qos_profile for d in (1, 10))
        assert q1.depth == 0 and q10.depth == 0, \
            "depth 读回来不再是 0 ⟹ 这套栈变了，`publisher_qos` 的说明与票面的验收方式都该重写"
        assert q1.history == HistoryPolicy.UNKNOWN, \
            f"history 读回来是 {q1.history!r} ⟹ 同上，这套栈变了"
        assert q1.depth == q10.depth, "depth=1 与 depth=10 必须读出同一个值（这正是它不可用的原因）"


def test_导航状态常量与_px4_msgs_里的一致():
    """夹具抄的枚举值必须与消息定义逐字一致（写错一个数的后果见文件顶部的说明）。

    ⚠️ 这里**不重抄** `px4_msgs` 的整张表：只对夹具用到的那几个，且值从
      `VehicleStatus` 的**常量**读（那是 `msg` 里生成的，`px4_msgs` 自己就是据）。
    """
    from px4_msgs.msg import VehicleStatus

    assert NAV_STATE_OFFBOARD == VehicleStatus.NAVIGATION_STATE_OFFBOARD
    assert NAV_STATE_POSCTL == VehicleStatus.NAVIGATION_STATE_POSCTL
    assert ARMING_STATE_ARMED == VehicleStatus.ARMING_STATE_ARMED
