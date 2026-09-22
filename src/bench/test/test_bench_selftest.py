"""夹具自检 —— **假 mavros 假得像不像**。

★ 为什么夹具要有自己的测试：它骗不到被测节点的前提是先骗过自己。
  一个把 `/mavros/local_position/pose` 发成可靠 QoS 的假 mavros，会让**所有**依赖它的测试
  出现"莫名其妙收不到数据"；而那种故障在日志里看不出来（QoS 不匹配不报错，只是静默不匹配）。
  这个文件把这几条"静默失败"变成会红的一行断言。

★ 这些用例**不起 DUT**（`start_dut=False`）：它们要回答的是"夹具自己的行为对不对"，
  与被测节点无关。DUT 相关的场景在 `src/control/test/`。
"""

import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
from rclpy.qos import QoSProfile, ReliabilityPolicy

from bench import Scenario
from bench.fake_mavros import mavros_sensor_qos, mavros_state_qos
from bench.vrpn_source import mocap_qos


def _fixture_only(**kw):
    """只起夹具、不起 DUT —— 场景的 `t()` 尺子仍然可用。"""
    return Scenario(dut=None, start_dut=False, **kw)


def _subscribe(sc, msg_type, topic, callback, qos):
    """在夹具节点上建一个订阅。回调里**只**记到达时刻（`time.monotonic()`）——
    用 ROS 时间或 `time.time()` 都可能在 NTP 校时时跳变，而这里要的是间隔。"""
    return sc.node.create_subscription(msg_type, topic, callback, qos)


# ───────────────────────────── 上游：假 mavros 自己发出的东西 ─────────────────────────────

def test_local_position_按_mavros_的样子发出来():
    """最基本的"活着"检查：话题名对、frame_id 对、有数据。"""
    with _fixture_only() as sc:
        got = []
        _subscribe(sc, PoseStamped, sc.fake.topics["local_position"], got.append,
                   mavros_sensor_qos())
        sc.advance_to(1.0)
        assert got, "假 mavros 一帧 local_position 都没发出来"
        assert got[-1].header.frame_id == "map", "真 mavros 的 local_position 用 frame_id = 'map'"
        assert got[-1].pose.orientation.w == 1.0


def test_state_是_1Hz():
    """★ `/mavros/state` 的频率必须是 1 Hz：**夹具不许比真机更灵敏**。

    实测 1.000 Hz，根源是 mavros 只在收到 MAVLink HEARTBEAT（PX4 的 1 Hz 定频流）时才发它。
    夹具若把它调快，"解锁上升沿最多晚 1 s"这类真问题就会在测试里消失 ⟹ 夹具就成了谎言。
    """
    with _fixture_only() as sc:
        arrival = []
        _subscribe(sc, State, sc.fake.topics["state"],
                   lambda m: arrival.append(time.monotonic()), mavros_state_qos())
        sc.advance_to(3.4)

        assert 2 <= len(arrival) <= 4, f"3.4 s 内应收到 2–3 帧 state，实收 {len(arrival)}"
        gaps = [b - a for a, b in zip(arrival, arrival[1:])]
        if gaps:
            mean_gap = sum(gaps) / len(gaps)
            assert 0.85 <= mean_gap <= 1.15, f"state 的平均间隔应 ≈1.0 s，实测 {mean_gap:.3f} s"


def test_state_是瞬态本地_非瞬态的订阅者拿不到缓存():
    """`/mavros/state` 是 `transient_local`（`mavros::StateQoS()`）。

    晚订阅者被分两类：瞬态本地的**立刻**拿到最后一帧缓存，普通的要等下一帧（1 Hz ⟹ 最多 1 s）。
    这条也在告诉后来人：想"一订阅就看到当前状态"，就得用 `transient_local`。
    """
    with _fixture_only() as sc:
        sc.advance_to(1.6)  # 先保证发过至少一帧

        transient, volatile = [], []
        _subscribe(sc, State, sc.fake.topics["state"], transient.append, mavros_state_qos())
        _subscribe(sc, State, sc.fake.topics["state"], volatile.append,
                   QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        sc.advance_to(1.85)

        assert transient, "瞬态本地的晚订阅者应当立刻拿到缓存帧"
        assert not volatile, "非瞬态的晚订阅者不该拿到缓存帧（它要等下一帧）"


def test_local_position_是_best_effort_可靠订阅者收不到():
    """★★ 本文件最重要的一条：**QoS 不匹配是静默的**。

    真 mavros 用 `SensorDataQoS()`（best_effort）发 `/mavros/local_position/pose`。
    按默认（可靠）订阅的节点**永远收不到**，且不报任何错 —— 这是本项目最容易踩、
    也最难查的一个坑（见 `control/include/control/px4_interface.hpp` 的 QoS 注释）。
    """
    with _fixture_only() as sc:
        reliable_got, sensor_got = [], []
        _subscribe(sc, PoseStamped, sc.fake.topics["local_position"], reliable_got.append,
                   QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        _subscribe(sc, PoseStamped, sc.fake.topics["local_position"], sensor_got.append,
                   mavros_sensor_qos())
        sc.advance_to(1.0)

        assert sensor_got, "best_effort 订阅者应当收到 local_position"
        assert not reliable_got, (
            "可靠订阅者**不该**收到 best_effort 发布方的数据。如果你看到这里失败，"
            "说明这个 DDS 实现放宽了 QoS 兼容性判定 —— 那么 control 的 QoS 对齐就"
            "不能只靠这条测试兜底了，请把发现写进 ticket 05 的 Comments。")


def test_静默是真的不发_不是发空值():
    """`mute` 必须让话题**停止发布**：控制节点的判据是"多久没收到一帧"，
    发空值等于"收到了"（判据恒真）⟹ 夹具必须能造出"真的没有"。"""
    with _fixture_only() as sc:
        got = []
        _subscribe(sc, PoseStamped, sc.fake.topics["local_position"], got.append,
                   mavros_sensor_qos())
        sc.advance_to(0.5)
        assert got, "静默之前应该已经收到过数据（否则这条测试证明不了什么）"

        sc.fake.mute("local_position")
        sc.advance_to(0.7)  # 让静默生效（队列里可能还有一帧）
        n_at_mute = len(got)
        sc.advance_to(1.2)
        assert len(got) == n_at_mute, "静默期间不该再有任何一帧"

        sc.fake.unmute("local_position")
        sc.advance_to(1.6)
        assert len(got) > n_at_mute, "恢复之后必须继续发（否则测的是「恢复不了」）"


def test_改状态真的反映到_state_话题上():
    """`set_state` / `set_voltage` 之类的注入必须真的走到话题上，而不是只改了 Python 属性。"""
    with _fixture_only() as sc:
        got = []
        _subscribe(sc, State, sc.fake.topics["state"], got.append, mavros_state_qos())
        sc.advance_to(0.3)

        sc.fake.set_state(connected=False, mode="OFFBOARD")
        sc.advance_to(2.4)  # 1 Hz ⟹ 最多 1 s 之后应该看到
        assert got[-1].connected is False
        assert got[-1].mode == "OFFBOARD"


# ───────────────────────────── 下游：DUT 会用到的东西 ─────────────────────────────

def test_记录_setpoint():
    """夹具得记下 DUT 发的每一帧 setpoint，并带上到达时刻。

    这里用**恰好可靠**的发布方（= control 节点的做法）来确认：
    它与夹具的 best_effort 订阅方**能**匹配（DDS 的 offered ≥ requested 规则）。
    反过来（DUT 用 best_effort 发）才是收不到的那种 —— 所以 control 那一侧不能改成 best_effort。
    """
    with _fixture_only() as sc:
        pub = sc.node.create_publisher(
            PositionTarget, sc.fake.topics["setpoint"], 10)  # 默认：可靠 / depth 10
        sc.advance_to(0.3)

        sent = PositionTarget()
        sent.type_mask = 2552
        sent.coordinate_frame = 1
        for _ in range(50):
            pub.publish(sent)
            sc.hold(0.02)

        assert sc.fake.setpoints, "夹具没记到任何 setpoint"
        assert sc.fake.setpoints[0].msg.type_mask == 2552
        assert sc.fake.setpoints[0].at_s <= sc.fake.setpoints[-1].at_s


def test_set_mode_被接受时_px4_的_mode_跟着变():
    """夹具模拟"PX4 真的接受 SET_MODE"：下一次 state 发布（1 Hz）时 mode 变 OFFBOARD。"""
    with _fixture_only() as sc:
        client = sc.node.create_client(SetMode, sc.fake.topics["set_mode"])
        assert client.wait_for_service(timeout_sec=5.0), "夹具没提供 /mavros/set_mode"

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = "OFFBOARD"
        future = client.call_async(req)
        sc.wait_for(lambda: future.done(), timeout_s=5.0, message="set_mode 没有回应")

        assert future.result().mode_sent is True
        assert [r.custom_mode for r in sc.fake.mode_requests] == ["OFFBOARD"]
        sc.wait_for(lambda: sc.fake.mode == "OFFBOARD", timeout_s=3.0,
                    message="夹具没有模拟出 PX4 接受 SET_MODE 之后的模式变化")


def test_set_mode_可以回答失败或不存在():
    """ticket 05 要覆盖的两条真实分支：
    `'fail'` = mavros 拒绝（`mode_sent=False`，DUT 该重试）；
    `'absent'` = 服务还没起来（DUT 该走 `service_is_ready() == False` 那条分支）。"""
    with _fixture_only() as sc:
        client = sc.node.create_client(SetMode, sc.fake.topics["set_mode"])
        assert client.wait_for_service(timeout_sec=5.0)

        sc.fake.set_mode_policy("fail")
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = client.call_async(req)
        sc.wait_for(lambda: future.done(), timeout_s=5.0, message="set_mode 没有回应")
        assert future.result().mode_sent is False
        assert sc.fake.mode == "POSCTL", "没被接受时不该改 mode"

        sc.fake.set_mode_policy("absent")
        sc.wait_for(lambda: not client.service_is_ready(), timeout_s=15.0,
                    message="销毁服务之后客户端仍认为服务可用（DDS 发现没跟上？）")


# ─────────────────── ticket 06（失效路径）第一次用到的注入能力，各来一条自检 ───────────────────
# ★ 纪律：**第一次用某个能力时，先证明这个能力真的有效**。上面几条是 ticket 05 用过的；
#   下面这几条都是 ticket 06 新引入的注入面，写在这里是为了让"注入没生效"与"节点没反应"
#   在测试红的时候能一眼分开。

def test_电压注入真的走到_battery_话题上():
    """`set_voltage` / `voltage_drop` 必须真的改掉 `/mavros/battery.voltage`。

    ⚠️ 这一条不测"低压会怎样"（那是 DUT 的事），只测**夹具真的把电压送出去了** ——
    否则 ticket 06 的"低压 ⟹ LAND"测试会在"注入根本没生效"的前提下变绿（假绿）。
    """
    import pytest
    from sensor_msgs.msg import BatteryState

    with _fixture_only() as sc:
        got = []
        _subscribe(sc, BatteryState, sc.fake.topics["battery"], got.append,
                   mavros_sensor_qos())
        sc.advance_to(1.4)
        assert got, "假 mavros 一帧 battery 都没发出来"
        assert got[-1].voltage == 25.0, f"默认电压应该是 25.0（6S 满电附近），实测 {got[-1].voltage}"

        sc.fake.set_voltage(19.3)
        sc.advance_to(2.6)   # battery 1 Hz ⟹ 最多 1 s 之后
        # `BatteryState.voltage` 是 **float32**（真 mavros 也一样），19.3 存进去是 19.299999…
        # ⟹ 这里只能近似比。⚠️ 别把这条改成"恰好相等"：那是在测浮点，不是测夹具。
        assert got[-1].voltage == pytest.approx(19.3, abs=1e-3), \
            f"注入的电压没到话题上，实测 {got[-1].voltage}"


def test_NAN_位置真的能送到_local_position_话题上():
    """`set_position(nan, ...)` —— DUT 的 `position_invalid` 判据（NaN/Inf）的注入面。

    ⚠️ **这条注入是"防御性"的，不是真机现象的复现**。第一手核实（2026-09-22，z0 固件
    `4a0e65f2` 那棵树）：
      · `src/modules/ekf2/EKF2.cpp::PublishLocalPosition` 直接发 `_ekf.getPosition()`，
        **没有任何 NaN 合成**（`lpos.x/y/z` 一路是实数；只有 `ref_lat/lon/alt` 在
        全局原点无效时是 NaN）；
      · `src/modules/mavlink/streams/LOCAL_POSITION_NED.hpp` 把 x/y/z 原样发出，
        **不带 `xy_valid`**（这个 flag 在 MAVLink 这一层就丢了）；
      · mavros 2.15.1 `plugins/local_position.cpp::handle_local_position_ned` 也是原样转 ENU。
    ⟹ 真机上"EKF2 位置失效"**不会**以 NaN 的形式出现在这条话题上（详见 ticket 06 的 Comments）。
    这条自检只证明：**夹具送得出 NaN**，所以 DUT 那个防御性分支在台架上是可测的。
    """
    with _fixture_only() as sc:
        got = []
        _subscribe(sc, PoseStamped, sc.fake.topics["local_position"], got.append,
                   mavros_sensor_qos())
        sc.advance_to(0.5)
        assert got and got[-1].pose.position.x == 0.0, "注入前应当是正常的位置"

        sc.fake.set_position(float("nan"), 0.0, 0.0)
        sc.advance_to(1.0)
        assert got[-1].pose.position.x != got[-1].pose.position.x, (
            "NaN 没有送到话题上（x != x 是 NaN 的判据）—— 那 DUT 的 position_invalid 判据"
            "在台架上就测不到，ticket 06 的 ④ 会变成假绿")
        assert got[-1].pose.position.y == 0.0, "只该坏掉被注入的那一轴"

        sc.fake.set_position(0.1, 0.0, 0.0)
        sc.advance_to(1.5)
        assert got[-1].pose.position.x == 0.1, "恢复有限值之后必须能回到正常（否则测的是「一去不回」）"


def test_遥控注入真的走到_rc_话题上而且静置不动():
    """两条都要：`set_rc_channel` 改得动；**不动它时通道是恒定的**。

    第二条是本条的重点：DUT 的"打杆夺权"判据看的是**变化率**，所以夹具要是让未注入时的
    通道值抖动（或每帧随机），就会出现"飞行员什么都没干却触发了夺权"的假故障 ——
    那种故障在 DUT 的日志里看着完全正常，只有夹具自己是罪魁。
    """
    from mavros_msgs.msg import RCIn

    with _fixture_only() as sc:
        got = []
        _subscribe(sc, RCIn, sc.fake.topics["rc"], got.append, QoSProfile(depth=10))
        sc.advance_to(1.0)
        assert len(got) >= 5, f"1 s 内应收到 ~10 帧 rc（夹具 10 Hz），实收 {len(got)}"

        # 未注入时恒定：所有帧的 1/2/3/4 通道都一样（1500），ch8/ch9 = 1000。
        for msg in got:
            assert list(msg.channels[:4]) == [1500, 1500, 1500, 1500], "静置时摇杆通道不该变"
            assert msg.channels[7] == 1000 and msg.channels[8] == 1000
        # 而且帧**在动**（否则"恒定"是"根本没发"的另一种说法）。
        assert got[0] is not got[-1]

        sc.fake.set_rc_channel(1, 1200)   # 通道号 1-based
        sc.advance_to(1.6)
        assert got[-1].channels[0] == 1200, f"注入的通道值没到话题上：{got[-1].channels[0]}"
        assert got[-1].channels[1] == 1500, "只该改被点名的那一路"


def test_动捕时间戳可以冻住也可以倒退():
    """`hold_stamp` / `rewind` —— ticket 06 的"动捕时间戳不前进"判据的两个注入面。

    ★ 与 `pause`（**停止发布**）是两种不同的失效，必须能分开造：
      · `pause`  ⟹ 话题层没数据（DUT 的"动捕超时"判据抓它）
      · `hold_stamp` ⟹ 帧照发、戳不动（只有"时间戳不前进"那条判据抓得到）
    """
    with _fixture_only() as sc:
        # 夹具默认就起脚本化动捕源（config 的 `mocap.target = 'mocap'` ⟹ 直接发 /mocap/pose_enu）。
        assert sc.mocap_source is not None, "夹具没起动捕源"
        got = []
        _subscribe(sc, PoseStamped, sc.mocap_source.topic, got.append, mocap_qos())
        sc.advance_to(0.5)
        assert got, "脚本化动捕一帧都没发出来"

        # 正常时戳在前进（否则下面的断言证明不了"冻住"是 `hold_stamp` 干的）。
        assert got[-1].header.stamp != got[0].header.stamp, "默认情况下动捕时间戳应该前进"

        sc.mocap_source.hold_stamp(True)
        sc.advance_to(1.0)
        n_before = len(got)
        assert n_before > 1, "冻住时间戳 ≠ 停止发布：帧必须照发"
        sc.advance_to(1.5)
        assert len(got) > n_before, "冻住时间戳期间必须继续发帧（否则测的是 pause）"
        assert got[-1].header.stamp == got[n_before - 1].header.stamp, "时间戳应该冻在同一个值"

        sc.mocap_source.hold_stamp(False)
        sc.advance_to(2.0)
        assert got[-1].header.stamp != got[n_before - 1].header.stamp, "解除冻结之后戳该继续前进"

        # 倒退：戳**比之前更小**（ticket 06 验收项里的"注入时间戳倒退"）。
        sc.mocap_source.rewind(5.0, hold_s=0.5)
        sc.advance_to(2.6)
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in got]
        assert min(stamps) < max(stamps) - 1.0, "rewind 没有造出「戳倒退」的场景"

