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
