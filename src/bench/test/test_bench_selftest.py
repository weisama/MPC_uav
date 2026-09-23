"""夹具自检 —— **假 mavros 假得像不像**。

★ 为什么夹具要有自己的测试：它骗不到被测节点的前提是先骗过自己。
  一个把 `/mavros/local_position/pose` 发成可靠 QoS 的假 mavros，会让**所有**依赖它的测试
  出现"莫名其妙收不到数据"；而那种故障在日志里看不出来（QoS 不匹配不报错，只是静默不匹配）。
  这个文件把这几条"静默失败"变成会红的一行断言。

★ 这些用例**不起 DUT**（`start_dut=False`）：它们要回答的是"夹具自己的行为对不对"，
  与被测节点无关。DUT 相关的场景在 `src/control/test/`。
"""

import time

import pytest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
from rclpy.qos import QoSProfile, ReliabilityPolicy

from bench import FakeTraj, Plant, Scenario, fly
from bench.fake_mavros import mavros_default_qos, mavros_sensor_qos, mavros_state_qos
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


# ─────────────── ticket 08（起飞/悬停）第一次用到的注入能力，各来一条自检 ───────────────

def test_速度注入真的走到_local_position_velocity_话题上():
    """`set_velocity` —— 悬停判稳判据里**速率那一半**的唯一注入面（ticket 08 的 ★ 项）。

    ★ 为什么要专门自检"位置不动、速度很大"这种**内部不自洽**的场面：
      真 mavros 里位置与速度是同一个回调（`local_position.cpp`）发的，物理上自洽；
      但 DUT 订的是两条话题，所以判据的两半必须能**分别**被证伪。否则
      `|v| < 0.10` 那一半在台架上根本没有反例，测试会在一半判据永不触发的情况下变绿。
    ⚠️ 这里只证明夹具送得出，不断言"DUT 会怎样"（那是 `src/control/test/` 的事）。
    """
    from geometry_msgs.msg import TwistStamped

    with _fixture_only() as sc:
        got = []
        _subscribe(sc, TwistStamped, sc.fake.topics["local_velocity"], got.append,
                   mavros_sensor_qos())
        sc.advance_to(0.5)
        assert got, "假 mavros 一帧 local_position/velocity_local 都没发出来"
        assert got[-1].header.frame_id == "map", "真 mavros 的速度话题也用 frame_id = 'map'"
        assert (got[-1].twist.linear.x, got[-1].twist.linear.y, got[-1].twist.linear.z) \
            == (0.0, 0.0, 0.0), "默认速度应当是零（= 静止）"

        # 位置**不动**、速度非零：这是"位置看着挺好但还在飘"的场面。
        sc.fake.set_position(0.0, 0.0, 1.0)
        sc.fake.set_velocity(0.0, 0.0, -0.3)
        sc.advance_to(1.0)
        assert got[-1].twist.linear.z == pytest.approx(-0.3), (
            f"注入的速度没到话题上，实测 {got[-1].twist.linear.z}")
        assert got[-1].twist.linear.x == 0.0, "只该改被点名的那一轴"

        # ★ 与位置**解耦**：两条话题各自独立，位置仍是刚设的 1.0。
        pose = []
        _subscribe(sc, PoseStamped, sc.fake.topics["local_position"], pose.append,
                   mavros_sensor_qos())
        sc.advance_to(1.4)
        assert pose and pose[-1].pose.position.z == 1.0
        assert got[-1].twist.linear.z == pytest.approx(-0.3), "改位置不该顺带改速度"

        sc.fake.set_velocity(0.0, 0.0, 0.0)
        sc.advance_to(1.8)
        assert got[-1].twist.linear.z == 0.0, "恢复零速之后必须能回到静止"


def test_机头注入真的改掉_local_position_的四元数():
    """`set_yaw` —— ticket 08「yaw = 起飞机头，锁定后不跟随」的唯一注入面。

    ⚠️ 转的是**四元数**，而 DUT 用 `yaw_from_quaternion` 反解 ⟹ 这条自检必须验证
    **整条链**：`set_yaw(θ)` → 四元数 → （DUT 那侧）反解回 θ。只断言"四元数变了"是不够的
    ——四元数的符号与轴序写错时，它照样"变了"，但反解出来是另一个角。
    这里自己做一次同样的反解，把口径钉死为 **ENU 绕 +z、逆时针为正**。
    """
    import math

    def yaw_of(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    with _fixture_only() as sc:
        got = []
        _subscribe(sc, PoseStamped, sc.fake.topics["local_position"], got.append,
                   mavros_sensor_qos())
        sc.advance_to(0.3)
        assert got and yaw_of(got[-1].pose.orientation) == pytest.approx(0.0), \
            "默认机头应当是 0（单位四元数）"

        sc.fake.set_yaw(0.7)
        sc.advance_to(0.6)
        assert yaw_of(got[-1].pose.orientation) == pytest.approx(0.7, abs=1e-6), (
            f"注入的机头没有（正确地）到话题上，反解出来是 {yaw_of(got[-1].pose.orientation)}")
        # 逆时针为正：负角必须反解回负角（符号写错时这一条会红）。
        sc.fake.set_yaw(-1.2)
        sc.advance_to(0.9)
        assert yaw_of(got[-1].pose.orientation) == pytest.approx(-1.2, abs=1e-6), \
            "yaw 的符号约定写错了（ENU 下逆时针为正）"
        # 位置不该被牵连（两者共用同一帧消息）。
        assert got[-1].pose.position.x == 0.0


def test_解锁服务可以答应_拒绝_或不存在():
    """`set_arming_policy` / `follow_arming` —— ticket 08 三条不同失效路径的注入面：

      · `'ok'`                  ⟹ `success=True`、`result=0`，且 armed 随后变真
      · `'fail'`                ⟹ `success=False`、`result=1`（TEMPORARILY_REJECTED）；
                                   **armed 不许变**，DUT 该报原因 + 退回 STANDBY、不重试
      · `follow_arming=False`   ⟹ 应答是肯定的，但 armed 永不上升沿（DUT 只能等到超时）
                                   ★ 与 `'fail'` 是**两种不同的失效**，必须能分开造
      · `'absent'`              ⟹ 服务不存在（DUT 走 `service_is_ready() == False`）

    ⚠️ `CommandBool` 的应答字段是 `success` + `result`（MAV_RESULT 码），不是
    `SetMode` 的 `mode_sent` —— 写错字段名的测试会永远读到默认的 False 而"看起来通过了"。
    """
    from mavros_msgs.srv import CommandBool

    with _fixture_only() as sc:
        client = sc.node.create_client(CommandBool, sc.fake.topics["arming"])
        assert client.wait_for_service(timeout_sec=5.0)

        def call(value=True):
            req = CommandBool.Request()
            req.value = value
            future = client.call_async(req)
            sc.wait_for(lambda: future.done(), timeout_s=5.0, message="arming 没有回应")
            return future.result()

        def wait_armed(expected, timeout_s=3.0):
            return sc.poll(lambda: sc.fake.armed is expected, timeout_s=timeout_s)

        # ① 接受：success / result=0，且跟随解锁（state 是 1 Hz ⟹ 最多等 1 s）。
        resp = call()
        assert resp.success is True and resp.result == 0
        assert wait_armed(True), "policy='ok' 时夹具没有真的把 armed 置真"
        assert sc.fake.arming_requests[-1].value is True
        assert len(sc.fake.arming_requests) == 1, "夹具该把每一次调用都记下来"

        # ② 拒绝：应答否定、armed **不许**变（此刻已被 ① 置真，先恢复）。
        assert wait_armed(True)
        sc.fake.set_state(armed=False)
        assert wait_armed(False)
        sc.fake.set_arming_policy("fail")
        resp = call()
        assert resp.success is False, "policy='fail' 必须应答否定"
        assert resp.result == 1, "MAV_RESULT 码该是 1（TEMPORARILY_REJECTED），DUT 报原因靠它"
        sc.advance_to(sc.t() + 1.5)
        assert sc.fake.armed is False, "被拒绝的解锁请求不该让 armed 变真"

        # ③ 答应但不真解锁：与 ② 的区别正是 DUT 能观测到的东西（应答 vs 上升沿）。
        sc.fake.set_arming_policy("ok")
        sc.fake.follow_arming = False
        resp = call()
        assert resp.success is True, "这一路上应答必须是肯定的"
        sc.advance_to(sc.t() + 1.5)
        assert sc.fake.armed is False, "follow_arming=False 时 armed 必须一直不变"
        sc.fake.follow_arming = True

        # ④ 服务不存在。
        sc.fake.set_arming_policy("absent")
        sc.wait_for(lambda: not client.service_is_ready(), timeout_s=15.0,
                    message="销毁服务之后客户端仍认为服务可用（DDS 发现没跟上？）")



# ─────────────── ticket 16（EKF2 位置失效的载体）第一次用到的注入能力，各来一条自检 ───────────────

def test_pos_horiz_abs注入真的走到_estimator_status_话题上():
    """`set_pos_horiz_abs` —— 「EKF2 位置失效」判据（ticket 16）唯一的注入面。

    ★★ 本条的**核心**是 QoS 那一半，而不是那位真假值：
      DUT 侧**故意用易失订阅**（`QoS(10)`）订这条话题 —— 真实 mavros 用
      `transient_local` 发（`mavros::StateQoS()`），durable 订阅会在启动时回放上一会话的
      ~10 帧缓存，把"连续为假"的计数当场喂满（2026-09-22 现场实测到过那个回放突发）。
      我们要求的是"两者兼容（可靠 + 可靠）、但不回放" ⟹ 夹具必须照**真 mavros 的发布方**
      抄（可靠 + 瞬态本地），否则这条"会不会被历史喂饱"的性质在台架上根本复现不出来。
      这里就用 DUT 的那种订阅 profile（`mavros_default_qos()` = 可靠 / 易失）去接：
      **接不上 ⟹ 真机上 DUT 也接不上**（QoS 不匹配是静默的，只有这条断言能提前抓出来）。

    ⚠️ 只证明夹具送得出，不断言 DUT 会怎样（那是 `src/control/test/` 的事）。
    """
    from mavros_msgs.msg import EstimatorStatus

    with _fixture_only() as sc:
        got = []
        _subscribe(sc, EstimatorStatus, sc.fake.topics["estimator_status"], got.append,
                   mavros_default_qos())   # ← DUT 用的那种（可靠 / 易失）
        sc.advance_to(1.5)
        assert got, ("DUT 的订阅 profile（可靠/易失）一帧都收不到 ⟹ 真机上 DUT 也收不到"
                     "（QoS 不匹配不报错，只会静默没有数据）")
        assert got[-1].pos_horiz_abs_status_flag is True, "默认应当是「有绝对参考」"

        sc.fake.set_pos_horiz_abs(False)
        sc.advance_to(2.5)   # 1 Hz ⟹ 最多 1 s 之后
        assert got[-1].pos_horiz_abs_status_flag is False, "注入的状态位没到话题上"

        # ★ 别的位必须保持 False（夹具只填 DUT 真正读的那一位）：
        #   全填 True 会让"以后有人顺手改成读另一位"这种改动在台架上照样绿。
        assert not got[-1].pos_horiz_rel_status_flag, "夹具不该顺手把别的位也点亮"
        assert not got[-1].velocity_horiz_status_flag, "夹具不该顺手把别的位也点亮"
        assert not got[-1].attitude_status_flag, "夹具不该顺手把别的位也点亮"

        sc.fake.set_pos_horiz_abs(True)
        sc.advance_to(4.0)
        assert got[-1].pos_horiz_abs_status_flag is True, "恢复之后必须能回到真"

        # ★ 帧计数（seam 1 的"连续 N 帧"消抖要靠它精确构造）。
        n0 = sc.fake.estimator_status_frames
        assert n0 >= 3, f"帧计数没有跟着发布走，实测 {n0}"
        sc.advance_to(5.2)
        assert sc.fake.estimator_status_frames > n0, "帧计数停住了"


# ─────────────── ticket 18（mavros 的合成断连帧）第一次用到的注入能力，各来一条自检 ───────────────

def test_合成断连帧只发一帧_且字段是常量不是夹具属性():
    """`synth_disconnect()` —— mavros **自己**发的那帧状态（ticket 18）唯一注入面。

    ★ 必须复刻的两点，缺一个测的就不是真机那个形状：
      1. **只发一帧** —— 真 mavros 由 `conn_timeout` 的 wall timer 发它（`sys_status.cpp:903-911`，
         `:1411` 触发），而 state 此后只在**收到 HEARTBEAT** 时才发 ⟹ 链路断着就一条都不再有。
         夹具若继续 1 Hz 发这一帧，"闩锁会不会被清掉"测的就不再是"那一帧"了。
      2. **字段是常量、不是夹具属性** —— 真 mavros 直接填 `armed=false`，
         与它对飞控的了解无关。下面的正控先让夹具自己**armed=True**，再按合成帧，
         断言话题上来的仍是 `armed=false`：两者分不开的话，DUT 那边"这帧的 armed 不能当读数"
         这条语义在台架上就无法被证伪。

    ⚠️ 只证明夹具送得出、送得对，不断言 DUT 会怎样（那是 `src/control/test/` 的事）。
    """
    with _fixture_only() as sc:
        got = []
        _subscribe(sc, State, sc.fake.topics["state"], got.append, mavros_state_qos())
        sc.fake.set_state(connected=True, armed=True, mode="OFFBOARD",
                          guided=True, manual_input=True)
        sc.advance_to(1.4)   # 1 Hz ⟹ 先让"真状态"（armed=True）到过话题上

        assert got[-1].armed is True and got[-1].connected is True, \
            "正控：合成帧之前，话题上应当是那次「真状态」"
        # ★ 正控的第二半：那四个 `false` 字段在夹具里**本来是真的** ⟹ 下面"它们全为 false"
        #   才是「常量覆盖了属性」，而不是"夹具从来没说过 true"（否则那条断言恒真、零判别力）。
        assert got[-1].guided is True and got[-1].manual_input is True, \
            "正控：合成帧之前，guided / manual_input 应当是夹具设的 true"
        n_before = len(got)

        sc.fake.synth_disconnect()
        # ⚠️ 用 `sc.poll()` 而**不是** `sc.wait_for()`：后者成功时返回 `None`（超时抛
        #    `BenchTimeout`），写进 `assert` 会永远失败 —— 夹具的等待 API 有"抛异常版"与
        #    "布尔版"两种，别混用（`bench/README.md` 的运行纪律，ticket 06 踩过一次）。
        assert sc.poll(lambda: len(got) > n_before, timeout_s=3.0), \
            "合成断连帧一帧都没发出来（它该由 state 的定时器拍发一次）"

        frame = got[n_before]
        assert frame.connected is False and frame.armed is False, (
            f"合成帧的字段不对：connected={frame.connected}, armed={frame.armed} —— "
            "真 mavros 在这帧里填的是常量 false，不是它对飞控的读数")
        assert frame.guided is False and frame.manual_input is False, (
            f"合成帧的 guided / manual_input 也是常量 false：guided={frame.guided}, "
            f"manual_input={frame.manual_input} —— 与 `armed` 同源，四个布尔全是占位值")
        assert frame.mode == "", "合成帧的 mode 是空串（真 mavros 逐字如此）"
        assert frame.system_status == 0, "合成帧的 system_status 是 UNINIT(0)"
        assert sc.fake.armed is True, (
            "★ 合成帧**不该改夹具自己的属性** —— 那些代表「下一帧真状态会说什么」，"
            "真 mavros 也不改它的内部状态")

        # ★ 只发一帧：此后静默（等 `reconnect_state`）。
        sc.advance_to(sc.t() + 3.0)
        assert len(got) == n_before + 1, (
            f"合成帧之后 state 应当**静默**（等下一帧 HEARTBEAT），却又来了 {len(got) - n_before - 1} 帧")

        # ★ 恢复：真 mavros 收到 HEARTBEAT ⟹ 话题回来了，报的是**真的**状态。
        sc.fake.reconnect_state(connected=True, armed=True, mode="POSCTL")
        assert sc.poll(lambda: len(got) > n_before + 1, timeout_s=3.0), \
            "reconnect_state 之后 state 没有恢复发布"
        assert got[-1].connected is True and got[-1].armed is True and got[-1].mode == "POSCTL"


def test_estimator_status的QoS与真_mavros一致():
    """夹具的发布方 QoS 必须是 `mavros::StateQoS()`（可靠 + **瞬态本地**）。

    ★ 为什么这一条在 ticket 16 里格外重要：DUT 侧**故意用易失订阅**（见 `px4_interface.cpp`
      那段 ★★ —— durable 订阅会在启动时回放上一会话的 ~10 帧缓存，把"连续为假"的计数
      当场喂满）。"易失订阅者接得上、但不拿缓存"这件事**只有夹具照抄真 mavros 的发布方
      QoS 时**才能在台架上复现：夹具若把发布方降成易失，这条性质就测不出来了。
    ★ 与上一条分工：上一条测"DUT 的订阅 profile 接得上"，这一条测"夹具抄的是真 mavros"。

    证据：`sys_status.cpp:579-584` 里 `/mavros/state` 与 `/mavros/estimator_status` 用的是
    **同一个** `mavros::StateQoS()`。
    """
    from mavros_msgs.msg import EstimatorStatus

    with _fixture_only() as sc:
        sc.advance_to(1.6)   # 先保证已经发过至少一帧

        durable, volatile = [], []
        _subscribe(sc, EstimatorStatus, sc.fake.topics["estimator_status"], durable.append,
                   mavros_state_qos())          # 瞬态本地：晚订阅也该**立刻**拿到缓存帧
        # 易失（= DUT 那种订阅 profile）：只能等下一帧（1 Hz）——与上面那条 state 自检同一个写法。
        _subscribe(sc, EstimatorStatus, sc.fake.topics["estimator_status"], volatile.append,
                   QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # ★★ 发现屏障（`bench/README.md` 纪律 1）：**两个**订阅者都匹配上之后才谈"收没收到" ——
        #   否则"易失那个没收到缓存"与"它压根还没接上"在日志里长得一模一样（都是空列表）。
        #   ⚠️ 这条是按复核意见补的（2026-09-22）：本文件里既有的 state QoS 用例是同一写法、
        #   同一处缺口，**未**一并改（那是 ticket 05 的成品，改它要单独记账）。
        assert sc.wait_for_subscribers("estimator_status", count=2), (
            "夹具的发布者没能跟两个旁路订阅者匹配上 ⟹ 下面的『易失没拿到缓存』不成立")
        sc.advance_to(1.85)

        assert durable, ("瞬态本地的晚订阅者没拿到缓存 ⟹ 夹具的发布方不是 durable，"
                         "与真 mavros 的 `StateQoS()` 不一致")
        assert not volatile, ("易失的晚订阅者不该拿到缓存帧。若这里失败，说明夹具把发布方"
                              "降成了易失 —— 那么「DUT 不会被历史回放喂饱」这条性质就测不出来了")


# ─────────────── ticket 09（轨迹跟踪）第一次用到的两件夹具，各来一组自检 ───────────────
#
# ★ 它们假的是**两个不同的东西**，所以自检的问法也不同：
#   · `Plant`     —— 假的是"飞机在动"。要证的是**它真的会动、且速度是真的**：
#                    一个"位置恒为 0、速度恒为 0"的假飞机不会让任何测试变红，
#                    只会让所有判据在一半前提（`|v| < 0.10`）恒真的情况下变绿。
#   · `FakeTraj`  —— 假的是**另一个 ROS 节点**。要证的是它把真 `traj` 那几条
#                    **会让 DUT 走错分支**的行为一条不差地复刻了（没收到 `/traj/start`
#                    不发帧、重选会静默、没选中时忽略 start、跑完停在 1.0）。
#                    这四条在话题层全都表现为"没数据"，错了不会报错 —— 只会静默地测错东西。

def _traj_target(z: float = 1.0) -> PositionTarget:
    """一帧 setpoint（形状同 control 发的：ENU 填位置，掩码只留位置+速度+yaw）。"""
    msg = PositionTarget()
    msg.type_mask = 2552
    msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    msg.position.z = z
    return msg


def test_假飞机跟着_setpoint_动且速度是真的():
    """★★ 本组最重要的一条：**假飞机必须真的动，而且报到话题上的速度是真的**。

    `FakeMavros` 刻意不含动力学（见 `src/bench/README.md`），于是 ticket 08 起
    "爬到 1 m""`|v| < 0.10`"这类判据的**两个前提**（位置会变、速度非零）全落在 `Plant` 上。
    它若只会"把位置直接设成目标"或"速度恒为 0"，那些判据在台架上就有一半没有反例。

    ★ 同时钉住**限速**这个模型本身：爬升段的速度必须恰好等于 `rate_mps`。
      随口改成"指数逼近"之类的模型，`rate_mps` 就不再是"飞机多快"，
      而测试里凡是用它算时间的断言（"1 m 要 0.67 s"）会悄无声息地全错。
    """
    from geometry_msgs.msg import TwistStamped

    with _fixture_only() as sc:
        # ⚠️ 顺序要紧：`Plant.__init__` 记的是**当时**已有的 setpoint 条数，
        #    它之后才到的帧才算"新指令"。先建 plant 再开定时器，别让定时器抢跑。
        plant = Plant(sc, rate_mps=1.5)
        target = _traj_target(1.0)
        pub = sc.node.create_publisher(PositionTarget, sc.fake.topics["setpoint"], 10)
        sc.node.create_timer(0.02, lambda: pub.publish(target))   # 50 Hz，同 DUT

        pose, vel = [], []
        _subscribe(sc, PoseStamped, sc.fake.topics["local_position"], pose.append,
                   mavros_sensor_qos())
        _subscribe(sc, TwistStamped, sc.fake.topics["local_velocity"], vel.append,
                   mavros_sensor_qos())

        fly(sc, plant, 2.0)   # 目标 1 m、限速 1.5 m/s ⟹ 0.67 s 到位

        assert pose and vel, "假飞机没有把位置/速度送到话题上"
        assert pose[-1].pose.position.z == pytest.approx(1.0, abs=1e-6), (
            f"假飞机没有跟着 setpoint 升到 1 m，实测 {pose[-1].pose.position.z}")
        assert max(v.twist.linear.z for v in vel) == pytest.approx(1.5, abs=1e-9), (
            "爬升段的速度必须恰好是 rate_mps（速度是「实际位移 / dt」算出来的，"
            "不是夹具另设的一个数）")
        # ★ 到位之后速度必须归零：否则"位置对了但还在飘"会变成每个用例的默认场面。
        assert vel[-1].twist.linear.z == pytest.approx(0.0, abs=1e-9), (
            f"到位之后速度该是 0，实测 {vel[-1].twist.linear.z}")
        assert pose[-1].pose.position.x == 0.0, "只该动被点名的那个轴"


def test_假飞机只在有新_setpoint_时动一格():
    """没有新指令就不许动 —— 这条防的是**夹具自己造出来的假故障**。

    `Plant.step()` 用 `dt = 本次 step 与上次 step 的间隔` 算速度。若它每调一次就挪一格，
    而调用方（`fly()` 的 0.02 s 轮询）比 DUT 的 50 Hz 快，两格之间**没有**新 setpoint，
    位移为 0 ⟹ 报出来的速度会在 0 与 2× 之间跳 —— 真 mavros 不会这样，
    这是夹具自己造的抖动，而它会污染每一条与速度有关的判据。
    """
    from geometry_msgs.msg import TwistStamped

    with _fixture_only() as sc:
        plant = Plant(sc, rate_mps=1.5)
        pub = sc.node.create_publisher(PositionTarget, sc.fake.topics["setpoint"], 10)
        vel = []
        _subscribe(sc, TwistStamped, sc.fake.topics["local_velocity"], vel.append,
                   mavros_sensor_qos())

        pub.publish(_traj_target(1.0))
        assert sc.poll(lambda: len(sc.fake.setpoints) >= 1, timeout_s=5.0), \
            "自己发的 setpoint 没进夹具记录（发现没跟上？）"
        sc.hold(0.05)
        plant.step()
        z_after_first = plant.z
        n_after_first = len(vel)
        assert z_after_first > 0.0, "有新 setpoint 时该动（否则下面那条断言证明不了什么）"

        plant.step()          # 没有新 setpoint
        plant.step()
        sc.hold(0.1)
        assert plant.z == z_after_first, "没有新指令却又挪了一格"
        # ⚠️ 不能数"速度话题上多了几帧"：假 mavros 按自己的 30 Hz 定时器**持续重发同一个值**
        #    （那是它作为"EKF 出向"的行为，不是 `Plant` 又算了一次）。要证的是**值没变** ——
        #    变了就说明 `step()` 在没有新指令时又算了一次位移（速度会变成 0 或 2×）。
        later = [v.twist.linear.z for v in vel[n_after_first:]]
        assert later, "0.1 s 内一帧速度都没到（那这条断言证明不了什么）"
        assert {round(z, 9) for z in later} == {1.5}, (
            f"没有新指令却又重算了一次速度：{[round(z, 3) for z in later]}（应当一直是 1.5）")


def _fake_traj_with_probes(sc, **kw):
    """起一个 `FakeTraj` 并接好三件事的旁路：参考、完成度、以及**我们自己的** `/traj/start` 发布者。

    ★ 两道**发现屏障**（`bench/README.md` 纪律 1）都是必需的：
      · 夹具的发布者要匹配上旁路订阅者，否则"没收到帧"与"压根还没接上"长得一模一样；
      · 夹具的订阅者要匹配上我们的 `/traj/start` 发布者，否则"发出去没反应"可能是根本没送到。
    """
    from std_msgs.msg import Empty, Float32

    traj = FakeTraj(sc.node, sc, **kw)
    refs, progs = [], []
    _subscribe(sc, PositionTarget, "/traj/reference", refs.append, mavros_default_qos())
    _subscribe(sc, Float32, "/traj/progress", progs.append, mavros_default_qos())
    start_pub = sc.node.create_publisher(Empty, "/traj/start", 10)

    assert sc.poll(lambda: traj.matched_subscribers() >= 1, timeout_s=5.0), \
        "旁路订阅者没接上 /traj/reference（发现没跟上）"
    assert sc.poll(lambda: start_pub.get_subscription_count() >= 1, timeout_s=5.0), \
        "我们的 /traj/start 发布者没接上夹具的订阅 —— 下面「发了 start」这个前提不成立"
    return traj, refs, progs, start_pub


def test_假_traj_没收到_start_之前一帧都不发():
    """★ 复刻 `traj_node.cpp::tick()` 的 `if (!running_) return;`（`:120-123` 的注释逐字说明了）。

    ★★ 为什么这条最要紧：真 `traj` **只**在收到 `/traj/start` 之后才发参考。
      夹具若一上来就 50 Hz 地发参考，DUT 在 `TRACK` 里"参考还没来"那条分支
      （`traj_timeout` 降级的入口、也是"进 TRACK 第一帧就有目标"这种错误实现的反例）
      就永远走不到 —— 而它是 ticket 09 最容易写错的一处。
    """
    with _fixture_only() as sc:
        # 这条只验"不发"，用不到那个 start 发布者（屏障仍要建，否则"没接上"与"没发"分不开）。
        traj, refs, progs, _ = _fake_traj_with_probes(sc, duration_s=2.0)
        sc.hold(0.5)   # 50 Hz ⟹ 本该有 ~25 帧
        assert refs == [], "没收到 /traj/start 却发了参考"
        assert progs == [], "没收到 /traj/start 却发了完成度"
        assert traj.starts == [], "夹具不该记到任何 start"
        assert traj.reference_frames == 0
        assert traj.running is False


def test_假_traj_收到_start_后按_duration_推进完成度并停在_1_0():
    """`/traj/start` 之后的完整生命史：从 0 涨到 1、然后**保持 1.0**（不归零、不停流）。

    ★ 后半句是刻意的复刻：跑完之后 `running_` 仍为真、参考停在终点、完成度保持 1.0
      ⟹ 同一个 `control` 进程飞第二个任务时，进 TRACK 的第一帧就会看到一个 1.0。
      DUT 靠"必须先见过 < 1.0"的闩锁挡它（`fsm.hpp::track_started_`）——
      夹具若不保持 1.0，那个闩锁就没有反例。
    """
    from std_msgs.msg import Empty

    with _fixture_only() as sc:
        traj, refs, progs, start_pub = _fake_traj_with_probes(sc, duration_s=1.0)
        start_pub.publish(Empty())
        sc.hold(0.4)

        assert len(traj.starts) == 1, "夹具没收到我们的 /traj/start"
        assert refs and progs, "收到 start 之后该开始发参考与完成度"
        assert refs[-1].header.frame_id == "traj_origin", \
            "参考的 frame_id 要照抄真 traj（`reference_frame_id` 默认值）"
        assert refs[-1].coordinate_frame == PositionTarget.FRAME_LOCAL_NED
        assert 0.0 < progs[-1].data < 1.0, f"0.4 s / 1.0 s 的完成度该在中间，实测 {progs[-1].data}"

        sc.hold(0.8)   # 越过 duration
        assert progs[-1].data == 1.0, f"跑完之后完成度该停在 1.0，实测 {progs[-1].data}"
        n_refs = len(refs)
        sc.hold(0.3)
        assert len(refs) > n_refs, \
            "跑完之后参考仍在发（真 traj 的定时器一直在跑）—— 停了的话「停在终点」就成了「断了」"
        assert progs[-1].data == 1.0, "跑完之后完成度不许自己归零"


def test_假_traj_运行中重选会让流静默且时钟归零():
    """★ 复刻 `traj_node.cpp::select_kind()`：`running_ = false; elapsed_ = 0`，**一帧都不发**。

    ★★ 也就是说"运行中重选轨迹"在话题层**只能表现为静默**（没有"我重选了"这条消息）
      ⟹ DUT 只能靠"参考流断了"推断，并靠 `traj_stream_gap_s` 那道闸门重发 `/traj/start`
      把 `traj` 重新拉起来。夹具若在重选时发一帧（哪怕是 `progress 0.0`），
      那条推断链就不再是"从静默里推"了 —— 测的会是另一件事。
    ⚠️ 与 `clear()` 是两种：`clear()` **会**发一帧 `progress 0.0`（真实现里只有它这么干）。
    """
    from std_msgs.msg import Empty

    with _fixture_only() as sc:
        traj, refs, progs, start_pub = _fake_traj_with_probes(sc, duration_s=5.0)
        start_pub.publish(Empty())
        sc.hold(0.4)
        assert len(refs) > 0 and 0.0 < progs[-1].data < 1.0

        traj.select()          # 运行中重选
        assert traj.elapsed_s == 0.0, "重选要把参考时钟归零"
        sc.hold(0.2)           # 让队列里剩的帧落地
        n_refs, n_progs = len(refs), len(progs)
        sc.hold(0.4)
        assert len(refs) == n_refs, "重选之后参考流必须静默"
        assert len(progs) == n_progs, "重选之后完成度也必须静默（select 什么都不发）"

        start_pub.publish(Empty())   # DUT 的补救：重发 start
        sc.hold(0.4)
        assert len(refs) > n_refs, "重发 /traj/start 之后流该恢复"
        assert progs[-1].data < 1.0, "恢复后完成度从 0 重新算（duration 是 5 s，不该立刻到 1）"
        assert len(traj.starts) == 2, "两次 start 都该被记下来"


def test_假_traj_没选中轨迹时收到_start_一帧都不发但记一笔():
    """复刻 `traj_node.cpp::on_start`：没选中轨迹时只打一条 WARN 就**忽略**。

    ★ 现场最像"按键没反应"的一种场面：`/traj/start` 发了、`control` 那边一切正常，
      而参考一帧都不来（操作员忘了在 `traj` 那边选轨迹）。DUT 的 `diagnose()` 专门为
      这一条写了提示 ⟹ 台架上必须能造出来。
    """
    from std_msgs.msg import Empty

    with _fixture_only() as sc:
        traj, refs, progs, start_pub = _fake_traj_with_probes(
            sc, duration_s=2.0, selected=False)
        start_pub.publish(Empty())
        sc.hold(0.5)

        assert len(traj.starts) == 1, "★ 收到就要**记一笔**（「发了但被忽略」与「根本没发」要能分开）"
        assert refs == [] and progs == [], "没选中轨迹时不该发任何帧"
        assert traj.reference_frames == 0 and traj.running is False


def test_假_traj_上一轮跑完的_1_0_能被造出来也能被_start_抹掉():
    """`replay_done()` —— "上一轮已经跑完"这个场面（DUT 那条闩锁唯一的反例来源）。

    ★★ 它要能造出**一个正在发布的 1.0**：不是"发一帧就停"，而是"参考还在流、完成度一直是 1.0"。
      少了这个前提，`TRACK` 里"看到 1.0 就降落"的判据在台架上没有反例，
      而真实缺陷（同一个 `control` 进程飞第二个任务时当场判"飞完了"）就测不出来。
    ★ 第二半同样重要：`replay_done` 不是单向门 —— 收到 `/traj/start` 之后必须回到从 0 计数，
      否则"重发 start"这条补救在台架上永远看不到效果。
    """
    from std_msgs.msg import Empty

    with _fixture_only() as sc:
        traj, refs, progs, start_pub = _fake_traj_with_probes(sc, duration_s=2.0)
        traj.replay_done()
        sc.hold(0.3)

        assert refs, "上一轮跑完时参考仍在发"
        assert progs and progs[-1].data == 1.0, "上一轮跑完时完成度停在 1.0"
        n_refs = len(refs)
        sc.hold(0.2)
        assert len(refs) > n_refs, "1.0 不是一次性的一帧：参考要持续在流"

        start_pub.publish(Empty())
        sc.hold(0.4)
        assert progs[-1].data < 1.0, "收到新的 /traj/start 之后完成度该从 0 重新算"

