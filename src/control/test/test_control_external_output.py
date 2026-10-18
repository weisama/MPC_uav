"""`output_mode=external` 的 seam 1（票 15）—— `control` 只发 `/control/target`，
由外部控制器把最终 setpoint 交给 PX4。

★★ 本文件钉住的是**两种控制能平替**的那条结构：
  · `external` 档下 `control` **一帧都不写** `/mavros/setpoint_raw/local`；
  · 它把**绝对目标**（位置 + 速度 + yaw）发到 `/control/target`，掩码 **2496**；
  · 开始键、解锁、切 OFFBOARD、起飞、判稳、轨迹原点、`/traj/start` **全部照旧** ——
    任务流程一个字都没变（这正是"换后端不换流程"的黑盒证据）。

★ 假飞机（`_TargetPlant`）**不读** `/mavros/setpoint_raw/local`（那条本来就没有帧），
  只读 `/control/target` —— 与真实 external 链上"外部控制器消费目标"的角色一致。
  少了它，"目标算对了没有"在飞机不动的情况下没有落点。
"""

import copy

import pytest
from mavros_msgs.msg import PositionTarget

from bench import DEFAULTS, FakeTraj, Scenario, fly_until

DUT = ("control", "control_node")

#: 内部绝对目标（`control.yaml` 的 `control_target_topic` 默认值）。
TARGET_TOPIC = "/control/target"

#: 与 `test_control_track.py` 同一组参数（只把 `output_mode` 换掉）。
#  ★ 判据阈值一律照 `control.yaml` 的原值；只把时间压短好让用例跑得快。
PARAMS = {
    "output_mode": "external",
    "setpoint_rate_hz": 50.0,
    "enter_offboard": True,
    "offboard_prestream_s": 1.0,
    "offboard_retry_s": 1.0,
    "state_timeout_s": 2.5,
    "local_pos_timeout_s": 0.5,
    "mocap_timeout_s": 0.5,
    "start_channel": 9,
    "start_threshold": 0.75,
    "release_threshold": 0.25,
    "debounce_ms": 200.0,
    "start_low_us": 1000.0,
    "start_high_us": 2000.0,
    "start_land_hold_s": 1.0,
    "takeoff_altitude_m": 1.0,
    "climb_speed_mps": 0.5,
    "takeoff_alt_tol_m": 0.10,
    "takeoff_vel_tol_mps": 0.10,
    "takeoff_settle_s": 1.0,
    "hover_pos_tol_m": 0.10,
    "hover_vel_tol_mps": 0.10,
    "hover_settle_s": 1.0,
    "wait_start_timeout_s": 120.0,
    "takeoff_timeout_s": 5.0,
    "traj_stream_gap_s": 0.5,
    "traj_timeout_s": 3.0,
}

INIT_X, INIT_Y = 0.3, -0.2
START_CHANNEL = 9
START_HIGH_US = 2000
START_LOW_US = 1000
HOLD_S = 0.5

#: 轨迹参考：位置 + **速度**都给非零 ⟹ 后者能证明它真的从 `/traj/reference`
#: 经 `fsm.hpp` 转发到了 `/control/target`（位置那一路在 external 档也走，但少了速度
#: 只有 MPC 的跟随会变慢，话题面上看不出来）。
REF = (0.5, -0.2, 0.1)
REF_V = (0.2, 0.0, 0.0)


def _config():
    cfg = copy.deepcopy(DEFAULTS)
    cfg["initial"]["position"] = [INIT_X, INIT_Y, 0.0]
    return cfg


class _TargetPlant:
    """跟着 `/control/target` 动的假飞机（接口与 `bench.Plant` 相同：`step()`）。

    ★ 与 `bench.Plant` 的唯一差别是**读哪条流**：那条读 `/mavros/setpoint_raw/local`
      （position 档），这条读 `/control/target`（external 档）。
    """

    def __init__(self, sc, frames, rate_mps: float = 1.5):
        self.sc = sc
        self.frames = frames
        self.rate_mps = rate_mps
        self.x, self.y, self.z = (float(v) for v in sc.fake.position)
        self._last_t = sc.t()
        self._last_n = len(frames)

    def step(self) -> None:
        now = self.sc.t()
        if not self.frames or len(self.frames) == self._last_n:
            return
        dt = now - self._last_t
        if dt <= 0.0:
            return
        self._last_n = len(self.frames)
        self._last_t = now
        target = self.frames[-1].position
        limit = self.rate_mps * dt

        def clamp(v):
            return max(-limit, min(limit, v))

        dx = clamp(target.x - self.x)
        dy = clamp(target.y - self.y)
        dz = clamp(target.z - self.z)
        self.x += dx
        self.y += dy
        self.z += dz
        self.sc.fake.set_velocity(dx / dt, dy / dt, dz / dt)
        self.sc.fake.set_position(self.x, self.y, self.z)


def _press(sc):
    sc.fake.set_rc_channel(START_CHANNEL, START_HIGH_US)
    sc.hold(HOLD_S)


def _release(sc):
    sc.fake.set_rc_channel(START_CHANNEL, START_LOW_US)
    sc.hold(HOLD_S)


def test_external档_只发绝对目标_开始键与任务流程照旧():
    """★ 一条用例同时钉住两件事（缺一半都分不清"后端换了"与"任务坏了"）：

    ① **输出面**：`/control/target` 有帧、`/mavros/setpoint_raw/local` **零帧**；
       目标值是**当前位置**（STANDBY 原地保持），掩码 2496。
    ② **任务面**：第 1 次开始键仍然解锁 + 切 OFFBOARD + 起飞到 WAIT_START；
       第 2 次（短按后松手）仍然进 TRACK；目标 = **轨迹原点 + 参考**，且参考速度
       原样出现在 `/control/target` 的 `velocity` 里。
    """
    with Scenario(dut=DUT, params=PARAMS, config=_config(), start_dut=False) as sc:
        frames = []
        sc.node.create_subscription(PositionTarget, TARGET_TOPIC, frames.append, 10)
        traj = FakeTraj(sc.node, sc, reference_fn=lambda _t: REF,
                        velocity_fn=lambda _t: REF_V)
        sc.start_dut()

        for key in ("state", "local_position", "local_velocity", "rc", "battery"):
            assert sc.wait_for_subscribers(key), f"DUT 没有接上 /mavros 的 {key}"
        sc.wait_for(lambda: len(frames) >= 20, timeout_s=15.0,
                    message="external 档下 /control/target 没有帧 ⟹ 输出后端没接上")
        assert sc.setpoints() == [], (
            "external 档下 control 仍在写 /mavros/setpoint_raw/local —— 两路 setpoint 会互相覆盖")
        # ★ 结构面：不只是"没发帧"，而是**发布者根本没建**。少了这条，
        #   "两档互斥"就只靠调用方记得别调另一个函数。
        assert sc.node.count_publishers("/mavros/setpoint_raw/local") == 0, (
            "external 档下仍建了 /mavros/setpoint_raw/local 的发布者 —— 两档输出没有结构互斥")
        assert traj.matched_subscribers() >= 1, "DUT 没订上 /traj/reference"

        first = frames[-1]
        assert first.type_mask == 2496, (
            f"/control/target 的掩码是 {first.type_mask}，该是 2496（位置 + 速度 + yaw）")
        # STANDBY 的语义：目标就是当前位置（原地保持）。
        assert (first.position.x, first.position.y, first.position.z) == pytest.approx(
            (INIT_X, INIT_Y, 0.0), abs=1e-6)
        assert (first.velocity.x, first.velocity.y, first.velocity.z) == pytest.approx(
            (0.0, 0.0, 0.0), abs=1e-6)

        sc.wait_for(lambda: sc.fake.mode == "OFFBOARD", timeout_s=10.0,
                    message="external 档没有把假 PX4 拉进 OFFBOARD")
        sc.hold(1.2)   # `/mavros/state` 是 1 Hz（解锁前置里的 in_offboard 最多晚 1 s）

        plant = _TargetPlant(sc, frames)
        _press(sc)     # 第 1 次上升沿 = 解锁 + 起飞
        fly_until(sc, plant, lambda: sc.control_state() == "AUTO/WAIT_START",
                  timeout_s=25.0,
                  message="external 档下第 1 次开始键没有走到 WAIT_START")
        assert len(sc.fake.arming_requests) == 1, (
            f"第 1 次按键该解锁**一次**，实际 {len(sc.fake.arming_requests)} 次")
        # ★ 整段任务里都不许出现第二条流。
        assert sc.setpoints() == [], (
            "起飞过程中 control 又写了 /mavros/setpoint_raw/local —— 两档输出没有互斥")

        origin = tuple(sc.fake.position)
        assert traj.reference_frames == 0, "还没进 TRACK，假 traj 就开始发参考了"
        _release(sc)
        _press(sc)     # 第 2 次上升沿：记下"欠一次开始轨迹"
        _release(sc)   # 松手 ⟹ 真的进 TRACK
        fly_until(sc, plant, lambda: sc.control_state() == "AUTO/TRACK",
                  timeout_s=15.0,
                  message="external 档下第 2 次开始键没有进 TRACK")
        fly_until(sc, plant, lambda: traj.reference_frames > 0, timeout_s=10.0,
                  message="DUT 没有发 /traj/start（开始键到 TRACK 的那一段没接上）")
        sc.hold(0.5)   # 让参考与目标都稳定几拍

        last = frames[-1]
        assert (last.position.x, last.position.y, last.position.z) == pytest.approx(
            (origin[0] + REF[0], origin[1] + REF[1], origin[2] + REF[2]), abs=1e-3), (
            "TRACK 的目标不是「轨迹原点 + 参考」—— external 档把原点丢了？")
        assert (last.velocity.x, last.velocity.y, last.velocity.z) == pytest.approx(
            REF_V, abs=1e-6), (
            "参考速度没有从 /traj/reference 转发到 /control/target（MPC 会少了前馈）")
        assert sc.setpoints() == [], "TRACK 里 control 仍在写 /mavros/setpoint_raw/local"


def test_不认识的_output_mode_入口当场拒():
    """⛔ 不认识的档**不许**静默退回 `position`：那会让"我明明配了 external"变成
    "飞机不动但每帧话题都正常"—— 现场最难查的一类。断言三层：退出码非 0、
    日志里有参数名 + 两个合法值、一帧 setpoint / 目标都没发。
    """
    with Scenario(dut=DUT, params={**PARAMS, "output_mode": "mpc"},
                  config=_config()) as sc:
        sc.wait_for(lambda: not sc.dut_alive(), timeout_s=10.0,
                    message="output_mode=mpc 没有被拒，节点一直活着")
        sc.hold(0.3)   # 让读取线程排空最后几行
        log = sc.dut_log()
        assert sc.dut_returncode() not in (0, None), \
            f"这个没意义的档没被拒（退出码 {sc.dut_returncode()}；日志尾：{log[-500:]!r}）"
        assert "output_mode" in log and "external" in log and "position" in log, \
            f"拒绝理由没说清参数名与合法值（日志尾：{log[-800:]!r}）"
        assert sc.setpoints() == [], "入口都拒绝了，却还有人写 /mavros/setpoint_raw/local"
        assert sc.node.count_publishers(TARGET_TOPIC) == 0, \
            "入口都拒绝了，却还有人建 /control/target 的发布者"
