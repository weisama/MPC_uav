"""`node_core` 的单元测试 —— **零 ROS、零求解器、零 acados**。

★ 这条接缝（seam 2 的延伸）值得单独测的理由：闸门、外推、参考构造这三件事的错误
  **在话题边界上分辨不出来** —— "节点没发"与"节点发了但发错了"在假 PX4 的记录里
  长得一模一样，而它们的根因完全不同。这里用纯 Python 对象（不 import `px4_msgs`）
  把每一条闸门单独钉死，节点侧的集成测试（`test_mpc_node.py`）就只需要验"接得对不对"。

★ 全部用**鸭子类型**的小替身（`_Status` / `_Pose`）而不是真消息：本模块按契约就
  "不 import ROS" ⟹ 测试若 import `px4_msgs`，那条契约就没人守着了
  （`test_sign_map.py` 里有同样的一条断言，见 `test_不依赖_ros` 那条用例）。
"""

from __future__ import annotations

import dataclasses
import math
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from nmpc.model import NX, quat_angle, yaw_quat
from nmpc.node_core import (ARMING_STATE_ARMED, FALLBACK_HOLD_FRAMES, JUMP_DT_FLOOR_S,
                            NAV_STATE_OFFBOARD, TAU_DEFAULT_S, WARMUP_CYCLES,
                            ControlConfig, FallbackLadder, NodeCore, StopConfig,
                            extrapolate_state, frame_is_finite, hold_reference,
                            mocap_stamp_s, pose_to_state, solution_usable, yaw_from_quat)
from nmpc.ocp import OcpConfig
from nmpc.params import DEFAULT as P
from nmpc.sign_map import Px4RatesSetpoint

N = OcpConfig().N


# ───────────────────────────── 小替身（鸭子类型，不 import ROS）─────────────────────────────

class _Vec:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = float(x), float(y), float(z)


class _Quat:
    def __init__(self, w, x, y, z):
        self.w, self.x, self.y, self.z = float(w), float(x), float(y), float(z)


class _Pose:
    def __init__(self, position, orientation):
        self.position = _Vec(*position)
        self.orientation = _Quat(*orientation)


def _stamp(s: float):
    """`builtin_interfaces/Time` 的形状（`sec` + `nanosec`），两种字段都要齐才算"有时间戳"。"""
    sec = int(s)
    nanosec = int(round((s - sec) * 1e9))
    if nanosec >= 1_000_000_000:      # 四舍五入顶到下一秒（浮点的边界）
        sec, nanosec = sec + 1, nanosec - 1_000_000_000
    return type("_Stamp", (), {"sec": sec, "nanosec": nanosec})()


class _Mocap:
    def __init__(self, position=(0.0, 0.0, 0.0), orientation=(1.0, 0.0, 0.0, 0.0),
                 velocity=None, stamp_s=None):
        self.pose = _Pose(position, orientation)
        # `/mocap/pose_enu` 的 twist 是**可选**的（上游差分节点给了才有）——
        # 缺字段时节点必须退化成"不外推"，而不是崩掉。
        if velocity is not None:
            self.twist = type("_Twist", (), {"linear": _Vec(*velocity)})()
        # 时间戳也是**可选**的：没有 `header` 时那两条判据（倒退 / 跳变）**不存在**，
        # 而不是"判成坏帧"（ticket 07 的 `mocap_stamp_s` 三种"读不出来"都返回 None）。
        if stamp_s is not None:
            self.header = type("_Header", (), {"stamp": _stamp(stamp_s)})()


class _Status:
    def __init__(self, nav_state=NAV_STATE_OFFBOARD, arming_state=ARMING_STATE_ARMED,
                 pre_flight_checks_pass=True):
        self.nav_state = int(nav_state)
        self.arming_state = int(arming_state)
        self.pre_flight_checks_pass = bool(pre_flight_checks_pass)


def _core(config: ControlConfig | None = None) -> NodeCore:
    return NodeCore(P, config or ControlConfig(), N)


# ───────────────────────────── 纯函数 ─────────────────────────────

def test_不依赖_ros():
    """★ 本模块的契约：`node_core` 不许把 `rclpy` / `px4_msgs` 带进来。

    理由不是洁癖：这两个包在**没起 DDS 的进程**里也 import 得动，所以"能跑"证明不了什么；
    而一旦有人图方便在这里 import 一下，`test_node_core.py` 就会从"一毫秒跑完"变成
    "要起 DDS"，且"闸门逻辑"再也无法在纯 Python 里被钉死。

    ★★ 判据必须在**子进程**里取（ticket 06 复核轮改的）：`sys.modules` 是**整个 pytest 进程**的，
      而同一目录下 `test_mpc_node.py`（import `px4_msgs`）与 `test_preflight_check.py`
      （import `nmpc.mpc_node`，连带 `rclpy`）**会先跑**（文件名序：`m` < `n` < `p`）⟹
      直接查 `sys.modules` 时这两条断言必红，于是 `pytest src/nmpc/test/` 一跑就假红，
      而 `colcon test`（一个文件一个进程）照样绿 —— **同一份代码两种结论**，比红更坏。
      子进程里"只 import `node_core`"问的是同一个契约，且与谁先跑无关（判据更强，不是更弱）。
    """
    assert "nmpc.node_core" in sys.modules, "本文件 import 的应该是 node_core 本身"
    code = ("import sys, nmpc.node_core;"
            " assert 'rclpy' not in sys.modules, 'node_core 把 rclpy 带进来了';"
            " assert 'px4_msgs' not in sys.modules, 'node_core 把 px4_msgs 带进来了'")
    done = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert done.returncode == 0, f"node_core 不再是纯模块：{done.stderr.strip()[-300:]}"


def test_yaw_四元数往返得回原角度():
    """`yaw_quat` 与 `yaw_from_quat` 必须互为逆 —— 机头是**参考的一部分**，错一个号就是悄悄自旋。"""
    for yaw in (0.0, 0.3, -1.2, math.pi / 2, math.pi - 0.01, -math.pi + 0.01):
        assert yaw_from_quat(yaw_quat(yaw)) == pytest.approx(yaw, abs=1e-9)


def test_pose_to_state_不做任何坐标系转换():
    """M7 的落点：动捕原生 ENU/FLU → 模型内部 ENU/FLU，**中间没有变换**。

    ★ 这条断言是"删除 NED→ENU 四元数变换"这件事的**守门人**：参考实现里那段手写变换
      如果哪天被谁"顺手加回来"，位置/姿态就会在这里当场变形。
    """
    x = pose_to_state((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0), (0.4, 0.5, 0.6))
    assert x.shape == (NX,)
    assert x[:3].tolist() == [1.0, 2.0, 3.0]
    assert x[3:6].tolist() == [0.4, 0.5, 0.6]
    assert x[6:10].tolist() == [1.0, 0.0, 0.0, 0.0]


def test_pose_to_state_会把四元数归一化():
    """动捕的四元数带噪声（模长 1±ε）；不归一化的话 `R(q)` 不再正交，误差会一路进模型。"""
    x = pose_to_state((0, 0, 0), (2.0, 0.0, 0.0, 0.0))
    assert float(np.linalg.norm(x[6:10])) == pytest.approx(1.0, abs=1e-12)


def test_外推只动位置且正好是_v_乘_tau():
    """票面要求 `x + v·τ`（τ = 50 ms）一行实现 —— 这里把它逐位钉死。"""
    x = np.array([0.0, 0.0, 1.0, 1.0, -2.0, 0.5, 1.0, 0.0, 0.0, 0.0])
    out = extrapolate_state(x, TAU_DEFAULT_S)
    assert out[:3].tolist() == pytest.approx([0.05, -0.1, 1.025], abs=1e-12)
    assert out[3:].tolist() == x[3:].tolist()      # 速度/姿态原样带走


def test_外推的_tau_为_0_就是原值():
    """正控：`τ = 0` 时它必须退回恒等 —— 否则上面那条可能在"到处都在动"的情况下才成立。"""
    x = np.array([1.0, 2.0, 3.0, 9.0, 9.0, 9.0, 1.0, 0.0, 0.0, 0.0])
    assert extrapolate_state(x, 0.0).tolist() == x.tolist()


def test_参考是测量位置加偏移_且姿态水平():
    """参考构造的三条：位置带偏移、速度为零、姿态是**水平 + 当前机头**（不是照抄测量姿态）。"""
    yaw = 0.7
    x_meas = pose_to_state((1.0, -2.0, 3.0),
                           yaw_quat(yaw), (0.0, 0.0, 0.0))
    cfg = ControlConfig(hold_offset=(0.5, -0.25, 0.0))
    ref = hold_reference(x_meas, cfg, N)
    assert ref.shape == (N + 1, NX)
    assert ref[0][:3].tolist() == pytest.approx([1.5, -2.25, 3.0])
    assert ref[0][3:6].tolist() == [0.0, 0.0, 0.0]
    assert yaw_from_quat(ref[0][6:10]) == pytest.approx(yaw, abs=1e-9)
    assert (ref == ref[0]).all(), "整条时域必须是同一个点（位置保持）"


def test_参考的姿态是水平的而不是照抄测量姿态():
    """★ 这条单独写，因为"照抄测量姿态"是一个**看起来更自然**的错法：
    飞机倾斜着悬停时，把倾斜姿态当参考 = 让 MPC（模型预测控制）去维持那个倾斜。"""
    tilted = quat_tilted(roll=0.4, pitch=-0.2)
    ref = hold_reference(pose_to_state((0, 0, 0), tilted), ControlConfig(), N)
    q = ref[0][6:10]
    assert abs(float(q[1])) < 1e-9 and abs(float(q[2])) < 1e-9, "参考不该带横滚/俯仰"


def quat_tilted(roll: float, pitch: float) -> np.ndarray:
    """构造一个带横滚/俯仰的四元数（只用在本文件里，避免与 model 的约定纠缠）。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return np.array([cr * cp, sr * cp, cr * sp, -sr * sp])


# ───────────────────────────── 闸门 ─────────────────────────────

def test_什么都还没收到时不发():
    core = _core()
    c = core.cycle(now_s=0.0)
    assert not c.publish and c.reason == "no_status"
    assert c.x0 is None and c.x_ref is None


def test_未解锁时不发_解锁后才发():
    """★ 票面验收项「未解锁 / 未在 offboard 时**不发布**」的一半。

    ★★ 负断言必须配正控（本仓的纪律）：所以这里在**同一个用例**里先把两个输入喂进去
       证明它**会**发，再把 `arming_state` 拨回去证明它**立刻**不发 ——
       只断言"没发"的话，"夹具/逻辑根本没工作"与"判据正确"长得一模一样。
    """
    core = _core()
    core.on_mocap(_Mocap(), at_s=0.0)
    core.on_status(_Status(nav_state=NAV_STATE_OFFBOARD, arming_state=1), at_s=0.0)
    assert core.cycle(0.1).reason == "not_armed"

    core.on_status(_Status(nav_state=NAV_STATE_OFFBOARD, arming_state=ARMING_STATE_ARMED),
                   at_s=0.1)
    assert _publishes_after_warmup(core, 0.2)

    core.on_status(_Status(nav_state=NAV_STATE_OFFBOARD, arming_state=1), at_s=0.9)
    assert core.cycle(0.95).reason == "not_armed", "解锁掉了必须**当拍**就停"


def test_不在_offboard_时不发():
    """另一半：`nav_state` 一离开 OFFBOARD 就停（打杆接管 / ch6 切模式在节点侧的唯一表现）。"""
    core = _core()
    core.on_mocap(_Mocap(), at_s=0.0)
    core.on_status(_Status(nav_state=2), at_s=0.0)          # 2 = POSCTL
    assert core.cycle(0.1).reason == "not_offboard"
    core.on_status(_Status(nav_state=NAV_STATE_OFFBOARD), at_s=0.1)
    assert _publishes_after_warmup(core, 0.2)


def test_停机条件可以单独关掉():
    """参数化的一层含义：每一条闸门都能单独关（票面「软急停接口参数化」）。"""
    core = _core(ControlConfig(stop=StopConfig(stop_on_nav_state_exit=False,
                                               stop_on_armed_drop=False)))
    core.on_mocap(_Mocap(), at_s=0.0)
    core.on_status(_Status(nav_state=2, arming_state=1), at_s=0.0)
    assert _publishes_after_warmup(core, 0.1)


def test_动捕静默超时会停_恢复后重新热():
    """`mocap_stale` 与"恢复后要重新热身"是同一个用例的两半。"""
    cfg = ControlConfig(stop=StopConfig(mocap_timeout_s=0.5, warmup_cycles=2))
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(), at_s=0.0)
    assert [core.cycle(0.1).reason, core.cycle(0.2).reason] == ["warmup", "warmup"]
    assert core.cycle(0.3).publish
    # ★ 状态那条也得保持新鲜，否则先撞上的是 `status_stale`（0.75 s 的阈值比动捕的 0.5 s 长，
    #   但两者的计时起点不同 ⟹ 不显式给状态，这里测的就不是动捕那条闸门）。
    core.on_status(_Status(), at_s=0.85)
    assert core.cycle(0.9).reason == "mocap_stale"          # 0.9 − 0.0 = 0.9 > 0.5
    core.on_mocap(_Mocap(), at_s=0.95)
    assert core.cycle(1.0).reason == "warmup", "断开之后必须重新热，而不是立刻续上"


def test_状态静默超时会停():
    """`/fmu/out/vehicle_status` 是 2 Hz ⟹ 0.75 s 的阈值只有 1.5 个周期（见 `StopConfig`）。"""
    # ⚠️ 这里刻意用 `warmup_cycles=0`（顺带钉住这个边界：0 = 第一拍就发），
    #    否则下面那两条断言测到的是热身，而不是状态新鲜度那条闸门。
    core = _core(ControlConfig(stop=StopConfig(status_timeout_s=0.75, warmup_cycles=0)))
    core.on_mocap(_Mocap(), at_s=0.0)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(), at_s=0.1)
    assert core.cycle(0.1).publish, "正控：状态新鲜时必须发"
    core.on_mocap(_Mocap(), at_s=0.7)
    assert core.cycle(0.7).publish, "0.70 ≤ 0.75：还在阈值内，不该停"
    core.on_mocap(_Mocap(), at_s=0.76)
    assert core.cycle(0.76).reason == "status_stale", "0.76 > 0.75：越过阈值就当拍停"


def test_坏掉的四元数不发_而好的发():
    """动捕丢帧时会给全零四元数；归一化它只会得到 NaN ⟹ 这一拍绝不能发。"""
    core = _core()
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(orientation=(0.0, 0.0, 0.0, 0.0)), at_s=0.0)
    assert core.cycle(0.1).reason == "bad_mocap_quat"
    core.on_mocap(_Mocap(orientation=(1.0, 0.0, 0.0, 0.0)), at_s=0.1)
    assert _publishes_after_warmup(core, 0.2)


def test_首帧不发的次数正好等于_warmup_cycles():
    """★ 这条盯的是 M6 那类静默失败：`SQP_RTI` 第 0 拍的解是**出厂迭代点**（`WARMUP_CYCLES`）。"""
    core = _core(ControlConfig(stop=StopConfig(warmup_cycles=3)))
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(), at_s=0.0)
    reasons = [core.cycle(0.1 * i).reason for i in range(1, 5)]
    assert reasons == ["warmup"] * 3 + ["ok"]
    # 默认值是 5：票面没规定具体几拍，但"第 0 拍绝对不能发"是硬要求
    assert WARMUP_CYCLES >= 1


def test_外推后的初值与参考不自洽_这正是要的():
    """状态外推补偿的是**执行延迟**，所以"初值比参考靠前"是正常的、必须的。

    ★ 反过来写（把参考也一起外推）会让修正量恰好抵消 —— 那等于没做外推。
    """
    cfg = ControlConfig(tau_s=0.05)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(velocity=(1.0, 0.0, 0.0)), at_s=0.0)
    c = core.cycle(0.1)
    assert c.x0[:3].tolist()[0] == pytest.approx(0.05)
    assert c.x_ref[0][:3].tolist() == [0.0, 0.0, 0.0]


def test_动捕缺_twist_时退化成不外推():
    """`/mocap/pose_enu` 的 twist 由上游可选节点填 —— 缺了不许崩，也不许瞎猜。"""
    core = _core()
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(), at_s=0.0)          # 无 velocity ⟹ 没有 twist 字段
    assert core.cycle(0.1).x0[:3].tolist() == [0.0, 0.0, 0.0]
    assert core.cycle(0.1).x0[3:6].tolist() == [0.0, 0.0, 0.0]


# ───────────────────────────── 输入合理性（ticket 07 / M5）─────────────────────────────
#
# 这一组拦的是"数值上都正常、但**不是这一秒的**"那一帧。★ 判据的取向是**宁可漏判**：
# 误判的代价是**无故停发**（飞机在飞，我们却因为一次正常机动停了指令流）。

def _fed_core(config: ControlConfig | None = None) -> NodeCore:
    """已经"解锁 + OFFBOARD + 动捕新鲜"的核心，用来把新判据单独夹出来。"""
    cfg = config or ControlConfig(stop=StopConfig(warmup_cycles=0))
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    return core


def test_真实机动不会被当成跳变():
    """★ 负控（判据**不许**误伤）：12 m/s 的水平机动逐帧喂进去，一帧都不该被拦。

    12 m/s 是 `MPC_XY_VEL_MAX`（z0 飞控现状）；动捕 200 Hz ⟹ 每帧 0.06 m。
    """
    core = _fed_core()
    for i, t in enumerate([0.0, 0.005, 0.010, 0.015]):
        core.on_mocap(_Mocap(position=(0.06 * i, 0.0, 0.0), stamp_s=100.0 + t), at_s=t)
        assert core.cycle(t).publish, f"第 {i} 帧（0.06 m/5 ms = 12 m/s）被误判成跳变了"


def test_位姿跳变会被拦下且读数进日志():
    """刚体认错 / 遮挡恢复给出的**瞬移**（这里 100 m）⟹ 这一拍不发，且要说清差多少。"""
    core = _fed_core()
    core.on_mocap(_Mocap(stamp_s=100.0), at_s=0.0)
    assert core.cycle(0.0).publish
    core.on_mocap(_Mocap(position=(100.0, 0.0, 0.0), stamp_s=100.005), at_s=0.005)
    c = core.cycle(0.005)
    assert c.reason == "pose_jump" and not c.publish
    assert "100.000" in c.detail and "0.200" in c.detail, f"读数没进日志：{c.detail!r}"


def test_姿态翻转会被拦下():
    """位置没动、姿态翻了 180°（另一个刚体的朝向被认成它了）—— 位置那条拦不住，这条能。"""
    core = _fed_core()
    core.on_mocap(_Mocap(stamp_s=100.0), at_s=0.0)
    assert core.cycle(0.0).publish
    core.on_mocap(_Mocap(orientation=(0.0, 1.0, 0.0, 0.0), stamp_s=100.005), at_s=0.005)
    c = core.cycle(0.005)
    assert c.reason == "attitude_jump" and not c.publish
    assert "180.0°" in c.detail


def test_小转角的真实姿态变化不会被拦():
    """负控：1°/帧（≈3.5 rad/s，比 `YAW_RATE_MAX` 小的机动态）必须放行。"""
    tilt = (math.cos(0.5 * math.radians(1.0)), math.sin(0.5 * math.radians(1.0)), 0.0, 0.0)
    core = _fed_core()
    core.on_mocap(_Mocap(stamp_s=100.0), at_s=0.0)
    assert core.cycle(0.0).publish
    core.on_mocap(_Mocap(orientation=tilt, stamp_s=100.005), at_s=0.005)
    assert core.cycle(0.005).publish, "1° 的转角被判成了姿态跳变"


def test_被拒的那一帧不许把基准搬走():
    """★ 这条是判据成立的前提：拒绝时若更新基准，"跳变来的那一帧"立刻成为新基准 ⟹
    闸门下一拍就开，判据等于不存在。"""
    core = _fed_core()
    core.on_mocap(_Mocap(stamp_s=100.0), at_s=0.0)
    assert core.cycle(0.0).publish
    core.on_mocap(_Mocap(position=(100.0, 0.0, 0.0), stamp_s=100.005), at_s=0.005)
    assert core.cycle(0.005).reason == "pose_jump"
    # 源仍然停在那个位置上 ⟹ **继续**拒（基准还是原点）
    core.on_mocap(_Mocap(position=(100.0, 0.0, 0.0), stamp_s=100.010), at_s=0.010)
    assert core.cycle(0.010).reason == "pose_jump"


def test_源回到基准附近就恢复而不是锁死():
    """★ 判据是"比较"，不是"闭锁"：一次性的跳变过去之后，流要能自己回来。

    （真实场景：刚体被短暂遮挡后回到原位；`VrpnSource` 的 `rewind` 窗口结束后同理。）
    """
    core = _fed_core()
    core.on_mocap(_Mocap(stamp_s=100.0), at_s=0.0)
    assert core.cycle(0.0).publish
    core.on_mocap(_Mocap(position=(100.0, 0.0, 0.0), stamp_s=100.005), at_s=0.005)
    assert not core.cycle(0.005).publish
    core.on_mocap(_Mocap(position=(0.0, 0.0, 0.0), stamp_s=100.010), at_s=0.010)
    assert core.cycle(0.010).publish, "源回来了却不放行 —— 判据把自己锁死了"


def test_动捕时间戳倒退会停_追平之后恢复():
    """`VrpnSource.rewind` / 上游重放：stamp 往回走 ⟹ 这一拍（以及倒退期间每一拍）都不发。"""
    core = _fed_core()
    core.on_mocap(_Mocap(stamp_s=100.0), at_s=0.0)
    assert core.cycle(0.0).publish

    core.on_mocap(_Mocap(stamp_s=95.0), at_s=0.1)          # 倒退 5 s
    c = core.cycle(0.1)
    assert c.reason == "stamp_regress" and not c.publish
    assert "5.0000" in c.detail, f"倒退量没进日志：{c.detail!r}"

    core.on_mocap(_Mocap(stamp_s=95.005), at_s=0.105)      # 还倒退着 ⟹ 继续停
    assert not core.cycle(0.105).publish

    core.on_mocap(_Mocap(stamp_s=100.1), at_s=0.11)        # 追平（向前跳回来）⟹ 放行
    assert core.cycle(0.11).publish, "stamp 追平之后不放行 —— 判据把自己锁死了"


def test_时间戳相等或小幅抖动不判倒退():
    """★ 容差不是保险余量，是**必需**：节点 100 Hz 取一拍而动捕 200 Hz ⟹ 同一个 stamp
    会被读到两次；DDS 的到达顺序也不保证单调。两者都不该被读成"源在重放"。"""
    core = _fed_core()
    core.on_mocap(_Mocap(stamp_s=100.0), at_s=0.0)
    assert core.cycle(0.0).publish
    core.on_mocap(_Mocap(stamp_s=100.0), at_s=0.005)                   # 完全相等
    assert core.cycle(0.005).publish
    core.on_mocap(_Mocap(stamp_s=100.0 - 0.005), at_s=0.010)           # 小于默认容差 0.01 s
    assert core.cycle(0.010).publish


def test_读不出_stamp_时判据不存在_而不是判成坏帧():
    """三种"读不出来"（无 `header` / 字段不齐 / 值为 0）都返回 `None` ⟹ 这两条判据不参与。

    ★ 取向：**不许因为源没填时间戳就把飞机停在空中** —— 那是"判据不存在"，不是"源不可信"。
    """
    core = _fed_core()
    core.on_mocap(_Mocap(), at_s=0.0)
    assert mocap_stamp_s(core.mocap) is None
    assert core.cycle(0.0).publish
    assert mocap_stamp_s(_Mocap(stamp_s=0.0)) is None, "stamp=0 是『源没填』，不是 1970 年"
    assert mocap_stamp_s(type("_M", (), {"header": type("_H", (), {"stamp": None})()})()) is None


def test_位姿判据的_dt_下限把频率与抖动挡在外面():
    """★ `JUMP_DT_FLOOR_S` 的作用：`允许 = 速率 × max(dt, 下限)`。

    dt 被 DDS 抖动压到 1 ms 时，若照 dt 算，允许位移只剩 0.02 m —— 一次正常机动就被误判。
    这里把同一个 0.15 m 的步进在两把不同的尺子下比一遍（都在下限之上/之下）。
    """
    core = _fed_core()
    core.on_mocap(_Mocap(stamp_s=100.0), at_s=0.0)
    assert core.cycle(0.0).publish
    assert JUMP_DT_FLOOR_S * StopConfig().mocap_jump_speed_max == pytest.approx(0.2)
    # 0.15 m：小于"下限给的 0.20 m" ⟹ 即便 stamp 只差 1 ms 也放行
    core.on_mocap(_Mocap(position=(0.15, 0.0, 0.0), stamp_s=100.001), at_s=0.020)
    assert core.cycle(0.020).publish, "dt 的下限没起作用（被投递抖动坑成假阳性）"


# ───────────────────────────── 整帧校验与"解可用吗"（ticket 07 / M5）─────────────────────

class _Info:
    """`SolveInfo` 的鸭子类型替身。

    ★ `ok` 与 `degraded` **照原样分开给**（`ocp.SolveInfo` 里就是两个独立的属性）：
      `ok` 只管状态码；`degraded` 还会因 `u` 越界等**数值**理由为真。判据只看前者 ——
      这正是"退化照发、状态码异常才兜底"这条分界线能在这层被钉死的原因。
    """

    def __init__(self, status: int = 0, ok: bool | None = None, degraded: bool | None = None,
                 line: str = ""):
        self.status = int(status)
        self._ok = (self.status == 0) if ok is None else bool(ok)
        self._degraded = (not self._ok) if degraded is None else bool(degraded)
        self._line = line or f"status={self.status}"

    @property
    def ok(self) -> bool:
        return self._ok

    @property
    def degraded(self) -> bool:
        return self._degraded

    def line(self) -> str:
        return self._line


class _Sol:
    def __init__(self, u=(7.3, 0.0, 0.0, 0.0), info=None):
        self.u = np.array(u, dtype=float)
        self.info = info if info is not None else _Info()


def test_整帧校验认得非有限值():
    """判据 2 的单元面：四个数里**任一个**非有限都算整帧不可发。

    ★★ 用**真帧类型**（`sign_map.Px4RatesSetpoint`）+ `dataclasses.replace` 造坏帧。
      本用例原先用 `type("_Sp", (), {...})()` 造替身 —— 那样四个字段全在**类**上，
      `vars(sp) == {}`，于是 `{**vars(sp), field: bad}` 造出的对象**只有那一个字段**，
      四次循环全部走 `frame_is_finite` 的 `except AttributeError` 分支，
      **`math.isfinite` 一次都没被调用过**（换成"只查 `sp.roll`"的假实现，本用例照样全绿）。
      换成真类型之后，"非有限"与"形状不对"这两条才真的是两件事。
    """
    sp = Px4RatesSetpoint(roll=0.0, pitch=0.0, yaw=0.0, thrust_body_z=-0.6235)
    assert frame_is_finite(sp)
    for field in ("roll", "pitch", "yaw", "thrust_body_z"):
        for bad in (float("nan"), float("inf"), float("-inf")):
            broken = dataclasses.replace(sp, **{field: bad})
            assert all(hasattr(broken, f) for f in ("roll", "pitch", "yaw", "thrust_body_z")), \
                "★ 四个字段必须都在 —— 缺字段是**另一条**判据（形状不对），会盖住这一条"
            assert not frame_is_finite(broken), f"{field}={bad} 没被整帧校验拦下"
    # 形状不对（少一个字段）⟹ 拒，不猜（这一条才是 AttributeError 那一支）
    assert not frame_is_finite(SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0))


def test_解可用吗_退化与不可用是两件事():
    """★ 这条是**兜底触发的分界线**，06 与 07 的交叉点，别合并：

    | 情形 | 判据 | 这一拍 |
    |---|---|---|
    | `ok=True`、`degraded=False` | 可用 | 发它的解 |
    | `ok=True`、`degraded=True`（如 `u` 越界 1e-3 N） | **仍然可用** | 照发 + `warn` + 计数（06 记录的现行行为） |
    | `ok=False`（状态码非零） | **不可用** | 走 `FallbackLadder` |
    | `u` 非有限 / 形状不对 | **不可用** | 走 `FallbackLadder`（判据 2） |
    """
    assert solution_usable(_Sol()) is None, "正常解必须是可用的"
    # 退化但状态码正常、解有限：**照发**（把它读成"失败"会让兜底在良性越界上乱触发）
    degraded_only = _Info(status=0, ok=True, degraded=True, line="u_viol=1.0e-3")
    assert solution_usable(_Sol(info=degraded_only)) is None, "只退化（如小量越界）不该走兜底"
    # 状态码非零 ⟹ 不可用，且原因里带上 acados 的那一行读数（日志里要能直接看到）
    reason = solution_usable(_Sol(info=_Info(status=2, line="status=2 qp_stat=[3, 0]")))
    assert reason and "status=2" in reason and "qp_stat" in reason, reason
    # 非有限的 u（本票的注入打的就是这一条）
    assert "非有限" in (solution_usable(_Sol(u=(float("nan"), 0.0, 0.0, 0.0))) or "")
    # 形状不对：[F, ωx, ωy] 也"能跑"（u[1:]/u[0] 都取得到）但发出去的东西是错的
    assert "形状" in (solution_usable(_Sol(u=(7.3, 0.0, 0.0))) or "")
    assert "形状" in (solution_usable(_Sol(u=(7.3, 0.0, 0.0, 0.0, 0.0))) or "")


# ───────────────────────────── 兜底阶梯（ticket 07 / M5）─────────────────────────────

def test_兜底阶梯先保持再悬停():
    """判据 1/3 的心跳：连续不可用 ≤ N 拍保持，第 N+1 拍起切安全悬停，**只上报一次**。"""
    ladder = FallbackLadder(hold_frames=3)
    for i in range(1, 4):
        phase, report = ladder.on_fail(f"第 {i} 次失败")
        assert phase == FallbackLadder.HOLD and report is None
    phase, report = ladder.on_fail("第 4 次失败")
    assert phase == FallbackLadder.HOVER
    assert report and "连续 4 拍" in report and "第 1 次" in report, f"上报语不对：{report!r}"
    # 继续失败：还是 hover，但**不再重复上报**（每拍重复会把日志淹掉）
    phase, report = ladder.on_fail("第 5 次失败")
    assert phase == FallbackLadder.HOVER and report is None
    assert (ladder.holds, ladder.hovers, ladder.events) == (3, 2, 1)


def test_兜底阶梯判据是连续不是累计():
    """★ 中间成功过一拍 ⟹ 计数清零（票面写的是"**连续**求解失败达阈值"）。

    这里累计失败 4 拍、阈值 2 拍：若判据写成"累计"，第 3 拍就该悬停了。
    """
    ladder = FallbackLadder(hold_frames=2)
    for _ in range(2):
        ladder.on_fail("x"); ladder.on_fail("x")
        assert ladder.phase == FallbackLadder.HOLD
        assert ladder.on_ok() is not None, "此前是 hold 档 ⟹ 恢复必须给出解除报告"
    assert (ladder.holds, ladder.hovers, ladder.events) == (4, 0, 0), \
        f"累计 4 拍失败被当成了『连续达阈值』：{ladder.describe()}"


def test_兜底阶梯解除时给出报告且计数清零():
    ladder = FallbackLadder(hold_frames=1)
    ladder.on_fail("求解器状态码异常")
    phase, report = ladder.on_fail("求解器状态码异常")
    assert phase == FallbackLadder.HOVER and report
    report = ladder.on_ok()
    assert report and "安全悬停" in report and "2 拍" in report, f"解除报告不对：{report!r}"
    assert ladder.streak == 0 and ladder.phase == FallbackLadder.NORMAL
    assert ladder.on_ok() is None, "本来就没在兜底 ⟹ 不该有报告"
    # 二次触发是**新的一次事件**（要能分别检索到）
    ladder.on_fail("x"); ladder.on_fail("x")
    assert ladder.events == 2


def test_兜底阶梯_hold_frames_为_0_就是第一次失败直接悬停():
    """参数化的边界：0 = 不保持（档位里 `hold` 一拍都不经过）。"""
    ladder = FallbackLadder(hold_frames=0)
    phase, report = ladder.on_fail("x")
    assert phase == FallbackLadder.HOVER and report and ladder.holds == 0


def test_兜底阶梯_没有可保持的帧时直接进悬停_且算一次事件():
    """★ 边界：**第一拍就不可用**（此前一直在热身或闸门关着）——`hold` 这一档不存在。

    这一条盯的是**簿记与实际发出的帧同源**：若阶梯还把档位判成 `hold`，就会出现
    "JSONL 打 `hover`、收尾摘要说'此刻仍在 hold'"以及"悬停帧被记成保持帧"（`holds` 虚高），
    而**这一次事件也不会被上报**（要等 streak 超过 `hold_frames`）—— 判据 3 要的正是它可检索。
    ⟹ 判据由**调用方**通过 `can_hold` 告诉阶梯（阶梯不认识帧，只有节点知道 `_last_sp` 在不在）。
    """
    ladder = FallbackLadder(hold_frames=10)          # 阈值给得很宽，好让"跳过 hold"这件事显形
    phase, report = ladder.on_fail("第一拍就没解", can_hold=False)
    assert (phase, ladder.phase) == (FallbackLadder.HOVER, FallbackLadder.HOVER), \
        "档位与实际发出的帧必须同源（不能返回 hover 而内部还停在 hold）"
    assert report and "安全悬停" in report and "第 1 次" in report, \
        "第一拍就进悬停，**这一次事件必须立刻上报**（判据 3）"
    assert "没有可保持的上一帧" in report, "上报里要说清为什么跳过了 hold 档"
    assert (ladder.holds, ladder.hovers, ladder.events) == (0, 1, 1), \
        f"悬停帧不许记成保持帧，实际 (holds, hovers, events)={ladder.holds, ladder.hovers, ladder.events}"
    assert ladder.streak == 1, "连续计数照记 —— '连续 N 拍没有可用解'这一点与能不能 hold 无关"
    # 对比：同样的输入，有可保持的帧时就该老老实实走 hold
    other = FallbackLadder(hold_frames=10)
    assert other.on_fail("有上一帧", can_hold=True) == (FallbackLadder.HOLD, None)
    assert (other.holds, other.hovers, other.events) == (1, 0, 0)


def test_闸门关闭时_reset_清档位但不清累计量():
    ladder = FallbackLadder(hold_frames=1)
    ladder.on_fail("x")                     # → 保持 1 拍
    ladder.on_fail("x")                     # → 安全悬停（第 1 次）
    assert (ladder.phase, ladder.hovers, ladder.events) == (FallbackLadder.HOVER, 1, 1)
    ladder.reset()                          # 闸门关了 ⟹ 档位毫无意义
    assert ladder.phase == FallbackLadder.NORMAL and ladder.streak == 0
    assert (ladder.holds, ladder.hovers, ladder.events) == (1, 1, 1), \
        "累计量（进收尾摘要的那三个）不许被清零 —— 那是这一趟飞行的病历"
    assert ladder.describe().startswith("兜底：保持 1 拍 / 安全悬停 1 拍 / 触发 1 次")


def test_默认阈值不是随手取的数():
    """★ 把"为什么是 10"钉在测试里 —— 要**换算成时间**才算钉住。

    （原先这里只断言 `>= 1`，把它改成 3 也全绿 ⟹ 那条断言其实什么都没钉。）
    100 Hz 一拍 10 ms（D6），两头都要卡住：

    · **下界**：给得起好几个求解预算 —— 实测最坏纯求解 3.58 ms（`offline-ocp.md` §5），
      比它短就变成"偶发一次慢拍就换姿态"；
    · **上界**：保持的是"上一帧**可行**指令"，越旧越不可信 ⟹ 保持时长的量级必须是 **0.1 s**
      而不是几秒。
    """
    tick_s = 1.0 / 100.0          # 100 Hz ⟹ 每拍 10 ms
    worst_solve_s = 3.58e-3       # 实测最坏纯求解（`offline-ocp.md` §5）
    held_s = FALLBACK_HOLD_FRAMES * tick_s
    assert held_s >= 20 * worst_solve_s, \
        f"阈值 {FALLBACK_HOLD_FRAMES} 拍 = {held_s * 1e3:.0f} ms 给不起 20 个求解预算"
    assert held_s <= 0.5, f"阈值 {held_s:.2f} s 太长 —— 保持的是一条越来越旧的指令"
    assert FALLBACK_HOLD_FRAMES >= 5, "太小 ⟹ `hold` 档名存实亡（一抖动就换姿态）"
    assert FALLBACK_HOLD_FRAMES == ControlConfig().fallback_hold_frames, \
        "`ControlConfig` 的默认值必须与常量同源（节点真正读的是它）"


def test_四元数测地角对符号不敏感():
    """`q` 与 `−q` 是同一个姿态 ⟹ 角度必须是 0，不是 2π（这条错了会把"没动"读成"翻了"）。"""
    q = np.array([0.6, -0.8, 0.0, 0.0])
    assert quat_angle(q, q) == pytest.approx(0.0, abs=1e-12)
    assert quat_angle(q, -q) == pytest.approx(0.0, abs=1e-12)
    assert quat_angle([1.0, 0, 0, 0], [0.0, 1.0, 0.0, 0.0]) == pytest.approx(math.pi)


# ───────────────────────────── 小工具 ─────────────────────────────

def _publishes_after_warmup(core: NodeCore, now_s: float) -> bool:
    """连推到闸门开放为止（默认 `WARMUP_CYCLES` 拍），返回最后一拍发不发。"""
    last = None
    for i in range(StopConfig().warmup_cycles + 1):
        last = core.cycle(now_s + 1e-6 * i)
    return bool(last.publish)
