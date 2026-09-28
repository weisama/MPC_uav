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

import math
import subprocess
import sys

import numpy as np
import pytest

from nmpc.model import NX, yaw_quat
from nmpc.node_core import (ARMING_STATE_ARMED, NAV_STATE_OFFBOARD, TAU_DEFAULT_S,
                            WARMUP_CYCLES, ControlConfig, NodeCore, StopConfig,
                            extrapolate_state, hold_reference, pose_to_state,
                            yaw_from_quat)
from nmpc.ocp import OcpConfig
from nmpc.params import DEFAULT as P

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


class _Mocap:
    def __init__(self, position=(0.0, 0.0, 0.0), orientation=(1.0, 0.0, 0.0, 0.0),
                 velocity=None):
        self.pose = _Pose(position, orientation)
        # `/mocap/pose_enu` 的 twist 是**可选**的（上游差分节点给了才有）——
        # 缺字段时节点必须退化成"不外推"，而不是崩掉。
        if velocity is not None:
            self.twist = type("_Twist", (), {"linear": _Vec(*velocity)})()


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


# ───────────────────────────── 小工具 ─────────────────────────────

def _publishes_after_warmup(core: NodeCore, now_s: float) -> bool:
    """连推到闸门开放为止（默认 `WARMUP_CYCLES` 拍），返回最后一拍发不发。"""
    last = None
    for i in range(StopConfig().warmup_cycles + 1):
        last = core.cycle(now_s + 1e-6 * i)
    return bool(last.publish)
