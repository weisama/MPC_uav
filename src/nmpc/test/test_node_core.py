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
from nmpc.node_core import (FALLBACK_HOLD_FRAMES, IGNORE_PX, IGNORE_PY,
                            IGNORE_PZ, IGNORE_VX, IGNORE_VY, IGNORE_VZ, IGNORE_YAW,
                            JUMP_DT_FLOOR_S, MODE_OFFBOARD, REFERENCE_SOURCE_CONTROL,
                            REFERENCE_SOURCE_TRAJ,
                            TAU_DEFAULT_S, WARMUP_CYCLES,
                            ControlConfig, FallbackLadder, NodeCore, StopConfig,
                            absolute_reference,
                            extrapolate_state, frame_is_finite, hold_feedforward,
                            hold_reference, mocap_stamp_s, pose_to_state, px4_pose_to_enu,
                            px4_quat_to_enu,
                            px4_stamp_s, px4_vector_to_enu, saturated_bounds, solution_usable,
                            traj_reference, traj_reference_fields, yaw_from_quat)
from nmpc.ocp import OcpConfig
from nmpc.params import DEFAULT as P
from nmpc.sign_map import MavrosAttitudeTarget

_OCP_CFG = OcpConfig()
N = _OCP_CFG.N
#: 求解器的离散步长 —— 与 `OfflineOcp.dt` **同一个口径**（`tf / N`，`ocp.py:423-425`）。
#: ★ 从配置**现算**、不写 `0.02`：参考外推用的就是它（`traj_reference`），
#:   而"外推的步长对不对"在话题面上完全静默（用成 `tau_s` 只会让斜率整体差 2.5 倍）。
#: ⚠️ 本文件的契约是"**不建**求解器"（`OcpConfig` 只是个 dataclass）⟹ 不去 `OfflineOcp` 上取那个属性
#:   （那会代码生成 + 编译 C）。
#: ☠️ **口径一致这件事只钉住了一半，说清楚**：本文件钉的是"`NodeCore` 用的步长**就是**调用方给的
#:   那个数，且那个参数**必填、无默认值**"（`test_时域步长是必填参数_口径不对必须当场红`）——
#:   于是"忘了传/传错成 `tau_s`"不可能静默。**跨模块那一半**（`mpc_node` 传的确实是
#:   `self.ocp.dt` 而不是别的数）靠的是**构造**：`mpc_node.py` 那一行只有一种写法，
#:   而 DUT 的启动日志把 OCP 的步长印成 ms（`内部步长 … ms`）⟹ 对不上时日志里看得见。
#:   ⟹ **不要**在这里写"由节点侧的用例钉" —— 那条用例并不存在，也不是靠断言钉的。
DT = _OCP_CFG.tf / _OCP_CFG.N


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


class _Velocity:
    """`/mocap/velocity_enu` 的形状（`geometry_msgs/TwistStamped`：`.twist.linear.{x,y,z}`）。

    ★ 只造被测代码会读到的字段（角速度刻意不造）—— 与 `_Mocap` 同一条纪律。ticket 15：
      速度**不与位姿同一条消息**，它是独立话题（所以这里是一个独立的替身，而不是 `_Mocap`）。
    """
    def __init__(self, vx=0.0, vy=0.0, vz=0.0):
        self.twist = type("_Twist", (), {"linear": _Vec(vx, vy, vz)})()


class _Status:
    """`mavros_msgs/State` 的鸭子类型替身 —— ☠️ **形状换过**（ticket 13 §二③）。

    旧替身抄的是 `px4_msgs/VehicleStatus` 的两个**整数**枚举
    （`nav_state=NAV_STATE_OFFBOARD`(14) / `arming_state=ARMING_STATE_ARMED`(2)）；
    现在抄的是 mavros 的一个 **bool**（`armed`）与一个**字符串**（`mode`，见 `MODE_OFFBOARD`）。
    ⛔ 别为了少改几行而把 `armed` 写成 `True`/`1` 的**整数**：`node_core` 那边是 `bool(...)`，
    整数 1 照样通过，于是"这个替身是不是真的像 mavros"就没人验了。
    """

    def __init__(self, armed=True, mode=MODE_OFFBOARD, pre_flight_checks_pass=True):
        self.armed = bool(armed)
        self.mode = str(mode)
        self.pre_flight_checks_pass = bool(pre_flight_checks_pass)


def _core(config: ControlConfig | None = None) -> NodeCore:
    return NodeCore(P, config or ControlConfig(), N, DT)


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


def test_参考是锚点位置加偏移_且姿态水平():
    """参考构造的三条：位置 = **锚点** + 偏移、速度为零、姿态是**水平 + 锚点机头**。

    ⚠️ 参数是**锚点**（ticket 09），不是当拍测量 —— "传测量进来"那种写法不报错、
      只让飞机追一个追不上的点，见 `hold_reference` 的 ★★ 与下面那一组锚点用例。
    """
    yaw = 0.7
    x_anchor = pose_to_state((1.0, -2.0, 3.0),
                             yaw_quat(yaw), (0.0, 0.0, 0.0))
    cfg = ControlConfig(hold_offset=(0.5, -0.25, 0.0))
    ref = hold_reference(x_anchor, cfg, N)
    assert ref.shape == (N + 1, NX)
    assert ref[0][:3].tolist() == pytest.approx([1.5, -2.25, 3.0])
    assert ref[0][3:6].tolist() == [0.0, 0.0, 0.0]
    assert yaw_from_quat(ref[0][6:10]) == pytest.approx(yaw, abs=1e-9)
    assert (ref == ref[0]).all(), "整条时域必须是同一个点（位置保持）"


def test_保持前馈是参考自己的平衡点_也就是解析前馈的定义():
    """★★ `hold_feedforward` = 静止参考的**解析前馈**：`f(x_ref, u_ref) = 0`。

    判据直接取定义：把 `u_ref` 喂给**模型**，参考点**不动**（RK4 走一步还是它自己）。
    这条比"推力 == 悬停油门"强 —— 后者只是一个数，前者说明这一对 `(x_ref, u_ref)`
    真的是模型的一条解，与姿态怎么取（本用例故意给一个非零机头）无关。

    ★ 为什么这条重要到要单独测：`u_ref` 在代价里是 `R·(u − u_ref)`，喂错的后果不是报错
      而是**控制目标整体偏掉** —— 2026-09-28 那场 runaway 就是喂了"上一帧解"
      （自我强化：解偏 ⟹ 下一帧参考量跟着偏），`offline-ocp.md` §4.4 与
      `hold_feedforward` 的 ★ 两段有第一手读数。
    """
    yaw = 0.4
    cfg = ControlConfig(hold_offset=(0.0, 0.0, 1.0), hold_yaw=yaw)
    x_ref = hold_reference(pose_to_state((1.0, 2.0, 3.0), yaw_quat(yaw)), cfg, N)
    u_ref = hold_feedforward(P, N)
    assert u_ref.shape == (N, 4)
    assert u_ref[0].tolist() == pytest.approx([P.t_hover, 0.0, 0.0, 0.0])
    assert (u_ref == u_ref[0]).all(), "解析前馈在时域上必须处处一样（参考是静止的）"
    # ☠️ 2026-09-29：这里**曾经**钉过 `hold_feedforward(P, N, 0.0) == u_ref` —— 那是为 ticket 20
    #   加进来的 `df` 形参写的。那条落点已被实测证伪、形参已删（见 `hold_feedforward` 的 ☠️）
    #   ⟹ 本用例回到"`f(x_ref, u_ref) = 0` **无条件**成立"，比"不传 == 传 0"强。

    from nmpc.model import Z0MultirotorRateModel
    model = Z0MultirotorRateModel(P)
    for x, u in ((x_ref[0], u_ref[0]), (x_ref[-1], u_ref[-1])):
        nxt = model.step_rk4(np.asarray(x, float), np.asarray(u, float), 0.02)
        assert nxt == pytest.approx(np.asarray(x, float), abs=1e-12), \
            "参考 + 解析前馈不是模型的平衡点 ⟹ 前馈算错了（飞机一定会被它推着走）"


# ☠️ 这里**曾经**有 `test_垂直偏置只进推力那一路_且偏离量正好是_df_over_m`：它钉的是
#   ticket 20 第一版把垂直积分偏置加进 `u_ref[:,0] = t_hover + df` 的那条落点
#   （当时"参考故意不再是模型的解"被写成了设计意图）。
#   2026-09-29 实测证伪：`u_ref` 是**代价参考**，`d u[0]/d u_ref[0] = −0.0616 N/N`（反号）
#   ⟹ 那条路是正反馈，可行域是空集（`mass+30` 从 −0.6406 恶化到 −0.7200 m）。
#   落点搬到**发出去的指令**上之后 `hold_feedforward` 的 `df` 形参整个删掉，
#   上面那条 `f(x_ref, u_ref) = 0` **无条件**成立 ⟹ 本用例没有存在的理由了。
#   ★ 新落点的判据在台架上，而且是**专门为它写的一条**
#   （`test_mpc_node.py::test_垂直偏置加在发出去的指令上_不在代价参考上`）：
#   那里能同时看到 `u_ref0`（不该含 ΔF）、`u`（求解器输出）、`thrust·t_max`（该含 ΔF）三列
#   （☠️ 第三列的名字在 ticket 13 §二② 换过：旧日志里叫 `−thrust_body_z·t_max`，
#    负号属于那个**字段名**，不是这个量本身），
#   而且那一场 `df` 从 0 长到几牛顿 ⟹ 断言有鉴别力（正控）；负控里 `df ≡ 0`，加不加都一样。
#   量法与完整读数：`tools/px4-sitl/probe_u_ref_authority.py` / ticket 20 §10。


def test_参考的姿态是水平的而不是照抄测量姿态():
    """★ 这条单独写，因为"照抄测量姿态"是一个**看起来更自然**的错法：
    飞机倾斜着悬停时，把倾斜姿态当参考 = 让 MPC 去维持那个倾斜。"""
    tilted = quat_tilted(roll=0.4, pitch=-0.2)
    ref = hold_reference(pose_to_state((0, 0, 0), tilted), ControlConfig(), N)
    q = ref[0][6:10]
    assert abs(float(q[1])) < 1e-9 and abs(float(q[2])) < 1e-9, "参考不该带横滚/俯仰"


def quat_tilted(roll: float, pitch: float) -> np.ndarray:
    """构造一个带横滚/俯仰的四元数（只用在本文件里，避免与 model 的约定纠缠）。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return np.array([cr * cp, sr * cp, cr * sp, -sr * sp])


# ───────────────────────── 位置保持的**锚点**（ticket 09）─────────────────────────
#
# 这一组盯的是 ticket 09 首飞暴露出来的那件事：参考若每拍用**当拍测量**重建，位置误差就
# 恒等于 `hold_offset`、**永远不收敛**。实测（z0 同构 SITL，`hold_offset_z = 1 m`，见
# `docs/control/tier-a-sitl-hover.md`）：飞机以 0.9 m/s 稳定爬升 37 s（EKF 帧抬升 30.9 m），
# 日志里 `x0[2] − x_ref0[2]` 从第 0 帧到第 3740 帧一次没关小，停在 `v·τ − 1 = −0.955 m`。
#
# 锚点 = 闸门第一次打开那一拍的测量（位置与机头分开记），之后**不随测量移动**。

def test_参考锚在闸门第一次打开那一拍_不跟着测量走():
    """★ 这是本组的主用例：位置误差必须真的**关得掉**（否则位置环等于没在工作）。

    ★ 时间刻意拉开到 0.3 s：位姿跳变闸门的上限是 `20 m/s × dt`，dt 太小时"飞机爬了 3 m"
      会先被判成 `pose_jump`，看到的就不是本用例要考的那件事（那种假红最费时间）。
    """
    cfg = ControlConfig(hold_offset=(0.0, 0.0, 1.0), stop=StopConfig(warmup_cycles=0))
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(position=(0.0, 0.0, 0.0)), at_s=0.0)
    c1 = core.cycle(0.01)
    assert c1.anchor == "anchor", "闸门第一次打开就把锚点钉住"
    assert c1.x_ref[0][:3].tolist() == pytest.approx([0.0, 0.0, 1.0])

    # 飞机爬到 3 m：参考**一动不动**（锚点 + 偏移），位置误差因此从 +1 m 变成 −2 m
    core.on_mocap(_Mocap(position=(0.0, 0.0, 3.0)), at_s=0.3)
    c2 = core.cycle(0.31)
    assert c2.reason == "ok" and c2.publish
    assert c2.x_ref[0][:3].tolist() == pytest.approx([0.0, 0.0, 1.0]), \
        "参考跟着测量走了 ⟹ 位置误差永远关不掉（首飞爬升 37 s 的机理）"
    assert (c2.x0[:3] - c2.x_ref[0][:3]).tolist() == pytest.approx([0.0, 0.0, 2.0]), \
        "参考在 1 m、飞机在 3 m ⟹ 误差该是 −2 m（节点要往下压）"
    assert c2.anchor == "", "只有钉/重钉的那一拍才该带事件"


def test_重开闸门不重钉锚点():
    """一次离闸（打杆接管 / 掉出 OFFBOARD）之后，目标该是**原来那个点** ——
    不是"在恢复的那一刻就近认一个新目标"（那等于把飞机飘到哪就当哪儿是家）。"""
    core = _core(ControlConfig(stop=StopConfig(warmup_cycles=0)))
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(position=(0.0, 0.0, 0.0)), at_s=0.0)
    assert core.cycle(0.01).anchor == "anchor"

    # 关闸：退出 OFFBOARD（这一拍不给初值/参考），期间飞机自己飘到 2 m 外
    # ☠️ 旧写法是 `_Status(nav_state=0)`（0 = px4_msgs 的 MANUAL）；现在这一档的名字是
    #    mavros 的 **字符串** —— 用 `"POSCTL"`（打杆接管的典型落点）而不是随手编一个词。
    core.on_status(_Status(mode="POSCTL"), at_s=0.1)
    core.on_mocap(_Mocap(position=(2.0, 0.0, 0.0)), at_s=0.1)
    c = core.cycle(0.11)
    assert c.reason == "not_offboard" and c.x_ref is None

    core.on_status(_Status(), at_s=0.4)
    core.on_mocap(_Mocap(position=(2.0, 0.0, 0.0)), at_s=0.4)
    c2 = core.cycle(0.41)
    assert c2.anchor == "", "重开闸门不重钉锚点（目标仍是原来那个点）"
    assert c2.x_ref[0][:3].tolist() == [0.0, 0.0, 0.0]


def test_px4_位置帧复位重钉位置目标_偏航复位只重钉机头():
    """★ 两个 reset 计数器各自对应一件事，**必须分开判**：`xy_reset_counter` 变了 ⟹ 位置帧
    换了（锚点里那个绝对点在新帧里已无意义，重钉到当前测量）；`quat_reset_counter` 变了 ⟹
    只是机头/姿态帧换了，**位置目标不该跟着搬**（混用一个 token 就会搬走）。
    """
    core = _core(_px4_config(warmup_cycles=0))
    core.on_status(_Status(), at_s=0.0)
    core.on_local_position(_LocalPosition(x=0.0, y=0.0, z=0.0, xy_reset_counter=0), at_s=0.0)
    core.on_attitude(_Attitude(quat_reset_counter=0), at_s=0.0)
    c0 = core.cycle(0.01)
    assert c0.anchor == "anchor" and c0.x_ref[0][:3].tolist() == [0.0, 0.0, 0.0]

    # 纯偏航复位（同一时刻的姿态消息带新的 quat_reset_counter）
    core.on_attitude(_Attitude(q=yaw_quat(0.0), quat_reset_counter=1), at_s=0.02)
    c1 = core.cycle(0.03)
    assert c1.anchor == "reanchor_quat", f"偏航复位该重钉机头，实测事件 {c1.anchor!r}"
    assert c1.x_ref[0][:3].tolist() == [0.0, 0.0, 0.0], "偏航复位把**位置**目标也搬走了"

    # 位置帧复位：NED (−5, 0, 0) ⟹ ENU (0, −5, 0)
    core.on_local_position(_LocalPosition(x=-5.0, y=0.0, z=0.0, xy_reset_counter=1), at_s=0.35)
    c2 = core.cycle(0.36)
    assert c2.anchor == "reanchor_xy", f"位置帧复位该重钉位置，实测事件 {c2.anchor!r}"
    assert c2.x_ref[0][:3].tolist() == pytest.approx([0.0, -5.0, 0.0]), \
        "重钉之后参考该落在**新的**测量上（帧换了，追旧点等于朝一个不存在的坐标飞）"


def test_锚点事件只在发生的那些拍出现():
    """逐帧日志里那列 `anchor` 的意义是"目标什么时候被搬过"—— 常态必须为空。
    ★ 没有这条，一个"每拍都写 `anchor`"的实现会让上面三条断言照样全绿。"""
    core = _core(ControlConfig(stop=StopConfig(warmup_cycles=0)))
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(), at_s=0.0)
    events = [core.cycle(0.01 * i).anchor for i in range(1, 6)]
    assert events == ["anchor", "", "", "", ""], f"事件列不对：{events}"


# ───────────────────────────── 闸门 ─────────────────────────────

def test_什么都还没收到时不发():
    core = _core()
    c = core.cycle(now_s=0.0)
    assert not c.publish and c.reason == "no_status"
    assert c.x0 is None and c.x_ref is None


def test_未解锁时不发_解锁后才发():
    """★ 票面验收项「未解锁 / 未在 offboard 时**不发布**」的一半。

    ★★ 负断言必须配正控（本仓的纪律）：所以这里在**同一个用例**里先把两个输入喂进去
       证明它**会**发，再把 `armed` 拨回去证明它**立刻**不发 ——
       只断言"没发"的话，"夹具/逻辑根本没工作"与"判据正确"长得一模一样。
    """
    core = _core()
    core.on_mocap(_Mocap(), at_s=0.0)
    core.on_status(_Status(mode=MODE_OFFBOARD, armed=False), at_s=0.0)
    assert core.cycle(0.1).reason == "not_armed"

    core.on_status(_Status(mode=MODE_OFFBOARD, armed=True), at_s=0.1)
    assert _publishes_after_warmup(core, 0.2)

    core.on_status(_Status(mode=MODE_OFFBOARD, armed=False), at_s=0.9)
    assert core.cycle(0.95).reason == "not_armed", "解锁掉了必须**当拍**就停"


def test_不在_offboard_时不发():
    """另一半：`mode` 一离开 OFFBOARD 就停（打杆接管 / ch6 切模式在节点侧的唯一表现）。

    ☠️ **判据的写法本身换过一次**（ticket 13 §二③）：旧版比的是整数
    （`nav_state == 14`），现在比的是 mavros 的**字符串** `"OFFBOARD"`。⛔ 所以这里
    **故意不做**"字符串包含 / 前缀匹配"（`mode.startswith("OFF")`）：mavros 那张表
    （`uas_stringify.cpp:138`）里的名字是定死的，松匹配会把将来某张表里的
    `"OFFBOARD_xxx"` 也算通过 —— 那正是"看起来闸门开着"的静默。
    """
    core = _core()
    core.on_mocap(_Mocap(), at_s=0.0)
    core.on_status(_Status(mode="POSCTL"), at_s=0.0)
    assert core.cycle(0.1).reason == "not_offboard"
    core.on_status(_Status(mode=MODE_OFFBOARD), at_s=0.1)
    assert _publishes_after_warmup(core, 0.2)


def test_停机条件可以单独关掉():
    """参数化的一层含义：每一条闸门都能单独关（票面「软急停接口参数化」）。"""
    core = _core(ControlConfig(stop=StopConfig(stop_on_nav_state_exit=False,
                                               stop_on_armed_drop=False)))
    core.on_mocap(_Mocap(), at_s=0.0)
    core.on_status(_Status(mode="POSCTL", armed=False), at_s=0.0)
    assert _publishes_after_warmup(core, 0.1)


def test_动捕静默超时会停_恢复后重新热():
    """`mocap_stale` 与"恢复后要重新热身"是同一个用例的两半。"""
    cfg = ControlConfig(stop=StopConfig(mocap_timeout_s=0.5, warmup_cycles=2))
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(), at_s=0.0)
    assert [core.cycle(0.1).reason, core.cycle(0.2).reason] == ["warmup", "warmup"]
    assert core.cycle(0.3).publish
    # ★ 状态那条也得保持新鲜，否则先撞上的是 `status_stale`（2.5 s 的阈值比动捕的 0.5 s 长，
    #   但两者的计时起点不同 ⟹ 不显式给状态，这里测的就不是动捕那条闸门）。
    #   ☠️ 这个数换过载体（0.75 → 2.5，见 `StopConfig.status_timeout_s`）：取值本身不影响
    #   本用例（这里用的是默认值、且判的是动捕那条闸门），但留一个**旧数**在注释里就是下一个
    #   人算错阈值的起点。
    core.on_status(_Status(), at_s=0.85)
    assert core.cycle(0.9).reason == "mocap_stale"          # 0.9 − 0.0 = 0.9 > 0.5
    core.on_mocap(_Mocap(), at_s=0.95)
    assert core.cycle(1.0).reason == "warmup", "断开之后必须重新热，而不是立刻续上"


def test_状态静默超时会停():
    """机制本身：状态那条**只要越过阈值就当拍停**。

    ☠️ 这里**显式**喂 `status_timeout_s=0.75`（而不是取默认值）⟹ 它测的是机制、不是生产阈值；
    生产阈值由下面 `test_状态源一_hz_阈值必须给够余量` 单独钉住。
    ⚠️ 顺带记一笔：0.75 这个数在换载体前**就是生产值**（旧闸门源 `/fmu/out/vehicle_status`
    是 2 Hz ⟹ 1.5 个周期）；现在留在用例里只是"一个比动捕那条更长的阈值"的角色。
    """
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


#: **闸门源的周期**（`/mavros/state`，1 Hz）—— ★ 与 `status_timeout_s` 是**两个数**，
#: 而它们之间那条关系（阈值 > 周期）正是这个参数唯一的物理含义。台架实测见
#: `docs/control/mpc-node-seam1.md` §11.2 与 Phase 1 的 `control.yaml:120-125`（1.000 Hz）。
STATE_PERIOD_S = 1.0


def test_状态源一_hz_阈值必须给够余量():
    """★★ **判据：闸门源的阈值必须严格大于它的周期**（ticket 13 §二③ 要求"必须有断言护住"）。

    机理：`status_timeout_s ≤ 周期` ⟹ `age_status_s` 每拍都超 ⟹ 判据**恒真** ⟹ 闸门**永远关着、
    一帧都不发**。而它的症状是"安静"：没有异常、没有报错，只是什么都不发 ——
    与"话题名写错"、"节点没起来"在现场完全分不开。
    ☠️ 换载体那一版就是踩在这上面：阈值 `0.75` 是给 **2 Hz** 的
    `/fmu/out/vehicle_status` 定的，换到 **1 Hz** 的 `/mavros/state` 之后
    `0.75 < 1.0` ⟹ 恒真。票面写的是"必须改大，推荐 2.5 s"。

    ★★ **变异测试**（本仓纪律，V 字要在自己身上验一次）：把 `StopConfig.status_timeout_s`
    的默认值改回 `0.75` ⟹ **本用例必须变红**。改完记得改回去。
    ⛔ 别把这条写成 `== 2.5`：那只是**钉住一个数**，"为什么是它"没人看得出来；
    写成"比周期大、且有余量"才是这个参数**能成立**的条件，换频率时它会跟着说话。
    """
    default = StopConfig().status_timeout_s
    assert default > STATE_PERIOD_S, (
        f"status_timeout_s={default} ≤ 闸门源周期 {STATE_PERIOD_S} s ⟹ 超时判据恒真、"
        "闸门一直关（症状是「什么都不发」，不是报错）")
    # ★ 余量：至少 2 个周期。取 1.5 个周期（1.5 s）也能用，但"最多晚 1.5 s 才发现丢状态"
    #   在起飞/切换模式那一瞬间是会被误读成"飞控不理我"的；2.5 s 与 Phase 1 的
    #   `state_timeout_s` 同值同理由（`src/control/config/control.yaml:120-125`）。
    assert default >= 2 * STATE_PERIOD_S, (
        f"status_timeout_s={default} 只有 {(default / STATE_PERIOD_S):.2f} 个周期："
        "抖动一下就会误判丢状态，而这条闸门一关就是停发")


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


def test_动捕速度从独立话题进外推():
    """ticket 15：速度走**独立话题** `/mocap/velocity_enu`（`TwistStamped`），真的进了 `x + v·τ`。

    ★ 与上面那条是一对正/负控，**缺一不可**：上面守着"**没有**速度时不崩、不瞎猜"，
      这条守着"**有**速度时它真的进了状态与那"一行外推"。⚠️ 只有上面那条时，速度这条路
      直到 ticket 15 之前都是**死的**（`PoseStamped` 没有 `twist` 字段 ⟹ 缺字段那条路每拍
      都在走），而它**照样绿** —— 这正是"没有速度"与"速度没接上"分不开的那个假绿。
    """
    cfg = ControlConfig(tau_s=0.05, stop=StopConfig(warmup_cycles=0))
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(), at_s=0.0)                        # 位姿（无 twist 字段）
    core.on_mocap_velocity(_Velocity(vx=1.0, vy=2.0), at_s=0.0)
    c = core.cycle(0.1)
    assert c.publish, f"这一拍该发（warmup_cycles=0），却被 {c.reason} 拦下"
    assert c.x0[3:6].tolist() == pytest.approx([1.0, 2.0, 0.0]), "速度没进 10 维状态"
    assert c.x0[:3].tolist() == pytest.approx([0.05, 0.1, 0.0]), "`x + v·τ` 没把速度算进去"
    # 取证面要能回答"速度到了几帧、多旧"（ticket 15 给 mocap 档加的两个键）
    d = core.describe_state(0.1)
    assert d["vel_cb"] == 1 and d["age_vel_s"] == pytest.approx(0.1)


def test_独立速度话题优先于位姿里内嵌的_twist():
    """两条来源同时在时，取**独立话题**那条 —— 那才是真身（ticket 15 的裁定）。"""
    cfg = ControlConfig(tau_s=0.0, stop=StopConfig(warmup_cycles=0))
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(velocity=(9.0, 9.0, 9.0)), at_s=0.0)   # 位姿里嵌了一个"错"速度
    core.on_mocap_velocity(_Velocity(vx=1.0), at_s=0.0)
    c = core.cycle(0.1)
    assert c.x0[3:6].tolist() == [1.0, 0.0, 0.0], \
        "取到了位姿里内嵌的 twist，而不是独立话题那条（来源优先级反了）"


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

    ★★ 用**真帧类型**（`sign_map.MavrosAttitudeTarget`）+ `dataclasses.replace` 造坏帧。
      本用例原先用 `type("_Sp", (), {...})()` 造替身 —— 那样四个字段全在**类**上，
      `vars(sp) == {}`，于是 `{**vars(sp), field: bad}` 造出的对象**只有那一个字段**，
      四次循环全部走 `frame_is_finite` 的 `except AttributeError` 分支，
      **`math.isfinite` 一次都没被调用过**（换成"只查 `sp.roll`"的假实现，本用例照样全绿）。
      换成真类型之后，"非有限"与"形状不对"这两条才真的是两件事。

    ☠️ **真帧类型换过**（ticket 13 §二②）：以前是这个模块的 `Px4RatesSetpoint`
    （字段 `roll/pitch/yaw/thrust_body_z`），现在是 mavros 路的 `MavrosAttitudeTarget`
    （`body_rate_x/y/z` + `thrust`）—— `frame_is_finite` 查的就是这四个新名字，
    拿旧类型造帧会**一路走 `AttributeError` 分支**（四个字段名全对不上），
    于是"非有限被拦下"这条断言变成**永远为真**（因为什么都拦）。这正是本用例开头那段
    ★★ 描述过的同一个坑，只是换了个入口。
    """
    sp = MavrosAttitudeTarget(body_rate_x=0.0, body_rate_y=0.0, body_rate_z=0.0, thrust=0.6235)
    assert frame_is_finite(sp)
    for field in ("body_rate_x", "body_rate_y", "body_rate_z", "thrust"):
        for bad in (float("nan"), float("inf"), float("-inf")):
            broken = dataclasses.replace(sp, **{field: bad})
            assert all(hasattr(broken, f)
                       for f in ("body_rate_x", "body_rate_y", "body_rate_z", "thrust")), \
                "★ 四个字段必须都在 —— 缺字段是**另一条**判据（形状不对），会盖住这一条"
            assert not frame_is_finite(broken), f"{field}={bad} 没被整帧校验拦下"
    # 形状不对（少一个字段）⟹ 拒，不猜（这一条才是 AttributeError 那一支）
    assert not frame_is_finite(
        SimpleNamespace(body_rate_x=0.0, body_rate_y=0.0, body_rate_z=0.0))


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


# ─────────────────── PX4 作为状态源（ticket 09 / A 档闭环）───────────────────
#
# ★ 这一组的理由（票面原文）："这是**我们的约定与真 PX4 一致**这个问题第一次被真正回答"。
#   04 只能验自洽性、06 只能验"映射表被正确实现了" —— 状态**从 PX4 那头进来**之后，
#   整条链（NED/FRD → ENU/FLU → 模型 → 符号映射 → 话题）才第一次闭合。
#   ⚠️ 转换那两条用**旋转矩阵恒等式**对账，不用手推的分量式子：手推错一个号，
#   症状是"飞机一进 OFFBOARD 就朝一边歪"，而符号表看着完全正确。

def _rot(q) -> np.ndarray:
    """四元数（w-first）→ 旋转矩阵。**本文件自己写一份**（不复用 `model.q_to_rot_mat`：
    那个是 CasADi 的符号表达式）—— 两份独立实现对账才有意义。"""
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


#: 两个"换标签"的定义（**照抄各自的 msg 与坐标系定义**，不照抄实现的写法）：
#:   · `L`：世界系 NED→ENU。NED 的 north=+x、east=+y、down=+z；ENU 的 east=+x、north=+y、up=+z
#:     ⟹ 把 NED 坐标读成 ENU 坐标就是 `(x, y, z) → (y, x, −z)`。
#:   · `M`：机体系 FRD→FLU。FRD 的 y 向右、z 向下；FLU 的 y 向左、z 向上 ⟹ `(x, y, z) → (x, −y, −z)`。
#:     ★ `sign_map.px4_rates_setpoint`（uXRCE-DDS 那条路）对**角速率**做的就是 `M`
#:       （`roll=ωx, pitch=−ωy, yaw=−ωz`）—— 同一个换标签。
#:       ⚠️ 换到 mavros 之后**发出去的那条路不再做 `M`**（`mavros_attitude_target` 一个字都不转，
#:       取负由 mavros 自己做）：本文件下面那两条对账用例验的是 `M` 这个**换标签的数学**本身
#:       （以及 `px4_pose_to_enu` 那条读入路径），与"我们发出去时转不转"是两件事。
L_NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
M_FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


class _LocalPosition:
    """`px4_msgs/VehicleLocalPosition` 的替身（**只带节点真读的那些字段**，与夹具同一条纪律）。"""

    def __init__(self, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, vz=0.0, timestamp_us=0,
                 xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
                 dead_reckoning=False, eph=0.0, epv=0.0, xy_reset_counter=0):
        self.x, self.y, self.z = float(x), float(y), float(z)
        self.vx, self.vy, self.vz = float(vx), float(vy), float(vz)
        self.timestamp = int(timestamp_us)
        self.xy_valid, self.z_valid = bool(xy_valid), bool(z_valid)
        self.v_xy_valid, self.v_z_valid = bool(v_xy_valid), bool(v_z_valid)
        self.dead_reckoning = bool(dead_reckoning)
        self.eph, self.epv = float(eph), float(epv)
        self.xy_reset_counter = int(xy_reset_counter)


class _Attitude:
    """`px4_msgs/VehicleAttitude` 的替身 —— `q` 是「FRD 机体 → NED 世界」（msg 里逐字写着）。"""

    def __init__(self, q=(1.0, 0.0, 0.0, 0.0), timestamp_us=0, quat_reset_counter=0):
        self.q = [float(v) for v in q]
        self.timestamp = int(timestamp_us)
        self.quat_reset_counter = int(quat_reset_counter)


#: `ControlConfig` 自己的字段名（`state_source` 与 `stop` 由 `_px4_config` 显式给）
_CC_FIELDS = {f.name for f in dataclasses.fields(ControlConfig)} - {"state_source", "stop"}


def _px4_config(**kw) -> ControlConfig:
    """`state_source=px4` 的一档配置。

    关键字按**字段名**分派：`ControlConfig` 有的给它（`tau_s`…）、其余给 `StopConfig`
    （`px4_timeout_s`、`warmup_cycles`…）—— 测试里只写关心的那几个，不必记住谁属于谁。
    """
    cc = {k: v for k, v in kw.items() if k in _CC_FIELDS}
    stop = {k: v for k, v in kw.items() if k not in _CC_FIELDS}
    return ControlConfig(state_source="px4", stop=StopConfig(**stop), **cc)


def test_px4_姿态转换满足旋转矩阵恒等式():
    """`R(q_enu_flu) == L · R(q_ned_frd) · M` —— **对任意姿态**都要成立。

    ★ 这条恒等式就是"ENU/FLU 约定与 PX4 的 NED/FRD 约定一致"的**完整定义**：
      把它写成断言之后，"某个分量少了一个负号"这类错**当场**就红，而且与符号无关
      （`q` 与 `−q` 是同一姿态，所以恒等式只谈旋转矩阵）。
    """
    rng = np.random.default_rng(20260928)
    quats = [np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0, 0.0])]
    for _ in range(20):
        q = rng.normal(size=4)
        quats.append(q / np.linalg.norm(q))
    for q in quats:
        q_enu = px4_quat_to_enu(q)
        assert np.dot(q_enu, q_enu) == pytest.approx(1.0, abs=1e-12), "转换后不再是单位四元数"
        assert _rot(q_enu) == pytest.approx(L_NED_TO_ENU @ _rot(q) @ M_FRD_TO_FLU, abs=1e-12), \
            f"q={q} 的转换不满足 R(q_enu) = L·R(q_ned)·M"


def test_px4_姿态转换与参考实现的手写公式一致_差一个整体符号():
    """与**参考实现的手写分量式**逐分量对账 —— 这是本仓对"外部来源"的既定做法
    （[`sign-map.md`](../../docs/control/sign-map.md) §4 就是拿参考实现逐行对的）。

    参考实现 `mpc_quadrotor.py` 的写法（引用见
    [`phase2-implementation.md`](../../docs/control/phase2-implementation.md) §2.3(a)）：

        q_enu = 1/√2 · [q0+q3, q1+q2, q1−q2, q0−q3]      # (w, x, y, z)

    ★ 两边**只差一个整体符号**：`q` 与 `−q` 是同一个旋转（双重覆盖），所以这不构成分歧，
      但它正好把"这个符号从哪来"钉在纸面上 —— 手写式的取号与"两个 180° 旋转复合"的取号
      不是同一套，而**旋转矩阵那一侧必须完全相等**（上面那条恒等式管的正是这件事）。
    """
    def ref(q):                      # 参考实现的手写式（照抄，只补归一化常数）
        return np.array([q[0] + q[3], q[1] + q[2], q[1] - q[2], q[0] - q[3]]) / math.sqrt(2)

    rng = np.random.default_rng(20260928)
    for _ in range(200):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        ours = px4_quat_to_enu(q)
        r = ref(q)
        same = min(np.abs(ours - r).max(), np.abs(ours + r).max())
        assert same < 1e-12, f"q={q}：我们的转换与参考实现的手写式不是同一个旋转"
        # 而"同一个旋转"这件事由矩阵恒等式独立确认（符号不影响它）
        assert _rot(ours) == pytest.approx(_rot(r), abs=1e-12)


def test_px4_位置与速度的_NED_到_ENU():
    """位置与速度**同一套**换标签（x/y 互换、z 取反），速度不许走另一条路。"""
    assert px4_vector_to_enu((1.0, 2.0, 3.0)) == pytest.approx((2.0, 1.0, -3.0))
    # 物理锚点：NED 的 (1,0,0) 是"北"，在 ENU 里是 +y
    assert px4_vector_to_enu((1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))
    # 反例（"每个分量取反"那种写法）：它会给出 (−1,0,0)，方向完全不对
    assert px4_vector_to_enu((1.0, 0.0, 0.0))[1] > 0


def test_px4_姿态转换在两个物理特例上对得上():
    """两个可以**用一句话说清**的特例 —— 手推错了这两条必红。

    ① NED/FRD 下的**单位姿态**（`q=(1,0,0,0)`）= 机头朝北、水平。ENU 里"北"是 +y
       ⟹ 换过去必须是**偏航 +90°**；
    ② NED/FRD 下**偏航 +90°**（绕 NED 的 +z，即朝下）⟹ 机头朝东。ENU 里"东"是 +x
       ⟹ 换过去必须是**单位姿态**。
    """
    q_enu_unit = px4_quat_to_enu(np.array([1.0, 0.0, 0.0, 0.0]))
    nose = _rot(q_enu_unit) @ np.array([1.0, 0.0, 0.0])
    assert nose == pytest.approx([0.0, 1.0, 0.0], abs=1e-12), "单位姿态换到 ENU 之后机头该朝北（+y）"
    assert yaw_from_quat(q_enu_unit) == pytest.approx(math.pi / 2, abs=1e-12)

    q_ned_yaw90 = yaw_quat(math.pi / 2)          # NED 的 yaw 与 ENU 的 yaw 差一个号，但这里
    q_enu = px4_quat_to_enu(q_ned_yaw90)         # 只需要"机头朝东"这一个事实
    assert _rot(q_enu) @ np.array([1.0, 0.0, 0.0]) == pytest.approx([1.0, 0.0, 0.0], abs=1e-12), \
        "NED 下偏航 +90°（朝东）换到 ENU 之后该是单位姿态"


def test_px4_源_没收到状态时不发():
    """闸门：`state_source=px4` 时**动捕完全不参与** —— 状态没到之前一帧都不发。"""
    core = _core(_px4_config())
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(), at_s=0.0)          # ★ 送进来也不该被用
    assert core.cycle(0.1).reason == "no_px4_state"


def test_px4_源_只看_px4_的龄_而不是动捕的():
    """`px4_timeout_s` 到点就停；★ 同时证明动捕那条路在这档里**根本没被判据读**：
    动捕一直在送，闸门照样关。"""
    cfg = _px4_config(px4_timeout_s=0.5, mocap_timeout_s=0.1, warmup_cycles=0)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_local_position(_LocalPosition(), at_s=0.0)
    core.on_attitude(_Attitude(), at_s=0.0)
    for i in range(20):                        # 动捕一直在送（且"新鲜"）
        core.on_mocap(_Mocap(), at_s=0.0 + 0.01 * i)
    assert core.cycle(0.1).publish, "px4 的状态是新鲜的，闸门该开"
    core.on_status(_Status(), at_s=0.6)        # 状态那条也得新鲜（否则先撞 status_stale）
    assert core.cycle(0.6).reason == "px4_stale", "px4 状态超时了还在发"


def test_px4_源_姿态单独超时也要停():
    """位置与姿态是**两条消息**（32 Hz 各自到）⟹ 两条各自的龄都要判。

    ★ 只判位置那一条的后果：姿态停在几秒前，节点却以为自己在闭环 —— 四元数过期在
      话题上**看不出来**（姿态变化慢，两个选择只差一个小的倾角）。所以这一条单独钉。
    """
    cfg = _px4_config(px4_timeout_s=0.5, warmup_cycles=1)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_local_position(_LocalPosition(), at_s=0.9)      # 位置很新
    core.on_attitude(_Attitude(), at_s=0.0)                 # 姿态是旧的
    core.on_status(_Status(), at_s=0.9)
    assert core.cycle(0.9).reason == "px4_attitude_stale"


def test_px4_源_坏四元数不发():
    """姿态非有限 / 近零 ⟹ 当拍不发（理由与动捕那条一致：NaN 进了求解器就是整条轨迹烂掉）。"""
    core = _core(_px4_config())
    core.on_status(_Status(), at_s=0.0)
    core.on_local_position(_LocalPosition(), at_s=0.0)
    core.on_attitude(_Attitude(q=(0.0, 0.0, 0.0, 0.0)), at_s=0.0)
    assert core.cycle(0.1).reason == "bad_px4_quat"

    core.on_attitude(_Attitude(q=(float("nan"), 0.0, 0.0, 1.0)), at_s=0.1)
    assert core.cycle(0.2).reason == "bad_px4_quat"


def test_px4_源_速度真的进了外推():
    """★ PX4 这条源**有速度**（`vx/vy/vz`）—— 而动捕那条没有（issue 15）。

    ⟹ 外推 `x + v·τ` 在这一档里第一次**真的在动**：喂一个非零速度，`x0[:3]` 必须
      等于 `p + v·τ` 而不是 `p`（`tau_s=0` 时两者相等，这条也是那个参数的负控）。
    """
    cfg = _px4_config(tau_s=0.05, warmup_cycles=0)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    # NED 速度 (vx=1, vy=0, vz=0) = 向北 1 m/s ⟹ ENU 速度 (0, 1, 0)
    core.on_local_position(_LocalPosition(x=0.0, y=0.0, z=0.0, vx=1.0, vy=0.0, vz=0.0), at_s=0.0)
    core.on_attitude(_Attitude(), at_s=0.0)
    c = core.cycle(0.01)
    assert c.x0[:3] == pytest.approx([0.0, 0.05, 0.0], abs=1e-12), \
        f"外推没把速度算进去：{c.x0[:3]}"
    assert c.x0[3:6] == pytest.approx([0.0, 1.0, 0.0], abs=1e-12), "ENU 速度该是 (0, 1, 0)"

    # 负控：同一个输入、`τ = 0` ⟹ 外推退化成恒等（这就是 D9 那一档 `-p tau_s:=0`）
    zero = _core(_px4_config(tau_s=0.0, warmup_cycles=0))
    zero.on_status(_Status(), at_s=0.0)
    zero.on_local_position(_LocalPosition(vx=1.0), at_s=0.0)
    zero.on_attitude(_Attitude(), at_s=0.0)
    z = zero.cycle(0.01)
    assert z.x0[:3] == pytest.approx([0.0, 0.0, 0.0], abs=1e-12)
    assert z.x0[3:6] == pytest.approx([0.0, 1.0, 0.0], abs=1e-12), \
        "速度本身照样进状态，只是不参与外推"


def test_px4_源_位姿跳变照样被拦():
    """输入合理性那两条判据**与源无关**（换成 PX4 之后仍然要在）：EKF 复位/认错给出的瞬移
    同样只关闸门、不发错指令。"""
    core = _core(_px4_config(warmup_cycles=0))
    core.on_status(_Status(), at_s=0.0)
    core.on_local_position(_LocalPosition(x=0.0, y=0.0, z=0.0), at_s=0.0)
    core.on_attitude(_Attitude(), at_s=0.0)
    assert core.cycle(0.01).publish

    core.on_local_position(_LocalPosition(x=100.0, y=0.0, z=0.0), at_s=0.02)
    core.on_attitude(_Attitude(), at_s=0.02)
    c = core.cycle(0.03)
    assert c.reason == "pose_jump" and not c.publish
    assert "位移" in c.detail, "跳变判据的读数要进日志（调阈值的人第一个要问它）"


def test_px4_源_状态龄与回调计数进取证():
    """`describe_state()` 是**取证面**：日志里要能回答"这一拍的状态有多旧、PX4 自己怎么说"。

    ★ 为什么连 `xy_valid` / `dead_reckoning` 这些标志也留下来：第一手（2026-09-28，
      z0 SITL 地面静息、真值恒为 0）：EKF 报 x≈−0.45 m、z 在 −0.18…−0.64 m 之间游走、
      速度到 0.58 m/s，而**这四个标志全是 1、`dead_reckoning=0`、`eph/epv≈0.33 m`**
      ⟹ 它们**不是**可用性判据（拿它当闸门 = 一个恒假的保证），但它们是复盘时
      "这个数到底是谁给的"那一条线索。所以：**记录，但不闸**。
    """
    core = _core(_px4_config())
    core.on_status(_Status(), at_s=0.0)
    core.on_local_position(_LocalPosition(x=-0.45, y=0.1, z=-0.5, eph=0.33, epv=0.34),
                           at_s=0.0)
    core.on_attitude(_Attitude(), at_s=0.02)
    d = core.describe_state(0.5)
    assert d["state_source"] == "px4"
    assert d["age_pos_s"] == pytest.approx(0.5) and d["age_att_s"] == pytest.approx(0.48)
    assert d["pos_cb"] == 1 and d["att_cb"] == 1
    assert d["ekf_xy_valid"] == 1 and d["ekf_dead_reckoning"] == 0
    assert d["ekf_eph"] == pytest.approx(0.33)

    mocap_core = _core()
    mocap_core.on_status(_Status(), at_s=0.0)
    mocap_core.on_mocap(_Mocap(), at_s=0.0)
    d2 = mocap_core.describe_state(0.25)
    assert d2["state_source"] == "mocap" and d2["age_state_s"] == pytest.approx(0.25)


def test_饱和标志认得出顶在约束上_且不把擦边当饱和():
    """「约束饱和标志」是票面要的日志项 —— 判据必须**用生产阈值**（与 OCP 的 `lbu/ubu` 同源），
    且要有容差。

    ★ 容差不是洁癖：两条第一手教训叠在一起 ——
      ① 求解器给出的"打满"是 `3.8399998` 而不是 `3.84`（QP 解从来不是整数）；
      ② 本仓已经出过一次"`== 0.0` 假绿"（见 `test_ocp_*.py` 的容差纪律）
      ⟹ 用等号判饱和会**漏报**，而漏报的标志比没有标志更坏（复盘时会以为"没打满"）。
    """
    p = P
    # 顶在四条约束上（推力上限 + 三轴角速率上限，符号按 lbu/ubu）
    sat = saturated_bounds([p.f_max, -p.roll_rate_max, p.pitch_rate_max, -p.yaw_rate_max], p)
    assert set(sat) == {"F_max", "wx", "wy", "wz"}, sat
    # ★ `F_max` 认的是 **`f_max`（按电池 11.8 N）**，不是归一化常数 `t_max`（46.3 N）——
    #   接错的话"顶到电池极限"这一档**永远不报**（假绿），而 46.3 N 那一档才是真到不了
    assert "F_max" not in saturated_bounds([p.t_max, 0.0, 0.0, 0.0], p), \
        "`F_max` 判据接成了归一化常数 `t_max`"
    # 擦边（差 1‰ 以上）不算饱和
    assert saturated_bounds([0.99 * p.f_max, 0.9, 0.9, 0.9], p) == ()
    # 零推力 = 顶在下界（FLU 下推力非负，下界就是 0）
    assert saturated_bounds([0.0, 0.0, 0.0, 0.0], p) == ("F_min",)


# ─────────────────── 轨迹参考 `/traj/reference`（ticket 17）───────────────────

class _TrajRef:
    """`mavros_msgs/PositionTarget` 的形状（只造被测代码会读的那几个字段：位置/速度/yaw/掩码）。"""
    def __init__(self, position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0),
                 yaw=0.0, mask=2496):
        self.position = _Vec(*position)
        self.velocity = _Vec(*velocity)
        self.yaw = float(yaw)
        self.type_mask = int(mask)


def _traj_config(**kw) -> ControlConfig:
    """`traj` 档的配置（与 `hold` 档**只差这一个字段**，所以其它字段照旧用默认值）。"""
    kw.setdefault("warmup_cycles", 0)
    stop = StopConfig(**kw)
    return ControlConfig(reference_source=REFERENCE_SOURCE_TRAJ, stop=stop)


def _control_config(**kw) -> ControlConfig:
    """`control` 档的配置（票 15）—— 与 `traj` 档只差参考源。"""
    kw.setdefault("warmup_cycles", 0)
    stop = StopConfig(**kw)
    return ControlConfig(reference_source=REFERENCE_SOURCE_CONTROL, stop=stop)


def test_掩码常量的低位与_位布局对得上():
    """★★ 这一条钉的是**掩码常量的数值本身**（不是"代码读了掩码"）。

    为什么值得单独一条：这几个数是**协议**（`mavros_msgs/PositionTarget` 的 `IGNORE_*`），
    而它们在我们这边是**手抄**的（`node_core` 不许 import ROS）。抄错一位的症状是
    "某一轴参考恒为 0"或"某一轴参考照着一个被忽略的字段走" —— 两者在话题面上都像正常飞行。
    ⚠️ 与真消息类的**交叉核对**在 `test_mpc_node.py`（那一侧 import 得动 `mavros_msgs`）；
      这里只验**内部自洽**：位置/速度/yaw 那七位恰好是低 11 位里的 1087，且与默认掩码
      `2496`（= IGNORE_AFX|AFY|AFZ|YAW_RATE，Phase 1 `traj_node.cpp` 的 `kTypeMask`）
      **无交集** —— 也就是"默认掩码下位置、速度、yaw 三样全都有效"。
    """
    valid = IGNORE_PX | IGNORE_PY | IGNORE_PZ | IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_YAW
    assert valid == 1087
    assert 2496 & valid == 0, "默认掩码 2496 下这三样该全有效；有交集说明抄错位了"


def test_轨迹参考掩码逐轴生效():
    """★★ **被忽略的轴必须真的不读** —— 而不是"读了但恰好值也对"。

    这条的鉴别力全在**故意填错值**上：被忽略的那几轴填 99 / −77 / 88 / 66，
    一旦代码去读了它们，读数会立刻变（而默认掩码下位置那几位恰好是真值 ⟹ 那个错法
    在默认掩码上**看不出来**，只有把掩码换成"忽略某几轴"才暴露）。第一手的反例形态见
    `traj_reference_fields` 的 ★★：被忽略的字段里上游**允许**放任意值。
    """
    msg = _TrajRef(position=(1.5, 99.0, -77.0), velocity=(0.3, 88.0, 66.0), yaw=0.7,
                   mask=IGNORE_PY | IGNORE_PZ | IGNORE_VY | IGNORE_VZ)
    p, v, yaw, bad = traj_reference_fields(msg)
    assert bad is None
    assert p[0] == pytest.approx(1.5)
    assert p[1] is None and p[2] is None, "位置 y/z 被掩码忽略 ⟹ 必须是 None（不是 99.0 / −77.0）"
    assert v[0] == pytest.approx(0.3)
    assert v[1] is None and v[2] is None, "速度 y/z 被掩码忽略 ⟹ 必须是 None（不是 88.0 / 66.0）"
    assert yaw == pytest.approx(0.7)


def test_有效轴上的非有限值是坏帧_被忽略的不是():
    """⚠️ 两条取向都在这一条里：**有效轴**上出现 NaN ⟹ 整帧作废（同 `_measure_mocap`）；
    **被忽略的轴**上出现 NaN **不算坏帧**（那一轴本来就不该读，上游往里放什么是它的自由）。"""
    ok = _TrajRef(position=(0.0, float("nan"), 0.0),
                  mask=IGNORE_PY)                       # 非有限的是**被忽略**的那一轴
    assert traj_reference_fields(ok)[3] is None
    bad = _TrajRef(position=(0.0, float("nan"), 0.0),
                   mask=IGNORE_PZ)                      # 非有限的是**有效**的那一轴
    assert traj_reference_fields(bad)[3] is not None
    # ★ 读数要指得出是哪一轴（而不是一句"帧坏了"）—— 复盘时那一格是唯一的线索。
    assert "y" in traj_reference_fields(bad)[3]


def test_轨迹参考是模型的一条精确解():
    """★★★ **本票的核心断言**（票面 ⚠️ 1 的落点）：`traj_reference` 给的那条参考
    连同 `hold_feedforward` 给的那个 `u`，**是模型自己的一条精确解**。

    为什么这条是核心：代价项是 `R·(u − u_ref)`，所以"参考 + `u_ref` 是不是模型的一条解"
    = "`u_ref` 对不对"。给定参考族（匀速直线 + 水平姿态 + 纯偏航）与 `u_ref = [m·g, 0,0,0]`，
    `f(x_ref, u_ref) = ẋ_ref` **恒成立**（无阻力模型里匀速平动不需要倾角，倾角是加速度才要的）
    ⟹ `hold_feedforward` **不需要**为轨迹档改动。票面那条 ⚠️（"前馈要按参考自己的导数重算"）
    对本参考族**不成立**：它要防的是"参考带持续加速度/角速率而 `u_ref` 还是悬停配平"。
    ☠️ 所以**这条测试红了就是在说**：参考族或者 `u_ref` 有一边改错了 —— 而且红法很难看
    （`step_rk4` 往前走一步就对不上参考），不是"差一点点"。

    ★ 鉴别力（**这条不是形式主义**）：把 `traj_reference` 换成**零阶保持**（位置钉在当拍参考上、
      速度那一项照旧填 `v`）当场变红 —— 那正是 `traj_reference` 的 ⚠️ 里描述的那个
      **自相矛盾**的参考（位置项要它停、速度项要它走）。2026-09-30 手工验证过：改坏之后
      位置那一行立刻差 `v·dt = 4 mm`（每步），而不是"抖一下"。
    """
    from nmpc.model import Z0MultirotorRateModel

    v = np.array([0.35, -0.12, 0.08])              # 三个轴都非 0（单轴的非 0 会漏掉耦合）
    yaw = 0.4
    cfg = ControlConfig(reference_source=REFERENCE_SOURCE_TRAJ, hold_yaw=None)
    anchor = pose_to_state((1.0, 2.0, 3.0), yaw_quat(yaw))
    x_ref = traj_reference(anchor, cfg, N, DT,
                           p_rel=(0.5, -0.25, 0.1), v_rel=tuple(v), yaw_ref=yaw)
    u_ref = hold_feedforward(P, N)
    model = Z0MultirotorRateModel(P)
    for k in range(N):                              # 逐步比（不累积误差）
        nxt = model.step_rk4(np.asarray(x_ref[k], float), np.asarray(u_ref[k], float), DT)
        assert nxt == pytest.approx(np.asarray(x_ref[k + 1], float), abs=1e-12), (
            f"第 {k} 步：参考 + 解析前馈不是模型的解 ⟹ 参考族或 u_ref 算错了"
            "（零阶保持就会这样红：位置项要它停、速度项要它走）")


def test_时域步长是必填参数_口径不对必须当场红():
    """★ "参考外推用的步长 = 求解器的离散步长"这条口径里，**本文件能钉的那一半**。

    ⚠️ 为什么值得单独一条：这三个候选数在本仓里**都是正常量级** ——
      求解器步长 `tf/N = 20 ms`、`tau_s = 50 ms`、控制周期 `1/100 Hz = 10 ms`。
      传错一个**不报错**，只让参考的时域斜率整体差一个常数倍
      （症状是"飞机总是慢一步/冲过头"，而话题上一切正常）。

    两件事各挡一个错法：
      ① `NodeCore` 的第 4 个参数**没有默认值** ⟹ "忘了传"当场 `TypeError`，
         不会退回某个看起来合理的缺省（有默认值的话，忘了传 = 静默用错步长）；
      ② **传什么就用什么**：步长翻倍，时域末点的外推距离就翻倍 ——
         这条同时钉住"它没有被 `NodeCore` 内部改写成 `tau_s` 或别的数"。
    """
    anchor = pose_to_state((0.0, 0.0, 1.0), yaw_quat(0.0))
    p_rel, v_rel = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)     # 朝 +x 以 1 m/s 走

    with pytest.raises(TypeError):
        NodeCore(P, ControlConfig(), N)                  # 少给第 4 个参数

    tips = []
    for dt in (DT, 2 * DT):
        cfg = _traj_config()
        core = NodeCore(P, cfg, N, dt)
        core.on_status(_Status(), at_s=0.0)
        core.on_traj_reference(_TrajRef(position=p_rel, velocity=v_rel), at_s=0.0)
        core.on_mocap(_Mocap(position=(0.0, 0.0, 1.0)), at_s=0.0)
        tips.append(core.cycle(0.01).x_ref[-1][0])

    assert tips[0] == pytest.approx(N * DT, abs=1e-6), "末点 = 步长 × 步数"
    assert tips[1] == pytest.approx(2 * tips[0], abs=1e-6), \
        "步长翻倍而外推距离没翻倍 ⟹ 传进去的那个数被谁改写过了"


def test_轨迹参考按参考自己的速度线性外推():
    """时域上 `p_ref(t_k) = 锚点 + p_rel + v·(k·dt)`，其余分量**逐点相同**。

    ★ 外推用的 `dt` 是**求解器的**离散步长（`OcpConfig.dt`），不是 `tau_s`、不是 1/rate。
      这三个数在本仓里分别是 20 ms / 50 ms / 10 ms ⟹ 用错了洞**不报错**，只让参考的斜率
      整体偏一个常数倍（"飞机总是慢一步/冲过头"，而话题上一切正常）。
    """
    cfg = ControlConfig(reference_source=REFERENCE_SOURCE_TRAJ)
    anchor = pose_to_state((1.0, 2.0, 3.0), yaw_quat(0.0))
    v = (0.5, 0.0, -0.25)
    x_ref = traj_reference(anchor, cfg, N, DT, p_rel=(1.0, 0.0, 0.5), v_rel=v, yaw_ref=0.0)
    assert x_ref.shape == (N + 1, NX)
    assert x_ref[0][:3] == pytest.approx([2.0, 2.0, 3.5])
    for k in range(N + 1):
        assert x_ref[k][:3] == pytest.approx([2.0 + 0.5 * k * DT, 2.0, 3.5 - 0.25 * k * DT])
        assert x_ref[k][3:6] == pytest.approx(list(v)), "速度那一项在时域上是常量"
    assert x_ref[0][6:10] == pytest.approx(x_ref[-1][6:10]), "姿态在时域上是常量"


def test_参考超时冻结_位置守住速度归零():
    """★ 参考断流 ⟹ **冻结**（Phase 1 的裁定："缺一帧参考就保持上一个目标，不回退到原点"）。

    ☠️ **只冻位置不冻速度是错的**（`traj_reference` 的 ★★）：外推项会继续把参考往前推，
      参考自己飞走而飞机照着追 —— 一个**不会报错**的漂移源。所以这条断言必须**同时**
      查两件事：位置不再外推（`x_ref[1] == x_ref[0]`）**且**速度那一项为 0。
    ★ 另一件同样重要的事：冻结**不关闸门**（`publish` 仍为 True）—— 关闸门会让兜底阶梯
      接管（hold 10 帧 ⟹ 安全悬停），那比"就地停住"激烈得多。
    """
    cfg = _traj_config(traj_stream_gap_s=0.2, mocap_timeout_s=10.0)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_traj_reference(_TrajRef(position=(0.5, 0.0, 0.0), velocity=(0.2, 0.0, 0.0)), at_s=0.0)
    core.on_mocap(_Mocap(position=(1.0, 2.0, 3.0)), at_s=0.0)
    fresh = core.cycle(0.05)
    assert fresh.publish and not core.traj_frozen(0.05)
    assert fresh.x_ref[0][3:6] == pytest.approx([0.2, 0.0, 0.0])
    assert fresh.x_ref[1][:3] == pytest.approx([1.5 + 0.2 * DT, 2.0, 3.0]), "新鲜时该外推"

    core.on_mocap(_Mocap(position=(1.0, 2.0, 3.0)), at_s=0.5)
    frozen = core.cycle(0.5)                       # 参考的龄 = 0.5 s > 0.2 s
    assert core.traj_frozen(0.5)
    assert frozen.publish, "冻结不是停发（停发会把兜底阶梯拉进来）"
    assert frozen.x_ref[0][:3] == pytest.approx([1.5, 2.0, 3.0]), "位置该守在被冻结的那一点"
    assert frozen.x_ref[0][3:6] == pytest.approx([0.0, 0.0, 0.0]), "★ 速度必须归零"
    assert frozen.x_ref[-1][:3] == pytest.approx(frozen.x_ref[0][:3]), "★ 时域上不许再外推"


def test_坏帧不刷新参考_龄继续变大直到冻结():
    """坏帧的处置：**不覆盖**上一帧好参考（于是龄继续变大 ⟹ 走到冻结），而不是"这一拍不发"。

    ★ 取向与输入合理性的那三条判据一致（**宁可退化，不可误停**）：上游偶发一帧 NaN
      不该让飞机停发；而"这一帧不算数"这件事**已经**由龄反映了 —— 不需要第二条判据。
    ★ 计数要分得开：`ref_cb` 涨而 `ref_bad` 也涨 ⟹ 上游在写坏值，`ref_bad_why` 指得出是哪一格。
    """
    cfg = _traj_config(traj_stream_gap_s=0.2, mocap_timeout_s=10.0)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_traj_reference(_TrajRef(position=(0.5, 0.0, 0.0), velocity=(0.2, 0.0, 0.0)), at_s=0.0)
    core.on_traj_reference(_TrajRef(position=(float("nan"), 0.0, 0.0), velocity=(0.2, 0.0, 0.0)),
                           at_s=0.25)
    d = core.describe_state(0.25)
    assert d["ref_cb"] == 2 and d["ref_bad"] == 1
    assert d["ref_age_s"] == pytest.approx(0.25), "坏帧不刷新 ⟹ 龄接着上一帧好参考算"
    assert "x" in d["ref_bad_why"]

    core.on_mocap(_Mocap(position=(1.0, 2.0, 3.0)), at_s=0.5)
    assert core.cycle(0.5).publish, "坏帧不该让闸门关上（它是可恢复的）"
    assert core.traj_frozen(0.5), "好帧一直不来 ⟹ 0.2 s 后冻结"


def test_没收到参考时闸门关闭且锚点不被钉住():
    """★★ 顺序断言：`traj` 档下**参考没到 ⟹ 这一拍整个作废，锚点不许被它钉住**。

    为什么这条要专门测（它不是一个"顺手"的顺序）：锚点是"闸门第一次打开那一拍的位置"，
    而 `traj` 档的目标点 = **锚点 + 相对量**。若在参考还没到的那些拍上就把锚点钉了，
    等参考真到的那一拍，目标点会**少走一个参考位移** —— 症状是"启动时目标偏了一点"，
    不会报错，也不会让任何阈值变红。
    ★ 断言用**黑盒**做法（不碰 `core._anchor_pos`）：先在没有参考时把飞机放在 A 点，
      再把飞机挪到 B 点并给上参考 ⟹ 目标点该是 `B + p_rel`。若是 `A + p_rel`，说明钉早了。
    """
    cfg = _traj_config(mocap_timeout_s=10.0)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(position=(1.0, 2.0, 3.0)), at_s=0.0)     # A 点，此时还没有参考
    assert core.cycle(0.05).reason == "no_traj_reference"

    core.on_traj_reference(_TrajRef(position=(0.5, 0.0, 0.0)), at_s=0.1)
    core.on_mocap(_Mocap(position=(4.0, 5.0, 6.0)), at_s=0.1)     # B 点
    cyc = core.cycle(0.15)
    assert cyc.publish
    assert cyc.x_ref[0][:3] == pytest.approx([4.5, 5.0, 6.0]), \
        "锚点被参考到达**之前**的那一拍钉住了（目标点该以 B 点为基准）"


def test_hold_档完全不理轨迹参考():
    """★ 两个来源**正交**的另一半：`hold` 档下 `/traj/reference` 即使送进来也一个字不读。

    这条是"默认档 = 不改变任何现有行为"的可执行版本：`hold` 档的目标点必须还是
    `锚点 + hold_offset`，与有没有参考消息无关。
    """
    cfg = ControlConfig(stop=StopConfig(warmup_cycles=0))
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_traj_reference(_TrajRef(position=(9.0, 9.0, 9.0), velocity=(5.0, 0.0, 0.0)),
                           at_s=0.0)
    core.on_mocap(_Mocap(position=(1.0, 2.0, 3.0)), at_s=0.0)
    cyc = core.cycle(0.05)
    assert cyc.publish
    assert cyc.x_ref[0][:3] == pytest.approx(
        [1.0 + cfg.hold_offset[0], 2.0 + cfg.hold_offset[1], 3.0 + cfg.hold_offset[2]])
    assert cyc.x_ref[0][3:6] == pytest.approx([0.0, 0.0, 0.0]), "hold 档的速度恒为 0"


def test_绝对目标直接用_不加锚点():
    """★★ 票 15 的核心口径：`control` 档的目标是**绝对点**，绝不再叠锚点。

    ★ 鉴别力：测量位置故意放在 `(1,2,3)`、目标放在 `(4,5,6)`。若代码照抄了 `traj` 档的
      "锚点 + 参考"，读数会是 `(1,2,3) + (4,5,6)` 或 `(1,2,3)` 一类的数；只有"直接用目标"
      才得到 `(4,5,6)`。这个错**不会报错**，只会让整架飞机偏一截（`hold_reference` 的 ★★
      是同一族静默失败）。
    """
    cfg = _control_config(mocap_timeout_s=10.0)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(position=(1.0, 2.0, 3.0)), at_s=0.0)
    core.on_control_target(_TrajRef(position=(4.0, 5.0, 6.0),
                                    velocity=(0.2, -0.1, 0.05), yaw=0.3), at_s=0.0)
    cyc = core.cycle(0.05)
    assert cyc.publish
    assert cyc.x_ref[0][:3] == pytest.approx([4.0, 5.0, 6.0])
    assert cyc.x_ref[0][3:6] == pytest.approx([0.2, -0.1, 0.05])
    assert yaw_from_quat(cyc.x_ref[0][6:10]) == pytest.approx(0.3)
    assert cyc.x_ref[-1][:3] == pytest.approx(
        [4.0 + 0.2 * N * DT, 5.0 - 0.1 * N * DT, 6.0 + 0.05 * N * DT]), \
        "绝对目标也按时域步长线性外推（与 traj 参考同一条自洽性）"


def test_绝对目标是模型的一条精确解():
    """与 `test_轨迹参考是模型的一条精确解` 同一条判据，换成绝对参考族。"""
    from nmpc.model import Z0MultirotorRateModel

    v = np.array([0.35, -0.12, 0.08])
    yaw = 0.4
    cfg = ControlConfig(reference_source=REFERENCE_SOURCE_CONTROL)
    x_ref = absolute_reference(cfg, N, DT, (1.0, 2.0, 3.0), tuple(v), yaw)
    u_ref = hold_feedforward(P, N)
    model = Z0MultirotorRateModel(P)
    for k in range(N):
        nxt = model.step_rk4(np.asarray(x_ref[k], float), np.asarray(u_ref[k], float), DT)
        assert nxt == pytest.approx(np.asarray(x_ref[k + 1], float), abs=1e-12), (
            f"第 {k} 步：绝对参考 + 解析前馈不是模型的解 ⟹ 参考族或 u_ref 算错了")


def test_控制目标超时关闸_不是冻结():
    """★★ 票 15 的安全语义：`control` 一停，本节点**停流**（关闸），不是把飞机冻在原地。

    ★ 与 `test_参考超时冻结` 刻意相反。理由：`control` 同时拿着开始键/解锁/切模式/降落，
      它失联后继续稳稳地悬停 = "任务总管没了"这件事在话题面上完全看不见；停流让 PX4 的
      OFFBOARD 断流保护接管，才是兜底。
    """
    cfg = _control_config(control_target_timeout_s=0.2, mocap_timeout_s=10.0)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(position=(1.0, 2.0, 3.0)), at_s=0.0)
    core.on_control_target(_TrajRef(position=(2.0, 2.0, 3.0)), at_s=0.0)
    fresh = core.cycle(0.05)
    assert fresh.publish

    core.on_mocap(_Mocap(position=(1.0, 2.0, 3.0)), at_s=0.5)
    stale = core.cycle(0.5)                       # 目标龄 0.5 s > 0.2 s
    assert stale.reason == "no_control_target"
    assert stale.x0 is None and stale.x_ref is None, "超时必须关闸，而不是拿冻结参考继续发"
    assert "0.50" in stale.detail and "0.2" in stale.detail, "读数要指得出龄与上限"


def test_控制目标缺轴算坏帧_不覆盖上一帧好的():
    """`/control/target` 必须三轴齐全（位置/速度/yaw）；缺任何一轴 ⟹ 整帧作废。

    ★ 与 `traj` 档不同：那里"忽略某轴"是合法语义（那一轴保持锚点）；这一档的发布方是
      我们自己的 `control`，每次都填满（`type_mask=2496`）⟹ 出现缺轴只可能是实现错了。
      把它当 `None` 塞进参考会让那一轴静默变成 0（z=0 = 目标在地面）。
    """
    cfg = _control_config(control_target_timeout_s=10.0, mocap_timeout_s=10.0)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_control_target(_TrajRef(position=(2.0, 3.0, 4.0)), at_s=0.0)
    core.on_control_target(_TrajRef(position=(9.0, 9.0, 9.0), mask=IGNORE_PX), at_s=0.1)
    d = core.describe_state(0.1)
    assert d["ref_cb"] == 2 and d["ref_bad"] == 1
    assert "x" in d["ref_bad_why"]
    core.on_mocap(_Mocap(position=(0.0, 0.0, 1.0)), at_s=0.1)
    cyc = core.cycle(0.15)
    assert cyc.publish
    assert cyc.x_ref[0][:3] == pytest.approx([2.0, 3.0, 4.0]), \
        "坏帧不许覆盖上一帧好目标（9,9,9 不能被用上）"


def test_控制档一帧目标都没到过_闸门关闭():
    cfg = _control_config(mocap_timeout_s=10.0)
    core = _core(cfg)
    core.on_status(_Status(), at_s=0.0)
    core.on_mocap(_Mocap(position=(1.0, 2.0, 3.0)), at_s=0.0)
    cyc = core.cycle(0.05)
    assert cyc.reason == "no_control_target"
    assert cyc.x0 is None


# ───────────────────────────── 小工具 ─────────────────────────────

def _publishes_after_warmup(core: NodeCore, now_s: float) -> bool:
    """连推到闸门开放为止（默认 `WARMUP_CYCLES` 拍），返回最后一拍发不发。"""
    last = None
    for i in range(StopConfig().warmup_cycles + 1):
        last = core.cycle(now_s + 1e-6 * i)
    return bool(last.publish)
