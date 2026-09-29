"""seam 2（**零 ROS**）：配平点与"推力/姿态该往哪边走"。

覆盖 ticket 04 的验收项 (b)(c)(d)：
  (b) 悬停配平 → F ≈ T_hover，ω ≈ 0
  (c) 目标在 +z 0.5 m → F > T_hover，ω ≈ 0
  (d) 目标在 +x 0.5 m → 倾斜方向与 **ticket 03 的推导一致**

⚠️ **(d) 只能验证自洽，不能验证正确**（票面原话）：这里断言的是
[`docs/control/sign-map.md`](../../../docs/control/sign-map.md) §3.4 的
`ω = κ(ẑ × d̂)`（+x ⟹ ω_y > 0、+y ⟹ ω_x < 0），那是**约定层**的一致性；
"真 PX4 是不是也这么认"只有 SITL 闭环（seam 3）能回答。

★ 两个贯穿本文件的取法，先写明白（都是实测逼出来的，理由见 `conftest.py`）：

1. **先热身再断言**（`solve_warm`）：单次 RTI 迭代的第 0 次 solve 拿到的是出厂初始点。
2. **`u_ref` 必须显式给**，且给什么决定了在测哪件事。静态偏移（`+x 0.5 m`）下两条实测
   （2026-09-28 本机，连解 12 次）：

   | `u_ref` | 第 0 次 | 第 1 次 | 第 11 次 | 形态 |
   |---|---|---|---|---|
   | `0`（参考实现的口径：`yref` 控制段填 0） | 0.0000 | +0.3268 | **+0.3239** | 很快稳定 |
   | **缺省**（上一帧解平移一拍） | 0.0000 | +0.3225 | **+1.3487** | **一路往上爬，不收敛** |

   ⟹ (d) 这条测试**必须显式给 `u_ref`**；缺省路径下这个量既不稳定、也不是 0
   （第 0 次的 0 只是初始迭代点）。缺省路径为什么会长、它对在线节点意味着什么，
   见 [`offline-ocp.md`](../../../docs/control/offline-ocp.md) §4.4 与
   `test_default_u_ref_on_a_static_reference_ramps`。
"""

from __future__ import annotations

import numpy as np
import pytest

from nmpc import G, Z0Params
from nmpc.ocp import OfflineOcp

#: 解析悬停推力 [N] —— 与 `Z0Params.t_hover` 同一来源，不抄数
T_HOVER = Z0Params().mass * G

#: "另外两轴不该动"的判据容差。★ 不是 0：QP 解出来的是 1e-18 不是 0（见 `test_ocp_constraints.py`
#: 的 `ZERO_TOL` 那条教训）。实测（2026-09-29 复核轮**逐项现算**，**两侧权重都量了**）：
#:   · ticket 16 方案 E（`qw=0.1, qatt=2.0, R_yaw=50`）：交叉轴 **1.1e-10 / 2.8e-9**
#:     （`test_lateral_target_tilts_around_roll_axis`；同一文件的 `test_forward_…` 交叉轴**恰为 0.0**）；
#:   · 旧权重（`qw=0, qatt=0.1, R_yaw=500`）对照：**同一用例**交叉轴 **3.3e-14 / 5.0e-14**。
#: ⟹ 泄漏确实随这组权重涨了 4–5 个数量级，但**始终比主轴 0.32 低 8 个量级**，
#:    且有一条用例是**精确 0** ⟹ 判它是**舍入/条件数**问题，不是物理耦合。
#: 取 1e-7：比主轴小 6 个量级，真出现"另一轴跟着动"仍会红。
#: ⚠️ **别把它和 `test_ocp_constraints.py` 的 `ZERO_TOL = 1e-9` 统一** —— 那个更紧，
#:    而这里实测到的 2.8e-9 **已经超过** 1e-9（且两者量的不是同一个东西）⟹ 统一会让本文件恒红。
CROSS_TOL = 1e-7


def test_hover_trim_is_exact_with_default_feedforward():
    """(b) 缺省路径（全新实例的首帧 `u_ref` = 悬停前馈）：配平稳稳落在 `m·g` 上，**精确**。

    这是"配平"最干净的口径：参考本身就是前馈一致的不动点，代价降到 1e-31 量级。

    ★ **用自己的实例**（`copy_with()`），不共用模块夹具：缺省路径吃的是"这个实例上一帧解"
    这份**私有状态**，借用夹具就等于把结论挂到"本文件里这个用例碰巧排第一"上。
    """
    ocp = OfflineOcp()
    x0 = ocp.hover_state()
    x_ref = np.tile(x0, (ocp.config.N + 1, 1))
    for _ in range(10):        # 这个实例上手动热身（`solve_warm` 绑的是模块夹具）
        sol = ocp.solve(x0, x_ref)
    assert sol.u[0] == pytest.approx(T_HOVER, rel=1e-9)
    assert sol.u[0] == pytest.approx(Z0Params().t_hover, rel=1e-9)
    assert np.allclose(sol.u[1:], 0.0, atol=1e-12)
    assert sol.info.cost < 1e-20, "不动点上代价应该基本为 0"


def test_default_u_ref_on_a_static_reference_ramps():
    """★ **缺省 `u_ref` 在静态参考下不收敛**，如实钉住（实测，不是缺陷报告）。

    实测（`+x 0.5 m`，全新实例、不给 `u_ref`，连解 12 次）：

        第 0 次 ω_y=0.0000（只是初始迭代点）→ 0.3225 → 0.5085 → 0.6598 → 0.7901
        → 0.9035 → 1.0027 → 1.0898 → 1.1667 → 1.2348 → 1.2951 → 第 11 次 1.3487

    单调上爬、F 从 7.36 掉到 6.77 ⟹ 缺省的 `u_ref` = **上一帧解**，于是它对自己的**前馈**
    构成正反馈：解出一点倾斜 → 下一帧的参考量就多认一点倾斜。

    ⚠️ **这不等价于"在线会发散"**：本用例把状态钉死在悬停，误差永远不消；闭环里状态会动、
    误差会减。**在线能不能走缺省这条路，只有 SITL 闭环能回答** —— 归 ticket 06/07/11
    （见 [`offline-ocp.md`](../../../docs/control/offline-ocp.md) §4.4）。本用例只保证这个
    离线事实**不被忘记**：它一红，就是有人动了缺省语义。
    """
    ocp = OfflineOcp()
    x0 = ocp.hover_state()
    x_ref = np.tile(x0, (ocp.config.N + 1, 1))
    x_ref[:, 0] += 0.5        # 静态偏移：目标恒在 +x 0.5 m

    omega_y = [ocp.solve(x0, x_ref).u[2] for _ in range(12)]   # 不给 u_ref ⟹ 走缺省路径

    shown = [round(v, 4) for v in omega_y]
    assert abs(omega_y[0]) < 1e-9, f"第 0 次就是初始迭代点，不该有倾斜：{shown}"
    assert all(b > a for a, b in zip(omega_y[1:], omega_y[2:])), \
        f"第 1 次起应**单调递增**（实测 0.3225 → … → 1.3487），实际 {shown}"
    assert omega_y[-1] > 1.0, f"12 次之内应明显爬过 1 rad/s，实际 {shown}"
    assert omega_y[-1] < 3.84, f"仍在速率上限内 —— 是慢漂移不是打满，实际 {shown}"


def test_hover_trim_has_rti_bias_without_feedforward(solve_warm, ocp: OfflineOcp, u_zero):
    """(b) 同一次悬停，`u_ref = 0` 时 F **偏大 6.16 %**（7.8111 vs 7.3575）—— 如实钉住。

    ★ 这不是冷启动，是**已经收敛的稳态值**（连解 12 次不动）。原因在根上：
    `SQP_RTI` 单次迭代的解是**迭代映射的不动点**，不是"这个问题的最优解"；
    `u_ref = 0` 把代价的谷底挪到了"零推力"，稳态就带着这个偏置。
    ⟹ 断言只给**方向 + 松带宽**（≤ 10 %），别把偏置当正确值写死；
    真要精确配平，走上面那条（前馈一致的参考）。
    """
    x0 = ocp.hover_state()
    sol = solve_warm(x0, np.tile(x0, (ocp.config.N + 1, 1)), u_zero)
    assert sol.u[0] > T_HOVER, "偏置方向是偏大（实测 +6.16 %）"
    assert sol.u[0] == pytest.approx(T_HOVER, rel=0.10)
    assert np.allclose(sol.u[1:], 0.0, atol=1e-12), "悬停参考不该产生任何角速率"


def test_upward_target_needs_more_thrust_and_no_tilt(solve_warm, const_ref, u_zero,
                                                     ocp: OfflineOcp):
    """(c) 目标在 +z 0.5 m：只加推力，不倾斜（实测 F = 9.5362 N）。"""
    x0 = ocp.hover_state()
    sol = solve_warm(x0, const_ref([0.0, 0.0, 0.5]), u_zero)
    assert sol.u[0] > T_HOVER, "要往上走必须比悬停推力大"
    assert np.allclose(sol.u[1:], 0.0, atol=1e-12), f"垂直目标不该产生角速率：ω={sol.u[1:]}"


def test_forward_target_tilts_pitch_downward_per_sign_map(solve_warm, const_ref, u_zero,
                                                          ocp: OfflineOcp):
    """(d) 目标在 +x 0.5 m：`sign-map.md` §3.4 说 ω_y > 0（低头前飞），另两轴不为所动。"""
    x0 = ocp.hover_state()
    sol = solve_warm(x0, const_ref([0.5, 0.0, 0.0]), u_zero)
    assert sol.u[2] > 0.0, f"约定是 +x 飞行 ⟹ ω_y > 0，实际 ω_y={sol.u[2]:.6f}"
    assert abs(sol.u[1]) < CROSS_TOL and abs(sol.u[3]) < CROSS_TOL, f"另外两轴不该动：ω={sol.u[1:]}"


def test_lateral_target_tilts_around_roll_axis(solve_warm, const_ref, u_zero,
                                               ocp: OfflineOcp):
    """+y 目标 ⟹ `ẑ × ŷ = −x̂` ⟹ ω_x < 0（左滚）。与 +x 那条对称（实测 −0.323941）。"""
    x0 = ocp.hover_state()
    sol = solve_warm(x0, const_ref([0.0, 0.5, 0.0]), u_zero)
    assert sol.u[1] < 0.0, f"按 κ(ẑ × ŷ) = κ(−x̂) 应是 ω_x < 0，实际 {sol.u[1]:.6f}"
    assert abs(sol.u[2]) < CROSS_TOL and abs(sol.u[3]) < CROSS_TOL, f"另外两轴不该动：ω={sol.u[1:]}"


@pytest.mark.parametrize("axis,expected_v_index", [(0, 0), (1, 1)])
def test_tilt_direction_is_self_consistent_with_the_model(solve_warm, const_ref, u_zero,
                                                          ocp: OfflineOcp, axis: int, expected_v_index: int):
    """把 (d) 往**物理**上再推一步：解出的 `u` 必须真的把机体推向目标那一侧。

    手法：拿 `Z0MultirotorRateModel.step_rk4` 用自己的数值动力学积分一步，看**水平加速度**
    往哪边去。这条挡住的是"符号在代价里对、在动力学里反"这种自相矛盾 ——
    **不是**在验证"PX4 也这么认"（那要 SITL）。
    """
    off = np.zeros(3)
    off[axis] = 0.5
    x0 = ocp.hover_state()
    sol = solve_warm(x0, const_ref(off), u_zero)
    v_next = ocp.model.step_rk4(x0, sol.u, ocp.dt)[3:6]
    assert v_next[expected_v_index] > 0.0, f"应该往 {off} 加速，实际 v={v_next}"
    other = 1 - expected_v_index
    assert abs(v_next[other]) < 1e-12, f"不该有侧向加速度：v={v_next}"
