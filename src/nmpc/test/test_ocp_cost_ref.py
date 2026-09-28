"""seam 2（**零 ROS**）：代价罚的是**相对 `u_ref` 的偏差**，不是角速率的绝对值（M6）。

覆盖 ticket 04 的验收项 (h)：**持续速率场景（绕圈、定点自旋）不该被系统性地拉向零**。

★ 这条是整张票里最容易"看起来对、实际上错"的一项：参考实现把 `yref` 的控制部分**全填 0**
（`multirotor_rate_mpc.py`），那等价于**罚角速率的绝对值** ⟹ 需要持续角速率的场景会被
无声压制。下面用**同一个偏航参考、两组不同的 `u_ref`** 把它变成可判定的：

    偏航以 4.0 rad/s 匀速转（参考本身就是这条轨迹）
      u_ref = 前馈（ω_z = 4.0）→ 解出 ω_z ≈ 4.0   ← 跟住了
      u_ref = 0（参考实现的口径）→ 解出 ω_z ≈ 0   ← 持续角速率被抑制

⚠️ 另外钉住 `info.cost` 的**出处**：`SQP_RTI` 下 `solver.get_cost()` 不是返回轨迹的代价
（实测差 ~96 倍，见 `SolveInfo` 的说明），本包自己算 —— 下面用一条**朴素双重循环**独立复核它。
"""

from __future__ import annotations

import numpy as np
import pytest

from nmpc import Z0Params, yaw_quat
from nmpc.ocp import OcpConfig, OfflineOcp

#: 匀速偏航的角速率 [rad/s] —— 取 4.0 是**有意的**：它 > 横滚上限的一半（1.92），
#: 于是这条测试同时是 M3（偏航上限独立）的行为证据；又 < 偏航上限（6.28），可行。
OMEGA_D = 4.0


def _yaw_spin_ref(ocp: OfflineOcp, omega_d: float) -> np.ndarray:
    """参考轨迹：以 `omega_d` 匀速偏航（位置/姿态其余分量不变）。"""
    x_ref = np.tile(ocp.hover_state(), (ocp.config.N + 1, 1))
    for i in range(ocp.config.N + 1):
        x_ref[i, 6:10] = yaw_quat(omega_d * i * ocp.dt)
    return x_ref


def test_sustained_yaw_is_not_suppressed_when_u_ref_carries_the_feedforward(solve_warm,
                                                                           ocp: OfflineOcp):
    """(h) 前馈一致的持续偏航：解出的 ω_z 必须**跟住 4.0**，而不是被拉向 0。"""
    p = Z0Params()
    x0 = ocp.hover_state()
    u_ref = np.tile([p.t_hover, 0.0, 0.0, OMEGA_D], (ocp.config.N, 1))
    sol = solve_warm(x0, _yaw_spin_ref(ocp, OMEGA_D), u_ref)
    assert sol.u[3] == pytest.approx(OMEGA_D, abs=1e-3), \
        f"持续角速率被吃掉了：ω_z={sol.u[3]:.6f}，期望 {OMEGA_D}"
    assert sol.u[0] == pytest.approx(p.t_hover, rel=1e-3), "匀速偏航不该改变推力需求"


def test_same_reference_with_u_ref_zero_suppresses_the_rate(solve_warm, ocp: OfflineOcp):
    """(h) 反面：**同一个参考**、`u_ref = 0`（参考实现的口径）⟹ 持续角速率被压到近零。

    这条不是在"验收一个失败"，而是把 M6 要修的**具体偏差**钉成可复现的数字 ——
    没有它，上面那条测试无法证明"差别来自 `u_ref`"而不是别的什么。
    """
    x0 = ocp.hover_state()
    u_ref = np.zeros((ocp.config.N, 4))
    sol = solve_warm(x0, _yaw_spin_ref(ocp, OMEGA_D), u_ref)
    assert abs(sol.u[3]) < 0.05, \
        f"u_ref=0 时持续偏航应被压制（这就是参考实现的病），实际 ω_z={sol.u[3]:.6f}"


def test_cost_matches_an_independent_naive_double_loop(ocp: OfflineOcp, solve_warm, const_ref,
                                                       u_zero):
    """`info.cost` 的**定义**由一条朴素双重循环独立复核（不共用任何向量化路径）。"""
    x0 = ocp.hover_state()
    x_ref = const_ref([0.5, 0.0, 0.0])
    sol = solve_warm(x0, x_ref, u_zero)

    cfg = ocp.config
    expected = 0.0
    for i in range(cfg.N):
        for k in range(len(cfg.q_diag)):
            expected += cfg.q_diag[k] * (sol.x_pred[i][k] - x_ref[i][k]) ** 2
        for k in range(len(cfg.r_diag)):
            expected += cfg.r_diag[k] * (sol.u_pred[i][k] - u_zero[i][k]) ** 2
    for k in range(len(cfg.q_e_diag)):
        expected += cfg.q_e_diag[k] * (sol.x_pred[cfg.N][k] - x_ref[cfg.N][k]) ** 2

    assert sol.info.cost == pytest.approx(expected, rel=1e-12)
    assert sol.info.cost > 0.0
    assert np.isfinite(sol.info.cost)


def test_cost_penalizes_the_deviation_from_u_ref(ocp: OfflineOcp):
    """`u_ref` 是**基准**，代价只看偏差 —— 三条解析可判定的性质（不经过求解器）。

    1. 偏差为 0 ⟹ 代价为 0（同一条轨迹，`u_ref = u_pred`）
    2. 同一条轨迹、换个 `u_ref` ⟹ 代价从 0 变成正数（说明 `u_ref` 是"参考量"而不是被忽略）
    3. 偏差翻倍 ⟹ 代价 ×4（**二次型**，罚的确实是那个偏差）

    反面（参考实现的口径）是把 `yref` 的控制部分全填 0 ⟹ 变成罚 `‖u‖²`，
    即"零角速率才是最优"，持续角速率场景被系统性压制 —— 见本文件上面两条偏航用例。
    """
    cfg = ocp.config
    x_ref = np.tile(ocp.hover_state(), (cfg.N + 1, 1))
    x_pred = x_ref.copy()
    u_pred = np.tile([Z0Params().t_hover, 0.0, 0.0, 0.0], (cfg.N, 1))

    assert cfg.cost(x_pred, u_pred, x_ref, u_pred.copy()) == pytest.approx(0.0, abs=1e-24)

    u_zero = np.zeros((cfg.N, 4))
    assert cfg.cost(x_pred, u_pred, x_ref, u_zero) > 0.0

    dev = np.zeros((cfg.N, 4))
    dev[:, 0] = 1.0                 # 只在推力上加 1 N 的偏差
    c1 = cfg.cost(x_pred, u_pred, x_ref, u_pred + dev)
    c2 = cfg.cost(x_pred, u_pred, x_ref, u_pred + 2 * dev)
    assert c1 == pytest.approx(cfg.r_diag[0] * 1.0 ** 2 * cfg.N, rel=1e-12)
    assert c2 == pytest.approx(4 * c1, rel=1e-12)


def test_cost_uses_the_config_weights_not_hardcoded_ones(ocp: OfflineOcp):
    """换一组权重，代价必须跟着变 —— 挡住"权重抄成常数"（M2 的同类毛病）。

    解析值：位置偏差 1 m，阶段权重 1e1 × 50 步 + 终端权重 3e2 ⟹ 800；
    把阶段位置权重提到 1e2 ⟹ 100×50 + 300 = 5300（**终端那一项没动**，所以不是整体 ×10）。
    """
    cfg = OcpConfig()
    x_pred = np.tile(ocp.hover_state(), (cfg.N + 1, 1))
    x_ref = x_pred.copy()
    x_ref[:, 0] += 1.0
    u_pred = np.zeros((cfg.N, 4))

    base = cfg.cost(x_pred, u_pred, x_ref, u_pred)
    heavier = OcpConfig(q_diag=(1e2,) + (1e1,) * 5 + (0.0, 0.1, 0.1, 0.1)).cost(
        x_pred, u_pred, x_ref, u_pred)
    assert base == pytest.approx(1e1 * cfg.N + 3e2, rel=1e-12)
    assert heavier == pytest.approx(1e2 * cfg.N + 3e2, rel=1e-12)
