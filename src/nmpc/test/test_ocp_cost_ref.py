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

import math

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


# ─────────────── 代价的**镜像井**及其修复（2026-09-28 在 SITL 闭环上撞出来，ticket 16 修复）───
#
# 这一节不是在验收 ticket 04 的交付，而是把**偏航代价的一个结构性后果**变成可复现的数字：
# 它不是"权重没调好"，而是**代价函数的形式**决定的，而且正是 ticket 09 那场 runaway 的底层原因
# （详见 `docs/control/tier-a-sitl-hover.md` §4）。
#
# ★ 2026-09-29 操作者裁定后，**默认权重已改**（`qw = 0.1, qatt = 2.0`；`R_yaw` 保持参考实现的
#   500 —— 同日一度跟着降到 50，因会让 RTI 迭代在大误差下**收敛极慢**（发指令的窗口里逐拍乱跳）
#   而撤回，见 `OcpConfig.r_diag` 的 ☠️）：
#   镜像井**已消掉**。下面两条一起读 —— 一条钉「参考实现的 `qw = 0` 为什么是井」（反事实，留住
#   是为了让"为什么非改不可"可执行、且万一有人把 `qw` 改回 0 会红），另一条钉「默认不再是井」。

_MIRROR_PSI = 1.618


def _ref_and_mirror(cfg: OcpConfig) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """参考机头 `ψ` 与它的镜像 `2π − ψ` 两条姿态，外加一条控制量（`u_ref = u_pred`）。"""
    x_ref = np.tile(np.concatenate([np.zeros(6), yaw_quat(_MIRROR_PSI)]), (cfg.N + 1, 1))
    x_pred = np.tile(np.concatenate([np.zeros(6), yaw_quat(2 * math.pi - _MIRROR_PSI)]),
                     (cfg.N + 1, 1))
    u = np.tile([Z0Params().t_hover, 0.0, 0.0, 0.0], (cfg.N, 1))
    return x_ref, x_pred, u


def test_the_cost_cannot_tell_the_reference_heading_from_its_mirror():
    """★★ 镜像井的**机理**：`ψ` 与 `2π − ψ` 的四元数**矢量部完全相同** ⟹ `qw = 0` 时无法区分。

    `yaw_quat(ψ) = [cos(ψ/2), 0, 0, sin(ψ/2)]`、`yaw_quat(2π−ψ) = [−cos(ψ/2), 0, 0, sin(ψ/2)]`
    ⟹ 两者只差 `w` 的符号；`w` 权重为 0 时这些代价**分不出**参考机头与它的镜像。

    ⚠️ **本用例现在钉的是「参考实现那组权重」这个反事实**（`q_diag` 四元数段 = `(0.0,0.1,0.1,0.1)`），
    而**默认已经不是它**（ticket 16）。留着它的两个理由：① 把"为什么非改不可"变成可执行的对照；
    ② 哪天有人把 `qw` 改回 `0`，这里会红。默认权重下的正向结论见下一条用例。
    """
    cfg = OcpConfig(q_diag=(1e1,) * 6 + (0.0, 0.1, 0.1, 0.1))
    x_ref, x_pred, u = _ref_and_mirror(cfg)

    at_ref = cfg.cost(np.tile(x_ref[0], (cfg.N + 1, 1)), u, x_ref, u)
    at_mirror = cfg.cost(x_pred, u, x_ref, u)
    assert at_ref == pytest.approx(0.0, abs=1e-24)
    assert at_mirror == pytest.approx(0.0, abs=1e-24), \
        f"镜像机头（相差 {2 * math.pi - 2 * _MIRROR_PSI:.3f} rad）的代价居然不是 0：{at_mirror}"

    # 反面：把 `w` 那一项罚起来（`qw = 0.1`）⟹ 镜像井消失。
    # 解析值：误差只在 w 上，|Δw| = 2·cos(ψ/2)，阶段权重 qw、共 N 步（终端权重里四元数段是 0）。
    w_err = 2.0 * math.cos(_MIRROR_PSI / 2.0)
    heavier = OcpConfig(q_diag=(1e1,) * 6 + (0.1, 0.1, 0.1, 0.1))
    fixed = heavier.cost(x_pred, u, x_ref, u)
    assert fixed == pytest.approx(0.1 * w_err ** 2 * cfg.N, rel=1e-9)
    assert fixed > 9.0, f"罚起 w 之后镜像机头该有明显的代价，实际 {fixed:.4f}"


def test_the_default_cost_separates_the_reference_heading_from_its_mirror():
    """★ ticket 16：**默认** `qw = 0.1` ⟹ 镜像机头的代价不再 = 0，而是 ≈ 9.53。

    解析值：镜像误差只在 `w` 上，`|Δw| = 2·cos(ψ/2)`，阶段权重 `qw = 0.1`、共 `N` 步
    （终端权重四元数段是 0）⟹ `0.1 · (2cos(ψ/2))² · N`。参考机头本身的代价仍是 0
    （同一条姿态，代价只看偏差）。
    """
    cfg = OcpConfig()
    assert cfg.q_diag[6] > 0.0, \
        f"前提变了：默认 `qw` 又成了 {cfg.q_diag[6]}（镜像井会复发）"
    x_ref, x_pred, u = _ref_and_mirror(cfg)

    w_err = 2.0 * math.cos(_MIRROR_PSI / 2.0)
    at_ref = cfg.cost(np.tile(x_ref[0], (cfg.N + 1, 1)), u, x_ref, u)
    at_mirror = cfg.cost(x_pred, u, x_ref, u)
    assert at_ref == pytest.approx(0.0, abs=1e-24)
    assert at_mirror == pytest.approx(cfg.q_diag[6] * w_err ** 2 * cfg.N, rel=1e-9)
    assert at_mirror > 9.0, \
        f"默认权重下镜像机头该有明显的代价（≈9.53），实际 {at_mirror:.4f}"
