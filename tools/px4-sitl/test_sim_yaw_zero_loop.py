"""`sim_yaw_zero_loop.py` 的判据（ticket 24 路线 4）。

**这组测试要能分辨「有错配」与「没错配」** —— 只比均值/只看收敛会漏掉它（票面原话）。
所以每条断言都挑一条**只有错配存在时才会变**的量：

| 判据 | 挑的是哪一列 | 为什么它区分得开 |
|---|---|---|
| `test_注入与_cpp_的左乘_Rz_逐分量同构` | 四元数本身 | `yaw0 = 0` 时**恰好**恒等、不为 0 时**恰好**是 `Rz(−yaw0)` |
| `test_注入只把机头减去_yaw0_倾角不动` | ZYX 分解 | 差一个 `Rz` 与不差，在这一列上是 `yaw0` 与 `0` 的区别（roll/pitch 必须精确不变） |
| `test_闭环位置误差被吸收_指纹只在机头那一列` | **机头**（不是位置） | 位置环**会**把位置误差吸收掉 ⟹ 只有机头那一列留得下指纹 |
| `test_闭环初段去向被系统性转过_yaw0` | **第一拍速度方向** | 位置列在 30° 以内是 `0.0025 m`（看着正常），这一列已经偏了 `29°` |
| `test_机头收敛时间常数与_yaw0_无关` | 机头 `τ` | **与 `yaw0` 无关** —— 错配不改对象、只转设定点（`τ` 绝对值随代价权重 `R_yaw` 走，见该用例） |
| `test_大_yaw0_下水平环发散_而一致臂不发散` | 末段振荡峰峰值 | 位置均值在两种情形下都接近 0，峰峰值不会；⚠️ 一致臂退化 ⟹ 归因不成立 |

⚠️ 机头那一列是**一阶慢收敛**，且**快慢由代价权重 `R_yaw` 定**（实测 `τ ≈ 2.05 · R_yaw`）：
当前默认 `R_yaw = 5e2` ⟹ `τ ≈ 1026 s`，60 s 只走完 `1 − e^{−60/1026} = 5.7%` 的路（实测 1.03° / 20°），
**不是**"几十秒就稳态等于 `yaw0`"；稳态才是 `参考 + yaw0`。（`R_yaw = 50` 的历史读数：τ ≈ 105 s、20 s 走完 17%。）

跑法（**先 source**）：

    source /opt/ros/humble/setup.bash && source /home/chen/px4_msgs_ws/install/setup.bash
    source install/setup.bash
    export ACADOS_SOURCE_DIR=$HOME/acados LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
    python3 -m pytest tools/px4-sitl/test_sim_yaw_zero_loop.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("nmpc")
pytest.importorskip("acados_template")

REPO = Path(__file__).resolve().parents[2]
for p in (REPO / "src" / "nmpc", REPO / "tools" / "px4-sitl"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import sim_yaw_zero_loop as S  # noqa: E402
from nmpc.node_core import yaw_from_quat  # noqa: E402


@pytest.fixture(scope="module")
def rig():
    """一个 `OfflineOcp` 给整个文件用 —— codegen 只做一次。"""
    return S.Rig()


# ───────────────────────── ① 注入本身：与 C++ 的 `yaw_zeroed` 同构 ─────────────────────────


def test_注入在_yaw0_为零时恒等_不为零时是真旋转():
    q = S.quat_from_yaw(math.radians(20.0))
    assert np.allclose(S.yaw_zero(q, 0.0), q, atol=1e-12), "yaw0 = 0 必须是**恰好**恒等"
    q90 = S.yaw_zero(q, math.radians(90.0))
    assert not np.allclose(q90, q, atol=1e-3), "yaw0 ≠ 0 必须是真的转了——否则整条实验是空跑"
    assert abs(float(np.linalg.norm(q90)) - 1.0) < 1e-12


def test_注入与_cpp_的左乘_Rz_逐分量同构():
    """`yaw_zeroed(q, yaw0) = Rz(−yaw0) ⊗ q`（wxyz）—— 手写旋转矩阵独立算一遍对账。

    对账口径：`Rz(−yaw0) ⊗ q` 与「先把 q 转成旋转矩阵、左乘 `Rz(−yaw0)` 的矩阵、再转回四元数」
    必须逐分量相等（允许返回同一个 `−q` 分支 —— 它表示同一个旋转）。
    """
    q = S.quat_from_yaw(math.radians(20.0))
    for yaw0_deg in (5.0, 37.0, 90.0):
        y0 = math.radians(yaw0_deg)
        got = S.yaw_zero(q, y0)

        rz = np.array([[math.cos(-y0), -math.sin(-y0), 0.0],
                       [math.sin(-y0), math.cos(-y0), 0.0],
                       [0.0, 0.0, 1.0]])
        want_mat = rz @ S.quat_to_rot_mat(q)
        want = S.rot_mat_to_quat(want_mat)
        same = np.allclose(got, want, atol=1e-9) or np.allclose(got, -want, atol=1e-9)
        assert same, f"yaw0={yaw0_deg}° 与手写 Rz(−yaw0)⊗q 对不上：{got} vs {want}"


def test_注入只把机头减去_yaw0_倾角不动():
    """★ `yaw_zeroed` 的**契约**：ZYX 分解下等价于「只减 yaw」⟹ roll/pitch 精确不变。

    这条是路线 3「无害」那一支的**前提**：若倾角被动了，整件事就不只是"水平推力方向转了个角"。
    """
    for roll_deg, pitch_deg, yaw_deg in ((0, 0, 20), (10, -5, 20), (-30, 15, -140)):
        q = S.quat_from_euler(np.radians([roll_deg, pitch_deg, yaw_deg]))
        for yaw0_deg in (0.0, 37.0, 90.0):
            got = S.yaw_zero(q, math.radians(yaw0_deg))
            r0, p0, _ = S.euler_from_quat(q)
            r1, p1, y1 = S.euler_from_quat(got)
            assert abs(r1 - r0) < 1e-9 and abs(p1 - p0) < 1e-9, "roll/pitch 必须精确不变"
            assert abs(S.wrap_deg(math.degrees(y1) - (yaw_deg - yaw0_deg))) < 1e-6


# ───────────────────────── ② 闭环：指纹落在机头那一列 ─────────────────────────


def test_闭环在无错配时收敛到参考点且机头不动(rig):
    r = S.run(0.0, rig=rig, seconds=10.0)
    assert r["ok"], f"yaw0 = 0 这一场不该求解失败：{r['fail_reason']}"
    assert r["final_horiz_err_m"] < 0.05, f"无错配时必须收敛，实测 {r['final_horiz_err_m']}"
    assert abs(r["heading_err_deg"]) < 2.0, f"无错配时机头不该偏，实测 {r['heading_err_deg']}"
    assert r["err_pp_m"] < 0.05, f"无错配时不该持续振荡，实测峰峰 {r['err_pp_m']}"


def test_闭环位置误差被吸收_指纹只在机头那一列(rig):
    """★★ 本票的**判别性读数**：位置环吸收得了**位置**，**吸收不了机头**。

    ⚠️ **窗口从 20 s 改成 60 s（2026-09-30，ticket 16 重量 `R_yaw` 之后）**：这一列是
    **一阶慢收敛**，快慢由代价权重 `R_yaw` 定（实测 `τ ≈ 2.05 · R_yaw`）—— 旧窗口是在
    `R_yaw = 50`（"方案 E"，τ ≈ 105 s）下定的，那时 20 s 已经走完 17%；ticket 16 把 `R_yaw`
    退回参考实现的 **500** 之后（τ ≈ 1026 s），20 s 只走完 `1.9%`
    —— 这个 `1.9%` 是**一阶模型** `1 − e^{−20/1026}`；**实测**是 `20°` ⟹ **`0.29°`**（= `1.45%`，
    比模型略慢，两者口径不同别混用）。反正都落在下面 `> 0.5` 的地板底下 ⟹ 窗口跟着放长，
    **判据本身一个字没动**。

    2026-09-30 实测（`R_yaw = 5e2`，60 s 一场）：`yaw0 = 20°` → 位置误差 `0.000000 m`
    但机头已漂 **1.0338°**（0.0172 °/s）；`30°` → 位置 `0.000001 m`、机头 **1.3918°**（0.0232 °/s）。
    ⟹ 机头是朝「参考机头 + `yaw0`」爬，但**一场 60 s 里爬不到** —— 机头那一路的权威很小
    （ticket 16 只把它从"死通道"提到"能爬"，`R_yaw` 那一项**没有**再加大权威）。
    （`R_yaw = 50` 下的历史读数：20 s ⟹ `20°` 3.37° / `30°` 4.79°。）

    ⚠️ 这条**能红**：注入别写错（比如位置也一起转、或干脆没进 `x0`）⟹ 机头这一列塌回 0。
    """
    for yaw0_deg in (20.0, 30.0):
        r = S.run(yaw0_deg, rig=rig, seconds=60.0)
        assert r["ok"], f"yaw0={yaw0_deg}° 求解失败：{r['fail_reason']}"
        assert r["final_horiz_err_m"] < 0.05, (
            f"位置环吸收得了这个错配 —— 位置那一列不该有指纹，实测 {r['final_horiz_err_m']} m")
        assert 0.5 < r["heading_err_deg"] < yaw0_deg, (
            f"yaw0={yaw0_deg}° 时机头应朝「参考 + {yaw0_deg}°」爬、但在 60 s 里爬不到；"
            f"实测 {r['heading_err_deg']}°（速率 {r['yaw_rate_dps']} °/s）")
        assert r["yaw_rate_dps"] > 0.0, "机头该是朝正方向爬，不是反的"


def test_机头收敛时间常数与_yaw0_无关(rig):
    """★★ **机理判据（最锐的一条）**：机头朝 `参考 + yaw0` 的收敛是**一阶**的，且 **τ 与 `yaw0` 无关**。

    一阶 `h(t) = yaw0·(1 − e^{−t/τ})` ⟹ 由 `t1`、`t2` 两点解出
    `τ = (t2 − t1) / ln( ((1 − h1/yaw0) / (1 − h2/yaw0)) )`。

    ⚠️ **判带随 `R_yaw` 走（2026-09-30，ticket 16 之后改）**：实测 `τ ≈ 2.05 · R_yaw`
    —— `R_yaw = 50`（"方案 E"）⟹ τ ≈ 105 s，本判带原为 `80 < τ < 130`；ticket 16 把 `R_yaw`
    退回参考实现的 **500** ⟹ τ ≈ 1026 s，判带改为 `800 < τ < 1300`。**两级都实测过**，不是外推。

    2026-09-30 实测（`R_yaw = 5e2`）：两点式（`t1 = 30 s`、`t2 = 60 s`）给出
    `yaw0 = 10°` ⟹ **τ = 1026.0 s**（`h(30) = 0.2536°`、末拍 `0.5344°`）、
    `yaw0 = 30°` ⟹ **τ = 1064.6 s**（`0.5742°` / `1.3919°`）；末段水平误差 **0.000000 m**（位置始终被吸收）。
    `R_yaw = 50` 下的历史读数：`yaw0 = 10°` 跑 400 s 逐点拟合 τ = **109.4 s(t=10) → 102.6(400)**，
    末拍机头 `9.7974°` = 97.97% · yaw0；`yaw0 = 30°` 的 120 s 场同样落在 ~104 s。

    ★ **τ 与 `yaw0` 无关**比"机头在爬"锐得多：错配**不改被控对象**，只把机头设定点转了 `yaw0`
      ⟹ 两个 `yaw0` 下 τ 必须相同。机理若被写错（例如以为它改了阻尼），这一条会先红。

    ⚠️ 先断 `0 < h1 < h2 < yaw0`：注入若不生效（`h ≡ 0`）⟹ 这里就红，而不是等到后面除零。
    """
    def tau_of(yaw0_deg: float, t1: float = 30.0, t2: float = 60.0) -> float:
        r = S.run(yaw0_deg, rig=rig, seconds=t2)
        assert r["ok"], f"yaw0={yaw0_deg}° 求解失败：{r['fail_reason']}"
        t = np.asarray(r["t"], dtype=float)
        h = np.asarray(r["yaw_series_deg"], dtype=float)
        h1, h2 = float(h[np.argmin(np.abs(t - t1))]), float(h[-1])
        assert 0.0 < h1 < h2 < yaw0_deg, (
            f"yaw0={yaw0_deg}° 时机头该单调爬向 yaw0；实测 h({t1})={h1}° 末段={h2}°")
        r1, r2 = 1.0 - h1 / yaw0_deg, 1.0 - h2 / yaw0_deg
        return (t2 - t1) / math.log(r1 / r2)

    tau_a, tau_b = tau_of(10.0), tau_of(30.0)
    #: 判带 = `2.05 · R_yaw` 附近（`R_yaw = 5e2`（当前默认）⟹ ~1026 s；`R_yaw = 50` ⟹ ~105 s）。
    #: ⚠️ 它**不是**"越接近越对"的目标值 —— `τ` 是**权重的结果**，换 `R_yaw` 就该整条带跟着动。
    for yaw0_deg, tau in ((10.0, tau_a), (30.0, tau_b)):
        assert 800.0 < tau < 1300.0, (
            f"yaw0={yaw0_deg}° 的机头时间常数实测 τ={tau:.1f} s，不在 ~1026 s（= 2.05 · R_yaw(5e2)）附近")
    assert abs(tau_a - tau_b) < 0.25 * (tau_a + tau_b) / 2.0, (
        f"★ 两个 yaw0 下的 τ 该相同（错配不改对象、只转设定点），实测 {tau_a:.1f} s vs {tau_b:.1f} s")


def test_闭环初段去向被系统性转过_yaw0(rig):
    """★★ **路线 4 要的那条读数**：同一个目标点，错配把**轨迹**（不只是机头）系统性转过去 `yaw0`。

    `move_dir_deg` = 第一拍水平速度的方向（`_metrics` `:307-308`）。位置误差在 **+x** ⟹
    无错配时应指向 **−x**（`180°`）。实测（2026-09-30，20 s 一场，取 `move_dir − 180°`）：

    | `yaw0` | 10° | 20° | 30° | 35° | 45° |
    |---|---|---|---|---|---|
    | 侧移量 | 9.696° | 19.402° | 29.126° | 33.998° | 43.766° |
    | 比值 | 0.970 | 0.970 | 0.971 | 0.971 | 0.973 |

    ⟹ **`≈ 0.97 · yaw0`，比值几乎不随 `yaw0` 漂**。差的那 3% 是位置环在一个采样周期内的拉力
    （★ 这也是一条反证：若致偏机理是别的，这个比值会随 `yaw0` 变）。

    ⚠️ **这条判据的鉴别力**：注入若不生效（`yaw_zero` 写成恒等 / 注入没进 `x0`）⟹ 侧移塌回
      `0°` ⟹ 变红。带宽下界取 `0.9`（允许位置环拉回 10%）、上界取 `1.0`（纯运动学上限）。
    """
    zero = S.run(0.0, rig=rig, seconds=20.0)
    assert abs(S.wrap_deg(zero["move_dir_deg"] - 180.0)) < 0.5, (
        f"无错配时初段必须正对着目标去（180°），实测 {zero['move_dir_deg']}° —— "
        f"否则后面那几档的'侧移'里混进了别的东西")
    for yaw0_deg in (10.0, 20.0, 30.0):
        r = S.run(yaw0_deg, rig=rig, seconds=20.0)
        excess = S.wrap_deg(r["move_dir_deg"] - 180.0)
        assert 0.9 * yaw0_deg < excess < 1.0 * yaw0_deg, (
            f"yaw0={yaw0_deg}° 的初段去向该被转过约 0.97·yaw0，实测转过 {excess:.3f}°")


def test_大_yaw0_下水平环发散_而一致臂不发散(rig):
    """★★ **机理判据**：错配让水平环发散，路线 2 那个"位置也一起转"的修法不会。

    ★★ 后一条断言是本工具**归因成立与否的开关**：`consistent` 臂把整个状态与参考一起转
       `Rz(−yaw0)`，数学上等价于 `yaw0 = 0` ⟹ 它**必须**和无错配一样好。
       它要是也退化，那"错配导致发散"这话就不成立，该先查本文件的环路。

    ⚠️ 表里那三档读数（`0°` 与 `45°`）是本条判据的现状快照；结论（发散/不发散）才是要守的东西。
    """
    small = S.run(0.0, rig=rig, seconds=20.0)
    bad = S.run(45.0, rig=rig, seconds=20.0)
    ctrl = S.run(45.0, rig=rig, mode="consistent", seconds=20.0)

    assert small["err_max_m"] < 0.45, f"无错配时最大偏差只该是初值 0.4 m，实测 {small['err_max_m']}"
    assert bad["err_max_m"] > 1.5, (
        f"yaw0 = 45° 下该发散（实测最大 {bad['err_max_m']} m）—— 没发散说明这条实验没造出错配")
    assert bad["err_pp_m"] > 0.3, "发散那一档的末段不该平"
    assert ctrl["err_max_m"] < 0.45 and ctrl["err_pp_m"] < 0.05, (
        f"★ 一致臂（路线 2）在 45° 下退化成了 max={ctrl['err_max_m']} pp={ctrl['err_pp_m']} —— "
        f"它数学上等价于无错配，退化说明归因不成立，先查环路")
    assert abs(ctrl["heading_err_deg"]) < 1.0, "一致臂的机头该对着参考，不该跟着 yaw0 漂"


def test_对照臂在_yaw0_为零时与错配臂完全相同(rig):
    """★★ **仪器自检**：`yaw0 = 0` 时两条臂在数学上是同一条（`Rz(0) = I`）⟹ 读数必须一致。

    ★ 故意把一场**会跑飞的**（`yaw0 = 60°`）夹在中间 —— 这条测试真正要抓的是
      **求解器热启动跨场污染**：本会话实测过一次，90° 跑飞之后紧跟的那一场机头稳到 −24.70°，
      而单独跑是 0.00°。污染**不报错**，只把读数改掉 ⟹ 只有这条判据看得见它。
    """
    S.run(60.0, rig=rig, seconds=6.0)                       # 先弄脏求解器
    a = S.run(0.0, rig=rig, mode="mismatch", seconds=8.0)
    b = S.run(0.0, rig=rig, mode="consistent", seconds=8.0)
    for k in ("final_horiz_err_m", "heading_err_deg", "err_pp_m", "err_max_m"):
        assert abs(float(a[k]) - float(b[k])) < 1e-9, (
            f"`yaw0=0` 两臂必须逐位一致，{k}: {a[k]} vs {b[k]} —— "
            f"不一致说明求解器带着上一场的状态进来了")


def test_早退不静默_跑不满窗口时如实报(rig):
    """跑不满判定窗口必须带得出场：`ok=False` + **非空**原因，不是"跑了一半当成功"。"""
    r = S.run(0.0, rig=rig, seconds=0.02)         # 50 Hz ⟹ 只跑 1 拍
    assert r["ok"] is False, "只跑 1 拍不能算跑成"
    assert r["fail_reason"], "未达时必须带原因，不能是空串 —— 空串会在打印时读成'没问题'"
    assert math.isnan(r["heading_err_deg"]), "未跑成时读数必须是 NaN，不能留一个像真的一样的数"


# ───────────────────────── ③ 工具自身的两条恒等式 ─────────────────────────


def test_旋转矩阵与_nmpc_model_同源():
    """★ 本文件那个 numpy 旋转矩阵**别自己发明一套** —— 与 `nmpc.model.q_to_rot_mat` 逐元素对账。

    两份约定一旦漂开，`rot_mat_to_quat` 那条对账测试就变成"自己跟自己对"，抓不出错。
    """
    import casadi as cs
    from nmpc.model import q_to_rot_mat

    sym = cs.MX.sym("q", 4)
    fn = cs.Function("rot", [sym], [q_to_rot_mat(sym)])
    for rpy in ([0, 0, 0], [10, -5, 20], [-30, 15, -140]):
        q = S.quat_from_euler(np.radians(rpy))
        assert np.allclose(fn(q).full(), S.quat_to_rot_mat(q), atol=1e-12), f"rpy={rpy}"


def test_四元数小工具自洽():
    q = S.quat_from_euler(np.radians([12.0, -7.0, 33.0]))
    assert np.allclose(S.quat_to_rot_mat(q) @ S.quat_to_rot_mat(q).T, np.eye(3), atol=1e-12)
    back = S.rot_mat_to_quat(S.quat_to_rot_mat(q))
    assert np.allclose(back, q, atol=1e-9) or np.allclose(back, -q, atol=1e-9)
    # 结合律 `(a⊗b)⊗c = a⊗(b⊗c)` —— 分量位置写错时它会红
    #（⚠️ 别写成 `quat_mul(a,b) == quat_mul(a,b)`：那是自比，永远真、零鉴别力）
    a = S.quat_from_yaw(0.3)
    b = S.quat_from_euler(np.radians([10.0, 20.0, 30.0]))
    c = S.quat_from_euler(np.radians([-25.0, 40.0, -60.0]))
    assert np.allclose(S.quat_mul(S.quat_mul(a, b), c),
                       S.quat_mul(a, S.quat_mul(b, c)), atol=1e-12)
    # 同态式：`R(a⊗b) = R(a)·R(b)`（换一套写法抓"分量次序写反"这类错）
    assert np.allclose(S.quat_to_rot_mat(S.quat_mul(a, b)),
                       S.quat_to_rot_mat(a) @ S.quat_to_rot_mat(b), atol=1e-12)


def test_机头提取与_node_core_同源():
    """别自己再写一份 `yaw` —— 与 `node_core.yaw_from_quat` 对账。"""
    for yaw_deg in (-170.0, -90.0, 0.0, 37.0, 179.0):
        q = S.quat_from_yaw(math.radians(yaw_deg))
        assert abs(S.wrap_deg(math.degrees(yaw_from_quat(q)) - yaw_deg)) < 1e-9
