"""`analyze_vertical_oscillation.py` 的离线单测（ticket 07）—— 不读归档、不起 SITL、不要 acados。

**为什么值得单测**：这个工具给出的结论（"阻尼项没有输入"）**不是一条飞行读数，是一条分析**——
分析错了不会报错，只会安静地把一句错话写进票面。工具里能出错的只有那几件**口径**，
而口径是可以拿人造数钉死的。★ 更要紧的是：它**自己带一条自证**（复算判层的两列读数对
归档 `checks[]`）—— 那条自证本身也必须被证明"该红的时候会红"，否则它只是一块装饰。

| 钉住的约定 | 为什么它必须成立 |
|---|---|
| 稳态窗按**真值第 0 列**切末尾 N 秒 | 判层 2026-10-09 栽过一次"两根钟混用"；这里再钉一次 |
| 空集**不是绿灯** | `bad == 0` 在一场都没读到的时候**也成立**（我这次真踩过：工具在 `/tmp` 跑，`EVID` 跟着落空，报告照样印"全部对上 ✔"） |
| `fit_ring` 能**恢复**已知的 `(f, σ)` | 恢复不了，后面所有 `D_eff` 都是编的 |
| `back_out_D` 与 `poles` **往返一致** | 它是"实测衰减率 → 等效阻尼"的唯一桥梁 |
| 延时在**没接线**的模型里把衰减率**单调压低** | 这是假设① 判"证伪"的方向依据；方向反了结论就反了 |
| 扫不到逐帧日志要出 `None`，**不是 `False`** | `False` = "查到有非零速度"，那是另一句话（三分法） |
| `ring_window` 切掉爬升段 | 不切，"模态"量到的是起飞那一下 |
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analyze_vertical_oscillation as A  # noqa: E402


# ────────────────────────────────────────────── 判层口径的复算

def test_稳态窗按真值自己的钟切末尾_n_秒():
    """窗口用真值第 0 列（流自己的时基），**不是**墙钟（第 7 列）。

    ★ 这一份人造数据里墙钟**中途冻住**（记录器卡了 5 s）—— 判层 2026-10-09 栽的就是
      "两根钟混用"。若实现读错列，窗口起点会从第 79 点滑到第 30 点（跨度也对不上）。
      ★ 两列只做平移是**区分不开**的（都按 `t_end − N` 切，平移会被抵消）⟹ 必须让它非线性。
    """
    rows = [[i * 0.1, 0.0, 0.0, i * 0.1, 0, 0, 0, min(i, 50) * 0.1] for i in range(100)]
    hov = A.judge_window(rows, steady_s=2.0)
    assert len(hov) == 21                                # 9.9 − 2.0 之后的第 79…99 点
    assert hov[0][A.T_TREL] == pytest.approx(7.9)        # 按真值时基切
    assert hov[0][A.T_WALL] == pytest.approx(5.0)        # 墙钟那一列早已冻住 ⟹ 证明没用它


def test_穿均值频率与判层公式同形():
    """`crossings/2/span`：16 点全交替 = **15 次变号** ⟹ 15/2/4.0 s = 1.875 Hz。

    ★ 钉的是判层 `crossings_series` 那个**具体形式**：相邻变号数 ÷ 2 ÷ **传进来的跨度**
      （不是由点数推的）。★ 我第一版把它记成"8 个周期就是 1 Hz"——分母/系数整整错一倍，
      这种错法不会报错，只会让后面每个频率读数都小一半。
    """
    assert A.osc_hz([1.0, -1.0] * 8, 4.0) == pytest.approx(1.875)
    assert A.osc_hz([1.0, -1.0] * 8, 8.0) == pytest.approx(0.9375)   # 跨度翻倍 ⟹ 读数减半
    assert A.osc_hz([1.0, -1.0] * 3, 4.0) is None       # < 8 点 ⟹ 判不了，不猜


def test_归档读数从_checks_文字里抠得出来():
    s = {"checks": [{"name": "★判据①·振荡 无持续振荡：…",
                     "detail": "末段 5797 点 / 8.00s：高度峰峰值 0.2423 m、穿均值 0.31 Hz；"}]}
    got = A.archived_osc_reading(s)
    assert got["z_p2p_m"] == pytest.approx(0.2423)
    assert got["osc_hz"] == pytest.approx(0.31)
    assert got["n_truth"] == 5797


# ────────────────────────────────────────────── 末段振铃

def _ring(t0=0.0, span=12.0, hz=0.30, sigma=0.20, amp=0.10, drift=0.01, fs=100.0):
    t = np.arange(0.0, span, 1.0 / fs)
    z = amp * np.exp(-sigma * t) * np.cos(2 * math.pi * hz * t) + drift * t
    return t + t0, z


def test_拟合能把已知的_f_和_sigma_恢复回来():
    t, z = _ring()
    fit = A.fit_ring(t, z)
    assert fit is not None
    assert fit["f_hz"] == pytest.approx(0.30, abs=0.02)
    assert fit["sigma_decay_per_s"] == pytest.approx(0.20, abs=0.03)
    assert fit["zeta_eff"] == pytest.approx(0.20 / math.hypot(0.20, 2 * math.pi * 0.30), abs=0.02)


def test_不是衰减正弦的一段要么振幅为零要么落在网格边界():
    """真数据上工具拒收的两类：纯直线（F3/F4「其实在掉」、T0「早就静了」）与跑到栅格外的解。

    ★ 我第一版把这条写成"纯直线必须 `at_grid_edge`"——**错了**。直线的残差在整个 `(f, σ)`
      网格上都是 0，argmin 落在哪是**平局随便挑**（这次挑在 f=0.459、amp0≈4e-19），
      落在内部并不代表"拟合到了东西"。挡"把直线读成振铃"的是**振幅**这一列；
      边界标记管的是另一类（拟合想跑出栅格）。
    """
    t = np.arange(0.0, 12.0, 0.01)

    line = A.fit_ring(t, 0.001 * t)
    assert line is not None
    assert abs(line["amp0"]) < 1e-9, "纯直线不许被当成一段衰减正弦（振幅该 ≈ 0）"

    fast = A.fit_ring(t, 0.05 * np.exp(-2.0 * t))          # 衰减比栅格上限 0.6 /s 还快
    assert fast is not None and fast["at_grid_edge"] is True

    low = A.fit_ring(t, 0.05 * np.cos(2 * math.pi * 0.05 * t))   # 频率在栅格下限 0.12 Hz 之下
    assert low is not None and low["at_grid_edge"] is True


def test_振铃窗会切掉爬升暂态():
    """先爬到 1.0 m，再从 12 s 起慢慢振 —— 窗起点必须在爬升之后。"""
    t = np.arange(0.0, 30.0, 0.01)
    z = np.zeros_like(t)
    z[t >= 10] = 1.0 + 0.15 * np.exp(-0.2 * (t[t >= 10] - 10.0)) * np.cos(
        2 * math.pi * 0.3 * (t[t >= 10] - 10.0))
    z[t < 10] = np.interp(t[t < 10], [0, 8, 10], [0.0, 1.75, 0.72])   # 爬升 + 过冲
    rows = [[t[i], 0.0, 0.0, z[i], 0, 0, 0, 1e6 + t[i]] for i in range(len(t))]
    rw = A.ring_window(rows, ring_s=20.0, tol=0.25)
    assert rw is not None
    assert rw["t0"] > 10.0, "窗起点必须落在爬升/过冲之后（过冲尾巴到 t≈10 才收进 ±0.25）"
    # ★ 别只断言一个我随手挑的数：这条要证明的是"窗里没有爬升段"。
    #   爬升段 z 从 0 爬到 1.75，只要它有一截留在窗里，跨度就会接近 1.75。
    assert rw["t"][0] > 8.0, "窗口不许回吞到爬升段里"
    assert rw["z"].max() - rw["z"].min() < 0.6, "窗里还装着爬升（z 跨度接近 1.75）⟹ 模态量到的是起飞那一下"


# ────────────────────────────────────────────── 模型侧：极点、反推、延时

def test_逐帧速度块三分法_恒零_有非零_没日志():
    def frames(vel):
        return [{"tick": i, "x0": [0.0, 0.0, 1.0, *vel, 1.0, 0.0, 0.0, 0.0]} for i in range(20)]
    got = A.wiring_audit([("zero", {"frames": frames((0.0, 0.0, 0.0))}),
                          ("some", {"frames": frames((0.01, 0.0, 0.0))}),
                          ("none", {"frames": []})])
    assert [g["all_zero"] for g in got] == [True, False, None]


def test_反推阻尼与极点往返一致():
    """给一个 D 算衰减率，再反推回 D —— 必须回到原值。"""
    for m, D in ((0.75, 0.88), (0.975, 0.57)):
        decay = -A.poles(m, D).real
        back = A.back_out_D(m, decay)
        assert back == pytest.approx(D, abs=1e-3)


def test_没接线时延时把复根继续往右推():
    """假设① 判"证伪"的方向依据：衰减率随 τ **单调不升** ⟹ 延时是放大器。"""
    for m in (0.75, 0.975):
        dec = [d["decay_per_s"] for d in A.delay_roots(m)]
        assert all(b <= a + 1e-9 for a, b in zip(dec, dec[1:])), dec
        assert dec[0] < 0.0, "没接线的模型本来就该是不稳的（衰减率为负）"
        assert dec[-1] < dec[0] - 0.1, "100 ms 的延时必须把衰减率压下去一截"


def test_模型的_zeta_与票面同源():
    """`m s³ + D s² + G s + ki = 0` 在 m=0.975、D=2.66、ki=1 上该给出 ζ ≈ 0.72。"""
    p = A.poles(0.975, 2.66, 1.0)
    assert -p.real / abs(p) == pytest.approx(0.72, abs=0.01)
    assert A.D_MODEL * A.G_MODEL / 0.975 == pytest.approx(9.41, abs=0.02)   # 票面那个 Routh 界


# ────────────────────────────────────────────── 两处修过的真 bug（各留一条回归）

def test_真值速度要按秒配对_不是拿纳秒去减秒():
    """回归：`wall_ns`（纳秒 ~1.79e18）曾直接与真值第 7 列（秒 ~1.79e9）相减 ⟹ 差 1e9
    ⟹ 配对恒定失败 ⟹ `vz` 全 `None` ⟹ `u_vs_vz_true` **静默缺席**，而票面当时引了它的数。

    ★ 这条测的就是那个单位：真值里放一个**已知的常数 `vz`**，配对对了就该原值回来。
      单位没对齐时 `df_column_readings` 根本不会产出这个键（`None`），
      ——所以"键在不在"本身就是判据。
    """
    n = 120
    epoch_ns = 1_790_000_000 * 10 ** 9
    frames = [{"tick": i + 1, "wall_ns": epoch_ns + (i + 1) * 10 ** 7} for i in range(n)]
    # 真值行：[t_rel, x, y, z, vx, vy, vz, t_abs] —— t_abs 是**秒**
    truth = [[i * 0.01, 0.0, 0.0, 1.0, 0.0, 0.0, 0.5, 1_790_000_000.0 + i * 0.01]
             for i in range(n)]
    dfcol = {"F0c": {"df_clamp": [-7.3575, 4.4425],
                     "frames": [[i + 1, 0.0, 0.0, 0.0] for i in range(n)]}}
    out = A.df_column_readings(dfcol, [("F0c", {"frames": frames, "truth": truth})],
                               tags=("F0c",))
    s = (out.get("F0c") or {}).get("u_vs_vz_true")
    assert s is not None, "单位没对齐 ⟹ 这条回归静默缺席（这正是那个 bug 的形态）"
    assert s["vz_range"] == pytest.approx([0.5, 0.5]), "真值里的常数 vz 必须原样回来"


def test_不衰减的振铃也要标成落在网格边界():
    """回归：`at_grid_edge` 原先把下界 `σ = 0` 漏掉了 ⟹ 顶在下界的拟合（实测 `T0`）
    被当成"有效拟合"、把 `D_eff` 的下界拉到 0.283（模型的 10.6%）—— 把"没衰减"读成"阻尼很小"。

    ★ 用一个**等幅**振荡最直接：真实衰减率是 0 ⟹ 拟合只能顶在下界上。
    """
    t = np.arange(0.0, 12.0, 0.01)
    fit = A.fit_ring(t, 0.05 * np.cos(2 * math.pi * 0.30 * t))
    assert fit is not None
    assert fit["sigma_decay_per_s"] == pytest.approx(0.0, abs=1e-9)
    assert fit["at_grid_edge"] is True, "σ 顶在下界也是边界，不许当成一个解"


def test_空壳目录里的_summary_要被跳过(tmp_path, monkeypatch):
    """回归：重搭前后**各有一个叫 `F0…F4` 的目录**，旧的那几份 `truth`/`frames` 都是空的
    ⟹ 不跳过就让延时段印出一排 `nan%`，还会让"共几场"这个分母虚高。
    """
    (tmp_path / "20261009-rebuild").mkdir()
    (tmp_path / "F0-summary.json").write_text('{"name": "F0", "truth": [], "frames": []}')
    (tmp_path / "F0b-summary.json").write_text(
        '{"name": "F0b", "truth": [[0, 0, 0, 1, 0, 0, 0, 1.0]], "frames": [{"tick": 1}]}')
    monkeypatch.setattr(A, "EVID", tmp_path)
    got = A.load_summaries()
    assert [t for t, _ in got] == ["F0b"], "空壳（真值与逐帧两样都空）必须被跳过"
