#!/usr/bin/env python3
"""垂直通道"三参数模型说它不该振、它却振了"的**取证工具**（ticket 07，2026-10-09）

**它回答什么**：phase2/20 §13.5 记了一场跨两个被控对象都复现的现象 —— `mass+30` / 标称
两场的末段高度峰峰值都在 0.21…0.24 m，而三参数模型（`u = t_hover + G·e_z − D·v_z`
再加积分器）给出的 `ζ ≈ 0.72` 说它两三秒内就该停。本工具**只从归档读**（不飞、不改
控制器参数、不改任何归档 JSON），把"哪条假设能解释"算成可复核的数。

**两条读数**（互为独立证据）：

1. **速度通路是死的**（直接读数，无拟合）：逐场扫 `frames[].x0[3:6]` —— 喂进求解器的
   速度块**恒为 0**（`mocap_replay.py` 只发 `PoseStamped`，`_mocap_velocity()` 退化成
   `(0,0,0)`）。三参数模型把环路**唯一的阻尼**记在 `−D·v_z` 上 ⟹ `ζ = 0.72` 是
   **记在账上、没接线**的阻尼。
2. **飞行的那条环路是"稳但极欠阻尼"**：把末段"最后 0.3 s 之外、误差落回 ±0.25 m 之后"
   的那一段拟成**衰减正弦 + 一次趋势**（对 f 与 σ 做可分离最小二乘），得到
   `f ≈ 0.28…0.32 Hz`、衰减率 `σ_decay`。判层那两条读数（末 8 s 峰峰值 / 穿均值频率）
   都能从它复算出来 ⟹ **峰峰值是"末窗里还剩多少振铃"，不是极限环**。

**把它俩拼起来**才是机理：模态频率对得上（`√(G/m)`），**衰减率差一个量级**
（模型 1.17 /s vs 飞行 0.07…0.43 /s），而差的那一截正是没接线的 `D`。

**本工具的规矩（同 `sim_vertical_bias_loop.py`）**：
- **先自证再下结论**：它必须复算出判层写进 `checks[]` 的那两条数（高度峰峰值、穿均值频率），
  **对不上就不许往下读**（`selfcheck` 一节会把差异打印出来）；
- 判读口径与判层逐字对齐的地方在注释里点明（`judge_window` / `osc_hz`）。

**它不能回答的**（缺口列在这，别外推）：
- **不给"接上线之后会不会好"的预测** —— 那要改夹具，属另一张票，且是实跑的事；
- **残余阻尼（`D_eff ≈ 0.42…0.97`，模型值的 16…37%）没找到出处**：这一截只能写成"缺什么"，
  不能写成"是什么"。★ 但要把口径写清楚 —— **归档并不是"既没有姿态、也没有速度"**
  （第一版这么写，**是错的**，与本文件 ①/⑥ 两节直接打架）：
  逐帧 `x0[3:6]`（速度**通路**）与 `x0[6:10]`（姿态，100 Hz）都有，`truth` 第 4–6 列还有
  **真值自己的** `vx/vy/vz`。缺的是**另一类**读数：`mocap` 那条 `/mocap/velocity_enu`
  在飞的那条环路上到底有没有被采到（`_mocap_velocity()` 的返回）、以及指令角速率 vs
  实际角速率那一对。前者由 ① 的"恒为 0"间接回答，后者本归档确实没有。

用法（**纯 numpy，不需要 acados / ROS**）：

    python3 tools/px4-sitl/analyze_vertical_oscillation.py
    python3 tools/px4-sitl/analyze_vertical_oscillation.py --json evidence/tier-b/oscillation-mechanism-20261009.json
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

# ── 模型常数（口径来源写在注释里，**不许从别处转抄**）──────────────────────────
#: `du/d e_z` [N/m]，ticket 20 §13.7 实测（旧树 `3.4494650610674324` / 新树
#: `3.449465028748461`，差 3e-8 与结论无关 ⟹ 本工具取旧树值）。
G_MODEL = 3.4494650610674324
#: `−du/d v_z` [N/(m·s)]，ticket 20 实测的 OCP 速度项 —— **三参数模型里环路唯一的阻尼**。
D_MODEL = 2.66
#: 现行积分增益（票 20 裁定 1 的落地值）。
KI_Z = 1.0
#: 判层自己那两条口径（`verify_tier_b_seam3.py`）：稳态窗长 / 峰峰值上界 / 穿均值上界。
STEADY_S = 8.0
OSC_P2P_M = 0.35
P2P_TARGET_M = 0.2            # 假设② 要对比的那个尺度：飞行实际出现的峰峰值量级（§13.5）
OSC_HZ = 2.0
#: 真值行的列号：`[t_rel, x, y, z, vx, vy, vz, wall]`。☠️ `r[2]` 是 y、`r[3]` 才是 z。
T_TREL, T_X, T_Y, T_Z, T_WALL = 0, 1, 2, 3, 7
T_VZ = 6                       # 真值行 [t_rel, x, y, z, vx, vy, vz, t_abs]（ENU，上游直接取消息的 vx/vy/vz）
#: `x0` 的状态向量 = `[x, y, z, vx, vy, vz, qw, qx, qy, qz]` ⟹ 速度块是 `[3:6]`。
X0_VEL = slice(3, 6)

EVID = Path(__file__).resolve().parent / "evidence" / "tier-b"


# ────────────────────────────────────────────── 判层口径的复算（自证用）

def judge_window(truth, steady_s=STEADY_S):
    """判层 `segments(..., hover_s=8.0)` 那条 hover 段**逐字复算**。

    ★ 关键在"用哪个时钟"：这段按**真值流自己的第 0 列**切末尾 8 s（`t_rel >= t_end − 8`），
      **不是**墙钟。判层 2026-10-09 栽的那一处（`_flight_window()` 把记录器钟的窗口喂给
      按绝对墙钟取数的函数）**不在这条路上** ⟹ `osc` 这一列不受那个缺陷影响。
    """
    rows = [r for r in (truth or []) if all(v is not None for v in r)]
    if not rows:
        return []
    t_end = rows[-1][T_TREL]
    return [r for r in rows if r[T_TREL] >= t_end - steady_s]


def osc_hz(vals, span_s):
    """`crossings_series` 逐字复算：穿自己均值的次数 / 2 / 跨度。"""
    if len(vals) < 8 or not span_s or span_s <= 0:
        return None
    mean = float(np.mean(vals))
    sign = [1 if x > mean else -1 for x in vals]
    n = sum(1 for i in range(1, len(sign)) if sign[i] != sign[i - 1])
    return n / 2.0 / span_s


def archived_osc_reading(summary):
    """从归档的 `checks[]` 里把判层当时那两条数**抠出来**（自证的对照物）。

    ★ 为什么要从**字符串**里抠：归档的 `steady` / `osc` 两个 dict 只有频率、没有峰峰值，
      峰峰值只写在判据行的 detail 里。抠它是为了拿"判层当时自己写的数"跟本工具复算的
      数**硬碰** —— 差一个数就说明本工具的口径没对齐，后面所有读数都不许用。
    """
    out = {}
    for c in summary.get("checks") or []:
        d = str(c.get("detail") or "")
        n = str(c.get("name") or "")
        if "振荡" in n or "峰峰值" in d:
            m = re.search(r"高度峰峰值\s*([-+0-9.eE]+)\s*m", d)
            if m:
                out.setdefault("z_p2p_m", float(m.group(1)))
            m = re.search(r"穿均值\s*([-+0-9.eE]+)\s*Hz", d)
            if m:
                out.setdefault("osc_hz", float(m.group(1)))
            m = re.search(r"末段\s*(\d+)\s*点\s*/\s*([-+0-9.eE]+)\s*s", d)
            if m:
                out.setdefault("n_truth", int(m.group(1)))
                out.setdefault("span_s", float(m.group(2)))
    return out


# ────────────────────────────────────────────── 末段振铃：衰减正弦 + 一次趋势

def ring_window(truth, ring_s=12.0, tol=0.25):
    """取"振铃段"：末 `ring_s` 秒内，**最后一次误差超出 ±`tol`** 之后的那一段。

    ★ 判层那个固定 8 s 窗里同时装着**起飞暂态**和振铃（F0c 的窗内 z 从 0.93 走到 1.13，
      而窗口前面 4 s 飞机还在爬）。要把"模态"量准，必须先切掉爬升那一下 ——
      切法写死在这里（而不是手调），任何人跑出来的窗口都一样。
    """
    rows = [r for r in (truth or []) if all(v is not None for v in r)]
    if not rows:
        return None
    t = np.array([r[T_TREL] for r in rows])
    z = np.array([r[T_Z] for r in rows])
    t_end = t[-1]
    z_ref = float(np.median(z[t >= t_end - ring_s]))
    outside = [i for i in range(len(t))
               if abs(z[i] - z_ref) > tol and t[i] > t_end - ring_s - 8.0]
    t0 = (t[outside[-1]] if outside else max(t_end - ring_s, t[0])) + 0.15
    sel = (t >= t0) & (t <= t_end - 0.3)
    if sel.sum() < 40:
        return None
    return {"t": t[sel], "z": z[sel], "t0": float(t0), "z_ref": z_ref,
            "span_s": float(t[sel][-1] - t[sel][0])}


def fit_ring(t, z, f_lo=0.12, f_hi=0.55, s_hi=0.60):
    """`z(t) ≈ a + b·t + A·e^{−σt}·cos(2πft + φ)`，对 `(f, σ)` 网格做**可分离**最小二乘。

    ★ 为什么不用 FFT 峰值：末段的 z 是"振铃叠在还在缓慢移动的均值上"，谱峰会被**二次
      谐波**抢走（本工具第一版就是被抢走了：FFT 报 0.63 Hz，而穿均值频率是 0.31 Hz ——
      正好一倍）。可分离拟合同时给出 `f` 与 `σ`，且残差能直接看拟合好不好。
    ★ 两段网格（粗扫 + 局部细扫）而不是 `curve_fit`：`curve_fit` 对初值敏感而这两场的
      σ 跨一个量级；两段网格既稳又快（细扫范围写死在粗解周围 ±0.02 Hz / ±0.04 /s，
      **不给它跑到别的局部极小去的自由**）。
    """
    t = np.asarray(t, float)
    z = np.asarray(z, float)
    if len(t) < 40:
        return None
    tc = t - t[0]

    def sweep(f_lo, f_hi, s_lo, s_hi, df, ds, seed=None):
        best = seed
        for f in np.arange(f_lo, f_hi + 1e-12, df):
            w = 2 * math.pi * f
            for s in np.arange(max(s_lo, 0.0), s_hi + 1e-12, ds):
                env = np.exp(-s * tc)
                B = np.vstack([np.ones_like(tc), tc, env * np.cos(w * t),
                               env * np.sin(w * t)]).T
                c, *_ = np.linalg.lstsq(B, z, rcond=None)
                rms = float(np.sqrt(np.mean((z - B @ c) ** 2)))
                if best is None or rms < best["rms"]:
                    best = {"rms": rms, "f_hz": float(f), "sigma_decay_per_s": float(s),
                            "coef": [float(v) for v in c],
                            "amp0": float(math.hypot(c[2], c[3]))}
        return best

    best = sweep(f_lo, f_hi, 0.0, s_hi, 0.010, 0.020)          # 粗扫
    best = sweep(max(best["f_hz"] - 0.02, f_lo), min(best["f_hz"] + 0.02, f_hi),
                 best["sigma_decay_per_s"] - 0.04, min(best["sigma_decay_per_s"] + 0.04, s_hi),
                 0.001, 0.002, seed=best)                        # 细扫（围着粗解）
    if best is None:
        return None
    # ★ 落在网格边上的解**不是解**：它只说明这一段不是"衰减正弦"（F3/F4 是注入失效那一档，
    #   飞机在掉而不是在振；T0 是长程轨迹，末段早就静态了）。标出来，后面的统计把它剔掉。
    #   ☠️ 三个边都要标：**下界 `σ = 0` 也是边**。第一版只标了 `σ = s_hi` 与 `f` 的上下界，
    #   于是 `T0`（`σ = 0.0000`，顶在下界）照样算"有效拟合"，把 `D_eff` 的下界拉到 0.283
    #   （模型值的 10.6%）—— 那是"没衰减"被当成了"阻尼很小"。
    best["at_grid_edge"] = bool(abs(best["f_hz"] - f_lo) < 1e-9
                                or abs(best["f_hz"] - f_hi) < 1e-9
                                or abs(best["sigma_decay_per_s"] - s_hi) < 1e-9
                                or abs(best["sigma_decay_per_s"]) < 1e-9)
    om = 2 * math.pi * best["f_hz"]
    best["zeta_eff"] = best["sigma_decay_per_s"] / math.hypot(best["sigma_decay_per_s"], om)
    return best


# ────────────────────────────────────────────── 三参数模型：极点、反推 D、延时

def poles(m, D, ki=KI_Z, G=G_MODEL):
    """`m s³ + D s² + G s + ki = 0` 的复根（取虚部为正的那一个）。"""
    r = np.roots([m, D, G, ki])
    pair = [x for x in r if abs(x.imag) > 1e-9]
    if not pair:
        return None
    return max(pair, key=lambda x: x.imag)


def back_out_D(m, decay_per_s, ki=KI_Z, G=G_MODEL, hi=4.0):
    """给定**实测衰减率**，反推环路里实际存在的等效阻尼 `D_eff`（对 D 二分）。

    ★ 这是本工具的核心读数：模型说 `D = 2.66`，飞行看到的衰减率对应多少？
      两者之比就是"记在账上 vs 接上线"的比例。
    """
    f = lambda D: poles(m, D, ki, G).real + decay_per_s   # σ_decay>0 ⟺ 实部 = −σ_decay
    lo = 0.0
    if f(lo) * f(hi) > 0:
        return None
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if f(lo) * f(mid) <= 0:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def delay_roots(m, G=G_MODEL, ki=KI_Z, taus=(0.0, 0.002, 0.005, 0.010, 0.020, 0.050, 0.100)):
    """带纯延时的特征方程 `m s³ + (G s + ki)·e^{−sτ} = 0` 的复根（沿 τ 延拓）。

    ★ 为什么必须算这一条：假设① 说"50 Hz 采样 + 动捕时延"是机理。在**有阻尼**的模型里
      延时是相位裕度问题；在**没接线**的模型里环路本来就不稳，延时的作用方向得单独量 ——
      量出来是**把不稳的复根继续往右推**（σ_decay 随 τ 单调变小），
      所以延时是**放大器**，不是根因（已飞过的 +50 ms 那一场同向：峰峰值翻倍）。
    """
    r0 = poles(m, 0.0, ki, G)
    s = np.array([r0.real, r0.imag], float)
    out = []
    for tau in taus:
        for _ in range(300):
            sc = complex(s[0], s[1])
            e = np.exp(-sc * tau)
            fv = m * sc ** 3 + (G * sc + ki) * e
            fp = 3 * m * sc ** 2 + G * e - tau * (G * sc + ki) * e
            sc = sc - fv / fp
            s = np.array([sc.real, sc.imag], float)
        out.append({"tau_ms": tau * 1e3, "decay_per_s": float(-s[0]),
                    "f_hz": float(abs(s[1])) / 2 / math.pi})
    return out


# ────────────────────────────────────────────── 归档里"速度有没有接上"的直接证据

def wiring_audit(summaries):
    """逐场扫 `frames[].x0` 的速度块 —— **"速度通路没接"最硬的一手证据**。

    ★ 为什么它比"回归 `u[0]` vs 真值 `v_z`"强：回归要给噪声留缝（斜率 0 也可以是
      "噪声太大看不出"），而这里读的是**喂进求解器的那个数本身**：恒为 0 就是恒为 0，
      没有噪声、没有拟合、没有 δ。
    """
    out = []
    for tag, s in summaries:
        fr = s.get("frames") or []
        mx, n, nz = 0.0, 0, 0
        for f in fr:
            x0 = f.get("x0")
            if not x0 or len(x0) < 6:
                continue
            n += 1
            v = max(abs(x) for x in x0[X0_VEL])
            mx = max(mx, v)
            if v == 0.0:
                nz += 1
        out.append({"tag": tag, "frames_with_x0": n, "max_abs_v_x0": mx,
                    "frames_vel_exactly_zero": nz,
                    "all_zero": (nz == n) if n else None})   # None = 本场没逐帧日志
    return out


# ────────────────────────────────────────────── `df` 列：把误差序列重建出来再回归

def df_column_readings(dfcol, summaries, tags=("F0c", "F1c")):
    """从 `df-column-*.json` 重建 `e_z` 序列，再回归 `u[0]` —— 两条通道各量一次。

    ★ 重建式（来源 `mpc_node._integrate_bias_z`，**先量它的前提再信它**）：

        Δdf = ki_z · e_z · dt  ⟹  e_z = Δdf / (ki_z · dt)

      ⚠️ 三个前提，缺一条这条读数就作废，下面显式判掉：
        (a) `dt` 必须是**节点那一拍的真实 dt**（`_bias_at_s` 之差）⟹ 用 `-summary.json`
            里逐帧的 `wall_ns` 按 tick 配回来，**不能用 1/100 s 顶替**；
        (b) `Δdf` 落在夹取界上的那几拍是**被削过的**，反推的 `e_z` 偏小 ⟹ 丢掉；
        (c) 只有 `fallback=normal` 的帧才积分 ⟹ `df-column` 已经是那一组（文件头写了）。
    """
    out = {}
    wall, truth = {}, {}
    for tag, s in summaries:
        if tag in tags:
            wall[tag] = {f["tick"]: f["wall_ns"] for f in (s.get("frames") or [])
                         if "tick" in f and "wall_ns" in f}
            truth[tag] = sorted(s.get("truth") or [], key=lambda r: r[T_WALL])
    for tag in tags:
        blk = dfcol.get(tag)
        if not blk or not wall.get(tag):
            continue
        lo, hi = blk["df_clamp"]
        w, fr = wall[tag], blk["frames"]
        rows = []
        for i in range(1, len(fr)):
            tk0, df0 = fr[i - 1][0], fr[i - 1][1]
            tk1, df1 = fr[i][0], fr[i][1]
            if tk0 not in w or tk1 not in w:
                continue
            dt = (w[tk1] - w[tk0]) / 1e9
            if not (1e-4 < dt < 0.1):
                continue
            eps = 1e-9                                        # (b) 贴夹取界的拍丢掉
            if min(abs(df1 - lo), abs(df1 - hi), abs(df0 - lo), abs(df0 - hi)) < eps:
                continue
            rows.append({"tick": tk1, "wall": w[tk1], "u0": fr[i][2],
                         "e_df": (df1 - df0) / (KI_Z * dt)})
        if len(rows) < 60:
            continue
        # ★ 真值的垂直速度**直接取真值自己的第 6 列**（ENU `vz`）：上游 `enu_rows()`
        #   就是从位姿消息里取 `vx/vy/vz`，并**明确写了"不另做差分"**（差分会把"速度"
        #   变成另一个量，A/B 两档才同形）。
        #   ☠️ 我第一版在这里写的是"真值第 4/5/6 列恒为 0 ⟹ 只能自己差分"——**那句话是错的**：
        #     这批归档里第 6 列最大到 1.8 m/s。错因是**单位没对齐**——拿真值第 7 列
        #     （`t_abs`，**秒**，~1.79e9）去配帧的 `wall_ns`（**纳秒**，~1.79e18），
        #     `searchsorted` 恒找不到 ⟹ `vz` 全 `None` ⟹ 下面那个回归**一次都没跑出来**，
        #     而票面当时已经引用了它的数字。教训与 ① 那条同族：**旁证字段是 `None` 时，
        #     不许当成"通道是死的"的读数**（`None` 是"没量到"，不是"量到 0"）。
        tr = truth.get(tag) or []
        tw = [r[T_WALL] for r in tr]
        vz = [None] * len(rows)
        for i, r in enumerate(rows):
            ts = r["wall"] / 1e9                      # wall_ns ⟹ 秒（与真值第 7 列同口径）
            j = min(max(int(np.searchsorted(tw, ts)), 1), len(tr) - 1)
            a, b = tr[j - 1], tr[j]
            if b[T_WALL] <= a[T_WALL] or not (a[T_WALL] - 0.05 <= ts <= b[T_WALL] + 0.05):
                continue
            k = (ts - a[T_WALL]) / (b[T_WALL] - a[T_WALL])
            vz[i] = a[T_VZ] + k * (b[T_VZ] - a[T_VZ])

        def lin(x, y):
            A = np.vstack([x, np.ones_like(x)]).T
            (k, c), *_ = np.linalg.lstsq(A, y, rcond=None)
            res = y - (k * x + c)
            se = float(np.sqrt(np.sum(res ** 2) / max(len(x) - 2, 1) /
                               max(np.sum((x - x.mean()) ** 2), 1e-30)))
            return float(k), se, float(np.std(res))

        res = {"n_pairs": len(rows), "df_clamp": [lo, hi], "u_vs_e_bands": []}
        for a, b in ((0.0, 0.05), (0.05, 0.15), (0.15, 0.40), (0.40, 1.20)):
            sel = [r for r in rows if a <= abs(r["e_df"]) < b]
            if len(sel) < 30:
                res["u_vs_e_bands"].append({"lo": a, "hi": b, "n": len(sel), "slope": None})
                continue
            k, se, sd = lin(np.array([r["e_df"] for r in sel]),
                            np.array([r["u0"] for r in sel]))
            res["u_vs_e_bands"].append({"lo": a, "hi": b, "n": len(sel), "slope": k,
                                        "slope_se": se, "resid_std": sd})
        sel = [(i, r) for i, r in enumerate(rows) if vz[i] is not None and abs(r["e_df"]) < 0.30]
        if len(sel) >= 30:
            x = np.array([vz[i] for i, _ in sel])
            y = np.array([r["u0"] for _, r in sel])
            k, se, sd = lin(x, y)
            res["u_vs_vz_true"] = {"n": len(sel), "slope": k, "slope_se": se, "resid_std": sd,
                                   "model_minus_D": -D_MODEL,
                                   "vz_range": [float(x.min()), float(x.max())],
                                   "signal_if_wired_N": float(abs(D_MODEL) * (x.max() - x.min()))}
        out[tag] = res
    return out


# ────────────────────────────────────────────── 假设②：姿态耦合到底能偷多少推力

def tilt_trace(summary, margin_s=0.3, span_s=12.0, band=(0.20, 0.45)):
    """逐帧姿态（`x0[6:10]` 的四元数，100 Hz）⟹ 倾角序列与其在模态频带里的幅度。

    ★ 为什么这一条能把假设② 判死而不是"取不到那个量"：归档虽然**没有**独立的姿态真值流
      （`truth` 只有位置那 8 列），但**每一帧都带着动捕姿态**（`x0` 后 4 位）。
      于是"姿态耦合能偷走多少垂直推力"可以当场算：推力沿机体系 z，抬起来 θ 就只剩
      `T·cos θ`，偷走 `(1−cos θ)·T`。把它和"要造成 0.2 m 的峰峰值所需的那 ~`G·0.2` N"
      比一下，就知道这条通道够不够。
    """
    fr = [f for f in (summary.get("frames") or []) if f.get("fallback") == "normal"
          and f.get("x0") and len(f["x0"]) >= 10]
    if len(fr) < 60:
        return None
    t = np.array([f["wall_ns"] for f in fr], float) / 1e9
    t = t - t[0]
    th = np.array([_tilt_deg(f["x0"][6:10]) for f in fr])
    sel = (t >= t[-1] - span_s) & (t <= t[-1] - margin_s)
    if sel.sum() < 60:
        return None
    tt, hh = t[sel], th[sel]
    hh = hh - np.polyval(np.polyfit(tt, hh, 1), tt)
    win = np.hanning(len(hh))
    sp = np.abs(np.fft.rfft(win * hh)) * 2.0 / float(np.sum(win))
    fr_ = np.fft.rfftfreq(len(hh), float(np.median(np.diff(tt))))
    m = (fr_ > band[0]) & (fr_ < band[1])
    amp = float(np.max(sp[m])) if m.any() else 0.0
    # 偷走的推力 `(1−cos θ)·T` 在**末段窗内**的**变化量** —— 这才是"能不能造成振荡"的那个数
    steal = 1.0 - np.cos(np.radians(th[sel]))
    # ★ 偷走的那份推力按**这一场自己的悬停推力**折算 —— 不能拿标称机（0.75 kg ⟹ 7.36 N）
    #   的常数套在 `mass+30`（`t_hover_n` = 9.56 N）上：那会**低报 30%**。
    #   来源：`-summary.json` 的 `plant_hover.t_hover_n`（判层算稳态用的同一个数）。
    T = (summary.get("plant_hover") or {}).get("t_hover_n")
    return {"n_frames": len(fr), "tilt_mean_deg": float(np.mean(th)),
            "tilt_p2p_deg": float(np.ptp(th)), "tail_detrend_p2p_deg": float(np.ptp(hh)),
            "band_amp_deg": amp, "band_hz": list(band), "thrust_ref_n": T,
            "force_steal_tail_mN": (None if T is None
                                    else float((steal.max() - steal.min()) * T * 1000)),
            "force_steal_tail_abs_mN": None if T is None else float(steal.max() * T * 1000),
            "force_stolen_mN_full_p2p": (None if T is None else
                                         float((1 - math.cos(math.radians(np.max(np.abs(th)))))
                                               * T * 1000)),
            "force_needed_for_p2p_N": float(G_MODEL * P2P_TARGET_M),
            "force_needed_for_p2p_m": P2P_TARGET_M}


def _tilt_deg(q):
    """四元数 `(w, x, y, z)` 下机体系 z 轴与竖直的夹角 [°]。"""
    w, x, y, z = [float(v) for v in q]
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    up = 1.0 - 2.0 * (x * x + y * y)          # (R·[0,0,1])_z
    return math.degrees(math.acos(max(-1.0, min(1.0, up))))


# ────────────────────────────────────────────── 主流程

def collect(summaries, ring_s=12.0, tol=0.25, steady_s=STEADY_S):
    rows = []
    for tag, s in summaries:
        tr = s.get("truth") or []
        hov = judge_window(tr, steady_s)
        if len(hov) < 32:
            continue
        zs = [r[T_Z] for r in hov]
        ts = [r[T_TREL] for r in hov]
        p2p = float(max(zs) - min(zs))
        ohz = osc_hz(zs, ts[-1] - ts[0])
        arc = archived_osc_reading(s)
        mass = (s.get("plant_hover") or {}).get("mass_kg")
        rw = ring_window(tr, ring_s, tol)
        fit = fit_ring(rw["t"], rw["z"]) if rw else None
        rec = {
            "tag": tag, "mass_kg": mass, "plant": s.get("plant"),
            "extra_delay_ms": s.get("extra_delay_ms"), "truth_hz": s.get("truth_hz"),
            "steady_n": len(hov), "steady_span_s": round(ts[-1] - ts[0], 3),
            "z_p2p_m": p2p, "z_mean_m": float(np.mean(zs)),
            "osc_hz_judge": ohz, "osc_p2p_pass": bool(p2p < OSC_P2P_M and (ohz or 1e9) < OSC_HZ),
            "archived": arc,
            "selfcheck_p2p_delta": (p2p - arc["z_p2p_m"]) if "z_p2p_m" in arc else None,
            "selfcheck_osc_delta": (ohz - arc["osc_hz"]) if ("osc_hz" in arc and ohz) else None,
            "ring": ({"t0": rw["t0"], "span_s": rw["span_s"], "z_ref": rw["z_ref"]} if rw else None),
            "fit": fit,
        }
        if mass and fit and not fit["at_grid_edge"]:
            rec["model_hz"] = math.sqrt(G_MODEL / mass) / 2 / math.pi
            rec["D_eff"] = back_out_D(mass, fit["sigma_decay_per_s"])
            rec["D_eff_frac_of_model"] = (rec["D_eff"] / D_MODEL) if rec["D_eff"] else None
            p = poles(mass, D_MODEL)
            rec["model_decay_per_s"] = float(-p.real)
            rec["model_zeta"] = float(-p.real / abs(p))
            rec["model_hz_pole"] = float(abs(p.imag)) / 2 / math.pi
            rec["decay_slowdown"] = (rec["model_decay_per_s"] / fit["sigma_decay_per_s"]
                                     if fit["sigma_decay_per_s"] > 0 else None)
            rec["model_ki_bound"] = float(D_MODEL * G_MODEL / mass)
            if rec["D_eff"]:
                rec["actual_ki_bound"] = float(rec["D_eff"] * G_MODEL / mass)
        rows.append(rec)
    return rows


def load_summaries(include_rebuild=True):
    """归档里所有带真值流的场次。

    ★ 两个批次用**目录名**做前缀（`20261009-rebuild/F0`）—— 目录名本身就是可 grep 回去的
      路径。☠️ 第一版用的是硬编码的 `2019/`：既不是年份也不是任何东西，纯属手滑，
      而且**重搭前后各有一个叫 `F0` 的目录**（旧的那份 `truth`/`frames` 都是空的），
      光写 `F0` 会指到空壳上去。
    """
    out = []
    for p in sorted(EVID.glob("*-summary.json")):
        out.append((p.name.replace("-summary.json", ""), json.loads(p.read_text())))
    if include_rebuild:
        d = EVID / "20261009-rebuild"
        for p in sorted(d.glob("*-summary.json")):
            out.append((f"{d.name}/{p.name.replace('-summary.json', '')}",
                        json.loads(p.read_text())))
    # ★ 归档里有一批**空壳**：重搭前后各有一个叫 `F0…F4` 的 `-summary.json`，
    #   旧的那几个 `truth` 与 `frames` **都是空的**（真数据在 `20261009-rebuild/` 下）。
    #   留着它们只会让延时段印出一排 `nan%`。真值、逐帧**两样都空**才算空壳 ——
    #   只有一样空可能是"这场没记姿态"，那是有信息的。
    return [(t, s) for t, s in out if (s.get("truth") or s.get("frames"))]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--steady-s", type=float, default=STEADY_S)
    ap.add_argument("--ring-s", type=float, default=12.0, help="振铃段最长取多少秒")
    ap.add_argument("--ring-tol", type=float, default=0.25, help="判「还在暂态」的误差界 [m]")
    ap.add_argument("--no-rebuild", action="store_true", help="只看重搭（票 04）之前那一批")
    ap.add_argument("--json", default="", help="把读数落成 JSON（默认只打印）")
    args = ap.parse_args(argv)

    steady_s = args.steady_s
    summaries = load_summaries(not args.no_rebuild)

    print("# 口径：G = %.6f N/m、D(模型) = %.2f N/(m/s)、ki_z = %.2f" % (G_MODEL, D_MODEL, KI_Z))
    print("# 判层同款：稳态窗 = 真值流末 %.1f s（按真值第 0 列）、峰峰值 < %.2f m 且穿均值 < %.1f Hz"
          % (steady_s, OSC_P2P_M, OSC_HZ))
    print("# ⚠️ 纯离线读归档：不飞、不改控制器、不改任何归档 JSON。\n")

    wiring = wiring_audit(summaries)
    print("=== ① 直接证据：喂进求解器的速度块（`frames[].x0[3:6]`）===")
    for w in wiring:
        if w["all_zero"] is None:
            print(f"  {w['tag']:<12} 无逐帧日志 ⟹ 这一条**本场不适用**（不是通过也不是不通过）")
        else:
            print(f"  {w['tag']:<12} 有 x0 的帧 {w['frames_with_x0']:>5}  "
                  f"max|v| = {w['max_abs_v_x0']:.3e} m/s  速度恰好为 0 的帧 "
                  f"{w['frames_vel_exactly_zero']:>5}  {'恒零 ✔' if w['all_zero'] else '有非零 ✘'}")
    ok = [w for w in wiring if w["all_zero"]]
    tot = [w for w in wiring if w["all_zero"] is not None]
    nz = sum(w["frames_with_x0"] for w in ok)
    print(f"  ⟹ {len(ok)}/{len(tot)} 场（共 {nz} 帧）速度块恒为 0")
    if ok and len(ok) == len(tot):
        print("     ⟹ 三参数模型里的 `−D·v_z` 那一项**没有输入**，`ζ = 0.72` 是没接线的阻尼。\n")
    else:
        print("     ☠️ **本场不是「全部恒零」** ⟹ 上面这条结论不成立，别引用它。\n")

    rows = collect(summaries, args.ring_s, args.ring_tol, steady_s)
    if not rows:
        # ★★ 空集**不是绿灯**：下面那个"全部对上"是 `bad == 0`，一场都没读到也会满足。
        #    （2026-10-09 我自己踩过：把工具拷到 `/tmp` 去跑变异测试，`EVID` 跟着落到
        #    `/tmp/evidence/tier-b` ⟹ 一场都读不到，而报告照样印"全部对上 ✔"。）
        print(f"☠️ 一场都没读到（找的是 {EVID}）⟹ 自证没有对照物，"
              "**这次什么都没量到，这不是通过（空集合不许读成绿）**", file=sys.stderr)
        return 2
    print("=== ② 自证：本工具复算的判层读数 vs 归档 `checks[]` 里判层当时写的 ===")
    # ★ 两列**都要接闸**：峰峰值对到归档印出的 4 位（容差 5e-4），穿均值对到它印出的
    #   2 位（归档写"0.31 Hz" ⟹ 可还原的精度只到 ±0.005 Hz，容差取 0.0051）。
    #   ☠️ 第一版只把峰峰值纳入判据，频率那一列**只印不判** —— 那等于在绿勾旁边摆一列
    #   没接闸的读数（频率真跑偏了，照样印"全部对上 ✔"）。
    bad = 0
    for r in rows:
        if r["selfcheck_p2p_delta"] is None:
            continue
        d1, d2 = r["selfcheck_p2p_delta"], r["selfcheck_osc_delta"]
        ok1 = abs(d1) < 5e-4 + 1e-9
        ah = r["archived"].get("osc_hz")
        ok2 = ah is None or (d2 is not None and abs(d2) < 5.1e-3 + 1e-9)
        flag = "✔" if (ok1 and ok2) else "✘"
        if flag == "✘":
            bad += 1
        print(f"  {r['tag']:<12} 峰峰值 {r['z_p2p_m']:.4f} vs 归档 {r['archived']['z_p2p_m']:.4f}"
              f"（差 {d1:+.6f}）{'✔' if ok1 else '✘'}   穿均值 "
              f"{'—' if r['osc_hz_judge'] is None else format(r['osc_hz_judge'], '.4f')} vs 归档 "
              f"{r['archived'].get('osc_hz', float('nan')):.4f}"
              f"（差 {'—' if d2 is None else format(d2, '+.4f')}）{'✔' if ok2 else '✘'}"
              f"（差 {'—' if d2 is None else format(d2, '+.6f')}）")
    print(f"  ⟹ {'全部对上 ✔ 口径已对齐' if bad == 0 else f'{bad} 场对不上 ✘ 下面的读数不许用'}\n")

    print("=== ③ 模态与衰减（末段振铃拟成衰减正弦 + 一次趋势）===")
    print(f"  {'tag':<12}{'m/kg':>6}{'末8s p2p':>10}{'穿均值':>8}{'振铃窗':>8}{'f实测':>8}"
          f"{'√(G/m)':>8}{'σ实测':>8}{'ζ_eff':>7}{'模型σ':>8}{'慢几倍':>8}{'D_eff':>7}{'占模型':>7}")
    NAN = float("nan")
    for r in rows:
        f = r.get("fit")
        m = r["mass_kg"]
        ohz = r["osc_hz_judge"] if r["osc_hz_judge"] is not None else NAN
        head = (f"  {r['tag']:<12}{(m if m is not None else NAN):>6.3f}{r['z_p2p_m']:>10.4f}"
                f"{ohz:>8.4f}")
        if f and f.get("at_grid_edge"):
            print(head + f"{r['ring']['span_s']:>8.1f}{f['f_hz']:>8.4f}{NAN:>8.4f}"
                         f"{f['sigma_decay_per_s']:>8.4f}{NAN:>7.3f}{NAN:>8.3f}{NAN:>8.1f}"
                         f"{NAN:>7.3f}{NAN:>6.0f}%"
                         "   ← **拟合落在网格边界** ⟹ 这一段不是「衰减正弦」，本场不适用")
            continue
        if not f:
            print(head + f"{NAN:>8.1f}{NAN:>8.4f}{NAN:>8.4f}{NAN:>8.4f}{NAN:>7.3f}"
                         f"{NAN:>8.3f}{NAN:>8.1f}{NAN:>7.3f}{NAN:>6.0f}%"
                         "   ← 本场没记被控对象质量（注入失效那一档）⟹ 模型列不适用")
            continue
        g = lambda k: (r[k] if r.get(k) is not None else NAN)
        print(f"{head}{r['ring']['span_s']:>8.1f}{f['f_hz']:>8.4f}{g('model_hz'):>8.4f}"
              f"{f['sigma_decay_per_s']:>8.4f}{f['zeta_eff']:>7.3f}{g('model_decay_per_s'):>8.3f}"
              f"{g('decay_slowdown'):>8.1f}{g('D_eff'):>7.3f}"
              f"{g('D_eff_frac_of_model') * 100:>6.0f}%")
    print("  ★ 拟合残差 RMS（看拟合可信不可信）：" + "、".join(
        f"{r['tag']} {r['fit']['rms']*1000:.1f} mm" for r in rows if r.get("fit")))

    print("\n=== ④ 模型 vs 飞行 ===")
    sh = [r for r in rows if r.get("fit") and not r["fit"].get("at_grid_edge")
          and r.get("D_eff") and r.get("decay_slowdown")]
    if sh:
        print(f"  模型 ζ（D = {D_MODEL} 若接线）    {min(r['model_zeta'] for r in sh):.3f}"
              f"…{max(r['model_zeta'] for r in sh):.3f}")
        print(f"  飞行 ζ_eff（端到端实测）        {min(r['fit']['zeta_eff'] for r in sh):.3f}"
              f"…{max(r['fit']['zeta_eff'] for r in sh):.3f}   ⟹ 差 "
              f"{min(r['decay_slowdown'] for r in sh):.1f}…{max(r['decay_slowdown'] for r in sh):.1f} 倍")
        print(f"  反推等效阻尼 D_eff              {min(r['D_eff'] for r in sh):.3f}"
              f"…{max(r['D_eff'] for r in sh):.3f} N/(m·s) = 模型值的 "
              f"{min(r['D_eff_frac_of_model'] for r in sh)*100:.0f}"
              f"…{max(r['D_eff_frac_of_model'] for r in sh)*100:.0f}%")
        print(f"  积分 Routh 界 ki < D·G/m：按模型 {min(r['model_ki_bound'] for r in sh):.2f}"
              f"…{max(r['model_ki_bound'] for r in sh):.2f}（票面 9.41）；按实测 D_eff "
              f"{min(r['actual_ki_bound'] for r in sh):.2f}…{max(r['actual_ki_bound'] for r in sh):.2f}"
              f" ⟹ 现行 ki_z = {KI_Z} 占实界的 "
              f"{KI_Z/max(r['actual_ki_bound'] for r in sh)*100:.0f}"
              f"…{KI_Z/min(r['actual_ki_bound'] for r in sh)*100:.0f}%")
        # 峰峰值是不是"末窗还剩多少振铃"：σ 与 p2p 该单调反向
        srt = sorted(sh, key=lambda r: r["fit"]["sigma_decay_per_s"])
        print("  ★ σ 与末 8 s 峰峰值排到一起（σ 是**唯一**的自由变量，p2p 跟着它单调走）：")
        for r in srt:
            print(f"     {r['tag']:<10} σ = {r['fit']['sigma_decay_per_s']:.4f} /s"
                  f"  ⟹  末 8 s 峰峰值 {r['z_p2p_m']:.4f} m")

    print("\n=== ⑤ 假设①（采样 + 动捕时延）：先看「这条通道这次能取到什么值」 ===")
    print(f"  {'tag':<12}{'夹具延迟':>10}{'注入':>8}{'占模态周期':>12}{'夹具发帧':>10}{'真值流':>9}"
          f"{'控制拍':>8}")
    for tag, s2 in summaries:
        rp = s2.get("replay") or {}
        if not rp.get("delay_ms"):
            continue
        f_ = next((r["fit"]["f_hz"] for r in rows if r["tag"] == tag and r.get("fit")), None)
        per = (rp["delay_ms"] / 1000.0 * f_ * 100.0) if f_ else None
        print(f"  {tag:<12}{rp['delay_ms']:>8.2f} ms{rp.get('extra_delay_ms', 0.0):>6.1f} ms"
              f"{(per if per is not None else float('nan')):>11.2f}%"
              f"{rp.get('frames', 0):>10}{(s2.get('truth_hz') or 0):>8.1f}Hz"
              f"{(s2.get('rate_hz') or 0):>7.0f}Hz")
    print("  ⟹ **这条通道这次能取到的值**：夹具延迟只有两档（0 / 注入 50 ms，外加自带的 5.34 ms）；"
          "\n     采样率**一次都没变过** ⟹ 票面那个「50 Hz 采样」在本归档里**没有对应的一列**（见下）。")
    mdl = EVID / "mocap-model-3.json"
    if mdl.exists():
        iv = (json.loads(mdl.read_text()).get("intervals") or {})
        lg = (json.loads(mdl.read_text()).get("lag") or {})
        if iv:
            print(f"     夹具用的那条动捕流（`mocap-model-3.json`）：相邻到达 "
                  f"中位数 {iv.get('p50_ms', 0):.2f} ms、均值 {iv.get('mean_ms'):.2f} ms"
                  f" ⟹ 约 {1000.0/iv.get('mean_ms'):.0f} Hz、p99 {iv.get('p99_ms'):.1f} ms、"
                  f"最大 {iv.get('max_ms'):.1f} ms")
        if lg:
            print(f"       到达滞后 中位数 {lg.get('p50_ms', 0):.2f} ms、"
                  f"p95 {lg.get('p95_ms'):.1f} ms、最大 {lg.get('max_ms'):.1f} ms")
    print("     标称的 5.34 ms 只占模态周期（~3.3 s）的 0.2%；注入的 50 ms 占 1.5%。")

    print("\n    在**没接线**的模型里，延时的作用方向（`m s³ + (G s + ki)·e^(−sτ) = 0`）：")
    for m in sorted({r["mass_kg"] for r in rows if r.get("mass_kg")}):
        print(f"  m = {m} kg（特征方程 m s³ + (G s + ki)·e^(−sτ) = 0）：")
        for d in delay_roots(m):
            print(f"     τ = {d['tau_ms']:>6.1f} ms ⟹ 衰减率 {d['decay_per_s']:+.4f} /s、"
                  f"f = {d['f_hz']:.4f} Hz"
                  f"{'   ← 衰减率转负（不稳）' if d['decay_per_s'] < 0 else ''}")
    print("  ⟹ 延时把复根**继续往右推**（衰减率单调下降）⟹ 它是放大器，不是根因。")

    print("\n=== ⑥ 假设②：姿态耦合能偷走多少垂直推力（逐帧 `x0` 四元数，100 Hz）===")
    print(f"  {'tag':<12}{'倾角均值':>9}{'整场 p2p':>10}{'末段去趋势 p2p':>15}"
          f"{'0.20–0.45 Hz 分量':>18}{'偷走的推力':>11}{'悬停推力(该场)':>16}"
          f"{'要造成 0.2 m 需要':>18}")
    need = G_MODEL * P2P_TARGET_M
    for tag, s in summaries:
        tt = tilt_trace(s)
        if not tt:
            continue
        T = tt["thrust_ref_n"]
        steal = tt["force_steal_tail_mN"]
        print(f"  {tag:<12}{tt['tilt_mean_deg']:>8.3f}°{tt['tilt_p2p_deg']:>9.3f}°"
              f"{tt['tail_detrend_p2p_deg']:>14.3f}°{tt['band_amp_deg']:>17.4f}°"
              f"{'—' if steal is None else format(steal, '>9.3f') + ' mN':>11}"
              f"{'—' if T is None else format(T, '>16.2f')}{need:>18.2f} N")
    if need:
        print(f"  ⟹ 姿态偷走的垂直推力是 **毫牛** 量级、要解释的现象是 **~{need:.2f} N** 量级"
              " ⟹ 差三个数量级")
        print("     （偷走的推力按**该场自己的** `plant_hover.t_hover_n` 折算：标称 7.36 N、"
              "`mass+30` 9.56 N；拿标称值套满配会低报 30%）")

    print("\n=== ⑦ 假设③：`G`／`D` 只是工作点局部常数吗（从 `df` 列重建误差再回归）===")
    dfp = EVID / "df-column-20260930.json"
    dfcol = json.loads(dfp.read_text()) if dfp.exists() else None
    dfr = df_column_readings(dfcol, summaries) if dfcol else {}
    if not dfr:
        print(f"  ☠️ 缺 {dfp.name} ⟹ 这一条**这次取不到那个量**")
    for tag, r in dfr.items():
        print(f"  [{tag}] 可用相邻帧对 {r['n_pairs']}（df 夹取界 "
              f"[{r['df_clamp'][0]:+.4f}, {r['df_clamp'][1]:+.4f}]，贴界的拍已丢）")
        for b in r["u_vs_e_bands"]:
            if b.get("slope") is None:
                print(f"      |e| ∈ [{b['lo']:.2f}, {b['hi']:.2f})  n = {b['n']:>4} 样本不够")
            else:
                print(f"      |e| ∈ [{b['lo']:.2f}, {b['hi']:.2f})  n = {b['n']:>4}  "
                      f"du[0]/de = {b['slope']:+.4f} ± {b['slope_se']:.4f} N/m"
                      f"（模型 G = {G_MODEL:.4f}）  残差 σ {b['resid_std']:.4f} N")
        s = r.get("u_vs_vz_true")
        if not s:
            print("      ☠️ 对**真值速度**那一列本次**取不到**（配不上/样本不足）—— "
                  "这**不是**「通道是死的」的读数（`None` 是没量到，不是量到 0）")
        if s:
            print(f"      ★ 对**真值速度**：du[0]/dv_z = {s['slope']:+.4f} ± {s['slope_se']:.4f}"
                  f" N/(m/s)（n = {s['n']}，真值 v_z ∈ [{s['vz_range'][0]:+.3f}, "
                  f"{s['vz_range'][1]:+.3f}] m/s），模型该是 {s['model_minus_D']:+.2f}")
            print(f"        ⟹ 这条通道若接上，`u[0]` 该有 ~{s['signal_if_wired_N']:.3f} N 的摆幅；"
                  f"实测残差只有 {s['resid_std']:.4f} N ⟹ 通道**是死的**，不是被噪声埋了")

    if args.json:
        blob = {
            "_这是": "ticket 07 的读数 —— 由 tools/px4-sitl/analyze_vertical_oscillation.py 生成，别手改",
            "_怎么复跑": "python3 tools/px4-sitl/analyze_vertical_oscillation.py --json "
                     "tools/px4-sitl/evidence/tier-b/oscillation-mechanism-20261009.json",
            "_口径": {"G_model_N_per_m": G_MODEL, "D_model_N_per_mps": D_MODEL, "ki_z": KI_Z,
                    "steady_s": steady_s, "osc_p2p_m": OSC_P2P_M, "osc_hz": OSC_HZ,
                    "ring_s": args.ring_s, "ring_tol_m": args.ring_tol},
            "wiring_audit": wiring,
            "flights": rows,
            "delay_sweep": {str(m): delay_roots(m)
                            for m in sorted({r["mass_kg"] for r in rows if r.get("mass_kg")})},
            "df_column": dfr or None,
        }
        Path(args.json).write_text(json.dumps(blob, indent=1, ensure_ascii=False) + "\n")
        print(f"\n⟹ 读数已落到 {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
