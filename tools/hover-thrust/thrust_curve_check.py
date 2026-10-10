#!/usr/bin/env python3
"""从飞控 ulog 反解 **`THR_MDL_FAC`** —— `hover-thrust` 家族的第四条交叉验证。

    python3 tools/hover-thrust/thrust_curve_check.py <悬停日志.ulg>
    python3 tools/hover-thrust/thrust_curve_check.py <log.ulg> --json out.json
    python3 tools/hover-thrust/thrust_curve_check.py --self-test      # 不需要日志、不需要飞控

**为什么需要这把尺子**：[`params-changes.md`](../../docs/reference/params-changes.md) 的
**C5**（`THR_MDL_FAC = 0.45`）是**非默认值**，说明这组参数被人动过。而它是推力模型的关键参数 ——
它决定"归一化推力 ↔ 电机信号"这条曲线的形状。悬停油门（`hover_thrust` / `MPC_THR_HOVER`）
工作在**归一化推力域**，曲线在**更下游**，所以两者**必须分开量、分开验**：
`hover_thrust` 由 [`hover_thrust_from_ulog.py`](hover_thrust_from_ulog.py) 量，本脚本量曲线。

## 它测的是什么

PX4 `v1.14.3` `lib/mixer_module/functions/FunctionMotors.hpp:77-101`：
分配器给出**归一化推力** `rel_thrust`（= `actuator_motors.control[]`），
混控器**反解**曲线求电机信号 `x`（= `actuator_outputs.output[]` 的线性前身）：

```
正:  rel_thrust = fac·x² + (1−fac)·x            （fac = THR_MDL_FAC）
反:  x = −t1 + sqrt(t1² + rel/fac),  t1 = (1−fac)/(2·fac)
```

于是日志里 `output` 与 `control` 的关系**只由 `fac` 决定形状**，
输出域里的那个**仿射**（DShot 的 scale/offset）是自由的、与 `fac` 无关。
⟹ 把 `fac` 当自由参数扫一遍，看飞行数据把它顶到哪 —— **r² 的 argmax 就是反解出来的 `fac`**。
这是一条**只依赖形状**的检验，不吃任何绝对标定（不需要知道 DShot 的范围、量程、单位）。

## 判据（自检里有用例钉着）

| 判据 | 判什么 |
|---|---|
| 扫 `fac∈[0,1]` 的 **argmax r²** | 曲线的**形状**；argmax 落在 `THR_MDL_FAC` 上 ⟹ 该值被飞行数据支持 |
| `fac = 0` 的 r² | "根本没有曲线"那一档的对照（`fac=0` 时正反解都退化成恒等） |
| `fac = 1` 的 r² | "`T ∝ 油门²`"那一档的对照 —— [`hardware.md`](../../docs/reference/hardware.md) §6.3① 的 42.9 % 就架在这个假设上 |

★ **不做绝对标定**：脚本不假设 `output = 1999·x`，也不假设零偏。
它只比较**曲线形状**解释力，所以 DShot 具体怎么编码不影响结论。

⚠️ **本脚本回答"曲线形状对不对"，不回答"`hover_thrust` 是多少"** ——
后者是 [`hover_thrust_from_ulog.py`](hover_thrust_from_ulog.py) 的事。

## 输出通道 ↔ 控制通道的对应

z0 上 `PWM_MAIN_FUNC1..4 = 103, 102, 101, 104`（非默认电机序 `3-2-1-4`），
所以 `output[j]` 对应的不是 `control[j]`。脚本**不硬编码**这个对应：
它按 `fac = THR_MDL_FAC` 逐对试出 r² 最高的一组，打印出来供人对照 `PWM_MAIN_FUNC*`，
并检查它确实是一个**双射**（不是四个输出都指向同一个控制通道）。
要覆盖就传 `--map 2,1,0,3`（`output[j] <- control[map[j]]`）。
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Dict, List, Optional, Sequence, Tuple

#: z0 的 `THR_MDL_FAC`（`docs/assets/z0.params:947` = `0.449999988`）。仅作**报告对照**用，
#: 不参与反解 —— 反解是扫出来的，本文件不许把它当已知量喂进拟合。
FAC_REFERENCE = 0.45

#: 只读取前 4 路（z0 是四旋翼）。`actuator_motors` 里 control[4..11] 与
#: `actuator_outputs` 里 output[4..15] 都是常值，不参与拟合。
N_MOTORS = 4

#: 扫 fac 的网格。0.450 必须是格点，否则"argmax 落在 0.450"这句话没法逐点复核。
FAC_GRID = tuple(round(0.005 * i, 3) for i in range(201))


# ───────────────────────────── 纯函数（自检直接调它们） ─────────────────────────────

def curve_forward(motor_signal: float, fac: float) -> float:
    """归一化推力 ← 电机信号。`FunctionMotors.hpp` 正解。`fac=0` 时退化成恒等。"""
    return fac * motor_signal * motor_signal + (1.0 - fac) * motor_signal


def curve_inverse(rel_thrust: float, fac: float) -> float:
    """电机信号 ← 归一化推力。`FunctionMotors.hpp:77-101` 那条求根公式。

    `fac=0` 时**不返回** `-inf` 之类，直接退化成恒等（与 PX4 的 `a=0` 分支同义）。
    `rel_thrust` 落在定义域外（<0）时按 PX4 的做法夹到 0（`FunctionMotors` 里那句
    `if (tmp4 < 0.f) tmp4 = 0.f;`）。
    """
    if fac <= 0.0:
        return max(rel_thrust, 0.0)
    t1 = (1.0 - fac) / (2.0 * fac)
    disc = t1 * t1 + max(rel_thrust, 0.0) / fac
    return -t1 + disc ** 0.5


def affine_r2(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float, float]:
    """把 `ys` 对 `xs` 做**仿射**最小二乘，返回 `(slope, intercept, r2)`。

    ★ 用仿射（而不是过原点）是刻意的：输出域里那个 DShot scale/offset 是自由的，
      过原点会把一个**标定假设**偷偷塞进判据里。r² 是**尺度无关**的 ⟹
      不同曲线候选之间可以直接比。
    """
    n = len(xs)
    if n < 3 or len(ys) != n:
        raise ValueError(f"样本数不够做仿射拟合：n = {n}")
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0.0:
        raise ValueError("xs 是常数 ⟹ 斜率不定（这一段没有激励）")
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx
    intercept = my - slope * mx
    sst = sum((y - my) ** 2 for y in ys)
    sse = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    if sst == 0.0:
        raise ValueError("ys 是常数 ⟹ r² 无定义（这段输出没动）")
    return slope, intercept, 1.0 - sse / sst


def mean_r2(pairs: Sequence[Tuple[Sequence[float], Sequence[float]]], fac: float) -> float:
    """一组 `(control[], output[])` 在给定 `fac` 下的**平均** r²（逐通道各拟合一次）。

    逐通道各拟合是有意的：四个电机的 DShot 增益可以差几个 counts，
    共用一个仿射会把通道间的差混进残差里、压低所有候选的 r²。
    """
    vals = []
    for ctrl, out in pairs:
        xs = [curve_inverse(c, fac) for c in ctrl]
        vals.append(affine_r2(xs, out)[2])
    return sum(vals) / len(vals)


def best_fac(pairs, grid: Sequence[float] = FAC_GRID) -> Tuple[float, float]:
    """扫一遍网格，返回 `(argmax 的 fac, 该点的平均 r²)`。"""
    best = None
    for fac in grid:
        r = mean_r2(pairs, fac)
        if best is None or r > best[1]:
            best = (fac, r)
    assert best is not None
    return best


def infer_mapping(pairs_by_out: Dict[int, List[Sequence[float]]], fac: float) -> Dict[int, int]:
    """按 r² 最高给每个 `output[j]` 挑一个 `control[i]`。返回 `{j: i}`。

    ⚠️ 这不是"随便挑一个能拟合的" —— 挑完要检查它是双射（见 `is_bijection`）：
    四个输出指向同一个控制通道，说明这一段**没有**四个独立的电机信号，
    那结论就该作废，而不是照一个假映射报数。
    """
    mapping: Dict[int, int] = {}
    for j, candidates in pairs_by_out.items():
        scored = []
        for i, (ctrl, out) in enumerate(candidates):
            xs = [curve_inverse(c, fac) for c in ctrl]
            scored.append((affine_r2(xs, out)[2], i))
        mapping[j] = max(scored)[1]
    return mapping


def is_bijection(mapping: Dict[int, int]) -> bool:
    return len(set(mapping.values())) == len(mapping) == N_MOTORS


# ───────────────────────────── 自检（不需要日志） ─────────────────────────────

def self_check(emit: bool = True) -> List[str]:
    """合成一段**知道真值**的数据，看脚本能不能把它读回来。

    合成法：给定真值 `fac_true` 与真值仿射 `(A, B)`，由**电机信号** `x` 造
    `control = curve_forward(x, fac_true)` 与 `output = round(A·x + B)`。
    再把 `fac` 当未知量扫 —— argmax 必须落在 `fac_true` 上。
    `round()` 是有意的：真日志里 DShot 是**整数量化**过的，不量化就测不出这一点。

    返回**不过的项名**（空列表 = 全过）。`emit=False` 时不出声 ——
    `test_thrust_curve_check.py` 就是靠它把这份自检接进 `pytest` 的（本文件不自带 `test_*`）。
    """
    failures: List[str] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        if emit:
            print(f"  {'✅' if ok else '❌'} {name}" + (f"：{detail}" if detail else ""))
        if not ok:
            failures.append(name)

    # ① 正反解互为逆
    worst = 0.0
    for fac in (0.0, 0.1, 0.45, 0.7, 1.0):
        for x in (0.0, 0.05, 0.2, 0.3187, 0.5, 0.9, 1.0):
            worst = max(worst, abs(curve_inverse(curve_forward(x, fac), fac) - x))
    check("正反解互为逆（fac ∈ {0,0.1,0.45,0.7,1}，x ∈ [0,1]）", worst < 1e-12, f"最大偏差 {worst:.2e}")

    # ② fac=0 退化 = 恒等（"没有曲线"那一档的解析形状）
    check("fac=0 时反解退化成恒等",
          all(abs(curve_inverse(r, 0.0) - r) < 1e-15 for r in (0.0, 0.2, 0.5, 1.0)))

    # ③ 曲线单调（否则反解不是函数）
    for fac in (0.1, 0.45, 1.0):
        prev = -1.0
        mono = True
        for i in range(101):
            v = curve_forward(i / 100.0, fac)
            mono &= v > prev
            prev = v
        check(f"fac={fac} 时正解严格单调", mono)

    # ④ ★ 定性核心：扫 fac 能**逐点**把真值顶出来（含量化）
    #
    # ⚠️ **合成数据的 x 范围决定这条检验有没有鉴别力** —— 这是本自检第一版真踩过的坑：
    #    最初只取 x ∈ [0.28, 0.36]（照悬停段挑的），扫出来的 argmax **恒定偏 +0.025**。
    #    原因不是公式错（① 已经钉住正反解互为逆），而是范围太窄时**任何**单调曲线在
    #    这一段上看起来都接近仿射，r² 的峰只差 ~1e-6，整数化一抖就把 argmax 顶走。
    #    所以范围照**整条日志**的真实跨度取（rel_thrust 0 – 0.47 ⟹ 电机信号 0 – 0.53），
    #    并显式断言"峰要比对照档高出多少"（下面 ⑤ 的 `MIN_GAP`）。
    A, B = 1888.8, 109.0          # 真日志里那把仿射的量级
    X_LO, X_HI = 0.02, 0.53       # ★ 按真日志的 control 跨度反解出来的电机信号范围

    def synth(fac_true: float, x_lo=X_LO, x_hi=X_HI, n=400):
        ctrl, out = [], []
        for k in range(n):
            x = x_lo + (x_hi - x_lo) * k / (n - 1)
            ctrl.append(curve_forward(x, fac_true))
            out.append(round(A * x + B))           # 整数量化，同真日志
        return [(ctrl, out)]

    for fac_true in (0.0, 0.25, 0.45, 0.8):
        fac_hat, r_hat = best_fac(synth(fac_true))
        check(f"合成数据（fac_true={fac_true}）扫出 fac_hat={fac_hat}",
              abs(fac_hat - fac_true) < 1e-9, f"r² = {r_hat:.6f}")

    # ⑤ 对照档必须**真的更低** —— 否则"argmax 落在 0.45"没有鉴别力
    pairs = synth(0.45)
    r_045 = mean_r2(pairs, 0.45)
    gap0 = r_045 - mean_r2(pairs, 0.0)
    gap1 = r_045 - mean_r2(pairs, 1.0)
    #: 峰与对照档的最小差距。这条线**不是**从真日志抄来的一个数，而是"这段合成数据的
    #: 鉴别力够不够"的自我检查：真日志上的 gap 见 `evidence/thrust_curve_2026-10-09.txt`
    #: （§3/§4 的"对照档"两行；argmax 行 − 对照 `fac=0` 行 ⟹ 两条真机日志的 gap ≈ 4.6e-3 / 4.5e-3），本合成数据给出的是**同一量级**。
    #: 取 1e-3 当底线：低于它就说明这段合成数据**太窄、答不出问题**，自检应当红。
    min_gap = 1e-3
    check("fac=0.45 的合成数据上，fac=0 与 fac=1 的 r² 确实更低且**差距够大**",
          gap0 > min_gap and gap1 > min_gap,
          f"r²(0.45) = {r_045:.6f}；r²(0) = {mean_r2(pairs, 0.0):.6f}（差 {gap0:.2e}）、"
          f"r²(1) = {mean_r2(pairs, 1.0):.6f}（差 {gap1:.2e}）；底线 {min_gap:.0e}")

    # ⑥ 仿射拟合的 r² 尺度无关：把 output 乘 7 加 3，r² 不动 ⟹ 判据不吃绝对标定
    xs = [0.1, 0.4, 0.9, 1.4]
    ys = [2.0, 3.0, 2.5, 4.5]
    r0 = affine_r2(xs, ys)[2]
    r1 = affine_r2(xs, [7 * y + 3 for y in ys])[2]
    check("r² 对 output 的仿射变换不变（⟹ DShot 编码不影响结论）", abs(r0 - r1) < 1e-12,
          f"{r0:.12f} vs {r1:.12f}")

    if emit:
        print()
        if failures:
            print(f"✗ 自检 {len(failures)} 项不过：" + "、".join(failures))
        else:
            print("✓ 自检全部通过。")
    return failures


# ───────────────────────────── ulog 适配层 ─────────────────────────────

def _load_series(u, topic: str, prefix: str, n: int, *, t_field: str):
    """读 `topic` 的一段，返回 `(t_seconds, [series_0, ...])`。"""
    try:
        ds = u.get_dataset(topic)
    except Exception as exc:  # noqa: BLE001 - pyulog 的报错类型不止一种
        raise SystemExit(f"✗ 这份 ulog 里没有 {topic}：{exc}")
    t = [v * 1e-6 for v in ds.data[t_field]]      # ulog 时间戳是**微秒**
    cols = []
    for i in range(n):
        key = f"{prefix}[{i}]"
        if key not in ds.data:
            raise SystemExit(f"✗ {topic} 里没有字段 {key}（这份日志的机型/固件不对？）")
        cols.append([float(v) for v in ds.data[key]])
    return t, cols


def _interp_at(t_src: Sequence[float], x_src: Sequence[float], t_dst: Sequence[float]) -> List[float]:
    """把 `x_src` 按时间线性插到 `t_dst` 上。

    ★ 这一步是必须的：`actuator_motors` 与 `actuator_outputs` **发布率可以不同**，
      按下标配对会把两个不同时刻的样本凑成一对，把曲线形状糊掉。
    """
    import bisect

    order = sorted(range(len(t_src)), key=lambda k: t_src[k])
    ts = [t_src[k] for k in order]
    xs = [x_src[k] for k in order]
    out = []
    for t in t_dst:
        i = bisect.bisect_left(ts, t)
        if i <= 0:
            out.append(xs[0])
        elif i >= len(ts):
            out.append(xs[-1])
        else:
            t0, t1 = ts[i - 1], ts[i]
            w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            out.append(xs[i - 1] + w * (xs[i] - xs[i - 1]))
    return out


def analyse_ulog(u, path: str, fac_ref: float = FAC_REFERENCE,
                 mapping: Optional[Dict[int, int]] = None,
                 window: Optional[Tuple[float, float]] = None) -> dict:
    t_c, ct = _load_series(u, "actuator_motors", "control", N_MOTORS, t_field="timestamp_sample")
    t_o, ot = _load_series(u, "actuator_outputs", "output", N_MOTORS, t_field="timestamp")

    # 采样窗口：默认取整条日志里"有推力"的那一段（control 非零）。可 --window 覆盖。
    if window is None:
        live = [k for k in range(len(t_c)) if max(ct[i][k] for i in range(N_MOTORS)) > 0.02]
        if len(live) < 100:
            raise SystemExit("✗ 这条日志里 control 非零的样本太少（没解锁 / 没飞）⟹ 测不了曲线")
        window = (t_c[live[0]], t_c[live[-1]])
    sel = [k for k in range(len(t_c)) if window[0] <= t_c[k] <= window[1]]
    if len(sel) < 100:
        raise SystemExit(f"✗ 窗口 {window} 内只有 {len(sel)} 个样本")

    ct_w = [[col[k] for k in sel] for col in ct]
    t_w = [t_c[k] for k in sel]
    ot_w = [_interp_at(t_o, col, t_w) for col in ot]

    # 变动的输出通道数（其余是常值，不参与）
    active = [j for j in range(N_MOTORS) if max(ot_w[j]) - min(ot_w[j]) > 1.0]
    if len(active) != N_MOTORS:
        raise SystemExit(f"✗ 只有 {len(active)} 路输出在动（应为 {N_MOTORS}）⟹ 认不出电机输出")

    # 变动的**控制**通道数 —— 上面那道闸门只看 `output`，这里补上它管不着的那一半。
    # ★ 为什么必须**在拟合之前**判：常量 `control` ⟹ `curve_inverse` 出来的是常值 ⟹
    #   `affine_r2` 的 `sxx == 0.0` 抛 `ValueError("xs 是常数 …")`。那句话本身是清楚的，
    #   但它是从 `mean_r2` → `best_fac` / `infer_mapping` 里冒出来的一串**栈** ——
    #   与本文件其它失败路径（缺话题、缺字段、样本太少）清一色 `SystemExit` 的体例不一致。
    # ★ 而且**不能**改成接住那个异常：常量 `control` 会让每个输出的候选**全部**抛一遍，
    #   "哪一路"取决于先扫到谁 —— 只有在这里判才指得准。
    # ⚠️ 判据是**严格恒定**（`max == min`），比"抛出那个异常"略宽：`sxx` 是按均值算的，
    #   全等的一组数在浮点下 `sum/n` 未必逐位回到原值，于是偶尔 `sxx > 0` 而不抛。
    #   两种情形结论一样（这一段没有激励）⟹ 在这里一并拦掉。
    still = [i for i in range(N_MOTORS) if max(ct_w[i]) == min(ct_w[i])]
    if still:
        raise SystemExit(
            f"✗ {'、'.join(f'control[{i}]' for i in still)} 在这段窗口里是常量（没有激励）"
            "⟹ 认不出这一路的曲线（这一段它没参与控制？）")

    if mapping is None:
        by_out = {j: [(ct_w[i], ot_w[j]) for i in range(N_MOTORS)] for j in range(N_MOTORS)}
        mapping = infer_mapping(by_out, fac_ref)
        inferred = True
    else:
        inferred = False
    if not is_bijection(mapping):
        raise SystemExit(f"✗ 推出来的 output→control 对应不是双射：{mapping} ⟹ 结论作废"
                         "（这一段可能没有四个独立电机信号）")

    pairs = [(ct_w[mapping[j]], ot_w[j]) for j in range(N_MOTORS)]
    fac_hat, r_hat = best_fac(pairs)

    # 对照档
    controls = {f"fac={f}": mean_r2(pairs, f) for f in (0.0, 1.0)}
    # 逐通道在 fac_ref 与 fac_hat 上的拟合（给人看 scale/offset 是否合理）
    per_channel = []
    for j in range(N_MOTORS):
        xs = [curve_inverse(c, fac_hat) for c in ct_w[mapping[j]]]
        slope, intercept, r2 = affine_r2(xs, ot_w[j])
        per_channel.append({"output": j, "control": mapping[j], "slope": slope,
                            "intercept": intercept, "r2": r2})

    # 悬停点上的中位数（与 hover_thrust_from_ulog 的读数对照用）
    def median(v):
        s = sorted(v)
        return s[len(s) // 2]

    ctrl_med = [median(col) for col in ct_w]
    out_med = [median(col) for col in ot_w]
    mean_ctrl = sum(sum(col) / len(col) for col in ct_w) / N_MOTORS
    motor_signal = curve_inverse(mean_ctrl, fac_ref)

    return {
        "file": path,
        "window_s": list(window),
        "n_samples": len(sel),
        "mapping_output_to_control": mapping,
        "mapping_inferred": inferred,
        "fac_reference": fac_ref,
        "fac_hat": fac_hat,
        "fac_hat_r2": r_hat,
        "r2_controls": controls,
        "per_channel": per_channel,
        "control_median": ctrl_med,
        "output_median": out_med,
        "control_mean": mean_ctrl,
        "motor_signal_at_mean_control": motor_signal,
    }


def _print(rep: dict) -> None:
    print(f"日志：{rep['file']}")
    print(f"窗口：{rep['window_s'][0]:.3f} – {rep['window_s'][1]:.3f} s（{rep['n_samples']} 样本）")
    mp = rep["mapping_output_to_control"]
    tag = "（脚本推出来的）" if rep["mapping_inferred"] else "（--map 给的）"
    print(f"output→control：{ {f'out{j}': f'ctrl[{i}]' for j, i in mp.items()} } {tag}")
    print("  ★ 请对照 `PWM_MAIN_FUNC1..4`（z0 = 103, 102, 101, 104 ⟹ 电机序 3-2-1-4）复核")
    print()
    print(f"THR_MDL_FAC 反解：扫 fac∈[0,1]（201 点）的 argmax = **{rep['fac_hat']}**"
          f"（平均 r² = {rep['fac_hat_r2']:.6f}）")
    print(f"  参考值（z0.params）  = {rep['fac_reference']}")
    for name, r in rep["r2_controls"].items():
        print(f"  对照 {name:8s} 平均 r² = {r:.6f}")
    print()
    print("逐通道仿射（fac = argmax）：")
    for c in rep["per_channel"]:
        print(f"  out{c['output']} = {c['slope']:8.1f}·inv(ctrl[{c['control']}]) + "
              f"{c['intercept']:7.1f}   r² = {c['r2']:.6f}")
    print("  ★ 斜率/截距是 **DShot 编码**（scale/offset），与 fac 无关；"
          "换一条日志它们可以变，fac 的形状结论不该跟着变")
    print()
    print(f"悬停点读数：control 中位数 = {['%.6f' % v for v in rep['control_median']]}"
          f"  均值 = {rep['control_mean']:.6f}")
    print(f"            output 中位数 = {['%.1f' % v for v in rep['output_median']]}")
    print(f"            按 fac={rep['fac_reference']} 反解出的电机信号（在 control 均值上）= "
          f"{rep['motor_signal_at_mean_control']:.6f}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="从飞控 ulog 反解 THR_MDL_FAC（只读本地文件，不碰飞控）")
    ap.add_argument("ulog", nargs="?", help="悬停日志 .ulg（--self-test 时不需要）")
    ap.add_argument("--self-test", action="store_true", help="不需要日志的自检")
    ap.add_argument("--fac", type=float, default=FAC_REFERENCE,
                    help=f"报告对照用的 fac（默认 {FAC_REFERENCE}）；不参与反解")
    ap.add_argument("--map", type=str, default=None,
                    help="output→control 对应，如 `2,1,0,3`；默认脚本自己推")
    ap.add_argument("--window", type=str, default=None, help="分析窗口 `t0,t1`（秒）")
    ap.add_argument("--json", type=str, default=None, help="把报告写成 JSON")
    args = ap.parse_args(argv)

    if args.self_test:
        return 1 if self_check() else 0
    if not args.ulog:
        ap.error("要么给一个 ulog，要么用 --self-test")

    mapping = None
    if args.map:
        vals = [int(v) for v in args.map.split(",")]
        if len(vals) != N_MOTORS:
            raise SystemExit(f"✗ --map 要给 {N_MOTORS} 个数")
        mapping = {j: vals[j] for j in range(N_MOTORS)}
    window = None
    if args.window:
        a, b = (float(v) for v in args.window.split(","))
        window = (a, b)

    from pyulog import ULog  # 延后 import：--self-test 不需要 pyulog

    rep = analyse_ulog(ULog(args.ulog), args.ulog, fac_ref=args.fac,
                       mapping=mapping, window=window)
    _print(rep)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(rep, fh, ensure_ascii=False, indent=1)
        print(f"\n（报告已写入 {args.json}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
