#!/usr/bin/env python3
"""从**飞控的 ulog** 反推模型里的 `hover_thrust` —— `hover_thrust_core` 的 pyulog 适配层。

量的是 PX4 **实际输出**的归一化推力：稳态悬停时 `vehicle_thrust_setpoint.xyz[2]` 的绝对值。
为什么它就是模型里的 `hover_thrust`、为什么它不是"在复述参数"，
以及符号约定 —— 全部写在 [`hover_thrust_core.py`](hover_thrust_core.py) 的模块 docstring 里。

用法::

    python3 tools/hover-thrust/hover_thrust_from_ulog.py <log.ulg>
    python3 tools/hover-thrust/hover_thrust_from_ulog.py <log.ulg> --json out.json
    # 有独立已知值时（例如 SITL 同构机）当场对一下：
    python3 tools/hover-thrust/hover_thrust_from_ulog.py <log.ulg> --expect 0.62330

只读本地文件。**不碰飞控、不发任何 MAVLink。**
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from datetime import datetime, timezone

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from hover_thrust_core import (  # noqa: E402
    G_DEFAULT,
    estimate_hover_thrust,
    interp_to,
    relative_gap,
    t_max_for,
    tilt_rad_from_quat,
)

# ulog 里的 topic 名（多实例会带 `_0` 后缀，两种都试）
TOPIC_THRUST = ("vehicle_thrust_setpoint", "vehicle_thrust_setpoint_0")
TOPIC_LPOS = ("vehicle_local_position", "vehicle_local_position_0")
TOPIC_ATT = ("vehicle_attitude", "vehicle_attitude_0")
TOPIC_CM = ("vehicle_control_mode", "vehicle_control_mode_0")
TOPIC_LAND = ("vehicle_land_detected", "vehicle_land_detected_0")
TOPIC_HTE = ("hover_thrust_estimate", "hover_thrust_estimate_0")

#: 模型里的质量。默认取 z0 的实测值（`docs/reference/hardware.md` §9）。
MASS_DEFAULT = 0.75

EXIT_OK = 0
EXIT_UNUSABLE = 2
EXIT_EXPECT_MISMATCH = 3
EXIT_NO_DATA = 4


# ------------------------------------------------------------------ 读取


def _sha256(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def _load(ulog, names, required, label, *, optional=False, path=None):
    """按候选名找 dataset。`optional=True` 时找不到返回 None（不报错）。

    ⚠️ `path` 只是**给报错用的**，不能改成 `ulog.file_name`：pyulog 的 `ULog`
    **压根不存文件名**（`_load_file()` 只留一个文件句柄 `_file_handle`），
    那么写会在最需要它的那条路径上抛 `AttributeError` —— 报错路径自己炸掉，
    真因（"这份日志里没有某个 topic"）就永远看不见了。
    """
    for n in names:
        try:
            ds = ulog.get_dataset(n)
        except (KeyError, IndexError):
            continue
        missing = [k for k in required if k not in ds.data]
        if missing:
            raise SystemExit(f"{label}（{n}）缺字段 {missing}；实际字段：{sorted(ds.data)}")
        return n, ds
    if optional:
        return None, None
    where = path or "<这份 ulog>"
    raise SystemExit(
        f"这份 ulog 里没有 {label}（找过 {list(names)}）。"
        f"\n看有哪些 topic：\n"
        f"  python3 -c \"from pyulog import ULog; "
        f"print(sorted(d.name for d in ULog('{where}').data_list))\"\n"
        f"⚠️ 本工具需要的几个 topic 全在 `SDLOG_PROFILE` 的 **bit 0（Default）** 里 —— "
        f"若不在，多半是这份日志根本不是这个固件/这个机型记录的，而不是某个可选项没开。"
    )


def _t(ds):
    """ulog 时间戳是**微秒**（PX4 hrt）。"""
    return np.asarray(ds.data["timestamp"], dtype=float)


def _step_lookup(t_query, t_src, x_src):
    """按**阶梯**语义取值：每个查询点取"此前最近的一个源样本"（离散状态必须这样取）。

    ⚠️ 不能对这些量做线性插值：`landed` 是布尔、`flag_armed` 是布尔，
    插值出来的 0.5 既不是"已落地"也不是"未落地"。用"此前最近"才是它们的真实语义。
    查询点早于第一个源样本时返回 `x_src[0]`。
    """
    idx = np.searchsorted(t_src, t_query, side="right") - 1
    np.clip(idx, 0, t_src.size - 1, out=idx)
    return np.asarray(x_src, dtype=float)[idx]


def _in_flight_mask(t_query, u, *, path=None):
    """`flag_armed ∧ ¬landed` —— **读日志里的第一手状态，不猜**。

    为什么不用"位置变化 > 阈值"之类的启发式：那类判据在 `tau-analysis` 上真栽过
    （它把"被人挪动"当成"在飞"）。日志里写着解锁与离地状态，没有理由去猜。

    两个 topic 缺任何一个都返回 `None`，交给调用方决定怎么兜底（本工具选择**报错退出**，
    因为"哪一段在地上"直接决定哪些样本算悬停，不能默认全要）。
    """
    # `optional=True`：topic **整个不在**时返回 `(None, None)`，交给调用方给干净结论；
    # 但 topic 在、字段不在时**照常报错**（与 LAND/LPOS/ATT/THRUST 一个待遇）——
    # 那说明固件/机型不对，不是"这份日志没数据"。以前这里直接 `cm.data["flag_armed"]`，
    # 缺字段会抛 `KeyError`：报错路径自己炸掉，真因就看不见了（正是本目录自检要防的形状）。
    _n_cm, cm = _load(
        u, TOPIC_CM, ["flag_armed"], "解锁状态（vehicle_control_mode）",
        optional=True, path=path,
    )
    if cm is None:
        return None, TOPIC_CM[0]
    # ★ 单位只在这里统一一次：ulog 原样是**微秒**，而调用方传进来的 `t_query` 是**秒**。
    # 混用不会报错、只会静默地把半个日志判成"没在飞"（下标全落到数组头上）。
    t_cm = _t(cm) * 1e-6
    armed = np.asarray(cm.data["flag_armed"], dtype=float)

    _, ld = _load(u, TOPIC_LAND, ["landed"], "离地状态（vehicle_land_detected）", path=path)
    landed = _step_lookup(t_cm, _t(ld) * 1e-6, np.asarray(ld.data["landed"], dtype=float))

    # ★ 先在 `control_mode` 那条时间基上把两个状态**合起来**，最后才落到查询点上。
    # 曾经把两条各自落到查询点再相与 —— 两条的样本数不同（67 vs 1587），
    # numpy 直接抛"广播不了"。这条错误幸好是响的；若两边样本数碰巧一样，
    # 它就会**静默地**把两个不同时刻的状态配到一起，判出错的在飞窗口。
    in_flight_on_cm = (armed > 0.5) & (landed < 0.5)
    return _step_lookup(t_query, t_cm, in_flight_on_cm.astype(float)) > 0.5, None


# ------------------------------------------------------------------ 报告


def analyse(path, mass=MASS_DEFAULT, min_duration=2.0, mask_kwargs=None):
    """打开 ulog 再分析。**不打印** —— 打印归 `main`。"""
    from pyulog import ULog

    return analyse_ulog(ULog(path), path, mass=mass, min_duration=min_duration,
                        mask_kwargs=mask_kwargs)


def analyse_ulog(u, path, mass=MASS_DEFAULT, min_duration=2.0, mask_kwargs=None):
    """跑一遍**已经打开的** ulog，返回 `(报告 dict, 退出码)`。

    单拆这一层是为了留一个测试接缝：`ULog` 的构造函数会去开文件，没法注入；
    而这个函数只用到 `get_dataset()` 与 `data_list` 两件事 —— 自检里用一个
    **假 ULog** 就能把"空在飞窗口""缺 topic"这些**只在坏日志上才走到的路径**钉住。
    （这批判据原本只在 175 份真日志上跑过一遍才暴露出来，见 `evidence/`。）
    """
    mask_kwargs = dict(mask_kwargs or {})
    size = os.path.getsize(path)
    digest = _sha256(path)

    rep = {
        "file": os.path.abspath(path),
        "bytes": size,
        "sha256": digest,
        "analysed_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }

    names = sorted(d.name for d in u.data_list)
    rep["topics_present"] = names
    if not any(n in names for n in TOPIC_THRUST):
        rep["error"] = f"日志里没有归一化推力 topic（找过 {list(TOPIC_THRUST)}）"
        return rep, EXIT_NO_DATA

    _, thr = _load(
        u, TOPIC_THRUST, ["xyz[2]"], "归一化推力（vehicle_thrust_setpoint）", path=path
    )
    t_thr = _t(thr) * 1e-6
    thrust_z = np.asarray(thr.data["xyz[2]"], dtype=float)

    _, lp = _load(
        u, TOPIC_LPOS, ["vx", "vy", "vz"], "本地位置（vehicle_local_position）", path=path
    )
    t_lp = _t(lp) * 1e-6
    speed = np.sqrt(
        np.asarray(lp.data["vx"], dtype=float) ** 2
        + np.asarray(lp.data["vy"], dtype=float) ** 2
        + np.asarray(lp.data["vz"], dtype=float) ** 2
    )

    _, att = _load(
        u, TOPIC_ATT, ["q[0]", "q[1]", "q[2]", "q[3]"], "姿态（vehicle_attitude）", path=path
    )
    t_att = _t(att) * 1e-6
    q = np.stack([np.asarray(att.data[f"q[{i}]"], dtype=float) for i in range(4)], axis=-1)
    tilt = tilt_rad_from_quat(q)

    # 全部插到**推力那条时间基**上：它是控制量，是我们要解释的对象，
    # 其余三条只是判据。★ 落到源区间之外会给 NaN（见 core 的 `interp_to`）。
    speed_at = interp_to(t_lp, speed, t_thr)
    tilt_at = interp_to(t_att, tilt, t_thr)

    in_flight, missing = _in_flight_mask(t_thr, u, path=path)
    if in_flight is None:
        rep["error"] = (
            f"日志里缺 {missing} —— 分不出「在地面」与「在飞」，"
            "而稳态悬停判据必须只在在飞段里生效（地面上推着飞机走也会满足速度/倾角判据）"
        )
        return rep, EXIT_NO_DATA
    if not in_flight.any():
        # ⚠️ 必须单独挡在这里：下面 `.min()`/`.max()` 遇到空数组会抛
        # "zero-size array to reduction operation minimum"，
        # 于是"这份日志里没有飞行段"（一个**正常结论**）会表现为**崩溃**。
        # 175 份 SITL 日志里这类占了大头（SITL 起不来/没起飞的那些）。
        rep["error"] = (
            "在飞窗口为空 —— `flag_armed ∧ ¬landed` 在这份日志里从未同时成立，"
            "也就是**没有飞行段**可分析（不是「没悬停」，是「没飞」）"
        )
        return rep, EXIT_NO_DATA
    t0, t1 = t_thr[in_flight].min(), t_thr[in_flight].max()
    rep["flight_window"] = {"t0": float(t0), "t1": float(t1), "n_thrust_samples": int(in_flight.sum())}

    # 不在飞 ⟹ 一律 NaN，掩码自然把它们排除，且**不**参与任何统计。
    keep = in_flight
    speed_at = np.where(keep, speed_at, np.nan)
    tilt_at = np.where(keep, tilt_at, np.nan)
    thrust_in = np.where(keep, thrust_z, np.nan)

    est = estimate_hover_thrust(
        t_thr, speed_at, tilt_at, thrust_in, min_duration=min_duration, **mask_kwargs
    )

    rep["hover_thrust"] = est.hover_thrust
    rep["usable"] = est.usable
    rep["sign"] = est.sign
    rep["notes"] = list(est.notes)
    rep["segment_spread"] = est.segment_spread
    rep["segments"] = [
        {
            "t0": s.t0,
            "t1": s.t1,
            "n": s.n,
            "hover_thrust": s.thrust,
            "thrust_z_median": s.thrust_z_median,
            "iqr": s.iqr,
            "block_spread": s.block_spread,
        }
        for s in est.segments
    ]
    rep["criteria"] = {
        "min_duration_s": min_duration,
        "v_eps": mask_kwargs.get("v_eps", 0.30),
        "tilt_eps_deg": np.degrees(mask_kwargs.get("tilt_eps", np.radians(10.0))),
    }

    # 飞控自己的悬停推力估计 —— ★ **不是独立口径**，见下面打印时的说明。
    _, hte = _load(
        u,
        TOPIC_HTE,
        ["hover_thrust", "valid"],
        "悬停推力估计（hover_thrust_estimate）",
        optional=True,
    )
    if hte is not None:
        t_hte = _t(hte) * 1e-6
        in_win = (t_hte >= t0) & (t_hte <= t1)
        hv = np.asarray(hte.data["hover_thrust"], dtype=float)[in_win]
        vd = np.asarray(hte.data["valid"], dtype=float)[in_win]
        valid = vd > 0.5
        rep["hte"] = {
            "n": int(in_win.sum()),
            "valid_fraction": float(valid.mean()) if in_win.any() else 0.0,
            "median_when_valid": float(np.median(hv[valid])) if valid.any() else None,
            "median_all": float(np.median(hv)) if in_win.any() else None,
        }

    if est.hover_thrust is not None:
        rep["t_max_n"] = t_max_for(est.hover_thrust, mass)
        rep["t_hover_n"] = mass * G_DEFAULT
    rep["mass_kg"] = mass
    return rep, (EXIT_OK if est.usable else EXIT_UNUSABLE)


# ------------------------------------------------------------------ 打印


def _print(rep, expect=None, expect_tol=0.02, model_hover=None):
    print(f"文件      : {rep['file']}")
    print(f"大小/摘要 : {rep['bytes'] / 1e6:.2f} MB  sha256={rep['sha256']}")

    if rep.get("error"):
        print(f"\n❌ {rep['error']}")
        return

    fw = rep["flight_window"]
    print(
        f"在飞窗口  : {fw['t0']:.3f} – {fw['t1']:.3f} s"
        f"（`flag_armed ∧ ¬landed`，其内 {fw['n_thrust_samples']} 个推力样本）"
    )
    c = rep["criteria"]
    print(
        f"稳态判据  : |v| ≤ {c['v_eps']} m/s、倾角 ≤ {c['tilt_eps_deg']:g}°、"
        f"|thrust_z| ∈ [0.15, 0.98]、段长 ≥ {c['min_duration_s']:g} s"
    )

    print("\n候选悬停段（★ = 主口径用的那段，最长）")
    for line in _fmt_segments_obj(rep["segments"]):
        print(line)
    if not rep["segments"]:
        print("  （没有）")

    print("\n读数")
    if rep["hover_thrust"] is None:
        print("  hover_thrust = —— （不可用）")
    else:
        print(f"  hover_thrust = {rep['hover_thrust']:.5f}   [符号约定：{rep['sign']}]")
        print(f"  → T_max = m·g/hover_thrust = {rep['t_max_n']:.4f} N（m = {rep['mass_kg']} kg）")
    print(f"  段间不一致 = {rep['segment_spread']:.2%}  可用 = {rep['usable']}")

    hte = rep.get("hte")
    if hte:
        print(
            f"\n飞控自己的悬停推力估计：{hte['n']} 个样本，valid 占比 {hte['valid_fraction']:.1%}"
            + (
                f"，valid 时中位数 {hte['median_when_valid']:.4f}"
                if hte["median_when_valid"] is not None
                else ""
            )
        )
        print(
            "  ⚠️ **这不是独立口径**：HTE 开着时，位置控制器用的就是它 —— 两者一致是同义反复。\n"
            "     它只回答一件事：**估计器收敛了没有**。SITL 上实测它是 0.5 且 valid=0 "
            "（没收敛），\n     而同一份日志的主口径给出 0.6233（= 已知真值）⟹ 拿它当读数会得到错的数。"
        )

    if model_hover is not None and rep["hover_thrust"] is not None:
        gap = relative_gap(rep["hover_thrust"], model_hover)
        print(f"\n与模型当前值 {model_hover:.5f} 相差 {gap:.2%}")

    for s in rep["notes"]:
        print(f"  ⚠️ {s}")

    if expect is not None and rep["hover_thrust"] is not None:
        gap = relative_gap(rep["hover_thrust"], expect)
        ok = gap <= expect_tol
        print(
            f"\n{'✅' if ok else '❌'} 与已知值 {expect:.5f} 相差 {gap:.4%}"
            f"（判据 ≤ {expect_tol:.2%}）"
        )


def _fmt_segments_obj(segments):
    out = []
    for i, s in enumerate(segments):
        mark = "★" if i == 0 else " "
        out.append(
            f"  {mark} [{s['t0']:8.3f}, {s['t1']:8.3f}] s  n={s['n']:5d}  "
            f"hover={s['hover_thrust']:.4f}  iqr={s['iqr']:.4f}  "
            f"段内不一致={s['block_spread']:.2%}"
        )
    return out


# ------------------------------------------------------------------ 入口


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="从飞控 ulog 反推模型里的 hover_thrust（只读本地文件）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("ulog")
    ap.add_argument("--mass", type=float, default=MASS_DEFAULT, help=f"起飞质量 [kg]，默认 {MASS_DEFAULT}")
    ap.add_argument("--min-duration", type=float, default=2.0, help="一段稳态悬停的最短时长 [s]")
    ap.add_argument(
        "--v-eps",
        type=float,
        default=0.30,
        help="速度模 `|v|`（**三维**，不是水平分量）的上限 [m/s] —— 只是粗筛；"
        "精度由 --tilt-eps-deg 保证（见 core 的说明）",
    )
    ap.add_argument("--tilt-eps-deg", type=float, default=10.0, help="倾角上限 [°]")
    ap.add_argument("--expect", type=float, default=None, help="已知的独立真值（SITL 同构机用）")
    ap.add_argument("--expect-tol", type=float, default=0.02, help="与 --expect 的允许相对偏差")
    ap.add_argument("--model-hover", type=float, default=None, help="模型里当前的值，用来打印差距")
    ap.add_argument("--json", default=None, help="把完整结果写成 JSON")
    args = ap.parse_args(argv)

    mask_kwargs = {"v_eps": args.v_eps, "tilt_eps": np.radians(args.tilt_eps_deg)}
    rep, code = analyse(
        args.ulog, mass=args.mass, min_duration=args.min_duration, mask_kwargs=mask_kwargs
    )
    _print(rep, expect=args.expect, expect_tol=args.expect_tol, model_hover=args.model_hover)

    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(rep, f, ensure_ascii=False, indent=2, sort_keys=True)
        print(f"\n（JSON 已写入 {args.json}）")

    if (
        args.expect is not None
        and rep.get("hover_thrust") is not None
        and relative_gap(rep["hover_thrust"], args.expect) > args.expect_tol
    ):
        return EXIT_EXPECT_MISMATCH
    return code


if __name__ == "__main__":
    raise SystemExit(main())
