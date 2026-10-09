#!/usr/bin/env python3
"""从**已归档的飞控 ulog** 读"电池可持续供出多少电流/功率" —— `battery_core` 的 pyulog 适配层。

服务 [phase3/08](../../.scratch/phase3/issues/08-battery-sustainable-thrust-from-archived-ulogs.md)。
**只读本地文件。不碰飞控、不发任何 MAVLink、不起链。**

用法::

    python3 tools/battery-thrust/battery_from_ulog.py <log.ulg>
    python3 tools/battery-thrust/battery_from_ulog.py bag/            # 目录：递归找 *.ulg
    python3 tools/battery-thrust/battery_from_ulog.py <log.ulg> --json out.json
    python3 tools/battery-thrust/battery_from_ulog.py <log.ulg> --f-max 11.8 --rating 15.0

三条结论各自的口径（票面要求写清"用的是哪段窗口、依据哪个字段"）：

1. **可持续电流** = 在飞窗口内的 **1 s 滑窗中位电流**的**最大值**（`--window` 可调），
   并**分前后半段各报一次** —— 前半 = 电量高的一端、后半 = 放电末端。
   ⚠️ 不是 `current_a.max()`：实测见过 11.38 A 的**孤点**（1 s 窗中位只有 4.78 A）。
2. **换算成推力/功率** = 以**稳态悬停**为锚点（稳态时推力 ≡ `m·g`，物理给的），
   再按 `P ∝ T^1.5` 外推到 `--f-max`。锚点与目标都是**数**，不是印象。
3. **取不到的** 落在 `channel_verdicts` / `missing` 两节，逐条点名（含为什么不能当判据）。

退出码：`0` 有在飞段且有稳态悬停锚点 / `2` 有在飞段但取不到锚点 / `4` 没有可分析的在飞段。
⚠️ 目录模式下取**各文件中最差的那个**（宁可看起来更糟，也不要因为有一个好人就让整批显示绿）。
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import sys
from datetime import datetime, timezone

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from battery_core import (  # noqa: E402
    BAT_AVRG_CURRENT_DEFAULT,
    CELL_RATING_A_DEFAULT,
    EXIT_NO_DATA,
    EXIT_OK,
    EXIT_UNAVAILABLE,
    F_MAX_DEFAULT,
    MASS_DEFAULT,
    PROP_EXPONENT_DEFAULT,
    T_MAX_DEFAULT,
    channel_verdicts,
    esr_and_signature,
    extrapolate_current,
    extrapolate_power,
    hover_anchor,
    in_flight_mask,
    interp_to,
    robust_noise,
    sustained_lands_at_flight_start,
    sustained_max,
    throughput_verdict,
    tilt_rad_from_quat,
    voltage_after_sag,
)

# ulog 里的 topic 名（多实例会带 `_0` 后缀，两种都试 —— 与同族工具同一写法）
TOPIC_BAT = ("battery_status", "battery_status_0")
TOPIC_CM = ("vehicle_control_mode", "vehicle_control_mode_0")
TOPIC_LAND = ("vehicle_land_detected", "vehicle_land_detected_0")
TOPIC_LPOS = ("vehicle_local_position", "vehicle_local_position_0")
TOPIC_ATT = ("vehicle_attitude", "vehicle_attitude_0")

#: 参数表里与本票有关的几项（**读日志内嵌的参数**，不是读 `z0.params` —— 归档的是当时那套）
PARAM_KEYS = (
    "BAT1_A_PER_V", "BAT1_V_DIV", "BAT1_N_CELLS", "BAT1_CAPACITY", "BAT1_SOURCE",
    "BAT1_I_CHANNEL", "BAT1_V_CHANNEL", "BAT1_R_INTERNAL", "BAT1_V_CHARGED", "BAT1_V_EMPTY",
    "BAT_V_OFFS_CURR", "BAT_AVRG_CURRENT", "MC_BAT_SCALE_EN", "MPC_THR_HOVER",
)


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
    """按候选名找 dataset。`optional=True` 时找不到返回 `(None, None)`。

    ⚠️ `path` 只是**给报错用的**，不能写成 `ulog.file_name`：pyulog 的 `ULog` **压根不存文件名**
    ⟹ 那么写会在最需要它的那条路径上抛 `AttributeError`，把真因盖掉。
    （这条与 `tools/hover-thrust` 的同名函数是同一个教训，注释也照抄一遍免得下一个人再踩。）
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
        f"这份 ulog 里没有 {label}（找过 {list(names)}）。\n看有哪些 topic：\n"
        f"  python3 -c \"from pyulog import ULog; "
        f"print(sorted(d.name for d in ULog('{where}').data_list))\""
    )


def _t(ds):
    """ulog 时间戳是**微秒**（PX4 hrt）；本模块内统一成**秒**。

    ★ 单位只在这里统一一次。混用不会报错，只会静默地把半个日志判成"没在飞"
    （下标全落到数组头上）—— 同族工具在这个形状上栽过。
    """
    return np.asarray(ds.data["timestamp"], dtype=float) * 1e-6


def _params(u):
    """读日志**内嵌**的参数（`initial_parameters`），缺键返回 `None` 而不是 0。

    ⚠️ `None` = "这份日志里没有这个参数"，与"这个参数是 0"是两回事 —— 后面的判据会用到这一点。
    """
    ip = getattr(u, "initial_parameters", None) or {}
    return {k: (float(ip[k]) if k in ip else None) for k in PARAM_KEYS}


# ------------------------------------------------------------------ 分段


def _span_stats(t, x, label, t0, t1, dt, min_n):
    m = (t >= t0) & (t <= t1)
    if not m.any():
        return None
    return {
        "label": label,
        "window_s": [float(t0), float(t1)],
        "n": int(m.sum()),
        "sustained_a": sustained_max(t[m], x[m], dt=dt, min_n=min_n),
        "mean_a": float(np.mean(x[m])),
        "voltage_v_min": None,  # 由调用方补（电压在另一条数上取）
    }


# ------------------------------------------------------------------ 分析


def analyse(path, mass=MASS_DEFAULT, f_max=F_MAX_DEFAULT, rating=CELL_RATING_A_DEFAULT,
            v_eps=0.30, tilt_eps_deg=10.0, min_duration=2.0, window=1.0, min_n=3,
            prop_exponent=PROP_EXPONENT_DEFAULT):
    """打开 ulog 再分析。**不打印** —— 打印归 `main`。"""
    from pyulog import ULog

    return analyse_ulog(
        ULog(path), path, mass=mass, f_max=f_max, rating=rating, v_eps=v_eps,
        tilt_eps_deg=tilt_eps_deg, min_duration=min_duration, window=window,
        min_n=min_n, prop_exponent=prop_exponent,
    )


def analyse_ulog(u, path, mass=MASS_DEFAULT, f_max=F_MAX_DEFAULT,
                 rating=CELL_RATING_A_DEFAULT, v_eps=0.30, tilt_eps_deg=10.0,
                 min_duration=2.0, window=1.0, min_n=3,
                 prop_exponent=PROP_EXPONENT_DEFAULT):
    """跑一遍**已经打开的** ulog，返回 `(报告 dict, 退出码)`。

    单拆这一层是为了留测试接缝：`ULog()` 的构造会去开文件、没法注入，
    而这里只用到 `get_dataset()` / `data_list` / `initial_parameters` 三件事 ——
    自检里用一个**假 ULog** 就能把"没在飞""缺 topic""单节电压全 0"这些**只在坏日志上才走到**的路径钉住。
    """
    rep = {
        "file": os.path.abspath(path),
        "bytes": os.path.getsize(path),
        "sha256": _sha256(path),
        "analysed_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }

    names = sorted(d.name for d in u.data_list)
    rep["topics_present"] = names
    params = _params(u)
    rep["battery_params"] = params

    if not any(n in names for n in TOPIC_BAT):
        rep["error"] = f"日志里没有 battery_status（找过 {list(TOPIC_BAT)}）"
        rep["missing"] = ["battery_status"]
        return rep, EXIT_NO_DATA

    _, bat = _load(u, TOPIC_BAT, ["voltage_v", "current_a"], "电池状态（battery_status）", path=path)
    tb = _t(bat)
    voltage = np.asarray(bat.data["voltage_v"], dtype=float)
    current = np.asarray(bat.data["current_a"], dtype=float)
    # 可选字段：有就用，没有就如实标 None（不编造）
    vol_f = np.asarray(bat.data["voltage_filtered_v"], dtype=float) if "voltage_filtered_v" in bat.data else None
    cur_f = np.asarray(bat.data["current_filtered_a"], dtype=float) if "current_filtered_a" in bat.data else None
    cur_avg = np.asarray(bat.data["current_average_a"], dtype=float) if "current_average_a" in bat.data else None
    discharged = np.asarray(bat.data["discharged_mah"], dtype=float) if "discharged_mah" in bat.data else None
    remaining = np.asarray(bat.data["remaining"], dtype=float) if "remaining" in bat.data else None
    scale = np.asarray(bat.data["scale"], dtype=float) if "scale" in bat.data else None
    cell_cols = sorted(k for k in bat.data if k.startswith("voltage_cell_v["))
    if cell_cols:
        cells = np.stack([np.asarray(bat.data[k], dtype=float) for k in cell_cols], axis=-1)
    else:
        cells = None

    # ---- 在飞窗口：`flag_armed ∧ ¬landed`（第一手状态，不猜）
    _n, cm = _load(u, TOPIC_CM, ["flag_armed"], "解锁状态（vehicle_control_mode）",
                   optional=True, path=path)
    if cm is None:
        rep["error"] = (f"日志里缺 {TOPIC_CM[0]} —— 分不出「在地面」与「在飞」；"
                        "电池的可持续读数只在在飞段里才有意义（地面上没有负载）")
        return rep, EXIT_NO_DATA
    _, ld = _load(u, TOPIC_LAND, ["landed"], "离地状态（vehicle_land_detected）", path=path)
    inflight = in_flight_mask(tb, _t(cm), np.asarray(cm.data["flag_armed"], dtype=float),
                              _t(ld), np.asarray(ld.data["landed"], dtype=float))
    if not inflight.any():
        rep["flight_window"] = None
        rep["error"] = ("在飞窗口为空 —— `flag_armed ∧ ¬landed` 在这份日志里从未同时成立"
                        "（这条本身是个**正常结论**：这份日志没有飞行段）")
        rep["missing"] = ["在飞段（因此三个问题都无从回答）"]
        return rep, EXIT_NO_DATA

    t0, t1 = float(tb[inflight].min()), float(tb[inflight].max())
    rep["flight_window"] = {"t0": t0, "t1": t1, "n": int(inflight.sum()),
                            "span_s": t1 - t0}

    # ---- 电压（在飞段）
    vin = voltage[inflight]
    rep["voltage"] = {
        "min_v": float(np.min(vin)),
        "median_v": float(np.median(vin)),
        "max_v": float(np.max(vin)),
        "noise_1sigma_v": float(robust_noise(vin)),
        "filtered_median_v": float(np.median(vol_f[inflight])) if vol_f is not None else None,
        "cell_count": (float(bat.data["cell_count"][0]) if "cell_count" in bat.data else None),
    }
    if cells is not None:
        cis = cells[inflight]
        rep["voltage"]["cell_voltage_max_abs"] = float(np.max(np.abs(cis)))
        rep["voltage"]["cell_voltage_all_zero"] = bool(np.max(np.abs(cis)) == 0.0)

    # ---- 电流（在飞段）：可持续 = 1 s 滑窗中位
    half = t0 + 0.5 * (t1 - t0)
    rep["current"] = {
        "sustained": sustained_max(tb[inflight], current[inflight], dt=window, min_n=min_n),
        "mean_a": float(np.mean(current[inflight])),
        "noise_1sigma_a": float(robust_noise(current[inflight])),
        "exact_zero_samples": int(np.sum(current[inflight] == 0.0)),
        "filtered_mean_a": float(np.mean(cur_f[inflight])) if cur_f is not None else None,
        "average_field_min_max_a": (
            [float(np.min(cur_avg)), float(np.max(cur_avg))] if cur_avg is not None else None
        ),
        "sustained_first_half": _span_stats(tb[inflight], current[inflight], "在飞前半（电量较高）",
                                            t0, half, window, min_n),
        "sustained_last_half": _span_stats(tb[inflight], current[inflight], "在飞后半（放电末端）",
                                           half, t1, window, min_n),
    }
    # ★ 票面问 ① 时**点名排除**了「起飞那一瞬的最大电流」。而"全段最大"这个统计量
    # 天生往负载最高的那一瞬跑 —— 不把这件事标出来，读者会把它当成问 ① 的答案。
    rep["current"]["sustained"]["at_flight_start"] = sustained_lands_at_flight_start(
        rep["current"]["sustained"], t0, window)
    # 两半的电压各补一次（"低电量端的电压跌落才决定还能不能顶住"）
    for key, (a, b) in (("sustained_first_half", (t0, half)), ("sustained_last_half", (half, t1))):
        s = rep["current"][key]
        if s is None:
            continue
        m = (tb >= a) & (tb <= b) & inflight
        s["voltage_v_min"] = float(np.min(voltage[m])) if m.any() else None
        s["voltage_v_median"] = float(np.median(voltage[m])) if m.any() else None

    if discharged is not None:
        d = discharged[inflight]
        rep["charge"] = {
            "discharged_mah_start": float(d[0]),
            "discharged_mah_end": float(d[-1]),
            "discharged_mah_delta": float(d[-1] - d[0]),
            "remaining_start": float(remaining[inflight][0]) if remaining is not None else None,
            "remaining_end": float(remaining[inflight][-1]) if remaining is not None else None,
            "rating_mah": None,  # BAT1_CAPACITY 为 -1 ⟹ 未知；见 battery_params
        }

    # ---- 判据：这两列各自能不能当判据
    rep["signature"] = esr_and_signature(tb[inflight], current[inflight], voltage[inflight],
                                         dt=window, min_n=min_n)
    rep["channel_verdicts"] = channel_verdicts({
        "voltage_cell_v": {"present": cells is not None, "values": cells[inflight] if cells is not None else None},
        "current_average_a": {"present": cur_avg is not None,
                              "values": cur_avg[inflight] if cur_avg is not None else None,
                              "bat_avrg_current": (params.get("BAT_AVRG_CURRENT")
                                                   or BAT_AVRG_CURRENT_DEFAULT)},
        "scale": {"present": scale is not None, "values": scale[inflight] if scale is not None else None},
    })
    rep["missing"] = _missing(rep, params)

    # ---- 锚点：稳态悬停（推力 ≡ m·g）
    have_lp = any(n in names for n in TOPIC_LPOS)
    have_att = any(n in names for n in TOPIC_ATT)
    anchor = None
    if have_lp and have_att:
        _, lp = _load(u, TOPIC_LPOS, ["vx", "vy", "vz"], "本地位置（vehicle_local_position）", path=path)
        tl = _t(lp)
        speed = np.sqrt(sum(np.asarray(lp.data[k], dtype=float) ** 2 for k in ("vx", "vy", "vz")))
        _, att = _load(u, TOPIC_ATT, [f"q[{i}]" for i in range(4)], "姿态（vehicle_attitude）", path=path)
        ta = _t(att)
        q = np.stack([np.asarray(att.data[f"q[{i}]"], dtype=float) for i in range(4)], axis=-1)
        tilt = tilt_rad_from_quat(q)
        # 插到**电池**那条时间基上：锚点要用电池的电压/电流，所以电池是主
        speed_at = interp_to(tl, speed, tb)
        tilt_at = interp_to(ta, tilt, tb)
        # ★ `eligible=inflight`：搜索范围**先夹到在飞段**。
        # 地面上飞机静止、倾角为零，比真正的悬停还"稳态" —— 不夹的话
        # 最长的稳态段会被地面段赢走（第一版就是这么一份锚点都没取到的）。
        anchor, why = hover_anchor(
            tb, voltage, current, speed_at, tilt_at, mass=mass, v_eps=v_eps,
            tilt_eps_rad=np.radians(tilt_eps_deg), min_duration=min_duration,
            eligible=inflight,
        )
        if anchor is None:
            rep["anchor_error"] = why.get("reason")
    else:
        rep["anchor_error"] = (f"缺 {'vehicle_local_position' if not have_lp else ''} "
                               f"{'vehicle_attitude' if not have_att else ''}".strip()
                               + " —— 挑不出稳态悬停锚点")

    if anchor is not None:
        rep["hover_anchor"] = anchor.as_dict()
        rep["extrapolation"] = _extrapolate(anchor, f_max, mass, rep["signature"], prop_exponent)
        rep["verdict"] = throughput_verdict(
            f_max, i_at_f_max=rep["extrapolation"]["current_a"], bat_current_rating=rating,
        )
        rep["verdict"]["f_max_n"] = float(f_max)
        rep["verdict"]["t_max_n"] = float(T_MAX_DEFAULT)
        rep["verdict"]["t_max_note"] = (
            "t_max 是**归一化**常数（按电机/桨，实测满量程），与 f_max **不是一个量**："
            "本工具的换算一律只用 f_max。两者之比 = %.3f 倍 —— 混用会把上限抬这么多倍"
            % (T_MAX_DEFAULT / float(f_max))
        )

    code = EXIT_OK if anchor is not None else EXIT_UNAVAILABLE
    rep["exit_code"] = code
    return rep, code


def _missing(rep, params):
    """**如实点名这次取不到的量**（票面问题 ③ 的正面清单）。"""
    out = []
    if rep["voltage"].get("cell_voltage_all_zero"):
        out.append("单节电压 `voltage_cell_v[*]`：字段在、**整份日志全为 0** —— "
                   "模拟电源模块不填它（PX4 只有 smart battery 类驱动会填），"
                   "`cell_count = %s` 只是 `BAT1_N_CELLS` 这个**参数**" % rep["voltage"]["cell_count"])
    if params.get("BAT1_CAPACITY") is not None and params["BAT1_CAPACITY"] <= 0:
        out.append("电池容量 `BAT1_CAPACITY = %.0f` ⟹ **容量未知**："
                   "`remaining` 走的是**电压法**（`Battery::estimateStateOfCharge` 的 else 分支），"
                   "不是安时积分；`discharged_mah` 虽有数，但它继承 `current_a` 的标度误差"
                   % params["BAT1_CAPACITY"])
    if params.get("BAT_V_OFFS_CURR") == 0.0:
        out.append("电流零点 `BAT_V_OFFS_CURR = 0` ⟹ **未标定**："
                   "未解锁段实测仍有 ~0.35 A 读数（可能是飞控/机载设备真实静态电流，"
                   "也可能是没扣掉的直流偏置）——**两者这次分不开**，"
                   "所以任何窗口均值都带一个量级 ±0.35 A 的系统不确定度")
    if params.get("BAT1_A_PER_V") is not None:
        out.append("电流标度 `BAT1_A_PER_V = %.1f`（以及 `BAT1_V_DIV = %s`）："
                   "**日志内无法独立验证**它对不对（没有拉力台、没有库仑计、没有第二条电流通路）。"
                   "⟹ 下面所有以安培为单位的数，以及由它导出的功率/比例，"
                   "都建立在「这个标度大致正确」这一条**未经本次验证的前提**上"
                   % (params["BAT1_A_PER_V"], params.get("BAT1_V_DIV")))
    out.append("**15 A 这个额定值本身**：它是电芯的**持续放电额定**（热/寿命约束），"
               "而日志能证伪的只是**电压崩塌**那一条。两者不是同一件事 —— "
               "本工具只能给「占额定的比例」，不能给「电池还能多供多少」")
    return out


def _extrapolate(anchor, f_max, mass, signature, prop_exponent):
    """锚点 → `f_max` 的换算，**每次迭代都记下来**（免得读数看起来像凭空来的）。"""
    r = signature.get("r_int_ohm")
    v0, i0 = anchor.voltage_v, anchor.current_a
    p_at = extrapolate_power(anchor.power_w, anchor.thrust_n, f_max, n=prop_exponent)
    # 电压随电流下沉 ⟹ 同样功率要多抽电流；只迭代两次，收敛与否如实记录
    i1 = extrapolate_current(anchor.power_w, anchor.thrust_n, f_max, v0, n=prop_exponent)
    v1 = voltage_after_sag(v0, i0, i1, r)
    i2 = extrapolate_current(anchor.power_w, anchor.thrust_n, f_max, v1, n=prop_exponent)
    v2 = voltage_after_sag(v0, i0, i2, r)
    return {
        "p_ref_w": anchor.power_w,
        "t_ref_n": anchor.thrust_n,
        "t_target_n": float(f_max),
        "thrust_ratio": float(f_max) / anchor.thrust_n,
        "prop_exponent": float(prop_exponent),
        "power_w": p_at,
        "current_a": i2,
        "current_first_iteration_a": i1,
        "voltage_after_sag_v": v2,
        "r_int_ohm": None if r is None else float(r),
        "estimated": True,
        "note": "`P ∝ T^%.2f` 是**经验外推**（hardware.md §6.2 同一条），不是测量；"
                "电压下沉用 `esr_and_signature` 回归出来的 R 修正一次。" % prop_exponent,
    }


# ------------------------------------------------------------------ 打印


def _print(rep, *, verbose=False):
    print(f"文件      : {rep['file']}")
    print(f"大小/摘要 : {rep['bytes'] / 1e6:.2f} MB  sha256={rep['sha256'][:16]}…")
    p = rep.get("battery_params") or {}
    print(f"内嵌参数  : BAT1_A_PER_V={p.get('BAT1_A_PER_V')} BAT1_V_DIV={p.get('BAT1_V_DIV')} "
          f"BAT1_N_CELLS={p.get('BAT1_N_CELLS')} BAT1_CAPACITY={p.get('BAT1_CAPACITY')} "
          f"BAT_AVRG_CURRENT={p.get('BAT_AVRG_CURRENT')}")

    if rep.get("error"):
        print(f"\n❌ {rep['error']}")
        return

    fw = rep["flight_window"]
    print(f"\n在飞窗口  : {fw['t0']:.1f} – {fw['t1']:.1f} s（{fw['span_s']:.1f} s，{fw['n']} 个电池样本）"
          f"  判据 `flag_armed ∧ ¬landed`")

    v = rep["voltage"]
    print(f"电压      : min {v['min_v']:.2f} / 中位 {v['median_v']:.2f} / max {v['max_v']:.2f} V"
          f"  （样本级 1σ 抖动 {v['noise_1sigma_v'] * 1000:.1f} mV）")
    if v.get("cell_voltage_all_zero"):
        print(f"          ⚠️ 单节电压 `voltage_cell_v[*]` **整份全为 0**（字段在，模拟电源模块不填）"
              f"；`cell_count = {v['cell_count']}` 只是参数")

    c = rep["current"]
    s = c["sustained"]
    if s["value"] is None:
        print("电流      : —— （在飞窗口太短，出不了 1 s 滑窗）")
    else:
        print(f"电流      : 全段可持续最大（1 s 窗中位）= **{s['value']:.3f} A** @ {s['at_s']:.1f} s"
              f"   窗口均值 {c['mean_a']:.3f} A")
        if s.get("at_flight_start"):
            print("            ⚠️ 这个最大值落在**在飞窗口的起点**（解锁/起飞那一瞬）——"
                  "正是票面问 ① 时**点名排除**的那一列。")
            print("               问 ① 的「电量低端」口径请看下面「在飞后半（放电末端）」那一行。")
        print("            ★ 它是**观测到的需求**，不是电池的**能力**：预算（15 A）从没被用掉过，"
              "所以只能当下界。")
        print(f"            单点最大 {s['raw_max']:.3f} A @ {s['raw_max_at_s']:.1f} s"
              f"（★ **不是判据**：单点可以是毛刺）"
              f"   零值样本 {c['exact_zero_samples']}")
        print(f"            样本级 1σ 抖动 {c['noise_1sigma_a']:.3f} A"
              f"   `current_filtered_a` 均值 "
              + (f"{c['filtered_mean_a']:.3f} A" if c["filtered_mean_a"] is not None else "——"))
        for key, lab in (("sustained_first_half", "在飞前半（电量较高）"),
                         ("sustained_last_half", "在飞后半（放电末端）")):
            h = c.get(key)
            if not h or h["sustained_a"]["value"] is None:
                continue
            print(f"            {lab}（{h['window_s'][0]:.1f}–{h['window_s'][1]:.1f} s）："
                  f"可持续 {h['sustained_a']['value']:.3f} A，"
                  f"电压 min {h['voltage_v_min']:.2f} / 中位 {h['voltage_v_median']:.2f} V")

    sig = rep["signature"]
    print("\n判据自检（这一列能不能当判据）")
    if sig["r_int_ohm"] is None:
        print("  ⚠️ 窗口太少，检验做不了（`no-data`）")
    else:
        ident = "可辨识" if sig["fit_is_identifying"] else "**不可辨识**（|corr| ≤ 3/√n ⟹ 这个 R 是噪声的产物，别当测量值引用）"
        print(f"  1 s 分箱回归 V = a + b·I + c·t ⟹ R = {sig['r_int_ohm'] * 1000:.0f} mΩ"
              f"  残差 sd {sig['residual_sd_v'] * 1000:.1f} mV  corr(I,V) = {sig['corr_binned']:+.3f}"
              f"  ⟹ 拟合{ident}")
        print(f"  电流若真有 {sig['noise_current_1sigma_a']:.2f} A 的样本级抖动，"
              f"电压上**至少**该留下 {sig['expected_signature_v'] * 1000:.0f} mV 的印记"
              f"（用 `R_min = {sig['r_min_ohm_ref'] * 1000:.0f} mΩ` 这个下界，"
              f"对「抖是真的」最宽容）；实测电压抖动只有 {sig['noise_voltage_1sigma_v'] * 1000:.1f} mV")
        print(f"  ⟹ 比值 {sig['signature_ratio']:.2f}× ⟹ 观察：**{sig['verdict']}**"
              f"（阈值 3×，实测落在 2.6–3.4× 这个骑线的区间上 —— **两种解释都活着**，"
              f"见工具 README §4）")
    print(f"  ★ 口径（**不依赖上面那条判据**）：{sig['policy']}")
    for k, val in rep["channel_verdicts"].items():
        print(f"  · {k}: {val['verdict']} —— {val['why']}")

    if rep.get("charge"):
        ch = rep["charge"]
        print(f"\n放电      : {ch['discharged_mah_start']:.1f} → {ch['discharged_mah_end']:.1f} mAh"
              f"（Δ {ch['discharged_mah_delta']:.1f} mAh）"
              f"   `remaining`（电压法）{ch['remaining_start']:.3f} → {ch['remaining_end']:.3f}")

    if "hover_anchor" in rep:
        a = rep["hover_anchor"]
        print(f"\n锚点（稳态悬停）: {a['window_s'][0]:.1f}–{a['window_s'][1]:.1f} s  n={a['n']}")
        print(f"  推力 ≡ m·g = {a['thrust_n']:.4f} N（物理给的，不是模型里抄的）")
        print(f"  {a['voltage_v']:.3f} V × {a['current_a']:.3f} A = **{a['power_w']:.1f} W**")
        for n in a["notes"]:
            print(f"    ⚠️ {n}")
        e = rep["extrapolation"]
        print(f"\n外推到 f_max = {e['t_target_n']:.2f} N（= 悬停的 {e['thrust_ratio']:.3f} 倍，"
              f"P ∝ T^{e['prop_exponent']:g}）")
        r_txt = "——" if e["r_int_ohm"] is None else f"{e['r_int_ohm'] * 1000:.0f} mΩ"
        print(f"  功率 ≈ {e['power_w']:.0f} W   电流 ≈ **{e['current_a']:.2f} A**"
              f"（未做电压下沉修正时 {e['current_first_iteration_a']:.2f} A；R = {r_txt}）")
        vd = rep["verdict"]
        print(f"  ⟹ 占额定 {vd['rating_a']:.1f} A 的 **{vd['fraction_of_rating']:.0%}**"
              f"（判据 <75% 算保守）⟹ **{vd['verdict']}**")
        print(f"     ⚠️ 前提：{vd['premise']}")
        print(f"     ⚠️ {vd['t_max_note']}")
    elif rep.get("anchor_error"):
        print(f"\n锚点      : ❌ {rep['anchor_error']}")

    if rep.get("missing"):
        print("\n★ 这次**取不到 / 不能当判据**的量（如实点名）")
        for m in rep["missing"]:
            print(f"  · {m}")


# ------------------------------------------------------------------ 入口


def _iter_targets(target):
    if os.path.isdir(target):
        return sorted(glob.glob(os.path.join(target, "**", "*.ulg"), recursive=True))
    return [target]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="从已归档的飞控 ulog 读电池可持续电流/功率（只读本地文件）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("ulog", help="单个 .ulg，或一个目录（递归找 *.ulg）")
    ap.add_argument("--mass", type=float, default=MASS_DEFAULT, help=f"起飞质量 [kg]，默认 {MASS_DEFAULT}")
    ap.add_argument("--f-max", type=float, default=F_MAX_DEFAULT,
                    help=f"OCP 的 F 上限 [N]（按电池），默认 {F_MAX_DEFAULT}")
    ap.add_argument("--rating", type=float, default=CELL_RATING_A_DEFAULT,
                    help=f"电芯持续放电额定 [A]（Mooch 15 A 口径），默认 {CELL_RATING_A_DEFAULT}")
    ap.add_argument("--v-eps", type=float, default=0.30, help="稳态悬停判据：三维速度模上限 [m/s]")
    ap.add_argument("--tilt-eps-deg", type=float, default=10.0, help="稳态悬停判据：倾角上限 [°]")
    ap.add_argument("--min-duration", type=float, default=2.0, help="稳态悬停段最短时长 [s]")
    ap.add_argument("--window", type=float, default=1.0, help="可持续电流的滑窗宽度 [s]")
    ap.add_argument("--min-n", type=int, default=3, help="滑窗内至少几个样本才算数")
    ap.add_argument("--prop-exponent", type=float, default=PROP_EXPONENT_DEFAULT,
                    help=f"P ∝ T^n 的指数，默认 {PROP_EXPONENT_DEFAULT}")
    ap.add_argument("--json", default=None, help="把完整结果写成 JSON（目录模式下是目录路径）")
    ap.add_argument("--quiet", action="store_true", help="目录模式下一份日志只打一行摘要")
    args = ap.parse_args(argv)

    targets = _iter_targets(args.ulog)
    if not targets:
        print(f"❌ {args.ulog} 下没有 *.ulg")
        return EXIT_NO_DATA

    worst = EXIT_OK
    all_reps = []
    for i, path in enumerate(targets):
        rep, code = analyse(
            path, mass=args.mass, f_max=args.f_max, rating=args.rating,
            v_eps=args.v_eps, tilt_eps_deg=args.tilt_eps_deg,
            min_duration=args.min_duration, window=args.window, min_n=args.min_n,
            prop_exponent=args.prop_exponent,
        )
        all_reps.append(rep)
        if i and not args.quiet:
            print()
        if args.quiet or len(targets) > 1:
            _summary_line(rep)
            if not args.quiet:
                print()
        if not args.quiet or len(targets) == 1:
            _print(rep)
        worst = max(worst, code)

    if args.json:
        out = all_reps[0] if len(all_reps) == 1 else all_reps
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2, sort_keys=True)
        print(f"\n（JSON 已写入 {args.json}）")
    return worst


def _scene_name(path):
    """`bag/<场景>/fcu/*.ulg` → `<场景>`；`bag/old/<场景>/fcu/*.ulg` → `old/<场景>`。

    ★ 保留 `old/` 前缀不是装饰：`3_traj1_hover` 在 `bag/` 与 `bag/old/` 下**同名但是两趟**，
    去掉前缀后摘要里会出现两行一模一样的名字（它们的一次电量差 2 V 以上）。
    """
    parts = os.path.normpath(path).split(os.sep)
    if len(parts) < 3:
        return os.path.basename(path)
    return (parts[-3] if parts[-4] != "old" else f"old/{parts[-3]}") if len(parts) >= 4 \
        else parts[-3]


def _summary_line(rep):
    name = _scene_name(rep["file"])
    if rep.get("error"):
        print(f"— {name}: ❌ {rep['error'][:60]}")
        return
    s = rep["current"]["sustained"]
    cur = "——" if s["value"] is None else f"{s['value']:.2f} A"
    # ★ 摘要行也要带这个标记：批量扫的时候读者只看这一行，不标就会把"起飞那一瞬"读成答案
    mark = "★起" if s.get("at_flight_start") else ""
    lh = (rep["current"].get("sustained_last_half") or {}).get("sustained_a", {})
    end = "" if lh.get("value") is None else f"，放电末 {lh['value']:.2f} A"
    anc = ""
    if "hover_anchor" in rep:
        a = rep["hover_anchor"]
        anc = f"  锚点 {a['voltage_v']:.2f} V × {a['current_a']:.2f} A = {a['power_w']:.0f} W"
    print(f"— {name}: 在飞 {rep['flight_window']['span_s']:.0f} s，可持续 {mark}{cur}"
          f"{end}（单点 max {s['raw_max']:.2f} A）{anc}")


if __name__ == "__main__":
    raise SystemExit(main())
