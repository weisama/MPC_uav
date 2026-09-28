#!/usr/bin/env python3
"""从**飞控的 ulog** 估 τ（端到端延迟）—— `tau_core` 的 pyulog 适配层。

τ 的定义：**动捕 (`vehicle_visual_odometry`) 相对飞控 IMU (`sensor_combined`) 的滞后**。
两者都打在飞控时钟上，所以这个差值是同一时基里的真延迟，不含跨机对时误差。

三个量，**必须一起看**：

  A  陀螺 vs 动捕差分角速度   —— 主口径。同一物理量（体轴角速度），无需换算。
  B  加速度计 vs 动捕体轴"下" —— 交叉口径。不同物理量，准静态下 a ≈ −9.81·d̂（**预期负相关**）。
  对照 陀螺 vs 飞控自身姿态差分 —— **这个方法自身的零点**。同一台 IMU、同一个时钟，
       它必须落在 0 附近。它若不为 0，A / B 的数就不能信。

用法::

    python3 tools/tau-analysis/tau_from_ulog.py <log.ulg> [--max-lag 0.3] [--fc 5.0]
                                               [--dt 0.002] [--warmup 3.0]
                                               [--segments 6] [--json out.json]

只读本地文件。**不碰飞控、不发任何 MAVLink。**
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from datetime import datetime, timezone

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tau_core import (  # noqa: E402
    estimate_lag,
    estimate_lag_segmented,
    find_active_span,
    quat_body_down,
    quat_to_rate,
)

# ulog 里的 topic 名（多实例会带 _0 后缀，两种都试）
TOPIC_IMU = ("sensor_combined",)
TOPIC_MOCAP = ("vehicle_visual_odometry_0", "vehicle_visual_odometry")
TOPIC_ATT = ("vehicle_attitude_0", "vehicle_attitude")

G_ACC = 9.80665


# ------------------------------------------------------------------ 读数


def _sha256(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def _load(ulog, names, required, label):
    """按候选名找 dataset，缺了就给一条**说人话**的报错。"""
    for n in names:
        try:
            ds = ulog.get_dataset(n)
        except (KeyError, IndexError):
            continue
        missing = [k for k in required if k not in ds.data]
        if missing:
            raise SystemExit(f"{label}（{n}）缺字段 {missing}；实际字段：{sorted(ds.data)}")
        return n, ds
    raise SystemExit(
        f"这份 ulog 里没有 {label}（找过 {list(names)}）。"
        f"\n用 `python3 -c \"from pyulog import ULog; "
        f"print(sorted(d.name for d in ULog('{ulog.file_name}').data_list))\"` 看有哪些 topic；"
        f"\n多半是 SDLOG_PROFILE 没开 Computer Vision and Avoidance（bit 7）。"
    )


def _cols(ds, base, n, scale=1.0):
    """取 `base[0..n-1]` 拼成 `(N, n)`。"""
    if n == 1:
        return np.asarray(ds.data[base], dtype=float)[:, None] * scale
    return np.stack(
        [np.asarray(ds.data[f"{base}[{i}]"], dtype=float) for i in range(n)], axis=-1
    ) * scale


def _raw_t(ds):
    """ulog 时间戳是**微秒**（PX4 hrt）。"""
    return np.asarray(ds.data["timestamp"], dtype=float)


def _to_seconds(t_us, origin_us):
    """★ 三条序列必须减**同一个**原点。

    曾经每条约各自减自己的 `t[0]` —— 那让三条序列坐在三个不同的时间原点上，
    于是"按动捕算出来的窗口"被拿去切 IMU 的数组，切到的是**另一段时间**。
    错得很安静：脚本照样出数，只是数是别的时段的。
    """
    return (t_us - origin_us) * 1e-6


def _rate_hz(t):
    if t.size < 2:
        return float("nan")
    return float((t.size - 1) / (t[-1] - t[0]))


# ------------------------------------------------------------------ 报告


def _flight_window(u, origin_us):
    """从日志的**第一手状态**圈出在飞的那一段：`flag_armed ∧ ¬landed`。

    为什么不猜：试过用"动捕位移 > 阈值"当启发式，它选中了 140–162 s，
    而那一段陀螺 RMS 只有 0.005 rad/s（飞机没转、只是在被人挪），
    真正在飞的是 307–329 s（陀螺 RMS 0.36 rad/s）。**日志里写着解锁和离地状态，别猜。**

    返回 `(t0, t1, 依据说明)`；两个 topic 都没有则返回 `None` 交给兜底。
    """
    cm = None
    for n in ("vehicle_control_mode", "vehicle_control_mode_0"):
        try:
            cm = u.get_dataset(n)
            break
        except (KeyError, IndexError):
            continue
    ld = None
    for n in ("vehicle_land_detected", "vehicle_land_detected_0"):
        try:
            ld = u.get_dataset(n)
            break
        except (KeyError, IndexError):
            continue
    if cm is None or "flag_armed" not in cm.data:
        return None

    t = _to_seconds(_raw_t(cm), origin_us)
    armed = np.asarray(cm.data["flag_armed"], dtype=int).astype(bool)

    ok = armed
    why = "flag_armed"
    if ld is not None and "landed" in ld.data:
        tl = _to_seconds(_raw_t(ld), origin_us)
        landed = np.asarray(ld.data["landed"], dtype=int).astype(bool)
        # 把 landed 的取值插值到 armed 的时间点上（同一 hrt 时钟，直接插）
        # ⚠️ 这里**不能**用 `except Exception: pass` 兜：一旦插值出问题，窗口会
        #    **静默**退回只看 flag_armed —— 把着陆后的时段也算进去，而调用方看不出来。
        #    改成先显式验前置条件，不满足就出声。
        usable = len(tl) == len(landed) and len(tl) >= 2 and bool(np.all(np.diff(tl) > 0))
        if usable:
            landed_at_t = np.interp(t, tl, landed.astype(float)) > 0.5
            ok = armed & ~landed_at_t
            why = "flag_armed ∧ ¬landed"
        else:
            print("  ⚠️ vehicle_land_detected 时间轴不可用（长度不符或非递增）"
                  "⟹ 窗口退回只看 flag_armed，可能把着陆后的时段也算进来")

    idx = np.flatnonzero(ok)
    if idx.size == 0:
        raise SystemExit(f"日志里有 {why} 的记录，但**没有任何时刻**同时满足 —— 没飞过")
    # 取最长的一段连续（飞行中可能有瞬时掉帧）
    splits = np.flatnonzero(np.diff(t[idx]) > 3.0)
    runs = np.split(idx, splits + 1)
    best = max(runs, key=lambda r: t[r[-1]] - t[r[0]])
    return float(t[best[0]]), float(t[best[-1]]), why


def _fmt(est):
    return f"τ = {est.lag_s * 1e3:+7.1f} ms   r = {est.r_peak:+.3f}   N = {est.n_samples}"


def _seg_line(ests, med, span):
    """逐段打印 `τ(激励σ)` —— **必须同时给激励σ**。

    没有激励的段里相关没有峰，τ 会贴到搜索窗边缘（±`max_lag_s`），
    只看 τ 会把"这段没飞"误读成"方法不稳"。**激励σ 才是那个区分器。**

    ⚠️ 区分器**不是** RMS：整段求 RMS 对单位向量（如 `quat_body_down`）恒等于 1，
    静止与翻滚分不开。见 `tau_core._excitation` 的 docstring（那里记了两个被实测
    证伪的错口径）与 `README.md`「激励σ 是逐通道沿时间求 std」。
    """
    if not ests:
        return "    分段：一段都没成（重叠太短或没有激励）"
    per = ", ".join(f"{e.lag_s * 1e3:+.0f}({e.exc_b:.3f})" for e in ests)
    return (
        f"    分段 τ(激励σ)（{len(ests)} 段）：[{per}]\n"
        f"      中位 {med * 1e3:+.1f} ms  极差 {span * 1e3:.1f} ms"
        f"（★ 极差被低激励段带偏时不要当方法不稳：括号里 σ 最小、τ 又贴着窗边的段就是没激励）"
    )


def _run(label, t_a, a, t_b, b, args, out):
    print(f"\n— {label}")
    try:
        est = estimate_lag(
            t_a, a, t_b, b, dt=args.dt, max_lag_s=args.max_lag, fc=args.fc, warmup_s=args.warmup
        )
    except ValueError as exc:
        print(f"    ✗ {exc}")
        out.append({"label": label, "error": str(exc)})
        return None
    print(f"    {_fmt(est)}")
    print(
        f"    重叠 {est.n_samples * est.dt:.1f} s；激励σ(IMU)={est.exc_a:.4g} "
        f"激励σ(动捕)={est.exc_b:.4g}"
    )
    try:
        ests, med, span = estimate_lag_segmented(
            t_a, a, t_b, b, n_segments=args.segments, dt=args.dt,
            max_lag_s=args.max_lag, fc=args.fc, warmup_s=args.warmup,
        )
    except ValueError:
        ests, med, span = [], float("nan"), float("nan")
    print(_seg_line(ests, med, span))
    rec = {
        "label": label,
        "tau_s": est.lag_s,
        "r_peak": est.r_peak,
        "n": est.n_samples,
        "dt": est.dt,
        "exc_imu": est.exc_a,
        "exc_mocap": est.exc_b,
        "segments": [e.lag_s for e in ests],
        "seg_median_s": med,
        "seg_span_s": span,
    }
    out.append(rec)
    return rec


# ------------------------------------------------------------------ 主流程


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulog", help="本地 .ulg 路径")
    ap.add_argument("--max-lag", type=float, default=0.3, help="搜索半径（秒），默认 0.3")
    ap.add_argument("--fc", type=float, default=5.0, help="零相位低通截止（Hz）；0 = 不滤")
    ap.add_argument("--dt", type=float, default=0.002, help="统一重采样步长（秒），默认 2 ms")
    ap.add_argument("--warmup", type=float, default=3.0, help="丢掉开头几秒（起飞/解锁瞬态），默认 3")
    ap.add_argument("--window", type=float, nargs=2, metavar=("T0", "T1"),
                    help="手工指定分析窗口（秒，从日志第一条算起）；不给则自动圈活动段")
    ap.add_argument("--move-eps", type=float, default=0.05,
                    help="判定'在动'的位移阈值（米/2秒），默认 0.05")
    ap.add_argument("--segments", type=int, default=6, help="分段数，默认 6")
    ap.add_argument("--json", help="把结果另存成 JSON")
    # 覆盖 topic 名：给 SITL 日志（没有动捕）或换名日志做调试用，正常飞不用管
    ap.add_argument("--imu-topic", help=f"覆盖 IMU topic（默认 {'/'.join(TOPIC_IMU)}）")
    ap.add_argument("--mocap-topic", help=f"覆盖动捕 topic（默认 {'/'.join(TOPIC_MOCAP)}）")
    ap.add_argument("--att-topic", help=f"覆盖姿态 topic（默认 {'/'.join(TOPIC_ATT)}）")
    args = ap.parse_args()

    from pyulog import ULog

    path = args.ulog
    if not os.path.isfile(path):
        raise SystemExit(f"没有这个文件：{path}")

    u = ULog(path)
    info = u.msg_info_dict
    digest = _sha256(path)  # 只算一次：这文件动辄几十 MB，而打印与落 JSON 都要它
    print("=" * 78)
    print("τ 估计 —— 飞控 IMU vs 动捕")
    print("=" * 78)
    print(f"日志文件   : {os.path.abspath(path)}")
    print(f"大小       : {os.path.getsize(path):,} 字节")
    print(f"sha256     : {digest}")
    print(f"文件 mtime : {datetime.fromtimestamp(os.path.getmtime(path)).isoformat()}")
    for k in ("ver_sw", "ver_hw", "sys_name", "time_ref_utc", "log_file"):
        if k in info:
            v = info[k]
            if k == "time_ref_utc":
                try:
                    v = f"{v}  ({datetime.fromtimestamp(int(v), tz=timezone.utc).isoformat()})"
                except (TypeError, ValueError, OSError):
                    pass
            print(f"{k:<10} : {v}")

    # ---- 取三条序列
    imu_name, imu = _load(
        u, (args.imu_topic,) if args.imu_topic else TOPIC_IMU,
        ("timestamp", "gyro_rad[0]", "accelerometer_m_s2[0]"), "飞控 IMU",
    )
    moc_name, moc = _load(
        u, (args.mocap_topic,) if args.mocap_topic else TOPIC_MOCAP,
        ("timestamp", "q[0]"), "动捕里程计",
    )
    att_name, att = _load(
        u, (args.att_topic,) if args.att_topic else TOPIC_ATT,
        ("timestamp", "q[0]"), "飞控姿态",
    )

    # ★ 统一原点：取三条序列里最早的那一条 —— 全部 topic 都减它。
    origin = min(_raw_t(imu)[0], _raw_t(moc)[0], _raw_t(att)[0])
    t_imu = _to_seconds(_raw_t(imu), origin)
    t_moc = _to_seconds(_raw_t(moc), origin)
    t_att = _to_seconds(_raw_t(att), origin)

    gyro = _cols(imu, "gyro_rad", 3)
    accel = _cols(imu, "accelerometer_m_s2", 3)
    q_moc = _cols(moc, "q", 4)
    q_att = _cols(att, "q", 4)

    for nm, t in ((imu_name, t_imu), (moc_name, t_moc), (att_name, t_att)):
        print(f"{nm:<26}: {t.size:>8} 条，{_rate_hz(t):6.1f} Hz，{t[-1] - t[0]:.1f} s")

    print(
        "\n★ 时间基说明：两条序列都用 `timestamp` 字段，即 **PX4 收到该消息的时刻**\n"
        "（`mavlink_receiver.cpp` 里 `odom.timestamp = hrt_absolute_time()`），\n"
        "与 IMU 同一个 hrt 时钟 ⟹ 这里量到的差值就是真延迟，不含跨机对时误差。\n"
        "不用 `timestamp_sample`（那是动捕自报的采样时刻，经 timesync 跨域换算，另有一层不确定度）。"
    )

    # ---- 先把要差分的量算出来（与窗口无关，窗口是对结果切片）
    # A：陀螺 vs 动捕差分角速度（主口径）
    t_moc_rate, moc_rate = quat_to_rate(t_moc, q_moc, attrib="midpoint")
    # 对照：陀螺 vs 飞控自身姿态差分
    t_att_rate, att_rate = quat_to_rate(t_att, q_att, attrib="midpoint")

    # ---- 圈活动段：飞机停在跑道/桌上的那几分钟必须剔掉，
    #      否则"一堆零"会把相关稀释到 |r|≈0，连对照都过不了。
    print("\n" + "=" * 78)
    print("分析窗口")
    print("=" * 78)
    pos = _cols(moc, "position", 3)
    if args.window:
        w0, w1 = args.window
        print(f"手工指定：{w0:.1f} s — {w1:.1f} s（{w1 - w0:.1f} s）")
    else:
        fw = _flight_window(u, origin)
        if fw is not None:
            w0, w1, why = fw
            print(f"取自日志第一手状态（{why}）的最长一段：{w0:.1f} s — {w1:.1f} s  =  {w1 - w0:.1f} s")
        else:
            try:
                w0, w1 = find_active_span(t_moc, pos, move_eps=args.move_eps)
            except ValueError as exc:
                raise SystemExit(f"日志里没有解锁状态 topic，位移启发式也圈不出活动段：{exc}")
            print(f"⚠ 日志里没有解锁状态 topic，退用位移启发式：{w0:.1f} s — {w1:.1f} s = {w1 - w0:.1f} s")
        # 交叉核对：位移启发式会选哪一段？两下不一致就说明其中一个是错的
        try:
            h0, h1 = find_active_span(t_moc, pos, move_eps=args.move_eps)
            if not (h1 < w0 or h0 > w1):
                print(f"  ✓ 位移启发式选的 {h0:.1f}—{h1:.1f} s 与之有交集")
            else:
                print(
                    f"  ★ 位移启发式选的是 {h0:.1f}—{h1:.1f} s，与上面**没有交集** —— "
                    f"两个判据打架，别急着信任何一边，先看清楚哪些时段飞机真在动。"
                )
        except ValueError:
            pass

    def clip(t, x):
        m = (t >= w0) & (t <= w1)
        return t[m] - w0, x[m]

    t_imu_w, gyro_w = clip(t_imu, gyro)
    _, accel_w = clip(t_imu, accel)
    t_moc_w, q_moc_w = clip(t_moc, q_moc)
    t_moc_rate_w, moc_rate_w = clip(t_moc_rate, moc_rate)
    t_att_rate_w, att_rate_w = clip(t_att_rate, att_rate)

    results = []
    print("\n" + "=" * 78)
    print("三个量")
    print("=" * 78)

    # A
    a_res = _run("A  陀螺(sensor_combined) vs 动捕差分角速度",
                 t_imu_w, gyro_w, t_moc_rate_w, moc_rate_w, args, results)

    # 动捕若自带 angular_velocity 且非零，是**不需要差分**的同物理量口径 —— 一并报
    if "angular_velocity[0]" in moc.data:
        av = _cols(moc, "angular_velocity", 3)
        if np.abs(av).max() > 1e-6:
            print("\n    （动捕消息自带 angular_velocity 且非零 —— 差分之外再量一次，不经过求导）")
            _, av_w = clip(t_moc, av)
            _run("A' 陀螺 vs 动捕自报 angular_velocity", t_imu_w, gyro_w, t_moc_w, av_w, args, results)
        else:
            print("\n    （动捕消息的 angular_velocity 全 0 —— 飞控侧的填充值，不能用作口径）")

    # B：加速度计 vs 动捕体轴"下"（交叉口径，预期负相关）
    _, moc_down_w = clip(t_moc, quat_body_down(q_moc))
    _run("B  加速度计 vs 动捕体轴'下'（预期 r<0）",
         t_imu_w, accel_w, t_moc_w, moc_down_w, args, results)

    # 对照：陀螺 vs 飞控自身姿态差分 —— 这个方法自己的零点
    ctl = _run("对照 陀螺 vs **飞控自身**姿态差分（应为 0）",
               t_imu_w, gyro_w, t_att_rate_w, att_rate_w, args, results)

    # ---- 结论
    print("\n" + "=" * 78)
    print("读法")
    print("=" * 78)
    if ctl is None:
        print("★ 对照没跑成 ⟹ A / B 的数**不能采信**：连方法自身的零点都验不了。")
    elif abs(ctl["tau_s"]) > 0.02:
        print(
            f"★ 对照落在 {ctl['tau_s'] * 1e3:+.1f} ms，离 0 太远。同一台 IMU、同一个时钟本该是 0，"
            f"\n  现在不为 0 ⟹ **问题出在这个方法上**（重采样/滤波/求导任一步有偏），"
            f"\n  A / B 的数一并不采信，先查方法。"
        )
    else:
        print(f"✓ 对照 {ctl['tau_s'] * 1e3:+.1f} ms，贴着 0 ⟹ 方法自身没有系统性偏移，A / B 可采信。")

    # A 的可信度**不是**拿一个绝对阈值去卡，而是拿**对照的离散度**当这把尺子的噪声底：
    # 同一套重采样/滤波/求导流程跑在"本该为 0"的数据上散多大，A 散多大于此就说明另有原因。
    ctl_span = ctl["seg_span_s"] if ctl else float("nan")
    gate = max(0.030, 3.0 * (0.0 if math.isnan(ctl_span) else ctl_span))
    print(f"\n分段极差：A {a_res['seg_span_s'] * 1e3:.1f} ms vs 对照 "
          f"{ctl_span * 1e3:.1f} ms（门槛 = max(30 ms, 3×对照极差) = {gate * 1e3:.0f} ms）"
          if a_res else "")

    if a_res is None:
        print("★ 主口径 A 没跑成 ⟹ 没有结论。")
    elif abs(a_res["r_peak"]) < 0.5:
        print(f"★ 主口径 A 的相关峰只有 r = {a_res['r_peak']:+.3f} ⟹ 两条序列根本没对上，"
              "\n  这段飞行对 τ **不够有激励**（或坐标系约定不符）。不要拿它当结论，"
              "\n  换一段机动更丰富的日志，或把结论写成「未量出」。")
    elif a_res["seg_span_s"] > gate:
        print(f"★ 主口径 A 分段散得厉害（极差 {a_res['seg_span_s'] * 1e3:.0f} ms > 门槛 "
              f"{gate * 1e3:.0f} ms）⟹ 整段给的那个 τ 里混着**没有激励的时段**，"
              "\n  别单独用。逐段看哪几段有激励，或缩小窗口只留有激励的那一段再量。")
    else:
        print(f"✓ 主口径 A：τ = {a_res['lag_s'] * 1e3:+.1f} ms，r = {a_res['r_peak']:+.3f}，"
              f"分段极差 {a_res['seg_span_s'] * 1e3:.0f} ms（在门槛 {gate * 1e3:.0f} ms 内）"
              f"⟹ 可采信。")

    if args.json:
        with open(args.json, "w") as f:
            json.dump(
                {
                    "ulog": os.path.abspath(path),
                    "sha256": digest,
                    "args": vars(args),
                    "results": results,
                },
                f,
                indent=2,
                ensure_ascii=False,
            )
        print(f"\n结果已存：{args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
