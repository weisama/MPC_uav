#!/usr/bin/env python3
"""从 rosbag2 包出「同口径跟踪误差」—— `ladder_core` 的 rosbag2 适配层。

用法::

    python3 tools/ladder-baseline/ladder_from_bag.py bag/4_traj2_square --caliber a
    python3 tools/ladder-baseline/ladder_from_bag.py <c 级包> --caliber c-target --json out.json

**口径**（`--caliber`）决定"指令"那一侧读哪个话题：

| 档 | 指令话题 | 实际话题 | 窗口 |
|---|---|---|---|
| `a` | `/mavros/setpoint_raw/local`（Phase 1 的 (A) 级，PX4 内置 PID 闭环） | `/mocap/pose_enu` | `AUTO/TRACK` |
| `c` | `/mpc/reference`（本仓 MPC（模型预测控制）节点喂给求解器的**绝对**参考） | `/mocap/pose_enu` | `AUTO/TRACK` |
| `c-target` | `/control/target`（`output_mode=external` 链上模型预测控制 **原样消费**的绝对目标） | `/mocap/pose_enu` | `AUTO/TRACK` |
| `t-anchored` | `/traj/reference` **+ 重建的锚点**（`reference_source=traj` 链：锚点是闸门首开那一拍的测量） | `/mocap/pose_enu` | 参考覆盖区间 ∩ `AUTO/TRACK`（包里没有 `/control/state` 时取参考首末帧） |

★★ **为什么有 `c-target` 而不是只有 `c`**：录包的话题正则
`^/(mavros|mocap|traj|control|vrpn)/` **不包含 `/mpc/`** ⟹ 实飞包里根本没有
`/mpc/reference`（`c` 在那些包上会直接报"话题不存在"，这是**包的边界**，不是缺陷）。
external 链上 `control` 发的 `/control/target` 就是模型预测控制当拍消费的目标，两档同一 datum。

★★ **口径要按模型预测控制的 `参考=` 选，不能按 control 的 `output_mode` 选，也不能"包里有
`/traj/reference` 就猜 traj 模式"**：2026-10-17 悬停那趟 control 是 `external`（照发
`/control/target`），模型预测控制启动行写的是 **`参考=/control/target`** ⟹ 正确口径是 `c-target`。
若误按 traj 模式重建锚点，两个"原点"实测差 **9.2 cm（x）**，悬停误差会从 **0.0861**
误报成 **0.0537**（低 38%）—— 不是控制器差异，是零点差异，而且两个数都"看起来正常"。

★★ **为什么 `t-anchored` 要重建锚点**：`/traj/reference` 是**相对轨迹原点**的偏移，
不是绝对位置。模型预测控制在 `reference_source=traj` 下把目标算成 `锚点 + 参考`，锚点是
**闸门第一次打开那一拍**的动捕测量，之后不随测量移动。拿 `/traj/reference` 直接当
指令，零点就错在"起飞那一刻飞机已经飘到的位置"—— 本仓系绳包实测差 **约 5 mm（z：4.8 mm 按插值、5.0 mm 按最近帧）**，
看着像跟踪误差、其实是锚点。

只读本地文件。**不碰飞控、不发任何 MAVLink、不启动任何节点。**
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ladder_core import (  # noqa: E402
    anchor_index,
    interp_columns,
    make_grid,
    segment_metrics,
    summarize,
    track_window,
    trim_window,
)

REPO = Path(__file__).resolve().parents[2]

#: `--caliber` → (指令话题, 指令怎么得到, 人话)
#:   `abs`      = 话题本身就是**绝对**位置，直接用
#:   `anchored` = 话题是**相对**偏移，要加锚点（模型预测控制的 `reference_source=traj` 规则）
CALIBERS = {
    "a": ("/mavros/setpoint_raw/local", "abs",
          "(A) 级：PX4 内置级联 PID 的闭环指令"),
    "c": ("/mpc/reference", "abs",
          "(C) 级：本仓 MPC（模型预测控制）节点喂给求解器的绝对参考"),
    "c-target": ("/control/target", "abs",
                 "(C) 级 external 链：control 发给模型预测控制、由模型预测控制原样消费的绝对目标"),
    "t-anchored": ("/traj/reference", "anchored",
                   "(C) 级 traj 链：锚点（闸门首开那一拍的动捕）+ /traj/reference"),
}

STATE_TOPIC = "/control/state"
ACTUAL_TOPIC = "/mocap/pose_enu"
MAVROS_STATE_TOPIC = "/mavros/state"
#: 判据 ①（进度到 1）与「条件」栏（电池区间）要能**从入库证据里复查**，
#: 不能只留在会话记录里 —— 这两条话题包里一直有，读一下就能进 JSON。
PROGRESS_TOPIC = "/traj/progress"
BATTERY_TOPIC = "/mavros/battery"

#: 载荷不是位置、不能走 `_pos_of` 的话题 —— 原消息单独存，位置那一列留 `None`
NON_POSITION_TOPICS = {STATE_TOPIC, MAVROS_STATE_TOPIC, PROGRESS_TOPIC, BATTERY_TOPIC}

MODE_OFFBOARD = "OFFBOARD"

#: ★★ **零点闸门** [m]。两侧的"垂直"必须是同一个 datum、同一个正方向。
#: (A) 侧的 `coordinate_frame` 实测是 `FRAME_LOCAL_NED`(1)，但那**只是标注** ——
#: 2026-10-08 悬停包实测：动捕 z = **+1.011**、同一时刻 setpoint z = **+1.012**，
#: 数值上是 ENU（z 朝上）。谁"照着 `coordinate_frame` 把 z 取反"，
#: `e_z` 会从毫米级变成 2 m 级、而 x/y 一切正常 —— 这类错**只在这一栏露头**。
DATUM_GATE_M = 2.0


@dataclass
class Series:
    t: np.ndarray
    #: ★ `STATE_TOPIC` 的载荷是字符串，`p` 装不下 —— 那一档是 `None`（载荷另存在 `states` 里）
    p: np.ndarray | None

    @property
    def count(self) -> int:
        """条数。**从 `t` 现算**，不另存一个可能对不上的字段。"""
        return int(self.t.size)


def _state_pairs(st: Series, msgs):
    """`/control/state` 的 `(接收时刻, 状态字符串)` —— 两条分支共用同一份窗口规则。"""
    return [(t, m.data) for t, m in zip(st.t, msgs)]


def _pos_of(msg, type_name: str) -> np.ndarray:
    """按消息类型取位置（ENU 三轴）。取不到就报错，**不猜**。"""
    for path in ("pose.position", "position", "point"):
        obj = msg
        ok = True
        for attr in path.split("."):
            if not hasattr(obj, attr):
                ok = False
                break
            obj = getattr(obj, attr)
        if ok and all(hasattr(obj, a) for a in ("x", "y", "z")):
            return np.array([obj.x, obj.y, obj.z], dtype=float)
    raise TypeError(f"不认识的位置字段：{type_name}（试过 pose.position / position / point）")


def read_bag(bag_dir: Path, topics, workdir: Path, optional=()):
    """读 `topics`，返回 `{话题: Series}`、`{话题: 类型名}`、`{话题: 原消息}`。

    ★ `NON_POSITION_TOPICS`（`/control/state` 是字符串、`/mavros/state` 是状态）
    不走 `_pos_of`；它们的 `Series.p` 是 `None`，原消息放在第三个返回值里。
    ★ `optional` 里的话题**包里没有就跳过**（`/control/state` 只在起了 control 的场次有）。

    ★ 每个话题**按自己的接收时刻排序** —— `AUTO/TRACK` 窗口是从 `/control/state`
    的时间轴取的，两条序列必须能按同一时基对齐。
    ★ 接收时刻用 bag 的 record time（`read_next` 的第三个返回值），不用消息自带 stamp：
      前者是同一个时钟，后者各话题的时基不同（动捕那条实测 stamp − recv 恒 ≈ −5 ms）。
    """
    sys.path.insert(0, str(REPO / "tools" / "px4-sitl"))
    from mocap_model import prepare_bag  # noqa: E402  共用的解压 + 改 metadata 助手

    import rosbag2_py  # noqa: E402
    from rclpy.serialization import deserialize_message  # noqa: E402
    from rosidl_runtime_py.utilities import get_message  # noqa: E402

    real = prepare_bag(bag_dir, workdir)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(real), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types and t not in optional]
    if missing:
        raise SystemExit(f"包 {bag_dir} 里没有这些话题：{missing}\n有：{sorted(types)}")
    topics = [t for t in topics if t in types]

    rows = {t: [] for t in topics}
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic in rows:
            rows[topic].append((stamp * 1e-9, deserialize_message(raw, get_message(types[topic]))))

    out = {}
    extras = {}
    for t in topics:
        seq = sorted(rows[t], key=lambda r: r[0])
        # ★ 排序与 `.t` **同一趟**做，两个序列的下标才保证一一对应
        #   （分两趟读、分两趟排，是"看着对齐、其实错位"的经典来源）。
        out[t] = Series(np.array([s for s, _ in seq]), None)
        if t in NON_POSITION_TOPICS:
            extras[t] = [m for _, m in seq]
        else:
            out[t] = Series(out[t].t, np.array([_pos_of(m, types[t]) for _, m in seq]))
    return out, {t: types[t] for t in topics}, extras


def _gate_datum(cmd: Series, act: Series, window) -> None:
    lo, hi = window
    g = np.linspace(lo, hi, 200)
    zc = np.interp(g, cmd.t, cmd.p[:, 2])
    za = np.interp(g, act.t, act.p[:, 2])
    d = float(np.mean(np.abs(zc - za)))
    if d > DATUM_GATE_M:
        raise SystemExit(
            f"零点闸门不过：指令与实际的 z 平均差 {d:.3f} m > {DATUM_GATE_M} m。\n"
            "  两侧还差一个 datum 或一个正方向（ENU↔NED）—— 减出来的不是误差。\n"
            "  先查 coordinate_frame / 锚点，别去调阈值。"
        )


def _anchored_cmd(series, extras):
    """按模型预测控制的 `reference_source=traj` 规则重建**绝对**指令。

    规则（`src/nmpc/nmpc/node_core.py` 的 `_update_anchor`）：
      闸门第一次打开 = 已被解锁 ∧ 模式 OFFBOARD ∧ 已收到 `/traj/reference`。
      锚点 = **那一拍手里最新的一帧动捕**（不是轨迹起点、不是飞机起飞瞬间）。
    之后锚点不变，目标 = 锚点 + `/traj/reference`。

    ★ 为什么这里只重建**一个**锚点：`_update_anchor` 只在 `state_source=px4` 时才会因
      `xy_reset_counter`/`quat_reset_counter` 变化而重钉；2026-10-17 四趟实飞的启动行都是
      **`状态源=mocap`** ⟹ `_state_reset_counters()` 返回 `(None, None)` ⟹ 锚点只钉一次。
      若哪天改用 px4 状态源，本函数需要跟着补重钉逻辑（否则目标会错在"帧换了"那一拍）。

    返回 `(cmd, window, info)`；窗口从**闸门打开**那一刻起（不早于参考首帧）到参考末帧。
    """
    ref = series["/traj/reference"]
    act = series[ACTUAL_TOPIC]
    st_t = series[MAVROS_STATE_TOPIC].t
    st_msgs = extras[MAVROS_STATE_TOPIC]

    off_t = next((float(t) for t, m in zip(st_t, st_msgs)
                  if bool(m.armed) and str(m.mode) == MODE_OFFBOARD), None)
    if off_t is None:
        raise SystemExit(
            f"{MAVROS_STATE_TOPIC} 里从未出现 armed ∧ mode={MODE_OFFBOARD} ⟹ "
            "模型预测控制的闸门从未打开，这个包里没有闭环段。"
        )
    t_gate = max(float(ref.t[0]), off_t)
    i = anchor_index(t_gate, act.t)
    anchor = act.p[i].copy()
    info = {
        "rule": "锚点 = 闸门首开那一拍已收到的最新动捕；目标 = 锚点 + /traj/reference",
        "t_first_reference": float(ref.t[0]),
        "t_first_offboard_armed": off_t,
        "t_gate": t_gate,
        "anchor_t": float(act.t[i]),
        "anchor_xyz": [float(v) for v in anchor],
        # ★ 锚点只能在"最新一帧动捕"里取，误差上限是一帧动捕的周期（z0 实测约 5 ms）。
        #   `/mavros/state` 1 Hz 只影响**哪条状态消息先到**，不影响取哪一帧动捕。
        "mocap_period_s": float(np.median(np.diff(act.t))),
    }
    # ★★ 窗口：有 `/control/state` 就用它的 `AUTO/TRACK`（与 a/c 两档**同一规则**）；
    #   没有（本节点自发起步、没起 control 的场次）才退回参考首末帧。
    #   两者都取之后还要与参考的覆盖区间**相交** —— `/traj/reference` 在 TRACK 结束后
    #   还可能继续发（悬停那趟一直发到降落完），把降落段算进跟踪误差是错的。
    # ★ 窗口从**闸门打开**那一刻起，不是从参考第一帧起：闸门没开时参考不被消费
    #   （本包两者只差 0.2 ms，但规则写对了，别的包才不会多算一段停着的参考）。
    window = (t_gate, float(ref.t[-1]))
    rule = "/traj/reference 首末帧（包里没有 /control/state）"
    if STATE_TOPIC in series and series[STATE_TOPIC].t.size:
        states = _state_pairs(series[STATE_TOPIC], extras[STATE_TOPIC])
        w = track_window(states)
        if w is not None:
            lo, hi = max(w[0], window[0]), min(w[1], window[1])
            if lo < hi:
                window = (float(lo), float(hi))
                rule = f"/traj/reference 覆盖区间 ∩ {STATE_TOPIC} 的 AUTO/TRACK"
    info["window_rule"] = rule
    cmd = Series(ref.t, ref.p + anchor)
    return cmd, window, info


def _readings(series, extras, act, cmd, window, hz):
    """判据 ①/②/③ 与「条件」栏里那些**不在 `summarize` 读数里**的量。

    ★ 这些数以前只写在票面（会话里现查），JSON 里没有 ⟹ 复核查不到。
      这里一次算清、随证据一起入库：
        - `progress_max` —— 轨迹是否走完（判据 ①）；
        - 窗内最低高度与 `armed ∧ OFFBOARD` 条数 —— 判据 ②；
        - 电池电压区间 —— 「条件」栏；
        - 误差峰值出现时刻 —— 瞬态/常驻的分界（`e_z` 峰值在起飞斜坡）。
    """
    lo, hi = window
    out = {}
    if PROGRESS_TOPIC in series:
        vals = np.array([float(m.data) for m in extras[PROGRESS_TOPIC]])
        if vals.size:
            out["progress_max"] = float(vals.max())
    if BATTERY_TOPIC in series:
        v = np.array([float(m.voltage) for m in extras[BATTERY_TOPIC]])
        v = v[np.isfinite(v) & (v > 0.0)]
        if v.size:
            out["battery_v_min"] = float(v.min())
            out["battery_v_max"] = float(v.max())
    if MAVROS_STATE_TOPIC in series:
        st = series[MAVROS_STATE_TOPIC]
        msgs = extras[MAVROS_STATE_TOPIC]
        idx = np.flatnonzero((st.t >= lo) & (st.t <= hi))
        out["state_window_total"] = int(idx.size)
        out["state_window_armed_offboard"] = int(sum(
            1 for i in idx
            if bool(msgs[i].armed) and str(msgs[i].mode) == MODE_OFFBOARD))
    idx = np.flatnonzero((act.t >= lo) & (act.t <= hi))
    if idx.size:
        k = idx[int(np.argmin(act.p[idx, 2]))]
        out["actual_z_min"] = float(act.p[k, 2])
        out["actual_z_min_t_rel"] = float(act.t[k] - lo)
    # 误差峰值时刻：与 `summarize` 用**同一张网格**，免得两边对不上
    g = make_grid(lo, hi, hz)
    e = interp_columns(act.t, act.p, g) - interp_columns(cmd.t, cmd.p, g)
    exy = np.hypot(e[:, 0], e[:, 1])
    k = int(np.argmax(exy))
    out["peak_xy"] = float(exy[k])
    out["peak_xy_t_rel"] = float(g[k] - lo)
    k = int(np.argmax(np.abs(e[:, 2])))
    out["peak_z_abs"] = float(abs(e[k, 2]))
    out["peak_z_t_rel"] = float(g[k] - lo)
    # ③ 的「从哪一刻起才算稳态」：|e_z| 最后一次超过 0.02 m 的时刻（此后一直在带内）
    bad = np.flatnonzero(np.abs(e[:, 2]) > 0.02)
    out["z_abs_settle_t_rel"] = float(g[bad[-1]] - lo) if bad.size else 0.0
    # ② 的「末 10 s 水平净位移」：两端**插值到同一时刻**再减，别拿两条最近邻的样本凑
    if hi - lo > 10.0:
        t0, t1 = hi - 10.0, hi
        d = np.array([np.interp(t1, act.t, act.p[:, k]) - np.interp(t0, act.t, act.p[:, k])
                      for k in (0, 1)])
        out["tail10_dx"], out["tail10_dy"] = float(d[0]), float(d[1])
        out["tail10_net"] = float(np.hypot(d[0], d[1]))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument("--caliber", choices=sorted(CALIBERS), default="a",
                    help="a = (A) 级 setpoint_raw；c = /mpc/reference；"
                         "c-target = /control/target；t-anchored = 锚点 + /traj/reference")
    ap.add_argument("--hz", type=float, default=50.0, help="评估网格频率")
    ap.add_argument("--no-align", action="store_true", help="跳过时间对齐诊断")
    ap.add_argument("--window-skip", type=float, default=0.0,
                    help="按窗口规则切出后，再切掉开头的秒数（默认 0）")
    ap.add_argument("--window-last", type=float, default=None,
                    help="只保留窗口末尾这么多秒（例如阶梯 3 的悬停 30 s）")
    ap.add_argument("--segments", type=int, default=4,
                    help="把窗口等分成几段各报一次（看瞬态 vs 常驻偏差）")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--workdir", type=Path, default=None,
                    help="解压 .db3.zstd 的落地目录（默认 /tmp/z0-ladder/<包名>）")
    args = ap.parse_args(argv)

    cmd_topic, cmd_mode, desc = CALIBERS[args.caliber]
    # ★★ 解压目录必须由**整个相对路径**决定，不能只用 `bag_dir.name`：
    #    `bag/old/3_traj1_hover` 与 `bag/3_traj1_hover` **同名**，
    #    用 `.name` 会让后者复用前者解出来的 `.db3`（`prepare_bag` 见文件在就跳过解压），
    #    于是两个包给出**一模一样**的读数 —— 而且没有任何报错。
    #    第一版就是这么写的，实测踩到（两个不同日期的包读数逐位相同）。
    slug = str(args.bag_dir).strip("/").replace("/", "__")
    workdir = args.workdir or (Path("/tmp/z0-ladder") / slug)
    topics = [ACTUAL_TOPIC, cmd_topic, MAVROS_STATE_TOPIC, PROGRESS_TOPIC, BATTERY_TOPIC]
    # ★ 补充读数那几个话题**缺了不影响误差计算**，一律 optional；
    #   `/control/state` 只在起了 control 的场次有，也 optional。
    optional = {PROGRESS_TOPIC, BATTERY_TOPIC}
    if cmd_mode == "anchored":
        # 锚点规则要用 `/mavros/state` 定闸门 ⟹ 这一档缺了必须报错，不能 optional
        topics.append(STATE_TOPIC)
        optional.add(STATE_TOPIC)
    else:
        topics.append(STATE_TOPIC)
        optional.add(MAVROS_STATE_TOPIC)
    series, types, extras = read_bag(args.bag_dir, topics, workdir, optional=optional)

    act = series[ACTUAL_TOPIC]
    anchor_info = None
    if cmd_mode == "anchored":
        cmd, window, anchor_info = _anchored_cmd(series, extras)
        window_rule = anchor_info["window_rule"]
        base_st = series.get(STATE_TOPIC)
    else:
        st = series[STATE_TOPIC]
        states = _state_pairs(st, extras[STATE_TOPIC])
        window = track_window(states)
        if window is None:
            raise SystemExit(f"{args.bag_dir} 里没有 AUTO/TRACK 段 —— 这个包不是飞轨迹的")
        cmd = series[cmd_topic]
        window_rule = f"{STATE_TOPIC} 里最长的一段 AUTO/TRACK"
        base_st = st

    untrimmed = window
    try:
        window = trim_window(window, args.window_skip, args.window_last)
    except ValueError as e:
        raise SystemExit(str(e))
    trim = {"skip_s": args.window_skip, "last_s": args.window_last,
            "applied": window != untrimmed}

    print(f"包      {args.bag_dir}")
    print(f"口径    {args.caliber} —— {desc}")
    print(f"      指令 {cmd_topic}（{types[cmd_topic]}，{cmd.count} 条）")
    print(f"      实际 {ACTUAL_TOPIC}（{act.count} 条）")
    print(f"窗口    {window_rule}  [{untrimmed[0]:.3f}, {untrimmed[1]:.3f}]"
          f"  {untrimmed[1]-untrimmed[0]:.2f} s")
    if anchor_info is not None:
        a = anchor_info
        print(f"      锚点 t={a['anchor_t']:.4f}（闸门首开 t={a['t_gate']:.4f}）"
              f"  xyz=[{a['anchor_xyz'][0]:+.4f}, {a['anchor_xyz'][1]:+.4f}, "
              f"{a['anchor_xyz'][2]:+.4f}] m，目标=锚点+参考")
    if trim["applied"]:
        print(f"      裁剪    skip={args.window_skip:g} s / last={args.window_last} s"
              f" ⟹ 评估 [{window[0]:.3f}, {window[1]:.3f}]  {window[1]-window[0]:.2f} s")
    if base_st is not None:
        # ★ 窗口的边界精度就是这个话题的周期 —— 把速率一起打出来，
        #   免得有人把"窗口长度"当成精确到毫秒的东西去用。
        dts = np.diff(base_st.t)
        print(f"      窗口边界来自 {STATE_TOPIC}（{base_st.count} 条，周期中位 "
              f"{np.median(dts)*1000:.0f} ms、最大 {dts.max()*1000:.0f} ms）⟹ 两端各 ±1 拍")
    _gate_datum(cmd, act, window)

    r = summarize(act.t, act.p, cmd.t, cmd.p, window, hz=args.hz, align=not args.no_align)
    readings = _readings(series, extras, act, cmd, window, args.hz)
    u = r["unaligned"]
    print("未对齐（验收口径）")
    print(f"     水平 均值 {u['xy_mean']:.4f}  最大 {u['xy_max']:.4f}  p95 {u['xy_p95']:.4f} m")
    print(f"     垂直 均值 {u['z_mean']:+.4f}  |均值| {u['z_absmean']:.4f}  最大 {u['z_max']:.4f} m")
    # 悬停时指令是常量 ⟹ 指令路径长度为 0，比值没有意义，别打出一个 1e9 的"比"
    ratio = (f"{r['path_len_actual']/r['path_len_cmd']:.3f}"
             if r["path_len_cmd"] > 1e-3 else "n/a（指令是常量）")
    print(f"     路径长度  实际 {r['path_len_actual']:.3f} m / 指令 {r['path_len_cmd']:.3f} m（比 {ratio}）")
    print(f"     包络  实际 {r['envelope_xy_actual'][0]:.3f}×{r['envelope_xy_actual'][1]:.3f}"
          f" / 指令 {r['envelope_xy_cmd'][0]:.3f}×{r['envelope_xy_cmd'][1]:.3f} m")
    # ★ 这些读数**不在**上面的误差指标里，但判据 ①② 与「条件」栏直接引用它们
    bits = []
    if "progress_max" in readings:
        bits.append(f"progress_max={readings['progress_max']:.4f}")
    if "battery_v_min" in readings:
        bits.append(f"电池 {readings['battery_v_min']:.2f}–{readings['battery_v_max']:.2f} V")
    if "state_window_total" in readings:
        bits.append(f"窗内 {readings['state_window_armed_offboard']}/"
                    f"{readings['state_window_total']} 条 armed∧OFFBOARD")
    if bits:
        print("     补充  " + "；".join(bits))
    if "actual_z_min" in readings:
        print(f"          实际最低 z {readings['actual_z_min']:.4f} m @ +"
              f"{readings['actual_z_min_t_rel']:.2f} s；峰值 e_xy "
              f"{readings['peak_xy']:.4f} m @ +{readings['peak_xy_t_rel']:.2f} s，"
              f"|e_z| {readings['peak_z_abs']:.4f} m @ +{readings['peak_z_t_rel']:.2f} s")
    if "z_abs_settle_t_rel" in readings:
        tail = ""
        if "tail10_net" in readings:
            tail = (f"；末 10 s 水平净位移 {readings['tail10_net']:.4f} m"
                    f"（dx {readings['tail10_dx']:+.4f}、dy {readings['tail10_dy']:+.4f}）")
        print(f"          |e_z| 最后一次 >0.02 m @ +{readings['z_abs_settle_t_rel']:.2f} s{tail}")
    # 分段只在下面两处用（打印 + JSON），**算一趟**就够了
    segs = segment_metrics(act.t, act.p, cmd.t, cmd.p, window,
                           n_seg=args.segments, hz=args.hz)
    if args.segments > 1:
        print(f"分段（{args.segments} 段，t 从窗口起点算）")
        for seg in segs:
            print(f"     t∈[{seg['t0']:6.2f},{seg['t1']:6.2f}]"
                  f"  水平 均值 {seg['xy_mean']:.4f} 最大 {seg['xy_max']:.4f}"
                  f"  垂直 均值 {seg['z_mean']:+.4f} m")
    if "aligned" in r:
        al = r["aligned"]
        m = al["metrics"]
        hint = "  ★ 曲线是平的 ⟹ 窗内无可对齐的激励，dt* 无意义" if al["flat"] else ""
        print(f"时间对齐（诊断口径，非验收）dt* = {al['dt_s']:+.2f} s"
              f"  coverage_ok={al['coverage_ok']}{hint}")
        print(f"     水平 均值 {m['xy_mean']:.4f}  最大 {m['xy_max']:.4f} m"
              f"   曲线 min/max {al['curve_xy_mean_min']:.4f}/{al['curve_xy_mean_max']:.4f}")

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "bag": str(args.bag_dir),
            "caliber": args.caliber,
            "cmd_topic": cmd_topic,
            "cmd_type": types[cmd_topic],
            "actual_topic": ACTUAL_TOPIC,
            "window_rule": window_rule,
            "window_untrimmed_s": [float(untrimmed[0]), float(untrimmed[1])],
            "window_trim": trim,
            "anchor": anchor_info,
            "n_cmd": cmd.count,
            "n_actual": act.count,
            "segments": segs,
            "readings": readings,
            **r,
        }
        args.json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                             encoding="utf-8")
        print(f"JSON    {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
