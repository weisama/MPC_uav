#!/usr/bin/env python3
"""从 rosbag2 包出「同口径跟踪误差」—— `ladder_core` 的 rosbag2 适配层。

用法::

    python3 tools/ladder-baseline/ladder_from_bag.py bag/4_traj2_square --caliber a
    python3 tools/ladder-baseline/ladder_from_bag.py <c 级包> --caliber c --json out.json

**口径**（`--caliber`）决定"指令"那一侧读哪个话题：

| 档 | 指令话题 | 实际话题 |
|---|---|---|
| `a` | `/mavros/setpoint_raw/local`（Phase 1 的 (A) 级，PX4 内置 PID 闭环） | `/mocap/pose_enu` |
| `c` | `/mpc/reference`（本仓 MPC（模型预测控制）节点实际喂给求解器的参考那一拍） | `/mocap/pose_enu` |

窗口一律取 `/control/state` 里**最长的一段 `AUTO/TRACK`**。

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

from ladder_core import segment_metrics, summarize, track_window  # noqa: E402

REPO = Path(__file__).resolve().parents[2]

#: `--caliber` → (指令话题, 人话)
CALIBERS = {
    "a": ("/mavros/setpoint_raw/local", "(A) 级：PX4 内置级联 PID 的闭环指令"),
    "c": ("/mpc/reference", "(C) 级：本仓 MPC（模型预测控制）节点当拍喂给求解器的参考"),
}

STATE_TOPIC = "/control/state"
ACTUAL_TOPIC = "/mocap/pose_enu"

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


def read_bag(bag_dir: Path, topics, workdir: Path):
    """读 `topics`，返回 `{话题: Series}` 与 `{话题: 类型名}`。

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
    missing = [t for t in topics if t not in types]
    if missing:
        raise SystemExit(f"包 {bag_dir} 里没有这些话题：{missing}\n有：{sorted(types)}")

    rows = {t: [] for t in topics}
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic in rows:
            rows[topic].append((stamp * 1e-9, deserialize_message(raw, get_message(types[topic]))))

    out = {}
    for t in topics:
        seq = sorted(rows[t], key=lambda r: r[0])
        if t == STATE_TOPIC:
            # ★ 状态那一路的载荷是字符串，`Series.p` 装不下 —— 单独放 `states`。
            #   排序与 `.t` **同一趟**做，两个序列的下标才保证一一对应
            #   （分两趟读、分两趟排，是"看着对齐、其实错位"的经典来源）。
            out[t] = Series(np.array([s for s, _ in seq]), None)
            states = [(s, m.data) for s, m in seq]
        else:
            out[t] = Series(np.array([s for s, _ in seq]),
                            np.array([_pos_of(m, types[t]) for _, m in seq]))
    return out, {t: types[t] for t in topics}, states


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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag_dir", type=Path)
    ap.add_argument("--caliber", choices=sorted(CALIBERS), default="a",
                    help="a = (A) 级 setpoint_raw；c = (C) 级 /mpc/reference")
    ap.add_argument("--hz", type=float, default=50.0, help="评估网格频率")
    ap.add_argument("--no-align", action="store_true", help="跳过时间对齐诊断")
    ap.add_argument("--segments", type=int, default=4,
                    help="把窗口等分成几段各报一次（看瞬态 vs 常驻偏差）")
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--workdir", type=Path, default=None,
                    help="解压 .db3.zstd 的落地目录（默认 /tmp/z0-ladder/<包名>）")
    args = ap.parse_args(argv)

    cmd_topic, desc = CALIBERS[args.caliber]
    # ★★ 解压目录必须由**整个相对路径**决定，不能只用 `bag_dir.name`：
    #    `bag/old/3_traj1_hover` 与 `bag/3_traj1_hover` **同名**，
    #    用 `.name` 会让后者复用前者解出来的 `.db3`（`prepare_bag` 见文件在就跳过解压），
    #    于是两个包给出**一模一样**的读数 —— 而且没有任何报错。
    #    第一版就是这么写的，实测踩到（两个不同日期的包读数逐位相同）。
    slug = str(args.bag_dir).strip("/").replace("/", "__")
    workdir = args.workdir or (Path("/tmp/z0-ladder") / slug)
    series, types, states = read_bag(args.bag_dir, [STATE_TOPIC, ACTUAL_TOPIC, cmd_topic], workdir)

    window = track_window(states)
    if window is None:
        raise SystemExit(f"{args.bag_dir} 里没有 AUTO/TRACK 段 —— 这个包不是飞轨迹的")

    cmd, act = series[cmd_topic], series[ACTUAL_TOPIC]
    print(f"包      {args.bag_dir}")
    print(f"口径    {args.caliber} —— {desc}")
    print(f"      指令 {cmd_topic}（{types[cmd_topic]}，{cmd.count} 条）")
    print(f"      实际 {ACTUAL_TOPIC}（{act.count} 条）")
    print(f"窗口    AUTO/TRACK  [{window[0]:.3f}, {window[1]:.3f}]  {window[1]-window[0]:.2f} s")
    # ★ `窗口` 的边界精度就是这个话题的周期 —— 把速率一起打出来，
    #   免得有人把"窗口长度"当成精确到毫秒的东西去用。
    st = series[STATE_TOPIC]
    dts = np.diff(st.t)
    print(f"      窗口边界来自 {STATE_TOPIC}（{st.count} 条，周期中位 "
          f"{np.median(dts)*1000:.0f} ms、最大 {dts.max()*1000:.0f} ms）⟹ 两端各 ±1 拍")
    _gate_datum(cmd, act, window)

    r = summarize(act.t, act.p, cmd.t, cmd.p, window, hz=args.hz, align=not args.no_align)
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
            "n_cmd": cmd.count,
            "n_actual": act.count,
            "segments": segs,
            **r,
        }
        args.json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                             encoding="utf-8")
        print(f"JSON    {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
