#!/usr/bin/env python3
"""从真实动捕包（`bag/`）里量出**噪声 / 时序 / 延迟**三样模型 —— B 档（ticket 11）回放夹具的输入。

**为什么要有这一层**：操作者 2026-09-29 裁定 B 档的接法是"**包当参考 + 噪声/延迟模型**"
（[11 号票](../../.scratch/phase2/issues/11-sitl-tier-b-seam3.md) Comments 四·2）——
即 `/mocap/pose_enu` 上放的 = **SITL 真值 ⊕ 包里的噪声与时序**。
"包里的"必须是**量出来的**：本文件就是那台仪器，它吐出的 JSON 就是回放节点唯一吃的输入。
⟹ **回放节点不自己量包**（否则"跑的这一场"与"留档的数"会各说各话）。

三样模型各自回答什么：

| 模型 | 从哪量 | 喂给回放节点的哪一处 |
|---|---|---|
| **时序**（逐帧间隔 + 丢帧缺口） | 的场景包 TRACK 段的 `stamp` 差 | 发帧的节奏（**逐帧复现**，不是抽一个分布） |
| **延迟**（`stamp`→记录器收到） | 同一个包的 `stamp` 与 `recv` 之差 | 基延迟（判据 3 在这个数上**再加** 50 ms） |
| **噪声**（逐轴 σ） | `1_static`（噪声基线）与场景包的**高频残差**，取大者 | 每帧叠的高斯噪声 |

★ **包只读、绝不就地动**（ticket 19 §「几条要知道的坑」）：`*.db3.zstd` 一律先解到 `--workdir`
（默认 `/tmp/mocap-model`）再打开 —— `rosbag2_py` **不能**直接开 `.db3.zstd`，
要 `zstd -d` 之后再把 `metadata.yaml` 里的 `relative_file_paths` 改回 `.db3`。

用法（**先 source 工作空间**，`rosbag2_py` 从那儿来）：

    source /opt/ros/humble/setup.bash && source ~/px4_msgs_ws/install/setup.bash
    source install/setup.bash

    python3 tools/px4-sitl/mocap_model.py dump bag/old/3_traj1_hover --noise-bag bag/1_static
    python3 tools/px4-sitl/mocap_model.py dump bag/old/3_traj1_hover --json /tmp/mocap-model-3.json

⚠️ 那份悬停包是 **09-29** 录的，2026-10-08 搬到了 `bag/old/3_traj1_hover/`
（`bag/` 一层每个场景只留最新的一份）。**`bag/3_traj1_hover/` 现在是 10-08 的另一次飞行。**
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

REPO = Path(__file__).resolve().parents[2]
BAG_ROOT = REPO / "bag"

#: 动捕那条话题（Phase 1 的约定，见 `docs/control/phase1-implementation.md`）
POSE_TOPIC = "/mocap/pose_enu"
#: `/control/state` 是 `std_msgs/String`（包里实测；不是自定义消息）
STATE_TOPIC = "/control/state"
#: 轨迹执行态的字符串（包 3 的迁移：`… →AUTO/TRACK→ MANUAL/LAND …`）
TRACK_STATE = "AUTO/TRACK"
#: 轨迹参考（`mavros_msgs/PositionTarget`）—— 判据② 的八字那条靠它当参考。
#: ☠️ 包里它是**相对量、以原点为中心、`z ≡ 0`**（实测：包 4 包络 1.000 × 0.500 m、
#: 1029 帧；包 3 恒为 0、3873 帧。读数与取证方式见 11 号票 Comments 2026-09-30）
#: ⟹ **高度不在里面**，回放侧要按 `--ref-z-offset-m` 叠一个起飞高度（理由同处）。
TRAJ_TOPIC = "/traj/reference"

#: "缺口"的门槛：超过它的帧间隔算一次丢帧（判据里用的是**逐帧复现**，这个门槛只用于汇总表）
GAP_MS = 20.0


# ------------------------------------------------------------------ 读包（只读）


def prepare_bag(bag_dir: Path, workdir: Path) -> Path:
    """把 `*.db3.zstd` 解到 `workdir`，返回**可以交给 `rosbag2_py` 的目录**。

    ★ 三件事必须一起做，少一件 `rosbag2_py` 就报 `Could not find metadata` 或直接崩：
      ① `zstd -d` 解压；② `metadata.yaml` 里 `relative_file_paths` 的 `.db3.zstd` → `.db3`；
      ③ 解出来的文件与 `metadata.yaml` **同目录**。
    ★ 已经解过一次就复用（`workdir` 里 metadata 在、`.db3` 在）—— 重解 27 MB 不值得。
    """
    zst = sorted(bag_dir.glob("*.db3.zstd"))
    if not zst:
        return bag_dir                      # 没压缩（或没包）⟹ 原样返回，让 rosbag2 自己报错
    meta_src = bag_dir / "metadata.yaml"
    if not meta_src.is_file():
        raise FileNotFoundError(f"{bag_dir} 里没有 metadata.yaml —— 这份包不是干净收口的")
    workdir.mkdir(parents=True, exist_ok=True)
    meta_dst = workdir / "metadata.yaml"
    text = meta_src.read_text(encoding="utf-8")
    for z in zst:
        plain = workdir / z.name[: -len(".zstd")]
        if not plain.is_file():
            subprocess.run(["zstd", "-d", "-f", "-q", str(z), "-o", str(plain)], check=True)
    if not meta_dst.is_file() or meta_dst.read_text(encoding="utf-8") != text.replace(".db3.zstd", ".db3"):
        meta_dst.write_text(text.replace(".db3.zstd", ".db3"), encoding="utf-8")
    return workdir


@dataclass
class Trace:
    """一个包里我们关心的两样东西：动捕帧 + 控制状态机的迁移。"""

    stamp_s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    recv_s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    pos: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    quat: np.ndarray = field(default_factory=lambda: np.zeros((0, 4)))   # xyzw
    states: List[Tuple[float, str]] = field(default_factory=list)         # (recv_s, 状态串)
    bag_dir: str = ""
    topics_seen: Dict[str, int] = field(default_factory=dict)

    @property
    def n(self) -> int:
        return int(self.pos.shape[0])

    @property
    def duration_s(self) -> Optional[float]:
        if self.n < 2:
            return None
        return float(self.recv_s[-1] - self.recv_s[0])

    def rate_hz(self) -> Optional[float]:
        d = self.duration_s
        return None if not d else (self.n - 1) / d


def read_trace(bag_dir: Path, workdir: Optional[Path] = None,
               pose_topic: str = POSE_TOPIC, state_topic: str = STATE_TOPIC) -> Trace:
    """一遍扫完包，只留动捕帧与控制状态迁移（其余话题只记条数）。"""
    import importlib                                        # noqa: PLC0415
    import rosbag2_py                                       # noqa: PLC0415
    from rclpy.serialization import deserialize_message      # noqa: PLC0415

    root = prepare_bag(Path(bag_dir), Path(workdir or (Path(tempfile.gettempdir()) / "mocap-model"
                                                       / Path(bag_dir).name)))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    tr = Trace(bag_dir=str(bag_dir))
    stamp, recv, pos, quat, states = [], [], [], [], []
    bag_t0 = None
    while reader.has_next():
        topic, data, t = reader.read_next()
        if bag_t0 is None:
            bag_t0 = t
        tr.topics_seen[topic] = tr.topics_seen.get(topic, 0) + 1
        if topic == pose_topic:
            msg = deserialize_message(data, _msg_class(types, topic, importlib))
            h = msg.header.stamp
            stamp.append(float(h.sec) + float(h.nanosec) * 1e-9)
            recv.append(t * 1e-9)
            p, q = msg.pose.position, msg.pose.orientation
            pos.append((p.x, p.y, p.z))
            quat.append((q.x, q.y, q.z, q.w))
        elif topic == state_topic:
            s = msg_data(data, types, topic, importlib)
            if not states or states[-1][1] != s:                 # 同一状态的重复帧不留（迁移才有意义）
                states.append((t * 1e-9, s))
    tr.stamp_s = np.asarray(stamp, dtype=float)
    tr.recv_s = np.asarray(recv, dtype=float)
    tr.pos = np.asarray(pos, dtype=float).reshape(-1, 3)
    tr.quat = np.asarray(quat, dtype=float).reshape(-1, 4)
    tr.states = states
    return tr


def _msg_class(types: Dict[str, str], topic: str, importlib):
    pkg, _, cls = types[topic].split("/")
    return getattr(importlib.import_module(pkg + ".msg"), cls)


def msg_data(data, types: Dict[str, str], topic: str, importlib) -> str:
    from rclpy.serialization import deserialize_message      # noqa: PLC0415
    return str(deserialize_message(data, _msg_class(types, topic, importlib)).data)


def read_reference(bag_dir: Path, workdir: Optional[Path] = None,
                   topic: str = TRAJ_TOPIC) -> dict:
    """把包里的 `/traj/reference` 读成一条**可以照着重发的序列**（判据② 的八字那条用）。

    ★ 位置/速度/yaw **原样取出、不加任何偏移**。要叠的东西（比如起飞高度 `z`）
      是**回放侧**的口径，不是包里的事实 ⟹ 落在 `mocap_replay.py --ref-z-offset-m` 上。
    ★ 时间用 `header.stamp` 的**相对量**（相对本序列第一帧），与动捕那条同一口径；
      倒退的帧**不修**（源的毛病要如实留在读数里，同 `intervals_s` 的 ★）。
    ★ `type_mask` / `coordinate_frame` **逐帧读**并回报"整段是不是同一个值" ——
      照着重发的前提是它们一致；不一致就得在这里说清楚，而不是挑第一帧当代表。
    ★ 话题不存在 ⟹ 返回 `n = 0` + `why`，**不抛** —— 包 1/2 没有这条话题，
      不该因此挡住整份模型的 dump。
    """
    import importlib                                        # noqa: PLC0415
    import rosbag2_py                                       # noqa: PLC0415
    from rclpy.serialization import deserialize_message      # noqa: PLC0415

    root = prepare_bag(Path(bag_dir), Path(workdir or (Path(tempfile.gettempdir()) / "mocap-model"
                                                       / Path(bag_dir).name)))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(root), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        return {"n": 0, "topic": topic,
                "why": f"包里没有 {topic}（含 traj/mocap 的话题："
                       + ", ".join(sorted(k for k in types if "traj" in k or "mocap" in k)) + "）"}
    cls = _msg_class(types, topic, importlib)
    stamp, pos, vel, yaw, mask, frame = [], [], [], [], [], []
    while reader.has_next():
        topic_i, data, _recv_ns = reader.read_next()
        if topic_i != topic:
            continue
        m = deserialize_message(data, cls)
        h = m.header.stamp
        stamp.append(float(h.sec) + float(h.nanosec) * 1e-9)
        pos.append((float(m.position.x), float(m.position.y), float(m.position.z)))
        vel.append((float(m.velocity.x), float(m.velocity.y), float(m.velocity.z)))
        yaw.append(float(m.yaw))
        mask.append(int(m.type_mask))
        frame.append(int(m.coordinate_frame))
    if not stamp:
        return {"n": 0, "topic": topic, "why": f"{topic} 一帧都没有"}
    t = np.asarray(stamp, dtype=float)
    return {
        "n": len(stamp), "topic": topic, "type": types[topic],
        "type_mask": mask[0], "type_mask_all_same": bool(len(set(mask)) == 1),
        "coordinate_frame": frame[0],
        "coordinate_frame_all_same": bool(len(set(frame)) == 1),
        "t_s": [float(x) for x in (t - t[0])],
        "p": [[float(c) for c in r] for r in pos],
        "v": [[float(c) for c in r] for r in vel],
        "yaw": yaw,
        "dt_p50_ms": float(np.percentile(np.diff(t), 50) * 1e3) if t.size > 1 else 0.0,
        "span_s": float(t[-1] - t[0]) if t.size > 1 else 0.0,
    }


# ------------------------------------------------------------------ 纯函数：三样模型


def intervals_s(stamp_s: np.ndarray) -> np.ndarray:
    """逐帧间隔（秒）。`stamp` 不单调的地方**不修**（那是源的毛病，读数要如实反映）。"""
    return np.diff(np.asarray(stamp_s, dtype=float))


def interval_stats(dt_s: Sequence[float], gap_ms: float = GAP_MS) -> dict:
    """间隔的汇总：均值/中位/p99/最大 + 超过 `gap_ms` 的缺口有几处、共多久。"""
    dt = np.asarray(list(dt_s), dtype=float)
    if dt.size == 0:
        return {"n": 0}
    gaps = dt[dt > gap_ms * 1e-3]
    return {
        "n": int(dt.size),
        "mean_ms": float(dt.mean() * 1e3),
        "p50_ms": float(np.percentile(dt, 50) * 1e3),
        "p99_ms": float(np.percentile(dt, 99) * 1e3),
        "max_ms": float(dt.max() * 1e3),
        "gaps_over_ms": gap_ms,
        "n_gaps": int(gaps.size),
        "gap_total_ms": float(gaps.sum() * 1e3) if gaps.size else 0.0,
        "n_backwards": int((dt < 0).sum()),
    }


def lag_stats(stamp_s: np.ndarray, recv_s: np.ndarray) -> dict:
    """`stamp`→记录器收到 的时延（秒）：均值/中位/p95/最大。

    ★ 这是**记录侧的一条腿**（发布 → DDS → 记录器），不是"动捕从看到到算完"的全链延迟 ——
      后者在包里量不到（要真值才能量），所以回放节点的基延迟就用这个数，且**明说它是什么**。
    """
    lag = np.asarray(recv_s, dtype=float) - np.asarray(stamp_s, dtype=float)
    if lag.size == 0:
        return {"n": 0}
    return {"n": int(lag.size),
            "mean_ms": float(lag.mean() * 1e3),
            "p50_ms": float(np.percentile(lag, 50) * 1e3),
            "p95_ms": float(np.percentile(lag, 95) * 1e3),
            "max_ms": float(lag.max() * 1e3),
            "min_ms": float(lag.min() * 1e3)}


def hf_residual(x: np.ndarray, t: np.ndarray, win_s: float) -> np.ndarray:
    """**高频残差**：减去宽度 `win_s` 的中心滑动均值后的剩余。

    为什么用它当"飞行中的噪声"：包里的动捕流既含测量噪声、也含真机真实的抖动，
    两者在**频谱上分不开**（没有真值可比）。所以本工具量的是"扣掉慢变趋势之后还剩多少"，
    并且**明说**这一条是保守口径：把真机振动也算进了噪声里（对控制器只会更不利）。

    ★★ `win_s` **不是无关紧要的旋钮**，量到的 σ 随它单调变大 —— 实测（见
      `test_mocap_model.py::test_量到的σ随窗口变宽单调变大_口径得连着窗宽一起报`，
      4 s / 2000 点、随机游走）：

          窗 0.05 s → 1.48e-3    0.2 s → 2.91e-3    0.5 s → 4.18e-3

      机理两条同向叠加：① 窗越宽，扣掉的高频越少（残差从**下方**逼近真高频噪声）；
      ② 端点被截断的"边"越宽（窗内均值不再等于窗心值，边宽 ∝ 半个窗）。
      ⟹ **报 σ 必须连着窗宽一起报**（`build_model` 把 `win_s` 写进模型了，就是为这个）；
      并且 `win_s` 要远小于被量的段长，否则 ②会盖过信号（默认 0.2 s 对 60 s 的 TRACK 段
      ⟹ 端点占比 ~0.7%，可忽略）。
    """
    x = np.asarray(x, dtype=float).reshape(len(t), -1)
    t = np.asarray(t, dtype=float)
    n = len(t)
    half = max(1, int(round(0.5 * win_s * (n - 1) / max(t[-1] - t[0], 1e-9))))
    out = np.empty_like(x)
    for i in range(n):
        a, b = max(0, i - half), min(n, i + half + 1)
        out[i] = x[i] - x[a:b].mean(axis=0)
    return out


def noise_sigma(x: np.ndarray, t: np.ndarray, win_s: float) -> np.ndarray:
    """`hf_residual` 的逐列标准差（"这段里的噪声有多大"）。"""
    r = hf_residual(x, t, win_s)
    return r.std(axis=0)


def track_window(states: Sequence[Tuple[float, str]],
                 state: str = TRACK_STATE) -> Optional[Tuple[float, float]]:
    """`/control/state` 的迁移表 → `(TRACK 起点, TRACK 终点)`（秒，记录器时钟）；没走到 ⟹ `None`。

    ★ 不用硬编码秒数：包 3 的 TRACK 是 26.6→86.6 s（`bag/README.md` §③），
      但那是**这一趟**的数。从这里推，重录一份也能自动跟上。
    """
    t0 = None
    for t, s in states:
        if t0 is None and s == state:
            t0 = float(t)
        elif t0 is not None and s != state:
            return (t0, float(t))
    return (t0, float(states[-1][0])) if t0 is not None else None


def slice_window(t: np.ndarray, a: float, b: float) -> np.ndarray:
    """窗口的布尔掩码（闭区间 `[a, b]`）。"""
    t = np.asarray(t, dtype=float)
    return (t >= a) & (t <= b)


# ------------------------------------------------------------------ 建模


def build_model(bag_dir: Path, noise_bag: Optional[Path] = None, *,
                win_s: float = 0.2, workdir: Optional[Path] = None,
                pose_topic: str = POSE_TOPIC) -> dict:
    """量一个包 → 回放节点唯一吃的那份模型（JSON 友好）。"""
    tr = read_trace(bag_dir, workdir=workdir, pose_topic=pose_topic)
    ref = read_reference(bag_dir, workdir=workdir)
    if tr.n < 10:
        raise SystemExit(f"{bag_dir} 的 {pose_topic} 只有 {tr.n} 帧 —— 量不出模型")
    win = track_window(tr.states)
    if win is None:
        t0, t1 = float(tr.stamp_s[0]), float(tr.stamp_s[-1])
        how = "没有 /control/state（或没走到 AUTO/TRACK）⟹ 取全段"
    else:
        t0, t1 = win
        how = f"{TRACK_STATE} 那一窗（由 /control/state 的迁移推出来）"
    m = slice_window(tr.stamp_s, t0, t1)
    dt = intervals_s(tr.stamp_s[m])
    seg_pos, seg_t, seg_quat = tr.pos[m], tr.stamp_s[m], tr.quat[m]

    # 噪声：静态基线 vs 本段的飞行残差，**取大者**（保守；理由见 hf_residual 的 ★）
    sig_fly = noise_sigma(seg_pos, seg_t, win_s)
    sig_stat, stat_src = None, None
    if noise_bag is not None:
        st = read_trace(noise_bag, workdir=workdir)
        if st.n >= 10:
            sig_stat = st.pos.std(axis=0)
            quat_stat = st.quat.std(axis=0)
            stat_src = str(noise_bag)
    sigma = sig_fly if sig_stat is None else np.maximum(sig_fly, sig_stat)

    model = {
        "bag": str(bag_dir),
        "pose_topic": pose_topic,
        "segment": {"t0_s": t0, "t1_s": t1, "how": how,
                    "n_frames": int(seg_pos.shape[0]),
                    "duration_s": float(seg_t[-1] - seg_t[0]) if seg_t.size > 1 else 0.0},
        # ★ 逐帧间隔**逐帧复现**（不抽分布）：包里的丢帧/抖动是它真实的样子，
        #   抽个均值再拼回去就把"真实时序"换成了"我编的时序"。
        "dt_seq_s": [round(float(x), 6) for x in dt],
        "intervals": interval_stats(dt),
        "lag": lag_stats(tr.stamp_s[m], tr.recv_s[m]),
        "noise": {
            "sigma_m": [float(x) for x in sigma],
            "sigma_fly_m": [float(x) for x in sig_fly],
            "sigma_static_m": None if sig_stat is None else [float(x) for x in sig_stat],
            "sigma_static_quat": None if stat_src is None else [float(x) for x in quat_stat],
            "static_bag": stat_src,
            "win_s": win_s,
            "note": "σ 取『本段飞行残差』与『静态基线』的逐轴较大者；残差含真机振动（保守口径）",
        },
        # **这架飞机当时悬在哪**（实测位置，不是指令）—— 悬停那条当"它该在哪儿"的旁证。
        # ⚠️ 与下面 `reference` 是两样东西：这里是**飞机**，那里是**指令**。
        "track": {
            "pos_mean_m": [float(x) for x in seg_pos.mean(axis=0)],
            "pos_std_m": [float(x) for x in seg_pos.std(axis=0)],
            "pos_p2p_m": [float(x) for x in (seg_pos.max(axis=0) - seg_pos.min(axis=0))],
            "quat_mean_xyzw": [float(x) for x in seg_quat.mean(axis=0)],
        },
        # 包里的**指令**参考（`/traj/reference`）—— 判据② 的八字那条照着重发它。
        # ☠️ 它是以原点为中心的**相对量、`z ≡ 0`**（悬停那条更是恒为 0）⟹ **高度不在里面**：
        #   回放侧要按 `--ref-z-offset-m` 叠起飞高度（理由见 11 号票 Comments 2026-09-30）。
        #   包里没有该话题时这里只有 `n = 0` + `why`（不抛）。
        "reference": ref,
        "bag_rate_hz": tr.rate_hz(),
        "states": [[round(t, 3), s] for t, s in tr.states],
    }
    return model


# ------------------------------------------------------------------ CLI


def render_reference(ref: dict) -> List[str]:
    """`/traj/reference` 那一段的读数（纯函数，只吃 `read_reference` 的返回值）。

    ★ 与上面那条「实测」分开印：**一条是指令、一条是飞机** —— 判据② 要重发的是指令那条，
      而"它到底跟没跟"要看两条之差。混印会让读者以为包里的参考就是飞机的轨迹。
    """
    if not ref.get("n"):
        return [f"指令参考：包里没有（{ref.get('why', '?')}）"]
    p = np.asarray(ref["p"], dtype=float)
    v = np.asarray(ref["v"], dtype=float)
    lo, hi = p.min(axis=0), p.max(axis=0)
    out = [
        f"指令参考：{ref['topic']}（{ref['type']}）{ref['n']} 帧、"
        f"{ref['span_s']:.2f} s、间隔中位 {ref['dt_p50_ms']:.2f} ms",
        f"      掩码 {ref['type_mask']}（整段一致：{'是' if ref['type_mask_all_same'] else '**否**'}）、"
        f"坐标系 {ref['coordinate_frame']}（整段一致："
        f"{'是' if ref['coordinate_frame_all_same'] else '**否**'}）",
        "      包围盒 xyz = " + ", ".join(f"[{lo[i]:+.3f}, {hi[i]:+.3f}]" for i in range(3))
        + f" m（z 跨度 {hi[2] - lo[2]:.3f} m）",
        f"      速度 |v| 最大 {float(np.abs(v).max()) if v.size else 0.0:.3f} m/s、"
        f"yaw 范围 [{min(ref['yaw']):+.3f}, {max(ref['yaw']):+.3f}] rad",
        f"      首点 ({ref['p'][0][0]:+.4f}, {ref['p'][0][1]:+.4f}, {ref['p'][0][2]:+.4f}) "
        f"→ 末点 ({ref['p'][-1][0]:+.4f}, {ref['p'][-1][1]:+.4f}, {ref['p'][-1][2]:+.4f}) m",
    ]
    if abs(hi[2] - lo[2]) < 1e-9:
        out.append("      ☠️ **高度不在参考里**（z 恒定）⟹ 回放侧用 `--ref-z-offset-m` 叠起飞高度")
    return out


def render(model: dict, noise_bag: Optional[Path]) -> str:
    seg, itv, lag, nz, tk = (model["segment"], model["intervals"], model["lag"],
                             model["noise"], model["track"])
    L = []
    L.append(f"包：{model['bag']}（{model['pose_topic']} 全程 {model['bag_rate_hz']:.1f} Hz）")
    L.append(f"窗口：{seg['how']}")
    L.append(f"      {seg['t0_s']:.2f} → {seg['t1_s']:.2f} s（{seg['duration_s']:.2f} s、"
             f"{seg['n_frames']} 帧、{len(model['dt_seq_s'])} 个间隔）")
    L.append(f"时序：均 {itv['mean_ms']:.2f} ms、中位 {itv['p50_ms']:.2f}、p99 {itv['p99_ms']:.2f}、"
             f"最大 {itv['max_ms']:.2f} ms；>{itv['gaps_over_ms']:.0f} ms 的缺口 "
             f"{itv['n_gaps']} 处（共 {itv['gap_total_ms']:.0f} ms）；倒退 {itv['n_backwards']} 处")
    L.append(f"延迟：stamp→记录器 均 {lag['mean_ms']:.2f} ms、中位 {lag['p50_ms']:.2f}、"
             f"p95 {lag['p95_ms']:.2f}、最大 {lag['max_ms']:.2f}（**记录侧一条腿**，见函数说明）")
    L.append("噪声：σ 逐轴（本段飞行残差 / 静态基线 / 取用）")
    for i, ax in enumerate("xyz"):
        s_f, s_s = nz["sigma_fly_m"][i], nz["sigma_static_m"]
        L.append(f"      {ax}：{s_f * 1e3:9.5f} mm / "
                 f"{'—' if s_s is None else f'{s_s[i] * 1e3:9.5f} mm'} / "
                 f"{nz['sigma_m'][i] * 1e3:9.5f} mm")
    if nz["sigma_static_quat"] is not None:
        L.append(f"      姿态（静态）σ(xyzw) = "
                 + " ".join(f"{v:.2e}" for v in nz["sigma_static_quat"]))
    L.append(f"实测：这架飞机在窗口里悬在 ({tk['pos_mean_m'][0]:+.3f}, {tk['pos_mean_m'][1]:+.3f}, "
             f"{tk['pos_mean_m'][2]:+.3f}) m，标准差 "
             + ", ".join(f"{v * 1e3:.1f}" for v in tk["pos_std_m"])
             + " mm，峰峰 xyz = " + ", ".join(f"{v * 1e3:.1f}" for v in tk["pos_p2p_m"]) + " mm")
    L.extend(render_reference(model["reference"]))
    L.append("状态机：" + " → ".join(f"{s}@{t:.1f}s" for t, s in model["states"]))
    return "\n".join(L)


def cmd_dump(argv: Sequence[str]) -> int:
    ap = argparse.ArgumentParser(prog="mocap_model.py dump", description=__doc__.splitlines()[0])
    # ⚠️ 默认那份是 **09-29** 的悬停包（本仓冻结的那份 `mocap-model-3.json` 出自它）。
    #    2026-10-08 操作者把 bag/ 清了一遍（每个场景只留最新的一份）⟹ 那份搬去了
    #    `bag/old/3_traj1_hover/`，而 **`bag/3_traj1_hover/` 现在装的是 10-08 另一次飞行的数据**
    #    —— 默认值不改的话，这里会**静默**换一份输入，重新生成的模型与冻结那份对不上，
    #    而症状是"数变了"，不是"路径错了"。
    ap.add_argument("bag", nargs="?", default=str(BAG_ROOT / "old" / "3_traj1_hover"))
    ap.add_argument("--noise-bag", default=str(BAG_ROOT / "1_static"),
                    help="噪声基线包（默认 bag/1_static；给 '' 就只用本段的飞行残差）")
    ap.add_argument("--win-s", type=float, default=0.2, help="高频残差的滑动均值窗宽 [s]")
    ap.add_argument("--json", default=None, help="模型写到哪（回放节点吃这个文件）")
    ap.add_argument("--workdir", default=str(Path(tempfile.gettempdir()) / "mocap-model"))
    args = ap.parse_args(argv)

    noise_bag = Path(args.noise_bag) if args.noise_bag else None
    model = build_model(Path(args.bag), noise_bag, win_s=args.win_s, workdir=Path(args.workdir))
    print(render(model, noise_bag))
    if args.json:
        out = Path(args.json)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(model, indent=1, ensure_ascii=False), encoding="utf-8")
        print(f"\n模型写到 {out}（{out.stat().st_size / 1024:.0f} KiB）")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in ("dump",):
        print(__doc__)
        print("子命令：dump <包目录> [--json 文件] [--noise-bag 包目录] [--win-s 秒]")
        return 2
    return {"dump": cmd_dump}[argv[0]](argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())
