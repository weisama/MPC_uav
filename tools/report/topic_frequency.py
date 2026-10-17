#!/usr/bin/env python3
"""话题频率探针：按包记录时间戳重算报告表 5-6 的频率读数。

只读时间戳，不反序列化消息，也不依赖 ROS —— 频率是录制层面的量。
解压复用 `extract.prepare_bag`（`mocap_model` 里那套 zstd + metadata 改写），
包目录的处理口径与数据管线一致。

两个口径都输出，读者按流的特点选：

- `rate_n_over_span_hz`：帧数 ÷（末帧 − 首帧）。动捕/视觉位姿是成簇到达的，
  相邻帧中位间隔会把频率高估到几百 Hz，这个口径才与 Nokov 的出包率相符。
- `median_rate_hz`：1 ÷ 相邻帧中位间隔。50 Hz 的参考与 setpoint 用这个，
  整包平均会被 MANUAL/LAND 的停发段拉低。

用法：
    python3 tools/report/topic_frequency.py bag/4_traj2_square bag/6_free__20261017-070446
    python3 tools/report/topic_frequency.py bag/* --json /tmp/freq.json
"""

from __future__ import annotations

import argparse
import json
import sqlite3
import sys
import tempfile
from pathlib import Path
from typing import Dict, Iterable, List

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import extract  # noqa: E402

DEFAULT_TOPICS = (
    "/vrpn/z0/pose",
    "/traj/start",
    "/traj/reference",
    "/traj/progress",
    "/control/state",
    "/control/target",
    "/mavros/setpoint_raw/local",
    "/mavros/setpoint_raw/attitude",
    "/mavros/vision_pose/pose",
    "/mocap/pose_enu",
    "/mocap/velocity_enu",
    "/mavros/local_position/pose",
    "/mavros/local_position/velocity_local",
    "/mavros/state",
    "/mavros/rc/in",
)


def summarize(t: np.ndarray) -> Dict[str, float]:
    """一条流的频率摘要。时间单位：秒。"""
    t = np.sort(np.asarray(t, dtype=float))
    n = int(t.size)
    if n < 2:
        return {"n": n, "span_s": 0.0, "rate_n_over_span_hz": float("nan"),
                "median_dt_ms": float("nan"), "median_rate_hz": float("nan"),
                "p95_gap_ms": float("nan"), "max_gap_ms": float("nan")}
    span = float(t[-1] - t[0])
    dt_ms = np.diff(t) * 1e3
    return {
        "n": n,
        "span_s": span,
        "rate_n_over_span_hz": n / span if span > 0 else float("nan"),
        "median_dt_ms": float(np.median(dt_ms)),
        "median_rate_hz": 1e3 / float(np.median(dt_ms)),
        "p95_gap_ms": float(np.percentile(dt_ms, 95)),
        "max_gap_ms": float(np.max(dt_ms)),
    }


def _db_path(root: Path) -> Path:
    files = sorted(Path(root).glob("*.db3"))
    if not files:
        raise FileNotFoundError(f"{root} 里找不到 .db3")
    if len(files) > 1:
        raise ValueError(f"{root} 有多个 .db3 分片；本工具只支持单分片，直接失败")
    return files[0]


def read_timestamps(bag_dir: Path, workdir: Path,
                    topics: Iterable[str]) -> Dict[str, np.ndarray]:
    """返回 {话题: 包记录时间戳（epoch 秒）}；包里没有的话题直接跳过。"""
    root = extract.prepare_bag(Path(bag_dir), Path(workdir))
    db = _db_path(root)
    con = sqlite3.connect(str(db))
    try:
        ids = {name: tid for tid, name in
               con.execute("select id, name from topics").fetchall()}
        out: Dict[str, np.ndarray] = {}
        for name in topics:
            if name not in ids:
                continue
            rows = con.execute(
                "select timestamp from messages where topic_id=? order by timestamp",
                (ids[name],)).fetchall()
            out[name] = np.asarray([r[0] for r in rows], dtype=np.int64) * 1e-9
        return out
    finally:
        con.close()


def probe(bag_dir: Path, topics: Iterable[str]) -> Dict[str, object]:
    topics = list(topics)
    with tempfile.TemporaryDirectory(prefix="topic-freq-") as tmp:
        stamps = read_timestamps(bag_dir, Path(tmp), topics)
        return {
            "bag": str(bag_dir),
            "topics": {name: summarize(t) for name, t in stamps.items()},
            "missing": [t for t in topics if t not in stamps],
        }


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument("--topics", nargs="*", default=list(DEFAULT_TOPICS))
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args(argv)

    results = [probe(b, args.topics) for b in args.bags]
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n",
                             encoding="utf-8")

    print(f"{'bag':28s} {'topic':38s} {'n':>7s} {'span_s':>7s} "
          f"{'n/span':>7s} {'med Hz':>7s} {'max gap ms':>10s}")
    for r in results:
        name = Path(r["bag"]).name
        for topic, s in r["topics"].items():
            print(f"{name:28s} {topic:38s} {s['n']:7d} {s['span_s']:7.2f} "
                  f"{s['rate_n_over_span_hz']:7.2f} {s['median_rate_hz']:7.2f} "
                  f"{s['max_gap_ms']:10.1f}")
        for topic in r["missing"]:
            print(f"{name:28s} {topic:38s} {'—':>7s} {'(缺)':>7s}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
