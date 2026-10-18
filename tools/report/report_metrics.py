#!/usr/bin/env python3
"""票 15：第 10 章的结果表 —— 从 CSV/JSON/求解摘要现算，正文不手抄数。

产出一份 `docs/report/generated/metrics.md`，并把同样的块写进
`docs/report/chapters/10-results.md` 的 `<!-- BEGIN phase5-metrics:<key> -->`
块里；两处的块逐字节相同（测试会核）。章节里的散文只做比较与解释，
具体数值一律在块里。

口径（与第 9 章一致，不在本脚本里另立一套）：

- 位置误差 = 实际 − 目标，公共窗、100 Hz 网格、轨迹原点相对系；
- 逐轴 RMSE 与本脚本外 `metrics.position_error` 的 h/z 统计互相印证：
  本脚本算完与 `data/<scene>.json` 的 `metrics_vs_target.position` 逐值比对，
  不一致就报错，避免表格与管线各说一套；
- 姿态误差取四元数最小旋转角，并与同名 JSON 的 `metrics_vs_target.attitude` 比对；
- 控制量只统计模型预测控制两源；饱和阈值直接读 `src/nmpc/nmpc/params.py`
  （不另抄一份数），判据复刻 `node_core.saturated_bounds` 的 `SAT_TOL_REL=1e-3`；
- 仿真的饱和拍数取 `solver-summary.json` 的 `saturation.frames_with_sat`（第 9 章 9.7.3）；
  实飞在 100 Hz 网格上按同一判据逐拍计数。

用法：

    python3 tools/report/report_metrics.py

产物：

    docs/report/generated/metrics.md
    docs/report/chapters/10-results.md        # 只替换 phase5-metrics:* 块
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import metrics as metrics_mod  # noqa: E402
import plot_traj  # noqa: E402
from captions import replace_section  # noqa: E402

SCENES = plot_traj.SCENES
SCENE_CN = plot_traj.SCENE_TITLE_CN

#: 位置表与控制表的行序；目标不是测量，不进表。
POSITION_SOURCES = ("offboard", "mpc_sim", "mpc_real")
MPC_SOURCES = ("mpc_sim", "mpc_real")

#: 三趟标称 SITL 的求解汇总（票 02）。
SOLVER_DIR = REPO / "tools/report/evidence/sitl"
SOLVER_SCENE = {
    "hover": "R-hover",
    "square": "R-square",
    "lemniscate": "R-lemniscate",
}

DATA_DIR = REPO / "docs/report/data"
GENERATED = REPO / "docs/report/generated/metrics.md"
CHAPTER = REPO / "docs/report/chapters/10-results.md"

PREFIX = "phase5-metrics"

GENERATED_HEADER = """# 第 10 章结果表（自动生成）

> 由 `tools/report/report_metrics.py` 从 `docs/report/data/*.csv`、同名 `.json`
> 与 `tools/report/evidence/sitl/*/solver-summary.json` 现算。**不要手改**：
> 重跑脚本会同时替换本文件与第 10 章里的 `phase5-metrics:*` 块。
"""

#: 与 `node_core.SAT_TOL_REL` 同值的生产容差。
SAT_TOL_REL = 1e-3


# ─────────────────────────── 参数（生产阈值） ───────────────────────────


def load_z0_params():
    """直接按文件加载 `params.py`，绕开 `nmpc/__init__.py` 的 ROS/OCP 依赖。"""
    path = REPO / "src/nmpc/nmpc/params.py"
    spec = importlib.util.spec_from_file_location("z0_params_for_report", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"读不到生产参数文件：{path}")
    module = importlib.util.module_from_spec(spec)
    # dataclass 在装饰时要能从 `sys.modules` 找回本模块；不登记会在 `@dataclass` 处炸。
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.Z0Params()


def saturated_bounds(u: Sequence[float], z0) -> Tuple[str, ...]:
    """复刻 `node_core.saturated_bounds`（阈值来自生产 `Z0Params`）。"""
    f, wx, wy, wz = (float(v) for v in u)
    out: List[str] = []
    if abs(f) <= SAT_TOL_REL:
        out.append("F_min")
    elif abs(f - z0.f_max) <= SAT_TOL_REL * max(1.0, abs(z0.f_max)):
        out.append("F_max")
    for name, w, w_max in (("wx", wx, z0.roll_rate_max),
                           ("wy", wy, z0.pitch_rate_max),
                           ("wz", wz, z0.yaw_rate_max)):
        if min(abs(w - w_max), abs(w + w_max)) <= SAT_TOL_REL * max(1.0, abs(w_max)):
            out.append(name)
    return tuple(out)


# ─────────────────────────── 读数据 ───────────────────────────


def _float(value: str) -> float:
    return float(value) if value.strip() else float("nan")


def read_scene(scene: str) -> Dict[str, Dict[str, np.ndarray]]:
    """按源切 CSV；空单元格读成 NaN（目标/offboard 没有控制列）。"""
    path = DATA_DIR / f"{scene}.csv"
    fields = ("t_s", "x_m", "y_m", "z_m", "qw", "qx", "qy", "qz",
              "u_thrust_n", "wx_radps", "wy_radps", "wz_radps")
    out: Dict[str, Dict[str, np.ndarray]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            source = row["source"]
            series = out.setdefault(source, {key: [] for key in fields})
            for key in fields:
                series[key].append(_float(row[key]))
    return {
        source: {key: np.asarray(values, dtype=float) for key, values in series.items()}
        for source, series in out.items()
    }


def read_metadata(scene: str) -> dict:
    return json.loads((DATA_DIR / f"{scene}.json").read_text(encoding="utf-8"))


def read_solver(scene: str) -> dict:
    path = SOLVER_DIR / SOLVER_SCENE[scene] / "solver-summary.json"
    return json.loads(path.read_text(encoding="utf-8"))


# ─────────────────────────── 指标 ───────────────────────────


def _position(data: Dict[str, Dict[str, np.ndarray]], source: str, meta: dict) -> dict:
    target = data["target"]
    actual = data[source]
    target_p = np.column_stack([target["x_m"], target["y_m"], target["z_m"]])
    actual_p = np.column_stack([actual["x_m"], actual["y_m"], actual["z_m"]])
    stats = metrics_mod.position_error(target_p, actual_p)
    err = actual_p - target_p
    stats["rmse_m"] = [float(np.sqrt(np.mean(err[:, axis] ** 2))) for axis in range(3)]

    # 与管线 JSON 对账：表格不能与 `data/<scene>.json` 各说一套。
    published = meta["sources"][source]["metrics_vs_target"]["position"]
    for key in ("h_mean_m", "h_max_m", "z_mean_abs_m", "z_max_abs_m"):
        if not math.isclose(stats[key], published[key], rel_tol=1e-6, abs_tol=1e-9):
            raise ValueError(
                f"{source} 的 {key} 与管线 JSON 不一致："
                f"{stats[key]!r} vs {published[key]!r}"
            )
    stats["completion_s"] = float(meta["sources"][source]["completion_s"])
    stats["completion_reason"] = meta["sources"][source]["completion_reason"]
    return stats


def _attitude(data: Dict[str, Dict[str, np.ndarray]], source: str, meta: dict) -> dict:
    def quat(series):
        return np.column_stack([series["qw"], series["qx"],
                                series["qy"], series["qz"]])

    stats = metrics_mod.attitude_error_deg(quat(data["target"]), quat(data[source]))
    published = meta["sources"][source]["metrics_vs_target"]["attitude"]
    for key in ("angle_mean_deg", "angle_max_deg"):
        if not math.isclose(stats[key], published[key], rel_tol=1e-6, abs_tol=1e-9):
            raise ValueError(
                f"{source} 的 {key} 与管线 JSON 不一致："
                f"{stats[key]!r} vs {published[key]!r}"
            )
    return stats


def _control(data: Dict[str, Dict[str, np.ndarray]], source: str) -> dict:
    series = data[source]
    rate = np.column_stack([series["wx_radps"], series["wy_radps"],
                            series["wz_radps"]])
    stats = metrics_mod.control_stats(series["u_thrust_n"], rate)
    z0 = load_z0_params()
    frames = 0
    names: Dict[str, int] = {}
    for thrust, wx, wy, wz in zip(series["u_thrust_n"], rate[:, 0],
                                  rate[:, 1], rate[:, 2]):
        if not all(np.isfinite([thrust, wx, wy, wz])):
            continue
        hit = saturated_bounds((thrust, wx, wy, wz), z0)
        if hit:
            frames += 1
            for name in hit:
                names[name] = names.get(name, 0) + 1
    stats["saturation_frames_grid"] = frames
    stats["saturation_by_name_grid"] = dict(sorted(names.items()))
    return stats


def _hover_steady(data: Dict[str, Dict[str, np.ndarray]], source: str) -> dict:
    """公共窗最后 10 s：水平均值/最大、垂直绝对值均值。"""
    t = data["target"]["t_s"]
    end = float(t[-1])
    mask = t >= end - 10.0
    err = np.column_stack([
        data[source]["x_m"] - data["target"]["x_m"],
        data[source]["y_m"] - data["target"]["y_m"],
        data[source]["z_m"] - data["target"]["z_m"],
    ])[mask]
    h = np.linalg.norm(err[:, :2], axis=1)
    return {
        "start_s": float(t[mask][0]),
        "end_s": end,
        "n": int(h.size),
        "h_mean_m": float(h.mean()),
        "h_max_m": float(h.max()),
        "z_mean_abs_m": float(np.abs(err[:, 2]).mean()),
    }


# ─────────────────────────── 表格 ───────────────────────────


def _table(header: Sequence[str], rows: Iterable[Sequence[str]]) -> str:
    lines = ["| " + " | ".join(header) + " |",
             "|" + "|".join("---" for _ in header) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(row) + " |")
    return "\n".join(lines)


def table_window(meta: Dict[str, dict]) -> str:
    rows = []
    for scene in SCENES:
        m = meta[scene]
        n = m["grid"]["n"]
        end = m["grid"]["common_end_s"]
        ends = []
        for source in ("target", *POSITION_SOURCES):
            entry = m["sources"][source]
            reason = {"progress": "progress", "ref_end": "ref_end"}[
                entry["completion_reason"]]
            ends.append(f"{entry['completion_s']:.4f}（{reason}）")
        decider = m["grid"]["common_end_reason"].split("：")[-1]
        rows.append([SCENE_CN[scene], str(n), f"0 – {end:.4f}", *ends, decider])
    return _table(
        ["场景", "网格点/源", "公共窗 (s)", "target 完成 (s)",
         "offboard 完成 (s)", "mpc_sim 完成 (s)", "mpc_real 完成 (s)",
         "右端由谁决定"],
        rows,
    )


def table_position(pos: Dict[str, Dict[str, dict]]) -> str:
    rows = []
    for scene in SCENES:
        for source in POSITION_SOURCES:
            s = pos[scene][source]
            rows.append([
                SCENE_CN[scene], source,
                f"{s['rmse_m'][0]:.4f}", f"{s['rmse_m'][1]:.4f}",
                f"{s['rmse_m'][2]:.4f}",
                f"{s['h_mean_m']:.4f}", f"{s['h_max_m']:.4f}",
                f"{s['z_mean_abs_m']:.4f}", f"{s['z_max_abs_m']:.4f}",
            ])
    return _table(
        ["场景", "源", "RMSE ex (m)", "RMSE ey (m)", "RMSE ez (m)",
         "水平均值 (m)", "水平最大 (m)", "垂直均值 (m)", "垂直最大 (m)"],
        rows,
    )


def table_hover_steady(steady: Dict[str, dict]) -> str:
    rows = []
    for source in POSITION_SOURCES:
        s = steady[source]
        rows.append([source, f"{s['h_mean_m']:.4f}", f"{s['h_max_m']:.4f}",
                     f"{s['z_mean_abs_m']:.4f}", f"{s['n']}"])
    return _table(
        ["源", "水平均值 (m)", "水平最大 (m)", "垂直绝对值均值 (m)", "网格点"],
        rows,
    )


def table_attitude(att: Dict[str, Dict[str, dict]]) -> str:
    rows = []
    for scene in SCENES:
        for source in POSITION_SOURCES:
            s = att[scene][source]
            rows.append([
                SCENE_CN[scene], source,
                f"{s['angle_mean_deg']:.2f}", f"{s['angle_max_deg']:.2f}",
                f"{s['tilt']['mean']:.2f}", f"{s['tilt']['max']:.2f}",
                f"{s['yaw_diff']['mean']:.2f}", f"{s['yaw_diff']['max']:.2f}",
            ])
    return _table(
        ["场景", "源", "总夹角均值 (°)", "总夹角最大 (°)",
         "倾角均值 (°)", "倾角最大 (°)", "偏航差均值 (°)", "偏航差最大 (°)"],
        rows,
    )


def table_control(ctl: Dict[str, Dict[str, dict]],
                  sat_sim: Dict[str, int]) -> str:
    rows = []
    for scene in SCENES:
        for source in MPC_SOURCES:
            s = ctl[scene][source]
            sat = sat_sim[scene] if source == "mpc_sim" else s["saturation_frames_grid"]
            rows.append([
                SCENE_CN[scene], source,
                f"{s['thrust_mean_n']:.3f}", f"{s['thrust_min_n']:.3f}",
                f"{s['thrust_max_n']:.3f}",
                f"{s['rate_norm_mean_radps']:.4f}",
                f"{s['rate_norm_max_radps']:.4f}",
                *(f"{v:.4f}" for v in s["rate_absmax_radps"]),
                str(sat),
            ])
    return _table(
        ["场景", "源", "推力均值 (N)", "推力最小 (N)", "推力最大 (N)",
         "角速率模长均值 (rad/s)", "角速率模长最大 (rad/s)",
         r"\|wx\|max (rad/s)", r"\|wy\|max (rad/s)", r"\|wz\|max (rad/s)",
         "饱和拍数"],
        rows,
    )


def table_solver(solver: Dict[str, dict]) -> str:
    rows = []
    for scene in SCENES:
        s = solver[scene]
        block = s["solve_ms_steady"]
        rows.append([
            SCENE_CN[scene], str(block["n"]),
            f"{block['mean']:.3f}", f"{block['p95']:.3f}", f"{block['max']:.3f}",
            str(s["degraded_frames"]), str(s["saturation"]["frames_with_sat"]),
        ])
    return _table(
        ["场景", "稳态求解拍数", "均值 (ms)", "p95 (ms)", "最坏 (ms)",
         "退化拍数", "饱和拍数"],
        rows,
    )


# ─────────────────────────── 入口 ───────────────────────────


def build_blocks() -> Dict[str, str]:
    meta = {scene: read_metadata(scene) for scene in SCENES}
    data = {scene: read_scene(scene) for scene in SCENES}
    pos = {scene: {src: _position(data[scene], src, meta[scene])
                   for src in POSITION_SOURCES} for scene in SCENES}
    att = {scene: {src: _attitude(data[scene], src, meta[scene])
                   for src in POSITION_SOURCES} for scene in SCENES}
    ctl = {scene: {src: _control(data[scene], src)
                   for src in MPC_SOURCES} for scene in SCENES}
    solver = {scene: read_solver(scene) for scene in SCENES}
    steady = {src: _hover_steady(data["hover"], src) for src in POSITION_SOURCES}
    sat_sim = {scene: int(solver[scene]["saturation"]["frames_with_sat"])
               for scene in SCENES}

    return {
        "window": table_window(meta),
        "position": table_position(pos),
        "hover-steady": table_hover_steady(steady),
        "attitude": table_attitude(att),
        "control": table_control(ctl, sat_sim),
        "solver": table_solver(solver),
    }


def _display(path: Path) -> str:
    try:
        return str(Path(path).relative_to(REPO))
    except ValueError:
        return str(path)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--out", type=Path, default=GENERATED)
    parser.add_argument("--chapter", type=Path, default=CHAPTER)
    args = parser.parse_args(argv)

    blocks = build_blocks()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    text = args.out.read_text(encoding="utf-8") if args.out.exists() else GENERATED_HEADER
    for key, body in blocks.items():
        text = replace_section(text, f"{PREFIX}:{key}", body)
    args.out.write_text(text.rstrip() + "\n", encoding="utf-8")
    print(f"[ok] {_display(args.out)}")

    if args.chapter.exists():
        chapter = args.chapter.read_text(encoding="utf-8")
        for key, body in blocks.items():
            chapter = replace_section(chapter, f"{PREFIX}:{key}", body)
        args.chapter.write_text(chapter.rstrip() + "\n", encoding="utf-8")
        print(f"[ok] {_display(args.chapter)}")
    else:
        print(f"[skip] {_display(args.chapter)} 不存在，只写了 generated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
