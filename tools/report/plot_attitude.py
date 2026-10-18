#!/usr/bin/env python3
"""票 04：三条轨迹的 roll / pitch / yaw 姿态角图（四源叠加）。

每张图（悬停 / 正方形 / 8 字）三行，分别是 roll、pitch、yaw；每行画四个源：
目标、offboard 实飞、模型预测控制仿真、模型预测控制实飞。单位 deg，横轴
`t [s]` 相对 `/traj/start`，窗口取票 01 的公共窗。

口径（票面钉死，改之前先看票）：

- **目标姿态**按控制器约定是「水平 + 纯偏航」，本报告里三条目标线均为 0
  （横滚、俯仰、偏航都是 0）。它不是由参考加速度反解的“理想倾角”。
- **实飞姿态**取 `/mavros/vision_pose/pose`（约 190 Hz，控制器状态源）；
  ⛔ 不用 30 Hz 的 EKF 姿态。它与 VP 有约 +0.03 rad 的滚转系统偏差，
  图注写明，避免把实际滚转均值读成跟踪误差。
- **仿真姿态**由 SITL summary 的 `x0[6:10]` 四元数解算（票 01 已完成）；
  旧日志里名叫 roll/pitch/yaw 的字段是角速率，不能当姿态角。

脚本只读 `docs/report/data/*.csv`，图注的来源信息读同名 `.json`；不读原始包、
不在这里重算对齐口径。

用法：

    python3 tools/report/plot_attitude.py --scene all

产物：

    docs/report/figures/fig-att-{scene}.{png,svg}
    docs/report/generated/figure-captions.md        # 追加 phase5-att 块
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, Sequence, Tuple

import numpy as np

import matplotlib

matplotlib.use("Agg")

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

# 顺序有讲究：`plot_traj` 在导入时修补 `mpl_toolkits`（本机两套 matplotlib），
# 必须赶在 `matplotlib.pyplot` 第一次导入之前；否则 3D 投影注册失败，会连累
# 轨迹图测试的 fixture（本脚本自己只画 2D）。
import plot_traj  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import style  # noqa: E402
from captions import CAPTION_HEADER, update_caption_file  # noqa: E402

SCENES = plot_traj.SCENES

#: 三个姿态角，行序即图内自上而下的顺序。
ANGLES = ("roll", "pitch", "yaw")

ANGLE_LABELS = {
    "roll": "Roll",
    "pitch": "Pitch",
    "yaw": "Yaw",
}

DATA_DIR = REPO / "docs/report/data"
FIGURE_DIR = REPO / "docs/report/figures"
CAPTION_FILE = REPO / "docs/report/generated/figure-captions.md"

#: 图注章节前缀（与其他脚本的块分开，互不覆盖）。
CAPTION_PREFIX = "phase5-att"


# ─────────────────────────── 画图 ───────────────────────────


def scene_window(data: Dict[str, Dict[str, np.ndarray]]) -> Tuple[float, float]:
    """公共窗 `[0, 末帧时间]`；四源已由票 01 对齐到同一网格。"""
    return 0.0, float(data["target"]["t"][-1])


def plot_scene(scene: str, data: Dict[str, Dict[str, np.ndarray]],
               xlim: Tuple[float, float]):
    """一张三行姿态图；返回 `(figure, axes)` 供测试检查。"""
    fig, axes = plt.subplots(
        len(ANGLES), 1, sharex=True, figsize=style.FIGSIZE_MULTIROW
    )
    t = data["target"]["t"]

    for row, angle in enumerate(ANGLES):
        ax = axes[row]
        for source in style.SOURCES:
            ax.plot(
                t,
                np.degrees(data[source][angle]),
                label=style.SOURCE_LABELS[source],
                **style.line_kwargs(source),
            )
        ax.set_xlim(*xlim)
        # 三行各自按该轴数据定范围：偏航在仿真源里约 90 deg，若与 roll/pitch
        # 共用纵轴，会把两行压成直线，图就没有信息量了。
        ax.set_ylabel(f"{ANGLE_LABELS[angle]} [deg]")
        ax.set_title(ANGLE_LABELS[angle], loc="left", fontsize=style.TITLE_SIZE)

    axes[-1].set_xlabel("t [s] (relative to /traj/start)")
    # 图例只在第一行出现一次，避免三份相同图例占地方。
    axes[0].legend(loc="upper right", ncol=4, frameon=False)
    fig.suptitle(f"{plot_traj.SCENE_TITLE_EN[scene]} - attitude", y=0.995)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.985))
    return fig, axes


# ─────────────────────────── 图注 ───────────────────────────


def _source_description(source: str, meta: dict) -> str:
    if source == "target":
        return ("`/traj/reference` 的偏航（控制器约定为水平 + 纯偏航；"
                "本报告三条目标线均为 0）")
    if source == "offboard":
        return (f"`{meta['inputs']['offboard_bag']['path']}` 的 "
                "`/mavros/vision_pose/pose`")
    if source == "mpc_sim":
        return (f"`{meta['inputs']['sim_summary']['path']}` 的 `x0` 四元数")
    return (f"`{meta['inputs']['mpc_bag']['path']}` 的 "
            "`/mavros/vision_pose/pose`")


def _caption(scene: str, meta: dict, xlim: Tuple[float, float],
             command: str) -> str:
    end = xlim[1]
    # 用 `window_trim_reason` 而不是 `common_end_reason`：前者写明悬停为何停在
    # 39.65 s（录制截断），后者只说"取最小"，容易被读成控制结束。
    reason = meta["grid"]["window_trim_reason"]
    lines = [
        f"### A{SCENES.index(scene) + 1} {plot_traj.SCENE_TITLE_CN[scene]}"
        f"（{scene}）姿态角：四源对比",
        "",
        f"- 文件：`figures/fig-att-{scene}.png`、"
        f"`figures/fig-att-{scene}.svg`（300 dpi）",
        f"- 数据：`data/{scene}.csv`（同名 `.json` 记输入 sha256 与窗口口径）",
        f"- 窗口：0.0000 - {end:.4f} s（{reason}）",
        "- 坐标：时间零点为 `/traj/start` 的包记录时间戳；三行分别是 roll、"
        "pitch、yaw，单位为 deg；三行纵轴各自按该轴数据范围，不共用。",
        "- 来源：目标来自 " + _source_description("target", meta) + "；"
        "offboard 来自 " + _source_description("offboard", meta) + "；"
        "模型预测控制仿真来自 " + _source_description("mpc_sim", meta) + "；"
        "模型预测控制实飞来自 " + _source_description("mpc_real", meta) + "。",
        "- 目标姿态口径：按控制器约定取「水平 + 纯偏航」，"
        "即 roll = pitch = yaw = 0；这不是用参考加速度反解出的理想倾角，"
        "理由见第 8 章。",
        "- 姿态口径：实飞姿态取 `/mavros/vision_pose/pose`（约 190 Hz），"
        "它与 30 Hz EKF 姿态有约 **+0.03 rad** 的滚转系统偏差，"
        "不要把实际滚转均值读成跟踪误差；仿真姿态由 `x0[6:10]` 四元数解算，"
        "旧日志里名叫 roll/pitch/yaw 的字段是角速率，不能当姿态角。",
        f"- 生成：`{command}`",
    ]
    if scene == "hover":
        lines.append(
            f"- 注意：offboard 包截止 {end:.2f} s（录制截断，不是控制失败）。"
        )
    return "\n".join(lines)


# ─────────────────────────── 入口 ───────────────────────────


def _display(path: Path) -> str:
    try:
        return str(Path(path).relative_to(REPO))
    except ValueError:
        return str(path)


def _command(scene_arg: str) -> str:
    return f"python3 tools/report/plot_attitude.py --scene {scene_arg}"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--scene", choices=("all", *SCENES), default="all")
    parser.add_argument("--data-dir", type=Path, default=DATA_DIR)
    parser.add_argument("--outdir", type=Path, default=FIGURE_DIR)
    parser.add_argument("--captions", type=Path, default=CAPTION_FILE)
    parser.add_argument("--no-captions", action="store_true",
                        help="只画图，不写 figure-captions.md")
    args = parser.parse_args(argv)

    scenes = SCENES if args.scene == "all" else (args.scene,)
    command = _command(args.scene)

    blocks: Dict[str, str] = {}
    for scene in scenes:
        data = plot_traj.load_scene(args.data_dir, scene)
        meta = plot_traj.load_metadata(args.data_dir, scene)
        xlim = scene_window(data)
        fig, _ = plot_scene(scene, data, xlim)
        stem = f"fig-att-{scene}"
        png, svg = style.save(fig, args.outdir, stem)
        plt.close(fig)
        print(f"[ok] {_display(png)}")
        print(f"[ok] {_display(svg)}")
        blocks[scene] = _caption(scene, meta, xlim, command)

    if not args.no_captions:
        update_caption_file(args.captions, CAPTION_HEADER, CAPTION_PREFIX, blocks)
        print(f"[ok] {_display(Path(args.captions))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
