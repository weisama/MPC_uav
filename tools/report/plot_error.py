#!/usr/bin/env python3
"""票 05 图 1：位置误差–时间（每场景三行，四源叠加）。

误差定义（票面钉死）：

    e(t) = 实际(t) − 目标(t)

三个分量分别是 `ex / ey / ez`，画在**轨迹原点相对系**里；单位 m，横轴
`t [s]` 相对 `/traj/start`，窗口取票 01 的公共窗。四源里的 `target`
与被减数相同，因此它的三条线恒为 0 —— 它留在图里是为了让读者一眼看出
“零误差”在哪，不是第三条实际轨迹。

脚本只读 `docs/report/data/*.csv`，图注的来源信息读同名 `.json`；不读原始包、
不在这里重算对齐口径。

用法：

    python3 tools/report/plot_error.py --scene all

产物：

    docs/report/figures/fig-error-{scene}.{png,svg}
    docs/report/generated/figure-captions.md        # 追加 phase5-error 块
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
# 必须赶在 `matplotlib.pyplot` 第一次导入之前；否则 3D 投影注册会连累轨迹图测试。
import plot_traj  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import style  # noqa: E402
from captions import CAPTION_HEADER, update_caption_file  # noqa: E402

SCENES = plot_traj.SCENES

#: 三个误差分量，行序即图内自上而下的顺序；CSV 列名。
AXES = ("x", "y", "z")
ERROR_LABELS = {
    "x": "ex",
    "y": "ey",
    "z": "ez",
}

DATA_DIR = REPO / "docs/report/data"
FIGURE_DIR = REPO / "docs/report/figures"
CAPTION_FILE = REPO / "docs/report/generated/figure-captions.md"

#: 图注章节前缀（与其他脚本的块分开，互不覆盖）。
CAPTION_PREFIX = "phase5-error"


# ─────────────────────────── 误差与窗口 ───────────────────────────


def scene_window(data: Dict[str, Dict[str, np.ndarray]]) -> Tuple[float, float]:
    """公共窗 `[0, 末帧时间]`；四源已由票 01 对齐到同一网格。"""
    return 0.0, float(data["target"]["t"][-1])


def error_series(data: Dict[str, Dict[str, np.ndarray]], source: str,
                 axis: str) -> np.ndarray:
    """`实际 − 目标` 的某一个分量（m）。"""
    return data[source][axis] - data["target"][axis]


# ─────────────────────────── 画图 ───────────────────────────


def plot_scene(scene: str, data: Dict[str, Dict[str, np.ndarray]],
               xlim: Tuple[float, float]):
    """一张三行误差图；返回 `(figure, axes)` 供测试检查。"""
    fig, axes = plt.subplots(
        len(AXES), 1, sharex=True, figsize=style.FIGSIZE_MULTIROW
    )
    t = data["target"]["t"]

    for row, axis in enumerate(AXES):
        ax = axes[row]
        for source in style.SOURCES:
            ax.plot(
                t,
                error_series(data, source, axis),
                label=style.SOURCE_LABELS[source],
                **style.line_kwargs(source),
            )
        ax.axhline(0.0, color=style.ORIGIN_LINE_COLOR,
                   linewidth=style.ORIGIN_LINE_WIDTH, zorder=0)
        ax.set_xlim(*xlim)
        # 三行各自按该轴误差范围，不共用：垂直误差是厘米级，水平误差在
        # 正方形/8 字场景是几十厘米，共用会把 ez 压成一条直线。
        ax.set_ylabel(f"{ERROR_LABELS[axis]} [m]")
        ax.set_title(ERROR_LABELS[axis], loc="left", fontsize=style.TITLE_SIZE)

    axes[-1].set_xlabel("t [s] (relative to /traj/start)")
    axes[0].legend(loc="upper right", ncol=4, frameon=False)
    fig.suptitle(f"{plot_traj.SCENE_TITLE_EN[scene]} - position error",
                 y=0.995)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.985))
    return fig, axes


# ─────────────────────────── 图注 ───────────────────────────


def _source_description(source: str, meta: dict) -> str:
    if source == "target":
        return "目标自身（`/traj/reference`），因此误差恒 0"
    if source == "offboard":
        return (f"`{meta['inputs']['offboard_bag']['path']}` 的 "
                "`/mavros/vision_pose/pose`")
    if source == "mpc_sim":
        return f"`{meta['inputs']['sim_summary']['path']}` 的 `x0` 位置"
    return (f"`{meta['inputs']['mpc_bag']['path']}` 的 "
            "`/mavros/vision_pose/pose`")


def _caption(scene: str, meta: dict, xlim: Tuple[float, float],
             command: str) -> str:
    end = xlim[1]
    reason = meta["grid"]["window_trim_reason"]
    lines = [
        f"### E{SCENES.index(scene) + 1} {plot_traj.SCENE_TITLE_CN[scene]}"
        f"（{scene}）位置误差：四源对比",
        "",
        f"- 文件：`figures/fig-error-{scene}.png`、"
        f"`figures/fig-error-{scene}.svg`（300 dpi）",
        f"- 数据：`data/{scene}.csv`（同名 `.json` 记输入 sha256 与窗口口径）",
        f"- 窗口：0.0000 - {end:.4f} s（{reason}）",
        "- 误差定义：**实际 − 目标**（不是目标 − 实际）；三个分量分别为 "
        "`ex`、`ey`、`ez`，单位 m；坐标均为轨迹原点相对系。",
        "- 来源：offboard 来自 " + _source_description("offboard", meta) + "；"
        "MPC 仿真来自 " + _source_description("mpc_sim", meta) + "；"
        "MPC 实飞来自 " + _source_description("mpc_real", meta) + "；"
        "目标行是 " + _source_description("target", meta) + "。",
        "- 目标姿态与位置均来自 `/traj/reference`；参考在 TRACK 窗内才成立，"
        "跑完后参考继续发零，不能拿末帧参考当完成判据。",
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
    return f"python3 tools/report/plot_error.py --scene {scene_arg}"


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
        stem = f"fig-error-{scene}"
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
