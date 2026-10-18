#!/usr/bin/env python3
"""票 03 追加（2026-10-18 操作者裁定）：9 张 pitch/roll 图。

3 个场景 × 3 个实际源 = 9 种情况，每种情况一张图；每张图包含：

- **Pitch** 与 **Roll** 两条曲线（单位 deg）；
- 一条 **0 deg 零线**（虚线）；
- 横轴 `t [s]`，相对 `/traj/start`。

目标姿态按控制器约定恒为 0，单独出图没有信息量（操作者 2026-10-18：
「无需目标值」），因此**不画 target**。

**坐标轴统一**（操作者明确要求，避免逐图看刻度）：

- 纵轴：9 张图共用同一个范围。范围由**全部** 9 组 pitch/roll 数据现算，
  以 0 为中心、向上取整后再留 1 deg 余量；不是逐图 autoscale；
- 横轴：同一场景的 3 张图共用 `[0, 公共窗终点]`；不同场景窗口本来不同。

只读 `docs/report/data/*.csv`；图注的来源信息读同名 `.json`（票 01 产出），
**不读原始包**，不在这里重算对齐口径。

用法：

    python3 tools/report/plot_pitch_roll.py --scene all

产物：

    docs/report/figures/fig-pitchroll-{scene}-{source}.{png,svg}
    docs/report/generated/figure-captions.md        # 追加 phase5-pitchroll 块
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
# 必须赶在 `matplotlib.pyplot` 第一次导入之前；反过来会让 3D 投影注册失败，
# 连累轨迹图测试的 fixture（即使本脚本自己只画 2D）。
import plot_traj  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import style  # noqa: E402
from captions import CAPTION_HEADER, update_caption_file  # noqa: E402

SCENES = plot_traj.SCENES

#: 只画实际姿态。目标姿态恒 0，不单独出图（操作者 2026-10-18：「无需目标值」）。
ANGLE_SOURCES = ("offboard", "mpc_sim", "mpc_real")

DATA_DIR = REPO / "docs/report/data"
FIGURE_DIR = REPO / "docs/report/figures"
CAPTION_FILE = REPO / "docs/report/generated/figure-captions.md"

#: 图注里给人看的中文源名；图内图例仍用英文。
SOURCE_LABELS_CN = {
    "offboard": "offboard 实飞",
    "mpc_sim": "模型预测控制仿真",
    "mpc_real": "模型预测控制实飞",
}

#: 图注章节前缀（与轨迹图的 `phase5-traj` 分开，互不覆盖）。
CAPTION_PREFIX = "phase5-pitchroll"


# ─────────────────────────── 数据与坐标轴 ───────────────────────────


def load_scenes(data_dir: Path) -> Dict[str, Dict[str, Dict[str, np.ndarray]]]:
    """读全部三个场景的 CSV（即使只重画一个场景，也要用全量数据定纵轴）。"""
    return {scene: plot_traj.load_scene(data_dir, scene) for scene in SCENES}


def compute_angle_limits(
    data_by_scene: Dict[str, Dict[str, Dict[str, np.ndarray]]],
) -> Tuple[float, float]:
    """9 组 pitch/roll 共用的纵轴范围（deg，以 0 为中心）。

    从数据现算，不在散文里写死：换一份数据重跑，范围自动跟着变。
    """
    values = []
    for data in data_by_scene.values():
        for source in ANGLE_SOURCES:
            values.append(np.degrees(data[source]["pitch"]))
            values.append(np.degrees(data[source]["roll"]))
    if not values:
        raise ValueError("没有可用的姿态数据")
    all_values = np.concatenate(values)
    span = float(np.max(np.abs(all_values)))
    # 向上取整到整数度，再留 1 deg 余量；对称范围让零线永远在正中。
    limit = float(np.ceil(span)) + 1.0
    return -limit, limit


def scene_window(data: Dict[str, Dict[str, np.ndarray]]) -> Tuple[float, float]:
    """公共窗 `[0, 末帧时间]`；四源已由票 01 对齐到同一网格。"""
    return 0.0, float(data["target"]["t"][-1])


# ─────────────────────────── 画图 ───────────────────────────


def plot_case(
    scene: str,
    source: str,
    data: Dict[str, Dict[str, np.ndarray]],
    ylim: Tuple[float, float],
    xlim: Tuple[float, float],
):
    """一张 pitch/roll 图；返回 `(figure, axes)` 供测试检查轴范围。"""
    values = data[source]
    t = values["t"]

    fig, ax = plt.subplots(figsize=style.FIGSIZE_SINGLE)
    for angle in ("pitch", "roll"):
        ax.plot(
            t,
            np.degrees(values[angle]),
            color=style.ANGLE_COLORS[angle],
            linewidth=style.ANGLE_LINEWIDTH,
            label=style.ANGLE_LABELS[angle],
            zorder=3,
        )
    ax.axhline(
        0.0,
        color=style.ZERO_LINE_COLOR,
        linestyle=style.ZERO_LINE_STYLE,
        linewidth=style.ZERO_LINE_WIDTH,
        label="Zero",
        zorder=1,
    )

    # 显式写死范围：同一场景的三张图共用横轴，9 张图共用纵轴。
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xlabel("t [s] (relative to /traj/start)")
    ax.set_ylabel("angle [deg]")
    ax.set_title(
        f"{plot_traj.SCENE_TITLE_EN[scene]} - "
        f"{style.SOURCE_LABELS[source]}: pitch and roll"
    )
    ax.legend(loc="upper right", ncol=3, frameon=False)
    fig.tight_layout()
    return fig, ax


# ─────────────────────────── 图注 ───────────────────────────


def _source_description(source: str, meta: dict) -> str:
    if source == "offboard":
        return (f"`{meta['inputs']['offboard_bag']['path']}` 的 "
                "`/mavros/vision_pose/pose`")
    if source == "mpc_sim":
        return (f"`{meta['inputs']['sim_summary']['path']}` 的 `x0` 四元数")
    return (f"`{meta['inputs']['mpc_bag']['path']}` 的 "
            "`/mavros/vision_pose/pose`")


def _attitude_note(source: str) -> str:
    if source == "mpc_sim":
        return ("仿真姿态由 `x0[6:10]` 四元数解算；旧日志里名叫 roll/pitch/yaw "
                "的字段是角速率，不能当姿态角。")
    return ("实飞姿态取 `/mavros/vision_pose/pose`；它与 30 Hz EKF 姿态有约 "
            "+0.03 rad 的滚转系统偏差，不要把滚转零偏读成跟踪误差。")


def _caption(scene: str, source: str, meta: dict,
             ylim: Tuple[float, float], xlim: Tuple[float, float],
             command: str) -> str:
    end = xlim[1]
    # 用 `window_trim_reason` 而不是 `common_end_reason`：前者写明悬停为何停在
    # 39.65 s（录制截断），后者只说"取最小"，容易被读成控制结束。
    reason = meta["grid"]["window_trim_reason"]
    lines = [
        f"### PR-{scene}-{source} {plot_traj.SCENE_TITLE_CN[scene]} · "
        f"{SOURCE_LABELS_CN[source]}：pitch 与 roll",
        "",
        f"- 文件：`figures/fig-pitchroll-{scene}-{source}.png`、"
        f"`figures/fig-pitchroll-{scene}-{source}.svg`（300 dpi）",
        f"- 数据：`data/{scene}.csv`",
        f"- 窗口：0.0000 - {end:.4f} s（{reason}）",
        f"- 统一坐标轴：纵轴 {ylim[0]:.0f} 到 {ylim[1]:.0f} deg"
        f"（{len(SCENES) * len(ANGLE_SOURCES)} 张图共用，以 0 为中心）；"
        "横轴 0 - "
        f"{end:.4f} s（同场景 {len(ANGLE_SOURCES)} 张共用）；零线为 0 deg。",
        f"- 来源：{_source_description(source, meta)}。",
        f"- 姿态口径：{_attitude_note(source)}",
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
    return f"python3 tools/report/plot_pitch_roll.py --scene {scene_arg}"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", choices=("all", *SCENES), default="all")
    parser.add_argument("--data-dir", type=Path, default=DATA_DIR)
    parser.add_argument("--outdir", type=Path, default=FIGURE_DIR)
    parser.add_argument("--captions", type=Path, default=CAPTION_FILE)
    parser.add_argument("--no-captions", action="store_true",
                        help="只画图，不写 figure-captions.md")
    args = parser.parse_args(argv)

    scenes = SCENES if args.scene == "all" else (args.scene,)
    command = _command(args.scene)

    # 纵轴必须用全部 9 组数据定，否则部分重跑会和其他图不一致。
    data_by_scene = load_scenes(args.data_dir)
    ylim = compute_angle_limits(data_by_scene)

    blocks: Dict[str, str] = {}
    for scene in scenes:
        data = data_by_scene[scene]
        meta = plot_traj.load_metadata(args.data_dir, scene)
        xlim = scene_window(data)
        for source in ANGLE_SOURCES:
            fig, _ = plot_case(scene, source, data, ylim, xlim)
            stem = f"fig-pitchroll-{scene}-{source}"
            png, svg = style.save(fig, args.outdir, stem)
            plt.close(fig)
            print(f"[ok] {_display(png)}")
            print(f"[ok] {_display(svg)}")
            blocks[f"{scene}:{source}"] = _caption(
                scene, source, meta, ylim, xlim, command)

    if not args.no_captions:
        update_caption_file(args.captions, CAPTION_HEADER, CAPTION_PREFIX, blocks)
        print(f"[ok] {_display(Path(args.captions))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
