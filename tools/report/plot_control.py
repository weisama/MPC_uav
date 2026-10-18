#!/usr/bin/env python3
"""票 05 图 2：控制量（只画模型预测控制仿真与实飞两源）。

每张图（悬停 / 正方形 / 8 字）四行：

1. 总推力 `F [N]`；
2. 机体角速率 `wx`、`wy`、`wz [rad/s]`（FLU，机体系）。

⛔ **只画两源**：模型预测控制仿真（`mpc_sim`）与模型预测控制实飞
（`mpc_real`）。offboard 的 `/mavros/setpoint_raw/local` 是**位置设定点**，
与“推力 + 角速率”不是同一层；画在一起会造成“看起来可比”的假象。

推力换算（票 01 已在 CSV 里完成，本脚本不重算）：

    u_thrust_n = thrust × t_max

`t_max` 逐趟取该趟 `mpc_node` 启动行（实飞）或 SITL 控制器参数（仿真），
具体数值与来源写在同名 `.json` 的 `sources.<源>.u_conversion` 里；图注按
该 JSON 现读，不抄常数。

脚本只读 `docs/report/data/*.csv`，图注的来源信息读同名 `.json`。

用法：

    python3 tools/report/plot_control.py --scene all

产物：

    docs/report/figures/fig-control-{scene}.{png,svg}
    docs/report/generated/figure-captions.md        # 追加 phase5-control 块
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

#: **只有这两源**。offboard 不在同一层（位置设定点 vs 推力 + 角速率）。
CONTROL_SOURCES = ("mpc_sim", "mpc_real")

#: 四行：CSV 列名、图内英文标签、单位。
ROWS = (
    ("u_thrust_n", "Thrust [N]", "N"),
    ("wx_radps", "wx [rad/s]", "rad/s"),
    ("wy_radps", "wy [rad/s]", "rad/s"),
    ("wz_radps", "wz [rad/s]", "rad/s"),
)

#: 控制量读取的列。目标与 offboard 没有这些列的值，所以必须只读两源。
CONTROL_COLUMNS = {"t": "t_s", **{column: column for column, _label, _unit in ROWS}}

DATA_DIR = REPO / "docs/report/data"
FIGURE_DIR = REPO / "docs/report/figures"
CAPTION_FILE = REPO / "docs/report/generated/figure-captions.md"

#: 图注章节前缀（与其他脚本的块分开，互不覆盖）。
CAPTION_PREFIX = "phase5-control"


# ─────────────────────────── 窗口与画图 ───────────────────────────


def scene_window(data: Dict[str, Dict[str, np.ndarray]]) -> Tuple[float, float]:
    """公共窗 `[0, 末帧时间]`；两源已由票 01 对齐到同一网格。"""
    return 0.0, float(data[CONTROL_SOURCES[0]]["t"][-1])


def plot_scene(scene: str, data: Dict[str, Dict[str, np.ndarray]],
               xlim: Tuple[float, float]):
    """一张四行控制量图；返回 `(figure, axes)` 供测试检查。"""
    fig, axes = plt.subplots(
        len(ROWS), 1, sharex=True, figsize=style.FIGSIZE_MULTIROW
    )
    t = data["mpc_sim"]["t"]

    for row, (column, label, _unit) in enumerate(ROWS):
        ax = axes[row]
        for source in CONTROL_SOURCES:
            ax.plot(
                t,
                data[source][column],
                label=style.SOURCE_LABELS[source],
                **style.line_kwargs(source),
            )
        ax.set_xlim(*xlim)
        ax.set_ylabel(label)
        ax.set_title(label.split(" [")[0], loc="left",
                     fontsize=style.TITLE_SIZE)

    axes[-1].set_xlabel("t [s] (relative to /traj/start)")
    axes[0].legend(loc="upper right", ncol=2, frameon=False)
    fig.suptitle(f"{plot_traj.SCENE_TITLE_EN[scene]} - control input",
                 y=0.995)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.985))
    return fig, axes


# ─────────────────────────── 图注 ───────────────────────────


def _source_description(source: str, meta: dict) -> str:
    if source == "mpc_sim":
        return (f"`{meta['inputs']['sim_summary']['path']}` 的求解指令"
                "（SITL 控制器参数）")
    return (f"`{meta['inputs']['mpc_bag']['path']}` 的 "
            "`/mavros/setpoint_raw/attitude`（`body_rate` + `thrust`）")


def _conversion_text(meta: dict) -> str:
    sim = meta["sources"]["mpc_sim"]["u_conversion"]
    real = meta["sources"]["mpc_real"]["u_conversion"]
    sim_tmax = sim.get("t_max_n")
    real_tmax = real.get("t_max_n")
    if sim_tmax != real_tmax:
        raise ValueError(
            "仿真与实飞的 t_max 不同，图注不能只写一个常数："
            f"{sim_tmax!r} vs {real_tmax!r}"
        )
    return (
        f"`u_thrust_n = thrust × t_max`，`t_max = {sim_tmax:g} N`；"
        "仿真取 SITL 控制器参数，实飞取该趟 `mpc_node` 启动行"
        "（`tools/report/evidence/mpc-normalization.json`）"
    )


def _caption(scene: str, meta: dict, xlim: Tuple[float, float],
             command: str) -> str:
    end = xlim[1]
    reason = meta["grid"]["window_trim_reason"]
    lines = [
        f"### C{SCENES.index(scene) + 1} {plot_traj.SCENE_TITLE_CN[scene]}"
        f"（{scene}）控制量：模型预测控制仿真 vs 实飞",
        "",
        f"- 文件：`figures/fig-control-{scene}.png`、"
        f"`figures/fig-control-{scene}.svg`（300 dpi）",
        f"- 数据：`data/{scene}.csv`（同名 `.json` 记输入 sha256 与窗口口径）",
        f"- 窗口：0.0000 - {end:.4f} s（{reason}）",
        "- 四行：总推力 `F [N]`、机体角速率 `wx`、`wy`、`wz [rad/s]`"
        "（FLU，机体系指令）；横轴 `t [s]` 相对 `/traj/start`。",
        "- 只有两源：模型预测控制仿真（" + _source_description("mpc_sim", meta)
        + "）与模型预测控制实飞（" + _source_description("mpc_real", meta)
        + "）。",
        "- 为什么没有 offboard：offboard 的控制量是 "
        "`/mavros/setpoint_raw/local` 的**位置设定点**，与“推力 + 角速率”"
        "不是同一层；画在一起会造成“看起来可比”的假象。两组只在位置跟踪"
        "这一层可比（见误差图）。",
        f"- 推力换算：{_conversion_text(meta)}。"
        "CSV 里的控制量按 100 Hz 网格**零阶保持**取样。",
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
    return f"python3 tools/report/plot_control.py --scene {scene_arg}"


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
        data = plot_traj.load_scene(
            args.data_dir, scene,
            columns=CONTROL_COLUMNS, sources=CONTROL_SOURCES,
        )
        meta = plot_traj.load_metadata(args.data_dir, scene)
        xlim = scene_window(data)
        fig, _ = plot_scene(scene, data, xlim)
        stem = f"fig-control-{scene}"
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
