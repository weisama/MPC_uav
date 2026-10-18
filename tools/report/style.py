#!/usr/bin/env python3
"""报告图的统一样式（Phase 5 票 03）。

四类图（轨迹、姿态、误差、控制量）共用这里的**线型、颜色、字号、图幅**，
避免每个脚本各画一套、最后拼在一份报告里像四份材料。

约定（票面要求，改之前先看票）：

- 目标线一律**虚线**，实际轨迹一律**实线**；
- 颜色按**源**固定，不随图变化 —— 读者在第 10 章来回翻图时不必重新认色；
- 图内文字一律英文（不依赖 Jetson 上的中文字体）；中文只出现在
  `docs/report/generated/figure-captions.md` 的图注里；
- PNG 固定 300 dpi，同时写一份 SVG。

用法：

    import style
    style.line_kwargs("offboard")          # 传给 ax.plot
    style.start_marker(ax, x0, y0, color)  # 各源同一形状的起点标记
    style.save(fig, outdir, "fig-traj-hover")
"""

from __future__ import annotations

from pathlib import Path

import matplotlib
from matplotlib.lines import Line2D

# ─────────────────────────── 源 ───────────────────────────

#: 固定顺序。图例、图注、检查清单都按这个顺序，别在单个脚本里重排。
SOURCES = ("target", "offboard", "mpc_sim", "mpc_real")

#: 图例英文名。`offboard` / `mpc_*` 是数据里的源名，不是给人看的标签。
SOURCE_LABELS = {
    "target": "Target",
    "offboard": "Offboard flight",
    "mpc_sim": "MPC simulation",
    "mpc_real": "MPC flight",
}

#: 颜色按源固定（色盲友好组合；目标用中性黑，避免和三条实际轨迹抢注意力）。
SOURCE_COLORS = {
    "target": "#000000",
    "offboard": "#1f77b4",
    "mpc_sim": "#d62728",
    "mpc_real": "#2ca02c",
}

#: 目标虚线、实际实线（票面硬要求）。
SOURCE_LINESTYLES = {
    "target": (0, (5, 2)),
    "offboard": "-",
    "mpc_sim": "-",
    "mpc_real": "-",
}

SOURCE_LINEWIDTHS = {
    "target": 1.4,
    "offboard": 1.15,
    "mpc_sim": 1.15,
    "mpc_real": 1.15,
}

SOURCE_ZORDER = {
    "target": 4,
    "offboard": 3,
    "mpc_sim": 2,
    "mpc_real": 5,
}

# ─────────────────────────── 字号 / 图幅 ───────────────────────────

DPI = 300

#: 左右两联图（轨迹、误差等两栏图）。
FIGSIZE_2PANEL = (12.0, 4.7)

#: 多行共用时间轴（姿态 3 行、控制量 4 行）。
FIGSIZE_MULTIROW = (11.0, 8.6)

#: 单面板图（票 03 追加的 12 张 pitch/roll 图）。
FIGSIZE_SINGLE = (10.0, 4.2)

#: 3D 候选图。
FIGSIZE_3D = (8.0, 6.8)

FONT_SIZE = 10
TITLE_SIZE = 11
LEGEND_SIZE = 9
TICK_SIZE = 9

#: 起点标记：四源同一形状，用各自源色描边、空心。
START_MARKER = "o"
START_MARKERSIZE = 6.0
START_MARKER_EDGE_WIDTH = 1.2
START_LEGEND_COLOR = "#555555"

#: 轨迹原点参考线（x=0 / y=0 / z=0），浅灰，不进图例。
ORIGIN_LINE_COLOR = "#c8c8c8"
ORIGIN_LINE_WIDTH = 0.6

# ─────────────────────────── 姿态角（pitch / roll） ───────────────────────────

#: 12 张 pitch/roll 图共用。颜色刻意避开四源色，避免跨图误认。
ANGLE_COLORS = {
    "pitch": "#6A3D9A",
    "roll": "#E69F00",
}
ANGLE_LABELS = {
    "pitch": "Pitch",
    "roll": "Roll",
}
ANGLE_LINEWIDTH = 1.25

#: 零线：每张 pitch/roll 图都画，虚线，进图例。
ZERO_LINE_COLOR = "#555555"
ZERO_LINE_WIDTH = 0.9
ZERO_LINE_STYLE = (0, (4, 2))


def apply() -> None:
    """把样式写进 matplotlib 全局 rcParams（可重复调用）。"""
    matplotlib.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["DejaVu Sans"],
        "font.size": FONT_SIZE,
        "axes.titlesize": TITLE_SIZE,
        "axes.labelsize": FONT_SIZE,
        "legend.fontsize": LEGEND_SIZE,
        "xtick.labelsize": TICK_SIZE,
        "ytick.labelsize": TICK_SIZE,
        "axes.grid": True,
        "grid.color": "#dcdcdc",
        "grid.linewidth": 0.6,
        "axes.axisbelow": True,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
        # 负号用 ASCII 的 '-'，不让 SVG 依赖字体里的 U+2212 字形。
        "axes.unicode_minus": False,
        # SVG 里保留可搜索的文字（默认会转成字形路径）。
        "svg.fonttype": "none",
        # 不抽稀：图上每 10 ms 一个点，抽稀会把小抖动吃掉。
        "path.simplify": False,
        # 让同一份数据重跑得到同一份 SVG（id 用哈希而不是随机数）。
        "svg.hashsalt": "phase5-report",
    })


def line_kwargs(source: str) -> dict:
    """`ax.plot(..., **line_kwargs(source))` 的参数字典。"""
    if source not in SOURCES:
        raise KeyError(f"未知源 {source!r}，只允许 {SOURCES}")
    return {
        "color": SOURCE_COLORS[source],
        "linestyle": SOURCE_LINESTYLES[source],
        "linewidth": SOURCE_LINEWIDTHS[source],
        "zorder": SOURCE_ZORDER[source],
        "solid_capstyle": "round",
    }


def _start_marker_kwargs(source: str) -> dict:
    return {
        "linestyle": "none",
        "marker": START_MARKER,
        "markersize": START_MARKERSIZE,
        "markerfacecolor": "none",
        "markeredgecolor": SOURCE_COLORS[source],
        "markeredgewidth": START_MARKER_EDGE_WIDTH,
        "zorder": SOURCE_ZORDER[source] + 10,
    }


def start_marker(ax, x, y, source: str) -> None:
    """在各源的起点画同一个空心圆标记（颜色仍按源区分）。"""
    ax.plot([x], [y], clip_on=False, **_start_marker_kwargs(source))


def start_marker_3d(ax, x, y, z, source: str) -> None:
    """3D 轴的起点标记；形状、颜色与 2D 图一致。"""
    ax.plot([x], [y], [z], **_start_marker_kwargs(source))


def source_handle(source: str) -> Line2D:
    """图例用的代理线（不依赖图上是否真的画出了线，比如悬停目标只有一个点）。"""
    return Line2D(
        [], [],
        color=SOURCE_COLORS[source],
        linestyle=SOURCE_LINESTYLES[source],
        linewidth=SOURCE_LINEWIDTHS[source],
        label=SOURCE_LABELS[source],
    )


def start_handle() -> Line2D:
    """图例里的起点标记代理。"""
    return Line2D(
        [], [],
        linestyle="none",
        marker=START_MARKER,
        markersize=START_MARKERSIZE,
        markerfacecolor="none",
        markeredgecolor=START_LEGEND_COLOR,
        markeredgewidth=START_MARKER_EDGE_WIDTH,
        label="Start",
    )


def legend_handles(include_start: bool = True) -> list[Line2D]:
    handles = [source_handle(source) for source in SOURCES]
    if include_start:
        handles.append(start_handle())
    return handles


def save(fig, outdir: Path | str, stem: str) -> tuple[Path, Path]:
    """同时写 PNG（固定 300 dpi）与 SVG，返回两个路径。"""
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    png = outdir / f"{stem}.png"
    svg = outdir / f"{stem}.svg"
    common = {"bbox_inches": "tight", "pad_inches": 0.08, "facecolor": "white"}
    fig.savefig(png, dpi=DPI, **common)
    # `Date: None` 去掉 SVG 里的生成时间，保证重跑逐字节可比。
    fig.savefig(svg, format="svg", metadata={"Date": None}, **common)
    return png, svg


# 导入即生效：所有画图脚本共用同一套 rcParams，不必各自记得调用。
apply()
