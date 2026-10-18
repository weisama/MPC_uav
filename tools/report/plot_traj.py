#!/usr/bin/env python3
"""票 03（Phase 5）：三张轨迹对比图 + 三张 3D 候选图。

每张图两个子图：

- 左：XY 俯视（四源叠加，等比例）；
- 右：Z-t（四源叠加，t 相对 `/traj/start`）。

四源 = 目标（虚线）、offboard 实飞、模型预测控制仿真、模型预测控制实飞。
**只读** `docs/report/data/<scene>.csv`；图注的来源信息读同名 `.json`（票 01 产出，
含输入包路径与窗口口径），**不读原始包**，也不在这里重算对齐口径。

**坐标轴统一**（操作者 2026-10-18）：轨迹 1/2/3 的 **XY 与 3D 视图**共用同一组
x/y/z 范围；范围由全部数据现算，不写死。XY 两轴取同一对称范围，保证等比例下
三张图的尺度一致。**Z–t 不统一**（操作者同日明确）：z 与时间轴按各场景自己的
窗口与数据范围画。

用法：

    python3 tools/report/plot_traj.py --scene all
    python3 tools/report/plot_traj.py --scene hover --candidates

产物：

    docs/report/figures/fig-traj-{hover,square,lemniscate}.{png,svg}
    docs/report/figures/candidates/fig-traj-{scene}-3d.{png,svg}   # 仅 --candidates
    docs/report/generated/figure-captions.md                       # 正文图注
    docs/report/generated/figure-captions-candidates.md            # 候选图注（不进正文）

3D 候选**不进正文**：先交操作者看图后裁定（票面验收）。
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Dict, Sequence

import numpy as np

import matplotlib


def _install_mpl_toolkits_shim() -> None:
    """让 3D 投影用**与当前 matplotlib 同版本**的 `mpl_toolkits`。

    Jetson 上同时装了两套：`~/.local` 的 pip matplotlib 3.10.9，和
    `/usr/lib/python3/dist-packages` 的 Debian matplotlib 3.5.1。pip 那份的
    `mpl_toolkits` 是 PEP 420 namespace package（没有 `__init__.py`），Debian 那份是
    regular package；regular package 优先，于是 `import mpl_toolkits.mplot3d` 拿到的是
    3.5.1，对 3.10 报 `cannot import name 'docstring'`，3D 投影直接不可用。

    这里把 pip matplotlib 旁边那份挂进 `sys.modules`，**只影响本进程**，不改环境。
    找不到同版本 mplot3d 时什么也不做（系统里只有一套 matplotlib 的正常情况）。
    """
    import sys as _sys
    import types as _types

    base = Path(matplotlib.__file__).resolve().parent.parent / "mpl_toolkits"
    if not (base / "mplot3d").is_dir():
        return
    # `import matplotlib` 本身已经把 Debian 那份 mpl_toolkits 拉进来了；
    # 只要 mplot3d 还没被导入、且当前不是同版本目录，就换掉顶层包。
    if "mpl_toolkits.mplot3d" in _sys.modules:
        return
    current = _sys.modules.get("mpl_toolkits")
    current_file = getattr(current, "__file__", None)
    if current_file and Path(current_file).resolve().parent == base:
        return
    package = _types.ModuleType("mpl_toolkits")
    package.__path__ = [str(base)]
    package.__package__ = "mpl_toolkits"
    _sys.modules["mpl_toolkits"] = package


_install_mpl_toolkits_shim()

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import MaxNLocator  # noqa: E402

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import style  # noqa: E402
from captions import (  # noqa: E402
    CANDIDATE_CAPTION_HEADER,
    CAPTION_HEADER,
    update_caption_file,
)

SCENES = ("hover", "square", "lemniscate")

SCENE_TITLE_EN = {
    "hover": "Hover",
    "square": "Square",
    "lemniscate": "Lemniscate",
}
SCENE_TITLE_CN = {
    "hover": "悬停",
    "square": "正方形",
    "lemniscate": "8 字",
}

#: 正文图号（第 10 章装配时沿用；图注里先固定下来）。
FIGURE_IDS = {"hover": "T1", "square": "T2", "lemniscate": "T3"}

#: 数据列 → 内部键。
_COLUMNS = {
    "t": "t_s",
    "x": "x_m",
    "y": "y_m",
    "z": "z_m",
    # 姿态列给票 03 追加的 pitch/roll 图用；轨迹图只取前四列。
    "roll": "roll_rad",
    "pitch": "pitch_rad",
    "yaw": "yaw_rad",
}

DATA_DIR = REPO / "docs/report/data"
FIGURE_DIR = REPO / "docs/report/figures"
CAPTION_FILE = REPO / "docs/report/generated/figure-captions.md"
#: 候选图注单独成文件：`figure-captions.md` 会被装配票内联进正文，
#: 3D 候选不能跟着进去（spec §6「不放进正文」）。
CANDIDATE_CAPTION_FILE = (
    REPO / "docs/report/generated/figure-captions-candidates.md"
)


# ─────────────────────────── 读数据 ───────────────────────────


def load_scene(
    data_dir: Path,
    scene: str,
    *,
    columns: Dict[str, str] | None = None,
    sources: Sequence[str] | None = None,
) -> Dict[str, Dict[str, np.ndarray]]:
    """读 `data_dir/<scene>.csv`，按源拆成 numpy 数组。

    缺列、缺源、非数值、网格不一致都直接抛错 —— 静默补 0 会让图看起来
    很完整、结论却反了（票 01 的同一条纪律）。

    `columns` / `sources` 让只画两源的图（如控制量）复用同一条读取与校验
    路径：目标与 offboard 的控制量列本来就是空的，不能按四源全列硬读。
    """
    columns = dict(_COLUMNS if columns is None else columns)
    sources = tuple(style.SOURCES if sources is None else sources)
    if not sources:
        raise ValueError("至少需要一个源")
    path = Path(data_dir) / f"{scene}.csv"
    if not path.exists():
        raise FileNotFoundError(f"缺少数据文件 {path}（先跑票 01 的 build.py）")

    raw: Dict[str, Dict[str, list]] = {
        source: {key: [] for key in columns} for source in sources
    }

    with path.open(newline="") as fh:
        reader = csv.DictReader(fh)
        fieldnames = reader.fieldnames or []
        needed = ["source", *columns.values()]
        missing = [name for name in needed if name not in fieldnames]
        if missing:
            raise ValueError(f"{path} 缺列：{missing}")
        for lineno, row in enumerate(reader, start=2):
            source = row["source"]
            if source not in raw:
                # 只读部分源时（如控制量只读模型预测控制两源），跳过其余
                # 已知源；真正未知的源名仍然报错。
                if source in style.SOURCES:
                    continue
                raise ValueError(f"{path}:{lineno} 未知源 {source!r}")
            for key, column in columns.items():
                text = row[column]
                if text is None or text == "":
                    raise ValueError(f"{path}:{lineno} 源 {source} 的 {column} 为空")
                try:
                    value = float(text)
                except ValueError as exc:
                    raise ValueError(
                        f"{path}:{lineno} 源 {source} 的 {column} 不是数值：{text!r}"
                    ) from exc
                raw[source][key].append(value)

    data: Dict[str, Dict[str, np.ndarray]] = {}
    for source in sources:
        if not raw[source]["t"]:
            raise ValueError(f"{path} 缺少源 {source}")
        data[source] = {key: np.asarray(values, dtype=float)
                        for key, values in raw[source].items()}
        for key, values in data[source].items():
            if not np.all(np.isfinite(values)):
                raise ValueError(f"{path} 源 {source} 的 {key} 含非有限值")

    reference_t = data[sources[0]]["t"]
    for source in sources[1:]:
        if data[source]["t"].shape != reference_t.shape:
            raise ValueError(f"{path} 源 {source} 的网格点数与其他源不同")
        if not np.allclose(data[source]["t"], reference_t, atol=1e-9, rtol=0.0):
            raise ValueError(f"{path} 源 {source} 与 {sources[0]} 不在同一时间网格")
    return data


def load_metadata(data_dir: Path, scene: str) -> dict:
    """读同名 `.json`，只为图注取来源与窗口口径（不是画图数据）。"""
    path = Path(data_dir) / f"{scene}.json"
    if not path.exists():
        raise FileNotFoundError(f"缺少元数据 {path}（图注需要窗口与来源）")
    with path.open(encoding="utf-8") as fh:
        return json.load(fh)


def _symmetric_limit(values: np.ndarray) -> float:
    """把一维数据包成以 0 为中心的对称范围，保留约 5% 余量、取到 0.1 m。"""
    span = float(np.max(np.abs(values)))
    if not np.isfinite(span):
        raise ValueError("坐标范围含非有限值")
    if span == 0.0:
        return 0.1
    return float(np.ceil(span * 1.05 * 10.0) / 10.0)


def compute_axis_limits(
    data_by_scene: Dict[str, Dict[str, Dict[str, np.ndarray]]],
) -> Dict[str, tuple[float, float]]:
    """三张轨迹图的 XY 与 3D 候选共用的坐标轴范围。

    - `xy`：x、y 取同一对称范围，`set_aspect("equal")` 下三张图尺度一致；
    - `z`：3D 候选共用，以 0 为中心。

    **Z–t 不在这里**：操作者 2026-10-18 明确 Z–t 不需要统一，由 `plot_scene`
    按各场景自己的窗口与 z 数据范围画。全部从数据现算，换数据重跑会自动变。
    """
    if not data_by_scene:
        raise ValueError("没有可用的轨迹数据")

    def collect(axis: str) -> np.ndarray:
        return np.concatenate([
            data[source][axis]
            for data in data_by_scene.values()
            for source in style.SOURCES
        ])

    xy_limit = max(_symmetric_limit(collect("x")),
                   _symmetric_limit(collect("y")))
    z_limit = _symmetric_limit(collect("z"))
    return {
        "xy": (-xy_limit, xy_limit),
        "z": (-z_limit, z_limit),
    }


# ─────────────────────────── 画图 ───────────────────────────


def _origin_lines(ax) -> None:
    """画 x=0 / y=0（或 z=0）参考线：读者一眼能看出四源是否同一原点。"""
    ax.axhline(0.0, color=style.ORIGIN_LINE_COLOR,
               linewidth=style.ORIGIN_LINE_WIDTH, zorder=0)
    ax.axvline(0.0, color=style.ORIGIN_LINE_COLOR,
               linewidth=style.ORIGIN_LINE_WIDTH, zorder=0)


def plot_scene(scene: str, data: Dict[str, Dict[str, np.ndarray]],
               limits: Dict[str, tuple[float, float]]):
    """一张图：左 XY 俯视、右 Z-t。返回 figure，由调用方保存/关闭。"""
    fig, (ax_xy, ax_zt) = plt.subplots(1, 2, figsize=style.FIGSIZE_2PANEL)

    for source in style.SOURCES:
        values = data[source]
        ax_xy.plot(values["x"], values["y"], **style.line_kwargs(source))
        ax_zt.plot(values["t"], values["z"], **style.line_kwargs(source))
        style.start_marker(ax_xy, values["x"][0], values["y"][0], source)
        style.start_marker(ax_zt, values["t"][0], values["z"][0], source)

    _origin_lines(ax_xy)
    _origin_lines(ax_zt)

    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("XY (top view)")
    # 三张图共用同一组范围；XY 两轴同范围，等比例下尺度才一致。
    ax_xy.set_xlim(*limits["xy"])
    ax_xy.set_ylim(*limits["xy"])
    ax_xy.set_aspect("equal", adjustable="box")

    ax_zt.set_xlabel("t [s] (relative to /traj/start)")
    ax_zt.set_ylabel("z [m]")
    ax_zt.set_title("Z-t")
    # 操作者明确 Z–t 不统一：时间轴到本场景公共窗终点，z 按本场景数据范围。
    zt_limit = _symmetric_limit(np.concatenate(
        [data[source]["z"] for source in style.SOURCES]))
    ax_zt.set_xlim(0.0, float(data["target"]["t"][-1]))
    ax_zt.set_ylim(-zt_limit, zt_limit)

    fig.suptitle(f"{SCENE_TITLE_EN[scene]}: four sources", y=0.985)
    fig.legend(
        handles=style.legend_handles(),
        loc="upper center",
        ncol=5,
        frameon=False,
        bbox_to_anchor=(0.5, 0.935),
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.86))
    return fig


def plot_scene_3d(scene: str, data: Dict[str, Dict[str, np.ndarray]],
                  limits: Dict[str, tuple[float, float]]):
    """3D 候选：四源叠加。三张候选共用同一组 x/y/z 范围与盒体比例。"""
    fig = plt.figure(figsize=style.FIGSIZE_3D)
    # 3D 轴的绘制盒会按 box_aspect 缩在 axes rect 里，`tight_layout` 对它的
    # 留白算不准；这里直接指定 rect，顶部留给图例与标题。
    ax = fig.add_axes((0.02, 0.01, 0.96, 0.82), projection="3d")

    for source in style.SOURCES:
        values = data[source]
        kwargs = style.line_kwargs(source)
        kwargs.pop("solid_capstyle", None)  # 3D Line3D 不接受
        ax.plot(values["x"], values["y"], values["z"], **kwargs)
        style.start_marker_3d(
            ax, values["x"][0], values["y"][0], values["z"][0], source)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(f"{SCENE_TITLE_EN[scene]}: 3D trajectory (candidate)", pad=14)
    ax.view_init(elev=24, azim=-58)
    # 与正文图同一组范围；三张候选的盒体比例也完全相同。
    ax.set_xlim(*limits["xy"])
    ax.set_ylim(*limits["xy"])
    ax.set_zlim(*limits["z"])
    ax.xaxis.set_major_locator(MaxNLocator(nbins=6))
    ax.yaxis.set_major_locator(MaxNLocator(nbins=6))
    ax.zaxis.set_major_locator(MaxNLocator(nbins=4))
    # box_aspect 与各轴实际跨度成正比：三张图的 z 都只有 x/y 的约 1/6，
    # 盒体会偏薄；这是统一范围后的真实比例，不是缩放错误。
    ax.set_box_aspect(
        (limits["xy"][1] - limits["xy"][0],
         limits["xy"][1] - limits["xy"][0],
         limits["z"][1] - limits["z"][0]),
        zoom=1.12,
    )

    fig.legend(
        handles=style.legend_handles(),
        loc="upper center",
        ncol=5,
        frameon=False,
        bbox_to_anchor=(0.5, 0.985),
    )
    return fig


# ─────────────────────────── 图注 ───────────────────────────


def _sim_path(meta: dict) -> str:
    return meta["inputs"]["sim_summary"]["path"]


def _window_text(meta: dict) -> str:
    grid = meta["grid"]
    return (
        f"0.0000 - {grid['common_end_s']:.4f} s"
        f"（{grid['common_end_reason']}）"
    )


def _axis_text(limits: Dict[str, tuple[float, float]]) -> str:
    """图注里的统一坐标轴说明（数值现算，不写死）。"""
    return (
        f"XY 两轴 {limits['xy'][0]:.1f} 到 {limits['xy'][1]:.1f} m；"
        f"3D 视图的 z 轴 {limits['z'][0]:.1f} 到 {limits['z'][1]:.1f} m"
    )


def _scene_caption(scene: str, meta: dict, command: str,
                   limits: Dict[str, tuple[float, float]]) -> str:
    """一张正文图的中文图注。"""
    figure_id = FIGURE_IDS[scene]
    title = SCENE_TITLE_CN[scene]
    offboard_bag = meta["inputs"]["offboard_bag"]["path"]
    mpc_bag = meta["inputs"]["mpc_bag"]["path"]
    sim_path = _sim_path(meta)

    lines = [
        f"### {figure_id} {title}（{scene}）轨迹：四源对比",
        "",
        f"- 文件：`figures/fig-traj-{scene}.png`、`figures/fig-traj-{scene}.svg`（300 dpi）",
        f"- 数据：`data/{scene}.csv`（同名 `.json` 记输入 sha256 与窗口口径）",
        f"- 窗口：{_window_text(meta)}",
        f"- 来源：目标与 offboard 来自 `{offboard_bag}`；"
        f"模型预测控制仿真来自 `{sim_path}`；"
        f"模型预测控制实飞来自 `{mpc_bag}`。",
        "- 坐标：四源各自减去自身的轨迹原点（`AUTO/WAIT_START` 期间的指令值），单位为 m；"
        "时间零点为 `/traj/start` 的包记录时间戳。",
        "- 实际轨迹：实飞两源取 `/mavros/vision_pose/pose`；仿真源取 SITL 的 `x0`"
        "（合成动捕）。offboard 与模型预测控制两组控制器看到的反馈源不同，"
        "详见 `data/README.md`。",
        "- 样式：目标为虚线，实际轨迹为实线；四源起点用同一种空心圆标记；"
        "左图 XY 等比例俯视，右图为高度-时间。",
        f"- 坐标轴统一：三张轨迹图的 XY 与 3D 候选共用同一组范围——"
        f"{_axis_text(limits)}；范围由全部数据现算。"
        f"Z–t **不统一**：时间轴到本场景公共窗终点，z 按本场景数据范围。",
        f"- 生成：`{command}`",
    ]
    if scene == "hover":
        end = meta["grid"]["common_end_s"]
        lines.append(
            f"- 注意：offboard 包截止 {end:.2f} s（录制截断，不是控制失败）。"
        )
        lines.append(
            f"- 裁剪口径：{meta['grid']['window_trim_reason']}"
        )
    return "\n".join(lines)


def _candidate_caption(scene: str, meta: dict, command: str,
                       limits: Dict[str, tuple[float, float]]) -> str:
    return "\n".join([
        f"### {FIGURE_IDS[scene]}-3D {SCENE_TITLE_CN[scene]}（{scene}）3D 轨迹候选",
        "",
        f"- 文件：`figures/candidates/fig-traj-{scene}-3d.png`、"
        f"`figures/candidates/fig-traj-{scene}-3d.svg`（300 dpi）",
        "- 状态：**未进入正文**，待操作者看图后裁定去留。",
        f"- 数据、窗口、坐标与来源同 {FIGURE_IDS[scene]}："
        f"`data/{scene}.csv`，窗口 {_window_text(meta)}。",
        f"- 坐标轴统一：三张候选与正文图共用同一组范围——{_axis_text(limits)}；"
        "盒体按该范围成比例，z 只有厘米级，因此盒体偏薄。",
        "- 注意：3D 透视视角不用于读数，只用于观察轨迹形状。",
        f"- 生成：`{command}`",
    ])


def write_captions(
    path: Path,
    scene_blocks: Dict[str, str],
    *,
    candidate_path: Path | None = None,
    candidate_blocks: Dict[str, str] | None = None,
) -> None:
    """写正文图注；候选图注写到**另一个文件**，避免装配时被内联进正文。"""
    update_caption_file(Path(path), CAPTION_HEADER, "phase5-traj",
                        scene_blocks)
    if candidate_blocks:
        if candidate_path is None:
            raise ValueError("写候选图注必须给 candidate_path")
        update_caption_file(
            Path(candidate_path), CANDIDATE_CAPTION_HEADER,
            "phase5-traj-candidates", candidate_blocks)


# ─────────────────────────── 入口 ───────────────────────────


def _display(path: Path) -> str:
    """打印用路径：仓库内显示相对路径，测试的临时目录原样显示。"""
    try:
        return str(Path(path).relative_to(REPO))
    except ValueError:
        return str(path)


def _command(scene_arg: str, candidates: bool) -> str:
    command = f"python3 tools/report/plot_traj.py --scene {scene_arg}"
    if candidates:
        command += " --candidates"
    return command


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", choices=("all", *SCENES), default="all")
    parser.add_argument("--candidates", action="store_true",
                        help="同时生成 3D 候选图（默认不生成）")
    parser.add_argument("--data-dir", type=Path, default=DATA_DIR)
    parser.add_argument("--outdir", type=Path, default=FIGURE_DIR)
    parser.add_argument("--captions", type=Path, default=CAPTION_FILE)
    parser.add_argument("--candidate-captions", type=Path,
                        default=CANDIDATE_CAPTION_FILE)
    parser.add_argument("--no-captions", action="store_true",
                        help="只画图，不写 figure-captions.md")
    args = parser.parse_args(argv)

    scenes = SCENES if args.scene == "all" else (args.scene,)
    command = _command(args.scene, args.candidates)
    scene_blocks: Dict[str, str] = {}
    candidate_blocks: Dict[str, str] = {}
    candidate_dir = Path(args.outdir) / "candidates"

    # 坐标轴必须用全部三个场景的数据定，否则部分重跑会和其他图不一致。
    data_by_scene = {scene: load_scene(args.data_dir, scene)
                     for scene in SCENES}
    limits = compute_axis_limits(data_by_scene)

    for scene in scenes:
        data = data_by_scene[scene]
        meta = load_metadata(args.data_dir, scene)
        n_points = len(data["target"]["t"])

        fig = plot_scene(scene, data, limits)
        png, svg = style.save(fig, args.outdir, f"fig-traj-{scene}")
        plt.close(fig)
        print(f"[ok] {_display(png)}  ({n_points} points/source)")
        print(f"[ok] {_display(svg)}")
        scene_blocks[scene] = _scene_caption(scene, meta, command, limits)

        if args.candidates:
            fig3d = plot_scene_3d(scene, data, limits)
            png, svg = style.save(
                fig3d, candidate_dir, f"fig-traj-{scene}-3d")
            plt.close(fig3d)
            print(f"[ok] {_display(png)}  (candidate)")
            print(f"[ok] {_display(svg)}")
            candidate_blocks[scene] = _candidate_caption(scene, meta, command, limits)

    if not args.no_captions:
        write_captions(
            args.captions,
            scene_blocks,
            candidate_path=args.candidate_captions,
            candidate_blocks=candidate_blocks or None,
        )
        print(f"[ok] {_display(Path(args.captions))}")
        if candidate_blocks:
            print(f"[ok] {_display(Path(args.candidate_captions))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
