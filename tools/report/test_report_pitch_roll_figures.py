#!/usr/bin/env python3
"""票 03 追加（2026-10-18）：12 张 pitch/roll 图的**产物与坐标口径**测试。

操作者的两条硬要求，各对应一组测试：

| 要求 | 测什么 |
|---|---|
| 每种情况一张图（3 场景 × 4 源 = 12 张） | 文件齐全、非空、PNG 300 dpi、SVG 可解析 |
| **坐标轴统一** | 12 张 SVG 的纵轴刻度集合完全相同；同场景 4 张横轴刻度相同 |
| 每张图画 pitch、roll 与零线 | 两条源色线 + `#555555` 虚线都在 SVG 里 |

另外守住：目标姿态恒 0、图注分块不覆盖轨迹图注、部分重跑不动别的块、
重跑逐字节一致。

图生成到 pytest 的临时目录，**不覆盖** `docs/report/figures/` 里的提交版。
"""

from __future__ import annotations

import hashlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import plot_pitch_roll  # noqa: E402
import plot_traj  # noqa: E402
import style  # noqa: E402

SCENES = plot_traj.SCENES
SOURCES = style.SOURCES
STEMS = [f"fig-pitchroll-{scene}-{source}"
         for scene in SCENES for source in SOURCES]


def _svg_texts(path: Path) -> list[str]:
    root = ET.parse(path).getroot()
    return [node.text for node in root.iter()
            if node.tag.endswith("text") and node.text]


def _tick_labels(path: Path, axis: str) -> set[str]:
    """从 SVG 里取 `axis` 轴（`x` / `y`）的刻度文字。

    matplotlib 把每个刻度包在 `<g id="ytick_1">` 里；用 id 前缀认，不靠顺序。
    """
    root = ET.parse(path).getroot()
    prefix = f"{axis}tick"
    labels: set[str] = set()
    for group in root.iter():
        if group.tag.endswith("g") and (group.get("id") or "").startswith(prefix):
            for node in group.iter():
                if node.tag.endswith("text") and node.text:
                    labels.add(node.text.strip())
    return labels


def _y_tick_labels(path: Path) -> set[str]:
    return _tick_labels(path, "y")


def _x_tick_labels(path: Path) -> set[str]:
    return _tick_labels(path, "x")


@pytest.fixture(scope="module")
def generated(tmp_path_factory):
    """完整生成 12 张图与图注，返回 `(图目录, 图注路径)`。"""
    outdir = tmp_path_factory.mktemp("pitchroll-figures")
    generated_dir = tmp_path_factory.mktemp("pitchroll-generated")
    captions = generated_dir / "figure-captions.md"
    rc = plot_pitch_roll.main([
        "--scene", "all",
        "--outdir", str(outdir),
        "--captions", str(captions),
    ])
    assert rc == 0
    return outdir, captions


# ─────────────────────────── 产物 ───────────────────────────


def test_twelve_figures_exist_and_are_non_empty(generated):
    outdir, _ = generated
    assert len(STEMS) == 12
    for stem in STEMS:
        for suffix in ("png", "svg"):
            path = outdir / f"{stem}.{suffix}"
            assert path.is_file(), f"缺少 {path}"
            assert path.stat().st_size > 10_000, f"{path} 太小，像是空图"


def test_png_is_300_dpi(generated):
    outdir, _ = generated
    for stem in STEMS:
        path = outdir / f"{stem}.png"
        with Image.open(path) as image:
            dpi = image.info.get("dpi")
            assert dpi is not None, f"{path} 没有 dpi 元数据"
            assert dpi[0] == pytest.approx(300, rel=1e-3), f"{path} dpi={dpi}"
            assert image.width >= 1800 and image.height >= 800, image.size


def test_svg_keeps_searchable_english_text(generated):
    outdir, _ = generated
    for stem in STEMS:
        path = outdir / f"{stem}.svg"
        texts = _svg_texts(path)
        assert texts, f"{path} 里没有可搜索的文字"
        joined = " ".join(texts)
        for label in ("Pitch", "Roll", "Zero", "angle [deg]"):
            assert label in joined, f"{path} 缺少 {label!r}"
        bad = sorted({ch for text in texts for ch in text if ord(ch) > 127})
        assert not bad, f"{path} 含非 ASCII 字符 {bad}"


def test_each_figure_has_two_curves_and_a_zero_line(generated):
    outdir, _ = generated
    for stem in STEMS:
        svg = (outdir / f"{stem}.svg").read_text(encoding="utf-8")
        assert "stroke: #6a3d9a" in svg, f"{stem} 缺 pitch 线"
        assert "stroke: #e69f00" in svg, f"{stem} 缺 roll 线"
        assert "stroke: #555555" in svg, f"{stem} 缺零线"
        assert "stroke-dasharray" in svg, f"{stem} 零线不是虚线"


# ─────────────────────────── 坐标轴统一（操作者硬要求） ───────────────────────────


def test_all_twelve_share_the_same_y_axis(generated):
    """纵轴统一：任一张图的 y 刻度集合不同，就说明某张 autoscale 了。"""
    outdir, _ = generated
    tick_sets = {stem: _y_tick_labels(outdir / f"{stem}.svg") for stem in STEMS}
    reference = tick_sets[STEMS[0]]
    assert reference, "没解析到 y 轴刻度"
    # 正负两侧都有刻度，才能看出范围是否对称、是否覆盖数据。
    assert {"-10", "10"} <= reference
    for stem, ticks in tick_sets.items():
        assert ticks == reference, f"{stem} 的 y 刻度 {ticks} != {reference}"


def test_same_scene_shares_the_same_x_axis(generated):
    outdir, _ = generated
    for scene in SCENES:
        ticks = {
            tuple(sorted(_x_tick_labels(
                outdir / f"fig-pitchroll-{scene}-{source}.svg")))
            for source in SOURCES
        }
        assert len(ticks) == 1, f"{scene} 的四张图横轴刻度不一致：{ticks}"


def test_limits_cover_every_value_without_autoscale():
    data = plot_pitch_roll.load_scenes(plot_pitch_roll.DATA_DIR)
    ymin, ymax = plot_pitch_roll.compute_angle_limits(data)
    assert ymin == -ymax, "纵轴应以 0 为中心，零线才在正中"
    values = np.concatenate([
        np.degrees(data[scene][source][angle])
        for scene in SCENES for source in SOURCES for angle in ("pitch", "roll")
    ])
    span = float(np.max(np.abs(values)))
    assert ymin < float(values.min()) and ymax > float(values.max())
    # 留的余量不超过 2 deg：范围跟着数据走，不许写死一个巨大的安全框。
    assert span < ymax <= span + 2.0


def test_scene_window_matches_common_end():
    data = plot_pitch_roll.load_scenes(plot_pitch_roll.DATA_DIR)
    for scene in SCENES:
        meta = plot_traj.load_metadata(plot_pitch_roll.DATA_DIR, scene)
        _, end = plot_pitch_roll.scene_window(data[scene])
        # CSV 里的 t 只保留有限位小数；1e-6 足够卡住口径，不卡浮点尾数。
        assert end == pytest.approx(meta["grid"]["common_end_s"], abs=1e-6)


def test_target_attitude_is_exactly_zero():
    data = plot_traj.load_scene(plot_pitch_roll.DATA_DIR, "hover")
    for angle in ("pitch", "roll"):
        np.testing.assert_array_equal(data["target"][angle], 0.0)


# ─────────────────────────── 图注 ───────────────────────────


def test_captions_have_twelve_blocks_with_window_source_and_axis(generated):
    _, captions = generated
    text = captions.read_text(encoding="utf-8")
    for scene in SCENES:
        for source in SOURCES:
            assert f"<!-- BEGIN phase5-pitchroll:{scene}:{source} -->" in text
            assert f"<!-- END phase5-pitchroll:{scene}:{source} -->" in text
        assert f"data/{scene}.csv" in text
    assert "bag/3_traj1_hover" in text
    assert "bag/4_traj2_square" in text
    assert "bag/5_traj3_lemniscate" in text
    assert "R-hover" in text and "R-square" in text and "R-lemniscate" in text
    assert "12 张图共用" in text
    assert "零线为 0 deg" in text
    assert "python3 tools/report/plot_pitch_roll.py --scene all" in text
    # 悬停截断原因要写全，不能只说"取最小"。
    assert "录制截断：progress 终值 0.661" in text
    assert "offboard 包截止 39.65 s（录制截断，不是控制失败）" in text


def test_caption_writer_preserves_traj_and_other_pitchroll_blocks(tmp_path):
    path = tmp_path / "figure-captions.md"
    path.write_text(
        "# 图注\n\n"
        "<!-- BEGIN phase5-traj:hover -->\n轨迹图注\n<!-- END phase5-traj:hover -->\n\n"
        "<!-- BEGIN phase5-pitchroll:square:target -->\n旧的正方形图注\n"
        "<!-- END phase5-pitchroll:square:target -->\n",
        encoding="utf-8",
    )
    rc = plot_pitch_roll.main([
        "--scene", "hover",
        "--outdir", str(tmp_path / "figures"),
        "--captions", str(path),
    ])
    assert rc == 0
    text = path.read_text(encoding="utf-8")
    assert "轨迹图注" in text, "轨迹图注被 pitch/roll 脚本覆盖了"
    assert "旧的正方形图注" in text, "部分重跑动了别的场景/源"
    assert "phase5-pitchroll:hover:target" in text


# ─────────────────────────── 边界 ───────────────────────────


def test_plotter_does_not_read_raw_bags():
    source = Path(plot_pitch_roll.__file__).read_text(encoding="utf-8")
    assert "rosbag2" not in source
    assert "import extract" not in source


def test_rerun_is_byte_identical(tmp_path):
    """同一输入连跑两次：PNG/SVG/图注逐字节一致（`svg.hashsalt` + 无 Date）。"""
    outdir = tmp_path / "figures"
    captions = tmp_path / "figure-captions.md"
    args = ["--scene", "hover", "--outdir", str(outdir),
            "--captions", str(captions)]

    def digest() -> dict[str, str]:
        paths = sorted(outdir.glob("*.png")) + sorted(outdir.glob("*.svg"))
        paths.append(captions)
        return {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in paths}

    assert plot_pitch_roll.main(args) == 0
    first = digest()
    assert plot_pitch_roll.main(args) == 0
    assert digest() == first


def test_partial_rerun_keeps_global_y_axis(generated, tmp_path):
    """只画一个场景时，纵轴仍由全部 12 组数据定，不能缩成该场景自身范围。"""
    reference_dir, _ = generated
    outdir = tmp_path / "hover-only"
    assert plot_pitch_roll.main([
        "--scene", "hover",
        "--outdir", str(outdir),
        "--no-captions",
    ]) == 0
    for source in SOURCES:
        stem = f"fig-pitchroll-hover-{source}"
        assert (_y_tick_labels(outdir / f"{stem}.svg")
                == _y_tick_labels(reference_dir / f"{stem}.svg"))
    # 若按悬停自身数据 autoscale，纵轴只会在 ±2 deg 量级，这里必须仍是 ±14。
    assert _y_tick_labels(outdir / "fig-pitchroll-hover-target.svg") == {
        "-10", "-5", "0", "5", "10"}
