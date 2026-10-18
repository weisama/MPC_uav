#!/usr/bin/env python3
"""票 03（Phase 5）：轨迹图的**产物与口径**测试。

这里不测控制器，只测票面那几条容易悄悄坏掉的约定：

| 组 | 回答的问题 |
|---|---|
| 数据入口 | 缺源/缺列/四源不在同一网格时**直接失败**，不静默补值 |
| 产物 | 三张正文图 + 三张 3D 候选都在、非空、PNG 300 dpi、SVG 可解析 |
| 样式 | 目标虚线 / 实际实线；图内文字全是英文（不依赖中文字体） |
| 图注 | 含窗口、来源、生成命令；悬停图注写明"录制截断，不是控制失败" |
| 边界 | 3D 候选默认不生成，且没有被章节正文引用 |

图生成到 pytest 的临时目录，**不覆盖** `docs/report/figures/` 里的提交版。
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import plot_traj  # noqa: E402
import style  # noqa: E402

SCENES = plot_traj.SCENES
FIGURES = [f"fig-traj-{scene}" for scene in SCENES]
CANDIDATES = [f"fig-traj-{scene}-3d" for scene in SCENES]


def _svg_texts(path: Path) -> list[str]:
    root = ET.parse(path).getroot()
    return [node.text for node in root.iter()
            if node.tag.endswith("text") and node.text]


@pytest.fixture(scope="module")
def generated(tmp_path_factory):
    """跑一次完整生成（含候选），返回 `(图目录, 正文图注, 候选图注)`。"""
    outdir = tmp_path_factory.mktemp("figures")
    generated_dir = tmp_path_factory.mktemp("generated")
    captions = generated_dir / "figure-captions.md"
    candidate_captions = generated_dir / "figure-captions-candidates.md"
    rc = plot_traj.main([
        "--scene", "all",
        "--candidates",
        "--outdir", str(outdir),
        "--captions", str(captions),
        "--candidate-captions", str(candidate_captions),
    ])
    assert rc == 0
    return outdir, captions, candidate_captions


# ─────────────────────────── 产物 ───────────────────────────


def test_main_figures_exist_and_are_non_empty(generated):
    outdir, _, _ = generated
    for stem in FIGURES:
        for suffix in ("png", "svg"):
            path = outdir / f"{stem}.{suffix}"
            assert path.is_file(), f"缺少 {path}"
            assert path.stat().st_size > 10_000, f"{path} 太小，像是空图"


def test_candidate_figures_exist_and_are_non_empty(generated):
    outdir, _, _ = generated
    for stem in CANDIDATES:
        for suffix in ("png", "svg"):
            path = outdir / "candidates" / f"{stem}.{suffix}"
            assert path.is_file(), f"缺少候选图 {path}"
            assert path.stat().st_size > 10_000, f"{path} 太小，像是空图"


def _all_stems() -> list[str]:
    return FIGURES + [f"candidates/{stem}" for stem in CANDIDATES]


def test_png_is_300_dpi_and_large_enough(generated):
    outdir, _, _ = generated
    for stem in _all_stems():
        path = outdir / f"{stem}.png"
        with Image.open(path) as image:
            dpi = image.info.get("dpi")
            assert dpi is not None, f"{path} 没有 dpi 元数据"
            assert dpi[0] == pytest.approx(300, rel=1e-3), f"{path} dpi={dpi}"
            # 候选图是竖版，正文图是横版；只卡下限，避免换个场景就假红。
            assert image.width >= 1800 and image.height >= 1000, image.size


def test_svg_is_parseable_and_keeps_text(generated):
    outdir, _, _ = generated
    for stem in _all_stems():
        path = outdir / f"{stem}.svg"
        texts = _svg_texts(path)
        assert texts, f"{path} 里没有可搜索的文字"
        joined = " ".join(texts)
        for label in ("Target", "Offboard flight", "MPC simulation", "MPC flight"):
            assert label in joined, f"{path} 缺少图例 {label!r}"


def test_figure_text_is_english_only(generated):
    """图内文字全 ASCII：Jetson 上没装中文字体，图不能依赖它。"""
    outdir, _, _ = generated
    for stem in _all_stems():
        texts = _svg_texts(outdir / f"{stem}.svg")
        bad = sorted({ch for text in texts for ch in text if ord(ch) > 127})
        assert not bad, f"{stem}.svg 含非 ASCII 字符 {bad}"


# ─────────────────────────── 样式约定 ───────────────────────────


def test_target_is_dashed_and_actuals_are_solid():
    assert style.SOURCE_LINESTYLES["target"] != "-"
    for source in ("offboard", "mpc_sim", "mpc_real"):
        assert style.SOURCE_LINESTYLES[source] == "-"


def test_each_source_has_a_distinct_color():
    colors = [style.SOURCE_COLORS[source] for source in style.SOURCES]
    assert len(set(colors)) == len(colors)


# ─────────────────────────── 数据入口 ───────────────────────────


def test_load_scene_keeps_four_sources_on_one_grid():
    data = plot_traj.load_scene(plot_traj.DATA_DIR, "hover")
    assert set(data) == set(style.SOURCES)
    reference = data["target"]["t"]
    for source in style.SOURCES:
        np.testing.assert_allclose(data[source]["t"], reference, atol=1e-9, rtol=0.0)


def test_load_scene_fails_on_missing_source(tmp_path):
    path = tmp_path / "broken.csv"
    path.write_text(
        "source,t_s,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad\n"
        "target,0.0,0,0,0,0,0,0\n"
        "target,0.01,0,0,0,0,0,0\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="缺少源"):
        plot_traj.load_scene(tmp_path, "broken")


def test_load_scene_fails_on_empty_column(tmp_path):
    path = tmp_path / "broken.csv"
    path.write_text(
        "source,t_s,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad\n"
        "target,0.0,0,0,0,0,0,0\n"
        "target,,0,0,0,0,0,0\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="为空"):
        plot_traj.load_scene(tmp_path, "broken")


def test_plotter_does_not_read_raw_bags():
    source = Path(plot_traj.__file__).read_text(encoding="utf-8")
    assert "rosbag2" not in source
    assert "import extract" not in source


# ─────────────────────────── 图注 ───────────────────────────


def test_captions_include_window_sources_and_command(generated):
    _, captions, _ = generated
    text = captions.read_text(encoding="utf-8")
    # 三个场景各有一条
    for scene in SCENES:
        assert f"data/{scene}.csv" in text
        assert f"figures/fig-traj-{scene}.png" in text
    # 窗口与来源
    assert "39.6494" in text
    assert "bag/3_traj1_hover" in text
    assert "bag/4_traj2_square" in text
    assert "bag/5_traj3_lemniscate" in text
    assert "R-hover" in text and "R-square" in text and "R-lemniscate" in text
    # 生成命令
    assert "python3 tools/report/plot_traj.py --scene all --candidates" in text
    # 悬停截断必须写明，避免被读成控制失败
    assert "offboard 包截止 39.65 s（录制截断，不是控制失败）" in text


def test_candidate_caption_says_not_in_body(generated):
    _, captions, candidate_captions = generated
    candidate_text = candidate_captions.read_text(encoding="utf-8")
    assert "未进入正文" in candidate_text
    assert "不保证等比例" in candidate_text
    # 正文图注里不能出现候选图：装配票会内联这个文件。
    assert "candidates/" not in captions.read_text(encoding="utf-8")


def test_caption_writer_preserves_other_sections(tmp_path):
    path = tmp_path / "figure-captions.md"
    path.write_text(
        "# 图注\n\n<!-- BEGIN other-ticket -->\n别动我\n<!-- END other-ticket -->\n",
        encoding="utf-8",
    )
    plot_traj.write_captions(path, {"hover": "新内容"})
    text = path.read_text(encoding="utf-8")
    assert "别动我" in text
    assert "新内容" in text
    assert "phase5-traj:hover" in text


def test_partial_scene_rerun_keeps_other_captions(tmp_path):
    path = tmp_path / "figure-captions.md"
    plot_traj.write_captions(
        path,
        {"hover": "HOVER_BLOCK", "square": "SQUARE_BLOCK",
         "lemniscate": "LEMNISCATE_BLOCK"},
    )
    plot_traj.write_captions(path, {"hover": "HOVER_BLOCK_2"})
    text = path.read_text(encoding="utf-8")
    assert "HOVER_BLOCK_2" in text
    assert "SQUARE_BLOCK" in text
    assert "LEMNISCATE_BLOCK" in text


# ─────────────────────────── 边界 ───────────────────────────


def test_candidates_are_not_generated_by_default(tmp_path):
    rc = plot_traj.main([
        "--scene", "hover",
        "--outdir", str(tmp_path),
        "--no-captions",
    ])
    assert rc == 0
    assert (tmp_path / "fig-traj-hover.png").is_file()
    assert not (tmp_path / "candidates").exists()


def test_candidates_are_not_referenced_by_chapters():
    chapters = REPO / "docs/report/chapters"
    for path in sorted(chapters.glob("*.md")):
        text = path.read_text(encoding="utf-8")
        assert "candidates/" not in text, f"{path.name} 引用了候选图目录"
        assert "-3d" not in text, f"{path.name} 引用了 3D 候选图"
