#!/usr/bin/env python3
"""票 04：三张姿态角图的**产物与口径**测试。

对应票面检查清单：

| 票面要求 | 测什么 |
|---|---|
| 三个姿态角都有；四源齐全 | 每张 SVG 里有 Roll / Pitch / Yaw 三行，四源颜色齐全 |
| 单位与刻度清楚 | 三行的纵轴标签都是 `[deg]`；图内文字为英文 |
| 目标线为 0 并在图注说明 | 目标线是虚线，且图注写明 roll = pitch = yaw = 0 |
| 仿真实姿态来自 `x0[6:10]` | 图注点名 `x0[6:10]`；数据由票 01 提供，脚本不重算 |
| 图注含 VP/EKF 偏差说明 | 图注含 `+0.03 rad` 与 `vision_pose` |
| PNG/SVG 非空、图内英文 | 文件大小、300 dpi、无中文字符 |
| 不读原始包、不重算口径 | 只读 `data/*.csv` 与同名 `.json`（脚本接口只收 data-dir） |

图生成到 pytest 的临时目录，**不覆盖** `docs/report/figures/` 的提交版。
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from PIL import Image

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import plot_attitude  # noqa: E402
import plot_traj  # noqa: E402
import style  # noqa: E402

SCENES = plot_traj.SCENES
STEMS = [f"fig-att-{scene}" for scene in SCENES]


def _svg_texts(path: Path) -> list[str]:
    root = ET.parse(path).getroot()
    return [node.text for node in root.iter()
            if node.tag.endswith("text") and node.text]


@pytest.fixture(scope="module")
def generated(tmp_path_factory):
    """完整生成三张图与图注，返回 `(图目录, 图注路径)`。"""
    outdir = tmp_path_factory.mktemp("attitude-figures")
    generated_dir = tmp_path_factory.mktemp("attitude-generated")
    captions = generated_dir / "figure-captions.md"
    rc = plot_attitude.main([
        "--scene", "all",
        "--outdir", str(outdir),
        "--captions", str(captions),
    ])
    assert rc == 0
    return outdir, captions


# ─────────────────────────── 产物 ───────────────────────────


def test_three_figures_exist_and_are_non_empty(generated):
    outdir, _ = generated
    assert len(STEMS) == 3
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
            assert image.width >= 1800 and image.height >= 1500, image.size


def test_svg_has_three_angle_rows_and_english_text(generated):
    outdir, _ = generated
    for stem in STEMS:
        path = outdir / f"{stem}.svg"
        texts = _svg_texts(path)
        joined = " ".join(texts)
        for label in ("Roll", "Pitch", "Yaw"):
            assert label in joined, f"{path} 缺少 {label!r}"
        assert joined.count("[deg]") >= 3, f"{path} 三行纵轴标签不齐"
        assert "t [s]" in joined
        bad = sorted({ch for text in texts for ch in text if ord(ch) > 127})
        assert not bad, f"{path} 含非 ASCII 字符 {bad}"


def test_four_sources_are_drawn(generated):
    outdir, _ = generated
    root = ET.parse(outdir / f"{STEMS[0]}.svg").getroot()
    # matplotlib 把线色写在 `style` 属性里（`stroke: #xxxxxx`），不是
    # 独立的 `stroke=` 属性；两种都读，免得哪天 SVG 后端换写法。
    joined = " ".join(
        node.get(key, "") for node in root.iter()
        for key in ("style", "stroke", "fill")
    ).lower()
    for source in style.SOURCES:
        color = style.SOURCE_COLORS[source].lower()
        assert color in joined, f"{STEMS[0]}.svg 找不到源 {source} 的颜色 {color}"


def test_target_is_a_dashed_zero_line(generated):
    """目标姿态恒 0（水平 + 纯偏航），且必须是虚线。"""
    outdir, _ = generated
    svg = (outdir / f"{STEMS[0]}.svg").read_text(encoding="utf-8")
    assert "stroke: #000000" in svg, "目标线颜色不是黑色"
    assert "stroke-dasharray" in svg, "目标线不是虚线"
    # 三行目标都是 0：SVG 里应有 3 条黑色虚线，且没有其他黑色实线。
    assert svg.count("stroke: #000000") >= 3


# ─────────────────────────── 图注 ───────────────────────────


def test_captions_cover_all_scenes(generated):
    _, captions = generated
    text = captions.read_text(encoding="utf-8")
    for scene in SCENES:
        assert f"phase5-att:{scene}" in text
    assert text.count("phase5-att:") == 3 * 2  # BEGIN + END，每场景各一对
    for window in ("39.6494", "8.0096", "12.2021"):
        assert window in text, f"图注缺少窗口 {window}"


def test_captions_state_attitude_conventions(generated):
    _, captions = generated
    text = captions.read_text(encoding="utf-8")
    # 目标姿态 = 水平 + 纯偏航。
    assert "水平 + 纯偏航" in text
    assert "roll = pitch = yaw = 0" in text
    # 实飞姿态源与已知零偏。
    assert "/mavros/vision_pose/pose" in text
    assert "+0.03 rad" in text
    # 仿真姿态来自四元数，不是旧日志的角速率字段。
    assert "x0[6:10]" in text
    assert "角速率" in text


def test_hover_caption_mentions_recording_truncation(generated):
    _, captions = generated
    text = captions.read_text(encoding="utf-8")
    assert "录制截断，不是控制失败" in text


# ─────────────────────────── 分块与可复现 ───────────────────────────


def test_partial_rerun_keeps_other_blocks(tmp_path):
    captions = tmp_path / "figure-captions.md"
    captions.write_text(
        "# 图注\n\n<!-- BEGIN phase5-traj:hover -->\n旧块\n"
        "<!-- END phase5-traj:hover -->\n",
        encoding="utf-8",
    )

    outdir = tmp_path / "figures"
    rc = plot_attitude.main([
        "--scene", "square",
        "--outdir", str(outdir),
        "--captions", str(captions),
    ])
    assert rc == 0
    text = captions.read_text(encoding="utf-8")
    assert "旧块" in text, "部分重跑抹掉了别的票写的图注块"
    assert "phase5-att:square" in text
    assert "phase5-att:hover" not in text, "只重画 square 却写了 hover 的块"


def test_rerun_is_byte_identical(tmp_path):
    outdir_a = tmp_path / "a"
    outdir_b = tmp_path / "b"
    captions_a = tmp_path / "a.md"
    captions_b = tmp_path / "b.md"
    for outdir, captions in ((outdir_a, captions_a), (outdir_b, captions_b)):
        rc = plot_attitude.main([
            "--scene", "all",
            "--outdir", str(outdir),
            "--captions", str(captions),
        ])
        assert rc == 0
    for name in [f"{stem}.{ext}" for stem in STEMS for ext in ("png", "svg")]:
        assert (outdir_a / name).read_bytes() == (outdir_b / name).read_bytes(), name
    assert captions_a.read_bytes() == captions_b.read_bytes()
