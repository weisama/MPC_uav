#!/usr/bin/env python3
"""票 05：位置误差图与控制量图的**产物与口径**测试。

对应票面检查清单：

| 票面要求 | 测什么 |
|---|---|
| 误差图四源齐全，符号约定写清 | 四源颜色齐全；图注写「实际 − 目标」；误差函数符号与 CSV 逐点核对 |
| 控制量图只有两源，并说明原因 | SVG 里只有两源颜色与两条图例；图注写 offboard 为何不在图内 |
| 推力单位与换算口径可追溯 | 图注的 `t_max` 与同名 JSON 的 `u_conversion.t_max_n` 逐场景一致 |
| 图内英文标签、PNG/SVG 非空 | 300 dpi、文件大小、无中文字符 |
| 与轨迹/姿态图共用窗口与相对系 | 图注窗口与 `grid.common_end_s` 一致；横轴 `t [s]` 相对 `/traj/start` |

图生成到 pytest 的临时目录，**不覆盖** `docs/report/figures/` 的提交版。
"""

from __future__ import annotations

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import plot_control  # noqa: E402
import plot_error  # noqa: E402
import plot_traj  # noqa: E402
import style  # noqa: E402

SCENES = plot_traj.SCENES
DATA_DIR = REPO / "docs/report/data"

#: 公共窗终点，票 01 已写进 JSON；测试从 JSON 现读，不抄常数。
WINDOWS = {
    scene: json.loads((DATA_DIR / f"{scene}.json").read_text(encoding="utf-8"))
    ["grid"]["common_end_s"]
    for scene in SCENES
}


def _svg_texts(path: Path) -> list[str]:
    root = ET.parse(path).getroot()
    return [node.text for node in root.iter()
            if node.tag.endswith("text") and node.text]


def _svg_colors(path: Path) -> str:
    """matplotlib 把线色写在 `style` 属性里；`stroke`/`fill` 也一并读。"""
    root = ET.parse(path).getroot()
    return " ".join(
        node.get(key, "") for node in root.iter()
        for key in ("style", "stroke", "fill")
    ).lower()


@pytest.fixture(scope="module")
def error_generated(tmp_path_factory):
    outdir = tmp_path_factory.mktemp("error-figures")
    generated_dir = tmp_path_factory.mktemp("error-generated")
    captions = generated_dir / "figure-captions.md"
    rc = plot_error.main([
        "--scene", "all",
        "--outdir", str(outdir),
        "--captions", str(captions),
    ])
    assert rc == 0
    return outdir, captions


@pytest.fixture(scope="module")
def control_generated(tmp_path_factory):
    outdir = tmp_path_factory.mktemp("control-figures")
    generated_dir = tmp_path_factory.mktemp("control-generated")
    captions = generated_dir / "figure-captions.md"
    rc = plot_control.main([
        "--scene", "all",
        "--outdir", str(outdir),
        "--captions", str(captions),
    ])
    assert rc == 0
    return outdir, captions


# ─────────────────────────── 误差图：产物 ───────────────────────────


def test_three_error_figures_exist_and_are_non_empty(error_generated):
    outdir, _ = error_generated
    for scene in SCENES:
        for suffix in ("png", "svg"):
            path = outdir / f"fig-error-{scene}.{suffix}"
            assert path.is_file(), f"缺少 {path}"
            assert path.stat().st_size > 10_000, f"{path} 太小，像是空图"


def test_error_png_is_300_dpi(error_generated):
    outdir, _ = error_generated
    for scene in SCENES:
        path = outdir / f"fig-error-{scene}.png"
        with Image.open(path) as image:
            dpi = image.info.get("dpi")
            assert dpi is not None, f"{path} 没有 dpi 元数据"
            assert dpi[0] == pytest.approx(300, rel=1e-3), f"{path} dpi={dpi}"


def test_error_svg_has_three_components_and_english_text(error_generated):
    outdir, _ = error_generated
    for scene in SCENES:
        path = outdir / f"fig-error-{scene}.svg"
        texts = _svg_texts(path)
        joined = " ".join(texts)
        for label in ("ex", "ey", "ez"):
            assert label in joined, f"{path} 缺少 {label!r}"
        assert joined.count("[m]") >= 3, f"{path} 三行纵轴标签不齐"
        assert "t [s]" in joined
        bad = sorted({ch for text in texts for ch in text if ord(ch) > 127})
        assert not bad, f"{path} 含非 ASCII 字符 {bad}"


def test_error_draws_four_sources_and_dashed_target(error_generated):
    outdir, _ = error_generated
    for scene in SCENES:
        path = outdir / f"fig-error-{scene}.svg"
        colors = _svg_colors(path)
        for source in style.SOURCES:
            color = style.SOURCE_COLORS[source].lower()
            assert color in colors, f"{path.name} 找不到源 {source} 的颜色 {color}"
        # 目标误差恒 0：黑色虚线；三行各一条。
        svg = path.read_text(encoding="utf-8")
        assert "stroke: #000000" in svg
        assert "stroke-dasharray" in svg
        texts = _svg_texts(path)
        assert "Target" in texts


# ─────────────────────────── 误差图：符号与口径 ───────────────────────────


def test_error_is_actual_minus_target_on_real_csv():
    """符号不能反：直接拿 CSV 里的数与 `error_series` 比，并对一个已知点定号。"""
    data = plot_traj.load_scene(DATA_DIR, "square")
    err = plot_error.error_series(data, "offboard", "x")
    expected = data["offboard"]["x"] - data["target"]["x"]
    np.testing.assert_allclose(err, expected)

    # 正方形 t≈2 s 时目标 x≈1.0，而 offboard 明显滞后（x≈0.27）；
    # 若实现写成「目标 − 实际」，这里会变成 +0.7 而不是 −0.7。
    i = int(np.argmin(np.abs(data["target"]["t"] - 2.0)))
    assert data["offboard"]["x"][i] < data["target"]["x"][i]
    assert err[i] < -0.5


def test_error_captions_state_convention_and_windows(error_generated):
    _, captions = error_generated
    text = captions.read_text(encoding="utf-8")
    assert text.count("phase5-error:") == 3 * 2
    assert "实际 − 目标" in text
    for scene in SCENES:
        assert f"phase5-error:{scene}" in text
        assert f"{WINDOWS[scene]:.4f}" in text, f"图注缺少 {scene} 的窗口"
    # 目标行恒 0 必须解释清楚，否则读者会以为少画了一条轨迹。
    assert "恒 0" in text
    assert "轨迹原点相对系" in text


def test_error_hover_caption_mentions_recording_truncation(error_generated):
    _, captions = error_generated
    text = captions.read_text(encoding="utf-8")
    assert "录制截断，不是控制失败" in text


# ─────────────────────────── 控制量图：产物 ───────────────────────────


def test_three_control_figures_exist_and_are_non_empty(control_generated):
    outdir, _ = control_generated
    for scene in SCENES:
        for suffix in ("png", "svg"):
            path = outdir / f"fig-control-{scene}.{suffix}"
            assert path.is_file(), f"缺少 {path}"
            assert path.stat().st_size > 10_000, f"{path} 太小，像是空图"


def test_control_png_is_300_dpi(control_generated):
    outdir, _ = control_generated
    for scene in SCENES:
        path = outdir / f"fig-control-{scene}.png"
        with Image.open(path) as image:
            dpi = image.info.get("dpi")
            assert dpi is not None, f"{path} 没有 dpi 元数据"
            assert dpi[0] == pytest.approx(300, rel=1e-3), f"{path} dpi={dpi}"


def test_control_svg_has_four_rows_and_english_text(control_generated):
    outdir, _ = control_generated
    for scene in SCENES:
        path = outdir / f"fig-control-{scene}.svg"
        texts = _svg_texts(path)
        joined = " ".join(texts)
        for label in ("Thrust", "wx", "wy", "wz"):
            assert label in joined, f"{path} 缺少 {label!r}"
        assert "[N]" in joined
        assert joined.count("[rad/s]") >= 3, f"{path} 三轴角速率标签不齐"
        assert "t [s]" in joined
        bad = sorted({ch for text in texts for ch in text if ord(ch) > 127})
        assert not bad, f"{path} 含非 ASCII 字符 {bad}"


def test_control_draws_only_two_mpc_sources(control_generated):
    outdir, _ = control_generated
    for scene in SCENES:
        path = outdir / f"fig-control-{scene}.svg"
        colors = _svg_colors(path)
        for source in plot_control.CONTROL_SOURCES:
            color = style.SOURCE_COLORS[source].lower()
            assert color in colors, f"{path.name} 找不到源 {source} 的颜色 {color}"
        # offboard 的蓝色与目标标签都不能出现；黑色是文字色，不能按颜色判。
        assert style.SOURCE_COLORS["offboard"].lower() not in colors, \
            f"{path.name} 把 offboard 画进了控制量图"
        texts = _svg_texts(path)
        assert "Offboard flight" not in texts
        assert "Target" not in texts
        assert "MPC simulation" in texts and "MPC flight" in texts


def test_control_caption_explains_offboard_exclusion(control_generated):
    _, captions = control_generated
    text = captions.read_text(encoding="utf-8")
    assert text.count("phase5-control:") == 3 * 2
    assert "位置设定点" in text
    assert "不是同一层" in text
    # 不能只说「没画」，要写清两组在位置层才可比。
    assert "位置跟踪" in text


def test_control_caption_thrust_conversion_matches_json(control_generated):
    _, captions = control_generated
    text = captions.read_text(encoding="utf-8")
    assert "u_thrust_n = thrust × t_max" in text
    for scene in SCENES:
        meta = json.loads((DATA_DIR / f"{scene}.json").read_text(encoding="utf-8"))
        sim = meta["sources"]["mpc_sim"]["u_conversion"]["t_max_n"]
        real = meta["sources"]["mpc_real"]["u_conversion"]["t_max_n"]
        assert sim == real, f"{scene}: 仿真与实飞 t_max 不同，图注不能写常数"
        assert f"`t_max = {sim:g} N`" in text, f"{scene}: 图注的 t_max 与 JSON 不符"
    assert "零阶保持" in text
    assert "mpc-normalization.json" in text


def test_control_hover_caption_mentions_recording_truncation(control_generated):
    _, captions = control_generated
    text = captions.read_text(encoding="utf-8")
    assert "录制截断，不是控制失败" in text


def test_control_windows_match_json(control_generated):
    _, captions = control_generated
    text = captions.read_text(encoding="utf-8")
    for scene in SCENES:
        assert f"0.0000 - {WINDOWS[scene]:.4f} s" in text


# ─────────────────────────── 分块与可复现 ───────────────────────────


@pytest.mark.parametrize("module", [plot_error, plot_control])
def test_partial_rerun_keeps_other_blocks(tmp_path, module):
    prefix = module.CAPTION_PREFIX
    captions = tmp_path / "figure-captions.md"
    captions.write_text(
        "# 图注\n\n<!-- BEGIN phase5-traj:hover -->\n旧块\n"
        "<!-- END phase5-traj:hover -->\n",
        encoding="utf-8",
    )
    outdir = tmp_path / "figures"
    rc = module.main([
        "--scene", "square",
        "--outdir", str(outdir),
        "--captions", str(captions),
    ])
    assert rc == 0
    text = captions.read_text(encoding="utf-8")
    assert "旧块" in text, "部分重跑抹掉了别的票写的图注块"
    assert f"{prefix}:square" in text
    assert f"{prefix}:hover" not in text, "只重画 square 却写了 hover 的块"


@pytest.mark.parametrize(
    "module,stem",
    [(plot_error, "fig-error"), (plot_control, "fig-control")],
)
def test_rerun_is_byte_identical(tmp_path, module, stem):
    outdir_a = tmp_path / "a"
    outdir_b = tmp_path / "b"
    captions_a = tmp_path / "a.md"
    captions_b = tmp_path / "b.md"
    for outdir, captions in ((outdir_a, captions_a), (outdir_b, captions_b)):
        rc = module.main([
            "--scene", "all",
            "--outdir", str(outdir),
            "--captions", str(captions),
        ])
        assert rc == 0
    for scene in SCENES:
        for ext in ("png", "svg"):
            name = f"{stem}-{scene}.{ext}"
            assert (outdir_a / name).read_bytes() == (outdir_b / name).read_bytes(), name
    assert captions_a.read_bytes() == captions_b.read_bytes()
