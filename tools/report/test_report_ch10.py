#!/usr/bin/env python3
"""票 15：第 10 章结果表的**生成、对账与章节纪律**测试。

覆盖：

- 表由 `report_metrics.py` 从 CSV/JSON/求解摘要现算，重跑逐字节一致；
- 第 10 章里的 `phase5-metrics:*` 块与 `generated/metrics.md` 逐字节相同
  （正文不是手抄数）；
- 六个块齐全，位置/姿态读数与票 01 公布的黄金值一致；
- 12 张正文图都存在、没有引用 3D 候选；
- 每个叶子小节末尾有且只有一行 `<big>**一句话总结：……**</big>`；
- 五张 `wontfix` 票逐条点名；没有裸词 `MPC` 与“基本通过”。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import report_metrics  # noqa: E402

CHAPTER = REPO / "docs/report/chapters/10-results.md"
GENERATED = REPO / "docs/report/generated/metrics.md"
FIGURES = REPO / "docs/report/figures"

FIGURES_EXPECTED = [
    *(f"fig-traj-{scene}" for scene in report_metrics.SCENES),
    *(f"fig-att-{scene}" for scene in report_metrics.SCENES),
    *(f"fig-error-{scene}" for scene in report_metrics.SCENES),
    *(f"fig-control-{scene}" for scene in report_metrics.SCENES),
]

SUMMARY = "<big>**一句话总结："
SUMMARY_RE = re.compile(r"^<big>\*\*一句话总结：.+\*\*</big>$")


# ─────────────────────────── 工具 ───────────────────────────


def _block(text: str, key: str) -> str:
    begin = f"<!-- BEGIN phase5-metrics:{key} -->"
    end = f"<!-- END phase5-metrics:{key} -->"
    assert begin in text, f"缺少 {key} 块的 BEGIN"
    body = text.split(begin, 1)[1].split(end, 1)[0]
    return body.strip()


def _headings(lines: list[str]):
    return [(i, len(m.group(1)), m.group(2).strip())
            for i, line in enumerate(lines)
            if (m := re.match(r"^(#{2,3})\s+(.*)$", line))]


def _leaf_sections(text: str):
    """返回 `[(标题, 行列表)]`，只含没有更深层子标题的 `##`/`###`。"""
    lines = text.splitlines()
    heads = _headings(lines)
    leaves = []
    for idx, (line_no, level, title) in enumerate(heads):
        end = len(lines)
        has_child = False
        for next_line, next_level, _ in heads[idx + 1:]:
            if next_level <= level:
                end = next_line
                break
            has_child = True
        if not has_child:
            leaves.append((title, lines[line_no + 1:end]))
    return leaves


# ─────────────────────────── 生成与对账 ───────────────────────────


@pytest.fixture(scope="module")
def generated(tmp_path_factory):
    out = tmp_path_factory.mktemp("ch10") / "metrics.md"
    chapter = tmp_path_factory.mktemp("ch10-chapter") / "10-results.md"
    chapter.write_text(CHAPTER.read_text(encoding="utf-8"), encoding="utf-8")
    assert report_metrics.main(["--out", str(out), "--chapter", str(chapter)]) == 0
    return out, chapter


def test_six_blocks_exist(generated):
    out, _ = generated
    text = out.read_text(encoding="utf-8")
    for key in ("window", "position", "hover-steady", "attitude",
                "control", "solver"):
        assert f"phase5-metrics:{key}" in text


def test_chapter_blocks_equal_generated_blocks(generated):
    """正文里的表必须就是生成物，不能是手抄的另一份。

    ★ 要比**仓库里那份第 10 章**，不是比夹具重新生成过的副本：后者会被
    `report_metrics.main` 就地重写，手改的表会被悄悄抹平 ⟹ 恒绿。
    """
    out, chapter = generated
    generated_text = out.read_text(encoding="utf-8")
    for source, name in ((chapter, "重生成的副本"),
                         (CHAPTER, "仓库里的第 10 章")):
        chapter_text = source.read_text(encoding="utf-8")
        for key in ("window", "position", "hover-steady", "attitude",
                    "control", "solver"):
            assert _block(chapter_text, key) == _block(generated_text, key), \
                f"{name}的 {key} 块与生成物不一致"


def test_rerun_is_byte_identical(generated, tmp_path):
    out_a, chapter_a = generated
    out_b = tmp_path / "metrics.md"
    chapter_b = tmp_path / "10-results.md"
    chapter_b.write_text(CHAPTER.read_text(encoding="utf-8"), encoding="utf-8")
    assert report_metrics.main(["--out", str(out_b),
                                "--chapter", str(chapter_b)]) == 0
    assert out_a.read_bytes() == out_b.read_bytes()
    assert chapter_a.read_bytes() == chapter_b.read_bytes()


def test_position_golden_values(generated):
    """票 01 公布的读数必须在表里原样出现（口径没被换掉）。"""
    out, _ = generated
    table = _block(out.read_text(encoding="utf-8"), "position")
    # 悬停 offboard：水平均值 / 最大值。
    assert "0.1047" in table and "0.1610" in table
    # 正方形 mpc_real：水平均值 / 最大值。
    assert "0.2753" in table and "0.3802" in table
    # 8 字 mpc_sim：垂直均值 / 最大值。
    assert "0.0868" in table and "0.2495" in table


def test_attitude_keeps_sim_yaw_reading(generated):
    """仿真偏航约 90° 是读数，表里必须保留，不能被悄悄扣掉。"""
    out, _ = generated
    table = _block(out.read_text(encoding="utf-8"), "attitude")
    assert "悬停 | mpc_sim | 90.51" in table
    assert "正方形 | mpc_sim | 90.05" in table
    assert "8 字 | mpc_sim | 85.48" in table


def test_solver_table_reads_summary(generated):
    out, _ = generated
    table = _block(out.read_text(encoding="utf-8"), "solver")
    assert "3.624" in table and "20.138" in table
    assert table.count("| 0 |") >= 3  # 退化与饱和各 0


def test_saturation_thresholds_come_from_production_params():
    z0 = report_metrics.load_z0_params()
    assert z0.f_max == pytest.approx(11.8)
    assert z0.rate_max == pytest.approx((3.84, 3.84, 6.28))
    assert report_metrics.SAT_TOL_REL == pytest.approx(1e-3)
    # 边界判定：差 1‰ 以内算饱和，差得多不算。
    assert report_metrics.saturated_bounds((11.8, 0.0, 0.0, 0.0), z0) == ("F_max",)
    assert report_metrics.saturated_bounds((11.5, 0.0, 0.0, 0.0), z0) == ()
    assert report_metrics.saturated_bounds((7.0, 3.84, -3.84, 6.28), z0) == \
        ("wx", "wy", "wz")


# ─────────────────────────── 章节纪律 ───────────────────────────


def test_twelve_figures_are_referenced_and_exist():
    text = CHAPTER.read_text(encoding="utf-8")
    for stem in FIGURES_EXPECTED:
        assert f"figures/{stem}.png" in text, f"正文没有引用 {stem}"
        assert (FIGURES / f"{stem}.png").is_file()
        assert (FIGURES / f"{stem}.svg").is_file()
    assert len(FIGURES_EXPECTED) == 12


def test_no_3d_candidates_in_body():
    text = CHAPTER.read_text(encoding="utf-8").lower()
    assert "3d" not in text
    assert "candidates" not in text


def test_every_leaf_section_has_exactly_one_summary():
    text = CHAPTER.read_text(encoding="utf-8")
    leaves = _leaf_sections(text)
    assert len(leaves) >= 10, f"只找到 {len(leaves)} 个叶子小节"
    for title, body in leaves:
        summaries = [line for line in body if line.startswith(SUMMARY)]
        assert len(summaries) == 1, f"{title}: {len(summaries)} 行总结"
        line = summaries[0]
        assert SUMMARY_RE.match(line), f"{title}: 总结行格式不对：{line!r}"
        # `---` 是章节之间的分隔线，不算小节内容。
        non_empty = [l for l in body if l.strip() and l.strip() != "---"]
        assert non_empty[-1] == line, f"{title}: 总结不在小节末尾"
        # 一句话、≤60 字、不引图号/表号。
        content = line[len(SUMMARY):-len("**</big>")]
        assert len(content) <= 60, f"{title}: 总结 {len(content)} 字"
        assert "图 10-" not in content and "表 10-" not in content


def test_all_tables_have_captions():
    text = CHAPTER.read_text(encoding="utf-8")
    for key in ("window", "position", "hover-steady", "attitude",
                "control", "solver"):
        begin = f"<!-- BEGIN phase5-metrics:{key} -->"
        before = text.split(begin, 1)[0].splitlines()[-4:]
        assert any(line.startswith("**表 10-") for line in before), \
            f"{key} 块前没有表题"


def test_five_wontfix_tickets_are_named():
    text = CHAPTER.read_text(encoding="utf-8")
    for ticket in ("phase4/01", "phase4/04", "phase4/09",
                   "phase4/14", "phase4/16"):
        assert ticket in text, f"没有点名 {ticket}"


def test_boundary_items_present():
    text = CHAPTER.read_text(encoding="utf-8")
    for phrase in ("录制截断", "τ 的实飞 A/B", "回退能力",
                   "+0.03 rad", "反馈源不同", "Tier A 与 Tier B 的 T0",
                   "从未在 SITL 跑过"):
        assert phrase in text, f"边界一节缺少 {phrase!r}"


def test_no_bare_mpc_and_no_pass_phrase():
    text = CHAPTER.read_text(encoding="utf-8")
    assert "基本通过" not in text
    # 去掉首次出现的“模型预测控制（MPC）”，其余正文不得再有裸词 MPC。
    stripped = text.replace("模型预测控制（MPC）", "", 1)
    assert "MPC" not in stripped, "正文还有裸词 MPC"


def test_caption_lines_are_close_to_figures():
    lines = CHAPTER.read_text(encoding="utf-8").splitlines()
    for i, line in enumerate(lines):
        if line.startswith("!["):
            window = "\n".join(lines[i:i + 4])
            assert "**图 10-" in window, f"第 {i + 1} 行的图后没有图题"
