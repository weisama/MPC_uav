#!/usr/bin/env python3
"""检查装配后的 ``docs/report/report.md``，并生成 ``REVIEW.md``。

票 16 的检查清单落在这里，能自动判的就自动判；内容质量仍由操作者
最终审查。检查器只读报告与 ``generated/``，不改任何章节。

用法::

    python3 tools/report/check_report.py
    python3 tools/report/check_report.py docs/report/report.md --no-review

退出码：0 = 没有失败项；1 = 至少一个失败项（警告不改变退出码）。
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
REPORT_DIR = REPO_ROOT / "docs" / "report"
DEFAULT_REPORT = REPORT_DIR / "report.md"
DEFAULT_REVIEW = REPORT_DIR / "REVIEW.md"
GENERATED_METRICS = REPORT_DIR / "generated" / "metrics.md"

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_links import check_file, github_slug  # noqa: E402

FENCE_RE = re.compile(r"^\s{0,3}(```+|~~~+)")
ATX_RE = re.compile(r"^(#{1,6})\s+(.*?)\s*$")
ANCHOR_RE = re.compile(r'<a\s+(?:name|id)\s*=\s*["\']([^"\']+)["\']')
BLOCK_RE = re.compile(
    r"<!-- BEGIN (phase5-[^ >]+) -->\n(.*?)\n<!-- END \1 -->", re.DOTALL
)
SUMMARY_MARK = "一句话总结"
SUMMARY_RE = re.compile(r"^<big>\*\*一句话总结：(.+?)\*\*</big>\s*$")
TOC_ENTRY_RE = re.compile(r"^(\s*)-\s+\[(.*?)\]\(#([^)]+)\)\s*$")
CHAPTER_RE = re.compile(r"^第\s*(\d+)\s*章\b")
SECTION_NUM_RE = re.compile(r"^(\d+(?:\.\d+)*)\s")


@dataclass
class Issue:
    severity: str  # "fail" | "warn"
    code: str
    detail: str


@dataclass
class Inventory:
    chapters: list[dict] = field(default_factory=list)
    figures: list[tuple[int, str]] = field(default_factory=list)
    tables: list[tuple[int, str]] = field(default_factory=list)


# ---------------------------------------------------------------------------
# 解析


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def iter_headings(text: str) -> list[tuple[int, int, str]]:
    """返回 ``(行号, 级别, 标题)``；跳过围栏代码。"""
    out: list[tuple[int, int, str]] = []
    in_fence = False
    marker = ""
    for no, line in enumerate(text.splitlines(), start=1):
        m = FENCE_RE.match(line)
        if m:
            fence = m.group(1)
            if not in_fence:
                in_fence, marker = True, fence[0] * 3
            elif fence.startswith(marker):
                in_fence = False
            continue
        if in_fence:
            continue
        hm = ATX_RE.match(line)
        if hm:
            # 装配器给标题包了着色用的 <span>；检查口径按纯文本算
            title = re.sub(r"</?span[^>]*>", "", hm.group(2)).strip()
            out.append((no, len(hm.group(1)), title))
    return out


def heading_anchor_map(text: str) -> dict[int, str]:
    """标题行号 → 紧邻它前面的显式锚点。"""
    lines = text.splitlines()
    anchors: dict[int, str] = {}
    last_anchor = None
    for no, line in enumerate(lines, start=1):
        am = ANCHOR_RE.search(line)
        if am:
            last_anchor = am.group(1)
            continue
        if ATX_RE.match(line) and last_anchor:
            anchors[no] = last_anchor
            last_anchor = None
        elif line.strip():
            # 锚点与标题之间只允许空行；出现别的内容就作废
            if not line.strip().startswith("<a "):
                last_anchor = None
    return anchors


def chapter_ranges(
    headings: list[tuple[int, int, str]], total_lines: int
) -> list[dict]:
    """把 H1 章节切开，返回每章的标题、起止行与子标题。"""
    h1 = [(no, title) for no, level, title in headings if level == 1]
    ranges: list[dict] = []
    for i, (start, title) in enumerate(h1):
        end = h1[i + 1][0] if i + 1 < len(h1) else total_lines + 1
        subs = [
            (no, level, t)
            for no, level, t in headings
            if start < no < end and level >= 2
        ]
        ranges.append({"start": start, "end": end, "title": title, "subs": subs})
    return ranges


def section_text(lines: list[str], start: int, end: int) -> str:
    return "\n".join(lines[start:end - 1])


def strip_inline_code(line: str) -> str:
    return re.sub(r"`[^`]*`", lambda m: " " * len(m.group(0)), line)


def strip_link_targets(line: str) -> str:
    return re.sub(r"\]\([^)]*\)", "](target)", line)


def chinese_len(s: str) -> int:
    return len(re.sub(r"\s", "", s))


def cjk_count(s: str) -> int:
    return len(re.findall(r"[一-鿿]", s))


# ---------------------------------------------------------------------------
# 各项检查


def check_chapters(text: str, headings, issues: list[Issue]) -> None:
    h1 = [(no, t) for no, level, t in headings if level == 1]
    if len(h1) < 12:
        issues.append(
            Issue("fail", "chapters", f"H1 数量 {len(h1)}，应有文档标题 + 第 0–10 章共 12 个")
        )
    nums = []
    for _, title in h1:
        m = CHAPTER_RE.match(title)
        if m:
            nums.append(int(m.group(1)))
    expected = list(range(0, 11))
    if sorted(nums) != expected:
        missing = sorted(set(expected) - set(nums))
        extra = sorted(set(nums) - set(expected))
        issues.append(
            Issue(
                "fail",
                "chapters",
                f"章号不连续：缺 {missing or '无'}，多 {extra or '无'}（实际 {sorted(nums)}）",
            )
        )


def check_toc(text: str, headings, issues: list[Issue]) -> None:
    lines = text.splitlines()
    toc_start = None
    for no, level, title in headings:
        if level == 1 and re.match(r"^第\s*1\s*章\b", title):
            toc_start = no
            break
    if toc_start is None:
        issues.append(Issue("fail", "toc", "找不到第 1 章目录"))
        return
    next_h1 = next(
        (no for no, level, _ in headings if level == 1 and no > toc_start), None
    )
    end = next_h1 or len(lines) + 1
    anchors = set(ANCHOR_RE.findall(text))
    entries: list[tuple[str, str]] = []
    for line in lines[toc_start - 1 : end - 1]:
        m = TOC_ENTRY_RE.match(line)
        if m:
            entries.append((m.group(2), m.group(3)))
    if not entries:
        issues.append(Issue("fail", "toc", "目录没有任何条目"))
        return
    for label, anchor in entries:
        if anchor not in anchors:
            issues.append(Issue("fail", "toc", f"目录条目锚点不存在: [{label}](#{anchor})"))
    # 每个标题（第 0、2–10 章，1–3 级）都应在目录里
    amap = heading_anchor_map(text)
    toc_anchors = {a for _, a in entries}
    for no, level, title in headings:
        if level > 3:
            continue
        if no == toc_start or no == 1:
            continue
        if level == 1 and not CHAPTER_RE.match(title):
            continue  # 文档标题不进目录
        anchor = amap.get(no)
        if anchor and anchor not in toc_anchors:
            issues.append(
                Issue("fail", "toc", f"标题未进目录（行 {no}）: {title} -> #{anchor}")
            )
    if any(SUMMARY_MARK in label for label, _ in entries):
        issues.append(Issue("fail", "toc", "目录里出现了一句话总结"))


def check_links(report: Path, issues: list[Issue]) -> None:
    problems = check_file(report, {})
    for lineno, target, reason in problems:
        issues.append(Issue("fail", "links", f"行 {lineno}: {target}（{reason}）"))
    text = read(report)
    for no, line in enumerate(text.splitlines(), start=1):
        if re.search(r"!?\[[^\]]*\]\((?:~|/)[^)]*\)", line):
            issues.append(
                Issue("fail", "links", f"行 {no}: 链接指向仓库外绝对路径: {line.strip()}")
            )


def collect_numbering(
    text: str, inventory: Inventory, issues: list[Issue]
) -> tuple[set[int], set[int]]:
    lines = text.splitlines()
    fig_caption: dict[int, str] = {}
    fig_alt: set[int] = set()
    tab_caption: dict[int, str] = {}
    in_fence = False
    marker = ""
    for no, line in enumerate(lines, start=1):
        m = FENCE_RE.match(line)
        if m:
            fence = m.group(1)
            if not in_fence:
                in_fence, marker = True, fence[0] * 3
            elif fence.startswith(marker):
                in_fence = False
            continue
        if in_fence:
            continue
        fm = re.match(r"^!\[图\s*(\d+)(?![\d-])", line)
        if fm:
            fig_alt.add(int(fm.group(1)))
            continue
        fm = re.match(r"^\*\*图\s*(\d+)(?![\d-])[　\s]*(.*?)\*\*", line)
        if fm:
            fig_caption[int(fm.group(1))] = fm.group(2)
            inventory.figures.append((int(fm.group(1)), fm.group(2)))
            continue
        tm = re.match(r"^\*\*表\s*(\d+)(?![\d-])[　\s]*(.*?)\*\*", line)
        if tm:
            tab_caption[int(tm.group(1))] = tm.group(2)
            inventory.tables.append((int(tm.group(1)), tm.group(2)))

    for kind, numbers in (("图", fig_caption), ("表", tab_caption)):
        if not numbers:
            continue
        got = sorted(numbers)
        if got != list(range(1, max(got) + 1)):
            issues.append(
                Issue("fail", "numbering", f"{kind}题编号不连续或重复：{got}")
            )
    if set(fig_caption) != fig_alt:
        issues.append(
            Issue(
                "fail",
                "numbering",
                f"图片 alt 与图题编号不一致：alt={sorted(fig_alt)}，题={sorted(fig_caption)}",
            )
        )
    # 正文引用必须落在现有编号里；行首的图题/表题本身不算引用
    ref_re = re.compile(r"(?<![0-9A-Za-z])([图表])\s*(\d+)(?![\d-])")
    for no, line in enumerate(lines, start=1):
        if re.match(r"^!\[图\s*\d+", line) or re.match(r"^\*\*[图表]\s*\d+", line):
            continue
        for m in ref_re.finditer(strip_inline_code(line)):
            kind, num = m.group(1), int(m.group(2))
            pool = fig_caption if kind == "图" else tab_caption
            if pool and num not in pool:
                issues.append(
                    Issue("fail", "numbering", f"行 {no}: 引用了不存在的 {kind} {num}")
                )
    for no, line in enumerate(lines, start=1):
        if re.search(r"[图表]\s*\d+-\d+", strip_inline_code(line)):
            issues.append(
                Issue("fail", "numbering", f"行 {no}: 仍残留章内编号: {line.strip()[:60]}")
            )
    return set(fig_caption), set(tab_caption)


def check_placeholders(text: str, issues: list[Issue]) -> None:
    patterns = [
        (r"\bTODO\b", "TODO"),
        (r"\bTBD\b", "TBD"),
        (r"\bFIXME\b", "FIXME"),
        (r"\bXXX\b", "XXX"),
        (r"待补", "待补"),
        (r"待填", "待填"),
        (r"未替换", "未替换"),
        (r"占位符", "占位符"),
        (r"\{\{[^}]*\}\}", "{{...}} 模板占位"),
        (r"<!--\s*(?:TODO|FIXME|TBD)", "HTML 注释占位"),
    ]
    for no, line in enumerate(text.splitlines(), start=1):
        for pat, label in patterns:
            if re.search(pat, line, flags=re.IGNORECASE):
                issues.append(
                    Issue("fail", "placeholder", f"行 {no}: 出现 {label}: {line.strip()[:80]}")
                )


def check_summaries(
    text: str, headings, ranges: list[dict], issues: list[Issue]
) -> None:
    lines = text.splitlines()
    all_summary_lines: list[int] = []
    for no, line in enumerate(lines, start=1):
        if SUMMARY_MARK in line:
            all_summary_lines.append(no)

    # 1) 每章：叶子小节数 vs 总结行数
    for r in ranges:
        title = r["title"]
        if not CHAPTER_RE.match(title):
            continue  # 文档标题不是章节，不参与统计
        if re.match(r"^第\s*1\s*章\b", title):
            continue
        subs = r["subs"]
        leaves: list[tuple[int, str]] = []
        for i, (no, level, t) in enumerate(subs):
            next_level = None
            for no2, level2, _ in subs[i + 1 :]:
                next_level = level2
                break
            if next_level is None or next_level <= level:
                leaves.append((no, t))
        expected = len(leaves)
        actual_lines = [
            no
            for no in all_summary_lines
            if r["start"] < no < r["end"]
        ]
        if expected != len(actual_lines):
            # 找出缺总结的叶子
            missing = []
            for no, t in leaves:
                # 该叶子到下一个标题之间有没有总结行
                later = [n for n, _, _ in subs if n > no]
                stop = min(later) if later else r["end"]
                if not any(no < n < stop for n in actual_lines):
                    missing.append(f"{t}（行 {no}）")
            detail = f"{title}: 叶子 {expected} 个，总结 {len(actual_lines)} 行"
            if missing:
                detail += "；缺总结的叶子: " + "、".join(missing)
            issues.append(Issue("fail", "summary-count", detail))
        if not leaves and not actual_lines:
            issues.append(
                Issue(
                    "warn",
                    "summary-count",
                    f"{title}: 无子节也无总结句（票面等式 0=0 通过；spec 的"
                    "“无子标题则在章末给一句”未满足，交操作者裁定）",
                )
            )
        elif leaves and actual_lines:
            extra = len(actual_lines) - expected
            if extra > 0:
                issues.append(
                    Issue("fail", "summary-count", f"{title}: 多出 {extra} 条总结")
                )

    # 2) 每行格式与写作规则
    banned = ("效果良好", "基本通过", "本节介绍")
    for no in all_summary_lines:
        line = lines[no - 1].strip()
        # 装配器在总结内容外着色 <span>；格式检查按纯文本算
        plain = re.sub(r"</?span[^>]*>", "", line)
        m = SUMMARY_RE.match(plain)
        if not m:
            issues.append(
                Issue(
                    "fail",
                    "summary-format",
                    f"行 {no}: 不是单行 <big>**一句话总结：……**</big>: {line[:80]}",
                )
            )
            continue
        content = m.group(1)
        if content.count("。") > 1:
            issues.append(Issue("fail", "summary-format", f"行 {no}: 一句话总结里有多个句号"))
        if re.search(r"[图表]\s*\d+", content):
            issues.append(Issue("fail", "summary-format", f"行 {no}: 总结引用了图号/表号"))
        for word in banned:
            if word in content:
                issues.append(
                    Issue("fail", "summary-format", f"行 {no}: 总结含“{word}”")
                )
        n = chinese_len(content)
        if n > 60:
            issues.append(
                Issue("warn", "summary-format", f"行 {no}: 总结 {n} 字，超过 60 字上限")
            )
        # 数只能来自本节正文
        r = next((x for x in ranges if x["start"] < no < x["end"]), None)
        if r is not None:
            # 本节 = 该总结前最近的标题到下一个标题
            prev = [h for h in headings if h[0] < no]
            start = prev[-1][0] if prev else r["start"]
            nxt = [h for h in headings if h[0] > no]
            end = nxt[0][0] if nxt else r["end"]
            section = "\n".join(lines[start - 1 : end - 1])
            for token in re.findall(r"\d+(?:\.\d+)?", content):
                if token not in section:
                    issues.append(
                        Issue(
                            "warn",
                            "summary-number",
                            f"行 {no}: 总结里的数 {token} 在本节正文中找不到",
                        )
                    )

    if any("&lt;big&gt;" in line or "&lt;big&gt;" in line for line in lines):
        issues.append(
            Issue("fail", "summary-format", "一句话总结的 <big> 被转义成 &lt;big&gt;")
        )


def check_terminology(text: str, issues: list[Issue]) -> None:
    """正文统一写 MPC；中文全称只允许出现在行内代码里（如日志原文）。"""
    in_fence = False
    marker = ""
    for no, line in enumerate(text.splitlines(), start=1):
        fm = FENCE_RE.match(line)
        if fm:
            fence = fm.group(1)
            if not in_fence:
                in_fence, marker = True, fence[0] * 3
            elif fence.startswith(marker):
                in_fence = False
            continue
        if in_fence:
            continue
        # 目录条目是标题的副本，术语问题按标题报一次即可
        if re.match(r"^\s*-\s+\[.*\]\(#", line):
            continue
        clean = strip_link_targets(strip_inline_code(line))
        clean = re.sub(r"<[^>]+>", "", clean)
        for m in re.finditer("模型预测控制", clean):
            issues.append(
                Issue(
                    "fail",
                    "terminology",
                    f"行 {no}: 正文出现“模型预测控制”，应统一写 MPC: "
                    f"…{clean[max(0, m.start() - 20):m.end() + 20]}…",
                )
            )


def check_generated_tables(text: str, issues: list[Issue]) -> None:
    if not GENERATED_METRICS.is_file():
        issues.append(Issue("fail", "generated", "缺少 generated/metrics.md"))
        return
    generated = {
        m.group(1): m.group(2).strip("\n") for m in BLOCK_RE.finditer(read(GENERATED_METRICS))
    }
    report_blocks = {
        m.group(1): m.group(2).strip("\n") for m in BLOCK_RE.finditer(text)
    }
    for key, body in generated.items():
        if key not in report_blocks:
            issues.append(Issue("fail", "generated", f"报告缺少结果块 {key}"))
        elif report_blocks[key] != body:
            issues.append(
                Issue("fail", "generated", f"结果块 {key} 与 generated/metrics.md 不一致")
            )
    # 第 10 章里所有表格都必须落在 phase5-metrics 块内
    lines = text.splitlines()
    in_ch10 = False
    in_block = False
    for no, line in enumerate(lines, start=1):
        if ATX_RE.match(line) and line.startswith("# "):
            in_ch10 = bool(re.match(r"^#\s+第\s*10\s*章", line))
        if in_ch10 and re.match(r"^<!-- BEGIN phase5-metrics:", line):
            in_block = True
        if in_ch10 and re.match(r"^<!-- END phase5-metrics:", line):
            in_block = False
        if in_ch10 and line.lstrip().startswith("|") and not in_block:
            issues.append(
                Issue(
                    "fail",
                    "generated",
                    f"第 10 章行 {no}: 表格不在 phase5-metrics 块内，可能是手抄数",
                )
            )


def check_candidates(text: str, issues: list[Issue]) -> None:
    for no, line in enumerate(text.splitlines(), start=1):
        if re.search(r"candidates/|fig-traj-[a-z]+-3d", line):
            issues.append(
                Issue("fail", "candidates", f"行 {no}: 正文引用了 3D 候选图: {line.strip()[:80]}")
            )


def check_cross_refs(text: str, headings, issues: list[Issue]) -> None:
    chapter_nums = {
        int(m.group(1)) for _, _, t in headings if (m := CHAPTER_RE.match(t))
    }
    section_nums = set()
    for _, level, title in headings:
        m = SECTION_NUM_RE.match(title)
        if m:
            section_nums.add(m.group(1))
    lines = text.splitlines()
    for no, line in enumerate(lines, start=1):
        clean = strip_inline_code(line)
        for m in re.finditer(r"第\s*(\d+)\s*章", clean):
            n = int(m.group(1))
            if n not in chapter_nums:
                issues.append(
                    Issue("fail", "cross-ref", f"行 {no}: 引用了不存在的第 {n} 章")
                )
        # 显式小节引用（见/按/§/第 X.Y 节）
        candidates = set()
        for pat in (
            r"(?:见|按|参见|详见)\s*(\d+(?:\.\d+)+)",
            r"§\s*(\d+(?:\.\d+)+)",
            r"第\s*(\d+(?:\.\d+)+)\s*节",
        ):
            candidates.update(re.findall(pat, clean))
        # 只认带“见/按/§/第 X 节”的显式小节引用。裸的 X.Y.Z 太容易撞
        # 版本号（PX4 1.14.3、L4T R36.5.0）与 IP，不作为内部引用判据。
        for token in candidates:
            if token in section_nums:
                continue
            try:
                first = int(token.split(".")[0])
            except ValueError:
                continue
            if first > 10:
                continue  # 22.2 V、39.64 s 之类，不是章号
            # 同一行提到仓库内 .md 时，§X.Y 视为那份外部文档的小节
            if ".md" in line and (f"§{token}" in line or f"{token} 节" in line):
                continue
            # 行内带有指向仓库文档的链接时，视为外部文档小节，不判
            if re.search(r"\]\([^)]*\.md(?:#[^)]*)?\)", line) and not re.search(
                r"\]\(#[^)]*\)", line
            ):
                continue
            issues.append(
                Issue("fail", "cross-ref", f"行 {no}: 引用了不存在的报告小节 {token}")
            )


def check_duplicates(headings, issues: list[Issue]) -> None:
    seen: dict[str, int] = {}
    for no, level, title in headings:
        if level > 3:
            continue
        key = re.sub(r"\s+", " ", title).strip()
        if key in seen:
            issues.append(
                Issue(
                    "fail",
                    "headings",
                    f"重复标题: “{key}”（行 {seen[key]} 与 {no}）",
                )
            )
        else:
            seen[key] = no
    prev_level = 0
    for no, level, title in headings:
        if prev_level and level > prev_level + 1:
            issues.append(
                Issue("warn", "headings", f"行 {no}: 标题层级从 H{prev_level} 跳到 H{level}")
            )
        prev_level = level


# ---------------------------------------------------------------------------
# 报告


def build_inventory(text: str, headings) -> Inventory:
    inv = Inventory()
    ranges = chapter_ranges(headings, len(text.splitlines()))
    lines = text.splitlines()
    for r in ranges:
        title = r["title"]
        if re.match(r"^第\s*1\s*章", title):
            continue
        if not CHAPTER_RE.match(title):
            continue
        body = "\n".join(lines[r["start"] - 1 : r["end"] - 1])
        inv.chapters.append(
            {
                "title": title,
                "lines": r["end"] - r["start"],
                "cjk": cjk_count(body),
                "chars": len(re.sub(r"\s", "", body)),
            }
        )
    # 图/表清单由 collect_numbering 填
    return inv


def write_review(
    path: Path,
    report_path: Path,
    text: str,
    inv: Inventory,
    issues: list[Issue],
    headings,
) -> None:
    fails = [i for i in issues if i.severity == "fail"]
    warns = [i for i in issues if i.severity == "warn"]
    out: list[str] = [
        "# 报告装配审查记录",
        "",
        f"> 由 `tools/report/check_report.py` 从 `{report_path.relative_to(REPO_ROOT)}` 生成。",
        "> 最终审查结论由操作者填写；本文件不代替人工审查。",
        "",
        "## 章节清单",
        "",
        "| 章 | 标题 | 行数 | 汉字数 | 非空白字符 |",
        "|---|---|---:|---:|---:|",
    ]
    for c in inv.chapters:
        m = re.match(r"^第\s*(\d+)\s*章", c["title"])
        label = f"第 {m.group(1)} 章" if m else c["title"][:12]
        out.append(
            f"| {label} | {c['title']} | {c['lines']} | {c['cjk']} | {c['chars']} |"
        )
    out += [
        "",
        "## 图题清单",
        "",
        "| 编号 | 图题 |",
        "|---:|---|",
    ]
    for no, caption in sorted(inv.figures):
        out.append(f"| {no} | {caption} |")
    out += [
        "",
        "## 表题清单",
        "",
        "| 编号 | 表题 |",
        "|---:|---|",
    ]
    for no, caption in sorted(inv.tables):
        out.append(f"| {no} | {caption} |")
    out += [
        "",
        "## 装配检查结果",
        "",
        f"- 失败项：{len(fails)}",
        f"- 警告项：{len(warns)}",
        "",
    ]
    if fails:
        out.append("### 失败")
        out.append("")
        for i in fails:
            out.append(f"- `{i.code}` {i.detail}")
        out.append("")
    if warns:
        out.append("### 警告")
        out.append("")
        for i in warns:
            out.append(f"- `{i.code}` {i.detail}")
        out.append("")
    if not fails and not warns:
        out.append("自动检查全部通过。")
        out.append("")

    out += [
        "## 未确定项",
        "",
    ]
    if fails:
        out.append("以下项在自动检查中失败，按票 16 的规则应退回对应章节票或由操作者裁定：")
        out.append("")
        for i in fails:
            out.append(f"- `{i.code}` {i.detail}")
    else:
        out.append("自动检查没有失败项；内容与口径的未确定项见第 10 章 10.7。")
    out += [
        "",
        "## 最终审查记录",
        "",
        "- 状态：**待操作者最终审查**（本装配稿不自行把票 16 或 spec 翻 `done`）",
        "- 审查意见：",
        "",
    ]
    path.write_text("\n".join(out) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="检查 docs/report/report.md")
    parser.add_argument("report", nargs="?", default=str(DEFAULT_REPORT))
    parser.add_argument("--no-review", action="store_true", help="不写 REVIEW.md")
    parser.add_argument("--review", default=str(DEFAULT_REVIEW), help="REVIEW.md 路径")
    args = parser.parse_args(argv)

    report = Path(args.report)
    if not report.is_file():
        print(f"找不到报告: {report}", file=sys.stderr)
        return 1
    text = read(report)
    headings = iter_headings(text)
    issues: list[Issue] = []
    inv = build_inventory(text, headings)

    check_chapters(text, headings, issues)
    check_toc(text, headings, issues)
    check_links(report, issues)
    fig_nums, tab_nums = collect_numbering(text, inv, issues)
    check_placeholders(text, issues)
    check_summaries(text, headings, chapter_ranges(headings, len(text.splitlines())), issues)
    check_terminology(text, issues)
    check_generated_tables(text, issues)
    check_candidates(text, issues)
    check_cross_refs(text, headings, issues)
    check_duplicates(headings, issues)

    fails = [i for i in issues if i.severity == "fail"]
    warns = [i for i in issues if i.severity == "warn"]
    for i in fails:
        print(f"FAIL [{i.code}] {i.detail}")
    for i in warns:
        print(f"WARN [{i.code}] {i.detail}")
    print(
        f"\n检查完成：{len(fails)} 失败，{len(warns)} 警告；"
        f"图 {len(fig_nums)} 张，表 {len(tab_nums)} 张。"
    )
    if not args.no_review:
        write_review(Path(args.review), report, text, inv, issues, headings)
        print(f"已写入 {Path(args.review).relative_to(REPO_ROOT)}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
