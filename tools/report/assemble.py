#!/usr/bin/env python3
"""把 ``docs/report/chapters/*.md`` 装配成 ``docs/report/report.md``。

装配只做**格式、链接、编号**级修订，不改章节正文：

- 按 0、2–10 的顺序合并章节；第 1 章目录由本脚本生成，没有源文件；
- 第 10 章的 ``phase5-metrics:*`` 结果表从 ``generated/metrics.md`` 内联；
- 正文引用的图，其图注从 ``generated/figure-captions.md`` 内联；
- 图题/表题统一改成全局连续编号（图 1、图 2…；表 1、表 2…）；
- 相对链接按 ``report.md`` 所在目录重写；
- 标题前插入显式锚点，目录链接只指向这些锚点，避免不同渲染器的
  GitHub slug 差异。

用法::

    python3 tools/report/assemble.py            # 写 docs/report/report.md
    python3 tools/report/assemble.py --check    # 装配后自动跑 check_report.py

退出码：0 = 已生成；1 = 有装配警告（仍生成，便于人工核对）。
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
REPORT_DIR = REPO_ROOT / "docs" / "report"
CHAPTER_DIR = REPORT_DIR / "chapters"
GENERATED_DIR = REPORT_DIR / "generated"
DEFAULT_OUT = REPORT_DIR / "report.md"

# 第 1 章目录由脚本生成，所以源文件里没有 01-*.md。
CHAPTER_FILES = [
    "00-summary.md",
    "02-background.md",
    "03-hardware.md",
    "04-references.md",
    "05-architecture.md",
    "06-code-structure.md",
    "07-launch-and-rc.md",
    "08-mpc-derivation.md",
    "09-experiment-design.md",
    "10-results.md",
]

FENCE_RE = re.compile(r"^\s{0,3}(```+|~~~+)")
ATX_RE = re.compile(r"^(#{1,6})\s+(.*?)\s*$")
BLOCK_RE = re.compile(
    r"<!-- BEGIN (phase5-[^ >]+) -->\n(.*?)\n<!-- END \1 -->", re.DOTALL
)
IMAGE_RE = re.compile(r"^!\[([^\]]*)\]\(([^)]+)\)\s*$")
LINK_RE = re.compile(r"(!?\[[^\]]*\]\()([^)]+)(\))")
EXTERNAL_SCHEMES = (
    "http://",
    "https://",
    "mailto:",
    "ftp://",
    "ftps://",
    "tel:",
    "data:",
    "file:",
    "ssh://",
    "git@",
)


# ---------------------------------------------------------------------------
# 基础读取与代码块处理


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def split_fences(text: str):
    """逐行返回 ``(line, in_fence)``；围栏标记行本身算围栏内。"""
    in_fence = False
    marker = ""
    for line in text.splitlines():
        m = FENCE_RE.match(line)
        if m:
            fence = m.group(1)
            if not in_fence:
                in_fence = True
                marker = fence[0] * 3
            elif fence.startswith(marker):
                in_fence = False
            yield line, True
            continue
        yield line, in_fence


def map_outside_fences(text: str, fn) -> str:
    """对围栏外的行调用 ``fn(line)``，围栏内原样保留。"""
    out: list[str] = []
    for line, in_fence in split_fences(text):
        out.append(line if in_fence else fn(line))
    # 保持原文末尾换行
    result = "\n".join(out)
    if text.endswith("\n"):
        result += "\n"
    return result


def map_outside_code(line: str, fn) -> str:
    """对行内代码之外的片段调用 ``fn``。"""
    parts = re.split(r"(`[^`]*`)", line)
    for i in range(0, len(parts), 2):
        parts[i] = fn(parts[i])
    return "".join(parts)


# ---------------------------------------------------------------------------
# generated 块


def parse_blocks(text: str) -> dict[str, str]:
    return {m.group(1): m.group(2).strip("\n") for m in BLOCK_RE.finditer(text)}


def inline_metrics(chapter: dict[str, str], warnings: list[str]) -> None:
    """把第 10 章的 phase5-metrics:* 块替换成 generated/metrics.md 的内容。"""
    generated = parse_blocks(read(GENERATED_DIR / "metrics.md"))
    text = chapter["10"]
    used: set[str] = set()

    def repl(m: re.Match[str]) -> str:
        key = m.group(1)
        if key not in generated:
            warnings.append(f"generated/metrics.md 缺少块 {key}，保留章节内原文")
            return m.group(0)
        used.add(key)
        return f"<!-- BEGIN {key} -->\n{generated[key]}\n<!-- END {key} -->"

    chapter["10"] = BLOCK_RE.sub(repl, text)
    for key in sorted(set(generated) - used):
        warnings.append(f"generated/metrics.md 的块 {key} 没有被第 10 章引用")


def figure_blocks() -> dict[str, dict[str, str]]:
    """把 generated/figure-captions.md 的块按图片文件名索引。"""
    blocks = parse_blocks(read(GENERATED_DIR / "figure-captions.md"))
    by_file: dict[str, dict[str, str]] = {}
    for key, body in blocks.items():
        for name in re.findall(r"`figures/([^`]+?\.(?:png|svg))`", body):
            by_file.setdefault(name, {"key": key, "body": body})
    return by_file


def inline_figure_captions(chapter: dict[str, str], warnings: list[str]) -> None:
    """把图片后面那段手写图注替换成 generated 图注。

    生成块里的 ``### T1 …`` 只是块标签，不进正文；真正的图题由图片 alt
    里的全局图号 + 生成块的标题拼成。
    """
    by_file = figure_blocks()
    used: set[str] = set()
    lines = chapter["10"].splitlines()
    out: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        m = IMAGE_RE.match(line)
        if not m:
            out.append(line)
            i += 1
            continue
        out.append(line)
        out.append("")
        alt, target = m.group(1), m.group(2)
        name = Path(target).name
        block = by_file.get(name)
        if block is None:
            warnings.append(f"图片 {name} 没有 generated 图注，保留手写图注")
            i += 1
            continue
        if block["key"] in used:
            warnings.append(f"图片 {name} 的图注块 {block['key']} 被重复使用")
            i += 1
            continue
        used.add(block["key"])
        # 跳过图片后的空行，找到现有图注段落（到下一个空行为止）
        j = i + 1
        while j < len(lines) and not lines[j].strip():
            j += 1
        k = j
        while k < len(lines) and lines[k].strip():
            k += 1
        if j >= len(lines):
            warnings.append(f"图片 {name} 后面没有图注段落，无法内联")
            out.extend(lines[i + 1 :])
            break
        fig_no = None
        fm = re.search(r"图\s*(\d+)", alt)
        if fm:
            fig_no = fm.group(1)
        body_lines = block["body"].splitlines()
        title = ""
        if body_lines and body_lines[0].startswith("### "):
            title = body_lines[0][4:].strip()
            title = re.sub(r"^[A-Z]\d+\s+", "", title)
            body_lines = body_lines[1:]
        while body_lines and not body_lines[0].strip():
            body_lines.pop(0)
        if fig_no and title:
            out.append(f"**图 {fig_no}　{title.rstrip('。')}。**")
            out.append("")
        out.extend(body_lines)
        out.append("")
        i = k
    chapter["10"] = "\n".join(out) + ("\n" if chapter["10"].endswith("\n") else "")
    unused = sorted(set(by_file) - {n for n, b in by_file.items() if b["key"] in used})
    # 未嵌入正文的图（9 张 pitch/roll）有图注块是正常的，只提示不报错。
    if unused:
        warnings.append(
            "figure-captions.md 中未内联的图注块（对应图未嵌入正文）："
            + ", ".join(sorted({by_file[n]["key"] for n in unused}))
        )


# ---------------------------------------------------------------------------
# 图题 / 表题全局编号


def build_numbering(chapters: dict[str, str]) -> tuple[dict, dict, list[str]]:
    """按文档顺序给图题、表题分配全局编号。

    返回 ``(figure_map, table_map, warnings)``，键是 ``(章号, 章内序号)``。
    """
    figure_map: dict[tuple[int, int], int] = {}
    table_map: dict[tuple[int, int], int] = {}
    warnings: list[str] = []
    fig_no = tab_no = 0
    for name in CHAPTER_FILES:
        chapter_no = int(name[:2])
        text = chapters[name[:2]]
        for line, in_fence in split_fences(text):
            if in_fence:
                continue
            fm = re.match(r"^!\[图\s*(\d+)-(\d+)", line) or re.match(
                r"^\*\*图\s*(\d+)-(\d+)", line
            )
            if fm:
                key = (int(fm.group(1)), int(fm.group(2)))
                if key not in figure_map:
                    fig_no += 1
                    figure_map[key] = fig_no
                continue
            tm = re.match(r"^\*\*表\s*(\d+)-(\d+)", line)
            if tm:
                key = (int(tm.group(1)), int(tm.group(2)))
                if key not in table_map:
                    tab_no += 1
                    table_map[key] = tab_no
    for key in figure_map:
        if key[0] != 10:
            warnings.append(f"图 {key[0]}-{key[1]} 不在第 10 章，仍按全局编号处理")
    return figure_map, table_map, warnings


def renumber(chapters: dict[str, str], figure_map, table_map, warnings) -> None:
    """把正文与图注里的 ``图 X-Y`` / ``表 X-Y`` 换成全局编号。"""
    token_re = re.compile(r"(?<![0-9A-Za-z])([图表])\s*(\d+)-(\d+)")

    def repl_for(line: str) -> str:
        def repl(m: re.Match[str]) -> str:
            kind, ch, idx = m.group(1), int(m.group(2)), int(m.group(3))
            mapping = figure_map if kind == "图" else table_map
            if (ch, idx) not in mapping:
                warnings.append(f"找不到 {kind} {ch}-{idx} 的图题/表题，保留原编号")
                return m.group(0)
            return f"{kind} {mapping[(ch, idx)]}"

        return map_outside_code(line, lambda segment: token_re.sub(repl, segment))

    for name in CHAPTER_FILES:
        chapters[name[:2]] = map_outside_fences(chapters[name[:2]], repl_for)


# ---------------------------------------------------------------------------
# 链接重写


def _split_target(raw: str) -> tuple[str, str]:
    raw = raw.strip()
    if raw.startswith("<") and raw.endswith(">"):
        raw = raw[1:-1].strip()
    if " " in raw:
        candidate = raw.split(" ", 1)[0]
        if raw[len(candidate) :].lstrip().startswith(('"', "'", "(")):
            raw = candidate
    if "#" in raw:
        path, frag = raw.split("#", 1)
        return path, frag
    return raw, ""


def rewrite_links(text: str, src_dir: Path, warnings: list[str]) -> str:
    """把相对链接改成从 ``report.md`` 出发可解析的路径。"""

    def rewrite_target(raw: str) -> str:
        target = raw.strip()
        if not target or target.startswith(EXTERNAL_SCHEMES) or target.startswith("#"):
            return raw
        path_part, frag = _split_target(target)
        if not path_part:
            return raw
        if path_part.startswith(("~", "/")):
            warnings.append(f"链接指向仓库外绝对路径，未重写: {target}")
            return raw
        trailing_slash = path_part.endswith("/")
        resolved = (src_dir / path_part).resolve()
        # 章与章之间的链接指向装配后的章节锚点，而不是章节源文件
        if resolved.parent == CHAPTER_DIR and resolved.suffix == ".md":
            m = re.match(r"^(\d{2})-", resolved.name)
            if m and not frag:
                return f"#ch{int(m.group(1))}"
            if m:
                warnings.append(f"章节链接带锚点，无法映射到装配锚点，保留文件链接: {target}")
        rel = os.path.relpath(str(resolved), str(REPORT_DIR))
        rel = Path(rel).as_posix()
        if trailing_slash and not rel.endswith("/"):
            rel += "/"
        return rel + (f"#{frag}" if frag else "")

    def per_line(line: str) -> str:
        return LINK_RE.sub(
            lambda m: m.group(1) + rewrite_target(m.group(2)) + m.group(3), line
        )

    return map_outside_fences(text, per_line)


# ---------------------------------------------------------------------------
# 锚点与目录


def slugify(text: str) -> str:
    """与 check_links.github_slug 同口径的近似 slug，用于无编号标题。"""
    s = re.sub(r"<[^>]+>", "", text)
    s = re.sub(r"`([^`]*)`", r"\1", s)
    s = re.sub(r"[*_~]", "", s).lower()
    s = re.sub(r"[^\w\s\-]", "", s, flags=re.UNICODE)
    return s.replace(" ", "-")


def insert_anchors(
    text: str, chapter_no: int, used: set[str], warnings: list[str]
) -> tuple[str, list[tuple[int, str, str]]]:
    """在标题前插入显式锚点，返回 ``(新文本, [(级别, 标题, 锚点)])``。"""
    lines = text.splitlines()
    out: list[str] = []
    entries: list[tuple[int, str, str]] = []
    in_fence = False
    marker = ""
    for line in lines:
        m = FENCE_RE.match(line)
        if m:
            fence = m.group(1)
            if not in_fence:
                in_fence, marker = True, fence[0] * 3
            elif fence.startswith(marker):
                in_fence = False
            out.append(line)
            continue
        if not in_fence:
            hm = ATX_RE.match(line)
            if hm:
                level = len(hm.group(1))
                title = hm.group(2)
                num = re.match(r"^(\d+(?:\.\d+)*)\s", title)
                if level == 1 and chapter_no is not None:
                    anchor = f"ch{chapter_no}"
                elif num:
                    anchor = "sec-" + num.group(1).replace(".", "-")
                else:
                    anchor = f"sec-{chapter_no}-{slugify(title)}"
                base = anchor
                n = 2
                while anchor in used:
                    anchor = f"{base}-{n}"
                    n += 1
                used.add(anchor)
                if anchor != base:
                    warnings.append(f"锚点重名，已改为 {anchor}（原标题：{title}）")
                out.append(f'<a id="{anchor}"></a>')
                out.append("")
                entries.append((level, title, anchor))
        out.append(line)
    result = "\n".join(out)
    if text.endswith("\n"):
        result += "\n"
    return result, entries


DOC_TOKEN_RE = re.compile(r"(`[^`]*`|\[[^\]]*\]\([^)]*\))")
SEC_REF_RE = re.compile(r"§\s*(\d+\.\d+(?:\.\d+)*)")
SEE_REF_RE = re.compile(r"(见|按|参见|详见|回到|留给)(\s*)(\d+\.\d+(?:\.\d+)*)(?=\s*节?)")
EXTERNAL_REF_HINTS = ("路线图", "演练页", "参考实现", "手册", "页首", "清单")


def link_internal_refs(chapters: dict[str, str], anchors: set[str]) -> None:
    """把指向本报告小节的 §X.Y / 见 X.Y 转成锚点链接。

    只处理能对上 ``sec-X-Y`` 锚点的编号；外部文档的 ``§``（如
    ``mocap-setup.md`` 后面的 §9）保持原样。
    """

    def anchor_for(num: str) -> str | None:
        key = "sec-" + num.replace(".", "-")
        return key if key in anchors else None

    def external_context(segment: str, pos: int, external_seen: bool) -> bool:
        before = segment[max(0, pos - 20) : pos]
        if external_seen:
            return True
        return any(hint in before for hint in EXTERNAL_REF_HINTS)

    def repl_segment(segment: str, external_seen: bool) -> str:
        def sec_repl(m: re.Match[str]) -> str:
            num = m.group(1)
            if external_context(segment, m.start(), external_seen):
                return m.group(0)
            anchor = anchor_for(num)
            return f"[§{num}](#{anchor})" if anchor else m.group(0)

        segment = SEC_REF_RE.sub(sec_repl, segment)

        def see_repl(m: re.Match[str]) -> str:
            num = m.group(3)
            lead = segment[max(0, m.start() - 20) : m.start()]
            if any(hint in lead for hint in EXTERNAL_REF_HINTS):
                return m.group(0)
            anchor = anchor_for(num)
            if not anchor:
                return m.group(0)
            return f"{m.group(1)}{m.group(2)}[{num}](#{anchor})"

        return SEE_REF_RE.sub(see_repl, segment)

    for name in CHAPTER_FILES:
        out: list[str] = []
        for line, in_fence in split_fences(chapters[name[:2]]):
            if in_fence or line.startswith("#"):
                out.append(line)
                continue
            parts = DOC_TOKEN_RE.split(line)
            # 同一行里先提到外部文档，后面的 §X.Y 默认算外部文档的节号
            external_seen = False
            rebuilt: list[str] = []
            for i, part in enumerate(parts):
                if i % 2 == 1:
                    if part.startswith("["):
                        target = part.split("](", 1)[1].rsplit(")", 1)[0]
                        if not target.startswith("#") and not target.startswith(
                            ("http://", "https://", "mailto:")
                        ):
                            external_seen = True
                    elif ".md" in part or ".yaml" in part:
                        external_seen = True
                    rebuilt.append(part)
                    continue
                rebuilt.append(repl_segment(part, external_seen))
            out.append("".join(rebuilt))
        text = "\n".join(out)
        if chapters[name[:2]].endswith("\n"):
            text += "\n"
        chapters[name[:2]] = text


HEADING_COLOR = "#0B5394"
SUMMARY_COLOR = "#C00000"
SUMMARY_COLOR_RE = re.compile(
    r"^(\s*)(<big>\*\*)(一句话总结：)(.*?)(\*\*</big>\s*)$"
)


def colorize(text: str) -> str:
    """给标题与一句话总结加颜色，便于和正文区分。

    只加在装配产物里，章节源文件保持纯 markdown；标题文字整体包进
    ``<span>``，检查脚本解析标题前会剥掉标签。
    """

    def per_line(line: str) -> str:
        sm = SUMMARY_COLOR_RE.match(line)
        if sm:
            return (
                f'{sm.group(1)}{sm.group(2)}'
                f'<span style="color:{SUMMARY_COLOR}">{sm.group(3)}{sm.group(4)}</span>'
                f"{sm.group(5)}"
            )
        hm = ATX_RE.match(line)
        if hm and "<span" not in hm.group(2):
            return (
                f'{hm.group(1)} <span style="color:{HEADING_COLOR}">'
                f"{hm.group(2)}</span>"
            )
        return line

    return map_outside_fences(text, per_line)


def render_toc(entries: list[tuple[int, str, str]]) -> str:
    """按标题级别渲染嵌套目录；只收标题，不收一句话总结。"""
    lines = [
        "# 第 1 章 目录",
        "",
        "> 本目录由 `tools/report/assemble.py` 从正文标题生成；链接指向标题前的显式锚点。",
        "",
    ]
    for level, title, anchor in entries:
        if level > 3:
            continue
        indent = "  " * (level - 1)
        safe = title.replace("]", "\\]")
        lines.append(f"{indent}- [{safe}](#{anchor})")
    lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 主流程


def discover_chapters() -> dict[str, str]:
    chapters: dict[str, str] = {}
    for name in CHAPTER_FILES:
        path = CHAPTER_DIR / name
        if not path.is_file():
            raise SystemExit(f"缺少章节源文件: {path}")
        chapters[name[:2]] = read(path)
    return chapters


def assemble() -> tuple[str, list[str]]:
    warnings: list[str] = []
    chapters = discover_chapters()

    inline_metrics(chapters, warnings)

    # 先统一编号，再内联图注：图注标题里的图号取自已编号的图片 alt。
    figure_map, table_map, numbering_warnings = build_numbering(chapters)
    warnings.extend(numbering_warnings)
    renumber(chapters, figure_map, table_map, warnings)

    inline_figure_captions(chapters, warnings)

    used_anchors: set[str] = set()
    toc_entries: list[tuple[int, str, str]] = []
    for name in CHAPTER_FILES:
        chapter_no = int(name[:2])
        text = chapters[name[:2]]
        text = rewrite_links(text, CHAPTER_DIR, warnings)
        text, entries = insert_anchors(text, chapter_no, used_anchors, warnings)
        chapters[name[:2]] = text
        toc_entries.extend(entries)

    link_internal_refs(chapters, used_anchors)
    toc = render_toc(toc_entries)

    header = (
        "# z0 无人机自主飞行项目报告\n\n"
        "> 本文件由 `tools/report/assemble.py` 从 `docs/report/chapters/` 装配生成；"
        "不要手改，改章节源文件后重跑。\n\n"
        "---\n\n"
    )
    body_parts: list[str] = []
    # 第 0 章
    body_parts.append(chapters["00"].rstrip())
    body_parts.append(toc.rstrip())
    for name in CHAPTER_FILES[1:]:
        body_parts.append(chapters[name[:2]].rstrip())
    report = header + "\n\n---\n\n".join(body_parts) + "\n"
    return colorize(report), warnings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="装配 docs/report/report.md")
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="输出文件")
    parser.add_argument(
        "--check", action="store_true", help="装配后运行 check_report.py"
    )
    args = parser.parse_args(argv)

    report, warnings = assemble()
    out = Path(args.out)
    out.write_text(report, encoding="utf-8")
    print(f"已写入 {out.relative_to(REPO_ROOT)}（{len(report.splitlines())} 行）")
    for w in warnings:
        print(f"warning: {w}", file=sys.stderr)

    if args.check:
        rc = subprocess.call(
            [sys.executable, str(Path(__file__).with_name("check_report.py")), str(out)]
        )
        return rc
    return 1 if warnings else 0


if __name__ == "__main__":
    sys.exit(main())
