#!/usr/bin/env python3
"""检查 markdown 文档内的相对链接与锚点。

无第三方依赖；只做"本地可解析性"检查，不访问网络。

用法::

    python3 tools/report/check_links.py                 # 扫默认范围
    python3 tools/report/check_links.py README.md docs  # 扫指定文件/目录

默认范围：README.md、operation_manual.md、run/README.md、docs/**/*.md、
src/*/README.md。跳过在制品与归档产物（docs/report/chapters、
docs/assets/papers、docs/assets/drill、.pytest_cache）。

检查项：
- 相对路径的目标文件/目录是否存在；
- 目标为 .md 时锚点是否存在（GitHub 风格 slug，支持中文标题与重名 -N 后缀）；
- 纯锚点 ``#foo`` 在本文件内是否存在；
- 图片链接同样检查。

退出码：0 = 全部可解析；1 = 有断链。
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_TARGETS = ["README.md", "operation_manual.md", "run/README.md", "docs", "src"]

# 不在本工具检查范围内的目录（在制品 / 归档原始产物 / 缓存）
EXCLUDE_DIR_PARTS = {
    (".git",),
    (".pytest_cache",),
    ("docs", "report", "chapters"),
    ("docs", "assets", "papers"),
    ("docs", "assets", "drill"),
}

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

# [text](target "title") / ![alt](target)
INLINE_LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
# [id]: target "title"
REF_DEF_RE = re.compile(r"^\s{0,3}\[[^\]]+\]:\s*(\S+)(?:\s+[\"'(].*)?$")
# fenced code block marker
FENCE_RE = re.compile(r"^\s{0,3}(```+|~~~+)")
# ATX heading
ATX_HEADING_RE = re.compile(r"^\s{0,3}(#{1,6})\s+(.*?)\s*#*\s*$")
# Setext heading underline
SETEXT_RE = re.compile(r"^\s{0,3}(=+|-+)\s*$")
# HTML anchor: <a name="x"> / <a id="x"> / id="x"
HTML_ANCHOR_RE = re.compile(r"""(?:<a\s+(?:name|id)\s*=\s*["']([^"']+)["'])|(?:<[^>]+\sid\s*=\s*["']([^"']+)["'])""")


def iter_markdown_files(targets: list[str]) -> list[Path]:
    files: list[Path] = []
    for raw in targets:
        p = (REPO_ROOT / raw).resolve() if not Path(raw).is_absolute() else Path(raw)
        if p.is_dir():
            for dirpath, dirnames, filenames in os.walk(p):
                dirnames[:] = [d for d in sorted(dirnames) if d != ".git"]
                rel = Path(dirpath).resolve().relative_to(REPO_ROOT)
                if any(_matches_prefix(rel.parts, ex) for ex in EXCLUDE_DIR_PARTS):
                    dirnames[:] = []
                    continue
                for name in sorted(filenames):
                    if name.endswith(".md"):
                        files.append(Path(dirpath) / name)
        elif p.is_file():
            if p.suffix == ".md":
                files.append(p)
        else:
            print(f"warning: 目标不存在，已跳过: {raw}", file=sys.stderr)
    # 去重并保持稳定顺序
    seen: set[Path] = set()
    out: list[Path] = []
    for f in files:
        r = f.resolve()
        if r not in seen:
            seen.add(r)
            out.append(r)
    return out


def _matches_prefix(parts: tuple[str, ...], prefix: tuple[str, ...]) -> bool:
    return len(parts) >= len(prefix) and tuple(parts[: len(prefix)]) == prefix


def strip_code(text: str) -> str:
    """把围栏代码块与行内代码替换成等长空行/空格，保留行号。"""
    lines = text.splitlines()
    out: list[str] = []
    in_fence = False
    fence_marker = ""
    for line in lines:
        m = FENCE_RE.match(line)
        if m:
            marker = m.group(1)
            if not in_fence:
                in_fence = True
                fence_marker = marker[0] * 3
                out.append("")
                continue
            if marker.startswith(fence_marker):
                in_fence = False
                out.append("")
                continue
        if in_fence:
            out.append("")
        else:
            # 去掉行内代码，避免把示例里的链接当真
            out.append(re.sub(r"`[^`]*`", lambda mm: " " * len(mm.group(0)), line))
    return "\n".join(out)


def github_slug(heading: str) -> str:
    """GitHub 风格标题锚点（够用的近似实现）。

    规则：去掉 markdown 行内标记与 HTML 标签；转小写；保留字母/数字/下划线/
    连字符/空格与 CJK；空格转连字符。
    """
    s = heading.strip()
    s = re.sub(r"<[^>]+>", "", s)          # HTML 标签
    s = re.sub(r"!\[[^\]]*\]\([^)]*\)", "", s)  # 图片
    s = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", s)  # 链接保留文字
    s = re.sub(r"`([^`]*)`", r"\1", s)     # 行内代码
    s = re.sub(r"[*_~]", "", s)            # 强调标记
    s = s.lower()
    # 只保留字母（含 CJK）、数字、下划线、连字符、空格
    s = re.sub(r"[^\w\s\-]", "", s, flags=re.UNICODE)
    s = s.replace(" ", "-")
    return s


def collect_anchors(path: Path) -> set[str]:
    text = path.read_text(encoding="utf-8", errors="replace")
    clean = strip_code(text)
    anchors: set[str] = set()
    counts: dict[str, int] = {}
    lines = clean.splitlines()

    for i, line in enumerate(lines):
        m = ATX_HEADING_RE.match(line)
        if m:
            slug = github_slug(m.group(2))
            _add_slug(anchors, counts, slug)
            continue
        # Setext: 上一行文字 + 下一行 === / ---
        if SETEXT_RE.match(line) and i > 0:
            prev = lines[i - 1].strip()
            if prev and not ATX_HEADING_RE.match(prev) and not SETEXT_RE.match(prev):
                _add_slug(anchors, counts, github_slug(prev))

    for m in HTML_ANCHOR_RE.finditer(text):
        anchor = m.group(1) or m.group(2)
        if anchor:
            anchors.add(anchor)
    return anchors


def _add_slug(anchors: set[str], counts: dict[str, int], slug: str) -> None:
    if not slug:
        return
    n = counts.get(slug, 0)
    anchors.add(slug if n == 0 else f"{slug}-{n}")
    counts[slug] = n + 1


def split_target(raw: str) -> tuple[str, str]:
    raw = raw.strip()
    if raw.startswith("<") and raw.endswith(">"):
        raw = raw[1:-1].strip()
    # 去掉 markdown title：target "title" / target 'title'
    if " " in raw:
        candidate = raw.split(" ", 1)[0]
        # 仅当 title 部分像引号包裹时采用；路径里通常不会有空格（若有需用 <>）
        if raw[len(candidate):].lstrip().startswith(("\"", "'", "(")):
            raw = candidate
    if "#" in raw:
        path_part, fragment = raw.split("#", 1)
    else:
        path_part, fragment = raw, ""
    return path_part, fragment


def check_file(path: Path, anchor_cache: dict[Path, set[str]]) -> list[tuple[int, str, str]]:
    """返回 (行号, 目标, 原因)。"""
    problems: list[tuple[int, str, str]] = []
    raw_text = path.read_text(encoding="utf-8", errors="replace")
    clean = strip_code(raw_text)
    lines = clean.splitlines()

    targets: list[tuple[int, str]] = []
    # 行内链接可能跨行（链接文字折行），按整段文本扫描再用偏移换算行号
    for m in INLINE_LINK_RE.finditer(clean):
        lineno = clean.count("\n", 0, m.start()) + 1
        targets.append((lineno, m.group(1)))
    # 引用式定义 ``[id]: path`` 只能独占一行
    for lineno, line in enumerate(lines, start=1):
        m = REF_DEF_RE.match(line)
        if m:
            targets.append((lineno, m.group(1)))

    if targets:
        for lineno, raw_target in targets:
            target = raw_target.strip()
            if not target:
                continue
            if target.startswith(EXTERNAL_SCHEMES):
                continue
            path_part, fragment = split_target(target)
            # 纯锚点
            if not path_part:
                if fragment and fragment not in _anchors_for(path, anchor_cache):
                    problems.append((lineno, target, f"锚点不存在: #{fragment}"))
                continue

            base = path.parent
            resolved = (base / path_part).resolve()
            # 目录链接也可能带末尾斜杠
            if not resolved.exists():
                problems.append((lineno, target, f"目标不存在: {_rel(resolved)}"))
                continue
            if fragment and resolved.suffix.lower() == ".md":
                if resolved.is_file() and fragment not in _anchors_for(resolved, anchor_cache):
                    problems.append((lineno, target, f"锚点不存在: {_rel(resolved)}#{fragment}"))
    return problems


def _anchors_for(path: Path, cache: dict[Path, set[str]]) -> set[str]:
    r = path.resolve()
    if r not in cache:
        try:
            cache[r] = collect_anchors(r)
        except OSError:
            cache[r] = set()
    return cache[r]


def _rel(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="检查 markdown 相对链接与锚点")
    parser.add_argument("targets", nargs="*", help="文件或目录（默认扫仓库活文档）")
    parser.add_argument("-q", "--quiet", action="store_true", help="只输出断链")
    args = parser.parse_args(argv)

    targets = args.targets or DEFAULT_TARGETS
    files = iter_markdown_files(targets)
    if not files:
        print("没有找到可检查的 markdown 文件", file=sys.stderr)
        return 1

    anchor_cache: dict[Path, set[str]] = {}
    total = 0
    broken = 0
    for f in files:
        total += 1
        problems = check_file(f, anchor_cache)
        if problems and not args.quiet:
            print(f"\n{_rel(f)}")
        for lineno, target, reason in problems:
            broken += 1
            print(f"  {_rel(f)}:{lineno}: {target}  ({reason})")

    print(f"\n检查了 {total} 个文件，断链 {broken} 处。")
    return 0 if broken == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
