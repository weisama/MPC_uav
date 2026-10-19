#!/usr/bin/env python3
"""把装配产物 `docs/report/report.md` 导出为 PDF。

用 pandoc + XeLaTeX 排版：公式由 LaTeX 直接渲染，图片统一压到版心内
（第 10 章的大图不会再被裁掉），表题/图题与正文样式分开。
只读 `report.md`，不改它。

用法：
    python3 tools/report/export_pdf.py
    python3 tools/report/export_pdf.py --input docs/report/report.md \
        --output docs/report/report.pdf
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

HEADER_TEX = r"""
\usepackage{xcolor}
\definecolor{MPCblue}{HTML}{0B5394}
\definecolor{MPCred}{HTML}{C00000}
\usepackage{titlesec}
\titleformat{\section}[block]
  {\clearpage\LARGE\bfseries\color{MPCblue}}{}{0em}{}
\titleformat{\subsection}[block]
  {\large\bfseries\color{MPCblue}}{}{0em}{}
\titleformat{\subsubsection}[block]
  {\normalsize\bfseries\color{MPCblue}}{}{0em}{}
\usepackage{graphicx}
% 全篇图片统一缩进版心：宽度不超行宽、高度不超版心高度的 0.72，
% 保持长宽比。第 10 章的大图靠这一条避免被裁切。
\setkeys{Gin}{width=\linewidth,height=0.72\textheight,keepaspectratio}
\usepackage{longtable,booktabs,array}
\renewcommand{\arraystretch}{1.18}
\setlength{\LTpre}{6pt}
\setlength{\LTpost}{6pt}
\usepackage{etoolbox}
\AtBeginEnvironment{longtable}{\footnotesize}
\usepackage{caption}
% 图题里已经手写了「图 N」，不要让 LaTeX 再自动加 "Figure N:"。
\captionsetup{labelformat=empty}
\usepackage{fancyhdr}
\pagestyle{fancy}
\fancyhf{}
\fancyfoot[C]{\thepage}
\renewcommand{\headrulewidth}{0pt}
\newenvironment{summary}{\par\color{MPCred}\bfseries}{\par}
\usepackage{ragged2e}
\setlength{\emergencystretch}{3em}
"""

LUA_FILTER = r"""
-- 把「一句话总结」段落染成红色，与 report.md 的装配样式一致。
function Para(el)
  -- 图片单独成段：居中；图题是后面单独一段，不能再让 alt 文本变成 LaTeX 图题。
  if #el.content == 1 and el.content[1].t == "Image" then
    return {
      pandoc.RawBlock("latex", "\\begin{center}"),
      el,
      pandoc.RawBlock("latex", "\\end{center}")
    }
  end
  local text = pandoc.utils.stringify(el)
  if text:sub(1, #"一句话总结：") == "一句话总结：" then
    return {
      pandoc.RawBlock("latex", "\\begin{summary}"),
      el,
      pandoc.RawBlock("latex", "\\end{summary}")
    }
  end
  return el
end

-- 表格按各列内容长度分宽度，强制 pandoc 输出 minipage 列；
-- 否则宽表会用 l 列直接冲出右边距（第 8/10 章的指标表就是这样被裁掉的）。
function Table(el)
  local n = #el.aligns
  if n == 0 then return el end
  local maxlen = {}
  for i = 1, n do maxlen[i] = 1 end
  local function scan(cells)
    for i, cell in ipairs(cells) do
      local s = pandoc.utils.stringify(cell)
      local len = utf8.len(s) or #s
      if len > maxlen[i] then maxlen[i] = len end
    end
  end
  scan(el.headers)
  for _, row in ipairs(el.rows) do scan(row) end
  local total = 0
  for i = 1, n do
    maxlen[i] = math.min(math.max(maxlen[i], 8), 36)
    total = total + maxlen[i]
  end
  local widths = {}
  for i = 1, n do widths[i] = 0.96 * maxlen[i] / total end
  el.widths = widths
  return el
end
"""

ANCHOR_RE = re.compile(r'^\s*<a id="([^"]+)"></a>\s*$')
HEADING_RE = re.compile(r"^(#{1,6})\s+(.*)$")


def hoist_anchors(text: str) -> str:
    """把标题前的 `<a id="..."></a>` 变成 pandoc 的 `{#id}` 标题属性。

    LaTeX 输出会丢掉裸 HTML 锚点；不改写的话，目录与 `§X.Y` 链接会全部失效。
    """
    out: list[str] = []
    pending: list[str] = []
    for line in text.splitlines():
        m = ANCHOR_RE.match(line)
        if m:
            pending.append(m.group(1))
            continue
        h = HEADING_RE.match(line)
        if pending and h and line.rstrip().endswith("</span>"):
            # 标题文本外面包着着色用 <span>；属性必须加在整行末尾。
            line = f"{line} {{#{pending[-1]}}}"
            pending.clear()
        elif pending and h:
            line = f"{line} {{#{pending[-1]}}}"
            pending.clear()
        out.append(line)
    return "\n".join(out) + ("\n" if text.endswith("\n") else "")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path,
                        default=Path("docs/report/report.md"))
    parser.add_argument("--output", type=Path,
                        default=Path("docs/report/report.pdf"))
    parser.add_argument("--keep-tex", action="store_true",
                        help="保留中间 .tex 与预处理后的 .md（默认删除）")
    args = parser.parse_args()

    src = args.input.resolve()
    if not src.exists():
        print(f"输入不存在：{src}", file=sys.stderr)
        return 1
    pandoc = shutil.which("pandoc")
    xelatex = shutil.which("xelatex")
    if not pandoc or not xelatex:
        print("需要 pandoc 与 xelatex；请先安装 pandoc、texlive-xetex。",
              file=sys.stderr)
        return 1

    out = args.output.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    prepared = hoist_anchors(src.read_text(encoding="utf-8"))
    tmpdir = Path(tempfile.mkdtemp(prefix="report-pdf-"))
    # 图片路径是相对 report.md 的；在临时目录里放一个 figures 链接，
    # 让 xelatex 从临时目录编译时也能找到图。
    figures = src.parent / "figures"
    if figures.is_dir():
        (tmpdir / "figures").symlink_to(figures, target_is_directory=True)
    md_path = tmpdir / "report.md"
    header_path = tmpdir / "header.tex"
    filter_path = tmpdir / "summary.lua"
    tex_path = tmpdir / "report.tex"
    md_path.write_text(prepared, encoding="utf-8")
    header_path.write_text(HEADER_TEX, encoding="utf-8")
    filter_path.write_text(LUA_FILTER, encoding="utf-8")

    cmd = [
        pandoc, str(md_path),
        "--from", "markdown+raw_html+pipe_tables-implicit_figures",
        "--to", "latex",
        "--output", str(tex_path),
        "--standalone",
        "--pdf-engine", xelatex,
        "--columns", "100",
        "--include-in-header", str(header_path),
        "--lua-filter", str(filter_path),
        "--variable", "documentclass=article",
        "--variable", "classoption=a4paper",
        "--variable", "fontsize=11pt",
        "--variable", "geometry:margin=20mm",
        "--variable", "mainfont=Noto Serif CJK SC",
        "--variable", "sansfont=Noto Sans CJK SC",
        "--variable", "monofont=DejaVu Sans Mono",
        "--variable", "CJKmainfont=Noto Serif CJK SC",
        "--variable", "CJKsansfont=Noto Sans CJK SC",
        "--variable", "CJKmonofont=Noto Sans Mono CJK SC",
        "--variable", "colorlinks=true",
        "--variable", "linkcolor=MPCblue",
        "--variable", "urlcolor=MPCblue",
    ]
    # pandoc 只写 .tex；再用 xelatex 编译，才能把图片路径按 report.md 所在目录解析。
    result = subprocess.run(cmd, cwd=tmpdir, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if result.returncode != 0:
        print(result.stdout, file=sys.stderr)
        return result.returncode

    for _ in range(2):  # 第二遍让目录/交叉引用稳定
        result = subprocess.run(
            [xelatex, "-interaction=nonstopmode", "-halt-on-error",
             str(tex_path)],
            cwd=tmpdir, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        if result.returncode != 0:
            print(result.stdout[-4000:], file=sys.stderr)
            return result.returncode

    produced = tex_path.with_suffix(".pdf")
    if not produced.exists():
        print("xelatex 没有生成 PDF", file=sys.stderr)
        return 1
    produced.replace(out)

    if args.keep_tex:
        md_path.replace(out.with_suffix(".export.md"))
        tex_path.replace(out.with_suffix(".tex"))
        header_path.replace(out.with_suffix(".header.tex"))
        filter_path.replace(out.with_suffix(".lua"))

    print(f"已生成 {out}（{out.stat().st_size / 1024:.0f} KiB）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
