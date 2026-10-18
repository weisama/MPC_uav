#!/usr/bin/env python3
"""图注文件的分块写入（报告图脚本共用）。

一份图可能由多张票、多个脚本生成。若每个脚本都整文件覆盖，后跑的会把先跑的
图注抹掉。这里约定：每个脚本只替换自己那几块

    <!-- BEGIN <prefix>:<key> -->
    ...
    <!-- END <prefix>:<key> -->

块外的内容（别的票写的块、手工说明）原样保留；部分重跑（比如只重画一个场景）
也不会动其他 key 的块。
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict

#: 正文图注文件的表头。
CAPTION_HEADER = """# 图注（自动生成）

> 本文件由报告图脚本生成，**不要手改**。脚本只替换对应的
> `<!-- BEGIN ... -->` / `<!-- END ... -->` 块，不会动别的票写的块。
"""

#: 3D 候选图注单独成文件：正文图注会被装配票内联进报告。
CANDIDATE_CAPTION_HEADER = """# 3D 候选图注（自动生成，不进正文）

> 对应 `figures/candidates/`。**本文件不进入正文**；3D 候选是否采用
> 由操作者看图后裁定（票 03 验收）。
"""


def replace_section(text: str, key: str, body: str) -> str:
    """替换（或追加）`<!-- BEGIN key --> ... <!-- END key -->` 块。"""
    begin = f"<!-- BEGIN {key} -->"
    end = f"<!-- END {key} -->"
    has_begin = begin in text
    has_end = end in text
    if has_begin != has_end:
        raise ValueError(f"图注里的 {key!r} 块只有一半标记，先手工修好再重跑")
    block = f"{begin}\n{body.rstrip()}\n{end}\n"
    if has_begin:
        before, rest = text.split(begin, 1)
        _, after = rest.split(end, 1)
        after = after.lstrip("\n")
        # `block` 自带结尾换行；块之间固定留一个空行（重跑逐字节一致）。
        return before + block + (f"\n{after}" if after else "")
    return text.rstrip() + "\n\n" + block


def update_caption_file(path: Path, header: str, prefix: str,
                        blocks: Dict[str, str]) -> None:
    """按 key 逐块替换；只动 `<prefix>:<key>` 这些块。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = path.read_text(encoding="utf-8") if path.exists() else header
    for key, body in blocks.items():
        text = replace_section(text, f"{prefix}:{key}", body)
    path.write_text(text.rstrip() + "\n", encoding="utf-8")
