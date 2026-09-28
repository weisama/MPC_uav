#!/usr/bin/env python3
"""`px4_console` 的**离线**单测：不启 SITL，直接往 `_raw` 喂字节。

只测一件夹具上真踩过的事：**游标必须是字节口径**。
`text()` 是整段 `_raw` 的重渲染，pxh 的行内重画（`ESC[2K\\r`）可以跨两次 `read()` 到达，
于是"当时的渲染长度"这个游标会让**新打印的字符被整个跳过** ——
表现是 `wait_for("^TOPIC: …")` 永远等不到、白等满超时再重试。

跑法：`python3 tools/px4-sitl/test_px4_console.py`（或 pytest）。
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from px4_console import Px4Console  # noqa: E402


def feed(px, chunk: bytes):
    """模拟一次 `read()`：把字节接到 `_raw` 尾巴上。"""
    px._raw += chunk


def new_console():
    """不启动进程的裸对象（`_raw` 是 `bytearray`，`proc` 用不上）。"""
    px = Px4Console.__new__(Px4Console)
    px._raw = bytearray()
    px.proc = None
    return px


def test_render_strips_ansi_and_collapses_cr():
    px = new_console()
    feed(px, b"pxh> ls\r\nabc\r\x1b[2Kabcdef\r\n")
    # `\r` 只留最后一段、ANSI 被剥掉、ONLCR 多出来的空段不留下多余空行
    assert px.text() == "pxh> ls\nabcdef\n", repr(px.text())


def test_text_offset_cursor_skips_fresh_output():
    """★ 反例（**故意**把 bug 钉在这里）：游标取在"半截转义"之后时，新输出会被跳过。

    这不只是"漏一个字符"：锚行首的 `^TOPIC:` 永远等不到 ⟹ 夹具白等满超时。
    """
    px = new_console()
    feed(px, b"\r\npxh> listener foo -n 1\r\nTOPIC: foo\r\n")
    feed(px, b"\x1b[")                 # 半截转义：不匹配 ANSI_RE，先当普通字符留着
    bad_mark = len(px.text())          # ✗ 文本偏移，且此刻正卡在半截转义之后
    feed(px, b"2K\rTOPIC: bar\r\n")    # 后半截：两截合成整段被吃掉，渲染整体往前缩
    assert "TOPIC: bar" in px.text()                       # 内容其实在
    assert not re.search(r"(?m)^TOPIC: bar", px.text()[bad_mark:])   # 但从旧游标起看不到行首
    assert px.text()[bad_mark:].startswith("PIC: bar")     # "TO" 被整个跳过


def test_byte_cursor_sees_fresh_output():
    """★ 正例：字节游标（`mark()`）不受跨块重渲染影响。"""
    px = new_console()
    feed(px, b"\r\npxh> listener foo -n 1\r\nTOPIC: foo\r\n")
    mark = px.mark()                   # ✓ 字节游标
    feed(px, b"\x1b[")
    feed(px, b"2K\rTOPIC: bar\r\n")
    assert px.text_since(mark) == "TOPIC: bar\n", repr(px.text_since(mark))


def test_byte_cursor_is_monotone_and_empty_when_nothing_new():
    px = new_console()
    feed(px, b"aa\r\n")
    mark = px.mark()
    assert px.text_since(mark) == ""
    feed(px, b"bb\r\n")
    assert px.text_since(mark) == "bb\n"


if __name__ == "__main__":
    failed = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"✅ {name}")
            except AssertionError as e:
                failed += 1
                print(f"❌ {name}: {e}")
    print("全部通过" if not failed else f"{failed} 个用例失败")
    sys.exit(1 if failed else 0)
