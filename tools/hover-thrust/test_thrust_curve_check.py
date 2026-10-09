#!/usr/bin/env python3
"""`thrust_curve_check` 的 **pytest 入口** —— 让 `pytest tools/hover-thrust/` 收得到它。

本文件**不重抄**那 11 条判据（重抄就等于第二份真话，改一处必漏一处）。
它只做一件事：把 [`thrust_curve_check.self_check()`](thrust_curve_check.py) 接进
仓库惯例的 pytest 入口 —— 因为 `self_check()` 是**现场报告**式的（打印 + 返回不过的项名），
pytest 收不到它，于是 `README.md` §5 广告的那条命令此前**跑不到**这把尺子。

跑法（两条等价，第一条是仓库惯例）：

    pytest tools/hover-thrust/test_thrust_curve_check.py
    python3 tools/hover-thrust/thrust_curve_check.py --self-test

⚠️ 自检**不需要**日志、不需要飞控通电，也不需要 `pyulog`（那个 import 是延后的）。
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from thrust_curve_check import self_check  # noqa: E402


def test_自检全部通过():
    """11 条：正反解互为逆、fac=0 退化、单调、合成真值回收、对照档差距、仿射不变。

    ★ 失败时把**具体哪几条不过**打出来 —— `self_check()` 返回的就是项名列表，
    不要把它压成一个裸 `assert`，否则红了只知道"有一处不过"。
    """
    failures = self_check(emit=False)
    assert not failures, "自检不过：" + "、".join(failures)
