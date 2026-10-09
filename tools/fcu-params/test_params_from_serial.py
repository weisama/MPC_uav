"""`params_from_serial` 的自检 —— **只覆盖它不碰串口的那一半**。

★ 这个文件的由来（值得记一笔）：`--self-test` 曾经是红的，而 `test_params_core.py`
  9 条**全绿** —— 因为红的那个判据（`self_test` 里的已知值核对用了 `abs(...) > 1e-9`、
  比模块自己的容差还严，于是把 float32 的表示误差当成了"解错"）**没有任何测试覆盖**。
  ⟹ 所以这里钉的不是"函数返回什么"，而是"`--self-test` 这条命令本身是绿的"。

⚠️ 全部**离线**：`self_test()` 按设计不发一个字节到串口（`pymavlink` 是延迟 import，
   这个文件里的任何一条都不 import 它）。所以它可以天天跑，也不需要有串口权限。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import params_from_serial as pfs  # noqa: E402

#: 仓库里那份真导出（2026-09-29）。自检的输入。
Z0_EXPORT = Path(__file__).resolve().parents[2] / "docs/assets/z0.params"

pytestmark = pytest.mark.skipif(not Z0_EXPORT.exists(), reason="仓库里没有那份导出文件")


def test_self_test_在真导出上是绿的():
    """★ 这条就是那个洞的补丁：判据必须与模块的容差**同一把尺**。

    ⚠️ 它红了**不代表**串口那半边有问题 —— 它只说明"解析 / 容差 / 对账"三者之间
    自相矛盾了（例如已知值核对偷偷用了比 `same_value` 更严的判据）。
    """
    assert pfs.self_test(str(Z0_EXPORT)) == 0


def test_命令行_self_test_也是绿的():
    """`--self-test` 这条路（`main` 的分支 + argparse 默认值）走通、退出码 0。"""
    assert pfs.main(["--self-test", "--self-test-export", str(Z0_EXPORT)]) == 0


def test_已知值核对与对账用的是同一把尺():
    """⛔ 反面：把已知值那一栏换成比 `same_value` 更严的判据，`--self-test` 必须变红。

    这是上面那条测试的**负控** —— 证明它真的在管这件事，而不是碰巧绿的。
    （`HTE_THR_RANGE` 在真文件里是 `0.200000002980232239`，与字面 `0.2` 差 3e-9。）
    """
    ref, _ = pfs.read_export(Z0_EXPORT)
    got = ref["HTE_THR_RANGE"].value
    assert got is not None and abs(got - 0.2) > 1e-9, "真文件里这一条居然精确等于 0.2 —— 负控失效"
    assert pfs.same_value(got, 0.2), "容差把 float32 的表示误差判成了不同 —— 对账会满屏假差异"
