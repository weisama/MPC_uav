"""ticket 23：字符类残留 `-?[0-9.eE+-]+` 上的「半截数值」—— 取不到就 `None`、绝不抛。

**为什么这一层要单测**：本仓几个取数器吃的是**有损窗口**（`px.text()[marker:]` 的文本偏移
窗口、或 `text_since(mark)` 的**字节窗口**），窗口边界可以落在数值中间。旧字符类
`-?[0-9.eE+-]+` **本身**能只匹配到一个裸 `-`（或 `e`/`E`/`.`/`+`），于是 `float('-')`
抛 `ValueError` —— 而那是在**一场飞行跑完之后**抛的，等于**整场作废**
（2026-09-29 加密档第一场就是这么死的，见 `docs/control/mismatch-sweep.md` §附.8）。

★ 这里钉两条：
  ① **半截值** ⟹ 取不到就 `None`、**绝不抛**（这是本次修的东西）；
  ② **合法写法一个都不许变**（这是"修法只影响原本会抛的输入"那条论断的机器可读版）。

★ 与 `test_mismatch_sweep.py` 里那一节的关系：那一节测的是 `verify_offboard_smoke.field()`，
  本文件接着把 ticket 23 **点名的其余几处**（`verify_z0_plant` 的两个取数器、
  `verify_offboard_smoke.World._read_param`）一并钉住。

★ 用**人造窗口**（不是真飞）：一条"崩"在真跑里要一整场，而它应该是一毫秒的事。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import verify_z0_plant as vz  # noqa: E402  （它自己会把 `z0-plant/` 塞进 sys.path）

# ------------------------------------------------------------------ verify_z0_plant
# 不依赖 ROS：这两个取数器吃的是 `marker = len(px.text())` → `px.text()[marker:]` 的
# **文本偏移**窗口，而用具 `px4_console.text()` / `wait_for()` 的 docstring 自己就把
# "文本偏移当游标"判为无效（渲染不是增量的）⟹ 边界可以落在数值中间。


@pytest.mark.parametrize("line, name, want", [
    (" x * SIH_MASS [843,1330] : 0.7500", "SIH_MASS", 0.7500),
    (" x   SIH_T_MAX [843,1330] : 2.9500", "SIH_T_MAX", 2.9500),
    (" x * CA_ROTOR0_PX [843,1330] : -0.1400", "CA_ROTOR0_PX", -0.1400),
])
def test_param_value_正常数值一个都没变(line, name, want):
    """修法**只该**把"半截值"从抛改成 `None` —— 合法 `param show` 行一律照旧。"""
    assert vz.param_value(line, name) == want


@pytest.mark.parametrize("line", [
    " x * SIH_MASS [843,1330] : -",   # ☠️ 第一手复现：窗口断在负号后
    " x * SIH_MASS [843,1330] : e",   # ☠️ 同理：'e' 也在旧字符类里
    " x * SIH_MASS [843,1330] : .",
])
def test_param_value_半截值给_None_绝不抛(line):
    assert vz.param_value(line, "SIH_MASS") is None


@pytest.mark.parametrize("line, want", [
    ("    z: 0.00016", 0.00016),
    ("    x: -0.00023", -0.00023),          # 负号 + 小数
    ("    vz: -1.2e-3", -0.0012),           # 科学计数
    ("    z: .5", 0.5),                     # 小数点开头
    ("    z: 5.", 5.0),                     # 小数点结尾
])
def test_field_正常数值一个都没变(line, want):
    assert vz.field(line, line.strip().split(":")[0]) == want


@pytest.mark.parametrize("line", [
    "    z: -",                    # ☠️ 真值流的窗口边界落在负号后面
    "    z: e",
    "    z: .",
])
def test_field_半截值给_None_绝不抛(line):
    assert vz.field(line, "z") is None


# ------------------------------------------------- verify_offboard_smoke（要 ROS 环境）
# 被测模块导入期就 `import rclpy` / `px4_msgs`（没有环境直接 skip，不让整文件红）。


def _smoke():
    pytest.importorskip("rclpy")
    pytest.importorskip("px4_msgs")
    import verify_offboard_smoke
    return verify_offboard_smoke


class _ConsoleStub:
    """够 `World._read_param` 用即可：它只要 `mark` / `send` / `drain` / `text_since`。"""

    def __init__(self, chunk):
        self._chunk = chunk

    def mark(self):
        return 0

    def send(self, *a, **k):
        pass

    def drain(self, *a, **k):
        pass

    def text_since(self, mark):
        return self._chunk


class _WorldStub:
    def __init__(self, chunk):
        self.px = _ConsoleStub(chunk)


def test_read_param_正常数值一个都没变():
    """`World._read_param`（ticket 23 点名的旧 `:862/:863`，修 `field()` 后现 `:875/:876`）。"""
    v = _smoke()
    got = v.World._read_param(_WorldStub("x   COM_OF_LOSS_T [281,483] : 1.0000"),
                              "COM_OF_LOSS_T")
    assert got == 1.0


def test_read_param_负值照旧():
    v = _smoke()
    got = v.World._read_param(_WorldStub("x   MPC_Z_VEL_P_ACC [1,2] : -2.5000"),
                              "MPC_Z_VEL_P_ACC")
    assert got == -2.5


def test_read_param_半截值给_None_绝不抛():
    """窗口 `... : -`（字节边界落在负号后）—— 旧字符类会 `float('-')` 抛。"""
    v = _smoke()
    assert v.World._read_param(_WorldStub("x   COM_OF_LOSS_T [281,483] : -"),
                               "COM_OF_LOSS_T") is None
