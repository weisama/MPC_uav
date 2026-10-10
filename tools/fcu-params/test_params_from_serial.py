"""`params_from_serial` 的自检 —— **只覆盖它不碰串口的那一半**。

★ 这个文件的由来（值得记一笔）：`--self-test` 曾经是红的，而 `test_params_core.py`
  9 条**全绿** —— 因为红的那个判据（`self_test` 里的已知值核对用了 `abs(...) > 1e-9`、
  比模块自己的容差还严，于是把 float32 的表示误差当成了"解错"）**没有任何测试覆盖**。
  ⟹ 所以这里钉的不是"函数返回什么"，而是"`--self-test` 这条命令本身是绿的"。

⚠️ 全部**离线**：没有一条开串口、没有一条需要串口权限。`self_test()` 按设计不发一个字节；
   `pull_params` 那条用的是**脚本化的假链路**（`pymavlink` 那半边是 `importorskip` + 猴补丁，
   真串口一个字节都不碰）—— 换来的好处是：那个 `break` 条件**真的**有测试管着。
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import params_from_serial as pfs  # noqa: E402

#: 仓库里那份真导出 —— 本文件自检的输入。
#: ★ **不在这里写日期**：这个路径**换过两次档**（说明头「换过两次」①②），而路径一直没变 ⟹
#:   写死日期就会像上一版那样，把 2026-10-10 那份的取值记成 2026-09-29 的出处。
#:   要溯源看那个文件自己的说明头（哪一档、为什么换、跟上一档差几条，都在那儿）。
Z0_EXPORT = Path(__file__).resolve().parents[2] / "docs/assets/z0.params"

#: 只在**跑真文件**的那几条上挂。`pull_params` 那条不依赖它（它喂的是假链路）。
_need_export = pytest.mark.skipif(not Z0_EXPORT.exists(), reason="仓库里没有那份导出文件")


@_need_export
def test_self_test_在真导出上是绿的():
    """★ 这条就是那个洞的补丁：判据必须与模块的容差**同一把尺**。

    ⚠️ 它红了**不代表**串口那半边有问题 —— 它只说明"解析 / 容差 / 对账"三者之间
    自相矛盾了（例如已知值核对偷偷用了比 `same_value` 更严的判据）。
    """
    assert pfs.self_test(str(Z0_EXPORT)) == 0


@_need_export
def test_命令行_self_test_也是绿的():
    """`--self-test` 这条路（`main` 的分支 + argparse 默认值）走通、退出码 0。"""
    assert pfs.main(["--self-test", "--self-test-export", str(Z0_EXPORT)]) == 0


@_need_export
def test_已知值核对与对账用的是同一把尺():
    """⛔ 反面：把已知值那一栏换成比 `same_value` 更严的判据，`--self-test` 必须变红。

    这是上面那条测试的**负控** —— 证明它真的在管这件事，而不是碰巧绿的。
    （`HTE_THR_RANGE` 在真文件里是 `0.200000002980232239`，与字面 `0.2` 差 3e-9。）
    """
    ref, _ = pfs.read_export(Z0_EXPORT)
    got = ref["HTE_THR_RANGE"].value
    assert got is not None and abs(got - 0.2) > 1e-9, "真文件里这一条居然精确等于 0.2 —— 负控失效"
    assert pfs.same_value(got, 0.2), "容差把 float32 的表示误差判成了不同 —— 对账会满屏假差异"


# ─────────────────── `pull_params` 的假链路（不开串口、不要串口权限） ───────────────────

class _FakeMsg:
    """一条 `PARAM_VALUE` 的**最小**形状 —— `pull_params` 只读这五个字段。"""

    def __init__(self, name, index, count, value=0.0, ptype=9):
        self.param_id = name.encode("ascii")   # 线上是 `char[16]`，bytes
        self.param_index = index
        self.param_count = count
        self.param_value = value
        self.param_type = ptype                # 9 = REAL32


class _FakeConn:
    """**脚本化**的假链路：按 `msgs` 顺序发，发完返 `None`（同真链路的空闲）。

    `recv_match_calls` 记**问了几次** —— 这是判"到点就收手"的那一半证据：
    只看"收齐了没有"，是分不出"读完就 `break`"与"压根没有 `break` 条件、一路问到超时"的。
    """

    def __init__(self, msgs):
        self._msgs = list(msgs)
        self.recv_match_calls = 0
        self.target_system = 1
        self.target_component = 1
        self.mav = types.SimpleNamespace(param_request_list_send=lambda *a, **k: None)

    def wait_heartbeat(self, timeout=None):
        return types.SimpleNamespace(autopilot=12, type=2)

    def recv_match(self, *a, **k):
        self.recv_match_calls += 1
        return self._msgs.pop(0) if self._msgs else None

    def close(self):
        pass


def test_pull_params_不被重传骗到而提前收手(monkeypatch):
    """★★ 这条钉的是 `pull_params` 里那个 `break` 条件**本身**。

    （`test_params_core.py::test_读满判据_*` 只钉了那把尺子 —— 尺子对而没被用上，
    症状一模一样。这两层缺一层都挡不住回归。）

    脚本：应收 3 条（index 0/1/2），**index 1 重传一次、且重传发生在 index 2 之前**。
    数**消息条数**的话，第 3 条消息一到就已经"3/3" —— 可那时 index 2 还在路上，
    于是这次读取会带着「⚠️ 没收全 … 缺的 param_index 是 [2]」收场，
    读起来像"飞控把这个参数删了"。

    ★ 两头都钉：**不许早收手**（上面那条），也**不许不收手**（`recv_match_calls == 4`
    —— 把 `break` 整句删掉，循环会一路问到超时，而收齐的参数照样是这三条，
    只看结果的判据抓不住）。

    ⚠️ 需要 `pymavlink`（只 import 模块本身，不开串口、不要权限）；没装就跳过。
    """
    mavutil = pytest.importorskip("pymavlink.mavutil")
    script = [
        _FakeMsg("A", 0, 3),
        _FakeMsg("B", 1, 3),
        _FakeMsg("B", 1, 3),        # ← 重传：按消息条数数的话，这里就"收满了"
        _FakeMsg("C", 2, 3),
    ]
    conn = _FakeConn(script)
    monkeypatch.setattr(mavutil, "mavlink_connection", lambda *a, **k: conn)

    params, info = pfs.pull_params("/dev/null", 921600, 5.0, quiet=True)

    assert sorted(params) == ["A", "B", "C"], "重传被当成了进度 ⟹ 提前 break，只收到两条"
    assert info["missing_indices"] == [], "提前 break 会让这次读取报一条假警报"
    assert info["duplicates"] == ["B"], "重传仍要单记一栏 —— 别把它一起修没了"
    assert conn.recv_match_calls == 4, "收齐第 4 条之后就该 break —— 多问的那几次是空转"
