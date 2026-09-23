"""`docs/control/flight-runbook.md`（现场那一页）的守卫 —— 编号与命令**只有一个出处**。

跑法：

    python3 -m pytest run/tests/test_flight_runbook.py -q

★ **为什么单开一份**：ticket 22 的复核抓到过一条真会误导现场的错 ——
`bench-runbook.md` 的指针把菜单号写成「**9= 起链核对**」（9 是操作台 `fly.py`，起链核对是 **2**），
照它敲会起到操作台。那是"文档引用代码里的编号"这一类腐坏的标准形状，且**肉眼复核抓不住**
（数字看着都对）。所以把新写的这一页钉在四个权威上：

| 文档里写的 | 钉在哪 |
|---|---|
| 菜单号 + 标题 | `z0.ACTIONS`（编号即表里的顺序） |
| 起链那几条命令 | `z0.ACTION_BY_KEY` / `z0.UP_CHAIN`（**命令的唯一出处**） |
| 核对表那几条话题 | `z0.PROBES`（核对器实际订阅的东西） |
| `degrade=` 的 token | `control` 的 `to_string(Degrade)`（C++ 词表，逐字取） |

★★ **有意只扫这一份文档**（不是 `docs/**`）：本仓有并行会话，别人的在制品混在同一个工作树里，
扫全 `docs/` 会让**他们**改文档时红在**我们**这里。要扩大范围应当先想清楚这件事。

★ 菜单引用的**写法是约定**（本文件的第一条用例即判据）：写成 `菜单 **N= 标题**`，
标题照抄 `run/z0.py --list` 那一列。散文里那种"菜单 3/4/5 都是起链"的写法本用例管不着 ——
所以定成显式的一种写法，而不是去猜自由散文。
"""

import pathlib
import re
import sys

RUN_DIR = pathlib.Path(__file__).resolve().parents[1]
REPO = RUN_DIR.parent
sys.path.insert(0, str(RUN_DIR))

import z0  # noqa: E402  （路径注入了才 import 得到）

DOC = REPO / "docs" / "control" / "flight-runbook.md"
FSM_HPP = REPO / "src" / "control" / "include" / "control" / "fsm.hpp"


def _doc() -> str:
    assert DOC.exists(), f"现场那一页不在：{DOC}"
    return DOC.read_text(encoding="utf-8")


def _normalize(text: str) -> str:
    """去掉加粗星号与多余空白 —— 只用来比对，不改文档本身。"""
    return re.sub(r"\s+", " ", text.replace("*", " ")).strip()


# ─────────────────────────── 菜单号 ───────────────────────────


def _menu_refs(text: str):
    """取出文档里所有 `菜单 **N= 标题**` 形式的引用。"""
    return [(int(n), title.strip())
            for n, title in re.findall(r"菜单\s*\*\*(\d+)=\s*([^*]+?)\*\*", text)]


def test_菜单引用与动作表对得上():
    refs = _menu_refs(_doc())
    assert len(refs) >= 6, f"现场那一页应当逐个点名菜单项（只认出 {len(refs)} 条）"
    for number, cited in refs:
        assert 1 <= number <= len(z0.ACTIONS), f"菜单里没有第 {number} 项"
        title = _normalize(z0.ACTIONS[number - 1].title).lstrip("★ ").strip()
        assert title.startswith(_normalize(cited).lstrip("★ ").strip()), (
            f"菜单 **{number}= {cited}** 与动作表对不上：第 {number} 项是「{z0.ACTIONS[number - 1].title}」")


# ─────────────────────────── 命令 ───────────────────────────


def _command_lines(text: str):
    """文档里出现的启动命令（**逐行**取，去掉缩进）—— 只认这两种前缀。"""
    return [line.strip() for line in text.splitlines()
            if line.strip().startswith(("ros2 launch", "ros2 run"))]


def test_文档里的启动命令都来自动作表():
    """★ 命令的唯一出处是 `ACTIONS`：文档不许另抄一份（抄了就会各自漂移）。
    命令**尾巴上的参数**（`fcu_url:=` / `gcs_url:=` / `use_vrpn:=false`）允许写全 —— 判据是前缀。"""
    lines = _command_lines(_doc())
    assert lines, "现场那一页应当把起链那几条命令写出来"
    cmds = [a.cmd for a in z0.ACTIONS if a.cmd]
    for line in lines:
        assert any(line == cmd or line.startswith(cmd + " ") for cmd in cmds), \
            f"这条命令不在动作表里：`{line}`"


def test_起链那几条命令的顺序与_UP_CHAIN_一致():
    """起链**顺序不能反**（`bench-runbook.md` 那一节的标题）⟹ 文档里 `ros2 launch` 的先后
    必须与 `UP_CHAIN` 逐条对齐。`fly.py` 那条（`ros2 run`）不算在起链里 —— 它是起完之后的事。"""
    launches = [line for line in _command_lines(_doc()) if line.startswith("ros2 launch")]
    expected = [z0.ACTION_BY_KEY[key].cmd for key in z0.UP_CHAIN]
    assert len(launches) == len(expected), \
        f"起链应当是 {len(expected)} 条，文档里写了 {len(launches)} 条：{launches}"
    for line, cmd in zip(launches, expected):
        assert line == cmd or line.startswith(cmd + " "), f"顺序对不上：期望 `{cmd}`，文档写 `{line}`"


def test_核对表那七条话题一条不少():
    text = _doc()
    for topic, _expected in z0.PROBES:
        assert topic in text, f"核对器会读 `{topic}`，文档里没写"


# ─────────────────────────── 日志 token ───────────────────────────


def _degrade_tokens() -> set:
    """从 C++ 词表里逐字取出 `degrade=` 的全部取值（**不在这里另抄一份**）。"""
    source = FSM_HPP.read_text(encoding="utf-8")
    start = source.rindex("inline const char * to_string(Degrade d)")
    body = source[start:source.index("\n}\n", start)]
    return set(re.findall(r'return "([a-z0-9_]+)"', body))


def test_引用的_degrade_token_都在词表里():
    text = _doc()
    cited = set(re.findall(r"degrade=([a-z0-9_]+)", text))
    assert cited, "现场那一页应当告诉人 `degrade=` 该长什么样"
    known = _degrade_tokens()
    assert known, f"没能从 {FSM_HPP} 里取出词表（文件结构变了？）"
    assert cited <= known, f"这些 token 不在词表里：{sorted(cited - known)}"
