"""纯函数：QGC 参数导出的解析 + 「此刻（串口读回） vs 快照（导出文件）」的对账。

★ **不 import** `pymavlink` / `serial` / `rclpy`，也**不读文件**（文件由调用方读成 `bytes`）。
  所以自检**不需要飞控通电、不需要导出文件存在**。
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

__all__ = [
    "REL_TOL", "ABS_TOL", "NAME_RE", "Param", "ExportParseError",
    "parse_qgc_export", "decode_export", "same_value", "Diff", "diff",
]

#: 数值相等的**相对**容差。两边的来源不同、打印位数也不同：
#: QGC 导出写的是 float32 的十进制展开（`0.45` ⟹ `0.449999988079071`，
#: `0.2` ⟹ `0.200000002980232239`），MAVLink 读回来的是同一个 float32。
#: 用**严格相等**会把一整批参数判成"变了"，而那是**表示误差，不是改动**。
#: 1e-6 比那层误差大几个数量级，又远小于任何**真的**改动（这个文件里最小的一档是 0.001）。
REL_TOL = 1e-6

#: 绝对下限。大数上相对容差会松得不合理（`921600` 的 1e-6 = 0.92，一位数的改动会被吞掉），
#: 而参数域里比 1e-4 更小的改动没有意义（它们都是**配置**，不是测量值）。
#: ⟹ 判据取 `|a−b| <= max(ABS_TOL, REL_TOL·max(|a|,|b|))`。
ABS_TOL = 1e-4

#: PX4 参数名的形状：大写字母/数字/下划线，首位是字母。MAVLink 的 `param_id` 是 `char[16]`
#: ⟹ 到 16 个字符为止（★ **不是**"看着像就行"：这个正则就是"这一行是不是参数行"的判据，
#: 解不出来的行会进 `skipped` 被原样报出来，**不静默丢**）。
NAME_RE = re.compile(r"[A-Z][A-Z0-9_]{0,15}")


class ExportParseError(Exception):
    """导出文件里**一行参数都没解出来** —— 那说明文件选错了，不是"文件是空的"。"""


@dataclass(frozen=True)
class Param:
    """一条参数的三个面：名字、**原样字符串**（导出文件里那一列）、解析出的数。"""

    name: str
    text: str
    value: Optional[float]  # 解不出数就是 None（枚举/字符串型参数）

    def __str__(self) -> str:
        return self.text


def decode_export(data: bytes) -> str:
    """把导出文件的原始字节解成文本。

    ★ **逐行**解，每行独立退回：z0 那份 `docs/assets/z0.params` 的 header 是 **UTF-8**、
    body 里带**原始 GBK 字节**（车名那一行），整块按任何一种编码解都会在别处炸。
    数据行本身是纯 ASCII，所以逐行解码对"我们真正要读的那部分"没有任何损失。
    （文件自己的 header 也写着这条：要改它只能按字节抠。）
    """
    out: List[str] = []
    for raw in data.split(b"\n"):
        out.append(raw.rstrip(b"\r").decode("utf-8", errors="replace"))
    return "\n".join(out)


def parse_qgc_export(text: str) -> Tuple[Dict[str, Param], List[str]]:
    """解析 QGC 导出的 `.params`。

    格式（第一手，`docs/assets/z0.params` 第 11 行起）：`sysid \\t compid \\t NAME \\t VALUE \\t TYPE`，
    tab 分隔、行尾 CRLF、`#` 开头的行是注释。

    返回 `(params, skipped)`：
      · `params` —— 名字 ⟹ `Param`；**同名后者覆盖前者**（QGC 导出里不会出现，真出现要能看见）；
      · `skipped` —— **非空、非注释、但解不出参数名**的行（原样留档）。
        ⚠️ 它非空**不一定**是坏事（QGC 有时真会加说明行），但**必须被打印出来**，
        否则"少了几条参数"就变成静默的。
    """
    params: Dict[str, Param] = {}
    skipped: List[str] = []
    for line in text.split("\n"):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        parts = line.split("\t")
        if len(parts) >= 4 and NAME_RE.fullmatch(parts[2].strip()):
            name = parts[2].strip()
            value_text = parts[3].strip()
            try:
                value: Optional[float] = float(value_text)
            except ValueError:
                value = None
            params[name] = Param(name=name, text=value_text, value=value)
        else:
            skipped.append(line.rstrip("\r"))
    if not params:
        raise ExportParseError(
            "这份文件里一行参数都没解出来 —— 大概率是选错了文件"
            "（要的是 QGC 导出的 `.params`，不是 ulog / 不是 yaml）")
    return params, skipped


def same_value(a: Optional[float], b: Optional[float]) -> bool:
    """两个读数是不是**同一个值**（容差见 `REL_TOL` / `ABS_TOL`）。

    两边都解不出数（枚举/字符串型）时**判为不同** —— 那种情况由调用方按字符串比，
    这里不替它下结论。
    """
    if a is None or b is None:
        return False
    return abs(a - b) <= max(ABS_TOL, REL_TOL * max(abs(a), abs(b)))


@dataclass(frozen=True)
class Diff:
    """「此刻 vs 快照」的三类结果。`changed` 是**降序按名字排**的，便于逐条核对。"""

    same: int
    changed: List[Tuple[str, str, str]]   # (名字, 快照值, 此刻值)
    only_current: List[str]               # 飞控里有、快照里没有
    only_reference: List[str]             # 快照里有、飞控里没有

    @property
    def clean(self) -> bool:
        """★ 三条都空才算"对上了"。只看 `changed` 会把"参数被删/被加"放过去。"""
        return not self.changed and not self.only_current and not self.only_reference


def diff(current: Dict[str, Param], reference: Dict[str, Param]) -> Diff:
    """`current`（此刻，串口读回）对 `reference`（快照，导出文件）逐条对账。"""
    changed: List[Tuple[str, str, str]] = []
    same = 0
    for name in sorted(current):
        cur = current[name]
        ref = reference.get(name)
        if ref is None:
            continue
        if cur.value is None or ref.value is None:
            ok = cur.text == ref.text
        else:
            ok = same_value(cur.value, ref.value)
        if ok:
            same += 1
        else:
            changed.append((name, ref.text, cur.text))
    only_current = sorted(set(current) - set(reference))
    only_reference = sorted(set(reference) - set(current))
    return Diff(same=same, changed=changed, only_current=only_current,
                only_reference=only_reference)
