"""纯函数：QGC 参数导出的解析 + 「此刻（串口读回） vs 快照（导出文件）」的对账。

★ **不 import** `pymavlink` / `serial` / `rclpy`，也**不读文件**（文件由调用方读成 `bytes`）。
  所以自检**不需要飞控通电、不需要导出文件存在**。
"""

from __future__ import annotations

import re
import struct
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

__all__ = [
    "REL_TOL", "ABS_TOL", "NAME_RE", "Param", "ExportParseError",
    "parse_qgc_export", "decode_export", "same_value", "Diff", "diff",
    "MAV_PARAM_TYPE_UINT8", "MAV_PARAM_TYPE_INT8", "MAV_PARAM_TYPE_UINT16",
    "MAV_PARAM_TYPE_INT16", "MAV_PARAM_TYPE_UINT32", "MAV_PARAM_TYPE_INT32",
    "MAV_PARAM_TYPE_UINT64", "MAV_PARAM_TYPE_INT64", "MAV_PARAM_TYPE_REAL32",
    "MAV_PARAM_TYPE_REAL64", "INT_PARAM_TYPES", "UNSIGNED_PARAM_TYPES",
    "decode_param_value", "Collected", "collect_params",
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


#: MAVLink `PARAM_VALUE.param_type` 的取值（`common.xml` 的 `MAV_PARAM_TYPE` 枚举）。
#: ★ PX4 只会发出 **`INT32`(6)** 与 **`REAL32`(9)** 两种（见下 `decode_param_value` 的依据）；
#: 外加一条 **`UINT32`(5)** 的**伪参数**（`_HASH_CHECK`，参数哈希）。其余档位是协议允许、PX4 不发。
MAV_PARAM_TYPE_UINT8 = 1
MAV_PARAM_TYPE_INT8 = 2
MAV_PARAM_TYPE_UINT16 = 3
MAV_PARAM_TYPE_INT16 = 4
MAV_PARAM_TYPE_UINT32 = 5
MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_UINT64 = 7
MAV_PARAM_TYPE_INT64 = 8
MAV_PARAM_TYPE_REAL32 = 9
MAV_PARAM_TYPE_REAL64 = 10

#: 整型那几档 —— 它们在线上是**位型**，不是数值（见 `decode_param_value`）。
INT_PARAM_TYPES = frozenset({MAV_PARAM_TYPE_UINT8, MAV_PARAM_TYPE_INT8,
                             MAV_PARAM_TYPE_UINT16, MAV_PARAM_TYPE_INT16,
                             MAV_PARAM_TYPE_UINT32, MAV_PARAM_TYPE_INT32,
                             MAV_PARAM_TYPE_UINT64, MAV_PARAM_TYPE_INT64})

#: 无符号的那几档 —— 用 `<I` 而不是 `<i` 解（`-1` 与 `4294967295` 是同一个位型）。
UNSIGNED_PARAM_TYPES = frozenset({MAV_PARAM_TYPE_UINT8, MAV_PARAM_TYPE_UINT16,
                                  MAV_PARAM_TYPE_UINT32, MAV_PARAM_TYPE_UINT64})


@dataclass(frozen=True)
class Param:
    """一条参数的四个面：名字、**字符串形式的值**、解析出的数、MAVLink 里的类型。"""

    name: str
    text: str
    value: Optional[float]  # 解不出数就是 None（枚举/字符串型参数）
    ptype: Optional[int] = None  # `PARAM_VALUE.param_type`；导出文件那一侧没有这个概念 ⟹ None

    def __str__(self) -> str:
        return self.text


def decode_param_value(raw: float, ptype: Optional[int]) -> Tuple[float, str]:
    """把 MAVLink 的 `PARAM_VALUE.param_value` 解成**真值**。返回 `(值, 字符串形式)`。

    ★★ 这是本工具最容易静默错的地方，机理是**第一手**的
    （`refs/px4-v1.14.3-build/src/modules/mavlink/mavlink_parameters.cpp`，
    `MavlinkParametersManager::send_param`，源码里那句注释自己就写着
    "since MAVLink encodes float and int params in the same space during transmission,
    copy param onto float val_buf"）：

    ```cpp
    if (param_type(param) == PARAM_TYPE_INT32) {
            int32_t param_value;
            param_get(param, &param_value);
            memcpy(&msg.param_value, &param_value, sizeof(param_value));  // ← 位型塞进 float 字段
    } else {
            msg.param_value = param_value;                                // ← 数值直传
    }
    ```

    ⟹ **不按 `param_type` 解，整型参数会以 float32 的位型现身**：
    `NAV_RCL_ACT = 1` 读成 `1.401298464324817e-45`（= 0x00000001 当 float32）、
    `-1` 读成 `nan`（= 0xFFFFFFFF）、`921600` 读成 `1.2914366647217514e-39`。
    ☠️ 后果是**看上去像"飞控上有一两百条参数被改了"**，而那正是这个工具唯一要回答的问题 ——
    整批假差异会把真差异（本例里只有 5 条：3 条 `CAL_ACC0_*` 与 2 个计时器）淹掉。

    ⚠️ `param_value` 字段只有 **4 字节**，所以 64 位档位（`UINT64`/`INT64`）实际也只承载 32 位；
    PX4 不发这两档，这里按 32 位解并在 `Param.ptype` 里留下原始类型以便复核。
    """
    v = float(raw)
    if ptype in INT_PARAM_TYPES:
        bits = struct.pack("<f", v)
        n = struct.unpack("<I" if ptype in UNSIGNED_PARAM_TYPES else "<i", bits)[0]
        return float(n), str(n)
    return v, repr(v)


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


@dataclass(frozen=True)
class Collected:
    """一遍全量读取的**汇总**。★ 它不是"读到了什么"，而是"这次读**可不可信**"。"""

    params: Dict[str, Param]        # 真参数：名字 ⟹ `Param`
    pseudo: List[str]               # `param_index < 0` 的伪参数（`_HASH_CHECK`）
    duplicates: List[str]           # 重名 ⟹ 飞控**重传过**（不是新参数）
    expected: Optional[int]         # 飞控自报的参数条数（`PARAM_VALUE.param_count`）
    missing_indices: List[int]      # 在 `[0, expected)` 里**没收到**的索引 ⟹ 链路上丢了

    @property
    def complete(self) -> bool:
        return self.expected is not None and not self.missing_indices


def collect_params(msgs) -> Collected:
    """把一串**已拆好字段**的 `PARAM_VALUE` 汇总成 `Collected`。

    ★ 纯函数：`msgs` 是 `(name, raw_value, param_type, param_index, param_count)` 的序列，
      **不碰串口、不 import pymavlink** ⟹ 离线就能把这三件事逐个造出来验：

      ① **伪参数**：PX4 回全量时第一条永远是 `_HASH_CHECK`（`param_index = -1`），
         它**不是**参数、QGC 导出里也没有。混进 `params` 就会在对账的"飞控多出"那一栏
         冒充一条新增参数。
      ② **重传**：同一条来两遍。按名字去重的 `dict` 会把它**悄悄吃掉**，
         看着像"没收全"；这里单独记下来，两种毛病才分得开。
      ③ **丢包**：完整性按 **`param_index` 覆盖**判，不按名字个数判。
         `param_count` 是飞控自报的 **used 参数个数**（`param_count_used()`），
         而每条真参数的 `param_index` 是 `param_get_used_index(param)`（`send_param` 里那行），
         取值恰好是 `[0, param_count_used())` 且**不重不漏** ⟹ 缺的索引就是 `[0, expected)` 减收到的。

    ⚠️ `msgs` 要是一个**可重复遍历**的序列（list/tuple），不是一次性生成器。
    """
    msgs = list(msgs)
    params: Dict[str, Param] = {}
    pseudo: List[str] = []
    indices: set = set()
    dupes: List[str] = []
    expected: Optional[int] = None
    for name, raw_value, ptype, index, count in msgs:
        if count is not None:
            expected = int(count)
        if index < 0:
            # ★ 伪参数（参数哈希）**不进** `params` —— 它不是参数，QGC 导出里也没有。
            if name not in pseudo:
                pseudo.append(name)
            continue
        value, text = decode_param_value(raw_value, ptype)
        if name in params:
            dupes.append(name)
        params[name] = Param(name=name, text=text, value=value, ptype=ptype)
        indices.add(int(index))
    missing = sorted(set(range(expected)) - indices) if expected else []
    return Collected(params=params, pseudo=pseudo, duplicates=sorted(set(dupes)),
                     expected=expected, missing_indices=missing)


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
