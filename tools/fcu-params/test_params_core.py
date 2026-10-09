"""`params_core` 的自检：解析、容差、对账三类。

★ 全部**离线** —— 不碰串口、不需要飞控通电。所以它可以天天跑。
★ 每一条都用**合成输入**，判据是"给一个已知的改动，能不能原样认出来"；
  其中容差那组是**双向**的：表示误差**不许**报、真改动**必须**报。
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from params_core import (  # noqa: E402
    MAV_PARAM_TYPE_INT32, MAV_PARAM_TYPE_REAL32, MAV_PARAM_TYPE_UINT32,
    ExportParseError, Param, collect_params, decode_export, decode_param_value, diff,
    parse_qgc_export, same_value,
)


def _msg(name, wire, ptype, index, count):
    """一条 `PARAM_VALUE` 拆出来的五元组 —— `pull_params` 喂给纯函数的正是这个形状。"""
    return (name, wire, ptype, index, count)


def _bit(u: int) -> float:
    """造一个"位型是 `u` 的 float32" —— 也就是 PX4 把一个 int32 塞进 `param_value` 之后的样子。"""
    return struct.unpack("<f", struct.pack("<I", u & 0xFFFFFFFF))[0]

#: 仓库里那份真导出（2026-09-29）。用来钉"解析器在**真文件**上确实能用"。
Z0_EXPORT = Path(__file__).resolve().parents[2] / "docs/assets/z0.params"


def _export_bytes() -> bytes:
    """合成一份"长得像真文件"的导出：UTF-8 的中文头 + CRLF 数据行 + 一行 GBK 字节。

    ★ 这不是随手编的：真文件 `docs/assets/z0.params` 的三条形状全在里面
      （header UTF-8 / body CRLF / body 里夹原始 GBK 字节 ⟹ 整块按任何单一编码解都会炸）。
    """
    head = "# z0 飞控参数文件（**真身**）\r\n".encode("utf-8")
    gbk = b"# Vehicle: \xb3\xb5\xd0\xcd\xa3\xa8GBK\xd7\xd6\xbd\xda\xa3\xa9\r\n"   # "车型（GBK字节）"
    body = (
        b"1\t1\tNAV_RCL_ACT\t1.000000000000000000\t6\r\n"
        b"1\t1\tMC_BAT_SCALE_EN\t0.000000000000000000\t6\r\n"
        b"1\t1\tMPC_THR_HOVER\t0.500000000000000000\t9\r\n"
        b"1\t1\tTHR_MDL_FAC\t0.449999988079071\t9\r\n"
        b"1\t1\tSER_TEL2_BAUD\t921600\t6\r\n"
    )
    return head + gbk + body


def test_解析_中文头_CRLF_GBK字节都拦不住它():
    """三条形状同时在场时仍能解出参数 —— 而且**不该整块解码**（真文件就是这种）。"""
    with pytest.raises(UnicodeDecodeError):
        _export_bytes().decode("utf-8")          # 负控：证明"整块解"在这份输入上真的会炸
    params, skipped = parse_qgc_export(decode_export(_export_bytes()))
    assert set(params) == {"NAV_RCL_ACT", "MC_BAT_SCALE_EN", "MPC_THR_HOVER",
                           "THR_MDL_FAC", "SER_TEL2_BAUD"}
    assert skipped == []                          # 两行 `#` 是注释，**不算** skipped
    assert params["NAV_RCL_ACT"].value == 1.0
    assert params["SER_TEL2_BAUD"].value == 921600.0


def test_解不出参数名的行进_skipped_不静默丢():
    text = ("# 注释\n"
            "这不是参数行\n"
            "1\t1\tbad_name\t1\t6\n"            # 小写 ⟹ 不是 PX4 参数名
            "1\t1\tthr_mdl_fac\t1\t6\n"
            "1\t1\tGOOD_ONE\t2.0\t9\n")
    params, skipped = parse_qgc_export(text)
    assert set(params) == {"GOOD_ONE"}
    assert len(skipped) == 3, skipped


def test_一行参数都解不出要报错而不是返回空():
    """⚠️ "文件选错了"必须与"文件是空的"分开 —— 前者要当场拦，不能被当成"对上 0 条"。"""
    with pytest.raises(ExportParseError):
        parse_qgc_export("# 只有注释\n\n")


def test_容差_float32的两种写法是同一个值():
    """QGC 写 `0.449999988079071`、MAVLink 读回 `0.45` —— **不许**报成改动。"""
    assert same_value(0.449999988079071, 0.45)
    assert same_value(0.200000002980232239, 0.2)
    assert same_value(921600.0, 921600.0)


def test_容差_真改动必须报出来():
    """双向：容差不许松到把**真改动**吞掉。这一组是"会不会太松"的那一半。"""
    assert not same_value(0.5, 0.1589)            # 本票要写的那一条
    assert not same_value(0.45, 0.451)            # 1e-3 量级
    assert not same_value(1.0, 1.001)
    assert not same_value(2.0, 12.0)              # `MPC_XY_VEL_MAX` 那个 6× 的悬空引用


def test_容差_解不出数的两边不替调用方下结论():
    """枚举/字符串型（`value is None`）**判为不同**，由调用方按字符串比。"""
    assert not same_value(None, None)
    assert not same_value(None, 1.0)


def test_对账_三类都要有且_clean_只看三类():
    ref = {"A": Param("A", "1.0", 1.0), "B": Param("B", "2.0", 2.0),
           "GONE": Param("GONE", "3.0", 3.0)}
    cur = {"A": Param("A", "1.0", 1.0), "B": Param("B", "9.0", 9.0),
           "NEW": Param("NEW", "4.0", 4.0)}
    d = diff(cur, ref)
    assert d.same == 1
    assert d.changed == [("B", "2.0", "9.0")]
    assert d.only_current == ["NEW"]
    assert d.only_reference == ["GONE"]
    assert not d.clean
    assert diff(ref, ref).clean and diff(ref, ref).same == len(ref)

    # ⛔ 上面那组三类同时非空 ⟹ `clean` 就算只看 `changed` 也照样是 False（没鉴别力）。
    #    这两条才是"只看 changed"的**反面**：一次只多一样，另外两样是空的。
    a = Param("A", "1.0", 1.0)
    only_new = diff({"A": a, "NEW": Param("NEW", "4.0", 4.0)}, {"A": a})
    assert only_new.changed == [] and only_new.only_current == ["NEW"]
    assert not only_new.clean, "只多出一条也必须是「没对上」—— 否则飞控加了参数会被静默放过"

    only_gone = diff({"A": a}, {"A": a, "GONE": Param("GONE", "3.0", 3.0)})
    assert only_gone.changed == [] and only_gone.only_reference == ["GONE"]
    assert not only_gone.clean, "少了一条同理（可能是被删、改名，或这次没读全）"


def test_位型解码_整型按param_type还原_浮点直传():
    """★★ 2026-10-09 真踩到的那个坑（`MavlinkParametersManager::send_param`）。

    PX4 对 `PARAM_TYPE_INT32` 是 `memcpy(&msg.param_value, &param_value, 4)` —— **位型**塞进
    那个 float 字段；浮点才是数值直传。不按 `param_type` 解，整批整型参数会以 float32 位型现身。
    """
    assert decode_param_value(_bit(1), MAV_PARAM_TYPE_INT32)[0] == 1.0
    assert decode_param_value(_bit(30), MAV_PARAM_TYPE_INT32)[0] == 30.0
    assert decode_param_value(_bit(921600), MAV_PARAM_TYPE_INT32)[0] == 921600.0
    assert decode_param_value(_bit(121212), MAV_PARAM_TYPE_INT32)[0] == 121212.0
    assert decode_param_value(_bit(0xFFFFFFFF), MAV_PARAM_TYPE_INT32)[0] == -1.0   # ★ 不是 nan
    # 无符号档（伪参数 `_HASH_CHECK` 那种）要按 `<I` 解
    assert decode_param_value(_bit(0xFFFFFFFF), MAV_PARAM_TYPE_UINT32)[0] == 4294967295.0
    # 浮点直传，一个 bit 都不许动
    assert decode_param_value(0.5, MAV_PARAM_TYPE_REAL32)[0] == 0.5
    assert decode_param_value(-0.061959743499755859, MAV_PARAM_TYPE_REAL32)[0] == -0.061959743499755859
    # `param_type` 未知（None）时按直传处理 —— 但**别**指望它；这是兜底，不是正路
    assert decode_param_value(0.5, None)[0] == 0.5


def test_位型解码_搞反了长什么样_负控():
    """⛔ 负控：把 `param_type` 丢掉（当成浮点直传），上面那几条**必须**面目全非。

    ★ 这一条存在的意义是让"忽略 `param_type`"这个 bug **无法静默**：
    `NAV_RCL_ACT = 1` 会读成 1.4e-45，`-1` 会读成 nan —— 而对账那边看上去就是
    "飞控上一两百条参数被改了"。这正是 2026-10-09 第一版工具的读数。
    """
    assert abs(decode_param_value(_bit(1), None)[0]) < 1e-44          # 1.4e-45，不是 1
    assert abs(decode_param_value(_bit(921600), None)[0]) < 1e-38     # 1.3e-39，不是 921600
    nan_like = decode_param_value(_bit(0xFFFFFFFF), None)[0]
    assert nan_like != nan_like                                       # NaN 自己不等于自己
    # 而按 param_type 解，三者都是干净的数
    assert decode_param_value(_bit(1), MAV_PARAM_TYPE_INT32)[0] == 1.0
    assert decode_param_value(_bit(921600), MAV_PARAM_TYPE_INT32)[0] == 921600.0
    assert decode_param_value(_bit(0xFFFFFFFF), MAV_PARAM_TYPE_INT32)[0] == -1.0


def test_位型解码_端到端_整批整型参数不该报成改动():
    """★★ 回归测试：把一批**已知类型**的真值编成线上位型，再走解码 + 对账，必须**干净**。

    ★ 挑的都是本票真在意的那几条（`NAV_RCL_ACT` / `MPC_USE_HTE` 是"此刻飞控里还是不是它"
      那几个框里的两条）。第一版工具在这份输入上会报出 8 条假差异里的绝大多数。
    """
    truth = {   # 名字 ⟹ (真值, param_type)
        "NAV_RCL_ACT": (1, MAV_PARAM_TYPE_INT32),
        "MPC_USE_HTE": (1, MAV_PARAM_TYPE_INT32),
        "COM_FLTMODE2": (-1, MAV_PARAM_TYPE_INT32),
        "SER_TEL2_BAUD": (921600, MAV_PARAM_TYPE_INT32),
        "MAV_1_CONFIG": (102, MAV_PARAM_TYPE_INT32),
        "MPC_THR_HOVER": (0.5, MAV_PARAM_TYPE_REAL32),
        "HTE_THR_RANGE": (0.2, MAV_PARAM_TYPE_REAL32),
        "MPC_THR_MIN": (0.12, MAV_PARAM_TYPE_REAL32),
    }
    def wire(n):
        v, pt = truth[n]
        return _bit(v) if pt == MAV_PARAM_TYPE_INT32 else float(v)

    # 快照那一侧：真值是**十进制字面**（QGC 导出就是这种）
    ref = {n: Param(n, str(truth[n][0]), float(truth[n][0])) for n in truth}
    # 此刻那一侧：按 PX4 的规则编成线上值，再走**生产那条解码**
    cur = {}
    for n in truth:
        value, text = decode_param_value(wire(n), truth[n][1])
        cur[n] = Param(n, text, value, truth[n][1])

    d = diff(cur, ref)
    assert d.clean, f"整型参数的位型没解对 ⟹ 假差异：{d.changed}"
    assert d.same == len(truth)


def test_汇总_伪参数不算参数也不占计数():
    """★★ `_HASH_CHECK` 是 PX4 **回全量的第一条**（`param_index = -1`），不是参数。

    ⚠️ 把它算进 `params`，对账的"飞控多出"那一栏就会凭空多一条 —— 2026-10-09 的读数里
    它就在那儿（`only_current: _HASH_CHECK`），看上去像飞控新加了一个参数。
    """
    c = collect_params([
        _msg("_HASH_CHECK", _bit(0xDEADBEEF), MAV_PARAM_TYPE_UINT32, -1, 3),
        _msg("NAV_RCL_ACT", _bit(1), MAV_PARAM_TYPE_INT32, 0, 3),
        _msg("MPC_THR_HOVER", 0.5, MAV_PARAM_TYPE_REAL32, 1, 3),
        _msg("SER_TEL2_BAUD", _bit(921600), MAV_PARAM_TYPE_INT32, 2, 3),
    ])
    assert set(c.params) == {"NAV_RCL_ACT", "MPC_THR_HOVER", "SER_TEL2_BAUD"}
    assert c.pseudo == ["_HASH_CHECK"]
    assert c.params["NAV_RCL_ACT"].value == 1.0
    assert c.params["SER_TEL2_BAUD"].value == 921600.0
    assert c.complete and c.missing_indices == []


def test_汇总_丢包按param_index报而不是按名字个数():
    """★★ 完整性只能按索引判：`param_count` 是飞控自报的 used 条数，缺哪个索引要能指名。"""
    c = collect_params([
        _msg("A", _bit(1), MAV_PARAM_TYPE_INT32, 0, 4),
        _msg("B", _bit(2), MAV_PARAM_TYPE_INT32, 2, 4),   # index 1 没来
        _msg("C", _bit(3), MAV_PARAM_TYPE_INT32, 3, 4),
    ])
    assert c.missing_indices == [1]
    assert not c.complete
    assert len(c.params) == 3


def test_汇总_重传单独记_且不冒充没收全():
    """★ 重传与丢包是**两件事**，必须分得开。

    按名字去重的 `dict` 会把重传悄悄吃掉，于是"收到 3 条 / 应有 4 条"看着像丢包；
    按索引判就不会：索引齐了就是齐了，重传另记一栏。
    """
    c = collect_params([
        _msg("A", _bit(1), MAV_PARAM_TYPE_INT32, 0, 2),
        _msg("A", _bit(1), MAV_PARAM_TYPE_INT32, 0, 2),   # 同一条来两遍
        _msg("B", _bit(2), MAV_PARAM_TYPE_INT32, 1, 2),
    ])
    assert c.duplicates == ["A"]
    assert c.missing_indices == []
    assert c.complete, "重传不该被判成没读全 —— 索引是齐的"
    assert len(c.params) == 2, "重传不该把参数算成两条"


def test_汇总_没有param_count就不敢判完整():
    """`count is None`（消息里没给）时**不许**说"读全了" —— 没依据。"""
    c = collect_params([_msg("A", _bit(1), MAV_PARAM_TYPE_INT32, 0, None)])
    assert c.expected is None
    assert not c.complete


def test_对账_枚举型参数按原样字符串比():
    ref = {"EKF2_MAG_TYPE": Param("EKF2_MAG_TYPE", "Auto", None)}
    same = {"EKF2_MAG_TYPE": Param("EKF2_MAG_TYPE", "Auto", None)}
    diffd = {"EKF2_MAG_TYPE": Param("EKF2_MAG_TYPE", "Manual", None)}
    assert diff(same, ref).clean
    assert diff(diffd, ref).changed == [("EKF2_MAG_TYPE", "Auto", "Manual")]


@pytest.mark.skipif(not Z0_EXPORT.exists(), reason="仓库里没有那份导出文件")
def test_在真导出上_自己对自己是干净的():
    """★ 容差与解析器在**真文件**（955 行 / 911 条 / 混合编码）上必须一致 ——
    这条要是红了，"对账"这件事本身就不可信。"""
    params, _ = parse_qgc_export(decode_export(Z0_EXPORT.read_bytes()))
    assert len(params) > 900
    assert params["NAV_RCL_ACT"].value == 1.0
    assert params["MC_BAT_SCALE_EN"].value == 0.0
    assert params["MPC_THR_HOVER"].value == 0.5
    assert params["MPC_XY_VEL_MAX"].value == 2.0
    assert diff(params, params).clean
