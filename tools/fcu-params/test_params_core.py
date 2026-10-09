"""`params_core` 的自检：解析、容差、对账三类。

★ 全部**离线** —— 不碰串口、不需要飞控通电。所以它可以天天跑。
★ 每一条都用**合成输入**，判据是"给一个已知的改动，能不能原样认出来"；
  其中容差那组是**双向**的：表示误差**不许**报、真改动**必须**报。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from params_core import (  # noqa: E402
    ExportParseError, Param, decode_export, diff, parse_qgc_export, same_value,
)

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
