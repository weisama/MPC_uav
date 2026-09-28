"""起飞前静态自检（`nmpc/preflight_check.py`）的测试 —— **每条检查都要有负控**。

★ 为什么这个文件值得存在：自检脚本最坏的形态是"永远打 ✅"—— 那样它比没有更糟（给人
  虚假的安全感）。所以这里对**每一组**检查都喂一个坏输入，断言它**真的会红**：
  符号反了、悬停没余量、心跳低于安全线、τ 太大、安全停机被关掉、参数名拼错、
  仓库根找不到（那一条算失败而不是跳过）、通道号写回代码里。

★ 全程**不起 DUT、不起夹具**：被测的是纯逻辑 + 参数表 + `ast` 扫描。
  （`_PARAMS` 从 `nmpc.mpc_node` 里取，所以本文件 import 得动 rclpy/acados 才行 —— `--solve`
   那一档才是真要解一次。）
"""

from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest

from nmpc.mpc_node import NodeParams
from nmpc.params import Z0Params
from nmpc.preflight_check import (
    Check,
    _channel_identifiers,
    build_node_params,
    check_normalization,
    check_params,
    check_signs,
    check_stop_conditions,
    find_repo_root,
    main,
)

REPO = Path(__file__).resolve().parents[3]


def _failing(checks) -> list:
    return [c.name for c in checks if not c.ok]


# ───────────────────────────── 正控：默认那一套全绿 ─────────────────────────────

def test_默认参数下快档全绿():
    """正控：`main` 的快档（不含 `--solve`）在默认参数下退出码 0。

    它同时钉住了"仓库根找得到"这件事 —— 找不到时符号那组会算失败（见下一条用例）。
    """
    assert main([]) == 0


def test_能找到仓库根():
    assert find_repo_root(REPO / "src" / "nmpc" / "nmpc") == REPO


# ───────────────────────────── 负控：符号 ─────────────────────────────

def test_符号被改反会被抓到():
    """把 pitch 的符号翻过来（**参考实现里最容易写错的那一行**）⟹ 两条对账都必须红。"""
    from nmpc.sign_map import px4_rates_setpoint

    def broken(omega, thrust, t_max):
        sp = px4_rates_setpoint(omega, thrust, t_max)
        return dataclasses.replace(sp, pitch=-sp.pitch)

    checks = check_signs(REPO, impl=broken)
    failed = [c for c in checks if not c.ok]
    assert [c.name for c in failed] == ["实现 == 文档", "实现 == 独立推导"], \
        f"只该有两条与实现相关的检查红，实际红的是 {[c.name for c in failed]}"


def test_文档与独立推导本身也要对得上():
    """第三条检查是"两边一起错"才会红的兜底 —— 它在默认情况下必须通过。"""
    checks = check_signs(REPO)
    assert all(c.ok for c in checks), _failing(checks)


# ───────────────────────────── 负控：参数 ─────────────────────────────

@pytest.mark.parametrize("field,value,expect", [
    ("hover_thrust", 1.2, "悬停有上余量"),
    ("heartbeat_hz", 1.0, "心跳 ≥2 Hz"),
    ("tau_s", 0.5, "外推 τ 在合理区间"),
    ("warmup_cycles", 0, "热身拍数 ≥1"),
    ("status_timeout_s", 0.2, "状态时限 ≥ 上游一个周期"),
    ("stop_on_armed_drop", False, "两条安全停机都开着"),
    ("stop_on_nav_state_exit", False, "两条安全停机都开着"),
    ("mass", 0.0, "质量为正"),
])
def test_坏参数会被点名(field, value, expect):
    p = dataclasses.replace(NodeParams(), **{field: value})
    assert expect in _failing(check_params(p)), \
        f"{field}={value} 应该让「{expect}」红，实际红的是 {_failing(check_params(p))}"


def test_归一化常数被硬编码会被抓到():
    """★ 这一组存在的理由就是参考实现那个坑：它把 `1/T_max` 硬编码成 `0.07`
    （改质量时静默错 33 %）。负控就是那一版：归一化不再跟着**形参** `t_max` 走。"""
    from nmpc.sign_map import Px4RatesSetpoint

    def hardcoded(omega, thrust, t_max):
        return Px4RatesSetpoint(roll=omega[0], pitch=-omega[1], yaw=-omega[2],
                                thrust_body_z=-thrust * 0.07)   # ← 参考实现那一行

    failed = _failing(check_normalization(Z0Params(), impl=hardcoded))
    assert "换 T_max 归一化跟着走" in failed, f"硬编码归一化常数没被抓到，红的是 {failed}"

    # 正控：实现里的每一行都跟着 t_max 走时，四条全绿
    assert all(c.ok for c in check_normalization(Z0Params())), \
        "正控：真实实现下这一组必须全绿（否则下面那条负控说明不了问题）"


# ───────────────────────────── 负控：停机条件与通道号 ─────────────────────────────

def test_通道号写回代码里会被抓到(tmp_path: Path):
    """★ 这一条**同时**是正控（注释里的 `ch8` 不算）：不然没人敢在注释里引用那张票。"""
    commented = tmp_path / "commented.py"
    commented.write_text('# 急停 = ch8 高值（phase1/issues/02 的结论）\nX = 1\n', encoding="utf-8")
    assert _channel_identifiers([commented]) == [], "注释里的 ch8 是引用文档，不该算违规"

    real = tmp_path / "real.py"
    real.write_text('ch8 = read_rc()\nname = "rc_channel"\n', encoding="utf-8")
    hits = _channel_identifiers([real])
    assert len(hits) == 2, f"ch8 与 \"rc_channel\" 都该被扫出来，实际 {hits}"


def test_停机条件的字段名会被对账():
    checks = check_stop_conditions(NodeParams())
    assert all(c.ok for c in checks), _failing(checks)
    assert "warmup_cycles" in checks[0].detail, \
        "多出来的闸门字段要被列出来（`warmup_cycles` 是首拍不发，不是停机条件）"


# ───────────────────────────── CLI 的边界 ─────────────────────────────

def test_参数名拼错会当场退():
    """rclpy 对拼错的参数名**不报错**（它会静静待着），所以自检必须自己当场退。"""
    with pytest.raises(SystemExit) as exc:
        build_node_params({"hover_thrus": "0.5"})
    assert "hover_thrus" in str(exc.value)


def test_值转不成类型也当场退():
    with pytest.raises(SystemExit) as exc:
        build_node_params({"mass": "很重"})
    assert "mass" in str(exc.value)


def test_找不到仓库根算失败而不是跳过(tmp_path: Path):
    """★ 这是**设计决定**的可执行版本：符号对账是本脚本的核心，找不到权威文档就**不能算过**
    （静默跳过 = 给人虚假的安全感）。`--repo` 指错地方也一样。"""
    assert main(["--repo", str(tmp_path)]) == 1


def test_检查对象是冻结的_结果只读():
    c = Check("组", "名", True, "细节")
    with pytest.raises(dataclasses.FrozenInstanceError):
        c.ok = False  # type: ignore[misc]
