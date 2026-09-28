"""判据层的离线单测 —— 不启 SITL、不启 agent，一毫秒跑完。

**为什么值得单独一个文件**：`verify_offboard_smoke.py` 的每一条判据在真跑里要 ~1 分钟
（每场一套全新 SITL），于是"判据本身写错了"这种事**没人会去试**：跑一次很贵，
看见一片 ✅ 就更不会怀疑判据。这里把**能脱离 SITL 判的那部分**（跨档两条 + ⑤ 的高度判据）
拉出来，喂人造数：它们必须在该过时过、该炸时炸。

★ 两条跨档判据走的是 `--replay`：观测是 JSON，判据是纯函数 —— 这正是"仪器与判据分开"
的用处（改判据不必重跑飞机），也让"报告层"第一次有了回归测试。

（这个文件要 `rclpy` / `px4_msgs`：被测模块在导入期就 import 它们。没有 ROS 的环境
直接 skip，不会变成"跑不过"。）
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("px4_msgs")

HERE = Path(__file__).resolve().parent


def _load():
    """按**路径**导入被测脚本（`tools/` 不是一个包，没有 `__init__.py`）。

    ⚠️ 必须先塞进 `sys.modules` 再 `exec_module`：`@dataclass` 装饰器会按
    `cls.__module__` 回表找模块对象，找不到就是 `AttributeError: 'NoneType'
    object has no attribute '__dict__'`。
    """
    name = "verify_offboard_smoke"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


vs = _load()


# ---------------------------------------------------------------- ⑤ 高度判据


@pytest.mark.parametrize("z, vz, expect", [
    (-1.61, 0.018, True),
    (-1.61, 0.0, True),        # ★ 假的零：vz=0.0 是**飞得最稳**的那一帧
    (-1.61, -0.049, True),
    (-1.61, -0.051, False),    # 真在掉
    (-1.61, None, False),      # 没测到，不是"没掉"
    (None, 0.0, False),        # 没测到
    (-0.10, 0.0, False),       # 还在地上 —— "没掉高"不等于"飞起来了"
])
def test_altitude_held(z, vz, expect):
    """`vz == 0.0` 与 `vz is None` 必须分开判。

    ☠️ 这条是复核逮到的：原判据写成 `(gt_vz_end or -1) > -0.05`，`0.0` 是假值 ⟹
    被换成 `-1` ⟹ **飞得最稳的那一帧被判 ❌**（而且只在最稳那次出现，最难复现）。
    """
    assert vs.altitude_held(z, vz) is expect


# ---------------------------------------------------------------- 跨档两条


def test_cross_qos_control_needs_both_sides():
    ok, text = vs.cross_qos_control({"baseline": 25, "qos-reliable": 0})
    assert ok is True and "25" in text and "0" in text
    # 两个都收得到 ⟹ "差别来自 QoS"这个解释不成立
    ok, text = vs.cross_qos_control({"baseline": 25, "qos-reliable": 25})
    assert ok is False and "❌" in text
    # 对照组自己在场却没收到 ⟹ 同样不成立
    ok, _ = vs.cross_qos_control({"baseline": 0, "qos-reliable": 0})
    assert ok is False


def test_cross_qos_control_says_when_it_did_not_run():
    """只跑了 qos-reliable 一场 ⟹ 这条判据**没做**，`None`（不是 False，也不是 True）。

    ★ 三种结果刻意分开：True / False / None（没做）。把"没做"算成通过就是假的绿；
    算成失败又会在 `--only` 的跑法里误报。
    """
    ok, text = vs.cross_qos_control({"qos-reliable": 0})
    assert ok is None and "baseline" in text and "没做" in text


def test_cross_delta_flags_drift_but_not_sampling_noise():
    ok, text = vs.cross_delta(0.084, 0.177)          # 实测那一跑
    assert ok is True and "一致" in text
    ok, text = vs.cross_delta(0.05, 0.48)            # |差| = 0.43 ⟹ 远超取样噪声
    assert ok is False and "❌" in text
    assert vs.cross_delta(None, 0.177)[0] is None
    assert vs.cross_delta(0.084, None)[0] is None


def test_cross_delta_is_documented_as_weak():
    """把这条判据的**弱**写在 docstring 里 —— 免得后来人把它当机制的铁证。

    （两个 lag 的点上，"有 Δ"与"age ∝ lag"两种模型拟合得一样好：
    0.916/1.823 对 0.916/1.832。机制的铁证是源码加三条单场判据。）
    """
    doc = vs.cross_delta.__doc__ or ""
    assert "分辨力有限" in doc and "c≈0.92" in doc


# ---------------------------------------------------------------- --replay 出口


def _lag_entry(name, age, enter):
    """人造一场 ts-lag 的观测 —— 只填 `evaluate` 真会读的那些字段。"""
    lag = float(name.split("-")[-1][0])
    return {
        "name": name,
        # `argv` 是**被测节点**那一场的开关（报告里"指令：…"那行，真跑时由夹具记下）
        "argv": ["--phases", f"lift:1.2:1.2:0,0,0", f"--timestamp-shift-us",
                 str(-int(lag) * 1_000_000)],
        "offboard_enter": enter,
        "offboard_enter_rel": enter,
        "armed_ever": True,
        "com_of_loss_t": 1.0,
        "px4_ts_age_s": age,
        "px4_ts_seen": 6_367_692.0,
        "status_seen": [-1, 4, 14] if enter is not None else [-1, 4],
        "dut": {"status_cb": 22, "ts_sent": 1_790_595_810_308_564,
                "nav_seen": [-1, 4, 14] if enter is not None else [-1, 4]},
        "ts_age_delta_s": lag - age,
    }


def _replay(tmp_path, results, extra=()):
    p = tmp_path / "obs.json"
    p.write_text(json.dumps(results, ensure_ascii=False), encoding="utf-8")
    return vs.main(["--replay", str(p), *extra])


def test_replay_of_a_consistent_run_exits_zero(tmp_path, capsys):
    rc = _replay(tmp_path, [_lag_entry("ts-lag-1s", 0.916, 1.0),
                            _lag_entry("ts-lag-2s", 1.823, None)])
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "跨档 Δ 一致性" in out


def test_replay_of_an_inconsistent_run_exits_nonzero_even_with_table(tmp_path, capsys):
    """☠️ 复核逮到的那一条：`--replay --table` 曾经直接 `return table(...)`。

    于是——**报告层的跨档判据一次都不跑、退出码恒 0**，而交付文档里生成那张表的命令
    正是 `--replay obs.json --table`；表里的数看着对，"判据过没过"这件事却没人判。
    """
    # ★ 两个 Δ 各自都还在单场判据的 (0.05, 0.5) 里 —— 所以退出码非 0 **只能**来自
    #   那条跨档判据（否则这个测试会"因为别的原因红"，证明不了 `--table` 那条修复）。
    bad = [_lag_entry("ts-lag-1s", 0.90, 1.0),    # Δ=0.10
           _lag_entry("ts-lag-2s", 1.55, None)]   # Δ=0.45 ⟹ |差|=0.35 > 0.20
    assert _replay(tmp_path, bad) == 1
    assert _replay(tmp_path, bad, extra=["--table"]) == 1   # ★ 加表格也必须是 1
    out = capsys.readouterr().out
    assert "❌" in out


def test_replay_of_an_empty_artifact_is_a_failure(tmp_path, capsys):
    """一场都没跑成（世界起不来时每场都被 `continue` 掉）**不是**"全部场景通过"。

    报告层原本 `bad = 0` 恒成立 ⟹ 打出一句"全部场景通过"、退出码 0 ——
    这是这份报告最丢人的一种错法：什么都没验，却报绿。
    """
    assert _replay(tmp_path, []) == 1
    assert "一场都没跑完" in capsys.readouterr().out
