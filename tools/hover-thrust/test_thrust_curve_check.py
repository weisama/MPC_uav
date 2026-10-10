#!/usr/bin/env python3
"""`thrust_curve_check` 的 **pytest 入口** —— 让 `pytest tools/hover-thrust/` 收得到它。

本文件**不重抄**那 11 条判据（重抄就等于第二份真话，改一处必漏一处）。
它做两件事：

1. 把 [`thrust_curve_check.self_check()`](thrust_curve_check.py) 接进仓库惯例的 pytest 入口 ——
   因为 `self_check()` 是**现场报告**式的（打印 + 返回不过的项名），pytest 收不到它，
   于是 `README.md` §5 广告的那条命令此前**跑不到**这把尺子。
2. 「一路控制通道没有激励」那条**失败路径的体例**（`analyse_ulog` 里那道闸门）——
   它不属于 `self_check()`（那条路要一份"日志"，而自检是纯合成的），所以只能在这里钉。

跑法（两条等价，第一条是仓库惯例）：

    pytest tools/hover-thrust/test_thrust_curve_check.py
    python3 tools/hover-thrust/thrust_curve_check.py --self-test

⚠️ 两者都**不需要**日志、不需要飞控通电，也不需要 `pyulog`（那个 import 是延后的）：
   下面那个假日志只实现 `analyse_ulog` **真正用到**的那一个方法。
"""

from __future__ import annotations

import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from thrust_curve_check import (  # noqa: E402
    N_MOTORS, analyse_ulog, curve_forward, self_check,
)


def test_自检全部通过():
    """11 条：正反解互为逆、fac=0 退化、单调、合成真值回收、对照档差距、仿射不变。

    ★ 失败时把**具体哪几条不过**打出来 —— `self_check()` 返回的就是项名列表，
    不要把它压成一个裸 `assert`，否则红了只知道"有一处不过"。
    """
    failures = self_check(emit=False)
    assert not failures, "自检不过：" + "、".join(failures)


# ─────────────────── `analyse_ulog` 的假日志（不需要 pyulog / 不需要 ulog 文件） ───────────────────

class _FakeDataset:
    def __init__(self, data):
        self.data = data


class _FakeUlog:
    """`analyse_ulog` 只用到 `u.get_dataset(topic).data[...]` —— 造这一个方法就够。"""

    def __init__(self, topics):
        self._topics = topics

    def get_dataset(self, topic):
        return _FakeDataset(self._topics[topic])


def _fake_log(control_of, output_of, n=400):
    """造一份假日志。`control_of(i, k)` / `output_of(i, k)` 给第 `i` 路第 `k` 个样本。

    时间戳按 ulog 惯例取**微秒**（`_load_series` 要自己乘 `1e-6`）。
    """
    t_us = [k * 10_000 for k in range(n)]
    motors = {"timestamp_sample": t_us}
    outputs = {"timestamp": t_us}
    for i in range(N_MOTORS):
        motors[f"control[{i}]"] = [control_of(i, k) for k in range(n)]
        outputs[f"output[{i}]"] = [output_of(i, k) for k in range(n)]
    return _FakeUlog({"actuator_motors": motors, "actuator_outputs": outputs})


#: 每路一份**互不为仿射**的电机信号曲线。★ 两个条件缺一不可：
#: ① 四路的区间/幂次不同 —— 否则它们是彼此的线性重标定、r² 全打平，
#:    `infer_mapping` 取 `max` 会把四个输出都指到同一路 ⟹ 双射检查先红在别处；
#: ② 不能用同一条斜坡 —— 即便区间不同，`x_j` 仍是 `x_i` 的**精确仿射**函数，r² 照样全等于 1。
_X_LO = (0.02, 0.04, 0.06, 0.08)
_X_SPAN = (0.10, 0.13, 0.16, 0.19)
_X_POW = (1.0, 1.35, 1.7, 2.05)
_A, _B = 1888.8, 109.0          # 真日志里那把仿射的量级（同 `self_check`）


def _x_of(i, k, n=400):
    return _X_LO[i] + _X_SPAN[i] * (k / (n - 1)) ** _X_POW[i]


def _healthy_log(fac=0.45, n=400):
    """四路都在动的正常日志 —— `control` 由电机信号正解、`output` 是它的仿射 + 整数量化。"""
    return _fake_log(
        lambda i, k: curve_forward(_x_of(i, k, n), fac),
        lambda i, k: round(_A * _x_of(i, k, n) + _B),
        n,
    )


def _dead_channel_log(jitter=True, n=400):
    """第 3 路 `control` **恒定**（这一段它没参与），其余三路正常。

    ★ `jitter` 是本条的**关键变量**：`output[3]` 也跟着恒定时，
    `analyse_ulog` 那条"四路输出都在动"的闸门会**先**拦下来（实测如此）——
    于是同一个错误形状里，只有"`control` 恒定而 `output` 在抖"这一半能走到拟合那一步。
    """
    def ctrl(i, k):
        return 0.0 if i == 3 else curve_forward(_x_of(i, k, n), 0.45)

    def out(i, k):
        if i == 3:
            return _B + (2.0 * ((k % 7) - 3) if jitter else 0.0)   # 抖动 >1 count ⟹ 过得了闸门
        return round(_A * _x_of(i, k, n) + _B)

    return _fake_log(ctrl, out, n)


def test_正常日志能一路走完_这道闸门不是见谁都拦():
    """★ 负控/对照：四路都在动时**不许**报"没有激励"。

    没有这一条，下面那条测试只要把闸门写成 `raise SystemExit` 就恒绿 —— 等于没鉴别力。
    """
    rep = analyse_ulog(_healthy_log(), "fake.ulg")
    assert rep["n_samples"] == 400
    assert rep["mapping_inferred"]


def test_一路control没激励时_报干净的SystemExit而不是栈():
    """★ 同族失败路径的体例：缺话题（`_load_series`）、缺字段、样本太少**全是** `SystemExit`。

    唯独"这一段没有激励"原来是从 `mean_r2` → `best_fac` 里抛 `ValueError` 冒泡成栈，
    而它恰好是**最可能**在真日志上遇到的一条（某一路电机没参与控制）。

    ★ 报的是**哪一路**：常量 `control` 会让每个 `output` 的 4 个候选**全部**抛一次，
      谁先被扫到取决于字典顺序 —— 所以必须在这之前判，不能靠接住异常。
    """
    with pytest.raises(SystemExit) as excinfo:
        analyse_ulog(_dead_channel_log(), "fake.ulg")
    assert "control[3]" in str(excinfo.value)


def test_output也恒定时_先被四路输出那道闸门拦下():
    """★ 实测边界，防止后来者把上一条读成"任何一路常量都会走到拟合"。

    `output[3]` 与 `control[3]` 一起恒定（= 真日志里"某电机没参与"的**自然形态**）时，
    `analyse_ulog` 在拟合**之前**就拦下了 —— 报的是"只有 3 路输出在动"，不是"没有激励"。
    """
    with pytest.raises(SystemExit) as excinfo:
        analyse_ulog(_dead_channel_log(jitter=False), "fake.ulg")
    assert "3 路输出在动" in str(excinfo.value)
