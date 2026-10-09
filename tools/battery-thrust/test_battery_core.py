#!/usr/bin/env python3
"""`battery_core` 的自检 —— 纯函数，**不需要 pyulog、不需要 ulog 文件、不需要飞控**。

每条断言都按"**能区分对错**"挑：写完之后要能想清楚"我把它写反/写坏了，哪一条会变红"。
下面几条是刻意这么设计的：

* `test_可持续判据忽略孤点` —— 把滑窗中位换成单点最大，那条 `value < raw_max` 立刻红；
* `test_判据自检在电流与电压无关时不认那点耦合` —— 造一份 `I` 与 `V` **无关**的数据，
  逼 `esr_and_signature` 说出"只有采样噪声"。若它对着无关数据报出 R，就是**机理反了**；
* `test_锚点搜索范围先夹到在飞段` —— 这条是**真的踩过的坑**（第一版五份日志一份锚点没取到）。

跑法::

    python3 tools/battery-thrust/test_battery_core.py --self-test
    pytest tools/battery-thrust/test_battery_core.py
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import battery_core as C  # noqa: E402


# ------------------------------------------------------------------ 滑窗 / 可持续


def test_可持续判据忽略孤点():
    """★ 判据是"1 s 滑窗内**中位**"的最大值，**不是**单点最大。

    这正是票面警告的那个坑：`4_traj2_square` 里单点能到 11.38 A，
    而同一段的滑窗中位只有 4.91 A。差 2.3 倍，且方向永远是"单点更大"。
    """
    t = np.arange(0.0, 20.0, 0.2)  # 5 Hz，1 s 窗 = 5 个样本
    x = np.full(t.size, 3.0)
    x[50] = 12.0  # 一个孤点
    m = C.sustained_max(t, x, dt=1.0, min_n=3)
    assert abs(m["value"] - 3.0) < 1e-12, f"孤点污染了滑窗中位：{m['value']}"
    assert abs(m["raw_max"] - 12.0) < 1e-12
    assert m["raw_max"] > m["value"], "这条不成立就说明判据退化成单点最大值了"


def test_滑窗不满不外推_返回None不是0():
    """★ 样本太少时返回 `None`，**不是 0** —— "没量到"与"量到 0 A"是两件事。"""
    t = np.array([0.0, 0.2, 0.4])
    x = np.array([3.0, 3.0, 3.0])
    m = C.sustained_max(t, x, dt=1.0, min_n=5)
    assert m["value"] is None and m["n_windows"] == 0
    assert m["raw_max"] == 3.0, "取不到滑窗不代表连原始样本都没读到"


def test_窗口按时间不按下标():
    """★ 窗口是**时间**窗，不是固定点数。采样率变了，判据不能跟着变。

    同一段物理过程，5 Hz 与 20 Hz 两份采样给的答案必须一致
    （按下标切窗会得到两个不同的数，且都不对）。
    """
    def ramp(fs):
        t = np.arange(0.0, 20.0, 1.0 / fs)
        return t, np.where(t < 10.0, 3.0, 6.0)

    a = C.sustained_max(*ramp(5))
    b = C.sustained_max(*ramp(20))
    assert abs(a["value"] - b["value"]) < 1e-9, (a["value"], b["value"])
    assert abs(a["value"] - 6.0) < 1e-9


# ------------------------------------------------------------------ 阶梯取值 / 掩码


def test_阶梯取值取此前最近的一个():
    """布尔量按阶梯取，**不能线性插值** —— 插出来的 0.5 既不是"已落地"也不是"未落地"。"""
    t_src = np.array([0.0, 10.0, 20.0])
    x = np.array([0.0, 1.0, 0.0])
    out = C.step_lookup(np.array([-1.0, 0.0, 9.99, 10.0, 20.0, 99.0]), t_src, x)
    assert list(out) == [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]


def test_插值超出源范围给NaN而不是端点值():
    """★ `np.interp` 默认把范围外**夹到端点值**——那会凭空造出一段"稳态"。

    夹住的话，日志开头/结尾那几秒会被当成"和端点同样快地飞"，锚点就可能落在虚假的稳态上。
    """
    t_src = np.array([10.0, 20.0, 30.0])
    x_src = np.array([1.0, 2.0, 3.0])
    out = C.interp_to(t_src, x_src, np.array([0.0, 15.0, 40.0]))
    assert np.isnan(out[0]) and np.isnan(out[2])
    assert abs(out[1] - 1.5) < 1e-12


def test_在飞掩码是解锁与未落地的与():
    """判据 = `flag_armed ∧ ¬landed`，两条各自在自己的时间基上取样后**逐点相与**。

    解锁与离地是**两条不同的时间基**（样本数不同）。广播对齐会静默地
    把两个不同时刻的状态配到一起 —— 这里用不同的采样率逼出这个形状。
    """
    t_q = np.arange(0.0, 40.0, 1.0)
    t_cm = np.arange(0.0, 40.0, 2.0)
    t_ld = np.arange(0.0, 40.0, 0.5)
    armed = (t_cm >= 10.0).astype(float)
    landed = ((t_ld < 12.0) | (t_ld >= 30.0)).astype(float)
    m = C.in_flight_mask(t_q, t_cm, armed, t_ld, landed)
    assert list(t_q[m]) == list(np.arange(12.0, 30.0, 1.0))


def test_地面静止比真悬停更稳态():
    """★ 把**这个物理事实**钉下来：地面上飞机不动、倾角为零，满足一切"稳态"判据。

    这不是理论担忧 —— `hover_anchor` 第一版没有 `eligible` 参数，
    于是"最长稳态段"被地面段整段赢走，八份归档日志**一份锚点都没取到**。
    """
    t = np.arange(0.0, 100.0, 0.2)
    n = t.size
    speed = np.where(t < 60.0, 0.0, 0.6)   # 0–60 s 在地面（静止），之后在飞（机动）
    tilt = np.zeros(n)
    speed = np.where((t >= 70.0) & (t < 80.0), 0.0, speed)  # 70–80 s 真空闲悬停
    m = C.steady_mask(speed, tilt)
    i0, i1 = C.longest_run(m)
    assert abs(t[i0]) < 1e-9, "最长稳态段应当确实被地面段赢走 —— 否则这个用例没鉴别力"
    assert abs(t[i1 - 1] - 59.8) < 0.21


def test_锚点搜索范围先夹到在飞段():
    """★★ 回归测试：`eligible` 把搜索范围夹到在飞段之后，锚点必须落到真正的悬停上。

    不夹 = 锚点落在地面（电压是静置电压，电流是静态电流，两个都不是我们要的量）。
    """
    t = np.arange(0.0, 100.0, 0.2)
    n = t.size
    speed = np.where(t < 60.0, 0.0, 0.6)
    speed = np.where((t >= 70.0) & (t < 80.0), 0.0, speed)
    tilt = np.zeros(n)
    voltage = np.where((t >= 60.0) & (t < 70.0), 21.0,
                       np.where((t >= 70.0) & (t < 80.0), 22.0, 25.0))
    current = np.where((t >= 60.0) & (t < 70.0), 6.0,
                       np.where((t >= 70.0) & (t < 80.0), 3.5, 0.4))
    eligible = (t >= 60.0) & (t < 90.0)
    a, _ = C.hover_anchor(t, voltage, current, speed, tilt, eligible=eligible)
    assert a is not None
    assert abs(a.t0 - 70.0) < 0.21 and abs(a.t1 - 79.8) < 0.21, (a.t0, a.t1)
    assert abs(a.voltage_v - 22.0) < 1e-9, "锚点取到了地面段的静置电压"
    assert abs(a.current_a - 3.5) < 1e-9, "锚点取到了地面段的静态电流"
    assert abs(a.thrust_n - 0.75 * 9.80665) < 1e-9


def test_稳态段不够长就不给锚点():
    """`min_duration` 是**真的在判**：唯一一段稳态只有 1 s，必须拒绝而不是挑那段。

    ⚠️ 造这条用例时踩过自己一次：初版写成 `t < 20 s 静止、其余在动`，
    于是**开头那段地面静止**本身就是一段 20 s 的稳态 —— 用例长得像在测
    `min_duration`，其实测的是别的东西。所以这里把"到处都是动的"写死。
    """
    t = np.arange(0.0, 30.0, 0.2)
    speed = np.full(t.size, 0.6)                            # 全段都在动
    speed = np.where((t >= 21.0) & (t < 22.0), 0.0, speed)  # 唯一一段静止：1 s
    a, why = C.hover_anchor(t, np.full(t.size, 22.0), np.full(t.size, 3.5),
                            speed, np.zeros(t.size), min_duration=2.0)
    assert a is None and "2 s" in why["reason"]


def test_倾角从四元数取的是总倾角():
    """`arccos(1 − 2(qx²+qy²))`：绕 z 转不动倾角，绕 x/y 才动。"""
    assert C.tilt_rad_from_quat([1.0, 0.0, 0.0, 0.0]) == 0.0
    yaw = [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]  # 90° 偏航
    assert abs(C.tilt_rad_from_quat(yaw)) < 1e-12
    tilt = [np.cos(np.pi / 8), np.sin(np.pi / 8), 0.0, 0.0]  # 45° 俯仰
    assert abs(C.tilt_rad_from_quat(tilt) - np.pi / 4) < 1e-12


# ------------------------------------------------------------------ 噪声 / 判据自检


def test_稳健噪声不是标准差():
    """★ 偶发孤点会把标准差抬高一大截，把真正的样本级抖动掩盖掉。

    这条用的是本仓真实形状的数据：`current_a` 上散着几个 0 与几个孤峰。
    """
    rng = np.random.default_rng(7)
    x = rng.normal(4.0, 0.05, 500)
    x[10] = 0.0
    x[200] = 12.0
    assert np.std(x) > 5 * C.robust_noise(x)


def test_判据自检能复原一个已知内阻():
    """端到端（合成）：`V = OCV − R·I`，回归必须把 `R` 收回来，且判成**可辨识**。"""
    rng = np.random.default_rng(3)
    t = np.arange(0.0, 120.0, 0.2)
    current = 3.0 + 1.5 * np.sin(t / 7.0) + rng.normal(0, 0.02, t.size)
    r_true = 0.085
    voltage = 21.5 - r_true * current + rng.normal(0, 0.002, t.size)
    sig = C.esr_and_signature(t, current, voltage, dt=1.0, min_n=3)
    assert sig["verdict"] == "visible-in-voltage"
    assert sig["fit_is_identifying"] is True
    assert abs(sig["r_int_ohm"] - r_true) < 0.01, sig["r_int_ohm"]
    assert sig["corr_binned"] < -0.9


def test_判据自检在电流与电压无关时不认那点耦合():
    """★★ 第三类静默失败（机理反了）的守卫。

    造一份 `I` 有大幅抖动、而 `V` **与它完全无关**的数据（只是各自在漂）。
    若工具在这种数据上报出"电压里看得见这个抖"，那它认的不是物理，是噪声 ——
    而报告看上去会完全正常。

    ⚠️ 这里**同时**钉住另一半：拟合出来的那个 `R` 必须被标成**不可辨识**。
    只测 verdict 的话，把 `b` 直接当内阻用（真日志里会给出 90–200 mΩ 这种
    "看起来很像内阻"的数）这条就漏掉了。
    """
    rng = np.random.default_rng(11)
    t = np.arange(0.0, 120.0, 0.2)
    current = 3.0 + 2.0 * rng.normal(0, 1.0, t.size)          # 大抖动
    voltage = 21.5 + 0.002 * rng.normal(0, 1.0, t.size)        # 完全独立
    sig = C.esr_and_signature(t, current, voltage, dt=1.0, min_n=3)
    assert sig["verdict"] == "not-visible-in-voltage", (
        "对着与电压无关的电流报出了「电压里看得见」 —— 这条自检认的是噪声，不是电池"
    )
    assert sig["signature_ratio"] > 10.0, sig["signature_ratio"]
    assert sig["fit_is_identifying"] is False, (
        "拟合出非零 R 却没标成不可辨识 ⟹ 那个数会被当测量值引用"
    )


def test_单点口径不依赖那条判据():
    """★ 观察与决定**分开**：`verdict` 说"电压里看不看得见"，`policy` 说我们怎么做。

    这条防的是"把骑线的判据当成决定"：实测比值落在 2.6–3.4×，
    换成"看得见"那一档时**口径不许跟着变** —— 两种解释下单点都是错的统计量。
    """
    rng = np.random.default_rng(3)
    t = np.arange(0.0, 120.0, 0.2)
    # A：电流的样本级抖动很小、电压安静 ⟹ 没有"抖"要找 ⟹ visible
    a = C.esr_and_signature(
        t, 3.0 + 1.5 * np.sin(t / 7.0) + rng.normal(0, 0.02, t.size),
        21.5 - 0.085 * (3.0 + 1.5 * np.sin(t / 7.0)) + rng.normal(0, 0.002, t.size),
        dt=1.0, min_n=3)
    # B：电流抖得厉害、电压照样安静 ⟹ 下界印记远大于实测抖动 ⟹ not-visible
    b = C.esr_and_signature(
        t, 3.0 + 2.0 * rng.normal(0, 1.0, t.size),
        21.5 + 0.002 * rng.normal(0, 1.0, t.size), dt=1.0, min_n=3)
    assert a["verdict"] != b["verdict"], "两种数据给了同一个观察 ⟹ 这个用例没鉴别力"
    assert a["verdict"] == "visible-in-voltage" and b["verdict"] == "not-visible-in-voltage"
    assert a["policy"] == b["policy"], "口径跟着观察变了 —— 那判据就成了决定"
    assert "单点永不作为判据" in a["policy"]


def test_分箱后样本太少就给no_data():
    t = np.array([0.0, 0.2, 0.4, 0.6])
    sig = C.esr_and_signature(t, np.full(4, 3.0), np.full(4, 21.0), dt=1.0, min_n=3)
    assert sig["verdict"] == "no-data" and sig["r_int_ohm"] is None


# ------------------------------------------------------------------ 列判据（哪些能当判据）


def test_全零单节电压判缺席_但非零常量判在场():
    """★ 判"缺席"看的是**值全零**，不是"字段在不在"。

    两个方向都要钉：z0 的模拟电源模块把 `voltage_cell_v[*]` 填 0（字段在、读数不在）；
    而**非零常量**（比如恒 3.85 V）是"字段在且真有读数"——不能也判成缺席。
    只测一个方向的话，把判据写成 `verdict = "absent"`（永真）也能过。
    """
    n = 100
    zero = np.zeros((n, 6))
    real = np.full((n, 6), 3.85)
    v_zero = C.channel_verdicts({"voltage_cell_v": {"present": True, "values": zero}})["voltage_cell_v"]
    v_real = C.channel_verdicts({"voltage_cell_v": {"present": True, "values": real}})["voltage_cell_v"]
    assert v_zero["verdict"] == "absent"
    assert v_real["verdict"] == "usable", "非零的单节电压被误判成缺席 ⟹ 判据写成了永真"


def test_电流平均列永远不是判据():
    """`current_average_a` 是慢滤波（初值 `BAT_AVRG_CURRENT`），不是测量。

    即使它长得像"平滑得很好的电流"，也不能当判据 —— 它的滞后与初值都不受控。
    """
    v = C.channel_verdicts({"current_average_a": {"present": True,
                                                  "values": np.full(100, 15.0),
                                                  "bat_avrg_current": 15.0}})["current_average_a"]
    assert v["verdict"] == "not-a-measurement"


def test_scale不是电流标定():
    """★ 票面自己写错了这一条（"`scale` 为 0 说明没标定"）—— 这里把订正钉住。

    `scale = v_charged / bat_v` 是**推力补偿系数**（`Battery::computeScale`，夹在 [1.0, 1.3]），
    电流的标定在 `BAT1_A_PER_V`。拿 `scale` 判"电流有没有标定"会判错。
    """
    v = C.channel_verdicts({"scale": {"present": True, "values": np.full(100, 1.15)}})["scale"]
    assert v["verdict"] == "not-a-measurement"
    assert "BAT1_A_PER_V" in v["why"]


# ------------------------------------------------------------------ 外推


def test_功率外推按P正比T的n次方():
    p = C.extrapolate_power(80.0, 7.355, 11.8, n=1.5)
    assert abs(p - 80.0 * (11.8 / 7.355) ** 1.5) < 1e-9
    # 推力翻倍 ⟹ 功率 ×2^1.5（而不是 ×2、也不是 ×4）
    assert abs(C.extrapolate_power(80.0, 10.0, 20.0, n=1.5) - 80.0 * 2 ** 1.5) < 1e-9


def test_电压下沉方向对_且电流升压降():
    """电流升 ⟹ 端电压**降**。方向搞反的话"低电量端"的结论会整体反号。"""
    assert C.voltage_after_sag(22.0, 3.6, 8.0, 0.09) < 22.0
    assert abs(C.voltage_after_sag(22.0, 3.6, 3.6, 0.09) - 22.0) < 1e-12
    a = C.voltage_after_sag(22.0, 3.6, 8.0, 0.09)
    b = C.voltage_after_sag(22.0, 3.6, 8.0, 0.20)
    assert b < a, "内阻越大压降应当越大"


def test_判据用的是占额定的比例():
    """★ 判据写成**比例**（占额定电流多少），因为电流有一条未标定的标度：

    若 `BAT1_A_PER_V` 整体错了 k 倍，分子分母**同时**错 k 倍 ⟹ 比例不变。
    绝对安培没有这个性质 —— 所以判据必须带 `premise` 字段写明它依赖什么。
    """
    v = C.throughput_verdict(11.8, i_at_f_max=7.5, bat_current_rating=15.0)
    assert abs(v["fraction_of_rating"] - 0.5) < 1e-12
    assert v["verdict"] == "conservative"
    assert "BAT1_A_PER_V" in v["premise"]
    # 标度整体错 2 倍：比例不变（这正是选它当判据的理由）
    v2 = C.throughput_verdict(11.8, i_at_f_max=15.0, bat_current_rating=30.0)
    assert abs(v2["fraction_of_rating"] - v["fraction_of_rating"]) < 1e-12


def test_判据有阈值而不是一句印象():
    """三段阈值（<75% 保守 / 75–100% 吃紧 / >100% 供不出），中间不许出现"大概吧"。

    ⚠️ 初版这里写的是二值（"not-conservative"）—— 是**测试写错了**，不是代码：
    代码给的是三段，而"吃紧"这一档是真实存在的一档（`f_max` 的出处 §6.2
    推出来恰好就是 ~100%），把它塞进"不保守"里会把票面问题 ② 的答案说糊涂。
    """
    v = C.throughput_verdict(11.8, i_at_f_max=7.0, bat_current_rating=15.0)
    assert v["verdict"] == "conservative" and abs(v["fraction_of_rating"] - 7 / 15) < 1e-12
    assert C.throughput_verdict(11.8, i_at_f_max=13.0, bat_current_rating=15.0)["verdict"] == "marginal"
    assert C.throughput_verdict(11.8, i_at_f_max=14.0, bat_current_rating=15.0)["verdict"] == "marginal"
    assert C.throughput_verdict(11.8, i_at_f_max=16.0, bat_current_rating=15.0)["verdict"] == "infeasible"
    assert C.throughput_verdict(11.8, i_at_f_max=None, bat_current_rating=15.0)["verdict"] == "unknown"


def test_全段最大落在在飞窗口起点要点名():
    """★ 票面问 ① 时**点名排除**了「起飞那一瞬的最大电流」，可"全段最大"这个统计量
    天生往负载最高的那一瞬跑 —— 归档里 8 份有 4 份正好落在解锁瞬间。这条判据把这件事
    标出来，免得读者把那一行直接当成问 ① 的答案（一次复核就是这么读的）。

    变异：把 `<=` 写成 `>`、或去掉 `window_s` 这一项，本用例立刻红。
    它区分的是"落在**头一个窗口**内"与"落在后面某个窗口"，不是"有没有数"。
    """
    s = {"value": 5.0, "at_s": 110.2}
    assert C.sustained_lands_at_flight_start(s, 110.2, 1.0) is True    # 恰在起点
    assert C.sustained_lands_at_flight_start(s, 109.5, 1.0) is True    # 头一个窗口内
    assert C.sustained_lands_at_flight_start(s, 100.0, 1.0) is False   # 落在后面
    # 边界：`at_s - t0` 恰好等于窗长时，仍算"落在头一个窗口里"
    assert C.sustained_lands_at_flight_start({"at_s": 111.2}, 110.2, 1.0) is True
    # 没量到 ⟹ `None`，**不是** `False`（"没量到"与"量到不在起点"不是一回事）
    assert C.sustained_lands_at_flight_start({"at_s": None}, 110.2, 1.0) is None
    assert C.sustained_lands_at_flight_start(None, 110.2, 1.0) is None
    assert C.sustained_lands_at_flight_start({"at_s": 110.2}, None, 1.0) is None


CASES = [v for k, v in sorted(globals().items()) if k.startswith("test_")]


def main() -> int:
    if "--self-test" not in sys.argv:
        print(__doc__)
        return 0
    passed, failed = 0, 0
    for fn in CASES:
        try:
            fn()
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"❌ {fn.__name__}\n     {type(exc).__name__}: {exc}")
        else:
            passed += 1
            print(f"✅ {fn.__name__}")
    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
