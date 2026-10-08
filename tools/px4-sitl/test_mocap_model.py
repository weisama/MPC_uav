"""B 档夹具那两件仪器的离线单测（ticket 11）—— 不读包、不起 SITL。

被测的是两层：

  · `mocap_model.py` 的**纯函数**（量包用的：间隔/延迟/高频残差/窗口）
  · `mocap_replay.py` 的**时间轴与插值**（`TruthBuffer`、`nlerp`）

**为什么值得单测**：这两件仪器是**夹具的夹具** —— 判据 1/3/4 的读数全部从它们过。
仪器错了，控制器会**带着错数一起通过**（这正是票面否决"自洽 Python 机型"的同一个理由：
夹具与被测共享一个错时，测试一起错并且通过）。所以这里喂人造数，把几条**口头约定**
钉成断言：

| 钉住的约定 | 为什么它必须成立 |
|---|---|
| 时间轴不修 | `stamp` 倒退是源的毛病，读数要如实反映，不能悄悄排序 |
| 延迟**不夹到端点** | 取不到就**别猜** —— 端点上"凑一帧"会伪造出一段真实里不存在的零延迟 |
| 四元数插值先对齐符号 | `q` 与 `−q` 是同一姿态；不对齐会插出**反方向绕远路**，姿态看着没错、转过的角是错的 |
| 常量/线性信号的高频残差为 0 | 否则"噪声 σ"里会混进**趋势**，噪声被人为放大 |

（`mocap_model.py` 导入期只要 numpy，所以前半段不 skip；后半段要 `rclpy`。）
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent


def _load(name: str):
    """按**路径**导入被测脚本（`tools/` 不是一个包）。

    与 `test_verify_judges._load` 同一条理由：先塞 `sys.modules` 再 `exec_module`。
    `mocap_model.py` 里有 `@dataclass`，不这么做会回表找不到模块对象。

    ★ **执行失败必须把半成品从 `sys.modules` 里摘掉**：不摘的话下一次 `_load` 拿到的
      是那个"建了对象但没跑完"的模块，报出来的是 `has no attribute '…'`
      —— 一个把人往"函数名写错了"上带的假象（本次真身是 `px4_msgs` 没 source 到）。
    """
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, HERE / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    try:
        spec.loader.exec_module(mod)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return mod


mm = _load("mocap_model")


# ────────────────────────────────────────────── 时序


def test_intervals_不修倒退的帧():
    """`stamp` 倒退（源的问题）**原样留在读数里** —— 排序会让 `n_backwards` 永远是 0，
    "这条流有没有倒着走"就再也量不出来了。"""
    t = np.array([0.0, 0.01, 0.005, 0.02])
    np.testing.assert_allclose(mm.intervals_s(t), [0.01, -0.005, 0.015])


def test_interval_stats_逐项对得上():
    r = mm.interval_stats([0.001, 0.002, 0.003, 0.100], gap_ms=20.0)
    assert r["n"] == 4
    assert r["mean_ms"] == pytest.approx(26.5)
    assert r["p50_ms"] == pytest.approx(2.5)
    assert r["max_ms"] == pytest.approx(100.0)
    assert r["n_gaps"] == 1                      # 只有 100 ms 那条越过 20 ms
    assert r["gap_total_ms"] == pytest.approx(100.0)
    assert r["n_backwards"] == 0


def test_interval_stats_空输入不说假话():
    assert mm.interval_stats([]) == {"n": 0}


def test_interval_stats_缺口门槛是严格大于():
    """正好等于门槛的**不算**丢帧 —— 门槛是"异常"的下界，边界不给它记一笔。"""
    r = mm.interval_stats([0.020], gap_ms=20.0)
    assert r["n_gaps"] == 0


def test_lag_stats_逐项对得上():
    stamp = np.array([10.000, 10.010, 10.020])
    recv = np.array([10.005, 10.013, 10.040])
    r = mm.lag_stats(stamp, recv)
    assert r["n"] == 3
    assert r["p50_ms"] == pytest.approx(5.0)
    assert r["mean_ms"] == pytest.approx((5 + 3 + 20) / 3)
    assert r["max_ms"] == pytest.approx(20.0)
    assert r["min_ms"] == pytest.approx(3.0)


def test_lag_stats_空输入不说假话():
    assert mm.lag_stats(np.array([]), np.array([])) == {"n": 0}


def test_track_window_按迁移推且含退出时刻():
    st = [(0.0, "MANUAL/ARM"), (26.6, "AUTO/TRACK"), (86.6, "MANUAL/LAND")]
    assert mm.track_window(st) == (26.6, 86.6)


def test_track_window_没收尾就用最后一帧():
    """跑到记录结束还在 TRACK（操作者直接拔了）—— 用最后一帧收尾，**不返回 None**：
    返回 None 会让调用方退回"整段"，把地面静止段也当成飞行段量噪声。"""
    st = [(1.0, "AUTO/TRACK"), (60.0, "AUTO/TRACK")]
    assert mm.track_window(st) == (1.0, 60.0)


def test_track_window_从没走到就是_None():
    assert mm.track_window([(0.0, "MANUAL/ARM"), (5.0, "MANUAL/LAND")]) is None


def test_track_window_只认第一次进入():
    """出去又回来 ⟹ 取**第一段**。第二段通常是重接，与第一段之间夹着一段非 TRACK，
    把它们并成一段会把中间那段拿别的东西当参考的飞行也量进去。"""
    st = [(1.0, "AUTO/TRACK"), (5.0, "MANUAL/POSCTL"), (9.0, "AUTO/TRACK")]
    assert mm.track_window(st) == (1.0, 5.0)


def test_slice_window_闭区间():
    t = np.array([0.0, 1.0, 2.0, 3.0])
    np.testing.assert_array_equal(mm.slice_window(t, 1.0, 2.0), [False, True, True, False])


# ────────────────────────────────────────────── 噪声


def test_高频残差_常量信号为零():
    """★ 这条不过，`noise_sigma` 量的就是"飞机有没有在动"而不是噪声。"""
    t = np.linspace(0.0, 2.0, 200)
    x = np.full((200, 3), 1.234)
    assert np.abs(mm.hf_residual(x, t, 0.2)).max() < 1e-12


def test_高频残差_线性趋势在内部被扣干净():
    """中心滑动均值对**线性**信号是无偏的（窗内均值 = 窗心值）⟹ 残差在内部 ≈ 0。
    端点必然有边（窗被截断），所以只看内部。"""
    t = np.linspace(0.0, 2.0, 400)
    x = (0.7 * t + 0.3).reshape(-1, 1)
    r = mm.hf_residual(x, t, 0.2)
    assert np.abs(r[50:-50]).max() < 1e-12


def test_高频残差_尖峰扣不掉():
    """扣掉慢变趋势**不等于**把什么都扣掉：单帧跳变必须留在残差里。"""
    t = np.linspace(0.0, 2.0, 400)
    x = np.zeros((400, 1))
    x[200, 0] = 1.0
    r = mm.hf_residual(x, t, 0.2)
    assert r[200, 0] > 0.9


def test_noise_sigma_白噪声量得回来():
    rng = np.random.default_rng(7)
    t = np.linspace(0.0, 4.0, 2000)
    x = rng.normal(0.0, 0.01, size=(2000, 3))
    s = mm.noise_sigma(x, t, 0.2)
    assert s.shape == (3,)
    np.testing.assert_allclose(s, 0.01, rtol=0.1)


def test_noise_sigma_随信号幅度线性放大():
    """σ 是**尺度**量：信号乘 3，σ 就该乘 3（不这么做说明窗口或归一化写错了）。"""
    rng = np.random.default_rng(11)
    t = np.linspace(0.0, 4.0, 2000)
    x = rng.normal(0.0, 0.01, size=(2000, 2))
    base = mm.noise_sigma(x, t, 0.2)
    np.testing.assert_allclose(mm.noise_sigma(3.0 * x, t, 0.2), 3.0 * base, rtol=1e-9)


def test_量到的σ随窗口变宽单调变大_口径得连着窗宽一起报():
    """★ **方向是实测出来的，不是推出来的**（这台仪器上先写反过一次）：

        随机游走（含慢变漂移），窗 0.05 / 0.2 / 0.5 s
        ⟹ σ = 1.48e-3 / 2.91e-3 / 4.18e-3   —— **单调变大**

    机理是两条同向叠加：窗越宽，扣掉的高频越少（残差 ≈ 真噪声，从下方逼近）；同时
    端点被截断的"边"越大（窗内均值不再等于窗心值）。⟹ `--win-s` 不是无关紧要的旋钮，
      **报 σ 时必须把窗宽一起报**，否则这个数没有意义。
    """
    rng = np.random.default_rng(3)
    t = np.linspace(0.0, 4.0, 2000)
    x = np.cumsum(rng.normal(0.0, 1e-3, size=(2000, 1)), axis=0)   # 含慢变漂移
    s = [mm.noise_sigma(x, t, w)[0] for w in (0.05, 0.2, 0.5)]
    assert s[0] < s[1] < s[2]


def test_线性趋势的残差只在端点不为零_且随窗宽变大():
    """端点效应**是新加的窗宽旋钮的主要代价**：线性信号在窗内均值 = 窗心值（内部残差 0），
    但窗口被数据边界截断时均值偏到一边 ⟹ 端点残差 ≈ 斜率 × 半个窗宽。

    ★ 这条同时是**用法的护栏**：`--win-s` 必须远小于被量的段长（默认 0.2 s 对 60 s 的
      TRACK 段 ⟹ 端点占比 ~0.7%，可忽略；窗开到 5 s 就不行了）。

    边界的宽度**正比于窗宽**（半个窗被截断时就偏），所以这里的余量写 300 而不是写个
    看着够大的数 —— 4 s / 2000 点上 0.5 s 的窗意味着 125 个点都算"端点区"。
    """
    t = np.linspace(0.0, 4.0, 2000)
    x = (0.7 * t + 0.3).reshape(-1, 1)
    for win in (0.05, 0.5):
        r = mm.hf_residual(x, t, win)
        assert np.abs(r[300:-300]).max() < 1e-9          # 端点区之外干净
        assert abs(r[0, 0]) > 1e-3                       # 端点有边
    assert abs(mm.hf_residual(x, t, 0.5)[0, 0]) > abs(mm.hf_residual(x, t, 0.05)[0, 0])


# ────────────────────────────────────────────── 回放节点的时间轴与插值（要 rclpy）


def _load_replay():
    """回放节点在导入期就 `import px4_msgs` ⟹ 两样都得在（工作空间要 source 齐）。"""
    pytest.importorskip("rclpy")
    pytest.importorskip("px4_msgs")
    return _load("mocap_replay")


def test_nlerp_同向就是对半分():
    mr = _load_replay()
    q = mr.nlerp(np.array([0.0, 0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0, 0.0]), 0.5)
    np.testing.assert_allclose(q, [0.0, 0.0, 2 ** -0.5, 2 ** -0.5], atol=1e-12)


def test_nlerp_反向符号先对齐():
    """★ 这条是整个插值里最会出错的一处：`[1,0,0,0]` 与 `[−1,0,0,0]` 是**同一个姿态**。
    不对齐就线性插值，中点得到 `[0,0,0,0]` —— 一个**不是旋转**的东西，
    归一化会把它炸成 NaN（或退回 q0，看着"没坏"）。"""
    mr = _load_replay()
    q = mr.nlerp(np.array([1.0, 0.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0, 0.0]), 0.5)
    assert np.isfinite(q).all()
    np.testing.assert_allclose(np.abs(q), [1.0, 0.0, 0.0, 0.0], atol=1e-9)


def test_nlerp_端点原样返回():
    mr = _load_replay()
    q0 = np.array([0.1, 0.2, 0.3, 0.9])
    q0 = q0 / np.linalg.norm(q0)
    np.testing.assert_allclose(mr.nlerp(q0, np.array([0.0, 0.0, 0.0, 1.0]), 0.0), q0, atol=1e-12)


def test_缓冲_样本不够就说没有():
    mr = _load_replay()
    b = mr.TruthBuffer()
    assert b.sample_at(0.0)[0] is None
    b.push(1.0, [0, 0, 0], [0, 0, 0, 1])
    assert b.sample_at(1.0)[0] is None           # 只有一帧，插不出来


def test_缓冲_窗口外不夹端点():
    """★ 取不到就**别猜**。夹到端点等于给出一帧"其实已经超出缓冲"的读数，
    在延迟实验里会伪造出**凭空少掉的延迟**（判据 3 要测的就是延迟，这条最不能错）。"""
    mr = _load_replay()
    b = mr.TruthBuffer()
    for i, t in enumerate([0.0, 1.0, 2.0]):
        b.push(t, [i, 0, 0], [0, 0, 0, 1])
    assert b.sample_at(-0.5)[0] is None
    assert b.sample_at(2.5)[0] is None
    assert b.sample_at(1.5)[0] is not None


def test_缓冲_中点插出平均值():
    mr = _load_replay()
    b = mr.TruthBuffer()
    b.push(0.0, [0.0, 0.0, 0.0], [0, 0, 0, 1])
    b.push(1.0, [2.0, -4.0, 6.0], [0, 0, 0, 1])
    p, q, dt = b.sample_at(0.5)
    np.testing.assert_allclose(p, [1.0, -2.0, 3.0])
    assert dt == pytest.approx(0.5)              # 用到的那一段"有多旧"
    np.testing.assert_allclose(np.abs(q), [0, 0, 0, 1], atol=1e-12)


def test_缓冲_只留窗口内的旧帧():
    """缓冲是**有限**的：延迟实验里取的就是"多久之前那一帧"，不清理就是无界内存。"""
    mr = _load_replay()
    b = mr.TruthBuffer(span_s=1.0)
    for i in range(6):
        b.push(float(i), [i, 0, 0], [0, 0, 0, 1])
    assert b.stats()["n"] == 2                   # 只剩 t=4,5
    assert b.sample_at(3.0)[0] is None           # 已经清掉了 ⟹ 不猜


def test_缓冲_倒着推的旧帧也要量得出来():
    """延迟 D 越大，取到的 `t_target` 越早；`sample_at` 返回的第三项是
    "实际用的那一帧比目标**新**多少" —— 它的量级必须是**插值步长**，不能是"整个延迟"。
    判成"整个延迟"会让日志里的数字看着像延迟没生效。"""
    mr = _load_replay()
    b = mr.TruthBuffer()
    for i in range(10):
        b.push(i * 0.1, [0, 0, 0], [0, 0, 0, 1])
    _, _, dt = b.sample_at(0.5)
    assert abs(dt) <= 0.1 + 1e-9


# ────────────────────────────────────────────── 姿态分量序（NED/FRD → ENU/FLU）
#
# ☠☠ 2026-09-29 的 B 档首飞就栽在这一族里：夹具把 `px4_pose_to_enu` 的 **wxyz** 输出
#     当成 xyzw 写进了 `PoseStamped.orientation`，水平机的姿态于是变成"绕 x 转 −90°"，
#     而参考姿态是单位四元数 ⟹ **姿态误差满 90°**。控制器整场都在"把滚转扳回来"，
#     代价是把推力丢到下限：真值全程 `z ≈ 0`（没离地）、`F_min` 饱和 1543/1544 次、
#     真值倾角 0.00°。**症状指向控制器，真身是夹具。**
#
#     `px4_quat_to_enu` 本身没错（下面第 2 条用矩阵恒等式钉住它），错在它两侧的**序**。
#     两条错都在"不报错、不 NaN、只是姿态偏 90°"这一类里 —— 只有恒等式抓得到。


def _R_from_wxyz(w, x, y, z):
    """`wxyz` 四元数 → 旋转矩阵（与 `nmpc` 无关的**独立**实现：被测与被验不能同源）。"""
    n = (w * w + x * x + y * y + z * z) ** 0.5
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


#: 世界换标签 NED→ENU：把 NED 分量读成 ENU 分量，`[n, e, d] → [e, n, −d]`。
_L_NED_TO_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
#: 机体换标签 FRD→FLU：`(x, y, z) → (x, −y, −z)`。
_M_FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])

#: 水平停机（NED/FRD 下就是单位四元数）、抬头 30°、右滚 20°、偏航 45°、以及一个任意姿态。
_ATTITUDES = [
    ("水平", (1.0, 0.0, 0.0, 0.0)),
    ("抬头30", (np.cos(np.pi / 12), 0.0, np.sin(np.pi / 12), 0.0)),
    ("右滚20", (np.cos(np.pi / 18), np.sin(np.pi / 18), 0.0, 0.0)),
    ("偏航45", (np.cos(np.pi / 8), 0.0, 0.0, np.sin(np.pi / 8))),
    ("任意", tuple(np.array([0.5, -0.5, 0.5, 0.5]))),
]


def test_姿态换算_与矩阵恒等式逐条一致():
    """★★ 这是 `px4_quat_to_enu` 的**定义式**：`R_enu = L · R_ned · M`。

    用两侧各自独立的实现（左边走夹具的函数、右边自己搭矩阵）对撞 ——
    只比"输出长什么样"是抓不到分量序错的：序错了，输出依然是个**合法**四元数。
    """
    mr = _load_replay()
    for name, q_ned in _ATTITUDES:
        _p, q_flu, _v = mr.ned_frd_to_enu_flu((0.0, 0.0, 0.0), q_ned)
        got = _R_from_wxyz(*q_flu)
        want = _L_NED_TO_ENU @ _R_from_wxyz(*q_ned) @ _M_FRD_TO_FLU
        np.testing.assert_allclose(got, want, atol=1e-12,
                                   err_msg=f"{name}：换算出来的姿态与 L·R·M 不等")


def test_水平机_ENU下是绕z的90度_不是绕x的90度():
    """☠☠ 首飞那个错的确切形状。

    水平停在 NED/FRD 下的姿态是单位四元数；换成 ENU/FLU 之后**机头仍朝上**
    （`R[2,2] = +1`，即与竖直夹角 0°），差的只是一个 90° 的**偏航**。
    错成"绕 x 转 90°"时 `R[2,2] = 0` ⟹ 飞机在控制器眼里**侧躺着**，
    而位置/姿态真值都显示它是平的 —— 这个断言就是拿"机头朝上"这条物理事实当尺子。
    """
    mr = _load_replay()
    _p, q_flu, _v = mr.ned_frd_to_enu_flu((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    R = _R_from_wxyz(*q_flu)
    assert R[2, 2] == pytest.approx(1.0, abs=1e-12), "机体的 +z 不指向世界的 +z ⟹ 不是水平"
    # 只剩偏航：绕 z 转 ±90°（`q` 与 `−q` 是同一姿态，所以只看 `|z|`）
    np.testing.assert_allclose([abs(q_flu[0]), q_flu[1], q_flu[2], abs(q_flu[3])],
                               [2 ** -0.5, 0.0, 0.0, 2 ** -0.5], atol=1e-12)


def test_发布出去的消息读回来还是同一个姿态():
    """★★ 消息边界那一次换序（`orientation_xyzw`）。

    `PoseStamped.orientation` 的字段是 x, y, z, w，而我们的四元数一路是 wxyz。
    这条把"按字段序发出去、再按字段序读回来"走完，断言**物理姿态没变** ——
    收端（节点）读的就是这四个字段，所以这是"线上到底是什么"的最直接证据。
    """
    mr = _load_replay()
    for name, q_ned in _ATTITUDES:
        _p, q_flu, _v = mr.ned_frd_to_enu_flu((0.0, 0.0, 0.0), q_ned)
        ox, oy, oz, ow = mr.orientation_xyzw(q_flu)          # 等于 msg 的四个字段
        q_back_wxyz = (ow, ox, oy, oz)                       # 收端读回来
        np.testing.assert_allclose(_R_from_wxyz(*q_back_wxyz), _R_from_wxyz(*q_flu),
                                   atol=1e-12, err_msg=f"{name}：消息往返后姿态变了")
        # 再钉一次"往返之后仍是水平的"（水平那条输入）
        if name == "水平":
            assert _R_from_wxyz(*q_back_wxyz)[2, 2] == pytest.approx(1.0, abs=1e-12)


# ────────────────────────────────────────────── 参考那一路的三段口径（判据② 的八字）


def test_参考_z偏移只加在位置上():
    """★ 两条规则各自都能**静默**出错，所以分开钉。

    `--ref-z-offset-m` 是"参考高度不在包里"（实测包 4 `z ≡ 0`、包 3 恒为 0）的补丁，
    DUT 侧**没有别的旋钮**可用（`reference_source=traj` 拒收非零 `hold_offset`）。
    加错地方的后果：加到 `velocity.z` 上 = 凭空给了一个爬升速度（飞机一直爬）；
    加到 `x`/`y` 上 = 高度根本没动、参考斜着飘。两种都不报错。
    """
    mr = _load_replay()
    pos, vel, _yaw = mr.reference_fields((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), 0.0, "bag", 1.0)
    assert pos == (1.0, 2.0, 4.0), "z 偏移该只叠在 position.z 上"
    assert vel == (4.0, 5.0, 6.0), "速度不该被偏移碰"
    # 偏移为 0 时（`hold`/直通的用法）逐字等于包里那帧
    assert mr.reference_fields((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), 0.0, "bag", 0.0)[0] == (1.0, 2.0, 3.0)


def test_参考_pre段速度置零_bag与hold段照抄():
    """★★ `pre` 段（顶住首点）速度置 0 是**全链路唯一一处不照抄**包里的值。

    为什么必须置 0：包里首帧带着八字起点的前馈 `v = (0.354, 0.354)`（|v| = 0.5 m/s 斜着），
    而 `pre` 段每拍都把位置钉回同一点 ⟹ 照抄就发出"位置不动、速度却在走"的**自相矛盾**帧，
    而 `traj_reference` 拿"位置按 `v·t` 走"当参考自洽的判据 —— 矛盾会变成起飞阶段一直被斜着拽。
    `pre` 段本来就是夹具加的（真实那场参考随 TRACK 一起开始，没有前摇），所以取"站住"的字面意思。

    ⚠️ 但**别把它推广到 `hold`/`bag`**：包 4 的尾巴本来就是 `p ≡ 0` 且 `v ≡ 0`
    （实测帧 305..1028），照抄就是对的。这条断言两面都钉住 ——
    只钉 `pre` 的话，"干脆全段置 0"会把八字整条的速度前馈也抹掉，而那是**照抄**的核心。
    """
    mr = _load_replay()
    v = (0.3536, 0.3535, 0.0)
    assert mr.reference_fields((0.0, 0.0, 0.0), v, 0.0, "pre")[1] == (0.0, 0.0, 0.0)
    for phase in ("bag", "hold"):
        assert mr.reference_fields((0.0, 0.0, 0.0), v, 0.0, phase)[1] == v, f"{phase} 段该照抄速度"


def test_参考_yaw原样带过去():
    """掩码 2496 下 `yaw` 是**有效轴**（只忽略 `IGNORE_YAW_RATE` 那一位置）⟹ 必须带过去。

    自己起一个 yaw（比如取 0）会静默改掉机头指令；包 4 的 yaw 恰为 0，
    所以"拿 0 顶替"在那一条包上**看不出来** —— 这正是要用人造数钉它的理由。
    """
    mr = _load_replay()
    for yaw in (0.0, 0.7, -1.2):
        assert mr.reference_fields((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), yaw, "bag")[2] == yaw
