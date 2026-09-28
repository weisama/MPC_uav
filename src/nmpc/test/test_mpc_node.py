"""seam 1（**话题上）的集成测试 —— 起真的 DUT 子进程 + 假 PX4 夹具（ticket 06）。

层次与另外两组测试的分工（别混）：

| 文件 | 被测的接缝 | 起 DUT 吗 |
|---|---|---|
| `test_node_core.py` | 闸门/外推/参考（纯逻辑，零 ROS） | 不起 |
| `test_ocp_*.py` | `(状态, 参考) → u`（零 ROS 的求解器） | 不起 |
| **本文件** | **"话题上出现了什么"** —— QoS、心跳、闸门、符号、单位 | **起**（子进程） |

★★ 本文件里所有的断言都**只看外部可观测的东西**（话题、QoS、时间戳、进程退出码），
  不 import 节点的类、不读它的成员 —— 这是 seam 1 的定义（`.scratch/phase1/spec.md`）。
  代价是几条验收项**在这一层根本量不了**，那几条**不写成假测试**，而是写成"为什么量不了"
  的一手结论（见本文件最后两条用例：`depth 1` 与 τ）。

★ 外部依赖（缺了会**静默**地变成"夹具什么都没测到"，见 `scenario.py` 的 `_resolve_dut`）：
  · `nmpc` 包已 `colcon build` 且 `install/setup.bash` 已 source（`lib/nmpc/mpc_node.py` 要在）
  · `bench` 包已装（夹具本身）
  · `px4_msgs` 工作区已 source
  · acados：**DUT 子进程**要 `ACADOS_SOURCE_DIR`/`LD_LIBRARY_PATH`（`_dut_env()` 会补上
    `~/acados` 这一档默认值）；本进程要 import `nmpc` 也走同一条路。

★★ **别并发跑，也别把 DUT 落在后台**（第一手，2026-09-28）：DUT 的话题名是**全固定**的
  （`/fmu/in/*`），ROS 域默认 73 也是全固定 —— 于是任何一个"上一次没杀干净的 mpc_node"
  都会和本文件的用例**同时**往同两条话题上发，夹具的记录里就混进了别人的帧
  （症状是"帧数对不上/频率翻倍"，而断言本身照样绿或红得莫名其妙）。
  同一时刻它们还共用 `$MPC_CODEGEN_DIR` 那个目录（560 KB 的生成代码）——
  两个进程同时判"要重新生成"会撞在一起，报一个与代码无关的
  `RuntimeError: Rendering of model.in.h failed!`（`t_renderer` 退出码非 0）。
  收工前 `pgrep -af mpc_node` 看一眼；`pytest -n`（xdist）也不要用在这个文件上。
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest
from px4_msgs.msg import VehicleStatus
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from bench import Scenario
from bench.fake_px4 import NAV_STATE_OFFBOARD
from bench.timeline import px4_arm, px4_nav_state, px4_status_silence
from nmpc.params import PITCH_RATE_MAX, Z0Params

#: session 级的 ROS 上下文（在 `test/conftest.py` 里）—— 本模块的每条用例都要起 DUT
pytestmark = pytest.mark.usefixtures("ros_context")

#: DUT 的定位：`("包名", "可执行名")` ⟹ `Scenario._resolve_dut` 去 ament 的 prefix 下找
#: `lib/nmpc/mpc_node.py`（由 CMakeLists 的 `install(PROGRAMS …)` 产生）。
#: ⚠️ 名字带 `.py`：`install(PROGRAMS …)` **不改名**，与 05 的 `offboard_smoke.py` 同一形态
#:   （`ros2 run nmpc mpc_node.py`）。
DUT = ("nmpc", "mpc_node.py")

#: 等 DUT 出现的最长时间。★ 给得宽是**有理由**的：`OfflineOcp()` 首次会代码生成 + 编译 C
#: （几十秒，之后复用 `$MPC_CODEGEN_DIR`）—— 卡在 10 s 会让"第一次跑就红、第二次就绿"。
STARTUP_S = 120.0


def _dut_env() -> dict:
    """DUT 子进程的额外环境变量：acados 的**安装目录**。

    ★ 只在外面真的没设时才补 `~/acados`：拿 `ACADOS_SOURCE_DIR` 指源码树是另一个坑
      （`docs/control/acados-orin-build.md` 坑 4），不能覆盖操作者给的值。
    """
    if os.environ.get("ACADOS_SOURCE_DIR"):
        return {}
    root = Path.home() / "acados"
    if not (root / "lib").is_dir():
        return {}
    return {"ACADOS_SOURCE_DIR": str(root),
            "LD_LIBRARY_PATH": f"{root / 'lib'}:{os.environ.get('LD_LIBRARY_PATH', '')}"}


def _scenario(**params) -> Scenario:
    """一个 Phase 2 的场景：假 PX4 + 脚本化动捕 + 真的 DUT 子进程。"""
    return Scenario(dut=DUT, px4=True, mocap=True, params=params, dut_env=_dut_env())


def _wait_dut_up(sc: Scenario, timeout_s: float = STARTUP_S) -> None:
    """等 DUT **真的起来了**：判据是夹具**收到了第一帧心跳**，而不是"发布者被发现"。

    ★★ 第一手（2026-09-28，本机）：**发布者被发现远早于节点初始化完成**。
      实测 `matched_publishers("mode")` 在 ~8 s 就 ≥ 1，而那一刻节点还在 `OfflineOcp()`
      里面（代码生成 + 编 C；同一个进程启动要 ~8 s，见 `STARTUP_S` 的说明），启动日志还
      压在 rcutils 的缓冲里没进管道。此时给它发 SIGINT 的症状是：
      **日志只剩开头那行 matplotlib 警告（373 字符）、`returncode == 0`、没有 traceback**
      —— 看上去像"收尾摘要没打出来"，其实节点压根没走到收尾路径（SIGINT 落在 `__init__`
      里就退出了）。这条判据不成立时，`test_SIGINT_*` 会红，而其余用例照样绿。
    ★ 心跳由 50 Hz 定时器发、`__init__` 全跑完才有第一帧 ⟹ 判据天然排在"初始化完成"之后；
      且**与日志缓冲无关**（帧走 DDS，不走 stdout），这正是它比"匹配上发布者"可靠的地方。
    ★ 用 `Scenario.wait_for`（而不是 `sc.px4.wait_for_publishers`）是为了**失败时拿到证据**：
      `BenchTimeout` 会带上 DUT 的日志尾部 —— "参数名写错、节点启动即退"与"QoS 不兼容、
      永不匹配"在断言里长得一样，但根因完全不同（`scenario.py` 的 `wait_for_subscribers`
      里写着同一条教训）。
    """
    sc.wait_for(lambda: len(sc.px4.published_frames("mode")) >= 1, timeout_s=timeout_s,
                message="DUT 起来之后没在 /fmu/in/offboard_control_mode 上发出第一帧心跳")


def _arm_and_offboard(sc: Scenario) -> None:
    """注入"已解锁 + 已切 OFFBOARD"—— 走 `Timeline`（夹具**唯一**那条注入面）。

    ★ 为什么排进 `timeline` 而不是直接 `sc.px4.set_armed(...)`：票面那条验收项要的是
      "**能按指定时刻注入故障**（为 07 准备好）"。"直接调 setter"是"立刻注入"，
      对 07 那种"跑 3 s 之后让这一拍求解失败"用不上；而排进 timeline 之后，
      **同一套写法**既能在 `t=0` 注入、也能在 `t=+3.0` 注入（`wait_for` / `advance_to`
      在等待期间照常触发事件，见 `Scenario.wait_for` 的 docstring）。
    ★ 两条注入面都会**即刻**发一帧（不等 2 Hz 的定频），所以下面这个等待是毫秒级的。
    """
    sc.timeline.extend(px4_arm(sc.px4, sc.t(), armed=True))
    sc.timeline.extend(px4_nav_state(sc.px4, sc.t(), NAV_STATE_OFFBOARD))
    sc.wait_for(lambda: sc.px4.status_frames and
                int(sc.px4.status_frames[-1].msg.arming_state) == VehicleStatus.ARMING_STATE_ARMED
                and int(sc.px4.status_frames[-1].msg.nav_state) == NAV_STATE_OFFBOARD,
                timeout_s=5.0, message="夹具注入的 armed/OFFBOARD 没落到状态帧上")


# ───────────────────────────── 接口：话题名 + QoS ─────────────────────────────

def test_接口与_QoS_与票面一致():
    """票面验收项：订阅两条、发布两条，QoS 逐个字段对。

    ★ 三条断言各自盯的东西（错了会长什么样）：
      · **发布方 QoS**：夹具自己的订阅是 `VOLATILE`（照抄 PX4 的 reader），与 DUT 发
        `TRANSIENT_LOCAL` **兼容** ⟹ 在"收得到吗"上永远看不出来，只能读发布方 info。
        ⚠️ 只能对 `reliability` / `durability` 下断言：`depth` 在这套栈的发现信息里恒为 0
        （见 `test_fake_px4_selftest.py::test_发现信息里读不回来的那两项_history_与_depth`）。
      · **订阅 `vehicle_status` 的 QoS**：这一项**不用读 info** —— PX4 的 writer 是
        BEST_EFFORT，DUT 若写成 RELIABLE（= ROS 2 的默认值！）就**一个回调都不进**、也
        **不会报错**，唯一的症状是 `matched_subscribers` 数不到它。 ⟹ `== 1` 就是那条判据。
      · **动捕那条话题**：`mocap` 节点用 `QoS(10)` 可靠发；DUT 若写 best_effort 同样永不匹配。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)

        for key in ("rates", "mode"):
            q = sc.px4.publisher_qos(key)
            assert q is not None, f"拿不到 {key} 的发布方 QoS"
            assert q.reliability == ReliabilityPolicy.BEST_EFFORT, \
                f"{key} 的 reliability 是 {q.reliability!r} —— PX4 的 reader 是 best_effort"
            assert q.durability == DurabilityPolicy.TRANSIENT_LOCAL, \
                f"{key} 的 durability 是 {q.durability!r} —— 票面要求 TRANSIENT_LOCAL"

        assert sc.px4.matched_subscribers("status") == 1, \
            "DUT 没接上 /fmu/out/vehicle_status（QoS 写错就永不匹配，且不报错）"
        assert sc.mocap_source.matched_subscribers() >= 1, \
            "DUT 没接上 /mocap/pose_enu（票面要求订 **ENU** 那条，不是 NED）"


# ───────────────────────────── 闸门：谁在发、谁不发 ─────────────────────────────

def test_闸门关闭时只发心跳_一帧_setpoint_都没有():
    """票面验收项「未解锁 / 未在 offboard 时**不发布**」+「心跳 ≥2 Hz」。

    ★★ 这里的读法是"不发**指令**"：`offboard_control_mode` 必须**无条件**发 —— 它是 PX4
      放行 OFFBOARD 的**前提**（`offboardCheck.cpp:40-65` 每轮先置
      `offboard_control_signal_lost = true`，只有收到新鲜的心跳才清零；
      `modeCheck.cpp:126-134` 在该标志为真时把 OFFBOARD 从"可运行"里删掉）。
      把心跳也闸在 `nav_state == OFFBOARD` 上就是**死锁**：不发心跳 ⟹ 进不去 OFFBOARD
      ⟹ 闸门永不开。真机上表现为"这飞机根本进不了 offboard"（理由与引文见 `mpc_node.py` 的 docstring）。

    ⇒ 所以这一条是**两条方向相反的断言合在一起**：心跳**在流**（正控：证明节点活着、
      动捕与状态都新鲜），而 setpoint **一帧都没有**（负断言）。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        sc.hold(0.5)                     # 先跑一段，让心跳频率的窗口有数据
        t_from = sc.t()
        sc.hold(2.0)

        modes = sc.px4.frames_between("mode", t_from, sc.t())
        assert len(modes) >= 4, f"{sc.t() - t_from:.1f} s 里只发了 {len(modes)} 个心跳（要 ≥2 Hz）"
        rate = sc.px4.rate_hz("mode", from_s=t_from, to_s=sc.t())
        assert rate >= 2.0, f"心跳只有 {rate:.2f} Hz，PX4 的 COM_OF_LOSS_T 是 1 s"
        for f in modes:
            m = f.msg
            assert m.body_rate is True, "心跳的 body_rate 必须是 True（(C) 级 CTBR 的接缝）"
            assert not any((m.position, m.velocity, m.acceleration, m.attitude, m.actuator)), \
                f"心跳里除 body_rate 外还有别的位为真：{m}"

        assert sc.px4.published_frames("rates") == [], \
            "还没解锁就发了 vehicle_rates_setpoint（闸门没关严）"

        # 闸门关着的原因要能被读到 —— 失败时"为什么没发"是第一个要回答的问题
        log = sc.dut_log()
        assert "闸门关闭" in log, f"DUT 没把闸门原因打进日志（日志尾：{log[-400:]!r}）"


def test_解锁切_OFFBOARD_之后立刻开始发():
    """闸门的**正控**：注入 armed + OFFBOARD 之后必须出现 setpoint。

    ★ 与上一条配对读 —— 只断言"解锁后有"无法区分"闸门根本没关"。
    ★ 阈值给 10 s 是刻意的：这是**事件驱动**的（夹具注入即刻发状态，节点 100 Hz 在跑），
      正常在 0.1 s 内就该出现；给宽只是不让 CI 抖动变成假红。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        sc.hold(0.5)
        assert sc.px4.published_frames("rates") == [], "正控的前提：此刻本来就不该有 setpoint"

        _arm_and_offboard(sc)
        sc.wait_for(lambda: sc.px4.published_frames("rates"), timeout_s=10.0,
                    message="解锁 + OFFBOARD 之后 10 s 内一帧 setpoint 都没发出来")
        assert sc.dut_alive(), "DUT 在发布之前就退出了"


def test_状态静默超过阈值之后_停发_setpoint_但心跳照发():
    """**按指定时刻注入故障**（票面那条验收项的实做）+ 停机条件在话题层被钉住。

    ★ 这条是 `px4_status_silence` 存在的理由，也是 07 要的接缝：故障**排在时刻上**
      （`t0 + 0.2`），不是"跑完再说"。窗口取 `status_timeout_s`（默认 0.75 s）之后
      再留 0.25 s 余量，且窗口内**两种帧分开数**：
      · setpoint **一帧都不能有** —— 状态龄超阈值 ⟹ `Cycle.publish=False`；
      · 心跳**照旧 50 Hz** —— 这正是票面读法的那条（§2.2）：不发热身/心跳会把 OFFBOARD 死锁。
    ★ 恢复之后再断言"又开始发" —— 否则"节点自己死了"和"闸门关着"在断言里长得一样。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: sc.px4.published_frames("rates"), timeout_s=10.0,
                    message="正控没成立：解锁前就不发，解锁后也没发")

        t0 = sc.t()
        sc.timeline.extend(px4_status_silence(sc.px4, t0 + 0.2, duration_s=3.0))

        # 静默 1.2 s 之后看 1.3 s 的窗口：阈值 0.75 s 早已越过，仍在静默期内。
        sc.advance_to(t0 + 2.5)
        n_sp = len(sc.px4.frames_between("rates", t0 + 1.2, t0 + 2.5))
        n_hb = len(sc.px4.frames_between("mode", t0 + 1.2, t0 + 2.5))
        assert n_sp == 0, f"状态龄超过 status_timeout_s 之后还在发 setpoint（{n_sp} 帧）"
        assert n_hb > 50, f"静默期间心跳掉了（{n_hb} 帧 / 1.3 s，期望 ~65）"
        assert sc.dut_alive(), "DUT 在静默期间退出了"

        # 恢复：状态重新流动 ⟹ 闸门重开（先重新热身 `warmup_cycles` 拍，100 Hz 下 50 ms）
        sc.advance_to(t0 + 6.5)
        n_after = len(sc.px4.frames_between("rates", t0 + 4.5, t0 + 6.5))
        assert n_after > 0, "状态恢复之后没有重新开始发 setpoint（闸门没重开）"


# ───────────────────────────── 参数：拼错就拒，标准参数放行 ─────────────────────────────

def test_拼错的参数当场拒_而_use_sim_time_放行():
    """`-p` 名字拼错 ⟹ **启动即退 exit 2**；ROS 自带的 `use_sim_time` ⟹ 参数照收、起得来。

    ★ 两半必须都在，因为它们会互相掩盖：
      · 少了"拼错就拒" ⇒ 那个补丁可以被删掉而没人知道（rclpy 对多写的参数**完全沉默**）；
      · 少了"标准参数放行" ⇒ 把 ROS 自带的名字也拒掉。**这条是复核抓出来的**：
        实测 `-p use_sim_time:=true` 曾让节点 exit 2（`未知参数：['use_sim_time']`），
        而 09/11 用 launch 起这个节点时带着它是常态 ⟹ 症状会是"节点起不来"，报错却指向参数名。
    ★ 拒绝那半**排在代码生成之前**，所以它毫秒级退出（实测：日志里只有开头那行 matplotlib 警告）。

    ★★ "放行"那半的判据**不能**用"收到第一帧心跳"（`_wait_dut_up`）—— 这是第一手踩到的：
      `use_sim_time=true` 而**没有任何节点发 `/clock`** 时，rclpy 的时钟冻在 0，
      50 Hz / 100 Hz 定时器**一次都不触发** ⟹ 节点活着、启动日志全打出来了、**一帧都不发**，
      看起来与"节点没起来"一模一样（第一次写这条时等了 120 s 才等到 BenchTimeout）。
      所以判据取"启动那行日志 + 进程还在"：那行是在 `declare_params` **之后**打的，
      参数被接受才有它；而 `未知参数` 这条字符串一旦出现就是被拒了。
      ⚠️ 顺带一条给 09/11 的事实：**这个节点在 `use_sim_time:=true` 下需要有人发 `/clock`**，
      否则它不发任何东西（不是坏了，是 ROS 时间没走）。
    """
    with _scenario(use_sim_time=True) as sc:
        sc.wait_for(lambda: "节点启动" in sc.dut_log(), timeout_s=STARTUP_S,
                    message="带上 use_sim_time 之后节点没能走到启动日志（参数检查那一关？）")
        assert "未知参数" not in sc.dut_log(), \
            f"ROS 自带的 use_sim_time 被当成未知参数拒了：{sc.dut_log()[-300:]!r}"
        assert sc.dut_alive(), "带上 use_sim_time 之后节点退出了"

    with _scenario(rate_hzz=50.0) as sc:
        sc.wait_for(lambda: not sc.dut_alive(), timeout_s=30.0,
                    message="参数名拼错（rate_hzz）却还在跑 —— 「未知参数」那道检查没生效")
        assert sc.dut_returncode() == 2, f"退出码是 {sc.dut_returncode()}，期望 2"
        assert "未知参数" in sc.dut_log(), f"退出时没说是未知参数：{sc.dut_log()[-300:]!r}"


# ───────────────────────────── 符号 + 推力归一化 ─────────────────────────────

def test_位置保持时推力归一化等于悬停油门_角速率近零():
    """票面验收项「推力归一化**从模型导出**」（M2 的结论：不再硬编码 `1/T_max`）。

    ★ 判据是一个**恒等式**：`thrust_body[2] = −F/T_max`，而位置保持的稳态 `F ≈ t_hover`
      ⟹ `thrust_body[2] ≈ −t_hover/T_max`。期望值**从模型算出来**（`Z0Params`），
      不写死 0.6235 —— 写死的话"改了质量、忘了改这里"就永远发现不了。

    ⚠️ 容差 0.06 是第 0 次迭代点的量级（出厂点 `F = 7.81 N` ⟹ 归一化 −0.662，与悬停的
      −0.6235 差 0.04），不是"随手给的"：RTI 的单次迭代 + 参考点就是测量点本身，
      稳态判据只能用带宽容差（求解器输出永远要容差断言，见 `test_ocp_*.py`）。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint")

        z0 = Z0Params()
        expected = -z0.t_hover / z0.t_max
        f = sc.px4.published_frames("rates")[-1].msg
        assert abs(float(f.thrust_body[2]) - expected) < 0.06, \
            f"位置保持的归一化推力 {float(f.thrust_body[2]):+.4f}，模型导出的悬停值是 {expected:+.4f}"
        # 位置保持 ⟹ 参考就是测量点 ⟹ 不出力（阈值宽松，只判"没有明显动作"）
        assert abs(f.roll) < 0.5 and abs(f.pitch) < 0.5 and abs(f.yaw) < 0.5, \
            f"位置保持时还在转：roll={f.roll:+.3f} pitch={f.pitch:+.3f} yaw={f.yaw:+.3f}"
        assert float(f.thrust_body[0]) == 0.0 and float(f.thrust_body[1]) == 0.0, \
            "(C) 级只用机体系 +z 那一路，另外两路必须是 0"


def test_目标在正前方时_pitch_为负_且顶约束表现为打满():
    """票面验收项「符号映射与 ticket 03 一致」+「饱和表现为**打满**，不静默截断」。

    ★ 符号（`sign-map.md` §2 的权威表）：机体系 +x 前进 ⟹ 需要绕 +y 转（FLU 右手系）
      ⟹ `ω_y > 0`，而 PX4 的 `pitch = −ω_y` ⟹ **`pitch` 是负的**。这一条把
      "符号表写反了"与"模型解错了"分开：前者会让 pitch 反号，后者只会让幅值不对。

    ★ "打满"：目标远在 10 m 外（1 s 时域内不可行）⟹ `ω_y` 顶在 `PITCH_RATE_MAX` 上。
      断言的是**等号附近**（容差 0.05），不是"小于上限" —— 后者对"被谁悄悄按比例缩回去"
      也是绿的。`mpc_node` 里刻意没有 `clip()`，就是为了让这一条量得出来。
    """
    with _scenario(hold_offset_x=10.0) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint")

        frames = sc.px4.published_frames("rates")
        first, last = float(frames[0].msg.pitch), float(frames[-1].msg.pitch)
        assert last < -3.0, f"目标在 +x 10 m 之外，pitch 却是 {last:+.3f}（符号映射反了？）"
        assert abs(abs(last) - PITCH_RATE_MAX) < 0.05, \
            f"|pitch| = {abs(last):.4f}，没有顶在 PITCH_RATE_MAX = {PITCH_RATE_MAX} 上" \
            "（饱和度被谁裁剪过？见 mpc_node 的「饱和表现为打满」）"
        # 默认热身 5 拍 ⟹ **第一帧**就已经带修正（否则等于在最需要动的时候朝悬停走）
        assert first < -3.0, \
            f"热身 5 拍之后的第一帧 pitch 只有 {first:+.3f} ⟹ 发出去的还是出厂迭代点"


def test_没热身时第一帧是出厂迭代点_热身之后就不再是():
    """★ 这一条把 `WARMUP_CYCLES` 的**理由**变成可观测的事实（票面要求"首拍不发"）。

    第一手（`test/conftest.py` 的模块 docstring，本机实测）：`SQP_RTI` 每周期只做**一次**
    迭代 ⟹ 全新实例的第 0 次 `solve()` 给的是"从出厂迭代点走一步"的结果，**不是**最优解：

        目标 +x 10 m（应当把 ω_y 打满）：第 0 次 ω_y = 0.0000、F = 7.8111（与悬停一模一样）
                                          第 1 次 ω_y = 3.8400（打满）

    ⇒ `warmup_cycles=0` 时，DUT 会把那一帧**发出去** —— 于是第一帧的 pitch ≈ 0，
      第二帧才打满。这正是"上电第一拍不要发"的实证：在最需要它动的时刻朝悬停走。

    ★ 端到端复现（2026-09-28 本机，`-p warmup_cycles:=0 -p hold_offset_x:=10`，话题上读到的
      前四帧，`pitch` 是与 ω_y 反号的那个量）：

        #1 pitch=-0.0000  thrust_body[2]=-0.6235   ← 出厂迭代点（等于悬停）
        #2 pitch=-3.8400  thrust_body[2]=-0.6235   ← 一步就到位（ω_y 打满）
        #3 pitch=-3.8400  thrust_body[2]=-0.8483
        #4 pitch=-3.8400  thrust_body[2]=-1.0000   ← 推力也顶到 T_max（"打满"的另一种）


    ⚠️ 这一条**依赖时序**（"第一帧必须是第 0 次解"）。它的正控是第二帧：如果 DUT 一上来就
      打满，说明热身参数没生效（而不是"第一帧恰好是稳态"）。
    """
    with _scenario(warmup_cycles=0, hold_offset_x=10.0) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 3, timeout_s=10.0,
                    message="解锁后没攒到 3 帧 setpoint")

        frames = sc.px4.published_frames("rates")
        p0 = float(frames[0].msg.pitch)
        p1 = float(frames[1].msg.pitch)
        assert abs(p0) < 0.05, \
            f"关掉热身之后第一帧 pitch = {p0:+.4f}，本该是出厂迭代点（≈0）—— 热身参数没生效？"
        assert p1 < -3.0, f"第二帧 pitch = {p1:+.3f} ⟹ 没有从「出厂迭代点」走出来"


# ───────────────────────────── 时间戳 ─────────────────────────────

def test_时间戳是墙钟微秒():
    """票面验收项「时间戳**微秒**」—— 单位错在 PX4 侧是**静默**的
    （`min(timestamp − time_offset, hrt)` 把它钳回"新鲜"，话题上看不出任何异常）。

    ★ 夹具不能替 DUT 发时间戳，但能把"它到底发了什么"翻译成一个有物理含义的数：
      `stamp_unit_guess` 按"相对**墙钟**的比值"把 µs / ms / ns / 零值分开（差三个数量级）。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: sc.px4.published_frames("rates"), timeout_s=10.0,
                    message="没有 setpoint 就没有时间戳可验")
        sc.hold(0.3)

        for key in ("rates", "mode"):
            unit, age = sc.px4.stamp_unit_guess(key)
            assert unit == "us", f"{key} 的时间戳被判成 {unit!r}（中位年龄 {age:.3g} s）"
            assert abs(age) < 0.5, f"{key} 的时间戳年龄 {age:.3g} s ⟹ 与墙钟不同步"


# ───────────────────────────── 进程收尾 ─────────────────────────────

def test_SIGINT_之后打出收尾摘要而不是_traceback():
    """★ 台架**读 stdout 那行摘要**（`mpc_node.main()` 用 `print` 而不是 `get_logger()`，
    因为 SIGINT 那条路上 rclpy 已经把 context 关掉了：发日志只会得到
    "Failed to publish log message to rosout"，而摘要本身看不到）。

    一条**负断言配正控**：没有 `Traceback`，同时**有**那行摘要 ——
    后者证明日志确实抓到了收尾路径（否则"没有 traceback"可能只是因为日志是空的）。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        sc.stop_dut()
        log, rc = sc.dut_log(), sc.dut_returncode()
        assert "[mpc_node] 收尾" in log, \
            f"SIGINT 之后没有收尾摘要；returncode={rc}；日志共 {len(log)} 字符：\n{log[:2000]}"
        assert "定时器" in log and "闸门" in log, "摘要里应当有定时器拍数与闸门统计"
        assert "Traceback" not in log, f"SIGINT 打出了 traceback\n{log[-2000:]}"
        assert rc == 0, f"SIGINT 之后 DUT 的退出码是 {rc}"


# ───────────────────────────── 频率与 RViz 观测面 ─────────────────────────────

def test_实做频率_心跳_50_Hz_setpoint_100_Hz():
    """票面验收项「心跳 ≥2 Hz、**实做 50 Hz**；**100 Hz 定时器**」。

    ★ 与闸门那条用例分工不同：那里断言的是 `≥2 Hz`（**PX4 的安全线**，`COM_OF_LOSS_T` 的量级），
      这里量的是**设计值有没有真的落在线上**——两者都要，因为把心跳写成 5 Hz 时前者照样绿。
    ★ 阈值刻意留宽（心跳 ±20%，setpoint 只卡下限）：这一条量的是"定时器是不是按设计在跑"，
      不是"Jetson 有多快"。**最坏耗时与降档决定归 [09]/[11]**（票面原文：频率在实测最坏
      耗时后再降档）⟹ 这里卡上限会把那张票该做的判断提前做掉。

    ★ 本机实测（2026-09-28，Orin NX，2 s 窗口、未解锁到解锁都在跑）：**心跳 50.0 Hz、
      setpoint 100.0 Hz**（两个定时器都恰好落在设计值上）。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 20, timeout_s=10.0,
                    message="解锁后没攒到 20 帧 setpoint，频率没得量")
        t_from = sc.t()
        sc.hold(2.0)
        hb = sc.px4.rate_hz("mode", from_s=t_from, to_s=sc.t())
        sp = sc.px4.rate_hz("rates", from_s=t_from, to_s=sc.t())
        assert 40.0 <= hb <= 60.0, f"心跳实测 {hb:.1f} Hz，设计值是 50 Hz（发得少就进不去 OFFBOARD）"
        assert sp >= 50.0, \
            f"setpoint 实测 {sp:.1f} Hz，设计值是 100 Hz（低了先查是不是每拍都在求解/被谁拖住）"


def test_RViz_的预测轨迹与参考点():
    """票面验收项：RViz 发布预测轨迹与参考点，用来肉眼判断 MPC（模型预测控制）是否在发疯。

    ★ 这两条话题（`/mpc/predicted_path` `nav_msgs/Path`、`/mpc/reference` `Marker`）是**本仓
      自己的观测面**，不接飞控 ⟹ 夹具里没有它们的位置，本用例现挂两个订阅收帧。
    ★ 把"肉眼判读"折算成三条可计算的代理：
      · `header.frame_id == "world"`（参数 `rviz_frame` 的默认值，写错在 RViz 里表现为整条不显示）
      · **点数 = N+1 = 51** ⟹ 画出来的是**整条时域**（N=50，见节点启动日志的
        `OCP：N=50 tf=1.0 s`），不是被谁截断的一小段；
      · 轨迹**起点 ≈ 当前位置**、**终点朝目标方向** —— 参考点也落在"测量位置 + 偏移"上
        （`hold_offset_x=5` ⟹ x ≈ 5）⟹ 偏移**符号**写反了这一条会红（那是肉眼最容易看漏的错）。

    ★ 本机实测（2026-09-28，`-p hold_offset_x:=5`）：51 点；起点
      `(0.0000, 0.0000, 0.0000)`；终点 `x=+2.7047`（1 s 时域、打满 pitch 的合理量级）；
      参考点 `x=+5.0000`；viz 定时器 10 Hz（实测 ~10 张/秒）。
    """
    from nav_msgs.msg import Path as NavPath
    from visualization_msgs.msg import Marker

    paths, refs = [], []
    with _scenario(hold_offset_x=5.0) as sc:
        # ★ 先建订阅、再等帧（别反过来：`count_publishers` 只说明"对方存在"，
        #   帧要晚一步才到 —— 同 `test_fake_px4_selftest.py::_publish_rates` 的教训）。
        subs = [sc.node.create_subscription(NavPath, "/mpc/predicted_path", paths.append, 10),
                sc.node.create_subscription(Marker, "/mpc/reference", refs.append, 10)]
        assert subs, "订阅对象要留着引用，否则会被 GC 掉"
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: paths and refs, timeout_s=15.0,
                    message="解锁之后 15 s 里没同时收到预测轨迹与参考点（viz 定时器没跑？）")

        path, ref = paths[-1], refs[-1]
        assert path.header.frame_id == "world", f"frame_id 是 {path.header.frame_id!r}"
        assert len(path.poses) == 51, \
            f"预测轨迹有 {len(path.poses)} 个点，应当是 N+1 = 51（整条时域）"
        p_first = path.poses[0].pose.position
        p_last = path.poses[-1].pose.position
        assert abs(p_first.x) < 0.2 and abs(p_first.y) < 0.2, \
            f"轨迹起点 ({p_first.x:+.3f}, {p_first.y:+.3f}) 不在当前位置附近（动捕原点）"
        assert p_last.x > p_first.x + 0.2, \
            f"轨迹终点 x = {p_last.x:+.3f} 没有朝 +x 的目标走（起点 {p_first.x:+.3f}）"

        assert ref.type == Marker.SPHERE and ref.action == Marker.ADD
        assert ref.header.frame_id == "world"
        assert abs(float(ref.pose.position.x) - 5.0) < 0.5, \
            f"参考点 x = {float(ref.pose.position.x):+.3f}，偏移参数写的 +5 m（符号反了？）"


# ───────────────────────────── 在这一层量不了的那几条（不写假测试）─────────────────────────────

def test_发现信息里的_depth_读不回来_所以票面那条只能靠读常量():
    """★ **票面验收项「pub … `depth 1`」在本套栈上黑盒不可观测** —— 这条用例钉住这个事实，
    而不是伪造一个恒红的断言。

    第一手（本机实测 2026-09-28，Humble + rmw_fastrtps，含对照）：DDS 发现信息里的
    `history` 是 `UNKNOWN`、`depth` **恒为 0**，且与发布者实际写的值**无关**
    （`depth=1` 与 `depth=10` 读回来一模一样）。重放也不可用——晚加入的
    `TRANSIENT_LOCAL` 订阅者**匹配得上**（发布者视角 `get_subscription_count() == 1`）、
    也收得到**新**帧，但**收不到任何旧帧** ⟹ 也拿不到"depth 语义"的间接证据。

    ⟹ 本节点这一条的守法是：① 发布方 QoS 常量（`QOS_PUB_FMU`）在**单测里**逐字段对
      （`depth=1` 在那里是能读的，对象就在进程内）；② 黑盒这一层只断言得到的两项
      （`test_接口与_QoS_与票面一致`）。夹具侧的同一事实见
      `test_fake_px4_selftest.py::test_发现信息里读不回来的那两项_history_与_depth`。
    """
    from rclpy.qos import HistoryPolicy

    from nmpc.mpc_node import QOS_PUB_FMU

    assert QOS_PUB_FMU.depth == 1, "发布方 QoS 常量里的 depth 必须是 1（票面）"
    assert QOS_PUB_FMU.history == HistoryPolicy.KEEP_LAST
    assert QOS_PUB_FMU.reliability == ReliabilityPolicy.BEST_EFFORT
    assert QOS_PUB_FMU.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_动捕那条话题没有速度字段_所以拍不出_tau_的黑盒测试():
    """★ 票面验收项「状态外推 `x + v·τ`」**在这一层无法黑盒验证** —— 原因不是夹具不够好，
    而是**输入里没有 `v`**（[issue 15](../../../.scratch/phase2/issues/15-mocap-velocity-source.md)）。

    第一手：`/mocap/pose_enu` 是 `geometry_msgs/PoseStamped`
    （`mocap_rel_pose_node.cpp:93`），而 `PoseStamped` **没有** `twist` 字段
    ⟹ 节点侧的 `v` 每拍都退化成 0（`node_core.cycle()` 里那条"缺字段"的路**一直在走**）
    ⟹ `x + v·τ` 在地面上恒等于恒等变换，**任何** τ 都能通过台架测试。

    速度其实一直在 `/vrpn/<刚体>/twist`（`vrpn_listener.cpp:46-65`），只是没人把它
    接到 Phase 2 这一侧。⚠️ 夹具**不能**偷偷塞一个带 twist 的 PoseStamped：
      ROS 2 里消息类型不匹配就是**不连接**，那会让夹具变成在说谎。

    ⇒ 所以 τ 只有 **`test_node_core.py` 里那两条**（给定 `v` 时外推正确 + 缺 `v` 时不崩）
      加上"从哪拿 `v`"这张票（15）解决之后，才可能有端到端的验证。
      本条断言把**前提**钉住：哪天上游换成带 twist 的类型，它会红，提醒来改这张票。
    """
    from geometry_msgs.msg import PoseStamped

    assert not hasattr(PoseStamped(), "twist"), \
        "PoseStamped 有 twist 了 ⟹ issue 15 可能已经解决，本文件该补一条 τ 的黑盒测试"
    assert not hasattr(PoseStamped(), "velocity")
