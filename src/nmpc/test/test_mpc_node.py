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

★★ **`pgrep -af mpc_node` 会把「别人的」DUT 一起列出来**（第一手，2026-09-28）：
  这个仓库里常有两个会话同时干活，其中一个正在跑 `pytest test -q`，它的 DUT 与我的
  **同一对固定话题**上发帧。看到的症状**不是**报错，而是**数字不对**：
  一条本该 `pitch = +0.5572` 的用例量出了 `pitch = −0.0000, thrust_body_z = −0.6235`
  —— 那是**默认配置（mocap 档、`hold_offset = 0`）那个 DUT 的悬停帧**混了进来。
  ⚠️ **认这个症状要看 `pitch = −0.0000`（≈ 零角速率），别认 `thrust_body_z` 那个数**：
  它是**旧归一化**下的悬停帧读数，2026-09-29 回填 `hover_thrust` 后悬停帧变成 ≈ −0.169
  （`t_hover/t_max`）⟹ 拿 −0.6235 当指纹会认不出来。
  ⟹ 判据：`ps -o ppid= -p <pid>` 看它的父进程 ——
    · 父进程是**活着的** `pytest` ⟹ 那是别人的工作，**别杀**，等它跑完（或换个时间再量）；
    · 父进程是 1（孤儿）⟹ 才是上头说的"上一次没杀干净"，该收掉。
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import numpy as np
import pytest
from px4_msgs.msg import VehicleStatus
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from bench import Scenario
from bench.fake_px4 import NAV_STATE_OFFBOARD
from bench.timeline import (mocap_pose_jump, mocap_stamp_rewind, px4_arm, px4_nav_state,
                            px4_state_silence, px4_status_silence)
from nmpc.params import PITCH_RATE_MAX, ROLL_RATE_MAX, Z0Params

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


def _读日志(path: Path) -> list:
    """逐帧 JSONL → 字典列表（**只读文件**，不碰节点；ticket 20 的四条用例共用）。

    ★ 数字全被 `mpc_node._jlist` 四舍五入到 **6 位小数** ⟹ 任何**两个读数相减**得到的量
      （`e_z = x_ref0[2] − x0[2]`、`ΔF` 的增量）都带 ~1e-6 的量化噪声
      ⟹ 断言别用比它更紧的容差（1e-9 那种会偶发假红）。
    """
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()]


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

    ★ 判据是一个**恒等式**：`thrust_body[2] = −F/t_max`（`t_max` = **归一化**常数，不是 `F` 上限），
      而位置保持的稳态 `F ≈ t_hover` ⟹ `thrust_body[2] ≈ −t_hover/t_max`。期望值**从模型算出来**
      （`Z0Params`），不写死 0.6235 —— 写死的话"改了质量、忘了改这里"就永远发现不了。

    ⚠️ 容差**按牛顿定义、再除以 `t_max`**（2026-09-29 订正）：它原来是归一化的 `0.06`，那是
      照着**旧归一化**（`t_max = 11.8 N`）量的第 0 次迭代点量级（出厂点 `F = 7.81 N` ⟹
      归一化 −0.662，与悬停 −0.6235 差 0.04）。`t_max` 换成实测的 46.3 N 后，**同一个物理
      偏差**折到归一化上只剩 1/3.9 ⟹ 还写 `0.06` 就是**比它本该管的事松 3.9 倍**（"假绿"）。
      不是"随手给的"：RTI 的单次迭代 + 参考点就是测量点本身，稳态判据只能用带宽容差
      （求解器输出永远要容差断言，见 `test_ocp_*.py`）。
    """
    #: 稳态推力与悬停值的允许偏差 [N]。归一化余量 = 本值 / `t_max`。
    #: **这个数不是新挑的**：旧阈值是归一化的 `0.06`，按旧归一化常数 `t_max = 11.8003 N`
    #: 折回物理量就是 `0.06 × 11.8003 = 0.708 N`（≈ 悬停推力的 10 %）。这里原样取那个物理量。
    HOVER_NORM_TOL_N = 0.708

    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint")

        z0 = Z0Params()
        expected = -z0.t_hover / z0.t_max
        tol = HOVER_NORM_TOL_N / z0.t_max
        f = sc.px4.published_frames("rates")[-1].msg
        assert abs(float(f.thrust_body[2]) - expected) < tol, \
            f"位置保持的归一化推力 {float(f.thrust_body[2]):+.4f}，模型导出的悬停值是 " \
            f"{expected:+.4f}（允许差 {tol:.4f} = {HOVER_NORM_TOL_N} N）"
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
        #4 pitch=-3.8400  thrust_body[2]=-1.0000   ← 推力也顶到上限（"打满"的另一种）

    ⚠️ **上面那一列 `thrust_body[2]` 是旧归一化（`t_max = 11.8 N`）下的读数**，别照着它认数
      （2026-09-29 回填 `hover_thrust` 后 `t_max = 46.3 N`）：现在悬停 ≈ **−0.169**、
      "顶到 `f_max`（11.8 N）" ≈ **−0.255**（不再是 −1.0 —— 归一化常数与上限**已经解耦**，
      `params.py`）。`pitch` 那一列不受影响（它不经过归一化）；本用例也只断言 `pitch`。


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
      · 轨迹**起点 ≈ 当前位置**、**终点朝目标方向** —— 参考点也落在"**锚点**位置 + 偏移"上
        （锚点 = 交班那一拍的测量；本用例的夹具位置不动，所以它就是原点）
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


# ───────────────────────────── 兜底与输入合理性（ticket 07 / M5）─────────────────────────────
#
# ★ 这一组**唯一**的"求解失败"注入面是 DUT 侧参数 `-p inject_solve_fail_after_frames:=N`。
#   理由（第一手）：夹具站在**话题边界**上，而"求解失败"发生在 DUT **进程内部** ——
#   话题面上只看得到帧，造不出"这一拍没解出来"。完整推导见 `mpc_node` 模块 docstring
#   的 ★ 那段与 `docs/control/mpc-node-fallback.md`。

def _hover_norm() -> float:
    """悬停归一化推力 —— **从模型算**（不是写死某个字面量，见归一化那条验收项）。"""
    z0 = Z0Params()
    return -z0.t_hover / z0.t_max


def _quad(f) -> tuple:
    """一帧 setpoint 里**会变的那四个数**（与 `node_core.frame_is_finite` 同一组）。"""
    m = f.msg
    return (float(m.roll), float(m.pitch), float(m.yaw), float(m.thrust_body[2]))


def _is_hover_frame(f) -> bool:
    return (max(abs(v) for v in _quad(f)[:3]) < 1e-6
            and abs(_quad(f)[3] - _hover_norm()) < 1e-3)


def _finite(f) -> bool:
    return all(math.isfinite(v) for v in _quad(f))


def test_求解失败时先保持上一帧_再切安全悬停_且一帧都不带非有限值():
    """票面判据 1/2/3：**绝不 raise** → `hold`（保持上一帧可行指令）→ `hover`（悬停推力 +
    零角速率），且**任何一帧都不许带非有限值**（判据 2 的端到端形式）。

    ★ 注入在"已发布 ≥ 20 帧之后"生效 ⟹ **同一条流上两个阶段**：注入前是正常解、注入后是
      兜底。不必去猜时刻，也不会有"注入有没有赶上"的含糊。
    ★ 两个阶段的形状**刻意拉开**：`hold_offset_x=10` 让正常解把 pitch 打满（−3.84），
      而悬停帧是 `(0, 0, 0, −t_hover/t_max)`（≈ −0.169，**从模型算**，别写死）—— 差两个量级，
      不会"看起来差不多"。
    ★ `hold` 的判据是"**内容原样、时间戳换新**"：PX4 会把最后一帧**锁存**下去
      （`MulticopterRateControl.cpp:177-184`）⟹ "我们不发了"与"飞控沿用旧帧"在飞控侧
      是同一件事，所以"在保持"必须是**发出去**的（否则它在话题上根本不可观测）。

    ★ 本票的"非有限值"那一条**与"求解失败"共用同一次注入**（坏解同时带：状态码非零 +
      `u` 全 NaN + 耗时 NaN，见 `mpc_node._poisoned`）。这一层能验的是"非有限值绝不到达
      话题"（下面那条扫描全帧的断言）；至于"是状态码先拦下的还是 NaN 先拦下的"，
      `solution_usable` 的判据顺序在 `test_node_core.py` 里逐条钉死 —— 两者在话题上
      **行为完全相同**，多写一条端到端用例只是重复。
    """
    hold_frames = 5
    with _scenario(hold_offset_x=10.0, fallback_hold_frames=hold_frames,
                   inject_solve_fail_after_frames=20) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 20 + hold_frames + 3,
                    timeout_s=10.0,
                    message="注入生效之后没有继续发帧（兜底把发布停掉了？那飞机就撒手了）")
        sc.hold(0.3)                       # 再攒几拍悬停，让"切过去了"这件事确定下来
        frames = sc.px4.published_frames("rates")
        assert sc.dut_alive(), "一个坏解就把节点搞崩了 —— 判据 1 要的是**绝不 raise**"
        assert all(_finite(f) for f in frames), \
            f"话题上出现了非有限值：{[_quad(f) for f in frames if not _finite(f)][:3]}"

        # 以"**第一帧安全悬停**"为锚点往前数（而不是按绝对帧号）：DDS 掉一两帧不该让用例变红，
        # 而"保持了几拍、保持的是不是上一帧的内容"这两件事都能从这个锚点相对地读出来。
        hovers_at = [i for i, f in enumerate(frames) if _is_hover_frame(f)]
        assert hovers_at, "注入之后一帧安全悬停都没有（兜底没切过去？）"
        i = hovers_at[0]
        assert i >= hold_frames + 1, f"第一帧悬停出现在第 {i} 帧，前面没有留出保持的空间"
        holds = frames[i - hold_frames:i]
        prev = frames[i - hold_frames - 1]

        assert abs(float(prev.msg.pitch)) > 3.0 and not _is_hover_frame(prev), \
            f"保持开始之前的那一帧不是打满的正常解：{_quad(prev)} ⟹ 正控不成立"
        assert not _is_hover_frame(holds[0]), \
            "『保持』发的是悬停帧 ⟹ 这一档根本没生效（下面几条断言会跟着变成废话）"
        for j, h in enumerate(holds):
            assert _quad(h) == _quad(prev), \
                f"第 {j + 1} 拍保持的**内容**变了：{_quad(h)} ≠ 上一帧可行指令 {_quad(prev)}"
        ts = [int(f.msg.timestamp) for f in holds]
        assert len(set(ts)) == len(ts) and ts[0] > int(prev.msg.timestamp), \
            f"保持帧的时间戳没有严格前进：{ts}（复用旧时间戳会让这一帧被读成陈旧的）"

        bad = [_quad(f) for f in frames[i:] if not _is_hover_frame(f)]
        assert not bad, f"切到安全悬停之后又出现了不是悬停的帧：{bad[:3]}"

        # 判据 3：这件事要在日志里**可检索**（"置标志并上报"落在进入悬停的那一次）
        sc.stop_dut()
        log = sc.dut_log()
        assert "兜底：连续" in log and "安全悬停" in log, \
            f"切安全悬停时没有上报（日志尾：{log[-500:]!r}）"
        assert "触发 1 次" in log, "收尾摘要里没有兜底的累计量（本次事件数）"
        assert "Traceback" not in log, f"兜底路径打出了 traceback\n{log[-1500:]}"


def test_动捕时间戳倒退时不发指令_追平之后恢复():
    """票面判据 4 的第一半：**不发指令**（而不是发一条错指令）。

    ★ 造法见 `bench.timeline.mocap_stamp_rewind`（`VrpnSource.rewind`）：**位置不动**，
      只有 stamp 往回走 2 s ⟹ 闸门关掉只可能是 `stamp_regress` 这一条判据干的
      （位姿判据看不见任何位移，超时判据也看不见"没数据"）。
    ★ 两半都要断言：停发**和**恢复。只断"停发"的话，"节点自己死了"也满足它。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint，闸门那条的正控不成立")

        t0 = sc.t()
        sc.timeline.extend(mocap_stamp_rewind(sc.mocap_source, t0 + 0.2, seconds=2.0, hold_s=1.0))

        sc.advance_to(t0 + 1.4)                     # 倒退中（0.2 → 1.2 是倒退窗口）
        n = len(sc.px4.frames_between("rates", t0 + 0.6, t0 + 1.2))
        assert n == 0, f"stamp 倒退期间还发了 {n} 帧 setpoint（判据 4：这种时候宁可不发）"
        assert sc.dut_alive(), "stamp 倒退把节点搞崩了"

        sc.advance_to(t0 + 2.5)                     # 追平：stamp 一次性跳回来
        n_after = len(sc.px4.frames_between("rates", t0 + 1.6, t0 + 2.5))
        assert n_after > 0, "stamp 追平之后没有重新开始发（闸门锁死了？）"

        assert "stamp_regress" in sc.dut_log(), \
            f"闸门原因没进日志（日志尾：{sc.dut_log()[-500:]!r}）"


def test_位姿跳变时不发指令_回到原位之后恢复():
    """票面判据 4 的第二半：刚体认错 / 遮挡恢复给出的**瞬移**同样只关闸门、不发错指令。

    ★ 100 m 的瞬移是 `hold_offset_x`（0）的 1000 倍，而默认判据的上限是 0.2 m
      （20 m/s × `JUMP_DT_FLOOR_S`）⟹ 拦下的余量极大，不受投递抖动影响。
    ★ 断言里带上读数（"位移 100.000 m"）：判据的**阈值**是给人调的，
      日志里没有这个数就无从调起（`Cycle.detail` 那条注释说的就是这件事）。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint，闸门那条的正控不成立")

        t0 = sc.t()
        sc.timeline.extend(mocap_pose_jump(sc.mocap_source, t0 + 0.2,
                                           position=(100.0, 0.0, 0.0), hold_s=1.0))

        sc.advance_to(t0 + 1.4)
        n = len(sc.px4.frames_between("rates", t0 + 0.6, t0 + 1.2))
        assert n == 0, f"位姿跳变期间还发了 {n} 帧 setpoint（那正是『发错指令』）"
        assert sc.dut_alive(), "位姿跳变把节点搞崩了"

        sc.advance_to(t0 + 2.5)
        n_after = len(sc.px4.frames_between("rates", t0 + 1.6, t0 + 2.5))
        assert n_after > 0, "源回到原位之后没有重新开始发（判据把自己锁死了？）"

        log = sc.dut_log()
        assert "pose_jump" in log, f"闸门原因没进日志（日志尾：{log[-500:]!r}）"
        assert "100.000" in log, f"跳变判据的读数（差多少米）没进日志：{log[-500:]!r}"


# ─────────────────── A 档：状态源换成 PX4 自己那两条（ticket 09）───────────────────
#
# ★★ 本组换掉的是**状态源**，不是坐标系 —— 话题面一点没变（还是 `VehicleRatesSetpoint`），
#   变的是"这个数是拿什么算出来的"：A 档从 `/fmu/out/vehicle_local_position` +
#   `/fmu/out/vehicle_attitude` 拿的是 **NED/FRD**，得先换到模型要的 ENU/FLU。于是判据只剩
#   两条**行为**：
#     ① 该订的订、**不该订的一帧都不订**（票面："本档**不使用动捕输入**"）；
#     ② 指令把机身**朝哪一边**拉 —— 而这一条在 NED 里想、在 ENU 里想会得到**不一样**的轴。
#
# ★ 期望值**不从 `node_core` import**（那是 DUT 内部，seam 1 的定义不许碰它）：本组照
#   `docs/control/sign-map.md` §2 的定义**自己**把两个换标签矩阵写出来、独立算一遍。
#   否则"转换写错了"与"断言抄了实现"就分不开 —— 这是本文件唯一一处"自己算一遍"的地方。

# ⚠️ 下面这三个（`L` / `M` / `_rot_from_quat`）与 `test_node_core.py` 里那份是**各写一份**的
#   —— 不是漏了复用：`colcon test` 一个文件一个进程、不保证两个测试模块能互相 import，
#   而这一处要的恰恰是**自己独立算一遍**（理由见上）。两份**一样**是结论，不是前提：
#   哪天对不上了，说明其中一份被改错了。
#
#: NED→ENU 的**世界换标签**：`p_enu = (y, x, −z)` 写成矩阵就是这个（`det = +1`，真旋转）。
L_NED_TO_ENU = np.array([[0.0, 1.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 0.0, -1.0]])
#: FRD→FLU 的**机体换标签**：绕 x 转 180°。
M_FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def _rot_from_quat(q) -> np.ndarray:
    """w-first 四元数 → 旋转矩阵（本组自己算，见上面那两段 ★：不与 `test_node_core.py` 共享，
    也与 `model.q_to_rot_mat` 无关 —— 那个是 CasADi 符号表达式）。"""
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _flu_rot(q_ned_frd) -> np.ndarray:
    """「FRD 机体 → NED 世界」的姿态 → 「FLU 机体 → ENU 世界」的旋转矩阵。"""
    return L_NED_TO_ENU @ _rot_from_quat(q_ned_frd) @ M_FRD_TO_FLU


def _ned_axis_quat(axis: str, deg: float):
    """绕 **NED** 某一根轴转 `deg` 度的姿态（w-first）—— 注入用的。"""
    half = math.radians(float(deg)) / 2.0
    c, s = math.cos(half), math.sin(half)
    return {"x": (c, s, 0.0, 0.0), "y": (c, 0.0, s, 0.0), "z": (c, 0.0, 0.0, s)}[axis]


def _cmd_to_flu(msg) -> np.ndarray:
    """话题上的三个角速率 → **FLU 机体系**角速率 `(ωx, ωy, ωz)`。

    反的就是 `sign_map.px4_rates_setpoint`：`roll = ωx`、`pitch = −ωy`、`yaw = −ωz`。
    ⟹ 这一行本身就是"两套约定是一回事"的断言：**日志里的 `u` 与话题上的 `roll/pitch/yaw`
      必须只差这一个式子**（`test_A_档的逐帧日志…` 里对着 JSONL 再钉一遍）。
    """
    return np.array([float(msg.roll), -float(msg.pitch), -float(msg.yaw)])


def _need_rate_toward(d_world, q_ned_frd=(1.0, 0.0, 0.0, 0.0)) -> np.ndarray:
    """把**推力矢量**朝世界方向 `d`（ENU）倾，所需的 **FLU 机体系**角速率方向。

    ★ 纯几何 + 一次符号映射，与 DUT 怎么算无关：`ω_world ∝ ẑ_body × d̂`（正转把机体 z 朝
      `d̂` 拉），再旋进机体系 `ω_body = R_fluᵀ ω_world`。量纲/增益**不是**要断言的东西
      （OCP 的增益由代价权重定），**方向**才是。
    ★ 机体系里 `roll = ω_x`、`pitch = −ω_y` ⟹ 想让飞机朝 `d` 加速：`roll` 的期望值就是
      `ω_body[0]`、`pitch` 的期望值是 `−ω_body[1]`。
    """
    R = _flu_rot(q_ned_frd)
    zb = R @ np.array([0.0, 0.0, 1.0])
    d = np.asarray(d_world, dtype=float).reshape(3)
    return R.T @ np.cross(zb, d / float(np.linalg.norm(d)))


def test_A_档只订_px4_那两条_动捕一帧都不订():
    """票面验收项「状态源取 PX4 自己的 `/fmu/out/vehicle_local_position`
    + `vehicle_attitude`」+「本档**不使用动捕输入**」。

    ★★ 「不使用动捕」这一条**只有条件订阅才测得出**：两档都订、只是不看的那种写法，
      在话题上**完全看不出来**（动捕帧照进、只是被丢掉）—— `matched_subscribers() == 0`
      是唯一能把它与"根本没订"分开的地方。这也是 `NodeParams.state_source` 做成
      **条件订阅**（而不是"都订、按参数挑一个"）的理由，见 `mpc_node` 的 ★★ 表。

    ⚠️ 断言的**顺序**是判据的一部分：必须等发现跑完才能说"没有订阅者"。
      屏障取"PX4 那两条都匹配上了"（⟹ 这个进程已经建完订阅），再加 `hold(1.0)` 让 DDS
      把该发现的发现完 —— 节点是在 `__init__` 里**一次性**建订阅的，不会再冒出新的。
      ⚠️ 反过来（先断言 == 0 再等）就是一条**永远绿**的假断言：那一刻谁都没匹配上。

    ★ 正控别省：解锁 + OFFBOARD 之后必须**真的开始发 setpoint** —— 否则"一帧都不订"
      与"节点压根没起来"在断言里长得一样。
    """
    with _scenario(state_source="px4") as sc:
        _wait_dut_up(sc)
        sc.wait_for(lambda: sc.px4.matched_subscribers("local_position") >= 1
                    and sc.px4.matched_subscribers("attitude") >= 1,
                    timeout_s=10.0,
                    message="DUT 没订上 /fmu/out/vehicle_local_position 或 vehicle_attitude")
        sc.hold(1.0)

        assert sc.mocap_source.matched_subscribers() == 0, \
            "A 档下 /mocap/pose_enu 上还有订阅者 —— 票面要求本档不使用动捕输入"

        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 5, timeout_s=10.0,
                    message="正控不成立：订上 PX4 两条之后并没有真的开始发 setpoint")


@pytest.mark.parametrize("key,reason", [("local_position", "px4_stale"),
                                        ("attitude", "px4_attitude_stale")])
def test_A_档_两条状态各判各的龄(key, reason):
    """票面验收项「状态源取 … `vehicle_local_position` + `vehicle_attitude`」的**判据面**。

    ★ 两条消息是**两条独立的流**（z0 SITL 实测 32.5 / 31.6 Hz，2026-09-28）⟹ 各判各的龄。
      所以本用例**按话题分别**掐断，且断言的是**哪一个理由**进了日志：
        · 只掐 `local_position` ⟹ 理由必须是 `px4_stale`；
        · 只掐 `attitude`      ⟹ 理由必须是 `px4_attitude_stale`。
      只断言"停发了"是不够的 —— 合并成一条判据（"任一条旧了就算旧"）也满足"停发了"，
      而它就丢掉了"姿态停在几秒前、节点却以为在闭环"这个唯一想抓的失效
      （姿态变化慢，两个选择只差一个小倾角，话题上几乎看不出来）。
    ★ 时间尺子：静默 3 s，判据是 `px4_timeout_s`（默认 0.5 s）、而这条流周期只有 ~31 ms
      ⟹ 跨得过闸门（与 `px4_status_silence` 同一条纪律：时长要**明显大于**阈值）。
    """
    with _scenario(state_source="px4") as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: sc.px4.published_frames("rates"), timeout_s=10.0,
                    message="正控没成立：解锁之后就没发过 setpoint")

        t0 = sc.t()
        sc.timeline.extend(px4_state_silence(sc.px4, t0 + 0.2, duration_s=3.0, key=key))

        sc.advance_to(t0 + 2.5)
        n_sp = len(sc.px4.frames_between("rates", t0 + 1.2, t0 + 2.5))
        n_hb = len(sc.px4.frames_between("mode", t0 + 1.2, t0 + 2.5))
        assert n_sp == 0, f"只掐了 {key}，setpoint 却还在发（{n_sp} 帧）—— 那一条没被判据看着"
        assert n_hb > 50, f"静默期间心跳掉了（{n_hb} 帧 / 1.3 s，期望 ~65）"
        assert sc.dut_alive(), "DUT 在静默期间退出了"
        log = sc.dut_log()
        assert reason in log, \
            f"停发的理由不是 {reason}（日志尾：{log[-500:]!r}）—— 两条状态被并成一条判据了？"

        sc.advance_to(t0 + 6.5)
        n_after = len(sc.px4.frames_between("rates", t0 + 4.5, t0 + 6.5))
        assert n_after > 0, f"{key} 回来之后没有重新开始发（闸门没重开）"


@pytest.mark.parametrize("offset, d_enu", [({"hold_offset_x": 10.0}, (1.0, 0.0, 0.0)),
                                           ({"hold_offset_y": 10.0}, (0.0, 1.0, 0.0))])
def test_A_档_同一条正前方任务_走哪一根轴由_NED_到_ENU_的换算决定(offset, d_enu):
    """票面验收项「符号映射与推力标定在**真 PX4** 上最终确认 —— 判据是**行为符合预期**」
    在话题层的落点：同一件事（"朝 ENU 的某个方向追 10 m"）在 A 档下必须走**换算出来**的那根轴。

    ★★ 为什么这是一条**有牙齿**的断言（而不是"抄了实现"）：任务方向 `d_enu` 是**测试给的**
      （`hold_offset` 就在 ENU 里），而机头朝向是**换算出来的** —— 两者之间那对关系由
      NED→ENU/FRD→FLU 决定。初始姿态是 NED 单位四元数（= 机头朝**北**），换算到 ENU 之后
      机头指 **ENU 的 +y** ⟹ 同一条 "ENU +x 追 10 m" 必须靠**绕机体 x 转**（`roll`）做到：

        mocap 档（同一张票面上的同一条任务）：初始姿态是 ENU 单位四元数（机头 = +x）
          ⟹ `pitch < 0`（见 `test_目标在正前方时_pitch_为负_且顶约束表现为打满`，实测 −3.84）
        A 档（本用例）：机头 = ENU +y ⟹ 主项必须是 **`roll`**（实测 +3.8400 = `ROLL_RATE_MAX`）

      ⟹ 任何"忘了转姿态"或"只翻了一半符号"的实现都会以为机头还在 +x、于是发 `pitch`
        （或者两根轴都发）—— 这条用例就会红。

    ★ 期望值走 `_need_rate_toward`（本组自己算的几何），**不写死"roll/pitch"**：
      10 m 在 1 s 的时域里不可行 ⟹ 主项必然**打满**，所以断言的是"顶在 `*_RATE_MAX` 上"
      （等号附近，容差 0.05），而不是"小于上限" —— 后者对"被谁悄悄按比例缩回去"也是绿的。

    ⚠️ 位置换算（`(y, x, −z)`）**在这条里没有被考**：本用例的夹具**位置不动**，而参考钉在
      交班那一拍的测量上（锚点）⟹ 位置**误差**恒等于 `hold_offset`，与怎么换无关。
      ★ 它**不是**"位置换算量不到"（那句话在锚点之前成立、之后就不成立了）：让夹具的位置
      动起来就能考到 —— 那是 `test_A_档_锚点让位置换算在话题层可观测` 的事。
    """
    with _scenario(state_source="px4", **offset) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint")

        w_need = _need_rate_toward(d_enu)      # 初始姿态 = NED 单位四元数（机头朝北）
        if abs(float(w_need[0])) >= abs(float(w_need[1])):
            main, other, want, rate_max = "roll", "pitch", float(w_need[0]), ROLL_RATE_MAX
        else:
            main, other, want, rate_max = "pitch", "roll", -float(w_need[1]), PITCH_RATE_MAX

        last = sc.px4.published_frames("rates")[-1].msg
        got = float(getattr(last, main))
        other_v = float(getattr(last, other))
        assert got * want > 0.0, \
            f"{offset}（朝 ENU {d_enu} 追 10 m）：主项该是 {main} 的" \
            f"{'+' if want > 0 else '−'} 号，实测 {got:+.4f}（换算把机头方向弄反了？）"
        assert abs(abs(got) - rate_max) < 0.05, \
            f"{main} = {got:+.4f}，没顶在 {rate_max} 上 —— 10 m 在 1 s 时域里不可行，该打满"
        assert abs(other_v) < 0.2, \
            f"{other} = {other_v:+.4f} 不该有动作（纯 {main} 的任务被换成了斜的？）"


def test_A_档_锚点让位置换算在话题层可观测():
    """★ **锚点改动的第二个后果**：位置换算第一次**在话题层**就能考（ticket 09 之前量不到）。

    交班之后把夹具的**位置**挪走 0.3 m（NED 北 ⟹ ENU 的 +y），锚点仍在原点 ⟹ 位置误差的
    **世界方向**是"从当前点指回锚点" = ENU 的 **−y**。于是节点该发的是一条朝 −y 倾转的角速率
    —— 方向由 `_need_rate_toward`（本组自己算的几何）给出，**不写死 roll/pitch**。

    ★★ 有牙齿的地方：`.3 m` 是**从 NED 报进去的**，而锚点与当前点都在 ENU 里比较 ——
      "忘了换轴"（把 NED 的 north 当 ENU +x）会让期望方向整整转 90°，投影当场变负。
    ★ 位移取 **0.3 m**：位姿跳变闸门的上限是 `20 m/s × dt`（dt ≈ 一帧 31 ms ⟹ 0.6 m），
      1 m 会先被判成 `pose_jump`，看到的就不是本用例要考的那件事。
    ★ 位置项在 0.3 m 上**不饱和**（不像上面那条 10 m 会打满）⟹ 判据是"投影增益为正、
      残差小"，不是"顶在 `*_RATE_MAX` 上"。
    ★ 本机实测（2026-09-28，本机 SITL 夹具）：`ω_cmd = [1.0e-04, −0.1456, −2.2e-03]`、
      需要的方向 `(0, −1, 0)` ⟹ 投影 `k = **+0.1456**`、落在期望方向之外的只剩 0.0022 rad/s。
      判据取 `k > 0.05`（实测的 1/3）与"残差 < 0.3·|ω_cmd|"（实测的 ~1/60）。
    """
    with _scenario(state_source="px4") as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint")

        # 锚点已经在原点钉住了（交班那一拍）—— 现在把飞机挪走
        sc.px4.set_position_ned(0.3, 0.0, 0.0)          # NED 北 0.3 m ⟹ ENU (0, +0.3, 0)
        n0 = len(sc.px4.published_frames("rates"))
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= n0 + 8, timeout_s=10.0,
                    message="注入位置偏移之后不再发 setpoint（撞上跳变闸门？）")
        last = sc.px4.published_frames("rates")[-1].msg

        w_need = _need_rate_toward((0.0, -1.0, 0.0))     # 从 ENU +y 回到锚点（原点）
        assert float(np.linalg.norm(w_need)) > 0.1, "这条场景本身没造出方向，用例写错了"
        w_cmd = _cmd_to_flu(last)
        k = float(w_cmd @ w_need) / float(w_need @ w_need)
        assert k > 0.05, \
            f"位置误差（0.3 m，世界 −y）在话题上没产生朝那个方向的倾转：k = {k:+.4f}、" \
            f"ω_cmd = {np.round(w_cmd, 4)}、需要 = {np.round(w_need, 4)}" \
            f"（位置换算把北当成了 +x？锚点没钉住？）"
        assert float(np.linalg.norm(w_cmd - k * w_need)) < 0.3 * float(np.linalg.norm(w_cmd)), \
            f"指令里有 {np.linalg.norm(w_cmd - k * w_need):.4f} rad/s 落在期望方向之外"


@pytest.mark.parametrize("axis, deg, expect_flat", [("y", 10.0, False),
                                                   ("x", 10.0, False),
                                                   ("z", 90.0, True)])
def test_A_档_注入的倾斜_指令方向是把机身拉平的那一边(axis, deg, expect_flat):
    """票面验收项「符号映射 … 在真 PX4 上最终确认」的姿态面：**注入一个倾斜，看它往哪边修**。

    ★ 参考姿态是**水平**的（`hold_reference`：位置 = **锚点** + 偏移、速度 0、姿态水平），
      而代价只罚**倾斜**（`OcpConfig.q_diag` 的 `qw` 权重是 0）⟹ 一个倾斜的测量姿态
      必然换出一条**朝水平修**的角速率，方向由换算决定（希望值走 `_need_rate_toward`）。

    ★★ 这条能抓住什么、抓不住什么（第一手，2026-09-28 本机，四种错法各算了一遍）：
      · 抓得住：**完全不转**（`q_enu = q_ned`）—— 它把倾斜轴换成另一根（NED-y 倾斜本该走
        `pitch`，它会走 `roll`），投影测试直接红；
      · **抓不住**："只翻 pitch/yaw"与"只换世界标签"这两种错法：对**单轴**倾斜，机体系里
        所需的转速轴与整体 90° 偏航**无关** ⟹ 投影仍是正的。它们要靠
        `test_A_档_同一条正前方任务…`（那条考的是机头朝向）与 SITL 闭环才是红的。
      ⟹ 本用例的价值是"姿态误差被修的方向是对的"，**不是**"转换完全正确"。

    ★ `deg=10` 是**按跳变闸门选的**，不是随手给的：`_pose_jump` 的上限是
      `mocap_jump_rate_max (12.6 rad/s) × dt`，而 `dt` 取源自己的 stamp 差（这条流 ~31 ms）
      ⟹ 允许的单拍转角 ≈ 12.6 × 0.031 = **22.4°**。10° 留了一倍余量；给到 30° 就会先撞上
      **跳变闸门**（停发），看到的是"什么都不发"而不是"往哪边修"。
    ★ `axis="z"`（纯偏航）是**负控**：偏航不产生倾斜 ⟹ 三轴角速率与推力都该是悬停值。
      它同时证明"上面那些非零读数**是**倾斜带来的"，而不是"节点一被注入就乱发"。
    """
    z0 = Z0Params()
    q_ned = _ned_axis_quat(axis, deg)
    with _scenario(state_source="px4") as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint")

        n0 = len(sc.px4.published_frames("rates"))
        sc.px4.set_attitude_quat(q_ned)
        # 注入之后**至少 8 帧**再看：状态流 ~32 Hz、节点 100 Hz ⟹ 8 帧足以让新姿态进闭环
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= n0 + 8, timeout_s=10.0,
                    message=f"注入 NED {axis} {deg:g}° 之后就不再发 setpoint 了（撞上跳变闸门？）")
        last = sc.px4.published_frames("rates")[-1].msg

        if expect_flat:
            w = _cmd_to_flu(last)
            assert float(np.linalg.norm(w)) < 0.02, \
                f"纯偏航（NED {axis} {deg:g}°）不该产生任何倾斜修正，实测 ω = {np.round(w, 4)}"
            hover = -z0.t_hover / z0.t_max
            # 容差按牛顿定义再除以 `t_max`（同 `test_位置保持…` 那条的理由）：纯偏航下
            # 推力该守在悬停值上。旧阈值是归一化的 `0.02` ⟹ 物理量 `0.02 × 11.8003 = 0.236 N`，
            # 原样取那个物理量（不写死归一化余量，它随 `t_max` 走）。
            tol = 0.236 / z0.t_max
            assert abs(float(last.thrust_body[2]) - hover) < tol, \
                f"纯偏航下推力该是悬停值 {hover:+.4f}，实测 {float(last.thrust_body[2]):+.4f}" \
                f"（允许差 {tol:.4f} = 0.236 N）"
            return

        w_need = _need_rate_toward((0.0, 0.0, 1.0), q_ned)   # 水平参考 ⟹ 朝「上」修
        assert float(np.linalg.norm(w_need)) > 0.1, "这条注入本身没造出倾斜，用例写错了"
        w_cmd = _cmd_to_flu(last)
        k = float(w_cmd @ w_need) / float(w_need @ w_need)
        assert k > 0.5, \
            f"指令在「把机身拉平」方向上的增益是 {k:+.3f}（该是正的、可观的）—— " \
            f"ω_cmd = {np.round(w_cmd, 4)}、需要 = {np.round(w_need, 4)}"
        assert float(np.linalg.norm(w_cmd - k * w_need)) < 0.1, \
            f"指令里有 {np.linalg.norm(w_cmd - k * w_need):.3f} rad/s 落在拉平之外的方向上"


def test_A_档_有速度之后_tau_第一次在话题上可观测_配对差分():
    """A 档的**新能力**：外推 `x + v·τ` 第一次能被黑盒考到 —— 因为这条源**有速度**。

    ★★ 为什么 A 档之前测不出来（见 `test_动捕那条话题没有速度字段_所以拍不出_tau_的黑盒测试`）：
      `/mocap/pose_enu` 是 `PoseStamped`，**没有** `twist` ⟹ `v ≡ 0` ⟹ `x + v·τ` 恒等变换
      ⟹ 话题上**任何** τ 都长得一样。A 档的 `vehicle_local_position` 带 `vx/vy/vz`。

    ★★ 为什么判据是**两次跑程的差**、而不是"有/没有动作"（第一手，2026-09-28 本机——
      这一条是**被实测推翻过一次**的判据，别改回去）：
      当初按"τ=0.05 有动作、τ≈0 没动作"写，靠的是一个"τ≈0 ⟹ 全零悬停帧"的观测 ——
      而那次观测是**别的会话的 DUT** 发在同一对话题上的（模块 docstring 的 ★★ 记了这次事故）。
      真实的机理是：`v ≠ 0` 时，参考的速度是 0 而状态的速度是 v ⟹ OCP 要**刹住**它，
      这一项**与 τ 无关**；τ 只通过位置那一项进来（`x_ref − x0 = hold_offset − v·τ`）。
      所以 τ 的效果是**幅度上的一点差**，不是"有/无"。实测（`hold_offset_z=0`、
      `v = NED (1,0,0)` 北向 1 m/s、末帧，2026-09-28 本机）：

          τ = 0.05  ⟹ ω_FLU = [3.0e-04, −0.5572, −2.2e-03]
          τ = 1e-6  ⟹ ω_FLU = [3.0e-04, −0.5340, −2.1e-03]
          差        = [0, −0.0232, −2e-04] ⟹ 投影到期望方向 k = **+0.0232**

      即灵敏度 ≈ 0.46 (rad/s) / s 的 τ（在 v = 1 m/s 下）。判据取 k > 0.004
      （离实测 6 倍）与"残差 < 0.3·|差|"（实测 0.009）。
      ⚠️ τ 的**方向**与"刹住速度"的方向是同一个（都是 −v̂）⟹ 方向判据分辨不出 τ，
        能分辨的只有**幅度**。这是本用例唯一的、也是诚实的判据。

    ★ 负控那一路**不能写 `tau_s=0.0`**（第一手，同日）：节点自己在参数校验里就拒了它 ——
      启动日志原文 `[mpc_node] 启动失败：参数 tau_s 要是正的有限数，收到 0.0`、
      `returncode=2`、一帧都不发（`_wait_dut_up` 空等满 120 s 才红）。**这本身是对的**：
      零阶外推在数值上就是没外推，与其让一个无意义的参数悄悄跑，不如在启动时拦住。
      ⟹ 负控用 `tau_s=1e-6`（外推量只有正例的 2e-5 倍）。
    ★ `hold_offset_z=0` 是**设计意图**（让两跑之间只剩速度相关项），不是"不这样就测不出来"：
      实测 `hold_offset_z` 取 0 与取默认 1.0 时，角速率读数到小数第三位都一样 ——
      "保持 1 m"那一项进的是**推力**（`u[0]`），不是横向角速率。
    """
    def run(tau_s: float) -> np.ndarray:
        with _scenario(state_source="px4", hold_offset_z=0.0, tau_s=tau_s) as sc:
            _wait_dut_up(sc)
            _arm_and_offboard(sc)
            sc.px4.set_velocity_ned(1.0, 0.0, 0.0)
            sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 15, timeout_s=10.0,
                        message="注入速度后没攒到 15 帧 setpoint")
            return _cmd_to_flu(sc.px4.published_frames("rates")[-1].msg)

    w_big, w_tiny = run(0.05), run(1e-6)
    d = w_big - w_tiny
    w_need = _need_rate_toward((0.0, -1.0, 0.0))     # ENU：NED 北 +1 m/s ⟹ ENU 东 +1 ⟹ 该朝南
    k = float(d @ w_need) / float(w_need @ w_need)
    print(f"\nτ=0.05 ⟹ ω={np.round(w_big, 4)}；τ=1e-6 ⟹ ω={np.round(w_tiny, 4)}；"
          f"差 {np.round(d, 4)}（k={k:+.4f}，期望方向 {np.round(w_need, 4)}）")

    assert k > 0.004, \
        f"两种 τ 的差没有落在外推该有的方向上：差 {np.round(d, 4)}、" \
        f"期望方向 {np.round(w_need, 4)}（k={k:+.5f}）—— 外推项没进指令？"
    assert float(np.linalg.norm(d - k * w_need)) < 0.3 * float(np.linalg.norm(d)), \
        f"两种 τ 的差不在一个方向上：差 {np.round(d, 4)}、k·w_need = {np.round(k * w_need, 4)}"


def test_A_档_垂直速度的外推_表现是推力低于悬停():
    """A 档外推的**另一面**：垂直方向的速度差改的是**推力**，不是角速率（纯几何上 `ẑ × ẑ = 0`）。

    ★ 这条顺手把**速度**的 z 符号钉住（`v_enu_z = −v_ned_z`）：注入 NED `vz = −1`（**上升** 1 m/s），
      τ = 0.05 ⟹ `x0` 比参考高 5 cm ⟹ 该**减推力**。符号如果反了，节点会以为在下降、于是加推力
      —— 而"加/减推力"在话题上就是 `thrust_body[2]` 的两个方向，一眼分得开。
    ★ 与上一条配对：一个考速度进**角速率**，一个考速度进**推力**；两条的期望符号都由
      "参考比外推点低/高"推出来，不抄实现。实测（2026-09-28 本机，**当时代码**）：
        `vz = −1` ⟹ `thrust_body[2] = −0.3974`（当时悬停是 −0.6235 ⟹ 确实**低于**悬停）

    ⚠️ **判据跟着物理量走，不跟着归一化走**（2026-09-29 回填 `hover_thrust` 后订正）：
      上面那组读数是在**旧归一化**（`t_max = 11.8 N`）下量的；现在 `t_max = 46.3 N`，
      **同一个物理推力差**折到归一化上是原来的 **1/3.9**。把阈值仍写成归一化的 `0.05`，
      余量会从 4.5 倍缩到 1.15 倍 —— 变成一条随时可能假红的用例。
      所以这里先把阈值定义成**牛顿**（`MIN_DELTA_THRUST_N`），再折算成归一化余量。
    """
    #: 外推 5 cm 该带来的推力变化，至少要有这么多 **牛顿**（≈ 悬停推力的 8 %）。
    #: **不是新挑的**：旧阈值是归一化的 `0.05`，按旧归一化常数 `t_max = 11.8003 N` 折回物理量
    #: 就是 `0.05 × 11.8003 = 0.590 N`。折算是 `MIN_DELTA_THRUST_N / t_max` ——
    #: 归一到旧常数 = 0.05（2026-09-28 用的就是这个），归一到实测的 46.3 N = 0.0127。
    MIN_DELTA_THRUST_N = 0.590

    z0 = Z0Params()
    hover = -z0.t_hover / z0.t_max
    min_delta = MIN_DELTA_THRUST_N / z0.t_max
    with _scenario(state_source="px4") as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.px4.set_velocity_ned(0.0, 0.0, -1.0)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 15, timeout_s=10.0,
                    message="注入垂直速度后没攒到 15 帧 setpoint")
        last = sc.px4.published_frames("rates")[-1].msg

        fz = float(last.thrust_body[2])
        assert fz > hover + min_delta, \
            f"向上 1 m/s ⟹ 外推点比参考高 ⟹ 该减推力，实测 {fz:+.4f}（悬停 {hover:+.4f}、" \
            f"要求高出 {min_delta:.4f} = {MIN_DELTA_THRUST_N} N）—— 速度的 z 符号反了？"
        assert abs(float(last.roll)) < 0.02 and abs(float(last.pitch)) < 0.02, \
            "垂直方向的速度差不该产生角速率（ẑ × ẑ = 0）"


def test_A_档的逐帧日志把状态源_求解耗时_饱和标志都记下来了(tmp_path):
    """票面验收项「日志记录 MPC（模型预测控制）内部状态、**求解耗时**、**约束饱和标志**，
    供飞行后复盘而不只依赖记忆」—— 这条验收项以前**没有任何用例**，而它的失效方式是静默的：
    键名写错、值不能 JSON 序列化（`float32` / `uint8`）、某一档忘了合并进去，
    症状都是"飞行之后打开日志发现少了一列"，那时已经晚了。

    ★ 判据分三层，**都对着文件读**（不读节点内部）：
      · **结构**：每行都是合法 JSON、`u` 是 4 维、`x0` 是 10 维（`u` 是求解器给的、
        `x0` 是喂进去的、`x_ref0` 是要它去哪 —— 复盘时靠这三列把一帧重算回去）；
      · **A 档的取证面**：`state_source` 与两条状态的**分龄**（`age_pos_s` / `age_att_s`）
        以及 EKF 自报的有效性标志 —— 这正是票面要的"谁给的、多旧"；
      · **与话题的一致性**（最要紧的一条）：同一帧里 `u` 是 FLU/ENU 的内部量、`roll/pitch` 是
        PX4 的 FRD/NED 量，两者**必须只差 `sign_map` 那一式**。这一条把"映射写错了"与
        "模型解错了"分开，也正是 `docs/control/sign-map.md` 要的东西。

    ★ 热身拍也**照记**（`fallback == "warmup"`）：那几拍不发，却是"最坏求解耗时"里最容易漏掉的
      （RTI 冷启动、代码页刚进 cache）—— 实测最坏耗时是票面 D6 那个数字的来源，
      漏掉冷启动那几拍就等于把最坏情况藏起来。
    ★ 饱和标志用 `hold_offset_x=10` 那条任务来造（roll 必然打满）⟹ `sat` 里要有 `"wx"`。
    """
    log_path = tmp_path / "mpc.jsonl"
    with _scenario(state_source="px4", hold_offset_x=10.0, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 15, timeout_s=10.0,
                    message="解锁后没攒到 15 帧 setpoint")

    # ★ 摘要在**进程退出时**才打（`mpc_node.main` 的 `finally`）⟹ 必须在 `with` 之外读：
    #   块内读到的只是"到这一刻为止"的输出，收尾那一行还没出生。台架 `__exit__` 会发 SIGINT、
    #   等进程退出、再 join 读线程，所以这里读到的是完整输出。
    summary = sc.dut_log()

    recs = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()
            if line.strip()]
    assert recs, "逐帧日志一行都没有"
    assert {r["fallback"] for r in recs} >= {"warmup", "normal"}, \
        f"日志里的档位只有 {sorted({r['fallback'] for r in recs})} —— 热身拍没记？"

    for r in recs:
        assert r["state_source"] == "px4", f"状态源记成了 {r['state_source']!r}"
        assert isinstance(r["age_pos_s"], (int, float)), "位置分龄没进日志"
        assert isinstance(r["age_att_s"], (int, float)), "姿态分龄没进日志"
        assert len(r["u"]) == 4 and len(r["x0"]) == 10, "u/x0 的维度不对（求解器接口变了？）"

    normal = [r for r in recs if r["fallback"] == "normal"]
    assert normal, "一行正常帧都没有"
    assert all(r["solve_ok"] and r["solve_ms"] > 0.0 for r in normal), \
        "正常帧里没有求解耗时（票面 D6 那个数字就从这一列来）"
    assert any("wx" in r["sat"] for r in normal), \
        "hold_offset_x=10 该把 roll 顶满 ⟹ 饱和标志里要有 'wx'"

    last = normal[-1]
    z0 = Z0Params()
    assert abs(last["u"][1] - last["roll"]) < 1e-4, \
        f"日志内部量 u[1]={last['u'][1]} 与话题上的 roll={last['roll']} 不符（符号映射只差这一式）"
    assert abs(last["thrust_body_z"] + last["u"][0] / z0.t_max) < 1e-4, \
        "thrust_body_z 与 −F/T_max 不符（归一化改了？）"

    # 收尾摘要：D6 那个数字与饱和计数必须能被**人**读到（不只是进了 JSONL）
    assert "求解耗时 p50" in summary and "最坏" in summary, \
        f"收尾摘要里没有求解耗时的分布（D6 要读它）：{summary[-300:]!r}"
    assert "约束饱和" in summary and "wx" in summary, \
        f"收尾摘要里没有饱和计数：{summary[-300:]!r}"


def test_垂直偏置不跟着解爬(tmp_path):
    """★★ 2026-09-28 的 runaway 回归 —— ticket 20 之后考的是**更强**的那条不变量。

    历史（第一手见 `node_core.hold_feedforward` 的 ★ 两段与
    [`tier-a-sitl-hover.md`](../../../docs/control/tier-a-sitl-hover.md)）：`u_ref` 取
    "上一帧解铺满时域" ⟹ `R·(u − u_ref)` 里解出一点偏、下一帧的参考量就跟着偏 —— 正反馈；
    `offline-ocp.md` §4.4 离线记过"单调上爬、不收敛"，真 PX4 闭环里放大成 runaway
    （`u[3]` 从 0 爬到 **+4.696 rad/s**、EKF 帧转了 **13.6 圈**）。

    ☠️ **ticket 20 推翻了原来那条断言**：垂直通道加了积分偏置之后，`u_ref0` 的推力那一路
    变成 `t_hover + ΔF`（时变）⟹ "`u_ref0` 逐帧恒等于 `[t_hover,0,0,0]`"**不再是恒等式**。
    要守的不变量换成更本质的那条：**`ΔF` 只来自实测位置误差、不来自 `sol.u`**。

    ★ 这条用例是**负控**（正控见 `test_垂直偏置积的是实测位置误差`）：
      任务 `hold_offset = (10, 0, 0)` ⟹ **`e_z ≡ 0`** ⟹ 积分器没有任何东西可积
      ⟹ `ΔF` 必须**逐帧严格为 0**，`u_ref0[0]` 恒等于 `t_hover`（用 `==`，不是 `approx`）。
      而同段里 `u[1]` 顶满（> 1 rad/s）、`u[0]` 也在动 ⟹ "**解在动、前馈纹丝不动**"
      就是"偏置不来自解"的直接读数。
    ★ 任务用 `hold_offset_x = 10 m`：位置误差大到把 roll 顶满 ⟹ 解明显非零。
      少了"同一段里 `u` 真的在动"这半条，测试就没有鉴别力（静态悬停时旧代码也看不出爬）。
    """
    log_path = tmp_path / "mpc.jsonl"
    with _scenario(state_source="px4", hold_offset_x=10.0, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 15, timeout_s=10.0,
                    message="解锁后没攒到 15 帧 setpoint")

    normal = [r for r in _读日志(log_path) if r["fallback"] == "normal"]
    assert len(normal) >= 10, f"正常帧只有 {len(normal)} 拍，测不出「爬不爬」"

    z0 = Z0Params()
    want = [z0.t_hover, 0.0, 0.0, 0.0]
    for r in normal:
        # ★ 两列一起断言：`df` 是积分器的**状态**，`u_ref0` 是它**发出去的样子**
        #   （`df > 0` 而 `u_ref0[0] == t_hover` 这种不一致就是"加进前馈的那一步漏了"）。
        assert r["df"] == 0.0, \
            f"第 {r['tick']} 拍的垂直偏置不是 0：df = {r['df']}（e_z = x_ref0[2]−x0[2] = " \
            f"{r['x_ref0'][2] - r['x0'][2]:.3e}，这一场它必须恒为 0）"
        assert r["u_ref0"] == pytest.approx(want), \
            f"第 {r['tick']} 拍的控制参考变成了 {r['u_ref0']}（角速率三路该恒为 0、推力该是 {want}）"
    assert max(abs(r["u"][1]) for r in normal) > 1.0, \
        "同段的解没动过 ⟹ 这条测试证明不了「前馈没跟着解爬」（任务参数失效？）"


def test_垂直偏置积的是实测位置误差(tmp_path):
    """★ ticket 20 的**正控**：`ΔF` 必须等于"**实测垂直位置误差**的纯积分"，一阶不差。

    与负控是同一条 x 任务（`hold_offset_x = 10` ⟹ `u[1]` 照样顶满、解照样在动），
    **只多一个 z 偏移** `hold_offset_z = 2.0` ⟹ `e_z ≡ +2.0` 永远关不掉（锚点从测量起，
    见 `node_core.hold_reference` 的 ★★）⟹ `ΔF` 该以 `ki_z·e_z` 的斜率线性长上去。

    | # | 判据 | 为什么它有牙齿 |
    |---|---|---|
    | ① | `e_z` 逐帧恒 = `hold_offset_z`（读日志的 `x_ref0[2] − x0[2]`） | 先把"输入是常量"钉住，②的线性才有意义 |
    | ② | `ΔF` 逐帧**严格单调增** | 正号累积（偏低 ⟹ 加推力） |
    | ③ | `ΔF_last − ΔF_first ≈ ki_z·e_z·(t_last − t_first)`（`t` 取日志的 `wall_ns`） | **望远镜求和**：右边 = `ki_z·∫e_z dt` 的离散形式，左边只由两端读数决定 ⟹ 与逐拍抖动无关 |
    | ④ | 全程 `ΔF` 没顶到界（`f_max − t_hover`） | ③ 在饱和段会失真 ⟹ 先排除 |
    | ⑤ | `u_ref0[1:]` 逐帧恒 0 且同段 `u[1]` > 1.0 | runaway 守卫（角速率三路**永远**是常量 0） |
    | ⑥ | ⚠️ **本夹具量不到**：只验"没有中途复位"这个前提（见下 ☠️） | 「钉锚点 ⟹ 复位」在本层**没有端到端读数**（起始那一次之前无历史；此后造不出第二个锚点） |

    ★ 为什么 ③ 能证伪"偏置来自上一帧解"：②③ 量到的是一条**跟着墙钟线性走**的斜坡，
      斜率**正好**是 `ki_z·e_z` —— 那是实测误差的函数。解那一路既不是线性的、也不带 `e_z`
      （同段 `u[1]` 就顶在 3.84 rad/s 上不动）⟹ 任何"把 `sol.u` 接进积分器"的写法都过不了 ③。
    ⚠️ ③ 是**差分**形式 ⟹ 对 `ΔF` 上的**恒定**偏置免疫（比如 OCP 自己的稳态偏置被算进去）。
      原打算由 ⑥（锚点处必须为 0）挡这一头，2026-09-29 实测发现 ⑥ **在本夹具上量不到**
      （☠️ 见下面那段），于是它现在只守"没有中途复位"这个前提 ⟹ **这条恒定偏置的缺口
      在本层是敞着的**，判据① 的独立读数仍以飞行留档为准（`df` 列的稳态值）。
    """
    log_path = tmp_path / "mpc.jsonl"
    # ★ `ki_z` 在这里**显式设成 1.0**、并拿同一个数去算预测值：既让③不依赖"节点当前的默认增益"
    #   （那是产品参数，会变），也不用 `import NodeParams`（seam 1 的纪律：不 import 节点的类）。
    ki = 1.0
    with _scenario(state_source="px4", hold_offset_x=10.0, hold_offset_z=2.0, ki_z=ki,
                   log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 150, timeout_s=15.0,
                    message="解锁后没攒到 150 帧 setpoint（1.5 s，够积分器长起来）")

    z0 = Z0Params()
    normal = [r for r in _读日志(log_path) if r["fallback"] == "normal"]
    assert len(normal) >= 100, f"正常帧只有 {len(normal)} 拍，积分器长不起来"

    e_z = [r["x_ref0"][2] - r["x0"][2] for r in normal]
    # 容差 1e-5：`x_ref0` 与 `x0` 是**各自**四舍五入到 6 位小数之后才落盘的（见 `_读日志`）
    assert all(e == pytest.approx(2.0, abs=1e-5) for e in e_z), \
        f"这一场的 e_z 本该恒为 hold_offset_z = 2.0，实际首尾 {e_z[0]:.9f} → {e_z[-1]:.9f}"

    df = [r["df"] for r in normal]
    assert all(b > a for a, b in zip(df, df[1:])), \
        f"ΔF 没有逐帧严格上升（前 10 拍：{[round(v, 4) for v in df[:10]]}）"
    assert max(abs(r["u_ref0"][1]) for r in normal) == 0.0 and \
        all(r["u_ref0"][2:] == [0.0, 0.0] for r in normal), \
        "偏置漏进了角速率那三路（那三路必须恒为 0）"
    assert max(abs(r["u"][1]) for r in normal) > 1.0, \
        "同段的解没动过 ⟹ 正控不成立（任务参数失效？）"

    hi = z0.f_max - z0.t_hover
    assert max(df) < hi - 1e-6, \
        f"ΔF 顶到了上界（{hi:.4f} N）⟹ ③ 会在饱和段失真：max = {max(df):.4f}"

    dt_s = (normal[-1]["wall_ns"] - normal[0]["wall_ns"]) / 1e9
    assert df[-1] - df[0] == pytest.approx(ki * 2.0 * dt_s, rel=0.05), \
        (f"ΔF 的增量不是 ki_z·e_z·Δt：实测 {df[-1] - df[0]:.4f} N / "
         f"{dt_s:.3f} s，预测 {ki * 2.0 * dt_s:.4f} N")

    # ☠️ **⑥（钉锚点那一拍 ΔF == 0）在这条用例里量不到，2026-09-29 实测后撤掉**，别再加回来：
    #    ① `_tick` 的顺序是**先归零、同一拍立刻积分**（`mpc_node.py:606-616`）⟹ 锚点那一拍
    #       落盘的 `df` **结构上就不是 0**，而是那一拍自己的积分量（实测 `tick=6`：
    #       `fallback=warmup, df=0.020045` = `ki·e_z·dt`）。断"锚点拍 df == 0"必红。
    #    ② 本夹具**结构上造不出第二个锚点**：`fake_px4._build_local_position` /
    #       `_build_attitude` 把 `xy_reset_counter` / `quat_reset_counter` 写死 0、
    #       没有注入口（`src/bench/bench/fake_px4.py:344`/`:352`）⟹ 起始那一次之后
    #       `cycle.anchor` 不会再来。而起始那次之前积分器还没有历史，"复位"与"没复位"
    #       读数相同 ⟹ **端到端不可观测**。
    #    ⟹ 只能验"没有中途复位"这个**前提**本身（真出中途复位时这条会红，提示把 ⑥ 加回来）。
    anchors = [r for r in _读日志(log_path) if r["ref_anchor"]]
    assert anchors, "日志里连起始锚点都没有 ⟹ 夹具或节点有问题（不是 ⑥ 的结论）"
    assert all(r["fallback"] != "normal" for r in anchors), \
        (f"锚点拍混进了正常帧 ⟹ 中途发生了复位（锚点判定/重置计数器变了？）："
         f"{[(r['tick'], r['fallback'], r['df']) for r in anchors if r['fallback'] == 'normal'][:3]}\n"
         f"    那就必须补上⑥「锚点拍 df == 0」（现在因为量不到而撤掉了，见上面的 ☠️）")


def test_垂直偏置顶在界上就不再累积(tmp_path):
    """★ **抗积分饱和**：夹的是**状态** ⟹ 饱和段里 `ΔF` 必须**逐帧恒等**。

    手法：`ki_z := 200`、`hold_offset_z = 5` ⟹ 几拍就顶到上界 `f_max − t_hover = 4.4425 N`。

    判据（全部读日志的 `df` 列）：
      · 每一帧 `ΔF ≤ f_max − t_hover`（**不出界**）；
      · 达到上界之后 `ΔF` **逐帧 `==`**（不是"还在长、只是输出被夹"）——
        这正是"夹状态"与"只夹输出"唯一能从外部区分的地方：
        只夹输出的话这一列会继续单调长，而 `u_ref0[0]` 照样恒 = `f_max`（**看不出来**）；
      · `u_ref0[0]` 逐帧 ≤ `f_max`（状态在界内 ⟹ 发出去的也在界内）。

    ★ 上界口径：`f_max = 11.8 N`（电池可持续推力，`params.py:108`），**不是** `t_max`
      （那是归一化常数 `t_hover / hover_thrust`，默认口径下 46.3 N）—— 见 ticket 20 §8.1 的 ☠️。
    ★ `ki_z = 200` 只是为了"几拍就饱和"（真参数不会这么大，§8.3 的起测点是 1）；
      这条用例量的是**饱和之后的行为**，与增益怎么标定无关。
    """
    log_path = tmp_path / "mpc.jsonl"
    with _scenario(state_source="px4", hold_offset_z=5.0, ki_z=200.0,
                   log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 40, timeout_s=15.0,
                    message="解锁后没攒到 40 帧 setpoint")

    z0 = Z0Params()
    hi, lo = z0.f_max - z0.t_hover, -z0.t_hover
    normal = [r for r in _读日志(log_path) if r["fallback"] == "normal"]
    assert len(normal) >= 30, f"正常帧只有 {len(normal)} 拍"
    assert all(lo <= r["df"] <= hi for r in normal), \
        f"ΔF 出界了（界 [{lo:.4f}, {hi:.4f}]）：{[r['df'] for r in normal if not lo <= r['df'] <= hi][:3]}"
    assert all(r["u_ref0"][0] <= z0.f_max + 1e-6 for r in normal), \
        f"发出去的推力参考超过了 f_max：{max(r['u_ref0'][0] for r in normal):.6f} > {z0.f_max}"

    sat = [r for r in normal if r["df"] > hi - 1e-6]
    assert len(sat) >= 10, \
        f"没造出饱和段（顶到上界的帧只有 {len(sat)} 拍）⟹ 这条测不出抗饱和"
    assert len({r["df"] for r in sat}) == 1, \
        (f"饱和段里 ΔF 还在往上涨（{len({r['df'] for r in sat})} 个不同值）⟹ "
         f"夹的是输出、不是状态：首尾 {sat[0]['df']} → {sat[-1]['df']}")
    # ★ 饱和段里误差仍在 +5（正控：不是"没东西可积了"才不涨）
    assert all(r["x_ref0"][2] - r["x0"][2] == pytest.approx(5.0, abs=1e-5) for r in sat), \
        "饱和段里 e_z 不是 +5 ⟹ 这条测不出「顶住了还在积」"


def test_关掉积分增益时垂直偏置恒为0(tmp_path):
    """★ `ki_z ≤ 0` = **硬关掉**（连状态一起清零），不是"停止累积、留着旧值"。

    为什么这条要单独测：ticket 20 的验收是一条 **A/B 对照**（有/无积分作用各飞一场）。
    若"关掉"只是停下积分、把上一段攒的 `ΔF` 留着，对照组会带着前一组的学习量起飞
    —— 两边比出来的差就不是"积分作用的效果"了，而且**从话题上完全看不出来**。

    判据：`ki_z := 0` + `hold_offset_z = 2.0`（足够大的正误差）下，`df` 列**逐帧严格 0.0**、
    `u_ref0` 逐帧恒等 `[t_hover, 0, 0, 0]` ⟹ 行为与 ticket 20 之前**逐位相同**。
    """
    log_path = tmp_path / "mpc.jsonl"
    with _scenario(state_source="px4", hold_offset_z=2.0, ki_z=0.0,
                   log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 40, timeout_s=15.0,
                    message="解锁后没攒到 40 帧 setpoint")

    z0 = Z0Params()
    normal = [r for r in _读日志(log_path) if r["fallback"] == "normal"]
    assert len(normal) >= 30, f"正常帧只有 {len(normal)} 拍"
    assert all(r["df"] == 0.0 for r in normal), \
        f"关掉之后 ΔF 不是 0：{[r['df'] for r in normal if r['df'] != 0.0][:3]}"
    assert all(r["u_ref0"] == pytest.approx([z0.t_hover, 0.0, 0.0, 0.0]) for r in normal), \
        "关掉之后控制参考不再是 `[t_hover,0,0,0]` ⟹ 「关掉」没有回到旧行为"
    assert all(r["x_ref0"][2] - r["x0"][2] == pytest.approx(2.0, abs=1e-5) for r in normal), \
        "这一场本该有 +2 m 的垂直误差（正控：不是「没东西可积」才保持 0）"


def test_闸门关闭期间垂直偏置冻结(tmp_path):
    """★ ticket 20 §8.2 的第 1 个时点：**闸门关着 ⟹ 冻结**（不积分），但**时钟要推过去**。

    闸门关着时位置误差**不可归因**（状态可能是几秒前那一帧，见 `Cycle` 的 `age_state_s`）
    ⟹ 不许积。而"冻结期间也推时钟"这半条是**防一个更隐蔽的错**：不推的话，恢复的第一拍
    `Δt` 会是**整段静默时长**⟹ 一拍积进去一大坨 —— 那正是判据① 的 8 s 窗口里最不该有的暂态。

    ★ 读数跨的是**两端正常帧**（`fallback == "normal"`），重开闸门后节点会先走一段
      **重入梯**（`warmup`：照常求解、照常积分，只是不发 setpoint，`node_core.cycle` 那层）
      ⟹ 这两端之间隔着"重入梯拍数 + 1"次积分，**不是 1 拍**。本夹具实测那次是
      **5 拍 warmup + 1 拍 normal**（增量为 0.020/0.020/0.027/0.024/0.022/0.018，
      合计 **+0.131379 N**）。所以界**不能**按"1 拍"给 —— 那是我第一版写错的地方（`< 0.05`）。
      界按**机理**给：正确行为是"只积了重入梯那几拍" ⟹ `≪ ki·e_z·空洞长度`；
      两种错法都正好等于右边那个量级 ⟹ `1/4` 是个两头都留 4× 以上的分割点。

    造法：`hold_offset_z = 2.0`（⟹ `ΔF` 以 2 N/s 稳定长）先飞 1 s，然后掐掉
    `vehicle_local_position` 2 s（闸门在 0.5 s 后关，`px4_timeout_s` 默认值）。

    | 读数 | 判据 | 为什么 |
    |---|---|---|
    | 日志里有一段 `wall_ns` 的**空洞** | 空洞 ≥ 0.8 s | 闸门**真的**关了（否则整条用例白跑） |
    | 空洞两端 `ΔF` 之差 | `0 ≤ Δ < ¼ · ki·e_z·空洞长度`（本次 = 0.80 N） | 下界挡住"清零"；上界挡住两种错法 —— 若**不推时钟**，恢复那一拍会积进 `ki·e_z·空洞长度 ≈ 3.2 N`；若**静默期间继续积**，空洞里没有日志行、但恢复后 `ΔF` 会跳同一个量级（`2 N/s × 1.6 s`）。两者都 ≥ 4× 于界 |
    | 闸门关着的那段（话题面） | 零帧 setpoint | 与 `test_状态静默…` 同一条纪律 |

    ⚠️ 两把时钟**不能混**：日志的 `wall_ns` 是 DUT 的墙钟，`sc.t()` 是夹具自己的时间线
      ⟹ 空洞在**日志里**找（不拿 `sc.t()` 去切日志），夹具时钟只用来查话题。
    """
    log_path = tmp_path / "mpc.jsonl"
    ki, e_z = 1.0, 2.0
    with _scenario(state_source="px4", hold_offset_z=e_z, ki_z=ki,
                   log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 100, timeout_s=15.0,
                    message="解锁后没攒到 100 帧 setpoint（1 s，先让 ΔF 长起来）")

        t0 = sc.t()
        sc.timeline.extend(px4_state_silence(sc.px4, t0 + 0.2, duration_s=2.0,
                                             key="local_position"))
        # 闸门在静默 0.5 s 后关（`px4_timeout_s`）⟹ [t0+1.0, t0+1.7] 整段都在关着
        sc.advance_to(t0 + 1.2)
        n_sp = len(sc.px4.frames_between("rates", t0 + 1.0, t0 + 1.7))
        assert n_sp == 0, f"闸门该关着，setpoint 却还在发（{n_sp} 帧）"
        sc.advance_to(t0 + 3.5)

    normal = [r for r in _读日志(log_path) if r["fallback"] == "normal"]
    gaps = [(b["wall_ns"] - a["wall_ns"]) / 1e9 for a, b in zip(normal, normal[1:])]
    i = int(np.argmax(gaps))
    assert gaps[i] >= 0.8, \
        f"日志里没有闸门关闭的那段空洞（最大间隔 {max(gaps):.3f} s）⟹ 这条用例白跑了"

    d_before, d_after = normal[i]["df"], normal[i + 1]["df"]
    assert d_before > 0.1, f"关闭之前 ΔF 还没长起来（{d_before}）⟹ 冻结/清零分不出来"
    assert d_after >= d_before, f"闸门重开之后 ΔF 掉了（{d_before} → {d_after}）⟹ 被清零了"
    # ☠️ 界**不是**"1 拍"：两端正常帧之间隔着闸门重开后的**重入梯**（warmup 拍照积、只是不发，
    #    见 `_tick` 的 `_integrate_bias_z` 只由 `reason is None` 把关、不看 `cycle.publish`）。
    #    本夹具实测 5+1 拍、+0.131379 N ⟹ 按"1 拍 ≈ 0.02 N"给的界（第一版的 `< 0.05`）会假红。
    # 错法下 Δ 的量级 = ki·e_z·空洞长度（不推时钟：恢复那一拍把整段积进去；照积：每拍都积）
    wrong = ki * e_z * gaps[i]
    assert d_after - d_before < 0.25 * wrong, \
        (f"闸门重开前后 ΔF 跳了 {d_after - d_before:.4f} N，超过「重入梯那几拍」该有的量级"
         f"（静默 {gaps[i]:.2f} s ⟹ 界 = ¼·{ki}·{e_z}·{gaps[i]:.2f} = {0.25 * wrong:.3f} N）⟹ "
         f"冻结期间要么没推进时钟、要么一直在积；两种错法的量级都是 {wrong:.3f} N")


def test_兜底期间垂直偏置冻结但不清零(tmp_path):
    """★ ticket 20 §8.2 的第 2 个时点：**兜底 ⟹ 冻结、但不清零**。

    ★ **故意不清零**：兜底帧的输出走 `self._hover_sp`（固定 `t_hover`，**不读**这个偏置）
      ⟹ 偏置此刻不影响输出，只决定"恢复之后从哪儿起步"。清零等于把已经学到的口径差扔掉，
      恢复后要重新收敛几秒 —— 而判据① 的窗口正是"最后一次正常帧往前 8 s"，这几秒会
      **直接算进稳态误差**。清零只留给"钉锚点"那一拍（被控对象/任务变了才该忘）。

    造法：`hold_offset_z = 2.0` ⟹ `ΔF` 稳定长；`inject_solve_fail_after_frames = 20`
      ⟹ 第 20 帧起解被毒化（`_poisoned`：状态码非零 + `u` 全 NaN）⟹ 兜底阶梯接管。

    判据（读日志的 `df` 列，**逐帧 `==`**）：
      · 兜底帧的 `ΔF` 与"最后一拍正常帧"的 `ΔF` **逐位相同**（冻结）；
      · 且那个值 `> 0.1 N`（**没被清零** —— 清成 0 也会让上面那条 `==` 失败，两条一起看才认得出是哪种错）。
    ⚠️ **这一层量不到"恢复之后"**：故障注入的语义是"从第 N 帧起**永远**坏"
      （`_inject_active` = `published >= n`，只增不减）⟹ 本夹具造不出"兜底之后再回到正常帧"。
      "恢复后从哪儿起步"因此**没有端到端读数**，只能靠这里的"冻结且保留"间接守住。
    """
    log_path = tmp_path / "mpc.jsonl"
    with _scenario(state_source="px4", hold_offset_z=2.0, ki_z=1.0,
                   inject_solve_fail_after_frames=20, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(sc.px4.published_frames("rates")) >= 40, timeout_s=15.0,
                    message="注入生效之后没有继续发帧（兜底把发布停掉了？）")

    recs = _读日志(log_path)
    normal = [r for r in recs if r["fallback"] == "normal"]
    fb = [r for r in recs if r["fallback"] in ("hold", "hover")]
    assert normal and fb, f"两个阶段没凑齐：normal {len(normal)} 拍、兜底 {len(fb)} 拍"
    assert normal[-1]["tick"] < fb[0]["tick"], "兜底帧排在正常帧之前 ⟹ 阶段切分的假设不成立"

    d = normal[-1]["df"]
    assert d > 0.1, f"兜底开始之前 ΔF 还没长起来（{d}）⟹ 「冻结」与「清零」分不出来"
    assert {r["df"] for r in fb} == {d}, \
        (f"兜底期间 ΔF 变了：正常帧末尾 {d}，兜底段取到 "
         f"{sorted({r['df'] for r in fb})[:5]} ⟹ 要么还在积（不冻结）、要么被清零了")



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

    ★★ ticket 09（A 档）之后，上面这条结论**只对动捕这一档成立**：
      `vehicle_local_position` **带** `vx/vy/vz` ⟹ 换到 A 档之后 τ 变成话题上
      **可观测**的量（实测 `v = 北向 1 m/s`：τ = 0.05 与 τ = 1e-6 的读数差 0.0232 rad/s，
      **是幅度上的差、不是"有/无"** —— 为什么只能这么判，见那条用例的 ★★）。
      黑盒用例见 `test_A_档_有速度之后_tau_第一次在话题上可观测_配对差分`。
      ⚠️ 两档的差别**只在有没有速度**，不在坐标系 —— 所以"τ 量不了"这句话本身
        一点都不该外推（它从来不是"节点不做外推"，而是"输入里没有 v"）。
    """
    from geometry_msgs.msg import PoseStamped

    assert not hasattr(PoseStamped(), "twist"), \
        "PoseStamped 有 twist 了 ⟹ issue 15 可能已经解决，本文件该补一条 τ 的黑盒测试"
    assert not hasattr(PoseStamped(), "velocity")


def test_位置保持锚点让位置换算第一次在话题层可观测():
    """★★ 这条是**结论被推翻过一次**的用例，别看错：ticket 09 的锚点改动把它反过来了。

    **锚点之前**（`hold_reference(x_meas, …)`，每拍用当拍测量重建参考）：参考与初值都从
    同一个测量点出发 ⟹ 位置差恒为 `hold_offset − v·τ`，与"位置是怎么换算出来的"无关
    （位置换错只是把两个点一起平移）⟹ 位置换算的**任何**错法在话题上都看不出动作差别，
    只能靠 SITL 闭环回答。这正是 ticket 09 存在的理由，也是当时那版用例的名字。

    **锚点之后**（ticket 09，2026-09-28 首飞暴露出来的事）：参考钉在**交班那一拍**测量上，
    与当拍测量无关 ⟹

        `x_ref − x0 = (p_anchor + hold_offset) − (p_meas + v·τ)`

    差里出现了 `p_meas − p_anchor` —— 它**取决于位置是怎么算出来的**。下面用两个相隔 7 m 的
    位置把这条演示出来：同一套 `hold_offset`/`v`/`τ`，位置不同 ⟹ 差**不同**。

    ⇒ 结论：位置那一半约定现在**在话题层与闭环里都能被考**（飞机往错误的方向飞、或者误差
      收敛到一个平移过的点）。闭环仍然是最终判据 —— 但不再是**唯一**的判据。
    ⚠️ 姿态那一半**一直**能在话题层量到（参考姿态取**水平**、而机头朝向与 `hold_offset`
      的方向是一对关系）—— 见 `test_A_档_同一条正前方任务_走哪一根轴由_NED_到_ENU_的换算决定`。
    """
    from nmpc.node_core import ControlConfig, extrapolate_state, hold_reference

    cfg = ControlConfig(hold_offset=(3.0, 4.0, 5.0), tau_s=0.05)
    v_enu = np.array([1.0, -2.0, 0.5])
    p_anchor = np.array([0.0, 0.0, 0.0])
    x_anchor = np.concatenate([p_anchor, np.zeros(3), [1.0, 0.0, 0.0, 0.0]])
    x_ref = hold_reference(x_anchor, cfg, 2)[0][:3]

    gaps = []
    for pos_enu in (np.array([0.0, 0.0, 0.0]), np.array([-7.0, 2.5, -3.0])):
        x = np.concatenate([pos_enu, v_enu, [1.0, 0.0, 0.0, 0.0]])
        gap = x_ref - extrapolate_state(x, cfg.tau_s)[:3]
        # 差里现在同时有"锚点 − 当前"与"偏移 − 外推"两项（旧版只有后一项）
        assert np.allclose(gap, (p_anchor + np.asarray(cfg.hold_offset))
                           - (pos_enu + cfg.tau_s * v_enu)), \
            f"参考与初值的位置差不再是 (锚点+偏移) − (测量+v·τ)：{gap}"
        gaps.append(gap)
    assert not np.allclose(gaps[0], gaps[1]), \
        "两个相隔 7 m 的位置给出了同一个差 ⟹ 位置换算又被抵消掉了（锚点丢了？）"
