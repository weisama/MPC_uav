"""seam 1（**话题上）的集成测试 —— 起真的 DUT 子进程 + 假 mavros 夹具（ticket 06 / 13）。

层次与另外两组测试的分工（别混）：

| 文件 | 被测的接缝 | 起 DUT 吗 |
|---|---|---|
| `test_node_core.py` | 闸门/外推/参考（纯逻辑，零 ROS） | 不起 |
| `test_ocp_*.py` | `(状态, 参考) → u`（零 ROS 的求解器） | 不起 |
| **本文件** | **"话题上出现了什么"** —— QoS、闸门、符号、单位 | **起**（子进程） |

★★ **载体在 ticket 13 §二 换过**：下行从 uXRCE-DDS 的 `/fmu/*`（`px4_msgs`）换成
  **mavros** 的 `/mavros/setpoint_raw/attitude`（`mavros_msgs/AttitudeTarget`），
  闸门源从 `/fmu/out/vehicle_status` 换成 `/mavros/state`。
  ⚠️ **换载体连带着三处静默符号**（本票的三条 N）：`thrust` 变**正号**、`body_rate` 要
  **FLU 原样**（mavros 自己转 FRD ⟹ 我们预先取负 = 转两次 ⟹ pitch/yaw 静默反向）、
  闸门源的**频率**变成 1 Hz。本文件是**唯一**能在台架上把这三条逐条证伪的地方。

★★ 本文件里所有的断言都**只看外部可观测的东西**（话题、QoS、时间戳、进程退出码），
  不 import 节点的类、不读它的成员 —— 这是 seam 1 的定义（`.scratch/phase1/spec.md`）。
  代价是几条验收项**在这一层根本量不了**，那几条**不写成假测试**，而是写成"为什么量不了"
  的一手结论（见本文件最后两条用例：`depth 1` 与 τ）。

★ 外部依赖（缺了会**静默**地变成"夹具什么都没测到"，见 `scenario.py` 的 `_resolve_dut`）：
  · `nmpc` 包已 `colcon build` 且 `install/setup.bash` 已 source（`lib/nmpc/mpc_node.py` 要在）
  · `bench` 包已装（夹具本身）
  · **`~/mavros_ws` 已 source**（`mavros_msgs` 从这里来）；
    ☠️ **`~/px4_msgs_ws` **仍然**必须 source** —— 这条在 §四 里写错过一次，第一手（2026-10-13，
      `--collect-only` 当场炸）修正：
        `ModuleNotFoundError: No module named 'px4_msgs'` ← `import bench` 时抛出，
        而不是某条用例失败。链条是 `bench/__init__.py` → `scenario.py:38` 的
        **模块级** `from .fake_px4 import FakePx4` → `fake_px4.py:46` 的 `px4_msgs.msg`。
      ⟹ "A 档被拒 ⟹ 不再需要 `px4_msgs`"这句话**对 DUT 成立、对夹具不成立**：
        夹具仍然 import 它（`fake_px4.py` 归入待裁，见票 13 §四④ / `config.py` 的 ☠️）。
      ⚠️ 教训的形态值得记：**"某个依赖不再被用"是一个关于 import 图的断言，
        不是一个关于用法的断言** —— 判它要跑一次 `--collect-only`，别推理。
        症状还很毒：**收集期就炸**，所以看起来像"测试环境坏了"而不是"我删错了什么"。
  · acados：**DUT 子进程**要 `ACADOS_SOURCE_DIR`/`LD_LIBRARY_PATH`（`_dut_env()` 会补上
    `~/acados` 这一档默认值）；本进程要 import `nmpc` 也走同一条路。

★★ **别并发跑，也别把 DUT 落在后台**（第一手，2026-09-28）：DUT 的话题名是**全固定**的
  （`/mavros/setpoint_raw/attitude`），ROS 域默认 73 也是全固定 —— 于是任何一个
  "上一次没杀干净的 mpc_node"都会和本文件的用例**同时**往同一条话题上发，
  夹具的记录里就混进了别人的帧（症状是"帧数对不上/频率翻倍"，而断言本身照样绿或红得莫名其妙）。
  同一时刻它们还共用 `$MPC_CODEGEN_DIR` 那个目录（560 KB 的生成代码）——
  两个进程同时判"要重新生成"会撞在一起，报一个与代码无关的
  `RuntimeError: Rendering of model.in.h failed!`（`t_renderer` 退出码非 0）。
  收工前 `pgrep -af mpc_node` 看一眼；`pytest -n`（xdist）也不要用在这个文件上。

★★ **`pgrep -af mpc_node` 会把「别人的」DUT 一起列出来**（第一手，2026-09-28）：
  这个仓库里常有两个会话同时干活，其中一个正在跑 `pytest test -q`，它的 DUT 与我的
  **同一条固定话题**上发帧。看到的症状**不是**报错，而是**数字不对**：
  一条本该 `body_rate_y = +0.5572` 的用例量出了 `body_rate_y = −0.0000, thrust = +0.169`
  —— 那是**默认配置（mocap 档、`hold_offset = 0`）那个 DUT 的悬停帧**混了进来。
  ⚠️ **认这个症状要看 `body_rate_y ≈ 0`（≈ 零角速率），别认 `thrust` 那个数**：
  悬停帧的 `thrust ≈ t_hover/t_max ≈ 0.169` 是个**处处都在**的常态值，拿它当指纹认不出来。
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from bench import Scenario
from bench.timeline import (Event, mocap_pose_jump, mocap_stamp_rewind,
                            silence_for, state_change)
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


#: **首飞冻结的实机参数集**（ticket [`phase4/07`](../../../.scratch/phase4/issues/07-seam1-rerun-real-params.md)
#: 的"第三次应用"）。出处：[`first-flight-config.md`](../../../docs/control/first-flight-config.md)
#: **§2**（`N1`–`N5`，逐条带理由与等级）。
#:
#: ★ 冻结的**形式**是「**除 `N2` 外全部取默认值**」—— 所以首飞命令行里只有**一个** `-p`
#:   （§2 原话），这里也就只有**一个**键。其余三个量（频率 / 悬停油门 / 推力归一化常数）
#:   不是"没写"，是**已经等于节点与模型的默认值**：
#:     · 频率 `N1` = `rate_hz 100` / `heartbeat_hz 50` / `viz_hz 10` ⟹ `NodeParams` 的默认；
#:     · 悬停油门 `0.1589` 与导出量 `t_max = 46.3027 N` ⟹ `Z0Params` 的默认（2026-09-29 实测回填）。
#:
#: ⚠️ **唯一的偏离是 `tau_s`**：节点默认 `0.05`（= z0 实测 ≈50 ms），首飞冻结 **`0.0`**（不补偿，
#:   [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) D9："过补偿比不补偿更危险"）。
#:   ⟹ **换机 / 换档就改这一行**，别在用例里散着写；`test_冻结档的_tau_真的是_0_而不是节点默认的_0_05`
#:   与 `test_preflight_check.py` 的那条守着它，改了会红。
FROZEN_PARAMS: dict = {"tau_s": 0.0}


def _scenario(start_dut: "bool | None" = None, **params) -> Scenario:
    """一个场景：假 mavros + 脚本化动捕 + 真的 DUT 子进程。

    ★★ **DUT 一律按首飞冻结档起**（`FROZEN_PARAMS`，见上）—— 这是 ticket `phase4/07` 的落点：
      本文件从 Phase 4 起就是「**实机档**的 seam 1」，而不再是"Phase 2 的默认档"。
      用例显式写的键**优先**（`{**FROZEN_PARAMS, **params}`）⟹ 探某个参数的用例照样能钉自己的值。

    ★ `start_dut` 是**台架的**开关（`Scenario.__init__`），不是 DUT 的 ROS 参数 —— 必须在这里
      摘出来转发。混进 `params` 的话 DUT 会以"未知参数"启动失败，而 `mpc_node` 的参数白名单
      恰好会把这件事**变成一条清楚的报错**（rclpy 自己不会因为多一个参数报错，所以它选择主动拒）。
      ⚠️ 第一手（2026-09-30）：写成 `**params` 一把抓时，`start_dut=False` 就是这样暴露的。

    ☠️ **2026-10-13（ticket 13 §四③）这里原本写的是 `px4=True`** —— 那是"再起一份
      `fake_px4`"的开关。换载体之后 DUT 的下行/闸门源全在 `/mavros/*` 上 ⟹ 起 `fake_px4`
      只是白起一份没人看的"外部世界"。现在**不起它**，DUT 面对的就是**假 mavros 那个世界**
      （`FakeMavros` 由 `Scenario` **无条件**创建，见 `scenario.py`）。
      ⚠️ 票面 §四③ 想要的是"**新增一个 mavros 档位**"（而不是像这样把 `px4=True` 去掉）——
      那要动 `scenario.py` 的 `Scenario.__init__` 签名，而它**不在本轮的改动范围**里 ⟹
      本轮以"不起 `fake_px4`"达成同一效果，**已在票 13 Comments 里登记**为待收口项。
    """
    return Scenario(dut=DUT, mocap=True, params={**FROZEN_PARAMS, **params},
                    start_dut=start_dut, dut_env=_dut_env())


def _wait_dut_up(sc: Scenario, timeout_s: float = STARTUP_S) -> None:
    """等 DUT **真的起来了**：判据是 **DUT 订上了 `/mavros/state`**，而不是"发布者被发现"。

    ★★ 第一手（2026-09-28，本机）：**发布者被发现远早于节点初始化完成**。
      实测"匹配上发布者"在 ~8 s 就成立，而那一刻节点还在 `OfflineOcp()`
      里面（代码生成 + 编 C；同一个进程启动要 ~8 s，见 `STARTUP_S` 的说明），启动日志还
      压在 rcutils 的缓冲里没进管道。此时给它发 SIGINT 的症状是：
      **日志只剩开头那行 matplotlib 警告（373 字符）、`returncode == 0`、没有 traceback**
      —— 看上去像"收尾摘要没打出来"，其实节点压根没走到收尾路径（SIGINT 落在 `__init__`
      里就退出了）。这条判据不成立时，`test_SIGINT_*` 会红，而其余用例照样绿。

    ☠️ **判据在 ticket 13 §二④ 换过**：原来靠"夹具收到第一帧**心跳**"
      （`/fmu/in/offboard_control_mode`）。心跳那条流**整个删掉了**（飞控从收到的
      `SET_ATTITUDE_TARGET` 自己派生 `OfflineControlMode`）⟹ 现在唯一的"节点活着的黑盒证据"
      是它**订上了闸门源**。
    ★ 为什么这个判据仍然有效（不是退化成"发布者被发现"那个坏判据）：`mpc_node.__init__`
      里 `OfflineOcp(...)`（慢，代码生成 + 编 C）**排在**建订阅**之前** ⟹ 订阅第一次被
      夹具看见，天然在"代码生成编完"之后；而且**与日志缓冲无关**（发现走 DDS，不走 stdout）。

    ☠️☠️ **但它**不**等于"节点已经在跑了" —— 2026-10-13 实测（本机，一次探针跑到
      `t=+8.174 s` 判据成立，随后逐条记下 DUT 的 `/rosout`）：
        · `+8.330 s`（判据成立后 **156 ms**）DUT 才打出第一条启动 info；
        · `+8.341 s`（**167 ms**）才跑出**第一拍** `_tick`（那条"闸门关闭（not_armed）"）。
      对照 `mpc_node.__init__` 的顺序：`create_timer`（`:747`）**排在**建订阅（`:640`）**之后**，
      而 `rclpy.spin()` 又在 `__init__` 之后 ⟹ **判据成立时进程可能还在 `__init__` 里**。
      此时给它发 SIGINT 的症状与"发布者被发现"那个坏判据**一模一样**：
      **日志只剩开头那行 matplotlib 警告、`returncode == 0`、没有 traceback，也没有收尾摘要**
      —— 因为 `main()` 的 `finally` 里那句 `print(f"[mpc_node] 收尾…")` 前面有一道
      `if node is not None`，而 `node = MpcNode(params)` 还没返回（第一手：这一条让
      `test_SIGINT_*` 独立跑也稳定红，见它的 ★★）。
      ⟹ **纪律**：`_wait_dut_up` 只回答"DUT 的图起来了没有"。谁在它返回后**立刻**
        杀进程 / **立刻**读图（`attitude_publishers()` 之类），必须**再等一个"定时器跑过
        至少一拍"的证据** —— 最容易拿到的那个就是**解锁后的一帧 setpoint**
        （`_tick` 只可能 `create_timer` 返回之后才跑 ⟹ 它同时证明了 `__init__` 返回了、
        `spin()` 进去了）。本文件其余用例都在等帧或 `hold(≥1 s)`，只有 `test_SIGINT_*`
        原本是"拿到就杀"，所以只有它会踩。

    ★ 用 `Scenario.wait_for`（而不是别的等待）是为了**失败时拿到证据**：
      `BenchTimeout` 会带上 DUT 的日志尾部 —— "参数名写错、节点启动即退"与"QoS 不兼容、
      永不匹配"在断言里长得一样，但根因完全不同（`scenario.py` 的 `wait_for_subscribers`
      里写着同一条教训）。
    """
    sc.wait_for(lambda: sc.fake.matched_subscribers("state") >= 1, timeout_s=timeout_s,
                message="DUT 起来之后没订上 /mavros/state（闸门源，换了载体）")


def _读日志(path: Path) -> list:
    """逐帧 JSONL → 字典列表（**只读文件**，不碰节点；ticket 20 的四条用例共用）。

    ★ 数字全被 `mpc_node._jlist` 四舍五入到 **6 位小数** ⟹ 任何**两个读数相减**得到的量
      （`e_z = x_ref0[2] − x0[2]`、`ΔF` 的增量）都带 ~1e-6 的量化噪声
      ⟹ 断言别用比它更紧的容差（1e-9 那种会偶发假红）。
    """
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip()]


def _发出去的推力(r: dict, z0: Z0Params) -> float:
    """一帧日志 → **真正发出去的那个推力** [N]（ticket 20 的落点判据要读它）。

    `thrust = F / t_max`（[`sign_map.py`](../nmpc/sign_map.py) 的 `mavros_attitude_target`）
    ⟹ 反解**不带负号**。☠️ **2026-10-13（ticket 13 §二②）这里原来是 `−thrust_body_z × t_max`**
    （uXRCE-DDS 口径下 `thrust_body_z` 是负的）—— 换载体后键名与符号一起换了：
    `thrust_body_z`（负）→ `thrust`（**正**）。⛔ 照旧名字/旧符号去读新日志会得到
    "推力的符号反了"这个结论，而**反的是名字**。反解只在这里写一遍，用例不许自己再写。

    ★ 为什么落点判据非它不可：`u` 是**求解器原样输出**、`df` 是**积分器状态**，两者都不含"加没加上"
      这个信息；只有这一列是"写进那一帧的数"。
    ★ 容差 `abs=1e-4`：`thrust` 与 `u` 都各自四舍五入到 6 位小数，而 `F = thrust × t_max`
      把 `1e-6` 放大成 `t_max·1e-6`（默认口径 `t_max ≈ 46.3` ⟹ ~5e-5），加上 `u[0]` 那 `1e-6`
      ⟹ 比 1e-4 更紧会偶发假红。**这一档由调用方给**（见下面两个用它的用例）。
    """
    return float(r["thrust"]) * z0.t_max


def _该发出去的推力(r: dict, z0: Z0Params, df_used: float) -> float:
    """`_发出去的推力` 的**期望值**：`clip(u[0] + ΔF_used, 0, f_max)`（发布点那次夹取）。

    ★ 夹取写在这里、不写进 `_发出去的推力`：一个是"实际读到的"、一个是"该是的"，
      两边口径不同才分得出"夹错了"与"加错了"。
    ☠️ **`df_used` 必须由调用方给，而且给的是"这一帧发出去时用的那个 ΔF"** —— 它是
      **上一拍**日志行里的 `df`，不是同一拍的那一列（`_normal_frame` 用上一拍积完的值，
      `_integrate_bias_z` 排在它后面）。2026-09-29 台架实测：拿同一拍的 `df` 去算，
      差 `ki_z·e_z·Δt`（`ki=20, e_z=0.15, 100 Hz` ⟹ **0.0300 N**），远大于 `1e-4` 的容差。
      ⟹ 谁把 `df` 这一格写成 `r["df"]`，就会得到一个**系统性差一拍**的断言。
    """
    return float(np.clip(r["u"][0] + df_used, 0.0, z0.f_max))


def _arm_and_offboard(sc: Scenario) -> None:
    """注入"已解锁 + 已切 OFFBOARD"—— 走 `Timeline`（夹具**唯一**那条注入面）。

    ☠️ **2026-10-13（ticket 13 §二③/§四）相比旧版变了三处**：
      · 注入面从 `px4_arm` / `px4_nav_state`（改 `fake_px4` 的 `/fmu/out/vehicle_status`）
        换成 `state_change`（改 `fake_mavros` 的 `/mavros/state` 的 `armed` + `mode`）；
      · 判据从"状态帧上的整数枚举"换成"`state_frames` 末帧的 `armed`/`mode` 字符串"；
      · **等待不再是毫秒级**：`/mavros/state` 是 **1 Hz 定频**（`sys_status.cpp:962-985`），
        `set_state` 只改字段、要等下一拍定时器才发出去 ⟹ 最多等约 1 s。这就是换载体后
        "解锁上升沿最多晚 1 s"那个真问题的来源（§二③ 把 `status_timeout_s` 从 0.75 提到 2.5
        正是为了它）；超时给 5 s 留足余量。

    ★ 为什么排进 `timeline` 而不是直接 `sc.fake.set_state(...)`：票面那条验收项要的是
      "**能按指定时刻注入故障**（为 07 准备好）"。"直接调 setter"是"立刻注入"，
      对 07 那种"跑 3 s 之后让这一拍求解失败"用不上；而排进 timeline 之后，
      **同一套写法**既能在 `t=0` 注入、也能在 `t=+3.0` 注入（`wait_for` / `advance_to`
      在等待期间照常触发事件，见 `Scenario.wait_for` 的 docstring）。
    """
    sc.timeline.extend(state_change(sc.fake, sc.t(), armed=True, mode="OFFBOARD"))
    sc.wait_for(lambda: bool(sc.fake.state_frames)
                and bool(sc.fake.state_frames[-1].msg.armed)
                and str(sc.fake.state_frames[-1].msg.mode) == "OFFBOARD",
                timeout_s=5.0, message="夹具注入的 armed/OFFBOARD 没落到 /mavros/state 上")


# ─────────────────── 新载体（mavros 下行）的取记录助手 ───────────────────
#
# ☠️ **2026-10-13（ticket 13 §四③）**：这一组助手顶替旧的 `sc.px4.published_frames("rates")` /
#   `frames_between("rates", …)` / `rate_hz("rates", …)`（那些读的是 `/fmu/in/vehicle_rates_setpoint`）。
#   新下行是 `/mavros/setpoint_raw/attitude`，夹具侧由 `FakeMavros.attitude_*` 记录。
#   ★ 为什么是**另一条话题**、不能复用 `sc.setpoints()`：`sc.setpoints()` 读的是 Phase 1
#     `control` 的 `/mavros/setpoint_raw/local`（`PositionTarget`），**不是这一条流**
#     （理由见 `config.py` 里 `topics["attitude"]` 的 ★）。

def _sp_frames(sc: Scenario) -> list:
    """至今为止收到的下行帧（`AttitudeSetpoint` 列表，按到达顺序）。"""
    return list(sc.fake.attitude_setpoints)


def _sp_between(sc: Scenario, from_s: float, to_s: float) -> list:
    """某窗口内的下行帧（判"闸门关着时零帧"用它，与旧的 `frames_between("rates")` 同义）。"""
    return sc.fake.attitude_setpoints_between(from_s, to_s)


def _sp_rate_hz(sc: Scenario, from_s: float, to_s: float) -> float:
    """某窗口内实测的下行频率（与旧的 `rate_hz("rates", …)` 同义）。"""
    return sc.fake.attitude_rate_hz(from_s=from_s, to_s=to_s)


# ───────────────────────────── 接口：话题名 + QoS ─────────────────────────────

def test_接口与_QoS_与票面一致():
    """票面验收项：下行接上 mavros 的 `/mavros/setpoint_raw/attitude`，闸门源订 `/mavros/state`，
    QoS 逐个字段对。

    ☠️ **2026-10-13（ticket 13 §四③）这条整体换过载体**：旧版读的是 `/fmu/*` 的两个发布方
      （`rates` / `mode`）。换载体后 DUT 在 mavros 那条话题上是**发布方**、在 `/mavros/state`
      上是**订阅方** ⟹ 断言的**方向也换了**。

    ★ 每条断言盯的东西（错了会长什么样）：
      · **夹具自己的订阅 QoS**（读夹具的 profile，属 seam 1 允许的"读夹具"）：必须是**真 mavros
        那一档**（best_effort / volatile）。⛔ 若谁"图省事"把它改成默认的可靠订阅，
        它会比真 mavros 更**严**：DUT 发 best_effort 时两边不兼容 ⟹ 它一帧都收不到 ⟹
        本用例会在 `wait_for` 上先炸（报"下行一条都没发出来"，**指错方向**），
        而且下面的负控也失去了对照（两个订阅者变成同一档）。
        ★ 换句话说：夹具的收端**必须是全篇最弱的那一档**，"QoS 写错"才拦得住 ——
        见 `fake_mavros.attitude_publishers` 的 ⚠️⚠️ 四格矩阵。
      · **下行真的接上了**：`attitude_publishers() == 1`。
        ☠️☠️ **这一条 2026-10-13 改过口径**：原文写它是"QoS 不匹配就收不到"的**正向判据** ——
        **反了**。兼容性只要求 `offered ≥ requested`，而夹具的收端是最弱的 best_effort
        ⟹ DUT 发 reliable（ROS 2 默认）或 transient_local **照样兼容**，这个数仍然是 1。
        ⟹ 它现在只证"**话题上有发布者**"（话题名写错 / DUT 没起 / 被参数拒了当场退 全都长这样），
        **QoS 写没写对由 ③ 直接读 profile 判**（第一手四格矩阵见 `fake_mavros
        .attitude_publishers` 的 ⚠️⚠️）。
        ⚠️ 顺带：本套栈上**不兼容会在两端各打一条 rmw WARN**
        （`… requesting incompatible QoS. No messages will be sent to it. Last incompatible
        policy: RELIABILITY`）⟹ "QoS 失配完全静默"也是**过强的说法**；但那条 WARN 只在
        发现那一刻打一次，不是判据。
      · **负控**：在同一话题上再挂一个 **RELIABLE** 订阅者。DUT 发的是 best_effort，
        `offered(best_effort) < requested(reliable)` ⟹ **不兼容** ⟹ 它一帧都收不到。
        它把上一条从"随便挂个订阅者都能数到 1"里摘出来（同一份话题、同一段时间，一个收得到、
        一个收不到，差别只在 QoS）。
      · **DUT 的发布方 QoS**：读 DUT 那条话题的发布方 info，reliability/durability 必须是
        best_effort/volatile。⚠️ 只能对这两项下断言：`depth` 在本套栈的发现信息里恒为 0
        （见 `test_fake_px4_selftest.py::test_发现信息里读不回来的那两项_history_与_depth`）。
      · **闸门源那条订阅**：`/mavros/state` 的 writer 是 RELIABLE + TRANSIENT_LOCAL，
        DUT 订上了 ⟹ `matched_subscribers("state") == 1`。
      · **动捕两条**：`mocap` 节点用 `QoS(10)` 可靠发；DUT 若写 best_effort 同样永不匹配。
    """
    from mavros_msgs.msg import AttitudeTarget

    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 3, timeout_s=10.0,
                    message="解锁后下行一条都没发出来 ⟹ 没有帧可用来判 QoS")

        # ① 夹具手上那个订阅就是真 mavros 的那一档（守着下一条的"红得了"）
        q = sc.fake.attitude_sub_qos()
        assert q.reliability == ReliabilityPolicy.BEST_EFFORT, \
            f"夹具在 attitude 上的订阅 reliability 是 {q.reliability!r} —— 该照抄 SensorDataQoS()"
        assert q.durability == DurabilityPolicy.VOLATILE, \
            f"夹具的 durability 是 {q.durability!r} —— SensorDataQoS() 是 volatile"

        # ② 下行接上了（★ 不是 QoS 判据：夹具收端是 best_effort ⟹ 来者不拒，见 docstring 的 ☠️☠️）
        assert sc.fake.attitude_publishers() == 1, \
            ("/mavros/setpoint_raw/attitude 上没有**能接上**的 DUT 发布者 ⟹ 话题名写错 / "
             "DUT 没起 / 被参数拒了当场退出，三者在这个数上长得一模一样"
             "（⛔ 别读成'发端 QoS 与 SensorDataQoS() 不兼容'：兼容规则是 offered ≥ requested，"
             "而夹具收端是最弱的 best_effort，接得住 reliable —— 见 `fake_mavros` 的四格矩阵）")

        # ③ 负控：RELIABLE 订阅者收不到 best_effort 的发布（纯 QoS 差异）
        wrong = []
        _sub = sc.node.create_subscription(
            AttitudeTarget, sc.fake.topics["attitude"], wrong.append,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        assert _sub is not None, "订阅对象要留着引用"
        sc.hold(1.0)
        assert wrong == [], \
            (f"一个 RELIABLE 订阅者收到了 {len(wrong)} 帧 best_effort 的下行 ⟹ 这条负控失效了；"
             "check: DUT 现在发的是不是 RELIABLE（那与本条不冲突，但接口就与票面不符）")

        # ④ DUT 的发布方 QoS
        q = sc.fake.attitude_publisher_qos()
        assert q is not None, "拿不到 DUT 在 attitude 上的发布方 QoS（发现还没跑完？）"
        assert q.reliability == ReliabilityPolicy.BEST_EFFORT, \
            f"DUT 发布方的 reliability 是 {q.reliability!r} —— 票面要求 best_effort（SensorDataQoS）"
        assert q.durability == DurabilityPolicy.VOLATILE, \
            f"DUT 发布方的 durability 是 {q.durability!r} —— 票面要求 volatile"

        # ⑤ 闸门源：DUT 订上了 /mavros/state（QoS 写错就永不匹配，且不报错）
        assert sc.fake.matched_subscribers("state") == 1, \
            "DUT 没接上 /mavros/state（QoS 写错就永不匹配，且不报错）"

        # ⑥ 动捕两条（状态源仍是默认的 mocap 档）
        assert sc.mocap_source.matched_subscribers() >= 1, \
            "DUT 没接上 /mocap/pose_enu（票面要求订 **ENU** 那条，不是 NED）"
        # ★ 速度是**独立的一条话题**（ticket 15）⟹ 位姿订上了**不等于**速度也订上了，
        #   而 QoS 不兼容同样"不报错、只是永不匹配" ⟹ 这一条要有自己的判据。
        assert sc.mocap_source.matched_velocity_subscribers() >= 1, \
            "DUT 没接上 /mocap/velocity_enu（ticket 15 的速度来源，QoS 写错就永不匹配）"


# ───────────────────────────── 闸门：谁在发、谁不发 ─────────────────────────────

def test_未解锁时一帧_setpoint_都没有():
    """票面验收项「未解锁 / 未在 offboard 时**不发布**」。

    ☠️ **2026-10-13（ticket 13 §二④④/§四）这条的"心跳"那一半删掉了**：旧版是
      「心跳**在流**（≥2 Hz）+ setpoint **一帧都没有**」两条反向断言。换载体后
      心跳那条流**整个删掉了** —— 飞控从收到的 `SET_ATTITUDE_TARGET` 自己派生
      `OfflineControlMode`，我们没有第二条要发的东西（§二④ 取 (a) 收成单频）。
      ⚠️ **代价**：现在"不发指令"与"不发心跳"是**同一件事**（见 `mpc_node._gate_closed`
      的 ☠️）—— 闸门关着超过 `COM_OF_LOSS_T` 会让飞控拒绝进 OFFBOARD，这是票面
      「换载体后后果最重的一句话」。
    ★ 正控改由"节点活着 + 订上闸门源"承担（`_wait_dut_up`）：心跳没了之后，
      "没发"和"节点没起来"不再有第三条流可以分开 ⟹ 那条等待就是这个用例的正控。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        sc.hold(2.5)                     # 未解锁状态下跑一段（闸门源 1 Hz，跑够几拍）

        assert _sp_frames(sc) == [], \
            "还没解锁就发了 /mavros/setpoint_raw/attitude（闸门没关严）"
        assert sc.dut_alive(), "未解锁这一段里 DUT 退出了"

        # 闸门关着的原因要能被读到 —— 失败时"为什么没发"是第一个要回答的问题
        log = sc.dut_log()
        assert "闸门关闭" in log, f"DUT 没把闸门原因打进日志（日志尾：{log[-400:]!r}）"
        assert "not_armed" in log, \
            f"闸门关着的理由不是 not_armed（日志尾：{log[-400:]!r}）—— 默认状态是未解锁"


def test_解锁切_OFFBOARD_之后立刻开始发():
    """闸门的**正控**：注入 armed + OFFBOARD 之后必须出现下行帧。

    ★ 与上一条配对读 —— 只断言"解锁后有"无法区分"闸门根本没关"。
    ★ 阈值给 10 s 是刻意的：`/mavros/state` 是 **1 Hz 定频**（`set_state` 要等下一拍定时器
      才发出去），加上 `warmup_cycles` 热身拍（100 Hz 下 50 ms）⟹ 正常一两秒内出现；
      给宽只是不让 CI 抖动变成假红。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        sc.hold(1.5)
        assert _sp_frames(sc) == [], "正控的前提：此刻本来就不该有 setpoint"

        _arm_and_offboard(sc)
        sc.wait_for(lambda: _sp_frames(sc), timeout_s=10.0,
                    message="解锁 + OFFBOARD 之后 10 s 内一帧 setpoint 都没发出来")
        assert sc.dut_alive(), "DUT 在发布之前就退出了"


def test_闸门源静默超过阈值之后停发_恢复之后重开():
    """**按指定时刻注入故障**（票面那条验收项的实做）+ 停机条件在话题层被钉住。

    ☠️ **2026-10-13（ticket 13 §二③/§四）这条整体换过载体，且"心跳照发"那半删掉了**：
      · 静默面从 `px4_status_silence`（`/fmu/out/vehicle_status`）换成 `silence_for(fake, 'state')`；
      · 阈值从 **0.75 s 提到 2.5 s**（§二③ 的裁定）—— `/mavros/state` 只在收到 MAVLink
        HEARTBEAT 时发布、**1 Hz 定频**，而 `0.75 < 1.0` ⟹ `status_stale` 会**恒真**；
      · **"心跳照发"那一半没有了**（心跳流已删）⟹ 窗口内只剩"setpoint 一帧都不能有"。

    ★ 故障**排在时刻上**（`t0 + 0.2`），不是"跑完再说"—— 这是 07 要的接缝。
    ★ 静默时长取 **4.0 s**，明显大于 2.5 s 的阈值（同 `px4_status_silence` 那条纪律：
      时长要**明显大于**阈值，否则跨不过闸门、用例白跑）。
    ★ 恢复之后再断言"又开始发" —— 否则"节点自己死了"和"闸门关着"在断言里长得一样。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: _sp_frames(sc), timeout_s=10.0,
                    message="正控没成立：解锁后一帧都没发")

        t0 = sc.t()
        sc.timeline.extend(silence_for(sc.fake, "state", at_s=t0 + 0.2, duration_s=4.0))

        # 静默 3.5 s 之后看窗口 [t0+3.3, t0+4.2]：阈值 2.5 s 早已越过，仍在静默期内
        # （静默到 t0+4.2 结束）。
        sc.advance_to(t0 + 4.2)
        n_sp = len(_sp_between(sc, t0 + 3.3, t0 + 4.2))
        assert n_sp == 0, f"状态龄超过 status_timeout_s 之后还在发 setpoint（{n_sp} 帧）"
        assert sc.dut_alive(), "DUT 在静默期间退出了"
        assert "status_stale" in sc.dut_log(), \
            f"停发的理由不是 status_stale（日志尾：{sc.dut_log()[-500:]!r}）"

        # 恢复：状态重新流动 ⟹ 闸门重开（先重新热身 `warmup_cycles` 拍，100 Hz 下 50 ms）
        sc.advance_to(t0 + 9.0)
        n_after = len(_sp_between(sc, t0 + 7.0, t0 + 9.0))
        assert n_after > 0, "状态恢复之后没有重新开始发 setpoint（闸门没重开）"


def test_预流_只在那两档发悬停帧_别的关闸原因照旧一帧不发(tmp_path):
    """★★ `offboard_prestream`（ticket 14 二定）：闸门关着时**只**在 `not_armed` / `not_offboard`
    这两档发一帧**悬停**，别的关闸原因照旧一帧不发。

    ★ 它在解决什么：闸门关着 ⟹ 一帧都不发 ⟹ 换载体后 **setpoint 就是心跳** ⟹
      操作者想手动把飞机带进 OFFBOARD 时，飞控因为"1 s 内没收到 setpoint"**拒绝**，
      而且拒绝得毫无线索（`/mavros/state` 上 mode 就是不变）。

    ★★ 这条用例的**鉴别力全在"哪几档发"**上，所以两个极端都要量（缺一半就是假绿）：
      · 只量"开了 ⟹ 有帧"：把名单放宽成**所有**关闸原因也能过 —— 而那一档的后果是
        **安全网消失**（输入不可信时我们仍然在发 ⟹ 飞控不会把我们踢出 OFFBOARD）。
      · 只量"闸门关着一帧不发"：那在**默认档**就成立，什么都没验到。
    ⚠️ 两条负向**不是**同一件事的重复：一条关的是闸门源（`status_stale`：我们的知识过期），
      一条关的是测量（动捕静默 ⟹ `no_mocap`/`mocap_stale`：飞机在哪都不可信）。
      "顺手全放"是最可能的错法，但只放一半的实现也见得到，所以两条各钉一次。

    ★ 帧的**内容**也要量（必须是**悬停**）：`thrust = t_hover/t_max`、三个角速率恒 0。
      ⛔ 若预流发的是"上一帧解"或"零推力"，飞机在 OFFBOARD 里会直接掉或飞 ——
      而话题面上"有帧在流"**照样成立**（这正是它在台架上必须被钉住的原因）。
      ★ 它的**权限**只有"让飞控收下我们"：不在 OFFBOARD 时飞控不消费这一帧的姿态/角速率
      （第一手 PX4 1.14.3 `mavlink_receiver.cpp:1562` / `:1592`；`offboard_control_mode` 那一路
      在 `:1530-1536`），见 `mpc_node.REASONS_PRESTREAM`。

    ★ 用 `log_file` 读 JSONL 判"这一帧走的是预流那条路"（`fallback == "prestream"`）而不是
      只看"有帧"：闸门**开着**时也有帧，两者在话题面上长得一模一样。
    """
    log = tmp_path / "prestream.jsonl"
    z0 = Z0Params()
    hover = z0.t_hover / z0.t_max
    with _scenario(offboard_prestream=True, log_file=str(log), log_every=1) as sc:
        _wait_dut_up(sc)

        # ① `not_armed`（未解锁，人还在准备）—— **应该**有预流帧。
        #    ★ 这一段的所有帧都在**解锁之前**（下面才 arm_and_offboard）⟹
        #      "这一档的预流帧是发在未解锁状态下的"这件事由**构造**保证，不是靠时序假设。
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 5, timeout_s=10.0,
                    message="开了 offboard_prestream，未解锁这一段却一帧都没发")
        sc.hold(0.5)                          # 攒够几帧再判内容
        assert not sc.fake.state_frames[-1].msg.armed, "前提不成立：夹具报告的已经是解锁状态"
        rows = [r for r in _读日志(log) if r.get("fallback") == "prestream"]
        assert rows, ("有帧在流，但逐帧日志里一帧 `fallback=prestream` 都没有 ⟹ "
                      "那些帧不是预流那条路发的（闸门不是关着？）")
        for r in rows:
            assert r["armed"] is False, "这一段都还没解锁，日志里却是已解锁 ⟹ 判据对不上"
            assert r["thrust"] == pytest.approx(hover, abs=1e-4), \
                (f"预流帧的推力是 {r['thrust']}，不是悬停归一化推力 {hover:.4f} —— "
                 "预流必须发**悬停**帧（发上一帧解/零推力会让飞机在 OFFBOARD 里掉或飞）")
            assert r["body_rate_x"] == pytest.approx(0.0, abs=1e-6) \
                and r["body_rate_y"] == pytest.approx(0.0, abs=1e-6) \
                and r["body_rate_z"] == pytest.approx(0.0, abs=1e-6), \
                f"预流帧的角速率不为 0（{r['body_rate_x']}, {r['body_rate_y']}, {r['body_rate_z']}）"
        assert "not_armed" in sc.dut_log(), \
            f"闸门理由不是 not_armed（日志尾：{sc.dut_log()[-400:]!r}）"

        # ② 解锁 + OFFBOARD ⟹ 闸门开，走的是正常帧（不再是预流）
        n0 = len(_sp_frames(sc))
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= n0 + 20, timeout_s=10.0,
                    message="解锁切 OFFBOARD 之后没有正常帧在流（闸门没开）")

        # ③ `status_stale`（闸门源静默 > `status_timeout_s` = 2.5 s）—— **不该**有帧
        t0 = sc.t()
        sc.timeline.extend(silence_for(sc.fake, "state", at_s=t0 + 0.2, duration_s=4.0))
        sc.advance_to(t0 + 4.2)
        n_stale = len(_sp_between(sc, t0 + 3.3, t0 + 4.2))
        assert n_stale == 0, (f"status_stale 期间还在发 {n_stale} 帧 —— 预流的名单被放到了"
                              "「所有关闸原因」上 ⟹ 输入不可信时飞控也不会把我们踢出 OFFBOARD")
        assert "status_stale" in sc.dut_log(), \
            f"停发的理由不是 status_stale（日志尾：{sc.dut_log()[-500:]!r}）"

        # ④ 恢复（正控：上面那句"零帧"不是节点死了），再让**动捕**静默 —— 同样不该有帧
        sc.advance_to(t0 + 9.0)
        n_back = len(_sp_between(sc, t0 + 7.0, t0 + 9.0))
        assert n_back > 0, "状态恢复之后闸门没有重开 ⟹ 上面那条零帧断言没鉴别力"

        t1 = sc.t()
        sc.timeline.extend([
            Event(at_s=t1 + 0.2, action=sc.mocap_source.pause, label="开始：动捕静默"),
            Event(at_s=t1 + 4.2, action=sc.mocap_source.resume, label="恢复：动捕"),
        ])
        sc.advance_to(t1 + 3.5)
        # 窗口从 t1+2.6 起：动捕静默 0.2 s + `mocap_timeout_s` 0.5 s 之后判据才可能成立，
        # 留大一点余量（静默到 t1+4.2 结束，窗口整段都在静默里）。
        n_nomocap = len(_sp_between(sc, t1 + 2.6, t1 + 3.5))
        assert n_nomocap == 0, \
            f"动捕静默（位置不可信）期间还在发 {n_nomocap} 帧 —— 预流的名单里混进了测量那几条"
        log_all = sc.dut_log()
        assert "mocap_stale" in log_all or "no_mocap" in log_all, \
            f"闸门理由里没有 mocap 那几条（日志尾：{log_all[-500:]!r}）"


def test_闸门源超时阈值_2_5_下正常跑不报_stale_而_0_75_下会():
    """★★ **票面 §二③ 点名要的变异测试**（"把参数改回 `0.75` ⟹ 必须变红"）——
    这条把**变异本身**写成用例的第二半，于是它永远在守着，不靠人记得手改一次。

    ★ 机理（票面原话）：`/mavros/state` **1 Hz 定频**，而 `status_timeout_s = 0.75 < 1.0`
      ⟹ 每一拍之间状态龄必然越过 0.75 ⟹ `status_stale` **恒真**、每秒报"飞控失联"并停发。
      2.5 s 是 §二③ 的裁定值（与 Phase 1 同值同理由）。
    ★ **为什么这条断言有鉴别力**：它不是"停发了没有"（两档都会停），而是
      **"健康的 1 Hz 流下会不会报失联"** —— 0.75 会因为"两帧之间空窗 1 s > 0.75"而
      **在每个周期都报一次**，2.5 则一次都不报。改回 0.75 会让**第一半**变红。
    ★ 第二半不是"为了让红"而写的假用例：它**正是**变异测试要证明的那个事实
      （"0.75 下真的会报"），所以留在文件里，下次谁改这个阈值，两边一起说话。

    ⚠️ 只用**一个**健康场景跑两档、中间不注入任何故障：读的就是"健康流会不会误报"。
    """
    def run(timeout_s: float) -> str:
        with _scenario(status_timeout_s=timeout_s) as sc:
            _wait_dut_up(sc)
            _arm_and_offboard(sc)
            sc.wait_for(lambda: _sp_frames(sc), timeout_s=10.0,
                        message="正控没成立：解锁后一帧都没发")
            sc.hold(4.0)                 # 跑够 ≥4 个 state 周期，让 0.75 档暴露
            return sc.dut_log()

    log_ok = run(2.5)
    assert "status_stale" not in log_ok, \
        ("默认 status_timeout_s（2.5）下，健康的 1 Hz 闸门源被误判成失联 ⟹ "
         f"阈值改太小了（§二③：0.75 < 1.0 会恒真）。日志尾：{log_ok[-500:]!r}")

    log_bad = run(0.75)
    assert "status_stale" in log_bad, \
        ("把阈值改回 0.75 之后竟然**没有**报 status_stale ⟹ 这条变异测试没有鉴别力:"
         "要么闸门源不是 1 Hz、要么这条判据已经和它脱钩了。"
         f"日志尾：{log_bad[-500:]!r}")


# ───────────────────────────── 参数：拼错就拒，标准参数放行 ─────────────────────────────

def test_拼错的参数当场拒_而_use_sim_time_放行():
    """`-p` 名字拼错 ⟹ **启动即退 exit 2**；ROS 自带的 `use_sim_time` ⟹ 参数照收、起得来。

    ★ 两半必须都在，因为它们会互相掩盖：
      · 少了"拼错就拒" ⇒ 那个补丁可以被删掉而没人知道（rclpy 对多写的参数**完全沉默**）；
      · 少了"标准参数放行" ⇒ 把 ROS 自带的名字也拒掉。**这条是复核抓出来的**：
        实测 `-p use_sim_time:=true` 曾让节点 exit 2（`未知参数：['use_sim_time']`），
        而 09/11 用 launch 起这个节点时带着它是常态 ⟹ 症状会是"节点起不来"，报错却指向参数名。
    ★ 拒绝那半**排在代码生成之前**，所以它毫秒级退出（实测：日志里只有开头那行 matplotlib 警告）。

    ★★ "放行"那半的判据**不能**用"等到 DUT 订上闸门源"（`_wait_dut_up`）—— 这是第一手踩到的：
      `use_sim_time=true` 而**没有任何节点发 `/clock`** 时，rclpy 的时钟冻在 0，
      100 Hz 定时器**一次都不触发** ⟹ 节点活着、启动日志全打出来了、**一帧都不发**，
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


def test_tau_取_0_起得来_而负的与过大的仍然当场拒():
    """★ 首飞**冻结值** `τ = 0` 必须能启动（ticket `phase4/05` —— D9 那一档，2026-10-09 放开）。

    ★★ 为什么这是"冻结配置能不能执行"的判据、而不是一条普通的参数校验：
      `τ = 0`（**不补偿**）是**实飞起始值**（ticket `phase4/03`：过补偿比不补偿更危险）。
      放开之前**节点自己会拒它**（第一手，2026-09-28：`参数 tau_s 要是正的有限数，收到 0.0`、
      `returncode=2`、一帧都不发）⟹ **冻结清单在飞机上根本执行不了**，
      而"起不来"与"参数/话题装错了"在现场分不开 —— 那正是这条冻结纪律要防的静默失败。
    ★ 三半都要在，少一半就会互相掩盖（理由同上面那条）：
      · **0 放行** —— 冻结值；
      · **负 τ 仍拒** —— 反向外推 = 拿"还没走到"的状态喂控制器；
      · **上界上仍拒** —— τ 到一拍（`0.2 s`）就不再是执行延迟，端点**取开区间**。
    ⚠️ 最后一个坏值取 **`0.2` 本身**（不是 `0.21`）：本票第一版节点侧写的是 `<= 0.2`，
      与 `preflight_check` 的 `< 0.2` 差**一个端点** ⟹ `τ = 0.2` 会"自检红、节点绿"。
      只有把端点喂进去才拦得住这种漂移 —— `0.21` 在两个口径下都会红，**看不见**这件事。
    ★ 正控那半的判据是"启动日志 + 进程还在"，**不是**`_wait_dut_up`（同 `use_sim_time`
      那条的理由：订阅要等 `OfflineOcp` 建完，~8 s）。这里要判的只是**参数校验那一关**；
      "真的跑起来并发了帧"由 τ 的配对差分用例（负控那一路现在就是 `τ = 0`）覆盖。
    ★ 拒绝那两半都**排在代码生成之前**，所以毫秒级退出，不必等 `STARTUP_S`。
    """
    with _scenario(tau_s=0.0) as sc:
        # ★ 判据取**两选一**（`节点启动` 出现 = 过；**进程先退了** = 也不过）：
        #   只等前者的话，失败要空等满 `STARTUP_S`（第一手：121 s 才红），
        #   而"起不来"这条路上原因就在 `dut_log()` 里现成（`启动失败：…`）。
        sc.wait_for(lambda: "节点启动" in sc.dut_log() or not sc.dut_alive(),
                    timeout_s=STARTUP_S,
                    message="τ=0 之后既没启动日志、进程也还在 —— 这一档不该出现")
        assert sc.dut_alive(), f"τ=0 是首飞冻结值，节点却退了：{sc.dut_log()[-300:]!r}"
        assert "节点启动" in sc.dut_log(), \
            f"τ=0 没走到启动日志：{sc.dut_log()[-300:]!r}"

    for bad in (-0.01, 0.2, 0.21):
        with _scenario(tau_s=bad) as sc:
            sc.wait_for(lambda: not sc.dut_alive(), timeout_s=30.0,
                        message=f"tau_s={bad} 是个非法值，节点却还在跑")
            assert sc.dut_returncode() == 2, \
                f"tau_s={bad}：退出码是 {sc.dut_returncode()}，期望 2"
            assert "tau_s" in sc.dut_log(), \
                f"tau_s={bad}：退出时没点名是哪个参数：{sc.dut_log()[-300:]!r}"


# ───────────────────────────── 符号 + 推力归一化 ─────────────────────────────
#
# ☠️ **2026-10-13（ticket 13 §四③）本节的断言对象换过**：旧版断言落在
#   `VehicleRatesSetpoint` 的 `roll/pitch/yaw/thrust_body[2]`（PX4 的 FRD/NED 口径）。
#   新载体 `AttitudeTarget` 的三个字段是 **FLU 原样**（`body_rate.x/y/z`）、推力是**正号**
#   （`thrust`）—— ⟹ 同一个物理指令，**发布出去的字段的符号可能整根反过来**。
#   ⚠️ 这正是本票三条静默失效里的 **N1（推力正负）** 与 **N2（body_rate 转两次）**：
#   拿旧字段名去读新日志会得到"符号反了"这个结论，而**反的是名字**。
#   ★ 专测 N2 的用例见 `test_N2_...`（就在本节末尾）。

def test_位置保持时推力归一化等于悬停油门_角速率近零():
    """票面验收项「推力归一化**从模型导出**」（M2 的结论：不再硬编码 `1/T_max`）。

    ★ 判据是一个**恒等式**：`thrust = F/t_max`（`t_max` = **归一化**常数，不是 `F` 上限；
      ☠️ 换载体前是 `thrust_body[2] = −F/t_max`，那个负号属于旧字段名。现在 `thrust` 是**正号**，
      取负在飞控/mavros 那一侧），而位置保持的稳态 `F ≈ t_hover` ⟹ `thrust ≈ +t_hover/t_max`。
      期望值**从模型算出来**（`Z0Params`），不写死 0.169 —— 写死的话"改了质量、忘了改这里"就永远发现不了。

    ⚠️ 容差**按牛顿定义、再除以 `t_max`**（2026-09-29 订正）：它原来是归一化的 `0.06`，那是
      照着**旧归一化**（`t_max = 11.8 N`）量的第 0 次迭代点量级（出厂点 `F = 7.81 N` ⟹
      归一化 −0.662，与悬停 −0.6235 差 0.04）。`t_max` 换成实测的 46.3 N 后，**同一个物理
      偏差**折到归一化上只剩 1/3.9 ⟹ 还写 `0.06` 就是**比它本该管的事松 3.9 倍**（"假绿"）。
      不是"随手给的"：RTI 的单次迭代 + 参考点就是测量点本身，稳态判据只能用带宽容差
      （求解器输出永远要容差断言，见 `test_ocp_*.py`）。
    ★ 顺手把 `type_mask` 与 `IGNORE_THRUST` 那两位钉住：`IGNORE_THRUST` 一旦被置上，
      **掉高度**而话题面上每帧都正常（票面 §二② 的 N1 那一族）。
    """
    #: 稳态推力与悬停值的允许偏差 [N]。归一化余量 = 本值 / `t_max`。
    #: **这个数不是新挑的**：旧阈值是归一化的 `0.06`，按旧归一化常数 `t_max = 11.8003 N`
    #: 折回物理量就是 `0.06 × 11.8003 = 0.708 N`（≈ 悬停推力的 10 %）。这里原样取那个物理量。
    HOVER_NORM_TOL_N = 0.708

    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint")

        z0 = Z0Params()
        expected = +z0.t_hover / z0.t_max          # ☠️ 正号（旧版这里是 −t_hover/t_max）
        tol = HOVER_NORM_TOL_N / z0.t_max
        f = _sp_frames(sc)[-1]
        assert abs(f.thrust - expected) < tol, \
            f"位置保持的归一化推力 {f.thrust:+.4f}，模型导出的悬停值是 " \
            f"{expected:+.4f}（允许差 {tol:.4f} = {HOVER_NORM_TOL_N} N）—— 正负号对吗？"
        # 位置保持 ⟹ 参考就是测量点 ⟹ 不出力（阈值宽松，只判"没有明显动作"）
        wx, wy, wz = f.body_rate
        assert abs(wx) < 0.5 and abs(wy) < 0.5 and abs(wz) < 0.5, \
            f"位置保持时还在转：body_rate=({wx:+.3f}, {wy:+.3f}, {wz:+.3f})"
        # ★ 掩码：只忽略姿态，**绝不能**顺带忽略推力或角速率（票面 §二②）
        assert f.type_mask & 64 == 0, \
            f"type_mask={f.type_mask} 置上了 IGNORE_THRUST(64) ⟹ 掉高度而话题面看起来正常"


def test_目标在正前方时_body_rate_y_为正_且顶约束表现为打满():
    """票面验收项「符号映射与 ticket 03 一致」+「饱和表现为**打满**，不静默截断」。

    ☠️ **2026-10-13（ticket 13 §四③）断言的字段变了，符号也跟着变**：
      · 旧版断言 PX4 口径的 `pitch = −ω_y` ⟹ **负**；
      · 新载体 `AttitudeTarget.body_rate.y` 是 **FLU 原样** ⟹ 同一个指令在这里是**正**的。
      ⟹ 这条用例现在同时守着两件事：① 模型解出来的**方向**对（ω_y > 0 把机身朝 +x 倾），
        ② 我们**没有预先取负**（取负 = 转两次，mavros 会再翻一次 ⟹ 真机上朝**反**方向冲）。
      ⚠️ 只断"方向"是不够的 —— 幅值也要对（见下一条"打满"）。

    ★ "打满"：目标远在 10 m 外（1 s 时域内不可行）⟹ `ω_y` 顶在 `PITCH_RATE_MAX` 上。
      断言的是**等号附近**（容差 0.05），不是"小于上限" —— 后者对"被谁悄悄按比例缩回去"
      也是绿的。`mpc_node` 里刻意没有 `clip()`，就是为了让这一条量得出来。
    """
    with _scenario(hold_offset_x=10.0) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint")

        frames = _sp_frames(sc)
        first, last = frames[0].body_rate[1], frames[-1].body_rate[1]
        assert last > 3.0, \
            f"目标在 +x 10 m 之外，body_rate.y 却是 {last:+.3f}（符号映射反了 / 我们预先取负了？）"
        assert abs(abs(last) - PITCH_RATE_MAX) < 0.05, \
            f"|body_rate.y| = {abs(last):.4f}，没有顶在 PITCH_RATE_MAX = {PITCH_RATE_MAX} 上" \
            "（饱和度被谁裁剪过？见 mpc_node 的「饱和表现为打满」）"
        # 默认热身 5 拍 ⟹ **第一帧**就已经带修正（否则等于在最需要动的时候朝悬停走）
        assert first > 3.0, \
            f"热身 5 拍之后的第一帧 body_rate.y 只有 {first:+.3f} ⟹ 发出去的还是出厂迭代点"


def test_没热身时第一帧是出厂迭代点_热身之后就不再是():
    """★ 这一条把 `WARMUP_CYCLES` 的**理由**变成可观测的事实（票面要求"首拍不发"）。

    第一手（`test/conftest.py` 的模块 docstring，本机实测）：`SQP_RTI` 每周期只做**一次**
    迭代 ⟹ 全新实例的第 0 次 `solve()` 给的是"从出厂迭代点走一步"的结果，**不是**最优解：

        目标 +x 10 m（应当把 ω_y 打满）：第 0 次 ω_y = 0.0000、F = 7.8111（与悬停一模一样）
                                          第 1 次 ω_y = 3.8400（打满）

    ⇒ `warmup_cycles=0` 时，DUT 会把那一帧**发出去** —— 于是第一帧的 `ω_y ≈ 0`，
      第二帧才打满。这正是"上电第一拍不要发"的实证：在最需要它动的时刻朝悬停走。

    ☠️ **2026-10-13（ticket 13 §四③）读的字段从 `pitch`（= −ω_y）换成 `body_rate.y`（= ω_y）**
      —— 符号跟着翻正（理由见上一条）。下面那段端到端读数**保留原样**（它是 2026-09-28
      在**旧载体**上量的，`pitch` 那一列与 `thrust_body[2]` 那一列都带旧字段的符号/单位）：

        #1 pitch=-0.0000  thrust_body[2]=-0.6235   ← 出厂迭代点（等于悬停）
        #2 pitch=-3.8400  thrust_body[2]=-0.6235   ← 一步就到位（ω_y 打满）
        #3 pitch=-3.8400  thrust_body[2]=-0.8483
        #4 pitch=-3.8400  thrust_body[2]=-1.0000   ← 推力也顶到上限（"打满"的另一种）

    ⚠️ **上面那一列 `thrust_body[2]` 是旧归一化（`t_max = 11.8 N`）下的读数**，别照着它认数
      （2026-09-29 回填 `hover_thrust` 后 `t_max = 46.3 N`）：现在悬停 ≈ **+0.169**、
      "顶到 `f_max`（11.8 N）" ≈ **+0.255**（不再是 ±1.0 —— 归一化常数与上限**已经解耦**，
      `params.py`）。`ω_y`（= 旧的 `−pitch`）那一列不受影响；本用例也只断言 `body_rate.y`。

    ⚠️ 这一条**依赖时序**（"第一帧必须是第 0 次解"）。它的正控是第二帧：如果 DUT 一上来就
      打满，说明热身参数没生效（而不是"第一帧恰好是稳态"）。
    """
    with _scenario(warmup_cycles=0, hold_offset_x=10.0) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 3, timeout_s=10.0,
                    message="解锁后没攒到 3 帧 setpoint")

        frames = _sp_frames(sc)
        w0 = frames[0].body_rate[1]
        w1 = frames[1].body_rate[1]
        assert abs(w0) < 0.05, \
            f"关掉热身之后第一帧 body_rate.y = {w0:+.4f}，本该是出厂迭代点（≈0）—— 热身参数没生效？"
        assert w1 > 3.0, f"第二帧 body_rate.y = {w1:+.3f} ⟹ 没有从「出厂迭代点」走出来"


def test_N2_下行里_body_rate_是_FLU_原样_没有预先取负(tmp_path):
    """★★ **票面 §四③ 点名要的"专测 N2"** —— 这条只测一件事：
    **我们发出去的 `body_rate` 是 FLU 原样**（`pitch/yaw` 那两轴**没有**被我们预先取负）。

    ★ 为什么它必须是**独立**一条、且断言落在**我们发出的那个值**上（票面 §四① 的原话）：
      mavros 自己会把 `body_rate`（FLU）→ FRD 再发下去（`setpoint_raw.cpp:321-331`）。
      **我们再预转换一次 = 转两次 ⟹ pitch/yaw 静默反向**。
      而这个错误**在"最终飞控收到的物理方向"上是看不出来的**（两次取负 = 原样），
      在话题上也只有"**我们发的那一帧里 body_rate_y 的符号**"能证伪 ——
      ✋ 见 [判据全绿也可能是机理反了] 那一族。所以本条的判据是**符号 + 与内部量逐位相等**。

    ★ 两面一起断（都要，缺一半就能被下面的错法蒙混过去）：
      · **话题面**：`hold_offset_x=10` ⟹ 机头 +x 追 10 m ⟹ FLU 的 `ω_y > 0`
        （若被预先取负，这里会是**负**的 ⟹ 红）；
      · **日志面**：日志的 `u` 是**求解器给的 FLU 内部量**，而 `body_rate_x/y/z` 是**发出去**的
        ⟹ 换载体这条路**只该改字段名、不该动一个符号** ⟹ 两者必须**逐位相等**
        （`u[1]→body_rate_x`、`u[2]→body_rate_y`、`u[3]→body_rate_z`）。
      ⚠️ 日志面比话题面更硬：它不依赖"这一段任务恰好把哪根轴顶满"，任何一帧都成立。

    ☠️ 旧载体上"两套约定是一回事"由 `roll=ω_x`、`pitch=−ω_y`、`yaw=−ω_z` 那一式守着
      （见旧 `_cmd_to_flu`）；换 mavros 之后那一式的**取负项整根消失**，所以本用例接替它。
    """
    log_path = tmp_path / "mpc.jsonl"
    with _scenario(hold_offset_x=10.0, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint")

        last = _sp_frames(sc)[-1]
        assert last.body_rate[1] > 3.0, \
            (f"话题上的 body_rate.y = {last.body_rate[1]:+.4f} ⟹ 负号 ⟹ **我们预先取了负**"
             "（mavros 还会再翻一次 = 转两次 = 真机上朝反方向冲）。N2。")

    rows = [r for r in _读日志(log_path) if r["fallback"] == "normal"]
    assert rows, "逐帧日志一行正常帧都没有"
    r = rows[-1]
    for key, idx in (("body_rate_x", 1), ("body_rate_y", 2), ("body_rate_z", 3)):
        assert abs(r[key] - r["u"][idx]) < 1e-6, \
            (f"日志里 {key} = {r[key]:+.6f} 与内部量 u[{idx}] = {r['u'][idx]:+.6f} 不等 ⟹ "
             "换载体这条路**动了符号**（该只改字段名、符号一个都不该动）。N2。")
    # 推力那一头同理：`thrust = F/t_max`（正号），而日志的 u[0] 就是 F [N]
    z0 = Z0Params()
    assert abs(r["thrust"] - r["u"][0] / z0.t_max) < 1e-6, \
        (f"日志里 thrust = {r['thrust']:+.6f}，而 u[0]/t_max = {r['u'][0] / z0.t_max:+.6f} ⟹ "
         "推力那一路的归一化/符号动了（票面 N1）。")


# ───────────────────────── ☠️ 时间戳（本类整条删除，2026-10-13）─────────────────────────
#
# ☠️☠️ **2026-10-13（ticket 13 §四③）：`test_时间戳是墙钟微秒` 已删除 —— 它在新的下行上没有对应物。**
#
#   旧用例断言的是 uXRCE-DDS 那条路的 `VehicleRatesSetpoint.timestamp`（恒为 0 就表示"DUT 没填"）
#   与 `OffboardControlMode.timestamp`：**µs 单位错在 PX4 侧是静默的**
#   （`min(timestamp − time_offset, hrt)` 把它钳回"新鲜"），所以当时只能靠
#   `FakePx4.stamp_unit_guess` 按"相对墙钟的比值"把 µs/ms/ns/零值分开。
#
#   **新载体上这条通路整个不存在了**：mavros 的 `AttitudeTarget.header.stamp` 进
#   `setpoint_raw.cpp` 之后是**给人看的**（转发给 `mavlink` 时另盖时间戳），DUT 那边填不填、
#   填什么单位，**飞控都不读**。⟹ 留着它最多能断言"我们填了个墙钟微秒"，而那**不是任何
#   验收项要的东西** —— 就是票面点名禁止的"绿着但不测任何东西"。
#   ⛔ **不是"暂时跳过"**：真机上这条通路将来也不会有。要再验，验的是别的东西（另开票）。
#
#   仍然被别处覆盖的：`header.stamp` 字段本身**在话题面上**由 §四① 的
#   `AttitudeSetpoint.stamp_ns` 记下来了（`fake_mavros.evidence()` 会回读最后一帧的
#   `stamp_ns`）—— 那只是"这一帧带了个戳"，**不是**单位判据。单位判据随本用例一起作废。


# ───────────────────────────── 进程收尾 ─────────────────────────────

def test_SIGINT_之后打出收尾摘要而不是_traceback():
    """★ 台架**读 stdout 那行摘要**（`mpc_node.main()` 用 `print` 而不是 `get_logger()`，
    因为 SIGINT 那条路上 rclpy 已经把 context 关掉了：发日志只会得到
    "Failed to publish log message to rosout"，而摘要本身看不到）。

    一条**负断言配正控**：没有 `Traceback`，同时**有**那行摘要 ——
    后者证明日志确实抓到了收尾路径（否则"没有 traceback"可能只是因为日志是空的）。

    ★★ **必须先等到"定时器跑过至少一拍"再 SIGINT**（2026-10-13，ticket 13 §四 现场）：
      `_wait_dut_up` 的判据（订上 `/mavros/state`）在 `create_timer` **之前**就成立
      —— 实测判据成立后 ~167 ms 才跑出第一拍（读数与推理见 `_wait_dut_up` 的 ☠️☠️）。
      原来这里写完 `_wait_dut_up` 就 `stop_dut()` ⟹ SIGINT 落在 `__init__` 里、
      `node` 还是 `None`、`finally` 里那句收尾摘要**压根没被执行** ⟹ 这条用例
      **独立跑也稳定红**（不是抖动）。现在先解锁 + OFFBOARD 并等到**真的收到下行帧**：
      那一帧只可能由 `_tick` 发出 ⟹ 它是"`__init__` 已返回、`spin()` 已进入"的**证明**，
      而不是"大概起来了吧"的猜测。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 1, timeout_s=10.0,
                    message="解锁后一帧下行都没有 ⟹ 节点还没真的跑起来，这时 SIGINT 测不到收尾路径")
        sc.stop_dut()
        log, rc = sc.dut_log(), sc.dut_returncode()
        assert "[mpc_node] 收尾" in log, \
            f"SIGINT 之后没有收尾摘要；returncode={rc}；日志共 {len(log)} 字符：\n{log[:2000]}"
        assert "定时器" in log and "闸门" in log, "摘要里应当有定时器拍数与闸门统计"
        assert "Traceback" not in log, f"SIGINT 打出了 traceback\n{log[-2000:]}"
        assert rc == 0, f"SIGINT 之后 DUT 的退出码是 {rc}"


# ───────────────────────────── 频率与 RViz 观测面 ─────────────────────────────

def test_实做频率_setpoint_100_Hz():
    """票面验收项「**100 Hz 定时器**」（☠️ 心跳那一半随 §二④ 一起作废，见下）。

    ☠️ **2026-10-13（ticket 13 §二④，操作者裁定 (a) 单一频率）**：旧版这条叫
      `test_实做频率_心跳_50_Hz_setpoint_100_Hz`，同时量**两个**流 —— `/fmu/in/vehicle_rates_setpoint`
      与 `/fmu/in/offboard_control_mode`（心跳，设计 50 Hz、验收线 "≥2 Hz"）。
      `offboard_control_mode` 是 **uXRCE-DDS 那条路特有**的东西：它存在的唯一理由是
      "uORB 侧的 OFFBOARD 看门狗要求 ≥2 Hz 的心跳"。走 mavros 之后**是 mavros 自己**
      在维持 OFFBOARD（`setpoint_raw` 插件替我们发）、DUT 一个心跳都不发
      ⟹ 那条流、`heartbeat_hz`/`heartbeat_always` 参数、`_publish_mode()` 一起**删除**。
      ⚠️ 删得对：**"≥2 Hz 才不掉 OFFBOARD"这个约束在新载体上不存在**，留一条测它的断言
      就是在测一个我们已经不发的流（"绿着但不测任何东西"）。

    ★ 剩下这一条量的仍然是**设计值有没有真的落在线上**：100 Hz 定时器是不是 100 Hz。
      阈值刻意**只卡下限**：它量的是"定时器是不是按设计在跑 / 是不是每拍都在求解被拖住"，
      不是"Jetson 有多快"。**最坏耗时与降档决定归 [09]/[11]**（票面原文：频率在实测最坏
      耗时后再降档）⟹ 这里卡上限会把那张票该做的判断提前做掉。

    ★ 本机实测（2026-09-28，Orin NX，2 s 窗口、未解锁到解锁都在跑）：**setpoint 100.0 Hz**。
    """
    with _scenario() as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 20, timeout_s=10.0,
                    message="解锁后没攒到 20 帧 setpoint，频率没得量")
        t_from = sc.t()
        sc.hold(2.0)
        sp = _sp_rate_hz(sc, t_from, sc.t())
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
    """悬停归一化推力 —— **从模型算**（不是写死某个字面量，见归一化那条验收项）。

    ☠️ **2026-10-13（ticket 13 §四③）：正号**。旧版这里是 `-t_hover / t_max`（FRD 口径的
      `thrust_body_z`）。新载体 `AttitudeTarget.thrust` 是**正号**（`F/t_max`，取负由飞控做）
      ⟹ 这个助手如果跟着忘记翻，下面每一条用它的断言都会变成"永远不成立"的红
      （好在是红不是绿 —— 反过来的错法才致命）。
    """
    z0 = Z0Params()
    return +z0.t_hover / z0.t_max


def _quad(f) -> tuple:
    """一帧 setpoint 里**会变的那四个数**。

    ☠️ **2026-10-13（ticket 13 §四③）换过**：旧 = `(roll, pitch, yaw, thrust_body_z)`；
      新 = `(body_rate.x, body_rate.y, body_rate.z, thrust)` —— **与 `node_core.frame_is_finite`
      校验的那四个是同一组**（那个函数 2026-10-12 已按 §二② 换过名字，两处必须一起看）。
    ★ `f` 是 `bench.fake_mavros.AttitudeSetpoint`（不是 rclpy 的 msg），它把
      "**我们发出去的原值**"以属性暴露出来（`type_mask` / `body_rate` / `thrust` / `stamp_ns`）。
    """
    return (f.body_rate[0], f.body_rate[1], f.body_rate[2], f.thrust)


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
    ★ 两个阶段的形状**刻意拉开**：`hold_offset_x=10` 让正常解把 `body_rate.y` 打满（+3.84），
      而悬停帧是 `(0, 0, 0, +t_hover/t_max)`（≈ +0.169，**从模型算**，别写死）—— 差两个量级，
      不会"看起来差不多"。
    ★ `hold` 的判据是"**内容原样、时间戳换新**"：PX4 会把最后一帧**锁存**下去
      （`MulticopterRateControl.cpp:177-184`）⟹ "我们不发了"与"飞控沿用旧帧"在飞控侧
      是同一件事，所以"在保持"必须是**发出去**的（否则它在话题上根本不可观测）。
      ☠️ 换载体后"时间戳"那一半读的是 `header.stamp`（`AttitudeSetpoint.stamp_ns`），
      不再是 `VehicleRatesSetpoint.timestamp`（微秒、且 PX4 侧有钳位）—— **判据本身没变**
      （"每次都在前进"），变的只是它的载体与单位。

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
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 20 + hold_frames + 3,
                    timeout_s=10.0,
                    message="注入生效之后没有继续发帧（兜底把发布停掉了？那飞机就撒手了）")
        sc.hold(0.3)                       # 再攒几拍悬停，让"切过去了"这件事确定下来
        frames = _sp_frames(sc)
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

        assert abs(prev.body_rate[1]) > 3.0 and not _is_hover_frame(prev), \
            f"保持开始之前的那一帧不是打满的正常解：{_quad(prev)} ⟹ 正控不成立"
        assert not _is_hover_frame(holds[0]), \
            "『保持』发的是悬停帧 ⟹ 这一档根本没生效（下面几条断言会跟着变成废话）"
        for j, h in enumerate(holds):
            assert _quad(h) == _quad(prev), \
                f"第 {j + 1} 拍保持的**内容**变了：{_quad(h)} ≠ 上一帧可行指令 {_quad(prev)}"
        ts = [f.stamp_ns for f in holds]
        assert len(set(ts)) == len(ts) and ts[0] > prev.stamp_ns, \
            f"保持帧的 header.stamp 没有严格前进：{ts}（复用旧戳会让这一帧被读成陈旧的）"

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
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint，闸门那条的正控不成立")

        t0 = sc.t()
        sc.timeline.extend(mocap_stamp_rewind(sc.mocap_source, t0 + 0.2, seconds=2.0, hold_s=1.0))

        sc.advance_to(t0 + 1.4)                     # 倒退中（0.2 → 1.2 是倒退窗口）
        n = len(_sp_between(sc, t0 + 0.6, t0 + 1.2))
        assert n == 0, f"stamp 倒退期间还发了 {n} 帧 setpoint（判据 4：这种时候宁可不发）"
        assert sc.dut_alive(), "stamp 倒退把节点搞崩了"

        sc.advance_to(t0 + 2.5)                     # 追平：stamp 一次性跳回来
        n_after = len(_sp_between(sc, t0 + 1.6, t0 + 2.5))
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
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 10, timeout_s=10.0,
                    message="解锁后没攒到 10 帧 setpoint，闸门那条的正控不成立")

        t0 = sc.t()
        sc.timeline.extend(mocap_pose_jump(sc.mocap_source, t0 + 0.2,
                                           position=(100.0, 0.0, 0.0), hold_s=1.0))

        sc.advance_to(t0 + 1.4)
        n = len(_sp_between(sc, t0 + 0.6, t0 + 1.2))
        assert n == 0, f"位姿跳变期间还发了 {n} 帧 setpoint（那正是『发错指令』）"
        assert sc.dut_alive(), "位姿跳变把节点搞崩了"

        sc.advance_to(t0 + 2.5)
        n_after = len(_sp_between(sc, t0 + 1.6, t0 + 2.5))
        assert n_after > 0, "源回到原位之后没有重新开始发（判据把自己锁死了？）"

        log = sc.dut_log()
        assert "pose_jump" in log, f"闸门原因没进日志（日志尾：{log[-500:]!r}）"
        assert "100.000" in log, f"跳变判据的读数（差多少米）没进日志：{log[-500:]!r}"


# ─────────────── 期望方向的独立几何（本文件自己算，不从 DUT import）───────────────
#
# ★ 期望值**不从 `node_core` import**（那是 DUT 内部，seam 1 的定义不许碰它）：下面这个
#   助手照 `docs/control/sign-map.md` §2 的定义**自己**把几何算出来。否则"转换写错了"与
#   "断言抄了实现"就分不开了。
#
# ☠️☠️ **2026-10-13（ticket 13 §二⑤ + §四③）：原来那一整套 NED/FRD 换算助手整块删掉了。**
#   删的是 `L_NED_TO_ENU` / `M_FRD_TO_FLU` / `_rot_from_quat` / `_flu_rot` / `_ned_axis_quat`
#   与"带 NED 姿态参数"的 `_need_rate_toward`。
#   **理由**：它们服务的**唯一**对象是"A 档机头朝北"那一组 e2e 用例（下面 §A 档的 ☠️ 段），
#   而 `state_source=px4` 现在**入口当场就拒**（操作者 2026-10-12 裁定取 (b)）⟹ 没有任何
#   活用例还会构造 NED 姿态。⛔ **别为了"以后可能重开"留着** —— 留着的换算助手会被下一个
#   读代码的人当成"活的约定"，而他没有任何用例能验证它。
#   ★ 那套换算本身**仍然有活对象**，只是考的地方换了层：`test_node_core.py`
#     的 `test_px4_位置与速度的_NED_到_ENU`、`test_px4_姿态转换满足旋转矩阵恒等式`、
#     `test_px4_姿态转换与参考实现的手写公式一致_差一个整体符号`、
#     `test_px4_姿态转换在两个物理特例上对得上` 直接对着 `px4_pose_to_enu` / `px4_quat_to_enu`
#     考（那些单测跑的是**仍在**的纯函数，本票一个字没动它们）。

def _w_of(f) -> np.ndarray:
    """话题上那一帧的三个角速率 → **FLU 机体系**角速率 `(ωx, ωy, ωz)`。

    ☠️ **2026-10-13（ticket 13 §四③）：这个助手现在是个恒等映射，那是刻意的。**
      旧版叫 `_cmd_to_flu`，要反一个 `roll = ωx`、`pitch = −ωy`、`yaw = −ωz`
      （PX4 的 FRD 口径）。新载体 `AttitudeTarget.body_rate` **就是 FLU 原样**
      ⟹ 在这里再取一次负 = **N2（转两次）**，真机上 pitch/yaw 静默反向。
    ★ 所以它现在唯一的价值是**把"这条路不该有任何符号"写成一处可被读到的代码**：
      谁哪天在里面加个负号，`test_N2_下行里_body_rate_是_FLU_原样_没有预先取负` 会红。
    """
    return np.array(f.body_rate)


def _need_rate_toward_enu(d_world) -> np.ndarray:
    """把**推力矢量**朝世界方向 `d`（ENU）倾，所需的 **FLU 机体系**角速率方向。

    ★ 纯几何，与 DUT 怎么算无关：`ω_world ∝ ẑ × d̂`（正转把机体 z 朝 `d̂` 拉），再旋进机体系
      `ω_body = R_fluᵀ ω_world`。量纲/增益**不是**要断言的东西（OCP 的增益由代价权重定），
      **方向**才是。
    ★ 为什么这里可以**省掉** `R_fluᵀ`：本文件剩下的用例**全都跑在动捕档**，
      而交班那一拍的姿态是**动捕源的 ENU 单位四元数**（机头 = +x），注入只动速度/位置、
      不动姿态 ⟹ `R_flu = I` ⟹ `ω_body = ω_world = ẑ × d̂`。
      ☠️ 这一点在 A 档下**不成立**（那时机头在 NED 里朝北 ⟹ ENU 里朝 +y），
      而 A 档那几条正是**唯一**需要完整换算的用例 —— 它们随 A 档一起作废了（见上面 ☠️）。
    """
    d = np.asarray(d_world, dtype=float).reshape(3)
    return np.cross(np.array([0.0, 0.0, 1.0]), d / float(np.linalg.norm(d)))


# ─────────── A 档（`state_source=px4`）：**只剩"入口当场拒绝"一条** ───────────
#
# ☠️☠️ **2026-10-13（ticket 13 §二⑤ + §四③）：本节原来的 7 条 e2e 用例全部作废。**
#
#   为什么：A 档读的是 `/fmu/out/vehicle_local_position` + `/fmu/out/vehicle_attitude`
#   —— 那两条是 **uXRCE-DDS 专用**的，随（C）级下行一起换到 mavros 之后**不存在了**。
#   操作者 2026-10-12 裁定取 **(b)**：**判为 uXRCE-DDS 专用、本阶段不可用，入口显式拒绝**
#   （fail loud，不是静默无数据）。原话与理由见票面 §二⑤：
#   "本阶段『A 级回退』的载体是 **Phase 1 的 `control` 包整条链**，不是节点内部的 px4 状态源。
#    把一条死路留着不如让它当场喊出来。"
#
#   ⟹ 这一档**起不来了** ⟹ 任何"跑一场 A 档、看它发什么"的用例都失去了对象。
#     原 7 条的**逐条去向**（映射表的正文在 `docs/control/mpc-node-seam1.md` §13）：
#
#     ① `…只订_px4_那两条_动捕一帧都不订`      → **删**。它考的是"条件订阅"这个**行为**，
#        而那一档**根本不进入建订阅那段代码**（`_validate` 就退了）。`mpc_node` 的模块 docstring
#        已把这条断言的**无对象**状态写死（"☠️ 这一条断言在**本阶段没有活对象**（那一档起不来）"）
#        —— 本文件照它的口径收口。
#     ② `…两条状态各判各的龄`（2 参数）        → **删**。纯逻辑**仍在**、且**仍有活单测**：
#        `test_node_core.py::test_px4_源_只看_px4_的龄_而不是动捕的` +
#        `test_px4_源_姿态单独超时也要停`（它们直接对着 `ControlCore.cycle()` 掐两条流）。
#     ③ `…同一条正前方任务_走哪一根轴由_NED_到_ENU_的换算决定`（2 参数） → **删**。
#     ④ `…锚点让位置换算在话题层可观测`        → **删**。
#     ⑤ `…注入的倾斜_指令方向是把机身拉平的那一边`（3 参数） → **删**。
#        （③④⑤ 的换算面由 `test_node_core.py` 的四条 `px4_*` 单测守着，见上面 ☠️ 段。）
#     ⑥ `…有速度之后_tau_第一次在话题上可观测_配对差分` → **改挂 B 档**（下面
#        `test_速度让_tau_在话题上可观测_配对差分`）—— ticket 15 之后动捕那条路**也有速度**了，
#        而这一条考的是**外推那条通路**，与状态源无关 ⟹ 换来源不丢鉴别力。
#     ⑦ `…垂直速度的外推_表现是推力低于悬停`   → **改挂 B 档**（与原来那条 B 档用例合并成一条，
#        见 `test_垂直速度的外推_表现是推力低于悬停`）。
#     ⑧ `…的逐帧日志把状态源_求解耗时_饱和标志都记下来了` → **改挂默认（mocap）档**，
#        断言的 `state_source` 从 `"px4"` 改成 `"mocap"`。
#     ★ ⑥⑦⑧ 是**保住覆盖率**的那三条：它们考的东西（外推通路、日志三列）**不依赖状态源**，
#       原样跟着 A 档一起删就是白白丢掉 ticket 20 / 票面 D6 的判据。
#
#   ⛔ **不要**为了"把 7 条数回来"去把 A 档重新打开 —— 那要重指到
#      `/mavros/local_position/*` + IMU 姿态，也就是票面 §二⑤ 明确**未采纳**的 (a)，
#      且它**是回退路径、属安全相关件**（要另开票、要问操作者）。

def test_A_档_state_source_px4_入口当场拒绝():
    """票面 §二⑤ 裁定 (b) 的**唯一**落点：`state_source=px4` 必须**入口当场拒绝**。

    ★ 为什么这条值得单开（而不是"反正没人用"）：`state_source` 是**回退路径**上的旋钮。
      留着一条"能起、但读的是一条不存在的话题"的死路，症状是
      **闸门永远关着、一帧都不发**（`no_px4_state`）—— 与"话题名写错"长得一模一样，
      而它是**安全相关**的：操作者以为切到了回退档，实际飞机在等一个永不到来的状态。
      ⟹ 裁定选了 fail loud。本用例就是"它真的 loud 了"的判据。

    ★ 断言的**三层**（缺一层都能假绿）：
      · **退出码非 0** —— 光有报错文字不够：一条 `warn` 之后照常跑起来的实现也满足"打了字"，
        而它会**静默地不开闸门**；
      · **那句拒绝文案** —— 退出码非 0 也可能是别的原因（参数名敲错、import 失败），
        所以钉住 `REFUSE_PX4_STATE_SOURCE` 的**特征串**（`A 档` + `§二⑤` 那个裁定号）；
      · **一帧 setpoint 都没有** —— 这是"拒绝"与"起了但不发"的**行为分界**：
        后者的退出码可能也是 0、日志也干净，只有在话题面才看得出来。

    ⚠️ **不许**去断言"它在拒绝之前还订了/没订什么话题"：拒绝发生在 `_validate`
      （建订阅之前），那是**实现细节**，钉上去等于把"拒绝得早"写死成契约。
      ★ 真正与操作者相关的只有"它拒绝了吗、理由说清了吗"。

    ★ 本机实测（2026-10-13，Orin NX）：退出码 `2`；合并输出（stdout+stderr）里出现
      `[mpc_node] 启动失败：A 档（state_source=px4）在本阶段不可用（ticket 13 §二⑤…）`；
      `/mavros/setpoint_raw/attitude` 上 0 个发布者。
      ⚠️ 那句走的是 **stderr**（`mpc_node.main` 的 `except SystemExit` 分支），而台架的
      DUT 读取线程是 `stderr=subprocess.STDOUT` 合的流 ⟹ `dut_log()` 两路都能读到。
      ★ 退出码 **`2`** 而不是 `1`：`main()` 自己 `return 2`（`SystemExit(消息)` 那条路
      把字符串塞进了 `exc.code`，不能 `int()` 它，所以是**写死**的 2）。
      ⟹ 本用例断的是"非 0"，**不钉 2**：退出码的具体值是实现细节，钉死它会在
      "换一种失败方式"时假红，而那一层（`main` 的错误码约定）不该由这条用例来管。
    """
    with _scenario(state_source="px4") as sc:
        sc.wait_for(lambda: not sc.dut_alive(), timeout_s=STARTUP_S,
                    message="state_source=px4 之后 DUT 一直没退出 —— 入口没有拒它")
        sc.hold(0.5)                 # 让读取线程把最后几行排空（进程没了不等于管道空了）
        rc = sc.dut_returncode()
        log = sc.dut_log()

        assert rc not in (0, None), \
            (f"state_source=px4 之后 DUT 的退出码是 {rc}（该是**非 0**）—— 它是不是"
             f"只是打了句警告就照常跑起来了？（日志尾：{log[-800:]!r}）")
        assert "A 档" in log and "§二⑤" in log, \
            (f"拒绝理由没进日志（要含『A 档』与『§二⑤』这两串），"
             f"日志尾：{log[-800:]!r}")
        assert sc.fake.attitude_publishers() == 0, \
            "入口都拒绝了，却还有人往 /mavros/setpoint_raw/attitude 上发 —— 那个进程是谁？"



# ☠️☠️ **2026-10-13（ticket 13 §四③）：这里原来还有三条 A 档 e2e 用例，删了。**
#   逐条说清**删掉的是什么**、以及**有没有别处接着守** —— 免得下一个人以为"覆盖率没变"：
#
#   · `test_A_档_同一条正前方任务_走哪一根轴由_NED_到_ENU_的换算决定`（2 参数）
#     ——考的是"机头朝**北**（NED 的 x）换算到 ENU 之后朝 +y"这件事在**指令轴**上的后果。
#     **换算那一半**仍有活单测（`test_node_core.py::test_px4_位置与速度的_NED_到_ENU`、
#     `test_px4_姿态转换在两个物理特例上对得上`：NED 单位四元数 ⟹ ENU 机头 +y）；
#     **"OCP 据此发哪一根轴"那一半**在 A 档下**没有活对象**了。
#   · `test_A_档_锚点让位置换算在话题层可观测` —— 位置误差（世界方向）⟹ 倾转方向。
#     这一条在**动捕档**有等价物且更直接：`test_目标在正前方时_body_rate_y_为正_…`
#     （位置误差在 +x ⟹ `body_rate.y > 0`，还多钉了"打满"）。差的那一点是"NED 报进来
#     的位置换算"—— 同上面，换算本身有单测。
#   · `test_A_档_注入的倾斜_指令方向是把机身拉平的那一边`（3 参数）—— 姿态误差 ⟹ 拉平方向。
#     ⚠️ **这一条的覆盖面本条就是丢的**：它在动捕档**没有**等价物
#     （`/mocap/pose_enu` 能注入姿态，但重写成动捕版要重算几何 + 重定跳变闸门的角阈值，
#     那是**新造一条判据**、要实测才敢下阈值 —— 不在本票要落地的范围里）。
#     ⟹ 已在票 13 Comments 的映射表里**单独点名**为"本条丢掉的覆盖"，留给操作者裁定
#     （要回来的话另开票）。⛔ 别在这里写一条"看着像"的替代断言充数。
#
#   ★ 三条的共同点：它们考的都是 **A 档那条源**（NED/FRD 的 `/fmu/out/*`）活着时才有意义的
#     东西。操作者 2026-10-12 裁定 (b)（`state_source=px4` 入口当场拒）之后，
#     "跑一场 A 档看它发什么"这件事**在台架上做不到**了 —— 与其留三条永远红的用例，
#     不如把它们删掉、把丢掉的覆盖面写在明处。


def _速度_外推档跑一场(**params) -> np.ndarray:
    """跑一场"动捕档 + 水平匀速 1 m/s"，返回**末帧发出去的 FLU 角速率**。

    ★ 抽出来是因为下面两条用例（τ 的可观测性、默认档落在哪一端）用的是**同一次跑程**：
      注入也一样（ENU 东 +1 m/s）。各写一份的话，哪天要换注入方向就会只改一处，
      两条用例就悄悄量起了**不同的东西** —— 而它们的结论是**互相引用**的。
    ★★ `**params` 而不是形参 `tau_s`：第二条用例要的恰恰是"**不写** `tau_s`"（走
      `FROZEN_PARAMS` 的冻结档）。写成形参就得传一个值进去，而"传什么值"正是那条用例
      要考的东西 —— 一传就把自己要测的默认档覆盖掉了（**测试自己把自己测没了**）。
    """
    with _scenario(hold_offset_z=0.0, **params) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.mocap_source.set_velocity(vx=1.0)     # ENU 东 1 m/s（与 pose 同 frame，零变换）
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 15, timeout_s=10.0,
                    message="注入速度后没攒到 15 帧 setpoint")
        return _w_of(_sp_frames(sc)[-1])


def test_速度让_tau_在话题上可观测_配对差分():
    """★ ticket 15 之后**动捕这一档也有速度了**（`/mocap/velocity_enu`，`TwistStamped`）
    ⟹ 外推 `x + v·τ` 第一次能在**本票剩下的那一档**上被黑盒考到。

    ☠️ **2026-10-13（ticket 13 §四③）这条原来叫 `test_A_档_有速度之后_tau_第一次在话题上
      可观测_配对差分`，跑在 A 档。** A 档整档被拒（§二⑤）⟹ **改挂动捕档**。
      **判据一个字没改** —— 它考的是**外推那条通路**（`x + v·τ`），与"速度从哪条话题来"
      无关；A 档那一版在 ticket 15 之前是**唯一**能跑它的地方，现在不是了。
      ⚠️ 旧的读数（下面那段 `ω_FLU = [...]`）是在 **A 档**、`v = NED 北向 1 m/s`
        （⟹ ENU 北 +1 m/s）下量的。本条现在注入的是 **ENU 东 +1 m/s** ——
        **不是同一个世界方向**（差 90°），所以那些数**别当成本条的实测基线去反推阈值**。
        能沿用它们的只有一条理由：两场里"要刹住的方向"投影到**机体系**之后都是
        `(0, −1, 0)`（mocap 档机头 +x、速度 +x ⟹ 该朝 −x 倾 ⟹ 绕机体 −y；
        A 档机头朝北、速度朝北 ⟹ 同样绕机体 −y）⟹ **量级可比、方向同轴**。
        ⛔ 但"可比"不是"采过"：本条改挂之后**没有重新采数**。

    ★★ 为什么判据是**两次跑程的差**、而不是"有/没有动作"（第一手，2026-09-28 本机——
      这一条是**被实测推翻过一次**的判据，别改回去）：
      当初按"τ=0.05 有动作、τ≈0 没动作"写，靠的是一个"τ≈0 ⟹ 全零悬停帧"的观测 ——
      而那次观测是**别的会话的 DUT** 发在同一对话题上的（模块 docstring 的 ★★ 记了这次事故）。
      真实的机理是：`v ≠ 0` 时，参考的速度是 0 而状态的速度是 v ⟹ OCP 要**刹住**它，
      这一项**与 τ 无关**；τ 只通过位置那一项进来（`x_ref − x0 = hold_offset − v·τ`）。
      所以 τ 的效果是**幅度上的一点差**，不是"有/无"。实测（`hold_offset_z=0`、
      `v = ENU 水平 1 m/s`、末帧，2026-09-28，**A 档**）：

          τ = 0.05  ⟹ ω_FLU = [3.0e-04, −0.5572, −2.2e-03]
          τ = 1e-6  ⟹ ω_FLU = [3.0e-04, −0.5340, −2.1e-03]
          差        = [0, −0.0232, −2e-04] ⟹ 投影到期望方向 k = **+0.0232**

      即灵敏度 ≈ 0.46 (rad/s) / s 的 τ（在 v = 1 m/s 下）。判据取 k > 0.004
      （离实测 6 倍）与"残差 < 0.3·|差|"（实测 0.009）。
      ⚠️ τ 的**方向**与"刹住速度"的方向是同一个（都是 −v̂）⟹ 方向判据分辨不出 τ，
        能分辨的只有**幅度**。这是本用例唯一的、也是诚实的判据。

    ★ 负控那一路取 `tau_s = 0.0` —— 它就是**首飞冻结值**（ticket `phase4/05` / D9 那一档）。
      ☠️ **2026-10-09 更正**：原写"负控不能写 `0.0`，节点自己在参数校验里就拒了它，
      **这本身是对的**" —— 上半句**过时了**（现已放开），下半句是**判错了**：
      零阶外推不是"无意义的参数"，它是"**过补偿比不补偿更危险**"那条纪律的落点
      （ticket `phase4/03`）⟹ 拒它等于把冻结值挡在门外，而后者正是本票要执行的配置。
      ⟹ 改用 `0.0` 之后，负控量到的是**真正的不外推**，不再是 1e-6 那个近似。
    ★ `hold_offset_z=0` 是**设计意图**（让两跑之间只剩速度相关项），不是"不这样就测不出来"：
      实测 `hold_offset_z` 取 0 与取默认 1.0 时，角速率读数到小数第三位都一样 ——
      "保持 1 m"那一项进的是**推力**（`u[0]`），不是横向角速率。
    """
    w_big, w_tiny = _速度_外推档跑一场(tau_s=0.05), _速度_外推档跑一场(tau_s=0.0)
    d = w_big - w_tiny
    # 要刹住的是 +x 的速度 ⟹ 世界方向 −x̂（`_need_rate_toward_enu` 收的就是"该朝哪边倾"）
    w_need = _need_rate_toward_enu((-1.0, 0.0, 0.0))
    k = float(d @ w_need) / float(w_need @ w_need)
    # ⚠️ 负控那一路标签**跟着实际值走**：2026-10-09 起跑的是 `0.0`（冻结值），不再是 `1e-6` 那个近似
    #   —— 标签留旧会让失败诊断把人往"测的还是旧近似"上带（`/code-review` 2026-10-10 指出，本票顺手改）。
    print(f"\nτ=0.05 ⟹ ω={np.round(w_big, 4)}；τ=0.0 ⟹ ω={np.round(w_tiny, 4)}；"
          f"差 {np.round(d, 4)}（k={k:+.4f}，期望方向 {np.round(w_need, 4)}）")

    assert k > 0.004, \
        f"两种 τ 的差没有落在外推该有的方向上：差 {np.round(d, 4)}、" \
        f"期望方向 {np.round(w_need, 4)}（k={k:+.5f}）—— 外推项没进指令？"
    assert float(np.linalg.norm(d - k * w_need)) < 0.3 * float(np.linalg.norm(d)), \
        f"两种 τ 的差不在一个方向上：差 {np.round(d, 4)}、k·w_need = {np.round(k * w_need, 4)}"


def test_冻结档的_tau_真的是_0_而不是节点默认的_0_05():
    """★ ticket [`phase4/07`](../../../.scratch/phase4/issues/07-seam1-rerun-real-params.md)：
    本文件现在按**实机冻结档**起 DUT（`FROZEN_PARAMS`）—— 这条钉住**那个默认档真的生效了**：
    `_scenario()` **不写** `tau_s` 时，τ 必须是 **`0.0`**（首飞冻结值），不是节点默认的 `0.05`。

    ★ 为什么要单开一条：`test_速度让_tau_在话题上可观测_配对差分` 与那两条
      "推力低于悬停"的用例**都是显式**给 `tau_s` 的 ⟹ 它们证明的是"这个旋钮两端都动得起来"，
      **不**证明"本文件的默认落在哪一端"。少了这条，把 `FROZEN_PARAMS` 从 `0.0` 改回 `0.05`
      会**全绿** —— 而那正是"搬到真机后参数搬错"，本票要挡的就是它。

    ☠️ **2026-10-13（ticket 13 §四③）这条原来跑在 A 档**（`state_source="px4"`）——
      A 档整档被拒（§二⑤）⟹ **改挂动捕档**；它考的是"**本文件的默认档**落在哪一端"，
      而那默认档从头到尾就是动捕档 ⟹ 换过来之后它**比原来更贴题**。
      ☠️ 同时**判据也换了**（原判据在这里已经不成立）：老判据是"冻结档 vs **`0.05`** 必须
      差得出外推该有的量"—— 那在 A 档下能红（A 档没有 `FROZEN_PARAMS` 的默认），
      改挂动捕档之后它就和上一条**逐字重复**了（两条都是 `0.05` vs `0.0`）。
      ⟹ 现在**直接断那句话本身**：**隐含档（不写 `tau_s`）= 显式 `0.0`**。

    ★ 判据 = **三个跑程**、同一个度量（两次跑程的差在"该朝哪边倾"方向上的投影）：
      · **主判据**：隐含（**不写** `tau_s` ⟹ 走 `FROZEN_PARAMS`）对 显式 `0.0` ⟹ 投影 **≈ 0**
        （两者**本来就该是同一档**）。把 `FROZEN_PARAMS` 从 `0.0` 改成 `0.05`，这一条当场红
        （那时 `k_self` 会等于下面那个正控的 `k_ctrl`，比值 ≈ 1 ≫ 0.25）。
      · **正控**：显式 `0.05` 对 显式 `0.0` ⟹ 投影**明显不为 0**。
      ★★ 正控**必须**在这一条里，不能靠隔壁那条用例：隔壁的跑程绿不绿与本条无关，
        而"**这一场的夹具到底分不分得开 0.05 与 0**"是本条断言成立的前提。
        见 [判据全绿也可能是机理反了] 那一族的"先问这个量这一场能不能取到别的值"。
      ★ 阈值取**比例**（`|k_self| < 0.25·|k_ctrl|`）而不是绝对读数：绝对阈值会把实测的
        0.0232 抄死（同本文件"不写死读数"的纪律）；比例阈值只要求"跑程抖动远小于 τ 的信号"，
        而那是上一条用例能成立的前提。⛔ 别把它放宽到 0.5 以上：再宽就与"判据没鉴别力"相邻了。
    """
    w_impl = _速度_外推档跑一场()                 # 不写 tau_s ⟹ 走 FROZEN_PARAMS
    w_zero = _速度_外推档跑一场(tau_s=0.0)         # 显式冻结值
    w_050 = _速度_外推档跑一场(tau_s=0.05)         # 节点默认值（"搬错"时会变成的那个）
    u = _need_rate_toward_enu((-1.0, 0.0, 0.0))   # 同一条注入 ⟹ 同一个期望方向
    u = u / float(np.linalg.norm(u))
    k_self = float((w_impl - w_zero) @ u)
    k_ctrl = float((w_050 - w_zero) @ u)
    print(f"\n隐含档 {FROZEN_PARAMS} ⟹ ω={np.round(w_impl, 4)}；显式 0.0 ⟹ {np.round(w_zero, 4)}；"
          f"显式 0.05 ⟹ {np.round(w_050, 4)}；k_self={k_self:+.5f}、k_ctrl={k_ctrl:+.5f}")

    assert abs(k_ctrl) > 0.004, \
        (f"正控不成立：显式 0.05 与显式 0.0 在期望方向上只差 {k_ctrl:+.5f} "
         f"（差 {np.round(w_050 - w_zero, 4)}）—— 这一场量不出外推的差，"
         "下面那条『隐含 = 0.0』就变成了【测不出任何东西】的绿。查 `tau_s` 有没有进参考。")
    assert abs(k_self) < 0.25 * abs(k_ctrl), \
        (f"隐含档（不写 tau_s）与显式 `tau_s=0.0` 差了 {k_self:+.5f}（正控的量级是 "
         f"{k_ctrl:+.5f}）⟹ `FROZEN_PARAMS` 里不是首飞冻结的 `tau_s = 0.0` 了吗"
         "（[first-flight-config.md] §2 的 N2）？")


def test_垂直速度的外推_表现是推力低于悬停():
    """★ 外推的**另一面**：垂直方向的速度差改的是**推力**，不是角速率（纯几何上 `ẑ × ẑ = 0`）。

    ☠️ **2026-10-13（ticket 13 §四③）：这一条是两条老用例合并来的** ——
      原来有一份跑 A 档（`sc.px4.set_velocity_ned(0,0,−1)`，NED 里"上升"= `vz = −1`）、
      一份跑 B 档（`sc.mocap_source.set_velocity(vz=+1)`，ENU 里"上升"= `vz = +1`）。
      内容**逐字重复**，只差一个来源。A 档被拒（§二⑤）之后 A 档那份失去了对象 ⟹
      合成这一条（保留 B 档的形态与那段 ticket 15 的来历），**判据一个字没动**。

    ★★ 这条顺手把**速度的 z 符号**钉住：注入 ENU `vz = +1`（**上升** 1 m/s），τ = 0.05
      ⟹ `x0` 比参考高 5 cm ⟹ 该**减推力**。符号如果没传到位（或反了），要么推力不动、
      要么更大 —— 一眼分得开。
      ⚠️ 符号在**两条源上的写法本来就不一样**（NED 的"上升"是 `vz = −1`、ENU 是 `+1`）
      ⟹ 合并时**没有**把旧读数直接搬过来：旧 A 档那份的读数（`thrust_body[2] = −0.3974`、
      悬停当时 −0.6235）是**旧归一化 + 旧载体**下的，只留在 `git log` 里。

    ⚠️ **判据跟着物理量走，不跟着归一化走**（2026-09-29 回填 `hover_thrust` 后订正）：
      阈值先按**牛顿**定（`MIN_DELTA_THRUST_N`），再折算成归一化余量 —— 否则归一化常数
      从 11.8 N 换到 46.3 N 之后，**同一个物理推力差**折到归一化上只剩 1/3.9，
      余量会从 4.5 倍缩到 1.15 倍，变成一条随时可能假红的用例。
    """
    #: 外推 5 cm 该带来的推力变化，至少要有这么多 **牛顿**（≈ 悬停推力的 8 %）。
    #: **不是新挑的**：旧阈值是归一化的 `0.05`，按旧归一化常数 `t_max = 11.8003 N` 折回物理量
    #: 就是 `0.05 × 11.8003 = 0.590 N`。折算是 `MIN_DELTA_THRUST_N / t_max` ——
    #: 归一到旧常数 = 0.05（2026-09-28 用的就是这个），归一到实测的 46.3 N = 0.0127。
    MIN_DELTA_THRUST_N = 0.590

    z0 = Z0Params()
    hover = +z0.t_hover / z0.t_max
    min_delta = MIN_DELTA_THRUST_N / z0.t_max
    # ★★ **显式钉 τ**（不跟 `FROZEN_PARAMS` 的冻结档 0.0）：本用例探的是**外推那一条通路**
    #   （`x + v·τ`），而 τ = 0 时它是**恒等变换** ⟹ 读数里剩下的只有"OCP 要把 v 刹到 0"那一项
    #   （与 τ 无关，见 `test_速度让_tau_在话题上可观测_配对差分` 的机理段）。
    #   不钉的话它会**照样绿**、却不再测外推 —— 那是本仓最恨的静默失效。
    with _scenario(tau_s=0.05) as sc:
        _wait_dut_up(sc)
        # 接口面先钉一下：DUT 真的订上了**速度那条话题**（QoS 不兼容就不报错、只是永不匹配）
        assert sc.mocap_source.matched_velocity_subscribers() >= 1, \
            "DUT 没接上 /mocap/velocity_enu —— ticket 15 的速度这条路没通"
        _arm_and_offboard(sc)
        sc.mocap_source.set_velocity(vz=1.0)      # ENU 向上 1 m/s（与 pose 同 frame，零变换）
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 15, timeout_s=10.0,
                    message="注入速度后没攒到 15 帧 setpoint")
        last = _sp_frames(sc)[-1]

        wx, wy, _wz = last.body_rate
        # ☠️ **`<` 不是 `>`**（2026-10-13，ticket 13 §四 现场实测后订正）：换载体时这一条
        #   连着**符号**一起搬，"比悬停**低**"在新口径下是 `<`，而它当时被写成了 `>`
        #   ⟹ 独立跑也稳定红（实测 `thrust = +0.0976` vs 悬停 `+0.1589`）。
        #   为什么老口径是 `>`：uXRCE-DDS 那条路上记的是 `thrust_body_z`，**NED/FRD 朝下 ⟹ 负**
        #   —— 在负号口径下"推力更小"= 数值更**大**（−0.0976 > −0.1589）。N1 把推力改成
        #   **正号**（`F/t_max`）之后，同一个物理结论就翻成了 `<`。
        #   ⚠️ 教训的形态：**符号换了，比较方向也要跟着换** —— 而"符号"这一类改动
        #   **不会**让一条方向写反的断言变红，它只会让**真结论**看起来像失败（这条当年就是这么红的）。
        assert last.thrust < hover - min_delta, \
            (f"向上 1 m/s ⟹ 外推点比参考高 ⟹ 该**减**推力，实测 {last.thrust:+.4f}"
             f"（悬停 {hover:+.4f}、要求低出 {min_delta:.4f} = {MIN_DELTA_THRUST_N} N）"
             "—— /mocap/velocity_enu 的速度没进外推？（⛔ 新载体的 `thrust` 是**正**号，"
             "所以'减推力'是**小于**悬停；旧口径 `thrust_body_z` 是负的，那时是 `>`）")
        assert abs(wx) < 0.02 and abs(wy) < 0.02, \
            f"垂直方向的速度差不该产生角速率（ẑ × ẑ = 0），实测 ω=({wx:+.4f}, {wy:+.4f})"


def test_逐帧日志把状态源_求解耗时_饱和标志都记下来了(tmp_path):
    """票面验收项「日志记录 MPC（模型预测控制）内部状态、**求解耗时**、**约束饱和标志**，
    供飞行后复盘而不只依赖记忆」—— 这条验收项以前**没有任何用例**，而它的失效方式是静默的：
    键名写错、值不能 JSON 序列化（`float32` / `uint8`）、某一档忘了合并进去，
    症状都是"飞行之后打开日志发现少了一列"，那时已经晚了。

    ★ 判据分三层，**都对着文件读**（不读节点内部）：
      · **结构**：每行都是合法 JSON、`u` 是 4 维、`x0` 是 10 维（`u` 是求解器给的、
        `x0` 是喂进去的、`x_ref0` 是要它去哪 —— 复盘时靠这三列把一帧重算回去）；
      · **取证面**：`state_source` 与状态源的**分龄**（这一档是 `age_state_s` + `vel_cb`）
        —— 这正是票面要的"谁给的、多旧"；
      · **与话题的一致性**（最要紧的一条）：同一帧里 `u` 是 FLU/ENU 的内部量、
        而 `body_rate_x/y/z` 与 `thrust` 是**发出去那一帧**的量。这一条把"映射写错了"与
        "模型解错了"分开，也正是 `docs/control/sign-map.md` 要的东西。

    ☠️☠️ **2026-10-13（ticket 13 §四③）这条原来跑在 A 档**（`state_source="px4"`），
      随 A 档被拒（§二⑤）改挂**默认（动捕）档**。连带**两处**必须一起换，别只换一处：
      · **取证面**：A 档记的是 `age_pos_s` / `age_att_s`（`node_core.describe_state` 的
        **px4 分支**才并进来的两格）。动捕档**根本没有这两格** ⟹ 照旧键名去读会 `KeyError`，
        或者更坏 —— 有人"顺手"给它填个 0，那就变成一条**永远绿**的假取证。
        动捕档的对应面是 `age_state_s`（状态龄）+ `vel_cb`（速度那条话题收了多少帧，
        ticket 15）。★ `vel_cb` 特别值得钉：`v` 恒为 0 时"没速度"与"静止"**在数值上分不开**，
        只有这一列能分开（`describe_state` 那段 ★ 说的就是这件事）。
      · **"与话题的一致性"那一式**：旧版断的是"日志内部量 `u` 与话题上的 `roll/pitch/yaw` /
        `thrust_body_z` **只差 `sign_map` 那一式**"（`u[1] == roll`、`thrust_body_z == −u[0]/t_max`）。
        ☠️ 那两式在**旧字段名**下是一条**要验的映射**；在新字段名（FLU 原样 + 正号推力）下
        它**恰好是恒等式** —— 而"恒等式"正是本票的结论之一：我们**没有**在下行那一侧
        再转一次 / 再取一次负（N1/N2）。所以这里**原样保留 `u[1] == body_rate_x` 这一式**，
        只是把"只差那一式"这句话改成了"**逐位相等**"。

    ★ 热身拍也**照记**（`fallback == "warmup"`）：那几拍不发，却是"最坏求解耗时"里最容易漏掉的
      （RTI 冷启动、代码页刚进 cache）—— 实测最坏耗时是票面 D6 那个数字的来源，
      漏掉冷启动那几拍就等于把最坏情况藏起来。
    ☠️ **饱和标志那一档在 2026-10-13 从 `wx` 换成了 `wy`** —— 不是笔误，是**换档带来的轴变了**
      （第一手：本用例在动捕档下 `sat` 里恒无 `"wx"`、`max|u[1]|` 恒为 0 ⟹ 那条"该把 roll 顶满"
      的断言在动捕档下**测的是一条恒为零的轴**，红得有理）。机理：`hold_offset_x = +10`
      把参考放到 **ENU +x（正东）**，而"往那边飞要转哪个轴"由**机头朝哪儿**决定 ——
        · A 档（`fake_px4`）：姿态四元数是 NED/FRD 里的单位四元数 ⟹ 机头朝**北** ⟹
          body `+x`（前）指向 ENU `+y` ⟹ **ENU `+x` 在机体的右手边** ⟹ 目标是"右边 10 m"
          ⟹ 压 **roll**（`u[1]` / `wx`）；
        · 默认（动捕）档：`/mocap/pose_enu` 的单位四元数 ⟹ body 轴 = ENU 轴 ⟹ 机头朝**东**
          ⟹ 同一个方向在**正前方** ⟹ 压 **pitch**（`u[2]` / `wy`）。
      ⟹ 同一句"往正东 10 m"在两档下**练的是不同的轴**，换档时这类断言必须逐条重新对轴
        （同族：`test_垂直偏置不跟着解爬` / `test_垂直偏置积的是实测位置误差` 的 `u[1]→u[2]`）。
    ★ 饱和标志用 `hold_offset_x=10` 那条任务来造（动捕档下该把 pitch/`wy` 顶满）
      ⟹ `sat` 里要有 `"wy"`。
    """
    log_path = tmp_path / "mpc.jsonl"
    with _scenario(hold_offset_x=10.0, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 15, timeout_s=10.0,
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
        assert r["state_source"] == "mocap", f"状态源记成了 {r['state_source']!r}"
        assert isinstance(r["age_state_s"], (int, float)), "状态源的龄没进日志"
        assert isinstance(r["vel_cb"], int) and r["vel_cb"] > 0, \
            f"速度那条话题的收帧数没进日志（vel_cb={r.get('vel_cb')!r}）—— 没有它，" \
            "『速度为 0』与『速度话题一帧都没到』在日志上分不开"
        assert len(r["u"]) == 4 and len(r["x0"]) == 10, "u/x0 的维度不对（求解器接口变了？）"

    normal = [r for r in recs if r["fallback"] == "normal"]
    assert normal, "一行正常帧都没有"
    assert all(r["solve_ok"] and r["solve_ms"] > 0.0 for r in normal), \
        "正常帧里没有求解耗时（票面 D6 那个数字就从这一列来）"
    assert any("wy" in r["sat"] for r in normal), \
        ("hold_offset_x=10 在**动捕档**下该把 pitch 顶满 ⟹ 饱和标志里要有 'wy'"
         "（⛔ 不是 'wx'：动捕档机头朝东，正东那条参考练的是 pitch，见上面 ☠️）")

    last = normal[-1]
    z0 = Z0Params()
    # ★ 「日志内部量」与「发出去的那个值」必须**逐位相等**（见上面第二处 ☠️）：
    for idx, key in ((1, "body_rate_x"), (2, "body_rate_y"), (3, "body_rate_z")):
        assert abs(last["u"][idx] - last[key]) < 1e-4, \
            (f"日志内部量 u[{idx}]={last['u'][idx]} 与发出去的 {key}={last[key]} 不等 ⟹ "
             "下行那一侧动了符号/换了轴（FLU 原样这条路被谁转了一次？）")
    assert abs(last["thrust"] - last["u"][0] / z0.t_max) < 1e-4, \
        f"发出去的 thrust={last['thrust']} 与 u[0]/T_max={last['u'][0] / z0.t_max} 不等" \
        "（归一化或符号改了？）"

    # 收尾摘要：D6 那个数字与饱和计数必须能被**人**读到（不只是进了 JSONL）
    assert "求解耗时 p50" in summary and "最坏" in summary, \
        f"收尾摘要里没有求解耗时的分布（D6 要读它）：{summary[-300:]!r}"
    assert "约束饱和" in summary and "wy" in summary, \
        f"收尾摘要里没有饱和计数（动捕档该出现 wy，见上面的 ☠️）：{summary[-300:]!r}"


def test_垂直偏置不跟着解爬(tmp_path):
    """★★ 2026-09-28 的 runaway 回归 —— ticket 20 之后考的是**更强**的那条不变量。

    历史（第一手见 `node_core.hold_feedforward` 的 ★ 两段与
    [`tier-a-sitl-hover.md`](../../../docs/control/tier-a-sitl-hover.md)）：`u_ref` 取
    "上一帧解铺满时域" ⟹ `R·(u − u_ref)` 里解出一点偏、下一帧的参考量就跟着偏 —— 正反馈；
    `offline-ocp.md` §4.4 离线记过"单调上爬、不收敛"，真 PX4 闭环里放大成 runaway
    （`u[3]` 从 0 爬到 **+4.696 rad/s**、EKF 帧转了 **13.6 圈**）。

    ☠️ **ticket 20 两次改过这条断言，别照抄旧版**：
      ① 第一版把积分偏置加进 `u_ref[:,0] = t_hover + ΔF` ⟹ "`u_ref0` 逐帧恒等于
         `[t_hover,0,0,0]`" 不再是恒等式；
      ② 2026-09-29 落点搬到**发出去的指令**上之后，`u_ref0` 又**真的是常量**了（`__init__` 建一次）
         —— 于是这一列对"偏置别跟着解爬"**没有鉴别力**：它常量是结构使然，与 `ΔF` 无关。
      ⟹ 现在守的是**两列**：`df` 必须逐帧严格为 0，**且**发出去的推力必须逐帧等于
         `clip(u[0] + ΔF, 0, f_max)`（`ΔF` 既没漏加、也没从别处来）。要守的不变量还是那条：
         **`ΔF` 只来自实测位置误差、不来自 `sol.u`**。

    ★ 这条用例是**负控**（正控见 `test_垂直偏置积的是实测位置误差`）：
      任务 `hold_offset = (10, 0, 0)` ⟹ **`e_z ≡ 0`** ⟹ 积分器没有任何东西可积
      ⟹ `ΔF` 必须**逐帧严格为 0**，`u_ref0` 恒等于 `[t_hover,0,0,0]`（用 `==`，不是 `approx`）。
      而同段里 `u[2]` 顶满（> 1 rad/s）、`u[0]` 也在动 ⟹ "**解在动、前馈纹丝不动**"
      就是"偏置不来自解"的直接读数。
    ⚠️ `ΔF ≡ 0` 这一场里"落点对不对"是**量不出来**的（加 0 与不加 0 一样）⟹ 落点那条判据的
      真读数在正控里（那边 `df ≠ 0`）。这里留着它只为"落点搬了一半"（比如加在了 `u` 上
      却又乘了系数）能当场红。
    ★ 任务用 `hold_offset_x = 10 m`：位置误差大到把**角速率**顶满 ⟹ 解明显非零。
      少了"同一段里 `u` 真的在动"这半条，测试就没有鉴别力（静态悬停时旧代码也看不出爬）。
      ☠️ **哪条角速率**由档位决定：A 档（机头朝北）下是 **roll（`u[1]`/`wx`）**，
      默认动捕档（机头朝东）下是 **pitch（`u[2]`/`wy`）** —— 轴与理由见
      `test_逐帧日志把状态源_求解耗时_饱和标志都记下来了` 的 ☠️。本组三处的
      `u[1] → u[2]` 就是这么来的（2026-10-13 实测：动捕档下 `max|u[1]| ≡ 0`）。

    ☠️ **2026-10-13（ticket 13 §四③）这一组（垂直偏置）整组换过档**：原来跑在 A 档
      （`state_source="px4"` + `fake_px4`），随 §二⑤ 的 A 档拒绝改挂**默认（动捕）档**
      （即 `_scenario` 里不再传 `state_source`）。★ 这一整组量的都是 `ki_z` 积分器
      —— 它在 `node_core` 里**与状态源无关**（误差由锚点参考与 `x0` 相减得到，两档走同一段代码）
      ⟹ 换档**不改变任何一条判据的语义**，只换"状态从哪来"。⚠️ 但两档的**龄**不同
      （A 档两条消息各有龄；动捕档只有 `age_state_s`），所以凡是读龄的断言都得跟着换键名。
    """
    log_path = tmp_path / "mpc.jsonl"
    with _scenario(hold_offset_x=10.0, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 15, timeout_s=10.0,
                    message="解锁后没攒到 15 帧 setpoint")

    normal = [r for r in _读日志(log_path) if r["fallback"] == "normal"]
    assert len(normal) >= 10, f"正常帧只有 {len(normal)} 拍，测不出「爬不爬」"

    z0 = Z0Params()
    want = [z0.t_hover, 0.0, 0.0, 0.0]
    for r in normal:
        # ★ 三列一起断言：`df` 是积分器的**状态**、`u_ref0` 是代价参考、发出去的推力是**落点**。
        assert r["df"] == 0.0, \
            f"第 {r['tick']} 拍的垂直偏置不是 0：df = {r['df']}（e_z = x_ref0[2]−x0[2] = " \
            f"{r['x_ref0'][2] - r['x0'][2]:.3e}，这一场它必须恒为 0）"
        assert r["u_ref0"] == pytest.approx(want), \
            f"第 {r['tick']} 拍的控制参考变成了 {r['u_ref0']}（角速率三路该恒为 0、推力该是 {want}）"
        # ☠️ 这一场 `df ≡ 0` ⟹"上一拍的 ΔF"与"同一拍的 ΔF"都是 0，差一拍那件事在这里**无感**
        #   （差一拍的实测与正确口径见 `test_垂直偏置加在发出去的指令上_不在代价参考上`）。
        assert _发出去的推力(r, z0) == pytest.approx(_该发出去的推力(r, z0, r["df"]), abs=1e-4), \
            (f"第 {r['tick']} 拍发出去的推力不是 `clip(u[0] + ΔF)`：实测 {_发出去的推力(r, z0):.5f} N、"
             f"该是 {_该发出去的推力(r, z0):.5f} N ⟹ 落点搬错了（或加了两遍）")
    assert max(abs(r["u"][2]) for r in normal) > 1.0, \
        ("同段的解没动过 ⟹ 这条测试证明不了「前馈没跟着解爬」（任务参数失效？）"
         "（动捕档看 `u[2]`/pitch —— 见本用例的 ☠️，⛔ 不是 `u[1]`/roll）")


def test_垂直偏置积的是实测位置误差(tmp_path):
    """★ ticket 20 的**正控**：`ΔF` 必须等于"**实测垂直位置误差**的纯积分"，一阶不差。

    与负控是同一条 x 任务（`hold_offset_x = 10` ⟹ `u[2]` 照样顶满、解照样在动），
    **只多一个 z 偏移** `hold_offset_z = 2.0` ⟹ `e_z ≡ +2.0` 永远关不掉（锚点从测量起，
    见 `node_core.hold_reference` 的 ★★）⟹ `ΔF` 该以 `ki_z·e_z` 的斜率线性长上去。

    | # | 判据 | 为什么它有牙齿 |
    |---|---|---|
    | ① | `e_z` 逐帧恒 = `hold_offset_z`（读日志的 `x_ref0[2] − x0[2]`） | 先把"输入是常量"钉住，②的线性才有意义 |
    | ② | `ΔF` 逐帧**严格单调增** | 正号累积（偏低 ⟹ 加推力） |
    | ③ | `ΔF_last − ΔF_first ≈ ki_z·e_z·(t_last − t_first)`（`t` 取日志的 `wall_ns`） | **望远镜求和**：右边 = `ki_z·∫e_z dt` 的离散形式，左边只由两端读数决定 ⟹ 与逐拍抖动无关 |
    | ④ | 全程 `ΔF` 没顶到界（`f_max − t_hover`） | ③ 在饱和段会失真 ⟹ 先排除 |
    | ⑤ | `u_ref0[1:]` 逐帧恒 0 且同段 `u[2]` > 1.0 | runaway 守卫（角速率三路**永远**是常量 0） |
    | ⑥ | ⚠️ **本夹具量不到**：只验"没有中途复位"这个前提（见下 ☠️） | 「钉锚点 ⟹ 复位」在本层**没有端到端读数**（起始那一次之前无历史；此后造不出第二个锚点） |

    ★ 为什么 ③ 能证伪"偏置来自上一帧解"：②③ 量到的是一条**跟着墙钟线性走**的斜坡，
      斜率**正好**是 `ki_z·e_z` —— 那是实测误差的函数。解那一路既不是线性的、也不带 `e_z`
      （同段 `u[2]` 就顶在 3.84 rad/s 上不动）⟹ 任何"把 `sol.u` 接进积分器"的写法都过不了 ③。
    ⚠️ ③ 是**差分**形式 ⟹ 对 `ΔF` 上的**恒定**偏置免疫（比如 OCP 自己的稳态偏置被算进去）。
      原打算由 ⑥（锚点处必须为 0）挡这一头，2026-09-29 实测发现 ⑥ **在本夹具上量不到**
      （☠️ 见下面那段），于是它现在只守"没有中途复位"这个前提 ⟹ **这条恒定偏置的缺口
      在本层是敞着的**，判据① 的独立读数仍以飞行留档为准（`df` 列的稳态值）。
    """
    log_path = tmp_path / "mpc.jsonl"
    # ★ `ki_z` 在这里**显式设成 1.0**、并拿同一个数去算预测值：既让③不依赖"节点当前的默认增益"
    #   （那是产品参数，会变），也不用 `import NodeParams`（seam 1 的纪律：不 import 节点的类）。
    ki = 1.0
    with _scenario(hold_offset_x=10.0, hold_offset_z=2.0, ki_z=ki,
                   log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 150, timeout_s=15.0,
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
    assert max(abs(r["u"][2]) for r in normal) > 1.0, \
        "同段的解没动过 ⟹ 正控不成立（任务参数失效？）"

    # ★★ 落点判据里**能在这一场判**的那半：`u_ref0` 必须仍是常量（代价参考里没有 ΔF）。
    #   ☠️ 实测（2026-09-29，"只加一半 ΔF"的变异把本条以外的断言全放过之后回查）：
    #   **这一场 `u[0]` 逐帧恒 = `f_max` = 11.8 N** —— `hold_offset_z = 2.0` 太大，求解器自己
    #   就把推力顶满了 ⟹ `clip(u[0] + ΔF)` 与 `clip(u[0] + 0.5ΔF)` **都是 11.8**。
    #   所以"发出去的 == `clip(u[0] + ΔF)`"这条等式**在这里是恒等式、量不出落点**
    #   （本张第一版就把它写在这儿，是**假绿**）。落点等式改在专门的用例里量，工况取
    #   **不饱和**的小误差：`test_垂直偏置加在发出去的指令上_不在代价参考上`。
    #   ⚠️ 取反要写 `not (… == approx)`，别写 `… != approx`（后者的取反语义不在 pytest 的契约里）。
    _u_ref_want = [z0.t_hover, 0.0, 0.0, 0.0]
    _u_ref_bad = [(r["tick"], r["u_ref0"]) for r in normal
                  if not r["u_ref0"] == pytest.approx(_u_ref_want)]
    assert not _u_ref_bad, \
        (f"代价参考 `u_ref0` 不是常量了（四路该恒为 `[t_hover,0,0,0]`）：{_u_ref_bad[:3]}\n"
         f"    若推力那一路随 `df` 在动 ⟹ 落点又被搬回 `u_ref` 了（那是一条正反馈，见 §10）")
    assert all(0.0 <= _发出去的推力(r, z0) <= z0.f_max + 1e-6 for r in normal), \
        f"发出去的推力出界（执行器界 [0, {z0.f_max}]）"

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
    #    ② 本夹具**结构上造不出第二个锚点**，而且换载体之后这一条**更强**了：
    #       ☠️ 旧版靠"`fake_px4` 把 `xy_reset_counter`/`quat_reset_counter` 写死 0、没有注入口"
    #       —— 那是 **A 档（px4 源）** 的重钉路子，随 A 档被拒（§二⑤）整条消失。
    #       现在这一场是**动捕档**，而 `node_core._state_reset_counters` 对非 px4 源
    #       **直接 `return None, None`**（"动捕那一档没有这种计数器"）⟹ `re_xy`/`re_quat`
    #       恒为假 ⟹ 第二个锚点**在这一层根本不存在**，不是"夹具没造出来"。
    #       而起始那次之前积分器还没有历史，"复位"与"没复位"读数相同 ⟹ **端到端不可观测**。
    #       ⚠️ 这条缺口因此从"夹具能力不足"升级成"**本层原理上量不到**"—— 要补 ⑥，
    #       得先在动捕档给锚点找一个"帧真的换了"的信号（另开票），别指望夹具补个注入口。
    #    ⟹ 只能验"没有中途复位"这个**前提**本身（真出中途复位时这条会红，提示把 ⑥ 加回来）。
    anchors = [r for r in _读日志(log_path) if r["ref_anchor"]]
    assert anchors, "日志里连起始锚点都没有 ⟹ 夹具或节点有问题（不是 ⑥ 的结论）"
    assert all(r["fallback"] != "normal" for r in anchors), \
        (f"锚点拍混进了正常帧 ⟹ 中途发生了复位（锚点判定/重置计数器变了？）："
         f"{[(r['tick'], r['fallback'], r['df']) for r in anchors if r['fallback'] == 'normal'][:3]}\n"
         f"    那就必须补上⑥「锚点拍 df == 0」（现在因为量不到而撤掉了，见上面的 ☠️）")


def test_垂直偏置加在发出去的指令上_不在代价参考上(tmp_path):
    """★★ **落点判据**（ticket 20，2026-09-29 裁定 1 被证伪之后换的新落点）。

    守的是这条不变量：`ΔF` 加在**发出去的指令**上（`u_cmd[0] = clip(u[0] + ΔF, 0, f_max)`，
    `mpc_node._normal_frame`），**不**在 OCP 的代价参考 `u_ref[:,0]` 上。
    为什么后者必须被挡住：`u_ref` 是**代价参考**而不是指令，实测 `d u[0]/d u_ref[0] = −0.0616 N/N`
    —— **符号是反的** ⟹ 把位置误差积进 `u_ref` 是一条**正反馈**（越沉 ⟹ ΔF 越大 ⟹ 推力越小），
    可行域是**空集**（`mass+30` 飞成 −0.7200 m，比不加还差）。量法见
    `tools/px4-sitl/probe_u_ref_authority.py`、完整读数见 ticket 20 §10/§10.7。

    ☠️ **为什么单独一条用例、而且工况必须"小误差"**：落点等式
    `thrust·t_max == clip(u[0] + ΔF, 0, f_max)` 只有在**求解器那份 `u[0]` 没顶到 `f_max`**
    时才有鉴别力。`hold_offset_z = 2.0`（正控用的那个）会让 `u[0]` **逐帧恒 = 11.8 N**（实测），
    于是等式两边都成常量、**什么错法都拦不住** —— 本张第一版就是这么写的（假绿），
    2026-09-29 用"只加一半 ΔF"的变异实测才发现 ⟹ 拆成这条：`hold_offset_z = 0.15`
    ⟹ `u[0] ≈ t_hover + 环路刚度×0.15 ≈ 7.9 N`，**远离** `f_max`。

    | # | 判据 | 为什么它有牙齿 |
    |---|---|---|
    | ① | `u_ref0` 四路恒为 `[t_hover, 0, 0, 0]` | 落点被搬回 `u_ref` 时，推力那一路会随 `ΔF` 走（变异实测：`[7.469102, …]` → 当场红） |
    | ② | 未顶夹的帧上 `发出去 == clip(u[0] + ΔF)`（容差 `1e-4`） | 漏加 / 加在 `u_ref` 上 / **加了一半** / 加两遍，全都过不了 |
    | ③ | 该场 `df` 长到 `> 1 N`、且**未顶夹的帧 ≥ 30 拍** | **鉴别力的自检**：夹具或增益一失效，①② 就退化成恒等式（本张栽过一次，所以这条必须有） |
    | ④ | 全程发出去的推力 ∈ `[0, f_max]` | 夹取那一步还在（执行器界） |

    ★ 增益取 `ki_z = 20`：小误差 ⟹ `ΔF` 涨得慢（`ki·e_z = 3 N/s`），20 才够在 1 s 内到 `> 1 N`。
      这条用例量的是**落点**，与增益怎么标定无关（§8.3 的起测点仍是 1）。
    """
    log_path = tmp_path / "mpc.jsonl"
    ki, e_z = 20.0, 0.15
    with _scenario(hold_offset_z=e_z, ki_z=ki, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 100, timeout_s=15.0,
                    message="解锁后没攒到 100 帧 setpoint（1 s，够 ΔF 长到 1 N 以上）")

    z0 = Z0Params()
    normal = [r for r in _读日志(log_path) if r["fallback"] == "normal"]
    assert len(normal) >= 60, f"正常帧只有 {len(normal)} 拍"

    # ① 代价参考里**没有** ΔF：四路恒为常量
    _u_ref_want = [z0.t_hover, 0.0, 0.0, 0.0]
    _u_ref_bad = [(r["tick"], r["u_ref0"]) for r in normal
                  if not r["u_ref0"] == pytest.approx(_u_ref_want)]
    assert not _u_ref_bad, \
        (f"代价参考 `u_ref0` 不是常量了（四路该恒为 `[t_hover,0,0,0]`）：{_u_ref_bad[:3]}\n"
         f"    推力那一路随 `df` 在动 ⟹ 落点又被搬回 `u_ref` 了：那是一条**正反馈**"
         f"（实测 `d u[0]/d u_ref[0] = −0.0616 N/N`），别搬 —— 见 ticket 20 §10")

    # ② 落点等式：逐个"（上一拍, 这一帧）"配对来判。
    #   ☠️ **`ΔF` 取上一拍日志行的 `df`**：这一帧发出去时用的是**上一拍积完**的值
    #   （`_normal_frame` 排在 `_integrate_bias_z` 之前，见 `_tick` 的 ★★）。取同一拍会**系统性**
    #   差一个 `ki_z·e_z·Δt`——本场实测 **0.0300 N**，2026-09-29 就是这么揪出那个差一拍的。
    #   ★ 配对取**相邻日志行**（不限于正常帧）：兜底/闸门关着的那几拍 `ΔF` 冻结不积，
    #     所以"上一行的 `df`"在各段里都等于"这一拍真正用掉的那个值"。
    _rows = _读日志(log_path)
    _idx = {r["tick"]: i for i, r in enumerate(_rows)}
    _pairs = [(_rows[_idx[r["tick"]] - 1], r) for r in normal if _idx[r["tick"]] > 0]

    # ③ **先自检鉴别力**，再判等式：否则下面那条可能只是在比两个常量
    df = [r["df"] for r in normal]
    assert df[-1] > 1.0, \
        (f"这一场 ΔF 只长到 {df[-1]:.4f} N（该以 `ki·e_z = {ki * e_z} N/s` 长）"
         f"⟹ 落点等式没有鉴别力（夹具/增益失效？）")
    _unclipped = [(p, r) for p, r in _pairs if r["u"][0] + p["df"] < z0.f_max - 1e-6]
    assert len(_unclipped) >= 30, \
        (f"未顶夹的帧只有 {len(_unclipped)} 拍（该场 `u[0]` 应远离 `f_max`："
         f"最大 `u[0]` = {max(r['u'][0] for r in normal):.5f}、`ΔF` 最大 {max(df):.5f}）\n"
         f"    ⟹ 顶夹之后等式两边都是 `f_max`，**落点错成什么样它都绿** ⟹ 工况选错了（本张栽过）")

    for p, r in _unclipped:
        f_out = _发出去的推力(r, z0)
        want = _该发出去的推力(r, z0, p["df"])
        assert f_out == pytest.approx(want, abs=1e-4), \
            (f"第 {r['tick']} 拍发出去的推力不是 `clip(u[0] + 上一拍的ΔF)`：实测 {f_out:.5f} N、"
             f"该是 {want:.5f} N（`u[0]` = {r['u'][0]:.5f}、上一拍 `df` = {p['df']:.5f}、"
             f"同一拍 `df` = {r['df']:.5f}）"
             f"⟹ 落点错了（漏加 / 加在 `u_ref` 上 / 加了一半 / 加了两遍）")
    # ④ 夹取还在（执行器界）—— 读的同样是"写进那一帧的数"
    assert all(0.0 <= _发出去的推力(r, z0) <= z0.f_max + 1e-6 for r in normal), \
        (f"发出去的推力出界：{[round(_发出去的推力(r, z0), 5) for r in normal if not 0.0 <= _发出去的推力(r, z0) <= z0.f_max + 1e-6][:3]}"
         f"（界 [0, {z0.f_max}]）")


def test_垂直偏置顶在界上就不再累积(tmp_path):
    """★ **抗积分饱和**：夹的是**状态** ⟹ 饱和段里 `ΔF` 必须**逐帧恒等**。

    手法：`ki_z := 200`、`hold_offset_z = 5` ⟹ 几拍就顶到上界 `f_max − t_hover = 4.4425 N`。

    判据（读日志的 `df` 列与**发出去的那一列**）：
      · 每一帧 `ΔF ≤ f_max − t_hover`（**不出界**）；
      · 达到上界之后 `ΔF` **逐帧 `==`**（不是"还在长、只是输出被夹"）——
        这正是"夹状态"与"只夹输出"唯一能从外部区分的地方：只夹输出的话这一列会继续单调长；
      · 发出去的推力 `thrust·t_max`（☠️ 旧载体上写作 `−thrust_body_z·t_max`）逐帧 ∈ `[0, f_max]`
        （状态在界内 + 发布点那次夹取）。
        ⚠️ 读数**不能**用 `u_ref0[0]`：ticket 20 的落点搬到指令上之后它恒为 `t_hover`
        （☠️ 本张第一版这里写的就是 `u_ref0[0]`，落点一搬它就变成一句废话 —— 断言不报错、
        只是不再测任何东西，这正是最难发现的那种退化）。

    ★ 上界口径：`f_max = 11.8 N`（电池可持续推力，`params.py:108`），**不是** `t_max`
      （那是归一化常数 `t_hover / hover_thrust`，默认口径下 46.3 N）—— 见 ticket 20 §8.1 的 ☠️。
    ★ `ki_z = 200` 只是为了"几拍就饱和"（真参数不会这么大，§8.3 的起测点是 1）；
      这条用例量的是**饱和之后的行为**，与增益怎么标定无关。
    """
    log_path = tmp_path / "mpc.jsonl"
    with _scenario(hold_offset_z=5.0, ki_z=200.0, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 40, timeout_s=15.0,
                    message="解锁后没攒到 40 帧 setpoint")

    z0 = Z0Params()
    hi, lo = z0.f_max - z0.t_hover, -z0.t_hover
    normal = [r for r in _读日志(log_path) if r["fallback"] == "normal"]
    assert len(normal) >= 30, f"正常帧只有 {len(normal)} 拍"
    assert all(lo <= r["df"] <= hi for r in normal), \
        f"ΔF 出界了（界 [{lo:.4f}, {hi:.4f}]）：{[r['df'] for r in normal if not lo <= r['df'] <= hi][:3]}"
    # ★ 输出侧：夹取在 `_normal_frame` 那一次 `clip(u[0] + ΔF, 0, f_max)` 上（`u_ref0` 是常量，
    #   拿它当"发出去多少"会变成一句废话 —— 见 docstring 的 ⚠️）。
    assert all(0.0 <= _发出去的推力(r, z0) <= z0.f_max + 1e-6 for r in normal), \
        (f"发出去的推力出界了：{[round(_发出去的推力(r, z0), 6) for r in normal if not 0.0 <= _发出去的推力(r, z0) <= z0.f_max + 1e-6][:3]}"
         f"（界 [0, {z0.f_max}]；这一场 ΔF 顶在上界 ⟹ 夹取必须真的在起作用）")
    # 这一场故意积到顶 ⟹ 夹取一定被吃到：`u[0]` 自己也会顶到 `f_max`，两者相加必然超 ⟹ 未夹的话这里必红
    assert max(_发出去的推力(r, z0) for r in normal) == pytest.approx(z0.f_max, abs=1e-3), \
        (f"发出去的推力从没到过 f_max（最大 {max(_发出去的推力(r, z0) for r in normal):.5f}）"
         f"⟹ `clip(…, 0, f_max)` 没在起作用（或这一场 `ΔF` 没顶到界）")

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
    **发出去的推力逐帧 `== u[0]`**（求解器输出原样，不加任何偏置）⟹ 行为与 ticket 20 之前**逐位相同**。
    ⚠️ 这里**不能**拿 `u_ref0` 当"没加偏置"的判据：落点搬到指令上之后它恒为 `[t_hover,0,0,0]`，
      与 `ki_z` 取什么值无关 ⟹ 用它断言等于什么都没测（本张第一版就是这么写的，本轮改掉）。
    """
    log_path = tmp_path / "mpc.jsonl"
    with _scenario(hold_offset_z=2.0, ki_z=0.0, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 40, timeout_s=15.0,
                    message="解锁后没攒到 40 帧 setpoint")

    z0 = Z0Params()
    normal = [r for r in _读日志(log_path) if r["fallback"] == "normal"]
    assert len(normal) >= 30, f"正常帧只有 {len(normal)} 拍"
    assert all(r["df"] == 0.0 for r in normal), \
        f"关掉之后 ΔF 不是 0：{[r['df'] for r in normal if r['df'] != 0.0][:3]}"
    assert all(r["u_ref0"] == pytest.approx([z0.t_hover, 0.0, 0.0, 0.0]) for r in normal), \
        "关掉之后控制参考不再是 `[t_hover,0,0,0]` ⟹ 「关掉」没有回到旧行为"
    # ★ **这条才是"关掉"的输出侧判据**（`u_ref0` 那一列现在恒为常量，见 docstring 的 ⚠️）：
    #   偏置为 0 ⟹ 发出去的推力必须**逐位等于**求解器输出（`u[0]` 本身在 `[0, f_max]` 内 ⟹ 夹取不动它）
    _biased = [(r["tick"], round(_发出去的推力(r, z0), 5), r["u"][0]) for r in normal
               if abs(_发出去的推力(r, z0) - r["u"][0]) > 1e-4]
    assert not _biased, \
        f"关掉积分增益之后发出去的推力仍不等于 `u[0]` ⟹ 还有偏置在加：(tick, 发出去, u[0]) = {_biased[:3]}"
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

    造法：`hold_offset_z = 2.0` ⟹ `ΔF` 以 `ki·e_z` 稳定长；先飞 1 s，然后**掐掉
    `/mavros/state` 4 s**（闸门源，§二④ 之后它是**唯一**那条 —— 心跳那条流整个删掉了）。

    | 读数 | 判据 | 为什么 |
    |---|---|---|
    | 日志里有一段 `wall_ns` 的**空洞** | 空洞 ≥ 0.8 s | 闸门**真的**关了（否则整条用例白跑） |
    | 空洞两端 `ΔF` 之差 | `0 ≤ Δ < ¼ · ki·e_z·空洞长度` | 下界挡住"清零"；上界挡住两种错法 —— 若**不推时钟**，恢复那一拍会积进 `ki·e_z·空洞长度`；若**静默期间继续积**，空洞里没有日志行、但恢复后 `ΔF` 会跳同一个量级。两者都 ≥ 4× 于界 |
    | 闸门关着的那段（话题面） | 零帧 setpoint | 与 `test_状态静默…` 同一条纪律 |

    ☠️☠️ **2026-10-13（ticket 13 §四③）这条改了两处，第二处不改就会静默失效**：
      · **静默对象**：旧版掐的是 `/fmu/*` 的 `vehicle_local_position`、闸门 0.5 s 后关
        （`px4_timeout_s`）。换成 `silence_for(sc.fake, "state", …)`（`/mavros/state`）。
      · ★★ **增益必须跟着降**：`/mavros/state` 是 **1 Hz + `status_timeout_s = 2.5`**
        （§二③ 从 0.75 提上来的）⟹ 闸门**最晚**在 `t0+2.7` 才关，比旧版晚了 ~2 s。
        若 `ki` 仍是 1.0，这 2 s 足够把 `ΔF` 顶到上界 `f_max − t_hover`（实测会顶满）
        ⟹ **两端都被夹在同一个常量上**，上界那条判据对两种错法**同时失效**（都绿）。
        这正是票面说的"绿着但不测任何东西"那类退化 ⟹ 取 **`ki_z = 0.3`**（0.6 N/s），
        闸门关时 `ΔF ≈ 0.7–1.1 N`，离上界（4.44 N）还差 4 倍。
        ⚠️ 换这个数之后，上界的量级也要跟着核：`¼·0.6 N/s×2.6 s ≈ 0.39 N`，而
        "静默期间照积"会给出 `0.6×2.6 ≈ 1.56 N` ⟹ 仍有 4× 分离。

    ⚠️ 两把时钟**不能混**：日志的 `wall_ns` 是 DUT 的墙钟，`sc.t()` 是夹具自己的时间线
      ⟹ 空洞在**日志里**找（不拿 `sc.t()` 去切日志），夹具时钟只用来查话题。
    """
    log_path = tmp_path / "mpc.jsonl"
    ki, e_z = 0.3, 2.0
    with _scenario(hold_offset_z=e_z, ki_z=ki, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 100, timeout_s=15.0,
                    message="解锁后没攒到 100 帧 setpoint（1 s，先让 ΔF 长起来）")

        t0 = sc.t()
        sc.timeline.extend(silence_for(sc.fake, "state", t0 + 0.2, duration_s=4.0))
        # 闸门在**最后一帧 state 之后 2.5 s** 关，而那帧最晚也只到 `t0+0.2`
        # ⟹ `[t0+3.0, t0+4.2]` 整段都在关着（静默到 `t0+4.2` 才解）。
        sc.advance_to(t0 + 4.5)
        n_sp = len(_sp_between(sc, t0 + 3.0, t0 + 4.2))
        assert n_sp == 0, f"闸门该关着，setpoint 却还在发（{n_sp} 帧）"
        # 解静默之后还要等下一拍 1 Hz 的 state（≤1 s）把闸门重新打开，再走一段重入梯
        sc.advance_to(t0 + 8.0)

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
    with _scenario(hold_offset_z=2.0, ki_z=1.0,
                   inject_solve_fail_after_frames=20, log_file=str(log_path)) as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 40, timeout_s=15.0,
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



# ─────────────────── 轨迹参考 `/traj/reference`（ticket 17）───────────────────

#: 假 `traj` 上电时**不动**（`FakeTraj.running` 只有收到 `/traj/start` 才为真，见它的 ★★ 第 1 条）
#: ⟹ 本组的用例必须自己按一下"开始"。
#: ☠️ **2026-10-16（ticket 14 二定）这里原本写着「Phase 2 里没有那个按键的合法主人 …
#:   `mpc_node` 不接管轨迹几何」** —— 那句话现在**只对一半**：`mpc_node` 仍然不碰几何，
#:   但它**多了一个可选的主人身份**（参数 `start_traj_reference`，默认关）：
#:   模型预测控制档下 `control` 不在场，再没人发这一帧的话，`traj` 就永远在"等开始"、
#:   而我们永远在"等参考"—— 两边都正常，飞机不动（见
#:   `test_自发起步_开了之后由节点自己按开始键_没人按就一直卡着`）。
#:   ⛔ 但**这一组**（下面那几条）仍然由测试自己按：它们要验的是"参考到了之后怎么样"，
#:   而那一条闸门（`no_traj_reference`）必须**走得到** —— 让 DUT 替我们按开始键的话，
#:   它当场就跨过去了。所以"谁按开始键"这件事在**这一组**里照旧是测试自己（= 操作员）。
_TOPIC_TRAJ_START = "/traj/start"

# ⚠️ **本组对"别的会话"比本文件其它用例更敏感**（第一手，2026-09-30）：本机的台架都跑在
#    domain 73 上，而 `/traj/*` 是**全固定**的话题名 ⟹ 另一个会话正在跑 `control` / `traj`
#    的测试时，它那份 `FakeTraj` 会**往同一对话题上发**：一份参考、以及它会发 `/traj/start`。
#    症状（2026-09-30 实测到一次）：`test_没有参考时闸门不开` 里那句
#    `traj.reference_frames == 0` 变成 205 —— 假 traj 被人从外面点着了。
#    ★ 方向是好的：这类污染只会让断言**变红**（多一份参考 ⟹ 目标点不对 / `ref_frozen` 不为真 /
#      闸门不该开却开了），**不会**让它假绿。所以判据是"重跑一次"而不是"改断言"。
_污染提示 = "（先看 `pgrep -af pytest`：本机 domain 73 上可能还有一个会话在跑台架）"


def _按下开始键(sc: Scenario, traj) -> None:
    """给假 `traj` 发一帧 `/traj/start`，并等到它**真的开始发参考**。

    ★ 两段等待都不是客套：① 发现是异步的，`get_subscription_count()` 到 1 之前发的那一帧
      会**丢**（症状是"按了没反应"）；② `FakeTraj` 是 50 Hz 定时器驱动的，`reference_frames`
      涨了才算参考真的上了线（否则后面的判据会量到"参考没来"）。
    """
    from std_msgs.msg import Empty

    pub = sc.node.create_publisher(Empty, _TOPIC_TRAJ_START, 10)
    sc.wait_for(lambda: pub.get_subscription_count() >= 1, timeout_s=10.0,
                message=f"假 traj 没订上 {_TOPIC_TRAJ_START}（发现还没跑完就发 ⟹ 那一帧丢了）")
    pub.publish(Empty())
    sc.wait_for(lambda: traj.reference_frames > 0, timeout_s=10.0,
                message="按了开始键，假 traj 一帧参考都没发（`selected` 为假？定时器没转？）")


def test_掩码常量与_mavros_msgs_逐位相同():
    """★ 本文件是**唯一**能验这条的地方：`node_core` 按契约不 import ROS ⟹ 它的 `IGNORE_*`
    是**手抄**的常量（见 `traj_reference_fields` 的 ★★）。

    抄错一位的症状是"某一轴参考恒为 0"或"某一轴照着一个被忽略的字段走" —— 两者在话题面上
    都像正常飞行。所以这里逐位对一遍真消息类的常量，并顺手把"默认掩码 `2496` 下
    位置/速度/yaw 全都有效"这件事**在真类上**再验一次（`test_node_core.py` 只能验到内部自洽）。

    ⚠️ 这一条**不起 DUT**（纯常量），所以它是廉价的那一档 —— 但它守的是**协议**，
      而协议的错法是这一票里最安静的一种。
    """
    from mavros_msgs.msg import PositionTarget

    from nmpc import node_core as nc

    for name in ("IGNORE_PX", "IGNORE_PY", "IGNORE_PZ",
                 "IGNORE_VX", "IGNORE_VY", "IGNORE_VZ", "IGNORE_YAW"):
        assert getattr(nc, name) == getattr(PositionTarget, name), \
            f"{name} 与 mavros_msgs 对不上（手抄的常量漂了）"
    # Phase 1 的 `kTypeMask`（`traj_node.cpp` 的默认掩码）：只忽略加速度三位与 yaw_rate。
    default_mask = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY
                    | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)
    assert default_mask == 2496, "默认掩码变了 ⟹ 位置/速度/yaw 的有效性断言要重写"
    assert default_mask & (nc.IGNORE_PX | nc.IGNORE_PY | nc.IGNORE_PZ
                           | nc.IGNORE_VX | nc.IGNORE_VY | nc.IGNORE_VZ
                           | nc.IGNORE_YAW) == 0, \
        "默认掩码下这三样该全有效；有交集说明掩码或常量抄错了"


def test_下行那条路的两个常量也对着真消息类():
    """★★ **换载体之后新出现的两个"手抄常量"，各配一条对真类的断言**（ticket 13 §二①③）。

    与上面那条同一纪律（**权威在 msg**），但守的是**另一条消息**：

    | 候选写法 | 权威 | 抄错的症状 |
    |---|---|---|
    | `mpc_node.TYPE_MASK_IGNORE_ATTITUDE = 128` | `AttitudeTarget.IGNORE_ATTITUDE` | 掩码写错一位：`orientation` 未被忽略（**填的是默认全零四元数**，飞控可能按它转姿态）或 `IGNORE_THRUST` 被置上（**掉高度**，而话题面上每帧都正常） |
    | `node_core.MODE_OFFBOARD = "OFFBOARD"` | `State.MODE_PX4_OFFBOARD` | 字符串写错（`"OFFBOARD "` / 大小写）：闸门**永远判"不在 OFFBOARD"**、一帧不发，而日志里那句 `mode='OFFBOARD'` 看着完全正确 |

    ★ 为什么必须**对真消息类**验而不是两边都自己写：那两个常量在 `node_core` 与本文件里
      都是手抄的（`node_core` 按契约不 import ROS），手抄的错误只会在**这里**露出来。
    ⚠️ 不起 DUT（纯常量）—— 它守的是协议，而协议的错法是本票里最安静的一种。
    """
    from mavros_msgs.msg import AttitudeTarget, State

    from nmpc import mpc_node, node_core

    assert mpc_node.TYPE_MASK_IGNORE_ATTITUDE == AttitudeTarget.IGNORE_ATTITUDE == 128
    # 另外三位**必须**是 0 的对应关系：我们只用角速率 + 推力 ⟹ 只忽略姿态，
    # 三个角速率位与推力位都不能置（置了就是把发出去的东西丢掉一半）。
    for bit in (AttitudeTarget.IGNORE_ROLL_RATE, AttitudeTarget.IGNORE_PITCH_RATE,
                AttitudeTarget.IGNORE_YAW_RATE, AttitudeTarget.IGNORE_THRUST):
        assert mpc_node.TYPE_MASK_IGNORE_ATTITUDE & bit == 0, f"掩码不该置 {bit}"
    assert node_core.MODE_OFFBOARD == State.MODE_PX4_OFFBOARD == "OFFBOARD"


def test_轨迹档才订_traj_reference_而默认档一帧都不订():
    """★ 条件订阅的可断言形态（与 A 档"不订动捕"那条同一纪律，ticket 09 的 ★★）。

    两个方向都要量，缺一半就是假断言：
      · `reference_source=traj` ⟹ `/traj/reference` 上**有**订阅者；
      · 默认档（`hold`）⟹ **一个都没有**（"订阅了但不看"在话题边界上不可证伪，
        而那正是"默认档不改变任何现有行为"这条承诺唯一的黑盒证据）。

    ⚠️ 断言的**顺序**是判据的一部分（第一手教训见 A 档那条）：必须等发现跑完再说"没有订阅者"。
      屏障取"假 traj 已经和 DUT 见面了"或"心跳已发出 + `hold(1.0)`"，再加正控 ——
      否则"零订阅者"与"节点压根没起来"在断言里长得一样。

    ★★ **两个方向都用"起 DUT 前后订阅者数目的差"来判**（`start_dut=False` + 手工 `start_dut()`），
      而**不是**判 `matched_subscribers() == 0`。第一手（2026-09-30）：那条更直觉的写法**会假红**
      —— `/traj/reference` 的**正常**订阅者里就有 Phase 1 的 `control_node`（它才是这条话题
      原本的消费者），而本机的台架都跑在 domain 73 上：另一个会话跑 `src/control/test` 时，
      它的 `control_node` 会让这个计数**恒 ≥ 1**。判差之后，外来的订阅者被同时计入前后两边
      而消掉，**只有我们自己这个进程**才动得了这个差。
      ⚠️ 同理，正向那条若写 `>= 1` 就是**假绿**的（外来订阅者替我们满足了它）。
    """
    with _scenario(reference_source="traj", start_dut=False) as sc:
        from bench import FakeTraj
        traj = FakeTraj(sc.node, sc)
        sc.hold(1.5)                            # 等发现跑完（此刻 DUT 还没起 ⟹ 基线不含我们）
        base = traj.matched_subscribers()
        sc.start_dut()
        _wait_dut_up(sc)
        sc.hold(1.0)
        assert traj.matched_subscribers() == base + 1, \
            f"轨迹档下 DUT 没有给 /traj/reference 加上一个订阅者（{base} → {traj.matched_subscribers()}）"
        _按下开始键(sc, traj)                                  # 没有参考的话闸门不会开
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 5, timeout_s=10.0,
                    message="正控不成立：订上参考之后并没有真的开始发 setpoint")

    with _scenario(start_dut=False) as sc:                    # 默认档 = hold
        from bench import FakeTraj
        traj = FakeTraj(sc.node, sc)
        sc.hold(1.5)
        base = traj.matched_subscribers()
        sc.start_dut()
        _wait_dut_up(sc)
        sc.hold(1.0)
        assert traj.matched_subscribers() == base, \
            ("默认（hold）档下 DUT 也订了 /traj/reference —— 那会让「默认档不改变任何现有行为」"
             "不可证伪（这条数错了不是外来的订阅者，差值已经把它消掉了）")
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 5, timeout_s=10.0,
                    message="正控不成立：默认档在解锁之后没发 setpoint（那说明「没订」是因为节点坏了）")


def test_轨迹参考驱动目标点_且参考速度进了日志(tmp_path):
    """★★ 本票的**行为**判据：`/traj/reference` 一到，目标点就从"锚点 + `hold_offset`"
    变成"**锚点 + 参考给的相对量**"，而且参考的**速度**（不是测量的速度）出现在日志里。

    ★ 鉴别力在哪儿：同样的动捕位置 `(1,2,3)`，`hold` 档的目标点是 `(1,2,3)`、速度恒 0；
      轨迹档这里是 `(1.5, 1.75, 3.0)`、速度 `(0.2,0,0)` ⟹ 三种数都**互不相同**，
      所以这条断言能把"参考真的接上了"与"参考被忽略、退回了悬停"分开
      （后者是这一票最可能的失效形态：掩码读错、坐标没接上、订阅没成功）。
    ★ `x_ref0_v` 是**参考的速度**而不是测量的：夹具的动捕速度是 0（`VrpnSource.velocity` 默认），
      所以这一格非 0 **只可能**来自参考那条消息 —— 这是它比位置更能证伪的地方。

    参考用的是**常量偏移 + 常量速度**这一族（也正是 `traj_reference` 的精确解族，见
    `hold_feedforward` 的 ☠️ 2026-09-30 订正）：`p_rel` 不随时间变，而**时域外推**由参考
    自己的速度给 ⟹ 目标点在时间轴上是一条以 `v` 前进的直线。
    """
    log = tmp_path / "traj.jsonl"
    with _scenario(reference_source="traj", log_file=str(log), log_every=1) as sc:
        from bench import FakeTraj
        sc.mocap_source.position = (1.0, 2.0, 3.0)
        traj = FakeTraj(sc.node, sc,
                        reference_fn=lambda _t: (0.5, -0.25, 0.0),
                        velocity_fn=lambda _t: (0.2, 0.0, 0.0))
        _wait_dut_up(sc)
        sc.wait_for(lambda: traj.matched_subscribers() >= 1, timeout_s=10.0,
                    message="DUT 没订上 /traj/reference")
        _按下开始键(sc, traj)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 20, timeout_s=10.0,
                    message="参考一直在发，闸门却没开 ⟹ 参考那条没进 `cycle()`")
        sc.hold(0.5)

        rows = [r for r in _读日志(log) if r.get("x_ref0")]
        assert rows, "逐帧日志一帧都没写（`log_file` 参数没生效？）"
        r = rows[-1]
        assert r["reference_source"] == "traj"
        assert r["x_ref0"] == pytest.approx([1.5, 1.75, 3.0], abs=1e-4), \
            "目标点不是「锚点 + 参考给的相对量」（退回参数化悬停了？）"
        assert r["x_ref0_v"] == pytest.approx([0.2, 0.0, 0.0], abs=1e-6), \
            "参考的速度没进日志（`velocity` 那三格被掩码吃了 / 没读？）"
        # ★ 正控的反面：测量速度是 0（假动捕不发速度）⟹ 上面那一格非 0 只可能来自参考。
        assert r["x0"][3:6] == pytest.approx([0.0, 0.0, 0.0], abs=1e-6), \
            "测量速度非 0 ⟹ 上面那条断言就不是「参考的速度」了（夹具的假设变了）"


def test_参考断流之后冻结_而不是停发(tmp_path):
    """★ Phase 1 的裁定在 Phase 2 的落点：**缺参考 ⟹ 保持上一个目标，不回退到原点**
    （`phase1-implementation.md` §3.3；`StopConfig.traj_stream_gap_s`）。

    ★ 参数名与 Phase 1 那一档**逐字相同**（`src/control/README.md:119` 的 `traj_stream_gap_s`）：
      那边 0.5 s ⟹ 保持上一个目标，`traj_timeout_s`（3.0 s）是**另一级**（放弃这一趟）。
      本节点只有第一级，见 `node_core.StopConfig.traj_stream_gap_s` 的 ⚠️。

    断言三件事，少一件这条就不成立：
      ① **setpoint 照发**（心跳那一路不算 —— 它本来就无条件发，见模块 docstring 的 ★★）；
      ② 日志里 `ref_frozen` 为真、`x_ref0_v` 归零；
      ③ 目标点**守住**（不回到原点、也不继续往前飘）。
    ☠️ ②里"速度归零"这一项是**必须**的：只冻位置不冻速度的话，参考会按自己的速度继续
      往前外推、自己飞走，而飞机照着追 —— 一个**不会报错**的漂移源。
    """
    log = tmp_path / "freeze.jsonl"
    with _scenario(reference_source="traj", traj_stream_gap_s=0.3,
                   log_file=str(log), log_every=1) as sc:
        from bench import FakeTraj
        sc.mocap_source.position = (1.0, 2.0, 3.0)
        traj = FakeTraj(sc.node, sc,
                        reference_fn=lambda _t: (0.5, -0.25, 0.0),
                        velocity_fn=lambda _t: (0.2, 0.0, 0.0))
        _wait_dut_up(sc)
        sc.wait_for(lambda: traj.matched_subscribers() >= 1, timeout_s=10.0,
                    message="DUT 没订上 /traj/reference")
        _按下开始键(sc, traj)
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 20, timeout_s=10.0,
                    message="参考一直在发，闸门却没开")

        t0 = sc.t()
        traj.silence()                       # 上游没了：不归零、不发任何东西
        sc.advance_to(t0 + 1.5)

        n_sp = len(_sp_between(sc, t0 + 0.8, t0 + 1.5))
        assert n_sp > 0, "参考断流之后 setpoint 停了 —— 该冻住，不该交给兜底阶梯"
        assert sc.dut_alive(), "DUT 在参考断流期间退出了"

        rows = [r for r in _读日志(log) if r.get("x_ref0")]
        tail = rows[-1]
        assert tail["ref_frozen"] is True, "参考断了 1.2 s 还没冻（`traj_stream_gap_s=0.3`）"
        assert tail["x_ref0_v"] == pytest.approx([0.0, 0.0, 0.0], abs=1e-6), \
            "★ 冻结必须把**速度**一起归零，否则参考自己按 v 继续外推、飞机照着追"
        assert tail["x_ref0"] == pytest.approx([1.5, 1.75, 3.0], abs=1e-4), "冻结该守住最后那一点"
        # 正控：断流之前它确实在动（否则"冻住"与"从头就没接上"分不开）。
        early = [r for r in rows if r.get("ref_frozen") is False]
        assert early and early[-1]["x_ref0_v"] == pytest.approx([0.2, 0.0, 0.0], abs=1e-6), \
            "断流之前参考速度就不是 0.2 ⟹ 这条用例没有鉴别力"


def test_没有参考时闸门不开_理由是_no_traj_reference():
    """轨迹档下**一帧参考都没到** ⟹ 闸门必须关着，且理由要指得出是"参考没来"。

    ★ 为什么看理由而不是只看"没发"：`not_armed` / `no_mocap` / `no_traj_reference`
      在"发了没有"这一层长得一模一样，而根因完全不同。这条闸门是**本票新增**的那一条，
      它进不了日志的话，现场"解锁了、offboard 了、什么都没发"就查不出是谁拦的。
    ★ **正控**（☠️ 2026-10-13，ticket 13 §四③ 换过）：旧版是"心跳照发"（
      `sc.px4.published_frames("mode") > 50`），而心跳那条流在 §二④ **整个删掉了**
      ⟹ 那条正控没有载体了。换成一条**更强**的：把参考真的送上去，看闸门**当场开**。
      它比"心跳在发"多证一件事 —— 心跳只证明"进程在跑"，拦它的完全可能是
      `not_armed` / `no_mocap`（那这条用例就根本没测到 `no_traj_reference`）；
      而"参考一到就发帧"把**停发的原因**唯一地钉在了参考上。
    """
    with _scenario(reference_source="traj") as sc:
        from bench import FakeTraj
        traj = FakeTraj(sc.node, sc)          # ★ 刻意**不按开始键**：一帧参考都不会有
        _wait_dut_up(sc)
        sc.wait_for(lambda: traj.matched_subscribers() >= 1, timeout_s=10.0,
                    message="DUT 没订上 /traj/reference")
        _arm_and_offboard(sc)
        sc.hold(1.5)

        assert traj.reference_frames == 0, "假 traj 自己发了参考 ⟹ 本用例的前提不成立"
        n_sp = len(_sp_frames(sc))
        assert n_sp == 0, f"参考一帧都没到，却发了 {n_sp} 帧 setpoint"
        assert sc.dut_alive()
        assert "no_traj_reference" in sc.dut_log(), \
            f"停发的理由不是 no_traj_reference（日志尾：{sc.dut_log()[-500:]!r}）"

        _按下开始键(sc, traj)                 # 正控：参考一到，闸门当场开
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 5, timeout_s=10.0,
                    message="参考送上去了闸门仍不开 ⟹ 上面那句'零帧'不是 no_traj_reference 拦的")


def test_自发起步_开了之后由节点自己按开始键_没人按就一直卡着():
    """★★ `start_traj_reference`（ticket 14 二定）：**模型预测控制档没有 `control` 在场**，
    所以那一个档要由 `mpc_node` 自己发 `/traj/start`；不开就维持老行为（谁都不发）。

    ★★ 为什么这条非有不可：`traj` 的"选轨迹"与"开始"是**两条**消息（见 `traj_node.cpp` 文件头），
      而没人按开始键时**两边都正常**——`traj` 说"我准备好了，等开始"、我们说"闸门关着，
      理由是 `no_traj_reference`"——**日志里找不出一条错**，飞机就是不动。
      这条用例把那个状态造出来（两个方向都造），因为它是这一档**唯一**会踩的坑。

    ★ 两个方向的鉴别力（缺一半就是一个假断言）：
      · **开**：全程**没有任何人**按开始键（测试不调 `_按下开始键`）⟹ 参考必须自己上线、
        闸门必须开、必须有下行帧。谁把这个参数删掉/写死成关，这条立刻红。
      · **关**（默认）：同样的场面、同样没人按 ⟹ 参考**一帧都不该有**、下行帧**一帧都不该有**、
        理由必须是 `no_traj_reference`。谁把它写死成"总是发"（或干脆不读这个参数），
        这条红。⟹ 与上面那条 `test_没有参考时闸门不开…` 是**同一个场面**，
        那条守的是老行为，这条守的是"开关真的分了岔"。

    ⚠️ **发出去那一帧可能丢**：DUT 只在"第一次见到 `no_traj_reference`"那一拍发**一次**
      （闩住是对的，见 `_maybe_start_traj` 的 ⛔），而 DDS 发现是异步的 —— 若那一帧落在
      假 traj 的订阅匹配好之前，它就没了，且**不会重发**。所以下面先等"假 traj 已经和
      DUT 的 `/traj/start` 发布者见上面"（`starts` 之外还有一条更早的判据：DUT 的发布者
      被本进程发现），再解锁。★ 这条等待是**判据的一部分**，不是客套：
      把它删掉，这条用例会变成一条**随机红**（同 `test_轨迹档才订…` 的 ★★）。
      而"解锁 → DUT 看见 OFFBOARD"这一段本身还要 ~1 s（`/mavros/state` 是 1 Hz）⟹
      两个窗叠起来，发现早就跑完了。
    """
    with _scenario(reference_source="traj", start_traj_reference=True) as sc:
        from bench import FakeTraj
        traj = FakeTraj(sc.node, sc)          # `selected=True`：收到 start 就会发参考
        _wait_dut_up(sc)
        sc.wait_for(lambda: traj.matched_subscribers() >= 1, timeout_s=10.0,
                    message="DUT 没订上 /traj/reference")
        # ★ 屏障：DUT 的 `/traj/start` 发布者**被本进程发现**之后才解锁（否则那一帧可能丢）
        sc.wait_for(lambda: sc.node.count_publishers(_TOPIC_TRAJ_START) >= 1, timeout_s=10.0,
                    message=f"DUT 没建 {_TOPIC_TRAJ_START} 的发布者"
                            "（`start_traj_reference=True` 时它该建 —— 条件是建了吗？）")
        assert traj.starts == [], "假 traj 还没上电就收到了 start ⟹ 本用例的前提不成立"

        _arm_and_offboard(sc)
        # 判据①：**没人按开始键**，而参考自己上线了
        sc.wait_for(lambda: traj.reference_frames > 0, timeout_s=10.0,
                    message=f"没人按开始键，假 traj 一帧参考都没发 ⟹ DUT 没有自己发"
                            f"{_TOPIC_TRAJ_START}（参数没生效？）{_污染提示}")
        assert len(traj.starts) == 1, \
            f"收到 {len(traj.starts)} 次 /traj/start（该是**恰好一次**）—— 每拍都发的话，"\
            "`traj` 会把参考时钟一直归零，飞机永远停在轨迹起点（见 `_maybe_start_traj` 的 ⛔）"
        # 判据②：参考上线之后闸门就该开（这条才是"飞得起来"）
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 5, timeout_s=10.0,
                    message="参考自己上线了，闸门却不开 ⟹ 上面那个 start 只是发了，没接上")
        assert "no_traj_reference" in sc.dut_log(), \
            "日志里没有过 no_traj_reference ⟹ 触发时刻不对（它不是被'等参考'叫醒的）"

    with _scenario(reference_source="traj") as sc:        # 老行为：默认关
        from bench import FakeTraj
        traj = FakeTraj(sc.node, sc)
        _wait_dut_up(sc)
        sc.wait_for(lambda: traj.matched_subscribers() >= 1, timeout_s=10.0,
                    message="DUT 没订上 /traj/reference")
        assert sc.node.count_publishers(_TOPIC_TRAJ_START) == 0, \
            (f"默认档（`start_traj_reference` 没打开）却建了 {_TOPIC_TRAJ_START} 的发布者 —— "
             "「默认档不改变任何现有行为」在话题面上就不可证伪了")
        _arm_and_offboard(sc)
        sc.hold(1.5)
        assert traj.starts == [] and traj.reference_frames == 0, \
            f"默认档下假 traj 自己动了起来（starts={traj.starts}）⟹ 有人替我们按了开始键"
        n_sp = len(_sp_frames(sc))
        assert n_sp == 0, f"没有参考却发了 {n_sp} 帧 setpoint"


def test_自发起步只在轨迹档有意义_别的档入口当场拒():
    """★ `start_traj_reference=true` + `reference_source=hold` ⟹ **入口当场拒**（ticket 14 二定）。

    ★ 为什么拒而不是"接受但忽略"：这个参数认的触发原因是 `no_traj_reference`，而
      `hold` 档下参考是自己算出来的、**不存在**"等参考"这个状态 ⟹ 那个旋钮在那一档
      **永远是死的**。接受它的症状是"我明明开了，参考还是不动"—— 而**参考本来就不该由它驱动**，
      两句话都成立，读日志的人会一路查到参考那一头去（那是对的，但白查）：
      与 `traj` 档拒非 0 的 `hold_offset` 是同一条纪律（**没意义的组合在入口说清楚**）。

    ★ 断言的三层同 `test_A_档_state_source_px4_入口当场拒绝`：退出码非 0、拒绝文案里要有
      特征串（`start_traj_reference` 与 `reference_source` 两个参数名）、**一帧 setpoint 都没有**
      —— 最后一层是"拒绝"与"起了但那一档不发"的行为分界。
    """
    with _scenario(reference_source="hold", start_traj_reference=True) as sc:
        sc.wait_for(lambda: not sc.dut_alive(), timeout_s=STARTUP_S,
                    message="start_traj_reference + reference_source=hold 之后 DUT 一直没退出")
        sc.hold(0.5)                 # 让读取线程把最后几行排空
        rc = sc.dut_returncode()
        log = sc.dut_log()
        assert rc not in (0, None), \
            (f"这个没意义的组合没被拒（退出码 {rc}）—— 它是不是只打了句警告就照常跑起来了？"
             f"（日志尾：{log[-800:]!r}）")
        assert "start_traj_reference" in log and "reference_source" in log, \
            f"拒绝理由没把两个参数名都说出来（日志尾：{log[-800:]!r}）"
        assert sc.fake.attitude_publishers() == 0, \
            "入口都拒绝了，却还有人往 /mavros/setpoint_raw/attitude 上发"


# ───────────────────────────── 票 15：`reference_source=control` ─────────────────────────────

_TOPIC_CONTROL_TARGET = "/control/target"


def _控制目标_msg(p=(4.0, 5.0, 6.0), v=(0.2, 0.0, 0.0), yaw=0.3):
    """一帧 `/control/target`：位置 + 速度 + yaw，掩码把加速度与 yaw_rate 忽略掉。

    ★ 掩码与 `control` 侧要发的**逐位相同**（2496）：少一位 ⟹ 那一路被当成 0 用，
      而话题面上每帧都"正常"（正是 `traj_reference` 那条 ☠️ 的同一个坑）。
    """
    from geometry_msgs.msg import Point, Vector3
    from mavros_msgs.msg import PositionTarget
    m = PositionTarget()
    m.type_mask = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY
                   | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)
    assert m.type_mask == 2496, f"掩码与 control 侧不一致：{m.type_mask}"
    m.position = Point(x=p[0], y=p[1], z=p[2])
    m.velocity = Vector3(x=v[0], y=v[1], z=v[2])
    m.yaw = yaw
    return m


def _开始发控制目标(sc: Scenario, **kw):
    """在 `/control/target` 上按 50 Hz 定时发；返回 `(pub, state)`。

    `state["on"] = False` 即断流（用来量"超时关闸"），不必销毁定时器。
    """
    from mavros_msgs.msg import PositionTarget
    msg = _控制目标_msg(**kw)
    state = {"on": True}
    pub = sc.node.create_publisher(PositionTarget, _TOPIC_CONTROL_TARGET, 10)

    def _tick():
        if state["on"]:
            pub.publish(msg)

    sc.node.create_timer(0.02, _tick)
    return pub, state


def test_控制档没有目标时闸门不开_理由是_no_control_target():
    """★ **控制档新增的那条闸门**：`/control/target` 一帧都没到 ⟹ 一帧 setpoint 都不发，
    理由必须指得出是"目标没来"，而不是 `not_armed` / `no_mocap`（三层在"没发"上长得一样）。

    ★ **正控**：把目标送上去，闸门必须**当场开** —— 只断言"没发"无法区分"闸门拦住了"
      与"节点根本没在跑"。正控同时把订阅关系也证了（话题面上 `pub` 的订阅者 ≥ 1）。
    """
    with _scenario(reference_source="control") as sc:
        _wait_dut_up(sc)
        _arm_and_offboard(sc)
        sc.hold(1.5)

        n_sp = len(_sp_frames(sc))
        assert n_sp == 0, f"一帧 /control/target 都没到，却发了 {n_sp} 帧 setpoint"
        assert sc.dut_alive()
        assert "no_control_target" in sc.dut_log(), \
            f"停发的理由不是 no_control_target（日志尾：{sc.dut_log()[-500:]!r}）"

        pub, _ = _开始发控制目标(sc)
        sc.wait_for(lambda: pub.get_subscription_count() >= 1, timeout_s=10.0,
                    message="DUT 没订上 /control/target")
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 5, timeout_s=10.0,
                    message="目标送上去了闸门仍不开 ⟹ 上面那句'零帧'不是 no_control_target 拦的")


def test_控制档目标直接用绝对量_不加锚点_速度进日志(tmp_path):
    """★★ 本票的**行为**判据：`/control/target` 是**绝对目标**，`mpc_node` **不许再加锚点**。

    ★ 鉴别力：动捕位置 `(1,2,3)`、目标 `(4,5,6)`。若误用了轨迹档的"锚点 + 相对量"，
      目标会变成 `(5,7,9)`；若参考没接上、退回悬停，则是 `(1,2,3)`。三个数互不相同
      ⟹ 这一格能把三种失效分开。
    ★ 速度用的是**参考的** `(0.2,0,0)`，而假动捕的速度恒 0 ⟹ 这一格非 0 **只可能**来自
      `/control/target`（与 `traj` 档那条同一个理由）。
    """
    log = tmp_path / "control.jsonl"
    with _scenario(reference_source="control", log_file=str(log), log_every=1) as sc:
        sc.mocap_source.position = (1.0, 2.0, 3.0)
        pub, _ = _开始发控制目标(sc)
        _wait_dut_up(sc)
        sc.wait_for(lambda: pub.get_subscription_count() >= 1, timeout_s=10.0,
                    message="DUT 没订上 /control/target")
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 20, timeout_s=10.0,
                    message="目标一直在发，闸门却没开 ⟹ 目标那条没进 `cycle()`")
        sc.hold(0.5)

        rows = [r for r in _读日志(log) if r.get("x_ref0")]
        assert rows, "逐帧日志一帧都没写（`log_file` 参数没生效？）"
        r = rows[-1]
        assert r["reference_source"] == "control"
        assert r["x_ref0"] == pytest.approx([4.0, 5.0, 6.0], abs=1e-4), \
            ("目标点不是 /control/target 的**绝对量** —— 要么被当成了「锚点 + 相对量」"
             "（动捕 (1,2,3) 会加出 (5,7,9)），要么参考没接上退回了悬停")
        assert r["x_ref0_v"] == pytest.approx([0.2, 0.0, 0.0], abs=1e-6), \
            "参考速度没进日志（`velocity` 三格被掩码吃了 / 没读？）"
        assert r["x0"][3:6] == pytest.approx([0.0, 0.0, 0.0], abs=1e-6), \
            "测量速度非 0 ⟹ 上面那条断言就不是「参考的速度」了（夹具的假设变了）"


def test_控制目标断流之后关闸_不是冻结(tmp_path):
    """★★ 与 `traj` 档**相反的**裁定（票 15）：目标断流 ⟹ **关闸**，不是冻住最后一点。

    ★ 为什么必须相反：控制档的 `control` 才是任务总管。它一旦死了/卡了，`mpc_node`
      继续拿最后一帧目标发姿态，飞机就会**一直照着那个点飞**，而没有任何一层会退出
      OFFBOARD；关闸之后 mpc 停止发 setpoint，PX4 的 OFFBOARD 失流保护（1 s）接管 ——
      这是两条链共用同一套安全兜底的前提。
    ★ 正控（断流之前帧确实在流）不能省：否则"断流后没发"与"从头就没接上"分不开。
    """
    log = tmp_path / "control_stale.jsonl"
    with _scenario(reference_source="control", control_target_timeout_s=0.3,
                   log_file=str(log), log_every=1) as sc:
        pub, state = _开始发控制目标(sc)
        _wait_dut_up(sc)
        sc.wait_for(lambda: pub.get_subscription_count() >= 1, timeout_s=10.0,
                    message="DUT 没订上 /control/target")
        _arm_and_offboard(sc)
        sc.wait_for(lambda: len(_sp_frames(sc)) >= 20, timeout_s=10.0,
                    message="目标一直在发，闸门却没开")

        t0 = sc.t()
        state["on"] = False                  # 上游没了：不归零、不发任何东西
        sc.advance_to(t0 + 1.5)

        n_sp = len(_sp_between(sc, t0 + 0.8, t0 + 1.5))
        assert n_sp == 0, \
            f"目标断流 0.8 s 之后还发了 {n_sp} 帧 —— 该关闸交给 PX4 的失流保护，不该冻住"
        # ★★ 这一条让 `control_target_timeout_s:=0.3` **真的可鉴别**：默认值是 0.5 s，
        #   只在 0.8 s 之后看，默认值也照样"通过"（参数没读进 NodeParams 时是假绿）。
        #   0.45–0.70 s 这个窗口：配了 0.3 ⟹ 早就该关；没读进参数（默认 0.5）⟹ 还会发几帧。
        n_sp_early = len(_sp_between(sc, t0 + 0.45, t0 + 0.70))
        assert n_sp_early == 0, (
            f"断流 0.45–0.70 s 之间还在发 {n_sp_early} 帧 —— `control_target_timeout_s` "
            "没有生效（默认 0.5 s？）")
        assert sc.dut_alive()
        assert "no_control_target" in sc.dut_log(), \
            f"关闸的理由不是 no_control_target（日志尾：{sc.dut_log()[-500:]!r}）"

        rows = [r for r in _读日志(log) if r.get("x_ref0")]
        assert [r for r in rows if r.get("ref_age_s", 0.0) < 0.3], \
            "断流之前目标就没被读进来过 ⟹ 这条用例没有鉴别力"

        # 正控的反面：重新发目标，闸门必须自己重开（不是"关一次就锁死"）
        state["on"] = True
        sc.advance_to(t0 + 2.5)
        assert len(_sp_between(sc, t0 + 1.8, t0 + 2.5)) > 0, \
            "目标恢复之后闸门没有自己重开（`control` 重启会撞上这个状态）"


# ───────────────────────────── 在这一层量不了的那几条（不写假测试）─────────────────────────────

def test_发现信息里的_depth_读不回来_所以票面那条只能靠读常量():
    """★ **票面验收项「pub … `depth 1`」在本套栈上黑盒不可观测** —— 这条用例钉住这个事实，
    而不是伪造一个恒红的断言。

    第一手（本机实测 2026-09-28，Humble + rmw_fastrtps，含对照）：DDS 发现信息里的
    `history` 是 `UNKNOWN`、`depth` **恒为 0**，且与发布者实际写的值**无关**
    （`depth=1` 与 `depth=10` 读回来一模一样）。重放也不可用——晚加入的
    `TRANSIENT_LOCAL` 订阅者**匹配得上**（发布者视角 `get_subscription_count() == 1`）、
    也收得到**新**帧，但**收不到任何旧帧** ⟹ 也拿不到"depth 语义"的间接证据。

    ⟹ 本节点这一条的守法是：① 发布方 QoS 常量（`QOS_PUB_ATT`）在**单测里**逐字段对
      （`depth` 在那里是能读的，对象就在进程内）；② 黑盒这一层只断言得到的两项
      （`test_接口与_QoS_与票面一致`）。夹具侧的同一事实见
      `test_fake_px4_selftest.py::test_发现信息里读不回来的那两项_history_与_depth`。

    ☠️☠️ **2026-10-13（ticket 13 §四③）这条换过对象，两处都得改，别只改名字**：
      · 常量从 `QOS_PUB_FMU` 换成 **`QOS_PUB_ATT`**（新载体那条下行）。
      · ★ **三个字段的值全变了**，因为它们对齐的是**另一个收端**：
        · `depth`：`1` → **`5`**（`mavros::SensorDataQoS()`，`setpoint_raw.cpp:62`）
        · `durability`：`TRANSIENT_LOCAL` → **`VOLATILE`**（同上；旧值的对齐对象是
          uXRCE-DDS 那条 `/fmu/in/*`，与本条无关）
        · `reliability`：`BEST_EFFORT` **不变**（两边都是）
      ⟹ 谁"顺手"只把常量名换掉、值照抄旧的那三个，这条会红 —— 这正是它存在的意义
        （**QoS 不匹配是静默的**：DDS 不报错，只是永不匹配，话题名与 `ros2 topic hz`
        全都看不出原因）。
      ⚠️ 写死的 `depth == 5` 是**故意的**：这一格在黑盒那层读不回来（本用例的 docstring
        就是在讲这件事），所以"常量里写的到底是几"只能在这里钉住。
    """
    from rclpy.qos import HistoryPolicy

    from nmpc.mpc_node import QOS_PUB_ATT

    assert QOS_PUB_ATT.depth == 5, "发布方 QoS 常量里的 depth 必须是 5（mavros 收端的 SensorDataQoS）"
    assert QOS_PUB_ATT.history == HistoryPolicy.KEEP_LAST
    assert QOS_PUB_ATT.reliability == ReliabilityPolicy.BEST_EFFORT
    assert QOS_PUB_ATT.durability == DurabilityPolicy.VOLATILE


def test_动捕那条话题没有速度字段_所以拍不出_tau_的黑盒测试():
    """★ 票面验收项「状态外推 `x + v·τ`」**在这一层无法黑盒验证** —— 原因不是夹具不够好，
    而是**输入里没有 `v`**（[issue 15](../../../.scratch/phase2/issues/15-mocap-velocity-source.md)）。

    ☠️ **【2026-09-29 作废（ticket 15 落地）】** 上面这两句**已不成立** —— 见下面对角线
    ★★★ 那段与 `test_B_档_速度从_velocity_enu_进外推_表现为推力低于悬停`。
    **⚠️ 连本用例的函数名也一起过期了**（`…所以拍不出_tau_的黑盒测试` 这半句现在是假话：
    B 档**拍得出**）。名字**刻意未改**（`mpc-node-seam1.md` §5 与 `test_mpc_node.py:1031/:1133`
    都按名字引用它，改名要一起动；改动前先确认是否值得）。下面这段原文**保留**为当时的第一手推导。

    第一手：`/mocap/pose_enu` 是 `geometry_msgs/PoseStamped`
    （`mocap_rel_pose_node.cpp:110`），而 `PoseStamped` **没有** `twist` 字段
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

    ★★★ 2026-09-29 更新（**ticket 15 已落地**）：动捕这一档**也有速度**了 ——
      `mocap` 节点把 `/vrpn/<刚体>/twist` 原样 republish 成 `/mocap/velocity_enu`
      （`TwistStamped`，与 pose 同一个 frame ⟹ 零变换），节点订这条独立话题。
      ⟹ B 档的 τ 黑盒用例见 `test_B_档_速度从_velocity_enu_进外推_表现为推力低于悬停`。
      ⚠️ **不是**把速度塞进 `PoseStamped` —— 那仍然做不到（下面两条断言），
        所以"速度走独立话题"这件事仍然由它们守着（有 twist 字段的 `PoseStamped` 才该提醒改票）。
    """
    from geometry_msgs.msg import PoseStamped

    assert not hasattr(PoseStamped(), "twist"), \
        "PoseStamped 有 twist 了 ⟹ 速度改走位姿内嵌字段了，本文件与 mocap 节点都要跟着改"
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
      的方向是一对关系）—— ☠️ 2026-10-13（ticket 13 §四③）：这里原来指的是
      `test_A_档_同一条正前方任务_走哪一根轴由_NED_到_ENU_的换算决定`，那条随 A 档被拒
      （§二⑤）**删掉了**（见 `test_A_档_state_source_px4_入口当场拒绝` 上方的 ☠️ 清单 ③）。
      现在接替它的是**动捕档**的 `test_目标在正前方时_body_rate_y_为正_且顶约束表现为打满`
      —— 任务改成正前方，判据落在 `body_rate_y`（而不是 `body_rate_x`）上。
      ⚠️ 换载体之后**"由哪根轴"这句话本身也换了内容**（旧版问的是 NED→ENU 那一步翻不翻，
      新版问的是 FLU 有没有被预先取负，即 N2）⟹ 结论的**形态**一样，**对象**不是同一个。
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
