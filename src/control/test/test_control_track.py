"""control 节点的轨迹跟踪（`TRACK`，**seam 1**）—— ticket 09 的验收清单。

★★ 本文件与 `test_control_takeoff.py` 的**根本**差别：任务推进的入口换人了。
   ticket 08 之前，任务自己会走完（判稳之后保持 N 秒 ⟹ 降落）；ticket 09 起**操作员按键**
   才是唯一入口（§H.5）：第 1 次上升沿 = 解锁起飞（把 ticket 08 那条链原样走一遍），
   第 2 次上升沿 = 开始轨迹（进 `TRACK`，照着 `/traj/reference` 飞）。
   ⟹ 本文件的每条用例都必须**真的按键**（`set_rc_channel(9, ...)` 推两下），
     而不是调 `/control/takeoff` 服务 —— "按键能不能推进任务"正是本张要验的东西本身。

★★ **ticket 07 增补（2026-09-24 用户裁定）改了同一把开关的第 3 种用法**：飞行中**长按 ≥1 s**
   ⟹ 立即自动降落（`end_reason=land_pressed`）。它带来两处**语义改写**，本文件因此不是"补两条
   用例"而是"改形"：
     · 第 2 次按键进 `TRACK` 的动作从**按下**挪到了**松手**（否则"按住不动"这条命令无从表达）；
     · §H.5.1 的"电平本身不携带含义"**收窄**为"*未在飞行态产生边沿的电平*不携带含义"
       ⟹ "在 TRACK 里按着不放"现在**确实**是一条命令（按住够久就降落）。
   两个数先记住：`start_land_hold_s = 1.0`（判据从**电平被确认**起算，即去抖 200 ms 之后）
   ⟹ **物理按住 ≈1.2 s** 才算长按。⟹ 本文件里"短按"用 0.5 s、"长按"用 `LONG_PRESS_HOLD_S`。

★ 一半的夹具是假的 `traj`（`bench.FakeTraj`），另一半是假飞机（`bench.Plant`）：
  前者造"参考来 / 不来 / 断掉 / 跑完"这四种场面（**逐条对着 `traj_node.cpp` 抄**），
  后者让飞机真的跟着目标动。两者缺一，"目标 = 轨迹原点 + 参考"这句话就没有落点
  —— 只发 setpoint 而飞机不动的话，谁都分不清目标算错了没有。

★ 假 EKF 的初始位置**刻意两个轴都非零**（`INIT_X/INIT_Y`），且与动捕原点 (0, 0, 0) 不同：
  「目标 = 轨迹原点 + 参考」与三种错法（直接用参考、用动捕原点加参考、用当前位置加参考）
  只有在这种初始条件下才**分得开**。原点在 (0, 0) 时前两种错法会与正确实现给出同一个数。

验收项 → 用例（工单 `.scratch/phase1/issues/09-track-mission-script.md`）：
  · 开始键的上升沿触发（第 1 次解锁、第 2 次进 TRACK）  → 两次上升沿
  · 开始键之前不发任何控制指令                        → 两次上升沿（按键前的那三条断言）
  · 进 TRACK 时发一次 /traj/start，运行中重选后要重发   → 两次上升沿 / 运行中重选
  · 目标 = 轨迹原点 + 参考（**不是**参考本身）          → 目标等于原点加参考
  · 参考断了：保持最后一个目标 + 按闸门重发 start       → 运行中重选
  · 参考一直不来：原地保持，超时判 `traj_timeout` 降落  → 参考一直不来
  · 完成度到 1.0 ⟹ 任务走完（`end_reason=traj_done`）   → 完成度到 1
  · 上一轮留下的 1.0 不许把新一轮当场收口               → 第二个任务
  · 完成度转发到 /control/traj_progress（逐字）         → 转发完成度

验收项 → 用例（ticket 07 的**增补**那一节，同上）：
  · 第 2 次按键 = **松手**才进 `TRACK`                  → 两次上升沿（`_enter_track` 里的松手）
  · `TRACK` 里短按（不到 1 s）什么都不发生              → 两次上升沿（末尾那一段）
  · `TRACK` 里长按 ≥1 s ⟹ 立即降落、`land_pressed`      → TRACK 里长按开始键
  · `WAIT_START` 里长按 ⟹ 直接降落、一帧 TRACK 都没进过  → WAIT_START 里长按开始键
  · 起飞过程中一直按着 ⟹ 长按被忽略（资格看**按下那一刻**）→ `_reach_wait_start` 末尾两条断言

⚠️ 本文件**不**覆盖：遥控器上 ch9 到底分给了哪个开关（§H.4，EdgeTX 侧的事）、
   三条轨迹实飞、一键盘 bringup —— 那几条是台架外的验收项，工单里对应的小方框保持未勾。
"""

import contextlib
import copy

import pytest
from std_msgs.msg import Float32

from bench import DEFAULTS, FakeTraj, Plant, Scenario, fly, fly_until
from bench.fake_mavros import mavros_default_qos

#: DUT =（包名，可执行名）—— 由 `ament_index_python` 解析成 <prefix>/lib/control/control_node
DUT = ("control", "control_node")

#: §H.1 的通道与量程（`control.yaml` 的默认值在这里显式写出来，让断言旁边就能读到口径）。
START_CHANNEL = 9
START_HIGH_US = 2000
START_LOW_US = 1000
#: 按键/松手各要**保持**多久（s）。
#  ★★ 这个数不能贴着 `debounce_ms` 取 —— 200 ms 的去抖判据对上 10 Hz 的采样**正好是 3 帧**
#     （第 1 帧起算、第 3 帧到点），而"第 3 帧到点"要求两帧间隔**恰好 ≥ 0.100 s**：
#     DDS 送达抖动让某一对间隔短了 100 µs，第 3 帧就不算数，得等第 4 帧。
#     按 0.35 s 按住时窗口里正好只有 3 帧（首帧相位均匀落在 0~0.1 s）⟹ 大约一半的机会
#     **按不动**，症状与"通道号写错 / 没接上"一模一样（本文件第一次跑就是这么红的：
#     第 2 次按键没有产生任何边沿，日志里连「被丢掉」都没有）。
#   ⟹ 按 0.5 s：窗口里必有 4–5 帧 ⟹ 到点那一刻至少过了 0.3 s，怎么抖都确认得上。
#   ⚠️ 这一条也是**给现场的数**：§H.1 的 200 ms 去抖 + 10 Hz 采样意味着开关要真的停住
#     ≥0.3 s 才算一次（"弹一下"的拨法有相当概率什么都不发生）。
START_HOLD_S = 0.5

#: **长按**要按住的时长（s）—— `start_land_hold_s`（1.0）+ 去抖（0.2）再留一帧余量。
#  ★ 为什么不能就按 1.2 s：判据从**电平被确认**那一帧起算，而确认落在"置高位之后的第 3 帧"
#    （10 Hz ⟹ 首帧相位均匀落在 0~0.1 s）⟹ 物理按住时长最坏要 1.4 s 才能凑满 1.0 s 的计时。
#    ★ 也不能取大：下面要用"松手之前就已经收口了"来钉"到点就执行"这条判据，按住越久越弱。
LONG_PRESS_HOLD_S = 1.5

#: 假 EKF 的初始位置。★ **两个轴都非零**，理由见文件头。
INIT_X, INIT_Y = 0.3, -0.2

#: `control` 对外提供的完成度观测面（**逐字**转发 `/traj/progress`，见 `control.yaml`）。
FWD_TOPIC = "/control/traj_progress"

#: **降级宣告行**的 token（`control_node.cpp::log_degrade` 的形状：`degrade=%s（持续 %.1f s…）`）。
#  ⚠️ 不能拿光秃秃的 `degrade=traj_timeout` 去 grep 日志：节点的**说明文字**里逐字写着这个
#     token（进 TRACK 那条 WARN 的「超过 3.0 s 仍然没有 ⟹ degrade=traj_timeout」）⟹ 只搜 token
#     会把那条 WARN 读成一次**真的降级**。
#     ★ 这个坑是 ticket 09 第一次跑本文件时踩出来的：`test_完成度到_1…` 红在它上面（"正常
#       飞完被记成了一次失效"），而日志里那三处 `degrade=traj_timeout` 全是**散文**。
DEGRADE_TRAJ_TIMEOUT = "degrade=traj_timeout（持续"

#: `TRACK` 里**参考流进出**的两条宣告（`control_node.cpp::detect_traj_stream`）。
#  ⚠️ 必须连着 `**` 一起匹配：节点在别的**说明文字**里引用了这两句话
#     （进 TRACK 那条 WARN 里写着「见紧接着的那一条「轨迹参考已经来了」」）⟹ 数裸句子会多算
#     —— `test_第二个任务…` 第一次跑就是这么红的（数出 4 条，其中 2 条是散文）。
#  ★ 本文件为同一类坑红了**两次**（`degrade=traj_timeout` 一次、这里一次），教训是一样的：
#    **日志的 token 与散文共用同一个字符串空间**，断言要钉在"宣告行的形状"上，不是钉在词上。
TRAJ_REF_ARRIVED = "**轨迹参考已经来了**"
TRAJ_REF_LOST = "**轨迹参考断了**"

#: 两条**互不相同**的参考（相对轨迹原点）：换一条 ⟹ 目标的每个轴都看得见变化。
REF_X = (0.5, 0.0, 0.0)
REF_Y = (0.0, 0.4, 0.0)

#: 与 `config/control.yaml` 一致的参数。⚠️ 只把**时间**相关的改成小值好让用例跑快些；
#: 判据阈值（10 cm / 0.1 m/s / 0.5 m/s）与 §H.1 的那一组（0.75 / 0.25 / 200 ms）一律照原值
#: —— 改它们就不是在验工单了。
PARAMS = {
    "setpoint_rate_hz": 50.0,
    "enter_offboard": True,
    "offboard_prestream_s": 1.0,
    "offboard_retry_s": 1.0,
    "state_timeout_s": 2.5,
    "local_pos_timeout_s": 0.5,
    "mocap_timeout_s": 0.5,
    # —— 开始键（§H.1 定案的那一组）——
    "start_channel": START_CHANNEL,
    "start_threshold": 0.75,
    "release_threshold": 0.25,
    "debounce_ms": 200.0,
    # ⚠️ 这两个端点必须是 **float**：节点把它们声明成 `double`（它要拿去做算术），
    #   而 rclcpp **不做** int→double 的隐式转换 —— 传 `1000` 会让节点在构造参数时
    #   抛 `InvalidParameterTypeException` **当场退进程**（症状是"接不上任何话题"）。
    "start_low_us": float(START_LOW_US),
    "start_high_us": float(START_HIGH_US),
    # ★ ticket 07 增补：飞行中长按开始键满这么久 ⟹ 立即自动降落（`control.yaml` 的默认值）。
    #   照原值写 1.0 —— 用例里的"短按"（0.5 s）与"长按"（`LONG_PRESS_HOLD_S`）都是按它定的。
    "start_land_hold_s": 1.0,
    # —— ticket 08 的起飞/悬停判据（本文件要靠它走到 WAIT_START）——
    "takeoff_altitude_m": 1.0,
    "climb_speed_mps": 0.5,
    "takeoff_alt_tol_m": 0.10,
    "takeoff_vel_tol_mps": 0.10,
    "takeoff_settle_s": 1.0,
    "hover_pos_tol_m": 0.10,
    "hover_vel_tol_mps": 0.10,
    "hover_settle_s": 1.0,
    # ★ WAIT_START 的兜底超时**必须放大**：本文件的任务推进靠按键，不是靠等超时。
    #   留着默认的 300 s 只会让"按键没生效"这类失败看起来像卡死（而不是超时并降落）。
    "wait_start_timeout_s": 120.0,
    "takeoff_timeout_s": 5.0,
    # —— ticket 09 的两条时限（照 `control.yaml` 的默认值）——
    #   ★ 断言里的时间窗全是按这两个数写的 ⟹ 改它们必须同时改断言。
    "traj_stream_gap_s": 0.5,
    "traj_timeout_s": 3.0,
}


# ───────────────────────────── 小工具 ─────────────────────────────

def _config():
    """把假 EKF 的初始位置搬到 (`INIT_X`, `INIT_Y`, 0)。

    ★ 走 `initial.position` 而不是起场景之后 `set_position`：后者会让 DUT 在 STANDBY 里
      一边喊着"原地保持"，一边看着飞机被搬走（那时它发的目标还是旧位置）。
    """
    cfg = copy.deepcopy(DEFAULTS)
    cfg["initial"]["position"] = [INIT_X, INIT_Y, 0.0]
    return cfg


def _target(sc):
    """最近一帧 setpoint 的目标位置 `(x, y, z)`（**ROS/ENU**，与假 EKF 报的位置同口径）。"""
    p = sc.setpoints()[-1].msg.position
    return (p.x, p.y, p.z)


def _press_key(sc, hold_s=START_HOLD_S):
    """把 ch9 拨到高位并按住 `hold_s` 秒 ⟹ 确认**一条**上升沿（§H.5：边沿，不是电平）。

    ★ ticket 07 增补之后它有两个用法，差别**只在这个 `hold_s`**：
      · 默认 0.5 s = **短按**：只产生边沿（长按门槛 1.0 s，见 `start_land_hold_s`）；
      · `LONG_PRESS_HOLD_S` = **长按** ⟹ 到点就立即自动降落。
    ⚠️ 本函数**不松手**（松手是 `_release_key` 的事），而长按计时**在按住期间一直走**
      —— 短按用完如果迟迟不松手，1 s 之后照样会冒出一条长按事件。⟹ 短按的调用点后面
      必须紧跟着松手，中间别插长动作（`test_两次上升沿` 末尾那一段就是这么写的）。
    """
    sc.fake.set_rc_channel(START_CHANNEL, START_HIGH_US)
    sc.hold(hold_s)


def _release_key(sc):
    """放回低位并确认。

    ★ §H.5.2：**上电即高位不算边沿**（必须先看见低位）⟹ 每次按键之前都必须走这一下，
      否则第二次按键会被当成"一直按着"，一个边沿都不产生。
    """
    sc.fake.set_rc_channel(START_CHANNEL, START_LOW_US)
    sc.hold(START_HOLD_S)


def _reach_wait_start(sc, traj):
    """把场景推到 **WAIT_START 等第 2 次按键**，返回假飞机。

    ★ 与 `test_control_takeoff.py::_reach_offboard` 同源，但多两件事：
      ① 等 DUT 接上 `/traj/reference`（ticket 09 的接口，本文件的一切都建在它上面）；
      ② 走**第 1 次上升沿**（按键）而不是调 `/control/takeoff` 服务。
    """
    for key in ("state", "local_position", "local_velocity", "rc", "battery"):
        assert sc.wait_for_subscribers(key), f"DUT 没有接上 /mavros 的 {key}"
    assert sc.poll(lambda: sc.mocap_source.matched_subscribers() >= 1), "DUT 没有接上动捕话题"
    assert sc.poll(lambda: traj.matched_subscribers() >= 1, timeout_s=15.0), (
        "DUT 没有接上 /traj/reference ⟹ 它的 traj 接口没起来（下面「发了 /traj/start」"
        "就没意义了）")
    assert sc.wait_for_setpoints(20), "守卫成立之后应当开始以 50 Hz 流 setpoint"
    sc.wait_for(lambda: sc.fake.mode == "OFFBOARD", timeout_s=8.0,
                message="DUT 没有把（假的）PX4 拉进 OFFBOARD")
    sc.hold(1.2)   # /mavros/state 是 1 Hz，解锁前置里的 in_offboard 最多晚 1 s 才看得见

    # —— 按键**之前**：飞机一点都没被驱动（§H.5 / 工单"开始键之前不发控制指令"）——
    assert not sc.fake.arming_requests, (
        "开始键还没按，就已经有解锁请求了 —— 飞机在自己动自己（§H.5 明令禁止）")
    assert traj.starts == [], "还没进 TRACK 就发了 /traj/start"
    assert _target(sc) == pytest.approx(sc.fake.position, abs=1e-6), (
        "开始键之前的目标必须就是**当前位置**（原地保持）：这一刻还没有任何任务，"
        f"目标 {_target(sc)} vs 位置 {sc.fake.position}")

    plant = Plant(sc, rate_mps=1.5)
    _press_key(sc)   # ★ 第 1 次上升沿
    fly_until(sc, plant, lambda: sc.control_state() == "AUTO/WAIT_START", timeout_s=25.0,
              message="第 1 次上升沿之后没有走到 WAIT_START（解锁 / 爬升 / 判稳那一段）")
    assert "第 1 次上升沿" in sc.dut_log(), "开始键的第 1 次上升沿没有被节点认出来"
    assert len(sc.fake.arming_requests) == 1, (
        f"第 1 次按键该解锁**一次**，实际 {len(sc.fake.arming_requests)} 次")

    # ★★ ticket 07 增补的**反例**，在这一步免费验掉了：第 1 次按键之后我们**一直没松手**
    #   （整个爬升过程开关都停在高位），1 s 到点时长按事件**照样会**产生 —— 但这次按下发生在
    #   **未解锁**的时候 ⟹ 状态机必须忽略它。若把资格判据写成"到点那一刻是否在飞"，
    #   此刻飞机正爬升到 1 m ⟹ 它会当场请求 `AUTO.LAND`，**刚离地就自己往回坐**
    #   （这条链的第一手记录见 ticket 07 增补的 Comments 第 1 条）。
    assert not any(r.custom_mode == "AUTO.LAND" for r in sc.fake.mode_requests), (
        "起飞过程中一直按着开始键，1 s 到点就把飞机判去降落了 —— 资格必须看**按下那一刻**")
    assert "长按**被忽略**了 —— 这一次按下发生在" in sc.dut_log(), (
        "那条长按事件连日志都没留下 —— 那一次按下确实产生了长按（开关一直停在高位），"
        "证据留在日志里正是为了现场能分清「事件没产生」与「产生了但没资格」"
        "（另一条同前缀的日志是夺权那条，所以这里连着后半句一起匹配）")
    return plant


def _enter_track(sc, plant, traj, timeout_s=10.0):
    """第 2 次**短按 + 松手** ⟹ `TRACK`。返回**松手那一刻**的场景时刻（给"start 什么时候发的"用）。

    ★★ ticket 07 增补之后这里多了一步**松手**，而且它不是"顺手补的"：裁定 3 把"第 2 次按键
      = 开始轨迹"的动作从**按下**挪到了**松手** —— 否则"按住不放"这条命令（长按降落）
      在 `WAIT_START` 里无从表达（按下就已经开跑了轨迹，那 1 s 里飞机已经跟着参考飞了）。
    ★ 返回**松手**时刻而不是按下时刻：`/traj/start` 现在紧跟着松手走，用按下时刻当基准
      会把"按键到松手"这 0.5 s 白算进去（旧版那条 1 s 的窗会变得很紧）。
    ⚠️ 那 0.5 s 的短按**远不到**长按门槛（1.0 s，且判据从电平确认起算）⟹ 不会误触发降落。
    """
    _release_key(sc)
    _press_key(sc)     # 第 2 次上升沿 ⟹ `WAIT_START` 里只记下"欠一次开始轨迹"
    t_release = sc.t()
    _release_key(sc)   # ★ 松手 ⟹ 才真的进 `TRACK`
    fly_until(sc, plant, lambda: sc.control_state() == "AUTO/TRACK", timeout_s=timeout_s,
              message="第 2 次上升沿（短按后松手）没有进 TRACK")
    assert "第 2 次上升沿" in sc.dut_log(), "开始键的第 2 次上升沿没有被节点认出来"
    assert "松手** ⟹ 进入 TRACK" in sc.dut_log(), (
        "松手那一刻没有留下「进入 TRACK」的日志（`control_node.cpp::log_start_release`）—— "
        "现场读日志时，「按了没反应」与「松手才生效」就靠这条分辨")
    return t_release


def _land_and_disarm(sc, plant):
    """等任务正常收口（`MANUAL/LAND`）、假 PX4 上锁、节点复位回 STANDBY。

    ⚠️ 夹具**不模拟降落**：`Plant` 只跟着 setpoint 动，而降落段我们不发 setpoint
      （那是 PX4 的事）⟹ 飞机在假 EKF 眼里会一直停在空中。所以这里手动把它放回地面，
      否则下一条任务会从 1 m 高处"再次起飞"。
    """
    fly_until(sc, plant, lambda: sc.control_state() == "MANUAL/LAND", timeout_s=10.0,
              message="任务走完之后没有进 LAND")
    sc.wait_for(lambda: any(r.custom_mode == "AUTO.LAND" for r in sc.fake.mode_requests),
                timeout_s=6.0,
                message="任务走完却没有请求 AUTO.LAND（上锁交给 PX4，但降落必须是我们请求的）")
    sc.fake.set_state(armed=False)   # PX4 的 COM_DISARM_LAND 会做，我们只等 armed 降沿
    sc.wait_for(lambda: sc.control_state() == "MANUAL/STANDBY", timeout_s=8.0,
                message="落地上锁之后应当自动复位回 STANDBY（不需要重启节点）")
    sc.fake.set_position(INIT_X, INIT_Y, 0.0)


@contextlib.contextmanager
def _mission(reference_fn=lambda _t: REF_X, duration_s=10.0, selected=True):
    """走到 WAIT_START 等第 2 次按键，然后把场景 / 假 traj / 假飞机交出去。

    ★★ 假 traj 必须建在 `start_dut()` **之前**：`/traj/start` 是"我们发一次、`traj` 收"的
      方向，而 DDS 发现是异步的 —— 夹具若在 DUT 已经进 TRACK 之后才出现，那一次 `start`
      就发到一个还不存在的订阅者上（丢了）。建在前面，发现早就在那儿了。
      ⚠️ 反方向（DUT 的 `/traj/reference` 订阅）由 `_reach_wait_start` 的发现屏障兜住。
    """
    with Scenario(dut=DUT, params=PARAMS, config=_config(), start_dut=False) as sc:
        traj = FakeTraj(sc.node, sc, duration_s=duration_s, selected=selected,
                        reference_fn=reference_fn)
        sc.start_dut()
        plant = _reach_wait_start(sc, traj)
        yield sc, traj, plant


# ─────────────────────── 开始键的三条动作（两次短按 + 一次长按）───────────────────────

def test_两次上升沿_第一次解锁起飞_第二次短按松手进_TRACK():
    """§H.5 的整条语义，一次按键走完：**第 1 次 = 解锁起飞，第 2 次 = 开始轨迹**。

    ★ 为什么一条用例管两次按键：它们的**分工**只有放在一起才看得见 ——
      第 1 次按键时飞机必须还没进 TRACK（否则"第 1 次解锁"这条语义是空的），
      第 2 次按键时飞机必须已经在天上且没解锁请求再发出去（否则第 2 次又解锁了一遍）。
    ★ 附带验 ticket 07 增补之后**剩下**的那半边 §H.5.1：在 TRACK 里**短按**（不到 1 s）
      什么都不发生 —— 它只锁存一条"降落资格"，而资格在**松手**那一刻就作废。
      §H.5.1 的条文本身也被这次增补**收窄**了：原来说"电平本身不携带含义"，
      现在正确的是"**未在飞行态产生边沿的电平**不携带含义"（产生过边沿、且持续为高
      ≥ `start_land_hold_s` 的电平**确实**是一条命令 ⟹ 见下面两条长按用例）。
    ★ 旧版这里最后一段写的是"边沿在 TRACK 里被**丢掉**"—— 增补之后那句话不再成立：
      现在它给资格、并留一条日志。`kRequested`/`kDone` 里被丢掉的那一支由 `test_fsm.cpp`
      直接钉住（seam 1 到不了那个瞬间：`kRequested` 是受理到解锁之间的几帧）。
    """
    with _mission() as (sc, traj, plant):
        # —— 第 1 次上升沿的后果在 `_reach_wait_start` 里已断言（解锁一次、进 WAIT_START）——
        assert sc.control_state() == "AUTO/WAIT_START"
        # ★ 到这一步为止 `/traj/start` 一帧都不能发过（工单"开始键之前不发控制指令"）。
        assert traj.starts == [], "第 2 次按键之前就发了 /traj/start"

        t_release = _enter_track(sc, plant, traj)
        assert sc.control_state() == "AUTO/TRACK"

        # —— 进 TRACK 时发**一次** ——
        sc.hold(1.0)   # 留足时间让"多余的重发"暴露出来（闸门是 traj_stream_gap_s = 0.5）
        assert len(traj.starts) == 1, (
            f"进 TRACK 时 /traj/start 该发**一次**，实际 {len(traj.starts)} 次"
            f"（时刻 {[round(t - t_release, 2) for t in traj.starts]}）—— 参考流是活的，"
            "重发的闸门不该放行")
        assert 0.0 <= traj.starts[0] - t_release <= 1.0, (
            f"那一次 /traj/start 应当紧跟着**松手**发出，实际晚了 "
            f"{traj.starts[0] - t_release:.2f} s")
        assert len(sc.fake.arming_requests) == 1, "第 2 次按键不该再发一次解锁请求"

        # —— TRACK 里的**短按**：只锁存一条降落资格，松手即作废 ⟹ 什么都不发生 ——
        #   ★ 这一段同时是"边沿必须留下日志"那条纪律的现场证据：TRACK 里的按下**不是**
        #     "被丢掉"（它给了资格），但照样得有一句话说明它做了什么。
        n_before = len(traj.starts)
        _press_key(sc)     # 0.5 s 短按 ⟹ 边沿，远不到 start_land_hold_s = 1.0
        assert "锁存降落资格" in sc.dut_log(), (
            "TRACK 里的边沿没有留下日志 —— 「按了没反应」只有两种可能：没产生边沿，"
            "或者产生了但当时拿不到资格，日志必须能分开这两者")
        _release_key(sc)   # ★ 松手：这次按下的降落资格作废（长按计时也一清）
        sc.hold(1.0)
        assert sc.control_state() == "AUTO/TRACK", (
            "TRACK 里短按（不到 start_land_hold_s 就松手）不该改任务态 —— "
            "它只锁存资格，资格在松手那一刻就作废了")
        assert len(traj.starts) == n_before, "被丢掉的那次边沿仍然触发了 /traj/start"
        assert not any(r.custom_mode == "AUTO.LAND" for r in sc.fake.mode_requests), (
            "短按松手之后飞机去降落了 —— 长按门槛（1 s）没有生效")
        assert len(sc.fake.arming_requests) == 1


# ──────────────────── 长按开始键 ≥1 s ⟹ 立即自动降落（ticket 07 增补）────────────────────

def test_TRACK里长按开始键_到点就立即降落():
    """★★ ticket 07 增补的主验收：**飞行中长按开始键 ≥1 s** ⟹ 立即走"正常收口"那条出口。

    ★ 三件事必须各有一条证据，缺一条都不算验过：
      ① `end_reason=land_pressed` —— 与"飞完"（`traj_done`）/"超时"（`traj_timeout`）分开：
         三条出口的**动作逐字相同**（停发 + `AUTO.LAND`），只有这句日志能分开它们；
      ② setpoint **停发**（真收口，不是"还在流但飞机在降"）；
      ③ **松手之前**就已经决定了 —— 裁定是"到点就执行"，不是"松手才执行"。用
         "开关此刻仍停在高位"来钉（`rc_channels[8]` 就是 ch9 的当前值）。
    ★ 还要钉住"它**不是一次降级**"：`degrade=` 一条都不许有 —— 长按是操作员要求的收口，
      现场读到 `degrade=` 会去查故障，而真相是"我叫它停的"。
    """
    with _mission(reference_fn=lambda _t: REF_X, duration_s=30.0) as (sc, traj, plant):
        _enter_track(sc, plant, traj)
        fly(sc, plant, 1.0)   # 先在 TRACK 里真飞一会儿：证明停的是**正在飞**的那个任务
        assert sc.control_state() == "AUTO/TRACK"
        n_arm = len(sc.fake.arming_requests)
        n_start = len(traj.starts)

        _press_key(sc, hold_s=LONG_PRESS_HOLD_S)   # ★ 按住（判据从电平确认起算 ⟹ 物理 ≈1.2 s）
        fly_until(sc, plant, lambda: sc.control_state() == "MANUAL/LAND", timeout_s=5.0,
                  message="TRACK 里长按开始键 1 s 没有立即降落")

        assert sc.fake.rc_channels[START_CHANNEL - 1] == START_HIGH_US, (
            "开关此刻应当还停在高位 —— 这次收口必须是**按满时长到点**那一刻做的，"
            "不是松手做的（裁定：到点就执行）")
        assert "end_reason=land_pressed" in sc.dut_log(), (
            "长按降落必须打出 `end_reason=land_pressed`（工单要的可 grep 证据）")
        assert "end_reason=traj_done" not in sc.dut_log(), (
            "长按降落被记成了「轨迹飞完」—— 现场会把一次人工叫停读成一次成功")
        assert DEGRADE_TRAJ_TIMEOUT not in sc.dut_log(), (
            "长按降落被记成了一次**失效**（降级）—— 它是操作员要求的收口，不是故障")
        assert any(r.custom_mode == "AUTO.LAND" for r in sc.fake.mode_requests), (
            "长按之后没有请求 AUTO.LAND（收口 = 停发 setpoint + 把降落交给 PX4）")

        frozen = len(sc.setpoints())
        sc.hold(1.0)
        assert len(sc.setpoints()) == frozen, (
            f"长按之后必须停止发 setpoint（冻结后又来了 {len(sc.setpoints()) - frozen} 帧）")
        assert len(traj.starts) == n_start, "长按降落不该再发 /traj/start"
        assert len(sc.fake.arming_requests) == n_arm, "长按降落不该再发解锁请求"

        _release_key(sc)   # 事后松手：**不该改变任何结论**（决定早就做完了）
        assert sc.control_state() == "MANUAL/LAND", "松手把已经决定的收口又改了"

        # —— 收尾与另外两条出口逐字相同（复用同一个出口，不是新造一条）——
        _land_and_disarm(sc, plant)


def test_WAIT_START里长按开始键_一帧_TRACK_都没进就降落():
    """`WAIT_START`（等第 2 次按键）里长按 ⟹ **直接**降落，一帧 `TRACK` 都没进过。

    ★ 这条验的是裁定 3 的另一半：`WAIT_START` 里长按**算数**，因此"第 2 次按键 = 开始轨迹"
      的动作必须从**按下**挪到**松手**（否则按下就已经开跑轨迹，那 1 s 里飞机已经跟着参考飞了，
      "按住不动"这条命令在 `WAIT_START` 里就无从表达）。
    ★ "没进过 TRACK"的可判据有两条，都用上：
      ① `/traj/start` **一帧都没发过** —— 进 TRACK 的第一件事就是发它（`traj.starts`）；
      ② `/control/state` 的**历史**里没有出现过 `AUTO/TRACK`（比"当前状态"强：
         它能排除"进去过又出来了"这种一闪而过的场面）。
    """
    with _mission(reference_fn=lambda _t: REF_X, duration_s=30.0) as (sc, traj, plant):
        assert sc.control_state() == "AUTO/WAIT_START"
        # ★ 必须先松手：第 1 次按键之后开关一直停在高位（`_reach_wait_start` 按下去就没松），
        #   而 §H.5.2 要求"只有确认过低位，高位才是一次边沿"—— 不松手就没有新边沿，
        #   也就没有"这次按下有资格"这回事。现场纪律"拨完要拨回"在这一步是硬前提。
        _release_key(sc)

        _press_key(sc, hold_s=LONG_PRESS_HOLD_S)
        fly_until(sc, plant, lambda: sc.control_state() == "MANUAL/LAND", timeout_s=5.0,
                  message="WAIT_START 里长按开始键没有直接降落")

        assert traj.starts == [], (
            "长按降落之前发了 /traj/start —— 那是「进 TRACK」的动作（裁定 3 要求直接降落）")
        assert all(state != "AUTO/TRACK" for _, state in sc.control_state_history()), (
            "长按降落之前进过 TRACK —— WAIT_START 里的长按必须**直接**降落")
        assert "end_reason=land_pressed" in sc.dut_log(), (
            "WAIT_START 里的长按降落没打 `end_reason=land_pressed`")
        _release_key(sc)
        assert sc.control_state() == "MANUAL/LAND", (
            "松手把飞机开跑轨迹了 —— 长按已经把「欠下的开始轨迹」清掉了")
        _land_and_disarm(sc, plant)


# ───────────────────────────── 目标 = 轨迹原点 + 参考 ─────────────────────────────

def test_目标等于轨迹原点加参考_不是参考本身():
    """★★ ticket 09 Comments 第 1 条：`setpoint = traj_origin + /traj/reference`。

    ★ 为什么这是本张最要命的一条：`traj` 发的是**相对轨迹原点**的位置（高度从 0 起步），
      直接拿它当目标 ⟹ 飞机当场俯冲扎向地面（目标高度 0）。而"直接转发"与"加原点"
      在原点恰为 (0, 0) 时给出同一个数 ⟹ 假 EKF 的初始位置必须**两个轴都非零**。
    ★ 断言用两个口径把这条钉死：
      ① **绝对**：目标的水平两轴 = 初始位置 + 参考（初始位置非零 ⟹ 与动捕原点分得开）；
      ② **差值**：目标 − WAIT_START 的目标（= 轨迹原点，工单要求它就等于悬停点）
        必须**逐字**等于参考。
    """
    with _mission(reference_fn=lambda _t: REF_X, duration_s=10.0) as (sc, traj, plant):
        # WAIT_START 的目标**就是**轨迹原点（"停在原地"）——
        # 它同时是下面那个差值的基准，也是"轨迹原点记在哪儿"的可观测证据。
        origin = _target(sc)
        assert origin == pytest.approx((INIT_X, INIT_Y, 1.0), abs=0.05), (
            f"WAIT_START 的目标应当是轨迹原点 ≈ (初始位置, 1 m)，实际 {origin}")

        _enter_track(sc, plant, traj)
        # 参考还没来之前先原地待着（目标 = 轨迹原点），来了之后才动。
        fly_until(sc, plant, lambda: _target(sc)[0] > origin[0] + 0.4, timeout_s=8.0,
                  message="进 TRACK 之后目标没有跟着 /traj/reference 走")
        target = _target(sc)

        assert target == pytest.approx(
            (origin[0] + REF_X[0], origin[1] + REF_X[1], origin[2] + REF_X[2]), abs=1e-5), (
            f"目标必须是「轨迹原点 + 参考」= {origin} + {REF_X}，实际 {target}。"
            "若水平值等于参考本身（0.5, 0.0），就是**把原点丢了**（ticket 09 Comments ①）；"
            "若等于当前位置 + 参考，说明原点是「此刻」而不是判稳那一刻记下的。")
        assert target[2] == pytest.approx(origin[2], abs=1e-5), (
            f"参考的 z 是**相对**量（这条轨迹恒为 0）⟹ 目标高度必须仍是轨迹原点的高度 "
            f"{origin[2]:.3f}，实际 {target[2]:.3f} —— 高度掉了就是丢掉原点的那个 bug")
        assert target[2] > 0.9, "目标高度必须还在 1 m 附近（掉了就是「直接拿参考当目标」）"

        # 飞机真的朝那个目标去了（不是"目标对了但没人发"）。
        fly_until(sc, plant, lambda: sc.fake.position[0] > origin[0] + 0.3, timeout_s=8.0,
                  message="飞机没有朝「轨迹原点 + 参考」飞")
        assert sc.control_state() == "AUTO/TRACK", "跟踪过程中不该离开 TRACK"


# ───────────────────────────── 运行中重选轨迹 ─────────────────────────────

def test_运行中重选轨迹_保持上一个目标并按闸门重发_start():
    """§H / ticket 09 Comments 第 2 条 + 第 3 条的**同一件事的两半**。

    ★ 运行中 `traj` 被重选（`select_kind()`）会 `running_ = false; elapsed_ = 0`，
      而它**不发任何消息** ⟹ 这一件事在话题层**只能表现为静默**：参考流断掉。
      于是 DUT 只能：① 保持最后一个目标（**不许**回到轨迹原点 —— 那是一道肉眼看不见的位移
      指令，而飞机此刻离原点可能半条轨迹远）；② 按 `traj_stream_gap_s` 的闸门重发
      `/traj/start`，把 `traj` 重新拉起来。
    ★ 闸门**必须真的等满**：重发会让 `traj` 把参考时钟归零 —— 它若其实还在跑，
      飞机就当场掉头飞回起点。所以这里既验"到点会重发"，也验"没到点不许重发"。
    """
    with _mission(reference_fn=lambda _t: REF_X, duration_s=10.0) as (sc, traj, plant):
        origin = _target(sc)
        _enter_track(sc, plant, traj)
        fly_until(sc, plant, lambda: _target(sc)[0] > origin[0] + 0.4, timeout_s=8.0,
                  message="进 TRACK 之后目标没有跟着参考走（后面两条断言就没了前提）")
        assert len(traj.starts) == 1

        # —— 运行中重选：流**静默**断掉，时钟归零（夹具照 `traj_node.cpp` 的样子做）——
        traj.select(reference_fn=lambda _t: REF_Y)
        t_silence = sc.t()

        # ① 闸门没到 ⟹ 不许重发，且目标必须**保持上一个**（不是回到轨迹原点）。
        sc.hold(0.4)   # < traj_stream_gap_s = 0.5
        assert len(traj.starts) == 1, (
            "参考才断了 0.4 s（闸门 0.5 s）就重发了 /traj/start —— 重发会把 traj 的参考时钟"
            "归零，而它若其实还在跑，飞机就当场掉头飞回起点")
        held = _target(sc)
        assert held == pytest.approx((origin[0] + REF_X[0], origin[1], origin[2]), abs=1e-5), (
            f"参考断了之后目标必须**保持最后一个** {origin[0] + REF_X[0]:.3f}，实际 {held} —— "
            "回到轨迹原点是一道飞机真的会执行的位移指令（跑完前 0.5 s 它就是这样起飞的）")

        # ② 闸门到点 ⟹ 重发一次（这是"运行中重选"唯一的自救动作）。
        assert sc.poll(lambda: len(traj.starts) > 1, timeout_s=3.0), (
            "参考断了超过闸门（traj_stream_gap_s）却没有重发 /traj/start ⟹ 运行中重选轨迹"
            "之后飞机永远停在原地")
        t_resend = traj.starts[1]
        # ★ 与 `test_完成度到_1…` 的负向断言配成一对：真的断了就**必须有**这条日志
        #   （否则那条"正常飞完不许出现它"只是在证明"这条日志根本不会打"）。
        assert TRAJ_REF_LOST in sc.dut_log(), (
            "参考真的断了（静默 > 闸门）却没有留下「轨迹参考断了」这条日志")
        assert 0.35 <= t_resend - t_silence <= 1.5, (
            f"重发时刻 {t_resend - t_silence:.2f} s 与闸门（{PARAMS['traj_stream_gap_s']} s）"
            "对不上：太早会在 traj 还在跑的时候把它的时钟归零，太晚说明闸门根本没按这个数走")

        # ③ 拉起来之后，目标跟着**新**参考走（证明重发真的让 traj 重新开始跑）。
        fly_until(sc, plant, lambda: _target(sc)[1] > origin[1] + 0.3, timeout_s=8.0,
                  message="重发 /traj/start 之后目标没有跟着新参考走（start 没生效？）")
        new_target = _target(sc)
        assert new_target == pytest.approx(
            (origin[0], origin[1] + REF_Y[1], origin[2]), abs=1e-5), (
            f"重选之后目标应当是「轨迹原点 + 新参考」= {origin} + {REF_Y}，实际 {new_target}")


# ───────────────────────────── 参考一直不来 ─────────────────────────────

def test_参考一直不来_原地保持_超时后判_traj_timeout():
    """★★ ticket 09 唯一那条"按键有效果、飞机却不动"的路径 —— 必须有人收拾。

    ★ 场面的来源是 `traj_node.cpp::on_start`：**没选中轨迹**时它只打一条 WARN 就忽略 start
      ⟹ DUT 那边一切正常（进了 TRACK、发了 start），却一帧参考都收不到。
      现场最像"按键没反应"的就是这一种，所以台架上必须能造出来。
    ★ 要验三件事：
      ① `TRACK` 里没有参考时飞机**原地保持**（目标 = 轨迹原点，不是"漂到别处去"）；
      ② 一直在按闸门重发 `/traj/start`（那是操作员补选了轨迹之后唯一的自救机会）；
      ③ 超过 `traj_timeout_s` ⟹ `degrade=traj_timeout` ⟹ 停发 + 请求降落。
         ★ 它**不是**"轨迹飞完了"（`end_reason` 应当是 `kNone`）：两者动作相同、意思相反，
           日志必须分得开，否则现场会把一次失效读成一次成功。
    """
    with _mission(reference_fn=lambda _t: REF_X, duration_s=10.0, selected=False) as (sc, traj, plant):
        origin = _target(sc)
        _enter_track(sc, plant, traj)

        fly(sc, plant, 1.0)   # 远小于 traj_timeout_s = 3.0
        assert sc.control_state() == "AUTO/TRACK", (
            f"参考还没到、也没超时，就离开了 TRACK（状态 {sc.control_state()!r}）")
        assert _target(sc) == pytest.approx(origin, abs=1e-5), (
            f"参考还没到时目标必须**原地保持**（= 轨迹原点 {origin}），实际 {_target(sc)}")
        assert len(traj.starts) >= 1, "进了 TRACK 却没有发 /traj/start（参考永远不可能来）"

        # ② 重发：闸门 0.5 s、已过 1.0 s ⟹ 至少又来了一次。
        assert sc.poll(lambda: len(traj.starts) >= 2, timeout_s=2.0), (
            "参考一直不来却没有按闸门重发 /traj/start —— 那是操作员补选轨迹之后唯一的自救机会")

        # ③ 超时 ⟹ 降级 + 请求降落。
        fly(sc, plant, 2.6)   # 累计 ≈3.6 s > traj_timeout_s
        assert sc.control_state() == "MANUAL/LAND", (
            f"参考断了超过 traj_timeout_s 之后应当停发并请求降落，实际状态 "
            f"{sc.control_state()!r}")
        assert DEGRADE_TRAJ_TIMEOUT in sc.dut_log(), (
            "超时降级的 token 没有进日志（工单要的可 grep 证据）")
        sc.wait_for(lambda: any(r.custom_mode == "AUTO.LAND" for r in sc.fake.mode_requests),
                    timeout_s=6.0, message="判了 traj_timeout 却没有请求 AUTO.LAND")
        frozen = len(sc.setpoints())
        sc.hold(1.0)
        assert len(sc.setpoints()) == frozen, (
            f"判了降级之后必须**停止**发 setpoint（冻结后又来了 {len(sc.setpoints()) - frozen} 帧）")
        # ★ 与"任务正常走完"分开：这一条不是跑完了（见 `end_reason` 的说明）。
        assert "end_reason=traj_done" not in sc.dut_log(), (
            "一次**失效**被记成了「任务走完」—— 现场会把一次降级读成一次成功")


# ───────────────────────────── 完成度到 1 ⟹ 任务收口 ─────────────────────────────

def test_完成度到_1_任务正常收口_end_reason_traj_done():
    """`progress ≥ 1.0` ⟹ `end_reason=traj_done` ⟹ 停发 + `AUTO.LAND` + 上锁后复位回 STANDBY。

    ★ 与上一条（`traj_timeout`）**成对**：两条出口的动作逐字相同（停发 + 降落），
      差别只在日志里怎么称呼它 ⟹ 所以这里必须验 `end_reason=traj_done` **在**、
      且 `degrade=traj_timeout` **不在**。只看"进了 LAND"是分不开这两条的。
    """
    with _mission(reference_fn=lambda _t: REF_X, duration_s=1.0) as (sc, traj, plant):
        _enter_track(sc, plant, traj)
        fly_until(sc, plant, lambda: sc.control_state() == "MANUAL/LAND", timeout_s=10.0,
                  message="完成度到 1.0 之后没有走正常出口（自动进 LAND）")

        assert "end_reason=traj_done" in sc.dut_log(), (
            "任务走完必须打出 `end_reason=traj_done`（工单要的可 grep 证据）")
        assert DEGRADE_TRAJ_TIMEOUT not in sc.dut_log(), (
            "正常飞完被记成了一次参考超时的**失效**")
        # ★ 回归钉：正常飞完时**不许**出现"参考断了"。这一条挡的是节点里那个真实的日志缺陷
        #   （`detect_traj_stream` 把"离开 TRACK"也算成 alive 由真变假 ⟹ 每次正常收口都打一条
        #   "轨迹参考断了"，而同一行里的年龄才 0.016 s）。它比 `degrade=` 那条更值钱：现场读到
        #   "飞完了 + 参考断了"会去查 traj，而真相是"我们不再需要它了"。
        assert TRAJ_REF_LOST not in sc.dut_log(), (
            "正常飞完却报了「轨迹参考断了」—— 离开 TRACK 被当成了参考流失效（见 "
            "`control_node.cpp::detect_traj_stream` 里的说明）")
        sc.wait_for(lambda: any(r.custom_mode == "AUTO.LAND" for r in sc.fake.mode_requests),
                    timeout_s=6.0, message="任务走完却没有请求 AUTO.LAND")
        frozen = len(sc.setpoints())
        sc.hold(1.0)
        assert len(sc.setpoints()) == frozen, (
            f"任务走完必须停止发 setpoint（冻结后又来了 {len(sc.setpoints()) - frozen} 帧）")

        # —— 任务收尾：上锁交给 PX4，节点自己复位回 STANDBY（不需要重启）——
        assert sc.fake.armed, "此刻还没到上锁的时候（PX4 会在它自己的着陆检测之后才上锁）"
        sc.fake.set_state(armed=False)
        sc.wait_for(lambda: sc.control_state() == "MANUAL/STANDBY", timeout_s=8.0,
                    message="落地上锁之后应当自动复位回 STANDBY")


# ───────────────────────────── 第二个任务 ─────────────────────────────

def test_第二个任务不会被上一轮留下的完成度_1_当场收口():
    """★★ 同一个节点进程里飞**第二个任务**：夹具的完成度此刻仍是上一轮的 `1.0`，而且**参考还在流**。

    ★ 缺陷现场：`traj` 跑完之后 `running_` 仍为真、完成度**保持 1.0**（`traj_node.cpp`），
      于是新一轮进 TRACK 的第一帧就会看到一个**新鲜**的 1.0 ⟹ 判据若只看"参考新鲜 + 完成度 ≥ 1"
      就会当场收口（飞机刚起飞、还没开始飞就降落）。要挡住它，只有"**亲眼见过它从 1 以下跑起来**"
      这一条闩锁（`fsm.hpp::track_started_`）。
    ★ 这条用例也是**唯一**能同时覆盖两件事的场景：任务的收口与再启动都在同一个进程里。
      正控放在末尾 —— 第二轮照样会自己飞完（否则"没当场收口"可能是"卡死不动"）。
    """
    with _mission(reference_fn=lambda _t: REF_X, duration_s=3.0) as (sc, traj, plant):
        # —— 任务 1：飞完、降落、上锁、复位 ——
        _enter_track(sc, plant, traj)
        _land_and_disarm(sc, plant)
        assert "end_reason=traj_done" in sc.dut_log(), "任务 1 没有正常收口（前提不成立）"

        # —— 任务 2：先松手回到低位 ——
        #   ★ §H.5.2：**只有确认过低位之后，"高位"才是一次边沿**。`_enter_track` 那一步
        #     已经松过手了，这里照旧再走一遍，是为了不让"低位"这个前提依赖于上一条用例的写法
        #     （少一次它，第二个任务的第一次按键就会一个边沿都不产生）。
        _release_key(sc)
        n_arm = len(sc.fake.arming_requests)
        plant2 = Plant(sc)   # 任务 2 从地面重新起飞（`_land_and_disarm` 把假飞机放回去了）
        _press_key(sc)
        fly_until(sc, plant2, lambda: sc.control_state() == "AUTO/WAIT_START", timeout_s=25.0,
                  message="第二个任务没有走到 WAIT_START")
        assert len(sc.fake.arming_requests) == n_arm + 1, "第二个任务应当新发一次解锁请求"
        assert traj.progress() == 1.0, (
            "夹具没能造出「上一轮的 1.0 还在流」这个场面（参考停下了？）⟹ 下面那条证明不了什么")

        _enter_track(sc, plant2, traj)
        # ★ 第二个任务要**重新播报**"参考已经来了"：闩锁在离开 TRACK 时复位（否则现场看第二个
        #   任务的日志时会以为"参考一直没来"—— 那条进流日志只有第一次才有）。
        assert sc.dut_log().count(TRAJ_REF_ARRIVED) == 2, (
            "第二个任务没有重新播报「轨迹参考已经来了」（离开 TRACK 时闩锁没复位）")
        fly(sc, plant2, 1.5)   # 远小于 duration_s = 3.0
        assert sc.control_state() == "AUTO/TRACK", (
            "第二个任务在 TRACK 里当场收口了 —— 上一轮留下的完成度 1.0 被当成了"
            "「这一轮飞完了」（这正是 `track_started_` 那道闩锁要挡的事）")
        assert traj.progress() < 1.0, (
            "DUT 重发的 /traj/start 没有让 traj 的时钟归零 —— 那第三个任务也一样会失败")

        # —— 正控：这一轮照样会自己飞完（不是"卡在 TRACK 里出不来"）——
        fly_until(sc, plant2, lambda: sc.control_state() == "MANUAL/LAND", timeout_s=10.0,
                  message="第二个任务没有自己飞完（闩锁挡过头了？）")


# ───────────────────────────── 完成度观测面 ─────────────────────────────

def test_control_traj_progress_逐字转发_traj_progress():
    """工单票根里的那条接口：`/control/traj_progress`（`Float32`）**逐字**转发 `/traj/progress`。

    ★ 为什么要有一条**独立**的转发话题（而不是让人直接看 `/traj/progress`）：
      后者是 `traj` 的域，而现场要的是"`control` 认为的进度"——两者不一致时，
      差异本身就是证据（比如 DUT 根本没接上那条话题）。
    ★ "逐字"是可判定的：收到的那一串值必须**逐个**出现在夹具发出去的那一串里
      （`Float32` 两侧同一精度 ⟹ 允许精确相等；任何再加工都会当场露馅）。
    """
    with _mission(reference_fn=lambda _t: REF_X, duration_s=10.0) as (sc, traj, plant):
        got = []
        sc.node.create_subscription(Float32, FWD_TOPIC, got.append, mavros_default_qos())
        # ★ 发现屏障：`/control/traj_progress` 的发布者从节点启动起就在（`control_state_topic`
        #   那一类都是常驻发布者）⟹ 等图里出现它即可。不看"有没有数据"——那要等进 TRACK，
        #   而下面一条断言恰恰要求"进 TRACK 之前这里什么都没有"。
        assert sc.poll(lambda: sc.node.count_publishers(FWD_TOPIC) >= 1, timeout_s=10.0), (
            f"DUT 没有提供 {FWD_TOPIC}（话题名或契约参数对不上？）")

        # ★ 进 TRACK 之前不该有**完成度**（traj 那时一帧都没发 ⟹ 没有东西可转发）。
        sc.hold(0.5)
        assert got == [], "traj 还没开始发完成度，这条转发话题上就已经有帧了"

        _enter_track(sc, plant, traj)
        fly_until(sc, plant, lambda: len(got) >= 20, timeout_s=8.0,
                  message="进了 TRACK 之后 /control/traj_progress 上没有帧（转发没接上？）")
        values = [m.data for m in got]

        assert len(values) >= 20, f"1 s 左右的 TRACK 只转发出 {len(values)} 帧（traj 是 50 Hz）"
        published = set(traj.progress_values)
        unknown = [v for v in values if v not in published]
        assert not unknown, (
            f"转发出来的值里有 traj **没发过**的：{unknown[:5]} —— 那不是逐字转发"
            f"（收到的前几个 {values[:5]}）")
        assert all(0.0 <= v <= 1.0 for v in values), "完成度必须落在 0–1"
        assert values == sorted(values), "完成度是单调不减的（traj 那边的时钟只往前走）"
        assert 0.0 < values[-1] < 1.0, (
            f"这才飞了一小段，完成度不该是边界值，实测 {values[-1]}")
