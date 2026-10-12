"""mavros 载体的**形状**与**启动参数**的断言（[`13`](../../../.scratch/phase4/issues/13-mavros-downlink-switch.md) §三①③）。

需要 `mavros_msgs`，**不需要** mavros 在跑、不需要飞机、不需要 init ROS（只 import 消息类）。

★ 为什么值得单独一组：这一组断言的东西全都属于"错了也不报错"那一类 ——

1. **消息形状**（字段名 / 常量值 / 服务字段）：写错一个字段名，ROS 那层会静默丢字段，
   飞控收到一条"看着合法"的消息；`type_mask` 误置一位（比如 `IGNORE_THRUST` = 64），
   飞控那一段 handler **直接不填推力**（`mavlink_receiver.cpp:1588`），现象是"消息发得出去、
   飞机没反应"。
2. **启动参数**：`--t-max` 必须显式给、`--break` 那两档改了名 —— 这两样都是操作者会照着
   票面**手敲**的东西。靠"我记得改过"守不住，靠断言才守得住。
3. **通路切换本身**（底部那两条）：散文里当然还会提到 px4_msgs（那是留痕），
   所以这里用 AST 只查**代码**：import 表与字符串字面量。
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from nmpc import offboard_smoke as smoke


# ───────────────────────────── 消息形状 ─────────────────────────────

def test_attitude_target_fields_are_the_ones_we_fill():
    """字段集**逐字**对 —— 节点里那几行赋值必须是**抄的**，不是**译的**。"""
    assert set(AttitudeTarget.get_fields_and_field_types()) == {
        "header", "type_mask", "orientation", "body_rate", "thrust"}


def test_our_type_mask_ignores_only_the_orientation():
    """★ `type_mask` 只置 `IGNORE_ATTITUDE`（= 128）：角速率三轴**被用**、**推力不被忽略**。

    ⛔ 误置 `IGNORE_THRUST`（64）⟹ 飞控那一段不填推力；误置某个 `IGNORE_*_RATE` ⟹
    对应那一轴退回实测角速率。两种都不报错，只表现为"某一轴没反应"。
    """
    mask = smoke.TYPE_MASK_IGNORE_ATTITUDE
    assert mask == AttitudeTarget.IGNORE_ATTITUDE == 128
    for bit in (AttitudeTarget.IGNORE_ROLL_RATE, AttitudeTarget.IGNORE_PITCH_RATE,
                AttitudeTarget.IGNORE_YAW_RATE, AttitudeTarget.IGNORE_THRUST):
        assert mask & bit == 0, f"type_mask 多置了 {bit}"


def test_offboard_mode_name_comes_from_the_message_constant():
    """模式名不许在节点里手写字符串：`State.MODE_PX4_OFFBOARD` 才是那个值。"""
    assert smoke.OFFBOARD_MODE == State.MODE_PX4_OFFBOARD == "OFFBOARD"


def test_state_has_the_fields_we_gate_on():
    """闸门只用 `/mavros/state` 里**真有**的字段。

    ★ 这一条是"旧守卫没有对应物"那件事的护栏：旧版等的是 `pre_flight_checks_pass`，
    `/mavros/state` 里**没有**它（列出来看）—— 别以为它还在。
    """
    fields = set(State.get_fields_and_field_types())
    assert {"armed", "mode", "connected", "system_status"} <= fields
    assert "pre_flight_checks_pass" not in fields


def test_arm_and_mode_are_services_with_the_fields_we_set():
    """解锁 / 切模式在这个载体上是**服务**（旧载体是往 `/fmu/in/vehicle_command` 发消息、无应答）。"""
    assert set(CommandBool.Request.get_fields_and_field_types()) == {"value"}
    assert set(SetMode.Request.get_fields_and_field_types()) == {"base_mode", "custom_mode"}
    # ★ 应答字段也要看一眼：`success`/`result` 是"被拒也读得出来"的那一半（旧载体**没有应答**）
    assert {"success", "result"} <= set(CommandBool.Response.get_fields_and_field_types())
    # ★ `mode_sent` 只说明"请求发出去了"，**不等于**飞控接受了 ⟹ 进 OFFBOARD 只认 `/mavros/state`
    assert "mode_sent" in SetMode.Response.get_fields_and_field_types()


# ───────────────────────────── 通路切换（AST，只看代码） ─────────────────────────────

def _tree() -> ast.Module:
    return ast.parse(Path(smoke.__file__).read_text(encoding="utf-8"))


def _imported_modules(tree) -> set:
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            mods.add(node.module or "")
    return mods


def _string_literals(tree) -> set:
    return {n.value for n in ast.walk(tree)
            if isinstance(n, ast.Constant) and isinstance(n.value, str)}


def test_the_px4_msgs_downlink_is_gone_from_the_code():
    """★ 通路切换：代码里**不许**再出现 px4_msgs 的 import 或 `/fmu/in/*` 主题。

    ⚠️ 判据刻意落在 AST 上、不是"整份文件 grep 不到 px4_msgs"：模块 docstring 里**必须**
    留着旧载体的名字（那是留痕，改通路这件事得说得出来）⟹ 整份 grep 会假红。
    所以这里问的是"**代码**里还有没有"，散文随便提。
    """
    tree = _tree()
    assert not [m for m in _imported_modules(tree) if m.split(".")[0] == "px4_msgs"], \
        "还在 import px4_msgs —— 通路没换干净"
    stray = [t for t in _string_literals(tree) if t.startswith("/fmu/in/")]
    assert not stray, f"代码里还有 uXRCE-DDS 主题：{stray}"


def test_the_four_topics_are_the_mavros_ones():
    """四条端点逐字：发布 setpoint、订 state、两个服务 —— 少一条就是少一段通路。"""
    literals = _string_literals(_tree())
    for topic in ("/mavros/setpoint_raw/attitude", "/mavros/state",
                  "/mavros/cmd/arming", "/mavros/set_mode"):
        assert topic in literals, f"找不到 {topic}"


# ───────────────────────────── 启动参数 ─────────────────────────────

def _args(*argv):
    return smoke.build_parser().parse_args(list(argv))


#: 正确的那一对（`m·g / hover_thrust`，0.75 kg / 0.1589）—— 大多数用例只需它通过
GOOD = ("--mass", "0.75", "--t-max", "46.3027")


def test_t_max_is_mandatory():
    """★★ `--t-max` **没有默认值**：不给就拒绝启动（2026-10-12 操作者裁定）。

    ★ 为什么不给个"看着合理"的默认：它的正确值随 `hover_thrust` 变，而 `hover_thrust`
    正是未结票 02 要回填的量 ⟹ 任何默认值都会在某一天**悄悄**变成错的。
    """
    with pytest.raises(SystemExit):
        _args("--mass", "0.75")


def test_config_has_no_default_t_max_either():
    """同一个意思在数据结构层也成立 —— `Config()` 少一个必填参数就构造不出来。"""
    with pytest.raises(TypeError):
        smoke.Config()


def test_the_old_default_is_rejected_before_the_node_is_built():
    """把 `--t-max` 填回旧默认 11.8（= `f_max`）⟹ 建节点**之前**就拒，并把正确值说出来。"""
    args = _args("--mass", "0.75", "--t-max", "11.8")
    with pytest.raises(SystemExit) as exc:
        smoke.validate_args(args)
    assert "46.3027" in str(exc.value)


def test_the_correct_pair_passes_validation():
    smoke.validate_args(_args(*GOOD))


@pytest.mark.parametrize("t_max", ["0", "-46.3", "nan"])
def test_meaningless_t_max_is_rejected(t_max):
    """0 / 负 / 非有限：同样在启动前拒（归一化常数没有意义的那一类）。"""
    with pytest.raises(SystemExit):
        smoke.validate_args(_args("--mass", "0.75", "--t-max", t_max))


def test_timestamp_switches_are_gone():
    """★ `--timestamp-unit` / `--timestamp-shift-us` **删掉**了，理由是它们在新载体上**没有对应物**
    （飞控那一段 handler 一次都没读 `time_boot_ms`，见 `sign-map.md` §2.3.1）。

    ⛔ 留着比删掉更糟：操作者会以为"时间戳这一档也测过了"，而这一场里那个开关**再也取不到
    第二个值**（[判据全绿也可能是机理反了] 里的"仪器没给那个量变过"那一族）。
    """
    for gone in ("--timestamp-unit", "--timestamp-shift-us"):
        with pytest.raises(SystemExit):
            _args(*GOOD, gone, "ms")


def test_break_names_say_what_they_now_test():
    """★ `pitch-sign` / `yaw-sign` → `pitch-preconvert` / `yaw-preconvert`。

    在新载体上那两档验的是 **N2**（我们自己又转了一次 FLU→FRD ⟹ 转两次 = 不转，
    飞控最终收到的分毫不差），**不是**"符号写反"。
    ⛔ 旧名字必须**不再被接受**：同一个名字指另一件事，正是操作者会误判的地方。
    """
    assert "pitch-preconvert" in smoke.BREAKS and "yaw-preconvert" in smoke.BREAKS
    assert "pitch-sign" not in smoke.BREAKS and "yaw-sign" not in smoke.BREAKS
    for old in ("pitch-sign", "yaw-sign"):
        with pytest.raises(SystemExit):
            _args(*GOOD, "--break", old)
    # 三档"真的写错一项"的注入仍在（`roll-sign` 在两条路上都是"符号写反"，名字不变）
    assert {"none", "thrust-sign", "thrust-scale", "roll-sign"} <= set(smoke.BREAKS)


def test_qos_default_follows_the_state_publisher():
    """闸门源 `/mavros/state` 由 `StateQoS`（reliable）发布 ⟹ 默认跟着它。

    ★ 两个取值**都收得到**（发布者可靠、订阅者尽力也收得到）⟹ 这一档在新载体上
    不再能造出"零回调"，`--qos best_effort` 只用于核对这条论断。
    """
    assert _args(*GOOD).qos == "reliable"
    assert smoke.Config.qos == "reliable"
    # 两个 QoSProfile 的差别**只在**可靠性 —— 深度/持久性那两项与这一档无关，别顺手改
    assert smoke.QOS_STATE_RELIABLE.reliability == ReliabilityPolicy.RELIABLE
    assert smoke.QOS_STATE_BEST_EFFORT.reliability == ReliabilityPolicy.BEST_EFFORT
    for q in (smoke.QOS_STATE_RELIABLE, smoke.QOS_STATE_BEST_EFFORT):
        assert q.depth == 1 and q.durability == DurabilityPolicy.VOLATILE
