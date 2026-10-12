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
from nmpc.node_core import STATUS_SOURCE_PERIOD_S
from nmpc.params import Z0Params
from nmpc.preflight_check import (
    FCU_FWDEXTSP,
    Check,
    _channel_identifiers,
    _param_table,
    build_node_params,
    check_downlink_prereqs,
    check_normalization,
    check_params,
    check_fallback,
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
    # ☠️ 这里原有 `("heartbeat_hz", 1.0, "心跳 ≥2 Hz")` —— 心跳那条流与它的参数随
    #    ticket 13 §二④（裁定 (a) 收成单频）一起删除，那条检查与它的负控一起走。
    ("rate_hz", 0.0, "定时器频率"),
    ("tau_s", 0.5, "外推 τ 在合理区间"),
    #: ★ 负 τ 与"太大"是**两个**方向，都要拦：反向外推 = 拿"还没走到"的状态喂控制器。
    ("tau_s", -0.01, "外推 τ 在合理区间"),
    #: ★ 端点 `0.2` 也喂进去：它与 `mpc_node._validate` 是**同一个区间**，
    #: 差一个端点就会"自检红、节点绿"（本票 2026-10-09 踩过一次），所以要钉住。
    ("tau_s", 0.2, "外推 τ 在合理区间"),
    ("warmup_cycles", 0, "热身拍数 ≥1"),
    #: ☠️ 这一条的**判据本身换过**（ticket 13 §二③）：名字从「状态时限 ≥ 上游一个周期」
    #:   换成「状态时限 ≥ 闸门源 2 个周期」，理由见下面对应的那条用例。
    ("status_timeout_s", 0.2, "状态时限 ≥ 闸门源 2 个周期"),
    ("stop_on_armed_drop", False, "两条安全停机都开着"),
    ("stop_on_nav_state_exit", False, "两条安全停机都开着"),
    ("mass", 0.0, "质量为正"),
])
def test_坏参数会被点名(field, value, expect):
    p = dataclasses.replace(NodeParams(), **{field: value})
    assert expect in _failing(check_params(p)), \
        f"{field}={value} 应该让「{expect}」红，实际红的是 {_failing(check_params(p))}"


def test_tau_取_0_是首飞冻结值_自检要放行():
    """★ 正控：`τ = 0`（**不补偿**）是首飞的**冻结值**（ticket `phase4/05`，D9 那一档）。

    ★★ 它不是"漏填的默认值"，而是"**过补偿比不补偿更危险**"那条纪律的落点
      （ticket `phase4/03` 的量测结论：实飞起始值取 0）。
      ⟹ 装桨前那道自检**必须**让这一档通过 —— 否则合法配置会被自己的闸门拦下，
        而"拦下了"在台架上的表现与"参数写错了"一模一样。
      ⚠️ 上界那条（`0.5` 会红）不能因为这次放开而松掉：两个方向都要有负控。
    """
    p = dataclasses.replace(NodeParams(), tau_s=0.0)
    assert _failing(check_params(p)) == [], _failing(check_params(p))


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


# ───────────────────────────── 负控：兜底与故障注入（ticket 07 / M5）─────────────────────────────

def test_兜底那组默认全绿():
    """正控：负控要能说明问题，前提是真实参数下这一组本来就绿。"""
    checks = check_fallback(NodeParams())
    assert all(c.ok for c in checks), _failing(checks)
    assert len(checks) == 3, f"这一组应当是 3 条，实际 {len(checks)}"


def test_参数表与_NodeParams_的默认值不许漂移():
    """★★ `_PARAMS`（节点**真正声明**的默认值）与 `NodeParams`（测试与自检读的那一份）
    是同一个参数面的两份写法，而**没有任何东西**看着它们一致（源码里那句
    "两处一起改"只是一句注释）。漂了不会报错：节点按 `_PARAMS` 飞、测试按 `NodeParams` 断言
    ⟹ 这正是本仓库最怕的那种"假绿"。本条是它的守卫（mypy/pytest 都抓不到这个形状）。
    """
    p = NodeParams()
    by_name = {f.name: f for f in dataclasses.fields(p)}
    table = dict((n, (d, t)) for n, d, t in _param_table())
    composed = {"hold_offset_x": 0, "hold_offset_y": 1, "hold_offset_z": 2}
    for name, (default, typ) in table.items():
        if name in composed:                       # 三个参数合成一个字段，单独取
            got = p.hold_offset[composed[name]]
        else:
            assert name in by_name, f"`_PARAMS` 里的 {name} 在 `NodeParams` 里没有对应字段"
            got = getattr(p, name)
        assert got == default and isinstance(got, typ), \
            f"{name}：`_PARAMS` 说 {default!r}（{typ.__name__}），`NodeParams` 说 {got!r}"
    # 反方向：`NodeParams` 的字段要么在参数表里，要么在"由多个参数合成"的名单里
    derived = {"hold_offset"}          # = hold_offset_x/y/z 三个参数
    extra = set(by_name) - set(table) - derived
    assert not extra, f"`NodeParams` 里有参数表之外的字段：{sorted(extra)}（要么补进 `_PARAMS`，要么列进 derived）"


def test_NodeParams_到配置对象的接线不许串位():
    """★ `control_config()` / `z0_params()` 是"参数 → 判据"的**接线板**（节点与自检各读一遍）。

    串位（比如把位移上限与转角上限对调）在台架上的表现与正确接线**一模一样**：
    同一条判据、同一个原因词（`pose_jump`），只有细节里的数值不同 ——
    也就是说**只有这条用例**看得见它。顺带钉住"三个数值在 `NodeParams` 与 `StopConfig`
    里各写一份"这件事（第三份在 `_PARAMS`，由上面那条对账管）。
    """
    p = NodeParams()
    c, z = p.control_config(), p.z0_params()
    assert (c.stop.mocap_jump_speed_max, c.stop.mocap_jump_rate_max) == \
        (p.mocap_jump_speed_max, p.mocap_jump_rate_max), "位姿跳变的两条上限串位了"
    assert c.stop.mocap_stamp_regress_s == p.mocap_stamp_regress_s
    assert c.fallback_hold_frames == p.fallback_hold_frames
    assert c.tau_s == p.tau_s and c.hold_offset == tuple(p.hold_offset)
    assert (c.stop.mocap_timeout_s, c.stop.status_timeout_s) == \
        (p.mocap_timeout_s, p.status_timeout_s)
    assert (z.mass, z.hover_thrust) == (p.mass, p.hover_thrust)


def test_兜底保持拍数为负会被抓到():
    p = dataclasses.replace(NodeParams(), fallback_hold_frames=-1)
    assert "兜底保持拍数 ≥0" in _failing(check_fallback(p)), \
        "负的保持拍数没被拦下 —— 它会让 `FallbackLadder` 的档位判断失去意义"


def test_位姿合理性阈值收得比真机动还紧会被抓到():
    """★ 这条负控指向的是**无故停发**：阈值比真机动还紧 ⟹ 一次正常机动就被读成跳变。

    两个方向各喂一次（位移 / 转角），因为它们拦的是两种不同的误判。
    """
    slow = dataclasses.replace(NodeParams(), mocap_jump_speed_max=2.0)
    assert "位姿合理性阈值比真机动宽" in _failing(check_fallback(slow)), \
        "位移上限 2 m/s（比飞控的 12 m/s 还小）没被拦下"

    tight = dataclasses.replace(NodeParams(), mocap_jump_rate_max=1.0)
    assert "位姿合理性阈值比真机动宽" in _failing(check_fallback(tight)), \
        "转角上限 1 rad/s（比模型的 6.28 rad/s 还小）没被拦下"


def test_故障注入开着会被拦在装桨之前():
    """★ 这一档一开，飞机收到的**每一帧**都是兜底帧 —— 它只该在台架/SITL 上出现。"""
    p = dataclasses.replace(NodeParams(), inject_solve_fail_after_frames=200)
    failed = _failing(check_fallback(p))
    assert "故障注入关着" in failed, "开着故障注入还能过自检 ⟹ 装桨前那道闸门形同虚设"
    assert "装桨前必须设回 0" in check_fallback(p)[2].detail, "细节里要写清怎么处置"


# ───────────── 下行前置：两个"整条下行静默死掉"的开关（ticket 13 §二⑤bis/⑥）─────────────

def _tmp_repo_with_snapshot(tmp_path: Path, body: str) -> Path:
    """造一个只带飞控参数快照的假仓库根（本组只从仓库里读这一个文件）。"""
    (tmp_path / "docs" / "assets").mkdir(parents=True)
    (tmp_path / "docs" / "assets" / "z0.params").write_text(body, encoding="utf-8")
    return tmp_path


def _tmp_mavros_config(tmp_path: Path, body: str, launch: bool = True) -> Path:
    tmp_path.mkdir(parents=True, exist_ok=True)
    p = tmp_path / "px4_config.yaml"
    p.write_text(body, encoding="utf-8")
    if launch:
        (tmp_path / "px4.launch").write_text("config_yaml: px4_config.yaml\n", encoding="utf-8")
    return p


def test_下行前置那组默认全绿():
    """正控：真实仓库 + 装好的 mavros 下这一组本来就绿，负控才说明问题。"""
    checks = check_downlink_prereqs(REPO)
    assert all(c.ok for c in checks), _failing(checks)
    assert len(checks) == 3, f"这一组应当是 3 条，实际 {len(checks)}"


def test_飞控没开_MAV_FWDEXTSP_要被抓在装桨之前(tmp_path: Path):
    """★ `MAV_FWDEXTSP = 0` ⟹ `SET_ATTITUDE_TARGET` 整条被飞控丢掉，而**飞控侧没有日志**。

    这个数**不在我们仓库里**，所以负控只能造一份快照来喂 —— 造的时候刻意走**按名字查**那条路
    （把这一行挪到别处、再夹几行别的参数），顺带钉住"不按行号查"这件事。
    """
    for value, ok in (("1", True), ("0", False)):
        repo = _tmp_repo_with_snapshot(
            tmp_path / f"v{value}", "# 头\n1\t1\tMAV_2_CONFIG\t0\t6\n"
                                   f"1\t1\t{FCU_FWDEXTSP}\t{value}\t6\n1\t1\tMAV_HASH_CHK_EN\t1\t6\n")
        checks = check_downlink_prereqs(repo)
        got = [c for c in checks if c.name.startswith(FCU_FWDEXTSP)]
        assert got and got[0].ok is ok, f"MAV_FWDEXTSP={value} 的判定反了：{got[0] if got else None}"
    repo = _tmp_repo_with_snapshot(tmp_path / "zero", "# 头\n1\t1\tMAV_2_CONFIG\t0\t6\n")
    got = [c for c in check_downlink_prereqs(repo) if c.name.startswith(FCU_FWDEXTSP)]
    assert got and not got[0].ok, "快照里根本没有这个参数，却判过了（『找不到』不是『通过』）"


def test_快照缺失算失败而不是跳过(tmp_path: Path):
    """★ 与"仓库根找不到"同一条纪律：这一条守的是静默掉帧，**没有跳过这一档**。"""
    got = [c for c in check_downlink_prereqs(tmp_path) if c.name.startswith(FCU_FWDEXTSP)]
    assert got and not got[0].ok, "快照找不到却判过 ⟹ 这条检查在没有证据时给了绿灯"
    assert "验不了" in got[0].detail


def test_thrust_scaling_不是_1_要被抓在装桨之前(tmp_path: Path):
    """三种错法各喂一次（ticket 13 §二② 新发现的硬前置）：

    · 缺这一项 ⟹ mavros 收到 `thrust != 0` 的帧**整帧丢**（角速率也一起丢）；
    · `0` ⟹ 帧不丢但推力恒 0（掉高度）；
    · `1.0` ⟹ 才是我们要的口径。
    """
    cases = {"未设": ("/**/setpoint_raw:\n  ros__parameters:\n    position_sent: 0.0\n", False),
             "零": ("/**/setpoint_raw:\n  ros__parameters:\n    thrust_scaling: 0.0\n", False),
             "一": ("/**/setpoint_raw:\n  ros__parameters:\n    thrust_scaling: 1.0\n", True),
             "别的节点": ("/**/setpoint_position:\n  ros__parameters:\n    thrust_scaling: 1.0\n", False)}
    for label, (body, ok) in cases.items():
        cfg = _tmp_mavros_config(tmp_path / label, body)
        got = [c for c in check_downlink_prereqs(REPO, cfg) if c.name.startswith("thrust_scaling")]
        assert got and got[0].ok is ok, f"{label} 的判定反了：{got[0] if got else None}"


def test_参数文件存在但没被_launch_加载也要说一声(tmp_path: Path):
    """★ "文件里写着 1.0" 与 "mavros 真的用上了这一份" 是两件事（`px4.launch:13` → `node.launch:24`）。"""
    cfg = _tmp_mavros_config(tmp_path, "/**/setpoint_raw:\n  ros__parameters:\n    thrust_scaling: 1.0\n",
                             launch=False)
    got = [c for c in check_downlink_prereqs(REPO, cfg) if c.name == "那份 px4_config.yaml 确实被加载"]
    assert got and not got[0].ok, "launch 没加载它却判过 ⟹ 那个值可能根本没进 mavros"


def test_状态时限收得比闸门源周期还短会被抓到():
    """★★ 这条负控钉的是 ticket 13 §二③ 那个**恒真**判据（旧值 0.75 vs 1 Hz 的源）。

    ⛔ 写成 `>= 0.5`（旧判据）时它**照样绿** —— 那正是这条检查换过的原因：
    旧判据在真机上"永远通过、而确实错了"。
    """
    for value, ok in ((2.5, True), (0.75, False), (1.5, False)):
        p = dataclasses.replace(NodeParams(), status_timeout_s=value)
        got = [c for c in check_params(p) if c.name == "状态时限 ≥ 闸门源 2 个周期"]
        assert got and got[0].ok is ok, f"status_timeout_s={value} 的判定反了"
    # ★ 边界：正好等于周期 ⟹ 红（`age > 阈值` 在 1 Hz 下会几乎每拍都成立）
    p = dataclasses.replace(NodeParams(), status_timeout_s=STATUS_SOURCE_PERIOD_S)
    got = [c for c in check_params(p) if c.name == "状态时限 ≥ 闸门源 2 个周期"]
    assert got and not got[0].ok, "阈值恰好等于源周期时不该放行"
    # ★★ `1.5` 那一档（上面 `(1.5, False)`）钉的是**门槛取 2 个周期**这件事：
    #    判据若写成 `> period`（只要求"比一个周期大"），1.5 会放行 —— 而它意味着
    #    源正常丢一拍就被判成丢状态。它是这条用例里**唯一**能区分 `> period`
    #    与 `>= 2*period` 的那个取值（0.75 / 1.0 在两个判据下都红 ⟹ 有它才不是"随便写都绿"）。


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
