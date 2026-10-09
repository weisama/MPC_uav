#!/usr/bin/env python3
"""从**串口直接**读回飞控此刻的参数，并与一份快照（QGC 导出）逐条对账。

服务于 ticket [`../../.scratch/phase4/issues/05-first-flight-config-freeze.md`](../../.scratch/phase4/issues/05-first-flight-config-freeze.md)
的 §5 ⑤（"起飞前 dump 一次飞控参数与 `docs/assets/z0.params` 对账"）、以及 §F1/§F2/§F5
那三个"**此刻**飞控里还是不是它"的框。

★ **只读**：只发 `PARAM_REQUEST_LIST`，把飞控回上来的 `PARAM_VALUE` 收全。
  **不发** `PARAM_SET`、**不发**心跳驱动的任何命令、**不碰** DDS。读完就关串口。
  这个文件里**没有**写参数的代码路径 —— 要写参数走 `params-changes.md` §I.1 的四步纪律
  （先存旧值 → 写 → 读回 → 再 dump 一次），那是**人的动作**。

链路（本仓库定案，第一手）：`/dev/ttyTHS1:921600`。依据：
  · `src/bringup/launch/mavros.launch.py` 的 `fcu_url` 默认值；
  · `docs/PROJECT.md:375-376`：`MAV_1_CONFIG = 102`（TELEM2）/ `MAV_1_MODE = 2`（Onboard）/ `SER_TEL2_BAUD = 921600`；
  · `docs/reference/params-analysis.md` §3.8：`UXRCE_DDS_CFG = 0` ⟹ 这口走的是 **MAVLink**，不是 uXRCE-DDS。
⟹ 所以这里用 pymavlink 直连**不需要** mavros 在跑（那条 `ros2 param dump /mavros/param`
是文档里的另一条路，见 `docs/PROJECT.md:406-407`）。

用法：

    # 0) 自检（不碰串口、不需要飞控通电）
    python3 tools/fcu-params/params_from_serial.py --self-test

    # 1) 读回 + 与快照对账（默认端口/波特率就是 z0 那条）
    python3 tools/fcu-params/params_from_serial.py --compare docs/assets/z0.params

    # 2) 只想看某几条
    python3 tools/fcu-params/params_from_serial.py --name NAV_RCL_ACT,MC_BAT_SCALE_EN,MPC_THR_HOVER

    # 3) 留一份全量快照（对账的输入，也是"此刻"的第一手证据）
    python3 tools/fcu-params/params_from_serial.py --yaml /tmp/fcu_params_2026-10-09.yaml
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

from params_core import (  # noqa: E402  （同目录，先插路径再 import）
    MAV_PARAM_TYPE_INT32 as INT32, MAV_PARAM_TYPE_REAL32 as REAL32,
    MAV_PARAM_TYPE_UINT32 as UINT32,
    ExportParseError, Param, collect_params, decode_export, decode_param_value, diff,
    parse_qgc_export, same_value,
)

#: 本仓库定案的那条链路。命令行可以覆盖，但默认值就是它。
DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 921600

#: 拉全量（z0 上是 ~900 条）的默认上限。读秒级就完事，给足冗余是为了链路慢时不误判。
DEFAULT_TIMEOUT_S = 30.0


def read_export(path) -> Tuple[Dict[str, Param], List[str]]:
    """读一份 QGC 导出（**按字节读**再逐行解码，见 `params_core.decode_export`）。"""
    return parse_qgc_export(decode_export(Path(path).read_bytes()))


def pull_params(port: str, baud: int, timeout_s: float,
                quiet: bool = False) -> Tuple[Dict[str, Param], Dict[str, object]]:
    """开串口、要一遍全量参数、读完就关。返回 `(params, info)`。

    ⚠️ 这里是**唯一**碰硬件的地方，且只有两件事：等 `HEARTBEAT`、发 `PARAM_REQUEST_LIST`。

    ★★ 三条**容易静默错**的处理，都在这里：

    ① **按 `param_type` 解值**（`decode_param_value`）。不这么做，整型参数会以 float32 位型现身，
       对账时变成一两百条假差异 —— 2026-10-09 真踩到过，见 `params_core.decode_param_value` 的注释。
    ② **把伪参数挑出去**。PX4 回全量时**第一条永远是 `_HASH_CHECK`**（`send_one()`：
       `if (_send_all_index == PARAM_HASH) {... param_index = -1; param_type = UINT32; memcpy(&hash)}`）
       —— 它不是参数、QGC 导出里也没有。混进来就会在"飞控多出"那一栏冒充一条新增参数。
    ③ **完整性按 `param_index` 数，不按名字数**。名字会重（重了就说明**重传过**），
       按 `len(dict)` 数两者都看不出来；按索引数才能同时报出"缺哪几个索引"与"哪几条重了"。
    """
    from pymavlink import mavutil  # 延迟 import：`--self-test` 不需要 pymavlink（也就不用串口权限）

    conn = mavutil.mavlink_connection(port, baud=baud)
    try:
        hb = conn.wait_heartbeat(timeout=timeout_s)
        if hb is None:
            raise RuntimeError(
                f"{timeout_s:.0f} s 内没收到 HEARTBEAT（{port}@{baud}）—— "
                f"按顺序查：① 飞控上电了没；② `fcu_url` 那条线是不是 TELEM2 ⟹ {port}；"
                f"③ 波特率是不是 921600（`SER_TEL2_BAUD`）；④ 端口是不是被 mavros / QGC 占着")
        info = {
            "target_system": conn.target_system,
            "target_component": conn.target_component,
            "autopilot": getattr(hb, "autopilot", None),
            "type": getattr(hb, "type", None),
        }
        conn.mav.param_request_list_send(conn.target_system, conn.target_component)

        raw = []          # `(名字, 线上值, param_type, param_index, param_count)` —— 拆包，交给纯函数
        got = 0
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            pid = msg.param_id
            if isinstance(pid, bytes):
                pid = pid.decode("ascii", errors="replace")
            pid = pid.split("\x00")[0].strip()
            if not pid:
                continue
            count = int(msg.param_count)
            idx = int(msg.param_index)
            raw.append((pid, msg.param_value, int(msg.param_type), idx, count))
            if idx >= 0:
                got += 1
            if not quiet and got and got % 50 == 0:
                print(f"\r  收到 {got}/{count} 条…", end="", flush=True)
            if count and got >= count:
                break
        c = collect_params(raw)
        if not quiet:
            print(f"\r  收到 {len(c.params)}/{c.expected} 条。      ")
        if not c.params:
            raise RuntimeError("一条 PARAM_VALUE 都没收到（HEARTBEAT 到了、参数没来）")
        if c.duplicates:
            # ⚠️ 重传**必须说出来**：按名字去重的 `dict` 会把它悄悄吃掉，
            #    看着像"没收全"，其实是同一条来了两遍 —— 两种毛病得分开。
            print(f"⚠️ 有重传：{c.duplicates} —— 同一条收到了不止一次")
        if not c.complete:
            # ⚠️ 收不全**必须说出来**：少了的那几条在对账里会变成"only_reference"，
            #    看上去像"飞控把这参数删了"，其实是这次没读全。
            print(f"⚠️ 没收全：{len(c.params)}/{c.expected} —— 缺的 param_index 是 "
                  f"{c.missing_indices}。对账里「快照有、此刻没有」那几条可能只是"
                  f"**这次没读到**，不是被删了")
        info.update(expected=c.expected, pseudo=c.pseudo, duplicates=c.duplicates,
                    missing_indices=c.missing_indices)
        return c.params, info
    finally:
        conn.close()


def fmt_params_yaml(params: Dict[str, Param]) -> str:
    """全量快照的落盘格式：`名字: 值`（名字有序，便于 diff 两份快照）。

    ★ 这里的值已经是**真值**（`decode_param_value` 解过）：整型参数落成 `1` / `-1` 这样的整数，
      不再是一串 `1.4e-45`。PX4 只发 `INT32` / `REAL32` 两档 ⟹ "看值是不是整数" 就能反推类型，
      所以不必再单独存一列（伪参数 `_HASH_CHECK` 已经不在这个文件里）。
    """
    lines = ["# 由 tools/fcu-params/params_from_serial.py 从串口**只读**读回",
             "# 值已按 MAVLink 的 param_type 解过（整型是整数、不是 float32 位型）",
             f"# 条数: {len(params)}"]
    for name in sorted(params):
        lines.append(f"{name}: {params[name].text}")
    return "\n".join(lines) + "\n"


def print_report(cur: Dict[str, Param], ref: Optional[Dict[str, Param]],
                 names: List[str], info: Optional[Dict[str, object]] = None) -> int:
    """打印对账结果。返回退出码（0 = 对上；1 = 有差异）。"""
    print(f"此刻从飞控读回 **{len(cur)}** 条参数。")
    if info and info.get("pseudo"):
        # ★ 不是参数、也不该出现在对账里。单独说一句，免得下一个人再被它骗一次。
        print(f"（另有伪参数 {', '.join(info['pseudo'])} —— `param_index = -1` 的参数哈希，"
              f"PX4 回全量时**第一条永远是它**，QGC 导出会滤掉；已从对账里剔除）")
    if names:
        print()
        print("指定的几条：")
        for n in names:
            p = cur.get(n)
            now = p.text if p else "（这次没读到）"
            old = f"  快照 {ref[n].text}" if (ref and n in ref) else ""
            print(f"  {n:24s} = {now:24s}{old}")
    if ref is None:
        return 0
    d = diff(cur, ref)
    print()
    print(f"对账（此刻 vs 快照，共 {len(ref)} 条）：相同 {d.same} 条，"
          f"不同 {len(d.changed)} 条，飞控多出 {len(d.only_current)} 条，"
          f"快照多出 {len(d.only_reference)} 条")
    if d.clean:
        print("✅ 逐条一致。")
        return 0
    if d.changed:
        print()
        print("⚠️ 值不同（名字 / 快照 / 此刻）：")
        for name, old, now in d.changed:
            print(f"  {name:24s} {old:24s} -> {now}")
    if d.only_current:
        print()
        print("⚠️ 飞控里有、快照里没有（新加 / 上次导出后改过名？）：")
        print("   " + " ".join(d.only_current))
    if d.only_reference:
        print()
        print("⚠️ 快照里有、飞控里没有（被删 / 改名 / **这次没读全**）：")
        print("   " + " ".join(d.only_reference))
    return 1


def self_test(export: str = "docs/assets/z0.params") -> int:
    """离线自检：**不碰串口**。用仓库里那份导出做解析 + 对账。

    三段：
      ① 解析：条数 > 0、若干条**已知值**对得上（`NAV_RCL_ACT = 1` 等，第一手）；
      ② 「自己对自己」对账必须是**干净的**（否则容差/解析有一处是错的）；
      ③ **变异**：把一条值改掉 ⟹ 必须**恰好**报出那一条（报 0 条 = 没鉴别力，
         报一批 = 容差或解析在把表示误差当改动）。
    """
    ref, skipped = read_export(export)
    print(f"① 解析 {export}：{len(ref)} 条；`skipped` {len(skipped)} 行"
          f"（非空/非注释但解不出参数名）")
    if skipped:
        for s in skipped[:3]:
            print(f"   skipped 例：{s!r}")
    known = {"NAV_RCL_ACT": 1.0, "MC_BAT_SCALE_EN": 0.0, "MPC_THR_HOVER": 0.5,
             "MPC_XY_VEL_MAX": 2.0, "HTE_THR_RANGE": 0.2}
    bad = []
    for name, want in known.items():
        got = ref[name].value if name in ref else None
        # ★ 用**对账那同一把尺**（`same_value`），不是 `==`：这几位差是 float32 的表示误差
        #   （`HTE_THR_RANGE` 解出 `0.20000000298023224`）。
        if not same_value(got, want):
            bad.append(f"{name}: 期望 {want}，解出 {got}")
    if bad:
        print("❌ 已知值对不上：" + "; ".join(bad))
        return 1
    print(f"   ✅ 5 条已知值对得上（{', '.join(sorted(known))}）")

    d = diff(ref, ref)
    print(f"② 自己对自己：相同 {d.same}，不同 {len(d.changed)}，"
          f"多出 {len(d.only_current)}/{len(d.only_reference)}")
    if not d.clean or d.same != len(ref):
        print("❌ 「自己对自己」不是干净的 —— 容差或解析有问题")
        return 1
    print("   ✅ 干净")

    # ③ 变异：改一条**真的**改动（0.5 -> 0.1589，正是本票要写的那一条）
    mutated = dict(ref)
    p = ref["MPC_THR_HOVER"]
    mutated["MPC_THR_HOVER"] = Param(name=p.name, text="0.1589", value=0.1589)
    d2 = diff(mutated, ref)
    print(f"③ 变异（把 MPC_THR_HOVER 改成 0.1589）：报出 {len(d2.changed)} 条")
    if [c[0] for c in d2.changed] != ["MPC_THR_HOVER"]:
        print(f"❌ 期望**恰好**报出 MPC_THR_HOVER，实际 {d2.changed}")
        return 1
    # ③′ 表示误差**不许**被当成改动：float32 往返（0.45 的两种写法）
    p2 = ref.get("THR_MDL_FAC")
    if p2 is not None and p2.value is not None:
        near = dict(ref)
        near["THR_MDL_FAC"] = Param(name=p2.name, text="0.45", value=0.45)
        d3 = diff(near, ref)
        if d3.changed:
            print(f"❌ float32 的两种写法被判成了改动：{d3.changed}")
            return 1
        print("   ✅ float32 的表示误差没有被当成改动（`%r` vs `0.45`）" % p2.value)

    # ④ ☠️ 整型参数的**位型**解码 —— 2026-10-09 真踩到过的那个坑
    #    （不按 param_type 解，`NAV_RCL_ACT = 1` 会读成 1.4e-45，一整批参数看着像"被改了"）
    bit = lambda u: struct.unpack("<f", struct.pack("<I", u & 0xFFFFFFFF))[0]  # noqa: E731
    cases = [  # (线上的位型, param_type, 期望的真值, 说明)
        (bit(1), INT32, 1.0, "NAV_RCL_ACT = 1"),
        (bit(0xFFFFFFFF), INT32, -1.0, "`-1` 不许读成 nan"),
        (bit(921600), INT32, 921600.0, "SER_TEL2_BAUD = 921600"),
        (bit(30), INT32, 30.0, "COM_FLIGHT_UUID = 30"),
        (bit(0xDEADBEEF), UINT32, float(0xDEADBEEF), "无符号档（伪参数 `_HASH_CHECK` 那种）"),
        (0.5, REAL32, 0.5, "浮点**直传**，不许动它"),
        (-0.061959743499755859, REAL32, -0.061959743499755859, "同上，负值"),
    ]
    bad4 = []
    for raw, pt, want, why in cases:
        got, _ = decode_param_value(raw, pt)
        if got != want:
            bad4.append(f"{why}: 位型 {raw!r} 应解出 {want}，实得 {got}")
    if bad4:
        print("❌ 整型位型解码不对：\n   " + "\n   ".join(bad4))
        return 1
    print(f"   ✅ {len(cases)} 条位型解码对得上"
          f"（整型按 param_type 还原，浮点原样过）")

    print("\n自检通过。")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="从串口**只读**读回飞控参数，并与一份快照逐条对账")
    ap.add_argument("--port", default=DEFAULT_PORT, help=f"默认 {DEFAULT_PORT}")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"默认 {DEFAULT_BAUD}")
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                    help=f"等 HEARTBEAT 与收参数的总上限 [s]，默认 {DEFAULT_TIMEOUT_S}")
    ap.add_argument("--compare", metavar="EXPORT",
                    help="与这份 QGC 导出对账（例如 docs/assets/z0.params）")
    ap.add_argument("--name", help="额外打印这几条（逗号分隔）")
    ap.add_argument("--yaml", metavar="OUT", help="把读回的全量写成 `名字: 值`")
    ap.add_argument("--self-test", action="store_true",
                    help="离线自检（不碰串口）")
    ap.add_argument("--self-test-export", default="docs/assets/z0.params",
                    help="自检用的导出文件")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test(args.self_test_export)

    ref: Optional[Dict[str, Param]] = None
    if args.compare:
        try:
            ref, _ = read_export(args.compare)
        except (ExportParseError, OSError) as exc:
            print(f"读快照失败：{exc}")
            return 2
        print(f"快照 {args.compare}：{len(ref)} 条")

    print(f"只读拉取 {args.port}@{args.baud}（只发 PARAM_REQUEST_LIST，不发 PARAM_SET）…")
    try:
        cur, info = pull_params(args.port, args.baud, args.timeout)
    except Exception as exc:  # 链路层的失败要**看得见**，不是一个 traceback 完事
        print(f"读回失败：{exc}")
        return 2
    print(f"  HEARTBEAT：sys={info['target_system']} comp={info['target_component']} "
          f"autopilot={info['autopilot']} type={info['type']}")

    if args.yaml:
        Path(args.yaml).write_text(fmt_params_yaml(cur), encoding="utf-8")
        print(f"全量快照已写：{args.yaml}")

    names = [n.strip() for n in args.name.split(",")] if args.name else []
    return print_report(cur, ref, names, info)


if __name__ == "__main__":
    sys.exit(main())
