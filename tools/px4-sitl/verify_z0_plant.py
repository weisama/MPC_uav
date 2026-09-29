#!/usr/bin/env python3
"""验收 z0 同构 SITL 被控对象：**静态**（参数真的生效了吗）+ **动态**（被控对象真的按 z0 出力吗）。

为什么静态不够：参数"设进去了"离"被控对象照它动了"还有一整条链 ——
    覆盖层 → 控制分配器（CA_ROTOR*）→ 混控推力曲线（THR_MDL_FAC）→ pwm_out_sim →
    `actuator_outputs_sim` → SIH 刚体 + 一阶电机
本脚本两段都做。

**静态**：`param show <前缀>*` 逐条比对期望值（期望值直接来自 `plant_spec.py`，不另抄一份）。

**动态**：把四个电机开到给定电机信号，读 SIH 的**地面真值**话题算实际总推力。
用真值而不是 `sensor_accel`，因为 SIH 给加速度计加了很大的高斯噪声
（`sih.cpp:440` 的 `noiseGauss3f(0.5f, 1.7f, 1.4f)`，σ≈1.4–1.7 m/s²）；而
`vehicle_local_position_groundtruth` 的 `az = _v_I_dot(2)`、`vz`、`z` 与
`vehicle_attitude_groundtruth` 的 `q` 都是**无噪声**的（`sih.cpp:495 publish_ground_truth()`）：

    离地后：az = [m·g − T·cosθ − KDV·vz] / m
    ⟹ T = [m·(g − az) − KDV·vz] / cosθ      （θ 与 az 取**同一帧**真值）

⚠️ 四个坑（都是实测踩出来的，全部七个见 docs/control/sitl-z0-plant.md §7）：

1. **命令要留间隔**：连发 4 条 `actuator_test set`（间隔 20 ms）会丢命令 ——
   实测只落了后三条，第 0 号电机停在 0（pxh 每条命令要逐字符回显重画，喂太快会丢）。
2. **中间态必须压在地面上**：`sih.cpp:373 equations_of_motion()` 里的假地面（钳位在
   `sih.cpp:385-397`）会把接地时的
   `_v_I`/`_w_B` 清零，所以"三电机在地面、第四台一推就起飞"的配置不会翻滚。
   于是取 Σu 使 **3u < m·g/SIH_T_MAX < 4u**（本脚本按变体自动解出 u）。
3. **等的是仿真时间不是墙钟**：SIH 是 lockstep，控制台一忙，仿真时间就慢于墙钟
   （实测能到 0.3×）。所以按真值话题的 `timestamp_sample` 推进量判断是否够稳。
4. **★ `-v` 是"相对推力"，曲线方向是反的**（详见 `curve_inv` 的文档串）：
   要得到电机信号 u，下发的是 `curve(u, a)`；`THR_MDL_FAC = 0` 时曲线是恒等映射，
   **方向搞反了也看不出来** —— 只有 `z0-thrcurve` 变体会把它暴露成
   "读回 0.852 而不是 0.7227"。本脚本第一版就是这么中招的。

用法：
    python3 tools/px4-sitl/verify_z0_plant.py                  # z0 基准
    python3 tools/px4-sitl/verify_z0_plant.py --variant z0-mass+30
    python3 tools/px4-sitl/verify_z0_plant.py --variants-all   # 全部变体各跑一遍
    python3 tools/px4-sitl/verify_z0_plant.py --static-only
    python3 tools/px4-sitl/verify_z0_plant.py --json /tmp/z0.json
"""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "z0-plant"))
import plant_spec  # noqa: E402  （同目录真源，期望值不另抄一份）

from px4_console import Px4Console, Px4ConsoleError, parse_overlay_into_env  # noqa: E402

G = 9.80665
# `param show` 只打到小数点后 4 位 ⟹ 比对容差按"打印精度"给，不按浮点位给
PARAM_TOL_ABS = 5e-5
PARAM_TOL_REL = 2e-3
# 动态推力的相对容差：覆盖层 → 混控 → 被控对象全链，留 10 %
THRUST_TOL_REL = 0.10
# 允许的倾斜（诊断用；T 的还原已按 cosθ 修正，倾斜本身不破坏公式）
MAX_TILT_DEG = 15.0
# 命令之间的间隔：实测 <0.1 s 会丢命令（见模块 docstring 坑 1）
CMD_SETTLE = 0.35
# 电机信号 u 的取法：u = A/3.45，A = m·g/SIH_T_MAX（集体悬停点）
# Σu 的取法：3u 还压在地面（接地时 SIH 把 _w_B 清零，sih.cpp:385-397），4u 才起飞；
# 3.45 ∈ (3,4) 取偏 4 一侧，让起飞裕度约 16 %（4u ≈ 1.16·A_悬停），同时 u 不撞 U_MAX。
U_FRAC = 3.45
# 0.97 是「别把电机信号顶到 1.0」的留白（留给混控器/分配器的余量），不是 z0 读数。
U_MAX = 0.97

#: 一个数的写法 —— `5` / `-1.5` / `.5` / `5.` / `1.5e-3`。**至少要有一个数字**。
#:
#: ☠️ 不能图省事写 `-?[0-9.eE+-]+`：那个字符类**本身**能只匹配到一个 `-`（或 `e`/`E`/`.`/`+`），
#:    于是 `float('-')` 抛 `ValueError`。本脚本的窗口是**文本偏移**
#:    （`marker = len(px.text())` → `px.text()[marker:]`），而 `px4_console.text()` /
#:    `wait_for()` 的 docstring 自己就把"文本偏移当游标"判为**无效**（渲染不是增量的，
#:    切片可能从 token 中间开始）⟹ 窗口边界可以落在数值中间。
#:    2026-09-29 [ticket 23] 点名此处（与 `verify_offboard_smoke.field()` 同一类修法）。
#:    第一手：`param_value(" x * SIH_MASS [843,1330] : -", "SIH_MASS")` 与
#:    `field("    z: -", "z")` 修前**都抛** `ValueError`；完整行分别给 `0.75` / `-0.01408`。
#:    ⚠️ **"可达"不是"已发生"** —— 没有证据说真跑里被咬过。
_NUM = r"-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?"

# `param show` 一行是 `%c %c %s [i,j] : v` —— **两列状态**（used + saved/unsaved），
# 即 `x * SIH_MASS [843,1330] : 0.7500` / `x   SENS_EN_GPSSIM [..] : 1`（默认值那列是空格）。
PARAM_RE = re.compile(rf"^\s*[x+*]?\s+[+* ]?\s*([A-Z0-9_]+)\s+\[[^\]]*\]\s*:\s*({_NUM})", re.M)
OUT_RE = re.compile(r"^\s*output:\s*\[([^\]]*)\]", re.M)
QT_RE = re.compile(r"^\s*q:\s*\[([^\]]*)\]", re.M)


def param_value(text: str, name: str):
    for m in PARAM_RE.finditer(text):
        if m.group(1) == name:
            return float(m.group(2))
    return None


def field(text: str, name: str):
    """取 `名字: 数` 一行（真值话题的字段）。

    ★ 数用 `_NUM`（**至少一个数字**）：半截窗口里的裸 `-`/`e`/`.`/`+` 一律取不到 ⟹ 返回
      `None`、**绝不抛**。旧字符类 `-?[0-9.eE+-]+` 会在这种半截值上 `float('-')` 抛
      （见 `_NUM` 上方那段；ticket 23）。
    """
    m = re.search(rf"^\s*{name}:\s*({_NUM})", text, re.M)
    return float(m.group(1)) if m else None


def outputs(text: str):
    m = OUT_RE.search(text)
    if not m:
        return None
    return [float(v) for v in m.group(1).split(",")]


def tilt_cos(text: str):
    """`vehicle_attitude_groundtruth` 的 q=[w,x,y,z] → cosθ（θ = 倾斜角）。"""
    m = QT_RE.search(text)
    if not m:
        return None
    q = [float(v) for v in m.group(1).split(",")]
    if len(q) != 4:
        return None
    # PX4 四元数是 w-first（Hamilton）；cosθ = 1 − 2(x²+y²)
    _w, x, y, _z = q
    return 1.0 - 2.0 * (x * x + y * y)


def curve(u: float, a: float) -> float:
    """电机信号 → 相对推力：`rel_thrust = a·u² + (1−a)·u`（`FunctionMotors` 的注释式）。"""
    return a * u * u + (1.0 - a) * u


def curve_inv(t: float, a: float) -> float:
    """★ 相对推力 → 电机信号：**`FunctionMotors::updateValues` 实际算的就是这个方向**。

    源码（`src/lib/mixer_module/functions/FunctionMotors.hpp`）：

        rel_thrust = factor * x^2 + (1-factor) * x   // x = 电机信号
        values[i] = -tmp1 + sqrtf(tmp2 + (control / a));   // 传进来的 control 是**推力**

    即：喂进去的是**推力**，吐出来的是**反解出的电机信号**。`actuator_test` 只是把
    `-v` 当推力交给它（`actuator_test.cpp:83`），所以 `-v` 的语义是**相对推力，不是电机信号**
    —— `THR_MDL_FAC = 0` 时两者恰好相等（曲线是恒等），这也是这个方向容易被搞反的原因。
    """
    if a <= 0.0:
        return t
    b = 1.0 - a
    return -b / (2.0 * a) + math.sqrt(b * b / (4.0 * a * a) + t / a)


def expected_params(variant: str) -> dict:
    """期望值 = 覆盖层会 `param set` 的那些 + 机型文件里的那些。

    ★ 查的是 `all_variants()`（基线 ∪ 加密档）而**不是** `build_variants()` ——
      后者只有 ±30 % 那套基线，`z0-mass+10` 这种加密档名会 **KeyError**
      （ticket 12 的判据⑤ 正靠这个函数，见 `mismatch_sweep.expected_params`）。
    """
    out = dict(plant_spec.all_variants()[variant])
    for name, value, _s, _w, scope in plant_spec.ROWS:
        if scope == "airframe":
            out[name] = value
    return out


def run_one(variant: str, args) -> dict:
    build_dir = Path(args.build_dir)
    overlay = HERE / "z0-plant" / "params" / f"{variant}.param"
    if not overlay.exists():
        raise SystemExit(f"没有这个变体：{variant}（--list 看全部）")
    if not (build_dir / "etc/init.d-posix/airframes/10046_sihsim_z0").exists():
        raise SystemExit("z0 机型没装 —— 先跑 install_z0_airframe.sh")

    if not args.keep_params:
        # 从零起手：否则 rootfs/parameters.bson 里的历史残留会让结果不可归因
        for f in ("parameters.bson", "parameters_backup.bson"):
            (build_dir / "rootfs" / f).unlink(missing_ok=True)

    tmpdir, env = parse_overlay_into_env(str(overlay))
    env["PX4_SIM_MODEL"] = "sihsim_z0"
    env["PX4_SIMULATOR"] = "sihsim"

    result = {"variant": variant, "overlay": str(overlay), "checks": [], "ok": True}
    px = Px4Console(cwd=build_dir, env=env)

    def check(name, ok, detail):
        result["checks"].append({"name": name, "ok": bool(ok), "detail": detail})
        result["ok"] &= bool(ok)

    def listen(topic, timeout=30.0):
        """敲一次 `listener <topic>`，等它自己的 `TOPIC:` 行回来再解析。

        为什么要等"哨兵"而不是固定睡眠：pxh 会**逐字符重画整行**（关掉驱动回显也还在），
        命令越长重画越多，控制台一忙响应就晚到 —— 固定睡眠只能靠猜（实测踩过：
        7 条 `param show` 只等到后两条的响应）。
        """
        marker = len(px.text())
        px.send(f"listener {topic}", settle=0.05)
        m = px.wait_for(rf"(?m)^TOPIC: {topic}", timeout=timeout, since=marker)
        px.drain(0.4)   # 让 uORB 那段字段一次写完
        return px.text()[marker + m.start():]

    try:
        px.start()
        px.wait_ready(args.ready_timeout)
        # rcS 跑完 ≠ 启动完了：logger / mavlink / EKF 还在后面刷屏。不等它们刷完就
        # 灌命令，控制台会被拖住（实测：7 条 `param show` 的响应要几十秒才吐完）。
        px.drain(args.boot_settle)

        # ---------------- 静态：参数真的生效了吗 ----------------
        marker = len(px.text())
        for prefix in ("SIH_", "CA_ROTOR", "PWM_MAIN_FUNC", "MC_", "MPC_", "SENS_EN_", "CBRK_"):
            px.send(f"param show {prefix}*", settle=0.05)
        # 哨兵：最后一条用**精确名**（带 `*` 的通配过滤必须吃掉整个名字，参 `do_show_print`）。
        # 命令是顺序处理的，看到它的响应就说明前面的都打完了。
        px.send("param show THR_MDL_FAC*", settle=0.05)
        px.wait_for(r"(?m)^\s*[x ]\s+[+* ]\s+THR_MDL_FAC\s+\[", timeout=60.0, since=marker)
        text = px.text()[marker:]
        want = expected_params(variant)
        for name, want_v in sorted(want.items()):
            got = param_value(text, name)
            if got is None:
                check(f"param {name}", False, "没读到（param show 输出里没有）")
                continue
            tol = PARAM_TOL_ABS + PARAM_TOL_REL * abs(want_v)
            check(f"param {name}", abs(got - want_v) <= tol,
                  f"期望 {want_v:.6g}，读到 {got:.6g}")

        # ---------------- 动态：被控对象真的按 z0 出力吗 ----------------
        dyn = {"checks": []}
        result["dynamic"] = dyn
        if not args.static_only:
            m = want["SIH_MASS"]
            kdv = want["SIH_KDV"]
            t_max = want["SIH_T_MAX"]
            a_mdl = want["THR_MDL_FAC"]

            # Σu 的悬停点（单位：电机信号之和）。取 3u < A < 4u：
            #   · 三台电机还压在地面上 → 不会翻滚（接地时 SIH 把 _w_B 清零）
            #   · 四台齐了就起飞，且只比悬停高 16 %（翻滚只来自最后那台的爬升）
            A = m * G / t_max
            u = min(A / U_FRAC, U_MAX)
            # `-v` 的语义是**相对推力**（见 curve_inv 的文档串）：想要电机信号 u，
            # 要发的是 curve(u)；混控器再把它反解回 u_expect —— 三方（我 / 混控器 / 读回）对得上，测量才成立。
            v = curve(u, a_mdl)
            u_expect = curve_inv(v, a_mdl)
            t_pred = 4.0 * t_max * u
            dyn.update(mass=m, t_max=t_max, sum_u_hover=A, motor_signal=u,
                       motor_signal_expect=u_expect, thrust_cmd=v,
                       thrust_pred=t_pred, thr_mdl_fac=a_mdl)

            gt0 = listen("vehicle_local_position_groundtruth")
            z0_, az0, vz0 = field(gt0, "z"), field(gt0, "az"), field(gt0, "vz")
            dyn["before"] = {"z": z0_, "az": az0, "vz": vz0}
            check("动态基线：落地静止（az==0 且 z≈0）",
                  None not in (z0_, az0) and abs(az0) < 1e-3 and abs(z0_) < 0.05,
                  f"z={z0_}, az={az0}, vz={vz0}")

            for motor in (1, 2, 3, 4):
                px.send(f"actuator_test set -m {motor} -v {v:.4f} -t {args.hold_s}", settle=CMD_SETTLE)

            # 按**仿真时间**推进，而不是墙钟（SIH lockstep，控台一忙仿真就慢）
            outs = gt = att = None
            resent = []
            t_ref = None
            for attempt in range(args.max_reads):
                outs = outputs(listen("actuator_outputs_sim"))
                # 漏掉的命令就地补发（坑 1）；只补**偏低**的，不把已到位的拉下来
                sent = False
                if outs is not None:
                    for motor in range(1, 5):
                        if outs[motor - 1] < u_expect - 0.02:
                            resent.append(motor)
                            sent = True
                            px.send(f"actuator_test set -m {motor} -v {v:.4f} -t {args.hold_s}",
                                    settle=CMD_SETTLE)
                gt = listen("vehicle_local_position_groundtruth")
                att = listen("vehicle_attitude_groundtruth")
                ts = field(gt, "timestamp_sample")
                if t_ref is None and ts is not None:
                    t_ref = ts
                elif ts is not None and ts - t_ref >= args.settle_sim_s * 1e6 and not sent:
                    # 刚补发过就不收工：这一轮的 `outs` 是补发**之前**读的，
                    # 不能拿它去证明"四条都到位"。
                    break
                px.drain(0.3)

            z1, az, vz = field(gt, "z"), field(gt, "az"), field(gt, "vz")
            ct = tilt_cos(att)
            dyn["after"] = {"z": z1, "az": az, "vz": vz, "cos_tilt": ct,
                            "outputs": outs, "reads": attempt + 1, "resent_motors": resent}

            if outs is None or gt is None or att is None:
                check("动态：读到真值话题", False, "listener 没给出可解析的输出")
            else:
                # 四条命令都落到位了吗（坑 1 的兜底：没落到位就是测量无效，不是被控对象的错）。
                # 期望值是**按混控器那套公式自己算的** `u_expect`：读回的电机信号与它对得上，
                # 才说明"我理解的曲线"和"混控器真正做的"是同一件事。
                landed = all(abs(o - u_expect) < 0.02 for o in outs[:4])
                ok_tilt = ct is not None and ct > math.cos(math.radians(MAX_TILT_DEG))
                ok_air = z1 is not None and z1 < -0.02
                t_est = (m * (G - az) - kdv * vz) / ct
                dyn["thrust_est"] = t_est
                dyn["thrust_err_rel"] = (t_est - t_pred) / t_pred
                check("动态：四条电机命令都落到位（`actuator_outputs_sim` = 电机信号）",
                      landed,
                      f"读到 {[round(o, 4) for o in outs[:4]]}，期望 {u_expect:.4f}"
                      f"（-v {v:.4f} 过混控器曲线，THR_MDL_FAC {a_mdl}）")
                check("动态：真的离地了（否则竖向下式不成立）", ok_air, f"z={z1}")
                check(f"动态：姿态基本水平（倾斜 <{MAX_TILT_DEG:.0f}°，公式只按 cosθ 修正）",
                      ok_tilt,
                      f"cosθ={ct:.4f}（{math.degrees(math.acos(max(-1.0, min(1.0, ct)))):.1f}°）")
                if az is None or ct is None:
                    check("动态：推力还原", False, f"az={az}, cosθ={ct}")
                else:
                    check("动态：实测总推力 ≈ 4 × SIH_T_MAX × 电机信号",
                          abs(dyn["thrust_err_rel"]) <= THRUST_TOL_REL,
                          f"实测 {t_est:.3f} N vs 期望 {t_pred:.3f} N"
                          f"（{dyn['thrust_err_rel'] * 100:+.1f}%）")
            px.drain(0.5)
    except Px4ConsoleError as e:
        # 控制台超时不算"被控对象不对"，但也绝不能算通过：记一条失败，并把
        # 原始控制台留在盘上（报告里看不到的东西，事后得能自己去看）。
        check("控制台", False, str(e).splitlines()[0])
        result["console_dump"] = args.debug_dump
        Path(args.debug_dump).write_text(px.text(), encoding="utf-8")
    finally:
        px.close()
        if tmpdir:
            shutil.rmtree(tmpdir, ignore_errors=True)
    return result


def report(results) -> int:
    bad = 0
    for r in results:
        print(f"\n=== 变体 {r['variant']} ===")
        for c in r["checks"]:
            mark = "✅" if c["ok"] else "❌"
            print(f"  {mark} {c['name']}：{c['detail']}")
            bad += 0 if c["ok"] else 1
        d = r.get("dynamic") or {}
        if "thrust_est" in d:
            print(f"  ·  电机信号 {d['motor_signal']:.4f}（-v {d['thrust_cmd']:.4f}，"
                  f"THR_MDL_FAC {d['thr_mdl_fac']}）")
            print(f"  ·  推力：实测 {d['thrust_est']:.3f} N，期望 {d['thrust_pred']:.3f} N"
                  f"（{d['thrust_err_rel'] * 100:+.2f}%）")
    print(f"\n{'全部通过' if bad == 0 else f'{bad} 项不通过'}")
    return 0 if bad == 0 else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--variant", default="z0")
    ap.add_argument("--variants-all", action="store_true")
    ap.add_argument("--static-only", action="store_true")
    ap.add_argument("--hold-s", type=int, default=20, help="actuator_test 的保持秒数")
    ap.add_argument("--boot-settle", type=float, default=3.0,
                    help="rcS 之后再多等几秒，让 logger/mavlink 刷完屏（否则控台很慢）")
    ap.add_argument("--debug-dump", default="/tmp/verify_z0_plant_console.log",
                    help="控制台出错时把原始控制台写到哪儿（报告里看不到的东西）")
    ap.add_argument("--settle-sim", dest="settle_sim_s", type=float, default=1.0,
                    help="最后一条命令之后要走的**仿真**秒数（不是墙钟）")
    ap.add_argument("--max-reads", type=int, default=8)
    ap.add_argument("--keep-params", action="store_true",
                    help="保留 rootfs/parameters.bson（默认先删，保证结果可归因）")
    ap.add_argument("--src", default=str(Path.home() / "refs/px4-v1.14.3-build"))
    ap.add_argument("--build-dir", default=None)
    ap.add_argument("--ready-timeout", type=float, default=90.0)
    ap.add_argument("--json", default=None, help="把结果写成 JSON（文档取证用）")
    args = ap.parse_args(argv)
    args.build_dir = args.build_dir or str(Path(args.src) / "build/px4_sitl_default")

    variants = list(plant_spec.build_variants()) if args.variants_all else [args.variant]
    results = []
    for v in variants:
        print(f"[verify] 跑 {v} …", file=sys.stderr)
        results.append(run_one(v, args))

    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=2, ensure_ascii=False),
                                   encoding="utf-8")
    return report(results)


if __name__ == "__main__":
    sys.exit(main())
