#!/bin/bash
# ticket 11 的矩阵（F0 归因对照 / F1 判据① / F2 判据③ / F3 判据④失配 / F4 判据④标称
#              + F0b/F1b、F0c/F1c 两次复跑（ticket 20）+ **T0 八字那一场**（判据②·轨迹2））
# ⚠️ 必须在**空机**上跑：这套数字里有一条是"实测最坏求解耗时"，多一个进程就把它抬高（A 档 §5.1）。
#
# ☠️ 别加 `set -u`：ROS 的 setup.bash 会读**未定义变量**，非交互 shell 下 `set -u`
#    直接让 shell 退场 —— 症状是"脚本跑了两行就没了，退出码 1"，看着像夹具崩了。
set +u

OUTDIR=/tmp/tier-b
LOG=/tmp/run_matrix.log
# ☠️ `--model` 故意**不写**：五场当时写的是 `/tmp/mocap-model-3.json`，而 `/tmp` **过不了重启** ——
#    机器 2026-09-29 10:40 重启后那份没了。验证器的默认值已改成**本归档里那份**
#    （`evidence/tier-b/mocap-model-3.json`，内容与当时一致，见其模块 docstring）。
#    ★ 机制订正（2026-09-29，本行原写"`/tmp` 是 tmpfs"）：`/tmp` 在 z0 上是**根盘 ext4**
#    （`df -T /tmp` ⟹ `/dev/nvme0n1p1 ext4`），清空来自 tmpfiles 里那条 `D /tmp`
#    （`/usr/lib/tmpfiles.d/tmp.conf:11`）。**同一次开机内 `/tmp` 是持久的** ——
#    这个差别有实际后果：照"tmpfs"理解会以为同会话也会丢，于是把证据随手放别处；
#    真相是"开机内能用、但一手证据仍得当场落仓库"。
COMMON="--hold-s 15 --dut-timeout-s 90 --offboard-timeout-s 45 --outdir $OUTDIR"

say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
: > "$LOG"

# ☠️ 别用 `pgrep -f 'px4|MicroXRCEAgent|...'`：`-f` 匹配的是**整条命令行**，
#    于是任何一条**只是文本里含这些词**的进程都会被算进来 —— 实测两次都栽在这：
#      · 本脚本自己的管道/子 shell（首字母加方括号能挡住自己的模式串，挡不住别的）
#      · 并行会话那条 `bash -c '...MicroXRCEAgent...'` 的**等待循环**（它只是 sleep，
#        既不发一帧 DDS 也不吃 CPU，却被算成"机上有人"，把空机判等成死循环）
#    改法：**按真进程名精确匹配**（`-x`）。要拦的只有三类真实参与者：
#    SITL 本体 / DDS 桥 / 跑 mpc_node 或 pytest 的 python3。
BUSY() {
  { pgrep -x px4_sitl_default
    pgrep -x MicroXRCEAgent
    pgrep -ax python3 2>/dev/null | grep -E "mpc_node\.py|pytest" || true
  } | grep -v "^$$" || true
}

# 等空机（最多 QUIET_MAX_S）：这不只是"别串 DDS"，还因为本套数字里有一条
# **实测最坏求解耗时** —— A 档量过，机上多几个进程就把它从 4 ms 抬到 11.3 ms（§5.1）。
QUIET_MAX_S=${QUIET_MAX_S:-1800}
t0=$(date +%s)
while [ -n "$(BUSY)" ]; do
  if [ $(( $(date +%s) - t0 )) -ge "$QUIET_MAX_S" ]; then
    say "⚠️ 等空机超时（${QUIET_MAX_S}s），机上仍有人："; BUSY | tee -a "$LOG"; exit 2
  fi
  say "等空机……（$(BUSY | wc -l) 个进程）"
  sleep 20
done

before=$(ls /dev/shm/ 2>/dev/null | grep -c '^fastrtps_port\|^sem\.fastrtps_port' || true)
rm -f /dev/shm/fastrtps_port* /dev/shm/sem.fastrtps_port*
say "空机；清了 $before 个 fastrtps_port* 残留"

source /opt/ros/humble/setup.bash                >/dev/null 2>&1
source /home/chen/px4_msgs_ws/install/setup.bash >/dev/null 2>&1
source /home/chen/ros_ws/install/setup.bash      >/dev/null 2>&1
export PYTHONPATH=/home/chen/ros_ws/src/nmpc:$PYTHONPATH
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
cd /home/chen/ros_ws || exit 1

fly() {  # fly <tag> <其余参数…>
  tag=$1; shift
  say "▶ $tag 起飞：$*"
  python3 tools/px4-sitl/verify_tier_b_seam3.py --tag "$tag" $COMMON \
      --json "$OUTDIR/$tag.json" --debug-dump "$OUTDIR/$tag-console.log" "$@" \
      >> "$LOG" 2>&1
  say "◀ $tag 退出码 $?（json: $OUTDIR/$tag.json）"
}

# 只跑 $TAGS 里点名的场 —— 便于先单跑一场看仪器通不通，再决定要不要把整批都烧掉。
# ☠️ 默认**不含 `T0`**（2026-10-09 订正：此行原写"默认全部"，而列表里没有 `T0` ——
#    那句注释与列表自相矛盾，症状是"照票面裸跑"时 `T0` 一场都不跑、末尾汇总只打印
#    一行 `T0: ⚠️ 没有 json`，**报告看着像跑完了**）。要判据② 就得显式点名：
#        TAGS="F0 F1 F2 F3 F4 T0" tools/px4-sitl/evidence/tier-b/run_matrix.sh
#    另有一层理由：`T0` 依赖包 5 那份模型（票 05），显式点名比"默认带上"更不容易踩。
# ★ 末尾那个汇总循环列的**是六场**（`F0…F4, T0`）—— 那是"报告里能并列的场次"，
#   不是"默认会飞的场次"；把两者读成一件事就会以为 `T0` 在里面。
for tag in ${TAGS:-F0 F1 F2 F3 F4}; do
  case "$tag" in
    F0) fly F0 --mismatch none ;;
    F1) fly F1 --mismatch mass+30 ;;
    F2) fly F2 --mismatch mass+30 --extra-delay-ms 50 ;;
    F3) fly F3 --mismatch mass+30 --inject-solve-fail-after-frames 700 ;;
    F4) fly F4 --mismatch none --inject-solve-fail-after-frames 700 ;;
    # ── ticket 20 的两场重跑（垂直通道加了积分作用之后，只重跑判据① 那一对）──
    # ★ 与 F0/F1 **同基线、同参数**，所以新数字与那五场可直接比（裁定 3：本张不重搭被控对象）。
    #   标签加 `b` 是为了**不覆盖**归档里的 F0/F1（它们的读数属于 08:59 那份 `install/` 快照）。
    # ☠️ 跑之前先 `colcon build --packages-select nmpc`：积分器写在 `mpc_node.py` 里，
    #   而 DUT 的**入口脚本**来自 `install/`（`ros2 run` 按 `<prefix>/lib/<包>/<名字>` 找文件）
    #   ⟹ 不 build 的话 `mpc_node.py` **自身**的改动完全不生效，
    #   症状指纹 = 日志里**没有 `df` 列**、读数与加积分作用之前逐位相同（ticket 20 §8.6 ★、§9.6）。
    #   ⚠️ 别推成"改什么都要 build"：被 `import` 的 `node_core.py`/`ocp.py` 走的是 `src/` 正规包
    #   （本脚本下面那行 `export PYTHONPATH=…/src/nmpc` 把 `src/` 前置、DUT 子进程继承）
    #   ⟹ 那类改动不 build 也会飞。☠️ 这里故意**不写行号**：上面一改行号就飘。
    F0b) fly F0b --mismatch none ;;
    F1b) fly F1b --mismatch mass+30 ;;
    # ── ticket 20 换落点（裁定 4 的候选 A）之后的第二次复跑 ──
    # ☠️ **为什么不是重跑 F0b/F1b**：那两份归档记的是**被证伪的那个落点**（偏置加在 `u_ref` 上，
    #   见 20 号票 §10）—— 沿用同一个标签会把 `/tmp/tier-b/{F0b,F1b}.json` 与逐帧覆盖掉，
    #   而"把作废读数留在旁边"是本仓库的纪律。换标签一次，两份记录并存、可对照。
    #   与 F0b/F1b 的唯一差别 = `_normal_frame` 里 `u_cmd[0] = clip(u[0] + ΔF, 0, f_max)`
    #   （§11.2），`ki_z` 仍是 1.0。
    F0c) fly F0c --mismatch none ;;
    F1c) fly F1c --mismatch mass+30 ;;
    # ── 判据② 的第二条轨迹：r = 1.0 m 八字（包 5，2026-10-08 录成；票 05 起 `T0` 接它）──
    # 这一场**不是** F 系列：`F*` 是"锚点 + 常量偏移"的定点悬停场，这一场让夹具逐点重发
    # 包 5 里那条 `/traj/reference`（`--publish-reference`），DUT 在 `reference_source=traj`
    # 下追它 ⟹ 判据②·轨迹2 才有得判（判据②·轨迹1 由 F0c/F1c 那几场判）。
    # ☠️ 三个都是"写错了完全静默"的旋钮，逐条说明：
    #   ① `--model` **必须**是包 5 那份。拿**包 3**（参考恒为 0、3873 帧全在原点）⟹
    #      夹具发帧无异常、飞机照飞，报告里只剩一句"参考一帧没动" —— 这一条**会被拦**，
    #      验证器 `traj_model_problem()` 在起世界之前就退（纯函数，`test_tier_b_judges.py` 钉着）。
    #      ☠️ 但拿**包 4**（0.5 m 的旧规格）**拦不住**：它参考非空、只是短一半（`span_s`
    #      20.56 vs 26.22）⟹ 报告会像"八字跑完了"，判的却不是当前规格那条。挡它的是
    #      `test_tier_b_judges.py::test_T0接的是1_0m那份八字`（离线，直接量参考包络）。
    #   ② `--hold-s 66`：下界 = 顶住首点 30 + 参考全长 26.22（模型里量的）+ 判据① 稳态窗口 8
    #      = 64.22 s。**一个都少不得**，少一截会让判据① 的稳态窗口混进八字/爬升段 ——
    #      读出来像"悬停精度很差"，与控制器无关。65 是下界（`math.ceil`），给 66 留一拍。
    #      ★ `--model` 与 `--hold-s` 这两个数**不逐字抄给测试**：
    #      `test_hold_s太短就拦住_下界从模型现算` 直接从这一行现取、与模型里的 `span_s` 对账，
    #      所以**改短这一行会先让测试红**。
    #   ③ `--hold-z 1.0`（默认）在 traj 档**不是** DUT 的 `hold_offset_z`，而是夹具的
    #      `--ref-z-offset-m`：包里的参考 z ≡ 0（那是一条贴地画的八字），高度得由夹具叠。
    #      DUT 那一档的 `hold_offset` 必须全 0（非零会被 `mpc_node` 拒跑）。
    # ⚠️ `--mismatch mass+30`：与 F1c 同一档错参数 ⟹ 读数可与 F1c 并排看
    #    （"同一架错参数的飞机，定点悬停 vs 追八字"）。要归因对照再加一场 `none`。
    T0) fly T0 --mismatch mass+30 --reference-source traj \
             --model tools/px4-sitl/evidence/tier-b/mocap-model-5.json \
             --hold-s 66 ;;
    *)  say "不认识的场次 $tag"; exit 2 ;;
  esac
done

say "全部跑完，汇总："
python3 - <<'PY' 2>&1 | tee -a "$LOG"
import json, pathlib
for tag in ("F0", "F1", "F2", "F3", "F4", "T0"):
    p = pathlib.Path(f"/tmp/tier-b/{tag}.json")
    print("=" * 78)
    if not p.exists():
        print(f"{tag}: ⚠️ 没有 json"); continue
    d = json.loads(p.read_text())
    print(f"{tag}  被控对象={d.get('mismatch')}  延迟={d.get('extra_delay_ms')} ms  "
          f"注入解失败={d.get('inject_solve_fail_after_frames')} 帧  悬停={d.get('hold_s')} s"
          f"  参考源={d.get('reference_source')}")
    # traj 档（T0）多两块：夹具的**参考时间线**（八字发出去没有、发到哪一刻）
    # 与判层留档的**跟踪误差**（只留档，spec 没有数值界 ⟹ 不设判据）。
    tl = d.get("reference_timeline") or {}
    if d.get("reference_source") == "traj":
        print(f"  参考时间线: n={tl.get('n')} 移动 {tl.get('n_moving')} 帧 "
              f"[{tl.get('move_first_s')}, {tl.get('move_last_s')}] s "
              f"相位 {tl.get('by_phase')}")
        er = ((d.get("traj") or {}).get("lemniscate") or {}).get("err") or {}
        print(f"  跟踪误差(只留档): 水平均值 {er.get('h_mean')} m / 最大 {er.get('h_max')} m "
              f"3D 均值 {er.get('d3_mean')} m（配对 {er.get('n_paired')}/{er.get('n')}）")
    print(f"  armed_ever={d.get('armed_ever')}  dut_dead={d.get('dut_dead')}  "
          f"replay_up={d.get('replay_up')}  真值 n={d.get('truth_n')} @ {d.get('truth_hz')} Hz")
    print(f"  agent: {d.get('agent')}")
    sv = d.get("solve") or {}
    if sv:
        print("  solve: " + json.dumps({k: v for k, v in sv.items() if not isinstance(v, dict)},
                                       ensure_ascii=False))
        if d.get("truth_hz") and sv:
            print("  solve 逐项: " + json.dumps(sv, ensure_ascii=False)[:600])
    print(f"  summary: {str(d.get('summary'))[:400]}")
    for c in d.get("checks", []):
        mark = {True: "✅", False: "❌", None: "⏭️"}.get(c.get("ok"), "?")
        cnt = "" if c.get("counts", True) else "（不计数）"
        print(f"    {mark}{cnt} {c.get('name')}")
PY
say "汇总结束"
