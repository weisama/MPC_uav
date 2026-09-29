#!/bin/bash
# ticket 11 的五场矩阵（F0 归因对照 / F1 判据① / F2 判据③ / F3 判据④失配 / F4 判据④标称）
# ⚠️ 必须在**空机**上跑：这套数字里有一条是"实测最坏求解耗时"，多一个进程就把它抬高（A 档 §5.1）。
#
# ☠️ 别加 `set -u`：ROS 的 setup.bash 会读**未定义变量**，非交互 shell 下 `set -u`
#    直接让 shell 退场 —— 症状是"脚本跑了两行就没了，退出码 1"，看着像夹具崩了。
set +u

OUTDIR=/tmp/tier-b
LOG=/tmp/run_matrix.log
COMMON="--model /tmp/mocap-model-3.json --hold-s 15 --dut-timeout-s 90 --offboard-timeout-s 45 --outdir $OUTDIR"

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

# 只跑 $TAGS 里点名的场（默认全部）—— 便于先单跑一场看仪器通不通，再决定要不要把五场都烧掉
for tag in ${TAGS:-F0 F1 F2 F3 F4}; do
  case "$tag" in
    F0) fly F0 --mismatch none ;;
    F1) fly F1 --mismatch mass+30 ;;
    F2) fly F2 --mismatch mass+30 --extra-delay-ms 50 ;;
    F3) fly F3 --mismatch mass+30 --inject-solve-fail-after-frames 700 ;;
    F4) fly F4 --mismatch none --inject-solve-fail-after-frames 700 ;;
    *)  say "不认识的场次 $tag"; exit 2 ;;
  esac
done

say "全部跑完，汇总："
python3 - <<'PY' 2>&1 | tee -a "$LOG"
import json, pathlib
for tag in ("F0", "F1", "F2", "F3", "F4"):
    p = pathlib.Path(f"/tmp/tier-b/{tag}.json")
    print("=" * 78)
    if not p.exists():
        print(f"{tag}: ⚠️ 没有 json"); continue
    d = json.loads(p.read_text())
    print(f"{tag}  被控对象={d.get('mismatch')}  延迟={d.get('extra_delay_ms')} ms  "
          f"注入解失败={d.get('inject_solve_fail_after_frames')} 帧  悬停={d.get('hold_s')} s")
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
