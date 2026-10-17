#!/usr/bin/env python3
"""给票 02 的三趟 nominal SITL 写 `command.txt` + `harness-identity.json`。

**为什么用脚本而不是手抄哈希**：身份文件的价值在于"事后有人能**重算**并逐字节对上"。
手抄一次就会抄错一个字符，而对不上时又说不清是抄错还是软件真变了。所以这里每个哈希都
**现算**（工作树当前内容），每个字段都能用同一条命令复现。

哈希的时机：**跑完之后、未再改动过**这三份脚本 / 控制器 / 模型 / 夹具
（本会话只读过 `tools/px4-sitl/` 下那几份，`src/nmpc/` 无改动，`git status` 可核）。
所以"与飞行当时相同"是**推**的，不是从 obs 里读的 —— 这一点写进 `★ 哈希时机`。

用法（仓库根）：

    python3 tools/report/evidence/sitl/write_harness_identity.py
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
EVID = Path("tools/report/evidence/sitl")

TAGS = {
    "R-hover": {
        "bag": Path("bag/3_traj1_hover"),
        "hold_s": 78.0,
        "ref_hold_s": 30.0,
    },
    "R-square": {
        "bag": Path("bag/4_traj2_square"),
        "hold_s": 56.0,
        "ref_hold_s": 30.0,
    },
    "R-lemniscate": {
        "bag": Path("bag/5_traj3_lemniscate"),
        "hold_s": 65.0,
        "ref_hold_s": 30.0,
    },
}

CONTROLLER = ["__init__.py", "commands.py", "model.py", "mpc_node.py", "node_core.py",
              "ocp.py", "offboard_smoke.py", "params.py", "preflight_check.py",
              "sign_map.py", "timing.py"]
HARNESS = ["verify_tier_b_seam3.py", "verify_offboard_smoke.py", "verify_tier_a_hover.py",
           "mocap_replay.py", "mocap_model.py", "z0-plant/plant_spec.py"]
WRAPPERS = ["build_hover_model.py", "run_nominal_sitl.py", "extract_solver_summary.py",
            "write_harness_identity.py"]

MAVROS_LAUNCH = ("ROS_DOMAIN_ID=55 ros2 launch bringup mavros.launch.py "
                 "fcu_url:=udp://:14540@127.0.0.1:14580")


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def file_fingerprint(path: Path) -> dict:
    return {"sha256": sha(path), "size": path.stat().st_size}


def controller_params() -> dict:
    """**实跑值**从 `params.py` 现读（不是抄 README 的散文）。

    DUT 没被命令行覆盖任何一项 ⟹ 值 = 节点默认 = `params.py` 的常量 / `Z0Params` 默认。
    把值**逐项**写进身份文件，是因为票面要求"控制器参数可追溯"，而只有一个文件哈希
    并不能让复核者看出"这趟到底用的多少"。
    """
    path = REPO / "src/nmpc/nmpc/params.py"
    spec = importlib.util.spec_from_file_location("nmpc_params_identity", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod                 # dataclass 要能从 sys.modules 找到本模块
    spec.loader.exec_module(mod)
    z0 = mod.Z0Params()
    return {
        "mass_kg": z0.mass,
        "hover_thrust": mod.HOVER_THRUST,
        "t_max_n": round(mod.t_max_from_hover(z0.mass, mod.HOVER_THRUST), 4),
        "roll_rate_max_radps": mod.ROLL_RATE_MAX,
        "pitch_rate_max_radps": mod.PITCH_RATE_MAX,
        "yaw_rate_max_radps": mod.YAW_RATE_MAX,
        "f_max_n": mod.F_MAX_BATTERY,
        "来源": (f"{path}（sha256 见下；DUT 未用命令行覆盖任何一项 ⟹ 值 = 节点默认）"),
    }


def git(*argv: str) -> str:
    try:
        return subprocess.run(["git", *argv], cwd=REPO, capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError) as e:      # noqa: BLE001
        return f"（取不到：{e!r}）"


#: 失配 overlay 在字段里可能出现的写法（`verify_tier_b_seam3.py` 的 `--mismatch` 取值）。
MISMATCH_TOKENS = ("mass+30", "mass-30", "thrust+30", "thrust-30", "+30%", "-30%")


def _mismatch_markers(summary: dict) -> list[str]:
    """从 obs 的**实际字段**里找失配标记（而不是写死一个空表）。

    ★ 为什么不能写死 `[]`：那样即使真跑了 `mass+30`，身份文件也照样声称"没有失配"，
      这条验收就恒真、零鉴别力。这里把 `mismatch` / `plant_param` / `params` 三处都扫一遍。
    """
    out = []
    mm = summary.get("mismatch")
    if mm not in (None, "none"):
        out.append(f"mismatch={mm}")
    pp = str(summary.get("plant_param") or "")
    if any(tok in pp for tok in MISMATCH_TOKENS):
        out.append(f"plant_param={pp}")
    for p in summary.get("params") or []:
        if any(tok in str(p) for tok in MISMATCH_TOKENS):
            out.append(f"param={p}")
    return out


def build(tag: str, cfg: dict) -> dict:
    model = EVID / "models" / f"{tag}-model.json"
    m = json.loads((REPO / model).read_text(encoding="utf-8"))
    summary = json.loads((REPO / EVID / tag / f"{tag}-summary.json").read_text(encoding="utf-8"))
    bag = cfg["bag"]
    db3 = next(bag.glob("*.db3.zstd"))
    return {
        "这是什么": ("票 02 的一趟 nominal SITL（`--reference-source traj`）跑完之后写下的 "
                     "**这次运行自身的身份** —— 参考 / 被控对象 / 控制器 / 夹具，逐项 sha256。"),
        "★ 哈希时机": ("跑完之后、未再改动下列文件时现算（脚本 write_harness_identity.py）；"
                       "`src/nmpc/` 在跑前跑后都是 HEAD 内容（见 `git.src_nmpc_dirty` 为空）。"),
        "tag": tag,
        "运行命令": {
            "中间件（另一个终端，先起）": MAVROS_LAUNCH,
            "跑本场": (f"python3 tools/report/evidence/sitl/run_nominal_sitl.py --tag {tag} "
                       f"--model {model} --reference-source traj --mismatch none "
                       f"--hold-s {cfg['hold_s']:g} --ref-hold-s {cfg['ref_hold_s']:g} "
                       f"--outdir /tmp/phase5-sitl/{tag} "
                       f"--json {EVID / tag / f'{tag}-summary.json'} "
                       f"--debug-dump {EVID / tag / f'{tag}-console.log'}"),
            "求解汇总抽取": (f"python3 tools/report/evidence/sitl/extract_solver_summary.py "
                             f"--jsonl /tmp/phase5-sitl/{tag}/{tag}-dut.jsonl --tag {tag} "
                             f"--out {EVID / tag / 'solver-summary.json'}"),
        },
        "git": {
            "head": git("rev-parse", "HEAD"),
            "src_nmpc_dirty": git("status", "--porcelain", "--", "src/nmpc/"),
            "本票范围外的改动": ("共享工作树里还有别的票/并行会话的在制品 "
                                 "（`.scratch/phase5/` 其余票、`docs/report/chapters/` 等）；"
                                 "本票只新增 `tools/report/evidence/sitl/**` 并改本票文件。"),
        },
        "nominal": {
            "mismatch": summary.get("mismatch"),
            "plant_param": summary.get("plant_param"),
            "mismatch_markers": _mismatch_markers(summary),
            "说明": ("`--mismatch none` ⟹ 被控对象 = 标称 z0.param；没有 `mass+30` / "
                     "`thrust±30` 之类的失配 overlay。`mismatch_markers` 从 "
                     "`mismatch` / `plant_param` / `params` 现算（空表 = 真没有，不是写死的）。"),
        },
        "参考模型": {
            "path": str(model),
            "sha256": sha(REPO / model),
            "bag": str(bag),
            "segment": m.get("segment"),
            "reference": {"n": (m.get("reference") or {}).get("n"),
                          "span_s": (m.get("reference") or {}).get("span_s")},
        },
        "参考包（逐字来源）": {
            str(db3): file_fingerprint(REPO / db3),
            str(bag / "metadata.yaml"): file_fingerprint(REPO / bag / "metadata.yaml"),
        },
        "被控对象": {
            "参数文件": {str(Path("tools/px4-sitl/z0-plant/params/z0.param")):
                         file_fingerprint(REPO / "tools/px4-sitl/z0-plant/params/z0.param")},
            "机型": {str(Path("tools/px4-sitl/z0-plant/airframes/10046_sihsim_z0")):
                     file_fingerprint(REPO / "tools/px4-sitl/z0-plant/airframes/10046_sihsim_z0")},
            "真源": {"tools/px4-sitl/z0-plant/plant_spec.py":
                     file_fingerprint(REPO / "tools/px4-sitl/z0-plant/plant_spec.py")},
            "悬停折算（从 z0.param 现算）": summary.get("plant_hover"),
        },
        "控制器参数（DUT 实跑值；DUT 未用命令行覆盖任何一项）": controller_params(),
        "控制器（DUT 实际 import 的）": {
            f"src/nmpc/nmpc/{name}": file_fingerprint(REPO / "src/nmpc/nmpc" / name)
            for name in CONTROLLER
        },
        "DUT 入口脚本（ros2 run 按 install/ 找）": {
            "install/nmpc/lib/nmpc/mpc_node.py": {
                "sha256": sha(REPO / "install/nmpc/lib/nmpc/mpc_node.py"),
                "注": ("symlink → src/nmpc/nmpc/mpc_node.py（同一次 build 装出来的），"
                       "与上表那份逐字节同源"),
            },
        },
        "harness / 判层 / 夹具": {
            **{f"tools/px4-sitl/{name}": file_fingerprint(REPO / "tools/px4-sitl" / name)
               for name in HARNESS},
            **{f"tools/report/evidence/sitl/{name}": file_fingerprint(REPO / EVID / name)
               for name in WRAPPERS},
        },
        "三处代码适配（本轮新增，全部在 run_nominal_sitl.py；共享 harness 一行未改）": {
            "offboard_prestream:=true": ("给 DUT 参数补的。票 13 换到 mavros 后，DUT 只认 "
                                         "`mode == OFFBOARD`、PX4 又要先收到 setpoint 才进 "
                                         "OFFBOARD ⟹ 互等；实飞那档同样用 prestream 破局。"),
            "mode → nav_state 合成": ("在 `Dut.frames()` 上补的。票 13 之后 JSONL 写 `mode`（字符串）"
                                       "而没有 `nav_state`（整数），而判据② 的「不失控」读 "
                                       "`nav_state == 14` ⟹ 不补会结构性读成 False。"
                                       "只补判据层的键，原始 JSONL 不动。"),
            "FRAME_KEYS 追加 thrust / body_rate_* / mode": (
                "旧键 `thrust_body_z` 在新载体里不存在 ⟹ summary 的推力列会全空，"
                "而票 01 要从 `thrust` 换 `u_thrust_n`。"),
        },
        "模型生成的绕过（只在悬停那支）": {
            "build_hover_model.py": ("bag/3_traj1_hover 录到 TRACK 中途截断 ⟹ "
                                     "mocap_model.track_window() 返回零长窗口、"
                                     "noise_sigma() 抛 ValueError。包装只把窗口终点改成包尾，"
                                     "其余逐字复用 build_model()。"),
        },
        "中间件（与旧 B 档的不同）": {
            "mavros": "2.15.1（源码建于 ~/mavros_ws）",
            "fcu_url": "udp://:14540@127.0.0.1:14580",
            "launch": {str(Path("src/bringup/launch/mavros.launch.py")):
                       file_fingerprint(REPO / "src/bringup/launch/mavros.launch.py")},
            "thrust_scaling": ("1.0（mavros_set_thrust_scaling.py 读回核对过；见 "
                               "src/bringup/scripts/mavros_set_thrust_scaling.py）"),
            "ROS_DOMAIN_ID": 55,
        },
        "本场参数（obs 里的原样）": {
            "params": summary.get("params"),
            "hold_s": summary.get("hold_s"),
            "ref_hold_s": summary.get("ref_hold_s"),
            "ref_z_offset_m": summary.get("ref_z_offset_m"),
            "rate_hz": summary.get("rate_hz"),
        },
        "产物": {
            "summary": f"{EVID / tag / f'{tag}-summary.json'}（含 frames 与 truth；票 01 的输入）",
            "solver-summary": f"{EVID / tag / 'solver-summary.json'}",
            "console": f"{EVID / tag / f'{tag}-console.log'}",
            "原始 DUT JSONL": f"/tmp/phase5-sitl/{tag}/{tag}-dut.jsonl（**未入库**）",
        },
    }


def main() -> int:
    for tag, cfg in TAGS.items():
        out = REPO / EVID / tag
        out.mkdir(parents=True, exist_ok=True)
        ident = build(tag, cfg)
        (out / "harness-identity.json").write_text(
            json.dumps(ident, indent=1, ensure_ascii=False), encoding="utf-8")
        (out / "command.txt").write_text("\n".join([
            f"# {tag} —— 票 02 nominal SITL（reproduce）",
            "# 终端 A（先起中间件；与三趟共用）",
            MAVROS_LAUNCH,
            "",
            "# 终端 B（先 source 工作空间）",
            "source /opt/ros/humble/setup.bash",
            "source ~/mavros_ws/install/setup.bash",
            "source ~/px4_msgs_ws/install/setup.bash",
            "source install/setup.bash",
            "export ROS_DOMAIN_ID=55",
            ident["运行命令"]["跑本场"],
            "",
            "# 原始 DUT JSONL 抽求解汇总（跑完、删原始日志之前）",
            ident["运行命令"]["求解汇总抽取"],
            "",
            f"# 参考模型：{ident['参考模型']['path']}（sha256 {ident['参考模型']['sha256']}）",
            f"# 参考包：{ident['参考模型']['bag']}（逐字取自该包 /traj/reference）",
        ]) + "\n", encoding="utf-8")
        print(f"{tag}: harness-identity.json / command.txt 写好")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
