#!/usr/bin/env python3
"""ticket 01 的验收脚本之一：`px4_msgs` 与 PX4 树的消息定义**逐类型比对**。

## 为什么不直接用官方那个脚本

ticket 01 的原文写的是「`Tools/check-message-compatibility.py`」，**这个路径不存在** ——
第一手核实（2026-09-28）：`PX4-Autopilot` 的 v1.14.3 / v1.15.0 / v1.16.0 / fork `5ba2e91f`
以及 `px4_msgs` 里**都没有**这个文件（`git ls-remote` 与本地三份 clone 逐棵树查过）。
它真正的位置是 **`auterion/px4-ros2-interface-lib` 的 `scripts/`**。

而**直接跑官方脚本会给出假阴性**：它的待查类型清单取自**它自己那个仓库**的
`px4_ros2_cpp/include/px4_ros2/components/message_compatibility_check.hpp`，
而该仓库**没有 `release/1.14` 分支**（只有 `release/1.5.1` / `1.16` / `1.17` / `1.18`）。
拿 main 的清单去查 v1.14 这一对，会报 20 个类型「缺失」——那 20 个**在 PX4 v1.14.3 里根本不存在**
（`RegisterExtComponent*` / `GotoSetpoint` / `Rover*` / `FixedWing*Setpoint` 等，都是更新版才加的）。
**那不是不兼容，是"v1.14 里没有这个东西"。**

## 本脚本做什么

**复用官方脚本的算法**（`message_hash`：按字段结构递归算 FNV-1a，忽略注释与格式），
但把待查类型换成 **两侧 `msg/` 目录里全部 `.msg` 的并集** —— 严格强于官方脚本的子集检查。
所以本脚本的 PASS/FAIL 与官方脚本不是同一件事：这里 **PASS 才真的说明"每个共有的消息对得上"**。

判据：
- 两侧都有的类型 → 逐类型比 hash，**全部相同才算过**（`--must-cover` 里的类型会单独打印，挡住"悄悄漏查"）
- 只在一侧有的类型 → **不作为失败**，但全部列出（这是文件集合差异，v1.14 这一对已知只有 `CMakeLists.txt`）

用法：
    python3 tools/px4-sitl/check_msg_compat.py \\
        --px4-tree  ~/refs/px4-v1.14.3-build \\
        --px4-msgs  ~/refs/px4_msgs \\
        --interface-lib ~/refs/px4-ros2-interface-lib

⚠️ `--interface-lib` 只用来**借它那份 `message_hash` 实现**（本脚本不抄一份算法自己维护，
那样"官方算法"这个说法就断了）。仓库外只读副本，见 `docs/reference/refs-manifest.md` §1。
"""

from __future__ import annotations

import argparse
import importlib.util
import subprocess
import sys
from pathlib import Path

# ★ 钉住的上游 commit —— 与 docs/reference/refs-manifest.md §1 记的是同一行。
#   ⚠️ 这个 pin 是**承重的**：`message_hash` 是借来的，它在上游会随版本改动。
#   若 `--interface-lib` 不在这个 commit 上，本脚本**默认直接拒绝运行** ——
#   否则"复用官方算法、不另写免得漂移"这句话就是空的（PASS 会用一份没钉住的算法算出来）。
INTERFACE_LIB_COMMIT = "ac2e8aeecb22de8efb29738b173e1ade5d54432b"

# ★ ticket 01 点名要覆盖的类型。「已知它是被破坏的那个」指的是早期误判
#   （拿 fork 的 msg/ 比出来的），现已作废；但这条**显式覆盖**的要求本身仍然有效。
MUST_COVER = ("OffboardControlMode", "VehicleRatesSetpoint")


def git_head(repo: Path) -> str | None:
    """读 repo 的 HEAD commit；不是 git 仓就返回 None。"""
    try:
        out = subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"],
                             capture_output=True, text=True, check=True)
    except (OSError, subprocess.CalledProcessError):
        return None
    return out.stdout.strip()


def load_official_hash(interface_lib: Path, allow_unpinned: bool = False):
    """从官方脚本里取出 `message_hash` —— 不另写一份算法。

    ★ 同时**核验 `interface_lib` 就在 `INTERFACE_LIB_COMMIT` 上**：`message_hash` 是从上游借的，
    它会随上游改动而变。不核验的话，这份 PASS/FAIL 就是拿一份未钉住的算法算出来的，
    而"复用官方算法"这个保证正是靠这个 pin 成立的。
    """
    script = interface_lib / "scripts" / "check-message-compatibility.py"
    if not script.is_file():
        sys.exit(f"找不到官方脚本：{script}\n（用 --interface-lib 指到 px4-ros2-interface-lib 的检出目录）")

    head = git_head(interface_lib)
    if head is None:
        print(f"⚠️  {interface_lib} 不是 git 仓 —— 无法核验算法版本，"
              f"下面的结论**不保证**出自 {INTERFACE_LIB_COMMIT[:12]}", file=sys.stderr)
    elif head != INTERFACE_LIB_COMMIT:
        msg = (f"{interface_lib} 在 {head}，不是钉住的 {INTERFACE_LIB_COMMIT}\n"
               f"（message_hash 会随上游改动而变 ⟹ 结果不可比。"
               f"要么 checkout 到钉住的 commit，要么加 --allow-unpinned-algorithm 承担风险）")
        if not allow_unpinned:
            sys.exit(f"❌ {msg}")
        print(f"⚠️  继续执行，但已偏离 pin：{msg}", file=sys.stderr)
    else:
        print(f"算法来源   : px4-ros2-interface-lib @ {head[:12]}（= 钉住的 commit）")

    spec = importlib.util.spec_from_file_location("_official_compat", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.message_hash


def msg_names(msg_dir: Path) -> set[str]:
    return {p.stem for p in msg_dir.glob("*.msg")}


def main() -> int:
    parser = argparse.ArgumentParser(description="PX4 树 vs px4_msgs：逐类型消息字段比对")
    parser.add_argument("--px4-tree", required=True, type=Path,
                        help="含 msg/ 的 PX4-Autopilot 检出目录")
    parser.add_argument("--px4-msgs", required=True, type=Path,
                        help="含 msg/ 的 px4_msgs 检出目录")
    parser.add_argument("--interface-lib", required=True, type=Path,
                        help="px4-ros2-interface-lib 检出目录（只为借它的 message_hash）")
    parser.add_argument("--limit", type=int, default=25,
                        help="最多打印几条差异明细（默认 25）")
    parser.add_argument("--allow-unpinned-algorithm", action="store_true",
                        help="★ message_hash 借自上游，默认要求 --interface-lib 恰在钉住的 commit 上；"
                             "本开关在偏离时只警告不退出（结论的可比性由你自己承担）")
    args = parser.parse_args()

    a_dir = args.px4_msgs / "msg"
    b_dir = args.px4_tree / "msg"
    for d in (a_dir, b_dir):
        if not d.is_dir():
            sys.exit(f"不是 msg/ 目录：{d}")

    message_hash = load_official_hash(args.interface_lib, args.allow_unpinned_algorithm)

    only_msgs = msg_names(a_dir) - msg_names(b_dir)
    only_tree = msg_names(b_dir) - msg_names(a_dir)
    shared = sorted(msg_names(a_dir) & msg_names(b_dir))

    print(f"px4_msgs   : {args.px4_msgs}  ({len(msg_names(a_dir))} 个 .msg)")
    print(f"PX4 树     : {args.px4_tree}  ({len(msg_names(b_dir))} 个 .msg)")
    print(f"共有       : {len(shared)} 个 → 逐个比字段布局")
    print()

    mismatched: list[str] = []
    errored: list[tuple[str, str]] = []
    for name in shared:
        try:
            # ⚠️ 传 msg/ 目录本身而不是仓库根：官方 main() 传仓库根，靠 os.walk 找文件，
            #    对 PX4 这棵 1.2 GB 的树会每个类型都走一遍全树。传 msg/ 等价且快得多。
            if message_hash(name, str(a_dir)) != message_hash(name, str(b_dir)):
                mismatched.append(name)
        except Exception as exc:  # 解析失败本身也是要暴露的事实，不能当成"通过"
            errored.append((name, f"{type(exc).__name__}: {exc}"))

    print(f"字段布局不同的 : {len(mismatched)} 个")
    for name in mismatched[: args.limit]:
        print(f"    - {name}.msg")
    print(f"解析失败的     : {len(errored)} 个")
    for name, why in errored[: args.limit]:
        print(f"    - {name}.msg  ({why})")

    print()
    print("只在一侧有的（**不计为失败**，但必须列出来）")
    print(f"  只在 px4_msgs 里 : {', '.join(sorted(only_msgs)) or '（无）'}")
    print(f"  只在 PX4 树里    : {', '.join(sorted(only_tree)) or '（无）'}")

    print()
    print("ticket 01 点名要覆盖的类型")
    for name in MUST_COVER:
        if name in shared:
            same = name not in mismatched and name not in {n for n, _ in errored}
            print(f"    {name:<24} 在共有集里，字段{'一致' if same else '★ 不一致'}")
        else:
            print(f"    {name:<24} ★ 不在共有集里（只在 "
                  f"{'px4_msgs' if name in only_msgs else 'PX4 树' if name in only_tree else '?'}）")

    ok = not mismatched and not errored
    print()
    print("RESULT: " + ("PASS —— 共有的每个消息字段布局都相同" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
