# 14: 清扫仍在指向「不存在的官方比对脚本」

Status: ready-for-agent
Type: task

**What to build:** [ticket 01](01-sitl-px4-msgs-baseline.md) 在 2026-09-28 **实测推翻**了
「PX4 官方比对脚本是 `Tools/check-message-compatibility.py`」这个说法，但只在**直接出处**
（[`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §S1 与 §6.1 末尾那条 S1 指令）
改了口径。**另外 2 个文件 3 处仍在原样引用那个路径** —— 照着它去找会**找不到文件**，
而更坏的是：**找到真身、直接跑，还会得到假 FAILED**（见下）。

**全部 3 处（已逐行核对，2026-09-28）**：

| # | 位置 | 现在写着 |
|---|---|---|
| 1 | [`PROJECT.md:450`](../../../docs/PROJECT.md) | 行尾「核对工具可用官方脚本 `./scripts/check-message-compatibility.py -v <px4_msgs> <PX4-Autopilot>`」 |
| 2 | [`px4-interface.md:178`](../../../docs/reference/px4-interface.md) | 代码块 `./scripts/check-message-compatibility.py -v <你的 px4_msgs 目录> <PX4-Autopilot 目录>`，上一行还写着「**（PX4 官方脚本）**」 |
| 3 | [`px4-interface.md:284`](../../../docs/reference/px4-interface.md) | 验收清单项「…跑过 `check-message-compatibility.py`」 |

**应该改成什么**（权威出处：[`docs/control/px4-sitl-baseline.md`](../../../docs/control/px4-sitl-baseline.md) §2.3）：

- ☠️ **`Tools/check-message-compatibility.py` 在 PX4-Autopilot 里不存在**。逐棵树查过
  v1.14.3 / v1.15.0 / v1.16.0 / fork `5ba2e91f` / `px4_msgs`，**全无**；`Tools/` 下只有
  `generate_msg_docs.py` / `px_generate_uorb_topic_files.py` / `px_generate_uorb_topic_helper.py`，
  **仓库根没有 `scripts/` 目录**。
- ★ **它的真身在 `auterion/px4-ros2-interface-lib` 的 `scripts/` 下**——而**直接跑它会报假 FAILED**：
  它的待查类型清单取自**它自己那个仓库**（`message_compatibility_check.hpp`），该仓**没有 `release/1.14` 分支**，
  拿它查 v1.14 这一对，会报 **20 个 v1.14 里根本不存在的类型**为「缺失」。
  报错原文里**没有** `Some files differ` 那一段 ⟹ **零个字段不符**。这一行 `FAILED!` 很容易被当真。
- **本项目该用的是 [`tools/px4-sitl/check_msg_compat.py`](../../../tools/px4-sitl/check_msg_compat.py)**：
  **复用官方那份 `message_hash` 算法**（从 `px4-ros2-interface-lib` 的 pin 住 commit 里 import，
  脚本会核验那个 pin、偏离就拒绝运行），但把类型换成两侧 `msg/` 的**并集（184 个全覆盖）**，
  **严格强于官方脚本的子集检查**。跑法见交付文档 §4。

**Blocked by:** 无。

- [ ] 上表 2 个文件 3 处**全部**对齐到新口径
- [ ] 按仓库纪律：**被推翻的原话用 ☠️ 标记保留**（写成「找错了地方」），**不静默删除** ——
      它在写下时是被当成"官方脚本"记的，这个误解本身值得留痕
- [ ] 每处都指向权威出处 `docs/control/px4-sitl-baseline.md` §2.3（或 §4 的跑法）
- [ ] 判据：`grep -rn 'check-message-compatibility' docs/ README.md` 的结果里，
      **未带 ☠️ 或 `~~` 标记**、且**不是**在指 `px4-ros2-interface-lib`/`check_msg_compat.py` 的引述，**归零**

## Comments

- **2026-09-28（agent，立项）**：`Status: ready-for-agent`。
  本票是 ticket [01](01-sitl-px4-msgs-baseline.md) 的**衍生物**：那张票收口时，
  它的 Comments 把「`PROJECT.md:450` / `px4-interface.md:178`、`:284` 仍写着错路径」列为遗留，
  并声明**不顺手清扫别的票**。按 [`triage-labels.md`](../../../docs/agents/triage-labels.md) 对 `done` 的定义
  （「未决事项必须另开 ticket 并在本文里点名」），此处就是那条遗留的承接票 —— 故它不是"顺带改别的票"，
  而是那张票按纪律交出来的东西。
  ⚠️ 与 [ticket 13](13-stale-acados-claims-sweep.md)（EOFError 清扫）**范围不重叠**：那张票只碰 `EOFError` 这一句话。
