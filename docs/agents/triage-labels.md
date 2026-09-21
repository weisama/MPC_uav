# Triage Labels

The skills speak in terms of five canonical triage roles. This file maps those roles to the strings this repo actually uses.

| Role in mattpocock/skills | String in our tracker | Meaning                                  |
| ------------------------- | --------------------- | ---------------------------------------- |
| `needs-triage`            | `needs-triage`        | Maintainer needs to evaluate this issue  |
| `needs-info`              | `needs-info`          | Waiting on reporter for more information |
| `ready-for-agent`         | `ready-for-agent`     | Fully specified, ready for an AFK agent  |
| `ready-for-human`         | `ready-for-human`     | Requires human implementation            |
| `wontfix`                 | `wontfix`             | Will not be actioned                     |
| —（上面五个角色之外，本仓库自加） | `done` | 该 ticket 的验收项**全部勾上**且结论已落进文档。**不是"没人管了"** —— 未决事项必须另开 ticket 并在本文里点名，不能靠 `done` 掩盖 |

When a skill mentions a role (e.g. "apply the AFK-ready triage label"), use the corresponding string from this table.

⚠️ **本仓库的 tracker 是本地 markdown，不是 GitHub Issues。** 这些字符串不进 GitHub label，
而是写进文件顶部的一行 `Status:`：

```markdown
# 03 verify offboard 模式切换时序

Status: needs-triage
Type: task
```

所以「打标签」这个动作 = 编辑该文件的那一行；**不要跑 `gh issue edit --add-label`**。
载体与路径约定见 `issue-tracker.md`。

**`spec.md` 同样带 `Status:` 行**（不带 `Type:`——spec 不是 ticket）。它的 H1 之后紧跟一行：

```markdown
# Phase 1 Spec —— offboard 位置定点飞行

Status: ready-for-agent
```

Edit the right-hand column to match whatever vocabulary you actually use.
