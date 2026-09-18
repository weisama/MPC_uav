# Triage Labels

The skills speak in terms of five canonical triage roles. This file maps those roles to the strings this repo actually uses.

| Role in mattpocock/skills | String in our tracker | Meaning                                  |
| ------------------------- | --------------------- | ---------------------------------------- |
| `needs-triage`            | `needs-triage`        | Maintainer needs to evaluate this issue  |
| `needs-info`              | `needs-info`          | Waiting on reporter for more information |
| `ready-for-agent`         | `ready-for-agent`     | Fully specified, ready for an AFK agent  |
| `ready-for-human`         | `ready-for-human`     | Requires human implementation            |
| `wontfix`                 | `wontfix`             | Will not be actioned                     |

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
