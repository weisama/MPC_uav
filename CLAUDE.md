# MPC_uav —— z0 无人机自主飞行栈

> 飞控 MicoAir NxtPX4v2（PX4 1.14.x，飞控自带）+ Jetson Orin NX 8GB（密码123） + Nokov 动捕（VRPN）。
> 目标：**MPC 接管飞控内置 PID**，分两阶段落地。

**先读**：`README.md`（两阶段路线 + 关键约束）→ `docs/PROJECT.md`（总纲）
→ `docs/control/phase1-implementation.md`（动手前必读）。

⚠️ 本项目**不是代码优先，是文档优先**：结论大多带第一手证据（源码路径、行号、commit）。
**改代码前先确认文档里的结论有没有被推翻**，不要凭记忆。`docs/reference/open-source-survey.md`
§0 记了两条早期被误导的教训，值得先看。

## Agent skills

### Issue tracker

Issues 与 spec 存在仓库内的 **`.scratch/`**（本地 markdown，**随 git 入库**）。
不用 GitHub Issues，不用 `gh`。See `docs/agents/issue-tracker.md`.

### Triage labels

五个标准角色，标签名即角色名（`needs-triage` / `needs-info` / `ready-for-agent` /
`ready-for-human` / `wontfix`）。See `docs/agents/triage-labels.md`.

### Domain docs

单 context。术语表在仓库根 **`CONTEXT.md`**。See `docs/agents/domain.md`.
