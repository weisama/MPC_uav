# Domain Docs

How the engineering skills should consume this repo's domain documentation when exploring the codebase.

## Before exploring, read these

- **`docs/CONTEXT.md`** —— ⚠️ 本仓库的术语表在 `docs/` 下，**不在根目录**。
  含 `MPC` 歧义澄清（PX4 的 `MPC_*` 参数前缀是 Multicopter Position Controller（级联 PID），
  与本项目的 Model Predictive Control 毫无关系）。
- **`docs/adr/`**：目录尚不存在，按需由 `/mattpocock-skills:domain-modeling` 惰性创建。
  （`docs/PROJECT.md` 是 ADR 风格的，但它是总纲，不替代 `docs/adr/`。）

If any of these files don't exist, **proceed silently**. Don't flag their absence; don't suggest creating them upfront. The `/mattpocock-skills:domain-modeling` skill (reached via `/mattpocock-skills:grill-with-docs` and `/mattpocock-skills:improve-codebase-architecture`) creates them lazily when terms or decisions actually get resolved.

## 本仓库的文档地图

| 路径 | 性质 | 什么时候读 |
|---|---|---|
| `README.md` | 两阶段路线 + 关键约束摘要 | 最先 |
| `docs/PROJECT.md` | ★ 总纲（ADR 风格，含开题记录） | 任何改动之前 |
| `docs/CONTEXT.md` | 术语表 | 命名/措辞有疑问时 |
| `docs/control/phase1-implementation.md` | ★★ Phase 1 实施清单 | 写 Phase 1 代码前 |
| `docs/control/phase2-implementation.md` | ★★ Phase 2 实施清单 | 写 Phase 2 代码前 |
| `.scratch/phase1/spec.md` | Phase 1 spec（本地 tracker；已入库） | 查 Phase 1 的范围与验收判据 |
| `.scratch/phase2/spec.md` | Phase 2 spec（本地 tracker；已入库） | 查 Phase 2 的范围与验收判据 |
| `docs/control/mpc-roadmap.md` | 资料汇总（**非决策**） | 追 D1–D10 决策沿革 |
| `docs/reference/` | **事实调查**（每个结论带第一手证据） | 结论有疑问时**回这里复核** |
| `docs/assets/` | `z0.params`、6 篇论文 PDF | 查参数 / 查论文 |

⚠️ **本仓库的硬规矩**：`docs/reference/` 与两份实施清单里的结论**大多标注了第一手来源
（源码路径、行号、commit）**。改代码时若发现与文档冲突，**先复核来源，不要直接改文档**。

## File structure

Single-context repo（本仓库）:

```
/
├── CLAUDE.md
├── .scratch/                          ← 本地 issue tracker（spec = <feature>/spec.md，ticket = <feature>/issues/NN-*.md）
│                                      ⚠️ 随 git 入库，不是可随手删的临时目录
├── docs/
│   ├── CONTEXT.md                     ← 术语表在这里，不在根目录
│   ├── adr/                           ← 尚未创建，按需惰性生成
│   ├── control/                       ← 实施清单（怎么落地）
│   └── reference/                     ← 事实调查（查证据）
└── src/                               ← ROS 2 代码，Phase 1 待建
```

## Use the glossary's vocabulary

When your output names a domain concept (in an issue title, a refactor proposal, a hypothesis, a test name), use the term as defined in `docs/CONTEXT.md`. Don't drift to synonyms the glossary explicitly avoids.

If the concept you need isn't in the glossary yet, that's a signal: either you're inventing language the project doesn't use (reconsider) or there's a real gap (note it for `/mattpocock-skills:domain-modeling`).

## Flag ADR conflicts

If your output contradicts an existing ADR, surface it explicitly rather than silently overriding:

> _Contradicts ADR-0007 (event-sourced orders), but worth reopening because…_
