# Phase 4 —— 本目录原先是「Phase 3」，2026-10-08 顺延为 Phase 4

> **这不是一张票，是这一层目录的说明页。** 体例同 [`../phase1/DELETED.md`](../phase1/DELETED.md)：
> 文件住在 `.scratch/` 里，但它记的是"**这个目录是怎么来的**"，不是待办。
> 待办一律看各票的 `Status:` 行与 [`spec.md`](spec.md)。

## 改了什么

| | 顺延前 | 顺延后 |
|---|---|---|
| 目录 | `.scratch/phase3/` | **`.scratch/phase4/`** |
| 票号 | `01`–`11` | **原样保留**（一个号都没动） |
| 票面内容 | MPC（模型预测控制）上实机 | **`Status:` 与正文一个字没改**（只动了指向别处的**路径**，见下） |

**为什么顺延**（操作者裁定，2026-10-08）：Phase 2（模型预测控制接管 PID，SITL 验证）
留下了一批**没做完的收尾工作**，而在直接上实机之前需要先把它们收干净。于是：

- **新的 Phase 3** = **整理 Phase 2 的产出**（裁掉不必要、总结必要的）+ **收口 Phase 2 未完成的工作**
  → [`.scratch/phase3/spec.md`](../phase3/spec.md)；
- **Phase 4** = 原来那个「MPC 上实机」 → **本目录**。

⟹ **读任何 2026-10-08 之前写的文档时，遇到「Phase 3」就要读成"现在的 Phase 4"**（实机飞行），
**不是**现在的这个 `.scratch/phase3/`。下面那节说明哪些地方已经改过来了、哪些是**故意留着**的。

## 这次落地时"引用清扫"的口径

一句话：**改活链接、留历史叙述**。逐桶（行数按改动前数）：

| 桶 | 是什么 | 处置 |
|---|---|---|
| **路径** | `…/phase3/…` 形式的链接与代码里的路径 | **改** → `phase4/` |
| **活文档的散文** | `README.md` / `docs/` / `tools/` / `src/` / `run/` / `operation_manual.md` / `CONTEXT.md` 里的 `Phase 3` —— 作用是把读者**指向接下来要做的事** | **改** → `Phase 4` |
| **`.scratch/phase1|phase2` 票面里的散文** | 逐行过了一遍：34 行里 **32 行是指向将来的**（"给 Phase 4 的交接"、"Phase 4 要重编固件时"、"直接交 Phase 4 上机"…）⟹ **改**；**2 行是纯历史叙述**（[`phase1/issues/10`](../phase1/issues/10-docs-1-14-sweep.md) 里那张"这一轮扫了哪些文档"的清单表）⟹ **留** | 见左 |
| **归档证据** | `tools/px4-sitl/evidence/tier-b/F*-summary.json` 里 `detail` 字符串中的 `Phase 3` | **留**（归档证据不就地改，改了就不是那一趟跑的留存了） |

★ **本目录的票面本身**（`spec.md` + 11 张票）里的自指也一并改成了 `Phase 4` —— 它自己就是那一阶段。
★ 唯一还留着旧字样的地方：上面那张清单表的 2 行，以及
[`phase1/issues/10`](../phase1/issues/10-docs-1-14-sweep.md) 里**引用操作者 2026-09-23 原话**的那一行
（引用就是引用，不改）。

## 票号对照（顺延前后同号，标题也没变）

| 票 | 标题 |
|---|---|
| 01 | 装桨前台架几何测量 —— 电机实际位置 + 动捕刚体↔IMU 外参 |
| 02 | 悬停油门回填 PX4 并与 MPC（模型预测控制）模型核对自洽 |
| 03 | 从 Phase 1 飞行日志量出端到端延迟 τ |
| 04 | 固件重编重刷（**修法待重定**）+ 版本终审 + 刷前/刷后参数 diff |
| 05 | 首飞配置冻结 —— PX4 参数集 + MPC（模型预测控制）节点参数 |
| 06 | 起飞前写定四级阶梯的量化判据 |
| 07 | Phase 2 的 seam 1 用实机参数重跑 |
| 08 | 安全路径实机实测 + 回退 (A) 级演练 + FSM 逐条审计 |
| 09 | 阶梯 1+2 —— 拆桨看方向 → 系绳 + 离地 20 cm 只给悬停推力 |
| 10 | 阶梯 3+4 —— 系绳闭环悬停 30 s → 自由飞三条轨迹 |
| 11 | τ 的跨飞行复现 + B 口径验证（需另一段日志） |

## 怎么按旧路径取回内容

`.scratch/` 是入库的，改名这件事在 git 历史里留下了痕迹。**11 张票**都被 git 认成 `R`（rename），
历史是续着的；`spec.md` 是 **rewrite + copy**（见下）。要按旧路径读某个文件：

```bash
git log --oneline --follow -- .scratch/phase4/spec.md   # 能追回 2665d0a / 243a835 …
git log --oneline --follow -- .scratch/phase4/issues/11-tau-cross-flight-reproduction.md
git show <提交>^:.scratch/phase3/issues/11-tau-cross-flight-reproduction.md   # 旧路径读改名前的版本
```

★ 也可以在**顺延之前**那棵树上直接读 —— 那棵树是**裸提交 `a119a02`**（2026-10-08，正是顺延那一提交 `e9c62ee` 的**父提交**）。
⚠️ 它原先叫 `v2.0` **标签**，而这个名字 **2026-10-12 已改指 Phase 2 收口点的分支**（`175e298`）⟹ **按名字取会拿到错的树，而且不报错**：

```bash
git show a119a02:.scratch/phase3/spec.md     # ← 这就是"旧的 Phase 3 spec"，即现在的 phase4/spec.md
```

⚠️ **`spec.md` 在两种查法下看到的答案不一样，别只看 `git status`**：
`git status` 把它显示成 `M .scratch/phase3/spec.md` + `A .scratch/phase4/spec.md`（**看不出**是改名 ——
`status` 默认的改名检测**不带 copy 检测**）；而落地那次提交（`e4ac56d`）的 `--stat` 记的是

```
rewrite .scratch/phase3/spec.md (97%)
copy    .scratch/{phase3 => phase4}/spec.md (98%)
```

—— 因为**同一个路径当场被新的 Phase 3 spec 接走了**，源路径没被删，所以是 **rewrite + copy**、
而不是 rename。效果上没问题：`git log --follow` 照样能追。
（这一条是落地时第一手核到的；★ 我先只读了 `git status` 就判"追不到旧历史"，被提交输出当场证伪。）
