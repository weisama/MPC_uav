# 01: 目录顺延 —— `.scratch/phase3/` → `.scratch/phase4/`，并清扫引用

Status: done
Type: task

**做了什么（一句话）：** 把原来那个「Phase 3 = MPC（模型预测控制）实机飞行」的整层目录
**物理改名**成 `.scratch/phase4/`（票号 `01`–`11` 一个没动），把 `.scratch/phase3/` 这个名字
让给本阶段（整理 Phase 2 的产出 + 收尾），并把全仓库指向它的引用按口径扫了一遍。

**操作者裁定（2026-10-08）**：顺延本身 + 引用清扫的三条口径（见下）。

## 改了什么

| | 顺延前 | 顺延后 |
|---|---|---|
| 目录 | `.scratch/phase3/` | **`.scratch/phase4/`** |
| 12 个文件 | `spec.md` + `issues/01`–`11` | 同名同号，**内容只动路径**（`git` 认成 `R` 的有 **11 张票**；`spec.md` 例外，见下 §★） |
| `Status:` | —— | **一个字没改**（票面结论与标签保持原样） |

新增一页对照：**[`.scratch/phase4/README.md`](../../phase4/README.md)**（老↔新对照 + 清扫口径 + 票号标题清单）。

## 引用清扫的口径（操作者裁定；一句话＝**改活链接、留历史叙述**）

| 桶 | 是什么 | 行数 | 处置 |
|---|---|---|---|
| **路径** | `…/phase3/…` 形式的链接与代码里的路径 | 12 | **改** → `phase4/` |
| **活文档的散文** | `README.md` / `docs/` / `tools/` / `src/` / `run/` / `operation_manual.md` / `CONTEXT.md` 里的 `Phase 3` | 37 | **改** → `Phase 4`（作用是把读者指向接下来要做的事） |
| **`.scratch/phase1|phase2` 的票面散文** | 逐行过了一遍：34 行里 **32 行指向将来**（"给 Phase 4 的交接"、"Phase 4 要重编固件时"…）⟹ 改；**2 行是纯历史叙述**（[`phase1/issues/10`](../../phase1/issues/10-docs-1-14-sweep.md) 那张"这轮扫了哪些文档"的清单表）⟹ 留 | 34 | 见左 |
| **归档证据** | `tools/px4-sitl/evidence/tier-b/F*-summary.json` 里 `detail` 字符串中的 `Phase 3` | 9 | **留**（不就地改归档证据） |

★ 顺延时**操作者追加裁定**：票面里"指向将来"的说法也一并改 —— 否则会出现
「正文写着 Phase 3、链接却点进 phase4」的自相矛盾（第一手：`.scratch/phase2/issues/27` 有一行两者同行）。

★ 清扫时还发现**小写裸词 `phase3`**（不带斜杠，如 `.scratch/phase1/spec.md` 的 "phase3 的实机阶梯"）
是另一类写法，正则 `Phase 3` 与 `phase3/` 都盖不住 —— 已补扫 3 处；
`phase1/issues/10` 里**引用操作者 2026-09-23 原话**的那一行**留着不改**（引用就是引用）。

## 一手证据

```bash
# ① 目录是重命名而不是删+加
$ git status --porcelain   # 11 行 R  .scratch/phase3/issues/... -> .scratch/phase4/issues/...

# ② 指向 phase4 的相对链接能落地（脚本逐个 resolve）
$ 指向 phase4 的相对链接：23 条能落地
$ 唯一"落不了地"的一条：.scratch/phase1/issues/10-docs-1-14-sweep.md -> ../phase4/issues/04-...
#   ⚠️ 那一条**本来就是记录那条坏链的行**（原样也解析不到），不是新断的

# ③ 残留只应有三类（历史叙述 / 引用原话 / 归档 JSON）
$ git grep -n -I -i -e 'phase[ _]3' -e 'phase3' -- . | wc -l
#   4 行在 .scratch/phase1/issues/10-docs-1-14-sweep.md（2 行历史叙述 + 1 行引用原话 + 1 行坏链记录）
#   + 9 行在归档 F*-summary.json
```

★ 断链检查**只覆盖 markdown 相对链接**。代码/脚本里那两处路径由 `phase3/` → `phase4/`
那一遍覆盖（`tools/px4-sitl/z0-plant/plant_spec.py` 的字符串 `phase3/02`、
`tools/tau-analysis/` 两条 README 的链接）；真正的漏网是**裸小写 `phase3`**（不带斜杠）——
`Phase 3` 与 `phase3/` 两个正则都盖不住它，是**复查时才发现的**，补扫 3 处。

★ **`spec.md` 这一条，两种查法给出的答案不一样 —— 别只看 `git status`。**

- `git status --porcelain` 把它显示成 `M .scratch/phase3/spec.md` + `A .scratch/phase4/spec.md`
  （**看不出**是改名）——`status` 默认的改名检测**不带 copy 检测**；
- 而**提交自己的 `--stat`** 是这样记的（第一手，见 `e4ac56d` 的提交输出）：

  ```
  rewrite .scratch/phase3/spec.md (97%)
  copy    .scratch/{phase3 => phase4}/spec.md (98%)
  ```

  ⟹ 是 **rewrite + copy**（不是 rename，因为源路径没被删——同一路径当场被新的 Phase 3 spec 接走了）。
- 效果上没问题：`git log --oneline --follow -- .scratch/phase4/spec.md`
  **能一路追到 `2665d0a` / `243a835` …**（已第一手核过）。

★ 这一条**我自己先判错过一次**：只看了 `git status` 就写下"`--follow` 追不到旧历史"，
被提交输出当场证伪。教训与「'没有 X'取决于搜法」同族 —— **查法决定了答案**：
`git status` 的改名检测形式推不出 `--follow` 的行为。

## 验收

- [x] 目录已改名、票号原样、11 张票 `git` 认成 rename；`spec.md` 认成 **rewrite + copy**
      （两种查法的差别见上 ★）
- [x] 四桶口径按上表落地，行数对得上
- [x] `.scratch/phase4/README.md` 落地（含票号标题清单与"怎么按旧路径取回"）
- [x] 指向 `phase4` 的相对链接逐条 resolve 通过
- [x] 归档证据 JSON **一个字节没改**（`git diff` 里 `evidence/` 下无改动）

## Comments

### #1 这一票为什么是 `done`

动作本身在**本阶段开工的同一次提交**里就做完了（改名是机械动作，且后面所有票都要建在
`phase4` 这个名字上）。留着本票是为了让"为什么目录叫 Phase 4 而不是 Phase 3"这件事
**在 tracker 里有出处**，而不是只留在 commit message 里。
