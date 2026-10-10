# 03: `docs/` 索引状态与 `☠️` 作废块压缩

Status: done
Type: task

**What's wrong（一句话）：** `docs/` 里作废的旧结论**没有被删掉，只是被划掉** ——
读者要读完整段划线原文才知道"这里作废了什么、现在该信哪个"；而 `docs/` 的索引与状态
也没跟上（有文档自称的状态与各票 `Status:` 行对不上）。

**为什么它排在 [02](02-entry-layer-trim-and-status-correction.md) 之后**：先把入口层那张
"该读什么"的地图弄准，再动正文；否则正文裁得再干净也没人找得到（见 spec Solution 甲）。

## 目标形态（一句话）

**读 `docs/` 的任一节，只看节首几行，就能知道"这里原来写的什么、被谁推翻了、现在该看哪"** ——
不必读划掉的原文。

## 要做的两件事

### ① `☠️` 作废块压缩

**形式**（spec Solution 甲·2）：

- **保留节首的结论行** —— 含**逐项点名**"被推翻的是哪几项"；
- **删掉被划掉（`~~…~~`）的正文** —— 它是"当时的原话"，不是"现在要读的东西"；
- **每一处删除都要带去向标记**：
  - 被推翻的 → 「**已被 X 推翻**（指向推翻处：文件 / 票号 / §N）」；
  - 换个地方放的 → 「**已搬进 §N**」。

★ **没有去向标记的删除一律算没做完。** 这与"作废标记要放被作废那一节的**节首**并逐项点名"
是同一条纪律（排序就是可见性 —— 读者不会为一句已作废的话读完一整节）。

★ **`☠️` 不等于作废块**：这一层里有两种写法，**别一刀切**。

- **(甲) 作废块** —— `☠️` 与 `~~…~~` 同时出现（真正要压的那种）；
- **(乙) 只是个警示标签** —— 单独一个 `☠️` 领起一句**今天仍然成立**的警告（"别再这么干"），
  **保留原样**，只是确认它不需要压缩。

口径：**先按 (甲) 筛，再对 (乙) 逐条确认它确实还成立。**

### ② `docs/` 的索引与状态跟上

- `docs/PROJECT.md` 的总纲、各文档自述的状态，**与 `.scratch/*/issues/` 里各票的 `Status:` 行对得上**；
- 对不上的：**以票面的 `Status:` 行为准**（那是唯一权威），改文档里那句，不改票；
- 文档里写的阶段编号必须是**顺延后**的（`phase4/`，不是 `phase3/`）。

## 范围与不做什么

- **只在 `docs/` 与入口层四份里动**（入口层那四份归 [02](02-entry-layer-trim-and-status-correction.md) 管，
  本票不重做一遍）；
- ❌ **`.scratch/**` 的票面内容一律不动**（Q4 / Q19）——票里那些 `☠️` 块**保持原样**，
  它们是"当时不知道什么"的记录，正是 tracker 该留的东西；
- ❌ **不新增文档层**：本阶段的目标是**减**。压缩优先，合并次之，新增不允许；
- ❌ **不删"还成立"的内容**。本票治的是"**作废的旧结论没清掉**"，不是"文档太长"。

## 规模（快照，**别引用这个数，用下面的命令现算**）

```bash
# 全仓库 docs/ 里 ☠️ 的行数与分布
$ git grep -c -I '☠️' -- docs/ | sort -t: -k2 -rn
#   ⟹ 20 个文件、159 行（2026-10-08 快照）

# 拆成两种写法（见上文 ★）
$ git grep -n -I '☠️' -- docs/ | grep -c '~~'      # (甲) 真作废块：33 行
$ git grep -n -I '☠️' -- docs/ | grep -vc '~~'     # (乙) 警示标签：126 行
```

★ 这张表**只是工作量估计**，不是判据。压缩过程中行数会变，判据是下面那三条。

## 验收

> ☠️ **2026-10-09 补勾（本票落地时漏了这一步）**：本票在 `9380dd3` 已切 `Status: done`，但**这五条框一直没勾**
> —— 证据当时写进了 Comments #4「落地记录」，那儿的副本是 `[x]`，正本却是 `[ ]`。
> 这就是[「票面有两份验收清单」](../../../docs/agents/triage-labels.md)那个坑：**只信副本就会漏**。
> 现按 Comments #4 的读数逐条勾上（**勾的是那条记录，不是重新跑了一遍**）。
> ⚠️ 第 3 条的读数**有一条已知的后续漂移**：`docs/control/phase2-implementation.md` §S5 末尾
> 「状态停在 `ready-for-human`」那句话，是**2026-10-09 收口 phase2 那 9 张票时新产生的**不一致 ——
> 已在同一轮就地加 ☠️ 块订正（见该处）。

- [x] 抽检每一处 (甲) 类作废块：**只看节首**，能回答出"被谁推翻 / 搬到哪"（Solution 甲·3）—— 见 Comments #4
- [x] 全 `docs/` 里**不再有"被划掉的正文而无去向标记"**的节 —— 见 Comments #4
- [x] `docs/` 里的状态自述与各票 `Status:` 行**一致**（不一致的按票面改文档）—— 落地时 5 处订正（Comments #4）；
      ★ 2026-10-09 收口 phase2 那 9 张票后又漂移出 **1 处**，已就地订正
- [x] 文档里不再指向 `phase3/`（实机那层已顺延为 `phase4/`，见 [01](01-directory-renumber-and-link-sweep.md)）
      —— `grep -rn -E 'phase3/|Phase ?3' docs/` ⟹ **0**
- [x] 按文件跑文档守卫全绿：
      `python3 -m pytest run/tests/test_operation_manual.py -q`
      ★ **别一次跑 `run/tests/` 整个目录** —— `test_z0_check_live.py` 的模块级 `importorskip`
      会让其余文件一条都不收集而**退出码仍是 0**（静默假绿），见
      [`phase2/issues/26`](../../phase2/issues/26-recording-six-scenes-and-the-rewrite-path.md) 的同名记录
      —— 落地时实测 `test_operation_manual.py` **5** / `test_z0_menu.py` **66** / `test_rec.py` **80**

## Comments

### #1 为什么"压缩"而不是"删除整节"

因为那些 `☠️` 块里**有一半是判断依据，不是结论** —— 比如"当时为什么排除固件 fork `5ba2e91f`"
（[`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.5(d)）**今天仍然有效**，删了下次会重踩。
要删的只是**被推翻的那句结论本身**，以及为它服务的那段划线原文；
"**为什么推翻**"必须留，而且必须留在**节首**。

### #2 顺手核到的一手事实：`docs/` 里**本来就有**一批断链（**不是**顺延那次弄断的）

写 [01](01-directory-renumber-and-link-sweep.md) 时做全仓库断链检查，拿 **HEAD 那棵树**做对照，
结论是**断链总数 26 → 27**，多出来的唯一一条是本票自己写错又当场改掉的路径
（`.scratch/phase3/issues/` → `../../../docs/reference/`）。⟹ 其余 26 条**全是顺延之前就断的**。

```bash
# HEAD 那棵树（顺延之前）的断链
$ git archive HEAD | tar -x -C /tmp/linkchk && cd /tmp/linkchk && python3 <断链检查>
#   ⟹ 26 条；工作树 27 条，差值 1 = 本票自己写错的那条
```

`docs/` 里那 **13 条**在本票范围内（"索引与状态跟上"那条）。**两类**病因：

| 文件 | 条数 | 病因 |
|---|---|---|
| [`mismatch-sweep.md`](../../../docs/control/mismatch-sweep.md) | **9** | 8 条**只写了文件名**（`09-sitl-tier-a-closed-loop.md` `:153`、`11-sitl-tier-b-seam3.md` `:195/:269/:349/:755/:769`、`08-sitl-z0-airframe.md` `:800`、`20-vertical-channel-mismatch-offset.md` `:1007`）—— 漏了 `.scratch/phase2/issues/` 前缀；第 9 条 `:1099` 是 `../../.scratch/phase2/issues/05-*.md`，**带 `*` 通配**，解析不到具体文件（**这条可能是有意的**，指向"05 那一批"，修之前先看语境） |
| [`mocap-setup.md`](../../../docs/reference/mocap-setup.md) | **3** | `../../../` **多了一层**（`:309` 的 `.scratch/phase1/issues/13-…`、`:982` 与 `:1083` 的 `src/vrpn_listener/VENDORED.md`）。同文件 `:59`/`:79` 写的是 `../../…`，**是对的** ⟹ 是同一份文档里两种写法并存 |
| [`sitl-offboard-smoke.md`](../../../docs/control/sitl-offboard-smoke.md) | **1** | 同第一类：`:367` 的 `06-mpc-node-fake-px4-fixture.md` 只写了文件名 |

★ 这些**不在** [01](01-directory-renumber-and-link-sweep.md) 的清扫口径里（那条只覆盖"指向 `phase3/`"的），
所以留给本票 —— 修的时候注意：**目标文件都还在**，是**路径写法**错了，不是"文件没了"。

### #3 与 02 的互不重叠

两票都动"入口层/文档"，边界是**改什么**：

| | 02 | 03 |
|---|---|---|
| 动哪 | 入口层四份（`CLAUDE.md` / `README.md` / `docs/PROJECT.md` / `CONTEXT.md`） | `docs/` 其余文档（`control/` `reference/` `agents/`） |
| 治什么 | **阅读路径 + 状态词**（"该读什么"说不清） | **作废块 + 状态自述**（"读完才知道作废了"） |

03 做完时 `docs/PROJECT.md` 会被两边都碰一下（02 改它的索引指针，03 改它的状态自述）——
**按这个切分，同一节不要两边各改一次**。

### #4 落地记录（2026-10-08）

**① 作废块压缩 —— 口径**

按票面 ★：先筛 (甲)（`☠️` 与 `~~…~~` 同行），再对 (乙)（孤 `☠️` 领起、今天仍成立的警告）逐条确认它确实还成立。

- 删的是**被划掉的正文**；留的是**"被推翻的是哪一项"的短标签** + 节首判词 + **去向**。
- 特例：**已结案/已撤销条目的短标签**（风险名、`O2`、决定名）连同旁边的 ✅ 状态与指针**保留** ——
  那是"逐项点名"的载体，删了就没人知道被推翻的是哪一项（见 Comment #1：判断依据要留）。

现算（不是快照）：

```bash
$ grep -rn '~~' docs/ | wc -l          # 剩 13 行 —— 全是上面那类短标签
$ grep -rn '☠️' docs/ | grep -c '~~'   # 剩 3 行，(甲) 类 = 0
$ grep -rn '☠️' docs/ | grep -vc '~~'  # 166 行，(乙) 逐条确认后原样保留
```

这 13 行每一条都带 ✅【已撤销 / 已结案 / 已关闭 / 已收敛 / 已推翻】或 `☠️`+理由，**无一条是"划线正文而无去向"**。

**② 索引与状态跟上 —— 订正 5 处**（每处都回一手核过；不一致一律按票面 `Status:` 改文档）

| 文档 | 原写 | 现按 |
|---|---|---|
| `phase2-implementation.md` §S1 | 「仍未复验：SITL 机型条目数」 | ✅ 实测 **56** 个条目（`ls ~/refs/px4-v1.14.3-build/ROMFS/px4fmu_common/init.d-posix/airframes/ \| wc -l`） |
| `acados-orin-build.md` §6 | 「z0 自己那一份的 P3 仍未结／待实测」 | ✅ 2026-09-28 由 `phase2/issues/04`（`done`）回填：纯求解 max 3.58 ms ⟹ D6 定 100 Hz |
| `px4-interface.md` 延迟表 | 「MPC 求解 … 待实测」 | 同上，行内带锚（★ 2026-10-10 清扫后该格首词已写"模型预测控制"） |
| `mpc-roadmap.md` §9 摘要 | 「D6 待实测」 | **D6 已定 100 Hz**（2026-09-28） |
| `phase2-implementation.md` §S4 清单 | 「（D6 待实测后调）」 | 「（D6 已定 100 Hz）」 |

**断链（Comment #2 那 13 条）—— 全修**

- 漏前缀的 10 条补 `../../.scratch/phase2/issues/`（`mismatch-sweep.md` 9 条 + `sitl-offboard-smoke.md` 1 条）；
- `mismatch-sweep.md:1099` 的 `05-*.md` 通配：按语境（"修不修属 ticket 08 / 05 的票面"）定为**指向 `05` 那一批** ⟹ `05-sitl-offboard-smoke.md`；
- `mocap-setup.md` 3 条：去掉多出的一层 `../`（`:982` 的**显示文本**也一并改成 `../../…`，同文件 `:59`/`:79` 的写法）。

`docs/` 内断链现为 **0**（`python3 <断链检查>` 在 `docs/` 下零命中；余下 13 条都在 `bag/` 与 `.scratch/`，不在本票范围）。

**复核（两轴）与我驳掉的**

- Standards 轴报出 **2 条真的锚指错** —— 已修：`tier-b-sitl-seam3.md` 引 `mismatch-sweep.md` §3（2026-09-29 订正在 **§2.8.1**）；`acados-orin-build.md` 把 **D6 行**算进 §5（D6 在 **§3.1**，P3 才在 §5）。
- Spec 轴报「24 处引文 >60 字、12 处 >100 字」—— **按放大数字驳回**：逐条重量为 **55 / 96 / 64 / 79** 字，非 165 / 164 / 154 / 149；剩的 ≥80 字引文全是**条目身份**（commit hash、分支名、两个 ADC 通道号）或已用「……」省略，正是要留的"逐项点名"。
- Spec 轴报「`sign-map.md:345`『由 ticket 10 定稿』与票面不符」—— **驳回**：`phase2/issues/10` 现为 `ready-for-human`（**未结**），条文与票面一致。
- Spec 轴「`3.58 ms` 抄进 4 处」—— **保留**：每处都带锚，且它就是 D6 定档的依据本身；README 那句「本格不抄读数」只约束它自己那张进度表。
- 采纳并已改的还有：`mpc-roadmap.md:314` 的作废项补上文件/§ 指针；`params-changes.md` A4-附注那条裸「已作废」补上推翻处（§A4 / `params-analysis.md` §3.5.1）；`tier-b-sitl-seam3.md` 那段 `/tmp` 订正**由 4 行压到 2 行**（本阶段目标是"减"，不加长）。

**范围外留痕（本票只动 `docs/`，不改代码）**

`tools/px4-sitl/verify_tier_b_seam3.py:78`、`:1516` 与 `tools/px4-sitl/mismatch_sweep.py:1372` 仍写「`/tmp` 是 tmpfs」
（**机制错、结论对** —— 实测根盘 `ext4` + `tmpfiles.d/tmp.conf:11` 开机清；同 `docs/control/tier-b-sitl-seam3.md` 的 2026-10-08 订正）。代码不在本票范围，留给下一张。

**验收**

- [x] 抽检 (甲)：**只看节首**即可回答"被谁推翻 / 搬到哪"（`refs-manifest.md` §2.5、`params-analysis.md` §3.5.1、`open-source-survey.md` §9.3、`code-inventory.md` A.6、`bench-runbook.md`「☠️ 已删」等）
- [x] 全 `docs/` 无"被划掉的正文而无去向标记"的节
- [x] `docs/` 状态自述与各票 `Status:` 行一致（5 处订正，见上）
- [x] 文档不再指向 `phase3/`（`grep -rn -E 'phase3/|Phase ?3' docs/` ⟹ **0**）
- [x] 守卫**按文件**全绿：`test_operation_manual.py` **5** / `test_z0_menu.py` **66** / `test_rec.py` **80**
