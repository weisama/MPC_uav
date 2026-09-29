# 13: 清扫仍在流传的「Tera 缺失报 `EOFError`」

Status: done
Type: task

**What to build:** ticket [02](02-acados-orin-build.md) 在 2026-09-28 **实测推翻**了
「缺 Tera 渲染器会报 `EOFError`」，但只在**直接出处**（[`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) §S2、
[`mpc-roadmap.md`](../../../docs/control/mpc-roadmap.md) §5）改了口径。
**另外 5 个文件仍在原样重复这句话** —— 照它去排查会找错方向（去搜 `EOFError`，而那个字符串在 acados 全史里都不存在）。

**全部 6 处（已逐行核对，2026-09-28）**：

| # | 位置 | 现在写着 |
|---|---|---|
| 1 | [`hardware.md:278`](../../../docs/reference/hardware.md) | `- 缺 Tera 渲染器会报 EOFError` |
| 2 | [`code-inventory.md:239`](../../../docs/reference/code-inventory.md) | `…、**缺 Tera 渲染器会报 EOFError**、必须设 ACADOS_SOURCE_DIR…` |
| 3 | [`PROJECT.md:408`](../../../docs/PROJECT.md) | `…、Tera 渲染器缺失会报 EOFError` |
| 4 | [`PROJECT.md:453`](../../../docs/PROJECT.md) | `无官方 Jetson 文档；t_renderer 缺失报 EOFError` |
| 5 | [`open-source-survey.md:61`](../../../docs/reference/open-source-survey.md) | `报 EOFError —— 这是 acados 最常见的坑` |
| 6 | [`CONTEXT.md:93`](../../../CONTEXT.md) | `…模板引擎二进制，**安装缺失时报 EOFError**，是 acados 最常见的坑。` |

★ **`hardware.md:278` 与 `open-source-survey.md:61` 是「acados 要点」那两节，是后来者最先看到的地方 —— 优先改。**

**应该改成什么**（权威出处：[`docs/control/acados-orin-build.md`](../../../docs/control/acados-orin-build.md) §4）：

- ☠️ 「报 `EOFError`」是 **v0.5.x 及以前**的事：那时 acados 会 `input('Do you wish to set up Tera renderer
  automatically?')` 交互式提问，非交互运行读到 EOF ⟹ Python 内建 `EOFError`。commit `a26c7d0ce` 删掉了该提问，
  **首发 v0.6.0** ⟹ 我们用的版本不会再有。
- ★ **该记的不是"报什么异常"，而是三种落点**：缺失+有网 → 自己下载（只慢）；
  缺失+无网 → `RuntimeError`，信息完整**直指根因**；**在、但渲染不出东西 → 完全不报错**，
  静默两层后才炸成 `OSError: …libacados_ocp_solver_….so: cannot open shared object file`
  （`utils.py:396-397` 用 `os.system` 只查退出码、不校验产物）。
  **真正坑人的是那一种静默的，不是异常类型。**

**Blocked by:** 无。

- [x] 上表 5 个文件 6 处**全部**对齐到新口径
- [x] 按仓库纪律：**被推翻的原话用 ☠️ 标记保留**（写成「换了版本，这句话不再成立」），**不静默删除** —— 它在 v0.5.x 时代是**对的**
- [x] 每处都指向权威出处 `docs/control/acados-orin-build.md` §4
- [x] 判据：`grep -rn "EOFError" docs/ README.md CONTEXT.md` 的结果里，**未带 ☠️ 标记的引述归零**
      （即：不再有任何一处把 `EOFError` 当作"会发生的症状"来写）

## Comments

- **2026-09-28（agent，立项）**：`Status: ready-for-agent`。
  本票是 ticket [02](02-acados-orin-build.md) 的**衍生物**：那张票收 `done` 时，它的遗留表里留了
  「别处还写着被推翻的话」这一条。按 `docs/agents/triage-labels.md`，
  **`done` 不允许靠"留给下一轮文档清扫"掩盖未决事项 —— 必须另开票并在本文里点名。**
  所以这张票存在的意义是**让那个"以后"有个编号**，而不是"顺手把别的票改掉"。

  ★ **为什么当时不直接改**：那几处不在 02 的范围内（02 是「acados 装起来」），
  按仓库纪律「收口一张票时不要顺带改另一张」，当时只在 02 的 Comments 与遗留表里**点出来**，没有自己拿主意。

- **2026-09-28（agent，★ 立项时把范围点错了，已改正 —— 记下来因为这类错很容易重犯）**：
  本票初稿写的是「三处」「`GENERIC` 掉一个量级」与「OpenMP 冲突」这两句也在别处」，并列了
  `hardware.md:276`、`open-source-survey.md:59` 两行。**逐行核对后两条都不成立**：

  | 初稿的说法 | 核对结果 |
  |---|---|
  | 「`GENERIC` 掉一个量级」在别处也有 | ❌ **只存在于 `phase2-implementation.md:444` 一处**。`hardware.md:276` 与 `open-source-survey.md:59` 只写「acados/BLASFEO 没有 A78 目标 → 用 A76」，**没有量级声明，本来就是对的** |
  | 「OpenMP 冲突」在别处也有 | ❌ **全仓库只有 `phase2-implementation.md:444` 一处**（`grep -rn ACADOS_WITH_OPENMP docs/ README.md` 在别处为空）—— `mpc-roadmap.md` §5 的原始表格里**根本没有 OpenMP 这一行** |
  | 三处 | ❌ 实为 **5 个文件 6 处**，漏了 `PROJECT.md:453` 与 `code-inventory.md:239` |

  **教训**：这两句"被推翻的话"其实**都被关在施工图 §S2 一个地方**，`grep` 一下就清楚 ——
  我却是靠记忆列的清单。**列"还有哪儿写着"这种清单，必须 `grep` 出来，不能凭印象。**
  （这也解释了为什么 02 当时只是"不越界"，而不是"漏了一大片"。）

- **2026-09-29（agent，收口）**：`Status: done`。全部验收项勾选。

  **实际改了 6 处**（票面表格 5 个文件 6 处，含本票立项时漏进表的 `CONTEXT.md`）：

  | file:line | 备注 |
  |---|---|
  | `docs/reference/hardware.md:293` | ★「acados 要点」节，优先改 |
  | `docs/reference/open-source-survey.md:61` | ★「Jetson 要点」表行，优先改 |
  | `docs/reference/code-inventory.md:239` | Orin NX 要点表行 |
  | `docs/PROJECT.md:413` | 词汇表行 |
  | `docs/PROJECT.md:458` | 风险表行 |
  | `CONTEXT.md:93` | ★ 仓库根术语表（非 `docs/` 下），见下方方法论 |

  六处一律：**原话用 ☠️ 保留**、指向权威出处 `docs/control/acados-orin-build.md` §4、
  重点落在「**三种落点，其中「在、但渲染不出东西」完全静默**」（而非「报什么异常」）；
  表格行用 inline code / ☠️ 前缀，`|` 语法未破坏。

  **判据原始输出**（`grep -rn "EOFError" docs/ README.md CONTEXT.md`）：
  6 处目标命中现已**全部带 ☠️**；剩余**未带 ☠️** 的命中**只出现在权威出处本身**
  `docs/control/acados-orin-build.md`（第 80/87/104/106/109/114/115/119/120/273 行）——
  那是讨论/推翻这句话的**源文档**，不是引述；第 80 行的表行虽无 ☠️，但同一格里就写着
  「**在 v0.6.0 上不成立**」。票面自身不在本判据的 grep 范围内（在 `.scratch/` 下）。
  ⟹ **判据通过：「把 `EOFError` 当症状写」的引述归零。**

  ★ **方法论一条（`CONTEXT.md` 这处的由来）**：票面原判据的 grep 形式是
  `grep -rn "EOFError" docs/ README.md` —— **照不到仓库根的 `CONTEXT.md`**，
  所以施工第一轮 5 处改完后判据"按字面通过"，但 `CONTEXT.md:93` 这第 6 处仍原样漏网。
  **换宽搜法**（把 `CONTEXT.md` 加进 grep 形式）才命中它；而它一旦命中，
  票面的「5 个文件 6 处」就从**自相矛盾**（表里只列 5 行）**升级成自洽**
  （5 文件 = `hardware.md` / `code-inventory.md` / `PROJECT.md` / `open-source-survey.md` / `CONTEXT.md`；
  6 处 = 前四者 5 行 + `CONTEXT.md` 1 行）—— **票面的计数本来是对的，漏的是表格里的一行**，
  本票已补上该行。这是「**缺席论证的强度等于你写下的那个 grep 形式**」的又一例：
  原形式的"只剩权威出处"是**真的**，但它证明的范围也**只有 `docs/` 与 `README.md`**。

  ★ **如实点名一处"不动的"**：`docs/control/acados-orin-build.md:273` 的遗留表也登记了本处清扫，
  它写「这句话还在 **5 个文件**里原样重复」，却**只枚举了 4 个**
  （`hardware.md` / `code-inventory.md` / `PROJECT.md` / `open-source-survey.md`）——
  **同样漏了 `CONTEXT.md`**。那行是**那份文档自己的账**，按仓库纪律「收口一张票时不要顺带改另一张」，
  **本票不改它**，仅在此点名，提醒后续读者：该行枚举与 2026-09-29 的事实已不一致。

  **行号漂移**（票面 2026-09-28 快照 → 2026-09-29 实测）：`hardware.md:278→293`、
  `PROJECT.md:408→413`、`PROJECT.md:453→458`；`code-inventory.md:239` 与 `open-source-survey.md:61` 未漂。

- **2026-09-29（agent，复核轮补记）**：`code-review` 轮抓出**一处方向性损失**，已补回。

  清扫把「**`t_renderer` 必须装**」这半句**一起扫掉了**：`code-inventory.md` 与 `PROJECT.md`
  两处变成「缺 Tera 渲染器**不再报** `EOFError`」—— 读起来像「缺了也没事」。而权威出处
  [`acados-orin-build.md`](../../../docs/control/acados-orin-build.md) §3 的表格自己写的是
  「**必须装这件事不变**；**症状与「真因」都要改写**」：该改的是**症状**，不是**要不要装**。

  ⚠️ **为什么容易漏**：原文（`- 缺 Tera 渲染器会报 \`EOFError\``）**并没有**明写「必须装」三个字，
  它的"必须装"是**靠症状隐含**的（缺了会报错 ⟹ 所以得装）。换口径时保留了「作废」那半句、
  丢掉了**仍然有效的那半句** —— 而这一票的标题恰恰是「清扫**仍在流传的**」，
  扫完变成「不再是个问题」就是**扫过头**。

  ⟹ 已在 **5 个文件 6 处**补回「★ **但「必须装」这件事不变**」：
  `hardware.md:293`、`open-source-survey.md:61`、`code-inventory.md:239`、
  `PROJECT.md:413`/`:458`、`CONTEXT.md:93`。

  ★ **教训（与本仓库既有规矩同源）**：作废标记要**保留"仍然有效的那半句"**，不能只留"作废"。
  一句话里若「症状」与「动作」绑在一起，**只作废症状时要把动作单独写出来**，
  否则读者会连动作一起丢掉。
