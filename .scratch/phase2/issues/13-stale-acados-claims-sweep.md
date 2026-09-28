# 13: 清扫仍在流传的「Tera 缺失报 `EOFError`」

Status: ready-for-agent
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

- [ ] 上表 5 个文件 6 处**全部**对齐到新口径
- [ ] 按仓库纪律：**被推翻的原话用 ☠️ 标记保留**（写成「换了版本，这句话不再成立」），**不静默删除** —— 它在 v0.5.x 时代是**对的**
- [ ] 每处都指向权威出处 `docs/control/acados-orin-build.md` §4
- [ ] 判据：`grep -rn "EOFError" docs/ README.md` 的结果里，**未带 ☠️ 标记的引述归零**
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
