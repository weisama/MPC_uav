# 10: `docs/` 里 "1.14" 表述清扫（Q36=b）

Status: ready-for-agent
Type: task

**What to build:** `docs/` 里仍残留**约 24 处**把 z0 说成 "PX4 1.14.x"、或引用 `release/1.14` 的陈述（分布在 `PROJECT.md`、`phase1-implementation.md`、`px4-interface.md`、`code-inventory.md`、`open-source-survey.md` 等）。

⚠️ **其中一部分是合法的** —— 指上游 v1.14.3 作版本对照 —— 所以需要**逐处甄别，不能一律替换**。spec 与 README 都把这件事点名交给 ticket 化步骤处理（Q36=b）。

**权威判定：** ☠️ **2026-09-21 本行整个反了，已重写 —— 见下。**

省事的做法是**先列一张处置表**，逐处写明「保留（理由）/ 改写（新表述）」，再动文本。处置表本身留档。

**★ 注意**：本张的结论不是"把所有 1.14 都删掉"。判据是**读者会不会把 z0 误读成跑 1.14** —— 会误读的才改。

## ☠️ 2026-09-21 重大更正：本张的**作业方向反了**（先读这段再动手）

本张原先的判据是「z0 = **v1.15 世代**，非法表述要改写成 `v1.15 世代 / fork 5ba2e91f`」。
**那个判据是错的**，源自 [ticket 11](11-firmware-version-conflict.md) 与
[`refs-manifest.md` §2.5(d)](../../../docs/reference/refs-manifest.md) 已定案的事实：

| | 原先（错） | **2026-09-21 定案（对）** |
|---|---|---|
| z0 固件真身 | fork `Peize-Liu/PX4-Autopilot @ 5ba2e91f`（v1.15 世代） | **`Minderring/PX4-Autopilot @ 4a0e65f2`**（**v1.14.3 系**的 MicoAir 构建） |
| 世代 | v1.15 | **v1.14.3 系** |
| `px4_msgs` 基准 | 「`release/1.14` 已实测否决」 | **`release/1.14` @ `ffb6e80e`**（与真身共有的 184 个 `msg/` 文件 blob 全同） |
| fork 的角色 | z0 的固件 | **与 z0 无关**（z0 不跑 fork） |

⟹ **所以本张要做的事正好相反：把当年被改错的表述改回 1.14.3 系，并把 fork 从"z0 的固件"降格为"一份无关的分支"。**
⚠️ **千万不要按原文的「非法的表述改写为 v1.15 世代」动手** —— 那会把文档改得更错。

**已污染、待本张清扫的落点（2026-09-21 grep 实测，非穷举）**：

- [`README.md`](../../../README.md)：第 4 行（"v1.15 世代"）、第 78/79 行（★ 那两行整块）
- [`docs/PROJECT.md`](../../../docs/PROJECT.md)：**第 437 行**（"固件版本标签错误" 那一行，判词整条反了）
- [`docs/control/phase2-implementation.md`](../../../docs/control/phase2-implementation.md)：第 587 行
- [`docs/reference/params-changes.md`](../../../docs/reference/params-changes.md)：第 14 行
- [`.scratch/phase2/spec.md`](../../phase2/spec.md)：第 298 行；[`.scratch/phase2/issues/06`](../../phase2/issues/06-mpc-node-fake-px4-fixture.md)：第 12 行
  （这两处是 "fork（v1.15 世代）" 的旁述，改的时候注意 M8 那个结论本身**仍然成立** —— `_v1` 话题机制在**真身**里同样不存在）

⚠️ **本张的一部分工作要等 ② 签字**：[`refs-manifest.md` §2.6](11-firmware-version-conflict.md) 的改法
（`px4_msgs` 基准 / SITL 被控对象）是**用户决策**，未签字前，
凡涉及"基准统一到谁"的表述**先列进处置表、不要落笔**。

**Blocked by:** 无阻塞项可立即开工（**清扫方向已被上面这段校正**）；仅"§2.6 相关表述"一栏等 ② 签字。

- [ ] 约 24 处**逐处列出**，每处标注保留或改写，并写明理由
- [ ] 合法的"**上游** v1.14.3 对照"表述保留，且不会让读者误读成"z0 跑上游 v1.14.3 原版"（真身是**厂商构建**）
- [ ] ☠️ **反了的旧判据改回来**：把 z0 表述为 **v1.14.3 系（真身 `4a0e65f2`）**，
      **删掉**"v1.15 世代"；fork `5ba2e91f` 降格为"与 z0 无关的分支"
- [ ] 上面那段"已污染落点"逐条清掉；`README.md:78/79`、`PROJECT.md:437` 是**整块的判词**，不是替换一个词就完事
- [ ] 改完后，`docs/` 里**没有再出现把 z0 与 fork `5ba2e91f` 绑定**的表述
      （原第 4 条要求的是"没有再出现把 z0 与 `release/1.14` 绑定"—— **那条要求本身反了**）
- [ ] 处置表留档（供复核），而不是只留下改后的文本
- [x] ~~若 [`01`](01-hardware-prereqs.md) 的 QGC 版本号读数已回来，与它核对一致~~
      → ✅ **已回来**（`flight_sw_version=0x010e0300` ⟹ `1.14.3`；`flight_custom_version` ⟹ `4a0e65f2`），
      **以它为准**，本张按上表改

---

**为什么这张放在 `phase1/issues/`**：它严格说**不属于 Phase 1 的实施工作**，是跨切面的文档债。放在这里只是因为当下 `.scratch/phase1/` 是唯一在用的 issue 集 —— 谁有空谁做，它不阻塞任何东西。
