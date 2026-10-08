# 05: 把 `T0` 从 0.5 m 八字接到 1.0 m 的新包（含生成 `mocap-model-5.json`）

Status: ready-for-agent
Type: task
**对着 Phase 2 的：[`27`](../../phase2/issues/27-lemniscate-doubled-and-downstream-labels.md)**（`ready-for-human`）
**阻塞：** 无 · **被阻塞：** [06](06-fly-t0-seam3-four-criteria.md)

**要做什么（一句话）：** 判据② 判的是**当前规格那条 1.0 m 八字**，但 `T0` 那一场现在接的
还是 **0.5 m** 的旧包 —— 把它接到 `bag/5_traj3_lemniscate`，并生成配套的回放模型。

## 这个接线断了在哪（一手，2026-10-08 核过）

`bag/` 整理那次（`38d55e3`）把工具的默认路径**按路径**改指了 `bag/old/…` —— 改对了，
那份包确实搬走了。**但没有跟着把语义换成 1.0 m**，于是现在是"路径对、内容错"：

| 处 | 现在指着 |
|---|---|
| [`run_matrix.sh`](../../../tools/px4-sitl/evidence/tier-b/run_matrix.sh) 的 `T0)` 行 | `--mismatch mass+30`、`--model …/mocap-model-4.json`、traj 参考仍取 **0.5 m** 包 |
| [`verify_tier_b_seam3.py:1570`](../../../tools/px4-sitl/verify_tier_b_seam3.py#L1570) | `--reference-source traj` 的默认包 = `bag/old/4_traj3_lemniscate` |
| `verify_tier_b_seam3.py:1475` | 报错里给的建议路径同上 |

★ **回放节点吃的参考/噪声/时序全部来自那份 JSON**（`mocap_model.py` 的 `cmd_dump`；
回放节点**不自己量包**）⟹ 不生成新模型，1.0 m 那条参考根本不会发出去。

## 要做的三件事

### ① 生成 `mocap-model-5.json`（入库）

```bash
python3 tools/px4-sitl/mocap_model.py dump bag/5_traj3_lemniscate \
    --noise-bag bag/1_static \
    --json tools/px4-sitl/evidence/tier-b/mocap-model-5.json
```

- 位置就挑 `evidence/tier-b/` —— `mocap-model-3.json` / `mocap-model-4.json` 都入库在那儿（`git ls-files` 可核）；
- 生成完 **把它加进 git**：它是 `T0` 那一场唯一的输入，落在 `/tmp` 会过不了重启
  （根盘 ext4 + tmpfiles 一条 `D /tmp`）。

### ② 两处默认路径改指新包

`verify_tier_b_seam3.py` 的 `--reference-source traj` 默认包与
`run_matrix.sh` 的 `T0)` 行，都从 `bag/old/4_traj3_lemniscate` / `mocap-model-4.json`
换成 `bag/5_traj3_lemniscate` / `mocap-model-5.json`。

### ③ ★ `--hold-s` 的下界要跟着重算（别照抄 60）

下界 = `ref_hold_s`（顶住首点等飞机起来）+ **模型里量出的参考全长**（`reference.span_s`）
+ 判据① 的稳态窗口 `STEADY_S`，**三段串起来**，少一截的症状是判据① 的稳态窗口里
混进八字或爬升段 —— 读出来像"悬停精度很差"，而控制器没问题。

| 包 | `span_s` | 下界 | `T0` 该给 |
|---|---|---|---|
| 旧（0.5 m，包 4） | 20.56 s | 30 + 20.56 + 8 = **58.56 s** | 60（`run_matrix.sh:108-109` 现在的注释就是这条） |
| 新（1.0 m，包 5） | ≈ **26.22 s** | 30 + 26.22 + 8 = **≈ 64.22 s** | **65 或 66**（给一拍余量） |

⚠️ `run_matrix.sh:105-107` 还写着 `--model` **必须**是包 4 那份 —— 那条理由（"包 3 的参考恒为 0"）
换包之后要改成对包 5 的说法，别把旧句子留着。

## 验收

- [ ] `mocap-model-5.json` 已生成、**已入库**，且里面有非零的 `/traj/reference`
      （判据：包 3 那份 `traj_model_problem()` 判非空 = "参考恒为 0，该被拦住"，
      包 5 必须判 `None`；这条现在是 `test_tier_b_judges.py` 里替包 4 写的，**照样式补包 5**）
- [ ] `run_matrix.sh` 的 `T0)` 行与 `verify_tier_b_seam3.py` 的默认包都指向 1.0 m 那份；
      `--hold-s` 按上表重算过
- [ ] ★ [`test_tier_b_judges.py::test_hold_s太短就拦住_下界从模型现算`](../../../tools/px4-sitl/test_tier_b_judges.py#L467)
      **会红** —— 它逐字断言 `hold_s=60.0` 通过（换成包 5 后 `need≈64.22`）。
      **改法是让它从包 5 现算**，不是删掉那条断言、也不是把 60 抄成 66 了事
- [ ] `verify_tier_b_seam3.py` 里 **15 处**还写着 `r = 0.5 m 八字` 的文案
      （`:9 :15 :393 :920 :996 :999 :1003 :1007 :1014 :1017 :1044 :1049 :1350 :1475 :1570`）
      与新规格对不上了 —— 逐处改成 1.0 m / 新包名
- [ ] `docstring` 与实际接线**一致**：判据标签里写"1.0 m"，脚本默认就真的是 1.0 m
- [ ] 不跑整场 `T0`（那是 [06](06-fly-t0-seam3-four-criteria.md)）——
      但要用 `--dry-run` 之类**不占窗口**的方式确认参考名与长度已经换过来了

## 边界

- ⚠️ **不动** phase2/27 票面里那张「有意不动」表 —— 它是历史叙述。票面里
  "`tools/px4-sitl/*` 与 `docs/control/tier-b-sitl-seam3.md` 的 `r = 0.5 m` 标签有意不动"
  这一条，**在本票落地后就成了过期说法**；在 phase2/27 的 `## Comments` 里补一句说明即可，
  **不要就地改那张表**（[别顺手改别的票]）。
- ⚠️ **归档证据不就地改** —— 但 `mocap-model-5.json` 是**新的**归档，不是改旧的。
- 本票不收口 phase2/27 的 `Status:`。

## Comments

### #1 为什么它 `ready-for-agent`

纯离线接线：跑一次 `mocap_model.py dump`、改两处默认路径、重算一个下界、改一批文案与一条测试。
不碰飞机、不占算力窗口、判据可复跑（生成物能 `git` 核、测试能红能绿）。
★ 本票**只接线不判判决**——"判据② 过没过"是 [06](06-fly-t0-seam3-four-criteria.md) 的事，
所以它才敢标 `ready-for-agent`；而它在 phase2 的母票（`27`）是 `ready-for-human`，两者不矛盾。
