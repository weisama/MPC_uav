# 05: 把 `T0` 从 0.5 m 八字接到 1.0 m 的新包（含生成 `mocap-model-5.json`）

Status: done
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

- [x] `mocap-model-5.json` 已生成、**已入库**，且里面有非零的 `/traj/reference`
      （判据：包 3 那份 `traj_model_problem()` 判非空 = "参考恒为 0，该被拦住"，
      包 5 必须判 `None`；这条现在是 `test_tier_b_judges.py` 里替包 4 写的，**照样式补包 5**）
      → 生成命令即票面那条（`dump bag/5_traj3_lemniscate --noise-bag bag/1_static`）。
      **实测**：`n=1312`、`span_s=26.2236`、参考包络 **2.000 × 1.000 m**（`mocap_model.py` 现算，
      印证票面 §③ 那张表）。测试扩成 `test_入库的三份模型各判各的`：包 3 拦、包 4 / 包 5 放行
- [x] `run_matrix.sh` 的 `T0)` 行与 `verify_tier_b_seam3.py` 的默认包都指向 1.0 m 那份；
      `--hold-s` 按上表重算过
      → `--model …/mocap-model-5.json`、`--hold-s` **60 → 66**（下界 30 + 26.2236 + 8 = **64.22**，
      `math.ceil` = 65 再留一拍）；验证器默认包 `bag/old/4_traj3_lemniscate` →
      **`bag/5_traj3_lemniscate`**（☠️ 新包在 `bag/` 一层、**不带 `old/`**，那行注释也一并订正）
- [x] ★ [`test_tier_b_judges.py::test_hold_s太短就拦住_下界从模型现算`](../../../tools/px4-sitl/test_tier_b_judges.py#L467)
      **会红** —— 它逐字断言 `hold_s=60.0` 通过（换成包 5 后 `need≈64.22`）。
      **改法是让它从包 5 现算**，不是删掉那条断言、也不是把 60 抄成 66 了事
      → **`--model` / `--hold-s` 改成从 `run_matrix.sh` 的 `T0)` 行现取**，参考全长从**那个模型**现量。
      ☠️ 为什么不改成硬写 `math.ceil(need)`：那会把"脚本那一行"这条守卫**整个丢掉**（见 Comments #2）
- [x] `verify_tier_b_seam3.py` 里 **15 处**还写着 `r = 0.5 m 八字` 的文案
      （`:9 :15 :393 :920 :996 :999 :1003 :1007 :1014 :1017 :1044 :1049 :1350 :1475 :1570`）
      与新规格对不上了 —— 逐处改成 1.0 m / 新包名
      → ☠️ **票面那串行号与"15 处"都已过期**（行号取自更早的版本，且两种东西混在一起数）。
      按**现文件**逐处扫完：**10 处** `r = 0.5 m 八字` 文案 + **5 处** `bag/old/4_traj3_lemniscate` 路径
      （另有 1 处票面没列的 `--model …/mocap-model-4.json` 建议路径）。**逐处点名见 Comments #2**
- [x] `docstring` 与实际接线**一致**：判据标签里写"1.0 m"，脚本默认就真的是 1.0 m
      → 并加了一条**离线断言**钉着它：`test_T0接的是1_0m那份八字_不是0_5m那份`
      （从接线指向那份模型直接量参考包络 = 2.000 × 1.000 m；0.5 m 那份只有 1.000 × 0.500 m）
- [x] 不跑整场 `T0`（那是 [06](06-fly-t0-seam3-four-criteria.md)）——
      但要用 `--dry-run` 之类**不占窗口**的方式确认参考名与长度已经换过来了
      → ⚠️ 本验证器**没有 `--dry-run` 开关**（`grep -n "dry.run" verify_tier_b_seam3.py` ⟹ 无）。
      改用两法**都在起世界之前**：(a) 上面那两条离线测试（读接线 + 读模型，不启 SITL）；
      (b) 直接 import 验证器调 `traj_model_problem()` 读接线 —— 读数见 Comments #2

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

### #2 2026-10-09 落地记录 —— 票面那串行号已过期；两条新断言都主动弄坏过

**① 「15 处」与那串行号都已过期**（票面写于 `7a25e17`，此后 `5ee6140` 动过这个文件 ⟹ 行号漂了）。
按**现文件**重扫，票面列的 15 个行号其实是**两种东西混在一起**，逐处点名：

| 类别 | 票面列的行号（当时） | 现在的行号 | 处置 |
|---|---|---|---|
| `r = 0.5 m 八字` 文案 | `:9 :15 :920 :996 :999 :1007 :1014 :1044 :1049 :1350`（**10 处**） | `:9 :15 :931 :1007 :1010 :1018 :1025 :1055 :1060 :1361` | ✅ 全改 `1.0 m` |
| `bag/old/4_traj3_lemniscate` 路径 | `:393 :1003 :1017 :1475 :1570`（**5 处**） | `:391 :1014 :1028 :1487 :1583` | ✅ 全改 `bag/5_traj3_lemniscate`（**不带 `old/`**） |
| ☠️ **票面漏列的** | —— | `:1485` 的 `--model …/mocap-model-4.json` | ✅ 一并改（同一条报错建议里的，不改就自相矛盾） |

★ 教训（与本阶段要治的是同一个病）：**票面里的行号/处数是"写下来的时点"的快照**，
拿它当清单去改，会**漏掉一处还数错一次**。改成按**语义**扫（`grep -n "r *= *0\.5\|0\.5 m 八字\|4_traj3_lemniscate"`），
改完再 `grep` 一遍确认归零。

**② 两条新断言都主动弄坏过**（"新断言要主动弄坏一次"那条纪律）：

| 变异 | 期望 | 实测 |
|---|---|---|
| `run_matrix.sh` 的 `--hold-s 66` → `60` | 红 | ✅ 红，且消息直接点名：`--hold-s 60 撑不住 mocap-model-5.json（下界 64.22 s：30 + 26.22 + 8）` |
| `T0)` 的 `--model` → `mocap-model-4.json` | 红 | ✅ 红：`` `T0` 接的参考包络 x = 1.000 m ⟹ 不是 1.0 m 那条八字`` |

**③ 离线复核读数**（不占窗口、不启 SITL；验证器**没有 `--dry-run` 开关**，见验收最后一条）：

```
--model    tools/px4-sitl/evidence/tier-b/mocap-model-5.json     （从 T0) 行现取）
--hold-s   66.0                                                  （从 T0) 行现取）
参考包络   2.000 x 1.000 m ; n = 1312 ; span_s = 26.2236
下界       64.22 s (30 + 26.2236 + 稳态窗口 8) ; 接线给 66  ⟹ pass
traj_model_problem(接线那份)          -> None
traj_model_problem(包 3，恒为 0)      -> 拦住
traj_model_problem(包 4，0.5 m)       -> None  ⟸ ☠️ 见下
```

**④ ☠️ 顺手量出来的一个边界（票面没提，记在这里免得下次重新踩）**：
起世界之前那道 `traj_model_problem()` 闸**只拦"参考恒为 0"**（拿包 3），**拦不住**
"拿包 4 那份 0.5 m 冒充当前规格" —— 旧包参考非空、只是短一半，`hold-s 66` 也够它（下界 58.56）。
⟹ 谁要是手工 `--model mocap-model-4.json`，报告读起来**一切正常**。
挡这件事的是**离线**的 `test_T0接的是1_0m那份八字_不是0_5m那份`（直接量接线指向那份模型的包络）。
**要不要把"包络必须是 2.0 × 1.0 m"也做成运行时闸，留给操作者裁定** —— 那会给验证器引入一个
新的写死数（规格一变就得跟着改），本票**没做**（票面没要求，且属"改验证器行为"）。

**⑤ 一并落地的相邻文案**（都不在票面验收里，但同族、留着就是过期说法）：

- `docs/control/tier-b-sitl-seam3.md` §3.4 / §6.1 / §6.2 / §6.4 —— 这是 phase2/27
  Comments #3 点名的"**本票收口时的连带改动**"，逐处加了带日期的订正块；
- `tools/px4-sitl/mocap_replay.py` 的用法示例（"判据② 的八字（包 4）"→ 包 5）与"包 4 的 `z ≡ 0`"一句；
- `tools/px4-sitl/mocap_model.py` 的 `TRAJ_TOPIC` ☠️ 实测例（补上包 5 的读数）；
- `tools/px4-sitl/evidence/tier-b/README.md` 的文件表（补 `mocap-model-4/5` 两行 —— 之前只列了 3）；
- `docs/control/phase2-implementation.md` §S5 那两条判据下方的 2026-09-29 段（"重放的是包里那条 0.5 m"）
  —— ☠️ **这一处是 `/code-review` 复核时才扫出来的**（票面与 Comments #2 都没列），同族、同日订正。
- ⚠️ **`mocap_replay.py` / `test_mocap_model.py` 里那些把"包 4"当**具体测量对象**的句子没动**
  （"包 4 的尾巴 `p ≡ 0`"之类）—— 它们说的是**那份包的事实**，不是当前规格，改了反而变错。
- ⚠️ **未动** phase2/27 票面那张「有意不动」表（按票面边界），仅在它的 `## Comments` 追加 #4 说明。

### #3 2026-10-09 `/code-review` 复核 —— 逐条回代码验过，三条采纳、两条驳回

按 [复核意见也当假设验] 那条纪律，每条都回文件核过再决定落不落地：

| 复核意见 | 核实结果 | 处置 |
|---|---|---|
| `mocap_model.py` 把新增的**包 5 读数**挂在"见 11 号票 Comments 2026-09-30"上，而 11 号票里没有包 5（`grep 1312` 零命中） | ✅ 属实 —— 引证与被引证的东西对不上 | ✅ 改：包 3/包 4 仍指 11 号票，**包 5 单指本票现 dump 的那份 JSON** |
| `docs/control/phase2-implementation.md` §S5 还写着 SITL "重放的是包里那条 **0.5 m**"、且"得重录一次八字场…" | ✅ 属实且**已过期**（包 5 就是那次重录，`T0` 已接上） | ✅ 改：加 2026-10-09 带日期的订正块 |
| `mocap_replay.py:448` 的 `hold` 段理由引的是**包 4 的数**（帧 305..1028）却**没标包 4**，而它就在 `T0` 现在走的这条路上 | ✅ 属实（同文件 `:406` 那条标了包 4，这条没标） | ✅ 改：换成**包 5 现量**的帧 610..1311、t = 12.20→26.22 s（尾段 `p ≡ 0 且 v ≡ 0` 对包 5 同样成立，已逐帧核过） |
| `test_tier_b_judges.py` 的 `lemniscate()` 夹具幅度由 0.5 → 1.0 属**票面没要求的改动** | ⚠️ 属实（确实没写在验收里） | ❌ 保留：它**建模的是当前规格那条轨迹**（`traj_obs` 那族判据测试共用），留 0.5 会让夹具与判据标签互相矛盾 —— 这条**故意**的，记在这里备查 |
| 本票 `Status: done` 与 phase2/27 不动 `Status` 是"两种处理" | ⚠️ 属实但**不矛盾** | ❌ 不改：phase3 的 01/02/03 就是 agent 全勾后收口成 `done`；`27` 是 `ready-for-human`（**操作者保留**），票面 §边界也明写"本票不收口 27 的 `Status`" |

★ 另有两条**复核确认无误**（不是意见，是回验）：`--hold-s 66 ≥ 下界 64.22`（从 `mocap-model-5.json` 现算）；
`mocap-model-5.json` 的 `n=1312`、`span_s=26.2236`、包络 2.000 × 1.000 m 与票面每个数字都对得上。
☠️ 复核**没找到**、但落地时才补上的一处：`mocap-model-5.json` 当时还是 untracked —— 票面 §① 明写"生成完加进 git"，
**这一步在提交里兑现**（[散文里写死的数会过期] 的另一面：状态词要带可核的锚）。
