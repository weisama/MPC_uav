# 02: 悬停油门回填 PX4 并与 MPC（模型预测控制）模型核对自洽

Status: ready-for-human
Type: task

**What to build:** 把**已经测出来的**悬停油门同时落到 PX4 与 MPC（模型预测控制）模型两处，并留下一份「两处确实一致」的核对记录。

**Blocked by:** 跨 feature 硬依赖 —— [`phase2/issues/10-param-measure-backfill.md`](../../phase2/issues/10-param-measure-backfill.md)。悬停油门只能从 Phase 1 的悬停日志反推，那一步在 Phase 2 的 `ready-for-human` 票里。**本票不重新测量，只做落地与核对。**

**为什么必须同时落两处**：PX4 对 `thrust_body` **不做任何重标定**。只改一处 = 一个**静默的推力增益错误** —— 语法完全合法的消息，PX4 照做，我只有从飞行日志里才能分辨是哪一环错了。

- [ ] `MPC_THR_HOVER` 写成**实测值**（现值 `0.5` 恰好是 PX4 默认值，看不出被改过）
- [x] 模型里的 hover thrust 与推力归一化常数**逐项一致**（改动清单里的 M4 那一类），核对记录留档
- [x] 核对记录写明：两个值各是什么、从哪个实测数字来、**为什么它们必须相等**
- [x] 与 `THR_MDL_FAC = 0.45` 的关系一并复核 —— 它是非默认值，说明这组参数被人动过，而它是推力模型的关键参数
- [x] 若实测值与理论推算（`√(187.5/1020) ≈ 42.9%`）差得多，**记录差异**而不是取平均
- [x] 改动带编号与理由，写进 [`phase2-implementation.md`](../../../docs/control/phase2-implementation.md) 并从本文件回链

---

**为什么是 `ready-for-human`**：改飞控参数要在 QGC 里做，且「哪个读数才是可信的」是人的判断。

*⚠️ spec 层约束：Phase 4 不与 Phase 1 / Phase 2 并行开工。*

---

## Comments

### 2026-10-09 —— 落地与核对做完；**只剩"写进 QGC"这一步（第 1 框，人的动作）**

**核对记录（本票的主交付）**：全部写在
[`docs/control/phase2-implementation.md`](../../../docs/control/phase2-implementation.md) **§3.3.5**
「★ 核对记录：PX4 `MPC_THR_HOVER` 与模型 `hover_thrust`（改动 `phase4/02-C1`）」，
共 7 小节，逐框对应如下。

| 本票的框 | 落在哪 | 一句话结论 |
|---|---|---|
| 2 逐项一致 | §3.3.5 ①② | 两处**都是** `0.1589`：`z0.params:628` 是 `0.5`（**待写**）、`params.py:62` 是 `0.1589`（已回填）；`t_hover = m·g` 与 `t_max = t_hover/hover_thrust` 逐项核过，`preflight_check` 24/24 全绿（含「归一化 = −hover_thrust」那条） |
| 3 三个问题 | §3.3.5 ①②③ | 两个值**是同一个量**（归一化推力域）；实测数字来自 Phase 1 悬停日志 `0.22199` @20.479 V / `0.22088` @20.526 V，按 `thrust ∝ V^1.608` 归一到满电 25.2 V 得 `0.1589`；**为什么必须相等**：PX4 对 `thrust_body` 不做任何重标定（`PositionControl.cpp:210` ⟹ 悬停时 `thrust_body[2] = −MPC_THR_HOVER`），两处不同值就是**静默的推力增益错误** |
| 4 `THR_MDL_FAC` 复核 | §3.3.5 ④ + 新工具 | 为它**新写了第二把尺子** [`tools/hover-thrust/thrust_curve_check.py`](../../../tools/hover-thrust/thrust_curve_check.py)，在**两条真机日志**上扫 `fac ∈ [0,1]`，r² argmax = **`0.455` / `0.450`**（对照 `fac=0` 0.9952/0.9955、`fac=1` 0.9871/0.9836）⟹ **`0.45` 被飞行数据支持、不用改**；两者**不同域**（它在"归一化推力↔电机信号"层），所以改 `MPC_THR_HOVER` 不需要动它 |
| 5 差异**记录**不平均 | §3.3.5 ⑤ | 42.9 % **是油门域**的数（建在 `T ∝ 油门²` = `fac=1` 上，**被上一条否定**）⟹ 不是本参数的候选值。**同域**对照：理论 `0.2373` vs 实测 `0.22199`，理论**高 6.9 %** —— 差异**记录在案**，不做平均、不用理论值替换实测量 |
| 6 编号 + 理由 + 回链 | §3.3.5 全节 + §5 的 **P7** 行（= `params-changes.md` 的 **C1**）+ §9 的 **O10** | 编号 `phase4/02-C1`；`params-changes.md` 的 **C1 / C5 / §G-3 / §I.1 第 5、6 行**已同步并回链本节 |

**第 1 框为什么没勾**：`MPC_THR_HOVER` 要写进飞控（**QGC**，或 mavros 的 `param set`，见文末那条登记）
⟹ 是**人的动作**。
本票**不重新测量**（按票面），也**不代跑车侧动作**。
写的时候**就写 `0.1589`**（不要写 `0.5`、也不要写理论值 `0.429`）；写完飞机重新上电读一次即可确认。

**写下去会连带变的四处**（写的人要知道，全文在 §3.3.5 ⑥）：

- **a. HTE 窗口解夹**：`MPC_USE_HTE = 1` ⟹ 窗口由 `constrain(MPC_THR_HOVER ∓ HTE_THR_RANGE, …)` 造，
  现在实测估计值**整段贴在下限**（`0.5 − 0.2 = 0.300`，`valid` 97.3 % ⟹ 夹钳真的咬住了）。写完之后下限变 `0.000` ⟹ 估计器**第一次真的能往下走**。
- **b. 手动模式手感**：`MPC_THR_CURVE = 0` 的语义是"居中杆 = `MPC_THR_HOVER`" ⟹ 居中杆推力 `0.5 → 0.1589`。**建议保留 `MPC_THR_CURVE = 0`**（改成 1 反而让悬停落在 8.6 % 杆位）。新增一条 **O10** 记着，**未实飞验证**。
- **c. 降落检测阈值**：`sys_low_throttle = MPC_THR_MIN + (hoverThrottle − MPC_THR_MIN)·thr_pct_hover`，
  而 `hoverThrottle` 与 `thr_pct_hover` 是**同一支**在切换（`MulticopterLandDetector.cpp:101-108` 只在 `hte.valid` 时
  才把 `hoverThrottle` 换成 HTE；`:217` 的 `thr_pct_hover = valid ? 0.6 : 0.3`）⟹ 写完之后有**三个**值，不是一个：

  | 情形 | `hoverThrottle` | `thr_pct_hover` | `sys_low_throttle` |
  |---|---|---|---|
  | 现状（参数 `0.5`，HTE 有效、估计值**贴着下限 `0.300`**） | 0.300 | 0.6 | **0.228** |
  | 写完，HTE **无效**（回落用参数本身） | 0.1589 | 0.3 | **0.132** |
  | 写完，HTE **有效但还没收敛**（估计值 ≈ 参数） | 0.1589 | 0.6 | **0.143** |
  | 写完，HTE **有效且收敛到真实值**（≈0.222） | 0.222 | 0.6 | **0.181** |

  ⟹ 稳态悬停读数 `0.222` **当前落在"低油门"这一侧**（`0.222 ≤ 0.228`），写完后**四种情形都不再触发**。
  ⚠️ 别把后两行的 `0.143`/`0.181` 读成"HTE 有效/无效"—— 它们**都是有效**那一支，区别只在估计值收敛到哪。
- **d. `MPC_THR_MIN = 0.12` 的余量**：留给控制的上半程只剩 32 %（`MulticopterPositionControl.cpp:238-241` 把指令夹在 `[0.12, 1.0]`）。**够用，但别再加 `MPC_THR_MIN`**。

⚠️ **两件本次没有验证的**（写在 §3.3.5 ⑦，别当已验证读）：① 离板飞行时 HTE 是否 `valid`（决定 a/c 两条实际生效的是哪一支）；
② 手动模式手感变化是否可接受 —— 两者都要**上电后手动悬停**时看一眼。

### 顺带的两件事（都已落地，供复核）

1. **新工具 + 证据**：[`tools/hover-thrust/thrust_curve_check.py`](../../../tools/hover-thrust/thrust_curve_check.py)
   （11 条自检 + **3 处变异各自弄红**，退出码 1 ⟹ 断言有鉴别力），原始输出落在
   [`tools/hover-thrust/evidence/thrust_curve_2026-10-09.txt`](../../../tools/hover-thrust/evidence/thrust_curve_2026-10-09.txt)，
   说明在 [`tools/hover-thrust/README.md` §8](../../../tools/hover-thrust/README.md)。**两条引用的 ulog 都在 git 里**，可逐字复跑。
2. **修正了一条旧算式**：[`phase2/10`](../../phase2/issues/10-param-measure-backfill.md) Comments 里
   「DShot 值 ≈ `640/1999` = 0.318」—— **数没写错（结论 0.318 对），算式错了**：
   DShot 是**仿射**编码（`output ≈ 1889·x + 107`），不是 `1999·x`；0.318 其实是 `inv(ctrl 的均值)`。
   ⚠️ 若不改算式，**下次复算会得出不同的数**。票已 `done`，**只在此登记**。

### 复核（2026-10-09，两轴 code review：标准 / 票面）—— 逐条处置

复核按仓库的两轴做法跑（标准轴 + 票面轴各一个独立子代理），**七条意见逐条回代码核实**后处置如下。
★ 记在这里是因为"复核说了什么、我为什么这么办"本身是证据；**驳掉的也写**。

| # | 轴 | 意见 | 处置 |
|---|---|---|---|
| 1 | 标准 | 新工具的自检**收不到** `pytest tools/hover-thrust/ -q`（家族里每个工具都有 `test_*.py`，README §5 又广告了这条命令）—— 是**最大的一处偏离家务** | ✅ **已改**：新增 [`tools/hover-thrust/test_thrust_curve_check.py`](../../../tools/hover-thrust/test_thrust_curve_check.py)，把 `self_check()` 的返回值由"0/1"改成"不过的项名列表"，pytest 只转接、**不重抄**判据。`pytest tools/hover-thrust/ -q` 由 36 → **37 passed** |
| 2 | 标准 | `thrust_curve_check.py` 里 `min_gap` 的注释写"真日志实测 5.6e-3（0.99968 vs 0.99410）"—— 这两个数**在真日志里根本不存在**（真值 4.6e-3 / 4.5e-3） | ✅ **已改**：注释不再写死一个数，改成"从 evidence 文件当场复算"的写法（正撞本仓库「散文里写死的数会过期」那条） |
| 3 | 标准 | evidence 文件里有一句**逐字重复** | ✅ **已删** |
| 4 | 票面 | 连带效应 c 把 `0.143` / `0.181` 标成"HTE 有效 / 无效"—— **错**：`hoverThrottle` 与 `thr_pct_hover` 是**同一支**在切换（`MulticopterLandDetector.cpp:101-108` + `:217`），"无效"那一支应该是 **`0.132`** | ✅ **已改**：本节上表与 §3.3.5 ⑥c 都改成**三个值**（0.132 / 0.143 / 0.181），⑦ 的交叉引用也跟着改 |
| 5 | 票面 | `README.md` §8 注意 3 写"两条命令的窗口差**几十毫秒**"—— 实测差**零点几秒**（`119.779–196.279` vs `120.599–196.398` vs 稳态段 `122.338–192.758`） | ✅ **已改**：三个窗口逐个数出来 |
| 6 | 标准 | `curve_inverse + affine_r2` 这个两行形状在 `mean_r2` / `infer_mapping` / `analyse_ulog` 里出现三次（Duplicated Code）；`analyse_ulog` 一个函数兼了适配层+推断+扫描（家族惯例是拆 `*_core.py` / `*_from_ulog.py`） | ⛔ **不改**（判断项）：三处各两行、且**没有第二份"真话"**（都在调同一个 `affine_r2`）；为一个测量脚本做拆分会把"一条等式一段代码"的可读性换掉。**记在这里备将来** |
| 7 | 票面 | 两处**范围扩张**：(i) 第 4 框只说"复核…关系"，却新建了 ~450 行的量具 + 证据文件 + README §8；(ii) 第 6 框只点名 `phase2-implementation.md`，却**也**改了 `params-changes.md`（C1/C5/§G/§I.1）；外加四条连带效应与新的 O10 | ⛔ **不撤**（判断项，理由是**这票的判据本身**要求"复核"而不是"复述"）：(i) 不量就只能把 `THR_MDL_FAC = 0.45` 抄一遍当"复核过"，而本仓库的纪律是第一手证据；(ii) `params-changes.md` 的 C1 行**正是**被本票改掉口径的那一处，不同步回链就留下两份互相矛盾的 C1。**对别的票的纪律守住了**：过期引用只登记、不回改（下一节） |

⚠️ 另一条**只登记不改**的：票面写"改飞控参数要在 QGC 里做"，而 `params-changes.md` §I.1 第 1–2 条
记的执行人却是"**agent（mavros）**"⟹ **QGC 不是唯一通路**（mavros 的 `param set` 也行）。
票面那句不是错（"哪个读数才算数"确实是人裁定），但写参数的人**不必**被 QGC 绑住。

### 登记的过时引用（**没有**动它们，留给各自的主人）

本票把 C1 的口径改了（`0.5` 待写 / 实测 `0.1589` / 42.9 % 只是油门域的数），下面几处的旧口径**因此过期**。
按"不顺手改别的票"的纪律，这里**只登记、不回改**：

| 位置 | 过时在哪 | 谁来收 |
|---|---|---|
| [`docs/PROJECT.md:469`](../../../docs/PROJECT.md)（O5） | 还写着「是默认值还是实测值？**一半已答**」—— 现在**已全答**（实测 `0.1589`） | PROJECT.md 的主人 |
| [`docs/PROJECT.md:155`](../../../docs/PROJECT.md) / [`:390`](../../../docs/PROJECT.md) | 把 `42.9 %` 与 `MPC_THR_HOVER = 0.5` 并列成两个候选 —— 前者**是油门域**的，不构成对本参数的候选 | 同上 |
| [`docs/control/phase1-implementation.md:876`](../../../docs/control/phase1-implementation.md) / [`:892`](../../../docs/control/phase1-implementation.md) | 「`MPC_THR_HOVER`（留到推力标定）」「**需标定** … XI35 的实测值是 0.45」—— 量完了、且 XI35 那个数是**另一台机**的 | phase1 文档的主人 |
| [`docs/reference/hardware.md:212`](../../../docs/reference/hardware.md) | 「与推算的 42.9 % 有出入」—— 出入的是**域**，不是测量分歧 | hardware.md 的主人 |
| [`docs/reference/params-analysis.md:522`](../../../docs/reference/params-analysis.md)（§3.11）、[`:893`](../../../docs/reference/params-analysis.md) | 「建议实测后回填」+ 待办表「⬜ 未做」 | params-analysis.md 的主人 |
| [`.scratch/phase1/issues/15`](../../phase1/issues/15-fcu-param-reconciliation.md)（多处，含 :275 的"**未做**"） | 同 C1/C5 口径 | 该票已 `done`，**只登记** |
| [`phase2/10`](../../phase2/issues/10-param-measure-backfill.md) Comments 引的悬停日志路径 | 引的是 `bag/3_traj1_hover/fcu/log_0_未知日期.ulg`，那是**另一条、更晚**的日志（反推得 `0.18471`）；`0.22199`/`0.22088` 出自 **`bag/old/`** 下的两条。**读数没变（已回填 `0.1589`），但引用路径会把人指错** | 该票已 `done`，**只登记** |
