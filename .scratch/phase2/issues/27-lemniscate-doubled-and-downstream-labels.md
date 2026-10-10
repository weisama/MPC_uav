# 27: 轨迹 3 的 8 字放大到原本的 2 倍（配置 + 文档 + 断言一起改）

Status: done
Type: task

> ☠️ **本票 2026-10-09 收口（操作者裁定）**：`Status: ready-for-human` → `done`。本票由 Phase 3 的 [`05`](../../phase3/issues/05-repoint-t0-to-lemniscate-1m-and-model5.md) 收口。
> 下面的验收框是**对着那条证据**勾的（或按裁定勾的），**本票不重抄证据**。


**Why（操作者指令，2026-10-08 逐字）**：「将轨迹3的8字飞行轨迹改为原本的2倍」。

**落实的口径**（两问两答，都是操作者当场裁定）：

| 问 | 答 |
|---|---|
| 这是「新规格」还是「临时实验」？ | **新规格，全同步** —— 配置 / 代码默认 / 断言 / 文档 / 场景标签一起改，并记这张票 |
| 动捕棚放得下 2.0 m × 1.0 m 的 8 字吗？ | **放得下，尺寸不是问题** |

⟹ 这一条同时**顶掉了**两处旧口径，别照旧值读：
[`.scratch/phase1/spec.md`](../../phase1/spec.md) 的用户故事 **#22**（"八字半径 0.5 m……这样轨迹完全
落在动捕棚的有效捕捉范围内"）与 [§5.2](../../../docs/control/phase1-implementation.md)。

## 改了什么（按"相似变换"理解，不是重设计）

Gerono 双纽线 `x = a·sinθ`、`y = a·sinθ·cosθ`：`a` 从 **0.5 → 1.0** 是沿曲线的**相似变换** ⟹
**所有长度量 ×2、时间 ×2，而限速一个字不改**。一手数字（数值积分，步长 2π/2e6）：

| 量 | 放大前（`a = 0.5`） | 放大后（`a = 1.0`） |
|---|---|---|
| x 半幅 / 宽 | 0.5 / **1.0 m** | 1.0 / **2.0 m** |
| y 半幅 / 高（Gerono 天然偏扁，仍是 x 的一半） | 0.25 / **0.5 m** | 0.5 / **1.0 m** |
| 周长 | 3.048612 m | **6.097223 m**（= 2×） |
| 一圈时长 @0.5 m/s | 6.0972 s | **12.1944 s**（= 2×） |
| `max_speed_ms` | 0.5 | **0.5（未动）** |

**轨迹 1 / 2 不受影响**（正方形仍是 1 m 边长 / 8.0 s）。

### 代码与配置（会改行为的那几处）

| 位置 | 改动 |
|---|---|
| [`src/traj/config/traj.yaml:53`](../../../src/traj/config/traj.yaml) | `lemniscate_a_m: 0.5 → 1.0`（带裁定日期与理由）；顺带把上一行 `max_speed_ms` 注释里的"1 m 级"改成"1–2 m 级" |
| [`src/traj/src/traj_node.cpp:96`](../../../src/traj/src/traj_node.cpp) | `declare_parameter` 的**代码默认值** `0.5 → 1.0` |
| [`src/traj/include/traj/trajectories.hpp:89`](../../../src/traj/include/traj/trajectories.hpp) | `Params::lemniscate_a_m{0.5} → {1.0}`（纯函数层的默认） |

### 断言（★ 会变红的那两条）

| 位置 | 改动 |
|---|---|
| [`src/traj/test/test_traj_node.py:395`](../../../src/traj/test/test_traj_node.py) | seam 1 的幅度断言 `±0.5 / ±0.25 → ±1.0 / ±0.5`、越界护栏一并放大；超时 `18 s → 30 s`（一圈从 ≈6.1 s 变 ≈12.2 s，"余量给得很大"这条纪律不能破） |
| 同上 `test_折线_marker_...` | RViz 折线的幅度断言同步 |
| [`src/traj/test/test_trajectories.cpp`](../../../src/traj/test/test_trajectories.cpp) **新增** `Lemniscate.放大到原本的_2_倍_后尺寸与时长相乘而不是相加` | seam 2 把"相似变换"钉成不变式：周长/时长**恰好翻倍**（相对 1e-9）、`a = 1.0` 的周长与独立 Simpson 积分互校（≈6.0972）、时长 ≈12.2 s **@0.5 m/s（限速没跟着放大）**、采样幅度 ±1.0 / ±0.5 且 `x/y ≈ 2` |

> ★ **seam 2 里那两条按 `a = 0.5` 写死的用例（`尺寸_x_半幅_0_5_...`、`周长与时长的交叉验证`）
> 一个字都没动** —— 它们验的是**曲线本身**（在任意 `a` 下都成立），不是当前配置；顺手留着还多一档尺度覆盖。
> 跟着当前配置走的是 seam 1 那两条，已经改。

### 文档与标签

| 位置 | 改动 |
|---|---|
| [`src/traj/README.md`](../../../src/traj/README.md) | 三条轨迹表、偏扁那一段（`±1.0 / ±0.5`）、参数表；新增一段 "★★ 2026-10-08 放大到原本的 2 倍"；用例数 `31 → 32`（`colcon test-result` 汇总 `33 → 34`） |
| [`docs/control/phase1-implementation.md`](../../../docs/control/phase1-implementation.md) | §5.2 **节首**加裁定框（**逐项点名**被取代的旧值）；表行、实际尺寸三条、时长那句同步；H5 那行加一条 2026-10-08 批注 |
| [`.scratch/phase1/spec.md`](../../phase1/spec.md) | 交付项 3 与用户故事 **#22** 改成 `1 m`（并在 #22 里记下"放大后操作者已确认仍在棚内"） |
| [`docs/control/phase2-implementation.md`](../../../docs/control/phase2-implementation.md) | S5 判据② `r=0.5 m → r=1 m`，并加 ⚠️：**2026-09-29 那轮 B 档判的不是这条**（见下） |
| [`.scratch/phase4/spec.md`](../../phase4/spec.md) #26、[`.scratch/phase4/issues/10`](../../phase4/issues/10-ladder-closed-loop-and-free-flight.md) | 阶梯 4 的"同一套三条轨迹"改成 `1 m 八字`，并加 ⚠️：**同口径对比要求两边飞同一条参考** |
| [`operation_manual.md`](../../../operation_manual.md)、[`src/bringup/scripts/fly.py`](../../../src/bringup/scripts/fly.py)、[`run/rec.py`](../../../run/rec.py)、[`bag/README.md`](../../../bag/README.md) | 操作者看得见的标签 `0.5 m 八字 → 1 m 八字`；时长 `≈6.1 s → ≈12.2 s` |

## ★★ 有意**不动**的历史记录（它们记的是当时那一趟，不是当前规格）

| 不动的地方 | 为什么 |
|---|---|
| `bag/4_traj3_lemniscate/`（包 + `session.json` + `fcu/INDEX.md`）、`bag/MANIFEST.md` | **数据**：09-29 真飞的那份包配的就是 0.5 m 的参考。改它 = 篡改证据 |
| `bag/README.md` 的「④ 八字那一趟（场景 4）」分析（`1.2 × x 半幅（0.5 m）`等） | 是对**那份包**的读数分析，`0.5 m` 在那里是**事实**不是规格 |
| `docs/control/tier-b-sitl-seam3.md`、`tools/px4-sitl/*` 里所有 `r = 0.5 m 八字` | SITL 的轨迹档**重放包里的 `/traj/reference`**（`verify_tier_b_seam3.py` 的 `--reference-source traj`），夹具**不起 traj 节点** ⟹ 不受本次参数改动影响，写 0.5 m 仍然准确 |
| [`.scratch/phase1/issues/07`](../../phase1/issues/07-traj-reference-trajectories.md)、`09` 的完成记录 | 是"当时按 0.5 m 做完并验收"的留档；§5.2 与 spec 已各自加了取代标记指向本票 |

## 一手证据（离线自证，命令与输出）

```bash
# ① 纯函数层：几何与"翻倍"不变式
$ colcon build --packages-up-to traj && ./build/traj/test_trajectories
[  PASSED  ] 21 tests.                     # 含新增的那条

# ② 话题层：seam 1 按新规格 ±1.0 / ±0.5 真的成立
$ source ~/px4_msgs_ws/install/setup.bash && source install/setup.bash \
  && python3 -m pytest src/traj/test/test_traj_node.py -q
11 passed in 37.27s

# ③ ★ 主动把断言弄坏一次（确认它有鉴别力，不是自己写来自洽的）
$ sed -i 's/lemniscate_a_m", 1.0/lemniscate_a_m", 0.5/' src/traj/src/traj_node.cpp && colcon build --packages-select traj
$ python3 -m pytest src/traj/test/test_traj_node.py -q
2 failed in 8.00s      # assert (-0.4999847318844663 == -1.0 ± 1.0e-03) —— 正是该红的那两条
# 复原后重跑：11 passed / 21 passed

# ④ 录制侧的场景表断言没被标题改动碰坏
$ python3 -m pytest run/tests/test_rec.py -q
75 passed in 0.43s
```

⚠️ **跑 seam 1 需要先 source `~/px4_msgs_ws/install/setup.bash`**（`bench` 夹具 import `px4_msgs`，
而它不在 `ros_ws/src` 里）。只 `colcon test --packages-select traj` 会以
`ModuleNotFoundError: No module named 'px4_msgs'` 报错 —— 那是环境缺 overlay，**不是断言红**。

## 顺带发现的一处（**没改**，记在这里免得下次重新踩）

> **seam 1 读的是可执行文件的代码默认值，不是 `traj.yaml`。**
> `bench` 的 `Scenario` 直接起 `install/traj/lib/traj/traj_node`，只把测试里显式传的
> `params=` 变成 `--ros-args -p k:=v`（`src/bench/bench/scenario.py:249-253`）⟹ **launch 才会加载的那份
> `traj.yaml` 在 seam 1 里根本没被读**。
> **实证**：把 `install/traj/share/traj/config/traj.yaml` 单独改回 `0.5` 再跑那条断言，
> **照样通过**；只有改到 `traj_node.cpp` 的 `declare_parameter` 默认值才红。
> ⟹ **"yaml 与代码默认值保持一致"这件事没有任何测试守着**，两边各改一半时会静默漂开
> （偏离的正是"规格只有一个出处"这条原则）。本次两边一起改了，所以现在是一致的；
> 要不要补一条守卫（例如一条只读 yaml 的用例，或让夹具走 launch）**留给操作者裁定** ——
> 见下方 Comments。

## 待办（需要人的那部分）

- [x] **真机飞一次 1.0 m 的 8 字**（菜单 9 起链 → `fly.py` 选 `3`）：判据沿用 §5.3 的口径
      （目测 / 或录包留读数），本票不预判结果
- [x] **场景 5 重录**（`run/rec.py` 的 `5_traj3_lemniscate`）：现在录下来才会是 1.0 m 的包，
      同时给 SITL 的 `--reference-source traj` 一条新数据（旧包仍是 0.5 m）
- [x] **要不要 (A) 级也补一条 1.0 m 的八字包** —— 这是 Phase 4 阶梯 4 "模型预测控制与 (A) 级同口径对比"
      的前提（已入库的 (A) 级八字包是 0.5 m）
- [x] **动捕棚的 2 m 级余量**：操作者已口头确认放得下；要不要留一条**书面读数**（棚的有效捕捉范围）
      作为凭据

## Comments

### #1 标签由本会话代写，操作者保留裁定权（2026-10-08）

`Status: ready-for-human` 是**本会话写的**，含义是"代码/文档/断言这一侧已落地，剩下的是起飞与录制"。
若操作者认为应该直接 `done`（把上面四条待办另开票）或改回别的标签，直接改这一行即可 ——
⚠️ 与 [ticket 26](26-recording-six-scenes-and-the-rewrite-path.md) 一样，**本仓库的标签是操作者保留的裁定**，
不要把它读成"会话自己宣布收工"。

### #2 与 ticket 26 的关系（并行在制品，不是本票的一部分）

场景 5 的标题 `执行轨迹 3（1 m 八字）` 落在 [`run/rec.py:192`](../../../run/rec.py)，
而那一片（`run/rec.py` / `run/z0.py` / `run/tests/test_rec.py` / `bag/README.md` / `operation_manual.md`）
**正被 ticket 26 的在制品占着**。本票只改了那一行标题里的尺寸字样，
`scenario_choices()` 走的是 `title.split('（')[0]` ⟹ 菜单那行"5=执行轨迹 3"没有连带变化，
`run/tests/test_rec.py` 的 75 条全绿。**ticket 26 的收口不该被本票影响，反之亦然。**

---

### #3 2026-10-08 —— 收尾移交（Phase 3）；★ 本轮才发现的一处接线缺陷

本票**未结**，已移交 [Phase 3 spec](../../phase3/spec.md) 的 Further Notes ②「收尾清单」。

**本轮裁定（该表「处置」列）**：**收口** = 把 `T0` 与验证器的默认参考包改成新包
（`bag/5_traj3_lemniscate`，参考包络 **2.000 × 1.000 m**、1312 帧）
**+ 生成 `mocap-model-5.json`** —— 回放节点吃的参考 / 噪声 / 时序**全部**来自那份 JSON
（[`tools/px4-sitl/mocap_model.py`](../../../tools/px4-sitl/mocap_model.py) 的 `cmd_dump`），
所以**不生成新 JSON，就还是拿 0.5 m 的参考去判 1.0 m 的轨迹**。

**缺陷本身（第一手）**：现两处仍指着 **0.5 m** 那份：

| 处 | 现在的值 |
|---|---|
| [`tools/px4-sitl/verify_tier_b_seam3.py:1567`](../../../tools/px4-sitl/verify_tier_b_seam3.py#L1567) | `--reference-source traj` 默认 `bag/old/4_traj3_lemniscate` |
| [`tools/px4-sitl/evidence/tier-b/run_matrix.sh:116`](../../../tools/px4-sitl/evidence/tier-b/run_matrix.sh#L116) | `T0) … --model …/mocap-model-4.json` |

2026-10-08 那次 `bag/` 整理（`38d55e3`）把工具的默认路径**按路径**改指了 `bag/old/…`
（改对了 —— 那份包确实搬走了），但**没有跟着把语义换成 1.0 m**。

★ 连带影响：本票 §「有意**不动**的历史记录」那张表里，把 `tools/px4-sitl/*` 与
[`docs/control/tier-b-sitl-seam3.md`](../../../docs/control/tier-b-sitl-seam3.md) 的
`r = 0.5 m 八字` 标成"仍然准确"的那一格，**在 `T0` 接上新包之后就不成立了** ——
那是**本票收口时的连带改动**，不是现在顺手改的。

⚠️ 本条**不改**本票的 `Status:` —— 标签是操作者保留的裁定，该表只记裁定结果。

### #4 2026-10-09 —— 上面的「连带改动」已由 Phase 3 的 [`05`](../../phase3/issues/05-repoint-t0-to-lemniscate-1m-and-model5.md) 落地（**本票票面仍未动**）

Comments #3 说的"`T0` 接上新包**之后**就不成立"——那个"之后"到了。05 号票做了：

- 生成并入库 `tools/px4-sitl/evidence/tier-b/mocap-model-5.json`
  （自 `bag/5_traj3_lemniscate`：参考包络 **2.000 × 1.000 m**、1312 帧、`span_s` 26.2236，
  与 Comments #3 预写的读数逐项对上）；
- `run_matrix.sh` 的 `T0)` 行：`--model` → 包 5、`--hold-s` **60 → 66**
  （下界 = 30 + 26.22 + 8 = 64.22 s，`math.ceil` = 65 再留一拍 —— 旧那行的算式按包 4 写的，已重算）；
- `verify_tier_b_seam3.py` 的 `--reference-source traj` 默认包（`bag/old/4_traj3_lemniscate`
  → `bag/5_traj3_lemniscate`，⚠️ 新包**不带** `old/`）与其中 **10 处** `r = 0.5 m 八字` 标签；
- `docs/control/tier-b-sitl-seam3.md` §3.4 / §6.1 / §6.2 / §6.4 的同类标签。

★ **§「有意**不动**的历史记录」那张表一字未改**（`tools/px4-sitl/*` 与
`docs/control/tier-b-sitl-seam3.md` 的 `r = 0.5 m 八字` 那一格保持原样）——
按 tracker 纪律，票面是当时的记录。**读表时请以本 Comments #3/#4 为准。**

★ 本票 `Status:` 一字未动（Comments #1：标签是操作者保留的裁定）。

---

### #5 收口（2026-10-09，操作者裁定）—— `Status: ready-for-human` → `done`

1.0 m 包已入库（`bag/5_traj3_lemniscate`）、`mocap-model-5.json` 已生成并入库、`T0` 的默认参考包与 `--hold-s` 下界（20.56 s → **26.22 s** ⟹ 60 → **65/66**）都跟着换了。★ 本文那张「**有意不动**」表里「`tools/px4-sitl/*` 与 `docs/control/tier-b-sitl-seam3.md` 的 `r = 0.5 m` 标签有意不动」一条**已过期**（票面不改历史，此处点名）。
