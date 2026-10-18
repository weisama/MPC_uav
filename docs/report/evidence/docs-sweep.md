# 文档清扫证据（Phase 5 票 18）

- **日期**：2026-10-18
- **范围**：`README.md`、`docs/**`、`operation_manual.md`、`run/README.md`、各包 README 的活文档。
- **未触碰**：`docs/report/chapters/**`（票 16 的在制品）、`.scratch/**` 的历史票面、
  `tools/*/evidence/**` 的归档证据、`docs/assets/{z0.params,papers,drill}/**` 的原始产物。
  工作树里这些路径的改动来自并行会话，不是本票的改动。
- **原则**：只修失效链接、过期状态词与明显矛盾的散文；每处语义改动都回第一手代码/证据/票面核实。
  被作废的结论按仓库惯例标 ☠️ 或写明"原写"，不把"没做"写成"做了"。

---

## 1. 链接检查

仓库原先没有可复用的 markdown 链接检查器（`tools/` 下只有证据脚本）。本票新增
[`tools/report/check_links.py`](../../../tools/report/check_links.py)：无第三方依赖，检查：

- 相对路径目标是否存在；
- GitHub 风格的标题锚点（含中文标题、重复标题的 `-1`/`-2` 后缀）；
- HTML `<a name/id>`、`<span id>` 锚点；
- 跨行行内链接；
- 围栏代码块与行内代码中的示例链接不参与检查。

运行命令与结果：

```bash
$ python3 tools/report/check_links.py
检查了 54 个文件，断链 0 处。
```

默认目标为 `README.md`、`operation_manual.md`、`run/README.md`、`docs`、`src`；
按票面要求排除 `docs/report/chapters/`、`docs/assets/papers/`、`docs/assets/drill/`。

---

## 2. 改动清单

### 2.1 `MPC_THR_HOVER` 与三条配置参数：从"待写入"改为"已写入并读回"

**第一手锚**：

- `MPC_THR_HOVER = 0.1589` 于 2026-10-13 经 mavros `/mavros/param/set` 写入，
  `force_pull` 后读回 `0.15889999270439148`（float32 的 `0.1589`）——
  `.scratch/phase4/issues/02-hover-thrust-backfill.md` 的 2026-10-13 Comments（含命令、回读与"没走 QGC"的登记）。
- `EKF2_GPS_CTRL = 0`、`SYS_HAS_GPS = 0`、`MPC_XY_VEL_MAX = 2.0` 于 2026-09-23 写入并读回，
  2026-09-28 复核仍在 —— `docs/reference/params-changes.md` §I.1 的读回表（三行均 ✅）。
- `z0.params` 是 **2026-10-10** 的导出快照，早于上述写入，所以文件里仍是 `MPC_THR_HOVER = 0.5`。

| 文件 | 位置 | 原文 | 改后 |
|---|---|---|---|
| `docs/reference/params-changes.md` | §C1 行 | 状态列"⬜ **待写入**（QGC，人的动作）" | "✅ **2026-10-13 已写入并读回**（mavros `/mavros/param/set`，不是 QGC；`force_pull` 后读回 `0.15889999270439148`）" |
| 同上 | §I 第 5 条 | "飞控侧仍待写"、"人（只剩写飞控这一步，QGC）" | "飞控侧 2026-10-13 已写并读回"、"✅ 2026-10-13 已通过 mavros `param set` 写入并读回" |
| 同上 | §I 第 5 条后记 | "飞控侧仍待写入" | "飞控侧 2026-10-13 已写入并读回" |
| 同上 | §I 末段 | "第 1–2 条已裁定但尚未写入" | "第 1–2 条已于 2026-09-23 写入并读回（见 §I.1）" |
| 同上 | C1 上方说明块 | "仍未做的是'写进飞控'……要在 QGC 里改" | 已写入（mavros，非 QGC）；`z0.params:628` 仍是 `0.5`，因为它是 2026-10-10 快照、不代表活值 |
| 同上 | §I 前"3 条配置类参数" | "另有 3 条配置类参数**待写入**" | "**已写入**（2026-09-23，见 §I.1）" |
| 同上 | §F6 行 | 状态列"⬜ **待写入**（QGC，人的动作）" | "✅ **2026-10-13 已写入并读回**（mavros `param set`；读回 `0.15889999270439148`）" |
| `docs/control/first-flight-config.md` | F6 行 | "★★ **待写入**（**人的动作**，QGC）"；导出文件"现读 = 0.5" | "✅ **2026-10-13 已写入并读回**（mavros `param set`，不是 QGC；……）"；注明导出文件是 2026-10-10 快照 |
| 同上 | §5 标题与第 ① 项 | "起飞前还没做的……"；①"写入 `MPC_THR_HOVER = 0.1589`（QGC）" | 标题改为"起飞前动作（……① 已于 2026-10-13 完成）"；①划掉并写已完成/读回值 |
| `docs/control/ladder-acceptance-criteria.md` | 作废横幅表 ③ 与正文第 3 条 | "状态仍是 ⬜ **待写入**、人的动作" | "状态 ✅ **2026-10-13 已写入并读回**（mavros `param set`）" |
| `docs/control/phase2-implementation.md` | P7 行 | "⬜ **待写**（QGC，人的动作……）" | "✅ **2026-10-13 已写入并读回**（……`force_pull` 后读回 `0.15889999270439148`）" |
| 同上 | §3.3.5 的 `0.1589` 取值行 | "飞控侧待写" | "飞控侧 2026-10-13 已写入并读回" |
| `docs/control/phase1-implementation.md` | `MPC_XY_VEL_MAX` 行 | "已裁定：改 `2.0`（**待写入**）" | "已裁定并落地：改 `2.0`（✅ 2026-09-23 写入并读回，§I.1）" |
| 同上 | `MPC_THR_HOVER` 行 | "⚠️ **需标定**；XI35 的实测值是 0.45" | "✅ **已量并已写入飞控**；z0 实测 `0.1589`（2026-10-09）……飞控侧 2026-10-13 写入并读回；XI35 的 0.45 是另一台机" |
| 同上 | `THR_MDL_FAC` 行 | "与 XI35 的 `K3` 不同，两个值至少有一个是错的" | 2026-10-09 两条真机日志 r² argmax `0.455`/`0.450`，`0.45` 被 z0 数据支持；XI35 是另一台机 |
| `docs/reference/params-analysis.md` | §8 第 8 项 | "**已裁定……待写入**" | "✅ **2026-09-23 已写入并读回 `0`**（§I.1），2026-09-28 复核仍在" |
| `docs/control/bench-runbook.md` | §I 上方提示 | "另有 3 条参数是「已裁定、待写入」……这次上电就是补写的机会" | "「已裁定、2026-09-23 已写入并读回」……写入与读回记录见 §I.1；2026-09-28 复核三条仍在" |

**证据补充**：`THR_MDL_FAC` 的 r² 值在
`docs/control/phase2-implementation.md` §3.3.5 的扫描表（`fac` argmax `0.455` / `0.450`），
原始输出归档于 `tools/hover-thrust/evidence/thrust_curve_2026-10-09.txt`。

### 2.2 版本、包数、spec 数

| 文件 | 位置 | 原文 | 改后 | 证据 |
|---|---|---|---|---|
| `README.md` | 版本表"当前版本/日期/分支/本版对应提交" | `v3.0`（`cdddeb5`，2026-10-09）；"v1.0/v2.0/v3.0 三个都是分支"；`git rev-parse v3.0` | `v4.0`（`f137472`，2026-10-17，Phase 4 收口后的快照）；补 `v4.0` 分支；`git rev-parse v4.0` | `git rev-parse --short v4.0` = `f137472`；`git tag -l` 为空；`git reflog show v4.0` = `Created from main`，2026-10-17 08:55:17 |
| 同上 | 工作区形态 | "6 个包" | "7 个包" | `find src -maxdepth 2 -name package.xml` = 7（含 `nmpc`） |
| 同上 | `control/` 目录树行 | "阶梯 1–3 级待做" | "Phase 4 阶梯 3–4 于 2026-10-17 飞完（票 10 `done`），阶梯 1–2 未做、裁定不补（票 09 `wontfix`）" | `.scratch/phase4/issues/10-*` `Status: done`；`09-*` `Status: wontfix` |
| 同上 | 进度说明 | "四份 spec" | "五份 spec（phase1–phase5）" | `.scratch/phase5/spec.md` 存在 |
| `CONTEXT.md` | `z0` 行 | "MPC（模型预测控制）仍未上机（只在 SITL 里跑）—— Phase 4 还没开始" | "模型预测控制 2026-10-17 已上 z0 实机（Phase 4 `spec` 已 `done`；三趟自由飞 = 悬停/正方形/八字……）" | `.scratch/phase4/spec.md` `Status: done`；`.scratch/phase4/issues/10-*` `Status: done` |
| `docs/reference/code-inventory.md` | 目录树总述 | "六个包都已落地" | "七个包都已落地" | 同上 7 个 `package.xml` |
| 同上 | 目录树 | 缺 `nmpc/` 行 | 补 "nmpc/ ← Phase 2 新写：模型预测控制节点（(C) 级 CTBR）+ 零 ROS 的 OCP / 符号映射" | `src/nmpc/package.xml`；`src/nmpc/nmpc/` 源码 |

### 2.3 `z0.params` 的行数/条数

现算（2026-10-18）：**955 行 / 911 条参数**。旧文档里的 **928 行 / 914 条**是
**2026-09-21 的快照**；文件头说明该文件已于 **2026-10-10 用 QGC 从 z0 飞控重导出**。
按仓库"要数就现算、别抄"的惯例，没有在所有地方盲目换成 955/911。

| 文件 | 位置 | 改动 |
|---|---|---|
| `CONTEXT.md` | `z0.params` 行 | 删除写死的"928 行/914 条"，改为现算命令；注明 2026-10-10 重导出；注明 914/915 对账只针对 2026-09-21 那一版 |
| `docs/PROJECT.md` | §5.1 | 标注 928/914 为 2026-09-21 快照；2026-10-10 已重导出，现算 955 行/911 条 |
| `docs/reference/params-analysis.md` | 页首 | 新增 ☠️ 说明：§1 的 928/914 是旧快照；§2 的逐模块统计未按新导出重算 |
| `docs/reference/params-changes.md` | 页首「现值」说明 | 标注 928/914 为旧快照；2026-10-10 已重导出；下表「现值」**没有按新导出逐条重算**，引用前先核 |
| `docs/reference/open-source-survey.md` | 目录树 | "z0.params（928 行，已归档）"改为"行数现算、别抄；2026-10-10 已从飞控重导出" |

### 2.4 Phase 4 / 票状态（含"收口 ≠ 做了"）

| 文件 | 位置 | 原文 | 改后 | 证据 |
|---|---|---|---|---|
| `docs/control/tier-a-sitl-hover.md` | 页首结论第 4 条、§ 末段、§"不能"第 2 条 | "闭环复飞的验证待 ticket 16 验收项 3（**待飞**）"、"（本票验收项 3，待飞）"、"ticket 16 验收项 3，**待飞**" | 三处统一为"**未飞**；该票 2026-10-09 已按操作者裁定收口为 `done`，结论由 ticket 25 给出" | `.scratch/phase2/issues/16-*` `Status: done` 且票面写明 2026-10-09 操作者裁定收口；`.scratch/phase2/issues/25-*` `Status: done` |
| `docs/control/phase2-implementation.md` | §S5 前记 | "S5 本身（闭环）仍未开始" | "**当时**仍未开始"；补 2026-09-29 起已跑（ticket 11 `done`），判据结果见 §S5 末表，其中第 1 条垂直误差/第 4 条失配兜底**未达标** | `.scratch/phase2/issues/11-*` `Status: done`；同文件 §S5 结果表 |

> ★ 这一组特意区分了"票 `done`"与"验收项真的做过"：ticket 16 是**裁定收口**，验收项 3 **没飞**；
> ticket 25 给出的是"机头回不来"的结论，不是"复飞通过"。

### 2.5 失效锚点

| 文件 | 位置 | 问题 | 改法 |
|---|---|---|---|
| `docs/control/mismatch-sweep.md` | §附.1 标题后 | 正文 `[§附.1](#附1)` 指向的锚点不存在 | 按同页 `附9` 的既有惯例新增 `<span id="附1"></span>`（同页已有 `<span id="附9"></span>`） |

### 2.6 测量口径更正（整周期 vs 求解耗时）

| 文件 | 位置 | 原文/问题 | 改后 | 证据 |
|---|---|---|---|---|
| `docs/control/offline-ocp.md` | §5 末 | 原写"整周期……尚未测 ⟹ 留给 seam 3"；本票初版补记误把 5.652 ms 写成"整周期稳态最坏" | 保留"当时尚未测"；补记改为"**逐拍求解耗时**稳态最坏 5.652 ms、实发 99.5–99.6 Hz（p50 10.0 ms、p99 11.4–11.9 ms）；**整周期墙钟没有单独分项**" | `docs/control/tier-b-sitl-seam3.md` §9 表头为"稳态最坏（求解耗时）"，F0 = 5.652 ms；同节给出实发频率 |
| `docs/control/phase2-implementation.md` | §5 计时块 | 承接上一条时容易把 5.652 ms 读成整周期 | 该句补"**逐拍求解耗时**"；前一句"仍未测"改为"**当时**仍未测"并指向下一条复核 | 同上 |

---

## 3. 未改动但记录在案

以下读了、核了，但**不动**，或按票面要求留给别的票：

1. **`README.md` 的四阶段路线表**：未改。表中没有 Phase 5 行 —— **待票 16** 在最终审查通过后补。
2. **`operation_manual.md` 的"截至 2026-09-28"、§8.1 的 2026-09-28 实飞记录、
   §8.2 的阶梯 1–3 未做**：都是带日期的历史记录，且与"2026-09-28 操作者裁定不再补做"一致，保留。
3. **"六个包"的自检描述**：`run/README.md` 与 `operation_manual.md` 里的"六个包"
   对应 `run/z0.py:153` 的 `EXPECTED_PACKAGES`（`vrpn_listener` / `mocap` / `traj` /
   `bench` / `control` / `bringup`，**不含 `nmpc`**），描述准确，未改。
   `docs/agents/domain.md` 的"Phase 1 的六个包"有阶段限定，也未改。
4. **`params-changes.md` 的「现值」列**：没有按 2026-10-10 新导出逐条重算，
   已在页首明确标注"引用前先核"。本票不把未核的数写成已核。
5. **`operation_manual.md` 未回链 `first-flight-config.md`**：该页自己写明"**未裁定，留着**"，
   属操作者裁定范围，本票不动。
6. **真正的未完成项**（不是过期状态词）：动捕外参 xy 仍未做、桨的转向仍未确认、
   `CA_ROTORx_PX/PY` 实测几何仍未量、`mocap-setup.md` 的部分前提仍未证等 ——
   这些是事实上的 open item，保留原样。
7. **`docs/report/chapters/**`、`.scratch/**` 历史票面、`tools/*/evidence/**`、`docs/assets/**`**：
   未改。工作树里这些路径的改动属于并行会话/票 16/票 17 的在制品。

---

## 4. 验证命令与结果

```bash
# 链接与锚点
$ python3 tools/report/check_links.py
检查了 54 个文件，断链 0 处。

# 文档守卫（4 个文件）
$ python3 -m pytest run/tests/test_operation_manual.py run/tests/test_safety_paths_drill.py \
      run/tests/test_z0_menu.py run/tests/test_rec.py -q
185 passed in 14.71s

# 夹具正控（需要 ROS 环境）
$ source /opt/ros/humble/setup.bash && source install/setup.bash
$ python3 -m pytest run/tests/test_z0_check_live.py -q
5 passed in 8.66s
```

第一手事实的复核命令：

```bash
find src -maxdepth 2 -name package.xml          # 7 个
grep -n 'EXPECTED_PACKAGES' -A1 run/z0.py       # 6 个，不含 nmpc
git rev-parse --short v1.0 v2.0 v3.0 v4.0       # 5ff9709 175e298 cdddeb5 f137472
git tag -l                                      # 空
git reflog show v4.0 --date=iso                 # Created from main, 2026-10-17 08:55:17
wc -l < docs/assets/z0.params                   # 955
grep -cE '^[A-Z0-9_]+[[:space:]]' docs/assets/z0.params   # 911
grep -H '^Status:' .scratch/phase4/spec.md .scratch/phase4/issues/*.md
grep -n '0.15889999270439148' .scratch/phase4/issues/02-hover-thrust-backfill.md
grep -n '5.652' docs/control/tier-b-sitl-seam3.md
```

---

## 5. 未做 / 待裁定

- **`README.md` 四阶段表的 Phase 5 行**：按票面要求留给**票 16**，本票只在记录里留这条。
- **`params-changes.md` 的「现值」逐条重算**：未做；已在页首标注口径与快照时点。
  若要把「现值」升级为 2026-10-10 导出后的逐条核对，需要单独一轮对账。
- **`v4.0` 是否作为"当前版本"**：本票按分支事实与 Phase 4 收口更新了版本行
  （`git reflog` 可核）。如果操作者只想把 `v3.0` 当作"最后一个文档版本"，
  这是表述选择，可回退该行而不影响其余事实修正。
- **`operation_manual.md` 与 `first-flight-config.md` 的互链**：未裁定，保持原状。
