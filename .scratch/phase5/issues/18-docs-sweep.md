# 18: 文档整理与失效引用清扫

Status: done
Type: task

**What to build:** 清扫 `README.md`、`docs/**`、`operation_manual.md`、`run/README.md` 的失效链接、
过期状态词与明显矛盾的散文；产出 `docs/report/evidence/docs-sweep.md`。

**Blocked by:** —

## 范围

- **活文档**：`README.md`、`docs/**`、`operation_manual.md`、`run/README.md`、各包 README；
- **不扫在制品**：`docs/report/chapters/**` 由票 16 统一查；本票只新增
  `docs/report/evidence/docs-sweep.md`，不碰并行起草中的章节；
- **不动**：`.scratch/**` 的历史票面、`tools/*/evidence/**` 的归档证据、
  `docs/assets/{z0.params,papers,drill}/**` 的原始产物。

## 允许的改动

- 修失效的相对链接/锚点；
- 修过期状态词（"未 build""已收口"等），但必须带可核的锚；
- 把明显矛盾的散文按第一手证据更正；
- 被作废的结论按仓库惯例在**节首**标 ☠️ 并逐项点名，**不原地改数**。
- ⛔ 不改 `README.md` 的四阶段路线表；Phase 5 行由**票 16** 在最终审查通过后补
  （18 在记录里留一条"待 16 补"即可）。

## ⛔ 不允许

- 改历史票面、归档证据、`.scratch/` 里的结论；
- 把"没做"写成"做了"；把"wontfix"写成"通过"；
- 大段重写文档风格；
- 顺手改别的票的内容（发现矛盾就记录，交回对应票）。

## 做法

1. 跑链接检查（`tools/report/check_links.py` 或仓库现有工具），列断链；
2. 扫状态词与关键数字，逐条回代码/证据核实；
3. 每处改动记录：文件、位置、原文、改后、证据；
4. 跑文档守卫测试（如 `python3 -m pytest run/tests/test_operation_manual.py -q`；逐文件跑）。

## 产物

- 文档改动；
- `docs/report/evidence/docs-sweep.md`：改动清单 + 未改动但记录在案的矛盾 + 证据。

## 检查清单

- [x] 链接检查通过
- [x] 每处语义改动都有证据
- [x] 作废标记在节首，逐项点名
- [x] 历史票面/归档证据未被改
- [x] 文档守卫测试通过
- [x] 没有改 README 四阶段表；只在记录里留"待票 16 补 Phase 5 行"

## 验收

- 操作者审过才 `done`；审阅记录写进 `## Comments`。

## Comments

### 2026-10-18 —— 清扫完成，转 `ready-for-human`

**改了什么**（逐条原文/改后/证据见 [`docs/report/evidence/docs-sweep.md`](../../../docs/report/evidence/docs-sweep.md)）：

- 新增 `tools/report/check_links.py`（无第三方依赖；相对路径、中文标题锚点、重复标题后缀、
  `<a>/<span id>`、跨行链接；排除代码块）。票 16 可复用。
- 修失效锚点：`docs/control/mismatch-sweep.md` 的 `[§附.1](#附1)` 补 `<span id="附1"></span>`。
- 参数落地状态：`MPC_THR_HOVER = 0.1589` 于 2026-10-13 经 mavros `param set` 写入并
  `force_pull` 读回 `0.15889999270439148`；三条配置参数于 2026-09-23 写入并读回。
  相应更新 `params-changes.md`、`first-flight-config.md`、`ladder-acceptance-criteria.md`、
  `phase1/phase2-implementation.md`、`params-analysis.md`、`bench-runbook.md` 的"待写入"。
- 版本/包数/spec 数：`README.md` 当前版本改为 `v4.0`（`f137472`，2026-10-17）、包数 6→7，
  `CONTEXT.md` / `code-inventory.md` 同步；**README 四阶段表未动**（Phase 5 行待票 16 补）。
- `z0.params` 计数：928/914 标注为 2026-09-21 快照；文件已于 2026-10-10 重导出（现算 955 行/911 条）；
  `params-changes.md` 的「现值」未按新导出逐条重算，已在页首明示。
- Phase 4 / 票状态：`tier-a-sitl-hover.md` 三处"待飞"改为"未飞；票 16 已裁定收口为 `done`，
  结论由票 25 给出"；`phase2-implementation.md` 的"S5 仍未开始"改为"当时仍未开始"并补 2026-09-29
  已跑（票 11 `done`，判据并非全绿）。
- 测量口径：`offline-ocp.md` / `phase2-implementation.md` 明确 5.652 ms 是**逐拍求解耗时**，
  不是整周期墙钟。

**验证**：

```bash
$ python3 tools/report/check_links.py
检查了 55 个文件，断链 0 处。
$ python3 -m pytest run/tests/test_operation_manual.py run/tests/test_safety_paths_drill.py \
      run/tests/test_z0_menu.py run/tests/test_rec.py -q
185 passed
$ source /opt/ros/humble/setup.bash && source install/setup.bash
$ python3 -m pytest run/tests/test_z0_check_live.py -q
5 passed
```

**未做 / 待裁定**：

- `README.md` 四阶段表的 Phase 5 行：**待票 16** 在最终审查通过后补。
- `params-changes.md` 的「现值」未按 2026-10-10 新导出逐条重算（已在页首标注）。
- `operation_manual.md` 与 `first-flight-config.md` 的互链：该页自己写明"未裁定，留着"。
- `v4.0` 作为"当前版本"是表述选择；若操作者不认，可只回退 `README.md` 版本行。
- 未触碰 `docs/report/chapters/**`、`.scratch/**` 历史票面、`tools/*/evidence/**`、
  `docs/assets/**`；工作树里这些路径的改动属于并行会话。
- 本票**未提交**（按协调者要求，不 `git add/commit`）。

### 2026-10-18 —— 操作者裁定收工

操作者 2026-10-18 指令「全部 tickets 收工」，并对 README 修订与装配结果放行；本票按「做完了」义把 `Status` 转 `done`。收口过程见 [票 20](20-closeout-and-v5.md)。
