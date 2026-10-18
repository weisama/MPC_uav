# 19: 资料索引

Status: done
Type: task

**What to build:** 一份资料索引，覆盖论文、演练日志、工具证据与录制包；供第 4/10 章引用。

**Blocked by:** —

## 范围

- `docs/assets/papers/`（6 篇 PDF + README）；
- `docs/assets/drill/`（安全路径演练日志）；
- `tools/*/evidence/`（各分析工具的 JSON/日志）；
- `bag/`（六份当前包 + `old/` + 参数导出 + MANIFEST）；
- `docs/assets/z0.params`、`bag/z0-params-*.params`。

## 产物

`docs/report/evidence/material-index.md`，表列：

`路径 | 是什么 | 日期/版本 | 被哪一章引用 | 备注`

- 论文写英文标题 + 中文译名；
- 证据 JSON 写"哪个工具、哪次运行、判什么"；
- 录制包写场景、控制器、时长、是否完整；
- `bag/old/` 单独标注"历史/被取代"，不参与主结论。

## ⛔ 禁止

- 不搬、不删、不改任何原始文件；
- **不读、不摘录 `docs/LOCAL-NOTES.md` 的凭据**；如需提及，只写"含本机凭据，报告不引用"；
- 不把索引写成第二份结论：只写"是什么/被谁引用"。

## 检查清单

- [x] 指定目录的每个文件都有一行
- [x] 每行有日期/版本或"未标注"
- [x] 被哪一章引用写清楚（没有引用就写"未引用"）
- [x] 没有凭据、没有个人信息
- [x] 不复制原始文件

## 验收

- 操作者审过才 `done`；审阅记录写进 `## Comments`。

## Comments

**2026-10-18 实施记录**

- 产物：`docs/report/evidence/material-index.md`，共 253 行数据。
- 覆盖：
  - 235 个逐文件条目，分布在 23 个目录：`docs/assets/papers/`（7 个文件）、`docs/assets/drill/`（12 个文件）、`tools/*/evidence/`（216 个文件、21 个子目录，含 `__pycache__` 里的 1 个 `.pyc`，已注明是生成物）。
  - `bag/` 按包级覆盖：9 份当前包、3 份 `old/` 历史包、`MANIFEST.md`、`bag/README.md`、`bag/old/README.md`；包内 `metadata.yaml`、`session.json`、`*.log`、`fcu/*.ulg` 与 `INDEX.md` 写在「备注」列。
  - 参数导出 3 份：`docs/assets/z0.params` 与 `bag/z0-params-2026-09-29.params`、`bag/z0-params-2026-10-10.params`。
- 验证方式：
  - 目录覆盖用 `os.walk` 枚举范围文件（235 个），与表格路径做集合比对，缺行/重复行断言为 0。
  - 「被哪一章引用」由 `grep` 对生成时的 `docs/report/chapters/**` 与 `docs/report/generated/**` 现查；`generated/` 的图注与指标并入第 10 章。最终生成前按要求重查了一次 `docs/report/chapters/`，当时 `00-summary.md` 已出现但未点名任何范围文件，故引用列无需改动。
  - 表格逐行校验 5 列均非空。
- 未标注：1 行（`docs/assets/papers/README.md`，文件本身无日期/版本）。
- 未引用：214 行；目录级引用与文件级引用的区别写在「备注」列。
- 未做项：
  - 操作者审阅（本票保持 `ready-for-human`）。
  - `docs/LOCAL-NOTES.md` 按票面要求未读、未摘录；索引只写「含本机凭据，报告不引用」。
  - `bag/` 未逐文件展开（票面要求写场景、控制器、时长、是否完整，已按包级给出）。

### 2026-10-18 —— 操作者裁定收工

操作者 2026-10-18 指令「全部 tickets 收工」，并对 README 修订与装配结果放行；本票按「做完了」义把 `Status` 转 `done`。收口过程见 [票 20](20-closeout-and-v5.md)。
