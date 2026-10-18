# 09: 第 4 章 参考项目与资料来源

Status: ready-for-human
Type: task

**What to build:** `docs/report/chapters/04-references.md`。

**Blocked by:** —

## 内容

- **开源项目**（每个写：是什么、借了什么、没借什么、钉在哪个 commit）：
  - acados（求解器/代码生成）；
  - agilicious（敏捷四旋翼硬件/软件）；
  - Fast-Drone-XI35（最接近的机体与 FSM 参考）；
  - OmniNxt（全向紧凑空中机器人）；
  - Sim2real-UAV-Params（参数辨识）；
  - MPC_uav（模型预测控制参考实现）；
  - px4-mpc（10 状态 OCP 的直接参考）；
  - Nxt-FC（厂商飞控/固件仓库）；
- **论文**（6 篇，标题保留英文，附中文译名与用途）：
  Sun2022 TRO（NMPC vs DFBC）、Foehn2022 SciRob（Agilicious）、
  Torrente2021 RAL（Data-Driven MPC）、Houska2011 Automatica（RTI）、
  Houska2011 OCAM（ACADO）、Liu2024 IROS（OmniNxt）；
- **证据分层**：哪些结论是第一手读源码、哪些是推断；引用 `docs/reference/open-source-survey.md`
  与 `refs-manifest.md` 的 §0 教训；
- **不**把参考项目说成"我们的实现"；明确移植边界与改动。

## 来源

- `docs/reference/open-source-survey.md`；
- `docs/reference/refs-manifest.md`（版本钉住、固件真身）；
- `docs/assets/papers/README.md` + 6 篇 PDF；
- `docs/reference/code-inventory.md`（逐文件取舍）；
- 票 19 的资料索引（若已完成）。

## 投影用一句话总结（2026-10-17 追加）

每个叶子内容小节（没有子标题的 `##`/`###`；独立「小结」节同样适用）**末尾**加一行，且只加一行：

`<big>**一句话总结：……**</big>`

- 一句话，建议 20–50 字、最多 60 字；只概括本节结论，能脱离上下文朗读。
- 不写“本节介绍……”；不引入本节之外的数、不引用图号/表号；不写“效果良好/基本通过”。
- 不单独成标题，不进目录。细则见 [spec.md](../spec.md) §8。

## 跨章阅读（2026-10-18 追加）

写作前**可以并建议**先读其他章节，尤其是相邻章和已落地章，用来对齐术语、符号、详略与过渡：

- 先读 `docs/report/chapters/` 下已存在的章节，再读本章票与相邻章的「内容」，明确本章边界；
- 术语/符号以 [CONTEXT.md](../../../CONTEXT.md) 为准；其他章已定下的写法优先沿用；
  发现冲突记进本票 `## Comments`，不改别人的章；
- 只借组织方式与措辞，不整段搬运；本章结论仍由本章自己给证据；
- 别的章还没写完时不等待、不阻塞；写完后再做一次交叉核对。

## 检查清单

- [ ] 写作前读过相邻章与已落地章；术语/符号与它们一致，冲突已记进 Comments

- [x] 每个叶子小节末尾有且只有一行 `<big>**一句话总结：……**</big>`，且句子不引入本节之外的数

- [x] 每个项目有 commit/版本锚
- [x] "借了什么"与"没借什么"分开写
- [x] 论文标题保留原文；中文译名放括号
- [x] 第一手 vs 推断有标记
- [x] 不出现未核实的版本号；版本以 `refs-manifest.md` 为准
- [x] 中文母语写作，无裸词 `MPC`（唯一命中见 Comments：3 处英文原题内）

## 验收

- 操作者审过才 `done`；审阅记录写进 `## Comments`。

## Comments

### 2026-10-17 —— 章节初稿（agent）

**状态：`ready-for-human`。** 正文已写完，等操作者审阅；未过审前不进入装配。

**本章摘要（≤10 行）**

- 交付 `docs/report/chapters/04-references.md`：初稿 392 行 / 8,385 个汉字；两轮复核后为 381 行 / 8,676 个汉字（2026-10-17 脚本现算，含链接与标点；不含代码块）。
- §4.1 证据分层：源码、运行/全文、推断、二手四级；并复盘三次被证据推翻的结论（私有仓库 404、README 当全文、假定版本字段编码）。
- §4.2 八个开源项目逐项写“是什么 / 借了什么 / 没借什么 / 钉住的 commit”：acados、agilicious、XI35、OmniNxt、Sim2real、MPC_uav、px4-mpc、Nxt-FC 与 z0 固件。
- §4.3 六篇论文：英文原题保留、中文译名放括号，逐篇写内容、用途与边界；论文数字只作为“论文报告了什么”。
- §4.4 移植边界：逐字节 vendored 的只有 `vrpn_listener`；`px4-mpc` 是改写，XI35 是重写，acados 是外部依赖；另列 guard、INDI、XI35 推力模型与固件等“只有概念、没有代码”的项。
- §4.5 复现命令与来源清单；§本章引用清单给 12 个可点击的仓库内锚点。

**引用 / 证据清单**

- 主来源：[`refs-manifest.md`](../../../docs/reference/refs-manifest.md)、[`open-source-survey.md`](../../../docs/reference/open-source-survey.md)、[`code-inventory.md`](../../../docs/reference/code-inventory.md)、[`papers/README.md`](../../../docs/assets/papers/README.md)。
- 第一手源码：`~/refs/` 下各仓库的钉住 commit；6 篇 PDF 的 `pdftotext` 全文（标题、卷期、作者、摘要中的 78%、70%、14 m/s、4 g、5 g、70 km/h 均已回原文核对）。
- 本仓库锚点：[`phase2-implementation.md`](../../../docs/control/phase2-implementation.md)、[`acados-orin-build.md`](../../../docs/control/acados-orin-build.md)、[`fsm.hpp`](../../../src/control/include/control/fsm.hpp)、[`model.py`](../../../src/nmpc/nmpc/model.py)、[`VENDORED.md`](../../../src/vrpn_listener/VENDORED.md)。
- 票 19（资料索引）仍为 `ready-for-agent`，未完成，本章未引用它；来源改由上述四份文档直接承担。

**未确定项（正文已写明，未当作结论）**

- `MPC_uav` 票面的括注“模型预测控制参考实现”与事实不符：它就是本仓库，本章按“本项目自己的仓库”写，并锚在 Phase 4 收口提交 `21eb4a7`。
- `papers/README.md` 旧索引称 OmniNxt 为“倾转旋翼、带舵机”；PDF 全文核实为常规四旋翼，本章按更正后的表述写，并点名这次更正。
- `refs-manifest.md` 记 Nxt-FC 为 51 个文件，2026-10-17 用 `find` 现算为 49 个普通文件；本章写现算值并标注日期。
- Sim2real 仓库没有 LICENSE 文件；vendored 的 `vrpn_listener` 的 `package.xml` 许可证字段仍为 `TODO`。本章只陈述事实，未替上游补许可证。
- 本章不写 z0 的跟踪误差与求解耗时；这些读数留给第 10 章从数据生成。

**自检**

- 叶子小节 21 个（4.1.1–4.1.2、4.2.1–4.2.8、4.3.1–4.3.6、4.4.1–4.4.3、4.5、本章引用清单），每个末尾恰好 1 行总结，共 21 行；父小节不加。脚本按标题层级解析。
- 总结句汉字数 26–39 字，均在 20–50 字区间；未引入本节之外的数、未引用图表号、未用“效果良好 / 基本通过”。
- 相对链接 29 个，全部存在（脚本核对；复核后新增 3 条教训的就地来源链接）。
- 裸词 `MPC`：正则 `RE_BARE_MPC` 命中 3 处，全部在必须逐字保留的英文原题内（`Nonlinear MPC`、`Data-Driven MPC`、`nonlinear MPC`）；中文正文与标题一律用“模型预测控制”或 `MPC（模型预测控制）`。
- 本章为纯文档改动，不新增代码与测试。

### 2026-10-17 —— 双轴复核与全量测试（agent 记录，非操作者验收）

**规范轴（15 条意见）**：逐条回代码/仓库核实后全部成立，已改。要点：

- `4.1.1` 标题与首句的“三类”改为“四类”，与表格四行、总结句一致。
- `4.2.6` 版本分支补齐 `v4.0 → f137472`，并纠正 `v2.0` 的性质（Phase 2 票面全部 `done`，阶段 spec 未收口）。
- Phase 5 的报告数据管线不再锚到 `21eb4a7`：控制与节点锚 Phase 4 收口，数据管线、图与结果表归第九、十章。
- `px4-mpc` 分支计数改为“本地快照 22 条 + 上游新增 1 条（`dev-elias_branch`）”；`model.py` 的复用措辞改为“方程与约定原样复用、文件经过改造”，不再写“逐字沿用”。
- `Nxt-FC` 的 51 与 49 已对账：51 是 `git ls-tree -r` 条目数（49 个普通文件 + 2 个 gitlink）。
- 求解器配置与四元数公式不再在本章重复展开，留给第 8 章；`ACADO` 原题破折号按 PDF 逐字保留。

**事实轴（6 条意见）**：全部回第一手核实并改：

- XI35 的非线性推力公式只在 `ctrl_param_fpv.yaml` 注释里；`controller.cpp` 实际是 `u = a_z / thr2acc_` 加 `volt2HoverPerOverM0` 电压补偿，`K1`–`K3` 读入未使用。
- Foehn2022 作者单位只有 UZH-RPG，删去 ETH 表述。
- `nxt-v1` 的启动脚本同时启用 BMI088、ICM42688P 与 BMP388，删去“单 IMU”定性。
- 15 A 是 VTC6 的 Mooch 独立实测，出处是 `hardware.md` §6，不是 `BOM.md`。
- px4-mpc 的 LICENSE 署名 JaeyoungLim，源文件头普遍写 PX4 Development Team；正文已注明按文件头保留。

**全量测试**

- `source /opt/ros/humble/setup.bash && source install/setup.bash`，`ACADOS_SOURCE_DIR=$HOME/acados`、`LD_LIBRARY_PATH=$HOME/acados/lib` 均设。
- `python3 -m pytest -q`：**1098 passed, 7 errors**（21 分 22 秒）。
- 7 个 error 全部在 `tools/report/test_report_traj_figures.py`（并行会话未入库的 Phase 5 图形测试），setup 阶段报 `ValueError: Unknown projection '3d'`，属本机 matplotlib 缺 3D 投影，与本章及 `src/` 无关。
- 章节自检最终值：21 个叶子小节 / 21 行总结；总结句汉字 26–39 字；29 个相对链接全部存在；裸词 `MPC` 仅 3 处英文原题。

### 2026-10-18 —— 按操作者意见精简（agent）

操作者 2026-10-18 的五条修改意见已全部落地：

1. 删除 `4.1.2`（三次被证据推翻的结论），`4.1` 合并为单个叶子节，只保留证据四级与四条引用纪律。
2. 开源项目表新增“来源实验室”列：acados 弗莱堡大学 syscop（Diehl 组）、agilicious UZH-RPG、XI35 哈工大 HIT-K1928-LAB、OmniNxt 与 Nxt-FC 港科大 Aerial Robotics Group、Sim2real 为未注明实验室的个人仓库、MPC_uav 为本项目、px4-mpc 为 KTH / ETH ATMOS 团队。
3. 每个项目的“借了什么 / 改了什么 / 没借什么”压缩为短段落，删去文件数、逐行细节与重复数据，只留结论与关键锚点。
4. 论文节每篇新增“出处与实验室”，并写明与开源项目的对应关系：Foehn2022 ↔ agilicious；Liu2024 ↔ OmniNxt 与 Nxt-FC；Houska2011 两篇 ↔ acados（RTI 与 ACADO 前身）；Sun2022、Torrente2021 与 agilicious 同组但无独立仓库。
5. `4.2` 标题改为“参考的开源项目”。
6. 开源项目表的“项目”列改为指向各自 GitHub 仓库的可点击链接（`5c86f33`）。

**自检（2026-10-18）**

- 20 个叶子小节 / 20 行总结；总结句汉字 20–32 字；父小节不加。
- 26 个相对链接全部存在；裸词 `MPC` 仅 3 处英文原题。
- 篇幅：316 行 / 4,638 个汉字（精简前 381 行 / 8,676 字）。
- 实验室归属依据：acados `docs/index.rst`；agilicious PDF 首页与 README；OmniNxt/Nxt-FC README；px4-mpc `package.xml` 的 `padr@kth.se` 与源码 `jalim@ethz.ch`；XI35 见 `open-source-survey.md`。
