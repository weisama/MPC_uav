# 11: 第 6 章 项目结构与代码结构

Status: done
Type: task

**What to build:** `docs/report/chapters/06-code-structure.md`。

**Blocked by:** —

## 内容

- **仓库结构**：colcon 根 = 仓库根；`src/` 下 7 个 ROS 2 包；`bag/`、`docs/`、`run/`、`tools/` 的职责；
  目录树在写稿时**现生成**并标快照时点，不抄 README 的旧树；
- **7 个包**（职责、入口、依赖、谁调用谁）：
  - `bench`：测试夹具（fake mavros/PX4、plant、scenario），不是飞行代码；
  - `bringup`：launch/编排，无飞行逻辑；
  - `control`：FSM、offboard 位置控制、arm/takeoff/track/land；
  - `mocap`：VRPN 刚体 → 相对位姿（ENU/FLU），输出 vision pose；
  - `nmpc`：MPC 节点 + 零 ROS 核心层（sign_map/commands/model/ocp/params）；
  - `traj`：解析参考轨迹（悬停/正方形/8 字/起飞），只发参考；
  - `vrpn_listener`：VRPN → ROS 2 桥；
- **数据流**：从 `/traj/reference` 到 `/mavros/setpoint_raw/*` 的两条路径；
- **MPC 的层分界**：`node_core.py`（无 ROS，可单测）vs `mpc_node.py`（话题/参数/日志）；
  说清改哪一层要不要 build（`mpc_node.py` 要，`node_core.py` 不要，按仓库既有经验）；
- **测试布局**：`src/*/test/`、`run/tests/`、`tools/*/test_*.py` 各自测什么；
- **证据目录**：`bag/`、`tools/*/evidence/`、`docs/assets/` 的角色（详细索引见第 4/10 章或资料索引）。

## 来源

- `src/*/package.xml`、`src/*/README.md`；
- `docs/reference/code-inventory.md`；
- `docs/control/phase2-implementation.md`；
- `README.md` 的目录树（只作对照，不直接抄）。

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

- [x] 目录树是写稿时现生成的，标了快照时点
- [x] 7 个包的职责与入口准确；不把 `bench` 说成飞行代码
- [x] 说清"零 ROS 核心层 / 节点层"的分界与测试含义
- [x] 不抄会过期的文件数/用例数；需要计数就现算
- [x] 中文母语写作，无裸词 `MPC`

## 验收

- 操作者审过才 `done`；审阅记录写进 `## Comments`。

## Comments

### 2026-10-17 交付记录（Claude Code）

**本章摘要**
- 仓库根即 colcon 根；`src/` 下 7 个包，`bag/`、`docs/`、`run/`、`tools/` 分职；目录树按 2026-10-17 16:18 UTC 快照现生成。
- 7 个包逐个给职责、入口、依赖与起链关系；`bench` 明确为测试夹具，`bringup` 明确不含飞行逻辑。
- 两条数据流按五段对照：`traj` → `control` →（位置 setpoint ｜ `/control/target` → `nmpc`）→ mavros → PX4。
- MPC 分核心层/节点层：核心层零 ROS，节点层只做话题、参数与日志。
- build 规则按「进程实际解析到哪份文件」说明：`ros2 run` 的入口脚本看 `install/`，包模块看 `PYTHONPATH` 与安装方式。
- 测试按包内 seam 1 / seam 2、`run/tests/` 文档守卫、`tools/*/test_*.py` 离线工具三层说明；证据目录按 `bag/`、`tools/*/evidence/`、`docs/assets/`、`docs/report/` 分工。

**引用/证据**
- `src/*/package.xml`、各包 `CMakeLists.txt` 与 `README.md`；`docs/reference/code-inventory.md`。
- `docs/control/phase2-implementation.md`、`docs/control/sign-map.md`、`docs/PROJECT.md`。
- `src/nmpc/nmpc/__init__.py`、`mpc_node.py`、`node_core.py`；`src/bringup/launch/z0.launch.py`；`src/bench/README.md`。
- `run/README.md`、`run/tests/`、`bag/README.md`、`docs/report/data/README.md`。
- 票 20（`.scratch/phase2/issues/20-vertical-channel-mismatch-offset.md`）与票 13（`.scratch/phase4/issues/13-mavros-downlink-switch.md`）。

**未确定项 / 请审阅**
1. `README.md` 的工作区表写「6 个包」，但同一棵目录树列了 7 个；本章按 `src/*/package.xml` 现算的 7 个写，并在 6.2 节点明差异。README 是否随文档清理票 18 修正，请操作者定。
2. 文档内部有一处措辞冲突：`src/nmpc/README.md:4` 写「姿态环与混控仍留在飞控里」，而 `docs/control/sign-map.md:23` 与 `docs/PROJECT.md` 写 PX4 只跑角速率环与混控。本章按后者写（6.4 节表 6-5）。请确认 `nmpc/README.md` 那句是否应改写。
3. build 规则：仓库记录的常规安装是拷贝式，但写稿时本机 `install/nmpc` 恰好是软链。6.5.2 节把规则写成「入口脚本看 `install/`，包模块看 `PYTHONPATH`」，并注明软链安装的例外；请确认这是现场希望保留的口径。
4. 表 6-2 的包数/测试文件数（7/26/5/23）是 2026-10-17 16:18 UTC 快照，命令随表给出；改动仓库后需重算。
5. 本章只讲结构，不含控制性能读数；涉及频率的两个数（约 30 Hz、约 190 Hz）沿用第 5 章的测量口径。

**复核后已修正**
- 假 PX4 的归属按 `src/bench/bench/fake_px4.py` 文件头改：它现在没有 DUT，只剩 `bench` 自测与 `Scenario(px4=True)` 的回退项会起它；`tools/px4-sitl/` 跑的是真 PX4 SITL，不 import 它。
- `bag/` 的描述按 `bag/README.md` 改：`MANIFEST.md` 是根目录总表，`.ulg` 归档在 `fcu/` 且不保证每份都有。
- 测试环境补了 `~/px4_msgs_ws`：`bench` 在模块级 import `px4_msgs`；缺了它，包内 seam 1 会在 collection 阶段报错，`run/tests` 与 `tools/*` 里依赖它的用例则被 `importorskip` 跳过。
- `bench` 与 `nmpc` 的依赖表按实际 import 补了 `std_msgs`；两包的 `package.xml` 目前没有声明它，这是仓库侧的 manifest 缺口，本章未改。
- seam 1 的断言范围删掉了假 PX4：节点级测试走假 mavros 的对应 setpoint 话题，假 PX4 只剩 `bench` 自测与保留的回退口径。
- 6.2 与 6.5.1 的一句话总结改准：前者限定为「五个飞行包」，后者写明零 ROS 边界尚缺 `mavros_msgs` 断言。
- `test_mavros_interface.py` 与 `test_preflight_check.py` 的归类改为消息契约测试与纯逻辑/参数/`ast` 测试，不再算 seam 1。

**字数**：17,532 字符（`wc -m`，含 Markdown 与表格），280 行；13 个叶子内容小节，13 行一句话总结。

**审阅**：待操作者审阅；未过审不转 `done`。建议重点核对 6.4 的两条数据流与 6.5.2 的 build 规则是否符合现场口径。

### 2026-10-18 —— 操作者裁定收工

操作者 2026-10-18 指令「全部 tickets 收工」，并对 README 修订与装配结果放行；本票按「做完了」义把 `Status` 转 `done`。收口过程见 [票 20](20-closeout-and-v5.md)。
