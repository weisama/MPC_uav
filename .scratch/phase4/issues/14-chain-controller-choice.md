# 14: z0.py 第 8/9 项起链前选「谁在上位控制飞机」—— offboard 控制 / 模型预测控制

Status: ready-for-agent
Type: task

**What to build:** 给 `run/z0.py` 的两条一键起链（菜单 **8 = `up_manual`**、**9 = `up`**）
在**起链前加一句选档**，让操作者按数字选「这一趟谁在上位控制飞机」：

| # | 选什么 | 起链的尾巴（公共前缀 `mavros → vrpn → mocap` 之外） |
|---|---|---|
| 1 | **不起控制器**（手动遥控飞） | `()` |
| 2 | **offboard 控制**（`control`，Phase 1 的 (A) 级） | `traj → control` |
| 3 | **MPC（模型预测控制）**（`nmpc` 的 `mpc_node.py`，参考源 `hold`） | `mpc`（**不起 traj**） |
| 4 | **模型预测控制**（`nmpc` 的 `mpc_node.py`，参考源 `traj`） | `traj → mpc` |

★ **默认值照菜单项各给一个**：第 8 项默认 **1**、第 9 项默认 **2**
⟹ **直接回车 = 今天的行为，一个字不变**（1 就是现在的第 8 项、2 就是现在的第 9 项）。

**为什么**（操作者 2026-10-16 原话）：「给 z0.py 脚本的 8 和 9 选择添加新的选项，可以选择启动
offboard 控制还是 mpc 控制，以至于我可以手动起链和录制以完成票 [10](10-ladder-closed-loop-and-free-flight.md)」。
票 10 的阶梯 3/4 要的是 **MPC（模型预测控制）自己飞**，而今天两条一键里唯一的控制器格是 `control`
（(A) 级）—— 想跑模型预测控制只能手打一条 `ros2 run nmpc mpc_node.py`，起链与录制两头都接不上。

**Blocked by:** 无。

## 范围

**只动 `run/z0.py` 的「选哪条链」这一层**：新增一条命令出口（`mpc`）、把两条一键的尾巴变成**选出来的**、
把这一格的话术按档写清。**不新增任何会动飞机的入口**（见下），
**不动** `mpc_node.py` / `control_node.cpp` / 任何参数文件。

## ★★ 这一轮**不解决**的三件事（都**不静默**，写在屏上 + 本票里）

1. **模型预测控制档下没人解锁、没人切 OFFBOARD。** `mpc_node` 的闸门要求 `armed + mode == "OFFBOARD"`
   （[`node_core.py:177-190`](../../../src/nmpc/nmpc/node_core.py#L177-L190) 起的那条闸门），
   而它**自己不 arm、不 set_mode**；闸门关着时它**连心跳都没有**
   （[`mpc_node.py:1002-1026`](../../../src/nmpc/nmpc/mpc_node.py#L1002)，"不发 setpoint" = 飞控 ~1 s 后
   判心跳丢失 ⟹ **也进不去 OFFBOARD**）。这件事今天只有两条路：`control` 的 FSM（遥控器开始键）
   或 `offboard_smoke.py`（`/mavros/cmd/arming` + `/mavros/set_mode`）。
   ⛔ **不许**把这两条塞进 `z0.py` —— `FORBIDDEN_IN_COMMANDS`
   （[`z0.py:538-543`](../../../run/z0.py#L538)）挡的正是 `ros2 service call`，
   而那正是"不提供任何让飞机动的入口"这条边界的落点。⟹ 起链之后**照旧手打**，
   本脚本只把"在哪"指出来。
2. **模型预测控制 + traj 档下没人发 `/traj/start`。** 它是 `control` 进 `TRACK` 那一刻自己发的
   （[`control_node.cpp:40`](../../../src/control/src/control_node.cpp#L40)）；`fly.py` 只发
   `/traj/select`（[`fly.py:9-13`](../../../src/bringup/scripts/fly.py#L9)），`mpc_node` 只**订阅**
   `/traj/reference`。没有它，`traj` **一帧都不发** ⟹ `traj` 档的模型预测控制只会看到断流、就地冻住
   （`traj_stream_gap_s = 0.5 s` 的 `freeze`，见 [`node_core.py:241-251`](../../../src/nmpc/nmpc/node_core.py#L241)）。
   ★ 这一条**记在这里当交接项**，不在本票实现：谁来发它是"谁解锁、谁切 OFFBOARD"的同一个归属问题
   （[`safety-paths-drill.md` §现状](../../../docs/control/safety-paths-drill.md) 里那条"没有归属"）。
3. **`fly.py` 在模型预测控制档下的状态栏是空的**：它显示 `/control/state` 与 `/control/traj_progress`，
   而模型预测控制档下没有 `control` 在跑 ⟹ 那两格不会有数（选轨迹那一半照常可用）。

## 设计

### 选档怎么问（起链的那一刻，**默认值看菜单项**）

```
起链后**谁在上位控制飞机**？（输数字；直接回车 = <默认那一档>）
  1 = 不起控制器 —— 只到动捕 + mavros，起完直接用遥控器手动飞
  2 = offboard 控制（control，(A) 级）—— 起 traj；起完选轨迹 + 按开始键
  3 = MPC（模型预测控制）—— mpc_node.py，参考源 hold；定点保持，不起 traj（阶梯 3 的系绳悬停用这档）
  4 = 模型预测控制 —— mpc_node.py，参考源 traj；**起 traj**，起完选轨迹（阶梯 4 飞轨迹用这档）
```

★ 第 3 档那一行是**本文件首处**提到本项目的那个控制器 ⟹ 按 `CONTEXT.md` 第一节写
`MPC（模型预测控制）`（全称即"模型预测控制"）；第 4 档起写全称，**不写裸词**。

- 打错的**再问一遍**（不猜、不吞）；`EOFError`（管道/重定向）⟹ 走默认 ——
  与 `ask_confirm` 同一条纪律。
- **`--dry-run` 不进入提问**：先把**这一项的默认那一档**整条摊开（命令 + 判据 + 起完的话），
  再另起一块列出**另外三档**的命令（选到才起）。★ 两块之间有一行分隔标题，
  免得"摊开的命令"与"选别的档才起的命令"混成一张表。

### 命令出口仍然只有 `ACTIONS` 一张表

- 新增一条**不进菜单**（`menu=False`，不占菜单号）的动作 `mpc`：
  `ros2 run nmpc mpc_node.py`，带一条可选参数 `reference_source_traj`
  （`--ros-args -p reference_source:=traj`）—— 第 4 档带上它。
  ★ `--ros-args -p` 这个写法**不是这里发明的**：
  [`first-flight-config.md:225-226`](../../../docs/control/first-flight-config.md#L225) 就是
  `ros2 run nmpc mpc_node.py --ros-args -p tau_s:=0.0`（而 `preflight_check.py` 的 `-p` 是它**自己**
  argparse 的选项，两个不是一回事）。
- `down`（菜单第 10 项）的白名单要**加一条 `mpc`**：起链能起的，清除就必须能扫到 ——
  这是 [`test_z0_menu.py`](../../../run/tests/test_z0_menu.py) 里那条不变式。
- 第 8/9 项的**标题与提示**要改成"起前选控制器"，否则第 8 项还写着"手动遥控飞"而它已经能起模型预测控制。

### `mpc` 那一步"起来了没有"的判据 = 它自己的启动日志

`mpc_node` 在 `armed + OFFBOARD` 之前**一帧都不发**（`_tick` 在 `cycle.x0 is None` 时直接 return；
`_viz_tick` 又要求 `last_prediction()` 非 `None`）⟹ **话题面上没有"我起来了"这个事实**。
所以这一步的判据落在**子进程的日志文件**里那一行 `MPC（模型预测控制）节点启动：`
（[`mpc_node.py:724`](../../../src/nmpc/nmpc/mpc_node.py#L724)）——
一条**正向**的判据，比"起了就当成功"强：节点因参数校验/acados 起不来时，这一条当场红并打出日志尾巴。
★ 代价：`describe()` 那一行的措辞改了，这里会变成"等超时"（**红**，不静默）。
⟹ 配一条用例断言那个 marker **仍出现在 `mpc_node.py` 的源码里**。

## 验收

★ 每条的**证据**写在方括号里（用例名 / 现算命令）；**没做**的照旧留空，不留"看起来做过"的空。

- [x] 第 8 / 9 项起链前各问一句「谁在上位控制飞机」，四个档都能选到；**空回车 = 该菜单项的默认档**
      〔`test_起链问的那一句_四档都列出来_且标出默认`、`test_起链问的那一句_空回车走默认_打错的再问一遍`、
      `test_起链先重入_source_再问选档`；屏上现算：`python3 run/z0.py up` 起链那一刻〕
- [x] 默认档下的行为与今天**逐字一致**（第 8 项 = `mavros/vrpn/mocap`；第 9 项 = `+ traj/control`）
      〔现算：把 `HEAD` 那一版取到 `/tmp` 跑 `--dry-run`，与今天那一版 diff（只归一化
      `source …/ros_ws/install/setup.bash` 这个**在树内才出现**的前缀）——
      **第 9 项**：只差两行（标题行 + 新增的"★ 下面摊开的是默认那一档（第 2 档）"句），
      6 条命令与 6 条判据行逐字相同；
      **第 8 项**：另有一处**有意**的措辞改动 —— 收尾里"本流程**没有** traj/control"改成
      "这一档**没有** traj/control"（它现在是四档之一，说"本流程"会指错）。
      〔用例：`test_四个档的尾巴_默认值_与_公共前缀`〕
- [x] 第 4 档（模型预测控制 + traj）的命令里确实有 `--ros-args -p reference_source:=traj`；第 3 档没有
      〔`test_只有_mpc_traj_档带那条_ros_args_参数`〕
- [x] `mpc` 那条命令**只**出现在 `ACTIONS` 里（一键按 key 取，不另抄）
      〔`test_mpc_那一步的日志判据仍出现在_mpc_node_源码里` 钉住 `ACTION_BY_KEY["mpc"].cmd`；
      `test_起链里没有会动飞机的入口` 钉住链里只写 key；`test_菜单十一条_…` 钉住它 `menu=False`〕
- [x] `mpc` 那一步等到启动日志那一行才算起来；等不到 ⟹ 打日志尾巴 + 起着的那几条**全部停掉** + 退出码 1
      〔`test_mpc_那一步等的是它自己的日志行_等不到就报红并停掉其余的`〕
- [x] `down` 的白名单能扫到 `mpc_node.py`（起链能起的，清除就能扫到）
      〔`test_清除目标表按名字扫_且把起链的每一条都盖住`〕
- [x] `--dry-run up` / `--dry-run up_manual` 仍然**一条都不起**，且把默认档与另外三档都摊开
      〔`test_dry_run_把四档的命令都摊开且默认那档排在前面`（提问与起进程都换成了抛异常的假件）、
      `test_dry_run_up_把顺序与判据全摊开_且一条都不起`〕
- [x] 起链的安全不变式照旧：动作表里没有任何 `ros2 service call` / `param set` / `topic pub` / `action send_goal`；
      一键起的命令**全部**来自 `ACTIONS`〔`test_动作表里没有任何会动飞机的入口`、
      `test_源码里不出现发布与服务调用`、`test_起链里没有会动飞机的入口`〕
- [x] 起链说完的话里，**模型预测控制档**要写清上面「这一轮不解决的三件事」该怎么办（在哪手打，别在这脚本里找）
      〔`test_模型预测控制两档的收尾都要说清_还差哪两步`：① 解锁/OFFBOARD（只指文档）、
      ② `/traj/start`（只在带 traj 的那一档）、③ `fly.py` 状态栏是空的（同上）〕
- [x] `operation_manual.md` 与 `run/README.md` 的菜单 8/9/10 描述跟着改；文档守卫（`test_operation_manual.py`）绿
      〔`test_operation_manual.py` 5 例 + `test_rec.py::test_文件头引用的菜单号与动作表对得上`〕
- [x] `python3 -m pytest run/tests/test_z0_menu.py run/tests/test_operation_manual.py -q` 绿
      〔2026-10-16：三个守卫文件**分开点名**跑 = 178 例全绿（`test_z0_menu` 83 / `test_rec` 80 /
      `test_operation_manual` 5 / `test_safety_paths_drill` 10）；⚠️ `pytest run/tests` 整目录跑会
      **收集到 0**（`test_z0_check_live.py` 顶层 `pytest.skip` 的既有毛病，与本票无关）〕

## Comments

### 2026-10-16 —— 操作者四问的裁定（本文所据）

| 问 | 答 |
|---|---|
| 票 14 不存在（13 之后没有 14；历史上的 `14-mpc-node-mavros-downlink.md` 已在 `84a6f60` 并进 13 并撤销），怎么办？ | **先建本票再实现** |
| 选档挂在哪些菜单项上？ | **8 和 9 都可以手动输入数字选择** |
| 模型预测控制那一格的命令写成什么？ | **起链时问一句（选档）** ⟹ 本票把它做成上表里的第 3 / 第 4 档 |
| 模型预测控制档下的链骨架（重点是 `traj` 与那条没人发的 `/traj/start`）？ | **MPC（模型预测控制）+ traj 档时才起 traj** ⟹ 第 3 档不起 traj、第 4 档起 |

★ 本票是**新建**的：`.scratch/phase4/issues/` 里 13 之后没有 14（现算：
`ls .scratch/phase4/issues/`），历史上那张同名号的票（`14-mpc-node-mavros-downlink.md`）是
`70fac4b` 开、`84a6f60` 与另外五张一并撤掉（并成当时那张 `19`，后来又是现在的 `13`）。
**号沿用 ≠ 内容沿用** —— 别拿本文去对那一张。

### 2026-10-16 —— 第一手核过的四条（写进设计前就核了）

1. `mpc_node` **不 arm、不 set_mode**，闸门关着时**连心跳都没有** ⟹ 它在 offboard 之前是**惰性**的
   （上面「不解决的三件事」① 引的两处源码）。
2. `/traj/start` 的**唯一发布者**是 `control`（`control_node.cpp:40`；`fly.py` 明写不发）。
3. 菜单 8/9 就是 `up_manual` / `up` —— 现算：`python3 run/z0.py --list`。
4. `mpc_node` 的参考只有两档（`hold` / `traj`），`traj` 档能追的**就是 `traj` 包那三条**
   （`/traj/select` 1 悬停 / 2 正方形 1.0 m / 3 八字 2.0×1.0 m，峰速 0.5 m/s）。
   参考里只有**位置 + 速度 + yaw**（掩码 `2496` 把加速度那几位全忽略）⟹ 带持续加速度的轨迹只是近似追。

### 2026-10-16 —— 落地与证据（实现记录）

**落了什么**（`run/z0.py` 为主）：

- 四档写成一张表：`ControllerChoice(key/title/tail/closing/handoff/extra_optionals)` × 4，
  两条链共用（`CHAIN_CHOICES`）；`Chain.default_choice` 第 8 项 = 1、第 9 项 = 2。
- 起链那一刻问一句 `ask_chain_choice(`[`z0.py`](../../../run/z0.py)`).py#L1330)`：空回车 = 该菜单项默认档、
  打错的**再问一遍**、`EOFError`（管道/重定向）走默认；★ `ensure_sourced()` 排在**提问之前**
  （它会把本进程 `exec` 掉重开，先问就白问了）。
- 尾巴是**选出来的**那一档：`MANUAL_CHAIN + choice.tail`；第 4 档多带
  `--ros-args -p reference_source:=traj`；要不要把终端让给 `fly.py` 也按档（`choice.handoff`）。
- `mpc` 进 `ACTIONS`（`menu=False`，不占菜单号）⟹ 命令仍只写一处；`DOWN_TARGETS` 同步加 `mpc`。
- `mpc` 那一步的判据 = 它**自己的启动日志**那一行（`wait_kind` 归到 `"log"` 一类）。

**复核查出、已就地修掉的六条**（★ 复核意见也当假设验，逐条回代码核过才动）：

1. **术语回退（最大的一处）**：新写的文本里**几十处**裸用那个三字母缩写，而 `HEAD` 上
   `run/z0.py`、`run/README.md`、`operation_manual.md` 对这一条正则的命中数**都是 0**
   （现算：`git grep -cP '(?<![A-Za-z_/\-])MPC(?![A-Za-z_（\-\d])' HEAD -- run/z0.py`
   `run/README.md operation_manual.md README.md`）⟹ 是回退不是续用。
   （★ 上面那条命令里的三字母缩写是**正则本身**，属 `CONTEXT.md` 的豁免类 ①，别去改它。）
   全部改成 `MPC（模型预测控制）`（每份文件首处）/ 全称「模型预测控制」，
   并**加了一条守卫用例** `test_三个文件里不裸用_MPC`（正则逐字搬自 `CONTEXT.md`
   那条全仓复扫命令）。★ 例外：`README.md` 有两处是**讲这个词本身**（`CONTEXT.md` 的豁免类 ①）
   ⟹ 不扫进守卫。
2. **票面第 ③ 条当时没写在屏上**（复核查出，验收第 9 条只做到 2/3）：
   第 4 档会把终端让给 `fly.py`，而它的状态栏读 `/control/state`、`/control/traj_progress`
   （[`fly.py:50-57`](../../../src/bringup/scripts/fly.py#L50)）—— 这一档没有 `control` 在跑
   ⟹ 那两格永远停在「还没收到」。已补进第 4 档的收尾，并用例钉住；
   同时把 `_MPC_CLOSING` 的编号**对齐票面**（① 解锁/OFFBOARD、② `/traj/start`、③ 状态栏）。
3. `run/README.md` 的用例数写错：`test_z0_menu` 80 → **81**（现算 `--collect-only`）。
   ★ 同一段里 `test_rec` 的「66 例」是**改动前就错**的（实为 80），一并改；
   `test_z0_check_live` 那行没动（它的 5 例被顶层 `pytest.skip` 挡住，是另一件事）。
4. `CHAIN_DEFAULT_CHOICE` **没有生产读者**（只有用例读它）⟹ 删掉这个"第二份编号"，
   用例改成 `{c.key: c.default_choice for c in z0.CHAINS}` 现算。
5. 「这一步按哪种判据等」原先在 **三处各写一遍** `if/elif`（`wait_hint` / `start_and_wait` /
   `_render_steps`）—— 漏改一处的表现是**静默的**。提成 `wait_kind()` 一处，三处都问它；
   新增 `test_判据的分类只有一处_三种情形都分对` + `--dry-run` 的**跨层**断言（摊开的等待上限
   与真等的那一处同源）。
6. 注释里的行号 `mpc_node.py:724` → `:724-725`（`get_logger().info(` 与那句串分两行）。

**没采纳的**（连理由一起记，免得下次又提）：

- `ControllerChoice.key` 与 `Chain.key` 同名不同义：两个 key 各在自己的命名空间
  （档 = `mpc_hold` / `mpc_traj`，链 = `up` / `up_manual`），改名只有 churn、不修 bug ⟹ 留着。
- 未把 `wait_hint` 的**逐条话术**也塞进 `wait_kind`：那些话术每步独有（`mavros` 那条还要
  解释"有帧 ≠ 连上飞控"），塞进分类函数只会把判据分类又搅浑。

**跑了什么**（2026-10-16）：

- `run/` 这一侧：`test_z0_menu` **83** / `test_rec` **80** / `test_operation_manual` **5** /
  `test_safety_paths_drill` **10** —— 分开点名跑 = **178 例全绿**（整目录跑会收集到 0，见验收末条）。
- `src/nmpc/test`：**266 passed**（478 s，`ACADOS_SOURCE_DIR` 与 `LD_LIBRARY_PATH` 都设了）。
  ★ 本票没动 `src/` 里的任何代码（只动了 `src/bringup/README.md` 这份文档）⟹ 它不受本票影响，
  这次跑的是**基线**。
- 变异测试（★ 新断言都主动弄坏一次）：把 `_render_steps` 的分类换回 `key in STEP_READY`、
  把 `wait_kind` 的 `"log"` 分支改成 `"ready"`、把第 ③ 条文案换掉 ⟹ 分别红在
  `test_dry_run_把四档的命令都摊开且默认那档排在前面`、三条（含
  `test_mpc_那一步等的是它自己的日志行_等不到就报红并停掉其余的`）、
  `test_模型预测控制两档的收尾都要说清_还差哪两步`；恢复后 83 例全绿。

★ **Status 不动**（`ready-for-agent`）：归谁收口是操作者的裁定（本仓那条纪律）。
