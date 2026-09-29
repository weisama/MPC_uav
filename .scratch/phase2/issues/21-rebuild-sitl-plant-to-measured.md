# 21: 把 SITL 同构机按真机实测口径重搭（会作废 A/B 两档基线）

Status: ready-for-agent
Type: task

**What to build:** 让 `tools/px4-sitl/z0-plant/` 生成的**被控对象**在推力满量程上与**真机实测**一致 ——
现在它比真机小 **3.92 倍**。改完，节点**不需要**再靠 `--hover-thrust 0.6235` 覆盖就能起飞。

**Blocked by:** [12](12-mismatch-sweep.md)、[20](20-vertical-channel-mismatch-offset.md)
（§7 第 2 条：裁定 3 已定「本张不重搭、用现基线量 ⟹ 重搭**另开票**」—— **本张就是那张票**）。

⚠️ **这个顺序是承重的，不是流程依赖**（`ros-ws-8b` 提请写进票面，本张认同）：
操作者裁定 3 是「**先补垂直通道，用现基线复跑**」⟹ **20 的那两场必须在现基线上飞完**。
**本张先落就等于那两场与 F0–F4 对不上基线，整批数字一起废** —— 12 的 13 场扫描同理。
⟹ 本张开工前必须拿到 20 与 12 **都飞完并归档**的确认，不是"它们看起来做完了"。

**来源：** 操作者 2026-09-29 裁定「重搭，但排到 [11](11-sitl-tier-b-seam3.md) 收口后」；
11 号票已交回（`238236b`）⟹ 前置条件满足。裁定原话与两套口径的对账见
[10 号票](10-param-measure-backfill.md) 第五轮与其补记。

---

## 1. 差在哪（第一手）

| 口径 | 总推力满量程 | 解析悬停油门 | 出处 |
|---|---|---|---|
| **现同构机** | `4 × SIH_T_MAX` = **11.8 N** | **0.6235169** | [`plant_spec.py:43,69`](../../../tools/px4-sitl/z0-plant/plant_spec.py)（`T_MAX_TOTAL = 11.8`） |
| A 档机型推得 | 同上 | **0.6233** | [`tier-a-sitl-hover.md:214`](../../../docs/control/tier-a-sitl-hover.md)（`m·g/(4·T_max)`；**节点参数当时被显式覆盖成 0.6235**，两个数来源独立） |
| **真机实测（本张目标）** | **46.3027 N** | **0.1589** | [`params.py`](../../../src/nmpc/nmpc/params.py) `hover_thrust`（ticket 10 回填） |

⟹ 差 **3.92 倍**（`46.3027 / 11.8`）。节点按真机的 `0.1589` 归一化发指令、而同构机要 `0.6233`
才悬停 ⟹ **照现在这套跑 A/B 两档会掉**（实测症状：`mismatch-sweep.md` 那张表 —— 真值 `z` 恒为
`+0.0002 m`、末段电机均值 0.2625 ⟹ 总推力 3.098 N < `m·g` = 7.355 N，**根本没离地**）。

## 2. ★ 为什么是"把同构机调大"，而不是"把模型调小"

现在的兜底做法是给 DUT 传一个 `--hover-thrust`，把**模型**的归一化常数缩到同构机那一档
—— 值取**标称**：`_resolve_hover_thrust("auto")` → `plant_hover_thrust()`（无入参，
`m·g/(4·SIH_T_MAX)`，11 个变体同一个值）。2026-09-29 11:38 现场抓到的进程行为
`--hover-thrust 0.6233040254237288` 正是这个标称值。
⚠️ 两个做法都能让两边自洽，但**物理上不等价**，本张要的是前者：

| | 被控对象满量程 | 控制器的 `F` 上限 | 后果 |
|---|---|---|---|
| 现兜底（`--hover-thrust 0.6235`） | 11.8 N | `f_max` = 11.8 N（**两个角色又塌回同一个数**） | 电池上限被**烘进被控对象**里 ⟹ 解耦在 SITL 里**测不出来** |
| 本张（重搭同构机） | **46.3 N** | `f_max` = 11.8 N（**保持解耦**） | 被控对象 = 电机能力，电池限制留在**约束层** ⟹ 与真机同构 |

**判断依据：** 电机/桨**瞬时**能出 46.3 N（悬停日志实测满量程），电池**可持续**只供得起
11.8 N 那一档 —— 所以"能出多少"属被控对象、"准不准要多少"属约束。SIH **没有电池模型**，
把 11.8 N 写进 `SIH_T_MAX` 等于让同构机自带一个不存在的电流限制。
⚠️ 反过来说：现兜底在**电池口径**上是自洽的（11.8 N 就是 15 A 那一档），
所以它**不是错的**，只是它把两个角色压平了 —— 这正是 [10 号票](10-param-measure-backfill.md)
2026-09-29 解耦要修的那件事在 SITL 侧的同一个影子。

## 清单

- [ ] `plant_spec.py` 的 `T_MAX_TOTAL` 改成**实测满量程**（46.3027 N）⟹ `SIH_T_MAX ≈ 11.5757 N`；
      ⚠️ 该常数的注释现在写着"**按电池**的可用总推力上限（phase2-implementation.md §3.3）"——
      **那句话本身就是那个错的角色分配**，改值的同时把理由改对（被控对象按电机，不按电池）
- [ ] `z0-plant/params/` 下由 `plant_spec.py` 生成的那些 `.param` **全部重新生成**并逐份核 ——
      实测它们**每一份都带 `SIH_T_MAX`**（12 份全中）：`z0.param`、`z0-mass±30`、`z0-inertia±30`、
      `z0-rate±30`、`z0-tau±30`、`z0-thrcurve`、**`z0-thrust±30`**。
      ⚠️ 它们是 `plant_spec.py` 的**一一映射**（见 [11 号票](11-sitl-tier-b-seam3.md):203），
      只改 `z0.param` 会让失配档与基准档**不在同一套口径上**，扫描结论直接作废。
      ★ 特别留意 `z0-thrust±30`：现值为 **2.065 / 3.835 N**（= 2.95 ∓30 %）——
      **推力这一档的失配是烘在被控对象里的**，重搭时必须让它仍相对**新基准**保持 ∓30 %，
      否则 B 档"推力失配"那一列量到的是两个变化的叠加（机型换了 + 失配档没跟上）
- [ ] 复算并记下新的解析悬停油门（应是 **0.1589** 那一档），与 `params.py` `hover_thrust` 对账；
      **两者一致才算搭对**（这正是"同构"的定义）
- [ ] **摘掉那个兜底覆盖，并清掉它的记账**（全在 `tools/px4-sitl/`）：
      `verify_tier_b_seam3.py` 的 `--hover-thrust` 默认值、模块 docstring 里那段 ☠☠、
      以及几处 3.92× 的归因算例/回显；`mismatch_sweep.py` 里 `plant_hover_thrust()` 与
      `_resolve_hover_thrust()` 的口径说明、`mismatch-sweep.md` §1 的读数。
      重搭后 3.92× 归零，这些记账**每一处都会变成过时叙述**。
      ⚠️ 两个文件都是 [11](11-sitl-tier-b-seam3.md)／[12](12-mismatch-sweep.md) 号票的在制品，
      **动之前先与作者对**（本张不擅自改）。
      ⚠️ **按「文件 + 符号名」找，别按行号找**（写这张票时 `mismatch_sweep.py` 还**未入库**，
      作者正在改它）。⚠️⚠️ 而且**光按符号名也不够 —— 这个仓库里有两个同名不同物的
      `plant_hover_thrust`**（`ros-ws-8b` 指出，本张已核）：

      | 文件 | 签名 | 返回 | 行为 |
      |---|---|---|---|
      | `verify_tier_b_seam3.py`（[11](11-sitl-tier-b-seam3.md)／[20](20-vertical-channel-mismatch-offset.md) 在制品） | `(param_file: str)` | dict | **读传进来的那个 `.param`** |
      | `mismatch_sweep.py`（[12](12-mismatch-sweep.md) 在制品） | `()` | float | **写死读标称 `z0`** |

      ⟹ ☠️ **别"统一签名"**：谁要是照 `mismatch_sweep.py` 那个无参版把 `verify_tier_b_seam3.py`
      的也改成 `()`，B 档 `--mismatch mass+30` 会**静默退回标称口径**（它的调用点传的是
      `cfg.plant_param`），判据④ 的预测下降率跟着错 —— **不报错、只是悄悄换口径**
- [ ] ★ **`test_mismatch_sweep.py::test_标称悬停推力从机型真源推_而不是读节点默认值` 必须同批改** ——
      它是这件事**机器可读的那份记录**，且是**三条**断言（`ros-ws-d6` 复核订正，本张初稿漏了第一条）：
      ① `== 0.750·g/(4×2.95)`（`rel=1e-9`，把 **m = 0.750 与 `SIH_T_MAX = 2.95` 都硬写死**）；
      ② `== 0.6233`（`±5e-4`）；③ `params.HOVER_THRUST < 0.5 × 它`。
      重搭后 **①②③ 一起红**（只改 ②③ 会在同一测试里再撞一次）。
      ⚠️ 它自己的 docstring 已经点名了这个耦合（"请连 `plant_hover_thrust()`、本测试与
      `mismatch-sweep.md` §1 一起改"）—— **照它说的做，别把它删了或改宽**（那三条断言是
      "重搭还没做"的守卫，重搭做完它们就是过时的）
      ⚠️ 该文件**未入库**、是 [12](12-mismatch-sweep.md) 会话（`tickets 212`）的在制品 ⟹
      **别自己动**，先与它对齐（`ros-ws-8b` 也点名了这条）
- [ ] **`--hover-thrust auto` 重搭后自动落到标称 —— 但要防它被改成"按本场推"**：
      `_resolve_hover_thrust("auto")` 调的是 `plant_hover_thrust()`，后者**只取标称**
      （`expected_params("z0")`，无 per-scenario 入参）⟹ 现在 11 个变体拿到的是**同一个**
      `0.6233`，重搭后同一个 `0.1589` ⟹ 覆盖可以整条摘掉。
      ⚠️ **别有人"为了保险"把它改成按本场 `.param` 现推** —— 那会让 `z0-mass+30` 那场
      拿到 `≈0.8106`、把**失配本身抵消掉**（节点被喂进"这架飞机要 0.81 才悬停"，
      M5 兜底路径悬得稳稳的，"mass 失配"就量不出来了）。
      ⟹ 票面把这条写下来当**反向守卫**：`auto` 必须始终等于**标称**。
      （本张初稿曾推断"auto 已经在抵消 mass 失配"，**是自己读码时把两个同名函数看串了**
      而推出的错论 —— 核实与订正过程见 Comments）
- [ ] `test_tier_b_judges.py` 里硬写 `model_ht=0.6235` 的合成夹具：
      逐条判"这是**夹具自洽性**测试（不碰真机型）还是**口径记录**" —— 前者留、后者跟着改，
      别一刀切
- [ ] **跑一次冒烟**证明"不传覆盖也能悬停"：至少 A 档那条最简闭环（`verify_tier_a_hover.py`）
      + B 档一场，读数与 `m·g` 对得上
- [ ] ★ **归档要记 harness 自身的身份，不能只记 DUT 与 overlay**
      （`ros-ws-d6` 撞见的口子，`tickets 212` 已核实并**补好了**）：原 `obs["sweep"]` 只记被测对象
      （`controller_code`），不记扫描器自己 —— 而 `mismatch_sweep.py` 未入库、mtime 实测
      `11:32:31` 正落在两批之间。现在已加 `obs["sweep"]["harness"] = {file, sha256, mtime}`，
      `summarise()` 在两批签名不同时报"**不是同一个实验**"、**没留档时明说"查不出"**。
      ⟹ 本张**只需沿用**（重跑时确认这个字段在）；⚠️ 别把 `overlay_sha256` 逐档不同读成漂移
      —— 那正是被扫描的机型，设计使然
- [ ] ★ **重跑要留痕：别用"事后补写"去对齐两批的字段**（`tickets 212` 的第一手教训，值得本张照抄）：
      它本想拿"前 3 场没有 `trace` 字段、第 4 场起才有"当旁证，结果事后跑 `--attach-trace`
      给前 3 场补齐，**把这条旁证自己抹平了**（现在 13 场的字段集合完全一致）⟹
      ① **事后补写会销毁"前后有差别"这条证据**；② **不留痕的改动，事后只能靠口供**。
      本张重跑扫描/重搭时，凡"事后补进去的"都要带上**谁补的、哪一次**（照它 `trace.attached_by`
      的做法），并在引用时按"**可能换过尺子**"处理
- [ ] ★ **"exec 的那份" vs "import 的那份" —— 方向要说准**（`tickets 212` 提请，本会话已核并**订正其表述**）：
      `install/` 下 `mpc_node.py` 是**两个文件**：
      `install/nmpc/lib/nmpc/mpc_node.py`（**`ros2 run` exec 它 ⟹ 飞的是这份**）与
      `install/nmpc/local/lib/python3.10/dist-packages/nmpc/mpc_node.py`
      （**`import nmpc.mpc_node` 拿的是这份 ⟹ 单测 import 的是这份**）。
      实测两份**字节相同**（`sha256 = 1c7fbd2947bdb6ce…`，也正是 12 号票归档里的
      `entry.sha256`——两边对得上 ✓），且 `lib/` 那份是**自足脚本**（自己 `import rclpy`、
      自带入口）⟹ **不是"飞的走后者"**，而是：**飞 = `lib/`，import = `dist-packages/`**。
      ⚠️ 真正的风险在这里：**同一个 `test_mpc_node.py` 里两种都用到**（它 `import nmpc.mpc_node`
      取 `QOS_PUB_FMU`、又 `ros2 run` 起 DUT）⟹ 只有 build 让两份同步时两边才可比；
      **半途而废的 build（只刷新一份）会让"单测绿"与"飞的那份"脱钩**。
      build 后两份**同时翻**，引 12 那张身份签名（`entry.sha256` + 11 个 `.py`）时**先核签名**
- [ ] ★★ **重跑前先确认 pytest 不是"静默跳过"**（`tickets 212` 的第一手，本会话**已复现**）：
      `tools/px4-sitl/` 的测试开头有 `pytest.importorskip("rclpy"/"px4_msgs"/"nmpc")`，
      而 **`px4_msgs` 不在本工作区**（在 `/home/chen/px4_msgs_ws/install`）⟹
      只 source 本仓 `install/setup.bash` 时**不报错、直接跳过**：
      实测 **`1 skipped in 0.09s`**（看着像"测试不存在"）；补上 source 后
      **`162 tests collected`** ⟹ 别把这个"绿"当成全绿。
      正确姿势（本张重跑扫描/重搭时照用）：
      `source /opt/ros/humble/setup.bash` + `source /home/chen/px4_msgs_ws/install/setup.bash`
      + `source install/setup.bash` ⟹ `python3 -m pytest tools/px4-sitl -q`
      ⚠️ 但 `src/nmpc/test/` **没有** `importorskip`（实测 grep 无命中）⟹ 那边缺 `px4_msgs`
      会**硬报错**而非静默跳过，两棵树的失败模式不同
- [ ] **按下面的《失效清单》逐份记「基线已换」**（见末节）：每份写明"这组数字出自旧机型"
      并给出旧机型常数（`SIH_T_MAX = 2.95 N`），**不许只改其中一份**
- [ ] ⚠️ **不改 `src/nmpc/nmpc/params.py`** —— 「11 补、10 不动」的裁定**仍然在效**
      （见 [20 号票](20-vertical-channel-mismatch-offset.md) §7）。本张只动被控对象
- [ ] 若重搭后 SITL 里出现**新**的不达标，**如实记未达标 + 实际数值**，不要为了让它过而回退机型

---

## 失效清单（重搭落地时**逐份**记「基线已换」）

| 载体 | 会失效什么 | 为什么 |
|---|---|---|
| [`tier-a-sitl-hover.md`](../../../docs/control/tier-a-sitl-hover.md)（[09](09-sitl-tier-a-closed-loop.md) 的交付） | 0.994 m 抬升、3.995 ms 最坏求解、`:214` 那条推力标定（0.6233 / 0.6235 / 0.6080） | 全在 `4×2.95` 那台机型上量的 |
| [`tier-b-sitl-seam3.md`](../../../docs/control/tier-b-sitl-seam3.md)（[11](11-sitl-tier-b-seam3.md) 的交付） | 五场读数、四条判据的通过/未达标结论 | 同上；且它记的 3.92× 记账会**归零** |
| [`mismatch-sweep.md`](../../../docs/control/mismatch-sweep.md)（[12](12-mismatch-sweep.md) 的交付） | **整张扫描表**（11 变体 × N 轮）| 扫描的被控对象换了 ⟹ 整表不可复现 |
| [20](20-vertical-channel-mismatch-offset.md) 的验收数字 | 它的两场复跑读数 | 见抬头的"顺序是承重的" |
| `test_mismatch_sweep.py` 的三条断言 | 见清单里那条（是**机器可读**的失效记录） | 它断言的就是"两者不同口径"这件事本身 |

⚠️ 抬头的 `Blocked by` 正是为了这份清单里 [12](12-mismatch-sweep.md)／[20](20-vertical-channel-mismatch-offset.md)
两行 —— **先让它们在旧基线上量完归档，再来换基线**。

---

## Comments

### 2026-09-29 —— 本张的由来与编号

由 [10 号票](10-param-measure-backfill.md) 第五轮的裁定 ③ 移交而来（原话：「重搭，**但排到 11 收口后**」）。
[20 号票](20-vertical-channel-mismatch-offset.md) §7 第 2 条把这件事从它自己范围里排出去并写了"另开票"，
但那张票此前**并不存在** —— 由 [10 号票](10-param-measure-backfill.md) 的会话开出来以补齐
"未决事项必须另开票并点名"那条纪律（操作者 2026-09-29 裁定"我开，但要防撞号"）。

⚠️ **编号取 21 是因为它是 `issues/` 里下一个空闲号**（写入时最新为 `20-vertical-channel-mismatch-offset.md`）。
若并行会话已另开一张同题材的票，**以先入库者为准、后到者并入**，别留两张。
（占号已通报 `ros-ws-d6` / `ros-ws-8b` / `tickets 212`；前两位回信：8b 说"21 不是我开的、也不打算开"
⟹ 无并票问题；d6 是旁观会话，不取号。）

### 2026-09-29 —— 两条**复核订正**（peer 当场核实，已落进上面正文）

1. ★★ **本张初稿那条"`auto` 已经在抵消 mass 失配"的推断，不成立 —— 根因是我把两个同名函数看串了。**
   初稿据 `plant_hover_thrust(param_file)`（**带参签名**）推断 `auto` 按本场 `.param` 现推、
   于是把 `z0-mass+30` 需要的 `≈0.8106` 喂给节点、顺带修掉了 mass 失配。
   **核实（本会话读码 + `ros-ws-8b` 独立复核）**：`mismatch_sweep.py` 里那个
   `_resolve_hover_thrust("auto")` → `plant_hover_thrust()`，变体名 `"z0"` 是**写死**的
   ⟹ `auto` 对 11 个变体一视同仁给同一个 `0.6233`，**不抵消**失配。推断**撤回**。
   ⚠️ **订正时我又错了一次，一并记下**：我先前写"同一会话内 `mismatch_sweep.py` 的行号
   从 `:390` 漂到 `:99`、签名从 `(param_file)` 变成 `()`"—— **那是两个不同的文件**
   （`:390` 那个带参版在 `verify_tier_b_seam3.py`，今天没动过），**不是同一个函数被重写**。
   教训因此升级为：**按「文件 + 符号名」找**（`ros-ws-8b` 的原话），
   因为同名会在两个文件里撞、且**行为不同**（一个读到传入的 `.param`、一个只读标称）。
2. **那条守卫测试是三条断言，不是两条**（`ros-ws-d6` 订正）：初稿漏了第一条
   `== 0.750·g/(4×2.95)`（`rel=1e-9`，把 `m = 0.750` 与 `SIH_T_MAX = 2.95` **都硬写死**）。
   已补进正文；只改后两条会在同一测试里再撞一次红。
3. **`ros-ws-d6` 提请的第三条（已落进清单）**：本轮扫描的 `mismatch_sweep.py` **mtime = 11:32:31**，
   正落在「基准 3 场跑完」与「失配档开跑」之间；该文件未入库，而 obs **只记 DUT 与 overlay 的身份、
   不记 harness 自身** ⟹ "评判者换了版本、归档里看不出来"这条路是通的（本轮实测没出事：
   10 份 obs 的 `controller_code` 与 `entry.sha256` 全同 `1c7fbd2947bdb6ce`）。
   本张要重跑扫描，故把"堵这条路"列进清单。
