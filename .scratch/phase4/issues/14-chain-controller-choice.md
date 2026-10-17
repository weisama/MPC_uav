# 14: z0.py 第 9 项起链前选「谁在上位控制飞机」—— offboard 控制 / 模型预测控制（第 8 项复原）

Status: wontfix
Type: task

> ★★ **2026-10-17 收口（操作者裁定）：本票被 [票 15](15-trajectory-flight-roles.md) 取代，切 `wontfix`。**
>
> **这是「被取代后收口」，不是「做完了」** —— 操作者在「做完了 / 被 15 取代后收口」两选一里
> 选了后者（原话：「**被 15 取代后收口**」）。
>
> - **被取代的是形态**：票 15 把菜单 9 的模型预测控制档改成 `traj → mpc → control`，`control`
>   成为唯一任务总管 —— `reference_source:=control`、`/traj/start` 归 `control`、**不再**用
>   `start_traj_reference`（HEAD 第一手：`run/z0.py` 的 `UP_CHOICES` 第 2 档尾巴是
>   `("traj","mpc","control")`）。本票「二定」里「第 2 档尾巴 `traj → mpc` + 三条参数」
>   「`mpc_node` 自己发 `/traj/start`」「开始键在这条链上不起作用」**作废**，以票 15 为准。
> - **不被取代的是零件**：菜单 8 复原、轨迹 4「起飞」、`offboard_prestream`、`ACADOS_ENV`
>   都被票 15 **继续使用**，不随本票收口。★ 其中轨迹 4 的 `src/traj` 实现（`kTakeoff` /
>   `takeoff_height_m` / 斜坡 + 用例）**至今只在工作树里、HEAD 没有**
>   （现算：`git show HEAD:src/traj/include/traj/trajectories.hpp | grep Takeoff` 为空）
>   ⟹ 它是**票 15 的依赖**，归票 15 处理，**不许**因本票 `wontfix` 被当成作废项。
> - **为什么不能标 `done`**：验收 11 条在**落地当时**为真（一版 `85bde0d` 已提交；二定代码里
>   `z0.py` / `mpc_node.py` / `fly.py` 随 `1dc5fdb` 整文件纳入），但对着 HEAD 验收第 2 条已不成立
>   （`reference_source:=traj` / `start_traj_reference` 被票 15 改掉）；且票面 markdown 的二定改写
>   与本票 2026-10-17 的 ACADOS 证据**仍未提交**。
> - **实飞验收不在本票**：本票本来就没列实飞项；「MPC（模型预测控制）两档真的用开始键飞
>   轨迹 1/2/3/4」是票 15 的 `[ ] 实飞验收（归操作者）`。

**What to build（二定，2026-10-16 —— 本票的现行版本）：**
`run/z0.py` 的菜单 **9（`up`）** 在**起链前问一句**，按数字选「这一趟谁在上位控制飞机」——
**只有两档，两档都必起 `traj`**：

| # | 选什么 | 起链的尾巴（公共前缀 `mavros → vrpn → mocap` 之外） | 这一档额外带的参数 |
|---|---|---|---|
| 1 | **offboard 控制**（`control`，Phase 1 的 (A) 级）—— **默认档** | `traj → control` | — |
| 2 | **MPC（模型预测控制）**（`nmpc` 的 `mpc_node.py`，参考源**只能 `traj`**） | `traj → mpc` | `reference_source:=traj` + `offboard_prestream:=true` + `start_traj_reference:=true` |

★ ☠️ **2026-10-17 更新（随票 15）**：上表**第 2 行**的尾巴与参数已被取代 —— HEAD 上是
`traj → mpc → control`、`reference_source:=control`、`/traj/start` 归 `control`。本票已切
`wontfix`，判据以文首收口块与票 15 为准。

★ 菜单 **8（`up_manual`）复原**：回到一版之前的样子 —— 直接起 `mavros → vrpn → mocap`、**不问**
（它就是"不起控制器、手动遥控飞"那一档，如今不再藏在菜单 9 的编号里）。

**为什么（二定，操作者 2026-10-16 原话）**：「将轨迹 1 起飞悬停，改为可以设置起飞高度。然后 z0.py 的
**8 复原**，**9** 变为可以选择 offboard 和 mpc（**不能 hold**，只能跟踪轨迹），然后**轨迹必须起没得选**，
我可以通过选择轨迹 1 再输入起飞高度，从而达到 mpc 控制起飞 20cm 悬停」；
当天稍后又改口：「**算了，不要修改轨迹 1 的东西，改为添加一条轨迹 0：起飞到高度 20cm**」。
⇒ 归结成三件事：
1. 菜单 9 只剩两档、`traj` **必起**、模型预测控制**没有 hold 那一档**（一版里的第 3 档删掉）；
2. **不改轨迹 1**：起飞改由 **`traj` 新增的第 4 条轨迹「起飞」**承担（编号取 4 不是 0 ——
   `/traj/select` 的 **0 是「清空」**，那是既有语义，`test_清空_参考停发_marker_删除_完成度报零` 钉着它）；
3. 那条链要真的跑得通 ⟹ `mpc_node` 添两条**默认关**的新参数（`offboard_prestream` /
   `start_traj_reference`，见「设计（二定）」）。

**为什么（一版，2026-10-16 早些时候）**：「给 z0.py 脚本的 8 和 9 选择添加新的选项，可以选择启动
offboard 控制还是 mpc 控制，以至于我可以手动起链和录制以完成票 [10](10-ladder-closed-loop-and-free-flight.md)」。
票 10 的阶梯 3/4 要的是 **MPC（模型预测控制）自己飞**，而今天两条一键里唯一的控制器格是 `control`
（(A) 级）—— 想跑模型预测控制只能手打一条 `ros2 run nmpc mpc_node.py`，起链与录制两头都接不上。

**Blocked by:** 无。

## 范围

**这一版动四处**（☠️ 一版那句"只动 `run/z0.py` 的「选哪条链」这一层……不动 `mpc_node.py`"**作废** ——
两档要真跑得通就得动 `src/`）：

1. `run/z0.py`：菜单 8 复原（不再问）、菜单 9 两档都带 `traj`、`mpc` 那条动作多两条**可选参数**；
2. `src/traj/`：**新增第 4 条轨迹「起飞」** + 两个 yaml 参数（`takeoff_height_m` / `takeoff_speed_ms`）；
   ★ **轨迹 1/2/3 的行为一个字不改**（操作者当天改口正是指的这一条）；
3. `src/nmpc/nmpc/mpc_node.py`：两条新参数 `offboard_prestream` / `start_traj_reference`，
   **默认都关** ⟹ 不传参数时行为与今天逐字一致（既有用例一条都不该红）；
4. `src/bringup/scripts/fly.py`：把第 4 条轨迹加进 `KINDS` 与命令字面（它只是"多发一个编号"）。

**仍然不新增任何会动飞机的入口**：`z0.py` 里照旧不许出现 `ros2 service call` / `topic pub` 那类字样
（`FORBIDDEN_IN_COMMANDS`）；两条新参数都长在 `mpc_node.py` 自己身上，**不解锁、不切模式、不发服务**。
**不动** `control_node.cpp`。

## ★★ 这一轮**不解决**的三件事（都**不静默**，写在屏上 + 本票里）

### ☠️ 逐项点名（二定，2026-10-16）—— 下面是**一版**的正文，其中两项已随二定改写

- **①（没人解锁、没人切 OFFBOARD）＝ 半作废。** 新增的 `offboard_prestream:=true` 拿掉了这一条里
  最硬的那半句："闸门关着时它连心跳都没有 ⟹ 飞控**拒绝**进 OFFBOARD"。一手依据见「设计（二定）」的
  那两处 `mavlink_receiver.cpp`。**但**"解锁 + 切 OFFBOARD 仍由操作者手打"照旧成立
  （`mpc_node` 自己不 arm、不 set_mode）—— 只是不再有"流"这一层坑。
- **②（没人发 `/traj/start`）＝ 作废。** 本版新增 `start_traj_reference:=true`（默认关，`z0.py` 的
  模型预测控制档打开）⟹ 由 `mpc_node` 在"armed + OFFBOARD 且还没收到过参考"那一拍发一次。
- **③（`fly.py` 的状态栏是空的）＝ 仍然成立。** 这一档没有 `control` 在跑，`/control/state` 与
  `/control/traj_progress` 不会有数（**选轨迹**那一半照常可用）。

下面这一版里凡是与上面三条冲突的说法，**以本节为准**。

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

## 设计（二定，2026-10-16）—— 本票的现行设计

### 菜单 8 复原、菜单 9 只剩两档

> ☠️ **部分作废（2026-10-17，随票 15）**：菜单 8 复原、菜单 9 两档、两档都带 `traj` **仍然有效**；
> **第 2 档的尾巴与参数作废** —— HEAD 上是 `traj → mpc → control` + `reference_source:=control`
> + `offboard_prestream`，`output_mode:=external` 加在 **`control`** 那条上，**不再**是本文写的
> `traj → mpc` + `reference_source:=traj` + `start_traj_reference`。见文首收口块。

```
起链后**谁在上位控制飞机**？（输数字；直接回车 = 1 offboard 控制）
  1 = offboard 控制（control，(A) 级）—— 起 traj；起完选轨迹 + 按开始键
  2 = MPC（模型预测控制）—— mpc_node.py，参考源 traj；**起 traj**，起完选轨迹
```

★ 首处提到本项目的那个控制器写 `MPC（模型预测控制）`（`CONTEXT.md` 第一节），其后写全称
「模型预测控制」，**不写裸词** —— `run/z0.py` 里那条正则守卫（`test_三个文件里不裸用_MPC`）扫的就是它。

- 两档的尾巴**都含 `traj`** ⟹「轨迹必须起」是结构上的事实，不是提示语；模型预测控制档
  **没有 `hold`**（一版第 3 档那条以默认值为 `hold` 的路删掉）。
- 菜单 **8** 不再问：它的 `Chain.choices` 只剩一档（`none`）⟹ `ask_chain_choice` 见**只有一档就不问**、
  直接返回它（屏上照旧打印这一档起什么）。这样"第 8 项起的东西"与一版之前**逐字一致**。
- `--dry-run`：菜单 8 不再有「其他档」那一块；菜单 9 仍是两块（默认档整块 + 另一档）。

### `traj` 新增第 4 条轨迹：起飞到 `takeoff_height_m`

| 编号 | 轨迹 | 参考（相对**轨迹原点**） |
|---|---|---|
| 4 | **起飞** | `z(t) = h · clamp01(t / (h/v))`；`h = takeoff_height_m`（默认 **0.2 m**）、`v = takeoff_speed_ms`（默认 **0.2 m/s**）⟹ 0.2 m 用 1.0 s；到顶后**停在那儿**到时长走完 |

- `z(0) = 0`：**起飞的零点就是轨迹原点**（对模型预测控制档 = 闸门第一次打开那一拍的**锚点**，
  飞机此刻停在**地面**）⟹ 目标 = 地面 + 0.2 m，这就是"起飞 20 cm"。
- 到顶之后就是悬停 ⟹ **时长沿用 `hover_duration_s`**（不另开一条参数），
  `progress = clamp01(t / duration)` 与悬停同形。
- **`height_offset_m` 不参与** `kTakeoff`（其余三条照旧参与）：起飞是从轨迹原点往上爬，
  再加偏移没有意义。★ 这条差别写进 `trajectories.hpp` 的文件头与 `traj.yaml` 的注释。
- 编号取 **4**、不是 0（操作者原话写的是"轨迹 0"，但 `/traj/select` 的 **0 = 清空**是既有语义：
  `traj_node.cpp` 的 `on_select`、`fly.py` 的「0 = 取消选择」、用例
  `test_清空_参考停发_marker_删除_完成度报零`）⟹ 0 一个字节都不动，新轨迹用下一个空号。
- **斜坡**（操作者在"斜坡 / 一步到位"两条里选的）：升速有界，与 §5.2 那条限速同取向；
  一步到位等于把爬升速率交给闭环自己定。

### `mpc_node` 两条新参数（**默认都是关的** ⟹ 不传参时行为与今天逐字一致）

> ☠️ **部分作废（2026-10-17，随票 15）**：`offboard_prestream` 仍被票 15 的模型预测控制档使用；
> `start_traj_reference` **代码仍在但菜单 9 不再用它**（`/traj/start` 归 `control`；
> `reference_source=control` 时该入口当场拒收）⟹ 本文「两条都由 `z0.py` 打开」那一层作废。

**`offboard_prestream`（bool，默认 false）** —— 闸门关着时，若理由是 `not_armed` / `not_offboard`
之一，就发**一帧纯悬停**（`ω = 0`、`thrust = t_hover / t_max`，就是 `__init__` 里那个 `_hover_sp`）。
白名单之外的理由（`status_stale` / `mocap_*` / `stamp_regress` / `pose_jump` / `no_traj_reference` …）
**照旧一帧不发** —— 那几条都意味着"本来该我们飞而出了事"，**停发正是要让飞控自己掉出 OFFBOARD**。

★★ 一手依据（PX4 1.14.3，`~/refs/px4-v1.14.3-build/`，逐条核过行号）：

- `mavlink_receiver.cpp:1562` `// Publish attitude setpoint only once in OFFBOARD`
  （速率那一路 `:1592` 同款）⟹ **不在 `OFFBOARD` 时，我们发的帧连控制器都到不了**：
  前置流能办的**只有**"让飞控肯放我们进 OFFBOARD"，**碰不动飞机**；
- `:1530-1536`（`:1535` 是那一句 `publish`）：`offboard_control_mode` **与模式无关**，
  只要这一帧带 `attitude` 或 `body_rates` 就发 ⟹ 这正是"前置流打开进 OFFBOARD 那条路"的机制；
- `:1523`：我们那张掩码 `128`（`IGNORE_ATTITUDE`）下 `body_rates = true` ⟹ 这一帧**确实**算"有信号"。

**`start_traj_reference`（bool，默认 false）** —— 参考源是 `traj` 时，闸门因 `no_traj_reference`
关着（★ 这条原因**蕴含** armed + `OFFBOARD` + status 新鲜：那三条判据在它**前面**，
见 `node_core.py` 的 `cycle()`）⟹ 发**一帧** `/traj/start`（`std_msgs/Empty`），发一次；
理由回到 `not_armed` / `not_offboard` / `no_status`（= 离开了 OFFBOARD）才重新武装。

### 这条链跑通的完整样子（= 目标"mpc 控制起飞 20cm 悬停"）

> ☠️ **整节作废（2026-10-17，随票 15）**：这条「这一档没有 `control`、遥控器开始键不起作用、
> 手打解锁 + 切 OFFBOARD」的流程已被票 15 取代 —— 现在 `control` 是唯一任务总管，两档现场动作
> 完全相同（选轨迹 + 开始键两次）。下面只作历史。

1. 菜单 9 → 输 `2`（模型预测控制）⟹ 起 `mavros/vrpn/mocap/traj/mpc`，终端让给 `fly.py`；
2. `fly.py` 里输 `4`（第 4 条轨迹：起飞）；
3. **手打**：解锁 + 切 `OFFBOARD`（这一档没有 `control`，**遥控器开始键在这条链上不起作用**）。
   `offboard_prestream` 让"必须已有连续流"这条不再是坑；
4. 闸门第一次满足（armed + `OFFBOARD` + status 新鲜）那一拍：`mpc_node` 发 `/traj/start`
   ⟹ `traj` 开始按斜坡发参考 ⟹ 锚点钉在**地面** ⟹ 目标 = 地面 + 0.2 m ⟹ 飞机爬到 20 cm 停住；
5. 录制那一头照旧（`run/rec.py`，场景 3/4/5/6 与它无关）。

---

## 设计（一版：四档）—— ☠️ 已被上面「设计（二定）」取代

> ☠️ **这一节整体作废，只作历史**：四档（含"不起控制器"档与"模型预测控制 + hold"档）已按操作者
> 2026-10-16 的二次裁定改成两档。**照本文施工会做出与本票现行版本不一致的东西**。
> 作废的理由与逐项点名见上面「不解决的三件事」节首，以及 `## Comments` 最后那一节。

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

## 验收（二定，2026-10-16）—— 本票的现行验收

★ 每一行**落地时**才打勾，并把证据（用例名 / 现算命令 / 输出行）写进方括号。**没做就留空。**

- [x] 菜单 **8 复原**：`up_manual` 不再问一句，起的东西与一版之前逐字一致
      〔`test_第八项不再问那一句_且与一版之前起的一样` 绿；**现算**：把 `85bde0d^:run/z0.py` 取到
      `/tmp` 跑同一条 `--dry-run up_manual`，归一化掉 `source …/ros_ws/install/setup.bash`
      这个"在树内才出现"的前缀后 `diff` = **只有两行新增**（`★ 下面摊开的是**默认那一档（第 1 档）**`
      / `★ 本链只有一档（第 1 档），起链时**不问**`），3 条命令与 3 条判据行**逐字相同**〕
- [x] 菜单 **9 只剩两档**，**两档的尾巴都含 `traj`**，模型预测控制档没有 `hold`、且带上
      `reference_source:=traj` / `offboard_prestream:=true` / `start_traj_reference:=true`
      〔`test_两档的尾巴都起_traj_且模型预测控制档带三条参数`、`test_模型预测控制档没有_hold_那一条` 绿；
      **现算** `python3 run/z0.py --dry-run up`：默认档 `4. 起 traj（单独）` → `5. 起 control（单独）`；
      「其他档」块只有 `[2] MPC（模型预测控制）`，第 5 步的命令里三条 `--ros-args -p` 齐全〕
      ☠️ **2026-10-17 随票 15 部分作废**：「两档、尾巴都带 `traj`、没有 hold」仍成立；
      **三条参数那一半作废** —— HEAD 是 `reference_source:=control` + `offboard_prestream`，
      `start_traj_reference` 不再带（`/traj/start` 归 `control`）。见文首收口块。
- [x] `traj` 第 4 条轨迹的**斜坡语义**：`z(0) = 0`、`z(h/v) = h`、斜坡内 `ż = v`、到顶后 `ż = 0`、
      `progress` 按时长走；★ **轨迹 1/2/3 的输出一个字不变**
      〔`./build/traj/test_trajectories` = **26 passed**（`Takeoff` 6 例全绿）+
      `test_traj_node.py::test_起飞_在话题上是一条斜坡_到顶后停住` 绿（12 例整文件绿）；
      `Hover`/`Square`/`Lemniscate` 的用例**一行未改**、照旧绿〕
- [x] `/traj/select 0` 仍是**清空**（第 4 条用新号，没抢 0 的语义）
      〔`test_traj_node.py::test_清空_参考停发_marker_删除_完成度报零` 绿（第 4 条的用例是新增，
      `0` 那条一个字没动）〕
- [x] `mpc_node` **两条参数默认都关** ⟹ 行为与今天逐字一致：闸门关着时**零帧 setpoint**
      〔**现算** `python3 -m pytest src/nmpc/test` = **269 passed**（526 s；
      `ACADOS_SOURCE_DIR=/home/chen/acados` 与 `LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib` **两个都设了**）；
      `test_未解锁时一帧_setpoint_都没有` 等既有用例照旧绿 —— 一版时的基线是 266，+3 = 本轮新增的三条〕
- [x] `offboard_prestream:=true` 时：**只在** `not_armed` / `not_offboard` 下发**纯悬停帧**；
      别的理由（`mocap_stale` 之类）**照旧零帧**
      〔`test_预流_只在那两档发悬停帧_别的关闸原因照旧一帧不发`（**一条用例盖住白名单两侧**：
      `not_armed`/`not_offboard` 发悬停帧、`status_stale` 零帧）绿〕
- [x] `start_traj_reference:=true` 时：闸门因 `no_traj_reference` 关着那一拍发**一帧** `/traj/start`，
      之后参考到齐、闸门打开、开始发 setpoint；**关掉时这条链仍然冻住**（负控）
      〔`test_自发起步_开了之后由节点自己按开始键_没人按就一直卡着`（含负控：默认关时零帧、
      参考不来 ⟹ 一直卡在 `no_traj_reference`）、`test_自发起步只在轨迹档有意义_别的档入口当场拒` 绿〕
- [x] 文档同步：`run/README.md` / `operation_manual.md` / `run/rec.py` 的菜单 8/9 引用、
      `traj` 的四条轨迹、`mpc_node` 的两条新参数（`src/traj/README.md`、`src/nmpc/README.md`）
      〔三个守卫文件**分开点名**跑全绿：`test_z0_menu` 83 / `test_rec` 80 / `test_operation_manual` 5 /
      `test_safety_paths_drill` 10 = **178 例**；`src/traj/README.md` 改四条轨迹 + 参数表两行 +
      测试数重算为 38；`src/nmpc/README.md` 新增「两条新参数」一节；
      另：`docs/control/phase1-implementation.md` §5.2 加第 4 行并标出 31/20/11 是快照〕
- [x] **变异测试**：本轮每条新断言都主动弄坏一次（改斜坡符号 / 把白名单放宽到全部理由 /
      把"只发一次"去掉 / 把两档尾巴里的 `traj` 删掉）⟹ 各自**当场红**
      〔四条见 Comments「跑了什么」的新一段；★ **两条不是红在断言上**，已在那一节逐条写明〕
- [x] **这条链第 5 步真的起得来**（2026-10-17 加，第一次实跑才发现）：`mpc` 那条命令自带
      acados 的两个环境变量 ⟹ 操作者不用先 export
      〔`test_mpc_那条命令自带_acados_的两个环境变量`；**实跑**：`ACADOS_SOURCE_DIR` 未设的 shell 里
      执行 `--dry-run` 摊出来的那条命令 ⟹ **10 s 起来**（日志 `MPC（模型预测控制）节点启动：` +
      `★预流=开 / 自发起步=开`）；★ 反证就是当天真实那次起链（死在 `OSError: libacados.so`）〕

---

## 验收（一版：四档）—— ☠️ 已落地，其中若干条随二定改写

> ☠️ **这一节的判据属于四档那一版**：条目本身的结论（哪些用例、哪些现算命令）仍然是真的，
> 但"四个档""第 8 项也问""模型预测控制 + hold 档"这些**形态**已被二定取代 ⟹
> 对着它施工会做出过期的东西。**现行验收在上面那一节。**

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

### 2026-10-16（当日二定）—— 四档 → 两档、第 8 项复原、新增轨迹 4、`mpc_node` 两条新参数

**本轮对话的工作并入本票**（操作者原话：「**将对话工作并入票 14**」）—— **不新开 15 号票**：
他在我上一轮给的"新开 15 号票 / 并进 14"里先答了前者，随后改成并入本票。★ 因此
**一版那半张票的历史（含四档的落地记录）留在下文，不另立新票。**

**操作者历次裁定的汇总（现行 / 已作废）**：

| 问 | 答 | 现况 |
|---|---|---|
| 这一轮改动怎么落票？ | 「**将对话工作并入票 14**」 | **现行** |
| 进 OFFBOARD 那条流归谁？ | **给 `mpc_node` 加前置流**（`offboard_prestream`，默认关） | **现行** |
| 新轨迹的编号？ | **用 4** —— `/traj/select` 的 0 是「清空」，那个号不动 | **现行** |
| 起飞高度固定还是可设？ | **写死 0.2 m**（`traj.yaml` 的 `takeoff_height_m`，起链时**不问**） | **现行** |
| 参考里的高度怎么给？ | **斜坡爬升**（`takeoff_speed_ms`，默认 0.2 m/s ⟹ 20 cm 用 1 s） | **现行** |
| 模型预测控制档的 `/traj/start` 谁发？ | **`mpc_node` 自己发**（`start_traj_reference`，默认关，那一档打开） | **现行** |
| 起飞高度在哪一步输入？ | ~~fly.py 选轨迹 1 时问~~ | ☠️ **作废** |
| 那个高度的作用范围？ | ~~只对轨迹 1 / 只在模型预测控制档~~ | ☠️ **作废** |

**两处改口的原话**（别拿更早那版去施工）：
1. 「将轨迹 1 起飞悬停，改为可以设置起飞高度。然后 z0.py 的 8 复原，9 变为可以选择 offboard 和 mpc
   （不能 hold，只能跟踪轨迹），然后轨迹必须起没得选，我可以通过选择轨迹 1 再输入起飞高度，
   从而达到 mpc 控制起飞 20cm 悬停」；
2. 随即改口：「**算了，不要修改轨迹 1 的东西，改为添加一条轨迹 0：起飞到高度 20cm**」
   ⟹ 高度通道从"改轨迹 1"变成"**新增一条轨迹**"；上面表里"问高度"与"作用范围"两问**同作废**。
   "轨迹 0"这个号与 `/traj/select` 的 `0 = 清空` 撞车 ⟹ 当场问过，他选了 **4**。

**为什么 `/traj/start` 这次必须解决**：一版把它记成"交接项"搁下了（见上面「不解决的三件事」②）。
但要真跑通"起飞 20 cm"，`traj` 必须发参考、而这一档**没有 `control`**（`/traj/start` 的唯一发布者）
⟹ 不解决它，操作者要的那一步**一步都走不动**。触发条件刻意收窄：**只因 `no_traj_reference` 关闸**
那一拍才发（那条原因**蕴含** armed + `OFFBOARD` + status 新鲜），离开 OFFBOARD 才重新武装；
**默认关**（不传参时行为与今天逐字一致），由 `z0.py` 的模型预测控制档打开。

### 2026-10-16（二定）—— 落了什么、跑了什么

**落了什么**：

- `run/z0.py`：菜单 8 的 `MANUAL_CHOICES` 只剩一档（`none`）+ `ask_chain_choice` 的**单档短路**
  （印一句"只有一档，**不问**"，直接返回）；菜单 9 的 `UP_CHOICES` 两档，`tail` 分别是
  `("traj","control")` 与 `("traj","mpc")` ⟹ "轨迹必须起"是**结构上的事实**；
  `mpc` 那条动作三个 `OptionalArg`（`reference_source_traj` / `offboard_prestream` / `start_traj_reference`）。
- `src/traj/`：`Kind::kTakeoff = 4`、`Takeoff` 类（斜坡）、`takeoff_height_m` / `takeoff_speed_ms`
  两个参数（默认 0.2 / 0.2）；★ **轨迹 1/2/3 的实现与用例一行未改**。
- `src/nmpc/nmpc/mpc_node.py`：`offboard_prestream`（白名单 `REASONS_PRESTREAM = ("not_armed",
  "not_offboard")`，帧是 `_hover_sp`，JSONL 里 `fallback="prestream"`）与 `start_traj_reference`
  （触发原因 `REASON_START_TRAJ = "no_traj_reference"`、重新武装 `REASONS_REARM_TRAJ_START`、
  发布者**只在那一档才建**）；`_validate` 拒 `start_traj_reference && reference_source != traj`。
  **两条默认都是 `False`**。
- `src/bringup/scripts/fly.py`：`KINDS` 与命令字面加第 4 条（`4 = 起飞（直上 0.2 m）`）；
  可执行位 `100755` 未动（`install(PROGRAMS)` 那条纪律）。

**跑了什么**（2026-10-16，全部现算）：

| 套件 | 结果 |
|---|---|
| `src/nmpc/test`（整包） | **269 passed**，526 s —— `ACADOS_SOURCE_DIR=/home/chen/acados` **与** `LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib` 两个都设了（一版基线 266 + 本轮新增 3） |
| `src/traj` seam 2（gtest） | **26 passed**（`test_trajectories`，含 `Takeoff` 6 例） |
| `src/traj` seam 1（pytest） | **12 passed**，39.6 s（`BENCH_ROS_DOMAIN_ID=79`） |
| `run/tests` 四个文件**分开点名** | 83 + 80 + 5 + 10 = **178 例全绿** |

★★ **一个环境陷阱，记在这里免得下次再踩**：`src/nmpc/test` 里那条夹具起的 DUT 子进程**同时要**
`ACADOS_SOURCE_DIR` 与 `LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib`。**只设前者**时子进程死在
`OSError: libhpipm.so`，而症状是**每条用例各等 120 s 的 `BenchTimeout`**（看着像代码坏了，
其实是环境）—— 所以那个"526 s"里有一部分是这类等待，别拿它当性能读数。

**变异测试**（★ 新断言都主动弄坏一次；每条都先把文件备份到 `/tmp`，恢复后用 `diff -q` 逐字节核过）：

| # | 把什么弄坏 | 红在哪 |
|---|---|---|
| A | 放宽预流白名单（`REASONS_PRESTREAM` 里塞进 `status_stale`） | `test_预流_只在那两档发悬停帧_别的关闸原因照旧一帧不发` —— `status_stale` 期间发了 **90 帧** |
| B | 去掉 `_maybe_start_traj` 的"只发一次"锁存 | `test_自发起步_开了之后由节点自己按开始键_没人按就一直卡着` —— **收到 2 次** `/traj/start` |
| C | 无条件建 `/traj/start` 发布者（去掉 `if params.start_traj_reference`） | 同文件 —— 默认档也建了发布者，`assert 1 == 0` |
| D | 去掉 `_validate` 对 `start_traj_reference && reference_source != traj` 的拒绝 | `test_自发起步只在轨迹档有意义_别的档入口当场拒` —— ★ **红成超时，不是红成断言**：DUT 不退出，用例等到夹具上限 |
| E | 把斜坡符号反过来（`height_ * (t/ramp_s_)` → `-height_ * …`） | gtest `Takeoff.斜坡_从原点升到高度_到顶后停住` + `Takeoff.有限差分证实斜坡内匀速_到顶后完全停住`；seam 1 的 `test_起飞_在话题上是一条斜坡_到顶后停住` 也红 |
| F | 把两档尾巴里的 `traj` 删掉（`("traj","control")` → `("control",)`、`("traj","mpc")` → `("mpc",)`） | `test_z0_menu` **8 红**，含 `test_两档的尾巴都起_traj_且模型预测控制档带三条参数`、`test_dry_run_把两档的命令都摊开且默认那档排在前面`；恢复后 83 绿 |

★ **D 与 E/F 的读法不一样**，别混：A/B/C/E/F 是"红在**那条**断言上"（鉴别力可直接读），
D 只说"这个入口没被拒绝 ⟹ 会撞到夹具上限"，**没有**单独指出是哪一条断言在守它。

**没采纳 / 记下来给下一轮的**：

- `_render_steps` 那句"★ 下面摊开的是默认那一档（第 N 档）"在**单档链**上也打 —— 一版的
  `--dry-run` 没有它。判为**有意的变更**（"不问"这件事必须在屏上可见，否则操作者会以为脚本坏了），
  并且它是第 8 项与一版之前**唯一**的两行差异（见验收第 1 条）。
- `src/traj/README.md` 旧写"限速扫三条轨迹" —— 已改四条（gtest 那条用例名就叫
  `限速.四条轨迹全程不超过限速`，现算不改就等于写错）。

★ **Status 不动**（`ready-for-agent`）：归谁收口是操作者的裁定（本仓那条纪律）。
★ **未提交**：等操作者确认（本仓纪律）。

### 2026-10-17 —— 第一次真起链：第 5 步当场退，查出来是环境变量

**现象**（操作者实跑 `run/z0.py up` ⟹ 第 9 项输 `2`，日志
`run/logs/20261017-014658-up/05-mpc.log`）：

```
File ".../src/nmpc/nmpc/ocp.py", line 300, in _build_solver
    self.solver = AcadosOcpSolver(ocp, json_file=str(json_path), verbose=verbose)
File ".../acados_template/utils.py", line 128, in get_shared_lib
OSError: /home/chen/refs/acados/lib/../lib/libacados.so: cannot open shared object file: No such file or directory
```

**判读**（★ 逐条核过，不是猜）：

- shell 里 **`ACADOS_SOURCE_DIR` 没设** ⟹ `acados_template` 从**它自己装在哪**去猜，猜到了
  `~/refs/acados` —— 那是 **acados 源码树**，`lib/libacados.so` 根本不在那儿；
- **只设它还不够**：缺 `LD_LIBRARY_PATH` 时 `libacados.so` 打得开、它依赖的 `libhpipm.so`
  找不到（同一个 `OSError`，换个文件名 —— 这条是票 13 踩过的）；
- ⟹ **不是代码坏、不是这次改动引起的**：`mpc_node.py` 一帧都还没发，死在 `ocp.py` 建求解器。

**修法**：`mpc` 那条动作的命令自带
`env ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$HOME/acados}" LD_LIBRARY_PATH="…/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"`
（`run/z0.py` 的 `ACADOS_ENV`）—— **写进命令、不靠操作者记着先 export**（"一键"就得真是一键），
且 `--dry-run` 摊出来的是同一条命令。外层已经 export 对了就听外层的（同
`src/nmpc/test/test_mpc_node.py` 的 `_dut_env()`）；`${LD_LIBRARY_PATH:+:…}` 是为了**不**在末尾
留一个空路径元素（空 = 当前目录）。

**验证**（★ 正反两面都有）：

- **正面**：在 **`ACADOS_SOURCE_DIR` 未设**的 shell 里执行 `--dry-run` 摊出来的那条命令
  （`ROS_DOMAIN_ID=97` 隔离）⟹ **10 s 起来**，日志有 `MPC（模型预测控制）节点启动：` 且
  `★预流=开 / 自发起步=开`；闸门 `no_status`（域里没有 mavros）⟹ 0 帧，符合预期；退出码干净。
- **反面**：就是当天真实那次起链（上面那段 `OSError`）—— 不需要再造一次。

**用例与变异**：新增 `test_mpc_那条命令自带_acados_的两个环境变量`（钉前缀里两个名字都在、
默认值 `$HOME/acados`、不覆盖外层、末尾不留空元素）；★ 把 `LD_LIBRARY_PATH` 从 `ACADOS_ENV`
里删掉 ⟹ **只有它红**（84 → 1 failed / 83 passed），恢复后 84 绿。
另：`test_动作表里没有任何会动飞机的入口` 那条"每条命令都以 `ros2 ` 开头"的断言**随之放宽**，
但**只放宽到「`env` 只设变量、后面仍走 `ros2`」** —— ⛔ 别把它放宽成"随便什么前缀"，
那句断言的用处正是"命令的出口只有 `ros2`"。

**文档同步**：`run/README.md`（`mpc` 那一行的"自带两个环境变量" + 隐藏动作表那段；
顺手修掉一处一版遗留的"第 3 / 4 档"）、`operation_manual.md` §3.1″、
`src/nmpc/README.md`（那两行 export 现在**只对手打/pytest**负责）。

★ 这处**不是**"文档缺失"：`src/nmpc/README.md` 一直写着要 export 这两条，是**起链没替操作者做**
—— 票面要的是"一键"，那就不能要求先手工 export。★ 修复后 `run/tests` 四个文件
**179 例全绿**（84 + 80 + 5 + 10）。

### 2026-10-17 —— 收口裁定（操作者）

> 「**被 15 取代后收口**」

在「做完了 / 被 15 取代后收口」两选一里选了后者 ⟹ `Status: wontfix`（判读与证据见文首收口块）。
**验收 11 条不撤销**：它们是落地当时的事实；第 2 条已就地标 ☠️ —— 被票 15 改掉的正是
「三条参数」那一半。**本票留下的依赖**：轨迹 4 的 `src/traj` 实现仍未提交，归
[票 15](15-trajectory-flight-roles.md)。
