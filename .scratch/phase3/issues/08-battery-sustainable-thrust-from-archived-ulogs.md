# 08: 从已归档的飞控日志读出"电池能持续供出多少推力/功率"

Status: ready-for-agent
Type: task
**对着 Phase 2 的：[`10`](../../phase2/issues/10-param-measure-backfill.md)**（`ready-for-human`，四条裁定已落地，只剩这一条）
**阻塞：** 无

**要做什么（一句话）：** phase2/10 的清单只剩「电池**可持续**供出多少推力/功率」这条
（§6.2 的 **15 A 瓶颈**），它决定 `f_max = 11.8 N` **保不保守**。
**不必新开台架日** —— 读数就在已入库的飞控日志里。

## 为什么这一票能从"必须人做"降级成"读日志"

phase2/10 把这条记成「一个台架/悬停测量（**只有人能产**）」。
本阶段的操作者裁定（[spec](../spec.md) Further Notes ②）把它**降级**为
「**从已归档的飞控日志读出来**」，读数落回票面 —— 因为 `bag/*/fcu/*.ulg` 里
就有负载下的电压/电流。**已第一手核过，五个场景都有 `.ulg`**：

```
bag/1_static/fcu/log_0_2026-9-29-15-22-20.ulg         9.4 MB
bag/2_manual_takeoff/fcu/log_1_2026-9-29-15-25-14.ulg 5.5 MB
bag/3_traj1_hover/fcu/log_0_未知日期.ulg             12.2 MB
bag/4_traj2_square/fcu/log_1_未知日期.ulg             5.5 MB
bag/5_traj3_lemniscate/fcu/log_4_未知日期.ulg        10.8 MB
```

且 `battery_status` 这个 topic **确实在**，字段也够用（`voltage_v` / `voltage_filtered_v` /
`current_a` / `current_filtered_a` / `current_average_a` / `discharged_mah` /
`average_power` / `voltage_cell_v[*]`）；`actuator_motors` / `actuator_outputs` 也在。

## 现成的读法别重造

仓库里已经有 pyulog 适配层，**照它的样式写**：

- [`tools/hover-thrust/hover_thrust_from_ulog.py`](../../../tools/hover-thrust/hover_thrust_from_ulog.py) ——
  它已经解决了「**怎么从日志里判"在飞"**」（`_in_flight_mask`：`flag_armed` + `landed` 两路）
  与「推力怎么换算」；**它只是没读 `battery_status`**；
- [`tools/tau-analysis/tau_from_ulog.py`](../../../tools/tau-analysis/tau_from_ulog.py) —— 同族的另一个适配层，可参考体例。

⟹ 本票多半是**加一个读电池的适配层**（或给现有工具加一条通道），不是从零解析 `.ulg`。

## 要回答的问题（票面结论必须是这几条之一，不能是"大致看了看"）

1. **可持续的电流上限是多少** —— 判据要选**能区分对错的那一列**：
   不是"起飞那一瞬的最大电流"，而是**同一块电池在电量低端**还能持续给出多少
   （放电末端的电压跌落才决定"还能不能顶住"）。写清用的是哪段窗口、依据哪个字段。
2. **换算成推力/功率是多少** —— 与 `f_max = 11.8 N` 并排给出，结论**保不保守**；
   并说清 `f_max`（OCP 的 `F` 上界，`ocp.py:247` 的 `ub[0]`）与 `t_max = 46.3027 N`
   （机架满量程）**是两个独立的量**，别混着算。
3. **这次取不到的量要如实点名** —— 例如日志里没有电流标定（`battery_status.scale`
   为 0 或明显未经标定）时，`current_a` 就**不能当判据**；这时要么换 `actuator_motors`
   侧推算，要么写明"这次取不到那个量"。

## 验收

- [ ] 结论落在本票 Comments（数字 + **口径** + **哪份日志** + 复跑命令）
- [ ] 复跑命令能在**不碰飞控**的前提下重现读数（只用本地 `.ulg`）
- [ ] ★ 产物**落仓库不落 `/tmp`**（根盘 ext4 + tmpfiles 一条 `D /tmp`；过不了重启）
- [ ] 读数回填进 phase2/10 的 `## Comments`；**不改它的 `Status:`**
- [ ] ★ 若这次的读数**推翻了** `f_max = 11.8 N` 的保守性判断，**另开票**，
      别在本票里顺手改 OCP 的上界

## 边界

- ⚠️ **只读**：不发任何 MAVLink、不碰飞控、不起链。
- ⚠️ **零点不同就别相减**：不同场景（不同电池、不同起始电量）的电压/电流**不是同一 datum**，
  横着相减出来的是"起机零点差"，不是分歧。要比就先说清比的是哪一段。
- ⚠️ **归档证据不就地改**：`bag/*/fcu/*.ulg` 是那一趟的原始留存。

## Comments

### #1 为什么它 `ready-for-agent`

纯离线读文件：不碰飞机、不占窗口、不需要现场。判据的形状是"**复跑命令能重现读数**"，
AFK agent 能自己判。★ 母票 `10` 是 `ready-for-human`（因为原先写的是"只有人能产"），
本票把那条**降级**成日志读取之后，人这一侧的门槛就没了 —— 所以标签**故意不一样**，
这个差别本身要留在票面上。
