# 12: 第 7 章 启动脚本与遥控等交互（含安全/FSM）

Status: ready-for-human
Type: task

**What to build:** `docs/report/chapters/07-launch-and-rc.md`。这是"现场那一页"的报告版，
不是代码导读。

**Blocked by:** —

## 内容

- **一键起链**：`run/z0.py` 菜单与 `up.sh` / `down.sh`；两条一键链
  （手动遥控飞 `up_manual` / offboard 或模型预测控制全链）；
  "谁在上位控制飞机"那一问的含义；
- **启动顺序与判读**：mavros → vrpn → mocap → traj → control（→ model predictive control）；
  每一步的就绪判据（`connected=True`、`/mocap/pose_enu` 有帧、`/control/state` 有帧、
  模型预测控制启动行）；
- **输出路径**：offboard 写 `setpoint_raw/local`；模型预测控制写 `setpoint_raw/attitude`，
  `control` 只写 `/control/target`；
- **录制**：场景 1–6、120 s 倒计时、`session.json`、`MANIFEST.md`；只订阅、不解锁、不发帧；
- **遥控**：
  - ch9 = 开始键（两次上升沿 + 一次长按）；
  - ch8 = 硬件杀车；
  - ch5 = 解锁开关（程序解锁为主，`RC_MAP_ARM_SW` 保留冗余）；
  - ch6 = 模式开关（位置 6 = Offboard）；
  - `COM_RC_IN_MODE=0`；
- **FSM**：两个正交维度（控制权 × 任务态）；正常路径与失效路径；三条软停路径、两条复位路径；
  转换表引用 `xi35-fsm.md`（不重抄全表）；
- **安全/失效**：
  - 打杆接管 = 立即停发 setpoint，由 PX4 按 `COM_OBL_RC_ACT` 处理；
  - offboard 心跳丢失由 PX4 失效保护处理；
  - 回退 (A) 级是一次演练过的操作；
  - 引用 `safety-paths-drill.md` 与 `docs/assets/drill/` 的第一手日志；
- **现场边界**：这条链里没有什么（不自动起桨、不自动切模式等）。

## 来源

- `run/README.md`；
- `operation_manual.md`；
- `docs/control/phase1-implementation.md` §4；
- `docs/reference/xi35-fsm.md`；
- `docs/control/safety-paths-drill.md`；
- `docs/assets/drill/*.log`；
- `docs/reference/params-changes.md` §H。

## 投影用一句话总结（2026-10-17 追加）

每个叶子内容小节（没有子标题的 `##`/`###`；独立「小结」节同样适用）**末尾**加一行，且只加一行：

`<big>**一句话总结：……**</big>`

- 一句话，建议 20–50 字、最多 60 字；只概括本节结论，能脱离上下文朗读。
- 不写“本节介绍……”；不引入本节之外的数、不引用图号/表号；不写“效果良好/基本通过”。
- 不单独成标题，不进目录。细则见 [spec.md](../spec.md) §8。

## 检查清单

- [x] 每个叶子小节末尾有且只有一行 `<big>**一句话总结：……**</big>`，且句子不引入本节之外的数

- [x] 命令、菜单号、开关位置与 `run/README.md` / `operation_manual.md` 逐字一致
- [x] 遥控通道表与 `params-changes.md` §H 一致；不写"只剩 ch7 空闲"这类已被推翻的话
- [x] FSM 写成"两个正交维度"，不是一个扁平枚举
- [x] 安全路径有第一手日志引用，不写成"应该会"
- [x] 不把 `NAV_RCL_ACT` 的旧值（Return）当现状；现值以实测为准
- [x] 中文母语写作，无裸词 `MPC`

## 验收

- 操作者审过才 `done`；审阅记录写进 `## Comments`。

## Comments

**2026-10-17 初稿完成，转 `ready-for-human`。**

本章摘要（10 行内）：

- 7.1 钉死控制权/任务态两个维度与 ch 编号约定；7.2 讲 11 项菜单、四条常用项、两条一键链与"谁在上位控制飞机"。
- 7.3 给 mavros → vrpn → mocap → traj →（模型预测控制）→ control 的判据与超时；7.4 说明三条输出路径与两种模式下的最后执行者。
- 7.5 讲场景 1–6、120 s 倒计时、`session.json`/`MANIFEST.md` 与"只订阅"的 AST 守卫；7.6 按 `params-changes.md` §H 给四个开关的职责与实测档位。
- 7.7 把状态机写成控制权 × 任务态并给正常路径七步；7.8 给三类失效、三条软停、两条复位与闩锁；7.9 逐条引用五场演练的第一手日志；7.10 列出这条链里没有的东西；7.11 给引用清单。

引用/证据：`run/README.md`、`run/z0.py`、`operation_manual.md`、`docs/reference/params-changes.md` §H、
`docs/control/phase1-implementation.md` §4、`docs/reference/xi35-fsm.md`、`docs/control/safety-paths-drill.md`、
`.scratch/phase4/issues/08-safety-paths-and-fallback-drill.md`、`docs/assets/drill/*.log`（六份）。

未确定项/待清扫（本章按现状写，未改源文件）：

1. `safety-paths-drill.md` §0/§2.1 仍写"打杆只试过半场"，与 07:23 实测通过（`degrade=rc_override`）不符；
2. 同文件 §3.4 仍写 (C) 链"没有工具，只能 Ctrl-C"，`DOWN_TARGETS` 现已含 `mpc`；
3. `run/z0.py` 的 `DOWN_WARNING` 仍写 `NAV_RCL_ACT = 2`（返航），实测现值为 `1`（Hold，2026-09-28）；
4. `operation_manual.md` §1 括号内 `COM_FLTMODE4=1`，实测为 `8`（Stabilized）；
5. 票面「内容」写「回退 (A) 级是一次演练过的操作」，实测只有第 1/4/5 步；第 2、3 步（切换动作本身）从未演练。
6. 求解失败注入未演练；回退 (A) 的切换动作同样未演练（票 08 已记操作者接受风险，无证据锚）。

字数：302 行，约 2.00 万字符（含标点）。
