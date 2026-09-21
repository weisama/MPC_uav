# 09: `WAIT_START → TRACK` —— 三条轨迹实飞 + 交互脚本 + bringup

Status: ready-for-agent
Type: task

**What to build:** **完整的自主任务闭环。**

悬停稳定后程序停在 `WAIT_START` 等操作者按遥控器开始键（**200 ms 去抖**），按下 → `TRACK` 执行所选轨迹；轨迹执行完毕 → `LAND`。启动脚本让操作者**输入数字 1/2/3 选轨迹**，不需要改代码或重新 launch；飞行中能看到当前状态与轨迹完成度，所以判断飞机行为时知道它"应该"在做什么。

同时做**一键 bringup**：一条命令起 mocap + traj + control（+ mavros）。

**三道防线在本张汇合：**
- 开始键按下之前，程序**不发送任何控制指令** —— 飞机在我不期望的时候不会进 offboard
- 任意打杆立刻交还遥控器（[`06`](06-control-failsafe-paths.md) 已实现，本张验的是它在**真实飞行**里的行为）
- offboard 心跳丢失由 PX4 失效保护接管，程序侧不试图自己兜

★ **开始信号的具体通道来自 [`02`](02-rc-channel-decision.md) 的结论**，不是本张自己拍板。在 02 resolved 之前实现保持**参数化**（`start_channel` + `start_threshold` + 去抖），不写死任何通道号。

**Blocked by:** 02, 07, 08

- [ ] `WAIT_START→TRACK` 由遥控器开始键**上升沿**触发，200 ms 去抖
- [ ] 开始键按下之前**不发送任何控制指令**
- [ ] 启动脚本输入 1/2/3 切换轨迹，无需改代码或重新 launch
- [ ] 飞行中当前状态与轨迹完成度**可实时看到**
- [ ] 一键 bringup：一条命令起 mocap + traj + control（+ mavros）
- [ ] 开始键通道在台架上用 `/mavros/rc/in` 核对过，确认**不触发 PX4 模式切换**（若与 [`02`](02-rc-channel-decision.md) 的决定不符，回 02 改决定，不要就地改代码）
- [ ] 三条轨迹在自由飞里都跑完：1 m 悬停 / 1 m 正方形 / 0.5 m 八字
- [ ] `TRACK` 完成后自动进 `LAND`，任务结束上锁
- [ ] **§5.3 验收全部达标**：定点水平误差 **≤15 cm**、**持续 60 s**、**全程不碰摇杆**、能触发自动降落
