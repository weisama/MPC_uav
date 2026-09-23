# 03: 从 Phase 1 飞行日志量出端到端延迟 τ

Status: ready-for-agent
Type: task

**What to build:** 一个**有出处**的 τ，而不是一个抄来的文献值。

**Blocked by:** 跨 feature —— Phase 1 的实飞日志（[`.scratch/phase1/issues/09-track-mission-script.md`](../../phase1/issues/09-track-mission-script.md)）。

- [ ] 交叉相关**飞控 IMU** 与**动捕**两条序列（同一段 Phase 1 飞行日志），取相关峰
- [ ] 记录**来源**：哪次飞行、哪个相关峰、什么方法 —— 使这个数字**可追溯**，而不是从一个我没法回查的地方来的
- [ ] 补偿形式沿用 Phase 2 的**一行状态外推**（`x + v·τ`），**不做完整延迟补偿**（完整补偿另立 spec）
- [ ] 给出实飞起始值 **0**，并写明理由：在这个环节**过补偿比不补偿更危险** —— 多外推一点就是在用错误的状态喂控制器
- [ ] 已知参照：Phase 1 在旧机上对 `vehicle_visual_odometry_0` 实测 **50 ms**。**这是参照，不是结论** —— 本票要量的是 z0 这一段
- [ ] 结论与出处写入文档，并从本文件回链

---

⚠️ 相关但**不属于本票**：在实飞中改变 τ 做对比，在 [10](10-ladder-closed-loop-and-free-flight.md) 的系绳阶段完成。

**为什么是 `ready-for-agent`**：纯离线数据处理，不需要飞机通电 —— 只要有 Phase 1 的日志。

*⚠️ spec 层约束：Phase 3 不与 Phase 1 / Phase 2 并行开工。*
