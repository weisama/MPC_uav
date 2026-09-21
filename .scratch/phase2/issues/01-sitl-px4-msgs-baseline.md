# 01: SITL + `px4_msgs` 版本基准锁定

Status: ready-for-agent
Type: task

**What to build:** 一条能跑的 SITL 链路，且它与 `px4_msgs` **同出于 z0 固件真身**。这是 Phase 2 全部后续工作的地基 —— 也是 Phase 3 重编固件（ADC 一行修复）要复用的构建环境，所以它顺手交付的东西比它自己看起来多。

**Blocked by:** ⚠️ **2026-09-21 起新增一条**：[phase1/11 飞控自报 1.14.3 vs 文档的「v1.15 世代」](../../phase1/issues/11-firmware-version-conflict.md)。
本张的**整个地基**是 `5ba2e91f` = z0 固件真身，而飞控第一手报出的是 **1.14.3**（[`refs-manifest.md` §2.5(c)](../../../docs/reference/refs-manifest.md)）。
**在那条冲突定案前，本张的"从 fork 生成 `px4_msgs`"这个前提不能算已确立** —— 若实际是 1.14 系，
`OffboardControlMode`（offboard 心跳必需的那条）的定义**是反的**。

**为什么必须是 fork 而不是上游 `px4_msgs`**：v1.15 及以前，`px4_msgs` 与固件不匹配时**话题存在、能发布、无报错，但 PX4 不理会** —— 排查时你会怀疑 MPC（模型预测控制）、怀疑符号、怀疑一切，唯独不会怀疑消息定义。而 `release/1.14` 作基准**已被实测否决**（184 个共用消息里 28 个字段布局不同，**含 `OffboardControlMode` —— offboard 心跳必需**）；`release/1.15` 也不必试，因为 `5ba2e91f` 早于 v1.15.0 发布，是走向 v1.15 的中间 commit。

- [ ] 从 fork `Peize-Liu/PX4-Autopilot @ 5ba2e91f` 做**带 `--recursive` 的完整 clone**（23 个 submodule），落在**仓库外的构建目录**（⚠️ 现有 `~/refs/px4-z0` 是 fetch-only，**不能直接构建**）
- [ ] SITL 起得来；机型的可用性在**动手前**确认过（不是假设它在），gazebo-classic 与 gz-sim 皆可
- [ ] `px4_msgs` **从 fork 自己的 `msg/` + `srv/` 生成**，不从任何上游 release 分支取
- [ ] 一致性判定留证：`Tools/check-message-compatibility.py` 与 `ros2 interface show px4_msgs/msg/VehicleRatesSetpoint` 双向比对，且**明确覆盖 `OffboardControlMode`**（已知它是被破坏的那个）
- [ ] `micro-xrce-dds-agent udp4 -p 8888` 接通后 `/fmu/in/*` 与 `/fmu/out/*` **双向前缀**都出现在 `ros2 topic list` 里
- [ ] 构建步骤（版本、commit、submodule 处理、命令、判定结果）写入文档，使 Phase 3 能照做重编固件
- [ ] 记录里写明：**SITL 从 fork 构建 ≠ 刷飞控**（Q12 不刷固件的决策不变）；Phase 2 不碰飞控，只碰仿真

---

**本张的判据刻意不止于"话题列出来了"** —— 那正是本坑骗人的地方。真正的判定在 [05](05-sitl-offboard-smoke.md)：PX4 是不是**真的照做**。
