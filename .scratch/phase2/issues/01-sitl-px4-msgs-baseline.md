# 01: SITL + `px4_msgs` 版本基准锁定

Status: ready-for-agent
Type: task

**What to build:** 一条能跑的 SITL 链路，且它与 `px4_msgs` **同出于 z0 固件真身**。这是 Phase 2 全部后续工作的地基 —— 也是 Phase 3 重编固件（ADC 一行修复）要复用的构建环境，所以它顺手交付的东西比它自己看起来多。

**Blocked by:** ⚠️ **2026-09-21 重写**：原写「本张地基 = fork `5ba2e91f`」——**这个地基没了**。
[phase1/11](../../phase1/issues/11-firmware-version-conflict.md) 已定案：飞控跑的是
**`4a0e65f2`（v1.14.3 系的 MicoAir 构建）**，其 `msg/` 与 **上游 v1.14.3** 逐字节相同（185/185），
与 **`px4_msgs release/1.14` @ `ffb6e80e`** 在**共有的 184 个文件**上 blob 全同
（[`refs-manifest.md` §2.5(d)](../../../docs/reference/refs-manifest.md)）。
剩余阻塞只有一条：**用户对 §2.6 改法的签字**（即 phase1/11 的 ②）—— ✅ **2026-09-21 已签字**。
⟹ `Status` 回到 **`ready-for-agent`**，下面这份清单**可以照此执行**
（[§2.6 执行版](../../../docs/reference/refs-manifest.md) 已改）。
⚠️ 唯一"我取的默认、未单独表态"的一项：**SITL 源用上游 v1.14.3**（用户只签了 `px4_msgs` 那一半）。

**为什么现在反而是上游 `release/1.14`（这是对原先反向结论的纠正）**：v1.15 及以前，
`px4_msgs` 与固件不匹配时**话题存在、能发布、无报错，但 PX4 不理会** ——
排查时你会怀疑 MPC（模型预测控制）、怀疑符号、怀疑一切，唯独不会怀疑消息定义。
所以「配对」这件事仍然要紧；**只是配的对象换了人**：
原判「`release/1.14` 作基准已被实测否决」是拿 **fork 的 `msg/`** 比的，而飞控不跑 fork。
对**真身**（v1.14.3 系）`release/1.14` 在**共有的 184 个 `msg/` 文件上 blob 全同**（0 个不同）。
**185/184 这两个数不矛盾** —— 固件那棵树是 185 个（多了 `msg/CMakeLists.txt`，PX4 自己的构建文件，
`px4_msgs` 本就不带），差异只在**文件集合**，不在任何共有文件的内容。

- [ ] 从**上游 `PX4-Autopilot @ v1.14.3`**（tag `1dacb4c`）做**带 `--recursive` 的完整 clone**，落在**仓库外的构建目录**（⚠️ 现有 `~/refs/px4-v1.14.3` 是 `--depth 1 --filter=blob:none`，**不能直接构建**）
  - ⚠️ 动手前先实测 airframe 条目数，确认 Phase 2 要用的机型**在**（§2.6 记过 v1.14.3 这里条目"较少"）
  - 也可改为直接编**飞控真身那个 commit**（`4a0e65f2`，`Minderring/PX4-Autopilot`）——它才有 `boards/micoair/*`；但对 SITL 而言上游 v1.14.3 已够，且更好复现 ⟹ **默认取上游**（§2.6 执行版已如此记）
- [ ] SITL 起得来；机型的可用性在**动手前**确认过（不是假设它在），gazebo-classic 与 gz-sim 皆可
- [ ] `px4_msgs` 钉 **`release/1.14` @ `ffb6e80e`**（**用户已签字**；不是从 fork 生成 —— 已实测它与真身在**共有的 184 个文件**上 blob 全同）
- [ ] 一致性判定留证：`Tools/check-message-compatibility.py` 与 `ros2 interface show px4_msgs/msg/VehicleRatesSetpoint` 双向比对，且**明确覆盖 `OffboardControlMode`**（已知它是被破坏的那个）
- [ ] `micro-xrce-dds-agent udp4 -p 8888` 接通后 `/fmu/in/*` 与 `/fmu/out/*` **双向前缀**都出现在 `ros2 topic list` 里
- [ ] 构建步骤（版本、commit、submodule 处理、命令、判定结果）写入文档，使 Phase 3 能照做重编固件
- [ ] 记录里写明：**从上游 v1.14.3 构建 SITL ≠ 刷飞控**（Q12 不刷固件的决策不变）；Phase 2 不碰飞控，只碰仿真

---

**本张的判据刻意不止于"话题列出来了"** —— 那正是本坑骗人的地方。真正的判定在 [05](05-sitl-offboard-smoke.md)：PX4 是不是**真的照做**。
