# 01: SITL + `px4_msgs` 版本基准锁定

Status: done
Type: task

**What to build:** 一条能跑的 SITL 链路，且它与 `px4_msgs` **同出于 z0 固件真身**。这是 Phase 2 全部后续工作的地基 —— ~~也是 Phase 4 重编固件（ADC 一行修复）要复用的构建环境~~，所以它顺手交付的东西比它自己看起来多。
☠️ **2026-10-09**：~~"Phase 4 重编固件要复用"~~ 这个下游用途**没了** —— 重刷固件已被操作者裁定取消
（[`phase4/issues/04`](../../phase4/issues/04-firmware-reflash.md)，`wontfix`），且那条 ADC 修复**本来就不需要打**
（z0 的板级配置早就是 `ADC12_CH(8)`）。**本票交付物本身仍然有效** —— 它是 SITL 环境，与刷不刷飞控无关。

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

- [x] 从**上游 `PX4-Autopilot @ v1.14.3`**（tag `1dacb4c`）做**带 `--recursive` 的完整 clone**，落在**仓库外的构建目录**（⚠️ 现有 `~/refs/px4-v1.14.3` 是 `--depth 1 --filter=blob:none`，**不能直接构建**）
  - ⚠️ 动手前先实测 airframe 条目数，确认 Phase 2 要用的机型**在**（§2.6 记过 v1.14.3 这里条目"较少"）
  - 也可改为直接编**飞控真身那个 commit**（`4a0e65f2`，`Minderring/PX4-Autopilot`）——它才有 `boards/micoair/*`；但对 SITL 而言上游 v1.14.3 已够，且更好复现 ⟹ **默认取上游**（§2.6 执行版已如此记）
- [x] SITL 起得来；机型的可用性在**动手前**确认过（不是假设它在），gazebo-classic 与 gz-sim 皆可
- [x] `px4_msgs` 钉 **`release/1.14` @ `ffb6e80e`**（**用户已签字**；不是从 fork 生成 —— 已实测它与真身在**共有的 184 个文件**上 blob 全同）
- [x] 一致性判定留证：`Tools/check-message-compatibility.py` 与 `ros2 interface show px4_msgs/msg/VehicleRatesSetpoint` 双向比对，且**明确覆盖 `OffboardControlMode`**（已知它是被破坏的那个）
- [x] `micro-xrce-dds-agent udp4 -p 8888` 接通后 `/fmu/in/*` 与 `/fmu/out/*` **双向前缀**都出现在 `ros2 topic list` 里
- [x] 构建步骤（版本、commit、submodule 处理、命令、判定结果）写入文档，使 ~~Phase 4 能照做重编固件~~（☠️ **2026-10-09**：重编已裁定取消，此用途没了；步骤本身已交付且对 SITL 复现仍有效）
- [x] 记录里写明：**从上游 v1.14.3 构建 SITL ≠ 刷飞控**（Q12 不刷固件的决策不变）；Phase 2 不碰飞控，只碰仿真

> ★ **三处交付形态与上面清单原文不同**（都已核实，理由见 Comments）：SITL 后端是 **SIH** 而非 gazebo；
> 一致性脚本改用 `tools/px4-sitl/check_msg_compat.py`（清单写的那个路径**不存在**）；Agent 二进制名是 **`MicroXRCEAgent`**。

---

**本张的判据刻意不止于"话题列出来了"** —— 那正是本坑骗人的地方。真正的判定在 [05](05-sitl-offboard-smoke.md)：PX4 是不是**真的照做**。

---

## Comments

### 2026-09-28 收口

**交付文档**：[`docs/control/px4-sitl-baseline.md`](../../../docs/control/px4-sitl-baseline.md)（构建步骤 / 版本与 commit / 五个坑 / 验收证据 / Phase 4 交接）。
**可复现脚本**：[`tools/px4-sitl/`](../../../tools/px4-sitl/) —— `build_px4_sitl.sh`、`build_xrce_agent.sh`、`run_sitl.sh`、`check_msg_compat.py`。

**落地值**

| | 值 |
|---|---|
| PX4 源码 | 上游 `v1.14.3` @ `1dacb4cdef2d7145754fc788fa8dc482eed74b40`，`~/refs/px4-v1.14.3-build/`（`--recursive --single-branch`） |
| SITL | **SIH**，`PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim`，`SYS_AUTOSTART=10040` |
| `px4_msgs` | `release/1.14` @ `ffb6e80e1c17e5714395611a020c282a87af8fa4`，`~/refs/px4_msgs` + colcon 工作区 `~/px4_msgs_ws`（**下位工作区**，在仓库外） |
| Agent | `MicroXRCEAgent`（`v2.4.2` @ `57d0862`），装到 `/usr/local/bin` |

**验收证据（全部当场跑出，非引用旧结论）**

1. **消息一致性 PASS** —— `check_msg_compat.py` 报 **共有 184/184、字段布局不同 0、解析失败 0**，
   `OffboardControlMode` 与 `VehicleRatesSetpoint` 均在共有集内且一致。
2. **真身 blob 三向核** —— 真身 `4a0e65f2`（对象在 `~/refs/px4-z0/`，`git cat-file -t` 确认；`log -1` 作者是 `Minderring`）
   的 `msg/` = **185 个文件**（184 `.msg` + `CMakeLists.txt`）：
   - 真身 ↔ 上游 v1.14.3：**共有 185、blob 不同 0、集合差异无**（← 这一条是本张**新补**的，把 §2.6 的结论坐实了）
   - 真身 ↔ `px4_msgs`：**共有 184、blob 不同 0**，唯一集合差异是 `CMakeLists.txt`（`px4_msgs` 不带）
3. **双向话题 + 类型 + 真实数据** —— `/fmu/in/*` **13 条**、`/fmu/out/*` **11 条**；
   类型解析出 `[px4_msgs/msg/VehicleRatesSetpoint]` / `[px4_msgs/msg/OffboardControlMode]` / `[px4_msgs/msg/VehicleStatus]`；
   `ros2 topic echo --qos-reliability best_effort --once /fmu/out/vehicle_status` 收到 `nav_state: 4`。
   ⚠️ **但按本张自己的话，这仍不是"通了"** —— 真判据在 [05](05-sitl-offboard-smoke.md)。

**三处与清单原文不同的交付形态（都已核实）**

- **① SITL 后端 = SIH，不是 gazebo。** 清单原文写「gazebo-classic 与 gz-sim 皆可」，
  **两条在本机都不成立**：gazebo-classic 的 dev 库 apt **无候选**（jammy/arm64 只有 `gazebo-common` 这种 arch:all 数据包）；
  gz-sim 在 v1.14.3 **只支持 Garden**（`gz_bridge/CMakeLists.txt:38-39` 明写 `IGN (Fortress and earlier) no longer supported`），
  要加 OSRF 源 + ~500 MB + 写 SDF 模型。SIH 零外部依赖。
  ★ **操作者 2026-09-28 裁定用 SIH，并接受其证明力代价**：SIH 是与我们控制器同层次的简化刚体 + 一阶电机模型，
  **证明不了"模型够真"**，只能证明约定对得上真 PX4（符号翻转、推力归一化）—— 而那恰好是 seam 3 的判据本身。
- **② 一致性脚本换了。** 清单写的 `Tools/check-message-compatibility.py` **在 PX4-Autopilot 里不存在**
  （v1.14.3 / v1.15.0 / v1.16.0 / fork `5ba2e91f` / `px4_msgs` 逐棵树查过，全无；仓库根**没有 `scripts/` 目录**）。
  它的真身在 `auterion/px4-ros2-interface-lib`，而**直接跑它会报假 FAILED**：该仓**无 `release/1.14` 分支**，
  它的类型清单按自己 main 走，会报 20 个 v1.14 根本不存在的类型为「缺失」；
  报错原文里**没有** `Some files differ` 那一段，即**零个字段不符**。
  ⟹ 改用 `check_msg_compat.py`：**复用官方那份 `message_hash`**（从该 clone import，不另写免得漂移），
  把类型换成两侧并集（184 个全覆盖），**严格强于官方脚本的子集检查**。
- **③ Agent 二进制名是 `MicroXRCEAgent`**（无小写别名）。清单写的 `micro-xrce-dds-agent` 会「命令找不到」。
  另：它的 superbuild 会去 checkout **上游已删除**的 `2.12.x` 分支，报错 `fatal: invalid reference`**不指向根因**；
  脚本改钉 tag `v2.12.2`（`~/refs/micro-xrce-dds-agent` 因此有 1 行改动，不再是纯只读副本）。

**已核实、但本张刻意不动的（**已开票承接**，不是"留给下一轮"）**

- `PROJECT.md:450`、`docs/reference/px4-interface.md:178` 与 `:284`（**2 个文件 3 处**）仍写着那个**不存在的脚本路径**
  （`:178` 还写成 `./scripts/…`）⟹ **已另开 [ticket 14](14-stale-check-msg-compat-path.md)** 承接。
  按 [ticket 02](02-acados-orin-build.md) 的先例，**本张只改自己那一节**
  （`phase2-implementation.md` §S1 及其下达给 S1 的指令、证据索引），**不顺手清扫别的票** ——
  开票点名是 `done` 的要求，动别人的文件不是。
- `phase2-implementation.md` 附录里「`gz_x500` 在 v1.14.3 存在」**仍是事实**，未改 —— 只在 §S1 点明
  **「存在 ≠ 本机可用」**。
- `refs-manifest.md` §1 原本**没有**本张新增的三个 `~/refs/` 副本 ⟹ 已补三行
  （`px4-v1.14.3-build/`、`micro-xrce-dds-agent/`、`px4-ros2-interface-lib/`）。
  本张新增的两个「不再严格只读」的副本（agent 改钉 1 行、PX4 构建树含 `build/` 产物）已在该表**逐行标明**。

**本张动过的文件**：`docs/control/px4-sitl-baseline.md`（新）、`tools/px4-sitl/`（新）、
`docs/reference/refs-manifest.md`（仅 §1 表新增三行 + 表头日期说明）、
`.scratch/phase2/issues/14-stale-check-msg-compat-path.md`（新，承接遗留）、
`.scratch/phase2/spec.md`（Q9 段落后补一段：后端落地为 SIH）、
`docs/control/phase2-implementation.md`（仅 §S1 / §6.1 末尾那条 S1 指令 / 附录证据索引）、本文件。

**未决事项**：脚本路径那 3 处 → **[ticket 14](14-stale-check-msg-compat-path.md)**。
证明力的真正判据本就在 [05](05-sitl-offboard-smoke.md)（S5 里程碑），本张按设计在那里交棒。

**收口后复核（`/code-review`）**：本张相关的 5 条已逐条回代码核实并处理 —— 其中
**`check_msg_compat.py` 的 `INTERFACE_LIB_COMMIT` 是死代码**这一条**成立且已修**：
那个 pin 原本只定义、从不比对，等于「复用官方算法、不漂移」这句保证是空的；
现在脚本**默认要求 `--interface-lib` 恰停在 `ac2e8aeecb22`**、偏离即拒绝运行
（`--allow-unpinned-algorithm` 可降级为警告），且跑通时输出会打印算法来源。
四条分支（命中 / 偏离 / 降级 / 非 git 仓）均已实测。
⚠️ 复核同时报了 **4 条属于 [ticket 02](02-acados-orin-build.md) 的发现**（`acados-orin-build.md` §6 的
「全部 ≤ 4.4 ms」与其上表 OpenMP 行矛盾、`mpc-roadmap.md` §8 里程碑 0 未含 SITL、`§S2` 验收漏「机上负载」前提、
`check_x500_ocp.py` 的 `--runs 1` 会抛 `StatisticsError`），**本张不动**，见对话中的转述。
