# 04: 固件重编重刷（**修法待重定**）+ 版本终审 + 刷前/刷后参数 diff

Status: needs-info
Type: task

> ☠️ **2026-09-23 整票重写。** 原票的动作是「把 `ADC_BATTERY_CURRENT_CHANNEL` 从 `ADC12_CH(5)` 改成
> `ADC12_CH(8)`，打在 `5ba2e91f` 上」—— **对象与修法双错**：
> ① `5ba2e91f` 是**与 z0 无关**的 fork（真身 = `Minderring/PX4-Autopilot` @ `4a0e65f2`）；
> ② 那个 `CH(5)`→`CH(8)` 修的是 `boards/hkust/nxt-dual`，而**真身树里没有 `hkust/`**，
> 它的 `boards/micoair/h743-aio` 写 `/* PC1 */ ADC1_CH(11)`，**与引脚相符、本来就没有这个缺陷**
> （2026-09-21 上电实测亦判「未中招」，见 [`params-changes.md`](../../../docs/reference/params-changes.md) §A4）。
>
> **⟹ 该票现在卡在一个未决问题上**（下面第 0 项），所以 `Status:` 从 `ready-for-human` 降为 **`needs-info`** ——
> **等的就是操作者回答"板级目标到底是哪块"**。答上之后本票回到 `ready-for-human`。

**What to build:** 飞控跑上**自己重编的**固件，并且**关于这台飞控的三个未知量**被同时回答掉：它实际报什么版本号、刷固件会不会重置参数、（顺带）电池电流读数是否正常。

### 0. ★ 动手前必须先解决：编哪块板？（**本票当前的唯一阻塞**）

真身树（`Minderring/PX4-Autopilot` @ `4a0e65f2`）的 `boards/` 里**只有 `micoair/{h743, h743-v2, h743-aio}`，没有 `hkust/`**；
而实物板名是 **MicoAir NxtPX4v2**，**对不上任何一个目录名**。（`hkust_nxt-dual` 只是**上游 ≥ 1.15.0 的目标名**，v1.14.3 里也没有它。）
三个 `micoair` 目标的板级默认值**互不相同**（`BAT1_A_PER_V = 40 / 40 / 14.14`、`IMU_GYRO_CUTOFF = 80 / 80 / 100`、`SYS_HAS_MAG = 1 / 1 / 0`），
而 `z0.params` 的实测值（`BAT1_A_PER_V = 17.0`、`BAT1_V_DIV = 10.02`）**与三个都不匹配**。
→ 详见 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) §2.5(d)④ 与 [`code-inventory.md`](../../../docs/reference/code-inventory.md) A.6。
**这一步没结论之前，下面任何一条都不要动手。**

- [ ] **0.** 确认板级目标（操作者：看丝印 / 量引脚 / 问厂商，三选一），并把结论回填到 `refs-manifest.md` §2.5(d)④

### 1. 重编重刷

- [ ] 按 Phase 2 记录的构建步骤，从**真身那棵树**（`4a0e65f2`）复现出一致的基线（刷之前先确认构建可重现，否则出错时分不清是哪一步）
- [ ] 第 0 项定了之后，再决定**要不要夹带任何改动** —— ☠️ ~~"只打那一行 ADC 修复"~~ 已作废：**当前没有已知要打的改动**，重编的目的一度只是"拿到可控的、可重现的固件基线"；若第 0 项查出目标不匹配，**这次重编本身就是必需的**
- [ ] **不引入任何其它上游改动**（Q12：不追上游）

### 2. 版本终审 + 参数 diff

- [ ] **刷前**从 QGC 导出 z0 自己的参数并留档
- [ ] 刷后**在 QGC 重新记录飞控实际报出的版本号**（`flight_sw_version` / `flight_custom_version`）—— 这是版本问题的**唯一终审**，也是 `px4_msgs` 选版的依据
- [ ] **刷后**再导出一次，与刷前那份 diff —— ★ **「刷固件是否重置参数、重置哪些」本轮未核实**：不假设"会重置"也不假设"不会"，**只按"必须重新 diff 一次"执行**
- [ ] 把上面这个行为**第一手核实掉**并记进文档（Q3 的「第一手可复核」纪律）
- [ ] 刷前/刷后两份导出都与 [`params-changes.md`](../../../docs/reference/params-changes.md) 的参考表 diff 过，差异清单留档 —— 参考参数文件是**另一台机器**导出的，**差异才是真正要改的东西**
- [ ] 若版本号与 `z0.params` 头声明的 `1.14.3 dev` / `4a0e65f2` 不一致，以飞控实际报出的为准并更正文档

### 3. 电池读数（☠️ 降级为"顺手核对"，不再是本票的目的）

- [ ] `listener battery_status` 记录电流读数 —— ☠️ ~~核实"不再是恒 0 / 怪值"（中招的判据）~~ 已作废：**真身不可能中招**（无 `hkust/`），且 **2026-09-21 已实测判「未中招」**（[`params-changes.md`](../../../docs/reference/params-changes.md) §A4）。此处的价值是**重编后再确认一次**（万一日志与实测不符，那才是新信息）
- [ ] 确认 `MC_BAT_SCALE_EN` **保持 0** —— ☠️ 理由已收窄："ADC 已知缺陷"不再是理由；保留的理由是**别把电池测量噪声引进控制环**（见 [`phase2/spec.md`](../spec.md) 的通信约定）

**Blocked by:** 第 0 项（操作者回答板级目标）；跨 feature —— [`phase2/issues/01-sitl-px4-msgs-baseline.md`](../../phase2/issues/01-sitl-px4-msgs-baseline.md)（那棵树的完整构建环境，本票重编要复用它）与 [`.scratch/phase1/issues/01-hardware-prereqs.md`](../../phase1/issues/01-hardware-prereqs.md)（QGC 只能通过串口连飞控，刷前导出依赖它已确认串口）。

---

**为什么是 `needs-info`（而不是 `ready-for-human`）**：本票现在唯一不能开工的原因是**"编哪块板"没有答案**，那要操作者提供，不是 agent 能查出来的。答上之后，接飞控、刷固件、在 QGC 里读数这些动作仍然要人做 ⟹ 届时改回 `ready-for-human`。
