# 21: 电调报「无信号」—— 上电一段时间后开始滴滴响

Status: needs-info
Type: task

**为什么单开一张：** [ticket 20](20-fcu-hardfault-i2c1-crash-dump.md) 刚立的**飞控硬故障**有一个肉眼可见的症状候选：
飞控重启/挂死的窗口里 DShot 停发，电调就会连续滴滴报「无信号」。
但**症状 ≠ 机理** —— 同一句「滴滴响」还可能是供电/接地、信号排线、驱动没起来、协议不匹配。
20 号票的焦点是「解锁被拦 + 转储解读」，**症状这一侧没人定义** ⟹ 单开一张，负责把现象钉死、给出判别实验。

## 一、现象（用户报告，2026-09-22）

> 电调上电一段时间后没收到信号滴滴响

★ **目前只有这一句。** 时间线（上电到开始响隔多久）、是否持续、飞控当时在不在线、几个电调一起响，全部未知
⟹ 这就是 `needs-info` 的原因，见 §五。

## 二、已确定的第一手事实（agent，2026-09-22 就地取）

### 2.1 实机读数（不是记忆，是刚从飞控上拉的）

取法：`/mavros/param/pull`（`force_pull: true`）→ `ros2 param dump /mavros/param`
→ **915 项**，存档 `~/.ros/evidence/z0_fcu_params_2026-09-22.yaml`（22685 字节）。

| 参数 | 值 | 对本票的意义 |
|---|---|---|
| `PWM_MAIN_TIM0` | **`-4` = DShot300** | ★ **输出协议的实际开关**。值表来自 `src/drivers/pwm_out/module.yaml`（px4-z0 @ `5ba2e91f`）：`-5`=DShot150 / **`-4`=DShot300** / `-3`=DShot600 / `-2`=DShot1200 / `-1`=OneShot / `>0`=PWM 频率。`reboot_required: true` |
| `PWM_MAIN_TIM1/2` | `400` / `400` | 其余定时器组仍是 400 Hz PWM（本机 4 电机只用 MAIN1–4 ⟹ `PWM_MAIN_FUNC1..4 = 101/102/103/104`） |
| `MOT_PWM_TYPE` | **不存在** | ⚠️ 别去找它：**官方 v1.14.3 / v1.15.0 也都没有**（两份 `src/drivers/pwm_out/module.yaml` 均无；`src/lib/mixer_module/motor_params.c` 只有 `MOT_SLEW_MAX` / `THR_MDL_FAC`）。**1.14 起协议由 `PWM_MAIN_TIM0` 决定** |
| `CBRK_IO_SAFETY` | `22027` | 安全开关已屏蔽 ⟹ **排除**「没按安全开关所以无输出」 |
| `DSHOT_TEL_CFG` / `DSHOT_3D_ENABLE` | `0` / `0` | 无遥测、非 3D |
| `DSHOT_MIN` | `0.055` | 默认值。语义是**解锁后**的下限（`module.yaml`：「Make sure to set this high enough so that the motors are always spinning while armed」），**与未解锁无关** |
| `SDLOG_MODE` / `SDLOG_PROFILE` | `2` / `129` | ★ `2` = **from boot until shutdown** ⟹ **未解锁的台架时段也在录**；`129` = bit0 默认集 + bit7（CV/避障）⟹ §四 T4 的 ulog 路线成立 |
| `COM_MOT_TEST_EN` | `1` | 点转测试可用（§四 T3） |

同时刻状态：`/mavros/state` = `connected: true, armed: false`；
`/mavros/battery` = **20.19 V / 17 %**（6S ⟹ 约 3.36 V/cell，**偏低**）。

### 2.2 ★ 源码结论：**驱动只要活着就在发帧，解锁与否都发**

| 结论 | 一手证据 |
|---|---|
| 「干脆不发」只有一个分支：`if (!_outputs_on) return false;` | px4-z0 @ `5ba2e91f` `src/drivers/dshot/DShot.cpp:417` |
| 而 `_outputs_on` 在 `Run()` 里**恒被置为 true** | 同上 `:533` `bool outputs_on = true;`（无任何条件）；**官方 v1.14.3 @ `1dacb4c` `:458` 写法一字不差** |
| **未解锁时发的是命令帧，不是静默** | 同上 `:436-448`：`stop_motors` 为真时逐个输出 `up_dshot_motor_command(i, DShot_cmd_motor_stop, …)`（`:445`） |
| `stop_motors` 何时为真 | `src/lib/mixer_module/mixer_module.cpp:475` `bool stop_motors = !_throttle_armed && !_actuator_test.inTestMode();`（`_throttle_armed` 在 `:417` 由 `armed && !lockdown` 得出） |

⟹ **「电调报无信号」等价于「帧真的没到」**：要么**发端停了**（驱动没起来 / FMU 重启或挂死），
要么**链路断了**（信号线 / 地线 / 连接器），要么**电调自己掉电重启**。
**不是**「PX4 在未解锁时故意不发」—— 这条可以直接从候选里划掉。

### 2.3 与 ticket 20 的接口

- 20 号票已记**一次 5.08 s 失联**：旧 mavros `06:53:47.87` `Lost connection` → `06:53:52.95` 重连
  ⟹ **那 5 秒里 DShot 大概率是停的** —— 这正是候选 A 在历史上的一个实例。
- 但**现在的 mavros（07:04 起）至今没有一次 `Lost connection`**（`/tmp/z0_mavros.log` 只有一条
  `CON: Got HEARTBEAT, connected.`）⟹ 这一小时里飞控没重启过。
  ★ 推论：**如果「滴滴响」是在飞控没重启的时候发生的，候选 A 当场被排除。**

### 2.4 ⚠️ 一处必须提醒的对齐风险

飞控实机固件是 `4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c`（ticket 20 转储里的 `FW git-hash`），
**与本地源码副本 `px4-z0 @ 5ba2e91f` 不是同一 commit**（旁证：实机参数表里没有该 fork 定义的 `DSHOT_BIDIR_EN`）。
⟹ §2.2 的源码结论按「同代参考」用，**最终判据是实机行为**（§四 T4/T5）。

## 三、候选成因与判别

| # | 候选 | 它会让电调怎么响 | 判别手段 |
|---|---|---|---|
| **A** | **飞控硬故障 / 重启**（= [ticket 20](20-fcu-hardfault-i2c1-crash-dump.md)） | **响一段（十几秒）后自己停**，与重启窗口同起同止 | `/mavros/state` 的 `connected` 掉一拍；ulog **session 数/间隙**；20 号票的 boot 计数 |
| **B** | 飞控根本没起来（卡在启动 / 反复 boot） | 一直响，不停 | 这时 mavros **连不上**。先看能不能连 |
| **C** | `dshot` 驱动没起来 / 中途退出（飞控本身正常） | 一直响 | ulog 里 `actuator_outputs` **在不在更新**（T4）；QGC MAVLink Console：`dshot status` |
| **D** | 供电 / 接地问题（ESC 由电池供电而 FC 由 USB 供电、BEC 掉压、**低电量**） | 低电量/掉压时出现，可能伴随飞控一起重启 | `/mavros/battery` 曲线 ↔ 滴滴时刻；**换满电包复现**（当前 20.19 V / 17 %） |
| **E** | 信号排线 / 连接器（MAIN1–4 ↔ 4-in-1，尤其**地线**） | 可能只响某几个 | T3 点转单电机；T5 量信号脚 |
| **F** | 协议不匹配（DShot300 vs 电调实际固件） | **从一上电就响**，不会「过一会儿」 | 若现象确实是「过一会儿」，F 基本可排除；仍可用 T6 对照 |

★ 注意 B/C/F 的共性是「**从通电起就一直响**」，而用户的原话是「**上电一段时间后**」
⟹ **现象本身的措辞就把 A 和 D 排在了最前面**（两者都能解释「一段时间之后」）。
这也是 §五 第 1、2 题最要紧的原因。

## 四、台架步骤（★ 拆桨！按 [`bench-runbook.md`](../../../docs/control/bench-runbook.md) P0）

| # | 谁做 | 做什么 | 判据 |
|---|---|---|---|
| **T1** | 人（1 分钟） | 复现并**记时间线**：上电 → 开始滴 = ? 秒；持续还是间歇；几个电调；飞控 LED 什么状态 | 喂给 §五 |
| **T2** | agent（用户在场复现时） | 滴滴当下同时跑：`date` + `ros2 topic hz /mavros/state`（应 1 Hz）+ `/mavros/battery` + `grep Lost connection /tmp/z0_mavros.log` | **`/mavros/state` 断 ⟹ A/B 胜出**；不断 ⟹ 看 T4 |
| **T3** | 人 + agent | `COM_MOT_TEST_EN=1` 已开：**在滴滴当下**用 QGC Actuator Test（或 `/mavros/actuator_control`）点转**单个**电机 | **能转 ⟹ 链路是通的**（那么「滴滴」另有含义，需重新定义现象）；**不能转 ⟹ 确实没信号** |
| **T4** | agent | 从 SD 卡拉该时段 ulog，看 **`actuator_outputs` 的时间戳是否连续**。★ `actuator_outputs` 是**输出驱动自己发的**（`src/lib/mixer_module/mixer_module.cpp:630-639` `setAndPublishActuatorOutputs`，在 `update()` 里调用 `:516-517`），且**默认就录**（官方 v1.14.3 `src/modules/logger/logged_topics.cpp:134`，在 `add_default_topics()` 内；`SDLOG_PROFILE` bit0 已置） | **连续 ⟹ 发端没停**，问题在链路/电调侧（E/D）；**断 ⟹ 发端停了**（A/B/C） |
| **T5** | 人（有工具才做） | 示波器/逻辑分析仪量 **MAIN1 信号脚对 GND**：DShot300 = 300 kbit/s 的脉冲串 | **有波形 = 发端在发**；无波形 = 发端没发。★ 这是唯一能直接看「线上到底有没有帧」的手段，T4 只是间接 |
| **T6** | 人（改参数，需重启飞控） | 对照实验：`PWM_MAIN_TIM0` 换 `-5`(DShot150) / `-3`(DShot600)，或临时退回 PWM `400`（配 `PWM_MAIN_DISx=1000`） | 响消失 ⟹ F 成立。★ **改前记原值 `-4`**，`reboot_required` |

> ⚠️ `ulog` 只能证明「**驱动在产值**」，**证明不了「帧有没有真的到电调」**（那是线上信号）。
> 判据要落在哪一侧，见 [`bench-runbook.md`](../../../docs/control/bench-runbook.md) T8 的「证据要落在哪一侧」。

★ **不要**为了「先让它别响」去动 `CBRK_BUZZER` 之类的开关 —— 那是消音，不是解决。

## 五、需要人回答（`needs-info` 的原因）

1. **从「上电」到开始滴滴，大约多久？**（推荐答案：**约 10–30 秒**，即飞控的启动窗口内）
2. **是一直响到断电，还是响一会儿自己停？**（推荐答案：**一直响到断电** ⟹ 指向 B/C）
3. **滴滴的时候飞控在不在线**（mavros / QGC 连得上吗）？LED 什么状态？
4. **四个电调一起响，还是某一个？**（推荐答案：**四个一起**）
5. **供电方式**：电调由电池供；**飞控的 5 V 从哪来**（电调 BEC / USB / 独立 BEC）？
   当时电池什么状态？（今天实测 **20.19 V / 17 %** —— 是在这种低电量下听到的吗？）
6. **这个现象是今天才注意到，还是以前也有过？** 若是以前，能不能与 ticket 20 那次 `06:53:47.87` 的失联对上？

## Comments

- **2026-09-22（agent，triage）**：`Status: needs-info` —— 现象只有一句话，**但它与 [ticket 20](20-fcu-hardfault-i2c1-crash-dump.md)
  的硬故障是同一批台架时段里的事**，而 §2.2 的源码结论已经把候选收窄到「发端停了 / 链路断了」两类。
  下一步只需要**一次带观测的复现**（T2 是 agent 可做的，T1/T3 要人在场）。
  相关：[ticket 20](20-fcu-hardfault-i2c1-crash-dump.md)（硬故障）、[ticket 01](01-hardware-prereqs.md)（硬件前置）、
  [`bench-runbook.md`](../../../docs/control/bench-runbook.md)（拆桨 + 台架顺序）。
