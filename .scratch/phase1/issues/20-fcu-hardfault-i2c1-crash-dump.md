# 20: 飞控硬故障（`wq:I2C1`）—— SD 卡上的 crash dump 让 PX4 **拒绝解锁**

Status: needs-triage
Type: task

**为什么单开一张：** 2026-09-22 台架上要试「解锁上升沿切原点」（[ticket 13](13-mavros-bench-vision-pose.md) 的验收项 /
[ticket 14](14-mavros-from-source.md) 的最后一条），**解锁被飞控自己拦下**：

> `crash dumps present on SD, vehicle needs service`

这是 PX4 的**保护性前置检查**（不是我们这套栈的 bug）。顺着它从 SD 卡上取到了 crash dump，
内容是 **`wq:I2C1` 任务的硬故障** —— 这是**安全相关**的新事实（飞行中硬故障 = 飞控重启 = 掉机），
与 [ticket 11](11-firmware-version-conflict.md)（固件来源）、[ticket 16](16-ekf2-position-failure-carrier.md)（失效载体）
都不同源，必须独立跟踪。

## 一、第一手证据：crash dump 原文（2026-09-22 取）

取法**只读**：`/mavros/ftp/list` 列目录 → 发现 SD 卡**根目录**（不是 `log/` 下）有
`fault_2000_01_01_00_00_01.log`（16919 字节）；用 `/mavros/ftp/open` + `/mavros/ftp/read` 分段读完
（脚本 `/tmp/z0_ftp_get.py`，**本机存档** `~/.ros/evidence/z0_fault_dump_2026-09-22.log`，16919/16919 字节）。

```
[hardfault_log] -- 2000-01-01-00:00:01 Begin Fault Log --
System fault Occurred on: 2000-01-01-00:00:01
 Type:Hard Fault in file:armv7-m/arm_hardfault.c at line: 173 running task: wq:I2C1
 FW git-hash: 4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c
 Build datetime: Nov 19 2024 11:10:49
 Build url: localhost
 r0:0x24000c6c r1:0x24006f64  r2:0x00000030  r3:0x00000000 ...
 xpsr:0x61000000 basepri:0x00000080 control:0x00000004
 Fault status registers: from NVIC
 cfsr:0x00008200 hfsr:0x40000000  dfsr:0x00000000  mmfsr:0x00000030  bfsr:0x00000030 afsr:0x00000000 abfsr:0x00000000
Invalid Stack! (Corrupted TCB)  Stack base:  0x240010c0 Stack size:  0x00000300
Invalid Stack! (Corrupted TCB)  Stack base:  0x3800eb28 Stack size:  0x00000908
[hardfault_log] -- 2000-01-01-00:00:01 END Fault Log --
```

**逐条读数**（agent 解读，标注与原文的差别）：

| 字段 | 值 | 含义 |
|---|---|---|
| `running task` | **`wq:I2C1`** | I2C1 的**工作队列任务**上出的故障（★ 不是主任务，也不是某条驱动线程） |
| 故障类型 | `Hard Fault`（`armv7-m/arm_hardfault.c:173`） | 由下层故障**升级**而来的硬故障 |
| `cfsr` | `0x00008200` | `BFSR = 0x82` ⟹ `PRECISERR`（精确数据总线错误）+ `BFARVALID`（地址寄存器有效） |
| `hfsr` | `0x40000000` | `FORCED` ⟹ 由可配置故障升级 |
| `mmfsr` / `bfsr` | `0x30` / `0x30` | `STKERR \| UNSTKERR` ⟹ **异常进出时的压栈/出栈失败** |
| 栈 | `Invalid Stack! (Corrupted TCB)` ×2 | 任务栈或其 TCB 已损坏 |

⟹ **特征吻合"某个任务/工作队列的栈被写坏（溢出或踩踏），异常进出时压栈失败，升级成硬故障"**。
★ 这只是**特征**读法：**根因未定**（栈溢出？I2C 驱动的野指针/越界？总线错误？）。

**固件身份**：`4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c` **与仓库里记录在案的 z0 固件一致**
（[ticket 11](11-firmware-version-conflict.md) §：`Minderring/PX4-Autopilot` @ `4a0e65f2`；
[`params-analysis.md`](../../../docs/reference/params-analysis.md) §头部 `Git Revision: 4a0e65f233000000`；
[`params-changes.md`](../../../docs/reference/params-changes.md)）⟹ **不是刷错的固件**，是我们要飞的那一版。

## 二、它拦住了什么（操作层面）

- **解锁被拦**：PX4 在 SD 卡上看到 crash dump 就拒绝解锁（"vehicle needs service"）⟹
  [ticket 13](13-mavros-bench-vision-pose.md) / [ticket 14](14-mavros-from-source.md) 的
  **「解锁上升沿切原点」（阶段 B）当前无法做**，与动捕断流是两个独立的阻塞。
- **清障办法**（不改代码、不动逻辑）：把 `/fs/microsd/fault_2000_01_01_00_00_01.log` 删掉
  —— ① QGC → MAVLink Console：`rm /fs/microsd/fault_2000_01_01_00_00_01.log`；
  ② 或 mavros：`/mavros/ftp/remove`（服务存在，2026-09-22 已确认）。
  ★ **删之前必须先留档**（已做：`~/.ros/evidence/z0_fault_dump_2026-09-22.log`）。
  ★ 删除是**飞控上的写操作** ⟹ 由操作者决定，agent 不擅自做。

## 三、时间：**不能从文件名定日期**（★ 别过度推断）

- 文件名/内文的时间都是 `2000-01-01-00:00:01` ⟹ **飞控 RTC 无效**（无 GPS/电池 RTC 对时）。
- **与今天一次失联时间吻合**：旧 mavros 日志 `06:53:47.87` `CON: Lost connection, HEARTBEAT timed out.`
  → `06:53:52.95` 重连（5.08 s）；`control` 日志同一秒 `degrade=mode_lost` —— 硬故障自动重启的形状与它一致。
- ⚠️ **但"卡上只有一个 fault 文件"不能推出"只故障过一次"**：RTC 无效 ⟹ **每次故障都会写成同一个文件名**
  （`fault_2000_01_01_00_00_01.log`），后来的会**覆盖**先前的 ⟹ 数量信息为 0。
- 要**定死**发生时刻：拉 `sess252`/`sess253`（该 boot 的 ulog）对齐，或 QGC 里看 boot 计数
  —— **本张还没做**。

## 四、危害与纪律

- **危害**：飞行中发生 = **飞控自己重启**，OFFBOARD/位置控制全部丢失 ⟹ 按现有失效保护多半掉机。
  今天**桨拆着、未起飞** ⟹ 无人员/设备风险，但**装桨自由飞之前必须弄清**。
- **纪律**：本张**不**建议任何"绕过解锁检查"的做法（那是 PX4 用来保护人和机的判据）。
  要么清掉故障记录并确认原因，要么先查清 I2C1 上的东西。

## 五、下一步（都需要第一手证据）

1. **I2C1 上挂了什么、`wq:I2C1` 在跑谁的驱动** —— 查板级配置（NxtPX4v2：双 BMI088 + SPL06，
   见 [`open-source-survey.md`](../../../docs/reference/open-source-survey.md) §固件 与
   [`phase1-implementation.md`](../../../docs/control/phase1-implementation.md) 的固件来源段），
   确认 I2C1 上是否有外部罗盘 / 气压计 / 其他外设 ⟹ **本机没有 PX4 源码，需要 vendor 树或上游 tag**。
2. **复现条件**：是否某个动作（I2C 事务、某次参数写入、某种供电状态）触发 ——
   台架上可留意"故障是否伴随某个操作"；PX4 侧可看 `dmesg` / `listener` 输出（QGC MAVLink Console）。
3. **定时刻**：拉 ulog 对齐（见 §三）。
4. 清障后**先只做解锁/上锁循环**（不起飞、拆桨）确认解锁恢复正常，再回到
   [ticket 13](13-mavros-bench-vision-pose.md) 的阶段 B。

## Comments

- **2026-09-22（agent，triage）**：`Status: needs-triage` —— 事实与危害已写清（转储原文 + 存档路径 + 时间的不确定性），
  **根因与 I2C1 归属都未知**。三件事待人裁定：① 是否先清障以解锁（并恢复阶段 B 的台架）；
  ② 要不要在装桨前做 I2C1 侧的排查；③ 本张要不要升级为"装桨前必须解决"的门禁。
