# 20: ★★ 飞控**反复硬故障重启**（`wq:I2C1`）—— 台架上一小时 8 次；crash dump 让 PX4 拒绝解锁

Status: needs-triage
Type: task

**为什么单开一张：** 2026-09-22 台架上要试「解锁上升沿切原点」（[ticket 13](13-mavros-bench-vision-pose.md) 的验收项 /
[ticket 14](14-mavros-from-source.md) 的最后一条），**解锁被飞控自己拦下**：

> `crash dumps present on SD, vehicle needs service`

顺着这条查下去拿到两件事：① 卡上那份 crash dump 的内容是 **`wq:I2C1` 任务的硬故障**；
② 更要紧的 —— **飞控在台架上自己反复重启，今天一小时里 8 次（每 ~11 分钟一次）**。
这是**飞行安全级**的发现（飞行中重启 = OFFBOARD 全丢 = 掉机），与
[ticket 11](11-firmware-version-conflict.md)（固件来源）、[ticket 16](16-ekf2-position-failure-carrier.md)（失效载体）都不同源。

## 一、第一手证据 (a)：crash dump 原文（2026-09-22 取，**已删，本机存档**）

取法**只读**：`/mavros/ftp/list` 列目录 → SD 卡**根目录**（不是 `log/` 下）有
`fault_2000_01_01_00_00_01.log`（16919 字节）；用 `/mavros/ftp/open` + `/mavros/ftp/read` 分段读完。
★ 该文件已于同日**按用户明确授权删除**（清障），内容**先留档**在
`~/.ros/evidence/z0_fault_dump_2026-09-22.log`（16919/16919 字节）。脚本：`/tmp/z0_ftp_get.py`（取）、
`/tmp/z0_ftp_rm.py`（删，带删前删后目录对照）。

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

**逐条读数**（agent 解读，与原文的差别已标）：

| 字段 | 值 | 含义 |
|---|---|---|
| `running task` | **`wq:I2C1`** | I2C1 的**工作队列任务**上出的故障（★ 不是主任务，也不是某条驱动线程） |
| 故障类型 | `Hard Fault`（`armv7-m/arm_hardfault.c:173`） | 由下层故障**升级**而来的硬故障 |
| `cfsr` | `0x00008200` | `BFSR = 0x82` ⟹ `PRECISERR`（精确数据总线错误）+ `BFARVALID`（BFAR 有效） |
| `hfsr` | `0x40000000` | `FORCED` ⟹ 由可配置故障升级 |
| `mmfsr` / `bfsr` | `0x30` / `0x30` | `STKERR \| UNSTKERR` ⟹ **异常进出时的压栈/出栈失败** |
| 栈 | `Invalid Stack! (Corrupted TCB)` ×2 | 任务栈或其 TCB 已损坏 |

⟹ **特征吻合"某任务/工作队列的栈被写坏（溢出或踩踏），异常进出时压栈失败，升级成硬故障"**。
★ 只是**特征**读法：**根因未定**（栈溢出？I2C 驱动野指针/越界？总线错误？）。

**固件身份**：`4a0e65f2331b23b9c8352656e9ccb6edeaea5d6c` **与仓库记录在案的 z0 固件一致**
（[ticket 11](11-firmware-version-conflict.md)；[`params-analysis.md`](../../../docs/reference/params-analysis.md) 头部
`Git Revision: 4a0e65f233000000`；[`params-changes.md`](../../../docs/reference/params-changes.md)），
ulog 头部另给 `ver_sw_branch = micoair743-v1.14.3`、`ver_hw = HKUST_NXT_DUAL` ⟹ **不是刷错的固件**。

## 二、第一手证据 (b)：★★ 飞控在**反复重启**，实测 8 次

### 触发物：mavros 打出的新 session

每次飞控重启，`logger` 会在 SD 卡上建一个**新的 session 目录**并把路径当 statustext 发出来，
mavros 原样打印（`~/.ros/log/mavros_node_*.log`，两个进程的日志合起来）：

| # | 本机时刻 | 新 session | 距上次 |
|---|---|---|---|
| 1 | 09-22 06:31:26 | `sess251` | — |
| 2 | 06:42:24 | `sess252` | 11.0 min |
| 3 | 06:53:53 | `sess253` | 11.5 min |
| 4 | 07:04:36 | `sess254` | 10.7 min |
| 5 | 07:15:19 | `sess255` | 10.7 min |
| 6 | 07:30:35 | `sess256` | 15.3 min |
| 7 | 07:41:23 | `sess257` | 10.8 min |
| 8 | 07:54:17 | `sess258` | 12.9 min |

### 关键：新 session = **一次开机**，不是"logger 中途重开"（★ 这条把定性钉死了）

ULog 文件头第 8–15 字节 = **开始记录时的自开机微秒数**。从 SD 卡取下的三个 ulog：

| ulog | 记录开始时自开机 |
|---|---|
| `sess253/log100.ulg` | **0.780 s** |
| `sess254/log100.ulg` | **0.756 s** |
| `sess255/log100.ulg` | **0.780 s** |

⟹ 每个 session 的文件都是**开机后不到 1 秒**创建的 ⟹ **每个 `sessNNN` 就是一次开机**。
（若是"logger 中途重开"，这里会是当时的 uptime，例如 600 s。）
复现：`python3 /tmp/z0_ftp_head.py /fs/microsd/log/sess255/log100.ulg 5000 | head -c 16` ⟹
`ULog\x01\x125` + ver=1 + `uint64 LE = 780000`。
旁证：这些 ulog 正文开头就是**只在启动时出现**的消息 ——
`INFO [param] selected parameter backup file /fs/microsd/parameters_backup.bson`、
`INFO [mavlink] mode: Normal, data rate: 1200 B/s on /dev/ttyS1 @ 921600B`、
`INFO [mavlink] mode: Onboard … on /dev/ttyS3`、`INFO [logger] logger started (mode=all)`。

### ★ 不要拿"mavros 没报断连"当反证

mavros 要**连续 5 s** 收不到 `HEARTBEAT` 才打 `Lost connection`。8 次重启里只有
**06:53:47 → 06:53:52（5.08 s）** 越过了这道阈值（那次确实报了 `Lost connection`，`control` 侧同秒 `mode_lost`），
其余 7 次重启的停机 **< 5 s** ⟹ mavros 一声不响。⟹ "没有断连告警" ≠ "没有重启"。
（另有一次 06:55:14→06:55:26 的 11.5 s 断连**不带**新 session（`mavros_router: link[1000] reconnected`），
那是**串口链路**自身的抖动，与重启是两回事，别混为一谈。）

### ★ 为什么每次开机都喊"SD 上有 crash dump"

每次新 session 之后 **0.1 s**，飞控就报 `Preflight Fail: Crash dumps present on SD, vehicle needs service`
（开机时 PX4 自己会跑一遍前置检查），1.5 s 后再报一次。⟹ 卡上**一直有** crash dump，
且每次开机它都在 —— 与"重启前刚发生过硬故障、故障转储落在卡上"完全一致。

### ⚠️ 一处必须说清的**不**确定性

文件名/内文时间都是 `2000-01-01-00:00:01` ⟹ **飞控 RTC 无效**。RTC 无效的后果是：
**每次故障都写同一个文件名**（`fault_2000_01_01_00_00_01.log`），后来的**覆盖**先前的
⟹ **"卡上只有一个 fault 文件"完全不能用来推断"只故障过一次"**（同名覆盖，个数信息为 0）。
本张**也没有**任何一条证据能证明"这 8 次重启每一次都由硬故障引起"——
这恰好是下面的观察器要问的问题。

## 三、正在跑的判定实验（只读，2026-09-22 08:00 起）

**逻辑**：清障（删掉 fault 文件）之后，若故障是**复发性**的，下一次必然**再写一个**
`fault_2000_01_01_00_00_01.log`（同名，所以只能靠"重新出现"判断），且时间上应与下一次重启对齐。

**做法**：`/tmp/z0_watch.py`（**只读**：订阅 `/mavros/state` + `/mavros/statustext/recv`，
每 60 s 列一次 `/fs/microsd` 与 `/fs/microsd/log`），日志 `/tmp/z0_watch.log`。
启动时刻本机 **07:58:57**，起始 session 数 **159（最大 `sess258`）**，当时 `fault 文件=无`。

⟹ 若 ~11 min 后（预计 **08:05–08:10**）出现：① 新 `sessNNN`；② `fault_*` 文件重新出现 ——
则**复发性硬故障**成立，且能把故障定到分钟；再把这个新 dump 取下来，看是不是又是 `wq:I2C1`。

## 四、它拦住了什么（操作层面）

- **解锁被拦**：PX4 在 SD 卡上看到 crash dump 就拒绝解锁（"vehicle needs service"）⟹
  [ticket 13](13-mavros-bench-vision-pose.md) / [ticket 14](14-mavros-from-source.md) 的
  **「解锁上升沿切原点」（阶段 B）当前无法做**，与动捕断流是两个独立的阻塞。
- **清障已完成**（2026-09-22，用户授权，agent 执行）：`/mavros/ftp/remove` →
  `success=True r_errno=0`，删后 `/fs/microsd` = `APM, dataman, log, parameters_backup.bson`（已无 fault 文件）。
  删前留档见 §一。★ 若复发性故障成立，这个文件会**再长出来** —— 那是信息，不是回到原点。
- 清障**只**解掉"解锁被拦"这一层；**根因（反复重启）没解**，见 §五。

## 五、危害与纪律

- **危害**：本机实测**约每 11 分钟一次**（最长间隔 15.3 min，最短 10.7 min）。放到飞行中
  = 飞控自己重启 ⟹ OFFBOARD 信号、位置/姿态控制全丢 ⟹ 按现有失效保护多半掉机。
  **装桨/自由飞之前必须先弄清。**
- **纪律**：本张**不**建议任何"绕过解锁检查"的做法（那是 PX4 用来保护人和机的判据）。

## 六、下一步（都需要第一手证据）

1. **看观察器**（§三）：故障是否复发、能否定到分钟。
2. **I2C1 上挂了什么、`wq:I2C1` 在跑谁的驱动** —— 查 `HKUST_NXT_DUAL` / `micoair743-v1.14.3` 的板级配置
   ⟹ **本机没有 PX4 源码**，需要 vendor 树或上游 tag（`Minderring/PX4-Autopilot @ 4a0e65f2`）。
3. **供电**：~11 min 的周期性也要排除**掉电/欠压**（台架电源、电池、接触）——
   `Preflight Fail: Missing FMU SD Card` 在 06:27:37、06:28:32、06:28:34 出现过三次，
   那是**卡没挂上**时的检查结果，与"供电/接触"这条线有关，别漏。
4. **全量 ulog 对齐**：`sess251…258` 每个 ~20–24 MB（MAVLink FTP 实测 ~46 KB/s，取一个约 9 min）。
   取一个"正常间隔"的 session 与"异常间隔"的做差分，比只在这里推理强。
5. 结论出来之前：**台架可以做只读/不动飞机的部分**；**装桨免谈**。

## Comments

- **2026-09-22（agent，triage）**：`Status: needs-triage`。事实已写清、且**方法可复现**（文件头读 uptime 那一步是定性的关键）。
  待人裁定的三件事：① 是否现在就投力气查根因（I2C1 器件 / 固件树 / 供电）；
  ② 本张要不要升为"**装桨前必须解决**"的门禁；③ 台架阶段 B 是否等这条线有结论后再做。
- **2026-09-22（agent，交叉复核 —— 症状侧新票 [21](21-esc-no-signal-beeping.md)）**：
  - 用户报告的现象「**电调上电一段时间后没收到信号滴滴响**」很可能是本张硬故障的**肉眼症状**
    （重启窗口里 DShot 停发 ⟹ 电调报警）⟹ **症状那一侧单开为 [ticket 21](21-esc-no-signal-beeping.md)**，
    由它负责定义现象并做判别实验（墙上时刻 ↔ `sessNNN` 对齐 / QGC `dshot status` / ulog）。
  - ★ **一处机制更正（本张 §二）**：文中「mavros 要**连续 5 s** 收不到 HEARTBEAT 才打 `Lost connection`」与源码不符。
    实为 `conn_timeout`（**默认 10.0 s**，`mavros/src/plugins/sys_status.cpp:512`）到点跑一次 `timeout_cb()`
    （`:1311-1313`），**无条件**置 `connected=false` 并打印（`mavros/src/lib/mavros_uas.cpp:462`）。
    ——**结论方向不变**（短重启会被整个漏掉、日志不能当反证），但阈值是「**10 s 周期定时器**」而不是「5 s 静默判据」，
    判读飞行日志/算停机时长时别按 5 s 算。（该 mavros 源码树：`/home/chen/mavros_ws/src/mavros`）
