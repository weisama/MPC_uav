# 20: SD 卡上的 crash dump（`wq:I2C1` 硬故障）拦了解锁 —— 转储已存档；★ 那些"重启"是**人工**的

Status: needs-info
Type: task

**为什么单开一张：** 2026-09-22 台架上要试「解锁上升沿切原点」（[ticket 13](13-mavros-bench-vision-pose.md) 的验收项 /
[ticket 14](14-mavros-from-source.md) 的最后一条），**解锁被飞控自己拦下**：

> `crash dumps present on SD, vehicle needs service`

顺着这条从 SD 卡上取到了那份 crash dump：内容是 **`wq:I2C1` 任务的硬故障**。
这是**安全相关**的事实（硬故障本身意味着栈被写坏一类的严重问题），与
[ticket 11](11-firmware-version-conflict.md)（固件来源）、[ticket 16](16-ekf2-position-failure-carrier.md)（失效载体）都不同源。

> ★★ **2026-09-22 重要更正（操作者答复）**：本张初稿曾把这天观察到的 **8 次开机**判为"飞控自己反复重启"
> 并据此升级为"飞行中必掉机、装桨前必须解决"。**操作者明确答复：那些重启是他自己重启的。**
> ⟹ 「反复硬故障重启」这个结论**撤回**，见 §二。转储本身、以及它拦住解锁这件事**不变**。

## 一、第一手证据：crash dump 原文（2026-09-22 取，**已删，本机存档**）

取法**只读**：`/mavros/ftp/list` 列目录 → SD 卡**根目录**（不是 `log/` 下）有
`fault_2000_01_01_00_00_01.log`（16919 字节）；用 `/mavros/ftp/open` + `/mavros/ftp/read` 分段读完。
★ 该文件已于同日**按用户明确授权删除**（清障），内容**先留档**在
`~/.ros/evidence/z0_fault_dump_2026-09-22.log`（16919/16919 字节）。脚本：`/tmp/z0_ftp_get.py`（取）、
`/tmp/z0_ftp_rm.py`（删，带删前删后目录对照）、`/tmp/z0_ftp_head.py`（取头部/区间）。

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

## 二、★★ 更正：那些"重启"是**人**重启的（不是飞控自己）

### 2.1 我当时的观察（原始数据，仍然有效）

`logger` 每次开机都会在 SD 卡上建**新的 session 目录**并把路径当 statustext 发出来，mavros 原样打印
（`~/.ros/log/mavros_node_*.log`）：

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

**"新 session = 一次开机"这一步的取证是扎实的**（值得留用）：
ULog 文件头第 8–15 字节 = **开始记录时的自开机微秒数**；
`sess253/254/255` 分别是 **0.780 / 0.756 / 0.780 s** ⟹ 文件都是**开机后不到 1 秒**创建的
（若是"logger 中途重开"，这里会是当时的 uptime，例如 600 s）。
旁证：这些 ulog 正文开头就是**只在启动时出现**的消息（`INFO [param] selected parameter backup file …`、
`INFO [mavlink] mode: Normal … on /dev/ttyS1`、`INFO [logger] logger started (mode=all)`）。
复现：`python3 /tmp/z0_ftp_head.py /fs/microsd/log/sess255/log100.ulg 5000 | head -c 16`。

### 2.2 我错在哪（★ 记下来，这个方法陷阱以后别再踩）

我从"8 次开机、间隔 10.7–15.3 min 很规则"推出"飞控自己在重启"，**跳过了唯一能定性的那一问：
"这些开机是谁触发的？"** —— 那 8 次开机**可能**是同一个人的手，而我**没有**任何证据把它排除。

**当时能证伪我的线索其实就在表里**（我看到了，但解释反了）：一天里 8 次里**只有 1 次**
（`06:53:47`）mavros 报了 `Lost connection`。我把它解释成"其余 7 次重启太快、没越过阈值"，
**但更简单的解释是"那 7 次根本不是重启，或者重启的停机时间各有长短"**；
而正确的动作是**先问人**（一句"这些重启是你做的吗"就能定案）。

**顺带把阈值这条机制记准**（已回源码核实，别再写 5 s）：
mavros 的判据是 `conn_timeout`（**默认 10.0 s**，`mavros/src/plugins/sys_status.cpp:512-519`）建的
**周期定时器**，**每收到一条 HEARTBEAT 就 reset**（同文件 `958-959`），到点跑 `timeout_cb()`
→ `update_connection_status(false)`（同文件 `1311-1313`）并打印 `CON: Lost connection, HEARTBEAT timed out.`
（`mavros/src/lib/mavros_uas.cpp:462`）。⟹ **WARN 的时间戳 = 最后一条心跳 + 10 s**：

- `06:53:47.87` WARN ⟹ 最后心跳 ≈ `06:53:37.87` → `06:53:52.95` 重连 ⟹ 静默 **≈15.1 s**（这次确有重启，`control` 侧 `mode_lost`）；
- `06:55:14.95` WARN ⟹ 最后心跳 ≈ `06:55:04.95` → `06:55:26.4` 重连 ⟹ 静默 **≈21.5 s**，且**不带**新 session
  （`mavros_router: link[1000] reconnected`）⟹ 那是**串口链路**自身抖动，与开机是两回事；
- 其余 7 次开机**没有** WARN ⟹ 那些开机的静默 **< 10 s**（与"软件重启（QGC 的 reboot 命令）"的形状一致，
  与"拔电重上电"的长时间静默不一致 —— 这也是事后能自查的一条判据）。

### 2.3 更正后，哪些结论**不动**、哪些**撤回**

| | 结论 | 状态 |
|---|---|---|
| a | SD 卡根目录有 crash dump ⟹ PX4 拒绝解锁（`crash dumps present on SD`） | ✅ **不动**（QGC 原话 + 飞控 statustext 都有） |
| b | 转储内容 = `wq:I2C1` 硬故障，固件 hash 与在飞版本一致 | ✅ **不动**（原文见 §一） |
| c | 转储**每次故障都写同名文件**（RTC 无效）⟹ "只有一个 fault 文件"推不出故障次数 | ✅ **不动**（这条是防止过度推断，正是它现在救了我） |
| d | **转储是什么时候写下的** | ❓ **未知**（见 §三） |
| e | **飞控自己在反复重启（约每 11 min 一次）** | ❌ **撤回**（操作者答复：是他自己重启的） |
| f | 于是"飞行中必掉机、装桨前必须解决" | ❌ **降级**：目前**没有**复发证据，见 §三 |

## 三、还剩什么真问题（更正后）

1. **这份转储是哪个时刻的？** 未知。可用的线索只有一条时间窗：
   06:27:37、06:28:32、06:28:34 三次 `Preflight Fail: Missing FMU SD Card`（**卡当时没挂上**，
   所以那几次解锁检查报的是"缺卡"而**不是**"有 crash dump"），而第一次报
   `Crash dumps present on SD` 是在 `06:31:26` 那次开机之后。
   ⟹ 转储**可能**是那之前不久写的（也无从排除是更早某天留下的）。**没有任何证据**能定死。
2. **它还会不会再犯？** 判定实验正在跑：清障（删掉 fault 文件）之后，若再犯，同名文件会**重新出现**。
   `/tmp/z0_watch.py`（**只读**：订阅 `/mavros/state` + `/mavros/statustext/recv`，每 60 s 列一次
   `/fs/microsd` 与 `/fs/microsd/log`），日志 `/tmp/z0_watch.log`，起点 **07:58:57**（session 数 159，最大 `sess258`）。
   **到 08:10 为止：`fault 文件=无`，也没有新 session** —— 清障后 **12 分钟零复发**，
   且这段时间**没有任何人重启**（操作者已停手）。时间窗还太短，但方向是"不复发"。
3. **`wq:I2C1` 上跑的是谁** —— 要真排查根因时才有用（参考树：`/home/chen/refs/px4-z0` 等，见 §四）。

## 四、它拦住了什么（操作层面）

- **解锁被拦**：PX4 在 SD 卡上看到 crash dump 就拒绝解锁（"vehicle needs service"）⟹
  [ticket 13](13-mavros-bench-vision-pose.md) / [ticket 14](14-mavros-from-source.md) 的
  **「解锁上升沿切原点」（阶段 B）当时无法做**，与动捕断流是两个独立的阻塞。
- **清障已完成**（2026-09-22，用户授权，agent 执行）：`/mavros/ftp/remove` →
  `success=True r_errno=0`，删后 `/fs/microsd` = `APM, dataman, log, parameters_backup.bson`（已无 fault 文件）；删前留档见 §一。
- 清障只解掉"解锁被拦"这一层。**转储的来历**仍未定（§三），但已**不再是**"装桨前必须解决"的门禁。

## 五、危害与纪律

- **危害（更正后）**：目前**没有**证据表明它会复发；若真复发（清障后又长出一份新转储），
  那才回到"飞行中硬故障 = 飞控重启 = 掉机"这一档，届时必须解决。
- **纪律**：本张**不**建议任何"绕过解锁检查"的做法（那是 PX4 用来保护人和机的判据）。
  ★ 另外记一条**方法纪律**：本张最大的错不是技术细节，是**把"规则的时间间隔"当成了"机器在自作主张"** ——
  观测到周期性事件时，**先问操作者**，再谈机理。

## 六、下一步

1. **继续看观察器**（§三）：零复发的时间越长，"不复发"越可信；若重现 ⟹ 立刻取新转储比对。
2. 真要做根因排查时（**不急**）：`HKUST_NXT_DUAL` / `micoair743-v1.14.3` 板级配置里 I2C1 挂了谁、
   `wq:I2C1` 在跑谁的驱动。★ 本机**有** PX4 参考树：`/home/chen/refs/px4-z0`（ticket 21 用的就是它，`5ba2e91f`）、
   `px4-v1.14.3`、`px4-v1.15.0`、`px4-v1.16.0`。
3. 供电：操作者答复为**台架稳压电源** ⟹ 掉电这条线优先级低（更正后也不再需要它来解释周期）。
4. **被本张推翻的前提要回头改**：[ticket 21](21-esc-no-signal-beeping.md) 曾把"电调滴滴响"解释成
   "重启窗口里 DShot 停发"，其触发前提（飞控自己在重启）**已不成立** ⟹ 已在 21 号票挂 Comment 更正。

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
- **2026-09-22（agent，★★ 更正 —— 操作者答复"我自己重启的"）**：`Status: needs-triage → needs-info`。
  - **撤回**"飞控自己在反复重启（~11 min 一次）"这个结论（§2.3 表 e/f）：那 8 次开机是**操作者手动重启**的。
    ⟹ 本张**不再**是"装桨前必须解决"的门禁；飞行风险一档**暂时撤销**（除非复发，§五）。
  - **不动**的：转储存在且拦了解锁、转储内容 = `wq:I2C1` 硬故障、同名覆盖 ⟹ 推不出故障次数（§2.3 表 a/b/c）。
  - **未定**的：转储写下的时刻（§三 第 1 条，只有一条时间窗线索）。
  - ★ **方法教训已写进 §2.2/§五**：把"规则的时间间隔"读成"机器自作主张"是本张最大的错；
    当时表里"8 次只有 1 次报断连"这条**本可证伪我**的线索，被我反向解释了。观测到周期性事件先问人。
  - 更正后的实测：清障后 **12 分钟零复发**（08:10，`/tmp/z0_watch.log`），且期间无人重启。
