# 13: 台架 —— 装 mavros(+extras) 并完成 QGC 侧的 vision_pose 验收（[ticket 04](04-mocap-relative-pose.md) 的残留项）

Status: done
Type: task

**What to build:** 把 [ticket 04](04-mocap-relative-pose.md) 剩下的**两条验收**做完 —— 它们的共同点是
**必须有人**（给飞控上电、开 QGC、把飞机平移 0.5 m / 升高 0.3 m），agent 在无飞控条件下做不了：

| # | 04 的验收项 | 现在缺什么 |
|---|---|---|
| ① | 未解锁时 QGC 的 `LOCAL_POSITION_NED` **已稳定**，且有 `VISION_POSITION_ESTIMATE` | 飞控上电 + `mavros` + QGC |
| ⑥ | **坐标系确认实验**：+x 平移 0.5 m → QGC 的 `y` 变化 0.5；升高 0.3 m → QGC 显示**向上** | 同上 + 有人动手挪飞机 |

**★ 04 已经把"节点侧那一半"验完了**（假输入：+x 平移 0.5 m ⟹ 输出 Δx = 0.5000 m，升高 0.3 m ⟹ Δz = +0.3000 m）。
**本张只验 mavros 那一段**（mavros 的 ENU→NED / FLU→FRD 是否真的按 §3.5 的源码结论转）。
⟹ 若第 ⑥ 条**不成立**（QGC 里变的是 `x` 而不是 `y`），**问题在 mavros 侧或参数侧，不在 `mocap` 节点**。

**为什么单独一张：**
1. **它是 [ticket 05](05-control-offboard-skeleton.md) 的前置** —— `control` 节点整条都建在 mavros 上；
2. 装 `mavros` 是**系统改动**（要走 apt、要用户同意，见 [ticket 03](03-vendor-vrpn-listener.md) 的先例）；
3. 「飞控未上电」是**物理状态**，不是代码缺陷 —— 与 [ticket 12](12-z0-mocap-stream-check.md) 同源。

## ⚠️ 三个坑（开做之前先看）

1. ☠️ **「apt 装 mavros」这条路已实测不通**（2026-09-21）：humble 的 `ros-humble-mavros` /
   `mavros-extras` **arm64 二进制在十个可达镜像里全都没有**（同仓库的 `mavros_msgs` 反而有）。
   ⟹ **改走源码构建**，全部证据与步骤见 **[ticket 14](14-mavros-from-source.md)**（用户 2026-09-21 裁定）。
   ★ 另注：`vision_pose` 插件在 `mavros-extras` 里（§3.5 引的源码路径就是
   `mavros_extras/src/plugins/vision_pose_estimate.cpp`），**构建时不能只建 `mavros`**。
2. ⚠️ **喂视觉数据会立刻改变 EKF2 的行为**。`z0.params` 里 `EKF2_EVP_NOISE = 0.06096`，
   而用户笔记说"**0.1 都过小、会发散**"（`phase1-implementation.md` §6）。
   ⟹ **台架上第一次接 vision_pose 时，建议先把 `EKF2_EVP_NOISE` 抬到 1.0**（属参数核对清单，不在本张改）。
3. ✅ **串口已定案（2026-09-21 实测）：`/dev/ttyTHS1` @ 921600** ——
   `mavlink_probe.py --scan` 报 `sysid=1, type=2`（飞控在此）；`ttyTHS2`/`ttyAMA0` 各波特率**一个字节都没有**。
   `dialout` 组也已修好（`dialout:x:20:chen`，当前会话即生效，不用重登）。
   ⟹ `mocap-setup.md` §10 第 2 项可结案（这一步本张已就地做掉，不必再等）。
4. ✅ **`/mavros/state` 的发布频率已定案（2026-09-21 实测）= `1.00 Hz`**（30 条 / 30 s，
   间隔 min `0.998` / mean `1.000` / max `1.002` s）—— 见下方 §实测。
   **推论**：`mocap` 节点检解锁上升沿**最多晚 1 s** ⟹ 阶段 B 的"起飞前静止帧"（60 帧 ≈ 0.3 s）
   有在**爬升中被采到**的风险；代价是起飞前原点带一个同量的位置偏置（EKF2 会当成真实运动）。
   **缓解（两条，都已落地）**：① **纪律**——解锁后到起飞之间留 ≥1.5 s（`phase1-implementation.md` §2 已写）；
   ② **检测**——若真被采到，`switch_jump` 会 > `jump_warn_m` 并打 WARN，日志里看得见。
   ⟹ 结论已写回 [ticket 04](04-mocap-relative-pose.md) §七 第 4 条。

**Blocked by:** 无（04 的代码已就绪）；**被它阻塞的是** [ticket 05](05-control-offboard-skeleton.md)

## 步骤（在 z0 的 Jetson `orin-nx` 上）

```bash
# 1) mavros —— ★ 不走 apt（二进制不可得），按 ticket 14 源码构建
#    产出的 ~/mavros_ws/install 必须与本仓库的 install 一起 source

# 2) 串口 ✅ 已定案：/dev/ttyTHS1 @ 921600（2026-09-21 实测，见上方"三个坑"第 3 条）

# 3) 动捕链路先通（顺序不能反：先确认输入存在，再谈处理对不对）
source /opt/ros/humble/setup.bash && source ~/ros_ws/install/setup.bash
ros2 launch vrpn_listener sync_entity_state.launch
ros2 topic list | grep vrpn        # ★ 现场核对刚体名，与 mocap.yaml 的 body_name 对齐
ros2 topic hz /vrpn/<刚体>/pose    # 应 ~200 Hz

# 4) mavros 起来（串口按第 2 步的结果填）
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600

# 5) mocap 节点起来（★ 起之前把飞机摆好、静止；起到解锁前别动它）
ros2 launch mocap mocap.launch.py
ros2 topic hz /mavros/state        # ★ 定案：它是不是 1 Hz？见下方"三个坑"第 4 条

# 5b) 试一次"解锁上升沿切原点"：在 QGC 里解锁（不用起飞），看 mocap 节点日志
#     ★ 阶段 B：…切换发生在此刻…   p_arm / p_boot / 归零的 yaw / 切换瞬间位置跳变
#     跳变应 ≈ 0（静止摆放的前提）；若 > 0.05 m 会同时打 WARN

# 6) QGC 侧：未解锁，看 MAVLink Inspector
#    VISION_POSITION_ESTIMATE  有数且不跳
#    LOCAL_POSITION_NED        稳定（不用解锁、不用飞）
```

## 验收项

- [x] mavros（含 `mavros_extras`）可用，`vision_pose` 链路通
      （判据：`/mavros/vision_pose/pose` 上的消息被 mavros 收下 —— 节点侧日志无异常、QGC 有数）
      —— ★ 装法已改为**源码构建**，见 [ticket 14](14-mavros-from-source.md)。
      ✅ **2026-09-21 已证**：订阅者 = `mavros/vision_pose` 插件（QoS 都是 RELIABLE），
      **且飞控侧 EKF2 确实在用**（`pos_horiz_abs` 反转实验，见 §实测 ③）
- [x] Jetson 侧串口**定案并记录**：**`/dev/ttyTHS1` @ 921600**（2026-09-21 实测 `--scan`，sysid=1 type=2）
- [x] **①** 未解锁时 QGC 的 `VISION_POSITION_ESTIMATE` **有数且不跳**，`LOCAL_POSITION_NED` **已稳定**
      ⟹ 这才是"EKF2 在解锁前就已经收敛"的第一手证据（[ticket 04](04-mocap-relative-pose.md) 的验收项 ①）
      ⏳ **实质部分已由 §实测 ③ 拿下**（飞控自己的 `estimator_status` 说明水平位置解由动捕锚定）；
      **只剩 QGC 目视**（人）
      ✅ **2026-09-22 人报：两个话题都有数、轴约定方向正确**（原话与解读见 §实测 2026-09-22）。
      ⚠️ "**不跳**"与具体 Δ **未单独留档**（操作者当日判定"不用太严谨"，见 ⑥ 那条的说明）
- [x] **⑥** 把飞机向动捕 **+x 方向平移 0.5 m** → QGC 的 **`y` 变化 0.5**；再**升高 0.3 m** → QGC 显示**向上**
      ⟹ 若变的是 `x`，**问题在 mavros 侧或参数侧**（`mocap` 节点已用假输入排除）
      ⏳ **仍需人动手**：mavros 的转换代码已按源码逐行核对（§实测 ②），这一步验的是**轴向约定**
      （ENU 的东是不是 PX4 的北的右手侧），**只能靠看飞控自己的 NED 读数**（QGC）
      —— ✅ **但"转换到底做没做"这一问已由 §实测 ⑦ 拿下**（人挪 60 cm，两条 ROS 话题逐毫米吻合且**未换轴**，
      若 mavros 入向没转换则出向会把轴换回来、0.594 会出现在 `x` 上）；
      ⏳ **剩下的是 ROS 话题看不到的那一层**：QGC 里 `LOCAL_POSITION_NED` 的**轴标签**。
      按源码映射（`transform_frame_enu_ned`：ENU `y`→NED `x`），飞机沿动捕 **+y** 挪的这 59.4 cm
      **在 QGC 里应读作 `x ≈ +0.59`**（`y ≈ +0.014`，`z ≈ +0.002`）。
      ⟹ 两条路任选其一即可收尾：**(a)** 直接读当前 QGC 的 `x` 是不是 +0.59（飞机保持不动）；
      **(b)** 再按原描述向动捕 **+x（机头方向）推 0.5 m**，看 QGC 的 `y` 于是变 0.5；
      外加**升高 0.3 m** → QGC 的 `z` 变 −0.3（NED 向下为正 ⟹ 读数变小 = 变高）
      ✅ **2026-09-22 人报：轴约定（左 `x+` / 机头 `y+` / 下 `z+`）与上述期望一致**
      （`z+` 向下 ⟹ 是 NED ⟹ 入向转换确实做了；见 §实测 2026-09-22 的解读）。
      ★ 操作者当日明确指示「**不用太严谨，验证坐标系正确即可**」⟹ **本框按定性判据勾选**：
      **具体 Δ 数值未记录**（`y` 的 Δ、升高后 `z` 的 Δ）与观测时刻均未留档。
      将来若要数值，按上面 (a)/(b) 两条路任一复核即可（须在"数据窗"内）。
- [x] 顺带记录 QGC 侧 `EKF2_EVP_NOISE` 的取值与现象（是否发散），供参数核对清单使用
      ⟹ ✅ **取值已从飞控真身读到：`0.06095999851822853`**（与参数文件一致）；
      台架静止 ~30 min 无发散、`pos_horiz_abs` 稳定为 True。**"0.061 是否过小"仍属飞行阶段判定**（见 [ticket 15](15-fcu-param-reconciliation.md)）
- [x] **`ros2 topic hz /mavros/state` 定案**（是不是 1 Hz ⟹ 解锁上升沿最多晚多久被检出），结论写回 04 §七
      ⟹ ✅ **1.00 Hz**（30 条 / 30 s），结论已写回 04 §七 第 4 条
- [x] **试一次解锁上升沿切原点**（QGC 里解锁、不起飞）：节点日志应打出「★ 阶段 B … 切换发生在此刻」，
      且**切换瞬间位置跳变 ≈ 0**（静止摆放的前提；> 0.05 m 会有 WARN）
      ⏳ **需要人解锁**（agent 不主动解锁真机）。★ 台架要求：**拆桨**、飞机摆好后静止不动
      ❌ **2026-09-22 上午 没做成**：`mocap` 节点日志全文**无「★ 阶段 B」**，
      而原因是源码级的必然 —— 阶段 B 的触发**只在 `on_pose` 内**（`mocap_rel_pose_node.cpp:171/190`），
      当时所有解锁都落在**输入断流窗口**里 ⟹ 边沿被错过
      🚧 **同日又添一道阻塞（与上面那条独立）**：操作者解锁时被**飞控自己**拦下 ——
      `crash dumps present on SD, vehicle needs service`（SD 卡根目录有硬故障转储 `fault_2000_01_01_00_00_01.log`，
      内容 = `wq:I2C1` 硬故障）⟹ 见 [ticket 20](20-fcu-hardfault-i2c1-crash-dump.md)（只读取证 + 清障）
      ✅ **同日 07:57 已清障**（用户授权，`/mavros/ftp/remove`，删前已留档）
      ✅ **同日 08:17:53 触发成功**（操作者解锁、**拆桨**、电机点转 4.2 s 后上锁）：
      日志打出「★ 阶段 B：解锁上升沿后的起飞前原点已锁定 —— 切换发生在此刻」（帧 stamp `1790065073.227048635`），
      比 `Armed by external command`（`08:17:52.542`）晚 **0.685 s**（与 `/mavros/state` 1 Hz + 60 帧均值一致）
      ⚠️ **但第二条判据没达标**：两阶段原点切换量 **0.5738 m** > 0.05 m ⟹ WARN 如实打出。
      原因是**流程偏差而非缺陷**：`mocap` 节点自 `06:27:33` 起就没重启过（阶段 A 原点取自那时），
      而飞机在这 110 分钟里被搬动过、动捕还断过两回 ⟹ 启动原点 ≠ 起飞点。
      ★ **用户 2026-09-22 裁定：判为已验证后收 `done`** —— 触发路径与阈值告警路径**都**已实测
      （0.5738 > 0.05 的 WARN 确实打得出来），跳变量本身由流程造成；
      「摆好 → **重启 `mocap`** → 解锁」已升格为**飞行日纪律**写进
      [`bench-runbook.md`](../../../docs/control/bench-runbook.md)（T4 与起链段）。
      **残留**：「跳变 ≈ 0」这一条**至今零观测**（哪天按纪律走一遍时顺手记一下即可，不必再单开验收）。
      **残留不藏**：「跳变 ≈ 0」这一条**至今零观测**；它已写进
      [`bench-runbook.md`](../../../docs/control/bench-runbook.md) **T4 的「记录」列**（`跳变量（m）`）⟹
      飞行日那一跑会**顺便**把它记下来，**不必再为它单开一张验收票**（不是"不用管"，是"有人在收"）。
      一手数据见 §实测 2026-09-22 §阶段 B
- [x] 把两条验收的读数**回填到 [ticket 04](04-mocap-relative-pose.md)**，并把 04 的 `Status` 改成 `done`
      ✅ **2026-09-22 已做**：04 的 §七 第 1 条标 ✅、`Status` → `done`；
      **残留（① ⑥ 的具体 Δ、以及 ⑥ 的观测时刻）在 04 里点名为本张的 §实测 2026-09-22 ⟹ 不藏在 `done` 下面**

★ **剩下这三条（① 目视 / ⑥ 轴标签 / 解锁切原点）已收进 [`docs/control/bench-runbook.md`](../../../docs/control/bench-runbook.md)
的 T1 / T2 / T4**（2026-09-22）：那张清单把"当下必须由人做的这一薄层"排成一次会话的顺序
（还含 ticket 05/06 的 4 条），并带上前置参数与记录表。做台架照它走。

## 为什么这两条非做不可

`phase1-implementation.md` §7 的验证顺序里，**第 1 步与第 1b/1c 步就是它们**。
跳过它们的代价不是"少一个勾"，而是「**飞机往反方向飞**」时你分不出是原点锁错、yaw 归零错、
还是 mavros 的边界转换不如预期 —— 那正是 [ticket 04](04-mocap-relative-pose.md) 要单独做一张的原因。
本张验的恰恰是**最后一个环节**（mavros 的转换），它是全链路里**唯一不由本项目代码控制**的那一段。

### 2026-09-21 —— 开工条记：串口已结案、mavros 换路径

- ✅ **串口定案**：`/dev/ttyTHS1` @ 921600（`mavlink_probe.py --scan`，`sysid=1, type=2`）。
  `dialout` 组已修好（`dialout:x:20:chen`，`id -Gn` 确认当前会话已在组内，**不用重登**）。
- ☠️ **apt 装 mavros 不通**（十个镜像都没有 humble 的 arm64 二进制）⟹ **改源码构建**，
  证据/裁定/步骤/坑全在 [ticket 14](14-mavros-from-source.md)（用户 2026-09-21 裁定）。
  本张的步骤 1、坑 1、坑 3 已就地改掉。
- 待 mavros 起来后继续：本张的 6 条验收（含 `ros2 topic hz /mavros/state` 定案与解锁上升沿测试）。

### 2026-09-21 —— ★ 台架实测（飞控已上电；mavros 源码构建已完成）

**环境**：`mavros` + `mavros_extras`（源码 2.15.1，见 [ticket 14](14-mavros-from-source.md)）**已连上飞控** ——
`CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot`；`/mavros/state` =
`connected: true, armed: false, guided: true, mode: AUTO.LOITER, system_status: 3`。
动捕主机 `10.1.1.198:3883` 在线（ping 7.7–42 ms），`vrpn_listener` 报 `Found new sender: z0`（**小写**）。

#### ① `/mavros/state` = `1.00 Hz`（**定案**）

```
/mavros/state: 30 条 / 30s = 1.00 Hz；间隔 min 0.998s / mean 1.000s / max 1.002s
```

⟹ 解锁上升沿最多晚 **1 s** 被检出（缓解措施见上方坑 4）。结论已写回 [ticket 04](04-mocap-relative-pose.md) §七 第 4 条。

★ **机理由 [ticket 05](05-control-offboard-skeleton.md) 补上（2026-09-21，源码级）**：这**不是 mavros 的定时器、改不了** ——
mavros 只在**收到 MAVLink `HEARTBEAT` 时**才发这个话题（`sys_status.cpp` 的 `handle_heartbeat` → `state_pub->publish`），
而 `HEARTBEAT` 在 PX4 里是定频流（`mavlink/streams/HEARTBEAT.hpp` 的 `const_rate()` 返回 true）。
⟹ `msg_timeout` 类判据**不能照抄 px4ctrl 的 0.5 s**（会每秒报"飞控失联"），ticket 05 取 **2.5 s**。

#### ② mavros 侧边界：把 §3.5 的结论对到**真源码**上

`ros2 topic info -v /mavros/vision_pose/pose`：

- Publisher count 1 = `mocap_rel_pose`（RELIABLE / VOLATILE）
- **Subscription count 1 = `vision_pose` @ `/mavros`（RELIABLE / VOLATILE）** ⟹ QoS 兼容，消息**真的进了 mavros**
- mavros 日志：`Plugin vision_pose created` / `Plugin vision_pose initialized`

源码 `~/mavros_ws/src/mavros/mavros_extras/src/plugins/vision_pose_estimate.cpp`（本次构建的那份）：

| 行 | 代码 | 含义 |
|---|---|---|
| `:125` | `if (last_transform_stamp == stamp) { ... return; }` | **唯一的丢弃条件 = 时间戳不前进**（DEBUG 级、不刷屏）⟹ "stamp 原样转发"是必须的 |
| `:133` | `ftf::transform_frame_enu_ned(tr.translation())` | 位置 **ENU→NED** |
| `:135-136` | `transform_orientation_enu_ned(transform_orientation_baselink_aircraft(q))` | 姿态 **FLU→FRD** 再 **ENU→NED** |
| `:138` | `transform_frame_enu_ned(cov)` | 协方差同步转 |
| `:143` | `vp.usec = stamp.nanoseconds() / 1000` | 时间戳以**微秒**发给飞控 |

#### ③ ★★ EKF2 真的在融合 —— `pos_horiz_abs` 反转实验（**零注入、无需人动手**）

被测量：`/mavros/estimator_status`（`ESTIMATOR_STATUS`，实测 1 Hz）的 `pos_horiz_abs_status_flag`。
**本机没有 GPS、没有光流**（`EKF2_GPS_CTRL = 7` 但没有 GPS 数据），水平绝对位置源**只有动捕**：

| 阶段 | 做法 | `pos_horiz_abs` |
|---|---|---|
| 有视觉 | `mocap` 节点在跑 | **True**（7/7 帧；另一次测 10/10） |
| 无视觉 | 停掉 `mocap` 节点，等 4 s | **False**（0/12 帧） |
| 恢复 | 重新起 `mocap` 节点，等 7 s | **True**（7/7 帧） |

⟹ **`/mavros/vision_pose/pose` → mavros → 飞控 → EKF2 这条链路是通的，且飞控确实在用动捕锚定水平位置。**
（比"QGC 里看着有数"强：这是**飞控自己的估计器状态**。）

★ 顺带一条安全结论：**视觉一断，水平锚定在 4 s 内就失效** ⟹ [ticket 04](04-mocap-relative-pose.md) 的动捕看门狗告警是**安全必需**，不是锦上添花。

#### ④ 记录用的读数

| 话题 | 读数 |
|---|---|
| `/vrpn/z0/pose` | sender = `z0`（小写） |
| `/mocap/pose_enu` | 181.8 Hz；位置 ≈ (0,0,0)、yaw ≈ 0°（阶段 A 已锁：`p_boot = (-1.2803, -0.1399, 0.0881)`） |
| `/mavros/local_position/pose` | 25.8 Hz；位置 ≈ 0、yaw ≈ 0° |

⚠️ **`/mavros/local_position/pose` ≈ 0 这一条不构成证据**：飞机轴对齐 + 无磁罗盘时，EKF2 即使**没有**视觉也会停在 ~0。真判据是 ③。

#### ⑤ 三个会静默骗人的坑（实测踩到）

1. **`/mavros/local_position/pose` 是 sensor QoS（BEST_EFFORT）** —— 用默认 RELIABLE 订阅**一帧都收不到**，
   只在日志里留一句 `New publisher discovered ... incompatible QoS ... Last incompatible policy: RELIABILITY`。
   ★ 对 [ticket 05](05-control-offboard-skeleton.md) 的假 mavros 夹具是硬要求（已在该张留言）。
2. **`/mavros/local_position/pose_cov` 与 `velocity_*_cov` 默认没有数据** —— 源码 `mavros/src/plugins/local_position.cpp:257`
   只在飞控推 `LOCAL_POSITION_NED_COV` 时才发布，而 PX4 默认不推这个流 ⟹ **"协方差是否发散"这类实验不能直接用它们**，
   得先用 `/mavros/set_message_interval` 请求该流。
3. **`/mavros/param/get` 在 ROS2 版 mavros 里不存在** —— `mavros/src/plugins/param.cpp` 里那行被注释掉了
   （`// rclcpp::Service<mavros_msgs::srv::ParamGet>::SharedPtr get_srv;`），只有 `~/pull` 与 `~/set`。
   ⚠️ **"只有两个服务"这句不完整**（2026-09-22 重读 `param.cpp` 更正）：还有 `~/get_parameters`（`:468`）与
   `~/set_parameters`（`:476`，`ros2 param set` 走的就是它）；`~/set_parameters_atomically` 硬编码不支持。
   **"没有 `~/get`"这个判断本身是对的**，不影响下面的结论。逐条表格见
   [ticket 15 的评论](15-fcu-param-reconciliation.md) 与 `params-changes.md` §I.2。
   读飞控参数的正确方式是 **`ros2 param get /mavros/param <参数名>`**（飞控参数被暴露成标准 ROS 参数，
   启动时自动拉取；本次缓存到 **915 条**）。见 [ticket 15](15-fcu-param-reconciliation.md)。

#### ⑦ ⑥ 的动态一半：飞机被真实挪动后，**EKF2 逐毫米跟上**（2026-09-21，操作者在场）

操作者把飞机**向左**挪了一下（当时 yaw ≈ 0 ⟹ 机体左侧 = 动捕 **+y**）。两边读数（5 s 均值）：

| | x | y | z | yaw |
|---|---|---|---|---|
| 动捕输出 `/mocap/pose_enu` | +0.0137 | **+0.5940** | −0.0022 | −1.18° |
| EKF2 `/mavros/local_position/pose` | +0.0134 | **+0.5939** | −0.0024 | −1.18° |

⟹ 两条**独立来源**（本节点算的输出 vs EKF2 自己估出来的）**差 0.3 mm、yaw 差 0.01°** ⟹
**EKF2 在实时跟随动捕**（不只是 ③ 里的静态锚定）；`pos_horiz_abs` 仍为 True。

★ 顺带**排除了一种失效模式：mavros 入向"没做 ENU→NED 转换"**。若入向没转，出向的 `NED→ENU`
会把两个水平轴**换回来** —— 那么 0.594 会出现在 `x` 上而不是 `y` 上。实测 0.594 在 `y` 上
⟹ 入向转换**确实执行了**（与 ② 的源码核对一致）。

> ✅ **与操作者的口述吻合**：操作者说挪了 **60 cm**（起初口误说 40 cm，随即更正），动捕量到 **59.4 cm**
> ⟹ 差 0.6 cm 是"人手摆位 + 刚体中心 vs 目测边缘"的量级。
> ⟹ 这一测是**定量**的：**人的 60 cm ≈ 动捕 59.4 cm ≈ EKF2 59.4 cm**。
> 另注：同时 yaw 变了 **−1.15°**（不是纯平移，飞机被轻微带转），而两边 yaw 读数**同样一致**（均 −1.18°）
> ⟹ **yaw 通道也在被融合**（`EKF2_EV_CTRL=11` 含 yaw 的又一个旁证）。

> ⚠️ **对"解锁上升沿切原点"测试的影响**：飞机现在离**启动原点**已 0.594 m，而纪律要求
> "节点启动到解锁之间静止"。**若现在直接解锁**，阶段 B 会把当前点当"起飞前原点"，
> 切换跳变 = 0.594 m > `jump_warn_m`（0.05）⟹ 会打 WARN，且 EKF2 会把这 0.594 m **当成真实运动**。
> ⟹ **做解锁测试前，先在飞机的最终摆放位置重启 `mocap` 节点**（让启动原点 = 起飞点），再解锁。

#### ⑧ 还剩什么（**需要人**）

1. **① 的 QGC 目视 + ⑥ 轴向确认**（本张两条主验收）：
   - ✅ **QGC 不必接数传线了**：2026-09-21 已把 mavros 的 MAVLink **经 WiFi 广播**出去
     （`ros2 param set /mavros_router gcs_urls "['udp-b://0.0.0.0:14555@:14550']"`，
     笔记本上 QGC 默认监听 14550 即自动建链；实测 ≈537 Hz、`0xfd` MAVLink v2）。
     步骤 + URL 语义 + 三个坑见 [`mocap-setup.md` §5.4](../../../docs/reference/mocap-setup.md)
     ⟹ **下面这几步现在人力可做**。
   - QGC → **MAVLink Inspector**：`VISION_POSITION_ESTIMATE` 有数且不跳、`LOCAL_POSITION_NED` 稳定；
   - **把飞机向动捕 +x（东）平移 0.5 m** ⟹ QGC 的 `LOCAL_POSITION_NED.y` 变 0.5；
     **升高 0.3 m** ⟹ 高度显示向上。
     ★ 若变的是 `x` ⟹ 问题在 mavros 侧或参数侧（`mocap` 节点已用假输入排除）。
   - 更硬的旁证（QGC → Analyze → **MAVLink Console** 里敲）：
     `listener vehicle_visual_odometry`（有数据）、`listener estimator_aid_src_ev_pos`（若存在，看 `fused`）。
2. **解锁上升沿切原点**（★ **拆桨**、飞机摆好后静止）：QGC 里解锁、不起飞，看 `mocap` 节点日志的
   「★ 阶段 B … 切换发生在此刻」与切换跳变量（应 ≈ 0）。
3. 回填 [ticket 04](04-mocap-relative-pose.md) 并把 04 的 `Status` 改成 `done`。

### 2026-09-21 —— 台架配套：★ QGC 走 WiFi（打通 ①/⑥ 的最后一公里）

操作者要求"在笔记本上用 QGC 看"。做法：给 mavros 的 router **热加**一条 GCS 链路，
把整条 MAVLink 往本机 WiFi 上**广播**（步骤/语义/坑已写进 [`mocap-setup.md` §5.4](../../../docs/reference/mocap-setup.md)）：

```bash
ros2 param set /mavros_router gcs_urls "['udp-b://0.0.0.0:14555@:14550']"
```

实测证据：`Set parameter successful`；`ss -ulnp` → `0.0.0.0:14555 users:(("mavros_node",pid=6511,...))`；
在 14550 放临时接收器 **6 s 收到 3223 个包（≈537 Hz），首字节全是 `0xfd`（MAVLink v2）**；
`ip route get 255.255.255.255` → `dev wlP1p1s0`（走 WiFi，不走 docker0）；
router 日志 `link[1002] opened successfully`。

**三个源码级坑**（都不是"猜"出来的，行号见 §5.4）：① 持链路的节点叫 **`mavros_router`**、参数是**复数
`gcs_urls`**，启动形参 `gcs_url` 只在启动时被搬一次 ⟹ 运行时改 `gcs_url` **无效**；
② `link[%d] detected remote address %d.%d` 打的是 **`sysid.compid` 而不是 IP** ⟹ 要确认 QGC 连上，
看日志里出现 **`255.x`**（QGC 的 sysid）而不是找 IP；③ 本机 `@` 之前是 **bind**、之后才是 remote。

> ⏳ **未决项（已当面问操作者，答案未知）**：router 日志里在**加 GCS 链路之前**就有
> `link[1000] detected remote address 32.0 / 59.93 / 75.1 / 0.205 / 21.159 …`（约每 30–60 s 新来一个、
> 又作为 stale 被清掉）⟹ 飞控这条链路上**有别的 MAVLink 系统**。
> 最可能的解释是飞控 **TELEM 口挂了数传电台**（`MAV_1_CONFIG=102` 指向 TELEM1），而电台网里还有别的设备。
> 若成立则**安全相关**（陌生 sysid 会参与 router 的 sysid 路由），需要一个归属；**当前无人认领**，
> 等操作者回答"TELEM 口有没有接电台"后再决定记到哪张 ticket。

### 2026-09-22 —— ★★ 台架实测（人做，agent 逐条记录）

操作者手动完成台架全部内容。**本节每个数字都有出处**（日志路径 / 源码行号 / 实跑命令）。
`Status` 当时是 `ready-for-human`：① ⑥ 的**具体读数**与「解锁上升沿切原点」还差人动手（见下）。
> ★ **2026-09-22 08:17 之后本张已 `done`**（阶段 B 触发、清障生效；残留见文末 §阶段 B）。下面那些
> "还差人动手"的记录是**当时**的状态，保留不改（记录不追改历史）。

#### ① ⑥ 的 QGC 目视 —— ✅ 有数、轴约定方向正确（人报）

操作者原话（逐字）：

> 查看 `VISION_POSITION_ESTIMATE` 和 `LOCAL_POSITION_NED` 话题，都是左为 `x+`，机头朝向为 `y+`，下为 `z+`

**agent 的解读（★ 不是原话，勿当读数引用）**：

- `z+` **向下** ⟹ QGC 里这两条显示的是 **NED**，不是动捕原生 ENU（后者的 `z+` 向上）
  ⟹ **mavros 的 ENU→NED 转换确实发生了**，与 §实测 ② 的源码逐行结论同向
  （`transform_frame_enu_ned`：ENU `y`→NED `x`）；
- "左 `x+` / 机头 `y+`" 与**"机头朝动捕 +x（东）摆放"**这一姿态下的 NED 自洽：
  NED `x` = 北 = 机头朝东时的**机体左侧**、NED `y` = 东 = 机头方向、NED `z` = 下；
- ⟹ **若**上述姿态成立，则 ⑥ 的期望随之成立：沿动捕 **+x** 平移 0.5 m 就是**沿机头方向推**
  ⟹ QGC 的 `y` 变 0.5，升高 0.3 m ⟹ NED `z` **读数变小**（`z+` 向下 = "显示向上"）。

⏳ **收敛这一条还差两件（本张当时因此保持 `ready-for-human`；08:17 后已 `done`）**：

1. **具体 Δ 读数**：`y` 变了几何、升高 0.3 m 后 `z` 变了几何（验收项 ⑥ 写的是 `y` 变 0.5 / `z` 变 −0.3）；
2. **观测时刻** —— 只有下面"数据窗"里通的那两段看到的才是活数据。

#### ★ 数据窗：动捕输入今天断过两次，**第二次现在还没恢复**

`~/.ros/log/mocap_rel_pose_node_48895_1790058454153.log` 全文只有 5 条（该进程 `06:27:33` 起从未重启）：

| 本机时刻 | 事件 |
|---|---|
| `1790058454.177` | 节点启动 |
| `1790058455.170` | ★ 阶段 A 锁定：`p_boot = (-2.6267, -0.8579, 0.0896)` |
| **`1790059928.203`** | ★ ERROR：**动捕数据已中断 5.4 s**（共收到 `284207` 帧，最后一帧在 `1790059922.805`） |
| `1790060722.957` | WARN：动捕数据已恢复（**中断了约 800.2 s**） |
| **`1790061319.274`** | ★ ERROR：**又中断 5.9 s**（共收到 `401275` 帧，最后一帧在 `1790061313.373`） |

⟹ 输入**只有两段是通的**：`06:27:34.9 → 06:52:02.8`（284207 帧 / 1467.859 s ⟹ ≈193.6 Hz）与
`07:05:22.9 → 07:15:13.4`（117068 帧 / 590.415 s ⟹ ≈198.3 Hz）；
**`07:15:13` 起断到 `07:29` 复核时仍为零帧**。

**这条断链不是我们的节点**（三处第一手核实，2026-09-22 07:29）：

1. `ros2 topic hz /vrpn/z0/pose` 在断流期**打不出任何读数**（只 `Terminated`）⟹ **上游就没在发**，
   不是订阅侧丢帧；★ 这个观察者是**宽容**的（best_effort 订阅方兼容两种发布方）⟹ "零帧"是真零帧；
2. `ros2 topic info -v /mocap/pose_enu` → `Publisher count: 1`（`mocap_rel_pose`）、`Subscription count: 0`
   ⟹ 图上**没有第二个发布者**（详见 [ticket 19](19-unidentified-pose-enu-publisher.md) 的排除清单）；
3. `mocap` 节点的订阅与其 QoS 自 `06:27:33` 起未变，同一订阅在断前断后都收到 ~200 Hz
   ⟹ 不是"best_effort 发布方配 reliable 订阅方"那类**静默收不到**（那个坑的表现是"从来收不到"，不是"收 280 万帧后停"）。
   旁证：`/tmp/z0_vrpn.log`（mtime `07:04`）里 Nokov 侧反复 TCP `dropping connection`。

★ **后果（必须记住）**：`06:52:02.8 → 07:05:22.9` 这 **800.2 s 里 EKF2 一个视觉样本都没有**
（`mocap` 节点唯一的发布点在 `on_pose` 内，见下），`07:15:13` 起至今同样没有。
**在这个状态下任何"飞机跟着动捕走"的读数都不能用**。`control` 的 `mocap` 守卫在这段时间里
**绝大多数运行也确实拒发 setpoint**（`动捕无数据（/mocap/pose_enu 静默 从未收到过）`、一次 `degrade=mocap_timeout`）
—— 判据工作正常。

#### ⏳ 步骤 §⑧ 第 2 项「解锁上升沿切原点」**没做成**（★ 但这不是 `mocap` 节点的 bug）

日志全文里**没有一行「★ 阶段 B」**。原因在源码里，是**必然**的：

- 阶段 B 的触发与打印**都在 `on_pose` 内**
  （`src/mocap/src/mocap_rel_pose_node.cpp:171` `selector_->observe(in, armed_state())`；
  `:190` `if (selector_->switched() && !logged_switch_)` ⟹ `:200 log_arm_lock()`）
  —— **没有输入帧就永远走不到这一步**；
- 而今天**所有解锁都落在断流窗口里**（`control` 日志：`06:53:33`、`06:55:36` … `07:05:03` 都有 `MAV_RESULT=0`），
  即解锁那一刻输入是断的；
- 解锁是**边沿触发**：边沿在断流期被"错过"后，输入恢复（`07:05:22.9`）时已经不再是边沿。

⟹ **这一项必须在输入通的时候重做**（`docs/control/bench-runbook.md` T4）：
摆放静止 → 确认 `/vrpn/z0/pose` 有 ~200 Hz → 解锁不起飞 → 看日志出现
「★ 阶段 B … 切换发生在此刻」且 `切换瞬间位置跳变 ≈ 0`。

> ✅ **2026-09-22 08:17 已重做**：输入通（`08:16:42` 恢复）时操作者解锁，日志打出「★ 阶段 B」——
> 本节记录的是**同一 08:17 之前**的状态。结果、残留与纪律见本文件末 §实测 2026-09-22 §阶段 B。

#### ★★ 一处**未解**异常：`07:05:00` 有人替 `mocap` 节点发了帧

`~/.ros/log/control_node_58161_1790060699586.log`（进程起于 `07:04:59.62`）：

| 时刻 | 行（节选） |
|---|---|
| `07:04:59.64` | `→ MANUAL/INIT … 动捕无数据（/mocap/pose_enu 静默 从未收到过）` |
| **`07:05:00.58`** | `→ MANUAL/STANDBY … 前置条件全成立（飞控已连接 ∧ **动捕有数据** ∧ local_position 有效 ∧ arm 守卫成立）` |
| `07:05:03` | 解锁 `MAV_RESULT=0`；`07:05:04` TAKEOFF；`07:05:07` HOVER 已到高；`07:05:11` WAIT_START；`07:05:13` `SIGINT` |

**这比 `mocap` 节点自己记录的恢复时刻（`07:05:22.957`）早了 22.4 s**：控制节点从 `/mocap/pose_enu`
上收到了消息，而 `mocap` 节点在那段时间**没有任何发布路径** —— 已通读源码：
唯一发布点 `:196-197`（在 `on_pose` 内）、`on_state`（`:126-130`）只写 `has_state_/armed_`、
订阅只有 `/vrpn/z0/pose`（`:84-86`）与 `/mavros/state`（`:87-89`）。

已排除的候选（每条都有第一手证据，**清单见 [ticket 19](19-unidentified-pose-enu-publisher.md)**）：
第二个发布者、mavros、`control` 节点自己、QoS 错配、sim time / `/clock`、第二份工作区或第二个 `mocap` 进程、
rosbag 回放、本仓库的 bench fixture（硬拒 domain 0）、cron/systemd/自启。

★ **安全含义要写清**：这一帧只影响 `control` 的**守卫**（"动捕有数据"），**没有**污染控制律
（位置来自 EKF2 的 `local_position`）；同一时刻 `/mavros/vision_pose/pose` 上
`Publisher count: 1`（仍是 `mocap_rel_pose`）⟹ **EKF2 那 800.2 s 确实没有视觉**。
但"守卫能被一个来历不明的发布者满足"本身是缺陷级问题 ⟹ 已另开 [ticket 19](19-unidentified-pose-enu-publisher.md)。

#### 台架前提（★ 记录读数时必须带上，否则会被读高）

操作者报：**"桨拆着，我用手托着举到 1 m"** ⟹ 今天所有 `已到高 +1.00 m` 的读数都是
**拆桨 + 手持**下取得的：它验的是"状态机 / 链路 / 高度判据对不对"，**不是**"飞机能自主起飞"。
真自主起飞要等装桨后的自由飞阶段。

#### 飞控侧（只读，顺带记录）

- **旧 mavros 实例（pid `47747`）日志**：`06:53:47.87` `CON: Lost connection, HEARTBEAT timed out.`
  → `06:53:52.95` 重连（就是操作者重启飞控那一下）；与 `control` 日志 `06:53:47` 的
  `degrade=mode_lost` 时刻吻合 ✓ **两条独立日志互证**。
  当前实例是 `07:06:30` 起的新 mavros（pid `58340`）。
- 飞控**有 SD 卡日志**：`FCU: [logger] /fs/microsd/log/sess253/log100.ulg`（`06:53:53`）
  ⟹ 以后要问"**飞控侧**到底有没有收到视觉"，可以直接拉 ulog 对 `vehicle_visual_odometry`，
  不必只靠 ROS 侧推理。★ 但**今天这份 ulog 的取证还没做**（不在本张验收里）。
- 从飞控真身实读（`ros2 param get /mavros/param …`，同日 `07:30` 前后）：
  `NAV_RCL_ACT = 1`（★ 2→1 已生效）、`EKF2_EV_CTRL = 11`、`COM_OF_LOSS_T = 1.0`、
  `EKF2_EVP_NOISE = 0.06095999851822853`、`EKF2_EV_POS_Z = 0.0`（仍未标定 ⟹ 归 [ticket 15](15-fcu-param-reconciliation.md)）。

### 2026-09-22 08:17 —— ★★ 阶段 B 实测（操作者解锁，agent 逐条取证）

#### 时刻线（本机时钟；epoch → 本地已换算）

| 本机时刻 | epoch | 事件 | 出处 |
|---|---|---|---|
| `08:16:42` | `1790065002.42` | 动捕输入恢复（中断 3689.0 s） | `mocap_rel_pose_node` 日志 |
| `08:17:52.542` | `1790065072.542` | `FCU: Armed by external command` | `~/.ros/log/mavros_node_*.log` |
| **`08:17:53.227`** | `1790065073.227` | ★ **阶段 B 原点锁定**（帧 stamp `…073.227048635`） | `mocap_rel_pose_node_48895_*.log` |
| `08:17:53.227` | — | ⚠️ 切换量 **0.5738 m** > 0.05 m ⟹ WARN | 同上 |
| `08:17:56.741` | `1790065076.741` | `FCU: Disarmed by external command` | mavros 日志 |
| `08:18:39` | `1790065119` | `/mavros/state` = `connected: true, armed: false, mode: POSCTL` | 就地读 |

#### 两条判据各自的结论

1. **「★ 阶段 B」触发** ✅ —— 日志原文：
   `★ 阶段 B：解锁上升沿后的起飞前原点已锁定 —— 切换发生在此刻（帧 stamp = 1790065073.227048635，本机 1790065073.227284431）`。
   **时延自洽**：`Armed`（`52.542`）→ 锁定（`53.227`）差 **0.685 s**。这一数不是巧合能凑出来的：
   `/mavros/state` 是 **1.00 Hz**（本票验收项已定案）⟹ 边沿检出本身最多晚 1 s；
   检出后还要采 **60 帧 ≈ 0.3 s**（`200 Hz`，`mocap_rel_pose_node.cpp`）才锁定并打印。
   `0.685 s` 落在 `[0.3 s, 1.3 s]` 里 ⟹ **两个已知常量把观测值夹住了**，是这跑最扎实的一处。
2. **「切换瞬间跳变 ≈ 0」** ❌ —— 实测 `0.5738 m`，WARN 原文：
   `⚠️ 两阶段原点切换量 0.5738 m 超过阈值 0.0500 m —— 说明「启动到解锁之间飞机被移动/转动了」。本次飞行位置会有一个同量的跳变（EKF2 会把它当成真实运动）。下次请严格遵守纪律：摆好后到解锁前别动它。`
   **归因（流程，不是缺陷）**：`mocap` 节点 PID `48895` 起于 `06:27:33`（阶段 A 原点 `p_boot = (-2.6267, -0.8579, 0.0896)`，
   `06:27:35` 锁定），而解锁发生在 **110 分钟后**；期间飞机被搬动（本票 §实测 ⑥ 那次"人挪 59.4 cm"就在其中）、
   动捕还断过 **800.2 s** 与 **3689.0 s** 两回。
   ⚠️ **说准一点**：`0.5738 m` **本身就是"两个原点之差"的实测值**（节点算完才打印 WARN），
   但**差的来源没有细分** —— 既可能是"飞机被搬动"，也可能是"动捕中途重锁/重识别"。
   ⟹ 能说的是：**两个原点取自相隔 110 分钟的两个时刻，实测差 0.5738 m**；
   **不能**说成"飞机一定被挪了 0.5738 m"。而**解锁前重启节点**这一条纪律对**两种来源都成立**
   （把两个原点压到同一时刻取）。
   ★ 用户裁定：触发路径与阈值告警路径**都已实测**（WARN 打得出来这条同样是第一手证据），判为**已验证**、本张收 `done`；
   「跳变 ≈ 0」这条留作**零观测的残留**（按 [`bench-runbook.md`](../../../docs/control/bench-runbook.md) T4 的纪律做时才可能看到）。

#### 顺带拿到的两条旁证

- **清障真的生效了**：同日更早时解锁被 `Crash dumps present on SD` 拒绝（见上方验收项），
  `07:57` 删掉转储后这次解锁**一次通过** ⟹ 「被拦 → 删 → 能解锁」因果闭合（细节见 [ticket 20](20-fcu-hardfault-i2c1-crash-dump.md)）。
- **点转没有产生新转储**：解锁 + 电机无桨点转 4.2 s 之后，SD 卡根目录仍是
  `APM / dataman / log / parameters_backup.bson`（**无 `fault_*` 文件**）⟹ 这次点转没有触发硬故障。
- `/mavros/vision_pose/pose` 同时刻 **197.7 Hz**（10 s / 1977 帧）⟹ 链路在跑，不是"EKF2 拿到了空输入"。

#### 纪律（★ 已写进 runbook，飞行日按它走）

**摆好 → 重启 `mocap` 节点（阶段 A 原点 = 起飞点）→ 起链 → 解锁**。
`T1/T2` 需要搬飞机（⑥ 就要求挪动），所以**必须**在 `T2` 做完、`T3` 起链**之前**重启 `mocap`
（放在 T3/T4 之间也行，但那会让 EKF2 在解锁前丢几秒视觉）；
不重启的话解锁瞬间会往 EKF2 灌一个同量级的位置跳变（本次 `0.5738 m`）。
