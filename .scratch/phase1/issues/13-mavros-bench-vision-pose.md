# 13: 台架 —— 装 mavros(+extras) 并完成 QGC 侧的 vision_pose 验收（[ticket 04](04-mocap-relative-pose.md) 的残留项）

Status: ready-for-human
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
- [ ] **①** 未解锁时 QGC 的 `VISION_POSITION_ESTIMATE` **有数且不跳**，`LOCAL_POSITION_NED` **已稳定**
      ⟹ 这才是"EKF2 在解锁前就已经收敛"的第一手证据（[ticket 04](04-mocap-relative-pose.md) 的验收项 ①）
      ⏳ **实质部分已由 §实测 ③ 拿下**（飞控自己的 `estimator_status` 说明水平位置解由动捕锚定）；
      **只剩 QGC 目视**（人）
- [ ] **⑥** 把飞机向动捕 **+x 方向平移 0.5 m** → QGC 的 **`y` 变化 0.5**；再**升高 0.3 m** → QGC 显示**向上**
      ⟹ 若变的是 `x`，**问题在 mavros 侧或参数侧**（`mocap` 节点已用假输入排除）
      ⏳ **仍需人动手**：mavros 的转换代码已按源码逐行核对（§实测 ②），这一步验的是**轴向约定**
      （ENU 的东是不是 PX4 的北的右手侧），**只能靠看飞控自己的 NED 读数**（QGC）
- [x] 顺带记录 QGC 侧 `EKF2_EVP_NOISE` 的取值与现象（是否发散），供参数核对清单使用
      ⟹ ✅ **取值已从飞控真身读到：`0.06095999851822853`**（与参数文件一致）；
      台架静止 ~30 min 无发散、`pos_horiz_abs` 稳定为 True。**"0.061 是否过小"仍属飞行阶段判定**（见 [ticket 15](15-fcu-param-reconciliation.md)）
- [x] **`ros2 topic hz /mavros/state` 定案**（是不是 1 Hz ⟹ 解锁上升沿最多晚多久被检出），结论写回 04 §七
      ⟹ ✅ **1.00 Hz**（30 条 / 30 s），结论已写回 04 §七 第 4 条
- [ ] **试一次解锁上升沿切原点**（QGC 里解锁、不起飞）：节点日志应打出「★ 阶段 B … 切换发生在此刻」，
      且**切换瞬间位置跳变 ≈ 0**（静止摆放的前提；> 0.05 m 会有 WARN）
      ⏳ **需要人解锁**（agent 不主动解锁真机）。★ 台架要求：**拆桨**、飞机摆好后静止不动
- [ ] 把两条验收的读数**回填到 [ticket 04](04-mocap-relative-pose.md)**，并把 04 的 `Status` 改成 `done`

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
   读飞控参数的正确方式是 **`ros2 param get /mavros/param <参数名>`**（飞控参数被暴露成标准 ROS 参数，
   启动时自动拉取；本次缓存到 **915 条**）。见 [ticket 15](15-fcu-param-reconciliation.md)。

#### ⑥ 还剩什么（**需要人**）

1. **① 的 QGC 目视 + ⑥ 轴向确认**（本张两条主验收）：
   - QGC → **MAVLink Inspector**：`VISION_POSITION_ESTIMATE` 有数且不跳、`LOCAL_POSITION_NED` 稳定；
   - **把飞机向动捕 +x（东）平移 0.5 m** ⟹ QGC 的 `LOCAL_POSITION_NED.y` 变 0.5；
     **升高 0.3 m** ⟹ 高度显示向上。
     ★ 若变的是 `x` ⟹ 问题在 mavros 侧或参数侧（`mocap` 节点已用假输入排除）。
   - 更硬的旁证（QGC → Analyze → **MAVLink Console** 里敲）：
     `listener vehicle_visual_odometry`（有数据）、`listener estimator_aid_src_ev_pos`（若存在，看 `fused`）。
2. **解锁上升沿切原点**（★ **拆桨**、飞机摆好后静止）：QGC 里解锁、不起飞，看 `mocap` 节点日志的
   「★ 阶段 B … 切换发生在此刻」与切换跳变量（应 ≈ 0）。
3. 回填 [ticket 04](04-mocap-relative-pose.md) 并把 04 的 `Status` 改成 `done`。
