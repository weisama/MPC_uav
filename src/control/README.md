# control —— 自主飞行控制节点

把 PX4 拉进 OFFBOARD，并以 50 Hz 持续流「位置 + yaw」setpoint。

> ⚠️ **本包目前只实现到 `STANDBY` + 失效路径**（ticket 05 + 06）：已在流 setpoint、已切 OFFBOARD，
> 但**飞机未解锁**（本包自己没有解锁路径）、setpoint 内容是"位置 = 当前位姿"（原地保持）
> ⟹ 正常路径下**它不会让飞机动**。失效路径可用：打杆夺权、输入超时、低压、位置异常 ⟹ 停发/`LAND`。
> 起飞/悬停/降落归 ticket 08，轨迹归 ticket 07。

## ★ 运行纪律（比代码重要）

1. **启动顺序**：`mavros` → `vrpn_listener` → `mocap` → 摆好飞机、静止 → 本节点。
   顺序错了**不会报错**，只会停在 `INIT`，并从日志里逐条告诉你缺什么
   （话题、静默多久、话题图上有几个发布者、QoS 是什么）。
2. **`/mavros/state` 是 1 Hz，不是"节点卡了"**。实测 1.000 Hz（min 0.998 / max 1.001），
   根源是 mavros 只在收到 MAVLink HEARTBEAT 时才发这个话题，而 HEARTBEAT 是 PX4 的 1 Hz 定频流。
   ⟹ 本节点对它的超时判据是 **2.5 s**（不是 `px4ctrl` 的 0.5 s，那样会每秒都在报"飞控失联"）；
   代价是**解锁上升沿最多晚 1 s 被检出**（ticket 08 起飞前要把这条算进去）。
3. **退出 OFFBOARD 有两种方式**：停本节点（断流超过 `COM_OF_LOSS_T` 后 PX4 自行退出），
   或遥控器接管（ticket 02 的通道分配）。**不要用 QGC 抢模式** —— 那会让"谁在控制"变得说不清。
4. **`armed` 上升沿 = 异常**（本张没有解锁路径，台架注入除外）⟹ 看到 ERROR 日志就去查是谁解的锁，
   不要"重试一下"。等 ticket 08 落地后这条才改。
5. **降级之后想复位，只有两条路**（都不要去"帮它一把"）：
   - **落地上锁**（`armed` true→false）—— 主路径，对四类原因都有效；
   - **夺权场景**：让**摇杆停止动作**，再把模式**离开、然后交回** OFFBOARD（判的是模式升沿，
     所以我们得先看到它离开；摇杆还在动的话一复位就会立刻重新夺权）。
   ⚠️ 闩锁期间节点**从不自己请求 OFFBOARD** ⟹ 模式回到我们手里的那条升沿**只可能来自人**。

## 两个正交维度（不是扁平枚举）

| 维度 | 取值 | 含义 |
|---|---|---|
| **控制权** `Authority` | `MANUAL` / `AUTO` | 谁在控制飞机 |
| **任务态** `Phase` | `INIT` / `STANDBY` / `TAKEOFF` / `HOVER` / `TRACK` / `LAND` / `RC_OVERRIDE` | 程序在干什么 |

★ **`AUTO` 的定义是"本项目正在发会驱动飞机的指令（已解锁的 offboard）"**。
所以 `STANDBY` 虽然已经切进 OFFBOARD 模式，但飞机未解锁 ⟹ `/control/state` 仍是 `MANUAL/STANDBY`。
这条定义让"切模式"与"交出控制权"两件事不被混为一谈。

`TAKEOFF/HOVER/TRACK` 仍是占位（归 ticket 08/07）；`LAND` 与 `RC_OVERRIDE` 由 ticket 06 的失效路径到达。

## 失效降级（ticket 06）

**总原则：任何一条关键输入坏掉 ⟹ 停止发 setpoint**（不是"不更新目标" —— 那等于让飞机悬在原目标上，
操作员打杆也拉不回来），然后按**是否已解锁**分岔处置。

| `degrade=` token | 判据 | 未解锁 ⟹ | 已解锁 ⟹ |
|---|---|---|---|
| `rc_override` | 摇杆变化率超 `rc_stick_ov_us_per_s`（飞行员在动手） | 停发 + 交还控制权 | 同左，**且不请求任何模式** |
| `mode_lost` | **我们正在驱动**的那个模式被外部切走（上一帧在流 setpoint 的 `OFFBOARD` / 我们请求的 `AUTO.LAND`） | 停发 + 交还控制权 | 同左 |
| `state_timeout` | `/mavros/state` 静默 > `state_timeout_s`（2.5 s）⟹ 飞控失联 | 停发 + 回 `INIT` | 停发 + `LAND`（请求 `AUTO.LAND`） |
| `mocap_timeout` | `/mocap/pose_enu` 静默 > `mocap_timeout_s` | 同上 | 同上 |
| `local_pos_timeout` | `/mavros/local_position/pose` 静默 > `local_pos_timeout_s` | 同上 | 同上 |
| `rc_timeout` | `/mavros/rc/in` 静默 > `rc_timeout_s`（**只在已解锁时算失效**） | —— | 停发 + `LAND` |
| `battery_low` | `/mavros/battery` 电压 < `low_voltage_v` | 同上 | 同上 |
| `position_invalid` | `local_position` 出现 NaN/Inf（**防御性**，见下） | 同上 | 同上 |
| `mocap_stamp_stalled` | 动捕帧照发但时间戳**没超过历史最大戳** > `mocap_stamp_stall_s`（含冻住与倒退到旧纪元两种形状） | 同上 | 同上 |
| `none` | 没有降级 | —— | —— |

★ **两类降级的处置完全不同，不要合并**：`rc_override` / `mode_lost` 是**控制权被夺走**
⟹ 停发、交还，并且**不请求任何模式**（既不抢回 OFFBOARD，**也不请求 `AUTO.LAND`** ——
那是在跟正在接管飞机的人抢）；其余是**输入坏了** ⟹ 已解锁时主动中止任务、请求 `AUTO.LAND`。

★★ **`mode_lost` 的判据要看"丢的是哪个模式"**（2026-09-22 复核补）：只有**我们在流 setpoint 的
`OFFBOARD`**、或**我们自己请求的 `AUTO.LAND`**，被切走才算夺权。理由是我们自己一停发，PX4 就会在
`COM_OF_LOSS_T`（1.0 s）后**自行**退出 `OFFBOARD` —— 而本节点的判据是 0.5 s，
**真机上必然先看到"自己退出"这个形状**，它与"人被夺权"逐帧无法区分。误判的代价有两层：
日志把排查方向指错；更实际的是夺权会**升级**已闩锁的输入失效 ⟹ 从此连 `AUTO.LAND` 都不再重发。
代价是**刻意接受**的：有人手动切走模式、而当时飞机并未被我们驱动 ⟹ 由**打杆**那条判据兜
（PX4 `COM_RC_OVERRIDE` 的口径也一样：人一动杆就是接管）。
★ 这一条**只能在 seam 2 验**：假 mavros 不假扮 PX4 的 `COM_OF_LOSS_T` 退出（`CONTEXT.md`），
台架上造不出这个形状。详见 `docs/control/phase1-implementation.md` §4.4(1)。

★★ **动捕戳的基准是历史最大戳，不是"上一帧的戳"**（同上复核）：EKF2 收外部视觉样本的尺子就是
"比**已缓冲的最新样本**还新"（`EKF/estimator_interface.cpp:393`；不满足时只打一条
`ECL_WARN("EV data too fast")` 并把这一帧丢掉，`:401-403`）。"上一帧"基准对**倒退到旧纪元之后
继续正常前进**完全盲 —— 每一帧都比上一帧新 ⟹ 判据说健康，而 PX4 那边每一帧都在丢。
判别力是**实测**的（把基准临时改回"上一帧"⟹ 那条用例立刻变红），见 §4.4(2)。

★ **可复位（不是单向 latch）**，两条路径：

1. **未解锁的失效不闩锁** ⟹ 判据一恢复就自动回 `STANDBY`（台架上的证据就在这条）。
2. **已解锁的失效闩锁到"落地上锁"** —— `armed` 的 true→false 沿。**不需要重启节点、
   也不需要任何"清故障"命令**：操作员落地上锁即复位。这是与 XI35 的 `kf_fusion_fail`
   （一旦置位只能手动着陆）刻意不同的地方。
3. 夺权闩锁额外可由"外部把模式交回 OFFBOARD"复位（闩锁期间我们从不自己请求 OFFBOARD
   ⟹ 那条升沿只可能来自人）。**已解锁的输入失效不因切模式复位**（电池不会因为有人切了模式就变满）。
4. 闩锁记的是**首因**（最接近根因），不被后续判据改写；但**夺权可以升级**一个已闩锁的降级
   —— 飞行员永远优先（否则会出现"飞行员在接管、我们还在请求 AUTO.LAND"的拉锯）。

★ **日志可检索**：每个降级写 `degrade=<token>`（token 就是上表第一列，来自 `fsm.hpp::to_string(Degrade)`），
复位的那一帧写 `degrade=none：降级已复位（此前 degrade=<原因>）` ⟹
`grep -oE "degrade=[a-z_]+" 日志 | grep -v none` 能捞出全部降级事件、
`grep "已复位" 日志` 能捞出全部复位时刻。

⚠️ **拿不到 `AUTO.LAND` 时怎么办（诚实版）**：`AUTO.LAND` 需要 EKF2 的 local position
（`mode_requirements.cpp:141`），位置真失效时 PX4 会**拒绝**它 —— 那时唯一兜底是 PX4 自己的失效保护
（`NAV_RCL_ACT` / `COM_OBL_RC_ACT`）。而且 **z0 现值 `NAV_RCL_ACT = 2`（Return）**，
室内无 GPS 会去返航，这是 `params-changes.md` **A1** 的阻塞项 ⟹ **A1 不落实，
本节点的"RC 超时"路径后果就不是"就地悬停"而是"试图返航"**。

⚠️ **`state_timeout` 这条在真机上不是"请求 `AUTO.LAND` 直到结束"**（2026-09-22 复核补，第一手）：
真丢链路时 mavros 会在 `conn_timeout`（默认 10 s）之后**自己发一帧** `connected=false, armed=false, mode=""`
的合成状态（`sys_status.cpp:903-911`，由 `:1411` 的 `connection_cb(false)` 触发）⟹ 我们在 2.5 s 报超时并请求
`AUTO.LAND`，~7.5 s 后那帧的 `armed` 降沿**把闩锁复位**（`fsm.hpp:396` 只判 `armed`，不判 `connected`）
⟹ 节点回 `INIT`、不再请求。链路已断时继续请求本来也到不了 PX4，所以不算错 —— 但⚠️ **链路回来那一刻**
闩锁已被清，PX4 若仍 armed，节点会自主重新请求 `OFFBOARD`。详见 `docs/control/phase1-implementation.md`
§4.4(3) 与 **ticket 18**（要决策，别擅自改）。

⚠️ **`position_invalid`（EKF2 位置失效的 NaN 判据）是防御性的，真机这条链上抓不到东西**。
第一手核实（z0 固件 `4a0e65f2`）：`EKF2.cpp::PublishLocalPosition` 发的 `lpos.x/y/z` 不含 NaN 合成；
`LOCAL_POSITION_NED.hpp` 在 MAVLink 层就丢掉了 `xy_valid`；mavros 2.15.1 原样转 ENU。
⟹ 这条判据留着不亏（夹具能注入 NaN，节点行为可测），但**"EKF2 位置失效"真正的兜底是动捕超时**
（动捕是唯一位置源）。可用的候选载体见 ticket 16。

⚠️ **`battery_timeout_s` 刻意不做判据**：低压是**值**判断，不是**新鲜度**判断 ——
最后一帧读数若是低的，飞控静默之后它**仍然是低的**，此时"超时就不判"是错的方向。
（真要加的话是"电池话题静默 ⟹ 降级"，那是另一个判据，没人论证过，见 ticket 06 的 Comments。）

`/control/state` 发 `"<控制权>/<任务态>"`（如 `MANUAL/STANDBY`），**transient_local**
⟹ 晚订阅者立刻拿到当前值；另外每 `diag_period_s` 心跳重发一次。
观察：`ros2 topic echo /control/state`

## 数据流

```
/mavros/state ─────────────┐
/mavros/local_position/pose ┤
/mocap/pose_enu ────────────┼─▶ 守卫 ─▶ 状态机 ─▶ /mavros/setpoint_raw/local（PositionTarget）
/mavros/rc/in、/mavros/battery ┘        └─▶ /control/state（"<控制权>/<任务态>"）
```

| 话题 | 方向 | 类型 | QoS | 备注 |
|---|---|---|---|---|
| `/mavros/state` | 订阅 | `mavros_msgs/State` | 可靠 + **transient_local** | 上游 1 Hz |
| `/mavros/local_position/pose` | 订阅 | `geometry_msgs/PoseStamped` | **best_effort** | 上游 ~30 Hz |
| `/mavros/rc/in` | 订阅 | `mavros_msgs/RCIn` | 可靠 | 两个判据：**打杆夺权**（变化率）+ **超时**（仅已解锁时算失效） |
| `/mavros/battery` | 订阅 | `sensor_msgs/BatteryState` | **best_effort** | 低电压判据（**只看值**，不看新鲜度） |
| `/mocap/pose_enu` | 订阅 | `geometry_msgs/PoseStamped` | 可靠 | 来自 `mocap` 节点；**是守卫的输入**（新鲜度决定发不发 setpoint） |
| `/mavros/setpoint_raw/local` | **发布** | `mavros_msgs/PositionTarget` | 可靠 | 唯一的控制输出 |
| `/control/state` | **发布** | `std_msgs/String` | 可靠 + transient_local | `"MANUAL/STANDBY"` |

★★ **QoS 必须与真 mavros 逐条对齐，而且错了不会报错、只会"永远收不到"**：
`local_position` / `battery` 在真 mavros 上是 `SensorDataQoS()`（best_effort），
按默认（可靠）订阅会**静默不匹配**。依据与实测见 `include/control/px4_interface.hpp`
与 `src/bench/README.md`；`bench` 包有一条专门的测试盯着这件事。

## setpoint 的三个硬指标

| 字段 | 值 | 依据 |
|---|---|---|
| `type_mask` | **2552** | IGNORE_VX/VY/VZ + IGNORE_AFX/AFY/AFZ + IGNORE_YAW_RATE ⟹ 忽略速度/加速度/yaw_rate，**保留位置 + yaw**。不含 FORCE(512) 与 IGNORE_YAW(1024) |
| `coordinate_frame` | **1**（`FRAME_LOCAL_NED`） | ⚠️ **枚举名有误导性** —— 见下 |
| `header.stamp` | `now()` | ⚠️ 见下 |

⚠️⚠️ **`FRAME_LOCAL_NED` 不代表"你填的值是 NED"。** mavros 对入向的 `FRAME_LOCAL_NED` setpoint 做的是
**ENU→NED 转换**（`setpoint_raw.cpp:231` `transform_frame_enu_ned(position)`）⟹
**我们填进 `PositionTarget` 的是 ROS/ENU 口径**：

- **`position`：x 东、y 北、`z` 向上为正**（不是"向下为正"！照 NED 去翻 z 的符号会得到一架往地里钻的飞机）
- **`yaw`：从 ENU 的 +x 轴逆时针**，实测换算 **`yaw_ned = π/2 − yaw_enu`**
  （`setpoint_raw.cpp:235`，按 `frame_tf.hpp:180` + `ftf_frame_conversions.cpp:33/40` 逐字复算）
  ⟹ **本项目填的 `yaw = 0`，PX4 收到的是 +90°，即"机头朝东"**（不是"保持当前机头"）

★ 也正因如此，把 `/mavros/local_position/pose` 原样转给 setpoint 是**正确的**：
mavros 出向已做 NED→ENU（`local_position.cpp:157`），进向再做 ENU→NED ⟹ 一进一出恰好抵消，
PX4 收到的是它自己报的那个位置。**这不是巧合，是"两边都在 ROS 口径里"的必然结果。**

⚠️ 本张飞机未解锁 ⟹ 上面这些**不会让飞机真动**；但 ticket 08 定"起飞朝向"之前必须处理 `yaw`——
**不要留着默认 0 当"保持当前机头"用**。

⚠️ **spec 里"发往 PX4 的消息用微秒时间戳"那条不适用于本路径**（那是 Phase 2 走 uXRCE-DDS 时的约定）：
mavros 把 `header.stamp` 换算成 MAVLink 的 `time_boot_ms`（**毫秒**，且会截断成 32 位，
`mavros/include/mavros/plugin.hpp:264-267`），而 **PX4 根本不读这个字段**
（`mavlink_receiver.cpp::handle_message_set_position_target_local_ned` 一律用 `hrt_absolute_time()`
自己打戳，v1.14.3）。所以这里的口径与 mavros 自带的 setpoint 插件一致：`header.stamp = now()`。

## 参数（`config/control.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `setpoint_rate_hz` | `50.0` | setpoint 频率。PX4 要求进入 OFFBOARD 期间 ≥2 Hz |
| `offboard_prestream_s` | `1.0` | ★ 切 OFFBOARD 前 setpoint 必须先连续流多久。**断流 PX4 会自动退出 OFFBOARD**（`COM_OF_LOSS_T`，z0 现值 1.0 s）⟹ 这是前提不是优化 |
| `offboard_retry_s` | `1.0` | 切模式失败后的重试间隔（真正的成功判据是 `/mavros/state.mode` 变成 `OFFBOARD`） |
| `enter_offboard` | `true` | 自动切 OFFBOARD（参数可关）。⚠️ 关掉它真 PX4 就永远读不到这些 setpoint |
| `yaw` | `0.0` | setpoint 的 yaw（弧度，**ENU 口径**）。⚠️ 填 0 ⟹ PX4 收到 +90°（机头朝东），见上文 |
| `diag_period_s` | `1.0` | 诊断日志周期；也是 `/control/state` 的心跳周期 |
| `state_timeout_s` | **`2.5`** | ★ 不是 0.5 s —— 该话题实测 1 Hz |
| `local_pos_timeout_s` | `0.5` | 上游 ~30 Hz |
| `mocap_timeout_s` | `0.5` | 上游 ~196 Hz |
| `rc_timeout_s` | `0.5` | 上游 10 Hz 级。★ 判据**只在已解锁时成立** —— 未解锁时遥控链路有没有都不影响安全，而首飞前遥控又是必须开机的 |
| `battery_timeout_s` | `2.0` | 上游 1 Hz 级。⚠️ **刻意不做判据**（只看值、不看新鲜度），理由见下文「失效降级」的末段 |
| `low_voltage_v` | `20.0` | 低电压阈值（**只看电压，不看电流**）。≈ 6S 的 3.33 V/cell，对齐 PX4 自己的 `BAT_LOW_THR`。⚠️ **换电池必须改**（阈值是按 6S 定的；4S/5S 都不合用），且 z0 的 `COM_LOW_BAT_ACT = 0` ⟹ PX4 只警告、得我们来做 |
| `rc_stick_ov_us_per_s` | `100.0` | 摇杆"在动"的判据 = **变化率**（µs/s），不是偏移量。对齐 PX4 的 `COM_RC_STICK_OV = 20`（0.2/s），按 z0 实测 `RC1_MIN/MAX` 换算 ≈102 µs/s。⚠️ 改了 PX4 那个参数必须同步改这里 |
| `mocap_stamp_stall_s` | `0.5` | 动捕**时间戳**连续不前进多久算"冻住"（话题层看不见的失效） |
| `rc_map_roll` … `rc_map_yaw` | `1/2/3/4` | 摇杆通道号（**1-based**）。⚠️ 必须与飞控上的 `RC_MAP_*` 一致 |
| `land_mode` | `AUTO.LAND` | 主动中止任务时请求的模式 |
| `own_arming` | `false` | ⚠️ **ticket 06 与 08 之间的临时缝**：本张没有解锁路径，于是"已解锁时的失效请求 `AUTO.LAND`"在台架上没法验；置 `true` = 把任何 `armed` 上升沿当作本节点发起的解锁。**只用于台架注入，实飞前保持 false，ticket 08 落地时删掉** |
| `*_topic` / `*_service` | mavros 2.15.1 默认名 | 多机/台架复用不改代码 |

## 与其它 ticket 的边界

| 本包**做** | 本包**不做**（归谁） |
|---|---|
| 守卫判定 → `INIT`/`STANDBY` | `TAKEOFF`/`HOVER` 的判据（ticket 08）、轨迹（07） |
| 50 Hz 流 setpoint、切 OFFBOARD | **解锁**（ticket 08）、任何位移 |
| 输入静默 / 低压 / 位置异常 ⟹ 停发 + `LAND` | `RTL` 等其它降级动作、以及"位置失效 ⟹ 停桨"（不归本包，见 ticket 16） |
| 打杆夺权 ⟹ 停发 + 交还（`RC_OVERRIDE`） | 打杆接管的**执行**在 PX4（`COM_OBL_RC_ACT`），本包只是不再抢 |
| 可复位闩锁 + `degrade=` 可检索日志 | 任何"清故障"命令 / 远程复位接口 |
| `/control/state` | `/control/traj_progress`（ticket 07） |

## 构建与测试

```bash
colcon build --packages-up-to control     # ★ 不要用 --packages-select：control test_depend 了 bench
colcon test  --packages-up-to control --executor sequential   # ★ 串行：并行跑包会在同一个 ROS 域里撞夹具
```

★ **跑测试要独占一个 ROS 域**（`src/bench/README.md`「安全边界」有完整事故记录）：
整仓 `colcon test` **默认并行跑包**，两个包同时用域 73 ⟹ 两边的假 mavros 互相往话题里灌帧 ⟹
本包的 DUT 看到 `armed` 抖动 ⟹ **假红**（实测：`test_control_standby` 9 条挂 5 条，单跑 9 条全绿）。
手动跑单文件时用 `BENCH_ROS_DOMAIN_ID=<非 0/73 的域号>`。

`control` 的测试分两层、四个文件：

| 文件 | 接缝 | 断言面 | 回答的问题 |
|---|---|---|---|
| `test/test_control_standby.py` | **seam 1**（话题边界） | 只断言外部可观测的话题 | 正常路径下飞机**发了什么** |
| `test/test_control_failsafe.py` | **seam 1**（话题边界） | 同上 | 失效时**停止**了什么、请求了什么模式 |
| `test/test_fsm.cpp` | **seam 2**（纯函数，零 ROS） | 零依赖，直接调 `fsm.hpp` | **是哪条守卫**放行的、降级怎么闩锁与复位 |
| `test/test_rc_stick.cpp` | **seam 2**（纯函数，零 ROS） | 零依赖，直接调 `rc_stick.hpp` | 摇杆**在不在动**（变化率判据）|

seam 1 的 DUT 是**子进程**（`control_node` 可执行文件），输入来自**假 mavros + 脚本化动捕**；
夹具在 `src/bench/`，用法见 `src/bench/README.md`。
**为什么两层都要**：四条守卫全失效在话题上的症状**完全一样**（没有 setpoint），
seam 1 到那里就没有诊断分辨率了；而"缺项清单是哪几条、顺序如何"是纯逻辑，在 seam 2 测，
失败信息直接指着那条守卫。

本包刻意**不**加 `ament_lint_auto`（沿用 `mocap` 的先例：全仓库既有 172 条 lint 失败全来自
vendored 的 `vrpn_listener`，新包再加 lint 只是增加噪声）。

覆盖的判据（seam 1）：四条守卫**逐条**缺失时不发 setpoint（每条都配了正控）、
外部解锁时停发、运行中 `/mavros/state` 静默后流在 `state_timeout_s` 内停止、前置流时长（两侧都断言）、
`enter_offboard = false` 只流不切、杀进程后流停止、以及冒烟用例对 setpoint 逐字段与频率的断言
（以上 `test/test_control_standby.py`）。

失效路径（seam 1，`test/test_control_failsafe.py`，13 例）：打杆 ⟹ setpoint 在 0.3 s 内停且
`RC_OVERRIDE` 且**不再请求任何模式**、外部切模式 ⟹ 同样不抢回、**交出即复位**、
三条输入超时各一条判据（`mocap`/`state`/`rc`，每条都断言 `degrade=` token 与 `AUTO.LAND` 请求）、
低压 ⟹ `LAND` 且**落地上锁才复位**（复位前一直请求、复位后 `degrade=none`）、NaN 位置 ⟹ 停发 + `LAND`、
未解锁失效 ⟹ 只停发且判据恢复**自动复位**、动捕戳**冻住** ⟹ 降级而**短促倒退不降级**、
戳**倒退到旧纪元并持续** > `mocap_stamp_stall_s` ⟹ 降级、
所有输入同时静默 ⟹ 节点不退出且报出最根本那条、通道数不足的遥控帧不崩、
未置 `own_arming` 时外部解锁 ⟹ 只停发不请求 `LAND`。

seam 2（`test/test_fsm.cpp`，42 例）：四条守卫**逐条**掀掉时缺项清单恰好是那一条（不是只断言"不放行"——
那样"某条守卫恒假"也会通过）、缺项顺序固定、全零 `Guards` 里"未解锁"成立（默认值的纪律）、
`STANDBY` 下控制权仍是 `MANUAL`、前置流时长两侧、已在 OFFBOARD 不再请求、**断流后前置流重新计时**；
以及降级路径的**每条分支**：原因优先级（同一帧里多个判据成立时谁说话）、未解锁失效不闩锁 ⟹ 自动复位、
已解锁失效闩锁 ⟹ **不因输入恢复而空中自动接管**、落地上锁才复位、**低压闩锁被夺权升级**、
夺权闩锁不被输入超时改写、自己请求的 `AUTO.LAND` 生效**不算夺权**、降级期间不抢回 OFFBOARD、
外部交回即复位、`rc_override` 与 `mode_lost` 的先后（打杆后模式才掉出来仍报打杆）。

seam 2（`test/test_rc_stick.cpp`，13 例）：变化率判据的边界 —— 第一帧不作判断、快速推杆/反向推杆算在动、
**慢速大行程漂移不算在动**、推到头保持不动会衰减到阈值以下、非正周期与 NaN 不作判断（且不污染状态）、
单帧越过阈值不算、默认阈值与固件 `COM_RC_STICK_OV` 同源。

⚠️ **没被自动化覆盖的一处**：`request_mode()` 的重试节流（重复请求抑制、悬空请求丢弃、
重试间隔）。它是**输出侧的发送节奏**而非输入判据，只有"切模式恰好失败/悬空"时才走到 ——
真台上的表现是"日志说在重试，实际没重试"，改动这段请连带跑一次台架。

## ⏳ 还没做的验收项（需要有飞控空闲的窗口）

| 项 | 为什么还没做 |
|---|---|
| 真 PX4 被拉进 OFFBOARD | `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` / 模式需求检查都是 **PX4 侧**行为，夹具不模拟。需要在**拆桨**台架上跑一次 |
| 杀节点 → PX4 在 `COM_OF_LOSS_T`（1.0 s）后自行退出 OFFBOARD | 同上；夹具这一半已由 `test_杀掉_DUT_之后_setpoint_流停止` 覆盖 |
| **打杆 → PX4 按 `COM_OBL_RC_ACT` 接管**（z0 现值 `0` = 回 Position 模式） | 同上：执行在 PX4。本节点这一半（立刻停发、退到 `RC_OVERRIDE`、**不抢回 OFFBOARD**）已由 `test_control_failsafe.py` 覆盖 |
| 低压判据面对的是**真电池**读数 | 台架注入的是假 mavros 的电压，判据逻辑已覆盖；**真实读数可不可信**是 ticket 01 的 ADC 判定（⚠️ 判据本身只看电压、不依赖电流） |
| ⚠️ **`NAV_RCL_ACT = 2`（Return）这条依赖** | 不是本包的实现项，是 `params-changes.md` **A1** 的阻塞项：A1 不落实，"RC 丢失超时 ⟹ LAND"之后 PX4 自己的 RC 失效保护仍会试图**返航** |
| **"EKF2 位置失效"的真载体** | 本链条上**抓不到**（第一手核实，见上文「失效降级」），真正的兜底是动捕超时；候选载体开在 ticket 16 |

做前两项时 **必须在台架上拆桨**，并按 `docs/control/phase1-implementation.md` §7 的顺序来。

## 目录

```
include/control/fsm.hpp           ★ 纯逻辑、零 ROS：两个正交维度 + 守卫清单 + 降级判据/闩锁（判据都在这里）
include/control/rc_stick.hpp      ★ 纯逻辑、零 ROS：摇杆"在动"的变化率判据（对齐 COM_RC_STICK_OV）
include/control/px4_interface.hpp mavros 边界：话题/服务名、超时口径、阈值依据、QoS 依据
src/px4_interface.cpp             订阅/发布/服务 + 每条输入的新鲜度与健康量
src/control_node.cpp              只做接线：参数、定时器、日志（★ 不要在这里加判断）
config/control.yaml               参数（含每一条的"为什么是这个值"）
launch/control.launch.py          一键启动（只起本节点）
test/test_fsm.cpp                 ★ seam 2：纯函数单测（零 ROS 依赖）
test/test_rc_stick.cpp            ★ seam 2：变化率判据单测（零 ROS 依赖）
test/test_control_standby.py      ★ seam 1：子进程 DUT + 假 mavros，只断言话题（正常路径）
test/test_control_failsafe.py     ★ seam 1：同上（失效路径）
```
