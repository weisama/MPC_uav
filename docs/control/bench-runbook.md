# 拆桨台架清单（Phase 1，**必须由人做**的部分）

> **这张表是什么**：`control` + `bench` 的 **171** 例自动化（`colcon test --packages-up-to control`）覆盖的是**本节点**
> 的行为（★ 逐文件计数与"为什么是这个数"见 [`phase1-implementation.md`](phase1-implementation.md) §7.1；
> 数目会随票漂，**别照抄这句**，要现数就 `grep -c '^TEST'` / `grep -cE '^(def test_|TEST)'`）；留在下面的每一项，**判据都在 PX4 一侧**（或需要真实的遥控器/动捕现场），夹具**不模拟 PX4**
> ⟹ 机器验不了。做一遍、把读数填进下面的表、回去勾掉对应 ticket 的框即可。
> ⚠️ **T4 是唯一的例外**：它的判据一半在我们这一侧（服务应答、`takeoff_origin=`、斜坡、`/control/state`），
> 自动化**已经**把这些验过了 —— 但它仍留在这里，因为"真 PX4 会不会按我们的解锁请求起飞""prearm 被拒时
> 报哪个 `MAV_RESULT` 码"**只有真飞控能给**，夹具的 `armed` 是我们自己置的。
>
> **不在本清单里的**：验收项 ④「EKF2 位置失效 ⟹ `LAND`」。原先它的判据在本链路上**恒假**
> （`position_invalid` 那条 NaN 判据在真机上永不触发）⟹ [ticket 16](../../.scratch/phase1/issues/16-ekf2-position-failure-carrier.md)
> **2026-09-22 已结清**：载体换成 `/mavros/estimator_status` 的 `pos_horiz_abs`（`ekf2_pos_lost`），
> **自动化**已经覆盖（假 mavros 直接注入那一位，`test_control_failsafe.py` 四条 + bench 自检两条）。
> ⚠️ 但**真台架上仍然看不到它单独触发**：停掉动捕会让 0.5 s 的 `mocap_timeout` 先成立 ⟹
> 报出来的是动捕那条。所以别在本清单里找它 —— 它属于"自动化能验"的那一半，不是 PX4 侧行为。

★ **要看"现场怎么把一次飞行从头到尾做完"**（遥控各通道干什么、FSM 怎么走、起链发哪几条命令、
每一步怎么判读、出事了先看哪）：[`operation_manual.md`](../../operation_manual.md)。
⚠️ 那是**导航图**，**不替代本清单** —— 本清单是**拆桨台架**的判据与读数记录（要填表、要回勾票），
那一页是**飞之前**该按的顺序与判读。

## ★ 2026-09-28 操作者裁定：**下一次台架日 = 这张表全跑一遍**（含阶梯的处置）

**背景**：2026-09-28 已经**自由飞过一次**（三条轨迹，目测正常，见 [`operation_manual.md`](../../operation_manual.md) §8.1），
但**本清单的框一个都没勾过**（T1–T8 全是空的），[`04`](../../.scratch/phase1/issues/04-mocap-relative-pose.md)/
[`05`](../../.scratch/phase1/issues/05-control-offboard-skeleton.md)/
[`06`](../../.scratch/phase1/issues/06-control-failsafe-paths.md)/
[`13`](../../.scratch/phase1/issues/13-mavros-bench-vision-pose.md) 里那批读数框也照旧空着。
⟹ **裁定：这些读数要补，下一次台架日一次性全跑、逐条回勾。**

| 项 | 裁定 | 备注 |
|---|---|---|
| **本清单 T1–T8** | ⏳ **要补，一次全做完**（这次会话就是它） | 含 **T4 的跳变量**（09-22 只观测到过"越阈值"那一次 `0.5738 m`，修法在 T4 正文）、**T5 断流**、**T6 打杆接管**、**T7 输入坏** |
| **ticket 04 的两个框**（QGC 定位稳定 / 坐标系确认实验） | ⏳ **由 T1 + T2 承接** | 它们本来就是 T1/T2 的落点 |
| **验收阶梯 1–3 级**（系绳 20 cm / 系绳悬停 30 s…） | ✅ **不再补做**（操作者原话：*"后续自由飞都测试过足以说明 1–3 已经实现，无需测试"*） | ⚠️ **是裁定，不是"做过了"** —— 1–3 级的读数**仍然没有**，别当证据用 |
| **A1** | ✅ **已落地** | ★ 2026-09-28 实测 `NAV_RCL_ACT = 1`，P0 表里那行已经不用改 |

★ **一次会话尽量全做完**：起链、搬飞机、解锁这些准备工作占大头，下面 7 项共用同一次准备。
（步骤顺序是刻意的：T1/T2/T3 必须在**解锁之前**做，T5/T6/T7 必须在**解锁之后**做。
**T4 是全程唯一的解锁动作**（ticket 08 起解锁只能由**程序**发起 —— 现场按开始键（CH9 第 1 次
上升沿），**拆桨台架上走 `/control/takeoff` 服务**，见 T4）。）

> ⚠️ **本清单只覆盖 ticket 08 验收阶梯的第 1 级**（拆桨台架：解锁 + 进 `OFFBOARD` + 爬升斜坡正确）。
> 阶梯的第 **2/3/4** 级（系绳离地 20 cm / 系绳闭环悬停 30 s / 自由飞轨迹 1）**要装桨**，
> 不属于"拆桨台架"这张表，判据见 [ticket 08](../../.scratch/phase1/issues/08-takeoff-hover-land.md) 的验收阶梯。
>
> ⚠️ **ticket 09 的 `TRACK` 也不在这张表里**（它接在 `HOVER` 之后，而拆桨态连 `HOVER` 都到不了 ⟹
> 台架上**没有**"进 `TRACK`、飞轨迹、收口"可看）。`TRACK` 的判据全在两层自动化测试里
> （`control` 的 `test_control_track.py` + `test_fsm.cpp` 的 `Track` / `StartEdge` 两组），
> 实飞判读靠固定形状的日志 token（`遥控器开始键：` / `TRACK 进行中：` / `end_reason=`），
> 见 `src/control/README.md` 的「日志里怎么复核」。
> ★ 但**开始键的通道核对（H.8）要在解锁之前先做** —— 它只需要遥控链路和 `/mavros/rc/in`，
> 不需要离地：通道错了的话，第 1 次上升沿会直接去解锁起飞。

---

## P0. 前置（缺一不可）

### 物理与安全

- [ ] **拆桨**（本清单全程无桨；`phase1-implementation.md` §7 步骤 3 的"别装桨！"）
- [ ] 飞机**摆好、静止、固定**（动捕原点与"稳定判据"都以此为前提；T4 尤其要求静止）
- [ ] 遥控器**开机、链路已建立**，且**在你手边** —— `failsafe.cpp:340-357`：**解锁那一刻 RC 就是丢的**，
      PX4 会把 manual control 当"可选"、**忽略 RC 丢失保护**直到 RC 重现（`params-changes.md` §H.7.1）
- [ ] 知道 **ch8 是硬件杀车**（停桨）：本清单不碰它，但它是唯一的人手急停
- [ ] ⚠️ **拆了桨 ≠ 电机不转**：`OFFBOARD` / `POSCTL` 下 PX4 会试图保持位置 ⟹ **电机会转**。
      拆桨只是让它不产生推力；**别把手、电池线、动捕线放在桨座附近**（转起来的电机能卷住线）
- [ ] 电池、QGC 链路（ticket 13 的 WiFi 桥：`gcs_url:=udp-b://0.0.0.0:14555@:14550`）
- [ ] ★ **飞控没被 `crash dumps present on SD` 拦住** —— SD 卡根目录若有 `fault_*.log`，
      PX4 会**拒绝解锁**（`vehicle needs service`），且**删档不会当场生效**。
      实测纪律（2026-09-22）：`07:52–07:59` 删掉转储后，飞控到 `08:02:46` **仍在报**同一条，
      直到 `sess259`/`sess260` **两次开机之后**，`08:17:52` 的解锁才通过。
      ⟹ **删掉转储后要重启一次飞控（或至少再上电一次）再试解锁**。
      ⚠️ 机制**未验**（候选：判据在开机时锁存 / FATFS 目录项被缓存在 RAM），**只作操作纪律**。
      查法：`python3 /tmp/z0_ftp_ls.py /fs/microsd`（无 `fault_*` = 干净）。
      ⚠️ 转储**同名覆盖**（RTC 无效 ⟹ 每次故障都写 `fault_2000_01_01_00_00_01.log`）
      ⟹ "只有一个 fault 文件"**推不出**故障次数。

### 参数：先**记录现值**，再改 A1

在 QGC 里逐个读出来填进下表（**改任何一个之前先记原值**，收尾要能恢复）：

| 参数 | 期望现值 | 与本清单的关系 | 实测值（填） |
|---|---|---|---|
| `NAV_RCL_ACT` | ✅ **已是 `1`（Hold）** —— ★ 2026-09-28 实测（`ros2 param get`）⟹ **A1 已落地，本行不必再改** | 原为 **A1 首飞阻塞项**（室内无 GPS 时 `2` 会去**返航**）。☠️ 本行此前写"期望现值 `2` → 改成 `1`"，**那是过时的**：13/15 早在 09-22/23 就记着已生效 | `1`（2026-09-28） |
| `COM_OBL_RC_ACT` | `0`（= Position 模式） | ★ **T6 判的就是它** | |
| `COM_RC_OVERRIDE` | `3`（刻意放宽） | T6 的**另一条**机制（见 T6 的判别跑） | |
| `COM_RC_STICK_OV` | `20`（= 归一化 `0.2/s`） | 与我们的 `rc_stick_ov_us_per_s = 100` **同源**（`config/control.yaml:68-73`） | |
| `COM_OF_LOSS_T` | `1.0 s` | ★ T5 的判据就是这个数 | |

> ★ **另有 3 条参数「已裁定、2026-09-23 已写入并读回」**（ticket
> [15](../../.scratch/phase1/issues/15-fcu-param-reconciliation.md) §四 的人裁定，2026-09-22）：
> `EKF2_GPS_CTRL` `7 → 0`、`SYS_HAS_GPS` `1 → 0`、`MPC_XY_VEL_MAX` `12.0 → 2.0`。
> 理由、命令与验收步骤见 [`params-changes.md`](../reference/params-changes.md) **§I.1**（用 `ros2 param set`，改完 re-dump 对账）。
> ⚠️ **它们不是台架 T1–T7 的判据** —— 只是**恰好共用这次上电**，别混进下面的验收表。
> ✅ 写入与读回记录见 §I.1；2026-09-28 复核三条仍在。

### ☠️ 已删：台架专用缝 `own_arming: true`（ticket 08 落地时删掉）

ticket 06 时期本张还没有解锁路径，于是有个参数 `own_arming` 把"任何 `armed` 上升沿"都当成本节点发起的解锁。
**ticket 08 加了 `/control/takeoff` 之后这个参数已删**（`px4_interface.cpp:134-152`）：解锁的归属改由
`/mavros/cmd/arming` 的**肯定应答** + 随后的第一个 `armed` 上升沿**真实追踪**（`arming_by_us_`）。

**这对台架操作的三个后果，别按旧笔记做**：

1. **没有命令行开关可选了** —— 解锁只有**程序**这一条路：现场按开始键（CH9 第 1 次上升沿），
   而**拆桨台架上用** `ros2 service call /control/takeoff`（T4）—— 台上没有遥控接收机时
   开始键那条路**走不通**（起飞前置里有一条"遥控在线"，§H.7.1，刻意如此）。
2. **`AUTO` 由应答决定，不由参数决定**：QGC 里手动解锁 ⟹ `armed` 为真而归属不是我们的
   ⟹ `guard_holds()` 的 `kArmingByUs = !armed || own_arming` **不成立** ⟹ 节点**停发并退回 `INIT`**
   （§H.6 的外部解锁语义，**刻意如此**）。想在台上拿到 `AUTO/STANDBY` 就必须走 T4 的服务。
3. ★ **重启节点不会"自动重新请求 `OFFBOARD`"**：`arming_by_us_` 是内存里的，重启即丢。
   飞机若**还解锁着**，新起的节点把它看成**外部解锁** ⟹ 停发。要继续做 T5/T6，**先在 QGC 里上锁**
   （或重跑 T4）—— 见 T5 的注意事项。

### 起链（顺序不能反）

> ★ **这一节的命令与核对已收进 [`run/z0.py`](../../run/z0.py) 的菜单**（3= mavros / 4= vrpn /
> 5= mocap / **2= 起链核对**；它自带你上面那三条 `source`）。★ 但**本表仍是判据与记录的权威** ——
> 脚本只把命令送出去、把读数摊开，**不含任何解锁/起飞入口**（见 [`run/README.md`](../../run/README.md)）。
>
> ★ **懒得一条条敲**：菜单 **9= 一键启动全部（起前选控制器；默认 offboard 控制）**（或 `run/up.sh`）按**本节这个顺序**
> 一次起齐（mavros 那条**自带 `gcs_url` 桥**），每步**等判据成立**再加下一步，起完把那张核对表摊开 ——
> 但它**不代做**"还差的那两步"（选轨迹、起飞 = 上面 T4 那条路），与上一段的边界一致。
> ★ 只想**手动遥控飞**（到 `mocap` 为止、不起 `traj`/`control`）：菜单 **8= 一键启动（动捕+mavros，手动遥控飞）**。
>
> ★ **停的时候**：那个终端还在就 `Ctrl-C`（脚本转发，先 SIGINT）；**终端已经关了**就用菜单
> **10= 清除后台节点（扫 mavros/vrpn/mocap/traj/control/mpc 并停掉）**（或 `run/down.sh`）—— 它**按名字扫**、
> 先列清单再问一句 y/N。
> ⚠️ 那是**停进程、不是降落**，台上要先把桨拆了、飞机固定好。

```bash
source /opt/ros/humble/setup.bash && source ~/mavros_ws/install/setup.bash && source ~/ros_ws/install/setup.bash
ros2 launch bringup mavros.launch.py fcu_url:=/dev/ttyTHS1:921600   # ① 串口定案见 ticket 13/14
# ☠️ 2026-10-13（票 13 第九轮）：**这一行从 `ros2 launch mavros px4.launch …` 改过来的**。
#   上游 `px4.launch` 把 `config_yaml` 写死，而 `plugin.cpp:35` 的 `use_global_arguments(false)`
#   让那份 yaml **到不了插件节点** ⟹ `thrust_scaling` 停在声明默认 `NAN` ⟹ 凡 `thrust != 0`
#   的 setpoint **被 mavros 整帧丢掉**（连 `body_rate` 一起）—— 零操纵，且只打一句 5 s 节流的 ERROR。
#   本仓那条包装多挂了一个 `mavros_set_thrust_scaling.py`（设完**读回核对**）。
#   ⛔ **照着旧命令敲 = 悄没声地把整条下行关掉**；起完可以这样核：1.0 才是对的
#      `ros2 param get /mavros/setpoint_raw thrust_scaling`
ros2 topic hz /mavros/state                                  #    1 Hz 是**正常**的
ros2 launch vrpn_listener sync_entity_state.launch           # ② 刚体名与 mocap.yaml 对齐
ros2 topic hz /vrpn/<刚体>/pose                              #    ~200 Hz
ros2 launch mocap mocap.launch.py                            # ③ 起之前飞机已摆好静止
ros2 topic hz /mocap/pose_enu                                #    有数
```

⚠️ **串口归属**：`/dev/ttyTHS1` 只能有一个 mavros。别让另一个会话占着（ticket 14）。
★ **③ 会在 T3 之前被重起一次**（不是笔误）：T1/T2 要搬飞机，而阶段 A 原点在节点启动时就锁定了
⟹ **重起它，让阶段 A 原点 = 起飞点**（重起后别再碰飞机）。见 T4 的 ★ 那条。
⚠️ **别在 domain 0 上跑夹具**（夹具**拒绝** domain 0 —— 那里跑着真 mavros + 真飞控，见 ticket 05
与 [`src/bench/README.md`](../../src/bench/README.md) 的「安全边界」）。

---

## T1. 动捕链路目视（勾 ticket 13 的 ①）

| | |
|---|---|
| **做** | 未解锁，QGC → MAVLink Inspector |
| **判据** | `VISION_POSITION_ESTIMATE` **有数且不跳**；`LOCAL_POSITION_NED` **稳定**（不用解锁、不用飞） |
| **记录** | 截图或读数区间；`EKF2_EVP_NOISE` 现值 —— ★ **2026-09-28 实测已是 PX4 默认 `0.1`**（09-21 读到的是 `0.06096`，操作者确认那次之后被人为改过，见 [`params-changes.md` §I.1′](../reference/params-changes.md)）。**照实读、照实记，别照抄 `0.061`** |
| **卡住看哪** | `mocap` 节点日志 / `progress` 字段；数值跳 → 动捕 WiFi 或 `EKF2_EVP_NOISE` |

## T2. 坐标轴确认（勾 ticket 13 的 ⑥ = §7 步 1b/1c）

| | |
|---|---|
| **做** | ★ **两条路任选一条即可收尾**（[ticket 13](../../.scratch/phase1/issues/13-mavros-bench-vision-pose.md) 的 ⑥ 已把上游证据做掉，**剩下的只是 QGC 里的轴标签**）：**(a)** 飞机**保持不动**，直接读当前 QGC `LOCAL_POSITION_NED` 的 `x` 是不是 **≈ +0.59**（`y ≈ +0.014`、`z ≈ +0.002`）—— 那是"人沿动捕 **+y** 挪了 59.4 cm"在 NED 里应有的样子（`transform_frame_enu_ned`：ENU `y` → NED `x`）；**(b)** 再按原描述向动捕 **+x（机头方向）推 0.5 m**，看 QGC 的 `y` 变 0.5。两种都再做一次**升高 0.3 m** |
| **判据** | (a) QGC 的 `x ≈ +0.59`（**不是 `y`**）；(b) QGC 的 `y` 变 0.5；(a)(b) 共同的：升高时 QGC 的 `z` 变 **−0.3**（NED 向下为正 ⟹ 读数变小 = 变高） |
| **记录** | 选的哪条路 + 三个读数（一张截图对比足够） |
| **卡住看哪** | 若变的是"错的那一轴" ⟹ 轴向约定反了，**别继续往下做**（T6 打杆时会把杆的语义也搞错）；原始读数在 `ros2 topic echo /mavros/local_position/pose` 上看不到这一层 —— 它验的正是 ROS 话题看不到的那一段 |

★ **为什么非做不可**：飞机"往反方向飞"时，你分不出是原点锁错、yaw 归零错、还是 mavros 的边界转换不如预期
（ticket 13 的"为什么这两条非做不可"）。这是全链路里**唯一不由本项目代码控制**的那一段。

> ⚠️ **顺手看一件没解的事（不是本清单的验收项）**：若 `/mocap/pose_enu` 上**有帧**、
> 而 `mocap` 节点日志显示输入正断着（它那时没有任何发布路径），那就是 2026-09-22 出现过一次、
> **未复现**的「来历不明的发布者」。当时排不出来，是因为日志只写"发布者：N 个（QoS）"、
> **不写是谁** —— 现在 `control` 的诊断会打出**节点名**（`发布者：1 个（/mocap_rel_pose [reliable]）`），
> `ros2 topic info -v /mocap/pose_enu` 也能直接指认。⟹ **看到就记一行带时间戳的原文**，
> 下次就不会再是一个悬案。它在 `control` 里的影响只有一条："动捕有数据"这个**守卫**可被它满足
> （位置来自 EKF2，控制律不受影响）。

## T3. 真 PX4 被拉进 `OFFBOARD`（勾 ticket 05 的 ①）

| | |
|---|---|
| **做** | ★ **先确认飞机是上锁的**（QGC 或 `/mavros/state` 的 `armed == false`；还解锁着的话本项做不到 —— 见上面那节第 3 条）。然后起 `control` 节点：`ros2 launch control control.launch.py`；另开一个终端 `ros2 topic echo /control/state`。★ **不要**调 `/control/takeoff` —— 这一项只验"未解锁时能不能进 `OFFBOARD`" |
| **判据** | ① 日志报守卫逐条成立；② `ros2 topic hz /mavros/setpoint_raw/local` ≈ **50 Hz**；③ 前置流满 `offboard_prestream_s`（1.0 s）后节点请求 `OFFBOARD`；④ `ros2 topic echo /mavros/state` 的 `mode` == `OFFBOARD` |
| **记录** | ★ **未解锁时 PX4 接不接受 `OFFBOARD`**（模式需求检查在 PX4 里，这是本次要回答的问题之一）；从"开始流"到"模式真的变成 OFFBOARD"的时间 |
| **卡住看哪** | 节点日志的守卫清单（它自己会说缺什么）；`mode` 一直是 `POSCTL` ⟹ 看 PX4 拒绝的原因（QGC 的 STATUSTEXT） |

★ 此时期望 `/control/state` = **`MANUAL/STANDBY`**（未解锁 ⟹ 控制权 `MANUAL`，`fsm.hpp:475`）。

## T4. `/control/takeoff` 程序解锁 ⟹ 锁原点 + 进 `TAKEOFF`（现场按开始键等价）
（**ticket 08 阶梯第 1 级**；[ticket 13](../../.scratch/phase1/issues/13-mavros-bench-vision-pose.md) 已于 2026-09-22 `done`
—— 本项若这次的**跳变量**没记，回填 [ticket 13](../../.scratch/phase1/issues/13-mavros-bench-vision-pose.md) 的 **§残留读数** 第 3 行）

★ **这是全程唯一的解锁动作**（ticket 08 起 QGC 手动解锁会被判成"外部解锁 ⟹ 停发"）。
★ 现场走的不是服务而是**开始键**（CH9 第 1 次上升沿，§H.5）；台架这一项之所以用服务，
是因为**台上没接遥控接收机**（起飞前置要"遥控在线"）⟹ 两条入口在 `Fsm` 里走的是**同一个
`request_takeoff`**，判据逐条相同。

★ **在 T3 起链之前，把 `mocap` 节点重起一次**（`Ctrl-C` 掉 `ros2 launch mocap mocap.launch.py`，
确认飞机**已摆好、静止**，再起；等日志打出「★ 阶段 A：启动原点已锁定」且 `/mocap/pose_enu` 有数 ⟹ 之后**别再碰飞机**）。
- **为什么**：**阶段 A 原点在节点启动时锁定**（30 帧均值），解锁上升沿才切到阶段 B（60 帧均值）。
  而 T1/T2 要求搬飞机（T2 的 ⑥ 就是"人挪 59.4 cm"）⟹ **走到这里时阶段 A 原点通常已经过期**；
  不重起的话两阶段切换量 > 0.05 m ⟹ 节点打 WARN，并把这**同量的位置跳变灌给 EKF2**（EKF2 当成真实运动）。
- **为什么放在 T3 之前而不是 T3/T4 之间**：T3 只是起链与进 `OFFBOARD`，**不动飞机**；
  夹在 T3 与 T4 之间重起会让 EKF2 在解锁前几秒丢掉视觉输入（台架上无害，但没必要）。
- ⚠️ **2026-09-22 实测**：未重起 ⟹ 切换量 **`0.5738 m`**（`mocap_rel_pose_node` 日志 WARN 原文见
  [ticket 13](../../.scratch/phase1/issues/13-mavros-bench-vision-pose.md) §实测 2026-09-22 §阶段 B）。

| | |
|---|---|
| **做** | 承接 T3 的状态（未解锁 + `OFFBOARD` + `STANDBY` + 正在流），然后：`ros2 service call /control/takeoff std_srvs/srv/Trigger` |
| **判据** | ① 服务应答 `success: true` **且有 `message`**（◎ 被拒时 `message` 要能说出缺什么）；② 日志 `MAV_RESULT=0`（`ACCEPTED`，`common.h:1135`；**被拒时把这个码原样记下来** —— prearm 失败到底报哪个码，只有真飞控能给）；③ 日志 `takeoff_origin=(x, y, z)`，且**不等于动捕原点**（EKF2 局部原点是它自己的）；④ `/control/state` 变 **`AUTO/TAKEOFF`**；⑤ `ros2 topic echo /mavros/setpoint_raw/local` 的 `z` **线性上升、斜率 ≤0.5 m/s、封顶 1.0 m**，水平两轴钉在 `takeoff_origin` 上；⑥ `mocap` 节点日志打出「★ 阶段 B … 切换发生在此刻」，**切换瞬间位置跳变 ≈ 0**（> 0.05 m 会同时 WARN） |
| **判据（止于此）** | ⚠️ **无桨 ⟹ 没有推力 ⟹ 飞机一直在起飞原点上，`/control/state` 永远停在 `AUTO/TAKEOFF`**。`TAKEOFF→HOVER` 的判据 `\|z − (原点+1.0)\| < 0.10 m` **按字面不可能成立** ⟹ **本项不判 `HOVER`**，那是阶梯第 2/3 级（要装桨）。别在台架上等 `WAIT_START` |
| **记录** | `MAV_RESULT` 码；`takeoff_origin` 三个数；`z` 斜坡的终点与用时；★ **跳变量（m）—— 期望 ≈ 0**（越 0.05 m 时节点会打 `⚠️ 两阶段原点切换量 … 超过阈值`，把那一行原样记下来；台架至今只观测到"越阈值"那一情形，见 [ticket 13](../../.scratch/phase1/issues/13-mavros-bench-vision-pose.md) 的 §残留读数）；`armed` 上升沿与"进 `OFFBOARD`"的时刻 |
| **卡住看哪** | 服务被拒 ⟹ 看 `message` 缺哪一条前置（遥控在线是最常见的：台上没接接收机时**永远解不了锁**，刻意如此）；`MAV_RESULT=4` ⟹ `FAILED`，看 QGC 的 STATUSTEXT |

## T5. 杀节点 ⟹ PX4 在 `COM_OF_LOSS_T` 后**自行退出** `OFFBOARD`（勾 ticket 05 的 ②）

| | |
|---|---|
| **做** | `Ctrl-C` 杀掉 `control` 节点；`ros2 topic echo /mavros/state` 盯着 `mode` |
| **判据** | 流停后 **≈1.0 s**（`COM_OF_LOSS_T`）`mode` 从 `OFFBOARD` 变成 `POSCTL` |
| **记录** | 两个时刻与其差（应 ≈1.0 s，**这是 PX4 侧的数字**，夹具永远造不出） |
| **卡住看哪** | 不退 ⟹ `COM_OF_LOSS_T` 现值不是 1.0？；`setpoint_raw` 还有别的发布者在发？ |

> ⚠️ **做 T6 之前必须先上锁**：杀节点不改飞机的解锁状态，而新起的节点**不再认**这次解锁
> （`arming_by_us_` 随进程一起没了 ⟹ 判成外部解锁 ⟹ 停发，见上面那节第 3 条）。
> 所以 T5 之后**在 QGC 里上锁**，再按 T3 → T4 的次序重来一遍（T4 会再把原点锁一次）。

## T6. 打杆 ⟹ 我们停发 + **PX4 按 `COM_OBL_RC_ACT` 接管**（勾 ticket 06 的 ① 后半）

| | |
|---|---|
| **做** | 按 T5 的注意事项**先上锁**，再重起节点并按 T4 用 `/control/takeoff` 重新解锁（★ 不要指望"起完自动重新请求 `OFFBOARD`" —— 那是 `own_arming` 参数时代的行为）。然后**快速推杆**：**半秒内推到半行程以上**（判据是变化率 `100 µs/s`；z0 半行程 511 µs ⟹ 推到底只要 >5.1 s 才不触发。**推到头后保持不动不触发**，变化率为 0） |
| **判据（我们这一半）** | ≤0.3 s 内 **停发 setpoint**（`ros2 topic hz` 归零）；`/control/state` 变 **`MANUAL/RC_OVERRIDE`**；此后**不再请求任何模式**（日志 `degrade=rc_override`） |
| **判据（PX4 那一半）** | `mode` 由 `OFFBOARD` **离开** ⟹ 按 `COM_OBL_RC_ACT = 0` 落回 **`POSCTL`（Position）** |
| **记录** | 两个时刻（我们停发 / PX4 换模式）与其差；打杆前 `mode` 的确认值 |

★ **⚠️ 这一格会混进两条机制**：`COM_RC_OVERRIDE = 3` 让 PX4 **自己**在打杆时就接管；我们停发之后
PX4 的 **offboard 断流保护**（`COM_OF_LOSS_T` 1.0 s → `COM_OBL_RC_ACT`）也会指向同一个结果
（`params-changes.md` §E 的 (a) 段）。**要分开**：临时把 `COM_RC_OVERRIDE` 置 `0` 再跑一次 ——
那次只剩"我们停发 ⟹ 1.0 s 后 `COM_OBL_RC_ACT`"这一条，两次的时间差就是两条机制各自的贡献。

## T7. ★ 输入坏掉 ⟹ **`mode_lost` 收窄**的真机现场（ticket 06 §4.4(1)）

这是本清单里**唯一一条"先在台架上验一个预测"**的项：收窄的判据（"上一帧模式是我们的 **且** 上一帧我们在流"）
在夹具上**永远造不出**，因为假 mavros 不假扮 PX4 的 `COM_OF_LOSS_T` 退出。

| | |
|---|---|
| **做** | 在 `OFFBOARD` + 已解锁 + 正在流的状态下，**停掉上游动捕**（`Ctrl-C` `vrpn_listener`，或直接停 `mocap` 节点），先 `ros2 topic hz /mocap/pose_enu` **确认它真的停了**（正控） |
| **判据** | ① 我们 0.5 s 的 `mocap_timeout` 先成立 ⟹ 停发 + 请求 `AUTO.LAND`，日志 `degrade=mocap_timeout`；② ≈1.0 s 后 PX4 **自己**退出 `OFFBOARD`；③ ★ **日志里不应出现 `degrade=mode_lost`**，`/control/state` **不应**变成 `MANUAL/RC_OVERRIDE` |
| **记录** | 三条各一条证据（时刻 + token + `/control/state`）；**若第 ③ 条不符 ⟹ 是判据错了，回来改 `first_degrade` 并补 seam 2 用例** |
| **卡住看哪** | 若 PX4 接受了 `AUTO.LAND`（台架上本来就在地面）可能**立刻落地上锁** ⟹ 闩锁复位、节点回 `INIT` —— 那本身是"可复位"的真机证据，记下来，别当成失败。★ 但这条复位要求上锁那一刻链路是**活着的**（`connected=true ∧ armed=false`，ticket 18 的「可信的落地上锁」，§4.4）：若那时 `/mavros/state` 已静默（mavros 发过它那帧合成断连帧），复位会推迟到**链路恢复之后**，别记成"没复位" |

★ 为什么断言"**不**应出现 `mode_lost`"：我们一 `halt()`，`prev_.streaming` 就转 false，
PX4 那次退出**不是**"模式被从我们手里夺走"（`fsm.hpp` 的 `first_degrade` 与 §4.4(1)）。
误判的代价是：`is_takeover()` 会把已闩锁的输入失效**升级成夺权** ⟹ **从此不再重发 `AUTO.LAND`**。

## T8. 收尾

- [ ] **恢复参数**：`COM_RC_OVERRIDE` 回 `3`（若为 T6 的判别跑改过）；`NAV_RCL_ACT` 保持 A1 的 `1`
- [ ] 落地上锁一次，确认闩锁复位（`/control/state` 回 `MANUAL/INIT` 或 `MANUAL/STANDBY`）。
      ★ 看的是上锁那一下 `/mavros/state` 上 `connected=true ∧ armed=false`（**可信的**降沿，ticket 18）——
      链路断过、mavros 已发过合成断连帧的场面里，复位要**等链路恢复**才发生
- [ ] 把上面各表的"实测值"回填到 **ticket 05 / 06 / 08 / 13**，并勾掉对应框
- [ ] 若飞控在录 ulog：把 `.ulg` 存档（判据全在 PX4 侧 ⟹ **它才是硬证据**，mavros 只是观察窗）

---

## 证据要落在哪一侧（★ 这条决定"验过了"算不算数）

T5/T6/T7 的判据**全部在 PX4 里**：

| 证据源 | 强度 | 说明 |
|---|---|---|
| **飞控自己的 ulog**（`SDLOG_MODE` / SD 卡） | ★ 硬证据 | 模式切换、failsafe 的**原始**时间线 |
| QGC 的模式时间线 + STATUSTEXT | 中 | 人工对时，能勾框，但不好复核 |
| 我们的 `degrade=<token>` 日志 / `/control/state` | 只证"我们这一半" | 证明不了 PX4 因此做了什么 |

★ 没有 ulog 时，**至少**要同时留下"我们这一侧的时刻"与"QGC 那一刻的 `mode`"，两条对得上才算过。

## 附：这些框各自属于哪张票

| 本清单的项 | 关掉哪张票的验收 |
|---|---|
| T1 / T2 / T4 | [ticket 13](../../.scratch/phase1/issues/13-mavros-bench-vision-pose.md)（✅ **`done`**：① ⑥ 与阶段 B 已于 2026-09-22 达成；★ 但 T4 的「跳变量（m）」当时未记 ⟹ 飞行日跑 T4 时顺手填，见 [ticket 13](../../.scratch/phase1/issues/13-mavros-bench-vision-pose.md) §实测 2026-09-22 §阶段 B） |
| T3 / T5 | [ticket 05](../../.scratch/phase1/issues/05-control-offboard-skeleton.md) 的两条遗留 |
| T4（解锁 + 锁原点 + `TAKEOFF` 斜坡） | [ticket 08](../../.scratch/phase1/issues/08-takeoff-hover-land.md) 的**验收阶梯第 1 级**（阶梯第 2/3/4 级要装桨，不在本清单） |
| T6 / T7 | [ticket 06](../../.scratch/phase1/issues/06-control-failsafe-paths.md) 的验收项 ① 后半（④ 见 ticket 16） |
| A1（`NAV_RCL_ACT = 1`） | ✅ **已落地**（2026-09-28 实测）—— [`params-changes.md`](../reference/params-changes.md) §A 的首飞阻塞项，**不必再做** |
| **T1 / T2 / T4 的「记录」列里还没填的三处读数**（① 的"不跳"、⑥ 的三个 Δ、T4 的跳变量） | [ticket 13](../../.scratch/phase1/issues/13-mavros-bench-vision-pose.md) 的 **§残留读数** —— 13 已 `done`，这三处是**不藏在 `done` 下面**的残留（`done` 的语义见 [`triage-labels.md`](../agents/triage-labels.md)） |
