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

1. ★ **只装 `ros-humble-mavros` 不够，`vision_pose` 插件在 `mavros-extras` 里**
   （`phase1-implementation.md` §3.5 引的源码路径就是 `mavros_extras/src/plugins/vision_pose_estimate.cpp`）。
   ⟹ 两个包都要装。装完用 `ros2 param list /mavros` 里有没有 `vision_pose` 相关节点/插件来确认。
2. ⚠️ **喂视觉数据会立刻改变 EKF2 的行为**。`z0.params` 里 `EKF2_EVP_NOISE = 0.06096`，
   而用户笔记说"**0.1 都过小、会发散**"（`phase1-implementation.md` §6）。
   ⟹ **台架上第一次接 vision_pose 时，建议先把 `EKF2_EVP_NOISE` 抬到 1.0**（属参数核对清单，不在本张改）。
3. ⚠️ **Jetson 侧串口是 `ttyTHS1` 还是 `ttyTHS2` 至今未定案**（`mocap-setup.md` §10 第 2 项）。
   **判据是扫一遍找心跳，不靠猜**：`python3 tools/prereq-evidence/mavlink_probe.py --scan`。
   飞控侧已确认走 **TELEM2**（`z0.params` 的 `MAV_1_CONFIG = 102` 与操作者口径一致）。
4. ⚠️ **`/mavros/state` 的发布频率未核实**（[ticket 04](04-mocap-relative-pose.md) §七 第 4 条）。
   `mocap` 节点靠它的 `armed` 字段检**解锁上升沿**：**若它是 1 Hz，边沿最多晚 1 s 被检出**，
   阶段 B 的"起飞前静止帧"就有可能在爬升中被采到（节点会因此打 WARN、日志里能看出跳变）。
   ⟹ 本张用 `ros2 topic hz /mavros/state` **定案**，并把结论写给 [ticket 04](04-mocap-relative-pose.md)。

**Blocked by:** 无（04 的代码已就绪）；**被它阻塞的是** [ticket 05](05-control-offboard-skeleton.md)

## 步骤（在 z0 的 Jetson `orin-nx` 上）

```bash
# 1) 装 mavros（★ 系统改动，装前先跟用户确认）
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# 2) 串口定案（飞控上电后）
python3 ~/ros_ws/tools/prereq-evidence/mavlink_probe.py --scan

# 3) 动捕链路先通（顺序不能反：先确认输入存在，再谈处理对不对）
source /opt/ros/humble/setup.bash && source ~/ros_ws/install/setup.bash
ros2 launch vrpn_listener sync_entity_state.launch
ros2 topic list | grep vrpn        # ★ 现场核对刚体名，与 mocap.yaml 的 body_name 对齐
ros2 topic hz /vrpn/<刚体>/pose    # 应 ~200 Hz

# 4) mavros 起来（串口按第 2 步的结果填）
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS?:921600

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

- [ ] `ros-humble-mavros` **与** `ros-humble-mavros-extras` 都装上，`vision_pose` 链路可用
      （判据：`/mavros/vision_pose/pose` 上的消息被 mavros 收下 —— 节点侧日志无异常、QGC 有数）
- [ ] Jetson 侧串口**定案并记录**（THS1 / THS2 二选一，判据是 `mavlink_probe.py --scan` 的心跳）
- [ ] **①** 未解锁时 QGC 的 `VISION_POSITION_ESTIMATE` **有数且不跳**，`LOCAL_POSITION_NED` **已稳定**
      ⟹ 这才是"EKF2 在解锁前就已经收敛"的第一手证据（[ticket 04](04-mocap-relative-pose.md) 的验收项 ①）
- [ ] **⑥** 把飞机向动捕 **+x 方向平移 0.5 m** → QGC 的 **`y` 变化 0.5**；再**升高 0.3 m** → QGC 显示**向上**
      ⟹ 若变的是 `x`，**问题在 mavros 侧或参数侧**（`mocap` 节点已用假输入排除）
- [ ] 顺带记录 QGC 侧 `EKF2_EVP_NOISE` 的取值与现象（是否发散），供参数核对清单使用
- [ ] **`ros2 topic hz /mavros/state` 定案**（是不是 1 Hz ⟹ 解锁上升沿最多晚多久被检出），结论写回 04 §七
- [ ] **试一次解锁上升沿切原点**（QGC 里解锁、不起飞）：节点日志应打出「★ 阶段 B … 切换发生在此刻」，
      且**切换瞬间位置跳变 ≈ 0**（静止摆放的前提；> 0.05 m 会有 WARN）
- [ ] 把两条验收的读数**回填到 [ticket 04](04-mocap-relative-pose.md)**，并把 04 的 `Status` 改成 `done`

## 为什么这两条非做不可

`phase1-implementation.md` §7 的验证顺序里，**第 1 步与第 1b/1c 步就是它们**。
跳过它们的代价不是"少一个勾"，而是「**飞机往反方向飞**」时你分不出是原点锁错、yaw 归零错、
还是 mavros 的边界转换不如预期 —— 那正是 [ticket 04](04-mocap-relative-pose.md) 要单独做一张的原因。
本张验的恰恰是**最后一个环节**（mavros 的转换），它是全链路里**唯一不由本项目代码控制**的那一段。
