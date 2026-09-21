# 12: `Z0` 刚体的动捕数据流确认（ticket 03 的残留项）

Status: ready-for-human
Type: task

**What to build:** 让 `/vrpn/Z0/pose` **真的有数据**，并把 ticket 03 的速率验收对 **z0 本体**补齐。

**为什么单独一张：** [ticket 03](03-vendor-vrpn-listener.md) 的五条验收里，第 3 条
（`ros2 topic hz /vrpn/<刚体>/pose` 稳定 100–400 Hz）**只在 `U_Tracker0` 上验到了 199.9 Hz** ——
**`Z0` 当前一次回调都没有**。链路、服务器、传输层全都正常（独立 VRPN 客户端实测
`robot0`/`U_Tracker0` 各 961 次 / 5 s），**缺的是"这个刚体正在被追踪"这个物理条件**。

**这张不是代码问题，是人在动捕软件/棚里的动作**，所以 `ready-for-human`。

**Blocked by:** 无（ticket 03 已 `done`）

## 已知的第一手事实（2026-09-21，ticket 03 实测）

| 项 | 读数 |
|---|---|
| sender 名单里有 `Z0` | ✅ 在（还有 `Z0_Marker1`…`Z0_Marker5`，**单颗球各自也是独立 sender**） |
| `Z0` 的 pose 回调 | ❌ **5 s 内 0 次** |
| 同一次运行里的对照组 | ✅ `robot0` 961 次、`U_Tracker0` 961 次（≈192 Hz） |
| 速率上限本身 | ✅ 已达标本张要求（`ros2 topic hz` 实测 **199.9 Hz**） |

**推断（未第一手证实）**：刚体在 Nokov 项目里**存在**（所以名字能被广播），但**没有被追踪/没有开流** ——
最可能是**飞机此刻不在有效捕捉范围内**。也可能是该刚体未启用输出、或反光球被遮挡/未建全。
**这三种要人到现场分辨，agent 分不了。**

## 验收项

- [ ] `Z0` 的 5 颗反光球在动捕棚内**可见且被追踪**（Nokov 界面里刚体状态正常、不发灰）
- [ ] Nokov 侧 `Z0` 的输出/streaming **已启用**
- [ ] 下面这条命令稳定给出 **~100–400 Hz**（动捕 200 Hz 出包）：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 launch vrpn_listener sync_entity_state.launch &
ros2 topic hz /vrpn/Z0/pose
```

- [ ] 记录 `ros2 topic echo /vrpn/Z0/pose --once` 的**原文**（`frame_id` 应为 `world`，`stamp` 应是**接收时刻**、且**每帧都变**）
- [ ] ★ **故意把刚体的名字记准**：若最终决定在 Nokov 里把小写 `z0` 用起来，**必须回头改
      [`mocap-setup.md`](../../../docs/reference/mocap-setup.md) §10 第 1 项与
      [ticket 04](04-mocap-relative-pose.md) 里的话题名** —— 话题名**原样取自 VRPN 的 sender 名**，大小写不归一化

## 为什么这条非查不可（不是"顺手")

- **ticket 04 的整张都在等它** —— 原点锁定、仅归零 yaw、两阶段切换，全部以
  `/vrpn/Z0/pose` 有数据为前提。没有数据，ticket 04 的验收项一条都做不了，
  而"没数据"和"我的相对位姿算错了"**在话题层面长得一模一样**。
- 这正是 `phase1-implementation.md` 反复强调的那类坑：**先确认输入存在，再谈处理对不对。**
