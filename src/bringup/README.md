# `bringup` —— z0 的启动层

**本包不含任何飞行逻辑。** 判据在 `control`、轨迹在 `traj`、相对位姿在 `mocap`；
这里只回答一件事：**按什么顺序、用什么参数，把这几个进程摆到台上**，外加一个操作员用的交互脚本。

```
ros2 launch bringup z0.launch.py          # 一键：mocap + traj + control（+ 可选 vrpn_listener / mavros）
ros2 run    bringup fly.py                # 操作台：选轨迹 1/2/3 + 实时看状态与完成度
ros2 launch bringup mavros.launch.py      # 单独起 mavros（上游 px4.launch 的薄包装）
```

★ **懒得手打上面这几条**（以及 mavros/vrpn/mocap 分开起、起完核对）：[`run/z0.py`](../../run/z0.py)
是给人工用的启动菜单（用法与排障见 [`run/README.md`](../../run/README.md)）—— 它带 `source` 前缀、
每步给读数、并且**不含任何解锁/起飞入口**（与下面的「起完之后**还差两步**」一致）；
`use_vrpn:=false` 这条**它在起之前会问一句**。本文件仍是参数的权威出处。
★ 要**一条命令起齐**（mavros 带 GCS 桥 → vrpn → mocap → traj + control，顺序起）：菜单 **10= 一键起链**
或 `run/up.sh` —— 它同样**不含**下面那两步的入口。

## 一键 bringup

| 参数 | 默认 | 说明 |
|---|---|---|
| `use_mavros` | `false` | 连 mavros 一起起。⚠️ **默认不起**，见下 |
| `use_vrpn` | `true` | 起 vendored 的 `vrpn_listener`（动捕数据源）。已单独起过就设 `false` |
| `fcu_url` | `/dev/ttyTHS1:921600` | mavros 的飞控串口（只在 `use_mavros:=true` 时有意义） |
| `gcs_url` | 空 | 把 MAVLink 桥给笔记本上的 QGC，例如 `udp-b://0.0.0.0:14555@:14550`（实测见 [`docs/reference/mocap-setup.md`](../../docs/reference/mocap-setup.md) §5.4） |
| `namespace` | 空 | 多机时给 mocap/traj/control 一起加命名空间 |
| `mocap_params` / `traj_params` / `control_params` | 空 | 空 = 用各自包里那份 `config/*.yaml`；给了就往下传 |

其余参数表（话题名、阈值、闸门…）**不在本文件里**，在各自包的 `config/*.yaml` 与 README。
本文件只 include 它们**自己的** launch，不重复、也不覆盖它们的默认值。

### ★★ 它不保证启动顺序 —— 顺序的保证在**设计**里，不在 launch 里

ROS 2 的 launch 把各 include 当成"同时"拉起（各是各的进程），而这几件事**对顺序都不敏感**：

- `control` 起早了 ⟹ 停在 `MANUAL/INIT`，日志**逐条列出缺什么**（飞控 / 动捕 / `local_position`），
  补齐之后自己往下走（`MANUAL/STANDBY` ⟹ 流 setpoint ⟹ 切 OFFBOARD）。**INIT 是个正常状态，不是故障。**
- `traj` 起早了 ⟹ 它一帧参考都不发，直到有人 `select` + `start`（`start` 是 `control` 进 TRACK 时自己发的）。
- `mocap` 起早了 ⟹ 它照旧在**自己启动那一刻**锁"阶段 A 原点"（见下）。

⚠️ **真正对顺序敏感的是唯一一件事，而它在 launch 之外**：

> **飞机必须在起 `mocap` 之前就摆好并静止。**
> 阶段 A 的原点在 `mocap` 节点启动时锁定，之后到"切两阶段原点"这段时间里**动了多少，就带上多少误差**。

所以正确用法是：**摆好飞机 → 起这一条 → 起完别再碰飞机**。

### ★ `use_mavros` 为什么默认 `false`

三个理由，都不是偷懒：

1. 它**独占** `/dev/ttyTHS1`（串口定案见 `docs/reference/mocap-setup.md` §6 那张表）。
   现场常见做法是它在专门的终端里长期跑着；再起第二个打不开串口，而那个报错长得像
   "launch 坏了"，不像"串口被占了"。
2. 它是最慢、最有状态的一环，"起来了没有"要靠人看一眼 `/mavros/state`
   （**1 Hz 是正常的**，见 [`../control/config/control.yaml`](../control/config/control.yaml) 里那段）——
   把它混在一条命令里，这个核对点就没了。
3. 它装在**另一个工作空间**（`~/mavros_ws`，源码构建）⟹ 找不到它时不能连累其余三个起不来。
   `use_mavros:=true` 而包里找不到 mavros 时，本 launch 会打印**怎么修**（`source ~/mavros_ws/...`
   或改用默认值），然后**照旧往下起** mocap/traj/control —— 此时 control 会一直停在 `MANUAL/INIT`。

### ★ `mavros.launch.py` 只是薄包装，且**没有** `config/mavros_z0.yaml`

`docs/control/phase1-implementation.md` 的目录图里画了一个 `config/mavros_z0.yaml` —— **没有写**。
因为上游 `mavros/launch/px4.launch` 里 `pluginlists_yaml` / `config_yaml` 是**写死的**
（`$(find-pkg-share mavros)/launch/px4_config.yaml`），没暴露成 arg ⟹ **注入不了我们自己的参数文件**。
z0 实际要改的两样（串口、GCS 桥）都是 **launch 参数**，不是 yaml 里的参数。
多一个"没人读的配置文件"比没有它**更难查** —— 现场会以为改了它就生效。这一条是**推翻文档结论**的地方。

## 操作台 `fly.py`

```
ros2 run bringup fly.py
   1 / 2 / 3 = 选轨迹    0 = 取消选择    h = 帮助    q = 退出    回车 = 立刻打印一次
```

只做两件事：发 `/traj/select`（`std_msgs/UInt8`）、显示 `/control/state` 与 `/control/traj_progress`
（外加 `/mavros/state` 的 armed/mode 作参考）。**它不发 `/traj/start`、不调起飞服务、不碰 setpoint**：

- `/traj/start` **由 `control` 在进 `TRACK` 那一刻自己发**（ticket 09 Comments ②）——
  参考时钟的零点必须与"开始飞"同一时刻；这里再发一次等于把正在跑的时钟归零。
- 起飞 = **遥控器开始键的第 1 次上升沿**（§H.5）。"开始键按下之前不发任何控制指令"是**刻意**的防线，
  给一个程序入口等于在墙上开一扇后门。

★ 显示粒度按**完成度**走（每 5%）而不是按时间：悬停 60 s 不会刷 60 行，6 s 的八字照样看得见它在动。

## 起完之后**还差两步**（刻意不自动化）

1. `ros2 run bringup fly.py` 选轨迹（1/2/3）。
   ⚠️ 轨迹**必须在起飞前选好**；运行中重选也支持（`control` 会保持上一个目标并按 `traj_stream_gap_s`
   的闸门重发 `/traj/start`），但那是有意的例外，不是常规流程。
2. **遥控器开始键**：第 1 次上升沿 = 解锁 + 起飞，第 2 次 = 进 `TRACK`（§H.5）。
   ⚠️ 上电即高**不算沿** —— 必须先看到低电平。现场动作要点：**每次压住 ≥ 0.3 s 再松**
   （200 ms 去抖 × 10 Hz 采样 = 恰好 3 帧，压得太准会有一半概率按不动）。

## 排障速查

| 现象 | 先看 |
|---|---|
| `control` 一直 `MANUAL/INIT` | 它的日志会逐条列出缺什么：`/mavros/state` 静默 / 动捕没来 / `local_position` 没动 |
| 没有任何参考流 | `traj` 是不是**没 select**；select 了但没 `start`（`control` 进 TRACK 才发） |
| `degrade=traj_timeout（持续 …）` | 参考流断了超过 `traj_timeout_s`（默认 3 s）——看 `traj` 活着没有、有没有被重选 |
| 飞机位置整体偏一截 | 起 `mocap` 之前飞机就在动（阶段 A 原点漂了），见上面那条 ⚠️ |
| 遥控器按了没反应 | 开始通道是不是 CH9（§H.1）；`ros2 topic echo /mavros/rc/in` 看 `channels[8]` |
