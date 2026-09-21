# 动捕链路搭建手册

> **来源**：本文整理自上游仓库 `Huan-qun/Sim2real-UAV-Params-` 的 `S2R/assets/doc/mocap.md`（594 行），
> 以及用户自己笔记 `~/weisama-log/25.06.09_UAV_mocap/` 中的实操记录。
> **适用范围**：**Nokov 动捕 → (VRPN) → 机载电脑 → (mavros) → PX4 EKF2** 这一条链路。
>
> 相关：[`px4-interface.md`](px4-interface.md)（接口事实）、[`params-analysis.md`](params-analysis.md)（飞控参数）、
> [`../PROJECT.md`](../PROJECT.md) 里程碑 1

> ⚠️ **上游是"多机集群 + 另一台机架"的配置**，z0 不能照抄。**第 10 节列出了 z0 必须改的地方。**

---

## 1. 数据流与话题

```
Nokov 软件 ──VRPN/UDP(3883)──▶ vrpn_listener ──▶ /vrpn/<刚体名>/pose
                                                      │
                                              vrpn_relative_pose   ← 减去上电时的初始位姿
                                                      │
                                                      ▼
                                          /mavros/vision_pose/pose ──▶ PX4 EKF2
```

**关键话题**：

| 话题 | 内容 |
|---|---|
| `/vrpn/<刚体>/pose` | 动捕原始位姿（`PoseStamped`） |
| `/vrpn/<刚体>/twist` | 动捕自算的速度（⚠️ **本项目不用**，见第 6 节） |
| `/mavros/vision_pose/pose` | 转发给 PX4 的相对位姿 |
| `/mavros/state` | 检查 mavros 连接状态 |

**上游的刚体命名是 `r11` / `r22`** —— 说明上游是**多机集群**（这也解释了上游文件 `uav1.params` 为什么叫 "Vehicle 1"）。**z0 只有一个刚体，应改成自己的命名。**

---

## 2. ★ 依赖：必须用**定制版** `vrpn_listener`

### 2.1 VRPN 库

> ☠️ **本节原教"源码编译 VRPN 到 `/usr/local`"，2026-09-21 已就地更正（ticket 03）。**
> **z0 上不用这条路** —— 走 apt，再补一条软链：

```bash
sudo apt install ros-humble-vrpn          # 7.35.0，装到 /opt/ros/humble
# ★ 只装 apt 那份编不过：vendored 的 CMakeLists.txt 里有一行裸名字的
#   target_link_libraries(vrpn_listener vrpn) → 链接器展开成 -lvrpn，
#   要求 libvrpn 落在 ld 默认搜索路径里，而 /opt/ros/humble/lib 不是。
#   那一行是 vendored 的、不能改，所以补一条软链（/usr/local/lib 在 ld 的 SEARCH_DIR 里）：
sudo ln -sfn /opt/ros/humble/lib/libvrpn.a /usr/local/lib/libvrpn.a
```

⚠️ **`libquat` 不需要软链** —— 它全程以绝对路径引用。
⚠️ 若坚持走源码编译那条老路（`git clone vrpn/vrpn` → `cmake` → `sudo make install`），
它装出来的布局**同样落在 `/usr/local`**，所以 `-lvrpn` 也能解析 —— 但那要 clone GitHub + 编译若干分钟，
**且版本不可 pin**。**本机实际走的是 apt 那条。**
完整依赖说明与验收记录见 [`../../src/vrpn_listener/VENDORED.md`](../../src/vrpn_listener/VENDORED.md) §5/§6。

### 2.2 ★★ `vrpn_listener` —— **不要从上游 GitHub 克隆**

> ⚠️ **上游明确警告**：**不要**用 `github.com/efc-robot/vrpn_client_ros2` 的公开版本。
>
> 该项目维护的是一个**定制版**，相对上游修了：
> - `stamp`
> - QoS
> - polling 频率
> - `handleTwist` / `handleAccel` 的 `frame_id`
>
> **用上游公开版会丢精度，并且会和下游 `vrpn_relative_pose` 的 reliable 订阅断连。**

**获取方式**：☠️ **本节原写"从已配置好的机器上打包整个 `vrpn_listener` 目录再拷过来"，
2026-09-21 起作废（ticket 03）** —— 该包**已经 vendored 进本仓库**，就在
[`src/vrpn_listener/`](../../src/vrpn_listener/)，**克隆本仓库即得**，不需要再从任何机器拷。
来源已钉在 zip + commit 上，并有逐文件 sha256 与一条可重跑的复核命令。

> ⚠️ **该包自带的 `README.md` 已过时**（它还在教人 clone 公开版）—— **不要读它**，
> 读 [`VENDORED.md`](../../src/vrpn_listener/VENDORED.md)（含**三个定制点**与**"勿替换为公开版"**）。
> 💡 原"不用 `rsync`"那条理由（容器里工具链不全）**连同本节一起作废**。

### 2.3 ★ 配置 `params.yaml`

```yaml
/vrpn_listener:
  ros__parameters:
    server: 10.1.1.198              # 动捕软件所在机器的网卡地址
    port: 3883
    frame_id: "world"
    mainloop_frequency: 400.0       # ★ 动捕 200 Hz 出包，400 Hz polling 留 2× 余量
    refresh_trackers_frequency: 1.0
    tracker_mainloop_frequency: 400.0
```

> 📌 **`mainloop_frequency` 要留 2× 余量**：动捕 200 Hz 出包，轮询设 400 Hz。**本项目 MPC 对延迟敏感，这个余量不能省。**

### 2.4 编译与运行

```bash
source /opt/ros/humble/setup.bash
cd ~/ros_ws && colcon build --packages-select vrpn_listener   # ★ colcon 根 = 仓库根，不是 src/
source install/setup.bash
ros2 launch vrpn_listener sync_entity_state.launch
```

> ✅ **2026-09-21（ticket 03）已按上面这组实跑并验收通过** ——
> 「colcon 根 = 仓库根」这条与本文件 §10 第 3 项一致。**构建产物已被仓库根 `.gitignore` 挡住。**
>
> ⚠️ **上游的工作空间路径是 `~/ws_livox`**（历史原因，来自 Livox 雷达项目）—— **上面的 `~/ros_ws` 就是 z0 的替代。**
> 本文档别处若还写着 `ws_livox`，一律按 `~/ros_ws` 读。

---

## 3. 坐标系与动捕标定

### 3.1 三套坐标系

| 坐标系 | 约定 | 出现在哪 |
|---|---|---|
| **PX4 本地/世界系** | **NED**（北-东-下） | `LOCAL_POSITION_NED`、`ODOMETRY(frame_id=1)`、`VISION_POSITION_ESTIMATE(frame_id=map)` |
| **PX4 机体系** | **FRD**（前-右-下） | 姿态、速度 |
| **ROS / 动捕 `map`** | **ENU**（东-北-上） | `/mavros/odometry/out`、动捕标定、刚体创建 |
| **ROS `base_link`** | **FLU**（前-左-上） | 机体坐标系，随飞机旋转 |

> ⚠️ **已纠正的表述**：早先这里写的是"mavros 会自动把 `/mavros/odometry/out` 的 **ENU 转成 NED 给飞控**"。**方向写反了。**
>
> **`mavros` 在边界上有两个方向的转换，容易记反**（已读 mavros 源码确认，见 [`px4-interface.md`](px4-interface.md) 5.1）：
>
> | 话题 | 方向 | mavros 做什么 |
> |---|---|---|
> | `/mavros/vision_pose/pose`（**我们 → PX4**） | 入 | **ENU→NED** + **FLU→FRD** |
> | `/mavros/odometry/out`（**PX4 → 我们**） | 出 | **NED→ENU** |
>
> **★ 对本项目的直接影响**：`/mavros/vision_pose/pose` 走的是**入方向**，所以**动捕节点应该发原生 ENU/FLU 值，不要自己先转**。自己转了 mavros 会再转一次 → **转两次**，症状是"能飞但方向不对/持续飘"。

### 3.2 ★ 人為设定的"东"

**这是一个刻意的简化**（上游原文）：

- PX4 的坐标数据大多遵循 NED，理论上"北"应是地磁北
- 但为了**坐标数据可视性强**，他们**假定"桌子朝窗的方向为东"**
- 配合 `EKF2_MAG_TYPE = None`（无磁罗盘）+ 不装 GPS → 飞机"找不着北" → **上电机头朝向即被当作北**

```
假定东 = 桌子朝窗的方向 = 动捕标定 map-ENU 的 x 正方向
上电机头朝向 = 桌子朝窗的方向 = 假定东 = 假定北
```

> ✅ **本项目也是无磁罗盘（`SYS_HAS_MAG = 0`），可以用同一套手法。** 但要记住：**这是一个约定，不是物理真值。** 文档里应写死"本项目的'北'指房间的哪个方向"。

> ⚠️ **本项目 Phase 1 对该流程有一处放宽 —— 但只放宽了一半**（详见 [`phase1-implementation.md`](../control/phase1-implementation.md) §3.4）：
>
> - ✅ **"机头必须朝向某个特定方向"不再必要** —— EKF2 融合动捕的 yaw（`EKF2_EV_CTRL = 11` 含 yaw 位）后，会把世界系对齐到动捕坐标系，**PX4 的"北"就是动捕 map 的 +x**，机头朝哪都行
> - ❌ **但"从节点启动到解锁之间不要移动/转动飞机"仍然是硬要求** —— 因为原点在**解锁上升沿**才最终锁定；启动到解锁之间若移动了飞机，原点切换就会产生一个位置/姿态阶跃，EKF2 会把它当成真实运动
>
> **一句话**：**朝向自由了，"摆好后别动它"没自由。**

### 3.3 动捕标定（L 校准 / T 校准）

在 **ENU** 下做：

```
x = 桌子朝窗的方向
y = x 的左侧
z = 上
```

---

## 4. 创建刚体

**上游的五个步骤**：

1. **反光球粘贴**：保证无人机顶部**能被全部摄像头捕捉**；以**金字塔五个顶点**的形式布置；⚠️ **注意非对称**（否则动捕无法区分朝向）
2. **无人机摆放参考**：机头朝向 = 桌子朝窗的方向 = 假定东 = x 正方向
3. **冻结帧**，`shift` 选中全部反光球**建立刚体**，命名
4. **骨骼朝向设为 x**（即机头方向）
5. **校验**：刚体的 xyz 应与动捕坐标系 xyz 一致；**机头 = 刚体 x，左侧 = 刚体 y，上 = 刚体 z**

> 📌 **z0 的动捕球是用铝柱架起来的**（见上游 BOM 条目"铝柱(装动捕球)"）。
> → **这意味着刚体原点与飞控 IMU 之间存在 Z 向偏移**，必须量出来填进 `EKF2_EV_POS_Z`。见 [`params-analysis.md`](params-analysis.md) 3.6。

---

## 5. 启动 mavros

### 5.1 安装 —— ☠️ **apt 装不到，必须源码构建**（2026-09-21 实测）

**apt 这条路已实测不通**：humble 的 `ros-humble-mavros` / `ros-humble-mavros-extras` 的 **arm64 二进制
在十个可达镜像里全都没有**（同一个仓库的 `ros-humble-mavros-msgs` 反而有 ⟹ 是**上游那次构建没产出/已撤**，
不是镜像没同步）；`packages.ros.org` 本机不可达。全部证据见
[`.scratch/phase1/issues/14-mavros-from-source.md`](../../.scratch/phase1/issues/14-mavros-from-source.md) §一。

**做法：源码构建，pin tag `2.15.1`**（与 apt 里的 `ros-humble-mavros-msgs 2.15.1-1jammy` 同代），
放在**独立工作区 `~/mavros_ws`**（**不入本仓库** —— mavros 是系统级依赖）：

```bash
mkdir -p ~/mavros_ws/src && cd ~/mavros_ws/src
git clone --depth 1 --branch 2.15.1 https://github.com/mavlink/mavros mavros
git -C mavros rev-parse HEAD          # 必须是 22ae5b7cc7cdb4cb9c2070a8213c72dae445a23e（= tag 2.15.1^{}）

# ★ 跳过三个包：mavros_msgs 用 apt 的同版本（不重复建一套）；另两个是示例与测试
cd ~/mavros_ws/src/mavros
touch mavros_msgs/COLCON_IGNORE mavros_examples/COLCON_IGNORE test_mavros/COLCON_IGNORE

sudo apt install ros-humble-eigen-stl-containers ros-humble-mavlink \
                 ros-humble-ament-cmake-google-benchmark   # ★ 第三个是构建时才暴露出来的

cd ~/mavros_ws && source /opt/ros/humble/setup.bash
colcon build --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release   # 8 GB 内存：必须限并行度
```

**依赖实测**：三个 `package.xml` 的 32 个依赖里 30 个已装（`libgeographic-dev 1.52`、`libasio-dev`、
`libconsole-bridge-dev`、`libeigen3-dev`、`libyaml-cpp-dev` 等），只缺 `ros-humble-eigen-stl-containers`
与 `ros-humble-mavlink`（构建期还缺 `ament_cmake_google_benchmark`，见上）。
**构建实测**：三个包全过、**0 失败**（`libmavconn` 37.7 s / `mavros` 7 min 2 s / `mavros_extras` 5 min 50 s）。

**GeographicLib 数据集（★ 必需，不是"真报错再装"）**：不装会**直接 launch 失败** ——
`GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm`。
源码构建时脚本在**源码树里**：

```bash
sudo ~/mavros_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

**三条环境纪律**：

1. `~/mavros_ws/install` 与本仓库的 `install` **必须同时 source**（两个工作区各有自己的 `setup.bash`）；
2. ⚠️ **两套 `mavros_msgs` 必须同代**：源码 mavros 2.15.1 配 apt 的 `2.15.1-1jammy`。
   哪天 apt 把它升到 2.16.x，源码 mavros 也得跟到同一 tag（否则 ABI 可能不匹配）；
3. `vision_pose` 插件在 **`mavros_extras`** 里 —— **不能只建 `mavros`**（否则 vision_pose 链路整个没有）。

### 5.2 ★ 用 launch 参数指定串口，**不要改共享文件**

```bash
# ★ z0 实测可用（2026-09-21）：
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600
```

> ⚠️ **不要改安装目录里的 `px4.launch`**（源码构建后是
> `~/mavros_ws/install/mavros/share/mavros/launch/px4.launch`；apt 安装时是
> `/opt/ros/humble/share/mavros/launch/px4.launch`）—— 那是**共享文件**，重装/升级就没了。一律通过 launch 参数传。

> 📌 **z0 的串口已定案：`/dev/ttyTHS1` @ 921600**（2026-09-21 用
> [`tools/prereq-evidence/mavlink_probe.py`](../../tools/prereq-evidence/mavlink_probe.py) `--scan` 逐口找心跳，
> 报 `sysid=1, type=2`；`ttyTHS2` / `ttyAMA0` 各波特率**一个字节都没有**）。
> 飞控侧走 TELEM2，与 `z0.params` 的 `MAV_1_CONFIG = 102`（TELEM2/Onboard）一致 ✅，
> 波特率与 `SER_TEL2_BAUD = 921600` 一致 ✅。
> ⚠️ **权限**：`chen` 必须在 `dialout` 组（`dialout:x:20:chen`），否则打不开串口。
>
> 📌 上游（另一台机器）用的是 **`/dev/ttyTHS3`** —— **设备名是每台机器各自实测出来的，不要照抄**。

### 5.3 校验

```bash
ros2 topic echo /mavros/state              # 看 connected: true
ros2 topic echo /mavros/vision_pose/pose   # 看位姿
```

**QGC 里三种数据的对应关系**（用于排查）：

| QGC MAVLink Inspector 里看到的 | 是什么 |
|---|---|
| `VISION_POSITION_ESTIMATE` (frame_id=map) | **动捕数据**（已做相对化） |
| `ODOMETRY` (frame_id=1, NED) | **动捕原始数据** |
| `LOCAL_POSITION_NED` | **EKF2 融合后**的估计 |

> 📌 **排查"动捕没生效"的最快方法**：如果 `VISION_POSITION_ESTIMATE` 有数但 `LOCAL_POSITION_NED` 不动，说明消息发到了但 EKF2 没用（检查 `EKF2_EV_CTRL`）。
> 如果 `VISION_POSITION_ESTIMATE` 根本没数，问题在动捕/mavros 这一侧。

#### ★ 不依赖 QGC 的硬校验（2026-09-21 实测有效，**推荐先用它**）

看**飞控自己的估计器状态**里"水平位置解有没有被绝对源锚定"：

```bash
ros2 topic echo /mavros/estimator_status        # 1 Hz；看 pos_horiz_abs_status_flag
```

本机**没有 GPS、没有光流**，水平绝对位置源**只有动捕** ⟹
**`pos_horiz_abs_status_flag = True` 就是"EKF2 正在用动捕锚定位置"的判据**。实测反转：

| 阶段 | 做法 | `pos_horiz_abs` |
|---|---|---|
| 有视觉 | `mocap` 节点在跑 | **True** |
| 无视觉 | 停掉 `mocap` 节点 4 s | **False** |
| 恢复 | 重新起 `mocap` 节点 | **True** |

★ **安全含义**：视觉一断，水平锚定**在 4 s 内就失效** —— 所以动捕链路断了必须立刻停飞/切手动
（`mocap` 节点的看门狗就是为这个）。

QGC 侧更硬的旁证（Analyze → **MAVLink Console** 里敲）：
`listener vehicle_visual_odometry`（应有数据）、`listener estimator_aid_src_ev_pos`（若存在，看 `fused`）。

#### ⚠️ 三个会静默骗人的坑（实测踩到）

1. **`/mavros/local_position/pose` 是 sensor QoS（BEST_EFFORT）** —— 用默认 RELIABLE 订阅**一帧都收不到**，
   只在日志里留一句 `New publisher discovered ... offering incompatible QoS ... Last incompatible policy: RELIABILITY`。
   工具里记得用 best-effort（Python: `rclpy.qos.qos_profile_sensor_data`）。
2. **`/mavros/local_position/pose_cov` 与 `velocity_*_cov` 默认没有数据** ——
   源码 `mavros/src/plugins/local_position.cpp:257` 只在飞控推 `LOCAL_POSITION_NED_COV` 时才发布，
   而 PX4 默认不推这个流。想用协方差做"是否发散"的实验，得先用 `/mavros/set_message_interval` 请求该流。
3. **`/mavros/state` 只有 `1.00 Hz`**（2026-09-21 实测：30 条 / 30 s，间隔 0.998–1.002 s）。
   ⟹ `armed` 的**解锁上升沿最多晚 1 s** 被检出；且 `ros2 topic hz` 在本机打不出读数（只打印 `Terminated`），
   量频率请用 rclpy 计数脚本。

### 5.4 ★★ 把飞控的 MAVLink 转给**笔记本上的 QGC**（WiFi 桥）

**用途**：QGC 不用接飞控的数传口，直接从本机的 WiFi 拿到整条 MAVLink 流。这也是
[ticket 13](../../../.scratch/phase1/issues/13-mavros-bench-vision-pose.md) 那两条**只能靠人看 QGC** 的验收
（① `VISION_POSITION_ESTIMATE` 有数 + `LOCAL_POSITION_NED` 稳定、⑥ 轴向确认）**唯一的落地途径**。

**给法（z0 实测可用，2026-09-21）**：

```bash
# ① 给**正在跑**的 mavros 热加一条 GCS 链路 —— 不用重启，不断飞控串口
ros2 param set /mavros_router gcs_urls "['udp-b://0.0.0.0:14555@:14550']"

# ② 下次启动时带上（热加的参数**不持久**，重启即失）
ros2 launch mavros px4.launch \
  fcu_url:=/dev/ttyTHS1:921600 \
  gcs_url:="udp-b://0.0.0.0:14555@:14550"
```

QGC 侧：什么都不用配 —— **QGC 默认就在 UDP `14550` 上监听**，广播包一到就自动建链。
（要手工加也行：`Application Settings → Comm Links → Add` → 类型 `UDP`、端口 `14550`。）

**实测证据**（本机 WiFi `10.1.1.114/24`，与动捕主机同网段）：在 14550 上放临时接收器，
**6 s 收到 3223 个 UDP 包（≈537 Hz），首字节全是 `0xfd`（MAVLink v2）**，来源 `10.1.1.114:14555`；
`ip route get 255.255.255.255` → `dev wlP1p1s0`（**走 WiFi，不走 docker0**）；
`ss -ulnp` → `0.0.0.0:14555 users:(("mavros_node",...))`；
router 日志 `Requested to add endpoint: … udp-b://0.0.0.0:14555@:14550` / `link[1002] opened successfully`。

#### 源码级语义（别凭记忆猜 URL 语法）

`udp://<bind_host>:<bind_port>@<remote_host>:<remote_port>` ——
**`@` 之前是 bind，之后是 remote**（`libmavconn/src/interface.cpp:418-437`）。

| 写法 | 含义 |
|---|---|
| `udp-b://0.0.0.0:14555@:14550` | **广播**：绑本机 14555，往 `255.255.255.255:14550` 发（`udp.cpp:94-98`，`address_v4::broadcast()`） |
| `udp://0.0.0.0:14555@10.1.1.x:14550` | **单播**给固定 IP（笔记本 DHCP 换地址即失效） |
| `udp://0.0.0.0:14555@:14550` | **服务端/学习**：先不发，收到第一个包记住对端再发 |

> 简写 `udp-b://@:14550` 与第一行**等价**（省略 bind 时的默认值是 `0.0.0.0:14555`，
> 见 `interface.cpp:435`）—— 显式写出来只是免得日后有人"顺手改成 localhost"。
> ★ 广播是"**先广播、被应答后自动转单播**"：收到真实包后 `remote_ep` 被改写为对端地址
> （`udp.cpp:303-315`）⟹ **笔记本一连上，本机 14550 上的监听器就会静默**（可当"确实连上了"的旁证）。

#### ☠️ 三个坑

1. **节点名和参数名都不是直觉那个**：2.15.x 的 mavros 是 **router 架构** ——
   `mavros` 是 UAS 节点，**持链路的是 `mavros_router`**，参数是复数的 **`gcs_urls`**（字符串数组）。
   启动形参 `gcs_url`（单数）只在**启动那一刻**被搬进 `gcs_urls`（`mavros_node.cpp:77-79`）
   ⟹ **运行时改 `gcs_url` 无效**；`gcs_urls` 才有增删回调（`mavros_router.cpp:281-360`），所以能热加。
2. **别想在日志里找对端 IP**：`link[%d] detected remote address %d.%d` 里的两个数是
   **MAVLink 的 `sysid.compid`**（`mavros_router.cpp:474`），不是 IP。QGC 的 sysid 通常是 **255**
   ⟹ 日志里出现 `detected remote address 255.x` 即为 QGC 连上。日志在 `~/.ros/log/mavros_node_<pid>_*.log`。
3. **带宽不小**：实测 ≈537 Hz 全量流。WiFi **广播帧无重传、且会复制给同一 AP 下所有站点**；
   网络吃紧就改单播（上表第 2 行）或降低 PX4 流率。

#### ⚠️ 两条安全提醒

- 这条桥是**双向**的：QGC 从此能**解锁、切模式、接管飞机** ⟹ 动线/调参时**拆桨**。
- **别同时再开第二条 QGC 链路**（例如飞控 TELEM 口的数传电台）：同一 sysid 会出现两个 vehicle，
  router 的 sysid 路由也会左右为难。

> 📌 §8.1 那份 `~/autostart.sh`（上游模板）里的 `gcs_url:="udp://@10.1.1.103"` 是**单播给固定地面站 IP**
> 的写法（语义同本表第 2 行；省略 bind ⟹ 默认 `0.0.0.0:14555`）。本机 2026-09-21 实测**没有**
> `~/autostart.sh`、也没有任何 systemd 单元，mavros 是手起的 ⟹ 先用上面 ① 热加。

---

## 6. ★★ 速度数据路由（重要的架构决策）

> ⚠️ **上游明确警告：不要把 `/vrpn/<刚体>/twist` relay 到 `/mavros/vision_speed/speed_twist`。**

**当前架构**：EKF2 **只融位置、不融速度**（`EKF2_EV_CTRL` 的 bit 3 不勾选）。

**速度从哪来**：

| 用途 | 速度来源 |
|---|---|
| 推理 policy（上游的 RL 控制器） | 直接订阅 `/vrpn/<刚体>/twist.linear`（**动捕自算的，最干净**） |
| PX4 EKF2 / Position 模式 | 由 `vision_pose` + IMU **内部微分**得到 |

**★ 不转发的原因（关键）**：

> `vrpn_listener` 的 `stamp` 用的是 `listener->now()`（ROS 接收时刻），**而不是 VRPN 的 `msg_time`**。
> **喂给 EKF2 反而引入偏差。**

**★ 但注意：这是一个刻意的设计，不是 bug。** 见 6.2。

---

### 6.2 ★★★ `stamp = now()` 是刻意的设计，**不要改**（本节推翻旧结论）

> ⚠️ **本节更正了本文档 2026-09-18 早先版本的错误结论。** 旧版本称 `stamp` 用 `now()` 是缺陷、"比真实采样时刻晚 **10–100 ms** 且带抖动"，并建议"把 `stamp` 还原成 `msg_time`"。**该结论是错的，照做会毁掉 EKF2。**

**第一手证据**：上游 `vrpn_listener.zip` 的 `src/vrpn_listener.cpp` 第 37–40 行**原样写着理由**：

```cpp
    pose_msg.header.frame_id = listener->_frame_id;
    // Use ROS receive time, not VRPN msg_time, because the mocap PC clock is
    // not NTP-synced with the Jetson (~4s offset observed). Adds ~2-5ms jitter
    // but keeps the stamp in the same clock domain as IMU/MAVROS for fusion.
    pose_msg.header.stamp = listener->now();
```

`twist`（第 62 行）与 `accel`（第 93 行）同样如此，理由相同。

> ### ✅ 2026-09-20 复核（Q16=b 的执行结果）
>
> 重新从上游取 `Huan-qun/Sim2real-UAV-Params-` 的 `S2R/assets/vrpn_listener.zip` 核对：
>
> - **仓库 HEAD `b7aeebb71cc708445fbac70a4863c736d4674630`（2026-08-30）**
> - zip 在 commit `1aa94a98`（2026-07-21）加入后**未再改动**（`vrpn_listener.cpp` 8088 B）
> - **上述 `stamp = listener->now()` 与注释一字不差仍在** ✅ —— **重新取上游不会丢掉这个行为**，
>   Q16=b 可放心执行。
> - 同一文件里还有一条相关定制：`handleTwist` **刻意把角速度留零**，注释说明
>   "多数动捕 VRPN server 不填 `vel_quat`，角速度应从 IMU 取"。**同属必须保留的上游行为。**
>
> ⚠️ **措辞纪律**：这是**上游自己发布的版本就这样**，不是"我们本地打的补丁"。
> 因此它**尤其容易**被后来者当成脏改动而"顺手清理" —— 这正是本节要防的事。
>
> **zip 内 `config/params.yaml` 的实际值**（旧版本文档只记了行数，且有偏差）：
>
> | 项 | 值 |
> |---|---|
> | `server` | `10.1.1.198` |
> | `port` | `3883` |
> | `frame_id` | `world` |
> | `mainloop_frequency` | `400.0`（⚠️ **代码内默认是 `100.0`**，yaml 覆盖为 400） |
> | `refresh_trackers_frequency` / `tracker_mainloop_frequency` | `1.0` / `400.0` |
>
> （`code-inventory.md` 称该文件「7 行」，实际 **8 行** —— 小瑕疵，以本节为准。）
>
> 📌 **上游还刚修了一个启动时序 bug**（HEAD commit 标题）：
> *"宿主机开机太快导致 autostart.service 启动得太早，Wi‑Fi/VRPN 是 not ready，listener 的 socket 进入了失败状态，并且之后没有自行恢复"*。
> **与 §3.9 里 `COM_RC_IN_MODE = 3`「第一个可用源用到重启为止」是同一类风险** ——
> z0 的启动流程需要保证「链路先就绪，再启动消费者」。

**更正对照**：

| | 旧结论（❌ 错） | 实际情况 |
|---|---|---|
| 抖动量 | 10–100 ms | **~2–5 ms** |
| 性质 | 缺陷，应改掉 | **刻意的 workaround** |
| 原因 | 未提 | ★ **动捕主机与 Jetson 时钟未同步，实测偏移 ~4 秒** |
| 改法 | "还原成 `msg_time`" | ⚠️ **直接改会引入 ~4 s 误差，彻底毁掉 EKF2** |

**为什么必须用 `now()`**：`msg_time` 是**动捕主机的时钟**。两台机器差 ~4 秒，把这个时间戳交给 EKF2，等于谎报数据是 4 秒前/后的。`now()` 是 ROS 接收时刻——**必真实采样时刻略晚（VRPN 传输 + 轮询间隔）**，但**与 IMU、mavros 在同一个时钟域**，EKF2 才能正确做时间对齐。用 2–5 ms 的精度损失换时钟域一致性，是**正确的工程取舍**。

**连带失效的两条旧推论**：

| 旧说法 | 更正 |
|---|---|
| "真实最坏延迟可能是 50 + 100 = **150 ms**" | ❌ 不成立。实际 ≈ 50 + 2~5 ≈ **55 ms** |
| [`../control/mpc-roadmap.md`](../control/mpc-roadmap.md) 第 7 节把"时戳抖动"列为 **★★ 最高风险**，建议"提前到 MPC 开发之前"修 | ❌ **该风险不存在**，不需要为它排任何工期 |

**因此本节的正确结论是**：

1. ✅ **保持 `stamp = now()` 不动。** 这是经过实机验证的配置。
2. ❌ **不要**把 `stamp` 改成 `msg_time`——除非**先完成 NTP/PTP 同步**（那是唯一的前置条件，也是它的意义所在）。
3. 🔶 **NTP 同步降级为"可选"**：只有在将来确实需要**真实采样时刻**（如多源融合里要精确对齐时间）时才必须做。**Phase 1 和 Phase 2 都不需要它。**
4. ✅ **一个安全的清理**（可选，与上面无关）：本项目重写的动捕节点应当**转发 `vrpn_listener` 的时间戳**，而**不要像 `vrpn_relative_pose` 那样用 `now()` 再打一次戳**（见 7.3）。两次打戳会让 EKF2 认为数据比实际更新，削弱它的延迟补偿。这仍然是**同一时钟域内**的操作，不涉及 4 秒偏移。

> 📌 **旧版本的待办"查源码确认 `stamp` 是否真的是 `now()`"——已关闭。答案：是，且是刻意的。**

---

## 7. `vrpn_relative_pose` 节点（★ 可复用的核心资产）

### 7.1 解决什么问题

**动捕上电时返回 (0,0,0)** 或者无人机**不在动捕原点**时，位姿会有一个常值偏移。

**解法**：节点启动后先**采样 10 帧求平均**作为初始偏移，之后**所有位姿都减去这个偏移**并转发。
→ **保证无人机上电后收到的动捕数据是 (0,0,0)，可以任意摆放。**

> 这与用户笔记里描述的 `vrpn_relative_pose` 手法完全一致 —— **上游和用户解决的是同一个问题**。

### 7.2 创建

```bash
cd ~/ws_livox/src
ros2 pkg create vrpn_relative_pose --build-type ament_cmake \
  --dependencies rclcpp geometry_msgs tf2 tf2_geometry_msgs
```

### 7.3 源码 `src/vrpn_relative_pose_node.cpp`

```cpp
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class VrpnRelativePoseNode : public rclcpp::Node
{
public:
    VrpnRelativePoseNode() : Node("vrpn_relative_pose_node"), is_initialized_(false), sample_count_(0)
    {
        // 声明参数，方便在 launch 文件中修改话题名
        this->declare_parameter<std::string>("input_topic", "/vrpn/r11/pose");
        this->declare_parameter<std::string>("output_topic", "/mavros/vision_pose/pose");
        this->declare_parameter<int>("init_samples", 10); // 采样10帧求平均值

        std::string input_topic, output_topic;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("output_topic", output_topic);
        this->get_parameter("init_samples", max_samples_);

        // 订阅原始动捕话题
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            input_topic, 10, std::bind(&VrpnRelativePoseNode::pose_callback, this, std::placeholders::_1));

        // 发布给 MAVROS 的相对位姿话题
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Relative Pose Node started. Waiting for initialization samples...");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!is_initialized_) {
            // 初始化阶段：累加位姿数据求均值
            sum_x_ += msg->pose.position.x;
            sum_y_ += msg->pose.position.y;
            sum_z_ += msg->pose.position.z;
            sample_count_++;

            if (sample_count_ >= max_samples_) {
                offset_x_ = sum_x_ / max_samples_;
                offset_y_ = sum_y_ / max_samples_;
                offset_z_ = sum_z_ / max_samples_;
                is_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "Initialization complete. Offset: [%.3f, %.3f, %.3f]",
                            offset_x_, offset_y_, offset_z_);
            }
            return;
        }

        // 计算相对位姿
        auto relative_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
        relative_pose->header = msg->header;
        relative_pose->header.frame_id = "map"; // 习惯上发给 MAVROS 的坐标系设为 odom 或 map
        relative_pose->header.stamp = this->get_clock()->now();

        // 位置减去偏移量
        relative_pose->pose.position.x = msg->pose.position.x - offset_x_;
        relative_pose->pose.position.y = msg->pose.position.y - offset_y_;
        relative_pose->pose.position.z = msg->pose.position.z - offset_z_;

        // 姿态（四元数）通常不需要减偏移，保持动捕定义的绝对朝向即可
        // 这样可以保证机头方向与动捕标定的方向一致
        relative_pose->pose.orientation = msg->pose.orientation;

        publisher_->publish(std::move(relative_pose));
    }

    // 状态变量
    bool is_initialized_;
    int sample_count_;
    int max_samples_;
    double sum_x_ = 0, sum_y_ = 0, sum_z_ = 0;
    double offset_x_ = 0, offset_y_ = 0, offset_z_ = 0;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VrpnRelativePoseNode>());
    rclcpp::shutdown();
    return 0;
}
```

> ⚠️ **注意第 353 行：`relative_pose->header.stamp = this->get_clock()->now();`**
> **它覆盖掉了 `vrpn_listener` 刚打的时间戳**（第 307 行的 `relative_pose->header = msg->header;` 本来已经拷贝了）。
> → 后果是 EKF2 认为数据比实际更新，**削弱它的延迟补偿**。量级很小（毫秒级），不是致命的。
> → **正确做法是删掉这一行、直接转发 `vrpn_listener` 的 stamp**（同一时钟域，**不是**改成 `msg_time`——见 6.2）。
>
> 📌 **本项目已决定重写这个节点**（原点是"解锁上升沿"而非"启动后 10 帧"，且只归零 yaw），重写时直接按上面的做法处理时间戳。本节的旧源码仅作参考。
>
> ✅ **2026-09-21 已实现（[ticket 04](../../.scratch/phase1/issues/04-mocap-relative-pose.md)）= [`src/mocap/`](../../src/mocap/)**，
> 且实际做法比上面的计划**多了一段**：**两阶段原点** ——
> ① 节点启动就锁定「启动原点」并**持续发布**（否则解锁前 EKF2 断粮，§7.1 的"启动后 10 帧"只覆盖了位置偏移，没覆盖"解锁前必须已经在发"这一点）；
> ② **解锁上升沿**再切到「起飞前静止帧原点」。连带的新纪律是「**启动到解锁之间不要搬动飞机**」。
> 参数表、运行纪律与理由见 [`src/mocap/README.md`](../../src/mocap/README.md)。

### 7.4 `launch/vrpn_relative.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    r11_node = Node(
        package='vrpn_relative_pose',
        executable='vrpn_relative_pose_node',
        name='vrpn_relative_pose_r11',
        output='screen',
        parameters=[{
            'input_topic': '/vrpn/r11/pose',
            'output_topic': '/mavros/vision_pose/pose',
            'init_samples': 10,
        }],
    )

    return LaunchDescription([
        r11_node,
    ])
```

> 💡 **多机时**：复制 `r11_node` 块改 `name` / `input_topic` / `output_topic`。主机发到 z0 的 `/mavros/vision_pose/pose`，副机刚体发到 `/vrpn/<name>/pose_relative`。

### 7.5 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(vrpn_relative_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(vrpn_relative_pose_node src/vrpn_relative_pose_node.cpp)
ament_target_dependencies(vrpn_relative_pose_node
  rclcpp geometry_msgs tf2 tf2_geometry_msgs
)

install(TARGETS vrpn_relative_pose_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### 7.6 `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vrpn_relative_pose</name>
  <version>0.0.0</version>
  <description>Subtract initial mocap pose offset and republish for MAVROS vision_pose.</description>
  <maintainer email="root@todo.todo">root</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

> 📌 **这个包应该被提取成本仓库里的正式 ROS2 功能包**，而不是留在文档里。

---

## 8. 开机自启动

### 8.1 启动脚本 `~/autostart.sh`

```bash
#!/bin/bash
# 加载环境（如果用户不是 root，把 /root 替换成 $HOME 或实际 home 路径）
source /opt/ros/humble/setup.bash
source /root/ws_livox/install/setup.bash
sleep 2

ros2 launch vrpn_listener sync_entity_state.launch &
sleep 2

ros2 launch mavros px4.launch \
  fcu_url:="/dev/ttyTHS3:921600" \
  gcs_url:="udp://@10.1.1.103" > mavros.log 2>&1 &
sleep 2

ros2 launch vrpn_relative_pose vrpn_relative.launch.py &

# 等待所有后台进程，比 while true sleep 更干净
wait
```

> 💡 `gcs_url` 里的 `10.1.1.103` 改成 z0 的地面站 IP；不需要 GCS 转发可省略该参数。

### 8.2 ★ systemd 服务（含"等待动捕服务器就绪"技巧）

```ini
[Unit]
Description=ROS 2 VRPN Autostart Service
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=<改成你的用户名>
ExecStartPre=/bin/bash -c 'until /usr/bin/nc -z -w 1 10.1.1.198 3883; do echo "waiting for VRPN server 10.1.1.198:3883"; sleep 2; done'
ExecStart=/home/<你的路径>/autostart.sh
Restart=on-failure
RestartSec=5s
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
```

> 📌 **`ExecStartPre` 那一行是很实用的技巧**：用 `nc -z` **轮询动捕服务器端口**，直到通才启动。避免开机时动捕软件还没起来导致 `vrpn_listener` 拿不到数据。

```bash
chmod +x ~/autostart.sh
sudo systemctl daemon-reload
sudo systemctl restart autostart.service
sudo systemctl status autostart.service --no-pager
# 停止 / 禁用
sudo systemctl stop autostart.service
sudo systemctl disable autostart.service
```

> ⚠️ **注意**：容器内的自启动（8.1）与宿主的 systemd（8.2）是**两套方案**。上游两条路都有。**本项目应统一走宿主 systemd**，避免容器带来的路径/权限复杂度。

---

## 9. 验证清单

按顺序执行，任何一步失败都不要往下走：

```bash
# 1) 动捕数据能进来吗
ros2 topic list | grep vrpn
ros2 topic echo /vrpn/<刚体>/pose

# 2) mavros 连上飞控了吗
ros2 topic echo /mavros/state          # connected: true

# 3) 位姿转发了吗
ros2 topic echo /mavros/vision_pose/pose

# 4) 频率够吗（应接近动捕出包率，200 Hz 量级）
ros2 topic hz /mavros/vision_pose/pose

# 5) 飞控侧：EKF2 用上了吗
#    在 QGC 的 MAVLink Inspector 里看：
#    VISION_POSITION_ESTIMATE 有数 + LOCAL_POSITION_NED 跟着动  → 成功
```

**然后**：QGC 切 **Position 模式**，看飞机能否自主定点悬停（**不要直接上 Offboard**）。

---

## 10. ★ z0 适配清单（相对上游要改什么）

上游是**多机集群 + 另一台机架**，以下都必须改：

| # | 项 | 上游值 | z0 要做什么 |
|---|---|---|---|
| 1 | **刚体命名** | `r11` / `r22` | ★★ **不要写死名字 —— 这一项的正确做法是「现场核对」，不是「抄一个常量」**（本项在 2026-09-21 一天之内被改过两次，教训比结论值钱）。<br>**机制**：下游话题 = `/vrpn/<sender 名>/pose`，名字**原样取自 VRPN**，`replaceSpace()` **只把空格换成 `_`、不改大小写、不做任何归一化**。<br>**当下实测值**：**`z0`（小写）** ⟹ **`/vrpn/z0/pose`**（2026-09-21 重测，ticket 12）。<br>☠️ **当天早些时候（ticket 03）实测到的是 `Z0`（大写）**，后来操作者在 Nokov 侧调整后变为小写 —— **两次都是真的，说明它是个变量**。<br>**怎么核对**（唯一可靠的办法，任何配置前先跑一次）：<br>`ros2 topic list \| grep vrpn` —— 看到什么名字就用什么。<br>⚠️ 它还会**连带**出现 `z0_Marker1..5`（单颗球各自也是独立 sender）。详见文末「ticket 03 实测」§二 与「ticket 12 重测」§一 |
| 2 | **串口设备** | `/dev/ttyTHS3` | ✅ **已实测**：z0（Jetson Orin NX）上**只有 `/dev/ttyTHS1` 和 `/dev/ttyTHS2`**，`THS3` 与 `THS0` 都不存在。<br>⚠️ **2026-09-21 操作者确认：飞控侧走 TELEM2**（与 `z0.params` 的 `MAV_1_CONFIG = 102` = TELEM2/Onboard ✅ 一致）。<br>✅ **2026-09-21 定案：`/dev/ttyTHS1` @ 921600** —— 判据是 `mavlink_probe.py --scan` 逐口找心跳：THS1 报 `sysid=1, type=2`（飞控在此），`ttyTHS2` / `ttyAMA0` 各波特率**一个字节都没有**（ticket 01 ② 结案）。<br>⚠️ `chen` 需在 `dialout` 组（已修：`dialout:x:20:chen`） |
| 3 | **工作空间路径** | `~/ws_livox` | 改成 `~/ros_ws`（z0 的实际工作空间） |
| 4 | **`gcs_url`** | `udp://@10.1.1.103` | 改成自己的地面站 IP |
| 5 | **动捕网段** | `10.1.1.x` | ✅ **不需要额外网卡**。VRPN 就是 UDP over IP，**把机载电脑连到动捕主机所在的同一个 WiFi 即可**。z0 曾连 `SUSTech-802.1x`（`10.29.12.x/14`），到 `10.1.1.198` 的路由经校园网网关绕出、不通；换到动捕所在网络后即通。<br>⚠️ **但「那个 WiFi」到底叫什么，本文档至今没写** —— 这是本项唯一未闭合的信息，见下方 2026-09-21 实测 |
| 6 | **EKF2 参数** | 见手册第 3 节 | 用 [`../assets/z0.params`](../assets/z0.params)，见 [`params-analysis.md`](params-analysis.md) |
| 7 | **`EKF2_EV_POS_Z`** | `0` | ⚠️ z0 用铝柱架动捕球，**必须量出 Z 偏移** |
| 8 | **计时同步** | 未做 | 🔶 **可选，不再强制**。见 6.2：`stamp = now()` 是刻意设计，2–5 ms 抖动可接受。**只有想让 `msg_time` 可用时才必须补 NTP** |
| 9 | **动捕节点的 stamp** | `now()` 重新打戳 | ✅ 重写时**转发 `vrpn_listener` 的 stamp，不要二次打戳**（同一时钟域，见 6.2）。**已实现并实测**：`src/mocap/` 把入参 `header` 整块拷贝到出参（2026-09-21，ticket 04） |

**第 5、8、9 项已根据第一手证据更正**：第 5 项不是硬件问题而是"连对 WiFi"；第 8 项从"必须"降级为"可选"；第 9 项只是避免二次打戳，与 `msg_time` 无关。

> ★ **2026-09-21 首次实测（在 z0 自己的 Jetson `orin-nx` 上）：当时第 5 项未完成，缺的具体是「SSID 名字」**
> —— ⚠️ **这一节已被同日稍晚的复测闭合（见本节末尾）**，原始读数保留以备对照。
>
> - 当前仍连 **`SUSTech-802.1x`**，IP `10.29.12.59/14`，默认网关 `10.31.255.254`（校园网）。
> - `ping 10.1.1.198` → **100% 丢包**；`ip route get 10.1.1.198` 显示经校园网关绕出。网关本身可通（≈3.5 ms），
>   所以不是"网断了"，就是**不在同一网段**。
> - 有线口 `enP8p1s0` = **NO-CARRIER**（没插网线），所以当前也不存在走有线的可能。
> - VRPN 业务端口 `10.1.1.198:3883` TCP 不可连（⚠️ VRPN 实际走 UDP，此点不通不必然等于 VRPN 不通）。
>
> **全量 AP 扫描（2026-09-21）**：剔除 `SUSTech-*` / `eduroam` 与隐藏名后，在范围内的候选为 ——
> `407lab`、`504509D`、`508office`、`508office-5G`、`BearWifi`、**`CLEARLab-2.4G`**、
> `DIRECT-8F-HP`、`SUST-XY-605-HUAWEI-AX6`、**`TPLINK_7A79_2.4G`**、`TPLINK_7A79_5G`、`Xuyang_Office`。
>
> ⚠️ **这份候选名单只是「机械剔除校园 SSID 后剩下的」，不是「动捕网就是其中之一」的结论。**
> 名字里带 `CLEARLab` / `TPLINK` 的看起来像实验室自己的路由器，但**这一点必须由人到现场指认**，
> 不能由 agent 猜。**这是第 5 项唯一剩下的缺口。**
>
> 采集脚本：[`../../tools/prereq-evidence/capture.sh`](../../tools/prereq-evidence/capture.sh)（只读，会重新扫一遍）。

---

### ★ 2026-09-21 复测（同日稍晚，同一台 Jetson `orin-nx`）—— 第 5 项 ✅ **已闭合**

**「那个 WiFi 叫什么」这个唯一缺口，答案是 `TPLINK_7A79_2.4G`。** 全部为第一手读数：

| 项 | 读数 |
|---|---|
| 已连 WiFi（`nmcli dev status`） | **`TPLINK_7A79_2.4G`**，接口 `wlP1p1s0` |
| 本机地址 | `10.1.1.114/24` |
| 默认网关 | `10.1.1.1`（**不再是校园网关 `10.31.255.254`**） |
| `ping -c5 10.1.1.198` | ✅ **0% 丢包**（5/5，RTT 1.7–37.1 ms，avg 13.3 ms） |
| ARP（`ip neigh`） | `10.1.1.198 lladdr e8:9c:25:1d:b8:64 REACHABLE` |
| 切换动作 | `chen` 手动接入；NetworkManager profile 落盘时间 `2026-09-21 06:02`（两处 profile mtime） |

> ✅ **这条同时就是 ticket 01 的 ① 验收项**（「Jetson 在动捕网段内，`ping 10.1.1.198` 0% 丢包，不再走校园网关」）。

#### ⚠️ 复测顺带查出的**新事实**：动捕主机的 3883 在 **TCP** 上答 VRPN banner，UDP 无应答

| 探测 | 结果 |
|---|---|
| TCP `10.1.1.198:3883` 连接 | ✅ **OPEN**，banner = `vrpn: ver. 07.34  0\x00\x00\x00\x00\x00` ⇒ **确有 VRPN 服务器在监听** |
| TCP 135 / 139 / 445 | ✅ OPEN ⇒ 动捕主机是 **Windows** |
| TCP 3389 / 5900 / 80 / 22 / 3383 | ✗ 关闭 |
| **UDP** 3883（8 个刚体名：`z0`/`r11`/`r22`/`uav1`/`drone`/`XI35`/`test`；3 种报文格式） | ✗ **一律超时** |
| TCP banner 之后再读 12 s（回 banner / 不回 banner 两种握手） | ✗ **一个字节都没推** |

> ⚠️ **以上只是读数，不是结论。** 至少三种可能**尚未甄别**：
> ① Windows 防火墙放行了 TCP/3883 但挡掉入向 UDP；② Nokov 侧 VRPN 输出未开 / 未播放；
> ③ Nokov 的 VRPN 只走 TCP。
>
> ★ **对本文 §1 数据流图 `VRPN/UDP(3883)` 的直接影响**：本节上方与 §1 都默认是 **UDP**，
> 但实测只看到 **TCP** 在答。**传输层到底是哪个，必须由 ticket 03 用真 `vrpn_listener` 判定**，
> 不能沿用这里的默认假设 —— 配错了表现为「listener 起来但收不到数据」，和「动捕没开」现象一样，
> 二分成本很高。
>
> 采集方式：`python3` 裸 socket（`socket.SOCK_STREAM` / `SOCK_DGRAM`），只读，未发任何写指令。

---

### ★★ 2026-09-21 ticket 03 实测：**传输层 = UDP**（上面那个未决项已闭合）+ 刚体名与大写问题

> 执行者：agent，在 z0 的 Jetson（`orin-nx`）上跑**真正的 `vrpn_listener`**（vendored，见
> [`../../../src/vrpn_listener/VENDORED.md`](../../../src/vrpn_listener/VENDORED.md)）。
> 全部为第一手读数。

#### 一、传输层判定 —— **UDP，§1 的数据流图不用改**

| 证据 | 读数 |
|---|---|
| `ss -tunap`（**vrpn_listener 进程**的 socket） | **3 个 UDP** 连到 `10.1.1.198:3883`（fd 15/16/17）—— 数据就是走这里 |
| 同进程的 TCP | `LISTEN 10.1.1.114:51185` + 一条 `ESTAB`，**对端是 `10.1.1.198:52182`** ⟹ 这条是**动捕主机主动连进本机监听口**的，不是我们发起的 |
| 抓包（`AF_PACKET`，见下） | 动捕主机 5 s 内向本机 UDP `:36039`（= vrpn_listener 的 socket）推 **2997 个包**，其中 2978 个载荷 1400 B ⟹ **服务器在主动推** |
| 独立 VRPN 客户端（`vrpn_Tracker_Remote("robot0@10.1.1.198:3883")`） | 5 s 收到 **961 次** pose 回调 ⟹ **≈192 Hz**，链路完全正常 |

⟹ **上面「TCP 答 banner、UDP 无应答」是探测方法的问题，不是服务器的行为**：
VRPN 的 UDP 不认裸 socket 发出去的随机报文（要先做它自己的端口协商），
**裸探针测不到 ≠ 不可用**。§1 的 `VRPN/UDP(3883)` **是对的**。

> ⚠️ **教训（与第三轮那条同源）**：「裸探针没回应」不能推出「那条路不通」——
> 这次是**用真 `vrpn_listener` 才判定出来的**，正如本节当初所要求的那样。

#### 二、★ 刚体名：`Z0`（大写），不是 `z0`

> ☠️ **本节结论已作废（当天稍后重测：名字是小写 `z0`）** —— 见下方「ticket 12 重测」§一。
> **原文保留**，因为本节记的读数当时是真的，而它要教的那条（**名字不是常量**）现在更成立。

`vrpn_listener` 自动发现到 **28 个 sender**，与 z0 相关的有：

| sender 名 | 说明 |
|---|---|
| **`Z0`** | ★ **刚体本体**，下游要订阅的是 **`/vrpn/Z0/pose`** |
| `Z0_Marker1` … `Z0_Marker5` | ⚠️ **单颗反光球各自也是独立 sender**，会各自产生 `/vrpn/Z0_MarkerN/{pose,twist,accel}` |

同一台服务器上还有别的项目的刚体：`robot0` / `robot0_Marker1..5`、`robot1` 及其 markers、
`U_Tracker0..9`、`VRPN Control`。**总数 28 → 自动生成 84 个话题。**
⟹ **下游必须按名字挑，不能"收到第一个就当成 z0"**。

#### 三、⏳ `Z0` 当前**没有数据流** —— 这条**阻塞** ticket 03 的速率验收（对 z0 而言）

> ✅ **本节已被解决**（不是被证伪）：当天稍后重测 **190.9–197.0 Hz**，五颗球全在。
> 见下方「ticket 12 重测」§二。**原文保留** —— 它记的「要人做的事」正是当时真正缺的那一步。

| 刚体 | 是否有 pose 流 |
|---|---|
| `robot0` / `U_Tracker0` | ✅ ≈192–200 Hz |
| **`Z0`** | ❌ **5 s 内 0 次回调**（独立 VRPN 客户端实测） |

**`Z0` 出现在 sender 名单里 ≠ 它在出数**：名单来自服务器的对象列表，
而出数要求该刚体**正在被追踪**（通常就是**飞机此刻在动捕棚有效捕捉范围内、且 Nokov 侧已开流**）。

⟹ **要人做的**：把 z0 放到动捕棚里 / 在 Nokov 软件里确认 `Z0` 已启用输出，
然后用下面这条验收（**速率本身已在 `U_Tracker0` 上验到 199.9 Hz，达标**）：

```bash
ros2 topic hz /vrpn/Z0/pose        # 期望 ~100–400 Hz（动捕 200 Hz 出包）
```

#### 四、版本：[只差一个小版本号，可忽略]

客户端（本机 `ros-humble-vrpn` **7.35.0**）连上服务器（banner **`vrpn: ver. 07.34  0`**）时打印：

```
check_vrpn_cookie(): VRPN Note: minor version number doesn't match:
(prefer 'vrpn: ver. 07.35', got 'vrpn: ver. 07.34  0').  This is not normally a problem.
```

**实测不影响收数**（`robot0` 的 200 Hz 就是在这条警告下收到的）。**不要为此去换 VRPN 版本。**

---

### ☠️ 2026-09-21 稍后重测（ticket 12）：上面 §二 §三 两条**已被推翻** —— `z0` 现在出数了，且**名字是小写 `z0`**

> 执行者：agent，同一台 Jetson。操作者在 Nokov 侧调整之后重跑。
> **两条旧结论都作废，但推翻的方式不同，分开记。**

#### 一、★ 刚体名现在是 **`z0`（小写）** —— §二 作废

| 项 | ticket 03 那次 | ★ 本次 |
|---|---|---|
| sender 名单里的名字 | `Z0`（大写） | **`z0`（小写）** |
| `Z0`（大写）还在名单里吗 | 在 | ❌ **已不在** |
| sender 总数 | 28 → 84 个话题 | **27 → 81 个话题** |

⟹ 下游要订阅的是 **`/vrpn/z0/pose`**。**上面 §二 与 §10 第 1 项写的 `Z0` 已过时。**

> ★★ **这一条真正的教训不是"大小写改了"，而是"这个名字不是常量"**：
> 它**原样取自 VRPN 的 sender 名**，而 sender 名来自 Nokov 项目里刚体**当下**怎么叫 ——
> **有人在软件里重命名一次，下游所有硬编码的名字就全错**。
> 所以：**名字必须按当期 sender 名单现场核对**（`ros2 topic list | grep vrpn`），
> **不要从文档里抄一个常量去写死**。§10 第 1 项已按这条重写。
> （`replaceSpace()` 只把空格换成 `_`、**不做大小写归一化** 这个机制描述**仍然成立** ——
> 两次实测都印证了它：发什么名就出什么话题。）

#### 二、★ `z0` 已有数据流，且**五颗球全在** —— §三 作废（而且是被"解决了"作废，不是被证伪）

| 判据 | 读数 |
|---|---|
| `/vrpn/z0/pose` 速率 | ✅ **190.9 – 197.0 Hz**（`ros2 topic hz` 两次 + 3 s 计数 592 帧） |
| `z0_Marker1..5` 的 pose | ✅ **五路全在，各 ≈196 Hz**（3 s 内 590–591 帧）⟹ **五颗球都被追踪、都在视野里** |
| `frame_id` | ✅ **`world`**（= `params.yaml` 的 `frame_id` 参数，不是源码写死的） |
| `stamp` | ✅ **严格递增、相邻间隔 < 50 ms**（5 帧逐帧核对）⟹ 是**接收时刻**（`stamp = now()` 定制生效），**mavros 不会丢** |
| 姿态 | ✅ 近单位四元数（`x,y,z ~1e-4`，`w=1.0`）—— 静止平放，符合预期 |
| 位置 | `(-1.2804, -0.1397, +0.0883)` —— 静止不动，`z ≈ 0.09 m`（铝柱垫起来的球心高度，与 §10 第 7 项要量 Z 偏移相关）<br>⚠️ **x 是 -1.28 m 而不是 0**：动捕原点在棚里别处 ⟹ **这正是 ticket 04 要减掉原点的原因** |
| `twist` 的 `angular` | ✅ **全 0**（`linear` 有噪声级读数）—— 与 [`VENDORED.md`](../../../src/vrpn_listener/VENDORED.md) §2b「角速度刻意留零、应从 IMU 取」**逐条对上** |

⟹ **ticket 03 第 3 条验收「对 z0 本体」现在补齐了**：`190.9 Hz`，落在 100–400 Hz 内。
**ticket 04 的前置条件（`/vrpn/z0/pose` 有数据）已满足，不再被阻塞。**

> ⚠️ **改话题名会连带影响的地方**：`params.yaml` **不用改**（它里面没有任何刚体名 —— 名字是自动发现的），
> 但**下游订阅者、ticket 04 的配置、任何文档里的 `/vrpn/Z0/pose`** 都要跟着改。
