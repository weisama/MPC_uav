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

```bash
git clone https://github.com/vrpn/vrpn.git
mkdir -p vrpn/build && cd vrpn/build
cmake .. && make -j8 && sudo make install
```

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

**获取方式**：从已配置好的机器上**打包整个 `vrpn_listener` 目录**再拷过来（用 U 盘 / scp / 共享挂载皆可）。

> 💡 **不用 `rsync` 的原因**（上游说明）：容器里通常没装 `rsync` / `ssh-client`，且容器到目标机的直连网络不一定通，**打包传输更稳**。

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
cd ~/ws_livox && colcon build --packages-select vrpn_listener
echo "source ~/ws_livox/install/setup.bash" >> ~/.bashrc
```

> ⚠️ **上游的工作空间路径是 `~/ws_livox`**（历史原因，来自 Livox 雷达项目）。**z0 应改成自己的路径。**

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

### 5.1 安装

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh   # 必须执行
```

### 5.2 ★ 用 launch 参数指定串口，**不要改共享文件**

```bash
ros2 launch mavros px4.launch \
  fcu_url:="/dev/ttyTHS3:921600" \
  gcs_url:="udp://@10.1.1.103"
```

> ⚠️ **不要改 `/opt/ros/humble/share/mavros/launch/px4.launch`** —— 那是 apt 管理的共享文件，**升级时会被覆盖**。一律通过 launch 参数传。

> 📌 **`fcu_url:="/dev/ttyTHS3:921600"`** —— 上游用的 Jetson 串口设备名是 **`/dev/ttyTHS3`**，波特率 921600（与 `z0.params` 里 `SER_TEL2_BAUD = 921600` 一致 ✅）。
> ⚠️ **z0 需要实测确认自己的设备名**（可能是 `ttyTHS0`/`ttyTHS1`/`ttyTHS2`，也可能是 USB 转串口的 `ttyUSB0`）。

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
| 1 | **刚体命名** | `r11` / `r22` | 改成自己的单机命名 |
| 2 | **串口设备** | `/dev/ttyTHS3` | ✅ **已实测**：z0（Jetson Orin NX）上**只有 `/dev/ttyTHS1` 和 `/dev/ttyTHS2`**，`THS3` 与 `THS0` 都不存在。接线时确认是哪一个 |
| 3 | **工作空间路径** | `~/ws_livox` | 改成 `~/ros_ws`（z0 的实际工作空间） |
| 4 | **`gcs_url`** | `udp://@10.1.1.103` | 改成自己的地面站 IP |
| 5 | **动捕网段** | `10.1.1.x` | ✅ **不需要额外网卡**。VRPN 就是 UDP over IP，**把机载电脑连到动捕主机所在的同一个 WiFi 即可**。z0 曾连 `SUSTech-802.1x`（`10.29.12.x/14`），到 `10.1.1.198` 的路由经校园网网关绕出、不通；换到动捕所在网络后即通。<br>⚠️ **但「那个 WiFi」到底叫什么，本文档至今没写** —— 这是本项唯一未闭合的信息，见下方 2026-09-21 实测 |
| 6 | **EKF2 参数** | 见手册第 3 节 | 用 [`../assets/z0.params`](../assets/z0.params)，见 [`params-analysis.md`](params-analysis.md) |
| 7 | **`EKF2_EV_POS_Z`** | `0` | ⚠️ z0 用铝柱架动捕球，**必须量出 Z 偏移** |
| 8 | **计时同步** | 未做 | 🔶 **可选，不再强制**。见 6.2：`stamp = now()` 是刻意设计，2–5 ms 抖动可接受。**只有想让 `msg_time` 可用时才必须补 NTP** |
| 9 | **动捕节点的 stamp** | `now()` 重新打戳 | ✅ 重写时**转发 `vrpn_listener` 的 stamp，不要二次打戳**（同一时钟域，见 6.2） |

**第 5、8、9 项已根据第一手证据更正**：第 5 项不是硬件问题而是"连对 WiFi"；第 8 项从"必须"降级为"可选"；第 9 项只是避免二次打戳，与 `msg_time` 无关。

> ★ **2026-09-21 实测（在 z0 自己的 Jetson `orin-nx` 上）：第 5 项仍未完成，且缺的具体是「SSID 名字」**
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
