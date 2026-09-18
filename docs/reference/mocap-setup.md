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

> 📌 **mavros 会自动把 `/mavros/odometry/out` 的 ENU 转成 NED 给飞控。**

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

> `vrpn_listener` 的 `stamp` 用的是 `listener->now()`，**比真实采样时刻晚 10–100 ms 且带抖动**。
> **喂给 EKF2 反而引入偏差。**

**要改的前置条件**：
1. 先做 **NTP 同步**
2. 把 `stamp` 还原成 `msg_time`

---

### 6.1 ★★★ 这一条对本项目 MPC 路线的影响（重要）

上游只是"不转发速度"，但**根因是 `stamp` 有 10–100 ms 抖动** —— 这对 MPC 是**直接威胁**：

| 影响 | 说明 |
|---|---|
| **MPC 的延迟补偿会被削弱** | [`../control/mpc-roadmap.md`](../control/mpc-roadmap.md) 第 7 节计划显式补偿 50 ms 动捕延迟。**但如果时间戳本身抖动 10–100 ms，补偿的基准就是错的。** |
| **状态外推会失准** | MPC 用时间戳做状态外推时，抖动会直接变成位置误差 |
| **50 ms 这个数可能被低估** | 用户实测的 50 ms 是"动捕→IMU 四元数"的对比，**不含 stamp 抖动**。真实的最坏延迟可能是 50 + 100 = **150 ms** |

**→ 建议把"stamp 修复 + NTP 同步"提前到 MPC 开发之前做。** 这是本文件中**对项目影响最大的一条发现**。

**具体动作**：
1. 查 `vrpn_listener` 源码里 `stamp` 的赋值方式，确认是否真的是 `now()`
2. 若确认，改成用 VRPN 消息里的原始时间戳（`msg_time`）
3. 动捕主机与 z0 做 **NTP / PTP 同步**
4. 重新测一次端到端延迟，**测最坏值而不是平均值**

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
> **这一行重新打了一个时间戳，把动捕的原始时间戳丢掉了。**
> → **这正是第 6.1 节所说问题的一个具体实例。** 本项目做 MPC 时**必须改成转发原始 stamp**（或至少记录两者之差）。

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
| 2 | **串口设备** | `/dev/ttyTHS3` | ⚠️ **实测确认**（`ls /dev/ttyTHS*` / `ttyUSB*`） |
| 3 | **工作空间路径** | `~/ws_livox` | 改成自己的（Livox 是历史遗留） |
| 4 | **`gcs_url`** | `udp://@10.1.1.103` | 改成自己的地面站 IP |
| 5 | **动捕网段** | `10.1.1.x` | ⚠️ **★ z0 的 WiFi 在 `10.29.12.x/14`，有线口是 DOWN 的** → **需要额外网卡或调整网络规划**（[`../PROJECT.md`](../PROJECT.md) O2） |
| 6 | **EKF2 参数** | 见手册第 3 节 | 用 [`../assets/z0.params`](../assets/z0.params)，见 [`params-analysis.md`](params-analysis.md) |
| 7 | **`EKF2_EV_POS_Z`** | `0` | ⚠️ z0 用铝柱架动捕球，**必须量出 Z 偏移** |
| 8 | **计时同步** | 未做 | ⚠️ **★ 建议补 NTP/PTP，见第 6.1 节** |
| 9 | **`vrpn_relative_pose` 的 stamp** | 用 `now()` 重新打戳 | ⚠️ **★ MPC 版本必须转发原始 stamp** |

**第 5、8、9 项是本项目相对上游的额外要求**，因为 MPC 对时延与时戳的敏感度远高于 PID。
