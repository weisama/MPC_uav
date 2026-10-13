# z0 的 mavros 启动 —— 上游 `px4.launch` 的**一层薄包装**，只钉死 z0 的两个启动参数。
#
#     ros2 launch bringup mavros.launch.py
#     ros2 launch bringup mavros.launch.py gcs_url:="udp-b://0.0.0.0:14555@:14550"
#
# ★★ 为什么是"薄包装"而不是自己写一个 node.launch 的 include：
#   上游 `px4.launch` 里 `pluginlists_yaml` / `config_yaml` 两项是**写死的**
#   （`$(find-pkg-share mavros)/launch/px4_config.yaml`，见 `mavros/launch/px4.launch`），
#   没有暴露成 arg ⟹ **注入不了我们自己的 mavros 参数文件**。
#   而 z0 实际要改的两样（串口、GCS 桥）都是 **launch 参数**，不是 yaml 里的参数：
#     · `fcu_url` = `/dev/ttyTHS1:921600`（定案见 `docs/reference/mocap-setup.md` §6）
#     · `gcs_url` = 把整条 MAVLink 桥给笔记本上的 QGC（同文件 §5.4，含实测证据）
#   ⟹ `docs/control/phase1-implementation.md` 的目录图里那个 `config/mavros_z0.yaml`
#     **没有写**：没有任何东西会读它，而"多一个没人读的配置文件"比"没有它"更难查
#     （现场会以为改了它就生效）。这一条是**推翻文档结论**的地方，理由如上。
#
# ⚠️ mavros 装在一个**独立的工作空间**（`~/mavros_ws`，源码构建）。
#   起这一条之前先 `source ~/mavros_ws/install/setup.bash`（本包**故意**不把它写成依赖，
#   见 `package.xml`；`z0.launch.py` 的 `use_mavros` 默认值那三条理由同源）。
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    px4_launch = get_package_share_directory("mavros") + "/launch/px4.launch"
    return LaunchDescription([
        DeclareLaunchArgument(
            "fcu_url", default_value="/dev/ttyTHS1:921600",
            description="飞控串口。★ z0 定案 THS1 @ 921600（设备名是每台机器各自实测的，别照抄）"),
        DeclareLaunchArgument(
            "gcs_url", default_value="",
            description="空 = 不给地面站转发。给 QGC 用：udp-b://0.0.0.0:14555@:14550"),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(px4_launch),
            launch_arguments={
                "fcu_url": LaunchConfiguration("fcu_url"),
                "gcs_url": LaunchConfiguration("gcs_url"),
            }.items(),
        ),
        # ★★ 起链补丁（票 13 第六轮）：`thrust_scaling` 的声明默认是 `NAN`，而
        #    `px4_config.yaml` 里那个 `/**/setpoint_raw:` 块**进不了插件节点**
        #    （`plugin.cpp:35` 的 `use_global_arguments(false)`）⟹ 必须运行期补一刀。
        #    ⛔ 这一步**不是**"让飞机动"的入口：它只改 mavros 自己的缩放系数，
        #      不解锁 / 不起飞 / 不切模式 / 不发帧（`run/z0.py` 的动作表也**故意**不合入它）。
        #    ⚠️ 它自己会等插件节点出现（`--timeout`），失败时**只打红字、不拦 launch**：
        #      真正的闸门是 `nmpc/preflight_check.py` 的下行前置那一条（读**活值**）。
        Node(
            package="bringup",
            executable="mavros_set_thrust_scaling.py",
            output="screen",
            arguments=["--timeout", "30.0"],
        ),
    ])
