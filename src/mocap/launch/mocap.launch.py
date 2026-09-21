# mocap 节点的一键启动。只起本节点 —— `vrpn_listener` 与 `mavros` 各自单独起
# （见 docs/reference/mocap-setup.md §7 的验证顺序：先确认动捕有数据，再起本节点）。
#
# ★ 启动顺序与纪律（改错顺序或中途搬动飞机，都会让 EKF2 吃到错的位置）：
#   1) vrpn_listener 起来，ros2 topic hz /vrpn/<刚体>/pose 有数
#   2) 现场核对刚体名：ros2 topic list | grep vrpn  →  与 yaml 的 body_name 对齐
#   3) 把飞机摆好、静止，然后起本节点
#   4) ★ 从这一刻到解锁之间，不要再搬动/转动飞机（两阶段原点的切换量 = 这段时间动的量）
#
# ★ 多台无人机复用：**改参数、不改代码**。
#   复制一份 yaml（如 config/mocap_z1.yaml），改 body_name 与三个话题名，然后
#     ros2 launch mocap mocap.launch.py params_file:=<那份 yaml> namespace:=z1
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = get_package_share_directory('mocap') + '/config/mocap.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='每台无人机一份 yaml：刚体名与三个话题都在里面，换机只改 yaml'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='多机时给实例加命名空间（yaml 里的话题名是绝对路径，不受影响）'),
        Node(
            package='mocap',
            executable='mocap_rel_pose_node',
            name='mocap_rel_pose',  # ← 与 yaml 里的 ros__parameters 键对应，不要改
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
