# traj 节点的一键启动。**只起本节点** —— mavros / mocap / control 各自单独起。
#
# ★ 运行时的用法（不需要重启、不需要改代码）：
#     ros2 launch traj traj.launch.py          # 起节点（此时不发任何参考）
#     ros2 topic pub -1 /traj/select std_msgs/msg/UInt8 "{data: 2}"   # 选正方形
#     ros2 topic pub -1 /traj/start  std_msgs/msg/Empty "{}"          # 开始（参考时钟零点）
#     ros2 topic pub -1 /traj/select std_msgs/msg/UInt8 "{data: 0}"   # 清空
#   飞的时候这两条由 control 的代跑脚本发（见 ticket 09）；手敲只用于离线核对。
#
# ★ 与 control 的分工：本节点**不发 setpoint、不解锁、不进 OFFBOARD**。
#   它吐的参考是**相对轨迹原点**的；把参考加上轨迹原点、变成 setpoint，是 control 的事。
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = get_package_share_directory('traj') + '/config/traj.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='尺寸/限速/话题名都在 yaml 里；多机时换一份即可，代码不动'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='多机时给实例加命名空间（yaml 里的话题名是绝对路径，不受影响）'),
        Node(
            package='traj',
            executable='traj_node',
            name='traj',  # ← 与 yaml 里的 ros__parameters 键对应，不要改
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
