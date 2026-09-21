# control 节点的一键启动。只起本节点 —— `vrpn_listener`、`mocap`、`mavros` 各自单独起。
#
# ★ 启动顺序（顺序错了本节点不会报错，只会停在 INIT，并从日志里告诉你缺什么）：
#   1) mavros 起来            → ros2 topic hz /mavros/state 有数（**1 Hz 是正常的**，见 yaml 注释）
#   2) vrpn_listener 起来     → ros2 topic hz /vrpn/<刚体>/pose 有数
#   3) mocap 节点起来         → ros2 topic hz /mocap/pose_enu 有数；确认 EKF2 已融合
#   4) 飞机摆好、静止，再起本节点
#
# ★ 本节点起来之后的动作（ticket 05 的全部内容）：
#   前置条件成立 → 以 50 Hz 流 setpoint（位置 = 当前位姿，即"原地保持"）
#                → 流满 offboard_prestream_s 后自动切 OFFBOARD
#                → 停在这里（`/control/state` = MANUAL/STANDBY），**不解锁、不动**。
#   观察：ros2 topic echo /control/state
#         ros2 topic hz /mavros/setpoint_raw/local
# ★ 退出 OFFBOARD 的两种方式：停本节点（断流 COM_OF_LOSS_T 后 PX4 自行退出）或遥控器接管。
#
# ★ 多台无人机复用：**改参数、不改代码**。
#   复制一份 yaml（如 config/control_z1.yaml），改话题名与服务名，然后
#     ros2 launch control control.launch.py params_file:=<那份 yaml> namespace:=z1
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = get_package_share_directory('control') + '/config/control.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='每台无人机一份 yaml：话题名、超时判据、setpoint 速率都在里面'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='多机时给实例加命名空间（yaml 里的话题名是绝对路径，不受影响）'),
        Node(
            package='control',
            executable='control_node',
            name='control',  # ← 与 yaml 里的 ros__parameters 键对应，不要改
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
