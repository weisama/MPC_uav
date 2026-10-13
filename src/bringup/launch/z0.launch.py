# z0 的一键 bringup（ticket 09）—— 一条命令起 **mocap + traj + control**（+ 可选的 vrpn_listener / mavros）。
#
#     ros2 launch bringup z0.launch.py                      # 现场常态：mavros 已在别的终端里跑着
#     ros2 launch bringup z0.launch.py use_mavros:=true      # 连 mavros 一起起（要先 source ~/mavros_ws）
#     ros2 launch bringup z0.launch.py use_vrpn:=false       # 动捕链路已经单独起过
#
# ★★ 这一条命令**不保证启动顺序**，也别指望它保证 —— 顺序的保证在**设计**里，不在 launch 里：
#   ROS 2 的 launch 把各 include 当成"同时"拉起（各是各的进程），而这几件事对顺序都不敏感：
#     · control 起早了 ⟹ 停在 `MANUAL/INIT`，日志**逐条列出缺什么**（飞控/动捕/local_position），
#       补齐之后自己往下走（`MANUAL/STANDBY` ⟹ 流 setpoint ⟹ 切 OFFBOARD）；
#     · traj    起早了 ⟹ 它一帧参考都不发，直到有人 `select` + `start`（那是 control 的事）；
#     · mocap   起早了 ⟹ 它照旧在启动那一刻锁"阶段 A 原点"（⚠️ 见下一条）。
#   ⚠️ 真正**对顺序敏感**的只有一件事，而且它在 launch 之外：
#      **飞机必须在起 mocap 之前就摆好并静止**（阶段 A 原点在节点启动时锁定，两阶段原点的切换量
#      = 这段时间里动的量）。所以正确用法是：**摆好飞机 → 起这一条 → 起完别再碰飞机**。
#
# ★ mavros **默认不起**（`use_mavros:=false`），三个理由（都不是偷懒）：
#   ① 它**独占** `/dev/ttyTHS1`（串口定案见 `docs/reference/mocap-setup.md` §6 那张表）。
#      现场常见的做法是它在一个专门的终端里长期跑着；再起第二个打不开串口，
#      而那个报错长得像"launch 坏了"，不像"串口被占了"。
#   ② 它是最慢、最有状态的一环，"起来了没有"要靠人看一眼 `/mavros/state`
#      （**1 Hz 是正常的**，见 `src/control/config/control.yaml` 里那段）——
#      把它混在一条命令里，这个核对点就没了。
#   ③ 它在**另一个工作空间**（`~/mavros_ws`，源码构建）⟹ 找不到它时不能连累其余三个起不来。
#
# ★ 起完之后**还差两步**（刻意不自动化）：
#   ① 选轨迹：`ros2 run bringup fly.py`（或 `ros2 topic pub -1 /traj/select std_msgs/msg/UInt8 "{data: 2}"`）
#   ② 起飞与开始轨迹：**遥控器开始键**，第 1 次上升沿 = 解锁起飞、第 2 次 = 进 `TRACK`
#      （§H.5；本包不给"程序起飞"的入口，因为它会绕开"开始键之前不发控制指令"这条防线）
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration

#: 飞控串口。★ 定案（ticket 01 ② / 13）：z0 只有 `/dev/ttyTHS1` 和 `/dev/ttyTHS2`，
#: 心跳只在 **THS1 @ 921600** 上探到（`docs/reference/mocap-setup.md` §6）。
#: ⚠️ **设备名是每台机器各自实测出来的**，换机器必须重新扫（上游那台用的是 ttyTHS3）。
FCU_URL = "/dev/ttyTHS1:921600"


def _py_launch(pkg: str, filename: str) -> PythonLaunchDescriptionSource:
    return PythonLaunchDescriptionSource(
        get_package_share_directory(pkg) + "/launch/" + filename)


def _xml_launch(pkg: str, filename: str) -> AnyLaunchDescriptionSource:
    # vrpn_listener 是 vendored 的 XML launch、mavros 是上游的 XML launch ⟹ 用 Any 那个源
    return AnyLaunchDescriptionSource(
        get_package_share_directory(pkg) + "/launch/" + filename)


def _setup(context):
    """在**launch 运行时**拼出 include 列表（而不是在描述构造时）—— 这样"找不到某个包"
    能被处理成一条说得清的话，而不是一个 import 期的 traceback。"""
    def val(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    def flag(name: str) -> bool:
        return val(name).strip().lower() in ("true", "1", "yes")

    namespace = val("namespace")
    actions = []
    started = []

    # ① mavros（可选，默认不起）
    if flag("use_mavros"):
        try:
            # ★★ 2026-10-13（票 13 第九轮）：**改指本仓的 `mavros.launch.py`，不再直接 include
            #   上游 `px4.launch`**。理由是那条路**绕开了起链补丁**：上游 `px4.launch` 把
            #   `config_yaml` 写死，而 `plugin.cpp:35` 的 `use_global_arguments(false)` 让那份 yaml
            #   到不了插件节点 ⟹ `thrust_scaling` 停在声明默认 `NAN` ⟹ 凡 `thrust != 0` 的 setpoint
            #   **被 mavros 整帧丢掉**（连 `body_rate` 一起），症状是**零操纵且几乎无声**。
            #   `bringup/launch/mavros.launch.py` 里多挂了 `mavros_set_thrust_scaling.py`
            #   把那一刀补上（设完**读回核对**）。
            # ⚠️ 这一句 `get_package_share_directory("mavros")` 是**探针**，保留它是为了那条
            #   "找不到 mavros 包"的友好提示 —— mavros 装在另一个工作空间，本包刻意不写依赖。
            get_package_share_directory("mavros")
            mavros = _py_launch("bringup", "mavros.launch.py")
        except PackageNotFoundError:
            mavros = None
        if mavros is None:
            actions.append(LogInfo(msg=(
                "⚠️ use_mavros:=true，但**找不到 mavros 包** —— 它装在另一个工作空间里，"
                "本包刻意不把它写成依赖（见 package.xml 的说明）。\n"
                "   修法二选一：① `source ~/mavros_ws/install/setup.bash` 之后再起这一条；"
                "② 干脆用默认的 use_mavros:=false，在别的终端里单独起它（现场常态）。\n"
                "   ⚠️ 这一条**照旧往下起** mocap/traj/control —— 但 control 会一直停在 "
                "`MANUAL/INIT`，它自己的日志会逐条列出缺什么。")))
        else:
            actions.append(IncludeLaunchDescription(mavros, launch_arguments={
                "fcu_url": val("fcu_url"),
                "gcs_url": val("gcs_url"),
            }.items()))
            started.append(f"mavros（fcu_url={val('fcu_url')}）")
    else:
        actions.append(LogInfo(msg=(
            "· mavros：**不起**（use_mavros:=false）—— 假定它已经在别的终端里跑着。"
            "核对：`ros2 topic hz /mavros/state`（**1 Hz 是正常的**）。")))

    # ② vrpn_listener（动捕数据源；可选，因为有人习惯单独起它）
    if flag("use_vrpn"):
        actions.append(IncludeLaunchDescription(_xml_launch("vrpn_listener", "sync_entity_state.launch")))
        started.append("vrpn_listener（话题：/vrpn/<刚体>/pose）")
    else:
        actions.append(LogInfo(msg="· vrpn_listener：**不起**（use_vrpn:=false）—— 假定它已经起了。"))

    # ③④⑤ mocap / traj / control —— 各自包里那份 launch 拥有自己的参数表，本文件不重复它们
    #      （params_file 非空才往下传：空 = 用那个包自己的默认 yaml）
    for pkg, filename, label, params_arg in (
            ("mocap", "mocap.launch.py", "mocap", "mocap_params"),
            ("traj", "traj.launch.py", "traj", "traj_params"),
            ("control", "control.launch.py", "control", "control_params")):
        launch_args = {"namespace": namespace}
        if val(params_arg):
            launch_args["params_file"] = val(params_arg)
        actions.append(IncludeLaunchDescription(_py_launch(pkg, filename), launch_arguments=launch_args.items()))
        started.append(label if not val(params_arg) else f"{label}（params_file={val(params_arg)}）")

    actions.append(LogInfo(msg=(
        "==== bringup 已拉起：" + "、".join(started) + " ====\n"
        "   ⚠️ 起这一条之前飞机就必须已经摆好并静止（mocap 的阶段 A 原点在它启动时锁定）。\n"
        "   下一步：① `ros2 run bringup fly.py` 选轨迹（1/2/3）；"
        "② 遥控器开始键第 1 次上升沿 = 解锁起飞、第 2 次 = 进 TRACK。")))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mavros', default_value='false',
            description='连 mavros 一起起（⚠️ 默认 false：它独占 /dev/ttyTHS1，现场常单独跑；'
                        'true 时要求 mavros 已在本机的 AMENT_PREFIX_PATH 上）'),
        DeclareLaunchArgument(
            'use_vrpn', default_value='true',
            description='起 vendored 的 vrpn_listener（动捕数据源；已单独起过就设 false）'),
        DeclareLaunchArgument(
            'fcu_url', default_value=FCU_URL,
            description='mavros 的飞控串口（只在 use_mavros:=true 时有意义）'),
        DeclareLaunchArgument(
            'gcs_url', default_value='',
            description='可选：把 MAVLink 桥给笔记本上的 QGC，例如 '
                        'udp-b://0.0.0.0:14555@:14550（用法与实测见 mocap-setup.md §5.4）'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='多机时给 mocap/traj/control 一起加命名空间'
                        '（⚠️ 各自 yaml 里的话题名是绝对路径，多机还要各带一份 yaml）'),
        DeclareLaunchArgument(
            'mocap_params', default_value='', description='空 = 用 mocap 自己的 config/mocap.yaml'),
        DeclareLaunchArgument(
            'traj_params', default_value='', description='空 = 用 traj 自己的 config/traj.yaml'),
        DeclareLaunchArgument(
            'control_params', default_value='', description='空 = 用 control 自己的 config/control.yaml'),
        OpaqueFunction(function=_setup),
    ])
