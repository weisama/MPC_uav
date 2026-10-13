cmake_minimum_required(VERSION 3.8)
project(bringup)

find_package(ament_cmake REQUIRED)

# 本包**没有编译单元**：它是 launch + 一个 rclpy 脚本（+ README）。
# install(DIRECTORY launch) 让 `ros2 launch bringup z0.launch.py` 找得到，
# install(PROGRAMS scripts/fly.py) 让 `ros2 run bringup fly.py` 找得到（并保留 +x）。
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/fly.py DESTINATION lib/${PROJECT_NAME})
# ★ 起链补丁（票 13 第六轮）：把 mavros `setpoint_raw` 的 `thrust_scaling` 设成 1.0。
#   它的声明默认是 `NAN` ⟹ `thrust != 0` 的帧被 mavros **整帧丢掉**（零操纵、几乎无声）。
#   ⚠️ 源文件必须是 **100755**（symlink-install 下可执行位在源文件上，不在 install/ 里）。
install(PROGRAMS scripts/mavros_set_thrust_scaling.py DESTINATION lib/${PROJECT_NAME})

# ★ 没有 `if(BUILD_TESTING)`：本包的可验部分全在别处（control 的关节点用例、traj 的几何单测）。
#   在这里补一条"launch 能不能展开"的测试只会重复 `ros2 launch bringup z0.launch.py --show-args`
#   已经能做的事。

ament_package()
