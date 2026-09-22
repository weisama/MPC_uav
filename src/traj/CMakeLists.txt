cmake_minimum_required(VERSION 3.8)
project(traj)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# `trajectories.hpp` 是 header-only（零 ROS 依赖）⟹ 只有一个编译单元。
# ★ 刻意**不**把它编译成库：只有一个可执行文件用它，拆库只会多一层 CMake 要找。
add_executable(traj_node src/traj_node.cpp)
target_include_directories(traj_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(traj_node
  rclcpp geometry_msgs std_msgs mavros_msgs visualization_msgs)

install(TARGETS traj_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # ★ seam 2（纯函数，零 ROS）：几何、尺寸、限速、完成度。
  #   ⚠️ 刻意**不**链接 rclcpp、不调 ament_target_dependencies —— 这组测试必须零 ROS 依赖：
  #      零依赖才跑得快（每次改公式都能秒级重跑），也才证明生成逻辑真的没有 ROS 耦合。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectories test/test_trajectories.cpp)
  target_include_directories(test_trajectories PRIVATE include)

  # ★ seam 1（话题边界）：选轨迹 / 开始 / 清空，**只断言外部可观测的话题**。
  #   ⚠️ 需要 bench 夹具，所以 `--packages-select traj` 单独跑会缺依赖；用
  #      `colcon test --packages-up-to traj`（或至少把 bench 一起选上）。
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_traj_node test/test_traj_node.py
    TIMEOUT 240)
endif()

ament_package()
