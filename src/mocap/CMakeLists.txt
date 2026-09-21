cmake_minimum_required(VERSION 3.8)
project(mocap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(mocap_rel_pose_node src/mocap_rel_pose_node.cpp)
target_include_directories(mocap_rel_pose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mocap_rel_pose_node
  rclcpp geometry_msgs std_msgs mavros_msgs)

install(TARGETS mocap_rel_pose_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # ★ seam 2（本项目唯一的纯函数单测）：相对位姿的三个纯函数。
  #   刻意**不**链接 rclcpp / 不调 ament_target_dependencies —— 这组测试必须零 ROS 依赖，
  #   它挡的是"飞起来才发现"的坐标类错误，见 .scratch/phase1/spec.md「seam 2」。
  ament_add_gtest(test_mocap_rel_pose test/test_mocap_rel_pose.cpp)
  target_include_directories(test_mocap_rel_pose PRIVATE include)
endif()

ament_package()
