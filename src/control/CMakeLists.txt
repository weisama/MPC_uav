cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# `fsm.hpp` 是 header-only（零 ROS 依赖），所以只有两个编译单元。
# 刻意**不**把 px4_interface.cpp 拆成独立库：它只服务这一个可执行文件，
# 而"拆库"会把 ROS 边界摊到两个包里，反而让"谁负责接线"变得需要翻 CMake。
add_executable(control_node
  src/control_node.cpp
  src/px4_interface.cpp)
target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(control_node
  rclcpp geometry_msgs sensor_msgs std_msgs mavros_msgs)

install(TARGETS control_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # ★ seam 2（纯函数，零 ROS）：`fsm.hpp` 的守卫清单与前置流计时。
  #   ⚠️ 刻意**不**链接 rclcpp、不调 ament_target_dependencies —— 这组测试必须零 ROS 依赖。
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fsm test/test_fsm.cpp)
  target_include_directories(test_fsm PRIVATE include)

  # ★ seam 2（纯函数，零 ROS）：打杆判据。单独一份是因为它守着一个**极易读错的语义**
  #   （`COM_RC_STICK_OV` 是变化率，不是偏移量），见 test/test_rc_stick.cpp 头注释。
  ament_add_gtest(test_rc_stick test/test_rc_stick.cpp)
  target_include_directories(test_rc_stick PRIVATE include)

  # ★ seam 1（话题边界）：喂假 mavros + 脚本化动捕输入，**只断言外部可观测的话题**。
  #   与上面那份的分工见 test/test_fsm.cpp 的头注释：这里回答"飞机发了什么"，
  #   上面回答"是哪条守卫放行的"（四条守卫失效在话题上症状一样，分辨率到顶）。
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_control_standby test/test_control_standby.py
    TIMEOUT 240)

  # ★ seam 1（话题边界）：**失效路径**（ticket 06）—— 打杆夺权 / 四条输入判据 / 低压 / NaN /
  #   可复位 / 节点不崩。与上面那份分成两个文件：那份问"守卫成立时发了什么"，这份问
  #   "失效时**停止**了什么"，红的时候一眼能看出是哪一类（两者的夹具开销各自算）。
  ament_add_pytest_test(test_control_failsafe test/test_control_failsafe.py
    TIMEOUT 480)
endif()

ament_package()
