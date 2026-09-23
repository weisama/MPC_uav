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
find_package(std_srvs REQUIRED)
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
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs mavros_msgs)

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

  # ★ seam 2（纯函数，零 ROS）：**开始键的上升沿判据**（ticket 09）。
  #   单独一份的理由同上一份：它守着三个"在话题上完全同形"的坑
  #   （上电即高位被当成一次边沿 / 抖动没去抖 / 一条边沿被消费两次），
  #   见 test/test_rc_switch.cpp 头注释。
  ament_add_gtest(test_rc_switch test/test_rc_switch.cpp)
  target_include_directories(test_rc_switch PRIVATE include)

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

  # ★ seam 1（话题边界）：**起飞 → 悬停 → 自动降落**（ticket 08）—— 从 `/control/takeoff`
  #   到 `armed` 降沿的整条链，断言的是外部可观测的话题/服务（setpoint 的**目标**、模式请求、
  #   解锁请求、`/control/state`）。单独一份的理由：它比前两份都长（要真爬完 1 m 的斜坡），
  #   夹具开销与超时都该自己算。
  ament_add_pytest_test(test_control_takeoff test/test_control_takeoff.py
    TIMEOUT 480)

  # ★ seam 1（话题边界）：**开始键 → TRACK → 任务收口**（ticket 09）—— 与上一份的分工：
  #   上一份的入口是 `/control/takeoff` 服务，这一份的入口是**遥控器 ch9 的上升沿**
  #   （第 1 次 = 解锁起飞、第 2 次 = 进 TRACK），而且多一个同伴节点（假 `traj`）在说话。
  #   ⟹ 单独一份：它要把"按键"真的推两下，夹具开销与超时都该自己算。
  #   ⚠️ 它比前三份都长（每条用例都要先真爬完 1 m 再飞一段轨迹），480 s 是实测的 3 倍余量。
  ament_add_pytest_test(test_control_track test/test_control_track.py
    TIMEOUT 480)
endif()

ament_package()
