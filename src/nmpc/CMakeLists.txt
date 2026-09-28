cmake_minimum_required(VERSION 3.8)
project(nmpc)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 纯 Python 包（与 bench 同形态）。
ament_python_install_package(${PROJECT_NAME})

# ★ 让 `ros2 run nmpc offboard_smoke.py` 能用：ros2 run 找的是 lib/<包名>/ 下的**可执行文件**，
#   而 ament_python_install_package 只装模块、不产生入口点。文件自带 shebang，
#   且开头有一段 `__package__ in (None, "")` 的引导，所以直接 `python3 .../offboard_smoke.py`
#   也能跑（不经 colcon 调试时用得上）。
install(PROGRAMS nmpc/offboard_smoke.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # ★ 两个文件都是**零 ROS**的纯逻辑测试（符号映射 + 指令时间轴），一毫秒跑完。
  #   test_sign_map 会去解析 docs/control/sign-map.md 的 `sign-map:canonical` 块 ——
  #   断言的是那份文档，不是实现里复述的口径（sign-map.md §8.5）。
  ament_add_pytest_test(test_sign_map test/test_sign_map.py
    TIMEOUT 60)
  ament_add_pytest_test(test_commands test/test_commands.py
    TIMEOUT 60)

  # ★ 离线 OCP（ticket 04）—— seam 2，同样**零 ROS**：把 `(状态, 参考) → u` 当作被测单元。
  #   它们回答的是"模型/代价/约束对不对"，与"话题上发了什么"（seam 1，ticket 06）是两回事。
  #   ⚠️ 需要 `ACADOS_SOURCE_DIR` 与 `LD_LIBRARY_PATH`，且首次运行会代码生成 + 编译 C
  #   （几十秒，之后复用 $MPC_CODEGEN_DIR）—— 所以 TIMEOUT 给得宽。
  ament_add_pytest_test(test_ocp_trim test/test_ocp_trim.py
    TIMEOUT 600)
  ament_add_pytest_test(test_ocp_constraints test/test_ocp_constraints.py
    TIMEOUT 600)
  ament_add_pytest_test(test_ocp_cost_ref test/test_ocp_cost_ref.py
    TIMEOUT 900)
  ament_add_pytest_test(test_ocp_health test/test_ocp_health.py
    TIMEOUT 600)
  ament_add_pytest_test(test_ocp_timing test/test_ocp_timing.py
    TIMEOUT 900)
endif()

ament_package()
