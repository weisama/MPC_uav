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

# ★ 同理装 Phase 2 的节点（ticket 06）：`Scenario(dut=("nmpc", "mpc_node.py"))` 找的正是
#   `lib/nmpc/mpc_node.py` 这个**可执行文件**（`install(PROGRAMS …)` **不改名**，
#   所以带 `.py`；`ros2 run nmpc mpc_node.py` 同理）—— 漏了它，seam 1 的集成测试会当场
#   `FileNotFoundError`（那条报错里写着"否则这个场景会静默地什么都没测"）。
install(PROGRAMS nmpc/mpc_node.py DESTINATION lib/${PROJECT_NAME})

# ★ 装桨之前的静态自检（ticket 06 的验收项）：`ros2 run nmpc preflight_check.py`。
#   它**不是**测试（那些在 `colcon test` 里），而是起飞流程里的一步 —— 按运行参数跑，
#   任何一条不过就退出码 1。⚠️ 安装成副本之后它离仓库根很远，那时要 `--repo <仓库根>`
#   （因为符号对账要读 docs/control/sign-map.md 那份**数值权威**）。
install(PROGRAMS nmpc/preflight_check.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # ★ 两个文件都是**零 ROS**的纯逻辑测试（符号映射 + 指令时间轴），一毫秒跑完。
  #   test_sign_map 会去解析 docs/control/sign-map.md 的 `sign-map:canonical` 块 ——
  #   断言的是那份文档，不是实现里复述的口径（sign-map.md §8.5）。
  ament_add_pytest_test(test_sign_map test/test_sign_map.py
    TIMEOUT 60)
  ament_add_pytest_test(test_commands test/test_commands.py
    TIMEOUT 60)

  # ★ 起飞前静态自检的测试（ticket 06）：同样是零 ROS 的纯逻辑（参数表 + 符号对账 +
  #   `ast` 扫描），但**每一组检查都配了负控** —— 一个"永远打 ✅"的自检脚本比没有更糟。
  #   ⚠️ 它会 import `nmpc.mpc_node`（要 rclpy/acados 在环境里）才拿得到参数表，
  #      但它**不建节点、不建求解器**（唯一真解一次的那档是 `preflight_check.py --solve`）。
  ament_add_pytest_test(test_preflight_check test/test_preflight_check.py
    TIMEOUT 300)

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

  # ★ **seam 1**（ticket 06）：起真的 DUT 子进程 + 假 PX4 夹具，断言话题上出现了什么。
  #   与被测逻辑（`test_node_core`）和求解器（`test_ocp_*`）分开是刻意的 ——
  #   这一层的失败要能一句话说清是"接口不对"还是"逻辑错了"。
  #   ⚠️ 它要 `bench` 包（test_depend 里声明了），且**每个用例都起一次 DUT**：
  #      首次会代码生成 + 编译 C ⟹ TIMEOUT 给得比其它文件更宽。
  ament_add_pytest_test(test_mpc_node test/test_mpc_node.py
    TIMEOUT 1800)
endif()

ament_package()
