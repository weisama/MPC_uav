cmake_minimum_required(VERSION 3.8)
project(bench)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 纯 Python 包（夹具没有可执行文件，它被别的包的 pytest import）。
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # ★ 夹具自己也有测试：验它**假得像不像**（频率、QoS 兼容性、故障注入真的生效）。
  #   夹具骗不到 DUT 的前提是它先骗过自己 —— 一个把 local_position 发成 reliable 的假 mavros
  #   会让所有依赖它的测试出现"莫名其妙收不到数据"，而那种故障在测试日志里看不出来。
  find_package(ament_cmake_pytest REQUIRED)
  # 一个文件一个 pytest 进程（ament_add_pytest_test 的粒度就是文件）。
  ament_add_pytest_test(test_bench_selftest test/test_bench_selftest.py
    TIMEOUT 180)
  # 时序的纯逻辑单测：零 ROS，一毫秒跑完 —— 与上面那个刻意分开，好让"跑得快的那些"不被 ROS 拖慢。
  ament_add_pytest_test(test_timeline test/test_timeline.py
    TIMEOUT 60)
endif()

ament_package()
