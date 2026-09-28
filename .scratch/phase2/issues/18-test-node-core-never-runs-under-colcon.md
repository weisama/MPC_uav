# 18: `test_node_core.py` 没注册进 `colcon test`（55 个用例从来没跑过）

Status: needs-triage
Type: bug

**现象**：`src/nmpc/test/test_node_core.py`（55 个用例，58 KB，**nmpc 里最大的纯逻辑测试**）
在 `src/nmpc/CMakeLists.txt` 里**没有注册** ⟹ `colcon test` 一帧都不跑它。

**第一手（2026-09-28，ticket 09 收口时发现）**：

- `grep -n "node_core" src/nmpc/CMakeLists.txt` 只有 **61 行的一句注释**（"与被测逻辑
  （`test_node_core`）…分开是刻意的"），**没有任何 `ament_add_pytest_test`**。
- `git log -S "ament_add_pytest_test(test_node_core" -- src/nmpc/CMakeLists.txt` 空 ⟹
  **从来没注册过**，不是被谁删掉的。
- `build/nmpc/test_results/nmpc/` 的 9 个 xunit 文件里没有它；
  `grep -c test_node_core build/nmpc/test_results/nmpc/test_commands.xunit.xml` = 0 ⟹
  也不是被别的文件顺手收集进去的（`test_mpc_node.py` 只在**注释/文档串**里提它，不 import）。
- 直接跑是绿的：`python3 -m pytest src/nmpc/test/test_node_core.py -q` ⟹ **55 passed（1.5 s）**。
  ⟹ 不是"跑不过所以摘掉了"，是**漏了接线**。

**为什么这条要紧**：好几处文档把它当作"判据所在处"，而这些判据实际上没有守卫：

- [06](06-mpc-node-fake-px4-fixture.md) 的复核轮第 10 条**就是按"`colcon test` 一个文件一个
  进程"来推的**（"`pytest src/nmpc/test/` 一跑就红，`colcon test` 照样绿"）—— 那个推理对
  `test_sign_map.py` 成立，对 `test_node_core.py` **前提不成立**（它根本不在那个集合里）。
- [09](09-sitl-tier-a-closed-loop.md) 的复核轮往这个文件里加了用例（`hold_feedforward`
  一侧的纯逻辑），`docs/control/offline-ocp.md` §4.4 与 `mpc-node-seam1.md` 也点名它 ⟹
  **新加的用例同样不会跑**。收口时"nmpc 145 通过"这个数是**不含这 55 个**的。

**候选改法**（等定归谁）：

- [ ] 最小修：`src/nmpc/CMakeLists.txt` 加一行
      `ament_add_pytest_test(test_node_core test/test_node_core.py TIMEOUT 300)`
      —— 必须在**同一个进程里先 import ROS 的那两个文件之前**的语义下仍然成立：06 已经把那两条
      "纯模块不许拉 ROS"的断言挪进子进程了，所以顺序不再是问题（**落地时要复跑一遍验证**）。
- [x] **已全仓扫过一遍**（`ament_add_pytest_test` 是显式列表，没有 glob）：逐包比对
      `src/*/test/test_*.py` 与对应 `CMakeLists.txt` ⟹ **只有这一个**漏注册。
      （`run/tests/`、`tools/*` 下的测试不走 colcon，各有自己的跑法，不在本条范围。）
- [ ] 归口：这条是 06 的漏，但 09 往里面加了用例 ⟹ 新票还是挂回 06 补齐，由操作者定。

相关：`src/nmpc/CMakeLists.txt` 第 59–66 行的那段注释（它描述的分层是对的，只是少了一行注册）。
