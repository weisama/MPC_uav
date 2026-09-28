# 18: `test_node_core.py` 没注册进 `colcon test`（55 个用例从来没跑过）

Status: done
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
  **新加的用例同样不会跑**。收口时报的 nmpc 通过数是 **136**，**不含这 55 个**。

**改法**（操作者 2026-09-28 裁定"现在补上并复验"）：

- [x] 最小修：`src/nmpc/CMakeLists.txt` 加一行
      `ament_add_pytest_test(test_node_core test/test_node_core.py TIMEOUT 300)`（落在
      `test_commands` 之后，带一段 ★ 说明它是哪一档、以及 ⚠️ 是被本票补上的）。
- [x] **已全仓扫过一遍**（`ament_add_pytest_test` 是显式列表，没有 glob）：逐包比对
      `src/*/test/test_*.py` 与对应 `CMakeLists.txt` ⟹ **只有这一个**漏注册。
      （`run/tests/`、`tools/*` 下的测试不走 colcon，各有自己的跑法，不在本条范围。）
- [x] **复验**（这条是关键，"注册上了"要看它真的进了进程）：

  ```
  colcon build --packages-select nmpc          # 9.4 s
  colcon test  --packages-select nmpc          # 6 min 14 s
  ```

  `build/nmpc/test_results/nmpc/` 从 9 个 xunit 变 **10 个**，新出现
  `test_node_core.xunit.xml: tests=55 errors=0 failures=0 skipped=0` ⟹
  nmpc 合计 **136 → 191**，全绿。
  ⚠️ 与别的文件合跑不冲突：那条"纯模块不许拉 ROS"的断言取在**子进程**里
  （`test_node_core.py:109-113`），与谁先跑无关。

**归口**：操作者裁定的原话是"现在补上并复验"，不是"挂回 06" ⟹ 改动落在本票名下，
`src/nmpc/CMakeLists.txt` 那一行已注明来自 ticket 18。

⚠️ 顺手订正一个数字：ticket 09 的收口 commit（`78f373d`）message 里写的"nmpc 145 通过"
是笔误，**实测是 136**（不含这 55 个）；本票注册后为 191。历史不动（未推送但已与并行会话
共享树），以这里为准。

相关：`src/nmpc/CMakeLists.txt` 那段 seam 分层注释（它描述的分层是对的，只是少了一行注册）。
