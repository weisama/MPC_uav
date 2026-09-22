# 17: 测试的 ROS 域隔离 —— `colcon test` 默认并行跑包 ⟹ 多份夹具撞在同一个域里

Status: needs-triage
Type: task

**为什么单开一张：** 2026-09-22 复核 ticket [06](06-control-failsafe-paths.md) 时，一整轮
**看起来是随机抽风的假红**查到最后发现不是被测代码的问题，而是**测试基础设施**的问题：
`colcon test` **默认并行跑包**，而 `src/bench/` 的夹具（假 mavros + 脚本化动捕 + 可选的 DUT 子进程）
把默认 ROS 域硬编码成 **73** ⟹ 两个包的 pytest **同时**往 `/mavros/state` 等话题上发假帧，
订阅方（DUT）收到**两股帧的混合**：`armed` 抖动、`mode` 乱跳、发布者计数变 2。
症状是**随机假红**（守卫被别包的帧打翻）或**随机假绿**（判据被别包的帧一直喂饱），
而 DUT 自己的日志**完全正常** —— 排查方向天然指向错的地方。

**第一手证据（同一份代码，同一天）：**

| 跑法 | 结果 |
|---|---|
| 整仓 `colcon test`（**默认并行**，`bench` 与 `control` 同时在域 73） | `control` 的 `test_control_standby` **9 条挂 5 条**（清一色 `MANUAL/STANDBY` + "没在发 setpoint"）；`bench` 的 `test_state_是_1Hz`（"3.4 s 内 2–3 帧"）测出 **7 帧** |
| 同一份测试**单独**跑 `BENCH_ROS_DOMAIN_ID=75` | **9 条全绿**；`test_state_是_1Hz` 稳定 3 帧 |
| 整仓 `colcon test --executor sequential` | **全绿**（ticket 06 的 92 例） |

另一条证据：`test_bench_selftest`（无 DUT、只发假帧）在 `colcon test` 里跑完之后，
`control` 的 DUT 立刻开始报 `MANUAL/STANDBY` —— 时间线对得上"上一位的夹具还在域里"。

**现状的临时做法（已写进 `src/bench/README.md` 的「安全边界」）：**

- 手动跑单文件：`BENCH_ROS_DOMAIN_ID=<非 0/73>`；
- 整仓跑：`--executor sequential`。
- ⚠️ **`BENCH_ROS_DOMAIN_ID` 对整仓跑没用**：它对**所有**包生效 ⟹ 两个包仍拿到同一个域，撞法不变。

**要做的事（方向未定，先记录选项）：**

- [ ] 定一个"每个包/每份测试默认独占一个域"的机制。候选：
      (a) 按**包名**派生默认域（`bench` 的 `pytest_support` 收一个"我是谁"的参数，
          `conftest.py` 传自己的包名）—— 解决跨包并行，但**同包并发两轮**（两个会话）仍会撞；
      (b) 按**进程**随机取一个空闲域（启动时探测 + 记录到日志）—— 连"两个会话跑同一个包"也解决，
          代价是"这次跑在哪个域"不再固定，排障时要看日志；
      (c) 保留现约定，只在文档/CI 里强制 `--executor sequential` —— 成本最低，但**挡不住两个会话**。
- [ ] 选定后**在夹具里加一条自检**：起 `Scenario` 前先看我们即将使用的话题上是否**已经有人**在发
      （`get_publishers_info_by_topic`；注意"陈旧租约"会给出假阳性，需要配合"等一帧看有没有真数据"），
      有 ⟹ **直接红并打印对方节点名**（这次排查一路都在手动做这件事）。
      ★ 这条比选哪个方案更重要：**把静默的假故障变成会红的一行**。
- [ ] 同步 `src/bench/README.md`（安全边界那节已写现状，机制定了再改）与
      `src/control/README.md` 的"构建与测试"（已写 `--executor sequential`）。
- [ ] ⚠️ 兼容性：改夹具默认域会影响**所有**用 `bench` 的包（`control` / `mocap` / 未来的 `traj`），
      落地方案时要顾着并发的开发会话（2026-09-22 当天就有一个并发会话在改 `src/bench/` 与 `src/traj/`）。

**Blocked by:** 无
**备注：** 本票**不改被测节点**；ticket 06 的结论不受它影响（ticket 06 的 92 例是在串行 + 独立域下取到的）。
