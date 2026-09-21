# 03: Vendor `vrpn_listener` + ROS2 工作区骨架

Status: ready-for-agent
Type: task

**What to build:** 仓库里第一次出现 `src/`。把**上游那份定制过的** `vrpn_listener` 原样 vendor 进来并让它跑起来，使「动捕主机 → VRPN/UDP → ROS2 话题」这一段成为一条**已被验证、可复用的底座**。

同时建立 ROS2 工作区骨架（能 `colcon build`，构建产物不进 git）—— 后面每个包都建在它上面，所以它先做。

**来源是钉住的，不要换成公开版：**
必须是 `Huan-qun/Sim2real-UAV-Params-` 的 `S2R/assets/vrpn_listener.zip`，与 [`refs-manifest.md`](../../../docs/reference/refs-manifest.md) 记录的 commit（`b7aeebb`）对得上。**不是**公开的 `efc-robot/vrpn_client_ros2`。

**两件必须原样保留的事：**

1. **`stamp = now()` 是刻意设计，不是 bug。** 动捕主机与 Jetson 有 **~4 秒**时钟差，改成 VRPN 的 `msg_time` 会毁掉 EKF2 融合。这一行**连注释一起原样保留**。
2. **zip 里的 `README.md` 已过时**（它还在教人 clone 公开版）。**不要信它** —— 用 `VENDORED.md` 取代它的地位。

`VENDORED.md` 要写明：来源 zip 与 commit、`stamp = now()` 这个定制点与理由、以及**「勿替换为公开版」**。

**Blocked by:** 01（Vendor 完成后要跑起来，需要动捕 WiFi 通）

- [ ] `colcon build` 干净通过；构建产物（`build/` `install/` `log/`）不在 git 里
- [ ] `VENDORED.md` 存在，含来源 commit、定制点、"勿替换"三条
- [ ] `ros2 topic hz /vrpn/<刚体>/pose` 稳定 **~100–400 Hz**（动捕主机在线时）
- [ ] 刚体名由 VRPN **自动发现**，不需要在配置里写死
- [ ] **没有改动 vendored 源码的任何一行**（这是 vendored 的定义）
