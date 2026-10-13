#!/usr/bin/env python3
"""把 mavros `setpoint_raw` 插件的 `thrust_scaling` 设成 `1.0` —— **起链时自动补的一步**。

    ros2 run bringup mavros_set_thrust_scaling.py
    ros2 run bringup mavros_set_thrust_scaling.py --timeout 30.0

═══ 为什么需要这一步（2026-10-13 实机定的案，见票 `phase4/13` 第六轮） ═══

`thrust_scaling` 的**声明默认值是 `NAN`**（`setpoint_raw.cpp:58`），而它只有在
`== 1.0` 时才是我们要的口径。它的正常设置途径是 `px4_config.yaml` 里的
`/**/setpoint_raw:` 块 —— **那条路在本机是死的**：

    plugin.cpp:34-37   插件节点建的时候写死 node_options.use_global_arguments(false)
                      ⟹ 进程级 `--params-file` **到不了插件节点**
    node.launch:24     那份 `<param from="…px4_config.yaml"/>` 只挂在**主节点**上

⟹ `px4_config.yaml` 里那 27 个 `/**/<插件>:` 块**全是死文本**。实机读数：
`ros2 param get /mavros/setpoint_raw thrust_scaling` = **`nan`**，而 yaml 里写着 `1.0`。
用**精确节点名**再单独喂一份 params-file，**仍是 `nan`** ⟹ 不是通配符的问题。

`setpoint_raw.cpp:294`：`ignore_thrust = req->thrust != 0.0 && std::isnan(thrust_scaling)`
⟹ `return`，**整帧丢弃（连 `body_rate` 一起）**，只打一句 5 s 节流的 ERROR。
我们下行的 `thrust = F/T_max ≈ 0.1589 ≠ 0` ⟹ **一帧都到不了飞控**（零操纵，且几乎无声）。

═══ 这个脚本做什么 / 不做什么 ═══

做：等 `/mavros/setpoint_raw/set_parameters` 出现 ⟹ 设 `thrust_scaling = 1.0` ⟹ **读回核对**。
   读回不是 1.0 就退出码 1 并打红字（不留"以为设上了"）。
不做：不解锁、不起飞、不切模式、不向飞控发任何帧。它只改 mavros 自己的缩放系数。
   ⛔ `run/z0.py` 的动作表**故意**不合入它（那张表不许有"改参数"这一类入口）——
     本脚本是给 `mavros.launch.py` 用的**起链补丁**，不是给操作者敲的入口。

★ 与飞机无关：飞控**不通电**照样能跑（插件节点在 mavros 起来时就有了，2026-10-13 实测）。

⚠️ **设不回 `nan`**：`ros2 param set` 把 `nan` 当字符串（报 "Wrong parameter type"）⟹
  一旦设过，那个 mavros 进程就固定是 1.0。想回到 `nan` 只能重启 mavros。
"""

from __future__ import annotations

import argparse
import sys

NODE = "/mavros/setpoint_raw"
PARAM = "thrust_scaling"
TARGET = 1.0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="把 mavros setpoint_raw 的 thrust_scaling 设成 1.0")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="等 mavros 插件节点的上限（秒）。mavros 起全要几秒，别设太小")
    ap.add_argument("--node", default=NODE, help=f"目标节点全名（默认 {NODE}）")
    # ★ 必须 `parse_known_args`：`launch_ros.actions.Node` 起这个脚本时会**自动追加**
    #   `--ros-args [--params-file …]`。用 `parse_args` 会当场退 2
    #   （2026-10-13 实跑抓到的：进程 exit 2、launch 里一句 `unrecognized arguments: --ros-args`）。
    args, _ = ap.parse_known_args(argv)

    import rclpy                                          # noqa: PLC0415
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: PLC0415
    from rcl_interfaces.srv import GetParameters, SetParameters              # noqa: PLC0415

    rclpy.init()
    node = rclpy.create_node("mavros_set_thrust_scaling")
    try:
        setter = node.create_client(SetParameters, f"{args.node}/set_parameters")
        if not setter.wait_for_service(timeout_sec=args.timeout):
            print(f"✗ 等不到 {args.node}/set_parameters（{args.timeout:.0f} s）⟹ mavros 没起全。"
                  f"先 `ros2 launch bringup mavros.launch.py`", file=sys.stderr)
            return 1

        req = SetParameters.Request()
        req.parameters = [Parameter(
            name=PARAM,
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=TARGET))]
        fut = setter.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
        if not fut.done() or fut.result() is None:
            print(f"✗ {args.node} 对 set_parameters 没应答（超时 {args.timeout:.0f} s）",
                  file=sys.stderr)
            return 1
        res = fut.result().results[0]
        if not res.successful:
            print(f"✗ 设 {args.node}/{PARAM} 失败：{res.reason}", file=sys.stderr)
            return 1

        # ★ 读回核对 —— "设过了"与"真是 1.0"是两件事，这一步才是判据。
        getter = node.create_client(GetParameters, f"{args.node}/get_parameters")
        req2 = GetParameters.Request()
        req2.names = [PARAM]
        fut2 = getter.call_async(req2)
        rclpy.spin_until_future_complete(node, fut2, timeout_sec=args.timeout)
        if not fut2.done() or fut2.result() is None:
            print(f"✗ 设完了但读不回 {PARAM} ⟹ 不敢说成了", file=sys.stderr)
            return 1
        pv = fut2.result().values[0]
        if pv.type != ParameterType.PARAMETER_DOUBLE or pv.double_value != TARGET:
            print(f"✗ 设完读回 = {pv.double_value!r}（type {pv.type}）≠ {TARGET} ⟹ 没成",
                  file=sys.stderr)
            return 1
        print(f"✓ {args.node}/{PARAM} = {pv.double_value}（已读回核对）")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
