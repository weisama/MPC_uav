# `2_manual_takeoff/fcu/` —— 场景 2 的飞控日志

**这一份配的是同一个目录里的那份包**（`bag/2_manual_takeoff`，场景 2：定点模式手飞起飞）。
共用口径与「SD 原始路径」那四条反证写在 [`1_static/fcu/INDEX.md`](../1_static/fcu/INDEX.md) 与本目录
上一级的 [`bag/README.md`](../../README.md) —— 这里只记这一份自己的读数。

| 项 | 值 |
|---|---|
| 拷进来时的文件名 | `log_1_2026-9-29-15-25-14.ulg` |
| 大小 | 5.52 MB（5,522,252 B） |
| sha256 | `1f8e76cd13f809894377388cfd6e20a5df91ea51b55c1008b0d7146f3bbcee8f` |
| 飞控内部时间跨度 | 151.2 s（**开机相对**，不是 UTC） |
| 解锁区间 | 58.6 s → 115.9 s（**57.3 s**） |
| 最高点 | **1.449 m** |
| 固件 | `ver_sw 4a0e65f2331b23b9…`（`micoair743-v1.14.3`）、`ver_hw HKUST_NXT_DUAL` |

## 这一份为什么是场景 2（判据）

两条**互相独立**的量同时对上（差在毫厘级，而四条场景彼此差得很远，不可能认错）：

| | 包（`/mavros/local_position/pose` / `/mavros/state`） | 这份日志（`vehicle_local_position` / `vehicle_status`） |
|---|---|---|
| 最高点 | 1.450 m | 1.449 m |
| 解锁时长 | 57.0 s | 57.3 s |

## SD 卡上的原始路径

**`/fs/microsd/log/2026-09-29/15_25_14.ulg`（★ 推断 —— 不是从卡上读到的）**

反证见 [`1_static/fcu/INDEX.md`](../1_static/fcu/INDEX.md)。这一份的 `15-25-14` 落在场景 2 的包收口
（07:24:51Z）之后约 23 s，与"录完就拷日志"这个动作对得上 —— **自洽不等于核实**。
