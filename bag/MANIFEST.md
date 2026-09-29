# `bag/` 清单（自动生成 —— 别手改；`run/z0.py rec_manifest` 重算）

★ 本表是**确定性输出**（不含生成时间）：重算一次必须逐字相同，好拿 `git diff` 当判据。

| 场景 | 目录 | 起点（UTC） | 时长 | 大小 | 消息 | 话题 | sha256（包内容） |
|---|---|---|---|---|---|---|---|
| 1 静止基线（动捕+mavros，飞机不动） | `bag/1_static` | 2026-09-29T07:19:00Z | 117.3 s | 27.0 MiB | 755884 | 137 | `56ba3e09a9d5ac45a6a1f81f1810c7a8dbb9333d1ee662f0b38f893fe6d47b2b` |
| 2 定点模式缓慢推摇杆起飞 | `bag/2_manual_takeoff` | 2026-09-29T07:23:37Z | 72.8 s | 18.1 MiB | 506854 | 137 | `4aafd2dfd67b8d0f6ee2162276284a3d23ce7f8bd6081dd8bb5e73a470d25b02` |
| 3 执行轨迹 1（原地悬停 60 s） | `bag/3_traj1_hover` | 2026-09-29T07:27:12Z | 105.0 s | 26.4 MiB | 742770 | 145 | `082a087bafb9e477e8fcb5f3a07a72fc0f61533d5ceb2c96df56716aefff11b1` |
| 4 执行轨迹 3（0.5 m 八字） | `bag/4_traj3_lemniscate` | 2026-09-29T07:31:19Z | 39.2 s | 10.5 MiB | 293832 | 148 | `131d8084792ed848d02ddca25459464fefa5792949e417c8da26bbd7e73d8c6e` |

## 缺了什么（预期里一帧都没到的）

（一份都不缺。）

## 飞控日志（`.ulg`）

| 文件 | 大小 | sha256 |
|---|---|---|
| `bag/1_static/fcu/log_0_2026-9-29-15-22-20.ulg` | 9.36 MB | `10712c0f51e534a6d7861dbc738b1b7b33078db477dea95ba3156c22b686f372` |
| `bag/2_manual_takeoff/fcu/log_1_2026-9-29-15-25-14.ulg` | 5.52 MB | `1f8e76cd13f809894377388cfd6e20a5df91ea51b55c1008b0d7146f3bbcee8f` |
| `bag/3_traj1_hover/fcu/log_2_2026-9-29-15-29-26.ulg` | 8.61 MB | `fb1a1751bff58f5a91625d12ea5b428c6b9ae71e87ce0ab8f4c4e393b7c92188` |
| `bag/4_traj3_lemniscate/fcu/log_3_2026-9-29-15-32-40.ulg` | 6.40 MB | `ead72549c16b1e49d9ec67af30209f9ac696d50ddfce9a30990c797df5470b75` |
