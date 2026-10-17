# `bag/` 清单（自动生成 —— 别手改；`run/z0.py rec_manifest` 重算）

★ 本表是**确定性输出**（不含生成时间）：重算一次必须逐字相同，好拿 `git diff` 当判据。

★ 本表**每个场景只列当前该用的那一份**。被后一次录制取代的旧包在 [`old/`](old/) 里，**不进本表** —— 它们**还在库里、还能回查**（谁取代了谁见 `bag/README.md`）。

| 场景 | 目录 | 起点（UTC） | 时长 | 大小 | 消息 | 话题 | sha256（包内容） |
|---|---|---|---|---|---|---|---|
| 1 静止基线（动捕+mavros，飞机不动） | `bag/1_static` | 2026-09-29T07:19:00Z | 117.3 s | 27.0 MiB | 755884 | 137 | `56ba3e09a9d5ac45a6a1f81f1810c7a8dbb9333d1ee662f0b38f893fe6d47b2b` |
| 2 定点模式缓慢推摇杆起飞 | `bag/2_manual_takeoff` | 2026-09-29T07:23:37Z | 72.8 s | 18.1 MiB | 506854 | 137 | `4aafd2dfd67b8d0f6ee2162276284a3d23ce7f8bd6081dd8bb5e73a470d25b02` |
| 3 执行轨迹 1（原地悬停 60 s） | `bag/3_traj1_hover` | 2026-10-08T09:39:42Z | 116.4 s | 25.5 MiB | 730941 | 146 | `03d905cb2aed6d05cefe4d6927367eaf8c18dfafdac6e97a1ac850c054a36fc9` |
| 4 执行轨迹 2（1 m 正方形） | `bag/4_traj2_square` | 2026-10-08T09:44:12Z | 36.9 s | 11.0 MiB | 311997 | 155 | `0b40e4afb1f8fc525965910fce1cccddefc01c53e911875573e45c7270c395f4` |
| 5 执行轨迹 3（1 m 八字） | `bag/5_traj3_lemniscate` | 2026-10-08T10:18:31Z | 49.6 s | 12.9 MiB | 367538 | 152 | `f7e191f739a665fc8c0a17ad7e3e1614176b5a50a2f4514e6596987a1bd5f33c` |
| 6 任意录制 —— 轨迹 4（起飞）· 模型预测控制 | `bag/6_free__20261017-020757` | 2026-10-17T02:08:00Z | 64.3 s | 11.5 MiB | 298675 | 174 | `20082507b6b827c3fcd2c22b89893688568551c99e3314bcd68d0b64d2e05b59` |
| 6 任意录制 —— 轨迹 1（悬停 60 s）· 模型预测控制 | `bag/6_free__20261017-070103` | 2026-10-17T07:01:03Z | 90.0 s | 15.4 MiB | 403341 | 144 | `e7d4a1fd7bf723b197af8008eaa362cf5a804d97643480b7097adfbd073e6c8e` |
| 6 任意录制 —— 轨迹 2（1 m 正方形）· 模型预测控制 | `bag/6_free__20261017-070446` | 2026-10-17T07:04:46Z | 38.0 s | 6.4 MiB | 170639 | 144 | `347bff1f46997efb19b72d203d01053ab48cef904d31ad4dfd49be0826fb8e13` |
| 6 任意录制 —— 轨迹 3（1 m 八字）· 模型预测控制 | `bag/6_free__20261017-070935` | 2026-10-17T07:09:35Z | 46.3 s | 8.2 MiB | 214528 | 144 | `e86424abbf8fe1fe9e20d83343adaae82734c640ba85e14cf95ac7631f80a53c` |

## 缺了什么（预期里一帧都没到的）

（一份都不缺。）

## 飞控日志（`.ulg`）

| 文件 | 大小 | sha256 |
|---|---|---|
| `bag/1_static/fcu/log_0_2026-9-29-15-22-20.ulg` | 9.36 MB | `10712c0f51e534a6d7861dbc738b1b7b33078db477dea95ba3156c22b686f372` |
| `bag/2_manual_takeoff/fcu/log_1_2026-9-29-15-25-14.ulg` | 5.52 MB | `1f8e76cd13f809894377388cfd6e20a5df91ea51b55c1008b0d7146f3bbcee8f` |
| `bag/3_traj1_hover/fcu/log_0_未知日期.ulg` | 12.22 MB | `3bfd90273cdc824c77bf93b4db065c654d89726290223e7bceb9ad507e851cda` |
| `bag/4_traj2_square/fcu/log_1_未知日期.ulg` | 5.45 MB | `bb2ff47c76cecad192efbe5262e389fc733221a682f9a14f1eaa17f655e91ce3` |
| `bag/5_traj3_lemniscate/fcu/log_4_未知日期.ulg` | 10.78 MB | `83de1da55495e6526e0e46732e5e2b2f63b1815cfbc5a3908038f9f6562ade8a` |
| `bag/6_free__20261017-070103/fcu/log_212_未知日期.ulg` | 8.68 MB | `be83e424d26a00a1321ddb4dbba07cee79d7bf13378a5be536613f09afe208e7` |
| `bag/6_free__20261017-070446/fcu/log_213_未知日期.ulg` | 4.78 MB | `5fc00c382d236e69f59be0b78f762d25b5a7cbd35c02cebf6eb8b2473144891e` |
| `bag/6_free__20261017-070935/fcu/log_214_未知日期.ulg` | 12.16 MB | `8b33509dee160d4b79d33b04c5a54ba25f92c96ebfe146badfdcfd7e0a946e67` |
