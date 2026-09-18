# LOCAL-NOTES —— 本机凭据与私有信息

> ⚠️ **本文件会随 `docs/` 一起提交到 [`weisama/MPC_uav`](https://github.com/weisama/MPC_uav)（private 仓库）。**
>
> **作者已明确决定：机载电脑系统密码 `123` 保留在本文中并上传**（2026-09-18）。该仓库为 **private**（未认证 HTTPS/API 均返回 404），且这是**离线内网开发机**的密码。
>
> ⚠️ **须知**：**一旦进入 git 历史，后续删除文件也清不掉它**——要彻底移除必须重写历史（`git filter-repo`）或轮换密码。若将来该仓库转为 public 或被加协作人，**必须先处理本节**。
> 缓解措施：Jetson 上**不要开 SSH 端口转发到公网**，密码只在内网使用。

---

## 1. 机载电脑（Jetson Orin NX）

| 项 | 值 |
|---|---|
| 主机名 / 用户 | `chen` |
| **系统密码** | **`123`** |
| 接入方式 | 双系统 Windows/Ubuntu 上的 VSCode Remote-SSH |
| `sudo` | 与登录密码相同（`sudo -S` 需交互，自动化脚本请勿依赖无密码 sudo） |

**已知影响**：

- `chen` **不在 `docker` 组**，`docker images` / `docker ps` 直接执行会被拒（`permission denied`）。需要 `sudo docker ...` 或把用户加入 docker 组。
  验证：`sudo -n true` 需要密码 → 无免密 sudo。
- 因此环境侦察时无法直接列出 Docker 镜像，只能通过 `journalctl` 与 `/proc/<pid>/root` 间接读取运行中容器。

## 2. 网络

| 接口 | 状态 | 地址 |
|---|---|---|
| `wlP1p1s0` | **UP（唯一在线）** | `10.29.12.59/14` |
| `enP8p1s0` | DOWN | — |
| `can0` | DOWN（mttcan 已加载） | — |
| `docker0` | DOWN | — |

| 用途 | 地址 |
|---|---|
| Nokov 动捕服务器 | `10.1.1.198:3883`（VRPN 端口，来自 `weisama-log/25.06.09_UAV_mocap/25.06.26_mocap.md`） |
| 旧机配套 IP 段 | `10.1.1.115/138/117/105/106`、`192.168.110.234/153/195/14/186` |
| 雷达 | `192.168.1.50`（口）/ `192.168.1.20`（雷达） |
| SLAM 模块热点 | `ssid: slam_mode` / `password: 12345678` |

> ⚠️ 动捕走 `10.1.1.x` 网段，而 Jetson 当前 WiFi 在 `10.29.12.x/14`。**要接动捕需要额外网卡或调整网络规划** —— 见 `PROJECT.md` 第 7 节开放问题。

## 3. GitHub SSH（2026-09-18 验证通过）

**结论：SSH key 已可用，两个仓库都能访问。**

| 项 | 值 |
|---|---|
| 私钥 | `~/.ssh/id_ed25519`（600 权限，2026-09-18 03:26 生成） |
| 公钥 | `~/.ssh/id_ed25519.pub` |
| `~/.ssh/config` | **不存在** —— 无 Host 别名，SSH 走默认 key |
| GitHub 身份 | **`weisama`** |
| `gh` CLI | **未安装**（`which gh` → not found） |

### 可复现的验证命令

```bash
# 1) 认证（退出码 1 是 GitHub 正常行为，不是错误）
ssh -o StrictHostKeyChecking=accept-new -T git@github.com
# → Hi weisama! You've successfully authenticated, but GitHub does not provide shell access.

# 2) 探测仓库可读性（不 clone，最快）
git ls-remote git@github.com:weisama/MPC_uav.git
# → 2cb9b14faaeef44e1ff49ba1689d7849b56e766b  HEAD
# → 2cb9b14faaeef44e1ff49ba1689d7849b56e766b  refs/heads/main

git ls-remote git@github.com:Huan-qun/Sim2real-UAV-Params-.git
# → refs/heads/main, refs/heads/update-docs, refs/pull/2..5/head

# 3) 浅克隆
git clone --depth 1 git@github.com:weisama/MPC_uav.git
```

### ★ 经验教训（重要）

**用 HTTPS / GitHub API 访问 private 仓库会返回 404，而不是 403。**

本次调查中，一个代理用 `https://api.github.com/repos/weisama/MPC_uav` 和 `https://github.com/weisama/MPC_uav` 探测，**两者都返回 404**，于是得出了「仓库不存在」的结论。但换成 SSH 后**立刻成功**。

原因：GitHub 对未授权的 private 仓库**故意返回 404 而非 403**，以避免泄露仓库是否存在。同时 `api.github.com/users/weisama` 会显示 `public_repos: 1`（只有 `slam_mode_book`），进一步误导。

**结论**：**在有 SSH key 的前提下，判断一个仓库能否访问，永远优先用 `git ls-remote git@github.com:<owner>/<repo>.git`，不要用 HTTPS 或 API。**

同样的问题也出现在 `Huan-qun/Sim2real-UAV-Params-` 上：HTTPS/API 路径全部 404，GitHub 全站搜索 `Sim2real-UAV-Params` 返回 `total_count: 0`（连搜索索引都不含 private 仓库），但 SSH 直接拿到了 923 行的 `uav1.params`。

## 4. Docker 环境

| 容器 | 镜像 | 用途 | 状态 |
|---|---|---|---|
| `noetic` | `dustynv/ros:noetic-desktop-l4t-r35.4.1` | **为必须跑 ROS1 的开源包准备**（Swarm-LIO2 / multi-MID360 实验） | 运行中（PID 13733） |
| `ubuntu22` → 改名 `px4` | `osrf/ros:humble-desktop-full` | **PX4 SITL 仿真**（PX4 1.13.3 + gazebo-classic） | **未在运行，需确认是否还在** |

`noetic` 容器启动参数：
```
--runtime=nvidia --net=host --privileged --user 1000:1000
-v /home/chen:/home/chen
-v /dev:/dev
```

容器内：Ubuntu 20.04.6 + ROS Noetic（237 个 share 包）+ gazebo11 + CUDA 11.4 + OpenCV + Eigen + `/workspace/ros_catkin_ws`

⚠️ **镜像 `l4t-r35.4.1` 是 JetPack 5，宿主是 `R36.5.0`（JetPack 6.2）—— 跨大版本。容器内 CUDA/Gazebo 是否真能工作需实测验证，不能假定。**

## 5. 用户笔记仓库

`/home/chen/weisama-log`（`git@github.com:weisama/log.git`）—— 中文技术笔记，按日期组织：

```
25.02.08_yolo_OrangPi_px4/    YOLO/ncnn + OrangePi + PX4 通信
25.02.27_px4_laerning/        ROS1 + PX4 + Gazebo-classic + mavros + offboard
25.04.10_wheeltec/            轮趣小车 / Arm 板
25.04.11_jetson/              Jetson Orin Nano 烧录
25.06.09_UAV_mocap/           ★ 动捕（最大重点，已实现定点飞行）
25.07.20_gazebo_docker/       Humble 容器仿真（PX4 SITL）
25.09.10_MID_cluster/         MID360 聚类
25.09.25_kalman/              KF/EKF/ESKF/ESIKF 理论自学
25.09.26_SLAM_mode/           ★ 最大目录（约 6000 行）
homework/                     ARMA/卡尔曼作业、电机阶跃响应两点法
orther/                       Linux / docker GPU / GitHub
```
