# MPC_uav

> 📦 **归档说明（2026-09-18）**：这是 `weisama/MPC_uav` 仓库**原来的根 README**，现已被新的根 [`README.md`](../../README.md) 取代（论文文件名已在 2026-09-18 规范化）。
> 保留在这里只是为了记录仓库的初始形态。**论文索引以 [`papers/README.md`](papers/README.md) 为准**，项目文档以 [`../PROJECT.md`](../PROJECT.md) 为准。

四旋翼无人机模型预测控制（MPC）相关文献与代码。

## papers/ 参考论文

| 文件 | 标题 | 出处 | 年份 |
| --- | --- | --- | --- |
| `Sun2022_TRO_NMPC_vs_DFBC.pdf` | A Comparative Study of Nonlinear MPC and Differential-Flatness-based Control for Quadrotor Agile Flight | IEEE Transactions on Robotics (T-RO) | 2022 |
| `Torrente2021_RAL_DataDrivenMPC.pdf` | Data-Driven MPC for Quadrotors | IEEE Robotics and Automation Letters (RA-L) | 2021 |
| `Foehn2022_SciRob_agilicious.pdf` | Agilicious: Open-Source and Open-Hardware Agile Quadrotor for Vision-Based Flight | Science Robotics, Vol. 7, Issue 67 | 2022 |
| `Liu2024_IROS_OmniNxt.pdf` | OmniNxt: A Fully Open-source and Compact Aerial Robot with Omnidirectional Visual Perception | IEEE/RSJ IROS | 2024 |
| `Houska2011_OCAM_ACADOToolkit.pdf` | ACADO toolkit — An open-source framework for automatic control and dynamic optimization | Optimal Control Applications and Methods, 32:298–312 | 2011 |
| `Houska2011_Automatica_RTI.pdf` | An auto-generated real-time iteration algorithm for nonlinear MPC in the microsecond range | Automatica, 47:2279–2285 | 2011 |

## 论文速览

> 📖 **每篇的中文标注（研究内容 + 对本项目的用处）见 [`papers/README.md`](papers/README.md)。**

- **`Sun2022_TRO_NMPC_vs_DFBC`** — NMPC 与微分平坦控制（DFBC）的实证对比，速度至 20 m/s。结论：NMPC 在动力学不可行轨迹上更优，但计算耗时高、有数值收敛风险；两者加内环 INDI + 气动阻力模型后跟踪误差降低 78% 以上。
- **`Torrente2021_RAL_DataDrivenMPC`** — 用高斯过程建模气动效应并嵌入 MPC，高速跟踪误差最多降低 70%，实测速度至 14 m/s、加速度超 4g。
- **`Foehn2022_SciRob_agilicious`** — Agilicious 软硬件协同设计的开源敏捷四旋翼平台，支持模型与神经网络控制器，5g / 70 km/h 轨迹跟踪。
- **`Liu2024_IROS_OmniNxt`** — 全向视觉感知的开源紧凑型空中机器人平台，含 Nxt-FC 飞控与多鱼眼相机组。
- **`Houska2011_OCAM_ACADOToolkit`** — 自动控制与动态优化的开源 C++ 框架，支持 MPC、状态/参数估计与符号表达式，可自动生成 C 代码。
- **`Houska2011_Automatica_RTI`** — ACADO 的自动生成实时迭代算法，把非线性 MPC 求解压到微秒级。
