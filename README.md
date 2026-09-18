# MPC_uav

四旋翼无人机模型预测控制（MPC）相关文献与代码。

## papers/ 参考论文

| 文件 | 标题 | 出处 | 年份 |
| --- | --- | --- | --- |
| `Sun2022_TRO_MPC_DFBC.pdf` | A Comparative Study of Nonlinear MPC and Differential-Flatness-based Control for Quadrotor Agile Flight | IEEE Transactions on Robotics (T-RO) | 2022 |
| `RAL21_Torrente.pdf` | Data-Driven MPC for Quadrotors | IEEE Robotics and Automation Letters (RA-L) | 2021 |
| `ScienceRobotics22_Foehn_agilicious.pdf` | Agilicious: Open-Source and Open-Hardware Agile Quadrotor for Vision-Based Flight | Science Robotics, Vol. 7, Issue 67 | 2022 |
| `OmniNxt_A_Fully_Open-source_and_Compact_Aerial_Robot_with_Omnidirectional_Visual_Perception.pdf` | OmniNxt: A Fully Open-source and Compact Aerial Robot with Omnidirectional Visual Perception | IEEE/RSJ IROS | 2024 |
| `acado.pdf` | ACADO toolkit — An open-source framework for automatic control and dynamic optimization | Optimal Control Applications and Methods, 32:298–312 | 2011 |
| `1-s2.0-S0005109811003918-main.pdf` | An auto-generated real-time iteration algorithm for nonlinear MPC in the microsecond range | Automatica, 47:2279–2285 | 2011 |

## 论文速览

- **Sun2022_TRO_MPC_DFBC** — NMPC 与微分平坦控制（DFBC）的实证对比，速度至 20 m/s。结论：NMPC 在动力学不可行轨迹上更优，但计算耗时高、有数值收敛风险；两者加内环 INDI + 气动阻力模型后跟踪误差降低 78% 以上。
- **RAL21_Torrente** — 用高斯过程建模气动效应并嵌入 MPC，高速跟踪误差最多降低 70%，实测速度至 14 m/s、加速度超 4g。
- **ScienceRobotics22_Foehn_agilicious** — Agilicious 软硬件协同设计的开源敏捷四旋翼平台，支持模型与神经网络控制器，5g / 70 km/h 轨迹跟踪。
- **OmniNxt (IROS 2024)** — 全向视觉感知的开源紧凑型空中机器人平台，含 Nxt-FC 飞控与多鱼眼相机组。
- **ACADO toolkit** — 自动控制与动态优化的开源 C++ 框架，支持 MPC、状态/参数估计与符号表达式，可自动生成 C 代码。
- **Automatica 2011 (Houska et al.)** — ACADO 的自动生成实时迭代算法，把非线性 MPC 求解压到微秒级。
