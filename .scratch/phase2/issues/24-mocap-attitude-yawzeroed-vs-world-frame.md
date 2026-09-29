# 24: B 档的 `q` 与 `p`/`v` 不在同一个旋转系 —— `yaw_zeroed` 归零了姿态，模型却假定 `q` 是 body→世界

Status: needs-triage
Type: bug（**待定性**：可能只是约定没写清、也可能是 B 档真实的建模错配）

**What's wrong（一句话）：** `mocap` 节点在**阶段 B（解锁之后）**把姿态做了 `yaw_zeroed`
（左乘 `Rz(−yaw0)`），而 `/mocap/velocity_enu` 与 `/mocap/pose_enu` 的**位置**都是**未经旋转**的
世界系 ⟹ 进 `nmpc` 的 10 状态 `x = [p, v, q]` 里，`p`/`v` 在世界系、`q` 在世界系被转过
`−yaw0` 的系里。而模型 [`model.py:180-182`](../../../src/nmpc/nmpc/model.py#L180-L182) 写的是
`v̇ = R(q)·[0,0,F]/m + g`，`q_to_rot_mat` 的契约是**「把机体系矢量转到世界系」**、
`g = (0,0,−9.81)` 是 ENU ⟹ 模型**假定 `q` 与 `p`/`v` 同系**。这个假定在 B 档飞行时**不成立**。

**来源：** ticket 15 的 `/code-review` 复核（2026-09-29）。复核给的框法是
「两路进 MPC 的量不在同一个旋转系」，**这句话要收窄**（见下面"复核意见的订正"）：
出问题的**不是** twist 那条路（它与**位置**同系，见下），是**姿态**那一项。

⚠️ **这条不是 ticket 15 引入的**：`yaw_zeroed` 早于 ticket 15 就存在。
但 ticket 15 之前 B 档的 `v ≡ 0`（`PoseStamped` 没有 `twist` 字段），
本票给了 `v` 之后，模型里那条 `R(q)` ↔ `v` 的耦合**才真正承重**。

⚠️ **编号取 24**：`issues/` 里下一个空闲号（写入时最新为 `23-charclass-residue-float-crash.md`）。
**若并行会话已另开同题材的票，以先入库者为准、后到者并入，别留两张。**

---

## 事实（逐条第一手，2026-09-29 复核轮亲验）

### 1. 代码：姿态被旋转，位置与速度没有

`src/mocap/src/mocap_rel_pose_node.cpp` 的 `on_pose`（`:239`）：

```cpp
const mocap::Pose out = selector_->apply(in);   // → publish(vision_pose_pub_, …) / publish(pose_enu_pub_, …)
```

`selector_->apply`（`src/mocap/include/mocap/mocap_rel_pose.hpp:282-283`）：

```cpp
out.p = relative_position(pose.p, origin_.p);                              // p − o，**纯平移，无旋转**
out.q = (stage() == OriginStage::Arm) ? yaw_zeroed(pose.q, origin_.yaw)    // ★ 姿态被左乘 Rz(−yaw0)
                                      : pose.q;
```

`yaw_zeroed`（同文件 `:134`）—— **是真旋转，不是恒等**：

```cpp
inline Quat yaw_zeroed(const Quat & q_in, double yaw0)
{ return normalize(quat_mul(quat_from_yaw(-yaw0), normalize(q_in))); }
```

而 `on_twist`（`mocap_rel_pose_node.cpp:171-172`）**原样带走**：

```cpp
out.header = msg->header;   // frame_id 照抄 "world"
out.twist = msg->twist;     // 三分量原样，无旋转
```

⟹ `frame_id` 两边都是 `"world"`，但**`frame_id` 相同 ≠ 数据在同一旋转系**。

### 2. 模型：假定 `q` 与 `v` 同系

`src/nmpc/nmpc/model.py`：`v̇ = R(q)·[0,0,F]/m + g`（`:180-182`），
`q_to_rot_mat` 的 docstring 是「把**机体系**矢量转到**世界系**」（`:69`），
`g = (0,0,−9.81)`「ENU：重力朝 −z」（`:177`）。`node_core.pose_to_state` **不做任何坐标系转换**
（M7，见 `test_node_core.py::test_pose_to_state_不做任何坐标系转换`）。

### 3. 力学后果（**推导，未在实飞里量过**）

`Rz(yaw0)·ẑ = ẑ` ⟹ **竖直分量不受影响**；受影响的只有**水平推力方向**：
模型相信的水平推力方向，比真实方向**差了 `yaw0`**（解锁那一刻的机头朝向，任意值）。
`p`、`v` 两条与位置控制自洽（都在世界系），姿态环在"归零系"内也自洽
⟹ 不一致**只出现在 `R(q)` 与 `v` 之间的那条耦合**上。

### 4. 台架不受影响（**缩小爆炸半径**）

`src/bench/bench/vrpn_source.py` 发的是**原始** `PoseStamped` / `TwistStamped`，两路都没过
`yaw_zeroed` ⟹ bench 自洽。**受影响的只有真飞 B 档**，且 `yaw0 ≈ 0` 时看不出来。

---

## 复核意见的订正（先把话说准）

复核原文断言「`/mocap/pose_enu` 的姿态被 yaw-zeroed，而新转发的 `/mocap/velocity_enu` 是
原样世界系 —— 两路进 MPC 的量不在同一个旋转系」。**代码事实全对，框法要收窄**：

- 速度那条路**与位姿的「位置」部分同系**（`relative_position` 只做平移）⟹
  `ṗ = v` 这条**是自洽的**；
- 真正错配的是 **`q` ↔ (`p`,`v`)**。

这也是为什么 ticket 15 文档里那句「与 pose **同一个 frame** ⟹ 零变换」**用于速度是站得住的**：
它要的正是"速度与位置同系"。**但**读起来像在说"pose 整体是世界系"，而**姿态不是** ——
这句话的**歧义**本身就是本票要修的东西之一（见下）。

**复核未能静态证明实飞一定致偏**（需要真 `yaw0` 才能定），本票同样**不下结论**，
只把「文档断言的前提不成立、状态确系混帧」这一条钉住。

---

## 要裁定 / 要做的（**别自己拿主意**）

三条路，选一条（或先做一次实验定其量级）：

1. **让 twist 也过同一个 `Rz(−yaw0)`** —— 与姿态同系。代价：破坏 `ṗ = v` 的自洽，
   **看起来是错的**（本票倾向排除）。
2. **把位置也一起转**（真·输入侧零变换）：`out.p = Rz(−yaw0)·(p − o)`，
   整条状态都在"归零系"里 ⟹ 模型假定成立。代价：**改变 vision_pose 发给 PX4 的位置**
   （同一份 `out` 也发 `vision_pose_pub_`）⟹ 牵动 EKF2 与既有飞场数据，
   且操作者选目标点时用的坐标系要跟着说清。
3. **定性为"约定如此、无害"** —— 那就要**证明**：给出 `yaw0` 多大时误差可忽略、
   以及为什么闭环能吸收它；并把它写进 `sign-map.md` / `mpc-node-seam1.md` 的坐标系约定里。
4. （先做的实验）**在 SITL 上量这个错配**：把 `yaw0` 设成一个明显的值（如 90°），
   给同一目标点，看轨迹是否被系统性地转过去 —— 这是能**离线或 SITL 复现**的
   （`src/bench/bench/vrpn_source.py` 是纯逻辑源，可以给它加一个"带 yaw0 的 zeroing"档
   来**故意**造出这个错配，从而把量级量出来）。★ 这条最有价值：它把"未定性的建模问题"
   变成一个**可判定**的实验。

★ **无论选哪条**，`docs/control/mpc-node-seam1.md` §「同一个 frame ⟹ 零变换」与
`docs/control/phase2-implementation.md` 那条 `/mocap/velocity_enu` 的措辞都要改成**实测口径**
（说清"与**位置**同系"，而不是含糊的"与 pose 同框架"）。

---

## 验收

- [ ] 三选一（或路线 4 的实验）**由操作者裁定**，决定写回本文。
- [ ] 若判为"要修"：加一条**会红的**判据钉住「进模型的 `p`/`v`/`q` 同系」
      （例如给状态加一个"同系"自检，或造一个 `yaw0 ≠ 0` 的 bench 档，断言闭环把偏差吸收掉）。
      ★ 判据必须能区分"同系"与"差一个 `Rz(yaw0)`"——**只比均值/只看收敛会漏掉它**。
- [ ] `mpc-node-seam1.md` / `phase2-implementation.md` 的措辞改为实测口径。
- [ ] 若判为"无害"：把"为什么无害"连同 `yaw0` 的量级判据写进坐标系约定，别只写一句结论。

## Comments

### #1 开票记录（2026-09-29，ticket 15 的 `/code-review` 复核轮）

- 复核提出⇒我逐行回代码核实（上面"事实"四条都是亲验，命令与源码片段见正文）。
- **收窄了复核的框法**：不是"twist 与 pose 不同系"，是"`q` 与 (`p`,`v`) 不同系"；
  速度那条路与**位置**同系，所以 ticket 15 那句"同一个 frame"**用于速度是站得住的**。
- 定 `needs-triage` 而**不是** `ready-for-agent`：修法牵动 vision_pose / EKF2 / 飞场坐标系，
  属于「归谁、按哪个约定」的裁定，**不是**我能自己 promote 的（本仓库纪律）。
- ⚠️ 本条**同时点名**：ticket 15 的票面与文档措辞是否需要随之订正 —— 见 ticket 15 Comments ⑩。
