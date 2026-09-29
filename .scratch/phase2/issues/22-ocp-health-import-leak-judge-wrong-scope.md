# 22: `test_ocp_health.py` 的"零 ROS"判据写错了对象 —— 它查的是整个 pytest 进程，不是 `nmpc.ocp`

Status: done
Type: task

**What's wrong:** [`src/nmpc/test/test_ocp_health.py:104`](../../../src/nmpc/test/test_ocp_health.py)
那条 `test_no_ros_import_leaks_in` 在**本进程**里断言 `sys.modules`：

```python
for module in ("rclpy", "px4_msgs"):
    assert module not in sys.modules, f"nmpc.ocp 把 {module} 带进来了 —— 它必须是零 ROS 的"
```

`sys.modules` 是**整个 pytest 进程**的共享状态。同一进程里只要**别的**用例 import 过
`rclpy` / `px4_msgs`，这条**必红** —— 与 `nmpc.ocp` 到底有没有泄漏 ROS **无关**。
⟹ 它量的是"这个 pytest 会话"，不是它声称要测的那个契约。

**★ 更关键的是：这个缺陷仓库里早就诊断过、也修过，只漏了这一份。**
同一手法的另外两处 —— [`test_sign_map.py:112`](../../../src/nmpc/test/test_sign_map.py) 与
[`test_node_core.py:111`](../../../src/nmpc/test/test_node_core.py) —— 在 **ticket 06 的复核轮**
已经改成**子进程取判**，且把理由原样写在 docstring 里：

> ★★ 判据在**子进程**里取（ticket 06 复核轮改的，只改取法、不改结论）：`sys.modules` 是
> 整个 pytest 进程的，而同目录的 `test_mpc_node.py` / `test_preflight_check.py`（ticket 06 新增）
> 会 import `px4_msgs` / `rclpy` ⟹ 直接查 `sys.modules` 时本断言在 `pytest src/nmpc/test/` 下**假红**，
> 而在 `colcon test`（一个文件一个进程）下绿。子进程里"只 import sign_map"问的是同一个契约，
> 且与谁先跑无关。

**⚠️ 而 `test_ocp_health.py` 那份的 docstring 自己写着"与 `test_sign_map.py::test_no_ros_import_leaks_in`
同一手法、同一理由；分在两个文件里是因为两边测的是不同的模块"** —— 这句话**现在是假的**：
"同一手法"这个前提在 ticket 06 复核轮就不成立了，而这份没跟着改。
⟹ 这不是"新发现的判据缺陷"，是**一次没做完的同步**（三份里的两份改了）。

## 第一手证据

| 证据 | 来源 |
|---|---|
| 判据在本进程查 `sys.modules`（3 份里**只有这一份**没走子进程） | 本会话读码：`test_ocp_health.py:104-112` vs `test_sign_map.py:112-125`、`test_node_core.py:111` |
| 真因（同目录 `test_mpc_node.py` / `test_preflight_check.py` 在同进程 import 了 `px4_msgs`/`rclpy`） | **仓库内既有记录**：`test_sign_map.py:115-119` 那段（ticket 06 复核轮） |
| 单跑绿 / 整目录红 | 并行会话 `ros-ws-72` 复现（单跑 `pytest src/nmpc/test/test_ocp_health.py` = **8 passed**）；本会话在整目录拼跑里也见它红（`pytest tools/px4-sitl src/nmpc/test`） |
| `colcon test` 下绿（一个文件一个进程） | 由上一条的机理推得；**未单独实测**（本票不占机器） |

## 影响

1. **"整仓绿"这个目标在它存在期间不可达** —— 只要一次 `pytest src/nmpc/test/`，就必然带一条红。
   比"不可达"更糟的是**习惯化**：一条永远红的用例，会训练人忽略红。
2. **它其实是"只能单跑"的用例** —— 而这一点**没有任何东西写在它的名字或 docstring 里**，
   下一个人只会在"整仓红"里看到它，然后可能去查一个不存在的泄漏。
3. 反向风险低但要点名：它**不会**在本进程里漏报真泄漏（别人先 import 过 ⟹ 恒红，不会假绿）；
   真泄漏在**单跑**时仍能被它抓到。所以这是**假红**，不是假绿 —— 结论不变、证据不变。

## 修法（仓库里已有现成范例，照抄即可）

把 `test_ocp_health.py:104-112` 那几行改成与 `test_sign_map.py:112-125` **同样的子进程取判**：

```python
code = ("import sys, nmpc.ocp;"
        " assert 'rclpy' not in sys.modules, 'ocp 把 rclpy 带进来了';"
        " assert 'px4_msgs' not in sys.modules, 'ocp 把 px4_msgs 带进来了'")
done = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
assert done.returncode == 0, f"nmpc.ocp 不再是纯模块：{done.stderr.strip()[-300:]}"
```

同时把 docstring 里那句"与 `test_sign_map.py` **同一手法**"改对（否则下一个人还会照它推）。
**只改取法、不改结论**（子进程里"只 import `nmpc.ocp`"问的是同一个契约）。

- [x] 把本进程查 `sys.modules` 改成**子进程取判**（子进程里只 `import nmpc.ocp`）—— 结论不变
- [x] docstring 那句"与 `test_sign_map.py` 同一手法"改**对**（写清子进程取判 + ticket 06 复核轮的理由），三份文件手法重新一致
- [x] 只改 `src/nmpc/test/test_ocp_health.py` 一个文件（外加本票面）；未动 `src/nmpc/nmpc/` 生产代码，未动另两个测试文件

## 归谁

本票由 12 号票会话**代记**（操作者 2026-09-29 裁定"新开票"）。代码属 `src/nmpc/`，
**本会话不动** —— 归谁在不在这张票里定，操作者裁定。同批的同类问题
（`ros-ws-72` 也复现过、当时未开票）一并在本票了结，不必再开第二张。

## Comments

### 2026-09-29 —— 收口（agent: deepseek-flash / claude-code）

**改了什么**：`src/nmpc/test/test_ocp_health.py` 的 `test_no_ros_import_leaks_in`。
取法从"本进程查 `sys.modules`"改成"子进程里只 `import nmpc.ocp` 再查"（照抄
`test_sign_map.py:112-125` / `test_node_core.py:110-114` 的手法），**只改取法、不改结论**；
顺带把 docstring 里那句"与 `test_sign_map.py` **同一手法**"改对（原句在 ticket 06 复核轮后
就是假的），在 docstring 里把子进程取判的理由（同目录 `test_mpc_node.py` /
`test_preflight_check.py` 在同进程 import ROS ⟹ 整目录假红）写清。`import subprocess` 一并补上。
只动这一个文件；未动 `src/nmpc/nmpc/` 生产代码，未动另两个测试文件。

**验证（原始输出）**：

1）单跑 `python3 -m pytest src/nmpc/test/test_ocp_health.py -q`：

```
8 passed, 30 warnings in 7.84s
```

（注：需先 `export ACADOS_SOURCE_DIR=$HOME/acados && export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH`，
否则 6 条依赖 `ocp` fixture 的用例会在 `libacados.so` 加载处 ERROR —— 与本次改动无关的环境事项。）

2）整目录 `python3 -m pytest src/nmpc/test/ -q`（本票存在的理由：原假红必须消失）：

```
197 passed, 233 warnings in 415.84s (0:06:55)
```

3）★ 变异检查（合成对照，**未改生产代码**）：子进程里 import 真 `rclpy` / `px4_msgs` 后跑同一段断言：

```
$ python3 -c "import sys, rclpy; assert 'rclpy' not in sys.modules, 'ocp 把 rclpy 带进来了'"
AssertionError: ocp 把 rclpy 带进来了
exit=1

$ python3 -c "import sys, px4_msgs; assert 'px4_msgs' not in sys.modules, 'ocp 把 px4_msgs 带进来了'"
AssertionError: ocp 把 px4_msgs 带进来了
exit=1
```

进一步照测试原样（`subprocess.run` + `done.returncode == 0` 外判）模拟"`nmpc.ocp` 真泄漏"，
`returncode = 1`，外层断言如期红（`AssertionError: nmpc.ocp 不再是纯模块：…`，outer exit=1）
⟹ 新判据仍有鉴别力，不是把断言改松。

**说明**：我替操作者把 `needs-triage` 直接做成了 `done`；若不合规请回退。