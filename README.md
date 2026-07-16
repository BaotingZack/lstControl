# 起重机 PD 速度控制仿真

桥式起重机三轴 PD 速度控制系统。支持**实验室仿真**和**真实 PLC 控制**两种运行模式，通过策略模式共用 PD 算法核心。

- **X**：大行车（桥架），沿导轨前进方向
- **Y**：小行车（台车），沿桥架横移方向
- **Z**：吊钩（抓斗），高度方向

## 控制律

```text
v_cmd = Kp * (target_position - measured_position) - Kd * filtered_velocity
```

D 项使用位置差分后低通滤波得到的速度，避免原始差分速度抖动。两种模式使用完全相同的位置控制器和速度滤波器。

---

## 运行模式

通过 `--plc-ip` 自动切换：

| 命令 | 模式 | 位置来源 | 执行 | 交互 |
|------|------|----------|------|------|
| `python3 main.py` | 实验室仿真 | 仿真对象模型 (100Hz) | 仿真对象 | 命令行 target → 自动运行 → 回放 |
| `python3 main.py --plc-ip 192.168.1.100 --live` | PLC 控制 | ROS `/localization_pose` (10Hz) | `ctypes` → Snap7 → 西门子 S7 PLC | 开 UI → 实时定位 → Apply Target → 实时控制 |

### 实验室仿真（默认）

```bash
python3 main.py --target-x 10 --target-y 5 --target-z 2 --live
```

不指定 `--plc-ip`，位置反馈来自仿真对象（含伺服滞后、扰动、测量噪声）。`--live` 时浏览器回放已完成的仿真。

### 真实 PLC 控制

```bash
python3 main.py --plc-ip 192.168.1.100 --live
```

指定 `--plc-ip` 后：
1. 连接 PLC → 启动心跳 → 订阅 ROS `/localization_pose`
2. 等待首次定位数据获取初始位置
3. 启动浏览器 UI，实时显示定位数据
4. 用户输入 Target → 点击 **Apply Target** 开始 PD 控制
5. 控制过程中浏览器以 10Hz 轮询位置/速度/指令（SSE 保留为诊断通道）

> ARM aarch64 + ROS Noetic 环境才能加载 `libsscarctrl.so`。加载失败时程序默认拒绝进入 PLC 控制；开发机只有显式传入 `--allow-mock-plc` 才会使用 MockPLC。

### 通信边界：ROS 反馈，Snap7 控制

本项目不是把所有数据都通过 ROS 传输：

- **反馈链路走 ROS 1**：`ros_bridge.py` 订阅 `/localization_pose`（`nav_msgs/Odometry`），提供 X/Y/Z 位置和速度反馈。
- **控制链路不走 ROS**：`PlcActuator` 通过 Python `ctypes` 调用 `libsscarctrl.so`，底层使用 Snap7/S7 TCP 与西门子 PLC 通信。
- **人机界面走 HTTP**：浏览器通过 HTTP API 设置目标、停止/复位，并轮询控制状态。

```text
ROS /localization_pose ──► RosPositionSource ──► PD 控制循环
                                                   │
浏览器 HTTP ─────────────► Target / STOP            ▼
                                         PlcActuator / ctypes
                                                   │
                                         libsscarctrl + Snap7
                                                   │
                                              Siemens PLC
```

---

## 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--target-x` | `8.0` | X 大车目标位置 (m)（仿真模式） |
| `--target-y` | `6.0` | Y 小车目标位置 (m)（仿真模式） |
| `--target-z` | `1.5` | Z 吊钩目标高度 (m)（仿真模式） |
| `--live` | 关闭 | 启动浏览器实时回放（仿真）或实时控制（PLC） |
| `--hz` | `10.0` | 显示刷新率 (Hz) |
| `--speed` | `1.0` | 回放速度倍率（仅仿真模式） |
| `--host` | `127.0.0.1` | Web 服务器地址 |
| `--port` | `8000` | Web 服务器端口（占用时自动切换） |
| `--plc-ip` | 空 | PLC IP 地址（空 = 仿真；指定 IP = PLC 模式） |
| `--plc-lib` | `plc_lib/lib/libsscarctrl.so` | PLC 库路径 |
| `--allow-mock-plc` | 关闭 | 显式允许 PLC 库加载失败时使用 MockPLC |
| `--workspace-x-min/max` | 空 | 可选 X 轴机械工作区；必须成对设置 |
| `--workspace-y-min/max` | 空 | 可选 Y 轴机械工作区；必须成对设置 |
| `--workspace-z-min/max` | 空 | 可选 Z 轴机械工作区；必须成对设置 |

真实设备部署应按机械行程显式设置三轴工作区，例如：

```bash
python3 main.py --plc-ip 192.168.1.100 --live \
  --workspace-x-min 0 --workspace-x-max 30 \
  --workspace-y-min -10 --workspace-y-max 10 \
  --workspace-z-min 0 --workspace-z-max 15
```

所有目标都会拒绝 `NaN`/无穷值和负 Z 高度；配置工作区后还会拒绝越界目标。

---

## 浏览器界面

运行 `python3 main.py --live` 后打开 `http://127.0.0.1:8000`：

### 左侧画布（3 个面板）
- **Bridge Crane Bay**：XY 平面俯视图，显示桥架、台车、吊钩、路径轨迹
- **Trolley Movement**：Y 轴特写，台车沿轨道运动
- **Hoist Height**：Z 轴高度随时间变化曲线

### 右侧边栏
- **状态栏**：时间、帧数、速度
- **位置/速度**：X/Y/Z 实时位置和速度，速度带颜色编码（绿→琥珀→红）
- **目标指令**：输入 X/Y/Z 目标点，点击 Apply Target
  - 仿真模式：重新运行仿真并回放
  - PLC 模式：启动实时 PD 控制

### 右下角标签页

| 标签 | 仿真模式 | PLC 模式 |
|------|---------|---------|
| **Progress** | 仿真进度条、到达时间日志 | 到达事件日志 |
| **Localization** | 空 | ROS `/localization_pose` 实时数据 |
| **Control** | 空 | PLC 连接状态、心跳、轴速度/高度、STOP ALL / Reset |

---

## 架构

```
                        ┌─────────────────────────┐
                        │    PositionPDController  │  ← 共用
                        │  LowPassVelocityEstimator│  ← 共用
                        └──────────┬──────────────┘
                                   │
                    ┌──────────────┴──────────────┐
                    │     run_pd_control()         │  ← 统一控制循环
                    └──────────────┬──────────────┘
                                   │
              ┌────────────────────┼────────────────────┐
              │                    │                    │
    ┌─────────┴─────────┐  ┌──────┴──────┐  ┌─────────┴─────────┐
    │  PositionSource   │  │  Actuator   │  │  ControlHooks     │
    └─────────┬─────────┘  └──────┬──────┘  └─────────┬─────────┘
              │                    │                    │
    ┌─────────┴─────────┐  ┌──────┴──────┐    ┌───────┴───────┐
    │SimPositionSource  │  │PlantActuator│    │ LiveControlHooks
    │ (100Hz 非阻塞)     │  │ (一阶伺服)   │    │ (轮询+SSE诊断)  │
    └───────────────────┘  └─────────────┘    └───────────────┘
              │                    │
    ┌─────────┴─────────┐  ┌──────┴──────┐
    │RosPositionSource  │  │PlcActuator  │
    │ (ROS 反馈, 2s超时) │  │(ctypes/Snap7)│
    └───────────────────┘  └─────────────┘
```

## 代码结构

```text
main.py                仿真入口、CLI 参数、工厂函数
crane_model.py         三轴状态、配置、PositionSource/Actuator 接口、
                       仿真实现、统一 run_pd_control()
pd_controller.py       位置目标到速度指令的 PD 控制器
velocity_filter.py     位置差分 + 低通滤波估计速度
visualizer.py          静态控制曲线和作业示意图 (matplotlib)
live_server.py         浏览器 live view (HTTP + SSE + 内嵌 HTML/JS)
                       支持仿真回放和 PLC 实时控制两种模式
plc_interface.py       PLC 接口 (MockPLC / RealPLC / PlcActuator)
ros_bridge.py          ROS 桥接 (订阅 /localization_pose + RosPositionSource)
plc_lib/               C++ PLC 控制库
  demo.cpp              PLC 控制 demo
  lib/
    ss_car_control.h     PLC API 头文件
    libsscarctrl.so      ARM 发布版
    libsnap7.so          Snap7 (Siemens S7 协议)
tests/                 pytest 回归测试
```

`motion_planner.py` 和 `s_curve.py` 是旧阶段式/S 曲线代码，主流程不再使用。

---

## 控制流程

```
run_pd_control(source, actuator, target):

  while not all_locked:
    ├─ pos = source.get_position()     ← 模拟: 100Hz 非阻塞
    │                                   ← PLC:  10Hz 阻塞等待, 2s超时→安全停止
    ├─ raw_v, filtered_v = filter.update(pos, dt)  ← 共用
    ├─ v_cmd = controller.update(target, pos, filtered_v)  ← 共用
    ├─ actuator.apply(vx, vy, vz, dt)  ← 模拟: plant.update_axis()
    │                                   ← PLC:  BigCarCtrl/SmallcarCtrl/liftctrl
    ├─ actuator.update_state(crane, pos) ← 模拟: done by plant
    │                                     ← PLC:  pos←定位, vel←Odometry
    ├─ 到达检测 (位置+速度判定)
    └─ hooks.on_step(step_data)        ← PLC: 入队→SSE→浏览器
```

### Z 轴特殊处理

PLC 的 `liftctrl` 接收绝对高度 (m)，而非速度。`PlcActuator` 内部将 `vz_cmd` 积分为高度目标：

```text
z_target_height += vz_cmd * dt
z_target_height = max(0.0, z_target_height)
plc.lift_ctrl(z_target_height)
```

积分基准每个定位周期用 ROS 实际 Z 位置重新同步；节流比较的是高度设定值而不是 `vz_cmd`，因此恒定升降速度也会持续更新 PLC 高度目标。

### 停机与故障处理

- 正常到位：X/Y 显式下发零速，Z 保持当前高度。
- 定位 2 秒无新数据、控制超时、PLC 断线、心跳异常或未处理异常：统一执行 STOP ALL 并清理控制运行。
- STOP 请求先设置线程停止标志，再通过执行器命令锁下发最终零指令，避免停止后穿插新的运动命令。
- 同一时刻只允许一个 PLC 控制任务运行；定位断流和人工停止不会再被上报为“目标到达”。

---

## PLC 控制接口

基于西门子 S7 系列 PLC，通过 Snap7 协议通信。API：

| 函数 | 参数 | 说明 |
|------|------|------|
| `connect_to_plc(ip)` | IP 地址 | 连接 PLC，返回 0 成功 |
| `disconnect_plc()` | — | 断开连接 |
| `BigCarCtrl(v, ip, flag)` | 速度 m/s, IP, 控制字 | X 大车速度控制 |
| `SmallcarCtrl(v, ip, flag)` | 速度 m/s, IP, 控制字 | Y 小车速度控制 |
| `liftctrl(h, ip)` | 高度 m, IP | Z 吊钩高度控制 |
| `SendPlcHeartbeat(ip)` | IP | 心跳，10Hz，3 次连续失败断开 |
| `EmergencyBrake(clamp, ip)` | True/False | 紧急制动 |
| `ResetControl(clamp, ip)` | True/False | 复位 |
| `GetActualLiftHeight()` | — | 读取吊钩实际高度 |
| `CheckPlcConnection()` | — | 连接状态 |

---

## 10Hz vs 100Hz PD 适配

PLC 模式的 `/localization_pose` 以 10Hz 发布，与仿真模式的 100Hz 不同。为保持 PD 行为一致：

| 参数 | 仿真 (100Hz, dt=0.01s) | PLC (10Hz, dt≈0.1s) |
|------|------------------------|---------------------|
| 速度滤波 τ | 0.25s | 0.50s |
| `alpha = 1-e^(-dt/τ)` | ≈0.039 | ≈0.18 |
| 滤波响应 (63%) | ~85ms (8.5步) | ~100ms (1步) |

起重机机械时间常数（秒级）远大于 0.1s 采样间隔，10Hz 控制频率对位置环足够。

## ROS 话题

| 话题 | 类型 | 用途 |
|------|------|------|
| `/localization_pose` | `nav_msgs/Odometry` | 实时定位（位置 + 速度），PLC 模式下作为位置反馈 |

---

## 参数调节

主要参数在 `CraneConfig` 中：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_pos` | 0.6 | 位置比例增益 |
| `kd_pos` | 0.45 | 速度阻尼增益 |
| `max_velocity_xy` | 0.3 | X/Y 最大速度 (m/s) |
| `max_velocity_z` | 0.2 | Z 最大速度 (m/s) |
| `velocity_filter_tau` | 0.25 | 仿真模式速度滤波时间常数 (s) |
| `velocity_filter_tau_plc` | 0.50 | PLC 模式速度滤波时间常数 (s) |
| `servo_time_constant_xy` | 0.18 | X/Y 速度环一阶响应 (s)（仅仿真） |
| `servo_time_constant_z` | 0.12 | Z 速度环一阶响应 (s)（仅仿真） |
| `arrival_pos_tol` | 0.01 | 到达判定位置容差 (m) |
| `arrival_vel_tol` | 0.005 | 到达判定速度容差 (m/s) |
| `workspace_x/y/z_bounds` | `None` | 可选机械工作区；Z 即使未配置也不得小于 0 |

---

## 测试

```bash
python3 -m pytest -q
```

已在宿主环境及 20rocker（Ubuntu 20.04、Python 3.8.10、ROS Noetic）中执行同一测试集。

## 依赖

- Python 3.8+
- matplotlib >= 3.6, numpy >= 1.23
- rospy + nav_msgs（仅 PLC / ROS 模式需要）
- libsscarctrl.so + libsnap7.so（仅真实 PLC 环境需要）
