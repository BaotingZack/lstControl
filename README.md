# 起重机 PD 速度控制仿真

桥式起重机三轴 PD 速度控制系统。支持**实验室仿真**和**真实 PLC 控制**两种运行模式。

- **X**：大行车（桥架），沿导轨前进方向
- **Y**：小行车（台车），沿桥架横移方向
- **Z**：吊钩（抓斗），高度方向

## 控制律

```text
v_cmd = Kp * (target_position - measured_position) - Kd * filtered_velocity
```

D 项使用位置差分后低通滤波得到的速度，避免原始差分速度抖动。

---

## 运行模式

通过 `--plc-ip` 自动切换：

| 命令 | 模式 | 位置来源 | 执行 |
|------|------|----------|------|
| `python3 main.py` | 实验室仿真 | 仿真对象模型 | 仿真对象 |
| `python3 main.py --plc-ip 192.168.1.100` | PLC 控制 | `/localization_pose` | 西门子 S7 PLC |

### 实验室仿真（默认）

```bash
python3 main.py --target-x 10 --target-y 5 --target-z 2 --live
```

不指定 `--plc-ip`，位置反馈来自仿真对象（含伺服滞后、扰动、测量噪声）。

### 真实 PLC 控制

```bash
python3 main.py --plc-ip 192.168.1.100 --live
```

指定 `--plc-ip` 后，位置反馈来自 ROS `/localization_pose` (nav_msgs/Odometry)，速度指令通过 `libsscarctrl.so` 发送到西门子 S7 PLC。

> ARM aarch64 + ROS Noetic 环境才能加载 `libsscarctrl.so`。x86 开发机会自动回退到 MockPLC（终端打印指令，不控制实物）。

---

## 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--target-x` | `8.0` | X 大车目标位置 (m) |
| `--target-y` | `6.0` | Y 小车目标位置 (m) |
| `--target-z` | `1.5` | Z 吊钩目标高度 (m) |
| `--live` | 关闭 | 启动浏览器实时回放 |
| `--hz` | `10.0` | 回放刷新率 (Hz) |
| `--speed` | `1.0` | 回放速度倍率 |
| `--host` | `127.0.0.1` | Web 服务器地址 |
| `--port` | `8000` | Web 服务器端口（占用时自动切换） |
| `--plc-ip` | 空 | PLC IP 地址（空 = 实验室仿真；指定 IP = PLC 控制模式） |
| `--plc-lib` | `plc_lib/lib/libsscarctrl.so` | PLC 库路径 |

---

## 浏览器界面

运行 `python3 main.py --live` 后打开 `http://127.0.0.1:8000`：

### 左侧画布（3 个面板）
- **Bridge Crane Bay**：XY 平面俯视图，显示桥架、台车、吊钩、路径轨迹
- **Trolley Movement**：Y 轴特写，台车沿轨道运动
- **Hoist Height**：Z 轴高度随时间变化曲线

### 右侧边栏
- **状态栏**：时间、帧数、回放速度
- **位置/速度**：X/Y/Z 实时位置和速度，速度带颜色编码（绿→琥珀→红）
- **目标指令**：输入新的 X/Y/Z 目标点，点击 Apply Target 重新仿真（连续路径，上次终点 = 下次起点）

### 右下角标签页

| 标签 | 内容 |
|------|------|
| **Progress** | 仿真进度条、各轴到达时间日志 |
| **Localization** | ROS `/localization_pose` 实时定位数据（位置、速度、时间戳） |
| **Control** | PLC 连接状态、心跳、轴速度/高度、夹爪状态、STOP ALL / Reset 按钮 |

---

## 代码结构

```text
main.py                仿真入口、CLI 参数、控制循环
crane_model.py         三轴状态、配置、伺服滞后、扰动和测量噪声
pd_controller.py       位置目标到速度指令的 PD 控制器
velocity_filter.py     位置差分 + 低通滤波估计速度
visualizer.py          静态控制曲线和作业示意图 (matplotlib)
live_server.py         浏览器 10Hz live view (HTTP + SSE + 内嵌 HTML/JS)
plc_interface.py       PLC 接口 (MockPLC / RealPLC + 心跳线程)
ros_bridge.py          ROS 桥接 (订阅 /localization_pose)
plc_lib/               C++ PLC 控制库 (libsscarctrl.so + libsnap7.so)
  demo.cpp              PLC 控制 demo (STOP ALL / 状态诊断 / 持续监控)
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
每个周期 (dt=0.01s, 100Hz):

  STEP 1  测量位置
            ├─ 实验室: plant.measure_position() + 噪声
            └─ PLC:    get_latest_pose() → /localization_pose

  STEP 2  速度滤波
            LowPassVelocityEstimator.update() → v_raw, v_filtered

  STEP 3  PD 控制
            PositionPDController.update() → vx_cmd, vy_cmd, vz_cmd

  STEP 4  执行速度指令
            ├─ 实验室: plant.update_axis() → 一阶伺服 + 扰动
            └─ PLC:    BigCarCtrl(vx) / SmallcarCtrl(vy) / liftctrl(height)
                       仅速度变化超过死区 (0.005) 时才发送 PLC 指令

  STEP 5  到位判定
            位置误差 < arrival_pos_tol 且 速度 < arrival_vel_tol → 锁定

  STEP 6  记录历史
```

### Z 轴特殊处理

PLC 的 `liftctrl` 接收绝对高度 (m)，而非速度。控制循环将 `vz_cmd` 积分为高度目标：

```text
z_target_height += vz_cmd * dt
clamp(z_target_height, 0, crane.z.position + 1.0)
plc.lift_ctrl(z_target_height)
```

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
| `velocity_filter_tau` | 0.05 | 速度滤波时间常数 (s) |
| `servo_time_constant_xy` | 0.18 | X/Y 速度环一阶响应 (s) |
| `servo_time_constant_z` | 0.12 | Z 速度环一阶响应 (s) |
| `arrival_pos_tol` | 0.005 | 到达判定位置容差 (m) |
| `arrival_vel_tol` | 0.002 | 到达判定速度容差 (m/s) |

---

## 测试

```bash
python3 -m pytest -q
```

## 依赖

- Python 3.8+
- matplotlib >= 3.6, numpy >= 1.23
- rospy + nav_msgs（仅 PLC / ROS 模式需要）
- libsscarctrl.so + libsnap7.so（仅真实 PLC 环境需要）
