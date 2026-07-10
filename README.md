# 起重机 PD 速度控制仿真

这是一个桥式起重机三轴控制仿真项目。调度系统只发送一个目标点
`(target_x, target_y, target_z)`，控制器根据位置反馈和滤波速度生成速度指令。

- X：大行车，沿导轨前进方向
- Y：小行车，沿桥架横移方向
- Z：吊钩，高度方向

控制律：

```text
v_cmd = Kp * (target_position - measured_position) - Kd * filtered_velocity
```

D 项使用由位置差分后低通滤波得到的速度，不直接使用抖动较大的原始 SLAM 速度。

## 功能

- 单目标点调度控制
- P + D 位置到速度控制
- 速度模式伺服一阶滞后
- 低频速度扰动和位置测量噪声
- 静态 plot 示意图输出
- 浏览器 10Hz 实时回放
- 浏览器中可输入新的 X/Y/Z 目标点重新仿真

## 安装

```bash
python3 -m pip install -r requirements.txt
python3 -m pip install -r requirements-dev.txt
```

## 运行

默认目标点为 `X=8.0, Y=6.0, Z=1.5`：

```bash
python3 main.py
```

运行后会生成：

- `crane_simulation.png`：控制曲线、速度、扰动、XY 轨迹
- `crane_operation_diagram.png`：行车作业示意图

指定目标点：

```bash
python3 main.py --target-x 4 --target-y 2 --target-z 2.2
```

## 实时显示

启动浏览器 live view：

```bash
python3 main.py --live
```

程序会打印类似地址：

```text
Live view: http://127.0.0.1:8000
```

打开该地址即可看到 10Hz 回放。页面右侧可以直接输入新的 X/Y/Z 目标点并点击
`Apply Target`，服务端会重新仿真并刷新回放。

也可以用 URL 参数指定目标：

```text
http://127.0.0.1:8000/?target_x=4&target_y=2&target_z=2.2
```

常用参数：

```bash
python3 main.py --live --hz 10 --speed 1.0 --host 127.0.0.1 --port 8000
```

如果端口被占用，程序会自动选择一个可用端口。

## 代码结构

```text
main.py               仿真入口和 CLI 参数
crane_model.py        三轴状态、配置、伺服滞后、扰动和测量噪声
pd_controller.py      位置目标到速度指令的 PD 控制器
velocity_filter.py    由位置差分得到速度，并进行低通滤波
visualizer.py         静态控制曲线和作业示意图
live_server.py        浏览器 10Hz live view
tests/                pytest 回归测试
```

`motion_planner.py` 和 `s_curve.py` 是旧阶段式/S 曲线代码，目前主流程不再依赖。

## 测试

```bash
python3 -m pytest -q
```

当前测试覆盖：

- PD 阻尼项使用滤波速度
- 单目标点能到达
- 扰动、测量噪声和伺服滞后生效
- plot 文件可生成
- live payload 按 10Hz 抽帧
- 浏览器 HTML 包含小车放大显示和目标输入
- 仿真超时保护

## 参数调节

主要参数在 `CraneConfig` 中：

- `kp_pos`：位置比例增益
- `kd_pos`：速度阻尼增益
- `velocity_filter_tau`：速度滤波时间常数
- `servo_time_constant_xy/z`：速度环一阶响应时间常数
- `disturbance_velocity_xy/z`：等效速度扰动幅值
- `measurement_noise_xy/z`：位置测量噪声标准差
- `arrival_pos_tol` / `arrival_vel_tol`：到达判定容差

默认配置偏向演示效果：能看到速度滞后、扰动和原始速度抖动，同时最终稳定到达目标点。
