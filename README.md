# 起重机 PD 速度控制仿真

桥式起重机三轴 PD 速度控制系统。支持**实验室仿真**和**真实 PLC 控制**两种运行模式，通过策略模式共用 PD 算法核心。

- **X**：大行车（桥架），沿导轨前进方向
- **Y**：小行车（台车），沿桥架横移方向
- **Z**：吊钩（抓斗），高度方向

## 近期重要更新（PLC 现场调试）

以下改动针对 PLC 模式现场测试中发现的问题，已合并进主分支：

| 类别 | 改动摘要 |
|------|----------|
| **PD 算法** | D 项优先使用 Odometry 原生 XY 速度；增加到位死区 (`arrival_pos_tol`) 与防反向抽动 (`reverse_guard_tol`)；X/Y 速度上限改为 **0.2 m/s** |
| **Z 轴反馈与控制** | Z 位置统一使用 PLC **`GetActualLiftHeight()`** 抓钩实测高度（UI 监视、目标预填、PD 反馈同一参考系）；`liftctrl` 为绝对高度伺服，PD 输出 `vz_cmd` 积分成高度设定值并**单调逼近目标**（不再每周期重锚到实测，避免 Z 几乎不动） |
| **方向修正** | 新增 `--invert-big-car` / `--invert-small-car` / `--invert-lift`，解决驱动正方向与定位轴相反的镜像问题 |
| **PLC 指令刷新** | 移除速度指令死区节流，**每 10Hz 周期**都下发 X/Y 速度与 Z 高度，避免看门狗停轴导致运动断断续续 |
| **安全高度** | `--min-lift-height`（默认 **0.5 m**）硬钳下发给 `liftctrl` 的绝对高度 |
| **到位判定** | 连续 **`arrival_debounce_cycles`**（默认 3 帧）满足条件才锁轴；丢弃单帧定位跳变/坏帧，防止“离目标十几厘米就停、PD 结束” |
| **运行容错** | 偶发 NaN/越界/跳变定位帧在预算内丢弃继续控制（`max_consecutive_bad_frames`，默认 10 帧）；心跳失败阈值放宽至 **10 次≈1s** 且线程**持续重试自愈**，避免一次网络抖动后心跳永久失效 |
| **UI** | PLC 模式下 `/localization/stream` 的 Z 也显示抓钩高度；PD **完成后冻结轨迹**（起点→目标），不再被实时定位 30 帧滑动窗口冲掉 |
| **目标 Z 参考系一致性** | 修复 Apply Target 输入的目标 Z 仍走 map↔crane **3D 旋转/平移**、而反馈 Z 已改用抓钩实测高度（不旋转/不平移）导致的参考系错位：一旦标定了 `origin_map_z` 或 roll/pitch，目标与反馈就会出现常数级偏差，PD 会朝错误高度收敛并在**远离真实目标**时就提前结束（v=0）。现在 Z 反馈来自抓钩高度时，目标 Z、实时轮询显示 Z 与控制反馈 Z 统一为同一物理量，不再经过旋转/平移 |
| **执行器瞬时故障容错** | `actuator.apply()` 因 PLC 连接/心跳瞬时抖动抛出的 `RuntimeError` 不再直接终止整段作业；预算内（`max_consecutive_actuator_errors`，默认 10 次≈1s）跳过本周期继续控制，超出预算才视为真实故障并安全停车 |
| **抓钩高度读数过滤** | `GetActualLiftHeight()` 在 PLC 连接抖动期间会返回 0 或 `4.49863e-312` 一类的 denormalized 垂圾值，导致 Z 出现"跳到 0 再跳回正确高度"的抽动；新增 `_LiftHeightSanitizer` 在 `RealPLC.get_lift_height()` 内过滤此类近零/不可能跳变/非数值读数，异常时沿用上一次可信读数，持续异常超过 2s 才放弃缓存值 |
| **停止阶段抗抖动调参** | 降低 `kp_pos`(0.4)、提高 `kd_pos`(0.55)、加大 PLC 速度滤波 `velocity_filter_tau_plc`(0.80 s)，并扩大 `arrival_pos_tol`(2.5 cm 死区)、`reverse_guard_tol`(10 cm 防反向)、到位捕获窗口——减少大车停止时速度指令频繁换向导致的抓钩摆动 |

详细说明见下文「控制律」「Z 轴特殊处理」「常见问题」与「参数调节」各节。

### 更新历史

| 日期 | Commit | 主要改动 |
|------|--------|----------|
| 2026-07-23 | [`18f404a`](https://github.com/BaotingZack/lstControl/commit/18f404a) | **停止阶段抗抖动调参**：`kp_pos` 0.4、`kd_pos` 0.55、`velocity_filter_tau_plc` 0.80 s；扩大 `arrival_pos_tol`(2.5 cm)、`reverse_guard_tol`(10 cm)、到位捕获窗口；`main.py` 不再覆盖 Kp/Kd |
| 2026-07-23 | [`9af12f3`](https://github.com/BaotingZack/lstControl/commit/9af12f3) | **README 更新历史**：汇总近期三次功能 commit |
| 2026-07-23 | [`4aa6314`](https://github.com/BaotingZack/lstControl/commit/4aa6314) | **抓钩高度读数过滤**：`_LiftHeightSanitizer` 过滤 S7 抖动期间的 0/垂圾值/不可能跳变；README 补充 `libsscarctrl.so` 心跳超时日志含义解读 |
| 2026-07-23 | [`f5e14e3`](https://github.com/BaotingZack/lstControl/commit/f5e14e3) | **目标 Z 参考系一致性**：`map_to_crane_target(z_is_hoist_height=True)` 让目标/显示/反馈 Z 统一为抓钩物理高度；**执行器瞬时故障容错** `max_consecutive_actuator_errors` |
| 2026-07-23 | [`a704d41`](https://github.com/BaotingZack/lstControl/commit/a704d41) | **PD 稳定性**：到位去抖、定位坏帧容忍、心跳自愈；**Z 轴**统一抓钩高度反馈 + 绝对高度伺服单调逼近；方向修正、0.2 m/s 限速、UI 轨迹冻结、安全高度下限 |

## 快速运行

### 1. 准备 Python 环境

项目要求 Python 3.8 或更高版本。在宿主机首次运行时执行：

```bash
cd /home/big/workspace/lstControl
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

后续重新打开终端时，只需进入项目并激活环境：

```bash
cd /home/big/workspace/lstControl
source .venv/bin/activate
```

### 2. 快速打开三维标定网页

标定网页本身不需要 ROS 或 PLC。为了跳过较长的默认运动仿真，可把仿真目标设为原点：

```bash
python3 main.py \
  --live \
  --target-x 0 \
  --target-y 0 \
  --target-z 0 \
  --host 127.0.0.1 \
  --port 8000
```

等待终端出现类似输出：

```text
Live view: http://127.0.0.1:8000
Press Ctrl+C to stop.
```

然后打开：

- 三维标定页面：`http://127.0.0.1:8000/calibration`
- 起重机控制/仿真页面：`http://127.0.0.1:8000/`

如果 `8000` 端口已被占用，程序会自动选择其他可用端口。此时必须使用终端实际打印的 `Live view` 地址。

标定页面的基本操作：

1. 设置模拟地图的 roll、pitch、yaw、三维原点和测量噪声。
2. 设置大车 Forward Run 与小车 Lateral Run 的实际有符号移动距离。
3. 点击 **SIMULATE RUN** 查看地图倾斜时的 XYZ 变化；也可以直接填写现场记录的三个三维 SLAM 点。
4. 点击 **CALIBRATE / 标定**，检查比例、正交误差、地面倾角和残差 RMS。
5. 点击 **COPY CLI**，将生成的 `--map-to-crane-*` 参数加入 PLC 启动命令。

### 3. 允许局域网其他电脑访问

服务器监听所有网卡：

```bash
python3 main.py \
  --live \
  --target-x 0 --target-y 0 --target-z 0 \
  --host 0.0.0.0 \
  --port 8000
```

在其他电脑浏览器打开：

```text
http://运行程序的电脑IP:8000/calibration
```

需要确保宿主机防火墙允许 TCP 8000 端口。`0.0.0.0` 只用于监听，不能直接作为浏览器访问地址。

### 4. 在现有 20rocker 容器中运行

进入容器并加载 ROS Noetic 环境：

```bash
docker exec -it rocker-20-ws /usr/bin/zsh
source /opt/ros/noetic/setup.zsh 2>/dev/null || source /opt/ros/noetic/setup.bash
cd /home/big/workspace/lstControl
python3 -m pip install --user -r requirements.txt
python3 main.py \
  --live \
  --target-x 0 --target-y 0 --target-z 0 \
  --host 0.0.0.0 \
  --port 8000
```

如果容器使用 host 网络，可从宿主机打开 `http://127.0.0.1:8000/calibration`；如果使用 bridge 网络，需要在创建容器时发布 8000 端口，或使用容器可达 IP。

### 5. 运行普通控制仿真

```bash
python3 main.py \
  --target-x 10 \
  --target-y 5 \
  --target-z 2 \
  --live
```

程序先完成控制仿真、生成曲线图片，再启动 Web 服务。默认目标距离较大时，需要等待一段时间，直到终端打印 `Live view` 才能访问网页。

### 6. 运行真实 PLC + ROS 模式

先确认 ROS 定位话题有数据：

```bash
source /opt/ros/noetic/setup.zsh 2>/dev/null || source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /localization_pose
```

再使用标定页生成的三维参数启动，例如：

```bash
python3 main.py \
  --plc-ip 192.168.0.1 \
  --live \
  --host 0.0.0.0 \
  --port 8000 \
  --map-to-crane-origin-x 10 \
  --map-to-crane-origin-y 20 \
  --map-to-crane-origin-z 1.5 \
  --map-to-crane-roll-deg 2 \
  --map-to-crane-pitch-deg -4 \
  --map-to-crane-yaw-deg 90 \
  --workspace-x-min 0 --workspace-x-max 30 \
  --workspace-y-min -10 --workspace-y-max 10 \
  --workspace-z-min -5 --workspace-z-max 15
```

真实 PLC 模式不会自动开始运动。打开控制页面、确认 PLC 与 Heartbeat 状态正常，输入 SLAM map 坐标目标后点击 **Apply Target**。停止服务使用 `Ctrl+C`；程序退出时会执行安全停止和资源清理。

### 常见问题

- **网页打不开**：先确认终端已经打印 `Live view`，再使用实际打印的端口。
- **局域网无法访问**：确认使用了 `--host 0.0.0.0`，并检查防火墙和容器端口映射。
- **标定 API 报错**：三个观测点都必须包含有限的 Map X/Y/Z，两段移动距离不能为 0，两条轨迹不能接近平行。
- **PLC 模式没有定位**：使用 `rostopic echo -n 1 /localization_pose` 检查 ROS 话题和时间戳是否持续更新。
- **PLC 库无法加载**：真实库只适用于对应 ARM 环境；开发测试必须显式添加 `--allow-mock-plc` 才会使用 MockPLC。
- **大/小车往目标反方向跑**：说明该轴驱动器正方向与定位坐标轴相反（PD 变成正反馈，越跑越远）。这是镜像关系，SLAM `yaw` 标定（旋转）无法表达，需用 `--invert-big-car` / `--invert-small-car`（Z 轴用 `--invert-lift`）翻转对应轴的速度指令符号。若整机 X、Y 同时反且轨道确实与地图反向，也可用 `--map-to-crane-yaw-deg 180`。
- **运动断断续续**：速度伺服需要每个控制周期持续刷新指令。执行器现在按 10Hz 每周期都下发 X/Y 速度与 Z 高度，避免 PLC 看门狗因收不到指令而停轴。若仍断续，检查 `/localization_pose` 是否稳定 10Hz、心跳是否正常。
- **Apply Target 后距离目标还很远 PD 就结束、输出速度为 0**：最常见原因是**目标 Z 与反馈 Z 参考系不一致**——若已启用抓钩高度作为 Z 反馈（默认如此），但标定过 `--map-to-crane-origin-z` 或 roll/pitch，旧逻辑会把网页输入的目标 Z 当作 SLAM map 坐标做完整 3D 旋转/平移，而反馈 Z 已经是不经旋转/平移的抓钩实测高度，二者相差一个常数偏移，PD 会朝着错误的高度收敛，看起来像“离目标还很远就停了”。现已修复：Z 反馈用抓钩高度时，目标 Z、显示 Z、控制反馈 Z 统一为同一参考系（详见「Z 轴特殊处理」）。若仍出现该现象，检查 `/api/start-control` 日志里打印的 `target=(...)` 与 `start=(...)` 是否确实接近真实物理距离。
- **三个轴中某一个先到目标附近就“停”，但整体还没到位**：这是**设计内的正常行为**——三轴 PD 各自独立锁轴，一个轴到位后该轴速度归零并不会影响另外两轴继续运动，直到三轴全部到位控制循环才真正结束（可在轮询状态 `arrivals` 里看到逐轴到达时间）。若观察到“先到位的轴其实离真实目标还有明显距离”，通常还是上一条的 Z 参考系错位问题（Z 轴最容易先到一个错误但数值上很近的“假目标”）；确认已启用抓钩高度反馈且用最新代码后应恢复正常。
- **PD 控制运行中途提前结束**：常见原因及已内置的容错措施：
  - *偶发定位坏帧*（SLAM/网络抖动导致的单帧 NaN、越界、或明显跳变）：不再单帧直接终止整个作业，而是丢弃坏帧继续控制；仅当连续坏帧数超过 `max_consecutive_bad_frames`（默认 10 帧≈1s）才判定为真实故障并安全停车。
  - *瞬时心跳丢失*（PLC 通信短暂抖动）：心跳失败连续达到阈值（默认 10 次≈1s）会短暂标记不健康、暂停当前控制运行，但心跳线程会持续重试并在恢复后自动重新置为健康——无需重启进程即可重新 Apply Target。
  - *执行器瞬时不可用*（`actuator.apply()` 因连接/心跳抖动抛异常）：预算内（`max_consecutive_actuator_errors`，默认 10 次≈1s）跳过本周期继续控制，不再让一次瞬时失败直接终止整段作业；超出预算才视为真实故障并安全停车。
  - *定位断流 ≥2s*、*控制超时（10 分钟）*、*PLC 断线*：这些仍视为真实故障，按设计中止并安全停车，检查 ROS 话题、网络与 PLC 连接。
- **Z 轴数值抽动：偶尔跳到 0，然后又跳回正确高度**：`GetActualLiftHeight()` 在底层 S7 连接短暂抖动期间会返回 0，或形如 `4.49863e-312` 的 denormalized 垂圾值，而不是保持上一次真实读数——这是闭源库 `libsscarctrl.so` 自身的行为，无法从 Python 侧修改库本身。现已在 `RealPLC.get_lift_height()` 内新增 `_LiftHeightSanitizer` 过滤：识别“近零骤降”“非数值 (NaN/Inf)”“单周期内物理上不可能的跳变”三类异常读数，异常时沿用上一次可信高度，仅当异常持续超过 2s 才放弃缓存值（返回 `None`，交给上层回退 SLAM Z 或判定为反馈不可用）。UI 显示、目标预填、PD 反馈都从同一个 `get_lift_height()` 读取，因此这一处过滤对全链路生效。
- **终端打印 `SendPlcHeartbeat: N` / `CheckPlcConnection: Heartbeat timeout!...` / `CheckPlcConnection: Heartbeat difference too large!...` / `Actual Lift Height: ...`**：这些**不是我们 Python 代码打印的**，而是闭源库 `libsscarctrl.so` 内部在 `SendPlcHeartbeat()` / `CheckPlcConnection()` / `GetActualLiftHeight()` 被调用时自带的调试输出（`strings libsscarctrl.so` 可确认）。含义：
  - `SendPlcHeartbeat: N`：库内部心跳发送计数器，每次调用自增，正常现象，不代表异常。
  - `CheckPlcConnection: Heartbeat timeout! No update for N consecutive checks.` / `Heartbeat difference too large! Diff=... (max allowed=10)`：库内部维护的心跳**回读/校验**长时间没有被确认更新，或与参考值差异过大——**这确实反映了一次真实的 S7 通信异常/抖动**，不是无意义的噪声；但它是否致命取决于持续时长：
    - 若只是偶发几次（对应我们侧的心跳自愈 + `max_consecutive_actuator_errors` 容忍窗口，约 1s），控制会自动跳过继续，不需要人工干预。
    - 若像日志里那样连续 700+ 次没有更新（10Hz 下约 70s 以上），说明通信链路已经处于持续不稳定状态（网络、交换机、PLC 侧 CPU 负载或 S7 连接数超限都可能是原因），仅靠软件容忍无法根治，需要检查物理网络/PLC 端。
  - `Actual Lift Height: X`：`GetActualLiftHeight()` 被调用时库自带的打印，紧跟在 `Heartbeat timeout/difference too large` 之后出现 `X=0` 或 denormalized 垂圾值并非偶然——正是本节上一条要修复的现象，现已在 Python 侧过滤。
  - 这些打印刷屏本身不会影响功能，但若频繁出现建议排查现场网络稳定性；无法通过修改闭源库消除这些打印。

---

## 控制律

```text
v_cmd = Kp * (target_position - measured_position) - Kd * damping_velocity
```

- **D 项速度反馈优先用原生速度**：`damping_velocity` 优先取位置源提供的原生速度（如 Odometry `twist`），只有原生速度缺失时才退回“位置差分 + 低通滤波”。对 10Hz 量化定位做差分噪声很大，直接进入 D 项会让速度指令在目标附近来回抖动、频繁换向（现场表现为大小行车到点前反复抽动）。
- **到位死区**：位置误差进入 `arrival_pos_tol` 窗口后速度指令直接归零，消除锁定前的微幅蠕动和换向脉冲。
- **防反向抽动**：在尚未越过目标（`|误差| < reverse_guard_tol`）时，禁止给出与位置误差方向相反的速度指令。速度伺服型行车“刹车”应是指令归零而非反向脉冲；真正越过目标后的回拉始终允许。

两种模式使用完全相同的位置控制器和速度滤波器。地图未倾斜时，PLC 状态显示与到位检测可使用 Odometry 原生 XY 速度；存在 roll/pitch 且 Map Vz 不可信时，XYZ 统一从变换后的位置差分并低通滤波得到速度。

---

## 运行模式

通过 `--plc-ip` 自动切换：

| 命令 | 模式 | 位置来源 | 执行 | 交互 |
|------|------|----------|------|------|
| `python3 main.py` | 实验室仿真 | 仿真对象模型 (100Hz) | 仿真对象 | 命令行 target → 自动运行 → 回放 |
| `python3 main.py --plc-ip 192.168.0.1 --live` | PLC 控制 | ROS `/localization_pose` (10Hz) | `ctypes` → Snap7 → 西门子 S7 PLC | 开 UI → 实时定位 → Apply Target → 实时控制 |

### 实验室仿真（默认）

```bash
python3 main.py --target-x 10 --target-y 5 --target-z 2 --live
```

不指定 `--plc-ip`，位置反馈来自仿真对象（含伺服滞后、扰动、测量噪声）。`--live` 时浏览器回放已完成的仿真。

### 真实 PLC 控制

```bash
python3 main.py --plc-ip 192.168.0.1 --live
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

- **反馈链路走 ROS 1**：`ros_bridge.py` 订阅 `/localization_pose`（`nav_msgs/Odometry`），提供 X/Y 位置；XY 可使用原生速度。**Z 位置默认取 PLC 抓钩实测高度 `GetActualLiftHeight()`**（物理 Z，编码器直读，比 SLAM Map Z 更可靠），Z 速度由高度差分低通估算。抓钩高度不可用（如 MockPLC）时回退到 SLAM Z。
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
| `--map-to-crane-origin-x` | `0.0` | 起重机原点在 SLAM 地图中的 X 坐标 (m) |
| `--map-to-crane-origin-y` | `0.0` | 起重机原点在 SLAM 地图中的 Y 坐标 (m) |
| `--map-to-crane-origin-z` | `0.0` | 起重机原点在 SLAM 地图中的 Z 坐标 (m) |
| `--map-to-crane-roll-deg` | `0.0` | 真实起重机坐标系在 SLAM 地图中的横滚角 (deg) |
| `--map-to-crane-pitch-deg` | `0.0` | 真实起重机坐标系在 SLAM 地图中的俯仰角 (deg) |
| `--map-to-crane-yaw-deg` | `0.0` | 起重机 +X 轨道在 SLAM 地图中的偏转角 (deg) |
| `--use-native-z-velocity` | 关闭 | 显式信任 Odometry 的 `twist.linear.z`；默认从高度估速 |
| `--invert-big-car` | 关闭 | 大车(X)驱动正方向与定位 X 轴相反时翻转速度指令符号 |
| `--invert-small-car` | 关闭 | 小车(Y)驱动正方向与定位 Y 轴相反时翻转速度指令符号 |
| `--invert-lift` | 关闭 | 吊钩(Z)驱动正方向与定位 Z 轴相反时翻转速度指令符号 |
| `--min-lift-height` | `0.5` | 下发给 `liftctrl` 的最小高度 (m)，保证抓钩离地不低于该值 |
| `--workspace-x-min/max` | 空 | 可选 X 轴机械工作区；必须成对设置 |
| `--workspace-y-min/max` | 空 | 可选 Y 轴机械工作区；必须成对设置 |
| `--workspace-z-min/max` | 空 | 可选 Z 轴机械工作区；必须成对设置 |

真实设备部署应按机械行程显式设置三轴工作区，例如：

```bash
python3 main.py --plc-ip 192.168.0.1 --live \
  --workspace-x-min 0 --workspace-x-max 30 \
  --workspace-y-min -10 --workspace-y-max 10 \
  --workspace-z-min 0 --workspace-z-max 15
```

所有目标都会拒绝 `NaN`/无穷值。X/Y/Z 坐标允许为负数，因为坐标零点由现场定义；配置工作区后才会拒绝越界目标。真实设备必须按机械行程显式配置 Z 边界，不能依赖坐标正负判断安全性。

---

## SLAM 地图与起重机轨道标定

SLAM 地图不仅可能与大车/小车轨道有水平偏角，也可能没有与真实地面完全平齐。程序使用完整三维刚体变换：

```text
map_point   = map_origin + Rz(yaw) Ry(pitch) Rx(roll) * crane_point
crane_point = R^T * (map_point - map_origin)
```

- 浏览器定位、目标输入和轨迹显示保持 **SLAM map 坐标**。
- PD 控制、机械工作区检查和 PLC 三轴指令使用 **真实起重机坐标**。
- XYZ 位置会旋转并平移，XYZ 速度只旋转；地图坡度造成的 Z 漂移不会再串入真实 Z。
- 原点、roll、pitch、yaw 全为 0 时是单位变换，兼容未标定的旧用法。

### 现场三点标定

1. 将大车/小车停到约定的起重机坐标原点，记录三维 SLAM 点 `START (X,Y,Z)`。
2. 仅沿起重机 **+X 大车轨道**移动已知的有符号距离，记录三维点 `AFTER X`。
3. 从 `AFTER X` 仅沿起重机 **+Y 小车轨道**移动已知的有符号距离，记录三维点 `AFTER Y`。
4. 打开 `http://127.0.0.1:8000/calibration`，输入三个三维观测点和两段实际移动距离，点击 **CALIBRATE / 标定**。
5. 检查 X/Y 比例、正交误差、地面倾角和残差 RMS；质量合格后复制页面生成的 CLI 参数。

两条不共线的三维水平轨迹已经确定了物理 +X/+Y 方向，其叉乘会得到真实地面法向 +Z，因此不需要为了该标定额外升降吊钩。若 SLAM 地图倾斜，大车或小车水平移动时原始 Map Z 会变化；变换到起重机坐标后，这三点的真实 Z 应接近同一个值。

例如地图中的起重机原点为 `(10, 20, 1.5)`，地图相对真实地面 roll 为 2°、pitch 为 -4°，大车 +X 在地图里指向 90°：

```bash
python3 main.py --plc-ip 192.168.0.1 --live \
  --map-to-crane-origin-x 10 \
  --map-to-crane-origin-y 20 \
  --map-to-crane-origin-z 1.5 \
  --map-to-crane-roll-deg 2 \
  --map-to-crane-pitch-deg -4 \
  --map-to-crane-yaw-deg 90 \
  --workspace-x-min 0 --workspace-x-max 30 \
  --workspace-y-min -10 --workspace-y-max 10 \
  --workspace-z-min -5 --workspace-z-max 15
```

标定页可先设置模拟的 roll、pitch、yaw、三维原点、移动距离和噪声，播放“大车前进 → 小车横移”过程。左上图显示原始 SLAM 平面轨迹，右上图显示变换后与轨道对齐的坐标，底部曲线显示水平移动产生的 Map Z 漂移以及标定后的真实地面 `Z≈0`。观测点可手动编辑，因此也能直接代入现场记录做离线计算。

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
| **Localization** | 空 | ROS `/localization_pose` 实时数据（Z 显示抓钩高度） |
| **Control** | 空 | PLC 连接状态、心跳、轴速度/高度、STOP ALL / Reset |

PLC 模式下 Apply Target 启动 PD 后，画布会记录起点→目标的运行轨迹；**控制完成或停止后轨迹会冻结保留**，便于展示，直到下一次 Apply Target 才清空重录。

控制台右上角的 **Coordinate Calibration** 可随时进入标定仿真页；该页面调用本机 `/api/calibrate` 计算标定结果，不依赖 ROS 或 PLC 在线。

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
    │+ CoordinateTransform│ │(ctypes/Snap7)│
    │ (ROS 反馈, 2s超时) │  │             │
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
                       支持仿真回放、PLC 实时控制和坐标标定页
coordinate_transform.py SLAM map ↔ 起重机坐标三维刚体变换
calibration.py         根据两段三维轨道移动估计原点、roll/pitch/yaw 和质量指标
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
    │                                     ← PLC:  pos←定位, vel←原生XY/差分Z
    ├─ 到达检测 (位置+速度判定)
    └─ hooks.on_step(step_data)        ← PLC: 入队→SSE→浏览器
```

### Z 轴特殊处理

PLC 的 `liftctrl` 接收**绝对高度** (m)，而非速度。与 X/Y 速度伺服不同，Z 由 PLC 内部位置环跟随下发的高度设定值。

**反馈**：`RosPositionSource` 通过 `GetActualLiftHeight()` 提供抓钩实测高度作为 `z_measured`；UI 的 `/localization/stream` 在 PLC 模式下同样用抓钩高度覆盖 SLAM Map Z，保证显示、预填目标与 PD 反馈同一参考系。

**读数过滤**：`libsscarctrl.so` 在 S7 连接抖动期间会让 `GetActualLiftHeight()` 返回 0 或 denormalized 垂圾值（而不是保持上次真实值）。`RealPLC.get_lift_height()` 内置 `_LiftHeightSanitizer`，对每次原始读数做三项检查——非数值 (NaN/Inf)、相对上次读数的“近零骤降”、单周期内物理上不可能的跳变——命中任意一项即丢弃该帧、沿用上一次可信高度；仅当异常连续超过 2s（`_LiftHeightSanitizer.STALE_TIMEOUT`）才放弃缓存值返回 `None`。这是唯一的读数入口，UI 展示、目标预填、`RosPositionSource`、`PD` 反馈全部共用，因此过滤对全链路生效，不会出现"跳到 0 再跳回正确高度"的抽动。

**目标解析同样要跳过地图旋转/平移**：抓钩高度是与 SLAM 地图完全独立的物理量（地面 = 0），不该套用 map↔crane 的 3D 变换。`_handle_start_control`（网页 Apply Target）与 `main.py` 的命令行 PLC 分支在检测到抓钩高度可用时，会用 `CoordinateTransform2D.map_to_crane_target(..., z_is_hoist_height=True)` 解析目标——X/Y 仍走标定变换，Z 直接取网页/CLI 输入的抓钩高度，不经过 `origin_map_z` 平移或 roll/pitch 旋转；展示回网页同理使用 `crane_to_map_display(...)` 反向跳过 Z。控制过程中实时轮询显示的 Z（`LiveControlHooks`/`control_step_to_map`）也用同一开关保持不旋转。**若目标 Z 仍按旧逻辑走完整 3D 变换，一旦标定了 `origin_map_z` 或 roll/pitch，目标与反馈就会出现常数级偏差，PD 会朝错误高度收敛、在离真实目标很远时就提前结束。**

**控制**：PD 照常计算 `vz_cmd`（含限速与 D 项阻尼）。`PlcActuator` 将 `vz_cmd` 积分为绝对高度设定值，并沿设目标时的方向**单调逼近目标、不越过**，每周期调用 `liftctrl`：

```text
z_setpoint += vz_cmd * dt
z_setpoint = clamp_toward_target(z_setpoint, z_target)   # 单调逼近，不越过
z_setpoint = max(min_lift_height, z_setpoint)              # 抓钩离地安全下限
plc.lift_ctrl(z_setpoint)
```

**注意**：设定值 (`z_setpoint`) 与实测高度 (`z_measured`) 分离——实测只用于 PD 反馈与到位判定，**不再每周期把设定值重锚到实测**。旧逻辑会导致下发高度永远只领先一步 `vz*dt`，真实吊钩伺服死区下 Z 几乎不动。

**抓钩离地安全下限**：下发给 `liftctrl` 的绝对高度会被硬钳到 `>= --min-lift-height`（默认 0.5m）。坐标校验层仍允许负坐标（零点由现场定义），执行器下限是物理硬保护。

> 若把目标 Z 设成低于该下限，抓钩会停在下限处、Z 轴无法判定到位而最终超时。要在输入层直接拒绝过低目标，可再配 `--workspace-z-min 0.5`。

现场没有 Z 原生速度时，`RosPositionSource` 将 `vz` 标记为不可用，控制循环按每帧高度变化计算：

```text
vz_raw = (z_now - z_previous) / dt
vz_filtered = low_pass(vz_raw, tau=0.50s)
```

该方案在 10Hz 高度数据连续、时间戳可靠、量化噪声不过大的条件下可行。低通滤波会引入少量延迟，因此到达判断同时检查位置、速度估计和速度指令，避免“高度刚进入窗口但吊钩仍在运动”时过早判定到位。地图存在 roll/pitch 且原生 Map Vz 不可信时，XYZ 三轴统一从变换后的连续位置估速，避免缺少 Vz 导致速度旋转错误；只有确认设备发布的三维 twist 真实可靠时才建议传入 `--use-native-z-velocity`。

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
| `SendPlcHeartbeat(ip)` | IP | 心跳，10Hz，连续失败达阈值（默认 10 次≈1s）标记不健康；心跳线程持续重试，恢复后自动重新置为健康（不会永久失效） |
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
| `/localization_pose` | `nav_msgs/Odometry` | 实时定位；提供 X/Y 位置（XY 速度可用）。Z 位置默认改用 PLC 抓钩高度，SLAM Z 仅作回退 |

---

## 参数调节

主要参数在 `CraneConfig` 中（`main.py` 的 `_config_from_args()` 直接采用这些默认值）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kp_pos` | **0.4** | 位置比例增益；降低可减少接近目标时的急刹与过冲 |
| `kd_pos` | **0.55** | 速度阻尼增益；提高可抑制停止阶段的回摆/换向 |
| `max_velocity_xy` | 0.2 | X/Y 最大速度 (m/s) |
| `max_velocity_z` | 0.2 | Z 最大速度 (m/s) |
| `velocity_filter_tau` | 0.25 | 仿真模式速度滤波时间常数 (s) |
| `velocity_filter_tau_plc` | **0.80** | PLC 模式速度滤波 (s)；加大可平滑 D 项、减少 10Hz 定位噪声引起的换向 |
| `servo_time_constant_xy` | 0.18 | X/Y 速度环一阶响应 (s)（仅仿真） |
| `servo_time_constant_z` | 0.12 | Z 速度环一阶响应 (s)（仅仿真） |
| `arrival_pos_tol` | **0.025** | 到达判定位置容差 (m)，**同时用作 PD 速度指令死区**——进入该窗口后 `v_cmd=0` |
| `arrival_vel_tol` | 0.005 | 到达判定速度容差 (m/s) |
| `reverse_guard_tol` | **0.10** | 防反向抽动保护带 (m)；\|误差\| 小于此值时禁止给出远离目标的速度指令 |
| `arrival_capture_pos_tol` | **0.04** | 到位捕获位置窗口 (m)；扩大可更早进入软着陆 |
| `arrival_cmd_tol` | **0.025** | 到位捕获速度指令窗口 (m/s) |
| `arrival_debounce_cycles` | 3 | 到位判定去抖：需连续 N 个周期都满足到位条件才锁轴，防止单帧定位跳变误锁 |
| `localization_jump_margin` | 0.30 | 定位跳变门限余量 (m)：单周期位置变化超过 `max_v*dt + 该余量` 视为异常帧 |
| `max_consecutive_bad_frames` | 10 | 单帧定位异常 (NaN/越界/跳变) 的连续容忍帧数；超出则视为真实故障并中止 |
| `max_consecutive_actuator_errors` | 10 | `actuator.apply()` 连续失败 (PLC 连接/心跳抖动) 的容忍次数；超出则视为真实故障并中止 |
| `workspace_x/y/z_bounds` | `None` | 可选机械工作区；未配置时只校验坐标有限性，不限制正负 |

### 停止阶段抓钩抖动（大车 X 轴速度频繁换向）

**现象**：大车接近目标停止时，`vx_cmd` 在 ±0.02~0.05 m/s 之间来回切换，桥架 jerk 通过钢丝绳传到抓钩，表现为抓钩摆动。

**机理**：10Hz SLAM 定位噪声使位置误差在零附近变号 → PD 输出正负速度交替；每次速度指令变化都会驱动 PLC 伺服，机械冲击引起抓钩摆。

**当前默认已针对该问题调优**（见上表加粗项）。若仍抖动，按优先级继续调整：

1. `reverse_guard_tol` ↑ (0.10 → 0.12~0.15) — 禁止目标附近反向脉冲
2. `arrival_pos_tol` ↑ (0.025 → 0.03~0.04) — 更早让速度指令归零
3. `velocity_filter_tau_plc` ↑ (0.80 → 1.0) — 进一步平滑 D 项
4. `kp_pos` ↓ (0.4 → 0.3) 或 `kd_pos` ↑ (0.55 → 0.65) — 软着陆 vs 阻尼

**诊断**：观察终端 `[PD] v_cmd=(x=...)` 或 UI 轮询的 `vx_cmd`；理想情况是离目标 5~10 cm 时单调减至 0，不再正负交替。

修改方式：编辑 `crane_model.py` 中 `CraneConfig` 的默认值，重启 PLC 模式进程后生效。

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
