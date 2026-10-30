# PTP 防摇方案（Phase 1）

## 代码布局

所有可插拔防摇算法在 **`anti_sway/`** 目录：

- `incl.py` / `gantry.py` — 各一种策略（实现 `correct_xy`）
- `registry.py` — 注册表；新算法在此登记
- `ptp_config.py` — PTP 模式与 `site_config.json` 字段合并
- `estimator.py` / `sim_feedback.py` / `settle.py` — θ 估计与仿真反馈

详见 [`anti_sway/README.md`](../anti_sway/README.md)。

## PTP 控制选项

| 选项 | `anti_sway_mode` | 控制律 | 参考 |
|------|------------------|--------|------|
| **纯 PD** | `off` | 仅位置 PD | — |
| **防摇 K·θ** | `incl` | `v = v_PD + K(L)·θ`（默认接近 0.4 m 内） | `CraneRemote-master` / dev_incl |
| **防摇 Gantry PD** | `gantry` | `v = v_PD − (Kp·θ + Kd·θ̇)`，全程 | `gantry-crane-anti-sway-control-main` |
| **防摇 Overhead LQR** | `overhead` | `v = v_PD − (1/M)·(Kθ·θ+Kω·θ̇)`，K 由绳长 L 在线 LQR | `OverheadCraneSwayControl-main` |
| **防摇 输入整形 ZVD** | `input_shape` | 对 `v_PD` 做 ZVD 三脉冲卷积，`ωₙ=√(g/L)`，**开环、不读倾角仪** | 经典 input shaping |

仿真 URL：`?anti_sway_mode=incl|gantry|overhead|input_shape`（旧 `anti_sway=1` 仍映射为 `incl`）。  
真机 POST：`{"anti_sway_mode":"gantry", "target_x":...}`。

## 参考与适配

| 来源 | 思路 | 本项目用法 |
|------|------|------------|
| `防摇方案/CraneRemote-master` | 视觉测钩偏 → P 控制修正大车/小车 | 用倾角仪 θ 代替视觉偏移 |
| `dev_incl` | Δv = K·θ，接近段/终端阻尼 | **incl** 模式，默认 approach 段 |
| `gantry-crane-anti-sway-control-main` | xPID ∥ thetaPID，`Output_x − Output_theta` | **gantry** 模式，速度域并联 PD |
| `OverheadCraneSwayControl-main` | 单摆 4 状态 LQR，`Q` 重罚 θ/θ̇ | **overhead** 模式，绳长随 Z 重算 K |
| `simulation.m`（CraneRemote） | 摆锤 + 过零步进 | 仿真闭环用 `SimSwayFeedback` |

### incl（K·θ）

```
v_cmd = v_PD + Δv
Δv_x = clip(K(L)·θ_x, ±max_correction)
```

### gantry（并联角度 PD）

```
u_x = clip(Kp·θ_x + Kd·θ̇_x, ±max_correction)
v_x,cmd = v_x,PD − u_x   （小车 Y 轴同理）
```

参考工程实验增益：`Kp_θ=0.9`, `Kd_θ=0.45`（θ 为 rad），死区 1°。对应字段见 `site_config.json` 中 `anti_sway_gantry_*`。

### overhead（单摆 LQR）

参考 `Crane_Sway_Control.m` 线性化：

```
ẋ = A x + B u,   u = −K x   （力 [N]）
v_x,cmd = v_x,PD − (1/M)·clip(Kθ·θ_x + Kω·θ̇_x, ±max)
```

默认 `m=5 kg`, `M=250 kg`, `Q=diag(1,1,1e7,1e7)`, `R=1`；有效绳长 `L` 来自 `rope_length_*` 与当前 Z。字段：`anti_sway_overhead_*`。

## 界面 A/B/C 测试

1. **仿真** `/ptp/sim`：选三种单选之一 →「移动到目标」→ 看左侧 Hook Sway 与轨迹。
2. **真机** `/ptp/live`：同样选择 → θ 来自 `/inclination/j1939_msg`。

同一目标下对比 **θ 峰值、到位后衰减、总时间**。

## 仿真 vs 真机 θ 来源

| 模式 | 控制闭环 θ | 画面摆角显示 |
|------|------------|--------------|
| Sim + 防摇 | `SimSwayFeedback` | `sim_inclinometer` |
| Live + 防摇 | `SwaySensorSource` | `/api/sway-snapshot` |

## 未纳入 Phase 1 的 dev_incl 能力

终端 settle、growth guard 等字段仍在 `site_config.json`，PTP goto 尚未接 `anti_sway_runtime`。

## 调参

- **incl**：`anti_sway_sway_gain` 1.5–2.6；`anti_sway_approach_pos` 略大于到位容差。
- **gantry**：`anti_sway_gantry_kp/kd`；过大易与 PD 打架，先小范围位移试车。
- **overhead**：`anti_sway_overhead_trolley_mass` / `load_mass` 尽量贴近实物；`q_theta/q_omega` 与参考一致时可先不动；幅值由 `max_correction` 限幅。
