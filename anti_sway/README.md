# anti_sway — 防摇算法包

## 目录

| 文件 | 作用 |
|------|------|
| `base.py` | `AntiSwayStepContext`、`VelocityCorrection`、策略协议 |
| `registry.py` | 算法注册表、`build_strategy()` |
| `ptp_config.py` | PTP 模式 `off/incl/gantry` 与 site JSON 合并 |
| `incl.py` | K·θ 接近段阻尼 |
| `input_shape.py` | PTP 开环 ZVD 输入整形（不读倾角仪） |
| `shaping.py` | ZVD 三脉冲离散卷积 |
| `baseline.py` | 消摆基准 K·θ |
| `crane_remote.py` | CraneRemote 消摆 (K·θ + 过零步进) |
| `gantry.py` | 并联角度 PD |
| `gantry_settle.py` | Gantry 消摆 (全程 θ PD) |
| `overhead.py` | 单摆 LQR (OverheadCraneSwayControl) |
| `overhead_settle.py` | Overhead LQR 消摆 |
| `lqr_math.py` | 连续 LQR / CARE |
| `estimator.py` | 倾角仪 → θx/θy |
| `sim_feedback.py` | 仿真闭环摆角 |
| `settle.py` / `rope.py` | 绳长几何与整定公式 |

项目根目录下的 `sway_*.py` / `sim_sway_feedback.py` 仅为兼容 re-export，新代码请 `import anti_sway`。

目录总览见 [`doc/PROJECT_LAYOUT.md`](../doc/PROJECT_LAYOUT.md)。

## 新增一种算法

1. 新建 `anti_sway/my_algo.py`：

```python
MODE_ID = 'my_algo'
MODE_LABEL = '显示名'
MODE_DESCRIPTION = '一句话说明'

def config_fields_from_site(site): ...
def build_strategy(config) -> MyStrategy | None: ...
```

2. 在 `MyStrategy` 中实现 `mode_id`、`log_startup()`、`correct_xy(vx, vy, ctx) -> VelocityCorrection`。

3. 在 `registry.py` 的 `_ALGORITHM_MODULES` 追加 `my_algo` 模块。

4. 在 `site_config.json` 的 `SITE_CONFIG_FIELDS`（`site_config.py`）加入该算法参数字段。

5. PTP 页使用下拉框 `Anti-sway / 防摇算法`（选项由 `list_algorithm_options()` 生成）；也可 GET `/api/anti-sway-algorithms`。

`run_pd_control` 只调用 `build_strategy` + `correct_xy`，无需再改 `crane_model` 分支。
