"""判稳阶段倾角指标 — 与固定零偏解耦的摆动幅值估计。

现场问题:
  - 倾角仪静态零偏随抓钩高度、张合状态变化 (见 site_config / 倾角仪初值)
  - 固定 bias 扣减后仍剩慢变分量; 用 |θ| 判稳会误拦或误放
  - 判稳窗 0.6s 常短于单摆周期 T≈2π√(L/g), 短窗峰峰值会漏检

本模块提供:
  - 绳长 → 摆周期 → 自适应判稳窗长
  - 线性去趋势后的峰峰值 (抑制窗内 bias 斜坡/阶跃的一部分)
  - 判稳等待用 θ_dc 慢跟踪, 阻尼只用 θ_ac = θ − θ_dc
"""

from __future__ import annotations

import math
from dataclasses import dataclass

GRAVITY = 9.81


def effective_rope_length(
    z_grab: float,
    *,
    sheave_height: float = 2.31,
    grab_offset: float = 1.5,
    cable_stretch: float = 0.0,
    min_length: float = 0.5,
) -> float:
    """有效摆长 L_eff [m] (与 sway_controller.RopeLengthModel 一致)。"""
    L = sheave_height - z_grab + grab_offset + cable_stretch
    return max(L, min_length)


def pendulum_period(
    z_grab: float,
    *,
    sheave_height: float = 2.31,
    grab_offset: float = 1.5,
    cable_stretch: float = 0.0,
    min_length: float = 0.5,
    g: float = GRAVITY,
) -> float:
    """单摆周期 T [s]。"""
    L = effective_rope_length(
        z_grab,
        sheave_height=sheave_height,
        grab_offset=grab_offset,
        cable_stretch=cable_stretch,
        min_length=min_length,
    )
    return 2.0 * math.pi * math.sqrt(L / g)


def adaptive_settle_window(
    z_grab: float,
    *,
    base_window: float,
    window_periods: float,
    sheave_height: float = 2.31,
    grab_offset: float = 1.5,
    cable_stretch: float = 0.0,
    min_length: float = 0.5,
    max_window: float = 8.0,
) -> float:
    """判稳滑动窗长 [s] = max(base, window_periods × T), 上限 max_window。"""
    T = pendulum_period(
        z_grab,
        sheave_height=sheave_height,
        grab_offset=grab_offset,
        cable_stretch=cable_stretch,
        min_length=min_length,
    )
    return min(max_window, max(base_window, window_periods * T))


def _linear_detrend_peak(values: list[float], times: list[float]) -> float:
    """对 (times, values) 做最小二乘线性去趋势, 返回残差峰峰值 [与 values 同单位]。"""
    n = len(values)
    if n < 2:
        return 0.0
    t0 = times[0]
    t_norm = [t - t0 for t in times]
    mean_t = sum(t_norm) / n
    mean_v = sum(values) / n
    num = sum((t_norm[i] - mean_t) * (values[i] - mean_v) for i in range(n))
    den = sum((t_norm[i] - mean_t) ** 2 for i in range(n))
    if den < 1e-18:
        ac = [v - mean_v for v in values]
    else:
        slope = num / den
        intercept = mean_v - slope * mean_t
        ac = [values[i] - (slope * t_norm[i] + intercept) for i in range(n)]
    return max(ac) - min(ac)


def sway_linear_detrend_peak_to_peak(
    sway_samples: list[tuple[float, float, float]],
) -> tuple[float, float] | None:
    """窗口内 (t, θx, θy) 线性去趋势后的峰峰值 [rad]。

    常数零偏不改变 max−min; 线性去趋势可削弱窗内 bias 斜坡、慢漂,
    以及夹取瞬间台阶在短窗内造成的"假摆幅"的一部分。
    """
    if len(sway_samples) < 3:
        return None
    times = [s[0] for s in sway_samples]
    xs = [s[1] for s in sway_samples]
    ys = [s[2] for s in sway_samples]
    return (_linear_detrend_peak(xs, times), _linear_detrend_peak(ys, times))


def samples_time_span(samples: list[tuple[float, ...]]) -> float:
    """样本时间跨度 [s]。"""
    if len(samples) < 2:
        return 0.0
    return float(samples[-1][0] - samples[0][0])


def window_samples_full(
    samples: list[tuple[float, ...]],
    window_s: float,
    *,
    min_samples: int = 3,
    fill_ratio: float = 0.85,
    poll_interval: float = 0.02,
) -> bool:
    """滑动窗是否已填满 (按样本跨度; 容差适配离散轮询)。"""
    if len(samples) < min_samples:
        return False
    span = samples_time_span(samples)
    # 轮询离散 + trim 留窗, 实际跨度常略小于 window_s
    return span + poll_interval >= window_s * fill_ratio


def sway_raw_peak_to_peak(
    sway_samples: list[tuple[float, float, float]],
) -> tuple[float, float] | None:
    """窗口内 raw θ 峰峰值 [rad] (不去趋势)。"""
    if len(sway_samples) < 2:
        return None
    xs = [s[1] for s in sway_samples]
    ys = [s[2] for s in sway_samples]
    return (max(xs) - min(xs), max(ys) - min(ys))


@dataclass
class ThetaDcTracker:
    """判稳等待期间的慢变零偏跟踪 → θ_ac = θ − θ_dc, 供阻尼使用。"""

    tau_s: float = 4.0
    dc_x: float | None = None
    dc_y: float | None = None

    def reset(self) -> None:
        self.dc_x = None
        self.dc_y = None

    def update(self, theta_x: float, theta_y: float, dt: float) -> tuple[float, float]:
        if self.dc_x is None:
            self.dc_x = theta_x
            self.dc_y = theta_y
            return 0.0, 0.0
        dt = max(dt, 1e-4)
        alpha = math.exp(-dt / max(self.tau_s, 0.1))
        self.dc_x = alpha * self.dc_x + (1.0 - alpha) * theta_x
        self.dc_y = alpha * self.dc_y + (1.0 - alpha) * theta_y
        return theta_x - self.dc_x, theta_y - self.dc_y


def estimate_period_zero_cross(
    sway_samples: list[tuple[float, float, float]],
    *,
    min_span_s: float = 2.0,
) -> float | None:
    """由 θx 去趋势后过零次数估计摆周期 T [s] (轻量, 适合实时)。"""
    if len(sway_samples) < 32:
        return None
    span = sway_samples[-1][0] - sway_samples[0][0]
    if span < min_span_s:
        return None
    times = [s[0] for s in sway_samples]
    xs = [s[1] for s in sway_samples]
    pp = _linear_detrend_peak(xs, times)
    if pp < 1e-5:
        return None
    mean_t = sum(times) / len(times)
    mean_v = sum(xs) / len(xs)
    t0 = times[0]
    t_norm = [t - t0 for t in times]
    mean_tn = sum(t_norm) / len(t_norm)
    num = sum((t_norm[i] - mean_tn) * (xs[i] - mean_v) for i in range(len(xs)))
    den = sum((t_norm[i] - mean_tn) ** 2 for i in range(len(t_norm)))
    if den < 1e-18:
        ac = [v - mean_v for v in xs]
    else:
        slope = num / den
        intercept = mean_v - slope * mean_tn
        ac = [xs[i] - (slope * t_norm[i] + intercept) for i in range(len(xs))]
    crossings: list[float] = []
    for i in range(1, len(ac)):
        if ac[i - 1] == 0.0:
            crossings.append(times[i - 1])
        elif ac[i - 1] * ac[i] < 0.0:
            a0, a1 = ac[i - 1], ac[i]
            t_a, t_b = times[i - 1], times[i]
            frac = abs(a0) / (abs(a0) + abs(a1)) if (abs(a0) + abs(a1)) > 1e-18 else 0.5
            crossings.append(t_a + frac * (t_b - t_a))
    if len(crossings) < 3:
        return None
    half_periods = [(crossings[i] - crossings[i - 1]) for i in range(1, len(crossings))]
    half_periods.sort()
    hp = half_periods[len(half_periods) // 2]
    if hp <= 0.05:
        return None
    return 2.0 * hp


@dataclass
class SwayLiveAnalyzer:
    """实时摆角分析 — 与调度器判稳/离线 bag 分析使用同一套指标。"""

    max_buffer_s: float = 12.0
    base_window: float = 0.6
    window_periods: float = 0.55
    window_max: float = 8.0
    sheave_height: float = 2.31
    grab_offset: float = 1.5
    tol_deg: float = 0.3
    _samples: list[tuple[float, float, float]] | None = None

    def __post_init__(self) -> None:
        self._samples = []

    def reset(self) -> None:
        self._samples.clear()

    def push(self, t: float, theta_x: float, theta_y: float) -> None:
        self._samples.append((t, theta_x, theta_y))
        while len(self._samples) > 1 and t - self._samples[0][0] > self.max_buffer_s:
            self._samples.pop(0)

    def _window_for_z(self, z_grab: float) -> float:
        return adaptive_settle_window(
            z_grab,
            base_window=self.base_window,
            window_periods=self.window_periods,
            sheave_height=self.sheave_height,
            grab_offset=self.grab_offset,
            max_window=self.window_max,
        )

    def metrics(self, z_grab: float) -> dict[str, float | bool | None]:
        """返回当前自适应窗内的判稳指标 (度制便于 UI)。"""
        if not self._samples:
            return {
                'window_s': self._window_for_z(z_grab),
                'window_full': False,
                'detrend_pp_x_deg': None,
                'detrend_pp_y_deg': None,
                'detrend_pp_max_deg': None,
                'raw_pp_x_deg': None,
                'raw_pp_y_deg': None,
                'raw_pp_max_deg': None,
                'period_s': None,
                'period_model_s': pendulum_period(
                    z_grab, sheave_height=self.sheave_height, grab_offset=self.grab_offset,
                ),
                'settled': None,
                'sample_count': 0,
            }
        now = self._samples[-1][0]
        window = self._window_for_z(z_grab)
        window_samples = [s for s in self._samples if now - s[0] <= window]
        window_full = (
            len(window_samples) >= 3
            and (window_samples[-1][0] - window_samples[0][0]) >= window * 0.95
        )
        det = sway_linear_detrend_peak_to_peak(window_samples) if len(window_samples) >= 3 else None
        raw = sway_raw_peak_to_peak(window_samples) if len(window_samples) >= 2 else None
        period_fft = estimate_period_zero_cross(self._samples)
        period_model = pendulum_period(
            z_grab, sheave_height=self.sheave_height, grab_offset=self.grab_offset,
        )
        det_max = max(det) if det else None
        settled = (
            window_full and det_max is not None and math.degrees(det_max) < self.tol_deg
        )
        return {
            'window_s': round(window, 3),
            'window_full': window_full,
            'detrend_pp_x_deg': round(math.degrees(det[0]), 4) if det else None,
            'detrend_pp_y_deg': round(math.degrees(det[1]), 4) if det else None,
            'detrend_pp_max_deg': round(math.degrees(det_max), 4) if det_max is not None else None,
            'raw_pp_x_deg': round(math.degrees(raw[0]), 4) if raw else None,
            'raw_pp_y_deg': round(math.degrees(raw[1]), 4) if raw else None,
            'raw_pp_max_deg': round(math.degrees(max(raw)), 4) if raw else None,
            'period_s': round(period_fft, 3) if period_fft else None,
            'period_model_s': round(period_model, 3),
            'settled': settled,
            'sample_count': len(window_samples),
        }
