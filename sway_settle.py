"""判稳阶段倾角指标 — 与固定零偏解耦的摆动幅值估计。

现场问题:
  - 倾角仪静态零偏随抓钩高度、张合状态变化 (见 site_config / doc/倾角仪初值)
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
HOIST_SHEAVE_HEIGHT = 4.0   # 电子葫芦出绳点离地 [m]
HOOK_STRUCTURE = 1.5        # 抓钩工作点 → 钢丝绳连接处 [m]


def effective_rope_length(
    z_grab: float,
    *,
    sheave_height: float = HOIST_SHEAVE_HEIGHT,
    grab_offset: float = 0.0,
    cable_stretch: float = 0.0,
    min_length: float = 0.5,
) -> float:
    """有效摆长 L_eff [m] (与 sway_controller.RopeLengthModel 一致)。

    现场: L_eff = H − Z (sheave_height=4, grab_offset=0)。不要把 1.5m 结构
    再加进 grab_offset, 那会变成 H−Z+h, 把结构算了两次。
    """
    L = sheave_height - z_grab + grab_offset + cable_stretch
    return max(L, min_length)


def wire_rope_length(
    z_grab: float,
    *,
    sheave_height: float = HOIST_SHEAVE_HEIGHT,
    structure: float = HOOK_STRUCTURE,
    min_length: float = 0.1,
) -> float:
    """钢丝绳长度 L_rope = H − Z − h [m]。"""
    return max(sheave_height - z_grab - structure, min_length)


def pendulum_period(
    z_grab: float,
    *,
    sheave_height: float = HOIST_SHEAVE_HEIGHT,
    grab_offset: float = 0.0,
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
    sheave_height: float = HOIST_SHEAVE_HEIGHT,
    grab_offset: float = 0.0,
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


def fit_sway_bias(
    sway_samples: list[tuple[float, float, float]],
    period: float,
) -> tuple[float, float] | None:
    """用已知摆周期从摆动信号里解出静态零偏 [rad]。

    动机: 静态零偏随库位/高度/载荷变化 (现场实测空载 vector.y 跨库位跨度 0.78°,
    载货态整体再偏 0.3~0.4°), 单一标定值覆盖不了。但零偏无须标定 —— 它可以直接
    从摆动信号的形状里解出来。

    做法: 摆角 ≈ bias + A·cos(ωt) + B·sin(ωt), ω=√(g/L) 由绳长模型给定。
    这是对 (bias, A, B) 的线性最小二乘, 窗长取一个周期即可精确分离零偏与摆动。
    比"窗内取均值"稳健: 均值只在窗长恰好是整数个周期时才无偏, 这里显式拟合掉
    基频, 窗长不必精确。

    返回 (bias_x, bias_y); 样本不足或窗太短时返回 None。
    """
    if len(sway_samples) < 8 or period <= 0.0:
        return None
    t0 = sway_samples[0][0]
    span = sway_samples[-1][0] - t0
    if span < period * 0.6:      # 不足 0.6 个周期, 基频与直流不可分辨
        return None
    omega = 2.0 * math.pi / period

    # 正规方程: 3x3, 直接解, 不引入 numpy 依赖
    n = len(sway_samples)
    sc = ss = scc = sss = scs = 0.0
    for s in sway_samples:
        dt = s[0] - t0
        c, si = math.cos(omega * dt), math.sin(omega * dt)
        sc += c
        ss += si
        scc += c * c
        sss += si * si
        scs += c * si

    def solve(values: list[float]) -> float | None:
        sv = svc = svs = 0.0
        for (s, v) in zip(sway_samples, values):
            dt = s[0] - t0
            sv += v
            svc += v * math.cos(omega * dt)
            svs += v * math.sin(omega * dt)
        # [[n, sc, ss], [sc, scc, scs], [ss, scs, sss]] · [bias, A, B] = [sv, svc, svs]
        m = [[n, sc, ss], [sc, scc, scs], [ss, scs, sss]]
        rhs = [sv, svc, svs]
        det = (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )
        if abs(det) < 1e-12:
            return None
        # 只需要 bias (第一个未知量) → Cramer 法则
        b = [
            [rhs[0], m[0][1], m[0][2]],
            [rhs[1], m[1][1], m[1][2]],
            [rhs[2], m[2][1], m[2][2]],
        ]
        det_b = (
            b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1])
            - b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0])
            + b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0])
        )
        return det_b / det

    bx = solve([s[1] for s in sway_samples])
    by = solve([s[2] for s in sway_samples])
    if bx is None or by is None:
        return None
    return bx, by


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
    """判稳等待期间的慢变零偏跟踪 → θ_ac = θ − θ_dc, 供阻尼使用。

    零偏初值必须是 0, 不能用"第一帧 θ"。悬停判稳开始的时刻通常正是摆幅最大处
    (PD 刚在目标附近刹停), 用第一帧做初值等于把 θ_dc 错误地锚在一个峰值上, 之后
    再以 τ 衰减 — θ_ac 于是带上一个同号的慢变偏置, 被积分成单向漂移。闭环仿真
    (K_eff=3.9, 初始摆 1°) 实测会把小车一路推到 150mm 的位移预算上限: 既吃光了
    阻尼行程 (预算耗尽后 clamp_damping_axis 只放行半个周期, 阻尼被半波整流),
    又在卸货点留下 15cm 放料偏差。

    从 0 开始学是安全的下限, 但不是最优: 现场实测静态零偏随库位/高度/载荷变化
    (空载 vector.y 跨库位跨度 0.78°, 载货态整体再偏 0.3~0.4°), 标定值扣不干净。
    残余零偏 b 会在 τ 的尺度上透过高通, 积分成 K·b·τ 的一次性漂移 —— 实测
    0.44° 残余 → 7.3cm, 几乎吃掉取放货处 ±5cm 的位移预算。

    因此优先用 seed(): 由 fit_sway_bias() 从摆动信号本身解出零偏并直接置入
    θ_dc, 高通从第一帧就居中, 那段漂移消失。解不出来 (样本不足/窗太短) 时才
    退回 0 起算。
    """

    tau_s: float = 4.0
    dc_x: float = 0.0
    dc_y: float = 0.0

    def reset(self) -> None:
        self.dc_x = 0.0
        self.dc_y = 0.0

    def seed(self, dc_x: float, dc_y: float) -> None:
        """置入零偏初值 [rad] — 供 fit_sway_bias() 的解算结果使用。"""
        self.dc_x = dc_x
        self.dc_y = dc_y

    def update(self, theta_x: float, theta_y: float, dt: float) -> tuple[float, float]:
        dt = max(dt, 1e-4)
        alpha = math.exp(-dt / max(self.tau_s, 0.1))
        self.dc_x = alpha * self.dc_x + (1.0 - alpha) * theta_x
        self.dc_y = alpha * self.dc_y + (1.0 - alpha) * theta_y
        return theta_x - self.dc_x, theta_y - self.dc_y


@dataclass
class SwayAmplitudeTracker:
    """瞬时振幅 Â² = θ_ac² + (θ̇/ωn)² — 每拍都能给出当前摆幅。

    不必等半个周期看峰峰值。θ̇ 用一阶低通差分, 压 10Hz 量化噪声。
    rate_amp = |θ̇|/ωn 对静态零偏接近 0, 可用来区分"真在摆"和"只是偏置"。
    """

    rate_tau: float = 0.25
    _prev_x: float | None = None
    _prev_y: float | None = None
    _rate_x: float = 0.0
    _rate_y: float = 0.0
    last_amp: float = 0.0
    last_rate_amp: float = 0.0
    last_rate_amp_x: float = 0.0
    last_rate_amp_y: float = 0.0
    last_ac_x: float = 0.0
    last_ac_y: float = 0.0

    def reset(self) -> None:
        self._prev_x = None
        self._prev_y = None
        self._rate_x = 0.0
        self._rate_y = 0.0
        self.last_amp = 0.0
        self.last_rate_amp = 0.0
        self.last_rate_amp_x = self.last_rate_amp_y = 0.0
        self.last_ac_x = 0.0
        self.last_ac_y = 0.0

    def update(
        self,
        ac_x: float,
        ac_y: float,
        dt: float,
        omega_n: float,
        *,
        rate_sample_x: float | None = None,
        rate_sample_y: float | None = None,
    ) -> tuple[float, float, float]:
        """返回 (Âx, Ây, Âmax) [rad]。"""
        dt = max(dt, 1e-4)
        wn = max(omega_n, 0.2)
        # 控制活动判断用传感器自身的变化，避免 DC 学习的衰减被误判为真实运动。
        sx = ac_x if rate_sample_x is None else rate_sample_x
        sy = ac_y if rate_sample_y is None else rate_sample_y
        if self._prev_x is None:
            self._prev_x, self._prev_y = sx, sy
        raw_x = (sx - self._prev_x) / dt
        raw_y = (sy - self._prev_y) / dt
        self._prev_x, self._prev_y = sx, sy
        alpha = math.exp(-dt / max(self.rate_tau, 0.05))
        self._rate_x = alpha * self._rate_x + (1.0 - alpha) * raw_x
        self._rate_y = alpha * self._rate_y + (1.0 - alpha) * raw_y
        self.last_ac_x, self.last_ac_y = ac_x, ac_y
        amp_x = math.hypot(ac_x, self._rate_x / wn)
        amp_y = math.hypot(ac_y, self._rate_y / wn)
        self.last_amp = max(amp_x, amp_y)
        self.last_rate_amp_x = abs(self._rate_x) / wn
        self.last_rate_amp_y = abs(self._rate_y) / wn
        self.last_rate_amp = max(self.last_rate_amp_x, self.last_rate_amp_y)
        return amp_x, amp_y, self.last_amp

    def amplitude_at(self, omega_n: float) -> float:
        """用当前 θ_ac/θ̇ 在给定 ωn 下重算 Â [rad]。"""
        wn = max(omega_n, 0.2)
        return max(
            math.hypot(self.last_ac_x, self._rate_x / wn),
            math.hypot(self.last_ac_y, self._rate_y / wn),
        )


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
    sheave_height: float = HOIST_SHEAVE_HEIGHT
    grab_offset: float = 0.0
    tol_deg: float = 0.3
    dc_tau: float = 2.5
    _samples: list[tuple[float, float, float]] | None = None
    _dc: ThetaDcTracker | None = None
    _amp: SwayAmplitudeTracker | None = None
    _last_t: float | None = None

    def __post_init__(self) -> None:
        self._samples = []
        self._dc = ThetaDcTracker(tau_s=self.dc_tau)
        self._amp = SwayAmplitudeTracker()
        self._last_t = None

    def reset(self) -> None:
        self._samples.clear()
        self._dc.reset()
        self._amp.reset()
        self._last_t = None

    def push(self, t: float, theta_x: float, theta_y: float) -> None:
        dt = 0.1 if self._last_t is None else max(t - self._last_t, 1e-3)
        self._last_t = t
        ac_x, ac_y = self._dc.update(theta_x, theta_y, dt)
        # ωn 用典型 L; metrics(z) 会按真实绳长重算 Â
        self._amp.update(ac_x, ac_y, dt, 2.0)
        self._samples.append((t, theta_x, theta_y))
        while len(self._samples) > 1 and t - self._samples[0][0] > self.max_buffer_s:
            self._samples.pop(0)

    def samples(self) -> list[tuple[float, float, float]]:
        """缓存内的原始样本 (t, θx, θy) — 供 fit_sway_bias() 解算零偏。"""
        return list(self._samples)

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
                'amp_max_deg': None,
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
        L = effective_rope_length(
            z_grab, sheave_height=self.sheave_height, grab_offset=self.grab_offset,
        )
        amp_deg = math.degrees(
            self._amp.amplitude_at(math.sqrt(GRAVITY / max(L, 0.5)))
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
            'amp_max_deg': round(amp_deg, 4),
        }
