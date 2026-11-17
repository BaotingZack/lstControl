"""deepseek-anti — 消摆测试专用：在线估计静态倾角，并按残余摆角阻尼。

只在 ``settle`` 场景注册（PTP 下拉框不会出现）。

模型
----
抓钩摆动的倾角可以写成

    θ(t) = θ* + A·cos(ωt) + B·sin(ωt),      ω = √(g / L_eff)

其中 θ* 就是抓钩**静止后最终会停住的角度**（静态倾角 / 平衡角）。
本模块在一个按摆周期计长的滑动窗口上做三元线性最小二乘，把 θ* 与两个摆动
分量分开；ω 由实时绳长给出（L_eff = sheave − Z），所以 Z 变化时窗口长度和
基函数都跟着变，不需要事先知道摆幅或相位。

控制
----
估计收敛后，只对残余摆动施加阻尼：

    θ_ac = θ − θ*
    Δv   = sign · clip(K · θ_ac, ±max_correction)

``sign`` 默认 +1：对速度伺服小车，``v = +K·θ`` 对应
``θ̈ + (K/L)θ̇ + (g/L)θ = 0``，即 ζ = K/(2√(gL)) 的粘性阻尼。若现场 θ 的正
方向与模型相反时不必改代码：``anti_sway_deepseek_sign`` 是总开关，另外
``anti_sway_deepseek_sign_x`` / ``_sign_y`` 可以单独翻转某一轴——倾角仪轴映射
（pitch→X、roll→Y）的符号如果只错了一个轴，表现就是"一个轴消得掉、另一个轴
越消越大"，此时翻对应轴即可，不要两边一起翻。

展示
----
``debug_snapshot()`` 返回估计值（弧度 + 度）、拟合摆幅、残差、收敛进度。
``run_pd_control`` 只在策略提供该方法时才把结果放进 step_data，因此其它
算法不会多出任何字段、行为完全不变。前端在消摆页左侧顶栏显示。
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any

from anti_sway.base import AntiSwayStepContext, VelocityCorrection
from anti_sway.rope import RopeLengthModel
from anti_sway.settle import GRAVITY

MODE_ID = 'deepseek_anti'
MODE_LABEL = '防摇 deepseek-anti'
MODE_DESCRIPTION = (
    '拟合静态倾角 θ* 并标注；Δv=K_eff·(θ_ac+T_lead·θ̇_ac) 相位超前阻尼，'
    '补偿 PLC 回路滞后，带绳长调度/位移预算/增长保护。仅消摆测试可选。'
)

_TWO_PI = 2.0 * math.pi
_MIN_SAMPLES = 8
# 至少覆盖一个完整摆周期，直流分量与摆动项才分得开。实测 0.6 个周期时后
# 估计出的 θ* 仍带 ~0.7° 偏差；满一个周期后残差降到 <0.1°。
_MIN_FIT_PERIODS = 1.0
# 判稳/展示仍用 _MIN_FIT_PERIODS；控制可更早切到 θ−θ*，缩短大摆角初期的等窗时间。
_CONTROL_MIN_PERIODS = 0.65


def _solve3(matrix: list[list[float]], rhs: list[float]) -> list[float] | None:
    """Solve a 3x3 linear system with partial pivoting; None if singular."""
    rows = [
        [matrix[r][0], matrix[r][1], matrix[r][2], rhs[r]]
        for r in range(3)
    ]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(rows[r][col]))
        if abs(rows[pivot][col]) < 1e-12:
            return None
        rows[col], rows[pivot] = rows[pivot], rows[col]
        pv = rows[col][col]
        for r in range(col + 1, 3):
            factor = rows[r][col] / pv
            if factor == 0.0:
                continue
            for c in range(col, 4):
                rows[r][c] -= factor * rows[col][c]
    out = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        acc = rows[r][3] - sum(rows[r][c] * out[c] for c in range(r + 1, 3))
        out[r] = acc / rows[r][r]
    return out


class _AxisStaticEstimator:
    """单轴静态角估计：对 θ(t) = θ* + A·cos(ωt) + B·sin(ωt) 做滑窗最小二乘。"""

    def __init__(
        self,
        *,
        window_periods: float = 1.1,
        min_window_s: float = 2.0,
        min_window_periods: float = 0.85,
        min_window_floor_s: float = 0.55,
        max_window_s: float = 12.0,
        static_slew_amp_rad: float = math.radians(0.12),
        static_slew_rad_per_s: float = math.radians(1.1),
        static_freeze_amp_rad: float = math.radians(0.22),
    ):
        if window_periods <= 0:
            raise ValueError('window_periods must be positive')
        if min_window_s <= 0 or max_window_s < min_window_s:
            raise ValueError('invalid window bounds')
        self.window_periods = float(window_periods)
        self.min_window_s = float(min_window_s)
        self.min_window_periods = float(min_window_periods)
        self.min_window_floor_s = float(min_window_floor_s)
        self.max_window_s = float(max_window_s)
        self.static_slew_amp_rad = max(0.0, float(static_slew_amp_rad))
        self.static_slew_rad_per_s = max(0.0, float(static_slew_rad_per_s))
        self.static_freeze_amp_rad = max(0.0, float(static_freeze_amp_rad))
        self.reset()

    def reset(self) -> None:
        self._samples: deque[tuple[float, float]] = deque()
        self.t = 0.0
        self.static = 0.0
        self.amplitude = 0.0
        self.rms = 0.0
        self.coverage = 0.0
        self.window_s = self.min_window_s
        self.ready = False
        # 最近一次拟合的摆动分量，用于解析求导（避免对 10 Hz 量化信号做差分）
        self._a_cos = 0.0
        self._a_sin = 0.0
        self._omega = 0.0
        self._t_last = 0.0
        self._static_frozen = False

    def update(self, theta: float, dt: float, omega: float) -> float:
        """Feed one sample; returns the current static angle estimate [rad]."""
        dt = max(1e-4, float(dt))
        omega = max(1e-3, float(omega))
        t_now = self.t
        self._samples.append((t_now, float(theta)))
        self.t += dt
        self._t_last = t_now

        period = _TWO_PI / omega
        min_by_period = max(
            self.min_window_floor_s,
            self.min_window_periods * period,
        )
        eff_min = min(self.min_window_s, min_by_period)
        self.window_s = min(
            self.max_window_s,
            max(eff_min, self.window_periods * period),
        )
        while self._samples and (self.t - self._samples[0][0]) > self.window_s:
            self._samples.popleft()

        n = len(self._samples)
        if n < _MIN_SAMPLES:
            self.static = sum(v for _, v in self._samples) / n if n else 0.0
            self.amplitude = 0.0
            self.rms = 0.0
            self.coverage = 0.0
            self.ready = False
            self._a_cos = self._a_sin = 0.0
            self._omega = omega
            return self.static

        span = self.t - self._samples[0][0]
        self.coverage = span / period

        s0 = float(n)
        s1 = s2 = s3 = s4 = s5 = 0.0
        r0 = r1 = r2 = 0.0
        for t, y in self._samples:
            c = math.cos(omega * t)
            s = math.sin(omega * t)
            s1 += c
            s2 += s
            s3 += c * c
            s4 += c * s
            s5 += s * s
            r0 += y
            r1 += y * c
            r2 += y * s

        sol = _solve3(
            [[s0, s1, s2], [s1, s3, s4], [s2, s4, s5]],
            [r0, r1, r2],
        )
        if sol is None:
            self.static = r0 / n
            self.amplitude = 0.0
            self.rms = 0.0
            self.ready = False
            self._a_cos = self._a_sin = 0.0
            self._omega = omega
            return self.static

        bias, a_cos, a_sin = sol
        err = 0.0
        for t, y in self._samples:
            fit = bias + a_cos * math.cos(omega * t) + a_sin * math.sin(omega * t)
            err += (y - fit) ** 2

        amp_now = math.hypot(a_cos, a_sin)
        unfreeze = (
            self.static_freeze_amp_rad > 0.0
            and amp_now > self.static_freeze_amp_rad * 1.75
        )
        if unfreeze:
            self._static_frozen = False
        freeze = (
            self.ready
            and self.static_freeze_amp_rad > 0.0
            and amp_now < self.static_freeze_amp_rad
        )
        if freeze:
            self._static_frozen = True
        elif (
            self.static_slew_rad_per_s > 0.0
            and amp_now < self.static_slew_amp_rad
        ):
            slew = self.static_slew_rad_per_s * dt
            bias = self.static + max(-slew, min(slew, bias - self.static))
        if self._static_frozen:
            bias = self.static

        self.static = bias
        self.amplitude = math.hypot(a_cos, a_sin)
        self.rms = math.sqrt(err / n)
        self.ready = self.coverage >= _MIN_FIT_PERIODS
        self._a_cos = a_cos
        self._a_sin = a_sin
        self._omega = omega
        return self.static

    def control_ready(self) -> bool:
        """拟合可用于阻尼（θ_ac / 解析 θ̇），比 ``ready`` 更早，不参与判稳。"""
        if len(self._samples) < _MIN_SAMPLES or self._omega <= 0.0:
            return False
        return self.coverage >= _CONTROL_MIN_PERIODS

    def model_rate(self) -> float:
        """摆动分量在最新采样时刻的解析导数 θ̇_ac [rad/s]（未收敛时返回 0）。"""
        if not self.control_ready() or self._omega <= 0.0:
            return 0.0
        w = self._omega
        tl = self._t_last
        return -self._a_cos * w * math.sin(w * tl) + self._a_sin * w * math.cos(w * tl)

    def snapshot(self) -> dict[str, Any]:
        return {
            'static_rad': self.static,
            'static_deg': math.degrees(self.static),
            'amp_deg': math.degrees(self.amplitude),
            'rms_deg': math.degrees(self.rms),
            'coverage': self.coverage,
            'window_s': self.window_s,
            'ready': self.ready,
            'samples': len(self._samples),
            'static_frozen': self._static_frozen,
        }


class _AxisGuard:
    """摆幅增长保护：某轴摆幅持续变大时暂停该轴修正。

    以"该轴摆幅历史最小值"为基线：正常消摆时基线单调下移；一旦当前摆幅超过
    基线 ``ratio`` 倍且高于 ``min_amp_deg``，连续两次检查即暂停
    ``suspend_periods`` 个摆周期，然后以当时的摆幅为新基线重新武装。
    这样符号/超前参数配错时不会把摆动越推越大（只损失一次试验）。
    """

    def __init__(
        self,
        *,
        enabled: bool = True,
        ratio: float = 1.25,
        min_amp_deg: float = 0.5,
        suspend_periods: float = 1.5,
    ):
        self.enabled = bool(enabled)
        self.ratio = max(1.0, float(ratio))
        self.min_amp_rad = math.radians(max(0.0, float(min_amp_deg)))
        self.suspend_periods = max(0.0, float(suspend_periods))
        self.reset()

    def reset(self) -> None:
        self.baseline: float | None = None
        self.violations = 0
        self.suspended_until = 0.0
        self.trips = 0
        self.suspended = False

    def update(
        self,
        *,
        t: float,
        amp_rad: float,
        period_s: float,
        active: bool,
    ) -> bool:
        """Return True when the axis may keep correcting."""
        if not self.enabled:
            return True
        if t < self.suspended_until:
            self.suspended = True
            return False
        self.suspended = False
        if not active:
            self.violations = 0
            return True
        if self.baseline is None or amp_rad < self.baseline:
            self.baseline = amp_rad
        threshold = max(self.baseline * self.ratio, self.min_amp_rad)
        if amp_rad > threshold:
            self.violations += 1
        else:
            self.violations = 0
        if self.violations >= 2:
            self.violations = 0
            self.trips += 1
            self.suspended_until = t + self.suspend_periods * max(period_s, 0.5)
            self.baseline = amp_rad
            self.suspended = True
            return False
        return True


class DeepseekAntiController:
    """静态角估计 + 相位超前阻尼。

        θ_ac = θ − θ*                （估计收敛后；否则直接用 θ）
        θ̇_ac = 拟合正弦分量的解析导数
        Δv   = sign · K_eff · (θ_ac + T_lead · θ̇_ac)

    T_lead 的作用：抵消执行器一阶滞后。若回路可近似为 ``1/(1+τs)``，取
    ``T_lead ≈ τ`` 时 ``(1+T_lead·s)/(1+τs) ≈ 1``，摆频处的相位滞后被补回，
    阻尼重新生效。数值扫描（仓库自己的一阶伺服模型）给出的最优点正是
    ``T_lead ≈ τ``：τ=1.5 s 时纯 K·θ 在 5 s 还剩 1.94°，T_lead=1.5 s 降到
    0.07°；而仿真对象 τ=0.18 s 时 T_lead 应取 0.18 s（过大会反而变慢）。

    ``K_eff`` 可按绳长调度：K_eff = min(K, 2·ζ_ref·√(gL))，只在绳长变短时下调，
    保持阻尼比恒定、不放大增益。
    """

    def __init__(
        self,
        rope_model: RopeLengthModel,
        *,
        gain: float = 3.2,
        max_correction: float = 0.35,
        deadband_rad: float = math.radians(0.08),
        sign: float = 1.0,
        sign_x: float = 1.0,
        sign_y: float = 1.0,
        window_periods: float = 1.1,
        min_window_s: float = 2.0,
        min_window_periods: float = 0.85,
        min_window_floor_s: float = 0.55,
        max_window_s: float = 12.0,
        lead_s: float = 1.5,
        sim_lead_s: float = 0.2,
        lead_ac_cap_ratio: float = 1.05,
        lead_ac_cap_min_deg: float = 0.35,
        lead_rope_ref_m: float = 3.5,
        lead_rope_exp: float = 1.25,
        lead_short_min_ratio: float = 0.35,
        tail_recenter_gain: float = 0.0,
        lead_taper_ac_deg: float = 0.35,
        static_slew_amp_deg: float = 0.35,
        static_slew_deg_per_s: float = 0.55,
        static_freeze_amp_deg: float = 0.22,
        tail_ema_tau_s: float = 2.8,
        tail_ema_amp_deg: float = 0.30,
        lead_off_ac_deg: float = 0.12,
        tail_deadband_deg: float = 0.025,
        rope_short_m: float = 2.35,
        rope_gain_ref_m: float = 2.5,
        rope_gain_max_scale: float = 1.22,
        zeta_ref: float = 0.0,
        max_shift_m: float = 0.0,
        guard_enabled: bool = True,
        guard_ratio: float = 1.25,
        guard_min_amp_deg: float = 0.5,
        guard_suspend_periods: float = 1.5,
    ):
        if gain < 0 or max_correction < 0:
            raise ValueError('gain and max_correction must be non-negative')
        self.rope_model = rope_model
        self.gain = float(gain)
        self.max_correction = float(max_correction)
        self.deadband_rad = max(0.0, float(deadband_rad))
        # sign 是总开关，sign_x / sign_y 允许逐轴翻转：两个水平轴的物理对称性
        # 由倾角仪轴映射（pitch→X, roll→Y）决定，映射符号错一个轴时只会表现为
        # "一个轴消得掉、另一个轴越消越大"，因此必须能独立修正。
        self.sign = 1.0 if float(sign) >= 0 else -1.0
        self.sign_x = self.sign * (1.0 if float(sign_x) >= 0 else -1.0)
        self.sign_y = self.sign * (1.0 if float(sign_y) >= 0 else -1.0)
        self.lead_s = max(0.0, float(lead_s))
        self.sim_lead_s = max(0.0, float(sim_lead_s))
        self.lead_ac_cap_ratio = max(0.0, float(lead_ac_cap_ratio))
        self.lead_ac_cap_min_rad = math.radians(
            max(0.0, float(lead_ac_cap_min_deg))
        )
        self.lead_rope_ref_m = max(0.5, float(lead_rope_ref_m))
        self.lead_rope_exp = max(0.0, float(lead_rope_exp))
        self.lead_short_min_ratio = min(1.0, max(0.0, float(lead_short_min_ratio)))
        self.tail_recenter_gain = min(1.0, max(0.0, float(tail_recenter_gain)))
        self.lead_taper_ac_rad = math.radians(max(0.0, float(lead_taper_ac_deg)))
        slew_amp = math.radians(max(0.0, float(static_slew_amp_deg)))
        slew_rate = math.radians(max(0.0, float(static_slew_deg_per_s)))
        freeze_amp = math.radians(max(0.0, float(static_freeze_amp_deg)))
        self.tail_ema_tau_s = max(0.2, float(tail_ema_tau_s))
        self.tail_ema_amp_rad = math.radians(max(0.0, float(tail_ema_amp_deg)))
        self.lead_off_ac_rad = math.radians(max(0.0, float(lead_off_ac_deg)))
        self.tail_deadband_rad = math.radians(max(0.0, float(tail_deadband_deg)))
        self.rope_short_m = max(0.5, float(rope_short_m))
        self.rope_gain_ref_m = max(0.5, float(rope_gain_ref_m))
        self.rope_gain_max_scale = max(1.0, float(rope_gain_max_scale))
        self.zeta_ref = max(0.0, float(zeta_ref))
        self.max_shift_m = max(0.0, float(max_shift_m))
        est_kw = dict(
            window_periods=window_periods,
            min_window_s=min_window_s,
            min_window_periods=min_window_periods,
            min_window_floor_s=min_window_floor_s,
            max_window_s=max_window_s,
            static_slew_amp_rad=slew_amp,
            static_slew_rad_per_s=slew_rate,
            static_freeze_amp_rad=freeze_amp,
        )
        self._est_x = _AxisStaticEstimator(**est_kw)
        self._est_y = _AxisStaticEstimator(**est_kw)
        self._guard_x = _AxisGuard(
            enabled=guard_enabled, ratio=guard_ratio,
            min_amp_deg=guard_min_amp_deg,
            suspend_periods=guard_suspend_periods,
        )
        self._guard_y = _AxisGuard(
            enabled=guard_enabled, ratio=guard_ratio,
            min_amp_deg=guard_min_amp_deg,
            suspend_periods=guard_suspend_periods,
        )
        self._last_theta_x = 0.0
        self._last_theta_y = 0.0
        self._last_ac_x = 0.0
        self._last_ac_y = 0.0
        self._last_rate_x = 0.0
        self._last_rate_y = 0.0
        self._last_u_x = 0.0
        self._last_u_y = 0.0
        self._last_rope_m = 0.0
        self._last_period_s = 0.0
        self._last_gain_eff = self.gain
        self._last_lead_s = self.lead_s
        self._last_dist_xy = 0.0
        self._last_sign_x = self.sign
        self._last_sign_y = self.sign
        self._budget_blocked = False
        self._last_recenter_x = False
        self._last_recenter_y = False
        self._theta_ema_x = 0.0
        self._theta_ema_y = 0.0
        self._tail_ema_x = False
        self._tail_ema_y = False
        self._last_lead_x = 0.0
        self._last_lead_y = 0.0

    def reset(self) -> None:
        self._est_x.reset()
        self._est_y.reset()
        self._guard_x.reset()
        self._guard_y.reset()
        self._last_theta_x = self._last_theta_y = 0.0
        self._last_ac_x = self._last_ac_y = 0.0
        self._last_rate_x = self._last_rate_y = 0.0
        self._last_u_x = self._last_u_y = 0.0
        self._budget_blocked = False
        self._last_recenter_x = self._last_recenter_y = False
        self._theta_ema_x = self._theta_ema_y = 0.0
        self._tail_ema_x = self._tail_ema_y = False

    def _axis_ac(
        self,
        est: _AxisStaticEstimator,
        theta: float,
        dt: float,
        ema_state: str,
    ) -> tuple[float, bool]:
        """θ_ac；尾段 LS 摆幅很小时用慢 EMA 作基准（避免 θ*≈θ 导致无输出）。"""
        if not est.control_ready():
            return float(theta), False
        tail = bool(
            est.ready
            and self.tail_ema_amp_rad > 0.0
            and est.amplitude < self.tail_ema_amp_rad
        )
        if tail:
            alpha = dt / (self.tail_ema_tau_s + dt)
            if ema_state == 'x':
                self._theta_ema_x += alpha * (float(theta) - self._theta_ema_x)
                base = self._theta_ema_x
                self._tail_ema_x = True
            else:
                self._theta_ema_y += alpha * (float(theta) - self._theta_ema_y)
                base = self._theta_ema_y
                self._tail_ema_y = True
            return float(theta) - base, True
        if ema_state == 'x':
            self._tail_ema_x = False
            self._theta_ema_x += 0.1 * (est.static - self._theta_ema_x)
        else:
            self._tail_ema_y = False
            self._theta_ema_y += 0.1 * (est.static - self._theta_ema_y)
        return float(theta) - est.static, False

    def _lead_taper(self, lead_s: float, ac: float) -> float:
        if lead_s <= 0.0:
            return 0.0
        if self.lead_off_ac_rad > 0.0 and abs(ac) < self.lead_off_ac_rad:
            return 0.0
        if self.lead_taper_ac_rad <= 0.0:
            return lead_s
        return lead_s * min(1.0, abs(ac) / self.lead_taper_ac_rad)

    def _effective_gain(self, rope_m: float) -> float:
        rope_m = max(rope_m, 1e-6)
        if self.zeta_ref <= 0.0:
            k = float(self.gain)
            ref = self.rope_gain_ref_m
            # 仅短绳抬 K；长绳保持配置 gain，避免长绳变弱。
            if ref > 0.0 and rope_m < ref:
                scale = math.sqrt(ref / rope_m)
                scale = min(self.rope_gain_max_scale, max(1.0, scale))
                k *= scale
            return k
        k_rope = 2.0 * self.zeta_ref * math.sqrt(GRAVITY * rope_m)
        return min(self.gain, k_rope)

    def _effective_lead(
        self,
        lead_s: float,
        rope_m: float,
        is_simulation: bool,
    ) -> float:
        """现场 T_lead 按长绳标定；绳变短后 ω↑，固定 1.5s 易过补相位并在尾段与 θ_ac 相消。"""
        if is_simulation or lead_s <= 0.0:
            return lead_s
        ref = self.lead_rope_ref_m
        if rope_m >= ref:
            return lead_s
        ratio = max(1e-6, rope_m / ref)
        scale = max(self.lead_short_min_ratio, ratio ** self.lead_rope_exp)
        return lead_s * scale

    def _combined_ac(
        self,
        ac: float,
        rate: float,
        omega: float,
        lead_s: float,
        rope_m: float,
    ) -> float:
        if lead_s <= 0.0:
            return ac
        lead_add = lead_s * rate
        # 长绳：保留完整 T_lead·θ̇（现场 1.5s 补偿 PLC 滞后）。短绳才限幅防过补偿。
        if rope_m >= self.rope_short_m:
            return ac + lead_add
        if (
            self.lead_ac_cap_ratio > 0.0
            and abs(ac) >= self.lead_ac_cap_min_rad
        ):
            cap = abs(ac) * self.lead_ac_cap_ratio
            if cap > 0.0 and abs(lead_add) > cap:
                lead_add = math.copysign(cap, lead_add)
        wtau = lead_s * max(omega, 0.0)
        if wtau > 3.0 and abs(ac) >= self.lead_ac_cap_min_rad:
            lead_add *= 1.0 / (1.0 + (wtau - 3.0) ** 2)
        return ac + lead_add

    def _law(
        self,
        ac: float,
        rate: float,
        k_eff: float,
        lead_s: float,
        sign: float,
        omega: float,
        rope_m: float,
        *,
        deadband_rad: float | None = None,
    ) -> float:
        db = self.deadband_rad if deadband_rad is None else max(0.0, deadband_rad)
        eff = self._combined_ac(ac, rate, omega, lead_s, rope_m)
        if abs(eff) < db:
            # 近平衡：T_lead·θ̇ 与 θ_ac 易相消，combined 掉进死区；保留 P 或 D 单独作用。
            if abs(ac) >= db:
                eff = ac
            elif lead_s > 0.0 and abs(lead_s * rate) >= db:
                eff = lead_s * rate
            elif abs(ac) * k_eff >= 0.5 * db:
                eff = ac
            else:
                return 0.0
        u = sign * k_eff * eff
        return max(-self.max_correction, min(self.max_correction, u))

    def compute(
        self,
        theta_x: float,
        theta_y: float,
        dt: float,
        z_grab: float,
        dist_xy: float = 0.0,
        is_simulation: bool = True,
    ) -> tuple[float, float]:
        L = self.rope_model.compute(z_grab)
        omega = math.sqrt(GRAVITY / max(L, 1e-6))
        period = _TWO_PI / omega
        k_eff = self._effective_gain(L)
        lead_base = self.sim_lead_s if is_simulation else self.lead_s
        lead_s = self._effective_lead(lead_base, L, is_simulation)

        tx = float(theta_x)
        ty = float(theta_y)
        static_x = self._est_x.update(tx, dt, omega)
        static_y = self._est_y.update(ty, dt, omega)
        ac_x, ema_x = self._axis_ac(self._est_x, tx, dt, 'x')
        ac_y, ema_y = self._axis_ac(self._est_y, ty, dt, 'y')
        self._last_recenter_x = False
        self._last_recenter_y = False
        lead_x = self._lead_taper(lead_s, ac_x)
        lead_y = self._lead_taper(lead_s, ac_y)
        self._last_lead_x = lead_x
        self._last_lead_y = lead_y
        rate_x = self._est_x.model_rate() if not ema_x else 0.0
        rate_y = self._est_y.model_rate() if not ema_y else 0.0
        db_x = self.tail_deadband_rad if ema_x else self.deadband_rad
        db_y = self.tail_deadband_rad if ema_y else self.deadband_rad

        # 逐轴符号修的是"现场倾角仪轴映射"的极性：仿真摆角是按物理定义直接
        # 生成的，不存在映射误差，所以仿真里只用总符号 sign。
        sign_x = self.sign if is_simulation else self.sign_x
        sign_y = self.sign if is_simulation else self.sign_y

        # 位移预算：小车离锚点超过预算时，本周期不再加速它，交给位置环拉回。
        # settle 场景里 ctx.dist_xy 就是"到锚点的水平距离"。
        blocked = self.max_shift_m > 0.0 and float(dist_xy) > self.max_shift_m
        if blocked:
            u_x = u_y = 0.0
        else:
            u_x = self._law(
                ac_x, rate_x, k_eff, lead_x, sign_x, omega, L, deadband_rad=db_x,
            )
            u_y = self._law(
                ac_y, rate_y, k_eff, lead_y, sign_y, omega, L, deadband_rad=db_y,
            )

        allow_x = self._guard_x.update(
            t=self._est_x.t, amp_rad=self._est_x.amplitude,
            period_s=period,
            active=(not blocked) and abs(u_x) > 0.0 and self._est_x.control_ready(),
        )
        allow_y = self._guard_y.update(
            t=self._est_y.t, amp_rad=self._est_y.amplitude,
            period_s=period,
            active=(not blocked) and abs(u_y) > 0.0 and self._est_y.control_ready(),
        )
        if not allow_x:
            u_x = 0.0
        if not allow_y:
            u_y = 0.0

        self._last_theta_x = tx
        self._last_theta_y = ty
        self._last_ac_x = ac_x
        self._last_ac_y = ac_y
        self._last_rate_x = rate_x
        self._last_rate_y = rate_y
        self._last_u_x = u_x
        self._last_u_y = u_y
        self._last_rope_m = L
        self._last_period_s = period
        self._last_gain_eff = k_eff
        self._last_sign_x = sign_x
        self._last_sign_y = sign_y
        self._last_lead_s = max(lead_x, lead_y)
        self._last_dist_xy = float(dist_xy)
        self._budget_blocked = blocked
        return u_x, u_y

    def debug_snapshot(self) -> dict[str, Any]:
        """Estimator + control state for the UI / run log (degrees first)."""
        ex = self._est_x.snapshot()
        ey = self._est_y.snapshot()
        return {
            'mode': MODE_ID,
            'theta_static_x_deg': ex['static_deg'],
            'theta_static_y_deg': ey['static_deg'],
            'theta_static_x_rad': ex['static_rad'],
            'theta_static_y_rad': ey['static_rad'],
            'theta_x_deg': math.degrees(self._last_theta_x),
            'theta_y_deg': math.degrees(self._last_theta_y),
            'theta_ac_x_deg': math.degrees(self._last_ac_x),
            'theta_ac_y_deg': math.degrees(self._last_ac_y),
            'rate_x_deg_s': math.degrees(self._last_rate_x),
            'rate_y_deg_s': math.degrees(self._last_rate_y),
            'amp_x_deg': ex['amp_deg'],
            'amp_y_deg': ey['amp_deg'],
            'rms_x_deg': ex['rms_deg'],
            'rms_y_deg': ey['rms_deg'],
            'coverage_x': ex['coverage'],
            'coverage_y': ey['coverage'],
            'ready_x': ex['ready'],
            'ready_y': ey['ready'],
            'ready': bool(ex['ready'] and ey['ready']),
            'window_s': ex['window_s'],
            'period_s': self._last_period_s,
            'rope_m': self._last_rope_m,
            'dv_x': self._last_u_x,
            'dv_y': self._last_u_y,
            'gain': self.gain,
            'gain_eff': self._last_gain_eff,
            'sign': self.sign,
            'sign_x': self._last_sign_x,
            'sign_y': self._last_sign_y,
            'lead_s': self._last_lead_s,
            'dist_xy': self._last_dist_xy,
            'budget_blocked': self._budget_blocked,
            'guard_x': self._guard_x.suspended,
            'guard_y': self._guard_y.suspended,
            'guard_trips': self._guard_x.trips + self._guard_y.trips,
            'tail_recenter_x': self._last_recenter_x,
            'tail_recenter_y': self._last_recenter_y,
            'static_frozen_x': ex.get('static_frozen'),
            'static_frozen_y': ey.get('static_frozen'),
            'tail_ema_x': self._tail_ema_x,
            'tail_ema_y': self._tail_ema_y,
            'lead_s_x': self._last_lead_x,
            'lead_s_y': self._last_lead_y,
        }

class DeepseekAntiStrategy:
    mode_id = MODE_ID

    def __init__(self, controller: DeepseekAntiController, config):
        self._ctrl = controller
        self._config = config

    def log_startup(self, *, verbose: bool) -> None:
        if not verbose:
            return
        print(
            f'[防摇 deepseek-anti] Δv=sign·K_eff·(θ_ac+T_lead·θ̇_ac), '
            f'K={self._ctrl.gain}, sign={self._ctrl.sign:+.0f}, '
            f'lead={self._ctrl.lead_s:.2f}s (仿真 {self._ctrl.sim_lead_s:.2f}s), '
            f'ζ_ref={self._ctrl.zeta_ref:.2f}, '
            f'窗口={self._ctrl._est_x.window_periods:g} 周期, '
            f'max={self._ctrl.max_correction} m/s, '
            f'预算={self._ctrl.max_shift_m:.2f}m'
        )

    def reset(self) -> None:
        self._ctrl.reset()

    def correct_xy(
        self,
        vx_cmd: float,
        vy_cmd: float,
        ctx: AntiSwayStepContext,
    ) -> VelocityCorrection:
        u_x, u_y = self._ctrl.compute(
            ctx.theta_x, ctx.theta_y, ctx.dt, ctx.z_grab,
            dist_xy=ctx.dist_xy,
            is_simulation=getattr(ctx, 'is_simulation', True),
        )
        return VelocityCorrection(
            vx_cmd=vx_cmd + u_x,
            vy_cmd=vy_cmd + u_y,
            dvx=u_x,
            dvy=u_y,
        )

    def debug_snapshot(self) -> dict[str, Any]:
        return self._ctrl.debug_snapshot()


def config_fields_from_site(site: dict[str, Any] | None) -> dict[str, Any]:
    site = site or {}
    return {
        'anti_sway_deepseek_gain': float(
            site.get('anti_sway_deepseek_gain', 3.2)
        ),
        'anti_sway_deepseek_max_correction': float(
            site.get('anti_sway_deepseek_max_correction', 0.35)
        ),
        'anti_sway_deepseek_deadband_deg': float(
            site.get('anti_sway_deepseek_deadband_deg', 0.08)
        ),
        'anti_sway_deepseek_window_periods': float(
            site.get('anti_sway_deepseek_window_periods', 1.1)
        ),
        'anti_sway_deepseek_sign': float(
            site.get('anti_sway_deepseek_sign', 1.0)
        ),
        # 逐轴符号：两轴共享同一控制律，但 θ 的极性来自倾角仪轴映射，
        # 映射错一个轴时只会表现为该轴越消越大。
        'anti_sway_deepseek_sign_x': float(
            site.get('anti_sway_deepseek_sign_x', 1.0)
        ),
        'anti_sway_deepseek_sign_y': float(
            site.get('anti_sway_deepseek_sign_y', 1.0)
        ),
        'anti_sway_deepseek_lead_s': float(
            site.get('anti_sway_deepseek_lead_s', 1.5)
        ),
        'anti_sway_deepseek_lead_ac_cap_ratio': float(
            site.get('anti_sway_deepseek_lead_ac_cap_ratio', 1.05)
        ),
        'anti_sway_deepseek_lead_ac_cap_min_deg': float(
            site.get('anti_sway_deepseek_lead_ac_cap_min_deg', 0.35)
        ),
        'anti_sway_deepseek_lead_rope_ref_m': float(
            site.get('anti_sway_deepseek_lead_rope_ref_m', 3.5)
        ),
        'anti_sway_deepseek_lead_rope_exp': float(
            site.get('anti_sway_deepseek_lead_rope_exp', 1.25)
        ),
        'anti_sway_deepseek_lead_short_min_ratio': float(
            site.get('anti_sway_deepseek_lead_short_min_ratio', 0.35)
        ),
        'anti_sway_deepseek_tail_recenter_gain': float(
            site.get('anti_sway_deepseek_tail_recenter_gain', 0.0)
        ),
        'anti_sway_deepseek_lead_taper_ac_deg': float(
            site.get('anti_sway_deepseek_lead_taper_ac_deg', 0.35)
        ),
        'anti_sway_deepseek_static_slew_deg_per_s': float(
            site.get('anti_sway_deepseek_static_slew_deg_per_s', 0.55)
        ),
        'anti_sway_deepseek_static_freeze_amp_deg': float(
            site.get('anti_sway_deepseek_static_freeze_amp_deg', 0.22)
        ),
        'anti_sway_deepseek_tail_ema_tau_s': float(
            site.get('anti_sway_deepseek_tail_ema_tau_s', 2.8)
        ),
        'anti_sway_deepseek_tail_ema_amp_deg': float(
            site.get('anti_sway_deepseek_tail_ema_amp_deg', 0.30)
        ),
        'anti_sway_deepseek_lead_off_ac_deg': float(
            site.get('anti_sway_deepseek_lead_off_ac_deg', 0.12)
        ),
        'anti_sway_deepseek_tail_deadband_deg': float(
            site.get('anti_sway_deepseek_tail_deadband_deg', 0.025)
        ),
        'anti_sway_deepseek_min_window_periods': float(
            site.get('anti_sway_deepseek_min_window_periods', 0.85)
        ),
        'anti_sway_deepseek_rope_short_m': float(
            site.get('anti_sway_deepseek_rope_short_m', 2.35)
        ),
        'anti_sway_deepseek_rope_gain_ref_m': float(
            site.get('anti_sway_deepseek_rope_gain_ref_m', 2.5)
        ),
        'anti_sway_deepseek_rope_gain_max_scale': float(
            site.get('anti_sway_deepseek_rope_gain_max_scale', 1.22)
        ),
        'anti_sway_deepseek_zeta_ref': float(
            site.get('anti_sway_deepseek_zeta_ref', 0.0)
        ),
        'anti_sway_deepseek_max_shift_m': float(
            site.get('anti_sway_deepseek_max_shift_m', 0.0)
        ),
        'anti_sway_deepseek_guard': float(
            site.get('anti_sway_deepseek_guard', 1.0)
        ),
    }


def build_strategy(config) -> DeepseekAntiStrategy | None:
    if not getattr(config, 'enable_anti_sway', False):
        return None
    if getattr(config, 'anti_sway_mode', 'off') != MODE_ID:
        return None
    gain = float(getattr(config, 'anti_sway_deepseek_gain', 0.0))
    if gain <= 0.0:
        return None
    rope = RopeLengthModel(
        sheave_height=getattr(config, 'rope_length_sheave_height', 4.0),
        grab_offset=getattr(config, 'rope_length_grab_offset', 0.0),
        cable_stretch=getattr(config, 'rope_length_cable_stretch', 0.0),
        min_length=getattr(config, 'rope_length_min', 0.5),
    )
    ctrl = DeepseekAntiController(
        rope,
        gain=gain,
        max_correction=float(
            getattr(config, 'anti_sway_deepseek_max_correction', 0.35)
        ),
        deadband_rad=math.radians(
            float(getattr(config, 'anti_sway_deepseek_deadband_deg', 0.10))
        ),
        sign=float(getattr(config, 'anti_sway_deepseek_sign', 1.0)),
        sign_x=float(getattr(config, 'anti_sway_deepseek_sign_x', 1.0)),
        sign_y=float(getattr(config, 'anti_sway_deepseek_sign_y', 1.0)),
        window_periods=float(
            getattr(config, 'anti_sway_deepseek_window_periods', 1.1)
        ),
        min_window_periods=float(
            getattr(config, 'anti_sway_deepseek_min_window_periods', 0.85)
        ),
        lead_s=float(getattr(config, 'anti_sway_deepseek_lead_s', 1.5)),
        sim_lead_s=float(getattr(config, 'servo_time_constant_xy', 0.2)),
        lead_ac_cap_ratio=float(
            getattr(config, 'anti_sway_deepseek_lead_ac_cap_ratio', 1.05)
        ),
        lead_ac_cap_min_deg=float(
            getattr(config, 'anti_sway_deepseek_lead_ac_cap_min_deg', 0.35)
        ),
        lead_rope_ref_m=float(
            getattr(config, 'anti_sway_deepseek_lead_rope_ref_m', 3.5)
        ),
        lead_rope_exp=float(
            getattr(config, 'anti_sway_deepseek_lead_rope_exp', 1.25)
        ),
        lead_short_min_ratio=float(
            getattr(config, 'anti_sway_deepseek_lead_short_min_ratio', 0.35)
        ),
        tail_recenter_gain=float(
            getattr(config, 'anti_sway_deepseek_tail_recenter_gain', 0.0)
        ),
        lead_taper_ac_deg=float(
            getattr(config, 'anti_sway_deepseek_lead_taper_ac_deg', 0.35)
        ),
        static_slew_deg_per_s=float(
            getattr(config, 'anti_sway_deepseek_static_slew_deg_per_s', 0.55)
        ),
        static_freeze_amp_deg=float(
            getattr(config, 'anti_sway_deepseek_static_freeze_amp_deg', 0.22)
        ),
        tail_ema_tau_s=float(
            getattr(config, 'anti_sway_deepseek_tail_ema_tau_s', 2.8)
        ),
        tail_ema_amp_deg=float(
            getattr(config, 'anti_sway_deepseek_tail_ema_amp_deg', 0.30)
        ),
        lead_off_ac_deg=float(
            getattr(config, 'anti_sway_deepseek_lead_off_ac_deg', 0.12)
        ),
        tail_deadband_deg=float(
            getattr(config, 'anti_sway_deepseek_tail_deadband_deg', 0.025)
        ),
        rope_short_m=float(
            getattr(config, 'anti_sway_deepseek_rope_short_m', 2.35)
        ),
        rope_gain_ref_m=float(
            getattr(config, 'anti_sway_deepseek_rope_gain_ref_m', 2.5)
        ),
        rope_gain_max_scale=float(
            getattr(config, 'anti_sway_deepseek_rope_gain_max_scale', 1.22)
        ),
        zeta_ref=float(getattr(config, 'anti_sway_deepseek_zeta_ref', 0.0)),
        max_shift_m=float(
            getattr(config, 'anti_sway_deepseek_max_shift_m', 0.06)
        ),
        guard_enabled=bool(getattr(config, 'anti_sway_deepseek_guard', 1.0)),
    )
    return DeepseekAntiStrategy(ctrl, config)
