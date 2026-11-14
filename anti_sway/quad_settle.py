"""四象限超前消摆控制器 (基于倾角仪 + 速度斜坡逆滞后补偿)。

核心控制思想（四象限速度-加速度相位匹配）：
设 θ 为钩子相对原点的前向摆角（向前摆 θ > 0，向后摆 θ < 0），ω = θ̇ 为角速度：
  1. 原点 → 前向最高点 (θ > 0, ω > 0)：向前加速 (期望速度 v* > 0，期望加速度 a* > 0)
  2. 前向最高点 → 回落原点 (θ > 0, ω < 0)：向前减速 (期望速度 v* > 0，期望加速度 a* < 0)
  3. 原点 → 后向最高点 (θ < 0, ω < 0)：向后加速 (期望速度 v* < 0，期望加速度 a* < 0)
  4. 后向最高点 → 回落原点 (θ < 0, ω > 0)：向后减速 (期望速度 v* < 0，期望加速度 a* > 0)

为什么能同时满足这四个象限？
取期望车速：
    v*(t) = Kv · θ(t)
对其求导即得期望加速度：
    a*(t) = dv*/dt = Kv · θ̇(t) = Kv · ω(t)
可以看到 v* 的正负由 θ 决定，a* 的正负由 ω 决定，天然在四个象限精确实现上述加减速方向！

如何解决“只能发速度指令、且驱动器有加速斜坡滞后”的问题？
真实 PLC 速度伺服不会瞬时到达 v_cmd，而是存在加速过程（近似一阶惯性 + 斜坡限幅）：
    a_actual = dv_actual/dt ≈ (v_cmd - v_actual) / τ_ramp
如果直接下发 v_cmd = v*，实际车速会滞后，在钩子刚过原点需要最大前向加速度时车还没动起来，
到了最高点该减速时车却还在加速，甚至反过来助长摆动。

为此引入【斜坡逆模型超前补偿 + 内部车速观测器】：
  1. 超前速度指令：
         v_cmd = v* + τ_lead · a* + k_track · (v* - v_hat)
     - 在 Q1 (θ>0, ω>0)：a*>0，v_cmd 被大幅提前顶高（过冲给定），逼迫驱动器立刻拉出正向最大加速度；
     - 在 Q2 (θ>0, ω<0)：还没等钩子回到原点，a*<0 就提前把 v_cmd 往下拉（甚至拉回 0 附近），
       利用驱动器自身的减速斜坡让实际车速平滑减速，恰好在钩子过原点时车速归零！
     - Q3、Q4 同理对称。
  2. 内部斜坡速度/位移估计器 (v_hat, x_hat)：
     实时模拟 PLC 驱动器的加减速斜坡，跟踪当前估计车速 v_hat 与累积漂移 x_hat，
     叠加微弱柔顺回中项 (-k_recenter · x_hat)，防止连续几个周期消摆后大/小车慢慢偏离原位，
     且全程无需依赖高频易跳变的 SLAM 位置差分。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from anti_sway.base import AntiSwayStepContext, VelocityCorrection

MODE_ID = 'quad_settle'
MODE_LABEL = '四象限超前消摆 (倾角短平快)'
MODE_DESCRIPTION = (
    '四象限加减速消摆 + 驱动速度斜坡超前补偿 (v*=Kv·θ, a*=Kv·θ̇, v_cmd=v*+τ·a*)，'
    '专为纯速度指令有加速滞后的现场行车设计'
)


@dataclass
class AxisQuadDebug:
    """单轴四象限诊断状态。"""

    quadrant: int = 0          # 0=死区静止, 1=前摆加速, 2=前摆回落减速, 3=后摆加速, 4=后摆回落减速
    theta_ac_rad: float = 0.0  # 扣除慢变零偏后的前向摆角 [rad]
    omega_rad_s: float = 0.0   # 滤波后摆角速度 [rad/s]
    dc_bias_rad: float = 0.0   # 在线跟踪的残余静态偏置 [rad]
    v_star: float = 0.0        # 理想目标车速 [m/s]
    a_star: float = 0.0        # 理想目标加速度 [m/s²]
    v_cmd: float = 0.0         # 经斜坡超前补偿后下发的速度指令 [m/s]
    v_hat: float = 0.0         # 估计的实际车速 [m/s]
    x_hat: float = 0.0         # 估计的累积位移漂移 [m]


def classify_sway_quadrant(
    theta: float,
    omega: float,
    *,
    deadband_rad: float,
    omega_deadband_rad_s: float,
) -> int:
    """判定当前处于四象限中的哪一阶段。

    Returns:
        0: 静止死区内
        1: 0 → +max (向前摆出，需向前加速: v>0, a>0)
        2: +max → 0 (向前最高点回落，需向前减速: v>0, a<0)
        3: 0 → -max (向后摆出，需向后加速: v<0, a<0)
        4: -max → 0 (向后最高点回落，需向后减速: v<0, a>0)
    """
    if abs(theta) < deadband_rad and abs(omega) < omega_deadband_rad_s:
        return 0
    if theta >= 0.0:
        return 1 if omega >= 0.0 else 2
    return 3 if omega <= 0.0 else 4


QUADRANT_LABELS = {
    0: 'IDLE(稳)',
    1: 'Q1(前摆→前加速)',
    2: 'Q2(前落→前减速)',
    3: 'Q3(后摆→后加速)',
    4: 'Q4(后落→后减速)',
}


class _AxisQuadLeadController:
    """单轴四象限 + 速度斜坡逆滞后控制器。"""

    def __init__(
        self,
        *,
        kv: float = 2.2,
        tau_lead: float = 0.45,
        max_vel: float = 0.30,
        drive_accel: float = 0.20,
        deadband_rad: float = math.radians(0.10),
        omega_tau: float = 0.06,
        dc_tau: float = 5.0,
        track_gain: float = 0.35,
        recenter_gain: float = 0.12,
        max_drift_m: float = 0.45,
        axis_sign: float = -1.0,
    ):
        self.kv = float(kv)
        self.tau_lead = max(0.0, float(tau_lead))
        self.max_vel = max(0.0, float(max_vel))
        self.drive_accel = max(0.02, float(drive_accel))
        self.deadband_rad = max(0.0, float(deadband_rad))
        self.omega_tau = max(0.0, float(omega_tau))
        self.dc_tau = float(dc_tau)
        self.track_gain = max(0.0, float(track_gain))
        self.recenter_gain = max(0.0, float(recenter_gain))
        self.max_drift_m = max(0.05, float(max_drift_m))
        # axis_sign 将外部输入的 theta 转为“与 +v_cmd 同向的前向摆角”:
        # - 真机现场日志 (localization_negate_xy=True, baseline v-=K·θ 实测衰减) 对应 sign = -1.0
        # - 仿真 SimSwayFeedback (alpha = -g/L·θ - a/L, v+=K·θ 阻尼) 对应 sign = +1.0
        self.axis_sign = 1.0 if float(axis_sign) >= 0.0 else -1.0
        self.reset()

    def reset(self) -> None:
        self._prev_theta_ac: float | None = None
        self._omega: float = 0.0
        self._dc_bias: float = 0.0
        self._v_hat: float = 0.0
        self._x_hat: float = 0.0
        self.last_debug = AxisQuadDebug()

    def step(self, theta_raw: float, dt: float) -> float:
        dt = max(1e-4, min(0.5, float(dt)))
        # 统一符号：theta_fwd > 0 表示钩子向 +v_cmd 方向（前方）摆出
        theta_fwd = self.axis_sign * float(theta_raw)

        # 慢变直流偏置跟踪（消除挂钩轻微不对称或残余静态角，避免持续往一边跑）
        if self.dc_tau > 0.0:
            alpha_dc = dt / (self.dc_tau + dt)
            self._dc_bias += alpha_dc * (theta_fwd - self._dc_bias)
        theta_ac = theta_fwd - self._dc_bias

        # 摆角速度估计 + 一阶低通滤波（抑制 J1939 倾角仪量化毛刺）
        if self._prev_theta_ac is None:
            raw_omega = 0.0
        else:
            raw_omega = (theta_ac - self._prev_theta_ac) / dt
        self._prev_theta_ac = theta_ac

        if self.omega_tau > 0.0:
            alpha_w = dt / (self.omega_tau + dt)
            self._omega += alpha_w * (raw_omega - self._omega)
        else:
            self._omega = raw_omega

        omega_db = max(math.radians(0.18), self.deadband_rad * 1.6)
        quad = classify_sway_quadrant(
            theta_ac,
            self._omega,
            deadband_rad=self.deadband_rad,
            omega_deadband_rad_s=omega_db,
        )

        if quad == 0:
            v_star = 0.0
            a_star = 0.0
        else:
            # 四象限期望车速与期望加速度：
            # Q1 (θ>0, ω>0): v*>0, a*>0 (向前加速追钩)
            # Q2 (θ>0, ω<0): v*>0, a*<0 (向前减速收车，回原点时恰好停住)
            # Q3 (θ<0, ω<0): v*<0, a*<0 (向后加速追钩)
            # Q4 (θ<0, ω>0): v*<0, a*>0 (向后减速收车，回原点时恰好停住)
            v_star = max(-self.max_vel, min(self.max_vel, self.kv * theta_ac))
            a_star = self.kv * self._omega

            # 相位保护：在回落减速段 (Q2 / Q4)，超前量 τ_lead·a* 允许把指令提前拉到 0
            # 甚至轻微反向制动（当估计车速 v_hat 仍较大时），但若实际车速已接近 0，
            # 不允许提前反向猛冲（否则会在钩子还没回落到原点时就往反方向加速）。
            pass

        v_recenter = -self.recenter_gain * self._x_hat
        v_lead = v_star + self.tau_lead * a_star + self.track_gain * (v_star - self._v_hat)

        # Q2 / Q4 回落减速段的防过早反向门控：
        # 钩子从最高点回落到原点途中，目标是“向前减速 (v>0, a<0)”或“向后减速 (v<0, a>0)”，
        # 也就是让实际车速从峰值平滑减到 0，而不是在回落半途就把车速反过来加速！
        # 如果内部估计车速 |v_hat| 已经很小，限制反向预制动幅度不超过平缓过零所需。
        if quad == 2 and v_lead < 0.0:
            # 允许利用适度反向指令给仍在前进的车“踩刹车”，但刹车下限与当前估计前向车速成正比
            min_brake_cmd = -0.6 * max(0.0, self._v_hat)
            v_lead = max(min_brake_cmd, v_lead)
        elif quad == 4 and v_lead > 0.0:
            max_brake_cmd = -0.6 * min(0.0, self._v_hat)
            v_lead = min(max_brake_cmd, v_lead)

        v_cmd = max(-self.max_vel, min(self.max_vel, v_lead + v_recenter))
        if quad == 0 and abs(self._x_hat) < 0.02 and abs(v_cmd) < 0.008:
            v_cmd = 0.0

        # 更新内部驱动斜坡模型：既有一阶惯性也有最大加速度斜坡限幅
        tau_plant = max(0.12, self.tau_lead)
        a_est = (v_cmd - self._v_hat) / tau_plant
        a_est = max(-self.drive_accel, min(self.drive_accel, a_est))
        self._v_hat = max(-self.max_vel, min(self.max_vel, self._v_hat + a_est * dt))
        self._x_hat = max(
            -self.max_drift_m,
            min(self.max_drift_m, self._x_hat + self._v_hat * dt),
        )

        self.last_debug = AxisQuadDebug(
            quadrant=quad,
            theta_ac_rad=theta_ac,
            omega_rad_s=self._omega,
            dc_bias_rad=self._dc_bias,
            v_star=v_star,
            a_star=a_star,
            v_cmd=v_cmd,
            v_hat=self._v_hat,
            x_hat=self._x_hat,
        )
        return v_cmd


class QuadLeadAntiSwayController:
    """双轴 (X大车 / Y小车) 四象限超前消摆控制器。"""

    def __init__(
        self,
        *,
        kv: float = 2.2,
        tau_lead: float = 0.45,
        max_vel: float = 0.30,
        drive_accel: float = 0.20,
        deadband_rad: float = math.radians(0.10),
        omega_tau: float = 0.06,
        dc_tau: float = 5.0,
        track_gain: float = 0.35,
        recenter_gain: float = 0.12,
        sign_x: float = -1.0,
        sign_y: float = -1.0,
    ):
        if kv < 0 or max_vel < 0:
            raise ValueError('kv and max_vel must be non-negative')
        self.kv = float(kv)
        self.tau_lead = float(tau_lead)
        self.max_vel = float(max_vel)
        self._x = _AxisQuadLeadController(
            kv=kv,
            tau_lead=tau_lead,
            max_vel=max_vel,
            drive_accel=drive_accel,
            deadband_rad=deadband_rad,
            omega_tau=omega_tau,
            dc_tau=dc_tau,
            track_gain=track_gain,
            recenter_gain=recenter_gain,
            axis_sign=sign_x,
        )
        self._y = _AxisQuadLeadController(
            kv=kv,
            tau_lead=tau_lead,
            max_vel=max_vel,
            drive_accel=drive_accel,
            deadband_rad=deadband_rad,
            omega_tau=omega_tau,
            dc_tau=dc_tau,
            track_gain=track_gain,
            recenter_gain=recenter_gain,
            axis_sign=sign_y,
        )

    def reset(self) -> None:
        self._x.reset()
        self._y.reset()

    def compute(self, theta_x: float, theta_y: float, dt: float) -> tuple[float, float]:
        vx = self._x.step(theta_x, dt)
        vy = self._y.step(theta_y, dt)
        return vx, vy

    def debug_snapshot(self) -> dict[str, Any]:
        dx = self._x.last_debug
        dy = self._y.last_debug
        return {
            'mode': MODE_ID,
            'quad_x': dx.quadrant,
            'quad_y': dy.quadrant,
            'quad_x_label': QUADRANT_LABELS.get(dx.quadrant, ''),
            'quad_y_label': QUADRANT_LABELS.get(dy.quadrant, ''),
            'theta_ac_x_deg': math.degrees(dx.theta_ac_rad),
            'theta_ac_y_deg': math.degrees(dy.theta_ac_rad),
            'omega_x_deg_s': math.degrees(dx.omega_rad_s),
            'omega_y_deg_s': math.degrees(dy.omega_rad_s),
            'v_star_x': dx.v_star,
            'v_star_y': dy.v_star,
            'a_star_x': dx.a_star,
            'a_star_y': dy.a_star,
            'v_hat_x': dx.v_hat,
            'v_hat_y': dy.v_hat,
            'x_hat': dx.x_hat,
            'y_hat': dy.x_hat,
            'dv_x': dx.v_cmd,
            'dv_y': dy.v_cmd,
        }


class QuadSettleStrategy:
    """接入统一 run_pd_control(profile='settle') 的策略适配器。"""

    mode_id = MODE_ID

    def __init__(self, controller: QuadLeadAntiSwayController, config):
        self._ctrl = controller
        self._config = config
        self._sim_polarity_applied = False

    def reset(self) -> None:
        self._ctrl.reset()

    def log_startup(self, *, verbose: bool) -> None:
        if not verbose:
            return
        print(
            f'[四象限超前消摆] Kv={self._ctrl.kv}, τ_lead={self._ctrl.tau_lead}s, '
            f'max={self._ctrl.max_vel} m/s '
            f'(sign_x={self._ctrl._x.axis_sign:+.0f}, sign_y={self._ctrl._y.axis_sign:+.0f})'
        )

    def correct_xy(
        self,
        vx_cmd: float,
        vy_cmd: float,
        ctx: AntiSwayStepContext,
    ) -> VelocityCorrection:
        # 仿真环境 SimSwayFeedback 的物理极性为 v += K·θ (sign=+1)，
        # 而现场真机 (localization_negate_xy=True) 实测 baseline v -= K·θ 为正阻尼 (sign=-1)。
        # 通过采样步长自动区分 100Hz 仿真 (dt≈0.01) 与真机，或尊重显式配置。
        if not self._sim_polarity_applied and ctx.dt <= 0.025:
            auto_sim = bool(getattr(ctx.config, 'anti_sway_quad_auto_sim_sign', True))
            if auto_sim:
                self._ctrl._x.axis_sign = 1.0
                self._ctrl._y.axis_sign = 1.0
            self._sim_polarity_applied = True
        else:
            self._sim_polarity_applied = True

        dvx, dvy = self._ctrl.compute(ctx.theta_x, ctx.theta_y, ctx.dt)
        # 在 settle 模式下，如果外部 PD 仍在工作，弱化外部 PD 对四象限速度曲线的干扰，
        # 只保留其柔和防跑偏作用，主导权交给四象限超前消摆指令。
        vx_out = 0.35 * vx_cmd + dvx
        vy_out = 0.35 * vy_cmd + dvy
        return VelocityCorrection(
            vx_cmd=vx_out,
            vy_cmd=vy_out,
            dvx=vx_out - vx_cmd,
            dvy=vy_out - vy_cmd,
        )

    def debug_snapshot(self) -> dict[str, Any]:
        return self._ctrl.debug_snapshot()


def config_fields_from_site(site: dict[str, Any] | None) -> dict[str, Any]:
    site = site or {}
    return {
        'anti_sway_quad_kv': float(site.get('anti_sway_quad_kv', 2.2)),
        'anti_sway_quad_tau_lead': float(site.get('anti_sway_quad_tau_lead', 0.45)),
        'anti_sway_quad_max_correction': float(
            site.get('anti_sway_quad_max_correction', 0.30)
        ),
        'anti_sway_quad_drive_accel': float(
            site.get('anti_sway_quad_drive_accel', 0.20)
        ),
        'anti_sway_quad_deadband_deg': float(
            site.get('anti_sway_quad_deadband_deg', 0.10)
        ),
        'anti_sway_quad_omega_tau': float(site.get('anti_sway_quad_omega_tau', 0.06)),
        'anti_sway_quad_dc_tau': float(site.get('anti_sway_quad_dc_tau', 5.0)),
        'anti_sway_quad_recenter_gain': float(
            site.get('anti_sway_quad_recenter_gain', 0.12)
        ),
        'anti_sway_quad_sign_x': float(site.get('anti_sway_quad_sign_x', -1.0)),
        'anti_sway_quad_sign_y': float(site.get('anti_sway_quad_sign_y', -1.0)),
    }


def build_strategy(config) -> QuadSettleStrategy | None:
    if not getattr(config, 'enable_anti_sway', False):
        return None
    if getattr(config, 'anti_sway_mode', 'off') != MODE_ID:
        return None
    kv = float(getattr(config, 'anti_sway_quad_kv', 2.2))
    if kv <= 0.0:
        return None
    ctrl = QuadLeadAntiSwayController(
        kv=kv,
        tau_lead=float(getattr(config, 'anti_sway_quad_tau_lead', 0.45)),
        max_vel=float(getattr(config, 'anti_sway_quad_max_correction', 0.30)),
        drive_accel=float(getattr(config, 'anti_sway_quad_drive_accel', 0.20)),
        deadband_rad=math.radians(
            float(getattr(config, 'anti_sway_quad_deadband_deg', 0.10))
        ),
        omega_tau=float(getattr(config, 'anti_sway_quad_omega_tau', 0.06)),
        dc_tau=float(getattr(config, 'anti_sway_quad_dc_tau', 5.0)),
        recenter_gain=float(getattr(config, 'anti_sway_quad_recenter_gain', 0.12)),
        sign_x=float(getattr(config, 'anti_sway_quad_sign_x', -1.0)),
        sign_y=float(getattr(config, 'anti_sway_quad_sign_y', -1.0)),
    )
    return QuadSettleStrategy(ctrl, config)
