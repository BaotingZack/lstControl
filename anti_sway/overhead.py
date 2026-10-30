"""Overhead crane pendulum LQR (OverheadCraneSwayControl reference).

Linearized cart-pendulum (MATLAB ``Crane_Sway_Control.m``):

  x = [trolley position, velocity, rope angle, angular rate]
  u = actuator force [N]  →  mapped to Δv via 1/M for velocity-command cranes.

Position PD handles tracking; this layer applies angle-state LQR as v_pd − scale·(Kθ·θ+Kω·θ̇).
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from anti_sway.base import AntiSwayStepContext, VelocityCorrection
from anti_sway.lqr_math import lqr_gain
from anti_sway.rope import RopeLengthModel

MODE_ID = 'overhead'
MODE_LABEL = '防摇 Overhead LQR — OverheadCraneSwayControl'
MODE_DESCRIPTION = (
    '单摆状态 LQR，绳长随 Z 更新 Kθ/Kω，全程；'
    '参考 防摇方案/OverheadCraneSwayControl-main (Crane_Sway_Control.m)'
)

GRAVITY = 9.81


def _pendulum_ab(
    *,
    load_mass: float,
    trolley_mass: float,
    rope_length: float,
    gravity: float = GRAVITY,
) -> tuple[np.ndarray, np.ndarray]:
    m = max(1e-3, float(load_mass))
    M = max(1e-3, float(trolley_mass))
    l = max(0.25, float(rope_length))
    g = gravity
    A = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -m * g / M, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, -(m + M) * g / (M * l), 0.0],
        ],
        dtype=float,
    )
    B = np.array([[0.0], [1.0 / M], [0.0], [1.0 / (M * l)]], dtype=float)
    return A, B


def compute_overhead_lqr_gains(
    *,
    load_mass: float,
    trolley_mass: float,
    rope_length: float,
    q_pos: float = 1.0,
    q_vel: float = 1.0,
    q_theta: float = 1.0e7,
    q_omega: float = 1.0e7,
    r_force: float = 1.0,
    gravity: float = GRAVITY,
) -> tuple[float, float]:
    """Return (K_theta, K_omega) from continuous LQR on the 4-state plant."""
    A, B = _pendulum_ab(
        load_mass=load_mass,
        trolley_mass=trolley_mass,
        rope_length=rope_length,
        gravity=gravity,
    )
    Q = np.diag([q_pos, q_vel, q_theta, q_omega])
    R = np.array([[max(1e-9, r_force)]], dtype=float)
    K = lqr_gain(A, B, Q, R)
    return float(K[2]), float(K[3])


class OverheadLqrController:
    """Angle-state LQR mapped to velocity corrections per axis."""

    def __init__(
        self,
        rope_model: RopeLengthModel,
        *,
        load_mass: float = 5.0,
        trolley_mass: float = 250.0,
        q_pos: float = 1.0,
        q_vel: float = 1.0,
        q_theta: float = 1.0e7,
        q_omega: float = 1.0e7,
        r_force: float = 1.0,
        force_to_velocity: float | None = None,
        max_correction: float = 0.2,
        deadband_rad: float = math.radians(1.0),
        omega_filter_tau: float = 0.03,
        rope_length_epsilon: float = 0.05,
    ):
        if max_correction < 0:
            raise ValueError('max_correction must be non-negative')
        self.rope_model = rope_model
        self.load_mass = load_mass
        self.trolley_mass = max(1e-3, trolley_mass)
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_theta = q_theta
        self.q_omega = q_omega
        self.r_force = r_force
        self.force_to_velocity = (
            force_to_velocity
            if force_to_velocity is not None
            else 1.0 / self.trolley_mass
        )
        self.max_correction = max_correction
        self.deadband_rad = max(0.0, deadband_rad)
        self.omega_filter_tau = max(0.0, omega_filter_tau)
        self.rope_length_epsilon = max(1e-4, rope_length_epsilon)

        self._prev_theta_x: float | None = None
        self._prev_theta_y: float | None = None
        self._omega_x = 0.0
        self._omega_y = 0.0
        self._k_theta = 0.0
        self._k_omega = 0.0
        self._last_rope_l: float | None = None

    def reset(self) -> None:
        self._prev_theta_x = self._prev_theta_y = None
        self._omega_x = self._omega_y = 0.0
        self._last_rope_l = None

    def _deadband(self, theta: float) -> float:
        if abs(theta) < self.deadband_rad:
            return 0.0
        return theta

    def _update_gains_for_rope(self, z_grab: float) -> None:
        L = self.rope_model.compute(z_grab)
        if (
            self._last_rope_l is not None
            and abs(L - self._last_rope_l) < self.rope_length_epsilon
        ):
            return
        k_theta, k_omega = compute_overhead_lqr_gains(
            load_mass=self.load_mass,
            trolley_mass=self.trolley_mass,
            rope_length=L,
            q_pos=self.q_pos,
            q_vel=self.q_vel,
            q_theta=self.q_theta,
            q_omega=self.q_omega,
            r_force=self.r_force,
        )
        self._k_theta = k_theta
        self._k_omega = k_omega
        self._last_rope_l = L

    def _axis_lqr(
        self,
        theta: float,
        prev: float | None,
        omega_state: float,
        dt: float,
    ) -> tuple[float, float]:
        theta = self._deadband(theta)
        if prev is None or dt <= 0:
            omega = 0.0
        else:
            raw = (theta - prev) / dt
            if self.omega_filter_tau > 0:
                alpha = dt / (self.omega_filter_tau + dt)
                omega = omega_state + alpha * (raw - omega_state)
            else:
                omega = raw
        force = self._k_theta * theta + self._k_omega * omega
        u = self.force_to_velocity * force
        u = max(-self.max_correction, min(self.max_correction, u))
        return u, omega

    def compute(
        self,
        theta_x: float,
        theta_y: float,
        dt: float,
        z_grab: float,
    ) -> tuple[float, float]:
        dt = max(1e-4, dt)
        self._update_gains_for_rope(z_grab)
        u_x, self._omega_x = self._axis_lqr(
            theta_x, self._prev_theta_x, self._omega_x, dt,
        )
        u_y, self._omega_y = self._axis_lqr(
            theta_y, self._prev_theta_y, self._omega_y, dt,
        )
        self._prev_theta_x = theta_x
        self._prev_theta_y = theta_y
        return u_x, u_y


class OverheadLqrStrategy:
    mode_id = MODE_ID

    def __init__(self, controller: OverheadLqrController, config):
        self._ctrl = controller
        self._config = config

    def log_startup(self, *, verbose: bool) -> None:
        if not verbose:
            return
        cfg = self._config
        print(
            f'[防摇 Overhead LQR] v_pd − (1/M)·(Kθ·θ+Kω·θ̇), '
            f'M={cfg.anti_sway_overhead_trolley_mass} kg, '
            f'm={cfg.anti_sway_overhead_load_mass} kg, '
            f'Qθ={cfg.anti_sway_overhead_q_theta:g}, '
            f'max={cfg.anti_sway_overhead_max_correction} m/s'
        )

    def correct_xy(
        self,
        vx_cmd: float,
        vy_cmd: float,
        ctx: AntiSwayStepContext,
    ) -> VelocityCorrection:
        u_x, u_y = self._ctrl.compute(
            ctx.theta_x, ctx.theta_y, ctx.dt, ctx.z_grab,
        )
        dvx = dvy = 0.0
        if not ctx.locked_x:
            vx_cmd -= u_x
            dvx = -u_x
        if not ctx.locked_y:
            vy_cmd -= u_y
            dvy = -u_y
        return VelocityCorrection(vx_cmd=vx_cmd, vy_cmd=vy_cmd, dvx=dvx, dvy=dvy)


def config_fields_from_site(site: dict[str, Any] | None) -> dict[str, Any]:
    site = site or {}
    return {
        'anti_sway_overhead_load_mass': float(
            site.get('anti_sway_overhead_load_mass', 5.0)
        ),
        'anti_sway_overhead_trolley_mass': float(
            site.get('anti_sway_overhead_trolley_mass', 250.0)
        ),
        'anti_sway_overhead_q_pos': float(site.get('anti_sway_overhead_q_pos', 1.0)),
        'anti_sway_overhead_q_vel': float(site.get('anti_sway_overhead_q_vel', 1.0)),
        'anti_sway_overhead_q_theta': float(
            site.get('anti_sway_overhead_q_theta', 1.0e7)
        ),
        'anti_sway_overhead_q_omega': float(
            site.get('anti_sway_overhead_q_omega', 1.0e7)
        ),
        'anti_sway_overhead_r_force': float(
            site.get('anti_sway_overhead_r_force', 1.0)
        ),
        'anti_sway_overhead_force_to_vel': site.get('anti_sway_overhead_force_to_vel'),
        'anti_sway_overhead_deadband_deg': float(
            site.get('anti_sway_overhead_deadband_deg', 1.0)
        ),
        'anti_sway_overhead_max_correction': float(
            site.get('anti_sway_overhead_max_correction', 0.2)
        ),
        'anti_sway_overhead_omega_tau': float(
            site.get('anti_sway_overhead_omega_tau', 0.03)
        ),
    }


def build_strategy(config) -> OverheadLqrStrategy | None:
    if not getattr(config, 'enable_anti_sway', False):
        return None
    if getattr(config, 'anti_sway_mode', 'incl') != MODE_ID:
        return None
    rope = RopeLengthModel(
        sheave_height=getattr(config, 'rope_length_sheave_height', 4.0),
        grab_offset=getattr(config, 'rope_length_grab_offset', 0.0),
        cable_stretch=getattr(config, 'rope_length_cable_stretch', 0.0),
        min_length=getattr(config, 'rope_length_min', 0.5),
    )
    ftv = getattr(config, 'anti_sway_overhead_force_to_vel', None)
    ctrl = OverheadLqrController(
        rope,
        load_mass=float(getattr(config, 'anti_sway_overhead_load_mass', 5.0)),
        trolley_mass=float(getattr(config, 'anti_sway_overhead_trolley_mass', 250.0)),
        q_pos=float(getattr(config, 'anti_sway_overhead_q_pos', 1.0)),
        q_vel=float(getattr(config, 'anti_sway_overhead_q_vel', 1.0)),
        q_theta=float(getattr(config, 'anti_sway_overhead_q_theta', 1.0e7)),
        q_omega=float(getattr(config, 'anti_sway_overhead_q_omega', 1.0e7)),
        r_force=float(getattr(config, 'anti_sway_overhead_r_force', 1.0)),
        force_to_velocity=float(ftv) if ftv is not None else None,
        max_correction=float(
            getattr(config, 'anti_sway_overhead_max_correction', 0.2)
        ),
        deadband_rad=math.radians(
            float(getattr(config, 'anti_sway_overhead_deadband_deg', 1.0))
        ),
        omega_filter_tau=float(
            getattr(config, 'anti_sway_overhead_omega_tau', 0.03)
        ),
    )
    return OverheadLqrStrategy(ctrl, config)


def build_overhead_anti_sway(config) -> OverheadLqrController | None:
    strat = build_strategy(config)
    return strat._ctrl if strat else None
