"""CraneRemote-inspired settle damping (vision P + zero-crossing catch steps).

Reference: ``防摇方案/CraneRemote-master``

- ``CraneRemote.py``: proportional drive from hook offset (here: θ from inclinometer).
- ``simulation.m``: at hook relative-velocity zero-crossings, nudge trolley by ``dxstep``
  until swing amplitude shrinks; adapted here as a short velocity pulse on filtered θ̇ zero-cross.
"""

from __future__ import annotations

import math
from typing import Any

from anti_sway.base import AntiSwayStepContext, VelocityCorrection

MODE_ID = 'crane_remote'
MODE_LABEL = 'CraneRemote 消摆'
MODE_DESCRIPTION = 'K·θ 纠偏 + θ̇ 过零步进 (CraneRemote / simulation.m)'


class CraneRemoteSettleController:
    def __init__(
        self,
        *,
        kp: float = 2.6,
        max_correction: float = 0.2,
        deadband_rad: float = math.radians(0.8),
        omega_tau: float = 0.03,
        zc_min_rad: float = math.radians(0.4),
        zc_step_velocity: float = 0.06,
        zc_release_ratio: float = 0.88,
    ):
        if max_correction < 0 or zc_step_velocity < 0:
            raise ValueError('velocity limits must be non-negative')
        self.kp = kp
        self.max_correction = max_correction
        self.deadband_rad = max(0.0, deadband_rad)
        self.omega_tau = max(0.0, omega_tau)
        self.zc_min_rad = max(0.0, zc_min_rad)
        self.zc_step_velocity = zc_step_velocity
        self.zc_release_ratio = min(1.0, max(0.1, zc_release_ratio))
        self._prev_theta_x: float | None = None
        self._prev_theta_y: float | None = None
        self._omega_x = 0.0
        self._omega_y = 0.0
        self._prev_omega_x: float | None = None
        self._prev_omega_y: float | None = None
        self._zc_dir_x = 0
        self._zc_dir_y = 0
        self._zc_arm_theta_x: float | None = None
        self._zc_arm_theta_y: float | None = None

    def reset(self) -> None:
        self._prev_theta_x = self._prev_theta_y = None
        self._omega_x = self._omega_y = 0.0
        self._prev_omega_x = self._prev_omega_y = None
        self._zc_dir_x = self._zc_dir_y = 0
        self._zc_arm_theta_x = self._zc_arm_theta_y = None

    def _filtered_omega(
        self, theta: float, prev_theta: float | None, omega_state: float, dt: float,
    ) -> tuple[float, float]:
        if prev_theta is None or dt <= 0:
            return 0.0, omega_state
        raw = (theta - prev_theta) / dt
        if self.omega_tau > 0:
            alpha = dt / (self.omega_tau + dt)
            omega = omega_state + alpha * (raw - omega_state)
        else:
            omega = raw
        return omega, omega

    def _p_term(self, theta: float) -> float:
        if abs(theta) < self.deadband_rad:
            return 0.0
        u = self.kp * theta
        return max(-self.max_correction, min(self.max_correction, u))

    def _zc_term(
        self,
        theta: float,
        omega: float,
        prev_omega: float | None,
        zc_dir: int,
        arm_theta: float | None,
    ) -> tuple[float, int, float | None]:
        """Return (velocity bump, new zc_dir, new arm_theta)."""
        bump = 0.0
        if zc_dir != 0:
            bump = zc_dir * self.zc_step_velocity
            if arm_theta is not None and abs(theta) <= abs(arm_theta) * self.zc_release_ratio:
                zc_dir = 0
                arm_theta = None
            elif abs(theta) < self.deadband_rad:
                zc_dir = 0
                arm_theta = None
        if prev_omega is not None and prev_omega * omega <= 0.0:
            if abs(theta) >= self.zc_min_rad:
                zc_dir = 1 if theta > 0 else -1
                arm_theta = theta
        return bump, zc_dir, arm_theta

    def _axis(
        self,
        theta: float,
        prev_theta: float | None,
        omega_state: float,
        prev_omega: float | None,
        zc_dir: int,
        arm_theta: float | None,
        dt: float,
    ) -> tuple[float, float, int, float | None]:
        omega, omega_state = self._filtered_omega(theta, prev_theta, omega_state, dt)
        u_p = self._p_term(theta)
        u_zc, zc_dir, arm_theta = self._zc_term(theta, omega, prev_omega, zc_dir, arm_theta)
        u = max(-self.max_correction, min(self.max_correction, u_p + u_zc))
        return u, omega_state, zc_dir, arm_theta

    def compute(self, theta_x: float, theta_y: float, dt: float) -> tuple[float, float]:
        dt = max(1e-4, dt)
        u_x, self._omega_x, self._zc_dir_x, self._zc_arm_theta_x = self._axis(
            theta_x,
            self._prev_theta_x,
            self._omega_x,
            self._prev_omega_x,
            self._zc_dir_x,
            self._zc_arm_theta_x,
            dt,
        )
        u_y, self._omega_y, self._zc_dir_y, self._zc_arm_theta_y = self._axis(
            theta_y,
            self._prev_theta_y,
            self._omega_y,
            self._prev_omega_y,
            self._zc_dir_y,
            self._zc_arm_theta_y,
            dt,
        )
        self._prev_omega_x, self._prev_omega_y = self._omega_x, self._omega_y
        self._prev_theta_x, self._prev_theta_y = theta_x, theta_y
        return u_x, u_y


class CraneRemoteSettleStrategy:
    mode_id = MODE_ID

    def __init__(self, controller: CraneRemoteSettleController, config):
        self._ctrl = controller
        self._config = config

    def log_startup(self, *, verbose: bool) -> None:
        if not verbose:
            return
        cfg = self._config
        print(
            f'[CraneRemote 消摆] v_xy += clip(K·θ + zc), K={cfg.anti_sway_crane_remote_kp}, '
            f'max={cfg.anti_sway_crane_remote_max_correction} m/s, '
            f'zc_step={cfg.anti_sway_crane_remote_zc_step_velocity} m/s'
        )

    def correct_xy(
        self,
        vx_cmd: float,
        vy_cmd: float,
        ctx: AntiSwayStepContext,
    ) -> VelocityCorrection:
        u_x, u_y = self._ctrl.compute(ctx.theta_x, ctx.theta_y, ctx.dt)
        vx_cmd += u_x
        vy_cmd += u_y
        return VelocityCorrection(vx_cmd=vx_cmd, vy_cmd=vy_cmd, dvx=u_x, dvy=u_y)


def config_fields_from_site(site: dict[str, Any] | None) -> dict[str, Any]:
    site = site or {}
    return {
        'anti_sway_crane_remote_kp': float(
            site.get('anti_sway_crane_remote_kp', site.get('anti_sway_sway_gain', 2.6))
        ),
        'anti_sway_crane_remote_max_correction': float(
            site.get(
                'anti_sway_crane_remote_max_correction',
                site.get('anti_sway_max_correction', 0.2),
            )
        ),
        'anti_sway_crane_remote_deadband_deg': float(
            site.get('anti_sway_crane_remote_deadband_deg', 0.8)
        ),
        'anti_sway_crane_remote_omega_tau': float(
            site.get('anti_sway_crane_remote_omega_tau', 0.03)
        ),
        'anti_sway_crane_remote_zc_min_deg': float(
            site.get('anti_sway_crane_remote_zc_min_deg', 0.4)
        ),
        'anti_sway_crane_remote_zc_step_velocity': float(
            site.get('anti_sway_crane_remote_zc_step_velocity', 0.06)
        ),
        'anti_sway_crane_remote_zc_release_ratio': float(
            site.get('anti_sway_crane_remote_zc_release_ratio', 0.88)
        ),
    }


def build_strategy(config) -> CraneRemoteSettleStrategy | None:
    if not getattr(config, 'enable_anti_sway', False):
        return None
    if getattr(config, 'anti_sway_mode', 'incl') != MODE_ID:
        return None
    kp = float(getattr(config, 'anti_sway_crane_remote_kp', 0.0))
    if kp <= 0:
        return None
    ctrl = CraneRemoteSettleController(
        kp=kp,
        max_correction=float(
            getattr(config, 'anti_sway_crane_remote_max_correction', 0.2)
        ),
        deadband_rad=math.radians(
            float(getattr(config, 'anti_sway_crane_remote_deadband_deg', 0.8))
        ),
        omega_tau=float(getattr(config, 'anti_sway_crane_remote_omega_tau', 0.03)),
        zc_min_rad=math.radians(
            float(getattr(config, 'anti_sway_crane_remote_zc_min_deg', 0.4))
        ),
        zc_step_velocity=float(
            getattr(config, 'anti_sway_crane_remote_zc_step_velocity', 0.06)
        ),
        zc_release_ratio=float(
            getattr(config, 'anti_sway_crane_remote_zc_release_ratio', 0.88)
        ),
    )
    return CraneRemoteSettleStrategy(ctrl, config)
