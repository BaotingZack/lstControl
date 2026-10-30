"""Parallel angle PD subtracted from position PD (gantry-crane-anti-sway)."""

from __future__ import annotations

import math
from typing import Any

from anti_sway.base import AntiSwayStepContext, VelocityCorrection

MODE_ID = 'gantry'
MODE_LABEL = '防摇 Gantry PD — gantry-crane-anti-sway-control'
MODE_DESCRIPTION = 'v = v_PD − (Kp·θ + Kd·θ̇)，全程；参考 防摇方案/gantry-crane-anti-sway-control-main'


class GantryAntiSwayController:
    def __init__(
        self,
        *,
        kp: float = 0.9,
        kd: float = 0.45,
        max_correction: float = 0.2,
        deadband_rad: float = math.radians(1.0),
        omega_filter_tau: float = 0.03,
    ):
        if max_correction < 0:
            raise ValueError('max_correction must be non-negative')
        self.kp = kp
        self.kd = kd
        self.max_correction = max_correction
        self.deadband_rad = max(0.0, deadband_rad)
        self.omega_filter_tau = max(0.0, omega_filter_tau)
        self._prev_theta_x: float | None = None
        self._prev_theta_y: float | None = None
        self._omega_x = 0.0
        self._omega_y = 0.0

    def reset(self) -> None:
        self._prev_theta_x = self._prev_theta_y = None
        self._omega_x = self._omega_y = 0.0

    def _deadband(self, theta: float) -> float:
        if abs(theta) < self.deadband_rad:
            return 0.0
        return theta

    def _axis_pd(
        self, theta: float, prev: float | None, omega_state: float, dt: float,
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
        u = self.kp * theta + self.kd * omega
        u = max(-self.max_correction, min(self.max_correction, u))
        return u, omega

    def compute(self, theta_x: float, theta_y: float, dt: float) -> tuple[float, float]:
        dt = max(1e-4, dt)
        u_x, self._omega_x = self._axis_pd(
            theta_x, self._prev_theta_x, self._omega_x, dt,
        )
        u_y, self._omega_y = self._axis_pd(
            theta_y, self._prev_theta_y, self._omega_y, dt,
        )
        self._prev_theta_x = theta_x
        self._prev_theta_y = theta_y
        return u_x, u_y


class GantryAntiSwayStrategy:
    mode_id = MODE_ID

    def __init__(self, controller: GantryAntiSwayController, config):
        self._ctrl = controller
        self._config = config

    def log_startup(self, *, verbose: bool) -> None:
        if not verbose:
            return
        print(
            f'[防摇 Gantry] v_pd − (Kp·θ+Kd·θ̇), '
            f'Kp={self._config.anti_sway_gantry_kp}, '
            f'Kd={self._config.anti_sway_gantry_kd}, '
            f'max={self._config.anti_sway_gantry_max_correction} m/s'
        )

    def correct_xy(
        self,
        vx_cmd: float,
        vy_cmd: float,
        ctx: AntiSwayStepContext,
    ) -> VelocityCorrection:
        u_x, u_y = self._ctrl.compute(ctx.theta_x, ctx.theta_y, ctx.dt)
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
        'anti_sway_gantry_kp': float(site.get('anti_sway_gantry_kp', 0.9)),
        'anti_sway_gantry_kd': float(site.get('anti_sway_gantry_kd', 0.45)),
        'anti_sway_gantry_deadband_deg': float(
            site.get('anti_sway_gantry_deadband_deg', 1.0)
        ),
        'anti_sway_gantry_max_correction': float(
            site.get('anti_sway_gantry_max_correction', 0.2)
        ),
        'anti_sway_gantry_omega_tau': float(
            site.get('anti_sway_gantry_omega_tau', 0.03)
        ),
    }


def build_strategy(config) -> GantryAntiSwayStrategy | None:
    if not getattr(config, 'enable_anti_sway', False):
        return None
    if getattr(config, 'anti_sway_mode', 'incl') != MODE_ID:
        return None
    kp = float(getattr(config, 'anti_sway_gantry_kp', 0.0))
    kd = float(getattr(config, 'anti_sway_gantry_kd', 0.0))
    if kp == 0.0 and kd == 0.0:
        return None
    deadband_deg = float(getattr(config, 'anti_sway_gantry_deadband_deg', 1.0))
    ctrl = GantryAntiSwayController(
        kp=kp,
        kd=kd,
        max_correction=float(getattr(config, 'anti_sway_gantry_max_correction', 0.2)),
        deadband_rad=math.radians(deadband_deg),
        omega_filter_tau=float(getattr(config, 'anti_sway_gantry_omega_tau', 0.03)),
    )
    return GantryAntiSwayStrategy(ctrl, config)


def build_gantry_anti_sway(config) -> GantryAntiSwayController | None:
    strat = build_strategy(config)
    return strat._ctrl if strat else None
