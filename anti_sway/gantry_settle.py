"""Gantry parallel angle PD for settle (anchor hold + damping).

Reference: ``防摇方案/gantry-crane-anti-sway-control-main`` (xPID ∥ thetaPID → combine).

PTP mode ``gantry`` skips correction on locked axes; settle keeps damping at anchor
even when position loop treats axes as settled (``profile=settle``).
"""

from __future__ import annotations

import math
from typing import Any

from anti_sway.base import AntiSwayStepContext, VelocityCorrection
from anti_sway.gantry import GantryAntiSwayController, config_fields_from_site as gantry_fields

MODE_ID = 'gantry_settle'
MODE_LABEL = 'Gantry 消摆 — gantry-crane-anti-sway-control'
MODE_DESCRIPTION = (
    'v = v_PD − (Kp·θ + Kd·θ̇) 全程消摆；'
    '参考 防摇方案/gantry-crane-anti-sway-control-main'
)


class GantrySettleStrategy:
    mode_id = MODE_ID

    def __init__(self, controller: GantryAntiSwayController, config):
        self._ctrl = controller
        self._config = config

    def log_startup(self, *, verbose: bool) -> None:
        if not verbose:
            return
        cfg = self._config
        print(
            f'[Gantry 消摆] v_xy − clip(Kp·θ+Kd·θ̇), '
            f'Kp={cfg.anti_sway_gantry_kp}, Kd={cfg.anti_sway_gantry_kd}, '
            f'max={cfg.anti_sway_gantry_max_correction} m/s'
        )

    def correct_xy(
        self,
        vx_cmd: float,
        vy_cmd: float,
        ctx: AntiSwayStepContext,
    ) -> VelocityCorrection:
        u_x, u_y = self._ctrl.compute(ctx.theta_x, ctx.theta_y, ctx.dt)
        vx_cmd -= u_x
        vy_cmd -= u_y
        return VelocityCorrection(
            vx_cmd=vx_cmd,
            vy_cmd=vy_cmd,
            dvx=-u_x,
            dvy=-u_y,
        )


def config_fields_from_site(site: dict[str, Any] | None) -> dict[str, Any]:
    return gantry_fields(site)


def build_strategy(config) -> GantrySettleStrategy | None:
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
    return GantrySettleStrategy(ctrl, config)
