"""Overhead LQR angle damping for settle (anchor hold).

Reference: ``防摇方案/OverheadCraneSwayControl-main`` (``Crane_Sway_Control.m``).

PTP ``overhead`` skips correction on locked axes; settle applies LQR damping for full
``profile=settle`` loop at fixed anchor.
"""

from __future__ import annotations

import math
from typing import Any

from anti_sway.base import AntiSwayStepContext, VelocityCorrection
from anti_sway.overhead import (
    OverheadLqrController,
    config_fields_from_site as overhead_fields,
)
from anti_sway.rope import RopeLengthModel

MODE_ID = 'overhead_settle'
MODE_LABEL = 'Overhead 消摆 — OverheadCraneSwayControl'
MODE_DESCRIPTION = (
    'v = v_PD − (1/M)·(Kθ·θ+Kω·θ̇)，绳长 LQR 全程消摆；'
    '参考 防摇方案/OverheadCraneSwayControl-main'
)


class OverheadSettleStrategy:
    mode_id = MODE_ID

    def __init__(self, controller: OverheadLqrController, config):
        self._ctrl = controller
        self._config = config

    def log_startup(self, *, verbose: bool) -> None:
        if not verbose:
            return
        cfg = self._config
        print(
            f'[Overhead 消摆] v_xy − (1/M)·(Kθ·θ+Kω·θ̇), '
            f'M={cfg.anti_sway_overhead_trolley_mass} kg, '
            f'm={cfg.anti_sway_overhead_load_mass} kg, '
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
        vx_cmd -= u_x
        vy_cmd -= u_y
        return VelocityCorrection(
            vx_cmd=vx_cmd,
            vy_cmd=vy_cmd,
            dvx=-u_x,
            dvy=-u_y,
        )


def config_fields_from_site(site: dict[str, Any] | None) -> dict[str, Any]:
    return overhead_fields(site)


def _build_controller(config) -> OverheadLqrController:
    rope = RopeLengthModel(
        sheave_height=getattr(config, 'rope_length_sheave_height', 4.0),
        grab_offset=getattr(config, 'rope_length_grab_offset', 0.0),
        cable_stretch=getattr(config, 'rope_length_cable_stretch', 0.0),
        min_length=getattr(config, 'rope_length_min', 0.5),
    )
    ftv = getattr(config, 'anti_sway_overhead_force_to_vel', None)
    return OverheadLqrController(
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


def build_strategy(config) -> OverheadSettleStrategy | None:
    if not getattr(config, 'enable_anti_sway', False):
        return None
    if getattr(config, 'anti_sway_mode', 'incl') != MODE_ID:
        return None
    return OverheadSettleStrategy(_build_controller(config), config)
