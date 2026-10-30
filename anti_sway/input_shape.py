"""Open-loop ZVD input shaping on PD velocity commands (PTP only, no inclinometer)."""

from __future__ import annotations

import math
from typing import Any

from anti_sway.base import AntiSwayStepContext, VelocityCorrection
from anti_sway.rope import RopeLengthModel
from anti_sway.shaping import omega_from_rope_length, zvd_step_gain

MODE_ID = 'input_shape'
MODE_LABEL = '防摇 输入整形 ZVD — open-loop'
MODE_DESCRIPTION = (
    '起动段对 PD 速度乘以 ZVD 阶跃包络 (ωₙ=√(g/L))，开环、不读倾角仪；'
    '约 2×半周期后恢复满增益'
)


class InputShapeStrategy:
    mode_id = MODE_ID
    requires_sway_feedback = False

    def __init__(
        self,
        *,
        rope: RopeLengthModel,
        zeta: float,
        config,
    ):
        self._rope = rope
        self._zeta = zeta
        self._config = config
        self._omega: float | None = None
        self._move_t = 0.0
        self._active = False

    def log_startup(self, *, verbose: bool) -> None:
        if not verbose:
            return
        print(
            f'[防摇 ZVD] 起动包络增益, ζ={self._zeta:g}, '
            f'绳长 sheave={self._config.rope_length_sheave_height}m '
            f'(不依赖倾角仪)'
        )

    def _omega_n(self, z_grab: float) -> float:
        if self._omega is None:
            L = self._rope.compute(z_grab)
            self._omega = omega_from_rope_length(L)
        return self._omega

    def correct_xy(
        self,
        vx_cmd: float,
        vy_cmd: float,
        ctx: AntiSwayStepContext,
    ) -> VelocityCorrection:
        raw_vx, raw_vy = vx_cmd, vy_cmd
        if ctx.locked_x and ctx.locked_y:
            self._active = False
            self._move_t = 0.0
            return VelocityCorrection(
                vx_cmd=vx_cmd, vy_cmd=vy_cmd, dvx=0.0, dvy=0.0,
            )

        mag = math.hypot(vx_cmd, vy_cmd)
        if not self._active:
            if mag > 0.02:
                self._active = True
                self._move_t = 0.0
        if self._active:
            self._move_t += ctx.dt
            gain = zvd_step_gain(
                self._move_t,
                omega_n=self._omega_n(ctx.z_grab),
                zeta=self._zeta,
            )
            if not ctx.locked_x:
                vx_cmd *= gain
            if not ctx.locked_y:
                vy_cmd *= gain

        return VelocityCorrection(
            vx_cmd=vx_cmd,
            vy_cmd=vy_cmd,
            dvx=vx_cmd - raw_vx,
            dvy=vy_cmd - raw_vy,
        )


def config_fields_from_site(site: dict[str, Any] | None) -> dict[str, Any]:
    site = site or {}
    return {
        'anti_sway_travel_damping': True,
        'anti_sway_approach_damping': False,
        'anti_sway_input_shape_zeta': float(
            site.get('anti_sway_input_shape_zeta', 0.01),
        ),
    }


def build_strategy(config) -> InputShapeStrategy | None:
    if not getattr(config, 'enable_anti_sway', False):
        return None
    if getattr(config, 'anti_sway_mode', 'off') != MODE_ID:
        return None
    zeta = float(getattr(config, 'anti_sway_input_shape_zeta', 0.01))
    rope = RopeLengthModel(
        sheave_height=getattr(config, 'rope_length_sheave_height', 4.0),
        grab_offset=getattr(config, 'rope_length_grab_offset', 0.0),
        cable_stretch=getattr(config, 'rope_length_cable_stretch', 0.0),
        min_length=getattr(config, 'rope_length_min', 0.5),
    )
    return InputShapeStrategy(rope=rope, zeta=zeta, config=config)
