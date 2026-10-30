"""K·θ approach damping (dev_incl / energy damping form)."""

from __future__ import annotations

from typing import Any

from anti_sway.base import AntiSwayStepContext, AntiSwayStrategy, VelocityCorrection
from anti_sway.rope import RopeLengthModel

MODE_ID = 'incl'
MODE_LABEL = '防摇 K·θ — CraneRemote / dev_incl'
MODE_DESCRIPTION = 'v = v_PD + K(L)·θ，默认接近目标段；参考 防摇方案/CraneRemote-master 比例纠偏 + 现场 dev_incl'


class AntiSwayController:
    """Δv = +K(L)·θ with optional gain schedule vs rope length."""

    def __init__(
        self,
        rope_model: RopeLengthModel,
        sway_gain: float = 0.0,
        gain_schedule: tuple = (),
        max_correction: float = 0.05,
    ):
        if max_correction < 0:
            raise ValueError('max_correction must be non-negative')
        self.rope_model = rope_model
        self._gain_fixed = sway_gain
        self.max_correction = max_correction
        self._schedule = tuple(sorted(gain_schedule, key=lambda g: g[0]))
        for a, b in zip(self._schedule, self._schedule[1:]):
            if b[0] <= a[0]:
                raise ValueError('gain_schedule rope lengths must be strictly increasing')

    def _gain(self, L: float) -> float:
        if not self._schedule:
            return self._gain_fixed
        if L <= self._schedule[0][0]:
            return self._schedule[0][1]
        if L >= self._schedule[-1][0]:
            return self._schedule[-1][1]
        for (l0, g0), (l1, g1) in zip(self._schedule, self._schedule[1:]):
            if l0 <= L <= l1:
                f = (L - l0) / (l1 - l0)
                return g0 + f * (g1 - g0)
        return self._gain_fixed

    def compute(
        self,
        theta_x: float,
        theta_y: float,
        z_grab: float,
    ) -> tuple[float, float, float]:
        L = self.rope_model.compute(z_grab)
        k = self._gain(L)
        dv_x = max(-self.max_correction, min(self.max_correction, k * theta_x))
        dv_y = max(-self.max_correction, min(self.max_correction, k * theta_y))
        return dv_x, dv_y, L


class InclAntiSwayStrategy:
    mode_id = MODE_ID

    def __init__(self, controller: AntiSwayController, config):
        self._ctrl = controller
        self._config = config

    def log_startup(self, *, verbose: bool) -> None:
        if not verbose:
            return
        cfg = self._config
        print(
            f'[防摇 K·θ] K={cfg.anti_sway_sway_gain}, '
            f'max_dv={cfg.anti_sway_max_correction}, '
            f'approach={cfg.anti_sway_approach_pos}m'
        )

    def correct_xy(
        self,
        vx_cmd: float,
        vy_cmd: float,
        ctx: AntiSwayStepContext,
    ) -> VelocityCorrection:
        cfg = ctx.config
        active = cfg.anti_sway_travel_damping or (
            cfg.anti_sway_approach_damping
            and ctx.dist_xy <= cfg.anti_sway_approach_pos
        ) or (ctx.locked_x and ctx.locked_y)
        dvx = dvy = 0.0
        if active:
            dvx, dvy, _L = self._ctrl.compute(ctx.theta_x, ctx.theta_y, ctx.z_grab)
        if ctx.locked_x:
            dvx = 0.0
        else:
            vx_cmd += dvx
        if ctx.locked_y:
            dvy = 0.0
        else:
            vy_cmd += dvy
        return VelocityCorrection(vx_cmd=vx_cmd, vy_cmd=vy_cmd, dvx=dvx, dvy=dvy)


def config_fields_from_site(site: dict[str, Any] | None) -> dict[str, Any]:
    return {}


def build_strategy(config) -> InclAntiSwayStrategy | None:
    if not getattr(config, 'enable_anti_sway', False):
        return None
    if getattr(config, 'anti_sway_mode', 'incl') != MODE_ID:
        return None
    if not getattr(config, 'anti_sway_sway_gain', 0.0):
        return None
    rope = RopeLengthModel(
        sheave_height=getattr(config, 'rope_length_sheave_height', 4.0),
        grab_offset=getattr(config, 'rope_length_grab_offset', 0.0),
        cable_stretch=getattr(config, 'rope_length_cable_stretch', 0.0),
        min_length=getattr(config, 'rope_length_min', 0.5),
    )
    ctrl = AntiSwayController(
        rope,
        sway_gain=config.anti_sway_sway_gain,
        gain_schedule=tuple(getattr(config, 'anti_sway_gain_schedule', ()) or ()),
        max_correction=getattr(config, 'anti_sway_max_correction', 0.05),
    )
    return InclAntiSwayStrategy(ctrl, config)


def build_anti_sway(config) -> AntiSwayController | None:
    """Legacy factory returning raw controller (tests / old imports)."""
    strat = build_strategy(config)
    return strat._ctrl if strat else None
