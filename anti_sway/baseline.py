"""Baseline settle damping: v_cmd -= clip(K·θ) on each axis (benchmark for 消摆测试)."""

from __future__ import annotations

import math
from typing import Any

from anti_sway.base import AntiSwayStepContext, VelocityCorrection

MODE_ID = 'baseline'
MODE_LABEL = '消摆基准 K·θ'
MODE_DESCRIPTION = 'v = −K·θ 全程（消摆测试默认基准，不依赖到位段）'


class BaselineDampingController:
    def __init__(
        self,
        *,
        gain: float = 0.85,
        max_correction: float = 0.15,
        deadband_rad: float = math.radians(0.15),
    ):
        if max_correction < 0:
            raise ValueError('max_correction must be non-negative')
        self.gain = gain
        self.max_correction = max_correction
        self.deadband_rad = max(0.0, deadband_rad)

    def reset(self) -> None:
        return

    def _axis(self, theta: float) -> float:
        if abs(theta) < self.deadband_rad:
            return 0.0
        u = self.gain * theta
        return max(-self.max_correction, min(self.max_correction, u))

    def compute(self, theta_x: float, theta_y: float) -> tuple[float, float]:
        return self._axis(theta_x), self._axis(theta_y)


class BaselineSettleStrategy:
    mode_id = MODE_ID

    def __init__(self, controller: BaselineDampingController, config):
        self._ctrl = controller
        self._config = config

    def log_startup(self, *, verbose: bool) -> None:
        if not verbose:
            return
        cfg = self._config
        print(
            f'[消摆基准] v_xy -= clip(K·θ), K={cfg.anti_sway_baseline_gain}, '
            f'max={cfg.anti_sway_baseline_max_correction} m/s'
        )

    def correct_xy(
        self,
        vx_cmd: float,
        vy_cmd: float,
        ctx: AntiSwayStepContext,
    ) -> VelocityCorrection:
        u_x, u_y = self._ctrl.compute(ctx.theta_x, ctx.theta_y)
        dvx = dvy = 0.0
        # 消摆模式：即使轴已“到位锁定”也继续施加阻尼
        vx_cmd -= u_x
        vy_cmd -= u_y
        dvx = -u_x
        dvy = -u_y
        return VelocityCorrection(vx_cmd=vx_cmd, vy_cmd=vy_cmd, dvx=dvx, dvy=dvy)


def config_fields_from_site(site: dict[str, Any] | None) -> dict[str, Any]:
    site = site or {}
    return {
        'anti_sway_baseline_gain': float(site.get('anti_sway_baseline_gain', 2.6)),
        'anti_sway_baseline_max_correction': float(
            site.get('anti_sway_baseline_max_correction', 0.35)
        ),
        'anti_sway_baseline_deadband_deg': float(
            site.get('anti_sway_baseline_deadband_deg', 0.08)
        ),
    }


def build_strategy(config) -> BaselineSettleStrategy | None:
    if not getattr(config, 'enable_anti_sway', False):
        return None
    if getattr(config, 'anti_sway_mode', 'incl') != MODE_ID:
        return None
    gain = float(getattr(config, 'anti_sway_baseline_gain', 0.0))
    if gain <= 0:
        return None
    ctrl = BaselineDampingController(
        gain=gain,
        max_correction=float(
            getattr(config, 'anti_sway_baseline_max_correction', 0.35)
        ),
        deadband_rad=math.radians(
            float(getattr(config, 'anti_sway_baseline_deadband_deg', 0.08))
        ),
    )
    return BaselineSettleStrategy(ctrl, config)
