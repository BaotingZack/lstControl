"""Shared types for pluggable anti-sway strategies."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class AntiSwayStepContext:
    """Per-cycle inputs for velocity correction (duck-typed config)."""

    theta_x: float
    theta_y: float
    dt: float
    z_grab: float
    dist_xy: float
    locked_x: bool
    locked_y: bool
    config: Any
    # True=仿真对象（一阶伺服 τ 已知）, False=真实 PLC 回路
    is_simulation: bool = True


@dataclass(frozen=True)
class VelocityCorrection:
    vx_cmd: float
    vy_cmd: float
    dvx: float
    dvy: float


class AntiSwayStrategy(Protocol):
    mode_id: str

    def log_startup(self, *, verbose: bool) -> None: ...

    def correct_xy(
        self,
        vx_cmd: float,
        vy_cmd: float,
        ctx: AntiSwayStepContext,
    ) -> VelocityCorrection: ...
