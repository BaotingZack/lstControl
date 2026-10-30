"""Overhead LQR anti-sway (OverheadCraneSwayControl reference)."""

from __future__ import annotations

from anti_sway.overhead import (
    MODE_ID,
    OverheadLqrController,
    build_overhead_anti_sway,
    compute_overhead_lqr_gains,
)
from anti_sway.ptp_config import apply_ptp_anti_sway_mode
from anti_sway.registry import list_algorithm_options
from anti_sway.rope import RopeLengthModel
from crane_model import CraneConfig


def test_lqr_matches_reference_order_of_magnitude():
    k_theta, k_omega = compute_overhead_lqr_gains(
        load_mass=5,
        trolley_mass=250,
        rope_length=3.0,
    )
    assert k_theta > 1000
    assert k_omega > 1000


def test_overhead_controller_subtracts_on_positive_theta():
    rope = RopeLengthModel(sheave_height=4.0, min_length=0.5)
    ctrl = OverheadLqrController(
        rope,
        load_mass=5,
        trolley_mass=250,
        max_correction=1.0,
        deadband_rad=0.0,
        omega_filter_tau=0.0,
    )
    u_x, _ = ctrl.compute(0.05, 0.0, 0.1, z_grab=1.0)
    assert u_x > 0


def test_build_overhead_requires_mode():
    cfg = CraneConfig(enable_anti_sway=True, anti_sway_mode='incl')
    assert build_overhead_anti_sway(cfg) is None
    cfg2 = apply_ptp_anti_sway_mode(
        CraneConfig(),
        'overhead',
        {'anti_sway_overhead_trolley_mass': 200.0},
    )
    assert cfg2.anti_sway_mode == 'overhead'
    assert build_overhead_anti_sway(cfg2) is not None


def test_registry_lists_overhead():
    ids = [o['id'] for o in list_algorithm_options()]
    assert MODE_ID in ids


def test_gains_update_when_rope_length_changes():
    rope = RopeLengthModel(sheave_height=8.0, min_length=0.5)
    ctrl = OverheadLqrController(rope, deadband_rad=0.0)
    ctrl.compute(0.0, 0.0, 0.1, z_grab=1.0)
    k1 = (ctrl._k_theta, ctrl._k_omega)
    ctrl.compute(0.0, 0.0, 0.1, z_grab=6.0)
    k2 = (ctrl._k_theta, ctrl._k_omega)
    assert k1 != k2
