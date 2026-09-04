import pytest

from input_shaper import (
    shape_command,
    zv_impulses,
    zvd_impulses,
    ei_impulses,
)


def test_zv_zero_damping():
    amps, times = zv_impulses(T=2.0, zeta=0.0)
    assert amps == pytest.approx([0.5, 0.5])
    assert times == pytest.approx([0.0, 1.0])  # T/2


def test_zvd_zero_damping():
    amps, times = zvd_impulses(T=2.0, zeta=0.0)
    assert amps == pytest.approx([0.25, 0.5, 0.25])
    assert times == pytest.approx([0.0, 1.0, 2.0])  # 0, T/2, T


def test_impulse_amplitudes_sum_to_one():
    for gen in (zv_impulses, zvd_impulses, ei_impulses):
        amps, _ = gen(T=3.0, zeta=0.0)
        assert sum(amps) == pytest.approx(1.0)


def test_shape_command_convolution():
    # 单位冲激命令经 ZV 整形后应为 [0.5, 0.5, 0, 0]
    values = [1.0, 0.0, 0.0, 0.0]
    amps, times = zv_impulses(T=2.0, zeta=0.0)
    shaped = shape_command(values, dt=1.0, amplitudes=amps, times=times)
    assert shaped == pytest.approx([0.5, 0.5, 0.0, 0.0])


def test_shape_command_rejects_bad_args():
    with pytest.raises(ValueError):
        shape_command([1.0], dt=0.0, amplitudes=[0.5, 0.5], times=[0.0, 1.0])
    with pytest.raises(ValueError):
        shape_command([1.0], dt=1.0, amplitudes=[0.5], times=[0.0, 1.0])


def test_zv_rejects_negative_period():
    with pytest.raises(ValueError):
        zv_impulses(T=-1.0)


def test_ei_reduces_to_zvd_at_zero_tolerance():
    assert ei_impulses(T=2.0, zeta=0.0, v_tol=0.0) == zvd_impulses(T=2.0, zeta=0.0)


def test_ei_differs_from_zvd_with_tolerance():
    amps, times = ei_impulses(T=2.0, zeta=0.0, v_tol=0.05)
    zvd_amps, _ = zvd_impulses(T=2.0, zeta=0.0)
    assert amps != zvd_amps
    assert amps == pytest.approx([1.05 / 4, 0.95 / 2, 1.05 / 4])
    assert times == pytest.approx([0.0, 1.0, 2.0])


def test_ei_rejects_bad_tolerance():
    with pytest.raises(ValueError):
        ei_impulses(T=2.0, zeta=0.0, v_tol=1.0)
