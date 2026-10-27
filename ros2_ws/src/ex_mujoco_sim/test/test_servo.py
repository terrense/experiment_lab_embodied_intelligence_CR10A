import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ex_mujoco_sim.servo import CommandInterpolator, gains_from_inertia, servo_torque  # noqa: E402


def test_gains_formula():
    g = gains_from_inertia([2.0], 10.0, 1.0)
    w = 2 * math.pi * 10
    assert g.kp[0] == pytest.approx(2 * w * w) and g.kd[0] == pytest.approx(2 * 2 * w)


def test_interpolator_extrapolates_and_caps():
    it = CommandInterpolator(1, max_horizon=0.02)
    it.update(0.00, [0.0])
    it.update(0.01, [0.1])  # 10 rad/s
    q, v = it.reference(0.015)
    assert q[0] == pytest.approx(0.15) and v[0] == pytest.approx(10.0)
    q, v = it.reference(0.5)  # stale command: hold last + capped extrapolation, zero velocity
    assert q[0] == pytest.approx(0.1 + 10.0 * 0.02) and v[0] == 0.0


def test_torque_saturates():
    g = gains_from_inertia([1.0], 10.0, 1.0)
    tau = servo_torque(np.zeros(1), np.zeros(1), [10.0], [0.0], np.zeros(1), g, [50.0])
    assert tau[0] == 50.0


def test_closed_loop_unit_inertia_tracks_ramp():
    """Discrete-time double integrator (M=1) with the servo: bandwidth 10 Hz, 1 kHz loop,
    100 Hz set-points of a ramp -> steady-state tracking error small thanks to feedforward."""
    g = gains_from_inertia([1.0], 10.0, 0.9)
    it = CommandInterpolator(1)
    q, dq, dt = 0.0, 0.0, 0.001
    errs = []
    for k in range(3000):
        t = k * dt
        if k % 10 == 0:
            it.update(t, [0.5 * t])  # 0.5 rad/s ramp
        q_ref, dq_ref = it.reference(t)
        tau = servo_torque(np.array([q]), np.array([dq]), q_ref, dq_ref, np.zeros(1), g, [1e9])[0]
        dq += tau * dt
        q += dq * dt
        if t > 1.0:
            errs.append(abs(0.5 * t - q))
    assert max(errs) < 2e-3
