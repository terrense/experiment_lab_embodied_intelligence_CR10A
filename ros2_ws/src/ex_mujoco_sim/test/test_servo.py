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
    it = CommandInterpolator(1, max_horizon=0.02, vel_filter_tau=0.0)
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


def test_velocity_uses_sender_stamps_under_arrival_jitter():
    """10 ms command stream at 1 rad/s whose ARRIVAL times jitter 4-16 ms (as measured over ROS):
    with sender stamps the velocity estimate stays at 1 rad/s; with arrival times it would not."""
    rng = np.random.default_rng(0)
    with_stamps = CommandInterpolator(1, vel_filter_tau=0.0)
    arrival_only = CommandInterpolator(1, vel_filter_tau=0.0)
    t_arr, est_s, est_a = 0.0, [], []
    for k in range(200):
        t_src = 0.01 * k
        t_arr = max(t_arr + 0.001, t_src + rng.uniform(0.0, 0.006))
        q = [1.0 * t_src]
        with_stamps.update(t_arr, q, t_src=t_src)
        arrival_only.update(t_arr, q)
        if k > 2:
            est_s.append(with_stamps.v[0])
            est_a.append(arrival_only.v[0])
    assert np.allclose(est_s, 1.0, atol=1e-9)
    assert max(abs(np.array(est_a) - 1.0)) > 0.3  # the bug this guards against


def test_velocity_filter_smooths_steps():
    it = CommandInterpolator(1, vel_filter_tau=0.02)
    for k in range(3):
        it.update(0.01 * k, [0.0], t_src=0.01 * k)
    it.update(0.03, [0.01], t_src=0.03)  # sudden 1 rad/s
    assert 0.0 < it.v[0] < 1.0
