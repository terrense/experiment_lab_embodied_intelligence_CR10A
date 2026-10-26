import math

import numpy as np
import pytest

from ex_robot_state.core.mock_motion import DEFAULT_FREQS_HZ, MockMotion, MockMotionConfig
from ex_robot_state.core.signal_checks import dominant_frequency_index, frequency_amplitudes

HOME = [0, 0, math.radians(90), 0, math.radians(-90), 0]


@pytest.fixture
def motion(cr10_model):
    return MockMotion(cr10_model.actuated_joints("tcp"), MockMotionConfig(home=HOME))


def test_starts_at_home_with_zero_velocity(motion):
    assert np.allclose(motion.positions(0.0), HOME)
    assert np.allclose(motion.velocities(0.0), 0.0, atol=1e-3)


def test_stays_within_limits_with_margin(motion, cr10_model):
    t = np.arange(0, 300, 0.05)
    Q = np.array([motion.positions(x) for x in t])
    for k, j in enumerate(cr10_model.actuated_joints("tcp")):
        assert Q[:, k].min() >= j.lower + motion.cfg.limit_margin - 1e-12
        assert Q[:, k].max() <= j.upper - motion.cfg.limit_margin + 1e-12


def test_amplitude_clipped_near_limit(cr10_model):
    joints = cr10_model.actuated_joints("tcp")
    home = list(HOME)
    home[2] = joints[2].upper - math.radians(15)  # 5 deg of room after the 10 deg margin
    m = MockMotion(joints, MockMotionConfig(home=home))
    assert m.amplitudes[2] == pytest.approx(math.radians(5))


def test_home_outside_limits_rejected(cr10_model):
    home = list(HOME)
    home[2] = 3.0
    with pytest.raises(ValueError):
        MockMotion(cr10_model.actuated_joints("tcp"), MockMotionConfig(home=home))


def test_frequencies_identify_each_joint(motion):
    """The mapping check used by pipeline_check: 20 s window is enough."""
    t = np.arange(0, 20, 0.008)
    Q = np.array([motion.positions(x) for x in t])
    assert [dominant_frequency_index(t, Q[:, k], DEFAULT_FREQS_HZ) for k in range(6)] == list(range(6))
    # swapped joints are detected
    Q[:, [1, 2]] = Q[:, [2, 1]]
    assert [dominant_frequency_index(t, Q[:, k], DEFAULT_FREQS_HZ) for k in range(6)] != list(range(6))


def test_frequency_amplitude_recovery():
    t = np.arange(0, 15, 0.01)
    y = 0.3 + 0.5 * np.sin(2 * np.pi * 0.10 * t) + 0.2 * np.cos(2 * np.pi * 0.13 * t + 0.4)
    a = frequency_amplitudes(t, y, DEFAULT_FREQS_HZ)
    assert a[0] == pytest.approx(0.5, abs=1e-6) and a[1] == pytest.approx(0.2, abs=1e-6)
    assert max(a[2:]) < 1e-6
