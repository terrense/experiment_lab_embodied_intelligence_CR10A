import math

import numpy as np
import pytest

from ex_robot_state.core import se3


@pytest.mark.parametrize("rpy", [(0, 0, 0), (0.3, -0.7, 2.1), (-3.0, 1.2, -0.4), (1.5707, 0.2, 0.1)])
def test_rpy_roundtrip(rpy):
    R = se3.rpy_to_matrix(*rpy)
    assert np.allclose(R @ R.T, np.eye(3))
    assert np.allclose(se3.rpy_to_matrix(*se3.matrix_to_rpy(R)), R, atol=1e-9)


def test_rpy_is_fixed_axis_xyz():
    # URDF spec: roll about X first, then pitch about fixed Y, then yaw about fixed Z
    R = se3.rpy_to_matrix(0.1, 0.2, 0.3)
    assert np.allclose(R, se3.rot_z(0.3) @ se3.rot_y(0.2) @ se3.rot_x(0.1))


def test_quat_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        R = se3.rpy_to_matrix(*rng.uniform(-3, 3, 3))
        q = se3.matrix_to_quat(R)
        assert q[3] >= 0
        assert np.allclose(se3.quat_to_matrix(*q), R, atol=1e-9)


def test_invert_and_angle():
    T = se3.make_transform(se3.rpy_to_matrix(0.4, -0.2, 1.0), np.array([1.0, -2.0, 0.5]))
    assert np.allclose(T @ se3.invert(T), np.eye(4))
    assert se3.rotation_angle_between(np.eye(3), se3.rot_z(0.5)) == pytest.approx(0.5)


def test_axis_angle_matches_rot_z():
    assert np.allclose(se3.axis_angle_to_matrix(np.array([0, 0, 1.0]), 0.7), se3.rot_z(0.7))
