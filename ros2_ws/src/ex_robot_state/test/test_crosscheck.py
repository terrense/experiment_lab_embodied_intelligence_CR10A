import math

import numpy as np
import pytest

from ex_robot_state.core.crosscheck import CrosscheckStats
from ex_robot_state.core.se3 import make_transform, matrix_to_quat, matrix_to_rpy, rpy_to_matrix


def _ctrl(T):
    """Controller-style report of T using extrinsic-XYZ Euler + (w,x,y,z) quaternion."""
    r, p, y = matrix_to_rpy(T[:3, :3])
    x, yq, z, w = matrix_to_quat(T[:3, :3])
    return [*(T[:3, 3] * 1000), *(math.degrees(v) for v in (r, p, y))], [w, x, yq, z]


def test_identical_pose_zero_error_and_best_hypothesis():
    st = CrosscheckStats()
    rng = np.random.default_rng(3)
    for _ in range(20):
        T = make_transform(rpy_to_matrix(*rng.uniform(-1.4, 1.4, 3)), rng.uniform(-0.8, 0.8, 3))
        st.add(T, *_ctrl(T))
    s = st.summary()
    rot = s["rot_err_deg_mean_by_hypothesis"]
    assert s["pos_err_mm_max"] < 1e-6
    assert rot["euler_xyz_extrinsic(Rz*Ry*Rx)"] < 1e-6 and rot["quat_wxyz"] < 1e-6
    assert rot["quat_xyzw"] > 1.0 and rot["euler_xyz_intrinsic(Rx*Ry*Rz)"] > 1.0
    assert s["best_orientation_hypothesis"] in ("euler_xyz_extrinsic(Rz*Ry*Rx)", "quat_wxyz")


def test_position_offset_measured():
    st = CrosscheckStats()
    T = make_transform(np.eye(3), np.array([0.5, 0.0, 0.3]))
    tv, q = _ctrl(T)
    tv[0] += 3.0  # controller says 3 mm further in x
    assert st.add(T, tv, q).pos_err_mm == pytest.approx(3.0)


def test_zero_quaternion_ignored():
    st = CrosscheckStats()
    s = st.add(np.eye(4), [0, 0, 0, 0, 0, 0], [0, 0, 0, 0])
    assert set(s.rot_err_deg) == {"euler_xyz_extrinsic(Rz*Ry*Rx)", "euler_xyz_intrinsic(Rx*Ry*Rz)"}


def test_split_raw_layout():
    from ex_robot_state.core.crosscheck import split_raw
    data = [1, 2, 3, 4, 5, 6, 0.1, 0.2, 0.3, 0.4, 10, 20, 30, 40, 50, 60, 1.7e12, 0, 2]
    r = split_raw(data)
    assert r["tool_vector"] == [1, 2, 3, 4, 5, 6] and r["quat"] == [0.1, 0.2, 0.3, 0.4]
    assert r["q_deg"] == [10, 20, 30, 40, 50, 60] and r["ctrl_ts_ms"] == 1.7e12
    assert (r["tool_id"], r["user_id"]) == (0, 2)
    assert split_raw(data[:10]) is None
