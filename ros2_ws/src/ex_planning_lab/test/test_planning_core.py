import math
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG.parent / "ex_robot_state"))

from ex_planning_lab.core.capsules import (Capsule, capsule_box_distance, fit_capsules, point_aabb_distance,  # noqa: E402
                                           point_segment_distance, read_stl_vertices)
from ex_planning_lab.core.metrics import (cartesian_path_length, densify, joint_path_length, min_clearance,  # noqa: E402
                                          summarize, tracking_error)
from ex_planning_lab.core.scene import Box, Scene, sample_in_tcp_region  # noqa: E402

MESHES = PKG.parent / "ex_description" / "meshes" / "cr10_official"


def _write_binary_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<3f", 0, 0, 0) + b"".join(struct.pack("<3f", *v) for v in t) + b"\0\0")


def test_stl_reader_binary(tmp_path):
    p = tmp_path / "t.stl"
    _write_binary_stl(p, [[(0, 0, 0), (1, 0, 0), (0, 1, 0)], [(0, 0, 0), (0, 1, 0), (0, 0, 2)]])
    v = read_stl_vertices(p)
    assert len(v) == 4 and np.allclose(v.max(axis=0), [1, 1, 2])


def test_real_cr10_meshes_parse():
    if not MESHES.exists():
        pytest.skip("meshes not vendored")
    for f in MESHES.glob("*.STL"):
        v = read_stl_vertices(f)
        assert len(v) > 100 and np.all(np.abs(v) < 1.0), f.name  # metres, sane extents


def test_distances():
    a, b = np.zeros(3), np.array([1.0, 0, 0])
    d = point_segment_distance(np.array([[0.5, 2, 0], [-1, 0, 0], [3, 0, 4]]), a, b)
    assert np.allclose(d, [2, 1, math.hypot(2, 4)])
    d = point_aabb_distance(np.array([[0, 0, 0], [2, 0, 0], [2, 2, 0]]), np.array([-1, -1, -1.0]), np.array([1, 1, 1.0]))
    assert np.allclose(d, [0, 1, math.sqrt(2)])


def test_fitted_capsule_contains_mesh(tmp_path):
    p = tmp_path / "m.stl"
    _write_binary_stl(p, [[(0, 0.3, 0), (0.5, 0, 0.1), (1.2, 0, 0)]])
    urdf = f'<robot name="r"><link name="L"><collision><geometry><mesh filename="{p}"/></geometry></collision></link></robot>'
    caps = fit_capsules(urdf, lambda u: Path(u), ["L"], {"L": np.array([1.0, 0, 0])})
    assert caps[0].radius <= 0.3 + 1e-12  # never looser than the bone fit (max(0.3, 0.1, 0.2))
    v = read_stl_vertices(p)
    assert np.all(point_segment_distance(v, caps[0].a, caps[0].b) <= caps[0].radius + 1e-9)


def test_pca_capsule_for_offset_mesh():
    from ex_planning_lab.core.capsules import tightest_enclosing_capsule
    rng = np.random.default_rng(0)
    # a 0.6 m long rod of radius 0.05 offset 0.2 m sideways from the kinematic bone
    t = rng.uniform(0, 0.6, 500)
    ang = rng.uniform(0, 2 * np.pi, 500)
    v = np.stack([t, 0.2 + 0.05 * np.cos(ang), 0.05 * np.sin(ang)], axis=1)
    a, b, r = tightest_enclosing_capsule(v, (np.zeros(3), np.array([0.6, 0, 0])))
    assert r < 0.06  # bone-based fit would need ~0.25
    assert np.all(point_segment_distance(v, a, b) <= r + 1e-9)


def test_capsule_box_clearance():
    cap = Capsule("L", np.zeros(3), np.array([1.0, 0, 0]), 0.1)
    T = np.eye(4)
    T[:3, 3] = [0, 0, 1.0]  # bone at height 1 m
    lo, hi = np.array([-1, -1, 0.0]), np.array([2, 1, 0.5])  # box top at 0.5
    assert capsule_box_distance(T, cap, lo, hi) == pytest.approx(0.4)


def test_path_lengths_and_densify():
    Q = np.array([[0, 0], [3, 4], [3, 4]], float)
    assert joint_path_length(Q) == pytest.approx(5.0)
    D = densify(Q, 0.5)
    assert np.allclose(D[0], Q[0]) and np.allclose(D[-1], Q[-1])
    assert np.max(np.abs(np.diff(D, axis=0))) <= 0.5 + 1e-12
    assert joint_path_length(D) == pytest.approx(5.0)
    assert cartesian_path_length(np.array([[0, 0, 0], [0, 0, 1.0]])) == 1.0


def test_min_clearance_uses_worst_state():
    cap = Capsule("L", np.zeros(3), np.zeros(3), 0.05)
    box = Box("b", np.array([0, 0, 0.0]), np.array([0.2, 0.2, 0.2]))

    def poses(q):
        T = np.eye(4)
        T[:3, 3] = [q[0], 0, 0]
        return {"L": T}

    Q = np.array([[1.0], [0.5], [0.8]])
    assert min_clearance(Q, poses, [cap], [box], capsule_box_distance) == pytest.approx(0.35)
    assert min_clearance(Q, poses, [cap], [], capsule_box_distance) == math.inf


def test_tracking_error():
    t = np.linspace(0, 1, 11)
    Qp = np.stack([t, 2 * t], axis=1)
    te = np.linspace(0, 1.2, 25)
    Qe = np.stack([te - 0.01, 2 * te], axis=1)
    e = tracking_error(t, Qp, te, Qe)
    assert e["rms"][0] == pytest.approx(0.01) and e["rms"][1] == pytest.approx(0.0, abs=1e-12)
    assert e["n"] == 21  # samples after t_plan end are excluded


def test_summarize():
    s = summarize([1, 2, 3, None, math.inf])
    assert s["n"] == 3 and s["median"] == 2
    assert summarize([]) == {"n": 0}


def test_scenes_load_and_are_sane():
    for f in (PKG / "scenes").glob("*.yaml"):
        sc = Scene.from_yaml(f)
        assert sc.boxes and (sc.queries or sc.sample_spec), f.name
        table = [b for b in sc.boxes if b.name == "table"][0]
        assert table.hi[2] < 0.0, "table top must be below the robot base (z=0) or the base collides"


def test_sampling_in_region():
    rng = np.random.default_rng(0)
    region = Box("r", np.array([0.5, 0, 0]), np.array([0.2, 2, 2]))
    q = sample_in_tcp_region(rng, lambda q: np.array([q[0], 0, 0]), np.array([0.0]), np.array([1.0]), region)
    assert q is not None and 0.4 <= q[0] <= 0.6
    assert sample_in_tcp_region(rng, lambda q: np.array([5.0, 0, 0]), np.array([0.0]), np.array([1.0]), region,
                                max_tries=50) is None


def test_time_offset_recovers_delay():
    from ex_planning_lab.core.metrics import estimate_time_offset
    t = np.linspace(0, 2, 201)
    Qp = np.stack([np.sin(t * 2), 0.5 * t ** 2], axis=1)
    te = np.linspace(-0.2, 2.5, 400)
    tq = np.clip(te - 0.12, 0, 2)  # executed = planned delayed by 120 ms
    Qe = np.stack([np.sin(tq * 2), 0.5 * tq ** 2], axis=1)
    assert estimate_time_offset(t, Qp, te, Qe) == pytest.approx(0.12, abs=0.003)


def test_min_clearance_ignores_mounting_pair_and_reports_pair():
    cap = Capsule("base_link", np.zeros(3), np.zeros(3), 0.1)
    table = Box("table", np.array([0, 0, -0.03]), np.array([2, 2, 0.04]))
    wall = Box("wall", np.array([1.0, 0, 0.5]), np.array([0.1, 1, 1]))
    poses = lambda q: {"base_link": np.eye(4)}  # noqa: E731
    d, pair = min_clearance(np.zeros((1, 1)), poses, [cap], [table, wall], capsule_box_distance, return_pair=True)
    assert pair == ("base_link", "wall") and d == pytest.approx(0.85)
    d2 = min_clearance(np.zeros((1, 1)), poses, [cap], [table], capsule_box_distance, ignore=frozenset())
    assert d2 < 0  # without the ignore rule the mount reads as penetration


def test_clearance_interval_brackets_truth():
    from ex_planning_lab.core.capsules import points_box_distance, voxel_downsample
    from ex_planning_lab.core.metrics import clearance_interval
    rng = np.random.default_rng(1)
    # "mesh": points on a thin rod offset from the bone; capsule fitted around it
    pts = np.stack([rng.uniform(0, 0.5, 4000), np.full(4000, 0.1), np.zeros(4000)], axis=1)
    cap = Capsule("L", np.zeros(3), np.array([0.5, 0, 0]), 0.1)   # bone fit, contains pts
    box = Box("b", np.array([0.25, 0.4, 0.0]), np.array([1.0, 0.2, 1.0]))  # face at y = 0.3
    poses = lambda q: {"L": np.eye(4)}  # noqa: E731
    lb, est, pair = clearance_interval(np.zeros((1, 1)), poses, [cap], {"L": voxel_downsample(pts, 0.005)}, [box],
                                       capsule_box_distance, points_box_distance)
    truth = 0.2  # rod at y=0.1, box face at y=0.3
    assert lb <= truth + 1e-9 <= est + 1e-9 and est - truth < 0.01 and pair == ("L", "b")
    assert lb == pytest.approx(0.2)  # capsule surface reaches y = 0.1 as well here


def test_points_box_penetration_negative():
    from ex_planning_lab.core.capsules import points_box_distance
    pts = np.array([[0.0, 0.0, 0.0]])
    assert points_box_distance(np.eye(4), pts, np.array([-0.1, -0.1, -0.05]), np.array([0.1, 0.1, 0.1])) == pytest.approx(-0.05)


def test_surface_samples_cover_long_thin_triangle():
    from ex_planning_lab.core.capsules import surface_samples
    tri = np.array([[[0, 0, 0], [0.3, 0, 0], [0, 0.02, 0]]], float)  # 300 mm sliver like the CR10 STL
    s = surface_samples(tri, 0.005)
    rng = np.random.default_rng(0)
    u, v = rng.uniform(size=(2, 2000))
    m = u + v > 1
    u[m], v[m] = 1 - u[m], 1 - v[m]
    q = tri[0, 0] + u[:, None] * (tri[0, 1] - tri[0, 0]) + v[:, None] * (tri[0, 2] - tri[0, 0])
    d = np.linalg.norm(q[:, None, :] - s[None, :, :], axis=2).min(axis=1)
    assert d.max() <= 0.005
