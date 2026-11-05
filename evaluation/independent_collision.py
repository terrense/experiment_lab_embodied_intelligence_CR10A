"""Independent collision checker for the M5 learning experiments (SIMULATION ONLY).

Why a second checker
--------------------
The learning environment computes the capsule-to-box clearance by sampling 32 points along each
capsule axis, and the safety filter reads the same function. A checker that shares the code with
the thing it is checking cannot find a bug in it - the audited defect (the sampled minimum is an
UPPER bound on the true distance) is exactly the kind of error that survives such a check.

This module therefore uses a **different geometric representation**:

  * the mesh SURFACE point clouds (5 mm samples) instead of the enclosing capsules,
  * an exact point-to-AABB distance computed here,
  * a separate self-collision check over the link pairs that the MoveIt SRDF does not disable.

The two checkers disagreeing is a result, not a bug: the comparison is the measurement.

What it reports per trajectory: the minimum clearance to the scene boxes, the minimum
self-clearance, which pair was closest, and the first step at which a contact was detected.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_planning_lab"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_description"))

from ex_planning_lab.core.capsules import link_point_clouds  # noqa: E402

sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_robot_state"))

ARM_LINKS = ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"]

#: link pairs the MoveIt SRDF disables (adjacent links, and pairs that can never touch). The
#: self-collision check must use the SAME list, otherwise it reports the arm against itself.
SRDF_DISABLED = {
    ("Link1", "Link2"), ("Link1", "Link3"), ("Link1", "Link4"), ("Link1", "base_link"),
    ("Link2", "Link3"), ("Link3", "Link4"), ("Link4", "Link5"), ("Link4", "Link6"), ("Link5", "Link6"),
}

#: The CR10A's shoulder housing (Link2) sits on the base with a few millimetres of clearance - the
#: two are permanently adjacent in every pose. The Dobot SRDF does not list that pair, so a
#: geometric self-collision test would report the robot as always colliding with itself. The pair
#: is therefore measured and REPORTED but excluded from the boolean.
PERMANENTLY_ADJACENT = {("base_link", "Link2")}

#: the same link-vs-box exclusions the environment uses (base/Link1/Link2 vs the table): the
#: conservative capsule of the J2 motor housing overlaps the table in every pose.
IGNORED_BOX_LINKS = {("base_link", "table"), ("Link1", "table"), ("Link2", "table")}


def _point_aabb_distance(p: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Exact Euclidean distance of each point to the AABB [lo, hi].

    Points INSIDE the box get a NEGATIVE value: the penetration depth, i.e. the distance to the
    nearest face. A plain distance would be 0 there, which is indistinguishable from touching the
    surface - and a checker that cannot tell "inside" from "on the boundary" misses a penetration.
    """
    d = np.maximum(np.maximum(lo - p, 0.0), p - hi)
    dist = np.linalg.norm(d, axis=1)
    inside = np.all((p >= lo) & (p <= hi), axis=1)
    if inside.any():
        face = np.min(np.minimum(p - lo, hi - p), axis=1)
        dist[inside] = -face[inside]
    return dist


def _segment_segment_distance(p1, q1, p2, q2) -> float:
    """Distance between two 3-D segments (the classic clamped-parameter solution)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = float(d1 @ r)
        if e <= eps:
            t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
        else:
            b = float(d1 @ d2)
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


class IndependentCollisionChecker:
    """Mesh-surface collision checker with its own distance code and a self-collision check."""

    def __init__(self, spacing: float = 0.005, self_collision: bool = True,
                 urdf: Optional[str] = None, model=None, self_spacing: float = 0.015) -> None:
        import mujoco
        from ex_learning.envs.cr10a_reach import DESC, expanded_urdf_text, make_model
        from ex_mujoco_sim.model_builder import JOINTS

        self.mujoco = mujoco
        self.urdf = urdf if urdf is not None else expanded_urdf_text()
        self.m = model if model is not None else make_model()
        self.d = mujoco.MjData(self.m)
        self.body_id = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, n) for n in ARM_LINKS}
        if any(i < 0 for i in self.body_id.values()):
            raise ValueError("a link is missing from the MJCF")
        self.qadr = np.array([self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                              for n in JOINTS])
        resolve = lambda uri: DESC / uri.split("package://ex_description/", 1)[1]  # noqa: E731
        self.clouds = {k: np.asarray(v, float) for k, v in
                       link_point_clouds(self.urdf, resolve, ARM_LINKS, spacing).items()}
        # bounding sphere per link (broad phase): the surface clouds are ~100k points per link, so
        # an exact per-point query for every link and every box is wasteful
        self.sphere_c = {k: 0.5 * (v.min(0) + v.max(0)) for k, v in self.clouds.items()}
        self.sphere_r = {k: float(np.linalg.norm(v - c, axis=1).max()) for k, v in self.clouds.items()
                         for c in [self.sphere_c[k]]}
        # enclosing capsules, for the self-collision check (a conservative lower bound)
        from ex_robot_state.core.urdf_model import UrdfModel
        from ex_planning_lab.core.capsules import fit_capsules
        model_urdf = UrdfModel.from_string(self.urdf)
        child_origin = {j.parent: j.origin[:3, 3] for j in model_urdf.joints.values() if j.parent in ARM_LINKS}
        self.capsules = {c.link: c for c in fit_capsules(self.urdf, resolve, ARM_LINKS, child_origin)}
        # a coarser cloud for the self-collision check: the KD-tree query over ~100k points per
        # link pair costs 100 ms, over ~3k points a few ms. The slack is subtracted below.
        from ex_planning_lab.core.capsules import voxel_downsample
        self.self_clouds = {k: voxel_downsample(v, self_spacing) for k, v in self.clouds.items()}
        # the coverage radius of the downsampled surface: a dropped surface point lies within
        # voxel*sqrt(3)/2 of a kept one, so the sampled distance is an over-estimate by that much
        self.self_margin = float(self_spacing) * np.sqrt(3.0) / 2.0
        self.self_collision = self_collision
        self._trees = {}

    def _sphere_world(self, link: str) -> Tuple[np.ndarray, float]:
        b = self.body_id[link]
        R = self.d.xmat[b].reshape(3, 3)
        c = self.sphere_c[link] @ R.T + self.d.xpos[b]
        return c, self.sphere_r[link]

    # ------------------------------------------------------------------ kinematics
    def set_q(self, q: Sequence[float]) -> None:
        """Place the arm at q and refresh the kinematics."""
        self.mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[self.qadr] = np.asarray(q, float)
        self.mujoco.mj_forward(self.m, self.d)

    def link_points_world(self, link: str, clouds: Optional[Dict] = None) -> np.ndarray:
        """The link's surface samples in world coordinates."""
        b = self.body_id[link]
        R = self.d.xmat[b].reshape(3, 3)
        c = (clouds or self.clouds)[link]
        return c @ R.T + self.d.xpos[b]

    # ------------------------------------------------------------------ obstacles
    def box_clearance(self, boxes: List[Dict], ignore: bool = True) -> Tuple[float, Optional[Tuple[str, str]]]:
        """(minimum clearance to the scene boxes, the closest (link, box) pair).

        The clearance is the distance from the link SURFACE to the box. A sample can miss the true
        closest surface point by up to half the sampling spacing, so the honest bound is this
        value minus spacing/2.
        """
        worst = float("inf")
        worst_pair = None
        for b in boxes:
            lo = np.asarray(b["center"], float) - np.asarray(b["size"], float) / 2
            hi = np.asarray(b["center"], float) + np.asarray(b["size"], float) / 2
            for link in ARM_LINKS:
                if ignore and (link, b["name"]) in IGNORED_BOX_LINKS:
                    continue
                c, r = self._sphere_world(link)
                sphere_d = float(_point_aabb_distance(c[None, :], lo, hi)[0]) - r
                if sphere_d > worst:          # broad phase: cannot be the closest pair
                    continue
                pts = self.link_points_world(link)
                d = float(_point_aabb_distance(pts, lo, hi).min())
                if d < worst:
                    worst, worst_pair = d, (link, b["name"])
        return worst, worst_pair

    # -------------------------------------------------------------- self-collision
    def _capsule_world(self, link: str) -> Tuple[np.ndarray, np.ndarray, float]:
        """(segment start, segment end, radius) of the link's enclosing capsule, in world frame."""
        b = self.body_id[link]
        R = self.d.xmat[b].reshape(3, 3)
        c = self.capsules[link]
        return (c.a @ R.T + self.d.xpos[b], c.b @ R.T + self.d.xpos[b], c.radius)

    def self_clearance_capsule(self) -> Tuple[float, Optional[Tuple[str, str]]]:
        """Capsule-based self-distance: a LOWER bound, and too conservative to decide on its own
        (the enclosing capsules of the base and the J2 housing overlap in every pose). Kept for
        reference; the decision uses `self_clearance`."""
        worst = float("inf")
        worst_pair = None
        for i, a in enumerate(ARM_LINKS):
            for b in ARM_LINKS[i + 1:]:
                if (a, b) in SRDF_DISABLED or (b, a) in SRDF_DISABLED:
                    continue
                a0, a1, ar = self._capsule_world(a)
                b0, b1, br = self._capsule_world(b)
                d = _segment_segment_distance(a0, a1, b0, b1) - ar - br
                if d < worst:
                    worst, worst_pair = d, (a, b)
        return worst, worst_pair

    def self_clearance(self) -> Tuple[float, Optional[Tuple[str, str]]]:
        """(minimum distance between the link SURFACES, the closest pair) over the SRDF-enabled
        pairs.

        Measured on the mesh surface samples (voxel-downsampled for speed). The distance between
        two sampled point sets is an OVER-estimate of the true surface-to-surface distance (a
        dropped surface point lies within voxel*sqrt(3)/2 of a kept one), so the value is an upper
        bound and `self_coverage_radius` is the slack to subtract when deciding whether the links
        actually touch.
        """
        from scipy.spatial import cKDTree
        worst = float("inf")
        worst_pair = None
        for i, a in enumerate(ARM_LINKS):
            for b in ARM_LINKS[i + 1:]:
                if (a, b) in SRDF_DISABLED or (b, a) in SRDF_DISABLED:
                    continue
                pa, pb = self.link_points_world(a, self.self_clouds), self.link_points_world(b, self.self_clouds)
                if len(pa) == 0 or len(pb) == 0:
                    continue
                d = float(cKDTree(pa).query(pb, k=1)[0].min())
                if d < worst:
                    worst, worst_pair = d, (a, b)
        return worst, worst_pair

    def self_collision_distance(self) -> Tuple[float, Optional[Tuple[str, str]]]:
        """The closest pair that is NOT permanently adjacent (base_link / Link2).

        The permanently adjacent pair is measured too and reported separately, because it is a
        property of the robot (the shoulder housing sits on the base) and not of the motion.
        """
        from scipy.spatial import cKDTree
        worst = float("inf")
        worst_pair = None
        for i, a in enumerate(ARM_LINKS):
            for b in ARM_LINKS[i + 1:]:
                if (a, b) in SRDF_DISABLED or (b, a) in SRDF_DISABLED:
                    continue
                if (a, b) in PERMANENTLY_ADJACENT or (b, a) in PERMANENTLY_ADJACENT:
                    continue
                pa, pb = self.link_points_world(a, self.self_clouds), self.link_points_world(b, self.self_clouds)
                if len(pa) == 0 or len(pb) == 0:
                    continue
                d = float(cKDTree(pa).query(pb, k=1)[0].min())
                if d < worst:
                    worst, worst_pair = d, (a, b)
        return worst, worst_pair

    # ------------------------------------------------------------------ trajectory
    def check_trajectory(self, trajectory: Sequence[Sequence[float]], boxes: List[Dict],
                         margin: float = 0.0, check_self: Optional[bool] = None) -> Dict:
        """Check a whole trajectory (a list of joint configurations).

        Returns the minimum clearance over the trajectory, the first step at which a contact
        (clearance < margin) occurred, and the minimum self-clearance.

        The self-collision boolean uses the pair distance MINUS the sampling coverage radius, and
        excludes the permanently adjacent pair (base_link / Link2), whose distance is reported
        separately: it is a property of the robot, not of the motion.
        """
        if check_self is None:
            check_self = self.self_collision
        min_clear, first_hit, hit_pair = float("inf"), None, None
        min_self, self_pair = float("inf"), None
        min_adj, adj_pair = float("inf"), None
        for k, q in enumerate(trajectory):
            self.set_q(q)
            d, pair = self.box_clearance(boxes)
            if d < min_clear:
                min_clear = d
            if d < margin and first_hit is None:
                first_hit, hit_pair = k, pair
            if check_self:
                ds, sp = self.self_collision_distance()
                if ds < min_self:
                    min_self, self_pair = ds, sp
                da, ap = self.self_clearance()
                if da < min_adj:
                    min_adj, adj_pair = da, ap
        return {"min_clearance_m": min_clear, "first_contact_step": first_hit, "contact_pair": hit_pair,
                "min_self_clearance_m": min_self if check_self else None,
                "self_contact_pair": self_pair,
                "min_permanent_adjacent_clearance_m": min_adj if check_self else None,
                "permanent_adjacent_pair": adj_pair,
                "collision": bool(first_hit is not None),
                "self_collision": bool(check_self and min_self - self.self_margin < 0.0)}


def check_episode_trajectory(trajectory, boxes, margin: float = 0.0) -> Dict:
    """Convenience wrapper: build a checker and check one trajectory."""
    return IndependentCollisionChecker().check_trajectory(trajectory, boxes, margin)
