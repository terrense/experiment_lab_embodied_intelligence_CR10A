"""CR10A reach-around-the-wall task (SIMULATION ONLY): the M2 "wall" scene in the M4 plant.

Start and goal sit on opposite sides of the wall (around the M2 query sweep_60, +-8 deg noise,
sides swapped at random), so the straight Cartesian path is blocked. Collision is DETECTED, not
simulated: robot geoms stay visual-only (as in M4).

Clearance = signed distance between the M2 link capsules (each CONTAINS its link mesh, so the
clearance is a conservative lower bound) and the scene's axis-aligned boxes, computed in numpy.
Not `mj_geomDistance` on the meshes: on this model it returned a spurious exact 0.0 for 3.3 % of
mesh-box queries (GJK failures), changing the CCD options crashed MuJoCo 3.3.7, and its `fromto`
endpoints came back swapped for mesh-vs-box pairs (found 2026-10-28, see docs/LEARNING.md).

Obs (22) = ReachEnv obs (21) + min clearance (m, capped at `clearance_cap`).
Reward   = progress_weight * (previous - current goal distance) - 0.01 |a|^2, +1 once on reaching the
           goal, - shaping near obstacles; contact (clearance < 0) ends the episode with
           -collision_penalty. Potential-based: standing still earns ~0, so dying early never pays
           (the v1 reward -distance/step taught PPO to crash, see docs/LEARNING.md).
           Success = TCP within `success_radius` at the end, no contact.
"""

from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import mujoco
import numpy as np
import yaml

from .cr10a_reach import DESC, ROOT, ReachConfig, ReachEnv, expanded_urdf_text, mjcf_text

sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_planning_lab"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_robot_state"))
from ex_planning_lab.core.capsules import fit_capsules, link_point_clouds, voxel_downsample  # noqa: E402
from ex_robot_state.core.urdf_model import UrdfModel  # noqa: E402

SCENES = ROOT / "ros2_ws" / "src" / "ex_planning_lab" / "scenes"
ARM_LINKS = ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"]
# base_link/Link1: mounted on the table (as in M2 metrics). Link2: its enclosing capsule (r 0.15 m, the J2
# motor housing) overlaps the table by 3.8 cm at the shoulder in EVERY pose - an artefact of the
# conservative capsule; arm-to-table contact is still covered by Link3..Link6.
IGNORE = {("base_link", "table"), ("Link1", "table"), ("Link2", "table")}
SEG_SAMPLES = 32


def load_scene(name: str) -> Tuple[List[Dict], Dict[str, Dict], Optional[Dict]]:
    s = yaml.safe_load((SCENES / f"{name}.yaml").read_text())
    return s["boxes"], {q["name"]: q for q in s.get("queries", [])}, s.get("sample_queries")


def model_with_boxes(urdf: str, boxes: List[Dict]) -> mujoco.MjModel:
    """M4 plant + the obstacle boxes as visual geoms (for the viewer only)."""
    root = ET.fromstring(mjcf_text(urdf))
    world = root.find("worldbody")
    for b in boxes:
        half = " ".join(f"{v / 2:.4f}" for v in b["size"])
        world.append(ET.Element("geom", {"name": f"obs_{b['name']}", "type": "box", "size": half,
                                         "pos": " ".join(map(str, b["center"])), "rgba": "0.8 0.3 0.3 0.4",
                                         "contype": "0", "conaffinity": "0"}))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


@dataclass
class WallConfig(ReachConfig):
    max_steps: int = 300
    scene: str = "wall"
    query: Optional[str] = "sweep_60"  # None: sample start/goal from the scene's sample_queries TCP boxes
    pool_size: int = 256              # sampled mode: valid start / goal configurations (fixed pool_seed)
    pool_seed: int = 7
    collision_model: str = "capsules"  # capsules (1 enclosing capsule/link, coarse) | points (mesh surface samples, tight)
    point_voxel: float = 0.01          # points model: voxel size (m); margin = voxel * sqrt(3)
    noise_deg: float = 8.0
    clearance_cap: float = 0.20
    shaping_margin: float = 0.05
    shaping_weight: float = 1.0
    collision_penalty: float = 10.0
    progress_weight: float = 10.0
    # generalisation experiment: a new wall every episode (None = the scene's fixed wall)
    wall_ranges: Optional[Dict[str, Tuple[float, float]]] = None  # keys cx, cy, height, width (m)
    wall_in_obs: bool = False          # append wall centre + half sizes (6) to the observation


TRAIN_WALLS = {"cx": (-0.80, -0.60), "cy": (-0.10, 0.10), "height": (0.70, 1.00), "width": (0.35, 0.60)}
OOD_WALLS = {"cx": (-0.85, -0.55), "cy": (-0.15, 0.15), "height": (1.00, 1.15), "width": (0.60, 0.75)}  # taller+wider than any training wall


class ReachWallEnv(ReachEnv):
    observation_dim = 22

    def __init__(self, cfg: Optional[WallConfig] = None, seed: int = 0) -> None:
        cfg = cfg or WallConfig()
        urdf = expanded_urdf_text()
        boxes, queries, sample_spec = load_scene(cfg.scene)
        super().__init__(cfg, model=model_with_boxes(urdf, boxes), seed=seed)
        self.sample_spec = None
        if cfg.query is not None:
            q = queries[cfg.query]
            self.q_a, self.q_b = np.radians(q["start_deg"]), np.radians(q["goal_deg"])
        else:
            if sample_spec is None:
                raise ValueError(f"scene {cfg.scene} has no sample_queries")
            self.sample_spec = sample_spec
        # obstacles: axis-aligned boxes
        self.box_names = [b["name"] for b in boxes]
        self.box_lo = np.array([np.subtract(b["center"], np.divide(b["size"], 2)) for b in boxes])
        self.box_hi = np.array([np.add(b["center"], np.divide(b["size"], 2)) for b in boxes])
        # robot: one enclosing capsule per link (same fit as the M2 clearance metric)
        model = UrdfModel.from_string(urdf)
        child_origin = {j.parent: j.origin[:3, 3] for j in model.joints.values() if j.parent in ARM_LINKS}
        resolve = lambda uri: DESC / uri.split("package://ex_description/", 1)[1]  # noqa: E731
        caps = fit_capsules(urdf, resolve, ARM_LINKS, child_origin)
        self.capsules = caps
        self.cap_body = np.array([mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, c.link) for c in caps])
        if np.any(self.cap_body < 0):
            raise ValueError("capsule link missing in the MJCF")
        self.cap_a = np.array([c.a for c in caps])
        self.cap_b = np.array([c.b for c in caps])
        self.cap_r = np.array([c.radius for c in caps])
        self.pairs = [(ci, bi) for ci, c in enumerate(caps) for bi, bn in enumerate(self.box_names) if (c.link, bn) not in IGNORE]
        self._pi = np.array([p[0] for p in self.pairs])
        self._pb = np.array([p[1] for p in self.pairs])
        self._t = np.linspace(0.0, 1.0, SEG_SAMPLES)
        if cfg.collision_model == "points":
            # mesh SURFACE samples (5 mm), voxel-downsampled: every dropped surface point lies within
            # voxel*sqrt(3) of a kept one, so subtracting that margin keeps the clearance conservative
            clouds = link_point_clouds(urdf, lambda uri: DESC / uri.split("package://ex_description/", 1)[1], ARM_LINKS, 0.005)
            clouds = {k: voxel_downsample(v, cfg.point_voxel) for k, v in clouds.items()}
            self.link_body = self.cap_body
            self.pt_local = np.concatenate([clouds[c.link] for c in caps])
            self.pt_link = np.concatenate([np.full(len(clouds[c.link]), i) for i, c in enumerate(caps)])
            starts = np.cumsum([0] + [len(clouds[c.link]) for c in caps])
            self.pt_slices = [slice(starts[i], starts[i + 1]) for i in range(len(caps))]
            self.pt_margin = cfg.point_voxel * math.sqrt(3)
            self.sphere_c = np.array([0.5 * (self.pt_local[sl].min(0) + self.pt_local[sl].max(0)) for sl in self.pt_slices])
            self.sphere_r = np.array([np.linalg.norm(self.pt_local[sl] - c, axis=1).max() for sl, c in zip(self.pt_slices, self.sphere_c)])
        elif cfg.collision_model != "capsules":
            raise ValueError(f"collision_model must be capsules|points, got {cfg.collision_model!r}")
        self.collided = False
        self.last_pair: Optional[Tuple[str, str]] = None
        if cfg.wall_ranges is not None or cfg.wall_in_obs:
            self.wall_i = self.box_names.index("wall")
            self.wall_geom = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "obs_wall")
        self.observation_dim = 22 + (6 if cfg.wall_in_obs else 0)
        if self.sample_spec is not None:
            self.start_pool = self._sample_pool(self.sample_spec["start_tcp_box"], cfg.pool_seed)
            self.goal_pool = self._sample_pool(self.sample_spec["goal_tcp_box"], cfg.pool_seed + 1)

    def _sample_pool(self, box: Dict, seed: int, max_tries: int = 2_000_000) -> np.ndarray:
        """Collision-free joint configurations whose TCP lies in `box` (rejection sampling, fixed seed).

        Two phases: (1) uniform over the joint ranges (J1 limited to the work side +-90 deg, J4/J6 to
        +-180 deg: their full range is redundant) until 50 configurations put the TCP in the box;
        (2) uniform over the bounding box of those hits padded by 10 deg. Phase 2 is ~10x more
        efficient for the far window goals; configurations outside that joint sub-box are never drawn."""
        rng = np.random.default_rng(seed)
        lo, hi = self.lo.copy(), self.hi.copy()
        lo[[0, 3, 5]] = np.maximum(lo[[0, 3, 5]], [-math.pi / 2, -math.pi, -math.pi])
        hi[[0, 3, 5]] = np.minimum(hi[[0, 3, 5]], [math.pi / 2, math.pi, math.pi])
        blo, bhi = np.asarray(box["lo"], float), np.asarray(box["hi"], float)
        hits, pool, tries = [], [], 0
        while len(pool) < self.cfg.pool_size and tries < max_tries:
            tries += 1
            if len(hits) == 50:  # switch to phase 2
                H = np.array(hits)
                lo = np.maximum(H.min(0) - math.radians(10), lo)
                hi = np.minimum(H.max(0) + math.radians(10), hi)
                hits.append(None)
            q = rng.uniform(lo, hi)
            self._set_q(q)
            p = self.tcp()
            if np.all(p >= blo) and np.all(p <= bhi):
                if len(hits) < 50:
                    hits.append(q)
                if self.clearance() > 0.02:
                    pool.append(q)
        if len(pool) < self.cfg.pool_size:
            raise RuntimeError(f"only {len(pool)} valid configurations in {box} after {tries} tries")
        self.pool_acceptance = getattr(self, "pool_acceptance", []) + [len(pool) / tries]
        return np.array(pool)

    # --------------------------------------------------------------- geometry
    def _segments_world(self) -> Tuple[np.ndarray, np.ndarray]:
        R = self.d.xmat[self.cap_body].reshape(-1, 3, 3)
        p = self.d.xpos[self.cap_body]
        A = np.einsum("nij,nj->ni", R, self.cap_a) + p
        B = np.einsum("nij,nj->ni", R, self.cap_b) + p
        return A, B

    @staticmethod
    def _unit_away(robot_pt: np.ndarray, box_pt: np.ndarray, centre: np.ndarray) -> np.ndarray:
        n = robot_pt - box_pt
        bad = np.linalg.norm(n, axis=-1) < 1e-9          # robot point inside the box: away from its centre
        n[bad] = robot_pt[bad] - centre[bad]
        return n / (np.linalg.norm(n, axis=-1, keepdims=True) + 1e-12)

    def pair_distances(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Every (link, box) pair -> (signed clearance, nearest point on the robot surface, unit normal away from the box)."""
        if self.cfg.collision_model == "points":
            return self._pair_distances_points()
        A, B = self._segments_world()
        pts = A[self._pi, None, :] + self._t[None, :, None] * (B - A)[self._pi, None, :]   # (P, S, 3)
        lo, hi = self.box_lo[self._pb][:, None, :], self.box_hi[self._pb][:, None, :]
        q = np.clip(pts, lo, hi)
        dist = np.linalg.norm(pts - q, axis=2)                                               # 0 inside
        k = np.argmin(dist, axis=1)
        idx = np.arange(len(self.pairs))
        seg, box = pts[idx, k], q[idx, k]
        centre = 0.5 * (self.box_lo[self._pb] + self.box_hi[self._pb])
        n = self._unit_away(seg, box, centre)
        r = self.cap_r[self._pi]
        return dist[idx, k] - r, seg - n * r[:, None], n

    def _pair_distances_points(self):
        R = self.d.xmat[self.link_body].reshape(-1, 3, 3)
        X = self.d.xpos[self.link_body]
        # broad phase: link bounding spheres vs boxes. Pairs farther than the observation cap keep that
        # lower bound (sphere distance) and their points are never transformed.
        C = np.einsum("lij,lj->li", R, self.sphere_c) + X
        sph = np.linalg.norm(C[self._pi] - np.clip(C[self._pi], self.box_lo[self._pb], self.box_hi[self._pb]), axis=1) - self.sphere_r[self._pi]
        d = np.maximum(sph, 0.0) + self.pt_margin
        rp = C[self._pi].copy(); bp = np.clip(C[self._pi], self.box_lo[self._pb], self.box_hi[self._pb])
        near = sph < self.cfg.clearance_cap + 0.05
        for li in np.unique(self._pi[near]):
            sl = self.pt_slices[li]
            W = self.pt_local[sl] @ R[li].T + X[li]
            for k in np.where(near & (self._pi == li))[0]:
                lo, hi = self.box_lo[self._pb[k]], self.box_hi[self._pb[k]]
                q = np.clip(W, lo, hi)
                dist = np.linalg.norm(W - q, axis=1)
                inside = dist == 0.0
                if inside.any():  # penetration depth (negative): distance to the nearest face
                    dist[inside] = -np.minimum(W[inside] - lo, hi - W[inside]).min(axis=1)
                j = int(np.argmin(dist))
                d[k], rp[k], bp[k] = dist[j], W[j], q[j]
        centre = 0.5 * (self.box_lo[self._pb] + self.box_hi[self._pb])
        return d - self.pt_margin, rp, self._unit_away(rp, bp, centre)

    def clearance(self, with_detail: bool = False):
        d, _, _ = self.pair_distances()
        i = int(np.argmin(d))
        return (float(d[i]), self.pairs[i]) if with_detail else float(d[i])

    def repulsion_points(self, d0: float):
        """[(signed distance, point on the robot surface, unit normal away from the box, body id)] for pairs within d0."""
        d, s, n = self.pair_distances()
        return [(float(d[i]), s[i], n[i], int(self.cap_body[self._pi[i]])) for i in np.where(d < d0)[0]]

    def obs(self) -> np.ndarray:
        c = min(self.clearance(), self.cfg.clearance_cap) if hasattr(self, "pairs") else self.cfg.clearance_cap
        self.last_clearance = float(c)
        o = np.concatenate([super().obs(), [c]])
        if self.cfg.wall_in_obs and hasattr(self, "wall_i"):
            lo, hi = self.box_lo[self.wall_i], self.box_hi[self.wall_i]
            o = np.concatenate([o, 0.5 * (lo + hi), 0.5 * (hi - lo)])
        return o

    def set_wall(self, cx: float, cy: float, height: float, width: float, thickness: float = 0.04) -> None:
        c = np.array([cx, cy, height / 2]); half = np.array([width / 2, thickness / 2, height / 2])
        self.box_lo[self.wall_i], self.box_hi[self.wall_i] = c - half, c + half
        self.m.geom_pos[self.wall_geom], self.m.geom_size[self.wall_geom] = c, half   # viewer only

    # --------------------------------------------------------------------- API
    def reset(self, seed: Optional[int] = None, goal=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        if self.sample_spec is not None:  # sampled mode: start on the robot side, goal behind the wall
            q_goal = self.goal_pool[self.rng.integers(len(self.goal_pool))]
            self._set_q(q_goal)
            goal_xyz = self.tcp()
            q0 = self.start_pool[self.rng.integers(len(self.start_pool))]
            self._set_q(q0)
            self.q_ref = q0.copy()
            self.goal = goal_xyz if goal is None else np.asarray(goal, float)
            self.t, self.succeeded, self.collided, self.succeeded_bonus = 0, False, False, False
            self.q_goal = q_goal.copy()  # privileged: only the diagnostic joint_interp baseline reads it
            return self.obs(), {"goal": self.goal.copy(), "q_goal": q_goal.copy()}
        if self.cfg.wall_ranges is not None:
            return self._reset_random_wall(goal)
        qa, qb = (self.q_a, self.q_b) if self.rng.random() < 0.5 else (self.q_b, self.q_a)
        n = math.radians(self.cfg.noise_deg)
        q_goal = qb + self.rng.uniform(-n, n, 6)
        self._set_q(q_goal)
        goal_xyz = self.tcp()
        for _ in range(20):  # start pose must itself be collision-free
            q0 = qa + self.rng.uniform(-n, n, 6)
            self._set_q(q0)
            if self.clearance() > 0.01:
                break
        self.q_ref = q0.copy()
        self.goal = goal_xyz if goal is None else np.asarray(goal, float)
        self.t, self.succeeded, self.collided, self.succeeded_bonus = 0, False, False, False
        self.q_goal = q_goal.copy()  # privileged: only the diagnostic joint_interp baseline reads it
        return self.obs(), {"goal": self.goal.copy(), "q_goal": q_goal}

    def _reset_random_wall(self, goal=None):
        """New wall every episode; start AND goal must be collision-free (else resample, wall included)."""
        r, n = self.cfg.wall_ranges, math.radians(self.cfg.noise_deg)
        for _ in range(200):
            self.set_wall(*(self.rng.uniform(*r[k]) for k in ("cx", "cy", "height", "width")))
            qa, qb = (self.q_a, self.q_b) if self.rng.random() < 0.5 else (self.q_b, self.q_a)
            q_goal = qb + self.rng.uniform(-n, n, 6)
            self._set_q(q_goal)
            if self.clearance() <= 0.02:
                continue
            goal_xyz = self.tcp()
            q0 = qa + self.rng.uniform(-n, n, 6)
            self._set_q(q0)
            if self.clearance() > 0.02:
                break
        else:
            raise RuntimeError("no valid random wall episode in 200 tries")
        self.q_ref = q0.copy()
        self.goal = goal_xyz if goal is None else np.asarray(goal, float)
        self.t, self.succeeded, self.collided, self.succeeded_bonus = 0, False, False, False
        self.q_goal = q_goal.copy()
        return self.obs(), {"goal": self.goal.copy(), "q_goal": q_goal}

    def step(self, action):
        d_prev = float(np.linalg.norm(self.goal - self.tcp()))
        obs, _, term, trunc, info = super().step(action)
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        # progress reward (potential-based): bounded episode return, ~0 for standing still.
        # v1 used the base reward (-distance every step): ending the episode early by colliding
        # then beat living on (-10 once vs ~-0.5 per remaining step) and PPO learned to crash.
        reward = self.cfg.progress_weight * (d_prev - info["dist"]) - 0.01 * float(a @ a)
        if info["success"] and not self.succeeded_bonus:
            reward += 1.0
            self.succeeded_bonus = True
        c = self.last_clearance  # NOT obs[-1]: with the wall in the observation that is the wall half-height
        if c < self.cfg.shaping_margin:
            reward -= self.cfg.shaping_weight * (self.cfg.shaping_margin - c) / self.cfg.shaping_margin
        if c < 0.0:
            self.collided, term = True, True
            reward -= self.cfg.collision_penalty
            _, (ci, bi) = self.clearance(with_detail=True)
            self.last_pair = (self.capsules[ci].link, self.box_names[bi])
        info.update({"clearance": c, "collision": self.collided, "success": bool(info["success"] and not self.collided)})
        return obs, reward, term, trunc, info
