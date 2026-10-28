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
Reward   = ReachEnv reward - shaping near obstacles; contact (clearance < 0) ends the episode
           with `collision_penalty`. Success = TCP within `success_radius` at the end, no contact.
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
from ex_planning_lab.core.capsules import fit_capsules  # noqa: E402
from ex_robot_state.core.urdf_model import UrdfModel  # noqa: E402

SCENES = ROOT / "ros2_ws" / "src" / "ex_planning_lab" / "scenes"
ARM_LINKS = ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"]
# base_link/Link1: mounted on the table (as in M2 metrics). Link2: its enclosing capsule (r 0.15 m, the J2
# motor housing) overlaps the table by 3.8 cm at the shoulder in EVERY pose - an artefact of the
# conservative capsule; arm-to-table contact is still covered by Link3..Link6.
IGNORE = {("base_link", "table"), ("Link1", "table"), ("Link2", "table")}
SEG_SAMPLES = 32


def load_scene(name: str) -> Tuple[List[Dict], Dict[str, Dict]]:
    s = yaml.safe_load((SCENES / f"{name}.yaml").read_text())
    return s["boxes"], {q["name"]: q for q in s["queries"]}


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
    query: str = "sweep_60"
    noise_deg: float = 8.0
    clearance_cap: float = 0.20
    shaping_margin: float = 0.05
    shaping_weight: float = 1.0
    collision_penalty: float = 10.0


class ReachWallEnv(ReachEnv):
    observation_dim = 22

    def __init__(self, cfg: Optional[WallConfig] = None, seed: int = 0) -> None:
        cfg = cfg or WallConfig()
        urdf = expanded_urdf_text()
        boxes, queries = load_scene(cfg.scene)
        super().__init__(cfg, model=model_with_boxes(urdf, boxes), seed=seed)
        q = queries[cfg.query]
        self.q_a, self.q_b = np.radians(q["start_deg"]), np.radians(q["goal_deg"])
        # obstacles: axis-aligned boxes
        self.box_names = [b["name"] for b in boxes]
        self.box_lo = np.array([np.subtract(b["center"], np.divide(b["size"], 2)) for b in boxes])
        self.box_hi = np.array([np.add(b["center"], np.divide(b["size"], 2)) for b in boxes])
        # robot: one enclosing capsule per link (same fit as the M2 clearance metric)
        model = UrdfModel.from_string(urdf)
        child_origin = {j.parent: j.origin[:3, 3] for j in model.joints.values() if j.parent in ARM_LINKS}
        caps = fit_capsules(urdf, lambda uri: DESC / uri.split("package://ex_description/", 1)[1], ARM_LINKS, child_origin)
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
        self.collided = False
        self.last_pair: Optional[Tuple[str, str]] = None

    # --------------------------------------------------------------- geometry
    def _segments_world(self) -> Tuple[np.ndarray, np.ndarray]:
        R = self.d.xmat[self.cap_body].reshape(-1, 3, 3)
        p = self.d.xpos[self.cap_body]
        A = np.einsum("nij,nj->ni", R, self.cap_a) + p
        B = np.einsum("nij,nj->ni", R, self.cap_b) + p
        return A, B

    def pair_distances(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """For every (capsule, box) pair: signed distance, nearest segment point, nearest box point."""
        A, B = self._segments_world()
        pts = A[self._pi, None, :] + self._t[None, :, None] * (B - A)[self._pi, None, :]   # (P, S, 3)
        lo, hi = self.box_lo[self._pb][:, None, :], self.box_hi[self._pb][:, None, :]
        q = np.clip(pts, lo, hi)
        dist = np.linalg.norm(pts - q, axis=2)                                               # 0 inside
        k = np.argmin(dist, axis=1)
        idx = np.arange(len(self.pairs))
        return dist[idx, k] - self.cap_r[self._pi], pts[idx, k], q[idx, k]

    def clearance(self, with_detail: bool = False):
        d, _, _ = self.pair_distances()
        i = int(np.argmin(d))
        return (float(d[i]), self.pairs[i]) if with_detail else float(d[i])

    def repulsion_points(self, d0: float):
        """[(signed distance, point on capsule surface, unit normal away from the box, body id)] for pairs within d0."""
        d, s, q = self.pair_distances()
        out = []
        for i in np.where(d < d0)[0]:
            n = s[i] - q[i]
            nn = np.linalg.norm(n)
            if nn < 1e-9:  # segment point inside the box: push away from the box centre
                c = 0.5 * (self.box_lo[self._pb[i]] + self.box_hi[self._pb[i]])
                n = s[i] - c
                nn = np.linalg.norm(n) + 1e-9
            n = n / nn
            out.append((float(d[i]), s[i] - n * self.cap_r[self._pi[i]], n, int(self.cap_body[self._pi[i]])))
        return out

    def obs(self) -> np.ndarray:
        c = min(self.clearance(), self.cfg.clearance_cap) if hasattr(self, "pairs") else self.cfg.clearance_cap
        return np.concatenate([super().obs(), [c]])

    # --------------------------------------------------------------------- API
    def reset(self, seed: Optional[int] = None, goal=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
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
        self.t, self.succeeded, self.collided = 0, False, False
        return self.obs(), {"goal": self.goal.copy(), "q_goal": q_goal}

    def step(self, action):
        obs, reward, term, trunc, info = super().step(action)
        c = float(obs[-1])
        if c < self.cfg.shaping_margin:
            reward -= self.cfg.shaping_weight * (self.cfg.shaping_margin - c) / self.cfg.shaping_margin
        if c < 0.0:
            self.collided, term = True, True
            reward -= self.cfg.collision_penalty
            _, (ci, bi) = self.clearance(with_detail=True)
            self.last_pair = (self.capsules[ci].link, self.box_names[bi])
        info.update({"clearance": c, "collision": self.collided, "success": bool(info["success"] and not self.collided)})
        return obs, reward, term, trunc, info
