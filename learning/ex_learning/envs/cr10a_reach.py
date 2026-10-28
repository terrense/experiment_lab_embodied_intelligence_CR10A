"""CR10A reaching task in MuJoCo (SIMULATION ONLY), Gymnasium-style API without the dependency.

Physics: the M4 plant (same MJCF builder + servo-drive model) at 1 kHz; the policy acts at 50 Hz.
Action:  joint velocity command in [-1, 1]^6, scaled by `max_joint_speed` (rad/s); integrated into a
         position set-point that the servo tracks (like a velocity-mode teleop on a real arm).
Obs:     [q (6), qd (6), tcp (3), goal (3), goal - tcp (3)]  -> 21-dim, float64
Reward:  -|goal - tcp| - 0.01 |a|^2 - joint-limit penalty, +1 once when within `success_radius`
Episode: `max_steps` policy steps (default 200 = 4 s); success = within radius at the end.

`make_model()` builds the MJCF from the calibrated ex_cr10 URDF via xacro (ROS env sourced),
so kinematics are identical to MoveIt / the live digital twin.
"""

from __future__ import annotations

import math
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_mujoco_sim"))
from ex_mujoco_sim.model_builder import JOINTS, DynamicsAssumptions, build_mjcf  # noqa: E402
from ex_mujoco_sim.servo import gains_from_inertia, servo_torque  # noqa: E402

HOME = np.array([0.0, -0.35, 1.75, -1.4, -1.5708, 0.0])  # arm in front (-x), wrist down


DESC = ROOT / "ros2_ws" / "src" / "ex_description"


def expanded_urdf_text(calibrated: bool = True) -> str:
    import xacro
    text = (DESC / "urdf" / "ex_cr10.urdf.xacro").read_text().replace("$(find ex_description)", str(DESC))
    with tempfile.NamedTemporaryFile("w", suffix=".xacro", delete=False) as f:
        f.write(text)
        tmp = f.name
    try:
        return xacro.process_file(tmp, mappings={"calibrated": "true" if calibrated else "false"}).toxml()
    finally:
        os.unlink(tmp)


def mjcf_text(urdf: str, dyn: Optional[DynamicsAssumptions] = None) -> str:
    return build_mjcf(urdf, lambda uri: DESC / uri.split("package://ex_description/", 1)[1], dyn or DynamicsAssumptions())


def make_model(calibrated: bool = True) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(mjcf_text(expanded_urdf_text(calibrated)))


@dataclass
class ReachConfig:
    max_steps: int = 200
    control_dt: float = 0.02
    max_joint_speed: float = 1.0          # rad/s at |a| = 1
    success_radius: float = 0.02          # m
    goal_low: Tuple[float, float, float] = (-0.95, -0.45, 0.35)
    goal_high: Tuple[float, float, float] = (-0.45, 0.45, 0.95)
    bandwidth_hz: float = 8.0
    joint_margin: float = math.radians(10)


class ReachEnv:
    observation_dim = 21
    action_dim = 6

    def __init__(self, cfg: Optional[ReachConfig] = None, model: Optional[mujoco.MjModel] = None, seed: int = 0) -> None:
        self.cfg = cfg or ReachConfig()
        self.m = model if model is not None else make_model()
        self.d = mujoco.MjData(self.m)
        self.rng = np.random.default_rng(seed)
        self.site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "tcp_site")
        self.qadr = np.array([self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS])
        self.dadr = np.array([self.m.jnt_dofadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS])
        self.lo = self.m.jnt_range[[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS], 0] + self.cfg.joint_margin
        self.hi = self.m.jnt_range[[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS], 1] - self.cfg.joint_margin
        self.tau_max = np.array(DynamicsAssumptions().torque_limit)
        self.n_sub = int(round(self.cfg.control_dt / self.m.opt.timestep))
        self._set_q(HOME)
        M = np.zeros((self.m.nv, self.m.nv))
        mujoco.mj_fullM(self.m, M, self.d.qM)
        self.gains = gains_from_inertia(np.diag(M)[self.dadr], self.cfg.bandwidth_hz, 0.9)
        self.goal = np.zeros(3)
        self.q_ref = HOME.copy()
        self.t = 0
        self.succeeded = False

    # ------------------------------------------------------------------ helpers
    def _set_q(self, q) -> None:
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[self.qadr] = q
        mujoco.mj_forward(self.m, self.d)

    def tcp(self) -> np.ndarray:
        return self.d.site_xpos[self.site].copy()

    def q(self) -> np.ndarray:
        return self.d.qpos[self.qadr].copy()

    def obs(self) -> np.ndarray:
        tcp = self.tcp()
        return np.concatenate([self.q(), self.d.qvel[self.dadr], tcp, self.goal, self.goal - tcp])

    def jacobian(self) -> np.ndarray:
        jp = np.zeros((3, self.m.nv))
        mujoco.mj_jacSite(self.m, self.d, jp, None, self.site)
        return jp[:, self.dadr]

    # --------------------------------------------------------------------- API
    def reset(self, seed: Optional[int] = None, goal=None) -> Tuple[np.ndarray, Dict]:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        q0 = HOME + self.rng.uniform(-0.15, 0.15, 6)
        self._set_q(q0)
        self.q_ref = q0.copy()
        self.goal = np.asarray(goal, float) if goal is not None else self.rng.uniform(self.cfg.goal_low, self.cfg.goal_high)
        self.t = 0
        self.succeeded = False
        return self.obs(), {"goal": self.goal.copy()}

    def step(self, action) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        v = a * self.cfg.max_joint_speed
        limit_hit = 0
        for _ in range(self.n_sub):
            self.q_ref = self.q_ref + v * self.m.opt.timestep
            clipped = np.clip(self.q_ref, self.lo, self.hi)
            limit_hit += int(np.any(clipped != self.q_ref))
            self.q_ref = clipped
            q, dq = self.d.qpos[self.qadr], self.d.qvel[self.dadr]
            tau = servo_torque(q, dq, self.q_ref, v * (clipped == self.q_ref), self.d.qfrc_bias[self.dadr], self.gains, self.tau_max)
            self.d.ctrl[:] = tau
            mujoco.mj_step(self.m, self.d)
        self.t += 1
        dist = float(np.linalg.norm(self.goal - self.tcp()))
        reward = -dist - 0.01 * float(a @ a) - 0.1 * (limit_hit > 0)
        if dist < self.cfg.success_radius and not self.succeeded:
            reward += 1.0
            self.succeeded = True
        truncated = self.t >= self.cfg.max_steps
        return self.obs(), reward, False, truncated, {"dist": dist, "limit_hit": limit_hit > 0,
                                                      "success": dist < self.cfg.success_radius}
