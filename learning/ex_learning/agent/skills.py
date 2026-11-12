"""The SKILL layer: the robot's capabilities as callable, verifiable tools (SIMULATION ONLY).

Why a skill layer
-----------------
The control line measures things a high-level agent needs: how often a safety claim fails, what
stopping costs, how biased the collision metric is. This module turns the existing stack into
the **tools** such an agent calls, and it is deliberately thin: every skill is a wrapper around
code that already exists and is already measured.

The skills, and what each one returns:

    plan_motion(target)   plan a path to a CARTESIAN target; the goal is realised into a joint
                          configuration by a damped-least-squares IK, exactly as a deployed
                          planner would. Returns the path, the planning time, and whether the
                          plan succeeded.
    execute_path()        follow the current path under the plant's dynamics, with the safety
                          filter active. Returns the number of steps taken and the outcome.
    inspect_state()       read the state: the clearance (rigorous lower bound), the mesh
                          clearance, the filter's own claim, whether the filter intervened.
    replan()              throw the path away and plan again for the current target.
    retreat()             back off along the last path, away from the obstacle.
    finish()              declare the task complete.

Every skill returns a **structured result** with a machine-readable `status` and a
`failure_reason`, because the agent has to be able to learn *why* a call failed, and the
verifier has to be able to score it. The failure vocabulary is fixed and small:

    ok              the skill did what it was asked
    plan_failed     no collision-free path was found within the budget
    unreachable     the target cannot be realised by the arm at all
    blocked         the execution was stopped by the safety layer
    timeout         the skill ran out of its step budget
    stale           the target or the feedback moved under the agent

Everything here is SIMULATION ONLY. The CR10A was not connected.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

#: the fixed failure vocabulary. An agent's policy is scored on these, so they must not drift.
STATUSES = ("ok", "plan_failed", "unreachable", "blocked", "timeout", "stale")


@dataclass
class SkillResult:
    """What a skill returns. `status` is one of `STATUSES`."""
    skill: str
    status: str = "ok"
    failure_reason: str = ""
    data: Dict = field(default_factory=dict)
    steps: int = 0
    seconds: float = 0.0

    def to_dict(self) -> Dict:
        return {"skill": self.skill, "status": self.status, "failure_reason": self.failure_reason,
                "steps": self.steps, "seconds": round(self.seconds, 3), "data": self.data}

    @property
    def ok(self) -> bool:
        return self.status == "ok"


class SkillLayer:
    """The robot's capabilities, as callable tools.

    The layer owns the plan and the waypoint follower; the caller (the agent) only chooses
    *what* to call. The safety filter is applied inside `execute_path`, because a skill that can
    be called without a safety layer is not a skill this platform can offer.
    """

    def __init__(self, env, margin: float = 0.02, plan_margin: Optional[float] = None,
                 plan_checks: int = 20_000,
                 plan_candidates: int = 4, plan_speed: float = 0.6,
                 use_filter: bool = True, exec_model=None) -> None:
        self.env = env
        # the execution filter's margin, and the planner's margin. The planner must keep MORE
        # clearance than the filter enforces: the execution lags the plan (the execution gap this
        # project measured), so a path planned at the filter's own margin is stopped short of its
        # end by that same filter. The planner therefore plans at twice the filter margin unless a
        # value is given.
        self.margin = margin
        self.plan_margin = plan_margin if plan_margin is not None else 2.0 * margin
        self.plan_checks = plan_checks
        self.plan_candidates = plan_candidates
        self.plan_speed = plan_speed
        self.use_filter = use_filter
        self.exec_model = exec_model
        self.path: Optional[List[np.ndarray]] = None
        self.target: Optional[np.ndarray] = None
        self.plan_time_s: float = 0.0
        self.last_plan_status: str = "ok"
        #: the joint trajectory of the steps taken since the last `take_trajectory()` call. The
        #: verifier needs the WHOLE trajectory, not just the end state, so every skill that moves
        #: the arm appends to it.
        self.trajectory: List[np.ndarray] = []

    def take_trajectory(self) -> List[np.ndarray]:
        """The trajectory since the last call, and clears the buffer."""
        out = list(self.trajectory)
        self.trajectory = []
        return out

    # ------------------------------------------------------------------ planning
    def realise_goal(self, cartesian_goal: np.ndarray, tries: int = 60,
                     steps: int = 250, tol: float = 0.002
                     ) -> List[Tuple[float, float, np.ndarray]]:
        """Realise a CARTESIAN goal into joint configurations with a DLS IK.

        The same routine the B6 experiment uses, and the same discipline: the agent is given the
        Cartesian target only, never a privileged joint configuration.

        Every candidate is returned with its IK error AND its clearance, because a goal
        configuration that is *inside* the wall cannot be planned to: the planner rejects it
        immediately. Only the collision-free realisations are usable, and the caller must not
        assume the most accurate one is usable.

        The arm is left EXACTLY where it was: the IK moves the arm around, and the caller must
        not inherit that.
        """
        q_saved = self.env.q().copy()
        qd_saved = self.env.d.qvel[self.env.dadr].copy()
        ref_saved = np.asarray(self.env.q_ref, float).copy()
        out: List[Tuple[float, float, np.ndarray]] = []
        rng = np.random.default_rng(1000 + int(abs(hash(tuple(np.round(cartesian_goal, 4)))) % 10_000))
        for i in range(tries):
            q = np.array(q_saved if i == 0 else rng.uniform(self.env.lo, self.env.hi), float)
            self.env._set_q(q)
            for _ in range(steps):
                e = cartesian_goal - self.env.tcp()
                if np.linalg.norm(e) < tol:
                    break
                J = self.env.jacobian()
                try:
                    dq = J.T @ np.linalg.solve(J @ J.T + 0.05 ** 2 * np.eye(3), e)
                except np.linalg.LinAlgError:
                    break
                q = np.clip(q + dq, self.env.lo, self.env.hi)
                self.env._set_q(q)
            err = float(np.linalg.norm(cartesian_goal - self.env.tcp()))
            out.append((err, float(self.env.clearance()), q.copy()))
        # restore the arm exactly
        self.env._set_q(q_saved)
        self.env.d.qvel[self.env.dadr] = qd_saved
        self.env.q_ref = ref_saved
        out.sort(key=lambda t: t[0])
        return out

    def plan_motion(self, cartesian_goal: np.ndarray) -> SkillResult:
        """Plan a path to a Cartesian target. Returns `plan_failed` when no path is found."""
        from ex_learning.planner import JointRRTConnect

        t0 = time.time()
        self.target = np.asarray(cartesian_goal, float).copy()
        q_start = self.env.q().copy()
        cands = self.realise_goal(self.target)
        if not cands:
            return SkillResult("plan_motion", "unreachable", "the target could not be realised",
                               {})
        best_err = cands[0][0]
        if best_err > 0.02:
            return SkillResult("plan_motion", "unreachable",
                               f"the target could not be realised by the arm (error {best_err:.3f} m)",
                               {"ik_error_m": best_err})
        # only the collision-free realisations can be planned to
        free = [c for c in cands if c[1] > self.plan_margin and c[0] < 0.02]
        rng = np.random.default_rng(1000 + int(abs(hash(tuple(np.round(self.target, 4)))) % 10_000))
        for _err, _clr, q_goal in free[:self.plan_candidates]:
            planner = JointRRTConnect(self.env, margin=self.plan_margin, step=0.15, edge_res=0.015,
                                      max_checks=self.plan_checks, rng=rng)
            path = planner.plan(q_start, q_goal)
            if path is not None:
                self.path = [np.asarray(p, float) for p in path]
                self.plan_time_s = time.time() - t0
                self.last_plan_status = "ok"
                self.env._set_q(q_start)
                self.env.q_ref = q_start.copy()
                return SkillResult("plan_motion", "ok", "",
                                   {"waypoints": len(self.path), "plan_s": round(self.plan_time_s, 2),
                                    "ik_error_m": round(_err, 5),
                                    "free_realisations": len(free)}, seconds=self.plan_time_s)
        self.path = None
        self.plan_time_s = time.time() - t0
        self.last_plan_status = "plan_failed"
        # the planner MOVED the arm while it searched (it checks configurations in place), so the
        # start pose has to be restored before the caller inherits it
        self.env._set_q(q_start)
        self.env.q_ref = q_start.copy()
        return SkillResult("plan_motion", "plan_failed",
                           "no collision-free path was found within the planning budget",
                           {"plan_s": round(self.plan_time_s, 2), "free_realisations": len(free)},
                           seconds=self.plan_time_s)

    # ------------------------------------------------------------------ execution
    def execute_path(self, max_steps: int = 300) -> SkillResult:
        """Follow the current path. The safety filter is active unless `use_filter` is False."""
        from ex_learning.planner import path_follower
        from ex_learning.safety_filter import cbf_policy_qp

        if self.path is None:
            return SkillResult("execute_path", "plan_failed", "there is no path to follow")

        env = self.env
        follower = path_follower(self.path, env, speed=self.plan_speed)
        policy = follower
        if self.use_filter:
            policy = cbf_policy_qp(env, follower, margin=self.margin, alpha=5.0, rho=0.0)
        obs = env.obs()
        collided = False
        steps = 0
        for _ in range(max_steps):
            a = np.asarray(policy(obs), float)
            self.trajectory.append(env.q().copy())
            obs, r, term, trunc, info = env.step(a)
            self.trajectory.append(env.q().copy())
            steps += 1
            collided = collided or bool(info.get("collision", False))
            if collided:
                return SkillResult("execute_path", "blocked",
                                   "the safety layer stopped the execution: the envelope was hit",
                                   {"collision": True}, steps=steps)
            if term or trunc:
                break
            # the path is done when the last waypoint is reached
            if np.max(np.abs(self.path[-1] - env.q())) < 0.02:
                break
        reached = float(np.linalg.norm(self.path[-1] - env.q()))
        status = "ok" if reached < 0.05 else "timeout"
        return SkillResult("execute_path", status,
                           "" if status == "ok" else f"the path was not finished ({reached:.3f} m left)",
                           {"final_waypoint_error_m": round(reached, 4)}, steps=steps)

    def retreat(self, max_steps: int = 120, speed: float = 0.4) -> SkillResult:
        """Move away from the closest obstacle, along the negative of the clearance gradient."""
        import mujoco
        env = self.env
        jp = np.zeros((3, env.m.nv))
        obs = env.obs()
        collided = False
        steps = 0
        for _ in range(max_steps):
            pts = env.repulsion_points(0.30)
            if not pts:
                break
            d, p, n, body = min(pts, key=lambda t: t[0])
            mujoco.mj_jac(env.m, env.d, jp, None, p, body)
            c = n @ jp[:, env.dadr]
            a = np.zeros(6)
            if np.linalg.norm(c) > 1e-9:
                a = c / np.linalg.norm(c) * min(speed / env.cfg.max_joint_speed, 1.0)
            self.trajectory.append(env.q().copy())
            obs, r, term, trunc, info = env.step(a)
            self.trajectory.append(env.q().copy())
            steps += 1
            collided = collided or bool(info.get("collision", False))
            if collided or term or trunc:
                break
        return SkillResult("retreat", "ok" if not collided else "blocked",
                           "" if not collided else "the retreat itself was stopped by the envelope",
                           {"min_clearance_m": float(env.clearance(conservative=True))}, steps=steps)

    def inspect_state(self) -> SkillResult:
        """Read the state: the clearance, the filter's claim and whether it intervened."""
        d = float(self.env.clearance(conservative=True))
        return SkillResult("inspect_state", "ok", "",
                           {"clearance_m": round(d, 4),
                            "tcp": [round(float(v), 4) for v in self.env.tcp()],
                            "has_path": self.path is not None,
                            "target": None if self.target is None
                            else [round(float(v), 4) for v in self.target]})

    def finish(self, note: str = "") -> SkillResult:
        return SkillResult("finish", "ok", note, {"waypoints": 0 if self.path is None else len(self.path)})
