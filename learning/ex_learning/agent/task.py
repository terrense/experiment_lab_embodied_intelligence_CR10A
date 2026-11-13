"""The multi-waypoint INSPECTION task, with fault injection (SIMULATION ONLY).

The task
--------
Three Cartesian targets A, B, C must be visited in order. At each one the arm has to hold
within a radius for a number of steps ("inspect"). The out-of-distribution wall is between the
start and the targets, so the arm has to go around it.

Why this task and not a grasp
-----------------------------
It needs no contact model, no vision and no force control, so it is entirely inside the
simulation the platform already has, and it produces a lot of **verifiable** agent decisions:
the agent has to decide, at every step, whether to continue, replan, retreat or give up. The
paper must therefore call it a *multi-stage robotic reaching / inspection* task and not
"manipulation" - the name is part of the honesty.

The faults
----------
The task injects the failures the agent is supposed to learn to recover from. Each is a
**calibrated** fault: it happens with a stated probability and it is recorded in the episode, so
a policy can be scored on the faults it was and was not given:

    plan_failure      the planner cannot find a path (the natural failure rate of the planner on
                      the out-of-distribution walls is already ~16 %, so this is the honest base
                      rate; the injection makes it controllable)
    target_moved      the current target shifts by a random offset
    stale_feedback    the observation handed to the agent is from an older step
    blocked_execution the path's execution is stopped by the safety layer

Everything here is SIMULATION ONLY. The CR10A was not connected.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

#: the inspection radius [m] and the number of steps the arm must hold inside it
INSPECT_RADIUS = 0.02
INSPECT_STEPS = 5

#: The three inspection points are the TCP positions of the episode's own goal configuration
#: with the BASE joint rotated by these angles. Two reasons for this construction:
#:
#:   * they are reachable by construction - the goal configuration is, and a small base rotation
#:     keeps them in the same well-connected region of the workspace. The earlier construction
#:     (a fixed Cartesian OFFSET from the goal) put the targets in a region the planner connects
#:     to in only about two thirds of the attempts, which made the episodes a measure of the
#:     planner's luck rather than of the agent's decisions;
#:   * they are the SAME kind of point as the episode's goal, which is what the planner was
#:     measured on (84 % planning success on the out-of-distribution walls).
#:
#: The angles are in radians. The base joint is J1.
TARGET_BASE_ANGLES = (0.0, 0.26, -0.26)


@dataclass
class FaultSpec:
    """How often each fault fires."""
    plan_failure: float = 0.10
    target_moved: float = 0.15
    stale_feedback: float = 0.10
    blocked_execution: float = 0.05
    max_steps_per_episode: int = 1500         # 30 s at 50 Hz
    max_calls_per_episode: int = 40          # a generous budget: running out of it
                                                     # must be a failure, not the norm


class InspectionTask:
    """The environment of the agent: the wall scene, three targets, and the faults.

    The task owns the MuJoCo wall environment and the episode bookkeeping. The agent sees a
    compact, textual state (the agent is an LLM or a small learned policy, so the observation is
    a small dict of numbers plus a short text rendering).
    """

    def __init__(self, env, n_targets: int = 3, faults: Optional[FaultSpec] = None,
                 rng: Optional[np.random.Generator] = None, plan_failure_override: Optional[float] = None):
        self.env = env
        self.n_targets = n_targets
        self.faults = faults or FaultSpec()
        if plan_failure_override is not None:
            self.faults.plan_failure = plan_failure_override
        self.rng = rng or np.random.default_rng(0)
        self.targets: List[np.ndarray] = []
        self.current_target = 0
        self.hold_steps = 0
        self.steps = 0
        self.calls = 0
        self.faults_fired: List[str] = []
        self.stale_until_step = -1
        self._obs_history: List[Dict] = []

    # ------------------------------------------------------------------ lifecycle
    def reset(self, seed: int) -> Dict:
        """Start an episode and build the three targets around the goal region."""
        obs, info = self.env.reset(seed=seed)
        self.rng = np.random.default_rng(seed)
        self.current_target = 0
        self.hold_steps = 0
        self.steps = 0
        self.calls = 0
        self.faults_fired = []
        self.stale_until_step = -1
        self._obs_history = []

        # The three inspection points. They are CARTESIAN points: the agent gets no privileged
        # joint configuration. (The task DESIGNER does use the episode's goal configuration - it
        # has to choose where the inspections are - but the agent is only ever told the
        # Cartesian positions.)
        q_saved = self.env.q().copy()
        q_goal = np.asarray(getattr(self.env, "q_goal", self.env.q()), float).copy()
        self.targets = []
        self.substituted_targets = 0
        for i in range(self.n_targets):
            angle = TARGET_BASE_ANGLES[i % len(TARGET_BASE_ANGLES)]
            q = q_goal.copy()
            q[0] += angle
            self.env._set_q(q)
            cand = self.env.tcp().copy()
            self.env._set_q(q_saved)
            if not self._reachable(cand):
                self.substituted_targets += 1
                cand = np.asarray(self.env.goal, float).copy()
            self.targets.append(cand)
        return self.observation()

    def _reachable(self, point: np.ndarray, tol: float = 0.01, tries: int = 40) -> bool:
        """Can the arm put its tool point at `point`?

        A DLS IK is a LOCAL method: from the current pose it cannot reach a target on the other
        side of the wall. The check therefore uses the same discipline as the skill layer - a set
        of random restarts - so that "reachable" means what the planner will actually find, not
        what a single local descent finds.
        """
        q_saved = self.env.q().copy()
        qd_saved = self.env.d.qvel[self.env.dadr].copy()
        ref_saved = np.asarray(self.env.q_ref, float).copy()
        rng = np.random.default_rng(4242 + int(abs(hash(tuple(np.round(point, 4)))) % 10_000))
        ok = False
        for i in range(tries):
            q = np.array(q_saved if i == 0 else rng.uniform(self.env.lo, self.env.hi), float)
            self.env._set_q(q)
            for _ in range(250):
                e = point - self.env.tcp()
                if np.linalg.norm(e) < tol:
                    ok = True
                    break
                J = self.env.jacobian()
                try:
                    dq = J.T @ np.linalg.solve(J @ J.T + 0.05 ** 2 * np.eye(3), e)
                except np.linalg.LinAlgError:
                    break
                q = np.clip(q + dq, self.env.lo, self.env.hi)
                self.env._set_q(q)
            if ok:
                break
        self.env._set_q(q_saved)
        self.env.d.qvel[self.env.dadr] = qd_saved
        self.env.q_ref = ref_saved
        return ok

    # ------------------------------------------------------------------ faults
    def maybe_fire(self, name: str) -> bool:
        p = getattr(self.faults, name, 0.0)
        if p > 0 and self.rng.random() < p:
            self.faults_fired.append(name)
            return True
        return False

    def apply_faults(self, skill: str) -> Optional[str]:
        """Apply the faults that are due for this skill call. Returns the FAULT NAME if one fired.

        A fault makes the tool FAIL, exactly as it would in reality: the planner exhausting its
        budget means the plan failed, an interrupted execution means the execution was blocked. A
        fault that only produced a sentence in the failure reason would leave the status 'ok',
        and the policy would learn that failures are free.
        """
        if skill == "plan_motion" and self.maybe_fire("plan_failure"):
            return "plan_failure"
        if skill == "execute_path" and self.maybe_fire("blocked_execution"):
            return "blocked_execution"
        if skill == "inspect_state" and self.maybe_fire("stale_feedback"):
            # a PERCEPTION fault: the tool worked, but the readings the agent gets are old. It
            # must NOT change the tool's status - the agent has to notice the stale readings
            # itself, and the status is what the verifier's "invalid call" count is built on.
            self.stale_until_step = self.steps + 5
            return "stale_feedback"
        if skill in ("plan_motion", "replan") and self.maybe_fire("target_moved"):
            i = self.current_target
            self.targets[i] = self.targets[i] + self.rng.normal(0.0, 0.03, 3)
            return "target_moved"
        return None

    # ------------------------------------------------------------------ observation
    def observation(self) -> Dict:
        """What the agent sees.

        Deliberately small: the clearance (the rigorous lower bound), the distance to the current
        target, the tool-step index, and the state of the last tool call. The text rendering is
        what an LLM policy would be given; the numbers are what a learned policy would be given.
        """
        tgt = self.targets[self.current_target]
        dist = float(np.linalg.norm(tgt - self.env.tcp()))
        clearance = float(self.env.clearance(conservative=True))
        obs = {
            "step": self.steps,
            "call": self.calls,
            "target_index": self.current_target,
            "targets_total": self.n_targets,
            "distance_to_target_m": round(dist, 4),
            "clearance_m": round(clearance, 4),
            "hold_steps": self.hold_steps,
            "last_status": self.last_status if hasattr(self, "last_status") else "",
            "last_failure": self.last_failure if hasattr(self, "last_failure") else "",
            "faults_fired": list(self.faults_fired),
            "text": (f"target {self.current_target + 1}/{self.n_targets}; "
                     f"distance {dist * 100:.0f} mm; clearance {clearance * 1000:.0f} mm; "
                     f"step {self.steps}; "
                     f"last tool: {self.last_status if hasattr(self, 'last_status') else 'none'}"),
        }
        self._obs_history.append(obs)
        if len(self._obs_history) > 64:
            self._obs_history.pop(0)
        return obs

    def observation_for_policy(self) -> Dict:
        """The observation the AGENT receives, with the stale-feedback fault applied.

        The stale feedback means the STATE READINGS are old - the distance and the clearance the
        agent sees are the ones from five steps ago. The tool log is not stale: the agent still
        knows what its last call returned.
        """
        obs = self.observation()
        if self.stale_until_step > self.steps and len(self._obs_history) > 6:
            old = self._obs_history[-6]
            stale = dict(obs)
            stale["distance_to_target_m"] = old["distance_to_target_m"]
            stale["clearance_m"] = old["clearance_m"]
            stale["text"] = (f"target {stale['target_index'] + 1}/{stale['targets_total']}; "
                              f"distance {old['distance_to_target_m'] * 100:.0f} mm; "
                              f"clearance {old['clearance_m'] * 1000:.0f} mm; "
                              f"step {stale['step']}; last tool: {stale['last_status']}")
            return stale
        return obs

    def note_skill(self, result) -> None:
        """Record a tool call's outcome and advance the inspection counter."""
        self.last_status = result.status
        self.last_failure = result.failure_reason
        self.calls += 1
        self.steps += result.steps
        # The inspection counter is about the ARM'S POSITION and the TIME it has been there, not
        # about the tool's status: if the arm is inside the radius of the current target, the
        # inspection is progressing, whatever the last tool call reported. (An earlier version
        # only counted an 'ok' execution, which meant a call that ended in a timeout at the target
        # counted for nothing.)
        #
        # An `inspect_state` call HOLDS the arm still for the inspection duration, so it
        # contributes the whole hold at once; an execution contributes the steps it spent inside
        # the radius.
        if result.skill in ("execute_path", "inspect_state"):
            tgt = self.targets[self.current_target]
            inside = float(np.linalg.norm(tgt - self.env.tcp())) < INSPECT_RADIUS
            if inside:
                self.hold_steps += int(result.data.get("held_steps", 0) or result.steps)
                if self.hold_steps >= INSPECT_STEPS:
                    self.current_target = min(self.current_target + 1, self.n_targets - 1)
                    self.hold_steps = 0

    def done(self) -> bool:
        return (self.current_target >= self.n_targets - 1 and self.hold_steps >= INSPECT_STEPS
                or self.calls >= self.faults.max_calls_per_episode
                or self.steps >= self.faults.max_steps_per_episode)
