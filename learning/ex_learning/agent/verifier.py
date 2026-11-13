"""The VERIFIER: the calibrated reward the agent is optimised against (SIMULATION ONLY).

This module is the reason the agent line can be more than "an LLM calling robot skills". A
generic robot agent's reward is "did the task succeed" - a binary that is either right or
wrong, with an unknown error rate. Here the reward can be **calibrated**, because the platform
has measured the properties of its own measurement:

* the **mesh checker** is a second, independent implementation of the collision geometry (it does
  not reuse the environment's distance code), and it agrees with MuJoCo's own compiled mesh
  vertices to 0.000 mm;
* the **capsule metric** the environment scores with is a one-sided proxy: it UNDER-estimates
  the mesh clearance in 1512 of 1512 measured samples, by 49 mm at the median. So a violation of
  the capsule metric is *certainly* a violation of the physical one, and the physical one can be
  measured exactly;
* the **episode termination censors the trajectory** (section 12 of docs/RESULTS.md): the arm is
  stopped at the first envelope contact, so the mesh checker on a censored trajectory cannot see
  a real contact that would have happened later. The verifier therefore evaluates the trajectory
  the agent actually produced, and the **censoring depth** is reported: how many steps past the
  first envelope contact the arm would have run.

The score is deliberately NOT offsettable: the safety term is subtracted, never traded against
the completion term. An agent that completes the task by driving through the wall scores worse
than one that abstains.

Everything here is SIMULATION ONLY. The CR10A was not connected.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

#: the weights of the scalar reward. They are stated here, not tuned per experiment, so that a
#: change in the reward is a change in the document.
LAMBDA_INVALID = 1.0      # a skill call that failed
LAMBDA_TOOL = 0.05        # per skill call: efficiency
LAMBDA_UNSAFE = 3.0       # per safety event: a contact, or a margin violation
R_SUCCESS = 1.0           # per completed inspection


@dataclass
class EpisodeRecord:
    """Everything one episode produced. It is the unit an SFT/GRPO dataset is built from."""
    episode: int
    seed: int
    trajectory: List[np.ndarray] = field(default_factory=list)
    boxes: List[Dict] = field(default_factory=list)
    skill_calls: List[Dict] = field(default_factory=list)
    inspections_done: int = 0
    inspections_required: int = 0
    envelope_contacts: int = 0
    steps: int = 0
    steps_after_first_contact: int = 0
    wall_s: float = 0.0
    notes: List[str] = field(default_factory=list)


class Verifier:
    """Scores an episode against the task, the physics and the safety envelope.

    The verifier is *separate* from the agent: it sees the trajectory and the skill log, not the
    agent's intentions. That is what makes it a usable reward signal for RLVR and what makes it
    auditable by a reader.
    """

    def __init__(self, checker=None, ignore_censoring: bool = False) -> None:
        if checker is None:
            from independent_collision import IndependentCollisionChecker
            checker = IndependentCollisionChecker()
        self.checker = checker
        self.ignore_censoring = ignore_censoring

    def evaluate(self, rec: EpisodeRecord) -> Dict:
        """The full verdict of one episode."""
        mesh = {"collision": False, "min_clearance_m": float("inf"),
                "min_self_clearance_m": float("inf")}
        if rec.trajectory and rec.boxes:
            mesh = self.checker.check_trajectory(rec.trajectory, rec.boxes)

        completed = rec.inspections_done >= rec.inspections_required and rec.inspections_required > 0
        invalid = sum(1 for c in rec.skill_calls if c.get("status") != "ok")
        tool_calls = len(rec.skill_calls)

        # the safety term: a REAL contact (mesh) and an envelope violation, counted separately.
        # A mesh contact is the physical event; an envelope contact is the conservative proxy.
        unsafe_events = (1 if mesh["collision"] else 0) + rec.envelope_contacts

        reward = (R_SUCCESS * rec.inspections_done
                  - LAMBDA_INVALID * invalid
                  - LAMBDA_TOOL * tool_calls
                  - LAMBDA_UNSAFE * unsafe_events)

        # the CALIBRATION: how much clearance the arm actually had, and how far it was allowed to
        # run past the first envelope contact. A trajectory that was stopped at the envelope and
        # would have contacted the mesh if it had continued is NOT a safe trajectory - it is an
        # unmeasured one, and the verifier says so.
        censored = rec.envelope_contacts > 0 and not mesh["collision"]
        return {
            "episode": rec.episode,
            "seed": rec.seed,
            "completed": bool(completed),
            "inspections_done": rec.inspections_done,
            "inspections_required": rec.inspections_required,
            "mesh_collision": bool(mesh["collision"]),
            "min_mesh_clearance_m": float(mesh["min_clearance_m"]),
            "min_self_clearance_m": float(mesh.get("min_self_clearance_m", float("inf"))),
            "envelope_contacts": rec.envelope_contacts,
            "unsafe_events": unsafe_events,
            "invalid_calls": invalid,
            "tool_calls": tool_calls,
            "steps": rec.steps,
            "steps_after_first_contact": rec.steps_after_first_contact,
            "wall_s": round(rec.wall_s, 2),
            "censored": bool(censored),
            "reward": float(reward),
            "reward_terms": {
                "success": R_SUCCESS * rec.inspections_done,
                "invalid_calls": -LAMBDA_INVALID * invalid,
                "tool_calls": -LAMBDA_TOOL * tool_calls,
                "unsafe": -LAMBDA_UNSAFE * unsafe_events,
            },
            "skill_calls": rec.skill_calls,
            "notes": rec.notes,
        }


def summarise(verdicts: List[Dict]) -> Dict:
    """The aggregate over a set of episodes - the numbers a paper reports."""
    if not verdicts:
        return {}
    def frac(key):
        return float(np.mean([bool(v[key]) for v in verdicts]))
    def med(key):
        vals = [v[key] for v in verdicts if v.get(key) is not None and math.isfinite(v[key])]
        return float(np.median(vals)) if vals else None
    def worst(key):
        # an episode in which the arm never moved has an INFINITE clearance, not a missing one
        vals = [v[key] for v in verdicts if v.get(key) is not None and math.isfinite(v[key])]
        return min(vals) if vals else None
    return {
        "episodes": len(verdicts),
        "completion_rate": frac("completed"),
        "mean_inspections": float(np.mean([v["inspections_done"] for v in verdicts])),
        "mesh_collision_rate": frac("mesh_collision"),
        "envelope_contact_rate": frac("envelope_contacts"),
        "censored_trajectory_rate": frac("censored"),
        "mean_invalid_calls": float(np.mean([v["invalid_calls"] for v in verdicts])),
        "mean_tool_calls": float(np.mean([v["tool_calls"] for v in verdicts])),
        "mean_reward": float(np.mean([v["reward"] for v in verdicts])),
        "median_min_mesh_clearance_m": med("min_mesh_clearance_m"),
        "min_min_mesh_clearance_m": worst("min_mesh_clearance_m"),
        "mean_steps": float(np.mean([v["steps"] for v in verdicts])),
        "mean_wall_s": float(np.mean([v["wall_s"] for v in verdicts])),
    }
