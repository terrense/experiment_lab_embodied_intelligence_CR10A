"""The agent HARNESS: the loop that turns a policy and a task into a verifiable episode (SIMULATION ONLY).

One loop, for every policy, so that the comparison is controlled by construction:

    observe -> decide -> apply the due faults -> call the tool -> record -> verify

The harness owns the episode record; the verifier scores it afterwards. The policy never sees
the verifier, and the verifier never sees the policy's intentions. That separation is what makes
the reward usable for RLVR and what makes an episode auditable by a reader.

Everything here is SIMULATION ONLY. The CR10A was not connected.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from .skills import SkillLayer
from .task import InspectionTask
from .verifier import EpisodeRecord, Verifier

#: how many control steps each skill is allowed to take
EXECUTE_STEPS = 1500
RETREAT_STEPS = 120


def _boxes_from_env(env) -> List[Dict]:
    """The scene boxes AS THEY ARE for the current episode (the wall moves every reset)."""
    out = []
    for i, name in enumerate(env.box_names):
        lo, hi = np.asarray(env.box_lo[i], float), np.asarray(env.box_hi[i], float)
        out.append({"name": name, "center": 0.5 * (lo + hi), "size": hi - lo})
    return out


def run_episode(env, task: InspectionTask, skills: SkillLayer, policy, seed: int,
                episode: int = 0) -> EpisodeRecord:
    """Run one episode and return its record."""
    t0 = time.time()
    obs = task.reset(seed)
    policy.reset(obs)
    rec = EpisodeRecord(episode=episode, seed=seed, inspections_required=task.n_targets)
    rec.boxes = _boxes_from_env(env)
    envelope_contacts = 0
    was_collided = False

    while not task.done():
        agent_obs = task.observation_for_policy()
        action = policy.act(agent_obs)
        skill_name = action.get("skill", "inspect_state")
        target_index = action.get("target_index")
        if target_index is not None:
            task.current_target = int(np.clip(target_index, 0, task.n_targets - 1))

        # the faults that are due for this call
        note = task.apply_faults(skill_name)

        if skill_name == "plan_motion":
            result = skills.plan_motion(task.targets[task.current_target])
        elif skill_name == "execute_path":
            result = skills.execute_path(max_steps=EXECUTE_STEPS)
        elif skill_name == "replan":
            skills.path = None
            result = skills.plan_motion(task.targets[task.current_target])
        elif skill_name == "retreat":
            result = skills.retreat(max_steps=RETREAT_STEPS)
        elif skill_name == "inspect_state":
            result = skills.inspect_state()
        elif skill_name == "finish":
            result = skills.finish()
        else:
            result = skills.inspect_state()
            result.status = "timeout"
            result.failure_reason = f"unknown skill {skill_name!r}"

        if note:
            result.failure_reason = (result.failure_reason + "; " + note).strip("; ")
        task.note_skill(result)
        rec.skill_calls.append(result.to_dict())

        # the trajectory: every control step the arm actually took
        rec.trajectory.extend(skills.take_trajectory())
        collided = bool(env.collided)
        if collided and not was_collided:
            envelope_contacts += 1
            rec.steps_after_first_contact = max(rec.steps_after_first_contact,
                                                 task.steps - rec.steps)
        was_collided = collided

        if skill_name == "finish":
            break

    rec.envelope_contacts = envelope_contacts
    # the inspections completed: the number of targets the arm has reached AND held
    rec.inspections_done = min(task.current_target + (1 if task.hold_steps >= 1 else 0),
                                task.n_targets)
    rec.steps = task.steps
    rec.wall_s = time.time() - t0
    rec.faults = list(task.faults_fired)
    return rec


def run_benchmark(policy_factory, task_factory, episodes: int = 50, seed0: int = 20000,
                  verifier: Optional[Verifier] = None) -> Dict:
    """Run `episodes` episodes with one policy and score them.

    The episodes are the same for every policy (the same seeds), so the comparison is paired.
    """
    if verifier is None:
        verifier = Verifier()
    verdicts = []
    for ep in range(episodes):
        env, task, skills = task_factory()
        policy = policy_factory()
        rec = run_episode(env, task, skills, policy, seed0 + ep, episode=ep)
        verdicts.append(verifier.evaluate(rec))
    return {"episodes": episodes, "seed_start": seed0, "verdicts": verdicts}
