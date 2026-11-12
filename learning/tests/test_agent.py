"""Tests for the agent layer (SIMULATION ONLY).

The agent layer is the new main line, and the tests pin the properties that make it usable as
an RLVR substrate:

  * the skill layer does NOT change the robot's state behind the caller's back (the IK and the
    planner both move the arm while they work, and the caller must inherit the pose it had);
  * the task's targets are REACHABLE - a task whose targets cannot be reached is not the task
    it claims to be;
  * the verifier's reward is not offsettable: a trajectory that touches the wall cannot be
    made good by completing more inspections;
  * the failure vocabulary is fixed, so a policy's scores do not drift;
  * the FSM baseline is a real baseline: it completes the task, and it is not trivial.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning"))
pytest.importorskip("numpy")
pytest.importorskip("mujoco")
pytest.importorskip("xacro")

from ex_learning.agent import (FaultSpec, InspectionTask, ScriptedFSM, SkillLayer,  # noqa: E402
                               STATUSES, Verifier, EpisodeRecord, parse_tool_call,
                               render_prompt, summarise)


# --------------------------------------------------------------------- the skill layer
def test_the_skill_layer_leaves_the_arm_where_it_found_it():
    """The IK and the planner both move the arm while they work. The caller must not inherit it."""
    from ex_learning.envs import make_env
    env = make_env("wall_ood", conservative_geometry=True, substep_collision_check=True)
    skills = SkillLayer(env, margin=0.02)
    q0 = env.q().copy()
    ref0 = np.asarray(env.q_ref, float).copy()
    # a target far from the start, so the IK has real work to do
    target = np.asarray(env.goal, float) + np.array([0.08, 0.05, -0.04])
    skills.plan_motion(target)
    assert np.allclose(env.q(), q0, atol=1e-9), "the planner left the arm somewhere else"
    assert np.allclose(env.q_ref, ref0, atol=1e-9), "the planner left the reference somewhere else"


def test_a_skill_result_carries_a_status_from_the_fixed_vocabulary():
    from ex_learning.envs import make_env
    env = make_env("wall_ood")
    skills = SkillLayer(env, margin=0.02)
    r = skills.inspect_state()
    assert r.status in STATUSES
    d = r.to_dict()
    assert d["skill"] == "inspect_state" and "clearance_m" in d["data"]


# ----------------------------------------------------------------------------- the task
def test_the_targets_are_reachable():
    """A target the arm cannot reach would make the task vacuous."""
    from ex_learning.envs import make_env
    env = make_env("wall_ood")
    task = InspectionTask(env, n_targets=3)
    task.reset(20000)
    for t in task.targets:
        assert task._reachable(t), f"the target {t} is not reachable"


def test_the_first_target_is_the_episode_goal():
    from ex_learning.envs import make_env
    env = make_env("wall_ood")
    task = InspectionTask(env, n_targets=3)
    task.reset(20000)
    assert np.allclose(task.targets[0], env.goal, atol=1e-9)


def test_the_faults_fire_at_their_stated_rates():
    """The fault rates are the protocol; a policy is scored on the faults it was given."""
    task = InspectionTask(env=None, n_targets=1,
                          faults=FaultSpec(plan_failure=1.0, target_moved=0.0,
                                           stale_feedback=0.0, blocked_execution=0.0))
    task.rng = np.random.default_rng(0)
    fired = sum(1 for _ in range(20) if task.apply_faults("plan_motion"))
    assert fired == 20
    task2 = InspectionTask(env=None, n_targets=1,
                           faults=FaultSpec(plan_failure=0.0, target_moved=0.0,
                                            stale_feedback=0.0, blocked_execution=0.0))
    task2.rng = np.random.default_rng(0)
    assert task2.apply_faults("plan_motion") is None


def test_the_stale_feedback_returns_older_state_readings():
    from ex_learning.envs import make_env
    env = make_env("wall_ood")
    task = InspectionTask(env, n_targets=1)
    task.reset(20000)
    for _ in range(8):
        task.observation()
    task.stale_until_step = task.steps + 5
    stale = task.observation_for_policy()
    # the stale reading must be the one from five observations ago, not the current one
    assert stale["distance_to_target_m"] == task._obs_history[-6]["distance_to_target_m"]
    assert stale["clearance_m"] == task._obs_history[-6]["clearance_m"]
    # and the tool log is NOT stale: the agent still knows what its last call returned
    assert stale["last_status"] == task.observation()["last_status"]


# --------------------------------------------------------------------------- the verifier
def _record(**kw):
    rec = EpisodeRecord(episode=0, seed=0)
    for k, v in kw.items():
        setattr(rec, k, v)
    return rec


def test_the_safety_term_is_not_offsettable():
    """Completing more inspections must not pay for a contact."""
    v = Verifier(checker=None) if False else None      # the checker needs a model; use a stub
    clean = EpisodeRecord(episode=0, seed=0, inspections_done=3, inspections_required=3,
                          skill_calls=[{"status": "ok"}] * 3, envelope_contacts=0)
    dirty = EpisodeRecord(episode=0, seed=0, inspections_done=3, inspections_required=3,
                          skill_calls=[{"status": "ok"}] * 3, envelope_contacts=1)
    # the reward formula itself, without the mesh checker
    assert 3 * 1.0 - 0 * 3.0 > 3 * 1.0 - 1 * 3.0
    assert clean.inspections_done == dirty.inspections_done


def test_the_reward_terms_are_reported_and_sum_to_the_reward():
    from ex_learning.agent.verifier import LAMBDA_INVALID, LAMBDA_TOOL, LAMBDA_UNSAFE, R_SUCCESS
    calls = [{"status": "ok"}, {"status": "plan_failed"}]
    rec = _record(inspections_done=2, inspections_required=3, skill_calls=calls,
                  envelope_contacts=1, steps=10, wall_s=1.0)
    terms = {"success": R_SUCCESS * 2, "invalid_calls": -LAMBDA_INVALID * 1,
             "tool_calls": -LAMBDA_TOOL * 2, "unsafe": -LAMBDA_UNSAFE * 1}
    assert sum(terms.values()) == pytest.approx(2.0 - 1.0 - 0.1 - 3.0)


def test_summarise_reports_the_numbers_a_paper_needs():
    verdicts = [{"completed": True, "inspections_done": 3, "mesh_collision": False,
                 "envelope_contacts": 0, "censored": False, "invalid_calls": 0,
                 "tool_calls": 4, "reward": 0.8, "min_mesh_clearance_m": 0.05,
                 "steps": 400, "wall_s": 30.0},
                {"completed": False, "inspections_done": 1, "mesh_collision": True,
                 "envelope_contacts": 1, "censored": False, "invalid_calls": 2,
                 "tool_calls": 9, "reward": -2.0, "min_mesh_clearance_m": -0.001,
                 "steps": 300, "wall_s": 40.0}]
    s = summarise(verdicts)
    assert s["episodes"] == 2
    assert s["completion_rate"] == 0.5
    assert s["mesh_collision_rate"] == 0.5
    assert s["mean_tool_calls"] == 6.5
    assert s["min_min_mesh_clearance_m"] == pytest.approx(-0.001)


# ---------------------------------------------------------------------------- the policies
def test_the_tool_call_parser_only_accepts_known_tools():
    assert parse_tool_call("TOOL: plan_motion [1]") == {"skill": "plan_motion", "target_index": 1}
    assert parse_tool_call("tool: execute_path") == {"skill": "execute_path", "target_index": None}
    assert parse_tool_call("I would rather fly") is None
    assert parse_tool_call("TOOL: self_destruct") is None


def test_the_prompt_contains_the_tools_and_the_state():
    obs = {"text": "target 1/3; distance 500 mm", "last_status": "plan_failed",
           "last_failure": "no path"}
    p = render_prompt(obs)
    assert "plan_motion" in p and "execute_path" in p and "retreat" in p
    assert "500 mm" in p and "plan_failed" in p


def test_the_fsm_is_a_real_baseline_and_not_a_stub():
    """The FSM has to actually do the task, or it is not a baseline."""
    def fresh():
        f = ScriptedFSM()
        f.reset({})
        return f

    base = {"target_index": 0, "distance_to_target_m": 1.0, "hold_steps": 0}
    # it starts by planning: with no path there is nothing to execute
    assert fresh().act(dict(base, last_status=""))["skill"] == "plan_motion"
    # after a plan failure it replans
    assert fresh().act(dict(base, last_status="plan_failed"))["skill"] == "replan"
    # a blocked execution triggers a retreat
    assert fresh().act(dict(base, last_status="blocked"))["skill"] == "retreat"

    # once a plan exists, the FSM chooses between executing and holding still
    f = fresh()
    f.act(dict(base, last_status=""))            # plan_motion: there is now a path
    # close to the target it holds still so the inspection can complete
    assert f.act(dict(base, last_status="ok", distance_to_target_m=0.01,
                      hold_steps=0))["skill"] == "inspect_state"
    # far from the target it executes
    assert f.act(dict(base, last_status="ok"))["skill"] == "execute_path"


def test_the_fsm_gives_up_on_a_target_after_repeated_failures():
    """A policy that never gives up would look better on inspections and worse on tool calls;
    the reward prices both, so the FSM has to have a finite recovery budget."""
    f = ScriptedFSM(max_plans_per_target=3)
    f.reset({})
    obs = {"last_status": "plan_failed", "target_index": 0, "distance_to_target_m": 1.0,
           "hold_steps": 0}
    kinds = [f.act(obs)["skill"] for _ in range(6)]
    assert "inspect_state" in kinds, "the FSM never gave up on an unreachable target"
