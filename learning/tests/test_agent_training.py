"""Tests for the agent's TRAINING pipeline: the SFT dataset and the GRPO loop (SIMULATION ONLY).

These two modules are the parts of the agent line that turn verified episodes into a trained
policy, so the properties that matter are pinned here:

  * the SFT dataset is built ONLY from recorded episodes - the generator never invents a state;
  * the recovery steps are labelled, because the paper's claim is about recovery;
  * the GRPO advantage is group-relative and is ZERO for a degenerate group, not noise;
  * a degenerate group is counted, because the fraction of such groups is what sets the group
    size.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning"))
pytest.importorskip("numpy")

from ex_learning.agent.grpo import (GRPOTrainer, Rollout, StubPolicy,  # noqa: E402
                                    group_relative_advantages, group_statistics)
from ex_learning.agent.sft_data import episodes_to_examples, summarise  # noqa: E402


# ------------------------------------------------------------------------------- the SFT data
def _records():
    return [
        {"episode": 0, "skill_calls": [
            {"skill": "plan_motion", "status": "ok", "target_index": 0, "failure_reason": ""},
            {"skill": "execute_path", "status": "blocked", "target_index": 0,
             "failure_reason": "the safety layer stopped the execution"},
            {"skill": "retreat", "status": "ok", "target_index": 0, "failure_reason": ""},
            {"skill": "execute_path", "status": "ok", "target_index": 0, "failure_reason": ""},
         ], "reward": 0.8, "envelope_contacts": 0, "inspections_done": 1},
        {"episode": 1, "skill_calls": [
            {"skill": "plan_motion", "status": "plan_failed", "target_index": 0,
             "failure_reason": "no collision-free path"},
            {"skill": "replan", "status": "ok", "target_index": 0, "failure_reason": ""},
         ], "reward": -1.0, "envelope_contacts": 1, "inspections_done": 0},
    ]


def test_the_dataset_is_built_only_from_recorded_calls():
    ex = episodes_to_examples(_records())
    assert len(ex) == 6                       # 4 + 2 recorded calls, nothing invented
    for e in ex:
        assert e["completion"].startswith("TOOL: ")
        assert "last tool" in e["prompt"]


def test_the_recovery_steps_are_labelled():
    ex = episodes_to_examples(_records())
    # the call after a blocked execution is a recovery step; the call after a plan failure too
    rec = [e for e in ex if e["is_recovery"]]
    assert len(rec) == 2
    assert rec[0]["completion"] == "TOOL: retreat [0]"
    assert rec[1]["completion"] == "TOOL: replan [0]"


def test_the_failed_calls_can_be_excluded():
    ex = episodes_to_examples(_records(), include_failures=False)
    assert all(e["tool_status"] == "ok" for e in ex)
    assert len(ex) == 4


def test_the_dataset_summary_reports_the_composition():
    s = summarise(episodes_to_examples(_records()))
    assert s["examples"] == 6
    assert s["recovery_examples"] == 2
    assert s["recovery_fraction"] == pytest.approx(2 / 6)
    assert set(s["by_tool"]) == {"plan_motion", "execute_path", "retreat", "replan"}


# -------------------------------------------------------------------------------- the GRPO
def test_the_advantage_is_group_relative():
    adv = group_relative_advantages([1.0, 0.0, -1.0])
    assert adv[0] > 0 > adv[2]
    assert adv[0] == pytest.approx(-adv[2])
    # a uniform shift of the rewards does not change the advantage
    assert group_relative_advantages([2.0, 1.0, 0.0]) == pytest.approx(adv)


def test_a_degenerate_group_has_zero_advantage_not_noise():
    adv = group_relative_advantages([0.5, 0.5, 0.5])
    assert adv == [0.0, 0.0, 0.0]


def test_the_group_statistics_report_the_degeneracy():
    st = group_statistics([0.5, 0.5, 0.5])
    assert st["reward_std"] == 0.0
    st2 = group_statistics([0.0, 1.0])
    assert st2["reward_std"] > 0


def test_the_trainer_collects_a_group_and_trains_on_it():
    policy = StubPolicy(completions=["TOOL: plan_motion [0]", "TOOL: execute_path [0]"])

    def rollout_fn(completion: str) -> Rollout:
        r = Rollout(prompt="", completion=completion)
        r.reward = 1.0 if completion.endswith("plan_motion [0]") else 0.0
        r.reward_terms = {"success": r.reward}
        return r

    t = GRPOTrainer(policy, group_size=4, seed=0)
    rec = t.step("a prompt", rollout_fn)
    assert len(rec["rewards"]) == 4
    assert set(rec["rewards"]) <= {0.0, 1.0}
    assert policy.trained == 1
    # the stub samples from the two completions at random, so the group's mean is whatever the
    # draw gave - what has to hold is that it is the mean of the group, not a fixed value
    assert rec["mean_reward"] == pytest.approx(sum(rec["rewards"]) / 4)
    assert 0.0 <= rec["mean_reward"] <= 1.0


def test_the_trainer_counts_the_degenerate_groups():
    """If every group has the same reward, GRPO has no signal - and the trainer must say so."""
    policy = StubPolicy(completions=["TOOL: inspect_state [0]"])

    def rollout_fn(completion: str) -> Rollout:
        return Rollout(prompt="", completion=completion, reward=0.3)

    t = GRPOTrainer(policy, group_size=4, seed=0)
    out = t.run(["p1", "p2"], rollout_fn, steps=1)
    assert out["degenerate_groups"] == 2
    assert out["mean_reward"] == pytest.approx(0.3)
