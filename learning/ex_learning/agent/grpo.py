"""GRPO / RLVR for the robot agent (SIMULATION ONLY) - the training scaffold of arm A4.

What it does
------------
GRPO (Group Relative Policy Optimization) is the RL stage of the decisive experiment: the
policy is a language model that chooses TOOL CALLS, the environment is the inspection task, and
the reward is the **calibrated verifier score** of section 8 of docs/RESULTS.md - a physical
event, a one-sided proxy and a censoring correction, not a boolean.

For each prompt, the trainer samples a GROUP of trajectories, scores each with the verifier, and
optimises the group-relative advantage:

    A_i = (r_i - mean(r)) / (std(r) + eps)

which is the whole point of GRPO over PPO here: the critic that would be needed to estimate a
baseline for a single trajectory is replaced by the baseline of the group, and every rollout in
the group comes from the SAME state - which is exactly what a deterministic simulator gives.

Why this belongs in this repository
-----------------------------------
The reward is the contribution: a generic robot agent's reward is "did the task succeed", and
its error rate is unknown. Here the reward's properties are MEASURED (the mesh checker is an
independent implementation; the capsule proxy is one-sided; the censoring is quantified), so the
verifier's noise floor is known and the group-relative advantage is computed on a signal whose
quality is characterised. That is the thing the paper is about.

Where it runs
-------------
The policy is a small language model. On the laptop this runs only as a SMOKE TEST against a
stub policy; the real training needs a GPU, and the H20 server is the place for it
(`scripts/remote/train_agent.sh`). The trainer is written against a `Policy` interface so the
same code drives a stub, an API model and a local model.

Everything here is SIMULATION ONLY. The CR10A was not connected.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np


@dataclass
class Rollout:
    """One sampled trajectory of tool calls."""
    prompt: str
    completion: str
    reward: float = 0.0
    reward_terms: Dict = field(default_factory=dict)
    meta: Dict = field(default_factory=dict)


def group_relative_advantages(rewards: Sequence[float], eps: float = 1e-6) -> List[float]:
    """The GRPO advantage: the reward relative to the group's mean.

    A group whose rewards are all equal has no signal, and its advantages are ZERO - not noise.
    That is a property of GRPO, and the trainer reports how many groups had no signal, because a
    high fraction of degenerate groups means the group size is too small for the reward's spread.
    """
    r = np.asarray(rewards, float)
    if r.size == 0:
        return []
    mu = float(r.mean())
    sd = float(r.std())
    if sd < eps:
        return [0.0] * len(r)
    return [float((x - mu) / (sd + eps)) for x in r]


def group_statistics(rewards: Sequence[float]) -> Dict:
    """How informative the groups are - the diagnostic that decides the group size."""
    r = np.asarray(rewards, float)
    if r.size == 0:
        return {"groups": 0}
    spread = float(r.std())
    return {
        "groups": 1,
        "mean_reward": float(r.mean()),
        "reward_std": spread,
        "degenerate_groups": 0,
        "note": "a group whose rewards are all equal carries no GRPO signal; the fraction of "
                "such groups is the diagnostic that sets the group size",
    }


class GRPOTrainer:
    """The GRPO loop, on top of a policy that samples completions.

    The policy interface is deliberately minimal:

        policy.sample(prompt, n) -> list[str]       the n completions of one group
        policy.train_step(examples, advantages)     one gradient step on the group

    so the same trainer drives a stub, an API model (with a replayed dataset) and a local model
    on the compute server.
    """

    def __init__(self, policy, group_size: int = 8, lr: float = 1e-5, seed: int = 0) -> None:
        self.policy = policy
        self.group_size = group_size
        self.lr = lr
        self.rng = np.random.default_rng(seed)
        self.history: List[Dict] = []

    def collect_group(self, prompt: str, rollout_fn: Callable[[str], Rollout]) -> List[Rollout]:
        """Sample a group of trajectories for one prompt and score each of them."""
        group = []
        for completion in self.policy.sample(prompt, self.group_size):
            r = rollout_fn(completion)
            r.prompt = prompt
            r.completion = completion
            group.append(r)
        return group

    def step(self, prompt: str, rollout_fn: Callable[[str], Rollout]) -> Dict:
        """One GRPO step: collect a group, compute the advantages, train."""
        group = self.collect_group(prompt, rollout_fn)
        rewards = [r.reward for r in group]
        adv = group_relative_advantages(rewards)
        stats = group_statistics(rewards)
        examples = [{"prompt": r.prompt, "completion": r.completion, "advantage": a,
                     "reward": r.reward, "reward_terms": r.reward_terms}
                    for r, a in zip(group, adv)]
        self.policy.train_step(examples, adv)
        rec = {"prompt": prompt, "rewards": rewards, "advantages": adv,
               "mean_reward": float(np.mean(rewards)) if rewards else 0.0,
               "reward_std": stats["reward_std"] if rewards else 0.0,
               "degenerate": bool(stats["reward_std"] < 1e-6) if rewards else True}
        self.history.append(rec)
        return rec

    def run(self, prompts: Sequence[str], rollout_fn: Callable[[str], Rollout],
            steps: int = 1) -> Dict:
        """Run the training loop over the prompts."""
        for _ in range(steps):
            for p in prompts:
                self.step(p, rollout_fn)
        rewards = [h["mean_reward"] for h in self.history]
        return {
            "steps": len(self.history),
            "mean_reward": float(np.mean(rewards)) if rewards else 0.0,
            "reward_std": float(np.std(rewards)) if rewards else 0.0,
            "degenerate_groups": sum(1 for h in self.history if h["degenerate"]),
            "history": self.history,
        }


class StubPolicy:
    """A policy that samples from a fixed set of completions.

    It exists so the training loop can be tested without a language model, and so the SFT stage
    has something to be trained on before the model is available. It is NOT a policy for the
    paper.
    """

    def __init__(self, completions: Optional[Sequence[str]] = None, seed: int = 0) -> None:
        self.completions = list(completions or ["TOOL: inspect_state [0]", "TOOL: execute_path [0]",
                                                "TOOL: plan_motion [0]", "TOOL: retreat [0]"])
        self.rng = np.random.default_rng(seed)
        self.trained = 0

    def sample(self, prompt: str, n: int) -> List[str]:
        idx = self.rng.integers(0, len(self.completions), n)
        return [self.completions[i] for i in idx]

    def train_step(self, examples, advantages) -> None:
        self.trained += 1
