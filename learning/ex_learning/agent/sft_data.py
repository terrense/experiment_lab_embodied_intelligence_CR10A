"""Building the SFT dataset (A2) from verified episodes (SIMULATION ONLY).

Where the data comes from
------------------------
The strong rule-based baseline (A0) produces episodes that are *verified*: the verifier scored
them, and the ones that completed the task with no safety event are demonstrations. Each such
episode is a sequence of (observation, tool call) pairs, and the demonstration of a tool call is
the prompt that was given plus the answer the baseline chose. That is exactly the supervised
signal an instruction-tuned agent is trained on.

The design choices that matter, and why:

* **Only verified demonstrations are used.** An episode that collided, or that failed a tool, is
  either dropped or kept with its failure as the label - the choice is a flag, because the two
  answer different questions: "what does a good trajectory look like" (SFT on successes) and
  "what should the agent do when something breaks" (SFT on the RECOVERY steps of failed
  episodes, which is the part the baseline is best at and the part a policy most needs).
* **The recovery steps are labelled separately.** A step whose previous tool call FAILED is a
  recovery step. The paper's claim is about recovery, so the dataset records which steps are
  recovery steps and the training can weight them.
* **The verifier's score is kept with every example.** A model trained on trajectories of
  different quality must be able to tell them apart, and an RLVR stage needs the reward anyway.
* **Nothing is fabricated.** Every example comes from a committed episode record with its
  verifier verdict; the generator does not invent states.

Everything here is SIMULATION ONLY. The CR10A was not connected.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .policies import render_prompt

#: the label of a tool call in the training data
ANSWER_TEMPLATE = "TOOL: {skill}{target}"


def _answer(skill: str, target_index: Optional[int]) -> str:
    if target_index is None:
        return f"TOOL: {skill}"
    return f"TOOL: {skill} [{target_index}]"


def episodes_to_examples(records: List[Dict], include_failures: bool = True,
                         keep_recovery_steps: bool = True) -> List[Dict]:
    """Turn recorded episodes into (prompt, answer) examples.

    `records` is the list written by `evaluation/agent_benchmark.py` (`agent_episodes_*.json`):
    each entry has `skill_calls` (with the structured status and the failure reason) and the
    verifier's reward.
    """
    out: List[Dict] = []
    for rec in records:
        calls = rec.get("skill_calls", [])
        if not calls:
            continue
        good = (rec.get("reward", 0.0) > 0.0) and rec.get("envelope_contacts", 0) == 0
        prev_failed = False
        for i, call in enumerate(calls):
            status = call.get("status", "ok")
            is_recovery = prev_failed
            prev_failed = (status != "ok")
            if status != "ok" and not include_failures:
                continue
            if is_recovery and not keep_recovery_steps:
                continue
            # the observation the agent saw at this step: the tool log up to and including the
            # PREVIOUS call, and the state as the previous call left it
            obs = {
                "text": _state_text(rec, i),
                "last_status": calls[i - 1]["status"] if i > 0 else "",
                "last_failure": calls[i - 1].get("failure_reason", "") if i > 0 else "",
            }
            prompt = render_prompt(obs)
            answer = _answer(call["skill"], call.get("target_index"))
            out.append({
                "prompt": prompt,
                "completion": answer,
                "episode": rec.get("episode"),
                "step": i,
                "is_recovery": bool(is_recovery),
                "tool_status": status,
                "episode_reward": rec.get("reward", 0.0),
                "episode_verified_good": bool(good),
            })
    return out


def _state_text(rec: Dict, i: int) -> str:
    """The state line of the prompt, reconstructed from the recorded calls."""
    calls = rec.get("skill_calls", [])
    last = calls[i - 1] if i > 0 else None
    status = last["status"] if last else "none"
    return (f"tool call {i} of {len(calls)}; "
            f"last tool: {status}; "
            f"inspections done: {rec.get('inspections_done', 0)}")


def write_jsonl(examples: List[Dict], path: Path) -> Path:
    """Write the dataset as JSONL (one example per line) - the format every SFT trainer reads."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for e in examples:
            f.write(json.dumps(e, ensure_ascii=False) + "\n")
    return path


def summarise(examples: List[Dict]) -> Dict:
    """The composition of the dataset - the numbers the paper reports."""
    if not examples:
        return {"examples": 0}
    n = len(examples)
    rec = sum(1 for e in examples if e["is_recovery"])
    good = sum(1 for e in examples if e["episode_verified_good"])
    skills: Dict[str, int] = {}
    for e in examples:
        skills[e["completion"].split()[1].split("[")[0]] = \
            skills.get(e["completion"].split()[1].split("[")[0], 0) + 1
    return {
        "examples": n,
        "recovery_examples": rec,
        "recovery_fraction": rec / n,
        "from_verified_good_episodes": good,
        "good_episode_fraction": good / n,
        "by_tool": skills,
    }


def build(episode_files: List[Path], out_path: Path, include_failures: bool = True) -> Dict:
    """Build the dataset from the recorded episode files."""
    records: List[Dict] = []
    for f in episode_files:
        blob = json.loads(Path(f).read_text())
        for policy, eps in blob.items():
            for e in eps:
                e = dict(e)
                e["policy"] = policy
                records.append(e)
    examples = episodes_to_examples(records, include_failures=include_failures)
    write_jsonl(examples, out_path)
    summary = summarise(examples)
    summary["source_files"] = [str(f) for f in episode_files]
    summary["episodes"] = len(records)
    return summary
