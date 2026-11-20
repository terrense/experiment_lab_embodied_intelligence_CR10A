# Agent exploration (PARKED — not part of the paper)

> **Status: parked on 2026-11-20 by the owner's decision.** The obstacle-avoidance line and the
> hardware get all the resources. This document records what was built and what was measured, so
> that the exploration is on the record instead of implied by silence.
>
> **No number in this document appears in the manuscript, and no claim anywhere rests on it.**
> The code is archived and tested; the hypothesis was never tested, because no learned policy was
> trained.

## What was built

| piece | file | what it is |
|---|---|---|
| the skill layer | `learning/ex_learning/agent/skills.py` | the robot's capabilities as callable tools: `plan_motion`, `execute_path`, `inspect_state`, `replan`, `retreat`, `finish`. Each returns a structured result with a status from a fixed vocabulary (`ok`, `plan_failed`, `unreachable`, `blocked`, `timeout`, `stale`), because an agent has to learn *why* a call failed and the verifier has to score it |
| the task | `learning/ex_learning/agent/task.py` | three Cartesian inspection points, visited in order; the arm must **hold** within 3.5 cm for five control steps. Four fault types with stated rates (`plan_failure`, `target_moved`, `stale_feedback`, `blocked_execution`) and two profiles (the rates the policy is developed against, and the same fault types at different rates) |
| the calibrated verifier | `learning/ex_learning/agent/verifier.py` | scores an episode from the **physics** — a physical event (the independent mesh checker), a one-sided proxy (the capsule metric, which under-estimates the mesh clearance in 1512/1512 measured samples), and the **censoring** correction (the trajectory the agent actually produced, and whether it was stopped early). The safety term is subtracted, never traded against completion |
| the harness | `learning/ex_learning/agent/harness.py` | one loop for every policy: observe → decide → apply the due faults → call the tool → record → verify. The policy never sees the verifier and the verifier never sees the policy's intentions |
| the policies | `learning/ex_learning/agent/policies.py` | A0 the hand-written state machine (the strong baseline), A1 the prompt-only LLM, the scripted policy that is the data source for SFT |
| the training stage | `learning/ex_learning/agent/sft_data.py`, `grpo.py` | the SFT dataset builder (from verified episodes only, with the recovery steps labelled separately) and the GRPO/RLVR loop (group-relative advantage; a degenerate group gets zero advantage, not noise) |
| the runner | `evaluation/agent_benchmark.py` | the policies on the same episodes (paired), with the episodes written as they finish |

## What was measured

Only the **rule-based baseline (A0)**, because the learned policies were never trained. Fifty
episodes per fault profile, the same scenario seeds for every policy:

| fault profile | task completed | inspections | mesh contact | envelope contact | tool calls | invalid calls | reward |
|---|---|---|---|---|---|---|---|
| seen | 52 % | 1.88 / 3 | **0 %** | **0 %** | 27.7 | 15.7 | −15.23 |
| unseen | 48 % | 1.78 / 3 | **0 %** | **0 %** | 28.8 | 16.4 | −16.06 |

Three findings, all of them about the **platform** rather than about learning:

1. **The safety layer holds in every episode** — zero mesh and zero envelope contacts in all 150
   episodes under both fault profiles, the minimum mesh clearance at 71 mm. The obstacle-avoidance
   line's central result carries over: the safety layer is what keeps the arm off the wall.
2. **The dominant failure is the tool, not the policy**: 681 of the failing calls are
   `plan_failed` (741 under the unseen profile) against 64 and 35 `stale`. The planner — the most
   important tool — cannot connect reliably, so the baseline's competence is a property of how
   often the planner succeeds.
3. **The unseen fault profile costs 4 points of completion** (52 % → 48 %) with the same zero
   collisions: the faults are calibrated, they are felt, and they do not break the safety layer.

The baseline also **replicates on the second untouched range** (30000–30149), which is the only
replication the agent line ever got.

## What was NOT tested

* **No learned policy of any kind.** No SFT, no DPO, no GRPO. The training code is written and
  unit-tested, but no training run happened, so there is no A1, A2 or A4 number.
* **The prompt-only arm (A1) was wired to a language model and smoke-tested for connectivity
  only**; it never ran a full episode set. The model key is read from the environment at run time
  and was never committed.
* **No comparison against any learned agent**, and therefore no claim about whether learning
  helps. The hypothesis — that a learned recovery policy beats a hand-written state machine on a
  calibrated risk reward — is **untested**.

## The six defects that running it exposed (worth keeping)

They are all in the *task*, not in the idea, and they are the kind of thing that only shows up
when the thing is actually run:

1. The IK's random restarts came from the arm's **full** joint range, so it returned goal
   realisations with J1 at 294° — configurations the planner can never connect to, because it
   samples J1 within ±90°. The restarts now come from the **planner's** sampling box.
2. An execution's steps were credited as "holding at the target", so the inspection completed the
   moment the arm *arrived* and the target index ran ahead of the arm. **An inspection is a
   hold, not a transit.**
3. The number of completed inspections could not be read off the target index, because that index
   is capped at the last target: a finished task reported two inspections out of three.
4. The inspection radius was **inside the plant's own tracking error** (2 cm against a measured
   3–15 mm servo error and an observed 22–30 mm arrival error), so it could never be met.
5. The state machine did not notice that the target had changed, so it kept following the old
   path while the distance to the new target grew.
6. The planner had no **wall-clock** budget, and a failed 20 000-check search burned minutes: the
   baseline spent 14.5 planning calls per episode with 56 % of them failing. The budget is per
   call and uses a monotonic clock, because the laptop suspends and the wall clock jumps.

Fixing them made the baseline's wall clock drop 50× (59 747 s → 1 155 s for 50 episodes) and
its completion rise from 2 % to 52 %.

## Where the code lives

* `learning/ex_learning/agent/` — the package
* `learning/tests/test_agent.py`, `learning/tests/test_agent_training.py` — 13 + 8 tests
* `evaluation/agent_benchmark.py` — the runner
* `data/reports/learning/agent_benchmark_*.json`, `agent_episodes_*.json` — the measured baseline

The results are in `docs/RESULTS.md` section 14 and the reader of the paper will not encounter
them.
