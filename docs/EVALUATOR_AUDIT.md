# Week-1 gate: is the published M5 table an artefact of the evaluator?

> **Status 2026-11-05.** The audit of the 2026-11-02 external review is complete: every alleged
> defect was independently re-derived, the confirmed ones are fixed with regression tests, and the
> legacy-vs-corrected comparison over the historical dev set has been run. **The gate passes**:
> the published table is not an artefact of the evaluator. One measurement is still running
> (the independent mesh-based checker) and is reported below as far as it has been verified.

---

## 1. How the audit was done

The review that started this (`CR10A_Research_Design_CN.md`, 2026-11-02) listed six P0 and four P1
defects. **No claim was taken on trust.** Each was re-derived from the code by an independent
agent that had to produce a runnable reproduction, and the ones that mattered were measured
against an exact reference (not against the code under test).

Two of the agent measurements had to be re-done by hand, because they contradicted a conclusion I
had already written down:

- P0-C: I had written that the sampling error "does not flip the sign" on the real CR10A
  capsules. The exact-reference measurement says it **does**, in 0.33 % of near-boundary
  configurations. My conclusion was too strong and is corrected below and in `RESEARCH_PLAN.md`.
- P1-c: I had measured a 3–5 percentage-point change in the held-out VAF. That was an artefact of
  a transcription error of my own (see §5), not of the leakage. The real difference is ≤ 0.1 points.

The harness used for the audit was validated by **reproducing the archived JSON exactly**:
running the archived protocol on `wall_ood`, 50 episodes from seed 1000, gives success 0.30,
collision 0.04, final distance 0.0289 m — the numbers in `data/reports/learning/gen_safe/`. The
audited code is the code that produced the published numbers.

## 2. Verdicts

| ID | Verdict | Severity after verification | Measured effect on the published numbers |
|---|---|---|---|
| **P0-A** CBF projection then clip | **CONFIRMED** | **P1** | The mechanism fires in **9.6 % of the active-constraint steps** and in **46 of 50 episodes** — not rare. But every one of the 583 violations had a feasible command inside the box (`Σ|cᵢ| ≥ g`); the filter simply did not find it. Downstream: with the exact bounded projection the collision verdict changes in **1 episode out of 150** and the success rate is unchanged in every configuration; the mean minimum-clearance difference is **0.1 mm**. The published *claim* ("the filter returns a command satisfying the constraint") was false; the published *numbers* are barely affected |
| **P0-B** dead joint-limit mask | **CONFIRMED** | **P2** | **None.** The mask fires **0 times in >1 M substeps** across all six archived PPO checkpoints, random actions and a Jacobian controller. When it does fire (a saturated push into the elbow limit) the elbow settles 0.036 rad past its own limit with a 153 N·m error torque — a real plant-fidelity defect, unreachable in the shipped tasks |
| **P0-C** sampled clearance is an upper bound | **CONFIRMED** | **P1** | The reviewer's counterexample reproduces **exactly** (+13.129 mm reported, geometry actually intersecting). On the real CR10A capsules the per-pair over-statement is sub-millimetre at p99 (0.14 mm) and at most 7.1 mm. The sign flips for **10/3000 (0.33 %)** near-boundary configurations, **1/3000 (0.03 %)** at the filter's 2 cm margin and **0/3000** at 5 cm. The Lipschitz correction removes all of them |
| **P0-D** collision checked once per policy step | **CONFIRMED** | **P1** | The check runs after the 20 physics substeps of a policy step, so an intersection inside a step is invisible. The per-substep checker samples the geometry 20 times inside the step instead of once at the end |
| **P0-E** window comparison not matched | **CONFIRMED** | **P1** | The planner's own docstring says it: *"the episode's sampled goal configuration plays that role — privileged information the RL policy does not get"*. It also gets **30 s** of execution against the reactive policies' 6 s, and a different collision model. The "planning wins clearly" statement is not a controlled comparison |
| **P1-a** reward shaping / dropped limit penalty | **CONFIRMED** | **P1** | The wall reward replaces the base reward and drops its joint-limit penalty; the shaping `d_t − d_(t+1)` is the γ = 1 version and is not policy-invariant at γ = 0.99 |
| **P1-b** placeholder dynamics | **CONFIRMED** | **P1** | The learning constructor never selected `identified_run02`: every archived policy is a policy on the *placeholder* plant |
| **P1-c** filter before split | **CONFIRMED** | **P3** | **≤ 0.1 VAF percentage points** (M1 97.2/97.3/98.5 → 97.2/97.3/98.4; M2 97.5/97.4/98.7 → 97.5/97.4/98.6); mass scale k = 1.636 and the Coulomb values unchanged. Real as a mechanism, negligible in magnitude |
| **P1-d** incomplete result manifest | **CONFIRMED** | **P2** | No effect on the numbers, but the archived JSONs could not be reproduced from themselves. Now fixed |

## 3. The gate result: legacy vs corrected

`evaluation/compare_evaluators.py` replays the **recorded actions** of every episode through the
corrected environment. The physics is identical (the servo torque depends only on `q`, `q̇`,
`q_ref` and the command); only the detector differs. The driver asserts this: replaying the
actions in the legacy environment must reproduce the legacy verdict exactly, or the run aborts.

50 episodes per cell, seeds 1000–1049, 3 policy seeds × 3 tasks × {raw, CBF}.

| | legacy verdict | corrected verdict | verdicts changed |
|---|---|---|---|
| 16 of the 18 cells | — | identical | **0 / 50** |
| wall_ood, raw, seed 0 | 38 % collisions | 40 % | 1 / 50 |
| wall_ood, raw, seed 1 | 36 % | 38 % | 1 / 50 |
| wall_ood, raw, seed 2 | 34 % | 44 % | 5 / 50 |

**Conclusion: the published table is not an artefact of the evaluator.** The conservative
geometry (a rigorous lower bound instead of an over-estimate) and the per-substep collision check
move the verdict in at most 5 episodes out of 50, and only in the setting where the collision rate
is highest anyway. The success rates are unchanged everywhere.

## 4. A finding the review did not anticipate

The environment's collision detector is **sound but loose**: it answers *"did the enclosing
capsule touch the wall?"*, not *"did the arm touch the wall?"*. The CR10A's link capsules have
radii of 61–150 mm, so the capsule can be in contact while the mesh is still **13 cm** away
(measured at the home pose: capsule clearance 0.054 m, mesh clearance 0.130 m).

This is not a reason to distrust the published numbers — the capsule bound is a valid *lower*
bound on the mesh distance, so it never misses a real contact. It is a reason to state precisely
what the published collision rates measure, and to use the independent mesh-based checker
(`evaluation/independent_collision.py`) whenever the question is "did the arm actually touch
something".

The same looseness appears in the **M2 planning metric** (`ex_planning_lab/core/capsules.py`,
16 samples along the capsule axis): the sampling error there reaches `L/30 ≈ 25 mm` for Link2,
which is **larger than the 1–2 cm path margin the M2 report quotes**. `capsule_box_distance` now
offers `conservative=True` (the default stays legacy so the archived M2 numbers reproduce), and
`docs/M2_PLANNING_LAB.md` flags that the real-robot padding conclusion has to be recomputed with
the corrected value — in the direction of **more** margin, not less.

## 5. Two bugs of my own, found by the reproducibility checks

Both were caught by making the pipeline reproduce something that already existed:

1. **The derivative in the dynamics identification.** I had transcribed `qdd = d(filtered
   velocity)/dt` as `d(filtered position)/dt`. It changed the legacy path silently and did not
   reproduce the archived `run02_id.json`. The legacy path now reproduces it exactly, and a
   regression test pins it.
2. **The episode sampling depended on the evaluator.** The corrected clearance is lower, so the
   rejection sampling that picks the wall / start / goal accepted different configurations — the
   two evaluators were not even comparing the same episodes. The sampling now always uses the
   legacy distance.

A third, milder one: the independently filtered segments are concatenated with a **time gap**, so
the acceleration has to be computed inside each segment — `np.gradient` over the concatenated
array invents huge spikes at the joins.

## 6. What is fixed, and how it stays reproducible

Every fix keeps the **legacy behaviour as the default**, and the corrected path is behind an
explicit flag. Nothing archived is overwritten.

| Fix | Where | Flag |
|---|---|---|
| Bounded projection onto box ∩ half-space | `qp_filter.project_box_halfspace`, `safety_filter.cbf_policy` | the legacy variant is `cbf_policy_legacy` |
| Joint-limit mask captured before the overwrite | `cr10a_reach.py::step` | — (behaviour identical unless a limit is hit) |
| Rigorous lower bound on the clearance | `cr10a_reach_wall.py::pair_distances` | `WallConfig.conservative_geometry` |
| Per-substep collision detection | `cr10a_reach_wall.py::_substep_check` | `WallConfig.substep_collision_check` |
| Dynamics preset selectable | `cr10a_reach.py::make_model` | `ReachConfig.dynamics` |
| Discount-consistent shaping + limit penalty | `cr10a_reach_wall.py::progress_reward` | `WallConfig.discount_consistent_shaping`, `WallConfig.limit_penalty` |
| Split before filter | `dynamics_id.py::preprocess` | `--split-before-filter` |
| Complete result manifest | `run_safe_eval.py::build_manifest` | — |

**199 unit tests pass**, including 15 audit regressions, 9 corrected-evaluator tests, 5 QP tests
and 4 dynamics-identification tests.

## 7. What the gate does NOT say

- It does **not** say the learned policies are good. It says the *evaluation* is sound.
- It does **not** say the CBF filter is a safety guarantee. It is a kinematic filter on a
  known-geometry model; the measured residual collisions are real and the margin analysis above
  bounds how much of the geometry error the margin has to cover.
- It does **not** validate the window-scene comparison, which is still not budget- or
  privilege-matched (P0-E). A matched protocol is the next deliverable.
- It does **not** touch the hardware: nothing learned has ever run on the CR10A.

## 8. Next

1. The independent mesh-based checker over the same dev episodes (running) — it arbitrates what a
   "collision" means and how much the capsule looseness costs.
2. The matched window protocol: same deadline, same goal information, same collision model, same
   success criterion for the planner and the reactive policies.
3. Week 2: the bounded all-pair QP (B3) with a tuned fixed margin (B4) — the strong baseline the
   proposed execution-aware filter has to beat. The first margin sweep already shows the expected
   trade-off (margin 0.05 → 25 % success / 0 % collisions; margin 0.10 → 5 % success, the arm
   stalls 941 mm short).
