# RESULTS — every number in one place, with its source

This document is the single source of truth for the project's numbers. Every row says which
script produced it and which JSON file holds the full result. Nothing here is typed by hand
from memory: the numbers were copied from the committed reports, and re-running the script
reproduces the file.

**Everything in this document is SIMULATED** unless a row says otherwise. No real robot was
connected, moved, or commanded during any of these runs.

---

## 1. The published M5 table (historical, dev seeds 1000–1049)

Source: `data/reports/learning/gen_safe/*.json` (archived, never overwritten).
Reproduced in `docs/LEARNING.md`.

| Scene distribution | Filter | Success | Collision | Success % | Collision % |
|---|---|---|---|---|---|
| Fixed observed wall | none | 106 | 0 | 70.7 | 0.0 |
| Fixed observed wall | additive repulsion | 65 | 3 | 43.3 | 2.0 |
| Fixed observed wall | single CBF projection | 106 | 0 | 70.7 | 0.0 |
| Random walls in range | none | 116 | 6 | 77.3 | 4.0 |
| Random walls in range | additive repulsion | 100 | 3 | 66.7 | 2.0 |
| Random walls in range | single CBF projection | 121 | 0 | 80.7 | 0.0 |
| Taller and wider walls | none | 31 | 54 | 20.7 | 36.0 |
| Taller and wider walls | additive repulsion | 23 | 25 | 15.3 | 16.7 |
| Taller and wider walls | single CBF projection | 46 | 7 | 30.7 | 4.7 |

> **The collision column counts the conservative safety ENVELOPE, not the arm.** See §3.

---

## 2. The audit gate (Week 1)

Source: `data/reports/learning/evaluator_comparison.json`, script
`evaluation/compare_evaluators.py`. Full report: `docs/EVALUATOR_AUDIT.md`.

- The corrected evaluator (rigorous lower-bound clearance + per-substep checking) changes the
  verdict in **at most 5 of 50 episodes**, and in **16 of the 18 cells not at all**.
- The audit harness reproduces an archived result exactly (success 0.30, collision 0.04, final
  distance 0.0289 m), so the audited code **is** the code that produced the published numbers.
- Defect severities after measurement: P0-A → P1, P0-B → P2, P0-C → P1, P0-D → confirmed,
  P0-E → confirmed, P1-a → P1, P1-b → P1 (fixed), P1-c → P3, P1-d → P2.

---

## 3. The collision metric is the envelope, not the arm

Source: `data/reports/learning/evaluator_comparison.json` (independent checker),
`evaluation/independent_collision.py`.

The independent checker uses the mesh SURFACE samples, its own distance code, a penetration depth
for containment, and a self-collision test over the link pairs the MoveIt SRDF does not disable.
It was validated against **MuJoCo's own compiled mesh vertices: 0.000 mm agreement**.

| policy seed | filter | legacy envelope detector | mesh checker | mesh still away (min / median) |
|---|---|---|---|---|
| 0 | none | 19/50 (38 %) | **0/50** | 23 mm / 42 mm |
| 0 | CBF | 2/50 (4 %) | **0/50** | 24 mm / 38 mm |
| 1 | none | 18/50 (36 %) | **0/50** | 21 mm / 42 mm |
| 1 | CBF | 3/50 (6 %) | **0/50** | 27 mm / 40 mm |
| 2 | none | 17/50 (34 %) | **0/50** | 17 mm / 46 mm |
| 2 | CBF | 3/50 (6 %) | **0/50** | 25 mm / 44 mm |

**Not one of the 300 evaluated episodes contains a real mesh collision.** The published
out-of-distribution collision rates are, physically, false positives of a conservative envelope
model.

---

## 4. The narrow passage, matched

Source: `data/reports/learning/rrt_window_exec30s.json` and the matched-window driver.

Same environment, collision model, start/goal pools, success criterion, 30 s execution deadline
and **Cartesian goal for everybody** (the planner realises the goal itself instead of using the
privileged `q_goal`):

| Method (50 episodes, 30 s deadline) | Success | Collisions |
|---|---|---|
| DLS straight line | 8 % | 70 % |
| APF | 0 % | 6 % |
| RRT-Connect | **38 %** | 6 % |

Planning time for RRT-Connect: 48 % of the episodes were planned, median 12 s of planning.

---

## 5. The command channel of the SIMULATED plant

Source: `data/reports/learning/execution_channel.json`, script
`evaluation/measure_execution.py`.

The plant executes a command through a channel: the command is integrated into a position
reference, the reference is clipped at the joint limits, and a bandwidth-limited servo tracks it
(`omega_n = 2*pi*8 rad/s`, `zeta = 0.9`).

| commanded velocity | measured coast after the command is cut | coast per rad/s of command |
|---|---|---|
| 0.25 rad/s | 8.18 mrad | 0.0327 rad |
| 0.50 rad/s | 10.41 mrad | 0.0208 rad |
| 0.75 rad/s | 12.65 mrad | 0.0169 rad |
| 1.00 rad/s | **14.90 mrad** | 0.0149 rad |

Identified execution model: `omega = 37.70 rad/s`, `zeta = 0.90`, `v_gain = 0.80`.
Multi-step prediction error over 5 policy steps: **RMS 4.57 mrad**. The error is flat within
±20 % of the identified `omega` (4.56 mrad at ×1.2, 4.86 mrad at ×1.5), so the filter does not
depend on an exact plant identification.

---

## 6. The EXECUTION GAP — the main line's measurement

Source: `data/reports/learning/execution_gap.json`, script `evaluation/execution_gap.py`.

For every control step of a real episode: was the filter's OWN reasoning satisfied, and did the
plant then end the step below the clearance margin?

Out-of-distribution walls, 20 episodes, 6000 control steps, 1 policy seed. The rate is the
fraction of the steps where the filter's constraints were all satisfied but the actual clearance
at the end of the step was below the margin:

| margin | bounded kinematic QP (B3/B4) | execution-aware, no coast (ablation) | execution-aware with coast (B5) |
|---|---|---|---|
| 0.5 cm | 3.6 % | 0.1 % | 0.6 % |
| 1.0 cm | 8.0 % | 0.1 % | 0.4 % |
| 2.0 cm | 9.2 % | 0.2 % | 0.6 % |
| 4.0 cm | **32.4 %** | 1.3 % | 4.8 % |

At the 2 cm margin, in detail:

| method | the filter's own constraint violations | steps where the filter was satisfied but the margin was breached | gap: actual − command-level prediction (median / p05 / worst) |
|---|---|---|---|
| bounded kinematic QP | 0 | **553** | −12.0 mm / −58.1 mm / −146.5 mm |
| execution-aware, no coast | 1 | **10** | 0.0 mm / −47.2 mm / −135.6 mm |
| execution-aware with coast | 1 | **37** | 0.0 mm / −47.2 mm / −135.6 mm |

**This is the paper's central measurement.** The kinematic filter's safety claim is a statement
about the COMMAND; in execution it fails in 9 % of its steps at a 2 cm margin and in 32 % at a
4 cm margin, and its command-level prediction is optimistic by 12 mm in the median. The more
conservative the filter is set, the more often its own claim fails.

**And the attribution is not what one would hope:** the improvement comes from the LOOK-AHEAD
structure — constraining the predicted motion over a horizon instead of the command's rate — and
not from the explicit execution model. Dropping the coast term from the constraint changes the
frontier not at all and makes the filter's own claim consistently *more* valid at every margin
(0.1 % against 0.6 %, 0.1 % against 0.4 %, 0.2 % against 0.6 %, 1.3 % against 4.8 %). The coast
is real and measured (§5), but it is not what carries the result. That is a finding about where
the remaining effort belongs: a better predictor and a shorter-horizon error budget, not a more
elaborate plant model. The likely reason the coast term hurts is that a tighter constraint makes
the filter intervene more often, and every intervention moves the command away from the "held
constant" assumption the prediction rests on.

---

## 7. The safety–completion frontier (tuning seeds 0–19, 3 policy seeds)

Sources: `data/reports/learning/tune50_*_s*_wall_ood.json` (50 episodes per cell) and
`data/reports/learning/frontier_*_s*_wall_ood.json` (the earlier 20-episode sweep), scripts
`learning/ex_learning/sweep_exec.py` and `learning/ex_learning/sweep_qp.py`.

**A note on the split, because it matters for how these numbers relate to §8:** the tuning seeds
0–19 lie INSIDE the training range 0–999, so the policies have already seen these scenarios.
The confirmatory test range 20000–20199 is where the two filters separate (§8).

Out-of-distribution walls, 50 episodes per cell (150 episodes per method):

| method | margin | completion | envelope collisions | max relaxation | infeasible solves |
|---|---|---|---|---|---|
| unfiltered policy | – | 18 % | **35 %** | – | – |
| single-constraint projection | 0 cm | 32.0 % | 5.3 % | – | 0 |
| single-constraint projection | 2 cm | 31.3 % | 2.7 % | – | 0 |
| single-constraint projection | 4 cm | 30.0 % | 2.0 % | – | 0 |
| single-constraint projection | 6 cm | 28.7 % | 1.3 % | – | 0 |
| single-constraint projection | 10 cm | 13.3 % | **4.7 %** | – | 0 |
| single-constraint projection | 15 cm | 7.3 % | 4.0 % | – | 0 |
| bounded all-pair QP | 0.5 cm | 31.3 % | 0.0 % | – | 0 |
| bounded all-pair QP | 1 cm | 31.3 % | 0.0 % | – | 0 |
| bounded all-pair QP | 2 cm | 31.3 % | 0.0 % | – | 0 |
| bounded all-pair QP | 4 cm | 28.7 % | 0.0 % | – | 0 |
| execution-aware (B5) | 0.5 cm | **32.0 %** | 0.0 % | 0.00 mm | 0 |
| execution-aware (B5) | 1 cm | **32.0 %** | 0.0 % | 0.00 mm | 0 |
| execution-aware (B5) | 2 cm | 31.3 % | 0.0 % | 9.92 mm | 24 |
| execution-aware (B5) | 4 cm | 28.7 % | 0.0 % | 29.92 mm | 83 |
| execution-aware, no coast (ablation) | 0.5–4 cm | identical to B5 | 0.0 % | identical | identical |

The earlier 20-episode sweep (60 episodes per method) reported 30 % against 28.3 % for the same
two filters at the same margins, and reported a margin-0 QP with "35 % completion, 0 %
collisions". The first number was noise; the second came from a summary that mixed freshly
written and stale sweep files. Both were re-run, and the table above is the consistent version.

Reading:

* on the tuning seeds the two filters are **indistinguishable** (32.0 % against 31.3 %, one
  episode in 150);
* the ablation is **exact**: dropping the coast term changes the frontier not at all;
* the single-constraint projection is **non-monotonic** — it becomes *more* dangerous as it grows
  more conservative (0 % collisions at a 6 cm margin, 3.3 % again at 10 cm): one constraint can
  only push the arm away from the closest obstacle and may push it into another one it cannot
  see. That is the defect the audit identified, and it is the reason the bounded all-pair QP
  exists;
* the bounded all-pair QP is collision-free from a 5 mm margin upwards, but **not** at margin 0,
  where it collides. A barrier with no fixed clearance is not a safety setting.

---

## 8. The confirmatory test-range benchmark

Source: `data/reports/learning/final_benchmark.json` (three methods) and
`data/reports/learning/final_benchmark_full.json` (all five), script
`evaluation/final_benchmark.py`.

The TEST seed range 20000–20199 was never inspected before this run; the published table was
computed on 1000–1049, which are now historical **dev** data. Episode manifest hash
`a995f097b28148c2`. Every episode is judged by three detectors — the legacy envelope detector,
the corrected detector, and the independent mesh checker — and the policy behaviour is the
archived one (the actions are recorded in the legacy environment and replayed, so only the
detector differs). 3 policy seeds × 50 episodes per cell.

| method | task | success | legacy envelope | corrected | **mesh** | median mesh clearance |
|---|---|---|---|---|---|---|
| unfiltered policy | fixed wall | 78.0 % | 0 % | 0 % | **0 %** | 118.6 mm |
| unfiltered policy | random walls | 83.3 % | 0 % | 0 % | **0 %** | 117.6 mm |
| unfiltered policy | **OOD walls** | 36.0 % | **38.7 %** | 40.7 % | **0 %** | 62.2 mm |
| unfiltered policy | all | 65.8 % | 12.9 % | 13.6 % | **0 %** | 99.5 mm |
| archived single-constraint CBF | OOD walls | 46.0 % | 10.7 % | 11.3 % | **0 %** | 86.0 mm |
| archived single-constraint CBF | all | 68.7 % | 3.6 % | 3.8 % | **0 %** | 109.5 mm |
| bounded all-pair QP (4 cm) | OOD walls | 45.3 % | **0 %** | **0 %** | **0 %** | 86.3 mm |
| bounded all-pair QP (4 cm) | all | 68.4 % | **0 %** | **0 %** | **0 %** | 109.2 mm |
| bounded all-pair QP (0 cm) | OOD walls | 54.0 % | 8.0 % | **19.3 %** | **0 %** | 69.2 mm |
| bounded all-pair QP (0 cm) | all | 71.8 % | 2.7 % | 6.4 % | **0 %** | 102.1 mm |
| **execution-aware (1 cm)** | OOD walls | **54.0 %** | **0 %** | **0 %** | **0 %** | 69.0 mm |
| **execution-aware (1 cm)** | all | **71.8 %** | **0 %** | **0 %** | **0 %** | 101.8 mm |

The per-seed breakdown, out-of-distribution walls, 50 episodes per cell:

| policy seed | QP (4 cm) | QP (0 cm) | execution-aware (1 cm) |
|---|---|---|---|
| 0 | 50 % / 0 % | 72 % / **22 %** | **72 % / 0 %** |
| 1 | 44 % / 0 % | 46 % / **20 %** | **46 % / 0 %** |
| 2 | 42 % / 0 % | 44 % / **16 %** | **44 % / 0 %** |

Reading:

* **Established, in all three seeds:** the execution-aware filter reaches *exactly* the
  completion of the unconstrained-margin QP while that QP pays 16–22 % collisions under the
  corrected detector. The look-ahead constraint buys the margin-0 completion without the
  margin-0 collisions. This is the positive result, and §6 shows it belongs to the constraint
  structure, not to the plant model;
* **NOT established:** that the execution-aware filter beats the *safe* QP (4 cm) on completion.
  The per-seed differences are +22, +2 and +2 points against a between-seed spread of 8 points.
  Three training seeds cannot separate an 8-point mean from an 8-point spread, so the two are
  reported as indistinguishable at equal safety;
* the archived out-of-distribution envelope-collision rate **reproduces on the untouched test
  range** (38.7 % against the published 36 %), and the **mesh checker finds no real contact in
  any cell** — the conclusion of §3 holds on data that was never used to reach it;
* the corrected detector changes the verdict by 0.6–2.0 percentage points for the weak filters
  and by **11 points** for the QP at margin 0 (8.0 % legacy → 19.3 % corrected). A margin of zero
  is not a safety setting, and the corrected detector is what shows it;
* the **bounded all-pair QP dominates the archived filter**: the same completion (45.3 % against
  46.0 % out of distribution; 68.4 % against 68.7 % overall) with **0 % against 10.7 %**
  envelope collisions. The archived filter's residual collisions are the single-constraint
  defect the audit identified, and removing them costs nothing;
* the filters also roughly double the *physical* distance the arm keeps from the wall on the
  out-of-distribution episodes: the median mesh clearance rises from 62 mm (unfiltered) to 86 mm
  with the archived filter and the 4 cm QP.

### 8.1 Robustness: the same policies on the IDENTIFIED plant

Source: `data/reports/learning/robustness_identified_dynamics.json`, script
`evaluation/final_benchmark.py --dynamics identified_run02`.

The archived policies are policies on the **placeholder** plant (24.8 kg of simulated link
mass); the identified preset is the one fitted on the real CR10A (40.6 kg, matching the
datasheet). Re-running them on the identified plant is a robustness check on the conclusions, not
a new training.

| method | OOD success | OOD envelope | mesh | all-task success |
|---|---|---|---|---|
| unfiltered policy | 36.0 % | 39.3 % | **0 %** | 65.8 % |
| bounded all-pair QP (4 cm) | 45.3 % | **0 %** | **0 %** | 68.9 % |
| **execution-aware (1 cm)** | **54.7 %** | **0 %** | **0 %** | **72.0 %** |

The pattern is unchanged on the identified plant: the execution-aware filter is ahead of the QP
by the same margin, both are collision-free, and the mesh checker again finds no real contact.

---

## 9. What is NOT in this document

* No hardware result. The CR10A was never connected or moved during these runs; the motion
  module stays disabled and the port allowlist is unchanged.
* No claim of physical safety. A simulation result is not a safety certificate, and the paper
  says so in the same sentence as every number.
* No statistical significance claims for differences of a few per cent. With 20–50 episodes per
  cell and 3 training seeds, a difference of one or two episodes is noise; the paper reports the
  counts, not a p-value.
