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

Sources: `data/reports/learning/frontier_*_s*_wall_ood.json`, scripts
`learning/ex_learning/sweep_qp.py` and `learning/ex_learning/sweep_exec.py`.

Out-of-distribution walls, 20 episodes per cell:

| method | margin | completion | envelope collisions | max relaxation | infeasible solves |
|---|---|---|---|---|---|
| unfiltered policy | – | 15 % | **42 %** | – | – |
| single-constraint projection | 0 cm | 30 % | 5.0 % | – | 0 |
| single-constraint projection | 2 cm | 28 % | 3.3 % | – | 0 |
| single-constraint projection | 4 cm | 28 % | 1.7 % | – | 0 |
| single-constraint projection | 6 cm | 22 % | 0.0 % | – | 0 |
| single-constraint projection | 10 cm | 8 % | **3.3 %** | – | 0 |
| single-constraint projection | 15 cm | 5 % | 1.7 % | – | 0 |
| bounded all-pair QP | 0 cm | 30 % | **3.3 %** | – | 0 |
| bounded all-pair QP | 0.5 cm | 28 % | 0.0 % | – | 0 |
| bounded all-pair QP | 1 cm | 28 % | 0.0 % | – | 0 |
| bounded all-pair QP | 2 cm | 28 % | 0.0 % | – | 0 |
| bounded all-pair QP | 4 cm | 28 % | 0.0 % | – | 0 |
| bounded all-pair QP | 6 cm | 18 % | 0.0 % | – | 0 |
| bounded all-pair QP | 10 cm | 5 % | 0.0 % | – | 107 |
| bounded all-pair QP | 15 cm | 0 % | 0.0 % | – | 1666 |
| execution-aware (B5) | 0.5 cm | **30 %** | 0.0 % | 0.00 mm | 0 |
| execution-aware (B5) | 1 cm | **30 %** | 0.0 % | 0.00 mm | 0 |
| execution-aware (B5) | 2 cm | 28 % | 0.0 % | 3.25 mm | 3 |
| execution-aware (B5) | 4 cm | 25 % | 0.0 % | 23.25 mm | 24 |
| execution-aware (B5) | 6 cm | 13 % | 0.0 % | 43.25 mm | 64 |
| execution-aware, no coast (ablation) | 0.5–6 cm | identical to B5 | 0.0 % | identical | ≈ identical |

Reading:

* the single-constraint projection becomes **more** dangerous as it grows more conservative
  (3.3 % collisions at a 10 cm margin, after 0 % at 6 cm): one constraint can only push the arm
  away from the closest obstacle and may push it into another one it cannot see;
* the bounded all-pair QP is collision-free from a 5 mm margin upwards, but **not** at margin 0,
  where it collides in 3.3 % of the episodes; its completion is flat at 28 % from 5 mm to 4 cm;
* the execution-aware filter is collision-free from a 5 mm margin as well, at 30 % completion —
  one episode more than the QP out of 60, which is noise — and it never needed a relaxed
  (violated) constraint below a 2 cm margin;
* **the execution-aware filter does not improve the envelope-level safety–completion trade-off
  in this setting.** Its demonstrated advantage is §6, not the frontier.

> A note on a number that was wrong at first: an earlier version of this table reported the
> bounded QP at margin 0 as "35 % completion, 0 % collisions". That came from a summary that
> mixed freshly-written and stale sweep files. The sweep was re-run with one consistent margin
> list for all three policy seeds; the corrected numbers are above, and the QP at margin 0 does
> collide.

---

## 8. The confirmatory test-range benchmark

Source: `data/reports/learning/final_benchmark_full.json`, script
`evaluation/final_benchmark.py`.

The TEST seed range 20000–20199 was never inspected before this run. Every episode is judged by
three detectors: the legacy envelope detector, the corrected detector, and the independent mesh
checker. The policy behaviour is the archived one (the actions are recorded in the legacy
environment and replayed, so only the detector differs).

*(filled in from the run; see the JSON for the per-cell detail)*

---

## 9. What is NOT in this document

* No hardware result. The CR10A was never connected or moved during these runs; the motion
  module stays disabled and the port allowlist is unchanged.
* No claim of physical safety. A simulation result is not a safety certificate, and the paper
  says so in the same sentence as every number.
* No statistical significance claims for differences of a few per cent. With 20–50 episodes per
  cell and 3 training seeds, a difference of one or two episodes is noise; the paper reports the
  counts, not a p-value.
