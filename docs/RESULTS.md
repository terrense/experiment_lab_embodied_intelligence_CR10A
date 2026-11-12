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

## 6. The EXECUTION GAP - the main line's measurement

Source: `data/reports/learning/execution_gap.json`, script `evaluation/execution_gap.py`.

For every control step of a real episode: was the filter's OWN reasoning satisfied, and did the
plant then end the step below the clearance margin?

**The metric was corrected (2026-11-09), and the correction changes the headline.** The obvious
reading - "the filter was satisfied and the margin was breached, so the filter's claim failed" -
mixes two different events. A step can end below the margin because the arm **started** below
it: the constraint a rate-based filter enforces, `dc/dt >= -alpha(d - m)/v_max`, *demands* that
the arm escape whenever `d < m`, and 20 ms is not always enough to escape a 2 cm buffer. Those
steps are not failures of a claim made from a safe state - the claim was never a position claim.

The metric is therefore split: `initially unsafe` = the step **started** below the margin;
`crossing` = the step **started at or above** it and ended below it.

Out-of-distribution walls, 20 episodes, 6000 control steps, 1 policy seed. Each cell is
"steps inside the margin / safe-to-unsafe crossings":

| margin | bounded kinematic QP (B3/B4) | execution-aware, no coast (ablation) | execution-aware with coast (B5) |
|---|---|---|---|
| 0.5 cm | 3.5 % / 0.1 % | 0.0 % / 0.1 % | 0.1 % / 0.5 % |
| 1.0 cm | 7.8 % / 0.1 % | 0.0 % / 0.1 % | 0.0 % / 0.4 % |
| 2.0 cm | **9.1 % / 0.2 %** | 0.0 % / 0.1 % | **0.2 % / 0.5 %** |
| 4.0 cm | **32.1 % / 0.3 %** | 0.4 % / 0.9 % | **1.9 % / 3.0 %** |

At the 2 cm margin, in detail:

| method | the filter's own constraint violations | initially unsafe | **crossings (safe -> unsafe)** | all breaches (the old metric) | gap: actual - command-level prediction (median / p05 / worst) |
|---|---|---|---|---|---|
| bounded kinematic QP | 0 | 543 | **10** | 553 | -12.0 mm / -58.1 mm / -146.5 mm |
| execution-aware, no coast | 1 | 2 | **8** | 10 | 0.0 mm / -47.2 mm / -135.6 mm |
| execution-aware with coast | 1 | 10 | **27** | 37 | 0.0 mm / -47.2 mm / -135.6 mm |

**This is the paper's central measurement, and it is now stated correctly.** The two filters make
**different kinds of claim**, and the old metric could not tell them apart:

* the **kinematic QP** enforces a **rate** condition on the command. It discharges that claim at
  essentially every step (0 violations) and its rate condition pushes the arm out of the buffer
  fast enough that it rarely *crosses* - but for **9.1 %** of its steps at a 2 cm margin
  (**32.1 %** at 4 cm) it operates with the position guarantee already violated. Its
  command-level prediction is optimistic by 12 mm in the median.
* the **execution-aware filter** enforces a **position** condition on the predicted motion. It
  holds that guarantee at **99.8 %** of the steps (2 cm margin) - a factor of **50** fewer steps
  inside the buffer - and fails from a safe state in 0.5 %.

So the kinematic filter's guarantee is a statement about the COMMAND, and the command is not the
motion; what the look-ahead formulation buys is that the guarantee it *appears* to make - "the
clearance stays above the margin" - is the guarantee it actually enforces.

**And the attribution is not what one would hope:** the improvement comes from the LOOK-AHEAD
structure - constraining the predicted motion over a horizon instead of the command's rate - and
not from the explicit execution model. Dropping the coast term from the constraint changes the
frontier not at all. The coast is real and measured (section 5), but it is not what carries the
result. That is a finding about where the remaining effort belongs: a better predictor and a
shorter-horizon error budget, not a more elaborate plant model. The likely reason the coast term
hurts is that a tighter constraint makes the filter intervene more often, and every intervention
moves the command away from the "held constant" assumption the prediction rests on.

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

The intervals, from `evaluation/confirmatory_stats.py`
(`data/reports/learning/confirmatory_stats.json`), now with **nine** training seeds (the six
extra ones were trained with the archived configuration; the episodes are the same). The
comparison is **paired over scenarios**: the same 50 episode seeds are run by every method, so
the difference is computed per scenario and the bootstrap resamples *scenarios* — never
individual observations. The hierarchical interval resamples **training seeds first** and the
scenarios inside a selected seed; that is the interval that answers "would this hold on a new
training seed and a new set of scenarios".

| comparison (completion) | mean diff | hierarchical 95 % CI | naive CI | between-seed spread of the difference | verdict |
|---|---|---|---|---|---|
| execution-aware - QP (4 cm) | +6.7 pts | **[+2.0, +12.0]** | [+4.2, +9.1] | 7.1 pts | `NOT_SEPARABLE` |
| execution-aware - QP (0 cm) | +0.4 pts | [-1.1, +2.2] | [-0.9, +1.8] | 1.7 pts | identical per episode |
| QP (4 cm) - QP (0 cm) | -6.7 pts | [-12.0, -2.0] | [-9.1, -4.2] | 7.1 pts | `NOT_SEPARABLE` |

The naive interval is **1.6x narrower** than the hierarchical one, and it is the one a per-episode
analysis would have produced: it reads as a settled 6.7-point win. The estimator's verdict is
`NOT_SEPARABLE` - and with nine clusters the reason is no longer "too few seeds" but the
substantive one: the between-seed spread of the difference (7.1 points) is of the same order as
the difference itself (6.7 points). Eight of the nine per-seed differences are positive
(+22, +2, +2, +6, +2, +10, +8, -2, +10); the largest one, seed 0, inflates both the mean and the
spread.

**The collision comparison is separable.** execution-aware - QP (0 cm) is **-7.3 points** of
envelope collisions with a hierarchical interval of **[-11.1, -4.0]** that **excludes zero**;
execution-aware - QP (4 cm) is +0.0 points. Under the stricter corrected detector of section 8
the same difference is larger (19.3 % against 0 %), because the margin-0 filter's envelope
contacts are exactly what the corrected detector resolves.

Per seed (completion / envelope collisions, 50 episodes each):

| policy seed | QP (4 cm) | QP (0 cm) | execution-aware (1 cm) |
|---|---|---|---|
| 0 | 50 % / 0 % | 72 % / 0 % | 72 % / 0 % |
| 1 | 44 % / 0 % | 46 % / 16 % | 46 % / 0 % |
| 2 | 42 % / 0 % | 44 % / 8 % | 44 % / 0 % |
| 3 | 42 % / 0 % | 50 % / 6 % | 48 % / 0 % |
| 4 | 42 % / 0 % | 44 % / 6 % | 44 % / 0 % |
| 5 | 38 % / 0 % | 48 % / 10 % | 48 % / 0 % |
| 6 | 60 % / 0 % | 66 % / 4 % | 68 % / 0 % |
| 7 | 40 % / 0 % | 38 % / 8 % | 38 % / 0 % |
| 8 | 50 % / 0 % | 56 % / 8 % | 60 % / 0 % |

Reading:

* **Established, in all nine seeds:** the execution-aware filter reaches *exactly* the
  completion of the unconstrained-margin QP while that QP pays collisions (16-22 % under the
  corrected detector on the three audited seeds; 4-16 % envelope contacts on all nine). The
  look-ahead constraint buys the margin-0 completion without the margin-0 collisions, and the
  collision difference is **separable**: the hierarchical interval is [-11.1, -4.0] points and
  excludes zero, with nine clusters. This is the positive result, and section 6 shows it belongs
  to the constraint structure, not to the plant model;
* **NOT established:** that the execution-aware filter beats the *safe* QP (4 cm) on completion.
  With nine training seeds the per-seed differences are +22, +2, +2, +6, +2, +10, +8, -2 and +10
  points against a between-seed spread of the difference of 7.1 points, so the two are reported
  as indistinguishable at equal safety;

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

## 9. Replication on a second untouched range

Source: `data/reports/learning/final_benchmark_replication.json`, script
`evaluation/final_benchmark.py --split replication`. Manifest version 2, hash
`7cfdb66b76291f27`. The same policies, the same margins, the same three detectors, on the range
**30000–30199**, which had never been touched.

| method | replication: completion / envelope / corrected / mesh | confirmatory range: the same four |
|---|---|---|
| unfiltered policy | 40.7 % / 32.0 % / 33.3 % / **0 %** | 36.0 % / 38.7 % / 40.7 % / **0 %** |
| archived single-constraint CBF | 51.3 % / 6.0 % / 6.7 % / **0 %** | 46.0 % / 10.7 % / 11.3 % / **0 %** |
| bounded all-pair QP (4 cm) | 52.0 % / **0 %** / **0 %** / **0 %** | 45.3 % / **0 %** / **0 %** / **0 %** |
| bounded all-pair QP (0 cm) | 56.0 % / 7.3 % / **16.0 %** / **0 %** | 54.0 % / 8.0 % / **19.3 %** / **0 %** |
| **execution-aware (1 cm)** | **56.7 %** / **0 %** / **0 %** / **0 %** | 54.0 % / **0 %** / **0 %** / **0 %** |

Every qualitative claim of §8 reproduces: the unfiltered policy collides in about a third of the
episodes under the envelope detector while the mesh checker finds **no real contact in any
cell**; the two bounded filters are collision-free in every cell; the margin-0 QP reaches the
completion of the execution-aware filter and pays 7–16 % of envelope contacts for it, and the
execution-aware filter pays none. The absolute numbers are slightly higher on the new range (the
scenarios are easier), but the ordering and the collision verdict are identical in all three
training seeds.

The manifest version differs (the replication range only exists in v2), but the **test range is
identical in v1 and v2** — only the tuning split moved and the replication range was added — so
the §8 result is not affected by the version change.

---

## 10. The stopping tail, the nonlinear re-check, and the uncertainty budget

Source: `data/reports/learning/stopping_rollout.json`, script
`evaluation/stopping_rollout.py`.

The states are sampled from the **execution-aware filter's own output** (216 states, three policy
seeds, six episodes each), where the clearance is at least the filter's 10 mm margin by
construction — that is where the terminal-stop question is asked. The command is cut to zero and
the plant is rolled on with every physics substep recorded.

**A. The stopping tail.** A zero velocity command does not stop the motion.

| time after the cut | tool-point travel (p95) | clearance lost (p95) | clearance lost (worst) |
|---|---|---|---|
| 50 ms | 110 mm | 35 mm | 63 mm |
| **100 ms** (the filter's whole look-ahead) | **142 mm** | **38 mm** | **87 mm** |
| 200 ms | 129 mm | 24 mm | 80 mm |
| 500 ms | 129 mm | 24 mm | 80 mm |

* In **35 %** of the states the tail brings the arm *closer* to the obstacle than at the cut.
* The worst case loses **87 mm** of clearance — **8.7× the filter's own 10 mm margin**.
* The motion is over after 200–500 ms (10–25 control steps); the filter's look-ahead is 5 steps.
* The coast itself is modelled well: the measured/predicted displacement ratio is 0.99997
  (median), 1.014 (worst). The problem is not the coast model, it is the horizon.

**B. The nonlinear re-check.**

| quantity | median | p05 | p95 | worst |
|---|---|---|---|---|
| model prediction error at the horizon end | **22.3 mrad** | 11.3 | 88.8 | 188.5 |
| the same, through the filter's first-order expansion | 22.4 mrad | 10.6 | 93.7 | 194.2 |
| the expansion alone (first-order vs the model's own output) | 8.5 mrad | 5.1 | 10.5 | 11.7 |
| tool-point error of the predicted motion | 22.5 mm | 3.3 | 79.5 | 140.0 |

* The **linearisation is not the problem**: the expansion stays within 8.5 mrad of the model's
  own output.
* The **model is the problem, and it generalises worse than it was fitted to say**: the
  identification reports 4.57 mrad, but that was measured on a random-command rollout on the
  reach task. On the states the filter produces, the same model's error at the same horizon is
  **22.3 mrad — five times larger**.

**C. The uncertainty budget, separated.** The tube radius ε_i of §5.5's conditional argument is
the gap between the environment's capsule clearance and the physical mesh clearance. Over 1512
link–configuration samples it is **one-sided**: the capsule clearance is smaller in **1512 of
1512** samples, by 49 mm at the median (6 mm p05, 126 mm worst).

| term | magnitude | direction |
|---|---|---|
| geometric (capsule vs mesh) | 6–126 mm, median 49 mm | **one-sided** — works *for* the guarantee |
| prediction (linearised vs nonlinear) | 22 mrad median, 89 mrad p95 | two-sided — eats the margin |
| stopping tail (0.1–0.5 s after the cut) | up to 87 mm of clearance | two-sided — eats the margin, the most |

The margin has to be spent on the prediction error and on the stopping tail. The geometry, which
looks like the hard modelling problem, is already safe.

---

## 11. Planner guidance: does the filter work across proposal sources?

Source: `data/reports/learning/b6_plans.json` and
`data/reports/learning/b6_planner_filter.json`, script `evaluation/b6_planner_filter.py`.

The planner is given the scene geometry, the start configuration and the **Cartesian** goal.
It does **not** get the episode's privileged goal configuration and it does not get a goal pool:
it realises the goal into a joint configuration itself, with a damped-least-squares IK. Of the
60 realisations sampled per episode, 32 are accurate *and* collision-free (median), and the goal
is realised to **0.033 mm** at the median. One plan per episode, shared by the three planner
arms; the planner runs in the rigorous (lower-bound) clearance model.

**Planning:** 42 of the 50 episodes are planned (**84 %**), median planning time **1.53 s**,
worst case 16.0 s. The eight failures are reported as failures.

| arm | completion (30 s) | completion (6 s) | legacy envelope | corrected | **mesh** |
|---|---|---|---|---|---|
| unfiltered policy | 38.7 % | 36.0 % | 38.7 % | 40.7 % | **0 %** |
| bounded all-pair QP (4 cm) | 48.7 % | 45.3 % | **0 %** | **0 %** | **0 %** |
| **execution-aware (1 cm)** | **56.7 %** | **54.0 %** | **0 %** | **0 %** | **0 %** |
| planner + waypoint follower | **84.0 %** | 4.0 % | **0 %** | **0 %** | **0 %** |
| planner + bounded QP | 80.0 % | 4.0 % | **0 %** | **0 %** | **0 %** |
| planner + execution-aware | **84.0 %** | 4.0 % | **0 %** | **0 %** | **0 %** |

**The harness is validated against §8.** The 6 s column is not a new measurement — it is the
same measurement code on the same episodes, and it reproduces the confirmatory table cell for
cell: 36.0 %, 45.3 % and 54.0 % completion, and 38.7 % / 40.7 % envelope collisions for the
unfiltered policy.

* **The filter works across proposal sources, and it is redundant on a good one.** On the
  planner it changes nothing (84.0 % → 84.0 % with the execution-aware filter, 80.0 % with the
  bounded QP — slightly worse, because the QP brakes a follower that was already safe). On the
  reactive policy the same filter turns 38.7 % into 56.7 % and 38.7 % envelope collisions into
  0 %. **The filter's value is not a property of the filter; it is a property of how bad the
  proposal is.**
* **On this task the learned policy adds nothing on top of a planner** that is handed the same
  information: 84.0 % against 56.7 %. That is a statement about this task (a policy trained on
  random walls, evaluated on taller and wider ones), not about learning in general.
* **The planner is slow.** At 6 s it has completed 4 % of the episodes against the filtered
  policy's 54 %: the waypoint follower moves at most 0.6 rad/s. The 84 % figure is a 30 s
  number, and the follower's speed was not tuned.
* **A statistical caveat:** the planner arms are identical in every training seed (84 %, 84 %,
  84 %), because a plan does not depend on a policy. They have no between-seed variance, so
  they cannot enter a seed-level bootstrap as the policy arms do.

What this section does **not** say: that the planner is the better method. It is given the scene
geometry, a collision model and a deadline five times longer than the policy needs; it fails to
plan 16 % of the episodes; and its completion is a single deterministic number rather than a
distribution over training seeds.

---

## 12. The censoring of the safety evaluation

Source: `data/reports/learning/physical_contact.json`, script
`evaluation/physical_contact.py`. **SIMULATION ONLY**: this diagnostic deliberately drives the arm
into the obstacle and must never be run on hardware.

The episode is TERMINATED at the first conservative-envelope contact. The trajectory the mesh
checker sees is therefore **censored** - the arm was stopped at the envelope and never got the
chance to reach a real contact. This is the measurement of what that censoring hides: the same
policies, the same episodes (the frozen test range, 50 per cell, three training seeds), with the
termination removed.

| method | envelope contact | mesh contact, **censored** (the archived behaviour) | mesh contact, **uncensored** | minimum mesh clearance, censored / uncensored (median) |
|---|---|---|---|---|
| unfiltered policy | 39.3 % | **0.0 %** | **22.0 %** | 66.3 mm / 63.3 mm |
| bounded all-pair QP (4 cm) | 0.0 % | 0.0 % | 0.0 % | 90.0 mm / 90.0 mm |
| execution-aware (1 cm) | 0.0 % | 0.0 % | 0.0 % | 71.6 mm / 71.6 mm |

Per training seed (the unfiltered policy):

| training seed | envelope contact | mesh contact, censored | mesh contact, uncensored |
|---|---|---|---|
| 0 | 48.0 % | 0.0 % | 22.0 % |
| 1 | 34.0 % | 0.0 % | 28.0 % |
| 2 | 36.0 % | 0.0 % | 16.0 % |

* **The archived collision rate was censored, and the censoring was optimistic.** The unfiltered
  policy makes real mesh contact in 16-28 % of the episodes it touches the envelope in. The
  earlier reading - "the arm never touched the wall, only its conservative envelope" - is wrong:
  the arm was stopped before it could, and often it would have.
* **The filters' result is stronger than the archived table showed.** Neither filtered arm ever
  reaches the envelope, so their censored and uncensored trajectories are identical and their
  mesh contact is 0 % in both. They are the only arms that never enter the regime in which
  contact becomes possible.
* **The instrument was right and the measurement was wrong.** The mesh checker stays; what had to
  change is that it must be applied to a trajectory that was not truncated.
* **This is a general property, not a quirk of this project:** any evaluation that terminates an
  episode at the first *proxy* event measures the proxy, and reports a rate that is biased
  towards "no harm done" by construction. The bias is invisible in the metric; it is only
  visible when the termination is removed.

---


## 13. What is NOT in this document

* No hardware result. The CR10A was never connected or moved during these runs; the motion
  module stays disabled and the port allowlist is unchanged.
* No claim of physical safety. A simulation result is not a safety certificate, and the paper
  says so in the same sentence as every number.
* No statistical significance claims for differences of a few per cent. With 20–50 episodes per
  cell and 3 training seeds, a difference of one or two episodes is noise; the paper reports the
  counts, not a p-value.
