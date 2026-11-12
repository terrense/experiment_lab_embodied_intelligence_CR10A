# Research plan: execution-aware safety filtering on a position-controlled CR10A

> **Status:** 2026-11-04, working plan. Supersedes the free-form "what next" list in
> `docs/ROADMAP.md` for M5. Every claim in this document is either measured here or marked
> UNVERIFIED — nothing is planned into existence.
>
> **Provenance.** The plan follows an external source review of the archived code
> (`CR10A_Research_Design_CN.md`, 2026-11-02) and the English manuscript draft. **Every defect the
> review alleged was independently re-derived from the code before anything was changed**; where
> the review over- or under-stated the severity, the corrected version is recorded below.

---

## 0. REVISED MAIN LINE (2026-11-09) — read this first

The plan below is the **control-line plan**, and it is 90 % done. It stays, because it produced
the project's only *measured* results and because the next line is **built on top of it**. What
changed is the question the work is organised around.

### 0.1 Why the main line changed

An external review (2026-11-08/09) and the owner's own technical direction agree on two things:

1. **The owner's long-term accumulation is in LLM-agent / post-training / RL / verifiable
   evaluation**, not in control theory. A paper whose contribution is "we measured the execution
   gap of a safety filter" is a robotics-systems paper, and it does not compound into the
   capability the owner is building.
2. **The control line's own falsification criterion already fired against it.** The ablation
   removes the coast term and the frontier does not move: the *look-ahead constraint structure*
   carries the result, not the plant model. The filter is a **repair** that makes a reactive
   proposal usable — and §7.10 of the manuscript shows it is redundant on a good proposal.

So the control line is **demoted from "the contribution" to "the substrate"**, and the new
contribution is one level up.

### 0.2 The new question

> **When the safety layer's own guarantee is known to be imperfect — and its imperfection is
> measured — how should a high-level agent decide when to keep going, when to slow down, when to
> retreat, and when to give up?**

This is a sequence-decision problem, not a control problem, and it sits exactly where the
owner's expertise is: agent policies, verifiable reward, credit assignment under a sparse final
outcome, and post-training (SFT / DPO / GRPO).

### 0.3 Why this repo is the right platform for it (and why it is not generic)

A generic "LLM calls verified robot skills" paper has been done (IROSA, RA-L 2026;
RecoveryChaining, IROS 2025; MAGMA-GEN, CoRL 2026). Assembling those pieces is not a
contribution. What this repo has that those papers do not is a **measured risk model of its own
execution layer**, and that is the asset:

| asset | measured value | why the agent needs it |
|---|---|---|
| the gap between a safety layer's claim and the plant | 9.2 % of control steps at a 2 cm margin, median error 12 mm, worst 146 mm | the agent's reward is "did the arm stay safe", and this says how reliable that reward is |
| the **stopping cost** of a decision | up to 87 mm of clearance lost after a command is cut; 35 % of states get closer | "keep going vs stop safely" is a **non-binary** trade-off, and the agent needs the price of stopping |
| the **geometric** model error | one-sided: the capsule metric under-estimates the mesh clearance in 1512/1512 samples, by 49 mm at the median | the verifier's bias is known and conservative, so the reward is usable |
| the **censoring** of the safety evaluation | the envelope metric stops the arm before it can touch; uncensored, the unfiltered policy makes real mesh contact in 50 % of the episodes | the reward must be computed on trajectories that are not stopped early |

**The contribution is therefore not "an agent that calls robot skills". It is "an agent whose
reward is a *calibrated* risk signal, and a study of what post-training buys on top of it".**

### 0.4 The new method ladder

The old ladder (B0–B8) is preserved below as the **control ladder** and is not rewritten. The new
ladder runs on top of it:

| group | agent layer | purpose |
|---|---|---|
| A0 | fixed state machine with hand-written recovery rules | **the strong baseline.** If the FSM matches the learned agent, the learning claim is void |
| A1 | LLM agent, prompt only, no training | the zero-training reference |
| A2 | LLM agent + SFT on successful trajectories | imitation floor |
| A3 | LLM agent + DPO on (success, failure) pairs | preference floor |
| A4 | LLM agent + GRPO / RLVR on the calibrated risk reward | **the research candidate** |
| A5 | A4 + failure-node-focused sampling (which failure to spend the budget on) | the mechanism contribution |

The **decisive experiment** is A0 vs A1 vs A2 vs A4 on **unseen fault combinations**, with the
same task, the same skills, the same budget and paired seeds. If A4 does not beat A0 and A2
stably, the RL claim is void and the paper says so.

### 0.5 What must be true before A4 is meaningful

The agent's reward is the verifier. **A wrong verifier optimises toward a wrong target.** That is
why the control line's Gate-1 repairs are not a detour — they are a prerequisite:

| Gate-1 repair | why the agent line needs it |
|---|---|
| the execution-gap metric is split into *initially unsafe* and *safe→unsafe crossing* | the agent must be scored on the failures it could have prevented, not on states it inherited |
| the censoring diagnostic (the physical-contact protocol) | the reward must not be computed on trajectories the safety layer truncated |
| the mesh checker on **every** episode (stride 1) | the reward's ground truth must have the same denominator as the comparison |
| the solver latency distribution (P50/P95/P99) | the agent's skills have a real time cost, and the reward must price it |
| **B4+**: the speed-dependent (stopping-distance) margin | if a trivial margin rule matches the look-ahead filter, then the agent has no reason to prefer the expensive one — and the substrate itself is wrong |

### 0.6 The immediate sequence

1. **Gate 1** (this week): the five repairs above, then re-state the control line's headline
   numbers with the corrected metric. *The censoring result already changes one headline: the
   unfiltered policy's "0 % real collisions" was an artefact of the episode being terminated at
   the envelope contact — uncensored, it makes real mesh contact in 50 % of the episodes.*
2. **The agent substrate** (2–3 weeks): the skill layer (`plan_motion`, `execute_path`,
   `inspect_state`, `replan`, `retreat`, `finish`), the fault injector, the calibrated verifier,
   the multi-waypoint inspection task, and the episode-level trajectory recorder.
3. **The decisive experiment** (2 weeks): A0 (FSM), A1 (prompt), A2 (SFT), A4 (GRPO) on unseen
   fault combinations. This is where the H20 server becomes genuinely useful (SFT/GRPO on a small
   model), instead of as a seed trainer.
4. **The control line is written up as the substrate**: the execution-gap measurement, the
   stopping-cost model and the calibrated verifier are the measurement contribution of the agent
   paper, and they are reusable by anyone who builds a safety layer.

### 0.7 What is explicitly NOT changing

* The hardware rules (no robot connection, no motion, the whitelist and the dry-run default).
* The commit-everything policy under `data/`.
* The statistical discipline: paired scenarios, hierarchical bootstrap over training seeds, and
  an explicit separability verdict.
* The claim checker: every number in the documents is re-derived from the committed JSON.

---

## 决策摘要（中文）

- **主线**：面向位置控制机械臂的**执行感知安全过滤**。规划器负责"绕哪边走"，学习策略负责局部效率，过滤器负责"这个指令能不能真的被执行"。
- **先修评测器，再谈算法**。第一周交付一个**可信的修正评测器**；在它出来之前，不做任何新训练、不引用任何未加标签的旧数字。
- **RL 是否保留为论文主贡献，由公平比较决定**：如果"规划器＋普通跟踪器"和残差 RL 打平，论文标题里就不写 RL。
- **否证标准写在前面**（第 6 节）：如果"调优固定裕量的有界 QP"和我们的新方法打平，那么执行模型的贡献不成立——这条必须留，不能只报正结果。
- **当前进度**：审计出的 6 个 P0/P1 缺陷已核实并修复（commit `5bf50aa`，175 个测试通过）。剩余：修正评测器与冻结协议。

---

## 1. The research question

> Can explicit treatment of **command-execution uncertainty** improve the safety–completion
> trade-off of a motion-policy filter on a single position-controlled manipulator?

Deliberately narrow. Obstacle geometry is known, the arm does not grasp, there is no vision and no
language. Perception uncertainty, end-to-end VLA, and contact-rich manipulation would each add a
confounder on top of an already open question.

**Three deliverables**

1. A documented distinction between *controller consistency*, *physical accuracy*, *geometric
   collision checking*, and *executed safety* — the review's first finding is that the project has
   been quietly conflating them.
2. An execution-aware filtering method with measurable assumptions and **explicit failure
   handling** (infeasible ⇒ reported as violated, never as a certificate).
3. Matched comparisons against bounded kinematic filtering, tuned fixed-margin filtering, and
   classical planning, so any gain is attributable to the proposed mechanism.

Deliverables 2 and 3 are **proposed, not demonstrated**. Deliverable 1 is supported by the audit
in section 2.

---

## 2. Audit: what was alleged, what was verified

Severity is the **post-verification** severity. "Changes the published numbers" means the
M5 success/collision figures in `docs/LEARNING.md` and `README.md`.

| ID | Alleged defect | Verdict | Severity now | Changes published numbers? |
|---|---|---|---|---|
| **P0-A** | `safety_filter.py::cbf_policy` projects onto the barrier half-space and then clips to the command box; the clip can re-violate the constraint | **CONFIRMED** (mechanism, 2-D counterexample reproduced) | **P1** (downgraded from P0) | **Barely.** Measured on wall_ood, 3 seeds, 50 dev episodes: the clip violates the constraint in **9.6 % of the active-constraint steps** and in **46 of 50 episodes** — the mechanism is *not* rare. But every violation had a feasible in-box command (`Σ|cᵢ| ≥ g`), and the downstream effect is small: with the exact bounded projection the collision verdict changes in **1 episode out of 150**, success is unchanged in every configuration, and the mean minimum-clearance difference is **0.1 mm**. Worst violation 0.1146 = 2.29 mm of extra approach within one 20 ms step. The published claim "the filter returns a command satisfying the constraint" was false; the published *numbers* are barely affected |
| **P0-B** | `cr10a_reach.py::step` assigns `self.q_ref = clipped` before evaluating `clipped == self.q_ref`, so the limit mask is always True and the servo velocity feed-forward is never suppressed | **CONFIRMED**, severity **downgraded to P2** | P2 (latent plant-fidelity defect) | **No** — the mask fires **0 times in >1 M substeps** across all 6 archived PPO checkpoints, random actions and a Jacobian controller; the goal distribution never reaches a soft limit. When it does fire (saturated push into the elbow), the elbow settles 0.036 rad *past* its own limit with a 153 N·m torque error — real, but unreachable in the shipped tasks |
| **P0-C** | `cr10a_reach_wall.py::pair_distances` samples 32 points along the capsule axis; the sampled minimum is an **upper** bound, so the clearance can read positive while the capsule intersects | **CONFIRMED** (mechanism and counterexample) | P1 | **Rarely, but not never.** Measured against an exact segment-AABB reference on the real CR10A capsules and the real wall box: the per-pair over-statement is sub-millimetre at p99 (0.14 mm) and at most 7.1 mm (adversarial, Link2 vs wall, where the true value is already deeply negative). The sign **does** flip for 0.33 % of near-boundary configurations (10/3000 grazing configs reported safe while the geometry collides), 0.03 % at the filter's 2 cm margin and 0 % at 5 cm; the error at a flip is 0.06–0.42 mm. The Lipschitz correction removes **all** of them. The same defect exists in the M2 planning metric (`ex_planning_lab/core/capsules.py`, samples = 16 → up to 25 mm, larger than the 1–2 cm path margin the M2 report quotes), so the real-robot padding discussion must use the corrected value |
| **P0-D** | Collision is checked only at the *end* of a policy step (20 physics substeps), so a brief intersection inside a step is invisible | **CONFIRMED** | P1 | Under measurement (see §3) |
| **P0-E** | The window comparison (RRT-Connect 38 % vs PPO 0 %) is not budget- or privilege-matched: the planner gets the goal **joint configuration** and a 30 s budget against the reactive policies' 6 s | **CONFIRMED** | P1 | **Yes** — the headline "planning wins clearly" is not a controlled statement |
| **P1-a** | The wall reward replaces the base reward and drops its joint-limit penalty; the "potential-based" shaping `d_t − d_(t+1)` is not the discount-consistent `γΦ(s′) − Φ(s)` for γ = 0.99 | **CONFIRMED** | P1 | Yes, for any retrained policy; the archived ones were trained with the flawed shaping and are what they are |
| **P1-b** | The learning environment always used the **placeholder** dynamics; the `identified_run02` preset existed but was not selectable from the learning constructor | **CONFIRMED** (first-hand) | P1 | Yes — every archived policy is a policy on the *placeholder* plant, not on the identified one |
| **P1-c** | `evaluation/dynamics_id.py` filters the whole series with `filtfilt` **before** splitting into time segments, coupling the split boundary | **CONFIRMED** (mechanism), **measured impact negligible** | P3 (methodological cleanliness) | **No.** With the split done first and each segment filtered separately (`--split-before-filter`), the archived run02 result moves by **≤ 0.1 VAF percentage points**: M1 97.2/97.3/98.5 → 97.2/97.3/98.4, M2 97.5/97.4/98.7 → 97.5/97.4/98.6, mass scale k = 1.636 unchanged, Coulomb unchanged. The leakage is real as a mechanism and irrelevant in magnitude on this recording. The fix is kept because a reviewer will ask |
| **P1-d** | `run_safe_eval.py` records only model/task/filter; the margin, α, environment, commit and checkpoint hash are missing; `cbf_policy`'s default margin is 2 cm while the evaluation script uses 5 cm | **CONFIRMED** (first-hand) | P2 (blocks reproducibility, not correctness) | No, but the archived JSONs cannot be reproduced from themselves |

### A finding the review did not anticipate

The environment's collision detector is **sound but loose**: it answers "did the *enclosing
capsule* touch the wall?", not "did the *arm* touch the wall?". The CR10A's link capsules have
radii of 61–150 mm, so the capsule can be in contact while the mesh is still **13 cm** away
(measured at the home pose: capsule clearance 0.054 m vs mesh clearance 0.130 m).

The consequence is not that the published collision rates are too *high* in a way that flatters
the method — the opposite. It means **the published numbers measure a different physical event
than the one the paper talks about**, and the independent mesh-based checker must be the arbiter
from now on. The legacy↔corrected comparison is running; its result decides whether the
published table is re-derived.

**Already fixed** (commit `5bf50aa`, 175 tests pass, 15 new regression tests in
`learning/tests/test_audit_fixes.py`): P0-A (bounded projection in
`learning/ex_learning/qp_filter.py`), P0-B (mask captured before the overwrite), P0-C
(`conservative_geometry` makes the clearance a rigorous lower bound), P0-D
(`substep_collision_check`), P1-b (`dynamics` is now a config field and is recorded).

**Design decisions taken in the fix**

- The **legacy behaviour stays the default** in every case (`conservative_geometry=False`,
  `substep_collision_check=False`, `cbf_policy_legacy`). The archived evaluations therefore stay
  reproducible, and the difference between *"the evaluation changed"* and *"the method improved"*
  can be attributed instead of conflated.
- The corrected numbers are written to **new** files. Nothing archived is overwritten.

---

## 3. The frozen evaluation protocol

Before any new number is produced, the protocol is frozen and versioned. Seeds 1000–1049 were
inspected while writing the existing results, so they are **historical development data** and can
never be a confirmatory test set again.

| Split | Seeds | Purpose |
|---|---|---|
| `train` | 0–999 | policy training (unchanged) |
| `tune` | 0–19 | hyper-parameters, margins, gains (unchanged) |
| `dev` | 1000–1049 | **the historical set** — the archived results live here; used for mechanism analysis and for the legacy↔corrected comparison |
| `test` | 20000–20199 (new, untouched) | the only set any new claim may be reported on |

Every reported number must carry: the manifest version, the episode seed range, the training seed,
the filter settings (margin, α, bounds), the plant preset, the collision evaluator version, the
git commit, and the checkpoint hash. `evaluation/episode_manifest.py` produces the manifest;
`learning/ex_learning/run_safe_eval.py` writes the manifest hash into every output JSON.

**Common comparison rules** (all methods, all scene families): same start/goal pools, same goal
information, same collision evaluator, same execution deadline, same success criterion, same
information access. Privileged results (the `joint_interp` diagnostic, the window planner's goal
joint configuration) are reported **separately**.

---

## 4. Method ladder and the decision it drives

| Group | Controller | Purpose |
|---|---|---|
| B0 | APF, DLS | legacy sanity checks |
| B1 | PPO, no filter | the policy-failure reference |
| B2 | PPO + the legacy single projection | the historical filter baseline (`cbf_policy_legacy`) |
| B3 | PPO + **bounded all-pair kinematic QP** | isolates the basic filter corrections |
| B4 | B3 + **tuned fixed clearance margin** | the strong conservatism baseline |
| B5 | **execution-aware filter** | isolates prediction + stopping verification |
| B6 | planner + waypoint follower + B4/B5 | separates global route selection |
| B7 | B6 + bounded residual learning | tests whether RL adds anything |
| B8 | cuRobo / robust predictive control, reproducibly adapted | an external strong comparison |

The claim under test is **not** "our filter is better than the old one". It is:

> *Given a position-command interface whose execution lags the request, does accounting for that
> lag — rather than inflating a fixed margin — move the safety–completion frontier?*

B4 is the baseline that decides it. A fixed margin is the cheapest possible way to buy safety,
and if it buys the same thing, the execution model is decoration.

---

## 5. Six weeks

### The compute rule (standing, from 2026-11-06)

**Every experiment gets a compute assessment before it is designed**, and the decision is written
into `docs/PROGRESS.md` with the experiment: *does this need more than the laptop?* If the
laptop is enough, the experiment runs on the laptop.

An NVIDIA H20 server is available over SSH (the owner's infrastructure; the target lives in the
git-ignored `scripts/remote/remote_config.local.sh`). The assessment and the procedure are in
`docs/REMOTE_COMPUTE.md`; the scripts are in `scripts/remote/`.

The assessment so far: the H20's **VRAM is not the binding constraint anywhere in this plan** —
the MuJoCo plant is CPU-bound and the policies are small MLPs, so the server's value is its core
count (many training seeds in parallel), which is exactly what the statistical weakness of the
headline comparison needs. The one place a GPU genuinely matters is the residual-RL arm (B7) in
Week 5. The server runs **simulation only**; the motion-capable package is deliberately not
copied there.

### Week 5 prerequisite added: the seed count

The confirmatory run showed that **three training seeds cannot separate an 8-point mean
difference from an 8-point between-seed spread**. The plan therefore adds a compute item: train
enough additional seeds to make the spread an estimate rather than an anecdote — 6 locally, and
12 more on the server when the owner connects it.

---

Each week has a **checkable deliverable** and a **decision gate**. The gate is the point: it is
allowed to stop the project.

### Week 1 — repair and mechanism diagnosis *(in progress)*

- ✅ P0-A/B/C/D + P1-b fixes with 15 regression tests (`5bf50aa`)
- ✅ `evaluation/independent_collision.py`: a checker that does **not** reuse the environment's
  distance code — mesh surface samples, an exact point-to-AABB distance, a penetration depth for
  containment, and the self-collision pairs defined in the MoveIt SRDF (the environment checks
  link-vs-box only). Validated against the environment: the capsule axes coincide to 1e-6 and
  every mesh point lies inside its capsule (max excess 0.000000 m) (`6e29b71`)
- ✅ `evaluation/episode_manifest.py`: the frozen manifest of §3
- ✅ `evaluation/compare_evaluators.py`: replays the *same recorded actions* through the three
  detectors, so the physics is identical and only the detector differs. Its self-consistency
  assertion caught two bugs of its own — most importantly that **the episode sampling depended on
  the evaluator** (the corrected clearance is lower, so the rejection sampling accepted different
  start/goal configurations and the two evaluators were not comparing the same episodes at all)
- ⬜ the legacy↔corrected↔independent comparison over the 3 × 3 × 50 dev episodes (running)
- ⬜ per-episode diagnosis of the residual OOD collisions: which link, which box, the
  reference-tracking error `q_ref − q`, the pre/post-projection residual, the clip mask, the
  sub-step minimum distance — allowing several causes per episode

**Gate.** If the corrected evaluator moves the collision counts materially, the published table is
re-derived *before* any new training. If the detection itself is still not trustworthy, no training
happens in week 2 either.

> **Gate result (2026-11-05): PASSED.** The corrected evaluator changes the verdict in at most
> **5 of 50** episodes, and only in the raw-policy out-of-distribution cells; 16 of the 18 cells
> are bit-identical. The published table is not an artefact of the evaluator. Full report, with
> the measured effect of every audited defect and the two bugs of my own that the reproducibility
> checks exposed: **`docs/EVALUATOR_AUDIT.md`**.

### Week 2 — bounded kinematic baseline (B3, B4)

- ⬜ the all-pair QP with the command bounds and joint limits **inside** the optimisation; the
  current sequential projection is not a QP and can violate a constraint it already satisfied
- ⬜ infeasibility, solver timeout, numerical residual and ill-conditioned distance gradients are
  logged, never smoothed over
- ⬜ margin sweep → the tuned fixed-margin baseline B4, with controller latency (p50/p95/p99)

**Gate.** If B4 already matches the proposed method on the collision–completion–latency frontier,
the execution-model contribution is **declared void** and the paper becomes an honest
fixed-margin study.

### Week 3 — command-channel identification (needs the owner on site)

- ⬜ characterise the **actual** command interface in an unobstructed, bounded workspace: latency,
  tracking error, queue behaviour, **stopping envelope**, and what happens to already-queued
  commands when a stop is requested
- ⬜ the existing whitelist and dry-run contract stay in force; nothing goes near the robot without
  the owner present at the e-stop
- ⬜ the recorded traces become the training/calibration/test sets for the predictor (split by
  trajectory *and* recording session)

**Gate.** The interface determines the method. If the controller only accepts queued `MovJ`
waypoints, the method becomes **segment-level prediction verification**, not a 50 Hz filtered
control loop — and the paper says so. This is the first experiment that decides whether the
topic exists at all; it is not a formality.

#### What this needs from the owner (everything else is done)

The simulation-only work of Weeks 1, 2 and 4 is complete. What is left needs the robot, and
nothing may run without the owner present at the e-stop. Concretely:

1. **Authorisation to run, in this order, in an unobstructed bounded workspace:**
   a. a **read-only** capture of the command path: one slow joint-space move, recording the
      command timestamps, the controller's acknowledgement, the feedback stream and the queue
      state — this opens no motion code and changes no state;
   b. a **step-response characterisation**: for a handful of small, slow, single-joint commands,
      record the executed motion against the commanded reference, so the latency, the tracking
      lag and the **stopping envelope** can be measured instead of assumed;
   c. only then, a **stop test**: issue a command and then a stop, and record what the arm
      actually does — in particular whether already-queued waypoints keep executing.
2. **A decision on the interface question**: if the controller only accepts queued `MovJ`
   waypoints, the method becomes segment-level prediction verification, and the paper says so
   instead of pretending a 50 Hz filtered loop exists.
3. The existing **whitelist, caps and dry-run default stay in force**; the motion module is not
   modified, and `ex_dobot_motion --execute` is not run without an explicit in-conversation
   confirmation that the owner is at the robot with the e-stop in reach.

Everything measured in week 3 becomes the training/calibration/test set of the predictor, split
by trajectory *and* recording session — the same discipline the simulation work used.

### Week 4 — execution-aware filter (B5) and the main ablations

- ⬜ short-horizon prediction of the augmented state `(q, q̇, q_ref − q, command history, feedback
  age)` with a measured delay buffer
- ⬜ the uncertainty budget `m = m_geom + m_frame + m_track + m_time`, with the terms separated and
  **no double counting** (a delay already propagated through the predictor is not added again)
- ⬜ a stopping rollout as a terminal feasibility condition — a zero velocity command does **not**
  stop the physical motion
- ⬜ every candidate re-checked by nonlinear rollout including the stopping tail; on failure, a
  previously verified fallback, never a silently relaxed constraint
- ⬜ ablations: single → multi constraint · clip → bounds-in-QP · fixed → velocity-dependent
  margin · no prediction → prediction · no stopping tail → stopping tail · placeholder →
  identified dynamics · no history → reference history

**Gate.** The predictor must have honest held-out coverage, including the worst tail near braking.
If it is badly under-covered on held-out sessions, the near-obstacle hardware stage does not
proceed.

#### What was completed in Week 4 WITHOUT the robot (2026-11-05)

The command-channel identification and the hardware stages need the owner on site, so the
simulation-only part of Week 4 was done first. It is recorded here because it **triggered
falsification criterion 1**.

- ✅ the channel of the SIMULATED plant is measured: the arm coasts **14.9 mrad** at a 1 rad/s
  command after the command is cut (`evaluation/measure_execution.py`,
  `data/reports/learning/execution_channel.json`);
- ✅ a second-order execution model is identified on that plant (ω = 37.7 rad/s, ζ = 0.90,
  v_gain = 0.80) with a **4.6 mrad RMS** prediction error over 5 policy steps, flat within ±20 %
  of the identified ω;
- ✅ the execution-aware filter (B5) is implemented as a look-ahead on the predicted motion, with
  the command bounds inside one bounded program and an explicit relaxation report for the
  infeasible case;
- ✅ the **execution gap** is measured on real episodes: the bounded kinematic QP's own
  constraints were satisfied while the arm ended the control step below the enforced margin in
  9.2 % of the steps at a 2 cm margin (32.4 % at 4 cm), median prediction error −12 mm. The
  look-ahead formulation reduces this to 0.1–1.3 %;
- ✅ the **ablation** (`account_execution=False`, the coast term removed) shows that the
  improvement comes from the **look-ahead structure**, not from the explicit execution model: the
  frontier is identical, and the filter's own claim is *more* valid without the coast;
- ❌ **the stopping rollout, the separated uncertainty budget and the hardware session are not
  done** — they need the robot.
- ✅ the **confirmatory run on the frozen test range** (20000–20199, never inspected before) with
  three detectors per episode: the execution-aware filter reproduces the completion of an
  **unconstrained-margin** QP (54.0 % out of distribution) with **0 % collisions**, where that QP
  collides in 16–22 % of the same episodes — in all three training seeds. Its completion
  advantage over the *safe* fixed-margin QP is **not** established (+22/+2/+2 points against an
  8-point between-seed spread);
- ✅ the **robustness check on the identified plant** (`--dynamics identified_run02`: 40.6 kg
  instead of the placeholder's 24.8 kg): the same pattern and the same numbers to within a point.

**Consequence for the plan.** Falsification criterion 1 is **partially triggered** (see §6). The
completion advantage over the safe fixed-margin QP does not survive the between-seed spread, the
latency cost of the method is real, and the ablation removes the execution model from the
explanation. What survives is narrower and still useful: **a filter that reasons about the command
is not reasoning about the motion**, and replacing the rate condition with a position condition
over a short horizon is what fixes that — the plant model is not needed for the fix, only a
prediction of the motion. The next work items are therefore the *predictor's* error budget (a
shorter horizon, an error-aware margin, and a predictor that is valid under the commands the
filter actually issues) and a **tuning split outside the training range**, since the current one
(seeds 0–19, inside the training range 0–999) cannot resolve the differences the method produces.

### Week 5 — planner guidance and the value of RL (B6, B7)

- ⬜ a waypoint follower as the primary nominal controller (B6)
- ⬜ a bounded residual learner on top (B7), trained **through** the same filter used at
  evaluation, with the replay buffer storing the proposed action, the executed action and the
  intervention flag
- ⬜ the window scene re-run under a **matched** protocol: same deadline, same goal information
  (Cartesian for everyone), same collision model, same success criterion

**Gate.** If the waypoint follower matches the residual learner, **RL is removed from the paper's
contribution and from its title.** The negative result is kept and reported.

### Week 6 — compliant hardware trials, statistics, figures

- ⬜ 4 methods × 3 static scene types × 20 attempts = 240 attempted trials, sized against the pilot
  variance and the manual reset cost
- ⬜ **no unsafe raw-policy baseline is executed on hardware** to obtain a collision count; it is
  evaluated in simulation and the hardware omission is stated
- ⬜ a physical intervention or a protective stop counts as a safety intervention and an
  unsuccessful autonomous completion
- ⬜ every trial logged with firmware, tool, payload, scene geometry, controller settings,
  acknowledgements/queue ids, host and controller clocks, `q`/`q_target`/`q̇`, the command, the stop
  trigger, the actual stop time, and operator intervention
- ⬜ Figures 3–6 of the manuscript filled from real data only; Figure 2 stays labelled as a
  proposed architecture until it is implemented

**Note.** The current runner's feedback read and Dashboard round-trip are serial. The feedback
source rate is not a safety-loop rate, and the manuscript must not describe it as one.

---

## 6. Falsification criteria (stated before the results)

These are the conditions under which the central claim is **declared false**. They are recorded
here so that a negative outcome cannot be quietly reinterpreted later.

1. If a **tuned fixed-margin bounded QP (B4)** matches the proposed method on collision rate,
   completion and latency → *the execution-model contribution does not hold.*
   **PARTIALLY TRIGGERED (2026-11-06, simulation only).** The three parts come out differently:
   * **collision rate — MATCHED.** Both the QP at 4 cm and the execution-aware filter are at
     0 % envelope collisions on the frozen test range, in all three training seeds;
   * **completion — NOT SEPARATED.** Out of distribution the execution-aware filter reaches
     54.0 % against the QP's 45.3 %, but the per-seed differences are +22, +2 and +2 points
     against a between-seed spread of 8 points. Three training seeds cannot resolve an 8-point
     mean from an 8-point spread, so the paper reports the two as **indistinguishable on
     completion at equal safety**, not as a win;
   * **latency — TRIGGERED AGAINST THE METHOD.** The execution-aware filter costs ~8× more per
     control step than the QP (a 14-way batched prediction and a larger constrained program);
   * and the **execution-model mechanism itself is not supported**: the ablation removes the
     coast term and changes nothing, so the benefit belongs to the look-ahead constraint
     structure, not to the plant model.

   What *is* established, in all three seeds, is a different and narrower statement: the
   look-ahead constraint reproduces the completion of an **unconstrained-margin** QP (54.0 %)
   while that QP collides in 16–22 % of the same episodes. The paper claims that, and not the
   completion advantage over the safe fixed-margin QP.
2. If a **route follower (B6)** matches the residual learner (B7) → *RL is not a contribution of
   this paper.*
3. If the predictor's held-out coverage is poor, especially in the braking tail → *the method
   does not advance to the near-obstacle hardware stage.*
   **Partially triggered (2026-11-05).** The identified model has a 4.6 mrad RMS error over
   five policy steps — a third of the coast it represents — and the prediction assumes the
   command is held constant, an assumption every filter intervention violates. The near-obstacle
   hardware stage should not proceed on this predictor.
4. If the corrected evaluator changes the archived conclusion → *the archived table is withdrawn
   and re-derived before anything new is claimed.*
   **NOT triggered.** The verdict changes in at most 5 of 50 episodes and not at all in 16 of the
   18 cells; the table stands, with the collision axis re-labelled as the safety envelope.
5. Zero collisions in N independent trials is **not** a guarantee: 50 independent trials still
   permit a one-sided 95 % upper bound of ≈ 5.8 %. Reusing the same 50 scenes across 3 training
   seeds is 150 policy–episode evaluations, **not** 150 independent scenes, and the paper says so.

---

## 7. Explicitly not doing (yet)

Not because they lack value, but because they dilute the single falsifiable question: LLM agents,
GRPO, VLA, T265-based NBV, a full Isaac perception stack, a full multi-constraint CBF with a
formal safety proof, and cross-embodiment generalisation. A single CR10A cannot support a
cross-robot claim.

Also not a contribution: "standard all-constraint QP + SAC + PointNet" assembled from parts. The
candidate contribution is a *constrained, empirically validated adaptation to a position-command
interface*, including the identification and verification of the execution envelope — and it is
still a candidate until the matched experiments exist.

---

## 8. Where the paper must be careful about wording

The review's most useful contribution was not a bug list but a list of sentences that would be
attacked in review. The project has already corrected the ones it found:

| Do not write | Write instead |
|---|---|
| "the robot's absolute accuracy improved to 0.002 mm" | "the controller's coordinate mapping is reproduced across sessions with a 0.002–0.011 mm residual" |
| "we proved the URDF masses are 1.64× too light" | "a uniform inertial scale of ≈1.64 improves prediction on local data, under the assumption that the torque field is in N·m" |
| "the CBF guarantees zero collisions" | "under the original evaluator the OOD collisions fell from 54/150 to 7/150; there is no formal guarantee and no hardware result" |
| "RL has learned general obstacle avoidance" | "training on randomised observed walls improves in-distribution reaching; size-OOD walls and new topologies still expose substantial failures" |
| "success roughly doubles out of distribution" | "OOD success rises 21 % → 31 %: +10 points, +48 % relative — and it remains below the potential field's 50 %" |
| "planning beats learning on narrow passages" | "the planner's 38 % used the privileged goal joint configuration and a 30 s budget against the reactive policies' 6 s; a matched comparison is pending" |

---

## 9. Links

- Per-stage documents: `docs/{HARDWARE_COMPATIBILITY,CALIBRATION,M2_PLANNING_LAB,M4_MUJOCO,DYNAMICS_ID,LEARNING}.md`
- Progress log: `docs/PROGRESS.md` · roadmap: `docs/ROADMAP.md`
- Self-contained brief for external readers: `docs/RESEARCH_STATE.md`
- Fixed code: `learning/ex_learning/qp_filter.py`, `learning/ex_learning/safety_filter.py`,
  `learning/ex_learning/envs/cr10a_reach_wall.py`
- Regression tests for the audit: `learning/tests/test_audit_fixes.py`
