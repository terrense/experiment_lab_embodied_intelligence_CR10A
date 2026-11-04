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
| **P0-C** | `cr10a_reach_wall.py::pair_distances` samples 32 points along the capsule axis; the sampled minimum is an **upper** bound, so the clearance can read positive while the capsule intersects | **CONFIRMED** (mechanism). **The specific "missed collision" does NOT occur with the real CR10A capsules** | P1 | **Not as a missed collision.** The CR10A's enclosing radii are 61–150 mm and dominate the 0–12 mm sampling error, so the sign never flips. What *does* happen: the clearance is systematically **over-stated by up to 12 mm** (Link2), against a filter margin of only 20–50 mm — the filter brakes up to 12 mm too late |
| **P0-D** | Collision is checked only at the *end* of a policy step (20 physics substeps), so a brief intersection inside a step is invisible | **CONFIRMED** | P1 | Under measurement (see §3) |
| **P0-E** | The window comparison (RRT-Connect 38 % vs PPO 0 %) is not budget- or privilege-matched: the planner gets the goal **joint configuration** and a 30 s budget against the reactive policies' 6 s | **CONFIRMED** | P1 | **Yes** — the headline "planning wins clearly" is not a controlled statement |
| **P1-a** | The wall reward replaces the base reward and drops its joint-limit penalty; the "potential-based" shaping `d_t − d_(t+1)` is not the discount-consistent `γΦ(s′) − Φ(s)` for γ = 0.99 | **CONFIRMED** | P1 | Yes, for any retrained policy; the archived ones were trained with the flawed shaping and are what they are |
| **P1-b** | The learning environment always used the **placeholder** dynamics; the `identified_run02` preset existed but was not selectable from the learning constructor | **CONFIRMED** (first-hand) | P1 | Yes — every archived policy is a policy on the *placeholder* plant, not on the identified one |
| **P1-c** | `evaluation/dynamics_id.py` filters the whole series with `filtfilt` **before** splitting into time segments, coupling the split boundary | Under independent verification | — | Under verification |
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
2. If a **route follower (B6)** matches the residual learner (B7) → *RL is not a contribution of
   this paper.*
3. If the predictor's held-out coverage is poor, especially in the braking tail → *the method
   does not advance to the near-obstacle hardware stage.*
4. If the corrected evaluator changes the archived conclusion → *the archived table is withdrawn
   and re-derived before anything new is claimed.*
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
