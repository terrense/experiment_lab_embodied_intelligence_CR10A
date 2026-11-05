# Execution-Aware Safety Filtering for Learned Motion Policies on a Position-Controlled Collaborative Manipulator

**Xin Shen** and collaborators — *authors and affiliations to be confirmed*

## Abstract

Learned motion policies can perform well in familiar scenes while producing unsafe commands after a
change in obstacle geometry. Deploying a kinematic safety filter is insufficient when its velocity
command is integrated into a position reference and tracked by a delayed, bandwidth-limited servo.
We study this mismatch using an existing Dobot CR10A research platform with controller-aligned
kinematics, recorded hardware excitation, a MuJoCo plant, and simulation-only reaching policies.

A source audit of the archived evaluation pipeline exposes four defects and two methodological
gaps, each reproduced before it was fixed: a barrier projection that could violate its own
constraint (in 9.6 % of the active-constraint steps), a dead joint-limit mask, a capsule-to-box
distance that is an over-estimate rather than a lower bound, and a collision test that only ran at
the end of a control step. The corrected evaluation changes the verdict in at most five of fifty
episodes, so the published table survives — but an independent mesh-geometry checker shows that the
collision axis of that table counts the *conservative safety envelope* (the enclosing link
capsules), not the arm: across 300 evaluated out-of-distribution episodes the arm never touched
the wall, while the envelope came within 17–65 mm of it. We therefore reformulate the claim and
report a bounded all-pair kinematic QP as a strong baseline, on the frozen, untouched test range.

The manuscript states the method, the conditional geometric reasoning and the evaluation protocol.
Implementation of the proposed execution-aware extension and closed-loop hardware experiments
remain pending; no performance improvement or physical safety guarantee is claimed for it.

**Keywords:** robot motion planning; safety filtering; sim-to-real; reinforcement learning;
collaborative manipulators.

---

## 1 Introduction

A collision-free command is not necessarily a collision-free execution. This distinction matters
for industrial collaborative manipulators whose accessible interface accepts position
trajectories rather than direct joint torques. A learned policy may request an admissible joint
velocity, while an internal reference integrator, command queue, communication delay, and
proprietary servo determine the physical trajectory. If a safety layer treats requested velocity
as measured velocity, its mathematical model omits precisely the dynamics that dominate braking
near an obstacle.

The CR10A platform studied here makes the issue concrete. Existing experiments combine a ROS 2
state pipeline, MoveIt planning, controller-model alignment, offline identification, and a MuJoCo
servo simulation. PPO improves reaching around familiar walls relative to a local artificial
potential field controller. Nevertheless, randomizing and observing the wall does not eliminate
extrapolation failures: policies come close to taller and wider walls than they were trained on.
A minimally invasive projection reduces the number of episodes in which the *safety envelope*
touches the wall, but the measured effect is smaller than the published wording suggested, and the
audit below shows why.

The present study asks a narrower question than general robot intelligence: can explicit treatment
of command execution uncertainty improve the safety–completion trade-off of a motion-policy filter
on a single position-controlled manipulator? We deliberately keep obstacle geometry known in the
first study. Perception uncertainty, end-to-end vision-language-action learning, grasping, and
contact-rich manipulation would each add a confounder on top of an already open question.

Three deliverables define the work. First, we document the distinction between controller
consistency, physical accuracy, geometric collision checking, and executed safety — including the
case where the project's own collision metric turned out to measure a different physical event
than the one its wording suggested. Second, we specify an execution-aware filtering method with
measurable assumptions and explicit failure handling. Third, we define matched comparisons against
bounded kinematic filtering, fixed-margin filtering, and classical planning, so that any future
gain can be attributed to the proposed mechanism. The first deliverable is supported by the audited
implementation and archived records; the latter two constitute the proposed study and must not be
presented as completed experiments.

## 2 Related work and positioning

**Neural MP** learns reactive motion generation from planner-generated demonstrations and combines
the learned policy with test-time optimization [1]. Its relevance is the separation between a fast
learned proposal and geometric verification. Our first experiments need neither its full training
scale nor its robot-specific weights: a CR10A planner can provide references and demonstrations
within the existing simulator.

**Deep Reactive Policy** uses point-cloud observations, large-scale expert trajectories, and
iterative student–teacher refinement for reactive manipulation [2]. It motivates richer geometry
and closed-loop corrections, but reproducing its ten-million-trajectory pretraining is outside this
project's initial scope. A compact per-link geometric observation is an engineering baseline here,
not a claim to reproduce DRP.

**Operational Space Control Barrier Functions** explicitly consider task-consistent objectives, many
simultaneous constraints, and kinematic or dynamic control [3]. This is the closest conceptual
baseline: replacing a single half-space with a bounded multi-constraint program is established
practice, not a standalone contribution. The remaining practical question is how to represent the
accessible CR10A command channel when torque-level implementation is unavailable.

Recent journal work also addresses constrained robotic dynamics. Wang et al. introduce augmented
dynamics for high-order barrier-based control under input-output constraints [4]. Wullt et al.
combine robust tube MPC and corridor planning for a simulated six-degree-of-freedom industrial
manipulator with model uncertainty [5]. Dai et al. formulate time-varying geometric barriers
through differentiable optimization and demonstrate obstacle avoidance with a Franka Research 3
[6]. These works prevent a broad novelty claim such as "the first dynamics-aware safe manipulator
controller". Our candidate contribution is a constrained, empirically validated adaptation to a
position-command interface, including the identification and verification of the execution
envelope.

Planning and learning can be complementary. **SPIRE** uses task and motion planning to structure
imitation and reinforcement learning subproblems [7], while **XMoP** explores motion policies
across robot embodiments [8]. We retain the planning–learning decomposition but restrict
embodiment to CR10A. A cross-robot generalization claim would require additional robots and is
not supported here. **cuRobo** supplies an additional strong GPU planning baseline [9]; its
original publication predates the main 2024–2026 reading window and is included for technical
relevance.

## 3 Platform and evidence boundaries

The audited repository snapshot is `c5c8858c4a9261daa5caae475c844ed7182910bd`, retrieved on
2 November 2026 [10]. Code acquisition and static review do not constitute an independent rerun
of ROS, MuJoCo training, or hardware experiments. The existing platform uses ROS 2 Humble on
Ubuntu 22.04 under WSL2. Historical training records describe CPU PPO and a laptop configuration.
New compute measurements must be recorded rather than inferred from GPU names.

### 3.1 Controller alignment

The state pipeline parses Dobot feedback, validates units and ordering, and compares URDF forward
kinematics with controller-reported TCP coordinates. Recorded sessions support millimetre-scale
agreement for the nominal CR10 geometry (RMS 1.6–3.0 mm, max 5.54 mm over 41 357 samples) and a
substantially larger discrepancy for the alternative CR10AF geometry. A regularized 45-parameter
calibration has an identifiable rank of 25 and reduces cross-session controller-coordinate
residuals to 0.002–0.011 mm.

These targets are computed by the controller from joint angles. Consequently, the calibration
identifies a **controller-consistent** kinematic model; it is not an independent measurement of
physical positioning accuracy. A physical clearance budget must include external geometry and
frame errors that cannot be estimated from controller self-consistency alone.

### 3.2 Dynamics and command execution

The simulation integrates physics at 1 kHz and policy actions at 50 Hz. Actions are bounded
six-dimensional joint-velocity references with a nominal scale of 1 rad/s, integrated into a
position set-point. A PD servo with gravity/Coriolis feed-forward and torque saturation tracks
that set-point. Simulated link geometries are visual-only; collision events are detected by a
separate geometric checker, not by simulated contact forces.

Hardware excitation run02 contains slow motion near one posture with a 0.4 kg flange attachment.
Offline regression uses the feedback field `m` as an assumed torque signal. A global inertial scale
of approximately 1.636 and friction adjustments improve prediction of this signal in a held-out
temporal segment. However, its units have not been independently established, the excitation is
limited, and some fitted terms are not physically identifiable. We therefore interpret these values
as local predictive parameters. The torque fit does not identify the command-to-motion transfer
function, and the simulation servo bandwidth remains a separate assumption.

The current learning constructor uses placeholder dynamics; the identified preset exists in the
simulation package but is not selected by the learning constructor. Likewise, the hardware runner
streams queued MovJ waypoints. It does not establish an equivalent 50 Hz velocity-control
interface. A new execution adapter and measured interface characterization are prerequisites for
deploying the proposed policy filter.

### 3.3 Existing tasks and evaluation

Existing wall tasks sample start and goal configurations around a reference motion. Random-wall
tasks vary wall center, height, and width; the observed version appends the wall center and half
sizes. The OOD split increases wall height and width beyond the training range. A narrow-window
task draws configurations from fixed pools.

Success currently means a TCP position error below 20 mm at episode termination, with no detected
collision. There is no orientation requirement or terminal dwell criterion. The archived results
therefore describe **position reaching under the original detector**, not full-pose manipulation,
continuously collision-free execution, or hardware autonomy.

## 4 Audit findings that motivate the method

Every allegation below was independently re-derived from the code before anything was changed, and
the ones that mattered were measured against an exact reference rather than against the code under
test. The audit harness was validated by reproducing an archived result exactly: running the
archived protocol on the OOD wall task, 50 episodes from seed 1000, gives success 0.30, collision
0.04 and final distance 0.0289 m — the numbers in the archived evaluation file. The audited code
is the code that produced the published numbers.

### 4.1 Command-space projection is not execution-space invariance

The existing filter finds the nearest link–obstacle pair and imposes `c·a ≥ g`, where `c` is a
distance Jacobian and `a` is the normalized requested velocity. This is appropriate only under a
matching velocity-level model. In the environment, `a` updates `q_ref` and the servo determines
`q̇`. Hidden reference accumulation can persist even when the safety layer changes the next
command. The policy observation includes `q` and `q̇` but not `q_ref − q` or the command history.
Execution prediction should therefore use an augmented state rather than assume `q̇ = a·v_max`.

### 4.2 Projection followed by clipping

Projection onto one half-space and subsequent componentwise clipping does not solve projection
onto the intersection of that half-space with a box. A two-dimensional algebraic example uses
`a = (1, −1)`, `c = (1, 0.1)`, and `g = 1.05`. Half-space projection produces
`(1.148515, −0.985149)`, satisfying `c·a = 1.05` exactly. Clipping to `[−1, 1]` gives
`c·a = 0.901485`, which violates the constraint even though the bounded feasible point
`(1, 0.5)` exists.

In the real simulation the mechanism is not rare: the post-clip action violates the constraint in
**9.6 % of the active-constraint steps** and in **46 of 50 episodes**. Every one of the 583
violations had a feasible command inside the box (`Σ|cᵢ| ≥ g`); the filter simply failed to find
it. The downstream effect, however, is small: with an exact bounded projection the collision
verdict changes in one episode out of 150 and the success rate is unchanged in every configuration
tested, with a mean minimum-clearance difference of 0.1 mm. The *claim* that the filter returns a
command satisfying its constraint was false; the published *numbers* are barely affected.

### 4.3 Geometry and temporal discretization

The capsule approximation encloses the link mesh, but the implemented capsule-to-box calculation
samples only 32 points along the capsule axis. The sampled minimum is an **upper** bound on the
continuous minimum, so the reported clearance over-states the true one by up to `L/(2(N−1))`
(12 mm for Link2). Measured against an exact segment-to-AABB reference on the real CR10A
capsules and the real wall box, the per-pair over-statement is sub-millimetre at p99 (0.14 mm) and
at most 7.1 mm; the collision sign flips for 0.33 % of near-boundary configurations (0.03 % at the
filter's 2 cm margin) with an error of 0.06–0.42 mm. Because the distance to a convex set is
1-Lipschitz, subtracting the sampling error makes the reported value a rigorous **lower** bound
and removes every observed miss.

The learning environment checks clearance after a policy step rather than every physics substep,
so a brief intersection can be missed. Its link–box pair list does not implement robot
self-collision.

### 4.4 Additional implementation and evidence issues

In `ReachEnv.step`, `q_ref` was assigned the clipped reference before evaluating
`clipped == self.q_ref`, which is always true for finite values, so the intended clipping mask
never suppressed the velocity feed-forward. The defect never fires in any archived policy (0 times
in more than 10⁶ sub-steps across six checkpoints, random actions and a Jacobian controller), but
when it does the elbow settles 0.036 rad past its own intended limit with a 153 N·m torque error —
a latent plant-fidelity defect rather than a numbers problem.

The wall reward replaces the base reward and omits its explicit joint-limit penalty. The progress
reward is described as potential-based, but with PPO discount 0.99 the expression
`d_t − d_(t+1)` is not the discount-consistent potential term `γΦ(s_(t+1)) − Φ(s_t)`; it is a
useful shaping signal, but policy invariance must not be claimed for it. The identification
pipeline filtered the full time series before temporal splitting, which couples the split boundary
through zero-phase filtering; splitting first and filtering each part separately changes the
held-out result by ≤ 0.1 VAF percentage points, so the published identification numbers stand.

### 4.5 The collision metric is the safety envelope, not the arm

The environment's detector asks whether the **enclosing link capsule** touched the obstacle. The
CR10A's capsules have radii of 61–150 mm around much smaller meshes, so the capsule can be in
contact while the mesh is still centimetres away. An independent checker — mesh surface samples,
its own distance code, a penetration depth for containment, and a self-collision test over the link
pairs the MoveIt SRDF does not disable — was validated against **MuJoCo's own compiled mesh
vertices** (agreement: 0.000 mm) and applied to the real episodes:

| policy seed | filter | legacy detector | mesh checker | mesh still away (min / median) |
|---|---|---|---|---|
| 0 | none | 19 / 50 (38 %) | **0 / 50** | 23 mm / 42 mm |
| 0 | CBF | 2 / 50 (4 %) | **0 / 50** | 24 mm / 38 mm |
| 1 | none | 18 / 50 (36 %) | **0 / 50** | 21 mm / 42 mm |
| 1 | CBF | 3 / 50 (6 %) | **0 / 50** | 27 mm / 40 mm |
| 2 | none | 17 / 50 (34 %) | **0 / 50** | 17 mm / 46 mm |
| 2 | CBF | 3 / 50 (6 %) | **0 / 50** | 25 mm / 44 mm |

**Not one of the 300 episodes contains a real mesh collision.** The published out-of-distribution
collision rates are, in the physical sense, false positives of a conservative envelope model: the
arm never touched the wall; the envelope came within 2–6 cm of it. The success rates are unaffected,
and keeping a conservative envelope clear is a meaningful engineering objective — but the event
being counted has to be named. From here on the paper reports the mesh clearance with the envelope
clearance alongside it, and the claim becomes "the filter keeps the safety envelope off the wall",
not "the filter prevents the arm from colliding".

## 5 Proposed execution-aware filter

### 5.0 What was implemented, and what it is not

The method of this section was **implemented and measured in simulation** before it was written
up, and the measurement changed the claim. The implementation is the B5 arm of the plan
(`learning/ex_learning/execution_model.py`, `safety_filter.cbf_policy_exec`); the results are
§7.4–§7.6. Three things are true of it and are stated here so that no reader infers more than
the evidence supports:

1. the **execution model is the simulated plant's own channel**, identified from the simulator
   (§7.4). It is not a model of the real CR10A command interface, and nothing here transfers to
   it without a hardware characterization;
2. the filter's constraint is a **look-ahead on the predicted motion**, not a rate condition on
   the command. The ablation in §7.6 separates the two contributions, and the result is that the
   look-ahead carries almost all of the benefit — a finding that qualifies the method rather
   than supporting it;
3. there is **no hardware result and no safety certificate**. The filter's claim is empirical:
   on the simulated plant, with a stated model error, its own constraint held in 99.4 % of the
   control steps of the episodes we measured.

### 5.1 State and predictor

Let `x_t` comprise measured joint position `q_t`, velocity `v_t`, reference tracking error `e_t`,
recent command history `H_t`, and feedback age. Let `u_t` denote the command accepted by the
eventual hardware adapter. The adapter semantics must be fixed before selecting a model. For a
verified streamed-position interface, a useful starting predictor augments second-order joint
servo dynamics with the reference integrator and a measured delay buffer. If only queued MovJ is
supported, the predictor must instead represent the queued segment and the experiment must
operate at segment level.

We write the sampled closed loop as `x_(t+1) = F_hat(x_t, u_t; θ) + w_t`. `θ` is identified from
command/feedback logs using multiple open-space trajectories; `w_t` represents bounded prediction
error under specified conditions. Train, calibration, and test sessions are separated by trajectory
*and* recording session. A linear or low-order predictor is the first baseline. A residual network
is introduced only if it improves held-out multi-step prediction, especially worst-tail error near
braking.

### 5.2 Geometry and uncertainty budget

For link–obstacle pair `i`, let `d_hat_i(q)` be the nominal clearance after correcting geometric
discretization, and define the required margin as
`m_i = m_geom,i + m_frame,i + m_track,i + m_time,i`. The terms address geometric approximation,
scene registration, closed-loop tracking prediction, and sampling/latency respectively. **Avoid
double counting:** a delay already propagated through `F_hat` must not also be added as a second
independent delay penalty.

A first-order engineering approximation for the tracking allowance is `‖J_i‖ ε_q`, with a
separately bounded nonlinear remainder over the operating region. A velocity-dependent stopping
allowance may use `v_close τ + v_close²/(2 a_stop)` only when the delay and minimum deceleration
assumptions have been measured. This scalar relation is an intuition and initialization, not a
general six-joint safety proof. The principal implementation evaluates complete predicted and
stopping rollouts instead.

Calibration uses per-rollout maximum link-position prediction residuals, grouped by configuration
and operating regime. An empirical quantile is useful for tuning but does not produce a
deterministic bound. If a probabilistic coverage statement is used, the sampling unit, calibration
size, exchangeability assumption, and response to distribution shift must be stated. Missing or
stale observations are reasons to enter the validated fallback, not to silently reuse an old
certificate.

### 5.3 Bounded kinematic baseline

Before the proposed predictor, the strongest inexpensive correction to the current filter is
implemented and reported as a baseline: a QP minimizing weighted deviation from the nominal
command and command variation, subject to **all** active distance inequalities, joint limits, and
command bounds, with the bounds **inside** the optimization. Bounds must not be enforced by an
unverified post-solve clipping operation.

For static geometry and a velocity-level approximation the constraints have the form
`J_d,i(q) u ≥ −α_i [d_hat_i(q) − m_i]`. This remains a kinematic baseline. Adding more
inequalities fixes omissions and saturation handling, but does not by itself model servo lag.
Infeasibility, solver timeout, numerical residuals, and ill-conditioned distance gradients must be
logged. A relaxed safety constraint is an explicit constraint violation, not a successful safety
certificate.

### 5.4 Short-horizon prediction and stopping verification

The proposed filter optimizes a short sequence `U = (u_t, …, u_(t+H−1))` around the nominal
policy or reference path. Its objective penalizes deviation from the nominal sequence, changes
between commands, and loss of path progress. Constraints require predicted clearance above the
uncertainty allowance, joint and command limits, and a terminal state from which a specified
stopping controller remains feasible.

A sequential linearization around a warm-start rollout can produce a small QP, with a trust region
restricting the validity of the local model. Candidate settings of `H = 10–25` policy steps are a
starting engineering range, not a measured optimum. Every candidate is rechecked by nonlinear
rollout, including physics substeps and a complete stopping tail. If linearization, feasibility, or
timing checks fail, the controller uses only a previously verified fallback whose validity has not
expired. **A zero velocity command is not assumed to stop the current physical motion.**

The initial implementation targets static, registered obstacles. The stopping rollout must include
queued commands that cannot be canceled immediately; otherwise a nominally conservative safety
layer could be invalidated by commands already accepted downstream.

### 5.5 Conditional geometric argument

Suppose that, for every time within a candidate and its stopping tail, the true link surface lies
within an `ε_i` tube of the predicted link surface, the obstacle geometry is known within its
included frame error, and the checked predicted surface has clearance strictly larger than the
complete tube radius. Then the true surfaces do not intersect: distance to a fixed obstacle is
1-Lipschitz under a bounded displacement of the robot surface. This is a standard geometric
implication, not a novel theorem.

Applying it requires bounds on every relevant link, continuous-time or substep coverage, valid
prediction under the selected commands, and a feasible terminal backup. The current data do not
establish these assumptions. Consequently, the proposed implementation is described as an
empirically calibrated safety filter until these conditions are supported. Neither an empirical
maximum nor zero collisions in a finite test establishes unconditional invariance.

### 5.6 Learning component and planner guidance

Use RRTConnect or a validated optimization planner to choose a collision-free route. A simple
waypoint follower is the primary nominal controller. PPO provides a second nominal controller, and
an optional residual SAC learns a bounded correction to waypoint following. This separation asks
whether the filter works across proposal sources and whether learning adds measurable benefit
beyond planning and tracking.

A compact observation can include `q`, `v`, reference error, several path look-ahead points, and
per-link nearest-obstacle distances and normals, with a validity mask for absent obstacles. This
is more directly reusable across boxes, windows, and shelves than six wall parameters, but it is
still a local representation and cannot resolve global homotopy by itself. The planner supplies
that global information.

If residual RL is used, it is trained through the same filter used during evaluation. The replay
buffer stores the proposed action, the executed action, the intervention flag, and the resulting
transition. The critic's action semantics must match the defined environment: if its action is the
proposed command, the filter is part of the transition kernel. No residual, behaviour-cloned
residual, and RL residual are compared under equal data and execution budgets before RL is
retained in the final paper title.

## 6 Experimental protocol

### 6.1 Questions and hypotheses

**H1** tests whether integrating action bounds and multiple constraints removes projection-induced
violations under a matched detector. **H2** tests whether execution prediction improves the
collision–completion Pareto frontier relative to the best fixed-margin bounded QP. **H3** tests
whether planner guidance improves narrow-passage completion under equal time budgets. **H4** tests
whether residual learning adds value after the planner and filter are already present. **H5**
tests whether predictor calibration transfers across held-out hardware sessions within the declared
operating envelope. These are hypotheses, not reported findings.

### 6.2 Conditions and baselines

A versioned episode manifest fixes the geometry, start state, target pose, payload, delay trace,
random seed, and feasibility labels, and separates training, tuning, calibration, and final test
sets. Because the existing results were already inspected, seeds 1000–1049 are **historical
evaluation data** and cannot be reused as a fresh confirmatory test; the confirmatory range is
20000–20199.

| Group | Controller | Purpose |
|---|---|---|
| B0 | APF and DLS | legacy sanity checks |
| B1 | PPO without filter | the policy-failure reference |
| B2 | PPO + the archived single projection | the historical filter baseline (its defect included) |
| B3 | PPO + bounded all-pair kinematic QP | isolates the basic filter corrections |
| B4 | B3 with a tuned fixed clearance margin | the strong conservatism baseline |
| B5 | proposed execution-aware filter | isolates prediction and stopping verification |
| B6 | planner + waypoint follower + B4/B5 | separates global route selection |
| B7 | B6 with bounded residual learning | tests the value of RL |
| B8 | cuRobo or a robust predictive controller, reproducibly adapted | an external strong comparison |

Every reported number carries the manifest version and hash, the episode seed range, the training
seed, the filter settings, the plant preset, the collision-evaluator variant, the git commit, and
the SHA-256 of the checkpoint, so that any number can be reproduced from its own file.

Scene families are familiar random walls, extrapolated walls, windows, shelves, and mixed
primitives. Dynamics tests vary servo response, delay, payload, friction, and observation age one
factor at a time, followed by a small combined-stress grid. All methods receive the same nominal
policy, initial states, detector, planning budget, execution deadline, and information access.
Privileged-goal results are reported separately unless all methods receive the same goal
representation.

### 6.3 Metrics and statistical analysis

Primary metrics are collision-free completion and any collision per attempted episode, computed
with **two** detectors: the conservative envelope detector (the historical metric) and the mesh
geometry detector, so that the physical event is unambiguous. Secondary metrics include minimum
independent mesh clearance, minimum self-clearance, terminal dwell success, time to completion,
path length, command variation, intervention fraction, solver infeasibility, stop requests, and
p50/p95/p99 controller latency. Planning failure, tracking failure, deadline failure, and
infeasible target are reported separately. All attempts, including setup failures and resets, are
included under predefined exclusion rules.

For trained policies, between-training-seed variation is summarized and paired, hierarchical
resampling across training seeds and shared scenario identities is performed. For one fixed policy
evaluated on independent physical trials, binomial intervals can be informative; repeated shared
simulator scenarios are not equivalent to independent trials. Zero collisions in 50 independent
trials still permits an approximately 5.8 % one-sided 95 % upper bound, and reusing the same 50
scenarios across three training seeds does not turn that into a 150-scenario guarantee.

### 6.4 Hardware sequence

First characterize the actual command interface and stopping response in an unobstructed, bounded
workspace, with the existing motion whitelist and dry-run behaviour unchanged. Next execute
prevalidated trajectories with large clearance while recording commands, targets, feedback,
acknowledgements, queue state, and stops. Then validate predictor coverage on separate sessions.
Only after those stages should low-speed closed-loop tests with compliant, measured fixtures begin.

An initial comparative hardware design is four methods by three static scene types by twenty
attempts, or 240 attempted trials, sized against pilot variability and available reset time.
There is one robot unit, so conclusions concern that unit and the tested conditions. **No unsafe
raw-policy baseline is executed on hardware** to obtain a collision count: it is evaluated in
simulation and the hardware omission is reported. A physical intervention or a protective stop
counts as a safety intervention and an unsuccessful autonomous completion.

## 7 Results

### 7.1 Policy and filter results under the original detector

Scene distribution, filter, success and collision counts out of 150 policy–episode evaluations
(three training seeds × the same 50 scenario seeds):

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

*Table 1.* Three training seeds, each evaluated on the same 50 scenario seeds. Counts are
policy–episode evaluations, not 150 independent scenes. **The collision column counts the
conservative safety envelope** (Section 4.5): under the mesh-geometry detector these six rows
contain **no** real contact. The naive additive repulsion reduces envelope collisions but harms
completion; the projection reduces them further and improves completion by 10 percentage points
(+48 % relative, not a doubling). The filtered policy still completes fewer out-of-distribution
episodes than the potential field (30.7 % vs 50 %), so the filtered policy does **not** dominate
the classical baseline out of distribution.

The corrected evaluation (a rigorous lower-bound clearance plus per-substep collision checking)
changes the verdict in at most five of fifty episodes, and in sixteen of eighteen cells not at
all; the table therefore stands as the *legacy* reference.

### 7.2 The bounded all-pair QP as a strong baseline

Sweeping the fixed margin of the bounded all-pair QP on the **tuning** seeds traces the
safety–completion frontier (Figure 5). On the out-of-distribution walls the bounded QP has
**0 % envelope collisions at every margin ≥ 2 cm**, whereas the single-constraint projection still
collides at 3.3 % at a 2 cm margin and becomes *more* dangerous as it becomes more conservative
(3.3 % at 10 cm): a single constraint can only push the arm away from the closest obstacle and may
push it into another one it cannot see. In distribution the QP is insensitive to the margin up to
6 cm (72–73 % completion) and degrades at 15 cm (43 %). The margin is chosen on the tuning seeds
and the frozen test range is used afterwards.

### 7.3 Narrow passage: a matched comparison

The archived window comparison was not controlled — the planner received the goal *joint*
configuration and 30 s of execution against the reactive policies' 6 s. Under a matched protocol
(same environment, collision model, start/goal pools, success criterion, execution deadline and
**goal information**, with the planner realising the Cartesian goal itself):

| Method (matched, 50 episodes, 30 s deadline) | Success | Collisions |
|---|---|---|
| DLS straight line | 8 % | 70 % |
| APF | 0 % | 6 % |
| RRT-Connect (goal realised by the planner) | **38 %** | 6 % |

The conclusion survives the matching: on the narrow passage, planning reaches the goal in 38 % of
the episodes and every reactive method in 0–8 %, even with five times more time.

### 7.4 The command channel of the simulated plant

The plant does not execute a command; it executes a *channel*. The command is integrated into a
position reference at the physics rate, the reference is clipped at the joint limits, and a
bandwidth-limited servo tracks it (`omega_n = 2*pi*8 rad/s`, `zeta = 0.9`, with gravity and
Coriolis feed-forward). Two consequences are measurable on the simulator:

| commanded velocity | coast after the command is cut | coast per rad/s of command |
|---|---|---|
| 0.25 rad/s | 8.2 mrad | 0.033 rad |
| 0.50 rad/s | 10.4 mrad | 0.021 rad |
| 0.75 rad/s | 12.7 mrad | 0.017 rad |
| 1.00 rad/s | **14.9 mrad** | 0.015 rad |

At the maximum command the arm travels a further 14.9 mrad after the command is cut — 8.9 mm of
extra travel for a link 0.6 m from the base. A second-order model of the channel, identified from
the simulator (`omega = 37.7 rad/s`, `zeta = 0.90`, `v_gain = 0.80`), predicts the joint position
over five policy steps with an RMS error of **4.6 mrad**. That error is flat within ±20 % of the
identified `omega`, so a filter built on the model does not require an exact plant
identification — but 4.6 mrad is a third of the coast it is meant to predict, which is the first
hint that the model, not the coast, is where the error budget goes.

### 7.5 The execution gap: the kinematic filter's own claim fails in execution

This is the measurement the study was built around. At every control step of a real episode we
ask two questions: was the filter's *own* reasoning satisfied, and did the plant then end the
step below the clearance margin? Out-of-distribution walls, 20 episodes, 6000 control steps,
2 cm margin:

| filter | the filter's own constraint violations | steps where the filter was satisfied but the margin was breached | rate | gap: actual − command-level prediction (median / worst) |
|---|---|---|---|---|
| bounded kinematic QP | 0 | **553** | **9.2 %** | −12.0 mm / −146.5 mm |
| execution-aware, no coast (ablation) | 1 | **10** | **0.2 %** | 0.0 mm / −135.6 mm |
| execution-aware with coast | 1 | **37** | **0.6 %** | 0.0 mm / −135.6 mm |

The kinematic filter issues a command that satisfies every one of its constraints, and the arm
still ends the step 12 mm below where the command said it would be — 9 % of the time it ends the
step below the margin the filter was enforcing. The gap is not noise: its median is −12 mm and
its worst case is −146 mm. **The filter's guarantee is a statement about the command, and the
command is not the motion.**

Sweeping the margin shows the failure is not a property of one operating point:

| margin | bounded kinematic QP | execution-aware, no coast | execution-aware with coast |
|---|---|---|---|
| 0.5 cm | 3.6 % | 0.1 % | 0.6 % |
| 1 cm | 8.0 % | 0.1 % | 0.4 % |
| 2 cm | 9.2 % | 0.2 % | 0.6 % |
| 4 cm | **32.4 %** | 1.3 % | 4.8 % |

The kinematic filter's own claim fails more often the *more conservative* it is set: a larger
barrier margin starts the rate condition earlier, further from the motion that actually
happens. The look-ahead formulation is nearly flat.

And the *honest* attribution is that this is the look-ahead structure, not the plant model:
dropping the coast term from the constraint changes the frontier not at all and makes the
filter's own claim consistently *more* valid at every margin (0.1 % against 0.6 %, 0.1 % against
0.4 %, 0.2 % against 0.6 %, 1.3 % against 4.8 %). The coast is real and measured (§7.4), but it
is not what carries the result. What carries the result is replacing a rate condition on the
command with a position condition on the predicted motion. That is a smaller claim than the one
the project started with, and it points the remaining effort at the predictor's error budget
rather than at a more elaborate plant model. The most likely reason the coast term *hurts* is
that a tighter constraint makes the filter intervene more often, and every intervention moves
the command away from the "held constant" assumption the prediction rests on — an interaction
that a longer, better-identified model would have to be checked against.

### 7.6 The frontier with the execution-aware filter

On the out-of-distribution walls, tuning seeds 0–19, 3 policy seeds, 20 episodes per cell
(Figure 5):

| method | margin | completion | envelope collisions | max relaxed constraint | infeasible solves |
|---|---|---|---|---|---|
| unfiltered policy | – | 15 % | **42 %** | – | – |
| bounded all-pair QP | 0 cm | **35 %** | 0 % | – | 0 |
| bounded all-pair QP | 2 cm | 28 % | 0 % | – | 0 |
| bounded all-pair QP | 6 cm | 18 % | 0 % | – | 0 |
| bounded all-pair QP | 10 cm | 5 % | 0 % | – | 42 |
| execution-aware | 0.5 cm | 30 % | 0 % | 0.00 mm | 0 |
| execution-aware | 1 cm | 30 % | 0 % | 0.00 mm | 0 |
| execution-aware | 2 cm | 28 % | 0 % | 3.25 mm | 3 |
| execution-aware | 4 cm | 25 % | 0 % | 23.25 mm | 24 |
| execution-aware | 6 cm | 13 % | 0 % | 43.25 mm | 64 |
| execution-aware, no coast (ablation) | 0.5–6 cm | identical | identical | identical | identical |

Two honest readings. First, the execution-aware filter reaches 0 % envelope collisions at every
margin including 5 mm, and below a 2 cm margin it never needed a relaxed constraint — the
relaxation column shows the violations it did have at larger margins, which are reported, not
hidden. Second, and against our own expectation, **it does not improve the safety–completion
trade-off**: the bounded QP at margin 0 reaches 35 % completion with 0 % collisions, and the
execution-aware filter's best is 30 %. On this metric the envelope is already far from the arm
(§4.5), so both filters are operating in a regime where the collision axis carries little
information. The demonstrated advantage of the execution-aware filter is §7.5, not the frontier.

### 7.7 The confirmatory benchmark on the frozen test range

The seeds 1000–1049 that produced the published table have been inspected, so they are now
historical **dev** data and the confirmatory range is **20000–20199**. Every episode of the
confirmatory run is judged by three detectors — the legacy envelope detector, the corrected
detector, and the independent mesh checker — and the policy behaviour is the archived one, so
only the detector differs. The full table is `data/reports/learning/final_benchmark_full.json`;
the per-cell numbers are in `docs/RESULTS.md` §8.

### 7.8 Supporting evidence and scope

| Evidence item | Archived observation | Interpretation |
|---|---|---|
| Controller-model calibration | held-out session RMS 0.002–0.011 mm | controller consistency, not external accuracy |
| Dynamics scale | ≈ 1.636 | predictive fit under assumed torque units |
| MuJoCo torque comparison | J2/J3 RMS about 24.0/24.9 to 3.9/5.6 signal units | local replay agreement, not servo identification |
| Identified plant at 8 Hz and nominal speed | tracking maximum median about 8.9 to 13.3 mm | the placeholder simulation was optimistic in that condition |
| Proposed filter on hardware | PENDING | no closed-loop learned-policy hardware result |

*Table 2.* Supporting evidence and scope. The dynamics comparison uses the recorded run02 regime;
the physical interpretation of the torque signal remains conditional.

## 8 Discussion

The strongest current observation is not that RL solves manipulation. It is that a policy can
benefit from a geometric correction, that the correction still fails to justify physical safety,
and that **the project's own collision metric was measuring a conservative envelope rather than
the robot** — a distinction that changes what the headline number means without changing the
number itself.

The repository offers unusually useful ingredients for studying this boundary: real feedback,
controller-consistent geometry, a local dynamics record, and policy failures under a controlled
shift in wall size.

There are three possible outcomes of the proposed study. If the execution-aware filter outperforms
the best fixed-margin bounded QP at comparable completion and latency, the measured execution model
is useful. If a fixed margin performs equally well, the simpler method should be preferred and the
predictor claim rejected. If planner following with either filter matches residual RL, the paper
should focus on filtering and execution rather than force an RL contribution. Negative results
remain informative when they identify which complexity is unnecessary.

Hardware constraints are central. An asynchronous training job can generate data or optimize a
small policy, but network-connected inference should not sit in a safety-critical control path.
The local machine should execute the controller and monitor its timing. A larger GPU does not
resolve an unverified command interface, missing geometry, or an inadequate stopping model.

## 9 Limitations and conclusion

The existing learning results use known geometry, a simplified servo, no physical contact response,
a detector whose geometric looseness is now measured, and position-only success. The proposed
filter is not yet implemented, its uncertainty tube is not validated, and learned policies have not
been deployed on the CR10A. One hardware unit cannot establish cross-robot generalization. Torque
signal units, independent physical accuracy, and command cancellation behavior require direct
verification. The archived out-of-distribution collision rates measure a safety envelope, not
contact with the arm, and the narrow-passage comparison required a matched protocol to be
meaningful at all.

The archived evidence establishes a concrete motivation for execution-aware filtering, and the
source audit identifies why the current projection cannot be treated as a safety guarantee. The
proposed study connects corrected geometric evaluation, bounded optimization, measured command
execution, and explicit stopping verification. Its final scientific contribution will depend on
matched experiments demonstrating an advantage over strong fixed-margin and planning baselines.
Until those measurements exist, this manuscript supports a reproducible research programme and a
preliminary empirical analysis, not a completed deployment claim.

---

## Appendix A Implementation algorithm

1. Acquire a fresh, timestamped state and the actual downstream command state. Reject invalid
   state estimates using the existing validation pipeline plus command-channel checks.
2. Generate a nominal command from a frozen policy or a waypoint follower. Retain both the
   proposal and its source identifier.
3. Form all relevant link–obstacle and self-collision constraints with corrected geometry; include
   joint and command bounds **inside** the optimization.
4. Predict the augmented closed loop and compute the calibrated uncertainty allowances for the
   declared regime.
5. Solve the bounded short-horizon problem with a trust region and explicit deadline. Record
   feasibility and residuals.
6. Recheck the candidate using nonlinear prediction, substeps, independent geometry where
   affordable, and the stopping tail.
7. Send only the first accepted command through the verified adapter. Otherwise execute a
   still-valid certified backup or invoke the prevalidated stop procedure.
8. Record proposals, commands, measurements, constraints, prediction errors, interventions, and
   timing. **Do not label a stopped or infeasible episode as successful completion.**

## Appendix B Reproducibility

The repository carries the frozen episode manifest, the corrected and legacy evaluators, the
independent mesh-geometry checker, the regression tests for every audited defect, and the result
manifests that let each number be reproduced from its own file. The figures are produced by
`evaluation/plot_*.py` from the committed JSON reports; the audited pipeline reproduces an
archived result exactly, which is the check that the code under audit is the code that produced
the published numbers.

## References

[1] M. Dalal, J. Yang, R. Mendonca, Y. Khaky, K. Salakhutdinov, and D. Pathak. Neural MP: A
Generalist Neural Motion Planner. IROS 2025; preprint arXiv:2409.05864, 2024.

[2] J. Yang, J. J. Liu, Y. Li, Y. Khaky, K. Shaw, and D. Pathak. Deep Reactive Policy: Learning
Reactive Manipulator Motion Planning for Dynamic Environments. CoRL 2025, PMLR 305, pp. 5056–5071.

[3] D. Morton and M. Pavone. Safe, Task-Consistent Manipulation with Operational Space Control
Barrier Functions. IROS 2025, pp. 187–194. doi:10.1109/IROS60139.2025.11246389.

[4] H. Wang, J. Peng, F. Zhang, and Y. Wang. High-Order Control Barrier Function-Based Safety
Control of Constrained Robotic Systems: An Augmented Dynamics Approach. IEEE/CAA Journal of
Automatica Sinica, 11(12), pp. 2487–2496, December 2024. doi:10.1109/JAS.2024.124524.

[5] B. Wullt, J. Kohler, P. Mattsson, M. Norrlof, and T. B. Schon. Robust Convex Model Predictive
Control With Collision Avoidance Guarantees for Robot Manipulators. IEEE Transactions on Control
Systems Technology, 34(5), pp. 2909–2916, 2026. doi:10.1109/TCST.2026.3707594.

[6] B. Dai, R. Khorrambakht, P. Krishnamurthy, and F. Khorrami. Differentiable Optimization Based
Time-Varying Control Barrier Functions for Dynamic Obstacle Avoidance. Robotics and Autonomous
Systems, 2025, article 105182. doi:10.1016/j.robot.2025.105182.

[7] Z. Zhou, A. Garg, D. Fox, C. R. Garrett, and A. Mandlekar. SPIRE: Synergistic Planning,
Imitation, and Reinforcement Learning for Long-Horizon Manipulation. CoRL 2024, published in PMLR
270, pp. 2347–2371, 2025.

[8] P. K. Rath and N. Gopalan. XMoP: Whole-Body Control Policy for Zero-Shot Cross-Embodiment
Neural Motion Planning. ICRA 2025, pp. 8299–8306.

[9] B. Sundaralingam and collaborators. cuRobo: Parallelized Collision-Free Robot Motion
Generation. ICRA 2023, pp. 8112–8119. doi:10.1109/ICRA48891.2023.10160765.

[10] terrense. experiment_lab_embodied_intelligence_CR10A. Repository snapshot
`c5c8858c4a9261daa5caae475c844ed7182910bd`. <https://github.com/terrense/experiment_lab_embodied_intelligence_CR10A/tree/c5c8858c4a9261daa5caae475c844ed7182910bd>
