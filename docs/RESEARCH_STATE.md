# Research state: a self-contained brief for external analysis

> **Purpose of this file.** This is a complete, stand-alone snapshot of the EMBODIED-X
> project, written so that an external reader (a paper-literate LLM, a colleague, a
> reviewer) can understand exactly where we are, what we have measured, what failed, and
> what we are stuck on — **without reading the rest of the repository**. The last section
> is the ask: numbered open questions where we want suggestions grounded in published
> papers and frontier work. (中文说明见文件末尾。)

---

## 1. What this project is

A research-engineering lab around a **real Dobot CR10A** 6-axis collaborative robot
(6 DoF, ~40 kg datasheet mass). The end goal is **sim-to-real embodied intelligence**: a
learned policy that reaches through clutter without colliding, developed entirely in
simulation, validated against a physical twin, and eventually run on the real arm — with
an honest account of what does and does not transfer.

The pipeline is deliberately staged, and each stage must ship *runnable code, tests,
measured results, and an honest failure analysis* before the next begins:

```
M1   read-only digital twin of the real CR10A (kinematic model verified on site)
M1.5 kinematic calibration from on-site data (recover the controller's own model)
M2   MoveIt 2 planning lab (simulation) — OMPL planner benchmark
M4   MuJoCo physics plant behind ros2_control (simulation) — servo tracking
M4.5 dynamics identification on the real, moving arm
M5   learning (simulation): reach / reach-around-the-wall / narrow passage + safety filter
M3   (planned, not started) visual perception: T265 VIO + hand–eye calibration
NBV  (planned, not started) active perception / next-best-view
```

Everything in **M5 is simulation only**. Nothing learned has ever run on hardware. Real and
simulated data are always labelled; results are never fabricated, and anything not run is
marked UNVERIFIED.

---

## 2. Hardware, compute, and access constraints

| Item | Detail |
|---|---|
| Robot | Dobot CR10A 6-axis cobot. On-site controller at `192.168.201.1` (via a USB NIC to the site LAN). |
| Real-robot access | **Only with on-site authorization + the owner at the e-stop.** M1 is read-only (feedback ports 30004/30005 only; command ports 29999/30003 never opened). Motion code lives only in `ex_dobot_motion` and is dry-run by default. |
| Peripheral vision HW | Jetson TX2 + RealSense T265 (note: librealsense dropped T265 support around v2.54 — must be verified before M3). |
| Dev machine | Windows 11 + WSL2 (`EmbodiedX-Humble`, Ubuntu 22.04, ROS 2 Humble). GPU **RTX 5060 Laptop 8 GB** (not 12 GB). |
| RL compute | **CPU only** (torch 2.5.1 CPU, gymnasium 1.0, Stable-Baselines3 2.4.1), 12 subprocess envs, ~550–1100 env steps/s. 3 M steps ≈ 22–38 min; 5 M ≈ ~37 min. |
| Physics | MuJoCo 3.3.7, 1 kHz physics, 50 Hz policy, same MJCF builder as the ros2_control plant. |

The CPU-only constraint matters: sample-efficient methods and off-policy / replay-buffer
algorithms are far more attractive than wall-clock-heavy on-policy training.

---

## 3. What is done (with measured numbers)

### M1 — read-only digital twin (REAL robot)
- Live joint sync over feedback port 30004 (the observer **sends 0 bytes** — read-only).
- The official `cr10` URDF matches the controller TCP to **1.6–3.0 mm RMS, max 5.54 mm**
  (41 357 samples across all six joints). The alternative `cr10af` model is off by ~43 mm.
- Convention resolved on site: Euler = extrinsic XYZ, quaternion = wxyz, Link6 = flange,
  base_link = user frame 0.
- Failure analysis documented: unit/degree detection blind spot, cross-topic pairing error,
  transient-fault masking, checker false positives, TCP half-close disconnects, 14 % packet
  loss fixed by per-packet publish.

### M1.5 — kinematic calibration (REAL data)
- 45-parameter regularised model, identifiable rank 25. Cross-session held-out error drops
  from **3.1–3.4 mm to 0.002–0.011 mm**, plus a **0.207° flange-tilt** correction.
- Ablation: zero-offset alone explains ~30 % of the error; the rest is link geometry.
- Interpretation (honest): this *reproduces the controller's own calibrated model*, not an
  external-metrology absolute accuracy.

### M2 — MoveIt 2 planning lab (simulation)
- 400 plans (3 scenes × 8 queries × 10 OMPL planners × 5). **RRTConnect and SBL get 39/40**;
  single-tree RRT/RRT* mostly time out on point goals.
- Narrow passage (window): **PRM 0/15**. **BiTRRT 0/40 — still unexplained.**
- Path clearance median is only 1–2 cm, a few× the 5.5 mm model error → real-robot execution
  will need padding.

### M4 — MuJoCo physics plant (simulation)
- The plant sits *behind* ros2_control so MoveIt and everything above it run unchanged.
- Tracking error is roughly **∝ 1/servo bandwidth**: 8 Hz → 7.6–15 mm, 16 Hz → 3.4–6.6 mm,
  4 Hz cannot track. A wrong `SetPayload` (10 kg true / 0 kg configured) sags 42 mm.

### M4.5 — dynamics identification (REAL, moving arm)
- Linear-in-parameters inverse-dynamics regressor, validated on MuJoCo synthetic data with
  known truth first. Method verified: **armature, Coulomb friction, and the mass scale are
  recovered; viscous friction is NOT identifiable at these speeds.**
- On the real run: **URDF masses are 1.64× too light** (≈ 40 kg datasheet), Coulomb friction
  **10–14 N·m** on J1–J3, held-out torque VAF 96–99 %. The sim-to-real torque gap on J2/J3
  drops from 24–25 N·m to 4–6 N·m with the identified preset.

### M5 — learning (SIMULATION ONLY)
Full detail in the next section. Headline numbers (50 evaluation episodes, seeds 1000–1049):

| Setting | DLS straight | APF potential field | PPO | + CBF safety filter |
|---|---|---|---|---|
| reach (free space) | **100 %** | — | — | — |
| wall (fixed) | 4 % / 96 % coll | 44 % / 0 % | **67 % / 0 %** (58/64/80) | 67 % / 0 % |
| window (narrow passage) | 6 % / 76 % | 0 % | 0 % | — |
| wall, moved (one-wall PPO) | — | — | 21 % / 7 % | — |
| wall, random+observed (generalist PPO) | 24 % / 76 % | 64 % / 0 % | **77 % / 4 %** | **81 % / 0 %** |
| wall, out-of-distribution (taller+wider) | 12 % / 88 % | **50 % / 2 %** | 21 % / 36 % | **31 % / 5 %** |
| window (RRT-Connect + follow) | — | — | — | **38 % / 2 %** |

Reading: free-space reaching is solved by a 5-line classical controller (so it is *not* a
reason to use RL). RL beats APF on the fixed/random wall; APF beats RL out of distribution;
the CBF safety filter removes most of the RL policy's OOD collisions (36 % → 5 %) and raises its
OOD success from 21 % to 31 % (+10 points, +48 % relative — **not** a doubling, and still below
APF's 50 %). All of these numbers come from the *legacy* collision evaluator; the audited
defects and the corrected protocol are in `docs/RESEARCH_PLAN.md`.

---

## 4. The learning setup in detail (where most of the open questions live)

### Environment
- Reuses the M4 MuJoCo plant (same MJCF builder + servo model). 1 kHz physics, 50 Hz policy.
- **Action**: joint-velocity command in `[-1, 1]^6` (× 1 rad/s), integrated into the servo
  set-point (the plant has its own bandwidth, so the action is a *command*, not a torque).
- **Observation** (base 21-dim): `[q(6), qd(6), tcp(3), goal(3), goal − tcp(3)]`.
  - wall env adds clearance → 22-dim.
  - `wall_obs`/`wall_rand`/`wall_ood` append the wall box (centre 3 + half-size 3) → 28-dim.
- **Success**: TCP within **2 cm** of the goal. Episode length 300 steps (6 s).
- **Collision is detected, not simulated**: signed distance between M2 enclosing link
  capsules and the scene boxes (conservative — the capsules contain the meshes). The arm
  passes through, but a collision is flagged and (for RL) the episode ends with a penalty.
  The window scene uses mesh point clouds (5 mm samples, 1 cm voxel, 1.73 cm margin) because
  the one-capsule-per-link model cannot thread a narrow passage.

### Training / evaluation protocol
- Baselines first, always, on the same evaluation set: DLS (damped-least-squares Jacobian),
  APF (potential field over every link capsule, gains tuned on seeds 0–19).
- PPO (SB3) on CPU, 12 parallel envs, 3 M–5 M steps, seeds 0/1/2.
- Reward v1 was `−distance` per step → PPO learned to **crash on purpose** (end the episode
  early with a −10 collision instead of living with −0.5/step). Fixed with a potential-based
  progress reward + a regression test that crashing early never pays.
- Evaluation: **50 episodes, seeds 1000–1049, deterministic policy**; tuning uses seeds 0–19.
  Reported metrics: success rate, collision rate, median final distance.

### Key learning results
1. **Fixed wall**: all 3 PPO seeds beat APF (58/64/80 % vs 44 %), 0 collisions. Success
   spreads because most "misses" stop 20–40 mm from the goal, right at the 2 cm threshold.
2. **Generalisation**: the one-wall PPO **memorised a path** — 67 % → 21 % when the wall
   moves. Training on randomised, *observed* walls reaches 77 % in distribution (beats APF
   64 %), but on **out-of-distribution** walls (taller + wider) it collides ~36 % of the time
   while APF collides 2 %.
3. **Safety filter** (the newest result): can we bolt "don't collide" onto a trained policy?
   - *Naive additive repulsion* (always-on APF term added to the action): **fails** — halves
     success everywhere (71 → 43 %) because it fights a policy trained without repulsion.
   - *Control-barrier (CBF) projection* (closest-point, single constraint, closed-form
     projection; only brakes when `clearance ≥ margin` is about to be violated): OOD collisions
     **36 % → 5 %** and OOD success **21 % → 31 %**, with **no in-distribution cost**
     (77 → 81 %, 4 → 0 %). This is the best current OOD result.
   - The CBF is a *practical* layer, not a proof of safety: it is a single-constraint
     closed-form projection (with a saturation caveat when clipped to the command bound), not
     a full QP over all link pairs + joint limits.

### Things we tried that did **not** work (so the reader does not re-suggest them)
- DLS straight line on any wall scene → collides (4 % / 96 % on the fixed wall).
- APF on the window (narrow passage) → 0 % (stuck in front of the wall; 9 gain settings tried).
- PPO on the window → 0 % (gets closer than APF, never through).
- Joint-space straight line (privileged) on the window → 0 % / 100 % collisions.
- Naive additive repulsion as a safety filter → halves success.
- One-wall PPO generalised by moving the wall only → memorises, fails.
- 6-number box observation → cannot represent non-box obstacles (window, shelf, a person).
- MuJoCo mesh–mesh GJK distance for collision → returned swapped endpoints and spurious zeros.
- Reward `−distance` → PPO crashes early.

---

## 5. Open research questions (the ask)

We want suggestions grounded in published papers and recent/frontier work. For each we give
our current hypothesis.

**A. Learning through clutter / obstacle-avoidance policy**
1. The policy represents obstacles as *6 numbers* (a box) and fails OOD. What is the current
   frontier on **geometric obstacle representations for manipulation policies** — point clouds,
   signed-distance/occupancy fields, learned scene encodings — that stay robust when the
   obstacle shape/size changes? What actually scales to a 6-DoF arm with CPU-only training?
2. The success criterion is a hard 2 cm threshold, and most failures stop 20–40 mm away. Is
   there a better **reward shaping / success metric** for "get close but don't touch" that
   doesn't teach the policy to stall short? (We already know potential-based shaping fixed
   the crashing problem; the residual problem is *under-shooting*, not crashing.)
3. PPO curves are still rising at 5 M steps. With ~10³ steps/s CPU-only, what **sample-efficient**
   algorithms or improvements should we try first (SAC/TD3, HRL, action/state history, PPO
   hyperparameters, Hindsight Experience Replay for "goal-directed with clutter")?
4. The narrow passage (window) is 0 % for every reactive method and 38 % for RRT-Connect — but
   the planner gets the privileged goal joint configuration and a 30 s budget against the reactive
   policies' 6 s, so this is not yet a controlled comparison.
   What is the best current way to **combine planning and learning** — planner-generated
   demonstrations for BC/RL, RRT-informed RL, curriculum over passage width, or planner-guided
   residual policies?

**B. Safe RL / safety filters**
5. Our CBF is a single-constraint closed-form projection on a *known* capsule model. What is
   the frontier on **control barrier functions + learned policies** for manipulators: full
   QP formulations (all pairs + joint limits), CBFs over learned dynamics, and — critically —
   how to make the safety argument hold when the collision model itself is approximate?
6. Are there established **safe-RL benchmarks/formalisms** (Safety Gym, shielded RL, safe
   exploration) we should adopt so our numbers are comparable, rather than hand-rolled?

**C. Sim-to-real**
7. We can identify the dynamics (masses 1.64×, Coulomb friction), but viscous friction was
   unidentifiable at our speeds. What **excitation design / identifiability** guidance exists
   for Coulomb+viscous on a cobot, and what **domain-randomisation** scheme over the
   identified parameters is known to transfer a learned policy across the sim-to-real gap?
8. Nothing learned has run on hardware. What is the accepted **safety case for a learned
   policy on a physical arm** (a real-time CBF on the real robot, conservative kinodynamic
   guard, velocity scaling)? We want a concrete checklist, not a vibe.

**D. Planning gaps**
9. **BiTRRT 0/40** on our scenes is unexplained. Is this a known BiTRRT limitation/sensitivity,
   or a MoveIt-config artifact we should chase down?
10. Narrow-passage planning is the clear strength of sampling-based planners here. Is there a
    better **narrow-passage planner / sampling bias** (experience-driven, learned samplers,
    LazyPRM, etc.) worth benchmarking next to RRTConnect?

**E. Perception (not started — scope before we begin)**
11. T265 support was dropped from librealsense ~v2.54. What is the current best way to get
    VIO on a Jetson TX2 (T265 with pinned old librealsense, Realsense D435i + internal VIO,
    or another sensor), and what is the standard **hand–eye + VIO-vs-FK trajectory alignment**
    (AX=XB + ATE/RPE + time-offset) pipeline we should follow?

**F. Prior art we should just adopt rather than reinvent**
12. Are there published **manipulation-learning codebases/benchmarks** (RLBench, ManiSkill,
    Robosuite, ORBIT/Isaac Lab) where our reach-around-the-wall / narrow-passage task already
    exists, so we can compare against the field instead of our own APF/DLS baselines?

---

## 6. Where things live (for reproducibility)

> **Read `docs/RESEARCH_PLAN.md` as well.** An external source review of the archived code
> (2026-11-02) found six defects in the evaluation path. They were independently re-derived and
> fixed (commit `5bf50aa`, 175 tests pass), the legacy numbers are kept as the *legacy* reference,
> and the plan states the frozen protocol, the method ladder and the falsification criteria that
> decide whether the learning component survives at all.

| What | Where |
|---|---|
| Learning envs, baselines (DLS/APF), PPO, safety filter | `learning/ex_learning/` (`cr10a_reach.py`, `cr10a_reach_wall.py`, `safety_filter.py`, `train_ppo.py`, `eval_ppo.py`, `run_safe_eval.py`) |
| Tests (160 unit tests incl. 10 learning + 4 safety-filter) | `learning/tests/` |
| Dynamics regressor + ID | `ex_robot_state/core/dynamics.py`, `evaluation/dynamics_id.py` |
| Motion-capable code (dry-run default) | `ros2_ws/src/ex_dobot_motion` |
| Per-stage docs | `docs/{HARDWARE_COMPATIBILITY,CALIBRATION,M2_PLANNING_LAB,M4_MUJOCO,DYNAMICS_ID,LEARNING}.md` |
| Progress log + roadmap | `docs/PROGRESS.md`, `docs/ROADMAP.md` |
| Failure analyses | each stage doc has a "Failure analysis" table |
| Figures | `assets/{m1_live,m2,m4,dynamics,learning}/` |

---

*中文说明：本文档是 EMBODIED-X 项目的完整自包含快照，供外部读者（论文导向的大模型、
同事、审稿人）在**不读仓库其余部分**的情况下理解我们走到了哪一步、测到了什么、什么失败了、
卡在哪里。最后第 5 节是诉求：逐条列出开放问题，希望结合论文与前沿工作给出具体建议。*
