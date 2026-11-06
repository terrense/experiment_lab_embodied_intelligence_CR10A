# EMBODIED-X: From Simulation to Physical Intelligence

A research-engineering lab around a real **Dobot CR10A** 6-axis cobot. Pipeline:
read-only digital twin → kinematic calibration → MoveIt planning → MuJoCo physics → dynamics identification on the real arm → learning.
Every stage ships working code, tests, measured results and an honest failure analysis. Simulated and real data are always labelled.

> 🇨🇳 想自己上手看画面：[hands_on/README.md](hands_on/README.md)，双击 `.bat` 就能打开 MuJoCo 和 RViz。

## At a glance
| Stage | Headline result | Details |
|---|---|---|
| **M1** Read-only digital twin (**real CR10A**) | Live joint sync over port 30004 (sends 0 bytes); the official `cr10` model matches the controller to 1.6–5.5 mm | [HARDWARE_COMPATIBILITY](docs/HARDWARE_COMPATIBILITY.md) |
| **M1.5** Kinematic calibration (**real data**) | Controller model recovered: held-out error **0.002–0.011 mm**, 0.207° flange tilt found | [CALIBRATION](docs/CALIBRATION.md) |
| **M2** MoveIt planning lab (sim) | 400-plan OMPL benchmark: RRTConnect/SBL 39/40, narrow passages separate the planners | [M2_PLANNING_LAB](docs/M2_PLANNING_LAB.md) |
| **M4** MuJoCo physics plant (sim) | Behind ros2_control, MoveIt unchanged; tracking error ∝ 1/servo bandwidth (3–15 mm) | [M4_MUJOCO](docs/M4_MUJOCO.md) |
| **M4.5** Dynamics identification (**real CR10A moving**) | URDF masses are 1.64× too light (≈ 40 kg datasheet); Coulomb friction 10–14 N m; sim-to-real torque gap cut **3–6×** | [DYNAMICS_ID](docs/DYNAMICS_ID.md) |
| **M5** Learning (sim) | Wall task: straight line 96 % collisions, potential field 44 % success, **PPO 67 % (3 seeds: 58/64/80 %), 0 collisions**. Moved walls: one-wall PPO 21 %, random-wall PPO 77 %, but 36 % collisions on out-of-distribution walls (APF 2 %). A **control-barrier safety filter** cuts those to 5 % collisions while success rises 21 % → 31 % (still below APF's 50 % success). ⚠ The collision metric is the *conservative safety envelope*, not the arm: an independent mesh-geometry checker finds **no real contact in any of the 300 evaluated episodes**. Window: RRT-Connect 38 % (matched protocol), APF/PPO 0 % |  [LEARNING](docs/LEARNING.md) · [AUDIT](docs/EVALUATOR_AUDIT.md) |
| **M5.5** What a safety filter actually guarantees (sim) | The plant executes a **channel**, not a command: the arm coasts **14.9 mrad** after a 1 rad/s command is cut. A kinematic filter's own constraints hold while the arm still ends the control step below the enforced margin in **9 %** of the steps (32 % at a 4 cm margin), median prediction error 12 mm — **the command is not the motion**. On the frozen test range the look-ahead filter reaches the completion of an unconstrained-margin filter (54 %) with **0 %** collisions where that filter collides in 16–22 % | [RESULTS](docs/RESULTS.md) · [PLAN](docs/RESEARCH_PLAN.md) · [PAPER](paper/manuscript.md) |

---

## M1: read-only digital twin of the real CR10A
![Real CR10A live sync](assets/m1_live/cr10a_live_rviz_sync.png)
*REAL robot, LIVE_OBSERVE (read-only, port 30004): the operator hand-guides the CR10A in drag mode and the RViz model follows. The yellow WARN is the 94 Hz publish-rate bug found on site (fixed afterwards: 123 Hz).*

| Simulation mode | Fault injection |
|---|---|
| ![SIMULATION mode](assets/screenshots/m1_sim_rviz.png) | ![Fault injection](assets/screenshots/m1_fault_degrees_rviz.png) |
| *Simulated data: CR10 model, TF frames, joint axes, TCP trail, HUD with joint angles, rate, freshness and TCP pose.* | *`fault:=degrees`: the adapter rejects the samples, the display freezes on the last valid state and the HUD turns red with the reason.* |

## M1.5: kinematic calibration from on-site data
![Calibration CDF](assets/m1_live/calibration_cdf.png)
*REAL data, cross-session (fit on one session, test on the other). Nominal official model 3.1–3.4 mm RMS → 45-parameter regularised model (rank 25) **0.002 / 0.011 mm**. This recovers the controller's own calibrated model; it is not a metrology result.*

## M2: MoveIt planning lab (simulation)
| Success rate | Planning time |
|---|---|
| ![success](assets/m2/success_rate.png) | ![time](assets/m2/planning_time.png) |
| **Clearance** | **Path length** |
| ![clearance](assets/m2/clearance.png) | ![path length](assets/m2/path_length.png) |

| Wall | Window (narrow passage) | Shelf |
|---|---|---|
| ![wall](assets/m2/tcp_paths_wall.png) | ![window](assets/m2/tcp_paths_window.png) | ![shelf](assets/m2/tcp_paths_shelf.png) |

*400 plans (3 scenes × 8 queries × 10 OMPL planners × 5). Bidirectional trees win (RRTConnect, SBL 39/40). In the window scene, PRM gets 0/15. Asymptotically optimal planners always use the full budget for small path gains. BiTRRT 0/40 is not explained yet.*

## M4: MuJoCo physics plant behind ros2_control (simulation)
![tracking](assets/m4/tracking_tcp_max.png)
*60 executions of one trajectory: mock plant 2.2 mm; servo 16 Hz 3.4–6.6 mm; 8 Hz 7.6–15 mm; 4 Hz cannot track (0/18).*

Other findings:
- A wrong `SetPayload` makes the PD servo sag (10 kg true / 0 kg configured gives 42 mm).
- With the dynamics identified on the real arm, tracking error at 8 Hz is **+50 %**, so the earlier predictions were optimistic.

## M4.5: dynamics identification on the real, moving CR10A
The arm moved under program control: owner on site at the e-stop, unit authorization, motion module `ex_dobot_motion` with whitelist, caps, validated plans and dry-run by default.

![torque fit](assets/dynamics/run02_torque_fit.png)
*REAL data, 85 s excitation, 0.4 kg camera on the flange. Black = measured joint torque. The URDF alone (grey) explains almost nothing: friction dominates and the masses are too light. Fitted models reach a held-out VAF of 96–99 %. The method was first validated on MuJoCo data with known truth: armature, Coulomb friction and mass scale are recovered; viscous friction is not identifiable at these speeds, and the docs say so.*

![sim2real](assets/dynamics/sim2real_torque.png)
*Same real trajectory replayed in MuJoCo: default plant vs the identified preset (`dynamics:=identified_run02`). The J2/J3 torque gap drops from 24–25 N m to 4–6 N m.*

## M5: learning (simulation)
![MuJoCo viewer](assets/hands_on/mujoco_viewer_apf_wall.png)
*Reach-around-the-wall task in the MuJoCo viewer.*

![wall task results](assets/learning/wall_task.png)
*Classical baselines first, then RL, all on the same 50 evaluation episodes. All 3 PPO seeds beat the potential field and never collide. The success rate spreads between seeds because most misses end just outside the 2 cm threshold.*

![generalisation](assets/learning/generalisation.png)
*Does it still avoid the wall when the wall moves? The one-wall PPO memorised a path (67 % → 21 % on moved walls). Trained on random, observed walls it reaches 77 % (APF 64 %), but on taller and wider walls than in training it collides in about a third of the episodes, where APF collides in 2 %. A policy for this family of walls, not a general obstacle-avoidance skill.*

![safety filter](assets/learning/safety_filter.png)
*Does a safety filter rescue the policy out of distribution? A naive always-on repulsion halves the success rate (it fights the policy). A control-barrier (CBF) projection — brake only when `clearance ≥ margin` is about to be violated — cuts the out-of-distribution collisions from 36 % to 5 % while success rises 21 % → 31 %, and it costs nothing in distribution (77 % → 81 %, 0 collisions). **Read the collision axis carefully**: it counts the *conservative safety envelope* (the enclosing link capsules) touching the wall, not the arm. An independent mesh-geometry checker finds **no real contact in any of the 300 evaluated episodes** — the arm never actually touched the wall ([audit](docs/EVALUATOR_AUDIT.md)).*

Window scene (narrow passage, forearm must go through a 0.5 × 0.4 m window): APF 0 %, PPO 0 %, RRT-Connect + path following 38 % (2 % collisions). Here sampling-based planning wins clearly — but the comparison is **not budget- or privilege-matched**: the planner is given the goal joint configuration and 30 s of execution, the reactive policies 6 s and only a Cartesian goal. A matched protocol is the first item on the research plan.

### M5.5: the command channel, and what a safety filter actually guarantees (simulation)

![execution gap](assets/learning/execution_gap.png)
*The plant does not execute a command, it executes a channel: the command is integrated into a position reference, the reference is clipped at the joint limits, and a bandwidth-limited servo tracks it. Measured on the simulator, the arm coasts **14.9 mrad** after a 1 rad/s command is cut. The left panel is the finding that matters: at every control step of a real episode we ask whether the filter's OWN reasoning was satisfied and whether the arm then ended the step below the enforced margin. The bounded kinematic QP's constraints hold while the arm still ends below the margin in 9 % of the steps at a 2 cm margin (32 % at 4 cm), with a median prediction error of 12 mm. **A kinematic filter's guarantee is a statement about the command, and the command is not the motion.** The look-ahead formulation fixes it (0.1–1.3 %).*

![frontier](assets/learning/frontier.png)
*The safety–completion frontier on the out-of-distribution walls, 50 episodes per cell. A single-constraint projection gets *more* dangerous as it grows more conservative (5.3 % → 1.3 % → 4.7 % collisions at 0, 6 and 10 cm): one constraint can only push the arm away from the closest obstacle and may push it into another one it cannot see. The bounded all-pair QP and the execution-aware filter are both collision-free from a 5 mm margin upwards.*

On the **frozen test range** (20000–20199, never inspected before, 3 seeds × 50 episodes per cell, three detectors per episode) the execution-aware filter reaches **54.0 %** completion out of distribution with **0 %** collisions — exactly the completion of an *unconstrained-margin* QP that collides in **16–22 %** of the same episodes, in every seed. Its completion advantage over the *safe* fixed-margin QP (45.3 %) is **not** established: the per-seed differences are +22, +2 and +2 points against an 8-point between-seed spread.

The honest part: the ablation shows the gain comes from the **look-ahead structure**, not from the explicit plant model — dropping the coast term from the constraint changes the frontier not at all. And the **tuning seeds overlap the training range**, which is why the tuning sweep could not resolve the difference the confirmatory run finds; the [research plan](docs/RESEARCH_PLAN.md) records both, together with the falsification criteria and their outcome.

The **manuscript is in the repository** as [`paper/manuscript.md`](paper/manuscript.md); every number in it has a committed JSON behind it, listed in [`docs/RESULTS.md`](docs/RESULTS.md).

## Every result, and where it comes from

Nothing under `data/` is excluded from the repository: the experiment results, the trained
policies, the training curves, the dynamics-identification data and the real-robot recordings are
all committed, so the analysis can be audited from the repository alone. The table below is the
index — claim → the script that produced it → the file that holds it.

| Claim | Script | Result file |
|---|---|---|
| The published M5 table (historical, dev seeds 1000–1049) | `learning/ex_learning/run_safe_eval.py` | `data/reports/learning/gen_safe/*.json` |
| Does the published table survive a corrected evaluator? (the Week-1 gate) | `evaluation/compare_evaluators.py` | `data/reports/learning/evaluator_comparison.json` |
| Is the collision metric the safety envelope or the arm? | `evaluation/independent_collision.py` | `data/reports/learning/evaluator_comparison.json` (the `independent` block) |
| The **confirmatory** run on the frozen test range 20000–20199, three detectors per episode | `evaluation/final_benchmark.py` | `data/reports/learning/final_benchmark_full.json` |
| The same on the **identified** plant (40.6 kg, the one fitted on the real CR10A) | `evaluation/final_benchmark.py --dynamics identified_run02` | `data/reports/learning/robustness_identified_dynamics.json` |
| The command channel of the simulated plant (the coast, the identified model) | `evaluation/measure_execution.py` | `data/reports/learning/execution_channel.json` |
| The **execution gap**: the filter's own claim vs what the plant did | `evaluation/execution_gap.py` | `data/reports/learning/execution_gap.json` |
| The safety–completion frontier of the filters | `learning/ex_learning/sweep_qp.py`, `sweep_exec.py` | `data/reports/learning/frontier_*.json`, `tune50_*.json` |
| The statistics (paired, hierarchical, separability) | `evaluation/confirmatory_stats.py`, `evaluation/stats.py` | `data/reports/learning/confirmatory_stats.json` |
| The narrow passage, matched protocol | `learning/ex_learning/matched_window.py` | `data/reports/learning/rrt_window_exec30s.json` |
| Classical baselines (DLS, APF) | `learning/ex_learning/baselines.py` | `data/reports/learning/baselines*.json` |
| Kinematic calibration (real data) | `evaluation/calibrate.py` | `data/reports/calibration/` |
| Dynamics identification on the real, moving CR10A | `evaluation/dynamics_id.py` | `data/reports/dynamics/`, `data/reports/run02/` |
| MuJoCo plant: tracking vs servo bandwidth | `evaluation/m4_tracking_study.py` | `data/reports/m4/` |
| MoveIt planning benchmark (400 plans, 8 planners, 3 scenes) | `ros2_ws/src/ex_planning_lab/` | `data/reports/planning_lab/` |
| The read-only digital twin sessions | `scripts/live_session.sh` | `data/reports/run0*/`, `data/live/`, `data/bags/` |
| Every figure | `evaluation/plot_*.py` | `assets/learning/*.png`, `assets/*/` |

The audited defects and their fixes: [`docs/EVALUATOR_AUDIT.md`](docs/EVALUATOR_AUDIT.md). The
working plan with the falsification criteria and their outcome:
[`docs/RESEARCH_PLAN.md`](docs/RESEARCH_PLAN.md). The dated work log:
[`docs/PROGRESS.md`](docs/PROGRESS.md). The compute assessment for the remote server:
[`docs/REMOTE_COMPUTE.md`](docs/REMOTE_COMPUTE.md).

Failures found and fixed on the way (all documented with regression tests):
- PPO learned to **crash on purpose** under a −distance reward.
- MuJoCo mesh distance queries returned swapped endpoints and spurious zeros.
- The passive viewer segfaults when it closes under WSLg.

## Hands-on: the full pipeline
![RViz MoveIt](assets/hands_on/rviz_moveit_wall.png)
*`hands_on/4_moveit_rviz_mujoco.bat`: MoveIt plans around the wall (RViz: planned trajectory trail, goal in orange, executed TCP path in cyan) and the MuJoCo plant executes it with physics in its own window (below, wall not modelled in physics).*

![MuJoCo plant window](assets/hands_on/mujoco_plant_window.png)

---

## Quick start (Windows 11 + WSL2)
```powershell
# 1. dedicated distro (does not touch your other WSL distros)
wsl --install Ubuntu-22.04 --name EmbodiedX-Humble --location C:\WSL\EmbodiedX-Humble --no-launch --web-download
wsl -d EmbodiedX-Humble -u root -- bash /mnt/c/Users/<you>/embodied-x/scripts/setup_wsl_humble.sh
```
```bash
# 2. inside EmbodiedX-Humble
cd /mnt/c/Users/<you>/embodied-x
bash scripts/fetch_third_party.sh          # pinned Dobot sources (only needed for tests/analysis)
bash scripts/build.sh                      # colcon -> ~/ex_ws_out
bash scripts/test.sh                       # unit tests (ROS-free core, safety, dynamics, learning envs)
bash scripts/integration_test.sh           # 4 stages, headless, ~3 min

# 3. demos (windows open via WSLg)
bash scripts/run_demo.sh simulation                                   # M1 digital twin in RViz
bash scripts/m2_session.sh start true plant:=mujoco mujoco_viewer:=true   # M2/M4: MoveIt + RViz + MuJoCo
cd learning && python3 -m ex_learning.view_policy --task wall --policy apf # M5 viewer
```
Or double-click the launchers in [`hands_on/`](hands_on/README.md). Live robot: **only** via [docs/LIVE_ROBOT_CHECKLIST.md](docs/LIVE_ROBOT_CHECKLIST.md).

## Repository map
```
ros2_ws/src/ex_description    CR10 URDF (official Dobot, MIT) + calibrated joint frames + world/flange/tcp
ros2_ws/src/ex_robot_state    core/ (ROS-free: feedback decoding, FK, calibration, dynamics regressor) · nodes · mock controller
ros2_ws/src/ex_bringup        M1 launch, modes, RViz config
ros2_ws/src/ex_moveit_config  MoveIt 2 + ros2_control (plant: mock | mujoco)
ros2_ws/src/ex_planning_lab   M2 scenes, benchmark, execute demo, capsule clearance metric
ros2_ws/src/ex_mujoco_sim     M4 MuJoCo plant: URDF->MJCF, servo model, dynamics presets
ros2_ws/src/ex_dobot_motion   the ONLY motion-capable package: whitelist, caps, validated plans, dry-run default
learning/                     M5 envs (reach, wall), baselines (DLS, APF), PPO, viewer, safety filters (bounded QP, execution-aware), execution model
evaluation/                   calibration, dynamics identification, sim2real, plots, the three-detector benchmark, the execution-gap measurement
paper/                        the manuscript (Markdown) + how to build the PDF
hands_on/                     double-click demos + guide (Chinese)
docs/                         one document per stage + SAFETY, LIVE_ROBOT_CHECKLIST, ROADMAP, PROGRESS, RESEARCH_STATE (self-contained brief for external analysis), RESEARCH_PLAN (the working plan + the audited defects and their fixes), EVALUATOR_AUDIT (the Week-1 gate: is the published table an artefact of the evaluator?), RESULTS (every number in one place, with the script that produced it), evidence/
```

## Troubleshooting
| Symptom | Cause / fix |
|---|---|
| `apt` in WSL can't reach mirrors, Windows can | Proxy in TUN/fake-ip mode not reachable from WSL NAT. Use the offline path in PROGRESS.md (download on Windows, install offline). |
| RViz robot not moving, `/joint_states` silent | QoS mismatch or adapter rejecting: `ros2 topic echo /diagnostics`, HUD shows the reason. |
| HUD text with huge gaps | RViz/OGRE font in WSLg renders spaces wide; HUD uses no spaces by design. |
| `CONFLICT: other publishers on /joint_states` | A `joint_state_publisher(_gui)` or a bag replaying `/joint_states` is running; stop it. |
| MoveIt answers inconsistently | More than one `move_group` alive: always use `scripts/m2_session.sh` (guarantees one). |
| bash `$'\r': command not found` | CRLF line endings; repo enforces LF via `.gitattributes` (CRLF only for `.bat`/`.ps1`). |

## License & attribution
Project code: MIT (see headers). `ex_description/meshes/cr10_official` and the derived CR10 macro come from [Dobot-Arm/DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4) (MIT, © Dobot).
Use of the physical robot and publication of recordings follow the owning organization's rules.
