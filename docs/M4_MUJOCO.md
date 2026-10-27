# M4 — MuJoCo Physics Plant behind ros2_control (SIMULATION ONLY)

## Goal
Replace the dynamics-free `mock_components` plant from M2 with a physics model, **without touching anything above it** (MoveIt, OMPL, TOTG, JointTrajectoryController, the M2 tools). Then measure how planned trajectories are actually tracked by a servo-driven arm under different servo bandwidths, payloads and speeds.

## Architecture
```
move_group ─FollowJointTrajectory─► arm_controller (JTC, 100 Hz, tolerances)
                                         │ position set-points
                                         ▼
         ros2_control_node: topic_based_ros2_control/TopicBasedSystem
           │ /mujoco/joint_commands (JointState, sender-stamped)   ▲ /mujoco/joint_states (500 Hz)
           ▼                                                       │
         ex_mujoco_sim: servo drive @ 1 kHz + MuJoCo 3.3.7 rigid-body dynamics (real-time paced)
```
Run: `bash scripts/m2_session.sh start true plant:=mujoco [bandwidth_hz:=8] [payload_kg:=0] [mujoco_viewer:=true]`

## Model (single source of truth)
- The MJCF is built **at runtime from the same `robot_description`** that MoveIt uses (calibrated CR10A by default). Unit test: MuJoCo FK agrees with URDF FK to about 1.5 µm (the difference is the finite precision of `mj_saveLastXML`).
- Link masses and inertias come from the official Dobot URDF (CAD). **Total arm mass in the URDF: 24.8 kg. The CR10A datasheet weight is 40 kg** (it includes the base and the drives/brakes). Inertias are probably underestimated. This needs identification against real joint-torque data.
- Robot geometry is visual-only in physics (contype 0). Collision avoidance is the planner's job; MuJoCo provides the arm dynamics.

### Explicit assumptions (UNVERIFIED, `DynamicsAssumptions`)
| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| damping (N·m·s/rad) | 8 | 8 | 5 | 1 | 1 | 0.5 |
| armature (kg·m²) | 0.40 | 0.40 | 0.20 | 0.05 | 0.05 | 0.02 |
| friction loss (N·m) | 3 | 3 | 2 | 0.8 | 0.8 | 0.4 |
| torque limit (N·m) | 300 | 300 | 150 | 60 | 60 | 30 |

## Servo-drive model (`ex_mujoco_sim/servo.py`)
`τ = clip(kp(q_ref − q) + kd(dq_ref − dq) + τ_bias, ±τ_max)`
- `kp = M_ii ω²`, `kd = 2ζ M_ii ω`: gains from each joint's effective inertia (mass-matrix diagonal + armature) and a target bandwidth (default 8 Hz, ζ = 0.9).
- `q_ref, dq_ref`: first-order hold on the 100 Hz set-points. **Velocity is differenced on the sender timestamps**, then low-pass filtered (τ = 20 ms). See the failure analysis.
- `τ_bias = qfrc_bias`: perfect gravity + Coriolis feedforward (optimistic).

## Failure analysis (found while building)
| # | Symptom | Root cause | Fix |
|---|---|---|---|
| 1 | Arm never moved; node looked healthy | `import mujoco.viewer` inside the thread function made `mujoco` a function-local name → `UnboundLocalError` at `mj_step`. The thread died; the process lived on. | Different import name; a physics-thread crash now terminates the node |
| 2 | **JTC reported SUCCESS for a plant that never moved** | No per-joint `goal` / `trajectory` tolerances → JTC only checks that the time has run out | Tolerances 0.005 / 0.15 rad. This matters for real hardware too. |
| 3 | `m2_session.sh` reported "found 0 move_group" right after start | A one-shot `--no-daemon` discovery returned before move_group was seen | Longer spin window + retries |
| 4 | Same trajectory, same config → 1 of 3 runs diverged (25° on the wrist) near the final deceleration | Velocity feedforward was differenced on **arrival** times. Measured jitter: arrival 3.9–16 ms vs sender stamps 8.9–11 ms for a 10 ms stream, so the velocity estimate was up to about 2.5× off, giving kd torque spikes and divergence near the torque limits | Use sender stamps + 20 ms low-pass; after the fix 3/3 succeed. Unit test reproduces the jitter. |

| 5 | A 4 Hz servo could not even finish the positioning move → `execute_demo` crashed → **runs silently missing**, which biased results towards "good" | Positioning failure was treated as an exception | Retry the positioning move at 0.1×/0.05× speed; if it still fails, **write a failure report** (counted in the study) |
| 6 | Study run02: errors grew as the study progressed (16 Hz/no payload: 83–97 mm, hundreds of torque-saturated steps vs about 4 mm in run01). **Run02 discarded.** | **33 orphaned `robot_state_monitor` processes** (started by every MoveIt session, missing from the cleanup list) each used 13–67 % CPU. Load average reached 26 on 16 cores, the Python 1 kHz plant could not keep real time, and state lagged behind commands. | Cleanup covers every EMBODIED-X node; start waits for loadavg < 4; the study runs with `monitor:=false`; reports now carry **per-move plant lag** and loadavg. Re-check in a clean box: 6.0 / 6.1 / 6.1 mm, 0 saturation (3 repeats). |

Lessons:
- A physics plant behind a message bus inherits the bus's timing noise. Any derivative taken on the receive side has to use the sender's clock.
- **A real-time simulator is only valid while it keeps real time.** Measure and report that per run; otherwise CPU contention shows up as "dynamics".
- The M2 planner benchmark run02 also ran with orphaned monitors present, so it is being re-run (run03) for comparison.

## Results (run03, clean environment, SIMULATED data)
Design: one RRTConnect trajectory (wall/sweep_60, 4.6 s at 0.3× speed scaling), replayed unchanged. Conditions: mock plant, then MuJoCo with servo bandwidth {4, 8, 16} Hz × payload {0, 5, 10} kg × speed {1.0, 1.8}, **3 repeats each** (60 executions). Load average during the study ≤ 1.27; plant real-time lag during moves: median 2.2 ms.
Full table: `docs/evidence/m4/m4_run03_summary.md`.

![tracking](../assets/m4/tracking_tcp_max.png)

| Servo bandwidth | Max TCP error, median (speed 1.0 / 1.8) | Executions completed | Start latency |
|---|---|---|---|
| mock (no dynamics) | 2.2 / 2.8 mm | 5/6 (1 unexplained abort, see below) | 26–28 ms |
| 4 Hz | 19–189 / 28–42 mm | **0/18**: JTC aborts (trajectory tolerance 0.15 rad); 2 runs could not even reach the start pose | 48–97 ms |
| 8 Hz | 7.6–9.2 / 12.0–15.1 mm | 15/18 | 30–40 ms |
| 16 Hz | 3.4–4.6 / 5.7–6.6 mm | 17/18 | 28–30 ms |

Findings (under the stated dynamics assumptions):
1. **Tracking error ≈ 1/bandwidth.** 8 → 16 Hz halves the error (9.2 → 4.6 mm at no payload). 1.8× speed raises it by about 1.5×. A 4 Hz servo cannot track a 0.3×-speed TOTG trajectory within a 0.15 rad tolerance.
2. **Payload barely matters here, and that is an artefact of the model.** The servo gains scale with the effective inertia *including* the payload, and the gravity/Coriolis feed-forward is perfect, so a known payload is fully compensated (errors even shrink slightly with payload). On a real Dobot this corresponds to a *correctly configured* `SetPayload`. The informative next experiment is a **payload mismatch** (true mass ≠ configured mass).
3. Torque saturation disappeared in the clean environment, except J3 at 16 Hz / 10 kg / 1.8× (51 steps). The saturation seen in run02 was mostly caused by the plant losing real time.
4. Even the best physics case (16 Hz, 3.4–6.6 mm) is about 2× the mock. Every planner-level clearance from M2 (median 1–2 cm) has to budget for **model error (M1, ≤ 5.5 mm before calibration) + tracking error (this study, 3–15 mm)**. Padding ≥ 2 cm is the minimum.

Open issue: 7 of 60 executions (including 1 on the mock plant) aborted with MoveIt code −4 at an unexplained moment, while the other repeats of the same condition were tight. 8 dedicated repeats on the mock afterwards: 0 failures. The cause is unknown and not plant-related. Session logs are now archived (`data/sessions/`) so the next occurrence is diagnosable. Statistics use the per-condition median, and aborts are counted, not hidden.

## Payload mismatch (pm02, SIMULATED data)
Question: what happens when the payload on the arm (true) differs from the payload the controller believes (`SetPayload`)?
Implementation: `sim_node` keeps a second MuJoCo model with the **configured** payload. The servo gains (kp = M·ω²) and the gravity/Coriolis feed-forward come from that model, and the physics run on the true one. Same trajectory as run03 (wall/sweep_60), bw 8 Hz, speed 1.0, 3 repeats. Script: `scripts/m4_payload_mismatch.sh`. Full table: `docs/evidence/m4/m4_payload_mismatch_pm02.md`.

| True / configured | Result (3/3 identical) | Max TCP error | Final error | Static hold error (failed runs) |
|---|---|---|---|---|
| 10 / 10 kg | OK | 6.5–7.5 mm | 0.00 mm | – |
| 10 / 8 kg | OK | 7.0–7.6 mm | **1.48 mm** | – |
| 10 / 5 kg | **positioning failed** | – | – | 8.9–10.4 mm (J4 ≈ 1.25°) |
| 10 / 0 kg | **positioning failed** | – | – | 41.5–42.9 mm (J4 ≈ 6.3°) |
| 0 / 0 kg | OK | 9.2–9.6 mm | 0.03 mm | – |
| 0 / 10 kg | **positioning failed** | – | – | 13.0–13.8 mm (J4 ≈ −1.55°, sags *upward*) |

"Positioning failed" means the arm could not settle at the trajectory's start pose within the JTC goal tolerance (0.005 rad ≈ 0.29°) after 3 attempts. The static error was measured after a 2 s hold.

Mechanism (checked against the logged gains): the servo here is **PD only (no integral term)**, so an uncompensated gravity torque Δτ ≈ Δm·g·r leaves a steady-state error Δq ≈ Δτ / kp(configured). Wrist J4 dominates:
- kp_J4 = 200 / 561 / 923 N·m/rad for a configured 0 / 5 / 10 kg.
- Back-computing the lever arm r from the measured sag gives 0.22–0.26 m in all three failing cases, so a single linear model explains them.
- That also explains the asymmetry: "configured too light" is worse than "configured too heavy" because the light configuration also *lowers the stiffness*.
- 2 kg under (10/8) still passes the tolerance, but leaves a permanent 1.5 mm offset.

Caveats:
- The real Dobot servo almost certainly has integral action and its own load identification, so the real arm would not sag like this. **This result belongs to the model, not the CR10A.** What carries over is the direction: a wrong `SetPayload` shows up as static offset and reduced stiffness, and "too light" is the dangerous side.
- Verifying this on hardware needs the excitation run (`ex_dobot_motion`) plus a known mass. It has not been done (UNVERIFIED).
