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

Lesson: a physics plant behind a message bus inherits the bus's timing noise. Any derivative taken on the receive side has to use the sender's clock.

## Results
See §Results (written from `data/reports/m4/run02`).
