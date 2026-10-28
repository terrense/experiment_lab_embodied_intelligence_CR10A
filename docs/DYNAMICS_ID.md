# Dynamics identification of the CR10A (M4.5)

Data: real excitation run02 (`docs/evidence/live/excitation_run02`, 85 s, half amplitude, joint speeds ≤ 10 °/s, 0.4 kg camera on the flange). Torque = controller field `m` (assumed to be joint torque in N m; this was not verified, but the sign and magnitude agree with the URDF gravity torque).

## Method
- `ex_robot_state/core/dynamics.py`: Newton-Euler inverse dynamics that is **linear in the 10 standard inertial parameters per link**, plus per-joint columns: armature (reflected rotor inertia), viscous, Coulomb (`tanh(qd/v_eps)`) and offset. Base parameters come from QR with pivoting on column-normalised random-state data: 54 for the rigid chain plus joint terms.
  - The calibrated joint frames contain micro-radian tilts. These leave one direction at a singular value of 1.6e-4, which is treated as unidentifiable.
- **Verified against MuJoCo** (`ex_mujoco_sim/test/test_dynamics_regressor.py`, 5 tests): the rigid-body torque equals `mj_inverse` to within 1e-5 relative, and the armature and viscous columns are exact.
  - The first test run "failed" because random states exceeded J3's limits, and MuJoCo's inverse dynamics then included joint-limit forces. The fix was to sample inside the limits.
- Pipeline `evaluation/dynamics_id.py`:
  1. Clean the packets.
  2. Resample to 8 ms on the controller clock.
  3. Apply a zero-phase 4 Hz Butterworth filter (the same filter on q, qd and τ); qdd = d(qd)/dt.
  4. Split by time: train 50 %, validation 25 %, test 25 %. `v_eps` and the ridge λ are chosen on validation only.
- Models:
  - M0: URDF (+ tool point mass), nothing fitted.
  - M1: M0 + armature, viscous, Coulomb and offset per joint (24 parameters).
  - M1s: k·M0 + friction, no offsets (19 parameters; tests the "masses too light" hypothesis).
  - M2: all 60 base parameters, ridge-regularised toward the M1 solution.

## Step 1: validate on synthetic data with known truth (SIMULATED)
`evaluation/dynamics_synthetic.py` replays run02's *measured* joint trajectory on the M4 MuJoCo plant: known armature, damping and frictionloss, a 0.4 kg flange payload, and 0.3 N m torque noise.

| Parameter (J1..J6) | Truth | Estimate (synthetic01, correct prior) | |
|---|---|---|---|
| armature [kg m²] | 0.4, 0.4, 0.2, 0.05, 0.05, 0.02 | 0.39, 0.38, 0.21, 0.048, 0.084, 0.018 | ✅ (J5 +70 %) |
| Coulomb [N m] | 3, 3, 2, 0.8, 0.8, 0.4 | 2.8, 3.1, 1.9, 0.67, 0.64, 0.31 | ✅ within ~20 % |
| viscous [N m s/rad] | 8, 8, 5, 1, 1, 0.5 | 17.7, 5.1, 7.0, 2.8, 2.8, 1.5 | ❌ off by 0.6–3× |
| mass scale k (synthetic02, true masses ×1.6) | 1.6 | 1.555 | ✅ |

Lessons from the known-truth test:
- At ≤ 10 °/s the viscous friction is not identifiable.
- If offsets are allowed, a gravity/payload error is absorbed into the offsets: the J2 offset becomes 23 N m when the masses are ×1.6.
- If link inertia is missing from the prior, the "armature" column absorbs it: J2 armature 6.2 instead of 0.4.
- A torque prediction VAF of 97–99 % says nothing about whether the parameters are right.

## Step 2: real CR10A (run02)
![fit](../assets/dynamics/run02_torque_fit.png)

| Model | Test RMS J1..J6 [N m] | Test VAF % |
|---|---|---|
| M0 URDF + 0.4 kg tool | 10.2, 24.9, 25.6, 2.4, 1.6, 1.8 | ≈ 0 (17.6 on J2) |
| M1s k·URDF + friction | 1.9, 4.2, 3.8, 0.57, 0.37, 0.24 | 95–99 |
| M1 URDF + friction + offsets | 1.7, 2.6, 1.2, 0.27, 0.30, 0.23 | 96–98.5 |
| M2 base params (ridge) | 1.6, 2.5, 1.1, 0.23, 0.28, 0.24 | 96.8–98.7 |

(Torque std on the test part: 10.2, 15.6, 9.5, 1.7, 1.6, 1.8 N m.)

Findings:
1. **The official URDF underestimates the mass.** With the URDF inertials, the gravity torque on J2/J3 is about 24 N m short.
   - The single-scale fit gives **k = 1.64**, i.e. 24.8 kg × 1.64 ≈ 40.6 kg, against the 40 kg in the datasheet.
   - The synthetic test shows k is recoverable (1.555 for a true 1.6).
   - A uniform scale does not explain everything: M1s is worse than M1 on J2/J3. The mass distribution also differs.
2. **Friction dominates at these speeds.** Coulomb (M1, v_eps = 0.002 rad/s chosen on validation): 10.4, 14.1, 9.3, 1.5, 1.5, 1.4 N m. This is 3–5× the placeholder `frictionloss` in `DynamicsAssumptions` (3, 3, 2, 0.8, 0.8, 0.4).
3. Viscous (5.0, 4.7, 14.1, 4.4, 1.9, 4.1) and armature (12.1, 12.7, 4.4, 0.32, 0.29, 0.31) **must not be read as physical values**. The synthetic test shows that viscous friction is not identifiable at these speeds and that armature absorbs missing link inertia; 12 kg m² is implausible for a reflected rotor inertia.
4. Useful today: within the excited region, a 24-parameter model predicts held-out joint torque to 1–2.6 N m on J1–J3 and 0.2–0.3 N m on the wrist. That is enough for a residual-torque collision monitor or for re-tuning the M4 plant's friction. It has **not** been tested on a different trajectory or posture.

## What it would take to identify the individual parameters
- A larger and faster excitation: full amplitude, a different posture (J2/J3 ±30° so that gravity changes), and speeds that cover a velocity range. This needs the owner on site, plus a wider envelope (more than `max_joint_excursion 20°`), which requires the owner's explicit OK because it widens the motion caps.
- A second trajectory for validation, to test generalisation.
- Design it in simulation first: `dynamics_synthetic.py` + `dynamics_id.py`, and pick the trajectory that recovers the known truth.
