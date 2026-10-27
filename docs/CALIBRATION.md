# CR10A Kinematic Alignment (on-site data, 2026-10-26)

**Question.** The official `cr10` URDF matches the real CR10A controller to 1.6–5.5 mm. Can we make our model match the controller exactly, and what would that mean?

**What the targets are.** Every feedback packet carries `QActual` together with the TCP pose that the **controller computes from those angles** using its factory-calibrated kinematic model (tool 0 / user 0). Fitting a model to these targets **recovers the controller's calibrated model**. It makes our FK, RViz display, MoveIt planning and future hand-eye calibration agree with the robot's own idea of where its flange is. It does **not** measure how accurately the physical arm moves; that needs an external reference such as a laser tracker. The datasheet repeatability is ±0.03 mm; absolute accuracy is unknown.

## Method (`evaluation/kinematic_calibration.py`, core `ex_robot_state/core/calibration.py`)
- **Model.** For each joint: origin translation δt (3), small origin rotation δr (3), and zero offset δθ (1). Plus a flange offset (3). That is 45 parameters. Joint offsets are folded into the URDF as rotations about each joint axis, because URDF has no offset field.
- **Data.** Two sessions of per-packet logs (`docs/evidence/m1_live/*.csv.gz`), deduplicated into 0.05° joint cells: 3424 + 3129 distinct poses. Static periods would otherwise dominate the fit.
- **Solver.** Gauss–Newton with Tikhonov regularization. With position-only data the Jacobian has **numerical rank 25 of 45**: directions such as translation along consecutive parallel axes are unidentifiable. Regularization keeps those at the minimum-norm split. Only the identifiable combinations are meaningful (e.g. "3.0 mm along the J6 axis", not how it is split between the joint-6 origin and the flange).
- **Validation.** Cross-session: fit on one session, test on the other. A same-session random split would be inflated by temporal autocorrelation. A synthetic recovery test (`test_recovers_synthetic_perturbation`) checks the method itself.
- **Orientation.** A constant flange rotation is invisible in position-only data. It is estimated from the bag orientation data as the chordal mean of R_fkᵀ R_ctrl (quaternion order w,x,y,z, verified on site).

## Results (real robot)
| Model | Params | Held-out RMS, test s1 / s2 (mm) |
|---|---|---|
| Nominal official `cr10` | 0 | 3.37 / 3.06 |
| A: joint zero offsets | 6 | 1.96 / 2.49 |
| B: A + flange offset | 9 | 0.89 / 1.31 |
| **C: full, regularised** | 45 (rank 25) | **0.002 / 0.011** |

![cdf](../assets/m1_live/calibration_cdf.png)

Final model (both sessions) checked through the independent rosbag pipeline (double-precision raw packets, 30,645 samples):

| | position RMS / max | orientation mean |
|---|---|---|
| nominal | 1.61 / 2.85 mm | 0.42° |
| **calibrated** | **0.0002 / 0.0008 mm** | **0.00004°** (after a 0.207° flange tilt, mostly about x) |

Identified corrections are small and physically plausible: translations ≤ 1.6 mm, rotations ≤ 0.37° (largest: a 0.37° twist correction at joint 3), zero offsets ≤ 0.01°. Full values: `docs/evidence/m1_live/calibration_report.json`.

Caveats:
- The bags cover the same time span as the CSV sessions, so the bag check validates the **pipeline**, not new data. The held-out evidence is the cross-session row above.
- The 3 orientation parameters were estimated on the bag data itself (in-sample; 3 parameters vs 6k samples).

## Usage
- `ex_cr10.urdf.xacro calibrated:=true|false`. **Default `true` in all launch files** (live display, simulation, MoveIt). `false` keeps the official geometry (used by unit tests as the reference).
- Regenerate: `python3 evaluation/kinematic_calibration.py --write-urdf` (writes `ex_description/urdf/dobot_cr10_calibrated.xacro`).

## Ablation insight
Zero offsets alone explain only about 30 % of the discrepancy. Most of it is link geometry: link lengths, twists, and the J6 flange distance (+3.0 mm along the J6 axis). This is the signature of a full geometric factory calibration, not just encoder zeroing.

## Consequences
- M2: our model now agrees with the controller to well under 1 mm. The remaining uncertainty for real execution is the controller's own absolute accuracy, which is not measured here. **Keep `default_robot_padding` ≥ 2 cm.**
- M3 (hand-eye calibration): use the calibrated FK as the kinematic side of AX = XB. Otherwise a 1.6–5.5 mm pose-dependent bias would leak into X.
