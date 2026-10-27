# learning/ — MuJoCo reach task (SIMULATION ONLY)

`ex_learning/envs/cr10a_reach.py` provides a Gymnasium-style `reset/step` API with no gymnasium dependency. It reuses the M4 plant: the same MJCF built from the calibrated `ex_cr10` URDF, the same servo model, 1 kHz physics and a 50 Hz policy.
- Action: joint-velocity command in [-1, 1]^6 (× 1 rad/s).
- Observation: 21-dim vector.
- Success: TCP within 2 cm of a random goal in the workspace box in front of the arm (x −0.95…−0.45, y ±0.45, z 0.35…0.95 m), within 4 s.

Rule: a baseline comes before RL. Run with `cd learning && python3 -m ex_learning.baselines --episodes 50`.

| Policy | Success (50 eps, seeds 1000–1049) | Final distance, median | Time to reach, median | Path, median |
|---|---|---|---|---|
| random | 0 % | 511 mm | – | 2.00 m |
| DLS Jacobian (gain 3, λ 0.05) | **100 %** | ≈ 0 mm | 1.14 s | 0.50 m |

Measured 2026-10-27, simulated data (`data/reports/learning/baselines.json`).

Takeaway: free-space reaching is solved by a 5-line classical controller. An RL policy is only worth training on a task where that controller struggles, such as obstacles in the workspace (M2 scenes), unknown payload (see the M4 payload-mismatch results) or contact. RL training itself is **not done yet**: torch / gymnasium / SB3 are not installed in `EmbodiedX-Humble` (offline WSL; wheels must come from the Windows side).
