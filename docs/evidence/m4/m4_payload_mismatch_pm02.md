# M4 payload mismatch pm02: raw per-run lines (SIMULATED)

Source: `data/reports/m4/pm02/log.txt`, written by `scripts/m4_payload_mismatch.sh` (bw 8 Hz, speed 1.0, wall/sweep_60).

```
[pm] true payload 10 kg, configured 10 kg
[pm]   pm_t10_c10_r1      ok=True code=1 tcp max 6.52 rms 3.44 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.67
[pm]   pm_t10_c10_r2      ok=True code=1 tcp max 7.47 rms 4.00 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.31
[pm]   pm_t10_c10_r3      ok=True code=1 tcp max 6.75 rms 3.58 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.73
[pm] true payload 10 kg, configured 8 kg
[pm]   pm_t10_c8_r1       ok=True code=1 tcp max 7.02 rms 4.22 final 1.48 mm sat=[0, 0, 0, 0, 0, 0] lag=1.99
[pm]   pm_t10_c8_r2       ok=True code=1 tcp max 7.56 rms 4.62 final 1.48 mm sat=[0, 0, 0, 0, 0, 0] lag=1.18
[pm]   pm_t10_c8_r3       ok=True code=1 tcp max 7.40 rms 4.44 final 1.48 mm sat=[0, 0, 0, 0, 0, 0] lag=1.68
[pm] true payload 10 kg, configured 5 kg
[pm]   pm_t10_c5_r1       POSITIONING FAILED: static hold error 10.37 mm, joints [-0.0256, 0.24, 0.3467, 1.2834, 0.0341, -0.0071] deg
[pm]   pm_t10_c5_r2       POSITIONING FAILED: static hold error 8.89 mm, joints [-0.0433, 0.1702, 0.3004, 1.2714, 0.0636, -0.0291] deg
[pm]   pm_t10_c5_r3       POSITIONING FAILED: static hold error 10.44 mm, joints [-0.0529, 0.245, 0.3656, 1.1847, 0.029, -0.0135] deg
[pm] true payload 10 kg, configured 0 kg
[pm]   pm_t10_c0_r1       POSITIONING FAILED: static hold error 41.49 mm, joints [0.0304, 0.572, 1.5776, 6.3056, 0.139, -0.0238] deg
[pm]   pm_t10_c0_r2       POSITIONING FAILED: static hold error 42.86 mm, joints [0.0333, 0.6068, 1.6696, 6.2565, 0.119, -0.0081] deg
[pm]   pm_t10_c0_r3       POSITIONING FAILED: static hold error 42.46 mm, joints [0.0405, 0.6145, 1.6123, 6.339, 0.1529, 0.0067] deg
[pm] true payload 0 kg, configured 0 kg
[pm]   pm_t0_c0_r1        ok=True code=1 tcp max 9.16 rms 5.47 final 0.03 mm sat=[0, 0, 0, 0, 0, 0] lag=1.71
[pm]   pm_t0_c0_r2        ok=True code=1 tcp max 9.58 rms 5.75 final 0.03 mm sat=[0, 0, 0, 0, 0, 0] lag=1.45
[pm]   pm_t0_c0_r3        ok=True code=1 tcp max 9.21 rms 5.54 final 0.03 mm sat=[0, 0, 0, 0, 0, 0] lag=1.87
[pm] true payload 0 kg, configured 10 kg
[pm]   pm_t0_c10_r1       POSITIONING FAILED: static hold error 13.01 mm, joints [-0.0069, -0.2961, -0.4477, -1.5228, -0.0154, 0.0065] deg
[pm]   pm_t0_c10_r2       POSITIONING FAILED: static hold error 13.79 mm, joints [-0.0233, -0.2956, -0.4991, -1.5807, -0.0372, 0.0344] deg
[pm]   pm_t0_c10_r3       POSITIONING FAILED: static hold error 13.43 mm, joints [-0.0162, -0.2661, -0.4992, -1.5883, -0.0616, 0.046] deg
[pm] done
```
