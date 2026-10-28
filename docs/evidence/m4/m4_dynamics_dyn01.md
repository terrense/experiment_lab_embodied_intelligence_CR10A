# M4 dyn01: placeholder vs identified_run02 dynamics, raw per-run lines (SIMULATED)

Script: `scripts/m4_dynamics_compare.sh` (wall/sweep_60 trajectory from run03, 0.4 kg payload, 3 repeats). Source: `data/reports/m4/dyn01/log.txt`.

```
[dyn] dynamics placeholder, bandwidth 8 Hz, payload 0.4 kg
[dyn]   placeholder_bw8_s1.0_r1        ok=True code=1 tcp max 8.94 rms 5.23 final 0.02 mm sat=[0, 0, 0, 0, 0, 0] lag=2.51 load=1.22
[dyn]   placeholder_bw8_s1.0_r2        ok=True code=1 tcp max 8.16 rms 4.83 final 0.02 mm sat=[0, 0, 0, 0, 0, 0] lag=2.94 load=1.25
[dyn]   placeholder_bw8_s1.0_r3        ok=True code=1 tcp max 9.04 rms 5.35 final 0.02 mm sat=[0, 0, 0, 0, 0, 0] lag=1.68 load=1.04
[dyn]   placeholder_bw8_s1.8_r1        ok=True code=1 tcp max 14.25 rms 8.77 final 0.05 mm sat=[0, 0, 0, 0, 0, 0] lag=2.1 load=0.95
[dyn]   placeholder_bw8_s1.8_r2        ok=True code=1 tcp max 14.24 rms 8.57 final 0.05 mm sat=[0, 0, 0, 0, 0, 0] lag=1.94 load=1.4
[dyn]   placeholder_bw8_s1.8_r3        ok=True code=1 tcp max 13.86 rms 7.93 final 0.05 mm sat=[0, 0, 0, 0, 0, 0] lag=2.12 load=1.31
[dyn]   plant: [mujoco_sim-6] [INFO] [1793167075.046086871] [mujoco_sim]: MuJoCo 3.3.7 plant: dt 1.0 ms, mass 25.19 kg (+payload 0.4 kg, controller believes 0.4 kg), M_eff [2.848, 6.086, 2.049, 0.091, 0.06, 0.02], kp [7196.0, 15377.0, 5177.0, 229.0, 153.0, 51.0], dynamics placeholder (link mass x1.0), frictionloss [3.0, 3.0, 2.0, 0.8, 0.8, 0.4]
[dyn] dynamics placeholder, bandwidth 16 Hz, payload 0.4 kg
[dyn]   placeholder_bw16_s1.0_r1       ok=True code=1 tcp max 3.98 rms 1.62 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=2.02 load=1.36
[dyn]   placeholder_bw16_s1.0_r2       ok=True code=1 tcp max 4.09 rms 1.87 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.73 load=1.33
[dyn]   placeholder_bw16_s1.0_r3       ok=True code=1 tcp max 4.18 rms 1.93 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.95 load=1.37
[dyn]   placeholder_bw16_s1.8_r1       ok=True code=1 tcp max 7.14 rms 3.42 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=2.26 load=1.24
[dyn]   placeholder_bw16_s1.8_r2       ok=True code=1 tcp max 6.43 rms 2.85 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.47 load=1.2
[dyn]   placeholder_bw16_s1.8_r3       ok=True code=1 tcp max 6.71 rms 3.23 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.86 load=1.09
[dyn]   plant: [mujoco_sim-6] [INFO] [1793167160.054883617] [mujoco_sim]: MuJoCo 3.3.7 plant: dt 1.0 ms, mass 25.19 kg (+payload 0.4 kg, controller believes 0.4 kg), M_eff [2.848, 6.086, 2.049, 0.091, 0.06, 0.02], kp [28783.0, 61507.0, 20709.0, 917.0, 610.0, 205.0], dynamics placeholder (link mass x1.0), frictionloss [3.0, 3.0, 2.0, 0.8, 0.8, 0.4]
[dyn] dynamics identified_run02, bandwidth 8 Hz, payload 0.4 kg
[dyn]   identified_run02_bw8_s1.0_r1   ok=True code=1 tcp max 13.27 rms 8.61 final 0.01 mm sat=[0, 0, 0, 0, 0, 0] lag=1.68 load=1.23
[dyn]   identified_run02_bw8_s1.0_r2   ok=True code=1 tcp max 12.98 rms 8.59 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.76 load=0.96
[dyn]   identified_run02_bw8_s1.0_r3   ok=True code=1 tcp max 13.49 rms 8.92 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.81 load=0.89
[dyn]   identified_run02_bw8_s1.8_r1   ok=True code=1 tcp max 16.96 rms 10.81 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.77 load=0.83
[dyn]   identified_run02_bw8_s1.8_r2   ok=True code=1 tcp max 17.43 rms 11.69 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.91 load=0.94
[dyn]   identified_run02_bw8_s1.8_r3   ok=True code=1 tcp max 17.80 rms 11.38 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.51 load=0.8
[dyn]   plant: [mujoco_sim-6] [INFO] [1793167245.486092784] [mujoco_sim]: MuJoCo 3.3.7 plant: dt 1.0 ms, mass 40.96 kg (+payload 0.4 kg, controller believes 0.4 kg), M_eff [4.273, 9.518, 3.099, 0.109, 0.064, 0.02], kp [10797.0, 24048.0, 7831.0, 276.0, 161.0, 51.0], dynamics identified_run02 (link mass x1.636), frictionloss [10.39, 14.15, 9.33, 1.48, 1.46, 1.43]
[dyn] dynamics identified_run02, bandwidth 16 Hz, payload 0.4 kg
[dyn]   identified_run02_bw16_s1.0_r1  ok=True code=1 tcp max 4.56 rms 2.15 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=2.18 load=0.73
[dyn]   identified_run02_bw16_s1.0_r2  ok=True code=1 tcp max 5.58 rms 2.83 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=2.4 load=0.85
[dyn]   identified_run02_bw16_s1.0_r3  ok=True code=1 tcp max 4.72 rms 2.17 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=2.56 load=0.73
[dyn]   identified_run02_bw16_s1.8_r1  ok=True code=1 tcp max 7.06 rms 3.47 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=3.28 load=0.78
[dyn]   identified_run02_bw16_s1.8_r2  ok=True code=1 tcp max 7.54 rms 4.12 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=1.76 load=0.66
[dyn]   identified_run02_bw16_s1.8_r3  ok=True code=1 tcp max 6.60 rms 3.14 final 0.00 mm sat=[0, 0, 0, 0, 0, 0] lag=2.07 load=0.74
[dyn]   plant: [mujoco_sim-6] [INFO] [1793167333.108310747] [mujoco_sim]: MuJoCo 3.3.7 plant: dt 1.0 ms, mass 40.96 kg (+payload 0.4 kg, controller believes 0.4 kg), M_eff [4.273, 9.518, 3.099, 0.109, 0.064, 0.02], kp [43187.0, 96193.0, 31324.0, 1105.0, 644.0, 206.0], dynamics identified_run02 (link mass x1.636), frictionloss [10.39, 14.15, 9.33, 1.48, 1.46, 1.43]
[dyn] done
```
