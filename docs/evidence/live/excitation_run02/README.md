# Live excitation run02: real CR10A, program-controlled motion (2026-10-28)

**REAL ROBOT DATA.** The owner was at the robot with the e-stop in reach and had unit authorization (confirmed in the session). Tool: 0.4 kg stereo camera, so `EnableRobot(0.4)` was sent. Collision-detection level could not be checked: there is no teach pendant, and the setting was not found in DobotStudio Pro. The owner accepted the e-stop plus the runner's per-packet checks as the protection.

Command:
`python3 -m ex_dobot_motion.runner --robot-ip 192.168.201.1 --payload-kg 0.4 --amplitude-scale 0.5 --cycles 4 --waypoints 240 --execute --confirm …`

| Item | Result |
|---|---|
| Moves | 241 MovJ, 84.7 s, speed/acc/SpeedFactor 20 % |
| Joint span (deg) | J1 8.0, J2 10.0, J3 10.0, J4 15.0, J5 15.0, J6 15.0 (= plan) |
| Max joint speed (deg/s) | 2.2, 4.1, 6.8, 9.8, 9.7, 10.3 |
| TCP height | 0.631 m at start, plan minimum 0.506 m above the base plane (the table corner under the flange was about 25 cm below the camera) |
| Return to start | within 0.01 deg; robot DISABLED afterwards, no error |
| Feedback | 10514 packets on port 30004; controller timestamp step 8 ms median (jitter 4–14 ms); `qd` vs numerical dq/dt correlation 0.986 for every joint |
| Currents/torques | `i` and `m` are populated (p99 \|m\| = 15, 94, 69, 6.2, 3.9, 3.0, units as reported by the controller, not yet verified) |

Findings:
1. **Controller quirk.** Exactly one packet during the DISABLED → ENABLE transition has a valid TestValue but garbage content: q = 0, `error_status` = −128, `collision_state` = 70, `current_command_id` ≈ 4.6e18. It was seen in run01 (row 88) and run02 (row 87) and came after a 294 ms pause in the stream. It must be dropped in analysis, and the runner deliberately does not run its abort checks while waiting for ENABLE. Had such a packet arrived during motion, the runner would have aborted, which errs on the safe side.
2. run01 (22 s, 61 moves) is not usable for identification: the recorder wrote `%.6g` and destroyed the timestamps. This was fixed (full precision, regression test) before run02.
3. Excitation is mild by design (half amplitude, about 10 °/s). It is enough for gravity and friction terms, but probably weak for inertial parameters. A dynamics-ID analysis has not been run on this data yet (UNVERIFIED).
