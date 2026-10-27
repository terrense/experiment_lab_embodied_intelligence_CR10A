# Case Study 1 — A Read-Only Digital Twin of a Dobot CR10A

*EMBODIED-X · Milestone 1 · 2026-10*

## Problem
Mirror a real 6-axis collaborative arm (Dobot CR10A) in RViz in real time: model, TF tree, joint angles, TCP pose and trail, plus data-health signals. The arm belongs to a shared lab, so the twin had to be **provably unable to command the robot**. Two things were unknown at the start: whether any official robot model actually matches a "CR10A" (the vendor only lists CR10 and CR10AF), and how the feedback protocol behaves on a real controller.

## System design
```
mock | rosbag2 | read-only feedback observer (TCP 30004)
        → /joint_states_robot → validating adapter → /joint_states
        → robot_state_publisher → /tf → RViz + HUD monitor
        └→ FK cross-check: URDF FK vs controller-reported TCP (per packet)
```
- **Same downstream for simulation, replay and live**, so offline tests exercise the live path.
- **Read-only at the network layer**, not just "don't call MoveIt". The vendor's ROS 2 driver connects to the command port (29999) on start-up and exposes about 130 unauthenticated motion/IO services. I wrote a feedback-only observer instead: a port allowlist (30004/5/6), no send API, a static scan of the source for socket writes, and a mock controller that counts client bytes (always 0).
- **Fail visibly.** The adapter rejects malformed, unit-suspect, out-of-limit or jumping samples. When it rejects or the feed is lost, the display freezes and the HUD turns red with the reason. It never shows fabricated zero poses (the vendor relay does).

## Implementation highlights
- The 1440-byte packet decoder resynchronizes on `MessageSize` + `TestValue` (the vendor reader permanently misaligns after one short read). Field offsets are unit-tested against the vendor C++ header and cross-checked against the vendor Python SDK dtype: 37 fields, 0 mismatches.
- A per-joint distinct-frequency mock lets an automated check *identify* every joint by spectral fit, so a swapped joint mapping fails CI.
- The model was vendored by script from a pinned upstream commit (MIT), with a provenance header and a datasheet joint-limit overlay.

## Experiments
1. Offline: 84 unit tests; a 4-stage integration test (simulation, replay, fault injection, live path against a mock controller).
2. On site (read-only, operator hand-guiding the arm in drag mode): 41 k feedback packets, FK cross-check against two candidate URDFs.

## Results
| Question | Answer (evidence) |
|---|---|
| Which model is a CR10A? | The official **`cr10`** geometry. URDF FK vs controller TCP: **RMS 1.6–3.0 mm, max 5.5 mm** over 41 k samples with all six joints moving (J1 124°, J5 155°, J6 90°). The `cr10af` model is off by a constant **43 mm** (different flange definition). |
| Orientation conventions | `rx,ry,rz` = extrinsic XYZ (same as URDF rpy); quaternion = (w,x,y,z). Mean error 0.42° vs ≥ 30° for the alternatives. |
| Frames | Link6 ≡ controller flange (tool 0); base_link ≡ user 0 |
| Streaming | 30004 and 30005 stream on this unit; 30006 does not. The controller clock runs 267.9 s behind the host (jitter σ 6.9 ms). |

The residual changes with pose (1.6 mm → 4.4 mm), so it comes from geometric parameters: theoretical vs factory-calibrated DH. It is not a mounting offset.

## Failure analysis
- **Half-closing the TCP socket** (a kernel-level "can't write" guarantee) made the real controller drop the connection immediately. This was flagged in advance as unverified, confirmed on site, and the default was changed. Three independent read-only layers remain.
- **14 % of packets were silently dropped**: the observer published only the newest packet per `recv()`, and NAT delivers packets in bursts. Controller timestamps exposed the problem (2140 gaps > 12 ms). After the fix: 4 gaps.
- **Cross-topic pairing bias.** The first offline identifier paired joint and TCP topics by arrival order, which put about 0.26 mm of fake error into a perfect mock. Fix: put the joints and TCP from the same packet into one message.
- **Unit check blind spot.** A magnitude test cannot catch degrees mistaken for radians when the value is below 2π. Only the FK cross-check catches it.
- **Human-in-the-loop timing.** One recording captured a stationary arm. The always-on per-packet cross-check log saved the session.

## Lessons learned
1. Make "read-only" a property of the transport, then test it with a byte-counting peer.
2. A convention-free metric (TCP position error) settles model identity faster than any document.
3. Log every packet. Down-sampling belongs in visualization, not in data capture.
4. Write down unverified assumptions *before* going on site. Each one became a 30-second check.

![live](../assets/m1_live/cr10a_live_rviz_sync.png)

*Real-robot data published with the owner's consent: per-packet cross-check logs in `docs/evidence/m1_live/` (gzip CSV: controller timestamp, q1..q6 [deg], URDF FK xyz, controller TCP xyz, error [mm]). Raw rosbags (~30 MB) are not in git.*
