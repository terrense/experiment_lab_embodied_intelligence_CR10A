# The on-site session: measuring the real arm's command channel

> **This is the paper's missing hardware piece.** The whole "execution-aware" argument rests on a
> claim about the command channel — that the command is integrated into a position reference
> and tracked by a delayed, bandwidth-limited servo, so the arm keeps moving after the request
> stops. That channel has only ever been measured **on the simulator** (14.9 mrad of coast at a
> 1 rad/s command; an identified second-order model, ω = 37.7 rad/s, ζ = 0.90, v_gain = 0.80).
>
> The measurement below makes the same measurement on the real arm. It is the one experiment that
> would turn "the simulator's channel behaves like X" into "the real arm's channel behaves like
> Y, and here is the gap the safety margin has to absorb".
>
> **Nothing here has been run against the robot.** The code is written and tested; the capture
> needs an operator at the machine with the e-stop in reach.

## Before you start

| requirement | why |
|---|---|
| the owner is **at the robot**, e-stop in reach | the only thing that makes a motion test acceptable |
| the arm is in a **clear, open pose**, nothing within reach | the step is on the wrist, but the arm must not be able to reach anything |
| the pendant shows **no error and no collision** | the script refuses to start otherwise |
| the workspace is free of people and obstacles | the e-stop is the last line of defence, not the first |
| `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1` | the same environment as every other session |

**The hard rules this procedure inherits** (they are not optional):

* the **dry run is the default**. `--execute` is required to send anything, and it also
  requires `--confirm I_AM_AT_THE_ROBOT_WITH_ESTOP`;
* the **command whitelist** in `ex_dobot_motion/dashboard.py` admits `MovJ` in joint form only,
  with `v` and `a` inside the caps (30 %). Nothing else can be sent;
* the **safety envelope** is checked before the first command goes out and on **every feedback
  packet** afterwards: joint excursion, TCP height, TCP excursion. A violation aborts with
  `Stop()` + `DisableRobot()`;
* the **caps are not widened** for this experiment. If the step has to be bigger to be
  measurable, that is a decision to make explicitly, not a default to change.

## The procedure

### 1. The dry run (no command is sent)

```bash
cd ros2_ws/src/ex_dobot_motion
python3 -m ex_dobot_motion.channel --robot-ip <ROBOT_IP>
```

It connects to the read-only feedback (port 30004), prints the current pose and the robot mode,
builds the step plan, **validates it against the safety envelope**, and writes the plan to
`data/live/channel_<timestamp>/plan.json`. Read that file before going on.

Check in the plan:
* `problems` is empty — the step is inside the envelope;
* the step is on the **wrist** (J5 by default) — it barely moves the TCP;
* `speed`, `acc` and `speed_factor` are at or below the caps (15 % by default).

### 2. The execution (only at the robot, e-stop in reach)

```bash
python3 -m ex_dobot_motion.channel --robot-ip <ROBOT_IP> \
    --execute --confirm I_AM_AT_THE_ROBOT_WITH_ESTOP
```

What it does, in order:

1. reads the current pose and refuses to continue on a controller error, a collision flag or an
   unexpected robot mode;
2. `RequestControl()`, `EnableRobot(payload)`, `SpeedFactor`, `AccJ`;
3. sends **two** `MovJ` commands: the wrist steps out by `--step-deg` and comes back;
4. keeps recording the feedback for `--settle-s` after the last command, which is where the
   **residual motion** is measured;
5. checks the safety envelope on every packet and aborts with `Stop()` + `DisableRobot()` on any
   violation;
6. writes `data/live/channel_<timestamp>/channel.csv` (the raw feedback, with the commanded
   target `qt{i}_deg`, the measured position `q{i}_deg` and the measured velocity `qd{i}`) and
   `capture.json` (the metadata).

### 3. The analysis (no robot needed)

```bash
cd learning && ~/ex_rlenv/bin/python ../evaluation/measure_execution_hardware.py \
    --capture ../data/live/channel_<timestamp>/channel.csv
```

It writes `data/reports/learning/execution_channel_hardware.json` in **the same schema as the
simulator's** `execution_channel.json`, plus a `comparison` block with the ratios — the real
channel's parameters over the simulator's. Those ratios are the number the paper needs: they say
how much of the simulator's margin is real.

## What the excitation has to be

**This is the one thing that can waste the session.** The servo settles in `2ζ/ω ≈ 48 ms`, so:

* the **step must be fast relative to the servo**. The controller's own interpolation is slow
  (a 4° wrist move at 15 % takes hundreds of ms), and a slow ramp lets many parameter sets
  reproduce the response equally well — the fit then reports the nearest grid point rather than
  the model. Use the **highest permitted speed** and the **smallest step that is still above the
  feedback noise** (0.5–1°): the interpolation is then short compared with the servo;
* the **step must be big enough to measure** — the feedback resolution is 0.01°, so a 0.1° step
  is at the noise floor and the coast would be indistinguishable from zero;
* the trade-off is narrow (0.5–1° at 30 %), and the script's default (4° at 15 %) is the
  conservative end. **If the fit lands on a grid edge, the excitation was too slow** — re-run
  with `--speed 30 --step-deg 1.0` rather than trusting the fit.

## What the numbers mean

| the measurement | the simulator's | what it is |
|---|---|---|
| the coast | 14.9 mrad at a 1 rad/s command | how far the arm still travels after the command stops changing. The hardware's counterpart is the **residual motion** |
| the tracking lag | 2ζ/ω = 48 ms | how long the arm takes to answer a change of target |
| the identified model | ω = 37.7 rad/s, ζ = 0.90, v_gain = 0.80 | the second-order model the simulator's channel is described by, fitted to the real response with **the same grid** |

The comparison the paper wants is the ratio of each. A ratio near 1 means the simulator's channel
model describes the real arm and the simulation results transfer; a ratio far from 1 says how
much the hardware margin has to be inflated, and the paper says so instead of pretending.

## If something goes wrong

* the script aborts by itself on a controller error, a collision flag, an unexpected mode, a
  feedback timeout or an envelope violation — it prints `[exec] ABORT: ...` and sends
  `Stop()` + `DisableRobot()`;
* `Ctrl-C` sends `Stop()` and `DisableRobot()` as well;
* if the arm is left enabled, `DisableRobot()` from the pendant, and tell the owner — the
  session's data is not worth a stuck robot;
* **the e-stop is always the first line of defence.** The script's checks are the second.

## What is NOT done in this session

* **no grasp, no contact, no payload change** — the session measures the channel, nothing else;
* **no widening of the whitelist or the caps**;
* the arm is **not** left enabled, and the recorded pose is **not** moved to a different one.
