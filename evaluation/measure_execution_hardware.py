"""The command channel of the REAL CR10A: the offline analysis (SIMULATION ONLY, until a capture
of the real arm exists).

What this does
--------------
The paper's premise is a command channel: the command is integrated into a position reference and
tracked by a delayed, bandwidth-limited servo, so the arm keeps moving after the request stops.
On the simulator that channel was measured (`evaluation/measure_execution.py`): 14.9 mrad of coast
at a 1 rad/s command, and an identified second-order model with omega = 37.7 rad/s,
zeta = 0.90, v_gain = 0.80.

**That measurement has only ever been made on the simulator.** This script makes the same
measurement on a capture of the REAL arm (`ros2_ws/src/ex_dobot_motion/ex_dobot_motion/channel.py`,
which records `q_target`, `q_actual` and `qd_actual` from the read-only feedback at ~125 Hz), and
writes the SAME JSON schema, so that the simulator's channel and the real one become two numbers
in the same table instead of an assumption.

The three quantities
--------------------
    the step response      how the actual position follows the commanded target: the delay
                           (the tracking lag), the overshoot and the settling. It is fitted with
                           the SAME second-order model the simulator's identification uses, so
                           the two sets of parameters are comparable by construction.
    the residual motion    the hardware counterpart of the simulator's COAST: when the commanded
                           target stops changing, how far the arm still travels, how fast it was
                           moving at that instant, and how long it takes to settle.
    the agreement          how far the two channels differ - the quantity the safety margin has
                           to absorb, and the one a reader of the paper will ask for.

Honesty
-------
* **Nothing here has run against the real arm.** Without a capture file the script says so and
  exits; it never invents a number. The simulator's channel is measured and committed, and the
  real one is UNVERIFIED until an on-site session provides the capture.
* The analysis is a pure function of the capture, so it can be re-run on the data as many times as
  needed without the robot - and it is tested on a SYNTHETIC capture, so the analysis code is
  verified before it ever sees the real arm.

    cd learning && ~/ex_rlenv/bin/python ../evaluation/measure_execution_hardware.py \\
        --capture data/live/channel_YYYYMMDD_HHMMSS/channel.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))
sys.path.insert(0, str(ROOT / "evaluation"))


# --------------------------------------------------------------------------------- reading
def read_capture(path: Path) -> Dict:
    """Read the CSV the on-site capture writes."""
    with Path(path).open(newline="") as f:
        rows = list(csv.reader(f))
    header = rows[0]
    data = np.array([[float(v) for v in r] for r in rows[1:]], float)
    out = {"header": header, "data": data}
    # the capture's columns (the same as the excitation runner's): q{i}_deg is the MEASURED
    # position, qt{i}_deg the COMMANDED target, qd{i} the measured velocity
    for i in range(1, 7):
        for key in (f"q{i}_deg", f"qt{i}_deg", f"qd{i}"):
            if key in header:
                out[key] = data[:, header.index(key)]
    return out


# -------------------------------------------------------------------------------- fitting
def fit_step_response(t: np.ndarray, q: np.ndarray, qd: np.ndarray, q_target: np.ndarray,
                      omega_grid: Optional[Sequence[float]] = None,
                      zeta_grid: Optional[Sequence[float]] = None,
                      v_gain_grid: Optional[Sequence[float]] = None) -> Dict:
    """Fit the second-order model to the step response.

    The model is the one the simulator's identification uses:

        qdd = omega^2 (q_target - q) + 2 zeta omega (v_gain * v_ref - qd)

    and the score is the RMS error of the PREDICTED position over the whole response, computed
    by integrating the model with the same time step as the data. Fitting the same model on both
    channels is what makes the two comparable.
    """
    from ex_learning.execution_model import ServoExecutionModel

    # The SAME grids the simulator's identification searches
    # (`ex_learning.execution_model.identify_execution_model`). Two channels identified with the
    # same search are comparable; two identified with different searches are not, and a coarse
    # grid would silently report a parameter that is simply the nearest grid point.
    omega_grid = list(omega_grid or [0.5 * 2 * math.pi * 8, 0.75 * 2 * math.pi * 8, 2 * math.pi * 8,
                                     1.25 * 2 * math.pi * 8, 1.5 * 2 * math.pi * 8])
    zeta_grid = list(zeta_grid or [0.7, 0.8, 0.9, 1.0, 1.1])
    v_gain_grid = list(v_gain_grid or [0.8, 0.9, 0.95, 1.0, 1.05])

    dt = float(np.median(np.diff(t)))
    best = (None, float("inf"))
    table = []
    for om in omega_grid:
        for ze in zeta_grid:
            for vg in v_gain_grid:
                m = ServoExecutionModel(omega=om, zeta=ze, v_gain=vg, dt=dt, sub=1)
                err = _simulate_error(m, t, q, qd, q_target)
                table.append({"omega": om, "zeta": ze, "v_gain": vg, "rms_rad": err})
                if err < best[1]:
                    best = ((om, ze, vg), err)
    (om, ze, vg), err = best
    return {"model": {"omega": om, "zeta": ze, "v_gain": vg, "dt": dt, "sub": 1},
            "prediction_error": {"rms_rad": err, "horizon_steps": len(t),
                                 "horizon_s": float(t[-1] - t[0])},
            "grid": table}


def _simulate_error(m, t, q, qd, q_target) -> float:
    """The RMS error of the model's prediction over the recorded response."""
    dt = m.dt
    qq, vv = float(q[0]), float(qd[0])
    pred = []
    for i in range(1, len(t)):
        # the command rate is what the plant sees: the target changes at the recorded instants
        v_ref = (q_target[i] - q_target[i - 1]) / dt
        qdd = m.omega ** 2 * (q_target[i - 1] - qq) + 2 * m.zeta * m.omega * (m.v_gain * v_ref - vv)
        vv += qdd * dt
        qq += vv * dt
        pred.append(qq)
    pred = np.asarray(pred)
    return float(np.sqrt(((pred - q[1:]) ** 2).mean()))


# ------------------------------------------------------------------------- residual motion
def tracking_lag(t: np.ndarray, qd: np.ndarray, q_target: np.ndarray,
                 max_lag_s: float = 0.2) -> Dict:
    """The delay between the commanded rate and the measured velocity.

    This is measured DIRECTLY, by cross-correlating the commanded target's rate with the
    measured joint velocity. It needs no model, so it does not inherit the model's
    identifiability problem: a slow ramp can hide the servo parameters, but it cannot hide the
    delay between "the controller asked for motion" and "the arm moved".
    """
    dt = float(np.median(np.diff(t)))
    cmd = np.gradient(q_target, dt)
    meas = qd - qd.mean()
    cmd_c = cmd - cmd.mean()
    n = len(cmd)
    max_lag = int(max_lag_s / dt)
    if n < 4 * max_lag:
        return {"error": "the capture is too short for a lag measurement"}
    best_lag, best_corr = 0, -2.0
    for lag in range(0, max_lag):
        a = cmd_c[lag:]
        b = meas[:n - lag]
        if len(a) < 8:
            break
        den = float(np.linalg.norm(a) * np.linalg.norm(b))
        if den < 1e-12:
            continue
        corr = float(a @ b / den)
        if corr > best_corr:
            best_corr, best_lag = corr, lag

    # The ONSET delay: the time between the command starting and the motion starting. It is a
    # LOWER BOUND on the transport lag and it needs no model and no correlation to be meaningful,
    # which is why it is reported next to the correlation estimate.
    onset = None
    cmd_peak = float(np.max(np.abs(cmd)))
    if cmd_peak > 1e-9:
        i_cmd = int(np.argmax(np.abs(cmd) > 0.05 * cmd_peak))
        v_peak = float(np.max(np.abs(qd)))
        if v_peak > 1e-9:
            after = np.abs(qd[i_cmd:]) > 0.10 * v_peak
            if after.any():
                onset = float(t[i_cmd + int(np.argmax(after))] - t[i_cmd])
    return {"lag_s": best_lag * dt, "lag_samples": best_lag,
            "correlation": best_corr,
            "response_delay_s": onset,
            "note": "the cross-correlation estimate of the delay between the commanded target's "
                    "rate and the measured joint velocity, and the ONSET delay (a model-free "
                    "lower bound on it). The simulator's model has a velocity-tracking lag of "
                    "2*zeta/omega = 48 ms."}


def residual_motion(t: np.ndarray, q: np.ndarray, qd: np.ndarray, q_target: np.ndarray,
                    settle_tol_rad: float = 0.002, settle_tol_speed: float = 0.01) -> Dict:
    """The hardware counterpart of the simulator's coast.

    The commanded target stops changing at some instant; from that instant the arm is still
    moving. This measures how far it still travels, how fast it was moving at that instant, and
    how long it takes to settle.
    """
    changed = np.where(np.abs(np.diff(q_target)) > 1e-6)[0]
    if len(changed) == 0:
        return {"error": "the commanded target never changed - the capture is not a step response"}
    t_cut = int(changed[-1]) + 1
    after = slice(t_cut, len(t))
    q_after = q[after]
    qd_after = qd[after]
    t_after = t[after]
    if len(q_after) < 3:
        return {"error": "the capture ends before the response could be measured"}
    travelled = float(np.max(np.abs(q_after - q_after[0])))
    # the speed at the cut: the plant may still be accelerating, so the peak over the first
    # 20 ms is the honest number
    window = max(1, int(0.02 / max(float(np.median(np.diff(t))), 1e-6)))
    speed_at_cut = float(np.max(np.abs(qd_after[:window])))
    # the settling tolerance is relative to that speed: a fixed 0.01 rad/s would be satisfied
    # immediately on a slow response, and would report "settled at 0.000 s"
    tol = min(settle_tol_speed, max(0.1 * speed_at_cut, 1e-6))
    below = np.abs(qd_after) < tol
    # the arm is settled when it has STAYED below the tolerance for 5 consecutive samples
    settle_s = None
    run = 0
    for i, b in enumerate(below):
        run = run + 1 if b else 0
        if run >= 5:
            settle_s = float(t_after[i - 4] - t_after[0])
            break
    return {
        "t_cut_s": float(t[t_cut]),
        "speed_at_cut_rad_s": speed_at_cut,
        "residual_travel_rad": travelled,
        "residual_travel_mrad": 1000 * travelled,
        "settle_s": settle_s,
        "settle_tol_speed_rad_s": tol,
        "note": "the distance the arm still travels after the commanded target stopped "
                "changing - the hardware counterpart of the simulator's coast",
    }


# ------------------------------------------------------------------------------ comparison
def compare_with_simulator(hardware: Dict, simulator_path: Path) -> Dict:
    """The simulator's channel against the real one, in the same units."""
    p = Path(simulator_path)
    if not p.exists():
        return {"error": f"{p} is not there"}
    sim = json.loads(p.read_text())
    out = {"simulator": {}, "hardware": {}}
    sm = sim["identification"]["model"]
    out["simulator"] = {"omega": sm["omega"], "zeta": sm["zeta"], "v_gain": sm["v_gain"],
                         "prediction_rms_rad": sim["identification"]["prediction_error"]["rms_rad"]}
    if "identification" in hardware:
        hm = hardware["identification"]["model"]
        out["hardware"] = {"omega": hm["omega"], "zeta": hm["zeta"], "v_gain": hm["v_gain"],
                            "prediction_rms_rad": hardware["identification"]["prediction_error"]["rms_rad"]}
        out["ratio_omega"] = hm["omega"] / sm["omega"] if sm["omega"] else None
        out["ratio_zeta"] = hm["zeta"] / sm["zeta"] if sm["zeta"] else None
    # the simulator's coast is per commanded velocity; the comparison uses the fastest one
    if "coast" in hardware and sim.get("rows"):
        fastest = max(sim["rows"], key=lambda r: r["command_rad_s"])
        out["simulator_coast_rad"] = fastest["coast_rad"]
        out["simulator_coast_command_rad_s"] = fastest["command_rad_s"]
        out["hardware_coast_rad"] = hardware["coast"]["residual_travel_rad"]
        out["coast_ratio"] = (hardware["coast"]["residual_travel_rad"] / fastest["coast_rad"]
                              if fastest["coast_rad"] else None)
    out["note"] = ("the ratio of the real channel's parameters to the simulator's is the quantity "
                   "the safety margin has to absorb; a ratio near 1 means the simulator's channel "
                   "model is a good description of the real arm")
    return out


def analyse(capture_path: Path, joint: int = 5, simulator: Optional[Path] = None) -> Dict:
    """The whole analysis of one capture."""
    cap = read_capture(capture_path)
    data = cap["data"]
    header = cap["header"]
    t = data[:, header.index("host_time")]
    t = t - t[0]
    q = cap[f"q{joint}_deg"]
    qd = cap[f"qd{joint}"]
    qt = cap[f"qt{joint}_deg"]
    q = np.radians(q)
    qd = np.radians(qd)
    qt = np.radians(qt)
    fit = fit_step_response(t, q, qd, qt)
    coast = residual_motion(t, q, qd, qt)
    lag = tracking_lag(t, qd, qt)
    out = {"capture": str(capture_path), "joint": joint, "packets": len(t),
           "duration_s": float(t[-1]), "identification": fit, "coast": coast,
           "tracking_lag": lag}
    if simulator is not None:
        out["comparison"] = compare_with_simulator(out, simulator)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", type=Path, default=None,
                    help="the CSV the on-site capture wrote")
    ap.add_argument("--joint", type=int, default=5)
    ap.add_argument("--simulator", type=Path,
                    default=ROOT / "data" / "reports" / "learning" / "execution_channel.json")
    ap.add_argument("--out", type=Path,
                    default=ROOT / "data" / "reports" / "learning" / "execution_channel_hardware.json")
    a = ap.parse_args(argv)

    if a.capture is None or not a.capture.exists():
        print("NO HARDWARE CAPTURE.\n"
              "This script analyses a capture of the REAL arm, taken on site with\n"
              "  python3 -m ex_dobot_motion.channel --robot-ip <ip> --execute "
              "--confirm I_AM_AT_THE_ROBOT_WITH_ESTOP\n"
              "The simulator's channel is measured and committed "
              "(data/reports/learning/execution_channel.json); the real one is UNVERIFIED until\n"
              "a capture exists, and no number is invented for it.")
        return 1

    rep = analyse(a.capture, a.joint, a.simulator)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(rep, indent=1))
    print(json.dumps({k: v for k, v in rep.items() if k != "identification"}, indent=1)[:1500])
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
