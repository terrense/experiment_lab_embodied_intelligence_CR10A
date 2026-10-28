#!/usr/bin/env python3
"""Synthetic excitation data with KNOWN dynamics, to validate evaluation/dynamics_id.py (SIMULATED).

Replays the joint trajectory of a real recording (default: live run02) on the M4 MuJoCo plant
(URDF rigid body + DynamicsAssumptions armature/damping/frictionloss + optional flange payload),
tracked by a stiff PD + gravity/Coriolis servo at 1 kHz. Writes the SAME CSV layout as
ex_dobot_motion (8 ms samples, q deg, qd deg/s, m = applied joint torque + noise) and truth.json.

    python3 evaluation/dynamics_synthetic.py --out data/reports/dynamics/synthetic01 --payload-kg 0.4
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_mujoco_sim"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_dobot_motion"))
sys.path.insert(0, str(ROOT / "evaluation"))
from dynamics_id import expanded_urdf, load  # noqa: E402
from ex_dobot_motion.runner import COLUMNS  # noqa: E402
from ex_mujoco_sim.model_builder import JOINTS, DynamicsAssumptions, build_mjcf  # noqa: E402
from ex_mujoco_sim.servo import gains_from_inertia  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", type=Path, default=ROOT / "docs/evidence/live/excitation_run02")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--payload-kg", type=float, default=0.0)
    ap.add_argument("--tau-noise", type=float, default=0.3, help="N m, white")
    ap.add_argument("--bandwidth-hz", type=float, default=16.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--mass-scale", type=float, default=1.0, help="scale ALL link masses+inertias of the true plant")
    for k in ("armature", "damping", "frictionloss"):
        ap.add_argument(f"--{k}", type=float, nargs=6, default=None, help=f"override DynamicsAssumptions.{k} (J1..J6)")
    a = ap.parse_args(argv)

    h, D = load(a.source)
    c = {n: i for i, n in enumerate(h)}
    Q = D[:, [c[f"q{k}_deg"] for k in range(1, 7)]]
    good = (np.abs(Q).sum(1) > 1e-6) & (D[:, c["mode"]] == 7)
    t_src = (D[good, c["ctrl_ms"]] - D[good, c["ctrl_ms"]][0]) / 1000.0
    keep = np.concatenate([[True], np.diff(t_src) > 0])
    t_src, q_src = t_src[keep], np.radians(Q[good][keep])

    urdf = expanded_urdf()
    desc = ROOT / "ros2_ws" / "src" / "ex_description"
    dyn = DynamicsAssumptions(**{k: list(v) for k in ("armature", "damping", "frictionloss") if (v := getattr(a, k)) is not None})
    m = mujoco.MjModel.from_xml_string(build_mjcf(urdf, lambda u: desc / u.split("package://ex_description/", 1)[1], dyn))
    if a.mass_scale != 1.0:  # COM unchanged -> every inertial parameter scales exactly
        m.body_mass[:] *= a.mass_scale
        m.body_inertia[:] *= a.mass_scale
    fl = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "flange")
    if a.payload_kg > 0:
        m.body_mass[fl] += a.payload_kg
        m.body_inertia[fl] += 1e-4 * a.payload_kg
    d = mujoco.MjData(m)
    qadr = np.array([m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS])
    dadr = np.array([m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS])

    dt = m.opt.timestep
    tg = np.arange(0.0, t_src[-1], dt)
    # smooth reference: interpolate, then zero-phase low-pass (MovJ blending is smooth too)
    from scipy.signal import butter, filtfilt
    b_, a_ = butter(4, 3.0 / (0.5 / dt))
    q_ref = filtfilt(b_, a_, np.stack([np.interp(tg, t_src, q_src[:, j]) for j in range(6)], 1), axis=0)
    qd_ref = np.gradient(q_ref, dt, axis=0)

    d.qpos[qadr] = q_ref[0]
    mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, M, d.qM)
    g = gains_from_inertia(np.diag(M)[dadr], a.bandwidth_hz, 0.9)
    rng = np.random.default_rng(a.seed)
    every = int(round(0.008 / dt))
    rows = []
    for k in range(len(tg)):
        q, qd = d.qpos[qadr].copy(), d.qvel[dadr].copy()
        tau = g.kp * (q_ref[k] - q) + g.kd * (qd_ref[k] - qd) + d.qfrc_bias[dadr]
        tau = np.clip(tau, -np.array(dyn.torque_limit), np.array(dyn.torque_limit))
        d.ctrl[:] = tau
        if k % every == 0:
            row = dict.fromkeys(COLUMNS, 0.0)
            row.update({"host_time": tg[k], "ctrl_ms": round(tg[k] * 1000.0), "mode": 7, "cmd_id": 1})
            for j in range(6):
                row[f"q{j + 1}_deg"] = math.degrees(q[j])
                row[f"qd{j + 1}"] = math.degrees(qd[j])
                row[f"m{j + 1}"] = tau[j] + rng.normal(0, a.tau_noise)
                row[f"qt{j + 1}_deg"] = math.degrees(q_ref[k, j])
            rows.append([row[n] for n in COLUMNS])
        mujoco.mj_step(m, d)

    a.out.mkdir(parents=True, exist_ok=True)
    with gzip.open(a.out / "feedback.csv.gz", "wt") as f:
        f.write(",".join(COLUMNS) + "\n")
        for r in rows:
            f.write(",".join(repr(float(v)) for v in r) + "\n")
    truth = {"SIMULATED": True, "source_trajectory": str(a.source), "payload_kg_at_flange": a.payload_kg, "mass_scale": a.mass_scale,
             "armature": dyn.armature, "viscous": dyn.damping, "coulomb": dyn.frictionloss, "offset": [0.0] * 6,
             "tau_noise_Nm": a.tau_noise, "servo_bandwidth_hz": a.bandwidth_hz, "samples": len(rows)}
    (a.out / "truth.json").write_text(json.dumps(truth, indent=1))
    print(f"wrote {len(rows)} samples ({tg[-1]:.1f} s) -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
