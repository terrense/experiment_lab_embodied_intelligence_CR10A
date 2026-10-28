#!/usr/bin/env python3
"""Sim-to-real torque check: replay a REAL excitation trajectory on MuJoCo plant variants and
compare the simulated joint torque with the torque the real controller reported.

    python3 evaluation/sim2real_torque.py --out data/reports/dynamics/sim2real

Plants (all + 0.4 kg flange payload, replaying docs/evidence/live/excitation_run02):
  P0  default M4 assumptions (URDF masses, placeholder friction)        - what M4 used so far
  P1  URDF masses x k (identified), placeholder friction
  P2  URDF masses x k + identified Coulomb friction (M1, docs/DYNAMICS_ID.md)
No parameter is fitted here: k and the Coulomb values come from evaluation/dynamics_id.py, which fitted
them on the first 50 % of the SAME run -> only the last-25 % segment is an out-of-sample comparison.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "evaluation"))
import dynamics_id as di  # noqa: E402
import dynamics_synthetic as ds  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--real", type=Path, default=ROOT / "docs/evidence/live/excitation_run02")
    ap.add_argument("--id-report", type=Path, default=ROOT / "docs/evidence/dynamics/run02_id.json")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--plot", type=Path, default=None)
    a = ap.parse_args(argv)
    idr = json.loads(a.id_report.read_text())
    k = idr["M1s_rigid_scale_k"]
    coul = idr["M1_joint_terms"]["coulomb"]
    plants = {"P0_default": [], "P1_mass_x_k": ["--mass-scale", str(k)],
              "P2_mass_x_k_plus_coulomb": ["--mass-scale", str(k), "--frictionloss", *map(str, coul)]}
    h, D = di.load(a.real)
    real = di.preprocess(h, D, 4.0)
    rep = {"real": str(a.real), "k": k, "coulomb": coul, "tau_std_real": real["tau"].std(0).round(3).tolist()}
    sims = {}
    for name, extra in plants.items():
        out = a.out / name
        ds.main(["--source", str(a.real), "--out", str(out), "--payload-kg", "0.4", "--tau-noise", "0", *extra])
        hs, Ds = di.load(out)
        s = di.preprocess(hs, Ds, 4.0)
        n = min(len(s["tau"]), len(real["tau"]))
        e = s["tau"][:n] - real["tau"][:n]
        sims[name] = s["tau"][:n]
        te = e[int(0.75 * n):]  # last 25 %: not used to fit k / Coulomb in dynamics_id.py
        rep[name] = {"rms_vs_real": np.sqrt(np.mean(e ** 2, 0)).round(3).tolist(),
                     "rms_vs_real_test_segment": np.sqrt(np.mean(te ** 2, 0)).round(3).tolist(),
                     "mean_error": e.mean(0).round(3).tolist(),
                     "vaf_pct": (100 * (1 - np.var(e, 0) / np.var(real["tau"][:n], 0))).round(1).tolist()}
        print(f"{name:26s} RMS all {rep[name]['rms_vs_real']}  test-segment {rep[name]['rms_vs_real_test_segment']}  VAF% {rep[name]['vaf_pct']}")
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "sim2real.json").write_text(json.dumps(rep, indent=1))
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        n = min(len(v) for v in sims.values())
        t = real["t"][:n]
        for r, j in enumerate((0, 1, 2)):
            ax[r].plot(t, real["tau"][:n, j], color="#222222", lw=1.4, label="real CR10A")
            for (name, colr) in zip(sims, ("#9a9a9a", "#2a7bd4", "#d4562a")):
                ax[r].plot(t, sims[name][:, j], color=colr, lw=1.0, label=name)
            ax[r].set_ylabel(f"J{j + 1} [N m]")
        ax[0].legend(ncol=4, fontsize=8, loc="upper right")
        ax[0].set_title("Same trajectory: real joint torque vs MuJoCo plant variants (no fitting in this plot)")
        ax[-1].set_xlabel("time [s]")
        fig.tight_layout()
        a.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(a.plot, dpi=110)
    return 0


if __name__ == "__main__":
    sys.exit(main())
