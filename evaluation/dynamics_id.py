#!/usr/bin/env python3
"""Joint-torque dynamics identification from an ex_dobot_motion excitation recording (offline).

    python3 evaluation/dynamics_id.py docs/evidence/live/excitation_run02 --tool-kg 0.4 \
        --out data/reports/dynamics/run02.json

Pipeline
  1. clean: drop malformed packets (q == 0 / error / collision flags), keep RUNNING rows (mode 7)
  2. resample on the CONTROLLER clock (ctrl_ms) to a uniform 8 ms grid
  3. zero-phase Butterworth low-pass (same filter on q, qd, tau so no relative lag), qdd = d(qd)/dt
  4. split by time: train / validation / test = first 50 % / next 25 % / last 25 %
  5. models, all evaluated on the TEST part (torque RMS per joint, % variance explained):
       M0  URDF rigid body (+ tool point mass), no friction            - zero fitted parameters
       M1  M0 + fitted per-joint armature, viscous, Coulomb, offset     - 24 parameters
       M1s k * M0 + armature, viscous, Coulomb (no offsets)             - 19 parameters (k: mass-scale test)
       M2  full base parameters, ridge-regularised toward the M1 prior  - lambda picked on VALIDATION
Units: q in the CSV is deg, qd deg/s (scale checked against dq/dt), tau = controller "m" field
(assumed N m; the assumption is reported, not verified).
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_robot_state"))
from ex_robot_state.core.dynamics import ChainDynamics, add_point_mass, base_columns, random_states  # noqa: E402

DT = 0.008
EXTRAS = ("armature", "viscous", "coulomb", "offset")


def expanded_urdf(calibrated: bool = True) -> str:
    import os
    import tempfile
    import xacro
    desc = ROOT / "ros2_ws" / "src" / "ex_description"
    text = (desc / "urdf" / "ex_cr10.urdf.xacro").read_text().replace("$(find ex_description)", str(desc))
    with tempfile.NamedTemporaryFile("w", suffix=".xacro", delete=False) as f:
        f.write(text)
    try:
        return xacro.process_file(f.name, mappings={"calibrated": "true" if calibrated else "false"}).toxml()
    finally:
        os.unlink(f.name)


def load(run: Path) -> Tuple[List[str], np.ndarray]:
    rows = list(csv.reader(gzip.open(run / "feedback.csv.gz", "rt")))
    return rows[0], np.array(rows[1:], float)


def preprocess(h: List[str], D: np.ndarray, fc_hz: float, trim_s: float = 0.5) -> Dict[str, np.ndarray]:
    from scipy.signal import butter, filtfilt
    c = {n: i for i, n in enumerate(h)}
    Q = D[:, [c[f"q{k}_deg"] for k in range(1, 7)]]
    good = (np.abs(Q).sum(1) > 1e-6) & (D[:, c["error"]] == 0) & (D[:, c["collision"]] == 0) & (D[:, c["mode"]] == 7)
    D = D[good]
    t = (D[:, c["ctrl_ms"]] - D[0, c["ctrl_ms"]]) / 1000.0
    keep = np.concatenate([[True], np.diff(t) > 0])  # duplicate controller stamps
    D, t = D[keep], t[keep]
    tg = np.arange(0.0, t[-1], DT)
    col = lambda fmt, scale=1.0: np.stack([np.interp(tg, t, D[:, c[fmt.format(k)]]) for k in range(1, 7)], 1) * scale  # noqa: E731
    q = col("q{}_deg", math.pi / 180)
    qd_raw = col("qd{}", math.pi / 180)
    tau = col("m{}")
    b, a = butter(4, fc_hz / (0.5 / DT))
    qf, qdf, tauf = (filtfilt(b, a, x, axis=0) for x in (q, qd_raw, tau))
    qdd = np.gradient(qdf, DT, axis=0)
    # sanity: controller qd vs numerical derivative of q (unit / scale check)
    dq = np.gradient(qf, DT, axis=0)
    scale = np.array([float(np.dot(dq[:, j], qdf[:, j]) / np.dot(qdf[:, j], qdf[:, j])) for j in range(6)])
    n = int(trim_s / DT)
    sl = slice(n, len(tg) - n)
    return {"t": tg[sl], "q": qf[sl], "qd": qdf[sl], "qdd": qdd[sl], "tau": tauf[sl], "qd_scale_vs_dqdt": scale,
            "n_raw": int(good.sum()), "duration_s": float(tg[-1])}


def metrics(tau: np.ndarray, pred: np.ndarray) -> Dict[str, List[float]]:
    e = tau - pred
    rms = np.sqrt(np.mean(e ** 2, 0))
    var = np.var(tau, 0)
    return {"rms": rms.round(3).tolist(), "vaf_pct": (100 * (1 - np.var(e, 0) / np.maximum(var, 1e-12))).round(1).tolist(),
            "tau_std": np.sqrt(var).round(3).tolist()}


def identify(data: Dict[str, np.ndarray], cd: ChainDynamics, theta_rigid_prior: np.ndarray, lambdas=(1e-4, 1e-3, 1e-2, 1e-1, 1, 10, 100)):
    N = len(data["t"])
    i_tr, i_va = int(0.5 * N), int(0.75 * N)
    # Coulomb smoothing width tanh(qd / v_eps): chosen on VALIDATION with the M1 model
    best_eps = None
    for eps in (0.002, 0.005, 0.01, 0.02, 0.04, 0.08):
        Ye = cd.regressor(data["q"], data["qd"], data["qdd"], extras=EXTRAS, v_eps=eps)
        r = data["tau"] - Ye[:, :, :cd.n_rigid] @ theta_rigid_prior
        Je = Ye[:, :, cd.n_rigid:]
        ph = np.linalg.lstsq(Je[:i_tr].reshape(-1, Je.shape[2]), r[:i_tr].reshape(-1), rcond=None)[0]
        v = float(np.sqrt(np.mean((r[i_tr:i_va] - Je[i_tr:i_va] @ ph) ** 2)))
        if best_eps is None or v < best_eps[0]:
            best_eps = (v, eps, Ye)
    _, v_eps, Y = best_eps                                                     # (N, 6, P)
    tau = data["tau"]
    P_rigid = cd.n_rigid
    split = {"train": slice(0, i_tr), "val": slice(i_tr, i_va), "test": slice(i_va, N)}
    flat = lambda s: (Y[s].reshape(-1, Y.shape[2]), tau[s].reshape(-1))  # noqa: E731
    out: Dict = {"n_samples": N, "split_s": [float(data["t"][i_tr]), float(data["t"][i_va])], "coulomb_v_eps": v_eps}

    # M0: prior rigid body, no friction
    tau_rigid_prior = Y[:, :, :P_rigid] @ theta_rigid_prior
    out["M0_urdf"] = metrics(tau[split["test"]], tau_rigid_prior[split["test"]])

    # M1: prior rigid body + fitted joint terms (residual LS)
    J = Y[:, :, P_rigid:]
    Jtr, rtr = J[split["train"]].reshape(-1, J.shape[2]), (tau - tau_rigid_prior)[split["train"]].reshape(-1)
    phi = np.linalg.lstsq(Jtr, rtr, rcond=None)[0]
    pred1 = tau_rigid_prior + J @ phi
    out["M1_urdf_plus_friction"] = metrics(tau[split["test"]], pred1[split["test"]])
    out["M1_joint_terms"] = {e: phi[k * 6:(k + 1) * 6].round(4).tolist() for k, e in enumerate(EXTRAS)}

    # M1s: global scale k on the prior rigid body + armature/viscous/Coulomb, NO offsets
    # (hypothesis test "URDF masses are uniformly too light"; with offsets k is not identifiable here)
    Js = np.concatenate([tau_rigid_prior[:, :, None], J[:, :, :18]], axis=2)
    ks = np.linalg.lstsq(Js[split["train"]].reshape(-1, 19), tau[split["train"]].reshape(-1), rcond=None)[0]
    pred1s = Js @ ks
    out["M1s_scaled_urdf_plus_friction"] = metrics(tau[split["test"]], pred1s[split["test"]])
    out["M1s_rigid_scale_k"] = round(float(ks[0]), 3)

    # M2: base parameters (structural, from random states) with ridge toward the M1 prior
    rng = np.random.default_rng(0)
    qr, qdr, qddr = random_states(6, 300, rng, data["q"].min(0) - 1.0, data["q"].max(0) + 1.0)
    ind, dep = base_columns(cd.regressor(qr, qdr, qddr, extras=EXTRAS).reshape(-1, Y.shape[2]))
    prior_full = np.concatenate([theta_rigid_prior, phi])
    Ytr, ttr = flat(split["train"])
    Yb_tr = Ytr[:, ind]
    # prior expressed in base coordinates: theta_b = theta[ind] + beta theta[dep]
    depi, beta = dep
    prior_b = prior_full[ind] + beta @ prior_full[depi]
    sc = np.linalg.norm(Yb_tr, axis=0) + 1e-12
    A = Yb_tr / sc
    cond = float(np.linalg.cond(A))
    best = None
    Yb_all = Y.reshape(-1, Y.shape[2])[:, ind]
    for lam in lambdas:
        # min |A x - t|^2 + lam |x - x0|^2  with x = sc * theta_b
        x0 = prior_b * sc
        x = np.linalg.solve(A.T @ A + lam * np.eye(A.shape[1]), A.T @ ttr + lam * x0)
        thb = x / sc
        pred = (Yb_all @ thb).reshape(N, 6)
        v = float(np.sqrt(np.mean((tau[split["val"]] - pred[split["val"]]) ** 2)))
        if best is None or v < best[0]:
            best = (v, lam, thb, pred)
    _, lam, thb, pred2 = best
    out["M2_base_ridge"] = metrics(tau[split["test"]], pred2[split["test"]])
    out["M2_lambda"] = lam
    out["M2_n_base"] = int(len(ind))
    out["M2_train_condition_number_scaled"] = round(cond, 1)
    out["base_columns"] = [cd.column_names(EXTRAS)[i] for i in ind]
    out["theta_base_estimate"] = thb.tolist()
    out["theta_base_prior"] = prior_b.tolist()
    return out, {"M0": tau_rigid_prior, "M1": pred1, "M1s": pred1s, "M2": pred2}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--tool-kg", type=float, default=0.0, help="tool point mass added to Link6 in the prior")
    ap.add_argument("--tool-com", type=float, nargs=3, default=(0.0, 0.0, 0.04), help="tool COM in Link6 frame (m), prior only")
    ap.add_argument("--fc", type=float, default=4.0, help="low-pass cut-off (Hz)")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--plot", type=Path, default=None)
    a = ap.parse_args(argv)

    h, D = load(a.run)
    data = preprocess(h, D, a.fc)
    cd = ChainDynamics(expanded_urdf())
    prior = cd.theta_urdf.copy()
    if a.tool_kg:
        prior[50:60] = add_point_mass(prior[50:60], a.tool_kg, a.tool_com)
    rep, preds = identify(data, cd, prior)
    rep.update({"run": str(a.run), "fc_hz": a.fc, "tool_kg_prior": a.tool_kg, "duration_s": data["duration_s"],
                "qd_scale_vs_dqdt": data["qd_scale_vs_dqdt"].round(4).tolist(), "tau_field": "m (assumed N m)"})
    print(f"{a.run}: {rep['n_samples']} samples @ {DT * 1000:.0f} ms, fc {a.fc} Hz, qd/dqdt scale {rep['qd_scale_vs_dqdt']}")
    print(f"  M1s rigid scale k = {rep['M1s_rigid_scale_k']}   Coulomb v_eps (val) = {rep['coulomb_v_eps']} rad/s")
    for k in ("M0_urdf", "M1_urdf_plus_friction", "M1s_scaled_urdf_plus_friction", "M2_base_ridge"):
        print(f"  {k:24s} test RMS {rep[k]['rms']}  VAF% {rep[k]['vaf_pct']}")
    print(f"  tau std (test)          {rep['M0_urdf']['tau_std']}")
    print(f"  M1 joint terms {rep['M1_joint_terms']}")
    print(f"  M2: {rep['M2_n_base']} base params, lambda {rep['M2_lambda']}, cond {rep['M2_train_condition_number_scaled']}")
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps(rep, indent=1))
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
        t = data["t"]
        for j in range(6):
            ax[j].plot(t, data["tau"][:, j], color="#222222", lw=1.4, label="measured")
            for k, colr in (("M0", "#9a9a9a"), ("M1s", "#2a9d5c"), ("M1", "#2a7bd4"), ("M2", "#d4562a")):
                ax[j].plot(t, preds[k][:, j], color=colr, lw=1.0, label=k)
            ax[j].axvspan(rep["split_s"][1], t[-1], color="#eeeeee", zorder=0)
            ax[j].set_ylabel(f"J{j + 1} [N m]")
        ax[0].legend(ncol=5, fontsize=8, loc="upper right")
        ax[0].set_title(f"{a.run.name}: measured vs predicted joint torque (grey band = test)")
        ax[-1].set_xlabel("time [s]")
        fig.tight_layout()
        a.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(a.plot, dpi=110)
    return 0


if __name__ == "__main__":
    sys.exit(main())
