#!/usr/bin/env python3
"""Identify URDF corrections for the real CR10A from the on-site FK cross-check logs.

  python3 evaluation/kinematic_calibration.py [--out data/reports/calibration]
          [--write-urdf]   # also generate ex_description/urdf/dobot_cr10_calibrated.xacro

Data: docs/evidence/m1_live/crosscheck_session_*.csv.gz (per packet: q1..q6 [deg],
controller TCP xyz [mm], tool 0 / user 0). Targets are the CONTROLLER's TCP, i.e. its
factory-calibrated model - this aligns our URDF with the controller, it is not metrology.

Protocol
  * dedupe each session into 0.05 deg joint cells (static periods would dominate)
  * models (ablation): A = joint zero offsets (6) | B = A + flange offset (9) | C = full (45, regularised)
  * cross-session validation: fit on session 1 -> test on session 2, and the reverse
  * final parameters: fit model C on both sessions
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_robot_state"))
sys.path.insert(0, str(ROOT / "evaluation"))

from ex_robot_state.core.calibration import N_PER_JOINT, ChainModel, dedupe_poses, rms  # noqa: E402
from ex_robot_state.core.urdf_model import UrdfModel  # noqa: E402

SESSIONS = {
    "s1_drag": ROOT / "docs/evidence/m1_live/crosscheck_session_drag_20261026.csv.gz",
    "s2_all_joints": ROOT / "docs/evidence/m1_live/crosscheck_session_all_joints_20261026.csv.gz",
}


def load(path: Path):
    d = np.genfromtxt(gzip.open(path), delimiter=",", names=True)
    q = np.radians(np.stack([d[f"q{i}"] for i in range(1, 7)], axis=1))
    x = np.stack([d["ctrl_x"], d["ctrl_y"], d["ctrl_z"]], axis=1) / 1000.0
    return dedupe_poses(q, x, math.radians(0.05))


def expand_urdf() -> str:
    import xacro
    path = ROOT / "ros2_ws/src/ex_description/urdf/ex_cr10.urdf.xacro"
    text = path.read_text().replace("$(find ex_description)", str(ROOT / "ros2_ws/src/ex_description"))
    tmp = Path("/tmp/ex_cr10_for_calib.xacro")
    tmp.write_text(text)
    return xacro.process_file(str(tmp), mappings={"calibrated": "false"}).toxml()


def mask_for(model: str, n: int) -> np.ndarray:
    m = np.zeros(n, bool)
    if model in ("A", "B"):
        for i in range(6):
            m[N_PER_JOINT * i + 6] = True
        if model == "B":
            m[-3:] = True
    else:
        m[:] = True
    return m


def fit_masked(chain: ChainModel, q, x, mask, reg):
    """Fit only the masked parameters (others fixed at 0) with Gauss-Newton + Tikhonov."""
    p = np.zeros(chain.n_params)
    lam = reg * len(q)
    hist = []
    for _ in range(25):
        r = chain.residuals(p, q, x)
        hist.append(rms(r.reshape(-1, 3)))
        J = chain.jacobian(p, q)[:, mask]
        A = J.T @ J + lam * np.eye(mask.sum())
        step = np.linalg.solve(A, -(J.T @ r + lam * p[mask]))
        p[mask] += step
        if np.linalg.norm(step) < 1e-11:
            break
    hist.append(rms(chain.residuals(p, q, x).reshape(-1, 3)))
    return p, hist


def errors(chain, p, q, x):
    e = np.linalg.norm(chain.fk_positions(q, p) - x, axis=1) * 1000
    return {"rms_mm": float(np.sqrt((e ** 2).mean())), "p50_mm": float(np.median(e)),
            "p95_mm": float(np.percentile(e, 95)), "max_mm": float(e.max()), "n": int(len(e))}, e


def flange_rotation_offset(urdf_xml: str, bags) -> tuple:
    """Constant rotation R_off with R_ctrl ~= R_fk * R_off, estimated from bag orientation data
    (position-only data cannot see a rotation of the flange frame about its own origin).
    Chordal L2 mean of R_fk^T R_ctrl, projected back onto SO(3). Returns (rpy, stats)."""
    from crosscheck_bag import read_samples  # same directory
    from ex_robot_state.core.se3 import matrix_to_rpy, quat_to_matrix, rotation_angle_between
    m = UrdfModel.from_string(urdf_xml)
    Rs, pairs = [], []
    for bag in bags:
        samples, _ = read_samples(str(bag))
        for r in samples[::5]:
            q = {f"joint{i + 1}": math.radians(v) for i, v in enumerate(r["q_deg"])}
            R_fk = m.fk(q, "flange", "base_link")[:3, :3]
            w, x, y, z = r["quat"]  # packet order (w,x,y,z), verified on site
            R_c = quat_to_matrix(x, y, z, w)
            Rs.append(R_fk.T @ R_c)
            pairs.append((R_fk, R_c))
    M = np.mean(Rs, axis=0)
    U, _, Vt = np.linalg.svd(M)
    R_off = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    before = [math.degrees(rotation_angle_between(a, b)) for a, b in pairs]
    after = [math.degrees(rotation_angle_between(a @ R_off, b)) for a, b in pairs]
    stats = {"n": len(pairs), "rot_err_deg_mean_before": float(np.mean(before)), "rot_err_deg_mean_after": float(np.mean(after)),
             "rot_err_deg_max_after": float(np.max(after)), "offset_angle_deg": math.degrees(rotation_angle_between(np.eye(3), R_off))}
    return matrix_to_rpy(R_off), stats


def write_urdf(chain: ChainModel, p: np.ndarray, out: Path, flange_rpy=(0.0, 0.0, 0.0)) -> None:
    src = ROOT / "ros2_ws/src/ex_description/urdf/dobot_cr10_official.xacro"
    import xml.etree.ElementTree as ET
    text = src.read_text()
    head, body = text.split("<robot", 1)
    root = ET.fromstring("<robot" + body)
    ns = "{http://www.ros.org/wiki/xacro}"
    macro = root.find(f"{ns}macro")
    macro.set("name", "dobot_cr10_calibrated")
    origins = chain.calibrated_origins(p)
    for i, (xyz, rpy) in enumerate(origins):
        j = macro.find(f"joint[@name='joint{i + 1}']")
        o = j.find("origin")
        o.set("xyz", " ".join(f"{v:.7f}" for v in xyz))
        o.set("rpy", " ".join(f"{v:.8f}" for v in rpy))
    flange = chain.post[:3, 3] + chain.post[:3, :3] @ p[-3:]
    for name, val in (("cal_flange_xyz", " ".join(f"{v:.7f}" for v in flange)),
                      ("cal_flange_rpy", " ".join(f"{v:.8f}" for v in flange_rpy))):
        prop = ET.SubElement(root, f"{ns}property", {"name": name, "value": val})
        root.remove(prop)
        root.insert(0, prop)
    ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")
    xml = ET.tostring(root, encoding="unicode")
    hdr = ("<?xml version=\"1.0\"?>\n<!--\n  GENERATED by evaluation/kinematic_calibration.py (write-urdf option). Do not edit.\n"  # no double hyphen: illegal in XML comments
           "  Official cr10 macro with joint origins re-identified from on-site CR10A data (2026-10-26):\n"
           "  aligns URDF FK with the CR10A controller's own (factory-calibrated) TCP; joint zero offsets\n"
           "  are folded into the origins as rotations about each joint axis. Not an external metrology result.\n"
           "  Meshes/inertias unchanged. License of the base model: MIT (c) Dobot.\n-->\n")
    out.write_text(hdr + xml + "\n")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "data/reports/calibration"))
    ap.add_argument("--reg", type=float, default=1e-7)
    ap.add_argument("--write-urdf", action="store_true")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    model = UrdfModel.from_string(expand_urdf())
    chain = ChainModel.from_chain(model.chain("flange", "base_link"))
    data = {k: load(v) for k, v in SESSIONS.items()}
    rep = {"targets": "controller-reported TCP (tool 0, user 0) - aligns URDF to the controller model",
           "dedupe_resolution_deg": 0.05, "reg": a.reg,
           "samples": {k: len(v[0]) for k, v in data.items()}, "nominal": {}, "cross_session": {}, "final": {}}
    for k, (q, x) in data.items():
        rep["nominal"][k] = errors(chain, None, q, x)[0]

    names = list(data)
    curves = {}
    for model_id in ("A", "B", "C"):
        mask = mask_for(model_id, chain.n_params)
        for tr, te in ((names[0], names[1]), (names[1], names[0])):
            p, hist = fit_masked(chain, *data[tr], mask, a.reg)
            stats_tr, _ = errors(chain, p, *data[tr])
            stats_te, e = errors(chain, p, *data[te])
            rep["cross_session"][f"{model_id}: fit {tr} -> test {te}"] = {
                "n_params": int(mask.sum()), "train": stats_tr, "test": stats_te}
            curves[(model_id, te)] = e
            print(f"model {model_id} ({mask.sum():2d} p)  fit {tr:14s} train rms {stats_tr['rms_mm']:.3f}  "
                  f"-> test {te:14s} rms {stats_te['rms_mm']:.3f} mm (nominal {rep['nominal'][te]['rms_mm']:.3f})")

    qa = np.vstack([d[0] for d in data.values()])
    xa = np.vstack([d[1] for d in data.values()])
    rank, s = chain.identifiability(qa)
    p_final, hist = fit_masked(chain, qa, xa, mask_for("C", chain.n_params), a.reg)
    rep["final"] = {"model": "C", "fit_on": names, "identifiable_rank": rank, "n_params": chain.n_params,
                    "rms_history_m": hist, "both_sessions": errors(chain, p_final, qa, xa)[0],
                    "per_session": {k: errors(chain, p_final, *v)[0] for k, v in data.items()},
                    "joint_zero_offsets_deg": [round(math.degrees(p_final[N_PER_JOINT * i + 6]), 4) for i in range(6)],
                    "origin_translation_mm": [np.round(p_final[N_PER_JOINT * i:N_PER_JOINT * i + 3] * 1000, 3).tolist() for i in range(6)],
                    "origin_rotation_deg": [np.round(np.degrees(p_final[N_PER_JOINT * i + 3:N_PER_JOINT * i + 6]), 4).tolist() for i in range(6)],
                    "flange_offset_mm": np.round(p_final[-3:] * 1000, 3).tolist()}
    print(f"final (both sessions): identifiable rank {rank}/{chain.n_params}, rms "
          f"{rep['final']['both_sessions']['rms_mm']:.3f} mm, max {rep['final']['both_sessions']['max_mm']:.3f} mm")
    (out / "calibration_report.json").write_text(json.dumps(rep, indent=2))
    np.save(out / "params_final.npy", p_final)

    # figure: held-out error CDF, nominal vs calibrated (model C), per test session
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), sharey=True)
    for ax, te in zip(axes, names):
        q, x = data[te]
        e0 = np.sort(errors(chain, None, q, x)[1])
        e1 = np.sort(curves[("C", te)])
        y = np.arange(1, len(e0) + 1) / len(e0)
        ax.plot(e0, y, color="#2a78d6", lw=2, label=f"nominal URDF (rms {np.sqrt((e0 ** 2).mean()):.2f} mm)")
        ax.plot(e1, y, color="#eb6834", lw=2, label=f"calibrated, fit on other session (rms {np.sqrt((e1 ** 2).mean()):.2f} mm)")
        ax.set_title(f"test session: {te}", loc="left", fontsize=10, fontweight="bold")
        ax.set_xlabel("|FK - controller TCP| (mm)")
        ax.grid(color="#e6e5e0")
        ax.legend(frameon=False, fontsize=8, loc="lower right")
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    axes[0].set_ylabel("fraction of poses")
    fig.suptitle("CR10A kinematic alignment - held-out (cross-session) error CDF, REAL robot data", x=0.01, ha="left",
                 fontweight="bold", fontsize=11)
    fig.tight_layout()
    fig.savefig(out / "calibration_cdf.png", dpi=160, facecolor="#fcfcfb")
    if a.write_urdf:
        target = ROOT / "ros2_ws/src/ex_description/urdf/dobot_cr10_calibrated.xacro"
        write_urdf(chain, p_final, target)  # position part first (orientation step needs this model)
        bags = [ROOT / "data/bags" / b for b in ("cr10a_live_01", "cr10a_live_02") if (ROOT / "data/bags" / b).exists()]
        if bags:
            import xacro
            path = ROOT / "ros2_ws/src/ex_description/urdf/ex_cr10.urdf.xacro"
            tmp = Path("/tmp/ex_cr10_for_calib_cal.xacro")
            tmp.write_text(path.read_text().replace("$(find ex_description)", str(ROOT / "ros2_ws/src/ex_description")))
            cal_xml = xacro.process_file(str(tmp), mappings={"calibrated": "true"}).toxml()
            rpy, st = flange_rotation_offset(cal_xml, bags)
            rep["final"]["flange_rotation_offset"] = {"rpy_rad": list(rpy), **st, "source": [b.name for b in bags]}
            print(f"flange rotation offset {st['offset_angle_deg']:.4f} deg: mean rot err "
                  f"{st['rot_err_deg_mean_before']:.4f} -> {st['rot_err_deg_mean_after']:.4f} deg (n={st['n']})")
            write_urdf(chain, p_final, target, rpy)
            (out / "calibration_report.json").write_text(json.dumps(rep, indent=2))
        else:
            print("no live bags found: orientation offset not estimated (rpy = 0)")
        print(f"wrote {target.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
