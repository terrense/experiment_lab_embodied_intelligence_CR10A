"""The stopping tail and the nonlinear re-check (SIMULATION ONLY).

§5.4 of the manuscript specifies two things the implemented filter does **not** do:

  * a **complete stopping tail** - "a zero velocity command is not assumed to stop the current
    physical motion";
  * a **nonlinear re-check** of every candidate command, including the physics substeps.

§5.5 then needs an **uncertainty budget** with its terms separated. This script measures all
three on the SIMULATED plant and writes one report:

  A. the STOPPING TAIL - measured, not modelled: the arm is driven to a state, the command is cut
     to zero, and the plant is rolled until it settles. How far does it still travel? How long
     does the tail last? Does the tail bring the arm CLOSER to the obstacle than at the cut?
     And does the execution model predict the coast it claims to predict?

  B. the NONLINEAR RE-CHECK - the filter reasons about a FIRST-ORDER expansion of the motion
     (dQ_k(v) ~= dQ_k(0) + G_k v). This script rolls the plant NONLINEARLY over the same
     horizon and reports how far the linearised prediction is from what actually happened. That
     error is what a margin has to absorb, and it is invisible to the filter.

  C. the SEPARATED UNCERTAINTY BUDGET - two different errors that are easy to confuse:
       * the GEOMETRIC error: the environment scores clearance with the enclosing capsules, the
         physical truth is the mesh surface. The gap is measured per link; it is the tube radius
         eps_i that §5.5's conditional argument needs.
       * the PREDICTION error: the linearised prediction against the nonlinear rollout (B).
     A margin m only certifies anything if m > eps_geo + delta_pred. The script reports the two
     terms separately, because only then can a reader see which one dominates.

Nothing here touches a real robot. The CR10A was not connected.

    cd learning && ~/ex_rlenv/bin/python ../evaluation/stopping_rollout.py --task wall_ood
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))
sys.path.insert(0, str(ROOT / "evaluation"))
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "ex_mujoco_sim"))

from ex_mujoco_sim.servo import servo_torque  # noqa: E402


# --------------------------------------------------------------------------- the plant
def restore(env, q, qd, q_ref) -> None:
    """Put the plant back into a state it was in (position, velocity and reference).

    The environment has no state setter, so this is written out here: MuJoCo's data is reset, the
    arm's generalised coordinates are written back, and the kinematics are refreshed. The scene
    (the wall) is untouched - it belongs to the episode, not to the motion.
    """
    mujoco.mj_resetData(env.m, env.d)
    env.d.qpos[env.qadr] = np.asarray(q, float)
    env.d.qvel[env.dadr] = np.asarray(qd, float)
    env.q_ref = np.asarray(q_ref, float).copy()
    mujoco.mj_forward(env.m, env.d)
    env.t = 0
    for attr, val in (("succeeded", False), ("collided", False), ("substep_collision", False),
                      ("succeeded_bonus", False)):
        if hasattr(env, attr):
            setattr(env, attr, val)
    env.min_clearance_step = float("inf")
    env.min_clearance_episode = float("inf")


def plant_rollout(env, v: np.ndarray, n_policy_steps: int, record_every: int = 2,
                  clearance: bool = True) -> Dict[str, np.ndarray]:
    """Roll the plant EXACTLY as the environment does, but record the state.

    The stepping is a copy of `ReachEnv.step` (reference integration, joint-limit clipping, the
    PD servo with its gravity/Corielis feed-forward and its torque saturation), so the recorded
    trajectory is the trajectory the environment would have produced. The difference is only that
    the state is sampled every `record_every` PHYSICS SUBSTEPS instead of once per policy step -
    which is what "rechecked by nonlinear rollout, including physics substeps" asks for.
    """
    v = np.asarray(v, float)
    Q, QD, QR, C = [], [], [], []
    Q.append(env.q())
    QD.append(env.d.qvel[env.dadr].copy())
    QR.append(np.asarray(env.q_ref, float).copy())
    if clearance:
        C.append(float(env.clearance()))
    k = 0
    for _ in range(n_policy_steps):
        for _ in range(env.n_sub):
            env.q_ref = env.q_ref + v * env.m.opt.timestep
            clipped = np.clip(env.q_ref, env.lo, env.hi)
            mask = clipped == env.q_ref
            env.q_ref = clipped
            q, dq = env.d.qpos[env.qadr], env.d.qvel[env.dadr]
            tau = servo_torque(q, dq, env.q_ref, v * mask, env.d.qfrc_bias[env.dadr],
                               env.gains, env.tau_max)
            env.d.ctrl[:] = tau
            mujoco.mj_step(env.m, env.d)
            k += 1
            if k % record_every == 0:
                Q.append(env.q())
                QD.append(env.d.qvel[env.dadr].copy())
                QR.append(np.asarray(env.q_ref, float).copy())
                if clearance:
                    C.append(float(env.clearance()))
    return {"Q": np.array(Q), "QD": np.array(QD), "QR": np.array(QR),
            "C": np.array(C) if C else np.zeros(0)}


# --------------------------------------------------------------------- the stopping tail
def stopping_tail(env, model, q, qd, q_ref, v: np.ndarray, hold_steps: int = 3,
                  max_tail_steps: int = 60, eps_stop: float = 1e-3, fk=None) -> Dict:
    """Hold `v` for `hold_steps`, cut the command to zero, and roll until the arm settles."""
    lo, hi = np.asarray(env.lo, float), np.asarray(env.hi, float)
    restore(env, q, qd, q_ref)
    plant_rollout(env, v, hold_steps, clearance=False)
    q_cut, qd_cut, qr_cut = env.q(), env.d.qvel[env.dadr].copy(), np.asarray(env.q_ref, float).copy()
    tcp_cut = env.tcp().copy()
    clr_cut = float(env.clearance())

    tail = plant_rollout(env, np.zeros(6), max_tail_steps)
    speeds = np.abs(tail["QD"]).max(axis=1)
    settled = np.where(speeds < eps_stop)[0]
    settle = int(settled[0]) if len(settled) else -1
    # the records are every `record_every` physics substeps, so a record index is not a policy step
    rec_dt = 2 * env.m.opt.timestep
    tail_s = settle * rec_dt if settle >= 0 else None

    q_end = tail["Q"][-1]
    # what the model predicts for the same cut (the reference keeps its value during the coast)
    _, Qp = model.predict_constant(q_cut, qd_cut, qr_cut, np.zeros(6), lo, hi,
                                   horizon=max_tail_steps)
    pred = Qp[0] - q_cut[None, :]

    # the COAST PROFILE: how far the arm has travelled a given time after the command was cut.
    # The filter's look-ahead is 0.1 s, so the profile says how much of the coast a filter with
    # that horizon cannot see.
    profile = {}
    for delay_ms in (50, 100, 200, 500, 1000):
        r = min(int(delay_ms / 1000.0 / rec_dt), len(tail["Q"]) - 1)
        entry = {"coast_rad": float(np.abs(tail["Q"][r] - q_cut).max())}
        if fk is not None:
            entry["coast_tcp_mm"] = float(1000 * np.linalg.norm(fk(tail["Q"][r]) - tcp_cut))
        if tail["C"].size:
            entry["clearance_loss_mm"] = float(1000 * (clr_cut - tail["C"][r]))
        profile[f"{delay_ms}ms"] = entry

    return {
        "coast_rad": float(np.abs(q_end - q_cut).max()),
        "coast_rad_mean": float(np.abs(q_end - q_cut).mean()),
        "coast_tcp_mm": float(1000 * np.linalg.norm(env.tcp() - tcp_cut)),
        "coast_profile": profile,
        "tail_records": settle, "tail_s": round(tail_s, 3) if tail_s is not None else None,
        "tail_max_steps": max_tail_steps,
        "speed_at_cut_rad_s": float(np.abs(qd_cut).max()),
        "speed_after_tail_rad_s": float(speeds[-1]),
        "clearance_at_cut_m": clr_cut,
        "clearance_min_tail_m": float(tail["C"].min()) if tail["C"].size else None,
        "clearance_final_m": float(tail["C"][-1]) if tail["C"].size else None,
        "tail_approaches_obstacle": bool(tail["C"].size and tail["C"].min() < clr_cut),
        "pred_coast_rad": float(np.abs(pred[-1]).max()),
        "pred_coast_rad_mean": float(np.abs(pred[-1]).mean()),
    }


# -------------------------------------------------------------- the nonlinear re-check
def make_fk(env):
    """Forward kinematics on a scratch MjData, so the environment's own state is not disturbed."""
    d = mujoco.MjData(env.m)

    def fk(q: np.ndarray) -> np.ndarray:
        mujoco.mj_resetData(env.m, d)
        d.qpos[env.qadr] = np.asarray(q, float)
        mujoco.mj_forward(env.m, d)
        return d.site_xpos[env.site].copy()
    return fk


def nonlinear_recheck(env, model, q, qd, q_ref, v: np.ndarray, horizon: int = 5,
                      sample_every: int = 2, fk=None) -> Dict:
    """Compare the filter's FIRST-ORDER prediction with the NONLINEAR rollout.

    The filter's constraint uses dQ_k(v) ~= dQ_k(0) + G_k v. This rolls the plant with the same
    held command and reports the error of that expansion, per sample and overall.
    """
    lo, hi = np.asarray(env.lo, float), np.asarray(env.hi, float)
    restore(env, q, qd, q_ref)
    truth = plant_rollout(env, v, horizon, clearance=False)

    times, Qp = model.predict_constant(q, qd, q_ref, v, lo, hi, horizon=horizon)
    dq0, G = model.affine(q, qd, q_ref, v, lo, hi, horizon=horizon, step=sample_every)
    # the affine sample k sits at model row (k+1)*sample_every. The model's rows are dt/sub = 2 ms
    # apart and the plant records every 2 physics substeps (1 ms each), so the recorded index and
    # the model row are the SAME number of milliseconds: record r == model row r.
    rows = (np.arange(dq0.shape[0]) + 1) * sample_every
    rows = rows[rows < len(truth["Q"])]
    actual = truth["Q"][rows] - q[None, :]
    nominal = Qp[0, rows] - q[None, :]
    affine = dq0[:len(rows)] + np.einsum("kij,j->ki", G[:len(rows)], np.asarray(v, float))

    err_affine = affine - actual
    err_nominal = nominal - actual
    err_lin = affine - nominal          # the LINEARISATION error alone (affine vs the model's own output)
    out = {
        "n_samples": int(len(rows)),
        "affine_err_rad_rms": float(np.sqrt((err_affine ** 2).sum(axis=1).mean())),
        "affine_err_rad_max": float(np.abs(err_affine).max()),
        "nominal_err_rad_rms": float(np.sqrt((err_nominal ** 2).sum(axis=1).mean())),
        "nominal_err_rad_max": float(np.abs(err_nominal).max()),
        "linearisation_err_rad_rms": float(np.sqrt((err_lin ** 2).sum(axis=1).mean())),
        "linearisation_err_rad_max": float(np.abs(err_lin).max()),
        # the SAME quantity the identification reports (the error at the END of the horizon), so
        # the two numbers are directly comparable
        "horizon_end_err_rad_rms": float(np.sqrt((err_nominal[-1] ** 2).sum())),
        "horizon_end_affine_err_rad_rms": float(np.sqrt((err_affine[-1] ** 2).sum())),
        "coast_pred_rad_max": float(np.abs(dq0).max()),
        "coast_actual_rad_max": float(np.abs(actual).max()),
        "per_sample_affine_err_mm": (1000 * np.abs(err_affine).max(axis=1)).tolist(),
    }
    if fk is not None:
        # the physically meaningful scale: the tool-centre-point error of the predicted motion.
        # The clearance is 1-Lipschitz in the robot's displacement, so this bounds the error of the
        # clearance prediction that the filter's margin has to absorb.
        tcp_pred = np.array([fk(nominal[k] + q) for k in range(len(rows))])
        tcp_act = np.array([fk(actual[k] + q) for k in range(len(rows))])
        tcp_err = np.linalg.norm(tcp_pred - tcp_act, axis=1)
        out["tcp_err_mm_median"] = float(1000 * np.median(tcp_err))
        out["tcp_err_mm_max"] = float(1000 * tcp_err.max())
        out["tcp_displacement_mm_max"] = float(1000 * np.linalg.norm(tcp_act - tcp_act[0], axis=1).max())
    return out


# ------------------------------------------------------------ the geometric uncertainty
def per_link_mesh_clearance(checker, boxes, q: Optional[np.ndarray] = None) -> Dict[str, float]:
    """The mesh-surface clearance per link (the physical truth).

    The checker owns its OWN MuJoCo model instance, so its pose must be set explicitly - otherwise
    the mesh is evaluated at whatever configuration the checker was left at, and the comparison
    against the environment's capsule clearance is meaningless.
    """
    from independent_collision import ARM_LINKS, IGNORED_BOX_LINKS, _point_aabb_distance
    if q is not None:
        checker.set_q(q)
    out = {}
    for link in ARM_LINKS:
        pts = checker.link_points_world(link)
        best = float("inf")
        for b in boxes:
            if (link, b["name"]) in IGNORED_BOX_LINKS:
                continue
            lo = np.asarray(b["center"], float) - np.asarray(b["size"], float) / 2
            hi = np.asarray(b["center"], float) + np.asarray(b["size"], float) / 2
            best = min(best, float(_point_aabb_distance(pts, lo, hi).min()))
        out[link] = best
    return out


def per_link_capsule_clearance(env) -> Dict[str, float]:
    """The environment's (capsule) clearance per link."""
    d, _, _ = env.pair_distances(conservative=True)
    out: Dict[str, float] = {}
    for i, (ci, bi) in enumerate(env.pairs):
        link = env.capsules[ci].link
        out[link] = min(out.get(link, float("inf")), float(d[i]))
    return out


def main(argv=None) -> int:
    from compare_evaluators import _make, _boxes_from_env
    from ex_learning.eval_ppo import ppo_policy_factory
    from ex_learning.execution_model import ServoExecutionModel
    from independent_collision import IndependentCollisionChecker

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="wall_ood")
    ap.add_argument("--seeds", nargs="*", type=int, default=[0, 1, 2])
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--states-per-episode", type=int, default=12)
    ap.add_argument("--state-stride", type=int, default=20,
                    help="policy steps between two sampled states")
    ap.add_argument("--hold-steps", type=int, default=3)
    ap.add_argument("--horizon", type=int, default=5)
    ap.add_argument("--sample-every", type=int, default=2)
    ap.add_argument("--max-tail-steps", type=int, default=60)
    ap.add_argument("--policy", default="exec", choices=("raw", "exec"),
                    help="which policy the states are sampled from. 'exec' samples the states the "
                         "FILTER produces, where the clearance is at least the filter's margin by "
                         "construction - that is where the terminal-stop question is asked. 'raw' "
                         "samples the unfiltered policy, where the arm is often already in "
                         "collision and the tail's erosion is not meaningful.")
    ap.add_argument("--exec-margin", type=float, default=0.01)
    ap.add_argument("--out", type=Path,
                    default=ROOT / "data" / "reports" / "learning" / "stopping_rollout.json")
    a = ap.parse_args(argv)

    m = json.loads((ROOT / "data" / "reports" / "learning" / "execution_channel.json")
                   .read_text())["identification"]["model"]
    model = ServoExecutionModel(omega=m["omega"], zeta=m["zeta"], v_gain=m["v_gain"],
                                dt=m["dt"], sub=m.get("sub", 10))

    env = _make(a.task, True)          # the rigorous (lower-bound) clearance
    checker = IndependentCollisionChecker()
    fk = make_fk(env)

    tails: List[Dict] = []
    rechecks: List[Dict] = []
    eps_rows: List[Dict] = []
    t0 = time.time()

    for s in a.seeds:
        model_dir = ROOT / "data" / "reports" / "learning" / f"ppo_wall_rand_s{s}"
        if not (model_dir / "ppo.zip").exists():
            raise SystemExit(f"missing policy {model_dir}")
        base = ppo_policy_factory(model_dir, a.task)(env)
        if a.policy == "exec":
            from ex_learning.safety_filter import cbf_policy_exec
            policy = cbf_policy_exec(env, base, model, margin=a.exec_margin)
        else:
            policy = base
        for ep in range(a.episodes):
            seed = 20000 + ep
            obs, _ = env.reset(seed=seed)
            boxes = _boxes_from_env(env)
            for k in range(a.states_per_episode):
                a_cmd = np.clip(np.asarray(policy(obs), float), -1.0, 1.0)
                state = (env.q(), env.d.qvel[env.dadr].copy(), np.asarray(env.q_ref, float).copy())
                t = stopping_tail(env, model, *state, a_cmd, hold_steps=a.hold_steps,
                                  max_tail_steps=a.max_tail_steps, fk=fk)
                t.update({"policy_seed": s, "episode": ep, "step": k})
                tails.append(t)
                r = nonlinear_recheck(env, model, *state, a_cmd, horizon=a.horizon,
                                      sample_every=a.sample_every, fk=fk)
                r.update({"policy_seed": s, "episode": ep, "step": k})
                rechecks.append(r)
                # the geometric model error, on the state the arm is in right now
                restore(env, *state)
                cap = per_link_capsule_clearance(env)
                mesh = per_link_mesh_clearance(checker, boxes, q=state[0])
                for link in cap:
                    if link in mesh and np.isfinite(mesh[link]):
                        eps_rows.append({"policy_seed": s, "episode": ep, "step": k, "link": link,
                                         "capsule_m": cap[link], "mesh_m": mesh[link],
                                         "eps_m": mesh[link] - cap[link]})
                for _ in range(a.state_stride):
                    obs, _, term, trunc, _ = env.step(a_cmd)
                    if term or trunc:
                        break

    def stats(vals) -> Dict:
        v = np.asarray([x for x in vals if np.isfinite(x)], float)
        if not len(v):
            return {"n": 0}
        return {"n": int(len(v)), "median": float(np.median(v)), "mean": float(v.mean()),
                "p05": float(np.percentile(v, 5)), "p95": float(np.percentile(v, 95)),
                "max": float(v.max()), "min": float(v.min())}

    coast = [t["coast_rad"] for t in tails]
    pred = [t["pred_coast_rad"] for t in tails]
    eps_by_link: Dict[str, List[float]] = {}
    for r in eps_rows:
        eps_by_link.setdefault(r["link"], []).append(r["eps_m"])

    out = {
        "task": a.task, "seeds": a.seeds, "episodes": a.episodes,
        "policy": a.policy, "exec_margin": a.exec_margin,
        "states_per_episode": a.states_per_episode, "hold_steps": a.hold_steps,
        "horizon": a.horizon, "sample_every": a.sample_every,
        "model": m, "wall_s": round(time.time() - t0, 1),
        "n_states": len(tails),
        "A_stopping_tail": {
            "coast_rad": stats(coast),
            "coast_tcp_mm": stats([t["coast_tcp_mm"] for t in tails]),
            "coast_profile": {
                delay: {
                    "coast_rad": stats([t["coast_profile"][delay]["coast_rad"] for t in tails]),
                    "coast_tcp_mm": stats([t["coast_profile"][delay].get("coast_tcp_mm")
                                           for t in tails if "coast_tcp_mm" in t["coast_profile"][delay]]),
                    "clearance_loss_mm": stats([t["coast_profile"][delay].get("clearance_loss_mm")
                                                for t in tails if "clearance_loss_mm" in t["coast_profile"][delay]]),
                } for delay in ("50ms", "100ms", "200ms", "500ms", "1000ms")
            },
            "tail_s": stats([t["tail_s"] for t in tails if t["tail_s"] is not None]),
            "tail_approaches_obstacle_rate": float(np.mean([t["tail_approaches_obstacle"] for t in tails])),
            "clearance_loss_in_tail_mm": stats([
                1000 * (t["clearance_at_cut_m"] - t["clearance_min_tail_m"])
                for t in tails if t["clearance_min_tail_m"] is not None]),
            "model_predicted_coast_rad": stats(pred),
            "coast_model_error_rad": stats([p - c for p, c in zip(pred, coast)]),
            "coast_ratio_measured_over_predicted": stats(
                [c / p if p > 1e-9 else float("nan") for c, p in zip(coast, pred)]),
        },
        "B_nonlinear_recheck": {
            "affine_err_rad_rms": stats([r["affine_err_rad_rms"] for r in rechecks]),
            "affine_err_rad_max": stats([r["affine_err_rad_max"] for r in rechecks]),
            "nominal_err_rad_rms": stats([r["nominal_err_rad_rms"] for r in rechecks]),
            "horizon_end_err_rad_rms": stats([r["horizon_end_err_rad_rms"] for r in rechecks]),
            "horizon_end_affine_err_rad_rms": stats([r["horizon_end_affine_err_rad_rms"]
                                                      for r in rechecks]),
            "linearisation_err_rad_rms": stats([r["linearisation_err_rad_rms"] for r in rechecks]),
            "linearisation_err_rad_max": stats([r["linearisation_err_rad_max"] for r in rechecks]),
            "tcp_err_mm_median": stats([r["tcp_err_mm_median"] for r in rechecks]),
            "tcp_err_mm_max": stats([r["tcp_err_mm_max"] for r in rechecks]),
            "tcp_displacement_mm_max": stats([r["tcp_displacement_mm_max"] for r in rechecks]),
            "coast_pred_rad_max": stats([r["coast_pred_rad_max"] for r in rechecks]),
            "coast_actual_rad_max": stats([r["coast_actual_rad_max"] for r in rechecks]),
        },
        "C_uncertainty_budget": {
            "eps_by_link_m": {k: stats(v) for k, v in eps_by_link.items()},
            "eps_overall_m": stats([r["eps_m"] for r in eps_rows]),
        },
        "rows": {"tails": tails, "rechecks": rechecks, "eps": eps_rows},
    }
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, indent=1))
    print(json.dumps({k: v for k, v in out.items() if k != "rows"}, indent=1))
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
