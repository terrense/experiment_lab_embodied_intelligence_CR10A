"""The CENSORING diagnostic: what happens if the arm is NOT stopped at the envelope contact? (SIMULATION ONLY).

The reason this script exists
-----------------------------
The project's headline collision finding is that the *conservative capsule envelope* touches the
wall in about 40 % of the out-of-distribution episodes while the **independent mesh checker
finds no real contact in any of them**. That is a strong statement, and until now it had a hole
in it:

    the episode is TERMINATED the moment the conservative envelope touches the obstacle
    (learning/ex_learning/envs/cr10a_reach_wall.py::step), so the trajectory the mesh checker
    sees is CENSORED - the arm was stopped at the envelope and never got the chance to reach a
    real contact.

So "no mesh contact" did not mean "the unfiltered policy is physically safe". It meant "the
unfiltered policy did not reach a real contact **before the envelope stopped it**". Those are
different claims, and the second one is much weaker.

This script measures the difference. The SAME policies run with `terminate_on_contact=False`,
so the arm keeps moving past the first envelope contact, and the independent mesh checker then
measures the PHYSICAL outcome: did the mesh actually touch, and how close did it get?

The rules this script follows
------------------------------
* **The training and the completion statistics are untouched.** `terminate_on_contact` defaults
  to True and only this diagnostic turns it off. A completion number from this mode would be
  meaningless (the episode runs past the contact) and is not reported here at all.
* **This is a SIMULATION-ONLY diagnostic.** It deliberately drives the arm into the obstacle.
  It must never be run on hardware, and the paper says so where the numbers appear.
* **The comparison is paired.** Each method runs on the same scenario seeds, and the censored
  (the archived behaviour) and the uncensored trajectory of the same episode are both reported,
  so the censoring depth - how much the arm would have kept moving - is visible.

    cd learning && ~/ex_rlenv/bin/python ../evaluation/physical_contact.py --task wall_ood
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "learning"))
sys.path.insert(0, str(ROOT / "evaluation"))

from episode_manifest import splits_for  # noqa: E402


def _make(task: str, terminate_on_contact: bool, post_contact_steps: int = 60):
    """The environment for the diagnostic.

    `conservative_geometry=True` (the rigorous lower bound) and `substep_collision_check=True`
    (the contact is seen inside the step) are used in BOTH the censored and the uncensored run,
    so the two differ only in the termination, not in the detector.
    """
    from ex_learning.envs import make_env
    return make_env(task, conservative_geometry=True, substep_collision_check=True,
                    terminate_on_contact=terminate_on_contact,
                    post_contact_steps=post_contact_steps)


def run_cell(env, policy_factory, episodes: int, seed0: int) -> List[Dict]:
    """Run the episodes and record the trajectory of each."""
    from compare_evaluators import _boxes_from_env
    out = []
    for ep in range(episodes):
        seed = seed0 + ep
        obs, _ = env.reset(seed=seed)
        boxes = _boxes_from_env(env)
        policy = policy_factory(env)
        traj = []
        collided = False
        for _ in range(env.cfg.max_steps):
            a = np.asarray(policy(obs), float)
            traj.append(env.q().copy())
            obs, r, term, trunc, info = env.step(a)
            collided = collided or bool(info.get("collision", False))
            if term or trunc:
                break
        traj.append(env.q().copy())
        out.append({"episode": ep, "seed": seed, "envelope_contact": collided,
                    "trajectory": traj, "boxes": boxes})
    return out


def _cell_worker(job) -> Dict:
    """One (policy seed, method, mode) cell. It runs in its own process.

    The independent mesh checker is the expensive part (it re-derives the link geometry and the
    distances for every configuration of every trajectory), so the cells are run in parallel.
    """
    (a, s, method, mode, seed0) = job
    from ex_learning.eval_ppo import ppo_policy_factory
    from ex_learning.execution_model import ServoExecutionModel
    from ex_learning.safety_filter import cbf_policy_exec, cbf_policy_qp
    from independent_collision import IndependentCollisionChecker

    m = json.loads((ROOT / "data" / "reports" / "learning" / "execution_channel.json")
                   .read_text())["identification"]["model"]
    model = ServoExecutionModel(omega=m["omega"], zeta=m["zeta"], v_gain=m["v_gain"],
                                dt=m["dt"], sub=m.get("sub", 10))
    checker = IndependentCollisionChecker()

    t0 = time.time()
    model_dir = ROOT / "data" / "reports" / "learning" / f"ppo_wall_rand_s{s}"
    base = ppo_policy_factory(model_dir, a.task)
    env = _make(a.task, terminate_on_contact=(mode == "censored"),
                post_contact_steps=a.post_contact_steps)

    def factory(e):
        if method == "raw":
            return base(e)
        if method == "qp":
            return cbf_policy_qp(e, base(e), margin=a.qp_margin, alpha=5.0, rho=0.0)
        return cbf_policy_exec(e, base(e), model, margin=a.exec_margin)

    rec = run_cell(env, factory, a.episodes, seed0)
    mesh_contacts, mins, depths = 0, [], []
    for r in rec:
        res = checker.check_trajectory(r["trajectory"], r["boxes"])
        mesh_contacts += int(res["collision"])
        mins.append(res["min_clearance_m"])
        depths.append(len(r["trajectory"]))
    env_contacts = sum(1 for r in rec if r["envelope_contact"])
    return {"policy_seed": s, "method": method, "mode": mode,
            "episodes": a.episodes,
            "envelope_contact_rate": env_contacts / len(rec),
            "mesh_contact_rate": mesh_contacts / len(rec),
            "min_mesh_clearance_m_median": float(np.median(mins)),
            "min_mesh_clearance_m_min": float(np.min(mins)),
            "trajectory_length_median": float(np.median(depths)),
            "wall_s": round(time.time() - t0, 1)}


def main(argv=None) -> int:
    import concurrent.futures as cf

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="wall_ood")
    ap.add_argument("--split", default="test")
    ap.add_argument("--seeds", nargs="*", type=int, default=[0, 1, 2])
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--qp-margin", type=float, default=0.04)
    ap.add_argument("--exec-margin", type=float, default=0.01)
    ap.add_argument("--post-contact-steps", type=int, default=60)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--out", type=Path,
                    default=ROOT / "data" / "reports" / "learning" / "physical_contact.json")
    a = ap.parse_args(argv)

    splits = splits_for()
    if a.split not in splits:
        raise SystemExit(f"unknown split {a.split!r}; known: {sorted(splits)}")
    seed0 = splits[a.split][0]

    for s in a.seeds:
        model_dir = ROOT / "data" / "reports" / "learning" / f"ppo_wall_rand_s{s}"
        if not (model_dir / "ppo.zip").exists():
            raise SystemExit(f"missing policy {model_dir}")

    METHODS = ("raw", "qp", "exec")
    out: Dict = {"task": a.task, "split": a.split, "seed_start": seed0, "episodes": a.episodes,
                 "seeds": a.seeds, "qp_margin": a.qp_margin, "exec_margin": a.exec_margin,
                 "post_contact_steps": a.post_contact_steps,
                 "note": "SIMULATION ONLY. The uncensored run deliberately drives the arm past the "
                         "conservative envelope contact to measure the PHYSICAL outcome. It must "
                         "never be run on hardware, and its completion numbers are not reported.",
                 "rows": []}

    print(f"{'seed':>4} {'method':8} {'mode':>11} {'env contact':>12} {'mesh contact':>13} "
          f"{'min mesh mm':>12} {'steps after':>12}")

    jobs = [(a, s, method, mode, seed0)
            for s in a.seeds for method in METHODS for mode in ("censored", "uncensored")]
    workers = max(1, min(a.workers, len(jobs)))
    with cf.ProcessPoolExecutor(max_workers=workers) as pool:
        for row in pool.map(_cell_worker, jobs):
            out["rows"].append(row)
            print(f"{row['policy_seed']:>4} {row['method']:8} {row['mode']:>11} "
                  f"{row['envelope_contact_rate']*100:11.1f}% "
                  f"{row['mesh_contact_rate']*100:12.1f}% "
                  f"{row['min_mesh_clearance_m_median']*1000:11.1f} "
                  f"{row['trajectory_length_median']:12.0f}", flush=True)

    # the aggregate: does letting the arm continue change the physical verdict?
    print("\n== the aggregate over the training seeds")
    agg: Dict = {}
    for method in METHODS:
        for mode in ("censored", "uncensored"):
            rows = [r for r in out["rows"] if r["method"] == method and r["mode"] == mode]
            agg[f"{method}|{mode}"] = {
                "envelope_contact_rate": float(np.mean([r["envelope_contact_rate"] for r in rows])),
                "mesh_contact_rate": float(np.mean([r["mesh_contact_rate"] for r in rows])),
                "min_mesh_clearance_m_median": float(np.mean(
                    [r["min_mesh_clearance_m_median"] for r in rows])),
                "min_mesh_clearance_m_worst": float(min(
                    r["min_mesh_clearance_m_median"] for r in rows)),
            }
    out["aggregate"] = agg
    for k, v in agg.items():
        print(f"  {k:22s} envelope {v['envelope_contact_rate']*100:5.1f}%   "
              f"mesh {v['mesh_contact_rate']*100:5.1f}%   "
              f"min mesh clearance {v['min_mesh_clearance_m_median']*1000:6.1f} mm (median), "
              f"{v['min_mesh_clearance_m_worst']*1000:6.1f} mm (worst)")

    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(out, indent=1))
    print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
