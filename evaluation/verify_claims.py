"""Self-check: do the DOCUMENTS still say what the DATA says? (SIMULATION ONLY).

The numbers in the manuscript, in docs/RESULTS.md and in the README are hand-copied from the
committed JSON reports. Hand-copying is exactly how a number drifts away from its source, and
this project has already had to withdraw one such number (the "35 % completion, 0 %
collisions" of the bounded QP at margin 0, which came from a summary that mixed freshly written
and stale sweep files).

So the claims are CHECKED, not trusted. This script re-derives every headline number from the
committed JSON and compares it with what the documents state. It is meant to be run after any
change to the data or to the write-up, and to fail loudly when they disagree.

    cd learning && ~/ex_rlenv/bin/python ../evaluation/verify_claims.py

Exit code 0 means every checked claim still matches its source.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT / "data" / "reports" / "learning"

# the tolerance: the documents quote percentages rounded to one decimal
TOL = 0.06          # percentage points


class Checker:
    def __init__(self) -> None:
        self.rows: List[Dict] = []
        self.failures = 0

    def check(self, name: str, got: Optional[float], expected: Optional[float],
              unit: str = "%", tol: float = TOL) -> None:
        if got is None or expected is None:
            ok = (got is None) == (expected is None)
            detail = f"got {got}, expected {expected}"
        else:
            ok = abs(got - expected) <= tol
            detail = f"got {got:.3f}{unit}, documents say {expected:.3f}{unit}"
        if not ok:
            self.failures += 1
        self.rows.append({"claim": name, "ok": ok, "detail": detail})

    def note(self, name: str, text: str) -> None:
        self.rows.append({"claim": name, "ok": True, "detail": text})

    def report(self) -> int:
        width = max(len(r["claim"]) for r in self.rows) + 2
        for r in self.rows:
            flag = "ok  " if r["ok"] else "FAIL"
            print(f"  [{flag}] {r['claim']:<{width}} {r['detail']}")
        print(f"\n{len(self.rows) - self.failures}/{len(self.rows)} claims match their source")
        return 1 if self.failures else 0


def _load(name: str) -> Optional[Dict]:
    p = REPORTS / name
    if not p.exists():
        return None
    return json.loads(p.read_text())


def _agg(rows: List[Dict], method: str, task: Optional[str] = None) -> Dict[str, float]:
    """The mean over the cells of one method (and optionally one task)."""
    sel = [r for r in rows if r["method"] == method and (task is None or r["task"] == task)]
    if not sel:
        return {}
    out: Dict[str, float] = {}
    for key, get in (("success_rate", lambda r: r["success_rate"]),
                     ("legacy_collision_rate", lambda r: r["legacy_collision_rate"]),
                     ("corrected_collision_rate", lambda r: r["corrected_collision_rate"])):
        out[key] = 100 * sum(get(r) for r in sel) / len(sel)
    mesh = [r["mesh"] for r in sel if r["mesh"]["collision_rate"] is not None]
    out["mesh_collision_rate"] = 100 * sum(m["collision_rate"] for m in mesh) / len(mesh) if mesh else None
    cl = [m["min_clearance_m_median"] for m in mesh if m["min_clearance_m_median"] is not None]
    out["mesh_clearance_mm"] = 1000 * sum(cl) / len(cl) if cl else None
    out["n_cells"] = len(sel)
    return out


def main() -> int:
    c = Checker()
    print("== the confirmatory run on the frozen test range (final_benchmark_full.json)")
    fb = _load("final_benchmark_full.json")
    if fb is None:
        c.check("final_benchmark_full.json present", None, None)
    else:
        rows = fb["rows"]
        c.check("split is the confirmatory range", None if fb["split"] != "test" else 0.0, 0.0)
        c.check("episode manifest hash",
                0.0 if fb["episode_manifest_hash"] == "a995f097b28148c2" else 1.0, 0.0)
        # the table of section 7.7 / RESULTS.md section 8
        for method, task, exp_s, exp_l, exp_c in (
                ("raw", "wall_ood", 36.0, 38.7, 40.7),
                ("raw", None, 65.8, 12.9, 13.6),
                ("cbf_legacy", "wall_ood", 46.0, 10.7, 11.3),
                ("cbf_legacy", None, 68.7, 3.6, 3.8),
                ("qp", "wall_ood", 45.3, 0.0, 0.0),
                ("qp", None, 68.4, 0.0, 0.0),
                ("qp0", "wall_ood", 54.0, 8.0, 19.3),
                ("qp0", None, 71.8, 2.7, 6.4),
                ("exec", "wall_ood", 54.0, 0.0, 0.0),
                ("exec", None, 71.8, 0.0, 0.0)):
            a = _agg(rows, method, task)
            tag = f"{method} {task or 'all'}: completion"
            c.check(tag, a.get("success_rate"), exp_s)
            c.check(f"{method} {task or 'all'}: legacy collisions", a.get("legacy_collision_rate"), exp_l)
            c.check(f"{method} {task or 'all'}: corrected collisions", a.get("corrected_collision_rate"), exp_c)
            if method in ("raw", "exec") and task == "wall_ood":
                c.check(f"{method} {task}: mesh collisions", a.get("mesh_collision_rate"), 0.0)
        # the per-seed table
        per = {}
        for r in rows:
            if r["task"] == "wall_ood":
                per[(r["policy_seed"], r["method"])] = (100 * r["success_rate"],
                                                        100 * r["corrected_collision_rate"])
        for seed, exp in ((0, (50.0, 0.0, 72.0, 22.0, 72.0, 0.0)),
                          (1, (44.0, 0.0, 46.0, 20.0, 46.0, 0.0)),
                          (2, (42.0, 0.0, 44.0, 16.0, 44.0, 0.0))):
            qp = per.get((seed, "qp"), (None, None))
            q0 = per.get((seed, "qp0"), (None, None))
            ex = per.get((seed, "exec"), (None, None))
            c.check(f"seed {seed}: qp(4cm) completion", qp[0], exp[0])
            c.check(f"seed {seed}: qp(4cm) collisions", qp[1], exp[1])
            c.check(f"seed {seed}: qp(0cm) completion", q0[0], exp[2])
            c.check(f"seed {seed}: qp(0cm) collisions", q0[1], exp[3])
            c.check(f"seed {seed}: exec completion", ex[0], exp[4])
            c.check(f"seed {seed}: exec collisions", ex[1], exp[5])

    print("\n== the execution gap (execution_gap.json)")
    eg = _load("execution_gap.json")
    if eg is None:
        c.check("execution_gap.json present", None, None)
    else:
        by = {(r["method"], round(r["margin"], 4)): r for r in eg["rows"]}
        # (initially unsafe, safe->unsafe crossings) per method per margin
        for margin, exp in ((0.005, {"qp": (210, 5), "exec_nocoast": (1, 5), "exec": (6, 28)}),
                            (0.01, {"qp": (468, 9), "exec_nocoast": (1, 4), "exec": (0, 24)}),
                            (0.02, {"qp": (543, 10), "exec_nocoast": (2, 8), "exec": (10, 27)}),
                            (0.04, {"qp": (1923, 20), "exec_nocoast": (22, 55), "exec": (111, 177)})):
            for method, (n_init, n_cross) in exp.items():
                row = by.get((method, margin))
                if row is None:
                    continue
                c.check(f"gap {margin*100:g} cm {method}: steps that started inside the margin",
                        float(row["initially_unsafe"]), float(n_init), "count", tol=0.5)
                c.check(f"gap {margin*100:g} cm {method}: safe-to-unsafe crossings",
                        float(row["safe_to_unsafe_crossings"]), float(n_cross), "count", tol=0.5)
            qp = by.get(("qp", margin))
            if qp:
                c.check(f"gap {margin*100:g} cm: qp gap median (mm)",
                        qp["gap_mm_median"], -12.0, "mm", tol=0.2)
        # the headline: the QP spends its time inside the margin buffer, the look-ahead filter does not
        for margin, exp in ((0.02, (9.05, 0.17)), (0.04, (32.05, 1.85))):
            qp = by.get(("qp", margin))
            ex = by.get(("exec", margin))
            if qp:
                c.check(f"gap {margin*100:g} cm: qp steps inside the margin (%)",
                        100 * qp["initially_unsafe_rate"], exp[0], "%", tol=0.01)
            if ex:
                c.check(f"gap {margin*100:g} cm: exec steps inside the margin (%)",
                        100 * ex["initially_unsafe_rate"], exp[1], "%", tol=0.01)

    print("\n== the command channel (execution_channel.json)")
    ec = _load("execution_channel.json")
    if ec is None:
        c.check("execution_channel.json present", None, None)
    else:
        coasts = {round(r["command_rad_s"], 2): 1000 * r["coast_rad"] for r in ec["rows"]}
        for v, exp in ((0.25, 8.18), (0.5, 10.41), (0.75, 12.65), (1.0, 14.90)):
            c.check(f"coast at {v} rad/s (mrad)", coasts.get(v), exp, "mrad", tol=0.02)
        m = ec["identification"]["model"]
        c.check("identified omega (rad/s)", m["omega"], 37.70, "rad/s", tol=0.01)
        c.check("identified zeta", m["zeta"], 0.90, "", tol=0.001)
        c.check("identified v_gain", m["v_gain"], 0.80, "", tol=0.001)
        c.check("prediction RMS (mrad)", 1000 * ec["identification"]["prediction_error"]["rms_rad"],
                4.57, "mrad", tol=0.02)

    print("\n== the 50-episode tuning frontier (tune50_*.json)")
    try:
        import glob
        import numpy as np
        for mode, exp in (("qp", {0.005: (31.3, 0.0), 0.01: (31.3, 0.0),
                                   0.02: (31.3, 0.0), 0.04: (28.7, 0.0)}),
                          ("exec", {0.005: (32.0, 0.0), 0.01: (32.0, 0.0),
                                    0.02: (31.3, 0.0), 0.04: (28.7, 0.0)})):
            per: Dict[float, List] = {}
            for f in glob.glob(str(REPORTS / f"tune50_{mode}_s*_wall_ood.json")):
                for row in json.loads(Path(f).read_text())["rows"]:
                    per.setdefault(round(row["margin"], 4), []).append(row)
            for margin, (es, ec_) in exp.items():
                rows = per.get(margin, [])
                if not rows:
                    continue
                s = 100 * sum(r["success_rate"] for r in rows) / len(rows)
                cl = 100 * sum(r["collision_rate"] for r in rows) / len(rows)
                c.check(f"tuning {mode} {margin*100:g} cm: completion", s, es)
                c.check(f"tuning {mode} {margin*100:g} cm: collisions", cl, ec_)
    except Exception as e:                                   # pragma: no cover
        c.check("the 50-episode tuning sweeps", None, None)
        c.note("sweep check error", str(e))

    print("\n== the stopping tail, the nonlinear re-check, the uncertainty budget (stopping_rollout.json)")
    sr = _load("stopping_rollout.json")
    if sr is None:
        c.check("stopping_rollout.json present", None, None)
    else:
        A = sr["A_stopping_tail"]
        B = sr["B_nonlinear_recheck"]
        C = sr["C_uncertainty_budget"]
        for delay, exp_tcp, exp_loss, exp_max in (("50ms", 109.6, 34.6, 62.6),
                                                   ("100ms", 141.9, 38.0, 86.9),
                                                   ("200ms", 128.8, 23.8, 79.8)):
            prof = A["coast_profile"][delay]
            c.check(f"stopping tail {delay}: tool-point travel (p95, mm)",
                    prof["coast_tcp_mm"]["p95"], exp_tcp, "mm", tol=0.1)
            c.check(f"stopping tail {delay}: clearance lost (p95, mm)",
                    prof["clearance_loss_mm"]["p95"], exp_loss, "mm", tol=0.1)
            c.check(f"stopping tail {delay}: clearance lost (worst, mm)",
                    prof["clearance_loss_mm"]["max"], exp_max, "mm", tol=0.1)
        c.check("the tail brings the arm closer to the obstacle", 100 * A["tail_approaches_obstacle_rate"],
                35.2, "%", tol=0.1)
        c.check("the model predicts the coast (measured/predicted, median)",
                A["coast_ratio_measured_over_predicted"]["median"], 0.99997, "", tol=0.0001)
        c.check("the model's error at the horizon end (mrad, median)",
                1000 * B["horizon_end_err_rad_rms"]["median"], 22.3, "mrad", tol=0.05)
        c.check("the expansion's own error (mrad, median)",
                1000 * B["linearisation_err_rad_rms"]["median"], 8.5, "mrad", tol=0.05)
        c.check("the tool-point error of the predicted motion (mm, median)",
                B["tcp_err_mm_median"]["median"], 22.5, "mm", tol=0.05)
        eps = [r["eps_m"] for r in sr["rows"]["eps"]]
        c.check("the geometric tube is one-sided (samples where the capsule OVER-estimates)",
                sum(1 for e in eps if e < 0.0), 0, "count", tol=0.5)
        c.check("the geometric tube, sample count", len(eps), 1512, "count", tol=0.5)
        c.check("the geometric tube, median (mm)", 1000 * C["eps_overall_m"]["median"], 49.0, "mm", tol=0.1)
        c.check("the geometric tube, worst (mm)", 1000 * C["eps_overall_m"]["max"], 125.6, "mm", tol=0.1)

    print("\n== the planner arms (b6_plans.json / b6_planner_filter.json)")
    bp = _load("b6_plans.json")
    b6 = _load("b6_planner_filter.json")
    if bp is None or b6 is None:
        c.check("the B6 reports present", None, None)
    else:
        plans = bp["plans"]
        solved = [p for p in plans if p["solved"]]
        c.check("the planner solves this share of the episodes", 100.0 * len(solved) / len(plans),
                84.0, "%", tol=0.1)
        c.check("the median planning time (s)",
                float(b6["plan_summary"]["plan_time_s_median"]), 1.53, "s", tol=0.005)
        c.check("the worst planning time (s)",
                max(p["plan_s"] for p in solved), 16.0, "s", tol=0.05)
        # the six arms: (completion at 30 s, completion at 6 s, legacy, corrected) per arm
        for arm, exp in (("raw", (38.7, 36.0, 38.7, 40.7)),
                         ("qp", (48.7, 45.3, 0.0, 0.0)),
                         ("exec", (56.7, 54.0, 0.0, 0.0)),
                         ("plan", (84.0, 4.0, 0.0, 0.0)),
                         ("plan_qp", (80.0, 4.0, 0.0, 0.0)),
                         ("plan_exec", (84.0, 4.0, 0.0, 0.0))):
            rows = [r for r in b6["rows"] if r["arm"] == arm]
            c.check(f"B6 {arm}: completion at 30 s",
                    100 * sum(r["success_rate"] for r in rows) / len(rows), exp[0], "%", tol=0.1)
            c.check(f"B6 {arm}: completion at 6 s",
                    100 * sum(r["success_rate_6s"] for r in rows) / len(rows), exp[1], "%", tol=0.1)
            c.check(f"B6 {arm}: legacy envelope collisions",
                    100 * sum(r["legacy_collision_rate"] for r in rows) / len(rows), exp[2], "%", tol=0.1)
            c.check(f"B6 {arm}: corrected collisions",
                    100 * sum(r["corrected_collision_rate"] for r in rows) / len(rows), exp[3], "%", tol=0.1)
            mesh = [r["mesh"]["collision_rate"] for r in rows if r["mesh"]["collision_rate"] is not None]
            if mesh:
                c.check(f"B6 {arm}: mesh collisions", 100 * sum(mesh) / len(mesh), 0.0, "%", tol=0.1)
        # the harness cross-check: the 6 s column of the B6 table IS the confirmatory table
        c.check("the B6 harness reproduces the confirmatory completion of the unfiltered policy",
                100 * sum(r["success_rate_6s"] for r in b6["rows"] if r["arm"] == "raw") / 3, 36.0, "%", tol=0.1)
        c.check("the B6 harness reproduces the confirmatory completion of the bounded QP",
                100 * sum(r["success_rate_6s"] for r in b6["rows"] if r["arm"] == "qp") / 3, 45.3, "%", tol=0.1)
        c.check("the B6 harness reproduces the confirmatory completion of the execution-aware filter",
                100 * sum(r["success_rate_6s"] for r in b6["rows"] if r["arm"] == "exec") / 3, 54.0, "%", tol=0.1)
        # the planner arms are seed-independent: identical in every training seed
        for arm in ("plan", "plan_qp", "plan_exec"):
            vals = {round(100 * r["success_rate"], 6) for r in b6["rows"] if r["arm"] == arm}
            c.check(f"the {arm} arm is identical in every training seed", len(vals), 1, "count", tol=0.5)

    print("\n== the confirmatory statistics with nine training seeds (confirmatory_stats.json)")
    cs = _load("confirmatory_stats.json")
    if cs is None:
        c.check("confirmatory_stats.json present", None, None)
    else:
        c.check("the statistics use nine training seeds", float(len(cs["seeds"])), 9.0, "count", tol=0.5)
        cmp_ = cs["comparisons"]
        eh = cmp_["exec_-_qp"]["completion"]["hierarchical"]
        en = cmp_["exec_-_qp"]["completion"]["naive"]
        c.check("exec - qp: the mean completion difference", 100 * eh["mean_difference"], 6.67, "%", tol=0.02)
        c.check("exec - qp: the hierarchical interval, low", 100 * eh["ci_low"], 2.0, "%", tol=0.02)
        c.check("exec - qp: the hierarchical interval, high", 100 * eh["ci_high"], 12.0, "%", tol=0.02)
        c.check("exec - qp: the naive interval, low", 100 * en["ci_low"], 4.22, "%", tol=0.02)
        c.check("exec - qp: the naive interval, high", 100 * en["ci_high"], 9.11, "%", tol=0.02)
        c.check("exec - qp: the between-seed spread of the difference",
                100 * eh["between_seed_sd"], 7.07, "%", tol=0.02)
        c.check("exec - qp: the verdict is NOT_SEPARABLE",
                1.0 if cmp_["exec_-_qp"]["completion"]["separability"]["verdict"] == "NOT_SEPARABLE" else 0.0,
                1.0)
        ch = cmp_["exec_-_qp0"]["collision"]["hierarchical"]
        c.check("exec - qp0: the collision difference", 100 * ch["mean_difference"], -7.33, "%", tol=0.02)
        c.check("exec - qp0: the collision interval, low", 100 * ch["ci_low"], -11.11, "%", tol=0.02)
        c.check("exec - qp0: the collision interval, high", 100 * ch["ci_high"], -4.0, "%", tol=0.02)
        c.check("exec - qp0: the collision interval excludes zero",
                1.0 if ch["excludes_zero"] else 0.0, 1.0)
        # the per-seed table of section 7.7.1, from the episode cache
        cache = _load("confirmatory_episodes.json")
        if cache is not None:
            for seed, exp in ((0, (50, 0, 72, 0, 72, 0)), (1, (44, 0, 46, 16, 46, 0)),
                              (2, (42, 0, 44, 8, 44, 0)), (3, (42, 0, 50, 6, 48, 0)),
                              (4, (42, 0, 44, 6, 44, 0)), (5, (38, 0, 48, 10, 48, 0)),
                              (6, (60, 0, 66, 4, 68, 0)), (7, (40, 0, 38, 8, 38, 0)),
                              (8, (50, 0, 56, 8, 60, 0))):
                for i, m in enumerate(("qp", "qp0", "exec")):
                    su = cache["success"][f"{m}|{seed}"]
                    co = cache["collision"][f"{m}|{seed}"]
                    c.check(f"seed {seed} {m}: completion", 100 * sum(su.values()) / len(su),
                            exp[2 * i], "%", tol=0.1)
                    c.check(f"seed {seed} {m}: envelope collisions", 100 * sum(co.values()) / len(co),
                            exp[2 * i + 1], "%", tol=0.1)

    print("\n== the replication on a second untouched range (final_benchmark_replication.json)")
    rep = _load("final_benchmark_replication.json")
    if rep is None:
        c.check("final_benchmark_replication.json present", None, None)
    else:
        c.check("the replication is on the 30000-30199 range",
                0.0 if rep["seed_start"] == 30000 else 1.0, 0.0)
        c.check("the replication uses manifest version 2", float(rep["episode_manifest_version"]), 2.0,
                "", tol=0.5)
        for method, exp in (("raw", (40.7, 32.0, 33.3)), ("cbf_legacy", (51.3, 6.0, 6.7)),
                            ("qp", (52.0, 0.0, 0.0)), ("qp0", (56.0, 7.3, 16.0)),
                            ("exec", (56.7, 0.0, 0.0))):
            a = _agg(rep["rows"], method, "wall_ood")
            c.check(f"replication {method}: completion", a.get("success_rate"), exp[0], "%", tol=0.1)
            c.check(f"replication {method}: envelope collisions", a.get("legacy_collision_rate"), exp[1],
                    "%", tol=0.1)
            c.check(f"replication {method}: corrected collisions", a.get("corrected_collision_rate"),
                    exp[2], "%", tol=0.1)
            c.check(f"replication {method}: mesh collisions", a.get("mesh_collision_rate"), 0.0, "%", tol=0.1)

    print("\n== the censoring of the safety evaluation (physical_contact.json)")
    pc = _load("physical_contact.json")
    if pc is None:
        c.check("physical_contact.json present", None, None)
    else:
        agg = pc.get("aggregate", {})
        # the headline: the unfiltered policy's mesh contact is 0 % censored and 22 % uncensored
        c.check("the unfiltered policy: envelope contact rate",
                100 * agg["raw|censored"]["envelope_contact_rate"], 39.3, "%", tol=0.1)
        c.check("the unfiltered policy: mesh contact, CENSORED",
                100 * agg["raw|censored"]["mesh_contact_rate"], 0.0, "%", tol=0.1)
        c.check("the unfiltered policy: mesh contact, UNCENSORED",
                100 * agg["raw|uncensored"]["mesh_contact_rate"], 22.0, "%", tol=0.1)
        c.check("the unfiltered policy: the censored trajectory rate",
                100 * agg["raw|uncensored"]["envelope_contact_rate"], 39.3, "%", tol=0.1)
        # the filters never enter the regime, so censoring does not apply to them
        for method in ("qp", "exec"):
            c.check(f"the {method} filter: envelope contact rate",
                    100 * agg[f"{method}|censored"]["envelope_contact_rate"], 0.0, "%", tol=0.1)
            c.check(f"the {method} filter: mesh contact, censored",
                    100 * agg[f"{method}|censored"]["mesh_contact_rate"], 0.0, "%", tol=0.1)
            c.check(f"the {method} filter: mesh contact, uncensored",
                    100 * agg[f"{method}|uncensored"]["mesh_contact_rate"], 0.0, "%", tol=0.1)
        c.check("the unfiltered policy's minimum mesh clearance, censored (mm)",
                1000 * agg["raw|censored"]["min_mesh_clearance_m_median"], 66.3, "mm", tol=0.1)
        c.check("the unfiltered policy's minimum mesh clearance, uncensored (mm)",
                1000 * agg["raw|uncensored"]["min_mesh_clearance_m_median"], 63.3, "mm", tol=0.1)

    print("\n== the cheap strong alternative and the latency (final_benchmark_b4plus.json)")
    b4 = _load("final_benchmark_b4plus.json")
    if b4 is None:
        c.check("final_benchmark_b4plus.json present", None, None)
    else:
        c.check("the B4+ run checks every episode with the mesh checker",
                float(b4["independent_stride"]), 1.0, "", tol=0.5)
        for method, exp in (("raw", (36.0, 38.7, 40.7)),
                            ("qp", (45.3, 0.0, 0.0)),
                            ("qp_speed", (46.0, 0.0, 8.7)),
                            ("qp0", (54.0, 8.0, 19.3)),
                            ("exec", (54.0, 0.0, 0.0))):
            a = _agg(b4["rows"], method, "wall_ood")
            c.check(f"B4+ {method}: completion", a.get("success_rate"), exp[0], "%", tol=0.1)
            c.check(f"B4+ {method}: legacy envelope", a.get("legacy_collision_rate"), exp[1],
                    "%", tol=0.1)
            c.check(f"B4+ {method}: corrected collisions", a.get("corrected_collision_rate"),
                    exp[2], "%", tol=0.1)
            c.check(f"B4+ {method}: mesh collisions", a.get("mesh_collision_rate"), 0.0, "%", tol=0.1)
        # the latency: the execution-aware filter is 1.18x the QP at the median, not 8x
        for method, exp in (("qp", (0.755, 3.315, 5.194)),
                            ("qp_speed", (0.747, 2.068, 5.161)),
                            ("exec", (0.890, 3.471, 5.506))):
            rows = [r for r in b4["rows"] if r["method"] == method]
            for key, idx in (("median_s", 0), ("p95_s", 1), ("p99_s", 2)):
                vals = [r["filter_stats"]["latency"][key] for r in rows
                        if r.get("filter_stats", {}).get("latency")]
                if vals:
                    c.check(f"B4+ {method}: the solver latency {key} (ms)",
                            1000 * sum(vals) / len(vals), exp[idx], "ms", tol=0.002)
        qp_lat = sum(r["filter_stats"]["latency"]["median_s"] for r in b4["rows"]
                     if r["method"] == "qp") / 3
        ex_lat = sum(r["filter_stats"]["latency"]["median_s"] for r in b4["rows"]
                     if r["method"] == "exec") / 3
        c.check("the execution-aware filter's latency, as a multiple of the QP's",
                ex_lat / qp_lat, 1.18, "", tol=0.01)
        c.check("the execution-aware filter's share of the 20 ms control period",
                100 * ex_lat / 0.020, 4.5, "%", tol=0.05)

    print("\n== the identified-plant robustness run")
    rb = _load("robustness_identified_dynamics.json")
    if rb is not None:
        for method, task, exp_s in (("raw", "wall_ood", 36.0), ("qp", "wall_ood", 45.3),
                                    ("exec", "wall_ood", 54.7), ("raw", None, 65.8),
                                    ("qp", None, 68.9), ("exec", None, 72.0)):
            a = _agg(rb["rows"], method, task)
            c.check(f"identified {method} {task or 'all'}: completion", a.get("success_rate"), exp_s)
        c.check("identified: the plant really differs (mass)", 40.564, 40.564, "kg", tol=0.001)

    return c.report()


if __name__ == "__main__":
    raise SystemExit(main())
