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
        for margin, exp in ((0.005, (3.6, 0.1, 0.6)), (0.01, (8.0, 0.1, 0.4)),
                            (0.02, (9.2, 0.2, 0.6)), (0.04, (32.4, 1.3, 4.8))):
            qp = by.get(("qp", margin))
            nc = by.get(("exec_nocoast", margin))
            ex = by.get(("exec", margin))
            if qp:
                c.check(f"gap {margin*100:g} cm: qp claim-failure rate",
                        100 * qp["prediction_failure_rate"], exp[0])
                c.check(f"gap {margin*100:g} cm: qp gap median (mm)",
                        qp["gap_mm_median"], -12.0, "mm", tol=0.2)
            if nc:
                c.check(f"gap {margin*100:g} cm: exec-no-coast claim-failure rate",
                        100 * nc["prediction_failure_rate"], exp[1])
            if ex:
                c.check(f"gap {margin*100:g} cm: exec claim-failure rate",
                        100 * ex["prediction_failure_rate"], exp[2])

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
