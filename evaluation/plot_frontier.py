#!/usr/bin/env python3
"""Safety-completion frontier of the kinematic filters (SIMULATION ONLY).

Figure 5 of the manuscript. Two panels, both from the same tuning-seed sweep:

  LEFT   completion against collision rate - the trade-off itself. Only the setting where the
         trade-off exists is shown (the out-of-distribution walls); in distribution every
         setting has 0 % collisions, so a collision axis would carry no information.
  RIGHT  completion against the margin - what conservatism costs, on the same sweep.

Design notes:
  * one axis per panel, no second y-axis anywhere;
  * the filters are separated by COLOUR and by MARKER SHAPE, so the figure survives colour-blind
    reading and a greyscale print;
  * the margin labels are placed alternately above and below the points so they cannot collide;
  * the unfiltered policy is a single reference POINT (it has no margin), drawn in the ink colour;
  * the grid and the axes are recessive, the ink is not.

    python3 evaluation/plot_frontier.py data/reports/learning --out assets/learning/frontier.png
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
# categorical, in fixed order; each series also gets its own marker shape
SERIES = [
    ("single", "single-constraint projection", "#efb118", "s"),
    ("qp", "bounded all-pair QP", "#4269d0", "D"),
]
TASK_LABEL = {"wall_ood": "out-of-distribution walls (taller + wider)",
               "wall_rand": "random walls (training range)",
               "wall_obs": "the original fixed wall"}
# the task whose sweep shows a real trade-off
FRONTIER_TASK = "wall_ood"


def load(root: Path) -> Dict[str, Dict[str, List[dict]]]:
    """{task: {series: [sweep rows]}} from the frontier JSONs written by sweep_qp.py."""
    out: Dict[str, Dict[str, List[dict]]] = {}
    for path in sorted(root.glob("frontier_*_s*_*.json")):
        rep = json.loads(path.read_text())
        task = rep["task"]
        key = "qp" if rep.get("mode", "qp") == "qp" else "single"
        out.setdefault(task, {}).setdefault(key, []).extend(rep["rows"])
    return out


def load_raw(root: Path) -> Dict[str, List[dict]]:
    """The unfiltered-policy reference points, one per (task, policy seed)."""
    out: Dict[str, List[dict]] = {}
    for path in sorted(root.glob("frontier_*_s*_*.json")):
        rep = json.loads(path.read_text())
        if "raw" in rep:
            out.setdefault(rep["task"], []).append(rep["raw"])
    return out


def mean_rows(rows: List[dict]) -> List[dict]:
    """Average the per-seed rows that share a margin."""
    by_margin: Dict[float, List[dict]] = {}
    for r in rows:
        by_margin.setdefault(round(r["margin"], 4), []).append(r)
    out = []
    for m, rs in sorted(by_margin.items()):
        out.append({"margin": m,
                    "success_rate": float(np.mean([x["success_rate"] for x in rs])),
                    "collision_rate": float(np.mean([x["collision_rate"] for x in rs])),
                    "n_seeds": len(rs)})
    return out


def pareto(points: List[dict]) -> List[bool]:
    """A point is Pareto-efficient if no other point has both a higher completion and a lower
    collision rate."""
    flags = []
    for i, p in enumerate(points):
        ok = True
        for j, q in enumerate(points):
            if i == j:
                continue
            if (q["success_rate"] >= p["success_rate"] and q["collision_rate"] <= p["collision_rate"]
                    and (q["success_rate"] > p["success_rate"] or q["collision_rate"] < p["collision_rate"])):
                ok = False
                break
        flags.append(ok)
    return flags


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)

    data = load(a.root)
    raw = load_raw(a.root)
    if FRONTIER_TASK not in data:
        print(f"no frontier data for {FRONTIER_TASK} in {a.root}")
        return 1

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED,
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.2, 4.6))

    # ---- LEFT: the trade-off -------------------------------------------------------------
    task = FRONTIER_TASK
    for key, label, color, marker in SERIES:
        rows = mean_rows(data[task].get(key, []))
        if not rows:
            continue
        xs = [r["collision_rate"] * 100 for r in rows]
        ys = [r["success_rate"] * 100 for r in rows]
        eff = pareto(rows)
        ax1.plot(xs, ys, color=color, lw=1.4, zorder=2, alpha=0.9)
        for k, (x, y, r, e) in enumerate(zip(xs, ys, rows, eff)):
            ax1.scatter([x], [y], marker=marker, s=64, facecolor=color if e else "white",
                        edgecolor=color, linewidth=1.6, zorder=3)
            dy = 9 if k % 2 == 0 else -14
            ax1.annotate(f"{r['margin']*100:.0f} cm", (x, y), textcoords="offset points",
                         xytext=(8, dy), fontsize=8, color=MUTED)
    if raw.get(task):
        rx = [r["collision_rate"] * 100 for r in raw[task]]
        ry = [r["success_rate"] * 100 for r in raw[task]]
        ax1.scatter(rx, ry, marker="o", s=64, facecolor="none", edgecolor=INK, linewidth=1.6, zorder=3)
        ax1.annotate("no filter", (np.mean(rx), np.mean(ry)), textcoords="offset points",
                     xytext=(10, -4), fontsize=8.5, color=INK)
    ax1.set_xlabel("collision rate [%]")
    ax1.set_ylabel("completion (success) [%]")
    ax1.set_title("the trade-off: " + TASK_LABEL[task], loc="left", fontweight="bold", color=INK, fontsize=10)
    ax1.set_xlim(-4, 26)
    ax1.set_ylim(-4, 52)

    # ---- RIGHT: what conservatism costs --------------------------------------------------
    for key, label, color, marker in SERIES:
        rows = mean_rows(data[task].get(key, []))
        if not rows:
            continue
        xs = [r["margin"] * 100 for r in rows]
        ys = [r["success_rate"] * 100 for r in rows]
        ax2.plot(xs, ys, color=color, lw=1.4, marker=marker, markersize=6, markerfacecolor=color,
                 markeredgecolor="white", markeredgewidth=1.2, zorder=3)
    ax2.set_xlabel("clearance margin [cm]")
    ax2.set_ylabel("completion (success) [%]")
    ax2.set_title("what a bigger margin costs", loc="left", fontweight="bold", color=INK, fontsize=10)
    ax2.set_ylim(-4, 52)

    for ax in (ax1, ax2):
        ax.grid(color=GRID, lw=0.8)
        ax.set_axisbelow(True)

    handles = [plt.Line2D([], [], color=c, marker=m, ls="-", lw=1.4, markersize=7, markerfacecolor=c,
                          markeredgecolor="white", label=l) for _k, l, c, m in SERIES]
    handles.append(plt.Line2D([], [], color=INK, marker="o", ls="", markersize=7, markerfacecolor="none",
                              markeredgecolor=INK, label="no filter"))
    fig.legend(handles=handles, frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.06),
               fontsize=9)
    fig.suptitle("A fixed margin buys safety and costs completion.  CR10A MuJoCo, tuning seeds 0-19, "
                 "3 policy seeds (SIMULATED)", x=0.01, ha="left", color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=120, bbox_inches="tight")
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
