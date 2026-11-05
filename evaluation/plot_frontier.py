#!/usr/bin/env python3
"""Safety-completion frontier of the kinematic filters (SIMULATION ONLY).

Figure 5 of the manuscript: for each filter, sweep its margin on the TUNING seeds and plot
completion against collision rate. The point of the figure is not "which curve is highest" but
"where does the method sit on the trade-off, and what does a more conservative setting cost".

Design notes (the chart must be readable without the caption):
  * one axis per panel - completion against collision rate; the latency is written next to the
    points instead of being a second y-axis;
  * the three filters are separated by COLOUR and by MARKER SHAPE, so the figure survives a
    colour-blind reader and a greyscale print;
  * every point is labelled with its margin, so the sweep is readable without a legend lookup;
  * the Pareto-efficient points of each curve are drawn filled, the dominated ones hollow;
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
    ("raw", "policy alone (no filter)", "#ff725c", "o"),
    ("single", "single-constraint projection", "#efb118", "s"),
    ("qp", "bounded all-pair QP", "#4269d0", "D"),
]
TASK_LABEL = {"wall_ood": "out-of-distribution walls\n(taller + wider)",
               "wall_rand": "random walls\n(training range)",
               "wall_obs": "the original fixed wall"}


def load(root: Path) -> Dict[str, Dict[str, List[dict]]]:
    """{task: {series: [sweep rows]}} from the frontier JSONs written by sweep_qp.py."""
    out: Dict[str, Dict[str, List[dict]]] = {}
    for path in sorted(root.glob("frontier_*_s*_*.json")):
        rep = json.loads(path.read_text())
        task = rep["task"]
        mode = rep.get("mode", "qp")
        key = "qp" if mode == "qp" else "single"
        out.setdefault(task, {}).setdefault(key, []).extend(rep["rows"])
    return out


def pareto(points: List[dict]) -> List[bool]:
    """A point is Pareto-efficient if no other point has both a higher completion and a lower
    collision rate (ties broken by the margin)."""
    flags = []
    for i, p in enumerate(points):
        ok = True
        for j, q in enumerate(points):
            if i == j:
                continue
            better = (q["success_rate"] >= p["success_rate"] and q["collision_rate"] <= p["collision_rate"]
                      and (q["success_rate"] > p["success_rate"] or q["collision_rate"] < p["collision_rate"]))
            if better:
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
    if not data:
        print("no frontier JSONs found in", a.root)
        return 1
    tasks = [t for t in ("wall_ood", "wall_rand", "wall_obs") if t in data]

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED,
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, len(tasks), figsize=(4.6 * len(tasks), 4.4), sharey=True)
    axes = np.atleast_1d(axes)

    for ax, task in zip(axes, tasks):
        for key, label, color, marker in SERIES:
            rows = data[task].get(key)
            if not rows:
                continue
            rows = sorted(rows, key=lambda r: r["margin"])
            xs = [r["collision_rate"] * 100 for r in rows]
            ys = [r["success_rate"] * 100 for r in rows]
            eff = pareto(rows)
            ax.plot(xs, ys, color=color, lw=1.4, zorder=2, alpha=0.9)
            for x, y, r, e in zip(xs, ys, rows, eff):
                ax.scatter([x], [y], marker=marker, s=64, facecolor=color if e else "white",
                           edgecolor=color, linewidth=1.6, zorder=3)
                ax.annotate(f"{r['margin']*100:.0f} cm", (x, y), textcoords="offset points",
                            xytext=(7, 5), fontsize=7.5, color=MUTED)
        ax.set_xlim(-3, 62)
        ax.set_ylim(-4, 62)
        ax.grid(color=GRID, lw=0.8)
        ax.set_xlabel("collision rate [%]")
        ax.set_title(TASK_LABEL.get(task, task), loc="left", fontweight="bold", color=INK, fontsize=10)
    axes[0].set_ylabel("completion (success) [%]")

    handles = [plt.Line2D([], [], color=c, marker=m, ls="", markersize=7, markerfacecolor=c,
                          markeredgecolor=c, label=l) for _k, l, c, m in SERIES]
    fig.legend(handles=handles, frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.04),
               fontsize=9)
    fig.suptitle("A fixed margin buys safety and costs completion.  CR10A MuJoCo, tuning seeds 0-19 "
                 "(SIMULATED)", x=0.01, ha="left", color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=120, bbox_inches="tight")
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
