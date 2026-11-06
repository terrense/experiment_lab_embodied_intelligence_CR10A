#!/usr/bin/env python3
"""Safety-completion frontier of the kinematic and execution-aware filters (SIMULATION ONLY).

Figure 5 of the manuscript. Two panels, both from the same tuning-seed sweep:

  LEFT   completion against collision rate - the trade-off itself. Only the setting where the
         trade-off exists is shown (the out-of-distribution walls); in distribution every
         setting has 0 % collisions, so a collision axis would carry no information.
  RIGHT  completion against the margin - what conservatism costs, on the same sweep.

Design notes:
  * one axis per panel, no second y-axis anywhere;
  * the methods are separated by COLOUR and by MARKER SHAPE, so the figure survives colour-blind
    reading and a greyscale print. The palette was validated with evaluation/validate_palette.py
    (OKLab separation and simulated deuteranopia/protanopia/tritanopia: the smallest pair
    separation is 12.1, above the floor of 8);
  * the ablation (the execution-aware filter with the coast term removed) is NOT drawn: it lies
    exactly on the execution-aware curve, so a second line would be a duplicate. The note in the
    right panel says so instead of drawing it;
  * the figure is drawn from the 50-episode-per-cell sweeps (`tune50_*`), so every series has
    the same statistical weight; the earlier 20-episode sweeps are kept in the repository for
    traceability;
  * only the series whose collision rate actually varies is labelled in the left panel - the
    bounded QP and the execution-aware filter both sit at 0 % collisions, so their labels would
    pile up on the y-axis. The right panel gives every point's margin;
  * the unfiltered policy is a single reference POINT (it has no margin), drawn in the ink colour;
    when it falls outside the collision axis it is shown as an arrow off the plot with its numbers,
    so the axis keeps the resolution where the interesting part is;
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
    ("single", "single-constraint projection", "#efb118", "s", "-"),
    ("qp", "bounded all-pair QP", "#4269d0", "D", "-"),
    ("exec", "execution-aware (B5)", "#d64550", "^", "-"),
]
# short tags for the in-plot point labels, so two series with the same margin stay distinguishable
TAG = {"single": "single", "qp": "QP", "exec": "exec", "exec_nocoast": "exec"}
TASK_LABEL = {"wall_ood": "out-of-distribution walls (taller + wider)",
               "wall_rand": "random walls (training range)",
               "wall_obs": "the original fixed wall"}
# the task whose sweep shows a real trade-off
FRONTIER_TASK = "wall_ood"
MODE_KEY = {"qp": "qp", "single": "single", "exec": "exec", "exec_nocoast": "exec_nocoast"}


def _paths(root: Path) -> List[Path]:
    """The sweep files, preferring the 50-episode-per-cell runs over the earlier 20-episode ones.

    Both are kept in the repository: the 20-episode sweep is what the earlier write-up used and
    it is kept for traceability, but the figure is drawn from the 50-episode runs so that every
    series has the same statistical weight.
    """
    fifty = sorted(root.glob("tune50_*_s*_*.json"))
    if fifty:
        return fifty
    return sorted(root.glob("frontier_*_s*_*.json"))


def load(root: Path) -> Dict[str, Dict[str, List[dict]]]:
    """{task: {series: [sweep rows]}} from the frontier JSONs written by sweep_qp.py / sweep_exec.py."""
    out: Dict[str, Dict[str, List[dict]]] = {}
    for path in _paths(root):
        rep = json.loads(path.read_text())
        task = rep["task"]
        key = MODE_KEY.get(rep.get("mode", "qp"))
        if key is None:
            continue
        out.setdefault(task, {}).setdefault(key, []).extend(rep["rows"])
    return out


def load_raw(root: Path) -> Dict[str, List[dict]]:
    """The unfiltered-policy reference points, one per (task, policy seed)."""
    out: Dict[str, List[dict]] = {}
    for path in _paths(root):
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
    for key, label, color, marker, ls in SERIES:
        rows = mean_rows(data[task].get(key, []))
        if not rows:
            continue
        xs = [r["collision_rate"] * 100 for r in rows]
        ys = [r["success_rate"] * 100 for r in rows]
        eff = pareto(rows)
        ax1.plot(xs, ys, color=color, lw=1.4, ls=ls, zorder=2, alpha=0.9)
        # Only the series whose collision rate actually varies is labelled: the QP and the
        # execution-aware filter sit at 0 % collisions, so their points would pile up on the
        # y-axis. The right panel gives every point's margin.
        varies = (max(xs) - min(xs)) > 0.5
        label_idx = {0, len(rows) - 1} if varies else set()
        for k, (x, y, r, e) in enumerate(zip(xs, ys, rows, eff)):
            ax1.scatter([x], [y], marker=marker, s=64, facecolor=color if e else "white",
                        edgecolor=color, linewidth=1.6, zorder=3)
            if k in label_idx:
                dy = 10 if k == 0 else -15
                ax1.annotate(f"{r['margin']*100:g} cm", (x, y), textcoords="offset points",
                             xytext=(9, dy), fontsize=8, color=MUTED)
    if raw.get(task):
        rx = [r["collision_rate"] * 100 for r in raw[task]]
        ry = [r["success_rate"] * 100 for r in raw[task]]
        mx, my = float(np.mean(rx)), float(np.mean(ry))
        if mx <= 9.0:
            ax1.scatter([mx], [my], marker="o", s=64, facecolor="none", edgecolor=INK,
                        linewidth=1.6, zorder=3)
            ax1.annotate("no filter", (mx, my), textcoords="offset points",
                         xytext=(16, -20), fontsize=8.5, color=INK,
                         bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
        else:
            # the unfiltered policy is far to the right of the filtered ones; an arrow off the
            # plot keeps the interesting 0-9 % range readable, and the numbers are stated
            ax1.annotate(f"no filter: {mx:.0f} % collisions,\n{my:.0f} % completion  →",
                         xy=(8.6, my), xytext=(5.2, my - 8.0), fontsize=8.5, color=INK,
                         ha="left", va="center",
                         arrowprops=dict(arrowstyle="->", color=INK, lw=1.2))
    ax1.set_xlabel("collision rate [%]")
    ax1.set_ylabel("completion (success) [%]")
    ax1.set_title("the trade-off: " + TASK_LABEL[task], loc="left", fontweight="bold", color=INK, fontsize=10)
    ax1.set_xlim(-2.5, 9)
    ax1.set_ylim(-4, 46)
    ax1.annotate("the single constraint gets MORE dangerous\nas it grows more conservative:\n"
                 "5.3 % -> 1.3 % -> 4.7 % collisions",
                 (0.30, 0.99), xycoords="axes fraction", fontsize=8.5, color=MUTED,
                 ha="left", va="top")

    # ---- RIGHT: what conservatism costs --------------------------------------------------
    for key, label, color, marker, ls in SERIES:
        rows = mean_rows(data[task].get(key, []))
        if not rows:
            continue
        xs = [r["margin"] * 100 for r in rows]
        ys = [r["success_rate"] * 100 for r in rows]
        ax2.plot(xs, ys, color=color, lw=1.4, ls=ls, marker=marker, markersize=6,
                 markerfacecolor=color if ls == "-" else "white",
                 markeredgecolor=color if ls == "-" else color, markeredgewidth=1.4, zorder=3)
    ax2.set_xlabel("clearance margin [cm]")
    ax2.set_ylabel("completion (success) [%]")
    ax2.set_title("what a bigger margin costs", loc="left", fontweight="bold", color=INK, fontsize=10)
    ax2.set_ylim(-4, 46)
    ax2.annotate("the ablation with the coast term removed\nlies exactly on the B5 curve",
                 (0.97, 0.06), xycoords="axes fraction", fontsize=8, color=MUTED,
                 style="italic", ha="right", va="bottom")

    for ax in (ax1, ax2):
        ax.grid(color=GRID, lw=0.8)
        ax.set_axisbelow(True)

    handles = [plt.Line2D([], [], color=c, marker=m, ls=ls, lw=1.4, markersize=7,
                          markerfacecolor=c if ls == "-" else "white",
                          markeredgecolor=c, markeredgewidth=1.4, label=l)
               for _k, l, c, m, ls in SERIES]
    handles.append(plt.Line2D([], [], color=INK, marker="o", ls="", markersize=7, markerfacecolor="none",
                              markeredgecolor=INK, label="no filter"))
    fig.legend(handles=handles, frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.08),
               fontsize=9)
    fig.suptitle("A fixed margin buys safety and costs completion.\n"
                 "CR10A MuJoCo, tuning seeds 0-19, 3 policy seeds, 50 episodes per cell (SIMULATED)",
                 x=0.01, ha="left", color=INK, fontweight="bold", fontsize=10)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=120, bbox_inches="tight")
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
