#!/usr/bin/env python3
"""Figure: the STOPPING TAIL (SIMULATION ONLY).

Figure 7 of the manuscript: what the filter still does not reason about.

  LEFT   how far the arm keeps travelling after the command is cut to zero, as a function of the
         time since the cut. The filter's whole look-ahead is 100 ms, marked on the axis: the
         motion it reasons about is a fraction of the motion that happens.
  RIGHT  how much clearance the arm loses during that tail, against the filter's own 10 mm
         margin. The MEDIAN state does not lose clearance - most of the sampled states are far
         from the wall and the coast moves the arm away from it. The erosion is in the third of
         the states that are close to it, which is why the worst case is 87 mm.

One entity (the stopping tail), so one hue: the median, the 95th percentile and the worst case
are separated by line style and by direct labels, not by colour. The reference lines - the
filter's horizon and its margin - are in the recessive ink.

    python3 evaluation/plot_stopping_tail.py data/reports/learning/stopping_rollout.json \\
        --out assets/learning/stopping_tail.png
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
ACCENT = "#efb118"          # the validated single-series hue
REF = "#59636e"             # the recessive ink, for the filter's horizon and margin


def series(tails, stat):
    """(delays in ms, median, p95, worst) for one statistic of the coast profile."""
    delays = sorted({float(k[:-2]) for t in tails for k in t["coast_profile"]})
    med, p95, worst = [], [], []
    for d in delays:
        key = f"{int(d)}ms"
        vals = [t["coast_profile"][key][stat] for t in tails
                if key in t["coast_profile"] and stat in t["coast_profile"][key]]
        med.append(float(np.median(vals)))
        p95.append(float(np.percentile(vals, 95)))
        worst.append(float(np.max(vals)))
    return np.array(delays), np.array(med), np.array(p95), np.array(worst)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("report", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--horizon-ms", type=float, default=100.0,
                    help="the filter's look-ahead horizon [ms]")
    ap.add_argument("--margin-mm", type=float, default=10.0,
                    help="the filter's clearance margin [mm]")
    a = ap.parse_args(argv)

    rep = json.loads(a.report.read_text())
    tails = rep["rows"]["tails"]
    if not tails:
        print("no rows in", a.report)
        return 1
    worse = sum(1 for t in tails if t["tail_approaches_obstacle"])
    rate = 100.0 * worse / len(tails)

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED,
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.6, 4.6))

    # ---------------------------------------------------------------- LEFT: the travel
    x, med, p95, worst = series(tails, "coast_tcp_mm")
    ax1.plot(x, med, color=ACCENT, lw=2.2, solid_capstyle="round", zorder=4)
    ax1.plot(x, p95, color=ACCENT, lw=1.5, ls="--", zorder=3)
    ax1.fill_between(x, 0, p95, color=ACCENT, alpha=0.10, lw=0, zorder=1)
    i100 = int(np.argmin(np.abs(x - a.horizon_ms)))
    ax1.plot([x[i100]], [p95[i100]], marker="o", ms=7, color=ACCENT, zorder=5,
             markeredgecolor="white", markeredgewidth=1.4)
    ax1.annotate(f"at the filter's horizon:\n{p95[i100]:.0f} mm (95th pct),\n"
                 f"{worst[i100]:.0f} mm (worst)",
                 (x[i100], p95[i100]), textcoords="offset points", xytext=(14, 26),
                 fontsize=8.5, color=INK, va="bottom",
                 arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8, alpha=0.7, shrinkA=2, shrinkB=6))
    ax1.annotate("median", (x[-1], med[-1]), textcoords="offset points", xytext=(-6, 10),
                 ha="right", fontsize=8.5, color=INK, fontweight="bold")
    ax1.annotate("95th percentile", (x[-1], p95[-1]), textcoords="offset points", xytext=(-6, -14),
                 ha="right", fontsize=8.5, color=MUTED)
    ax1.axvline(a.horizon_ms, color=REF, lw=1.2, ls=":", zorder=2)
    ax1.annotate(f"the filter's whole look-ahead ({a.horizon_ms:.0f} ms)",
                 (a.horizon_ms, 8), textcoords="offset points", xytext=(-7, 0),
                 fontsize=8.5, color=REF, ha="right", va="bottom", rotation=90)
    ax1.set_xlabel("time since the command was cut [ms]")
    ax1.set_ylabel("tool-point travel since the cut [mm]")
    ax1.set_title("a zero command does not stop the motion", loc="left",
                  fontweight="bold", color=INK, fontsize=10)
    ax1.set_xlim(0, x[-1] * 1.03)
    ax1.set_ylim(0, max(p95.max(), worst.max()) * 1.12)
    ax1.grid(color=GRID, lw=0.8)
    ax1.set_axisbelow(True)

    # --------------------------------------------------------------- RIGHT: the erosion
    x, med, p95, worst = series(tails, "clearance_loss_mm")
    ax2.plot(x, med, color=ACCENT, lw=2.2, solid_capstyle="round", zorder=4)
    ax2.plot(x, p95, color=ACCENT, lw=1.5, ls="--", zorder=3)
    ax2.plot(x, worst, color=ACCENT, lw=1.3, ls=":", zorder=3)
    ax2.fill_between(x, med, p95, color=ACCENT, alpha=0.12, lw=0, zorder=1)
    ax2.axhline(0.0, color=GRID, lw=1.0, zorder=2)
    ax2.axhline(a.margin_mm, color=REF, lw=1.2, ls=":", zorder=2)
    ax2.annotate(f"the filter's own margin ({a.margin_mm:.0f} mm)",
                 (x[-1], a.margin_mm), textcoords="offset points", xytext=(-6, 6),
                 ha="right", fontsize=8.5, color=REF)
    ax2.annotate("median", (x[-1], med[-1]), textcoords="offset points", xytext=(-6, -14),
                 ha="right", fontsize=8.5, color=INK, fontweight="bold")
    ax2.annotate("95th percentile", (x[-1], p95[-1]), textcoords="offset points", xytext=(-6, 13),
                 ha="right", fontsize=8.5, color=MUTED)
    ax2.annotate("worst", (x[-1], worst[-1]), textcoords="offset points", xytext=(-6, 10),
                 ha="right", fontsize=8.5, color=MUTED)
    ax2.axvline(a.horizon_ms, color=REF, lw=1.2, ls=":", zorder=2)
    ax2.annotate(f"in {rate:.0f} % of the states the tail brings the arm CLOSER to the wall;\n"
                 f"the median state is far from it and moves away",
                 (0.03, 0.64), xycoords="axes fraction", fontsize=8.5, color=INK, va="top")
    ax2.set_xlabel("time since the command was cut [ms]")
    ax2.set_ylabel("clearance lost since the cut [mm]")
    ax2.set_title("and the tail eats the margin", loc="left",
                  fontweight="bold", color=INK, fontsize=10)
    ax2.set_xlim(0, x[-1] * 1.03)
    ax2.set_ylim(min(med.min(), -5) * 1.25, max(worst.max(), a.margin_mm) * 1.12)
    ax2.grid(color=GRID, lw=0.8)
    ax2.set_axisbelow(True)

    fig.suptitle("The filter reasons over 100 ms; the motion it has to cover lasts 200–500 ms, "
                 "and the tail erases up to 87 mm of clearance.",
                 x=0.012, ha="left", fontsize=11, color=INK, fontweight="bold")
    fig.text(0.012, 0.925, "CR10A MuJoCo, states from the execution-aware filter's own output.  "
             "SIMULATION ONLY - no real robot was connected.",
             fontsize=8.5, color=MUTED, ha="left")
    fig.text(0.012, 0.015, "216 states, three policy seeds, 50 episodes each; the command is cut "
             "to zero and the plant is rolled on with every physics substep recorded.",
             fontsize=8.5, color=MUTED, ha="left")
    fig.tight_layout(rect=(0, 0.04, 1, 0.90))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=170)
    print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
