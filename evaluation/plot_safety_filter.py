#!/usr/bin/env python3
"""M5 safety-filter figure: does a safety filter rescue the learned policy out of distribution? (SIMULATED)

Three settings, same 50 paired episodes (seeds 1000-1049), generalist PPO (trained on random observed walls):
  original wall | new random walls (training range) | out-of-distribution walls (taller + wider)
Three ways of running that policy:
  raw           - the policy alone
  + repulsion   - the naive always-on potential-field repulsion added to the action (d0 0.10, k 0.02)
  + CBF         - a single-constraint control-barrier projection (margin 0.05, alpha 5)
PPO bars = mean over seeds 0/1/2, dots = individual seeds.

    python3 evaluation/plot_safety_filter.py data/reports/learning --out assets/learning/safety_filter.png
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
SUCCESS, COLLISION = "#2a9d5c", "#d4562a"
SETTINGS = [("wall_obs", "original wall"), ("wall_rand", "new random walls\n(training range)"),
            ("wall_ood", "out-of-distribution walls\n(taller + wider)")]
METHODS = [("raw", "policy alone (raw)"), ("additive", "policy + repulsion\n(naive filter)"),
           ("cbf", "policy + CBF\n(safety filter)")]


def rates(path: Path):
    r = json.loads(path.read_text())
    return r["success_rate"], r["collision_rate"]


def collect(root: Path):
    g = root / "gen"
    s = root / "gen_safe"
    out = {}
    for key, label in METHODS:
        runs = {}
        for task, _ in SETTINGS:
            if key == "raw":
                pat = str(g / "ppo_wall_rand_s{}_on_{}.json")
            else:
                pat = str(s / ("ppo_wall_rand_s{}_on_{}_" + key + ".json"))
            runs[task] = [rates(Path(pat.format(k, task))) for k in range(3) if Path(pat.format(k, task)).exists()]
        out[label] = runs
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    data = collect(a.root)
    methods = list(data)
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": MUTED, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), sharey=True)
    y = np.arange(len(methods))[::-1]
    for ax, (task, title) in zip(axes, SETTINGS):
        for yi, m in zip(y, methods):
            runs = np.array(data[m][task]) * 100
            s, c = runs[:, 0].mean(), runs[:, 1].mean()
            ax.barh(yi + 0.17, s, height=0.32, color=SUCCESS, edgecolor="white", linewidth=1.5)
            ax.barh(yi - 0.17, c, height=0.32, color=COLLISION, edgecolor="white", linewidth=1.5)
            ax.scatter(runs[:, 0], np.full(len(runs), yi + 0.17), s=14, color=INK, zorder=3)
            ax.scatter(runs[:, 1], np.full(len(runs), yi - 0.17), s=14, color=INK, zorder=3)
            ax.text(max(s, runs[:, 0].max()) + 2.5, yi + 0.17, f"{s:.0f}%", va="center", fontsize=8.5, color=INK)
            ax.text(max(c, runs[:, 1].max()) + 2.5, yi - 0.17, f"{c:.0f}%", va="center", fontsize=8.5, color=INK)
        ax.set_xlim(0, 105)
        ax.set_title(title, loc="left", fontweight="bold", color=INK, fontsize=10)
        ax.grid(axis="x", color=GRID, lw=0.8)
        ax.set_xlabel("evaluation episodes [%]")
    axes[0].set_yticks(y)
    axes[0].set_yticklabels(methods)
    handles = [plt.Rectangle((0, 0), 1, 1, color=SUCCESS), plt.Rectangle((0, 0), 1, 1, color=COLLISION),
               plt.Line2D([], [], marker="o", ls="", color=INK, markersize=4)]
    fig.legend(handles, ["success", "collision", "individual PPO seeds"], frameon=False, ncol=3, loc="lower center",
               bbox_to_anchor=(0.55, -0.04))
    fig.suptitle("A safety filter (CBF) removes out-of-distribution collisions without hurting success.  "
                 "CR10A MuJoCo, 50 paired episodes (SIMULATED)",
                 x=0.01, ha="left", color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=120, bbox_inches="tight")
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
