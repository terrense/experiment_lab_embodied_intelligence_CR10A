#!/usr/bin/env python3
"""M5 figure: PPO learning curves (all seeds) + evaluation outcome vs the classical baselines.

    python3 evaluation/plot_learning.py data/reports/learning --out assets/learning/wall_task.png
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

INK, MUTED, GRID = "#1f2328", "#59636e", "#d8dee4"
SEED_COLORS = ["#2a7bd4", "#d4562a", "#2a9d5c", "#8a5cd4"]
OUTCOME = {"success": "#2a9d5c", "missed": "#c9a227", "collision": "#d4562a"}


def curve(run: Path):
    rows = []
    for f in run.glob("monitor_*.monitor.csv"):
        with open(f) as fh:
            next(fh)
            for r in csv.DictReader(fh):
                rows.append((float(r["t"]), float(r["r"]), int(r["l"])))
    rows.sort()
    steps = np.cumsum([l for _, _, l in rows])
    ret = np.array([r for _, r, _ in rows])
    k = max(1, len(ret) // 60)
    sm = np.convolve(ret, np.ones(k) / k, mode="valid")
    return steps[k - 1:], sm


def outcome(rep):
    eps = rep["per_episode"]
    n = len(eps)
    s = sum(e["success"] for e in eps) / n
    c = sum(e.get("collision", False) for e in eps) / n
    return s, 1 - s - c, c


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    runs = sorted(p for p in a.root.glob("ppo_wall_s[0-9]") if (p / "eval.json").exists())
    base = json.loads((a.root / "baselines_wall.json").read_text())

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": MUTED, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw={"width_ratios": [1.15, 1]})
    for i, run in enumerate(runs):
        x, y = curve(run)
        ax1.plot(x / 1e6, y, color=SEED_COLORS[i % 4], lw=2, label=run.name.replace("ppo_wall_s", "PPO seed "))
    ax1.axhline(0, color=MUTED, lw=1, ls=":")
    ax1.text(0.02, 0.3, "standing still ~ 0", color=MUTED, fontsize=8, transform=ax1.get_yaxis_transform())
    ax1.set_xlabel("environment steps [million]")
    ax1.set_ylabel("episode return (smoothed)")
    ax1.grid(axis="y", color=GRID, lw=0.8)
    ax1.legend(frameon=False, loc="lower right")
    ax1.set_title("PPO learning curves", loc="left", fontweight="bold", color=INK)

    labels, data = [], []
    for name, key in (("random", "random"), ("DLS straight line", "dls_ik"), ("APF potential field", "apf")):
        labels.append(name)
        data.append(outcome(base[key]))
    for run in runs:
        labels.append(run.name.replace("ppo_wall_s", "PPO seed "))
        data.append(outcome(json.loads((run / "eval.json").read_text())))
    y = np.arange(len(labels))[::-1]
    left = np.zeros(len(labels))
    for j, (k, col) in enumerate(OUTCOME.items()):
        vals = np.array([d[j] for d in data]) * 100
        ax2.barh(y, vals, left=left, color=col, height=0.62, edgecolor="white", linewidth=2, label=k)
        for yi, v, l0 in zip(y, vals, left):
            if v >= 8:
                ax2.text(l0 + v / 2, yi, f"{v:.0f}%", ha="center", va="center", color="white", fontsize=9, fontweight="bold")
        left += vals
    ax2.set_yticks(y)
    ax2.set_yticklabels(labels)
    ax2.set_xlim(0, 100)
    ax2.set_xlabel("evaluation episodes [%] (50 episodes, seeds 1000-1049)")
    ax2.legend(frameon=False, ncol=3, loc="upper center", bbox_to_anchor=(0.5, -0.16))
    ax2.set_title("Outcome on the wall task", loc="left", fontweight="bold", color=INK)
    fig.suptitle("Reach around the wall, CR10A MuJoCo plant (SIMULATED)", x=0.01, ha="left", color=INK, fontweight="bold")
    fig.tight_layout()
    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=120, bbox_inches="tight")
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
