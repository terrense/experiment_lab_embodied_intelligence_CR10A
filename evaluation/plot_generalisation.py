#!/usr/bin/env python3
"""M5 generalisation figure: who still avoids the wall when the wall changes? (SIMULATED)

Three settings, same 50 evaluation episodes per setting for every method (paired):
  original wall | new random walls (training ranges) | out-of-distribution walls (taller + wider)
Methods: DLS (straight line), APF (potential field), PPO trained on the ONE fixed wall (wall not
observed), PPO trained on random walls (wall observed). PPO bars = mean over seeds, dots = seeds.

    python3 evaluation/plot_generalisation.py data/reports/learning --out assets/learning/generalisation.png
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
SETTINGS = [("original", "original wall"), ("rand", "new random walls\n(training range)"), ("ood", "out-of-distribution walls\n(taller + wider)")]


def rates(path: Path):
    r = json.loads(path.read_text())
    if "per_episode" not in r:
        raise ValueError(path)
    return r["success_rate"], r["collision_rate"]


def collect(root: Path):
    g = root / "gen"
    base = {"original": json.loads((root / "baselines_wall.json").read_text()),
            "rand": json.loads((g / "baselines_wall_rand.json").read_text()),
            "ood": json.loads((g / "baselines_wall_ood.json").read_text())}
    out = {}
    for name, key in (("DLS straight line", "dls_ik"), ("APF potential field", "apf")):
        out[name] = {s: [(base[s][key]["success_rate"], base[s][key]["collision_rate"])] for s, _ in SETTINGS}
    spec = {"original": "ppo_wall_s{}_on_wall.json", "rand": "ppo_wall_s{}_on_wall_rand_blind.json", "ood": "ppo_wall_s{}_on_wall_ood_blind.json"}
    gen = {"original": "ppo_wall_rand_s{}_on_wall_obs.json", "rand": "ppo_wall_rand_s{}_on_wall_rand.json", "ood": "ppo_wall_rand_s{}_on_wall_ood.json"}
    for name, pat in (("PPO, trained on ONE wall\n(wall not observed)", spec), ("PPO, trained on RANDOM walls\n(wall observed)", gen)):
        out[name] = {s: [rates(g / pat[s].format(k)) for k in range(5) if (g / pat[s].format(k)).exists()] for s, _ in SETTINGS}
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
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), sharey=True)
    y = np.arange(len(methods))[::-1]
    for ax, (key, title) in zip(axes, SETTINGS):
        for yi, m in zip(y, methods):
            runs = np.array(data[m][key]) * 100
            s, c = runs[:, 0].mean(), runs[:, 1].mean()
            ax.barh(yi + 0.17, s, height=0.32, color=SUCCESS, edgecolor="white", linewidth=1.5)
            ax.barh(yi - 0.17, c, height=0.32, color=COLLISION, edgecolor="white", linewidth=1.5)
            if len(runs) > 1:
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
    fig.legend(handles, ["success", "collision", "individual PPO seeds"], frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(0.55, -0.04))
    fig.suptitle("Does it still avoid the wall when the wall changes?  CR10A MuJoCo, 50 paired episodes per setting (SIMULATED)",
                 x=0.01, ha="left", color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(a.out, dpi=120, bbox_inches="tight")
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
