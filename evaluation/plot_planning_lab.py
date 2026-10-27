#!/usr/bin/env python3
"""Figures for the M2 planner benchmark (SIMULATED data).

  python3 evaluation/plot_planning_lab.py data/reports/planning_lab/run01 --out assets/m2

Writes: success_rate.png, planning_time.png, path_length.png, clearance.png, tcp_paths_<scene>.png
Palette: reference data-viz palette (light), categorical slots 1-3 only (validated all-pairs);
magnitude = single blue ramp. Text in neutral inks, never in series colors.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

SURFACE, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]  # categorical slots 1-3
BLUE = "#2a78d6"
BLUE_RAMP = LinearSegmentedColormap.from_list("blue", ["#eef4fc", "#bcd5f4", "#6fa6e8", "#2a78d6", "#174a8c"])

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
    "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.axisbelow": True,
})


def load(run: Path):
    rows = list(csv.DictReader(open(run / "results.csv")))
    for r in rows:
        r["success"] = int(r["success"])
        for k in ("planning_time_s", "joint_path_rad", "tcp_path_m", "duration_s", "clearance_lb_m", "clearance_est_m"):
            r[k] = float(r[k]) if r.get(k) not in (None, "") else math.nan
    summary = json.loads((run / "summary.json").read_text())
    paths = json.loads((run / "paths.json").read_text()) if (run / "paths.json").exists() else {}
    return rows, summary, paths


def ordered(values):
    seen = []
    for v in values:
        if v not in seen:
            seen.append(v)
    return seen


def fig_success(rows, scenes, planners, out):
    M = np.zeros((len(planners), len(scenes)))
    N = np.zeros_like(M)
    for r in rows:
        i, j = planners.index(r["planner"]), scenes.index(r["scene"])
        M[i, j] += r["success"]
        N[i, j] += 1
    rate = np.divide(M, N, out=np.zeros_like(M), where=N > 0)
    fig, ax = plt.subplots(figsize=(1.6 + 1.3 * len(scenes), 0.42 * len(planners) + 1.4))
    ax.imshow(rate, cmap=BLUE_RAMP, vmin=0, vmax=1, aspect="auto")
    ax.grid(False)
    ax.set_xticks(range(len(scenes)), scenes)
    ax.set_yticks(range(len(planners)), planners)
    for i in range(len(planners)):
        for j in range(len(scenes)):
            ax.text(j, i, f"{int(M[i, j])}/{int(N[i, j])}", ha="center", va="center", fontsize=9,
                    color="#ffffff" if rate[i, j] > 0.6 else INK)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title("Planning success (successes / attempts)", loc="left")
    fig.tight_layout()
    fig.savefig(out / "success_rate.png", dpi=160)
    plt.close(fig)


def fig_strip(rows, scenes, planners, key, title, xlabel, fname, out, log=False):
    fig, axes = plt.subplots(1, len(scenes), figsize=(3.6 * len(scenes), 0.38 * len(planners) + 1.6), sharey=True)
    axes = np.atleast_1d(axes)
    rng = np.random.default_rng(0)
    for ax, sc in zip(axes, scenes):
        for i, p in enumerate(planners):
            v = np.array([r[key] for r in rows if r["scene"] == sc and r["planner"] == p and r["success"]])
            v = v[np.isfinite(v)]
            if len(v) == 0:
                ax.text(0.98, i, "no success", ha="right", va="center", fontsize=8, color=MUTED,
                        transform=ax.get_yaxis_transform())
                continue
            ax.scatter(v, i + rng.uniform(-0.15, 0.15, len(v)), s=16, color=BLUE, alpha=0.55, linewidths=0)
            ax.plot([np.median(v)] * 2, [i - 0.3, i + 0.3], color=INK, lw=2, solid_capstyle="round")
        ax.set_title(sc, loc="left")
        ax.set_xlabel(xlabel)
        if log:
            ax.set_xscale("log")
        ax.grid(axis="y", visible=False)
    axes[0].set_yticks(range(len(planners)), planners)
    axes[0].invert_yaxis()
    fig.suptitle(title + "  (dots = trials, bar = median)", x=0.01, ha="left", fontweight="bold", color=INK)
    fig.tight_layout()
    fig.savefig(out / fname, dpi=160)
    plt.close(fig)


def fig_clearance(rows, scenes, planners, bound, out):
    fig, axes = plt.subplots(1, len(scenes), figsize=(3.6 * len(scenes), 0.38 * len(planners) + 1.8), sharey=True)
    axes = np.atleast_1d(axes)
    for ax, sc in zip(axes, scenes):
        for i, p in enumerate(planners):
            ok = [r for r in rows if r["scene"] == sc and r["planner"] == p and r["success"]]
            if not ok:
                ax.text(0.98, i, "no success", ha="right", va="center", fontsize=8, color=MUTED,
                        transform=ax.get_yaxis_transform())
                continue
            lb = np.median([r["clearance_lb_m"] for r in ok]) * 1000
            est = np.median([r["clearance_est_m"] for r in ok]) * 1000
            ax.plot([lb, est], [i, i], color="#bcd5f4", lw=6, solid_capstyle="round")
            ax.scatter([est], [i], s=36, color=BLUE, zorder=3, edgecolors=SURFACE, linewidths=2)
            ax.scatter([lb], [i], s=36, color=SURFACE, zorder=3, edgecolors=BLUE, linewidths=1.5)
        ax.axvline(0, color=MUTED, lw=1)
        ax.set_title(sc, loc="left")
        ax.set_xlabel("min distance to obstacles (mm)")
        ax.grid(axis="y", visible=False)
    axes[0].set_yticks(range(len(planners)), planners)
    axes[0].invert_yaxis()
    fig.suptitle(f"Path clearance (median over successful trials)\nfilled = mesh-surface estimate "
                 f"(>= truth, <= truth + {bound * 1000:.0f} mm)   hollow = capsule lower bound (<= truth)",
                 x=0.01, ha="left", fontweight="bold", color=INK, fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "clearance.png", dpi=160)
    plt.close(fig)


def draw_box(ax, lo, hi, color=MUTED, alpha=0.18):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    V = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])
    F = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4]]
    ax.add_collection3d(Poly3DCollection([V[f] for f in F], facecolors=color, edgecolors="#b5b4ae", linewidths=0.4, alpha=alpha))


def fig_paths(paths, scene_yaml: Path, scene: str, query: str, planners, out):
    import yaml
    d = yaml.safe_load(scene_yaml.read_text())
    fig = plt.figure(figsize=(6.4, 5.4))
    ax = fig.add_subplot(projection="3d")
    for b in d["boxes"]:
        if b["name"] == "table":
            continue
        c, s = np.array(b["center"]), np.array(b["size"])
        draw_box(ax, c - s / 2, c + s / 2)
    shown = 0
    for p in planners:
        key = f"{scene}/{query}/{p}"
        if key not in paths or shown >= 3:
            continue
        P = np.array(paths[key])
        ax.plot(P[:, 0], P[:, 1], P[:, 2], color=SERIES[shown], lw=2, label=p)
        shown += 1
    if shown:
        ax.scatter(*P[0], color=INK, s=20)
        ax.scatter(*P[-1], color=INK, s=20, marker="x")
    ax.plot([0, 0], [0, 0], [0, 0.18], color=INK2, lw=5, solid_capstyle="butt")  # robot base column
    ax.text(0, 0, 0.22, "robot base", color=INK2, fontsize=8)
    lims = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    ax.set_box_aspect(lims[:, 1] - lims[:, 0])  # equal metric scale on all axes
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title(f"TCP paths, scene '{scene}', query {query} (first successful trial)", loc="left", fontsize=10)
    ax.legend(loc="upper left", frameon=False, labelcolor=INK2)
    ax.view_init(elev=22, azim=-130)
    fig.tight_layout()
    fig.savefig(out / f"tcp_paths_{scene}.png", dpi=160)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--out", default="assets/m2")
    ap.add_argument("--scenes-dir", default="ros2_ws/src/ex_planning_lab/scenes")
    a = ap.parse_args()
    run, out = Path(a.run), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rows, summary, paths = load(run)
    scenes = ordered(r["scene"] for r in rows)
    planners = summary["config"]["planners"]
    fig_success(rows, scenes, planners, out)
    fig_strip(rows, scenes, planners, "planning_time_s", "Planning time", "seconds (log)", "planning_time.png", out, log=True)
    fig_strip(rows, scenes, planners, "joint_path_rad", "Joint-space path length", "rad", "path_length.png", out)
    fig_clearance(rows, scenes, planners, summary.get("clearance_estimate_error_bound_m", 0.005), out)
    first_q = defaultdict(str)
    for r in rows:
        first_q.setdefault(r["scene"], r["query"])
    for sc in scenes:
        fig_paths(paths, Path(a.scenes_dir) / f"{sc}.yaml", sc, first_q[sc], ["RRTConnect", "PRM", "RRTstar"], out)
    print("wrote", sorted(p.name for p in out.glob("*.png")))


if __name__ == "__main__":
    main()
