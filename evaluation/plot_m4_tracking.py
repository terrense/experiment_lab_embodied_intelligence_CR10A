#!/usr/bin/env python3
"""Summarise the M4 tracking study (SIMULATED data).

  python3 evaluation/plot_m4_tracking.py data/reports/m4/run01 --out assets/m4
Writes <out>/tracking_tcp_max.png and <run>/summary.md
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SURFACE, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#8a8984", "#e6e5e0"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]  # slots 1-3 (payload 0 / 5 / 10 kg)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--out", default="assets/m4")
    a = ap.parse_args()
    run, out = Path(a.run), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    reps = {}
    for f in sorted(run.glob("*/execute_report.json")):
        r = json.loads(f.read_text())
        if r.get("tag"):
            reps[r["tag"]] = r
    import numpy as np

    def cond(tag):  # strip the repeat suffix
        return tag.rsplit("_r", 1)[0] if "_r" in tag.rsplit("_s", 1)[-1] else tag

    groups = {}
    for tag, r in reps.items():
        groups.setdefault(cond(tag), []).append(r)
    rows = ["| condition | speed | runs | aborted | TCP max median [min, max] (mm) | TCP rms median (mm) | start latency median (ms) | torque-saturated steps (median, J1..J6) |",
            "|---|---|---|---|---|---|---|---|"]
    for c in sorted(groups):
        g = groups[c]
        ok = [r for r in g if r["execution_ok"]]
        tm = [r["tcp_tracking_max_mm"] for r in g]
        sats = [r["torque_saturation_steps_during_move"] for r in g if r["torque_saturation_steps_during_move"] is not None]
        sat = np.median(np.array(sats), axis=0).astype(int).tolist() if sats else "-"
        rows.append(f"| {c.rsplit('_s', 1)[0]} | {g[0]['speed_scale']} | {len(g)} | {len(g) - len(ok)} | "
                    f"{np.median(tm):.2f} [{min(tm):.2f}, {max(tm):.2f}] | {np.median([r['tcp_tracking_rms_mm'] for r in g]):.2f} | "
                    f"{np.median([r['estimated_start_latency_s'] for r in g]) * 1000:.0f} | {sat} |")
    (run / "summary.md").write_text("\n".join(rows) + "\n")
    print("\n".join(rows))

    speeds = sorted({r["speed_scale"] for r in reps.values()})
    fig, axes = plt.subplots(1, len(speeds), figsize=(5.2 * len(speeds), 3.9), sharey=True, facecolor=SURFACE)
    axes = axes if len(speeds) > 1 else [axes]
    for ax, sp in zip(axes, speeds):
        ax.set_facecolor(SURFACE)
        for k, pl in enumerate((0, 5, 10)):
            xs, med, lo, hi, nab = [], [], [], [], []
            for bw in (4, 8, 16):
                g = groups.get(f"mj_bw{bw}_pl{pl}_s{sp}")
                if not g:
                    continue
                tm = [r["tcp_tracking_max_mm"] for r in g]
                x = bw * (0.94, 1.0, 1.06)[k]  # dodge
                xs.append(x); med.append(np.median(tm)); lo.append(min(tm)); hi.append(max(tm))
                nab.append(sum(not r["execution_ok"] for r in g))
            ax.vlines(xs, lo, hi, color=SERIES[k], lw=1.5, alpha=0.6)
            ax.plot(xs, med, color=SERIES[k], lw=2, marker="o", ms=7, label=f"MuJoCo, payload {pl} kg")
            for x, m_, n in zip(xs, med, nab):
                if n:
                    ax.text(x, m_ * 1.15, f"{n} aborted", color=INK2, fontsize=7, ha="center")
        mock = groups.get(f"mock_s{sp}")
        if mock:
            mm = np.median([r["tcp_tracking_max_mm"] for r in mock])
            ax.axhline(mm, color=MUTED, ls="--", lw=1.2)
            ax.text(16.5, mm, " mock plant", color=INK2, va="bottom", ha="right", fontsize=8)
        ax.set_xscale("log", base=2)
        ax.set_xticks([4, 8, 16], ["4", "8", "16"])
        ax.set_xlabel("servo bandwidth (Hz)", color=INK2)
        ax.set_title(f"speed x{sp}", loc="left", fontweight="bold", color=INK, fontsize=10)
        ax.grid(color=GRID)
        for s_ in ("top", "right"):
            ax.spines[s_].set_visible(False)
        ax.tick_params(colors=INK2)
    axes[0].set_ylabel("max TCP tracking error (mm)", color=INK2)
    axes[0].set_yscale("log")
    axes[-1].legend(frameon=False, fontsize=8, labelcolor=INK2)
    fig.suptitle("Same trajectory, different plants: max TCP deviation (dot = median, bar = min-max over repeats)",
                 x=0.01, ha="left", fontweight="bold", fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(out / "tracking_tcp_max.png", dpi=160, facecolor=SURFACE)
    print("wrote", out / "tracking_tcp_max.png")


if __name__ == "__main__":
    main()
