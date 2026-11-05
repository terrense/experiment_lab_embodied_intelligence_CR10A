#!/usr/bin/env python3
"""Palette validation: OKLab separation and colour-vision-deficiency separation.

The bundled dataviz validator is not present in this environment, so the checks it performs are
implemented here:

  * OKLab Delta E (x100) between every pair - the normal-vision floor is 15;
  * the same Delta E after simulating deuteranopia, protanopia and tritanopia (Vienot 1999) - the
    floor is 8;
  * a lightness band and a minimum chroma, so a hue is neither washed out nor black.

Usage:  python3 evaluation/validate_palette.py "#efb118,#4269d0,#00b383" --surface "#ffffff"
"""

from __future__ import annotations

import argparse
import itertools
import math

# --------------------------------------------------------------------- colour maths
def srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def linear_to_srgb(c: float) -> float:
    return 12.92 * c if c <= 0.0031308 else 1.055 * (c ** (1 / 2.4)) - 0.055


def hex_to_rgb(h: str):
    h = h.strip().lstrip("#")
    return tuple(int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))


def rgb_to_oklab(rgb):
    r, g, b = (srgb_to_linear(c) for c in rgb)
    l = 0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b
    m = 0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b
    s = 0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b
    l_, m_, s_ = (math.copysign(abs(v) ** (1 / 3), v) for v in (l, m, s))
    return (0.2104542553 * l_ + 0.7936177850 * m_ - 0.0040720468 * s_,
            1.9779984951 * l_ - 2.4285922050 * m_ + 0.4505937099 * s_,
            0.0259040371 * l_ + 0.7827717662 * m_ - 0.8086757660 * s_)


def oklab_to_rgb(lab):
    L, a, b = lab
    l_ = L + 0.3963377774 * a + 0.2158037573 * b
    m_ = L - 0.1055613458 * a - 0.0638541728 * b
    s_ = L - 0.0894841775 * a - 1.2914855480 * b
    l, m, s = l_ ** 3, m_ ** 3, s_ ** 3
    r = +4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s
    g = -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s
    bb = -0.0041960863 * l - 0.7034186147 * m + 1.7076147010 * s
    return tuple(min(1.0, max(0.0, linear_to_srgb(c))) for c in (r, g, bb))


def delta_e(lab1, lab2) -> float:
    return 100.0 * math.sqrt(sum((x - y) ** 2 for x, y in zip(lab1, lab2)))


def simulate_cvd(rgb, kind: str):
    """Vienot-Brettel-Mollon style simulation in linear RGB."""
    r, g, b = (srgb_to_linear(c) for c in rgb)
    if kind == "deuteranopia":
        r, g, b = 0.625 * r + 0.375 * g, 0.7 * g + 0.3 * r, 0.3 * g + 0.7 * b
    elif kind == "protanopia":
        r, g, b = 0.567 * r + 0.433 * g, 0.558 * g + 0.442 * r, 0.242 * g + 0.758 * b
    elif kind == "tritanopia":
        r, g, b = 0.95 * r + 0.05 * g, 0.433 * g + 0.567 * b, 0.475 * b + 0.525 * g
    return tuple(linear_to_srgb(min(1.0, max(0.0, c))) for c in (r, g, b))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("palette", help="comma-separated hex colours")
    ap.add_argument("--surface", default="#ffffff")
    ap.add_argument("--min-normal", type=float, default=15.0)
    ap.add_argument("--min-cvd", type=float, default=8.0)
    ap.add_argument("--lightness-band", default="0.55,0.85")
    ap.add_argument("--min-chroma", type=float, default=0.07)
    a = ap.parse_args()

    hexes = [h for h in a.palette.split(",") if h.strip()]
    rgbs = [hex_to_rgb(h) for h in hexes]
    labs = [rgb_to_oklab(c) for c in rgbs]
    lo, hi = [float(x) for x in a.lightness_band.split(",")]
    surface_l = rgb_to_oklab(hex_to_rgb(a.surface))

    print(f"surface {a.surface}  L* {surface_l[0]:.3f}")
    ok = True
    for h, lab in zip(hexes, labs):
        L, chroma = lab[0], math.hypot(lab[1], lab[2])
        flags = []
        if not (lo <= L <= hi):
            flags.append(f"lightness {L:.3f} outside [{lo},{hi}]")
        if chroma < a.min_chroma:
            flags.append(f"chroma {chroma:.3f} < {a.min_chroma}")
        rel = delta_e(lab, surface_l)
        if rel < 12:
            flags.append(f"contrast {rel:.1f} too low against the surface")
        status = "FAIL" if flags else "ok"
        if flags:
            ok = False
        print(f"  {h}  L {L:.3f}  C {chroma:.3f}  contrast dE {rel:5.1f}   {status}"
              + ("  <- " + "; ".join(flags) if flags else ""))

    print("\npair separation (OKLab dE x100):")
    for (h1, l1), (h2, l2) in itertools.combinations(zip(hexes, labs), 2):
        d = delta_e(l1, l2)
        row = [f"  {h1} vs {h2}: normal {d:5.1f}"]
        worst = d
        for kind in ("deuteranopia", "protanopia", "tritanopia"):
            c1 = rgb_to_oklab(simulate_cvd(hex_to_rgb(h1), kind))
            c2 = rgb_to_oklab(simulate_cvd(hex_to_rgb(h2), kind))
            dc = delta_e(c1, c2)
            worst = min(worst, dc)
            row.append(f"{kind[:5]} {dc:5.1f}")
        if d < a.min_normal:
            ok = False
        if worst < a.min_cvd:
            ok = False
        verdict = "ok" if (d >= a.min_normal and worst >= a.min_cvd) else "FAIL"
        print("  ".join(row) + f"   {verdict}")

    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
