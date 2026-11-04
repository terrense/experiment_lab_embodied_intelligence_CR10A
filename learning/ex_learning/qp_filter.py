"""Bounded optimisation for safety filters (SIMULATION ONLY).

The safety filter has to answer one question every control step: *what is the command closest to
what the policy asked for that still respects the safety constraint?* When the command itself is
bounded (the policy commands a joint-velocity reference in [-1, 1]^6 and the controller can only
accept a bounded command), "closest" means the closest point in the INTERSECTION of the command box
with the safety half-space -- not "project onto the half-space, then clip".

Those two are different operations. Projection onto a half-space followed by componentwise clipping
can leave the constraint violated:

    a = (1, -1), c = (1, 0.1), g = 1.05
    half-space projection -> (1.148515, -0.985149),  c.a = 1.05   (exactly on the boundary)
    clip to [-1, 1]      -> (1, -0.985149),          c.a = 0.901485 < 1.05   (VIOLATED)
    while (1, 0.5) is inside the box and satisfies c.a = 1.05 exactly.

So this module solves the bounded problem exactly, and reports infeasibility instead of hiding it:
an over-relaxed constraint is a constraint violation, not a successful safety certificate.

The solver is the closed-form KKT solution of

    minimise  ||x - a||^2   subject to   lo <= x <= hi   and   c.x >= g

whose solution has the form  x(lam) = clip(a + lam * c, lo, hi)  for some lam >= 0. The multiplier
is found by bisection, because  c.x(lam)  is monotone non-decreasing in lam.
"""

from __future__ import annotations

from typing import Tuple

import numpy as np


def project_box_halfspace(a: np.ndarray, c: np.ndarray, g: float,
                           lo: float = -1.0, hi: float = 1.0,
                           iters: int = 64, grow: int = 64) -> Tuple[np.ndarray, bool, float]:
    """Closest point to `a` in {lo <= x <= hi} ∩ {c.x >= g}.

    Returns (x, feasible, residual):
      x        - the projected command (already inside the box)
      feasible - False if no point in the box satisfies c.x >= g (then x is the best effort)
      residual - g - c.x  (0 or negative when the constraint holds; POSITIVE means VIOLATED)
    """
    a = np.clip(np.asarray(a, float), lo, hi)
    c = np.asarray(c, float)
    if float(c @ a) >= g:
        return a, True, g - float(c @ a)

    def h(lam: float) -> float:
        return float(c @ np.clip(a + lam * c, lo, hi))

    lam_hi = 1.0
    for _ in range(grow):                       # grow until the constraint becomes reachable
        if h(lam_hi) >= g:
            break
        lam_hi *= 2.0
    else:
        x = np.clip(a + lam_hi * c, lo, hi)
        return x, False, g - float(c @ x)

    lam_lo = 0.0
    for _ in range(iters):                     # bisect: h is monotone non-decreasing in lam
        lam = 0.5 * (lam_lo + lam_hi)
        if h(lam) >= g:
            lam_hi = lam
        else:
            lam_lo = lam
    x = np.clip(a + lam_hi * c, lo, hi)
    return x, True, g - float(c @ x)


def project_box_halfspace_multi(a: np.ndarray, C: np.ndarray, g: np.ndarray,
                                lo: float = -1.0, hi: float = 1.0,
                                iters: int = 64) -> Tuple[np.ndarray, bool, np.ndarray]:
    """Sequential projection onto several half-spaces, each intersected with the command box.

    Order matters and this is NOT a true QP solve: projecting onto half-space 1 can push the point
    out of half-space 2. The function therefore returns the residuals of EVERY constraint so the
    caller can log which ones are violated instead of assuming they hold. It is the cheap
    multi-constraint baseline; the full QP (all pairs, joint limits, command bounds solved
    together) is the next step.

    Returns (x, all_feasible, residuals) with residuals = g - C.x (positive means violated).
    """
    x = np.clip(np.asarray(a, float), lo, hi)
    C = np.atleast_2d(np.asarray(C, float))
    g = np.atleast_1d(np.asarray(g, float))
    feasible = True
    for i in range(len(C)):
        x, ok, _ = project_box_halfspace(x, C[i], float(g[i]), lo, hi, iters=iters)
        feasible = feasible and ok
    return x, feasible, g - C @ x
