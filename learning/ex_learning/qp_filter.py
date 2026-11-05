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

from typing import Dict, Optional, Tuple

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
    multi-constraint baseline; `solve_bounded_qp` is the real one.

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


def solve_bounded_qp(a: np.ndarray, constraints, lo: float = -1.0, hi: float = 1.0,
                      x_prev: Optional[np.ndarray] = None, rho: float = 0.0,
                      max_iter: int = 100, ftol: float = 1e-9) -> Tuple[np.ndarray, Dict]:
    """Solve the BOUNDED multi-constraint program

        minimise   0.5 (x-a)^T W (x-a) + 0.5 rho ||x - x_prev||^2
        subject to C_i x >= g_i   for every constraint i
                   lo <= x <= hi

    The command bounds are INSIDE the optimisation: they are not applied afterwards by a clip,
    because clipping after a projection is what made the original filter violate its own
    constraint. `rho` penalises the change against the previous command, which keeps the command
    smooth - without it the solution can jump between two feasible corners every step.

    Returns (x, info) with info: feasible (bool), residuals (g - C x, positive = violated),
    iterations, time_s, solver ("slsqp" | "none").
    """
    import time
    from scipy.optimize import Bounds, LinearConstraint, minimize

    a = np.asarray(a, float)
    n = a.shape[0]
    if not constraints:
        x = np.clip(a, lo, hi)
        return x, {"feasible": True, "residuals": np.zeros(0), "iterations": 0, "time_s": 0.0,
                   "solver": "none"}

    C = np.atleast_2d(np.asarray([c for c, _ in constraints], float))
    g = np.asarray([gi for _, gi in constraints], float)
    W = np.eye(n)
    xp = a if x_prev is None else np.asarray(x_prev, float)

    def fun(x):
        d = x - a
        f = 0.5 * d @ W @ d
        if rho:
            f += 0.5 * rho * float(np.sum((x - xp) ** 2))
        return f

    def jac(x):
        d = x - a
        j = W @ d
        if rho:
            j = j + rho * (x - xp)
        return j

    t0 = time.perf_counter()
    res = minimize(fun, np.clip(a, lo, hi), jac=jac, method="SLSQP",
                   bounds=Bounds(lo, hi), constraints=[LinearConstraint(C, g, np.inf)],
                   options={"maxiter": max_iter, "ftol": ftol})
    dt = time.perf_counter() - t0
    x = np.clip(np.asarray(res.x, float), lo, hi)
    residuals = g - C @ x
    return x, {"feasible": bool(np.all(residuals <= 1e-9)), "residuals": residuals,
               "iterations": int(res.nit), "time_s": dt, "solver": "slsqp"}


def solve_bounded_qp_relaxed(a: np.ndarray, constraints, lo: float = -1.0, hi: float = 1.0,
                              x_prev: Optional[np.ndarray] = None, rho: float = 0.0,
                              max_iter: int = 100, ftol: float = 1e-9) -> Tuple[np.ndarray, Dict]:
    """The bounded program, with an EXPLICIT relaxation when it is infeasible.

    An execution-aware filter can face a genuinely infeasible program: the arm is already
    coasting toward an obstacle faster than any command can stop it, so no command keeps the
    margin. Hiding that (clipping the constraint, reporting success) would be a lie; the
    honest behaviour is to return the command that comes CLOSEST to satisfying the program and
    to report the violation.

    Two stages:
      1. solve the hard program; if it is feasible, that is the answer (the common case);
      2. otherwise find the smallest uniform relaxation s* for which the program becomes
         feasible (a linear program), then re-solve the program with every constraint softened
         by s*, which returns the command closest to the policy's request.

    The returned `info` always describes the ORIGINAL program:
      feasible        do the ORIGINAL constraints hold at the returned command?
      relaxation      the uniform relaxation that had to be applied (0 when none was needed)
      max_violation   the worst violation of the ORIGINAL constraints, in the constraint units
      residuals       the violations of the ORIGINAL constraints
    A non-zero `max_violation` is a violation, never a certificate: the caller must count it.
    """
    x, info = solve_bounded_qp(a, constraints, lo=lo, hi=hi, x_prev=x_prev, rho=rho,
                               max_iter=max_iter, ftol=ftol)
    info = dict(info)
    info["relaxation"] = 0.0
    info["max_violation"] = float(np.max(info["residuals"])) if len(info["residuals"]) else 0.0
    if info["feasible"]:
        return x, info

    from scipy.optimize import linprog

    C = np.atleast_2d(np.asarray([c for c, _ in constraints], float))
    g = np.asarray([gi for _, gi in constraints], float)
    n = len(a)
    # variables [x (n), s];  minimise s  s.t.  -C x - s*1 <= -g,  lo <= x <= hi,  s >= 0
    A_ub = np.hstack([-C, -np.ones((len(g), 1))])
    b_ub = -g
    bounds = [(lo, hi)] * n + [(0.0, None)]
    lp = linprog(c=np.concatenate([np.zeros(n), [1.0]]), A_ub=A_ub, b_ub=b_ub, bounds=bounds,
                 method="highs")
    s = float(lp.x[-1]) if lp.success else float(np.max(info["residuals"]))
    s = max(s, 0.0)

    soft = [(c, gi - s - 1e-9) for c, gi in constraints]
    x2, info2 = solve_bounded_qp(a, soft, lo=lo, hi=hi, x_prev=x_prev, rho=rho,
                                  max_iter=max_iter, ftol=ftol)
    residuals = g - C @ x2
    info2 = dict(info2)
    info2["feasible"] = bool(np.all(residuals <= 1e-9))
    info2["relaxation"] = s
    info2["max_violation"] = float(np.max(residuals))
    info2["residuals"] = residuals
    info2["hard_feasible"] = False
    return x2, info2
