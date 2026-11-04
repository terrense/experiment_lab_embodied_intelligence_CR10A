"""Safety filter for a learned policy (SIMULATION ONLY).

Two filters, both using the environment's geometric clearance (the same point/segment model as
everywhere else in this repo):

* ``safe_policy``  - the NAIVE filter: an always-on potential-field repulsion (the APF term) added to
  the learned action. It is the straw-man the barrier filter is compared against; it fights a policy
  that was trained without repulsion and halves the success rate.
* ``cbf_policy``   - a control-barrier projection: the closest obstacle point defines the constraint
  ``clearance >= margin``; the learned action is projected onto it with the MINIMAL change, so it
  only brakes the approach near contact and is inactive otherwise.

The projection is onto the INTERSECTION of the command box with the barrier half-space, because a
half-space projection followed by componentwise clipping can leave the constraint violated
(see ``qp_filter.project_box_halfspace`` and its 2-D counterexample in the tests).
``cbf_policy_legacy`` keeps the old project-then-clip behaviour, so the archived 2026-10-29
evaluations stay reproducible and the paper can attribute the difference between "the evaluation
changed" and "the method improved".

Both filters return a *diagnostics* dict through the optional ``stats`` argument: how often the
constraint was active, how often the solved command still violated it, and how often the problem
was infeasible. A relaxed constraint is a constraint violation, not a safety certificate.
"""

from __future__ import annotations

from typing import Callable, Dict, Optional

import numpy as np

from .qp_filter import project_box_halfspace


def safe_policy(env, base_policy: Callable, d0: float = 0.10, k_rep: float = 0.02,
                stats: Optional[Dict] = None) -> Callable:
    """Return base_policy + soft-CBF repulsion. base_policy(obs) must return an action in [-1, 1]^6.

    This is the NAIVE safety filter: an always-on potential-field repulsion (the APF term) added to
    the learned action. It removes collisions only when the activation range d0 is large, and then it
    holds the arm far from the wall (local minimum), destroying the success rate. Kept as the
    straw-man the CBF projection below is compared against.
    """
    import mujoco
    jp = np.zeros((3, env.m.nv))

    def act(obs):
        dq = np.asarray(base_policy(obs), float) * env.cfg.max_joint_speed
        for d, p_robot, n, body in env.repulsion_points(d0):
            mujoco.mj_jac(env.m, env.d, jp, None, p_robot, body)
            dd = max(d, 0.005)
            dq = dq + jp[:, env.dadr].T @ (k_rep * (1.0 / dd - 1.0 / d0) / dd ** 2 * n)
        a = dq / env.cfg.max_joint_speed
        m = np.max(np.abs(a))
        return a / m if m > 1 else a
    return act


def cbf_policy(env, base_policy: Callable, margin: float = 0.02, alpha: float = 5.0,
               d_max: float = 0.30, stats: Optional[Dict] = None) -> Callable:
    """Safety filter as a single-constraint control barrier function (CBF).

    The closest robot point within `d_max` of an obstacle defines the constraint h = clearance - margin >= 0.
    The learned action a_base is projected onto it with the *minimal* change, so it only brakes the
    approach near contact and is inactive otherwise.

    The projection solves  min ||x - a||^2  s.t.  |x| <= 1  and  c.x >= g  exactly (see
    ``qp_filter.project_box_halfspace``), so the returned command satisfies the constraint whenever
    it is satisfiable at all. `stats` (optional dict) collects: active (constraint engaged),
    violated (the solved command still violates it), infeasible (no command in the box satisfies it)
    and the worst residual.

    base_policy(obs) returns an action in [-1, 1]^6; the projection is done in action space.
    """
    import mujoco
    jp = np.zeros((3, env.m.nv))

    def act(obs):
        a = np.asarray(base_policy(obs), float).copy()
        pts = env.repulsion_points(d_max)
        if pts:
            d, p, n, body = min(pts, key=lambda t: t[0])
            mujoco.mj_jac(env.m, env.d, jp, None, p, body)
            c = n @ jp[:, env.dadr]                                   # row: dh/dt = c @ a * max_speed
            g = -alpha * (d - margin) / env.cfg.max_joint_speed       # require c @ a >= g
            if float(c @ a) < g:                                      # constraint violated -> project
                a_new, feasible, residual = project_box_halfspace(a, c, g)
                if stats is not None:
                    stats["active"] = stats.get("active", 0) + 1
                    if not feasible:
                        stats["infeasible"] = stats.get("infeasible", 0) + 1
                    if residual > 0:
                        stats["violated"] = stats.get("violated", 0) + 1
                        stats["worst_residual"] = max(stats.get("worst_residual", 0.0), float(residual))
                a = a_new
        return np.clip(a, -1.0, 1.0)
    return act


def cbf_policy_legacy(env, base_policy: Callable, margin: float = 0.02, alpha: float = 5.0,
                       d_max: float = 0.30) -> Callable:
    """LEGACY CBF filter: half-space projection followed by componentwise clipping.

    Kept ONLY to reproduce the archived 2026-10-29 evaluations (data/reports/learning/gen_safe/).
    The clipping step can re-violate the constraint, so this filter must not be used for new
    results and must not be described as a safety guarantee. See ``cbf_policy`` for the bounded
    projection.
    """
    import mujoco
    jp = np.zeros((3, env.m.nv))

    def act(obs):
        a = np.asarray(base_policy(obs), float).copy()
        pts = env.repulsion_points(d_max)
        if pts:
            d, p, n, body = min(pts, key=lambda t: t[0])
            mujoco.mj_jac(env.m, env.d, jp, None, p, body)
            c = n @ jp[:, env.dadr]
            g = -alpha * (d - margin) / env.cfg.max_joint_speed
            cc = float(c @ c)
            if cc > 1e-12 and c @ a < g:
                a = a + c * ((g - float(c @ a)) / cc)
            a = np.clip(a, -1.0, 1.0)
        return a
    return act


def cbf_policy_qp(env, base_policy: Callable, margin: float = 0.05, alpha: float = 5.0,
                  d_max: float = 0.30, rho: float = 0.0, stats: Optional[Dict] = None) -> Callable:
    """Bounded ALL-PAIR kinematic QP (the B3 baseline of the research plan).

    Unlike `cbf_policy`, which constrains only the single closest obstacle, this one builds a
    constraint for EVERY (link, box) pair within `d_max` and solves them together with the command
    bounds in one program:

        minimise   ||x - a||^2 + rho ||x - x_prev||^2
        subject to n_i^T J_i x >= -alpha (d_i - margin) / max_speed   for every pair i
                   -1 <= x <= 1

    The bounds are inside the optimisation, so the returned command satisfies every constraint
    that is satisfiable at all - no post-hoc clipping. Infeasibility, solver time and the
    constraint residuals are collected in `stats`; a relaxed constraint is reported as a violation,
    never as a certificate.
    """
    import mujoco
    from .qp_filter import solve_bounded_qp
    jp = np.zeros((3, env.m.nv))
    state = {"prev": None}

    def act(obs):
        a = np.asarray(base_policy(obs), float).copy()
        constraints = []
        for d, p, n, body in env.repulsion_points(d_max):
            mujoco.mj_jac(env.m, env.d, jp, None, p, body)
            c = n @ jp[:, env.dadr]
            if float(c @ c) < 1e-12:
                continue
            constraints.append((c, -alpha * (d - margin) / env.cfg.max_joint_speed))
        if constraints:
            x, info = solve_bounded_qp(a, constraints, x_prev=state["prev"], rho=rho)
            state["prev"] = x
            if stats is not None:
                stats["solves"] = stats.get("solves", 0) + 1
                stats["infeasible"] = stats.get("infeasible", 0) + int(not info["feasible"])
                stats["max_violation"] = max(stats.get("max_violation", 0.0),
                                             float(np.max(info["residuals"])) if len(info["residuals"]) else 0.0)
                stats["solve_time_s_median"] = info["time_s"]
            a = x
        return np.clip(a, -1.0, 1.0)
    return act
