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


def _record_latency(stats: Optional[Dict], t: float) -> None:
    """Collect the solver latency properly.

    This used to be `stats["solve_time_s_median"] = info["time_s"]`, which OVERWRITES the value
    on every step: the field that claimed to be a median was the latency of the LAST solve. The
    latency is a headline number for this method (the execution-aware filter solves a program per
    control step, and a filter that cannot run inside the control period is not deployable), so
    the samples are kept and the median and the tail are computed from them.

    `solve_time_s_median` is kept as the key for compatibility, and it is now a real median.
    """
    if stats is None:
        return
    stats.setdefault("solve_times_s", []).append(float(t))


def latency_summary(stats: Optional[Dict]) -> Optional[Dict]:
    """The latency distribution of a filter's solver, from the collected samples."""
    if not stats:
        return None
    t = np.asarray(stats.get("solve_times_s", []), float)
    if t.size == 0:
        return None
    out = {"n": int(t.size), "median_s": float(np.median(t)),
           "p95_s": float(np.percentile(t, 95)), "p99_s": float(np.percentile(t, 99)),
           "max_s": float(t.max()), "mean_s": float(t.mean())}
    stats["solve_time_s_median"] = out["median_s"]
    stats["solve_time_s_p95"] = out["p95_s"]
    stats["solve_time_s_p99"] = out["p99_s"]
    return out


def speed_dependent_margin(env, base_policy: Callable, margin0: float = 0.01,
                           tau: float = 0.022, a_min: float = 2.0,
                           margin_max: float = 0.04,
                           v_close: Optional[float] = None, alpha: float = 5.0,
                           stats: Optional[Dict] = None) -> Callable:
    """B4+: the FIXED-margin QP with a SPEED-DEPENDENT margin (the cheap strong baseline).

    This is the alternative that a reviewer will ask for, and it is the one that can kill the
    proposed method: if simply inflating the margin with the closing speed reproduces the
    execution-aware filter's behaviour, then the look-ahead prediction is not needed - a
    textbook braking-distance margin is enough.

    The margin is the standard stopping distance, in the units of the CLEARANCE (metres per
    second of closing speed):

        m(v) = min( margin_max, m0 + v_close * tau + v_close^2 / (2 * a_min) )

    with `tau` the command-to-motion lag (the servo lag this project measured: 2*zeta/omega =
    48 ms for the lag and ~22 ms for the velocity tracking, so 22 ms is the conservative choice),
    and `a_min` the smallest Cartesian deceleration the arm can achieve [m/s^2].

    **The cap is what makes this a FAIR baseline.** Without it the margin reaches half a metre
    for a fast approach, the constraint becomes infeasible everywhere, and the filter becomes
    actively harmful - it then drives the arm into the obstacle, which is not a strong baseline
    but a straw man. With the cap at the SAME margin the tuned fixed-margin QP uses, B4+ can only
    ever be *safer* than B4, never more conservative: it spends the margin where the risk is.

    The margin is applied to the CLOSING speed towards the obstacle - the rate at which the
    clearance shrinks - not to the joint speed: the arm moving fast AWAY from the wall needs no
    extra margin.
    """
    from .qp_filter import solve_bounded_qp
    import mujoco
    jp = np.zeros((3, env.m.nv))
    state = {"prev": None}

    def act(obs):
        a = np.asarray(base_policy(obs), float).copy()
        constraints = []
        pairs = []
        for d, p, n, body in env.repulsion_points(0.30):
            mujoco.mj_jac(env.m, env.d, jp, None, p, body)
            c = n @ jp[:, env.dadr]
            if float(c @ c) < 1e-12:
                continue
            # the closing speed: the rate at which this pair's clearance shrinks [m/s]
            v_close_here = -float(c @ (a * env.cfg.max_joint_speed))
            vc = v_close if v_close is not None else max(v_close_here, 0.0)
            m = min(margin_max, margin0 + vc * tau + vc * vc / (2.0 * a_min))
            constraints.append((c, -alpha * (d - m) / env.cfg.max_joint_speed))
            pairs.append((float(d), c, float(vc), float(m)))
        info = {"feasible": True, "residuals": np.zeros(0), "time_s": 0.0}
        if constraints:
            x, info = solve_bounded_qp(a, constraints, x_prev=state["prev"], rho=0.0)
            state["prev"] = x
            if stats is not None:
                stats["solves"] = stats.get("solves", 0) + 1
                stats["infeasible"] = stats.get("infeasible", 0) + int(not info["feasible"])
                stats["max_violation"] = max(stats.get("max_violation", 0.0),
                                             float(np.max(info["residuals"])) if len(info["residuals"]) else 0.0)
                stats["interventions"] = stats.get("interventions", 0) + int(np.linalg.norm(x - a) > 1e-9)
                _record_latency(stats, info["time_s"])
            a = x
        return np.clip(a, -1.0, 1.0)
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
                  d_max: float = 0.30, rho: float = 0.0, stats: Optional[Dict] = None,
                  trace: Optional[list] = None) -> Callable:
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

    `trace` (optional list) collects one record per control step - the constraints, the solution
    and the residuals - so an external diagnostic can check the filter's own safety claim against
    what the plant actually did (see evaluation/execution_gap.py).
    """
    import mujoco
    from .qp_filter import solve_bounded_qp
    jp = np.zeros((3, env.m.nv))
    state = {"prev": None}

    def act(obs):
        a_base = np.asarray(base_policy(obs), float).copy()
        a = a_base.copy()
        constraints = []
        pairs = []
        for d, p, n, body in env.repulsion_points(d_max):
            mujoco.mj_jac(env.m, env.d, jp, None, p, body)
            c = n @ jp[:, env.dadr]
            if float(c @ c) < 1e-12:
                continue
            constraints.append((c, -alpha * (d - margin) / env.cfg.max_joint_speed))
            pairs.append((float(d), c))
        info = {"feasible": True, "residuals": np.zeros(0), "time_s": 0.0}
        if constraints:
            x, info = solve_bounded_qp(a, constraints, x_prev=state["prev"], rho=rho)
            state["prev"] = x
            if stats is not None:
                stats["solves"] = stats.get("solves", 0) + 1
                stats["infeasible"] = stats.get("infeasible", 0) + int(not info["feasible"])
                stats["max_violation"] = max(stats.get("max_violation", 0.0),
                                             float(np.max(info["residuals"])) if len(info["residuals"]) else 0.0)
                _record_latency(stats, info["time_s"])
            a = x
        if trace is not None:
            trace.append({"a_base": a_base.copy(),
                          "a": np.clip(a, -1.0, 1.0).copy(),
                          "pairs": pairs, "residuals": np.asarray(info["residuals"], float),
                          "feasible": bool(info["feasible"])})
        return np.clip(a, -1.0, 1.0)
    return act


def cbf_policy_exec(env, base_policy: Callable, exec_model, margin: float = 0.02,
                     d_max: float = 0.30, horizon: int = 5, sample_every: int = 2,
                     rho: float = 0.0, account_execution: bool = True,
                     stats: Optional[Dict] = None, trace: Optional[list] = None) -> Callable:
    """B5: the EXECUTION-AWARE safety filter (SIMULATION ONLY).

    The kinematic filters (`cbf_policy`, `cbf_policy_qp`) constrain the COMMAND. They assume
    that the executed velocity equals the commanded velocity. In this plant that is false: the
    command is integrated into a position reference, the reference is clipped at the joint
    limits, and a bandwidth-limited servo tracks it - so the arm keeps moving after the command
    changes, and the motion it actually performs lags the motion the filter reasoned about.

    This filter constrains the PREDICTED MOTION instead:

    1. the execution model predicts where the arm goes if the candidate command is held for
       `horizon` policy steps, starting from the EXECUTED state (q, qdot, q_ref) - not from the
       command alone;
    2. the prediction is expanded to first order in the command,

           dQ_k(v)  ~=  dQ_k(0) + G_k . v

       where ``dQ_k(0)`` is the displacement with a ZERO command: the COAST that a kinematic
       filter never accounts for;
    3. for every (link, obstacle) pair and every sample along the horizon, the predicted
       clearance is required to stay above the margin,

           d_i + c_i . dQ_k(v)  >=  margin
        =>  (c_i . G_k) . v  >=  margin - d_i - c_i . dQ_k(0)

       which is a LINEAR constraint in the command, so the whole thing is one bounded program
       with the command bounds INSIDE the optimisation (the same solver as `cbf_policy_qp`);
    4. if the program is infeasible - the arm is already coasting toward an obstacle faster than
       any command can stop it - the solver returns the command that comes closest, and the
       violation is reported in `stats["relaxation"]`. A relaxed constraint is a violation, never
       a certificate.

    The clearance used here is the RIGOROUS LOWER BOUND (`conservative=True`), not the sampled
    upper bound the archived evaluations used.

    `account_execution=False` is the ABLATION: the coast term `dQ_k(0)` is dropped, so the
    constraint is the same LOOKAHEAD but with an execution-blind model. It separates "the
    lookahead structure" from "the execution model", so the paper can say which one carries the
    result.

    `stats` collects: solves, infeasible (relaxed) solves, max_relaxation (in metres of
    clearance), the median solve time, and the intervention rate.
    """
    import mujoco
    from .qp_filter import solve_bounded_qp_relaxed
    jp = np.zeros((3, env.m.nv))
    state = {"prev": None}

    def act(obs):
        a_base = np.clip(np.asarray(base_policy(obs), float), -1.0, 1.0)
        a = a_base.copy()
        # the pairs, with the rigorous lower-bound clearance
        d_all, pts, nrm = env.pair_distances(conservative=True)
        idx = np.where(d_all < d_max)[0]
        rows = []
        for i in idx:
            body = int(env.cap_body[env._pi[i]])
            mujoco.mj_jac(env.m, env.d, jp, None, pts[i], body)
            c = nrm[i] @ jp[:, env.dadr]
            if float(c @ c) > 1e-12:
                rows.append((float(d_all[i]), c))
        if not rows:
            state["prev"] = a
            if trace is not None:
                trace.append({"a_base": a_base.copy(), "a": a.copy(), "pairs": [],
                              "residuals": np.zeros(0), "feasible": True})
            return a

        # the predicted motion, affine in the command
        dq0, G = exec_model.affine(env.q(), env.d.qvel[env.dadr], np.asarray(env.q_ref, float),
                                   a, env.lo, env.hi, horizon, step=sample_every)
        if not account_execution:
            dq0 = np.zeros_like(dq0)
        constraints = []
        for di, c in rows:
            for k in range(dq0.shape[0]):
                constraints.append((c @ G[k], margin - di - float(c @ dq0[k])))

        x, info = solve_bounded_qp_relaxed(a, constraints, x_prev=state["prev"], rho=rho)
        state["prev"] = x
        if stats is not None:
            stats["solves"] = stats.get("solves", 0) + 1
            stats["infeasible"] = stats.get("infeasible", 0) + int(not info.get("feasible", True))
            stats["max_relaxation"] = max(stats.get("max_relaxation", 0.0),
                                          float(info.get("max_violation", 0.0)))
            _record_latency(stats, info["time_s"])
            stats["interventions"] = stats.get("interventions", 0) + int(np.linalg.norm(x - a) > 1e-9)
            stats["steps"] = stats.get("steps", 0) + 1
        if trace is not None:
            trace.append({"a_base": a_base.copy(), "a": np.clip(x, -1.0, 1.0).copy(),
                          "pairs": rows, "residuals": np.asarray(info.get("residuals", np.zeros(0)), float),
                          "feasible": bool(info.get("feasible", True))})
        return np.clip(x, -1.0, 1.0)
    return act
