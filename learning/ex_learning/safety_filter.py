"""Safety filter for a learned policy (SIMULATION ONLY).

A soft control-barrier term: the base policy's joint velocity is augmented by a potential-field
repulsion from every link point within `d0` of an obstacle -- the SAME geometric term the APF
baseline uses. It grows like 1/d as the clearance d -> 0, so near contact the repulsion dominates
and pushes the arm away; far from obstacles it is exactly zero and the base policy is untouched.

This is the cheapest possible "don't collide" guarantee to bolt onto a memorising policy. The
experiment: does it kill the out-of-distribution collisions (36 % on taller + wider walls) without
destroying the success rate?
"""

from __future__ import annotations

from typing import Callable

import numpy as np


def safe_policy(env, base_policy: Callable, d0: float = 0.10, k_rep: float = 0.02) -> Callable:
    """Return base_policy + soft-CBF repulsion. base_policy(obs) must return an action in [-1, 1]^6.

    This is the NAIVE safety filter: an always-on potential-field repulsion (the APF term) added to the
    learned action. It removes collisions only when the activation range d0 is large, and then it holds
    the arm far from the wall (local minimum), destroying the success rate. Kept as the straw-man the
    CBF projection below is compared against.
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
               d_max: float = 0.30) -> Callable:
    """Safety filter as a single-constraint control barrier function (CBF).

    The closest robot point within `d_max` of an obstacle defines the constraint h = clearance - margin >= 0.
    The learned action a_base is projected onto the half-space `n^T J a >= -alpha * h / max_speed` (h decay
    at least as fast as -alpha*h), i.e. the MINIMAL change that stops the approach before contact. Far from
    the obstacle the constraint is inactive and the base action passes through unchanged.

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
            cc = float(c @ c)
            if cc > 1e-12 and c @ a < g:                              # constraint violated -> project
                a = a + c * ((g - float(c @ a)) / cc)
            a = np.clip(a, -1.0, 1.0)
        return a
    return act
