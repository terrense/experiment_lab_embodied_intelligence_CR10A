"""Kinematic parameter identification (position-only), ROS-free.

Model: for each actuated joint i of a serial chain
    T_i(q) = Origin_i * Delta_i * Rot(axis_i, q_i + dtheta_i)
    Delta_i = Trans(dt_i) * RotX(dr_i[0]) RotY(dr_i[1]) RotZ(dr_i[2])
and a flange offset dt_f appended after the last link. Parameters p = [ (dt_i[3], dr_i[3],
dtheta_i) for i in 1..6, dt_f[3] ]  -> 45 values (m, rad).

Position-only data cannot identify all of them (e.g. rotation of a joint origin about its
own axis is the same as dtheta_i; translations along consecutive parallel axes trade off).
`identifiability()` reports the numerical rank of the Jacobian; the solver uses Tikhonov
regularisation so unidentifiable directions stay at ~0 instead of drifting.

What the result means for EMBODIED-X: the measurements are the controller's OWN TCP (its
factory-calibrated model), so this aligns our URDF with the controller - not with an
external metrology reference.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from .se3 import matrix_to_rpy, rot_x, rot_y, rot_z
from .urdf_model import JointSpec

N_PER_JOINT = 7


def _axis_angle_batch(axis: np.ndarray, ang: np.ndarray) -> np.ndarray:
    """(N,4,4) rotations about a fixed unit axis by angles ang (N,)."""
    x, y, z = axis
    c, s = np.cos(ang), np.sin(ang)
    C = 1 - c
    T = np.zeros((len(ang), 4, 4))
    T[:, 0, 0] = c + x * x * C
    T[:, 0, 1] = x * y * C - z * s
    T[:, 0, 2] = x * z * C + y * s
    T[:, 1, 0] = y * x * C + z * s
    T[:, 1, 1] = c + y * y * C
    T[:, 1, 2] = y * z * C - x * s
    T[:, 2, 0] = z * x * C - y * s
    T[:, 2, 1] = z * y * C + x * s
    T[:, 2, 2] = c + z * z * C
    T[:, 3, 3] = 1.0
    return T


def delta_transform(dt: Sequence[float], dr: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rot_x(dr[0]) @ rot_y(dr[1]) @ rot_z(dr[2])
    T[:3, 3] = dt
    return T


@dataclass
class ChainModel:
    """Serial chain base -> flange with fixed transforms between actuated joints.

    pre[i]  : fixed transform from the previous actuated joint frame (or base) to joint i's origin
              (product of any fixed joints in between and joint i's own <origin>)
    axes[i] : joint axis in its own frame
    post    : fixed transform from the last joint frame to the flange
    """
    pre: List[np.ndarray]
    axes: List[np.ndarray]
    post: np.ndarray

    @classmethod
    def from_chain(cls, chain: Sequence[JointSpec]) -> "ChainModel":
        pre, axes = [], []
        acc = np.eye(4)
        for j in chain:
            acc = acc @ j.origin
            if j.is_actuated:
                pre.append(acc)
                axes.append(np.asarray(j.axis, float))
                acc = np.eye(4)
        return cls(pre, axes, acc)

    @property
    def n_params(self) -> int:
        return N_PER_JOINT * len(self.pre) + 3

    def fk_positions(self, q: np.ndarray, p: np.ndarray | None = None) -> np.ndarray:
        """Flange positions (N,3) for joint angles q (N,n) [rad] under parameters p."""
        N, n = q.shape
        p = np.zeros(self.n_params) if p is None else p
        T = np.broadcast_to(np.eye(4), (N, 4, 4)).copy()
        for i in range(n):
            k = N_PER_JOINT * i
            D = delta_transform(p[k:k + 3], p[k + 3:k + 6])
            T = T @ (self.pre[i] @ D)
            T = T @ _axis_angle_batch(self.axes[i], q[:, i] + p[k + 6])
        post = self.post.copy()
        post[:3, 3] = post[:3, 3] + post[:3, :3] @ p[-3:]
        T = T @ post
        return T[:, :3, 3]

    # ------------------------------------------------------------ identification
    def residuals(self, p: np.ndarray, q: np.ndarray, x_meas: np.ndarray) -> np.ndarray:
        return (self.fk_positions(q, p) - x_meas).ravel()

    def jacobian(self, p: np.ndarray, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        base = self.fk_positions(q, p).ravel()
        J = np.empty((base.size, len(p)))
        for k in range(len(p)):
            dp = p.copy()
            dp[k] += eps
            J[:, k] = (self.fk_positions(q, dp).ravel() - base) / eps
        return J

    def identifiability(self, q: np.ndarray, p: np.ndarray | None = None, rel_tol: float = 1e-6) -> Tuple[int, np.ndarray]:
        """Numerical rank and singular values of the (column-scaled) Jacobian."""
        p = np.zeros(self.n_params) if p is None else p
        J = self.jacobian(p, q)
        s = np.linalg.svd(J / np.maximum(np.linalg.norm(J, axis=0), 1e-12), compute_uv=False)
        return int((s > rel_tol * s[0]).sum()), s

    def fit(self, q: np.ndarray, x_meas: np.ndarray, reg: float = 1e-6, iters: int = 20,
            tol: float = 1e-10) -> Tuple[np.ndarray, List[float]]:
        """Gauss-Newton with Tikhonov regularisation (min ||r||^2 + reg*N*||p||^2).
        Returns parameters and the RMS position error history (same units as x_meas)."""
        p = np.zeros(self.n_params)
        lam = reg * len(q)
        hist = []
        for _ in range(iters):
            r = self.residuals(p, q, x_meas)
            hist.append(float(np.sqrt((r.reshape(-1, 3) ** 2).sum(1).mean())))
            J = self.jacobian(p, q)
            A = J.T @ J + lam * np.eye(len(p))
            step = np.linalg.solve(A, -(J.T @ r + lam * p))
            p = p + step
            if np.linalg.norm(step) < tol:
                break
        r = self.residuals(p, q, x_meas)
        hist.append(float(np.sqrt((r.reshape(-1, 3) ** 2).sum(1).mean())))
        return p, hist

    def calibrated_origins(self, p: np.ndarray) -> List[Tuple[np.ndarray, Tuple[float, float, float]]]:
        """New (xyz, rpy) for each actuated joint's combined pre-transform, with the joint zero
        offset folded in as a rotation about the joint axis (URDF has no offset field)."""
        out = []
        for i in range(len(self.pre)):
            k = N_PER_JOINT * i
            T = self.pre[i] @ delta_transform(p[k:k + 3], p[k + 3:k + 6]) @ _axis_angle_batch(self.axes[i], np.array([p[k + 6]]))[0]
            out.append((T[:3, 3].copy(), matrix_to_rpy(T[:3, :3])))
        return out


def dedupe_poses(q: np.ndarray, x: np.ndarray, resolution: float) -> Tuple[np.ndarray, np.ndarray]:
    """Average samples that fall in the same joint-space cell (static periods would
    otherwise dominate the fit). resolution in the units of q."""
    keys = np.round(q / resolution).astype(np.int64)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.ravel()
    n = inv.max() + 1
    cnt = np.bincount(inv, minlength=n)[:, None]
    qa = np.zeros((n, q.shape[1]))
    xa = np.zeros((n, x.shape[1]))
    np.add.at(qa, inv, q)
    np.add.at(xa, inv, x)
    return qa / cnt, xa / cnt


def rms(e: np.ndarray) -> float:
    return float(math.sqrt((e ** 2).sum(axis=1).mean()))
