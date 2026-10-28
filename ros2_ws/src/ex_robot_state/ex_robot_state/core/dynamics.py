"""Rigid-body inverse dynamics that is LINEAR in the inertial parameters (ROS-free, numpy only).

    tau = Y(q, qd, qdd) @ theta

theta per moving link (10, about the LINK ORIGIN, in link coordinates):
    [m, m*cx, m*cy, m*cz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
optional per-joint columns (appended in this order, 6 each):
    armature   qdd_j                  reflected rotor inertia
    viscous    qd_j
    coulomb    tanh(qd_j / v_eps)     smooth sign
    offset     1                      torque-sensor / current bias

Recursion: classic Newton-Euler in link frames (Craig), gravity via base acceleration +g.
Serial chains only; each actuated joint's child link carries the mass, massless fixed
joints after the last actuated joint are ignored (flange/tcp frames).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .se3 import axis_angle_to_matrix, rpy_to_matrix
from .urdf_model import UrdfModel

N_LINK_PARAMS = 10
EXTRA_COLUMNS = ("armature", "viscous", "coulomb", "offset")


@dataclass
class Inertial:
    mass: float
    com: np.ndarray            # (3,) in link frame
    inertia_com: np.ndarray    # (3,3) about the COM, link-frame axes


def parse_inertials(xml: str) -> Dict[str, Inertial]:
    out: Dict[str, Inertial] = {}
    for link in ET.fromstring(xml).findall("link"):
        el = link.find("inertial")
        if el is None:
            continue
        o = el.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        i = el.find("inertia").attrib
        Ic = np.array([[float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
                       [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
                       [float(i["ixz"]), float(i["iyz"]), float(i["izz"])]])
        R = rpy_to_matrix(*rpy)
        out[link.get("name")] = Inertial(float(el.find("mass").get("value")), xyz, R @ Ic @ R.T)
    return out


def standard_params(inr: Inertial) -> np.ndarray:
    """Inertial -> theta (10) about the link origin (parallel-axis theorem)."""
    m, c = inr.mass, inr.com
    Io = inr.inertia_com + m * (c @ c * np.eye(3) - np.outer(c, c))
    return np.array([m, *(m * c), Io[0, 0], Io[0, 1], Io[0, 2], Io[1, 1], Io[1, 2], Io[2, 2]])


def add_point_mass(theta_link: np.ndarray, mass: float, pos: Sequence[float]) -> np.ndarray:
    """theta of a link plus a point mass at `pos` (link frame), e.g. a tool on Link6."""
    p = np.asarray(pos, float)
    Io = mass * (p @ p * np.eye(3) - np.outer(p, p))
    return theta_link + np.array([mass, *(mass * p), Io[0, 0], Io[0, 1], Io[0, 2], Io[1, 1], Io[1, 2], Io[2, 2]])


def _skew(v: np.ndarray) -> np.ndarray:
    """(N,3) -> (N,3,3)"""
    S = np.zeros(v.shape[:-1] + (3, 3))
    S[..., 0, 1], S[..., 0, 2] = -v[..., 2], v[..., 1]
    S[..., 1, 0], S[..., 1, 2] = v[..., 2], -v[..., 0]
    S[..., 2, 0], S[..., 2, 1] = -v[..., 1], v[..., 0]
    return S


def _L(w: np.ndarray) -> np.ndarray:
    """(N,3) -> (N,3,6) with L(w) @ [Ixx,Ixy,Ixz,Iyy,Iyz,Izz] = I @ w"""
    L = np.zeros(w.shape[:-1] + (3, 6))
    x, y, z = w[..., 0], w[..., 1], w[..., 2]
    L[..., 0, 0], L[..., 0, 1], L[..., 0, 2] = x, y, z
    L[..., 1, 1], L[..., 1, 3], L[..., 1, 4] = x, y, z
    L[..., 2, 2], L[..., 2, 4], L[..., 2, 5] = x, y, z
    return L


class ChainDynamics:
    def __init__(self, xml: str, tip_link: str = "Link6", base_link: str = "base_link",
                 gravity: Sequence[float] = (0.0, 0.0, -9.81)) -> None:
        model = UrdfModel.from_string(xml)
        self.joints = model.actuated_joints(tip_link, base_link)
        full = model.chain(tip_link, base_link)
        if [j.name for j in full] != [j.name for j in self.joints]:
            raise ValueError("fixed joints inside the arm chain are not supported")
        self.n = len(self.joints)
        self.links = [j.child for j in self.joints]
        self.R0 = np.array([j.origin[:3, :3] for j in self.joints])
        self.p = np.array([j.origin[:3, 3] for j in self.joints])
        self.axis = np.array([j.axis for j in self.joints])
        self.g = np.asarray(gravity, float)
        inr = parse_inertials(xml)
        self.theta_urdf = np.concatenate([standard_params(inr[l]) for l in self.links])

    @property
    def n_rigid(self) -> int:
        return N_LINK_PARAMS * self.n

    def column_names(self, extras: Sequence[str] = EXTRA_COLUMNS) -> List[str]:
        names = [f"{l}.{p}" for l in self.links for p in ("m", "mcx", "mcy", "mcz", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz")]
        return names + [f"{e}{j + 1}" for e in extras for j in range(self.n)]

    def regressor(self, q, qd, qdd, extras: Sequence[str] = EXTRA_COLUMNS, v_eps: float = 0.01) -> np.ndarray:
        """q, qd, qdd: (N, n) in rad, rad/s, rad/s^2  ->  Y: (N, n, 10 n + 6 len(extras))"""
        q, qd, qdd = (np.atleast_2d(np.asarray(a, float)) for a in (q, qd, qdd))
        N, n = q.shape
        R = np.empty((N, n, 3, 3))
        for i in range(n):  # parent->child rotation at q
            rot = np.array([axis_angle_to_matrix(self.axis[i], a) for a in q[:, i]])
            R[:, i] = self.R0[i] @ rot
        z = self.axis
        w = np.zeros((N, 3)); wd = np.zeros((N, 3)); a = np.tile(-self.g, (N, 1))
        A = np.empty((N, n, 6, N_LINK_PARAMS))
        for i in range(n):
            Rt = np.swapaxes(R[:, i], 1, 2)
            p = self.p[i]
            a = np.einsum("nij,nj->ni", Rt, a + np.cross(wd, p) + np.cross(w, np.cross(w, p)))
            w_par = np.einsum("nij,nj->ni", Rt, w)
            w = w_par + qd[:, i:i + 1] * z[i]
            wd = np.einsum("nij,nj->ni", Rt, wd) + np.cross(w_par, qd[:, i:i + 1] * z[i]) + qdd[:, i:i + 1] * z[i]
            Sw, Swd = _skew(w), _skew(wd)
            Ai = np.zeros((N, 6, N_LINK_PARAMS))
            Ai[:, 0:3, 0] = a
            Ai[:, 0:3, 1:4] = Swd + Sw @ Sw
            Ai[:, 3:6, 1:4] = -_skew(a)
            Ai[:, 3:6, 4:10] = _L(wd) + Sw @ _L(w)
            A[:, i] = Ai
        Y = np.zeros((N, n, self.n_rigid))
        for i in range(n):              # contribution of link i ...
            F, M = A[:, i, 0:3].copy(), A[:, i, 3:6].copy()
            for j in range(i, -1, -1):  # ... propagated down to joint j
                Y[:, j, N_LINK_PARAMS * i:N_LINK_PARAMS * (i + 1)] = np.einsum("k,nkc->nc", z[j], M)
                if j > 0:
                    Rj = R[:, j]
                    F = Rj @ F
                    M = Rj @ M + np.cross(self.p[j][None, :, None], F, axis=1)
        cols = [Y]
        for e in extras:
            D = np.zeros((N, n, n))
            src = {"armature": qdd, "viscous": qd, "coulomb": np.tanh(qd / v_eps), "offset": np.ones_like(qd)}[e]
            idx = np.arange(n)
            D[:, idx, idx] = src
            cols.append(D)
        return np.concatenate(cols, axis=2)


def base_columns(Y_stacked: np.ndarray, tol: float = 1e-3) -> Tuple[np.ndarray, Tuple[np.ndarray, np.ndarray]]:
    """Numerically identifiable column subset, from data with rich (e.g. random) excitation.

    Columns are scaled to unit norm first (masses and inertias differ by orders of magnitude);
    the rank comes from the singular-value gap (tol relative to the largest), QR with pivoting
    picks which columns to keep. Returns (independent column indices, (dependent, beta)) with
    Y[:, dep] ~= Y[:, ind] @ beta, so the base parameters are theta_b = theta[ind] + beta @ theta[dep].

    On the calibrated CR10 the micro-radian joint-frame tilts leave one direction at ~1.6e-4 (vs
    ~0.4 for the weakest real one and ~1e-12 for structural zeros): treated as unidentifiable."""
    import scipy.linalg
    nrm = np.linalg.norm(Y_stacked, axis=0)
    live = np.where(nrm > 1e-9 * nrm.max())[0]
    Yn = Y_stacked[:, live] / nrm[live]
    s = np.linalg.svd(Yn, compute_uv=False)
    r = int(np.sum(s > tol * s[0]))
    _, _, piv = scipy.linalg.qr(Yn, mode="economic", pivoting=True)
    ind = np.sort(live[piv[:r]])
    dep = np.setdiff1d(np.arange(Y_stacked.shape[1]), ind)
    beta = np.linalg.lstsq(Y_stacked[:, ind], Y_stacked[:, dep], rcond=None)[0]
    return ind, (dep, beta)


def to_base(theta: np.ndarray, ind: np.ndarray, dep_beta) -> np.ndarray:
    dep, beta = dep_beta
    return theta[ind] + (beta @ theta[dep] if len(dep) else 0.0)


def random_states(n: int, N: int, rng: np.random.Generator, lower=None, upper=None) -> Tuple[np.ndarray, ...]:
    lo = np.full(n, -math.pi) if lower is None else np.asarray(lower)
    hi = np.full(n, math.pi) if upper is None else np.asarray(upper)
    return rng.uniform(lo, hi, (N, n)), rng.normal(0, 1.5, (N, n)), rng.normal(0, 3.0, (N, n))
