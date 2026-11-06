"""Execution model of the SIMULATED command channel (SIMULATION ONLY).

Why this module exists
----------------------
The learning plant executes a joint-velocity command through a CHANNEL, not directly:

  * the command is integrated into a position reference ``q_ref`` (once per physics substep),
  * the reference is CLIPPED at the joint limits, so a command can be silently reduced,
  * a PD servo with a gravity/Coriolis feed-forward tracks the reference
    (``ex_mujoco_sim.servo.servo_torque``), and the resulting torque is saturated.

A kinematic safety filter treats the REQUESTED velocity as the EXECUTED velocity. It therefore
omits exactly the dynamics that dominate braking. For this plant the omission has a closed form:
the servo tracks a velocity ramp with a constant lag of ``2*zeta*v/omega`` rad, so when the
command drops to zero the arm coasts a further distance before it stops. The closed form gives
0.036 rad at the maximum command and the 8 Hz bandwidth; the coast **measured on the simulator**
is **14.9 mrad** (0.015 rad) - see ``evaluation/measure_execution.py`` - so the analytic value is
an over-estimate, and the measured one is what the results use.

This module provides

  * ``ServoExecutionModel`` - a second-order model of the channel (reference integration with
    joint-limit clipping, first-order velocity response),
  * ``identify_execution_model`` - fits the model to the SIMULATED plant and reports the
    multi-step prediction error, so the filter's assumptions are measured rather than asserted.

Honesty notes
-------------
* Everything here is SIMULATED. The real CR10A channel (queued MovJ waypoints, a proprietary
  servo, an unspecified latency) is a DIFFERENT object; none of these numbers transfer to it.
* The model omits the gravity/Coriolis feed-forward and the torque saturation. The omission is
  measured by ``identify_execution_model`` and reported, not assumed to be negligible.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np


@dataclass
class ServoExecutionModel:
    """Second-order model of the command channel.

    The plant obeys (per joint, after the gravity/Coriolis feed-forward cancels the bias term)

        qdd = omega^2 * (q_ref - q) + 2*zeta*omega * (v_gain*v_ref - qd)

    with ``q_ref`` integrating the command and being clipped at the joint limits, and ``v_ref``
    the reference velocity (zero on a clipped joint, because a clipped joint is not moving).

    Attributes
    ----------
    omega   closed-loop bandwidth [rad/s] (the servo gains use 2*pi*bandwidth_hz)
    zeta    damping ratio of the closed loop
    v_gain  steady-state velocity gain: the fraction of the commanded reference velocity the
            joint actually reaches (1.0 for the ideal model; < 1 when the torque saturates)
    dt      policy period [s]
    sub     prediction sub-steps per policy step (the plant runs at 1 kHz, the prediction at
            1/sub of the policy rate; 10 sub-steps = 2 ms, which resolves the 22 ms braking
            transient with 11 samples)
    """

    omega: float = 2.0 * math.pi * 8.0
    zeta: float = 0.9
    v_gain: float = 1.0
    dt: float = 0.02
    sub: int = 10

    # ------------------------------------------------------------------ prediction
    def predict(self, q: np.ndarray, qd: np.ndarray, q_ref: np.ndarray,
                v_seq: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Predict the joint trajectory, for a BATCH of commands at once.

        Parameters
        ----------
        q, qd, q_ref   the current state and reference, (6,) or (B, 6)
        v_seq          (H, 6) or (H, B, 6) normalised commands, one per policy step. The filter
                       passes a constant sequence ("if I keep issuing this, here is where I go");
                       the identification passes the commands that were actually issued.
        lo, hi        the joint limits (the reference is clipped at them)

        Returns
        -------
        times   (H*sub + 1,) [s]
        Q       (B, H*sub + 1, 6) predicted joint positions

        The batch dimension exists because the affine decomposition needs 14 predictions (the
        nominal, the zero command and 6 central differences) at every control step; running
        them as one array keeps the prediction inside the 20 ms control budget.
        """
        q = np.atleast_2d(np.asarray(q, float))
        qd = np.atleast_2d(np.asarray(qd, float))
        q_ref = np.atleast_2d(np.asarray(q_ref, float))
        B = q.shape[0]
        v_seq = np.asarray(v_seq, float)
        if v_seq.ndim == 2:
            v_seq = v_seq[:, None, :].repeat(B, axis=1)
        H = v_seq.shape[0]
        dt_p = self.dt / self.sub
        w2 = self.omega ** 2
        two_zw = 2.0 * self.zeta * self.omega
        lo = np.asarray(lo, float)
        hi = np.asarray(hi, float)
        Q = np.empty((B, H * self.sub + 1, 6))
        Q[:, 0] = q
        k = 0
        for step in range(H):
            v = v_seq[step]                                    # (B, 6)
            for _ in range(self.sub):
                q_ref_new = np.clip(q_ref + v * dt_p, lo, hi)
                moved = (q_ref_new != q_ref).astype(float)
                q_ref = q_ref_new
                qdd = w2 * (q_ref - q) + two_zw * (self.v_gain * v * moved - qd)
                qd = qd + qdd * dt_p
                q = q + qd * dt_p
                k += 1
                Q[:, k] = q
        times = np.arange(len(Q[0])) * dt_p
        return times, Q

    def predict_constant(self, q, qd, q_ref, v, lo, hi, horizon: int):
        """Predict with the command HELD constant for `horizon` policy steps."""
        v = np.atleast_2d(np.asarray(v, float))
        v_seq = np.tile(v[None, :, :], (horizon, 1, 1))
        return self.predict(q, qd, q_ref, v_seq, lo, hi)

    def affine(self, q, qd, q_ref, v, lo, hi, horizon: int, step: int = 1,
               eps: float = 0.05) -> Tuple[np.ndarray, np.ndarray]:
        """The affine decomposition of the prediction in the command.

            dQ_k(v)  ~=  dQ_k(0) + G_k . v

        ``dQ_k(0)`` is the displacement the arm makes if the command were ZERO - i.e. the COAST
        that a kinematic filter never accounts for. ``G_k`` is the sensitivity of the predicted
        displacement to the command, obtained by central finite differences (the reference
        clipping makes the response only piecewise affine, so the FD is local).

        All 14 predictions run as one batch.

        Returns
        -------
        dq0   (K, 6)  the coasting displacement at each sample
        G     (K, 6, 6) the sensitivity at each sample
        """
        v = np.clip(np.asarray(v, float), -1.0, 1.0)
        q = np.atleast_2d(np.asarray(q, float))
        qd = np.atleast_2d(np.asarray(qd, float))
        q_ref = np.atleast_2d(np.asarray(q_ref, float))
        B = 14   # the nominal, the zero command, and 6 central differences
        cmds = np.empty((B, 6))
        cmds[0] = v
        cmds[1] = 0.0
        for j in range(6):
            vp = v.copy()
            vm = v.copy()
            vp[j] = min(v[j] + eps, 1.0)
            vm[j] = max(v[j] - eps, -1.0)
            cmds[2 + 2 * j] = vp
            cmds[3 + 2 * j] = vm
        _, Q = self.predict_constant(np.repeat(q, B, axis=0), np.repeat(qd, B, axis=0),
                                      np.repeat(q_ref, B, axis=0), cmds, lo, hi, horizon)
        S = Q[:, step::step, :]                     # (B, K+1, 6) - includes k=0
        dq0 = (S[1, 1:, :] - q[0])                  # the coast: displacement with a zero command
        G = np.empty((dq0.shape[0], 6, 6))
        for j in range(6):
            G[:, :, j] = (S[2 + 2 * j, 1:, :] - S[3 + 2 * j, 1:, :]) / (cmds[2 + 2 * j, j] - cmds[3 + 2 * j, j])
        return dq0, G


# --------------------------------------------------------------------- identification
def _log_rollout(env, policy, n_steps: int, seed: int) -> Dict[str, np.ndarray]:
    """Run one episode and log everything the identification needs.

    The log holds the state, the reference and the command at every step, so a candidate model
    can be scored by replaying the commands and comparing the prediction with what happened.
    """
    obs, _ = env.reset(seed=seed)
    Q, QD, QR, V = [], [], [], []
    for _ in range(n_steps):
        a = np.clip(np.asarray(policy(obs), float), -1.0, 1.0)
        Q.append(env.q())
        QD.append(env.d.qvel[env.dadr].copy())
        QR.append(np.asarray(env.q_ref, float).copy())
        V.append(a.copy())
        obs, _, term, trunc, _ = env.step(a)
        if term or trunc:
            break
    return {"Q": np.array(Q), "QD": np.array(QD), "QR": np.array(QR), "V": np.array(V)}


def _prediction_error(model: ServoExecutionModel, log: Dict[str, np.ndarray],
                      lo: np.ndarray, hi: np.ndarray, horizon: int) -> Dict[str, float]:
    """The RMS joint-position error of the model over the horizon, on ONE logged rollout."""
    errs = []
    n = len(log["V"]) - horizon
    for t in range(max(n, 0)):
        _, Qp = model.predict(log["Q"][t], log["QD"][t], log["QR"][t],
                              log["V"][t:t + horizon], lo, hi)
        actual = log["Q"][t + horizon]
        errs.append(Qp[0, -1] - actual)
    if not errs:
        return {"rms_rad": float("nan"), "max_rad": float("nan"), "n": 0}
    E = np.array(errs)
    return {"rms_rad": float(np.sqrt((E ** 2).mean())), "max_rad": float(np.abs(E).max()),
            "n": len(E)}


def identify_execution_model(env, policy=None, n_rollouts: int = 4, n_steps: int = 120,
                             horizon: int = 5, seeds: Optional[Sequence[int]] = None,
                             omega_grid: Optional[Sequence[float]] = None,
                             zeta_grid: Optional[Sequence[float]] = None,
                             v_gain_grid: Optional[Sequence[float]] = None) -> Tuple[ServoExecutionModel, Dict]:
    """Fit the execution model to the SIMULATED plant and report the prediction error.

    The score is the multi-step prediction error with the commands that were actually issued:
    that is the error the filter inherits, because the filter predicts with a held command.

    Returns the fitted model and a report with the fit, the error, and the SENSITIVITY of the
    error to the parameters - the last one is what tells us whether the filter's advantage
    survives a wrong execution model.
    """
    seeds = list(seeds) if seeds is not None else list(range(1000, 1000 + n_rollouts))
    logs = []
    for s in seeds:
        if policy is None:
            rng = np.random.default_rng(s)
            pol = lambda obs, rng=rng: rng.uniform(-1.0, 1.0, 6)  # noqa: E731
        else:
            pol = policy
        logs.append(_log_rollout(env, pol, n_steps, s))

    lo = np.asarray(env.lo, float)
    hi = np.asarray(env.hi, float)
    omega_grid = list(omega_grid) if omega_grid is not None else \
        [0.5 * 2 * math.pi * 8, 0.75 * 2 * math.pi * 8, 2 * math.pi * 8, 1.25 * 2 * math.pi * 8, 1.5 * 2 * math.pi * 8]
    zeta_grid = list(zeta_grid) if zeta_grid is not None else [0.7, 0.8, 0.9, 1.0, 1.1]
    v_gain_grid = list(v_gain_grid) if v_gain_grid is not None else [0.8, 0.9, 0.95, 1.0, 1.05]

    best, best_err, table = None, float("inf"), []
    for om in omega_grid:
        for ze in zeta_grid:
            for vg in v_gain_grid:
                m = ServoExecutionModel(omega=om, zeta=ze, v_gain=vg, dt=env.cfg.control_dt)
                rms = np.mean([_prediction_error(m, lg, lo, hi, horizon)["rms_rad"] for lg in logs])
                table.append({"omega": om, "zeta": ze, "v_gain": vg, "rms_rad": float(rms)})
                if rms < best_err:
                    best_err, best = rms, m
    report = {
        "model": {"omega": best.omega, "zeta": best.zeta, "v_gain": best.v_gain,
                  "dt": best.dt, "sub": best.sub},
        "prediction_error": {"rms_rad": float(best_err), "horizon_steps": horizon,
                             "horizon_s": horizon * env.cfg.control_dt},
        "n_rollouts": len(logs), "n_steps": n_steps,
        "grid": table,
    }
    return best, report


def coast_distance(env, model: ServoExecutionModel, v: np.ndarray = None,
                   settle_s: float = 0.5) -> Dict[str, float]:
    """Measure the COAST: how far the arm travels after the command is cut to zero.

    This is the quantity a kinematic safety filter ignores, and it is measured on the SIMULATED
    plant (not from the model), so the number in the paper is a measurement.
    """
    v = np.full(6, 0.5) if v is None else np.asarray(v, float)
    lo, hi = np.asarray(env.lo, float), np.asarray(env.hi, float)
    obs, _ = env.reset(seed=7)
    # 1) let the servo reach the commanded velocity
    n_settle = int(settle_s / env.cfg.control_dt)
    for _ in range(n_settle):
        obs, _, term, trunc, _ = env.step(v)
        if term or trunc:
            break
    q_at_cut = env.q()
    v_at_cut = env.d.qvel[env.dadr].copy()
    # 2) cut the command and let the arm coast
    n_coast = int(0.6 / env.cfg.control_dt)
    for _ in range(n_coast):
        obs, _, term, trunc, _ = env.step(np.zeros(6))
        if term or trunc:
            break
    q_end = env.q()
    actual = np.abs(q_end - q_at_cut)
    # 3) what the model predicts for the same cut
    _, Qp = model.predict_constant(q_at_cut, v_at_cut, np.asarray(env.q_ref, float),
                                   np.zeros(6), lo, hi, horizon=30)
    # the reference keeps its value during the coast (the command is zero), so predict with it
    return {"command_rad_s": float(np.max(np.abs(v))),
            "velocity_at_cut_rad_s": float(np.max(np.abs(v_at_cut))),
            "coast_rad": float(actual.max()),
            "coast_rad_mean": float(actual.mean()),
            "coast_ratio": float(actual.max() / (np.max(np.abs(v)) + 1e-9)),
            "pred_coast_rad": float(np.abs(Qp[0, -1] - q_at_cut).max())}
