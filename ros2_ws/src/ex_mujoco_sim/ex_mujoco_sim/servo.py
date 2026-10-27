"""Joint servo-drive model for the simulated plant (ROS-free, unit tested).

A real servo drive receives position set-points at the controller rate (here: ros2_control
JointTrajectoryController, 100 Hz) and closes a much faster loop. We model it as
    tau = clip( kp (q_ref - q) + kd (dq_ref - dq) + tau_bias , +-tau_max )
with
  * q_ref, dq_ref : first-order-hold extrapolation of the 100 Hz set-points (drives interpolate;
                    a zero-order hold would inject a 100 Hz staircase)
  * tau_bias      : gravity + Coriolis compensation from the model (MuJoCo qfrc_bias), i.e.
                    a perfect-model feedforward - an OPTIMISTIC assumption, made explicit
  * kp, kd        : from the joint's effective inertia M_ii and a target bandwidth:
                    kp = M w^2, kd = 2 zeta M w  (critically damped 2nd-order joint for zeta=1)
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass
class ServoGains:
    kp: np.ndarray
    kd: np.ndarray


def gains_from_inertia(m_eff: Sequence[float], bandwidth_hz: float, zeta: float) -> ServoGains:
    m = np.asarray(m_eff, float)
    w = 2 * math.pi * bandwidth_hz
    return ServoGains(kp=m * w * w, kd=2 * zeta * m * w)


class CommandInterpolator:
    """First-order hold on set-points, extrapolation capped at `max_horizon` s.

    Velocity feed-forward is differenced on the SENDER timestamps when available: arrival
    times jitter 3.9-16 ms for a 10 ms command stream over ROS (measured 2026-10-27, sender
    stamps 8.9-11 ms), and differencing arrival times inflated the velocity estimate up to
    ~2.5x -> kd torque spikes -> divergence near torque limits. The estimate is additionally
    low-pass filtered (first order, `vel_filter_tau` s)."""

    def __init__(self, n: int, max_horizon: float = 0.02, vel_filter_tau: float = 0.02) -> None:
        self.n = n
        self.max_horizon = max_horizon
        self.tau = vel_filter_tau
        self.t_last: Optional[float] = None
        self.src_last: Optional[float] = None
        self.q_last = np.zeros(n)
        self.v = np.zeros(n)

    def update(self, t: float, q: Sequence[float], t_src: Optional[float] = None) -> None:
        q = np.asarray(q, float)
        ref_prev = self.src_last if (t_src is not None and self.src_last is not None) else self.t_last
        ref_now = t_src if (t_src is not None and self.src_last is not None) else t
        if self.t_last is not None and ref_prev is not None and ref_now > ref_prev:
            dt = ref_now - ref_prev
            v_raw = (q - self.q_last) / dt
            a = dt / (self.tau + dt) if self.tau > 0 else 1.0
            self.v = self.v + a * (v_raw - self.v)
        elif self.t_last is None:
            self.v = np.zeros(self.n)
        self.t_last, self.q_last = t, q
        self.src_last = t_src

    def has_command(self) -> bool:
        return self.t_last is not None

    def reference(self, t: float):
        if self.t_last is None:
            raise RuntimeError("no command yet")
        h = min(max(t - self.t_last, 0.0), self.max_horizon)
        v = self.v if (t - self.t_last) <= self.max_horizon else np.zeros(self.n)  # stale -> hold
        return self.q_last + self.v * h, v


def servo_torque(q, dq, q_ref, dq_ref, bias, gains: ServoGains, tau_max) -> np.ndarray:
    tau = gains.kp * (np.asarray(q_ref) - q) + gains.kd * (np.asarray(dq_ref) - dq) + bias
    return np.clip(tau, -np.asarray(tau_max), np.asarray(tau_max))
