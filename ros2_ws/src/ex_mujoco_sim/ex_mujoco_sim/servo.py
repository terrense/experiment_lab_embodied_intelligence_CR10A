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
    """First-order hold on irregular set-points, extrapolation capped at `max_horizon` s."""

    def __init__(self, n: int, max_horizon: float = 0.02) -> None:
        self.n = n
        self.max_horizon = max_horizon
        self.t_last: Optional[float] = None
        self.q_last = np.zeros(n)
        self.v = np.zeros(n)

    def update(self, t: float, q: Sequence[float]) -> None:
        q = np.asarray(q, float)
        if self.t_last is not None and t > self.t_last:
            self.v = (q - self.q_last) / (t - self.t_last)
        else:
            self.v = np.zeros(self.n)
        self.t_last, self.q_last = t, q

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
