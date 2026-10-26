"""Signal-level checks used by pipeline_check (ROS-free, unit tested)."""

from __future__ import annotations

import math
from typing import List, Sequence

import numpy as np


def frequency_amplitudes(t: Sequence[float], y: Sequence[float], freqs: Sequence[float]) -> List[float]:
    """Joint least-squares fit y ~ c + sum_k a_k sin(2 pi f_k t) + b_k cos(2 pi f_k t).

    A joint fit (rather than one projection per frequency) separates closely
    spaced frequencies even when the window is shorter than 1/df.
    Returns the amplitude sqrt(a_k^2 + b_k^2) for each candidate frequency.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    cols = [np.ones_like(t)]
    for f in freqs:
        w = 2 * math.pi * f
        cols += [np.sin(w * t), np.cos(w * t)]
    X = np.stack(cols, axis=1)
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    return [float(math.hypot(coef[1 + 2 * k], coef[2 + 2 * k])) for k in range(len(freqs))]


def dominant_frequency_index(t: Sequence[float], y: Sequence[float], freqs: Sequence[float]) -> int:
    amps = frequency_amplitudes(t, y, freqs)
    return int(np.argmax(amps))
