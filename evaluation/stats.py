"""Statistics for the learning experiments (SIMULATION ONLY).

What this module is for
-----------------------
The manuscript's protocol section (6.3) promises a statistics treatment, and this is it. The
problem is specific and it is easy to get wrong:

* the SAME 50 scenario seeds are evaluated for several METHODS and for several TRAINING SEEDS;
* the episodes are therefore correlated in two directions:
  - within a cell, across methods (the same scenario, the same wall, the same start pose);
  - across cells, through the training seed (the same policy, evaluated on 50 scenarios).

A naive per-episode test treats all of those observations as independent, and the interval it
produces is far too narrow. That is not a hypothetical: the study's headline comparison comes
out at +22, +2 and +2 percentage points across three training seeds, with a between-seed spread
of 8 points, and the naive reading of that is "a 8.7-point win" - which three seeds cannot
support.

The estimators here
-------------------
``paired_difference``      the per-scenario difference between two methods, with a bootstrap
                           interval that resamples SCENARIOS and preserves the pairing.
``hierarchical_bootstrap`` the interval that answers "would this hold on a new training seed
                           and a new set of scenarios": resample TRAINING SEEDS with replacement,
                           then SCENARIOS within the selected seed with replacement.
``seed_spread``            the between-training-seed spread, reported ALONGSIDE any mean
                           difference so that a reader can see the two together.
``separability``           an explicit answer to "can these two methods be distinguished with
                           the seeds we have?", not a p-value in disguise.
``binomial_interval``      the one-sided Clopper-Pearson upper bound for zero collisions in N
                           trials.

What this module does NOT license you to claim
----------------------------------------------
* A bootstrap interval is a statement about the sampling procedure, not a guarantee. It says
  nothing about the simulator being an unbiased model of the robot.
* Resampling SCENARIOS does not make the training seeds independent of each other: they share the
  same code, the same reward, the same observation space and the same initialisation scheme.
  The hierarchical interval propagates the seed-level variance, but a shared *bias* (a flaw that
  affects every seed equally) is invisible to any of these estimators.
* The Clopper-Pearson bound assumes INDEPENDENT trials. The same 50 scenarios evaluated under
  three training seeds are 150 policy-episode evaluations, NOT 150 independent scenes; the
  bound computed on the number of evaluations would be optimistic. `binomial_interval` says so
  in its result.
* With three training seeds the hierarchical bootstrap has three clusters. A cluster bootstrap
  with three clusters cannot resolve small differences, and the module says so when
  `separability` is asked.

Every function returns the numbers AND the assumptions it is making, so a caller cannot
accidentally report an interval without its caveats.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "paired_difference",
    "hierarchical_bootstrap",
    "naive_independent_interval",
    "seed_spread",
    "separability",
    "binomial_interval",
]


# --------------------------------------------------------------------------- helpers
def _percentile_ci(samples: np.ndarray, alpha: float) -> Tuple[float, float]:
    lo = float(np.percentile(samples, 100.0 * alpha / 2.0))
    hi = float(np.percentile(samples, 100.0 * (1.0 - alpha / 2.0)))
    return lo, hi


def _as_pairs(values_a: Sequence[float], values_b: Sequence[float]) -> np.ndarray:
    a = np.asarray(values_a, float)
    b = np.asarray(values_b, float)
    if a.shape != b.shape:
        raise ValueError(f"the two methods must be evaluated on the same set: {a.shape} vs {b.shape}")
    if a.ndim != 1:
        raise ValueError("expected one value per scenario")
    return a - b


# --------------------------------------------------------------------- paired scenario
def paired_difference(values_a: Sequence[float], values_b: Sequence[float],
                      n_boot: int = 4000, alpha: float = 0.05, seed: int = 0) -> Dict:
    """The paired difference between two methods over a shared set of scenarios.

    The bootstrap resamples the SCENARIOS (rows), not the individual observations, and the
    pairing is preserved: the same resampled scenario index is used for both methods.

    Returns a dict with the mean difference, the percentile interval, the two-sided bootstrap
    p-value (the fraction of replicates on the other side of zero), and the assumptions.
    """
    diff = _as_pairs(values_a, values_b)
    n = len(diff)
    if n == 0:
        raise ValueError("no observations")
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, n, size=(n_boot, n))
    reps = diff[idx].mean(axis=1)
    lo, hi = _percentile_ci(reps, alpha)
    p = 2.0 * min((reps <= 0).mean(), (reps >= 0).mean())
    return {
        "mean_difference": float(diff.mean()),
        "ci_low": lo, "ci_high": hi,
        "alpha": alpha, "n_boot": n_boot, "n_scenarios": n,
        "p_two_sided_bootstrap": float(min(p, 1.0)),
        "excludes_zero": bool(lo > 0.0 or hi < 0.0),
        "method": "scenario-level bootstrap, pairing preserved",
        "assumes": [
            "the scenarios are exchangeable within the evaluated range",
            "the methods are evaluated on exactly the same scenarios (the pairing is real)",
            "the training seeds are NOT accounted for here - use hierarchical_bootstrap for that",
        ],
    }


# -------------------------------------------------------------------------- hierarchical
def hierarchical_bootstrap(per_seed_a: Dict[object, Dict[object, float]],
                           per_seed_b: Dict[object, Dict[object, float]],
                           n_boot: int = 4000, alpha: float = 0.05,
                           seed: int = 0) -> Dict:
    """The interval for a method difference that accounts for BOTH levels of correlation.

    ``per_seed_a[training_seed][scenario_id]`` is the value method A produced for that
    (training seed, scenario) cell; ``per_seed_b`` the same for method B. The two must cover the
    same seeds and the same scenarios.

    Each bootstrap replicate:
      1. draws ``n_seeds`` training seeds WITH replacement (the seed is the cluster);
      2. for each drawn seed, draws ``n_scenarios`` scenarios WITH replacement;
      3. computes the paired difference on the resampled scenarios;
      4. averages over the resampled seeds.

    Returns the mean difference, the percentile interval, the between-seed spread of the
    per-seed means, and an explicit note about how many clusters the interval rests on.
    """
    seeds = sorted(set(per_seed_a) & set(per_seed_b), key=str)
    if not seeds:
        raise ValueError("no common training seeds")
    ids = None
    per_seed_diff = []
    for s in seeds:
        sa, sb = per_seed_a[s], per_seed_b[s]
        if set(sa) != set(sb):
            raise ValueError(f"the methods cover different scenarios for seed {s!r}")
        if ids is None:
            ids = sorted(sa, key=str)
        elif sorted(sa, key=str) != ids:
            raise ValueError(f"the scenarios differ between seeds (seed {s!r})")
        per_seed_diff.append(np.array([float(sa[i]) - float(sb[i]) for i in ids]))
    n_seeds, n_scen = len(per_seed_diff), len(ids)
    if n_seeds < 2:
        raise ValueError("a hierarchical bootstrap needs at least two training seeds")

    rng = np.random.default_rng(seed)
    reps = np.empty(n_boot)
    for b in range(n_boot):
        pick = rng.integers(0, n_seeds, size=n_seeds)
        cols = rng.integers(0, n_scen, size=(n_seeds, n_scen))
        reps[b] = float(np.mean([per_seed_diff[pick[k]][cols[k]].mean() for k in range(n_seeds)]))
    lo, hi = _percentile_ci(reps, alpha)
    p = 2.0 * min((reps <= 0).mean(), (reps >= 0).mean())
    means = np.array([d.mean() for d in per_seed_diff])
    return {
        "mean_difference": float(means.mean()),
        "ci_low": lo, "ci_high": hi,
        "alpha": alpha, "n_boot": n_boot,
        "n_training_seeds": n_seeds, "n_scenarios": n_scen,
        "per_seed_mean_difference": [float(v) for v in means],
        "between_seed_sd": float(means.std(ddof=1)),
        "p_two_sided_bootstrap": float(min(p, 1.0)),
        "excludes_zero": bool(lo > 0.0 or hi < 0.0),
        "method": "two-level bootstrap: training seeds, then scenarios within a seed",
        "assumes": [
            "the training seeds are exchangeable draws from the training procedure",
            "the scenarios are exchangeable within the evaluated range",
            "a bias shared by ALL seeds (a flaw of the method, the reward or the environment) is "
            "invisible to this estimator - it propagates variance, not shared bias",
            f"the interval rests on {n_seeds} clusters; with few clusters it cannot resolve small "
            "differences, and the interval should be read as descriptive",
        ],
    }


def naive_independent_interval(per_seed_a: Dict[object, Dict[object, float]],
                                per_seed_b: Dict[object, Dict[object, float]],
                                n_boot: int = 4000, alpha: float = 0.05,
                                seed: int = 0) -> Dict:
    """The interval a naive analysis would produce: every (seed, scenario) cell independent.

    It is here for ONE reason: to show how much narrower it is than the hierarchical one, and
    therefore how much a naive analysis over-claims. It should never be reported as the result.
    """
    a = np.array([v for s in sorted(per_seed_a, key=str) for v in per_seed_a[s].values()], float)
    b = np.array([v for s in sorted(per_seed_b, key=str) for v in per_seed_b[s].values()], float)
    if a.shape != b.shape:
        raise ValueError("the two methods must have the same number of cells")
    diff = a - b
    n = len(diff)
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, n, size=(n_boot, n))
    reps = diff[idx].mean(axis=1)
    lo, hi = _percentile_ci(reps, alpha)
    return {"mean_difference": float(diff.mean()), "ci_low": lo, "ci_high": hi,
            "width": hi - lo, "alpha": alpha, "n_boot": n_boot, "n_cells": n,
            "method": "NAIVE: cells treated as independent - for comparison only, do not report"}


# ------------------------------------------------------------------------------ spread
def seed_spread(per_seed_values: Dict[object, Sequence[float]]) -> Dict:
    """The between-training-seed spread of a method's mean.

    This is the number that decides whether a mean difference means anything: a difference of
    the same size as the spread cannot be separated from it with the seeds at hand.
    """
    means = [float(np.mean(np.asarray(list(v), float))) for v in per_seed_values.values()]
    if len(means) < 2:
        raise ValueError("the spread needs at least two training seeds")
    m = np.asarray(means, float)
    return {
        "per_seed_mean": [float(x) for x in m],
        "mean": float(m.mean()),
        "sd": float(m.std(ddof=1)),
        "range": float(m.max() - m.min()),
        "n_seeds": len(m),
        "interpretation": "a mean difference of the same order as this spread cannot be "
                          "separated from it with the seeds at hand",
    }


def separability(mean_difference: float, spread_sd: float, n_seeds: int,
                 ci_low: Optional[float] = None, ci_high: Optional[float] = None) -> Dict:
    """An explicit, non-p-value answer to "can these two methods be distinguished?".

    The rule is deliberately conservative and stated up front: a difference is only called
    SEPARABLE when the interval excludes zero AND the between-seed spread is not of the same
    order as the difference. Three seeds cannot resolve a small difference, and the module says
    so rather than letting a p-value imply more than the data supports.
    """
    ratio = abs(mean_difference) / spread_sd if spread_sd > 0 else float("inf")
    enough_seeds = n_seeds >= 5
    excludes_zero = None if ci_low is None else bool(ci_low > 0.0 or ci_high < 0.0)
    if excludes_zero is None:
        verdict = "UNDETERMINED"
        why = "no interval was supplied"
    elif not excludes_zero:
        verdict = "NOT_SEPARABLE"
        why = "the interval includes zero"
    elif not enough_seeds:
        verdict = "NOT_SEPARABLE"
        why = (f"the interval excludes zero, but {n_seeds} training seeds cannot resolve a "
               f"difference of {abs(mean_difference)*100:.1f} percentage points against a "
               f"between-seed spread of {spread_sd*100:.1f} points")
    elif ratio < 1.0:
        verdict = "NOT_SEPARABLE"
        why = (f"the between-seed spread ({spread_sd*100:.1f} points) is of the same order as the "
               f"difference ({abs(mean_difference)*100:.1f} points)")
    else:
        verdict = "SEPARABLE"
        why = (f"the interval excludes zero, {n_seeds} seeds are available, and the difference is "
               f"{ratio:.1f} times the between-seed spread")
    return {"verdict": verdict, "why": why,
            "mean_difference": mean_difference, "spread_sd": spread_sd,
            "difference_over_spread": ratio, "n_seeds": n_seeds,
            "ci_low": ci_low, "ci_high": ci_high}


# --------------------------------------------------------------------------- binomial
def binomial_interval(k: int, n: int, confidence: float = 0.95, one_sided: bool = True) -> Dict:
    """The Clopper-Pearson interval for k successes in n trials.

    For k = 0 the one-sided upper bound is ``1 - alpha**(1/n)`` in closed form; for 50 trials at
    95 % that is 5.83 %, the number the manuscript quotes.

    **The trials must be independent.** The same 50 scenarios evaluated under three training
    seeds are 150 policy-episode evaluations, not 150 independent scenes: the bound computed on
    150 would be optimistic, because the scenario effect is shared. The result says so.
    """
    if n <= 0:
        raise ValueError("n must be positive")
    if not (0 <= k <= n):
        raise ValueError(f"k must be in [0, {n}]")
    alpha = 1.0 - confidence
    from scipy.stats import beta
    if one_sided:
        upper = float(beta.ppf(1.0 - alpha, k + 1, n - k))
        lower = 0.0
    else:
        lower = float(beta.ppf(alpha / 2.0, k + 1, n - k))
        upper = float(beta.ppf(1.0 - alpha / 2.0, k + 1, n - k))
    closed_form = None
    if k == 0 and one_sided:
        closed_form = 1.0 - alpha ** (1.0 / n)
    return {
        "k": k, "n": n, "confidence": confidence, "one_sided": one_sided,
        "lower": lower, "upper": upper,
        "closed_form_zero_success_upper": closed_form,
        "point_estimate": k / n,
        "assumes": [
            "the trials are INDEPENDENT - repeated evaluations of the same scenarios under "
            "different training seeds are NOT independent trials, and a bound computed on the "
            "number of evaluations would be optimistic",
            "the trials are exchangeable draws from the same process",
        ],
    }


def zero_collision_bound(n: int, confidence: float = 0.95) -> float:
    """The one-sided upper bound on the collision probability after n trials with no collision."""
    return float(binomial_interval(0, n, confidence=confidence, one_sided=True)["upper"])
