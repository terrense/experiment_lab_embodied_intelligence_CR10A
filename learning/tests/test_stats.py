"""Tests for the statistics module (evaluation/stats.py).

The point of the module is that the naive analysis is wrong in a specific way, so the tests
check the naive interval against the hierarchical one on data where the truth is known.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "evaluation"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stats import (binomial_interval, hierarchical_bootstrap, naive_independent_interval,  # noqa: E402
                   paired_difference, seed_spread, separability, zero_collision_bound)


def _cells(per_seed_values):
    """{seed: {scenario: value}} from {seed: [values]}."""
    return {s: {f"sc{i}": v for i, v in enumerate(vals)} for s, vals in per_seed_values.items()}


def test_a_constant_paired_difference_is_recovered_and_excludes_zero():
    a = [0.5, 0.4, 0.6, 0.55, 0.45, 0.52, 0.48, 0.5, 0.47, 0.53]
    b = [x - 0.12 for x in a]
    r = paired_difference(a, b, n_boot=2000, seed=1)
    assert abs(r["mean_difference"] - 0.12) < 1e-12
    assert r["ci_low"] > 0.0, "a constant offset must give an interval that excludes zero"
    assert r["excludes_zero"]


def test_identical_methods_give_an_interval_that_includes_zero():
    rng = np.random.default_rng(0)
    x = rng.normal(0.5, 0.1, 60)
    r = paired_difference(x, x.copy(), n_boot=2000, seed=2)
    assert r["mean_difference"] == 0.0
    assert r["ci_low"] <= 0.0 <= r["ci_high"]
    assert not r["excludes_zero"]


def test_the_pairing_is_real_shuffling_one_method_changes_the_answer():
    """If the pairing were ignored, a shuffled method would look identical to the original.

    The scenario effect has to be CORRELATED for this to show up: with a constant offset the
    shuffle changes nothing, because every difference is the same number either way.
    """
    rng = np.random.default_rng(3)
    scenario = rng.normal(0.5, 0.20, 60)          # a strong per-scenario effect
    b = scenario + rng.normal(0.0, 0.01, 60)      # method B tracks it closely
    paired = paired_difference(b, scenario, n_boot=2000, seed=4)
    shuffled = paired_difference(b, rng.permutation(scenario), n_boot=2000, seed=4)
    assert abs(paired["mean_difference"] - shuffled["mean_difference"]) < 1e-12, \
        "the mean is the same either way - it is the INTERVAL that the pairing changes"
    assert (paired["ci_high"] - paired["ci_low"]) < (shuffled["ci_high"] - shuffled["ci_low"]), \
        "the truly paired interval must be the narrower one"


def test_the_hierarchical_interval_is_wider_than_the_naive_one_when_scenarios_are_shared():
    """This is the whole point of the module.

    The scenario effect is shared by every training seed (the same wall, the same start pose),
    and the training seed shifts the difference itself. A naive per-cell analysis treats the
    (seed, scenario) cells as independent: it divides the seed-level variance by the number of
    cells and produces an interval that is far too narrow.
    """
    rng = np.random.default_rng(7)
    n_scen, n_seeds = 50, 6
    scenario_effect = rng.normal(0.0, 0.30, n_scen)     # shared by every seed
    delta = rng.normal(0.0, 0.08, n_seeds)              # the per-seed shift of the difference
    a, b = {}, {}
    for s in range(n_seeds):
        a[s] = {f"sc{i}": 0.6 + 0.1 * scenario_effect[i] for i in range(n_scen)}
        b[s] = {f"sc{i}": 0.6 + 0.1 * scenario_effect[i] - delta[s] for i in range(n_scen)}
    hier = hierarchical_bootstrap(a, b, n_boot=2000, seed=8)
    naive = naive_independent_interval(a, b, n_boot=2000, seed=8)
    hier_w = hier["ci_high"] - hier["ci_low"]
    naive_w = naive["ci_high"] - naive["ci_low"]
    assert hier_w > naive_w, \
        f"the hierarchical interval ({hier_w:.4f}) must be wider than the naive one ({naive_w:.4f})"
    # and the naive one must be badly wrong: it divides the seed-level spread by 300 cells
    assert naive_w < 0.5 * hier_w, "the naive interval is not just a little narrower - it is wrong"


def test_the_hierarchical_bootstrap_recovers_a_known_difference():
    rng = np.random.default_rng(11)
    n_scen, n_seeds, delta = 40, 5, 0.08
    a, b = {}, {}
    for s in range(n_seeds):
        noise = rng.normal(0.0, 0.02, n_scen)
        a[s] = {f"sc{i}": 0.5 + float(noise[i]) for i in range(n_scen)}
        b[s] = {f"sc{i}": 0.5 + float(noise[i]) - delta for i in range(n_scen)}
    r = hierarchical_bootstrap(a, b, n_boot=2000, seed=12)
    assert abs(r["mean_difference"] - delta) < 1e-9
    assert r["excludes_zero"]


def test_identical_methods_give_a_hierarchical_interval_that_includes_zero():
    rng = np.random.default_rng(13)
    a, b = {}, {}
    for s in range(4):
        x = rng.normal(0.5, 0.05, 30)
        a[s] = {f"sc{i}": v for i, v in enumerate(x)}
        b[s] = {f"sc{i}": v for i, v in enumerate(x)}
    r = hierarchical_bootstrap(a, b, n_boot=2000, seed=14)
    assert r["ci_low"] <= 0.0 <= r["ci_high"]
    assert not r["excludes_zero"]


def test_the_seed_spread_is_the_between_seed_quantity():
    per_seed = {0: [0.50, 0.52, 0.48], 1: [0.60, 0.61, 0.59], 2: [0.40, 0.41, 0.39]}
    r = seed_spread(per_seed)
    assert r["n_seeds"] == 3
    assert abs(r["mean"] - 0.5) < 1e-12
    assert abs(r["sd"] - np.std([0.5, 0.6, 0.4], ddof=1)) < 1e-12
    assert abs(r["range"] - 0.2) < 1e-12


def test_separability_is_not_separable_when_the_spread_dominates():
    r = separability(mean_difference=0.087, spread_sd=0.08, n_seeds=3,
                     ci_low=0.01, ci_high=0.16)
    assert r["verdict"] == "NOT_SEPARABLE"
    assert "3 training seeds" in r["why"]


def test_separability_is_separable_when_the_interval_excludes_zero_and_there_are_seeds():
    r = separability(mean_difference=0.20, spread_sd=0.05, n_seeds=9, ci_low=0.12, ci_high=0.28)
    assert r["verdict"] == "SEPARABLE"


def test_the_zero_collision_bound_for_fifty_trials_is_five_point_eight_percent():
    b = binomial_interval(0, 50, confidence=0.95, one_sided=True)
    assert abs(b["upper"] - 0.0583) < 1e-3
    assert abs(b["closed_form_zero_success_upper"] - (1.0 - 0.05 ** (1.0 / 50.0))) < 1e-12
    assert abs(zero_collision_bound(50) - b["upper"]) < 1e-12
    # and the bound must shrink with more trials
    assert zero_collision_bound(200) < zero_collision_bound(50)


def test_the_binomial_interval_carries_the_independence_warning():
    b = binomial_interval(0, 150, confidence=0.95, one_sided=True)
    assert any("NOT independent" in a or "not independent" in a.lower() for a in b["assumes"])
    assert b["n"] == 150


def test_the_estimators_are_deterministic_given_a_seed():
    rng = np.random.default_rng(17)
    a, b = {}, {}
    for s in range(3):
        a[s] = {f"sc{i}": float(v) for i, v in enumerate(rng.normal(0.5, 0.05, 25))}
        b[s] = {f"sc{i}": float(v) for i, v in enumerate(rng.normal(0.4, 0.05, 25))}
    r1 = hierarchical_bootstrap(a, b, n_boot=500, seed=99)
    r2 = hierarchical_bootstrap(a, b, n_boot=500, seed=99)
    r3 = hierarchical_bootstrap(a, b, n_boot=500, seed=100)
    assert r1["ci_low"] == r2["ci_low"] and r1["ci_high"] == r2["ci_high"]
    assert (r1["ci_low"] != r3["ci_low"]) or (r1["ci_high"] != r3["ci_high"])


def test_mismatched_inputs_are_rejected_rather_than_silently_reshaped():
    with pytest.raises(ValueError):
        paired_difference([0.1, 0.2, 0.3], [0.1, 0.2])
    with pytest.raises(ValueError):
        hierarchical_bootstrap({0: {"a": 1.0}}, {0: {"b": 1.0}})
    with pytest.raises(ValueError):
        binomial_interval(5, 3)
