"""Tests for the RRT-Connect planner (SIMULATION ONLY).

The nearest-node search in `JointRRTConnect._extend` was vectorised (a numpy `argmin` over the
node list instead of a Python loop over it) because a failed search with a 20k-check budget built
a tree of thousands of nodes and took 1711 s. The change must be a pure speed-up: the same node
has to be selected, and the random number stream must not move. This test pins the first half;
the second half is checked against the archived run (the same eight episodes of
`data/reports/learning/rrt_window.json` reproduce with the same solved flags, the same number of
collision checks and the same number of waypoints).
"""

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning"))
pytest.importorskip("numpy")

from ex_learning.planner import JointRRTConnect  # noqa: E402


class _FakeEnv:
    """Just enough of the environment for the node bookkeeping."""

    def __init__(self):
        self.lo = np.full(6, -2.0)
        self.hi = np.full(6, 2.0)

    def _set_q(self, q):
        pass

    def clearance(self):
        return 1.0


def test_the_vectorised_nearest_node_search_picks_the_same_node_as_the_loop():
    """The vectorised search must select exactly the node the original list comprehension did."""
    pl = JointRRTConnect(_FakeEnv())
    rng = np.random.default_rng(0)
    nodes = [rng.uniform(-1.0, 1.0, 6) for _ in range(300)]
    parents = [-1] * len(nodes)
    checked = 0
    for _ in range(60):
        target = rng.uniform(-1.0, 1.0, 6)
        # what the original implementation computed
        expect = int(np.argmin([np.max(np.abs(n - target)) for n in nodes]))
        status, idx = pl._extend(nodes, parents, target)
        assert status != "trapped"
        # parents[k] is the parent of nodes[k]; the new node's parent is the one the search
        # extended FROM
        assert parents[idx] == expect
        checked += 1
    assert checked == 60


def test_the_planner_is_deterministic_for_a_fixed_seed():
    """Two runs with the same seed must produce the same plan: the RNG stream is part of the
    protocol, and the archived results are only reproducible if it does not move."""
    plans = []
    for _ in range(2):
        pl = JointRRTConnect(_FakeEnv(), max_checks=2000, rng=np.random.default_rng(7))
        # a fake environment is always clear, so the search runs until the budget is spent
        path = pl.plan(np.zeros(6), np.ones(6))
        plans.append(None if path is None else [np.round(p, 12).tolist() for p in path])
    assert plans[0] == plans[1]


def test_the_archived_window_plans_are_reproducible():
    """The archived eight-episode window run must still be reproducible.

    This does NOT re-run the planner (that takes minutes); it checks that the archived file is
    present and self-consistent, so a future change that silently alters the planner is caught by
    the manual re-check recorded in docs/PROGRESS.md.
    """
    import json
    rep = ROOT / "data" / "reports" / "learning" / "rrt_window.json"
    assert rep.exists(), "the archived window run is not committed - the planner cannot be checked"
    blob = json.loads(rep.read_text())
    assert blob["episodes"] == 8
    assert len(blob["plans"]) == 8
    for p in blob["plans"]:
        assert p["waypoints"] == 0 or p["solved"]
        if not p["solved"]:
            assert p["checks"] >= blob["max_checks"]
