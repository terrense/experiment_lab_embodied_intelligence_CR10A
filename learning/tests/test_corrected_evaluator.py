"""Tests for the corrected evaluation path (SIMULATION ONLY).

Covers the two new modules of the 2026-11-02 audit response:
  * evaluation/episode_manifest.py  - the frozen episode protocol
  * evaluation/independent_collision.py - a SECOND checker with its own geometry and distance
    code, including the robot-against-itself check the environment does not do
"""

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "learning"))
sys.path.insert(0, str(ROOT / "evaluation"))
pytest.importorskip("mujoco")
pytest.importorskip("xacro")

from episode_manifest import SPLITS, build_manifest, episodes_for, manifest_hash  # noqa: E402
from ex_learning.envs.cr10a_reach_wall import ReachWallEnv, WallConfig, load_scene  # noqa: E402


# ----------------------------------------------------------------------- the manifest
def test_split_seed_ranges_do_not_overlap_except_tune_inside_train():
    # "tune" is deliberately a subset of "train" (the hyper-parameter seeds 0-19 are training
    # seeds). Every other pair must be disjoint.
    assert SPLITS["tune"][0] >= SPLITS["train"][0] and SPLITS["tune"][1] <= SPLITS["train"][1]
    names = [n for n in SPLITS if n != "tune"]
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            a0, a1 = SPLITS[a]
            b0, b1 = SPLITS[b]
            assert a1 <= b0 or b1 <= a0, f"{a} and {b} overlap"


def test_the_historical_seeds_are_the_dev_split_and_the_test_split_is_new():
    # the published M5 numbers were computed on seeds 1000-1049 -> those are dev data now
    assert SPLITS["dev"] == (1000, 1050)
    assert SPLITS["test"][0] >= 20000, "the confirmatory range must not overlap the historical one"
    dev = episodes_for("dev", "wall_ood")
    assert len(dev) == 50 and dev[0]["seed"] == 1000 and dev[-1]["seed"] == 1049
    test = episodes_for("test", "wall_ood")
    assert len(test) == 200 and test[0]["seed"] == 20000


def test_manifest_hash_is_stable_and_version_sensitive():
    m = build_manifest(("wall_ood",))
    assert manifest_hash(m) == manifest_hash(build_manifest(("wall_ood",)))
    assert manifest_hash(m) != manifest_hash(build_manifest(("wall_ood",), version=2))
    assert manifest_hash(m) != manifest_hash(build_manifest(("wall_rand",)))


# --------------------------------------------------------------- the independent checker
@pytest.fixture(scope="module")
def checker():
    from independent_collision import IndependentCollisionChecker
    return IndependentCollisionChecker()


@pytest.fixture(scope="module")
def boxes():
    return load_scene("wall")[0]


def test_independent_checker_builds_a_geometry_for_every_link(checker):
    assert set(checker.clouds) == {"base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"}
    assert all(len(v) > 100 for v in checker.clouds.values())


def test_independent_checker_sees_a_box_placed_on_the_arm(checker, boxes):
    from ex_learning.envs.cr10a_reach_wall import ReachWallEnv
    env = ReachWallEnv(WallConfig(max_steps=4))
    env.reset(seed=0)
    checker.set_q(env.q())
    tcp = env.tcp()
    moved = []
    for b in boxes:
        b = dict(b)
        if b["name"] == "wall":
            b["center"] = list(tcp)
        moved.append(b)
    d, pair = checker.box_clearance(moved)
    assert d < 0.0, "a box on the TCP must be detected as a collision"
    assert pair is not None


def test_independent_checker_reports_no_self_collision_at_the_home_pose(checker):
    d, pair = checker.self_clearance()
    assert d > 0.0, f"the arm touches itself at the home pose: {d} ({pair})"


def test_environment_collisions_are_confirmed_by_the_independent_checker(checker, boxes):
    # the environment's capsule CONTAINS the mesh, so its clearance is a lower bound: every
    # configuration the environment calls a collision must also be a collision for the mesh
    env = ReachWallEnv(WallConfig(max_steps=4))
    rng = np.random.default_rng(0)
    checked = 0
    w = env.box_names.index("wall")
    for _ in range(12):
        env.reset(seed=int(rng.integers(0, 10_000)))
        q = env.q() + rng.uniform(-0.25, 0.25, 6)
        env._set_q(q)
        # put the wall onto the arm so the configuration really is a collision
        tcp = env.tcp()
        old_lo, old_hi = env.box_lo[w].copy(), env.box_hi[w].copy()
        env.box_lo[w], env.box_hi[w] = tcp - 0.05, tcp + 0.05
        try:
            env_clear = env.clearance()
        finally:
            env.box_lo[w], env.box_hi[w] = old_lo, old_hi
        moved = [dict(b, center=list(tcp)) if b["name"] == "wall" else b for b in boxes]
        checker.set_q(q)
        ind_clear, _ = checker.box_clearance(moved)
        assert env_clear < 0.0, "the fixture must be a collision for the environment too"
        assert ind_clear < 0.0, f"the environment reports a collision ({env_clear:.4f}) but the " \
                                f"mesh checker does not ({ind_clear:.4f})"
        checked += 1
    assert checked == 12


def test_mesh_clearance_is_never_below_the_capsule_clearance(checker, boxes):
    # the enclosing capsule contains the mesh, so the mesh distance is the larger one (up to the
    # sampling slack of both models)
    env = ReachWallEnv(WallConfig(max_steps=4))
    rng = np.random.default_rng(1)
    for _ in range(12):
        env.reset(seed=int(rng.integers(0, 10_000)))
        q = env.q() + rng.uniform(-0.25, 0.25, 6)
        env._set_q(q)
        env_clear = env.clearance()
        checker.set_q(q)
        ind_clear, _ = checker.box_clearance(boxes)
        assert ind_clear >= env_clear - 0.02, f"mesh {ind_clear:.4f} below capsule {env_clear:.4f}"


def test_corrected_environment_clearance_is_never_larger_than_the_legacy_one():
    legacy = ReachWallEnv(WallConfig(max_steps=4))
    strict = ReachWallEnv(WallConfig(max_steps=4, conservative_geometry=True))
    for seed in range(3):
        legacy.reset(seed=seed)
        strict.reset(seed=seed)
        for _ in range(5):
            q = legacy.q() + np.random.default_rng(seed).uniform(-0.2, 0.2, 6)
            legacy._set_q(q)
            strict._set_q(q)
            assert np.all(strict.pair_distances()[0] <= legacy.pair_distances()[0] + 1e-12)


# ------------------------------------------------------------------ the result manifest
def test_result_manifest_is_complete(tmp_path):
    # P1-d: an archived number must be reproducible from its own file
    from ex_learning.envs import make_env
    from ex_learning.run_safe_eval import build_manifest
    model = tmp_path / "ppo_wall_rand_s0"
    model.mkdir()
    (model / "ppo.zip").write_bytes(b"not a real checkpoint")
    env = make_env("wall_ood")
    m = build_manifest(model, "wall_ood", "cbf",
                       {"margin": 0.05, "alpha": 5.0, "d_max": 0.30}, env, 50, 1000, ROOT)
    for key in ("model", "task", "filter", "filter_params", "env", "episodes", "seed_start",
                "seed_end_exclusive", "checkpoint_sha256_16", "git_commit", "episode_manifest_hash"):
        assert key in m, f"the manifest is missing {key}"
    assert m["env"]["dynamics_preset"] == "placeholder"
    assert m["env"]["collision_evaluator"] == "legacy"
    assert m["checkpoint_sha256_16"] != "MISSING"
    assert m["git_commit"] != "UNKNOWN"
    assert m["episode_manifest_hash"] != ""


def test_the_corrected_evaluator_is_labelled_in_the_manifest(tmp_path):
    from ex_learning.envs import make_env
    from ex_learning.run_safe_eval import build_manifest
    model = tmp_path / "m"
    model.mkdir()
    (model / "ppo.zip").write_bytes(b"x")
    env = make_env("wall_ood", conservative_geometry=True, substep_collision_check=True)
    m = build_manifest(model, "wall_ood", "cbf", {}, env, 50, 1000, ROOT)
    assert m["env"]["collision_evaluator"] == "corrected"
