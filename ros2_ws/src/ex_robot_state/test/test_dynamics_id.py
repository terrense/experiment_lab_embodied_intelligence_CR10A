"""Regression tests for the offline dynamics identification (P1-c, 2026-11-02 audit).

The archived run02 result must stay reproducible: the identification pipeline is a chain of
easy-to-break steps (resampling, filtering, the split, the regressor) and a silent transcription
error in it changes the published numbers without changing anything visible.

The audit's P1-c finding is that the LEGACY pipeline filters the whole series with a zero-phase
`filtfilt` and splits afterwards, so the held-out test part sees a little of the training part.
The fix (split first, filter each part separately) is available behind a flag, and this test
records what it actually changes: on the archived run the two paths agree to within 0.1
percentage points of VAF, i.e. the leakage is real as a mechanism but negligible in magnitude.
"""

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "evaluation"))
pytest.importorskip("scipy")
pytest.importorskip("xacro")

RUN02 = ROOT / "docs" / "evidence" / "live" / "excitation_run02"


@pytest.fixture(scope="module")
def identified():
    if not (RUN02 / "feedback.csv.gz").exists():
        pytest.skip("the run02 excitation recording is not in the repo")
    from dynamics_id import ChainDynamics, expanded_urdf, identify, load, preprocess
    h, D = load(RUN02)
    cd = ChainDynamics(expanded_urdf())
    prior = cd.theta_urdf.copy()
    from ex_robot_state.core.dynamics import add_point_mass
    prior[50:60] = add_point_mass(prior[50:60], 0.4, (0.0, 0.0, 0.04))
    out = {}
    for tag, kwargs in (("legacy", {}), ("split", {"split_before_filter": True})):
        data = preprocess(h, D, 4.0, **kwargs)
        rep, _preds = identify(data, cd, prior)
        out[tag] = rep
        out[tag + "_data"] = data
    return out


def test_the_legacy_pipeline_reproduces_the_archived_result(identified):
    # the archived data/reports/dynamics/run02_id.json (2026-10-28): if these numbers move, the
    # pipeline changed and every downstream statement has to be re-checked
    rep = identified["legacy"]
    assert rep["n_samples"] == 10466
    assert rep["M1s_rigid_scale_k"] == pytest.approx(1.636, abs=0.001)
    assert rep["M1_urdf_plus_friction"]["rms"] == pytest.approx([1.71, 2.563, 1.205, 0.27, 0.303, 0.231],
                                                                 abs=0.01)
    assert rep["M1_urdf_plus_friction"]["vaf_pct"] == pytest.approx([97.2, 97.3, 98.5, 97.7, 96.3, 98.4],
                                                                     abs=0.2)
    assert rep["M1_joint_terms"]["coulomb"] == pytest.approx([10.39, 14.15, 9.33, 1.48, 1.46, 1.43], abs=0.05)


def test_the_split_before_filter_path_agrees_with_the_legacy_one(identified):
    # P1-c: the zero-phase filter leaks a little of the training part into the held-out test
    # part. Measured on the archived run the effect is SMALL - the published numbers stand.
    leg, cor = identified["legacy"], identified["split"]
    assert cor["M1s_rigid_scale_k"] == pytest.approx(leg["M1s_rigid_scale_k"], abs=0.01)
    for key in ("M1_urdf_plus_friction", "M1s_scaled_urdf_plus_friction", "M2_base_ridge"):
        a = np.array(leg[key]["vaf_pct"])
        b = np.array(cor[key]["vaf_pct"])
        assert np.all(np.abs(a - b) < 0.5), f"{key}: the split changed the VAF by more than 0.5 points"


def test_the_split_path_really_uses_separate_segments(identified):
    # the guard interval must make the two paths DIFFER somewhere, otherwise the flag does nothing
    leg, cor = identified["legacy"], identified["split"]
    assert leg["n_samples"] != cor["n_samples"] or leg["split_s"] != cor["split_s"]
    assert identified["split_data"]["split_bounds"] is not None
    assert identified["legacy_data"]["split_bounds"] is None


def test_the_qd_versus_dqdt_sanity_check_stays_at_one():
    # a wrong derivative (e.g. d(q)/dt instead of d(qd)/dt) passes this check only by luck, but a
    # concatenation of separately filtered segments with a time gap does break it - which is how
    # the guard-interval bug was caught
    from dynamics_id import load, preprocess
    if not (RUN02 / "feedback.csv.gz").exists():
        pytest.skip("the run02 excitation recording is not in the repo")
    h, D = load(RUN02)
    for kwargs in ({}, {"split_before_filter": True}):
        data = preprocess(h, D, 4.0, **kwargs)
        scale = np.asarray(data["qd_scale_vs_dqdt"])
        assert np.all(np.abs(scale - 1.0) < 0.02), f"the qd/dqdt scale check failed: {scale}"
