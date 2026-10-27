import math

import numpy as np
import pytest

from ex_robot_state.core.calibration import N_PER_JOINT, ChainModel, dedupe_poses, rms


@pytest.fixture(scope="module")
def chain(cr10_model):
    return ChainModel.from_chain(cr10_model.chain("flange", "base_link"))


def test_zero_params_match_urdf_fk(chain, cr10_model):
    rng = np.random.default_rng(0)
    q = rng.uniform(-2, 2, (50, 6))
    X = chain.fk_positions(q)
    for qi, xi in zip(q, X):
        T = cr10_model.fk({f"joint{i + 1}": v for i, v in enumerate(qi)}, "flange", "base_link")
        assert np.allclose(xi, T[:3, 3], atol=1e-12)


def test_recovers_synthetic_perturbation(chain):
    """Generate 'controller' data from a perturbed model (mm-level offsets, 0.1 deg zero
    errors), fit, and require the held-out residual to collapse to the noise level."""
    rng = np.random.default_rng(1)
    p_true = np.zeros(chain.n_params)
    for i in range(6):
        k = N_PER_JOINT * i
        p_true[k:k + 3] = rng.normal(0, 0.002, 3)          # 2 mm
        p_true[k + 3:k + 6] = rng.normal(0, 0.001, 3)      # ~0.06 deg
        p_true[k + 6] = rng.normal(0, math.radians(0.1))   # zero offsets
    q_train = rng.uniform(-2.5, 2.5, (600, 6))
    q_test = rng.uniform(-2.5, 2.5, (300, 6))
    noise = 0.00005
    x_train = chain.fk_positions(q_train, p_true) + rng.normal(0, noise, (600, 3))
    x_test = chain.fk_positions(q_test, p_true) + rng.normal(0, noise, (300, 3))
    before = rms(chain.fk_positions(q_test) - x_test)
    p, hist = chain.fit(q_train, x_train, reg=1e-9, iters=15)
    after = rms(chain.fk_positions(q_test, p) - x_test)
    assert before > 0.002
    assert after < 3 * noise * math.sqrt(3), (before, after, hist)


def test_identifiability_is_rank_deficient(chain):
    rng = np.random.default_rng(2)
    rank, s = chain.identifiability(rng.uniform(-2.5, 2.5, (300, 6)))
    assert rank < chain.n_params  # position-only data can't identify all 45 parameters
    assert rank >= 24


def test_calibrated_origins_reproduce_fit(chain):
    """Folding the deltas + zero offsets into new origins must give the same FK."""
    rng = np.random.default_rng(3)
    p = rng.normal(0, 0.002, chain.n_params)
    origins = chain.calibrated_origins(p)
    from ex_robot_state.core.se3 import make_transform, rpy_to_matrix
    folded = ChainModel([make_transform(rpy_to_matrix(*rpy), xyz) for xyz, rpy in origins], chain.axes,
                        chain.post.copy())
    folded.post[:3, 3] = chain.post[:3, 3] + chain.post[:3, :3] @ p[-3:]
    q = rng.uniform(-2, 2, (40, 6))
    assert np.allclose(folded.fk_positions(q), chain.fk_positions(q, p), atol=1e-9)


def test_dedupe_averages_cells():
    q = np.array([[0.0, 0.0], [0.01, 0.0], [1.0, 1.0]])
    x = np.array([[1.0], [3.0], [5.0]])
    qa, xa = dedupe_poses(q, x, 0.1)
    assert len(qa) == 2 and sorted(xa.ravel().tolist()) == [2.0, 5.0]


def test_calibrated_description_expands_and_stays_close(cr10_model):
    """The generated calibrated model keeps names/limits and moves the flange by only a few mm."""
    xacro = pytest.importorskip("xacro")
    import os
    from pathlib import Path
    from ex_robot_state.core.urdf_model import UrdfModel
    pkg = Path(__file__).resolve().parents[2] / "ex_description"
    text = (pkg / "urdf" / "ex_cr10.urdf.xacro").read_text().replace("$(find ex_description)", str(pkg))
    tmp = Path(os.environ.get("PYTEST_TMP", "/tmp")) / "ex_cr10_cal_test.xacro"
    tmp.write_text(text)
    cal = UrdfModel.from_string(xacro.process_file(str(tmp), mappings={"calibrated": "true"}).toxml())
    nom_joints = cr10_model.actuated_joints("tcp")
    cal_joints = cal.actuated_joints("tcp")
    assert [j.name for j in cal_joints] == [j.name for j in nom_joints]
    assert [(j.lower, j.upper) for j in cal_joints] == [(j.lower, j.upper) for j in nom_joints]
    rng = np.random.default_rng(5)
    d = []
    for q in rng.uniform(-2.5, 2.5, (200, 6)):
        qq = {f"joint{i + 1}": v for i, v in enumerate(q)}
        d.append(np.linalg.norm(cal.fk(qq, "flange")[:3, 3] - cr10_model.fk(qq, "flange")[:3, 3]))
    assert 0.0005 < np.mean(d) < 0.01  # mm-level correction, never more than ~1 cm
