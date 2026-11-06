"""The documents must say what the data says.

`evaluation/verify_claims.py` re-derives every headline number from the committed JSON reports
and compares it with what the manuscript, docs/RESULTS.md and the README state. This test runs
it, so a number that drifts away from its source fails the test suite instead of surviving
until a reader notices.

The reports are committed (nothing under data/ is blocked), so this test runs in a fresh clone.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "evaluation"))

pytest.importorskip("numpy")

from verify_claims import main as verify  # noqa: E402


def test_every_claim_in_the_documents_matches_its_source(capsys):
    rc = verify()
    out = capsys.readouterr().out
    if rc != 0:
        pytest.fail("the documents disagree with the data:\n" + out)
    assert "claims match their source" in out


def test_the_confirmatory_report_is_committed():
    """The claim checker is only meaningful if the report it reads is in the repository."""
    rep = ROOT / "data" / "reports" / "learning" / "final_benchmark_full.json"
    assert rep.exists(), "the confirmatory report is not committed - the analysis cannot be audited"
    import json
    blob = json.loads(rep.read_text())
    assert blob["split"] == "test"
    assert blob["episode_manifest_hash"] == "a995f097b28148c2"
