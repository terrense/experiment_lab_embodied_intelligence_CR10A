"""Frozen, versioned episode manifest for the M5 learning experiments (SIMULATION ONLY).

Why this exists
---------------
The evaluation seeds 1000-1049 were used to produce the published results AND were inspected
while writing them. Once a result has been looked at, the set it was computed on can no longer
serve as a confirmatory test set: any later method that was chosen while looking at those
episodes is, in part, fitted to them.

So the seeds are partitioned:

    train  0 ..   999   policy training
    tune   0 ..    19   hyper-parameters, margins, gains (subset of train, as before)
    dev  1000 .. 1049   the HISTORICAL set - the archived results live here. Use it for
                        mechanism analysis and for the legacy-vs-corrected comparison only.
    test 20000 .. 20199 a NEW, untouched range. Any new claim is reported here and nowhere else.

A manifest is a JSON file with a content hash. Every result JSON records the manifest version and
the hash, so a number can always be traced back to the exact episode set that produced it, and two
results can always be checked for having been computed on the same episodes.

The environment's `reset(seed=...)` fully determines the episode (start pose, goal, and - for the
random-wall tasks - the wall itself), so the seed IS the episode identity; the manifest fixes the
seed ranges and records them, it does not need to store the scenes themselves.

Version 2 - why it exists
--------------------------
Version 1 put the **tuning** split at seeds 0-19, which lie INSIDE the training range 0-999. The
operating points (the filter margins) were therefore chosen on scenarios the policies have already
been trained on, and on those scenarios the two filters are indistinguishable (32.0 % against
31.3 %), while on the untouched test range they separate. A tuning split that overlaps the
training split cannot resolve the differences a method produces, and it is a protocol flaw worth
fixing rather than a result worth explaining away.

Version 2 therefore moves the tuning split OUT of the training range and adds a second untouched
range for replication:

    train  0 ..   999   policy training
    tune 5000 .. 5099   DISJOINT from train (this is the fix)
    dev  1000 .. 1049   the historical set
    test 20000 .. 20199 the confirmatory range
    replication 30000 .. 30199  a second untouched range, for replication of the confirmatory run

**Version 1 is frozen and unchanged.** Its hash (`a995f097b28148c2`) is quoted in the committed
result reports, so the splits it used are kept verbatim in `SPLITS_BY_VERSION` and its hash still
reproduces. The tuning split of version 1 is retained as `tune_v1_contaminated` in the notes of
version 2, because the write-up has to be able to say which operating point was chosen where.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Dict, List

ROOT = Path(__file__).resolve().parents[1]

MANIFEST_VERSION = 2

#: split -> (first seed, last seed exclusive).  The boundaries are part of the protocol: changing
#: them is a protocol change and needs a new manifest version. Version 1 is kept verbatim so that
#: the hash quoted in the committed reports still reproduces.
SPLITS_BY_VERSION: Dict[int, Dict[str, tuple]] = {
    1: {
        "train": (0, 1000),
        "tune": (0, 20),            # a SUBSET of train - the flaw that version 2 fixes
        "dev": (1000, 1050),
        "test": (20000, 20200),
    },
    2: {
        "train": (0, 1000),
        "tune": (5000, 5100),       # DISJOINT from train
        "dev": (1000, 1050),
        "test": (20000, 20200),
        "replication": (30000, 30200),
    },
}

#: the splits of the CURRENT manifest version, for the scripts that import them
SPLITS: Dict[str, tuple] = SPLITS_BY_VERSION[MANIFEST_VERSION]

#: the scene families the final study is planned over
TASK_FAMILIES = ("wall_obs", "wall_rand", "wall_ood", "window", "shelf")


def splits_for(version: int = MANIFEST_VERSION) -> Dict[str, tuple]:
    """The splits of one manifest version."""
    if version not in SPLITS_BY_VERSION:
        raise ValueError(f"unknown manifest version {version!r}; known: {sorted(SPLITS_BY_VERSION)}")
    return SPLITS_BY_VERSION[version]


def episodes_for(split: str, task: str, version: int = MANIFEST_VERSION) -> List[Dict]:
    """The episode list of one (split, task) pair."""
    splits = splits_for(version)
    if split not in splits:
        raise ValueError(f"unknown split {split!r}; known: {sorted(splits)}")
    start, end = splits[split]
    return [{"split": split, "task": task, "seed": s, "id": f"{split}/{task}/{s}"} for s in range(start, end)]


def build_manifest(tasks: tuple = TASK_FAMILIES, version: int = MANIFEST_VERSION) -> Dict:
    """The full manifest: the split boundaries and the episode list of every (split, task)."""
    splits = splits_for(version)
    notes = [
        "dev (1000-1049) is the historical set: the published M5 numbers were computed on it",
        "and inspected. It may be used for mechanism analysis and for the legacy-vs-corrected",
        "comparison, never as a confirmatory test set.",
        "test (20000-20199) is the only range a new claim may be reported on.",
    ]
    if version >= 2:
        notes += [
            "v2 moves the tuning split OUT of the training range: v1 used seeds 0-19, which are",
            "inside train (0-999), so the operating points were chosen on scenarios the policies",
            "have already seen. v1's tuning split is retained as tune_v1_contaminated below.",
            "v2 adds replication (30000-30199), a second untouched range, for replicating the",
            "confirmatory run without reusing it.",
        ]
    out = {
        "version": version,
        "splits": {k: {"seed_start": v[0], "seed_end_exclusive": v[1], "n_episodes": v[1] - v[0]}
                   for k, v in splits.items()},
        "tasks": list(tasks),
        "episodes": [e for task in tasks for split in splits for e in episodes_for(split, task, version)],
        "notes": notes,
    }
    if version >= 2:
        out["tune_v1_contaminated"] = {"seed_start": 0, "seed_end_exclusive": 20,
                                        "why": "the operating points of the confirmatory run were "
                                               "selected here; the split lies inside the training "
                                               "range, which is the protocol flaw v2 fixes"}
    return out


def manifest_hash(manifest: Dict) -> str:
    """Content hash of the manifest (stable across runs: the episodes are sorted by construction)."""
    payload = json.dumps({k: v for k, v in manifest.items() if k != "notes"}, sort_keys=True,
                         separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


def load_manifest(path: Path) -> Dict:
    return json.loads(Path(path).read_text())


def write_manifest(path: Path, tasks: tuple = TASK_FAMILIES, version: int = MANIFEST_VERSION) -> Dict:
    m = build_manifest(tasks, version)
    m["hash"] = manifest_hash(m)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(m, indent=1))
    return m


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--tasks", nargs="*", default=list(TASK_FAMILIES))
    ap.add_argument("--version", type=int, default=MANIFEST_VERSION)
    a = ap.parse_args(argv)
    tasks = tuple(a.tasks)
    versions = [a.version] if a.version else sorted(SPLITS_BY_VERSION)
    for v in versions:
        out = a.out or (ROOT / "data" / "reports" / "learning" / f"manifest_v{v}.json")
        m = write_manifest(out, tasks, v)
        print(f"manifest v{m['version']} hash {m['hash']}  -> {out}")
        for name, s in m["splits"].items():
            print(f"  {name:12s} seeds [{s['seed_start']}, {s['seed_end_exclusive']})  n={s['n_episodes']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
