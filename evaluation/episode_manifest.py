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
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Dict, List

ROOT = Path(__file__).resolve().parents[1]

MANIFEST_VERSION = 1

#: split -> (first seed, last seed inclusive).  The boundaries are part of the protocol: changing
#: them is a protocol change and needs a new manifest version.
SPLITS: Dict[str, tuple] = {
    "train": (0, 1000),        # exclusive end
    "tune": (0, 20),           # exclusive end; a subset of train, as before
    "dev": (1000, 1050),       # the historical set: the published numbers live here
    "test": (20000, 20200),    # untouched
}

#: the scene families the final study is planned over
TASK_FAMILIES = ("wall_obs", "wall_rand", "wall_ood", "window", "shelf")


def episodes_for(split: str, task: str) -> List[Dict]:
    """The episode list of one (split, task) pair."""
    if split not in SPLITS:
        raise ValueError(f"unknown split {split!r}; known: {sorted(SPLITS)}")
    start, end = SPLITS[split]
    return [{"split": split, "task": task, "seed": s, "id": f"{split}/{task}/{s}"} for s in range(start, end)]


def build_manifest(tasks: tuple = TASK_FAMILIES, version: int = MANIFEST_VERSION) -> Dict:
    """The full manifest: the split boundaries and the episode list of every (split, task)."""
    return {
        "version": version,
        "splits": {k: {"seed_start": v[0], "seed_end_exclusive": v[1], "n_episodes": v[1] - v[0]}
                   for k, v in SPLITS.items()},
        "tasks": list(tasks),
        "episodes": [e for task in tasks for split in SPLITS for e in episodes_for(split, task)],
        "notes": [
            "dev (1000-1049) is the historical set: the published M5 numbers were computed on it",
            "and inspected. It may be used for mechanism analysis and for the legacy-vs-corrected",
            "comparison, never as a confirmatory test set.",
            "test (20000-20199) is the only range a new claim may be reported on.",
        ],
    }


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
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "reports" / "learning" / "manifest_v1.json")
    ap.add_argument("--tasks", nargs="*", default=list(TASK_FAMILIES))
    ap.add_argument("--version", type=int, default=MANIFEST_VERSION)
    a = ap.parse_args(argv)
    m = write_manifest(a.out, tuple(a.tasks), a.version)
    print(f"manifest v{m['version']} hash {m['hash']}  -> {a.out}")
    for name, s in m["splits"].items():
        print(f"  {name:6s} seeds [{s['seed_start']}, {s['seed_end_exclusive']})  n={s['n_episodes']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
