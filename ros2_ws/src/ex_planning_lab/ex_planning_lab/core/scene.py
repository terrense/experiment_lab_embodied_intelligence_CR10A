"""Benchmark scenes: axis-aligned boxes in the world frame + query definitions.

YAML schema (see scenes/*.yaml):
  name: wall
  description: ...
  boxes:      [{name, center: [x,y,z], size: [sx,sy,sz]}]   # metres, world frame
  queries:    [{name, start_deg: [6], goal_deg: [6]}]       # explicit joint queries
  sample_queries:                                            # optional, reproducible
    n: 4, seed: 0,
    start_tcp_box: {lo: [..], hi: [..]}, goal_tcp_box: {lo: [..], hi: [..]}
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml


@dataclass(frozen=True)
class Box:
    name: str
    center: np.ndarray
    size: np.ndarray

    @property
    def lo(self) -> np.ndarray:
        return self.center - self.size / 2

    @property
    def hi(self) -> np.ndarray:
        return self.center + self.size / 2

    def contains(self, p: np.ndarray) -> bool:
        return bool(np.all(p >= self.lo) and np.all(p <= self.hi))


@dataclass(frozen=True)
class Query:
    name: str
    start: List[float]  # rad
    goal: List[float]  # rad


@dataclass
class Scene:
    name: str
    description: str
    boxes: List[Box]
    queries: List[Query]
    sample_spec: Optional[Dict] = field(default=None)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Scene":
        d = yaml.safe_load(Path(path).read_text())
        boxes = [Box(b["name"], np.array(b["center"], float), np.array(b["size"], float)) for b in d.get("boxes", [])]
        for b in boxes:
            if np.any(b.size <= 0):
                raise ValueError(f"box {b.name}: size must be positive")
        qs = [Query(q["name"], [math.radians(v) for v in q["start_deg"]], [math.radians(v) for v in q["goal_deg"]])
              for q in d.get("queries", [])]
        for q in qs:
            if len(q.start) != 6 or len(q.goal) != 6:
                raise ValueError(f"query {q.name}: need 6 joints")
        return cls(d["name"], d.get("description", ""), boxes, qs, d.get("sample_queries"))


def sample_in_tcp_region(rng: np.random.Generator, fk_tcp, lower: np.ndarray, upper: np.ndarray,
                         region: Box, max_tries: int = 20000) -> Optional[np.ndarray]:
    """Rejection-sample a joint vector whose TCP lies inside `region` (validity
    against the planning scene is checked by the caller with MoveIt)."""
    for _ in range(max_tries):
        q = rng.uniform(lower, upper)
        if region.contains(fk_tcp(q)):
            return q
    return None
