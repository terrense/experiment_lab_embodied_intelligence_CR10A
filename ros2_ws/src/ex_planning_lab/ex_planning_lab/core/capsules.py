"""Conservative capsule model of the arm, fitted from the URDF meshes.

For each link two candidate capsules are fitted - one along the kinematic
"bone" (link origin -> child joint origin), one along the mesh's principal axis -
each with radius = largest vertex-to-segment distance, so both CONTAIN the mesh;
the one with smaller volume is kept. Distances computed with capsules are therefore lower bounds of the true
mesh distance (clearance is reported conservatively).

Used only for the clearance metric of the planning lab; collision checking in
MoveIt itself still uses the meshes via FCL.
"""

from __future__ import annotations

import math
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np


def read_stl_vertices(path: str | Path) -> np.ndarray:
    """Unique vertices (N,3) of a binary or ASCII STL."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = struct.unpack_from("<I", data, 80)[0]
        if 84 + n * 50 == len(data):  # binary
            rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                                count=n, offset=84)
            return np.unique(rec["v"].reshape(-1, 3).astype(float), axis=0)
    pts = [list(map(float, ln.split()[1:4])) for ln in data.decode("ascii", "ignore").splitlines()
           if ln.strip().startswith("vertex")]
    if not pts:
        raise ValueError(f"cannot parse STL {path}")
    return np.unique(np.array(pts), axis=0)


def point_segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Distance of points p (N,3) to segment ab."""
    ab = b - a
    denom = float(ab @ ab)
    t = np.zeros(len(p)) if denom < 1e-12 else np.clip((p - a) @ ab / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.linalg.norm(p - proj, axis=1)


def point_aabb_distance(p: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    """Euclidean distance of points p (N,3) to an axis-aligned box [lo, hi] (0 inside)."""
    d = np.maximum(np.maximum(lo - p, 0.0), p - hi)
    return np.linalg.norm(d, axis=1)


@dataclass(frozen=True)
class Capsule:
    link: str
    a: np.ndarray  # segment start, link frame
    b: np.ndarray  # segment end, link frame
    radius: float


def fit_capsules(urdf_xml: str, resolve: Callable[[str], Optional[Path]], links: List[str],
                 child_origin: Dict[str, np.ndarray]) -> List[Capsule]:
    """Fit one capsule per link from its <collision> (else <visual>) mesh.

    child_origin[link] = origin of the next joint in this link's frame (end of bone).
    Mesh origins/scales in the URDF are assumed identity (true for the Dobot URDFs; asserted).
    """
    root = ET.fromstring(urdf_xml)
    out = []
    for link in links:
        el = root.find(f"link[@name='{link}']")
        mesh = None
        if el is not None:
            for tag in ("collision", "visual"):
                g = el.find(f"{tag}/geometry/mesh")
                if g is not None:
                    o = el.find(f"{tag}/origin")
                    if o is not None and any(float(v) != 0 for v in (o.get("xyz", "0 0 0") + " " + o.get("rpy", "0 0 0")).split()):
                        raise ValueError(f"{link}: non-identity mesh origin not supported")
                    mesh = g.get("filename")
                    break
        if mesh is None:
            continue
        path = resolve(mesh)
        if path is None:
            raise FileNotFoundError(mesh)
        v = read_stl_vertices(path)
        bone = (np.zeros(3), np.asarray(child_origin.get(link, np.zeros(3)), dtype=float))
        out.append(Capsule(link, *tightest_enclosing_capsule(v, bone)))
    return out


def capsule_volume(a: np.ndarray, b: np.ndarray, r: float) -> float:
    return math.pi * r * r * float(np.linalg.norm(b - a)) + 4.0 / 3.0 * math.pi * r ** 3


def tightest_enclosing_capsule(v: np.ndarray, bone):
    """Smaller-volume of two capsules that both contain all vertices:
    (1) along the kinematic bone, (2) along the mesh's principal axis (PCA).
    Meshes whose material is offset from the joint line (e.g. the CR10 upper
    arm) get a much tighter fit from (2)."""
    a1, b1 = bone
    r1 = float(point_segment_distance(v, a1, b1).max())
    c = v.mean(axis=0)
    _, _, vt = np.linalg.svd(v - c, full_matrices=False)
    ax = vt[0]
    s = (v - c) @ ax
    a2, b2 = c + s.min() * ax, c + s.max() * ax
    r2 = float(point_segment_distance(v, a2, b2).max())
    if capsule_volume(a2, b2, r2) < capsule_volume(a1, b1, r1):
        return a2, b2, r2
    return a1, b1, r1


def capsule_box_distance(T_world_link: np.ndarray, cap: Capsule, lo: np.ndarray, hi: np.ndarray,
                         samples: int = 16) -> float:
    """Approximate signed clearance capsule <-> AABB (negative = penetration estimate)."""
    a = T_world_link[:3, :3] @ cap.a + T_world_link[:3, 3]
    b = T_world_link[:3, :3] @ cap.b + T_world_link[:3, 3]
    t = np.linspace(0.0, 1.0, samples)[:, None]
    pts = a + t * (b - a)
    return float(point_aabb_distance(pts, lo, hi).min()) - cap.radius
