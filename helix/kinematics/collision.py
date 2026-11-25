"""Collision envelope: capsules per link, read from ``spec/arm.yaml: collision`` (ADR 0003).

The capsules are the M1 contract for the mechanical design: each link's CAD (M3) must fit inside its
capsules, so anything proven collision-free here stays collision-free once the real parts exist. Checks:
capsule-capsule self-collision for every pair not listed in ``collision.skip`` (pairs that touch at a
joint by construction) and not on the same link, and every moving capsule against the table plane.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from helix.spec import Spec, fk


@dataclass(frozen=True)
class Capsule:
    name: str
    link: str            # "base" or a joint name (the capsule moves with that joint's frame)
    a: np.ndarray
    b: np.ndarray
    r: float


@dataclass(frozen=True)
class Envelope:
    capsules: tuple[Capsule, ...]
    pairs: tuple[tuple[int, int], ...]   # index pairs that are checked
    margin: float
    table_z: float


def segment_distance(p1, q1, p2, q2):
    """Closest distance between segments p1q1 and p2q2 (Ericson, Real-Time Collision Detection 5.1.9)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    eps = 1e-15
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s, t = 0.0, np.clip(f / e, 0.0, 1.0)
    else:
        c = d1 @ r
        if e <= eps:
            t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
        else:
            b = d1 @ d2
            den = a * e - b * b
            s = np.clip((b * f - c * e) / den, 0.0, 1.0) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t, s = 1.0, np.clip((b - c) / a, 0.0, 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def from_spec(spec: Spec) -> Envelope:
    c = spec.raw["collision"]
    links = {"base"} | {j.name for j in spec.joints}
    caps = []
    for k in c["capsules"]:
        if k["link"] not in links:
            raise ValueError(f"capsule {k['name']}: unknown link {k['link']}")
        caps.append(Capsule(k["name"], k["link"], np.asarray(k["a"], float), np.asarray(k["b"], float), float(k["r"])))
    names = [k.name for k in caps]
    if len(set(names)) != len(names):
        raise ValueError("duplicate capsule names")
    skip = set()
    for x, y in c["skip"]:
        if x not in names or y not in names:
            raise ValueError(f"skip pair {x}, {y}: unknown capsule")
        skip.add(frozenset((x, y)))
    pairs = tuple((i, j) for i in range(len(caps)) for j in range(i + 1, len(caps))
                  if caps[i].link != caps[j].link and frozenset((names[i], names[j])) not in skip)
    return Envelope(tuple(caps), pairs, float(c["margin_m"]), float(c["table_z"]))


def world_segments(env: Envelope, spec: Spec, q) -> list[tuple[np.ndarray, np.ndarray]]:
    T = fk(spec, q)
    T["base"] = np.eye(4)
    out = []
    for k in env.capsules:
        M = T[k.link]
        out.append((M[:3, :3] @ k.a + M[:3, 3], M[:3, :3] @ k.b + M[:3, 3]))
    return out


@dataclass(frozen=True)
class Report:
    self_clearance: float     # smallest (distance - r1 - r2) over the checked capsule pairs
    self_worst: str           # the pair that gave it
    table_clearance: float    # lowest moving capsule surface above the table
    table_worst: str
    margin: float

    @property
    def self_collides(self) -> bool:
        return self.self_clearance < self.margin

    @property
    def table_collides(self) -> bool:
        return self.table_clearance < self.margin

    @property
    def collides(self) -> bool:
        return self.self_collides or self.table_collides


def check(env: Envelope, spec: Spec, q) -> Report:
    seg = world_segments(env, spec, q)
    best, who = np.inf, ""
    for i, j in env.pairs:
        d = segment_distance(*seg[i], *seg[j]) - env.capsules[i].r - env.capsules[j].r
        if d < best:
            best, who = d, f"{env.capsules[i].name}/{env.capsules[j].name}"
    tbest, twho = np.inf, ""
    for k, (a, b) in zip(env.capsules, seg):
        if k.link == "base":
            continue
        d = min(a[2], b[2]) - k.r - env.table_z
        if d < tbest:
            tbest, twho = d, k.name
    return Report(float(best), who, float(tbest), twho, env.margin)
