"""Modified (Craig) Denavit-Hartenberg parameters derived from the spec's joint axis lines.

DH is a derived view (ADR 0001): frames are built from the zero-pose axis lines (common normals, with the
intersecting / parallel / collinear special cases), each link transform is factored as
``Rx(alpha_{i-1}) Tx(a_{i-1}) Rz(theta_i) Tz(d_i)``, and the result is tested against ``helix.spec.fk``.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from helix.kinematics.poe import from_spec as poe_from_spec
from helix.spec import Spec

TOL = 1e-12


@dataclass(frozen=True)
class DHRow:
    alpha: float   # alpha_{i-1}
    a: float       # a_{i-1}
    theta0: float  # joint-angle offset: theta_i = q_i + theta0
    d: float       # d_i


@dataclass(frozen=True)
class DH:
    rows: tuple[DHRow, ...]
    base: np.ndarray   # spec base frame -> DH frame 0
    tool: np.ndarray   # DH frame n -> TCP


def _frame(o, x, z) -> np.ndarray:
    F = np.eye(4)
    F[:3, 0], F[:3, 2] = x, z
    F[:3, 1] = np.cross(z, x)
    F[:3, 3] = o
    return F


def _closest_on_line(p, z, q):
    return p + np.dot(q - p, z) * z


def link_transform(r: DHRow, q: float) -> np.ndarray:
    ca, sa = np.cos(r.alpha), np.sin(r.alpha)
    ct, st = np.cos(q + r.theta0), np.sin(q + r.theta0)
    return np.array([
        [ct, -st, 0.0, r.a],
        [st * ca, ct * ca, -sa, -sa * r.d],
        [st * sa, ct * sa, ca, ca * r.d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def zero_pose_frames(spec: Spec) -> list[np.ndarray]:
    """DH frames 1..n at the zero pose, in base coordinates."""
    m = poe_from_spec(spec)
    zs = [m.S[:3, i] / np.linalg.norm(m.S[:3, i]) for i in range(m.n)]
    ps = list(m.points)
    frames: list[np.ndarray] = []
    x_prev = None
    o_prev = ps[0]
    for i in range(m.n):
        z, p = zs[i], ps[i]
        if i == m.n - 1:
            o = _closest_on_line(p, z, o_prev)
            x = x_prev
        else:
            z1, p1 = zs[i + 1], ps[i + 1]
            n = np.cross(z, z1)
            if np.linalg.norm(n) > 1e-9:                   # skew or intersecting
                n /= np.linalg.norm(n)
                # closest points between the two lines
                A = np.array([[np.dot(z, z), -np.dot(z, z1)], [np.dot(z, z1), -np.dot(z1, z1)]])
                b = np.array([np.dot(p1 - p, z), np.dot(p1 - p, z1)])
                t, _ = np.linalg.solve(A, b)
                o = p + t * z
                c1 = _closest_on_line(p1, z1, o)
                x = c1 - o
                x = n if np.linalg.norm(x) < 1e-9 else x / np.linalg.norm(x)
            else:                                           # parallel or collinear
                o = _closest_on_line(p, z, o_prev)
                x = _closest_on_line(p1, z1, o) - o
                if np.linalg.norm(x) < 1e-9:                # collinear: keep previous x
                    x = x_prev if x_prev is not None else _any_perp(z)
                else:
                    x /= np.linalg.norm(x)
        if x is None:
            x = _any_perp(z)
        x = x - np.dot(x, z) * z
        x /= np.linalg.norm(x)
        frames.append(_frame(o, x, z))
        x_prev, o_prev = x, o
    return frames


def _any_perp(z):
    v = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 0, 1.0])
    v = v - np.dot(v, z) * z
    return v / np.linalg.norm(v)


def _factor(T: np.ndarray) -> DHRow:
    R, p = T[:3, :3], T[:3, 3]
    if abs(R[0, 2]) > 1e-9:
        raise ValueError("link transform is not of DH form (x_{i-1} not perpendicular to z_i)")
    alpha = np.arctan2(-R[1, 2], R[2, 2])
    theta = np.arctan2(-R[0, 1], R[0, 0])
    a = p[0]
    d = -p[1] / np.sin(alpha) if abs(np.sin(alpha)) > 0.5 else p[2] / np.cos(alpha)
    row = DHRow(float(alpha), float(a), float(theta), float(d))
    if not np.allclose(link_transform(row, 0.0), T, atol=1e-9):
        raise ValueError("DH factorisation failed")
    return row


def from_spec(spec: Spec) -> DH:
    m = poe_from_spec(spec)
    F = zero_pose_frames(spec)
    base = F[0].copy()          # Craig: frame 0 coincides with frame 1 at q1 = 0
    rows = [DHRow(0.0, 0.0, 0.0, 0.0)]
    for i in range(1, len(F)):
        rows.append(_factor(np.linalg.inv(F[i - 1]) @ F[i]))
    tool = np.linalg.inv(F[-1]) @ m.M
    return DH(tuple(rows), base, tool)


def fk(dh: DH, q) -> np.ndarray:
    T = dh.base.copy()
    for r, qi in zip(dh.rows, np.asarray(q, float)):
        T = T @ link_transform(r, qi)
    return T @ dh.tool


def table(dh: DH, names) -> str:
    lines = ["| i | joint | alpha_{i-1} (rad) | a_{i-1} (m) | theta offset (rad) | d_i (m) |", "|---|---|---|---|---|---|"]
    for i, (n, r) in enumerate(zip(names, dh.rows), 1):
        lines.append(f"| {i} | {n} | {r.alpha:+.6f} | {r.a:+.4f} | {r.theta0:+.6f} | {r.d:+.4f} |")
    return "\n".join(lines)
