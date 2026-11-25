"""Product-of-exponentials model generated from the spec (Lynch & Park, Modern Robotics ch. 4-5).

Space-frame screw axes ``S_i = (w_i, -w_i x p_i)`` and the home pose ``M`` are derived from
``spec/arm.yaml`` at the zero pose. ``fk`` here is an independent implementation of the forward
kinematics; tests check it against the reference chain in ``helix.spec.fk``.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from helix.spec import Spec, fk as chain_fk


def skew(v) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def exp6(S: np.ndarray, theta: float) -> np.ndarray:
    """exp([S] theta) for a unit-rotation screw S = (w, v), |w| = 1 (Rodrigues + MR eq. 3.88)."""
    w, v = S[:3], S[3:]
    W = skew(w)
    s, c = np.sin(theta), np.cos(theta)
    R = np.eye(3) + s * W + (1 - c) * W @ W
    G = np.eye(3) * theta + (1 - c) * W + (theta - s) * W @ W
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = G @ v
    return T


def adjoint(T: np.ndarray) -> np.ndarray:
    R, p = T[:3, :3], T[:3, 3]
    A = np.zeros((6, 6))
    A[:3, :3] = R
    A[3:, 3:] = R
    A[3:, :3] = skew(p) @ R
    return A


@dataclass(frozen=True)
class PoE:
    S: np.ndarray        # 6 x n space screw axes (w; v)
    M: np.ndarray        # 4 x 4 home pose of the TCP
    points: np.ndarray   # n x 3 points on each joint axis at the zero pose (joint origins)

    @property
    def n(self) -> int:
        return self.S.shape[1]


def from_spec(spec: Spec) -> PoE:
    home = chain_fk(spec, np.zeros(len(spec.joints)))
    S, pts = [], []
    for j in spec.joints:
        T = home[j.name]
        w = T[:3, :3] @ j.axis
        p = T[:3, 3]
        S.append(np.concatenate([w, -np.cross(w, p)]))
        pts.append(p)
    return PoE(np.array(S).T, home["tcp"], np.array(pts))


def fk(model: PoE, q) -> np.ndarray:
    T = np.eye(4)
    for i, qi in enumerate(np.asarray(q, float)):
        T = T @ exp6(model.S[:, i], qi)
    return T @ model.M


def jacobian_space(model: PoE, q) -> np.ndarray:
    """6 x n space Jacobian (rows: angular, linear), MR eq. 5.11."""
    J = np.zeros((6, model.n))
    T = np.eye(4)
    for i, qi in enumerate(np.asarray(q, float)):
        J[:, i] = adjoint(T) @ model.S[:, i]
        T = T @ exp6(model.S[:, i], qi)
    return J


def jacobian_tcp(model: PoE, q) -> np.ndarray:
    """6 x n geometric Jacobian of the TCP in base-frame coordinates (rows: linear velocity of the TCP
    point, angular velocity). This is the one used for manipulability and DLS IK."""
    Js = jacobian_space(model, q)
    p = fk(model, q)[:3, 3]
    Jw = Js[:3]
    Jv = Js[3:] - skew(p) @ Jw  # v_tcp = v_s + w x p
    return np.vstack([Jv, Jw])
