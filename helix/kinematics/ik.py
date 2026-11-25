"""Analytic inverse kinematics (Pieper decomposition) plus a damped-least-squares fallback.

Geometry the closed form relies on (checked by ``Geometry.from_spec``; ADR 0001, ADR 0003):
J1 yaw about the base z axis; J2 and J3 parallel pitch axes (+y); a constant lateral offset ``D`` between
the J1 axis and the arm plane (the elbow offset of ADR 0003); J4-J6 a roll-pitch-roll spherical wrist
whose centre is the J5 origin; the TCP on the J6 axis.

Position: wrist centre -> J1 (2 branches: front / back) -> planar two-link J2, J3 (elbow up / down).
Orientation: R_36 = Rx(q4) Ry(q5) Rx(q6), X-Y-X Euler angles (wrist flip: q5 sign, 2 branches).
That gives up to 8 solutions; ``solve`` returns all of them, wrapped to (-pi, pi] and, optionally,
moved by 2 pi into the joint limits where that helps.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from helix.kinematics import poe
from helix.spec import Spec

SING_EPS = 1e-9


class IKError(ValueError):
    pass


@dataclass(frozen=True)
class Geometry:
    h: float        # height of the J2 axis above the base frame
    D: float        # lateral (y) offset of the arm plane from the J1 axis, in the J1 frame
    a2: float       # J2 -> J3 (upper arm)
    a3: float       # J3 -> wrist centre (forearm), in the arm plane
    d6: float       # wrist centre -> TCP along the J6 axis
    lower: np.ndarray
    upper: np.ndarray

    @staticmethod
    def from_spec(spec: Spec) -> "Geometry":
        j = {jt.name: jt for jt in spec.joints}
        o1, o2, o3, o4, o5, o6 = (j[n].origin for n in
                                  ("base_yaw", "shoulder_pitch", "elbow_pitch", "forearm_roll", "wrist_pitch", "wrist_roll"))
        need = [
            (np.allclose(o1[:2], 0), "J1 origin must be on the base z axis"),
            (np.allclose(o2[0], 0), "J2 origin must have no forward offset"),
            (np.allclose(o3[2], 0) and np.allclose(o4[1:], 0) and np.allclose(o5[1:], 0),
             "upper arm and forearm must lie in one plane (only a y offset at the elbow)"),
            (np.allclose(o6[1:], 0) and np.allclose(spec.tcp[1:], 0), "TCP must be on the J6 axis"),
        ]
        for ok, msg in need:
            if not ok:
                raise IKError(f"closed-form IK precondition: {msg}")
        return Geometry(h=float(o1[2] + o2[2]), D=float(o2[1] + o3[1]), a2=float(o3[0]),
                        a3=float(o4[0] + o5[0]), d6=float(o6[0] + spec.tcp[0]),
                        lower=spec.lower.copy(), upper=spec.upper.copy())


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def _Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def wrist_centre(g: Geometry, T: np.ndarray) -> np.ndarray:
    return T[:3, 3] - g.d6 * T[:3, 0]


def position_branches(g: Geometry, pw: np.ndarray):
    """(q1, q2, q3) for the wrist centre ``pw``; up to 4 tuples. Raises IKError if unreachable."""
    px, py, pz = pw
    rho2 = px * px + py * py - g.D * g.D
    if rho2 < -1e-12:
        raise IKError("wrist centre inside the lateral-offset cylinder")
    rho = np.sqrt(max(rho2, 0.0))
    w = -(pz - g.h)                                   # positive pitch is down (ADR 0001)
    out = []
    for X in (rho, -rho):
        q1 = np.arctan2(py, px) - np.arctan2(g.D, X)
        c3 = (X * X + w * w - g.a2 ** 2 - g.a3 ** 2) / (2 * g.a2 * g.a3)
        if c3 > 1 + 1e-10 or c3 < -1 - 1e-10:
            continue
        c3 = float(np.clip(c3, -1.0, 1.0))
        for s in (1.0, -1.0):
            q3 = s * np.arccos(c3)
            q2 = np.arctan2(w, X) - np.arctan2(g.a3 * np.sin(q3), g.a2 + g.a3 * np.cos(q3))
            out.append((q1, q2, q3))
        if rho < 1e-12:                               # on the offset cylinder: both X give the same set
            break
    if not out:
        raise IKError("wrist centre out of reach")
    return out


def wrist_branches(R36: np.ndarray, q4_hint: float = 0.0):
    """(q4, q5, q6) with Rx(q4) Ry(q5) Rx(q6) = R36; 2 tuples (1 family at the q5 = 0 singularity, where
    q4 = q4_hint and q6 takes the rest)."""
    c5 = float(np.clip(R36[0, 0], -1.0, 1.0))
    s5 = np.hypot(R36[1, 0], R36[2, 0])
    if s5 < SING_EPS:
        q5 = 0.0 if c5 > 0 else np.pi
        # R36 = Rx(q4) Ry(q5) Rx(q6) collapses to Rx(q4 +- q6)
        phi = np.arctan2(R36[2, 1], R36[1, 1])
        q4 = q4_hint
        q6 = phi - q4 if c5 > 0 else q4 - phi
        return [(q4, q5, q6)]
    q5 = np.arctan2(s5, c5)
    q4 = np.arctan2(R36[1, 0], -R36[2, 0])
    q6 = np.arctan2(R36[0, 1], R36[0, 2])
    return [(q4, q5, q6), (q4 + np.pi, -q5, q6 + np.pi)]


def solve(g: Geometry, T: np.ndarray, *, within_limits: bool = False, q4_hint: float = 0.0) -> list[np.ndarray]:
    """All closed-form solutions for the TCP pose ``T`` (4x4, base frame)."""
    R = T[:3, :3]
    sols = []
    for q1, q2, q3 in position_branches(g, wrist_centre(g, T)):
        R03 = _Rz(q1) @ _Ry(q2 + q3)
        for q4, q5, q6 in wrist_branches(R03.T @ R, q4_hint):
            q = wrap(np.array([q1, q2, q3, q4, q5, q6]))
            if within_limits:
                q = _into_limits(g, q)
                if q is None:
                    continue
            sols.append(q)
    return sols


def _into_limits(g: Geometry, q: np.ndarray):
    out = q.copy()
    for i in range(len(q)):
        for k in (0.0, 2 * np.pi, -2 * np.pi):
            if g.lower[i] - 1e-12 <= q[i] + k <= g.upper[i] + 1e-12:
                out[i] = q[i] + k
                break
        else:
            return None
    return out


def nearest(g: Geometry, T: np.ndarray, q_ref) -> np.ndarray:
    """The in-limit closed-form solution closest to ``q_ref`` (weighted joint distance)."""
    q_ref = np.asarray(q_ref, float)
    sols = solve(g, T, within_limits=True, q4_hint=float(q_ref[3]))
    if not sols:
        raise IKError("no solution inside the joint limits")
    return min(sols, key=lambda q: float(np.sum((q - q_ref) ** 2)))


def pose_error(Ta: np.ndarray, Tb: np.ndarray) -> np.ndarray:
    """6-vector (position error; rotation-vector error) of Tb relative to Ta, both in the base frame."""
    dp = Tb[:3, 3] - Ta[:3, 3]
    Re = Tb[:3, :3] @ Ta[:3, :3].T
    v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    sn = np.linalg.norm(v) / 2                       # sin(angle)
    cs = (np.trace(Re) - 1) / 2                      # cos(angle)
    ang = np.arctan2(sn, cs)                         # well conditioned for small angles (unlike arccos)
    if sn > 1e-9:
        dw = ang * v / (2 * sn)
    elif cs > 0:
        dw = v / 2                                   # first order: log(R) ~ (R - R^T) / 2
    else:
        dw = _axis_pi(Re) * ang
    return np.concatenate([dp, dw])


def _axis_pi(R):
    B = (R + np.eye(3)) / 2
    i = int(np.argmax(np.diag(B)))
    v = B[:, i] / np.sqrt(B[i, i])
    return v


def dls(model: poe.PoE, T: np.ndarray, q0, *, wbar: float = 1e-6, tol: float = 1e-12, iters: int = 500,
        lower=None, upper=None):
    """Levenberg-Marquardt / damped least-squares IK with error-proportional damping (Sugihara 2011,
    "Solvability-unconcerned inverse kinematics by the Levenberg-Marquardt method"):
    dq = (J^T J + W_N)^-1 J^T e,  W_N = (|e|^2 + wbar) I. Far from the target the damping keeps steps
    bounded (and passes through singularities); near it the step becomes Gauss-Newton, so convergence is
    fast even when the target pose is close to singular. Returns (q, converged)."""
    q = np.asarray(q0, float).copy()
    for _ in range(iters):
        e = pose_error(poe.fk(model, q), T)
        if np.linalg.norm(e) < tol:
            return q, True
        J = poe.jacobian_tcp(model, q)
        H = J.T @ J + (e @ e + wbar) * np.eye(len(q))
        q = q + np.linalg.solve(H, J.T @ e)
        if lower is not None:
            q = np.clip(q, lower, upper)
    return q, bool(np.linalg.norm(pose_error(poe.fk(model, q), T)) < tol)
