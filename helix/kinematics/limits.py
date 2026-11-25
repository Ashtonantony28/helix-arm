"""Derive the pitch-joint limits from the collision envelope (ADR 0003).

A joint's limit is where the links on its two sides first come within ``collision.margin_m`` of each
other while every other joint is at zero, considering only capsules that are rigid with respect to that
joint's two sides:

- shoulder_pitch: capsules on base, base_yaw, shoulder_pitch (the upper arm against the turret);
- elbow_pitch: capsules on shoulder_pitch and everything distal (forearm/wrist/gripper folding onto the
  upper arm), wrist straight;
- wrist_pitch: capsules on elbow_pitch and everything distal (gripper against the forearm).

Contacts that depend on more than one joint (wrist into the base, anything into the table) are not joint
limits; they are pose-dependent and belong to the host-side collision check, as in vla-arm. The limit is
the first contact angle minus ``SAFETY_RAD``, rounded down to ``STEP_RAD``.

    python -m helix.kinematics.limits      # prints the derivation and the proposed limits
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from helix.kinematics import collision as C
from helix.spec import JOINT_ORDER, Spec, load_spec

SAFETY_RAD = 0.02
STEP_RAD = 0.05
SWEEP_STEP_RAD = 0.001
DERIVED = {
    "shoulder_pitch": ("base", "base_yaw", "shoulder_pitch"),
    "elbow_pitch": JOINT_ORDER[1:],
    "wrist_pitch": JOINT_ORDER[2:],
}


@dataclass(frozen=True)
class Contact:
    joint: str
    sign: int
    angle: float | None       # first contact angle, None if free over the half-turn
    pair: str
    limit: float              # proposed |limit|


def _restricted(env: C.Envelope, links) -> C.Envelope:
    keep = tuple((i, j) for i, j in env.pairs if env.capsules[i].link in links and env.capsules[j].link in links)
    return C.Envelope(env.capsules, keep, env.margin, env.table_z)


def first_contact(spec: Spec, env: C.Envelope, joint: str, sign: int) -> Contact:
    i = JOINT_ORDER.index(joint)
    sub = _restricted(env, DERIVED[joint])
    q = np.zeros(len(JOINT_ORDER))
    for a in np.arange(0.0, math.pi + 1e-12, SWEEP_STEP_RAD):
        q[i] = sign * a
        r = C.check(sub, spec, q)
        if r.self_collides:
            lim = math.floor((a - SAFETY_RAD) / STEP_RAD + 1e-9) * STEP_RAD
            return Contact(joint, sign, float(a), r.self_worst, round(lim, 4))
    return Contact(joint, sign, None, "", math.pi)


def derive(spec: Spec | None = None) -> list[Contact]:
    spec = spec or load_spec()
    env = C.from_spec(spec)
    return [first_contact(spec, env, j, s) for j in DERIVED for s in (1, -1)]


def report(spec: Spec | None = None) -> str:
    spec = spec or load_spec()
    lines = ["| joint | direction | first contact (rad) | pair | derived limit (rad) | spec limit (rad) |",
             "|---|---|---|---|---|---|"]
    for c in derive(spec):
        lo, hi = spec.joint(c.joint).limits
        sl = hi if c.sign > 0 else lo
        ang = "free" if c.angle is None else f"{c.sign * c.angle:+.3f}"
        lines.append(f"| {c.joint} | {'+' if c.sign > 0 else '-'} | {ang} | {c.pair or '-'} | "
                     f"{c.sign * c.limit:+.2f} | {sl:+.2f} |")
    return "\n".join(lines)


if __name__ == "__main__":
    print(report())
