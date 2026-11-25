"""Load and validate ``spec/arm.yaml`` (the single source of truth; conventions in docs/adr/0001).

Everything downstream (kinematics, dynamics, CAD, firmware config, sim) reads geometry from here, so the
loader checks the conventions ADR 0001 fixes instead of trusting the file.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml

SPEC_PATH = Path(__file__).resolve().parents[1] / "spec" / "arm.yaml"

# ADR 0001: joint order and names are fixed; changing them is a new ADR.
JOINT_ORDER = ("base_yaw", "shoulder_pitch", "elbow_pitch", "forearm_roll", "wrist_pitch", "wrist_roll")
UNIT_AXES = {(0, 0, 1), (0, 1, 0), (1, 0, 0)}
MODULES = ("L", "M", "S")


@dataclass(frozen=True)
class Joint:
    name: str
    axis: np.ndarray
    origin: np.ndarray
    limits: tuple[float, float]
    vmax: float
    module: str


@dataclass(frozen=True)
class Spec:
    joints: tuple[Joint, ...]
    tcp: np.ndarray
    raw: dict

    def joint(self, name: str) -> Joint:
        return next(j for j in self.joints if j.name == name)

    @property
    def lower(self) -> np.ndarray:
        return np.array([j.limits[0] for j in self.joints])

    @property
    def upper(self) -> np.ndarray:
        return np.array([j.limits[1] for j in self.joints])


class SpecError(ValueError):
    pass


def _check(cond: bool, msg: str) -> None:
    if not cond:
        raise SpecError(msg)


def parse(raw: dict) -> Spec:
    js = raw.get("joints") or []
    names = tuple(j.get("name") for j in js)
    _check(names == JOINT_ORDER, f"joint order {names} != ADR 0001 order {JOINT_ORDER}")
    joints = []
    for j in js:
        axis = np.asarray(j["axis"], float)
        _check(tuple(int(a) for a in axis) in UNIT_AXES and np.all(axis == np.round(axis)),
               f"{j['name']}: axis must be +x, +y or +z (ADR 0001), got {axis.tolist()}")
        lo, hi = (float(x) for x in j["limits"])
        _check(lo < 0.0 < hi, f"{j['name']}: limits must bracket the zero pose, got {lo}, {hi}")
        _check(hi - lo <= 2 * np.pi, f"{j['name']}: range exceeds one turn")
        _check(float(j["vmax"]) > 0, f"{j['name']}: vmax must be positive")
        _check(j["module"] in MODULES and j["module"] in raw["modules"], f"{j['name']}: unknown module {j['module']}")
        origin = np.asarray(j["origin"], float)
        _check(origin.shape == (3,) and np.all(np.abs(origin) < 1.0), f"{j['name']}: origin must be metres (< 1 m)")
        joints.append(Joint(j["name"], axis, origin, (lo, hi), float(j["vmax"]), j["module"]))
    tcp = np.asarray(raw["tool"]["tcp"], float)
    spec = Spec(tuple(joints), tcp, raw)
    # spherical wrist: J4..J6 axes meet at the wrist centre (J5 origin) -> J4/J6 axes lie on the x line
    for n in ("forearm_roll", "wrist_roll"):
        jt = spec.joint(n)
        _check(tuple(jt.axis) == (1.0, 0.0, 0.0) and np.allclose(jt.origin[1:], 0.0),
               f"{n}: a spherical wrist needs a roll axis along the link x line")
    _check(np.allclose(spec.joint("wrist_pitch").origin[1:], 0.0), "wrist_pitch must sit on the link x line")
    r = raw["ratings"]
    # reach is measured in the arm plane (x-z), perpendicular to the shoulder axis: the elbow's lateral
    # offset (ADR 0003) moves the arm sideways but does not lengthen it
    zp = zero_pose_points(spec)
    xz = [0, 2]
    reach = float(np.linalg.norm((zp["tcp"] - zp["shoulder_pitch"])[xz]))
    _check(abs(reach - r["reach_m"]) < 1e-9, f"reach {reach} != ratings.reach_m {r['reach_m']}")
    wc = float(np.linalg.norm((zp["wrist_pitch"] - zp["shoulder_pitch"])[xz]))
    _check(abs(wc - r["wrist_centre_reach_m"]) < 1e-9, f"wrist centre reach {wc} != {r['wrist_centre_reach_m']}")
    b = raw["vla_arm_bridge"]
    _check(set(b["order"][:-1]) | set(b["held"]) == set(JOINT_ORDER) and b["order"][-1] == "gripper",
           "vla_arm_bridge must map 5 joints + gripper and hold the rest")
    return spec


@lru_cache(maxsize=4)
def load_spec(path: str | Path = SPEC_PATH) -> Spec:
    return parse(yaml.safe_load(Path(path).read_text()))


def rot(axis: np.ndarray, q: float) -> np.ndarray:
    """Rotation matrix for angle ``q`` about a unit axis (Rodrigues)."""
    k = np.asarray(axis, float)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


def fk(spec: Spec, q) -> dict[str, np.ndarray]:
    """4x4 base->frame transforms of every joint frame and the TCP (product of exponentials on the
    ADR 0001 joint chain). This is the reference the M1 DH/PoE models are tested against."""
    q = np.asarray(q, float)
    T = np.eye(4)
    out = {}
    for j, qi in zip(spec.joints, q):
        S = np.eye(4); S[:3, 3] = j.origin
        R = np.eye(4); R[:3, :3] = rot(j.axis, qi)
        T = T @ S @ R
        out[j.name] = T.copy()
    E = np.eye(4); E[:3, 3] = spec.tcp
    out["tcp"] = T @ E
    return out


def zero_pose_points(spec: Spec) -> dict[str, np.ndarray]:
    return {k: v[:3, 3] for k, v in fk(spec, np.zeros(len(spec.joints))).items()}
