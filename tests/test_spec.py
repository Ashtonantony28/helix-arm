"""M0: the spec loads, obeys ADR 0001, and the reference FK agrees with hand-computed poses."""
import copy
import math

import numpy as np
import pytest
import yaml

from helix.spec import JOINT_ORDER, SPEC_PATH, SpecError, fk, load_spec, parse, zero_pose_points


@pytest.fixture(scope="module")
def raw():
    return yaml.safe_load(SPEC_PATH.read_text())


def test_loads_in_adr_order():
    s = load_spec()
    assert tuple(j.name for j in s.joints) == JOINT_ORDER


def test_zero_pose_matches_plan_numbers(raw):
    p = zero_pose_points(load_spec())
    assert p["shoulder_pitch"] == pytest.approx([0.0, 0.0, 0.180])
    assert p["wrist_pitch"] == pytest.approx([0.520, 0.080, 0.180])   # wrist centre, 80 mm elbow offset (ADR 0003)
    assert p["tcp"] == pytest.approx([0.600, 0.080, 0.180])           # 600 mm reach in the arm plane (plan sec. 3)


def test_positive_pitch_is_down_and_positive_yaw_is_left():
    s = load_spec()
    q = np.zeros(6); q[1] = 0.3
    assert fk(s, q)["tcp"][2, 3] < 0.180
    q = np.zeros(6); q[0] = 0.3
    assert fk(s, q)["tcp"][1, 3] > 0.0


def test_shoulder_up_and_elbow_fold_hand_values():
    s = load_spec()
    q = np.zeros(6); q[1] = -math.pi / 2                              # upper arm straight up
    t = fk(s, q)["tcp"][:3, 3]
    assert t == pytest.approx([0.0, 0.080, 0.180 + 0.600])
    q = np.zeros(6); q[2] = math.pi / 2                               # forearm straight down
    t = fk(s, q)["tcp"][:3, 3]
    assert t == pytest.approx([0.260, 0.080, 0.180 - 0.340])


def test_spherical_wrist_centre_independent_of_wrist_joints():
    s = load_spec()
    rng = np.random.default_rng(0)
    for _ in range(200):
        q = rng.uniform(s.lower, s.upper)
        q2 = q.copy(); q2[3:] = rng.uniform(s.lower[3:], s.upper[3:])
        assert fk(s, q)["wrist_pitch"][:3, 3] == pytest.approx(fk(s, q2)["wrist_pitch"][:3, 3], abs=1e-12)


def test_rotations_are_proper():
    s = load_spec()
    q = np.random.default_rng(1).uniform(s.lower, s.upper)
    for T in fk(s, q).values():
        R = T[:3, :3]
        assert R @ R.T == pytest.approx(np.eye(3), abs=1e-12)
        assert np.linalg.det(R) == pytest.approx(1.0)


@pytest.mark.parametrize("mutate, msg", [
    (lambda r: r["joints"].reverse(), "joint order"),
    (lambda r: r["joints"][1].__setitem__("axis", [0, -1, 0]), "axis"),
    (lambda r: r["joints"][2].__setitem__("limits", [0.1, 2.0]), "bracket"),
    (lambda r: r["joints"][2].__setitem__("origin", [260.0, 0, 0]), "metres"),
    (lambda r: r["joints"][5].__setitem__("origin", [0.04, 0.01, 0.0]), "spherical"),
    (lambda r: r["ratings"].__setitem__("reach_m", 0.65), "reach"),
    (lambda r: r["vla_arm_bridge"].__setitem__("held", {}), "vla_arm_bridge"),
])
def test_rejects_convention_violations(raw, mutate, msg):
    bad = copy.deepcopy(raw)
    mutate(bad)
    with pytest.raises(SpecError, match=msg):
        parse(bad)


def test_bridge_matches_vla_arm_channel_order(raw):
    # vla-arm protocol v1: base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll, gripper
    assert raw["vla_arm_bridge"]["order"] == ["base_yaw", "shoulder_pitch", "elbow_pitch",
                                              "wrist_pitch", "wrist_roll", "gripper"]
    assert raw["vla_arm_bridge"]["held"] == {"forearm_roll": 0.0}


def test_payload_sizing_sanity(raw):
    """Plan sec. 3 hand estimate: J2 static ~ (1.0 kg x 0.6 m + ~2.5 kg x 0.3 m) x 9.81 ~ 13 N*m, well
    inside the first L-module peak rating (M2 replaces this with the full sizing loop)."""
    tau = (raw["ratings"]["payload_kg"] * raw["ratings"]["reach_m"] + 2.5 * 0.3) * 9.81
    assert 12.0 < tau < 14.0
    assert raw["modules"]["L"]["peak_torque_nm"] > 2 * tau
