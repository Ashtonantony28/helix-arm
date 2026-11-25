"""Capsule envelope: segment distance against closed-form cases and brute force; envelope parsing."""
import copy

import numpy as np
import pytest
import yaml

from helix.kinematics import collision as C
from helix.spec import SPEC_PATH, load_spec, parse


def brute(p1, q1, p2, q2, n=401):
    t = np.linspace(0, 1, n)
    A = p1 + np.outer(t, q1 - p1)
    B = p2 + np.outer(t, q2 - p2)
    return np.sqrt(((A[:, None, :] - B[None, :, :]) ** 2).sum(-1)).min()


def test_segment_distance_closed_forms():
    v = np.array
    assert C.segment_distance(v([0, 0, 0.]), v([1, 0, 0.]), v([0, 1, 0.]), v([1, 1, 0.])) == pytest.approx(1.0)  # parallel
    assert C.segment_distance(v([-1, 0, 0.]), v([1, 0, 0.]), v([0, -1, 2.]), v([0, 1, 2.])) == pytest.approx(2.0)  # skew crossing
    assert C.segment_distance(v([0, 0, 0.]), v([1, 0, 0.]), v([3, 0, 0.]), v([4, 0, 0.])) == pytest.approx(2.0)  # collinear gap
    assert C.segment_distance(v([0, 0, 0.]), v([0, 0, 0.]), v([1, 1, 0.]), v([1, 1, 0.])) == pytest.approx(np.sqrt(2))  # points


def test_segment_distance_vs_brute_force():
    rng = np.random.default_rng(8)
    for _ in range(300):
        p = rng.normal(size=(4, 3))
        d = C.segment_distance(*p)
        assert d <= brute(*p) + 1e-12
        assert d == pytest.approx(brute(*p), abs=2e-2)


def test_zero_pose_is_clear():
    s = load_spec()
    r = C.check(C.from_spec(s), s, np.zeros(6))
    assert not r.collides


def test_upper_arm_and_forearm_never_collide():
    """The 80 mm elbow offset (ADR 0003) lets the forearm fold past the upper arm: tube radii 35 + 35 mm
    plus 5 mm margin < 80 mm at any elbow angle."""
    s = load_spec()
    env = C.from_spec(s)
    ia = [k.name for k in env.capsules].index("upper_arm")
    ib = [k.name for k in env.capsules].index("forearm")
    for q3 in np.linspace(s.lower[2], s.upper[2], 300):
        q = np.zeros(6); q[2] = q3
        seg = C.world_segments(env, s, q)
        d = C.segment_distance(*seg[ia], *seg[ib]) - 0.070
        assert d >= env.margin - 1e-12


def test_reaching_into_the_table_is_detected():
    s = load_spec()
    q = np.zeros(6); q[1] = 0.75; q[2] = 1.0
    assert C.check(C.from_spec(s), s, q).table_collides


def test_envelope_rejects_bad_entries():
    raw = yaml.safe_load(SPEC_PATH.read_text())
    bad = copy.deepcopy(raw)
    bad["collision"]["capsules"][0]["link"] = "nope"
    with pytest.raises(ValueError):
        C.from_spec(parse(bad))
    bad = copy.deepcopy(raw)
    bad["collision"]["skip"].append(["upper_arm", "ghost"])
    with pytest.raises(ValueError):
        C.from_spec(parse(bad))


def test_skip_pairs_and_same_link_pairs_are_not_checked():
    s = load_spec()
    env = C.from_spec(s)
    names = [k.name for k in env.capsules]
    checked = {frozenset((names[i], names[j])) for i, j in env.pairs}
    for a, b in s.raw["collision"]["skip"]:
        assert frozenset((a, b)) not in checked
    assert frozenset(("upper_arm", "elbow_module")) not in checked
    assert frozenset(("upper_arm", "wrist_module")) in checked
