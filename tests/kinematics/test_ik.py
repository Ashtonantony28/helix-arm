"""Closed-form IK is exact on 20,000 random poses (plan M1 gate) and matches the golden fixture.

Tolerances: reconstructed pose within 1e-9 (m and rotation-matrix entries); the generating joint vector is
among the solutions to 1e-7 rad away from singularities (|sin q3|, |sin q5| > 1e-3)."""
import json
from pathlib import Path

import numpy as np
import pytest

from helix.kinematics import golden, ik, poe
from helix.spec import load_spec

N_POSES = 20000


@pytest.fixture(scope="module")
def setup():
    s = load_spec()
    return s, ik.Geometry.from_spec(s), poe.from_spec(s)


def test_ik_exact_on_20k_random_poses(setup):
    s, g, m = setup
    rng = np.random.default_rng(20261125)
    worst, missed, counts = 0.0, 0, {}
    for _ in range(N_POSES):
        q = rng.uniform(s.lower, s.upper)
        T = poe.fk(m, q)
        sols = ik.solve(g, T)
        counts[len(sols)] = counts.get(len(sols), 0) + 1
        for x in sols:
            worst = max(worst, float(np.abs(poe.fk(m, x) - T).max()))
        if abs(np.sin(q[2])) > 1e-3 and abs(np.sin(q[4])) > 1e-3:
            if not any(np.abs(ik.wrap(x - q)).max() < 1e-7 for x in sols):
                missed += 1
    assert worst < 1e-9, worst
    assert missed == 0
    assert counts == {8: N_POSES}     # generic poses have all 8 Pieper branches


def test_eight_branches_are_distinct(setup):
    s, g, m = setup
    q = np.array([0.4, -0.6, 1.1, 0.5, 0.9, -0.3])
    sols = ik.solve(g, poe.fk(m, q))
    for i in range(8):
        for j in range(i + 1, 8):
            assert np.abs(ik.wrap(sols[i] - sols[j])).max() > 1e-3


def test_nearest_returns_seed_and_respects_limits(setup):
    s, g, m = setup
    rng = np.random.default_rng(5)
    for _ in range(500):
        q = rng.uniform(s.lower, s.upper)
        x = ik.nearest(g, poe.fk(m, q), q)
        assert np.all(x >= s.lower - 1e-12) and np.all(x <= s.upper + 1e-12)
        if abs(np.sin(q[4])) > 1e-3 and abs(np.sin(q[2])) > 1e-3:
            assert np.abs(x - q).max() < 1e-7


def test_wrist_singularity(setup):
    """q5 = 0: J4 and J6 aligned; one family per position branch (q4 from the hint) where s5 is exactly
    zero, two where rounding leaves it tiny; either way every solution reproduces the pose."""
    s, g, m = setup
    q = np.array([0.2, -0.3, 1.0, 0.7, 0.0, -0.4])
    T = poe.fk(m, q)
    sols = ik.solve(g, T, q4_hint=0.7)
    assert 4 <= len(sols) <= 8
    assert any(np.abs(ik.wrap(x - q)).max() < 1e-9 for x in sols)
    for x in sols:
        assert np.abs(poe.fk(m, x) - T).max() < 1e-9


def test_unreachable_raises(setup):
    s, g, m = setup
    T = np.eye(4); T[:3, 3] = [0.9, 0.0, 0.18]
    with pytest.raises(ik.IKError):
        ik.solve(g, T)


def test_offset_cylinder_is_handled(setup):
    """Wrist centre exactly on the lateral-offset cylinder (rho = 0): solutions still reproduce the pose."""
    s, g, m = setup
    # choose q2, q3 so the wrist centre is straight above the J2 axis: a2 c2 + a3 c23 = 0
    q3 = 1.2
    q2 = -np.pi / 2 - q3 / 2
    q = np.array([0.3, q2, q3, 0.2, 0.8, 0.1])
    T = poe.fk(m, q)
    sols = ik.solve(g, T)
    assert sols and all(np.abs(poe.fk(m, x) - T).max() < 1e-9 for x in sols)


def test_dls_converges_from_perturbed_seeds(setup):
    s, g, m = setup
    rng = np.random.default_rng(6)
    for _ in range(100):
        q = rng.uniform(s.lower * 0.9, s.upper * 0.9)
        T = poe.fk(m, q)
        x, ok = ik.dls(m, T, q + rng.normal(0, 0.15, 6))   # includes near-singular targets
        assert ok
        assert np.abs(poe.fk(m, x) - T).max() < 1e-9


def test_golden_fixture():
    data = json.loads(Path(golden.PATH).read_text())
    s = load_spec()
    assert data["spec_version"] == s.raw["version"], "spec changed: regenerate the fixture (reviewed change)"
    g = ik.Geometry.from_spec(s)
    for c in data["cases"]:
        sols = sorted(np.round(x, 12).tolist() for x in ik.solve(g, np.array(c["T"])))
        assert np.allclose(np.array(sols), np.array(c["solutions"]), atol=1e-9)


def test_dls_starts_from_a_singular_seed(setup):
    """Seed with elbow stretched and wrist aligned (two singularities at once): damping carries it out."""
    s, g, m = setup
    q = np.array([0.5, -0.4, 1.3, 0.6, 0.9, -0.2])
    x, ok = ik.dls(m, poe.fk(m, q), np.zeros(6))
    assert ok and np.abs(poe.fk(m, x) - poe.fk(m, q)).max() < 1e-9
