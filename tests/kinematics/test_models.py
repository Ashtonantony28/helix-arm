"""PoE and DH models generated from the spec agree with the reference chain (helix.spec.fk), and the
TCP Jacobian agrees with finite differences. Tolerances: 1e-12 m / rad for the models, 1e-6 for FD."""
import numpy as np
import pytest

from helix.kinematics import dh, poe
from helix.spec import fk, load_spec


@pytest.fixture(scope="module")
def spec():
    return load_spec()


def test_poe_matches_chain(spec):
    m = poe.from_spec(spec)
    rng = np.random.default_rng(10)
    for _ in range(2000):
        q = rng.uniform(spec.lower, spec.upper)
        assert np.abs(poe.fk(m, q) - fk(spec, q)["tcp"]).max() < 1e-12


def test_dh_matches_chain(spec):
    d = dh.from_spec(spec)
    rng = np.random.default_rng(11)
    for _ in range(2000):
        q = rng.uniform(spec.lower, spec.upper)
        assert np.abs(dh.fk(d, q) - fk(spec, q)["tcp"]).max() < 1e-12


def test_dh_table_hand_values(spec):
    """Upper arm appears as a_2 = 0.26, the elbow offset as d_3 = 0.08, the forearm as d_4 = 0.26, and
    every other length is zero (spherical wrist, no shoulder offset)."""
    r = dh.from_spec(spec).rows
    assert r[2].a == pytest.approx(0.260) and abs(r[2].d) == pytest.approx(0.080)
    assert abs(r[3].d) == pytest.approx(0.260)
    for i, row in enumerate(r):
        if i not in (2, 3):
            assert abs(row.a) < 1e-12 and abs(row.d) < 1e-12
    for i in (1, 3, 4, 5):     # consecutive axes perpendicular
        assert abs(abs(r[i].alpha) - np.pi / 2) < 1e-12
    assert abs(r[2].alpha) < 1e-12  # J2 || J3


def test_dh_handles_skew_and_parallel_lines():
    """A synthetic chain with a skew pair (x offset between z and y axes) still factors exactly."""
    import copy

    import yaml

    from helix.spec import SPEC_PATH, parse
    raw = yaml.safe_load(SPEC_PATH.read_text())
    raw2 = copy.deepcopy(raw)
    raw2["joints"][1]["origin"] = [0.03, 0.02, 0.12]   # shoulder ahead of and beside the J1 axis
    s2 = parse(raw2)
    d = dh.from_spec(s2)
    rng = np.random.default_rng(12)
    for _ in range(300):
        q = rng.uniform(s2.lower, s2.upper)
        assert np.abs(dh.fk(d, q) - fk(s2, q)["tcp"]).max() < 1e-12


def test_tcp_jacobian_matches_finite_differences(spec):
    m = poe.from_spec(spec)
    rng = np.random.default_rng(13)
    h = 1e-7
    for _ in range(50):
        q = rng.uniform(spec.lower, spec.upper)
        J = poe.jacobian_tcp(m, q)
        T0 = poe.fk(m, q)
        for i in range(6):
            dq = np.zeros(6); dq[i] = h
            T1 = poe.fk(m, q + dq)
            v = (T1[:3, 3] - T0[:3, 3]) / h
            W = (T1[:3, :3] @ T0[:3, :3].T - np.eye(3)) / h
            w = np.array([W[2, 1], W[0, 2], W[1, 0]])
            assert np.allclose(J[:3, i], v, atol=1e-6) and np.allclose(J[3:, i], w, atol=1e-6)
