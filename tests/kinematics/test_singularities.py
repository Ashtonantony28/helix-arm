"""|det J| against the analytic factorisation for an arm with a spherical wrist and a lateral offset:
|det J| = a2 * a3 * |sin q3| * |a2 cos q2 + a3 cos(q2 + q3)| * |sin q5|  (independent of D, q1, q4, q6).
Zeros: elbow (q3 = 0), shoulder (wrist centre on the offset cylinder), wrist (q5 = 0)."""
import numpy as np
import pytest

from helix.kinematics import ik, poe
from helix.kinematics.workspace import manipulability
from helix.spec import load_spec


@pytest.fixture(scope="module")
def setup():
    s = load_spec()
    return s, ik.Geometry.from_spec(s), poe.from_spec(s)


def analytic(g, q):
    return g.a2 * g.a3 * abs(np.sin(q[2])) * abs(g.a2 * np.cos(q[1]) + g.a3 * np.cos(q[1] + q[2])) * abs(np.sin(q[4]))


def test_det_matches_analytic_factorisation(setup):
    s, g, m = setup
    rng = np.random.default_rng(7)
    for _ in range(2000):
        q = rng.uniform(s.lower, s.upper)
        assert manipulability(m, q) == pytest.approx(analytic(g, q), abs=1e-14)


@pytest.mark.parametrize("q", [
    [0.3, -0.4, 0.0, 0.2, 0.7, 0.1],                    # elbow stretched
    [0.3, -0.4, 1.0, 0.2, 0.0, 0.1],                    # wrist: J4 || J6
    [0.3, -np.pi / 2 - 0.6, 1.2, 0.2, 0.7, 0.1],        # wrist centre on the offset cylinder
])
def test_singular_configurations(setup, q):
    s, g, m = setup
    assert manipulability(m, np.array(q)) < 1e-12
