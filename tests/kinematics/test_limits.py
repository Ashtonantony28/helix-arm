"""M1 spec freeze: every pitch limit in the spec lies inside the limit derived from the envelope, and two
derived contacts match closed-form geometry (tolerance: one sweep step, 1 mrad)."""
import math

import numpy as np
import pytest

from helix.kinematics import limits
from helix.spec import load_spec


@pytest.fixture(scope="module")
def derived():
    return {(c.joint, c.sign): c for c in limits.derive()}


def test_spec_limits_inside_derived(derived):
    s = load_spec()
    for (j, sign), c in derived.items():
        lo, hi = s.joint(j).limits
        assert (hi if sign > 0 else -lo) <= c.limit + 1e-12, (j, sign)


def test_spec_frozen():
    assert load_spec().raw["version"] == "1.0"


def test_wrist_contact_closed_form(derived):
    """Gripper start (40 mm past the wrist centre) against the forearm end (80 mm before it):
    |F - G|^2 = f^2 + g^2 + 2 f g cos(theta) = (r_f + r_g + margin)^2."""
    f, g, D = 0.080, 0.040, 0.035 + 0.040 + 0.005
    theta = math.acos((D * D - f * f - g * g) / (2 * f * g))
    for sign in (1, -1):
        assert derived[("wrist_pitch", sign)].angle == pytest.approx(theta, abs=1e-3)
        assert derived[("wrist_pitch", sign)].pair == "forearm/gripper"


def test_shoulder_contact_closed_form(derived):
    """J1 module top (0, 0, 0.040), r 60 mm, against the upper-arm line through the J2 axis (z 0.180):
    distance 0.140 cos(theta) = 0.060 + 0.035 + 0.005."""
    theta = math.acos(0.100 / 0.140)
    c = derived[("shoulder_pitch", 1)]
    assert c.angle == pytest.approx(theta, abs=1e-3) and c.pair == "base_column/upper_arm"


def test_limit_rounding(derived):
    for c in derived.values():
        if c.angle is not None:
            assert c.limit <= c.angle - limits.SAFETY_RAD + 1e-9
            assert abs(c.limit / limits.STEP_RAD - round(c.limit / limits.STEP_RAD)) < 1e-9
