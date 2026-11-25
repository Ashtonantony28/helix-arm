"""Workspace numbers against closed forms, and the report builds."""
import numpy as np
import pytest

from helix.kinematics import ik, workspace
from helix.spec import load_spec


def test_monte_carlo_reach_bounds():
    s = load_spec()
    g = ik.Geometry.from_spec(s)
    mc = workspace.monte_carlo(s, 3000, seed=1)
    rho = np.sqrt(np.maximum(mc.wc[:, 0] ** 2 + mc.wc[:, 1] ** 2 - g.D ** 2, 0))
    d = np.hypot(rho, mc.wc[:, 2] - g.h)
    dmin = np.sqrt(g.a2 ** 2 + g.a3 ** 2 + 2 * g.a2 * g.a3 * np.cos(s.upper[2]))
    assert d.max() <= g.a2 + g.a3 + 1e-12
    assert d.min() >= dmin - 1e-12
    # the lateral offset keeps every wrist centre at least D from the J1 axis
    assert np.all(np.hypot(mc.wc[:, 0], mc.wc[:, 1]) >= g.D - 1e-12)


def test_tool_down_pose():
    T = workspace.tool_down([0.3, 0.08, 0.1])
    assert np.allclose(T[:3, 0], [0, 0, -1]) and np.allclose(T[:3, 1], [0, 1, 0])
    assert np.isclose(np.linalg.det(T[:3, :3]), 1.0)


def test_report_builds(tmp_path):
    pytest.importorskip("matplotlib")
    st = workspace.build(tmp_path, samples=300)
    assert (tmp_path / "kinematics.md").exists() and (tmp_path / "workspace_rz.png").exists()
    assert st["tool_down_reach_at_z005"] is not None
