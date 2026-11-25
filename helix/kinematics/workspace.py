"""Workspace and manipulability maps, and the M1 kinematics report.

    python -m helix.kinematics.workspace [--samples N] [--out docs/kinematics]

writes ``kinematics.md`` (DH table, limit derivation, workspace numbers) and two PNG maps:
``workspace_rz.png`` (reachable TCP and wrist-centre region in the arm plane from Monte Carlo joint
samples, split into collision-free / pose-dependent collision) and ``manipulability_rz.png`` (Yoshikawa
manipulability of the 6x6 TCP Jacobian with the gripper pointing straight down, best in-limit,
collision-free IK branch at each grid point).
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from helix.kinematics import collision as C
from helix.kinematics import dh as DH
from helix.kinematics import ik, limits, poe
from helix.spec import Spec, fk, load_spec

DEFAULT_OUT = Path(__file__).resolve().parents[2] / "docs" / "kinematics"


def manipulability(model: poe.PoE, q) -> float:
    """Yoshikawa measure sqrt(det(J J^T)) = |det J| for the square 6x6 TCP Jacobian."""
    return float(abs(np.linalg.det(poe.jacobian_tcp(model, q))))


def tool_down(p) -> np.ndarray:
    """TCP pose at point p with the tool x axis pointing down (-z) and the jaws closing along y."""
    T = np.eye(4)
    T[:3, :3] = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])  # Ry(+pi/2): x -> -z
    T[:3, 3] = p
    return T


@dataclass
class MonteCarlo:
    tcp: np.ndarray
    wc: np.ndarray
    self_hit: np.ndarray
    table_hit: np.ndarray


def monte_carlo(spec: Spec, n: int, seed: int = 0) -> MonteCarlo:
    env = C.from_spec(spec)
    rng = np.random.default_rng(seed)
    tcp, wc, sh, th = [], [], [], []
    for _ in range(n):
        q = rng.uniform(spec.lower, spec.upper)
        T = fk(spec, q)
        tcp.append(T["tcp"][:3, 3]); wc.append(T["wrist_pitch"][:3, 3])
        r = C.check(env, spec, q)
        sh.append(r.self_collides); th.append(r.table_collides)
    return MonteCarlo(np.array(tcp), np.array(wc), np.array(sh), np.array(th))


def manip_grid(spec: Spec, r_vals, z_vals):
    """Manipulability over an (r, z) grid in the y = D plane (the arm plane at q1 = 0); NaN where no
    in-limit, collision-free solution exists for the tool-down pose."""
    model, g, env = poe.from_spec(spec), ik.Geometry.from_spec(spec), C.from_spec(spec)
    W = np.full((len(z_vals), len(r_vals)), np.nan)
    for a, z in enumerate(z_vals):
        for b, r in enumerate(r_vals):
            T = tool_down([r, g.D, z])
            try:
                sols = ik.solve(g, T, within_limits=True)
            except ik.IKError:
                continue
            best = [manipulability(model, q) for q in sols if not C.check(env, spec, q).collides]
            if best:
                W[a, b] = max(best)
    return W


def build(out: Path = DEFAULT_OUT, samples: int = 20000) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    spec = load_spec()
    g = ik.Geometry.from_spec(spec)
    out.mkdir(parents=True, exist_ok=True)
    mc = monte_carlo(spec, samples)
    ok = ~(mc.self_hit | mc.table_hit)
    def rad(p):  # distance from the J1 axis measured in the arm plane (which is offset by D)
        return np.sqrt(np.maximum(p[:, 0] ** 2 + p[:, 1] ** 2 - g.D ** 2, 0.0))

    tcp_r = rad(mc.tcp)
    d_wc = np.hypot(rad(mc.wc), mc.wc[:, 2] - g.h)

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.scatter(tcp_r[ok], mc.tcp[ok, 2], s=1, c="#3b7dd8", alpha=0.35, label="TCP, collision-free")
    ax.scatter(tcp_r[~ok], mc.tcp[~ok, 2], s=1, c="#d8553b", alpha=0.25, label="TCP, pose-dependent collision")
    t = np.linspace(0, 2 * np.pi, 400)
    R = g.a2 + g.a3
    ax.plot(R * np.cos(t), g.h + R * np.sin(t), "k--", lw=1, label=f"wrist-centre reach {R:.3f} m")
    ax.axhline(spec.raw["collision"]["table_z"], color="0.3", lw=1)
    ax.set_xlabel("radial distance in the arm plane (m)"); ax.set_ylabel("z (m)")
    ax.set_aspect("equal"); ax.legend(loc="lower left", markerscale=8, fontsize=8)
    ax.set_title(f"Helix workspace, {samples} uniform joint samples")
    fig.tight_layout(); fig.savefig(out / "workspace_rz.png", dpi=110); plt.close(fig)

    r_vals = np.linspace(0.0, 0.62, 63)
    z_vals = np.linspace(-0.10, 0.80, 91)
    W = manip_grid(spec, r_vals, z_vals)
    fig, ax = plt.subplots(figsize=(7, 6))
    im = ax.imshow(W, origin="lower", extent=[r_vals[0], r_vals[-1], z_vals[0], z_vals[-1]], cmap="viridis",
                   aspect="equal")
    fig.colorbar(im, ax=ax, label="|det J| (Yoshikawa manipulability)")
    ax.axhline(spec.raw["collision"]["table_z"], color="w", lw=1)
    ax.set_xlabel("forward distance x (m), arm plane at q1 = 0"); ax.set_ylabel("TCP z (m)")
    ax.set_title("Manipulability, gripper pointing down")
    fig.tight_layout(); fig.savefig(out / "manipulability_rz.png", dpi=110); plt.close(fig)

    table_cells = W[np.argmin(np.abs(z_vals - 0.05))]
    reach_down = r_vals[np.isfinite(table_cells)]
    stats = {
        "samples": samples,
        "self_collision_frac": float(mc.self_hit.mean()),
        "table_collision_frac": float(mc.table_hit.mean()),
        "wc_reach_max": float(d_wc.max()),
        "wc_reach_min": float(d_wc.min()),
        "tool_down_reach_at_z005": (float(reach_down.min()), float(reach_down.max())) if len(reach_down) else None,
        "tool_down_cells_frac": float(np.isfinite(W).mean()),
    }
    model = DH.from_spec(spec)
    md = [
        "# Helix kinematics (M1, spec v" + str(spec.raw["version"]) + ")",
        "",
        "Generated by `python -m helix.kinematics.workspace`; do not edit. Everything is computed from",
        "`spec/arm.yaml`; nothing is measured (ADR 0002).",
        "",
        "## Modified DH parameters (derived view of the spec, ADR 0001)",
        "",
        DH.table(model, [j.name for j in spec.joints]),
        "",
        f"Arm-plane geometry used by the closed-form IK: shoulder height {g.h:.3f} m, lateral elbow offset "
        f"{g.D:.3f} m (ADR 0003), upper arm {g.a2:.3f} m, forearm to wrist centre {g.a3:.3f} m, wrist centre to "
        f"TCP {g.d6:.3f} m.",
        "",
        "## Joint limits from the collision envelope",
        "",
        limits.report(spec),
        "",
        "`free` means no contact over the half turn; that limit is kept from the plan (no need to reach further back).",
        "",
        "## Workspace",
        "",
        f"- {samples} uniform samples inside the joint limits.",
        f"- Wrist-centre distance from the shoulder axis: {stats['wc_reach_min']:.3f} .. {stats['wc_reach_max']:.3f} m"
        f" (analytic {g.a2 + g.a3:.3f} m max, {np.sqrt(g.a2**2 + g.a3**2 + 2*g.a2*g.a3*np.cos(spec.upper[2])):.3f} m"
        " min at the elbow limit).",
        f"- Pose-dependent self-collision in {100*stats['self_collision_frac']:.1f} % of samples and table contact in "
        f"{100*stats['table_collision_frac']:.1f} %; these are left to the host-side collision check, not to joint limits.",
        f"- Gripper pointing down, 50 mm above the table: reachable (in limits, collision-free) from "
        + (f"{reach_down.min():.3f} to {reach_down.max():.3f} m forward." if len(reach_down) else "nowhere (!)"),
        "",
        "![workspace](workspace_rz.png)",
        "",
        "![manipulability](manipulability_rz.png)",
        "",
        "Singularities (|det J| = 0), all checked in `tests/kinematics/test_singularities.py`: wrist (q5 = 0, J4 and",
        "J6 aligned), elbow (q3 = 0 or pi, arm stretched), shoulder (wrist centre on the cylinder of radius D around",
        "the J1 axis).",
        "",
    ]
    (out / "kinematics.md").write_text("\n".join(md))
    return stats


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=20000)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    a = ap.parse_args()
    print(build(a.out, a.samples))
