"""README demo: the M1 kinematics driving a pick-and-place path.

    python scripts/ik_demo.py            # -> docs/media/ik-demo.{mp4,gif,png}  (needs ffmpeg)

A tool-down TCP path (minimum-jerk Cartesian segments with a wrist yaw turn) is solved frame by frame
with the closed-form IK (``ik.nearest``, in-limit branch closest to the previous frame). Every frame is
checked before it is drawn:
  * forward kinematics of the IK answer reproduces the target pose (< 1e-9 m / rad),
  * every joint is inside its spec limit,
  * the M1 capsule envelope is collision-free with the spec margin (self and table).
The arm is drawn as that capsule envelope (spec/arm.yaml `collision`), not as CAD (M3). Kinematics
only: there is no dynamics or control in this clip (M2+), and nothing here is hardware.
"""
from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from helix.kinematics import collision as C
from helix.kinematics import ik, poe
from helix.kinematics.workspace import manipulability, tool_down
from helix.spec import fk, load_spec

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "media"
FPS = 25
BG, PANEL, GRID, TXT, DIM = "#0d1117", "#161b22", "#30363d", "#e6edf3", "#8b949e"
ACCENT, OK, WARN = "#f0883e", "#3fb950", "#f85149"
LINK_COL = {"base": "#6e7681", "base_yaw": "#8b949e", "shoulder_pitch": "#58a6ff", "elbow_pitch": "#58a6ff",
            "wrist_pitch": "#d2a8ff", "wrist_roll": ACCENT}
CUBE = 0.04            # carried block edge, m (illustration)
PICK = np.array([0.34, -0.20, 0.05])      # TCP at the grasp: block centre is 30 mm lower, resting on the table
PLACE = np.array([0.20, 0.30, 0.05])
LIFT = 0.17


def _Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def min_jerk(n):
    s = np.linspace(0, 1, n)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def path():
    """(position, yaw, carrying, phase label) per frame."""
    home = np.array([0.30, 0.0, 0.30])
    segs = [  # (from, to, yaw0, yaw1, seconds, carrying, label)
        (home, PICK + [0, 0, LIFT], 0.0, 0.0, 1.6, False, "approach"),
        (PICK + [0, 0, LIFT], PICK, 0.0, 0.0, 1.0, False, "descend"),
        (PICK, PICK, 0.0, 0.0, 0.4, False, "grasp"),
        (PICK, PICK + [0, 0, LIFT], 0.0, 0.0, 1.0, True, "lift"),
        (PICK + [0, 0, LIFT], PLACE + [0, 0, LIFT], 0.0, np.pi / 2, 2.2, True, "transfer + 90° wrist turn"),
        (PLACE + [0, 0, LIFT], PLACE, np.pi / 2, np.pi / 2, 1.0, True, "place"),
        (PLACE, PLACE, np.pi / 2, np.pi / 2, 0.4, False, "release"),
        (PLACE, home, np.pi / 2, 0.0, 1.8, False, "return"),
    ]
    out = []
    for a, b, y0, y1, sec, carry, lab in segs:
        for s in min_jerk(int(sec * FPS)):
            out.append((a + (b - a) * s, y0 + (y1 - y0) * s, carry, lab))
    return out


def target(p, yaw):
    T = tool_down(p)
    T[:3, :3] = _Rz(yaw) @ T[:3, :3]
    return T


def solve_all(spec):
    g = ik.Geometry.from_spec(spec)
    env = C.from_spec(spec)
    model = poe.from_spec(spec)
    q = ik.nearest(g, target(np.array([0.30, 0.0, 0.30]), 0.0), np.zeros(6))
    frames = []
    for p, yaw, carry, lab in path():
        T = target(p, yaw)
        q = ik.nearest(g, T, q)
        err = np.abs(ik.pose_error(T, fk(spec, q)["tcp"])).max()
        assert err < 1e-9, f"FK(IK) mismatch {err:.2e}"
        assert np.all(q >= spec.lower - 1e-12) and np.all(q <= spec.upper + 1e-12), "joint limit"
        rep = C.check(env, spec, q)
        assert not rep.collides, f"collision at {p}: {rep}"
        frames.append({"q": q.copy(), "p": p, "carry": carry, "lab": lab, "err": err, "self": rep.self_clearance,
                       "table": rep.table_clearance, "manip": manipulability(model, q)})
    return frames, env


LIGHT = np.array([0.4, -0.5, 0.75]) / np.linalg.norm([0.4, -0.5, 0.75])


def _shade(col, n):
    rgb = np.array(matplotlib.colors.to_rgb(col))
    k = 0.38 + 0.62 * max(float(n @ LIGHT), 0.0)
    return tuple(np.clip(rgb * k, 0, 1))


def tube_faces(a, b, r, col, n=20, m=8):
    """Quads of a capsule (cylinder + hemispherical caps) with flat Lambert shading."""
    v = b - a
    L = np.linalg.norm(v)
    u = v / L if L > 1e-9 else np.array([0, 0, 1.0])
    k = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = np.cross(u, k); e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)
    th = np.linspace(0, 2 * np.pi, n + 1)
    # profile: (axial offset from a, radial scale) from the bottom pole to the top pole
    ph = np.linspace(-np.pi / 2, np.pi / 2, 2 * m + 1)
    prof = [(np.sin(p) * r + (L if p > 0 else 0.0), np.cos(p) * r) for p in ph[:m + 1]]
    prof += [(np.sin(p) * r + L, np.cos(p) * r) for p in ph[m:]]
    rings = [[a + u * z + rad * (np.cos(t) * e1 + np.sin(t) * e2) for t in th] for z, rad in prof]
    faces, cols = [], []
    for i in range(len(rings) - 1):
        for j in range(n):
            q = np.array([rings[i][j], rings[i][j + 1], rings[i + 1][j + 1], rings[i + 1][j]])
            c = q.mean(0)
            mid = a + u * np.clip((c - a) @ u, 0, L)
            nrm = c - mid
            nn = np.linalg.norm(nrm)
            if nn < 1e-12:
                continue
            faces.append(q); cols.append(_shade(col, nrm / nn))
    return faces, cols


def cube_faces(c, yaw, col):
    h = CUBE / 2
    R = _Rz(yaw)
    V = np.array([[x, y, z] for x in (-h, h) for y in (-h, h) for z in (-h, h)]) @ R.T + c
    idx = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    normals = [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1)]
    return [V[f] for f in idx], [_shade(col, R @ np.array(nv, float)) for nv in normals]


def render(spec, frames, env, d, only=None):
    names = [j.name for j in spec.joints]
    lo, hi = spec.lower, spec.upper
    Q = np.array([f["q"] for f in frames])
    t = np.arange(len(frames)) / FPS
    tcp = np.array([f["p"] for f in frames])
    manip = np.array([f["manip"] for f in frames])
    # block position: sits at PICK until grasped, rides under the TCP while carried, then stays at PLACE
    yaw_of = [np.arctan2(fk(spec, f["q"])["tcp"][1, 1], fk(spec, f["q"])["tcp"][0, 1]) - np.pi / 2 for f in frames]
    picked = next(i for i, f in enumerate(frames) if f["carry"])
    placed = max(i for i, f in enumerate(frames) if f["carry"])
    for i, f in enumerate(frames):
        if only is not None and i not in only:
            continue
        fig = plt.figure(figsize=(12.8, 7.2), dpi=100, facecolor=BG)
        ax = fig.add_axes([-0.02, 0.0, 0.62, 0.94], projection="3d", facecolor=BG)
        ax.set_proj_type("persp", focal_length=0.6)
        ax.view_init(elev=22, azim=-38 + 14 * np.sin(2 * np.pi * i / len(frames)))
        for a in (ax.xaxis, ax.yaxis, ax.zaxis):
            a.set_pane_color((0, 0, 0, 0)); a.line.set_color((0, 0, 0, 0))
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        ax.set_xlim(-0.12, 0.48); ax.set_ylim(-0.30, 0.38); ax.set_zlim(0.0, 0.44)
        ax.set_box_aspect((0.60, 0.68, 0.44), zoom=1.12)
        gx = np.linspace(-0.12, 0.48, 13); gy = np.linspace(-0.30, 0.38, 18)
        for x in gx:
            ax.plot([x, x], [gy[0], gy[-1]], [0, 0], color=GRID, lw=0.6)
        for y in gy:
            ax.plot([gx[0], gx[-1]], [y, y], [0, 0], color=GRID, lw=0.6)
        for c, col in ((PICK, OK), (PLACE, ACCENT)):
            th = np.linspace(0, 2 * np.pi, 40)
            ax.plot(c[0] + 0.035 * np.cos(th), c[1] + 0.035 * np.sin(th), 0 * th + 0.001, color=col, lw=1.4)
        ax.plot(*tcp[: i + 1].T, color=ACCENT, lw=1.3, alpha=0.8)
        faces, cols = [], []
        for k, (a, b) in zip(env.capsules, C.world_segments(env, spec, f["q"])):
            fa, co = tube_faces(a, b, k.r, LINK_COL.get(k.link, DIM)); faces += fa; cols += co
        if i < picked:
            fa, co = cube_faces(np.r_[PICK[:2], CUBE / 2], 0.0, OK)
        elif i <= placed:
            fa, co = cube_faces(f["p"] - [0, 0, 0.05 - CUBE / 2], yaw_of[i], OK)
        else:
            fa, co = cube_faces(np.r_[PLACE[:2], CUBE / 2], yaw_of[placed], OK)
        faces += fa; cols += co
        ax.add_collection3d(Poly3DCollection(faces, facecolors=cols, edgecolors=cols, linewidths=0.3))

        fig.text(0.02, 0.955, "Helix Arm · closed-form IK on a pick-and-place path", color=TXT, fontsize=17, weight="bold")
        fig.text(0.02, 0.918, f"phase: {f['lab']}", color=ACCENT, fontsize=12)
        fig.text(0.02, 0.03, "SIMULATION · kinematics only · arm drawn as the M1 collision capsules (CAD is M3)",
                 color=DIM, fontsize=10)

        # joint positions against their spec limits
        axq = fig.add_axes([0.60, 0.50, 0.37, 0.40], facecolor=PANEL)
        for j in range(6):
            axq.barh(j, hi[j] - lo[j], left=lo[j], color=GRID, height=0.55)
            axq.plot([Q[: i + 1, j].min(), Q[: i + 1, j].max()], [j, j], color="#58a6ff", lw=5, alpha=0.45,
                     solid_capstyle="butt")
            axq.plot(f["q"][j], j, "o", color=ACCENT, ms=8)
        axq.set_yticks(range(6), [n.replace("_", " ") for n in names], color=TXT, fontsize=10)
        axq.invert_yaxis()
        axq.set_xlim(-np.pi - 0.2, np.pi + 0.2)
        axq.set_xticks([-np.pi, -np.pi / 2, 0, np.pi / 2, np.pi], ["-π", "-π/2", "0", "π/2", "π"], color=DIM)
        axq.set_title("joint angle within spec limits (blue = range used so far)", color=TXT, fontsize=11, loc="left")
        for s in axq.spines.values():
            s.set_color(GRID)
        axq.tick_params(colors=DIM)

        axc = fig.add_axes([0.60, 0.13, 0.37, 0.26], facecolor=PANEL)
        axc.plot(t, manip * 1e3, color=GRID, lw=1)
        axc.plot(t[: i + 1], manip[: i + 1] * 1e3, color=OK, lw=1.8)
        axc.plot(t[i], manip[i] * 1e3, "o", color=ACCENT, ms=6)
        axc.set_xlim(0, t[-1]); axc.set_ylim(0, manip.max() * 1e3 * 1.15)
        axc.set_xlabel("time (s)", color=DIM); axc.set_ylabel("|det J| × 1e3", color=DIM)
        axc.set_title("manipulability (0 = singular): the path stays clear of singularities", color=TXT,
                      fontsize=11, loc="left")
        for s in axc.spines.values():
            s.set_color(GRID)
        axc.tick_params(colors=DIM)
        fig.text(0.60, 0.02, f"each frame checked: FK(IK) err {f['err']:.0e}, in limits,\n"
                 f"capsules collision-free with the {env.margin * 1000:.0f} mm spec margin", color=DIM, fontsize=10, family="monospace")
        fig.savefig(d / f"f{i:04d}.png", facecolor=BG)
        plt.close(fig)


def main():
    spec = load_spec()
    frames, env = solve_all(spec)
    worst = max(f["err"] for f in frames)
    print(f"{len(frames)} frames: max FK(IK) error {worst:.1e}, min clearance "
          f"{min(min(f['self'], f['table']) for f in frames) * 1000:.1f} mm, all in limits, no collisions")
    OUT.mkdir(parents=True, exist_ok=True)
    d = Path(tempfile.mkdtemp())
    try:
        render(spec, frames, env, d)
        shutil.copy(d / f"f{len(frames) // 2 - 6:04d}.png", OUT / "ik-demo.png")
        ff = ["ffmpeg", "-y", "-loglevel", "error"]
        subprocess.run(ff + ["-framerate", str(FPS), "-i", str(d / "f%04d.png"), "-c:v", "libx264",
                             "-pix_fmt", "yuv420p", "-crf", "23", str(OUT / "ik-demo.mp4")], check=True)
        subprocess.run(ff + ["-i", str(OUT / "ik-demo.mp4"), "-vf",
                             ("fps=12,scale=900:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];"
                              "[b][p]paletteuse=dither=none:diff_mode=rectangle"), str(OUT / "ik-demo.gif")], check=True)
    finally:
        shutil.rmtree(d)
    for n in ("ik-demo.mp4", "ik-demo.gif", "ik-demo.png"):
        print(n, f"{(OUT / n).stat().st_size / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
