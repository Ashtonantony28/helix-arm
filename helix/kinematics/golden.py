"""Golden IK fixture: 64 seeded random in-limit poses with all closed-form solutions.

    python -m helix.kinematics.golden      # rewrites tests/kinematics/fixtures/ik_golden.json

Regenerating it is a reviewed change (CLAUDE.md): a diff means the IK or the spec changed.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from helix.kinematics import ik
from helix.spec import fk, load_spec

PATH = Path(__file__).resolve().parents[2] / "tests" / "kinematics" / "fixtures" / "ik_golden.json"
N, SEED = 64, 20261125


def generate() -> dict:
    spec = load_spec()
    g = ik.Geometry.from_spec(spec)
    rng = np.random.default_rng(SEED)
    cases = []
    for _ in range(N):
        q = rng.uniform(spec.lower, spec.upper)
        T = fk(spec, q)["tcp"]
        sols = sorted((np.round(s, 12).tolist() for s in ik.solve(g, T)))
        cases.append({"q": np.round(q, 12).tolist(), "T": np.round(T, 12).tolist(), "solutions": sols})
    return {"spec_version": spec.raw["version"], "seed": SEED, "cases": cases}


if __name__ == "__main__":
    PATH.write_text(json.dumps(generate(), indent=1) + "\n")
    print(f"wrote {PATH}")
