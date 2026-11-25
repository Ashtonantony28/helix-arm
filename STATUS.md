# Status

| Milestone | State | Verify |
|---|---|---|
| M0 Repo skeleton | merged (#1) | `pytest -q` |
| M1 Kinematics + spec freeze | in review: PoE/DH, closed-form IK (8 branches, exact on 20k poses) + LM fallback, collision envelope, derived limits, workspace maps, spec 1.0 (ADR 0003) | `pytest -q tests/kinematics` |
| M2 Dynamics + actuator sizing | next | `pytest -q tests/dynamics` |
