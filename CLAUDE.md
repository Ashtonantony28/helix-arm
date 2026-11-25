# CLAUDE.md: conventions for Helix Arm

This file is the contract for anyone (human or agent) changing this repository.

## Prime directives

1. **Every change ships with a Verify command.** A PR description names the exact command that proves it
   works (`pytest -q ...`, `ctest ...`, `python -m sim.sil ...`). If something cannot be verified yet, say so
   and name the milestone that will make it verifiable.
2. **Analytic before numeric.** Every physics, kinematics or actuator model has an analytic or published
   reference test before anything depends on it. Tolerances are stated in the test and never loosened
   silently.
3. **`spec/arm.yaml` is the single source of truth** for geometry, limits, ratings and module sizes. No
   magic numbers in kinematics, dynamics, CAD, firmware config or sim code: read the spec (`helix.spec`).
4. **Nothing is measured.** This is a simulation-only project (ADR 0002). Every motor, reducer, friction,
   thermal or datasheet number cites its source, and reports keep "computed" separate from "measured".
5. **Frames, units, joint order and signs follow ADR 0001.** Changing any of them is a new ADR.

## Architecture Decision Records

- Hard-to-reverse decisions get an ADR in `docs/adr/`, numbered sequentially, from `0000-template.md`.
- ADRs are immutable once Accepted; supersede with a new ADR and mark the old one `Superseded by NNNN`.
- Reference ADR numbers in code comments and PR descriptions.

## Code standards

- Python 3.11+, tests in `tests/` (and per-package `*/tests/`), run with `pytest -q` from the repo root.
- Golden fixtures (IK, dynamics, gear profiles) are committed and regenerated only as reviewed changes.
- Firmware (from M6): C11, warning-free with `-Wall -Wextra`, ASan + UBSan host build, HAL boundary with host
  stubs; the SIL plant and the MuJoCo actuator model share one implementation.
- CAD (from M3): parametric build123d from the spec; every assembly change passes the pairwise interference
  sweep at full joint travel before it is shown or merged.

## Branch / PR discipline

- Never commit directly to `main`. One milestone or one logical change per PR.
- PR description: what changed, **Verify command**, ADRs touched, milestone (docs/PLAN.md sec. 7).
- Red CI never merges. Commit messages explain *why*.
