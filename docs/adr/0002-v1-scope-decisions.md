# 0002. v1 scope decisions (plan sec. 11, D1-D8)

- **Status:** Accepted
- **Date:** 2026-11-25

## Context

The plan (docs/PLAN.md sec. 11) left eight decisions open with a recommendation for each. The user approved
the plans on 2026-11-19 ("read through all plans and they are approved"). On 2026-11-25 they created this
repository as **public** and asked for the plan to be pushed here and building to start.

## Decision

We will build v1 with the plan's recommendations, except D7:

| # | Decision | Choice |
|---|---|---|
| D1 | Physical build | Simulation only, with fab-ready outputs |
| D2 | Size class | 1.0 kg at 600 mm reach |
| D3 | Bus voltage | 48 V |
| D4 | Link material | Aluminium tube links + printed joint housings |
| D5 | Motors | Catalogue BLDCs modelled from datasheets |
| D6 | Wrist reducers | Planetary in v1 (strain-wave as a v2 comparison) |
| D7 | Repo | Own repository `Ashtonantony28/helix-arm`, **public** (the plan recommended private; the user created it public) |
| D8 | Name | Helix Arm |

## Consequences

Every number that would normally be measured is a model with a cited source (CLAUDE.md directive 4). The
48 V bus sets the gate-driver, MOSFET and connector classes for M7. Because the repo is public, nothing is
committed that should not be public: no keys and no vendor files under restrictive licences.
