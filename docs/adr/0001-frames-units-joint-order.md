# 0001. Frames, units, joint order and sign conventions

- **Status:** Accepted
- **Date:** 2026-11-25

## Context

Every work stream (kinematics, dynamics, CAD, firmware, sim, the vla-arm bridge) has to agree on where
frames are, which way joints turn and what order they come in. Helix must also look like vla-arm to
vla-arm's tools (plan sec. 8): its `armctl` client, LeRobot environment and MuJoCo tasks use a base frame
with +z up and +x forward, zero pose stretched along +x, and positive pitch meaning down. The plan's sec. 8
calls the held joint "J5 (wrist yaw)". With a roll-pitch-roll spherical wrist the joint vla-arm lacks is
the forearm roll (J4), so that wording is corrected here.

## Decision

We will use:

- **Units:** SI everywhere in code and spec (m, rad, kg, s, V, A, N*m). Firmware may use fixed-point or
  milli-units internally but converts at its API boundary.
- **Base frame:** origin at the centre of the base plate underside, +z up, +x forward, +y left.
- **Zero pose:** arm stretched horizontally along +x at shoulder height, flange facing +x, gripper jaws
  closing along world y.
- **Joint order and names:** J1 `base_yaw` (+z), J2 `shoulder_pitch` (+y), J3 `elbow_pitch` (+y),
  J4 `forearm_roll` (+x), J5 `wrist_pitch` (+y), J6 `wrist_roll` (+x), then the `gripper`.
- **Joint model:** each joint rotates about its `axis` in the parent link frame; `origin` is the joint's
  position relative to the previous joint frame (product-of-exponentials chain, `helix.spec.fk`). Axes are
  unit +x, +y or +z only.
- **Signs:** right-hand rule. Positive pitch moves +x toward -z (DOWN), positive yaw moves +x toward +y,
  positive roll moves +y toward +z. Same as vla-arm.
- **Spherical wrist:** J4, J5 and J6 axes meet at the wrist centre, which is the J5 origin. The spec loader
  rejects any geometry that breaks this, because the M1 analytic IK depends on it.
- **vla-arm bridge:** protocol v1 clients see vla-arm's six channels `base_yaw, shoulder_pitch,
  elbow_pitch, wrist_pitch, wrist_roll, gripper`, mapped to J1, J2, J3, J5, J6 and the gripper; J4
  `forearm_roll` is held at `vla_arm_bridge.held` (0 rad).

## Consequences

vla-arm datasets and IK stay valid on Helix through the bridge without any conversion. Code can assert the
conventions instead of documenting them, and `helix.spec.parse` does so. The DH parameters derived in M1 are
a derived view and are tested against `helix.spec.fk`, never the other way round. A different wrist
(e.g. an offset wrist) would need a new ADR and a new IK.
