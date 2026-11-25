# 0003. Elbow offset, collision envelope and the M1 spec freeze

- **Status:** Accepted
- **Date:** 2026-11-25

## Context

The plan's first geometry put the upper arm, forearm and wrist in one vertical plane. In M1 the
collision envelope showed that this cannot fold. Two 35 mm-radius tubes hinged at the elbow touch as
soon as the fold passes roughly 90°, so the forearm could never come back toward the base. Without that
fold the arm cannot work close to its own base or reach the table near itself.

There were three options:

- **Clevis/yoke elbow.** One link straddles the other. This needs wide forks around an M-size module and
  still limits the fold.
- **Smaller fold.** Accept a fold of about ±1.6 rad. This loses the inner workspace (the minimum
  wrist-centre distance would be about 0.37 m).
- **Lateral elbow offset.** Offset the forearm sideways, as UR-style arms do. The forearm then runs beside
  the upper arm. The Pieper closed form still applies, because a constant offset between the J1 axis and
  the arm plane is the classic shoulder-offset case.

The plan also says that M1 fixes the joint limits from a base and collision model and freezes the spec.

## Decision

We will:

- Offset the elbow by **80 mm in +y**. In `spec/arm.yaml`, `elbow_pitch.origin` = `[0.260, 0.080, 0]`.
  This is enough for two 35 mm tubes plus the 5 mm margin at any fold angle.
- Measure `ratings.reach_m` and `wrist_centre_reach_m` in the arm (x-z) plane, perpendicular to the
  shoulder axis. The offset moves the arm sideways; it does not lengthen it.
- Define a **collision envelope** in `spec/arm.yaml: collision`. It holds capsules per link, a 5 mm
  margin, a table plane, and the joint-touching pairs that are not checked. The envelope is a contract
  for M3: each link's CAD must fit inside its capsules, so collision results from M1 stay valid.
- Derive the **pitch limits** from the envelope (`helix.kinematics.limits`). Each limit is the first
  contact between the links on either side of the joint, with the other joints at zero, minus 0.02 rad,
  rounded down to 0.05 rad. The results are:
  - shoulder +0.75 (upper arm onto the J1 module; the plan had 1.20)
  - elbow ±2.90 (wrist onto the upper arm)
  - wrist pitch ±1.80 (gripper onto the forearm end)
- Leave the yaw and roll limits at the plan's values. They are set by cabling, not by collision.
- Keep contacts that depend on more than one joint out of the joint limits. Examples are the wrist into
  the base and anything into the table. These are left to a host-side collision check, as in vla-arm.
  In uniform joint samples they occur in 5.9 % of poses (self) and 24 % (table).
- **Freeze the spec at version 1.0.** From here, any change to geometry, limits or the envelope is a
  reviewed change. It must regenerate the golden IK fixture and `docs/kinematics/`.

## Consequences

- The forearm folds past the upper arm. The wrist centre can come within 63 mm of the shoulder axis,
  and the gripper can point straight down from 0.07 to 0.51 m in front of the base.
- The arm is no longer mirror-symmetric. Every wrist centre stays at least 80 mm from the J1 axis, and
  the third Jacobian zero is the wrist centre on that 80 mm cylinder. It used to be on the J1 axis.
- vla-arm compatibility is unaffected. The bridge is in joint space (ADR 0001), and vla-arm's own IK
  never runs on Helix geometry.
- M3 must:
  - put the J2 module on the -y side of the upper arm and the J3 module between the two links;
  - end the forearm tube 80 mm before the wrist centre (the J5 clevis);
  - keep every part inside its capsule, or change the envelope here first.
- M2 sizes the modules. If an L, M or S module comes out larger than its capsule radius, the envelope
  and the limits are re-derived.
