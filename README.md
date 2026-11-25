# Helix Arm

A desktop 6-axis robot arm (6 joints + gripper, 1.0 kg at 600 mm reach) designed from the actuator up and
proven entirely in simulation: own BLDC + cycloidal/planetary joint modules, joint-driver and hub PCBs,
FOC firmware, rigid-body dynamics and a firmware-in-the-loop MuJoCo twin. Its hub speaks the
[vla-arm](https://github.com/Ashtonantony28/vla-arm) protocol so vla-arm's policies can run on it.

The plan is in [docs/PLAN.md](docs/PLAN.md). Nothing has been built or measured on hardware.
