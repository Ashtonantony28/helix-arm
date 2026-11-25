# Helix Arm: a 6-axis robot arm with custom actuators, designed and proven entirely in simulation

**Status:** Approved 2026-11-19 with the section 11 recommendations (see docs/adr/0002). M0 in progress.
**Drafted:** 2026-11-19
**Working name:** Helix Arm. Suggested repo `Ashtonantony28/helix-arm` (you create it; threads can't create repos from the cloud container).
**One line:** design a desktop 6-axis arm from the actuator up (your own BLDC + cycloidal joint modules, your own motor-driver and hub PCBs, your own FOC firmware), model its kinematics and dynamics, close every loop in a firmware-in-the-loop simulation, and hand vla-arm a much more capable arm to run its policies on.

---

## 1. How this differs from vla-arm

vla-arm is an ML project with a hardware platform attached. Its arm is 5 joints + gripper on off-the-shelf Feetech bus servos, with your own STM32 controller board in between; custom FOC joints are listed there only as a v2 stretch. Its effort went into data, policies and edge inference.

Helix is the opposite: **the arm is the product**. It takes that v2 stretch and makes it the whole project, at a higher spec.

| | vla-arm (exists) | Helix Arm (this plan) |
|---|---|---|
| Joints | 5 + gripper | **6 + gripper** (spherical wrist, full 6-DOF pose) |
| Actuators | Bus servos (bought) | **Own joint modules**: BLDC + designed cycloidal / planetary reducer + dual encoders |
| Payload / reach | 0.20 kg / ~45 cm | 1.0 kg / 60 cm (target, set by D2) |
| Electronics | One controller board | **Joint driver board** (×7) + **hub board** + power/e-stop board, in Zener → KiCad 10 |
| Firmware | Position setpoints to servos at 1 kHz | **FOC current loop at 20–40 kHz**, velocity/position cascade, cogging and friction compensation, CAN-FD |
| Control | Joint position | Gravity comp, computed torque, **task-space impedance**, collision detection via momentum observer |
| Dynamics | Torque budget spreadsheet | **Full rigid-body dynamics** (Pinocchio), parameter identification, actuator thermal model |
| Sim | MuJoCo twin for policy training | MuJoCo twin **plus firmware-in-the-loop**: the real firmware runs against a motor + reducer plant model |
| ML | The point of the project | Not in scope; Helix exposes a vla-arm-compatible interface instead |

**The link between them:** Helix ships an adapter so vla-arm's `armctl` client, its LeRobot environment and its MuJoCo tasks can drive Helix unchanged (§8). vla-arm then gets a stronger, more precise, torque-sensing arm, and its policies get re-scored on it in sim.

## 2. Goals

### Must reach (v1, all in the cloud)
1. **Mechanical design** in build123d (same flow as drone-design): base, 6 links, spherical wrist, parallel gripper, 3 sizes of joint module. Fully parametric from `spec/arm.yaml`; STEP/STL exports; mass properties straight from CAD into the dynamics model.
2. **Actuator design:** BLDC motor selection (catalogue motors, modelled from datasheets), designed cycloidal reducers for joints 1–3 and planetary reducers for 4–6, with tooth-profile generation, contact-stress and backlash/stiffness estimates, and FEA on housings and links.
3. **Kinematics:** analytic IK for the spherical-wrist geometry (all 8 branches), singularity handling, workspace and manipulability maps, collision model.
4. **Dynamics:** RNEA/CRBA via Pinocchio, cross-checked against MuJoCo and a from-scratch Newton–Euler implementation; excitation trajectories and least-squares parameter identification on simulated data.
5. **Electronics:** joint driver PCB (STM32G431, 3-phase gate driver + MOSFETs, inline phase-current sensing, motor-side + output-side magnetic encoders, CAN-FD, temperature), hub PCB (STM32H7, USB ↔ CAN-FD, e-stop logic, vla-arm protocol bridge), power board (48 V or 24 V bus, pre-charge, brake resistor / regen clamp, hardware e-stop). Schematics in Zener, layout and DRC in KiCad 10 (Docker), fab outputs and costed BOM.
6. **Firmware:** FOC with SVPWM, current/velocity/position cascade, encoder calibration and offset search, cogging-torque map, friction feed-forward, thermal derating, fault handling, CAN-FD protocol. Builds for Cortex-M4/M7 and for the host (SIL). Runs under Renode where peripheral models exist.
7. **Control stack (host side, Python + C++ core):** gravity compensation, computed-torque tracking, task-space impedance control, time-optimal trajectory parameterisation (TOPP-RA), momentum-observer collision detection with reaction.
8. **Simulation:** (a) MuJoCo model generated from CAD; (b) actuator plant model (motor electrical dynamics, reducer friction, backlash, compliance, thermal); (c) **full-stack firmware-in-the-loop**: compiled joint firmware ×7 → simulated CAN-FD bus → hub firmware → host controller → MuJoCo, deterministic and seeded.
9. **Validation report** with numbers: tracking error, repeatability under modelled backlash and encoder noise, collision-detection latency and force, thermal margin on a duty cycle, per-joint torque/speed margin, cost and mass.
10. **vla-arm bridge:** vla-arm's `armctl` and MuJoCo tasks run on Helix in sim, with the scripted experts at 100% and the existing policies re-scored.

### Stretch (v2)
- Joint-torque sensing without a torque sensor (current + model-based), used for hand-guiding / lead-through teaching in sim.
- Strain-wave (harmonic) reducer variant for the wrist, compared against planetary on backlash and efficiency.
- Motor electromagnetic FEA (2D) to check the catalogue-motor model, or to design a custom stator (see D5).
- ROS 2 `ros2_control` hardware interface + MoveIt 2 config (running against the sim).
- Port the host control loop to run on Vortex-V (RISC-V) in co-sim, the same way Talon does for its flight software.

### Out of scope
Physically building or ordering anything (see D1), ML policy training (that is vla-arm), safety certification, mobile base, bimanual.

## 3. Top-level specification (first estimates, to be fixed in M1)

| Item | Target | Notes |
|---|---|---|
| Axes | 6 + gripper | J1 base yaw, J2 shoulder, J3 elbow, J4–J6 spherical wrist |
| Reach | 600 mm | To wrist centre ≈ 520 mm |
| Payload | 1.0 kg at full reach | Rated continuous |
| Arm mass | ≤ 6 kg | Without base plate |
| TCP speed | 1.0 m/s | Peak |
| Repeatability | ±0.3 mm (sim, with modelled backlash + encoder noise) | Output-side encoders make this feasible |
| Bus voltage | 24 V or 48 V (D3) | |
| Control rates | FOC 20–40 kHz, joint position 1–2 kHz, host 1 kHz, CAN-FD 1 kHz | |

**Rough actuator sizing** (to be redone from the trajectory suite in M2): J2 static holding ≈ (1.0 kg × 0.6 m + ~2.5 kg × 0.3 m) × 9.81 ≈ 13 N·m; with dynamics and margin, peak ≈ 35 N·m. That points to an 80-class gimbal/QDD BLDC (≈1.5–2 N·m peak) behind a ~1:20 cycloidal. Wrist joints need ≈ 3–5 N·m peak: a 40-class BLDC behind a ~1:10 planetary. So three module sizes: **L** (J1–J2), **M** (J3), **S** (J4–J6, gripper).

## 4. Architecture

```
 host (cloud sim now; PC/Jetson later)
   control stack: IK/ID, TOPP-RA, impedance, collision observer   ← vla-arm armctl via adapter
          │ USB (vla-arm protocol v1 compatible + Helix v2 extensions)
 ┌────────▼──────────── Hub board (STM32H7) ─────────────┐
 │ 1 kHz cycle: fan out setpoints/torque FF, gather state │
 │ e-stop chain, watchdog, bus-voltage monitor            │
 └────────┬───────────────────────────────────────────────┘
          │ CAN-FD 5 Mbit/s, 1 kHz synchronous cycle
  ┌───────┼────────┬──────── ... ────────┐
  ▼       ▼        ▼                     ▼
 J1(L)   J2(L)    J3(M)   J4–J6(S)    gripper(S)
 each joint module = BLDC + reducer + motor encoder + output encoder
                   + joint driver board (STM32G431, FOC 20–40 kHz)
 power board: 24/48 V bus, pre-charge, brake resistor, hardware e-stop (STO-style gate disable)
```

In simulation every box above is real code except the physics: MuJoCo stands in for the links, the actuator plant model stands in for motors and reducers, and a deterministic CAN-FD bus model connects the compiled firmware images.

## 5. Work streams

### 5.1 Mechanical (build123d)
- `spec/arm.yaml` is the single source of truth: link lengths, module sizes, joint limits, materials, print/machining process.
- Parametric joint modules: motor pocket, cycloidal disc pair (180° phased, for balance), eccentric bearing, pin ring, output flange with crossed-roller or 4-point-contact bearing, encoder magnet mounts on motor and output shafts, cable pass-through (hollow shaft on J4–J6).
- Links as hollow sections sized by FEA (aluminium tube or printed PA-CF; D4), wrist as a compact 3-module spherical stack, gripper with a single S module driving a rack.
- Checks as pytest: link lengths match spec, interference sweep over the full joint space, wall thickness, bolt engagement, bearing ratings vs. computed loads, print-bed or stock-size fit.
- FEA: gmsh + CalculiX (both installable in the container) on links and housings under worst-case static + dynamic loads; deflection budget feeds the repeatability estimate.

### 5.2 Actuators
- **Cycloidal reducer design:** exact epitrochoid profile with pin-clearance and profile-shift modification, generated in code; tooth-contact force distribution (published Blanche/Yang or Gorla method), Hertzian contact stress, lost motion and torsional stiffness estimates, efficiency model. Profile exported to build123d.
- **Planetary reducer:** involute gears generated in code, AGMA-style bending/contact checks.
- **Motor model:** per-motor parameters (Kt, R, L, pole pairs, cogging amplitude, thermal resistance and time constant) from datasheets, kept with sources in `spec/motors/`. A dq-frame electrical model plus a two-node thermal model.
- **Sizing loop:** a suite of reference trajectories (pick-place at speed, full-reach payload hold, worst-case wrist flips) is run through inverse dynamics to give per-joint torque-speed point clouds and RMS torque; each module must cover them with stated margin (peak ≥ 1.3×, RMS thermal ≥ 1.2×). This is the loop that fixes module sizes and ratios in M2.

### 5.3 Kinematics and dynamics
- DH and product-of-exponentials models generated from the spec, checked against each other and against CAD joint frames.
- Analytic IK (Pieper decomposition, 8 branches) plus damped least-squares fallback; IK checked exact on ≥ 20,000 random poses, as vla-arm did.
- Pinocchio for RNEA/CRBA/ABA and derivatives; a from-scratch Newton–Euler in numpy as an independent reference; MuJoCo inverse dynamics as a third. All three agree to 1e-9 on random states.
- Identification: base-parameter regressor, optimised Fourier excitation trajectories (condition-number objective), weighted least squares with physical-consistency constraints (LMI via cvxpy), friction (Coulomb + viscous + Stribeck) per joint. Validated by recovering known parameters from noisy simulated data.

### 5.4 Electronics (Zener → KiCad 10)
Same toolchain drone-design proved: Diode `pcb`/Zener schematics, KiCad 10 in Docker for layout, DRC, and fab outputs.
- **Joint driver** (one design, two power stages for L vs. S modules): STM32G431, DRV8353-class gate driver or discrete half-bridges, 3-shunt or inline current sense with INA240-class amplifiers, MA732/AS5047P-class encoders (motor side on-board, output side on a flex/satellite board), NTC on motor and FETs, CAN-FD transceiver, SWD. Round board to fit behind the motor.
- **Hub:** STM32H7, USB-FS/HS, CAN-FD, e-stop input chain, status LEDs, bus-voltage and current monitoring.
- **Power board:** pre-charge, reverse-polarity, brake chopper sized from the regen energy of a full-speed stop, hardware e-stop that disables all gate drivers independent of firmware.
- Gates: ERC clean, DRC 0 errors / 0 unconnected, impedance targets for CAN-FD and USB, current-density and thermal checks on power traces, BOM with live-orderable part numbers (prices dated).

### 5.5 Firmware (C11, ctest + Renode)
- Joint firmware: FOC (Clarke/Park, PI current loops with anti-windup, SVPWM with dead-time compensation), field weakening off by default, velocity and position loops, feed-forward torque input from the host, encoder calibration (electrical offset, nonlinearity table), cogging map, dual-encoder backlash handling, thermal derating, faults (over-current, over-temp, encoder loss, bus under/over-voltage, CAN timeout → hold → torque off).
- Hub firmware: 1 kHz synchronous CAN-FD cycle, vla-arm protocol v1 bridge, Helix v2 messages (torque mode, impedance gains, state with currents/temperatures), e-stop and watchdog.
- Everything builds for the host as a library (SIL) and for Cortex-M (size and timing reports). Renode used where it models the peripherals; where it doesn't, a thin HAL with host stubs, and the register-level layer is marked unverified, as vla-arm's STATUS does honestly today.
- Determinism: fixed-point or bounded float paths, worst-case execution time estimated by cycle counting on the M4 build, current loop budget stated.

### 5.6 Control stack (host)
- Joint PD + gravity compensation, computed-torque tracking, task-space impedance / operational-space control, null-space posture task.
- TOPP-RA for time-optimal paths under velocity, acceleration and torque limits from the actuator models.
- Momentum-observer collision detection with thresholds tuned on simulated collisions; reaction = torque-mode compliance then safe stop.
- C++ core for the 1 kHz loop with Python bindings; Python reference implementation for tests.

### 5.7 Simulation
- MuJoCo model generated from CAD mass properties (no hand-entered inertias), with actuator plants as custom callbacks: motor dq model, reducer friction/backlash/compliance, encoder quantisation and noise, thermal state.
- Firmware-in-the-loop harness: N compiled joint firmware instances + hub firmware, a CAN-FD bus model with arbitration and latency, the host stack, MuJoCo; lock-stepped at the FOC rate for single-joint tests and at a decimated rate (averaged-model) for full-arm runs, with a documented equivalence check between the two.
- Scenario suite: step and chirp responses per joint, payload sweeps, duty-cycle thermal runs, collisions at set speeds, e-stop from full speed (stopping distance and bus over-voltage), encoder loss, CAN dropout.

## 6. Verification approach

Following the conventions of your other repos (CLAUDE.md with "every change ships with a Verify command", ADRs, spec as single source of truth, analytic before numeric):
- **Analytic references first:** two-link dynamics closed forms, cycloidal geometry identities, DC-motor step response, published identification benchmarks.
- **Three-way dynamics cross-check** (Pinocchio, own Newton–Euler, MuJoCo).
- **Firmware:** ctest unit tests, ASan + UBSan, SIL against the plant model, Renode where possible; the SIL plant and the MuJoCo actuator model share one implementation so they can't drift.
- **Golden fixtures** for IK, dynamics and cycloidal profiles, committed and regenerated only as reviewed changes.
- **CI** on GitHub Actions: Python suites, firmware host build + tests, a short SIL scenario, CAD checks; KiCad DRC as a job using the Docker image.

## 7. Milestones

Each milestone ends with a merged PR, a Verify command in its description, and a status line in `/mnt/project-files/project-plans/robotic-arm/STATUS.md`.

| # | Milestone | Done when | Verify |
|---|---|---|---|
| M0 | Repo skeleton | CLAUDE.md, ADR 0001 (frames, units, joint order, sign conventions), `spec/arm.yaml`, CI green | `pytest -q` |
| M1 | Kinematics + spec freeze | DH/PoE models, analytic IK 8 branches, workspace + manipulability maps, collision primitives | `pytest -q tests/kinematics` (IK exact on 20k poses) |
| M2 | Dynamics + actuator sizing | 3-way dynamics agreement, trajectory suite, torque-speed clouds, module sizes and ratios fixed by ADR | `pytest -q tests/dynamics tests/sizing` |
| M3 | Joint module CAD | L/M/S modules in build123d, cycloidal + planetary profiles generated, stress/backlash/stiffness estimates, FEA on housings | `pytest -q hardware/tests` |
| M4 | Full arm CAD | Links, wrist, gripper, base; mass properties exported; interference sweep clean; FEA on links; renders | `pytest -q hardware/tests` + `python -m hardware.export` |
| M5 | Actuator plant + MuJoCo twin | Motor/reducer/thermal model, MuJoCo model from CAD, step/chirp responses match analytic references | `pytest -q sim/tests` |
| M6 | Joint firmware (SIL) | FOC + cascade + calibration + faults, host build, single-joint SIL hitting current-loop bandwidth target | `ctest --test-dir firmware/build` + `python -m sim.sil --joint J2 --scenario step` |
| M7 | Electronics: schematics | Joint driver, hub, power boards in Zener; ERC clean; BOM costed | `pcb build ecad/ && pytest -q ecad/tests` |
| M8 | Electronics: layout | All three boards laid out, DRC 0/0, fab outputs | `ecad/layout/run.sh && pytest -q ecad/tests` |
| M9 | Hub firmware + full-arm FIL | 7 joint images + hub + CAN-FD model + host stack + MuJoCo in one deterministic run | `python -m sim.fil --scenario pick_place --seed 0` |
| M10 | Control stack | Gravity comp, computed torque, impedance, TOPP-RA, collision observer; scenario suite passing | `pytest -q control/tests` + `python -m sim.fil --suite all` |
| M11 | Identification | Parameters recovered from noisy sim data within stated tolerance; identified model improves tracking | `pytest -q tests/ident` |
| M12 | vla-arm bridge | vla-arm `armctl` + LeRobot env drive Helix in sim; scripted experts 100%; existing ACT policy re-scored | `pytest -q tests/vla_bridge` |
| M13 | Validation report | Report with all v1 numbers, known gaps, build-readiness checklist | `python -m report.build` |

M1–M2 gate everything else (they fix the module sizes). After M2, mechanical (M3–M4), electronics (M7–M8) and firmware (M6) can run in parallel threads.

## 8. vla-arm integration in detail
- **Protocol:** the hub speaks vla-arm's protocol v1 framing unchanged (`A5 5A`, CRC-16/CCITT-FALSE, SET_TARGETS/STATE/ACK). Helix extensions use new TYPE codes and a VER bump that v1 clients never send, so old clients keep working.
- **Joint count:** vla-arm assumes N = 6 in the order `base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll, gripper`. Helix has 7. The v1 bridge maps those 6 onto Helix and holds J5 (wrist yaw) at a configured angle, so vla-arm's IK and datasets stay valid. A v2 client gets all 7.
- **Sim:** a Helix MuJoCo scene that registers in vla-arm's task registry, so the pick-place, stack and sort tasks and the scripted experts run on Helix.
- **Ownership:** Helix code lives in its own repo. Any change inside vla-arm (e.g. a `--robot helix` flag) is a separate small PR to vla-arm.

## 9. Tooling (all cloud, all previously proven in this project except where noted)
| Need | Tool | Status |
|---|---|---|
| CAD | build123d | Proven in drone-design |
| FEA | gmsh + CalculiX | pip/conda; not yet used in this project (drone-design used its own FEA path) |
| Dynamics | Pinocchio, numpy | pip/conda-forge |
| Optimisation | cvxpy, TOPP-RA | pip |
| Sim | MuJoCo | Proven in vla-arm (needs `libegl1 libgl1-mesa-dri`) |
| E-CAD | Diode `pcb`/Zener + KiCad 10 Docker (`kicad/kicad:10.0.8`) | Proven in drone-design |
| Firmware | arm-none-eabi-gcc, CMake/ctest, Renode | gcc proven in vla-arm; Renode new |
| Heavy design work | Fable 5.2 subagents for PCB layout/routing and FOC/FIL debugging, per your model policy | |

## 10. Risks

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| **No hardware means no ground truth.** Friction, backlash, cogging and thermal numbers are models, not measurements | Certain | High | Every model cites its source and states its assumption; report separates "computed" from "measured"; build-readiness checklist lists what must be measured first if built (D1) |
| Cycloidal reducer accuracy depends on machining tolerances the sim can't know | High | Medium | Tolerance sweep in the backlash/stiffness model; design for printed + machined variants; output-side encoder closes the position loop |
| Full-arm FIL at FOC rate is too slow in the cloud | Medium | Medium | Averaged-model mode for full-arm runs, FOC-rate lockstep only for single joints, with an equivalence test between them |
| Renode lacks STM32G4/H7 peripheral models (timers, ADC injection, FDCAN) | Medium | Medium | HAL boundary with host stubs; register layer marked unverified, same as vla-arm |
| Zener/KiCad toolchain friction (it took real effort in drone-design: Docker restarts, routing) | Medium | Medium | Reuse drone-design's working setup and maze-router scripts; Fable 5.2 for layout |
| Datasheet parameters for motors and gate drivers are wrong or missing | Medium | Medium | `spec/motors/` keeps sources; datasheet VERIFY items listed as in drone-design |
| Scope: 13 milestones across mech, EE, firmware, control | High | High | M1–M2 gate; parallel threads after M2; v1 list is fixed, everything else is v2 |
| Thermal limits on the shoulder under 1 kg payload | Medium | High | Sizing loop uses RMS torque and the thermal model; fallback is a counterbalance spring (as vla-arm needed) or lower payload rating |
| Container restarts lose long runs | Medium | Low | Bundles to project files and frequent pushes, as other threads do |

## 11. Decisions for you

| # | Decision | Options | Recommendation |
|---|---|---|---|
| D1 | Physical build | (a) Sim-only, fab-ready outputs; (b) plan an eventual build with a measured-parameter phase | **(a)** for this project, as you asked for cloud-only; outputs are fab-ready so (b) stays open |
| D2 | Size class | (a) 1.0 kg / 600 mm desktop cobot; (b) 0.5 kg / 450 mm, matching vla-arm's scale; (c) 3 kg / 800 mm | **(a)**: clearly a step up from vla-arm while staying desk-sized |
| D3 | Bus voltage | 24 V or 48 V | **48 V**: lower current for the same power, better for the L modules; 24 V if you want cheaper parts |
| D4 | Link material | (a) Printed PA-CF/PETG; (b) aluminium tube + printed joints; (c) machined aluminium | **(b)**: stiffness for repeatability, still cheap to make |
| D5 | Motors | (a) Catalogue BLDCs modelled from datasheets; (b) design custom stators with EM FEA | **(a)** for v1; (b) as a stretch |
| D6 | Wrist reducers | Planetary or strain-wave | **Planetary** in v1, strain-wave as a v2 comparison |
| D7 | Repo and visibility | New repo `helix-arm` (private) or a `helix/` folder inside vla-arm | **New private repo**: different discipline, keeps vla-arm focused |
| D8 | Name | "Helix Arm" or your choice | Your call; the name only affects the repo and docs |

## 12. Rough effort

M0–M2: 1 thread. M3–M4, M6, M7–M8: three parallel threads after M2. M9–M13: 1–2 threads. In wall-clock terms with your usual pacing, roughly the same order as Vortex-V's M0–M7.

## 13. First steps after approval
1. You create the empty repo `Ashtonantony28/helix-arm` (command: `gh repo create Ashtonantony28/helix-arm --private`).
2. M0 and M1 in one thread; ADR 0001 fixes frames and joint order compatible with vla-arm's.
3. M2 sizing loop, then fan out.
