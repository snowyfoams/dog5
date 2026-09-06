# Chapter 3 — Kinematics: model, conventions, calibration

**Status: PASSED.** The independent NumPy forward kinematics and Jacobian agree
with MuJoCo to **1.665e-16 m** over 1024 poses.

Source: [`src/dog5_description/`](../src/dog5_description) ·
calibration tools in [`src/calibration/`](../src/calibration)

---

## 1. There is no URDF

Worth saying plainly, because the phrase "URDF to MJCF" describes a pipeline
this project does not have. The real one is:

```
Fusion 360  ──►  robot_export.json  ──►  fusion2mjcf  ──►  dog5.xml
   CAD            links, masses,          (external)        MJCF
                  inertias, frame
```

`fusion2mjcf` **is not in this repository**, so `dog5.xml` cannot be
regenerated from `robot_export.json` here. There is no URDF at any stage, and
therefore no ROS or Pinocchio path. `dog5.xml` is the model of record; treat
`robot_export.json` as the provenance for the numbers inside it.

## 2. The coordinate statement

Everything downstream is this one sentence, quoted verbatim from
`robot_export.json`:

> z-up remap of Fusion Y-up world (x fwd, y left, z up); origin = trunk
> motor-plane centre; flat calibration pose = all joints 0; axes: abd +x,
> pitch/knee +z (at flat); PLA density 1.0 g/cm³

Unpacked:

- **FLU**: x forward, y **left**, z up — the same frame chapter 2's IMU output
  is converted into.
- **Origin** is the trunk motor-plane centre, not the CoM and not the
  geometric centre. The CoM sits **14.7 mm** away, which is a real 0.84 N·m
  pitch bias that chapter 4 has to feed forward.
- **Zero pose is the flat calibration pose** — all twelve joints at 0 puts the
  legs straight out sideways. This is not a pose the robot can stand in, and
  it is *singular*: at flat, the leg Jacobian is rank-1 and a Cartesian force
  command has no vertical authority at all. That single fact is why every
  stand in this repository is staged through a crouch rather than pushed
  straight up from zero.
- Masses come from a **uniform PLA density of 1.0 g/cm³**. Nothing was weighed.

## 3. Quaternion and rotation conventions

Fixed project-wide. Nothing in the shipped loop integrates a rotation — the
attitude is the AHRS's own fused output and the leg solver never leaves the
trunk frame — so the quaternion row below is convention, not code in use:

| item | convention |
|---|---|
| frames | I = inertial, gravity-aligned, origin at startup. B = body (IMU frame after extrinsics) |
| quaternion | `[x, y, z, w]`, representing I → B |
| product | **JPL**: `C(a ⊗ b) = C(a) · C(b)` |
| rotation matrix | `C(q)` maps I-coordinates to B-coordinates |
| attitude error | body-frame rotation vector δφ, **left**-multiplied: `q = ζ(δφ) ⊗ q̂`; small angle `C(ζ(δφ)) ≈ I − δφ×` |
| gravity | `g = [0, 0, −9.81]` in I, so a level robot at rest reads `f ≈ [0, 0, +9.81]` in B |
| units | m, m/s, m/s², rad, rad/s throughout — convert deg/s and g-units in the driver, never inside the filter |
| dimensions | 28 nominal states, **27** error states (the quaternion has 4 components but 3 DoF, so P is 27×27) |

Error-state index map: `δr` 0:3 · `δv` 3:6 · `δφ` 6:9 · `δp₁…δp₄` 9:21 ·
`δb_f` 21:24 · `δb_ω` 24:27.

JPL versus Hamilton is the classic silent bug in this area — both conventions
are self-consistent, and mixing them produces a filter that looks right and
diverges slowly. It is fixed here and stated in one place.

## 4. Reading `dog5.xml`

145 lines. The parts that carry a decision:

```xml
<compiler angle="radian" meshdir="meshes" autolimits="true"/>
<option timestep="0.002" integrator="implicitfast"/>
```

- **`armature="0.0085"`** on every joint — the rotor inertia reflected through
  the 10:1 gearbox (850 g·cm² × 10²). Without it the knees chatter in
  simulation. This is the single most important line in the file for anyone
  reproducing the sim.
- **Leg meshes are visual only** (`contype="0"`). Contact is done with explicit
  primitives: foot spheres of r = 0.02, thigh pads, and a trunk hull on
  `contype="2" conaffinity="1"`. Convex hulls of the real leg meshes produced
  phantom self-collisions.
- Trunk mass 2.6189 kg; whole-robot mass **5.8151 kg**, of which the legs are
  **3.196 kg — 55%**. Chapter 4 depends on that ratio.
- `<site name="imu" pos="0 0 0"/>` sits at the trunk origin, which is right in
  simulation and **38 mm too high on hardware** (chapter 2 §6).

## 5. Independent NumPy kinematics

`dog5_kinematics.py` implements forward kinematics, the translational foot
Jacobian and per-leg gravity torque in the trunk frame, in plain NumPy, and
**deliberately imports no MuJoCo**. Geometry is a controlled copy of
`dog5.xml`.

```python
LEGS = ("FL", "FR", "RL", "RR")
foot_position(leg, q)      -> (3,)   foot in trunk frame
foot_jacobian(leg, q)      -> (3,3)  d(foot)/d(q)
leg_gravity_torque(leg, q) -> (3,)
```

The Jacobian is built column-by-column from the geometric identity

```
J_i = axis_i × (p_foot − anchor_i)
```

The point of writing it twice is that the two implementations fail
differently. If a MuJoCo-only pipeline has the wrong sign somewhere, nothing
disagrees with it.

## 6. Cross-verification against MuJoCo

`check_dog5_kinematics.py` compares the NumPy implementation against
`mj_jacSite` **and** against finite differences, on 6 fixed plus 250 random
poses per leg. Reproduced on 2026-08-26 in this repository:

```
 leg   poses      max |FK-MJ|       max |J-MJ|       max |J-FD|
 FL      256        1.110e-16        1.388e-16        5.674e-10
 FR      256        1.665e-16        1.665e-16        6.542e-10
 RL      256        1.665e-16        1.388e-16        6.006e-10
 RR      256        1.110e-16        1.388e-16        4.978e-10
 limits             1.000e-10        1.000e-10        1.000e-08
```

FK and the Jacobian agree with MuJoCo **at machine epsilon** — these are not
"close", they are the same computation reaching the same floating-point
answer. The finite-difference column is looser only because differencing is.

```bash
python3 src/dog5_description/check_dog5_kinematics.py
```

> An earlier note in `SIM_TO_HARDWARE_PLAN.md` quotes 4024 poses and
> 6.499e-10. The reproducible figure from the committed seed (20260715) is
> 1024 poses and 6.542e-10. The numbers above are the ones this repository
> actually produces.

## 7. The hardware map — and the three maps that lost

`dog5_hardware_map.py` is the single source of truth. A frozen dataclass tuple
in canonical order `[FL, FR, RL, RR] × [abd, pitch, knee]`, with an import-time
`_validate()` asserting the IDs are exactly 1…12:

| joint | CAN | dir | | joint | CAN | dir |
|---|---:|---:|---|---|---:|---:|
| `hip_abd_FL` | 7 | +1 | | `hip_abd_RL` | 4 | −1 |
| `hip_pitch_FL` | 8 | +1 | | `hip_pitch_RL` | 5 | +1 |
| `knee_FL` | 9 | −1 | | `knee_RL` | 6 | −1 |
| `hip_abd_FR` | 10 | +1 | | `hip_abd_RR` | 1 | −1 |
| `hip_pitch_FR` | 11 | +1 | | `hip_pitch_RR` | 2 | +1 |
| `knee_FR` | 12 | −1 | | `knee_RR` | 3 | −1 |

The contract is deliberately minimal:

```
q = direction × motor_output_angle
```

**No software offset. No gearbox division.** Zeros live in each driver's flash,
written once with command `0x19`; the 10:1 reduction is already handled inside
`MotorBus` (chapter 1). Anything that adds its own offset or divides again is
wrong.

### The conflict this replaces

For several weeks in mid-2026 **four** joint maps were live at once:

| file | claim | verdict |
|---|---|---|
| `dog5_hardware_map.py` | the table above | **authoritative** |
| a superseded `coordinates.md` worksheet | FL 10/11/12, FR 7/8/9, RL 1/2/3, RR 4/5/6 | **left and right swapped**; the direction column was never filled in |
| a legacy recorded-pose runner | RL 1/2/3, RR 4/5/6, FR 7/8/9, FL 10/11/12 | a third arrangement |
| a `hil_map.json` | — | self-labelled `"PLACEHOLDER ONLY … NOT the real leg/joint wiring"` |

Only `dog5_hardware_map.py` validates itself at import, and it is the only one
that survives here — the other three are in the private history. The reason to
write this down at all is that **none of the four announced itself as wrong**;
each was found by a robot moving the wrong leg.

### And two encoder conventions

`calibration/calibrate12.py` computes `motor_output = encoder × 360/65535`
with **no gear divide**, while the robot code divides by `GEAR = 10`. Its own
docstring says to reconcile before mixing. They have not been unified; do not
mix them.

## 8. What `h` actually means

The controllers express a foot target in the trunk frame as
`p_des = [foot_x, foot_y, −h]`, so `h = −p_foot_trunk[z]`.

**`h` is the trunk-frame hip-to-foot distance along the trunk's own Z axis. It
is not the height of the robot above the floor.** Encoder angles alone cannot
tell you whether a foot is touching the ground, whether it is slipping, or
where gravity is. Lift the whole robot without moving a joint and `h` does not
change.

This is the single most important sentence in the chapter. Chapter 4 buys
back part of what it says the encoders cannot give you — attitude from the
IMU, and trunk velocity from the legs — and is explicit about the part it does
not: **contact is asserted by a clock, never measured**, and there are no force
sensors on this robot.

## 9. Calibration on the robot

| tool | what it does |
|---|---|
| `calibration/calibrate12.py` | `--observe` (live encoder table, all motors back-drivable), `--verify` (check the zero pose), `--set-zero` (write `0x19`; takes effect only after a power cycle) |
| `calibration/calibrate_leg.py` | the same for one leg's three IDs |
| `calibration/setzero_one.py` | one motor, including clearing a `0x80` latch over CAN first |
| `calibration/dog5_pose_monitor.py` | zero-torque live pose view; `C` captures a hand-arranged pose to JSON, `H` holds it under low-torque joint PD. This is how the recorded crouch that every runner starts from was produced |

Frequent use of `0x19` shortens driver life — it writes flash.

## Known limitations / what's next

- **No URDF, and `fusion2mjcf` is not in the repository**, so the model cannot
  be regenerated here and there is no ROS/Pinocchio interoperability.
- **Inertias are assumed, not weighed** — uniform PLA at 1.0 g/cm³ for every
  printed part. Every model-based result in chapter 4 inherits that error, and
  nobody has bounded it.
- **The IMU site in `dog5.xml` is wrong on hardware by 38 mm** and is corrected
  in the consumers instead of in the model.
- **Leg meshes are visual-only**, so simulated contact is spheres and pads, not
  geometry. Foot-edge and shin contacts that the real robot makes do not exist
  in sim.
- `dog5_hardware_map.py` is reached by most code only *transitively*, through
  `robot_base/stand_dog5_hw.py`. Only three modules import it directly.
- **The two encoder conventions (§7) were never reconciled**, only documented.
- Four conflicting joint maps coexisted for weeks and nothing but a convention
  stops that recurring — `_validate()` checks the IDs are 1…12, not that they
  are the *right* 1…12.
- **`view_dog5.py` and `make_gif.py` do their work at import time**, with no
  `if __name__ == "__main__"` guard. Importing the first opens a blocking
  viewer window; importing the second renders and *overwrites* the GIF in
  `docs/images/`. Nothing imports them, but any tooling that sweeps the tree
  importing modules must skip both. `selftest/test_layout.py` knows they are
  scripts rather than dead files because they have top-level statements, not
  because of a guard.
- `imu_frame_test.py` imports `termios` and so is POSIX-only; it will not run
  on Windows even with the vendor SDK present.
