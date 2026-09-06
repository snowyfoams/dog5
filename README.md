# DOG5

A 5.8 kg quadruped built and brought up from scratch — twelve CAN servos, one
IMU, a MuJoCo model, and a torque-mode control stack that goes from "does the
bus answer" to a trot.

**This repository contains only code that has run on the robot.** Everything
in it was used, tested, and flown on hardware. A good deal more was written —
an error-state EKF, a cone-constrained force QP, a convex MPC — and none of
that is here, because none of it has ever been in the loop. [What is not
here](#what-is-not-here) says exactly what and why.

![DOG5 standing](docs/images/dog5_stand.gif)

---

## The demo, in one run

`trot_video/trot.MOV` is a single continuous run of `walk_demo.py`, and it
contains the whole stack:

| in the video | stage | what is running |
|---|---|---|
| the robot folds to a crouch | `CROUCH` | native `0xA4` **position** mode. No torque, no model |
| it stands up, slowly | `RISE` | **torque** mode. 8 s cubic ramp; the trunk is held up by forces this code computes |
| it holds, and resists a push | `HOLD` | the same loop, at a fixed height. Push it and the recovery *is* the feedback |
| it trots in place | `TROT` | the gait clock opens, diagonal pairs alternate |
| it walks out, and walks back | `TROT` + walk ledger | every landing point displaced 20 mm; `R` returns the same number of swings |

The video is tracked with **Git LFS** (136 MB). `git lfs install` before
cloning, or `git lfs pull` after, if you want it.

## Prove it works, with no robot

```bash
git clone <this repo> && cd dog5
pip install -r requirements.txt

python3 src/selftest/test_all.py
```

Thirteen suites, ~30 s, no CAN interface, no IMU, no data files. It runs the
repository's layout gates, the NumPy-vs-MuJoCo kinematics cross-check over
1024 poses, the statics model against MuJoCo inverse dynamics, and the offline
gates of every hardware runner that ships.

```bash
python3 src/dog5_description/view_dog5.py      # look at the robot in MuJoCo
```

## The stack, in one picture

```
                    twelve encoders            one DETA10 AHRS
                          │                          │
                          ▼                          ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  feedback_estimator.py           z from leg FK                  │
 │                                  v from the leg-odometry identity│
 │                                  roll/pitch/yaw straight from    │
 │                                  the AHRS, ω from the gyro       │
 └─────────────────────────────────────────────────────────────────┘
                          │  (z, v, C, ω)          no filter, no fused state
                          ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  1. dynamic_model.py     WRENCH        world axes,     83.3 Hz  │
 │     F = Kp(p_ref−p) + Kd(v_cmd−v) + mg ẑ                        │
 │     M = Kp(rpy_ref−rpy) + Kd(0−ω)                               │
 └─────────────────────────────────────────────────────────────────┘
                          │  w = (0, 0, F_z, M_x, M_y, M_z)
                          ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  2. force_totorque.py    ALLOCATION    world axes,     83.3 Hz  │
 │     A f = w by damped least squares, then clamp:                │
 │     unilateral → friction cone → rescale Σf_z                   │
 └─────────────────────────────────────────────────────────────────┘
                          │  four foot forces
                          ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  3. τ = −Jᵀ Cᵀ f + leg_gravity         JOINT           83.3 Hz  │
 │     …then the impedance floor kp(q_ref−q) − kd q̇        250 Hz  │
 │     …then TorqueGate: ramp, cap, 60 N·m/s slew                  │
 └─────────────────────────────────────────────────────────────────┘
                          │  twelve torques, 0xA1
                          ▼
                    CAN bus, 250 Hz per motor

 ╌╌ gait.py ╌╌ a pure function of t: which legs are in which branch ╌╌
```

Read it downwards. The trunk asks for a wrench, the feet split it, each motor
is told a torque. **Layer 3 contains no trunk feedback at all** — it is pure
transmission. The gait clock reads nothing.

[`docs/ch4_quasi_dynamic_trot.md`](docs/ch4_quasi_dynamic_trot.md) is this
picture at length, block by block, with the measured numbers.

## Running it on the robot

```bash
sudo bash src/bringup/t1_preflight.sh              # brings can0 up at 1 Mbit/s
python3 src/bringup/t2_ping_scan.py                # do all twelve answer?

cd src
sudo HOME=$HOME chrt -f 50 python3 dog5_trot_quasi_static_model/walk_demo.py
```

Keys: `ENTER` rise · `T` trot / back to HOLD · `R` walk back · `P` park (from
HOLD only) · `SPACE` limp · `X` stop.

**Parameters are edits, not flags.** Open
`src/dog5_trot_quasi_static_model/config.py`, change the constant, save. Every
run logs itself to `run_<date>_<time>.npz`, and the log embeds
`config.snapshot()` — every constant as imported, plus `config.py`'s own source
text — so the run says what flew without anyone writing it down. The only
tuning flag left is `--tau-max`.

The full procedure, in the order it must be done, is in
[`docs/runbooks/DEMO_RUNBOOK.md`](docs/runbooks/DEMO_RUNBOOK.md).

## Repository map

```
src/
  dog5_paths.py          the ONLY sys.path logic in the repo
  motor/                 the CAN motor library: motorbus, motor_library,
                         motor_gains, mac_can
  bringup/               the T1–T8 CAN bring-up ladder. t1 brings can0 up
  motor_demos/           single-motor scripts: read a state, follow a
                         trajectory, feel an impedance
  IMU_sensor/            DETA10 driver, frame test, noise logger, imu_calib.json
  dog5_description/      THE MODEL: dog5.xml, meshes, NumPy kinematics,
                         statics, the hardware map, the MuJoCo cross-check
  calibration/           zeroing, per-leg zeroing, and the pose capture that
                         produced the recorded crouch
  robot_base/            the shared CAN/joint hardware layer, and the
                         position-mode runner the crouch comes from
  torque_stand/          THE CONTROLLER: params, crouch/park, feedback,
                         wrench, allocation, and the torque stand
  dog5_trot_quasi_static_model/   the trot: config, gait clock, and the three
                         runners (package — deliberately off sys.path)
  selftest/              the gates no single module owns, and test_all.py
  tools/                 log conversion, forensics, and the --web telemetry
docs/                    four chapters, a summary, and the runbooks
data/                    run logs — gitignored, see data/README.md
trot_video/              the demo run (Git LFS)
```

`src/dog5_paths.py` puts every directory under `src/` on the path, so any
script anywhere can `import motorbus`. The trot package is deliberately kept
**off** it: it contains a `config.py`, and the resulting shadowing used to kill
the CAN layer with an unrelated `AttributeError` several imports later.
`src/selftest/test_layout.py` gates against that, against a stale import
anywhere in the tree, and against a file nothing reaches.

### Modules used by more than one chapter

Chapter identity lives in `docs/`, not in the directory names, because several
modules genuinely belong to several chapters.

| module | home | used by |
|---|---|---|
| `motor/motorbus.py` | ch1 | 1, 4 |
| `IMU_sensor/imu_dog.py` | ch2 | 2, 4 |
| `dog5_description/dog5_kinematics.py` | ch3 | 3, 4 |
| `dog5_description/dog5_statics.py` | ch3 | 3, 4 |
| `robot_base/stand_dog5_hw.py` | ch3 | 3, 4 |

## Chapters

| # | chapter | claim | status |
|---|---|---|---|
| 1 | [Motor library](docs/ch1_motor_library.md) | 12 motors on one CAN bus at 250 Hz **per motor**, recovery with no power cycle | **PASSED** |
| 2 | [IMU sensor](docs/ch2_imu_sensor.md) | ~200 Hz, 0 CRC errors, 0.01–0.03° jitter under motor load | **PASSED** |
| 3 | [Kinematics & calibration](docs/ch3_kinematics_and_calibration.md) | NumPy FK/Jacobian agree with MuJoCo to 1.665e-16 m | **PASSED** |
| 4 | [Quasi-dynamic trot](docs/ch4_quasi_dynamic_trot.md) | Torque stand works and is repeatable; the trot runs and drifts | **PARTIAL** |

A one-page version: **[docs/SUMMARY.md](docs/SUMMARY.md)**.

## What is not here

Three substantial bodies of code were written for this robot and are **not** in
this repository, because none of them has ever been in the control loop. They
live in the private `dog_stand_compliance_control` history at tag
`pre-public-reorg-2026-08-26`.

| what | why it is not here |
|---|---|
| **An error-state (Bloesch) EKF** — quaternion attitude, IMU bias tracking, contact-constrained velocity, plus a read-only worker, a replay harness and acceptance gates | It has never run in the loop. The shipped feedback is the direct chain: AHRS attitude, FK height, algebraic leg odometry. §5 of chapter 4 is what actually runs |
| **A cone-constrained force-distribution QP** and a `TrotController` around it — SO(3) log-map attitude error, weighted wrench residual, contact-weight bounds, world-frame Raibert swing | The QP was instantiated only inside its own self-test. What flies is damped least squares plus clamps — chapter 4 §6 |
| **A convex MPC** over the single-rigid-body model | Ported, runs offline, never validated on hardware and never gated |

Also removed: the position-mode stand-and-crawl track (hardware-verified July
2026, but nothing in the trot chain imports it), the July torque primitives
that `torque_stand/` superseded, and the Virtual Model Control track whose
simulation passed and whose hardware runner did not.

This is stated plainly because the alternative — shipping the sophisticated
version and letting a reader assume it is what runs — is the more misleading
choice. The interesting result here is what a quasi-static model with no
estimator actually does on a real robot, and that needs the real code.

## Hardware and environment

- 12 × LK / K-TECH brushless servo drivers, CAN IDs 1–12, 10:1 reduction
- SocketCAN `can0` at 1 Mbit/s (`src/bringup/t1_preflight.sh` brings it up)
- 1 × FDILink DETA10 AHRS on the trunk
- Robot mass 5.8151 kg, of which the legs are 3.196 kg — **55 %**
- A Pi-class Linux host running the control loop at 250 Hz per motor

**`fdilink_imu`, the IMU vendor SDK, is not on PyPI and is not vendored here.**
Install it on the robot host at `~/Documents/IMU_sensor/fdilink_imu`. Without
it everything still imports and every offline gate still passes; only opening a
real IMU fails. See [chapter 2](docs/ch2_imu_sensor.md) §7.

MuJoCo is a **hard** dependency, not an optional simulation extra — the
hardware stack imports it transitively for FK and Jacobians.

## Safety

Every `*_hw.py` moves a 5.8 kg machine with twelve motors that can bite.

- Run `--self-test` first. Every runner has one, and it needs no hardware.
- Follow the runbooks: [demo](docs/runbooks/DEMO_RUNBOOK.md),
  [stand](docs/runbooks/DOG5_STAND_RUNBOOK.md).
- Start suspended, then use the torque ladder on the floor. `TAU_START_MAX` is
  1.0 N·m for a reason.

**The trips are not a safety net you can lean on.** Chapter 4 §9 documents a
run where the tilt trip did not fire, the robot ended up on its belly at 2.4×
body weight, **and read perfectly level while doing so** — because a robot
lying flat is level. Know what each gate can and cannot see.

## Provenance

Assembled from two private repositories, both tagged
`pre-public-reorg-2026-08-26`:

- `can_motor_control` — the CAN/motor layer
- `dog_stand_compliance_control` — everything robot-level, embedded in the
  first as a gitlink with no `.gitmodules`, so cloning the parent produced an
  empty directory

## Citation

> Zhan Zhi, *DOG5: a quadruped from CAN bus to quasi-dynamic gait*, Nanyang
> Technological University, 2026. https://github.com/snowyfoams/dog5

## Licence

[MIT](LICENSE). The IMU vendor SDK and the Fusion→MJCF exporter are
third-party and are not included; see the notices at the foot of `LICENSE`.
