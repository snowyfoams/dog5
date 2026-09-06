# DOG5 — the whole thing, on one page

*Every number below is measured, and names the run or gate it came from.*

---

## The robot

A 5.8151 kg quadruped, built from scratch.

- **12** LK/K-TECH brushless servos, CAN IDs 1–12, 10:1 reduction
- **1** FDILink DETA10 AHRS on the trunk
- One `can0` bus at 1 Mbit/s; a Pi-class Linux host
- PLA printed structure on a steel frame
- Legs are **3.196 kg of 5.815 kg — 55 % of the robot**

No force sensors. No contact switches. No motion capture. Everything below was
done with encoders, one IMU, and a model.

---

## What works

| | result | evidence |
|---|---|---|
| **CAN** | 12 motors at **250 Hz each** — 3000 frames/s — with recovery that never needs a power cycle | `t3_rate_sweep`, `t7_recover_no_powercycle` |
| **IMU** | **~200 Hz**, 0 CRC errors, **0.01–0.03°** jitter with motors loaded | `imu_noise_log` |
| **Kinematics** | NumPy FK and Jacobian match MuJoCo to **1.665e-16 m** over 1024 poses | `check_dog5_kinematics` |
| **Statics** | `−Jᵀf + leg_gravity` matches MuJoCo floating-base inverse dynamics to **1.8e-15 N·m**; `−Jᵀf` alone is wrong by 0.472 N·m at the crouch | `test_dog5_statics` |
| **Torque stand** | quasi-static wrench → GRF → joint torque, held on hardware, repeatable | ch4 |
| **Trot + walk** | runs, stays up, and drifts | ch4 §9 |

---

## The stack

```
  ch1  motor       does the bus answer, and how fast
   │
  ch2  sensor      where is down
   │
  ch3  model       where are the feet          <- verified twice, independently
   │
  ch4  torque      software computes tau       <- stand works; the gait drifts
```

Each rung was only attempted once the one below it had a hardware pass.

**The control loop itself**, in four boxes:

```
encoders + AHRS  ->  (z, v, C, w)     no filter, no fused state
                 ->  6-DoF wrench     PD on {z, roll, pitch, yaw} + mg
                 ->  four foot forces damped least squares, then clamp
                 ->  twelve torques   -J^T C^T f + leg gravity, then impedance
```

83.3 Hz for the model block, **250 Hz for the impedance and the CAN sweep**.
Only the feedforward is held between model updates — the term that actually
stabilises a joint never gets sub-sampled.

---

## The method

Three things applied at every stage, and they transfer better than any control
law here:

1. **A parameter file with no logic and no imports.** `torque_stand/params.py`
   is readable from a test, a notebook or a plotting script. It ended a class
   of bug where a constant was declared twice and drifted.
2. **An offline gate suite.** 13 suites, no robot, no CAN, no data. A control
   law is not "done" until it has gates. `python3 src/selftest/test_all.py`.
3. **A runbook per hardware procedure**, with the failure modes written down
   *before* the run, not after.

---

## Measure twice, with different physics

The most useful habit in the project: never let one sensor be the only witness.

- **FK vs MuJoCo** — two independent kinematics implementations, agreeing at
  machine epsilon. A sign error in one would not have been visible in the
  other.
- **AHRS vs a plane through the feet** — `rp:d = ahrs − fk` has no IMU in the
  second term, so it measures floor slope + mount tilt + leg zeros. It is the
  only independent check on attitude, and it printed ~0.5° with the robot
  standing still.
- **Commanded torque vs measured `iq`** — the foot-load sum in every exit
  report. It must read 57 N. It is the only end-to-end evidence that a
  commanded newton-metre becomes a real one.
- **Encoders vs IMU** — and this is the one that paid: during a crawl step,
  encoder FK reported **0.2 mm** of sag while the IMU showed **7°** of body
  roll, about **14 mm** of corner height. A factor of seventy. The joints were
  tracking perfectly; the deflection was backlash, rubber feet and frame flex —
  everything *outside* the encoders.

---

## What does not work yet

- **The trot drifts** in position and heading. Nothing in the loop is trying
  to hold a position: rows `x` and `y` of the wrench are identically zero, and
  the one horizontal mechanism the controller has — where it puts its feet —
  is switched off (`RAIBERT_ON = False`).
- **The roll is real, and it is always negative.** −6 to −12°, never positive,
  across every logged run at `DUTY = 0.60`. Unfiltered gyro `w_x` reads
  2.2–2.7 rad/s through every excursion, against < 1.93° over 31.5 s of
  standing.
- **What ships is a more conservative gait than what was analysed.**
  `DUTY = 0.80` gives 0.72 s of four-foot support in every 1.2 s cycle. That
  is much of why the demo holds.
- **A safety trip failed to fire.** Run t2: height 149 → **−53 mm**, vertical
  force **136 N = 2.4× body weight**. The robot was on its belly — and read
  **level**, because a robot lying flat is level. Tilt could not catch it, and
  the load check does not run during the gait.
- **Attitude stiffness ships below what any log stands behind** (3.0 against
  the provenance tables' 10) and the comments were not updated.

---

## The three things I would do next

1. **Swap the lead diagonal.** The roll is consistently negative. One run
   separates "handover asymmetry" from "fixed bias" (CoM, IMU mount, or leg
   zeros). It is the cheapest decisive experiment available, it is one edit —
   `LEAD_DIAGONAL` — and it has not been done.
2. **Add the angular-momentum terms.** The model is quasi-static — no `Iα`, no
   `ω × Iω`. A nearly controlled pair (t1 vs t5, attitude gains 10/0.5 against
   0/0) changed the divergence hardly at all, which is what you would expect if
   the missing physics is the problem rather than the tuning.
3. **Bound the velocity estimator's bias.** Foot placement is the actuator in
   the plane and it rides on measured velocity: 0.150 s of gain on the
   estimate, so a 10 mm/s bias is 38 mm of displacement in ten seconds. That
   is fixed with an estimator, not with a gain.

---

## Honest limits of the whole thing

- **There is no state estimator in the loop.** Attitude is the AHRS's own
  fusion, height is FK, velocity is algebraic leg odometry over a contact set
  that is *scheduled, not measured*. No covariance, no bias tracking, no fused
  position.
- **Contact is asserted by a clock, never measured.** No force or contact
  sensing anywhere, and that assumption is behind more than one bug in this
  record.
- **Inertias are assumed, never weighed** — uniform PLA at 1.0 g/cm³. Every
  model-based result inherits that error and nobody has bounded it.
- No URDF; the MJCF cannot be regenerated here (the exporter is external).
- One IMU, no redundancy. No motion capture: ground truth is a tape measure.
- Force allocation **clamps after a solve** rather than constraining one, so a
  foot near lifting can be asked for force it cannot make.
- Yaw is closed on rate only — the magnetometer is unusable beside twelve
  motors and a steel frame.
- No CI. The gates are real, but nothing runs them automatically.

---

## Where the code is

**https://github.com/snowyfoams/dog5** — four chapters, and only code that has
run on the robot. What was written but never flew is listed in the README
rather than shipped.

The clearest single artefact for a reader in a hurry:
[`docs/ch4_quasi_dynamic_trot.md`](ch4_quasi_dynamic_trot.md) §5 — what a
control loop actually knows about a robot when it has twelve encoders, one
IMU, and no filter.
