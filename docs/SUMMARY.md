# DOG5 — two months, summarised

*One `##` per slide. Every number is measured, and every one names the run or
gate it came from.*

---

## The robot

A 5.8151 kg quadruped, built from scratch.

- **12** LK/K-TECH brushless servos, CAN IDs 1–12, 10:1 reduction
- **1** FDILink DETA10 AHRS on the trunk
- One `can0` bus at 1 Mbit/s; a Pi-class Linux host
- PLA printed structure on a steel frame
- Legs are **3.196 kg of 5.815 kg — 55% of the robot**

No force sensors. No contact switches. No motion capture. Everything below was
done with encoders, one IMU, and a model.

---

## What works

| | result | evidence |
|---|---|---|
| **CAN** | 12 motors at **250 Hz each** — 3000 frames/s — with recovery that never needs a power cycle | `t3_rate_sweep`, `t7_recover_no_powercycle` |
| **IMU** | **~200 Hz**, 0 CRC errors, **0.01–0.03°** jitter with motors loaded | `imu_noise_log` |
| **Kinematics** | NumPy FK and Jacobian match MuJoCo to **1.665e-16 m** over 1024 poses | `check_dog5_kinematics` |
| **Position stand** | 3-leg stand hold, **all four legs** | hardware, 2026-07-23 |
| **Crawl** | autonomous, 3+ cycles, 120+ mm, **40 mm** steps, no operator input | hardware, 2026-07-23 |
| **Closed-loop stand** | EKF-driven height and attitude trim | hardware PASS, 2026-08-11 |
| **Torque stand** | quasi-static wrench → GRF → joint torque, on hardware | ch6 |

---

## How it was built — the ladder

```
  ch1  motor       does the bus answer, and how fast
   │
  ch2  sensor      where is down
   │
  ch3  model       where are the feet          <- verified twice, independently
   │
  ch4  open loop   stand and crawl             <- position mode; IMU may judge, never steer
   │
  ch5  closed loop stand on the estimate       <- still position mode
   │
  ch6  torque      software computes tau        <- stand works; gaits drift
   │
  ch7  MPC         ported, unvalidated
```

Each rung is only attempted once the one below it has a hardware pass.

---

## The method

Three things applied to every stage, and they are more transferable than any
control law here:

1. **A parameter file with no logic and no imports.** `stand_params.py`,
   `torque_params.py`. Readable from a test, a notebook or a plotting script.
   It ended a class of bug where a constant was declared twice and drifted.
2. **An offline gate suite.** **504 gates across 14 suites**, all runnable with
   no robot, no CAN interface and no data. A control law is not "done" until it
   has gates.
3. **A runbook per hardware procedure**, with the failure modes written down
   before the run, not after.

---

## Measure twice, with different physics

The most useful habit in the project: never let one sensor be the only witness.

- **FK vs MuJoCo** — two independent kinematics implementations, agreeing at
  machine epsilon. A sign error in one would not have been visible in the
  other.
- **EKF vs AHRS** — the raw sensor holds a *veto* over the filter, not a vote.
- **zEKF vs zFK** — when three feet are planted, both fully determine trunk
  height. Agreement is the pass condition.
- **Encoders vs IMU** — and this is the one that paid: during a crawl step,
  encoder FK reported **0.2 mm** of sag while the IMU showed **7°** of body
  roll, about **14 mm** of corner height. A factor of seventy. The joints were
  tracking perfectly; the deflection was backlash, rubber feet and frame flex —
  everything *outside* the encoders.

---

## What does not work yet

- **Trot-in-place drifts and does not sustain.** Three of four logged runs hit
  the 12° tilt e-stop within **0.4–0.8 s** of starting to trot.
- **Walk is slow, and also drifts.**
- **The roll is real** — unfiltered gyro `w_x` reads 2.2–2.7 rad/s through
  every excursion, against <1.93° over 31.5 s of standing.
- **It is always negative**, −6 to −12°, never positive, across every run.
- **The FK attitude check assumes flat ground and four feet down** — so it is
  blind exactly when attitude matters.
- **A safety trip failed to fire.** Run t2: height 149 → **−53 mm**, vertical
  force **136 N = 2.4× body weight**. The robot was on its belly — and read
  **level**, because a robot lying flat is level. Tilt could not catch it, and
  the load check does not run during the gait.

---

## The three things I would do next

1. **Swap the lead diagonal.** The roll is consistently negative. One run
   separates "handover asymmetry" from "fixed bias" (CoM, IMU mount, or leg
   zeros). It is the cheapest decisive experiment available and it has not been
   done.
2. **Add the angular-momentum terms.** The model is quasi-static — no `Iα`, no
   `ω × Iω`. A nearly controlled pair (t1 vs t5, attitude gains 10/0.5 against
   0/0) changed the divergence hardly at all, which is what you would expect if
   the missing physics is the problem rather than the tuning.
3. **Give yaw a real reference.** The magnetometer is unusable beside twelve
   motors and a steel frame, so yaw is closed on rate only and drifts.

---

## Honest limits of the whole thing

- No URDF; the MJCF cannot be regenerated here (the exporter is external).
- **Inertias are assumed, never weighed** — uniform PLA at 1.0 g/cm³. Every
  model-based result inherits that error, and nobody has bounded it.
- One IMU, no redundancy. No force or contact sensing anywhere — **contact is
  asserted, not measured**, and that assumption is behind more than one bug in
  this record.
- No motion capture: ground truth is a tape measure.
- The force-distribution QP is hand-rolled, with no optimality certificate and
  no bound on solve time.
- No CI. The gates are real, but nothing runs them automatically.
- Chapter 7 has no hardware result and no gates at all.

---

## Where the code is

**https://github.com/snowyfoams/dog5** — seven chapters, 504 offline gates, and
an `archive/` that keeps the dead ends with an explanation of each.

The clearest single artefact for a reader in a hurry:
`docs/ch4_position_stand_and_crawl.md` §7 — two sensors measuring the same
event and disagreeing by a factor of seventy, and what that forced.
