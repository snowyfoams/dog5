# Chapter 4 — Position-mode stand and crawl

**Status: PASSED.** Three-leg stand hold on all four legs (2026-07-23), and an
autonomous crawl — 3+ gait cycles, 120+ mm, 40 mm steps, no operator input, no
aborts.

Source: [`src/position_stand_crawl/`](../src/position_stand_crawl) ·
shared base in [`src/robot_base/`](../src/robot_base) ·
runbooks in [`runbooks/`](runbooks)

---

## 1. Why position mode first

The drivers already contain tuned position loops. Command `0xA4` and the
firmware holds the joint — no software torque law, no 4 ms deadline to miss,
no chance of a runaway from a sign error in a wrench calculation.

So the whole of chapter 4 is: **the motors do joint control; the host decides
where the joints should be; the IMU is allowed to judge but never to steer.**
Every safety mechanism is a trip, not a correction. That constraint is what
made it possible to get a walking robot before the state estimator existed.

`stand_by_position_command.py` is the clean ancestor —
`READ → CROUCH → WAIT → STAND → HOLD → PARK` using nothing but `0xA4`.

## 2. What you inherit from `robot_base/`

Four modules that look like superseded first-generation runners and are in
fact the shared substrate of chapters 4, 5 and 6. `stand_dog5_hw.py` alone has
**38 importers**.

| module | what later code takes from it |
|---|---|
| `stand_dog5_hw.py` | `JOINT_LABELS`, `CROUCH_JOINT_TARGET_DEG`, the staged `REST → ROLL → FOLD → STAND` machine, and the operator gating |
| `stand_dog5_recorded_hw.py` | the *recorded* crouch — a hand-arranged pose captured with `dog5_pose_monitor.py` — which every later stand starts from |
| `stand_dog5_inplace_hw.py` | standing **in place** (feet rise straight up from their crouch x/y) plus the per-leg vertical-force integral trim, and `ConfirmedSafetyGate` |
| `crawl_dog5_hw.py` | the first-generation crawl; still imported by eight modules |

Note that importing any of these pulls in MuJoCo, because `stand_dog5_hw.py`
imports `stand_dog5.py` which imports `mujoco` at module scope. MuJoCo is used
here only for FK and Jacobians — never as a plant.

Standing *in place* rather than sprawling outward was a hardware lesson, not a
design choice: on a grippy floor an outward-sliding foot sticks, and in one
run `FR_knee` stalled 45.9° short of its target while the controller happily
reported it was tracking.

## 3. The FK attitude check, and exactly when it lies

With four feet on the floor, forward kinematics gives four foot positions in
the trunk frame. Fit a plane through them and you get the trunk's attitude
relative to that plane, plus a floor height — with no IMU at all.

That is `fk_attitude` / `fk_floor_height`, and it is genuinely useful: it is an
*independent* check on the estimator in chapter 5, derived from a completely
different sensor.

**It is valid only when the ground is flat and all four feet are on it.**

Both halves matter. Lift a foot and the plane is fitted through three points
plus one meaningless one. Put the robot on a slope and the "attitude" it
reports is attitude relative to the slope, which is not what any leveling law
wants. And chapter 3 §8 already said the deeper version: encoders cannot see
contact, slip or gravity, so the check is blind in precisely the situations
where you most want to know your attitude.

> These helpers currently live inside a *chapter 5* runner
> (`imu_closedloop_stand/stand_ekf_level_hw.py`) plus `stand_params.py`, not in
> `dog5_kinematics`. Moving them is deliberately out of scope for this
> reorganisation, which made no behavioural changes.

## 4. Shift the CoM, then unload — and measure it

The core primitive of static walking: to lift a foot, first move the body so
that foot carries no load.

The naive version shifts by a computed amount and lifts. That fails, because
the amount depends on compliance nobody modelled. Two tools exist because of
that:

- **`manual_swing_hw.py`** — keys 1–4 swing the CoM off a chosen corner and
  hold there indefinitely while the `mg/4` feedforward on that leg fades. You
  *watch the measured load on that foot fall to zero* before anything lifts.
  `+`/`−` grows the swing 2 mm at a time, live. This is the teaching tool: it
  makes the unload observable instead of assumed.
- **`com_swing_test_hw.py`** — the same idea as a timed episode:
  `SHIFT → UNLOAD → LIFT → HOLD3 → LOWER → LOAD → RECENTER`, where **UNLOAD
  must measure unloaded**, not wait a fixed time.

The shift direction is perpendicular to the *limiting edge of the current
stance triangle*, so it adapts as the support polygon changes shape rather
than assuming a nominal square stance.

## 5. Three-leg stand, then the crawl

`stand3_hold_hw.py` is the ladder that had to work before any gait could:
in-place stand → diagonal shift → pre-lift with a **clear gate** → lift →
`HOLD3` → lower → recentre. Hardware-passed on all four legs, 2026-07-23.

The clear gate is the honest part: a step is only allowed to proceed when the
foot has measurably risen (≥5 mm) **and** the swing torque has dropped
(|τ| ≤ 0.7 N·m). Airborne swing torque measures 0.2–0.5 N·m against ≥0.9 when
loaded, so the 0.7 threshold separates cleanly.

`walk1_hw.py` is the reference crawl: N cycles in the order **RR → FL → RL →
FR**, each step being the `stand3` ladder plus `SWING` → `TOUCHDOWN` →
`RECENTER`. Touchdown commits the **measured** foot position as the new ground
anchor, rather than the commanded one — which is what keeps the accumulated
gait geometry honest across cycles. It also hosts a read-only EKF worker and a
`--web` dashboard, which is how chapter 5's estimator was validated against a
motion the robot was already able to perform.

## 6. What the hardware actually measured

From the 2026-07-23 review:

| quantity | measured | note |
|---|---|---|
| corner sag at the clear gate | **~0.2 mm** | simulation guessed ~12 mm; real servos are far stiffer |
| foot rise at 20 mm pre-lift | +19.7 … +19.9 mm | clear gate passes first try, every step |
| touchdown onto the plane | +0.2 … +1.3 mm | |
| anchor advance | exactly **+40 mm** per cycle, every foot | |
| stance support margin | 20.5–30.9 mm | gate is 15 mm |
| peak joint torque | **2.0 N·m** | trip at 6.0 |
| control compute per sweep | ≤0.9 ms | budget ~6 ms |

The abduction soft limit of **±1.75 rad** is the binding constraint nearly
everywhere — lateral CoM travel is only about 2 cm. Step length is 40 mm
against a leg-length wall at 45 mm.

## 7. The problem this chapter ends on

During **every** FL step's lift and swing, the trunk rolls to **−7.2° peak**
(left side down). RR steps reach about −5.5° near touchdown. Nothing trips,
nothing fails — and 7° of uncommanded body roll is large.

The diagnosis is the interesting part:

> The clear gate measures **~0.2 mm** of sag by encoder FK while the IMU shows
> **7°** of real body roll — about 14 mm of differential corner height at the
> 0.1125 m half-track.

The joints are tracking their targets almost perfectly. The deflection is
entirely in what sits *outside* the encoders: **gear backlash under load
reversal, rubber-foot compression, and frame flex.** Position mode has no
posture correction, so unmodelled compliance simply lands wherever it lands.

Two independent measurements of the same event disagreeing by a factor of
seventy is the cleanest possible argument for adding a sensor that can see
gravity. That is chapter 5.

Related: `archive/legacy_runners/twostand_hw.py` deliberately attempted a
two-leg diagonal stand and recorded that the CoM sits left of the FL–RR
diagonal, so the body rolls **−8.5°** onto RL. Rotation about the support
diagonal is unactuated. That is chapter 6's motivation.

## 8. Run it

Full procedures, safety gates and every CLI flag are in the runbooks:

- [`runbooks/DOG5_STAND_RUNBOOK.md`](runbooks/DOG5_STAND_RUNBOOK.md) — stand at
  250 Hz, including the suspended low-torque check and the floor ladder
  5 → 6 → 7 → 8 N·m
- [`runbooks/DOG5_CRAWL_RUNBOOK.md`](runbooks/DOG5_CRAWL_RUNBOOK.md) — crouch →
  compliant stand → statically stable crawl; the phase machine, the gates, and
  gate-failure versus e-stop

Offline first, always:

```bash
python3 src/position_stand_crawl/walk1_hw.py --self-test
```

## Known limitations / what's next

- **The FK attitude check assumes flat ground and four feet down**, so it is
  blind exactly when attitude matters most — mid-step, or on a slope.
- **An unexplained −7.2° trunk roll on every FL step** that the encoders cannot
  see. Attributed to backlash, foot compression and frame flex; never
  separated into those three components.
- **No IMU in the control loop here** — it judges and can e-stop, but it never
  steers. Nothing corrects posture.
- No force sensors and no contact switches. Touchdown is *inferred* from
  encoder FK, which §7 shows can be wrong by a factor of seventy about
  deflection.
- The CoM is assumed to project at the trunk origin. Chapter 3 §2 says it is
  14.7 mm away.
- 40 mm of a 45 mm reachable step leaves no margin, and ±1.75 rad of abduction
  gives only ~2 cm of lateral CoM travel — the crawl is close to its geometric
  limits in every direction.
- `RECENTER` is open-loop in yaw; heading drift over many cycles is unmeasured.
