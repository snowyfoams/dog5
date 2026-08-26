# Chapter 5 — Closing the loop on the IMU (still position mode)

**Status: PASSED.** Stage 1 hardware PASS 2026-08-11; stages 2, 2b and 3 all
ran on hardware.

Source: [`src/imu_closedloop_stand/`](../src/imu_closedloop_stand) ·
estimator in [`src/state_estimator/`](../src/state_estimator) ·
close-out in [`src/ekf_closeout/`](../src/ekf_closeout)

---

Chapter 4 ended with encoder FK reporting 0.2 mm of sag while the robot was
actually rolling 7°. This chapter adds the sensor that can see gravity — and,
just as importantly, keeps the robot in **position mode** while doing it. The
motors' own loops still do joint control. What changes is only *where the
targets are put*.

## 1. The stages

| stage | file | what it adds |
|---|---|---|
| 1 | `stand_ekf_verify_hw.py` | rigid position stand, EKF **read-only**. Measures whether the estimator can be believed at all |
| 2 | `stand_ekf_height_hw.py` | EKF height → a **common** foot-z offset |
| 2b | `stand_ekf_level_hw.py` | EKF roll/pitch → **per-foot differential** z offsets |
| 2b-x | `lift_ekf_contact_hw.py` | lift one foot 20 mm and compare zEKF against zFK at 4-down / 3-down / re-planted |
| A | `stand_ahrs_level_hw.py` | leveling closed on the **raw AHRS**, no EKF at all |
| B | `stand_ekf_schedcontact_hw.py` | the fix that experiment A's diagnosis implied |
| 3 | `trot_fk_switch_hw.py` | trot in place with **no gait clock** |

## 2. Where the tunables live

`stand_params.py` — every tunable in the track, **pure literals, zero
imports**, with at-a-glance tables in its own docstring (all slews, all
heights, all gains) and a map of which module owns which shared function.

The "zero imports" part is a contract, not an accident: it is what makes the
file readable from a test, a notebook or a plotting script without dragging in
NumPy, CAN or an IMU. `src/selftest/test_layout.py` gates it, and that gate
caught a regression during this very reorganisation.

This is the single best pattern in the project. It ended a class of bug where
the same constant was declared in two runners and drifted.

## 3. Three different things called "height"

Mixing these is how a phantom 100 mm scare happened.

- **foot site** — `foot_position()` returns the *centre* of a 20 mm contact
  sphere, so the floor is `FOOT_RADIUS_M` below it.
- **hip axis** — the FK trunk origin; `dog5.xml` puts all four hip bodies at
  z = 0. This is what a tape measure to the hip pivot gives.
- **IMU board** — what the EKF's `r` actually tracks, and therefore what `zFK`
  reports by default.

The board sits **38 mm below the trunk origin** on hardware while `dog5.xml`
models it at the origin. The offset is a **body-frame** constant, so its
vertical component is `38 mm · C[2,2]` — it shrinks as the trunk tilts (by
0.6 mm at 10°) rather than being subtracted flat. Getting that wrong is a
slow, plausible-looking error.

## 4. Which estimate to trust for what

Stage 1 settled this, and everything after follows it:

| quantity | use | why |
|---|---|---|
| **roll / pitch** | **EKF** | directly observable from gravity; drift-free, and it is what leveling closes on |
| **absolute height, four feet down** | **FK** (`zFK`) | no integration at all, so it cannot drift. Measured *more* accurate than `zEKF` while holding |
| **height while a foot is lifting** | **EKF** | FK height needs a contact assumption; mid-swing there isn't one |
| **velocity** | **EKF** | FK gives no velocity without differentiating noisy encoders |

The EKF is not redundant — it owns attitude and velocity outright. It simply
should not be asked for a height the legs already know better. Note this is a
*division of labour*, not a fusion: each quantity has one owner.

## 5. Stage 1 — read-only, and the gates that let stage 2 start

`stand_ekf_verify_hw.py` stands rigidly and runs the estimator without letting
it touch anything. Four measurements:

- **LEVEL** — the resting roll/pitch, which *is* the setpoint stage 2 needs.
  There is no such thing as "level" in the abstract; there is the attitude
  this robot rests at on this floor.
- **AGREE** — |EKF − AHRS|, which must stay under ~2° on both axes.
- **REPEAT** — park/stand repeatability.
- **zFK vs zEKF drift** over a ≥30 s hold. A slow ramp means bias is still
  converging, and stage 2 would chase it.

Hardware PASS 2026-08-11.

A subtlety worth stating: **EKF `z` is not zero when parked.** It is the height
of the IMU board above the inertial origin fixed at startup, not a height above
the floor.

## 6. Stage 2 — height, and why it is an integrator and nothing else

EKF height drives a **common** offset applied to all four foot targets.
Integral only: no proportional term, no feedforward. 1 mm deadband.

That is deliberate. The estimator's height is the *least* trustworthy of the
things it produces (§4), the plant is a stack of position loops with their own
dynamics, and the whole point of position mode is that nothing surprising
happens fast. An integrator with a deadband can only creep.

**FK is the watchdog, not the feedback.** If `zFK` and `zEKF` disagree by more
than 30 mm, or the estimate goes stale or unhealthy, the trim freezes where it
is. The independent sensor gets a veto but not a vote.

There is also a real-time detail: IK inside the CAN sweep cost 5–7 ms against
a 4 ms budget. It was replaced by a **precomputed z → q table per leg**, at
**63 µs**. No inverse kinematics runs in the sweep at all.

## 7. Stage 2b — attitude

EKF roll/pitch drive **per-foot differential** z offsets: push two feet down,
lift two, so the trunk rotates without translating. The offsets are
**zero-meaned over the planted feet**, which is what keeps height and attitude
from fighting each other. The setpoint is *latched* from stage 1's LEVEL
measurement, and the **AHRS holds a veto** — if the raw sensor and the filter
disagree, the trim stops.

Every later runner imports its leveling law from this module rather than
reimplementing it.

## 8. The two experiments, and what they found

This is the most instructive sequence in the project.

**Experiment A** (`stand_ahrs_level_hw.py`) was written because during the rise
ramp the EKF's attitude split **4–5°** from the AHRS. So: level on the raw
AHRS instead, no EKF anywhere, and see whether the filter is the problem.

**The diagnosis**, and it was not what anyone expected: the recorded crouch
leaves the **rear feet off the floor**. The estimator's `initialise()` was
anchoring two footholds **in mid-air** and then trying to reconcile them with
gravity.

**Experiment B** (`stand_ekf_schedcontact_hw.py`) is the fix — feed the filter
an honest gait-schedule contact input instead of asserting all four feet are
planted. The estimator was never wrong; it was being lied to about contact.

Related A/B: contacts **ON** beats `--dead-reckon-ramps`, which drifted
**+11 mm** over the rise.

## 9. The one-foot-lift measurement

`lift_ekf_contact_hw.py` lifts one foot 20 mm and prints zEKF against zFK at
four-down, three-down and re-planted. The headline result is easy to
misread:

> **zEKF and zFK agree — and that is success, not a failed test.**

Three planted feet fully determine trunk height. When both methods say the
same thing, the geometry is consistent. `--fake-contacts` is the informative
A/B: assert a contact that isn't there and watch them diverge.

## 10. Stage 3 — trot in place with no clock

`trot_fk_switch_hw.py` runs a trot where **FK decides when to switch
diagonals**, not a timer: the next pair is released when the current pair has
measurably returned. `TrotFKSwitch`, `SwingClearance` and `StanceLoadBalance`
are the three pieces.

Two hardware findings:

- **`clearance ≈ lift + push − 2 × 13 mm`.** Lifting is not enough — the stance
  pair has to *push*, or the swing feet do not clear. There is a cliff between
  14 mm and 16 mm of lift.
- **One foot lifts and the other doesn't is a statics problem, not a lift
  problem.** Rotation about the support diagonal is unactuated, so where the
  CoM sits decides which of the two feet actually leaves the ground. Fixed with
  a constant `--com-shift-x/y` trim measured from per-leg sag.

It stands at **0.17 m rather than 0.19 m**, deliberately: at 0.19 m there is
not enough remaining leg extension for the stance pair to push.

## 11. The estimator underneath

A Bloesch et al. (RSS 2013) quaternion **error-state** EKF —
`dog5_state_estimator.py`, 28 nominal states and 27 error states, with every
comment carrying its equation number from the paper. Conventions are chapter 3
§3.

`ekf_runtime.EkfShared` + `ekf_worker` is the reusable **read-only** worker
thread that every runner in this chapter hosts. It exists because running the
2.8 ms filter inside the 10 ms CAN sweep caused input-lost latches — the
estimator must not share a deadline with the bus.

By design, **x, y and yaw are unobservable**. Nothing in this chapter tries to
fix that; it treats them as unavailable.

Offline gates C1–C7 (`test_estimator.py`) all pass, including the decisive C5:
static-noisy |v| RMS **4.88 mm/s** (limit 5), z RMS **0.28 mm** (limit 5),
roll/pitch **0.062°** (limit 0.3).

## 12. Close-out

`ekf_closeout/` replaces a motion-capture room with two tape-measure sessions —
a distance walk for x/y drift, and a three-height crouch→stand for z scale.
See [`runbooks/EKF_CLOSEOUT_RUNBOOK.md`](runbooks/EKF_CLOSEOUT_RUNBOOK.md),
which is unusually honest about its own limits.

`estimator_health.py` fixes a real numerical artefact: the health flag tested
`min_eig > -1e-9` as an **absolute** tolerance, while a swing-leg covariance
inflation of `1e4` pushed `max(diag P)` to ~7e8. Every healthy filter looked
unhealthy. The fix is a *relative* tolerance.

## 13. Run it

```bash
python3 src/selftest/test_all.py                    # 504 gates, no robot
python3 src/state_estimator/test_estimator.py       # EKF gates C1-C7
python3 src/imu_closedloop_stand/stand_ekf_verify_hw.py --self-test
```

## Known limitations / what's next

- **The EKF is read-only. It never commands anything.** Every correction in
  this chapter is a *target offset*; the motors' own position loops do the
  work.
- **Correction is integral-only**, with deadbands, so the bandwidth is
  deliberately tiny. This rejects slow bias, not disturbance.
- **EKF and AHRS attitude split 4–5° during ramps** and the resolution was to
  give the AHRS a *veto*, not to reconcile the two. The underlying cause was
  contact scheduling, and it is not certain it is fully resolved.
- **Contacts must be scheduled honestly or the filter anchors footholds in
  mid-air.** There is still no contact sensor — the schedule is asserted.
- x, y and yaw remain unobservable by design; there is no absolute position.
- Stage 3 stands 20 mm lower than intended purely to buy extension authority.
- **Still position mode.** The robot has no compliance: it cannot absorb an
  impact, and a disturbance is rejected only as fast as an integrator with a
  deadband can move a target. That is chapter 6's problem.
