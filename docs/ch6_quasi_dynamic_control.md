# Chapter 6 — Quasi-dynamic model control: stand, trot, walk

**Status: PARTIAL.** The torque stand works. Trot-in-place drifts and does not
sustain. Walk is slow and drifts.

Source: [`src/torque_stand/`](../src/torque_stand) ·
[`src/torque_primitives/`](../src/torque_primitives) ·
[`src/dog5_trot_quasi_static_model/`](../src/dog5_trot_quasi_static_model) ·
analysis in [`src/tools/`](../src/tools)

---

## 1. What "quasi-dynamic" means here, precisely

The trunk is treated as a single rigid body, and its state error is mapped to a
6D wrench. But the wrench is built **without the angular-momentum terms** — no
`Iα`, no `ω × Iω`. `Dynamic_Model`, now `dynamic_model.py`, says so in its own
docstring.

So the model knows about gravity, position error and velocity error, and does
not know that a spinning body resists being turned. Naming it *quasi-static*
is the honest description, and it is also — as §8 argues — probably why the
roll cannot be corrected.

The other honest framing: **this is the first chapter where software computes
torque.** Chapters 4 and 5 let the drivers' own loops do joint control. Here
the host computes `τ` and sends it. Everything that gets harder in this
chapter follows from that.

## 2. T0 — is a commanded N·m a real N·m?

`tau_calib_hw.py` asks the question that has to come first. The naive check —
command zero torque and read the current — does not work, because the driver's
own loop is not idle at zero.

Instead: sweep a gravity scale and measure the resulting impedance error. If
commanded torque is proportional to delivered torque, the sweep is linear and
its slope is the calibration.

## 3. The statics, and the term that was missing

```
τ = −Jᵀ f + leg_gravity
```

`dog5_statics.py` documents at length why **`−Jᵀf` alone is wrong**: the legs
are **3.196 kg of a 5.815 kg robot — 55% of the mass.** Treating the legs as
massless linkages that merely transmit a foot force throws away more than half
the robot's weight. The `leg_gravity` term is not a refinement; it is the
majority of the correction on a swing leg.

Validated against MuJoCo floating-base inverse dynamics.

This is also where the earlier VMC work is reused rather than replaced:
`stance_law.py` imports `body_wrench`, `grasp_map` and `distribute_wrench` from
`dog5_vmc_core` **unchanged**, on the grounds that they are sim-proven by gates
V1–V4 (see `archive/README.md`). Only the stance branch is replaced, and only
to add the leg-gravity term.

## 4. The rate correction, and the 100/250 Hz split

`torque_params.py` opens with the **12× loop-rate correction**: the CAN loop
runs at 250 Hz *per motor* — a 4 ms sweep — not the 20.8 Hz that an earlier
docstring claimed. Chapter 1 §3 has the full story. The torque track had been
abandoned on the strength of the wrong number.

The control law does not fit in a 4 ms sweep anyway. Measured on the robot's
Pi, per 4-leg sweep:

```
dog5_vmc_core.compute_vmc_torques      998 us
the corrected stance law (dog5_statics) 822 us
```

Either is 3–4× over the 333 µs CAN slot. So `stance_law.py` splits the work:

- a **100 Hz worker thread** computes the wrench → GRF → `τ_ff` and `q_ref`;
- the **250 Hz CAN sweep** adds `kp(q_ref − q) − kd·q̇` on telemetry no older
  than 4 ms, and keeps sweeping between worker updates.

The terms that actually stabilise a joint are three array operations and want
the freshest `q̇` available; the terms that need the whole robot's geometry can
be one worker tick stale. The watchdog is fed at 250 Hz regardless — which is
the lesson from the July failure, where running the estimator *and* the torque
law inside the sweep caused input-lost latches.

## 5. Feedback without a motion-capture rig

`feedback_estimator.py` produces `(z, v, C, ω)` from what exists:

- **attitude** from the AHRS (not the EKF — see §11);
- **height** from FK;
- **velocity** from an algebraic leg-odometry identity:

```
v_world = −Cᵀ (ω × s_i + J_i q̇_i)
```

for any stance leg `i`. No differentiation of noisy encoder positions, no
integration of accelerometer bias — if a foot is planted, the body velocity
follows in closed form from the joint rates. It is exact, and it is only as
good as the contact assumption, which is the recurring theme of this project.

## 6. Wrench → foot forces → joint torques

`force_totorque.py`: a **grasp map** relates the per-foot contact forces to the
resulting body wrench; inverting it with **damped least squares** distributes a
desired wrench across the stance feet; `−Jᵀ` then turns each foot force into
joint torques, with leg gravity added per §3.

For the gaits, `balance_qp.py` replaces the damped-least-squares step with a
proper **force-distribution QP** — unilateral constraints (feet can push, not
pull) and a friction pyramid. The solver is hand-rolled, because the robot's
virtual environment has NumPy and nothing else.

## 7. The torque stand — the part that works

`stand_torque_mode.py`: `CROUCH → WAIT → RISE → HOLD → PARK`, with
`RunawayBrake` and `LoadWatch` as the safety layer. The crouch and park
bookends are still native `0xA4` position commands — torque mode is entered
only once the robot is in a known pose.

This works. It is the foundation everything after it stands on.

## 8. Trot and walk — the part that does not

`dog5_trot_quasi_static_model/` splits the controller six ways:

| module | responsibility |
|---|---|
| `config.py` | every constant; MASS 5.8151 kg, whole-robot inertia about the whole-robot CoM, hip offsets — all traced to `dog5.xml`. Also explains why DOG5 has no textbook L1/L2/L3 |
| `leg_kin.py` | IK only; FK and Jacobians delegate to `dog5_kinematics` |
| `gait.py` | the trot clock: time in, contact schedule out. No robot state at all |
| `balance_qp.py` | virtual-model wrench + the force-distribution QP (§6) |
| `swing.py` | `SwingPlanner` with Raibert landing, where the nominal is the **stance foot position, not the hip**, plus Cartesian impedance for the swing leg |
| `controller.py` | orchestration, and all three frame conversions in one place — including the **CoM-vs-trunk-origin 14.7 mm lever = 0.84 N·m pitch bias** |

Runners: `trot_hw.py` (adds a yaw loop and a load-handover ramp),
`trot_demo.py` (a fixed four-foot re-level settle every N cycles, alternating
lead diagonal), `walk_demo.py` (displaced landing points — `T` sends the feet
out, `R` brings them back — with a per-leg swing ledger).

### What the logs actually say

Four torque-mode trot runs, 2026-08-18, 250 Hz. Robot weight 57 N.

| run | total | TROT | roll min/max (°) | peak \|gyro-x\| | z min (mm) | Σfz max (N) | kp/kd att | τ_max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| t1 | 12.3 s | **0.77 s** | −11.24 / −0.80 | 2.23 rad/s | 136 | 63 | 10 / 0.5 | 1.0 |
| t2 | 21.8 s | 6.01 s | −11.98 / +10.49 | 6.80 rad/s | **−53** | **136** | 10 / 0.5 | 3.0 |
| t3 | 11.5 s | **0.41 s** | −9.28 / −0.91 | 2.73 rad/s | 122 | 66 | 0 / 0.1 | 1.5 |
| t5 | 12.1 s | **0.51 s** | −11.44 / −0.74 | 2.16 rad/s | 140 | 61 | 0 / 0.0 | 2.0 |

Three conclusions, and they are the substance of this chapter:

**1. The roll is real.** t1, t3 and t5 all ended on the 12° tilt e-stop within
**0.4–0.8 s** of entering TROT. This is not an IMU spike: the unfiltered gyro
`w_x` independently reads **2.2–2.7 rad/s** (125–155°/s) through every
excursion, and the same IMU on the same robot never exceeds 1.93° over 31.5 s
of standing. The excursions build over about two gait cycles, not one sample.

**2. It always goes the same way** — negative, −6 to −12°, never positive.
That is what makes **swapping the lead diagonal the decisive next experiment**:
if the sign follows which diagonal leaves the ground, it is the handover; if it
does not, it is a fixed bias (CoM, IMU mount, or leg zeros). *That experiment
has not been run.*

**3. t2 is what happens when the safety trip does not fire.** `z` goes
149 → **−53 mm** and `Σfz` to **136 N — 2.4× body weight**. The robot is on its
belly. For the next ~7 s the gyro reads ~0.003 rad/s and roll sits at +0.6°,
**because a robot lying flat reads level.** Tilt cannot catch it, and the load
check is gated to `HOLD` and does not run during `TROT`.

t1 versus t5 is nearly a controlled pair — same period, duty and swing height,
attitude gains 10/0.5 against 0/0. The roll went to −11.24° and −11.44°.
`τ_max` differs (1.0 against 2.0) so it is not clean, but **the attitude loop
did not visibly change the divergence.** Which is what §1 predicts: a
quasi-static wrench has no angular-momentum term to fight a developing rotation
with.

## 9. Reading the logs

```bash
python3 src/tools/tools_npz_to_csv.py data/torque_trot/t1.npz --outdir out
```

Two files per run: one row per 4 ms sweep with 110 named columns (units always
in the column name), plus a `_meta.csv` of run scalars. Nothing but NumPy is
needed — no robot, no CAN, no MuJoCo.

- `tools_tau_audit.py` compares `τ_des` against `τ_cmd` against `τ_meas` per
  joint, which separates "the control law was wrong" from "a gate clipped it".
- `tools_swing_analysis.py` does swing-leg forensics: per-swing x excursion,
  pullback, touchdown speed, torque against cap, roll per swing window.

The logs themselves are not in this repository — see
[`data/README.md`](../data/README.md).

## Known limitations / what's next

- **Trot-in-place drifts and does not sustain.** Three of four logged runs
  hit the 12° tilt e-stop within 0.8 s of starting to trot.
- **Walk is slow and also drifts.**
- **The model is quasi-static: no `Iα`, no `ω × Iω`.** There is no
  angular-momentum term available to fight a developing rotation, and the t1/t5
  pair suggests the attitude gains cannot substitute for one.
- **The decisive experiment has not been run.** The roll is consistently
  negative; swapping the lead diagonal would separate "handover asymmetry" from
  "fixed bias" in a single test.
- **A safety trip failed to fire, and the failure mode is silent** — a
  belly-down robot reads level, and the load check does not run in `TROT`. Any
  future gate needs to be able to catch "the floor is holding you up", not just
  "you are tilted".
- The force-distribution QP is hand-rolled with no optimality certificate and
  no bound on solve time.
- Yaw is closed on rate only, because the magnetometer is unusable (chapter 2).
- The 0.84 N·m CoM pitch bias is a static feedforward measured once, not
  estimated online — and the CoM moves as the legs do.
- Leg inertias are assumed, never weighed (chapter 3), and every model-based
  term here inherits that.

## 11. A note on which estimator is used

Chapter 5 built and validated a Bloesch EKF. This chapter's runners take
**attitude from the AHRS and height from FK**, not from that filter. The EKF
is offered back to the MPC track for **velocity only**
(`srb_mpc_hw/ekf_feedback.py`).

That is a deliberate consequence of chapter 5 §4's division of labour, not an
oversight — but it does mean the most sophisticated estimator in the
repository is not in the loop of its most sophisticated controller.
