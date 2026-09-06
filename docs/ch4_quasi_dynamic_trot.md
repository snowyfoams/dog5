# Chapter 4 — Quasi-dynamic torque control: stand, trot, walk

**Status: PARTIAL.** The torque stand works and is repeatable. The trot runs
and the robot stays up, but it **drifts** — in position and in heading — and
the drift has no path back into the loop.

Source: [`src/torque_stand/`](../src/torque_stand) ·
[`src/dog5_trot_quasi_static_model/`](../src/dog5_trot_quasi_static_model) ·
statics in [`src/dog5_description/dog5_statics.py`](../src/dog5_description/dog5_statics.py) ·
log forensics in [`src/tools/`](../src/tools)

This is the first chapter where **software computes the torque**. Chapters 1–3
brought up a bus, a sensor and a model; here the host computes `τ` and sends
it, and everything that gets harder follows from that.

---

## 1. What "quasi-dynamic" means here, precisely

The trunk and all four legs are lumped into **one rigid body**, and
Newton–Euler about its CoM is the entire model:

```
m p̈ = Σ fᵢ − m g ẑ
I ω̇ + ω × (I ω) = Σ rᵢ × fᵢ
```

Then the acceleration terms are sent to zero — `p̈ → 0`, `ω̇ → 0`,
`ω × (Iω) → 0` — and the pair collapses to **pure statics**:

```
Σ fᵢ = m g ẑ            Σ rᵢ × fᵢ = 0
```

Those statics have **no state**. Hand them a wrench and they return foot
forces, every tick, with no memory. They are an **actuator map, not a plant**.

**What the lumping costs, in numbers.** `ω × (Iω)` is **0.094 N·m** at 30 °/s,
and it points almost entirely at yaw. The swinging diagonal pair pushes the
trunk at **26.4 N peak — 46 % of `mg`**. Neither is predicted; both arrive at
the loop as *disturbance*, at about 2.5 Hz, which is above every closed-loop
bandwidth in §4.

That is the honest framing, and §9 argues it is part of why the roll cannot be
corrected. A trot that *travels* wants the inertia terms back; this code is
explicit that it does not have them.

## 2. The stack, in three layers and a clock

Read downwards. Each layer is one block, and nothing skips a layer.

| | layer | frame | rate | what it decides |
|---|---|---|---|---|
| 1 | **wrench** `dynamic_model.py` | world | 83.3 Hz | what force and moment the *trunk* needs |
| 2 | **allocation** `force_totorque.py` | world | 83.3 Hz | how to split it across whichever feet are down |
| 3 | **joint** `stand_torque_mode.py` / `trot_hw.py` | trunk → joint | 250 Hz | transmission, then the impedance floor |
| + | **gait clock** `gait.py` | — | pure function of *t* | which legs are in which branch |

The outer envelope is a stage machine, and it is the demo:

```
CROUCH (0xA4, position) → WAIT → RISE (torque) → HOLD → TROT → PARK
```

**Why the rise is torque-mode.** In position mode the drivers' own loops hold
the pose, so once the robot settles *nothing the upper layer computes has any
observable effect* — you cannot tell a working feedback loop from an ignored
one. Here the trunk is held up by forces this code computes. Push it, and the
recovery **is** the feedback.

**And trot cannot be entered from the crouch**: that would ask a folded robot
to balance on a diagonal.

## 3. The five files, and why the split is where it is

| file | responsibility |
|---|---|
| `crouch_and_park.py` | the `0xA4` bookends. No torque, no model |
| `feedback_estimator.py` | sensors → `(z, v, C, ω)`. Measurement only |
| `dynamic_model.py` | state error → 6-DoF wrench. **The controller** — the only place a gain touches the trunk |
| `force_totorque.py` | wrench → per-foot GRF → 12 joint torques. Pure transmission; no trunk feedback at all |
| `stand_torque_mode.py` | when to do which, at what rate, and every way to stop |

`trot_hw.py` **is** `stand_torque_mode.py` with one stage added. `CROUCH`,
`WAIT`, `RISE` and `HOLD` are that file unchanged — same gains, same
`q_ref_for_height`, same `TorqueGate`, same e-stop set. What is new is `TROT`,
and only `TROT`: the contact clock, and a swing arc.

> A trot that cannot stand first is not a trot problem, and rewriting the part
> that already works is how the previous attempt ended up unable to rise at all.

## 4. The wrench law, and the loop it closes

With `mg` cancelled, the plant is a **double integrator** per axis — two poles
at the origin, no restoring term anywhere. It cannot hold a height, let alone
recover one. So a loop is closed around it, and `Dynamic_Model.body_wrench()`
is that entire choice:

```
F_des = Kp_pos (p_ref − p) + Kd_pos (v_cmd − v) + (0, 0, mg)
M_des = Kp_ori (rpy_ref − rpy) + Kd_ori (0 − ω)
```

Both errors are **plain subtractions**. `rpy_ref = (0,0,0)`: at `start()` the
world frame *is* the body frame, so *level and straight ahead* is the whole
reference.

**`mg` is feedforward, not part of the PD.** A robot sitting exactly at
`z_ref` has zero error, so a pure PD commands zero force — and it falls.
`mg ẑ` sets the operating point and the PD only trims about it, which is also
what makes the plant above a clean double integrator. Verified: level, at
target, at rest, the ask is exactly `(0, 0, 57.046) N`; 10 mm low asks 3.00 N
more.

**Every reference is latched off the robot itself**, not read from a config
file. At `start()`, `z_ref` is the robot's own measured height and `q_ref` its
own crouch pose, so the error every gain sees on the first sweep is *exactly
zero*. Hand it `STAND_HEIGHT` instead, to a robot standing 20 mm lower, and
the wrench steps by `300 × 0.02 = 6.0 N` on the first block, into legs that
are still ramping.

### The gains that ship

From [`torque_stand/params.py`](../src/torque_stand/params.py) and
[`dog5_trot_quasi_static_model/config.py`](../src/dog5_trot_quasi_static_model/config.py).
`config.assert_shared()` refuses to start if the two disagree on a constant the
week-2 modules read internally.

| channel | Kp | Kd | inertia | closed loop |
|---|---:|---:|---|---|
| z | 300 N/m | 40 N·s/m | 5.815 kg | 1.14 Hz, ζ = 0.48 |
| roll | 3.0 N·m/rad | 0.5 N·m·s/rad | 0.0658 kg·m² | — |
| pitch | 3.0 | 0.5 | 0.4075 | — |
| yaw | 3.0 | 0.5 | 0.4572 | — |
| **x, y** | **0** | **0** | | **no loop at all** |

Two of the six channels have **no reference and no gain**, and that is not a
soft tuning: leg odometry measures *velocity* and has no origin, so an
absolute `x, y` never exists to be stiff to. `p_ref[:2] := p[:2]` makes it
explicit rather than accidental. What goes downstream is

```
w = (0, 0, F_z, M_x, M_y, M_z)
```

> **Read this before you trust a gain comment.** The provenance tables in both
> `params.py` and `config.py` name **10 / 0.5** as the attitude pair the
> 2026-08-18 ladder verified, and the live values are **3.0 / 0.5**. The
> stiffness was lowered again afterwards and the tables were not updated. At
> `kp_att = 3.0` a 1.2° standing roll residual is answered with 0.06 N·m of
> the ~6.4 N·m this stance can make — the loop is *barely* pulling. If you
> raise it, do it the way the ladder did: one gain, one logged run, then write
> the run's name into the table.

## 5. Feedback without a motion-capture rig

`feedback_estimator.py` produces `(z, v, C, ω)` from what exists. **There is no
EKF in this loop, and no fused state of any kind.**

- **attitude** — the DETA10's own `0x41` fusion, minus a mount-tilt setpoint.
  Roll and pitch are a *gravity* estimate, valid only while the specific force
  **is** gravity. Under acceleration it is not — which is exactly when a trot
  needs it. Yaw is latched at `start()` and used as a difference.
- **height** — leg FK, from the planted feet. Encoders and attitude only.
- **velocity** — an algebraic **leg-odometry identity**, not a differentiation.
  A planted foot is fixed in the world, so differentiating `pᵢ = r + Cᵀ sᵢ`
  with `pᵢ` constant gives

  ```
  v_world = −Cᵀ (ω × sᵢ + Jᵢ q̇ᵢ)
  ```

  averaged over the planted set. A direct read at 250 Hz: no integration, no
  filter state, nothing that can drift.

**Where `q̇` must come from: the encoder, finite-differenced.** *Not* the
driver's speed field, which arrives in the same reply and is tempting for
exactly that reason. On 2026-08-17 that field reported 8.1 rad/s on a joint
whose encoder had moved 0.31 — and because the identity above multiplies `q̇`
by a Jacobian, a glitch there becomes **344 mm/s of phantom trunk velocity and
24 N of phantom force** through `kd_z`, on a 57 N robot. The trunk shook too
hard to reach `HOLD`.

**What an operator can falsify.** Height and attitude were both dead reckoning
with nothing to check them against until 2026-08-17, and two errors were
sitting there the moment anyone looked:

- **the frame.** `fk_trunk_height` returned floor-to-*hip-axis*, a plane
  nothing physical sits on. It printed 191 mm where a ruler on the trunk
  bottom read ~160; **38 mm of that was the frame.**
- **the mount tilt.** `rp:d = ahrs − fk` compares the AHRS against a
  least-squares plane through the four measured feet, with no IMU in it. It
  printed ~0.5° with the robot standing still — floor slope plus IMU mount
  plus leg zeros, all spent holding the robot off true level.

The runner prints both live. `SETPOINT_ROLL_DEG = −0.29`,
`SETPOINT_PITCH_DEG = 0.12` are the current values;
[the stand runbook](runbooks/DOG5_STAND_RUNBOOK.md) has the procedure for
re-measuring them.

## 6. Allocation: what is built, and what is not

Six equations, twelve unknowns. `A f = w` with the grasp map

```
A = [  I     I     I     I  ]        f = (f₀, f₁, f₂, f₃)
    [[r₀]×  [r₁]× [r₂]× [r₃]×]       w = (F_des, M_des)
```

**`rᵢ` are measured from the CoM**, in world axes, *not* from the trunk origin.
Passing trunk-origin vectors is not an error you can tune out — it is a wrong
model, worth a constant 14.7 mm lever on every vertical force, 0.84 N·m of
pitch bias at a 57 N load.

**What ships is not a QP.** `force_totorque.distribute()` solves the grasp map
by **damped least squares** (`GRASP_LAMBDA = 1e-3`), then **clamps**:
unilateral first (no foot may be asked to pull), the friction cone second
(`μ = 0.6`), then a rescale so the vertical total still adds up.

Clamping after a solve is not the same thing as constraining a solve, and
where the difference shows is a foot near lifting — which is every handover in
a trot. That is the honest limit of this layer. It is also why the contact
ramp (§8) exists: the ramp takes a foot's load down *before* the schedule
lifts it, so the clamp is not the thing handling the transition.

*(A cone-constrained QP with a soft wrench residual is the right answer here
and was written; it has never run on this robot, so it is not in this
repository. See the README's "What is not here".)*

## 7. Statics to joint torque

```
τᵢ = −Jᵢᵀ (C fᵢ)
```

Derived from virtual work: with an external force at the foot,
`f · J δq + τ · δq = 0`. **The minus sign is not optional** — `f` is the
*ground reaction*, the force the floor applies to the foot, which is what the
allocator solves for and why `Af` sums to `+mg`. Writing `+Jᵀf` is a robot
that answers "push up" by driving its feet into the floor.

The rotation is there because the allocator works in **world** axes and `Jᵢ` is
a **trunk-frame** Jacobian; virtual work needs both in the same frame. Four
legs, one rotation each, every model block.

**And then leg gravity.** `dog5_statics.py` documents at length why `−Jᵀf`
alone is wrong: the legs are **3.196 kg of a 5.815 kg robot — 55 % of the
mass**. Treating them as massless linkages that merely transmit a foot force
throws away more than half the robot's weight.

```
τ = −Jᵀ f + leg_gravity(q, ĝ_body)
```

Gated against **MuJoCo floating-base inverse dynamics** to 1.8e-15 N·m, and
against an independent moment sum, and the omission is shown to cost 0.472 N·m
at the recorded crouch — concentrated on **abduction** (62 % of `|Jᵀf|`), where
it *opposes* the `Jᵀf` term, so the error adds rather than cancels.
`python3 src/selftest/test_dog5_statics.py` is that argument, executable.

## 8. The joint layer, the rates, and the swing

### `z_ref` is one number, and it is never stepped

Four boundary conditions — leave the crouch at rest, arrive at the target at
rest — pin exactly one cubic:

```
z_ref(t) = z₀ + Δz (3 t²/T² − 2 t³/T³)
```

For 130 mm over 8 s that is 24.4 mm/s peak, **0.195 mm of target motion per
model block**. A step is not a plan: the same 130 mm applied in one block
lands the foot *outside the workspace*.

`q_ref_for_height()` then solves the IK — damped least squares on the position
error, warm-started from the previous answer. Damped because the Jacobian is
ill-conditioned near full extension; warm-started because the target moved
0.195 mm, so one or two iterations converge.

### The 83.3 / 250 Hz split

Measured on this Pi, not assumed:

| block | cost | rate |
|---|---:|---|
| per-sweep impedance + `TorqueGate` | 28 µs | **250 Hz**, every sweep |
| model block (wrench + grasp map + Jᵀ + IK) | 1384 µs | 83.3 Hz (`MODEL_EVERY = 3`) |
| foot-load check from measured `iq` | 1029 µs | 20.8 Hz (`LOAD_EVERY = 12`) |

The two heavy blocks are **staggered** (`LOAD_OFFSET = 1`) so they never land
in the same sweep: worst single-sweep delay 1.38 ms rather than the 2.41 ms of
both together, well inside the driver's 50 ms input-lost watchdog.

**Only the feedforward is held between updates.** The joint impedance — the
term that actually stabilises a joint — runs every sweep on `q̇` at most 4 ms
old. `KP_IMP = 3.0`, `KD_IMP = 0.1`; the 15 / 0.6 pair it replaced is 68 % of
the sampled-damper bound at the measured 16 ms loop delay and shook this robot
at 9–12 Hz.

### The gait clock

A gait is nothing more than which feet are on the ground, and when. Trot is
the two diagonal pairs taking turns, and the schedule is driven **by time
alone**:

```
φᵢ = (t/T + Δᵢ) mod 1        stance while φᵢ < duty
```

The clock never reads the robot — no contact sensor, no force, no state. Ask
it at time `t` and it answers the same thing every time.

`GAIT_PERIOD = 1.2 s`, `PHASE_OFFSET = (0, ½, ½, 0)` (FL+RR against FR+RL),
`LEAD_DIAGONAL = "fr-rl"`.

> **`DUTY = 0.80` is what ships, and the comment above it argues for 0.60.**
> The constraint the comment states is real — the overlap `duty − 0.5` must
> exceed the ramp length `CONTACT_RAMP × duty`, or the total contact weight
> dips below 2.0 at the crossover and no foot is allowed to carry the robot.
> 0.80 satisfies it with a wide margin: 0.30 of overlap against 0.12 of ramp.
>
> Computed from the shipped constants — `TrotGait` at `T = 1.2 s`,
> `duty = 0.80`, `ramp = 0.15`:
>
> | | |
> |---|---|
> | all four feet down | **0.72 s of every 1.2 s cycle (60 %)**, in two 360 ms windows |
> | swing, per leg | 0.24 s |
> | total contact weight | never leaves [2.0, 4.0] |
>
> That is a substantially more conservative gait than the 0.60 the §9 logs
> were taken at — closer to a walk than to the trot the analysis below
> assumes — and it is a large part of why the demo holds.

### The contact-weight ramp

`contact_weight` is the same clock once more — a smoothstep in phase, 1
through mid-stance, 0 at both ends — and it reaches the robot as a **bound on
that foot's normal force**, with the friction rows taking the tangential force
down with it.

**No force sensor, no load estimate.** The schedule says what a foot is
*allowed* to carry, and the distributor hands the rest to the others. The ramp
lives *inside* stance, not straddling the lift: a foot is already unloaded by
the time the schedule lifts it, which is what makes liftoff free.

Measured: worst handover step **1.768 N·m** with the ramp against **2.208 N·m**
without. Overlapping the gait alone does not fix it — duty 0.50 → 0.65 moved it
only 2.208 → 2.096. What fixes it is scaling the bound itself.

### The swing arc, and why it is two smoothsteps

The chain is **trunk-frame throughout**, and that is the whole design:

1. `swing_foot_body()` returns a trunk-frame point:
   `p_des = (x_crouch, y_crouch, z_site + 40 mm · b(u))`. Only `z` moves.
2. Where the foot *is*, in that same frame, from the encoders alone.
3. A Cartesian spring: `f = kp(p_des − p) + kd(ṗ_des − J q̇)`.
4. `τ = Jᵀ f` — both sides in the trunk frame, so **no rotation appears**.
5. Plus leg gravity; nothing else holds the limb up.

Not one step touches `p_com`, `C`, or any velocity estimate, so **nothing in
it can be got wrong by a bad estimate.** That is a real virtue, and §9 is the
bill for it.

The bump is a *second* smoothstep, up over the first half and down over the
second, so vertical velocity is **zero at liftoff, at the apex and at
touchdown** — measured 0.000 mm/s in all three axes. A sine bump, the usual
shortcut, arrives at `πh/T_sw` = **785 mm/s** straight into the floor.

## 9. What the robot does, and what it does not

### Trot in place holds — and it walks away

`RAIBERT_ON = False` is the verified state, and with it the swing target's
`x/y` is the crouch value **in the trunk frame**. So the print follows the
body wherever the body has gone.

**Nothing in the loop is trying to hold a position.** Rows `x` and `y` of the
wrench are identically zero — zero gain *and* zero error — so in the plane this
controller commands nothing at all. The one horizontal mechanism it has is
*where it puts its feet*, and that is the mechanism not wired in. The drift is
not a tuning failure.

Raibert's correction is what closes it:

```
p_land = p_nom + v·T_st/2 + k_v (v − v_cmd)
```

with `p_nom` the **standing footprint**, not the point under the hip: DOG5's
abduction sits at ~90°, so its nominal foot is **119.6 mm forward** of its own
hip, and the textbook form commands a 120 mm backwards lunge from a standing
start. At `v = 0` both correction terms vanish and `p_land = p_lift` —
straight up 40 mm, straight back down — which is exactly why the shipped
vertical arc is defensible for a trot *in place*, and why it stops being
defensible the instant `v ≠ 0`.

But the gain rides on a **measured** `v`: `T_st/2 + k_v = 0.150 s` of gain on
the estimate, so it is the estimator's *bias*, not its noise, that becomes
displacement.

| `v` bias | step offset | in 10 s |
|---:|---:|---:|
| 5 mm/s | 0.75 mm | 19 mm |
| 10 mm/s | 1.50 mm | 38 mm |
| 20 mm/s | 3.00 mm | 75 mm |

Leg odometry is algebraic and cannot drift in the *integration* sense, but it
is low-passed at 5 Hz and averaged over a contact set that is **scheduled
rather than measured** — a bias is exactly what it is prone to. That is the
argument for a fused estimator, and it is an argument this repository does not
yet answer: **there is no EKF in the loop, and the one that was written has
never run on this robot.**

### The roll, from the 2026-08-18 logs

Four torque-mode runs at `DUTY = 0.60`, 250 Hz, robot weight 57 N.

| run | total | TROT | roll min/max (°) | peak \|gyro-x\| | z min (mm) | Σfz max (N) | kp/kd att | τ_max |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| t1 | 12.3 s | **0.77 s** | −11.24 / −0.80 | 2.23 rad/s | 136 | 63 | 10 / 0.5 | 1.0 |
| t2 | 21.8 s | 6.01 s | −11.98 / +10.49 | 6.80 rad/s | **−53** | **136** | 10 / 0.5 | 3.0 |
| t3 | 11.5 s | **0.41 s** | −9.28 / −0.91 | 2.73 rad/s | 122 | 66 | 0 / 0.1 | 1.5 |
| t5 | 12.1 s | **0.51 s** | −11.44 / −0.74 | 2.16 rad/s | 140 | 61 | 0 / 0.0 | 2.0 |

**1. The roll is real.** t1, t3 and t5 all ended on the 12° tilt e-stop within
**0.4–0.8 s** of entering TROT. Not an IMU spike: the unfiltered gyro `w_x`
independently reads **2.2–2.7 rad/s** through every excursion, against < 1.93°
over 31.5 s of standing. It builds over about two gait cycles, not one sample.

**2. It always goes the same way** — negative, −6 to −12°, never positive.
That is what makes **swapping the lead diagonal the decisive next experiment**:
if the sign follows which diagonal leaves the ground it is the handover; if it
does not, it is a fixed bias (CoM, IMU mount, or leg zeros). `LEAD_DIAGONAL`
exists for exactly this A/B. **That experiment has still not been run.**

**3. t1 versus t5 is nearly a controlled pair** — same period, duty and swing
height, attitude gains 10/0.5 against 0/0. The roll went to −11.24° and
−11.44°. `τ_max` differs so it is not clean, but the attitude loop **did not
visibly change the divergence** — which is what §1 predicts if the missing
physics is the problem rather than the tuning.

### And a safety trip that did not fire

**t2 is what happens when it does not.** `z` goes 149 → **−53 mm** and `Σfz`
to **136 N — 2.4× body weight.** The robot is on its belly. For the next ~7 s
the gyro reads ~0.003 rad/s and roll sits at +0.6°, **because a robot lying
flat reads level.** Tilt cannot see it, and the load check is gated to `HOLD`
and does not run during `TROT`.

Know what each gate can and cannot see. The trips are not a safety net you can
lean on.

### The demo, and what it is

`trot_demo.py` does not solve the roll; it **sidesteps it**. The schedule is
`trot_hw`'s, unchanged, but once every `DEMO_SETTLE_EVERY = 2` full cycles the
gait clock **freezes** for `DEMO_SETTLE_S = 0.2 s` with all four contact
weights at 1, so the distributor spreads force over four feet and the attitude
spring pulls the roll back — a standing re-level — before the next cycle is
released. `DEMO_ALTERNATE_LEAD = True` swaps the two diagonals every cycle, so
whichever asymmetry the handover has does not accumulate in one direction.

`walk_demo.py` is that, plus exactly one thing: every swing's landing point is
displaced `WALK_STEP_M = 20 mm` in x. `T` walks out, `R` walks the **same
number of swings** back (a per-leg ledger — swings, not seconds, so the return
is distance-equal whatever the settles did to the clock). The body follows the
feet, because `q_ref` pins every stance foot's `x/y` at its crouch value and
the joint impedance drags the trunk over a foot that landed ahead. No new
force law, no new gait clock.

## 10. Reading a run

Every runner takes `--log run.npz` and records at the full 250 Hz —
`tau_cmd`, `tau_meas` and `q_ref` alongside `q`, so the impedance error is
recoverable after the fact.

```bash
python3 src/tools/tools_npz_to_csv.py run.npz --outdir out
```

Two files per run: one row per 4 ms sweep with 110 named columns (units always
in the column name), plus a `_meta.csv` of run scalars. Nothing but NumPy is
needed.

- `tools_tau_audit.py` — `τ_des` vs `τ_cmd` vs `τ_meas` per joint, which
  separates "the control law was wrong" from "a gate clipped it".
- `tools_swing_analysis.py` — per-swing x excursion, pullback, touchdown
  speed, torque against cap, roll per swing window.

**The foot-load sum is the number that matters most**, and it is in the exit
report of every run: measured `iq`, inverted through `J⁻ᵀ` with each leg's own
weight removed, and it must read ~57 N. Torque calibration was dropped, so
this is the *only* end-to-end evidence that commanded torque becomes real
force. If it does not add up, the grasp map is fantasy and a good-looking
attitude proves nothing.

The logs themselves are not in this repository — see
[`data/README.md`](../data/README.md).

## Known limitations / what's next

- **The trot drifts in position and heading.** Nothing closes a loop on where
  the body is, because nothing observes it. §9.
- **`RAIBERT_ON = False`**, so the one horizontal mechanism the controller has
  is off. Turning it on needs a velocity estimate whose *bias* is bounded, and
  that is a filter, not a gain.
- **The model is quasi-static: no `Iα`, no `ω × Iω`.** The t1/t5 pair suggests
  the attitude gains cannot substitute for the missing terms.
- **The lead-diagonal A/B has not been run.** It is the cheapest decisive
  experiment available and `LEAD_DIAGONAL` is one edit.
- **Attitude stiffness ships below what any log stands behind** (3.0 against
  the table's 10) and the provenance comments were not updated. §4.
- **`DUTY = 0.80` ships against a comment arguing for 0.60.** The shipped
  demo is a substantially more conservative gait than the logged analysis. §8.
- **Allocation clamps rather than constrains**, so a foot near lifting can be
  asked for force it cannot make. §6.
- **A safety trip failed to fire, and the failure mode is silent** — a
  belly-down robot reads level, and the load check does not run in `TROT`.
- **Contact is asserted by a clock, never measured.** No force sensors, no
  contact switches. That assumption is behind more than one bug in this record.
- Yaw is closed on rate only; the magnetometer is unusable (chapter 2).
- Leg inertias are assumed, never weighed (chapter 3), and every model-based
  term here inherits that.
