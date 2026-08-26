# DOG5 Hardware Crawl Runbook

`dog_stand_compliance_control/dog5_description/crawl_dog5_hw.py` brings DOG5
from its current pose to the recorded crouch, stands it with Cartesian
compliance, and then runs a finite, statically stable crawl on real CAN
hardware.

The filename is **`crawl_dog5_hw.py`** (with `crawl`, not `craw`). The program
is interactive and must be run from a real terminal. It does not use MuJoCo as
a simulated plant; MuJoCo/NumPy routines are used only for model calculations
such as forward kinematics and Jacobians.

> **Hardware warning:** Mechanically support or tether the robot for initial
> tests, keep the feet clear when requested, and keep a hand ready to press
> **X**. Do not disable a safety gate merely to make a failing motion continue.

## 1. Prepare and verify the hardware

From the repository root:

```bash
cd /home/robot01/Documents/can_motor_control
sudo ./t1_preflight.sh
.venv/bin/python t2_ping_scan.py
```

All 12 configured CAN IDs must respond. Motor zero calibration and joint
directions must already have been verified as described in
[`DOG5_STAND_RUNBOOK.md`](DOG5_STAND_RUNBOOK.md).

Run the offline validation before opening CAN:

```bash
.venv/bin/python \
  dog_stand_compliance_control/dog5_description/crawl_dog5_hw.py \
  --self-test
```

The self-test checks the planned stand and crawl without commanding motors. It
tests inverse-kinematics reachability, joint limits, Jacobian conditioning,
support margin, phase transitions, abort recovery, and estimated static
torque requirements.

## 2. Recommended test progression

First validate only part of the stand while the robot is supported:

```bash
.venv/bin/python \
  dog_stand_compliance_control/dog5_description/crawl_dog5_hw.py \
  --tau-max 1.0 --travel-scale 0.25
```

This intentionally cannot crawl. A crawl requires `--travel-scale 1.0` and a
fully settled `HOLD`.

Next isolate the leg swing from forward walking and pause at every crawl phase
boundary:

```bash
.venv/bin/python \
  dog_stand_compliance_control/dog5_description/crawl_dog5_hw.py \
  --tau-max 3.0 --swing-test --observe-phases
```

`--swing-test` forces the forward step length to zero. The body still performs
`SHIFT` and `RECENTER`, because those motions are required to unload a foot
safely.

After the zero-step motion is understood and validated, test one forward gait
cycle:

```bash
.venv/bin/python \
  dog_stand_compliance_control/dog5_description/crawl_dog5_hw.py \
  --tau-max 3.0 --step-length 0.03 --observe-phases
```

Remove `--observe-phases` only after every phase has been checked. The default
command is equivalent to a 30 mm step, 20 mm lift, one four-leg gait cycle,
and a 3 second time slot per leg:

```bash
.venv/bin/python \
  dog_stand_compliance_control/dog5_description/crawl_dog5_hw.py
```

## 3. Live keyboard commands

The keys are read without requiring an extra Enter, except that the Enter key
itself is used to approve state changes.

| Key | Meaning | When accepted |
|---|---|---|
| **Enter** | Approve the next motion or resume a paused crawl phase | At the zero-torque check, recorded-crouch hold, full stand hold, diagonal-pair hold, or an `--observe-phases` pause |
| **P** | Park from a stationary Cartesian hold back toward the recorded crouch | In `HOLD`, `CRAWL_HOLD`, `HOLD_PARTIAL`, or `HOLD_SAG`; refused during `CRAWL` |
| **X** | Stop the run | At any time |
| **Ctrl-C** | Interrupt the run | At any time |

Enter is refused if the relevant pose, speed, tracking, touchdown, or motor
health checks do not pass. Read the `[stage]` message instead of pressing it
repeatedly.

## 4. Operator sequence

The normal macro-level sequence is:

```text
zero-torque preflight
        |
        | Enter, while supported and stationary
        v
CURRENT -> CROUCH -> WAIT_CROUCH
                           |
                           | Enter after inspecting the recorded crouch
                           v
                       STAND -> WAIT_STAND -> HOLD
                                                  |
                                                  | Enter
                                                  v
                                               CRAWL
                                                  |
                                                  v
                                             CRAWL_HOLD
                                              |       |
                                  Enter repeats       P parks
```

Detailed behavior:

1. The bus arms while streaming zero torque.
2. The terminal displays current joint angles and errors relative to the
   recorded crouch. Support the robot, hold it still, and press **Enter**.
3. Native motor position control moves `CURRENT -> CROUCH`. This stage has
   separate measured-torque and encoder-speed trips because the Cartesian
   torque cap is not the active control mechanism yet.
4. At `WAIT_CROUCH`, inspect the settled crouch and press **Enter** to start
   the Cartesian stand.
5. `STAND` maintains its final target in `WAIT_STAND` until Cartesian error
   and encoder speed settle. A full successful stand enters `HOLD`.
6. Press **Enter** in `HOLD` to start a finite crawl batch.
7. The gait order is `RR -> FL -> RL -> FR`. The program always pauses on all
   four feet after each diagonal pair, so press **Enter** after inspecting the
   pair. With `--observe-phases`, it also pauses at every individual phase
   boundary.
8. A completed or gracefully aborted batch enters `CRAWL_HOLD`. Press
   **Enter** to start a fresh batch, **P** to park, or **X** to stop.

If the stand cannot settle within 20 seconds, it enters `HOLD_SAG`. The hold
target is rebased to the achieved foot heights, crawl remains locked out, and
only parking or stopping is allowed.

## 5. Crawl phase meaning

Each leg uses this phase sequence:

```text
SHIFT -> UNLOAD -> LIFT -> SWING -> LOWER -> LOAD -> RECENTER
```

| Phase | Default nominal time | Meaning and completion check |
|---|---:|---|
| `SHIFT` | 0.60 s | Move the trunk inward from the nearest limiting edge of the three-foot support triangle. Liftoff needs at least 15 mm measured support margin, no more than 25 mm Cartesian error, and low joint speed. |
| `UNLOAD` | 0.30 s | Keep the foot planted while fading its gravity-support share from `mg/4` to zero. Pitch/knee feedback torque and relative leg sag must prove that the foot is unloaded. |
| `LIFT` | 0.45 s | Raise only the swing foot by `--swing-height`; each of the other three feet receives an `mg/3` support share. |
| `SWING` | 0.30 s | Move the raised foot forward by `--step-length`. |
| `LOWER` | 0.45 s | Lower the foot. Encoder forward kinematics must place it close to the plane through the other three feet and show that it is nearly stationary before touchdown is accepted. |
| `LOAD` | 0.30 s | Restore the landed foot from zero support to `mg/4` while the other legs return from `mg/3` to `mg/4`. |
| `RECENTER` | 0.60 s | Move the trunk to the next neutral body position while all four conceptual ground anchors remain fixed. Tracking and speed must settle before the next step. |

The nominal times sum to one `--leg-cycle` slot. Tracking gates can extend a
phase beyond its nominal duration, and operator pauses add unlimited time.
With the 3 second default, a four-leg cycle has 12 seconds of commanded motion
plus gate settling and operator holds.

For a 30 mm step, every foot moves forward 30 mm during its turn, while the
neutral body target advances 7.5 mm after each leg. After all four legs have
stepped, the body has advanced 30 mm and the trunk-frame foot layout is again
the original neutral stand layout.

### Gate failure versus emergency stop

A `SHIFT`, `UNLOAD`, `LOWER`, or `RECENTER` timeout starts a controlled abort:
the swing leg is loaded where it stands, the body returns to a four-foot
neutral stance, and the batch ends in `CRAWL_HOLD` with an `ABORTED` message.

A real loss of the support triangle during reduced support, an ordinary
safety-gate e-stop, **X**, or Ctrl-C stops the run. A negative measured support
margin for three consecutive checks is treated as support-triangle loss.

## 6. Terminal option reference

Distances are in metres, torque is in N·m, joint speed is in rad/s, and motor
position speed is in motor-degrees/s.

| Option | Default | Allowed values | Meaning |
|---|---:|---:|---|
| `--self-test` | off | flag | Run offline checks and do not open CAN. |
| `--tau-max` | `3.0` | `> 0` to `3.0` | Per-joint Cartesian torque cap. The runner intentionally limits this to the staged-test maximum. |
| `--cart-gain-scale` | `0.7` | `> 0` to `1.0` | Scale Cartesian proportional and derivative gains. Lower is softer and usually tracks less tightly. |
| `--support-scale` | `0.65` | `0` to `1.0` | Fraction of modeled gravity feedforward; the slow support-trim integrator supplies the remaining correction. |
| `--stand-height` | `0.15` | `0.10` to `0.21` | Hip-to-foot height used for the full stand and crawl. |
| `--travel-scale` | `1.0` | `0.05` to `1.0` | Fraction of the crouch-to-stand path. Values below `1.0` are stand-only tests and cannot crawl. |
| `--shift-distance` | `0.026` | `0.02` to `0.05` | Magnitude of the adaptive pre-liftoff body shift. It is directed inward, perpendicular to the currently limiting support-triangle edge. |
| `--step-length` | `0.03` | `0` to `0.06` | Forward placement of each foot. Zero performs the gait in place. |
| `--swing-test` | off | flag | Force `--step-length 0`, even if another step length was supplied. |
| `--observe-phases` | off | flag | Pause safely at every crawl phase boundary until **Enter**. Mandatory diagonal-pair holds still occur without this option. |
| `--swing-height` | `0.020` | `0.015` to `0.05` | Vertical swing-foot clearance. |
| `--unload-tau-trip` | `0.45` | `0.1` to `2.0` | Maximum absolute measured pitch/knee torque for the pre-lift unload gate. Raising it makes the unloaded-foot proof less conservative. |
| `--leg-cycle` | `3.0` | `2.0` to `4.0` | Nominal seconds assigned to all seven phases for one leg. |
| `--gait-cycles` | `1` | `1` to `20` | Number of four-leg cycles started by one Enter press. |
| `--crouch-max-speed-dps` | `100` | `1` to `600` | Motor-side speed cap used by native position control for `CURRENT -> CROUCH`. With the configured 10:1 reduction, 100 motor-deg/s is about 10 output-deg/s. |
| `--crouch-torque-trip` | `2.0` | `0.1` to `3.0` | Measured joint-torque emergency trip during native crouch positioning; it is not an active position-command torque cap. |
| `--crouch-speed-trip` | `1.0` | `0.1` to `3.0` | Encoder-derived joint-speed trip during native crouch positioning. |
| `--qd-estop` | `7.0` | `> 0` to `< --qd-estop-hard` | Sustained, encoder-confirmed driver-speed threshold. |
| `--qd-estop-hard` | `8.0` | `> --qd-estop` to `12.0` | Hard overspeed threshold with the controller's confirmation rules. |
| `--no-overspeed` | off | flag | **Dangerous:** disable both overspeed tiers. Joint-limit, temperature, CAN-miss, and motor-fault stops remain active. |
| `-h`, `--help` | — | flag | Print the script description and option summary. |

Useful combinations:

```bash
# Zero forward travel, explicit spelling instead of --swing-test
.venv/bin/python dog_stand_compliance_control/dog5_description/crawl_dog5_hw.py \
  --step-length 0.0 --observe-phases

# Two complete four-leg gait cycles per batch
.venv/bin/python dog_stand_compliance_control/dog5_description/crawl_dog5_hw.py \
  --gait-cycles 2

# Slower motion with a smaller forward step
.venv/bin/python dog_stand_compliance_control/dog5_description/crawl_dog5_hw.py \
  --leg-cycle 4.0 --step-length 0.02 --observe-phases
```

## 7. Reading status output

The `[crawl]` status line contains:

| Field | Meaning |
|---|---|
| `mode` | `WALK` or `SWING_TEST` (zero forward travel). |
| `stage` | Macro state such as `STAND`, `HOLD`, `CRAWL`, or `CRAWL_HOLD`. |
| `phase` / `motion` | Current crawl phase and its category (`BODY_MOTION`, `WEIGHT_TRANSFER`, `LEG_SWING`, or `TOUCHDOWN_LOAD`). |
| `paused` | Whether Enter is required to continue. |
| `swing`, `step` | Active leg and current step number in the batch. |
| `margin` | Encoder-FK distance from the assumed CoM projection to the nearest support-triangle edge. Positive is inside. During `UNLOAD`, the field also shows swing-leg torque and relative sag. |
| `cart_err` | Largest foot Cartesian tracking error. |
| `sigma_min` | Smallest leg-Jacobian singular value; small values indicate poor conditioning. |
| `force_max` | Largest requested Cartesian foot-force magnitude. |
| `max|qd_enc|`, `max|qd_driver|` | Encoder-derived and driver-reported joint speeds. |
| `max|tau|`, `max|tau_fb|` | Largest commanded and measured joint torques. |
| `Tmax`, `latched` | Highest motor temperature and count of input-timeout latches. |

The following `[legs]` line reports support-trim commands, a torque/Jacobian
estimate of per-leg vertical load, and the load-distribution estimate of CoM
xy. These are estimates, not force-sensor or IMU measurements.

## 8. Code flow

The important classes have separate responsibilities:

| Component | Responsibility |
|---|---|
| `CrawlSequence` | Macro state machine: crouch, stand, hold, crawl, crawl hold, and park. It decides whether Enter or P is legal. |
| `CrawlGaitPlanner` | Per-leg phase state machine, ground-anchor bookkeeping, adaptive body shift, support and touchdown gates, pauses, and graceful abort recovery. |
| `CrawlController` | Converts each planned foot target, velocity, support share, and trim scale into 12 joint torques using Cartesian compliance. |
| `CrawlSafetyGate` | Applies torque ramp/cap/slew and joint-limit protection, and evaluates e-stop conditions. It also implements the explicit `--no-overspeed` override. |
| `MotorBus` | Polls encoder/status data and sends round-robin native-position or torque commands to the 12 CAN motors. |

Top-level call flow:

```text
main()
  -> build_parser()
  -> validate argument ranges
  -> offline_self_test(args)                         [--self-test]
     or
  -> run_hardware(args)
       -> construct CrawlController
       -> validate_crawl_configuration()             [before CAN opens]
       -> construct CrawlGaitPlanner + CrawlSafetyGate
       -> open KeyPoller + MotorBus
       -> arm with zero torque
       -> zero_torque_preflight()                    [wait for Enter]
       -> construct CrawlSequence
       -> repeated hardware loop
            -> poll encoders, speed, torque, status
            -> sequence.update(...)
                 -> planner.update(...) during CRAWL
            -> choose controller by macro stage
                 CROUCH: native position command
                 STAND/HOLD/PARK: inherited stand controller
                 CRAWL: CrawlController.compute_crawl()
            -> safety_gate.apply(requested_torque)
            -> evaluate e-stops and support margin
            -> handle Enter / P / X
            -> send the next motor command
       -> damped soft stop, then motor STOP
```

During `CRAWL`, `CrawlGaitPlanner.target()` supplies each leg with:

```text
(desired foot position,
 desired foot velocity,
 gravity-support share,
 support-trim scale)
```

`CrawlController.compute_crawl()` then performs, per leg:

```text
foot_position = FK(q)
foot_velocity = J(q) * qd
force = Kp * position_error + Kd * velocity_error
force_z -= support_scale * robot_weight * support_share
force_z -= trim_scale * vertical_trim
joint_torque = J(q).T * force - joint_damping * qd
```

The force is bounded before conversion to torque. The resulting joint torque
then passes through the safety gate before it reaches the motor bus. The swing
leg's vertical trim is frozen from `UNLOAD` through `LOAD`; only the planted
stance legs integrate trim while the robot is on three-leg support.

## 9. Important limitations

- There is no IMU, force sensor, or contact switch. The geometric support
  calculation assumes the CoM projection is the trunk-frame origin.
- The `UNLOAD` torque/sag gate is a physical cross-check for that assumption,
  but measured joint torque includes unmodeled gear friction.
- Touchdown is inferred from encoder kinematics relative to the plane through
  the other three feet; it is not directly sensed.
- Cartesian compliance does not actively stabilize trunk attitude.
- `--no-overspeed` removes protection against a real runaway and should not be
  used as a routine tuning option.
