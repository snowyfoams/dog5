# DOG5 demo runbook — crouch, stand, trot, walk

The procedure behind `trot_video/trot.MOV`, in the order it has to be done.
One runner does the whole thing:
[`src/dog5_trot_quasi_static_model/walk_demo.py`](../../src/dog5_trot_quasi_static_model/walk_demo.py).

> **This moves a 5.8 kg machine under torque control.** An operator keeps a
> hand on `SPACE` for the whole run. Read [chapter 4
> §9](../ch4_quasi_dynamic_trot.md#and-a-safety-trip-that-did-not-fire) first —
> a tilt trip has failed to fire on this robot, and the failure was silent.

---

## 0. Offline, before you touch the robot

```bash
python3 src/selftest/test_all.py
```

Thirteen suites, ~30 s, nothing plugged in. If this is not green, do not
power anything.

## 1. Bus

```bash
sudo bash src/bringup/t1_preflight.sh          # can0 up: 1 Mbit/s, restart-ms 100,
                                               # txqueuelen 1000
python3 src/bringup/t2_ping_scan.py            # all twelve IDs must answer
```

`t1_preflight.sh` also prints the one-time **termination check** — power off,
measure CAN_H to CAN_L, expect ~60 Ω. Do it once per harness, not per session.

If a motor does not answer, `src/bringup/` has the rest of the ladder:
`t3_rate_sweep` (does it hold 250 Hz per motor), `t4_watchdog_trip`,
`t6_unplug_multi`, `t7_recover_no_powercycle` (recovery without a power cycle).

## 2. Zeros — only after mechanical work

Skip this if nothing has been disassembled. `0x19` writes flash and frequent
use shortens driver life.

```bash
python3 src/calibration/calibrate12.py --observe     # live table, all limp
python3 src/calibration/calibrate12.py --verify      # check the zero pose
python3 src/calibration/calibrate12.py --set-zero    # writes 0x19
```

All twelve joints at model angle 0 is the **flat calibration pose** — legs
straight out sideways. Arrange it by hand; the robot stays back-drivable
throughout. `--set-zero` takes effect only after a power cycle. One motor at a
time: `calibrate_leg.py --ids 1,2,3` or `setzero_one.py`.

**Note the two encoder conventions.** `calibrate12.py` computes
`motor_output = encoder × 360/65535` with **no gear divide**, while the robot
code divides by `GEAR = 10`. They were never unified — see chapter 3 §7. Do
not mix the numbers.

## 3. Joint map and direction — after any rewiring

```bash
python3 src/robot_base/stand_dog5_hw.py --self-test    # offline
python3 src/robot_base/stand_dog5_hw.py --tau-max 3.0  # suspended, feet clear
```

Move each joint by hand and watch the printed `FL/FR/RL/RR` angles. Confirm
the **named joint, the direction and the magnitude** for all twelve. If one is
wrong, stop and fix that entry's `can_id` or `direction` in
`dog5_description/dog5_hardware_map.py`.

**Never continue to powered motion with an incorrect mapping or sign.** Four
conflicting joint maps were live at once in mid-2026 and none of them
announced itself as wrong — each was found by a robot moving the wrong leg.

## 4. The resting attitude — once per floor

The AHRS reading with the robot standing still is floor slope **plus** IMU
mount tilt **plus** leg zeros, and it must be subtracted or the attitude loop
spends its authority holding the robot off true level.

```bash
cd src
sudo HOME=$HOME chrt -f 50 python3 dog5_trot_quasi_static_model/walk_demo.py
```

Let it reach `CROUCH`, read the `rp:` block it prints, press `X`, and write
`rp:d` into `SETPOINT_ROLL_DEG` / `SETPOINT_PITCH_DEG` in
`torque_stand/params.py`.

Current values: `−0.29` / `0.12`. **A different floor moves them.**

To separate mount tilt from floor slope: rotate the robot 180° on the *same*
patch of floor and repeat. The part that flips sign is the floor.

## 5. The run

Suspended first, then on the floor.

```bash
cd src
sudo HOME=$HOME chrt -f 50 python3 dog5_trot_quasi_static_model/walk_demo.py
```

`sudo` is for real-time priority; `HOME=$HOME` is because the IMU SDK is
resolved relative to the invoking user's home and `sudo` would otherwise make
that `/root`.

| key | does |
|---|---|
| `ENTER` | `CROUCH` → `RISE`, and re-engage from `LIMP` |
| `T` | `HOLD` → `TROT`, and back. **The exit is latched** and taken at the next sweep with all four feet scheduled down, so no leg is abandoned mid-swing |
| `R` | walk the outbound swings back, then return to `HOLD` by itself |
| `P` | park — **from `HOLD` only**. Deliberately dead while trotting: `0xA4` mid-swing is a lurch onto a diagonal |
| `SPACE` | `LIMP`. Does *not* leave the `TROT` stage |
| `X` | stop. Cuts torque at height, at any instant |

Add `--park-on-stop` if you want `X` to put the robot down in position mode
rather than handing you the weight.

### The order that matters

1. **Leave `TAU_START_MAX` at 1.0** for the first run after any edit. Raise
   toward `TAU_STAGED_MAX = 3.0` only after a quiet `HOLD`.
2. **Do not press `T` until `HOLD` is quiet.** A trot entered from a shaking
   stand is a stand problem, not a trot problem.
3. **Do not zero `FORCE_FRAC_DEFAULT` with the feet on the floor.** It scales
   the distributed ground reaction — the only term holding the trunk up. At 0
   the legs fold until the impedance's positional error balances 57 N, about
   90 mrad per joint; seven joints latched within 2 s of `RISE` when this was
   tried on 2026-08-14.

### Watch these two numbers

The status line prints at 2 Hz and carries what a human holding the robot can
act on:

- **`zI`** — floor to **trunk bottom**, the point a ruler reaches. `(H…)`
  beside it is the hip axis, which is where the leg tables and the IK work.
  They differ by 38 mm, and confusing them was the 2026-08-17 frame bug.
- **`rp:d`** — `ahrs − fk`, the AHRS against a least-squares plane through the
  four measured feet. **The only independent check on the AHRS**, and the
  reason §4 exists.

Everything else — `|tau|`, tracking, `|dq|` — is in the `.npz` at the full
250 Hz, which is where it was actually read from when the 2026-08-17 shake was
diagnosed.

**The exit report carries the foot-load sum**, and it is the number that
matters most: measured `iq`, inverted through `J⁻ᵀ` with each leg's own weight
removed. **It must read ~57 N.** Torque calibration was dropped, so this is
the only end-to-end evidence that a commanded newton-metre becomes a real one.
If it does not add up, the grasp map is fantasy and a good-looking attitude
proves nothing.

## 6. Parameters are edits, not flags

Changed 2026-08-21. Open `dog5_trot_quasi_static_model/config.py`, change the
constant, save, re-run the same one line.

Every run logs itself to `run_<date>_<time>.npz` and the log embeds
`config.snapshot()` — every constant as imported, plus `config.py`'s own source
text. **The log says what flew**, so a flag that shadows `config.py` would make
the snapshot lie. `--tau-max` is the one tuning flag left; the rest are
plumbing (`--port`, `--log` / `--no-log`, `--park-on-stop`, `--self-test`).

`mv` a run worth keeping to a real name, and promote its values into
`config.py`'s provenance tables.

### One gain per run

The 2026-08-18 ladder is the model: one gain, one logged run, then write the
run's name into the table beside the value. The gait knobs are `GAIT_PERIOD`,
`DUTY`, `SWING_HEIGHT` — move **one** per run.

Two experiments that are one edit each and have not been done:

- **`LEAD_DIAGONAL = "fl-rr"`.** The roll went the same way in every logged
  run. If the sign follows the lead diagonal it is the handover; if it does
  not, it is a fixed bias. This is the cheapest decisive experiment available.
- **`RAIBERT_ON = True`.** The only horizontal mechanism this controller has.
  It rides on the measured velocity, so read chapter 4 §9's bias table first.

## Safety gates, and what each can see

| gate | value | action |
|---|---:|---|
| torque ramp | 0 → cap in `TORQUE_RAMP_S` = 0.3 s | clip |
| slew | 60 N·m/s | clip |
| staged cap | `TAU_STAGED_MAX` = 3.0 N·m | clip |
| hard ceiling | 9.0 N·m (`iq` saturates at 9.94) | clip |
| tilt | `TILT_STOP_DEG` = 20° | limp |
| foot-load sum | off by more than `LOAD_SUM_TOL_FRAC` = 40 % | limp |
| overspeed, temperature, CAN misses | see `stand_dog5_hw.py` | stop |
| operator | `SPACE` / `X` | limp / stop |

**Joint soft limits are off** in the torque runners, as in `crouch_and_park`:
the symmetric ±2.6 rad box sits 6.9° from the resting crouch on the knees while
allowing a knee 298° of travel. An **abduction sign check** replaces it — no
leg may swing through the body.

**The load check does not run during `TROT`**, and tilt cannot see a robot
lying flat. Both were true in run t2, where `z` went 149 → −53 mm and the
vertical force reached 136 N — 2.4× body weight — while the attitude read
level.
