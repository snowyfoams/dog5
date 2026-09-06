# DOG5 position-mode stand runbook

[`src/robot_base/stand_dog5_hw.py`](../../src/robot_base/stand_dog5_hw.py) is
the shared hardware layer, and it is also a runner in its own right: it stands
the robot up in **position mode** with no IMU in the loop at all.

Use it for two things:

1. **Verifying the joint map and the directions** after any rewiring. This is
   the safest powered motion in the repository and it is where a wrong sign
   should be found.
2. **A baseline stand** to compare the torque stand against. Here the drivers'
   own loops hold the pose; in [the torque
   stand](../ch4_quasi_dynamic_trot.md) the host computes every newton-metre.

It reads the real encoders, evaluates the unchanged controller from
`dog5_description/stand_dog5.py`, and sends commands through
`motorbus.MotorBus` at 250 Hz per motor. MuJoCo is used only for kinematics,
Jacobians and the gravity bias through `mj_forward`; the hardware entry point
never calls `mj_step`.

**This controller assumes a level trunk.** `HOLD` is leg-frame Cartesian
compliance, not body-attitude stabilisation. It does not know where down is.

---

## The joint map it uses

`HARDWARE_JOINTS` in
[`dog5_description/dog5_hardware_map.py`](../../src/dog5_description/dog5_hardware_map.py),
canonical order `[FL, FR, RL, RR] × [abd, pitch, knee]`:

| leg | abd | pitch | knee | direction |
|---|---:|---:|---:|---:|
| FL | 7 | 8 | 9 | +1, +1, −1 |
| FR | 10 | 11 | 12 | +1, +1, −1 |
| RL | 4 | 5 | 6 | −1, +1, −1 |
| RR | 1 | 2 | 3 | −1, +1, −1 |

It validates itself at import. It is the **only** map in this repository —
three others were live at once in mid-2026 and are described in chapter 3 §7.

## 1. Prepare

Support the robot with its feet clear of the floor. An operator keeps a hand
on **X**.

```bash
sudo bash src/bringup/t1_preflight.sh
python3 src/bringup/t2_ping_scan.py
```

All twelve configured CAN IDs must respond before continuing.

## 2. Zeros, once after assembly

See [DEMO_RUNBOOK §2](DEMO_RUNBOOK.md#2-zeros--only-after-mechanical-work).
`calibration/calibrate12.py --set-zero` writes `0x19`; power-cycle all twelve
motors together afterwards, keep the robot in the zero pose, and re-run
`--verify`. All twelve should pass within ±1 output degree.

## 3. Suspended low-torque check

```bash
python3 src/robot_base/stand_dog5_hw.py --self-test      # offline, first
python3 src/robot_base/stand_dog5_hw.py --tau-max 3.0     # suspended
```

The runner gates before applying torque:

1. It arms all motors while streaming zero torque.
2. Lay the robot in the flat calibration pose and press **SPACE** to capture
   encoder zero.
3. **Move each joint by hand** and watch the printed `FL/FR/RL/RR` angles.
   Confirm the named joint, the direction and the magnitude — all twelve.
4. Return every joint to within 0.12 rad of the captured pose and press
   **SPACE**. It refuses GO otherwise.

Expected motion is `ROLL → FOLD → STAND → HOLD`. **X** or Ctrl-C winds down.

If a displayed joint is wrong, stop and edit that entry in `HARDWARE_JOINTS`.
Never continue to powered motion with an incorrect mapping or sign.

## 4. Floor test

Put the robot on the floor and raise the cap gradually — 3, 5, 6, 7 N·m —
before the configured maximum. The status line shows phase, maximum joint
angle, pose error, current and run-peak joint speed, commanded torque,
temperature, and the input-timeout latch count.

## Safety gates

| gate | value | action |
|---|---:|---|
| torque ramp | 0 → `--tau-max` in 1.0 s | clip |
| hard torque ceiling | 9.0 N·m | clip |
| joint soft limits | abd ±1.75, pitch/knee ±2.6 rad | block outward torque |
| start pose | every joint within 0.12 rad of zero | refuse GO |
| overspeed, hard | above 8.0 rad/s, one sample | stop |
| overspeed, sustained | driver speed > 7.0 rad/s **and** encoder-derived speed > 3.5 rad/s for 3 consecutive checks (~144 ms) | stop |
| temperature | above 80 °C | stop |
| CAN replies | 20 consecutive misses | stop |
| non-watchdog motor fault | any status bit 0–6 | stop |
| input-timeout latch | status bit `0x80` | rate-limited CAN recovery |
| operator | **X** or Ctrl-C | viscous wind-down, then stop all |

**Why overspeed has two tiers.** It is judged once per round-robin (~21 Hz).
The hard tier trusts the driver's reported speed field and stops immediately
above 8.0 rad/s. The sustained tier cross-checks that field against velocity
computed independently from unwrapped encoder positions, because **that field
glitches**: a stuck or noisy report would otherwise end a healthy stage. The
driver must claim > 7.0 rad/s *and* the encoder must show motion above half
that, for three checks. A genuine runaway still trips within ~144 ms, and the
joint-limit e-stop trips instantly regardless.

For scale: peak *commanded* joint speed is about 0.59 rad/s, so the 7.0 rad/s
gate sits ~12× above normal motion. The observed 5.9 rad/s RR_abd report sits
below the default and does not stop the sequence.

That same speed field is why `torque_stand/feedback_estimator.py` reads
**differenced encoders** and never the driver's velocity — chapter 4 §5 has
the incident.
