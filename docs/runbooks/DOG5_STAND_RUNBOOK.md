# DOG5 Hardware Stand Runbook

`dog_stand_compliance_control/dog5_description/stand_dog5_hw.py` runs the
12-motor DOG5 stand-up directly on `can0`. It does not use HIL, UDP, a PC
bridge, or a simulated plant.

The application reads the real encoders and velocities, evaluates the
unchanged controller from `stand_dog5.py`, and sends torque commands through
`motorbus.MotorBus` at 250 Hz per motor. MuJoCo is used only for robot
kinematics, Jacobians, and gravity bias through `mj_forward`; the hardware
entry point never calls `mj_step`.

## Configuration kept by the rewrite

The DOG5-only table `HARDWARE_JOINTS` in `dog5_hardware_map.py` is shared by
the verifier and direct hardware controller; it does not use the HIL map. Its
canonical order is `[FL, FR, RL, RR] × [abd, pitch, knee]`:

| Leg | abd | pitch | knee | direction |
|---|---:|---:|---:|---:|
| FL | 7 | 8 | 9 | +1, +1, -1 |
| FR | 10 | 11 | 12 | +1, +1, -1 |
| RL | 4 | 5 | 6 | -1, +1, -1 |
| RR | 1 | 2 | 3 | -1, +1, -1 |

The controller continues to use the phase times, waypoints, Cartesian gains,
joint gains, foot targets, and feed-forward support force defined in
`stand_dog5.py`. CAN unit conversions still come from `config.py` through
`motorbus.py`.

## 1. Prepare the hardware

Support the robot with its feet clear of the floor. Make sure an operator can
press **X** immediately.

```bash
cd /home/robot01/Documents/can_motor_control
sudo ./t1_preflight.sh
.venv/bin/python t2_ping_scan.py
```

All 12 configured CAN IDs must respond before continuing.

## 2. Calibrate motor zero after assembly

Do this once after assembling or mechanically realigning the dog. Support the
robot and manually place **all 12 joints at model angle 0**, which is the flat
calibration pose.

```bash
.venv/bin/python \
  dog_stand_compliance_control/dog5_description/dog5_zero_calibrate.py
```

The program streams zero torque, samples the stationary pose, shows all 12
encoder values, and waits for **W** before sending firmware command `0x19` once
per motor. This command writes the motor encoder offset; the vendor warns not
to use it frequently. A receipt is saved beside the script.

After it reports COMPLETE, power-cycle all 12 motors together. Keep the robot
in the same zero pose and run the read-only checker:

```bash
.venv/bin/python \
  dog_stand_compliance_control/dog5_description/dog5_zero_check.py
```

The checker never sends `0x19` and does not alter the current calibration. All
12 joints should pass within ±1 output degree. The stand runner's later
flat-pose capture is a per-session software safety reference; it does not write
the motor calibration again.

## 3. Run a suspended low-torque check

```bash
.venv/bin/python \
  dog_stand_compliance_control/dog5_description/stand_dog5_hw.py \
  --tau-max 3.0
```

The application performs these gates before applying torque:

1. It arms all motors while streaming zero torque.
2. Lay the robot in the flat calibration pose and press **SPACE** to capture
   encoder zero.
3. Move each joint by hand and watch the printed `FL/FR/RL/RR` angles. Confirm
   the named joint, direction, and magnitude are correct.
4. Return all joints to within 0.12 rad of the captured flat pose. Press
   **SPACE** to start; the application refuses GO if a joint is not near zero.

Expected motion is ROLL, FOLD, STAND, then a compliant HOLD. Press **X** or
Ctrl-C to wind down and stop.

If a displayed joint is wrong, stop and edit that entry's `can_id` or
`direction` in `HARDWARE_JOINTS`. Never continue to powered motion with an
incorrect mapping or sign.

## 4. Floor test

After the suspended check is correct, put the robot on the floor and increase
the cap gradually:

```bash
# Repeat at 5, 6, and 7 N*m before using the configured 8 N*m maximum.
.venv/bin/python \
  dog_stand_compliance_control/dog5_description/stand_dog5_hw.py \
  --tau-max 5.0

# Default hardware run: 8 N*m.
.venv/bin/python \
  dog_stand_compliance_control/dog5_description/stand_dog5_hw.py
```

The status line shows phase, maximum joint angle, pose error, current and
run-peak joint speed, commanded torque, temperature, and input-timeout latch
count.

## Safety gates

| Gate | Value | Action |
|---|---:|---|
| Torque ramp | 0 to `--tau-max` in 1.0 s | clip |
| Hard torque ceiling | 9.0 N·m | clip |
| Joint soft limits | abd ±1.75, pitch/knee ±2.6 rad | block outward torque |
| Start pose | every joint within 0.12 rad of zero | refuse GO |
| Overspeed (hard) | above 8.0 rad/s, one sample | stop |
| Overspeed (sustained) | driver speed above 7.0 rad/s and encoder-derived speed above 3.5 rad/s for 3 consecutive checks (~144 ms) | stop |
| Temperature | above 80 °C | stop |
| CAN replies | 20 consecutive misses | stop |
| Non-watchdog motor fault | any status bit 0–6 | stop |
| Input-timeout latch | status bit `0x80` | rate-limited CAN recovery |
| Operator | **X** or Ctrl-C | viscous wind-down, then stop all |

Overspeed is judged once per round-robin (~21 Hz). The hard tier uses the
driver's reported speed field directly and stops immediately above 8.0 rad/s.
The sustained tier cross-checks that field against velocity independently
calculated from unwrapped encoder position changes. This prevents a noisy or
stuck speed field from ending a healthy stage: the driver must report above
7.0 rad/s and the encoder must show motion above half that threshold for three
checks. A genuine runaway still trips within about 144 ms, and the joint-limit
e-stop still trips instantly regardless. Tune with `--qd-estop` and
`--qd-estop-hard` (both capped at 12 rad/s); compare `max|qd|` with
`max|qd_enc|` in the status line before changing them. For reference, peak
*commanded* joint speed is about 0.59 rad/s, so the 7.0 rad/s gate sits about
12x above normal motion. The observed 5.9 rad/s RR_abd report is below this
default and does not stop the sequence.

The controller assumes a level trunk because no IMU is connected. HOLD is
leg-frame Cartesian compliance, not active body-attitude stabilization.
