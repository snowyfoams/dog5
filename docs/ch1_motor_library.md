# Chapter 1 — The motor library

**Status: PASSED.** Twelve motors on one CAN bus at 250 Hz *per motor*, with
fault recovery that never needs a power cycle.

Source: [`src/motor/`](../src/motor) · demos in
[`src/motor_demos/`](../src/motor_demos) · bring-up evidence in
[`src/bringup/`](../src/bringup)

---

## 1. The hardware

Twelve LK / K-TECH brushless servo drivers, CAN IDs 1–12, on one `can0` bus at
1 Mbit/s. Each driver runs its own position, speed and torque loops in
firmware; the host sends setpoints and reads state. A driver replies on
arbitration ID `0x140 + id`, so replies are self-identifying — a fact chapter 1
turns out to depend on entirely.

```python
MOTOR_IDS  = list(range(1, 13))
REPLY_BASE = 0x140
CAN_IFACE  = "can0"
BITRATE    = 1_000_000
IQ_HARD    = 2048          # protocol max |iq| for 0xA1 / 0xA2
```

Each joint sits behind a **10:1 reduction**, and the encoder is on the *motor*
shaft, before the gearbox. `motor_demos/encoder_compare.py` demonstrates this
directly: spin the output through 360° and watch the encoder wrap ten times.
The consequence — a single-turn encoder means the output angle is only known
modulo 36° — is chapter 3's problem, not chapter 1's.

## 2. Two libraries, and why there are two

### `motor_library.LKMotor` — the original, one motor at a time

A direct wrapper of the driver protocol: send a command, block, read the reply.
It is readable and it is correct **for one motor**.

It is not safe on a shared bus, and the reason is worth understanding because
it is the whole argument for the second library. `_receive_response()` reads
frames until it sees one whose arbitration ID matches *this* motor — and
**discards every frame that does not match**. Point two `LKMotor` objects at
one bus and each one silently eats the other's replies. With twelve, telemetry
is nonsense.

It is still here, and still imported by all of `src/bringup/` and most of
`src/motor_demos/`, because those are single-motor tools where the blocking
API is the clearer one. Its own docstrings point at `MotorBus` in three
places.

### `motorbus.MotorBus` — the one the robot runs on

Round-robin, non-blocking, **one outstanding request per motor**, replies
matched by arbitration ID. That is the only primitive that works at
twelve-motor rates.

```
MotorRecord    per-motor state: last command, last reply, miss counter
RoundRobinBus  reset_stats · send · poll · wait_reply · finalize · bus_load
MotorBus       arm · torque · speed · position · keepalive · stop · hold
               poll · slot · pace · recover · flush_rx
               status1 · status2 · status1_req · clear · set_zero_all
               temps · errors · voltages · speeds_dps · torques_nm
               encoders_deg · offsets · stop_all · close
```

`MotorBus` is a context manager, and everything it exposes is in engineering
units — torque in N·m, speed in deg/s, position in degrees at the **output**
joint — with the gear ratio and the LSB scaling already applied.

```python
import motorbus

with motorbus.MotorBus() as bus:
    bus.arm()                                  # power-on watchdog handled
    for _ in bus.slot(250.0):                  # 250 Hz PER MOTOR
        bus.position(leg_targets_deg)
        q = bus.encoders_deg()
```

## 3. `slot(rate_hz)` is per motor

The single most consequential API detail in this repository.

`MotorBus.slot(250.0)` means **each of the twelve motors** is commanded at
250 Hz. The sweep period is 4 ms, each motor's CAN slot is 333 µs, and the
aggregate frame rate is **3000 frames/s**, not 250.

Older comments elsewhere in this project state that the per-joint rate is
~20.8 Hz — that is 250/12, and it is **wrong**. The error is a factor of
twelve. It mattered: the torque-control track was abandoned in July 2026 on the
argument that ~20 Hz could not stabilise a leg, and was restarted only after
the rate was corrected (commit `c97f1f1`). At the true 4 ms sweep the
sampled-damper bound `kd < 2J/dt` is 4.4 N·m·s/rad on the knee rather than
0.37 — which is the whole reason a joint-space PD is possible at all here.
[`torque_stand/params.py`](../src/torque_stand/params.py) opens with that
derivation so nobody re-derives 20.8 Hz.

## 4. The bus budget

Twelve motors × 400 Hz × 2 frames (command + reply) = **9600 frames/s**.
Measured capacity on this bus is roughly **8000–8300 frames/s**. So 400 Hz per
motor does not fit, and no amount of tuning makes it fit.

`bringup/t3_rate_sweep.py` measures the real ceiling rather than assuming it.
It interleaves slots evenly instead of bursting per cycle — so each motor's gap
*is* its command period — and passes only on: zero misses, zero send failures,
max gap < period + 2 ms, p99 latency < 5 ms, and no growth in the SocketCAN
error counters. The answer is ~250–330 Hz per motor, which is why the robot
runs at 250.

## 5. `motor_gains.py` is not the robot's configuration

`src/motor/motor_gains.py` (48 lines) holds unit conversions and direction
signs for a one- or two-motor bench demo:

| name | value | meaning |
|---|---|---|
| `encoder_gain` | 360/65535 | raw count → degrees |
| `pos_gain` | 1000 | position LSB × gear |
| `torque_gain` | 206.04 | iq LSB per N·m at the output |
| `vel_gain`, `vel_state_gain` | 1000, 10 | speed command / feedback scaling |
| `dir_1`, `dir_2` | +1, −1 | direction signs for the two bench motors |

It was called `config.py` until this reorganisation. It was renamed because a
second `config.py` exists in the trot package, and a bare `import config`
picking the wrong one killed the CAN layer with an unrelated
`module 'config' has no attribute 'encoder_gain'` several imports later.
`src/selftest/test_layout.py` now gates against that ever recurring.

**The robot's actual joint configuration is
`dog5_description/dog5_hardware_map.py`** — chapter 3.

## 6. Faults, and how the recovery ladder was found

Every driver has a **10 ms input-signal timeout**. Miss it and the motor
latches error bit 7 (`0x80`) and stops responding to motion commands. On a
12-motor bus with a 4 ms sweep there is very little margin, so this is not a
rare event — it is the normal failure mode.

The recovery encoded in `MotorBus.recover()` is
**`0x9B` (clear errors) → `0x88` (run) + keep-alive**, and no power cycle.

That is an experimental result, not a reading of the datasheet.
`bringup/t7_recover_no_powercycle.py` asked whether the latch clears without
the `0x80` shutdown step — which momentarily de-energises the joint and drops a
loaded leg — using a seven-step per-motor cycle: arm → trip → confirm →
resume-probe → recover → verify → fallback A/B. It does.

`bringup/t8_lost_probe.py` then asked *why* motors latch, and separated two
causes: **H1**, the host stalled ≥50 ms and missed the deadline; **H2**, the
motor rebooted from a supply dip and latched with no preceding gap. It records
the send gap, the worst host stall, the missed-reply delta and the last two
voltage readings at each latch. Nothing moves during the test.

## 7. The bring-up ladder

Run in order on new hardware. Each prints a verdict and writes a CSV.

| | what it proves |
|---|---|
| `t1_preflight.sh` | `can0` up at 1 Mbit/s with `txqueuelen 1000` and `restart-ms 100`. The default queue length of 10 overflows at 12-motor rates |
| `t2_ping_scan.py` | all twelve IDs answer; no duplicates; nothing above the range |
| `t2read.py` | wide scan 1–32 and a full decode of every read-only field, for a rig that is not configured yet |
| `t3_rate_sweep.py` | the real command-rate ceiling (§4) |
| `t3b_broadcast_probe.py` | whether the `0x280`/`0x281`/`0x282` multi-motor broadcast is accepted, where replies land (`0x240+id`), and critically whether it feeds the 10 ms watchdog. All phases zero-iq |
| `t4_watchdog_trip.py` | the true timeout window on one spinning motor, then a four-rung recovery ladder |
| `t5_motion_soak.py` | all twelve moving for minutes; gates on misses, gaps, error flags, temperature rise <15 °C and <60 °C absolute, and supply sag |
| `t6_unplug_multi.py` | unplug one motor mid-run; only that ID's misses grow |
| `t7_recover_no_powercycle.py` | §6 |
| `t8_lost_probe.py` | §6 |

## 8. Try it

```bash
sudo ./src/bringup/t1_preflight.sh          # bring can0 up
python3 src/motor_demos/can_smoke.py        # do the motors answer at all
python3 src/motor_demos/encoder_compare.py  # the encoder is pre-gearbox, 10:1
python3 src/motor_demos/demo_motors.py      # open the bus ONCE, share it
```

## Known limitations / what's next

- **`motor_library.LKMotor` is unsafe on a shared bus** and is retained only
  because seventeen files still import it. Nothing prevents a newcomer from
  reaching for it first; the docstrings warn, the type system does not.
- `prime_watchdog()` is deprecated in favour of `MotorBus.arm()` but still
  present and still called by the demos.
- `open_bus()` predates the `MotorBus` context manager and duplicates part of
  its job.
- **A `gs_usb` adapter can only be opened once per process.** Open the bus once
  and share it; do not open, close and reopen. `mac_can.py` handles adapter
  detection on macOS and has no automated coverage at all.
- **No CAN-FD path.** The 8000-frame/s ceiling in §4 is a classical-CAN
  ceiling, and it is the binding constraint on control rate for the whole
  project. CAN-FD is the obvious way past it and has not been tried.
- The bring-up suite is Linux/SocketCAN only; `t1_preflight.sh` is a bash
  script around `ip link`.
