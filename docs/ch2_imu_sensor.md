# Chapter 2 — The IMU

**Status: PASSED.** ~200 Hz, zero CRC errors, 0.01–0.03° of jitter with all
twelve motors loaded.

Source: [`src/IMU_sensor/`](../src/IMU_sensor) · consumed by
[`torque_stand/feedback_estimator.py`](../src/torque_stand/feedback_estimator.py)

---

## 1. The device

One **FDILink DETA10** AHRS on the trunk, read over serial. `ImuDog` wraps the
vendor's `DETA10` class and hands back attitude already expressed in the dog's
frame, with a staleness measure attached.

Two packet types matter:

- **`0x41`** — attitude already fused on the sensor. This is what the standing
  and trotting controllers use.
- **`0x40`** — raw specific force and angular rate. **Nothing in this
  repository reads it.** It is what a filter doing its own fusion would want,
  because such a filter must not be fed someone else's posterior — and the
  shipped loop has no such filter. See [chapter 4
  §5](ch4_quasi_dynamic_trot.md#5-feedback-without-a-motion-capture-rig).

## 2. Frames: NED → FLU

The sensor speaks **NED** (X forward, Y right, Z **down**) — the aerospace
convention, where a level board reads (0, 0, −9.81) because +Z points into the
ground. The robot, `dog5.xml`, MuJoCo and every foot target in this repository
speak **FLU** (X forward, Y **left**, Z **up**).

```
   IMU sensor frame (NED)                DOG / trunk frame (FLU)

         X (nose)                          Z (up)   X (nose)
        /                                     |     /
       /                                      |    /
      +----------- Y (right)    Y (left) -----+---/
      |
      Z (down)
```

The transform is a rotation of 180° about X — negate Y and Z. Applied to
**Euler angles** that is a *conjugation*, not an added roll:

| quantity | NED | → FLU | why |
|---|---|---|---|
| roll | φ | **φ** | X is shared |
| pitch | θ | **−θ** | Y negated |
| yaw | ψ | **−ψ** | Z negated |
| body rates | (wx, wy, wz) | (wx, **−wy**, **−wz**) | vector remap |

This took two wrong attempts to get right, and both failures are instructive.
Adding 180° to roll treated it as a physical flip, and a level robot then read
−177°. Using the identity got *level* right but would have left pitch and yaw
signs inverted against the model — which would not have shown up until the
robot leaned. `sensor_to_dog()` in `imu_dog.py` implements the conjugation.

### Sign conventions in FLU (right-hand rule)

```
roll  > 0  →  RIGHT side down
pitch > 0  →  nose DOWN
yaw   > 0  →  nose swings LEFT (CCW from above)
```

**`+pitch` is nose down**, which is the opposite of the aerospace habit. Every
leveling law in chapters 4–6 depends on that sign.

## 3. Calibration

Two layers, and they are different in kind:

1. **The frame transform** above — exact, structural, never measured.
2. **Mounting offsets** — the board is not perfectly square to the trunk.
   Measured with the robot flat on a level floor and persisted in
   `imu_calib.json`: **roll +0.62°, pitch +0.18°**. Subtracted after the frame
   transform.

`imu_frame_test.py` is the Phase-0 procedure: hand-rotate the robot about each
axis in turn and confirm every sign against the table above before trusting
anything downstream.

## 4. Yaw is not usable as a heading

Yaw comes from the magnetometer, and the magnetometer sits a few centimetres
from twelve brushless motors bolted to a steel frame. It is **display and
logging only**.

Yaw *rate*, from the gyro, is inertial and is fine — it drifts slowly when
integrated but is trustworthy over a step or a gait cycle. This is why every
later chapter closes yaw on **rate**, never on absolute heading. The heading
the AHRS reports is latched once at start-up and every later reading is taken
relative to it, so the absolute number never has to mean anything.

## 5. Noise, measured

`imu_noise_log.py` logs every AHRS packet (~200 Hz) to CSV, prints noise
statistics and recommends a low-pass cutoff. The Phase-0 result, with motors
powered and loaded: **~200 Hz sustained, 0 CRC errors, 0.01–0.03° jitter.**

That number is the reason an attitude loop can use a 0.2° deadband and still
be doing something real: the jitter is an order of magnitude below it.

## 6. Where the IMU physically is

The board sits **38 mm below the trunk origin** — `IMU_BELOW_TRUNK_ORIGIN_M` in
the stand parameters. But `dog5.xml` declares:

```xml
<site name="imu" pos="0 0 0"/>
```

at the trunk origin. **That is correct in simulation and wrong on hardware.**
The discrepancy is compensated in the consumers rather than fixed in the model,
which is a real wart — see chapter 3's limitations. It is also one of the
three different "heights" that [chapter 4](ch4_quasi_dynamic_trot.md) has to
keep separate, and getting them confused was the 2026-08-17 frame bug.

## 7. The vendor SDK, and the `$HOME` trap

`fdilink_imu` is a vendor SDK. It is **not on PyPI and not vendored in this
repository**; install it on the robot host at
`~/Documents/IMU_sensor/fdilink_imu`.

`imu_dog.py` originally resolved it as `Path.home()/Documents/IMU_sensor`,
which breaks under `sudo` — `$HOME` becomes `/root`. Real-time priority wants
root, so every `chrt` run hit it. Resolution now goes through
`dog5_paths.add_fdilink_root()`, which searches repo-relative **first** and
falls back to the invoking user's home (including `$SUDO_USER`).

The import is also **guarded**: without the SDK, `imu_dog` still imports, and
its constants and pure frame maths still work. Only constructing an `ImuDog`
raises, and it raises with an explanation. This matters because eleven of the
seventeen call sites in this repository want nothing from the module but
`DEFAULT_PORT`, and the offline gate suite fakes the sensor outright
(`selftest_common.FakeAhrs` / `FakeFeed`). Before the guard, one missing vendor
package took every offline gate with it.

## 8. Downstream

| consumer | what it takes |
|---|---|
| `torque_stand/feedback_estimator.py` | fused `0x41` attitude — roll and pitch, with the mount-tilt setpoint subtracted. This is the only attitude the control loop sees |
| `dog5_trot_quasi_static_model/att_web.py` | a browser dashboard of roll/pitch/yaw, sampled independently of the loop so the two traces can be compared |

## 9. Try it

```bash
python3 src/IMU_sensor/imu_frame_test.py    # hand-rotate, check every sign
python3 src/IMU_sensor/imu_noise_log.py     # 200 Hz to CSV + noise stats
```

Both need the robot and the vendor SDK. `imu_frame_test.py` additionally needs
a POSIX host — it imports `termios` for raw keyboard input and will not run on
Windows.

## Known limitations / what's next

- **Yaw is unusable as an absolute reference indoors.** No magnetometer
  calibration procedure was ever written, and next to this much steel and this
  many motors it is not clear one would help. An external heading reference is
  the real fix.
- **The sim/hardware IMU site disagree by 38 mm** and the fix lives in the
  consumers, not in `dog5.xml`. Any new consumer that forgets is silently
  wrong by 38 mm.
- One IMU. No redundancy, no cross-check, no way to tell a bad sensor from a
  bad mount except by disagreeing with FK.
- No temperature compensation, and no characterisation of bias drift over a
  session longer than a few minutes.
- The vendor SDK is not vendored, so the IMU path is not reproducible from this
  repository alone.
