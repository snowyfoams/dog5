# =====================
# Demo: trajectory following for legged-robot joints
#
# These LK motors have no combined "MIT" command (pos+vel+Kp+Kd+tau in one
# frame), so we close an impedance-style loop in software:
#
#     speed_cmd = qd_desired + KP * (q_desired - q_measured)
#
# and send it through the motor's velocity loop (speed_loop_control, 0xA2).
# The desired path q(t), qd(t) is produced by trajectory() below.
#
# Position feedback uses the single-turn encoder returned each tick; it is
# unwrapped into a continuous angle, with the startup position taken as 0 deg.
# =====================

import time
import math

from motor_library import LKMotor, open_bus, prime_watchdog
import motor_gains as param


# ---- Control settings ----
CONTROL_HZ = 200          # control loop rate (Hz)
DURATION = 10.0           # run time in seconds; set None to run until Ctrl-C
KP = 5.0                  # position-error gain (deg/s of speed per deg of error)
PRINT_HZ = 5              # console print rate (Hz)

# Current/torque field of the 0xA2 speed frame. In the standard LK protocol
# these bytes are reserved (NULL) and the speed value is the effective command,
# so this is left at 0.
IQ_LIMIT = 0


# ---- Joints: one entry per motor ----
# dir: +1 / -1 sign convention for this joint (see config.py)
JOINTS = [
    {"id": 1, "dir": param.dir_1},
    {"id": 5, "dir": param.dir_2},
]


def trajectory(t, joint_index):
    """Desired path for a joint at time t.

    Returns (q_des, qd_des) in output-shaft degrees and deg/s, relative to the
    startup home position. Default: a sinusoidal swing, opposite phase per joint.
    """
    amp = 20.0                          # amplitude (deg)
    freq = 0.5                          # frequency (Hz)
    offset = 0.0                        # joint mid-point (deg)
    phase = joint_index * math.pi       # 180 deg out of phase between joints

    w = 2.0 * math.pi * freq
    q_des = offset + amp * math.sin(w * t + phase)
    qd_des = amp * w * math.cos(w * t + phase)
    return q_des, qd_des


def main():
    dt = 1.0 / CONTROL_HZ
    print_every = max(1, int(CONTROL_HZ / PRINT_HZ))

    # ---- Connect (one shared bus for all joints) ----
    bus = open_bus(bitrate=1_000_000)
    motors = {j["id"]: LKMotor(motor_id=j["id"], bus=bus) for j in JOINTS}
    print(f"Connected to joints {[j['id'] for j in JOINTS]}")

    # Feed the MCU comms watchdog through power-on: start THIS script first, then
    # switch on motor power. Without it the watchdog trips ~500 ms after power-on
    # (latched, needs a power cycle). See motor_library.prime_watchdog().
    try:
        if not prime_watchdog(motors.values()):
            print("Motor(s) not ready (watchdog) -- aborting.")
            bus.shutdown()
            return
    except KeyboardInterrupt:
        print("\nAborted before start.")
        bus.shutdown()
        return

    for m in motors.values():
        m.motor_run()

    # ---- Seed feedback state: startup position becomes the home (0 deg) ----
    prev_raw = {}    # last single-turn encoder reading (deg, before dir)
    cont_deg = {}    # unwrapped continuous angle since home (deg, before dir)
    for j in JOINTS:
        res = motors[j["id"]].read_motor_status_2()
        enc = res[3] if res else 0
        prev_raw[j["id"]] = enc * param.encoder_gain
        cont_deg[j["id"]] = 0.0

    print(f"Following trajectory at {CONTROL_HZ} Hz "
          f"for {DURATION if DURATION else 'unlimited'} s. Ctrl-C to stop.\n")

    tick = 0
    t0 = time.time()
    try:
        while True:
            loop_start = time.time()
            t = loop_start - t0
            if DURATION is not None and t > DURATION:
                break

            for ji, j in enumerate(JOINTS):
                mid, d = j["id"], j["dir"]

                q_des, qd_des = trajectory(t, ji)
                q_meas = cont_deg[mid] * d

                # velocity feedforward + position PD
                err = q_des - q_meas
                speed_cmd = qd_des + KP * err          # deg/s, output shaft

                res = motors[mid].speed_loop_control(
                    IQ_LIMIT,
                    int(speed_cmd * d * param.vel_gain),
                )
                if res is None:
                    print(f"[Motor {mid}] ====== Lost Packet ======")
                    continue

                _, _, _, enc = res

                # unwrap single-turn encoder into a continuous angle
                raw = enc * param.encoder_gain         # 0..360 deg, before dir
                delta = raw - prev_raw[mid]
                if delta > 180.0:
                    delta -= 360.0
                elif delta < -180.0:
                    delta += 360.0
                cont_deg[mid] += delta
                prev_raw[mid] = raw

                if tick % print_every == 0:
                    print(f"t={t:5.2f}  [M{mid}] "
                          f"q_des={q_des:+6.2f}  q={cont_deg[mid] * d:+6.2f}  "
                          f"err={err:+5.2f}  cmd={speed_cmd:+6.1f} dps")

            tick += 1

            # hold the loop rate
            time.sleep(max(0.0, dt - (time.time() - loop_start)))

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        # stop motion, release each motor, then shut down the shared bus
        for mid, m in motors.items():
            m.speed_loop_control(IQ_LIMIT, 0)
            m.motor_release()
            print(f"[Motor {mid}] released")
        bus.shutdown()


if __name__ == "__main__":
    print("Start trajectory-following demo.")
    main()
    print("End trajectory-following demo.")
