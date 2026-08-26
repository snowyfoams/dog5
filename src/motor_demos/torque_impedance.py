# =====================
# Demo: torque / impedance control for legged-robot joints
#
# Software joint impedance (the "MIT mode" law these motors lack natively):
#
#     tau = KP*(q_des - q) + KD*(qd_des - qd) + tau_ff
#
# commanded through the motor's TORQUE loop (torque_loop_control, 0xA1).
# Torque is converted to a current command via config.torque_gain.
#
# DEFAULT BEHAVIOUR: a virtual spring-damper that holds the startup position
# (q_des = 0, qd_des = 0). Push a joint by hand and it softly springs back.
# This is the safe way to feel impedance control. Set TRACK_TRAJECTORY = True
# to instead follow the generated sinusoid.
#
# !!! SAFETY !!!
#   - Start with LOW gains and a LOW IQ_MAX, then raise gradually.
#   - Wrong encoder feedback or high gains can make the joint apply large force.
#   - Keep hands clear and be ready to Ctrl-C (exit zeros the torque).
# =====================

import time
import math

from motor_library import LKMotor, open_bus, prime_watchdog
import motor_gains as param


# ---- Control settings ----
CONTROL_HZ = 200          # control loop rate (Hz)
DURATION = 15.0           # run time in seconds; set None to run until Ctrl-C
PRINT_HZ = 5              # console print rate (Hz)
TRACK_TRAJECTORY = False  # False = hold home (virtual spring); True = follow sinusoid

# Impedance gains -- TUNE THESE, START LOW.
KP = 0.05                 # stiffness  (N*m per deg of position error)
KD = 0.005                # damping    (N*m per deg/s of velocity error)

# Hard safety clamp on the current command (|iq| <= 2048 is the protocol max).
# 500 LSB ~= 500 / torque_gain ~= 2.4 N*m. Raise only once it behaves.
IQ_MAX = 500


# ---- Joints: one entry per motor ----
JOINTS = [
    {"id": 1, "dir": param.dir_1},
    {"id": 2, "dir": param.dir_2},
]


def trajectory(t, joint_index):
    """Desired joint state (q_des, qd_des) in output deg and deg/s, vs. home.

    Hold home unless TRACK_TRAJECTORY is set, then a sinusoidal swing.
    """
    if not TRACK_TRAJECTORY:
        return 0.0, 0.0

    amp = 15.0                          # deg
    freq = 0.5                          # Hz
    phase = joint_index * math.pi
    w = 2.0 * math.pi * freq
    q_des = amp * math.sin(w * t + phase)
    qd_des = amp * w * math.cos(w * t + phase)
    return q_des, qd_des


def gravity_feedforward(q_meas, joint_index):
    """Optional gravity-compensation feedforward torque (N*m).

    Returns 0 by default. For a single-link approximation you would use
    tau_g = m * g * l_cm * cos(q) with the joint's mass/length; fill in once
    you have the leg's parameters.
    """
    return 0.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


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

    # ---- Seed feedback: startup position becomes home (0 deg) ----
    prev_raw = {}    # last single-turn encoder reading (deg, before dir)
    cont_deg = {}    # unwrapped continuous angle since home (deg, before dir)
    qd_meas = {}     # measured output velocity (deg/s, after dir)
    for j in JOINTS:
        res = motors[j["id"]].read_motor_status_2()
        enc = res[3] if res else 0
        spd = res[2] if res else 0
        prev_raw[j["id"]] = enc * param.encoder_gain
        cont_deg[j["id"]] = 0.0
        qd_meas[j["id"]] = j["dir"] * spd / param.vel_state_gain

    mode = "tracking sinusoid" if TRACK_TRAJECTORY else "holding home (virtual spring)"
    print(f"Impedance control at {CONTROL_HZ} Hz, {mode}. KP={KP} KD={KD}. Ctrl-C to stop.\n")

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

                # impedance law: spring + damper + feedforward  -> torque (N*m)
                tau = (KP * (q_des - q_meas)
                       + KD * (qd_des - qd_meas[mid])
                       + gravity_feedforward(q_meas, ji))

                # torque (N*m) -> current command (LSB), with sign and clamp
                iq_cmd = clamp(int(tau * param.torque_gain * d), -IQ_MAX, IQ_MAX)

                res = motors[mid].torque_loop_control(iq_cmd)
                if res is None:
                    print(f"[Motor {mid}] ====== Lost Packet ======")
                    continue

                _, iq_raw, spd_raw, enc = res

                # update velocity feedback
                qd_meas[mid] = d * spd_raw / param.vel_state_gain

                # unwrap single-turn encoder into a continuous angle
                raw = enc * param.encoder_gain
                delta = raw - prev_raw[mid]
                if delta > 180.0:
                    delta -= 360.0
                elif delta < -180.0:
                    delta += 360.0
                cont_deg[mid] += delta
                prev_raw[mid] = raw

                if tick % print_every == 0:
                    tau_meas = iq_raw * d / param.torque_gain
                    print(f"t={t:5.2f}  [M{mid}] "
                          f"q_des={q_des:+6.2f}  q={cont_deg[mid] * d:+6.2f}  "
                          f"tau_cmd={tau:+5.2f}  tau_meas={tau_meas:+5.2f} Nm")

            tick += 1
            time.sleep(max(0.0, dt - (time.time() - loop_start)))

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        # zero torque first, release each motor, then shut down the shared bus
        for mid, m in motors.items():
            m.torque_loop_control(0)
            m.motor_release()
            print(f"[Motor {mid}] released")
        bus.shutdown()


if __name__ == "__main__":
    print("Start torque/impedance demo.")
    main()
    print("End torque/impedance demo.")
