# =====================
# Demo: control motors (id 1, 2) sequentially
# =====================

import time
import numpy as np

from motor_library import LKMotor, open_bus, prime_watchdog
import motor_gains as param


# ---- Demo settings ----
MOTOR_IDS = [1, 2]
TARGET_DEG = 30.0          # target angle to move each motor to (deg)
MAX_SPEED = param.max_speed_pos
HOLD_TIME = param.position_hold_time
RESET_ZERO = True          # re-zero each motor's current position before moving


def main():
    # ---- Connect to all motors on one shared CAN bus ----
    # A gs_usb adapter can only be opened once, so open the bus here and share
    # it across every motor (works on Linux/SocketCAN too).
    bus = open_bus(bitrate=1_000_000)
    motors = {mid: LKMotor(motor_id=mid, bus=bus) for mid in MOTOR_IDS}
    print(f"Connected to motors {MOTOR_IDS}")

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

    # ---- Reset: define the current position of each motor as 0 deg ----
    if RESET_ZERO:
        for mid, m in motors.items():
            m.set_position_to_angle(0)
            print(f"[Motor {mid}] position reset to 0 deg")

    try:
        # ---- Move each motor, one after another ----
        for mid in MOTOR_IDS:
            print(f"\n[Motor {mid}] moving to {TARGET_DEG:+.1f} deg")

            res = motors[mid].multi_turn_position_control(
                int(TARGET_DEG * param.pos_gain),
                int(MAX_SPEED),
            )

            if res is None:
                print(f"[Motor {mid}] ====== Lost Packet ======")
                continue

            temperature, iq_raw, spd_raw, enc = res

            th_deg = enc * param.encoder_gain
            tau = iq_raw / param.torque_gain
            dth_deg = spd_raw / param.vel_state_gain

            print(
                f"[Motor {mid}] "
                f"angle={th_deg:+.2f} deg  "
                f"speed={dth_deg:+.2f} dps  "
                f"torque={tau:+.2f}  "
                f"temp={temperature} C"
            )

            # wait for this motor to settle before driving the next one
            time.sleep(HOLD_TIME)

        print("\nSequential demo complete.")

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        # ---- Release every motor, then shut down the shared bus ----
        for mid, m in motors.items():
            m.motor_release()
            print(f"[Motor {mid}] released")
        bus.shutdown()


if __name__ == "__main__":
    print("Start motor demo.")
    main()
    print("End motor demo.")
