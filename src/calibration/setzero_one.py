#!/usr/bin/env python3
"""setzero_one.py -- hardware set-zero (0x19) for ONE motor.

Change MOTOR_ID below (or pass it on the command line) to the motor you want.
Brings that motor up LIMP (zero torque, back-drivable), clears a 0x80 input-lost
latch over CAN if needed (no power cycle), then writes 0x19 so the CURRENT pose
becomes the motor's zero. The zero takes effect only after a POWER-CYCLE.

    python setzero_one.py          # zeroes MOTOR_ID below
    python setzero_one.py 12       # zeroes id 12 (overrides MOTOR_ID)
"""
import sys
import time

from motorbus import MotorBus, ENCODER_GAIN

MOTOR_ID = 12          # <-- change this to the id you want to zero


def main():
    mid = int(sys.argv[1]) if len(sys.argv) > 1 else MOTOR_ID

    print(f"HARDWARE zero (0x19) of motor id{mid} at its current pose.")
    print("0x19 affects the driver lifetime; it takes effect after a power-cycle.")
    input(f"Pose the joint, then press Enter to prime + zero id{mid} "
          "(Ctrl-C aborts) > ")

    with MotorBus([mid]) as mb:
        print(f"id{mid}: priming (zero torque, back-drivable). Switch the motor ON "
              "if it is off.")
        # Feed the ~10 ms input-lost watchdog; if error bit 0x80 is latched, clear
        # it over CAN (0x9B -> 0x88) -- no power cycle. Break once it replies clear.
        t0 = time.perf_counter()
        while True:
            mb.keepalive(mid)                       # 0xA1 iq=0 -- feed, no motion
            err, _state = mb.status1(timeout=0.05)[mid]
            if err is not None and not (err & 0x80):
                break                               # replying, latch clear
            if err is not None and (err & 0x80):
                mb.recover(mid)                     # 0x9B -> 0xA1 -> 0x88 over CAN
            if time.perf_counter() - t0 > 30:
                raise SystemExit(f"id{mid}: no clear reply in 30 s -- check power / "
                                 "CAN / that this id exists on the bus.")
            time.sleep(0.004)

        jd = mb.encoders_deg()[mid]
        jd = (jd + 180.0) % 360.0 - 180.0           # fold to +/-180 for display
        print(f"id{mid}: at joint_deg={jd:+.2f}. Writing 0x19 (set this pose as ZERO)...")

        off = mb.set_zero_all(targets=[mid])[mid]   # paced + retried 0x19
        if off is None:
            raise SystemExit(f"id{mid}: NO ACK to 0x19 -- the motor replies but did "
                             "not accept the zero. Check its id / wiring (e.g. a "
                             "duplicate CAN id) or firmware.")
        print(f"id{mid}: OK -- new encoder_offset = {off}")
        print(f"DONE. POWER-CYCLE motor id{mid}; it will then read joint_deg ~ 0 "
              "at this pose.")


if __name__ == "__main__":
    main()
