#!/usr/bin/env python3
"""
read_state.py -- read and decode the current motor state over CAN (no motion).

Queries status-1 (temp/voltage/current/state/errors), status-2 (temp/iq/speed/
encoder), status-3 (phase currents) and the multi-turn angle, then prints a
decoded summary using the gains in config.py. Read-only: commands no motion.

Usage:
  python read_state.py                 # motor id 1
  python read_state.py --motor-id 2
"""

import argparse

from motor_library import LKMotor
import motor_gains as param

# errorState bit meanings, from the LK K-TECH CAN protocol V2.36 (status-1 reply).
ERROR_BITS = [
    "low voltage", "high voltage", "driver over-temp", "motor over-temp",
    "over-current", "short circuit", "stall", "input signal lost timeout",
]


def decode_errors(err: int) -> str:
    flags = [name for i, name in enumerate(ERROR_BITS) if err & (1 << i)]
    return "none" if not flags else ", ".join(flags)


def main():
    p = argparse.ArgumentParser(description="Read and decode the motor state.")
    p.add_argument("--motor-id", type=int, default=1)
    args = p.parse_args()

    m = LKMotor(motor_id=args.motor_id)
    try:
        print(f"=== Motor {args.motor_id} state ===")

        s1 = m.read_motor_status_1()
        if s1 is None:
            print("  status-1: NO REPLY (check id / bus / power)")
        else:
            temp, volt, curr, state, err = s1
            print(f"  temperature : {temp} C")
            print(f"  bus voltage : {volt:.2f} V")
            print(f"  bus current : {curr:.2f} A")
            print(f"  motor state : 0x{state:02x} "
                  f"({'running' if state == 0x00 else 'stopped' if state == 0x10 else 'unknown'})")
            print(f"  error flags : 0x{err:02x} ({decode_errors(err)})")

        s2 = m.read_motor_status_2()
        if s2 is None:
            print("  status-2: NO REPLY")
        else:
            temp, iq, spd, enc = s2
            print(f"  torque iq   : {iq} raw ({iq / param.torque_gain:+.3f} N.m output)")
            print(f"  speed       : {spd} raw ({spd / param.vel_state_gain:+.1f} dps output)")
            print(f"  encoder     : {enc} raw ({enc * param.encoder_gain:.2f} deg motor-side, single-turn)")

        s3 = m.read_motor_status_3()
        if s3 is not None:
            _temp, ia, ib, ic = s3
            print(f"  phase A/B/C : {ia} / {ib} / {ic} raw")

        ang = m.read_multi_turn_angle()
        print(f"  multi-turn  : {ang} raw 0.01deg "
              f"({ang / param.pos_gain:+.2f} deg output)")
    finally:
        m.motor_release()


if __name__ == "__main__":
    main()
