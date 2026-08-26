#!/usr/bin/env python3
"""Move motors 1, 4, 7, and 10 to fixed joint-angle targets.

Targets are absolute output-joint angles relative to each motor's calibrated
zero:

    motor 1  ->  +90 deg
    motor 4  ->  -90 deg
    motor 7  ->  +90 deg
    motor 10 ->  -90 deg

Startup and fault recovery use MotorBus's T7-proven minimal recovery sequence:
0x9B (clear) -> 0x88 (run) with zero-torque keep-alives. No 0x80 shutdown or
power cycle is required for the input-signal-lost latch.

The position frames are streamed continuously because the motor input watchdog
expects regular CAN traffic. With --all-zero, all 12 motors are commanded to
0 degrees. Press Ctrl-C to stop and release the commanded motors.
"""

import argparse
import time

from motorbus import MAX_SPEED_POS, MotorBus, decode_errors


TARGET_DEG = {
    1: 90.0,
    4: -90.0,
    7: 90.0,
    10: -90.0,
}
MOTOR_IDS = list(TARGET_DEG)
ALL_MOTOR_IDS = list(range(1, 13))
ZERO_TARGET_DEG = {mid: 0.0 for mid in ALL_MOTOR_IDS}
CONTROL_HZ = 250.0
STATUS_PERIOD_S = 0.1
PRINT_PERIOD_S = 0.5


def folded_encoder_deg(mb, mid):
    """Return the last single-turn encoder reading folded to [-180, 180)."""
    encoder = mb.rec(mid).encoder
    if encoder is None:
        return None
    angle = encoder * mb.encoder_gain
    return (angle + 180.0) % 360.0 - 180.0


def run(max_speed_dps, hold_s=None, target_deg=None):
    if target_deg is None:
        target_deg = TARGET_DEG
    active_motor_ids = list(target_deg)

    targets = ", ".join(
        f"M{mid}={target:+.0f} deg" for mid, target in target_deg.items()
    )
    print(f"Targets: {targets}")
    print("Support the robot and keep an emergency stop within reach.")
    input("Press Enter to arm the motors and move (Ctrl-C aborts) > ")

    with MotorBus(active_motor_ids) as mb:
        # MotorBus.arm() is the T7 recovery method: a watchdog-safe zero-torque
        # stream plus the minimal 0x9B -> 0x88 clear/run ladder.
        if not mb.arm(rate_hz=CONTROL_HZ, timeout_s=None):
            raise RuntimeError("one or more motors could not be armed")

        slot = mb.slot(CONTROL_HZ)
        deadline = time.perf_counter() + slot
        start = time.perf_counter()
        next_print = start
        next_status = {mid: start for mid in active_motor_ids}
        index = 0

        print("Moving and holding position; press Ctrl-C to stop.")
        while hold_s is None or time.perf_counter() - start < hold_s:
            mid = active_motor_ids[index % len(active_motor_ids)]
            index += 1
            mb.poll()

            # A status request replaces one position frame about every 100 ms.
            # It still feeds the input watchdog and lets us detect a re-latch.
            now = time.perf_counter()
            if now >= next_status[mid]:
                mb.status1_req(mid)
                next_status[mid] = now + STATUS_PERIOD_S
            else:
                error = mb.rec(mid).error
                if error is not None and error & 0x80:
                    print(
                        f"M{mid}: {decode_errors(error)}; applying T7 recovery"
                    )
                    # The surrounding position stream supplies T7's settle
                    # traffic, so the in-loop recovery itself must not block.
                    mb.recover(mid, settle_s=0.0, verify=False)
                    mb.rec(mid).error = None
                    next_status[mid] = now + STATUS_PERIOD_S

                if not mb.position(mid, target_deg[mid], max_speed_dps):
                    raise RuntimeError(f"CAN transmit failed for motor {mid}")

            if now >= next_print:
                readings = []
                for motor_id in active_motor_ids:
                    angle = folded_encoder_deg(mb, motor_id)
                    readings.append(
                        f"M{motor_id}={'--' if angle is None else f'{angle:+.1f} deg'}"
                    )
                print("  ".join(readings))
                next_print = now + PRINT_PERIOD_S

            mb.pace(deadline)
            deadline += slot


def move_all_to_zero(max_speed_dps, hold_s=None):
    """Position-control motors 1 through 12 to 0 degrees.

    This sends ordinary position commands. It does not write a new encoder zero
    or require a power cycle.
    """
    run(max_speed_dps, hold_s, target_deg=ZERO_TARGET_DEG)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--max-speed-dps",
        type=float,
        default=MAX_SPEED_POS,
        help=(
            "motor-side position speed cap in deg/s "
            f"(default: {MAX_SPEED_POS:g}, about {MAX_SPEED_POS / 10:g} "
            "output deg/s with the 10:1 reducer)"
        ),
    )
    parser.add_argument(
        "--hold-s",
        type=float,
        default=None,
        help="stop after this many seconds (default: hold until Ctrl-C)",
    )
    parser.add_argument(
        "--all-zero",
        action="store_true",
        help="position-control motors 1 through 12 to 0 deg",
    )
    args = parser.parse_args()

    if args.max_speed_dps <= 0:
        parser.error("--max-speed-dps must be positive")
    if args.hold_s is not None and args.hold_s <= 0:
        parser.error("--hold-s must be positive")

    try:
        if args.all_zero:
            move_all_to_zero(args.max_speed_dps, args.hold_s)
        else:
            run(args.max_speed_dps, args.hold_s)
    except KeyboardInterrupt:
        print("\nStopped; all motors released.")


if __name__ == "__main__":
    main()
