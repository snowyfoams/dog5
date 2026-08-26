#!/usr/bin/env python3
"""Hold the recorded DOG5 pose with all 12 motors in position control.

The targets below are the folded ``joint_deg`` readings captured with
``calibrate12.py --observe``.  They are therefore in each motor's raw encoder
coordinate, not yet in the DOG5 model-positive coordinate.  MotorBus is left
at its default ``dir=+1`` so the position targets use that same raw convention.

The robot must be supported.  By default the script refuses to enable position
control when any joint is more than 8 degrees from its recorded target.  This
makes the normal use case "capture this pose, then hold it" and avoids an
unexpected large move.  Pass ``--allow-large-move`` only when the robot is
supported, every joint has a clear path, and a deliberate move is intended.

Position frames are streamed continuously to satisfy the motor input watchdog.
Press Ctrl-C to stop and release all motors.
"""

import argparse
import time

from motorbus import MotorBus, decode_errors


# CAN mapping recorded in coordinates.md:
#   RL = 1,2,3; RR = 4,5,6; FR = 7,8,9; FL = 10,11,12
JOINT_NAME = {
    1: "RL_abd",
    2: "RL_hip",
    3: "RL_knee",
    4: "RR_abd",
    5: "RR_hip",
    6: "RR_knee",
    7: "FR_abd",
    8: "FR_hip",
    9: "FR_knee",
    10: "FL_abd",
    11: "FL_hip",
    12: "FL_knee",
}

# Recorded folded encoder angles in degrees.  Keep dir=+1 for these raw targets.
TARGET_DEG = {
    1: +84.38,
    2: -92.04,
    3: +3.56,
    4: -85.07,
    5: +89.18,
    6: +1.08,
    7: +84.97,
    8: -93.46,
    9: +0.41,
    10: -83.90,
    11: +85.90,
    12: -3.32,
}

MOTOR_IDS = list(TARGET_DEG)
CONTROL_HZ = 250.0          # command rate per motor
STATUS_PERIOD_S = 0.10
PRINT_PERIOD_S = 0.50
DEFAULT_MAX_SPEED_DPS = 100.0   # motor-side cap; about 10 output deg/s at 10:1
DEFAULT_START_TOL_DEG = 8.0
MAX_TEMPERATURE_C = 80
MAX_OUTPUT_SPEED_DPS = 30.0


def fold180(deg):
    """Fold an angle into [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def circular_error_deg(target, measured):
    """Shortest signed target-minus-measured error in degrees."""
    return fold180(target - measured)


def measured_deg(mb, mid):
    """Last raw encoder reading in the same folded frame as TARGET_DEG."""
    raw = mb.rec(mid).encoder
    if raw is None:
        return None
    return fold180(raw * mb.encoder_gain)


def print_targets():
    print("\nRecorded hold targets (raw encoder coordinates):")
    print(f"  {'id':>2}  {'joint':<8}  {'target':>9}")
    for mid in MOTOR_IDS:
        print(f"  {mid:2d}  {JOINT_NAME[mid]:<8}  {TARGET_DEG[mid]:+8.2f} deg")


def check_start_pose(mb, tolerance_deg, allow_large_move):
    """Verify feedback exists and report distance from the recorded pose."""
    mb.hold(0.25, rate_hz=CONTROL_HZ)  # zero torque while feedback settles

    print("\nZero-torque start-pose check:")
    print(f"  {'id':>2}  {'joint':<8}  {'measured':>9}  {'target':>9}  {'error':>8}")
    outside = []
    for mid in MOTOR_IDS:
        measured = measured_deg(mb, mid)
        if measured is None:
            raise RuntimeError(f"no encoder feedback from CAN ID {mid}")
        error = circular_error_deg(TARGET_DEG[mid], measured)
        print(
            f"  {mid:2d}  {JOINT_NAME[mid]:<8}  {measured:+8.2f}  "
            f"{TARGET_DEG[mid]:+8.2f}  {error:+7.2f}"
        )
        if abs(error) > tolerance_deg:
            outside.append((mid, error))

    if outside and not allow_large_move:
        details = ", ".join(
            f"{JOINT_NAME[mid]} {error:+.1f} deg" for mid, error in outside
        )
        raise RuntimeError(
            f"start pose is outside the {tolerance_deg:g} deg hold tolerance: "
            f"{details}. Re-pose the supported robot or deliberately pass "
            "--allow-large-move."
        )


def hold_pose(max_speed_dps, hold_s, start_tolerance_deg, allow_large_move):
    # No dirs mapping is supplied: TARGET_DEG is explicitly in raw motor signs.
    with MotorBus(MOTOR_IDS) as mb:
        if not mb.arm(rate_hz=CONTROL_HZ, timeout_s=None):
            raise RuntimeError("one or more motors could not be armed")

        check_start_pose(mb, start_tolerance_deg, allow_large_move)

        slot = mb.slot(CONTROL_HZ)
        deadline = time.perf_counter() + slot
        start = time.perf_counter()
        next_print = start
        next_status = {
            mid: start + index * STATUS_PERIOD_S / len(MOTOR_IDS)
            for index, mid in enumerate(MOTOR_IDS)
        }
        index = 0

        print("\nPosition control active; holding the recorded pose.")
        print("Press Ctrl-C to stop and release all motors.")

        while hold_s is None or time.perf_counter() - start < hold_s:
            mid = MOTOR_IDS[index % len(MOTOR_IDS)]
            index += 1
            mb.poll()
            now = time.perf_counter()
            rec = mb.rec(mid)

            # A status request replaces one position frame every 100 ms.  It
            # still feeds the watchdog and makes fault flags visible.
            if now >= next_status[mid]:
                mb.status1_req(mid)
                next_status[mid] += STATUS_PERIOD_S
            else:
                if rec.error is not None and rec.error:
                    if rec.error & 0x7F:
                        raise RuntimeError(
                            f"{JOINT_NAME[mid]} fault: {decode_errors(rec.error)}"
                        )
                    if rec.error & 0x80:
                        print(
                            f"{JOINT_NAME[mid]}: input timeout; applying CAN recovery"
                        )
                        mb.recover(mid, settle_s=0.0, verify=False)
                        rec.error = None
                        next_status[mid] = now + STATUS_PERIOD_S

                if not mb.position(mid, TARGET_DEG[mid], max_speed_dps):
                    raise RuntimeError(f"CAN transmit failed for motor {mid}")

            if rec.temp is not None and rec.temp >= MAX_TEMPERATURE_C:
                raise RuntimeError(
                    f"{JOINT_NAME[mid]} over-temperature: {rec.temp} C"
                )

            if rec.speed is not None:
                output_dps = rec.speed / mb.vel_state_gain
                if abs(output_dps) > MAX_OUTPUT_SPEED_DPS:
                    raise RuntimeError(
                        f"{JOINT_NAME[mid]} overspeed: {output_dps:+.1f} deg/s"
                    )

            if now >= next_print:
                fields = []
                for motor_id in MOTOR_IDS:
                    measured = measured_deg(mb, motor_id)
                    if measured is None:
                        fields.append(f"{JOINT_NAME[motor_id]}=--")
                    else:
                        error = circular_error_deg(TARGET_DEG[motor_id], measured)
                        fields.append(
                            f"{JOINT_NAME[motor_id]}={measured:+.1f}({error:+.1f})"
                        )
                print("  ".join(fields))
                next_print = now + PRINT_PERIOD_S

            mb.pace(deadline)
            deadline += slot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--max-speed-dps",
        type=float,
        default=DEFAULT_MAX_SPEED_DPS,
        help=(
            "motor-side position speed cap in deg/s "
            f"(default: {DEFAULT_MAX_SPEED_DPS:g})"
        ),
    )
    parser.add_argument(
        "--hold-s",
        type=float,
        default=None,
        help="release after this many seconds (default: hold until Ctrl-C)",
    )
    parser.add_argument(
        "--start-tolerance-deg",
        type=float,
        default=DEFAULT_START_TOL_DEG,
        help=(
            "refuse to engage farther than this from the recorded pose "
            f"(default: {DEFAULT_START_TOL_DEG:g} deg)"
        ),
    )
    parser.add_argument(
        "--allow-large-move",
        action="store_true",
        help="allow position control even when the start-pose check is outside tolerance",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the pre-arm Enter prompt",
    )
    args = parser.parse_args()

    if args.max_speed_dps <= 0:
        parser.error("--max-speed-dps must be positive")
    if args.hold_s is not None and args.hold_s <= 0:
        parser.error("--hold-s must be positive")
    if args.start_tolerance_deg <= 0:
        parser.error("--start-tolerance-deg must be positive")

    print_targets()
    print("\nWARNING: support the robot with every leg clear of obstructions.")
    print("Keep an emergency stop within reach.")
    if not args.yes:
        input("Press Enter to arm and hold this pose (Ctrl-C aborts) > ")

    try:
        hold_pose(
            args.max_speed_dps,
            args.hold_s,
            args.start_tolerance_deg,
            args.allow_large_move,
        )
    except KeyboardInterrupt:
        print("\nStopped by operator; all motors released.")
    except RuntimeError as exc:
        raise SystemExit(f"\nSTOPPED: {exc}\nAll motors released.") from exc
    else:
        print("\nHold duration complete; all motors released.")


if __name__ == "__main__":
    main()
