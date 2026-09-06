#!/usr/bin/env python3
"""Observe, verify, or write the twelve joint zeros.

Every angle in this SDK is measured from ONE pose: the calibration pose, legs
flat fore-aft, trunk on the ground -- :data:`dog5_sdk.poses.Q_ZERO`.  That zero
does not live in software.  It lives in each driver's encoder-offset register,
written once by the 0x19 command.  This tool is how it gets there, and how you
check it afterwards.

    python tools/calibrate_zero.py --observe    # which ID is which, and which
                                                # way does it count?  (safe)
    python tools/calibrate_zero.py --verify     # at the zero pose, do all
                                                # twelve read ~0?  (safe)
    python tools/calibrate_zero.py --set-zero   # WRITE.  Read the warning.

EVERY MODE HOLDS ALL TWELVE MOTORS AT ZERO TORQUE.  Nothing is commanded to
move in any mode; you pose the robot by hand and it stays back-drivable.

--observe is how you confirm the hardware map without trusting it: turn ONE
joint by hand and watch which row moves and which way it counts.  The
``joint`` column applies the direction from the map, so a joint moved in its
POSITIVE direction must show a POSITIVE delta.  If it does not, the map is
wrong for that motor, and no amount of control tuning will fix that.

--set-zero warnings, all three of which are real:
  * the vendor warns that writing the encoder offset affects driver lifetime.
    Do it when the robot is built or re-assembled, not routinely;
  * it takes effect only after a POWER CYCLE.  Then run --verify;
  * it is not reversible.  There is no previous zero to go back to.
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

import dog5_sdk as d5
from dog5_sdk.calibration import joint_state, new_unwrappers, set_zero_all
from dog5_sdk.keys import KeyPoller
from dog5_sdk.motor import motorbus


def _table(q_deg, motoroutput_deg, delta_deg=None) -> str:
    header = (f"  {'CAN':>3}  {'joint':<10} {'dir':>3}  {'motoroutput':>12} "
              f"{'joint deg':>10}")
    if delta_deg is not None:
        header += f" {'delta':>9}"
    rows = [header]
    for index, joint in enumerate(d5.HARDWARE_JOINTS):
        row = (f"  {joint.can_id:>3}  {joint.leg + '_' + joint.joint:<10} "
               f"{joint.direction:>+3}  {motoroutput_deg[index]:>+11.2f} "
               f"{q_deg[index]:>+9.2f}")
        if delta_deg is not None:
            row += f" {delta_deg[index]:>+8.2f}"
        rows.append(row)
    return "\n".join(rows)


def _stream(mb, unwrap, seconds=None, label="", tol=None):
    """Zero-torque keep-alive stream, printing the live table.  Returns the
    last (q, motoroutput_deg) pair read."""
    slot = mb.slot(250.0)
    deadline = time.perf_counter() + slot
    start = time.perf_counter()
    index = 0
    last_print = 0.0
    first = None
    q = motoroutput = None

    with KeyPoller() as keys:
        while True:
            mb.poll()
            slot_index = index % len(mb.ids)
            if slot_index == 0:
                now = time.perf_counter()
                try:
                    q, _qd = joint_state(mb, unwrap)
                except RuntimeError:
                    q = None
                if q is not None:
                    motoroutput = d5.joint_rad_to_motoroutput_deg(q)
                    if first is None:
                        first = np.rad2deg(q).copy()
                    if now - last_print >= 0.25:
                        delta = np.rad2deg(q) - first
                        print("\033[H\033[J" if last_print else "", end="")
                        print(f"[{label}] zero torque, back-drivable.  "
                              "Ctrl-C or x to stop.")
                        print(_table(np.rad2deg(q), motoroutput, delta))
                        if tol is not None:
                            worst = float(np.max(np.abs(np.rad2deg(q))))
                            verdict = "OK" if worst <= tol else "OFF ZERO"
                            print(f"\n  worst |joint| = {worst:.2f} deg "
                                  f"(tolerance {tol:.2f})  -> {verdict}")
                        last_print = now
                if keys.get() in ("x", "X"):
                    break
                if seconds is not None and now - start >= seconds:
                    break
            mb.keepalive(mb.ids[slot_index])
            index += 1
            mb.pace(deadline)
            deadline += slot
    return q, motoroutput


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--observe", action="store_true",
                      help="live table; turn a joint by hand and watch it")
    mode.add_argument("--verify", action="store_true",
                      help="at the zero pose, check all twelve read ~0")
    mode.add_argument("--set-zero", action="store_true",
                      help="WRITE the current pose as every joint's zero")
    parser.add_argument("--tol", type=float, default=2.0,
                        help="--verify tolerance, degrees")
    parser.add_argument("--seconds", type=float, default=None,
                        help="stop after this long (default: until x)")
    parser.add_argument("--yes", action="store_true",
                        help="--set-zero without the confirmation prompt")
    args = parser.parse_args()

    d5.calibration.validate()
    unwrap = new_unwrappers()

    with motorbus.MotorBus(d5.MOTOR_IDS, dirs=d5.MOTOR_DIRECTIONS) as mb:
        print("[calib] arming limp: zero-torque keep-alives to all twelve.  "
              "Switch 24 V on if it is off; a 0x80-latched motor is cleared "
              "over CAN, no power cycle.")
        if not mb.arm(rate_hz=250.0):
            print("[calib] not every motor armed", file=sys.stderr)
            return 1

        if args.observe:
            print("[calib] OBSERVE: turn ONE joint by hand.  A joint moved in "
                  "its POSITIVE direction must show a POSITIVE delta.")
            _stream(mb, unwrap, args.seconds, label="observe")
            return 0

        if args.verify:
            print("[calib] VERIFY: pose the robot at the CALIBRATION POSE "
                  "(legs flat fore-aft, trunk down).")
            q, _ = _stream(mb, unwrap, args.seconds or 10.0,
                           label="verify", tol=args.tol)
            if q is None:
                print("[calib] no complete reading", file=sys.stderr)
                return 1
            worst = float(np.max(np.abs(np.rad2deg(q))))
            print(f"\n[calib] worst joint is {worst:.2f} deg off zero "
                  f"(tolerance {args.tol:.2f})")
            return 0 if worst <= args.tol else 1

        # --set-zero
        print("[calib] SET ZERO: pose the robot at the CALIBRATION POSE now.")
        q, motoroutput = _stream(mb, unwrap, args.seconds or 10.0,
                                 label="set-zero")
        if q is None:
            print("[calib] no complete reading; nothing written",
                  file=sys.stderr)
            return 1
        print("\nThis pose is about to become every joint's permanent zero.")
        print("  * the vendor warns 0x19 affects driver lifetime")
        print("  * it takes effect only after a POWER CYCLE")
        print("  * it cannot be undone")
        if not args.yes:
            if input("Type 'zero' to write it: ").strip() != "zero":
                print("[calib] nothing written")
                return 1

        offsets = set_zero_all(mb, confirm=True)
        failed = [mid for mid, value in offsets.items() if value is None]
        for joint in d5.HARDWARE_JOINTS:
            value = offsets.get(joint.can_id)
            print(f"  CAN {joint.can_id:>2}  {joint.leg}_{joint.joint:<6} "
                  f"offset {'-- NOT WRITTEN' if value is None else value}")
        if failed:
            print(f"[calib] {failed} did not ack -- their zeros were NOT "
                  "written.  Re-run before power-cycling.", file=sys.stderr)
            return 1
        print("[calib] written.  POWER CYCLE, then run --verify.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
