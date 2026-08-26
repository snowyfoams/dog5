#!/usr/bin/env python3
"""calibrate_leg.py -- observe / verify / hardware set-zero for ONE LEG (a SUBSET
of the CAN ids), default ids 1,2,3.

Why a new file: calibrate12.py is the validated ALL-12 baseline and is left
untouched. This script only narrows the id set -- it imports calibrate12's
bring-up / sampling / observe / verify functions unchanged (they are already
id-agnostic: every one of them works off mb.ids), so the electrical behaviour is
bit-identical to the 12-motor tool. Only the set-zero confirmation + summary is
re-written here so the counts say "3", not "12".

Use it after re-assembling a single leg: only the motors you name are streamed,
read and (optionally) 0x19-zeroed. The other motors are never touched.

  1) OBSERVE (safe, nothing is commanded to move -- all listed motors at ZERO
     torque, back-drivable):
       python calibrate_leg.py --observe
       python calibrate_leg.py --observe --ids 4,5,6
     Turn ONE joint by hand and watch which id moves and which way it counts.

  2) SET ZERO (HARDWARE 0x19 at the current pose, ONLY the listed ids):
       python calibrate_leg.py --set-zero              # ids 1,2,3
       python calibrate_leg.py --set-zero --yes        # skip the confirmation
     Pose the leg at its intended zero FIRST, then run this.
     !! 0x19 affects the DRIVER LIFETIME -- use it rarely. It takes effect only
     after a POWER-CYCLE.

  3) VERIFY (safe) -- after the power-cycle, at the same zero pose:
       python calibrate_leg.py --verify --tol 1.0
     Each listed motor should read motoroutput ~ 0.

Encoder convention (same as calibrate12.py / the 2-DOF arm): joint_deg =
encoder * ENCODER_GAIN, i.e. the 0x9C read is already the OUTPUT angle -- no
10:1 gear divide.

Exit status is non-zero if a listed motor never replied (or, with --tol, is off
zero), so the tool is scriptable.
"""
import argparse

from motorbus import MotorBus
from calibrate12 import (DEFAULT_RATE_HZ, bring_up_limp, sample_positions,
                         observe, verify, fold180)

DEFAULT_IDS = [1, 2, 3]        # the re-assembled leg


def parse_ids(text):
    ids = [int(t) for t in text.replace(",", " ").split()]
    if not ids:
        raise argparse.ArgumentTypeError("--ids: give at least one CAN id")
    for mid in ids:
        if not 1 <= mid <= 12:
            raise argparse.ArgumentTypeError(f"--ids: id {mid} outside 1..12")
    if len(set(ids)) != len(ids):
        raise argparse.ArgumentTypeError(f"--ids: duplicate id in {ids}")
    return ids


def set_zero_subset(mb, settle_s, rate_hz=DEFAULT_RATE_HZ, assume_yes=False):
    """0x19 set-zero of exactly mb.ids at the current pose. Same guards as
    calibrate12.set_zero (refuse on a partial set; re-arm after the blocking
    prompt; refuse to write to a still-latched motor), with subset-correct
    counts."""
    ids = mb.ids
    n = len(ids)
    pos = sample_positions(mb, dur_s=settle_s, rate_hz=rate_hz)
    missing = [mid for mid in ids if pos[mid]["joint_deg"] is None]
    if missing:
        raise SystemExit(f"[set-zero] no reply from ids {missing} -- refusing to "
                         "zero a partial set. Fix the bus/power and retry.")

    print(f"\nAbout to hardware-zero (0x19) these {n} motors at their CURRENT pose:\n")
    print(f"  {'id':>3}  {'encoder':>7}  {'motoroutput':>11}")
    for mid in ids:
        jd = fold180(pos[mid]["joint_deg"])
        print(f"  {mid:>3}  {pos[mid]['enc']:>7}  {jd:>+11.2f}")
    print("\n!! 0x19 writes each driver's encoder offset. The vendor warns this "
          "affects\n   the DRIVER LIFETIME -- use it rarely. It takes effect only "
          "after a\n   POWER-CYCLE.")
    print(f"   Motors NOT in {ids} are not written and are not even streamed here.")

    if not assume_yes:
        # The blocking prompt stops the keep-alive stream, so every listed motor
        # will set its input-lost latch (0x80) while it waits -- we re-arm below
        # before writing, and refuse to write 0x19 to a latched driver.
        ans = input(f"\nType 'yes' to write the zero to ids {ids} > ").strip().lower()
        if ans != "yes":
            print("Aborted -- no zero written.")
            return 1

    for mid in ids:
        mb.rec(mid).error = None        # drop stale pre-prompt telemetry
    if not bring_up_limp(mb, rate_hz=rate_hz, timeout_s=5.0, quiet=True):
        stuck = [m for m in ids if (mb.rec(m).error or 0x80) & 0x80]
        raise SystemExit(f"[set-zero] could not clear ids {stuck} over CAN before "
                         "writing -- aborting so NO zero is written to a latched "
                         "motor. Power-cycle those and retry.")
    offsets = mb.set_zero_all(rate_hz=rate_hz, targets=ids)

    print("\n  id   new encoder_offset   ack")
    n_ack = 0
    for mid in ids:
        off = offsets[mid]
        if off is None:
            print(f"  {mid:>3}   {'--':>16}   NO ACK <--")
        else:
            print(f"  {mid:>3}   {off:>16}   ok")
            n_ack += 1
    print(f"\n{n_ack}/{n} motors acknowledged the zero write.")
    if n_ack < n:
        print("Some motors did not ACK -- re-run --set-zero (a fresh keep-alive "
              "stream).")
    print("\nDONE. POWER-CYCLE the motors (the 0x19 zero only takes effect on "
          f"restart),\nthen run `python calibrate_leg.py --verify --ids "
          f"{','.join(str(m) for m in ids)} --tol 1.0` to confirm\nevery listed "
          "motor reads motoroutput ~ 0 at the zero pose.")
    return 0 if n_ack == n else 1


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--observe", action="store_true",
                   help="live zero-torque read of the listed ids (id + direction check)")
    g.add_argument("--verify", action="store_true",
                   help="read the listed ids at the zero pose (motoroutput ~ 0)")
    g.add_argument("--set-zero", action="store_true",
                   help="HARDWARE 0x19 set-zero of the listed ids at the current pose")
    ap.add_argument("--ids", type=parse_ids, default=DEFAULT_IDS,
                    help="CAN ids to act on, e.g. 1,2,3 (default 1,2,3)")
    ap.add_argument("--yes", action="store_true",
                    help="skip the confirmation prompt for --set-zero")
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ,
                    help=f"keep-alive stream rate per motor, Hz (default {DEFAULT_RATE_HZ:g})")
    ap.add_argument("--settle", type=float, default=0.4,
                    help="encoder averaging window for verify/set-zero, s (default 0.4)")
    ap.add_argument("--tol", type=float, default=None,
                    help="verify: flag motors whose |motoroutput| exceeds this (deg)")
    ap.add_argument("--timeout", type=float, default=None,
                    help="bring-up timeout in s (default: wait indefinitely)")
    args = ap.parse_args()

    ids = args.ids
    print(f"calibrate_leg: acting on ids {ids} only. They come up at ZERO torque "
          "(back-drivable) --\nnothing is commanded to move; a 0x80-latched motor "
          "is cleared over CAN (no power cycle).")
    with MotorBus(ids) as mb:
        if not bring_up_limp(mb, rate_hz=args.rate, timeout_s=args.timeout):
            raise SystemExit("[calibrate_leg] bring-up failed -- see above.")
        if args.observe:
            observe(mb, rate_hz=args.rate)
        elif args.verify:
            n_missing, n_off = verify(mb, args.settle, rate_hz=args.rate, tol=args.tol)
            if n_missing or n_off:
                raise SystemExit(1)
        else:
            if set_zero_subset(mb, args.settle, rate_hz=args.rate,
                               assume_yes=args.yes):
                raise SystemExit(1)
    print("[calibrate_leg] bus closed. The input-lost latch sets ~10 ms after the "
          "stream ends,\nbut the next run clears it over CAN -- no power cycle needed.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted.")
