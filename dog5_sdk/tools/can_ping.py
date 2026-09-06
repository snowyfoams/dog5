#!/usr/bin/env python3
"""Do all twelve motors answer?  The first thing to run on a new setup.

    sudo ip link set can0 up type can bitrate 1000000     # Linux, once
    python tools/can_ping.py

Opens the bus, streams zero-torque keep-alives while it reads every motor's
status, and prints one line per CAN ID: does it reply, what is its error byte,
its temperature, its supply voltage and its encoder angle.  Nothing is ever
commanded to move -- every motor is held at iq=0 and stays back-drivable
throughout, so this is safe with the robot on the bench.

Exit status is non-zero if any motor did not answer, so it is scriptable.

A motor that answers with error bit 0x80 (input signal lost) is NOT broken: it
stopped hearing commands for longer than its 10 ms watchdog, which is what
happens if it was powered up before this tool started.  It is cleared over CAN,
with no power cycle, by the arming ladder this tool runs.
"""
from __future__ import annotations

import argparse
import sys

import dog5_sdk as d5
from dog5_sdk.motor import motorbus


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ids", default=None,
                        help="comma-separated CAN IDs (default: the twelve "
                             "from the hardware map)")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--rate", type=float, default=250.0,
                        help="keep-alive rate per motor, Hz")
    parser.add_argument("--no-arm", action="store_true",
                        help="skip the arming ladder; report whatever the "
                             "motors say as they are")
    args = parser.parse_args()

    ids = ([int(part) for part in args.ids.split(",")] if args.ids
           else list(d5.MOTOR_IDS))
    print(f"[ping] opening the bus at {args.bitrate / 1e6:.1f} Mbit/s for "
          f"IDs {ids}")

    with motorbus.MotorBus(ids, bitrate=args.bitrate,
                           dirs=d5.MOTOR_DIRECTIONS) as mb:
        if not args.no_arm:
            print("[ping] arming (zero-torque stream + 0x9B -> 0x88 ladder); "
                  "switch 24 V on if it is off")
            if not mb.arm(rate_hz=args.rate):
                print("[ping] not every motor armed -- the table below says "
                      "which", file=sys.stderr)
        status = mb.status1()
        mb.status2()
        mb.poll()

        temps, volts = mb.temps(), mb.voltages()
        encoders = mb.encoders_deg()
        speeds = mb.speeds_dps()

        print()
        print(f"  {'CAN':>3}  {'joint':<10} {'dir':>3}  {'reply':<5} "
              f"{'error':<28} {'T':>4} {'V':>6} {'enc deg':>9} {'dps':>7}")
        missing = []
        for joint in d5.HARDWARE_JOINTS:
            mid = joint.can_id
            if mid not in ids:
                continue
            error, _state = status.get(mid, (None, None))
            replied = error is not None
            if not replied:
                missing.append(mid)
            print(f"  {mid:>3}  {joint.leg + '_' + joint.joint:<10} "
                  f"{joint.direction:>+3}  {'yes' if replied else 'NO':<5} "
                  f"{(motorbus.decode_errors(error) if replied else '-'):<28} "
                  f"{(temps.get(mid) if temps.get(mid) is not None else 0):>3}C "
                  f"{(volts.get(mid) or 0.0):>5.1f}V "
                  f"{encoders.get(mid, 0):>8.2f}  {speeds.get(mid, 0):>+6.1f}")

        print()
        print(f"[ping] bus load {mb.rr.bus_load() * 100:.1f} % of "
              f"{args.bitrate / 1e6:.0f} Mbit/s")
        if missing:
            print(f"[ping] NO REPLY from {missing} -- check the termination, "
                  "the daisy chain, and that each driver has 24 V",
                  file=sys.stderr)
            return 1
        print(f"[ping] all {len(ids)} motors answered")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
