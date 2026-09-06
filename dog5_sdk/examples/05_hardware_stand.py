#!/usr/bin/env python3
"""The example-02 controller, unchanged, on the real robot.

    # on the robot host, with can0 up at 1 Mbit/s:
    sudo ip link set can0 up type can bitrate 1000000
    sudo HOME=$HOME chrt -f 50 python3 examples/05_hardware_stand.py

    python examples/05_hardware_stand.py --dry-run     # no CAN, checks only

READ THIS BEFORE YOU RUN IT
    This moves a 5.8 kg machine with twelve motors that can bite.

    1. SUSPEND THE ROBOT for the first runs.  Not "hold it".
    2. Leave ``--tau-max`` at 1.0 for the first run.  Raise it in stages, from
       the logs, and never past 3.0 without a reason you can state.
    3. Have the power switch within reach, and know that ``x`` stops the run,
       space limps it, and Ctrl-C does both.
    4. Run the same controller in simulation first
       (``examples/02_sim_stand.py``).  Every trip you find there is one you
       do not find with the robot in the air.

WHAT THE RUN DOES
    crouch (the drivers' own 0xA4 position loop)  ->  ENTER  ->  torque, and
    the reference ramps from the crouch to a stand over ``--rise`` seconds ->
    hold -> soft stop -> back to the crouch in position mode.

    The controller is imported from ``02_sim_stand.py``.  That is not a
    shortcut for the example's sake; it is the point of the SDK.  The class
    is not modified, subclassed or wrapped for hardware.
"""
import argparse
import importlib.util
import os
import sys

import numpy as np

import dog5_sdk as d5


def _rise_controller_class():
    """Import RiseAndHold from 02_sim_stand.py (its name is not an identifier)."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "02_sim_stand.py")
    spec = importlib.util.spec_from_file_location("sim_stand_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.RiseAndHold


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--height", type=float, default=0.15,
                        help="hip height to stand at, m")
    parser.add_argument("--rise", type=float, default=8.0,
                        help="seconds of reference ramp.  Slower than the "
                             "simulator's on purpose")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--tau-max", type=float, default=1.0,
                        help="torque cap, N*m.  START HERE.")
    parser.add_argument("--kp", type=float, default=8.0)
    parser.add_argument("--kd", type=float, default=0.4)
    parser.add_argument("--no-park", action="store_true",
                        help="leave the robot standing instead of returning "
                             "it to the crouch")
    parser.add_argument("--dry-run", action="store_true",
                        help="check the config and the pose maths, open no bus")
    args = parser.parse_args()

    RiseAndHold = _rise_controller_class()

    if args.tau_max > d5.safety.TAU_STAGED_MAX:
        print(f"refusing tau_max={args.tau_max}: the staged ceiling is "
              f"{d5.safety.TAU_STAGED_MAX} N*m.  Raise it in the source, "
              "deliberately, after reading a log from the lower setting.",
              file=sys.stderr)
        return 2

    d5.calibration.validate()
    q_stand = d5.stand_pose(args.height)
    print(f"[preflight] CAN IDs      {d5.MOTOR_IDS}")
    print(f"[preflight] directions   "
          f"{[d5.MOTOR_DIRECTIONS[mid] for mid in d5.MOTOR_IDS]}")
    print(f"[preflight] crouch -> stand, worst joint moves "
          f"{np.rad2deg(np.max(np.abs(q_stand - d5.Q_CROUCH))):.0f} deg")
    print("[preflight] stand pose on the motors' own dials (deg):")
    dials = d5.joint_rad_to_motoroutput_deg(q_stand)
    for label, mid, value in zip(d5.JOINT_LABELS, d5.MOTOR_IDS, dials):
        print(f"              {label:<10} CAN {mid:>2}  {value:+8.2f}")
    print(f"[preflight] tau cap {args.tau_max:.2f} N*m, rise {args.rise:.1f} s")

    if args.dry_run:
        print("[preflight] --dry-run: nothing was opened and nothing moved")
        return 0

    print("\n*** SUSPEND THE ROBOT.  x stops, space limps. ***\n")

    controller = RiseAndHold(args.height, args.rise, args.kp, args.kd)
    with d5.Dog5Hardware() as robot:
        robot.crouch()
        result = robot.run(controller, duration=args.duration,
                           tau_max=args.tau_max, log=True, status_hz=2.0)
        if not args.no_park and not result.tripped:
            print("[park] returning to the crouch in position mode")
            robot.crouch()

    print(result)
    path = f"dog5_stand_{int(result.duration_s)}s.npz"
    print(f"wrote {result.save(path)}")
    if result.tripped:
        print("Read the log before the next run: what tripped, and at what "
              "torque.", file=sys.stderr)
    return 0 if not result.tripped else 1


if __name__ == "__main__":
    raise SystemExit(main())
