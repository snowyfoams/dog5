#!/usr/bin/env python3
"""The template.  Copy this file, fill in `update`, run it.

    python examples/04_write_your_own.py                  # simulation
    python examples/04_write_your_own.py --fake-hardware  # the CAN path, no robot
    python examples/04_write_your_own.py --hardware       # the robot

Everything outside :class:`MyController` is boilerplate you should not need to
touch.  Everything inside it is yours.

THE CONTRACT
    ``update(state)`` is called once per control period (250 Hz in both
    backends) and returns twelve joint torques in N*m, in the canonical order
    ``[FL, FR, RL, RR] x [abd, pitch, knee]``.  Return None for zero torque.

    You get, in `state`:
        state.q, state.qd        joint angles and rates, rad and rad/s
        state.tau                MEASURED joint torque, N*m
        state.rpy, state.omega   trunk attitude and body rates, rad, rad/s
        state.contact            (4,) planted mask, FL FR RL RR
        state.z, state.v         trunk height (m) and world velocity (m/s)
        state.foot_positions()   {leg: (3,)} in the trunk frame
        state.foot_jacobians()   {leg: (3, 3)}
        state.extra              backend extras, and in sim the ground truth

    And these hooks, all optional:
        on_start(robot, state)   once, after arming
        on_stop(state, reason)   once, however the run ends
        on_estop(state, reason)  when a safety trip fires

WHAT THE HARNESS DOES FOR YOU
    Puts your torque through the safety gate (ramp, cap, joint-limit block,
    slew), runs the trips, paces the loop, and on hardware schedules the CAN
    bus and recovers the input-lost latch.  You do not write any of that, and
    it is the same code in both backends.

FOUR THINGS THAT CATCH PEOPLE OUT
    1. Do not call ``robot.move_to`` or ``robot.crouch`` from inside
       ``update``.  Those are position-mode commands and they fight your
       torque.  Use them BETWEEN runs.
    2. ``state.contact`` on hardware is whatever you declare with
       ``robot.set_contact(mask)``.  There are no foot switches.  If your
       controller has a gait clock, it owns that mask.
    3. ``state.z`` and ``state.v`` come from the planted feet.  With fewer than
       three down they are not determined -- check
       ``state.extra["estimate_valid"]`` before you multiply either by a gain.
    4. The first hardware run is ``tau_max=1.0`` with the robot SUSPENDED.
"""
import argparse

import numpy as np

import dog5_sdk as d5
from dog5_sdk import kinematics as kin  # noqa: F401  -- for your own use


class MyController(d5.Controller):
    """<your controller>.  This one holds the pose it started in."""

    def __init__(self, kp: float = 8.0, kd: float = 0.3):
        self.kp = float(kp)
        self.kd = float(kd)
        self.q_ref = None

    def on_start(self, robot, state):
        # Hold where we actually are, not a nominal pose -- on hardware the
        # legs sag under their own weight and the two differ.
        self.q_ref = state.q.copy()
        print(f"[mine] holding the start pose, kp={self.kp}, kd={self.kd}")

    def update(self, state):
        # ------------------------------------------------------------------
        # YOUR CONTROL LAW GOES HERE.  Return (12,) N*m.
        #
        # A joint-space law, as here:
        #     return self.kp * (q_ref - state.q) - self.kd * state.qd
        #
        # A Cartesian one, per leg (see examples/03 for the full version):
        #     J = kin.foot_jacobian(leg, q_leg)
        #     tau_leg = J.T @ force + kin.leg_gravity_torque(leg, q_leg)
        # ------------------------------------------------------------------
        return self.kp * (self.q_ref - state.q) - self.kd * state.qd

    def on_stop(self, state, reason):
        print(f"[mine] stopped after {state.t:.2f} s: {reason}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hardware", action="store_true",
                        help="run on the real robot over CAN")
    parser.add_argument("--fake-hardware", action="store_true",
                        help="run the FULL CAN path against twelve simulated "
                             "drivers -- no adapter, no robot.  Checks the "
                             "plumbing (IDs, directions, units, arming); "
                             "checks no physics")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--tau-max", type=float, default=None,
                        help="torque cap, N*m (default 1.0 on hardware, "
                             "6.0 in simulation)")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--log", metavar="PATH", default=None)
    args = parser.parse_args()

    tau_max = args.tau_max if args.tau_max is not None else (
        1.0 if args.hardware else 6.0)

    if args.hardware:
        robot = d5.Dog5Hardware()
    elif args.fake_hardware:
        from dog5_sdk.fake_bus import FakeDriverBus
        robot = d5.Dog5Hardware(bus=FakeDriverBus(), prompt=False)
    else:
        robot = d5.Dog5Sim(pose="crouch", viewer=args.viewer,
                           realtime=args.viewer)

    with robot:
        robot.crouch()                       # a known starting pose, both ways
        result = robot.run(MyController(), duration=args.duration,
                           tau_max=tau_max, log=bool(args.log))

    print(result)
    if args.log:
        print(f"wrote {result.save(args.log)}")
    return 0 if not result.tripped else 1


if __name__ == "__main__":
    raise SystemExit(main())
