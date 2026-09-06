#!/usr/bin/env python3
"""A controller that stands the robot up, in simulation.

    python examples/02_sim_stand.py
    python examples/02_sim_stand.py --viewer --realtime
    python examples/02_sim_stand.py --log run.npz

This is the smallest thing that is a real controller: a joint-space PD whose
REFERENCE ramps from the crouch to a standing pose over `--rise` seconds.  The
ramp is not decoration -- a PD handed a step target accelerates a joint hard
enough to trip the overspeed guard, which you can see for yourself with
``--rise 0.1``.

Read it as the template for the shape of a controller: state in, twelve
torques out, no reaching into the robot object, nothing that knows whether it
is talking to MuJoCo or to a CAN bus.  ``examples/04_hardware_stand.py`` runs
this exact class on the machine.
"""
import argparse

import numpy as np

import dog5_sdk as d5


class RiseAndHold(d5.Controller):
    """Ramp a joint-PD reference from wherever we start to a standing pose.

        tau = kp (q_ref(t) - q) - kd qd

    kp/kd are joint-space, N*m/rad and N*m*s/rad.  The values here are the
    conservative end: enough to lift 5.8 kg through the crouch, gentle enough
    that the ramp, not the gain, sets how fast the robot moves.
    """

    def __init__(self, height: float = 0.15, rise_s: float = 4.0,
                 kp: float = 25.0, kd: float = 0.8):
        self.height = float(height)
        self.rise_s = float(rise_s)
        self.kp = float(kp)
        self.kd = float(kd)
        self.q_start = None
        self.q_stand = None

    def on_start(self, robot, state):
        # Start from where the robot actually IS, not from a nominal crouch:
        # on hardware those differ by the legs' sag under their own weight.
        self.q_start = state.q.copy()
        self.q_stand = d5.stand_pose(self.height)
        print(f"[rise] {self.rise_s:.1f} s ramp to a {self.height * 1e3:.0f} mm "
              f"stand; worst joint moves "
              f"{np.rad2deg(np.max(np.abs(self.q_stand - self.q_start))):.0f} deg")

    def update(self, state):
        u = min(1.0, state.t / self.rise_s)
        smooth = u * u * (3.0 - 2.0 * u)          # cubic ease, zero end slopes
        q_ref = self.q_start + smooth * (self.q_stand - self.q_start)
        return self.kp * (q_ref - state.q) - self.kd * state.qd


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--height", type=float, default=0.15,
                        help="hip height to stand at, m")
    parser.add_argument("--rise", type=float, default=4.0,
                        help="seconds of reference ramp")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--tau-max", type=float, default=8.0,
                        help="torque cap, N*m (hardware starts at 1.0)")
    parser.add_argument("--kp", type=float, default=25.0)
    parser.add_argument("--kd", type=float, default=0.8)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--log", metavar="PATH", default=None,
                        help="write the run to a .npz")
    args = parser.parse_args()

    controller = RiseAndHold(args.height, args.rise, args.kp, args.kd)
    with d5.Dog5Sim(pose="crouch", viewer=args.viewer,
                    realtime=args.realtime) as robot:
        result = robot.run(controller, duration=args.duration,
                           tau_max=args.tau_max, log=True, status_hz=1.0)
        final = robot.read()

    print()
    print(result)
    print(f"height: asked {args.height * 1e3:.0f} mm at the hips, "
          f"estimator says {final.z * 1e3:.0f} mm at the trunk bottom, "
          f"simulator says {final.extra['z_true'] * 1e3:.0f} mm at the trunk "
          "origin")
    print(f"peak torque: {np.max(np.abs(result.log['tau_cmd'])):.2f} N*m "
          f"of a {args.tau_max:.2f} N*m cap")
    print(f"estimator error vs ground truth: "
          f"{abs(final.z - (final.extra['z_true'] - 0.038)) * 1e3:.1f} mm")

    if args.log:
        print(f"wrote {result.save(args.log)}")
    return 0 if not result.tripped else 1


if __name__ == "__main__":
    raise SystemExit(main())
