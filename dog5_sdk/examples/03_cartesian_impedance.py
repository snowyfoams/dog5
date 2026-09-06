#!/usr/bin/env python3
"""A Cartesian controller: foot targets, the Jacobian, and leg gravity.

    python examples/03_cartesian_impedance.py --viewer --realtime

The joint PD in example 02 does not know it has legs.  This one does:

    per leg:   f = Kp (p_ref - p) + Kd (0 - pdot)      a force at the FOOT
               tau = J^T f  +  leg_gravity_torque(q)   the three motor torques

and that is the whole shape of every force-based quadruped controller.  What
changes in a real one is where ``p_ref`` comes from -- a body wrench split
across the stance feet, a swing arc, an MPC -- not this transmission step.

THE TERM PEOPLE LEAVE OUT
    ``J^T f`` alone assumes MASSLESS legs.  DOG5's legs are 55 % of the robot,
    so ``leg_gravity_torque`` is not a refinement: without it the stance
    torques are wrong by about 0.48 N*m, which is half the first-run torque
    cap.  :func:`dog5_sdk.statics.verify_against_model` is the check that
    settled that, against MuJoCo's floating-base inverse dynamics.

    Note the SIGNS.  ``kinematics.leg_gravity_torque`` returns the torque the
    MOTORS MUST APPLY to hold the links up, so it is ADDED.  ``f`` here is the
    force the foot must apply to the WORLD, so the joint torque that produces
    it is ``+J^T f`` -- the opposite sign from
    :func:`dog5_sdk.statics.stance_torque`, which takes the ground reaction ON
    THE BODY instead.  Pick one convention and check it against a known case.
"""
import argparse

import numpy as np

import dog5_sdk as d5
from dog5_sdk import kinematics as kin, poses


class FootImpedance(d5.Controller):
    """Hold each foot at a target point in the TRUNK frame, compliantly.

    kp_cart   N/m    how hard a foot is pulled back to its target
    kd_cart   N*s/m  damping on the foot's velocity relative to the trunk
    support   N      per-foot vertical feed-forward.  With no body controller
                     nothing else holds the trunk up, so a share of the weight
                     is fed forward; the impedance then only has to correct.
    """

    def __init__(self, height: float = 0.15, rise_s: float = 4.0,
                 kp_cart: float = 700.0, kd_cart: float = 8.0,
                 support_frac: float = 0.9):
        self.height = float(height)
        self.rise_s = float(rise_s)
        self.kp = float(kp_cart)
        self.kd = float(kd_cart)
        self.support = (support_frac * d5.statics.WEIGHT_N / 4.0)
        self.p_start = {}
        self.p_stand = poses.stand_feet(self.height)

    def on_start(self, robot, state):
        self.p_start = state.foot_positions()
        print(f"[impedance] kp={self.kp:.0f} N/m, kd={self.kd:.1f} N*s/m, "
              f"support {self.support:.1f} N per foot")

    def update(self, state):
        u = min(1.0, state.t / self.rise_s)
        blend = u * u * (3.0 - 2.0 * u)

        tau = np.zeros(d5.N_JOINTS)
        for leg in poses.LEGS:
            section = poses.leg_slice(leg)
            q_leg, qd_leg = state.q[section], state.qd[section]

            p = kin.foot_position(leg, q_leg)
            jac = kin.foot_jacobian(leg, q_leg)
            p_dot = jac @ qd_leg                       # foot velocity, trunk frame
            p_ref = self.p_start[leg] + blend * (self.p_stand[leg]
                                                 - self.p_start[leg])

            # The force this foot must push into the ground with.  +z is up in
            # the trunk frame, so pushing DOWN to hold the body up is -z.
            force = (self.kp * (p_ref - p) - self.kd * p_dot
                     + np.array([0.0, 0.0, -self.support]))
            tau[section] = jac.T @ force + kin.leg_gravity_torque(leg, q_leg)
        return tau


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--height", type=float, default=0.15)
    parser.add_argument("--rise", type=float, default=4.0)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--tau-max", type=float, default=8.0)
    parser.add_argument("--kp", type=float, default=700.0)
    parser.add_argument("--kd", type=float, default=8.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    args = parser.parse_args()

    controller = FootImpedance(args.height, args.rise, args.kp, args.kd)
    with d5.Dog5Sim(pose="crouch", viewer=args.viewer,
                    realtime=args.realtime) as robot:
        result = robot.run(controller, duration=args.duration,
                           tau_max=args.tau_max, log=True, status_hz=1.0)
        final = robot.read()

    print()
    print(result)
    print(f"foot targets vs measured, at the end:")
    for leg, p in final.foot_positions().items():
        want = controller.p_stand[leg]
        print(f"  {leg}: {np.round(p, 4)} vs {np.round(want, 4)} "
              f"-> {np.linalg.norm(p - want) * 1e3:5.1f} mm")
    return 0 if not result.tripped else 1


if __name__ == "__main__":
    raise SystemExit(main())
