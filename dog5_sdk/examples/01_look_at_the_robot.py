#!/usr/bin/env python3
"""Start here.  Prints the robot, checks the kinematics, opens the viewer.

    python examples/01_look_at_the_robot.py
    python examples/01_look_at_the_robot.py --viewer

No robot, no CAN, no torque -- this is the model and the geometry only.  If
this runs, your install is complete.
"""
import argparse

import numpy as np

import dog5_sdk as d5
from dog5_sdk import ik, kinematics as kin, poses, statics


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--viewer", action="store_true",
                        help="open the interactive MuJoCo viewer")
    parser.add_argument("--pose", default="stand",
                        choices=["zero", "crouch", "stand"])
    args = parser.parse_args()

    print(f"dog5_sdk {d5.__version__}")
    print(f"model    {d5.MODEL_XML}")
    print(f"mass     {statics.total_mass():.4f} kg, of which the four legs are "
          f"{4 * statics.leg_mass():.3f} kg "
          f"({400 * statics.leg_mass() / statics.total_mass():.0f} %)")
    print(f"legs     {poses.LEGS}")
    print(f"joints   {poses.JOINT_LABELS}")
    print()

    print("CAN map (the order every 12-vector is in):")
    print(f"  {'idx':>3}  {'joint':<10} {'CAN':>3}  {'dir':>3}  {'model body'}")
    for index, joint in enumerate(d5.HARDWARE_JOINTS):
        print(f"  {index:>3}  {joint.leg + '_' + joint.joint:<10} "
              f"{joint.can_id:>3}  {joint.direction:>+3}  {joint.model_name}")
    print()

    # --- forward kinematics -------------------------------------------
    q = poses.stand_pose(0.15)
    print(f"stand_pose(0.15) hip height = {poses.hip_height(q):.4f} m")
    for leg in poses.LEGS:
        foot = kin.foot_position(leg, q[poses.leg_slice(leg)])
        print(f"  {leg} foot in the trunk frame: "
              f"[{foot[0]:+.4f} {foot[1]:+.4f} {foot[2]:+.4f}] m")
    print()

    # --- inverse kinematics round-trips -------------------------------
    target = kin.foot_position("FL", q[poses.leg_slice("FL")])
    solved = ik.leg_ik("FL", target, q_seed=poses.Q_CROUCH[poses.leg_slice("FL")])
    residual = np.linalg.norm(kin.foot_position("FL", solved) - target)
    print(f"IK round-trip on FL: {residual * 1e9:.3f} nm from the target")

    # --- the Jacobian, and what gravity costs the motors ---------------
    jac = kin.foot_jacobian("FL", q[poses.leg_slice("FL")])
    print(f"FL foot Jacobian condition number: {np.linalg.cond(jac):.1f}")
    tau_g = kin.leg_gravity_torque("FL", q[poses.leg_slice("FL")])
    print(f"FL leg-gravity torque (motors hold this with no foot load): "
          f"[{tau_g[0]:+.3f} {tau_g[1]:+.3f} {tau_g[2]:+.3f}] N*m")
    print()

    # --- and the model agrees with all of it ---------------------------
    tau_model, tau_ours = statics.verify_against_model(q.reshape(4, 3))
    print("MuJoCo floating-base inverse dynamics vs this SDK's statics: "
          f"max difference {np.max(np.abs(tau_model - tau_ours)):.2e} N*m")

    if args.viewer:
        print("\nopening the viewer -- close the window to exit")
        with d5.Dog5Sim(pose=args.pose, viewer=True, realtime=True) as robot:
            robot.run(d5.ZeroTorque(), duration=30.0, keys=True, verbose=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
