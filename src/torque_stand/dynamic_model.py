#!/usr/bin/env python3
"""The model: trunk state error -> the 6D wrench the body needs from the floor.

    state (z, v, C, omega)  +  z_des  ->  W = [Fx Fy Fz Mx My Mz]

This is the ONLY place a control gain acts on the trunk.  Everything
downstream (force_totorque) is bookkeeping: how to split W across four feet
and how to turn each foot force into three joint torques.  Everything upstream
(feedback_estimator) is measurement.  So this file is the controller.

WHAT MODEL, EXACTLY -- IT IS QUASI-STATIC, AND THAT IS DELIBERATE
    The trunk is treated as a single rigid body whose acceleration is ~0:

        F = virtual spring/damper on {z}  +  m*g          (the weight)
        M = virtual spring/damper on {roll, pitch}

    There is NO inertia term -- no I*alpha, no omega x (I omega).  For a robot
    standing still that is not an approximation being smuggled in, it is the
    correct reduction: alpha and omega are both ~0, so both terms are ~0.  The
    virtual spring/damper IS the control law; m*g is what makes the feet push
    at all.

    What this buys: the whole model is six lines, every gain has a unit you
    can check by hand, and the sim gates V1-V4 that validated this exact form
    still apply.  What it costs: it cannot command a deliberate acceleration.
    A jump, a fast squat, or a trot push-off would need the inertia terms and
    the trunk inertia tensor out of dog5.xml.  Standing does not.

STIFF ONLY WHERE THE STATE IS OBSERVABLE
    z, roll and pitch get a spring AND a damper.
    x, y and yaw get a damper ONLY.

    That asymmetry is not tuning.  Without an EKF, x and y come from leg
    odometry, which measures VELOCITY and has no absolute origin -- there is
    no position to be stiff to, so a kp_x would be a spring anchored to an
    arbitrary point that drifts.  Yaw is worse: the AHRS's yaw is
    magnetometer-derived, next to twelve motors and a steel frame.  Damping
    those three axes still resists being pushed, which is all that is wanted.

RUN
    A library: no bus, no IMU, no state of its own -- `body_wrench` is a pure
    function, which is why the whole file is testable offline.
    cd <repo>/src
    python3 torque_stand/dynamic_model.py --self-test        # no hardware
"""
from __future__ import annotations

import os, sys                                                  # noqa: E401
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dog5_paths  # noqa: E402,F401  -- every src/ dir onto sys.path

import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))

import params as P                                        # noqa: E402


def attitude_rp(C):
    """Roll and pitch (rad) from C (I->B), ignoring yaw.

    g_b is the inertial up-axis expressed in the body frame, so a level robot
    gives g_b = [0, 0, 1] -> roll = pitch = 0.  The exact inverse of
    feedback_estimator.C_from_rp.
    """
    g_b = C @ np.array([0.0, 0.0, 1.0])
    roll = math.atan2(g_b[1], g_b[2])
    pitch = math.atan2(-g_b[0], math.hypot(g_b[1], g_b[2]))
    return roll, pitch


def wrap_pi(a):
    """Wrap an angle to (-pi, pi].  The only safe way to difference a heading."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def body_wrench(state, z_des, mass=P.MASS_KG, v_cmd=(0.0, 0.0, 0.0),
                yawrate_cmd=0.0, gains=None, yaw_ref=None):
    """The wrench the four feet must produce between them, in WORLD axes.

    `gains` is any object carrying the kp_*/kd_* attributes; None uses the
    params defaults.  Returned as [Fx, Fy, Fz, Mx, My, Mz]: a force at the CoM
    and moments about the world axes, valid for the small tilts a stand holds.

    THE m*g TERM IS THE WHOLE POINT.  Drop it and the springs only correct
    ERROR -- a robot sitting exactly at its target height would be told to
    push with zero force and would fall.  The springs trim; gravity is what
    holds it up.

    YAW GAINED A STIFFNESS ON 2026-08-21, AND IT IS OPT-IN THREE TIMES OVER.
    The heading was damping-only for as long as imu_dog.py called the DETA10's
    magnetometer yaw untrusted next to twelve motors and a steel frame.  It
    was then watched under power through att_web and found to hold, so the
    ANGLE is now available -- but every existing caller must be unaffected,
    so the term needs all three of `yaw_ref` passed, `state["yaw"]` present
    and a non-zero `kp_yaw` on the gains object before it contributes
    anything.  The stand passes none of them and its wrench is unchanged bit
    for bit; the self-test pins that.

    `yaw_ref` IS A LOCK THE CALLER LATCHES, not a constant like the level
    reference roll and pitch use.  There is no absolute heading a robot ought
    to have, so the reference is "wherever you were pointing when the torque
    armed" and the runner owns it.

    THE ERROR IS WRAPPED AND THEN CLAMPED, and both matter.  Wrapped, because
    a robot at +179 deg against a -179 deg lock has a 2 deg error and not a
    358 deg one.  Clamped, because yaw authority is 100% friction -- a
    diagonal pair makes a yaw couple entirely out of tangential force -- so an
    unbounded error would ask for a moment the cone cannot deliver and the
    distributor would answer by slewing every foot's tangential force at once.
    The clamp is `gains.yaw_err_max` when the gains object carries one.
    """
    g = _Gains() if gains is None else gains
    z = float(np.asarray(state["r"])[2])
    v = np.asarray(state["v"], dtype=float)
    w = np.asarray(state["w"], dtype=float)
    roll, pitch = attitude_rp(state["C"])
    v_cmd = np.asarray(v_cmd, dtype=float)

    Fz = g.kp_z * (z_des - z) + g.kd_z * (0.0 - v[2]) + mass * P.GRAVITY
    Fx = g.kd_x * (v_cmd[0] - v[0])          # damping-only axes: no kp, see
    Fy = g.kd_y * (v_cmd[1] - v[1])          # the header
    Mx = g.kp_roll * (0.0 - roll) + g.kd_roll * (0.0 - w[0])
    My = g.kp_pitch * (0.0 - pitch) + g.kd_pitch * (0.0 - w[1])
    Mz = g.kd_yaw * (yawrate_cmd - w[2])
    # The stiffness half, absent unless the caller opted in on all three
    # counts.  `state` is a plain dict everywhere, so .get is the test for
    # "this estimator publishes a heading at all".
    kp_yaw = float(getattr(g, "kp_yaw", 0.0))
    yaw = state.get("yaw") if hasattr(state, "get") else None
    if kp_yaw and yaw is not None and yaw_ref is not None:
        e_yaw = wrap_pi(float(yaw_ref) - float(yaw))
        e_max = float(getattr(g, "yaw_err_max", P.YAW_ERR_MAX_RAD))
        Mz += kp_yaw * max(-e_max, min(e_max, e_yaw))
    return np.array([Fx, Fy, Fz, Mx, My, Mz])


class _Gains:
    """The params values as an object, so a runner can override one of them
    (`g = default_gains(); g.kp_roll = 0` is the ablation half of an A/B)
    without editing params.py."""

    def __init__(self):
        self.kp_z, self.kd_z = P.KP_Z, P.KD_Z
        self.kp_roll, self.kd_roll = P.KP_ROLL, P.KD_ROLL
        self.kp_pitch, self.kd_pitch = P.KP_PITCH, P.KD_PITCH
        self.kd_x, self.kd_y, self.kd_yaw = P.KD_X, P.KD_Y, P.KD_YAW
        self.kp_yaw = P.KP_YAW
        self.yaw_err_max = P.YAW_ERR_MAX_RAD
        self.kd_joint = P.KD_JOINT

    def __repr__(self):
        # kd_yaw IS PRINTED even though it is the least interesting gain: it
        # used to be the one omitted, so a runner that zeroed everything the
        # banner showed still had 4.0 Nms/rad live on the yaw axis and the
        # banner read as all-off.  Every gain the wrench uses, or none.
        # Two decimals on anything that can sensibly be a fraction: at :.0f a
        # live 0.5 damper prints as "0" and the banner reads as all-off, which
        # is the exact failure the paragraph above records.
        return (f"Gains(kp_z={self.kp_z:.0f} kd_z={self.kd_z:.0f} "
                f"kp_att={self.kp_roll:.0f} kd_att={self.kd_roll:.2f} "
                f"kd_xy={self.kd_x:.2f} kd_yaw={self.kd_yaw:.2f} "
                f"kp_yaw={self.kp_yaw:.2f})")


def default_gains():
    return _Gains()


def height_ramp(z_from, z_to, u):
    """Smoothstepped height reference, `u` in [0, 1] along the rise.

    The reference is ABSOLUTE and must start at the MEASURED crouch height,
    never at the stand height.  This is not a nicety: Fz is kp_z*(z_des - z),
    so starting a 0.19 m reference while the robot sits at a 0.04 m crouch
    demands 0.8*(0.15)*1000 = ~120 N of extra support on a 57 N robot -- three
    times its weight, on the first sweep, before the ramp has done anything.
    """
    u = float(np.clip(u, 0.0, 1.0))
    s = u * u * (3.0 - 2.0 * u)
    return float(z_from) + s * (float(z_to) - float(z_from))


