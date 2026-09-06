#!/usr/bin/env python3
"""Sensors -> the trunk state a body controller needs: z, v, C, omega.

VERBATIM from ``torque_stand/feedback_estimator.py`` in the dog5 research
repository, from ``LEGS = st.LEGS`` down; only the import header differs.
``tools/sync_from_repo.py`` regenerates this file.

The pure functions here are what :class:`dog5_sdk.Dog5Sim` and
:class:`dog5_sdk.Dog5Hardware` use to fill :class:`dog5_sdk.RobotState`, so a
controller sees the SAME height, velocity and attitude definitions in
simulation and on the robot.  ``BodyState`` is the full hardware object and
needs a live AHRS.
"""
from __future__ import annotations

import math

import numpy as np

from . import params as P
from . import statics as st

LEGS = st.LEGS
N_LEGS = len(LEGS)


def C_from_rp(roll, pitch):
    """I->B rotation whose (roll, pitch) is exactly the pair given.

    The exact inverse of dynamic_model.attitude_rp, so a state built here and
    read back by the model round-trips.

    YAW IS STILL ABSENT HERE, AND THAT IS NOT AN OVERSIGHT NOW THAT YAW IS
    CONTROLLED (2026-08-21).  The frame this returns is the world rotated by
    -yaw about the vertical, so its z axis IS true vertical -- a rotation
    about z cannot tilt z.  Every consumer of C is therefore unaffected by
    the heading:

        leg odometry      returns v in a yaw-following frame.  x/y have no
                          absolute reference anyway (kp is 0 on both), so a
                          heading-locked v would buy nothing and would make
                          the damper's axes rotate under it.
        fk_trunk_height   a height, invariant to yaw outright.
        the grasp map     moments come back about the SAME axes, so the Mz
                          the wrench asks for is a moment about true
                          vertical -- which is exactly what yaw control
                          wants.

    So the yaw ANGLE rides beside C in the state dict rather than inside it.
    Putting it into C would rotate the odometry and the force split for no
    gain, and would break the attitude_rp round-trip this module is pinned to.
    """
    g = np.array([-math.sin(pitch),
                  math.cos(pitch) * math.sin(roll),
                  math.cos(pitch) * math.cos(roll)])
    e = np.array([0.0, 0.0, 1.0])
    v = np.cross(e, g)
    c = float(np.dot(e, g))
    if float(np.linalg.norm(v)) < 1e-12:
        return np.eye(3) if c > 0 else -np.eye(3)
    vx = np.array([[0.0, -v[2], v[1]],
                   [v[2], 0.0, -v[0]],
                   [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx / (1.0 + c)


def leg_odometry_velocity(q, qd, C, omega, planted, frames=None):
    """World-frame trunk velocity from the planted legs.  See the header.

    Returns (v_world(3,), n_planted).  An empty planted set returns zeros and
    n=0 rather than raising -- the caller decides what too-few-feet means.
    """
    q = np.asarray(q, dtype=float)
    qd = np.asarray(qd, dtype=float)
    omega = np.asarray(omega, dtype=float)
    planted = np.asarray(planted, dtype=bool)

    acc = np.zeros(3)
    n = 0
    for i in range(N_LEGS):
        if not planted[i]:
            continue
        sl = slice(3 * i, 3 * i + 3)
        fr = st.leg_frames(LEGS[i], q[sl]) if frames is None else frames[i]
        foot, anchors, axes, _ = fr
        J = st.foot_jacobian_from(foot, anchors, axes)
        # the foot's body-frame velocity relative to the trunk, plus what the
        # trunk's own rotation contributes at that lever arm
        acc += -(np.cross(omega, foot) + J @ qd[sl])
        n += 1
    if n == 0:
        return np.zeros(3), 0
    return C.T @ (acc / n), n


def hip_from_imu(z_imu, C=None):
    """floor->trunk-bottom  ->  floor->hip-axis.  The ONE conversion.

    The offset is a BODY-frame constant [0, 0, -d], so its vertical component
    shrinks with the trunk's tilt -- hence C[2,2] rather than a flat add.  At
    the couple of degrees a stand holds, the two differ by ~0.02 mm; the factor
    is here so the convention is right, not because it is worth 0.02 mm.
    """
    cz = 1.0 if C is None else float(C[2, 2])
    return float(z_imu) + P.IMU_BELOW_TRUNK_ORIGIN_M * cz


def imu_from_hip(z_hip, C=None):
    """floor->hip-axis  ->  floor->trunk-bottom.  The exact inverse."""
    cz = 1.0 if C is None else float(C[2, 2])
    return float(z_hip) - P.IMU_BELOW_TRUNK_ORIGIN_M * cz


def fk_trunk_height(q, C, planted, frames=None, ref="imu"):
    """Height of a trunk reference point above the planted feet (m).

    Drift-free by construction: encoders and attitude, no integration.  The
    foot site is the CENTRE of a 20 mm sphere, so the floor is FOOT_RADIUS_M
    below it.

    `ref` picks WHICH POINT, and every "trunk height" in this repo has to say:

        "imu"  (default) the TRUNK BOTTOM, where the IMU board is bolted.
               IMU_BELOW_TRUNK_ORIGIN_M under the hip axis.  This is the point
               a ruler can actually reach, so it is the only one an operator
               can falsify, and it is what the height loop controls.
        "hip"  the FK trunk origin = the hip-axis plane, where dog5.xml puts
               all four hip bodies.  The frame the leg tables and the IK work
               in.  Nothing physical sits here.

    THE DEFAULT CHANGED ON 2026-08-17 and it moved the reported number by
    38 mm: this function used to return "hip" unconditionally, the runner
    printed 191 mm, and a ruler on the trunk bottom read ~160.  See
    params.IMU_BELOW_TRUNK_ORIGIN_M.
    """
    if ref not in ("imu", "hip"):
        raise ValueError(f"ref must be 'imu' or 'hip', got {ref!r}")
    q = np.asarray(q, dtype=float)
    planted = np.asarray(planted, dtype=bool)
    drops = []
    for i in range(N_LEGS):
        if not planted[i]:
            continue
        sl = slice(3 * i, 3 * i + 3)
        fr = st.leg_frames(LEGS[i], q[sl]) if frames is None else frames[i]
        drops.append(-(C.T @ fr[0])[2])       # foot site in world-aligned axes
    if not drops:
        return float("nan")
    h_hip = float(np.mean(drops)) + P.FOOT_RADIUS_M
    return h_hip if ref == "hip" else imu_from_hip(h_hip, C)


def fk_attitude_from_feet(feet):
    """Trunk roll/pitch relative to the plane through `feet` (body frame).

    Feet on one flat floor satisfy u . s_i = const, where u is the floor normal
    in BODY coordinates.  A least-squares plane through the measured foot
    positions, z = d + b*x + c*y, hands that normal back as u ~ (-b, -c, 1) --
    and u is exactly C @ e_z, which is what dynamic_model.attitude_rp reads an
    attitude out of.  So this comes out in the SAME convention as the AHRS
    branch and the two can be subtracted directly.

    Needs 3+ non-collinear planted feet; returns (nan, nan) otherwise.
    """
    feet = np.asarray(feet, dtype=float)
    if len(feet) < 3:
        return float("nan"), float("nan")
    A = np.column_stack([np.ones(len(feet)), feet[:, 0], feet[:, 1]])
    if np.linalg.matrix_rank(A, tol=1e-9) < 3:          # collinear anchors
        return float("nan"), float("nan")
    (_d, b, c), *_ = np.linalg.lstsq(A, feet[:, 2], rcond=None)
    u = np.array([-b, -c, 1.0])
    u /= np.linalg.norm(u)
    return (math.atan2(u[1], u[2]),
            math.atan2(-u[0], math.hypot(u[1], u[2])))


def fk_attitude(q, planted, frames=None):
    """`fk_attitude_from_feet` on the MEASURED encoders.  No IMU anywhere.

    WHAT IT IS: the trunk's attitude relative to the plane its feet stand on,
    from joint angles alone.  Number for number comparable with the AHRS's
    roll/pitch, which is the whole point -- it is the only independent read of
    attitude this track has, and without it nothing checks the AHRS at all.

    WHAT IT IS NOT: an attitude relative to gravity.  It cannot be; nothing in
    the legs knows which way is down.  It equals the true attitude only when
    the foot plane happens to be horizontal.

    So AHRS - FK is  floor slope + IMU mount tilt + encoder-zero error.  Those
    are three constants that one reading cannot separate, but they do not move
    on a fixed floor, which is what makes the difference worth printing: a
    CHANGE in it is real, and rotating the robot 180 deg on the same spot flips
    the floor's share and not the mount's.

    Computed from MEASURED angles, so it is not trivially zero even though the
    commanded pose puts all four feet at one z: the legs sag unequally under
    load, and that sag shows up here.
    """
    q = np.asarray(q, dtype=float)
    planted = np.asarray(planted, dtype=bool)
    feet = [(st.leg_frames(LEGS[i], q[3 * i:3 * i + 3]) if frames is None
             else frames[i])[0]
            for i in range(N_LEGS) if planted[i]]
    return fk_attitude_from_feet(feet)


class BodyState:
    """AHRS + encoders -> the state dict, or an honest refusal.

    `read()` returns (state, active, reason).  `active` is False -- and the
    caller must fall back rather than command a wrench -- when the AHRS has
    gone stale or too few feet are planted.  It never returns a stale-but-
    plausible state.
    """

    def __init__(self, ahrs, setpoint_roll=0.0, setpoint_pitch=0.0,
                 lpf_fc_hz=P.ODOM_LPF_FC_HZ, stale_s=P.AHRS_STALE_S,
                 min_planted=P.MIN_PLANTED):
        self.ahrs = ahrs
        self.sp_roll = float(setpoint_roll)
        self.sp_pitch = float(setpoint_pitch)
        self.lpf_fc_hz = float(lpf_fc_hz)
        self.stale_s = float(stale_s)
        self.min_planted = int(min_planted)
        self.v = np.zeros(3)              # low-passed odometry velocity
        self._last_t = None
        self.n_planted = 0
        self.roll = self.pitch = 0.0            # AHRS, setpoint subtracted
        self.roll_raw = self.pitch_raw = 0.0    # AHRS as it arrived
        # THE HEADING, AS IT ARRIVES.  No setpoint is subtracted and none can
        # be: roll and pitch have a rest value this rig measurably sits at,
        # but a heading has no such thing -- "level" is a fact about the
        # floor, "north" is not a fact about the robot.  What makes a yaw
        # error mean something is the LOCK the runner latches when torque
        # arms, which lives there and not here.
        self.yaw = 0.0
        self.roll_fk = self.pitch_fk = float("nan")   # from the feet, no IMU
        self.z = float("nan")             # floor -> TRUNK BOTTOM (controlled)
        self.z_hip = float("nan")         # floor -> hip axis (the IK's frame)

    def read(self, now, q, qd, planted, frames=None):
        planted = np.asarray(planted, dtype=bool)
        sample = self.ahrs.sample()
        if sample is None or self.ahrs.is_stale(self.stale_s):
            return None, False, "AHRS stale"
        if int(planted.sum()) < self.min_planted:
            return None, False, (f"only {int(planted.sum())} planted feet; "
                                 f"leg odometry needs {self.min_planted}")

        # The setpoints are the RESTING attitude measured on this rig, not
        # zero: the IMU is bolted to the trunk through a mechanical mount, and
        # that mount's tilt is in every reading.  Subtracting it here is what
        # makes "level" mean level.
        self.roll_raw = math.radians(sample.roll_deg)
        self.pitch_raw = math.radians(sample.pitch_deg)
        self.roll = self.roll_raw - self.sp_roll
        self.pitch = self.pitch_raw - self.sp_pitch
        self.yaw = math.radians(sample.yaw_deg)
        C = C_from_rp(self.roll, self.pitch)
        omega = np.deg2rad([sample.roll_rate_dps,
                            sample.pitch_rate_dps,
                            sample.yaw_rate_dps])

        q = np.asarray(q, dtype=float)
        if frames is None:
            frames = [st.leg_frames(LEGS[i], q[3 * i:3 * i + 3])
                      for i in range(N_LEGS)]
        v_raw, n = leg_odometry_velocity(q, qd, C, omega, planted, frames)
        # Floor to TRUNK BOTTOM: the point a ruler reaches and the one the
        # height loop controls.  The hip-axis value comes out of the same
        # average and is kept because the leg IK works in that frame.
        z = fk_trunk_height(q, C, planted, frames, ref="imu")
        z_hip = fk_trunk_height(q, C, planted, frames, ref="hip")
        # The independent attitude.  Costs one 4x3 least squares per outer
        # step and is the ONLY thing in this track that can contradict the
        # AHRS -- without it a mount tilt is indistinguishable from a real one.
        roll_fk, pitch_fk = fk_attitude(q, planted, frames)
        self.n_planted = n
        self.z, self.z_hip = z, z_hip
        self.roll_fk, self.pitch_fk = roll_fk, pitch_fk

        # First-order low pass.  The only filter state in the module, and it is
        # on a VELOCITY, not a position, so it cannot integrate an error away.
        dt = P.SWEEP_S if self._last_t is None else max(now - self._last_t, 1e-4)
        self._last_t = now
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.lpf_fc_hz * dt)
        self.v += alpha * (v_raw - self.v)

        return {
            # only r[2] is ever read; x/y stay at zero rather than pretending
            # to an origin that nothing observes
            "r": np.array([0.0, 0.0, z]),
            "v": self.v.copy(),
            "C": C,
            "w": omega,
            # THE HEADING RIDES BESIDE C, NOT INSIDE IT -- see C_from_rp.  It
            # is an absolute angle in (-pi, pi], so anything differencing it
            # must wrap; dynamic_model.body_wrench is the only thing that
            # does, and it does.
            "yaw": self.yaw,
            "z_fk": z,              # trunk bottom -- what r[2] is
            "z_hip": z_hip,         # hip axis -- what the leg IK wants
            "roll_fk": roll_fk,
            "pitch_fk": pitch_fk,
            "n_planted": n,
            "v_raw": v_raw,
        }, True, ""

    def attitude_residual(self):
        """AHRS - FK, in radians.  floor slope + mount tilt + encoder zeros."""
        return (self.roll - self.roll_fk, self.pitch - self.pitch_fk)

    @staticmethod
    def status_legend():
        """Printed ONCE, above the stream, because a status line that needs a
        legend and does not have one is a status line nobody reads."""
        return (
            "  height  floor to the TRUNK BOTTOM, the point a ruler reaches\n"
            "  tilt    roll/pitch the controller acts on = AHRS - setpoint\n"
            "  feet    the same two angles from the FOOT PLANE, no IMU in it\n"
            "  diff    tilt - feet = floor slope + IMU mount tilt + encoder\n"
            "          zeros.  Should be a CONSTANT; if it moves, something\n"
            "          the loop believes is not true\n"
            "  vel     trunk velocity (x,y,z) from leg odometry\n"
            "  feet-N  how many feet the estimator is counting as planted")

    def status(self):
        dr, dp = self.attitude_residual()
        return (f"height {self.z*1e3:4.0f}mm  "
                f"tilt {math.degrees(self.roll):+5.2f}/"
                f"{math.degrees(self.pitch):+5.2f}  "
                f"feet {math.degrees(self.roll_fk):+5.2f}/"
                f"{math.degrees(self.pitch_fk):+5.2f}  "
                f"diff {math.degrees(dr):+5.2f}/{math.degrees(dp):+5.2f} deg  "
                f"vel {self.v[0]*1e3:+4.0f},{self.v[1]*1e3:+4.0f},"
                f"{self.v[2]*1e3:+4.0f} mm/s  "
                f"feet-{self.n_planted}")


