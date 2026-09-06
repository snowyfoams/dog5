"""The joint order, the named poses, and the flat-vector <-> per-leg helpers.

THE CANONICAL ORDER, ONCE
    Every 12-vector in this SDK -- ``q``, ``qd``, ``tau``, joint limits, the
    MJCF's actuators and its ``qpos[7:]`` -- is::

        [FL, FR, RL, RR] x [abduction, hip pitch, knee]

    index = 3 * leg_index + joint_index.  :data:`JOINT_LABELS` is that order
    spelled out, and :mod:`dog5_sdk.hardware_map` maps it to CAN IDs.  Nothing
    in this package uses any other order, and the sim and the hardware use the
    same one, which is the whole point of the SDK.

THE NAMED POSES
    ZERO    all twelve joints at their calibrated zero: legs flat fore-aft,
            trunk on the ground.  This is the pose the hardware set-zero
            (0x19) was written at, so it is a fact about the robot, not a
            choice.  It is also RANK-1 SINGULAR -- every foot Jacobian loses
            vertical authority there, so a Cartesian force cannot lift the
            robot out of it.  Escape it in joint space.

    ROLL    hips abducted to +/-90 deg, legs still flat.  The intermediate the
            stand goes through so the left and right legs cannot cross the
            midline while folding, and so the knee plane ends up vertical.

    CROUCH  the pose every torque run starts and ends on, measured on the
            assembled robot and then mirrored across all four legs to take out
            its left/right bias.  ``Dog5Hardware.crouch()`` drives to exactly
            this with the drivers' own 0xA4 position loop.

    STAND   not a literal.  :func:`stand_pose` solves it, so you can ask for a
            height and get the pose rather than copying twelve numbers.
"""
from __future__ import annotations

import numpy as np

from . import ik
from . import kinematics as kin

LEGS = kin.LEGS
N_LEGS = len(LEGS)
N_JOINTS = 12

JOINT_NAMES = ("abd", "pitch", "knee")

#: The canonical order, spelled out.  ``JOINT_LABELS[i]`` names joint ``i``.
JOINT_LABELS = tuple(f"{leg}_{joint}" for leg in LEGS for joint in JOINT_NAMES)


def stack(per_leg) -> np.ndarray:
    """{leg: (3,)} (or (4, 3)) -> the (12,) vector in canonical order."""
    if isinstance(per_leg, dict):
        return np.concatenate([np.asarray(per_leg[leg], dtype=float)
                               for leg in LEGS])
    return np.asarray(per_leg, dtype=float).reshape(N_JOINTS)


def unstack(flat) -> dict:
    """The (12,) vector -> {leg: (3,)}."""
    flat = np.asarray(flat, dtype=float).reshape(N_JOINTS)
    return {leg: flat[3 * i:3 * i + 3].copy() for i, leg in enumerate(LEGS)}


def leg_slice(leg: str) -> slice:
    """The slice of a 12-vector belonging to `leg`."""
    return slice(3 * LEGS.index(leg), 3 * LEGS.index(leg) + 3)


# ---------------------------------------------------------------------------
# named poses -- degrees at the top, radians below, so the numbers stay the
# ones an operator reads off the robot
# ---------------------------------------------------------------------------
ROLL_DEG = {
    "FL": (90.0, 0.0, 0.0),
    "FR": (-90.0, 0.0, 0.0),
    "RL": (90.0, 0.0, 0.0),
    "RR": (-90.0, 0.0, 0.0),
}

#: Measured on the assembled robot, then mirrored across the four legs.
CROUCH_DEG = {
    "FL": (88.33, 48.04, -142.11),
    "FR": (-88.33, -48.04, 142.11),
    "RL": (88.33, -48.04, 142.11),
    "RR": (-88.33, 48.04, -142.11),
}

Q_ZERO = np.zeros(N_JOINTS)
Q_ROLL = np.deg2rad(stack(ROLL_DEG))
Q_CROUCH = np.deg2rad(stack(CROUCH_DEG))

# ---------------------------------------------------------------------------
# stance geometry
# ---------------------------------------------------------------------------
#: Feet this far outboard of their hips.  Wider is more stable through the
#: roll; this is the value the shipped stand uses.
STANCE_OUT_M = 0.04

#: Hip-to-foot height at the crouch and at the stand the torque track holds.
H_CROUCH_M = 0.10
H_STAND_M = 0.152

#: Radius of the foot contact sphere -- the foot SITE is its centre, so the
#: floor is this far below the site.
FOOT_RADIUS_M = 0.020


def default_foot_xy(leg: str, stance_out: float = STANCE_OUT_M) -> np.ndarray:
    """The (x, y) a foot sits at when it is directly outboard of its hip."""
    hip = np.asarray(kin.LEG_GEOMETRY[leg].hip)
    return np.array([hip[0], hip[1] + np.sign(hip[1]) * stance_out])


def stand_feet(height: float = H_STAND_M,
               stance_out: float = STANCE_OUT_M) -> dict:
    """{leg: foot position} for a square stance `height` below the hips."""
    return {leg: np.array([*default_foot_xy(leg, stance_out), -float(height)])
            for leg in LEGS}


def stand_pose(height: float = H_STAND_M, stance_out: float = STANCE_OUT_M,
               q_seed=None) -> np.ndarray:
    """The (12,) joint vector for a square stance at `height` below the hips.

    Solved, not tabulated.  Seeded from the crouch by default, which is the
    branch the robot is physically on when it stands up -- see
    :mod:`dog5_sdk.ik` on why the seed is the branch.
    """
    return ik.body_ik(stand_feet(height, stance_out),
                      Q_CROUCH if q_seed is None else q_seed)


def hip_height(q, feet_down=None) -> float:
    """Mean hip-to-foot drop for the given pose, in metres.

    The height the leg tables are written in.  Add :data:`FOOT_RADIUS_M` for
    floor-to-hip; :func:`dog5_sdk.estimator.fk_trunk_height` does the full job
    with attitude and a contact set.
    """
    q = np.asarray(q, dtype=float).reshape(N_JOINTS)
    legs = LEGS if feet_down is None else [
        leg for leg, down in zip(LEGS, feet_down) if down]
    if not legs:
        raise ValueError("hip_height needs at least one foot down")
    drops = []
    for leg in legs:
        section = leg_slice(leg)
        hip_z = kin.LEG_GEOMETRY[leg].hip[2]
        drops.append(hip_z - kin.foot_position(leg, q[section])[2])
    return float(np.mean(drops))
