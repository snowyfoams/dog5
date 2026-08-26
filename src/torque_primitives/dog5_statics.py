#!/usr/bin/env python3
from __future__ import annotations

import os, sys                                                  # noqa: E401
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dog5_paths  # noqa: E402,F401  -- every src/ dir onto sys.path

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_DESC = os.path.join(dog5_paths.SRC, "dog5_description")

import dog5_kinematics as kin                              # noqa: E402
from torque_params import DOG5_MASS_KG, GRAVITY_M_S2       # noqa: E402

LEGS = kin.LEGS
N_LEGS = len(LEGS)

# Controlled copy of the dog5.xml trunk inertial (line 37), same pattern
# dog5_kinematics uses for LEG_GEOMETRY and LINK_INERTIALS.  Gated against the
# MJCF in verify_against_model().
TRUNK_MASS_KG = 2.61890103
TRUNK_COM_BODY = np.array([0.00071848, 0.00000646, 0.00838375])

# Body-frame gravity DIRECTION (unit, pointing down) for a level trunk.  The
# sign convention below is worth stating once: `leg_gravity_torque_tilted`
# returns the torque the MOTORS MUST APPLY to hold the links up, which is the
# negative of the gravitational generalised force.  That matches
# dog5_kinematics.leg_gravity_torque and matches how the swing branch of
# dog5_vmc_core uses it (added, not subtracted).
_G_DOWN_LEVEL = np.array([0.0, 0.0, -1.0])

_LEG_MASS_KG = float(sum(li.mass for li in kin.LINK_INERTIALS["FL"]))
_TOTAL_MASS_KG = TRUNK_MASS_KG + 4.0 * _LEG_MASS_KG

# Per-link weights (N) and the "joint j carries link k" mask, hoisted out of
# leg_gravity_torque_tilted so the hot path is three array ops.  _CARRIES is
# upper-triangular because the chain is serial: the hip carries hip+thigh+shin,
# the pitch joint carries thigh+shin, the knee carries only the shin.
_LINK_WEIGHTS_N = {leg: np.array([li.mass * GRAVITY_M_S2 for li in links])
                   for leg, links in kin.LINK_INERTIALS.items()}
_CARRIES = np.triu(np.ones((3, 3)))

# Import-time sanity.  This is the cheap half of the check that would have
# caught both the DOG5_MASS_KG = 0 incident (CONTROL_ROADMAP.md Phase 4) and
# the 5.3 that is still live in stand_dog5_hw.py:106.  The expensive half --
# against the MJCF itself -- is verify_against_model(), run by the gates.
if not abs(_TOTAL_MASS_KG - DOG5_MASS_KG) < 1e-3:
    raise RuntimeError(
        f"torque_params.DOG5_MASS_KG = {DOG5_MASS_KG} disagrees with the link "
        f"inertials ({_TOTAL_MASS_KG:.6f} kg = trunk {TRUNK_MASS_KG} + 4 x "
        f"{_LEG_MASS_KG:.6f}).  Do NOT paper over this: the mass is the "
        f"gravity feedforward in body_wrench, and stand_dog5_hw.py:106 "
        f"already carries a wrong one (5.3).")


def total_mass() -> float:
    """Whole-robot mass (kg), derived from the link inertials, never a literal."""
    return _TOTAL_MASS_KG


def leg_mass() -> float:
    """One leg's mass (kg).  55% of the robot lives in the four of these."""
    return _LEG_MASS_KG


# ===========================================================================
# one chain walk
# ===========================================================================

def leg_frames(leg: str, q):
    """Walk one leg's chain ONCE and return everything downstream needs.

    Returns (foot, anchors(3,3), axes(3,3), com_pts(3,3)) in the trunk frame:
        foot      foot-site position (centre of the 20 mm contact sphere)
        anchors   the three joint origins  [hip, pitch, knee]
        axes      the three joint axes, already rotated into the trunk frame
        com_pts   the three link CoMs      [hip, thigh, shin]

    `dog5_kinematics` exposes foot_position, foot_jacobian and
    leg_gravity_torque as three separate entry points, each of which rebuilds
    this chain -- so the naive "just add the gravity term" costs a third walk.
    Measured on the Pi, leg_gravity_torque x4 alone is 860 us.  Fusing keeps
    the whole stance law inside the 100 Hz worker's budget.
    """
    q_abd, q_pitch, q_knee = kin._checked_q(q)
    geom = kin._geometry(leg)

    hip = np.asarray(geom.hip)
    r_hip = kin._rot_x(q_abd)
    pitch = hip + r_hip @ np.asarray(geom.hip_to_pitch)
    r_thigh = r_hip @ kin._rot_z(q_pitch)
    knee = pitch + r_thigh @ np.asarray(geom.pitch_to_knee)
    r_shin = r_thigh @ kin._rot_z(q_knee)
    foot = knee + r_shin @ np.asarray(geom.knee_to_foot)

    anchors = np.stack((hip, pitch, knee), axis=0)
    axes = np.stack((kin._X_AXIS, r_hip @ kin._Z_AXIS, r_thigh @ kin._Z_AXIS),
                    axis=0)
    # each link's hinge sits at its own body origin, so the CoM frames are
    # (hip, r_hip), (pitch, r_thigh), (knee, r_shin) -- same pairing as
    # dog5_kinematics.leg_gravity_torque
    inertials = kin.LINK_INERTIALS[leg]
    com_pts = np.stack((
        hip + r_hip @ np.asarray(inertials[0].com),
        pitch + r_thigh @ np.asarray(inertials[1].com),
        knee + r_shin @ np.asarray(inertials[2].com),
    ), axis=0)
    return foot, anchors, axes, com_pts


def foot_jacobian_from(foot, anchors, axes) -> np.ndarray:
    """d(foot)/dq from an already-walked chain.  Same value as
    dog5_kinematics.foot_jacobian, without the second walk."""
    return np.column_stack([np.cross(axes[i], foot - anchors[i])
                            for i in range(3)])


# ===========================================================================
# the omitted term
# ===========================================================================

def leg_gravity_torque_tilted(leg: str, q, g_down_body=None,
                              frames=None) -> np.ndarray:
    """Torque the three motors must hold against this leg's own link weights.

    `g_down_body` is the unit gravity direction (pointing DOWN) expressed in
    the TRUNK frame.  For a level trunk that is (0, 0, -1) and this function
    returns exactly `dog5_kinematics.leg_gravity_torque(leg, q)` -- gated to
    machine precision, so the two can never drift apart.

    To get it from an estimator: `C` maps INERTIAL -> BODY and the inertial
    up-axis is +z, so up_body = C @ [0,0,1] (this is precisely
    dog5_vmc_core.attitude_error_rp's `g_b`) and g_down_body = -up_body.

    Why it matters that this takes a direction at all: the stock function
    assumes the trunk is horizontal.  A leveling stand holds a few degrees of
    mount tilt permanently, and the abduction term -- the largest of the
    three -- is the one most sensitive to it, because abduction swings the
    whole leg mass about a trunk-x axis.
    """
    g = _G_DOWN_LEVEL if g_down_body is None else np.asarray(g_down_body,
                                                             dtype=float)
    _, anchors, axes, com_pts = (leg_frames(leg, q) if frames is None
                                 else frames)

    # Vectorised form of the obvious double loop
    #     for link, for j <= link:  tau[j] -= m[link]*g_mag * g . (a[j] x d)
    # d[j, link] = com_pts[link] - anchors[j], so the cross products for all
    # nine (joint, link) pairs go out in one call and _CARRIES masks off the
    # pairs a joint does not carry (a joint holds only the links at or below
    # it in the chain).  Same arithmetic, ~7x less interpreter.
    d = com_pts[None, :, :] - anchors[:, None, :]         # (3 joints, 3 links, 3)
    lever = np.cross(axes[:, None, :], d)                 # (3, 3, 3)
    # gravitational generalised force is F . (axis x d) with F = weight * g;
    # the motor must supply the negative of it
    return -((lever @ g) * _LINK_WEIGHTS_N[leg] * _CARRIES).sum(axis=1)


# ===========================================================================
# whole-body CoM
# ===========================================================================

def com_body(q_all, frames_all=None) -> np.ndarray:
    """Whole-body CoM in the trunk frame, for the grasp map's lever arms.

    `q_all` is (4, 3), LEGS x [abd, pitch, knee].

    Third in priority behind leg gravity (0.48 Nm) and total mass (0.09 Nm):
    this is worth about 0.02 Nm.  It moves a lot -- +24.6 mm at the recorded
    crouch to -19.3 mm at a 0.19 m stand, a 44 mm swing that changes sign --
    but almost all of that is in z, and z very nearly cancels: in
    dog5_vmc_core.grasp_map the lever enters as skew(r_w), and for a
    world-vertical force  r x (0,0,fz) = (ry*fz, -rx*fz, 0), so r_z drops out
    identically.  It reaches the answer only through TANGENTIAL forces (the
    kd_x/kd_y damping and the friction clamp).  In x/y the offset is 0.3 mm
    against a 340 mm foot half-spacing.

    Compute it anyway -- it is 13 vector adds and it removes an assumption --
    but do not expect it to move a stand.
    """
    q_all = np.asarray(q_all, dtype=float)
    if q_all.shape != (4, 3):
        raise ValueError(f"q_all must be (4, 3), got {q_all.shape}")
    moment = TRUNK_MASS_KG * TRUNK_COM_BODY.copy()
    for i, leg in enumerate(LEGS):
        com_pts = (leg_frames(leg, q_all[i])[3] if frames_all is None
                   else frames_all[i][3])
        for link, inertial in enumerate(kin.LINK_INERTIALS[leg]):
            moment += inertial.mass * com_pts[link]
    return moment / _TOTAL_MASS_KG


# ===========================================================================
# the corrected stance law
# ===========================================================================

def stance_torque(leg: str, q, f_body, g_down_body=None, frames=None):
    """Joint torque for ONE stance leg carrying `f_body`.

    `f_body` is the ground reaction ON THE BODY (up, +z), in body coordinates
    -- exactly what `dog5_vmc_core.distribute_wrench` returns.  The foot
    pushes DOWN on the ground with -f_body, hence the sign on the J^T term;
    this matches dog5_vmc_core.py:227 and is not the part that was wrong.

    Returns (tau(3,), J(3,3)) -- J comes back because the caller needs it for
    the singularity guard and for foot_load_map on the way out.
    """
    if frames is None:
        frames = leg_frames(leg, q)
    foot, anchors, axes, _ = frames
    J = foot_jacobian_from(foot, anchors, axes)
    tau = -J.T @ np.asarray(f_body, dtype=float)
    tau += leg_gravity_torque_tilted(leg, q, g_down_body, frames=frames)
    return tau, J


def gravity_down_body(C) -> np.ndarray:
    """Unit DOWN direction in the body frame, from an I->B rotation matrix."""
    return -(np.asarray(C, dtype=float) @ np.array([0.0, 0.0, 1.0]))


# ===========================================================================
# the MuJoCo cross-check -- test-only, mujoco imported lazily
# ===========================================================================

def verify_against_model(q_all, contacts=None, f_world=None):
    """Full floating-base inverse dynamics from the MJCF, for the gates.

    Returns (tau_model(12,), tau_ours(12,)) so a caller can assert they agree.
    The robot is placed level and static (qacc = qvel = 0) with the requested
    per-foot world forces applied, so MuJoCo's `qfrc_bias` is pure gravity and

        tau_model = qfrc_bias[actuated] - Jc^T f

    is the exact torque each joint must hold.  This is the ground truth that
    showed `-J^T f` alone is 0.482 Nm out and `-J^T f + leg gravity` is exact.

    Imported lazily: no hardware runner should ever pull MuJoCo into a
    control process.
    """
    import mujoco                                          # noqa: PLC0415

    xml = os.path.join(_DESC, "dog5.xml")
    model = mujoco.MjModel.from_xml_path(xml)
    data = mujoco.MjData(model)

    q_all = np.asarray(q_all, dtype=float).reshape(4, 3)
    if contacts is None:
        contacts = np.ones(4, dtype=bool)
    contacts = np.asarray(contacts, dtype=bool)
    if f_world is None:
        n = max(int(contacts.sum()), 1)
        share = _TOTAL_MASS_KG * GRAVITY_M_S2 / n
        f_world = np.tile(np.array([0.0, 0.0, share]), (4, 1))
    f_world = np.asarray(f_world, dtype=float).reshape(4, 3)

    # free joint: identity pose, then the 12 hinges in dog5.xml body order,
    # which is LEGS x [abd, pitch, knee] -- the same order used everywhere.
    data.qpos[:3] = 0.0
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    data.qpos[7:] = q_all.reshape(-1)
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    mujoco.mj_forward(model, data)

    # gravity/bias term at qvel = 0 is exactly the gravity generalised force
    mujoco.mj_rnePostConstraint(model, data)
    bias = data.qfrc_bias.copy()

    tau_model = np.zeros(12)
    for i, leg in enumerate(LEGS):
        sl = slice(7 + 3 * i, 7 + 3 * i + 3)          # qpos slice
        vsl = slice(6 + 3 * i, 6 + 3 * i + 3)         # qvel/force slice
        tau_model[3 * i:3 * i + 3] = bias[vsl]
        if contacts[i]:
            jacp = np.zeros((3, model.nv))
            site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,
                                     f"foot_{leg}")
            mujoco.mj_jacSite(model, data, jacp, None, site)
            tau_model[3 * i:3 * i + 3] -= (jacp[:, vsl].T @ f_world[i])
        del sl

    tau_ours = np.zeros(12)
    for i, leg in enumerate(LEGS):
        if contacts[i]:
            tau_i, _ = stance_torque(leg, q_all[i], f_world[i])
        else:
            tau_i = leg_gravity_torque_tilted(leg, q_all[i])
        tau_ours[3 * i:3 * i + 3] = tau_i
    return tau_model, tau_ours


def model_total_mass() -> float:
    """`MjModel.body_mass.sum()` -- the authority the literals are gated on."""
    import mujoco                                          # noqa: PLC0415
    model = mujoco.MjModel.from_xml_path(os.path.join(_DESC, "dog5.xml"))
    return float(model.body_mass.sum())
