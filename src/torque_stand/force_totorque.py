#!/usr/bin/env python3
"""Wrench -> per-foot ground reaction -> joint torque.  The transmission half.

    W (6,)  --grasp map + damped LS-->  f_i (3,) per stance foot
    f_i     --  -J^T f + leg gravity -->  tau (12,)

dynamic_model decides HOW MUCH force the body needs.  This file decides HOW
that force is produced: which foot pushes how hard, and what each of the three
motors in that leg must do to make it happen.  No control gain acts on the
trunk here -- the only feedback term is a small joint damping, and it is a
robustness term, not a controller.

TWO THINGS THIS FILE GETS RIGHT THAT THE 2026-07-30 ATTEMPT DID NOT
    1. LEG GRAVITY.  `-J^T f` alone assumes MASSLESS legs.  Ours are 55% of
       the robot.  Two different balances are being conflated:

           EXTERNAL (the wrench, correct):  sum f = m*g about the whole-body
                                            CoM.  Sets HOW BIG f is.
           INTERNAL (omitted in 2026-07-30): with the trunk as base, each
                                            joint carries the foot GRF AND the
                                            weight of every link distal to it.
                                            Sets HOW f MAPS TO tau.

       Checked against MuJoCo floating-base inverse dynamics (qfrc_bias -
       Jc^T f at qacc = 0): -J^T f alone is off by 0.482 Nm; with the leg
       gravity term it matches to machine precision.  As a fraction of
       |J^T f|, and OPPOSITE in sign: abduction 63-65%, hip pitch 29-33%,
       knee 2%.  That is why RR_abd ran away.

    2. THE MASS.  P.MASS_KG is 5.8151 from dog5.xml.  stand_dog5_hw carries
       5.3, which is 8.9% low, and it is not used anywhere in this track.

WHY THE DISTRIBUTION IS A DAMPED LEAST SQUARES AND NOT A QP
    G f = W is 6 equations in 12 unknowns for a four-foot stance, so it is
    under-determined and the minimum-norm solution is the natural pick: it
    spreads the load rather than loading one leg.  A rank-deficient stance
    (a diagonal pair has no moment authority about its own support line) makes
    G G^T singular, and the lam*I regulariser is what makes that case return
    the achievable part instead of blowing up.  A proper friction-cone QP
    would be better for a gait; for a symmetric four-foot stand the clamp
    below essentially never triggers.

RUN
    A library: no bus, no IMU, no state.  `stance_torque` is a pure function
    of (q, qd, state, W, planted).
    cd <repo>/src
    python3 torque_stand/force_totorque.py --self-test        # no hardware
"""
from __future__ import annotations

import os, sys                                                  # noqa: E401
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dog5_paths  # noqa: E402,F401  -- every src/ dir onto sys.path

import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_AUG = os.path.dirname(_HERE)
_ROOT = dog5_paths.SRC

# Reused, not rewritten: leg_frames / foot_jacobian_from /
# leg_gravity_torque_tilted / com_body are the MuJoCo-verified half of the
# 2026-07-30 correction.  Rewriting them would mean re-deriving that check.
import dog5_statics as st                                  # noqa: E402
import params as P                                         # noqa: E402

LEGS = st.LEGS
N_LEGS = len(LEGS)
N_JOINTS = P.N_JOINTS


def _slice(i):
    return slice(3 * i, 3 * i + 3)


def _skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def grasp_map(foot_pos_body, stance, C, com_body):
    """G (6 x 3k): stacked WORLD-frame foot forces -> the body wrench.

    Lever arms are taken about the CoM and expressed in the world frame,
    r_i = C^T (s_i - com), so G [f_stack] = [sum f ; sum r x f].
    """
    G = np.zeros((6, 3 * len(stance)))
    for j, leg in enumerate(stance):
        i = LEGS.index(leg)
        r_w = C.T @ (foot_pos_body[i] - com_body)
        G[0:3, 3 * j:3 * j + 3] = np.eye(3)
        G[3:6, 3 * j:3 * j + 3] = _skew(r_w)
    return G


def distribute(W, foot_pos_body, stance, C, com_body=None,
               mu=P.MU_FRICTION, fz_min=P.FZ_MIN_N, lam=P.GRASP_LAMBDA,
               weights=None):
    """Split the wrench across the stance feet.  Returns {leg: force in BODY}.

    Minimum-norm solve f = G^T (G G^T + lam I)^-1 W, then a per-foot
    unilateral + friction clamp: a foot can only PUSH (fz >= fz_min, never
    pull the robot down) and can only produce tangential force inside mu*fz
    (beyond that it would need grip the floor does not have).

    `weights` (len(stance), in [0,1], optional) makes the split a WEIGHTED
    minimum norm: min f^T diag(w)^-1 f s.t. G f = W, i.e. a foot with weight
    0.1 volunteers a tenth of what a full foot does.  This is how a gait hands
    the load over -- a foot entering stance ramps its share up instead of
    taking half the robot in one model step, and a binary handover on THIS
    stance is a roll impulse: a diagonal pair has no moment authority about
    its own support line, so whatever the step imparts about that line is
    unopposed until the next pair lands.  None (every stand) is exactly the
    unweighted solve, bit for bit -- the self-test pins that.
    """
    if not stance:
        return {}
    com = np.zeros(3) if com_body is None else np.asarray(com_body, float)
    G = grasp_map(foot_pos_body, stance, C, com)
    if weights is None:
        S = G @ G.T + lam * np.eye(6)
        f_stack = G.T @ np.linalg.solve(S, W)
    else:
        w = np.repeat(np.clip(np.asarray(weights, float), 1e-3, 1.0), 3)
        GW = G * w                       # = G @ diag(w)
        S = GW @ G.T + lam * np.eye(6)
        f_stack = w * (G.T @ np.linalg.solve(S, W))

    clamped = []
    for j, leg in enumerate(stance):
        f_w = f_stack[3 * j:3 * j + 3].copy()
        if f_w[2] < fz_min:               # unilateral: feet push, never pull
            f_w[2] = fz_min
        t = f_w[:2]
        t_max = mu * f_w[2]
        t_norm = float(np.linalg.norm(t))
        if t_norm > t_max and t_norm > 1e-9:
            f_w[:2] = t * (t_max / t_norm)
        clamped.append([leg, f_w])

    # THE UNILATERAL CLAMP ONLY EVER RAISES fz, SO IT ADDS TOTAL FORCE.
    # Left alone that is not a saturation, it is a runaway: this stance is
    # only +/-112 mm wide laterally, so a roll moment saturates at
    # WEIGHT * 0.112 = 6.4 Nm -- which kp_roll = 120 Nm/rad reaches at just
    # 3.1 deg.  Past that the loaded side keeps being asked for more while the
    # unloaded side sticks at fz_min, and the TOTAL climbs: measured 122 N
    # commanded on a 57 N robot at 10 deg of roll.  Last week this was hidden
    # because --tau-max 1.0 truncated the result; the bug was still there.
    #
    # Total support is what holds the robot up and must be honoured.  The
    # moment is a trim, so it is the moment that saturates: rescale the whole
    # stack back to the commanded Fz.  Scaling f entirely (not just its z)
    # keeps every foot inside the friction cone it was just clamped into.
    fz_sum = sum(float(f[2]) for _, f in clamped)
    fz_cmd = float(W[2])
    saturated = fz_cmd > 0.0 and fz_sum > fz_cmd * (1.0 + 1e-9)
    if saturated:
        s = fz_cmd / fz_sum
        for pair in clamped:
            pair[1] = pair[1] * s

    return {leg: C @ f_w for leg, f_w in clamped}   # body frame for the J^T map


def moment_capacity(foot_pos_body, stance, fz_total=P.WEIGHT_N):
    """Largest (Mx, My) this stance can produce before a foot must pull (Nm).

    A foot can only push, so the achievable moment about an axis is bounded by
    the total vertical load times the lever arm to the support-polygon edge.
    Asking for more does not produce more -- it just unloads one side and
    saturates, which is why `distribute` has to rescale.

    This is the number kp_roll has to be sized against, and on this rig the
    two axes are wildly different: +/-112 mm laterally against +/-341 mm
    longitudinally, so roll saturates 3x sooner than pitch.
    """
    if not stance:
        return 0.0, 0.0
    ys = [abs(float(foot_pos_body[LEGS.index(l)][1])) for l in stance]
    xs = [abs(float(foot_pos_body[LEGS.index(l)][0])) for l in stance]
    return fz_total * float(np.mean(ys)), fz_total * float(np.mean(xs))


def stance_torque(q, qd, state, W, planted, gains, force_frac=P.FORCE_FRAC_DEFAULT,
                  leg_gravity=P.STANCE_LEG_GRAVITY, com_body=None,
                  contact_weight=None):
    """The full feedforward: wrench in, 12 joint torques out.

    `force_frac` scales the DISTRIBUTED GRF -- the only term holding the TRUNK
    up.  Leg gravity is deliberately NOT scaled by it: an unloaded leg still
    has to hold its own links up.

        1.0  full force, the joint impedance only as a floor.  Normal.
        0.0  no body support at all.  With the feet ON THE FLOOR this is NOT a
             benign plumbing test -- the trunk is then carried solely by the
             impedance's positional error, the legs fold until kp*dq balances
             57 N, and hardware 2026-08-14 latched seven joints inside 2 s.

    Returns (tau(12,), diag).
    """
    q = np.asarray(q, dtype=float)
    qd = np.asarray(qd, dtype=float)
    planted = np.asarray(planted, dtype=bool)
    stance = [LEGS[i] for i in range(N_LEGS) if planted[i]]

    frames = [st.leg_frames(LEGS[i], q[_slice(i)]) for i in range(N_LEGS)]
    foot_pos_body = [fr[0] for fr in frames]

    if com_body is None:
        com_body = (st.com_body(q.reshape(4, 3), frames_all=frames)
                    if P.CONFIG_DEPENDENT_COM else np.zeros(3))

    # contact_weight is (4,) over ALL legs; distribute wants it per stance
    # leg, in stance order.  None everywhere else keeps every stand unchanged.
    wts = None if contact_weight is None else \
        [float(contact_weight[i]) for i in range(N_LEGS) if planted[i]]
    forces = distribute(W, foot_pos_body, stance, state["C"], com_body,
                        weights=wts)
    g_down = st.gravity_down_body(state["C"])

    tau = np.zeros(N_JOINTS)
    singular = []
    for i in range(N_LEGS):
        leg = LEGS[i]
        foot, anchors, axes, _ = frames[i]
        J = st.foot_jacobian_from(foot, anchors, axes)
        if float(np.linalg.svd(J, compute_uv=False)[-1]) < P.MIN_JAC_SINGULAR:
            singular.append(leg)

        tau_i = np.zeros(3)
        if planted[i] and leg in forces:
            # forces[leg] is the reaction ON THE BODY (up); the foot pushes
            # DOWN on the ground with its negative, hence the sign.
            tau_i = -J.T @ (force_frac * forces[leg])
            tau_i -= gains.kd_joint * qd[_slice(i)]
        if leg_gravity:
            tau_i += st.leg_gravity_torque_tilted(leg, q[_slice(i)], g_down,
                                                  frames=frames[i])
        tau[_slice(i)] = tau_i

    # `forces` is BODY frame, because that is what J^T consumes.  The support
    # force a human reads -- and the one that must sum to the weight -- is the
    # WORLD vertical component, and on a tilted trunk those are not the same
    # number: body-z picks up part of the tangential force.  Report both, and
    # be explicit about which is which.
    fz_world = [float((state["C"].T @ forces[l])[2]) if l in forces else 0.0
                for l in LEGS]
    # `feet` IS RETURNED SO A CALLER CAN REBUILD THE GRASP MAP.  Purely
    # additive -- a new key cannot break a caller that does not read it -- and
    # it is what turns `forces` from a number you have into a WRENCH you can
    # compare against the one that was asked for.  Without it the achieved
    # wrench costs four leg_frames the caller has already paid for once.
    return tau, {"forces": forces, "stance": stance, "singular": singular,
                 "com_body": com_body, "fz": fz_world,
                 "feet": foot_pos_body}


def leg_gravity_only(q, C=None):
    """Each leg holds its own links up, and nothing else.

    The honest fallback when the estimator refuses: a controller that has lost
    the trunk should not keep pushing on a world model that stopped updating.
    This commands no body wrench at all, and the joint impedance in the sweep
    still holds the pose.
    """
    q = np.asarray(q, dtype=float)
    g_down = None if C is None else st.gravity_down_body(C)
    out = np.zeros(N_JOINTS)
    for i, leg in enumerate(LEGS):
        out[_slice(i)] = st.leg_gravity_torque_tilted(leg, q[_slice(i)], g_down)
    return out


def foot_load_from_torque(q, tau_meas, C=None):
    """Invert MEASURED joint torque back to a per-foot support force (N).

    WITH TORQUE CALIBRATION DROPPED THIS IS THE ONLY END-TO-END CHECK THAT THE
    FORCE LOOP IS REAL.  It takes measured iq, removes each leg's own link
    weight, and solves J^T f = -tau for the vertical component.  The four must
    sum to the robot's weight.  If they do not, the grasp map is fantasy no
    matter how good the attitude looks.

    Pass `C` to get the WORLD vertical component (what must sum to the
    weight); without it the body-frame z is returned, which differs by the
    trunk tilt -- 1.5% at 10 degrees, and in the direction that flatters the
    reading, so do not omit C when the robot is leaning.

    Returns (support(4,) in N, ok(4,) bool).  A singular leg gives NaN.
    """
    q = np.asarray(q, dtype=float)
    tau_meas = np.asarray(tau_meas, dtype=float)
    support = np.full(N_LEGS, np.nan)
    ok = np.zeros(N_LEGS, dtype=bool)
    for i, leg in enumerate(LEGS):
        frames = st.leg_frames(leg, q[_slice(i)])
        J = st.foot_jacobian_from(frames[0], frames[1], frames[2])
        if float(np.linalg.svd(J, compute_uv=False)[-1]) < P.MIN_JAC_SINGULAR:
            continue
        g_down = None if C is None else st.gravity_down_body(C)
        tau_grf = tau_meas[_slice(i)] - st.leg_gravity_torque_tilted(
            leg, q[_slice(i)], g_down, frames=frames)
        try:
            f_body = np.linalg.solve(J.T, -tau_grf)
        except np.linalg.LinAlgError:
            continue
        support[i] = float(f_body[2] if C is None else (C.T @ f_body)[2])
        ok[i] = True
    return support, ok


