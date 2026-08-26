#!/usr/bin/env python3
"""Every constant the hardware SRB-MPC reads.  No logic, no sibling imports.

WHAT IS *NOT* HERE, AND WHY THAT IS THE POINT
    Mass, inertia, the hip and stance geometry, the gait period and the joint
    caps are NOT re-declared in this file.  They are imported from
    dog5_trot_quasi_static_model.config and torque_stand.params, which are the controlled copies
    of dog5.xml and of the measured bus facts respectively.

    params.py exists because eight gains were once declared TWICE as
    independent literals and editing one silently left the other behind.  A
    second copy of MASS in an MPC file is that bug waiting again -- and worse
    here than anywhere, because the MPC's B matrix is 1/m and its moment rows
    are I^-1: a mass that drifts 5% from the one force_totorque uses makes the
    plan and the execution disagree about the same robot.

    So this file declares only what is genuinely NEW: the horizon, the cost,
    the solver budget, and the constants the swing and contact layers need.

THE HORIZON IS ONE GAIT CYCLE, AND IT IS DERIVED
    N_HORIZON * MPC_DT == dog5_trot_quasi_static_model.config.GAIT_PERIOD, exactly, because
    MPC_DT is computed from the other two.  A horizon shorter than a cycle
    cannot see the next touchdown, which is the whole reason a trot wants an
    MPC instead of the instantaneous grasp map; a horizon longer than a cycle
    spends QP on a schedule the next replan will have moved anyway.

    The simulation used N=10, dt=0.03 against a 0.42 s period -- 71% of a
    cycle -- and got away with it because a simulated foot never lands early.

THE COST WEIGHTS ARE SIZED FROM THE GAINS THIS ROBOT HAS STOOD ON
    This is the one place where copying the simulation would have been wrong.
    A cost weight is not a gain, but the closed loop has an EFFECTIVE gain and
    it can be read straight out of the solver: put a pure roll error into x0,
    solve, and sum r_i x f_i.  The runner prints those four numbers in its
    banner at every start, so a weight change is never committed without the
    gain it implies.

    The targets are torque_stand's verified pairs -- kp_z 300 N/m, kd_z 40,
    kp_att 10 Nm/rad, kd_att 0.5 -- and NOT the simulator's, because:

        kp_att   this stance is +/-112 mm wide, so its roll capacity is
                 WEIGHT * 0.112 = 6.4 Nm and a 120 Nm/rad gain saturates it
                 at 3.1 deg.  The simulator's attitude weights sit in that
                 regime.  Hardware walked the gain DOWN to 10 across the
                 2026-08-18 ladder, and params.py names the log for each step.
        kp_z     800/120 has 7.4 deg of phase margin against the measured
                 20 ms outer delay.  300/40 has 27.

    The MEASURED effective gains at the weights below are in the table beside
    them.  Re-read them from the banner after any edit.

WHAT THIS TRACK RUNS THAT WEEK 2 NEVER DID
    W_VEL[0:2] is a live gain on horizontal velocity.  params.KD_X and KD_Y
    are 0.0 and their comment says "never run > 0" -- the leg-odometry
    velocity carries a 5 Hz filter and a 12 ms hold, and nobody had tested
    damping on it.  A trot needs it: without a velocity term the MPC has no
    reason to produce the tangential force that answers a push, and the
    Raibert placement is left to do all of it one step later.

    It is small, it is friction-limited by the QP's own pyramid, and
    --w-vel 0 is the ablation half of that A/B.  Run the ablation.
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

# THE TWO OWNERS OF EVERY NUMBER THIS FILE DOES NOT DECLARE.
#   tcfg  geometry, mass, inertia, gait, swing, joint caps -- from dog5.xml
#   P     bus timing, torque caps, friction, the estimator's constants
from dog5_trot_quasi_static_model import config as tcfg  # noqa: E402
import params as P                                         # noqa: E402

LEGS = tcfg.LEGS                     # ("FL", "FR", "RL", "RR") -- HARDWARE order
N_LEGS = tcfg.N_LEGS
N_JOINTS = tcfg.N_JOINTS
JOINT_INDEX = tcfg.JOINT_INDEX
PHASE_OFFSET = tcfg.PHASE_OFFSET

# THE LEG ORDER IS NOT THE SIMULATOR'S, AND NOTHING CONVERTS BETWEEN THEM.
# srb_mpc/robot.py is [FR, FL, RR, RL]; this tree is ("FL", "FR", "RL", "RR")
# everywhere -- dog5_kinematics, force_totorque, the CAN motor map, and every
# log ever recorded on this robot.  So the port ADOPTS the hardware order
# outright rather than carrying a permutation that one file would eventually
# forget to apply.  The trot pairing survives it: the diagonals are (FL, RR)
# and (FR, RL) here, exactly as (FR, RL) and (FL, RR) there, and
# tcfg.PHASE_OFFSET is already written in this order.

# ===========================================================================
# mass, inertia and geometry -- re-exported, never re-declared
# ===========================================================================
MASS = tcfg.MASS                     # 5.8151 kg
WEIGHT = tcfg.WEIGHT                 # 57.05 N
GRAVITY = tcfg.GRAVITY
# Whole-robot tensor about the whole-robot CoM at the nominal stance, in the
# TRUNK frame.  This is precisely the SRB-MPC's I_body: the simulator computes
# the same composite tensor from the MJCF at its standing keyframe.
INERTIA_BODY = tcfg.INERTIA_BODY
COM_BODY = tcfg.COM_BODY             # nominal; the loop uses the live one
FOOT_STANCE_BODY = tcfg.FOOT_STANCE_BODY
FOOT_RADIUS = tcfg.FOOT_RADIUS
LEG_REACH = tcfg.LEG_REACH

# ===========================================================================
# the horizon
# ===========================================================================
N_HORIZON = 8                        # knots.  12 forces each -> 96 QP variables
GAIT_PERIOD = tcfg.GAIT_PERIOD       # 1.2 s -- the trot track's VERIFIED
                                     # cycle (was 0.40 when this file was
                                     # first tuned; it follows tcfg, so the
                                     # cost table below was re-measured at
                                     # 1.2, see the table's date)
DUTY = tcfg.DUTY                     # 0.80, ditto -- 30% double support,
                                     # which the contact_weight ramp needs
MPC_DT = GAIT_PERIOD / N_HORIZON     # 0.150 s -- DERIVED, see the header
MPC_HZ = 40.0                        # solves per second, off-thread.  Six per
                                     # knot at the 1.2 s cycle, so the plan
                                     # the robot is holding is never more
                                     # than a sixth of a knot old.
# CONTROL_ROADMAP Phase 5 budgets "12 vars x 10-step at 25-50 Hz on the Pi 5"
# and calls solve time the phase's main risk.  It is measured IN THE RUN --
# the worker publishes min/mean/max and the exit report prints them -- rather
# than assumed here.  --mpc-hz is the flag to lower if that report says so.
#
# WHY 8 KNOTS AND NOT THE SIMULATOR'S 10, WHICH IS A NUMPY FACT AND NOT A
# CONTROL ONE.  The decision variable is 12*N, and BLAS switches the 120x120
# inverse onto its threaded path somewhere just under 100x100.  Measured on
# the development box, same code, same solve:
#
#     N = 8   (96 vars)   inverse 0.08 ms mean, 0.14 max
#     N = 10 (120 vars)   inverse 2.21 ms mean, 27.3 max
#
# A 27 ms outlier is longer than a whole replan period, and it arrives as a
# missed publish rather than a slow one.  8 knots keeps every array under the
# cliff, and the horizon is STILL exactly one gait cycle because MPC_DT is
# derived -- 0.05 s knots instead of 0.04.  Run the worker with
# OPENBLAS_NUM_THREADS=1 anyway (the runner's RUN block does); on a 4-core Pi
# sharing threads with a 250 Hz CAN loop is a jitter source, not a speed-up.

# WHAT THE SCHEDULE IS EVALUATED AT, WHICH IS NOT `now` AND IS NOT `now` PLUS
# THE LATENCY EITHER.
# ---------------------------------------------------------------------------
# Knot 0's force is not an instant, it is a BOX: the QP holds it constant
# across [t_sched, t_sched + MPC_DT], so its centre of mass in time is
# t_sched + MPC_DT/2.  What the robot actually feels is another box: the CAN
# loop applies the published force from the publish until the next one and the
# joint feels it params.LOOP_DELAY_S later, so that box is centred at
# t_solve + LOOP_DELAY + (1/MPC_HZ)/2.  Line the two centres up:
#
#     lead = LOOP_DELAY + 1/(2 MPC_HZ) - MPC_DT/2
#          = 0.016 + 0.0125 - 0.025 = 3.5 ms
#
# THE MIDDLE TERM ALONE IS THE MISTAKE THIS FILE MADE FIRST, and it is worth
# writing down because it looked right: leading by "half a replan period plus
# the loop delay" (28.5 ms) forgets that the knot the plan is being compared
# against is 50 ms wide.  The plan then runs more than half a knot ahead of
# the contact set the applied force is CLAMPED against, and at every diagonal
# handover it hands the load to a foot the clamp still calls airborne.
# Measured, four gait cycles driven at the model rate:
#
#     lead 28.5 ms   applied support fell below 90% of the weight on 11 of
#                    133 ticks, worst 24 N on a 57 N robot
#     lead  3.5 ms   see the same run in the commit message
#
# The STATE is deliberately not propagated forward by any of this.  Propagating
# needs the model to be right about the next 20 ms, and if it were that right
# the delay would not matter; the schedule shift needs only the CLOCK, which
# is exact.
#
# AT THE VERIFIED 1.2 s CYCLE THE max() CLAMPS THIS TO ZERO: the knot is
# 150 ms wide, so its half-width (75 ms) swallows the 16 + 12.5 ms of delay
# with room to spare -- the applied box already sits inside the first knot's
# box, and leading the schedule would only push the plan AHEAD of the
# contact set, which is the exact failure the derivation above records.
MPC_SCHEDULE_LEAD_S = max(P.LOOP_DELAY_S + 0.5 / MPC_HZ - 0.5 * MPC_DT, 0.0)

# ===========================================================================
# the cost
# ===========================================================================
# State order is the simulator's: [rpy(3), p(3), omega_world(3), v(3), g].
# The trailing gravity state is a constant and carries weight 0.
#
# RETUNED 2026-08-25, AND IT HAD TO BE, TWICE OVER.  First, the targets
# moved: the verified table is the TROT TRACK's now (tcfg.KP_ORI = [3,3,3],
# KD_ORI 0.5, kp_z 300, kd_z 40 -- the gains trot_hw and trot_demo have
# actually flown on this rig), not week 2's kp_att = 10.  Second, the gait
# moved under the weights: GAIT_PERIOD follows tcfg and went 0.40 -> 1.2 s,
# which stretched every knot 3x and silently re-priced every weight -- at
# the OLD weights the new horizon measured kp_roll 2.07, kp_z 119, i.e. the
# committed table had drifted 2.5x without one line of this file changing.
# That is why the table below carries a date and the banner re-measures it
# every start.
#
# THE EFFECTIVE GAINS AT THESE WEIGHTS, MEASURED 2026-08-25 at the 1.2 s /
# 0.80 gait, four feet down, nominal stance -- put a unit error in one
# state, solve, read the wrench back out (mpc_controller.effective_gains):
#
#     axis      this file    trot track verified   where the trot records it
#     kp_roll      3.06         3                  tcfg.KP_ORI[0]
#     kp_pitch     3.19         3                  tcfg.KP_ORI[1]
#     kp_z       301.2        300                  tcfg.KP_POS[2] (s2_kpz300)
#     kd_roll      0.67         0.5                tcfg.KD_ORI[0]
#     kd_pitch     2.93         0.5                tcfg.KD_ORI[1]
#     kd_z        60.4         40                  tcfg.KD_POS[2] (s1_kdz40)
#     kd_x         0            0     (see W_VEL and W_POS: the horizontal
#     kp_x         0            0      channels were flown and lost, twice)
#
# and the support at zero error is 57.046 N against a 57.05 N robot, which is
# the gravity-referenced regulariser doing its job (see convex_mpc).
#
# THE STIFFNESSES MATCH AND THE DAMPINGS SIT ABOVE, AND THAT IS THE HORIZON.
# ---------------------------------------------------------------------------
# A rate error is a position error one horizon later, so an MPC prices the
# two together and lands MORE damped than the PD ladder did.  In fractions
# of critical damping, against the trot track's verified pairs:
#
#     roll    critical is 2 sqrt(kp Ixx) = 2 sqrt(3.06 x 0.0658) = 0.90.
#             This runs 0.67 = 0.75x critical; the verified 0.5 is 0.56x.
#     pitch   critical is 2 sqrt(3.19 x 0.4075) = 2.28.  This runs 2.93 =
#             1.29x critical -- Iyy is 6x Ixx, so the same W_OMEGA prices
#             pitch rate high.  Over-damped, not twitchy; left alone.
#     height  critical is 2 sqrt(301 x 5.815) = 83.7 Ns/m.  This runs 60.4
#             = 0.72x critical; the verified 40 is 0.48x.
#
# THE ONE TO WATCH IS kd_z, because params.py names it as the axis carrying
# the whole lag budget (a 5 Hz filter on leg odometry plus a 12 ms hold):
#     phase    the loop crosses over near kd/m = 10.4 rad/s = 1.65 Hz, where
#              the measured 20 ms outer delay is 12 deg -- margin set by the
#              second-order rolloff, not the delay
#     noise    encoder-differenced qd is 0.026 rad/s filtered, which through
#              this kd_z is 0.3 N on a 57 N robot.  The 8.1 rad/s DRIVER
#              speed field that caused the 2026-08-17 shake is not read by
#              anything in this stack, exactly as in week 2.
# If a height chatter appears, --w-z is the flag: kp_z moves SLOWER than the
# square root of it at this horizon (240 -> 119, 2400 -> 261 at W_VEL_z 12)
# while kd_z barely moves, so it is a stiffness knob and not a damping one.
# The damping knob is the HORIZON, and shortening that shortens what the
# plan can see.
#
# HOW THE WEIGHTS HIT THE TARGETS, for the next retune (measured, this box):
#     kp_z     saturates in W_POS[2] alone (240->119, 2400->261): W_VEL[2]
#              prices the approach VELOCITY and caps the stiffness, so the
#              pair moves together -- 2400 with W_VEL[2] 6.0 lands 301.
#     kp_roll  same shape against W_OMEGA[0]: 320 with the 1.5 lands 3.06.
#     kp_pitch W_ATT[1] alone: 3 lands 3.19.
W_ATT = np.array([320.0, 3.0, 20.0])       # roll, pitch, yaw
# W_POS x/y ARE ZERO TOO, AND THE PAIR OF ZEROS IS ONE DECISION: nothing in
# the cost may act on the horizontal.  Zeroing W_VEL alone LOOKED like that
# ablation and was not -- a velocity error integrates into position error
# over the horizon, so through W_POS[0:2] = 1 the solver still carried an
# effective kd_x of 19 and kp_x of 25 (measured offline; the flown result
# is run_20260825_142034: during the FR/RL diagonal window the plan's
# tangential force thrashed at +-13..17 N flipping sign every ~50 ms, roll
# wobbled at ~7 Hz, joint speeds hit 9..15 rad/s and the drivers dropped
# out at the spike, same as run_20260825_141317).  With both zeroed the
# measured kd_x and kp_x are exactly 0.00 -- the same "nothing acts on
# x/y" the verified trot runners fly, where drift is answered by FOOT
# PLACEMENT, not by force.  The Raibert placement in mpc_swing is the
# horizontal loop now, exactly as in trot_hw.
W_POS = np.array([0.0, 0.0, 2400.0])       # x, y, z
W_OMEGA = np.array([1.5, 1.5, 3.0])        # wx, wy, wz (world axes)
# THE HORIZONTAL VELOCITY WEIGHTS ARE ZERO, AND THAT IS A HARDWARE RESULT,
# not a retreat to week 2's habit.  Flown at 0.25 on 2026-08-25
# (run_20260825_135546): at the verified 1.2 s gait the leg-odometry slosh
# sits at 0.83 Hz, which the 5 Hz odometry filter passes at 99%, and the
# damper turned it into a net tangential force with a p95 of 24.5 N -- a
# gait-synchronous ~1.3 Nm roll moment against a roll spring whose whole
# authority at 5 deg is 0.27 Nm.  Zeroed (run_20260825_141317, via
# --w-vel 0) the same trot ran clean diagonal handovers at ~1 N tangential
# until the scheduler -- a separate fault, see mpc_worker -- dropped the
# drivers.  The old 0.25 was tuned at the 0.40 s gait, where the slosh sat
# at 2.5 Hz and half of it never reached the damper.  Bring it back only
# with a slosh filter in front of the MPC's velocity state (a cycle EMA,
# like the placement's), and one run at a time.
W_VEL = np.array([0.0, 0.0, 6.0])          # vx, vy, vz

# X AND Y ARE OUT OF THE COST ENTIRELY (2026-08-25) -- see the W_POS and
# W_VEL provenance notes above.  The horizon-relative drift penalty this
# paragraph used to describe was tried on hardware and lost twice: whatever
# observes x/y here is the integral of a lagged, gait-frequency-sloshing
# leg-odometry velocity, and a force answered to it lands as tangential
# thrash at the feet.  Horizontal stays foot placement's job (mpc_swing's
# Raibert terms), exactly as on the verified trot runners.  --w-vel scales
# a pair of zeros now; the flag remains only so a future re-test with a
# properly filtered velocity state is one edit away.

W_FORCE = 2.0e-4             # regulariser on every force in the horizon; the
                             # simulator's value, doing the same job here --
                             # 12 forces against 6 wrench rows is a 6-wide
                             # null space at every knot.
W_SMOOTH = 2.0e-4            # on (f_0 - f_applied) ONLY.  New here, and it is
                             # a HARDWARE term: the horizon smooths the plan,
                             # but nothing smooths the STEP between one solve's
                             # first knot and the next's, and at 40 Hz that
                             # step lands on a slew-limited actuator.  Same
                             # role as balance_qp.W_SMOOTH, one knot wide.
#
# IT IS THE SAME SIZE AS W_FORCE AND IT MUST NOT BE MUCH LARGER, WHICH IS A
# MEASUREMENT AND NOT A PREFERENCE.  The term pulls the new plan towards the
# force currently applied -- and that force was itself pulled towards the one
# before it, with nothing anchoring the chain.  Above about W_FORCE the drag
# beats the gravity anchor and the whole sequence walks downhill.  Driven with
# 300 solves of realistic per-solve state noise on a live trot schedule:
#
#     W_SMOOTH   total support: min / mean / max (N), on a 57.05 N robot
#     0          45.8 / 57.0 / 68.7
#     2e-4       47.0 / 57.0 / 67.7        <- ships
#     1e-3       40.3 / 55.6 / 64.8
#     5e-3       21.1 / 45.4 / 62.3        <- 12 N light, on average, for ever
#
# The 5e-3 row is what this file shipped before the sweep was run, and it is
# exactly the failure mode the gravity-referenced regulariser was added to
# remove -- reintroduced by a different term.  A standing robot that is 12 N
# light does not fall over; it sags, and every gain above it reads wrong.

# ===========================================================================
# the friction cone and the force box
# ===========================================================================
MU = P.MU_FRICTION           # 0.6, the same clamp the stand uses
# THE PYRAMID IS INSCRIBED IN THE CIRCLE.  |fx| <= mu fz and |fy| <= mu fz
# permits a RESULTANT of sqrt(2) mu fz on the diagonals -- 41% past the
# friction the floor has, i.e. the QP would call a slipping plan feasible.
# Dividing by sqrt(2) puts the pyramid inside the circle.  Identical argument,
# and identical constant, to dog5_trot_quasi_static_model/balance_qp.py.
MU_AXIS = MU / np.sqrt(2.0)
FZ_MIN = P.FZ_MIN_N          # 1.0 N: a planted foot never unloads to nothing
FZ_MAX = tcfg.FZ_MAX         # 1.5 * WEIGHT: one diagonal pair carries it all,
                             # with room to answer a push

# ===========================================================================
# the solver -- dense ADMM, because the venv has numpy and nothing else
# ===========================================================================
# dog5_trot_quasi_static_model/balance_qp.py records the fact this depends on: no scipy, no osqp,
# no quadprog, no cvxpy, no qpsolvers.  The iteration is OSQP's, written out.
#
# THE COST IS SPENT WHERE THE STRUCTURE IS.  The constraint matrix is
# blockdiag(D) with the SAME 5x3 D at every foot of every knot, so C x, C^T y
# and C^T C are a reshape and a 3x3 -- and the whole per-iteration cost is one
# 120x120 matrix-vector product.  60 of those is 0.9 MFLOP, against the 4
# MFLOP of building the condensed Hessian once.
QP_ITERS = 60                # the BUDGET -- and since the 2026-08-25 retune
                             # the trot solve SPENDS ALL OF IT: the stiffer
                             # weights put the 1 mN early exit out of reach
                             # (59.6 iterations mean, measured over a warm-
                             # started gait cycle; it was 5-10 at the old
                             # weights).  Deliberately left so: loosening
                             # QP_TOL_N to 5 mN cuts the count to 31 but
                             # truncates the height response -- effective
                             # kp_z drops 301 -> 270, i.e. the tolerance had
                             # become a gain.  The full budget costs 2.85 ms
                             # against a 25 ms replan period.
# RHO IS 0.05 AND NOT balance_qp's 1.0, AND THE DIFFERENCE IS NOT COSMETIC.
# ADMM's rho balances the objective against the constraints, so it has to be
# scaled to the problem -- and this one is two orders of magnitude stiffer
# than the 12-variable force split, because the condensed Hessian carries the
# horizon's state weights (hundreds) where balance_qp carries only W_TASK
# (tens).  Measured, at the weights above, standing:
#
#     rho 1.00   60 iterations, still 0.55 N of support missing
#     rho 0.30   60 iterations, 0.21 N missing
#     rho 0.05    5 iterations, 0.03 N missing
#
# A mis-scaled rho does not announce itself: the solve returns, the forces
# look plausible, and the robot stands 1% light for ever.  If the state
# weights are ever moved by an order of magnitude, re-read those three rows
# before trusting the new gains.
# RE-READ 2026-08-25 after the retune moved W_POS[2] 240 -> 2400: support at
# zero error is 57.046 N at rho 0.05, 0.02 and 0.01 alike, and the measured
# effective gains move under 2% across that range -- 0.05 stands.
QP_RHO = 0.05
QP_SIGMA = 1.0e-6
# THE CONVERGENCE TOLERANCE IS IN NEWTONS, because the constraint rows are.
# 1 mN is three orders below the 0.02 N the torque sensing can even see, and
# the first knot is clamped feasible on the way out regardless -- so this is
# an early exit, not a correctness gate.
QP_TOL_N = 1.0e-3

# ===========================================================================
# swing -- Raibert placement and the arc
# ===========================================================================
# THE SWING FRAME IS THE BODY'S, AS OF 2026-08-25, AND THAT IS A HARDWARE
# RESULT.  mpc_swing originally planned its arcs in the gravity-aligned
# trunk-anchored frame -- correct in a simulator whose base barely tilts.
# On this robot a trot's diagonal-support roll is fast (the unstable
# timescale about the support line is ~80 ms, measured as -1 deg -> -10 deg
# in 0.19 s on run_20260825_142811), and in a gravity-aligned frame the
# swing TARGETS stay put while the body rotates under them: measured
# offline, 10 deg of roll moves a mid-swing target 6-10 mm in the body at
# a standstill, and during the 0.19 s transient the displacement grows with
# the roll excursion toward the foot's full ~0.35 m lever -- the 140 N/m
# swing impedance then slings the leg (the operator's word for it), the
# landing foot arrives displaced exactly when it was needed under the hip,
# and the catch that saves trot_hw at the same roll never happens.
# trot_hw's verified swing is pure body frame ("no world frame, so no frame
# to get wrong") and self-rights from 10 deg rolls precisely because the
# arc rides the trunk and the foot lands under the hip regardless of
# attitude.  True on this switch, the MPC runner does the same; the cost is
# a landing height off by (1 - cos(tilt)) -- 2 mm at 10 deg -- against the
# 60 mm of lateral slingshot removed.  False restores the port's original
# frame for A/B.
SWING_FRAME_BODY = True
SWING_HEIGHT = tcfg.SWING_HEIGHT     # 0.04 m apex over the stance plane
RAIBERT_KV = tcfg.RAIBERT_KV         # 0.03 s, the velocity-ERROR term
HIP_OFFSET = tcfg.HIP_OFFSET         # the reach clamp measures from here
# The Cartesian impedance that TRACKS the arc.  Scalars, not 3x3 diagonals: a
# trot in place has no reason to be stiffer in one direction than another, and
# a diagonal matrix that is secretly one number invites the reader to think
# otherwise.  Same values dog5_trot runs.
KP_SWING = float(tcfg.KP_SWING[0, 0])        # 140 N/m, follows tcfg
KD_SWING = float(tcfg.KD_SWING[0, 0])        # 8 Ns/m
# Joint damping added to a STANCE leg inside the force law, on top of the
# runner's 250 Hz impedance.  force_totorque.stance_torque carries the same
# term with the same value; it is a robustness term against the force law's
# velocity-level nature, not a controller.
KD_JOINT_STANCE = P.KD_JOINT                 # 0.15 Nms/rad
# A SECOND low pass on the placement velocity, on top of the estimator's 5 Hz.
# The simulation records why: the instantaneous CoM velocity oscillates at
# GAIT frequency, and feeding it raw into the touchdown target wobbles the
# footholds in resonance and sustains a rocking limit cycle (0.3 deg / 2 mm
# over 30 s once filtered).  Parametrised by a TIME CONSTANT, so it means the
# same thing at any control rate.
# ONE GAIT PERIOD, FOLLOWING tcfg (2026-08-25) -- the trot track's verified
# value, and its argument transfers whole: gait frequency is 1/1.2 = 0.83 Hz
# now, which BOTH the estimator's 5 Hz corner and the old 0.10 s tau
# (1.6 Hz) pass almost untouched.  A cycle of EMA keeps the drift and
# cancels the slosh; see tcfg.RAIBERT_V_TAU_S for the measurement.
RAIBERT_V_TAU = tcfg.RAIBERT_V_TAU_S  # s, = GAIT_PERIOD
# The reachable-set clamp, as a fraction of LEG_REACH.  A Raibert step at
# speed asks for a foot the leg cannot meet; clamping in the PLANNER makes
# that a shorter step, which the gait survives, instead of a saturated joint.
REACH_FRAC = 0.95
# What this stance height can actually travel at, derived in dog5_trot_quasi_static_model.config
# from the leg reach left over at the nominal stance: 0.043 m/s.  It is a
# TROT IN PLACE and the number says so; a velocity command past it is clamped
# by the runner rather than quietly asking for an unreachable foothold.
V_CMD_MAX = tcfg.MAX_FORWARD_V_AT_STAND_HEIGHT
WZ_CMD_MAX = 0.30                    # rad/s.  The abduction limit caps lateral
                                     # authority at ~2 cm (CONTROL_ROADMAP,
                                     # "Risks"), and yaw is produced entirely
                                     # by tangential friction at the feet.

# ===========================================================================
# the contact-aware layer
# ===========================================================================
# The clock schedule is UNCHANGED and stays the plan.  What this adds is the
# simulator's early-touchdown promotion, and it is not cosmetic there: a purely
# clock-driven trot pumps itself over within 12 s, because each early contact
# injects an unmodelled impulse at gait frequency while the MPC still treats
# that foot as force-free.
#
# ON HARDWARE THE MEASUREMENT IS THE MOTOR CURRENT, not a contact buffer.
# force_totorque.foot_load_from_torque already inverts measured iq through
# J^-T with each leg's own weight removed -- the same quantity that has to sum
# to 57 N in the exit report.  A foot carrying more than CONTACT_FZ_ON of it
# is standing on something.
# AND IT IS OFF UNTIL SOMEBODY MEASURES THE DETECTOR ON THIS ROBOT.
# The inversion that reads the foot load is the exact inverse of the map the
# swing controller commands through, so on a swinging leg it reads back that
# controller's own force: -12 N of downward swing command comes back as +9 N of
# "ground reaction" with the foot in the air, and the sign is positive exactly
# in the late-swing window promotion looks at.  mpc_gait subtracts the command,
# which removes the dominant term and leaves the current loop's ~24% tracking
# error -- better, and still not measured with legs in the air.
#
# A false promotion is worse than no promotion: the gait plants an airborne
# foot, the MPC allocates it force, and that share of the weight is pushed into
# nothing.  Off, the trot runs on the clock alone -- which is what
# dog5_trot_quasi_static_model/trot_hw already does on this robot.
# ContactAwareGait names the one measurement that earns --promote.
PROMOTE_ENABLED = False
PROMOTE_AFTER = 0.5          # only a LATE-swing contact promotes.  An early
                             # one is the foot still leaving the ground.
CONTACT_FZ_ON = 8.0          # N.  0.55 of an even four-way share (14.3 N),
                             # and 8x the FZ_MIN a planted foot may sit at, so
                             # a foot merely grazing does not latch.
CONTACT_FZ_OFF = 4.0         # N.  Hysteresis: one sample at the boundary must
                             # not chatter the contact set, because every
                             # change of it forces a replan.

# ===========================================================================
# rates, staleness and the handover
# ===========================================================================
CONTROL_HZ = P.CONTROL_HZ            # 250, the sweep -- impedance and gate
MODEL_EVERY = P.MODEL_EVERY          # 3, so estimator + torque map at 83 Hz
# The MPC is the ONE block that cannot be sub-sampled into a sweep at any
# ratio: a 120-variable condensed QP is milliseconds, the CAN slot is 333 us
# and the driver's input-lost watchdog is 10 ms.  CONTROL_ROADMAP says the
# same thing as a standing constraint -- "keep the EKF/MPC off-thread pattern
# and the CAN loop dumb" -- and torque_primitives/torque_worker.py is the
# pattern this follows, publishing discipline included.
MPC_STALE_S = 3.0 / MPC_HZ           # 0.075 s.  Three missed solves.
# WHAT A STALE MPC MEANS, stated once.  In position mode a stale estimate
# leaves the robot standing, because the drivers hold their last target.  In
# TORQUE mode a frozen plan keeps pushing on a world model that has stopped
# updating -- and this plan carries a CONTACT SCHEDULE, so a frozen one keeps
# pushing with feet that have since left the ground.  The runner limps.

# ===========================================================================
# caps and the height -- ALIGNED TO THE TROT TRACK'S VERIFIED TABLE (2026-08-25)
# ===========================================================================
# These four used to come from torque_stand/params.py.  They now come from
# tcfg, because the trot track is where every one of them was last verified
# ON THIS ROBOT and this runner must fly the same numbers:
#
#   TAU_MAX_DEFAULT   4.0, the cap trot_hw has flown since 2026-08-24 (params
#                     still says 1.0, a first-run value the trot outgrew)
#   TAU_HARD_NM       9.0, driver iq saturation -- the CLI ceiling for
#                     --tau-max, exactly as trot_hw relaxed it 2026-08-24
#                     (the old ceiling here was TAU_STAGED_MAX = 3.0)
#   STAND_HEIGHT      tcfg.STAND_TRUNK_BOTTOM_M -- the trot track's ONE
#                     height knob, floor to TRUNK BOTTOM, 0.140 as of
#                     2026-08-25.  params.STAND_HEIGHT is week 2's own copy
#                     and stays 0.152; reading it here is how this runner
#                     would silently stand 12 mm above the verified pose.
TAU_MAX_DEFAULT = tcfg.TAU_START_MAX     # 4.0 Nm, trot_hw's flown default
TAU_STAGED_MAX = tcfg.TAU_STAGED_MAX     # 3.0 Nm, the staged-ladder ceiling
TAU_HARD_NM = tcfg.TAU_HARD_NM           # 9.0 Nm, driver saturation
FORCE_FRAC_DEFAULT = tcfg.FORCE_FRAC_DEFAULT
STAND_HEIGHT = tcfg.STAND_TRUNK_BOTTOM_M  # floor to TRUNK BOTTOM, the knob
T_RISE = tcfg.T_RISE                 # 8.0 s, and the rise is still week 2's
