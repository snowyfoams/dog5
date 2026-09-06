"""Inverse kinematics: a foot position in the trunk frame -> three joint angles.

The forward direction (:mod:`dog5_sdk.kinematics`) is closed form and exact --
it is a controlled copy of the MJCF geometry, gated against MuJoCo to 1.7e-16 m.
The inverse is solved numerically, by damped least squares on the same
Jacobian, because that is the honest way to invert a chain whose foot offset
(``knee_to_foot`` carries a 1.1 mm y term) is not quite planar.

Two entry points:

    leg_ik(leg, p_target, q_seed)   one leg, one foot target
    body_ik(feet, q_seed)           all four, returns the (12,) joint vector

Both take and return the canonical joint order ``[abduction, pitch, knee]``,
radians, foot positions in metres in the TRUNK frame.

WHICH SOLUTION YOU GET
    A three-joint leg reaching a three-dimensional point has a discrete set of
    solutions -- knee forward or knee back -- and no numerical method chooses
    between them for you.  The seed does.  Seed from the pose you are near
    (the previous tick's ``q``, or :data:`dog5_sdk.poses.Q_CROUCH`) and you
    stay on that branch; seed from zero, which is the singular calibration
    pose, and you will get whatever the damping happens to walk into.  There
    is no default seed for that reason.
"""
from __future__ import annotations

import numpy as np

from . import kinematics as kin

LEGS = kin.LEGS

#: Levenberg-Marquardt damping (m^2).  Large enough to stay finite through the
#: rank-1 singularity at the all-zero calibration pose, small enough that a
#: well-conditioned pose still converges in a handful of iterations.
DEFAULT_DAMPING = 1.0e-4

#: Convergence: stop when the foot is within this of the target.
DEFAULT_TOL_M = 1.0e-6

DEFAULT_MAX_ITERS = 200

#: Per-iteration cap on |dq| (rad).  Stops a near-singular step from throwing
#: the seed across the workspace, which is how an IK lands on the wrong branch.
_MAX_STEP_RAD = 0.2


class IKError(RuntimeError):
    """The solver did not reach `tol` within `max_iters` iterations."""


def leg_ik(leg: str, p_target, q_seed, *, damping: float = DEFAULT_DAMPING,
           tol: float = DEFAULT_TOL_M, max_iters: int = DEFAULT_MAX_ITERS,
           strict: bool = True) -> np.ndarray:
    """Joint angles putting `leg`'s foot at `p_target` (trunk frame, metres).

    `q_seed` (3,) picks the branch and is the starting point -- see the module
    docstring.  Raises :class:`IKError` if the solver does not converge, unless
    ``strict=False``, in which case the best iterate is returned; check it with
    :func:`dog5_sdk.kinematics.foot_position` before you command it.
    """
    p_target = np.asarray(p_target, dtype=float)
    if p_target.shape != (3,):
        raise ValueError(f"p_target must have shape (3,), got {p_target.shape}")
    q = np.array(q_seed, dtype=float).reshape(3).copy()

    eye = np.eye(3)
    lam = float(damping)
    residual = float(np.linalg.norm(p_target - kin.foot_position(leg, q)))

    # Levenberg-Marquardt: the damping ADAPTS.  A FIXED lambda is what stops a
    # plain damped-least-squares IK from ever reaching machine precision --
    # close to the solution the damping is the only thing left in the step, and
    # it puts a floor under the residual (2 um here, with lambda = 1e-4).
    # Halving lambda after a successful step removes that floor; quadrupling it
    # after a failed one is what keeps the singular poses -- the all-zero
    # calibration pose is rank 1 -- from diverging.
    for _ in range(max_iters):
        if residual <= tol:
            return q
        error = p_target - kin.foot_position(leg, q)
        jac = kin.foot_jacobian(leg, q)
        step = jac.T @ np.linalg.solve(jac @ jac.T + lam * eye, error)
        norm = np.linalg.norm(step)
        if norm > _MAX_STEP_RAD:
            step *= _MAX_STEP_RAD / norm
        candidate = q + step
        trial = float(np.linalg.norm(
            p_target - kin.foot_position(leg, candidate)))
        if trial < residual:
            q, residual = candidate, trial
            lam = max(0.5 * lam, 1.0e-12)
        else:
            lam *= 4.0
            if lam > 1.0e6:              # no step from here improves anything
                break

    if residual <= tol:
        return q
    if strict:
        raise IKError(
            f"{leg}: IK did not converge in {max_iters} iterations; foot is "
            f"{residual * 1e3:.3f} mm from the target.  Either the target is "
            f"outside the workspace, or the seed {np.round(q_seed, 3)} is on "
            f"the wrong branch."
        )
    return q


def body_ik(feet, q_seed, **kwargs) -> np.ndarray:
    """Solve all four legs.  `feet` is a {leg: (3,)} dict or a (4, 3) array.

    `q_seed` is the (12,) joint vector to start from.  Returns (12,) in the
    canonical order LEGS x [abd, pitch, knee].
    """
    q_seed = np.asarray(q_seed, dtype=float).reshape(12)
    out = np.empty(12)
    for index, leg in enumerate(LEGS):
        target = feet[leg] if isinstance(feet, dict) else np.asarray(feet)[index]
        section = slice(3 * index, 3 * index + 3)
        out[section] = leg_ik(leg, target, q_seed[section], **kwargs)
    return out
