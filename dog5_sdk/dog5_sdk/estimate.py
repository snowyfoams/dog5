"""The one place ``RobotState.z`` and ``RobotState.v`` are computed.

Both backends call :func:`trunk_estimate`, which is why a controller reading
``state.z`` in simulation is reading the same definition it will read on the
robot -- FK height to the TRUNK BOTTOM through the planted feet, and the
algebraic leg-odometry velocity.  Neither integrates anything, so neither
drifts; both are only as good as the contact set they are given.

Everything here is a thin arrangement of :mod:`dog5_sdk.estimator`, which is
the hardware-proven code.  Nothing is re-derived.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import estimator as est
from . import params as P
from . import statics as st
from .poses import LEGS, N_JOINTS


@dataclass(frozen=True)
class TrunkEstimate:
    """z, v and the honesty flag that says whether to believe them."""

    z: float                  #: floor to trunk bottom, m
    v: np.ndarray             #: world-frame trunk velocity, m/s
    n_planted: int            #: how many feet the estimate used
    valid: bool               #: False -> too few feet; z and v are not determined
    z_hip: float              #: the same height in the hip-axis (IK) frame
    rpy_fk: np.ndarray        #: roll, pitch from the foot plane; no IMU in it


def leg_frames_all(q) -> list:
    """Walk all four chains once and reuse the result everywhere below.

    One walk per leg per tick, rather than the three that calling
    ``foot_position`` / ``foot_jacobian`` / ``leg_gravity_torque`` separately
    would cost.  Measured on the robot's Pi, that difference is most of a
    millisecond in a 4 ms sweep.
    """
    q = np.asarray(q, dtype=float).reshape(N_JOINTS)
    return [st.leg_frames(leg, q[3 * i:3 * i + 3]) for i, leg in enumerate(LEGS)]


def trunk_estimate(q, qd, rpy, omega, contact, *,
                   min_planted: int = P.MIN_PLANTED,
                   frames=None) -> TrunkEstimate:
    """Height and world velocity of the trunk from encoders + attitude.

    `rpy` and `omega` come from the AHRS on hardware and from the MJCF's IMU
    site in simulation.  `contact` is the (4,) planted mask.

    With fewer than `min_planted` feet down the trunk velocity is NOT
    determined by the legs -- this returns ``valid=False`` and zeros rather
    than a plausible-looking number.  In torque mode a fiction here becomes
    real force, so check the flag; the shipped stand falls back to
    leg-gravity-only when it goes False.
    """
    q = np.asarray(q, dtype=float).reshape(N_JOINTS)
    qd = np.asarray(qd, dtype=float).reshape(N_JOINTS)
    contact = np.asarray(contact, dtype=bool).reshape(4)
    omega = np.asarray(omega, dtype=float).reshape(3)
    if frames is None:
        frames = leg_frames_all(q)

    C = est.C_from_rp(float(rpy[0]), float(rpy[1]))
    n_planted = int(contact.sum())

    if n_planted == 0:
        return TrunkEstimate(z=float("nan"), v=np.zeros(3), n_planted=0,
                             valid=False, z_hip=float("nan"),
                             rpy_fk=np.array([np.nan, np.nan]))

    z = est.fk_trunk_height(q, C, contact, frames=frames, ref="imu")
    z_hip = est.fk_trunk_height(q, C, contact, frames=frames, ref="hip")
    v, _ = est.leg_odometry_velocity(q, qd, C, omega, contact, frames=frames)
    roll_fk, pitch_fk = est.fk_attitude(q, contact, frames=frames)

    valid = n_planted >= min_planted
    return TrunkEstimate(z=float(z), v=np.asarray(v, dtype=float),
                         n_planted=n_planted, valid=valid, z_hip=float(z_hip),
                         rpy_fk=np.array([roll_fk, pitch_fk]))


class EncoderVelocity:
    """Filtered finite difference of joint position -- the second velocity.

    The driver reports a speed field in the same reply as the encoder, and it
    is free.  It has also been seen to report 8.1 rad/s on a joint whose
    encoder had moved 0.31 rad.  Anything that multiplies velocity by a
    Jacobian -- leg odometry, Cartesian damping -- turns such a glitch into
    force, so those consumers get THIS instead.

    ``alpha`` is a one-pole low pass on the difference; ``params.QD_ALPHA``
    (0.35 at 250 Hz, about a 17 Hz corner) is the value the robot runs.
    """

    def __init__(self, alpha: float = P.QD_ALPHA):
        self.alpha = float(alpha)
        self.qd = np.zeros(N_JOINTS)
        self._last_q = None
        self._last_t = None

    def update(self, t: float, q) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(N_JOINTS)
        if self._last_q is None:
            self._last_q, self._last_t = q.copy(), float(t)
            return self.qd
        dt = float(t) - self._last_t
        if dt > 1e-6:
            raw = (q - self._last_q) / dt
            self.qd += self.alpha * (raw - self.qd)
            self._last_q, self._last_t = q.copy(), float(t)
        return self.qd.copy()
