"""The calibrated joint contract: raw 16-bit encoder <-> joint angle in radians.

THE CONTRACT, IN TWO LINES, WITH NOTHING HIDDEN IN IT

    motoroutput_deg = raw_encoder * ENCODER_GAIN        ENCODER_GAIN = 360/65535
    joint_rad       = radians(direction * motoroutput_deg)

There is NO software zero offset and NO gearbox division.  Both absences are
deliberate and both have bitten this project before:

    NO OFFSET     the zero lives in the DRIVER, written once by the 0x19
                  set-zero command at the calibration pose (see
                  :func:`set_zero_all`).  A software offset captured at
                  start-up would mean the robot's zero changes with whatever
                  pose it happened to boot in.
    NO /10        the 0x9C encoder register on this rig already reports the
                  OUTPUT shaft, so dividing by the 10:1 reduction would be
                  wrong by exactly that factor.  Older HIL code for this robot
                  divides the same register by 10; the two calibrations are
                  incompatible and must not be mixed.

The 16-bit register wraps.  :class:`EncoderUnwrap` turns it into a continuous
angle by counting the wraps, centring the FIRST sample into [-180, 180) deg so
a joint sitting just below zero reads as a small negative number rather than
359 deg.  One tracker per joint, fed every sweep -- it cannot see a wrap it was
not shown, so do not skip samples and do not share a tracker between joints.

SOFT LIMITS are here too, because they are stated in the same coordinates.
They are SOFTWARE bounds measured against the calibrated zero, not mechanical
stops: a pose outside them is not necessarily a pose that damages anything,
and one inside them is not necessarily safe.
"""
from __future__ import annotations

import numpy as np

from .hardware_map import HARDWARE_JOINTS
from .motor import ENCODER_GAIN
from .poses import JOINT_LABELS, N_JOINTS

#: CAN IDs in canonical joint order -- the order to command and read in.
MOTOR_IDS = [joint.can_id for joint in HARDWARE_JOINTS]

#: {can_id: +/-1}.  Pass this to MotorBus(ids, dirs=MOTOR_DIRECTIONS) and
#: torque, speed and position commands are all in JOINT coordinates.
MOTOR_DIRECTIONS = {joint.can_id: joint.direction for joint in HARDWARE_JOINTS}

#: The same signs as a (12,) vector, for the encoder conversion below.
JOINT_DIRECTIONS = np.asarray(
    [joint.direction for joint in HARDWARE_JOINTS], dtype=float
)

# ---------------------------------------------------------------------------
# soft limits
# ---------------------------------------------------------------------------
ABD_LIM, PITCH_LIM, KNEE_LIM = 1.75, 2.6, 2.6

#: How far outside the soft limits a joint may be measured before the safety
#: gate calls it an e-stop rather than just refusing to push further out.
LIMIT_ESTOP_MARGIN = 0.05


def soft_limits() -> tuple:
    """(low, high), each (12,) radians, in canonical joint order."""
    low = np.tile([-ABD_LIM, -PITCH_LIM, -KNEE_LIM], 4)
    high = np.tile([+ABD_LIM, +PITCH_LIM, +KNEE_LIM], 4)
    return low, high


# ---------------------------------------------------------------------------
# the conversion, both ways
# ---------------------------------------------------------------------------
def motoroutput_deg_to_joint_rad(motoroutput_deg) -> np.ndarray:
    """Unwrapped motor-output degrees (12,) -> joint angles in radians."""
    motoroutput_deg = np.asarray(motoroutput_deg, dtype=float)
    if motoroutput_deg.shape != (N_JOINTS,):
        raise ValueError(f"motoroutput must have shape ({N_JOINTS},), "
                         f"got {motoroutput_deg.shape}")
    return np.deg2rad(JOINT_DIRECTIONS * motoroutput_deg)


def joint_rad_to_motoroutput_deg(joint_rad) -> np.ndarray:
    """The inverse -- what a joint target looks like on the motor's own dial.

    Useful for reading a commanded pose off the robot with a protractor, and
    for building a 0xA4 position payload by hand.
    """
    joint_rad = np.asarray(joint_rad, dtype=float)
    if joint_rad.shape != (N_JOINTS,):
        raise ValueError(f"joint angle must have shape ({N_JOINTS},), "
                         f"got {joint_rad.shape}")
    # direction is +/-1, so it is its own inverse.
    return JOINT_DIRECTIONS * np.rad2deg(joint_rad)


class EncoderUnwrap:
    """Continuous motor-output degrees from the wrapping 16-bit register.

    One per joint.  The first sample is centred into [-180, 180) deg; every
    later sample is compared with the previous one and a jump of more than half
    a turn is counted as a wrap.
    """

    _FULL_TURN = 65536

    def __init__(self):
        self._previous_raw = None
        self._turns = 0

    def update(self, raw: int) -> float:
        raw = int(raw)
        if not 0 <= raw < self._FULL_TURN:
            raise ValueError(f"encoder value outside uint16 range: {raw}")
        if self._previous_raw is None:
            self._turns = -1 if raw >= self._FULL_TURN // 2 else 0
        else:
            delta = raw - self._previous_raw
            if delta > self._FULL_TURN // 2:
                self._turns -= 1
            elif delta < -self._FULL_TURN // 2:
                self._turns += 1
        self._previous_raw = raw
        return (raw + self._turns * self._FULL_TURN) * ENCODER_GAIN


def new_unwrappers() -> list:
    """One :class:`EncoderUnwrap` per joint, in canonical order."""
    return [EncoderUnwrap() for _ in range(N_JOINTS)]


def joint_state(mb, unwrappers) -> tuple:
    """(q, qd) in radians from a polled MotorBus.

    `qd` is the DRIVER's speed field, direction-corrected.  It arrives in the
    same reply as the encoder, so it is free -- but it has been seen to lie:
    on 2026-08-17 it reported 8.1 rad/s on a joint whose encoder had moved
    0.31 rad.  :class:`dog5_sdk.Dog5Hardware` therefore differentiates the
    encoder as well and uses THAT for leg odometry, where a bad velocity is
    multiplied by a Jacobian and becomes force.  Both are in
    ``RobotState.extra``.

    Raises RuntimeError if any motor has not answered yet -- a missing encoder
    is never silently substituted.
    """
    raw = [mb.rec(mid).encoder for mid in MOTOR_IDS]
    missing = [mid for mid, value in zip(MOTOR_IDS, raw) if value is None]
    if missing:
        raise RuntimeError(f"no encoder reply from CAN IDs: {missing}")
    motoroutput_deg = np.asarray(
        [tracker.update(value) for tracker, value in zip(unwrappers, raw)]
    )
    q = motoroutput_deg_to_joint_rad(motoroutput_deg)
    speeds = mb.speeds_dps()          # MotorBus applies the same directions
    qd = np.deg2rad(np.asarray([speeds[mid] for mid in MOTOR_IDS]))
    return q, qd


# ---------------------------------------------------------------------------
# validation and the set-zero write
# ---------------------------------------------------------------------------
def validate() -> None:
    """Check the map is the one confirmed on the robot.  Raises on any drift."""
    if N_JOINTS != 12:
        raise ValueError(f"expected 12 configured joints, got {N_JOINTS}")
    if len(set(MOTOR_IDS)) != N_JOINTS:
        raise ValueError(f"CAN IDs must be unique: {MOTOR_IDS}")
    if sorted(MOTOR_IDS) != list(range(1, 13)):
        raise ValueError(f"CAN IDs must be 1..12: {MOTOR_IDS}")
    if any(d not in (-1, +1) for d in MOTOR_DIRECTIONS.values()):
        raise ValueError("every hardware direction must be +1 or -1")
    negative = {mid for mid, d in MOTOR_DIRECTIONS.items() if d < 0}
    if negative != {1, 3, 4, 6, 9, 12}:
        raise ValueError("direction table does not match the verified result "
                         f"{{1, 3, 4, 6, 9, 12}}: {sorted(negative)}")
    if len(JOINT_LABELS) != N_JOINTS:
        raise ValueError("joint labels do not match the joint count")
    # the conversion must round-trip
    probe = np.linspace(-1.5, 1.5, N_JOINTS)
    if not np.allclose(
        motoroutput_deg_to_joint_rad(joint_rad_to_motoroutput_deg(probe)), probe
    ):
        raise ValueError("joint <-> motoroutput conversion does not round-trip")


def set_zero_all(mb, confirm: bool = False) -> dict:
    """HARDWARE set-zero (0x19): the CURRENT pose becomes every joint's zero.

    Pose the robot at the calibration pose FIRST -- legs flat fore-aft, trunk
    on the ground -- because that pose is what every angle in this SDK is
    measured from.  :data:`dog5_sdk.poses.Q_ZERO` is it.

    Three things about 0x19 that are not negotiable:

      * the vendor warns that writing the encoder offset affects the driver's
        lifetime.  Do it when the robot is built or re-assembled, not as part
        of a run;
      * it takes effect only after a POWER CYCLE.  Read back about 0 on every
        joint afterwards to confirm;
      * it is not reversible from software.  There is no previous zero to go
        back to.

    Pass ``confirm=True`` to say you have read that.  Returns
    ``{can_id: encoder_offset or None}``; a None is a motor that never acked
    and whose zero was therefore NOT written.
    """
    if not confirm:
        raise RuntimeError(
            "set_zero_all writes the drivers' encoder offsets, takes effect "
            "only after a power cycle, and cannot be undone.  Pose the robot "
            "at the calibration pose, then call with confirm=True."
        )
    return mb.set_zero_all()
