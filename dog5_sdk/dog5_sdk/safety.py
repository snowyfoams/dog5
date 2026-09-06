"""The gate every torque command goes through, and the trips that stop a run.

This is the shipped hardware gate from the DOG5 stand runner, lifted out so
that :class:`dog5_sdk.Dog5Sim` and :class:`dog5_sdk.Dog5Hardware` can put YOUR
controller behind exactly the same one.  Your control law returns a torque; the
gate decides what actually reaches the motors:

    1. RAMP        the cap rises from 0 to ``tau_cap`` over ``ramp_s`` when the
                   run arms, so an aggressive first tick cannot be a step
    2. CAP         |tau| <= tau_cap, then |tau| <= TAU_HARD unconditionally
    3. LIMIT BLOCK a joint at or past its soft limit cannot be pushed further
                   out; the block is immediate and is re-applied after the
                   slew limiter so residual torque cannot leak through
    4. SLEW        |dtau/dt| <= tau_slew, which is what turns a controller
                   discontinuity into a ramp instead of an impact

and, separately from shaping, :meth:`SafetyGate.estop_reason` answers "should
this run stop right now" from position, speed, temperature, missed CAN replies
and the drivers' own fault bits.

THE TRIPS ARE NOT A SAFETY NET YOU CAN LEAN ON
    Chapter 4 of the research repository documents a run where the tilt trip
    did not fire, the robot ended up on its belly at 2.4x body weight, and read
    perfectly LEVEL while doing so -- because a robot lying flat is level.
    Every gate here can only see what it measures.  ``tau_cap`` starting at
    1.0 N*m, and a robot that is mechanically supported for its first runs, are
    the things actually keeping the machine intact.

THE OVERSPEED TRIP HAS TWO TIERS, AND THE LOWER ONE NEEDS TWO WITNESSES
    The driver's speed field and the encoder are separate numbers in the same
    reply.  A sustained trip on the driver field alone was a nuisance-trip
    source (a spurious 5.9 rad/s on a stationary joint), so the lower tier
    fires only when finite-differenced ENCODER position agrees, for
    ``QD_ESTOP_STREAK`` consecutive checks.  The hard tier fires immediately on
    either.  Call :meth:`estop_reason` (or :meth:`overspeed_reason`) exactly
    once per control decision -- the streak counter counts calls.
"""
from __future__ import annotations

import numpy as np

from .calibration import LIMIT_ESTOP_MARGIN, MOTOR_IDS, soft_limits
from .poses import JOINT_LABELS, N_JOINTS

# ---------------------------------------------------------------------------
# limits.  Defaults are the CONSERVATIVE first-run values from the stand
# runner, not the trot's; raise them deliberately, from logs, one at a time.
# ---------------------------------------------------------------------------
#: Absolute ceiling, above any tau_cap.  The 0xA1 iq field saturates at
#: 2048 LSB / 206.04 = 9.94 N*m anyway, so nothing above this is even sendable.
TAU_HARD_NM = 9.0

#: --tau-max for a first run on a supported robot.  There is a reason it is 1.
TAU_START_MAX = 1.0

#: The ceiling the staged runner will raise tau_cap to.  Past this, you are
#: not staging any more.
TAU_STAGED_MAX = 3.0

DEFAULT_TAU_SLEW_NM_S = 5.0
DEFAULT_TORQUE_RAMP_S = 1.0

QD_ESTOP = 7.0                          # rad/s, sustained, needs both witnesses
QD_ESTOP_HARD = 8.0                     # rad/s, immediate
QD_ESTOP_STREAK = 3                     # consecutive confirmed checks
QD_ESTOP_ENCODER_CONFIRM_RATIO = 0.5    # encoder must exceed this * QD_ESTOP

TEMP_ESTOP_C = 80
MISS_ESTOP = 20                         # consecutive missed CAN replies


class SafetyGate:
    """Torque shaping (ramp, cap, limit block, slew) plus the e-stop tests."""

    def __init__(self, tau_cap: float = TAU_START_MAX, *,
                 tau_hard: float = TAU_HARD_NM,
                 tau_slew: float = DEFAULT_TAU_SLEW_NM_S,
                 ramp_s: float = DEFAULT_TORQUE_RAMP_S,
                 qd_estop: float = QD_ESTOP,
                 qd_estop_hard: float = QD_ESTOP_HARD,
                 limits=None):
        if tau_cap > tau_hard:
            raise ValueError(f"tau_cap {tau_cap} exceeds the hard limit "
                             f"{tau_hard} N*m")
        self.tau_cap = float(tau_cap)
        self.tau_hard = float(tau_hard)
        self.tau_slew = float(tau_slew)
        self.ramp_s = float(ramp_s)
        self.qd_estop = float(qd_estop)
        self.qd_estop_hard = float(qd_estop_hard)
        self.low, self.high = soft_limits() if limits is None else limits

        self.started_at = None
        self.last_time = None
        self.previous_tau = np.zeros(N_JOINTS)
        self.qd_streaks = np.zeros(N_JOINTS, dtype=int)
        self.qd_peak = np.zeros(N_JOINTS)
        self.encoder_qd = np.zeros(N_JOINTS)
        self.encoder_qd_peak = np.zeros(N_JOINTS)
        self._overspeed_last_q = None
        self._overspeed_last_time = None

    # -- lifecycle -------------------------------------------------------
    def start(self, now: float, q=None) -> None:
        """Arm the gate at `now`; the ramp starts here.  `q` seeds the
        encoder-velocity witness so the first check is not a step."""
        self.started_at = float(now)
        self.last_time = float(now)
        if q is not None:
            self._overspeed_last_q = np.asarray(q, dtype=float).copy()
            self._overspeed_last_time = float(now)

    def cap_now(self, now: float) -> float:
        """The cap in force at `now` -- ``tau_cap`` scaled by the ramp."""
        if self.started_at is None:
            raise RuntimeError("SafetyGate.start() has not been called")
        fraction = np.clip((now - self.started_at) / self.ramp_s, 0.0, 1.0)
        return self.tau_cap * float(fraction)

    # -- shaping ---------------------------------------------------------
    def apply(self, tau, q, now: float) -> np.ndarray:
        """Shape a requested (12,) torque into the one to actually send."""
        cap = self.cap_now(now)
        q = np.asarray(q, dtype=float)
        limited = np.clip(np.asarray(tau, dtype=float), -cap, cap)
        limited = np.clip(limited, -self.tau_hard, self.tau_hard)
        limited = np.where((q >= self.high) & (limited > 0.0), 0.0, limited)
        limited = np.where((q <= self.low) & (limited < 0.0), 0.0, limited)

        dt = np.clip(now - self.last_time, 1.0e-4, 0.05)
        max_change = self.tau_slew * dt
        output = self.previous_tau + np.clip(
            limited - self.previous_tau, -max_change, max_change
        )
        # A directional limit block is immediate, even where the slew limiter
        # would otherwise leave residual torque pointing further out of bounds.
        output = np.where((q >= self.high) & (output > 0.0), 0.0, output)
        output = np.where((q <= self.low) & (output < 0.0), 0.0, output)
        self.previous_tau = output
        self.last_time = float(now)
        return output

    # -- trips -----------------------------------------------------------
    def overspeed_reason(self, qd, q, now: float):
        """Two-tier overspeed trip.  One call == one check; see the module
        docstring.  Returns a reason string, or None."""
        qd = np.asarray(qd, dtype=float)
        q = np.asarray(q, dtype=float)
        speed = np.abs(qd)
        self.qd_peak = np.maximum(self.qd_peak, speed)

        if self._overspeed_last_q is None:
            confirmed = np.zeros(N_JOINTS, dtype=bool)
            self.encoder_qd.fill(0.0)
        else:
            dt = float(now) - self._overspeed_last_time
            if np.isfinite(dt) and dt > 0.0:
                self.encoder_qd = (q - self._overspeed_last_q) / dt
                limit = QD_ESTOP_ENCODER_CONFIRM_RATIO * self.qd_estop
                confirmed = np.abs(self.encoder_qd) > limit
                self.encoder_qd_peak = np.maximum(
                    self.encoder_qd_peak, np.abs(self.encoder_qd)
                )
            else:
                self.encoder_qd.fill(0.0)
                confirmed = np.zeros(N_JOINTS, dtype=bool)
        self._overspeed_last_q = q.copy()
        self._overspeed_last_time = float(now)

        if np.any(speed > self.qd_estop_hard):
            index = int(np.argmax(speed))
            return (f"overspeed {JOINT_LABELS[index]}: {qd[index]:+.1f} rad/s "
                    f"over the {self.qd_estop_hard:.1f} rad/s hard limit")

        self.qd_streaks = np.where(
            (speed > self.qd_estop) & confirmed, self.qd_streaks + 1, 0
        )
        tripped = self.qd_streaks >= QD_ESTOP_STREAK
        if np.any(tripped):
            index = int(np.argmax(np.where(tripped, speed, 0.0)))
            return (f"confirmed overspeed {JOINT_LABELS[index]}: driver "
                    f"{qd[index]:+.1f} rad/s, encoder "
                    f"{self.encoder_qd[index]:+.1f} rad/s for "
                    f"{int(self.qd_streaks[index])} consecutive checks over "
                    f"the {self.qd_estop:.1f} rad/s driver limit")
        return None

    def estop_reason(self, q, qd, now: float, *, temps=None,
                     miss_streaks=None, errors=None,
                     enforce_position_limits: bool = True):
        """Should this run stop?  Returns a reason string, or None.

        `temps`, `miss_streaks` and `errors` are the hardware-only witnesses
        and may be omitted (the simulator has none of them).
        """
        q = np.asarray(q, dtype=float)
        qd = np.asarray(qd, dtype=float)
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(qd)):
            return "invalid encoder or velocity state"

        if enforce_position_limits:
            outside = ((q < self.low - LIMIT_ESTOP_MARGIN)
                       | (q > self.high + LIMIT_ESTOP_MARGIN))
            if np.any(outside):
                index = int(np.flatnonzero(outside)[0])
                return (f"joint limit exceeded: {JOINT_LABELS[index]}="
                        f"{q[index]:+.2f} rad")

        overspeed = self.overspeed_reason(qd, q, now)
        if overspeed:
            return overspeed

        if temps is not None:
            temps = np.asarray(temps)
            if np.any(temps > TEMP_ESTOP_C):
                index = int(np.argmax(temps))
                return (f"overtemp CAN {MOTOR_IDS[index]}: "
                        f"{int(temps[index])} C")

        if miss_streaks is not None:
            miss_streaks = np.asarray(miss_streaks)
            if np.any(miss_streaks >= MISS_ESTOP):
                index = int(np.argmax(miss_streaks))
                return (f"CAN {MOTOR_IDS[index]} missed "
                        f"{int(miss_streaks[index])} consecutive replies")

        if errors:
            # bit 7 (0x80) is the input-signal-lost latch, which the runner
            # recovers over CAN; bits 0-6 are real faults and stop the run.
            hard = {mid: err for mid, err in errors.items() if err & 0x7F}
            if hard:
                detail = ", ".join(f"CAN {mid}=0x{err:02x}"
                                   for mid, err in hard.items())
                return f"motor fault: {detail}"
        return None


class CanMissMonitor:
    """Consecutive-missed-reply streak per joint, from a MotorBus's counters."""

    def __init__(self, mb):
        self.previous = np.asarray([mb.rec(mid).missed for mid in MOTOR_IDS])
        self.streaks = np.zeros(N_JOINTS, dtype=int)

    def update(self, mb) -> np.ndarray:
        current = np.asarray([mb.rec(mid).missed for mid in MOTOR_IDS])
        added = current - self.previous
        self.streaks = np.where(added > 0, self.streaks + added, 0)
        self.previous = current
        return self.streaks.copy()
