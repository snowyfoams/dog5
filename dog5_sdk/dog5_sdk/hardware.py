"""The CAN backend: your controller, on the real DOG5.

    from dog5_sdk import Dog5Hardware, JointPD, poses

    with Dog5Hardware() as robot:            # prompts, arms all twelve motors
        robot.crouch()                       # native 0xA4 position mode
        robot.run(MyController(), duration=10.0, tau_max=1.0)

BEFORE THE FIRST RUN, IN THIS ORDER
    1. ``sudo ip link set can0 up type can bitrate 1000000`` (or the repo's
       ``bringup/t1_preflight.sh``), then check all twelve answer.
    2. Run your controller in :class:`dog5_sdk.Dog5Sim` first.  Every trip you
       find there is one you do not find with the robot in the air.
    3. SUSPEND THE ROBOT.  Not "hold it" -- suspend it.
    4. ``tau_max=1.0``.  That is the shipped default and it is the value the
       whole torque track was brought up on.  Raise it in stages, from logs.

THE LOOP THIS DRIVES IS THE ONE THAT FLEW
    Twelve motors share one 1 Mbit/s bus.  One frame per slot, twelve slots per
    sweep, 250 Hz per motor and 250 Hz for the whole 12-vector -- because the
    0xA1 torque reply carries temperature, iq, speed and encoder in the SAME
    frame, so commanding a joint IS sampling it.  (250/12 = 20.8 Hz is wrong,
    appears in older code for this robot, and once cost the project a year.)

    Strictly single-threaded and non-blocking: no reader thread, no queue.  A
    thread under the GIL adds millisecond-scale jitter on a Pi-class host; a
    paced poll loop is deterministic and measurable.

WHAT THIS BACKEND CANNOT MEASURE, AND WHAT IT DOES INSTEAD
    CONTACT   there are no foot switches.  ``state.contact`` is whatever you
              declare with :meth:`Dog5Robot.set_contact` -- your gait clock
              knows the schedule and the robot does not.  The default is all
              four planted, which is right for a stand and wrong for a gait.
    ATTITUDE  needs the AHRS.  With no ``ahrs=`` object, roll and pitch fall
              back to the FOOT-PLANE attitude from the encoders, yaw is 0 and
              ``omega`` is zeros.  That is attitude relative to the floor the
              robot is standing on, not relative to gravity -- fine for a
              static check, not a substitute for an IMU in a balance loop.
              You are warned once, loudly, at arm time.
"""
from __future__ import annotations

import math
import time

import numpy as np

from . import estimator as est
from . import params as P
from .base import Dog5Robot
from .calibration import (MOTOR_DIRECTIONS, MOTOR_IDS, joint_state,
                          new_unwrappers, validate)
from .estimate import EncoderVelocity, leg_frames_all, trunk_estimate
from .poses import N_JOINTS
from .safety import CanMissMonitor
from .state import RobotState

#: How often each joint's control frame is replaced by a 0x9A status request,
#: so the driver's error byte stays fresh enough for the fault trip.
FAULT_STATUS_HZ = 5.0

#: How often an input-signal-lost latch (error bit 0x80) may be re-cleared.
RECOVER_PERIOD_S = 0.1

#: How old a motor's last reply may be before :meth:`Dog5Hardware.read` refuses
#: to build a state out of it.  20 sweeps at 250 Hz -- the same order as the
#: missed-reply e-stop.  This exists because the QUIET failure is the dangerous
#: one: every telemetry field on a MotorBus is last-known, so a bus that has
#: gone silent reads as a robot frozen in its last pose, and a controller will
#: drive torque from it without noticing.
STALE_REPLY_S = 0.08

#: Soft stop: a short velocity-damping ramp before the torque goes to zero, so
#: a loaded leg is let down rather than dropped.
STOP_DAMPING = 0.05
STOP_SECONDS = 0.4

#: Motor-side speed cap for position moves (1 dps per LSB, motor shaft, so the
#: joint moves at a tenth of this).  Lower it to go slower; do not add a ramp,
#: the driver's own loop is doing the approach.
POSITION_MAX_MOTOR_DPS = 100.0
POSITION_POSE_TOL_RAD = 0.08
POSITION_QD_TOL_RAD_S = 0.25
POSITION_SETTLE_S = 0.5
POSITION_TIMEOUT_S = 30.0


class Dog5Hardware(Dog5Robot):
    """DOG5 over SocketCAN, behind the same interface as the simulator.

    ahrs        an object with ``sample()`` and ``is_stale(s)`` -- the DETA10
                wrapper, or anything that quacks like it (see the module
                docstring for what happens with None)
    prompt      wait for the operator to confirm before the first torque
    control_hz  250 Hz.  It is a fact about the bus and the drivers' 10 ms
                input-lost watchdog, not a preference
    bus         an already-open python-can bus to share; None opens one
    """

    name = "dog5-hw"

    def __init__(self, *, ahrs=None, prompt: bool = True,
                 control_hz: float = P.CONTROL_HZ, bus=None,
                 ids=None, dirs=None, bitrate: int = 1_000_000,
                 setpoint_roll_deg: float = P.SETPOINT_ROLL_DEG,
                 setpoint_pitch_deg: float = P.SETPOINT_PITCH_DEG):
        super().__init__()
        validate()
        self.control_hz = float(control_hz)
        self.ids = list(MOTOR_IDS if ids is None else ids)
        self.dirs = dict(MOTOR_DIRECTIONS if dirs is None else dirs)
        self.ahrs = ahrs
        self.prompt = bool(prompt)
        self._bitrate = int(bitrate)
        self._external_bus = bus
        self.mb = None

        # The IMU mount's own tilt, measured on this rig.  Subtracting it is
        # what makes "level" mean level; without it the attitude loop pushes
        # against a constant.
        self._sp_roll = math.radians(setpoint_roll_deg)
        self._sp_pitch = math.radians(setpoint_pitch_deg)

        self._unwrap = new_unwrappers()
        self._encoder_qd = EncoderVelocity()
        self._miss = None
        self._tau = np.zeros(N_JOINTS)
        self._t0 = None
        self._slot = 1.0 / (self.control_hz * N_JOINTS)
        self._deadline = None
        self._next_status = None
        self._last_recover = {mid: -RECOVER_PERIOD_S for mid in self.ids}
        self._ahrs_warned = False

    # -- lifecycle -------------------------------------------------------
    def _arm(self) -> None:
        from .motor import MotorBus                          # noqa: PLC0415

        print(f"[{self.name}] joints  = {list(self.ids)}")
        print(f"[{self.name}] q = radians(direction * motoroutput); "
              "no software offset, no gearbox division")
        if self.ahrs is None and not self._ahrs_warned:
            print(f"[{self.name}] NO AHRS: roll/pitch will be the FOOT-PLANE "
                  "attitude from the encoders, yaw 0, omega 0.  That is "
                  "attitude relative to the floor, NOT to gravity.")
            self._ahrs_warned = True

        self.mb = MotorBus(self.ids, bus=self._external_bus,
                           bitrate=self._bitrate, dirs=self.dirs)
        print(f"[{self.name}] arming with a zero-torque stream -- switch 24 V "
              "on when prompted.  A latched motor is cleared over CAN.")
        if not self.mb.arm(rate_hz=self.control_hz):
            self.mb.close()
            self.mb = None
            raise RuntimeError("not all motors armed")

        self.mb.poll()
        self._miss = CanMissMonitor(self.mb)
        self._encoder_qd = EncoderVelocity()
        self._unwrap = new_unwrappers()
        self._t0 = time.perf_counter()
        self.t = 0.0
        self._deadline = time.perf_counter() + self._slot
        period = 1.0 / FAULT_STATUS_HZ
        self._next_status = np.asarray(
            [period + index * period / N_JOINTS for index in range(N_JOINTS)]
        )
        self._tau = np.zeros(N_JOINTS)

        if self.prompt:
            state = self._read_raw()
            print(f"[{self.name}] ZERO-TORQUE CHECK -- every motor is limp and "
                  "back-drivable.")
            print(f"[{self.name}] {state}")
            print(f"[{self.name}] SUPPORT THE ROBOT, then press ENTER to arm "
                  "torque (Ctrl-C aborts).")
            input()

    def _disarm(self, reason: str) -> None:
        if self.mb is None:
            return
        print(f"[{self.name}] stopping: {reason}")
        try:
            self._soft_stop()
        except Exception as exc:
            print(f"[{self.name}] soft stop failed: {exc}; sending STOP")
        finally:
            try:
                self.mb.close()
            finally:
                self.mb = None

    def _soft_stop(self, seconds: float = STOP_SECONDS) -> None:
        """Damp the joints to a halt, then let go.  Not a brake -- the motors
        have none -- but it stops a loaded leg from being dropped."""
        deadline = time.perf_counter() + self._slot
        end = time.perf_counter() + seconds
        index = 0
        while time.perf_counter() < end:
            self.mb.poll()
            mid = self.ids[index % N_JOINTS]
            qd = math.radians(self.mb.speeds_dps()[mid])
            self.mb.torque(mid, -STOP_DAMPING * qd)
            index += 1
            self._pace(deadline)
            deadline += self._slot

    # -- state -----------------------------------------------------------
    def _read_raw(self) -> RobotState:
        mb = self.mb
        mb.poll()
        now = time.perf_counter()
        self.t = now - self._t0
        self._refuse_stale(mb, now)

        q, qd_driver = joint_state(mb, self._unwrap)
        qd_encoder = self._encoder_qd.update(self.t, q)
        frames = leg_frames_all(q)
        contact = self.contact_mask()

        rpy, omega, ahrs_ok = self._attitude(q, contact, frames)
        # Leg odometry multiplies velocity by a Jacobian, so it gets the
        # ENCODER velocity -- see estimate.EncoderVelocity for why.
        estimate = trunk_estimate(q, qd_encoder, rpy, omega, contact,
                                  frames=frames)

        temps = mb.temps()
        errors = mb.errors()
        torques = mb.torques_nm()
        tau = np.asarray([torques[mid] for mid in self.ids], dtype=float)

        return RobotState(
            t=self.t, q=q, qd=qd_driver, tau=tau, rpy=rpy, omega=omega,
            contact=contact, z=estimate.z, v=estimate.v,
            extra={
                "qd_driver": qd_driver,
                "qd_encoder": qd_encoder,
                "estimate_valid": estimate.valid,
                "n_planted": estimate.n_planted,
                "z_hip": estimate.z_hip,
                "rpy_fk": estimate.rpy_fk,
                "ahrs_ok": ahrs_ok,
                "frames": frames,
                # the hardware-only e-stop witnesses; base.run() reads these
                "temps": np.asarray(
                    [temps[mid] if temps[mid] is not None else 0
                     for mid in self.ids], dtype=int),
                "errors": errors,
                "miss_streaks": self._miss.update(mb),
                "voltages": mb.voltages(),
                "bus_load": mb.rr.bus_load(),
            },
        )

    @staticmethod
    def _refuse_stale(mb, now: float, max_age_s: float = STALE_REPLY_S) -> None:
        """Raise if any motor's last reply is older than `max_age_s`.

        ``joint_state`` only catches a motor that has NEVER replied.  A motor
        that answered once and then went quiet -- an unplugged connector, a
        driver that browned out, a bus that dropped off -- keeps its last
        encoder value in the record for as long as the process runs.  Building
        a state out of that is how a controller ends up commanding torque
        against a pose the robot left several seconds ago.
        """
        stale = []
        for mid in mb.ids:
            last = mb.rec(mid).last_reply_t
            if last is None or (now - last) > max_age_s:
                stale.append(mid)
        if stale:
            raise RuntimeError(
                f"no encoder reply from CAN IDs {stale} in the last "
                f"{max_age_s * 1e3:.0f} ms -- their telemetry is stale and "
                "will not be used.  Check power, wiring and termination.")

    def _attitude(self, q, contact, frames) -> tuple:
        """(rpy, omega, ahrs_ok).  Falls back to the foot plane with no AHRS."""
        if self.ahrs is not None:
            sample = self.ahrs.sample()
            stale = getattr(self.ahrs, "is_stale", lambda _s: False)
            if sample is not None and not stale(P.AHRS_STALE_S):
                rpy = np.array([
                    math.radians(sample.roll_deg) - self._sp_roll,
                    math.radians(sample.pitch_deg) - self._sp_pitch,
                    math.radians(sample.yaw_deg),
                ])
                omega = np.deg2rad([sample.roll_rate_dps,
                                    sample.pitch_rate_dps,
                                    sample.yaw_rate_dps])
                return rpy, omega, True
        if not np.any(contact):
            return np.zeros(3), np.zeros(3), False
        roll_fk, pitch_fk = est.fk_attitude(q, contact, frames=frames)
        return (np.array([roll_fk, pitch_fk, 0.0]), np.zeros(3),
                self.ahrs is None)

    # -- actuation -------------------------------------------------------
    def _send_torque(self, tau) -> None:
        self._tau = np.asarray(tau, dtype=float).reshape(N_JOINTS)

    def _tick(self) -> None:
        """One 250 Hz sweep: twelve slots, one CAN frame each, paced.

        The stored torque vector goes out here rather than in
        :meth:`_send_torque` because the bus is the thing being scheduled --
        one frame per 333 us slot is what keeps the TX queue from overflowing
        and the replies from colliding.
        """
        mb = self.mb
        elapsed = time.perf_counter() - self._t0
        for index in range(N_JOINTS):
            mb.poll()
            mid = self.ids[index]
            if elapsed >= self._next_status[index]:
                # Substitute a status read for this slot's control frame: the
                # driver's error byte is what the fault trip reads, and a stale
                # one is worse than a slot of missing torque.
                mb.status1_req(mid)
                self._next_status[index] += 1.0 / FAULT_STATUS_HZ
            else:
                mb.torque(mid, float(self._tau[index]))
            self._pace(self._deadline)
            self._deadline += self._slot
            elapsed = time.perf_counter() - self._t0
        mb.poll()
        self._recover_latched(elapsed)
        self.t = elapsed

    def _recover_latched(self, elapsed: float) -> None:
        """Clear the input-signal-lost latch (0x80) over CAN, no power cycle.

        Bit 7 means the driver stopped hearing commands for longer than its
        10 ms watchdog -- a dropped frame, a blocking print, a scheduler stall.
        It is recoverable in place with 0x9B -> 0x88, which is the t7 result
        that removed the power cycle between runs.  The other error bits are
        real faults and are left for the e-stop.
        """
        errors = self.mb.errors()
        targets = [mid for mid, err in errors.items()
                   if err & 0x80
                   and elapsed - self._last_recover[mid] >= RECOVER_PERIOD_S]
        if not targets:
            return
        detail = ", ".join(f"CAN {mid}" for mid in targets)
        print(f"[{self.name}] input-lost latch on {detail}; 0x9B -> 0x88")
        self.mb.recover(targets, settle_s=0.0, verify=False)
        for mid in targets:
            # The verdict must come from a FRESH 0x9A, not the pre-recovery
            # byte, or the trip fires on an error that is already cleared.
            self.mb.rec(mid).error = None
            self._last_recover[mid] = elapsed

    # -- position mode ---------------------------------------------------
    def move_to(self, q_target, duration_s: float = None, *,
                max_motor_dps: float = POSITION_MAX_MOTOR_DPS,
                pose_tol: float = POSITION_POSE_TOL_RAD,
                qd_tol: float = POSITION_QD_TOL_RAD_S,
                settle_s: float = POSITION_SETTLE_S,
                timeout_s: float = POSITION_TIMEOUT_S,
                verbose: bool = True) -> np.ndarray:
        """Drive to a joint pose with the drivers' OWN 0xA4 position loop.

        No software profile: the target is a constant and ``max_motor_dps``
        (motor side, 1 dps/LSB, so a tenth of that at the joint) is what limits
        the approach.  To go slower, lower the cap -- do not add a ramp.

        Returns once the pose is held: within `pose_tol` AND under `qd_tol` for
        `settle_s`.  Both halves are needed -- pose alone passes while a leg is
        still swinging through the target, speed alone passes while a leg is
        stalled short of it.  `duration_s` is accepted and ignored, so the call
        is interchangeable with the simulator's.

        Raises RuntimeError on a timeout.
        """
        if not self.started:
            self.start()
        target = np.asarray(q_target, dtype=float).reshape(N_JOINTS)
        target_joint_deg = np.rad2deg(target)
        mb = self.mb

        if verbose:
            print(f"[{self.name}] 0xA4 position move, capped at "
                  f"{max_motor_dps:.0f} motor dps.  Settle = "
                  f"|dq| < {pose_tol:.2f} rad and |qd| < {qd_tol:.2f} rad/s "
                  f"for {settle_s:.1f} s.")

        start = time.perf_counter()
        deadline = time.perf_counter() + self._slot
        settled_since = None
        index = 0
        q = target.copy()

        while True:
            mb.poll()
            slot_index = index % N_JOINTS
            if slot_index == 0:
                now = time.perf_counter()
                q, qd = joint_state(mb, self._unwrap)
                ok = (float(np.max(np.abs(q - target))) < pose_tol
                      and float(np.max(np.abs(qd))) < qd_tol)
                if ok:
                    settled_since = settled_since or now
                    if now - settled_since >= settle_s:
                        break
                else:
                    settled_since = None
                if now - start > timeout_s:
                    raise RuntimeError(
                        f"position move did not settle in {timeout_s:.0f} s; "
                        f"worst joint is {float(np.max(np.abs(q - target))):.3f} "
                        f"rad from the target")
            mid = self.ids[slot_index]
            # MotorBus.position() applies this motor's direction itself, so the
            # argument is the JOINT angle in degrees and the wire gets the
            # motor-output angle.  Do not pre-apply the direction here.
            mb.position(mid, float(target_joint_deg[slot_index]),
                        max_dps=max_motor_dps)
            index += 1
            self._pace(deadline)
            deadline += self._slot

        # Hand back to torque: the loop clock restarts here, so the safety
        # gate's ramp starts from the pose the move actually reached.
        self._t0 = time.perf_counter() - self.t
        self._deadline = time.perf_counter() + self._slot
        if verbose:
            print(f"[{self.name}] move_to: settled, worst joint "
                  f"{float(np.max(np.abs(q - target))) * 1e3:.0f} mrad out")
        return q

    # -- extras ----------------------------------------------------------
    def status(self) -> dict:
        """One-shot 0x9A of every motor: {can_id: (error_byte, state)}."""
        return self.mb.status1()

    def temperatures(self) -> dict:
        return self.mb.temps()
