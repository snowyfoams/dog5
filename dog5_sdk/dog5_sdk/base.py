"""The interface both backends implement, and the run loop they share.

The whole point of the SDK is here: :meth:`Dog5Robot.run` is ONE loop, and
:class:`dog5_sdk.Dog5Sim` and :class:`dog5_sdk.Dog5Hardware` differ only in
what ``_read_raw``, ``_send_torque`` and ``_tick`` do underneath it.  The
safety gate, the trip checks, the operator keys, the logging and the order in
which your controller's hooks are called are the same code in simulation and on
the robot.  Anything a controller sees change between the two is a real
difference in the machine, not a difference in the harness.

    robot = Dog5Sim()            # or Dog5Hardware()
    with robot:
        robot.crouch()
        result = robot.run(MyController(), duration=10.0, tau_max=3.0)
    print(result.reason)

WHAT THE LOOP DOES, IN ORDER, EVERY TICK

    1. read the robot                       -> RobotState
    2. poll the operator keys               -> x stops, space limps
    3. run the trips                        -> a reason string stops the run
    4. call controller.update(state)        -> your (12,) torque, or None
    5. put it through the SafetyGate        -> ramp, cap, limit block, slew
    6. send it                              -> data.ctrl, or twelve CAN frames
    7. advance one control period           -> mj_step, or pace the bus

    Step 3 runs BEFORE step 4, so a controller is never asked to produce a
    torque for a state that has already tripped.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from . import params as P
from .keys import KEY_LIMP, KEY_STOP, KeyPoller
from .poses import JOINT_LABELS, N_JOINTS
from .safety import SafetyGate, TAU_START_MAX
from .state import Controller, RobotState


@dataclass
class RunResult:
    """What a run did.  ``reason`` is why it ended -- always a sentence."""

    reason: str
    ticks: int
    duration_s: float
    tripped: bool = False
    log: dict = field(default_factory=dict)

    def save(self, path: str) -> str:
        """Write the log to a .npz.  Raises if the run was not logged."""
        if not self.log:
            raise RuntimeError("this run was not logged; pass log=True to run()")
        np.savez(path, reason=self.reason, ticks=self.ticks,
                 duration_s=self.duration_s, tripped=self.tripped,
                 joint_labels=np.asarray(JOINT_LABELS), **self.log)
        return path

    def __str__(self) -> str:
        verdict = "TRIPPED" if self.tripped else "ended"
        return (f"{verdict} after {self.duration_s:.2f} s / {self.ticks} ticks: "
                f"{self.reason}")


class Dog5Robot:
    """Common interface.  Do not instantiate -- use Dog5Sim or Dog5Hardware."""

    #: Control rate.  250 Hz is a fact about the CAN bus, not a preference:
    #: twelve motors x 250 Hz = 3000 frames/s, and the drivers' input-lost
    #: watchdog is 10 ms.  The simulator matches it so that a controller's
    #: discrete-time behaviour is the same in both.
    control_hz: float = P.CONTROL_HZ

    name = "dog5"

    def __init__(self):
        self.t = 0.0
        self.started = False
        self._closed = False
        self._contact_override = None

    # -- contact ---------------------------------------------------------
    def set_contact(self, mask) -> None:
        """Declare which feet are planted, FL FR RL RR.  ``None`` clears it.

        This exists because the robot has no foot switches.  A gait controller
        knows its own contact schedule; the machine does not.  Whatever you set
        here is what ``state.contact`` reports and what the height and velocity
        estimates are computed over, in BOTH backends -- so a gait tested in
        simulation against a declared schedule is tested the same way it will
        run.

        With no override the simulator uses its MEASURED contact forces and the
        hardware assumes all four planted, which is right for a stand and wrong
        for anything with a swing phase.
        """
        self._contact_override = (
            None if mask is None
            else np.asarray(mask, dtype=bool).reshape(4).copy()
        )

    def contact_mask(self, measured=None) -> np.ndarray:
        """The contact set to use this tick: the override, else `measured`,
        else all four planted."""
        if self._contact_override is not None:
            return self._contact_override.copy()
        if measured is not None:
            return np.asarray(measured, dtype=bool).reshape(4)
        return np.ones(4, dtype=bool)

    # -- backend hooks ---------------------------------------------------
    def _arm(self) -> None:
        raise NotImplementedError

    def _disarm(self, reason: str) -> None:
        raise NotImplementedError

    def _read_raw(self) -> RobotState:
        raise NotImplementedError

    def _send_torque(self, tau) -> None:
        raise NotImplementedError

    def _tick(self) -> None:
        """Advance exactly one control period and update ``self.t``."""
        raise NotImplementedError

    def move_to(self, q_target, duration_s: float = 3.0, **kwargs):
        """Drive to a joint pose without a torque controller.  Backend-specific:
        the drivers' own position loop on hardware, a stiff PD in simulation."""
        raise NotImplementedError

    # -- lifecycle -------------------------------------------------------
    def start(self) -> "Dog5Robot":
        """Arm the robot.  On hardware this is the CAN bring-up; it prompts."""
        if self.started:
            return self
        self._arm()
        self.started = True
        return self

    def stop(self, reason: str = "stop() called") -> None:
        """Torque off.  Safe to call more than once."""
        if not self.started:
            return
        self._disarm(reason)
        self.started = False

    def close(self) -> None:
        """Stop and release the bus / the viewer."""
        self.stop("close() called")
        self._closed = True

    def __enter__(self) -> "Dog5Robot":
        return self.start()

    def __exit__(self, *exc) -> bool:
        self.close()
        return False

    # -- state -----------------------------------------------------------
    def read(self) -> RobotState:
        """The current :class:`dog5_sdk.RobotState`."""
        if not self.started:
            raise RuntimeError(f"{type(self).__name__}: call start() (or use "
                               "it as a context manager) before read()")
        return self._read_raw()

    def crouch(self, **kwargs):
        """Drive to the recorded crouch -- the pose every run starts from."""
        from .poses import Q_CROUCH
        return self.move_to(Q_CROUCH, **kwargs)

    # -- the loop --------------------------------------------------------
    def run(self, controller: Controller, *, duration: float = None,
            tau_max: float = TAU_START_MAX, gate: SafetyGate = None,
            log: bool = False, keys: bool = True, verbose: bool = True,
            status_hz: float = 2.0, on_tick=None) -> RunResult:
        """Run `controller` on this robot until it stops.

        duration   seconds, or None to run until a key or a trip stops it
        tau_max    the torque cap, N*m.  1.0 is the first-run value and the
                   default for a reason; raise it from logs, in stages
        gate       a pre-built :class:`SafetyGate` if you want other limits;
                   `tau_max` is ignored when you pass one
        log        collect per-tick arrays into ``RunResult.log``
        keys       poll the keyboard: x stops, space limps
        on_tick    optional callback(state, tau_sent) -- for your own plotting
                   or telemetry.  Runs inside the loop; keep it cheap.
        """
        if not self.started:
            self.start()

        gate = gate or SafetyGate(tau_cap=tau_max)
        controller.robot = self

        history = {key: [] for key in
                   ("t", "q", "qd", "tau_cmd", "tau_meas", "rpy", "omega",
                    "z", "v", "contact")} if log else None

        state = self.read()
        gate.start(state.t, state.q)
        controller.on_start(self, state)

        if verbose:
            print(f"[{self.name}] running {type(controller).__name__} at "
                  f"{self.control_hz:.0f} Hz, tau_cap={gate.tau_cap:.2f} N*m"
                  + (f", {duration:.1f} s" if duration else ", until stopped"))

        poller = KeyPoller() if keys else None
        if verbose and poller is not None and poller.active:
            print(f"[{self.name}] keys: {KEY_STOP} stop, space limp")

        reason = "duration reached"
        tripped = False
        limp = False
        ticks = 0
        started_t = state.t
        last_status = state.t

        try:
            while True:
                state = self.read()
                if duration is not None and state.t - started_t >= duration:
                    reason = f"duration {duration:.2f} s reached"
                    break

                if poller is not None:
                    key = poller.get()
                    if key and key.lower() == KEY_STOP:
                        reason = "operator pressed x"
                        break
                    if key == KEY_LIMP:
                        limp = not limp
                        if verbose:
                            print(f"[{self.name}] "
                                  f"{'LIMP' if limp else 'torque resumed'}")

                trip = gate.estop_reason(
                    state.q, state.qd, state.t,
                    temps=state.extra.get("temps"),
                    miss_streaks=state.extra.get("miss_streaks"),
                    errors=state.extra.get("errors"),
                )
                if trip:
                    reason, tripped = trip, True
                    controller.on_estop(state, trip)
                    break

                requested = None if limp else controller.update(state)
                if requested is None:
                    requested = np.zeros(N_JOINTS)
                requested = np.asarray(requested, dtype=float).reshape(N_JOINTS)
                if not np.all(np.isfinite(requested)):
                    reason, tripped = (
                        f"{type(controller).__name__}.update returned a "
                        "non-finite torque", True)
                    controller.on_estop(state, reason)
                    break

                tau = gate.apply(requested, state.q, state.t)
                self._send_torque(tau)

                if history is not None:
                    history["t"].append(state.t)
                    history["q"].append(state.q.copy())
                    history["qd"].append(state.qd.copy())
                    history["tau_cmd"].append(tau.copy())
                    history["tau_meas"].append(state.tau.copy())
                    history["rpy"].append(state.rpy.copy())
                    history["omega"].append(state.omega.copy())
                    history["z"].append(state.z)
                    history["v"].append(state.v.copy())
                    history["contact"].append(state.contact.copy())

                if on_tick is not None:
                    on_tick(state, tau)

                if verbose and status_hz and state.t - last_status >= 1.0 / status_hz:
                    print(f"[{self.name}] {state}")
                    last_status = state.t

                ticks += 1
                self._tick()

        except KeyboardInterrupt:
            reason, tripped = "KeyboardInterrupt", True
        finally:
            if poller is not None:
                poller.close()
            try:
                self._send_torque(np.zeros(N_JOINTS))
            except Exception:
                pass
            controller.on_stop(state, reason)
            if verbose:
                print(f"[{self.name}] {'TRIPPED' if tripped else 'stopped'}: "
                      f"{reason}")

        return RunResult(
            reason=reason, ticks=ticks, duration_s=state.t - started_t,
            tripped=tripped,
            log={key: np.asarray(value) for key, value in history.items()}
            if history is not None else {},
        )

    # -- helpers ---------------------------------------------------------
    @property
    def dt(self) -> float:
        """One control period, in seconds."""
        return 1.0 / self.control_hz

    @staticmethod
    def _pace(deadline: float) -> float:
        """Sleep-then-spin until `deadline` (perf_counter).  Returns overrun."""
        now = time.perf_counter()
        remaining = deadline - now
        if remaining <= 0:
            return -remaining
        if remaining > 0.0015:
            time.sleep(remaining - 0.001)
        while time.perf_counter() < deadline:
            pass
        return 0.0
