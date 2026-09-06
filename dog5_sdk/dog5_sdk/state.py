"""What a controller is handed, and what a controller is.

:class:`RobotState` is the ONE structure both backends fill.  The simulator and
the real robot measure completely different things, so the point of this class
is that they agree on the definitions -- the same joint order, the same height
reference, the same velocity identity, the same attitude convention.  A
controller written against it does not know, and does not need to know, which
one it is talking to.

    field     shape   meaning
    ------------------------------------------------------------------------
    t         float   seconds since the run armed
    q         (12,)   joint angles, rad, canonical order (see poses)
    qd        (12,)   joint velocities, rad/s
    tau       (12,)   MEASURED joint torque, N*m (hardware: from iq)
    rpy       (3,)    trunk roll, pitch, yaw, rad
    omega     (3,)    trunk angular rate in the BODY frame, rad/s
    contact   (4,)    bool per leg, order FL FR RL RR
    z         float   floor to trunk bottom, m -- FK through the planted feet
    v         (3,)    trunk linear velocity in the WORLD frame, m/s
    extra     dict    backend-specific: temperatures, error bytes, bus load,
                      the MuJoCo handles, the second velocity estimate...

WHY z AND v ARE ESTIMATES EVEN IN SIMULATION
    Both backends compute them the SAME way -- FK height through the planted
    feet, and the algebraic leg-odometry velocity
    ``v = -C^T (omega x s_i + J_i qdot_i)`` averaged over planted feet.  The
    simulator has the true values and puts them in ``extra`` as ``z_true`` and
    ``v_true``, but ``state.z`` and ``state.v`` are the estimates, because a
    controller that silently depends on ground truth is a controller that
    works in simulation and falls over on the robot.  Compare the two in
    ``extra`` to see exactly how much your controller is asking of the
    estimator.

    Both estimates assume PLANTED FEET.  With fewer than
    ``params.MIN_PLANTED`` feet down the trunk velocity is not determined; the
    fields hold the last valid value and ``extra["estimate_valid"]`` is False.
    In torque mode a plausible-looking fiction drives real force, so check it.

CONTROLLER
    Subclass :class:`Controller` and implement :meth:`Controller.update`.  Three
    optional hooks -- ``on_start``, ``on_stop``, ``on_estop`` -- exist so that a
    controller can hold state across a run without the runner having to know
    about it.  Nothing else is required, and there is no registration step.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .poses import JOINT_LABELS, LEGS, N_JOINTS, leg_slice


@dataclass
class RobotState:
    """One control tick's worth of robot, identical in sim and on hardware."""

    t: float
    q: np.ndarray
    qd: np.ndarray
    tau: np.ndarray
    rpy: np.ndarray
    omega: np.ndarray
    contact: np.ndarray
    z: float
    v: np.ndarray
    extra: dict = field(default_factory=dict)

    # -- convenience -----------------------------------------------------
    @property
    def roll(self) -> float:
        return float(self.rpy[0])

    @property
    def pitch(self) -> float:
        return float(self.rpy[1])

    @property
    def yaw(self) -> float:
        return float(self.rpy[2])

    def leg_q(self, leg: str) -> np.ndarray:
        """This leg's three joint angles, [abd, pitch, knee]."""
        return self.q[leg_slice(leg)]

    def leg_qd(self, leg: str) -> np.ndarray:
        return self.qd[leg_slice(leg)]

    def foot_positions(self) -> dict:
        """{leg: (3,)} foot positions in the TRUNK frame, from the encoders."""
        from . import kinematics as kin
        return {leg: kin.foot_position(leg, self.leg_q(leg)) for leg in LEGS}

    def foot_jacobians(self) -> dict:
        """{leg: (3, 3)} d(foot)/dq in the trunk frame."""
        from . import kinematics as kin
        return {leg: kin.foot_jacobian(leg, self.leg_q(leg)) for leg in LEGS}

    def copy(self) -> "RobotState":
        return RobotState(
            t=self.t, q=self.q.copy(), qd=self.qd.copy(), tau=self.tau.copy(),
            rpy=self.rpy.copy(), omega=self.omega.copy(),
            contact=self.contact.copy(), z=self.z, v=self.v.copy(),
            extra=dict(self.extra),
        )

    def __str__(self) -> str:
        return (f"t={self.t:6.3f}s  z={self.z * 1e3:4.0f}mm  "
                f"rp={np.rad2deg(self.rpy[0]):+5.1f}/"
                f"{np.rad2deg(self.rpy[1]):+5.1f}deg  "
                f"v={self.v[0]:+.2f},{self.v[1]:+.2f},{self.v[2]:+.2f}m/s  "
                f"contact={''.join('1' if c else '0' for c in self.contact)}  "
                f"max|q|={np.max(np.abs(self.q)):.2f}rad  "
                f"max|tau|={np.max(np.abs(self.tau)):.2f}Nm")


class Controller:
    """Base class for a DOG5 controller.  Implement :meth:`update`.

    The contract is deliberately small::

        class MyController(dog5_sdk.Controller):
            def update(self, state):
                return my_torque_vector          # (12,) N*m, joint order

    Return a (12,) torque in JOINT coordinates and N*m.  The runner puts it
    through :class:`dog5_sdk.safety.SafetyGate` before it reaches a motor, in
    simulation exactly as on hardware, so a controller that saturates the cap
    behaves the same in both.

    Return ``None`` to command zero torque for that tick (limp).

    POSITION MODE is not part of this interface.  It is a different command to
    the driver, not a different torque, so it lives on the robot object:
    ``robot.crouch()`` and ``robot.move_to(q, ...)``.  Call those between runs,
    not from inside ``update``.
    """

    #: Set by the runner before :meth:`on_start`; the robot you are driving.
    robot = None

    def on_start(self, robot, state: RobotState) -> None:
        """Called once, after the robot is armed and the first state is read."""

    def update(self, state: RobotState):
        """Return the (12,) joint torque for this tick, in N*m, or None."""
        raise NotImplementedError

    def on_stop(self, state: RobotState, reason: str) -> None:
        """Called once when the run ends, for any reason including a trip."""

    def on_estop(self, state: RobotState, reason: str) -> None:
        """Called when a safety trip fires, before the run is torn down."""


class ZeroTorque(Controller):
    """Commands nothing.  The robot hangs limp and back-drivable.

    Genuinely useful: it is how you check directions and calibration by hand,
    and it is the safe thing to run first on any new setup.
    """

    def update(self, state: RobotState):
        return None


class JointPD(Controller):
    """Hold a fixed joint pose: ``tau = kp (q_ref - q) - kd qd``.

    The simplest thing that does something, and the reference every richer
    controller is measured against.  Note that with ``q_ref`` at a pose the
    robot cannot hold, this will sit at the torque cap and buzz -- that is the
    cap doing its job, not the controller failing.
    """

    def __init__(self, q_ref, kp: float = 3.0, kd: float = 0.1):
        self.q_ref = np.asarray(q_ref, dtype=float).reshape(N_JOINTS)
        self.kp = float(kp)
        self.kd = float(kd)

    def update(self, state: RobotState):
        return self.kp * (self.q_ref - state.q) - self.kd * state.qd


__all__ = ["RobotState", "Controller", "ZeroTorque", "JointPD",
           "JOINT_LABELS", "N_JOINTS"]
