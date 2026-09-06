"""DOG5 SDK -- write one controller, run it in MuJoCo and on the robot.

    import numpy as np
    from dog5_sdk import Controller, Dog5Sim, poses

    class Stand(Controller):
        def __init__(self, q_ref):
            self.q_ref = q_ref

        def update(self, state):
            return 3.0 * (self.q_ref - state.q) - 0.1 * state.qd

    with Dog5Sim(pose="crouch") as robot:
        robot.run(Stand(poses.stand_pose(0.15)), duration=5.0, tau_max=6.0)

Swap ``Dog5Sim`` for ``Dog5Hardware`` and the same controller drives twelve
CAN servos.  Nothing else in the example changes.

WHAT IS IN HERE

    Dog5Sim         MuJoCo, on the robot's own MJCF and meshes
    Dog5Hardware    SocketCAN, twelve LK/K-TECH drivers at 250 Hz per motor
    Controller      the one method you implement: update(state) -> (12,) N*m
    RobotState      what you are handed: q, qd, tau, rpy, omega, contact, z, v
    SafetyGate      the ramp/cap/limit/slew every torque goes through
    poses           the canonical joint order and the named poses
    kinematics      forward kinematics, foot Jacobians, leg gravity torque
    ik              damped-least-squares inverse kinematics
    statics         mass properties, the fused chain walk, the MuJoCo check
    estimator       height, leg-odometry velocity, foot-plane attitude
    calibration     encoder <-> joint angle, soft limits, the 0x19 set-zero
    motor           the CAN library itself: MotorBus, LKMotor, the gains
    params          every constant the shipped stand and trot use

WHAT IS NOT
    A controller.  This is the machine and the harness, not a gait -- writing
    the control law is the exercise.  What ships is the stack under it: the
    geometry, the calibrated joint contract, the CAN layer, the safety gate
    and a simulator that speaks the same units as the robot.

THE ONE THING TO READ BEFORE TOUCHING HARDWARE
    ``tau_max`` defaults to 1.0 N*m and the robot should be suspended for its
    first runs.  The trips in :mod:`dog5_sdk.safety` are not a safety net you
    can lean on -- see that module's docstring for a run where the tilt trip
    did not fire and the robot read perfectly level while lying on its belly.

python-can and the CAN modules are imported LAZILY: ``import dog5_sdk`` and
``Dog5Sim`` need only numpy and mujoco, so the simulator runs on a laptop with
no CAN hardware and no python-can installed.
"""
from __future__ import annotations

__version__ = "1.0.0"

from . import (calibration, estimate, estimator, ik, kinematics, motor,
               params, poses, safety, statics)
from .base import Dog5Robot, RunResult
from .calibration import (MOTOR_DIRECTIONS, MOTOR_IDS, EncoderUnwrap,
                          joint_rad_to_motoroutput_deg,
                          motoroutput_deg_to_joint_rad, soft_limits)
from .hardware_map import HARDWARE_JOINTS, HardwareJoint
from .keys import KeyPoller
from .poses import (JOINT_LABELS, LEGS, N_JOINTS, Q_CROUCH, Q_ROLL, Q_ZERO,
                    stand_pose)
from .safety import SafetyGate
from .sim import MODEL_XML, Dog5Sim
from .state import Controller, JointPD, RobotState, ZeroTorque

_LAZY = {"Dog5Hardware": ".hardware"}


def __getattr__(name):
    """Import the CAN backend only when it is actually asked for.

    ``motorbus`` imports python-can at module level.  A classmate running the
    simulator on a laptop has no CAN adapter and may have no python-can, and
    should not need either to ``import dog5_sdk``.  (``dog5_sdk.motor`` itself
    is safe to import: it loads only the pure unit-gain table eagerly -- see
    that package's docstring.)
    """
    if name in _LAZY:
        import importlib
        module = importlib.import_module(_LAZY[name], __name__)
        value = getattr(module, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(list(globals()) + list(_LAZY))


__all__ = [
    # backends
    "Dog5Sim", "Dog5Hardware", "Dog5Robot", "RunResult",
    # writing a controller
    "Controller", "RobotState", "JointPD", "ZeroTorque", "SafetyGate",
    # the robot itself
    "kinematics", "ik", "statics", "estimator", "estimate", "poses",
    "params", "calibration", "motor", "MODEL_XML",
    "LEGS", "N_JOINTS", "JOINT_LABELS", "Q_ZERO", "Q_ROLL", "Q_CROUCH",
    "stand_pose", "soft_limits",
    "MOTOR_IDS", "MOTOR_DIRECTIONS", "HARDWARE_JOINTS", "HardwareJoint",
    "EncoderUnwrap", "motoroutput_deg_to_joint_rad",
    "joint_rad_to_motoroutput_deg", "KeyPoller",
    "__version__",
]
