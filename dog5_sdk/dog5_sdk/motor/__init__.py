"""The DOG5 CAN motor library, exactly as it runs on the robot.

The four modules in this directory are BYTE-FOR-BYTE copies of the research
repository's ``src/motor/`` (``tools/sync_from_repo.py`` regenerates them and
``tests/test_sync.py`` gates them).  They are the code that has driven twelve
LK/K-TECH drivers at 250 Hz per motor on the real machine, including the
over-CAN recovery of the input-signal-lost latch that removes the power cycle
between runs.  Nothing here is re-derived or cleaned up for the SDK.

    from dog5_sdk.motor import MotorBus, motorbus

WHY THE LOADING IS NOT A PLAIN IMPORT
    Being verbatim, the modules import each other by FLAT name --
    ``import motor_gains as config``, ``from motor_library import open_bus`` --
    which is what they do in the research tree, where ``dog5_paths`` puts every
    source directory on ``sys.path``.  This package must not do that: adding
    directories to ``sys.path`` is how a stray ``config.py`` shadows another
    one and kills the CAN layer with an unrelated ``AttributeError`` several
    imports later.  So each module is loaded from its own file and registered
    under both names -- the flat one its siblings expect, and the dotted
    ``dog5_sdk.motor.*`` one you should import it by.  ``sys.path`` is never
    touched, and a module already present under its flat name (you also have
    the research tree imported in the same process) is reused, not shadowed.

WHY MOST OF IT IS LAZY
    ``motorbus`` and ``motor_library`` import ``python-can`` at module level.
    A classmate running the simulator on a laptop has no CAN adapter and may
    have no python-can, and must not need either to ``import dog5_sdk``.  Only
    ``motor_gains`` -- pure numbers, no imports -- is loaded eagerly, because
    the joint calibration needs ``ENCODER_GAIN`` and nothing else.  Everything
    else arrives the first time you name it.
"""
from __future__ import annotations

import importlib.util
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))

#: Load order is the dependency order: motorbus needs motor_library, which
#: needs mac_can on macOS.
_ORDER = ("mac_can", "motor_gains", "motor_library", "motorbus")

#: Loaded on first use.  motor_gains is the exception, loaded below.
_LAZY_MODULES = ("mac_can", "motor_library", "motorbus")

#: {attribute: (module, name in it)} for what is worth exposing at package
#: level without naming the module it lives in.
_LAZY_ATTRS = {
    "MotorBus": ("motorbus", "MotorBus"),
    "RoundRobinBus": ("motorbus", "RoundRobinBus"),
    "MotorRecord": ("motorbus", "MotorRecord"),
    "arm_motors": ("motorbus", "arm_motors"),
    "decode_errors": ("motorbus", "decode_errors"),
    "LKMotor": ("motor_library", "LKMotor"),
    "open_bus": ("motor_library", "open_bus"),
}


def _load(name: str):
    """Import ``<name>.py`` from this directory under its flat module name."""
    existing = sys.modules.get(name)
    if existing is not None:
        sys.modules.setdefault(f"{__name__}.{name}", existing)
        return existing

    spec = importlib.util.spec_from_file_location(
        name, os.path.join(_HERE, f"{name}.py")
    )
    module = importlib.util.module_from_spec(spec)
    # Registered BEFORE exec so a sibling's `import motor_library` during exec
    # finds it, and so a failed load leaves no half-built module behind.
    sys.modules[name] = module
    sys.modules[f"{__name__}.{name}"] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        sys.modules.pop(f"{__name__}.{name}", None)
        raise
    return module


def _load_chain(name: str):
    """Load `name` and everything before it in the dependency order."""
    for candidate in _ORDER:
        module = _load(candidate)
        globals()[candidate] = module
        if candidate == name:
            return module
    raise AttributeError(name)


# motor_gains has no imports at all, so it costs nothing, and the joint
# calibration needs its ENCODER_GAIN.  mac_can loads with it only to keep the
# order honest: it imports ctypes and os, nothing that needs a CAN adapter.
motor_gains = _load_chain("motor_gains")

#: Unit conversions -- the single source for every one of them.
ENCODER_GAIN = motor_gains.encoder_gain      # raw encoder * this = motor-output deg
TORQUE_GAIN = motor_gains.torque_gain        # N*m * this = iq LSB
VEL_GAIN = motor_gains.vel_gain              # commanded dps * this = speed LSB
VEL_STATE_GAIN = motor_gains.vel_state_gain  # raw speed / this = output dps
POS_GAIN = motor_gains.pos_gain              # output deg * this = angle LSB
MAX_SPEED_POS = motor_gains.max_speed_pos


def __getattr__(name):
    if name in _LAZY_MODULES:
        return _load_chain(name)
    if name in _LAZY_ATTRS:
        module_name, attribute = _LAZY_ATTRS[name]
        value = getattr(_load_chain(module_name), attribute)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals()) | set(_LAZY_MODULES) | set(_LAZY_ATTRS))


__all__ = [
    "mac_can", "motor_gains", "motor_library", "motorbus",
    "MotorBus", "RoundRobinBus", "MotorRecord", "LKMotor", "open_bus",
    "arm_motors", "decode_errors",
    "ENCODER_GAIN", "TORQUE_GAIN", "VEL_GAIN", "VEL_STATE_GAIN", "POS_GAIN",
    "MAX_SPEED_POS",
]
