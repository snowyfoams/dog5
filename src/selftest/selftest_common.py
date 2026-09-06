#!/usr/bin/env python3
"""The PASS/FAIL protocol every offline gate in this repository prints.

WHY THIS EXISTS
    Every runner grew its own copy of `_FAIL` + `_check` -- six of them, two
    of which had drifted into different line wrapping.  One harness, imported,
    is the whole rule.  A gate is one line of output and one boolean:

        [PASS] the damper is inside the bound at the MEASURED loop delay  (23%)

    and a suite ends with `self-test PASS` or `self-test FAIL: <names>`.

WHAT IS HERE
    harness    check / raises / report / failures
    fixtures   C_from_rp  -- the one rotation helper more than one gate needs
    budget     SLOT_BUDGET_S -- what a per-sweep block is allowed to cost

    Nothing in here talks to CAN, to the IMU, or to MuJoCo, and it imports no
    module from this repository.  That is deliberate: the harness must be
    importable from a module whose own import is the thing under test.

WHO USES IT
    selftest/test_layout.py           selftest/test_dog5_statics.py
    torque_stand/stand_torque_mode.py --self-test
    dog5_trot_quasi_static_model/trot_hw.py --self-test
    tools/web_telemetry.py --self-test

    `selftest/test_all.py` runs those and every other offline gate in the
    tree, each in its own process.

    Three modules deliberately do NOT use it -- `gait.py`, `trot_demo.py` and
    `walk_demo.py` carry a four-line `check` of their own, because they are
    inside the package that is kept off `sys.path` and a bare
    `import selftest_common` is exactly the shadowing hazard `dog5_paths`
    exists to prevent.
"""
from __future__ import annotations

import math

import numpy as np

#: The CAN slot a per-sweep block must fit inside.  250 Hz across 12 motors is
#: a 333 us slot; a block that overruns delays the FIRST motor of the sweep,
#: which is FL -- part of the set with the shortest driver watchdog.  Gates
#: that time a block assert against this, not against the 4 ms sweep.
SLOT_BUDGET_S = 250e-6


# ===========================================================================
# harness
# ===========================================================================

_FAIL = []


def check(name, ok, detail=""):
    """Report one gate.  Returns the verdict so callers can chain on it."""
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}"
          + (f"  ({detail})" if detail else ""))
    if not ok:
        _FAIL.append(name)
    return bool(ok)


def raises(fn):
    """True if `fn()` throws -- for gates that assert an input is refused."""
    try:
        fn()
    except Exception:
        return True
    return False


def report():
    """Print the verdict line, clear the failures, return the exit code.

    Clearing matters: `test_all.py` may run several suites in one process,
    and a suite must not inherit the previous one's failures.
    """
    print("self-test " + ("FAIL: " + ", ".join(_FAIL) if _FAIL else "PASS"))
    rc = 1 if _FAIL else 0
    _FAIL.clear()
    return rc


def failures():
    """The gates that have failed so far (live view, for test_all)."""
    return list(_FAIL)


# ===========================================================================
# fixtures
# ===========================================================================

def C_from_rp(roll, pitch):
    """An I->B rotation whose _rp() is exactly (roll, pitch): Rodrigues
    rotation taking e_z to the gravity direction implied by (roll, pitch)."""
    g = np.array([-math.sin(pitch),
                  math.cos(pitch) * math.sin(roll),
                  math.cos(pitch) * math.cos(roll)])
    e = np.array([0.0, 0.0, 1.0])
    v = np.cross(e, g)
    c = float(np.dot(e, g))
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1.0 + c)
