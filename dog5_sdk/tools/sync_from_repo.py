#!/usr/bin/env python3
"""Regenerate every file this SDK COPIES from the dog5 research repository.

The SDK is deliberately self-contained: a classmate gets one directory, runs
``pip install -e .``, and has the whole robot -- the CAN library, the
kinematics, the calibrated joint contract, the MJCF and the meshes -- with no
reference back to the research tree.  Self-contained means duplicated, and
duplicated means it can drift.  This script is the answer to that.

Two kinds of copy:

VERBATIM   byte-for-byte.  ``motor/motorbus.py`` is the file that has driven
           twelve motors at 250 Hz on the real machine; there is no version of
           "improving" it here that is not a risk.  Copied, not edited.

ADAPTED    the file's own header is replaced, and NOTHING else.  Three modules
           in the research tree open with a ``dog5_paths`` preamble that puts
           every ``src/`` directory on ``sys.path`` and then import their
           siblings by flat name.  Inside a package that preamble is wrong, so
           the header up to a fixed marker line is swapped for package-relative
           imports.  Everything from the marker down is byte-for-byte.

Usage::

    python tools/sync_from_repo.py            # report drift, change nothing
    python tools/sync_from_repo.py --write    # copy from ../src

``tests/test_sync.py`` runs the check half in CI.  With no research tree beside
the SDK (the classmate's normal case) both simply skip: the copies are the
shipped source of truth, and there is nothing to compare them against.
"""
from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SDK = os.path.dirname(HERE)                       # dog5_sdk/
PKG = os.path.join(SDK, "dog5_sdk")               # dog5_sdk/dog5_sdk/
REPO_SRC = os.path.normpath(os.path.join(SDK, os.pardir, "src"))

# ---------------------------------------------------------------------------
# verbatim copies: {destination relative to the package, source under src/}
# ---------------------------------------------------------------------------
VERBATIM = {
    "motor/motor_library.py":  "motor/motor_library.py",
    "motor/motorbus.py":       "motor/motorbus.py",
    "motor/motor_gains.py":    "motor/motor_gains.py",
    "motor/mac_can.py":        "motor/mac_can.py",
    "kinematics.py":           "dog5_description/dog5_kinematics.py",
    "hardware_map.py":         "dog5_description/dog5_hardware_map.py",
    "params.py":               "torque_stand/params.py",
    "model/dog5.xml":          "dog5_description/dog5.xml",
    "model/meshes/trunk.stl":  "dog5_description/meshes/trunk.stl",
    "model/meshes/hip.stl":    "dog5_description/meshes/hip.stl",
    "model/meshes/thigh.stl":  "dog5_description/meshes/thigh.stl",
    "model/meshes/shin.stl":   "dog5_description/meshes/shin.stl",
}

# ---------------------------------------------------------------------------
# adapted copies: header replaced up to (and excluding) MARKER, body verbatim
# ---------------------------------------------------------------------------
_STATICS_HEADER = '''#!/usr/bin/env python3
"""Mass properties, fused chain walk, leg gravity and the MuJoCo cross-check.

VERBATIM from ``dog5_description/dog5_statics.py`` in the dog5 research
repository, from ``LEGS = kin.LEGS`` down.  Only the import header differs:
the original reaches its siblings through ``dog5_paths``/``sys.path``, which
inside a package would be wrong.  ``tools/sync_from_repo.py`` regenerates this
file and ``tests/test_sync.py`` gates the body against the original.

The numbers here are gated against the MJCF itself by
``verify_against_model()`` -- see ``tests/test_offline.py``.
"""
from __future__ import annotations

import os

import numpy as np

from . import kinematics as kin

#: The MJCF and its meshes, shipped inside this package.
_DESC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model")

'''

_ESTIMATOR_HEADER = '''#!/usr/bin/env python3
"""Sensors -> the trunk state a body controller needs: z, v, C, omega.

VERBATIM from ``torque_stand/feedback_estimator.py`` in the dog5 research
repository, from ``LEGS = st.LEGS`` down; only the import header differs.
``tools/sync_from_repo.py`` regenerates this file.

The pure functions here are what :class:`dog5_sdk.Dog5Sim` and
:class:`dog5_sdk.Dog5Hardware` use to fill :class:`dog5_sdk.RobotState`, so a
controller sees the SAME height, velocity and attitude definitions in
simulation and on the robot.  ``BodyState`` is the full hardware object and
needs a live AHRS.
"""
from __future__ import annotations

import math

import numpy as np

from . import params as P
from . import statics as st

'''

ADAPTED = {
    "statics.py": ("dog5_description/dog5_statics.py",
                   "LEGS = kin.LEGS", _STATICS_HEADER),
    "estimator.py": ("torque_stand/feedback_estimator.py",
                     "LEGS = st.LEGS", _ESTIMATOR_HEADER),
}


def _read(path: str) -> bytes:
    with open(path, "rb") as handle:
        return handle.read()


def _body_after(text: str, marker: str, origin: str) -> str:
    """Everything from the line that starts with `marker`, to the end."""
    lines = text.splitlines(keepends=True)
    for index, line in enumerate(lines):
        if line.startswith(marker):
            return "".join(lines[index:])
    raise SystemExit(f"{origin}: marker {marker!r} not found -- the upstream "
                     "header changed shape; update sync_from_repo.py")


def rendered() -> dict:
    """{destination path -> the exact bytes it should hold}, from ../src."""
    out = {}
    for dst, src in VERBATIM.items():
        out[dst] = _read(os.path.join(REPO_SRC, src))
    for dst, (src, marker, header) in ADAPTED.items():
        origin = os.path.join(REPO_SRC, src)
        body = _body_after(_read(origin).decode("utf-8"), marker, origin)
        out[dst] = (header + body).encode("utf-8")
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--write", action="store_true",
                        help="copy from ../src (default: report only)")
    args = parser.parse_args()

    if not os.path.isdir(REPO_SRC):
        print(f"[sync] no research tree at {REPO_SRC} -- nothing to sync "
              "against.  The files in this SDK are the source of truth.")
        return 0

    drift = []
    for dst, want in rendered().items():
        path = os.path.join(PKG, dst)
        have = _read(path) if os.path.exists(path) else None
        if have == want:
            continue
        drift.append(dst)
        if args.write:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "wb") as handle:
                handle.write(want)
            print(f"[sync] wrote  {dst}")
        else:
            state = "MISSING" if have is None else "DIFFERS"
            print(f"[sync] {state}  {dst}")

    total = len(VERBATIM) + len(ADAPTED)
    if not drift:
        print(f"[sync] {total} copied files match {REPO_SRC}")
        return 0
    if args.write:
        print(f"[sync] {len(drift)}/{total} file(s) updated")
        return 0
    print(f"[sync] {len(drift)}/{total} file(s) out of date -- "
          "rerun with --write", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
