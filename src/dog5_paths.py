"""Put every source directory of this repository on ``sys.path``, exactly once.

This is the ONLY ``sys.path`` logic in the repository.  It replaces the ~69
hand-rolled ``_HERE`` / ``_AUG`` / ``_ROOT`` / ``_REPO`` preambles the code
carried while it lived in two nested git repositories, each of which had to
guess its way back up to the other.

Layout contract
---------------
Every source directory is EXACTLY one level below ``src/``::

    src/motor/            src/dog5_description/   src/robot_base/
    src/IMU_sensor/       src/state_estimator/    src/torque_stand/   ...

so a bare ``import motorbus`` / ``import dog5_kinematics`` resolves from any
script in the tree, and the three-line header that calls this module is
byte-identical in every file that needs it.

Packages are deliberately NOT added
-----------------------------------
``dog5_trot_quasi_static_model/`` and ``srb_mpc_hw/`` stay importable only as
packages.  Both contain a ``config.py``, and ``motor/motorbus.py`` imports the
motor unit-gain table.  If those directories went on ``sys.path`` the wrong
``config`` would win and the CAN layer would die with a bare
``module 'config' has no attribute 'encoder_gain'``.  Keeping them off the path
makes that collision structurally impossible instead of merely commented
against.  ``src/selftest/test_shadowing.py`` is the standing regression gate.

The empty/``.``/script-directory entries are also stripped, for the same reason:
running ``python src/dog5_trot_quasi_static_model/trot_hw.py`` would otherwise
put that package directory itself at the front of ``sys.path``.
"""

from __future__ import annotations

import os
import sys

SRC = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(SRC)

#: Directories that must be reached as packages, never as path entries.
PACKAGES = frozenset({"dog5_trot_quasi_static_model", "srb_mpc_hw"})

_INSTALLED = False


def install() -> list:
    """Add every non-package directory under ``src/`` to ``sys.path``.

    Idempotent: safe to call from every module in the tree.  Returns the list
    of directories that are on the path as a result.
    """
    global _INSTALLED
    added = []
    for name in sorted(os.listdir(SRC)):
        path = os.path.join(SRC, name)
        if not os.path.isdir(path) or name in PACKAGES:
            continue
        if name.startswith(".") or name == "__pycache__":
            continue
        if path not in sys.path:
            sys.path.insert(0, path)
        added.append(path)

    # src/ itself, so `import dog5_trot_quasi_static_model` and `import srb_mpc_hw`
    # resolve as packages.
    if SRC not in sys.path:
        sys.path.insert(0, SRC)

    # A script's own directory (or "" / ".") at the front of sys.path is how the
    # config.py collision used to happen.  Drop those entries.
    for dup in ("", ".", os.getcwd()):
        while dup in sys.path:
            sys.path.remove(dup)
    for pkg in PACKAGES:
        pkg_dir = os.path.join(SRC, pkg)
        while pkg_dir in sys.path:
            sys.path.remove(pkg_dir)

    _INSTALLED = True
    return added


def add_fdilink_root():
    """Find the ``fdilink_imu`` package WITHOUT trusting ``$HOME``.

    ``imu_dog`` resolves the vendor SDK as ``Path.home()/Documents/IMU_sensor``,
    which breaks under ``sudo`` because ``$HOME`` becomes ``/root``.  Real-time
    priority wants root, so resolve repo-relative first and fall back to the
    invoking user's home.

    ``fdilink_imu`` is a vendor SDK and is NOT vendored in this repository; it
    is not pip-installable either.  Install it on the robot host at
    ``~/Documents/IMU_sensor/fdilink_imu``.  Returns the directory that was
    added, or ``None`` when the SDK is not present (which is the normal case on
    a development machine with no robot attached).
    """
    cands = [os.path.join(REPO, "IMU_sensor"),
             os.path.join(os.path.dirname(REPO), "IMU_sensor"),
             os.path.join(os.path.expanduser("~"), "Documents", "IMU_sensor")]
    if os.environ.get("SUDO_USER"):
        cands.append(os.path.join("/home", os.environ["SUDO_USER"],
                                  "Documents", "IMU_sensor"))
    for root in cands:
        if os.path.isdir(os.path.join(root, "fdilink_imu")):
            if root not in sys.path:
                sys.path.insert(0, root)
            return root
    return None


def data(*parts) -> str:
    """Absolute path into the (gitignored) ``data/`` directory.

    Run logs are not tracked in this repository; see ``data/README.md`` for
    which files to drop in and where they come from.
    """
    return os.path.join(REPO, "data", *parts)


install()
add_fdilink_root()
