#!/usr/bin/env python3
"""Gates on the repository layout itself, not on any control law.

These exist because the reorganisation that produced this repository merged two
nested git repositories into one tree, and the failure modes it had to design
around are silent ones: a shadowed module that dies three imports later with an
unrelated AttributeError, a parameter file that quietly stops being safe to read
from a notebook, a path that only resolves on the machine it was written on.

Every check here is cheap and has caught a real regression at least once.

Run:  python3 test_layout.py
"""
from __future__ import annotations

import os, sys                                                  # noqa: E401
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dog5_paths  # noqa: E402,F401  -- every src/ dir onto sys.path

import ast                                                     # noqa: E402
import pathlib                                                 # noqa: E402

from selftest_common import check, report                      # noqa: E402

SRC = pathlib.Path(dog5_paths.SRC)

#: Parameter files whose whole value is that they can be read from anywhere --
#: a test, a notebook, a plotting script -- without dragging in numpy, CAN, or
#: an IMU.  The moment one of them grows an import, that stops being true.
IMPORT_FREE = ["imu_closedloop_stand/stand_params.py",
               "torque_primitives/torque_params.py"]


def test_no_config_shadowing():
    """`config` must resolve to exactly one module: the trot package's.

    Two directories in this tree contain a config.py.  motor/motorbus.py wants
    the motor unit-gain table (now named motor_gains.py precisely so it cannot
    collide) and dog5_trot_quasi_static_model/config.py is the trot constants.
    When the trot package directory lands on sys.path, a bare `import config`
    picks up whichever came first, and the CAN layer dies far away with
    `module 'config' has no attribute 'encoder_gain'`.
    """
    import motor_gains
    import dog5_trot_quasi_static_model.config as trot_cfg

    check("motor_gains and the trot config are distinct modules",
          motor_gains is not trot_cfg,
          f"{getattr(motor_gains, '__file__', '?')} vs "
          f"{getattr(trot_cfg, '__file__', '?')}")
    check("the motor gain table is the one motorbus reads",
          hasattr(motor_gains, "encoder_gain"),
          "encoder_gain present" if hasattr(motor_gains, "encoder_gain")
          else "MISSING encoder_gain")
    check("package directories are kept OFF sys.path",
          all(str(SRC / p) not in sys.path for p in dog5_paths.PACKAGES),
          ", ".join(sorted(dog5_paths.PACKAGES)))
    check("no bare `config` module is importable",
          "config" not in sys.modules or
          sys.modules["config"] is trot_cfg,
          sys.modules.get("config", "not imported"))


def test_param_files_import_nothing():
    """The contract that makes the parameter tables readable from anywhere."""
    for rel in IMPORT_FREE:
        p = SRC / rel
        bad = [n.lineno for n in ast.parse(p.read_text(encoding="utf-8")).body
               if isinstance(n, (ast.Import, ast.ImportFrom))]
        check(f"{rel} imports nothing", not bad,
              f"imports at lines {bad}" if bad else "safe to read from anywhere")


def test_layout_is_flat():
    """Every source directory sits exactly one level below src/.

    This is what lets the bootstrap header be byte-identical in every file, and
    what makes `import motorbus` resolve from any script in the tree.
    """
    offenders = []
    for d in sorted(SRC.iterdir()):
        if not d.is_dir() or d.name == "__pycache__":
            continue
        for sub in d.iterdir():
            if sub.is_dir() and sub.name not in ("__pycache__", "meshes"):
                offenders.append(f"{d.name}/{sub.name}")
    check("no nested source directories under src/", not offenders,
          "; ".join(offenders) or "flat")


def test_no_absolute_or_stale_paths():
    """No path that only resolves on one machine, and no pre-reorg directory."""
    stale = ["stand_postion_mode", "torque_mode_control", "august_week2",
             "Aug_trotinplace_closedloop", "dog_stand_compliance_control",
             "/home/robot01", "D:\\mujoco", "catersian"]
    hits = []
    for p in SRC.rglob("*.py"):
        if "__pycache__" in p.parts or p.name == "test_layout.py":
            continue          # this file necessarily spells the tokens out
        text = p.read_text(encoding="utf-8")
        for token in stale:
            if token in text:
                hits.append(f"{p.relative_to(SRC)}:{token}")
    check("no stale or machine-specific paths anywhere in src/", not hits,
          "; ".join(hits[:6]) or f"{len(list(SRC.rglob('*.py')))} files scanned")


def test_bootstrap_header_is_uniform():
    """Every file that bootstraps does it the same way, or not at all.

    A file may legitimately have no header (a leaf module, or one of the
    import-free parameter tables).  What it may not do is roll its own
    sys.path arithmetic, because that is what produced sixty-nine different
    preambles in the two repositories this tree replaces.
    """
    bad = []
    for p in SRC.rglob("*.py"):
        if "__pycache__" in p.parts or p.name == "dog5_paths.py":
            continue
        for i, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            s = line.strip()
            if not s.startswith("sys.path.insert") and not s.startswith("sys.path.append"):
                continue
            if s == ("sys.path.insert(0, os.path.dirname("
                     "os.path.dirname(os.path.abspath(__file__))))"):
                continue                      # the sanctioned header line
            bad.append(f"{p.relative_to(SRC)}:{i}")
    check("sys.path is only touched by the uniform header or dog5_paths",
          not bad, "; ".join(bad[:8]) or "uniform")


def self_test():
    print("repository layout self-test (no hardware)")
    test_no_config_shadowing()
    test_param_files_import_nothing()
    test_layout_is_flat()
    test_no_absolute_or_stale_paths()
    test_bootstrap_header_is_uniform()
    return report()


if __name__ == "__main__":
    sys.exit(self_test())
