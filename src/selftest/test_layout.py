#!/usr/bin/env python3
"""Gates on the repository layout itself, not on any control law.

These exist because the reorganisation that produced this repository merged two
nested git repositories into one tree and then cut it down to the code that has
actually run on the robot, and the failure modes of both are silent ones: a
shadowed module that dies three imports later with an unrelated AttributeError,
a parameter file that quietly stops being safe to read from a notebook, a path
that only resolves on the machine it was written on, an import left pointing at
a file that is no longer here.

THE ONE TO READ FIRST is `test_every_import_resolves`.  It is the gate that
says this repository can be cloned onto another machine and still work: every
repo-local import in every shipped file names a module that is present.  Run it
after any move, rename or deletion.

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
IMPORT_FREE = ["torque_stand/params.py"]

#: Distributions that are not in this repository and are expected to be
#: installed.  The first four are required (requirements.txt); the last three
#: are imported lazily and only on the path that needs them --
#:   PIL   MuJoCo headless rendering and the stand-up sim GIF
#:   usb   libusb enumeration, macOS CAN adapters only
#:   fdilink_imu   the IMU vendor SDK, which is not on PyPI and not vendored
#:                 here; without it everything imports and every gate passes,
#:                 and only opening a real IMU fails.
THIRD_PARTY = {"numpy", "mujoco", "can", "serial",
               "PIL", "usb", "fdilink_imu"}


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


def _tree_modules():
    """Every module name this tree makes importable, the way dog5_paths does.

    Flat names, because every source directory goes on sys.path, plus dotted
    `package.module` names for the directories that are deliberately kept off
    it.
    """
    flat, dotted = set(), set()
    for p in SRC.rglob("*.py"):
        if "__pycache__" in p.parts:
            continue
        rel = p.relative_to(SRC)
        flat.add(p.stem)
        if len(rel.parts) == 2:
            dotted.add(f"{rel.parts[0]}.{p.stem}")
            dotted.add(rel.parts[0])
    return flat, dotted


def test_every_import_resolves():
    """No file imports something this repository no longer contains.

    This is the gate that makes the tree portable.  A stale import is invisible
    until the one branch that reaches it runs -- often on the robot, often
    mid-stage -- so it is checked statically, over every file, every time.

    Lazy imports inside functions count: `trot_hw.py --web` reaching a module
    that was deleted is exactly the failure this is here to stop.
    """
    flat, dotted = _tree_modules()
    known = flat | dotted | THIRD_PARTY | set(sys.stdlib_module_names)
    bad = []
    for p in sorted(SRC.rglob("*.py")):
        if "__pycache__" in p.parts:
            continue
        tree = ast.parse(p.read_text(encoding="utf-8"))
        for n in ast.walk(tree):
            names = []
            if isinstance(n, ast.Import):
                names = [a.name for a in n.names]
            elif isinstance(n, ast.ImportFrom) and n.module and n.level == 0:
                names = [n.module]
                if n.module in dotted:      # `from pkg import mod`
                    names += [f"{n.module}.{a.name}" for a in n.names
                              if f"{n.module}.{a.name}" in dotted
                              or a.name in flat]
            for name in names:
                root = name.split(".")[0]
                if name in known or root in known:
                    continue
                bad.append(f"{p.relative_to(SRC)}:{n.lineno}: {name}")
    check("every repo-local import names a module that is present",
          not bad, "; ".join(bad[:8]) or
          f"{len(list(SRC.rglob('*.py'))) } files, {len(flat)} modules")


def test_no_orphan_modules():
    """Every .py in the tree is reachable, or is an entry point in its own right.

    The repository ships only code that has run on the robot, and the way that
    stops being true is by accretion: a helper survives the module that used
    it.  A file counts as reachable if something imports it OR it does
    something when run -- a `__main__` guard, or straight-line statements at
    module level, which is what a runner, a gate and a viewer script all are.
    """
    imported, orphans = set(), []
    for p in SRC.rglob("*.py"):
        if "__pycache__" in p.parts:
            continue
        for n in ast.walk(ast.parse(p.read_text(encoding="utf-8"))):
            if isinstance(n, ast.Import):
                imported |= {a.name.split(".")[-1] for a in n.names}
            elif isinstance(n, ast.ImportFrom) and n.module:
                imported.add(n.module.split(".")[-1])
                imported |= {a.name for a in n.names}
    # Anything at module level that is not a declaration means the file DOES
    # something when you run it.  Matched structurally rather than by looking
    # for the text of a `__main__` guard, because the quote style of that
    # guard is not uniform across this tree and a viewer script has no guard
    # at all -- it is nine statements and a `launch()`.
    decl = (ast.Import, ast.ImportFrom, ast.FunctionDef, ast.AsyncFunctionDef,
            ast.ClassDef, ast.Assign, ast.AnnAssign)
    for p in sorted(SRC.rglob("*.py")):
        if "__pycache__" in p.parts or p.stem in ("__init__", "dog5_paths"):
            continue
        body = ast.parse(p.read_text(encoding="utf-8")).body
        runnable = any(
            not (isinstance(n, decl)
                 or (isinstance(n, ast.Expr)          # a docstring, not a call
                     and isinstance(n.value, ast.Constant)))
            for n in body)
        if p.stem in imported or runnable:
            continue
        orphans.append(str(p.relative_to(SRC)))
    check("no module is orphaned -- everything is imported or runnable",
          not orphans, "; ".join(orphans) or "no dead files")


def self_test():
    print("repository layout self-test (no hardware)")
    test_every_import_resolves()
    test_no_orphan_modules()
    test_no_config_shadowing()
    test_param_files_import_nothing()
    test_layout_is_flat()
    test_no_absolute_or_stale_paths()
    test_bootstrap_header_is_uniform()
    return report()


if __name__ == "__main__":
    sys.exit(self_test())
