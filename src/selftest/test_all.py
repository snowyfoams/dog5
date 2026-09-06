#!/usr/bin/env python3
"""Run every offline gate in this repository and summarise the verdicts.

    python3 src/selftest/test_all.py            # everything, ~30 s
    python3 src/selftest/test_all.py trot gait  # only suites whose path matches
    python3 src/selftest/test_all.py -q         # summary table only

NO ROBOT, NO CAN INTERFACE, NO DATA.  Every suite below runs on a laptop with
nothing plugged in.  That is the point of them: a control law is not "done"
here until it has gates that can be re-run by someone who has never seen the
machine.

WHERE THE GATES LIVE, AND WHY THEY ARE NOT ALL IN THIS DIRECTORY
    Two kinds of gate, and the split is deliberate.

    `selftest/test_*.py`   gates on something no single module owns -- the
                           repository's own layout, or the statics model
                           checked against the MJCF from four directions.

    `<module> --self-test` gates on ONE module, living in that module, below
                           the code they check.  A reader who opens
                           `stand_torque_mode.py` to find out what the joint
                           impedance is allowed to do finds the assertions in
                           the same file, not two directories away -- and a
                           gain cannot be edited without the gate that bounds
                           it being right there in the diff.

NOT EVERY SUITE COUNTS GATES
    Most print one `[PASS] <claim>` line per assertion and this script tallies
    them.  Five older ones -- the kinematics cross-check, the two position-mode
    runners, the pose monitor and the tau audit -- assert internally and print
    a narrative instead, so they show `--` in the count column.  Their exit
    code is what matters, and it is checked the same way.

EACH SUITE RUNS IN ITS OWN PROCESS
    A suite must not be handed a module that another suite has already poked
    at, so a green run here means each script is also green when run alone,
    which is how they are actually used.
"""
from __future__ import annotations

import os
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(_HERE)

#: (title, path relative to src/, extra argv)
SUITES = [
    ("layout    repo structure and import hygiene",
     "selftest/test_layout.py", []),
    ("model     NumPy FK/Jacobian vs MuJoCo, 1024 poses",
     "dog5_description/check_dog5_kinematics.py", []),
    ("statics   -J^T f + leg gravity, four ways",
     "selftest/test_dog5_statics.py", []),
    ("base      shared CAN/joint hardware layer",
     "robot_base/stand_dog5_hw.py", ["--self-test"]),
    ("crouch    the recorded-pose position runner",
     "robot_base/stand_dog5_recorded_hw.py", ["--self-test"]),
    ("pose      the calibration pose monitor",
     "calibration/dog5_pose_monitor.py", ["--self-test"]),
    ("stand     torque-mode stand: stages, impedance, gate, load",
     "torque_stand/stand_torque_mode.py", ["--self-test"]),
    ("gait      the trot clock and the contact-weight ramp",
     "dog5_trot_quasi_static_model/gait.py", ["--self-test"]),
    ("trot      the trot runner, stand stages included",
     "dog5_trot_quasi_static_model/trot_hw.py", ["--self-test"]),
    ("settle    trot + the periodic four-foot re-level",
     "dog5_trot_quasi_static_model/trot_demo.py", ["--self-test"]),
    ("walk      the out-and-back step ledger",
     "dog5_trot_quasi_static_model/walk_demo.py", ["--self-test"]),
    ("web       the telemetry ring buffer and server",
     "tools/web_telemetry.py", ["--self-test"]),
    ("audit     the foot-load sum from measured iq",
     "tools/tools_tau_audit.py", ["--self-test"]),
]


def run_suite(rel, argv, echo):
    """Run one suite; return (rc, n_pass, n_fail, seconds, output)."""
    t0 = time.perf_counter()
    proc = subprocess.run([sys.executable, os.path.join(_SRC, rel)] + argv,
                          capture_output=True, text=True)
    dt = time.perf_counter() - t0
    out = proc.stdout + proc.stderr
    if echo:
        print(out, end="" if out.endswith("\n") else "\n")
    n_pass = out.count("[PASS]")
    n_fail = out.count("[FAIL]")
    return proc.returncode, n_pass, n_fail, dt, out


def main():
    wanted = [a for a in sys.argv[1:] if not a.startswith("-")]
    echo = "-q" not in sys.argv[1:]
    suites = [s for s in SUITES
              if not wanted or any(w in s[1] for w in wanted)]
    if not suites:
        print(f"no suite matches {wanted}; known:\n  "
              + "\n  ".join(s[1] for s in SUITES))
        return 2

    rows, rc_total = [], 0
    for title, rel, argv in suites:
        print(f"\n{'='*78}\n=== {title}  ({rel})\n{'='*78}")
        rc, n_pass, n_fail, dt, out = run_suite(rel, argv, echo)
        rc_total |= rc
        rows.append((rel, rc, n_pass, n_fail, dt))
        if rc and not echo:
            print(out, end="")

    print(f"\n{'='*78}\nSUMMARY\n{'='*78}")
    total_pass = total_fail = 0
    for rel, rc, n_pass, n_fail, dt in rows:
        total_pass += n_pass
        total_fail += n_fail
        count = f"{n_pass:3d} gates" if n_pass else "  -- gates"
        print(f"  [{'PASS' if rc == 0 else 'FAIL'}] {rel:52s} "
              f"{count}  {dt:5.1f}s"
              + (f"  {n_fail} FAILED" if n_fail else ""))
    counted = sum(1 for _, _, n, _, _ in rows if n)
    print(f"  {total_pass} counted gates across {counted} of {len(rows)} "
          f"suites, {total_fail} failed; the other "
          f"{len(rows) - counted} assert internally")
    return 1 if rc_total else 0


if __name__ == "__main__":
    sys.exit(main())
