#!/usr/bin/env python3
"""Run every gate in this SDK.  About a minute, no robot, no CAN adapter.

    python tests/test_all.py

Three suites:

    test_offline.py        the model, the kinematics, the statics, the safety
                           gate and a controller standing up in simulation
    test_hardware_path.py  the whole CAN path against twelve simulated drivers
    test_sync.py           whether the files copied from the research
                           repository have drifted (skips if it is not there)

Run this after any change, and once on the machine you are handed the SDK on:
it is the answer to "did the whole thing arrive intact".
"""
from __future__ import annotations

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SUITES = ("test_offline.py", "test_hardware_path.py", "test_sync.py")


def main() -> int:
    failed = []
    for suite in SUITES:
        print(f"\n{'=' * 70}\n{suite}\n{'=' * 70}", flush=True)
        result = subprocess.run([sys.executable, "-u",
                                 os.path.join(HERE, suite)])
        if result.returncode != 0:
            failed.append(suite)

    print(f"\n{'=' * 70}")
    if failed:
        print(f"FAILED: {', '.join(failed)}")
        return 1
    print(f"all {len(SUITES)} suites passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
