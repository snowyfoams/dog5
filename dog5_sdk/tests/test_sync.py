#!/usr/bin/env python3
"""Have the copied files drifted from the research repository?

    python tests/test_sync.py

This SDK is self-contained on purpose: a classmate gets one directory and needs
nothing else.  Self-contained means the CAN library, the kinematics, the MJCF
and the meshes are DUPLICATED from the tree they were developed in, and
duplication drifts.  ``tools/sync_from_repo.py`` regenerates them; this gate
says whether they are current.

With no research tree beside the SDK -- which is the normal case once it has
been handed over -- there is nothing to compare against and the test SKIPS.
The shipped copies are then the source of truth, which is exactly what a
handover means.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

import sync_from_repo                                        # noqa: E402


def test_copied_files_match_the_research_repo():
    if not os.path.isdir(sync_from_repo.REPO_SRC):
        print(f"skip: no research tree at {sync_from_repo.REPO_SRC}")
        return

    drift = []
    for destination, want in sync_from_repo.rendered().items():
        path = os.path.join(sync_from_repo.PKG, destination)
        if not os.path.exists(path):
            drift.append(f"{destination}: MISSING")
            continue
        with open(path, "rb") as handle:
            if handle.read() != want:
                drift.append(f"{destination}: DIFFERS")
    assert not drift, (
        "copied files are out of date with ../src:\n  "
        + "\n  ".join(drift)
        + "\nrun: python tools/sync_from_repo.py --write")


def test_every_verbatim_file_really_is_verbatim():
    """The verbatim set must be byte-identical, not merely similar.

    Stated as its own gate because it is the load-bearing claim: the CAN layer
    in this SDK is the code that ran on the robot, not a port of it.
    """
    if not os.path.isdir(sync_from_repo.REPO_SRC):
        print(f"skip: no research tree at {sync_from_repo.REPO_SRC}")
        return

    for destination, source in sync_from_repo.VERBATIM.items():
        with open(os.path.join(sync_from_repo.PKG, destination), "rb") as ours:
            with open(os.path.join(sync_from_repo.REPO_SRC, source),
                      "rb") as theirs:
                assert ours.read() == theirs.read(), (
                    f"{destination} is not byte-identical to src/{source}")


def main() -> int:
    failed = []
    for name, test in sorted(globals().items()):
        if not name.startswith("test_") or not callable(test):
            continue
        try:
            test()
        except Exception as exc:                             # noqa: BLE001
            failed.append(name)
            print(f"FAIL  {name}\n        {exc}")
        else:
            print(f"ok    {name}")
    if failed:
        print(f"\n{len(failed)} FAILED")
        return 1
    print("\nsync gates passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
