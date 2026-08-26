#!/usr/bin/env python3
"""walk_demo.py -- trot_demo, plus the open-loop walk: out on T, back on R.

WHAT THIS IS, AND WHAT IT ADDS
    trot_demo UNCHANGED -- the settle-trot clock (a fixed settle every
    DEMO_SETTLE_EVERY cycles), the alternating lead, DEMO_RAIBERT_ON's
    placement -- plus exactly ONE thing:
    every swing's landing point is displaced WALK_STEP_M in x.  +x from T
    until the R key; R walks the SAME NUMBER of swings back, -x each (a
    per-leg ledger, trot_hw.WalkSequence -- swings, not seconds, so the
    return is distance-equal whatever the settles did to the clock), and
    then the runner latches the same orderly exit the T key does and
    returns to HOLD by itself; P parks from there as always.

    The mechanism, stated once (trot_hw's header and config.py's WALK block
    carry the full argument): the landing point is the ONLY horizontal
    channel this stack has -- kd_xy is 0 and the QP is asked for zero net
    Fx/Fy -- and the body FOLLOWS the feet, because q_ref pins every stance
    foot's x/y at its crouch value and the joint impedance drags the trunk
    over a foot that landed ahead.  Walking is the same swing arc with its
    endpoint moved, nothing else: no new force law, no new gait clock.

HOW IT IS BUILT (three runners, one loop, and the wrap order matters)
    trot_hw.run() is the loop and is used unchanged.  Its walk path is gated
    on args.walk_step, which trot_hw.main() PINS to None -- so trot_hw and
    trot_demo trot in place whatever config.py's WALK block says, and this
    file is the only place walking can come from.  main() here wraps
    trot_hw.run with a shim that sets args.walk_step from WALK_ON /
    WALK_STEP_M, THEN defers to trot_demo.main(): trot_demo captures the
    shim as the run it defers to, so its own injection (the settle clock,
    DEMO_GAIT_PERIOD, DEMO_RAIBERT_ON) lands first and this file inherits
    it all without touching a line of it.  Same keys plus R, same stages,
    same npz logging -- walk_step_m rides beside raibert_kv, and the
    cfg_WALK_* snapshot says what flew.

WHAT TO EXPECT ON THE FLOOR
    Speed is a consequence, not a command: ~WALK_STEP_M per wall-clock cycle
    (settles included).  At the defaults the settle is 0.2 s once every 2
    cycles -- 0.1 s per cycle on average -- so 0.020 m over (1.2 + 0.1) s is
    ~15 mm/s: two clean steps, one brief pause, deliberately.  The step
    SHARES the reach room with the Raibert drift correction (both are
    clamped together in raibert_step_body): at the 0.140 crouch there are
    ~23.6 mm of room, so 20 mm of walk leaves ~3.6 mm of correction --
    about the ceiling for this height.  To walk bigger steps, crouch
    further before growing WALK_STEP_M (at 0.152 the room is only 10.2 mm).

RUN -- trot_demo's keys plus R (ENTER, T from HOLD, R walks back, SPACE, X)
    cd <repo>/src

    python3 dog5_trot_quasi_static_model/walk_demo.py --self-test   # offline, the wiring only
    sudo python3 dog5_trot_quasi_static_model/walk_demo.py          # hardware

    Parameters are edited in dog5_trot_quasi_static_model/config.py (the
    WALK block: WALK_ON, WALK_STEP_M) and land in every npz via
    cfg.snapshot().  The sequence itself -- step passthrough, the shared
    clamp, the WalkSequence ledger R flips and the done that parks it -- is
    proved in trot_hw's own self-test, which owns the loop; this file's
    self-test owns only what this file adds: the wiring, and the pin that
    keeps the other two runners walk-free.
"""
from __future__ import annotations

import os, sys                                                  # noqa: E401
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dog5_paths  # noqa: E402,F401  -- every src/ dir onto sys.path

import inspect
import os
import sys
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_AUG = os.path.dirname(_HERE)

# Imported as package modules so every module identity is shared with
# trot_demo and trot_hw -- the same reason trot_demo imports trot_hw this way.
from dog5_trot_quasi_static_model import trot_hw            # noqa: E402
from dog5_trot_quasi_static_model import trot_demo          # noqa: E402
from dog5_trot_quasi_static_model import config as cfg      # noqa: E402


def walk_run_factory(true_run):
    """Wrap a run() so it walks: set args.walk_step from the WALK block,
    then defer.

    A FACTORY rather than a closure over trot_hw.run directly, so the
    self-test can hand it a stub and check the wiring without a robot.
    WALK_ON False passes None through -- the exact value trot_hw.main()
    pinned -- so the A/B halves differ by nothing but the config bit.
    """
    def _walk_run(args):
        args.walk_step = cfg.WALK_STEP_M if cfg.WALK_ON else None
        if args.walk_step is None:
            print("[walk] WALK_ON is False: this run trots in place, the "
                  "in-place half of the walk A/B")
        else:
            print(f"[walk] open-loop walk armed: {cfg.WALK_STEP_M*1e3:.0f} "
                  f"mm/swing +x after T; R walks the same swings back, "
                  f"then HOLD by itself (edit the WALK block in "
                  f"dog5_trot_quasi_static_model/config.py)")
        return true_run(args)
    return _walk_run


# ===========================================================================
# self-test -- the wiring only; trot_hw --self-test owns the walk itself
# ===========================================================================
_PASS = [0, 0]


def check(label, ok, detail=""):
    _PASS[1] += 1
    _PASS[0] += bool(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
          + (f"  ({detail})" if detail else ""))


def self_test():
    # -- the shim sets exactly the walk args and defers --------------------
    seen = []

    def stub(args):
        seen.append(args)
        return "sentinel"

    ns = types.SimpleNamespace(walk_step=None)
    saved = cfg.WALK_ON
    try:
        cfg.WALK_ON = True
        out = walk_run_factory(stub)(ns)
        check("the shim arms the walk from the config block and defers",
              out == "sentinel" and seen and seen[-1] is ns
              and ns.walk_step == cfg.WALK_STEP_M,
              f"walk_step {ns.walk_step} = WALK_STEP_M, wrapped run "
              f"called once")
        cfg.WALK_ON = False
        walk_run_factory(stub)(ns)
        check("WALK_ON False passes None through -- the in-place A/B half",
              ns.walk_step is None,
              "identical to the pin trot_hw.main() applies")
    finally:
        cfg.WALK_ON = saved

    # -- the other two runners CANNOT walk ---------------------------------
    # The claim the header makes for trot_demo staying untouched rests on
    # trot_hw.main() pinning args.walk_step to None; read the pin off the
    # source so removing it fails HERE, in the runner that depends on it,
    # not silently in a hardware run of the demo.
    check("trot_hw.main() pins the walk path off, so trot_demo cannot walk",
          "args.walk_step = None" in inspect.getsource(trot_hw.main),
          "walking must reach the loop only through this file's shim")
    check("...and run()'s walk machinery is gated on that same attribute",
          "args.walk_step is not None" in inspect.getsource(trot_hw.run),
          "the gate the pin and the shim both talk to")

    # -- the pieces this file composes are the ones already proved ---------
    ws = trot_hw.WalkSequence(cfg.WALK_STEP_M)
    dx_out = ws.step_for(0)
    ws.reverse()
    dx_back = ws.step_for(0)
    check("the walk this shim arms is trot_hw's own WalkSequence ledger",
          dx_out == cfg.WALK_STEP_M and dx_back == -cfg.WALK_STEP_M
          and ws.done,
          "+step out, R gives the same swing back, then done; the full "
          "ledger is trot_hw's self-test")
    check("the settle clock this file inherits is trot_demo's, untouched",
          trot_demo.SettleTrotGait is not None
          and inspect.getmodule(trot_demo.SettleTrotGait) is trot_demo,
          "this file defines no gait of its own")

    print(f"self-test {'PASS' if _PASS[0] == _PASS[1] else 'FAIL'} "
          f"({_PASS[0]}/{_PASS[1]})")
    return 0 if _PASS[0] == _PASS[1] else 1


# ===========================================================================
# main -- arm the walk, then defer to trot_demo, which defers to trot_hw
# ===========================================================================
def main():
    if "--self-test" in sys.argv:
        return self_test()

    # WRAP FIRST, DEFER SECOND: trot_demo.main() captures trot_hw.run as the
    # run it defers to, so rebinding it here means the chain is
    # _demo_run (settle clock, demo args) -> _walk_run (walk args) -> the
    # verified loop.  Both wrappers only ADD to args; neither edits the
    # other's.
    trot_hw.run = walk_run_factory(trot_hw.run)
    return trot_demo.main()


if __name__ == "__main__":
    sys.exit(main())
