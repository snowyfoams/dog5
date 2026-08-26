#!/usr/bin/env python3
"""trot_demo.py -- trot_hw, plus a FIXED four-foot SETTLE every N cycles.

WHAT THIS IS, AND WHAT IT IS NOT
    The roll problem is not solved; this file sidesteps it for a demo.  The
    schedule is trot_hw's, UNCHANGED -- same period, same duty, same swing --
    but once every DEMO_SETTLE_EVERY full gait cycles, right after that
    cycle's second touchdown has ramped to full support, the gait clock
    FREEZES.  While frozen all four contact weights are 1, so the balance QP
    re-distributes force over four feet and the attitude spring pulls the
    roll back to the setpoint -- a standing re-level -- before the next
    cycle is released.

    THE SETTLE IS FIXED AND PERIODIC.  It used to grow cycle by cycle
    (DEMO_SETTLE_S + n * grow); that accumulation is gone since 2026-08-25:
    the robot now trots DEMO_SETTLE_EVERY full cycles, re-levels for
    DEMO_SETTLE_S, and repeats -- at the defaults, two cycles then 0.2 s.
    Less settle per cycle means walk_demo covers ground faster; the price is
    re-level time, which is what the settle exists to provide.

    THE LEAD ALTERNATES (DEMO_ALTERNATE_LEAD): every cycle the two diagonals
    swap phase, so the diagonal that swung FIRST this cycle swings SECOND
    the next.  The robot walks out of place, and any drift component that
    depends on the swing ORDER keeps its sign every cycle and integrates;
    alternating the order makes it flip sign and cancel over pairs of
    cycles.  The flip is placed at the freeze point (mid-settle when that
    cycle has one) because that is the one moment both diagonals are
    mid-stance at full weight -- phases 0.15 and 0.65 simply swap sides, so
    no contact state, no weight and no swing phase changes at the flip
    instant.  Order-INDEPENDENT drift (every foot landing displaced the
    same way) is untouched by this; if the robot still walks with it on,
    the cause is elsewhere.

HOW IT IS BUILT (build on the verified runner, do not rewrite it)
    trot_hw.run() is used UNCHANGED.  Everything the loop asks of the gait --
    contact(), contact_weight(), swing_phase() -- derives from phase(t), so
    SettleTrotGait subclasses TrotGait and overrides ONLY phase(): a
    piecewise-linear warp maps wall time onto gait time, flat (frozen) for
    DEMO_SETTLE_S once every DEMO_SETTLE_EVERY cycles.  Outside the flats
    the warp is 1:1, so swing durations, the contact ramp and the stance
    math are exactly trot_hw's.  main() injects the subclass, then defers
    to trot_hw.main() -- same keys, same stages, same npz logging
    (cfg_DEMO_* ride in the snapshot).

WHERE THE FREEZE SITS, AND WHY IT MUST
    With offsets {0, 0.5} and duty > 0.5, the stretch of elapsed cycle time
    [0, duty-0.5) is all-stance: the second diagonal's swing ends exactly at
    the cycle wrap, so this window IS "after the full cycle".  The freeze
    point is its MIDPOINT, h = (duty-0.5)/2 in cycles, because the freeze is
    only a standing re-level if every weight is already ramped to a full 1.0
    there -- frozen inside a ramp, one diagonal would hold less than full
    authority for the whole settle.  The constructor MEASURES that
    (contact_weight == 1 for all four at the freeze point) and refuses the
    schedule otherwise, so a CONTACT_RAMP/DUTY retune cannot silently turn
    the settle into a lean.

RUN -- same procedure and keys as trot_hw (ENTER, T from HOLD, SPACE, X)
    cd <repo>/src

    python3 dog5_trot_quasi_static_model/trot_demo.py --self-test    # offline, the warp only
    sudo python3 dog5_trot_quasi_static_model/trot_demo.py           # hardware

    Parameters are edited in dog5_trot_quasi_static_model/config.py
    (DEMO_GAIT_PERIOD, DEMO_SETTLE_S, DEMO_SETTLE_EVERY,
    DEMO_ALTERNATE_LEAD, DEMO_RAIBERT_ON) and land in every npz via
    cfg.snapshot().  DEMO_RAIBERT_ON gates trot_hw's existing Raibert
    placement path for THIS runner only -- feet land downstream of the
    measured drift instead of on the spot they lifted from, which is the
    answer to the order-independent part of walking out of place.
"""
from __future__ import annotations

import os, sys                                                  # noqa: E401
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dog5_paths  # noqa: E402,F401  -- every src/ dir onto sys.path

import os
import sys
import types

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_AUG = os.path.dirname(_HERE)

# trot_hw's own import block builds the full sys.path (week2, torque_mode,
# repo root) and deliberately strips _HERE -- see its comment.  Importing it
# as the package module keeps every module identity shared with it.
from dog5_trot_quasi_static_model import trot_hw            # noqa: E402
from dog5_trot_quasi_static_model import config as cfg      # noqa: E402
from dog5_trot_quasi_static_model import gait as gait_mod   # noqa: E402


class SettleTrotGait(gait_mod.TrotGait):
    """TrotGait plus one frozen four-foot settle every settle_every cycles.

    The settle is a FIXED settle_s seconds and lands at the head of cycles
    settle_every, 2 * settle_every, ... -- trot settle_every full cycles,
    re-level once, repeat.  (The per-cycle growing settle this class used to
    schedule is gone; the accumulation bought nothing the fixed cadence does
    not.)  phase() is the ONLY override: contact(), contact_weight() and
    swing_phase() in the base class all go through phase(t), so the freeze
    propagates to everything the runner reads.  stance_duration /
    swing_duration stay the base values -- the warp is 1:1 outside the flats
    and a swing never spans one, so those are still the real seconds the
    Raibert planner and the swing-rate dsdt need.
    """

    def __init__(self, period=cfg.DEMO_GAIT_PERIOD, duty=cfg.DUTY,
                 offsets=cfg.PHASE_OFFSET, settle_s=None, settle_every=None,
                 alternate_lead=None):
        super().__init__(period, duty, offsets)
        if settle_s is None:
            settle_s = cfg.DEMO_SETTLE_S
        if settle_every is None:
            settle_every = cfg.DEMO_SETTLE_EVERY
        if alternate_lead is None:
            alternate_lead = cfg.DEMO_ALTERNATE_LEAD
        self.alternate_lead = bool(alternate_lead)
        if settle_s < 0.0:
            raise ValueError(f"settle_s must be >= 0, got {settle_s}")
        if int(settle_every) != settle_every or settle_every < 1:
            raise ValueError(
                f"settle_every must be a whole number of cycles >= 1, got "
                f"{settle_every}")
        if not self.duty > 0.5:
            raise ValueError(
                f"duty {self.duty} <= 0.5 has no all-stance window to freeze "
                f"in -- the settle would hold a foot that is meant to swing")
        self.settle_s = float(settle_s)
        self.settle_every = int(settle_every)
        # the freeze point, in seconds of GAIT time: mid the all-stance
        # window that follows the cycle's last touchdown
        self._h = (self.duty - 0.5) / 2.0 * self.period
        # MEASURED, not assumed: at the freeze point every leg must be
        # planted at FULL weight, or the settle is a lean, not a re-level.
        # ramp passed EXPLICITLY: contact_weight's default was bound at
        # gait.py import time, and this guard must see the value as
        # configured now (which is also what makes it testable).
        probe = gait_mod.TrotGait(self.period, self.duty, self.offsets)
        probe.reset(0.0)
        w_h = probe.contact_weight(self._h, ramp=cfg.CONTACT_RAMP)
        if not (bool(probe.contact(self._h).all())
                and bool(np.all(w_h >= 1.0 - 1e-9))):
            raise ValueError(
                f"freeze point {self._h:.3f}s is not full four-foot support "
                f"(duty {self.duty}, ramp {cfg.CONTACT_RAMP}): weights "
                f"{np.round(w_h, 3)}")

    # -- the periodic-settle bookkeeping -----------------------------------
    def settle_of(self, n: int) -> float:
        """Cycle n's settle, in seconds (n = 0 for the first cycle):
        settle_s when cycle n opens with one -- n a positive multiple of
        settle_every -- else 0.  Cycle 0 never settles: the settle is a
        re-level AFTER cycles, and nothing has happened yet."""
        return (self.settle_s
                if n > 0 and n % self.settle_every == 0 else 0.0)

    def cycle_start(self, n: int) -> float:
        """Wall seconds (since reset) at which cycle n begins: n periods plus
        every settle already served -- one settle_s per settle_every
        COMPLETED cycles (cycle n's own settle, if any, lies inside it)."""
        served = (n - 1) // self.settle_every if n > 0 else 0
        return n * self.period + served * self.settle_s

    def _cycle_index(self, elapsed: float) -> int:
        """Largest n with cycle_start(n) <= elapsed.  A closed-form guess off
        the settle_every-cycle block length, then nudged to absorb the
        within-block offset and float error at the boundaries."""
        if elapsed <= 0.0:
            return 0
        block = self.settle_every * self.period + self.settle_s
        n = self.settle_every * int(elapsed // block)
        while n > 0 and self.cycle_start(n) > elapsed:
            n -= 1
        while self.cycle_start(n + 1) <= elapsed:
            n += 1
        return n

    # -- the warp ----------------------------------------------------------
    def _warp(self, elapsed: float) -> float:
        """Wall seconds since reset -> gait seconds.  Piecewise linear,
        monotone, slope 1 everywhere except one flat per cycle, the flat of
        cycle n lasting settle_of(n)."""
        n = self._cycle_index(elapsed)
        e = elapsed - self.cycle_start(n)
        S = self.settle_of(n)
        if e < self._h:
            w = e
        elif e < self._h + S:
            w = self._h
        else:
            w = e - S
        return n * self.period + w

    def _flips(self, elapsed: float) -> int:
        """How many mid-settle lead flips lie before `elapsed`: one per
        completed cycle, plus this cycle's if its settle midpoint has passed."""
        n = self._cycle_index(elapsed)
        e = elapsed - self.cycle_start(n)
        return n + (1 if e >= self._h + self.settle_of(n) / 2.0 else 0)

    def phase(self, t: float) -> np.ndarray:
        elapsed = float(t) - self._t0
        ph = np.mod(self._warp(elapsed) / self.period + self.offsets, 1.0)
        if self.alternate_lead and self._flips(elapsed) % 2:
            # half a cycle swaps the diagonals: the lead of the previous
            # cycle becomes the follower of this one.  Applied mid-settle,
            # where both diagonals are mid-stance at full weight, so the
            # jump moves no contact edge and no weight.
            ph = np.mod(ph + 0.5, 1.0)
        return ph

    def settling(self, t: float) -> bool:
        """True while the clock is frozen (all four down, re-levelling)."""
        elapsed = float(t) - self._t0
        n = self._cycle_index(elapsed)
        e = elapsed - self.cycle_start(n)
        return bool(self._h <= e < self._h + self.settle_of(n))

    def __repr__(self):
        return (f"SettleTrotGait(period={self.period:.3f}s + "
                f"{self.settle_s:.2f}s settle every {self.settle_every} "
                f"cycle{'s' if self.settle_every != 1 else ''}, "
                f"lead {'ALTERNATES' if self.alternate_lead else 'fixed'}, "
                f"duty={self.duty:.2f} "
                f"stance={self.stance_duration*1e3:.0f}ms "
                f"swing={self.swing_duration*1e3:.0f}ms)")


# ===========================================================================
# self-test -- the warp only; trot_hw --self-test still owns the runner
# ===========================================================================
_PASS = [0, 0]


def check(label, ok, detail=""):
    _PASS[1] += 1
    _PASS[0] += bool(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
          + (f"  ({detail})" if detail else ""))


def self_test():
    g = SettleTrotGait()
    g.reset(0.0)
    print(f"  {g}")
    T = g.period
    K = g.settle_every
    N_CYC = 3 * K                     # three settle blocks' worth of cycles
    t_end = g.cycle_start(N_CYC)

    check("cycle n settles iff n is a positive multiple of settle_every",
          all(abs(g.settle_of(n)
                  - (g.settle_s if n > 0 and n % K == 0 else 0.0)) < 1e-12
              for n in range(N_CYC + 2)),
          f"settles open cycles "
          f"{[n for n in range(N_CYC + 2) if g.settle_of(n) > 0.0]}, "
          f"{g.settle_s:.2f}s each -- no growth, no accumulation")
    check("cycle starts accumulate every settle served so far",
          all(abs(g.cycle_start(n)
                  - (n * T + sum(g.settle_of(k) for k in range(n)))) < 1e-9
              for n in range(N_CYC + 2)),
          f"cycle {N_CYC} starts at {g.cycle_start(N_CYC):.2f}s wall "
          f"= {N_CYC}*{T:.1f}s + {(N_CYC - 1) // K} settles")

    # the closed-form cycle index must agree with cycle_start, on the
    # configured cadence and on off-default ones either side of it
    ok_idx = True
    for gk in (g, SettleTrotGait(settle_s=0.3, settle_every=1),
               SettleTrotGait(settle_s=0.25, settle_every=3)):
        for u in np.linspace(0.0, gk.cycle_start(8) - 1e-6, 3001):
            n = gk._cycle_index(u)
            ok_idx &= gk.cycle_start(n) <= u < gk.cycle_start(n + 1)
    check("_cycle_index inverts cycle_start at cadences 1, 3 and the "
          "configured one", ok_idx)
    check("every cycle begins at the offsets, shifted by that cycle's lead",
          all(np.allclose(
              g.phase(g.cycle_start(n) + 1e-9),
              np.mod(g.offsets + (0.5 if g.alternate_lead and n % 2 else 0.0),
                     1.0), atol=1e-6)
              for n in range(N_CYC)))

    # -- the alternating lead ---------------------------------------------
    # which diagonal swings FIRST each cycle: leg 0 (FL, one diagonal) vs
    # leg 1 (FR, the other).  The order must flip every cycle.
    leads = []
    for n in range(N_CYC):
        tt = np.linspace(g.cycle_start(n), g.cycle_start(n + 1), 2000,
                         endpoint=False)
        cc = np.array([g.contact(u) for u in tt])
        first = {}
        for leg in (0, 1):
            off = np.flatnonzero(~cc[:, leg])
            first[leg] = off[0] if len(off) else 10**9
        leads.append(0 if first[0] < first[1] else 1)
    check("the lead diagonal alternates cycle by cycle",
          all(leads[k] != leads[k + 1] for k in range(len(leads) - 1)),
          f"first swinger per cycle: {['FL/RR' if l == 0 else 'FR/RL' for l in leads]}")

    # the flip itself moves nothing: dense samples around every flip instant
    # (mid-settle when the cycle has one, the bare freeze point when not)
    # stay all-planted at full weight
    ok_flip = True
    for n in range(N_CYC):
        tf = g.cycle_start(n) + g._h + g.settle_of(n) / 2.0
        for u in np.linspace(tf - 0.02, tf + 0.02, 81):
            ok_flip &= bool(g.contact(u).all())
            ok_flip &= bool(np.all(g.contact_weight(u) >= 1.0 - 1e-9))
    check("the lead flip moves no contact edge and no weight, settle or not",
          ok_flip)

    # all four feet down, at full weight, through EVERY settle window; the
    # cycles between settles must never report settling at all
    ok_c = ok_w = ok_flag = True
    for n in range(N_CYC):
        S = g.settle_of(n)
        if S == 0.0:
            continue
        h0 = g.cycle_start(n) + g._h
        for u in np.linspace(h0 + 1e-6, h0 + S - 1e-6, 300):
            ok_c &= bool(g.contact(u).all())
            ok_w &= bool(np.all(g.contact_weight(u) >= 1.0 - 1e-9))
            ok_flag &= g.settling(u)
    check("all four feet planted through every settle", ok_c)
    check("...at FULL contact weight (a re-level, not a lean)", ok_w)
    check("settling(t) marks exactly those windows", ok_flag
          and not g.settling(g._h - 1e-3)
          and not any(g.settling(u) for u in np.linspace(
              g.cycle_start(1) + 1e-6, g.cycle_start(2) - 1e-6, 200)))

    # the flats, measured off the warp itself: one per settling cycle, each
    # lasting settle_s -- the cadence is in the schedule, not just the
    # arithmetic
    us = np.linspace(0.0, t_end, 40001)
    ws = np.array([g._warp(u) for u in us])
    du = us[1] - us[0]
    flat = np.diff(ws) < 1e-9
    edges = np.flatnonzero(np.diff(flat.astype(int)))
    runs = [(edges[i], edges[i + 1]) for i in range(0, len(edges) - 1, 2)]
    settle_cycles = [n for n in range(N_CYC) if g.settle_of(n) > 0.0]
    ok_n = len(runs) == len(settle_cycles)
    ok_len = ok_n and all(
        abs((b - a) * du - g.settle_s) < 3 * du for a, b in runs)
    check("one flat per settling cycle, each lasting settle_s",
          ok_n and ok_len,
          f"cycles {settle_cycles}: measured "
          f"{[round((b - a) * du, 2) for a, b in runs]} s")

    check("gait time is monotone under the warp",
          bool(np.all(np.diff(ws) >= -1e-12)))
    check("...and advances at wall rate outside the flats (no scaling)",
          abs(ws[-1] - N_CYC * T) < 1e-6,
          f"{ws[-1]:.4f}s of gait time in {t_end:.4f}s of wall time")

    # the schedule really is trot_hw's: each diagonal swings once per cycle
    # for exactly (1-duty)*period wall seconds, growth notwithstanding
    c = np.array([g.contact(u) for u in us])
    for leg in (0, 1):                       # one leg of each diagonal
        sw = ~c[:, leg]
        n_swings = int(np.sum(np.diff(sw.astype(int)) == 1))
        dur = float(np.sum(sw[:-1]) * du)
        check(f"leg {leg}: one swing per cycle, duration unchanged",
              n_swings == N_CYC
              and abs(dur / N_CYC - g.swing_duration) < 5e-3,
              f"{n_swings}/{N_CYC} cycles, {dur/N_CYC*1e3:.0f}ms vs "
              f"{g.swing_duration*1e3:.0f}ms")

    check("the settle follows the SECOND touchdown: no swing between the "
          "cycle wrap and the freeze",
          bool(np.all(np.array([g.contact(u) for u in
                                np.linspace(0.0, g._h, 200)]).all(axis=1))))

    check("at least two feet are planted at EVERY instant, flips included",
          bool(np.all(c.sum(axis=1) >= 2)),
          f"planted counts seen: {sorted(set(c.sum(axis=1).tolist()))}")

    check("settle_s=0, fixed lead degenerates to the plain TrotGait "
          "clock (= trot_hw)",
          np.allclose(
              np.array([SettleTrotGait(settle_s=0.0, alternate_lead=False)
                        .phase(u) for u in np.linspace(0, 3 * T, 500)]),
              np.array([gait_mod.TrotGait(cfg.DEMO_GAIT_PERIOD, cfg.DUTY,
                                          cfg.PHASE_OFFSET).phase(u)
                        for u in np.linspace(0, 3 * T, 500)])))
    g_every = SettleTrotGait(settle_every=1)
    check("settle_every=1 settles after EVERY cycle (the old cadence, "
          "minus the growth)",
          all(abs(g_every.settle_of(n)
                  - (g_every.settle_s if n > 0 else 0.0)) < 1e-12
              for n in range(5)))

    for bad_kw in ({"settle_s": -0.1}, {"settle_every": 0},
                   {"settle_every": 1.5}, {"settle_every": -2},
                   {"duty": 0.5}, {"duty": 0.45}):
        try:
            SettleTrotGait(**bad_kw)
            ok = False
        except ValueError:
            ok = True
        if not ok:
            break
    check("negative settle, bad settle_every and duty <= 0.5 are refused "
          "at construction", ok)

    # the freeze-point full-weight guard actually fires: a ramp so long it
    # cannot reach 1 inside the window must be refused, not run
    try:
        _saved = cfg.CONTACT_RAMP
        cfg.CONTACT_RAMP = 0.49
        SettleTrotGait()
        ok = False
    except ValueError:
        ok = True
    finally:
        cfg.CONTACT_RAMP = _saved
    check("a ramp too long for full weight at the freeze point is refused", ok)

    print(f"self-test {'PASS' if _PASS[0] == _PASS[1] else 'FAIL'} "
          f"({_PASS[0]}/{_PASS[1]})")
    return 0 if _PASS[0] == _PASS[1] else 1


# ===========================================================================
# main -- inject the settle clock, then defer to trot_hw
# ===========================================================================
def main():
    if "--self-test" in sys.argv:
        return self_test()

    # run() reaches the clock only through gait_mod.TrotGait(period, duty,
    # offsets); handing it a namespace whose TrotGait is the settle subclass
    # is the whole injection -- the loop cannot tell the difference.
    trot_hw.gait_mod = types.SimpleNamespace(TrotGait=SettleTrotGait)

    _verified_run = trot_hw.run

    def _demo_run(args):
        args.period = cfg.DEMO_GAIT_PERIOD
        # THE DEMO'S OWN RAIBERT SWITCH.  trot_hw.main() derived args.raibert
        # from RAIBERT_ON; the demo re-derives it from DEMO_RAIBERT_ON so the
        # two runners A/B independently.  None disables the placement latch
        # in run(), exactly as in trot_hw.
        args.raibert = cfg.RAIBERT_KV if cfg.DEMO_RAIBERT_ON else None
        print(f"[demo] settle-trot: trot_hw's schedule "
              f"({cfg.DEMO_GAIT_PERIOD:.2f}s cycle) + a {cfg.DEMO_SETTLE_S:.2f}s "
              f"four-foot settle every {cfg.DEMO_SETTLE_EVERY} cycle(s), lead "
              f"{'ALTERNATING' if cfg.DEMO_ALTERNATE_LEAD else 'fixed'}, "
              f"raibert {f'kv={cfg.RAIBERT_KV}' if cfg.DEMO_RAIBERT_ON else 'off'} "
              f"(edit DEMO_* in dog5_trot_quasi_static_model/config.py)")
        return _verified_run(args)

    trot_hw.run = _demo_run
    return trot_hw.main()


if __name__ == "__main__":
    sys.exit(main())
