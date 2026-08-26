#!/usr/bin/env python3
"""t8: WHY do motors latch input-lost (0x80)?  Gap-vs-brownout discriminator.

CONTEXT (2026-08-12)
    Input-lost latches persist across runs.  Bus load is ruled out (they
    survive --control-hz 100), wiring is ruled out by inspection.  Two
    hypotheses remain, and they leave different fingerprints:

    H1  HOST STALL: the control process stops sending for >= the driver's
        50 ms watchdog (GC / print blocking / kernel).  Fingerprint: the
        latched motor's last command was sent >= ~50 ms before the latch.

    H2  MOTOR REBOOT: a supply dip resets the driver, and a rebooting driver
        comes up with 0x80 ALREADY SET (motorbus arm() docs) -- no command
        gap needed, which is exactly why lowering the rate changed nothing.
        Fingerprint: latch with NO preceding send gap, missed replies from
        that motor around the event, possibly a voltage sag in 0x9A reads.

    This probe streams the same zero-torque keep-alive the runners use and
    records, for every latch: the send gap to that motor, the worst host
    stall in the last second, the motor's missed-reply delta, and its last
    two voltage readings.  Robot just sits there; nothing moves.

RUN
    python3 t8_lost_probe.py --self-test
    sudo chrt -f 50 python3 t8_lost_probe.py --ids 1-12 --minutes 5
    # compare a nice-less run (no chrt) to make host stalls MORE likely:
    python3 t8_lost_probe.py --ids 1-12 --minutes 5

    Leave everything powered exactly as in a failing stand run.  If latches
    only ever happen in real stands (under load), rerun this with the robot
    STANDING under a runner first, then use the verdict table here as the
    reference for reading that runner's latch prints.
"""
from __future__ import annotations

import argparse
import collections
import sys
import time

import motorbus


def parse_ids(spec):
    ids = []
    for part in spec.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-")
            ids.extend(range(int(a), int(b) + 1))
        elif part:
            ids.append(int(part))
    return ids


class GapTracker:
    """Per-motor send times + a global top-N of inter-iteration stalls."""

    def __init__(self, ids, top_n=10):
        self.last_send = {mid: None for mid in ids}
        self.top = []                 # (gap_s, t, "loop"/mid) worst first
        self.top_n = top_n
        self._last_iter = None

    def loop_tick(self, now):
        if self._last_iter is not None:
            self._note(now - self._last_iter, now, "loop")
        self._last_iter = now

    def sent(self, mid, now):
        prev = self.last_send[mid]
        self.last_send[mid] = now
        if prev is not None:
            self._note(now - prev, now, mid)

    def _note(self, gap, now, what):
        self.top.append((gap, now, what))
        self.top.sort(reverse=True)
        del self.top[self.top_n:]

    def gap_to(self, mid, now):
        prev = self.last_send[mid]
        return float("inf") if prev is None else now - prev

    def worst_recent(self, now, window_s=1.0):
        return max((g for g, t, w in self.top if now - t <= window_s and
                    w == "loop"), default=0.0)


class LatchLog:
    """One row per 0x80 latch, with the evidence for the H1/H2 verdict."""

    def __init__(self, stall_ms=40.0):
        self.rows = []
        self.stall_s = stall_ms * 1e-3

    def record(self, t, mid, send_gap, worst_loop_gap, missed_delta,
               volts_prev, volts_now):
        self.rows.append(dict(t=t, mid=mid, send_gap=send_gap,
                              loop_gap=worst_loop_gap, missed=missed_delta,
                              v_prev=volts_prev, v_now=volts_now))

    def verdict(self, row):
        if row["send_gap"] >= self.stall_s or row["loop_gap"] >= self.stall_s:
            return "H1 host-stall"
        if row["missed"] > 0:
            return "H2 reboot?  (no gap, replies dropped)"
        return "H2 reboot?  (no gap)"

    def summary(self):
        if not self.rows:
            return ["[t8] NO latches this run -- nothing to explain. "
                    "Run longer, or probe while a stand is loading the rail."]
        lines = [f"[t8] {len(self.rows)} latch(es):",
                 "   t(s)  motor  send-gap  worst-loop-gap  missedΔ  "
                 "V(prev->at)  verdict"]
        per = collections.Counter()
        for r in self.rows:
            per[self.verdict(r).split()[0]] += 1
            v = ("     ?      " if r["v_now"] is None else
                 f"{r['v_prev'] if r['v_prev'] is not None else float('nan'):5.1f}"
                 f"->{r['v_now']:5.1f}")
            lines.append(
                f"  {r['t']:6.1f}  {r['mid']:5d}  "
                f"{r['send_gap']*1e3:7.1f}ms  {r['loop_gap']*1e3:9.1f}ms  "
                f"{r['missed']:7d}  {v}  {self.verdict(r)}")
        lines.append(f"  -> {per['H1']} host-stall, {per['H2']} reboot-like."
                     "  H1 fix: find the stall (gap top-10 above)."
                     "  H2 fix: power rail / connectors at the MOTOR side, "
                     "scope the supply during a stand.")
        return lines


def run(args):
    ids = parse_ids(args.ids)
    log = LatchLog(stall_ms=args.stall_ms)
    gaps = GapTracker(ids)
    volts = {mid: (None, None) for mid in ids}      # (prev, latest)
    missed_at = {mid: 0 for mid in ids}
    latched_before = set()

    print(f"[t8] streaming zero-torque keep-alive to {ids} at "
          f"{args.rate_hz:.0f} Hz/motor for {args.minutes:.1f} min; "
          f"one 0x9A per motor every ~{args.status_s:.1f}s "
          f"(stall threshold {args.stall_ms:.0f} ms, watchdog 50 ms)")
    with motorbus.MotorBus(ids) as mb:
        if not mb.arm(rate_hz=args.rate_hz):
            raise RuntimeError("arm failed")
        slot = mb.slot(args.rate_hz)
        deadline = time.perf_counter() + slot
        t0 = time.perf_counter()
        end = t0 + args.minutes * 60.0
        idx = 0
        next_status = t0 + args.status_s / len(ids)
        while time.perf_counter() < end:
            mb.poll()
            now = time.perf_counter()
            gaps.loop_tick(now)
            mid = ids[idx % len(ids)]

            if now >= next_status:
                mb.status1_req(mid)          # voltage + error flags refresh
                next_status = now + args.status_s / len(ids)
            else:
                mb.keepalive(mid)
            gaps.sent(mid, now)

            rec = mb.rec(mid)
            if rec.voltage is not None and rec.voltage != volts[mid][1]:
                volts[mid] = (volts[mid][1], rec.voltage)

            err = rec.error
            if err is not None and (err & 0x80):
                if mid not in latched_before:
                    latched_before.add(mid)
                    log.record(now - t0, mid, gaps.gap_to(mid, now),
                               gaps.worst_recent(now), rec.missed
                               - missed_at[mid], *volts[mid])
                    print(f"[t8] LATCH motor {mid} at t={now-t0:.1f}s  "
                          f"send-gap {gaps.gap_to(mid, now)*1e3:.1f}ms  "
                          f"missedΔ {rec.missed - missed_at[mid]}")
                    # t7 ladder (0x9B -> 0x88), then invalidate the cached
                    # error so the next verdict comes from a fresh 0x9A
                    mb.recover([mid], settle_s=0.0, verify=False)
                    mb.rec(mid).error = None
            else:
                latched_before.discard(mid)
            missed_at[mid] = rec.missed

            idx += 1
            over = mb.pace(deadline)
            deadline += slot
            if over and over > 2.0 * slot:
                deadline = time.perf_counter() + slot

    print("[t8] top stalls (loop + per-motor send gaps):")
    for g, t, what in gaps.top:
        print(f"    {g*1e3:7.2f} ms at t={t-t0:7.1f}s  ({what})")
    for line in log.summary():
        print(line)


# --------------------------- offline self-test -----------------------------

def self_test():
    fails = []

    def check(name, ok):
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
        if not ok:
            fails.append(name)

    g = GapTracker([1, 2])
    g.loop_tick(0.000); g.sent(1, 0.000)
    g.loop_tick(0.004); g.sent(2, 0.004)
    g.loop_tick(0.104)                       # a 100 ms loop stall
    g.sent(1, 0.104)
    # motor 1's send gap spans the whole stall (0.000 -> 0.104)
    check("send gap includes the stall", abs(g.gap_to(1, 0.104)) < 1e-9
          and abs(g.top[0][0] - 0.104) < 1e-9)
    check("worst_recent sees the loop stall",
          abs(g.worst_recent(0.2) - 0.100) < 1e-9)

    log = LatchLog(stall_ms=40)
    log.record(1.0, 7, send_gap=0.120, worst_loop_gap=0.120, missed_delta=3,
               volts_prev=24.1, volts_now=24.0)
    log.record(2.0, 8, send_gap=0.004, worst_loop_gap=0.006, missed_delta=5,
               volts_prev=24.1, volts_now=22.7)
    log.record(3.0, 9, send_gap=0.004, worst_loop_gap=0.006, missed_delta=0,
               volts_prev=None, volts_now=None)
    check("gap past threshold reads H1", log.verdict(log.rows[0]).startswith("H1"))
    check("no gap + drops reads H2", "H2" in log.verdict(log.rows[1])
          and "dropped" in log.verdict(log.rows[1]))
    check("no gap, clean replies still H2", "H2" in log.verdict(log.rows[2]))
    s = log.summary()
    check("summary counts both hypotheses", any("1 host-stall" in l and
                                                "2 reboot-like" in l for l in s))
    print("self-test " + ("FAIL: " + ", ".join(fails) if fails else "PASS"))
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", default="1-12", help="e.g. 1 | 1,7,8 | 1-12")
    ap.add_argument("--rate-hz", type=float, default=250.0)
    ap.add_argument("--minutes", type=float, default=5.0)
    ap.add_argument("--status-s", type=float, default=1.0,
                    help="seconds between 0x9A voltage/error reads per motor")
    ap.add_argument("--stall-ms", type=float, default=40.0,
                    help="send/loop gap that counts as a host stall")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        sys.exit(self_test())
    run(args)


if __name__ == "__main__":
    main()
