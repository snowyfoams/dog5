#!/usr/bin/env python3
"""
t6_unplug_multi.py -- fault-injection: unplug ONE motor mid-run, prove the
other 11 don't care.

Runs the round-robin loop on all motors (status reads by default; add
--motion for the T5 sinusoid) and keeps a live per-motor miss display. While
it runs, physically unplug one motor's CAN drop, wait ~10 s, replug.

What should happen (CAN theory says so; this test PROVES it on your harness):
frames to the missing motor are still ACKed by the other nodes, so its
absence shows up ONLY as missed replies on that one id -- no error frames, no
disturbance to the other 11. If the unplug/replug glitch DOES disturb the
harness (bad stub, loose ground), the link error counters and other motors'
misses will show it.

When the missing motor answers again, the script re-enables it over CAN
(0x9B clear + 0x88 run) and reports whether that recovered it.

Pass: only the unplugged motor's misses grow; every other motor has ZERO
misses and gaps in budget; bus never leaves ERROR-ACTIVE (or auto-restarts);
the motor is usable again after replug without restarting the script.

Usage:
  python t6_unplug_multi.py --rate 200 --minutes 3
  python t6_unplug_multi.py --rate 200 --motion --amp-dps 20   # moving variant
"""

import argparse
import math
import time

from motor_library import open_bus
import cantest_common as ct
import motor_gains as param

SILENT_AFTER = 5          # consecutive misses -> declare the motor offline


def main():
    p = argparse.ArgumentParser(description="Single-motor unplug fault injection.")
    p.add_argument("--ids", default="1-12")
    p.add_argument("--rate", type=float, default=200.0, help="Hz per motor")
    p.add_argument("--minutes", type=float, default=3.0)
    p.add_argument("--motion", action="store_true",
                   help="drive the T5 speed sinusoid instead of status reads")
    p.add_argument("--amp-dps", type=float, default=20.0)
    p.add_argument("--freq", type=float, default=0.2)
    p.add_argument("--iq-limit", type=int, default=100)
    args = p.parse_args()

    if "-" in args.ids:
        lo, hi = args.ids.split("-")
        ids = list(range(int(lo), int(hi) + 1))
    else:
        ids = [int(x) for x in args.ids.split(",")]
    n = len(ids)
    slot = 1.0 / (args.rate * n)
    gap_limit = max(0.007, 1.0 / args.rate + 0.002)

    print(__doc__.split("Usage:")[0])
    print(f"Settings: {n} motors at {args.rate:.0f} Hz, {args.minutes:.0f} min, "
          f"{'MOTION (sinusoid)' if args.motion else 'status reads only'}")
    input("Enter to start; then unplug ONE motor's CAN drop, wait ~10 s, replug. ")

    ct.print_monitor_hint()
    before = ct.link_snapshot()
    bus = open_bus()
    rr = ct.RoundRobinBus(bus, ids)
    log = ct.CsvLogger("unplug_multi",
                       ["t_s", "event", "motor_id", "detail"])
    problems = []
    offline = set()
    outage_t0 = {}
    recovered = {}

    try:
        if args.motion:
            # continuous zero-torque stream; clears 0x80 latches and leaves
            # every motor running without starving the 10 ms protection
            if not ct.arm_motors(rr, rate_hz=max(args.rate, 250.0),
                                 timeout_s=None):
                raise SystemExit("arm_motors failed")
        else:
            ct.stop_all(bus, ids)
            time.sleep(0.1)
            while bus.recv(timeout=0.0) is not None:
                pass
        rr.reset_stats()

        phases = {mid: 2 * math.pi * k / n for k, mid in enumerate(ids)}
        consec_miss = {mid: 0 for mid in ids}
        rcvd_prev = {mid: 0 for mid in ids}
        miss_prev = {mid: 0 for mid in ids}
        duration = args.minutes * 60.0
        i = 0
        last_line = 0.0
        t0 = time.perf_counter()
        deadline = t0 + slot
        while True:
            t = time.perf_counter() - t0
            if t >= duration:
                break
            mid = ids[i % n]
            i += 1
            rr.poll()
            r = rr.rec[mid]

            # offline/online bookkeeping from the counters' deltas
            if r.missed > miss_prev[mid] and r.rcvd == rcvd_prev[mid]:
                consec_miss[mid] += r.missed - miss_prev[mid]
            elif r.rcvd > rcvd_prev[mid]:
                if mid in offline:
                    dt_out = t - outage_t0[mid]
                    print(f"\n  [t={t:.1f}s] motor {mid} BACK after "
                          f"{dt_out:.1f}s -- re-enabling (0x9B + 0x88)")
                    log.row(t_s=f"{t:.1f}", event="replug", motor_id=mid,
                            detail=f"outage {dt_out:.1f}s")
                    rr.send(mid, ct.CMD_CLEAR)
                    rr.wait_reply(mid, 0.02)
                    if args.motion:
                        rr.send(mid, ct.CMD_RUN)
                        rr.wait_reply(mid, 0.02)
                    recovered[mid] = True
                    offline.discard(mid)
                consec_miss[mid] = 0
            miss_prev[mid], rcvd_prev[mid] = r.missed, r.rcvd

            if consec_miss[mid] >= SILENT_AFTER and mid not in offline:
                offline.add(mid)
                outage_t0[mid] = t
                print(f"\n  [t={t:.1f}s] motor {mid} OFFLINE "
                      f"({consec_miss[mid]} consecutive misses)")
                log.row(t_s=f"{t:.1f}", event="offline", motor_id=mid, detail="")

            # the command itself
            if args.motion and mid not in offline:
                dps = args.amp_dps * math.sin(
                    2 * math.pi * args.freq * t + phases[mid])
                rr.send(mid, ct.CMD_SPEED,
                        ct.speed_cmd_data(args.iq_limit,
                                          int(dps * param.vel_gain)))
            else:
                # keep pinging an offline motor too -- that's how we see it return
                rr.send(mid, ct.CMD_STATUS2)

            if t - last_line > 1.0:
                misses = " ".join(f"{m2}:{rr.rec[m2].missed}" for m2 in ids)
                print(f"  t={t:5.0f}s  misses  {misses}   ", end="\r")
                last_line = t

            ct.pace(deadline)
            deadline += slot
        rr.finalize()

    except KeyboardInterrupt:
        print("\ninterrupted.")
    finally:
        ct.stop_all(bus, ids)
        log.close()

    # -- report ---------------------------------------------------------------
    after = ct.link_snapshot()
    print("\n\n== T6 unplug results ==")
    unplugged = sorted(set(list(offline) + list(recovered)))
    rows = []
    for mid in ids:
        r = rr.rec[mid]
        tag = ("UNPLUGGED" if mid in unplugged else "")
        rows.append([mid, r.sent, r.rcvd, r.missed,
                     f"{r.max_gap*1e3:.2f}", tag])
        if mid not in unplugged:
            if r.missed:
                problems.append(f"motor {mid} (not unplugged): {r.missed} misses"
                                " -- outage was NOT isolated")
            if r.max_gap > gap_limit:
                problems.append(f"motor {mid}: max gap {r.max_gap*1e3:.1f} ms")
    ct.print_table(["id", "sent", "rcvd", "miss", "gap ms", "note"], rows)

    if not unplugged:
        problems.append("no motor was ever unplugged -- test not exercised")
    for mid in unplugged:
        if recovered.get(mid):
            print(f"motor {mid}: went offline and RECOVERED over CAN after replug")
        else:
            problems.append(f"motor {mid} never came back / not re-enabled")

    lines, link_problems = ct.link_report(before, after)
    print("\nlink stats delta:")
    print("\n".join(lines))
    # bus-off with auto-restart is tolerated if it recovered; link_report
    # already fails a bus that ENDED degraded, and counts restarts
    problems += link_problems

    bus.shutdown()
    print(f"\nCSV: {log.path}")
    raise SystemExit(0 if ct.suite_verdict(problems) else 1)


if __name__ == "__main__":
    main()
