#!/usr/bin/env python3
"""
t2_ping_scan.py -- enumeration + health check of all 12 motors (no motion).

Pings every expected motor id with alternating status reads (0x9A / 0x9C),
strictly one request in flight at a time, so a dead motor cannot be confused
with a bus drop. Also probes a few ids ABOVE the expected range to catch
mis-assigned ids, and flags duplicate ids (two replies to one request).

Read-only: commands no motion. Motors only need power; a latched
input-signal-timeout flag (error bit 7) does not stop a motor from replying.

Usage:
  python t2_ping_scan.py                     # ids 1..12, 50 pings each
  python t2_ping_scan.py --ids 1-6 --pings 100
"""

import argparse
import time

from motor_library import open_bus
import cantest_common as ct


def parse_ids(spec: str) -> list:
    if "-" in spec:
        lo, hi = spec.split("-")
        return list(range(int(lo), int(hi) + 1))
    return [int(x) for x in spec.split(",")]


def main():
    p = argparse.ArgumentParser(description="Enumerate and ping all motors.")
    p.add_argument("--ids", default="1-12", help="expected ids, e.g. 1-12 or 1,2,5")
    p.add_argument("--pings", type=int, default=50, help="pings per motor")
    p.add_argument("--deadline-ms", type=float, default=20.0,
                   help="per-ping reply deadline")
    args = p.parse_args()

    ids = parse_ids(args.ids)
    probe_ids = [i for i in range(max(ids) + 1, max(ids) + 5)]  # catch strays
    deadline = args.deadline_ms / 1000.0

    ct.print_monitor_hint()
    before = ct.link_snapshot()
    bus = open_bus()
    rr = ct.RoundRobinBus(bus, ids + probe_ids)
    log = ct.CsvLogger("ping_scan",
                       ["motor_id", "seq", "cmd", "ok", "latency_us"])
    problems = []

    try:
        # -- expected motors: N pings each, alternating 0x9A / 0x9C ---------
        for mid in ids:
            for seq in range(args.pings):
                cmd = ct.CMD_STATUS1 if seq % 2 == 0 else ct.CMD_STATUS2
                rr.send(mid, cmd)
                ok = rr.wait_reply(mid, deadline)
                lat = rr.rec[mid].latencies[-1] * 1e6 if ok else ""
                log.row(motor_id=mid, seq=seq, cmd=f"0x{cmd:02X}",
                        ok=int(ok), latency_us=lat)
                time.sleep(0.001)   # ~100 Hz aggregate, gentle
        rr.finalize()

        # -- stray-id probe ---------------------------------------------------
        strays = []
        for mid in probe_ids:
            rr.send(mid, ct.CMD_STATUS1)
            if rr.wait_reply(mid, deadline):
                strays.append(mid)
            time.sleep(0.002)
        rr.finalize()

        # -- report -----------------------------------------------------------
        after = ct.link_snapshot()
        print(f"\n== T2 ping scan: {args.pings} pings x {len(ids)} motors ==")
        rows = []
        volts = []
        for mid in ids:
            r = rr.rec[mid]
            p50, p99, mx = ct.percentiles([x * 1e6 for x in r.latencies])
            ok = r.rcvd >= args.pings - 1 and r.unexpected == 0
            if r.rcvd < args.pings - 1:
                problems.append(f"motor {mid}: only {r.rcvd}/{args.pings} replies")
            if r.unexpected > 0:
                problems.append(f"motor {mid}: {r.unexpected} extra replies "
                                "-- DUPLICATE ID suspected")
            if r.error:
                desc = ct.decode_errors(r.error)
                if r.error & 0x80:
                    desc += "  (power-on latch: power-cycle while a zero-torque "
                    desc += "stream runs -- see prime_watchdog)"
                problems.append(f"motor {mid}: error flags 0x{r.error:02X} ({desc})")
            if r.voltage:
                volts.append((mid, r.voltage))
            state = ("run" if r.state == 0x30 else
                     "stop" if r.state == 0x10 else
                     f"0x{r.state:02X}" if r.state is not None else "?")
            rows.append([mid, f"{r.rcvd}/{args.pings}",
                         f"{p50:.0f}/{p99:.0f}/{mx:.0f}",
                         r.temp if r.temp is not None else "?",
                         f"{r.voltage:.1f}" if r.voltage else "?",
                         state,
                         f"0x{r.error:02X}" if r.error is not None else "0x00",
                         "OK" if ok and not r.error else "CHECK"])
        ct.print_table(["id", "replies", "lat us p50/p99/max", "temp C",
                        "volt V", "state", "err", "verdict"], rows)

        if strays:
            problems.append(f"unexpected motors replying on ids {strays} "
                            "-- id assignment wrong")
        if volts:
            vmin, vmax = min(v for _, v in volts), max(v for _, v in volts)
            print(f"\nsupply voltage across motors: {vmin:.1f} .. {vmax:.1f} V")
            if vmin < 0.9 * vmax:
                problems.append(f"voltage spread > 10% ({vmin:.1f} vs {vmax:.1f} V)"
                                " -- check power wiring/gauge to far motors")

        lines, link_problems = ct.link_report(before, after)
        print("\nlink stats delta:")
        print("\n".join(lines))
        problems += link_problems

    finally:
        log.close()
        bus.shutdown()

    print(f"\nCSV: {log.path}")
    raise SystemExit(0 if ct.suite_verdict(problems) else 1)


if __name__ == "__main__":
    main()
