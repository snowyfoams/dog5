#!/usr/bin/env python3
"""
t3_rate_sweep.py -- find the maximum clean command rate for 12 motors on one
bus (no motion; status reads 0x9C only).

Why: 12 motors x 400 Hz x 2 frames (cmd+reply) = 9,600 frames/s, but a
1 Mbit/s bus carries only ~8,000-8,300 standard 8-byte frames/s. 400 Hz per
motor CANNOT fit with request-reply on one bus. This test sweeps the rate and
reports the highest step with ZERO misses, zero send failures, and every
motor's inter-command gap safely inside the input-protection window.

Commands are evenly interleaved (slot period = 1/(rate x N)), not sent in
per-cycle bursts, so each motor's gap equals its command period exactly.

Pass (per step): 0 misses, 0 send fails, every motor's max inter-command gap
< nominal period + 2 ms, p99 latency < 5 ms, no link error growth. The sweep
stops escalating at the first failed step. Steps whose period exceeds ~7 ms
(rates under ~150 Hz) are noted as too slow to feed a 10 ms input protection
with margin -- they are comm-only data points.

Usage:
  python t3_rate_sweep.py --step-seconds 30 --probe-broadcast
  python t3_rate_sweep.py --rates 250 --step-seconds 60      # confirmation run
"""

import argparse
import time

import can

from motor_library import open_bus
import cantest_common as ct

WATCHDOG_BUDGET_S = 0.007   # 10 ms input protection minus 30 % margin
JITTER_ALLOW_S = 0.002      # scheduling slack on top of the nominal period
P99_LIMIT_S = 0.005


def main():
    p = argparse.ArgumentParser(description="12-motor CAN rate sweep (no motion).")
    p.add_argument("--ids", default="1-12")
    p.add_argument("--rates", default="100,150,200,250,300,350,400",
                   help="per-motor rates in Hz, comma separated")
    p.add_argument("--step-seconds", type=float, default=30.0)
    p.add_argument("--probe-broadcast", action="store_true",
                   help="probe the 0x280 multi-motor command (iq=0, motors "
                        "stopped) -- decides if 400 Hz can fit on one bus")
    p.add_argument("--dump-latencies", action="store_true",
                   help="also write every latency sample to a CSV")
    args = p.parse_args()

    if "-" in args.ids:
        lo, hi = args.ids.split("-")
        ids = list(range(int(lo), int(hi) + 1))
    else:
        ids = [int(x) for x in args.ids.split(",")]
    rates = [float(r) for r in args.rates.split(",")]
    n = len(ids)

    ct.print_monitor_hint()
    before = ct.link_snapshot()
    bus = open_bus()
    rr = ct.RoundRobinBus(bus, ids)
    log = ct.CsvLogger("rate_sweep",
                       ["rate_hz", "motor_id", "sent", "rcvd", "missed",
                        "miss_pct", "send_fail", "unexpected",
                        "lat_p50_us", "lat_p99_us", "lat_max_us",
                        "max_gap_ms", "overruns", "bus_load_pct"])
    lat_log = (ct.CsvLogger("rate_sweep_latencies",
                            ["rate_hz", "motor_id", "latency_us"])
               if args.dump_latencies else None)
    problems = []
    max_clean = None

    try:
        # make sure nothing moves and RX starts clean
        ct.stop_all(bus, ids)
        time.sleep(0.1)
        while bus.recv(timeout=0.0) is not None:
            pass

        for rate in rates:
            slot = 1.0 / (rate * n)
            period = 1.0 / rate
            # the gap check is scheduling consistency: each motor's command
            # period must stay at its nominal value plus a little jitter. The
            # 10 ms watchdog fit is a property of the RATE, reported separately.
            gap_limit = period + JITTER_ALLOW_S
            print(f"\n-- step: {rate:.0f} Hz/motor "
                  f"({rate * n * 2:.0f} frames/s incl. replies, "
                  f"slot {slot * 1e6:.0f} us, {args.step_seconds:.0f} s) --")
            if period > WATCHDOG_BUDGET_S:
                print(f"   note: {rate:.0f} Hz means a {period*1e3:.1f} ms "
                      "command period -- too slow to feed a 10 ms input "
                      "protection with margin; comm-only data point.")
            rr.reset_stats()
            overruns = 0
            i = 0
            deadline = time.perf_counter() + slot
            t_end = time.perf_counter() + args.step_seconds
            while time.perf_counter() < t_end:
                rr.poll()
                rr.send(ids[i % n], ct.CMD_STATUS2)
                i += 1
                if ct.pace(deadline) > 0.0005:
                    overruns += 1
                deadline += slot
            rr.finalize()
            load = rr.bus_load()

            step_bad = []
            rows = []
            for mid in ids:
                r = rr.rec[mid]
                p50, p99, mx = ct.percentiles(r.latencies)
                miss_pct = 100.0 * r.missed / max(r.sent, 1)
                rows.append([mid, r.sent, r.rcvd, r.missed,
                             f"{miss_pct:.3f}", r.send_fail,
                             f"{p50*1e6:.0f}/{p99*1e6:.0f}/{mx*1e6:.0f}",
                             f"{r.max_gap*1e3:.2f}"])
                log.row(rate_hz=rate, motor_id=mid, sent=r.sent, rcvd=r.rcvd,
                        missed=r.missed, miss_pct=f"{miss_pct:.4f}",
                        send_fail=r.send_fail, unexpected=r.unexpected,
                        lat_p50_us=f"{p50*1e6:.0f}", lat_p99_us=f"{p99*1e6:.0f}",
                        lat_max_us=f"{mx*1e6:.0f}",
                        max_gap_ms=f"{r.max_gap*1e3:.3f}", overruns=overruns,
                        bus_load_pct=f"{load*100:.1f}")
                if lat_log:
                    for s in r.latencies:
                        lat_log.row(rate_hz=rate, motor_id=mid,
                                    latency_us=f"{s*1e6:.0f}")
                if r.missed:
                    step_bad.append(f"motor {mid}: {r.missed} misses")
                if r.send_fail:
                    step_bad.append(f"motor {mid}: {r.send_fail} send fails (TX queue full)")
                if r.max_gap > gap_limit:
                    step_bad.append(f"motor {mid}: max gap {r.max_gap*1e3:.1f} ms "
                                    f"> {gap_limit*1e3:.1f} ms (period + jitter)")
                if p99 > P99_LIMIT_S:
                    step_bad.append(f"motor {mid}: p99 latency {p99*1e3:.1f} ms > 5 ms")

            ct.print_table(["id", "sent", "rcvd", "miss", "miss%",
                            "sfail", "lat us p50/p99/max", "gap ms"], rows)
            print(f"bus load ~{load*100:.0f} %   loop overruns {overruns}"
                  f"   error frames {rr.error_frames}   foreign {rr.foreign}")
            if step_bad:
                print(f"STEP {rate:.0f} Hz: FAIL")
                for b in step_bad[:8]:
                    print(f"  - {b}")
                problems.append(f"{rate:.0f} Hz step failed "
                                f"({len(step_bad)} findings)")
                break
            print(f"STEP {rate:.0f} Hz: CLEAN")
            max_clean = rate

        # -- optional 0x280 multi-motor probe --------------------------------
        broadcast = None
        if args.probe_broadcast:
            print("\n-- probing 0x280 multi-motor iq command (iq=0, motors stopped) --")
            while bus.recv(timeout=0.0) is not None:
                pass
            bus.send(can.Message(arbitration_id=0x280, data=[0x00] * 8,
                                 is_extended_id=False))
            got = set()
            t_end = time.perf_counter() + 0.2
            while time.perf_counter() < t_end:
                msg = bus.recv(timeout=0.05)
                if msg is None or msg.is_error_frame:
                    continue
                # LK-family multi-motor replies land on 0x240+id, not 0x140+id
                if 0x241 <= msg.arbitration_id <= 0x244:
                    got.add(msg.arbitration_id - 0x240)
                elif 0x141 <= msg.arbitration_id <= 0x144:
                    got.add(msg.arbitration_id - 0x140)
            if got:
                broadcast = True
                print(f"  supported: motors {sorted(got)} replied to 0x280")
            else:
                broadcast = False
                print("  no reply -- unsupported on this firmware, or replies "
                      "disabled; INCONCLUSIVE, cross-check docs/motor_control.pdf")

        # -- summary ------------------------------------------------------------
        after = ct.link_snapshot()
        lines, link_problems = ct.link_report(before, after)
        print("\nlink stats delta:")
        print("\n".join(lines))
        problems += link_problems

        print("\n== T3 verdict ==")
        if max_clean is None:
            print("no clean step at all -- fix wiring/termination before rate work")
            problems.append("no clean rate found")
        else:
            print(f"max clean rate: {max_clean:.0f} Hz/motor "
                  f"({max_clean * n * 2:.0f} frames/s)")
            if max_clean < 250:
                problems.append(f"max clean rate {max_clean:.0f} Hz < 250 Hz target")
        print("\nGetting to 400 Hz/motor:")
        print(f"  a) accept the measured ceiling ({max_clean or 0:.0f} Hz) on one bus")
        print("  b) split into 2 buses x 6 motors -> 400 Hz x 6 x 2 = 4800 f/s "
              "(~58 % load) per bus: fits")
        if args.probe_broadcast:
            if broadcast:
                print("  c) 0x280 multi-motor iq: SUPPORTED -> 3 cmd frames + 12 "
                      "replies = 15 f/cycle; 400 Hz = 6000 f/s (~72 % load): fits")
            else:
                print("  c) 0x280 multi-motor iq: not confirmed on this firmware")
        else:
            print("  c) re-run with --probe-broadcast to check the 0x280 option")

    except KeyboardInterrupt:
        print("\ninterrupted.")
    finally:
        ct.stop_all(bus, ids)
        log.close()
        if lat_log:
            lat_log.close()
        bus.shutdown()

    print(f"\nCSV: {log.path}")
    raise SystemExit(0 if ct.suite_verdict(problems) else 1)


if __name__ == "__main__":
    main()
