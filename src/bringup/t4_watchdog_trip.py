#!/usr/bin/env python3
"""
t4_watchdog_trip.py -- measure the REAL input-signal-timeout threshold and the
recovery path, on ONE motor.

Why: the protection window is believed to be 10 ms (motor setting), but the
library docs describe a ~500 ms power-on watchdog that LATCHES until a power
cycle. Every other test's minimum safe command rate depends on the real
number, so this test measures it: spin one motor slowly, deliberately pause
the command stream for increasing gaps, and see which gap trips protection.

After a trip it walks a recovery ladder and reports the first rung that works:
  a) just resume commanding
  b) clear error flags (0x9B)
  c) shutdown+run restart (0x9B + 0x80 + 0x88)
  d) power cycle required

SAFETY: bench only. The output must be free to rotate CONTINUOUSLY (no wire
wrap / hard stop). Keep an e-stop in reach. Low speed, low current limit.

Usage:
  python t4_watchdog_trip.py --motor-id 1
  python t4_watchdog_trip.py --gaps 5,8,10,12,15,20,50,100,200,500,700
"""

import argparse
import time

from motor_library import LKMotor, open_bus, prime_watchdog
import cantest_common as ct
import motor_gains as param


def stream(rr, mid, speed_data, seconds, rate_hz):
    """Stream the speed command for `seconds` at rate_hz."""
    dt = 1.0 / rate_hz
    deadline = time.perf_counter() + dt
    t_end = time.perf_counter() + seconds
    while time.perf_counter() < t_end:
        rr.poll()
        rr.send(mid, ct.CMD_SPEED, speed_data)
        ct.pace(deadline)
        deadline += dt
    rr.poll()


def query(rr, mid, cmd, timeout=0.05):
    """One request-reply through the round-robin layer; returns the record."""
    rr.send(mid, cmd)
    rr.wait_reply(mid, timeout)
    return rr.rec[mid]


def main():
    p = argparse.ArgumentParser(description="Measure the input-protection window.")
    p.add_argument("--motor-id", type=int, default=1)
    p.add_argument("--speed-dps", type=float, default=30.0)
    p.add_argument("--iq-limit", type=int, default=100)
    p.add_argument("--rate-hz", type=float, default=200.0)
    p.add_argument("--gaps", default="5,8,10,12,15,20,50,100,200,500,700",
                   help="command-stream pauses to try, in ms, ascending")
    args = p.parse_args()

    mid = args.motor_id
    gaps_ms = [float(g) for g in args.gaps.split(",")]
    speed_data = ct.speed_cmd_data(args.iq_limit,
                                   int(args.speed_dps * param.vel_gain))
    stop_data = ct.speed_cmd_data(args.iq_limit, 0)
    moving_floor = args.speed_dps * param.vel_state_gain * 0.5  # raw units

    print(__doc__.split("Usage:")[0])
    print(f"Settings: motor {mid}, {args.speed_dps} dps, iq-limit "
          f"{args.iq_limit}, stream {args.rate_hz:.0f} Hz, gaps {gaps_ms} ms")
    input("Motor free to spin continuously? Enter to continue, Ctrl-C to abort. ")

    bus = open_bus()
    motor = LKMotor(motor_id=mid, bus=bus)
    rr = ct.RoundRobinBus(bus, [mid])
    log = ct.CsvLogger("watchdog_trip",
                       ["gap_ms", "speed_after_gap_raw", "err_after_gap",
                        "tripped", "auto_resumed", "recovery"])
    results = []
    problems = []

    try:
        # power-on flow: stream zero torque, user powers the motor on
        if not prime_watchdog(motor):
            raise SystemExit("prime_watchdog failed")
        motor.flush_rx()
        motor.ensure_running()

        for gap_ms in gaps_ms:
            print(f"\n-- gap {gap_ms:.0f} ms --")
            # spin up and hold so a stop is unambiguous
            stream(rr, mid, speed_data, 2.0, args.rate_hz)
            r = query(rr, mid, ct.CMD_STATUS2)
            print(f"  spinning at {r.speed} raw "
                  f"({(r.speed or 0) / param.vel_state_gain:+.1f} dps output)")
            if r.speed is None or abs(r.speed) < moving_floor:
                problems.append(f"motor never reached speed before {gap_ms} ms gap")
                print("  NOT SPINNING -- aborting sweep")
                break

            # the deliberate silence
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < gap_ms / 1000.0:
                pass  # busy wait: nothing may go on the wire for this motor

            # first frames after the gap are READS, before any resume
            r = query(rr, mid, ct.CMD_STATUS1)
            err_after = r.error or 0
            r = query(rr, mid, ct.CMD_STATUS2)
            spd_after = r.speed or 0
            tripped = bool(err_after & 0x80) or abs(spd_after) < moving_floor
            print(f"  after gap: speed {spd_after} raw, error 0x{err_after:02X} "
                  f"({ct.decode_errors(err_after)})  -> "
                  f"{'TRIPPED' if tripped else 'no trip'}")

            auto_resumed = None
            recovery = ""
            if tripped:
                # ladder rung a: just resume the stream
                stream(rr, mid, speed_data, 1.0, args.rate_hz)
                r = query(rr, mid, ct.CMD_STATUS2)
                auto_resumed = abs(r.speed or 0) >= moving_floor
                if auto_resumed:
                    recovery = "resume commands"
                else:
                    query(rr, mid, ct.CMD_CLEAR)               # rung b
                    stream(rr, mid, speed_data, 1.0, args.rate_hz)
                    r = query(rr, mid, ct.CMD_STATUS2)
                    if abs(r.speed or 0) >= moving_floor:
                        recovery = "clear errors (0x9B)"
                    else:
                        query(rr, mid, ct.CMD_CLEAR)           # rung c
                        query(rr, mid, ct.CMD_SHUTDOWN)
                        query(rr, mid, ct.CMD_RUN)
                        stream(rr, mid, speed_data, 1.0, args.rate_hz)
                        r = query(rr, mid, ct.CMD_STATUS2)
                        if abs(r.speed or 0) >= moving_floor:
                            recovery = "clear + restart (0x9B/0x80/0x88)"
                        else:
                            recovery = "POWER CYCLE REQUIRED"
                print(f"  recovery: {recovery}")
            results.append((gap_ms, spd_after, err_after, tripped,
                            auto_resumed, recovery))
            log.row(gap_ms=gap_ms, speed_after_gap_raw=spd_after,
                    err_after_gap=f"0x{err_after:02X}", tripped=int(tripped),
                    auto_resumed="" if auto_resumed is None else int(auto_resumed),
                    recovery=recovery)

            if tripped and recovery == "POWER CYCLE REQUIRED":
                ans = input("  power-cycle the motor now and press Enter to "
                            "continue the sweep (or 'q' to stop): ")
                if ans.strip().lower() == "q":
                    break
                if not prime_watchdog(motor):
                    break
                motor.flush_rx()
                motor.ensure_running()

        # wind down
        stream(rr, mid, stop_data, 0.5, args.rate_hz)

        # -- verdict ---------------------------------------------------------
        print("\n== T4 results ==")
        rows = [[f"{g:.0f}", s, f"0x{e:02X}", "yes" if t else "no",
                 ("" if a is None else ("yes" if a else "no")), rec or "-"]
                for g, s, e, t, a, rec in results]
        ct.print_table(["gap ms", "speed raw", "err", "tripped",
                        "auto-resume", "recovery"], rows)

        trip_gaps = [g for g, _, _, t, _, _ in results if t]
        if trip_gaps:
            threshold = min(trip_gaps)
            safe_rate = 3.0 / (threshold / 1000.0)
            print(f"\nmeasured protection threshold: first trip at "
                  f"{threshold:.0f} ms gap")
            print(f"minimum safe command rate per motor: 3x margin -> "
                  f"{safe_rate:.0f} Hz  (period <= {threshold/3:.1f} ms)")
            # a LARGER gap that did NOT trip after a smaller one did means the
            # protection is not deterministic -- do not rely on it as a stop
            no_trip_above = [g for g, _, _, t, _, _ in results
                             if not t and g > threshold]
            if no_trip_above:
                print(f"\n!!! SAFETY FINDING: gaps {no_trip_above} ms did NOT "
                      f"trip although {threshold:.0f} ms did -- protection is "
                      "inconsistent; do not rely on it as an e-stop. !!!")
                problems.append("protection threshold inconsistent across gaps")
        else:
            print(f"\nno trip up to {max(gaps_ms):.0f} ms -- the effective "
                  "window is larger than every tested gap. If you expected "
                  "10 ms, check the motor's protection setting in the LK "
                  "config tool.")
            problems.append("no trip observed; threshold not established")

    except KeyboardInterrupt:
        print("\ninterrupted.")
    finally:
        ct.stop_all(bus, [mid])
        log.close()
        bus.shutdown()

    print(f"\nCSV: {log.path}")
    raise SystemExit(0 if ct.suite_verdict(problems) else 1)


if __name__ == "__main__":
    main()
