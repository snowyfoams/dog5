#!/usr/bin/env python3
"""
t5_motion_soak.py -- all 12 motors MOVING simultaneously at the proven rate,
for several minutes. The final stability proof.
 
Motion: a small zero-mean speed sinusoid per motor (default 30 dps amplitude,
0.2 Hz), phases staggered across motors so the supply current stays smooth.
Position oscillates roughly +/- amp/(2*pi*f) degrees (~24 deg at defaults) --
outputs must still be FREE TO ROTATE in both directions with margin, because
speed tracking bias makes the position wander over minutes.

The 0xA2 reply already carries temp/iq/speed/encoder, so telemetry is free.
Once per 5 s each motor's slot substitutes a status-1 read (0x9A) for
voltage + error-flag monitoring.

Run t3_rate_sweep first: --rate must be a rate T3 proved clean (use one step
below the ceiling). The script refuses rates above 300 Hz without --force.

Pass: zero misses / send fails, every gap < 7 ms, no error flag ever set, no
motor unexpectedly stopped, temp rise < 15 C and < 60 C absolute, voltage
never < 90 % of start, no link error growth.

SAFETY: outputs free to rotate, e-stop in reach, supply rated for 12 motors.
Ctrl-C at any time -> stop-all.

Usage:
  python t5_motion_soak.py --rate 200 --minutes 5
  python t5_motion_soak.py --rate 250 --minutes 10 --amp-dps 20
"""

import argparse
import math
import time

from motor_library import open_bus
import cantest_common as ct
import motor_gains as param

STATUS_PERIOD_S = 5.0     # per-motor 0x9A substitution interval
RAMP_S = 2.0              # amplitude ramp at start/end


def main():
    p = argparse.ArgumentParser(description="12-motor motion soak test.")
    p.add_argument("--ids", default="1-12")
    p.add_argument("--rate", type=float, required=True,
                   help="per-motor command rate in Hz -- use a rate "
                        "t3_rate_sweep proved clean (one step below ceiling)")
    p.add_argument("--minutes", type=float, default=5.0)
    p.add_argument("--amp-dps", type=float, default=30.0,
                   help="speed sinusoid amplitude, output deg/s")
    p.add_argument("--freq", type=float, default=0.2, help="sinusoid Hz")
    p.add_argument("--iq-limit", type=int, default=100,
                   help="current limit for the speed loop, raw (keep modest)")
    p.add_argument("--gap-limit-ms", type=float, default=7.0,
                   help="max tolerated inter-command gap per motor. Default "
                        "7 ms = 10 ms input protection minus margin; raise it "
                        "if t4_watchdog_trip measured a larger real window")
    p.add_argument("--force", action="store_true",
                   help="allow rates above 300 Hz")
    args = p.parse_args()

    if "-" in args.ids:
        lo, hi = args.ids.split("-")
        ids = list(range(int(lo), int(hi) + 1))
    else:
        ids = [int(x) for x in args.ids.split(",")]
    n = len(ids)

    if args.rate > 300 and not args.force:
        raise SystemExit(f"{args.rate:.0f} Hz x {n} motors is near/over the "
                         "1 Mbit/s bus ceiling. Prove it clean with "
                         "t3_rate_sweep first, then pass --force.")

    slot = 1.0 / (args.rate * n)
    # gap budget: the watchdog margin, but never tighter than the commanded
    # period itself plus scheduling jitter (a 100 Hz run has 10 ms gaps by design)
    gap_limit = max(args.gap_limit_ms / 1000.0, 1.0 / args.rate + 0.002)
    if 1.0 / args.rate > args.gap_limit_ms / 1000.0:
        print(f"WARNING: at {args.rate:.0f} Hz the command period "
              f"({1000.0/args.rate:.1f} ms) already exceeds the "
              f"{args.gap_limit_ms:.0f} ms protection budget -- motors with a "
              "true 10 ms window will trip. Continue only if t4 measured a "
              "larger window.")
    swing = args.amp_dps / (2 * math.pi * args.freq)
    print(__doc__.split("Usage:")[0])
    print(f"Settings: {n} motors, {args.rate:.0f} Hz/motor "
          f"(slot {slot*1e6:.0f} us, ~{args.rate*n*2*ct.FRAME_BITS/ct.BITRATE*100:.0f} % bus load), "
          f"{args.minutes:.0f} min, amp {args.amp_dps:.0f} dps @ {args.freq} Hz "
          f"(position swing ~+/-{swing:.0f} deg)")
    print("\nChecklist: [ ] all outputs free to rotate BOTH ways with margin"
          "\n           [ ] e-stop in reach"
          "\n           [ ] supply rated for 12 motors")
    if input("Type 'y' to arm: ").strip().lower() != "y":
        raise SystemExit("aborted.")

    ct.print_monitor_hint()
    before = ct.link_snapshot()
    bus = open_bus()
    rr = ct.RoundRobinBus(bus, ids)
    log = ct.CsvLogger("motion_soak",
                       ["t_s", "motor_id", "cmd_dps", "meas_dps", "temp",
                        "voltage", "error"])
    problems = []
    temp_start, temp_max = {}, {}
    volt_start, volt_min = {}, {}
    track_err2, track_n = {mid: 0.0 for mid in ids}, {mid: 0 for mid in ids}

    try:
        # power-on flow: continuous zero-torque stream that clears the 0x80
        # latch and leaves every motor in RUN state, never letting any
        # motor's command gap reach the 10 ms protection window. (The old
        # prime_watchdog + sequential wait_reply enables starved the stream
        # with 12 motors and re-latched the protection every time.)
        if not ct.arm_motors(rr, rate_hz=max(args.rate, 250.0),
                             timeout_s=None):
            raise SystemExit("arm_motors failed")

        # baselines from the status replies arm_motors just collected -- a
        # sequential read sweep here would starve the stream again
        for mid in ids:
            r = rr.rec[mid]
            temp_start[mid] = temp_max[mid] = r.temp if r.temp is not None else 0
            volt_start[mid] = volt_min[mid] = r.voltage if r.voltage else 0.0
        rr.reset_stats()

        print("\nsoaking... Ctrl-C to stop early (stop-all runs either way)\n")
        duration = args.minutes * 60.0
        phases = {mid: 2 * math.pi * k / n for k, mid in enumerate(ids)}
        next_status = {mid: STATUS_PERIOD_S * (k + 1) / n
                       for k, mid in enumerate(ids)}
        cmd_dps = {mid: 0.0 for mid in ids}
        overruns = 0
        last_line = 0.0
        i = 0
        t0 = time.perf_counter()
        deadline = t0 + slot
        while True:
            now = time.perf_counter()
            t = now - t0
            if t >= duration:
                break
            mid = ids[i % n]
            i += 1
            rr.poll()

            # amplitude ramps in at the start and out at the end
            ramp = min(1.0, t / RAMP_S, max(0.0, (duration - t) / RAMP_S))
            if t >= next_status[mid]:
                rr.send(mid, ct.CMD_STATUS1)
                next_status[mid] += STATUS_PERIOD_S
            else:
                dps = args.amp_dps * ramp * math.sin(
                    2 * math.pi * args.freq * t + phases[mid])
                cmd_dps[mid] = dps
                rr.send(mid, ct.CMD_SPEED,
                        ct.speed_cmd_data(args.iq_limit,
                                          int(dps * param.vel_gain)))

            # accumulate tracking + temp/volt extremes from fresh telemetry
            r = rr.rec[mid]
            if r.speed is not None:
                meas = r.speed / param.vel_state_gain
                track_err2[mid] += (meas - cmd_dps[mid]) ** 2
                track_n[mid] += 1
            if r.temp is not None and r.temp > temp_max.get(mid, -99):
                temp_max[mid] = r.temp
            if r.voltage and r.voltage < volt_min.get(mid, 1e9):
                volt_min[mid] = r.voltage

            if t - last_line > 1.0:
                worst_gap = max(x.max_gap for x in rr.rec.values())
                misses = sum(x.missed for x in rr.rec.values())
                sfails = sum(x.send_fail for x in rr.rec.values())
                print(f"  t={t:5.0f}s  misses={misses}  sfail={sfails}  "
                      f"worst gap={worst_gap*1e3:5.2f}ms  "
                      f"load~{rr.bus_load()*100:3.0f}%  overruns={overruns}",
                      end="\r")
                for m2 in ids:
                    r2 = rr.rec[m2]
                    log.row(t_s=f"{t:.1f}", motor_id=m2,
                            cmd_dps=f"{cmd_dps[m2]:.1f}",
                            meas_dps=("" if r2.speed is None
                                      else f"{r2.speed / param.vel_state_gain:.1f}"),
                            temp=r2.temp, voltage=r2.voltage,
                            error=("" if r2.error is None else f"0x{r2.error:02X}"))
                last_line = t

            if ct.pace(deadline) > 0.0005:
                overruns += 1
            deadline += slot
        rr.finalize()

    except KeyboardInterrupt:
        print("\ninterrupted -- stopping all motors.")
    finally:
        ct.stop_all(bus, ids)
        log.close()

    # -- report ---------------------------------------------------------------
    after = ct.link_snapshot()
    print("\n\n== T5 motion soak results ==")
    rows = []
    for mid in ids:
        r = rr.rec[mid]
        p50, p99, mx = ct.percentiles(r.latencies)
        rms = (math.sqrt(track_err2[mid] / track_n[mid])
               if track_n[mid] else float("nan"))
        rows.append([mid, r.sent, r.missed, r.send_fail,
                     f"{r.max_gap*1e3:.2f}",
                     f"{p99*1e6:.0f}",
                     f"{temp_start.get(mid, '?')}->{temp_max.get(mid, '?')}",
                     f"{volt_min.get(mid, 0):.1f}",
                     f"0x{r.error_ever:02X}",
                     f"{rms:.1f}"])
        if r.missed:
            problems.append(f"motor {mid}: {r.missed} missed replies")
        if r.send_fail:
            problems.append(f"motor {mid}: {r.send_fail} send failures")
        if r.max_gap > gap_limit:
            problems.append(f"motor {mid}: max gap {r.max_gap*1e3:.1f} ms > "
                            f"{gap_limit*1e3:.1f} ms")
        if r.error_ever:
            problems.append(f"motor {mid}: error flags seen "
                            f"0x{r.error_ever:02X} ({ct.decode_errors(r.error_ever)})")
        tmax, tstart = temp_max.get(mid), temp_start.get(mid)
        if tmax is not None and tstart is not None:
            if tmax - tstart > 15:
                problems.append(f"motor {mid}: temp rise {tmax - tstart} C > 15 C")
            if tmax > 60:
                problems.append(f"motor {mid}: temp {tmax} C > 60 C")
        if volt_start.get(mid) and volt_min.get(mid, 0) < 0.9 * volt_start[mid]:
            problems.append(f"motor {mid}: voltage sagged to "
                            f"{volt_min[mid]:.1f} V (<90 % of start)")
    ct.print_table(["id", "sent", "miss", "sfail", "gap ms", "lat p99 us",
                    "temp C", "vmin", "err ever", "track rms dps"], rows)

    lines, link_problems = ct.link_report(before, after)
    print("\nlink stats delta:")
    print("\n".join(lines))
    problems += link_problems

    bus.shutdown()
    print(f"\nCSV: {log.path}")
    raise SystemExit(0 if ct.suite_verdict(problems) else 1)


if __name__ == "__main__":
    main()
