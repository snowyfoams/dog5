#!/usr/bin/env python3
"""
t7_recover_no_powercycle.py -- can the input-signal-lost latch be cleared over
CAN with the MINIMAL ladder 0x9B (clear) + 0x88 (run) + zero-torque keep-alive,
WITHOUT the 0x80 shutdown step and WITHOUT a power cycle?

Background: when the command stream pauses past the firmware input-timeout
(~10 ms), the motor self-stops and LATCHES status-1 error bit 7 (0x80,
"input signal lost timeout"). The suite's proven recovery ladder is
0x9B -> 0x80 -> 0x88 (see clear_fault_and_restart / arm_motors); the 0x80
shutdown de-energizes the motor and drops any latched command. This test asks
whether the shutdown is actually needed: does 0x9B (clear) immediately followed
by 0x88 (run) and a resumed keep-alive stream recover the motor on its own?

Dropping 0x80 matters because shutdown momentarily kills the output -- under
load that is a drop/jerk. A clear->run recovery keeps the motor energized.

Per cycle, for every selected motor:
  1. ARM      -- zero-torque stream + full ladder, so it starts clean & running
  2. TRIP     -- silence the wire for --gap-ms (default 200, >> 10 ms window)
  3. CONFIRM  -- read status-1; require error bit 0x80 latched (else no trip)
  4. (opt) RESUME-PROBE -- stream bare keep-alive briefly and re-read: did the
                latch clear WITHOUT any 0x9B? (t4 "rung a"). Distinguishes a
                sticky latch from one that self-clears on resumed traffic.
  5. RECOVER  -- the sequence under test: 0x9B -> 0x88 -> zero-torque keep-alive
  6. VERIFY   -- re-read status-1: is 0x80 clear and the motor running?
                (opt --spin-check also commands a gentle move to prove the
                 output actuates, not just that the flag cleared)
  7. (opt) FALLBACK -- if minimal failed, try the full 0x9B->0x80->0x88 ladder
                to A/B whether the 0x80 shutdown is what makes the difference.

PASS = every selected motor's 0x80 latch cleared via the minimal ladder on
every cycle (no power cycle, no 0x80 shutdown needed).

SAFETY: bench only. With --spin-check every motor's output must be free to
rotate continuously (no wire wrap / hard stop). Keep an e-stop in reach.

Usage:
  python t7_recover_no_powercycle.py --ids 1 --cycles 5
  python t7_recover_no_powercycle.py --ids 1-12 --cycles 3 --resume-probe --fallback
  python t7_recover_no_powercycle.py --ids 1 --spin-check --speed-dps 30
"""

import argparse
import time

from motor_library import open_bus
import cantest_common as ct
import motor_gains as param


def parse_ids(spec):
    if "-" in spec:
        lo, hi = spec.split("-")
        return list(range(int(lo), int(hi) + 1))
    return [int(x) for x in spec.split(",")]


def stream_keepalive(rr, ids, seconds, rate_hz):
    """Round-robin zero-torque keep-alive (0xA1, iq=0) to every id for
    `seconds`. Worst-case per-motor gap ~2/rate_hz -- keep rate_hz*... inside
    the 10 ms window (250 Hz/motor => 8 ms)."""
    n = len(ids)
    slot = 1.0 / (rate_hz * n)
    t_end = time.perf_counter() + seconds
    deadline = time.perf_counter() + slot
    i = 0
    while time.perf_counter() < t_end:
        rr.poll()
        rr.send(ids[i % n], ct.CMD_TORQUE)          # 0xA1 + zero data => iq 0
        i += 1
        ct.pace(deadline)
        deadline += slot
    rr.poll()


def batch_status1(rr, ids, timeout=0.1):
    """Status-1 (0x9A) read of EVERY id at once: send to all back-to-back, then
    collect replies. Feeds every motor within ~1.4 ms (12 frames) so a slow
    per-motor sequential read cannot starve later motors past the 10 ms window
    and re-trip them mid-verify.

    Drains stragglers first (a late 0xA1 keep-alive reply would otherwise be
    matched to a status request by arbitration id and read as no-error). error
    is reset so only a real status-1 reply populates it. Returns
    {mid: (error_byte, motor_state)}; (None, None) for a motor that never
    replied within `timeout`."""
    rr.poll()
    for mid in ids:
        rr.rec[mid].error = None
        rr.send(mid, ct.CMD_STATUS1)
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        rr.poll()
        if all(rr.rec[m].pending_t is None for m in ids):
            break
        time.sleep(0.0002)
    return {m: (rr.rec[m].error, rr.rec[m].state) for m in ids}


def batch_status2(rr, ids, timeout=0.1):
    """Status-2 (0x9C) read of every id at once; returns {mid: speed_raw}. Same
    all-at-once feeding as batch_status1 so the reads don't starve any motor."""
    rr.poll()
    for mid in ids:
        rr.rec[mid].speed = None
        rr.send(mid, ct.CMD_STATUS2)
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        rr.poll()
        if all(rr.rec[m].pending_t is None for m in ids):
            break
        time.sleep(0.0002)
    return {m: rr.rec[m].speed for m in ids}


def latched(err):
    return err is not None and bool(err & 0x80)


def minimal_ladder(rr, ids, targets, rate_hz):
    """The sequence under test: 0x9B (clear) then 0x88 (run) on each target,
    with the whole batch bounded well under 10 ms, then hand back to a resumed
    keep-alive stream (the caller streams after this returns).

    Interleaved with a keep-alive round so no motor -- target or not -- sees a
    gap over ~1/rate between the clear and the run."""
    for mid in targets:                             # clear error flags
        rr.send(mid, ct.CMD_CLEAR)                   # 0x9B
    for mid in ids:                                 # feed everyone once
        rr.send(mid, ct.CMD_TORQUE)                  # 0xA1 iq 0
    for mid in targets:                             # enable output
        rr.send(mid, ct.CMD_RUN)                     # 0x88
    rr.poll()


def full_ladder(rr, ids, targets, rate_hz):
    """Reference ladder 0x9B -> 0x80 -> 0x88 (the shutdown variant), same
    interleaving. Used only by --fallback to A/B against the minimal ladder."""
    for mid in targets:
        rr.send(mid, ct.CMD_CLEAR)                    # 0x9B
    for mid in ids:
        rr.send(mid, ct.CMD_TORQUE)
    for mid in targets:
        rr.send(mid, ct.CMD_SHUTDOWN)                 # 0x80
    for mid in ids:
        rr.send(mid, ct.CMD_TORQUE)
    for mid in targets:
        rr.send(mid, ct.CMD_RUN)                      # 0x88
    rr.poll()


def spin_check(rr, ids, targets, rate_hz, speed_data, stop_data, moving_floor):
    """Command a gentle speed on each target for ~1 s and report which ones
    actually moved -- proves the output actuates, not just that 0x80 cleared."""
    dt = 1.0 / (rate_hz * len(ids))
    t_end = time.perf_counter() + 1.0
    deadline = time.perf_counter() + dt
    i = 0
    while time.perf_counter() < t_end:
        rr.poll()
        mid = ids[i % len(ids)]
        rr.send(mid, ct.CMD_SPEED, speed_data if mid in targets else None)
        i += 1
        ct.pace(deadline)
        deadline += dt
    rr.poll()
    speeds = batch_status2(rr, ids)
    moved = {mid for mid in targets if abs(speeds.get(mid) or 0) >= moving_floor}
    # settle to a stop, keep feeding so nothing re-trips
    for _ in range(3):
        for mid in ids:
            rr.send(mid, ct.CMD_SPEED, stop_data if mid in targets else None)
        time.sleep(0.003)
        rr.poll()
    return moved


def main():
    p = argparse.ArgumentParser(
        description="Verify over-CAN recovery from the input-lost latch using "
                    "only 0x9B + 0x88 + keep-alive (no 0x80, no power cycle).")
    p.add_argument("--ids", default="1", help="e.g. 1  |  1,2,5  |  1-12")
    p.add_argument("--cycles", type=int, default=5)
    p.add_argument("--rate-hz", type=float, default=250.0,
                   help="keep-alive rate per motor")
    p.add_argument("--gap-ms", type=float, default=200.0,
                   help="stream silence used to force the trip (>> 10 ms)")
    p.add_argument("--settle-s", type=float, default=0.5,
                   help="keep-alive stream time after the ladder, before verify")
    p.add_argument("--resume-probe", action="store_true",
                   help="first test whether bare keep-alive alone clears the "
                        "latch (t4 rung a) before applying 0x9B+0x88")
    p.add_argument("--fallback", action="store_true",
                   help="if the minimal ladder fails, try the full 0x9B+0x80+0x88 "
                        "ladder to see whether the shutdown step is required")
    p.add_argument("--spin-check", action="store_true",
                   help="also command a gentle move to prove the output "
                        "actuates (output MUST be free to spin)")
    p.add_argument("--speed-dps", type=float, default=30.0)
    p.add_argument("--iq-limit", type=int, default=100)
    args = p.parse_args()

    ids = parse_ids(args.ids)
    speed_data = ct.speed_cmd_data(args.iq_limit,
                                   int(args.speed_dps * param.vel_gain))
    stop_data = ct.speed_cmd_data(args.iq_limit, 0)
    moving_floor = args.speed_dps * param.vel_state_gain * 0.5

    print(__doc__.split("Usage:")[0])
    print(f"Settings: ids {ids}, {args.cycles} cycles, {args.rate_hz:.0f} Hz/motor "
          f"keep-alive, trip gap {args.gap_ms:.0f} ms, settle {args.settle_s:.1f} s"
          + ("  [SPIN-CHECK]" if args.spin_check else ""))
    if args.spin_check:
        print("SPIN-CHECK on: every output must be FREE TO ROTATE continuously.")
    input("Enter to start (motor power may be OFF -- you'll be prompted), "
          "Ctrl-C to abort. ")

    bus = open_bus()
    rr = ct.RoundRobinBus(bus, ids)
    log = ct.CsvLogger("recover_no_powercycle",
                       ["cycle", "motor_id", "err_after_gap", "tripped",
                        "resume_cleared", "minimal_cleared", "state_after",
                        "moved", "fallback_cleared", "verdict"])
    problems = []
    # per-motor tallies across all cycles
    trips = {m: 0 for m in ids}
    min_ok = {m: 0 for m in ids}
    resume_ok = {m: 0 for m in ids}
    needed_full = {m: 0 for m in ids}
    needed_cycle = {m: 0 for m in ids}

    try:
        # bring every motor up clean & running (full ladder, waits for power-on)
        if not ct.arm_motors(rr, rate_hz=max(args.rate_hz, 250.0), timeout_s=None):
            raise SystemExit("[t7] arm failed")
        rr.reset_stats()

        for c in range(1, args.cycles + 1):
            print(f"\n===== cycle {c}/{args.cycles} =====")
            # -- hold the stream a moment so we start each cycle running clean --
            stream_keepalive(rr, ids, 0.3, args.rate_hz)

            # -- TRIP: nothing on the wire for gap_ms ---------------------------
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < args.gap_ms / 1000.0:
                pass                                 # busy wait: wire stays silent

            # -- CONFIRM the latch ---------------------------------------------
            errs = batch_status1(rr, ids)
            for mid in ids:
                err, st = errs[mid]
                tag = ct.decode_errors(err) if err is not None else "no reply"
                print(f"  motor {mid}: after {args.gap_ms:.0f} ms gap "
                      f"err={'--' if err is None else f'0x{err:02X}'} ({tag})"
                      f"  -> {'LATCHED' if latched(err) else 'no trip'}")

            trip_targets = [m for m in ids if latched(errs[m][0])]
            for m in trip_targets:
                trips[m] += 1
            if not trip_targets:
                print("  no motor latched at this gap -- widen --gap-ms; "
                      "skipping recovery this cycle")
                for mid in ids:
                    log.row(cycle=c, motor_id=mid,
                            err_after_gap=("" if errs[mid][0] is None
                                           else f"0x{errs[mid][0]:02X}"),
                            tripped=0, resume_cleared="", minimal_cleared="",
                            state_after="", moved="", fallback_cleared="",
                            verdict="no-trip")
                # re-arm and continue
                minimal_ladder(rr, ids, ids, args.rate_hz)
                stream_keepalive(rr, ids, args.settle_s, args.rate_hz)
                continue

            # -- optional RESUME-PROBE: does bare keep-alive clear it? ----------
            resume_cleared = {}
            if args.resume_probe:
                stream_keepalive(rr, ids, args.settle_s, args.rate_hz)
                probe = batch_status1(rr, ids)
                cleared_now = []
                for mid in trip_targets:
                    resume_cleared[mid] = not latched(probe[mid][0])
                    if resume_cleared[mid]:
                        resume_ok[mid] += 1
                        cleared_now.append(mid)
                        # self-cleared: log it here; it won't reach the ladder
                        log.row(cycle=c, motor_id=mid,
                                err_after_gap=f"0x{errs[mid][0]:02X}", tripped=1,
                                resume_cleared=1, minimal_cleared="",
                                state_after="", moved="", fallback_cleared="",
                                verdict="resume-cleared")
                if cleared_now:
                    print(f"  RESUME-PROBE: latch self-cleared on {cleared_now} "
                          "with keep-alive ONLY (no 0x9B needed)")
                # motors still latched fall through to the ladder below
                trip_targets = [m for m in trip_targets if not resume_cleared[m]]
                if not trip_targets:
                    continue

            # -- RECOVER with the sequence under test: 0x9B -> 0x88 -> keep-alive
            print(f"  applying MINIMAL ladder 0x9B+0x88 to {trip_targets}")
            minimal_ladder(rr, ids, trip_targets, args.rate_hz)
            stream_keepalive(rr, ids, args.settle_s, args.rate_hz)

            # -- VERIFY ---------------------------------------------------------
            min_cleared, state_after = {}, {}
            verify = batch_status1(rr, ids)
            for mid in trip_targets:
                err, st = verify[mid]
                min_cleared[mid] = not latched(err)
                state_after[mid] = st
                if min_cleared[mid]:
                    min_ok[mid] += 1
                print(f"  motor {mid}: after 0x9B+0x88 "
                      f"err={'--' if err is None else f'0x{err:02X}'} "
                      f"state={'--' if st is None else f'0x{st:02X}'}  -> "
                      f"{'RECOVERED' if min_cleared[mid] else 'still latched'}")

            moved = set()
            if args.spin_check:
                ok_ids = [m for m in trip_targets if min_cleared[m]]
                moved = spin_check(rr, ids, ok_ids, args.rate_hz,
                                   speed_data, stop_data, moving_floor)
                for mid in ok_ids:
                    print(f"  motor {mid}: spin-check "
                          f"{'MOVED' if mid in moved else 'did NOT move'}")
                    if mid not in moved:
                        problems.append(f"cycle {c} motor {mid}: 0x80 cleared but "
                                        "output did not actuate on spin-check")

            # -- FALLBACK: try the shutdown ladder on any still-latched motor ---
            fb_cleared = {}
            still = [m for m in trip_targets if not min_cleared[m]]
            if still and args.fallback:
                print(f"  FALLBACK: full 0x9B+0x80+0x88 on {still}")
                full_ladder(rr, ids, still, args.rate_hz)
                stream_keepalive(rr, ids, args.settle_s, args.rate_hz)
                fb = batch_status1(rr, ids)
                for mid in still:
                    fb_cleared[mid] = not latched(fb[mid][0])
                    if fb_cleared[mid]:
                        needed_full[mid] += 1
                    print(f"  motor {mid}: after full ladder "
                          f"-> {'recovered (needed 0x80)' if fb_cleared[mid] else 'STILL latched -- power cycle'}")

            # -- per-motor verdict + log ---------------------------------------
            for mid in trip_targets:
                if min_cleared[mid]:
                    verdict = "minimal-ok"
                elif fb_cleared.get(mid):
                    verdict = "needed-0x80"
                    problems.append(f"cycle {c} motor {mid}: minimal ladder "
                                    "failed; 0x80 shutdown was required")
                else:
                    verdict = "power-cycle"
                    needed_cycle[mid] += 1
                    problems.append(f"cycle {c} motor {mid}: no over-CAN ladder "
                                    "cleared the latch -- power cycle required")
                log.row(cycle=c, motor_id=mid,
                        err_after_gap=f"0x{errs[mid][0]:02X}",
                        tripped=1,
                        resume_cleared=int(resume_cleared.get(mid, 0))
                        if args.resume_probe else "",
                        minimal_cleared=int(min_cleared[mid]),
                        state_after="" if state_after[mid] is None
                        else f"0x{state_after[mid]:02X}",
                        moved=int(mid in moved) if args.spin_check else "",
                        fallback_cleared=int(fb_cleared[mid])
                        if mid in fb_cleared else "",
                        verdict=verdict)

        # -- summary -----------------------------------------------------------
        print("\n== T7 recovery summary ==")
        rows = []
        for mid in ids:
            rows.append([mid, trips[mid], min_ok[mid],
                         resume_ok[mid] if args.resume_probe else "-",
                         needed_full[mid] if args.fallback else "-",
                         needed_cycle[mid]])
        ct.print_table(
            ["id", "trips", "minimal cleared", "resume cleared",
             "needed 0x80", "power-cycle"], rows)

        total_trips = sum(trips.values())
        total_min = sum(min_ok.values())
        if total_trips == 0:
            problems.append("no motor ever tripped -- gap too small; test not "
                            "exercised")
        else:
            print(f"\nminimal ladder (0x9B+0x88, no shutdown) cleared "
                  f"{total_min}/{total_trips} latches over CAN "
                  f"({100.0 * total_min / total_trips:.0f}%).")
            if total_min == total_trips:
                print("=> the 0x80 shutdown is NOT required: clear+run+keep-alive "
                      "recovers the input-lost latch without a power cycle.")

    except KeyboardInterrupt:
        print("\ninterrupted.")
    finally:
        ct.stop_all(bus, ids)
        log.close()
        bus.shutdown()

    print(f"\nCSV: {log.path}")
    raise SystemExit(0 if ct.suite_verdict(problems) else 1)


if __name__ == "__main__":
    main()
