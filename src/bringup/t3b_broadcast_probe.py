#!/usr/bin/env python3
"""
t3b_broadcast_probe.py -- definitive test of the 0x280 multi-motor iq command
(broadcast mode): is it supported, who replies where, and does it FEED THE
10 ms INPUT PROTECTION -- i.e. can it replace per-motor frames and raise the
control rate?

Why it matters: request-reply costs 2 frames per motor per cycle -> 24
frames/cycle for 12 motors -> measured ceiling 300 Hz (t3). One 0x280 frame
carries iq for FOUR motors (4 x int16, motors 1-4). If the firmware also
accepts 0x281 (motors 5-8) and 0x282 (motors 9-12), a full 12-motor command
cycle is 3 frames + up to 12 replies = 15 frames -> 400 Hz at ~72 % bus load.

Phases (all zero-iq -- ZERO torque, no motion):
  1. probe    : send 0x280/0x281/0x282 once each, log every reply and its
                arbitration id (LK-family replies land on 0x240+id; the old
                t3 probe listened on 0x141-0x144 and could miss them).
  2. stream   : arm all motors (clears 0x80 latches, cantest_common.arm_motors)
                then switch to broadcast-ONLY streaming for --stream-seconds,
                with occasional per-motor 0x9A reads. If no motor re-latches
                0x80, broadcast frames feed the input protection -> broadcast
                mode is usable as the primary command path. If they all
                re-latch, broadcast does NOT reset the watchdog and it CANNOT
                replace per-motor commands, whatever the bandwidth math says.

Motors that never answer a broadcast id get no keep-alive during the stream
phase and will show a re-latched 0x80 afterwards -- that is expected bench
state (any idle motor latches), not a failure; they are excluded from the
watchdog verdict.

Usage:
  python t3b_broadcast_probe.py                       # probe only
  python t3b_broadcast_probe.py --stream-seconds 10   # + 400 Hz stream test
  python t3b_broadcast_probe.py --stream-seconds 10 --stream-rate 500
"""

import argparse
import struct
import time

import can

from motor_library import open_bus
import cantest_common as ct

# MEASURED on this firmware (broadcast mode enabled, 2026-07-06): 0x28x are
# three broadcast COMMAND TYPES, not motor groups -- ALL 12 motors reply to
# each, on their normal 0x140+id reply ids:
#   0x280 -> replies echo 0xA1 (torque broadcast)
#   0x281 -> replies echo 0xA2 (speed broadcast; zeros = speed 0, safe)
#   0x282 -> replies echo 0xA3 (POSITION broadcast; zeros = GO TO 0 deg --
#            NEVER probe it with zero data; excluded below)
# Also measured: with broadcast mode enabled the single-motor protocol is
# COMPLETELY DEAD (no 0x9A/0x9C/0x92/0x9B replies) -- no error flags, no
# angles, no fault clear.
BCAST_IDS = (0x280, 0x281)
BCAST_NAMES = {0x280: "torque", 0x281: "speed"}
BCAST_REPLY_BASE = 0x240
STATUS_PERIOD_S = 0.5               # per-motor 0x9A cadence during the stream


def bcast_group(arb: int, ids: list) -> list:
    """Motor ids a broadcast id may address. Measured: every motor replies to
    every 0x28x command type (the manual's ids-1..4 grouping did not hold)."""
    return list(ids)


def classify(msg, ids: list):
    """(kind, motor_id) for a reply frame; kind in {'bcast','single',None}."""
    a = msg.arbitration_id
    if BCAST_REPLY_BASE + 1 <= a <= BCAST_REPLY_BASE + 32:
        m = a - BCAST_REPLY_BASE
        return ("bcast", m) if m in ids else (None, m)
    if ct.REPLY_BASE + 1 <= a <= ct.REPLY_BASE + 32:
        m = a - ct.REPLY_BASE
        return ("single", m) if m in ids else (None, m)
    return (None, None)


def probe(bus, ids, tries=10):
    """Send each broadcast id `tries` times; map arb -> {motor: reply count}."""
    seen = {arb: {} for arb in BCAST_IDS}
    lat = {arb: [] for arb in BCAST_IDS}
    cmd_bytes = set()
    for arb in BCAST_IDS:
        for _ in range(tries):
            while bus.recv(timeout=0.0) is not None:
                pass
            t0 = time.perf_counter()
            bus.send(can.Message(arbitration_id=arb, data=[0x00] * 8,
                                 is_extended_id=False))
            t_end = t0 + 0.05
            while time.perf_counter() < t_end:
                msg = bus.recv(timeout=0.005)
                if msg is None or msg.is_error_frame:
                    continue
                kind, m = classify(msg, ids)
                if kind is None:
                    continue
                seen[arb][m] = seen[arb].get(m, 0) + 1
                lat[arb].append(time.perf_counter() - t0)
                if len(msg.data) == 8:
                    cmd_bytes.add(msg.data[0])
    return seen, lat, cmd_bytes


def armed_probe(bus, rr, ids, tries=10, rate_hz=250.0):
    """Re-probe broadcast while every motor is held OUT of protect mode.

    Some firmware may suppress broadcast replies while the input-timeout
    fault is latched -- and on a bench with a 10 ms window, every idle motor
    is latched, so a cold probe can false-negative. Here a 0xA2 zero-speed
    keep-alive stream runs continuously (feeds the watchdog; replies echo
    0xA2), one broadcast id is injected per cycle, and any reply echoing 0xA1
    -- or arriving on 0x240+id -- can only be a broadcast answer."""
    if not ct.arm_motors(rr, rate_hz=rate_hz, timeout_s=None):
        return {arb: {} for arb in BCAST_IDS}
    n = len(ids)
    slot = 1.0 / (rate_hz * n)
    keepalive = [ct.CMD_SPEED] + ct.speed_cmd_data(100, 0)   # hold speed 0
    seen = {arb: {} for arb in BCAST_IDS}
    sequence = [arb for arb in BCAST_IDS for _ in range(tries)]
    current = None
    deadline = time.perf_counter() + slot
    for k in range(len(sequence) + 1):        # +1 cycle catches stragglers
        for mid in ids:
            bus.send(can.Message(arbitration_id=ct.REPLY_BASE + mid,
                                 data=keepalive, is_extended_id=False))
            while True:
                msg = bus.recv(timeout=0.0)
                if msg is None:
                    break
                if msg.is_error_frame or len(msg.data) != 8 or current is None:
                    continue
                a = msg.arbitration_id
                if BCAST_REPLY_BASE + 1 <= a <= BCAST_REPLY_BASE + 32 \
                        and (a - BCAST_REPLY_BASE) in ids:
                    m = a - BCAST_REPLY_BASE
                elif ct.REPLY_BASE + 1 <= a <= ct.REPLY_BASE + 32 \
                        and msg.data[0] == ct.CMD_TORQUE \
                        and (a - ct.REPLY_BASE) in ids:
                    m = a - ct.REPLY_BASE
                else:
                    continue
                seen[current][m] = seen[current].get(m, 0) + 1
            ct.pace(deadline)
            deadline += slot
        if k < len(sequence):
            current = sequence[k]
            bus.send(can.Message(arbitration_id=current, data=[0x00] * 8,
                                 is_extended_id=False))
    return seen


def main():
    p = argparse.ArgumentParser(
        description="Probe 0x280-family broadcast commands (zero iq, no motion).")
    p.add_argument("--ids", default="1-12")
    p.add_argument("--tries", type=int, default=10)
    p.add_argument("--stream-seconds", type=float, default=0.0,
                   help="after the probe: arm, then broadcast-only stream this "
                        "long to prove the watchdog is fed (0 = skip)")
    p.add_argument("--stream-rate", type=float, default=400.0,
                   help="broadcast cycles per second in the stream phase")
    p.add_argument("--iface", default=ct.CAN_IFACE,
                   help="CAN interface (default can0; vcan0 for bench tests)")
    args = p.parse_args()

    if "-" in args.ids:
        lo, hi = args.ids.split("-")
        ids = list(range(int(lo), int(hi) + 1))
    else:
        ids = [int(x) for x in args.ids.split(",")]

    ct.print_monitor_hint()
    before = ct.link_snapshot(args.iface)
    if args.iface == ct.CAN_IFACE:
        bus = open_bus()
    else:
        bus = can.Bus(interface="socketcan", channel=args.iface)
    problems = []

    try:
        ct.stop_all(bus, ids)
        time.sleep(0.1)
        while bus.recv(timeout=0.0) is not None:
            pass

        rr = ct.RoundRobinBus(bus, ids)

        # -- phase 1: who answers which broadcast id? -----------------------
        print(f"== probe: {[hex(a) for a in BCAST_IDS]} x {args.tries}, "
              "zero iq ==")
        seen, lat, cmd_bytes = probe(bus, ids, args.tries)
        supported = []
        for arb in BCAST_IDS:
            group = bcast_group(arb, ids)
            got = seen[arb]
            if not group:
                continue
            if got:
                p50, p99, mx = ct.percentiles([x * 1e6 for x in lat[arb]])
                print(f"  0x{arb:03X} ({BCAST_NAMES[arb]}): replies from "
                      f"{sorted(got)} x{args.tries} -> "
                      f"lat us p50/p99/max {p50:.0f}/{p99:.0f}/{mx:.0f}")
                missing = [m for m in group if got.get(m, 0) < args.tries]
                if missing:
                    print(f"        incomplete replies from {missing}")
                supported.append(arb)
            else:
                print(f"  0x{arb:03X} ({BCAST_NAMES[arb]}): NO replies")
        if cmd_bytes:
            print(f"  reply command byte(s): {[hex(b) for b in cmd_bytes]}")
        if not supported and args.stream_seconds > 0:
            # rule out protect-mode suppression: every idle motor on a 10 ms
            # window is latched, and latched firmware may ignore 0x28x
            print("\nno cold replies -- re-probing with motors ARMED "
                  "(latches cleared, zero-speed hold, no motion)...")
            seen2 = armed_probe(bus, rr, ids, args.tries)
            for arb in BCAST_IDS:
                group = bcast_group(arb, ids)
                got = seen2[arb]
                if group and got:
                    print(f"  0x{arb:03X} armed (motors {group}): replies "
                          f"from {sorted(got)}")
                    supported.append(arb)
                    seen[arb] = got
            if not supported:
                print("  still nothing while armed.")

        if not supported:
            print("\nverdict: broadcast NOT usable -- no motor answered any "
                  "0x28x id (cold" +
                  (" or armed" if args.stream_seconds > 0 else "") + ").")
            print("  Likely causes on this firmware (all silent):")
            print("  - broadcast mode not enabled per motor in the LK PC "
                  "software (enable, SAVE, power-cycle, re-run this probe)")
            print("  - broadcast only addresses motor ids 1-4: ids 5+ never "
                  "listen on 0x280, and 0x281/0x282 may not exist at all")
            print("  Even if enabled, 4 motors/frame means broadcast cannot "
                  "carry 12 motors on one bus. Realistic 400 Hz routes:")
            print("  - 2 buses x 6 motors, request-reply (proven pattern)")
            print("  - 3 buses x 4 motors with broadcast, torque mode only")
            print("  Otherwise: stay at the measured 300 Hz on one bus.")
            problems.append("no broadcast support")
            return

        covered = sorted({m for a in supported for m in bcast_group(a, ids)
                          if seen[a].get(m)})
        print(f"\nbroadcast-covered motors: {covered} "
              f"({len(supported)} cmd frame(s)/cycle)")

        # -- phase 2: does broadcast feed the input protection? -------------
        if args.stream_seconds <= 0:
            print("\nrun again with --stream-seconds 10 to prove the "
                  "watchdog accepts broadcast as input signal (the decisive "
                  "half of this test).")
            return

        # stream torque-zeros only: 0x281 zeros = active speed-0 hold, and a
        # watchdog test must not depend on two command types at once
        stream_arbs = [0x280] if 0x280 in supported else supported[:1]
        print(f"\n== stream: arming, then broadcast-only "
              f"({[hex(a) for a in stream_arbs]}) at "
              f"{args.stream_rate:.0f} Hz for {args.stream_seconds:.0f} s ==")
        # arming needs the single-motor protocol, which broadcast mode kills
        # entirely -- probe it first instead of hanging in arm_motors
        rr.send(ids[0], ct.CMD_STATUS1)
        if rr.wait_reply(ids[0], 0.05):
            if not ct.arm_motors(rr, rate_hz=250.0, timeout_s=None):
                raise SystemExit("arm failed")
        else:
            print("single-motor protocol DEAD (broadcast mode active): "
                  "cannot clear latches or read error flags over CAN. "
                  "Streaming anyway to measure reply sustainability only.")

        cycle = 1.0 / args.stream_rate
        n_cyc = 0
        bc_cnt = {m: 0 for m in ids}
        bc_last = {m: None for m in ids}
        bc_gap = {m: 0.0 for m in ids}
        err_seen = {m: None for m in ids}
        status_next = {m: STATUS_PERIOD_S * (k + 1) / len(ids)
                       for k, m in enumerate(ids)}
        send_fail = 0
        overruns = 0

        def drain(linger: float):
            """Classify replies; linger > 0 waits out in-flight stragglers."""
            t_stop = time.perf_counter() + linger
            while True:
                msg = bus.recv(timeout=0.005 if linger else 0.0)
                if msg is None:
                    if time.perf_counter() >= t_stop:
                        return
                    continue
                if msg.is_error_frame or len(msg.data) != 8:
                    continue
                kind, m = classify(msg, ids)
                rx_t = time.perf_counter()
                if kind == "bcast":
                    bc_cnt[m] += 1
                    if bc_last[m] is not None:
                        bc_gap[m] = max(bc_gap[m], rx_t - bc_last[m])
                    bc_last[m] = rx_t
                elif kind == "single" and msg.data[0] in (ct.CMD_STATUS1,
                                                          ct.CMD_CLEAR):
                    err_seen[m] = msg.data[7]

        t0 = time.perf_counter()
        deadline = t0 + cycle
        while True:
            now = time.perf_counter()
            t = now - t0
            if t >= args.stream_seconds:
                break
            for arb in stream_arbs:
                try:
                    bus.send(can.Message(arbitration_id=arb, data=[0x00] * 8,
                                         is_extended_id=False))
                except (can.CanOperationError, OSError):
                    send_fail += 1
            n_cyc += 1
            for m in ids:                      # sparse per-motor status reads
                if t >= status_next[m]:
                    try:
                        bus.send(can.Message(
                            arbitration_id=ct.REPLY_BASE + m,
                            data=[ct.CMD_STATUS1] + [0x00] * 7,
                            is_extended_id=False))
                    except (can.CanOperationError, OSError):
                        send_fail += 1
                    status_next[m] += STATUS_PERIOD_S
            drain(0.0)
            if ct.pace(deadline) > 0:
                overruns += 1
            deadline += cycle
        drain(0.05)                    # catch the last cycle's replies
        ct.stop_all(bus, ids)

        # -- report ----------------------------------------------------------
        print(f"\ncycles sent: {n_cyc}  ({args.stream_rate:.0f}/s x "
              f"{len(supported)} frame(s))  send fails: {send_fail}  "
              f"pace overruns: {overruns}")
        rows = []
        relatched = []
        for m in ids:
            in_bc = m in covered
            miss = n_cyc - bc_cnt[m]
            err = err_seen[m]
            err_s = f"0x{err:02X}" if err is not None else "?"
            note = ""
            if not in_bc:
                note = "not broadcast-covered (0x80 after stream is normal)"
            elif err is not None and err & 0x80:
                note = "RE-LATCHED DURING BROADCAST STREAM"
                relatched.append(m)
            rows.append([m, bc_cnt[m], miss if in_bc else "-",
                         f"{bc_gap[m]*1000:.1f}" if bc_last[m] else "-",
                         err_s, note])
        ct.print_table(["id", "bc replies", "miss", "max gap ms",
                        "err", "note"], rows)

        judged = [m for m in covered if err_seen[m] is not None]
        print("\n== verdict ==")
        if not judged:
            print("no error-flag telemetry: the single-motor protocol is "
                  "disabled while broadcast mode is on, so the 0x80 latch "
                  "cannot be read over CAN. Broadcast replies were "
                  "sustained, but latch state is UNVERIFIABLE in this mode.")
            problems.append("latch state unverifiable in broadcast mode")
        elif relatched:
            print(f"broadcast does NOT feed the input protection on "
                  f"{relatched} -- 0x28x cannot replace per-motor commands "
                  "for a 10 ms window. Stick to request-reply <= 300 Hz or "
                  "split buses.")
            problems.append("watchdog not fed by broadcast")
        else:
            miss_bad = [m for m in covered if n_cyc - bc_cnt[m] > 0]
            print(f"broadcast FEEDS the input protection (no 0x80 re-latch "
                  f"on {judged} across {args.stream_seconds:.0f} s at "
                  f"{args.stream_rate:.0f} Hz).")
            if miss_bad:
                print(f"but replies were missed on {miss_bad} -- "
                      "check bus load before adopting.")
                problems.append("broadcast replies missed")
            frames = args.stream_rate * (len(supported) + len(covered))
            print(f"projected full-rate load: {len(supported)} cmd + "
                  f"{len(covered)} replies per cycle = "
                  f"{frames:.0f} f/s ~ "
                  f"{frames * ct.FRAME_BITS / ct.BITRATE * 100:.0f} % bus")
    finally:
        ct.stop_all(bus, ids)
        after = ct.link_snapshot(args.iface)
        lines, link_problems = ct.link_report(before, after)
        print("\nlink stats delta:")
        print("\n".join(lines))
        problems += link_problems
        ct.suite_verdict(problems)
        bus.shutdown()


if __name__ == "__main__":
    main()
