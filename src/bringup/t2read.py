#!/usr/bin/env python3
"""
t2read.py -- discover which motor ids are actually on the bus and dump each
motor's full decoded state (no motion).

Why: t2_ping_scan assumes ids are already assigned 1..12. When they are not
(new motors, factory ids, half-configured rig) it reports 12 dead motors and
tells you nothing. This tool scans a WIDE id range (default 1..32), reports
who answers, flags duplicate ids (two replies to one request), then reads and
decodes everything a motor will tell us without moving:

  status-1  0x9A  temp / bus voltage / bus current / state / error flags
  status-2  0x9C  temp / iq / speed / encoder
  status-3  0x9D  phase A/B/C currents
  encoder   0x90  value / raw / offset
  angles    0x92 multi-turn, 0x94 single-turn

Read-only: commands no motion. Motors only need power.

Usage:
  python t2read.py                     # scan ids 1-32, dump every responder
  python t2read.py --ids 1-12          # scan only the expected range
  python t2read.py --id 3              # one motor, full dump
  python t2read.py --watch 1           # re-read found motors every 1 s
                                       # (leave running while fixing wiring)
"""

import argparse
import struct
import time

import can

from motor_library import open_bus
import cantest_common as ct
import motor_gains as param

SCAN_TIMEOUT_S = 0.02     # per-attempt reply deadline during the id scan
READ_TIMEOUT_S = 0.05     # per-command deadline for the state dump
SCAN_RETRIES = 3
DUP_LISTEN_S = 0.005      # after a reply, listen this long for a duplicate


def parse_ids(spec: str) -> list:
    if "-" in spec:
        lo, hi = spec.split("-")
        return list(range(int(lo), int(hi) + 1))
    return [int(x) for x in spec.split(",")]


def i16(data, off) -> int:
    return struct.unpack_from("<h", bytes(data), off)[0]


def u16(data, off) -> int:
    return struct.unpack_from("<H", bytes(data), off)[0]


class Prober:
    """One request in flight at a time; matches replies by arbitration id AND
    echoed command byte, so a stale frame can never masquerade as an answer."""

    def __init__(self, bus: "can.BusABC"):
        self.bus = bus
        self.foreign = []          # (arb_id, data) frames nobody asked for

    def flush(self):
        while self.bus.recv(timeout=0.0) is not None:
            pass

    def ask(self, mid: int, cmd: int, timeout: float = READ_TIMEOUT_S,
            retries: int = 1) -> tuple:
        """Returns (data bytes or None, duplicate_count)."""
        arb = ct.REPLY_BASE + mid
        for _ in range(retries):
            try:
                self.bus.send(can.Message(arbitration_id=arb,
                                          data=[cmd] + [0x00] * 7,
                                          is_extended_id=False))
            except (can.CanOperationError, OSError):
                continue
            reply = self._wait(arb, cmd, timeout)
            if reply is not None:
                return reply, self._count_duplicates(arb, cmd)
        return None, 0

    def _wait(self, arb: int, cmd: int, timeout: float):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            msg = self.bus.recv(timeout=0.002)
            # No is_rx filter: on vcan every frame is flagged locally-generated
            # (is_rx False), and on real hardware our own frames never come
            # back (receive_own_messages defaults to False).
            if msg is None or msg.is_error_frame:
                continue
            if msg.arbitration_id == arb and len(msg.data) == 8 \
                    and msg.data[0] == cmd:
                return msg.data
            self.foreign.append((msg.arbitration_id, bytes(msg.data)))
        return None

    def _count_duplicates(self, arb: int, cmd: int) -> int:
        """A SECOND reply to one request = two motors share this id."""
        dups = 0
        deadline = time.perf_counter() + DUP_LISTEN_S
        while time.perf_counter() < deadline:
            msg = self.bus.recv(timeout=0.001)
            if msg is None or msg.is_error_frame:
                continue
            if msg.arbitration_id == arb and msg.data[0] == cmd:
                dups += 1
            else:
                self.foreign.append((msg.arbitration_id, bytes(msg.data)))
        return dups


def scan(pr: Prober, ids: list) -> tuple:
    """Ping every id with 0x9A; returns (found ids, {id: duplicate count})."""
    found, dups = [], {}
    for mid in ids:
        data, d = pr.ask(mid, ct.CMD_STATUS1, SCAN_TIMEOUT_S, SCAN_RETRIES)
        if data is not None:
            found.append(mid)
            if d:
                dups[mid] = d
    return found, dups


def state_word(state) -> str:
    if state is None:
        return "?"
    return {0x00: "running", 0x10: "stopped"}.get(state, f"0x{state:02X}?")


def dump_motor(pr: Prober, mid: int) -> dict:
    """Full decoded state dump for one motor; returns summary fields."""
    out = {"id": mid}
    print(f"\n=== motor id {mid} (0x{ct.REPLY_BASE + mid:03X}) ===")

    s1, _ = pr.ask(mid, ct.CMD_STATUS1, retries=2)
    if s1 is None:
        print("  status-1 : NO REPLY")
    else:
        temp = struct.unpack_from("<b", bytes(s1), 1)[0]
        volt, curr = i16(s1, 2) * 0.01, i16(s1, 4) * 0.01
        state, err = s1[6], s1[7]
        out.update(temp=temp, volt=volt, state=state, err=err)
        print(f"  status-1 : temp {temp} C   bus {volt:.2f} V  {curr:+.2f} A   "
              f"state {state_word(state)}")
        print(f"  errors   : 0x{err:02X} ({ct.decode_errors(err)})")

    s2, _ = pr.ask(mid, ct.CMD_STATUS2, retries=2)
    if s2 is None:
        print("  status-2 : NO REPLY")
    else:
        iq, spd, enc = i16(s2, 2), i16(s2, 4), u16(s2, 6)
        out.update(iq=iq, speed=spd, encoder=enc)
        print(f"  status-2 : iq {iq} raw ({iq / param.torque_gain:+.3f} N.m)   "
              f"speed {spd} raw ({spd / param.vel_state_gain:+.1f} dps out)   "
              f"encoder {enc}")

    s3, _ = pr.ask(mid, 0x9D, retries=2)
    if s3 is not None:
        print(f"  phase    : A {i16(s3, 2)}  B {i16(s3, 4)}  C {i16(s3, 6)} raw")

    e, _ = pr.ask(mid, 0x90, retries=2)
    if e is not None:
        print(f"  encoder  : value {u16(e, 2)}  raw {u16(e, 4)}  "
              f"offset {u16(e, 6)}   "
              f"({u16(e, 2) * param.encoder_gain:.2f} deg motor-side)")

    a, _ = pr.ask(mid, 0x92, retries=2)
    if a is not None:
        raw = int.from_bytes(bytes(a[1:8]), "little", signed=True)
        out["angle_deg"] = raw / param.pos_gain
        print(f"  angle    : multi-turn {raw} raw 0.01deg "
              f"({raw / param.pos_gain:+.2f} deg output)")

    st, _ = pr.ask(mid, 0x94, retries=2)
    if st is not None:
        raw = struct.unpack_from("<I", bytes(st), 4)[0]
        print(f"  angle    : single-turn {raw * 0.01:.2f} deg motor-side")

    return out


def watch(pr: Prober, ids: list, period: float):
    """Compact repeating state table -- leave running while fixing the rig."""
    print(f"\nwatch mode: re-reading every {period:g} s, Ctrl-C to stop")
    while True:
        rows = []
        for mid in ids:
            s1, _ = pr.ask(mid, ct.CMD_STATUS1)
            s2, _ = pr.ask(mid, ct.CMD_STATUS2)
            a, _ = pr.ask(mid, 0x92)
            if s1 is None and s2 is None:
                rows.append([mid, "NO REPLY", "", "", "", "", ""])
                continue
            temp = struct.unpack_from("<b", bytes(s1), 1)[0] if s1 else "?"
            volt = f"{i16(s1, 2) * 0.01:.1f}" if s1 else "?"
            errs = ct.decode_errors(s1[7]) if s1 else "?"
            st = state_word(s1[6]) if s1 else "?"
            spd = f"{i16(s2, 4) / param.vel_state_gain:+.1f}" if s2 else "?"
            ang = (f"{int.from_bytes(bytes(a[1:8]), 'little', signed=True) / param.pos_gain:+.2f}"
                   if a else "?")
            rows.append([mid, st, f"{temp}", volt, spd, ang, errs])
        print(f"\n-- {time.strftime('%H:%M:%S')} --")
        ct.print_table(["id", "state", "temp C", "volt V", "dps", "deg", "errors"],
                       rows)
        time.sleep(period)


def main():
    p = argparse.ArgumentParser(
        description="Discover motor ids and read decoded state (no motion).")
    p.add_argument("--ids", default="1-32",
                   help="id range to scan, e.g. 1-32 or 1,2,5 (default 1-32)")
    p.add_argument("--id", type=int, default=None,
                   help="skip the scan, dump this single motor id")
    p.add_argument("--watch", type=float, default=None, metavar="SECONDS",
                   help="after the dump, keep re-reading at this period")
    p.add_argument("--iface", default=ct.CAN_IFACE,
                   help="CAN interface (default can0; vcan0 for bench tests)")
    args = p.parse_args()

    ct.print_monitor_hint()
    if args.iface == ct.CAN_IFACE:
        bus = open_bus()
    else:
        bus = can.Bus(interface="socketcan", channel=args.iface)
    pr = Prober(bus)
    pr.flush()

    try:
        if args.id is not None:
            found, dups = [args.id], {}
        else:
            ids = parse_ids(args.ids)
            print(f"== scanning ids {ids[0]}..{ids[-1]} "
                  f"({SCAN_RETRIES} tries each) ==")
            found, dups = scan(pr, ids)
            if found:
                print(f"  responding : {found}")
            else:
                print("  responding : NONE")
                print("  -> nothing answered a 0x9A status read at "
                      f"0x{ct.REPLY_BASE + ids[0]:03X}..0x{ct.REPLY_BASE + ids[-1]:03X}.")
                print("     Check motor power first; run candump in another "
                      "terminal and power-cycle the motors.")
            for mid, d in dups.items():
                print(f"  WARNING: id {mid} answered {1 + d} times -- "
                      f"{1 + d} motors share this id!")

        for mid in found:
            dump_motor(pr, mid)

        if pr.foreign:
            seen = sorted({arb for arb, _ in pr.foreign})
            print(f"\nNOTE: {len(pr.foreign)} unsolicited frame(s) from "
                  f"id(s) {[hex(x) for x in seen]} -- unexpected node or "
                  "duplicate id.")

        if args.watch is not None and found:
            watch(pr, found, args.watch)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
