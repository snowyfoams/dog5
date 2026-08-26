#!/usr/bin/env python3
"""
cantest_common.py -- test-harness helpers for the 12-motor CAN test suite
(t2_ping_scan / t3_rate_sweep / t4_watchdog_trip / t5_motion_soak /
t6_unplug_multi / t7_recover_no_powercycle).

The motor-control primitives that used to live here -- RoundRobinBus,
MotorRecord, arm_motors, stop_all, pace, the CMD_* constants, decode_errors,
speed_cmd_data, i16/i32 -- now live in the clean library `motorbus.py` and are
re-exported below, so existing `import cantest_common as ct; ct.RoundRobinBus`
call sites keep working unchanged. New code should import from `motorbus`
directly.

What remains here is genuinely test-only: SocketCAN link-stat bracketing, CSV
logging, table/verdict printing -- none of it is motor control.
"""
import csv
import json
import os
import subprocess
import time
from typing import Optional

# -- re-exported motor-control primitives (moved to motorbus.py) -------------
from motorbus import (                                        # noqa: F401
    RoundRobinBus, MotorRecord, arm_motors, stop_all, pace,
    decode_errors, ERROR_BITS, speed_cmd_data, i16, i32,
    MOTOR_IDS, REPLY_BASE, CAN_IFACE, FRAME_BITS, BITRATE,
    CMD_STATUS1, CMD_CLEAR, CMD_STATUS2, CMD_SHUTDOWN, CMD_STOP,
    CMD_RUN, CMD_TORQUE, CMD_SPEED,
    open_bus, MotorBus,
)


# ---------------------------------------------------------------------------
# SocketCAN link statistics (ip -j -d -s link show can0)
# ---------------------------------------------------------------------------

def link_snapshot(iface: str = CAN_IFACE) -> Optional[dict]:
    """Snapshot of the CAN link state + error counters, or None if the query
    fails (no interface / no iproute2 JSON support)."""
    try:
        out = subprocess.run(
            ["ip", "-j", "-d", "-s", "link", "show", iface],
            capture_output=True, text=True, timeout=5,
        )
        info = json.loads(out.stdout)[0]
    except Exception:
        return None

    can_info = info.get("linkinfo", {}).get("info_data", {})
    berr = can_info.get("berr_counter", {})
    st64 = info.get("stats64", {})
    rx, tx = st64.get("rx", {}), st64.get("tx", {})
    return {
        "oper_up": info.get("operstate") != "DOWN" and "UP" in info.get("flags", []),
        "can_state": can_info.get("state", "?"),          # ERROR-ACTIVE / -PASSIVE / BUS-OFF / STOPPED
        "bitrate": can_info.get("bittiming", {}).get("bitrate", 0),
        "restart_ms": can_info.get("restart_ms", 0),
        "qlen": info.get("txqlen", 0),
        "berr_tx": berr.get("tx", 0),
        "berr_rx": berr.get("rx", 0),
        "restarts": can_info.get("restarts",
                                 st64.get("restarts", 0)) or 0,
        "rx_packets": rx.get("packets", 0),
        "tx_packets": tx.get("packets", 0),
        "rx_errors": rx.get("errors", 0),
        "tx_errors": tx.get("errors", 0),
        "rx_dropped": rx.get("dropped", 0),
        "tx_dropped": tx.get("dropped", 0),
    }


def link_report(before: Optional[dict], after: Optional[dict]) -> tuple:
    """Compare two snapshots. Returns (lines_to_print, problems). Any entry in
    problems means the bus itself degraded during the test -> test FAIL."""
    if before is None or after is None:
        return (["  link stats unavailable (ip -j failed?)"],
                ["link stats unavailable"])
    lines, problems = [], []
    lines.append(f"  can state   : {before['can_state']} -> {after['can_state']}")
    # "?" = interface exposes no CAN state (vcan) -- not a degradation
    if after["can_state"] not in ("ERROR-ACTIVE", "?"):
        problems.append(f"CAN controller left ERROR-ACTIVE ({after['can_state']})")
    for key, label in (("berr_tx", "berr tx"), ("berr_rx", "berr rx"),
                       ("rx_errors", "rx errors"), ("tx_errors", "tx errors"),
                       ("rx_dropped", "rx dropped"), ("tx_dropped", "tx dropped"),
                       ("restarts", "restarts")):
        d = after[key] - before[key]
        lines.append(f"  {label:<12}: {before[key]} -> {after[key]}  (delta {d:+d})")
        if d > 0:
            problems.append(f"{label} grew by {d}")
    lines.append(f"  frames      : rx +{after['rx_packets'] - before['rx_packets']}"
                 f"  tx +{after['tx_packets'] - before['tx_packets']}")
    return lines, problems


# ---------------------------------------------------------------------------
# Stats
# ---------------------------------------------------------------------------

def percentiles(samples: list) -> tuple:
    """(p50, p99, max) in the samples' own unit; (0,0,0) if empty."""
    if not samples:
        return 0.0, 0.0, 0.0
    import numpy as np
    arr = np.asarray(samples)
    return (float(np.percentile(arr, 50)), float(np.percentile(arr, 99)),
            float(arr.max()))


# ---------------------------------------------------------------------------
# CSV logging
# ---------------------------------------------------------------------------

class CsvLogger:
    def __init__(self, name: str, fieldnames: list):
        os.makedirs("log", exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join("log", f"{name}_{stamp}.csv")
        self._fh = open(self.path, "w", newline="")
        self._writer = csv.DictWriter(self._fh, fieldnames=fieldnames)
        self._writer.writeheader()

    def row(self, **kwargs):
        self._writer.writerow(kwargs)

    def close(self):
        self._fh.flush()
        self._fh.close()


# ---------------------------------------------------------------------------
# Reporting helpers
# ---------------------------------------------------------------------------

def print_table(headers: list, rows: list) -> None:
    widths = [max(len(str(h)), max((len(str(r[i])) for r in rows), default=0))
              for i, h in enumerate(headers)]
    line = "  ".join(str(h).ljust(w) for h, w in zip(headers, widths))
    print(line)
    print("-" * len(line))
    for r in rows:
        print("  ".join(str(c).ljust(w) for c, w in zip(r, widths)))


def print_monitor_hint() -> None:
    print("Run in another terminal to watch the wire:\n"
          f"  candump -td -e {CAN_IFACE},#FFFFFFFF\n"
          f"  canbusload {CAN_IFACE}@{BITRATE} -r\n")


def suite_verdict(problems: list) -> bool:
    """Print PASS/FAIL and the reasons; returns True on pass."""
    if problems:
        print("\nRESULT: FAIL")
        for p in problems:
            print(f"  - {p}")
        return False
    print("\nRESULT: PASS")
    return True
