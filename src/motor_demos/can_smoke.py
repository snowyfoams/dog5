#!/usr/bin/env python3
"""CAN health check: open the bus and ping motors, print their status.

Works on macOS (PEAK PCAN-USB or CANable/gs_usb, no sudo) and Linux (SocketCAN):

    python3 can_smoke.py            # ping motors 1 and 2
    python3 can_smoke.py 1 2 3      # ping a custom list of motor ids
"""
import sys
import time

import can

from motor_library import open_bus


def main(ids) -> None:
    bus = open_bus(bitrate=1_000_000)
    print("BUS OPEN ->", bus.channel_info)
    try:
        for mid in ids:
            bus.send(
                can.Message(
                    arbitration_id=0x140 + mid,
                    data=[0x9A, 0, 0, 0, 0, 0, 0, 0],
                    is_extended_id=False,
                )
            )
        print(f"sent status request (0x9A) to motors {ids}; listening 1.0 s ...")

        replies = {}
        t = time.time()
        while time.time() - t < 1.0:
            m = bus.recv(timeout=0.1)
            if m is None or not m.is_rx:
                continue  # skip our own TX echo (gs_usb loops sent frames back)
            mid = m.arbitration_id - 0x140
            replies[mid] = m.data
            temp = m.data[1]
            volt = int.from_bytes(m.data[2:4], "little") * 0.01
            err = m.data[7]
            print(f"  motor {mid}: temp={temp} C  volt={volt:.1f} V  error=0x{err:02x}")

        if replies:
            print("MOTORS RESPONDING:", sorted(replies))
        else:
            print("NO motor replies (check power / wiring / bitrate / motor IDs)")
    finally:
        bus.shutdown()
        print("SHUTDOWN OK")


if __name__ == "__main__":
    ids = [int(a) for a in sys.argv[1:]] or [1, 5]
    main(ids)
