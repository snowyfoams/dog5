#!/usr/bin/env python3
"""A ring buffer, an HTTP server and a URL printer.  No robot in it anywhere.

WHAT THIS IS FOR
    The hardware runners can stream a live page to a phone or a laptop on the
    same network (`trot_hw.py --web`), so that whoever is holding the robot
    can see roll and pitch without leaning over the terminal.  This module is
    the half of that which has nothing to do with DOG5: a sequence-numbered
    ring buffer, a threading HTTP server that answers on both IP stacks, and
    the code that works out which URL to print.

    The half that DOES know about the robot -- what a sample is, what the page
    shows -- stays in the caller.  `dog5_trot_quasi_static_model/att_web.py`
    is the one caller today: it brings its own sampler thread and its own HTML.

WHY IT IS A SEPARATE FILE
    It used to live inside the EKF's own dashboard, which meant that turning
    on `--web` imported the whole state estimator -- an estimator that has
    never run in the control loop.  Splitting it here is what let that code
    leave the repository without taking the dashboard with it.

THE SEQUENCE NUMBER IS THE WHOLE PROTOCOL
    The browser asks for `/data?since=N` and gets back every row from N
    onwards plus the next N to ask for.  No sessions, no websockets, and a
    client that misses a poll catches up on the next one.  `capacity` bounds
    the buffer, and `since()` returns what survives -- a client that has been
    asleep longer than HISTORY_S simply sees a gap, which is the right
    failure for telemetry.

    `status` is different: it is a small dict REBOUND whole by the producer,
    never appended to, so it always reads as the latest value of everything
    that has no history worth keeping.

NOTHING HERE MAY RAISE INTO THE CONTROL LOOP
    The server runs on daemon threads and the caller's sampler is expected to
    swallow its own exceptions.  A dashboard that can take the robot down is
    worse than no dashboard.
"""
from __future__ import annotations

import os, sys                                                  # noqa: E401
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dog5_paths  # noqa: E402,F401  -- every src/ dir onto sys.path

import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DEFAULT_PORT = 8080
DEFAULT_SAMPLE_HZ = 20.0
HISTORY_S = 120.0


class Telemetry:
    """Sequence-numbered ring buffer of sample rows, plus a status dict."""

    def __init__(self, capacity=int(HISTORY_S * DEFAULT_SAMPLE_HZ)):
        self.capacity = int(capacity)
        self._rows = []
        self._first_seq = 0          # seq of self._rows[0]
        self._lock = threading.Lock()
        self.status = {}

    def append(self, row):
        with self._lock:
            self._rows.append(row)
            if len(self._rows) > self.capacity:
                drop = len(self._rows) - self.capacity
                del self._rows[:drop]
                self._first_seq += drop

    def since(self, seq):
        """Rows with sequence >= seq, the next sequence number, and status."""
        with self._lock:
            start = max(0, int(seq) - self._first_seq)
            rows = self._rows[start:]
            return rows, self._first_seq + len(self._rows), dict(self.status)

    def extend(self, rows):
        with self._lock:
            self._rows.extend(rows)
            self._first_seq = 0
            self.capacity = max(self.capacity, len(self._rows))


def make_handler(tel, page):
    """An HTTP handler serving `page` at / and `tel` as JSON at /data."""

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):      # keep the runner's stdout clean
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path.startswith("/data"):
                seq = 0
                if "?" in self.path:
                    qs = self.path.split("?", 1)[1]
                    for part in qs.split("&"):
                        if part.startswith("since="):
                            try:
                                seq = int(part[6:])
                            except ValueError:
                                seq = 0
                rows, next_seq, status = tel.since(seq)
                body = json.dumps({"seq": next_seq, "rows": rows,
                                   "status": status}).encode()
                self._send(body, "application/json")
            elif self.path in ("/", "/index.html"):
                self._send(page.encode(), "text/html; charset=utf-8")
            else:
                self.send_response(404)
                self.send_header("Content-Length", "0")
                self.end_headers()
    return Handler


class DualStackServer(ThreadingHTTPServer):
    """Bind :: with V6ONLY off so IPv4 and IPv6 clients both reach it."""
    daemon_threads = True
    allow_reuse_address = True
    address_family = socket.AF_INET6

    def server_bind(self):
        try:
            self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        except OSError:
            pass
        super().server_bind()


def urls(port):
    """Reachable URLs: every non-loopback IPv4 on the box, plus the mDNS name.

    Link-local IPv6 is deliberately omitted -- it needs a scope id
    (http://[fe80::1%eth0]:8080) which browsers handle badly.  `ip -br -4 addr`
    is used because getaddrinfo(hostname) misses secondary addresses, and it is
    read once at startup, never in the control path.
    """
    out = []
    try:
        import subprocess                                       # noqa: PLC0415
        txt = subprocess.run(["ip", "-br", "-4", "addr", "show"],
                             capture_output=True, text=True, timeout=2).stdout
        for ln in txt.splitlines():
            parts = ln.split()
            if not parts or parts[0].startswith("lo"):
                continue
            for tok in parts[2:]:
                ip = tok.split("/")[0]
                if ip and not ip.startswith("127."):
                    out.append(f"http://{ip}:{port}/")
    except Exception:
        pass
    out.append(f"http://{socket.gethostname()}.local:{port}/")
    return out


def self_test():
    from selftest_common import check, report                   # noqa: PLC0415

    tel = Telemetry(capacity=4)
    for i in range(3):
        tel.append([i])
    rows, nxt, _ = tel.since(0)
    check("since(0) returns everything appended so far",
          rows == [[0], [1], [2]] and nxt == 3, f"{rows}, next={nxt}")

    rows, nxt, _ = tel.since(2)
    check("...and since(n) returns only what the client has not seen",
          rows == [[2]] and nxt == 3, f"{rows}, next={nxt}")

    for i in range(3, 7):
        tel.append([i])
    rows, nxt, _ = tel.since(0)
    check("the ring drops the oldest rows and the sequence keeps counting",
          rows == [[3], [4], [5], [6]] and nxt == 7, f"{rows}, next={nxt}")
    check("...so a client asleep past the window sees a gap, not stale rows",
          tel.since(1)[0] == [[3], [4], [5], [6]])

    tel.status = {"stage": "TROT"}
    check("status is rebound whole, and read back with every poll",
          tel.since(7)[2] == {"stage": "TROT"})
    check("...and the copy handed out cannot mutate the producer's dict",
          (tel.since(7)[2].__setitem__("stage", "X") or tel.status["stage"])
          == "TROT")

    check("urls() always offers the mDNS name, with no network at all",
          any(h.endswith(".local:8080/") for h in urls(8080)), str(urls(8080)))

    return report()


if __name__ == "__main__":
    raise SystemExit(self_test())
