#!/usr/bin/env bash
# t1_preflight.sh -- bring up and verify can0 for the 12-motor test suite.
#
# Configures: 1 Mbit/s, restart-ms 100 (auto-recover from bus-off; a conscious
# choice -- it changes bus-off behaviour for everything on this interface),
# txqueuelen 1000 (default 10 overflows instantly at 12-motor rates).
#
# Usage:  bash t1_preflight.sh [iface]     (default can0)

set -u
IFACE="${1:-can0}"
BITRATE=1000000
QLEN=1000
RESTART_MS=100
FAIL=0

echo "== T1 preflight: $IFACE =="

if [ ! -d "/sys/class/net/$IFACE" ]; then
    echo "FAIL: $IFACE does not exist. Is the PCAN-USB adapter plugged in?"
    echo "      dmesg | grep -i peak_usb   should show 'attached to PCAN-USB'."
    exit 1
fi

echo "-- configuring link (needs sudo)..."
sudo ip link set "$IFACE" down 2>/dev/null
sudo ip link set "$IFACE" up type can bitrate $BITRATE restart-ms $RESTART_MS || {
    echo "FAIL: could not bring $IFACE up"; exit 1; }
sudo ip link set "$IFACE" txqueuelen $QLEN

DETAILS=$(ip -d -s link show "$IFACE")
echo "$DETAILS"
echo

check() {  # check <label> <grep-pattern>
    if echo "$DETAILS" | grep -q "$2"; then
        echo "  OK   $1"
    else
        echo "  FAIL $1   (expected: $2)"
        FAIL=1
    fi
}

check "interface UP"            "state UP\|<.*UP.*>"
check "CAN state ERROR-ACTIVE"  "can state ERROR-ACTIVE"
check "bitrate $BITRATE"        "bitrate $BITRATE"
check "txqueuelen $QLEN"        "qlen $QLEN"
check "restart-ms $RESTART_MS"  "restart-ms $RESTART_MS"

BERR=$(echo "$DETAILS" | grep -o "berr-counter tx [0-9]* rx [0-9]*" || true)
if [ -n "$BERR" ]; then
    TXE=$(echo "$BERR" | awk '{print $3}')
    RXE=$(echo "$BERR" | awk '{print $5}')
    if [ "$TXE" -eq 0 ] && [ "$RXE" -eq 0 ]; then
        echo "  OK   error counters zero ($BERR)"
    else
        echo "  WARN error counters nonzero ($BERR) -- wiring/termination suspect."
        echo "       Re-run this script (down/up resets them) and see if they grow."
        FAIL=1
    fi
fi

cat <<'EOF'

-- termination check (do this ONCE, by hand, POWER OFF):
   Power everything off, unplug the PCAN adapter, then measure the resistance
   between CAN_H and CAN_L anywhere on the harness:
     ~60 ohm  = correct (two 120-ohm terminators, one at each physical END)
     ~120 ohm = one terminator missing
     <50 ohm  = too many terminators
   With 12 drops on one bus at 1 Mbit/s, keep stubs short (< ~30 cm each) and
   terminate the two harness ENDS only -- never mid-bus.

-- persistent bring-up (optional, recommended):
   Create /etc/udev/rules.d/90-can0.rules with:
     ACTION=="add", SUBSYSTEM=="net", KERNEL=="can0", \
       RUN+="/bin/sh -c '/sbin/ip link set can0 txqueuelen 1000; /sbin/ip link set can0 up type can bitrate 1000000 restart-ms 100'"
   then: sudo udevadm control --reload
EOF

if [ "$FAIL" -eq 0 ]; then
    echo "== T1 RESULT: PASS =="
else
    echo "== T1 RESULT: FAIL (see items above) =="
fi
exit $FAIL
