#!/usr/bin/env python3
"""
velocity_unplug_test.py -- staged velocity + unplug watchdog check.

Flow:
  1. Stream torque = 0 (motor limp, no motion). Power on / power-cycle the motor
     now -- the zero-torque stream keeps comms alive so the input-signal-timeout
     fault does not re-latch. The live motor state/error is shown.
  2. Type 'y' + Enter when the motor is powered and ready -> the script clears any
     error, starts the motor, and streams a constant VELOCITY command (it spins).
  3. Physically UNPLUG the CAN cable and watch the motor:
       * watchdog WORKING     -> motor stops within the timeout.
       * watchdog NOT working -> motor keeps spinning (latches the last command).

WHILE UNPLUGGED the host can't talk to the motor: the script prints "SEND FAILED"
(no bus ACK) as the comms-lost marker. To STOP: REPLUG the cable, then Ctrl-C
(cleanup commands torque 0 + motor_stop, which only land once reconnected). If in
doubt, cut motor power.

SAFETY: bench only; output must rotate CONTINUOUSLY in one direction (no wire wrap
/ hard stop). Keep an E-stop in reach. Run in an interactive terminal.

Usage:
  python velocity_unplug_test.py                  # motor id 1, 30 dps
  python velocity_unplug_test.py --speed-dps 45 --dir -1
"""

import argparse
import select
import sys
import time

from motor_library import LKMotor
import motor_gains as param


def wait_for_go(motor, dt) -> bool:
    """Phase 1: stream torque=0 (limp) and show live state until the user types 'y'.

    Returns True when the user typed 'y', or False on EOF (Ctrl-D).
    """
    print("\n[Phase 1] Streaming torque=0 (motor limp). Power on / power-cycle the "
          "motor now.")
    print("          Type 'y' + Enter when it's powered and ready to SPIN "
          "(Ctrl-C to abort).")
    last_show = 0.0
    while True:
        t0 = time.time()
        try:
            motor.send_zero_command()             # keep the motor limp + comms alive
        except Exception:                          # (fire-and-forget: no 0.5 s reply wait)
            pass                                   # motor maybe still off / unplugged

        if t0 - last_show > 0.5:                   # throttled live state readout
            s = motor.read_motor_status_1()
            if s is None:
                print("  waiting... motor not replying (power it on)      ", end="\r")
            else:
                run = "stopped" if s[3] == 0x10 else "running"
                print(f"  state=0x{s[3]:02x} ({run})  error=0x{s[4]:02x}   "
                      f"-> type 'y' to spin   ", end="\r")
            last_show = t0

        # non-blocking check for the 'y' keypress, also paces the loop
        r, _, _ = select.select([sys.stdin], [], [], max(0.0, dt - (time.time() - t0)))
        if r:
            line = sys.stdin.readline()
            if not line:                           # EOF / Ctrl-D
                return False
            if line.strip().lower() == "y":
                return True


def main():
    p = argparse.ArgumentParser(description="Staged velocity + unplug watchdog check.")
    p.add_argument("--motor-id", type=int, default=1)
    p.add_argument("--speed-dps", type=float, default=30.0,
                   help="output speed to hold, deg/s (default 30)")
    p.add_argument("--dir", type=int, choices=[1, -1], default=1,
                   help="spin direction (+1 or -1)")
    p.add_argument("--iq-limit", type=int, default=100,
                   help="current limit for the speed loop, raw (keep modest)")
    p.add_argument("--rate-hz", type=float, default=8000, help="command stream rate")
    args = p.parse_args()

    speed_cmd = int(args.speed_dps * param.vel_gain * args.dir)  # output dps -> LSB
    print(__doc__.split("Usage:")[0])
    print(f"Settings: motor-id={args.motor_id}  speed={args.speed_dps} dps "
          f"(dir {args.dir:+d})  iq-limit={args.iq_limit}  rate={args.rate_hz} Hz")

    motor = LKMotor(motor_id=args.motor_id)
    dt = 1.0 / args.rate_hz
    fail_streak = 0
    # Phase 2 achieved-rate tracking (defined here so the finally summary is safe
    # even if we abort during Phase 1). Timestamps are reset when Phase 2 starts.
    cur_hz = 0.0                 # last computed windowed rate, for live display
    win_count = 0                # iterations in the current display window
    win_start = time.time()      # wall-clock start of the current window
    total_count = 0              # iterations since Phase 2 began (for avg)
    total_start = win_start      # Phase 2 start time (for avg)
    min_hz = float("inf")        # min/max over completed windows
    max_hz = 0.0
    try:
        # ----- Phase 1: torque=0, wait for the user to power on and type 'y' -----
        if not wait_for_go(motor, dt):
            print("\nAborted before spin.")
            return

        # ----- Phase 2: clear error, start motor, stream velocity, unplug test -----
        # Phase 1 streamed zero commands fire-and-forget; the motor's unread replies
        # are now backlogged in the RX queue. Drop them so the velocity readout below
        # reflects the LIVE motor, not a minute of stale "speed 0" frames.
        dropped = motor.flush_rx()
        if dropped:
            print(f"  (flushed {dropped} stale RX frames from phase 1)")
        state = motor.ensure_running()             # clears error + motor_run if stopped
        print(f"\n[Phase 2] motor state now 0x{(state if state is not None else 0):02x}. "
              "Velocity command LIVE -- UNPLUG the CAN cable and watch the motor.")
        print("          Replug then Ctrl-C to stop.\n")
        win_start = total_start = time.time()   # start the rate clock at Phase 2
        while True:
            t0 = time.time()
            try:
                res = motor.speed_loop_control(args.iq_limit, speed_cmd)
                if fail_streak:
                    print(f"\n  [comms back after {fail_streak} failed sends]")
                    fail_streak = 0
            except Exception as e:
                fail_streak += 1
                if fail_streak == 1:
                    print(f"\n  SEND FAILED (CAN unplugged?): {e}")
                elif fail_streak % 50 == 0:
                    print(f"  still failing ({fail_streak} sends)...")
                res = None

            # measure the achieved loop rate over a ~0.5 s rolling window
            win_count += 1
            total_count += 1
            if t0 - win_start >= 0.5:
                cur_hz = win_count / (t0 - win_start)
                min_hz = min(min_hz, cur_hz)
                max_hz = max(max_hz, cur_hz)
                win_count = 0
                win_start = t0

            if res is not None:
                _temp, _iq, spd_raw, _enc = res
                print(f"  cmd {args.speed_dps:+.0f} dps | "
                      f"measured {spd_raw / param.vel_state_gain:+6.1f} dps | "
                      f"{cur_hz:6.0f} Hz", end="\r")
            time.sleep(max(0.0, dt - (time.time() - t0)))

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        if total_count > 0:
            avg_hz = total_count / max(1e-9, time.time() - total_start)
            lo = 0.0 if min_hz == float("inf") else min_hz
            print(f"Loop rate: target {args.rate_hz:.0f} Hz | "
                  f"avg {avg_hz:.0f}  min {lo:.0f}  max {max_hz:.0f} Hz")
        print("Cleanup: torque 0, motor_stop, release.")
        try:
            motor.torque_loop_control(0)
            motor.motor_stop()
        except Exception as e:
            print(f"  cleanup send failed (still unplugged?): {e}")
        motor.motor_release()


if __name__ == "__main__":
    main()
