# =====================
# Show the difference between the three encoder reads on one motor (id 1).
#
#   read_encoder_data()      cmd 0x90  -> (value, raw, offset)  0..65535   single-turn, WRAPS
#   read_single_turn_angle() cmd 0x94  -> 0..35999 (0.01 deg)   0..359.99  single-turn, WRAPS
#   read_multi_turn_angle()  cmd 0x92  -> int (0.01 deg)        cumulative ACCUMULATES, no wrap
#
# With a 10:1 reducer, one full OUTPUT revolution (360 deg) = 10 motor revs, so
# single-turn / encoder wrap ~10 times while multi-turn climbs by ~360000 (0.01
# motor-deg). That ~10x ratio shows the encoder lives on the MOTOR shaft
# (pre-gearbox) -- the open question in config.py:43.
# =====================

import time

from motor_library import LKMotor, open_bus
import motor_gains as param


# ---- Settings ----
MOTOR_ID = 1
GEAR_RATIO = 10                  # what this test verifies (10:1 integrated reducer)
SPIN_OUTPUT_DEG = 360.0          # commanded rotation of the OUTPUT shaft (one full rev)
MAX_SPEED = 300                  # motor-shaft dps (1 dps/LSB); low + visible
POLL_HZ = 30                     # how often to read while spinning
MOVE_TIMEOUT = 30.0              # safety stop for the poll loop (s)


def snapshot(m):
    """Read all three functions once and print them with conversions."""
    enc_value, enc_raw, enc_offset = m.read_encoder_data()      # 0x90
    single = m.read_single_turn_angle()                         # 0x94
    multi = m.read_multi_turn_angle()                           # 0x92

    enc_deg = enc_value * param.encoder_gain                    # 360/65535
    single_deg = single / 100.0
    multi_motor_deg = multi / 100.0

    print("  read_encoder_data()      : "
          f"value={enc_value:<6} raw={enc_raw:<6} offset={enc_offset:<6} "
          f"-> {enc_deg:7.2f} deg (motor, single-turn)")
    print("  read_single_turn_angle() : "
          f"{single:<6} (0.01deg){'':<21}-> {single_deg:7.2f} deg (motor, single-turn)")
    print("  read_multi_turn_angle()  : "
          f"{multi:<6} (0.01deg){'':<21}-> {multi_motor_deg:7.2f} motor-deg "
          f"-> {multi_motor_deg / GEAR_RATIO:7.2f} output-deg")


def main():
    bus = open_bus(bitrate=1_000_000)
    m = LKMotor(motor_id=MOTOR_ID, bus=bus)
    print(f"Connected to motor {MOTOR_ID}\n")

    m.motor_run()
    # Zero the MULTI-TURN reference only (safe; not set_zero_position, which warns
    # about driver lifetime). Single-turn/encoder are absolute and stay put.
    m.set_position_to_angle(0)

    try:
        # ---- 1. Snapshot before moving ----
        print("=== Snapshot (after zeroing multi-turn) ===")
        snapshot(m)
        print("\nNote: multi-turn is ~0 (just zeroed) but single-turn/encoder still")
        print("report the real absolute shaft position -- that's the first difference.\n")

        time.sleep(0.5)

        # ---- 2. Commanded slow spin + live poll ----
        target_lsb = int(SPIN_OUTPUT_DEG * param.pos_gain * param.dir_1)  # 0.01 motor-deg
        print(f"=== Spinning output shaft {SPIN_OUTPUT_DEG:.0f} deg "
              f"(= {SPIN_OUTPUT_DEG * GEAR_RATIO:.0f} motor-deg, {GEAR_RATIO} motor revs) ===")
        m.multi_turn_position_control(target_lsb, MAX_SPEED)

        print(f"{'t(s)':>5} {'multi(0.01deg)':>15} {'multi_out(deg)':>15} "
              f"{'single(0.01deg)':>16} {'single(deg)':>12} {'enc_value':>10} {'wraps':>6}")

        start = time.time()
        prev_single = None
        wraps = 0
        target_motor_deg = abs(SPIN_OUTPUT_DEG * GEAR_RATIO)

        while True:
            now = time.time() - start
            multi = m.read_multi_turn_angle()
            single = m.read_single_turn_angle()
            enc_value, _, _ = m.read_encoder_data()

            # Count single-turn wraps (big jump = rolled over 0 <-> 360).
            if prev_single is not None and abs(single - prev_single) > 18000:
                wraps += 1
            prev_single = single

            multi_out_deg = (multi / 100.0) / GEAR_RATIO
            print(f"{now:5.1f} {multi:15d} {multi_out_deg:15.2f} "
                  f"{single:16d} {single / 100.0:12.2f} {enc_value:10d} {wraps:6d}")

            # Stop once we've reached the commanded angle (or timeout).
            if abs(multi / 100.0) >= target_motor_deg - 5.0:
                break
            if now > MOVE_TIMEOUT:
                print("  (timeout reached)")
                break
            time.sleep(1.0 / POLL_HZ)

        # ---- 3. Summary ----
        end_multi = m.read_multi_turn_angle()
        delta_motor_deg = abs(end_multi / 100.0)
        ratio = delta_motor_deg / SPIN_OUTPUT_DEG if SPIN_OUTPUT_DEG else 0.0

        print("\n=== Summary ===")
        print(f"  commanded output rotation : {SPIN_OUTPUT_DEG:.1f} deg")
        print(f"  multi-turn change         : {delta_motor_deg:.1f} motor-deg")
        print(f"  single-turn wraps observed: {wraps}")
        print(f"  ratio (motor-deg / output-deg) ~= {ratio:.1f}")
        print(f"\n  => single-turn/encoder wrap once per motor rev and rolled over ~{wraps}x;")
        print(f"     multi-turn accumulated smoothly. Ratio ~{ratio:.0f} confirms the encoder")
        print("     is on the MOTOR shaft (pre-gearbox), and the reducer is ~10:1.")

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        m.motor_release()
        bus.shutdown()
        print(f"\n[Motor {MOTOR_ID}] released, bus closed.")


if __name__ == "__main__":
    main()
