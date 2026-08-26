#!/usr/bin/env python3
"""
imu_frame_test.py -- Phase 0 frame & sign calibration (motors OFF).

Shows raw sensor angles next to the dog-frame angles from imu_dog.py so you
can verify the mounting transform by hand-rotating the body.

DOG frame = FLU (X fwd, Y LEFT, Z UP) -- matches dog5.xml / dog5_kinematics:

    dog flat & level      -> dog roll ~ 0, dog pitch ~ 0
    nose DOWN             -> dog pitch goes POSITIVE
    nose UP               -> dog pitch goes NEGATIVE
    RIGHT side DOWN       -> dog roll  goes POSITIVE
    LEFT side DOWN        -> dog roll  goes NEGATIVE
    spin CCW from above (nose swings LEFT) -> dog yaw increases

Rates (verify while moving, sign convention matches the angles):
    rolling right-side-down   -> roll rate  > 0
    pitching nose-down        -> pitch rate > 0
    spinning nose-left        -> yaw rate   > 0

Keys:
    z   capture mounting offsets (1 s average -- dog must be flat & still)
    c   clear offsets
    s   save offsets to imu_calib.json
    q   quit

Usage:  python3 imu_frame_test.py [port]     (default /dev/fdilink_imu)
"""
import select
import sys
import termios
import time
import tty

from imu_dog import DEFAULT_PORT, ImuDog


class KeyPoller:
    def __init__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("frame test requires an interactive terminal")
        self.fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self._closed = False

    def get(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if readable else ""

    def close(self):
        if not self._closed:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old)
            self._closed = True


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    print(__doc__)
    with ImuDog(port) as imu:
        print(f"Opened {port}. Waiting for data...")
        if not imu.wait_for_data(timeout=3):
            print("No data. Check cable / udev rule / baud.")
            return
        if imu.roll_offset_deg or imu.pitch_offset_deg:
            print(f"Loaded offsets: roll {imu.roll_offset_deg:+.2f}  "
                  f"pitch {imu.pitch_offset_deg:+.2f} deg")

        keys = KeyPoller()
        msg = ""
        try:
            while True:
                k = keys.get()
                if k == "q":
                    break
                elif k == "z":
                    print("\ncapturing offsets (keep the dog flat & still)...")
                    got = imu.capture_offsets(1.0)
                    msg = ("no data!" if got is None else
                           f"offsets set: roll {got[0]:+.2f} pitch {got[1]:+.2f}")
                elif k == "c":
                    imu.clear_offsets()
                    msg = "offsets cleared"
                elif k == "s":
                    path = imu.save_calib()
                    msg = f"saved -> {path.name}"

                d = imu.sample()
                if d is None:
                    line = "waiting for AHRS packets..."
                else:
                    a = d.raw
                    stale = "  STALE!" if d.age_s > 0.05 else ""
                    line = (
                        f"DOG roll {d.roll_deg:+7.2f}  pitch {d.pitch_deg:+7.2f}  "
                        f"yaw {d.yaw_deg:+7.1f}(mag!) | "
                        f"rate r {d.roll_rate_dps:+6.1f} p {d.pitch_rate_dps:+6.1f} "
                        f"y {d.yaw_rate_dps:+6.1f} dps | "
                        f"raw r {a.roll_deg:+7.1f} p {a.pitch_deg:+6.1f} | "
                        f"{imu.rate_hz:3.0f} Hz crc {imu.crc_error_count}"
                        f"{stale}"
                    )
                if msg:
                    line += f"   [{msg}]"
                print(f"\r\x1b[2K{line}", end="", flush=True)
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            keys.close()
            print("\nstopped.")


if __name__ == "__main__":
    main()
