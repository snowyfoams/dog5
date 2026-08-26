#!/usr/bin/env python3
"""
imu_noise_log.py -- Phase 0 step 3: quantify roll/pitch noise under power.

Captures EVERY AHRS packet (~200 Hz, via callback) for a fixed duration,
writes a CSV, then prints noise statistics and a low-pass filter
recommendation.

Intended procedure:
    1. Motors OFF, dog standing still:
           python3 imu_noise_log.py --tag motors_off
    2. Motors ARMED in a static hold (run the stand script in another
       terminal -- the IMU is a separate USB device, no CAN conflict):
           python3 imu_noise_log.py --tag motors_on
    3. Compare the two reports; pick the LPF cutoff.

Re-analyze an existing log:
    python3 imu_noise_log.py --analyze logs/imu_noise_motors_on_XXXX.csv

Options:
    --secs N      capture duration (default 60)
    --tag NAME    label baked into the CSV filename (default "run")
    --port PORT   serial port (default /dev/fdilink_imu)
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np

from imu_dog import DEFAULT_PORT, ImuDog

LOG_DIR = Path(__file__).resolve().parent / "logs"

# Candidate first-order LPF cutoffs to evaluate against the log
CANDIDATE_CUTOFFS_HZ = [50.0, 30.0, 20.0, 10.0, 5.0, 2.0, 1.0]
# Phase 2 deadband is ~0.5 deg; we want filtered noise well below it
NOISE_TARGET_DEG = 0.05


def capture(port: str, secs: float, tag: str) -> Path:
    rows = []
    with ImuDog(port) as imu:
        print(f"Opened {port}. Waiting for data...")
        if not imu.wait_for_data(timeout=3):
            sys.exit("No data. Check cable / udev rule.")

        t0 = time.monotonic()

        def on_ahrs(a):
            # runs in the reader thread -- keep it cheap
            roll, pitch, yaw, rr, pr, yr = ImuDog.sensor_to_dog(a)
            rows.append((time.monotonic() - t0, a.timestamp_us,
                         roll, pitch, yaw, rr, pr, yr))

        imu._imu.on_ahrs(on_ahrs)
        print(f"Logging {secs:.0f} s (tag: {tag})... keep the dog still.")
        while time.monotonic() - t0 < secs:
            time.sleep(0.5)
            n = len(rows)
            print(f"\r  {time.monotonic()-t0:5.1f} s  {n:6d} samples  "
                  f"{imu.rate_hz:3.0f} Hz  crc {imu.crc_error_count}",
                  end="", flush=True)
        print()
        if imu.crc_error_count:
            print(f"WARNING: {imu.crc_error_count} CRC errors during capture")

    LOG_DIR.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = LOG_DIR / f"imu_noise_{tag}_{stamp}.csv"
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_host_s", "timestamp_us", "roll_deg", "pitch_deg",
                    "yaw_deg", "roll_rate_dps", "pitch_rate_dps",
                    "yaw_rate_dps"])
        w.writerows(rows)
    print(f"wrote {len(rows)} samples -> {path}")
    return path


def first_order_lpf(x: np.ndarray, dt: float, fc_hz: float) -> np.ndarray:
    """Simple causal first-order low-pass, the same filter the 250 Hz
    control loop would run: y += alpha * (x - y)."""
    tau = 1.0 / (2.0 * math.pi * fc_hz)
    alpha = dt / (tau + dt)
    y = np.empty_like(x)
    acc = x[0]
    for i, v in enumerate(x):
        acc += alpha * (v - acc)
        y[i] = acc
    return y


def dominant_freqs(x: np.ndarray, fs: float, n_peaks: int = 3):
    """Top spectral peaks (Hz, relative power) above 0.5 Hz."""
    x = x - x.mean()
    win = np.hanning(len(x))
    spec = np.abs(np.fft.rfft(x * win)) ** 2
    freqs = np.fft.rfftfreq(len(x), 1.0 / fs)
    mask = freqs > 0.5
    spec, freqs = spec[mask], freqs[mask]
    if not len(spec) or spec.max() == 0:
        return []
    order = np.argsort(spec)[::-1][:n_peaks]
    total = spec.sum()
    return [(freqs[i], spec[i] / total) for i in order]


def analyze(path: Path) -> None:
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = data["t_host_s"]
    n = len(t)
    # Host arrival times come in USB bursts -- median diff is useless.
    # Rate from elapsed time; gap detection from the sensor's own clock.
    fs = (n - 1) / (t[-1] - t[0])
    dt = 1.0 / fs
    print(f"\n=== {path.name} ===")
    print(f"{n} samples, {t[-1]-t[0]:.1f} s, AHRS rate ~ {fs:.0f} Hz")

    imu_gaps_ms = np.diff(data["timestamp_us"]) / 1000.0
    worst = imu_gaps_ms.max()
    print(f"worst sensor-clock gap: {worst:.1f} ms"
          + ("  <-- dropped packets, check USB!" if worst > 50 else ""))

    for name in ("roll_deg", "pitch_deg"):
        x = data[name]
        mean, std, p2p = x.mean(), x.std(), x.max() - x.min()
        peaks = dominant_freqs(x, fs)
        peak_str = "  ".join(f"{f:5.1f} Hz ({p*100:2.0f}%)" for f, p in peaks)
        print(f"\n{name:10s} mean {mean:+7.3f}  std {std:6.3f}  "
              f"p2p {p2p:6.3f} deg")
        print(f"           dominant: {peak_str or 'none'}")

    yaw = data["yaw_deg"]
    yaw_p2p = yaw.max() - yaw.min()
    print(f"\nyaw (mag!)  p2p {yaw_p2p:6.2f} deg  "
          f"(magnetometer corruption indicator -- expect large under power)")

    print(f"\nLPF candidates (causal 1st-order, as run at {fs:.0f} Hz; "
          f"target filtered std < {NOISE_TARGET_DEG} deg):")
    print(f"{'fc (Hz)':>8} {'roll std':>10} {'pitch std':>10} {'lag (ms)':>9}")
    recommend = None
    for fc in CANDIDATE_CUTOFFS_HZ:
        stds = []
        for name in ("roll_deg", "pitch_deg"):
            x = data[name]
            y = first_order_lpf(x, dt, fc)
            skip = min(int(fs), len(y) // 4)  # drop settling, keep >= 3/4
            stds.append(y[skip:].std())
        lag_ms = 1000.0 / (2.0 * math.pi * fc)
        mark = ""
        if max(stds) < NOISE_TARGET_DEG and recommend is None:
            recommend = fc
            mark = "  <-- recommended"
        print(f"{fc:8.1f} {stds[0]:10.4f} {stds[1]:10.4f} {lag_ms:9.1f}{mark}")
    if recommend is None:
        print("no candidate met the target -- noise floor too high, "
              "inspect the spectrum above")
    else:
        tau = 1.0 / (2.0 * math.pi * recommend)
        dt250 = 1.0 / 250.0
        alpha = dt250 / (tau + dt250)
        print(f"\nrecommended: fc = {recommend} Hz -> "
              f"alpha = {alpha:.4f} at 250 Hz  (y += alpha*(x-y))")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--secs", type=float, default=60.0)
    ap.add_argument("--tag", default="run")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--analyze", type=Path, metavar="CSV",
                    help="skip capture, analyze an existing log")
    args = ap.parse_args()

    path = args.analyze or capture(args.port, args.secs, args.tag)
    analyze(path)


if __name__ == "__main__":
    main()
