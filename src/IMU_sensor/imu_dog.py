#!/usr/bin/env python3
"""
imu_dog.py -- DETA10 adapter that reports attitude in the DOG (trunk) frame.

Frames (settled in Phase 0, 2026-07-22)
---------------------------------------
* Sensor: NED body (X fwd, Y right, Z down) -- what the DETA10 reports.
* Dog / trunk frame: **FLU** (X fwd, Y LEFT, Z UP), matching dog5.xml and
  dog5_kinematics.py (hip_FL at +Y, trunk z positive up).
* FLU = NED axes rotated 180 deg about X.  Physically the PCB is mounted
  aligned with the trunk (flat dog reads sensor roll/pitch ~ 0; the ~+2.5 deg
  residual is mounting error, removed by the offsets).

Sign conventions in the DOG (FLU) frame:
        roll  > 0  -> right side DOWN   (left side rises)
        pitch > 0  -> nose DOWN
        yaw   > 0  -> nose swings LEFT  (CCW seen from above)

Axes-convention change NED -> FLU (conjugation by R_x(180deg)):

    roll_dog  =  roll_sensor
    pitch_dog = -pitch_sensor
    yaw_dog   = -heading_sensor -- EXPOSED BUT UNTRUSTED: magnetometer-based,
                and the mag sits next to 12 motors + a steel frame.  Fine for
                casual display; do NOT close a control loop on it until it is
                validated (watch it during the noise-under-power test).
    rates: (wx, wy, wz)_dog = (wx, -wy, -wz)_sensor

Verify the signs with imu_frame_test.py before trusting them in control.
yaw_rate (gyro wz) is inertial, NOT magnetometer-based -- it is trustworthy
for short-horizon relative heading (drifts slowly when integrated).

Mounting offsets (dog flat on a level floor) are subtracted after the frame
transform and persisted in imu_calib.json next to this file.
"""
from __future__ import annotations

import os, sys                                                  # noqa: E401
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dog5_paths  # noqa: E402,F401  -- every src/ dir onto sys.path

import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# fdilink_imu is a VENDOR SDK: not pip-installable, and not vendored in this
# repository.  Install it on the robot host at ~/Documents/IMU_sensor/fdilink_imu.
# dog5_paths.add_fdilink_root() has already put it on sys.path if it is present;
# that helper searches repo-relative BEFORE $HOME, which is what makes the import
# survive sudo (where $HOME becomes /root and the old Path.home() lookup failed).
#
# The import is guarded so that this module stays importable on a machine with no
# SDK.  Eleven of the seventeen call sites in this repository want only
# DEFAULT_PORT, and the offline gate suite fakes the sensor outright
# (selftest_common.FakeAhrs / FakeFeed), so a hard import here would block every
# no-hardware test for the sake of one constant.  Opening a real IMU still fails
# loudly -- see ImuDog.__init__.
try:
    from fdilink_imu import DETA10, DEFAULT_BAUD, AHRSData  # noqa: E402
    _FDILINK_IMPORT_ERROR = None
except ImportError as _exc:                                  # no SDK on this host
    DETA10 = None
    AHRSData = object
    DEFAULT_BAUD = None
    _FDILINK_IMPORT_ERROR = _exc

DEFAULT_PORT = "/dev/fdilink_imu"
DEFAULT_CALIB_PATH = Path(__file__).resolve().parent / "imu_calib.json"


def wrap_deg(a: float) -> float:
    """Wrap an angle to (-180, 180]."""
    a = math.fmod(a + 180.0, 360.0)
    if a <= 0.0:
        a += 360.0
    return a - 180.0


@dataclass(frozen=True)
class DogAttitude:
    """Attitude in the dog FLU frame (X fwd, Y left, Z up) -- see module doc."""

    roll_deg: float          # >0 = right side down; mounting offset removed
    pitch_deg: float         # >0 = nose DOWN; mounting offset removed
    yaw_deg: float           # >0 = nose left; magnetometer -- UNTRUSTED near
                             # motors, no offset applied; display/logging only
    roll_rate_dps: float     # dog frame (signs to be verified in Phase 0)
    pitch_rate_dps: float
    yaw_rate_dps: float      # gyro wz -- inertial, OK for relative heading
    age_s: float             # host time since the packet arrived
    raw: AHRSData            # untouched sensor packet (NED sensor frame)


class ImuDog:
    """Wraps DETA10; outputs roll/pitch in the dog frame with staleness info."""

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
                 calib_path: Path = DEFAULT_CALIB_PATH):
        if DETA10 is None:
            raise ImportError(
                "the fdilink_imu vendor SDK is required to talk to the IMU but "
                "was not found on this host. It is not pip-installable; install "
                "it at ~/Documents/IMU_sensor/fdilink_imu (or beside this repo). "
                "Importing imu_dog without it is supported -- constants and the "
                "pure frame maths work -- but opening a device does not."
            ) from _FDILINK_IMPORT_ERROR
        self._imu = DETA10(port, baud)
        self._calib_path = Path(calib_path)
        self.roll_offset_deg = 0.0
        self.pitch_offset_deg = 0.0
        self._last_ahrs: Optional[AHRSData] = None
        self._last_rx_mono: float = 0.0
        self._imu.on_ahrs(self._on_ahrs)
        self.load_calib()

    # -- lifecycle ---------------------------------------------------------
    def start(self) -> "ImuDog":
        self._imu.start()
        return self

    def stop(self) -> None:
        self._imu.stop()

    def __enter__(self) -> "ImuDog":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def wait_for_data(self, timeout: Optional[float] = None) -> bool:
        return self._imu.wait_for_data(timeout)

    # -- stream health -----------------------------------------------------
    @property
    def rate_hz(self) -> float:
        return self._imu.rate_hz

    @property
    def crc_error_count(self) -> int:
        return self._imu.crc_error_count

    def age_s(self) -> float:
        """Host time since the last AHRS packet (inf before the first one)."""
        if self._last_rx_mono == 0.0:
            return float("inf")
        return time.monotonic() - self._last_rx_mono

    def is_stale(self, max_age_s: float = 0.05) -> bool:
        return self.age_s() > max_age_s

    # -- attitude ----------------------------------------------------------
    def _on_ahrs(self, a: AHRSData) -> None:
        self._last_ahrs = a
        self._last_rx_mono = time.monotonic()

    @staticmethod
    def sensor_to_dog(a: AHRSData):
        """Frame transform only (no mounting offsets). Returns deg/deg-per-s.

        NED sensor -> FLU dog (Y and Z axes negated, X shared).
        """
        roll = a.roll_deg
        pitch = -a.pitch_deg
        yaw = wrap_deg(-a.heading_deg)          # magnetometer-based, untrusted
        wx, wy, wz = a.angular_rates            # rad/s, sensor (NED) axes
        roll_rate = math.degrees(wx)
        pitch_rate = math.degrees(-wy)
        yaw_rate = math.degrees(-wz)
        return roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate

    def sample(self) -> Optional[DogAttitude]:
        a = self._last_ahrs
        if a is None:
            return None
        roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate = self.sensor_to_dog(a)
        return DogAttitude(
            roll_deg=wrap_deg(roll - self.roll_offset_deg),
            pitch_deg=pitch - self.pitch_offset_deg,
            yaw_deg=yaw,
            roll_rate_dps=roll_rate,
            pitch_rate_dps=pitch_rate,
            yaw_rate_dps=yaw_rate,
            age_s=self.age_s(),
            raw=a,
        )

    # -- mounting-offset calibration --------------------------------------
    def capture_offsets(self, duration_s: float = 1.0) -> Optional[tuple]:
        """Average dog-frame roll/pitch for duration_s -> mounting offsets.

        Call with the dog flat on a level floor.  Circular mean, so it is
        safe even right at the wrap point.  Returns (roll, pitch) or None
        if no fresh data arrived.
        """
        sin_r = cos_r = 0.0
        pitch_sum = 0.0
        n = 0
        last_ts = None
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            a = self._last_ahrs
            if a is not None and a.timestamp_us != last_ts:
                last_ts = a.timestamp_us
                roll, pitch, *_ = self.sensor_to_dog(a)
                sin_r += math.sin(math.radians(roll))
                cos_r += math.cos(math.radians(roll))
                pitch_sum += pitch
                n += 1
            time.sleep(0.002)
        if n == 0:
            return None
        self.roll_offset_deg = math.degrees(math.atan2(sin_r, cos_r))
        self.pitch_offset_deg = pitch_sum / n
        return self.roll_offset_deg, self.pitch_offset_deg

    def clear_offsets(self) -> None:
        self.roll_offset_deg = 0.0
        self.pitch_offset_deg = 0.0

    def save_calib(self) -> Path:
        data = {
            "roll_offset_deg": self.roll_offset_deg,
            "pitch_offset_deg": self.pitch_offset_deg,
            "captured_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "frame_note": "offsets are in the DOG FLU frame (X fwd, Y left, Z up)",
        }
        self._calib_path.write_text(json.dumps(data, indent=2) + "\n")
        return self._calib_path

    def load_calib(self) -> bool:
        try:
            data = json.loads(self._calib_path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            return False
        self.roll_offset_deg = float(data.get("roll_offset_deg", 0.0))
        self.pitch_offset_deg = float(data.get("pitch_offset_deg", 0.0))
        return True
