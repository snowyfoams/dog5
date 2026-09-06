"""A software stand-in for twelve LK drivers on a CAN bus.

    from dog5_sdk import Dog5Hardware
    from dog5_sdk.fake_bus import FakeDriverBus

    with Dog5Hardware(bus=FakeDriverBus(), prompt=False) as robot:
        robot.crouch()
        robot.run(MyController(), duration=2.0)

WHAT IT IS FOR
    Running the WHOLE hardware path -- arming, the recovery ladder, the
    round-robin schedule, the encoder unwrap, the direction convention, the
    position mode, the soft stop -- with no bus, no adapter and no robot.  Every
    byte your code puts on the wire is decoded here by the same protocol the
    drivers use, so a sign error, a unit error or a wrong CAN ID shows up on a
    laptop instead of on a 5.8 kg machine.

WHAT IT IS NOT
    A robot.  There is no dynamics: a joint sits where it is until a position
    command moves it, torque commands move nothing, and the reported iq is the
    COMMANDED current, not a measured one.  It answers the question "does my
    hardware code speak the protocol correctly", never "will this controller
    stand up" -- that is what :class:`dog5_sdk.Dog5Sim` is for.

    In particular a green run here says nothing about torque values.  Use the
    simulator for the control law and this for the plumbing.

WHAT IT DOES MODEL
    * the reply frames for 0x9A/0x9B/0x9C/0xA1/0xA2/0xA3/0xA4/0x81/0x80/0x19,
      byte for byte, including which telemetry rides in which reply;
    * the input-signal-lost latch: every driver boots with error bit 0x80 set
      (that is what really happens when the motors are powered before the
      control stream starts) and clears it only on 0x9B, so
      ``arm_motors``' 0x9B -> 0x88 ladder is genuinely exercised;
    * the drivers' own 0xA3/0xA4 position loop, speed-capped, advancing in
      wall-clock time -- so ``Dog5Hardware.move_to`` really does settle;
    * the 16-bit encoder register, so the unwrap is exercised through a wrap;
    * optional frame loss, to see what your code does when the bus drops.
"""
from __future__ import annotations

import random
import struct
import time

import numpy as np

from .calibration import MOTOR_DIRECTIONS, MOTOR_IDS
from .motor import ENCODER_GAIN, POS_GAIN, VEL_STATE_GAIN

REPLY_BASE = 0x140
_FULL_TURN = 65536


class _Message:
    """The two fields the motor library reads off a received frame."""

    __slots__ = ("arbitration_id", "data", "is_error_frame", "is_extended_id")

    def __init__(self, arbitration_id, data):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)
        self.is_error_frame = False
        self.is_extended_id = False


class _Driver:
    """One virtual LK driver."""

    def __init__(self, motoroutput_deg: float, latched: bool):
        self.motoroutput_deg = float(motoroutput_deg)
        self.target_deg = None            # 0xA3/0xA4 position target
        self.max_motor_dps = None         # 0xA4 speed cap, motor side
        self.iq = 0                       # last commanded iq, LSB
        self.output_dps = 0.0
        self.error = 0x80 if latched else 0x00
        self.state = 0
        self.temp = 30
        self.voltage_lsb = 2400           # 24.00 V, 0.01 V per LSB
        self.encoder_offset = None
        self._last_t = time.perf_counter()

    def advance(self) -> None:
        """Run the driver's own position loop forward to now."""
        now = time.perf_counter()
        dt = now - self._last_t
        self._last_t = now
        if self.target_deg is None or dt <= 0.0:
            self.output_dps = 0.0
            return
        # the cap is motor-side dps at 1 dps/LSB; the output shaft is 10:1 down
        cap = (self.max_motor_dps or 600.0) / 10.0
        error = self.target_deg - self.motoroutput_deg
        step = max(-cap * dt, min(cap * dt, error))
        self.motoroutput_deg += step
        self.output_dps = step / dt if dt > 0 else 0.0
        if abs(self.target_deg - self.motoroutput_deg) < 1e-9:
            self.motoroutput_deg = self.target_deg
            self.output_dps = 0.0

    @property
    def encoder_raw(self) -> int:
        return int(round(self.motoroutput_deg / ENCODER_GAIN)) % _FULL_TURN


class FakeDriverBus:
    """A ``python-can``-shaped bus that answers like twelve LK drivers.

    ids               which CAN IDs exist on this bus
    pose_deg          starting MOTOR-OUTPUT angles (12,), or a {can_id: deg}
                      dict, or None for all zero
    latched_at_boot   start with the input-signal-lost latch set, which is what
                      a real driver does when it is powered before the control
                      stream starts.  Leave it True: clearing it is the code
                      path that removed the power cycle between runs
    drop_rate         fraction of replies to silently drop, for testing what
                      your code does when the bus is lossy
    """

    def __init__(self, ids=None, pose_deg=None, *, latched_at_boot: bool = True,
                 drop_rate: float = 0.0, seed: int = 0):
        self.ids = list(MOTOR_IDS if ids is None else ids)
        self.drop_rate = float(drop_rate)
        self._random = random.Random(seed)
        self._queue = []
        self.sent_frames = 0
        self.dropped_frames = 0
        self.is_shutdown = False

        start = self._resolve_pose(pose_deg)
        self.drivers = {mid: _Driver(start[mid], latched_at_boot)
                        for mid in self.ids}

    def _resolve_pose(self, pose_deg) -> dict:
        if pose_deg is None:
            return {mid: 0.0 for mid in self.ids}
        if isinstance(pose_deg, dict):
            return {mid: float(pose_deg.get(mid, 0.0)) for mid in self.ids}
        values = np.asarray(pose_deg, dtype=float).reshape(-1)
        if values.size != len(self.ids):
            raise ValueError(f"pose_deg must have {len(self.ids)} values")
        return {mid: float(values[index])
                for index, mid in enumerate(self.ids)}

    # -- the joint-level view, for tests --------------------------------
    def set_joint_angles(self, q_rad, dirs=None) -> None:
        """Place the virtual robot at a JOINT pose (12,), radians.

        Applies the same direction convention the real map uses, so a test can
        say "the robot is at the crouch" without doing the sign arithmetic.
        """
        dirs = MOTOR_DIRECTIONS if dirs is None else dirs
        q_rad = np.asarray(q_rad, dtype=float).reshape(len(self.ids))
        for index, mid in enumerate(self.ids):
            self.drivers[mid].motoroutput_deg = float(
                np.rad2deg(q_rad[index]) * dirs[mid])
            self.drivers[mid].target_deg = None

    def motoroutput_deg(self) -> dict:
        return {mid: driver.motoroutput_deg
                for mid, driver in self.drivers.items()}

    def commanded_iq(self) -> dict:
        """The last iq each driver was commanded, in raw LSB.  This is where
        a direction or unit error becomes visible."""
        return {mid: driver.iq for mid, driver in self.drivers.items()}

    # -- the python-can interface ---------------------------------------
    def send(self, msg, timeout=None) -> None:
        if self.is_shutdown:
            raise OSError("bus is shut down")
        self.sent_frames += 1
        mid = msg.arbitration_id - REPLY_BASE
        driver = self.drivers.get(mid)
        if driver is None:
            return                       # a frame for a motor not on this bus
        driver.advance()
        reply = self._handle(driver, bytes(msg.data))
        if reply is None:
            return
        if self.drop_rate and self._random.random() < self.drop_rate:
            self.dropped_frames += 1
            return
        self._queue.append(_Message(REPLY_BASE + mid, reply))

    def recv(self, timeout=0.0):
        return self._queue.pop(0) if self._queue else None

    def shutdown(self) -> None:
        self.is_shutdown = True
        self._queue.clear()

    def flush_tx_buffer(self) -> None:
        pass

    # -- the protocol ----------------------------------------------------
    def _handle(self, driver: _Driver, data: bytes):
        cmd = data[0]

        if cmd == 0x9A:                                   # status 1
            return self._status1(driver)
        if cmd == 0x9B:                                   # clear error flags
            # This is the whole point of the latch emulation: 0x9B is what
            # clears bit 0x80, and nothing else does.
            driver.error = 0x00
            return self._status1(driver)
        if cmd == 0x9C:                                   # status 2
            return self._status2(driver, 0x9C)
        if cmd == 0x88:                                   # run
            driver.state = 1
            return self._status2(driver, 0x88)
        if cmd in (0x80, 0x81):                           # shutdown / stop
            driver.state = 0
            driver.iq = 0
            driver.target_deg = None
            return bytes([cmd, 0, 0, 0, 0, 0, 0, 0])
        # Payload layouts, which are the thing this class exists to check:
        #   0xA1  [cmd, 0, 0, 0, iq int16, 0, 0]              iq at byte 4
        #   0xA2  [cmd, 0, iq int16, speed int32]             iq 2, speed 4
        #   0xA3  [cmd, 0, 0, 0, angle int32]                 angle at 4
        #   0xA4  [cmd, 0, maxdps int16, angle int32]         cap 2, angle 4
        if cmd == 0xA1:                                   # torque
            driver.iq = struct.unpack_from("<h", data, 4)[0]
            driver.target_deg = None                      # torque cancels 0xA4
            return self._status2(driver, 0xA1)
        if cmd == 0xA2:                                   # speed
            driver.iq = struct.unpack_from("<h", data, 2)[0]
            driver.target_deg = None
            return self._status2(driver, 0xA2)
        if cmd == 0xA3:                                   # position, no cap
            driver.target_deg = struct.unpack_from("<i", data, 4)[0] / POS_GAIN
            driver.max_motor_dps = None
            return self._status2(driver, 0xA3)
        if cmd == 0xA4:                                   # position, capped
            driver.max_motor_dps = float(struct.unpack_from("<h", data, 2)[0])
            driver.target_deg = struct.unpack_from("<i", data, 4)[0] / POS_GAIN
            return self._status2(driver, 0xA4)
        if cmd == 0x19:                                   # set zero
            driver.encoder_offset = driver.encoder_raw
            driver.motoroutput_deg = 0.0
            driver.target_deg = None
            return bytes([0x19, 0, 0, 0, 0, 0]) + struct.pack(
                "<H", driver.encoder_offset)
        return None                                       # unknown: no reply

    @staticmethod
    def _status1(driver: _Driver) -> bytes:
        return (bytes([0x9A]) + struct.pack("<b", driver.temp)
                + struct.pack("<h", driver.voltage_lsb)
                + struct.pack("<h", 0)
                + bytes([driver.state, driver.error]))

    @staticmethod
    def _status2(driver: _Driver, cmd: int) -> bytes:
        speed_raw = int(round(driver.output_dps * VEL_STATE_GAIN))
        speed_raw = max(-32768, min(32767, speed_raw))
        return (bytes([cmd]) + struct.pack("<b", driver.temp)
                + struct.pack("<h", driver.iq)
                + struct.pack("<h", speed_raw)
                + struct.pack("<H", driver.encoder_raw))
