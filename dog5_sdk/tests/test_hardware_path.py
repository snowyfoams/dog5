#!/usr/bin/env python3
"""The whole CAN path, with no CAN: Dog5Hardware against twelve fake drivers.

    python tests/test_hardware_path.py          # needs python-can, no adapter

:class:`dog5_sdk.fake_bus.FakeDriverBus` decodes the frames this SDK sends with
the same protocol the real drivers use, so these gates catch the errors that
are expensive to find on the robot: a wrong CAN ID, a flipped direction, a unit
that is out by the 10:1 reduction, a payload field at the wrong byte, a
position command that never settles, a torque that is not cleared on the way
out.

What they do NOT catch is anything physical.  A green run here says the
plumbing is right, not that a controller works -- that is
``tests/test_offline.py`` and the simulator.
"""
from __future__ import annotations

import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import dog5_sdk as d5                                        # noqa: E402
from dog5_sdk import poses                                   # noqa: E402
from dog5_sdk.fake_bus import FakeDriverBus                  # noqa: E402


def _robot(**kwargs):
    """A Dog5Hardware wired to a fake bus, armed and ready to read."""
    bus = FakeDriverBus(**kwargs)
    robot = d5.Dog5Hardware(bus=bus, prompt=False)
    return robot, bus


# ---------------------------------------------------------------------------
# the encodings
# ---------------------------------------------------------------------------
def test_torque_reaches_the_wire_with_the_right_sign_and_scale():
    """A joint torque command must arrive as ``tau * 206.04 * direction``.

    This is the single most expensive thing to get wrong: a sign error here
    turns a stabilising controller into a diverging one, and the robot is the
    thing that finds out.
    """
    from dog5_sdk.motor import MotorBus

    bus = FakeDriverBus()
    with MotorBus(d5.MOTOR_IDS, bus=bus, dirs=d5.MOTOR_DIRECTIONS) as mb:
        for mid in d5.MOTOR_IDS:
            mb.torque(mid, 1.0)
        mb.poll()
        commanded = bus.commanded_iq()
        for joint in d5.HARDWARE_JOINTS:
            expected = int(round(1.0 * 206.04 * joint.direction))
            assert commanded[joint.can_id] == expected, (
                f"CAN {joint.can_id} ({joint.leg}_{joint.joint}): got "
                f"{commanded[joint.can_id]} LSB, expected {expected}")


def test_torque_payload_puts_iq_at_bytes_4_5():
    """The layout, checked directly rather than through the fake driver."""
    from dog5_sdk.motor import motorbus

    captured = []

    class _Recorder:
        def send(self, msg, timeout=None):
            captured.append(bytes(msg.data))

        def recv(self, timeout=0.0):
            return None

        def shutdown(self):
            pass

    rr = motorbus.RoundRobinBus(_Recorder(), [1])
    mb = motorbus.MotorBus([1], bus=_Recorder(), dirs={1: 1})
    mb._rr = rr
    mb.torque(1, -2.5)
    assert captured, "nothing was sent"
    frame = captured[-1]
    assert frame[0] == 0xA1
    assert struct.unpack_from("<h", frame, 4)[0] == int(round(-2.5 * 206.04))


def test_encoder_reads_back_as_the_joint_angle_it_was_placed_at():
    """The full feedback chain: raw uint16 -> unwrap -> direction -> radians."""
    robot, bus = _robot()
    with robot:
        for pose in (poses.Q_CROUCH, poses.Q_ROLL, np.zeros(12)):
            bus.set_joint_angles(pose)
            # Telemetry only arrives in REPLIES, so a sweep has to go out
            # before the new position can be read.  Resetting the unwrappers
            # is what a teleport needs; a real joint cannot jump 180 deg
            # between two 4 ms samples, and the unwrap is entitled to assume
            # it did not.
            robot._unwrap = d5.calibration.new_unwrappers()
            robot._tick()
            state = robot.read()
            worst = float(np.max(np.abs(state.q - pose)))
            assert worst < 1e-4, (f"read back {np.round(state.q, 4)} for a "
                                  f"pose of {np.round(pose, 4)}")


def test_position_move_settles_at_the_commanded_pose():
    robot, bus = _robot()
    with robot:
        reached = robot.move_to(poses.Q_CROUCH, max_motor_dps=6000.0,
                                verbose=False)
        assert np.max(np.abs(reached - poses.Q_CROUCH)) < 0.08
        # and the drivers really hold the MOTOR-output angle, direction applied
        dials = d5.joint_rad_to_motoroutput_deg(poses.Q_CROUCH)
        actual = bus.motoroutput_deg()
        for index, joint in enumerate(d5.HARDWARE_JOINTS):
            assert abs(actual[joint.can_id] - dials[index]) < 5.0, (
                f"CAN {joint.can_id}: driver at {actual[joint.can_id]:.1f} deg, "
                f"the pose wants {dials[index]:.1f}")


# ---------------------------------------------------------------------------
# arming and the latch
# ---------------------------------------------------------------------------
def test_arming_clears_the_input_lost_latch_over_can():
    """Every driver boots latched (0x80).  Arming must clear it with the
    0x9B -> 0x88 ladder and no power cycle -- that is the result the whole
    hardware bring-up rests on."""
    robot, bus = _robot(latched_at_boot=True)
    for driver in bus.drivers.values():
        assert driver.error == 0x80
    with robot:
        state = robot.read()
        assert all(err == 0 for err in state.extra["errors"].values())
        assert all(driver.state == 1 for driver in bus.drivers.values()), (
            "arming must leave every driver in the RUN state")


def test_a_run_ends_with_every_motor_commanded_to_zero():
    """However a run ends, nothing may be left pushing."""
    robot, bus = _robot()
    with robot:
        robot.run(d5.JointPD(poses.Q_CROUCH, kp=2.0, kd=0.05), duration=0.2,
                  tau_max=1.0, keys=False, verbose=False)
    assert all(iq == 0 for iq in bus.commanded_iq().values()), (
        f"left commanded: {bus.commanded_iq()}")
    # An INJECTED bus belongs to the caller and is left open -- the same
    # contract MotorBus has, so two robots can share one adapter.  A bus the
    # SDK opened itself is shut down instead.
    assert not bus.is_shutdown


# ---------------------------------------------------------------------------
# the loop
# ---------------------------------------------------------------------------
def test_a_full_run_through_the_can_path():
    class Counting(d5.Controller):
        def __init__(self):
            self.ticks = 0
            self.started = False
            self.stopped = None

        def on_start(self, robot, state):
            self.started = True

        def update(self, state):
            self.ticks += 1
            return np.full(12, 0.2)

        def on_stop(self, state, reason):
            self.stopped = reason

    robot, bus = _robot()
    controller = Counting()
    with robot:
        bus.set_joint_angles(poses.Q_CROUCH)
        robot._unwrap = d5.calibration.new_unwrappers()
        robot._tick()
        result = robot.run(controller, duration=0.4, tau_max=1.0, keys=False,
                           verbose=False, log=True)

    assert controller.started and controller.stopped
    assert not result.tripped, result.reason
    assert controller.ticks == result.ticks > 50, result.ticks
    # 250 Hz per motor: 0.4 s is about 100 sweeps, twelve frames each
    assert 80 < result.ticks < 130, f"loop rate is off: {result.ticks} ticks"
    assert result.log["tau_cmd"].shape == (result.ticks, 12)
    # the gate's ramp must be visible at the start of the log
    assert result.log["tau_cmd"][0].max() < 0.2


def test_the_safety_gate_is_live_on_hardware_too():
    class Slam(d5.Controller):
        def update(self, state):
            return np.full(12, 50.0)          # way over any cap

    robot, bus = _robot()
    with robot:
        result = robot.run(Slam(), duration=0.5, tau_max=1.0, keys=False,
                           verbose=False, log=True)
    peak = float(np.max(np.abs(result.log["tau_cmd"])))
    assert peak <= 1.0 + 1e-9, f"the cap leaked: {peak} N*m"
    commanded = max(abs(iq) for iq in bus.commanded_iq().values())
    assert commanded <= int(1.0 * 206.04) + 1, commanded


def test_a_silent_bus_is_an_error_not_a_stale_reading():
    """Arm on a healthy bus, then make it go silent mid-run.

    The failure this guards against is the quiet one: reading the LAST known
    encoder value forever and driving torque from it.  A missing reply must
    raise, not be substituted.
    """
    robot, bus = _robot()
    with robot:
        robot.read()                           # healthy
        bus.drop_rate = 1.0                    # the bus stops answering
        for _ in range(30):                    # drain what is still in flight
            robot._tick()
        try:
            robot.read()
        except RuntimeError as exc:
            assert "no encoder reply" in str(exc), exc
            return
    raise AssertionError("a bus that answers nothing must not read as healthy")


def test_declared_contact_is_what_the_estimator_uses():
    robot, bus = _robot()
    with robot:
        bus.set_joint_angles(poses.stand_pose(0.15))
        robot._unwrap = d5.calibration.new_unwrappers()
        robot._tick()
        assert np.all(robot.read().contact), "hardware defaults to four down"
        robot.set_contact([True, False, False, True])
        state = robot.read()
        assert state.extra["n_planted"] == 2
        assert not state.extra["estimate_valid"]


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------
def main() -> int:
    try:
        import can                                          # noqa: F401
    except ImportError:
        print("skip: python-can is not installed, so the CAN path cannot be "
              "exercised even against a fake bus")
        return 0

    tests = [(name, value) for name, value in sorted(globals().items())
             if name.startswith("test_") and callable(value)]
    failed = []
    for name, test in tests:
        try:
            test()
        except Exception as exc:                             # noqa: BLE001
            failed.append(name)
            print(f"FAIL  {name}\n        {type(exc).__name__}: {exc}")
        else:
            print(f"ok    {name}", flush=True)
    print()
    if failed:
        print(f"{len(failed)} of {len(tests)} FAILED")
        return 1
    print(f"all {len(tests)} hardware-path gates passed (against fake drivers)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
