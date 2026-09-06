#!/usr/bin/env python3
"""Every gate that needs no robot.  Run this first, and after any change.

    python tests/test_offline.py            # ~20 s, no CAN, no IMU
    pytest tests/                           # the same tests

What it proves, in order:

  * the package is COMPLETE -- every file it claims to ship is there
  * the joint order in the MJCF is the joint order in the hardware map, so a
    12-vector cannot mean two different things in the two backends
  * the NumPy kinematics agree with MuJoCo, position and Jacobian
  * the statics agree with MuJoCo's floating-base inverse dynamics, including
    the leg-gravity term that a massless-leg model leaves out
  * the mass in the MJCF, in the statics and in the control params is ONE mass
  * IK inverts FK
  * the encoder <-> joint contract round-trips, and the unwrap counts wraps
  * the safety gate ramps, caps, blocks at the joint limits and slews
  * a controller actually stands the robot up in simulation, and the estimator
    the controller reads agrees with the simulator's ground truth
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import dog5_sdk as d5                                        # noqa: E402
from dog5_sdk import (calibration, estimate, ik, kinematics as kin,   # noqa: E402
                      params, poses, safety, statics)

RNG = np.random.default_rng(20260906)

#: Sampling box for the random poses the cross-checks use.  Wide enough to
#: exercise every sign, inside the soft limits so nothing is tested in a
#: configuration the robot is not allowed to reach.
_Q_LOW, _Q_HIGH = calibration.soft_limits()


def random_poses(n: int) -> np.ndarray:
    return RNG.uniform(_Q_LOW, _Q_HIGH, size=(n, 12))


# ---------------------------------------------------------------------------
# 1. completeness
# ---------------------------------------------------------------------------
def test_package_is_complete():
    root = os.path.dirname(os.path.abspath(d5.__file__))
    required = [
        "__init__.py", "base.py", "calibration.py", "estimate.py",
        "estimator.py", "hardware.py", "hardware_map.py", "ik.py",
        "keys.py", "kinematics.py", "params.py", "poses.py", "safety.py",
        "sim.py", "state.py", "statics.py", "fake_bus.py",
        "motor/__init__.py", "motor/motorbus.py", "motor/motor_library.py",
        "motor/motor_gains.py", "motor/mac_can.py",
        "model/dog5.xml",
        "model/meshes/trunk.stl", "model/meshes/hip.stl",
        "model/meshes/thigh.stl", "model/meshes/shin.stl",
    ]
    missing = [name for name in required
               if not os.path.exists(os.path.join(root, name))]
    assert not missing, f"missing from the package: {missing}"


def test_sim_does_not_need_python_can():
    """Importing the SDK and building a simulator must not touch python-can.

    A classmate running the simulator on a laptop may have no python-can and
    no adapter.  ``motorbus`` and ``motor_library`` import ``can`` at module
    level, so if anything on the simulator path imported either of them
    eagerly, that laptop would be stuck at the first import.

    Checked in a SUBPROCESS: this process has almost certainly touched the CAN
    modules by now, and a test of import side effects that depends on what
    other tests ran first is not a test.
    """
    import subprocess

    script = (
        "import sys; import dog5_sdk;"
        " robot = dog5_sdk.Dog5Sim(pose='crouch'); robot.start(); robot.read();"
        " leaked = [m for m in ('can', 'motorbus', 'motor_library')"
        "           if m in sys.modules];"
        " print('LEAKED' if leaked else 'CLEAN', leaked)"
    )
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ, PYTHONPATH=root)
    out = subprocess.run([sys.executable, "-c", script], capture_output=True,
                         text=True, env=env, cwd=root)
    assert out.returncode == 0, out.stderr
    assert out.stdout.startswith("CLEAN"), (
        f"the simulator path imported the CAN layer: {out.stdout.strip()}")


# ---------------------------------------------------------------------------
# 2. the joint order is ONE order
# ---------------------------------------------------------------------------
def test_joint_order_matches_the_model():
    import mujoco

    model = mujoco.MjModel.from_xml_path(d5.MODEL_XML)
    names = [joint.model_name for joint in d5.HARDWARE_JOINTS]

    # the MJCF's own hinge order, skipping the free joint
    hinges = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
              for i in range(model.njnt)
              if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    assert hinges == names, (
        f"the MJCF hinge order {hinges} is not the hardware-map order {names}; "
        "a 12-vector would mean different things in the two backends")

    actuators = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                 for i in range(model.nu)]
    assert actuators == [f"{name}_motor" for name in names]

    # ctrl must BE torque: gear 1, on the joint, no transmission scaling
    assert np.allclose(model.actuator_gear[:, 0], 1.0), (
        "actuator gear is not 1, so data.ctrl is not joint torque in N*m")

    # qpos[7:] is the same order, contiguous, one dof each
    adr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                               name)] for name in names]
    assert adr == list(range(7, 19)), adr


def test_labels_and_can_ids_line_up():
    calibration.validate()
    assert len(d5.JOINT_LABELS) == 12
    assert list(d5.MOTOR_IDS) == [joint.can_id for joint in d5.HARDWARE_JOINTS]
    assert sorted(d5.MOTOR_IDS) == list(range(1, 13))
    for index, joint in enumerate(d5.HARDWARE_JOINTS):
        assert d5.JOINT_LABELS[index] == f"{joint.leg}_{joint.joint}"


# ---------------------------------------------------------------------------
# 3. kinematics against MuJoCo
# ---------------------------------------------------------------------------
def test_forward_kinematics_matches_mujoco():
    import mujoco

    model = mujoco.MjModel.from_xml_path(d5.MODEL_XML)
    data = mujoco.MjData(model)
    sites = {leg: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,
                                    f"foot_{leg}") for leg in poses.LEGS}

    worst = 0.0
    for q in random_poses(256):
        data.qpos[:3] = 0.0
        data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        data.qpos[7:] = q
        mujoco.mj_forward(model, data)
        for index, leg in enumerate(poses.LEGS):
            ours = kin.foot_position(leg, q[3 * index:3 * index + 3])
            theirs = data.site_xpos[sites[leg]]
            worst = max(worst, float(np.max(np.abs(ours - theirs))))
    assert worst < 1e-12, f"FK disagrees with the MJCF by {worst:.3e} m"


def test_jacobian_matches_mujoco():
    import mujoco

    model = mujoco.MjModel.from_xml_path(d5.MODEL_XML)
    data = mujoco.MjData(model)

    worst = 0.0
    for q in random_poses(64):
        data.qpos[:3] = 0.0
        data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        data.qpos[7:] = q
        mujoco.mj_forward(model, data)
        for index, leg in enumerate(poses.LEGS):
            site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE,
                                     f"foot_{leg}")
            jacp = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jacp, None, site)
            theirs = jacp[:, 6 + 3 * index:6 + 3 * index + 3]
            ours = kin.foot_jacobian(leg, q[3 * index:3 * index + 3])
            worst = max(worst, float(np.max(np.abs(ours - theirs))))
    assert worst < 1e-12, f"the Jacobian disagrees with the MJCF by {worst:.3e}"


# ---------------------------------------------------------------------------
# 4. statics against floating-base inverse dynamics
# ---------------------------------------------------------------------------
def test_statics_matches_inverse_dynamics():
    worst = 0.0
    for q in random_poses(24):
        tau_model, tau_ours = statics.verify_against_model(q.reshape(4, 3))
        worst = max(worst, float(np.max(np.abs(tau_model - tau_ours))))
    assert worst < 1e-9, f"statics is {worst:.3e} N*m from MuJoCo"


def test_leg_gravity_is_not_negligible():
    """The term a massless-leg model drops.  It is worth about half the
    first-run torque cap, which is why it is in the stance law."""
    q = poses.stand_pose(0.15)
    tau_g = np.concatenate([kin.leg_gravity_torque(leg, q[poses.leg_slice(leg)])
                            for leg in poses.LEGS])
    assert np.max(np.abs(tau_g)) > 0.3, np.max(np.abs(tau_g))
    # and the tilted form must reduce to it exactly for a level trunk
    for leg in poses.LEGS:
        section = poses.leg_slice(leg)
        assert np.allclose(statics.leg_gravity_torque_tilted(leg, q[section]),
                           kin.leg_gravity_torque(leg, q[section]), atol=1e-15)


def test_one_mass_everywhere():
    from_model = statics.model_total_mass()
    assert abs(statics.total_mass() - from_model) < 1e-6, (
        f"statics says {statics.total_mass()} kg, the MJCF says {from_model}")
    assert abs(params.MASS_KG - from_model) < 1e-3, (
        f"params.MASS_KG={params.MASS_KG} is not the model's {from_model}")
    assert 0.5 < 4 * statics.leg_mass() / statics.total_mass() < 0.6, (
        "the legs should be about 55 % of this robot")


# ---------------------------------------------------------------------------
# 5. IK inverts FK
# ---------------------------------------------------------------------------
def test_ik_round_trips():
    worst = 0.0
    for q in random_poses(64):
        for index, leg in enumerate(poses.LEGS):
            q_leg = q[3 * index:3 * index + 3]
            target = kin.foot_position(leg, q_leg)
            # seed near, but not at, the answer -- an IK seeded exactly at the
            # solution proves nothing
            seed = q_leg + RNG.normal(0.0, 0.05, 3)
            solved = ik.leg_ik(leg, target, seed)
            worst = max(worst, float(np.linalg.norm(
                kin.foot_position(leg, solved) - target)))
    assert worst < 1e-5, f"IK left the foot {worst * 1e3:.4f} mm out"


def test_ik_reports_failure_instead_of_guessing():
    far = np.array([2.0, 0.0, 0.0])          # metres away; unreachable
    try:
        ik.leg_ik("FL", far, poses.Q_CROUCH[poses.leg_slice("FL")])
    except ik.IKError:
        return
    raise AssertionError("IK returned a pose for an unreachable target")


def test_stand_pose_has_the_height_it_says():
    for height in (0.10, 0.15, 0.19):
        q = poses.stand_pose(height)
        assert abs(poses.hip_height(q) - height) < 1e-6
        for leg in poses.LEGS:
            foot = kin.foot_position(leg, q[poses.leg_slice(leg)])
            assert abs(foot[2] + height) < 1e-6


# ---------------------------------------------------------------------------
# 6. the calibrated joint contract
# ---------------------------------------------------------------------------
def test_encoder_contract_round_trips():
    q = RNG.uniform(-2.0, 2.0, 12)
    dials = calibration.joint_rad_to_motoroutput_deg(q)
    assert np.allclose(calibration.motoroutput_deg_to_joint_rad(dials), q)


def test_encoder_unwrap_counts_wraps_and_centres_the_first_sample():
    unwrap = calibration.EncoderUnwrap()
    # a joint sitting just below zero must read as a small NEGATIVE angle,
    # not as 359.97 deg
    first = unwrap.update(65530)
    assert -0.1 < first < 0.0, first

    gain = 360.0 / 65535.0
    unwrap = calibration.EncoderUnwrap()
    start = unwrap.update(100)
    assert abs(start - 100 * gain) < 1e-9, start
    wrapped = unwrap.update(65500)           # crossed zero going backwards
    assert abs(wrapped - (65500 - 65536) * gain) < 1e-9, wrapped
    back = unwrap.update(100)                # and forwards again
    assert abs(back - start) < 1e-9, back

    # a full turn forward must come back as +360 deg, not as 0
    unwrap = calibration.EncoderUnwrap()
    unwrap.update(100)
    unwrap.update(30000)
    unwrap.update(60000)
    full = unwrap.update(100)
    assert abs(full - (100 + 65536) * gain) < 1e-9, full


def test_hardware_directions_are_the_confirmed_ones():
    negative = {joint.can_id for joint in d5.HARDWARE_JOINTS
                if joint.direction < 0}
    assert negative == {1, 3, 4, 6, 9, 12}, sorted(negative)


# ---------------------------------------------------------------------------
# 7. the safety gate
# ---------------------------------------------------------------------------
def test_gate_ramps_then_caps():
    gate = safety.SafetyGate(tau_cap=2.0, tau_slew=1e6, ramp_s=1.0)
    gate.start(0.0, np.zeros(12))
    q = np.zeros(12)
    assert np.max(gate.apply(np.full(12, 50.0), q, 0.0)) == 0.0
    half = np.max(gate.apply(np.full(12, 50.0), q, 0.5))
    assert abs(half - 1.0) < 1e-9, half
    full = np.max(gate.apply(np.full(12, 50.0), q, 2.0))
    assert abs(full - 2.0) < 1e-9, full


def test_gate_slews():
    gate = safety.SafetyGate(tau_cap=9.0, tau_slew=5.0, ramp_s=0.0001)
    gate.start(0.0, np.zeros(12))
    q = np.zeros(12)
    gate.apply(np.zeros(12), q, 0.0)
    out = gate.apply(np.full(12, 9.0), q, 0.004)      # one 250 Hz tick
    assert np.allclose(out, 5.0 * 0.004), out


def test_gate_blocks_at_a_joint_limit():
    low, high = calibration.soft_limits()
    gate = safety.SafetyGate(tau_cap=9.0, tau_slew=1e6, ramp_s=0.0001)
    gate.start(0.0, low.copy())
    out = gate.apply(np.full(12, -9.0), low.copy(), 0.01)
    assert np.allclose(out, 0.0), "torque pushed further past the low limit"
    gate.previous_tau[:] = 0.0
    out = gate.apply(np.full(12, +9.0), low.copy(), 0.02)
    assert np.all(out > 0.0), "torque back INTO range must not be blocked"


def test_gate_trips_on_confirmed_overspeed_and_not_on_a_single_glitch():
    gate = safety.SafetyGate(tau_cap=1.0)
    gate.start(0.0, np.zeros(12))
    q = np.zeros(12)
    fast = np.zeros(12)
    fast[0] = 7.5                                     # over the soft tier
    # driver says fast, encoder does not move -> a glitch, must NOT trip
    for step in range(1, 10):
        assert gate.estop_reason(q, fast, 0.004 * step) is None

    # both witnesses agree -> trips within the streak
    gate = safety.SafetyGate(tau_cap=1.0)
    gate.start(0.0, np.zeros(12))
    reason = None
    for step in range(1, 10):
        moving = np.zeros(12)
        moving[0] = 0.05 * step                       # 12.5 rad/s at 250 Hz
        reason = gate.estop_reason(moving, fast, 0.004 * step)
        if reason:
            break
    assert reason and "overspeed" in reason, reason


def test_gate_trips_on_a_driver_fault_but_not_on_the_input_lost_latch():
    gate = safety.SafetyGate(tau_cap=1.0)
    gate.start(0.0, np.zeros(12))
    latched = {mid: 0x80 for mid in d5.MOTOR_IDS}     # recoverable over CAN
    assert gate.estop_reason(np.zeros(12), np.zeros(12), 0.01,
                             errors=latched) is None
    faulted = {mid: 0x00 for mid in d5.MOTOR_IDS}
    faulted[d5.MOTOR_IDS[3]] = 0x10                   # over-current
    reason = gate.estop_reason(np.zeros(12), np.zeros(12), 0.02,
                               errors=faulted)
    assert reason and "motor fault" in reason, reason


# ---------------------------------------------------------------------------
# 8. the simulator, end to end
# ---------------------------------------------------------------------------
def test_sim_stands_and_the_estimator_agrees_with_the_truth():
    class Rise(d5.Controller):
        def on_start(self, robot, state):
            self.q0 = state.q.copy()
            self.q1 = poses.stand_pose(0.15)

        def update(self, state):
            u = min(1.0, state.t / 3.0)
            q_ref = self.q0 + u * u * (3.0 - 2.0 * u) * (self.q1 - self.q0)
            return 25.0 * (q_ref - state.q) - 0.8 * state.qd

    with d5.Dog5Sim(pose="crouch") as robot:
        start = robot.read()
        assert np.all(start.contact), "the crouch must start with four feet down"
        result = robot.run(Rise(), duration=6.0, tau_max=8.0, keys=False,
                           verbose=False, log=True)
        final = robot.read()

    assert not result.tripped, result.reason
    assert final.z > 0.10, f"the robot did not stand: z={final.z:.3f} m"
    assert np.all(final.contact)

    # the estimate the CONTROLLER reads, against the simulator's ground truth
    truth = final.extra["z_true"] - params.IMU_BELOW_TRUNK_ORIGIN_M
    assert abs(final.z - truth) < 5e-3, (
        f"FK height {final.z:.4f} vs truth {truth:.4f} m")
    v_error = np.linalg.norm(final.v - final.extra["v_true"])
    assert v_error < 0.05, f"leg odometry is {v_error:.3f} m/s from the truth"

    assert result.log["q"].shape == (result.ticks, 12)
    assert np.max(np.abs(result.log["tau_cmd"])) <= 8.0 + 1e-9


def test_sim_and_hardware_agree_on_the_units_of_torque():
    """ctrl is joint torque, in the same N*m the CAN layer sends.

    A 1 N*m command must produce 1 N*m of joint torque in MuJoCo, and must
    encode to the iq LSB the driver expects.  If these ever diverge, a
    controller tuned in simulation will be wrong on the robot by that factor.
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(d5.MODEL_XML)
    data = mujoco.MjData(model)
    data.ctrl[:] = 1.0
    mujoco.mj_forward(model, data)
    assert np.allclose(data.actuator_force, 1.0)

    from dog5_sdk.motor import motor_gains
    assert abs(motor_gains.torque_gain - 206.04) < 1e-9
    assert abs(1.0 * motor_gains.torque_gain - 206.04) < 1e-9


def test_declared_contact_overrides_the_measured_one():
    with d5.Dog5Sim(pose="crouch") as robot:
        assert np.all(robot.read().contact)
        robot.set_contact([True, False, False, True])       # a trot diagonal
        state = robot.read()
        assert list(state.contact) == [True, False, False, True]
        assert state.extra["n_planted"] == 2
        # two feet is below MIN_PLANTED: the estimate must say so, not lie
        assert not state.extra["estimate_valid"]
        robot.set_contact(None)
        assert np.all(robot.read().contact)


def test_leg_odometry_is_an_exact_identity():
    """Leg odometry is algebra, not a filter, so it should be EXACT.

    Construct the case it claims to solve: four feet planted and not slipping,
    trunk moving at a known velocity.  A planted foot is fixed in the world, so
    each leg's joint velocity is forced to be ``qd_i = -J_i^-1 v`` (level trunk,
    no body rotation).  Fed those, the estimator must return the velocity that
    generated them, to machine precision.  Anything else is a sign error or a
    frame error, and both are invisible in a filtered comparison.
    """
    from dog5_sdk import estimator as est

    q = poses.stand_pose(0.15)
    identity = np.eye(3)
    omega = np.zeros(3)
    for v_world in ([0.3, 0.0, 0.0], [0.0, -0.2, 0.0], [0.1, 0.05, -0.15]):
        v_world = np.asarray(v_world, dtype=float)
        qd = np.zeros(12)
        for index, leg in enumerate(poses.LEGS):
            section = slice(3 * index, 3 * index + 3)
            jac = kin.foot_jacobian(leg, q[section])
            qd[section] = np.linalg.solve(jac, -v_world)
        got, n = est.leg_odometry_velocity(q, qd, identity, omega,
                                           np.ones(4, dtype=bool))
        assert n == 4
        assert np.allclose(got, v_world, atol=1e-12), (got, v_world)

    # and the SDK wrapper carries it through unchanged
    qd = np.zeros(12)
    v_world = np.array([0.25, 0.0, 0.0])
    for index, leg in enumerate(poses.LEGS):
        section = slice(3 * index, 3 * index + 3)
        qd[section] = np.linalg.solve(kin.foot_jacobian(leg, q[section]),
                                      -v_world)
    result = estimate.trunk_estimate(q, qd, np.zeros(3), omega,
                                     np.ones(4, dtype=bool))
    assert result.valid and result.n_planted == 4
    assert np.allclose(result.v, v_world, atol=1e-12), result.v


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------
def main() -> int:
    tests = [(name, value) for name, value in sorted(globals().items())
             if name.startswith("test_") and callable(value)]
    failed = []
    for name, test in tests:
        try:
            test()
        except Exception as exc:                     # noqa: BLE001
            failed.append((name, exc))
            print(f"FAIL  {name}\n        {type(exc).__name__}: {exc}")
        else:
            print(f"ok    {name}", flush=True)
    print()
    if failed:
        print(f"{len(failed)} of {len(tests)} FAILED")
        return 1
    print(f"all {len(tests)} offline gates passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
