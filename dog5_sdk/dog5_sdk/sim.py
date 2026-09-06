"""The MuJoCo backend: the same API as the robot, on the shipped MJCF.

    from dog5_sdk import Dog5Sim, JointPD, poses

    with Dog5Sim(pose="crouch", viewer=True) as robot:
        robot.run(JointPD(poses.stand_pose(0.15)), duration=5.0, tau_max=6.0)

WHAT THE MODEL IS
    ``model/dog5.xml`` is the robot's own MJCF, exported from the CAD and
    shipped here with its four meshes.  Its inertials are the CAD's, its
    ``armature`` is the real rotor inertia reflected through the 10:1
    reduction, and its actuators are direct-drive ``motor`` actuators with
    ``gear=1`` -- so ``data.ctrl`` IS joint torque in N*m, in the same units
    and the same order as the torque you send over CAN.  That is what makes
    one controller run on both.

    Bodies, joints, actuators and ``qpos[7:]`` are all in the canonical order
    (see :mod:`dog5_sdk.poses`), but this module looks every index up BY NAME
    anyway, so a future edit to the XML cannot silently transpose a leg.

WHAT THE SIMULATOR IS NOT
    It has no CAN bus, so no missed replies, no input-lost latch, no driver
    temperature and no 4 ms of round-robin phasing between the twelve joints.
    It has a rigid floor with MuJoCo's contact model, not a real one.  A
    controller that stands here will not necessarily stand on the robot -- the
    stack this SDK comes from documents a run where it did not.  What the
    simulator IS good for is the things that are true in both: the geometry,
    the mass distribution, the sign of every joint, the shape of your control
    law, and whether it saturates the torque cap.
"""
from __future__ import annotations

import math
import os

import numpy as np

from . import params as P
from . import poses
from .base import Dog5Robot
from .estimate import EncoderVelocity, leg_frames_all, trunk_estimate
from .hardware_map import HARDWARE_JOINTS
from .poses import LEGS, N_JOINTS
from .state import RobotState

#: The MJCF and its meshes, shipped inside the package.
MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model")
MODEL_XML = os.path.join(MODEL_DIR, "dog5.xml")

#: Per-foot normal force counted as contact.  The robot weighs 57 N, so a
#: foot carrying 1 N is carrying nothing; this is a contact test, not a load
#: measurement.
CONTACT_FORCE_N = 1.0

#: Gains for the simulated position mode (``move_to``).  They stand in for the
#: drivers' own 0xA4 loop, which is a current-mode servo inside the motor and
#: is not modelled -- see the note in :meth:`Dog5Sim.move_to`.
POSITION_KP = 80.0
POSITION_KD = 3.0


def rpy_from_mat(mat) -> np.ndarray:
    """Roll, pitch, yaw from a body->world rotation matrix.

    Roll and pitch use the SAME definition as the hardware estimator
    (``estimator.C_from_rp`` / ``dynamic_model.attitude_rp``): the inertial
    up-axis expressed in the body frame.  A level robot reads 0, 0.
    """
    mat = np.asarray(mat, dtype=float).reshape(3, 3)
    g_b = mat.T @ np.array([0.0, 0.0, 1.0])      # up-axis, in the body frame
    roll = math.atan2(g_b[1], g_b[2])
    pitch = math.atan2(-g_b[0], math.hypot(g_b[1], g_b[2]))
    yaw = math.atan2(mat[1, 0], mat[0, 0])
    return np.array([roll, pitch, yaw])


class Dog5Sim(Dog5Robot):
    """DOG5 in MuJoCo, behind the :class:`dog5_sdk.base.Dog5Robot` interface.

    pose      starting pose: "crouch", "zero", "stand", or a (12,) vector
    viewer    open the interactive MuJoCo viewer (needs a display)
    realtime  pace the loop to wall-clock; off by default, because a headless
              run should go as fast as it can
    gravity   set to 0.0 to float the robot -- useful for checking a leg
              controller without the trunk falling over
    """

    name = "dog5-sim"

    def __init__(self, pose="crouch", *, viewer: bool = False,
                 realtime: bool = None, gravity: float = None,
                 control_hz: float = P.CONTROL_HZ, xml: str = MODEL_XML):
        super().__init__()
        import mujoco                                       # noqa: PLC0415

        self._mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.control_hz = float(control_hz)

        if gravity is not None:
            self.model.opt.gravity[:] = (0.0, 0.0, -abs(float(gravity)))

        substeps = self.dt / self.model.opt.timestep
        self._substeps = max(1, int(round(substeps)))
        if abs(substeps - self._substeps) > 1e-9:
            raise ValueError(
                f"control period {self.dt * 1e3:.3f} ms is not a whole number "
                f"of MJCF timesteps ({self.model.opt.timestep * 1e3:.3f} ms); "
                "change control_hz or the XML timestep")

        self._index_by_name()
        self._realtime = bool(viewer) if realtime is None else bool(realtime)
        self._want_viewer = bool(viewer)
        self.viewer = None
        self._encoder_qd = EncoderVelocity()
        self._wall_deadline = None
        self._ctrl = np.zeros(N_JOINTS)

        self.reset(pose)

    # -- indexing --------------------------------------------------------
    def _index_by_name(self) -> None:
        mj, model = self._mj, self.model

        def ident(kind, name):
            index = mj.mj_name2id(model, kind, name)
            if index < 0:
                raise ValueError(f"{name!r} is not in {os.path.basename(MODEL_XML)}")
            return index

        names = [joint.model_name for joint in HARDWARE_JOINTS]
        self.joint_ids = [ident(mj.mjtObj.mjOBJ_JOINT, n) for n in names]
        self.actuator_ids = [ident(mj.mjtObj.mjOBJ_ACTUATOR, f"{n}_motor")
                             for n in names]
        self.qpos_adr = np.asarray([model.jnt_qposadr[i] for i in self.joint_ids])
        self.dof_adr = np.asarray([model.jnt_dofadr[i] for i in self.joint_ids])
        self.foot_site_ids = [ident(mj.mjtObj.mjOBJ_SITE, f"foot_{leg}")
                              for leg in LEGS]
        self.trunk_id = ident(mj.mjtObj.mjOBJ_BODY, "trunk")

        #: {geom id: leg index}, for the contact scan.
        self._leg_of_geom = {
            ident(mj.mjtObj.mjOBJ_GEOM, f"foot_{leg}"): index
            for index, leg in enumerate(LEGS)
        }

        self._sensor = {}
        for index in range(model.nsensor):
            name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_SENSOR, index)
            adr, dim = model.sensor_adr[index], model.sensor_dim[index]
            self._sensor[name] = slice(adr, adr + dim)

    # -- state -----------------------------------------------------------
    def reset(self, pose="crouch", *, height: float = None, yaw: float = 0.0,
              settle_s: float = 0.15) -> "Dog5Sim":
        """Put the robot at `pose`, resting on the floor, and zero the clock.

        `pose` is "zero", "crouch", "stand" or a (12,) joint vector.  Unless
        `height` is given the trunk is dropped so the lowest collidable point
        just touches the floor, which is what you want in every case except
        deliberately studying a fall.

        `settle_s` then holds that pose against gravity for a moment.  Placed
        at exact contact the feet carry ZERO force, so nothing is in contact
        yet and the first tick would report ``contact=0000`` and a NaN height.
        Settling is what makes the first state a real standing state.  Set it
        to 0 to see the untouched placement.
        """
        q = self._resolve_pose(pose)
        self._mj.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_adr] = q
        self.data.qvel[:] = 0.0

        half = 0.5 * float(yaw)
        self.data.qpos[3:7] = (math.cos(half), 0.0, 0.0, math.sin(half))
        self.data.qpos[0:3] = (0.0, 0.0, 1.0)     # provisional, see below

        if height is None:
            # Drop the robot until its LOWEST collidable point touches the
            # floor -- measured, not assumed to be a foot.  At the crouch it is
            # a foot; at the calibration pose the trunk itself is on the
            # ground, and starting a run with the trunk mesh through the floor
            # is a large impulse on the first step.
            self._mj.mj_forward(self.model, self.data)
            height = 1.0 - self._lowest_collidable_z()
        self.data.qpos[0:3] = (0.0, 0.0, float(height))

        self.data.ctrl[:] = 0.0
        self._ctrl[:] = 0.0
        self._mj.mj_forward(self.model, self.data)

        if settle_s > 0:
            self._hold_pose(q, int(round(settle_s * self.control_hz)))

        self.data.ctrl[:] = 0.0
        self._ctrl[:] = 0.0
        self._encoder_qd = EncoderVelocity()
        self._mj.mj_forward(self.model, self.data)
        self.t = 0.0
        self._wall_deadline = None
        return self

    def _hold_pose(self, q_ref, ticks: int) -> None:
        """Stiff joint PD onto `q_ref` for `ticks` control periods, no clock."""
        q_ref = np.asarray(q_ref, dtype=float).reshape(N_JOINTS)
        for _ in range(max(0, ticks)):
            q = self.data.qpos[self.qpos_adr]
            qd = self.data.qvel[self.dof_adr]
            tau = np.clip(POSITION_KP * (q_ref - q) - POSITION_KD * qd,
                          -9.0, 9.0)
            self.data.ctrl[self.actuator_ids] = tau
            for _ in range(self._substeps):
                self._mj.mj_step(self.model, self.data)

    @staticmethod
    def _resolve_pose(pose) -> np.ndarray:
        if isinstance(pose, str):
            table = {"zero": poses.Q_ZERO, "crouch": poses.Q_CROUCH}
            if pose == "stand":
                return poses.stand_pose()
            if pose not in table:
                raise ValueError(f"unknown pose {pose!r}; use 'zero', "
                                 "'crouch', 'stand' or a (12,) vector")
            return np.asarray(table[pose], dtype=float)
        return np.asarray(pose, dtype=float).reshape(N_JOINTS)

    def _lowest_collidable_z(self) -> float:
        """World z of the lowest point of any geom that can hit the floor.

        Exact for the shapes this model uses -- spheres (the feet), boxes (the
        thigh pads) and the trunk mesh, whose vertices are transformed and
        minimised over.  Geoms the MJCF has switched off for collision (the
        visual leg meshes, ``contype=0``) are ignored, because they cannot
        stop the robot from falling through.
        """
        mj, model, data = self._mj, self.model, self.data
        floor = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")
        lowest = float("inf")
        for geom in range(model.ngeom):
            if geom == floor:
                continue
            collides = ((model.geom_contype[geom] & model.geom_conaffinity[floor])
                        or (model.geom_contype[floor]
                            & model.geom_conaffinity[geom]))
            if not collides:
                continue
            pos = data.geom_xpos[geom]
            rot = data.geom_xmat[geom].reshape(3, 3)
            size = model.geom_size[geom]
            kind = model.geom_type[geom]
            if kind == mj.mjtGeom.mjGEOM_SPHERE:
                bottom = pos[2] - size[0]
            elif kind == mj.mjtGeom.mjGEOM_BOX:
                bottom = pos[2] - float(np.abs(rot[2, :3]) @ size[:3])
            elif kind == mj.mjtGeom.mjGEOM_MESH:
                mesh = model.geom_dataid[geom]
                start = model.mesh_vertadr[mesh]
                verts = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
                bottom = float(np.min(verts @ rot[2, :3]) + pos[2])
            else:                      # capsule, cylinder, plane, ...
                bottom = pos[2] - float(model.geom_rbound[geom])
            lowest = min(lowest, bottom)
        return 0.0 if not np.isfinite(lowest) else lowest

    def _contact_forces(self) -> np.ndarray:
        """Per-foot normal contact force, N, in leg order."""
        forces = np.zeros(4)
        buffer = np.zeros(6)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            leg = self._leg_of_geom.get(int(contact.geom1))
            if leg is None:
                leg = self._leg_of_geom.get(int(contact.geom2))
            if leg is None:
                continue
            self._mj.mj_contactForce(self.model, self.data, index, buffer)
            forces[leg] += abs(float(buffer[0]))   # normal, in the contact frame
        return forces

    def _read_raw(self) -> RobotState:
        data = self.data
        q = data.qpos[self.qpos_adr].copy()
        qd_true = data.qvel[self.dof_adr].copy()
        tau = data.actuator_force[self.actuator_ids].copy()

        quat = data.sensordata[self._sensor["imu_quat"]].copy()
        mat = np.zeros(9)
        self._mj.mju_quat2Mat(mat, quat)
        rpy = rpy_from_mat(mat.reshape(3, 3))
        omega = data.sensordata[self._sensor["imu_gyro"]].copy()

        forces = self._contact_forces()
        contact = self.contact_mask(measured=forces > CONTACT_FORCE_N)

        qd_enc = self._encoder_qd.update(self.t, q)
        frames = leg_frames_all(q)
        estimate = trunk_estimate(q, qd_enc, rpy, omega, contact, frames=frames)

        trunk_pos = data.sensordata[self._sensor["trunk_pos"]].copy()
        trunk_vel = data.sensordata[self._sensor["trunk_linvel"]].copy()

        return RobotState(
            t=self.t, q=q, qd=qd_true, tau=tau, rpy=rpy, omega=omega,
            contact=contact,
            z=estimate.z if estimate.n_planted else float("nan"),
            v=estimate.v,
            extra={
                # ground truth -- for CHECKING the estimate, never for control
                "z_true": float(trunk_pos[2]),
                "v_true": trunk_vel,
                "qd_true": qd_true,
                "qd_encoder": qd_enc,
                # the same fields the hardware backend fills
                "estimate_valid": estimate.valid,
                "n_planted": estimate.n_planted,
                "z_hip": estimate.z_hip,
                "rpy_fk": estimate.rpy_fk,
                "contact_force_n": forces,
                "frames": frames,
                "model": self.model,
                "data": data,
            },
        )

    # -- actuation -------------------------------------------------------
    def _send_torque(self, tau) -> None:
        self._ctrl = np.asarray(tau, dtype=float).reshape(N_JOINTS)
        self.data.ctrl[self.actuator_ids] = self._ctrl

    def _tick(self) -> None:
        for _ in range(self._substeps):
            self._mj.mj_step(self.model, self.data)
        self.t += self.dt
        if self.viewer is not None:
            self.viewer.sync()
        if self._realtime:
            if self._wall_deadline is None:
                import time
                self._wall_deadline = time.perf_counter()
            self._wall_deadline += self.dt
            self._pace(self._wall_deadline)

    # -- lifecycle -------------------------------------------------------
    def _arm(self) -> None:
        if self._want_viewer and self.viewer is None:
            import mujoco.viewer                            # noqa: PLC0415
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self._mj.mj_forward(self.model, self.data)

    def _disarm(self, reason: str) -> None:
        self.data.ctrl[:] = 0.0
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None

    # -- position mode ---------------------------------------------------
    def move_to(self, q_target, duration_s: float = 3.0, *,
                kp: float = POSITION_KP, kd: float = POSITION_KD,
                settle_s: float = 0.5, verbose: bool = True) -> np.ndarray:
        """Drive to `q_target` with a stiff joint PD, then hold for `settle_s`.

        This STANDS IN for the drivers' native 0xA4 position loop, which is a
        current controller inside the motor and is not modelled here.  It gets
        the robot into a pose so a torque controller can take over; it is not
        a claim that the real position mode behaves like a PD with these gains.
        Do not use it as a controller -- write one and hand it to
        :meth:`run`.
        """
        if not self.started:
            self.start()
        target = np.asarray(q_target, dtype=float).reshape(N_JOINTS)
        start_q = self.data.qpos[self.qpos_adr].copy()
        steps = max(1, int(round(duration_s * self.control_hz)))
        hold = max(0, int(round(settle_s * self.control_hz)))

        for step in range(steps + hold):
            u = min(1.0, (step + 1) / steps)
            # cubic ease so the pose does not start with a velocity step
            blend = u * u * (3.0 - 2.0 * u)
            q_ref = start_q + blend * (target - start_q)
            q = self.data.qpos[self.qpos_adr]
            qd = self.data.qvel[self.dof_adr]
            self._send_torque(np.clip(kp * (q_ref - q) - kd * qd, -9.0, 9.0))
            self._tick()

        q = self.data.qpos[self.qpos_adr].copy()
        error = float(np.max(np.abs(q - target)))
        if verbose:
            print(f"[{self.name}] move_to: settled {error * 1e3:.1f} mrad "
                  f"from the target")
        return q
