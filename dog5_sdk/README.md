# DOG5 SDK

Write one controller. Run it in MuJoCo, then run **the same class, unchanged**
on the robot — twelve CAN servos at 250 Hz per motor.

```python
import dog5_sdk as d5

class Stand(d5.Controller):
    def on_start(self, robot, state):
        self.q_ref = d5.stand_pose(0.15)

    def update(self, state):                 # -> twelve joint torques, N*m
        return 25.0 * (self.q_ref - state.q) - 0.8 * state.qd

with d5.Dog5Sim(pose="crouch") as robot:     # or d5.Dog5Hardware()
    robot.run(Stand(), duration=6.0, tau_max=8.0)
```

Everything under that — the geometry, the mass properties, the calibrated
encoder contract, the CAN protocol, the safety gate, the state estimator — is
the code that has run on the real DOG5, extracted from the research repository
and shipped here as one self-contained package.

---

## 中文快速开始

```bash
pip install -e .                        # 安装（numpy + mujoco + python-can）
python tests/test_all.py                # 全部离线测试，约 1 分钟，不需要机器人
python examples/01_look_at_the_robot.py --viewer    # 看一眼模型
python examples/02_sim_stand.py --viewer --realtime # 仿真里站起来
```

写你自己的控制器：复制 `examples/04_write_your_own.py`，只改 `update(state)`
一个函数，返回 12 个关节力矩（N·m），顺序固定为
`[FL, FR, RL, RR] × [髋外摆, 髋俯仰, 膝]`。

三种运行方式，同一个控制器类，代码一行不用改：

| 命令 | 跑在哪 | 需要什么 |
|---|---|---|
| `python examples/04_write_your_own.py` | MuJoCo 仿真 | 一台笔记本 |
| `python examples/04_write_your_own.py --fake-hardware` | 完整 CAN 通信链路（12 个模拟驱动器） | 一台笔记本 |
| `python examples/04_write_your_own.py --hardware` | 真机 | can0 + 24 V + **吊起来的机器人** |

**上真机之前**：先在仿真里跑通；把机器人**吊起来**；`tau_max` 保持默认的
1.0 N·m。这个数字不是保守，是这套力矩栈唯一被验证过的起点。运行中 `x` 停机，
空格键松力矩，Ctrl-C 两者都做。详见下面的 [Safety](#safety)。

---

## Install

```bash
cd dog5_sdk
pip install -e .            # or: pip install -r requirements.txt
python tests/test_all.py    # ~1 min, no robot, no CAN adapter
```

`numpy` and `mujoco` are required. `python-can` is only needed to talk to the
robot: `import dog5_sdk` and `Dog5Sim` never touch it, so the simulator runs on
a laptop with no adapter installed. Python 3.9+.

## What you get

| module | what is in it |
|---|---|
| `Dog5Sim` | MuJoCo, on the robot's own MJCF and meshes |
| `Dog5Hardware` | SocketCAN, twelve LK/K-TECH drivers, 250 Hz per motor |
| `Controller`, `RobotState` | the interface you implement, and what you are handed |
| `safety.SafetyGate` | ramp, cap, joint-limit block, slew, and the e-stop trips |
| `poses` | the canonical joint order and the named poses |
| `kinematics` | forward kinematics, foot Jacobians, leg-gravity torque |
| `ik` | damped-least-squares inverse kinematics |
| `statics` | mass properties, the fused chain walk, the MuJoCo cross-check |
| `estimator`, `estimate` | trunk height, leg-odometry velocity, foot-plane attitude |
| `calibration` | encoder ↔ joint angle, soft limits, the 0x19 set-zero |
| `motor` | the CAN library itself: `MotorBus`, `LKMotor`, the unit gains |
| `fake_bus` | twelve simulated drivers, for running the CAN path with no robot |
| `params` | every constant the shipped stand and trot use |

Plus `model/dog5.xml` and its four meshes, `examples/`, `tools/` and `tests/`.

**What is not in it: a controller.** This is the machine and the harness. The
control law is the exercise.

## The joint order, once

Every 12-vector in this SDK — `q`, `qd`, `tau`, the joint limits, the MJCF's
actuators and its `qpos[7:]` — is

```
[FL, FR, RL, RR] × [abduction, hip pitch, knee]      index = 3*leg + joint
```

```
 idx  joint       CAN  dir     idx  joint       CAN  dir
   0  FL_abd        7   +1       6  RL_abd        4   -1
   1  FL_pitch      8   +1       7  RL_pitch      5   +1
   2  FL_knee       9   -1       8  RL_knee       6   -1
   3  FR_abd       10   +1       9  RR_abd        1   -1
   4  FR_pitch     11   +1      10  RR_pitch      2   +1
   5  FR_knee      12   -1      11  RR_knee       3   -1
```

`tests/test_offline.py` gates this order against the MJCF, so a 12-vector
cannot come to mean two different things in the two backends.

The calibrated encoder contract has no software offset and no gearbox divide:

```
motoroutput_deg = raw_encoder * 360/65535
joint_rad       = radians(direction * motoroutput_deg)
```

The zero lives in each driver's own encoder-offset register, written once at
the calibration pose by `tools/calibrate_zero.py --set-zero`.

## Writing a controller

Subclass `Controller` and implement `update`:

```python
class MyController(d5.Controller):
    def on_start(self, robot, state): ...      # optional
    def update(self, state):                   # required -> (12,) N*m, or None
        ...
    def on_stop(self, state, reason): ...      # optional
    def on_estop(self, state, reason): ...     # optional
```

`state` is a `RobotState`:

| field | shape | meaning |
|---|---|---|
| `t` | float | seconds since the run armed |
| `q`, `qd` | (12,) | joint angles and rates, rad, rad/s |
| `tau` | (12,) | **measured** joint torque, N·m |
| `rpy`, `omega` | (3,) | trunk attitude, and body-frame rates |
| `contact` | (4,) | planted mask, FL FR RL RR |
| `z` | float | floor to trunk bottom, m, from FK through the planted feet |
| `v` | (3,) | world-frame trunk velocity, from leg odometry |
| `extra` | dict | temperatures, error bytes, bus load; in sim, the ground truth |

plus `state.foot_positions()` and `state.foot_jacobians()`, both `{leg: array}`
in the trunk frame.

The runner puts your torque through the safety gate, runs the trips, paces the
loop, and on hardware schedules the CAN bus and recovers the input-lost latch —
the same code in both backends.

### Four things that catch people out

1. **Do not call `robot.move_to` or `robot.crouch` from inside `update`.**
   Those are position-mode commands and they fight your torque. Use them
   between runs.
2. **`state.contact` on hardware is what you declare.** There are no foot
   switches. `robot.set_contact(mask)` — a gait controller owns its own
   schedule; the default is all four planted, which is right for a stand and
   wrong for anything with a swing phase.
3. **`state.z` and `state.v` come from the planted feet.** Below three feet
   down they are not determined; check `state.extra["estimate_valid"]` before
   multiplying either by a gain. In torque mode a fiction becomes real force.
4. **The first hardware run is `tau_max=1.0`, robot suspended.**

## Running it

```bash
python examples/01_look_at_the_robot.py [--viewer]   # model, FK/IK, the map
python examples/02_sim_stand.py [--viewer --realtime]# a joint-PD stand
python examples/03_cartesian_impedance.py            # J^T f + leg gravity
python examples/04_write_your_own.py                 # the template
python examples/05_hardware_stand.py --dry-run       # the hardware runner
```

On the robot host:

```bash
sudo ip link set can0 up type can bitrate 1000000
python tools/can_ping.py                             # do all twelve answer?
sudo HOME=$HOME chrt -f 50 python3 examples/05_hardware_stand.py
```

`chrt -f 50` is worth it: it cuts the loop's scheduler jitter measurably on a
Pi-class host.

### Calibration

```bash
python tools/calibrate_zero.py --observe     # which ID is which, which way
python tools/calibrate_zero.py --verify      # at the zero pose, all twelve ~0
python tools/calibrate_zero.py --set-zero    # WRITE (read the warnings first)
```

All three hold every motor at zero torque throughout: nothing is commanded to
move, you pose the robot by hand, and it stays back-drivable.

## Safety

Every hardware run moves a 5.8 kg machine with twelve motors that can bite.

- **Suspend the robot** for its first runs. Not "hold it".
- **`tau_max=1.0`** to start. Raise it in stages, from logs, and not past 3.0
  without a reason you can state.
- **`x` stops, space limps, Ctrl-C does both.** Keep a hand on the power.
- **Run it in `Dog5Sim` first.** Every trip you find there is one you do not
  find with the robot in the air.

**The trips are not a safety net you can lean on.** Chapter 4 of the research
repository documents a run where the tilt trip did not fire, the robot ended up
on its belly at 2.4× body weight, **and read perfectly level while doing so** —
because a robot lying flat is level. Every gate can only see what it measures.

## What the simulator is and is not

It has the robot's real geometry, its CAD inertials, the rotor inertia
reflected through the 10:1 reduction, and direct-torque actuators with
`gear=1` — so `data.ctrl` **is** joint torque in N·m, in the same units and the
same order as the torque that goes out over CAN. That is what makes one
controller run on both.

It has no CAN bus: no missed replies, no input-lost latch, no driver
temperature, no round-robin phasing. It has a rigid floor and MuJoCo's contact
model, not a real one. **A controller that stands here will not necessarily
stand on the robot** — the stack this SDK comes from documents a run where it
did not. What the simulator is good for is what is true in both: the geometry,
the mass distribution, the sign of every joint, the shape of your control law,
and whether it saturates the cap.

`fake_bus.FakeDriverBus` covers the other half: it decodes your frames with the
real protocol, so a wrong CAN ID, a flipped direction, a unit error or a
misplaced payload byte shows up on a laptop instead of on the robot. It models
no physics at all. Between them:

| | geometry & control law | CAN protocol & units |
|---|---|---|
| `Dog5Sim` | yes | no |
| `FakeDriverBus` | no | yes |
| the robot | yes | yes |

## Provenance, and keeping it honest

This SDK is extracted from the DOG5 research repository. The files it copies
are copied, not re-derived:

- **verbatim, byte for byte** — `motor/` (the CAN library), `kinematics.py`,
  `hardware_map.py`, `params.py`, `model/dog5.xml` and the meshes;
- **header replaced, body verbatim** — `statics.py`, `estimator.py`, whose
  originals reach their siblings through a `sys.path` preamble that is wrong
  inside a package.

`tools/sync_from_repo.py` regenerates all of it from `../src`, and
`tests/test_sync.py` fails if any copy has drifted. With no research tree
beside the SDK — the normal case after a handover — both simply skip, and the
shipped copies are the source of truth.

The claim that matters: **the CAN layer in this package is the code that ran on
the robot**, not a port of it.

## The IMU

`Dog5Hardware(ahrs=...)` takes any object with `sample()` and `is_stale(s)`,
where the sample carries `roll_deg`, `pitch_deg`, `yaw_deg` and the three
`*_rate_dps`. The FDILink DETA10 vendor SDK (`fdilink_imu`) is **not on PyPI
and is not redistributed here**; install it on the robot host and wrap it.

With no AHRS, roll and pitch fall back to the **foot-plane** attitude from the
encoders, yaw is 0 and `omega` is zeros. That is attitude relative to the floor
the robot stands on, not relative to gravity — fine for a static check, not a
substitute for an IMU in a balance loop. You are warned, loudly, at arm time.

## Tests

```bash
python tests/test_all.py            # all three suites
python tests/test_offline.py        # model, kinematics, statics, gate, sim
python tests/test_hardware_path.py  # the CAN path against fake drivers
python tests/test_sync.py           # drift against the research repo
```

The cross-checks are the interesting ones: forward kinematics and the foot
Jacobians against MuJoCo to 1e-12, the statics against MuJoCo's floating-base
inverse dynamics to 1e-9 N·m (including the leg-gravity term a massless-leg
model drops, which is worth 0.48 N·m — half the first-run torque cap), and leg
odometry against the identity it claims to be, to machine precision.

## Hardware

- 12 × LK / K-TECH brushless servo drivers, CAN IDs 1–12, 10:1 reduction
- SocketCAN `can0` at 1 Mbit/s
- 1 × FDILink DETA10 AHRS on the trunk (optional for this SDK)
- 5.8151 kg, of which the legs are 3.196 kg — **55 %**, which is why leg
  gravity is in the stance law and not an afterthought
- a Pi-class Linux host running the loop at 250 Hz per motor

## Licence

MIT, as the parent repository. The IMU vendor SDK and the Fusion→MJCF exporter
are third party and are not included.

> Zhan Zhi, *DOG5: a quadruped from CAN bus to quasi-dynamic gait*, Nanyang
> Technological University, 2026. https://github.com/snowyfoams/dog5
