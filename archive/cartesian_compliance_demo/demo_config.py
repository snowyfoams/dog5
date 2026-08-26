# =============================================================
# demo_config.py  — parameters for the 1-DOF Cartesian compliance demo
#
# Self-contained: does NOT import the jump-robot config.py (which
# references undefined symbols). Gains copied from
# ../Code_ForCanBusTest/config.py and confirmed against motor_library.py.
# =============================================================

import os
import platform

import numpy as np

# ---------- Mechanism (single link, end-effector on a circle of radius L) ----------
L = 0.132                       # link length [m]  (distal/shank, per user)

# ---------- Motor / CAN bus ----------
MOTOR_ID      = 1               # joint 1 motor id  (2-DOF later adds id=5)
DIR           = +1             # +1: counter-clockwise positive (matches dir_1)

# Bus backend is chosen by platform so the same demo runs in both places:
#   * macOS dev Mac  -> PEAK PCAN-USB via python-can's "pcan" backend, which
#     loads the MacCAN libPCBUSB.dylib userspace driver (no sudo, no kext).
#   * Linux robot host -> SocketCAN "can0" (bring up with `ip link set can0 ...`).
if platform.system().lower() == "darwin":
    BUS_INTERFACE = "pcan"
    BUS_CHANNEL   = "PCAN_USBBUS1"
    # python-can resolves libPCBUSB.dylib via find_library("PCBUSB"), which only
    # searches the dyld paths. Our no-sudo install lives in ~/.local/lib, not a
    # default search dir on recent macOS, so prepend the candidate dirs here
    # (set before any bus is opened; ctypes reads os.environ at resolve time).
    _pcbusb_dirs = [os.path.expanduser("~/.local/lib"), "/usr/local/lib"]
    _cur = os.environ.get("DYLD_LIBRARY_PATH", "")
    _have = _cur.split(":") if _cur else []
    os.environ["DYLD_LIBRARY_PATH"] = ":".join(
        [d for d in _pcbusb_dirs if d not in _have] + _have
    )
else:
    BUS_INTERFACE = "socketcan"
    BUS_CHANNEL   = "can0"
BITRATE       = 1_000_000

# ---------- Driver gains (from config.py / motor_library.py) ----------
ENCODER_GAIN   = 360 / 65535    # encoder_value * gain -> OUTPUT-shaft degrees
VEL_STATE_GAIN = 10             # status-2 speed:  spd_raw / gain -> output dps
TORQUE_GAIN    = 206.04         # N*m <-> iq integer  (iq = tau * gain)
IQ_SAT         = 2048           # torque_loop_control iq hard limit (+/-)
                                # -> max commandable torque = 2048/206.04 ~= 9.94 N*m

# ---------- Control ----------
LOOP_HZ          = 400          # target control rate
K_RAMP_SEC       = 2.0          # ramp stiffness 0 -> target at startup (never step)
ZETA             = 0.8          # damping ratio seed
M_EFF            = 0.3          # effective end-point mass [kg] for damping seed
K_SPRING         = 300.0        # default target Cartesian stiffness [N/m]
VEL_FILTER_ALPHA = 0.7          # first-order low-pass on q_dot (0..1, higher = smoother)

# ---------- Graceful shutdown (wind-down before release) ----------
STOP_DAMPING     = 0.6          # joint damper on exit [N*m per rad/s] -> brake to rest
STOP_QDOT        = 0.15         # treat as "stopped" below this joint speed [rad/s]
STOP_TIMEOUT_SEC = 1.5          # give up damping after this long and release anyway

# ---------- Joint friction (measured) ----------
# Empirical breakaway/static friction of the joint (~0.11 N*m), from a low-k run
# where the link parked instead of returning. Used only by the live plot to show
# when commanded torque is below friction (spring too weak to move the link).
JOINT_FRICTION_NM = 0.11

# ---------- Safety ----------
TAU_CLAMP_NM      = 3.0                         # normal per-joint torque clamp [N*m]
WATCHDOG_TAU_MAX  = 8.0                         # e-stop torque threshold [N*m] (< 9.94 iq sat)
QDOT_MAX          = 0.8                         # e-stop end-point speed ||p_dot|| [m/s]
Q_LIMITS_RAD      = (np.deg2rad(-170), np.deg2rad(170))   # joint angle bounds [rad]
