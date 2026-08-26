# =============================================================
# safety.py — safety layer (clamp, watchdog, limits, K ramp)
# Active from Stage 1, per spec section 6.
# =============================================================

import numpy as np


def clamp(tau, limit) -> float:
    """Hard-clamp a torque [N*m] to +/- limit. Non-negotiable saturation."""
    return float(np.clip(tau, -limit, limit))


class Watchdog:
    """Latches to TRIPPED if end-point speed or torque exceeds a threshold."""

    def __init__(self, pdot_max, tau_max):
        self.pdot_max = pdot_max
        self.tau_max = tau_max
        self.tripped = False
        self.reason = None

    def check(self, pdot, tau) -> bool:
        speed = float(np.linalg.norm(pdot))
        if speed > self.pdot_max:
            self.tripped = True
            self.reason = f"||p_dot||={speed:.3f} > {self.pdot_max}"
        elif abs(tau) > self.tau_max:
            self.tripped = True
            self.reason = f"|tau|={abs(tau):.3f} > {self.tau_max}"
        return self.tripped


def in_joint_limits(q, q_limits) -> bool:
    """True if joint angle q [rad] is within (low, high)."""
    return q_limits[0] <= q <= q_limits[1]


def k_ramp_scale(t, ramp_sec) -> float:
    """Linear 0->1 stiffness ramp over ramp_sec; saturates at 1."""
    if ramp_sec <= 0:
        return 1.0
    return min(1.0, max(0.0, t / ramp_sec))
