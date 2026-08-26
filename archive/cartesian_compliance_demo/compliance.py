# =============================================================
# compliance.py — Cartesian stiffness control law (pure math, no I/O)
#
#   F   = -K_c (p - p_d) - D_c p_dot        (task-space PD / Level-1 impedance)
#   tau = J^T F                              (no gravity term: horizontal plane)
#
# For 1-DOF, J is 2x1 and purely tangential, so K_c = k I behaves as a
# tangential spring (radial errors project to zero through J^T).
# =============================================================

import numpy as np
from kinematics import fk, jacobian


def build_Kc(k) -> np.ndarray:
    """Isotropic 2x2 Cartesian stiffness K_c = k * I [N/m]."""
    return np.eye(2) * float(k)


def build_Dc(Kc, zeta, m_eff) -> np.ndarray:
    """Per-axis critical-damping seed: d_i = 2 zeta sqrt(k_i m_eff)."""
    k_diag = np.diag(Kc)
    d = 2.0 * zeta * np.sqrt(np.maximum(k_diag, 0.0) * m_eff)
    return np.diag(d)


def compliance_torque(q, qdot, p_d, Kc, Dc, L):
    """
    Compute the joint torque realizing the Cartesian spring/damper.

    Args:
        q     : joint angle [rad] (scalar)
        qdot  : joint velocity [rad/s] (scalar, filtered)
        p_d   : task-space equilibrium [x, y] [m]
        Kc,Dc : 2x2 Cartesian stiffness / damping
        L     : link length [m]

    Returns:
        tau  : scalar joint torque [N*m]
        diag : dict with p, e, F, pdot, J for logging / safety
    """
    p = fk(q, L)
    J = jacobian(q, L)                  # 2x1
    pdot = J.ravel() * float(qdot)      # 2-vector  (= J @ [qdot])
    e = p - p_d
    F = -Kc @ e - Dc @ pdot
    tau = float(J.ravel() @ F)          # scalar:  J^T F  ==  sum(J_i F_i)
    diag = {"p": p, "e": e, "F": F, "pdot": pdot, "J": J}
    return tau, diag
