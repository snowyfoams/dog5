# =============================================================
# stage0_check.py — Stage 0 validation (NO hardware; runs on any machine w/ numpy)
#
# Validates the analytic Jacobian against a central finite-difference of FK,
# and checks the sign of the restoring torque. This is the single best catch
# for sign/index errors before sending any torque to a motor.
#
#   run:  python3 stage0_check.py
# =============================================================

import numpy as np

import demo_config as cfg
from kinematics import fk, jacobian
from compliance import build_Kc, compliance_torque


def fd_jacobian(q, L, h=1e-6):
    """Central-difference Jacobian (2x1) of fk w.r.t. the single joint angle."""
    Jn = np.zeros((2, 1))
    Jn[:, 0] = (fk(q + h, L) - fk(q - h, L)) / (2.0 * h)
    return Jn


def main():
    L = cfg.L
    rng = np.random.default_rng(0)

    # --- 1. analytic Jacobian vs finite difference over random configs ---
    max_err = 0.0
    for _ in range(1000):
        q = float(rng.uniform(-np.pi, np.pi))
        err = float(np.max(np.abs(jacobian(q, L) - fd_jacobian(q, L))))
        max_err = max(max_err, err)
    assert max_err < 1e-6, f"Jacobian mismatch: max err {max_err:.2e}"

    # --- 2. det/structure sanity: J is purely tangential (J . radial == 0) ---
    for _ in range(100):
        q = float(rng.uniform(-np.pi, np.pi))
        J = jacobian(q, L).ravel()
        radial = np.array([np.cos(q), np.sin(q)])   # outward radial direction
        assert abs(float(J @ radial)) < 1e-9, "J should have no radial component"

    # --- 3. restoring-torque sign: a tangential push must be opposed ---
    # At q=0 the end-effector is at [L, 0] and the tangent (velocity dir) is +y.
    q0 = 0.0
    p_d = fk(q0, L)
    Kc = build_Kc(100.0)
    Dc = np.zeros((2, 2))
    eps = 1e-3
    # simulate the end-effector pushed in +y by perturbing the angle slightly
    q_pushed = q0 + eps          # +q moves the tip toward +y
    tau, _ = compliance_torque(q_pushed, 0.0, p_d, Kc, Dc, L)
    assert tau < 0.0, f"restoring torque should oppose +q push, got tau={tau:+.4f}"

    print("Stage 0 OK")
    print(f"  max |J_analytic - J_fd|   = {max_err:.2e}")
    print(f"  restoring torque (eps={eps}) = {tau:+.5f} N*m  (correctly opposes push)")


if __name__ == "__main__":
    main()
