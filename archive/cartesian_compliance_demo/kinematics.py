# =============================================================
# kinematics.py — planar forward kinematics + Jacobian (pure math, no I/O)
#
# 1-DOF now: single revolute joint, one link of length L.
#   end-effector lies on a circle of radius L.
#   p(q)  = [ L cos q , L sin q ]
#   J(q)  = [ -L sin q ; L cos q ]   (2x1, purely tangential)
#
# 2-DOF later (motor id=5, l1=0.12, l2=0.132):
#   x = l1 c1 + l2 c12 ;  y = l1 s1 + l2 s12
#   J = [[-l1 s1 - l2 s12, -l2 s12],
#        [ l1 c1 + l2 c12,  l2 c12]]
#   det J = l1 l2 sin(q2)   (singular at q2 = 0 or pi)
# =============================================================

import numpy as np


def _q1(q) -> float:
    """Accept a scalar or length-1 array and return the joint angle as float."""
    return float(np.atleast_1d(q)[0])


def fk(q, L) -> np.ndarray:
    """Forward kinematics: joint angle q [rad] -> end-effector position [x, y] [m]."""
    q1 = _q1(q)
    return np.array([L * np.cos(q1), L * np.sin(q1)])


def jacobian(q, L) -> np.ndarray:
    """Jacobian dp/dq as a 2x1 matrix so that p_dot = J @ [q_dot]."""
    q1 = _q1(q)
    return np.array([[-L * np.sin(q1)],
                     [ L * np.cos(q1)]])
