"""Damped least-squares IK on the MuJoCo Jacobian, with a null-space posture objective.

Primary task: grasp-site position (3D). Each 7-DOF arm keeps 4 redundant DOF, and the
null-space term decides how the load is shared between joints:

  mode="min_norm"  pull toward a nominal posture q_nom            (B1, B2, B4)
  mode="torque"    descend  sum_i w_i * tau_i(q)^2  for the held payload
                   w = 1      -> energy-optimal IK                 (B3)
                   w = w(T)   -> thermal-weighted IK               (proposed)

Static joint torque tau(q) comes from mj_inverse at zero velocity with the payload set,
so the objective is the actual gravity load each joint motor must hold.
"""
from __future__ import annotations

import mujoco
import numpy as np

from .robot import Robot


def _jac(robot: Robot, side: str, q14: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    m, d = robot.model, robot.data
    robot.set_q(q14)
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)
    jacp = np.zeros((3, m.nv))
    mujoco.mj_jacSite(m, d, jacp, None, robot.site[side])
    sl = robot.arm_slice(side)
    return d.site_xpos[robot.site[side]].copy(), jacp[:, robot.dadr[sl]]


def static_torque(robot: Robot, q14: np.ndarray) -> np.ndarray:
    z = np.zeros(14)
    return robot.inverse_dynamics(q14, z, z)


def _torque_grad(robot, side, q14, w, eps=1e-4):
    """Gradient of sum w_i tau_i^2 over this arm's 7 joints (finite differences)."""
    sl = robot.arm_slice(side)
    base = static_torque(robot, q14)[sl]
    f0 = np.sum(w * base**2)
    g = np.zeros(7)
    for k in range(7):
        qp = q14.copy()
        qp[sl.start + k] += eps
        g[k] = (np.sum(w * static_torque(robot, qp)[sl] ** 2) - f0) / eps
    return g, f0


TRUST = np.array([0.6, 0.6, 0.6, 0.6, 0.3, 0.3, 0.3])   # max departure from q_start (rad)


def optimize_posture(robot: Robot, side: str, target: np.ndarray, q_start: np.ndarray,
                     w: np.ndarray, torque_margin: float = 0.85, maxiter: int = 150,
                     trust: np.ndarray = TRUST) -> tuple[np.ndarray, float, bool]:
    """Constrained posture optimization (SLSQP) over one arm's 7 joints.

        minimize   sum_i w_i tau_i(q)^2            (w = copper-loss coefficients x thermal weights)
        subject to grasp_pos(q) = target            (3 equalities)
                   tau_i(q)^2 <= (margin tau_max_i)^2   (static torque inside actuator limits)
                   joint limits, and |q - q_start| <= trust   (bounds)
    The trust region keeps joint-space motion between waypoints close to the min-norm
    motion; without it the optimizer twists the wrist roll by > 1 rad, which is cheap
    statically but needs ~3x the wrist's torque limit to move through in 1 s.

    q_start must already reach the target (e.g. the min-norm solution). Returns
    (q14, position error, success). Falls back to q_start if the optimizer fails.
    """
    from scipy.optimize import minimize

    sl = robot.arm_slice(side)
    w = np.asarray(w, float)
    lim2 = (torque_margin * robot.tau_max[sl]) ** 2
    q_full = q_start.copy()

    def full(x):
        q = q_full.copy()
        q[sl] = x
        return q

    tau0 = static_torque(robot, q_start)[sl]
    f0 = max(float(np.sum(w * tau0**2)), 1e-9)

    def obj(x):
        tau = static_torque(robot, full(x))[sl]
        return float(np.sum(w * tau**2)) / f0

    def eq(x):
        return (robot.grasp_pos(full(x), side) - target) * 100.0      # cm

    def ineq(x):
        tau = static_torque(robot, full(x))[sl]
        return (lim2 - tau**2) / lim2

    x0 = q_start[sl]
    bounds = list(zip(np.maximum(robot.lo[sl] + 1e-3, x0 - trust),
                      np.minimum(robot.hi[sl] - 1e-3, x0 + trust)))
    res = minimize(obj, q_start[sl], method="SLSQP", bounds=bounds,
                   constraints=[{"type": "eq", "fun": eq}, {"type": "ineq", "fun": ineq}],
                   options={"maxiter": maxiter, "ftol": 1e-7})
    q = full(np.clip(res.x, robot.lo[sl], robot.hi[sl]))
    err = float(np.linalg.norm(robot.grasp_pos(q, side) - target))
    feasible = err < 2e-3 and np.all(ineq(q[sl]) >= -1e-3)
    if not feasible or obj(q[sl]) > 1.0 + 1e-6:       # never return something worse than the start
        return q_start.copy(), float(np.linalg.norm(robot.grasp_pos(q_start, side) - target)), False
    return q, err, True


def solve(robot: Robot, side: str, target: np.ndarray, q_init: np.ndarray,
          mode: str = "min_norm", q_nom: np.ndarray | None = None,
          w: np.ndarray | None = None, iters: int = 200, tol: float = 1e-3,
          lam: float = 0.03, k_null: float = 0.5) -> tuple[np.ndarray, float]:
    """Return (q14, position error in m). Only this side's 7 joints change."""
    sl = robot.arm_slice(side)
    q = q_init.copy()
    q_nom = robot.home() if q_nom is None else q_nom
    w = np.ones(7) if w is None else np.asarray(w, float)
    lo, hi = robot.lo[sl], robot.hi[sl]
    err = np.inf
    for it in range(iters):
        p, J = _jac(robot, side, q)
        e = target - p
        err = np.linalg.norm(e)
        # damped pseudo-inverse
        JJt = J @ J.T + lam**2 * np.eye(3)
        Jpinv = J.T @ np.linalg.solve(JJt, np.eye(3))
        N = np.eye(7) - Jpinv @ J
        step = Jpinv @ np.clip(e, -0.05, 0.05)
        if mode == "min_norm":
            null = k_null * (q_nom[sl] - q[sl])
        elif mode == "torque":
            g, f0 = _torque_grad(robot, side, q, w)
            null = -g / (np.linalg.norm(g) + 1e-9) * min(0.05, 0.02 + 0.0 * f0)
        else:
            raise ValueError(mode)
        dq = step + N @ null
        q[sl] = np.clip(q[sl] + dq, lo + 1e-3, hi - 1e-3)
        if err < tol and (mode == "min_norm" and np.linalg.norm(N @ null) < 1e-4):
            break
    # final cleanup: pure position convergence from where the null-space search ended
    for _ in range(50):
        p, J = _jac(robot, side, q)
        e = target - p
        err = np.linalg.norm(e)
        if err < tol * 0.5:
            break
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), e)
        q[sl] = np.clip(q[sl] + dq, lo + 1e-3, hi - 1e-3)
    return q, float(err)
