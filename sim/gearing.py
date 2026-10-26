"""Re-gear the arm joints of a CycleLibrary: fixed ratio changes or ideal two-speed shifting.

Cached cycle torques come from MuJoCo inverse dynamics on Apollo, whose joints carry Apollo's own
reflected rotor inertia (`armature`). For a joint driven by a TBM motor through ratio N:

    motor torque   tau_m = tau_link / (N eta) + N J_m qdd
    tau_link       = tau_ID - armature_apollo * qdd          (Apollo's own rotor inertia removed)

The thermal pipeline converts joint torque to motor torque with the baseline ratio N0, so each
re-geared cycle stores the equivalent joint torque at N0:

    tau_eq = tau_m N0 eta = (N0 / N) (tau_link + eta N^2 J_m qdd)

Copper loss therefore falls as 1/N^2 for the load term but rises with the reflected rotor inertia
(N^2 J_m) for the acceleration term. The torque-limit check in CycleLibrary (peak tau_eq vs the
joint limit) is then the motor peak-torque check, because the baseline ratios were sized so that
motor peak torque x N0 x eta matches Apollo's joint limits.

Speed limit: N |qdot| <= omega_max at every quadrature node (TBM(S)-6013-A: 4,000 rpm at rated
power on a 48 V bus, Kollmorgen TBM selection guide).

Two-speed (ideal upper bound): each joint has ratios N0 and k N0 and, for every trajectory segment,
uses the high ratio when the segment's peak speed allows it. Shifting takes no time and adds no mass
or loss; a real two-speed joint can only do worse.
"""
from __future__ import annotations

import copy

import numpy as np

from .cycles import GL_S, GL_W, SEGMENTS, Cycle

J_M = 1.41e-5                         # kg m^2, TBM(S)-6013-A rotor inertia [datasheet]
OMEGA_MAX = 4000 * 2 * np.pi / 60     # rad/s at the motor, 48 V [datasheet, rated-power speed]


def _kinematics(cyc: Cycle):
    """(nseg, nnode, 14) joint velocity and acceleration at the cycle's quadrature nodes."""
    nseg, nn = len(SEGMENTS), len(GL_S)
    qd = np.zeros((nseg, nn, 14))
    qdd = np.zeros((nseg, nn, 14))
    for si, (a, b, _, _) in enumerate(SEGMENTS):
        T = cyc.durations[si]
        dq = cyc.waypoints[b] - cyc.waypoints[a]
        for n, s in enumerate(GL_S):
            qd[si, n] = (30 * s**2 - 60 * s**3 + 30 * s**4) / T * dq
            qdd[si, n] = (60 * s - 180 * s**2 + 120 * s**3) / T**2 * dq
    return qd, qdd


def regear_cycle(cyc: Cycle, N0, armature, eta, N_seg, eta_seg=None) -> tuple[Cycle, bool]:
    """N_seg: (nseg, 14) ratio used in each segment; eta_seg: (nseg, 14) gear efficiency in each
    segment (default: the baseline eta). Returns (new cycle, speed_ok)."""
    qd, qdd = _kinematics(cyc)
    N = N_seg[:, None, :]
    speed_ok = bool(np.all(N * np.abs(qd) <= OMEGA_MAX * (1 + 1e-9)))
    E = (np.broadcast_to(eta, N_seg.shape) if eta_seg is None else eta_seg)[:, None, :]
    # tau_m = tau_link/(N E) + N J qdd; stored as tau_m * N0 * eta (what the thermal model divides by)
    scale = (N0 * eta)[None, None, :] / (N * E)
    t0 = scale * (cyc.tau0_n - armature * qdd + E * N**2 * J_M * qdd)
    t1 = scale * cyc.tau1_n
    new = copy.copy(cyc)
    new.tau0_n, new.tau1_n = t0, t1
    new.tau0_sq = np.einsum("n,snj->sj", GL_W, t0**2)
    new.cross = np.einsum("n,snj->sj", GL_W, 2 * t0 * t1)
    new.tau1_sq = np.einsum("n,snj->sj", GL_W, t1**2)
    return new, speed_ok


def regear_library(lib, robot, params, mult, mode: str = "fixed", lo_mult=None, eta_hi=None):
    """Copy of `lib` with re-geared cycles.

    mult: (14,) ratio multipliers. mode "fixed": every segment uses mult*N0, and a cycle whose
    speed limit is violated is removed (the controller then cannot pick it). mode "twospeed":
    per segment and joint, use mult*N0 when that segment's peak speed allows it, else N0.
    Returns (library, info) where info counts removed cycles and high-gear segment share.
    """
    N0 = np.array([p.N for p in params], float)
    eta = np.array([p.eta for p in params], float)
    armature = np.array([robot.model.dof_armature[robot.dadr[i]] for i in range(14)])
    mult = np.asarray(mult, float)
    lo = np.ones(14) if lo_mult is None else np.asarray(lo_mult, float)
    new = copy.copy(lib)
    new.cycles = {}
    removed, hi, tot = 0, 0, 0
    for key, cyc in lib.cycles.items():
        qd, _ = _kinematics(cyc)
        seg_peak = np.abs(qd).max(axis=1)                       # (nseg, 14)
        if mode == "fixed":
            N_seg = np.broadcast_to(mult * N0, seg_peak.shape).copy()
        elif mode == "twospeed":
            hi_ok = (mult * N0)[None, :] * seg_peak <= OMEGA_MAX
            N_seg = np.where(hi_ok, mult * N0, lo * N0)
            hi += int(hi_ok[:, mult > lo].sum())
            tot += int(hi_ok[:, mult > lo].size)
        else:
            raise ValueError(mode)
        eta_seg = None
        if eta_hi is not None:   # efficiency of the raised ratio (lower for higher strain-wave ratios)
            eta_seg = np.where(N_seg > lo * N0 + 1e-9, eta_hi, np.broadcast_to(eta, N_seg.shape))
        c2, ok = regear_cycle(cyc, N0, armature, eta, N_seg, eta_seg)
        if not ok:
            removed += 1
            continue
        new.cycles[key] = c2
    _patch_reach(new, lib)
    return new, {"removed": removed, "total": len(lib.cycles),
                 "high_gear_share": hi / tot if tot else np.nan}


def _patch_reach(new, old):
    """Cycles removed for speed must look infeasible to the controllers."""
    get, feasible = new.get, new.feasible

    def feasible2(i, j, arm, k, pace, mass):
        if (i, j, arm, k, pace) not in new.cycles:
            return False
        return feasible(i, j, arm, k, pace, mass)

    new.feasible = feasible2
