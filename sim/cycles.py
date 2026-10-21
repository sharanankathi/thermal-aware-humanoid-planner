"""Pick-and-place cycles: slot grid, posture library, and cached cycle torques.

A cycle is 9 joint-space segments with minimum-jerk (quintic) profiles:
  home -> pick_above -> pick -> [grasp] -> pick_above -> place_above -> place -> [release]
  -> place_above -> home
Joint torques are computed with MuJoCo inverse dynamics at 8 Gauss-Legendre points per
segment. Payload torque is linear in box mass (box modeled as a cube whose inertia scales
with mass), so each segment stores tau0 (no box) and tau1 (per kg). The mean tau^2 for any
box mass then follows exactly, without re-running dynamics.

Posture library per (target point, arm), c_i = copper-loss coefficient R/(N eta Kt)^2:
  0      min-norm IK toward the stand pose
  1      energy-optimal: minimize total copper loss  sum c_i tau_i^2  (nominal payload)
  2..8   relieve joint k: minimize sum w_i c_i tau_i^2 with w = 1 + 9 e_k
Postures 1..8 come from constrained SLSQP (ik.optimize_posture) started at posture 0:
grasp point held on target, joint limits, static torque <= 85% of Apollo's limit.
"""
from __future__ import annotations

import hashlib
import json
import pickle
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .ik import optimize_posture, solve
from .robot import Robot

CACHE_DIR = Path(__file__).resolve().parents[1] / ".cache"

PICK_Z = 0.85 + 0.12          # table top + box half-height (grasp at box mid-height)
SHELF_Z = {"std": 1.04 + 0.12, "high": 1.42 + 0.12}
PICK_SLOTS = [(0.36, y, PICK_Z) for y in (-0.10, -0.05, 0.0, 0.05, 0.10)]
PLACE_SLOTS = [(0.46, y, SHELF_Z[s]) for s in ("std", "high") for y in (-0.10, 0.0, 0.10)]
PLACE_SHELF = ["std"] * 3 + ["high"] * 3
LIFT = 0.10                   # approach/retreat height above pick/place
BIMANUAL_HALF_SPAN = 0.13     # each hand 13 cm either side of the box centre (y)
NOMINAL_PAYLOAD_KG = 5.0      # payload used when optimizing the posture library
N_POSTURES = 9
PACES = (1.0, 1.25, 1.5)      # duration multipliers

# (from, to, duration_s at pace 1.0, box held?)
SEGMENTS = [("home", "pick_above", 1.0, False), ("pick_above", "pick", 0.4, False),
            ("pick", "pick", 0.3, True), ("pick", "pick_above", 0.4, True),
            ("pick_above", "place_above", 1.2, True), ("place_above", "place", 0.4, True),
            ("place", "place", 0.3, True), ("place", "place_above", 0.4, False),
            ("place_above", "home", 1.0, False)]
GL_X, GL_W = np.polynomial.legendre.leggauss(8)
GL_S, GL_W = (GL_X + 1) / 2, GL_W / 2       # nodes on [0,1], weights sum to 1


def posture_weights(k: int, c_arm: np.ndarray) -> tuple[str, np.ndarray | None]:
    """c_arm: (7,) copper-loss coefficients for one arm (normalized inside)."""
    if k == 0:
        return "min_norm", None
    w = c_arm / c_arm.mean()
    if k >= 2:
        w = w.copy()
        w[k - 2] *= 10.0
    return "torque", w


def _hand_targets(point, arm):
    p = np.asarray(point, float)
    if arm == "both":
        return {"l": p + [0, BIMANUAL_HALF_SPAN, 0], "r": p - [0, BIMANUAL_HALF_SPAN, 0]}
    return {arm: p}


@dataclass
class Cycle:
    durations: np.ndarray            # (9,) seconds
    tau0_sq: np.ndarray              # (9, 14) mean tau0^2
    cross: np.ndarray                # (9, 14) mean 2*tau0*tau1   (per kg)
    tau1_sq: np.ndarray              # (9, 14) mean tau1^2        (per kg^2)
    tau0_n: np.ndarray               # (9, 8, 14) tau0 at every quadrature node
    tau1_n: np.ndarray               # (9, 8, 14) tau1 (per kg) at every node
    waypoints: dict = field(repr=False, default_factory=dict)

    @property
    def T(self) -> float:
        return float(self.durations.sum())

    def seg_tau_sq(self, mass: float) -> np.ndarray:
        """(9, 14) mean tau^2 per segment for a box of `mass` kg (split already applied)."""
        return self.tau0_sq + mass * self.cross + mass**2 * self.tau1_sq

    def peak(self, mass: float) -> np.ndarray:
        """(14,) max |tau| over the cycle for this box mass (exact at the nodes)."""
        return np.abs(self.tau0_n + mass * self.tau1_n).max(axis=(0, 1))

    def mean_tau_sq(self, mass: float) -> np.ndarray:
        return (self.seg_tau_sq(mass) * self.durations[:, None]).sum(0) / self.T


class CycleLibrary:
    """Builds and caches IK postures and cycle torques for every slot pair, arm, posture, pace."""

    def __init__(self, robot: Robot, loss_c: np.ndarray, rebuild: bool = False,
                 verbose: bool = True):
        self.robot = robot
        self.verbose = verbose
        self.c = np.asarray(loss_c, float)
        cfg = dict(pick=PICK_SLOTS, place=PLACE_SLOTS, lift=LIFT, span=BIMANUAL_HALF_SPAN,
                   nom=NOMINAL_PAYLOAD_KG, seg=SEGMENTS, paces=PACES, c=np.round(self.c, 6).tolist(),
                   v=7)
        key = hashlib.md5(json.dumps(cfg, default=str).encode()).hexdigest()[:10]
        self.path = CACHE_DIR / f"cycles_{key}.pkl"
        if self.path.exists() and not rebuild:
            with open(self.path, "rb") as f:
                self.postures, self.reach, self.cycles = pickle.load(f)
        else:
            self._build()

    # ------------------------------------------------------------ IK library
    def _points(self):
        pts = {}
        for i, p in enumerate(PICK_SLOTS):
            pts[("pick", i)] = np.array(p)
            pts[("pick_above", i)] = np.array(p) + [0, 0, LIFT]
        for j, p in enumerate(PLACE_SLOTS):
            pts[("place", j)] = np.array(p)
            pts[("place_above", j)] = np.array(p) + [0, 0, LIFT]
        return pts

    def _build(self):
        r = self.robot
        home = r.home()
        self.postures, self.reach = {}, {}
        pts = self._points()
        n = 0
        for (kind, idx), point in pts.items():
            for arm in ("l", "r", "both"):
                targets = _hand_targets(point, arm)
                # posture 0: min-norm IK; postures 1..8: constrained optimization from it
                q_mn, ok_mn = home.copy(), True
                for side, t in targets.items():
                    r.set_payload(side, NOMINAL_PAYLOAD_KG / len(targets))
                    q_mn, err = solve(r, side, t, q_mn, mode="min_norm")
                    r.set_payload(side, 0.0)
                    ok_mn &= err < 5e-3
                for k in range(N_POSTURES):
                    q, ok = q_mn.copy(), ok_mn
                    if k > 0 and ok_mn:
                        for side, t in targets.items():
                            _, w = posture_weights(k, self.c[r.arm_slice(side)])
                            r.set_payload(side, NOMINAL_PAYLOAD_KG / len(targets))
                            q, err, _ = optimize_posture(r, side, t, q, w)
                            r.set_payload(side, 0.0)
                            ok &= err < 5e-3
                    self.postures[(kind, idx, arm, k)] = q
                    self.reach[(kind, idx, arm, k)] = ok
                    n += 1
            if self.verbose:
                print(f"  IK {kind}[{idx}] done ({n} solves)", flush=True)
        self.cycles = {}
        for i in range(len(PICK_SLOTS)):
            for j in range(len(PLACE_SLOTS)):
                for arm in ("l", "r", "both"):
                    for k in range(N_POSTURES):
                        if not self.reachable(i, j, arm, k):
                            continue
                        for pace in PACES:
                            self.cycles[(i, j, arm, k, pace)] = self._cycle(i, j, arm, k, pace)
        CACHE_DIR.mkdir(exist_ok=True)
        with open(self.path, "wb") as f:
            pickle.dump((self.postures, self.reach, self.cycles), f)

    def reachable(self, i, j, arm, k=0) -> bool:
        keys = [("pick", i), ("pick_above", i), ("place", j), ("place_above", j)]
        return all(self.reach[(a, b, arm, k)] for a, b in keys)

    # ------------------------------------------------------------ dynamics
    def _cycle(self, i, j, arm, k, pace) -> Cycle:
        r = self.robot
        home = r.home()
        wp = {"home": home}
        for name, idx in [("pick", i), ("pick_above", i), ("place", j), ("place_above", j)]:
            wp[name] = self.postures[(name, idx, arm, k)]
        sides = ("l", "r") if arm == "both" else (arm,)
        nseg = len(SEGMENTS)
        dur = np.array([s[2] for s in SEGMENTS]) * pace
        t0sq, cr, t1sq = (np.zeros((nseg, 14)) for _ in range(3))
        n0, n1 = np.zeros((nseg, len(GL_S), 14)), np.zeros((nseg, len(GL_S), 14))
        for si, (a, b, _, held) in enumerate(SEGMENTS):
            qa, qb, T = wp[a], wp[b], dur[si]
            dq = qb - qa
            tau0 = np.zeros((len(GL_S), 14))
            tau1 = np.zeros((len(GL_S), 14))
            for n_, s in enumerate(GL_S):
                h = 10 * s**3 - 15 * s**4 + 6 * s**5             # min-jerk
                hd = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
                hdd = (60 * s - 180 * s**2 + 120 * s**3) / T**2
                q, qd, qdd = qa + h * dq, hd * dq, hdd * dq
                for sd in ("l", "r"):
                    r.set_payload(sd, 0.0)
                tau0[n_] = r.inverse_dynamics(q, qd, qdd)
                if held:
                    for sd in sides:   # 1 kg total, split across hands
                        r.set_payload(sd, 1.0 / len(sides))
                    tau1[n_] = r.inverse_dynamics(q, qd, qdd) - tau0[n_]
                    for sd in sides:
                        r.set_payload(sd, 0.0)
            t0sq[si] = GL_W @ tau0**2
            cr[si] = GL_W @ (2 * tau0 * tau1)
            t1sq[si] = GL_W @ tau1**2
            n0[si], n1[si] = tau0, tau1
        return Cycle(dur, t0sq, cr, t1sq, n0, n1, wp)

    def get(self, i, j, arm, k, pace) -> Cycle:
        return self.cycles[(i, j, arm, k, pace)]

    def feasible(self, i, j, arm, k, pace, mass) -> bool:
        """Peak joint torque within Apollo's limits for this box (hard constraint)."""
        return bool(np.all(self.cycles[(i, j, arm, k, pace)].peak(mass) <= self.robot.tau_max))

    def postures_ok(self, i, j, arm, pace, mass) -> list[int]:
        return [k for k in range(N_POSTURES) if self.feasible(i, j, arm, k, pace, mass)]

    def idle_tau_sq(self) -> np.ndarray:
        """Static tau^2 with both arms resting at the stand pose (no box)."""
        r = self.robot
        z = np.zeros(14)
        r.set_payload("l", 0.0)
        r.set_payload("r", 0.0)
        return r.inverse_dynamics(r.home(), z, z) ** 2
