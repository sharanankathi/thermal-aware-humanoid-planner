"""Controllers. All share one interface: reset(), plan(box, t) -> Plan, after(...).

B1 Dominant arm     right arm always, min-norm IK, full pace
B2 Round-robin      alternate arms, min-norm IK, full pace
B3 Energy-optimal   nearest arm, minimize sum tau^2 posture, full pace
B4 Reactive derate  nearest arm, min-norm IK; slow down past a temperature threshold
B5 RR + energy IK   alternate arms, minimize copper-loss posture, full pace (thermally blind)
P  Thermal budget   forecast-based arm/both choice, thermal-weighted posture, planned pacing
"""
from __future__ import annotations

import numpy as np

from .cycles import N_POSTURES, PACES, PICK_SLOTS
from .shift import Plan


def nearest_arm(box) -> str:
    y = PICK_SLOTS[box.pick][1]
    return "l" if y > 0 else "r"          # ties (y = 0) go to the dominant right arm


class Controller:
    name = "base"

    def reset(self, th, lib, sc, idle):
        self.th, self.lib, self.sc, self.idle = th, lib, sc, idle

    def loss(self, box, arm, k, pace=1.0, w=None):
        """Mean copper-loss proxy sum_j w_j c_j tau_j^2 for one cycle (W at 25 C if w = 1)."""
        tsq = self.lib.get(box.pick, box.place, arm, k, pace).mean_tau_sq(box.mass)
        c = self.lib.c if w is None else self.lib.c * w
        return float((c * tsq).sum())

    def min_norm_or_feasible(self, box, arm, pace=1.0):
        """Posture 0 (min-norm) unless it breaks a torque limit; then the lowest-loss feasible one."""
        ok = self.lib.postures_ok(box.pick, box.place, arm, pace, box.mass)
        if 0 in ok or not ok:
            return 0
        return min(ok, key=lambda k: self.loss(box, arm, k, pace))

    def after(self, box, plan, t):
        pass


class DominantArm(Controller):
    name = "B1 dominant"

    def plan(self, box, t):
        return Plan("r", self.min_norm_or_feasible(box, "r"))


class RoundRobin(Controller):
    name = "B2 round-robin"

    def reset(self, *a):
        super().reset(*a)
        self.next = "r"

    def plan(self, box, t):
        arm, self.next = self.next, ("l" if self.next == "r" else "r")
        return Plan(arm, self.min_norm_or_feasible(box, arm))


class EnergyOptimal(Controller):
    name = "B3 energy-opt"

    def plan(self, box, t):
        arm = nearest_arm(box)
        # feasible posture with the lowest total copper loss for this box
        ok = self.lib.postures_ok(box.pick, box.place, arm, 1.0, box.mass) or [0]
        return Plan(arm, min(ok, key=lambda k: self.loss(box, arm, k)))


class RoundRobinEnergy(Controller):
    """Strongest thermally blind baseline: alternate arms AND energy-optimal posture."""
    name = "B5 round-robin + energy IK"

    def reset(self, *a):
        super().reset(*a)
        self.next = "r"

    def plan(self, box, t):
        arm, self.next = self.next, ("l" if self.next == "r" else "r")
        ok = self.lib.postures_ok(box.pick, box.place, arm, 1.0, box.mass) or [0]
        return Plan(arm, min(ok, key=lambda k: self.loss(box, arm, k)))


class ReactiveDerate(Controller):
    """Commercial-style protection: past T_on, slow to the slowest pace and add pauses."""
    name = "B4 reactive"

    def __init__(self, T_on: float = 120.0, T_off: float = 110.0, pause: float = 4.0):
        self.T_on, self.T_off, self.pause = T_on, T_off, pause

    def reset(self, *a):
        super().reset(*a)
        self.derated = False

    def plan(self, box, t):
        hot = self.th.T_w.max()
        if hot >= self.T_on:
            self.derated = True
        elif hot <= self.T_off:
            self.derated = False
        arm = nearest_arm(box)
        if self.derated:
            return Plan(arm, self.min_norm_or_feasible(box, arm, PACES[-1]), PACES[-1], self.pause)
        return Plan(arm, self.min_norm_or_feasible(box, arm))


class ThermalBudget(Controller):
    """Proposed controller (three levers in priority order, all from a temperature forecast).

    1. Mode and pace: candidate policies = {alternate single arms, both arms} x pace x pause.
       Each policy's mean joint load is forecast forward; choose the highest-throughput
       policy whose forecast keeps every winding below the limit until shift end.
    2. Arm: under 'alternate', the arm whose hottest joint has the most forecast budget.
    3. Posture: minimize sum_j w_j tau_j^2 with w_j = ((T_lim - T_amb)/(T_lim - T_j))^2,
       which reduces to energy-optimal IK when cold and protects hot joints when warm.
    Switches (levers) allow the ablations: use_arm, use_posture, use_pace.
    """

    PAUSES = (0.0, 1.0, 2.0, 4.0, 8.0, 15.0)

    def __init__(self, use_arm=True, use_posture=True, use_pace=True, margin_s=60.0,
                 replan_every=5, forecast_dt=10.0, adapt=True, name="P thermal-budget"):
        self.use_arm, self.use_posture, self.use_pace = use_arm, use_posture, use_pace
        self.adapt = adapt
        self.margin = margin_s
        self.replan_every, self.fdt = replan_every, forecast_dt
        self.name = name

    def reset(self, *a):
        super().reset(*a)
        self.mass_ema = None
        self.n_since = 10**9
        self.policy = None
        self.prev = None          # (Tw, Th, steps) at the previous decision, for adaptation
        self.th.gain[:] = 1.0

    def _weights(self):
        th = self.th
        return ((th.T_limit - th.T_amb) / np.maximum(th.T_limit - th.T_w, 1.0)) ** 2

    def _posture(self, box, arm, w):
        """Thermal-weighted copper loss over feasible postures. None if no posture is feasible."""
        ok = self.lib.postures_ok(box.pick, box.place, arm, PACES[-1], box.mass)
        ok = [k for k in ok if self.lib.feasible(box.pick, box.place, arm, k, 1.0, box.mass)]
        if not ok:
            return None
        if not self.use_posture:
            return 0 if 0 in ok else min(ok, key=lambda k: self.loss(box, arm, k))
        return min(ok, key=lambda k: self.loss(box, arm, k, w=w))

    def _load(self, cyc, mass, arm):
        """Energy-weighted segment tau^2 and duration of one cycle (idle arm at stand)."""
        seg = cyc.seg_tau_sq(mass)
        if arm in ("l", "r"):
            other = slice(7, 14) if arm == "l" else slice(0, 7)
            seg[:, other] = self.idle[other]
        return (seg * cyc.durations[:, None]).sum(0), cyc.T

    # ------------------------------------------------------------ online model correction
    ADAPT_RATE, GAIN_MIN, GAIN_MAX, MIN_SENS = 0.3, 0.5, 2.5, 0.02

    def _adapt(self):
        """Per-joint copper-loss gain from the last box: measured vs predicted winding rise.

        Predict the box from the previous measured state with gains g and g(1+d); the
        difference gives the sensitivity of the prediction to each joint's gain. A damped
        Newton step moves g toward agreement. Joints with little load (sensitivity under
        MIN_SENS degC) are left alone. Absorbs errors in R, Kt and thermal resistance.
        """
        Tw0, Th0, steps = self.prev
        g = self.th.gain.copy()
        d = 0.05
        G = np.stack([g, g * (1 + d)])
        Tw_pred, _ = self.th.rollout(np.stack([Tw0, Tw0]), np.stack([Th0, Th0]), steps, gain=G)
        sens = (Tw_pred[1] - Tw_pred[0]) / (g * d)
        err = self.th.T_w - Tw_pred[0]
        upd = np.abs(sens) > self.MIN_SENS
        g[upd] += self.ADAPT_RATE * err[upd] / sens[upd]
        self.th.gain[:] = np.clip(g, self.GAIN_MIN, self.GAIN_MAX)

    def _steps(self, box, plan):
        cyc = self.lib.get(box.pick, box.place, plan.arm, plan.posture, plan.pace)
        seg = cyc.seg_tau_sq(box.mass)
        if plan.arm in ("l", "r"):
            other = slice(7, 14) if plan.arm == "l" else slice(0, 7)
            seg[:, other] = self.idle[other]
        steps = list(zip(cyc.durations, seg))
        if plan.pause > 0:
            steps.append((plan.pause, self.idle))
        return steps

    def plan(self, box, t):
        if self.adapt and self.prev is not None:
            self._adapt()
        plan = self._plan(box, t)
        if self.adapt:
            self.prev = (self.th.T_w.copy(), self.th.T_h.copy(), self._steps(box, plan))
        return plan

    def _plan(self, box, t):
        lib, idle = self.lib, self.idle
        self.mass_ema = box.mass if self.mass_ema is None else 0.9 * self.mass_ema + 0.1 * box.mass
        m_fc = self.mass_ema
        w = self._weights()
        k = {a: self._posture(box, a, w) for a in ("l", "r", "both")}
        remaining = max(self.sc.shift_s - t, 0.0)
        self.n_since += 1
        if self.policy is None or self.n_since >= self.replan_every:
            self.policy = self._choose_policy(box, k, m_fc, remaining)
            self.n_since = 0
        mode, pace, pause = self.policy
        if mode == "both" and k["both"] is None:
            mode = "alt"
        if mode == "both":
            return Plan("both", k["both"], pace, pause)
        if self.use_arm:
            arm = self._pick_arm(box, k, pace, m_fc, remaining)
        else:
            arm = nearest_arm(box)
        return Plan(arm, k[arm], pace, pause)

    def _choose_policy(self, box, k, m_fc, remaining):
        lib, idle = self.lib, self.idle
        modes = ["alt", "both"] if (self.use_arm and k["both"] is not None) else ["alt"]
        paces = PACES if self.use_pace else (1.0,)
        pauses = self.PAUSES if self.use_pace else (0.0,)
        cands, loads, rates = [], [], []
        for mode in modes:
            for pace in paces:
                if mode == "alt" and not self.use_arm:
                    # arm lever off: the nearest arm does every pick, so forecast that arm alone
                    a = nearest_arm(box)
                    E, T = self._load(lib.get(box.pick, box.place, a, k[a], pace), m_fc, a)
                    n = 1
                elif mode == "alt":
                    El, Tl = self._load(lib.get(box.pick, box.place, "l", k["l"], pace), m_fc, "l")
                    Er, Tr = self._load(lib.get(box.pick, box.place, "r", k["r"], pace), m_fc, "r")
                    E, T, n = El + Er, Tl + Tr, 2
                else:
                    E, T = self._load(lib.get(box.pick, box.place, "both", k["both"], pace),
                                      m_fc, "both")
                    n = 1
                for p in pauses:
                    Ttot = T + n * p
                    cands.append((mode, pace, p))
                    loads.append((E + n * p * idle) / Ttot)
                    rates.append(n / Ttot)
        ttl = self.th.forecast_ttl(np.array(loads), horizon=remaining + self.margin, dt=self.fdt)
        budget = ttl.min(1)
        rates = np.array(rates)
        ok = budget >= remaining + self.margin
        if ok.any():
            idx = int(np.argmax(np.where(ok, rates, -1)))       # fastest safe policy
        else:
            idx = int(np.argmax(budget))                         # survive as long as possible
        return cands[idx]

    def _pick_arm(self, box, k, pace, m_fc, remaining):
        """Arm whose hottest joint has more forecast budget if it took this pick repeatedly."""
        loads = []
        for a in ("l", "r"):
            E, T = self._load(self.lib.get(box.pick, box.place, a, k[a], pace), m_fc, a)
            loads.append(E / T)
        ttl = self.th.forecast_ttl(np.array(loads), horizon=remaining + self.margin, dt=self.fdt)
        b = [ttl[0, 0:7].min(), ttl[1, 7:14].min()]
        if np.isinf(b[0]) and np.isinf(b[1]):
            # both safe: keep the cooler arm working (largest margin on its hottest joint)
            marg = [self.th.T_limit[0:7].min() - self.th.T_w[0:7].max(),
                    self.th.T_limit[7:].min() - self.th.T_w[7:].max()]
            return "l" if marg[0] > marg[1] else "r"
        return "l" if b[0] > b[1] else "r"


def all_controllers():
    return [DominantArm(), RoundRobin(), EnergyOptimal(), ReactiveDerate(), ThermalBudget()]
