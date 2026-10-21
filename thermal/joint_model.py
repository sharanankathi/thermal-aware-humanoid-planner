"""Two-node (winding + housing) thermal model for electric joint actuators.

    C_w dT_w/dt = P_cu(tau, T_w) - (T_w - T_h)/R_wh
    C_h dT_h/dt = (T_w - T_h)/R_wh - (T_h - T_amb)/R_ha

    P_cu = I^2 R_ll(T_w),   I = tau_m / Kt(T_w),   tau_m = |tau_joint| / (N * eta)
    R_ll(T) = R_25 [1 + alpha_cu (T - 25)]
    Kt(T)   linear between datasheet cold (25 C) and hot (155 C) values (magnet weakening)

Conventions follow the Kollmorgen datasheet (6-step drive, Adc, line-to-line R), so
copper loss is I_dc^2 * R_ll. Magnet temperature is taken equal to winding temperature,
which is conservative (rotor runs cooler than the winding).

All functions are vectorized: pass arrays of shape (n_joints,) for multi-joint use.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class ActuatorParams:
    name: str
    N: float            # gear ratio
    eta: float          # gear efficiency
    Kt_cold: float      # Nm/A at 25 C
    Kt_hot: float       # Nm/A at T_hot
    T_hot: float        # C, winding temp for Kt_hot
    R25: float          # ohm, line-to-line at 25 C
    alpha_cu: float     # 1/C
    R_wh: float         # C/W
    R_ha: float         # C/W
    C_w: float          # J/K
    C_h: float          # J/K
    T_limit: float      # C, protective-stop threshold
    Tc: float           # Nm, datasheet continuous stall torque (motor side)
    Ic: float           # A, datasheet continuous current
    TPR: float          # C/W, datasheet total thermal resistance
    T_amb_rated: float  # C

    @property
    def tau_w(self) -> float:
        return self.C_w * self.R_wh

    @property
    def tau_h(self) -> float:
        return self.C_h * self.R_ha

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ActuatorParams":
        d = yaml.safe_load(Path(path).read_text())
        m, g, s = d["motor"], d["gear"], d["thermal_split"]
        TPR = m["TPR_C_per_W"]
        R_wh = s["R_wh_fraction"] * TPR
        C_w = s["winding_node_mass_kg"] * s["winding_node_cp_J_per_kgK"]
        C_h = sum(p["mass_kg"] * p["cp_J_per_kgK"] for p in s["housing_node"])
        return cls(
            name=m["name"], N=g["ratio"], eta=g["efficiency"],
            Kt_cold=m["Kt_cold_Nm_per_Adc"], Kt_hot=m["Kt_hot_Nm_per_Adc"],
            T_hot=m["T_rated_winding_C"], R25=m["R_ll_25C_ohm"], alpha_cu=m["alpha_cu_per_C"],
            R_wh=R_wh, R_ha=TPR - R_wh, C_w=C_w, C_h=C_h,
            T_limit=d["limits"]["T_limit_C"], Tc=m["Tc_Nm"], Ic=m["Ic_Adc"], TPR=TPR,
            T_amb_rated=m["T_ambient_rated_C"],
        )

    @classmethod
    def from_library(cls, motor: str, path: str | Path, gear_ratio: float | None = None
                     ) -> "ActuatorParams":
        """Build from params/actuators.yaml (one datasheet, several motor sizes)."""
        d = yaml.safe_load(Path(path).read_text())
        c, s, m = d["common"], d["common"]["thermal_split"], d["motors"][motor]
        R_wh = s["R_wh_fraction"] * m["TPR"]
        mass = m["mass"]
        C_w = s["winding_node_mass_fraction"] * mass * s["winding_cp_J_per_kgK"]
        C_h = mass * (s["housing_al_kg_per_motor_kg"] * s["housing_al_cp"]
                      + s["gear_steel_kg_per_motor_kg"] * s["gear_steel_cp"])
        return cls(
            name=motor, N=gear_ratio or m["gear_ratio"], eta=c["gear_efficiency"],
            Kt_cold=m["Kt_cold"], Kt_hot=m["Kt_hot"], T_hot=c["T_rated_winding_C"],
            R25=m["R25"], alpha_cu=c["alpha_cu_per_C"], R_wh=R_wh, R_ha=m["TPR"] - R_wh,
            C_w=C_w, C_h=C_h, T_limit=c["T_limit_C"], Tc=m["Tc"], Ic=m["Ic"], TPR=m["TPR"],
            T_amb_rated=c["T_ambient_rated_C"],
        )

    def scaled(self, **factors: float) -> "ActuatorParams":
        """Copy with selected fields multiplied, e.g. scaled(R_ha=1.3, C_w=0.7) for ablations."""
        vals = {k: getattr(self, k) for k in self.__dataclass_fields__}
        for k, f in factors.items():
            vals[k] = vals[k] * f
        return ActuatorParams(**vals)


# ---------------------------------------------------------------- physics
def resistance(p: ActuatorParams, T_w):
    return p.R25 * (1.0 + p.alpha_cu * (np.asarray(T_w) - 25.0))


def torque_constant(p: ActuatorParams, T_w):
    frac = (np.asarray(T_w) - 25.0) / (p.T_hot - 25.0)
    return p.Kt_cold + (p.Kt_hot - p.Kt_cold) * frac


def motor_torque(p: ActuatorParams, tau_joint):
    """Motor-side torque. Gear loss charged in both directions (conservative)."""
    return np.abs(np.asarray(tau_joint)) / (p.N * p.eta)


def copper_loss_motor(p: ActuatorParams, tau_m, T_w):
    I = np.asarray(tau_m) / torque_constant(p, T_w)
    return I**2 * resistance(p, T_w)


def copper_loss(p: ActuatorParams, tau_joint, T_w):
    return copper_loss_motor(p, motor_torque(p, tau_joint), T_w)


def derivatives(p: ActuatorParams, T_w, T_h, P_in, T_amb):
    q_wh = (T_w - T_h) / p.R_wh
    dTw = (P_in - q_wh) / p.C_w
    dTh = (q_wh - (T_h - T_amb) / p.R_ha) / p.C_h
    return dTw, dTh


# ---------------------------------------------------------------- simulation
class JointThermalModel:
    """Thermal state for n joints sharing (or not) one parameter set.

    step() uses RK2 (Heun). dt up to ~1 s is accurate (winding tau ~ 50 s);
    in the MuJoCo loop call it at the control rate or with cycle-averaged tau^2.
    """

    def __init__(self, params: ActuatorParams | list[ActuatorParams], n: int | None = None,
                 T_amb: float = 25.0):
        if isinstance(params, ActuatorParams):
            params = [params] * (n or 1)
        self.params = params
        self.n = len(params)
        self.T_amb = T_amb
        self.T_w = np.full(self.n, T_amb, dtype=float)
        self.T_h = np.full(self.n, T_amb, dtype=float)
        # vectorized parameter arrays (one entry per joint)
        a = lambda k: np.array([getattr(q, k) for q in params], dtype=float)
        self._Neta = a("N") * a("eta")
        self._Ktc, self._Kth, self._Thot = a("Kt_cold"), a("Kt_hot"), a("T_hot")
        self._R25, self._alpha = a("R25"), a("alpha_cu")
        self._Rwh, self._Rha, self._Cw, self._Ch = a("R_wh"), a("R_ha"), a("C_w"), a("C_h")
        self.T_limit = a("T_limit")
        self.gain = np.ones(self.n)   # multiplies copper loss; fitted online by the controller

    def reset(self, T0: float | None = None):
        T0 = self.T_amb if T0 is None else T0
        self.T_w[:] = T0
        self.T_h[:] = T0

    def power(self, Tw, tau_sq):
        """Copper loss per joint (W) for mean-square joint torque tau_sq."""
        Kt = self._Ktc + (self._Kth - self._Ktc) * (Tw - 25.0) / (self._Thot - 25.0)
        R = self._R25 * (1.0 + self._alpha * (Tw - 25.0))
        return self.gain * tau_sq / (self._Neta * Kt) ** 2 * R

    def _deriv(self, Tw, Th, tau_sq):
        q_wh = (Tw - Th) / self._Rwh
        dTw = (self.power(Tw, tau_sq) - q_wh) / self._Cw
        dTh = (q_wh - (Th - self.T_amb) / self._Rha) / self._Ch
        return dTw, dTh

    def step_tau_sq(self, tau_sq, dt: float):
        """Advance by dt given the mean of tau^2 over the interval (exact for heat input)."""
        tau_sq = np.asarray(tau_sq, dtype=float)
        k1w, k1h = self._deriv(self.T_w, self.T_h, tau_sq)
        k2w, k2h = self._deriv(self.T_w + dt * k1w, self.T_h + dt * k1h, tau_sq)
        self.T_w += 0.5 * dt * (k1w + k2w)
        self.T_h += 0.5 * dt * (k1h + k2h)
        return self.T_w

    def rollout(self, Tw, Th, steps, gain=None):
        """Propagate (Tw, Th) through [(dt, tau_sq), ...] without touching the live state.
        Tw/Th may be (n,) or (G, n); gain overrides self.gain for this call."""
        saved = self.gain
        if gain is not None:
            self.gain = gain
        Tw, Th = np.array(Tw, float), np.array(Th, float)
        try:
            for dt, tsq in steps:
                k1w, k1h = self._deriv(Tw, Th, tsq)
                k2w, k2h = self._deriv(Tw + dt * k1w, Th + dt * k1h, tsq)
                Tw = Tw + 0.5 * dt * (k1w + k2w)
                Th = Th + 0.5 * dt * (k1h + k2h)
        finally:
            self.gain = saved
        return Tw, Th

    def state(self):
        return self.T_w.copy(), self.T_h.copy()

    def set_state(self, s):
        self.T_w[:], self.T_h[:] = s

    def step(self, tau_joint, dt: float):
        """Advance by dt with joint torques held (or tau as RMS over dt)."""
        tau_sq = np.asarray(tau_joint, dtype=float) ** 2
        k1w, k1h = self._deriv(self.T_w, self.T_h, tau_sq)
        k2w, k2h = self._deriv(self.T_w + dt * k1w, self.T_h + dt * k1h, tau_sq)
        self.T_w += 0.5 * dt * (k1w + k2w)
        self.T_h += 0.5 * dt * (k1h + k2h)
        return self.T_w.copy()

    def at_limit(self) -> np.ndarray:
        return self.T_w >= self.T_limit

    # ------------------------------------------------------------ budget
    def time_to_limit(self, tau_rms, horizon: float = 7200.0, dt: float = 2.0) -> np.ndarray:
        """Forecast seconds until each winding reaches T_limit if tau_rms is sustained.

        Rolls a copy of the current state forward. Returns np.inf for joints that
        plateau below the limit within the horizon.
        """
        Tw, Th = self.T_w.copy(), self.T_h.copy()
        tau_sq = np.asarray(tau_rms, dtype=float) ** 2
        lim = np.array([p.T_limit for p in self.params])
        out = np.full(self.n, np.inf)
        out[Tw >= lim] = 0.0
        t = 0.0
        while t < horizon and np.isinf(out).any():
            k1w, k1h = self._deriv(Tw, Th, tau_sq)
            k2w, k2h = self._deriv(Tw + dt * k1w, Th + dt * k1h, tau_sq)
            Tw_new = Tw + 0.5 * dt * (k1w + k2w)
            Th = Th + 0.5 * dt * (k1h + k2h)
            t += dt
            hit = np.isinf(out) & (Tw_new >= lim)
            if hit.any():  # linear interpolation inside the step
                frac = (lim[hit] - Tw[hit]) / np.maximum(Tw_new[hit] - Tw[hit], 1e-12)
                out[hit] = t - dt + frac * dt
            Tw = Tw_new
        return out

    def forecast_ttl(self, tau_sq, horizon: float, dt: float = 5.0) -> np.ndarray:
        """Batch forecast: tau_sq (C, n) candidate loads -> (C, n) seconds to T_limit.

        Every candidate starts from the current state and holds its mean tau^2.
        np.inf means the joint stays below the limit for the whole horizon.
        """
        tau_sq = np.atleast_2d(np.asarray(tau_sq, float))
        C = tau_sq.shape[0]
        Tw = np.broadcast_to(self.T_w, (C, self.n)).copy()
        Th = np.broadcast_to(self.T_h, (C, self.n)).copy()
        out = np.where(Tw >= self.T_limit, 0.0, np.inf)
        # Exact shortcut. The two-node network is a cooperative (monotone) system: if the
        # state starts at or below the steady state for this load, it stays below it. So a
        # joint whose steady winding temperature is under the limit, and which starts below
        # its steady state on both nodes, can never reach the limit.
        Rt = self._Rwh + self._Rha
        Tss = np.full_like(Tw, self.T_amb)
        for _ in range(60):
            Tss = self.T_amb + Rt * self.power(np.minimum(Tss, 1e3), tau_sq)
        Thss = self.T_amb + self._Rha * self.power(np.minimum(Tss, 1e3), tau_sq)
        safe = (Tss < self.T_limit) & (Tw <= Tss + 1e-9) & (Th <= Thss + 1e-9)
        live = ~safe & np.isinf(out)
        rows = np.where(live.any(1))[0]
        if rows.size == 0:
            return out
        Tw, Th, tau_sq = Tw[rows], Th[rows], tau_sq[rows]
        sub = out[rows]
        t = 0.0
        while t < horizon and np.isinf(sub[live[rows]]).any():
            k1w, k1h = self._deriv(Tw, Th, tau_sq)
            k2w, k2h = self._deriv(Tw + dt * k1w, Th + dt * k1h, tau_sq)
            Tw_new = Tw + 0.5 * dt * (k1w + k2w)
            Th = Th + 0.5 * dt * (k1h + k2h)
            hit = np.isinf(sub) & (Tw_new >= self.T_limit)
            if hit.any():
                frac = (self.T_limit - Tw) / np.maximum(Tw_new - Tw, 1e-12)
                sub = np.where(hit, t + frac * dt, sub)
            Tw = Tw_new
            t += dt
        out[rows] = sub
        return out

    def _forecast_ttl_bruteforce(self, tau_sq, horizon: float, dt: float = 5.0) -> np.ndarray:
        """Reference implementation without the shortcut (used in tests)."""
        tau_sq = np.atleast_2d(np.asarray(tau_sq, float))
        C = tau_sq.shape[0]
        Tw = np.broadcast_to(self.T_w, (C, self.n)).copy()
        Th = np.broadcast_to(self.T_h, (C, self.n)).copy()
        out = np.where(Tw >= self.T_limit, 0.0, np.inf)
        t = 0.0
        while t < horizon and np.isinf(out).any():
            k1w, k1h = self._deriv(Tw, Th, tau_sq)
            k2w, k2h = self._deriv(Tw + dt * k1w, Th + dt * k1h, tau_sq)
            Tw_new = Tw + 0.5 * dt * (k1w + k2w)
            Th = Th + 0.5 * dt * (k1h + k2h)
            hit = np.isinf(out) & (Tw_new >= self.T_limit)
            if hit.any():
                frac = (self.T_limit - Tw) / np.maximum(Tw_new - Tw, 1e-12)
                out = np.where(hit, t + frac * dt, out)
            Tw = Tw_new
            t += dt
        return out

    def robot_budget(self, tau_rms, **kw) -> float:
        """Robot-level budget: smallest remaining safe time across all joints."""
        return float(np.min(self.time_to_limit(tau_rms, **kw)))


# ---------------------------------------------------------------- steady state
def steady_state_motor(p: ActuatorParams, tau_m: float, T_amb: float, tpr: float | None = None,
                       tol: float = 1e-9, max_iter: int = 500) -> float:
    """Steady winding temperature for a constant motor torque.

    Solves T = T_amb + TPR * P(T) by fixed-point iteration. Returns np.inf when no
    steady state exists (thermal runaway: loss grows faster than conduction removes it).
    """
    tpr = p.R_wh + p.R_ha if tpr is None else tpr
    T = T_amb
    for _ in range(max_iter):
        T_new = T_amb + tpr * copper_loss_motor(p, tau_m, T)
        if not np.isfinite(T_new) or T_new > 1000:
            return np.inf
        if abs(T_new - T) < tol:
            return float(T_new)
        T = T_new
    return np.inf


def continuous_joint_torque(p: ActuatorParams, T_amb: float, T_max: float | None = None) -> float:
    """Largest sustained joint torque whose steady winding temperature stays at T_max.

    Closed form: T_max - T_amb = TPR * (tau_m/Kt(T_max))^2 * R(T_max).
    This is the design-study quantity (continuous payload limit vs ambient).
    """
    T_max = p.T_limit if T_max is None else T_max
    if T_max <= T_amb:
        return 0.0
    P_allow = (T_max - T_amb) / (p.R_wh + p.R_ha)
    tau_m = torque_constant(p, T_max) * np.sqrt(P_allow / resistance(p, T_max))
    return float(tau_m * p.N * p.eta)
