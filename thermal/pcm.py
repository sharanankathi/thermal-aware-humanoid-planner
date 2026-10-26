"""Phase-change cartridge models.

Lumped enthalpy model (used in the shift simulation)
----------------------------------------------------
One enthalpy state H for the PCM composite plus the aluminum fins/channels inside the cartridge.
T(H) is piecewise linear: solid -> mushy zone (T_solidus..T_liquidus, carries the latent heat)
-> liquid. The heat flow to/from the coolant passes three resistances in series:

    Q = (T_coolant - T_pcm) / (R_hx + R_interface + R_layer(f))

  R_hx        coolant-side convection inside the cartridge heat exchanger (1 / UA_hx)
  R_interface 0 for a dry-break fluid coupling; a contact resistance for a dry/TIM pad interface
  R_layer     conduction through the phase layer next to the fins: while melting, the liquid layer
              of thickness f*gap/2; while freezing, the solid layer of thickness (1-f)*gap/2.
              This is the quasi-steady Stefan approximation (valid for small Stefan number;
              Ste ~ 0.2 here) and is what makes melting and re-freezing slow down as they proceed.

Reference models (used only in tests / validation)
--------------------------------------------------
fv_slab_*   1D finite-volume enthalpy method across half a fin gap (adiabatic midplane), wall
            node holds the fin metal; the "truth" for the lumped model.
neumann_*   exact one-phase Neumann solution, used to verify the finite-volume solver.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import brentq
from scipy.special import erf

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class CartridgeParams:
    m_pcm: float          # kg
    m_al: float           # kg (fins/channels, same temperature as the PCM in the lumped model)
    cp_s: float
    cp_l: float
    cp_al: float
    L: float              # J/kg latent
    Ts: float             # C solidus
    Tl: float             # C liquidus
    k: float              # W/m K composite conductivity
    rho: float            # kg/m3
    gap: float            # m, PCM layer between fins
    UA_hx: float          # W/K coolant-side
    R_int: float = 0.0    # K/W interface
    m_pkg: float = 0.0    # kg packaged (for reporting)

    @property
    def A_fin(self) -> float:
        """Fin wetted area: plates on both faces of every gap, A = 2 V / gap."""
        return 2.0 * (self.m_pcm / self.rho) / self.gap

    @property
    def C_solid(self) -> float:
        return self.m_pcm * self.cp_s + self.m_al * self.cp_al

    @property
    def C_liquid(self) -> float:
        return self.m_pcm * self.cp_l + self.m_al * self.cp_al

    @property
    def H_s(self) -> float:
        """Enthalpy (J, from 0 C all-solid) at the start of melting."""
        return self.C_solid * self.Ts

    @property
    def H_l(self) -> float:
        dT = self.Tl - self.Ts
        return self.H_s + self.m_pcm * self.L + dT * 0.5 * (self.C_solid + self.C_liquid)

    @property
    def latent_J(self) -> float:
        return self.m_pcm * self.L

    def with_(self, **kw) -> "CartridgeParams":
        return replace(self, **kw)

    @classmethod
    def from_yaml(cls, packaged_kg: float, path=ROOT / "params/cooling.yaml", **override
                  ) -> "CartridgeParams":
        c = yaml.safe_load(Path(path).read_text())["cartridge"]
        p = c["pcm"]
        kw = dict(
            m_pcm=c["pcm_mass_fraction"] * packaged_kg, m_al=c["hx_al_mass_fraction"] * packaged_kg,
            cp_s=p["cp_solid"], cp_l=p["cp_liquid"], cp_al=c["al_cp"], L=p["latent_J_per_kg"],
            Ts=p["T_solidus_C"], Tl=p["T_liquidus_C"], k=p["k_W_per_mK"], rho=p["density_kg_per_m3"],
            gap=c["fin_gap_m"], UA_hx=c["UA_hx_W_per_K_per_kg"] * packaged_kg,
            R_int=c["R_interface_K_per_W"], m_pkg=packaged_kg,
        )
        if "T_melt" in override:              # generic HC paraffin at another melt point, 3 K range
            Tm = override.pop("T_melt")
            kw["Ts"], kw["Tl"] = Tm - 1.5, Tm + 1.5
        kw.update(override)
        return cls(**kw)


# ---------------------------------------------------------------- lumped enthalpy model
def temperature(p: CartridgeParams, H):
    H = np.asarray(H, float)
    return np.where(H < p.H_s, H / p.C_solid,
           np.where(H < p.H_l, p.Ts + (H - p.H_s) / (p.H_l - p.H_s) * (p.Tl - p.Ts),
                    p.Tl + (H - p.H_l) / p.C_liquid))


def melt_fraction(p: CartridgeParams, H):
    return np.clip((np.asarray(H, float) - p.H_s) / (p.H_l - p.H_s), 0.0, 1.0)


def enthalpy_at(p: CartridgeParams, T: float, f: float | None = None) -> float:
    """Enthalpy of a cartridge at uniform temperature T (melt fraction f inside the mushy zone)."""
    if T < p.Ts:
        return p.C_solid * T
    if T > p.Tl:
        return p.H_l + (T - p.Tl) * p.C_liquid
    f = (T - p.Ts) / (p.Tl - p.Ts) if f is None else f
    return p.H_s + f * (p.H_l - p.H_s)


def heat_flow(p: CartridgeParams, H, T_coolant):
    """W into the cartridge (negative = re-freezing)."""
    T = temperature(p, H)
    f = melt_fraction(p, H)
    dT = np.asarray(T_coolant, float) - T
    layer = np.where(dT >= 0, f, 1.0 - f) * 0.5 * p.gap / (p.k * p.A_fin)
    return dT / (1.0 / p.UA_hx + p.R_int + layer)


def time_to_capacity(p: CartridgeParams, H0: float, T_coolant: float, f_target: float = 0.95,
                     dt: float = 1.0, t_max: float = 36000.0, refreeze: bool = False) -> float:
    """Seconds for a cartridge held against coolant at T_coolant to reach melt fraction f_target
    (or, with refreeze=True, to fall to melt fraction f_target from above)."""
    H, t = H0, 0.0
    while t < t_max:
        f = float(melt_fraction(p, H))
        if (not refreeze and f >= f_target) or (refreeze and f <= f_target and
                                                 float(temperature(p, H)) <= p.Ts):
            return t
        q = float(heat_flow(p, H, T_coolant))
        k1 = q
        k2 = float(heat_flow(p, H + dt * k1, T_coolant))
        H += 0.5 * dt * (k1 + k2)
        t += dt
    return np.inf


# ---------------------------------------------------------------- 1D finite-volume reference
def fv_slab(p: CartridgeParams, T_coolant: float, t_end: float, H0_T: float = 20.0, n: int = 24,
            record_dt: float = 10.0):
    """Melt one half-gap of PCM from a fin wall whose coolant side sees T_coolant through
    (R_hx + R_int), scaled per unit fin area. Wall node carries the fin aluminum.
    Returns (t, melt_fraction, Q_total_W) sampled every record_dt.
    Explicit enthalpy method; dt chosen for stability."""
    A = p.A_fin
    L_half = 0.5 * p.gap
    dx = L_half / n
    r_w = (1.0 / p.UA_hx + p.R_int) * A          # m2 K / W, per unit fin area
    m_cell = p.rho * dx                           # kg per m2 per cell
    c_al = p.m_al * p.cp_al / A                   # J/K per m2 of fin
    # per-cell enthalpy (J/m2), reference 0 C solid
    Ts, Tl, cs, cl, L = p.Ts, p.Tl, p.cp_s, p.cp_l, p.L
    hs = cs * Ts
    hl = hs + L + 0.5 * (cs + cl) * (Tl - Ts)

    def T_of(h):  # specific enthalpy J/kg -> T
        return np.where(h < hs, h / cs, np.where(h < hl, Ts + (h - hs) / (hl - hs) * (Tl - Ts),
                                                 Tl + (h - hl) / cl))

    def h_of(T):
        return cs * T if T < Ts else (hl + (T - Tl) * cl if T > Tl else hs + (T - Ts) / (Tl - Ts) * (hl - hs))

    h = np.full(n, h_of(H0_T))
    Tw = H0_T
    alpha_max = p.k / (p.rho * min(cs, cl))
    dt = 0.4 * dx * dx / alpha_max
    dt = min(dt, 0.4 * c_al / (1.0 / r_w + 2 * p.k / dx))
    t, out_t, out_f, out_q, nxt = 0.0, [0.0], [0.0], [0.0], record_dt
    G = p.k / dx
    while t < t_end:
        T = T_of(h)
        q_in = (T_coolant - Tw) / r_w                      # coolant -> wall
        q_w0 = 2 * G * (Tw - T[0])                         # wall -> first cell (half cell)
        flux = G * (T[:-1] - T[1:])                        # i -> i+1
        dh = np.empty(n)
        dh[0] = q_w0 - flux[0]
        dh[1:-1] = flux[:-1] - flux[1:]
        dh[-1] = flux[-1]                                  # adiabatic midplane
        h += dt * dh / m_cell
        Tw += dt * (q_in - q_w0) / c_al
        t += dt
        if t >= nxt:
            f = np.clip((h - hs) / (hl - hs), 0, 1).mean()
            out_t.append(t)
            out_f.append(float(f))
            out_q.append(float(q_in * A))
            nxt += record_dt
    return np.array(out_t), np.array(out_f), np.array(out_q)


def lumped_trace(p: CartridgeParams, T_coolant: float, t_end: float, H0_T: float = 20.0,
                 dt: float = 1.0, record_dt: float = 10.0):
    H = enthalpy_at(p, H0_T)
    t, out_t, out_f, out_q, nxt = 0.0, [0.0], [0.0], [0.0], record_dt
    while t < t_end:
        k1 = float(heat_flow(p, H, T_coolant))
        k2 = float(heat_flow(p, H + dt * k1, T_coolant))
        H += 0.5 * dt * (k1 + k2)
        t += dt
        if t >= nxt - 1e-9:
            out_t.append(t)
            out_f.append(float(melt_fraction(p, H)))
            out_q.append(k1)
            nxt += record_dt
    return np.array(out_t), np.array(out_f), np.array(out_q)


# ---------------------------------------------------------------- Neumann exact solution
def neumann_lambda(stefan: float) -> float:
    """Root of  lam * exp(lam^2) * erf(lam) = Ste / sqrt(pi)  (one-phase melting)."""
    return brentq(lambda lam: lam * np.exp(lam * lam) * erf(lam) - stefan / np.sqrt(np.pi),
                  1e-9, 5.0)


def neumann_front(t, k: float, rho: float, cp: float, L: float, T_wall: float, T_melt: float):
    """Melt-front position (m) for a semi-infinite solid at T_melt with the wall held at T_wall."""
    alpha = k / (rho * cp)
    lam = neumann_lambda(cp * (T_wall - T_melt) / L)
    return 2.0 * lam * np.sqrt(alpha * np.asarray(t, float))


def fv_front_fixed_wall(k, rho, cp, L, T_wall, T_melt, depth, t_end, n=200, dT_mush=0.1):
    """Finite-volume enthalpy solution with a fixed wall temperature (for the Neumann check).
    Returns (t, front position)."""
    dx = depth / n
    Ts, Tl = T_melt - dT_mush / 2, T_melt + dT_mush / 2
    hs = cp * Ts
    hl = hs + L + cp * dT_mush
    T_of = lambda h: np.where(h < hs, h / cp, np.where(h < hl, Ts + (h - hs) / (hl - hs) * dT_mush,
                                                      Tl + (h - hl) / cp))
    h = np.full(n, hs)                       # solid at the melting point (one-phase problem)
    alpha = k / (rho * cp)
    dt = 0.4 * dx * dx / alpha
    G = k / dx
    t, ts, fs, nxt = 0.0, [], [], t_end / 50
    while t < t_end:
        T = T_of(h)
        flux = G * (T[:-1] - T[1:])
        dh = np.empty(n)
        dh[0] = 2 * G * (T_wall - T[0]) - flux[0]
        dh[1:-1] = flux[:-1] - flux[1:]
        dh[-1] = flux[-1]
        h += dt * dh / (rho * dx)
        t += dt
        if t >= nxt:
            ts.append(t)
            fs.append(float(np.clip((h - hs) / (hl - hs), 0, 1).sum() * dx))
            nxt += t_end / 50
    return np.array(ts), np.array(fs)
