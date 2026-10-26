"""14-joint thermal model plus a torso coolant loop with a radiator and/or a PCM cartridge.

Per joint j (unchanged two-node model, plus a cold plate on plumbed joints):
    C_w dT_w = P_cu - (T_w - T_h)/R_wh
    C_h dT_h = (T_w - T_h)/R_wh - (T_h - T_amb)/R_ha - g_j (T_h - T_c)
Loop (one well-mixed node; at ~1 L/min the inlet/outlet split is a few K, stated limitation):
    C_c dT_c = sum_j g_j (T_h - T_c) - UA_rad (T_c - T_amb) - Q_cart - Q_dock
Cartridge (lumped enthalpy, thermal/pcm.py):
    dH = Q_cart = heat_flow(H, T_c)                (only while a cartridge is installed)
Dock (charge pad / swap bay with chilled water pushed through the robot loop):
    Q_dock = UA_dock (T_c - T_chill)               (only while docked with the chiller on)

The class keeps the JointThermalModel interface (T_w, T_h, step_tau_sq, power, at_limit),
so the existing controllers run unchanged. Energy flows are integrated for an energy-balance check.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from . import pcm
from .joint_model import JointThermalModel

ROOT = Path(__file__).resolve().parents[1]


@dataclass
class LoopConfig:
    plumbed: np.ndarray            # (n,) bool
    R_cp: float = 0.5              # K/W housing -> coolant per plumbed joint
    C_c: float = 1800.0            # J/K coolant + cold plates
    UA_rad: float = 0.0            # W/K radiator (0 = none)
    cart: pcm.CartridgeParams | None = None
    T_chill: float = 10.0
    UA_dock: float = 200.0

    @classmethod
    def from_yaml(cls, joint_names, radiator_kg=0.0, cartridge_kg=0.0, path=ROOT / "params/cooling.yaml",
                  R_cp=None, ua_rad_per_kg=None, cart_override=None):
        d = yaml.safe_load(Path(path).read_text())
        lp, rad, dock = d["loop"], d["radiator"], d["dock"]
        plumbed = np.array([n[2:] in lp["plumbed_joints"] for n in joint_names])
        cart = (pcm.CartridgeParams.from_yaml(cartridge_kg, path, **(cart_override or {}))
                if cartridge_kg > 0 else None)
        upk = rad["UA_per_kg_W_per_K"] if ua_rad_per_kg is None else ua_rad_per_kg
        return cls(plumbed=plumbed, R_cp=lp["R_coldplate_K_per_W"] if R_cp is None else R_cp,
                   C_c=lp["coolant_mass_kg"] * lp["coolant_cp"], UA_rad=upk * radiator_kg,
                   cart=cart, T_chill=dock["T_chill_C"], UA_dock=dock["UA_dock_W_per_K"])


@dataclass
class EnergyLog:
    copper: float = 0.0
    air: float = 0.0          # joints -> ambient through R_ha
    coldplate: float = 0.0    # joints -> coolant
    radiator: float = 0.0
    cartridge: float = 0.0    # into the installed cartridge (net, refreeze negative)
    dock: float = 0.0         # removed by the dock chiller through the loop
    def as_dict(self):
        return dict(self.__dict__)


class CooledJointModel(JointThermalModel):
    max_dt = 1.0   # internal sub-step (s); coolant time constant >= ~6 s with the dock coupled

    def __init__(self, params, loop: LoopConfig | None, T_amb: float = 25.0):
        super().__init__(params, T_amb=T_amb)
        self.loop = loop
        self.g = (loop.plumbed / loop.R_cp) if loop is not None else np.zeros(self.n)
        self.T_c = T_amb
        self.cart = loop.cart if loop is not None else None
        self.H = pcm.enthalpy_at(self.cart, T_amb) if self.cart else 0.0
        self.cart_installed = self.cart is not None
        self.docked = False
        self.E = EnergyLog()

    # ---------------------------------------------------------- state helpers
    def install_cartridge(self, T_fresh: float):
        self.H = pcm.enthalpy_at(self.cart, T_fresh)
        self.cart_installed = True

    @property
    def melt_fraction(self) -> float:
        return float(pcm.melt_fraction(self.cart, self.H)) if self.cart else 0.0

    @property
    def T_cart(self) -> float:
        return float(pcm.temperature(self.cart, self.H)) if self.cart else np.nan

    def reset(self, T0=None):
        super().reset(T0)
        T0 = self.T_amb if T0 is None else T0
        self.T_c = T0
        if self.cart:
            self.H = pcm.enthalpy_at(self.cart, T0)

    # ---------------------------------------------------------- dynamics
    def _rates(self, Tw, Th, Tc, H, tau_sq):
        P = self.power(Tw, tau_sq)
        q_wh = (Tw - Th) / self._Rwh
        q_air = (Th - self.T_amb) / self._Rha
        q_cp = self.g * (Th - Tc)
        dTw = (P - q_wh) / self._Cw
        dTh = (q_wh - q_air - q_cp) / self._Ch
        if self.loop is None:
            return dTw, dTh, 0.0, 0.0, (P.sum(), q_air.sum(), 0.0, 0.0, 0.0, 0.0)
        # radiator bypass valve: a radiator never pumps room heat into a sub-ambient loop
        q_rad = self.loop.UA_rad * max(Tc - self.T_amb, 0.0)
        q_cart = float(pcm.heat_flow(self.cart, H, Tc)) if (self.cart and self.cart_installed) else 0.0
        q_dock = self.loop.UA_dock * (Tc - self.loop.T_chill) if self.docked else 0.0
        dTc = (q_cp.sum() - q_rad - q_cart - q_dock) / self.loop.C_c
        return dTw, dTh, dTc, q_cart, (P.sum(), q_air.sum(), q_cp.sum(), q_rad, q_cart, q_dock)

    def step_tau_sq(self, tau_sq, dt: float):
        tau_sq = np.asarray(tau_sq, float)
        nsub = max(1, int(np.ceil(dt / self.max_dt)))
        h = dt / nsub
        for _ in range(nsub):
            Tw, Th, Tc, H = self.T_w, self.T_h, self.T_c, self.H
            a = self._rates(Tw, Th, Tc, H, tau_sq)
            b = self._rates(Tw + h * a[0], Th + h * a[1], Tc + h * a[2], H + h * a[3], tau_sq)
            self.T_w = Tw + 0.5 * h * (a[0] + b[0])
            self.T_h = Th + 0.5 * h * (a[1] + b[1])
            self.T_c = Tc + 0.5 * h * (a[2] + b[2])
            self.H = H + 0.5 * h * (a[3] + b[3])
            fa, fb = a[4], b[4]
            for i, k in enumerate(("copper", "air", "coldplate", "radiator", "cartridge", "dock")):
                setattr(self.E, k, getattr(self.E, k) + 0.5 * h * (fa[i] + fb[i]))
        return self.T_w

    def stored_energy(self) -> float:
        """Sensible + latent energy above a common reference (for the energy-balance test)."""
        e = float((self._Cw * self.T_w).sum() + (self._Ch * self.T_h).sum())
        if self.loop is not None:
            e += self.loop.C_c * self.T_c
        if self.cart and self.cart_installed:
            e += self.H
        return e
