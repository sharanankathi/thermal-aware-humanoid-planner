"""Fit a motor's two-node thermal model from a bench log (potting experiment, docs/bench_potting_plan.md).

Input CSV columns: t_s, V, I, T_h, T_amb   (one row per sample; DC current through two phases,
rotor locked; V and I measured at the motor leads; T_h = thermocouple on the stator back iron /
housing; T_amb = room). Also pass the cold line-to-line resistance R25 measured at 25 C (or at the
recorded room temperature with --T0).

Winding temperature comes from resistance:  T_w = T0 + (V/I / R0 - 1) / 0.00393
Steady state (mean of the last 10% of the log):
    P = V I,   R_wh = (T_w - T_h) / P,   R_ha = (T_h - T_amb) / P
Transient: least-squares fit of C_w, C_h (and R_wh, R_ha) to the whole log.

Usage:
    python experiments/bench_rwh.py log.csv --R0 12.3 --T0 22.5
    python experiments/bench_rwh.py before.csv after.csv --R0 12.3 12.4 --T0 22.5 23.0   # compare
"""
from __future__ import annotations

import argparse
import csv

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares

ALPHA = 0.00393


def load(path, R0, T0):
    rows = list(csv.DictReader(open(path)))
    a = {k: np.array([float(r[k]) for r in rows]) for k in ("t_s", "V", "I", "T_h", "T_amb")}
    a["R"] = a["V"] / a["I"]
    a["T_w"] = T0 + (a["R"] / R0 - 1.0) / ALPHA
    a["P"] = a["V"] * a["I"]
    return a


def steady(a):
    n = max(5, len(a["t_s"]) // 10)
    s = slice(-n, None)
    P = a["P"][s].mean()
    Tw, Th, Ta = a["T_w"][s].mean(), a["T_h"][s].mean(), a["T_amb"][s].mean()
    return dict(P=P, T_w=Tw, T_h=Th, T_amb=Ta, R_wh=(Tw - Th) / P, R_ha=(Th - Ta) / P)


def fit_transient(a, ss):
    t, P, Ta = a["t_s"], a["P"], a["T_amb"]

    def sim(x):
        Rwh, Rha, Cw, Ch = np.exp(x)
        Pf = lambda tt: np.interp(tt, t, P)
        Taf = lambda tt: np.interp(tt, t, Ta)

        def f(tt, y):
            Tw, Th = y
            q = (Tw - Th) / Rwh
            return [(Pf(tt) - q) / Cw, (q - (Th - Taf(tt)) / Rha) / Ch]
        y0 = [a["T_w"][0], a["T_h"][0]]
        return solve_ivp(f, (t[0], t[-1]), y0, t_eval=t, max_step=5.0).y

    def res(x):
        y = sim(x)
        return np.concatenate([y[0] - a["T_w"], y[1] - a["T_h"]])

    x0 = np.log([max(ss["R_wh"], 1e-3), max(ss["R_ha"], 1e-3), 50.0, 300.0])
    sol = least_squares(res, x0)
    Rwh, Rha, Cw, Ch = np.exp(sol.x)
    rms = float(np.sqrt(np.mean(sol.fun ** 2)))
    return dict(R_wh=Rwh, R_ha=Rha, C_w=Cw, C_h=Ch, tau_w=Cw * Rwh, tau_h=Ch * Rha, rms_K=rms)


def report(path, R0, T0):
    a = load(path, R0, T0)
    ss = steady(a)
    tr = fit_transient(a, ss)
    print(f"\n{path}")
    print(f"  steady: P {ss['P']:.2f} W  T_w {ss['T_w']:.1f}  T_h {ss['T_h']:.1f}  T_amb {ss['T_amb']:.1f} C")
    print(f"          R_wh {ss['R_wh']:.3f} K/W   R_ha {ss['R_ha']:.3f} K/W   "
          f"winding share of total {ss['R_wh'] / (ss['R_wh'] + ss['R_ha']):.0%}")
    print(f"  fit:    R_wh {tr['R_wh']:.3f}  R_ha {tr['R_ha']:.3f}  C_w {tr['C_w']:.0f} J/K  C_h {tr['C_h']:.0f} J/K"
          f"  tau_w {tr['tau_w']:.0f} s  tau_h {tr['tau_h']:.0f} s  (rms {tr['rms_K']:.2f} K)")
    return ss


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--R0", nargs="+", type=float, required=True, help="cold resistance per log (ohm)")
    ap.add_argument("--T0", nargs="+", type=float, required=True, help="temperature at R0 (C) per log")
    a = ap.parse_args()
    out = [report(p, r, t) for p, r, t in zip(a.logs, a.R0, a.T0)]
    if len(out) == 2:
        d = 1 - out[1]["R_wh"] / out[0]["R_wh"]
        print(f"\nR_wh change: {-d:+.0%}  ->  {'PASS: worth pursuing' if d >= 0.25 else 'KILL: below the 25% threshold'}")


if __name__ == "__main__":
    main()
