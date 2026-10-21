"""Week-1 validation: does the two-node model reproduce the datasheet rated point?

For every motor in params/actuators.yaml (TBM(S)-6013-A, -6025-A, -6051-A):
Check B  constant current      = Ic at 25 C ambient  -> winding should sit at 155 C   (GATE)
Check A  constant motor torque = Tc at 25 C ambient  -> winding should sit at 155 C   (info)
Pass if steady-state temperature RISE is within +/-10% of the datasheet rise (130 C),
the same tolerance Kollmorgen quotes on Kt and Rm.

Also writes: figures/step_response.png and figures/continuous_torque_vs_ambient.png
Run:  python scripts/rated_point_check.py
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from thermal.joint_model import (ActuatorParams, JointThermalModel, continuous_joint_torque,
                                 resistance, steady_state_motor, torque_constant)

TOL = 0.10


def steady_from_current(p, I, T_amb):
    T = T_amb
    for _ in range(500):
        T = T_amb + p.TPR * I**2 * resistance(p, T)
    return T


LIB = ROOT / "params/actuators.yaml"


def main():
    import yaml
    motors = list(yaml.safe_load(LIB.read_text())["motors"])
    ok_all = True
    for name in motors:
        ok_all &= check(ActuatorParams.from_library(name, LIB))
        print()
    figures(ActuatorParams.from_library("TBM6013A", LIB))
    return 0 if ok_all else 1


def check(p):
    T_amb, T_rated = p.T_amb_rated, p.T_hot
    rise_rated = T_rated - T_amb

    print(f"Actuator: {p.name}, N={p.N:g}, eta={p.eta}")
    print(f"R_wh={p.R_wh:.3f}  R_ha={p.R_ha:.3f} C/W   C_w={p.C_w:.0f}  C_h={p.C_h:.0f} J/K")
    print(f"tau_w={p.tau_w:.0f} s   tau_h={p.tau_h/60:.1f} min   T_limit={p.T_limit:g} C\n")

    # Gate: current-based check. It exercises exactly the model inputs (R(T), TPR).
    TB = steady_from_current(p, p.Ic, T_amb)
    errB = (TB - T_amb - rise_rated) / rise_rated
    passed = abs(errB) <= TOL
    print(f"Check B  current = Ic (GATE): T_w,ss = {TB:6.1f} C  (datasheet {T_rated:g} C)  "
          f"rise error {errB:+.1%}  -> {'PASS' if passed else 'FAIL'}")

    # Informational: torque-based check, with the datasheet's +/-10% Kt tolerance band.
    TA = steady_state_motor(p, p.Tc, T_amb)
    lo = steady_state_motor(p.scaled(Kt_cold=1.1, Kt_hot=1.1), p.Tc, T_amb)
    hi = steady_state_motor(p.scaled(Kt_cold=0.9, Kt_hot=0.9), p.Tc, T_amb)
    errA = (TA - T_amb - rise_rated) / rise_rated
    print(f"Check A  torque = Tc (info): T_w,ss = {TA:6.1f} C  rise error {errA:+.1%}; "
          f"Kt +/-10% band {lo:.0f}-{hi:.0f} C -> 155 C "
          f"{'inside' if lo <= T_rated <= hi else 'OUTSIDE'} band")
    gap = float(torque_constant(p, 155)) * p.Ic / p.Tc - 1
    print(f"  Why A and B differ: Ic x Kt_hot = {float(torque_constant(p, 155)) * p.Ic:.3f} Nm "
          f"but datasheet Tc = {p.Tc} Nm. The datasheet's torque and current ratings disagree "
          f"by {gap:.1%}, {(1 + gap) ** 2 - 1:.1%} in loss.")
    tau_cont = continuous_joint_torque(p, T_amb)
    print(f"Continuous JOINT torque (N={p.N:g}) at T_limit={p.T_limit:g} C: {tau_cont:.1f} Nm "
          f"@25 C, {continuous_joint_torque(p, 35.0):.1f} Nm @35 C")
    return passed


def figures(p):
    T_amb = p.T_amb_rated
    tau_cont = continuous_joint_torque(p, T_amb)

    # ---- step responses at fractions of the continuous joint torque
    fig, ax = plt.subplots(figsize=(7, 4))
    t_end, dt = 3600.0, 1.0
    t = np.arange(0, t_end + dt, dt)
    for frac in [0.8, 1.0, 1.15, 1.3]:
        m = JointThermalModel(p, n=1, T_amb=T_amb)
        ttl = m.time_to_limit([frac * tau_cont], horizon=t_end)[0]
        Tw = [m.T_w[0]]
        for _ in t[1:]:
            Tw.append(m.step([frac * tau_cont], dt)[0])
        lab = f"{frac:.2f}x ({frac*tau_cont:.0f} Nm)" + ("" if np.isinf(ttl) else f", limit at {ttl/60:.1f} min")
        ax.plot(t / 60, Tw, label=lab)
    ax.axhline(p.T_limit, color="k", ls="--", lw=1)
    ax.text(1, p.T_limit + 2, f"protective stop {p.T_limit:g} °C", fontsize=8)
    ax.set_xlabel("time (min)")
    ax.set_ylabel("winding temperature (°C)")
    ax.set_title(f"{p.name}, {p.N:g}:1 (shoulder/elbow): constant-torque step response, 25 °C")
    ax.legend(fontsize=8, frameon=False)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(ROOT / "figures/step_response.png", dpi=150)

    # ---- design-study preview: continuous joint torque vs ambient
    amb = np.linspace(15, 45, 31)
    fig, ax = plt.subplots(figsize=(6, 3.6))
    for (name, col, ls) in [("TBM6013A", "#2a78d6", "-"), ("TBM6025A", "#eb6834", "--"),
                            ("TBM6051A", "#1baf7a", ":")]:
        ps = ActuatorParams.from_library(name, LIB, gear_ratio=p.N)
        y = [continuous_joint_torque(ps, a) for a in amb]
        ax.plot(amb, y, ls=ls, color=col, lw=2)
        ax.annotate(name.replace("TBM", "TBM(S)-").replace("A", "-A"), (amb[-1], y[-1]),
                    xytext=(4, 0), textcoords="offset points", fontsize=8, va="center")
    ax.set_xlabel("ambient (°C)")
    ax.set_ylabel("continuous joint torque (Nm)")
    ax.set_xlim(15, 52)
    ax.set_title(f"Continuous shoulder torque at a {p.T_limit:g} °C winding limit ({p.N:g}:1)")
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(ROOT / "figures/continuous_torque_vs_ambient.png", dpi=150)



if __name__ == "__main__":
    sys.exit(main())
