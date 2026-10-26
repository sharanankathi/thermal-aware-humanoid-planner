"""Validation tests for the PCM cartridge and coolant-loop models."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from thermal import pcm  # noqa: E402
from thermal.joint_model import ActuatorParams  # noqa: E402
from thermal.loop import CooledJointModel, LoopConfig  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def test_enthalpy_temperature_roundtrip():
    p = pcm.CartridgeParams.from_yaml(2.0)
    for T in (10.0, 30.0, 42.0, 50.0):
        assert abs(float(pcm.temperature(p, pcm.enthalpy_at(p, T))) - T) < 1e-9
    assert pcm.melt_fraction(p, pcm.enthalpy_at(p, 30.0)) == 0.0
    assert pcm.melt_fraction(p, pcm.enthalpy_at(p, 50.0)) == 1.0


def test_finite_volume_matches_neumann():
    k, rho, cp, L = 5.0, 800.0, 1900.0, 187000.0
    t, s = pcm.fv_front_fixed_wall(k, rho, cp, L, 60.0, 42.5, 0.06, 150.0, n=120)
    ex = pcm.neumann_front(t, k, rho, cp, L, 60.0, 42.5)
    assert np.max(np.abs(s[10:] - ex[10:]) / ex[10:]) < 0.01


def test_lumped_matches_finite_volume():
    for k, gap in ((5.0, 0.006), (0.2, 0.012)):
        p = pcm.CartridgeParams.from_yaml(2.0, k=k, gap=gap)
        tf, ff, _ = pcm.fv_slab(p, 55.0, 1800.0, n=12)
        tl, fl, _ = pcm.lumped_trace(p, 55.0, 1800.0)
        t90 = lambda t, f: t[np.argmax(f >= 0.9)]
        assert abs(t90(tl, fl) - t90(tf, ff)) / t90(tf, ff) < 0.05


def test_loop_energy_balance():
    """Copper loss = stored energy change + heat to air + radiator + cartridge + dock."""
    names = [f"{s}_{j}" for s in ("l", "r") for j in
             ("shoulder_aa", "shoulder_ie", "shoulder_fe", "elbow_fe", "wrist_roll", "wrist_yaw", "wrist_pitch")]
    p = ActuatorParams.from_library("TBM6013A", ROOT / "params/actuators.yaml")
    cfg = LoopConfig.from_yaml(names, radiator_kg=1.0, cartridge_kg=1.0)
    m = CooledJointModel([p] * 14, cfg, T_amb=30.0)
    m.install_cartridge(20.0)
    e0 = m.stored_energy()
    tau_sq = np.full(14, 400.0)
    for i in range(1200):
        m.docked = 600 <= i < 700
        m.step_tau_sq(tau_sq, 1.5)
    E = m.E
    out = E.air + E.radiator + E.dock
    # the cartridge is part of the stored energy, so its flow is internal
    assert abs(E.copper - (m.stored_energy() - e0) - out) / E.copper < 2e-3
