import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from thermal.joint_model import (ActuatorParams, JointThermalModel, continuous_joint_torque,
                                 copper_loss, steady_state_motor)


@pytest.fixture
def p():
    return ActuatorParams.from_library("TBM6051A", ROOT / "params/actuators.yaml")


def test_zero_torque_stays_at_ambient(p):
    m = JointThermalModel(p, n=3, T_amb=30.0)
    for _ in range(600):
        m.step([0, 0, 0], 1.0)
    assert np.allclose(m.T_w, 30.0) and np.allclose(m.T_h, 30.0)


def test_cooling_relaxes_to_ambient(p):
    m = JointThermalModel(p, n=1, T_amb=25.0)
    m.reset(100.0)
    for _ in range(4 * 3600):
        m.step([0.0], 1.0)
    assert abs(m.T_w[0] - 25.0) < 0.5


def test_transient_converges_to_steady_state(p):
    tau = 60.0
    m = JointThermalModel(p, n=1)
    for _ in range(6 * 3600):
        m.step([tau], 2.0)
    T_ss = steady_state_motor(p, tau / (p.N * p.eta), 25.0)
    assert abs(m.T_w[0] - T_ss) < 0.5


def test_continuous_torque_lands_on_limit(p):
    tau = continuous_joint_torque(p, 25.0)
    T_ss = steady_state_motor(p, tau / (p.N * p.eta), 25.0)
    assert abs(T_ss - p.T_limit) < 0.1


def test_time_to_limit_matches_direct_sim(p):
    tau = 1.3 * continuous_joint_torque(p, 25.0)
    m = JointThermalModel(p, n=1)
    forecast = m.time_to_limit([tau], dt=1.0)[0]
    t = 0.0
    while not m.at_limit()[0]:
        m.step([tau], 0.5)
        t += 0.5
    assert abs(forecast - t) / t < 0.01


def test_below_continuous_never_hits_limit(p):
    tau = 0.9 * continuous_joint_torque(p, 25.0)
    assert np.isinf(JointThermalModel(p, n=1).time_to_limit([tau])[0])


def test_hot_joint_heats_faster_at_same_torque(p):
    assert copper_loss(p, 80.0, 120.0) > copper_loss(p, 80.0, 25.0)


def test_runaway_detected(p):
    # with absurd thermal resistance, loss growth outruns conduction: no steady state
    assert np.isinf(steady_state_motor(p.scaled(R_wh=40, R_ha=40), 1.5, 25.0))


def test_robot_budget_is_min_over_joints(p):
    m = JointThermalModel(p, n=14)
    tau = np.full(14, 20.0)
    tau[3] = 1.4 * continuous_joint_torque(p, 25.0)   # one overloaded elbow
    ttl = m.time_to_limit(tau)
    assert m.robot_budget(tau) == pytest.approx(ttl[3]) and np.isinf(ttl).sum() == 13


def test_warm_room_shrinks_budget(p):
    tau = [1.3 * continuous_joint_torque(p, 25.0)]
    cool = JointThermalModel(p, n=1, T_amb=25.0).time_to_limit(tau)[0]
    warm = JointThermalModel(p, n=1, T_amb=35.0).time_to_limit(tau)[0]
    assert warm < cool


def test_forecast_shortcut_matches_bruteforce(p):
    rng = np.random.default_rng(1)
    tau_c = continuous_joint_torque(p, 25.0)
    for trial in range(20):
        m = JointThermalModel(p, n=6)
        m.T_w[:] = rng.uniform(25, 135, 6)
        m.T_h[:] = np.minimum(m.T_w, rng.uniform(25, 120, 6))
        loads = (rng.uniform(0, 1.5, (5, 6)) * tau_c) ** 2
        fast = m.forecast_ttl(loads, horizon=7200)
        slow = m._forecast_ttl_bruteforce(loads, horizon=7200)
        assert np.array_equal(np.isinf(fast), np.isinf(slow))
        fin = np.isfinite(slow)
        assert np.allclose(fast[fin], slow[fin], atol=1e-6)
