"""Run a pick-and-place shift: box stream -> controller -> cycle torques -> thermal model.

The thermal model advances once per cycle segment using that segment's exact mean tau^2,
so heat input is integrated exactly (segments are <= 1.8 s; winding tau >= 74 s).
A winding at T_limit triggers a protective stop, which ends the shift (the failure measured).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from thermal.joint_model import ActuatorParams, JointThermalModel

from .cycles import PICK_SLOTS, PLACE_SHELF, PLACE_SLOTS, CycleLibrary
from .robot import ARM_SUFFIXES, JOINT_NAMES

ROOT = Path(__file__).resolve().parents[1]


def joint_params(library_yaml=ROOT / "params/actuators.yaml", motor_override=None, scale=None):
    import yaml
    assign = yaml.safe_load(Path(library_yaml).read_text())["joint_actuator"]
    out = []
    for name in JOINT_NAMES:
        motor, ratio, _ = assign[name[2:]]
        motor = motor_override or motor
        p = ActuatorParams.from_library(motor, library_yaml, gear_ratio=ratio)
        if scale:
            p = p.scaled(**scale)
        out.append(p)
    return out


def loss_coeff(params) -> np.ndarray:
    """Copper loss per joint-torque^2 at 25 C (W/Nm^2): R25 / (N eta Kt_cold)^2."""
    return np.array([p.R25 / (p.N * p.eta * p.Kt_cold) ** 2 for p in params])


@dataclass
class Box:
    mass: float
    pick: int        # index into PICK_SLOTS
    place: int       # index into PLACE_SLOTS


@dataclass
class Scenario:
    name: str
    masses: tuple            # box masses drawn uniformly from this tuple
    shelves: tuple           # which shelves receive boxes ("std", "high")
    T_amb: float = 25.0
    shift_s: float = 7200.0
    pick_slots: tuple | None = None   # restrict where boxes arrive (None = all five slots)
    warm_arm: str | None = None       # arm that starts the shift already warm ("l"/"r")
    warm_T: tuple = (110.0, 90.0)     # its starting (winding, housing) temperatures


SCENARIOS = {
    # masses scaled to the actuators: B1 (one arm, no thermal awareness) survives 4 kg to the
    # high shelf for the full shift but stops at 53 min with 5 kg (peak torque 68% of limit)
    # Heavy = 8 kg: single-arm peak torque 93% of Apollo's limit; B1 stops at 9 min.
    "light_std":  Scenario("light_std", masses=(2.0,), shelves=("std",)),
    "heavy_high": Scenario("heavy_high", masses=(8.0,), shelves=("high",)),
    "warm_heavy": Scenario("warm_heavy", masses=(8.0,), shelves=("high",), T_amb=35.0),
    "mixed":      Scenario("mixed", masses=(2.0, 4.0, 6.0, 8.0), shelves=("std", "high")),
    "conveyor_r": Scenario("conveyor_r", masses=(8.0,), shelves=("high",), pick_slots=(0, 1)),
    "warm_start": Scenario("warm_start", masses=(8.0,), shelves=("high",), warm_arm="r"),
}


def box_stream(sc: Scenario, seed: int, n: int = 4000) -> list[Box]:
    rng = np.random.default_rng(seed)
    places = [j for j, s in enumerate(PLACE_SHELF) if s in sc.shelves]
    picks = list(sc.pick_slots) if sc.pick_slots else list(range(len(PICK_SLOTS)))
    return [Box(float(rng.choice(sc.masses)), int(rng.choice(picks)),
                int(rng.choice(places))) for _ in range(n)]


@dataclass
class Plan:
    arm: str          # "l", "r" or "both"
    posture: int      # posture-library index
    pace: float = 1.0
    pause: float = 0.0   # idle seconds after the cycle


@dataclass
class ShiftResult:
    controller: str
    scenario: str
    seed: int
    boxes: int
    stopped: bool
    t_stop: float            # time of protective stop (np.inf if none)
    t_end: float
    peak_T: np.ndarray       # (14,)
    energy_J: float          # copper energy, all joints
    arm_counts: dict
    cycle_time_mean: float
    peak_torque_ratio: float  # max |tau| / Apollo limit seen
    trace_t: np.ndarray = field(repr=False, default=None)
    trace_T: np.ndarray = field(repr=False, default=None)
    trace_events: list = field(repr=False, default_factory=list)


def run_shift(lib: CycleLibrary, controller, sc: Scenario, seed: int, params=None,
              ctrl_params=None, record: bool = True, trace_dt: float = 10.0) -> ShiftResult:
    """params: the plant (true joints). ctrl_params: the controller's forecast model
    (defaults to the plant). The controller always sees the plant's measured temperatures."""
    params = params or joint_params()
    th = JointThermalModel(params, T_amb=sc.T_amb)
    if sc.warm_arm:
        sl = slice(0, 7) if sc.warm_arm == "l" else slice(7, 14)
        th.T_w[sl], th.T_h[sl] = sc.warm_T
    th_ctrl = th if ctrl_params is None else JointThermalModel(ctrl_params, T_amb=sc.T_amb)
    idle = lib.idle_tau_sq()
    boxes = box_stream(sc, seed)
    controller.reset(th_ctrl, lib, sc, idle)
    t, nbox, energy, stopped, t_stop = 0.0, 0, 0.0, False, np.inf
    peak_T = th.T_w.copy()
    counts = {"l": 0, "r": 0, "both": 0}
    tr_t, tr_T, next_rec = [0.0], [th.T_w.copy()], trace_dt
    peak_ratio = 0.0
    tau_max = lib.robot.tau_max
    cyc_times = []
    events = []
    for box in boxes:
        if t >= sc.shift_s:
            break
        if th_ctrl is not th:          # sensor readout: controller sees true temperatures
            th_ctrl.T_w[:], th_ctrl.T_h[:] = th.T_w, th.T_h
        plan = controller.plan(box, t)
        if record:
            events.append((t, box, plan, th.T_w.copy()))
        cyc = lib.get(box.pick, box.place, plan.arm, plan.posture, plan.pace)
        seg = cyc.seg_tau_sq(box.mass)
        # idle arm sits at stand; replace its torques with the idle values
        if plan.arm in ("l", "r"):
            other = slice(7, 14) if plan.arm == "l" else slice(0, 7)
            seg[:, other] = idle[other]
        pk = cyc.peak(box.mass)
        peak_ratio = max(peak_ratio, float((pk / tau_max).max()))
        steps = list(zip(cyc.durations, seg))
        if plan.pause > 0:
            steps.append((plan.pause, idle))
        for dt, tsq in steps:
            energy += float(np.sum(th.power(th.T_w, tsq))) * dt
            th.step_tau_sq(tsq, dt)
            t += dt
            np.maximum(peak_T, th.T_w, out=peak_T)
            if record and t >= next_rec:
                tr_t.append(t)
                tr_T.append(th.T_w.copy())
                next_rec += trace_dt
            if th.at_limit().any():
                stopped, t_stop = True, t
                break
        if stopped:
            break
        nbox += 1
        counts[plan.arm] += 1
        cyc_times.append(cyc.T + plan.pause)
        controller.after(box, plan, t)
    tr_t.append(t)
    tr_T.append(th.T_w.copy())
    return ShiftResult(controller.name, sc.name, seed, nbox, stopped, t_stop, t, peak_T, energy,
                       counts, float(np.mean(cyc_times)) if cyc_times else np.nan, peak_ratio,
                       np.array(tr_t), np.array(tr_T), events)
