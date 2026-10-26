"""Long shift with battery service and thermal service (radiator / in-place refreeze / swap).

The work policy is fixed (default: the reactive 130 C limb switch, the strongest v2 controller);
only the cooling hardware and the service logistics change between options. This isolates the
question the cartridge study asks: when does swappable thermal capacity pay for itself?

Options (ThermalOption.kind):
  air       no loop (the v2 robot)
  loop      loop with no sink: cold plates only spread heat between joints' air paths
  radiator  loop + radiator of `radiator_kg`
  inplace   loop + fixed PCM of `cartridge_kg`; the charge dock pushes chilled water through the
            loop during the battery dwell (re-freezes in place, limited by dwell time)
  swap      loop + swappable cartridge; a dedicated trip whenever melt fraction >= f_swap
  coswap    swap only during battery service (never an extra trip)
  smart     swap at every battery service, plus an extra trip only when the cartridge saturates
            early enough that the throughput it would lose exceeds the trip's cost
  `radiator_kg` may be combined with any cartridge option (hybrid).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from thermal.loop import CooledJointModel, LoopConfig
from thermal import pcm

from .robot import JOINT_NAMES
from .shift import Scenario, box_stream, joint_params


@dataclass
class Logistics:
    run_min: float = 90.0        # battery runtime between services
    dwell_min: float = 9.0       # battery service time at the dock
    travel_s: float = 45.0       # one way
    swap_s: float = 60.0         # cartridge exchange
    shift_h: float = 8.0
    T_fresh: float = 20.0        # issued cartridge temperature
    thermal_trip_s: float | None = None  # round-trip time of a cartridge trip (None: 2*travel + swap)


@dataclass
class ThermalOption:
    kind: str = "air"
    radiator_kg: float = 0.0
    cartridge_kg: float = 0.0
    f_swap: float = 0.95         # trip threshold (swap / smart)
    breakeven_rho: float = 0.3   # smart: fractional throughput lost while saturated (held-out calibration)
    R_cp: float | None = None
    ua_rad_per_kg: float | None = None
    cart_override: dict = field(default_factory=dict)

    @property
    def label(self) -> str:
        parts = [self.kind]
        if self.radiator_kg:
            parts.append(f"rad{self.radiator_kg:g}")
        if self.cartridge_kg:
            parts.append(f"cart{self.cartridge_kg:g}")
        return "+".join(parts)


@dataclass
class ServiceResult:
    option: str
    scenario: str
    seed: int
    boxes: int
    stopped: bool
    t_stop: float
    shift_s: float
    work_s: float               # time spent doing pick-and-place
    battery_s: float            # battery service incl. travel
    thermal_trip_s: float       # extra time spent on cartridge trips
    n_trips: int
    n_swaps: int                # swaps incl. those done during battery service
    peak_T: float
    frac_two_arm: float         # share of cycles in the cooling (two-arm) mode
    energy: dict                # integrated heat flows (J)
    swap_log: list              # (t, heat absorbed by the removed cartridge since issue, J)
    chiller_J: float            # heat the dock or rack must remove (J), for the energy cost
    sat_s: float                # time with an installed cartridge at melt fraction >= 0.99
    trace: dict = field(repr=False, default=None)


def make_model(params, opt: ThermalOption, T_amb: float) -> CooledJointModel:
    if opt.kind == "air":
        return CooledJointModel(params, None, T_amb=T_amb)
    cfg = LoopConfig.from_yaml(JOINT_NAMES, radiator_kg=opt.radiator_kg,
                               cartridge_kg=opt.cartridge_kg if opt.kind not in ("radiator", "loop") else 0.0,
                               R_cp=opt.R_cp, ua_rad_per_kg=opt.ua_rad_per_kg,
                               cart_override=opt.cart_override)
    return CooledJointModel(params, cfg, T_amb=T_amb)


def run_service_shift(lib, controller, sc: Scenario, seed: int, opt: ThermalOption,
                      lg: Logistics = Logistics(), params=None, record: bool = False,
                      trace_dt: float = 30.0) -> ServiceResult:
    params = params or joint_params()
    th = make_model(params, opt, sc.T_amb)
    if th.cart is not None:
        # a cartridge is issued frozen: at the rack temperature, or 5 K under its solidus
        lg = Logistics(**{**lg.__dict__, "T_fresh": min(lg.T_fresh, th.cart.Ts - 5.0)})
        th.install_cartridge(lg.T_fresh)
    idle = lib.idle_tau_sq()
    boxes = box_stream(sc, seed, n=int(lg.shift_h * 3600 / 2.0) + 100)
    controller.reset(th, lib, sc, idle)
    shift_s = lg.shift_h * 3600.0
    t, nbox, stopped, t_stop = 0.0, 0, False, np.inf
    work_s = batt_s = trip_s = sat_s = 0.0
    n_trips = n_swaps = n_cyc = n_two = 0
    peak = float(th.T_w.max())
    next_batt = lg.run_min * 60.0
    H_issued = th.H
    swap_log, chiller_J = [], 0.0
    tr = {"t": [], "Tmax": [], "Tc": [], "f": [], "mode": []} if record else None
    next_rec = 0.0
    rho = opt.breakeven_rho
    t_trip_cost = (lg.thermal_trip_s if lg.thermal_trip_s is not None
                   else 2 * lg.travel_s + lg.swap_s)

    def advance(tsq, dt):
        nonlocal t, peak, stopped, t_stop, sat_s, next_rec
        th.step_tau_sq(tsq, dt)
        t += dt
        peak = max(peak, float(th.T_w.max()))
        if th.cart is not None and th.cart_installed and th.melt_fraction >= 0.99:
            sat_s += dt
        if record and t >= next_rec:
            tr["t"].append(t); tr["Tmax"].append(float(th.T_w.max())); tr["Tc"].append(th.T_c)
            tr["f"].append(th.melt_fraction); tr["mode"].append(getattr(controller, "cool", False))
            next_rec += trace_dt
        if th.at_limit().any() and not stopped:
            stopped, t_stop = True, t

    def idle_for(dt):
        while dt > 1e-9 and not stopped:
            h = min(dt, 30.0)
            advance(idle, h)
            dt -= h

    def do_swap(log_t):
        nonlocal H_issued, n_swaps, chiller_J
        absorbed = th.H - H_issued
        swap_log.append((log_t, absorbed))
        # rack re-freeze removes everything above the issue state
        chiller_J += max(absorbed, 0.0)
        th.install_cartridge(lg.T_fresh)
        H_issued = th.H
        n_swaps += 1

    i = 0
    while i < len(boxes) - 1 and t < shift_s and not stopped:
        # ------------------------------------------------ battery service
        if t >= next_batt:
            t0 = t
            idle_for(lg.travel_s)
            dwell = lg.dwell_min * 60.0
            if opt.kind == "inplace" and th.cart is not None:
                H0 = th.H
                th.docked = True
                E0 = th.E.dock
                idle_for(dwell)
                th.docked = False
                chiller_J += th.E.dock - E0
            elif opt.kind in ("coswap", "smart") and th.cart is not None:
                idle_for(max(dwell, lg.swap_s) - lg.swap_s)
                idle_for(lg.swap_s)
                do_swap(t)
            else:
                idle_for(dwell)
            idle_for(lg.travel_s)
            batt_s += t - t0
            next_batt = t + lg.run_min * 60.0
            continue
        # ------------------------------------------------ thermal trip
        if th.cart is not None and opt.kind in ("swap", "smart") and th.melt_fraction >= opt.f_swap:
            go = True
            if opt.kind == "smart":
                # trip only if the saturated stretch until the next battery service would cost more
                # boxes than the trip itself: (t_next - t) * rho > t_trip
                go = (next_batt - t) * rho > t_trip_cost
            if go:
                t0 = t
                if lg.thermal_trip_s is not None:   # swap point near the work cell
                    idle_for(lg.thermal_trip_s)
                    do_swap(t)
                else:
                    idle_for(lg.travel_s)
                    idle_for(lg.swap_s)
                    do_swap(t)
                    idle_for(lg.travel_s)
                trip_s += t - t0
                n_trips += 1
                continue
        # ------------------------------------------------ one work cycle (as sim/shift.py)
        box = boxes[i]
        controller.next_box = boxes[i + 1]
        plan = controller.plan(box, t)
        if plan.arm == "dual":
            box2 = boxes[i + 1]
            cyc = lib.get(box.pick, box.place, "l", plan.posture, plan.pace)
            cyc2 = lib.get(box2.pick, box2.place, "r", plan.posture2, plan.pace)
            seg = cyc.seg_tau_sq(box.mass)
            seg[:, 7:14] = cyc2.seg_tau_sq(box2.mass)[:, 7:14]
            n_here = 2
        else:
            cyc = lib.get(box.pick, box.place, plan.arm, plan.posture, plan.pace)
            seg = cyc.seg_tau_sq(box.mass)
            if plan.arm in ("l", "r"):
                other = slice(7, 14) if plan.arm == "l" else slice(0, 7)
                seg[:, other] = idle[other]
            n_here = 1
        steps = list(zip(cyc.durations, seg))
        if plan.pause > 0:
            steps.append((plan.pause, idle))
        t0 = t
        for dt, tsq in steps:
            advance(tsq, dt)
            if stopped:
                break
        work_s += t - t0
        if stopped:
            break
        nbox += n_here
        n_cyc += 1
        n_two += plan.arm == "both"
        controller.after(box, plan, t)
        i += n_here

    if th.cart is not None and th.cart_installed and opt.kind not in ("inplace",):
        chiller_J += max(th.H - H_issued, 0.0)    # the last cartridge goes back to the rack too
    return ServiceResult(opt.label, sc.name, seed, nbox, stopped, t_stop, min(t, shift_s), work_s,
                         batt_s, trip_s, n_trips, n_swaps, peak, n_two / max(n_cyc, 1),
                         th.E.as_dict(), swap_log, chiller_J, sat_s, tr)
