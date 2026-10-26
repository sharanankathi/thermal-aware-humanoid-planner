"""Swappable PCM cartridge study: sweeps. Usage:
    python experiments/run_cartridge.py melt            # options x melt temp x 3 ambients x 5 seeds
    python experiments/run_cartridge.py melt --quick    # 1 seed, 1 scenario (smoke test)
    python experiments/run_cartridge.py crossover       # cartridge trip time x radiator strength x ambient
Results: results/cartridge/<name>.csv
Work policy is fixed to B8-130 (reactive limb switch); only cooling hardware/logistics vary.
"""
from __future__ import annotations

import csv
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("MENAGERIE_PATH", str(Path.home() / "mujoco_menagerie"))

from sim.controllers import ReactiveModeSwitch  # noqa: E402
from sim.cycles import CycleLibrary  # noqa: E402
from sim.robot import Robot  # noqa: E402
from sim.service import Logistics, ThermalOption, run_service_shift  # noqa: E402
from sim.shift import SCENARIOS, Scenario, joint_params, loss_coeff  # noqa: E402

SCEN = {
    "heavy_high": SCENARIOS["heavy_high"],                       # 25 C
    "warm_heavy": SCENARIOS["warm_heavy"],                       # 35 C
    "hot_heavy": Scenario("hot_heavy", masses=(8.0,), shelves=("high",), T_amb=40.0),
}
MELTS = (18, 25, 32, 44)


def melt_options():
    opts = [("air", None, ThermalOption("air")),
            ("loop", None, ThermalOption("loop")),
            ("radiator2", None, ThermalOption("radiator", radiator_kg=2))]
    for Tm in MELTS:
        ov = {"T_melt": Tm}
        opts += [(f"inplace2", Tm, ThermalOption("inplace", cartridge_kg=2, cart_override=ov)),
                 (f"swap2", Tm, ThermalOption("swap", cartridge_kg=2, cart_override=ov)),
                 (f"coswap2", Tm, ThermalOption("coswap", cartridge_kg=2, cart_override=ov)),
                 (f"smart2", Tm, ThermalOption("smart", cartridge_kg=2, cart_override=ov)),
                 (f"hybrid1+1", Tm, ThermalOption("smart", radiator_kg=1, cartridge_kg=1,
                                                  cart_override=ov))]
    return opts


TRIPS = (15, 30, 60, 90, 150)     # s, full cartridge-trip round trip (melt sweep used 150)
RAD_UA = (5.0, 15.0)             # W/K per kg: radiator buried in a cramped torso vs nominal


def crossover_jobs():
    """(name, T_melt, option, logistics, extra columns) for the trip-time / radiator crossover."""
    out = []
    for ua in RAD_UA:
        out.append((f"radiator2", None, ThermalOption("radiator", radiator_kg=2, ua_rad_per_kg=ua),
                    Logistics(), {"rad_UA_per_kg": ua, "trip_s": ""}))
    out.append(("inplace2", 44, ThermalOption("inplace", cartridge_kg=2, cart_override={"T_melt": 44}),
                Logistics(), {"rad_UA_per_kg": "", "trip_s": ""}))
    for trip in TRIPS:
        lg = Logistics(thermal_trip_s=float(trip))
        x = {"rad_UA_per_kg": "", "trip_s": trip}
        for Tm in (18, 25):
            out.append(("swap2", Tm, ThermalOption("swap", cartridge_kg=2, cart_override={"T_melt": Tm}), lg, x))
        out.append(("smart2", 18, ThermalOption("smart", cartridge_kg=2, cart_override={"T_melt": 18}), lg, x))
        out.append(("hybrid1+1", 25, ThermalOption("smart", radiator_kg=1, cartridge_kg=1, ua_rad_per_kg=5.0,
                                                   cart_override={"T_melt": 25}), lg,
                    {"rad_UA_per_kg": 5.0, "trip_s": trip}))
    return out


_LIB = None


def _init():
    global _LIB
    _LIB = CycleLibrary(Robot.load(), loss_coeff(joint_params()), verbose=False)


def _job(args):
    name, Tm, opt, scn, seed = args[:5]
    lg, extra = (args[5], args[6]) if len(args) > 5 else (Logistics(), {})
    t0 = time.time()
    r = run_service_shift(_LIB, ReactiveModeSwitch(130.0, 120.0, 135.0), SCEN[scn], seed, opt, lg)
    E = r.energy
    return dict(option=name, T_melt=Tm if Tm is not None else "", **extra, scenario=scn,
                T_amb=SCEN[scn].T_amb, seed=seed, boxes=r.boxes, stopped=int(r.stopped),
                peak_T=round(r.peak_T, 2), frac_two_arm=round(r.frac_two_arm, 4),
                n_trips=r.n_trips, n_swaps=r.n_swaps, trip_min=round(r.thermal_trip_s / 60, 2),
                batt_min=round(r.battery_s / 60, 2), sat_min=round(r.sat_s / 60, 2),
                copper_Wh=round(E["copper"] / 3600, 1), coldplate_Wh=round(E["coldplate"] / 3600, 1),
                radiator_Wh=round(E["radiator"] / 3600, 1), cartridge_Wh=round(E["cartridge"] / 3600, 1),
                dock_Wh=round(E["dock"] / 3600, 1), chiller_Wh=round(r.chiller_J / 3600, 1),
                wall_s=round(time.time() - t0, 1))


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "melt"
    quick = "--quick" in sys.argv
    assert which in ("melt", "crossover"), which
    scns = ["hot_heavy"] if quick else list(SCEN)
    if which == "melt":
        seeds = range(1) if quick else range(5)
        opts = melt_options()[:4] if quick else melt_options()
        jobs = [(n, Tm, o, s, sd) for s in scns for (n, Tm, o) in opts for sd in seeds]
    else:
        seeds = range(1) if quick else range(3)
        cj = crossover_jobs()[:4] if quick else crossover_jobs()
        jobs = [(n, Tm, o, s, sd, lg, x) for s in scns for (n, Tm, o, lg, x) in cj for sd in seeds]
    out = ROOT / "results/cartridge" / f"{which}{'_quick' if quick else ''}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"{len(jobs)} runs on {os.cpu_count()} cores -> {out}")
    t0, rows = time.time(), []
    with Pool(initializer=_init) as pool:
        for i, row in enumerate(pool.imap_unordered(_job, jobs), 1):
            rows.append(row)
            if i % 10 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)}  {time.time() - t0:.0f}s", flush=True)
    rows.sort(key=lambda r: (r["scenario"], r["option"], str(r["T_melt"]), str(r.get("trip_s", "")),
                             str(r.get("rad_UA_per_kg", "")), r["seed"]))
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"done in {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
