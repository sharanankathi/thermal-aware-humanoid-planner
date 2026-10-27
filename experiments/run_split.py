"""Do the day's conclusions survive a physics-based winding/housing resistance split?
Usage: python experiments/run_split.py        (results/split/split.csv)

thermal/slot_model.py estimates the winding-to-housing resistance R_wh of the TBM(S)-6013-A from
slot geometry: about 25% of the datasheet TPR for a varnished winding (5-95%: 15-45%) and ~14% if
potted, against the 35% assumed in params/actuators.yaml. This sweep reruns the key comparisons
(air, sinkless loop, radiator, in-place refreeze, best cartridge swap, re-gearing) at three splits:
  assumed35  R_wh = 35% of TPR (as everywhere else in the repo)
  physics25  R_wh = 25%, R_ha = 75% (TPR held at the datasheet 3.55 K/W)
  potted14   R_wh = 14% (potting), R_ha as physics25
"""
from __future__ import annotations

import csv
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("MENAGERIE_PATH", str(Path.home() / "mujoco_menagerie"))

from sim.controllers import ReactiveModeSwitch  # noqa: E402
from sim.cycles import CycleLibrary  # noqa: E402
from sim.gearing import OMEGA_MAX, _kinematics, regear_library  # noqa: E402
from sim.robot import Robot  # noqa: E402
from sim.service import Logistics, ThermalOption, run_service_shift  # noqa: E402
from sim.shift import SCENARIOS, Scenario, joint_params, loss_coeff  # noqa: E402

SPLITS = {"assumed35": {},
          "physics25": {"R_wh": 0.25 / 0.35, "R_ha": 0.75 / 0.65},
          "potted14": {"R_wh": 0.14 / 0.35, "R_ha": 0.75 / 0.65}}
SCEN = {"heavy_high": SCENARIOS["heavy_high"], "warm_heavy": SCENARIOS["warm_heavy"],
        "hot_heavy": Scenario("hot_heavy", masses=(8.0,), shelves=("high",), T_amb=40.0)}
OPTIONS = {
    "air": (ThermalOption("air"), Logistics(), False),
    "loop": (ThermalOption("loop"), Logistics(), False),
    "radiator2": (ThermalOption("radiator", radiator_kg=2), Logistics(), False),
    "inplace2_44C": (ThermalOption("inplace", cartridge_kg=2, cart_override={"T_melt": 44}), Logistics(), False),
    "swap2_18C_15s": (ThermalOption("swap", cartridge_kg=2, cart_override={"T_melt": 18}),
                      Logistics(thermal_trip_s=15.0), False),
    "regeared_air": (ThermalOption("air"), Logistics(), True),
}
_G = {}


def _init():
    r = Robot.load()
    P = joint_params()
    lib = CycleLibrary(r, loss_coeff(P), verbose=False)
    N0 = np.array([p.N for p in P])
    qmax = np.zeros(14)
    for k, c in lib.cycles.items():
        if k[4] == 1.0:
            qmax = np.maximum(qmax, np.abs(_kinematics(c)[0]).max(axis=(0, 1)))
    mult = np.minimum(np.floor(OMEGA_MAX / (N0 * qmax) * 0.98 * 100) / 100, 2.0)
    _G.update(lib=lib, geared=regear_library(lib, r, P, mult, "fixed")[0])


def _job(args):
    split, opt_name, scn, seed = args
    t0 = time.time()
    opt, lg, geared = OPTIONS[opt_name]
    lib = _G["geared"] if geared else _G["lib"]
    res = run_service_shift(lib, ReactiveModeSwitch(130.0, 120.0, 135.0), SCEN[scn], seed, opt, lg,
                            params=joint_params(scale=SPLITS[split] or None))
    return dict(split=split, option=opt_name, scenario=scn, T_amb=SCEN[scn].T_amb, seed=seed,
                boxes=res.boxes, stopped=int(res.stopped), peak_T=round(res.peak_T, 2),
                n_trips=res.n_trips, wall_s=round(time.time() - t0, 1))


def main():
    quick = "--quick" in sys.argv
    jobs = [(sp, o, s, sd) for sp in SPLITS for o in OPTIONS for s in SCEN for sd in range(2)]
    if quick:
        jobs = [(sp, "radiator2", "hot_heavy", 0) for sp in SPLITS]
    out = ROOT / "results/split" / f"split{'_quick' if quick else ''}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"{len(jobs)} runs on {os.cpu_count()} cores -> {out}")
    t0, rows = time.time(), []
    with Pool(initializer=_init) as pool:
        for i, row in enumerate(pool.imap_unordered(_job, jobs), 1):
            rows.append(row)
            if i % 10 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)}  {time.time() - t0:.0f}s", flush=True)
    rows.sort(key=lambda r: (r["split"], r["scenario"], r["option"], r["seed"]))
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"done in {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
