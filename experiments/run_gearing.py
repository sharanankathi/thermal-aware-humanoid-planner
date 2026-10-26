"""Gear-ratio study: does raising (or shifting) the arm gear ratios beat cooling hardware?
Usage:
    python experiments/run_gearing.py shift2h    # 2 h shifts: base vs fixed vs two-speed (x eta)
    python experiments/run_gearing.py shift8h    # 8 h shifts: gearing vs coolant loop + radiator
Results: results/gearing/<name>.csv. Work policy: reactive 130 C limb switch (B8-130).

Gearing variants (sim/gearing.py):
  base        baseline ratios, TBM rotor inertia replacing Apollo's armature
  fixed       each joint raised to 98% of its speed headroom (capped at 2x)
  twospeed_k  ideal two-speed: low gear = baseline, high gear = k x baseline wherever speed allows
eta_hi: efficiency of the raised ratio (0.8 = same as baseline, 0.7 = pessimistic).
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
from sim.robot import JOINT_NAMES, Robot  # noqa: E402
from sim.service import ThermalOption, run_service_shift  # noqa: E402
from sim.shift import SCENARIOS, Scenario, joint_params, loss_coeff, run_shift  # noqa: E402

SCEN = {"heavy_high": SCENARIOS["heavy_high"], "warm_heavy": SCENARIOS["warm_heavy"],
        "hot_heavy": Scenario("hot_heavy", masses=(8.0,), shelves=("high",), T_amb=40.0)}
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
    head = OMEGA_MAX / (N0 * qmax)
    fixed = np.minimum(np.floor(head * 0.98 * 100) / 100, 2.0)
    _G.update(r=r, P=P, lib=lib, fixed=fixed, cache={})


def library(variant, eta_hi):
    key = (variant, eta_hi)
    if key not in _G["cache"]:
        r, P, lib = _G["r"], _G["P"], _G["lib"]
        if variant == "base":
            L, _ = regear_library(lib, r, P, np.ones(14), "fixed")
        elif variant == "fixed":
            L, _ = regear_library(lib, r, P, _G["fixed"], "fixed", eta_hi=eta_hi)
        elif variant.startswith("twospeed_"):
            k = float(variant.split("_")[1])
            L, _ = regear_library(lib, r, P, np.full(14, k), "twospeed", eta_hi=eta_hi)
        else:
            raise ValueError(variant)
        _G["cache"][key] = L
    return _G["cache"][key]


def _job(args):
    exp, variant, eta_hi, cooling, scn, seed = args
    t0 = time.time()
    L = library(variant, eta_hi)
    ctrl = ReactiveModeSwitch(130.0, 120.0, 135.0)
    if exp == "shift2h":
        res = run_shift(L, ctrl, SCEN[scn], seed, record=False)
        boxes, stopped, peak = res.boxes, res.stopped, res.peak_T
    else:
        opt = ThermalOption("air") if cooling == "air" else ThermalOption("radiator", radiator_kg=2)
        res = run_service_shift(L, ctrl, SCEN[scn], seed, opt)
        boxes, stopped, peak = res.boxes, res.stopped, None
    hot = int(np.argmax(peak)) if peak is not None else None
    return dict(exp=exp, variant=variant, eta_hi=eta_hi if eta_hi else 0.8, cooling=cooling,
                scenario=scn, T_amb=SCEN[scn].T_amb, seed=seed, boxes=boxes, stopped=int(stopped),
                peak_T=round(float(np.max(peak)), 2) if peak is not None else "",
                hottest=JOINT_NAMES[hot] if hot is not None else "",
                wall_s=round(time.time() - t0, 1))


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "shift2h"
    if which == "shift2h":
        variants = [("base", None)] + [(v, e) for v in ("fixed", "twospeed_1.5", "twospeed_2.0",
                                                         "twospeed_3.0") for e in (None, 0.7)]
        jobs = [(which, v, e, "air", s, sd) for s in ("heavy_high", "warm_heavy")
                for v, e in variants for sd in range(5)]
    elif which == "shift8h":
        variants = [("base", None), ("fixed", None), ("fixed", 0.7)]
        jobs = [(which, v, e, c, s, sd) for s in ("heavy_high", "warm_heavy", "hot_heavy")
                for v, e in variants for c in ("air", "radiator") for sd in range(3)]
    else:
        raise SystemExit(f"unknown experiment {which}")
    out = ROOT / "results/gearing" / f"{which}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"{len(jobs)} runs on {os.cpu_count()} cores -> {out}")
    t0, rows = time.time(), []
    with Pool(initializer=_init) as pool:
        for i, row in enumerate(pool.imap_unordered(_job, jobs, chunksize=2), 1):
            rows.append(row)
            if i % 10 == 0 or i == len(jobs):
                print(f"  {i}/{len(jobs)}  {time.time() - t0:.0f}s", flush=True)
    rows.sort(key=lambda r: (r["scenario"], r["variant"], r["eta_hi"], r["cooling"], r["seed"]))
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"done in {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
