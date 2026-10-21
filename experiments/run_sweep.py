"""Experiment sweeps. Usage:
    python experiments/run_sweep.py main        # 6 scenarios x 20 seeds x 8 controllers
    python experiments/run_sweep.py ablation    # levers alone + forecast horizon margin
    python experiments/run_sweep.py paramerr    # plant params +/-30% (random per seed)
Results: results/<name>.csv, plus seed-0 temperature traces in results/traces_<name>.pkl
"""
from __future__ import annotations

import csv
import os
import pickle
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("MENAGERIE_PATH", str(Path.home() / "mujoco_menagerie"))

from sim.controllers import (DominantArm, EnergyOptimal, ReactiveDerate, RoundRobin,  # noqa: E402
                             RoundRobinEnergy, ThermalBudget, AlwaysTwoArm,
                             AlwaysDual, ReactiveModeSwitch)
from sim.cycles import CycleLibrary  # noqa: E402
from sim.robot import JOINT_NAMES, Robot  # noqa: E402
from sim.shift import SCENARIOS, joint_params, loss_coeff, run_shift  # noqa: E402

SEEDS = range(20)
CTRL = {
    "B1": DominantArm, "B2": RoundRobin, "B3": EnergyOptimal, "B4": ReactiveDerate,
    "B5": RoundRobinEnergy, "B6": AlwaysTwoArm, "B7": AlwaysDual,
    "B8": ReactiveModeSwitch,                                              # 120 / 110 C
    "B8-130": lambda: ReactiveModeSwitch(130.0, 120.0, 135.0),             # tuned hotter
    "B8-135": lambda: ReactiveModeSwitch(135.0, 125.0, 138.0),             # as hot as P runs
    "P": ThermalBudget,
    "P-nolook": lambda: ThermalBudget(lookahead=False, name="P no lookahead"),
    "P-arm": lambda: ThermalBudget(use_posture=False, use_pace=False, name="P arm only"),
    "P-ik": lambda: ThermalBudget(use_arm=False, use_pace=False, name="P IK only"),
    "P-pace": lambda: ThermalBudget(use_arm=False, use_posture=False, name="P pace only"),
    "P-noadapt": lambda: ThermalBudget(adapt=False, name="P no online correction"),
    "P-nomargin": lambda: ThermalBudget(margin_s=0.0, name="P margin 0"),
    "P-margin10": lambda: ThermalBudget(margin_s=600.0, name="P margin 10 min"),
}
PARAM_KEYS = ("R_wh", "R_ha", "C_w", "C_h", "R25", "Kt_cold", "Kt_hot")

# Sensitivity to the provisional node split and capacitances (plant AND controller use the
# variant, so this tests whether the conclusions depend on those estimates, not model error).
# Total thermal resistance TPR is held at the datasheet value; nominal split is 35/65.
SPLIT = lambda f: {"R_wh": f / 0.35, "R_ha": (1 - f) / 0.65}
VARIANTS = {
    "split20": SPLIT(0.20), "split50": SPLIT(0.50),
    "Cw_x0.5": {"C_w": 0.5}, "Cw_x2": {"C_w": 2.0},
    "Ch_x0.5": {"C_h": 0.5}, "Ch_x2": {"C_h": 2.0},
}

_LIB = None


def _init():
    global _LIB
    _LIB = CycleLibrary(Robot.load(), loss_coeff(joint_params()), verbose=False)


def _perturbed(seed):
    rng = np.random.default_rng(10_000 + seed)
    # one factor per parameter, shared by all joints (a systematically wrong datasheet / housing)
    f = {k: float(rng.uniform(0.7, 1.3)) for k in PARAM_KEYS}
    f["Kt_hot"] = f["Kt_cold"]
    return f


def _job(args):
    ctrl_key, scn, seed, perturb = args[:4]
    variant = args[4] if len(args) > 4 else None
    plant = joint_params(scale=_perturbed(seed)) if perturb else None
    ctrl_params = joint_params() if perturb else None
    if variant:
        plant = joint_params(scale=VARIANTS[variant])
    t0 = time.time()
    res = run_shift(_LIB, CTRL[ctrl_key](), SCENARIOS[scn], seed, params=plant,
                    ctrl_params=ctrl_params, record=(seed == 0))
    row = dict(ctrl=ctrl_key, scenario=scn, seed=seed, perturb=int(perturb), variant=variant or "",
               boxes=res.boxes, stopped=int(res.stopped),
               t_stop_min=(res.t_stop / 60 if res.stopped else np.nan),
               t_end_min=res.t_end / 60, cycle_s=res.cycle_time_mean,
               energy_Wh=res.energy_J / 3600, peak_T=float(res.peak_T.max()),
               hottest=JOINT_NAMES[int(np.argmax(res.peak_T))],
               T_spread=float(res.peak_T.max() - res.peak_T[[2, 3, 9, 10]].min()),
               n_l=res.arm_counts["l"], n_r=res.arm_counts["r"], n_both=res.arm_counts["both"],
               n_dual=res.arm_counts.get("dual", 0),
               peak_tau_ratio=res.peak_torque_ratio, wall_s=time.time() - t0)
    if perturb:
        row.update({f"f_{k}": v for k, v in _perturbed(seed).items()})
    trace = (ctrl_key, scn, res.trace_t, res.trace_T) if (seed == 0 and not variant) else None
    return row, trace


def main(which: str):
    HEAVY = ("heavy_high", "warm_heavy", "conveyor_r", "warm_start")
    BASE = ("B2", "B4", "B6", "B7", "B8", "B8-130", "B8-135")
    if which == "main":
        jobs = [(c, s, k, False) for s in SCENARIOS for c in BASE + ("P",) for k in SEEDS]
    elif which == "paramerr":
        jobs = [(c, s, k, True) for s in HEAVY
                for c in ("B6", "B8", "B8-130", "B8-135", "P", "P-noadapt") for k in SEEDS]
    elif which == "ablation":
        jobs = [(c, s, k, False) for s in ("warm_heavy", "conveyor_r", "warm_start")
                for c in ("P-nolook", "P-arm", "P-ik", "P-pace", "P-nomargin", "P-margin10")
                for k in range(10)]
    elif which == "sensitivity":
        jobs = [(c, s, k, False, v) for v in VARIANTS for s in ("warm_heavy", "conveyor_r")
                for c in ("B6", "B8", "B8-135", "P") for k in range(10)]
    else:
        raise SystemExit(__doc__)
    # slow jobs first so the pool stays busy
    jobs.sort(key=lambda j: not j[0].startswith("P"))
    print(f"{which}: {len(jobs)} runs on {os.cpu_count()} cores", flush=True)
    out = ROOT / "results"
    out.mkdir(exist_ok=True)
    rows, traces = [], {}
    t0 = time.time()
    with Pool(os.cpu_count(), initializer=_init) as pool:
        for i, (row, tr) in enumerate(pool.imap_unordered(_job, jobs), 1):
            rows.append(row)
            if tr:
                traces[(tr[0], tr[1])] = (tr[2], tr[3])
            if i % 20 == 0 or i == len(jobs):
                print(f"{i}/{len(jobs)} done, {time.time() - t0:.0f}s", flush=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(out / f"{which}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: (r["scenario"], r["ctrl"], r["seed"])))
    with open(out / f"traces_{which}.pkl", "wb") as f:
        pickle.dump(traces, f)
    print("wrote", out / f"{which}.csv")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "main")
