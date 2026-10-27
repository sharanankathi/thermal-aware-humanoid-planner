"""Figure for the stator-slot study. Usage: python experiments/make_figures_slot.py
Runs thermal/slot_model.py (a few seconds plus a 150-sample uncertainty sweep, ~30 s) and writes
figures/slot_fig1.png and results/slot/slot.csv.
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from thermal.slot_model import Geometry, r_wh  # noqa: E402

INK, MUTED, GRID, BG = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False,
                     "axes.spines.right": False, "figure.facecolor": BG, "axes.facecolor": BG,
                     "savefig.facecolor": BG})
RANGES = {"phi_slot": (0.30, 0.50), "enamel_ratio": (0.05, 0.12), "n_slots": (12, 24),
          "t_yoke": (2.5e-3, 4.0e-3), "w_tooth": (2.5e-3, 4.0e-3), "t_liner": (0.10e-3, 0.25e-3),
          "r_contact": (5e-5, 5e-4), "D_in": (37e-3, 42e-3), "h_end_air": (5.0, 30.0),
          "t_pot_end": (1.5e-3, 5e-3)}


def main(n_mc=150):
    g0 = Geometry()
    nom = {m: r_wh(g0, m, return_parts=True) for m in ("air", "varnish", "epoxy")}
    rng = np.random.default_rng(0)
    mc = []
    for _ in range(n_mc):
        kw = {k: (int(rng.choice([12, 18, 24])) if k == "n_slots" else float(rng.uniform(*v)))
              for k, v in RANGES.items()}
        g = g0.with_(**kw)
        mc.append([r_wh(g, m, h=0.08e-3) for m in ("air", "varnish", "epoxy")])
    mc = np.array(mc)
    out = ROOT / "results/slot"
    out.mkdir(parents=True, exist_ok=True)
    with (out / "slot.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["case", "R_wh_nominal", "R_wh_p5", "R_wh_median", "R_wh_p95", "share_of_TPR_median"])
        for j, m in enumerate(("air", "varnish", "epoxy")):
            p5, p50, p95 = np.percentile(mc[:, j], [5, 50, 95])
            w.writerow([m, round(nom[m][0], 3), round(p5, 3), round(p50, 3), round(p95, 3), round(p50 / 3.55, 3)])
        cut = 1 - mc[:, 2] / mc[:, 1]
        w.writerow(["potting_cut_vs_varnish", round(1 - nom["epoxy"][0] / nom["varnish"][0], 3),
                    *np.round(np.percentile(cut, [5, 50, 95]), 3), ""])

    fig, axes = plt.subplots(1, 3, figsize=(7.6, 3.3), gridspec_kw={"width_ratios": [1.05, 1, 1]})
    info = nom["varnish"][1]["info"]
    T = info["T"] * 1000 * 1.0  # K per (W/m) x 1000 -> mK per W/m (relative map)
    ax = axes[0]
    im = ax.pcolormesh(info["X"] * 1e3, info["Y"] * 1e3, np.ma.masked_invalid(T), cmap="inferno",
                       shading="auto")
    ax.contour(info["X"] * 1e3, info["Y"] * 1e3, info["bundle"].astype(float), [0.5], colors="w",
               linewidths=0.6)
    ax.set_aspect("equal")
    ax.set_xlabel("mm"); ax.set_ylabel("mm")
    ax.set_title("One slot pitch, varnished\n(relative temperature rise)", loc="left", fontsize=8.5)
    ax = axes[1]
    labels = ["Bare\n(air)", "Varnish", "Potted\n(epoxy)"]
    med = np.median(mc, axis=0)
    lo, hi = np.percentile(mc, 5, axis=0), np.percentile(mc, 95, axis=0)
    cols = ["#8f8e89", "#2a78d6", "#1baf7a"]
    ax.bar(range(3), med, color=cols, width=0.6)
    ax.errorbar(range(3), med, yerr=[med - lo, hi - med], fmt="none", ecolor=INK, capsize=3, lw=0.8)
    ax.axhline(0.35 * 3.55, color="#e87ba4", ls=(0, (4, 2)), lw=1.2)
    ax.text(2.45, 0.35 * 3.55 + 0.06, "assumed in the\nsimulations (35%)", ha="right", fontsize=7,
            color="#e87ba4")
    ax.set_xticks(range(3), labels, fontsize=8)
    ax.set_ylabel("Winding-to-housing R (K/W)")
    ax.set_title("Median and 5–95% band", loc="left", fontsize=8.5)
    ax = axes[2]
    ax.hist(100 * (1 - mc[:, 2] / mc[:, 1]), bins=18, color="#1baf7a")
    ax.axvline(25, color=INK, ls=(0, (3, 3)), lw=0.9)
    ax.text(26, ax.get_ylim()[1] * 0.92, "kill threshold\n25%", fontsize=7, color=INK)
    ax.set_xlabel("Potting cut vs varnish (%)")
    ax.set_ylabel("Count of 150 geometry samples")
    ax.set_title("Potting effect", loc="left", fontsize=8.5)
    fig.tight_layout()
    fig.savefig(ROOT / "figures/slot_fig1.png", dpi=200)
    print(open(out / "slot.csv").read())


if __name__ == "__main__":
    main()
