"""Figures and tables for the gear-ratio study. Usage: python experiments/make_figures_gearing.py
Reads results/gearing/{shift8h,shift2h,ratio,ratios}.csv (ratio.csv optional);
writes figures/gear_fig*.png and results/gearing/tables.md. Style follows make_figures_v2.py.
"""
from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RES, FIG = ROOT / "results/gearing", ROOT / "figures"
INK, MUTED, GRID, BG = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({
    "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
    "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.axisbelow": True,
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG})
C = {"air": "#8f8e89", "rad": "#1baf7a", "gear": "#2a78d6", "gear70": "#e87ba4", "two": "#eda100"}
SCN = {"heavy_high": "25 °C", "warm_heavy": "35 °C", "hot_heavy": "40 °C"}
CEIL8, CEIL2 = 9734, 2668


def load(name):
    p = RES / f"{name}.csv"
    return list(csv.DictReader(open(p))) if p.exists() else None


def group(rows, *keys, val="boxes"):
    d = defaultdict(list)
    for r in rows:
        d[tuple(r[k] for k in keys)].append(float(r[val]))
    return {k: float(np.mean(v)) for k, v in d.items()}


def finish(ax, title):
    ax.set_title(title, loc="left", fontsize=10, color=INK)
    ax.grid(axis="x", visible=False)


def fig_8h(rows, tables):
    g = group(rows, "scenario", "variant", "eta_hi", "cooling")
    scns = list(SCN)
    ser = [("Current gearing, air", C["air"], ("base", "0.8", "air")),
           ("Current gearing + loop + 2 kg radiator", C["rad"], ("base", "0.8", "radiator")),
           ("Re-geared, air (gear eff. 80%)", C["gear"], ("fixed", "0.8", "air")),
           ("Re-geared, air (gear eff. 70%)", C["gear70"], ("fixed", "0.7", "air"))]
    fig, ax = plt.subplots(figsize=(7.6, 4.0))
    x = np.arange(len(scns))
    w = 0.8 / len(ser)
    for i, (lab, col, key) in enumerate(ser):
        vals = [g[(s, *key)] for s in scns]
        pos = x + (i - (len(ser) - 1) / 2) * w
        ax.bar(pos, vals, w * 0.9, color=col, label=lab, linewidth=0)
        for xp, v in zip(pos, vals):
            ax.text(xp, v + 60, f"{v/1000:.1f}k", ha="center", va="bottom", fontsize=6.5, color=MUTED)
    ax.axhline(CEIL8, color=MUTED, lw=0.8, ls=(0, (3, 3)), label=f"{CEIL8:,} = no temperature limit")
    ax.set_xticks(x, [f"Heavy boxes, {SCN[s]} room" for s in scns], fontsize=8)
    ax.set_ylabel("Boxes per 8 h shift")
    ax.set_ylim(0, 10800)
    finish(ax, "Re-gearing beats a liquid-cooling system, with no added hardware")
    ax.legend(fontsize=7.5, ncol=2, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.12))
    fig.tight_layout()
    fig.savefig(FIG / "gear_fig1_8h.png", dpi=200)
    plt.close(fig)
    t = ["| Option | " + " | ".join(SCN[s] for s in scns) + " |", "|---|" + "---|" * len(scns)]
    for lab, _, key in ser + [("Re-geared + loop + 2 kg radiator (70%)", None, ("fixed", "0.7", "radiator"))]:
        t.append(f"| {lab} | " + " | ".join(f"{g[(s, *key)]:,.0f}" for s in scns) + " |")
    tables.append("## Table 1. Boxes per 8 h shift (3 seeds)\n\n" + "\n".join(t))


def fig_twospeed(rows, tables):
    g = group(rows, "scenario", "variant", "eta_hi")
    pk = group(rows, "scenario", "variant", "eta_hi", val="peak_T")
    variants = [("base", "Current"), ("fixed", "Best fixed"), ("twospeed_1.5", "Two-speed 1.5x"),
                ("twospeed_2.0", "Two-speed 2x"), ("twospeed_3.0", "Two-speed 3x")]
    fig, axes = plt.subplots(1, 2, figsize=(7.6, 3.5), sharey=True)
    t = ["| Room | Gear eff. | " + " | ".join(l for _, l in variants) + " |", "|---|---|" + "---|" * len(variants)]
    for ax, s in zip(axes, ["heavy_high", "warm_heavy"]):
        x = np.arange(len(variants))
        for j, (eta, col) in enumerate((("0.8", C["gear"]), ("0.7", C["gear70"]))):
            vals = [g.get((s, v, eta if v != "base" else "0.8"), np.nan) for v, _ in variants]
            ax.bar(x + (j - 0.5) * 0.38, vals, 0.36, color=col, linewidth=0,
                   label=f"High-gear efficiency {int(float(eta) * 100)}%")
            t.append(f"| {SCN[s]} | {int(float(eta)*100)}% | " + " | ".join(
                f"{v:,.0f} ({pk.get((s, k, eta if k != 'base' else '0.8'), np.nan):.0f} °C)"
                for v, (k, _) in zip(vals, variants)) + " |")
        ax.axhline(CEIL2, color=MUTED, lw=0.8, ls=(0, (3, 3)))
        ax.set_xticks(x, [l for _, l in variants], fontsize=7.5, rotation=20, ha="right")
        ax.set_ylim(1000, 2900)
        finish(ax, f"Heavy boxes, {SCN[s]} room (2 h)")
    axes[0].set_ylabel("Boxes per 2 h shift")
    axes[1].text(4.4, CEIL2 + 25, "no temperature limit", ha="right", va="bottom", fontsize=7, color=MUTED)
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, fontsize=7.5, ncol=2, frameon=False, loc="lower center")
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(FIG / "gear_fig2_twospeed.png", dpi=200)
    plt.close(fig)
    tables.append("## Table 2. Fixed vs two-speed gearing, 2 h shift (5 seeds; peak winding in brackets)\n\n"
                  + "\n".join(t))


def fig_ratio(rows, tables):
    g = group(rows, "scenario", "variant", "eta_hi")
    us = [0.75, 1.0, 1.25, 1.5, 1.75, 2.0]
    fig, ax = plt.subplots(figsize=(7.6, 3.6))
    t = ["| Room | Gear eff. | " + " | ".join(f"{u}x" for u in us) + " |", "|---|---|" + "---|" * len(us)]
    for s, ls in (("heavy_high", "-"), ("warm_heavy", (0, (4, 2)))):
        for eta, col in (("0.8", C["gear"]), ("0.7", C["gear70"])):
            v = [g[(s, f"ratio_{u}", eta)] for u in us]
            ax.plot(us, v, color=col, ls=ls, lw=1.5, marker="o", ms=3.5,
                    label=f"{SCN[s]} room, {int(float(eta)*100)}% efficiency")
            t.append(f"| {SCN[s]} | {int(float(eta)*100)}% | " + " | ".join(f"{x:,.0f}" for x in v) + " |")
    ax.axhline(CEIL2, color=MUTED, lw=0.8, ls=(0, (3, 3)))
    ax.text(0.76, CEIL2 + 25, "no temperature limit", fontsize=7, color=MUTED, va="bottom")
    ax.set_xlabel("Gear ratio multiplier (each joint capped at its motor speed limit)")
    ax.set_ylabel("Boxes per 2 h shift")
    finish(ax, "Throughput climbs steeply with gear ratio until the task ceiling")
    ax.legend(fontsize=7.5, frameon=False, loc="lower right")
    fig.tight_layout()
    fig.savefig(FIG / "gear_fig3_ratio.png", dpi=200)
    plt.close(fig)
    tables.append("## Table 3. Throughput vs ratio multiplier, 2 h shift (5 seeds; efficiency applies above 1x)\n\n"
                  + "\n".join(t))


def table_inertia(rows, tables):
    t = ["| Joint | Ratio now | Speed headroom | Best fixed ratio | Apollo reflected inertia | "
         "TBM at current ratio | at best fixed | at 2x |", "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        t.append(f"| {r['joint']} | {float(r['N0']):.0f} | {float(r['speed_headroom']):.2f}x | "
                 f"{float(r['N_fixed']):.0f} | {r['apollo_armature_kgm2']} | {r['J_refl_N0_kgm2']} | "
                 f"{r['J_refl_fixed_kgm2']} | {r['J_refl_2x_kgm2']} |")
    tables.append("## Table 4. Gear ratios and joint-side reflected rotor inertia (kg·m²)\n\n" + "\n".join(t))


def main():
    tables = []
    fig_8h(load("shift8h"), tables)
    fig_twospeed(load("shift2h"), tables)
    ratio = load("ratio")
    if ratio:
        fig_ratio(ratio, tables)
    else:
        print("results/gearing/ratio.csv not found: run `python experiments/run_gearing.py ratio` first")
    table_inertia(load("ratios"), tables)
    (RES / "tables.md").write_text("# Gear-ratio study tables (generated)\n\n" + "\n\n".join(tables) + "\n")
    print("\n\n".join(tables))


if __name__ == "__main__":
    main()
