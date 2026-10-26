"""Figures for the cartridge study. Usage: python experiments/make_figures_cartridge.py
Reads results/cartridge/{melt,crossover,joint}.csv; writes figures/cart_fig*.png and
results/cartridge/tables.md. Palette and style follow make_figures_v2.py.
"""
from __future__ import annotations

import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from thermal import pcm  # noqa: E402

RES, FIG = ROOT / "results/cartridge", ROOT / "figures"
INK, MUTED, GRID, BG = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({
    "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
    "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.axisbelow": True,
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG})
C = {"air": "#8f8e89", "loop": "#eda100", "rad": "#1baf7a", "inplace": "#2a78d6", "swap": "#e87ba4",
     "smart": "#008300", "hybrid": "#52514e"}
SCN = {"heavy_high": "25 °C", "warm_heavy": "35 °C", "hot_heavy": "40 °C"}
CEIL = 9734   # always parallel with no temperature limit, same logistics (computed, seed 0)


def load(name):
    return list(csv.DictReader(open(RES / f"{name}.csv")))


def group(rows, *keys):
    d = defaultdict(list)
    for r in rows:
        d[tuple(r[k] for k in keys)].append(float(r["boxes"]))
    return {k: (np.mean(v), np.std(v), len(v)) for k, v in d.items()}


def finish(ax, title):
    ax.set_title(title, loc="left", fontsize=10, color=INK)
    ax.grid(axis="x", visible=False)


# ------------------------------------------------------------------ fig 1: architecture ladder
def fig_ladder(melt, tables):
    g = group(melt, "scenario", "option", "T_melt")
    scns = list(SCN)
    best_swap = {}
    for s in scns:
        cand = {(o, t): g[(s, o, t)][0] for (ss, o, t) in g if ss == s and o in ("swap2", "smart2")}
        best_swap[s] = max(cand.items(), key=lambda kv: kv[1])
    series = [("Air only (v2 robot)", C["air"], [g[(s, "air", "")][0] for s in scns]),
              ("Coolant loop, no sink", C["loop"], [g[(s, "loop", "")][0] for s in scns]),
              ("Loop + 2 kg radiator", C["rad"], [g[(s, "radiator2", "")][0] for s in scns]),
              ("Loop + 2 kg PCM, refrozen at charge dock", C["inplace"],
               [g[(s, "inplace2", "44")][0] for s in scns]),
              ("Loop + 2 kg PCM, swapped (best policy, 150 s trip)", C["swap"],
               [best_swap[s][1] for s in scns])]
    fig, ax = plt.subplots(figsize=(7.6, 4.0))
    x = np.arange(len(scns))
    w = 0.8 / len(series)
    for i, (lab, col, vals) in enumerate(series):
        pos = x + (i - (len(series) - 1) / 2) * w
        ax.bar(pos, vals, w * 0.9, color=col, label=lab, linewidth=0)
        for xp, v in zip(pos, vals):
            ax.text(xp, v + 60, f"{v/1000:.1f}k", ha="center", va="bottom", fontsize=6.5, color=MUTED)
    ax.axhline(CEIL, color=MUTED, lw=0.8, ls=(0, (3, 3)), label=f"{CEIL:,} = no temperature limit")
    ax.set_xticks(x, [f"Heavy boxes, {SCN[s]} room" for s in scns], fontsize=8)
    ax.set_ylabel("Boxes per 8 h shift")
    ax.set_ylim(0, 10800)
    finish(ax, "The coolant loop is the big step; the sink behind it barely matters")
    ax.legend(fontsize=7.5, ncol=2, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.12))
    fig.tight_layout()
    fig.savefig(FIG / "cart_fig1_ladder.png", dpi=200)
    plt.close(fig)
    t = ["| Option | " + " | ".join(SCN[s] for s in scns) + " |", "|---|" + "---|" * len(scns)]
    for lab, _, vals in series:
        t.append(f"| {lab} | " + " | ".join(f"{v:,.0f}" for v in vals) + " |")
    t.append("| Best swap policy / melt point | " + " | ".join(
        f"{best_swap[s][0][0][:-1]}, {best_swap[s][0][1]} °C" for s in scns) + " |")
    tables.append("## Table 1. Boxes per 8 h shift by cooling option (5 seeds)\n\n" + "\n".join(t))


# ------------------------------------------------------------------ fig 2: trip-time crossover
def fig_crossover(cross, tables):
    g = group(cross, "scenario", "option", "T_melt", "rad_UA_per_kg", "trip_s")
    trips = [15, 30, 60, 90, 150]
    lines = [("Swap, 18 °C PCM", "swap2", "18", "", C["swap"], "-"),
             ("Swap, 25 °C PCM", "swap2", "25", "", C["inplace"], (0, (4, 2))),
             ("Smart swap, 18 °C PCM", "smart2", "18", "", C["smart"], (0, (1, 1.5))),
             ("1 kg weak radiator + 1 kg PCM", "hybrid1+1", "25", "5.0", C["hybrid"], (0, (6, 2, 1, 2)))]
    fig, axes = plt.subplots(1, 3, figsize=(7.6, 3.4), sharey=False)
    rows = []
    for ax, s in zip(axes, SCN):
        rad = g[(s, "radiator2", "", "15.0", "")][0]
        weak = g[(s, "radiator2", "", "5.0", "")][0]
        ax.axhspan(min(rad, weak), max(rad, weak) + 1, color=C["rad"], alpha=0.25, lw=0)
        ax.axhline(rad, color=C["rad"], lw=1.4)
        ax.text(150, rad + 12, "2 kg radiator (band: at 1/3 strength)", ha="right", va="bottom",
                fontsize=6.5, color=C["rad"])
        for lab, o, tm, ua, col, ls in lines:
            v = [g[(s, o, tm, ua, str(t))][0] for t in trips]
            ax.plot(trips, v, color=col, ls=ls, lw=1.4, marker="o", ms=3, label=lab)
        ax.set_xticks(trips)
        ax.set_xlabel("Cartridge trip, round trip (s)")
        finish(ax, f"{SCN[s]} room")
        lo = min(g[(s, o, tm, ua, "150")][0] for _, o, tm, ua, *_ in lines)
        ax.set_ylim(lo - 80, rad + 120)
        best15 = max(g[(s, o, tm, ua, "15")][0] for _, o, tm, ua, *_ in lines)
        rows.append((SCN[s], rad, weak, best15,
                     max(g[(s, o, tm, ua, "60")][0] for _, o, tm, ua, *_ in lines),
                     max(g[(s, o, tm, ua, "150")][0] for _, o, tm, ua, *_ in lines)))
    axes[0].set_ylabel("Boxes per 8 h shift")
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, fontsize=7.5, ncol=4, frameon=False, loc="lower center")
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(FIG / "cart_fig2_crossover.png", dpi=200)
    plt.close(fig)
    t = ["| Room | Radiator 15 W/K/kg | Radiator 5 W/K/kg | Best swap, 15 s trip | Best swap, 60 s | Best swap, 150 s |",
         "|---|---|---|---|---|---|"]
    t += [f"| {a} | {b:,.0f} | {c:,.0f} | {d:,.0f} | {e:,.0f} | {f:,.0f} |" for a, b, c, d, e, f in rows]
    tables.append("## Table 2. Swapping vs a radiator as trip time shrinks (3 seeds)\n\n" + "\n".join(t))


# ------------------------------------------------------------------ fig 3: joint heat path
JP = {"nominal": "Nominal joint\n(1.74 K/W)", "coldplate0.1": "Better cold plate\n(1.34 K/W)",
      "coldplate0.1+potted": "Cold plate + potted\nwinding (0.72 K/W)"}


def fig_joint(joint, tables):
    g = group(joint, "scenario", "joint_path", "option", "trip_s")
    swaps = [("coswap2", ""), ("swap2", "15"), ("swap2", "60"), ("smart2", "60"), ("hybrid1+1", "60")]
    fig, axes = plt.subplots(1, 2, figsize=(7.6, 3.6), sharey=True)
    t = ["| Room | Joint path | Loop, no sink | 2 kg radiator | Weak radiator | Best swap | % of no-limit |",
         "|---|---|---|---|---|---|---|"]
    for ax, s in zip(axes, ["heavy_high", "hot_heavy"]):
        x = np.arange(len(JP))
        ser = [("Loop, no sink", C["loop"], [g[(s, j, "loop", "")][0] for j in JP]),
               ("Loop + 2 kg radiator", C["rad"], [g[(s, j, "radiator2", "")][0] for j in JP]),
               ("Loop + PCM, best swap policy", C["swap"],
                [max(g[(s, j, o, tr)][0] for o, tr in swaps) for j in JP])]
        w = 0.8 / len(ser)
        for i, (lab, col, vals) in enumerate(ser):
            ax.bar(x + (i - 1) * w, vals, w * 0.9, color=col, label=lab, linewidth=0)
        ax.axhline(CEIL, color=MUTED, lw=0.8, ls=(0, (3, 3)))
        ax.set_xticks(x, list(JP.values()), fontsize=7.5)
        ax.set_ylim(6000, 10200)
        finish(ax, f"Heavy boxes, {SCN[s]} room")
        for j in JP:
            loop = g[(s, j, "loop", "")][0]
            rad = g[(s, j, "radiator2", "")][0]
            weak = g[(s, j, "radiator2_weak", "")][0]
            bs = max(g[(s, j, o, tr)][0] for o, tr in swaps)
            t.append(f"| {SCN[s]} | {JP[j].replace(chr(10), ' ')} | {loop:,.0f} | {rad:,.0f} | {weak:,.0f} | "
                     f"{bs:,.0f} | {max(loop, rad, bs) / CEIL:.0%} |")
    axes[0].set_ylabel("Boxes per 8 h shift")
    axes[0].text(2.45, CEIL + 40, "no temperature limit", ha="right", va="bottom", fontsize=7, color=MUTED)
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, fontsize=7.5, ncol=3, frameon=False, loc="lower center")
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(FIG / "cart_fig3_joint_path.png", dpi=200)
    plt.close(fig)
    tables.append("## Table 3. Joint heat path vs sink (3 seeds)\n\n" + "\n".join(t))


# ------------------------------------------------------------------ fig 4: PCM model validation
def fig_validation(tables):
    k, rho, cp, L = 5.0, 800.0, 1900.0, 187000.0
    ts, s = pcm.fv_front_fixed_wall(k, rho, cp, L, 60.0, 42.5, 0.06, 300.0, n=150)
    ex = pcm.neumann_front(ts, k, rho, cp, L, 60.0, 42.5)
    err_n = float(np.max(np.abs(s[10:] - ex[10:]) / ex[10:]))
    fig, axes = plt.subplots(1, 2, figsize=(7.6, 3.2))
    ax = axes[0]
    ax.plot(ts, ex * 1000, color=C["inplace"], lw=1.6, label="Exact (Neumann)")
    ax.plot(ts[::3], s[::3] * 1000, "o", ms=3, color=C["swap"], label="Finite-volume enthalpy")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Melt front (mm)")
    finish(ax, "Finite-volume solver vs exact solution")
    ax.legend(fontsize=7.5, frameon=False)
    ax = axes[1]
    rows = []
    for lab, kk, gap, col in [("EG composite, 6 mm gap", 5.0, 0.006, C["inplace"]),
                              ("Pure paraffin, 12 mm gap", 0.2, 0.012, C["loop"])]:
        p = pcm.CartridgeParams.from_yaml(2.0, k=kk, gap=gap)
        tf, ff, _ = pcm.fv_slab(p, 55.0, 2400.0, n=16)
        tl, fl, _ = pcm.lumped_trace(p, 55.0, 2400.0)
        ax.plot(tf / 60, ff, color=col, lw=1.6, label=f"{lab}: finite volume")
        ax.plot(tl[::12] / 60, fl[::12], "o", ms=3, mfc="none", color=col, label=f"{lab}: lumped")
        t90 = lambda t, f: t[np.argmax(f >= 0.9)] if f[-1] >= 0.9 else np.inf
        rows.append((lab, t90(tf, ff), t90(tl, fl), float(np.max(np.abs(np.interp(tl, tf, ff) - fl)))))
    ax.set_xlabel("Time (min), coolant at 55 °C")
    ax.set_ylabel("Melt fraction")
    finish(ax, "Lumped cartridge model vs finite volume")
    ax.legend(fontsize=6.5, frameon=False, loc="lower right")
    fig.tight_layout()
    fig.savefig(FIG / "cart_fig4_validation.png", dpi=200)
    plt.close(fig)
    t = [f"Neumann check: max relative front error {err_n:.2%}.", "",
         "| Cartridge | t90 finite volume (s) | t90 lumped (s) | Max melt-fraction error |", "|---|---|---|---|"]
    t += [f"| {a} | {b:.0f} | {c:.0f} | {d:.3f} |" for a, b, c, d in rows]
    tables.append("## Table 4. PCM model validation\n\n" + "\n".join(t))


def main():
    tables = []
    fig_ladder(load("melt"), tables)
    fig_crossover(load("crossover"), tables)
    fig_joint(load("joint"), tables)
    fig_validation(tables)
    (RES / "tables.md").write_text("# Cartridge study tables (generated)\n\n" + "\n\n".join(tables) + "\n")
    print("\n\n".join(tables))


if __name__ == "__main__":
    main()
