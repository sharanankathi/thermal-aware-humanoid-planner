"""v2 figures and tables (parallel lifting). Usage: python experiments/make_figures_v2.py

Colors validated with the dataviz palette checker. Bars (adjacent): B6 yellow, B8 green, B8-130 aqua,
B8-135 magenta, Planner blue, with 2 px gaps as secondary encoding. Lines (all pairs): orange, aqua,
blue, plus neutral grey; every line is also direct-labelled and dashed differently.
"""
from __future__ import annotations

import csv
import pickle
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RES, FIG = ROOT / "results", ROOT / "figures"
INK, MUTED, GRID, BG = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({
    "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
    "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.axisbelow": True,
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG})

BAR = {"B6": "#eda100", "B8": "#008300", "B8-130": "#1baf7a", "B8-135": "#e87ba4", "P": "#2a78d6"}
LABEL = {"B2": "Alternate arms", "B4": "Reactive slow-down (one arm)", "B6": "Always two-arm",
         "B7": "Always parallel", "B8": "Reactive switch 120 °C", "B8-130": "Reactive switch 130 °C",
         "B8-135": "Reactive switch 135 °C", "P": "Forecast planner",
         "P-noadapt": "Planner, no correction", "P-nolook": "Planner, no lookahead",
         "P-arm": "Planner: limb choice only", "P-ik": "Planner: posture only",
         "P-pace": "Planner: pacing only", "P-nomargin": "Planner, 0 s margin",
         "P-margin10": "Planner, 10 min margin"}
SCN = {"light_std": "Light boxes", "heavy_high": "Heavy, high shelf", "warm_heavy": "Heavy, 35 °C room",
       "mixed": "Mixed payloads", "conveyor_r": "Heavy, right-side\nconveyor",
       "warm_start": "Heavy, right arm\nstarts warm"}
HEAVY = ["heavy_high", "warm_heavy", "conveyor_r", "warm_start"]


def load(name):
    return list(csv.DictReader(open(RES / f"{name}.csv")))


def agg(rows):
    d = defaultdict(list)
    for r in rows:
        d[(r["scenario"], r["ctrl"])].append(r)
    return d


def stat(rr):
    b = np.array([float(r["boxes"]) for r in rr])
    return b.mean(), b.std(), sum(int(r["stopped"]) for r in rr), \
        np.mean([float(r["peak_T"]) for r in rr]), len(rr)


def bars(rows, ctrls, scns, title, ylabel, out, ylim=None):
    d = agg(rows)
    fig, ax = plt.subplots(figsize=(7.6, 4.0))
    w = 0.8 / len(ctrls)
    x = np.arange(len(scns))
    for i, c in enumerate(ctrls):
        st = [stat(d[(s, c)]) for s in scns]
        pos = x + (i - (len(ctrls) - 1) / 2) * w
        ax.bar(pos, [m for m, *_ in st], w * 0.92, color=BAR[c], label=LABEL[c], linewidth=0)
        for xp, (m, sd, ns, pk, n) in zip(pos, st):
            if ns:
                ax.text(xp, m + 18, f"{ns}✕", ha="center", va="bottom", fontsize=7, color=INK)
    ax.axhline(1334, color=MUTED, lw=0.8, ls=(0, (3, 3)),
               label="1,334 = most an always-two-arm robot can do")
    ax.set_xticks(x, [SCN[s] for s in scns], fontsize=8)
    ax.set_ylabel(ylabel)
    if ylim:
        ax.set_ylim(*ylim)
    ax.set_title(title, loc="left", fontsize=10, color=INK)
    ax.legend(fontsize=7.5, ncol=3, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.17))
    ax.grid(axis="x", visible=False)
    fig.tight_layout()
    fig.savefig(FIG / out, dpi=200)
    plt.close(fig)


def traces(scn="warm_heavy"):
    tr = pickle.load(open(RES / "traces_main.pkl", "rb"))
    spec = [("B6", "#8a8984", (0, (1, 1.5)), "Always two-arm"),
            ("B7", "#eb6834", "--", "Always parallel"),
            ("B8-130", "#1baf7a", (0, (5, 1.5, 1, 1.5)), "Reactive switch 130 °C"),
            ("P", "#2a78d6", "-", "Forecast planner")]
    fig, ax = plt.subplots(figsize=(7.2, 3.5))
    ends = []
    for c, col, ls, lab in spec:
        t, T = tr[(c, scn)]
        h = T.max(1)
        ax.plot(t / 60, h, color=col, ls=ls, lw=1.8 if c != "P" else 2.2)
        stopped = t[-1] < 7200 - 30
        if stopped:
            ax.plot(t[-1] / 60, h[-1], "x", color=INK, ms=7, mew=2)
            ax.annotate(f"{lab}: protective stop at {t[-1] / 60:.0f} min", (t[-1] / 60, h[-1]),
                        xytext=(10, -46), textcoords="offset points", fontsize=8, color=INK,
                        arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6))
        else:
            ends.append((h[-1], lab))
    ends.sort()
    ys = []
    for y, lab in ends:                       # keep end labels apart
        y = max(y, ys[-1] + 5.5) if ys else y
        ys.append(y)
        ax.annotate(lab, (120.5, y), fontsize=8, color=INK, va="center")
    ax.axhline(140, color=INK, lw=1, ls=(0, (4, 3)))
    ax.text(60, 142, "140 °C protective stop", fontsize=8, color=MUTED, ha="center")
    ax.set_xlim(0, 150)
    ax.set_ylim(top=152)
    ax.set_xticks([0, 20, 40, 60, 80, 100, 120])
    ax.set_xlabel("time into shift (min)")
    ax.set_ylabel("hottest of 14 windings (°C)")
    ax.set_title("8 kg boxes, 35 °C room: parallel lifting overheats in minutes; switching holds",
                 loc="left", fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(FIG / "v2_fig_traces.png", dpi=200)
    plt.close(fig)


def table(rows, ctrls, scns, name, note=""):
    d = agg(rows)
    lines = [f"# {name}\n", note, "| Scenario | " + " | ".join(LABEL[c] for c in ctrls) + " |",
             "|---|" + "---|" * len(ctrls)]
    for s in scns:
        cells = []
        for c in ctrls:
            if (s, c) not in d:
                cells.append("-")
                continue
            m, sd, ns, pk, n = stat(d[(s, c)])
            cells.append(f"{m:.0f}" + (f" ({ns}/{n} ✕)" if ns else "") + f", {pk:.0f} °C")
        lines.append(f"| {SCN[s].replace(chr(10), ' ')} | " + " | ".join(cells) + " |")
    (RES / f"v2_table_{name}.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines) + "\n")


if __name__ == "__main__":
    main, pe, ab, se = load("main"), load("paramerr"), load("ablation"), load("sensitivity")
    traces()
    bars(main, ["B6", "B8", "B8-130", "B8-135", "P"], HEAVY,
         "Boxes per 2-hour shift, exact thermal model (mean of 20 runs; ✕ = runs with a stop)",
         "boxes per shift", "v2_fig_main.png", (0, 1950))
    bars(pe, ["B6", "B8", "B8-130", "B8-135", "P"], HEAVY,
         "Thermal parameters off by up to ±30% (mean of 20 runs; ✕ = runs with a stop)",
         "boxes per shift", "v2_fig_model_error.png", (0, 1950))
    table(main, ["B2", "B4", "B6", "B7", "B8", "B8-130", "B8-135", "P"], list(SCN), "main",
          "Boxes per 2-hour shift (mean of 20 runs), runs with a protective stop, and mean peak winding temperature. Exact thermal model.\n")
    table(pe, ["B6", "B8", "B8-130", "B8-135", "P", "P-noadapt"], HEAVY, "model_error",
          "Every plant thermal parameter scaled by a factor in [0.7, 1.3] per run; controllers keep datasheet values. 20 runs.\n")
    table(ab + [r for r in main if r["ctrl"] == "P" and r["scenario"] in
                ("warm_heavy", "conveyor_r", "warm_start")],
          ["P", "P-nolook", "P-arm", "P-pace", "P-ik"], ["warm_heavy", "conveyor_r", "warm_start"],
          "ablation", "Planner with one lever at a time (10 runs; full planner 20 runs).\n")
    for v in sorted({r["variant"] for r in se}):
        table([r for r in se if r["variant"] == v], ["B6", "B8", "B8-135", "P"],
              ["warm_heavy", "conveyor_r"], f"sensitivity_{v}", f"Variant {v}, 10 runs.\n")
