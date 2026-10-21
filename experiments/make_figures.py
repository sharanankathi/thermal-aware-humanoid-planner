"""Paper figures from results/*.csv and traces. Usage: python experiments/make_figures.py

Colors: validated categorical palette (dataviz skill reference). One fixed slot per
controller across every figure; identity is also carried by direct labels / line style.
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
sys.path.insert(0, str(ROOT))
RES, FIG = ROOT / "results", ROOT / "figures"
FIG.mkdir(exist_ok=True)

# validated in display order (B1..B5, P): adjacent CVD dE >= 9.1, normal-vision >= 19.6
COLOR = {"P": "#2a78d6", "B1": "#eb6834", "B2": "#1baf7a", "B3": "#eda100", "B4": "#e87ba4",
         "B5": "#008300"}
LABEL = {"P": "Proposed", "B1": "B1 dominant arm", "B2": "B2 round-robin",
         "B3": "B3 energy-optimal", "B4": "B4 reactive derating",
         "B5": "B5 round-robin + energy IK",
         "P-noadapt": "Proposed, no online correction",
         "P-arm": "arm/two-arm only", "P-ik": "posture only", "P-pace": "pacing only",
         "P-nomargin": "no safety margin", "P-margin10": "10 min margin"}
STYLE = {"P": "-", "B1": (0, (1, 1.5)), "B2": "--", "B3": (0, (5, 1.5, 1, 1.5)), "B4": ":",
         "B5": (0, (6, 2))}
ORDER = ["B1", "B2", "B3", "B4", "B5", "P"]
SCN_LABEL = {"light_std": "Light boxes\n(sanity)", "heavy_high": "Heavy boxes,\nhigh shelf",
             "warm_heavy": "Heavy, 35 °C\nroom", "mixed": "Mixed\npayloads",
             "conveyor_r": "Heavy, right-side\nconveyor", "warm_start": "Heavy, right arm\nstarts warm"}
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"

plt.rcParams.update({
    "font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
    "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.axisbelow": True,
    "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb",
})


def load(name):
    with open(RES / f"{name}.csv") as f:
        return list(csv.DictReader(f))


def agg(rows, key="boxes"):
    d = defaultdict(list)
    for r in rows:
        d[(r["scenario"], r["ctrl"])].append(float(r[key]))
    return d


# ---------------------------------------------------------------- Fig 1: traces
def fig_traces(scn="warm_heavy"):
    from sim.robot import JOINT_NAMES
    traces = pickle.load(open(RES / "traces_main.pkl", "rb"))
    fig, ax = plt.subplots(figsize=(7.0, 3.4))
    for c in ["B2", "B4", "P"]:
        t, T = traces[(c, scn)]
        hot = int(np.argmax(T.max(0)))
        ax.plot(t / 60, T[:, hot], color=COLOR[c], ls=STYLE[c], lw=2)
        stopped = t[-1] < 7200 - 30
        y_end = T[-1, hot]
        txt = LABEL[c]
        if stopped:
            ax.plot(t[-1] / 60, y_end, "x", color=INK, ms=8, mew=2)
            txt += f": protective stop at {t[-1] / 60:.0f} min"
        if stopped:
            ax.annotate(txt, (t[-1] / 60, y_end), xytext=(8, 5), textcoords="offset points",
                        fontsize=8, color=INK, va="bottom", ha="left")
        else:
            ax.annotate(txt, (t[-1] / 60, y_end), xytext=(4, 0), textcoords="offset points",
                        fontsize=8, color=INK, va="center", ha="left")
    ax.axhline(140, color=INK, lw=1, ls=(0, (4, 3)))
    ax.text(1, 141.5, "140 °C winding limit", fontsize=8, color=MUTED)
    ax.set_xlim(0, 125)
    ax.set_ylim(top=152)
    ax.set_xlabel("time into shift (min)")
    ax.set_ylabel("hottest of 14 windings (°C)")
    ax.set_title("8 kg boxes to the high shelf in a 35 °C room", loc="left", fontsize=10,
                 color=INK)
    fig.tight_layout()
    fig.savefig(FIG / "fig1_traces_warm_heavy.png", dpi=200)
    plt.close(fig)


# ---------------------------------------------------------------- Fig 2: boxes per shift
def fig_boxes(rows, out="fig2_boxes.png", scns=None, title="Boxes completed in a 2-hour shift",
              key="boxes", ylabel="boxes per shift (mean ± SD, 20 seeds)", limit=None,
              stops=False):
    d = agg(rows, key)
    d_stop = agg(rows, "stopped")
    scns = scns or [s for s in SCN_LABEL if any(k[0] == s for k in d)]
    fig, ax = plt.subplots(figsize=(7.4, 3.6))
    ctrls = [c for c in ORDER if any(k[1] == c for k in d)]
    width = 0.8 / len(ctrls) - 0.01
    x = np.arange(len(scns))
    for i, c in enumerate(ctrls):
        means = [np.mean(d[(s, c)]) if d.get((s, c)) else np.nan for s in scns]
        sds = [np.std(d[(s, c)]) if d.get((s, c)) else 0 for s in scns]
        pos = x + (i - (len(ctrls) - 1) / 2) * (width + 0.01)
        ax.bar(pos, means, width, color=COLOR[c], label=LABEL[c],
               yerr=None if stops else sds,
               error_kw=dict(ecolor=MUTED, lw=0.8, capsize=1.5), linewidth=0)
        if stops:
            for xp, s_, mval in zip(pos, scns, means):
                n_st = int(sum(d_stop.get((s_, c), [])))
                ax.text(xp, mval + 15, f"{n_st}/{len(d_stop.get((s_, c), []))}", ha="center",
                        va="bottom", fontsize=7.5, color=INK)
    ax.set_xticks(x, [SCN_LABEL[s] for s in scns], fontsize=8)
    ax.set_ylabel(ylabel)
    if limit:
        ax.axhline(limit, color=INK, lw=1, ls=(0, (4, 3)))
        ax.text(x[-1] + 0.45, limit + 1, f"{limit:g} °C limit", fontsize=7.5, color=MUTED,
                ha="right", va="bottom")
        ax.set_ylim(20, limit + 12)
    ax.set_title(title, loc="left", fontsize=10, color=INK)
    ax.legend(fontsize=7.5, ncol=3, frameon=False, loc="upper center",
              bbox_to_anchor=(0.5, -0.2))
    ax.grid(axis="x", visible=False)
    fig.tight_layout()
    fig.savefig(FIG / out, dpi=200)
    plt.close(fig)


# ---------------------------------------------------------------- tables
def table(rows, name):
    d_b, d_s = agg(rows), agg(rows, "stopped")
    d_T, d_c = agg(rows, "peak_T"), agg(rows, "cycle_s")
    d_t = defaultdict(list)
    for r in rows:
        if r["t_stop_min"] not in ("", "nan"):
            d_t[(r["scenario"], r["ctrl"])].append(float(r["t_stop_min"]))
    scns = list(dict.fromkeys(r["scenario"] for r in rows))
    ctrls = [c for c in ORDER if any(r["ctrl"] == c for r in rows)] + \
        sorted({r["ctrl"] for r in rows} - set(ORDER))
    lines = [f"# {name}\n", "| Scenario | Controller | Runs | Boxes (mean ± SD) | Stops | "
             "Median stop (min) | Peak winding (°C) | Cycle (s) |", "|" + "---|" * 8]
    for s in scns:
        for c in ctrls:
            if (s, c) not in d_b:
                continue
            b = d_b[(s, c)]
            stp = d_t.get((s, c))
            med = f"{np.median(stp):.0f}" if stp else "-"
            lines.append(f"| {s} | {LABEL.get(c, c)} | {len(b)} | {np.mean(b):.0f} ± {np.std(b):.0f} | "
                         f"{int(sum(d_s[(s, c)]))} | {med} | ")
            lines[-1] += f"{np.mean(d_T[(s, c)]):.0f} | {np.mean(d_c[(s, c)]):.2f} |"
    (RES / f"table_{name}.md").write_text("\n".join(lines) + "\n")
    return "\n".join(lines)


if __name__ == "__main__":
    main = load("main")
    fig_traces()
    fig_boxes(main)
    fig_boxes(main, "fig2b_peak_temp.png", title="Hottest winding reached during the shift",
              key="peak_T", ylabel="peak winding (°C, mean ± SD, 20 seeds)", limit=140)
    print(table(main, "main"))
    if (RES / "paramerr.csv").exists():
        pe = load("paramerr") + (load("paramerr_p") if (RES / "paramerr_p.csv").exists() else [])
        fig_boxes([r for r in pe if r["ctrl"] != "P-noadapt"], "fig3_param_error.png",
                  title="Every thermal parameter off by up to ±30% (label: runs that stopped)",
                  ylabel="boxes per shift (mean of 20 runs)", stops=True)
        print(table(pe, "paramerr"))
    if (RES / "ablation.csv").exists():
        ab = [r for r in load("ablation") if r["ctrl"] != "P-pace" or
              not (RES / "ablation_pace.csv").exists()]
        if (RES / "ablation_pace.csv").exists():
            ab += load("ablation_pace")
        print(table(ab + [r for r in main if r["ctrl"] == "P" and r["scenario"] in
                          {x["scenario"] for x in ab}], "ablation"))
