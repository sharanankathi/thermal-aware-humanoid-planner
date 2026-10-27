"""Heat-path diagram for the project report. Usage: python experiments/make_report_diagram.py"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

ROOT = Path(__file__).resolve().parents[1]
INK, MUTED, BG, BLUE = "#111418", "#52514e", "#fcfcfb", "#2a78d6"
boxes = [
    ("Copper winding", "heat is made here\n(gear ratio sets how much)", "#111418", "white"),
    ("Winding → housing", "inside the motor\nTHE BOTTLENECK\n(potting speeds it up)", "white", INK),
    ("Housing → coolant", "liquid cold plate\nor air", "white", INK),
    ("Coolant → world", "radiator or\nswappable cartridge", "white", INK),
]
fig, ax = plt.subplots(figsize=(7.6, 2.3))
fig.patch.set_facecolor(BG)
ax.set_xlim(0, 100); ax.set_ylim(0, 30); ax.axis("off")
w, h, gap = 20.5, 22, 5.5
for i, (title, sub, fc, tc) in enumerate(boxes):
    x = 1 + i * (w + gap)
    edge = BLUE if i == 1 else ("#111418" if i == 0 else "#c9ced4")
    ax.add_patch(FancyBboxPatch((x, 4), w, h, boxstyle="round,pad=0.4,rounding_size=1.6",
                                fc=fc, ec=edge, lw=2.2 if i == 1 else 1.0))
    ax.text(x + w / 2, 4 + h - 5, title, ha="center", va="center", fontsize=9.5, fontweight="bold",
            color=BLUE if i == 1 else tc)
    ax.text(x + w / 2, 4 + h / 2 - 3, sub, ha="center", va="center", fontsize=7.6,
            color="#dde3ea" if i == 0 else MUTED, linespacing=1.35)
    if i < 3:
        ax.add_patch(FancyArrowPatch((x + w + 0.8, 15), (x + w + gap - 0.8, 15), arrowstyle="-|>",
                                     mutation_scale=12, color=MUTED, lw=1.4))
fig.tight_layout(pad=0.2)
fig.savefig(ROOT / "figures/report_heatpath.png", dpi=220)
