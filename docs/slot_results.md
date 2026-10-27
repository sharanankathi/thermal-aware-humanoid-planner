# Stator-slot thermal model: winding-to-housing resistance and potting (Oct 2026)

**Why.** Every thermal result in this repo uses a winding-to-housing resistance `R_wh` equal to 35% of
the motor datasheet's total thermal resistance (TPR 3.55 K/W for the TBM(S)-6013-A). That split was
a provisional guess. The planned bench test (docs/bench_potting_plan.md) could not be run, so this
estimates `R_wh` from physics instead, and asks what potting the winding would do.

## Model (thermal/slot_model.py)
1. **Winding bundle conductivity** across the wires: enamelled copper wires in a matrix of air
   (bare), impregnating varnish (0.20 W/m·K, typical production) or filled potting epoxy
   (0.70 W/m·K). Closed-form (coated cylinder, then Maxwell–Garnett) checked against a fine-grid
   finite-difference model of square and hexagonal wire arrays: agreement within 1–8%, the formula
   on the conservative side at high fill.
2. **2D finite-difference conduction model of one slot pitch**: homogenized bundle with uniform heat
   generation, slot liner, teeth, back iron; stator outer surface at housing temperature. Checked
   against the exact solution for a heated annulus: 0.07% error at the 0.05 mm grid used.
3. **End turns** (41% of the copper here) as a three-node network: axial conduction along the wires
   into the slot copper, plus a direct path to the housing (still internal air, or epoxy if potted);
   a contact resistance between stator and housing.

**Geometry.** From the datasheet: outer diameter 59.06 mm, stack 12.70 mm, 12 poles. Assumed and
swept: slot count (18 nominal; 12–24), bore, yoke and tooth widths, copper fill, enamel build, slot
liner, stator-housing contact, end-turn air and epoxy paths. Copper mass comes out at 48 g of the
221 g motor, a plausible share.

## Results (results/slot/slot.csv, figures/slot_fig1.png)
| Winding | Nominal R_wh | Median of 150 samples | 5–95% band | Share of TPR (median) |
|---|---|---|---|---|
| Bare (air between wires) | 2.83 K/W | 2.72 | 1.26–5.77 | 77% |
| Varnished (likely production) | 0.83 K/W | 0.89 | 0.55–1.59 | 25% |
| Potted with epoxy | 0.45 K/W | 0.50 | 0.36–0.71 | 14% |
| *Assumed in all simulations* | *1.24 K/W* | | | *35%* |

- **A bare winding is inconsistent with the datasheet**: it would leave too little of the measured
  3.55 K/W for the housing-to-air path through a 3.5-inch heat sink. The data fit a varnished winding.
- **The assumed 35% split is inside the uncertainty band but on the high side**; physics suggests
  about 25%. The motor's interior is likely a somewhat smaller bottleneck than the simulations assumed.
- **Potting cuts R_wh by 44% relative to varnish** (5–95%: 31–59%). It clears the 25% kill threshold
  set for the bench test in every sample: potting is a real lever.
- **Slot count dominates the uncertainty** (12 slots: 1.30 K/W varnished; 18: 0.83; 24: 0.59).
  A teardown photo or a manufacturer cross-section would remove most of the band.

## What it means for the earlier results
`experiments/run_split.py` reruns the key comparisons at the assumed 35% split, the physics 25% split,
and the potted 14% split (results/split/split.csv; 108 eight-hour shifts, 2 seeds, no protective stops).
Boxes per 8 h shift, heavy boxes, 40 °C room (25 and 35 °C rank the same way):

| Option | Assumed 35% | Physics 25% | Potted 14% |
|---|---|---|---|
| Air only | 5,191 | 5,223 | 5,728 |
| Coolant loop, no sink | 6,904 | 7,413 | 8,584 |
| Loop + 2 kg radiator | 8,027 | 8,088 | 8,813 |
| Loop + PCM re-frozen at dock | 8,030 | 8,088 | 8,813 |
| Loop + PCM swapped, 15 s trip | 8,024 | 8,077 | 8,783 |
| Re-geared, air only | 9,714 | 9,734 | 9,734 |

- **Every conclusion survives the corrected split.** The ranking is unchanged in all three rooms:
  swapping never beats the radiator, and re-gearing stays at or near the no-limit ceiling.
- **At the physics split, the sinkless loop gains most (+7%)**: with less resistance inside the motor,
  the cold plates do more. The radiator moves under 1%.
- **Potting adds 9–10% on top of air or a radiator** and lets the sinkless loop reach 99% of the
  ceiling at 25 °C (9,675 of 9,734).

## Limitations
- An estimate from assumed internal geometry, not a measurement; the band reflects only the swept inputs.
- Steady conduction only; no convection inside the motor beyond a fixed end-turn coefficient.
- Varnish assumed to fill the bundle completely; real impregnation leaves voids (raises R_wh).
- Uniform heat generation (no skin or proximity effects; fine at these speeds).

## Reproduce
```bash
pytest -q tests/test_slot.py                 # FD vs exact annulus; formula vs wire-array FD
python experiments/make_figures_slot.py      # ~30 s: results/slot/slot.csv, figures/slot_fig1.png
python experiments/run_split.py              # ~108 shifts: results/split/split.csv
```
