# Cartridge study tables (generated)

## Table 1. Boxes per 8 h shift by cooling option (5 seeds)

| Option | 25 °C | 35 °C | 40 °C |
|---|---|---|---|
| Air only (v2 robot) | 5,940 | 5,458 | 5,193 |
| Coolant loop, no sink | 7,867 | 7,225 | 6,909 |
| Loop + 2 kg radiator | 9,017 | 8,368 | 8,022 |
| Loop + 2 kg PCM, refrozen at charge dock | 8,999 | 8,362 | 8,027 |
| Loop + 2 kg PCM, swapped (best policy, 150 s trip) | 8,892 | 8,254 | 7,925 |
| Best swap policy / melt point | smart, 32 °C | smart, 44 °C | smart, 44 °C |

## Table 2. Swapping vs a radiator as trip time shrinks (3 seeds)

| Room | Radiator 15 W/K/kg | Radiator 5 W/K/kg | Best swap, 15 s trip | Best swap, 60 s | Best swap, 150 s |
|---|---|---|---|---|---|
| 25 °C | 9,017 | 9,013 | 9,001 | 8,934 | 8,745 |
| 35 °C | 8,366 | 8,347 | 8,365 | 8,292 | 8,154 |
| 40 °C | 8,023 | 8,007 | 8,028 | 7,944 | 7,808 |

## Table 3. Joint heat path vs sink (3 seeds)

| Room | Joint path | Loop, no sink | 2 kg radiator | Weak radiator | Best swap | % of no-limit |
|---|---|---|---|---|---|---|
| 25 °C | Nominal joint (1.74 K/W) | 7,865 | 9,017 | 9,013 | 8,992 | 93% |
| 25 °C | Better cold plate (1.34 K/W) | 8,341 | 9,017 | 9,017 | 8,986 | 93% |
| 25 °C | Cold plate + potted winding (0.72 K/W) | 9,734 | 9,734 | 9,734 | 9,734 | 100% |
| 40 °C | Nominal joint (1.74 K/W) | 6,908 | 8,023 | 8,007 | 8,023 | 82% |
| 40 °C | Better cold plate (1.34 K/W) | 7,310 | 8,044 | 8,044 | 8,009 | 83% |
| 40 °C | Cold plate + potted winding (0.72 K/W) | 9,221 | 9,229 | 9,229 | 9,229 | 95% |

## Table 4. PCM model validation

Neumann check: max relative front error 0.16%.

| Cartridge | t90 finite volume (s) | t90 lumped (s) | Max melt-fraction error |
|---|---|---|---|
| EG composite, 6 mm gap | 350 | 360 | 0.007 |
| Pure paraffin, 12 mm gap | 1250 | 1270 | 0.068 |
