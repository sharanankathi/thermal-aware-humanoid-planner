# Gear-ratio study tables (generated)

## Table 1. Boxes per 8 h shift (3 seeds)

| Option | 25 °C | 35 °C | 40 °C |
|---|---|---|---|
| Current gearing, air | 5,941 | 5,459 | 5,192 |
| Current gearing + loop + 2 kg radiator | 9,011 | 8,370 | 8,024 |
| Re-geared, air (gear eff. 80%) | 9,734 | 9,734 | 9,721 |
| Re-geared, air (gear eff. 70%) | 9,030 | 8,326 | 7,946 |
| Re-geared + loop + 2 kg radiator (70%) | 9,734 | 9,734 | 9,734 |

## Table 2. Fixed vs two-speed gearing, 2 h shift (5 seeds; peak winding in brackets)

| Room | Gear eff. | Current | Best fixed | Two-speed 1.5x | Two-speed 2x | Two-speed 3x |
|---|---|---|---|---|---|---|
| 25 °C | 80% | 1,639 (133 °C) | 2,668 (107 °C) | 2,668 (107 °C) | 2,668 (68 °C) | 2,551 (131 °C) |
| 25 °C | 70% | 1,639 (133 °C) | 2,470 (131 °C) | 2,470 (131 °C) | 2,668 (84 °C) | 2,475 (132 °C) |
| 35 °C | 80% | 1,506 (133 °C) | 2,668 (123 °C) | 2,668 (123 °C) | 2,668 (81 °C) | 2,310 (132 °C) |
| 35 °C | 70% | 1,506 (133 °C) | 2,281 (131 °C) | 2,281 (131 °C) | 2,668 (97 °C) | 2,234 (132 °C) |

## Table 3. Throughput vs ratio multiplier, 2 h shift (5 seeds; efficiency applies above 1x)

| Room | Gear eff. | 0.75x | 1.0x | 1.25x | 1.5x | 1.75x | 2.0x |
|---|---|---|---|---|---|---|---|
| 25 °C | 80% | 993 | 1,639 | 2,295 | 2,668 | 2,668 | 2,668 |
| 25 °C | 70% | 993 | 1,639 | 1,876 | 2,470 | 2,470 | 2,470 |
| 35 °C | 80% | 889 | 1,506 | 2,115 | 2,668 | 2,668 | 2,668 |
| 35 °C | 70% | 889 | 1,506 | 1,726 | 2,281 | 2,281 | 2,281 |

## Table 4. Gear ratios and joint-side reflected rotor inertia (kg·m²)

| Joint | Ratio now | Speed headroom | Best fixed ratio | Apollo reflected inertia | TBM at current ratio | at best fixed | at 2x |
|---|---|---|---|---|---|---|---|
| shoulder_aa | 70 | 1.47x | 100 | 0.0983 | 0.0691 | 0.1413 | 0.2764 |
| shoulder_ie | 60 | 1.92x | 113 | 0.0823 | 0.0508 | 0.1794 | 0.203 |
| shoulder_fe | 100 | 1.54x | 150 | 0.1128 | 0.141 | 0.3173 | 0.564 |
| elbow_fe | 100 | 1.16x | 113 | 0.1026 | 0.141 | 0.18 | 0.564 |
| wrist_roll | 10 | 15.15x | 20 | 0.0368 | 0.0014 | 0.0056 | 0.0056 |
| wrist_yaw | 30 | 3.96x | 60 | 0.0033 | 0.0127 | 0.0508 | 0.0508 |
| wrist_pitch | 30 | 6.64x | 60 | 0.0029 | 0.0127 | 0.0508 | 0.0508 |
