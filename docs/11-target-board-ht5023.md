# 11 – Target Board: HT5023 Energy Meter v1

Source: `../ht5023_energy_meter_v1.kicad_pcb`. Numbers were taken from
`../solder_helper/board_data.js` (made by `extract_board.py`) on 2026-09-29.
Run the analysis again after changing the PCB.

## Board

| Item | Value |
|---|---|
| Size | 130 × 140 mm (KiCad bounding box x 65.0–195.0, y 20.0–160.0). This is also the largest board the machine can take (R8, D17) |
| Thickness | 1.6 mm, 4 copper layers |
| Fiducials | **None** |
| Mounting holes (non-plated, 3.2 mm, M3) | H1 (69.3, 24.3) · H2 (190.7, 24.3) · H3 (190.7, 155.7) · H4 (69.3, 155.7) |
| Hole spacing | 121.4 mm (X) × 131.4 mm (Y) |
| Rotations used | 0°, 90°, −90°, 180° only |

**Registration:** use the 4 mounting holes (decision D9). They are far apart, which is good for
angle accuracy. The holder uses dowel pins in two diagonal holes (see [03-mechanical.md](03-mechanical.md#5-pcb-holder)).

## SMD placements

"SMD" here means **all pads are SMD** (decision D8). DNP and "exclude from BOM" parts are not
counted. J8 and J9 (`2.54mm_pad`) are bare pads, not parts, so they are left out.

| Side | Placements | Unique value + footprint lines |
|---|---|---|
| F (top) | 31 | 11 |
| B (bottom) | 136 | 65 |
| **Total** | **167** | ~70 (6 lines are used on both sides) |

**KiCad attribute trap:** 28 of these footprints are *not* marked as SMD in KiCad
(for example SW1, U18, L3, U14, U2, U8, X1, U7, D1, F1). A KiCad `.pos` export with
"only SMD footprints" would miss them.

## Package mix and what the machine does with it

| Package | Count | Phase | Nozzle | Tape |
|---|---|---|---|---|
| R 0805 | 46 | 1 | N2 20G | 8 mm |
| C 0805 (incl. `D_0805_2012Metric` caps) | 29 + 9 | 1 | N2 | 8 mm |
| R 0603 | 24 | 1 | N1 22G | 8 mm |
| C 0603 | 7 | 1 | N1 | 8 mm |
| R 1206 | 15 | 1 | N2 | 8 mm |
| R 2512 (R86) | 1 | 1 | N3 16G | 12 mm |
| SOD-123 (D2, D3, D4) | 3 | 1 | N2 | 8 mm |
| SMA / DO-214AC (D1, D5, D7, D13, D14, D17) | 6 | 1 | N3 | 12 mm |
| SOT-23 / SOT-23-3 (D8, Q1, Q2, Q3) | 4 | 1 | N2 | 8 mm |
| SOT-23-6 (U2, U6) | 2 | 1 | N2 | 8 mm |
| SOT-89 (U3) | 1 | 1 | N3 | 12 mm |
| Crystal 3215 (X1) | 1 | 1 | N2 | 8 mm |
| SOIC-8 / SOP-8 / LNK3206D (U14, U7, F1) | 3 | 7 (bottom camera) | N3 | 12 mm |
| SOIC-16 wide (U18, U22) | 2 | 7 | N4 14G | 16 mm |
| Opto SMD-4 (U5, U8, U9, U10, U15, U20, U21) | 7 | 7 | N3 | 16 mm |
| LQFP-100 0.5 mm (U1, HT5023) | 1 | Hand, later 7 | — | tray |
| Tact switch 6 × 6 (SW1, SW3, SW4, SW5) | 4 | Hand | — | — |
| Power inductors (L3 8 × 8, L4 7.1 × 6.6) | 2 | Hand | — | — |

`1N4007_SMD` is assumed to be SMA (M7). **Check the real part.**

The slot tape mark fits **8 mm and 12 mm tape** (D14), so every phase-1 part fits. 16 mm tape
(SOIC-16 wide, optocouplers: phase 7) does not fit. How to feed those is _TBD_ (phase 7).

**Summary:** 148 of 167 placements (89%) are phase-1 parts. 12 need the bottom camera. 7 stay hand-placed for now.

| Side | Phase 1 | Phase 7 | Hand |
|---|---|---|---|
| F | 24 | 2 | 5 |
| B | 124 | 10 | 2 |

## Feed-tray loads (phase 1 parts)

The feed tray has 5 slots on this machine (D17), one value + footprint line per slot, as one
cut-tape strip (D11, D14). So one load = up to 5 lines.
See [05-feed-tray.md](05-feed-tray.md#4-loads-up-to-5-lines-each).

| Side | Lines (8 mm tape + 12 mm tape) | Loads of ≤ 5 lines |
|---|---|---|
| F | 8 (8 + 0) | **2 loads** (8 ÷ 5 = 1.6) |
| B | 55 (50 + 5) | **about 11 loads** (55 ÷ 5 = 11, so 11 is the minimum) |

(With 8 slots, before D17, it was 1 load for F and about 7 for B.)

The planner sorts the lines by nozzle, so most loads use only one nozzle. Step 6.2 of the build
guide checks these numbers against the real job file.

Largest single line: 0.1 µF 0805 on B, 11 pieces → 13 with spares. The 60 mm tape mark holds a
strip of about 15 parts at 4 mm pitch, so every line fits in one slot as one strip.

## Assembly order

This follows the solder_helper passes:
1. **Top SMD** (F): stencil → machine places 24 parts → hand-place 5 (SW1/3/4/5, L3) + U18/U22 until phase 7 → reflow.
2. **Bottom SMD** (B): flip the board. The holder must leave ≥ 12 mm clearance under it for the top parts.
   Stencil → machine places 124 parts → hand-place U1, L4 and the phase-7 parts → reflow.
   *(Whether the top parts stay in place during the second reflow depends on Q7.)*
3. **THT** by hand.

## Data notes
- Several capacitors use `D_0805_2012Metric` (a diode footprint name) with values like `100nF`, `22uF` and `10uF/25V`.
  The package mapping must go by body size / footprint geometry, not by the `R_`/`C_`/`D_` prefix.
- The same value is written in different ways (`100nF` vs `0.1uF`, `10K` vs `10k`, `100k` vs `100K`).
  The planner should normalize values before grouping, or use the LCSC number as the key when it's present.

## Suggested changes for board v2 (to help the machine)
1. Add **3 fiducials per side**: 1 mm copper dot, 2 mm solder mask opening, in an L shape near the corners.
2. Set the footprint type to **SMD** for the 28 footprints listed above.
3. Use `C_` capacitor footprints for capacitors.
4. Keep at least 0.5 mm between parts, so the nozzle doesn't touch neighbours.
5. Use one spelling for each value (for example always `100nF`).
