# 03 – Mechanical

## 1. Repair and inspection (phase 0)

First find and fix what is broken. Then measure and write the results in this table.

| Check | How | Result |
|---|---|---|
| What is broken | — | _TBD_ |
| Bed size (mm) | Ruler | 240 left-right (X) × 440 front-back (Y) (owner, measured again 2026-10-06, D16. The first 320 × 232 was wrong) |
| X / Y / Z travel (mm) | Move by hand, measure with a ruler | Camera-centre reach, measured on the bench 2026-10-06 (jogged to every end, stopped ~5 mm before it): only about **150 (X) × 230 (Y)**, D17. From HOME: about X −154 … −4, Y −227 … +3 (estimate until `tools/bed_test_run.py --measure-reach`). The full travel of each axis and Z: _TBD_ |
| Drive type per axis | Lead screw (pitch?) / ball screw / belt | _TBD_ |
| Rails move smoothly | Push by hand along the full length, no tight spots | _TBD_ |
| Backlash per axis | Dial indicator: move +1 mm, then −1 mm, and read the difference | _TBD_ |
| X–Y squareness | Draw or scratch a large square, measure both diagonals | _TBD_ |
| Bed flatness | Dial indicator or feeler gauge across the bed | _TBD_ |

**Limits:**
- Backlash up to 0.1 mm is fine, because the camera corrects position and all final moves can come from the same side.
- Diagonals of a 150 mm square should differ by less than 0.3 mm. The rest is corrected in software.

## 2. Bed layout (D17)

The bed is **240 × 440 mm**: 240 mm left-right (X) × 440 mm front-back (Y) (owner, measured
again 2026-10-06, D16). But the machine reaches only part of it: the **camera centre reaches
only about 150 × 230 mm** (measured 2026-10-06 on the bench, see §1 and
[01-requirements.md](01-requirements.md#work-area-needed)). So the layout of D17 puts everything
inside that reach. The owner chose it after the first bed test run.

The labels are the same as before (the owner's set: PT-D410, 18 mm tape, 2026-10-06, D15; the
A4 sheet, D13, also works). Only 5 slot labels and the HOME label are used. They are stuck on the **T-slot bars** of the bed: the
bars run front-back, about 30 mm apart.

**Coordinates:** mm from the **centre of the HOME marker**, +X right, +Y back (away from the
operator). These are the same directions as the machine axes, and the reach was measured from
HOME too. The numbers are in `config/bed.yaml` → `layout` (a plan; the bed survey measures the
real positions). `tools/make_labels.py` draws the layout in `hardware/print/bed_map.png`, with
the T-slot bars and the measured reach.

Drawing roughly to scale, seen from above (1 character ≈ 3 mm in X, 1 line ≈ 10 mm in Y):
```
                    BACK (away from you)
    ┌┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┐        ← Y ≈ +3 (end of the reach)
    ┆▣         ▣          ▣         ▣         ▣       ┆ ⊕      ← Y ≈ 0
    ┆7         6          5         4         3       ┆HOME
    ┆░         ░          ░         ░         ░       ┆        ← Y ≈ −17 … −77 (tape marks)
    ┆░         ░          ░         ░         ░       ┆
    ┆░         ░          ░         ░         ░       ┆
    ┆░         ░          ░         ░         ░       ┆
    ┆░         ░          ░         ░         ░       ┆
    ┆░         ░          ░         ░         ░       ┆
    ┆  ┌───────────────────────────────────────────┐  ┆        ← Y = −84 (~3 mm gap)
    ┆  │                                           │  ┆
    ┆  │                                           │  ┆
    ┆  │                                           │  ┆
    ┆  │        PCB area = the HT5023 board        │  ┆
    ┆  │                                           │  ┆
    ┆  │           130 × 140 mm (X × Y)            │  ┆
    ┆  │                                           │  ┆
    ┆  │         (−144, −224) … (−14, −84)         │  ┆
    ┆  │                                           │  ┆
    ┆  │          PCB holder inside (§5)           │  ┆
    ┆  │                                           │  ┆
    ┆  │                                           │  ┆
    ┆  │                                           │  ┆
    ┆  └───────────────────────────────────────────┘  ┆        ← Y = −224
    └┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┘        ← Y ≈ −227 (end of the reach)
X ≈ −154                                          X ≈ −4
                    FRONT (you)
```
`▣ n` = slot label n: marker at the back end, then the number, then the tape mark (`░`) towards
the front. Each slot label lies on its own T-slot bar. `⊕` = HOME label. The dashed box =
where the **camera centre** can go (estimate from HOME, `config/bench.yaml` →
`limits.reach_from_home`). HOME is just outside it.

| Item | Position (mm from HOME) | Notes |
|---|---|---|
| HOME label (ID 49) | (0, 0) | On the next T-slot bar right of slot 3 (the old slot-2 bar), in line with the slot markers. Its text is cut off: a 15 mm square label (marker only). It is ~4 mm right of the camera-centre reach, so homing measures it in the picture (§8). Never move it |
| Slot labels 3–7 (IDs 42–46) | Marker centres, measured by the bed survey (2026-10-06): slot 3 (−29.2, −1.5), slot 4 (−58.9, +0.8), slot 5 (−89.0, +2.7), slot 6 (−120.5, +2.5), slot 7 (−150.0, +1.7) | One label on each T-slot bar, slot 3 on the right, slot 7 on the left. Each runs front-back: marker at the **back** end, tape mark towards the front (17 … 77 mm in front of the marker centre). Tape holes to the **right**. The bars are 29.5–31.5 mm apart (not exactly 30) |
| Slot labels 1, 2, 8 (IDs 40, 41, 47) | — | Spares |
| PCB area | (−144, −224) to (−14, −84) | **The HT5023 board itself, 130 × 140 mm** (X × Y), just in front of the slot labels: only **~3 mm** between the board's back edge and the front ends of the slot labels. The PCB holder (§5) holds the board here. No corner labels: the board is found by its 4 mounting holes (D9) |
| PCB corner labels 1–4 (IDs 30–33) | — | Not used (they were out of reach). Spares |
| Camera-centre reach | X −154 … −4, Y −227 … +3 | Estimate, worked out from the reach measured at the old HOME. Measure it with `tools/bed_test_run.py --measure-reach` ([08-calibration.md](08-calibration.md#reach-no-limit-switches)) |
| Bottom camera (phase 7) | _TBD_ | There is no room inside the reach |
| Nozzle rest (spare nozzles) | _TBD_ | Somewhere outside the labels and the PCB area |

Rules:
- **Reach rule:** the camera must **see** every label and the 4 board holes. The nozzle must
  **reach** every part row (the tape marks) and the whole PCB area. The nozzle reaches the same
  box as the camera, **shifted by the camera-to-nozzle offset**. That is why the nozzle must sit
  very close to the camera (§3). The numbers are in
  [01-requirements.md](01-requirements.md#work-area-needed).
- The layout is final only when the full travel is known (Q1) and the camera-to-nozzle offset is
  measured. If it doesn't fit, propose a new layout in build step 0.4 and wait for the owner's OK.
- **There are no limit switches.** The head hit the right end of its travel once on the bench
  (2026-10-06; slow, no damage). The bench tools now refuse every move outside the reach in
  `config/bench.yaml`.
  The reach is measured from HOME: if HOME moves, measure the reach again.
- Put the HOME mark where it never gets moved, away from where parts fall.
- The labels don't need exact placement: the camera measures every label (bed survey,
  [08-calibration.md](08-calibration.md#bed-survey-phase-4)). But once surveyed, don't move them.
  If a label moves, run the bed survey again.
- The PCB holder is pinned in place (dowel pins), so it always sits in the same spot.
- History: the layout of D14 was made for a wrong bed size (320 × 232 mm). The layout of D16
  (slots at the front-left, PCB area at the back) needed about 405–440 mm of travel in Y. The
  owner's 7-slot layout (same day) had PCB corner labels out of reach. D17 replaces them all.

## 3. Head

Take the spindle off the Z carriage and fit a light mounting plate (3D-printed PETG or 3 mm aluminium). Mount these on it:

| Item | Notes |
|---|---|
| Nozzle servo (**MG995** hobby servo, D18) | Turns the nozzle. No hollow shaft: the nozzle sits on the servo horn, **centred on the output axis**. About 55 g. The 270° version (owner, 2026-10-07); the real range is measured with the camera ( [08-calibration.md](08-calibration.md#nozzle-servo-pulse--angle-d18)) |
| Nozzle holder with spring | 1–2 mm of travel, so a small Z error doesn't crush parts or leave them in the air |
| Top camera + LED ring + diffuser | Looks straight down. **As close to the nozzle as possible** (see the design note below; record the real offset). Put the lens at a height that gives ~30 px/mm (see vision doc) |
| Short soft silicone tube (2–4 mm inner diameter) | From the valve to the nozzle on the servo horn, as a **loop** that lets the nozzle turn (it twists up to the servo's 270°). Or a small rotary union |

- **Weight:** keep the head light (goal < 300 g). The CNC's Z axis is strong enough, but a lighter head allows more acceleration.
- **Z travel per cycle:** only 5–10 mm. Set the safe height just above the tallest part already on the board.
- **Cable chain:** USB camera cable (high-flex), servo (3 wires: brown = GND, red = +V, orange = signal), LED ring (2 wires), vacuum tube.
- **Servo limits (D18):** a few tenths of a degree dead band and some jitter, and about 55 g more on the Z axis.
  The servo body is big, and the nozzle sits on its output axis: check in step 2.1 that the camera
  can still sit close enough to the nozzle (design note below).

**Design note: keep the camera-to-nozzle offset small (D17, risk K9).** The machine reaches only
about 150 × 230 mm. The nozzle can go only where the camera centre can go, **shifted by the
offset**. The nozzle must reach the part rows (slot 7's parts at X ≈ −150) and the right edge of
the board (X −14): that is 136 mm of the 150 mm in X. In Y it must reach from the first pocket
(Y ≈ −15) to the front edge of the board (Y −224): 209 mm of the 230 mm. With the estimated reach
(§2), this gives a rough limit (estimate; work it out again after `--measure-reach`):

| Nozzle position, compared with the camera | Room (about) |
|---|---|
| Nozzle to the right of the camera | ≤ 4 mm |
| Nozzle to the left of the camera | ≤ 10 mm |
| Nozzle behind the camera (to the back) | ≤ 3 mm |
| Nozzle in front of the camera | ≤ 18 mm |

So the camera may sit at most about 10 mm right of and 18 mm behind the nozzle (about 20 mm
on the diagonal), and almost nothing in the other directions. The first plan (camera ~35 mm
from the nozzle) **does not fit**. Ideas for step 2.1 (owner to decide): a small camera board
right next to the nozzle; the LED ring around both; first find out if the travel can be larger
(Q1). If the offset must be larger, the layout must change (fewer slots, or only part of each
tape mark used): ask the owner first.

## 4. Nozzles

Use blunt luer-lock dispensing needles (stainless). Cut the tip square and deburr it. A nozzle's
**outside diameter must be smaller than the part's short side**, so the vacuum seals on the flat top.

| Nozzle | Needle | Outside diameter | Parts on the HT5023 board | Count |
|---|---|---|---|---|
| N1 | 22G | 0.72 mm | 0603 R/C | 31 |
| N2 | 20G | 0.91 mm | 0805 R/C, 1206, SOT-23/-6, SOD-123, 3215 crystal | 109 |
| N3 | 16G | 1.65 mm | SMA, SOT-89, 2512 (later SOIC-8, SOP-8, optocouplers) | 18 |
| N4 | 14G | 2.11 mm | SOIC-16 wide (later) | 2 |

These gauges are a starting point. Tune them in phase 2. Changing nozzles is manual: the app
places all parts for one nozzle, then pauses and asks for the next one.

## 5. PCB holder

- The board sits on **4 supports under the mounting holes**. **2 dowel pins (3.0 mm)** go into
  two diagonal holes: H1 (69.3, 24.3) and H3 (190.7, 155.7).
- When the board is flipped (B side up), the same pins go through H2 and H4 instead. The four
  holes form a rectangle, so the board still fits.
- **Clearance under the board: ≥ 12 mm.** When the bottom side is placed, the top-side SMD parts
  are already soldered and hang underneath (tact switches, 8 × 8 mm inductor).
- Keep the board's top surface at a known height. It goes into `machine.yaml` as `board_top_z`.
- Use the same holder for stencil printing if possible, so paste and parts line up the same way.
- The holder holds the board at the PCB area of §2: (−144, −224) to (−14, −84) from HOME, for the
  HT5023 (130 × 140 mm, the largest board on this machine, R8). There are no corner labels.
- **Only ~3 mm** lie between the back edge of the board and the front ends of the slot labels.
  The holder must not cover the slot labels, the tape marks or the 4 mounting holes (the camera
  must see the holes from above, D9). If it can't fit, ask the owner before you move the PCB
  area: it must stay inside the reach (§2).

## 6. Feed tray

- The feed tray is the **5 slot labels (3–7) stuck flat on the T-slot bars** (D11, D17). There is
  nothing to build: no plates, no dowel pins, no magnets. Details are in [05-feed-tray.md](05-feed-tray.md).
- Each slot holds **one cut-tape strip**, cover film peeled off by hand, laid in the slot's tape
  mark (D14). Each label lies along its bar (front-back), marker at the back, so the strips lie
  side by side, about 30 mm apart (§2).
- Clean the bed before you stick the labels. Press them flat, with no bubbles or lifted edges,
  so a strip or a part can't slide under an edge.
- Keep the slot area clean: no dust, flux, solder balls or old parts on the tape marks.

## 7. Bottom camera mount

- Position: _TBD_. There is no room for it inside the reach of the D17 layout (§2).
- It looks straight up and has an LED ring.
- Mount it so the underside of a part held on the nozzle is in focus at `bottom_camera_z` (set in calibration).
- Protect the lens from falling parts. Tilt it slightly or add a clear cover you can clean.

## 8. HOME mark

- A label from `tools/make_labels.py` (D12), printed on the A4 sticker sheet (D13) or on 18 mm
  tape (D15; the owner's set is on tape): ArUco marker ID 49.
  The full label is about 44 × 15 mm (marker + text). On this bed its text is **cut off**: it is a
  15 mm square label (marker only).
- The marker is 10 mm, like every bed marker. The app reads the size from `config/bed.yaml`
  (`markers.size_mm`).
- It sits on the next T-slot bar right of slot 3, in line with the slot markers (§2). It is the
  origin (0, 0) of the layout and of the machine.
- It is ~4 mm right of the camera-centre reach, so the camera can't centre on it. Homing
  **measures** it in the picture instead: the camera stops as close as the reach allows, and the
  app works out where HOME's centre is from the image (off-centre homing,
  [08-calibration.md](08-calibration.md#6-home-mark-and-camera-homing)).
- Once placed, **never move it**. All saved positions, and the reach, are measured from it. If it
  ever moves, measure the reach again (`tools/bed_test_run.py --measure-reach`), home again, and
  redo the bed survey and every saved position.
