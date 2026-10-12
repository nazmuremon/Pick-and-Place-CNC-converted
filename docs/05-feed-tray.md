# 05 – Feed Tray

The feed tray holds the parts for one load. It has **no motors** and there is nothing to build:
it is **5 slot labels** (printed labels 3–7 of the 8), stuck on the T-slot bars of the bed
(decisions D11–D15 and D17). Each slot holds **one cut-tape strip**: the operator peels the cover
film off by hand and lays the strip in the slot's tape mark (decision D14). The 5 strips lie side
by side, about 30 mm apart. The top camera finds every part before picking.

Only 5 slots, because the machine reaches only about 150 × 230 mm of the bed (D17). Labels 1, 2
and 8 are spares.

It replaces the old magazine plates (decision D7, retired on 2026-10-06). Plates A/B, lanes, dowel
pins, magnets, plate ID markers 10–19 and plate corner markers 0–3 are not used any more.

## 1. Principles

1. **The camera decides where a part is.** Loading by hand never needs to be precise. The labels
   don't need exact placement either: the camera measures every label (the bed survey).
2. **The software decides which line is in which slot.** The camera can measure a part's size,
   but it can't read its value: a 10k and a 1k resistor look the same. So the app tells the
   operator what goes in each slot (the load sheet), and the operator follows it.
3. **Never guess.** If a part is unclear (touching another part, upside down, wrong size,
   polarity not clear), the app skips it and says so (R5).
4. **One line per slot.** One value + footprint line in a slot (one strip), never two.
5. **Tape is the normal way** (D14). The tape keeps the parts upright and in a known orientation,
   and one strip holds a whole line. The camera still checks every part.

## 2. The slot label

Each slot is one label, made by `tools/make_labels.py` and printed on the A4 sticker sheet (D13)
or on 18 mm tape (D15, the owner's set). The sketch shows the label as printed (text upright):

```
                TOP side of the label (on this bed: to the RIGHT, D17)
  ┌────────────────────────────────────────────────────────────────────────────┐
  │               ┌                                                       ┐    │
  │  ▓▓▓▓▓▓▓        o   o   o   o   o   o   o   o   o   o   o   o   o   o      │  ← sprocket holes on the top side
  │  ▓ 42  ▓  3 ━   ▪   ▪   ▪   ▪   ▪   ▪   ▪   ▪   ▪   ▪   ▪   ▪   ▪   ▪   ━  │  ← row of parts on the centre marks
  │  ▓▓▓▓▓▓▓                                                                   │
  │               └                                                       ┘    │
  └────────────────────────────────────────────────────────────────────────────┘
   marker  slot  ━ = centre mark      tape mark: 60 × 16 mm, corner ticks,
   ID 42  number   (at both ends)     starts 17 mm from the marker centre
  ←─────────────────────────── 87 mm long, 20 mm high ─────────────────────────→
                BOTTOM side of the label (on this bed: to the LEFT)
```
Sketch only, with a strip in the mark. The real drawing comes from `tools/make_labels.py`: it
writes the sheet (`hardware/print/bed_labels_a4.pdf`) and a picture of it
(`hardware/print/bed_labels_a4_preview.png`). With `--tape --print --strip` it prints the same
labels on the PT-D410 with 18 mm tape (D15): then a slot label is 18 mm high instead of 20 mm.

**On this bed (D17)** the label is turned 90° clockwise (seen from above): the marker end is at
the **back**, the tape mark runs towards the **front**, and the top side of the label (where the
sprocket holes go) is on the **right**.

| Item | Value |
|---|---|
| Marker | ArUco `DICT_4X4_50`. **ID 40–47 = slot 1–8**. In use on this bed: 42–46 = slots 3–7 (D17) |
| Marker size | **10 mm**, like every bed marker, with a white border of at least 2.5 mm. The app reads it from `config/bed.yaml` (`markers.size_mm`) |
| Slot number | Printed next to the marker, for the operator |
| Tape mark (the zone) | White area, **60 × 16 mm** (60 mm along the label, 16 mm across). Corner ticks mark it. It starts 17 mm from the marker centre: in the slot frame it is u = 17 … 77 mm, v = −8 … +8 mm. Parts are searched 1.0 mm inside its edges (`search_margin_mm`), so the ticks are not seen as parts |
| Centre marks | A short black mark just outside each end of the tape mark, on its centre line. The row of parts goes on this line |
| What fits | **8 mm and 12 mm tape.** 16 mm tape does not fit (phase-7 parts). 60 mm holds about **15 parts at 4 mm pitch** (7 at 8 mm pitch) |
| Label size | **87 × 20 mm** |
| Paper | One A4 sheet of **matte** sticker paper, printed with a normal printer at 100 % (D13). Cut the labels out along the thin grey lines. Or 18 mm PT-D410 tape (D15, the owner's set): the label is then 18 mm high |

- The tape mark's position is **fixed relative to the marker**. So a label can be stuck at any
  angle, and the app still knows where its tape mark is. Stick each one straight along its T-slot
  bar anyway, so the strips lie side by side.
- Use **matte** sticker paper, not glossy: glossy paper reflects the LED ring (glare). Still watch
  for glare in the camera image.
- Where the 5 labels go on the bed (D17): one on each T-slot bar, slot 3 on the right, slot 7 on
  the left, markers at the back end in one line with HOME. See
  [03-mechanical.md](03-mechanical.md#2-bed-layout-d17) (`config/bed.yaml` → `layout`,
  `hardware/print/bed_map.png`). Marker centres measured by the bed survey, in mm from HOME
  (+X right, +Y back):

  | Slot | 3 | 4 | 5 | 6 | 7 |
  |---|---|---|---|---|---|
  | ArUco ID | 42 | 43 | 44 | 45 | 46 |
  | Marker centre | (−29.2, −1.5) | (−58.9, +0.8) | (−89.0, +2.7) | (−120.5, +2.5) | (−150.0, +1.7) |

  The bars are 29.5–31.5 mm apart, not exactly 30: that is fine, the survey measures every label.
- The other bed label (HOME) is listed in [06-vision.md](06-vision.md#41-aruco-markers-bed-labels).
  The PCB corner labels are not used (D17).

## 3. Loading rules

- **One line per slot.** Only parts of the value + footprint that the load sheet shows for that slot.
- **Empty slots first.** Before you load a new line into a slot, take out the old strip and every
  leftover part. A leftover 10k in the 1k slot is a wrong part, and the camera can't see that.
- **Lay one cut-tape strip in the slot** (the normal way):
  1. Cut a strip with the quantity from the load sheet (it already includes 2 spares). It must
     fit in the 60 mm mark: about 15 parts at 4 mm pitch, 7 at 8 mm pitch.
  2. Peel the cover film off completely, by hand. Keep the strip flat, so no part falls out of
     its pocket.
  3. Lay the strip in the tape mark: sprocket **holes to the right** (the side the top of the slot
     number points to, D17), the **row of parts on the centre marks**, the strip along the label.
     The parts are not in the middle of the tape, so line up the parts, not the tape edges.
  4. **Hold the strip flat**, for example with a small piece of tape across each end of the strip
     (over an empty pocket, never over a part).
- **Inside the tape mark only.** A part on the edge of the mark or outside it is not used.
- **Polarized parts** (SOD-123, SMA, SOT-23, SOT-23-6, SOT-89): with the holes to the right, the
  parts lie in a known orientation (`angle_in_tape` in `config/feeder.yaml`). Measure it on your
  first strip of each polarized package. The app still checks the polarity of every part from
  the image.
- **Loose parts** (only if you have no tape; not the normal way): put them on the centre line,
  **right side up**, a few mm apart. They must not touch. A part closer than `min_gap_mm`
  (1.0 mm) to another part is "unclear" and is not picked. If a part lands upside down, turn it
  over with tweezers.
- Keep the tape marks clean: no dust, flux or old parts.

## 4. Loads (up to 5 lines each)

The target board has about 70 unique value + footprint lines. See [11-target-board-ht5023.md](11-target-board-ht5023.md).
5 slots can't hold them all, so the job planner splits the job into **loads**:

1. Group the placements by (value, footprint). Each group is one **line**.
2. Sort the lines by nozzle, then by part height (small, low parts first).
3. Fill the 5 slots with the next 5 lines (the number of slots is `feed_tray.slots` in
   `config/bed.yaml`). Lines for the same nozzle stay together, also across loads, so the nozzle
   changes only a few times.
4. Each load is up to 5 lines. The job runs load 1, then pauses so the operator can empty the
   slots and load the next lines, then runs load 2, and so on.

If a slot runs empty but the job still needs that line, the app pauses and asks for a refill.

Example for the HT5023, phase-1 parts:

| Side | Lines | Loads |
|---|---|---|
| F (top) | 8 | **2 loads** (8 ÷ 5 = 1.6) |
| B (bottom) | about 55 (50 in 8 mm tape + 5 in 12 mm tape) | **about 11 loads** (55 ÷ 5 = 11) |

Build step 6.2 checks these numbers against the real job file.

One strip holds a whole line: the largest HT5023 line needs 13 parts (11 + 2 spares), and the
60 mm mark holds about 15 at 4 mm pitch.

This means more loads and more refills than the old plates: with 5 slots (D17), about 13 loads
for both sides of the HT5023, instead of about 8 with 8 slots. That is the price of a feed tray
with nothing to build (D11) on a machine with a small reach (D17).

## 5. Loading workflow in the app

1. The app shows the **load sheet** for the next load: slot, value, footprint, quantity
   (+ 2 spares), LCSC number and drawer.
2. Scan the LCSC bag's QR code (camera or USB scanner, reusing the solder_helper scanner).
   The app shows the slot for that part, or warns if the bag is not in this load.
3. Cut the strip, peel the film off and lay it in the slot (rules in §3). Optional: print part
   labels for the load on the Brother PT-D410 (through `label_server.py`).
4. **Feed-tray scan:** the camera looks at every slot and counts the parts (§6).
5. Before the job starts, the app shows a warning for:
   - a slot that is short (fewer parts than needed),
   - unclear parts (upside down or tilted in the pocket, touching, polarity not clear): fix them
     by hand and scan again, or accept that they are skipped,
   - a part of the wrong size (probably the wrong package in that slot).
   If a slot is empty but the load needs it, the job doesn't start.
6. After the load, the app asks you to empty the slots (take the strips out; leftover parts go
   back into their bags). It checks with the camera that the slots are empty, then shows the
   next load sheet.

## 6. Feed-tray scan (what the camera does)

For each slot used in the current load, the camera:
1. goes to the slot marker (its machine position comes from the bed survey,
   [08-calibration.md](08-calibration.md#bed-survey-phase-4)) and finds the marker again
   (ID and size check),
2. works out the tape mark from the marker (fixed geometry, any label angle),
3. takes **2 views**: one camera view (about 60 × 34 mm) can't hold the marker and the whole
   tape mark (together about 82 × 16 mm). View 1 = the marker and the first part of the strip,
   view 2 = the rest. Both views are placed from the marker found in view 1,
4. searches **along the strip**, `search_margin_mm` inside the edges of the tape mark. The parts
   lie on the centre line, one in each pocket, `pitch_mm` apart. Each part found and `pitch_mm`
   tell where the next pocket should be, but every part is still measured: the camera never
   assumes a part is there. Empty pockets are normal (already picked, or a gap in the tape),
5. for each part: measures its center (x, y) and angle, checks its size against the package
   expected in that slot, checks the gap to other parts, checks that it is right side up, and
   finds the polarity (polarized packages only). The polarity comes from the image, and is
   cross-checked with `angle_in_tape` when it is set: if they disagree, the part is unclear,
6. marks each part **found** or **unclear**. Unclear parts are never picked.

The pipeline is described in [06-vision.md](06-vision.md#43-part-detection-in-a-feed-slot).

The results are kept in memory. After each pick, the camera looks at that pocket again (pick
check): the picked part must be gone. The new image also updates the positions of the parts left,
because a pick can move the strip a little.

## 7. Config files

- [`config/bed.yaml`](../config/bed.yaml): the labels themselves. Bed size, A4 print settings (page,
  check bars), marker IDs and size, the slots in use (`feed_tray.slots`, `slot_numbers`), tape
  mark (zone) size and search margin, PCB area, and the layout in mm from HOME (D17).
- [`config/feeder.example.yaml`](../config/feeder.example.yaml) (copy it to `config/feeder.yaml`):
  - `slots:` what is in each slot (value, footprint, quantity). The planner fills it for every
    load, or you fill it by hand for tests.
  - `detection:` `size_tolerance` (±20 %) and `min_gap_mm` (1.0 mm).
  - `packages:` body size, height, nozzle, tape width (`tape_mm`), pocket pitch (`pitch_mm`),
    polarized or not, the polarity rule, and for polarized parts `angle_in_tape`: the part's 0°
    direction (as in its KiCad footprint) relative to the slot direction, with the holes on the
    top side of the label (to the right on this bed). `null` = not measured yet: measure it on your first strip of each polarized package.
  - `footprint_map:` KiCad footprint → package. Map by body size, not by the `R_`/`C_`/`D_` prefix.
