# 08 – Calibration

> **v2 (2026-10-12, Phase C):** the camera steps (lens, tilt, camera height, steps/mm check) are one ChArUco calibration in the page (Setup tab, step 3, D28); results go to the calibration store with their inputs, not `config/machine.yaml`. See [07-software.md](07-software.md#software-v2-srcpnp). The steps below stay valid for the bench tools.

Do the steps **in this order**. Each step uses results from the steps before it.
Results go into `config/machine.yaml`. The calibration tools write them there; don't edit them
by hand unless a step says so.

Redo everything from step 5 onward if you move the top camera, change the lens height or change the head.

## 1. Mechanics check
**Tools:** dial indicator, square, ruler.
1. Measure backlash on X, Y and Z (see [03-mechanical.md](03-mechanical.md#1-repair-and-inspection-phase-0)).
2. Measure X–Y squareness (diagonals of a large square).

**Pass:** backlash ≤ 0.1 mm, and the diagonals differ by ≤ 0.3 mm over 150 mm.
**Stored:** `mechanics.backlash_mm` (for information only).

## 2. Steps per mm
1. Work out the value: `steps/rev × microsteps ÷ mm per rev`.
2. Set `$100–$102` (X, Y, Z; the nozzle servo is not a grblHAL axis, D18).
3. Move 100 mm and measure with a ruler (camera check in step 5).

**Pass:** error ≤ 0.1 mm over 100 mm.
**Stored:** grblHAL settings → `firmware/grblhal-settings.txt`.

## 3. Speed and acceleration
1. Raise `$110–$112` and `$120–$122` until motors stall or the head shakes.
2. Then use **60%** of those values.

**Pass:** 50 moves back and forth over the full travel, then back to a mark with no visible error.
**Stored:** grblHAL settings.

## 4. Top camera: focus, light and lens
1. Set the lens height for ~30 px/mm. Focus sharply on the board surface height.
2. Turn off auto exposure, auto white balance and auto focus. Set the values so nothing is washed out
   (check the histogram in the UI).
3. **Lens distortion:** print a ChArUco board (`tools/make_markers.py` makes the PDF) on a normal
   laser printer, on matte paper, at 100 %. Put it on the bed, take
   15–20 images at different positions. Then run `cv2.calibrateCamera` (or the ChArUco version).

**Pass:** reprojection error < 0.5 px.
**Stored:** `top_camera.controls`, `top_camera.matrix`, `top_camera.dist`.

**The camera must look straight down.** A tilted camera sees the square markers as trapezoids, and the
pixel-to-mm matrix (step 5) gets skewed axes, so the calibration refuses it. Check it after every change
to the head: **on the bench**, `tools/camera_square_check.py` (live window, nothing moves) measures the
marker nearest the picture centre. **Pass:** sides within 3 %, corners 90 ± 2° (`config/bench.yaml`
`camera_square`). The edge nearest to the lens is drawn red: raise the camera on that side. The same
window shows a focus number (higher = sharper) for the lens ring.
On 2026-10-07, after the nozzle was mounted, slot 3 had sides 11 % apart and corners 85–94.5° (morning:
1.6 % and 88.7–89.7°), and the calibration was refused (X–Y angle 86.8°).

**If the camera can't be mounted straight** (bench, 2026-10-07: ~25° off vertical), `tools/camera_rectify.py`
straightens its pictures instead:
1. A rough calibration on the slot label nearest the picture centre.
2. The head moves over a grid (`config/bench.yaml` `camera_rectify`), only to the front and towards the
   middle slots: the reach is not trusted after the camera moved.
3. A pinhole camera with lens bending (k1, k2; principal point at the picture centre) is fitted to every label
   corner, with each label's place, size and angle. A label that doesn't fit (seen at the bent edge) is left
   out.
4. Z goes up 3 mm and back: the lens height and the **height shift** g. A thing h mm higher than the labels,
   seen in the middle of the picture, is g·h mm away from where a label at that spot would be. The nozzle
   offset must allow for it (tape, part tops and the board are at other heights than the labels).
5. **Pass:** in the straightened picture every label is square (sides within 2 %, corners 90 ± 1.5°) and all
   are the same size. Only then is `config/camera_rectify.yaml` written. The bench tools then straighten every
   frame, and the normal calibration (step 5) works as with a straight camera.

The measurements are kept in `logs/camera_rectify/<time>/`, and `--refit` fits them again without the machine.

## 5. Pixel-to-mm matrix M
1. Put a small sharp mark (a slot marker works; HOME is just outside the reach, D17) under the
   camera and find it in the image.
2. Jog +5 mm X, find the mark. Go back. Jog +5 mm Y, find the mark. Repeat at −5 mm. Longer moves
   give a better M, as long as the mark stays in view. (The bench tool uses only 3 mm moves; that
   was one reason its HOME check was 0.2 mm off, see step 6.)
3. Solve M (2 × 2) from the pixel shifts. It holds the scale, the camera rotation and the mirroring.
   On the bench (2026-10-06): ≈ 7.3 px/mm, the picture turned ≈ 3.8°, no mirroring (the raw
   picture is already a view from the front).

**Pass:** checked at 3 other positions, the predicted mark position is within 0.03 mm.
**Stored:** `top_camera.M`, `top_camera.px_per_mm` (for information).

## 6. HOME mark and camera homing
1. Print the HOME label (ID 49) with `tools/make_labels.py` (A4 sheet at 100 %, D13, or 18 mm tape,
   D15) and stick it flat on the bed (see [03-mechanical.md](03-mechanical.md#8-home-mark)). On
   this bed it is a 15 mm square (text cut off), on the next T-slot bar right of slot 3 (D17).
   Its marker is 10 mm; the app reads the size from `config/bed.yaml` (`markers.size_mm`).
2. **Off-centre homing:** HOME is ~4 mm outside the camera-centre reach, so the camera can't sit
   over it. Jog until HOME is in view, as close as the reach allows (always the same spot). The
   app finds HOME's centre (with the 3% size check), works out its offset (dx, dy) from the image
   centre with M, and sets the origin from it: `G10 L20 P1 X<−dx> Y<−dy>`. Now X0 Y0 = HOME's centre.
   The method: [06-vision.md](06-vision.md#46-bed-survey-and-off-centre-home).
3. Test: drive to a random position, home again, and record the correction. Do this 10 times.

**Pass:** spread ≤ ±0.05 mm (R7).
**Stored:** `homing.marker_id`, `homing.max_drift_mm`.

**First bench result (2026-10-06):** after the automatic survey, HOME measured again was
0.20 mm (X) / 0.12 mm (Y) off. Above R7, but mostly measurement error (no lens calibration, M
from 3 mm moves, HOME 15 mm off-centre at the start and 4 mm at the end). This step (build steps
3.5 and 3.6) is the real check.

### Reach (no limit switches)
There are no limit switches, so the app needs to know how far the head can go. The head hit the
right end once on the bench (2026-10-06; slow, no damage). Measure the reach **before** the bed
survey, and again whenever HOME moves:
1. Run `tools/bed_test_run.py --measure-reach`. This turns the reach guard **off**: go slowly,
   with a hand on the power switch.
2. Jog until HOME is in view and press `h`.
3. Jog slowly to each end (left, right, front, back), and stop about 5 mm before it.
4. Quit with `q`. The tool saves the reach (camera centre, mm from HOME) to `config/bench.yaml`
   → `limits.reach_from_home`. From then on, the bench tools refuse every move outside it.

**Now (2026-10-06):** measured from the old HOME: X −94 … +56, Y −160 … +70 (about 150 × 230 mm).
From the new HOME (D17) it is an **estimate**: X −154 … −4, Y −227 … +3. Measure it.
**Stored:** `config/bench.yaml` → `limits.reach_from_home` (bench). Later the app's `soft_limits`
in `config/machine.yaml`.

### Bed survey (phase 4)
Do this once all bed labels are stuck (build step 4.1), and again if any label moves.
**On the bench now:** `tools/bed_test_run.py --auto` (start with a slot label or HOME in view).
The app's version comes in build step 4.2 (`feeder.py`), with the same method
([06-vision.md](06-vision.md#46-bed-survey-and-off-centre-home)).
1. First make sure the two 100 mm check bars on the A4 sheet (or the 100 mm check label on 18 mm
   tape, D15) measured 100.0 mm (±0.5 mm) with a ruler (build step 4.1). If not, the markers are
   scaled too: print again before you go on.
2. Home (step 6). Then the camera visits every label in use: slots 3–7 (IDs 42–46, D17). There
   are no PCB corner labels. It starts at the layout position from `config/bed.yaml` (already in
   mm from HOME) and searches around it, only inside the reach.
3. For each marker: check the ID and the size (3%), centre on it as far as the reach allows, and
   store its machine position and angle.
4. Back near HOME: measure HOME again (drift check).
5. The app works out the 5 tape marks (slot zones) from the markers, takes the PCB area from
   `config/bed.yaml` (the board itself is found later by its holes, D9), and draws them on the
   live image.

**Pass:** all labels found (slots 3–7). The HOME check at the end is within R7 (±0.05 mm), once
the camera is calibrated (steps 4–6). Every part row and the whole PCB area can be reached by the
nozzle (with the camera-to-nozzle offset, step 7), and every label and board hole can be seen by
the camera, inside the reach.
**Stored:** the machine positions of all labels, in `config/machine.yaml` (written by the tool).
The bench tool writes `logs/test_run/<time>/survey.yaml` instead.

**First bench run (2026-10-06, `logs/test_run/20261006-163410/`):** all 5 slots found within
3 mm of the plan (measured positions in [03-mechanical.md](03-mechanical.md#2-bed-layout-d17));
HOME check 0.20 / 0.12 mm (see step 6).

## 7. Camera-to-nozzle offset
1. Put a piece of paper with a thin layer of soft putty (or an ink pad) on the bed.
2. Lower the nozzle until it leaves a ring mark, then lift it.
3. Move the camera over the mark and find the center of the ring.
4. Offset = nozzle position − camera position. Repeat 3 times at different places and average.

**Pass:** the three results agree within 0.05 mm. The offset must be small enough that the nozzle
reaches every part row and the whole PCB area inside the reach (D17,
[03-mechanical.md](03-mechanical.md#3-head)). If not, stop and tell the owner.
**Stored:** `head.nozzle_offset_xy` (nozzle position = camera position + offset).

**On the bench:** `tools/nozzle_calibrate.py`, key `o`, does this at several servo positions (D18).
If the needle is not exactly on the servo shaft, its tip goes round a small circle as the servo turns.
The tool fits that circle: its centre is the shaft, its radius is the runout. It keeps the offset for each
pulse, so the app knows where the tip is at every angle. Before that, set the spots and touch heights with
keys `p`, `l`, `t` and `k` (see the tool's help). Results go to `config/nozzle_calibration.yaml`.

## 8. Z heights
Use a spring nozzle and a feeler gauge (or a sheet of paper, ~0.1 mm).
1. `board_top_z`: lower the nozzle onto a bare board in the holder until the paper just drags.
2. `pick_z` for each package: the same on the top of a part in its cut-tape strip, laid in a slot
   as for a job (film off, strip held flat), then subtract 0.3 mm so the spring compresses a
   little. The part sits on the bottom of its pocket, so paper tape and plastic tape can give
   different heights: measure in the tape you really use. A loose part lies a little lower.
3. `safe_z`: the tallest part already on the board + 2 mm (for the B side, the top parts hang *under* the board and don't count).

**Stored:** `z.board_top`, `z.safe`, `feeder.yaml → packages.*.pick_z_offset`.

**On the bench (no Z probe, no limit switch):** `tools/nozzle_calibrate.py` measures Z with the camera.
The camera's scale (px per mm of the HOME label) depends on its height: 1 / scale grows in a straight
line with the height (pinhole camera).
- Keys `t` / `k` keep a touch height together with the HOME label's scale of that session.
- Key `z` (Z sweep): Z steps 0 to −4 mm and back over HOME. It gives the straight line, how far Z is from
  it (lost or sticking steps), the play when Z turns, and the repeatability. After a sweep, a Z that slid
  while the power was off is measured at the start of each session (about ±0.05 mm), and the touch heights
  are corrected. Without a sweep, a changed scale only stops every move down.
- Key `b` (Z steps/mm): a block of known height (3–6 mm, from calipers) with a spare label on top, next
  to a slot label. It measures the block label, then the slot label at Z0 and one block height lower. Equal
  scales mean 1 commanded mm = 1 real mm. It is repeated 3 times; the spread shows the real precision.
  The tool only suggests the new `$100`; it never changes it.

**Heights set in software (bench, `tools/nozzle_calibrate.py`):** jog Z with `[` `]` (0.1 mm) and `{` `}` (1 mm), then:
- `k` pick height: the nozzle tip just touches a part in its slot (the pick goes `press_mm` lower).
- `d` place height: best with a part held (`a` = pump on): jog down until the part just touches the board.
- `m` moving height: parts are carried at this Z. It must be at least `nozzle.move_clearance_mm` above the
  pick and place heights.
- `t` touch on the paper (for the offset marks).

Every height is stored in mm from **Z0 = the remembered camera height** (D19). At every start (`h`) the tool measures
the pick slot's label at the picture centre and drives Z back to that height (up by itself up to
`z_cal.max_drift_mm`, down only up to `z_cal.restore_max_down_mm`). Only then are the heights and offsets used. `0` twice
makes the height of now the new reference (then set everything again). The real pick and place (`r`, one part per press)
refuses to start until `k`, `d`, `m` and both `n` offsets are set, and the moving height and Z0 are
`nozzle.move_clearance_mm` (4.5 mm: the tallest part 2.5 mm + 2 mm) above the pick and place heights.

**Order on the bench:** measure the reach again (`tools/bed_test_run.py --measure-reach`) → start
`tools/nozzle_calibrate.py` → `h` → the first time only: jog Z until the nozzle is clear of the board, the holder and
the strips (at least 4.5 mm above everything it will touch), then `0` twice (this camera height becomes Z0) →
`k` (on a part) → `a` + `d` (a part held, on the board) → `m` → `t` (paper, for `o`) → `n` twice on a part → `n` twice
on a board hole → `r` (one part per press). Next days: `h` brings the camera back to Z0, and everything set before
is used again.

Every program that opens the Grbl port marks the machine "not parked" (`logs/nozzle_state.yaml`); only a clean quit
with the nozzle up marks it "parked" again. Not parked (a program was stopped, or `gcode_console` was used): the
nozzle tool starts LOCKED, so lift Z by hand (`}`), then press `h`. The same after a Grbl reset or an ALARM (Grbl's
position is checked after every move). The remembered camera height is refused (never converted) when the Z
steps/mm (`$100` on the bench), the needle (`nozzle.od_mm`), `config/camera_rectify.yaml` or `z_cal.label` change:
`0` twice and set everything again.

**Nozzle offset by eye (`n` twice):** with the tilted camera the offset depends on the height of what the camera
looks at. So measure it on a part in the tape (PICK offset) and on a board hole (PLACE offset). The offset job
(`o`, marks in paper) is the precise way; `n` is good to about 0.3 mm.

### Nozzle servo: pulse → angle (D18)
The MG995 servo has no feedback, so the app needs a table: which pulse gives which angle. Do this
after steps 4–8 (it needs a calibrated camera, the nozzle offset and `pick_z`).
1. **Pulse range:** turn the servo slowly towards each end (`M3 S<us>` in small steps), and stop before it hits its end stop.
2. **Table:** pick a part with a clear long side, turn the servo to a pulse, place the part on
   double-sided tape, and measure its angle with the top camera. Do this at many pulses over the
   whole range.
3. **Settle time:** repeat step 2 with shorter and shorter waits after the turn. `settle_s` = the
   shortest wait that still gives the same angle, plus a margin.

**Pass:** the table predicts the measured angle within R1 (±1°). The app refuses a turn that
doesn't fit in the measured range.
**Stored:** `servo.pulse_range_us`, `servo.pulse_to_angle`, `servo.range_deg`, `servo.settle_s`.

**On the bench:** `tools/nozzle_calibrate.py`, key `v`, does step 2. It picks each part at the centre pulse,
turns the servo, places the part on the tape and measures the turn. The part angle comes from the second
moments of the part's dark top, not from its outline: a 1206 top is only about 16 × 10 pixels on the
Xitech camera, and the outline gave errors of up to 4°. The moments give about 0.5° (tested with simulated
pictures). The real accuracy shows in the fit: `worst_deg` in `config/nozzle_calibration.yaml`.

## 9. Bottom camera (phase 7)
1. Lock focus with the nozzle tip in focus at `bottom_camera_z`. Lock exposure.
2. Find the nozzle tip center in the image → `bottom_camera.center_xy` (in machine coordinates when the nozzle is there).
3. Pixel-to-mm matrix: move the nozzle ±1 mm while it's over the camera and track the tip (like step 5).
4. **Nozzle runout:** turn the nozzle servo to several angles over its whole range and measure the
   tip position. Store the circle (only an arc: the servo turns 270°, not a full circle, D18).
   The app corrects placement by the runout at the place angle.

**Pass:** runout after correction ≤ 0.03 mm.
**Stored:** `bottom_camera.*`, `head.runout`.

## 10. Squareness correction (optional)
Print a precise grid (or use the ChArUco board). Measure 9 points across the bed with the camera,
and fit an affine correction (skew and axis scale).
**Stored:** `mechanics.skew_correction`.

## 11. Final check (after any calibration change)
- Camera homing 5 times: spread ≤ 0.05 mm.
- Tape test: place 20 × 0805 on double-sided tape and measure them with the camera. **Pass: R1** (±0.1 mm, ±1°).
