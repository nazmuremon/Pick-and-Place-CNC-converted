# 06 – Vision

> **v2 (2026-10-12, Phase C):** `src/pnp/vision/` replaces the bench vision with one camera model (D28), parts by an operator-confirmed template (D29), registration on holes and corners (D30) and the stall check from the model (D31). How to use it: [07-software.md](07-software.md#software-v2-srcpnp). This page stays the background for the bench tools.

## 1. Cameras and lighting

| | Top camera | Bottom camera |
|---|---|---|
| Hardware | USB (UVC), 1080p, **manual focus** (e.g. ELP) | Raspberry Pi Camera Module 3 (CSI) |
| Mount | On the head, looking down, as close to the nozzle as possible (D17: small reach, see [03-mechanical.md](03-mechanical.md#3-head)) | Fixed on the bed, looking up. Position _TBD_ (no room inside the reach, D17) |
| Used for | HOME mark, bed survey (labels), feed-tray scan, board holes/fiducials, pick check, dry run | Nozzle tip position, part offset/angle on the nozzle (phase 7) |
| Control | `v4l2-ctl`: turn off auto exposure, auto white balance and auto focus, then set fixed values | `picamera2`: `AfMode=Manual`, fixed `LensPosition`, `ExposureTime`, `AnalogueGain` |
| Light | LED ring + diffuser | LED ring + diffuser |

**Lighting matters more than the algorithm.** Rules:
- Put a shroud around the work area. Room light and sunlight change during the day.
- Use a diffuser, so the metal ends of parts don't make bright spots.
- **Lock** exposure, gain, white balance and focus. Save the values in `machine.yaml`.

## 2. Resolution

Target **≥ 25 px/mm**. A 1080p camera covering about 60 × 34 mm gives ~32 px/mm.

| Part | Short side | Pixels across at 32 px/mm |
|---|---|---|
| 0603 | 0.8 mm | 26 |
| 0805 | 1.25 mm | 40 |
| SOT-23 body | 1.3 mm | 42 |
| 3.2 mm mounting hole | 3.2 mm | 102 |

**Bench (2026-10-06):** the Xitech camera on the head gives only ≈ 7.3 px/mm at its height
(7.26 along X, 7.40 along Y; `logs/test_run/20261006-163410/survey.yaml`). Good enough to find
the bed labels, far too low for parts (step 0.3).

A slot marker plus its whole tape mark is about 82 × 16 mm (10 mm marker, tape mark 60 × 16 mm
starting 17 mm from the marker centre, D14). A view of about 60 × 34 mm can't hold both, so the
feed-tray scan takes **2 views per slot**: the marker and the first part of the strip, then the
rest. If the view is smaller, it needs more images per slot.

## 3. Capture rules (every image)

1. Wait until grblHAL reports `Idle`.
2. Wait ~150 ms more for vibration to stop.
3. Use a frame captured **after** step 2. USB cameras return old buffered frames, and OpenCV keeps
   4 buffers, so dropping 1–2 frames is not enough. Keep a grab thread running, or flush the whole buffer.
4. Remove lens distortion with the saved calibration (`cv2.undistort` or a remap table made once).
5. Convert pixels to machine mm:

```
machine_xy = camera_xy + M · (pixel − image_center)
```
- `camera_xy` = the machine position of the camera center (nozzle position + camera offset).
- `M` = 2 × 2 matrix from calibration. It holds the scale, the camera rotation and any mirroring.
  See [08-calibration.md](08-calibration.md#5-pixel-to-mm-matrix-m).
- On the bench the raw picture is already a **view from the front**: u to the right = +X, v down =
  to the front (−Y). It is turned ≈ 3.8° to the machine axes. `M` handles that angle: don't turn
  the image by hand.

## 4. Detectors

### 4.1 ArUco markers (bed labels)
All bed markers are labels from `tools/make_labels.py` (decision D12), printed on one A4 sheet of
matte sticker paper with a normal printer at 100 % (decision D13).

| IDs (`DICT_4X4_50`) | What | Where |
|---|---|---|
| 49 | HOME mark | Bed label, text cut off (15 mm square). Just outside the camera reach: measured off-centre (§4.6) |
| 30–33 | PCB area corners 1–4 (1 front-left, 2 front-right, 3 back-right, 4 back-left) | Printed, but **not used** since D17 (out of reach) |
| 40–47 | Feed slots 1–8 | Bed labels. In use: 42–46 = slots 3–7 (D17) |
| 20–23 | `smd_identify` demo sheet | Paper, bench only |
| 0–3, 10–19 | Retired (old magazine plates) | Not used on the bed |

The A4 sheet also has two **100 mm check bars** (one across, one down). Measure both with a ruler:
each must be 100.0 mm (±0.5 mm). If not, the printer scaled the page: print again at 100 %
("Actual size", no "fit to page"). On 18 mm tape (D15) the **100 mm check label** does the same job
for the tape feed (the direction across the tape is fixed by the print head). The ChArUco lens-calibration board is a separate sheet
(`tools/make_markers.py`, build step 3.3), printed on a laser printer, on matte paper.

- **Marker size: 10 mm** for every bed marker (6 cells), with a white border of at least 2.5 mm.
  Read it from `config/bed.yaml` (`markers.size_mm`). Never put a fixed size in code.
- Use `cv2.aruco.ArucoDetector`, dictionary `DICT_4X4_50`, with sub-pixel corner refinement.
- The marker center is the mean of its 4 corners. The angle comes from the top edge.
- Check the marker's size in pixels against the size from `config/bed.yaml`. If it's more than 3%
  off, reject it (wrong height, a scaled or stretched print, or a bad detection).
- **Glare:** glossy paper reflects the LED ring and can hide marker cells, so use **matte**
  sticker paper. Use the diffuser too, and check the image during the bed survey.

### 4.2 Board mounting holes (board registration, target board)
- Search a window of ±2 mm around each expected hole (from the holder position).
- Find the hole with `cv2.HoughCircles`, or with contours plus a circularity test.
  Expect a 3.2 mm diameter, ±10%.
- Use all 4 holes. Fit the board transform (see [07-software.md](07-software.md#board-registration)).
  **Stop** if any hole is more than 0.15 mm away from the fit.

### 4.3 Part detection in a feed slot
Each slot holds one cut-tape strip, cover film removed, laid in the slot's tape mark: holes to the
back, the row of parts on the centre line (D14). Loose parts are still possible, but not the
normal way. See [05-feed-tray.md](05-feed-tray.md). For each slot:
1. Find the slot marker (ID 40–47) and check its size (§4.1). Work out the tape mark (the zone)
   from the marker: its position is fixed relative to the marker, so the label can sit at any angle.
2. Take the **2 views** that cover the tape mark (§2). Place the second one from the marker found
   in the first.
3. Search the tape mark, `search_margin_mm` inside its edges (`config/bed.yaml`). An outline that
   crosses the search edge is `unclear`; parts outside are ignored. Search **along the strip**:
   the parts lie on the centre line, one in each pocket, `pitch_mm` apart (`config/feeder.yaml`).
   Use each part found and `pitch_mm` to predict where the next pocket is, but measure every
   part: never assume a part is there. An empty pocket is normal.
4. Grayscale → light blur → threshold. Otsu at first; switch to adaptive if the tape (pocket
   walls, sprocket holes, tape edges) gets in the way.
5. `findContours` → `minAreaRect` → center, width, height, angle of each outline.
6. **Size filter:** width and height must be within ±`size_tolerance` (20 %) of `body_mm` of the
   package expected in this slot (either way round). This rejects:
   - dust,
   - two parts touching (one outline that is too big),
   - a part standing on its edge or tilted in its pocket,
   - the edge of the tape, a pocket or a sprocket hole,
   - a part of the wrong package (the app warns: probably the wrong line in the slot).
   The size check can't tell values apart (10k and 1k look the same). The load sheet must be right.
7. **Minimum gap:** a part closer than `min_gap_mm` (1.0 mm) to any other outline is `unclear`.
   The nozzle could pick two parts or push the neighbour.
8. **Upside down:** a part that lies upside down is `unclear` (for example a chip resistor that
   shows its white back, or a SOT-23 with its legs up).
9. **Polarity** (only packages marked `polarized: true`): find the full 0–360° angle with the rule
   for that package, from the image alone. The strip can lie at any angle (like its label), and
   loose parts can lie at **any** angle, so the rule must work over the full 360°. Check every
   part, also in tape. Then **cross-check** with `angle_in_tape` (`config/feeder.yaml`, measured
   on the first strip with the holes on the top side of the label, to the right on this bed) when it is set: the angle from the image,
   relative to the slot direction, must agree with it. If they disagree (for example by 180°,
   because the strip lies the wrong way round, with the holes to the left), the part is `unclear`.

| Package | Polarity rule |
|---|---|
| SOD-123, SMA | Cathode band: the darker or lighter band at one end. Compare the brightness of the two ends |
| SOT-23 | 1 lead on one side, 2 leads on the other: count the leads on each long side |
| SOT-23-6 | Pin-1 dot or bevel. The mark is small: if it can't be seen clearly → `unclear` |
| SOT-89 | The wide tab (lead 2) is on one side |
| SOIC / SOP (phase 7) | Pin-1 dot or chamfered edge |

10. The result for each part: `found` / `unclear`, with (x, y, angle, confidence). A slot with no
   part is `empty`. **Unclear parts are never picked.** The app skips them and flags them.

The settings are in `config/feeder.yaml` (`detection:`, `packages:`) and `config/bed.yaml` (`feed_tray:`).

### 4.4 Pick check
After a pick, look at that pocket again: the picked part should be gone. If it is still there,
the pick failed. Retry (up to 3 times), then move on to the next part in the slot. Use the new
image for the next pick, because a pick can move the strip a little.

### 4.5 Bottom camera (phase 7)
1. Move the nozzle with the part over the bottom camera, at `bottom_camera_z`.
2. Use a strong backlight or the ring light. Threshold, then take the biggest outline, then `minAreaRect`.
   For ICs, fit the lead rows.
3. The result is the part center's offset from the nozzle axis (dx, dy) and the angle error dθ.
4. Correct the place position and angle before placing.

### 4.6 Bed survey and off-centre HOME
`tools/bed_test_run.py` (bench, 2026-10-06) finds every bed label this way. The app's bed survey
(build step 4.2) should use the same method:
1. **Calibrate** on a marker in view: move a small step to the left, then one to the front, and
   see how far the marker moves in the picture. This gives the 2 × 2 matrix (scale, angle,
   mirroring). Left and front, because HOME sits near the right and back ends of the reach. The
   bench uses 3 mm moves; longer moves give a better matrix (step 3.5).
2. **Go** to the planned position of each label (`config/bed.yaml` → `layout`, nearest label
   first), as far as the reach allows. If the marker is not in view, look at a few spots around
   it (±15 and ±30 mm in Y, ±20 mm in X), only inside the reach.
3. **Centre** on the marker: move by the offset seen in the picture (at most 15 mm per move), up
   to 6 times, until it is within 0.05 mm of the image centre, or as close as the reach allows.
4. **Measure** the rest in the picture (average of 6 fresh frames): marker position = camera
   position + offset in the picture. So a marker just outside the reach is still measured.
5. **Stall check:** after a move, every marker seen before and after must move at least half as
   far as the matrix predicts. If not, the head is stuck at an end of its travel or lost steps:
   stop. The position is lost, so home again.
6. **HOME check:** at the end, go back as close to HOME as the reach allows and measure HOME again.
   The difference is the drift.

**HOME is measured off-centre.** HOME is ~4 mm right of the camera-centre reach (D17), so the
camera never sits right over it. Homing measures its offset from the image centre with the matrix
and sets the origin from it. The error grows with the offset and with any error in the matrix
(lens distortion, short calibration moves). So: undistort first (step 3.5), calibrate with longer
moves, and always home from the same spot, as close to HOME as the reach allows.

First automatic bench run (2026-10-06, `logs/test_run/20261006-163410/`): all 5 slot markers
found within 3 mm of the plan. HOME, measured again after the trip, was 0.20 mm (X) / 0.12 mm (Y)
off. That is above R7 (±0.05 mm), but mostly measurement error: the matrix came from 3 mm moves
only, and HOME was 15 mm off-centre at the start and 4 mm at the end. Steps 3.5 and 3.6 do the
real check.

## 5. Debugging and test data

- Save every image the app uses, with the result drawn on it, to `logs/vision/<date>/`.
  This is how you tune thresholds.
- Collect a **regression set**: about 20 images per package, labelled correctly, in `tests/vision_data/`.
  Every change to a detector must still pass that set.
- The UI shows the live camera with the detected outlines drawn on top.

## 6. Performance

With OpenCV on the Pi 5, detection in one full frame takes < 100 ms, and in one slot zone less.
The machine moves much slower than that, so a neural network is **not needed**.
If it's ever needed later, a Raspberry Pi AI HAT+ (Hailo) can run YOLO.
