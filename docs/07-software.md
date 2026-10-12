# 07 – Software

## Stack

| Layer | Choice |
|---|---|
| OS | Raspberry Pi OS Bookworm, 64-bit |
| Language | Python 3.11 |
| Libraries | `opencv-python` (includes the ArUco module), `numpy`, `picamera2` (from apt), `pyserial`, `PyYAML`, `fastapi` + `uvicorn`, `pytest` |
| Motion firmware | grblHAL for RP2040, **3 axes** (X, Y, Z), built with the grblHAL Web Builder. Keep the build settings in `firmware/` |
| Nozzle servo | MG995 servo on the spindle PWM output of the motion controller: no second board (D18). The angle goes through G-code: `M3 S<us>` = pulse width in microseconds. Bench: our Grbl 1.1h servo build ([../firmware/README.md](../firmware/README.md)). grblHAL: spindle PWM at 50 Hz, planned (to be checked when there is a Pico) |
| Job export | KiCad's Python (`pcbnew`), run on the PC that has KiCad, like `../solder_helper/extract_board.py` |

## Planned source layout

```
pick_and_place_robot/
├── config/
│   ├── machine.yaml        # calibration results, offsets, Z heights, soft limits, speeds, bed survey
│   ├── bed.yaml            # bed size, print settings, marker IDs + size, PCB area, slots in use, layout (mm from HOME)
│   ├── bench.yaml          # test bench only (exists): axis map, feed/step caps, reach from HOME
│   └── feeder.yaml         # what is in each slot (per load), detection, packages, footprint map
├── tools/
│   ├── export_job.py       # KiCad → jobs/<board>_<side>.json (pcbnew)
│   ├── make_labels.py      # bed labels (D12, D13, D15) → hardware/print/, and bed_map.png (exists)
│   └── bench.py, bed_test_run.py, …   # test bench tools (exist, see "Bench tools" below)
├── src/pnp/
│   ├── motion.py           # grblHAL serial: send/ok, status poll, wait-idle, jog, set origin
│   ├── safety.py           # homed flag, soft limits, safe-Z rule, feed caps
│   ├── head.py             # pump, valve, nozzle servo: makes G-code only, never uses GPIO (D3, D18)
│   ├── lights.py           # LED rings (Pi GPIO PWM)
│   ├── camera.py           # top (USB/V4L2) and bottom (picamera2), locked settings, fresh frames
│   ├── geometry.py         # pixel→mm, board fit, frames, angles
│   ├── vision/
│   │   ├── markers.py      # ArUco: HOME mark, slot labels
│   │   ├── holes.py        # board mounting holes / fiducials
│   │   ├── parts.py        # part detector + polarity rules
│   │   └── bottom.py       # part-on-nozzle alignment (phase 7)
│   ├── calibrate.py        # calibration steps (writes machine.yaml)
│   ├── feeder.py           # bed survey, slot zones, feed-tray scan (along the strip, 2 views per slot), inventory
│   ├── planner.py          # loads of ≤ 5 lines (one per slot), nozzle groups, placement order, load sheet
│   ├── runner.py           # pick/place state machine, retries, logging
│   └── sim.py              # fake grblHAL (servo, pump, valve outputs) + fake cameras, for work without hardware
├── ui/                     # FastAPI app + static web page
├── tests/                  # pytest + tests/vision_data/ (labelled images)
├── jobs/                   # exported job files
└── logs/                   # jobs/*.jsonl, vision/<date>/*.jpg
```

## Coordinate frames

| Frame | Origin and axes | Units |
|---|---|---|
| **Machine** | Origin = center of the HOME mark. +X right, +Y away from the operator, +Z up. Z0 = safe height | mm (the nozzle angle, in degrees, comes from the servo: `M3 S<us>` through the calibration table, D18) |
| **Camera pixels** | Top-left of the image. u to the right, v down | px |
| **Board (KiCad)** | KiCad internal coordinates: mm, **Y down**. Angles counter-clockwise as seen on screen | mm, degrees |
| **View** | The board as the top camera sees it, with Y up (see below) | mm, degrees |
| **Layout** | Center of the HOME marker, +X right, +Y away from the operator: the same as the machine X/Y (D17). The planned label positions and the PCB area (`config/bed.yaml` → `layout`). A plan only: the bed survey measures the real positions. (Before D17 this was a bed-corner frame) | mm |
| **Slot** | Center of the slot marker. +X = from the marker's corner 0 to corner 1 (along the label, towards the tape mark), +Y = from corner 0 to corner 3. The tape mark (zone) has a fixed position in this frame: X = 17 … 77, Y = −8 … +8 (`slot_zone_mm()` in `tools/make_labels.py`). The parts lie on Y = 0. The sprocket holes are on the −Y side (the top side of the label). With the label stuck as in `bed_map.png` (D17: marker at the back, the label running to the front), slot +X points to the front (machine −Y) and slot +Y to the left (machine −X), so the holes are on the right | mm |

- Set grblHAL's direction-invert setting (`$3`) so the machine axes point as in the table.
  Then everything is right-handed as seen from above.
- **Layout → machine:** the layout is already in mm from HOME, so a layout position is the first
  guess for the camera centre (D17). The bed survey measures the real machine position of every label.
- **Board → view:**
  - **F side** (top up): `p_view = ( x, −y)`, `θ_view = θ_kicad`
  - **B side** (flipped left–right, bottom up): `p_view = (−x, −y)`, `θ_view = 180° − θ_kicad` ← **decision D10, not verified yet**
- **View → machine** is only a rotation + translation (no mirroring), found by board registration.
- **Check for D10:** also work out the angle from the pads (pin-1 pad relative to the pad centroid,
  in the view frame). This doesn't depend on KiCad's angle convention. If the two methods disagree
  for any part, stop and report it.

## Board registration

1. Move the camera over each of the 4 mounting holes (nominal positions come from the holder position in `machine.yaml`).
2. Detect each hole (see [06-vision.md](06-vision.md#42-board-mounting-holes-board-registration-target-board)). This gives machine points `m_i`.
3. Convert the KiCad hole positions to the view frame. This gives `v_i`.
4. Fit `m = R(φ)·v + t` with least squares (`cv2.estimateAffinePartial2D`, or the Kabsch/Umeyama method).
5. **Checks (stop if any fails):**
   - the scale is within 1 ± 0.002
   - every hole's residual is ≤ 0.15 mm
   - |φ| < 3° (a large angle means the board is in the holder wrongly)
6. Placement target: `m = R(φ)·p_view + t`. Nozzle turn between pick and place:
   `Δ = φ + θ_view − angle of the picked part`, wrapped to −180 … +180° (+ bottom-camera correction in phase 7).
   The servo makes this turn (D18). The angle at the pick is free, so the app picks at a servo angle
   that leaves room for Δ inside the measured servo range. If Δ doesn't fit, it refuses the part.

## G-code rules and grblHAL settings

**Settings** (store the full list in `firmware/grblhal-settings.txt`):

| Setting | Value | Why |
|---|---|---|
| `$1` | 255 | Motors stay powered when idle, so no drift (there are no switches to re-home) |
| `$20`, `$21`, `$22` | 0 | Soft limits, hard limits and homing are off in grblHAL. The app does homing and soft limits |
| `$100–$102` | steps/mm (X, Y, Z) | `motor steps/rev × microsteps ÷ mm per rev`. Example: 200 × 8 ÷ 8 mm lead = 200 |
| `$110–$112` | max speed (mm/min) | Start low, tune in phase 1 |
| `$120–$122` | acceleration (mm/s²) | Start low, tune in phase 1 |
| `$3` | direction invert mask | Makes the axes point as in the frames table |
| `$30`, `$32` | bench Grbl build: `$30=2500`, `$32=0`. grblHAL: _TBD_ (check when there is a Pico) | `$32=0`: laser mode off. `$30=2500`: the status report shows the servo pulse width (D18, [../firmware/README.md](../firmware/README.md)) |

**Protocol:** send one line, wait for `ok` (or `error:n` → stop). Poll `?` every 100–200 ms for the
state and position. "Motion done" means the state is `Idle` **and** the planner is empty.

**Rules for generated G-code:**
- Always `G21 G90` (mm, absolute). Use `G54` work coordinates.
  Set the origin so that X0 Y0 = the centre of the HOME mark. HOME is just outside the camera
  reach (D17), so the camera can't sit over it: the camera stops as close as it can, measures
  HOME's offset (dx, dy) in the picture, and the app sends `G10 L20 P1 X<−dx> Y<−dy>` (the camera
  is at −dx, −dy from HOME).
- Lift to `safe_z` **before** any XY move. Moves at low Z are only straight down and up.
- **Backlash:** approach every pick, place and camera point from the same side.
  End with a short move (e.g. 0.5 mm) in +X +Y.
- Pump `M8` (on) / `M9` (off): flood coolant. Valve `M7` (vacuum) / `M9` (vent): mist coolant (D3, changed by D18).
  `M9` turns off both: it vents the valve **and** stops the pump. So the app sends `M8` again after each place.
- **Nozzle angle (D18):** `M3 S<us>` (pulse width in microseconds, from the calibration table), `M5` = servo limp.
  Like the pump and valve, Grbl runs it only after the queued moves. Send it only with Z up
  (right after the lift: the servo turns during the travel). The servo has no feedback, so a
  `G4 P<settle>` (`servo.settle_s`) must come after the last turn and before Z goes down.

**Pick and place sequence** (the numbers come from the config):
```gcode
G0 Z0                    ; safe height
M8                       ; pump on (runs when Z is up; the valve is still vented)
M3 S..                   ; servo to the pick angle (pulse width in µs, from the calibration table)
G0 X112.40 Y63.15        ; nozzle over the part (with backlash approach; the servo turns meanwhile)
G4 P..                   ; servo.settle_s: the turn must be finished before Z goes down
G1 Z-4.2 F600            ; pick height of the package
M7                       ; valve to vacuum (runs after Z arrives)
G4 P0.15                 ; let the vacuum build
G0 Z0
M3 S..                   ; servo to the place angle (runs after the lift, D18)
; (phase 7: G0 to the bottom camera, measure, correct)
G0 X.. Y..               ; place position (the servo turns during the travel)
G4 P..                   ; servo.settle_s
G1 Z-3.1 F600            ; board_top_z + part height (the spring absorbs the error)
M9                       ; vent → the part drops (this also stops the pump)
G4 P0.10
G0 Z0
```
Every line goes to Grbl. Grbl runs `M3`, `M7`, `M8`, `M9` and `G4` only after the moves before them (D3, D18).

## Job flow

1. **Export** (on the KiCad PC): `tools/export_job.py board.kicad_pcb --side B` → `jobs/<board>_B.json`.
   - The list includes only parts with pads where none of them is through-hole. DNP and "exclude from board" parts are left out.
   - Each part has: ref, value, footprint, x, y, rot, side, pads (with pin 1), LCSC number.
2. **Plan:** match each (value, footprint) to a package in `feeder.yaml` (`footprint_map`).
   Unknown packages → **manual list** (placed by hand; the solder helper can guide them).
   Then:
   - group the placements by nozzle,
   - sort by part height,
   - pick the shortest path inside each group (nearest neighbour),
   - split into **loads of up to 5 lines** (one value + footprint line per slot; 5 slots, D17,
     `feed_tray.slots` in `config/bed.yaml`). Keep the lines of one nozzle together across loads,
     so nozzle changes are few,
   - make a load sheet for each load: slot → value, footprint, quantity (the camera can't read values).
3. **Load:** the load sheet and QR-scan helper ([05-feed-tray.md](05-feed-tray.md#5-loading-workflow-in-the-app)).
4. **Run:** home, register the board, scan the feed tray, then run the pick/place loop.
   After each load, the operator empties the slots and loads the next one.
5. **Report:** placed, skipped and failed parts per reference, plus a list for hand-fixing.

## Pick and place state machine

```
 START ─▶ NOT_HOMED ──(camera homing ok)──▶ READY ──(start job)──▶ REGISTER_BOARD
                ▲                                                     │
                │ E-stop / ALARM / serial lost                         ▼
                └──────────── FAULT ◀── any state            SCAN_FEED_TRAY (current load)
                                                                      │
          ┌──────────────────────────────────────────────────────────▼
          │  SELECT next placement (nozzle right? if not → WAIT_OPERATOR "change nozzle")
          │  PICK → PICK_CHECK ──fail──▶ retry ≤ 3 ──▶ next part in the slot ──▶ slot empty → WAIT_OPERATOR "refill slot n"
          │  (ALIGN on the bottom camera, phase 7)
          │  PLACE → LOG
          │  every 20 parts: CHECK_HOME (drift > 0.05 mm → PAUSE, home again, register the board again)
          └── end of load → WAIT_OPERATOR "empty the slots, load the next lines" → SCAN_FEED_TRAY → … → DONE
```

## Safety rules in software

1. **No motion while `NOT_HOMED`**, except jogging (at low speed, within the soft limits as they were at the last homing, with a warning).
   The homed flag is cleared by: E-stop, ALARM, a Pico reset, or losing the serial link.
2. Every target is checked against `soft_limits` (X, Y, Z) **before** it is sent. There are no
   limit switches, so X/Y `soft_limits` come from the **measured reach** (D17): where the camera
   centre can go, in mm from HOME, measured by jogging to every end and stopping ~5 mm before it
   (`tools/bed_test_run.py --measure-reach`, now in `config/bench.yaml` → `limits.reach_from_home`).
   A nozzle target is checked as the camera position it needs (nozzle point − nozzle offset).
   The reach is measured from HOME: if HOME moves, measure it again.
3. XY moves are only sent when Z ≥ `safe_z`.
4. Z never goes below the lowest allowed height for the current action (`pick_z` of the package, or `board_top_z + part height − spring travel`).
5. Speeds are capped at `max_feed` from the config.
6. The pump, the valve and the nozzle servo are only switched through G-code (D3, D18).
7. On any exception: send feed hold `!`, move to the FAULT state, and tell the operator.
8. Watchdog: no status reply for 2 s → FAULT.
9. Every place target must lie inside the PCB area (from `config/bed.yaml`, D17), and every pick
   inside a slot zone (from the bed survey). Otherwise the app refuses the move.
10. The nozzle servo turns only with Z up (`M3 S<us>` right after the lift), and only inside its measured
    pulse range. Between the last turn and the next Z-down move there is always a `G4 P<settle>` (`servo.settle_s`, D18).

## Config and log files

| File | Written by | Contents |
|---|---|---|
| `config/machine.yaml` | calibration tools (also the bed survey) + a person | See [../config/machine.example.yaml](../config/machine.example.yaml) |
| `config/bed.yaml` | a person (read by `make_labels.py` and the app) | Bed size, A4 print settings (page, check bars), marker IDs and size, PCB area, slots in use, slot zone, layout in mm from HOME (D17). See [../config/bed.yaml](../config/bed.yaml) |
| `config/bench.yaml` | a person + `bed_test_run.py --measure-reach` (the reach) | Test bench only: serial port, axis map, feed and step caps, the reach from HOME. See [../config/bench.yaml](../config/bench.yaml) |
| `config/feeder.yaml` | a person + the planner (`slots:` for each load) | See [../config/feeder.example.yaml](../config/feeder.example.yaml) |
| `hardware/print/bed_labels_a4.pdf`, `bed_labels_a4_preview.png`, `bed_map.png` | `make_labels.py` | The A4 label sheet (vector PDF, print at 100 %), a picture of that page, and a picture of the bed showing where to stick each label |
| `jobs/*.json` | `export_job.py` | Placement list for one board side |
| `logs/jobs/<date>.jsonl` | runner | One line per placement: ref, slot, pick result, positions, time |
| `logs/vision/<date>/` | vision | Images with the results drawn on them |
| `logs/test_run/<time>/` | `tools/bed_test_run.py` (bench) | `survey.yaml` (every marker in mm from HOME, the camera matrix, the HOME drift) and a picture of every label |

Keep `config/` under version control, so you can see when a calibration value changed.

## Web UI (FastAPI + one web page)

| Screen | Does |
|---|---|
| Status | Homed?, position, grblHAL state, E-stop, current load |
| Jog + camera | Live top/bottom camera with outlines drawn on top, jog buttons, "go to" point |
| Calibration | Wizard for each step in [08-calibration.md](08-calibration.md) |
| Job | Import a job, show the loads, load sheet (slot → value), QR scan to find the slot, part labels, feed-tray scan warnings |
| Run | Start / pause / skip / stop, progress, live camera, last errors |
| Log | Placement history, failed parts |

**Built on the bench (2026-10-08): `ui/app.py`** (`.venv/bin/python ui/app.py`, opens http://127.0.0.1:8000;
`--lan` for a phone or tablet). It runs on the bench engine (`ui/engine.py` = `tools/nozzle_calibrate.py` with every
lock of D19), not yet on `src/pnp/` (D20). The page has:
- **Live camera** with the red cross, the labels, the board's corner holes and where the nozzle comes down.
  Click the picture: that spot goes to the red cross.
- **Bed map**: reach, labels, PCB area, board, parts (to place / placed), camera and nozzle. Click the map: the
  camera goes there.
- **Machine**: Home, X / Y / Z jog, the nozzle turn (servo), pump. Keys: arrows, `[ ] { }`, `, .`, `h`, space = STOP.
- **Setup**: 1 reach (drive to the 4 ends), 2 camera height (Z0), 3 heights (pick, place, moving, paper),
  4 camera-to-nozzle offset (`n` way, or 4b: land the nozzle, then click where it landed), 5 board: side, top edge,
  the 4 corner holes by hand (Go, click the hole, Set, Fit) or found automatically.
- **Feeders**: which component line (value + package) is in which slot (`config/feeder.yaml`). Only parts with
  2 pads and no polarity (R, C) are offered; the others are listed as hand-placed.
- **Job**: dry run (camera only) or REAL; pause / resume / stop; an empty slot stops for a refill; parts already
  placed on the board are never placed again (`logs/job_state.yaml`, "New board" clears it).
Saved data: `config/nozzle_calibration.yaml`, `config/board_registration.yaml`, `config/feeder.yaml`,
`config/bench.yaml` (reach). Tests: `tests/test_ui.py`.

## Reuse from `../solder_helper`

- `extract_board.py`: the `pcbnew` code for position, rotation, pads and fields → base of `tools/export_job.py`.
- LCSC QR scanning (zxing-cpp in `vendor/`) → the "which slot?" helper.
- `label_server.py` + Brother PT-D410 → part labels for a load (optional). The bed labels come
  from `tools/make_labels.py`: an A4 PDF (D13), or on the PT-D410 with 18 mm tape through
  `ptouch-print` (`--tape --print --strip`, D15).
- The same pass order (Top SMD → Bottom SMD → THT by hand). Parts on the manual list can be shown in the solder helper.

## Bench tools (before the Pico)

Until the Pico is wired, a **test bench** moves the CNC: an Arduino Uno + CNC shield with our Grbl 1.1h
servo build (3 axes; D11 = nozzle servo, A3 = pump relay, D18, [../firmware/README.md](../firmware/README.md)),
and the Xitech camera on the head. Settings in `config/bench.yaml`:
- `axis_map`: the motors are not on the drivers their names say. Found with the owner watching
  (2026-10-06): +X (head right) = Grbl −Y, +Y (head back) = Grbl +Z, +Z (up) = Grbl −X.
  Bench tools move only through `bench.Grbl` (`tools/bench.py`), never with raw G-code axes.
- `limits`: feed cap, largest single move, and `reach_from_home` (where the camera centre can go,
  mm from HOME).
- `nozzle_servo` (firmware version, pulse range, settle time) and `pump` (`M8` / `M9`).

`tools/bed_test_run.py` is the bed test: a live camera window (the picture as seen from the
front), jog keys, homing on HOME and the bed survey.

| Key | Does |
|---|---|
| arrows, or j / l / i / k | Jog: left / right = X, up = to the back (+Y), down = to the front (−Y) |
| 1 2 3 4 | Jog step 1 / 5 / 10 / 30 mm |
| h | HOME in view: calibrate the camera (small moves to the left and the front), then measure where HOME is in the picture (no move onto it) and make it X0 Y0 |
| s | Survey: visit every slot label, centre on it as far as the reach allows, measure it, save `logs/test_run/<time>/survey.yaml` and pictures, go back near HOME and measure the drift |
| g | Go back near HOME |
| a | The whole test automatically (also `--auto`, then it quits at the end): calibrate on a label in view, drive to HOME along the slot row with a stall check after every 10 mm step, measure HOME, survey, come back |
| space | Stop after the current short move |
| q / Esc | Quit (the head stays where it is) |

- **Reach guard:** after h, every move outside `reach_from_home` is refused.
  `--measure-reach` turns the guard off: press h, drive to every end (stop ~5 mm before it), and
  the reach is saved to `config/bench.yaml` on quit.
- **Stall check:** if a marker in view doesn't move as commanded, the tool stops (end of travel or
  lost steps), forgets HOME, and locks g and s until you press h again.
- The method is in [06-vision.md](06-vision.md#46-bed-survey-and-off-centre-home). Tests against a
  simulated head and camera: `tests/test_bed_test_run.py`.
- Other bench tools: `tools/gcode_console.py` (serial console), `tools/axis_camera_test.py` (moves
  one axis and measures it with the camera), `tools/zoom_identify.py` (steps Z down and names parts),
  `tools/servo_pump_test.py` (pump 3 s, servo sweep; `--us` holds one pulse width, `--off` = servo limp, pump off).

## Testing

| Level | What | When |
|---|---|---|
| Unit tests (`pytest`) | pixel→mm, board fit (made-up data including the B-side mirror), planner loads, exact G-code strings, soft-limit checks | Every change |
| Vision regression | Labelled images in `tests/vision_data/` | Every detector change |
| Simulator (`sim.py`) | The whole app with a fake grblHAL (with the servo, pump and valve outputs) and fake cameras, on a laptop | Before running on the machine |
| Dry run (on the machine) | No Z down, no vacuum. The camera visits every placement and draws the footprint over the live image | Before every new board or side. **Required to check D10** |
| Tape test | Place parts on double-sided tape, measure them with the camera | Phase 5 |
