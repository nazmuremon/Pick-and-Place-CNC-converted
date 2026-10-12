# 09 – Project Plan

**Current phase: 0 (planning and first tests)** · updated 2026-10-07 (design change D11/D12/D13: feed tray on printed labels, A4 sticker paper; D14: one cut-tape strip per slot, slots 30 mm apart; D16: bed 240 × 440 mm; first bed test run on the bench; D17: the layout that fits the measured reach: 5 slots, PCB area = the board; D18: the nozzle rotation is an MG995 hobby servo on the spindle PWM pin of Grbl, not a 4th axis; the servo Grbl build is flashed on the bench Uno)

Effort is a rough estimate for **part-time** work. Each phase ends with an acceptance test. Don't
start the next phase until the test passes, because later phases depend on it.

## Overview

| Phase | Name | Effort | Done when | Depends on |
|---|---|---|---|---|
| 0 | Repair and inspection | 2–5 days | Axes move smoothly, measurements recorded | Q1 |
| 1 | Motion | 1 week | 50 repeated moves return to the same point | Q2, Q3 |
| 2 | Head and vacuum | 1 week | An 0805 stays on at full speed and drops cleanly | Q4, Q5 |
| 3 | Cameras and calibration | 1–2 weeks | Camera homing repeats within ±0.05 mm | 1, 2 |
| 4 | Feed tray and detection | 2 weeks | R3: ≥ 99% found, ≤ 0.1 mm, ≤ 2° | 3 |
| 5 | First placements | 1 week | R1: ±0.1 mm, ±1° on the tape test | 4 |
| 6 | Full job | 2 weeks | One HT5023 bottom side placed and reflowed | 5, Q6, Q7 |
| 7 | Upgrades | Ongoing | — | 6 |

## Phase 0: Repair and inspection
- [ ] Answer Q1–Q5 in [01-requirements.md](01-requirements.md#open-questions)
- [ ] Find and fix the broken part
- [x] Measure the camera-centre reach on the bench (2026-10-06, jogged to every end: only about 150 × 230 mm, D17)
- [ ] Measure the reach from the new HOME exactly: `tools/bed_test_run.py --measure-reach` (it is an estimate now)
- [ ] Measure the full travel of each axis (and Z), drive type, backlash, squareness and flatness (table in [03-mechanical.md](03-mechanical.md))
- [x] Check the bed layout against the reach → D17 (5 slots, HOME on the next bar right of slot 3, PCB area = the HT5023 board, no PCB corner labels), 03-mechanical updated
- [ ] Check the D17 layout again when the full travel (Q1) and the camera-to-nozzle offset are known
- [ ] Find a place for the bottom camera (phase 7): no room inside the reach now
- [ ] Order the parts ([10-bom.md](10-bom.md))
- [ ] Set up the Pi 5: Raspberry Pi OS 64-bit, active cooler, SSH, Python venv, git repo for this folder

**Accept:** every axis moves smoothly by hand; all measurements recorded.

## Phase 1: Motion
- [ ] Build grblHAL for RP2040 with 3 axes, X, Y, Z (Web Builder; the nozzle servo is not a grblHAL axis, D18), spindle PWM at 50 Hz for the nozzle servo, flood (pump) and mist (valve) outputs: to be checked (D18). Save the build options in `firmware/`
- [ ] Fill in the Pico pin map in [04-electrical.md](04-electrical.md)
- [ ] Wire the 5 V buffer, X/Y/Z drivers and E-stop
- [ ] Set `$1=255`, `$20=$21=$22=0`, `$3` directions, steps/mm, speeds and accelerations
- [ ] `motion.py`: connect, send/ok, status poll, wait until idle, jog, set origin. Plus `safety.py` soft limits (X/Y from the measured reach, D17)
- [ ] `sim.py` fake grblHAL, so the app can be developed on a laptop

**Accept:**
- 50 × 100 mm moves on each axis come back to the same point. Check with a dial indicator: ≤ 0.05 mm.
- The E-stop stops everything and puts grblHAL in alarm.

## Phase 2: Head and vacuum
- [ ] Head design with a **small camera-to-nozzle offset** (D17: the nozzle reaches only the camera reach shifted by the offset; rough limit in [03-mechanical.md](03-mechanical.md#3-head))
- [ ] Take off the spindle, build the head plate, fit the nozzle servo (MG995, D18: nozzle on the horn, tube loop) and the spring nozzle holder
- [x] Bench: Grbl 1.1h servo build on the Uno (D18, [../firmware/README.md](../firmware/README.md)): D11 = servo pulses (`M3 S<us>` / `M5`), A3 = pump relay (`M8` / `M9`), A4 = valve later (`M7` / `M9`). Flashed and tested 2026-10-07 (`tools/servo_pump_test.py`)
- [x] Bench: real pick and place, one part per press (`tools/nozzle_calibrate.py` key `r`), heights set in software and remembered from the camera height (D19: `k` pick, `d` place, `m` moving, `0` twice = new camera height), offsets by eye (`n` on a part / a board hole). Three adversarial reviews, 157 tests, **simulation only** (2026-10-07)
- [ ] Bench, before the first real pick and place: fix the lost steps (camera grid run 2026-10-07), run the speed test again, measure the reach again (`limits.reach_stale`), then `h`, `0` twice, `k`, `a` + `d`, `m`, `n` twice on a part and on a board hole, `r`
- [ ] Nozzle servo on the Pico: grblHAL spindle PWM at 50 Hz (to be checked when there is a Pico). Servo power: own 5–6 V supply, about 1000 µF at the plug, common GND, never the Uno or Pico 5 V pin (D18)
- [ ] Wire the pump, the 3/2 valve, the MOSFETs and the flyback diodes
- [ ] Test `M7/M8/M9` timing: the valve opens only after Z arrives. Test `M3 S<us>` + `G4` settle: the turn is finished before Z goes down (D18)
- [ ] Try nozzles N1–N3 on real parts (0603, 0805, SOT-23, SMA)

**Accept:** an 0805 and an SMA diode stay on the nozzle during full-speed moves, and fall off within 0.1 s of `M9`.

## Phase 3: Cameras and calibration
- [ ] Mount the top camera + LED ring + shroud. Mount the bottom camera (it can stay unused until phase 7)
- [ ] `camera.py` with locked settings and fresh frames
- [ ] Calibration steps 4–8 and the nozzle servo table (pulse → angle, range, settle time, D18) in [08-calibration.md](08-calibration.md)
- [x] Bench: measuring HOME off-centre works (`tools/bed_test_run.py`, 2026-10-06; HOME is just outside the reach). R7 not checked yet
- [ ] Camera homing on the HOME mark in the app (off-centre, D17). Make the app refuse moves when not homed

**Accept:** R7, homing repeats within ±0.05 mm (10 tries). Camera-to-nozzle offset agrees within 0.05 mm.

## Phase 4: Feed tray and detection
- [x] Printable bed labels generated (`tools/make_labels.py`; slot labels with 60 × 16 mm tape marks, D14)
- [x] Print the bed labels and check the scale (2026-10-06: on the PT-D410, 18 mm tape, D15; the 100 mm check label measured 100 mm)
- [x] Stick them on the bed (2026-10-06, D17 layout, `bed_map.png`): slot labels 3–7 on the T-slot bars, markers at the back; HOME (text cut off) on the next bar right of slot 3. No PCB corner labels
- [x] Bed survey on the bench: `tools/bed_test_run.py --auto` finds HOME and all 5 slots (2026-10-06: within 3 mm of the plan; HOME check 0.20 / 0.12 mm, see [08-calibration.md](08-calibration.md#6-home-mark-and-camera-homing))
- [ ] Bed survey in the app (`feeder.py`, step 4.2): store the machine positions, draw the PCB area and the 5 tape marks on the live image
- [ ] `feeder.py` + `vision/parts.py`: find every part in a cut-tape strip in a slot (search along the strip, pocket pitch, 2 views per slot; position, angle, size check, min gap) for 0603, 0805, 1206, SOT-23, SOD-123, SMA
- [ ] Polarity rules over 360° from the image (the strip can lie at any angle), cross-checked with `angle_in_tape`. Touching, tilted and upside-down parts → unclear
- [ ] Feed-tray scan and part count, with warnings for short, empty or unclear slots. Save images to `logs/vision/`
- [ ] Start the vision regression set (`tests/vision_data/`)

**Accept:** R3: 100 parts per package → ≥ 99% found, position ≤ 0.1 mm, angle ≤ 2°, no wrong polarity.
No unclear part (touching, upside down, wrong size) is ever reported as found.

## Phase 5: First placements
- [ ] Full pick → check → place loop for one package (0805), from `runner.py`
- [ ] Tape test: 20 parts on double-sided tape, measured with the camera
- [ ] Tune pick/place Z, the pause times and the backlash approach
- [ ] Repeat for 0603 (nozzle N1) and SOT-23

**Accept:** R1: ±0.1 mm, ±1° for 0805 and 0603. Pick success ≥ 98% (R4).

## Phase 6: Full job on the HT5023 board
- [ ] `tools/export_job.py` (pcbnew; SMD decided by pads). Test it against solder_helper's `board_data.js`
- [ ] PCB holder that fits the PCB area next to the slots: 130 × 140 mm, only ~3 mm to the slot labels (D17, step 6.3)
- [ ] Board registration on the 4 mounting holes, with the fit checks
- [ ] Planner: loads of ≤ 5 lines (one per slot, D17), nozzle groups, order, manual list
- [ ] Web UI: load sheet (slot → value), QR slot finder, run screen
- [ ] **Dry run on the B side, and check decision D10 (B-side rotation)**
- [ ] Paste, place and reflow one HT5023 top side (24 phase-1 parts, simplest), then one bottom side (124 phase-1 parts)

**Accept:** one board with all phase-1 parts placed by the machine, reflowed, and passing visual inspection.
Every failed or skipped part is logged.

## Phase 7: Upgrades (choose by need)
- [ ] Bottom-camera alignment → SOIC, SOP, optocouplers (12 parts on the HT5023). Its position is _TBD_: no room inside the reach (D17)
- [ ] LQFP-100 0.5 mm (U1, HT5023) with the bottom camera; target R2
- [ ] Vacuum sensor for checking picks
- [ ] 3 limit switches for automatic rough homing (the inputs are already free)
- [ ] Speed tuning (R6)
- [ ] Board v2: add fiducials and fix the SMD attributes (see [11-target-board-ht5023.md](11-target-board-ht5023.md))

## Risks

| # | Risk | Effect | How to reduce it |
|---|---|---|---|
| K1 | Missed steps (no limit switches) | Parts placed in the wrong spot, crash | Camera homing; HOME check every 20 parts; moderate acceleration; `$1=255`; driver ALARM wired; switches later. The bench tools stop when a marker doesn't move as commanded (stall check) |
| K2 | Crash (no hard stops). It happened once on the bench: the head hit the right end (2026-10-06; slow, no damage) | Broken nozzle or camera | Soft limits on every move, from the **measured reach** (D17; the bench tools refuse moves outside it); measure the reach again if HOME moves; safe-Z rule; E-stop; slow jogging when not homed |
| K3 | Light changes | Missed or false detections | Shroud; fixed LED rings; locked camera settings; regression images |
| K4 | Z error | Crushed or tilted parts | Spring nozzle; Z per package and per board; tape test |
| K5 | Parts stick to the nozzle | Wrong part dragged along | Valve vent; pause; pick check; optional small air puff |
| K6 | Camera cable breaks | Stops the machine | USB camera + high-flex cable in a cable chain; CSI only on the fixed camera |
| K7 | Part upside down or the wrong way round (a strip laid the wrong way round, with the holes to the left, or a loose part at any angle) | Wrong polarity | Polarity rules over 360° from the image, cross-checked with `angle_in_tape`; upside-down check; size filter; "unclear → skip"; parts in cut-tape strips, holes to the right (D14, D17) |
| K8 | B-side rotation formula wrong (D10) | Whole side rotated wrong | Pad-based check; required dry run before the first B-side job |
| K9 | **Small reach** (this is now a fact, not only a risk): the camera centre reaches only about 150 × 230 mm of the 240 × 440 mm bed (measured 2026-10-06, D17). The nozzle reaches the same box **shifted by the camera-to-nozzle offset**, and the D17 layout already uses most of it (136 of 150 mm in X, 209 of 230 mm in Y) | Part rows or parts of the board out of the nozzle's reach; no room for the bottom camera | Measure the reach exactly (`--measure-reach`); find out why the travel is so small (Q1); **head design with the nozzle very close to the camera** (step 2.1; rough limit: camera at most ~10 mm right of and ~18 mm behind the nozzle, estimate, see [03-mechanical.md](03-mechanical.md#3-head)); check the nozzle offset against the reach in step 3.7. If it still doesn't fit: fewer slots, or only part of each tape mark (owner's OK needed, see [01-requirements.md](01-requirements.md#work-area-needed)) |
| K10 | Pi timing jitter | Uneven motion | All timing in grblHAL (D1) |
| K11 | Wrong line in a slot, or a leftover part from the last load (the camera can't read values) | Wrong part placed (R5) | Load sheet; slot numbers on the labels; QR scan shows the slot; size check against the package; camera checks the slots are empty before each new load |
| K12 | Glare on glossy labels | Markers or parts not found, false outlines | Matte sticker paper, not glossy (D13); diffuser; check during the bed survey |
| K13 | A tape strip lifts or slides (it is not flat, or a pick pulls it) | Parts move, failed picks | Hold the strip flat (a small piece of tape at each end); pick check looks again and updates the positions |
| K14 | HOME is just outside the camera reach, so it is measured off-centre (D17) | Homing error grows with the offset and with errors in M (first bench check: 0.20 / 0.12 mm) | Lens calibration and longer calibration moves (step 3.5); always home from the same spot, as close as the reach allows; 10-homing test (step 3.6, R7) |
| K15 | Nozzle servo (D18): no feedback, a few tenths of a degree dead band, some jitter, about 55 g more on the Z axis; the wrong power can make a voltage dip | Parts placed at a wrong angle; a dip resets Grbl and the position is lost | Pulse → angle table measured with the camera; settle time after each turn; turn only with Z up; own 5–6 V supply + about 1000 µF, never the Uno or Pico 5 V pin; tape test (R1: ±1°) |
