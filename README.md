# Pick and Place Robot (CNC conversion)

A small SMD pick and place machine built from a broken CNC gantry.

- A **Raspberry Pi 5** runs the vision, the job and the web UI.
- A **Raspberry Pi Pico running grblHAL** moves the motors.
- Parts are put **by hand into a feed tray of 5 slots**: printed labels on the T-slot bars of the
  bed, no motors. Each slot holds one cut-tape strip (cover film peeled off by hand), and the
  strips lie side by side, about 30 mm apart. The top camera finds each part in its strip before
  the nozzle picks it.

First target board: **HT5023 energy meter v1** (`../ht5023_energy_meter_v1.kicad_pcb`),
167 SMD placements on two sides. See [docs/11-target-board-ht5023.md](docs/11-target-board-ht5023.md).

## Status

**Phase 0 – planning and first tests (updated 2026-10-07).** The planned app (`src/pnp/`) is
not written yet, and the Pico is not wired yet. What exists:

- The design docs and example config files.
- **Bed labels** (D11–D15): printed on the PT-D410 with 18 mm tape, and stuck on the bed.
  A slot label is 87 × 20 mm with a 60 × 16 mm tape mark for one cut-tape strip (D14).
- **A test bench** on the PC: an Arduino Uno with our Grbl 1.1h servo build moves the CNC, and the Xitech camera
  sits on the head (`config/bench.yaml`, `tools/bench.py`). Bench tools: `tools/gcode_console.py`,
  `tools/axis_camera_test.py`, `tools/zoom_identify.py`, `tools/bed_test_run.py`,
  `tools/dry_run_bench.py` and `tools/servo_pump_test.py`.

**2026-10-06: first bed test run and decision D17.** The bed is 240 × 440 mm (X × Y, D16), but
the machine reaches only part of it. The head hit the right end once (no limit switches; slow,
no damage). Then the owner drove to every end: the camera centre reaches only about
**150 × 230 mm**. The new layout fits inside this reach (D17, `config/bed.yaml`,
`hardware/print/bed_map.png`), in mm from the HOME marker:

- **5 slots** (labels 3–7) on the T-slot bars, running front-back, markers at the back, tape
  holes to the right. **Loads of ≤ 5 lines.**
- **HOME** on the next bar right of slot 3, in line with the slot markers.
- **PCB area = the HT5023 board, 130 × 140 mm**, just in front of the slots. No PCB corner labels.
  This is the largest board on this machine.
- Bottom camera: _TBD_ (no room inside the reach).

`tools/bed_test_run.py --auto` found HOME and all 5 slots (within 3 mm of the plan). Next: measure
the reach from the new HOME exactly (`--measure-reach`). **Risk:** the nozzle reaches only the
camera reach shifted by the camera-to-nozzle offset, so the head must keep the nozzle very close
to the camera (step 2.1).

**2026-10-06: speed test and the first dry pick and place.** The bench now runs at 2000 mm/min
(80 mm/s²) without losing steps. The camera checked this. `tools/dry_run_bench.py` found the
HT5023 board on its 4 holes (worst hole 0.25 mm). It then visited all 15 bottom-side 100 nF
places from slot 3, with the camera in place of the nozzle. After the run, HOME had moved only
0.08 mm.

**2026-10-07: nozzle servo on the bench Uno (D18).** The nozzle turns with an MG995 hobby servo.
Its pulses come from Grbl itself: our Grbl 1.1h build ([firmware/README.md](firmware/README.md)) sends
them on the spindle pin D11 (`M3 S<us>` = pulse width in microseconds, `M5` = servo limp). The pump
relay moved to flood coolant (`M8`/`M9`), and the valve (later) goes on mist (`M7`/`M9`). No second
controller. The Pico with grblHAL should work the same way: to be checked when there is a Pico.

**2026-10-08 to 2026-10-12: web page, first real placements, feeder trays.** The bench is now run from a web
page (`ui/app.py`): live camera, bed map, jog, the set-up steps (reach, camera height, nozzle offset by ink
marks or by eye, board corners), feeders and the job. The first 100 nF parts were picked from a strip and
placed on the HT5023 board with the nozzle and the pump; every pick is checked by looking at the pocket
afterwards. Three things learned the hard way: the head camera is fixed on the head (it does not move with
Z), the MG995 servo does not come back to the same angle, and the needle sits 1.6 mm off the servo axis.
The software works around all three, but a stepper for the rotation and a Z reference are the next hardware
steps. Six 3D printed feeder trays (`hardware/tray/`) are designed: they slide into the T-slots, peel the
cover film off while the strip is pushed in, and carry the slot marker. Not printed yet.

Some design details wait on the [open questions](docs/01-requirements.md#open-questions)
(CNC model and travel, driver model, pump).

## Try it now: which SMD part is this?

`tools/smd_identify.py` is a small stand-alone demo. Put SMD parts on a printed sheet under the
camera, and it names the package (0603, 0805, SOT-23, SOIC-8, …) from the measured size.

```
python3 -m venv .venv && .venv/bin/pip install opencv-python numpy pyyaml pytest   # once
.venv/bin/python tools/smd_identify.py --make-sheet    # writes hardware/print/smd_identify_sheet.pdf
.venv/bin/python tools/smd_identify.py                 # live window (q = quit, d = debug, s = save)
.venv/bin/python -m pytest tests/                      # tests with a synthetic camera image
```

Print the sheet at **100 % (Actual size)** and check that the bar is 50 mm. The package sizes and
detection settings are in `config/smd_identify.yaml`. It is a size-only method: parts with nearly
the same size (1206 and a 3215 crystal) show two guesses, and the resistor/capacitor hint is only a guess.

## Quick facts

| Item | Choice |
|---|---|
| Main computer | Raspberry Pi 5, 8 GB |
| Motion controller | Raspberry Pi Pico + grblHAL, 3 axes (X, Y, Z) |
| Motor drivers | 3 existing microstep/servo drivers (X, Y, Z) |
| Nozzle rotation | MG995 hobby servo, not a G-code axis. Its pulses come from the spindle PWM pin of the motion controller (`M3 S<us>`, D18). No extra board |
| Vacuum | DC pump (`M8`/`M9`) + 3/2-way solenoid valve (`M7`/`M9`) + spring-loaded nozzle |
| Homing | Camera homing on a fixed HOME mark (no limit switches). HOME sits just outside the camera reach, so it is measured in the picture (D17) |
| Cameras | Top: USB camera on the head. Bottom: Pi camera, fixed, looking up |
| Feeding | Hand-loaded feed tray: 5 slot labels (ArUco marker + tape mark) on the T-slot bars, about 30 mm apart, no motors. One cut-tape strip per slot (film off by hand, 8 or 12 mm tape). Up to 5 part lines per load (D17) |
| Bed markers | Labels from `tools/make_labels.py`, on A4 matte sticker paper (D13) or 18 mm PT-D410 tape (D15): HOME and feed slots (the PCB corner labels are not used, D17) |
| Software | Python 3 (OpenCV, picamera2, pyserial) + web UI |
| Work area | Bed 240 × 440 mm, but the camera centre reaches only about 150 × 230 mm. Largest board 130 × 140 mm (D17) |
| Accuracy target | ±0.1 mm for chip passives |

## Documents

Read them in order the first time.

| # | Document | What's in it |
|---|---|---|
| 01 | [Requirements](docs/01-requirements.md) | Goals, scope, constraints, target numbers, open questions |
| 02 | [Architecture](docs/02-architecture.md) | System diagram, who does what, decision log |
| 03 | [Mechanical](docs/03-mechanical.md) | CNC conversion, head, nozzles, bed layout, PCB holder |
| 04 | [Electrical](docs/04-electrical.md) | Wiring, power, I/O map, safety |
| 05 | [Feed tray](docs/05-feed-tray.md) | Slot labels, loading the tape strips, loads of ≤ 5 lines, feed-tray scan |
| 06 | [Vision](docs/06-vision.md) | Cameras, lighting, detection pipelines |
| 07 | [Software](docs/07-software.md) | Modules, coordinate frames, G-code rules, job flow, safety |
| 08 | [Calibration](docs/08-calibration.md) | Step-by-step calibration procedures |
| 09 | [Project plan](docs/09-project-plan.md) | Phases, checklists, acceptance tests, risks |
| 10 | [Bill of materials](docs/10-bom.md) | What you have, what to buy |
| 11 | [Target board: HT5023](docs/11-target-board-ht5023.md) | Analysis of the first board to build |

Example configuration: [config/machine.example.yaml](config/machine.example.yaml),
[config/feeder.example.yaml](config/feeder.example.yaml). Bed and labels: [config/bed.yaml](config/bed.yaml).
Test bench: [config/bench.yaml](config/bench.yaml).

## Folder layout

```
pick_and_place_robot/
├── README.md       this file
├── docs/           design documents (numbered in reading order)
├── config/         configuration files (copy the examples to machine.yaml / feeder.yaml and fill in)
├── tools/          stand-alone tools: labels, demo, test bench, calibration
├── ui/             the web page for the bench: live camera, jog, set-up steps, feeders, job (ui/app.py)
├── firmware/       the bench Grbl build with the nozzle servo (see firmware/README.md)
├── tests/          pytest tests for the tools
├── hardware/print/ printable labels and the bed map
├── hardware/tray/  3D printed feeder trays (the script that makes them, and the STL files)
└── logs/           results of the tools (for example logs/test_run/)
```

Folders to add later: `src/` (the Python app for the Pi), the grblHAL build and settings in `firmware/`,
wiring photos.

## Related tools

`../solder_helper/` is the web app for hand-assembling this board. This project will reuse:

- `extract_board.py`: reads the KiCad board with `pcbnew` to get positions, rotations and pads.
- The LCSC QR code scanner, so scanning a part bag shows which feed slot to put it in.
- `label_server.py` and the Brother PT-D410, to print part labels. The bed labels come from
  `tools/make_labels.py`: as an A4 PDF (D13), or straight on the PT-D410 with 18 mm tape (D15).
