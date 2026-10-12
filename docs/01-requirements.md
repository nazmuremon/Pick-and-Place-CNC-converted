# 01 – Requirements

## Goal
Place SMD parts on boards that already have solder paste (printed with a stencil), for prototypes
and small batches. The machine must be built from the broken CNC and the parts we already have.

## Users and use
- One operator. The operator:
  1. prints solder paste,
  2. lays the cut-tape strips for the first load in the 5 feed slots (the app shows which line goes in which slot),
  3. puts the board in the holder,
  4. starts the job,
  5. answers the machine when it pauses (next load, refill a slot, bad pick, nozzle change).
- The UI is a web page served by the Pi. It can be opened on the Pi's own screen or on a laptop on the LAN.

## Fixed constraints
| # | Constraint |
|---|---|
| C1 | The main computer is a Raspberry Pi 5, 8 GB. |
| C2 | Use the 3 existing microstep/servo stepper drivers. |
| C3 | Use the existing DC vacuum pump and nozzle. |
| C4 | No limit switches. |
| C5 | One kind of feeder: a feed tray of **5 slots** on this machine (printed slot labels 3–7 on the T-slot bars, D17), loaded by hand with one cut-tape strip per slot (film peeled off by hand), strips side by side about 30 mm apart. No motors, no automatic magazine. At most 5 lines at a time. |
| C6 | The camera must detect the parts in the feed slots and measure every bed label. We never assume a part's exact position. |
| C7 | The bed is 240 × 440 mm: 240 mm left-right (X) × 440 mm front-back (Y) (measured again 2026-10-06, D16). But the machine reaches only part of it: the camera centre reaches about **150 × 230 mm** (measured 2026-10-06, D17). The largest board on this machine is **130 × 140 mm** (R8). |

## Scope

**In scope (first version)**
- Chip resistors and capacitors from 0603 to 2512.
- SOD-123, SMA (DO-214AC), SOT-23, SOT-23-6, SOT-89 and the 3215 crystal.
- Placing on the top or bottom side (one side per job).
- Loading the feed slots by hand: one cut-tape strip per slot, cover film peeled off by hand
  (LCSC supplies cut tape). 8 mm and 12 mm tape. Loose parts are possible, but not the normal way.
- Board registration on mounting holes or fiducials.

**Later (phase 7)**
- Bottom-camera alignment for SOIC, SOP and SMD optocouplers.
- LQFP-100 with 0.5 mm pitch (HT5023).
- 0402 parts.
- Vacuum sensor for checking picks.

**Out of scope**
- Dispensing solder paste.
- Motorized feeders.
- More than one nozzle at a time.
- Speed tuning beyond the targets below.

## Target numbers
| # | Requirement | Target | How to check |
|---|---|---|---|
| R1 | Placement accuracy, chip passives | ±0.1 mm, ±1° | Place on tape, measure with the camera (phase 5) |
| R2 | Placement accuracy, ICs with the bottom camera | ±0.05 mm, ±0.5° | Phase 7 |
| R3 | Part detection in the feed slots | ≥ 99% found, position error ≤ 0.1 mm, angle error ≤ 2° | 100 parts per package (phase 4) |
| R4 | Pick success on the first try | ≥ 98% | Job log |
| R5 | Wrong part placed | **Never.** The app stops and asks instead of guessing | Job log, review |
| R6 | Cycle time | ≤ 15 s per part at first (~240 parts/hour) | Job log |
| R7 | Homing repeatability | ±0.05 mm | Home 10 times (phase 3) |
| R8 | Largest board | **130 × 140 mm on this machine** (the HT5023 itself). The first target was 150 × 150 mm, but the reach is too small (D17) | Holder design; the PCB area in `config/bed.yaml` |
| R9 | Tallest part | Phase 1: 2.5 mm. Later: 5 mm | Nozzle and Z travel |
| R10 | Safety | E-stop cuts motion in hardware, and there are soft limits on every move | Test in phase 1 |

## Work area needed
Everything the nozzle picks or places, and everything the top camera must see, has to be
reachable.

**Bed:** 240 mm left-right (machine X) × 440 mm front-back (machine Y) (measured again by the
owner, 2026-10-06, D16). The first numbers, 320 × 232 mm with 320 along X, were wrong.

**Reach (measured 2026-10-06, D17):** the machine reaches only part of the bed. During the first
bed test on the bench, the head hit the right end of its travel once (there are no limit
switches; it was slow, no damage). Then the owner jogged to every end and stopped about 5 mm
before it. The **camera centre** reaches only about **150 mm in X × 230 mm in Y**: from the old
HOME position, X −94 … +56 and Y −160 … +70.

HOME then moved (D17). From the new HOME the reach is about **X −154 … −4, Y −227 … +3** (mm from
the HOME marker centre). This is an estimate, worked out from the old measurement. It is in
`config/bench.yaml` (`limits.reach_from_home`). Measure it exactly with
`tools/bed_test_run.py --measure-reach`, and again whenever HOME moves. The bench tools refuse
every move outside it. The real travel of each axis, the Z travel, and what is broken are still
_TBD_ (Q1).

The layout of D16 needed about 190–225 mm in X and 405–440 mm in Y, so it doesn't fit. The layout
of D17 is made for the measured reach. Positions in mm from the HOME marker centre (+X right,
+Y back):

| What | Where | Camera must see it | Nozzle must reach it |
|---|---|---|---|
| HOME | (0, 0) | Yes. It is ~4 mm right of the camera reach, so it is measured in the picture | No |
| Slot markers 3–7 | X −150 … −29, Y ≈ 0 | Yes | No |
| Parts in the tape marks | On the bars (X ≈ −150 … −29), Y ≈ −15 … −80 | Yes | Yes |
| PCB area (the board, its 4 holes) | X −144 … −14, Y −224 … −84 | Yes | Yes |
| Bottom camera (phase 7) | _TBD_: no room inside the reach | — | Yes |

**Z:** needs ≥ 25 mm of travel (safe height above the tallest part, plus pick/place depth and
bottom camera focus). The real Z travel is _TBD_ (Q1).

Drawing and positions: [03-mechanical.md](03-mechanical.md#2-bed-layout-d17).
The numbers are in `config/bed.yaml` (`layout:`, `pcb_area:`).

**The nozzle reach is the camera reach, shifted by the camera-to-nozzle offset.** The nozzle
must reach X ≈ −150 … −14 (136 of the 150 mm) and Y ≈ −224 … −15 (209 of the 230 mm). With the
estimated reach, that leaves room for only a small offset: the nozzle may sit at most about
4 mm right or 10 mm left of the camera, and at most about 3 mm behind or 18 mm in front of it.
A little more if no part sits right at the board edge. These are rough estimates: work them out
again after `--measure-reach` and Q1. The first plan (camera ~35 mm from the nozzle) does **not**
fit. So the head design (build step 2.1) must keep the nozzle very close to the camera. This is
a big risk (K9 in [09-project-plan.md](09-project-plan.md#risks)).

If it still doesn't fit, options (owner's OK needed, build step 0.4):
- find out why the travel is so small (Q1). Maybe it is part of what is broken,
- fewer slots (fewer lines per load, so more loads),
- use only the part of each tape mark that the nozzle can reach (fewer parts per strip),
- move the labels (they don't need exact placement; change `layout` in `config/bed.yaml` and run
  the bed survey again).

The layout is final only when the full travel is known (Q1) and the camera-to-nozzle offset is
measured ([08-calibration.md](08-calibration.md#7-camera-to-nozzle-offset)).

## Open questions
These must be answered before the matching phase can start.

| # | Question | Blocks |
|---|---|---|
| Q1 | CNC model, X/Y/Z travel, lead screw or belt, and **what is broken**? **Partly answered (2026-10-06):** bed = 240 × 440 mm, X × Y (corrected the same day, D16: the first 320 × 232 mm was wrong). The camera centre reaches only about 150 × 230 mm (X × Y), measured on the bench by jogging to every end (D17); from the new HOME about X −154 … −4, Y −227 … +3 (estimate, measure it with `tools/bed_test_run.py --measure-reach`). Still _TBD_: model, the real travel of each axis (and Z), drive type, what is broken (and does it limit the travel?) | Phase 0, bed layout, head design (camera-to-nozzle offset) |
| Q2 | Exact driver model. Closed-loop? ALARM output? Input signal voltage (5 V or 24 V) and current? | Phase 1 wiring |
| Q3 | Motor type (NEMA 17 or 23) and rated current. | Phase 1 |
| Q4 | Pump voltage, current and vacuum level. Is there already a valve? | Phase 2 |
| Q5 | What nozzle do you have now (tip size, fitting)? | Phase 2 |
| Q6 | Is there a solder paste stencil and printing jig for this board? | Phase 6 |
| Q7 | Reflow method (oven or hot plate)? This decides the double-sided order. | Phase 6 |
| Q8 | Where will the UI be used: a screen on the Pi, or a laptop over LAN? | Phase 6 |
