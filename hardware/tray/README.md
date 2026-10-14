# SMD strip holders (3D printed, one per T-slot)

Six holders slide into the T-slots of the bed bars. Each holds one cut-tape strip in its own channel and carries
its slot's marker and number (sunk into the top: spray them black). Peel the cover film off by hand (no blade).

| File (`stl/`) | Holder | Tape | For |
|---|---|---|---|
| `holder_1_8mm_paper.stl` ... `holder_4_8mm_paper.stl` | 1-4 (ArUco 40-43) | 8 mm paper | 0402-1206 resistors and capacitors |
| `holder_1_8mm_embossed.stl` ... `holder_4_8mm_embossed.stl` | 1-4 (ArUco 40-43) | 8 mm plastic | SOT-23, SOD-123, SOD-323, small diodes / LEDs |
| `holder_5_12mm_embossed.stl` | 5 (ArUco 44) | 12 mm plastic | SOIC-8, SOP, SMA / SMB, SOT-89, SOT-223, crystals |
| `holder_6_16mm_embossed.stl` | 6 (ArUco 45) | 16 mm plastic | large parts: SOIC-14 / 16, optocouplers, bigger inductors |

**All on one plate:** `stl/plate_all.stl` (holders 1-4 paper, 5, 6, PCB corners 1-4 and HOME laid out on a 220 x 220
plate; picture: `plate.png`). Make it again: `.venv/bin/python hardware/tray/plate.py` (`--kind8 embossed`,
`--bed 256 256`).

For each of holders 1-4 print ONE of its two files (paper or plastic tape, the kind of strip you put in it).
Make the files again: `.venv/bin/python hardware/tray/smd_tray.py` (other numbers or kinds: `--set 1:8:paper
2:8:embossed 3:12`).

## How it holds every part
- The channel is 0.3 mm wider than the tape; lips 1 mm thick reach 0.8 mm over both tape edges and hold it down
  (0.15 mm room): the strip can't lift or move sideways. The window between the lips is open over the parts.
- **Any part height:** in the plastic-tape (embossed) holders the pockets hang into a groove up to 6 mm deep. To
  hold the pockets from below too, make it with the strip's pocket depth, e.g.
  `.venv/bin/python hardware/tray/smd_tray.py --set 5:12 --pocket 2.5`
- Thin paper tape (0402 / 0603, about 0.7 mm): `--set 1:8:paper --paper-t 0.7`.
- **One pick height for all:** in every holder the top of the tape (and of the parts) is 7.5 mm above the bar, and
  the parts' centre line is over the bar centre.

## Sizes (mm)
- Holders 100 long, 20 wide (holder 6: 23.6), top 8.65 above the bar. The top copies the 87 x 20 slot label, plus a
  13 mm entry at the front. Marker 10 x 10 (centre 7.5 from the back end), number next to it, part window 17-77 mm
  from the marker centre (60 mm, about 15 parts at 4 mm pitch), sprocket holes on the side the top of the number
  points to (as on the paper labels).
- Paper holders: the tape lies on a flat floor at 6.5 (1.0 mm paper tape). Plastic-tape holders: the tape edges
  rest at 7.2 (0.3 mm carrier), the groove runs from 0.5 inside the far edge to 1 mm before the hole line.
- T-rail for the measured slot A 6 / B 3.2 / C 9.5 / D 4: neck 6.2, head 9.7 x 4.2, shoulders 3.1 below the
  holder. That is 0.1 mm BIGGER than the measured slot on every side (`fit` = -0.10): the first print with 0.25
  clearance was loose, then 0.05 and another 0.3 mm bigger on every side (owner, 2026-10-12).
- Paint recesses 0.8 deep.

## Print
- White PLA, 0.4 nozzle, 0.2 layers, 3 walls, 20 % infill. **No supports.** The STLs are already upside down (top
  face on the bed: flat, sharp marker).
- Print one holder first. Too loose in the slot: lower `fit` in `smd_tray.py` (-0.10 -> -0.15); too tight: raise
  it (-0.10 -> 0.0).

## Paint the code
1. Spray the top with matt black (light coats) so the marker cells and the number are filled.
2. When dry, sand the top flat on fine sandpaper (240-400) on a flat plate until the top is white again.
   The code stays black in the recesses. The marker must be: white top, black cells, sharp edges.

## Use
1. Slide the holder into the T-slot from the end of the bar, marker towards the back (like the old labels).
2. Peel the cover film off the strip by hand. Push the strip in from the front end (the lips open upwards there),
   sprocket holes on the side the number's top points to, until it stops at the wall near the number.
3. The parts are now 6.5 mm higher than on the paper labels; the board still lies on the bar. Set
   `config/bed.yaml` `feed_tray.tape_floor_mm: 6.5`, restart the page, then set the pick height again (Use as
   PICK on a part in a holder, then Home), and check the nozzle's pick offset again: the tilted camera sees higher
   things shifted (about 0.47 mm per mm).

The software still uses the paper labels 3-7 (config/bed.yaml `feed_tray.slot_numbers: [3, 4, 5, 6, 7]`). When the
holders are on the bed: set it to `[1, 2, 3, 4, 5, 6]` and `tape_floor_mm: 6.5`, run the bed survey (page, Setup 0)
and assign the parts to the holder numbers again (page, Feeders).

# PCB corner pieces (3D printed, PCB 1-4)

Four pieces that slide into the bars like the trays (same rail) and replace the paper PCB 1-4 labels. Each carries
its marker (ArUco 30-33) and number, painted like the trays. Files: `stl/pcb_corner_1.stl` ... `pcb_corner_4.stl`,
made by `.venv/bin/python hardware/tray/pcb_corner.py` (every size is a setting at the top).

- **PCB 4 (back-left) and PCB 3 (back-right) hold the board.** Each is an L: a wall along the board's back edge
  and a wall along its side edge, 2.6 mm high (1 mm above the board). Push the board **back and left** into them:
  PCB 4 sets where the board is, PCB 3 only stops the back edge (its side wall leaves 1.5 mm room).
- **PCB 1 (front-left) and PCB 2 (front-right) mark the front corners**: the end of the arm is the corner.
- Left bar (with slot 7): PCB 1 and PCB 4. Right bar (right of HOME): PCB 2 and PCB 3. Marker towards the
  outside, arm towards the board. The left arm puts the board's left edge 29.3 mm from the bar centre, the right
  arm its right edge 22.0 mm from the bar centre (bed survey 2026-10-08, board registration 2026-10-11).

Place them:
1. Slide PCB 4 and PCB 3 in until their back walls sit 2 mm in front of the slot 7 tray (board back edge about
   95 mm in front of HOME, as before).
2. Lay the board in (B side up, KiCad top edge at the front) and push it back and left into the L of PCB 4.
3. Slide PCB 1 and PCB 2 back until their arm ends touch the board's front corners.
4. In `config/bed.yaml` set `pcb_area.corner_holders: {left_arm_mm: 29.3, right_arm_mm: 22.0, edge_mm: 8.5}`,
   restart the page and run the bed survey (Setup 0): the board area then comes from the pieces.

# HOME piece (3D printed)

`stl/home.stl` (made by `pcb_corner.py`): the HOME marker (ArUco 49) and an "H" sunk into a 20 x 27 plate, 2 mm
high, on the same rail. It replaces the paper HOME label. Paint it like the others.

1. Take the paper HOME label off. Slide the HOME piece into the same bar (the one right of holder 1, see
   `bed_layout.png`) with the "H" towards the front, and put the marker's centre where the paper marker's centre
   was (everything is measured from it: if it moves, measure the reach and run the bed survey again).
2. HOME is now 2 mm higher than the paper label: the camera calibration at every Home measures it again; run the
   bed survey (Setup 0) once.
