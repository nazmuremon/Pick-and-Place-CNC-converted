# SMD tray (3D printed, one per T-slot)

A tray that slides into a T-slot of the bed bar and holds one cut-tape strip. When you push the strip in, a PLA
blade peels the cover film off. The slot's marker and number are sunk into the top: spray paint them black.

Files: `smd_tray.py` (the design, every size is a setting at the top) and `stl/smd_tray_slot1.stl` ... `slot6.stl`
(trays 1 ... 6: marker ArUco 40 ... 45 = slot 1 ... 6). Make new STLs: `.venv/bin/python hardware/tray/smd_tray.py`

## Sizes (mm)
- Tray 100 x 20, top 3.65 above the bar. The top copies the 87 x 20 slot label, plus a 13 mm entry with the blade.
- Marker 10 x 10 (centre 7.5 from the back end), number next to it, part window 17-77 mm from the marker centre
  (60 mm, about 15 parts at 4 mm pitch), parts on the centre line, sprocket holes on the side the top of the
  number points to (as on the paper labels: the software finds the parts the same way).
- Tape channel for 8 mm paper tape, 1.0 mm thick (+0.25 room), lips 0.8 over both tape edges.
- T-rail for the measured slot A 6 / B 3.2 / C 9.5 / D 4 with 0.25 clearance: neck 5.5, head 9.0 x 3.5.
- Paint recesses 0.8 deep.

## Print
- White PLA, 0.4 nozzle, 0.2 layers, 3 walls, 20 % infill. **No supports.**
- The STLs are already upside down: the top face lies on the bed (flat, sharp marker and blade).
- Test the fit with one tray first. Too tight in the slot: raise `fit` (0.25 -> 0.3); too loose: lower it.

## Paint the code
1. Spray the top with matt black (light coats) so the marker cells and the number are filled.
2. When dry, sand the top flat on fine sandpaper (240-400) on a flat plate until the top is white again.
   The code stays black in the recesses. The marker must be: white top, black cells, sharp edges.

## Use
1. Slide the tray into the T-slot from the end of the bar, marker towards the back (like the old labels).
2. Peel back 3-4 cm of the cover film by hand. Push the tape end into the entry, **under** the blade, with the
   sprocket holes on the side the number's top points to. Lay the loose film **over** the blade, back towards you.
3. Hold the film end lightly (or clip it) and push the strip in until it stops at the wall near the number:
   the blade peels the film off as the strip goes in.
4. Parts are now higher than on the paper labels (tape floor 1.6 mm above the bar): set the pick height again
   (Use as PICK on a part in the tray, then Home).

The software still uses the paper labels 3-7 (config/bed.yaml `feed_tray.slot_numbers: [3, 4, 5, 6, 7]`). When the
trays are on the bed: set it to `[1, 2, 3, 4, 5, 6]`, run the bed survey (page, Setup 0) and assign the parts to the
tray numbers again (page, Feeders).
