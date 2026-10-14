# The Setup tab, step by step

Every step shows: **valid** (measured, still true), **stale** (something it depends on changed: measure it again) or
**missing**. "How" opens what you do, what the machine does, and when the step is done. A step is grey until the
steps it needs are valid. **Undo** puts the value before the last one back.

1. **Connect and check.** Connect, then "Test the outputs": the pump clicks on and off, the nozzle turns a little.
2. **Travel.** Home first. For each end: "Creep X+" (and the others) moves 2 mm at a time and checks every step with
   the camera; keep a hand near STOP (Space). It stops at the end and keeps the end a little inside. Home again, then
   "Save the travel". Or jog there yourself and press "End here".
3. **Camera.** Lights on, "Find and lock the exposure". Print the calibration sheet at 100 % (both bars must measure
   100.0 mm), lay it flat with room around it, "Sheet on the bed"; put it on a block you measured with a caliper, type
   the height, "Sheet on the block"; take it away, "Solve and save".
4. **Home and Z.** Jog until the HOME label is in view, Home. Then the touch-off: go down with the close-up on the
   right (1, then 0.1, then 0.02 mm steps) until the tip just touches the label, "Set Z here".
5. **Nozzle offset.** A sheet of white paper under the nozzle (the blue ring on the picture), type its thickness, a
   little ink on the needle, "Make marks". Without ink: "The tip is on it" with the tip on a sharp corner, lift, then
   "Then click the spot" and click that corner on the picture.
6. **Rotation.** The same sheet: "Measure the angle table". It also measures how well the servo repeats.
7. **Heights.** Nothing to do: check the numbers.
8. **Bed survey.** The trays where the bed map shows them, "Survey".
9. **Board.** Import it (Board tab), lay it in the board area, choose the side up and the top edge, "Register". The
   pads drawn on the live picture must sit on the real pads.
10. **First part.** A strip of resistors or capacitors in a tray; "Look at the slot", say Yes if it is a part; choose a
    board part of that package, "Place it". Look at the photos: OK, or type how far to move it.

**Steps that appear only with that hardware:** 2 becomes "Homing switches" (switches: Home 3 times, the travel from
the controller); 5b "Bottom camera" (the bare needle over it, sharp, then Measure); 6 "Rotation axis" for a stepper
(scale, backlash, repeat by ink marks); 6b "Vacuum sensor" (Open, then Closed with the tip pressed on the paper).
With more than one nozzle, step 5 (and 5b) is done for each nozzle: put it on, "Nozzle changed" (Job tab), measure.

After a controller reset (power, USB) the machine locks: lift the nozzle (Z up), "Nozzle is clear", then step 4
again. Nothing else needs measuring again.
