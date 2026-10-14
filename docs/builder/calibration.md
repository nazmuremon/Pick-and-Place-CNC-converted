# Calibration: how it works (for builders)

## One camera model (D28)
A pinhole with radial lens bending (k1, k2) and a free tilt, fixed to the head. Its reference point is the bed point
at the picture centre. A printed ChArUco sheet seen from 9 head positions on the bed and 9 on a block of known height
gives the lens, the tilt, the height and the head's own move scale (a steps/mm check) in one fit. Every height (tape,
board, part top) is a plane of the same model: a tilted camera sees a thing h mm higher about tan(tilt)·h mm shifted,
and the model accounts for it everywhere.

## The validity chain (D32)
Every result is saved (data folder `calibration*.yaml`) with its inputs: hardware ids from the profile (camera,
nozzle, rotation, homing) and the dates of the results it was built on. Measure the camera again and everything built
on it (survey, nozzle offset, angle table, first part) shows "stale" with the reason. Every save keeps the last five
values for Undo; a copy of the file is kept before every write (`config_history`).

| Result | Made by | Depends on |
|---|---|---|
| camera_head, exposure_head | Setup 3 | camera id |
| travel | Setup 2 | homing method |
| bed_survey | Setup 8 | camera id, homing, camera_head |
| nozzle_offset (axis, runout, phase) | Setup 5 | camera, nozzle, rotation, camera_head |
| rotation_table (pulse -> angle, repeat) | Setup 6 | rotation, nozzle, camera_head |
| first_part (dx, dy, dθ) | Setup 10 | nozzle, rotation, nozzle_offset |
| board_registration | Board tab | camera, board, the controller session |

## Heights are derived (D33)
pick = tray tape floor + part height − press; place = board top + part height − press; moving = the highest thing +
the part on the nozzle + clearance. Z goes down by itself only over a known surface (a tray zone, the registered
board, the mark sheet).

## Ink marks (D34)
The needle touches a paper sheet at 5 servo angles; each mark is measured from straight above. The marks lie on a
circle: its centre is the servo axis, its radius the needle's runout. With the needle ≥ 0.5 mm off the axis, a mark's
direction also measures the servo's real angle: the pulse -> angle table and its repeat.

## Checks in a job (D35, D39)
The pick check matches the pocket against the confirmed template (part still there = missed). The place check pastes
the part's own picture into the "before" picture at candidate positions and angles and keeps the one that explains
the "after" picture best.
