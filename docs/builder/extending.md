# Extending the program (for builders)

## Add a PCB importer
`src/pnp/board/`: an importer reads files into the one `Board` model (`model.py`: mm, X right, Y up, origin = the
outline box's bottom-left; a part's angle CCW seen from its own side, library zero; the bottom view turned over
left-right). Add it to `service.py`'s detection, write a sample file into `tests/data/boards/` and a test that it
imports the HT5023 the same as KiCad (all parts within 0.05 mm, the same rotations). Never trust a tool's "SMD" flag:
the pads decide.

## Add a package
`config/packages.user.yaml` (read first, so it can also change a shipped entry): body, height, pads, polar, pickable
(yes / bottom_camera / by_hand), tape width and pitch, `tape_angle` (the part's angle in the pocket), aliases and
patterns. A polar part without `tape_angle` is skipped by the job (with that reason).

## Add a tray / feeder type
Trays are a list (`config/feeders.yaml`, `src/pnp/feeders/bank.py`). A new strip tray needs no code: "Add tray" in the
Feeders tab. A new KIND (a reel feeder, a stick, a matrix tray) is a class with the Feeder interface
(`capabilities/later.py`: `next_part()`, `remaining`, `refill()`), its simulator version, and its wizard step. The
printed holder generator is `src/pnp/feeders/traygen.py` (numbers 1-99, 8 / 12 / 16 / 24 mm tape, any marker).

## Add a hardware capability
`src/pnp/capabilities/`: every interface has a real, a `none` and a simulator implementation; the profile says which
is plugged in (`capabilities/__init__.py: build`). The job and the wizard ask the capability (`availability()`,
`accuracy_deg`, `can_sense`...), never the hardware name. Add the simulator version (`src/pnp/sim/`) and an end-to-end
test, so the change is a profile change plus one wizard step.

## Tests
`.venv/bin/python -m pytest -q`. The v2 tests (`tests/v2/`) run against the simulator only and use frozen copies of
the settings (`tests/data/`): calibrating the real machine never breaks them (D25).
