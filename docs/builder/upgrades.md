# Upgrade paths (docs/13 §4.14)

Each upgrade is a profile change plus its Setup step; the job and the calibration code do not change. Status
2026-10-14 (Phase G built, in the simulator):

| Upgrade | Profile change | Setup step | Program status |
|---|---|---|---|
| XY limit switches | `homing.method: switches`, `motion.travel.frame: controller` | 2: "Homing switches" (3 homes, the HOME label measures the repeat, travel from $130/$131) | works (simulated) |
| Z probe | `z_reference.method: probe` (feed, depth, retract) | 4: "Probe Z" | works (simulated) |
| Stepper rotation | `rotation.type: stepper_axis` (letter, feed) | 6: "Rotation axis" (scale, zero, backlash, repeat by ink marks) | works (simulated) |
| Bottom camera | `cameras.bottom` (v4l2 or picamera2) | 5b: "Bottom camera" | works (simulated); picamera2 not tested on a Pi |
| Light control | `lights.head` (a controller output, e.g. `M62 P0` / `M63 P0` on grblHAL) | 3: "Find and lock" measures both light states | works (simulated) |
| Valve | `vacuum.type: pump_valve`, `valve` | — | works (simulated) |
| Vacuum sensor | `vacuum.type: pump_valve_sensor`, `sensor` | 6b: "Vacuum sensor" (open / closed) | the logic works (simulated); HOW the controller reports the input is TBD |
| More nozzles (manual change) | `nozzles.items[]` with their `packages` | 5 (and 5b) for each nozzle | works (simulated): the job pauses for each change |
| Nozzle rack | `nozzles.change: rack` | — | TBD: positions and change moves depend on the rack |
| grblHAL / Pico | `controller.type: grblhal` | 1: Connect check | works (same G-code; simulated) |
| More trays | "Add tray" (Feeders tab) | 8: Survey | works |
| Matrix tray, stick | "Add tray" with the grid / pick point and its two heights | 8: Survey | works (simulated) |
| Motorized reel feeder | — | — | TBD: needs the feeder hardware |
| Panels | Board tab: copy 2, 3, … with its centre | 9 for each copy | works (simulated) |
