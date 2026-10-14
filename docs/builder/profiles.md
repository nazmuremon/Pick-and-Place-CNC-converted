# Machine profiles (for builders)

Everything special about a machine is in ONE file, `config/profiles/<name>.yaml` (schema: `src/pnp/profile/schema.py`,
checked with the field and the line of every error). `null` = TBD: the capability that needs it is off and the Doctor
and the Setup tab say what to measure. Never put a number in code: put it in the profile, or let a Setup step measure it.

| Section | What | Notes |
|---|---|---|
| `controller` | type (grbl / grblhal), port (a device, `socket://` for the port keeper), firmware check, `axis_map` (machine X / Y / Z -> controller letter and sign), `expected_settings` | the axis map is how the bench lives with swapped motors |
| `motion` | feeds, slow jog before Home, largest move, `z_clearance_mm` (moving height above the highest thing), `travel` (frame home / controller; the measured travel of Setup step 2 replaces it) | |
| `homing` | camera_marker / switches / manual | switches need `travel.frame: controller` |
| `z_reference` | touch_off / probe (feed, depth, retract) / switch / none | |
| `rotation` | none / servo_pwm (pulses, settle, approach, accuracy) / stepper_axis | the angle table of Setup step 6 replaces the nominal range |
| `vacuum` | pump_only / pump_valve / pump_valve_sensor; outputs by role as G-code lines; `release_s`, `pick_dwell_s` | outputs only through the controller (D3) |
| `lights`, `cameras` | the head camera (driver, device, resolution, `hardware_id`, mounting), a bottom camera | change `hardware_id` when the camera, lens or mount changes: its calibrations become stale |
| `nozzles` | ids, OD, `press_mm`, `spring_mm`, `offset_guess_mm` | a new nozzle id makes the offset stale |
| `job` | place / angle tolerance, pick attempts, HOME check interval, drift limit, photo retention | the brief's targets (docs/13 §8) |
| `sim` | the simulator's own machine: frame ends, the TRUE camera, nozzle offset, runout, spring, servo play / scale | guesses are fine: nothing real moves |

Two examples ship with the program: `bench_uno` (the test bench) and `pico_grblhal` (the planned machine, mostly TBD).
`pnp init` writes a new one from questions; `pnp profile check NAME` validates one.
