"""The calibration wizard (docs/13 §4.6): ONE ordered list of steps, built from the profile's capabilities.

Every step says: its status (valid / stale / missing, with the reason and the date), what you do, what the machine
does, when it is done, which actions it has, and whether Undo can put the last value back. A step whose earlier
steps are not valid is blocked (greyed out on the page).

The steps follow the capabilities (§4.14): a probe replaces the touch-off, switches replace the travel creep, a
servo has an angle table, a stepper axis its own step (Phase G), no rotation = no step 6.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import TYPE_CHECKING

from . import chain

if TYPE_CHECKING:
    from ..machine import Machine


@dataclass
class Action:
    id: str                          # the machine action: /api/action/setup {step, action, ...params}
    label: str
    params: list[str] = field(default_factory=list)    # what the page asks for (names of inputs)
    danger: bool = False             # it moves Z down by itself: shown in red


@dataclass
class Step:
    id: str
    n: int
    title: str
    you_do: str
    machine_does: str
    done_when: str
    state: str = "missing"           # valid | stale | missing | later | absent
    why: str = ""
    date: str | None = None
    needs: list[str] = field(default_factory=list)
    blocked_by: list[str] = field(default_factory=list)
    actions: list[Action] = field(default_factory=list)
    undo: str | None = None          # the calibration key Undo restores
    info: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return asdict(self)


def _chain_state(m: Machine, key: str) -> tuple[str, str, str | None]:
    st = chain.status(m, key)
    return st["state"], st["why"], st.get("date")


def build(m: Machine) -> list[Step]:
    p = m.profile
    caps = m.caps
    steps: list[Step] = []

    # 1 ------------------------------------------------------------ connect and check
    s = Step("connect", 1, "Connect and check",
             "Plug in the controller and the camera, press Connect. Then try the outputs: the pump clicks on and off, "
             "the nozzle turns a little.",
             "Reads the controller's version and settings, compares them with the profile, lists what is still TBD.",
             "Connected, the firmware and the settings match, the outputs answer.")
    ctl = m.ctl
    if not m.safety.connected or ctl is None:
        s.state, s.why = "missing", "not connected"
    elif not ctl.info.firmware_ok:
        s.state, s.why = "stale", "the controller firmware is not the profile's"
    elif ctl.info.mismatched:
        s.state, s.why = "stale", "settings differ: " + ", ".join(f"{k}={v[1]} (want {v[0]})"
                                                                for k, v in ctl.info.mismatched.items())
    else:
        s.state = "valid"
    s.actions = [Action("connect", "Connect"), Action("outputs_test", "Test the outputs")]
    s.info = {"tbd": [x.field for x in __import__("pnp.profile", fromlist=["missing"]).missing(p)]}
    steps.append(s)

    # 2 ------------------------------------------------------------ travel
    if p.homing.method == "switches":
        s = Step("travel", 2, "Homing switches",
                 "Home once (Machine tab). Jog until the HOME label is in view, then press 'Test the switches'.",
                 "Homes with the switches 3 times; after each, the camera measures the HOME label (how well the "
                 "switches repeat). The travel comes from the controller's max travel ($130 / $131).",
                 "Home works 3 times in a row; the travel is saved.", needs=["connect"])
        s.state, s.why, s.date = _chain_state(m, "travel")
        s.actions = [Action("switch_test", "Test the switches")]
        s.undo = "travel"
    else:
        s = Step("travel", 2, "Travel (soft limits)",
                 "Home first. Then for each end: press its 'Creep' button and watch (STOP if anything looks wrong), "
                 "or jog there yourself and press 'End here'. Home again after an end was found. Save.",
                 "Moves 2 mm at a time toward the end and checks every step with the camera. The first step the picture "
                 "does not show is the end; it keeps the end one step and 2 mm inside.",
                 "All four ends found and saved. The travel never shrinks by itself later.", needs=["connect"])
        s.state, s.why, s.date = _chain_state(m, "travel")
        if s.state == "missing" and m.limits.xy_known:
            s.state, s.why = "stale", "from the profile, not measured with this program"
        s.actions = [Action("creep", f"Creep {d}", ["dir"]) for d in ("X-", "X+", "Y-", "Y+")] + \
                    [Action("end_here", "End here", ["dir"]), Action("save_travel", "Save the travel")]
        s.undo = "travel"
        s.info = m.setup.travel_now()
    steps.append(s)

    # 3 ------------------------------------------------------------ camera
    s = Step("camera", 3, "Camera: exposure, lens, tilt, heights",
             "Lights on. 'Find and lock' the exposure. Print the ChArUco sheet at 100 % (both bars 100.0 mm), lay it flat "
             "where the head has room, press 'Sheet on the bed'; put it on a measured block, 'Sheet on the block'; "
             "take it away, 'Solve and save'.",
             "Finds a good exposure and keeps it fixed. Takes 9 pictures of the sheet per height and fits the camera: lens, "
             "tilt, height, and checks the head moves (steps/mm).",
             "Saved with a fit error under 1 px and a head-move scale within 0.5 % of 1.", needs=["connect"])
    st_c, why_c, date_c = _chain_state(m, "camera_head")
    st_e, why_e, _ = _chain_state(m, "exposure_head")
    s.state, s.why, s.date = st_c, why_c, date_c
    if st_c == "valid" and st_e != "valid" and not (p.cameras.head and p.cameras.head.exposure.value is not None):
        s.state, s.why = "stale", "the exposure is not locked: " + why_e
    s.actions = [Action("auto_exposure", "Find and lock the exposure"), Action("calibrate_bed", "Sheet on the bed"),
                 Action("calibrate_block", "Sheet on the block", ["block_mm"]), Action("calibrate_solve", "Solve and save")]
    s.undo = "camera_head"
    if p.cameras.head is None:
        s.state, s.why = "absent", "no head camera in the profile"
    steps.append(s)

    # 4 ------------------------------------------------------------ home and Z
    zk = caps.z_reference.kind
    s = Step("home_z", 4, "Home (X / Y) and the Z reference",
             "Jog until the HOME label is in view, press Home. " + (
                 "Then the probe: press 'Probe Z' over the HOME label." if zk == "probe" else
                 "Then jog the nozzle down onto the HOME label surface (1, then 0.1, then 0.02 mm steps, watch the "
                 "close-up) until it just touches, and press 'Set Z here'."),
             "Measures HOME in the picture. Keeps the Z of the touch as Z 0 (the label surface). After a controller "
             "reset it asks only for this step again.",
             "Homed and Z set.", needs=["camera"] if p.homing.method == "camera_marker" else ["connect"])
    s.state = "valid" if m.safety.homed and m.safety.z_ref else "missing"
    s.why = "" if s.state == "valid" else ("not homed" if not m.safety.homed else "Z has no reference")
    s.actions = [Action("home", "Home"), Action("set_z", "Set Z here" if zk != "probe" else "Probe Z", ["surface_z"])]
    steps.append(s)

    # 5 ------------------------------------------------------------ nozzle offset
    s = Step("nozzle_offset", 5, "Nozzle offset" + (f" (nozzle {m.nozzle_current()})" if len(p.nozzles.items) > 1 else ""),
             "Lay a sheet of white paper on the bed under the nozzle (the ring on the picture), type its height (paper: "
             "measure it, ~0.1 mm), put a little ink on the needle tip, press 'Make marks'. No ink? Use 'Pointer by "
             "eye': jog the tip onto a sharp corner, press 'The tip is on it', lift, then click that corner on the picture.",
             "Touches the sheet at several servo angles and measures each mark from above it. The circle of the marks "
             "gives the servo axis and how far the needle is off it.",
             "Saved; the marks fit their circle within 0.05 mm. A change of more than 1 mm waits for your yes.",
             needs=["home_z"])
    s.state, s.why, s.date = _chain_state(m, "nozzle_offset")
    s.actions = [Action("nozzle_marks", "Make marks", ["sheet_mm"], danger=True),
                 Action("pointer_tip", "The tip is on it"), Action("pointer_click", "Click the spot on the picture")]
    if "nozzle_offset" in m.setup.pending:
        s.actions.append(Action("accept", "Accept the new offset"))
        s.info["pending"] = {k: v for k, v in m.setup.pending["nozzle_offset"].items() if k != "value"}
    s.undo = "nozzle_offset"
    steps.append(s)

    # 5b ----------------------------------------------------------- bottom camera (only with one, Phase G)
    if p.cameras.bottom is not None:
        s = Step("bottom", "5b", "Bottom camera",
                 "Jog the bare needle (no part) over the bottom camera and down until the needle is sharp in the "
                 "bottom picture. Press 'Measure'.",
                 "Moves 1 mm in X and in Y (lifting in between) to learn the picture's scale, turn and mirror, then "
                 "centres the needle; the nozzle offset gives where the camera looks.",
                 "Saved. From then on every part is looked at from below before it is placed: its real centre and "
                 "angle on the needle are corrected.", needs=["nozzle_offset"])
        s.state, s.why, s.date = _chain_state(m, "bottom_camera")
        s.actions = [Action("bottom_calibrate", "Measure")]
        s.undo = "bottom_camera"
        steps.append(s)

    # 6 ------------------------------------------------------------ rotation
    rk = caps.rotation.kind
    if rk == "none":
        s = Step("rotation", 6, "Rotation", "", "", "", state="absent", why="this nozzle does not turn")
    elif rk == "stepper_axis":
        s = Step("rotation", 6, "Rotation axis (stepper)",
                 "The same sheet, with ink. Press 'Measure the axis'.",
                 "Marks at 5 angles give the axis's real turn (steps per degree); 0° from below and from above gives "
                 "the backlash; 5 marks at 0° the repeat. From then on every turn comes from below.",
                 "Saved; the repeat is within the profile's tolerance.", needs=["nozzle_offset"])
        s.state, s.why, s.date = _chain_state(m, "rotation_table")
        s.actions = [Action("rotation_marks", "Measure the axis", ["sheet_mm"], danger=True)]
        s.undo = "rotation_table"
        s.info = {"accuracy_deg": getattr(caps.rotation, "accuracy_deg", None)}
    else:
        s = Step("rotation", 6, "Rotation: the servo's angle table",
                 "The same sheet, with ink. Press 'Measure the angle table'.",
                 "Makes a mark at 7 pulses and 5 marks at the centre pulse; the marks' direction around the servo axis is "
                 "the servo's real angle. Then the repeat test.",
                 "Saved. When the repeat is worse than the profile's tolerance, the job turns parts only where the pads "
                 "allow (0° / 180°).", needs=["nozzle_offset"])
        s.state, s.why, s.date = _chain_state(m, "rotation_table")
        s.actions = [Action("rotation_marks", "Measure the angle table", ["sheet_mm"], danger=True)]
        s.undo = "rotation_table"
        s.info = {"accuracy_deg": getattr(caps.rotation, "accuracy_deg", None)}
    steps.append(s)

    # 6b ----------------------------------------------------------- vacuum sensor (only with one, Phase G)
    if p.vacuum.sensor is not None:
        s = Step("vacuum", "6b", "Vacuum sensor",
                 "Needle in the air: 'Open'. Then press the tip on the paper (or hold a part on it): 'Closed'.",
                 "Reads the sensor with the pump on, both ways; a part counts as held above half way. The pick "
                 "check then asks the sensor first, the camera second.",
                 "Saved: closed and open are clearly apart.", needs=["connect"])
        s.state, s.why, s.date = _chain_state(m, "vacuum_sensor")
        if getattr(caps.vacuum, "reader", None) is None:
            s.state, s.why = "later", "how your controller reports the sensor is not known yet (TBD)"
        s.actions = [Action("vacuum_open", "Open"), Action("vacuum_closed", "Closed")]
        s.undo = "vacuum_sensor"
        steps.append(s)

    # 7 ------------------------------------------------------------ heights
    s = Step("heights", 7, "Heights (derived)",
             "Nothing to type. Check the numbers; if one looks wrong, fix its source (press and clearance in the profile, "
             "the board's thickness, the tray's tape floor).",
             "Works out every height from the Z reference: pick = part top − press, place = board top + part − press, "
             "moving = the highest thing + clearance.",
             "All sources known.", needs=["home_z"])
    try:
        from ..job.pickplace import height_rules
        board = m.boards.current()
        rules = height_rules(m, board)
        s.info = rules.as_dict(1.35)
        s.state = "valid" if m.safety.z_ref else "stale"
        s.why = "" if m.safety.z_ref else "Z has no reference (step 4)"
    except Exception as err:                                   # a TBD number: say which
        s.state, s.why = "missing", getattr(getattr(err, "refusal", None), "why", str(err))
    steps.append(s)

    # 8 ------------------------------------------------------------ bed survey (feeders)
    s = Step("survey", 8, "Bed survey (trays and labels)",
             "Put the trays / labels where the bed map shows them. Press 'Survey'.",
             "Drives to every label of the plan and measures where it is and which way it points.",
             "Every slot label found.", needs=["camera", "home_z"])
    s.state, s.why, s.date = _chain_state(m, "bed_survey")
    s.actions = [Action("survey", "Survey")]
    s.undo = "bed_survey"
    steps.append(s)

    # 9 ------------------------------------------------------------ board
    s = Step("board", 9, "Board: import, side, registration",
             "Import the board (Board tab). Lay it in the board area. Choose the side up and the top edge, press Register. "
             "Check the drawn pads on the live picture.",
             "Measures the holes and corners it can reach and fits the board's position and turn.",
             "Registered with a fit residual ≤ 0.1 mm RMS (no feature over 0.2 mm); the drawn pads sit on the pads.", needs=["camera", "home_z"])
    board, reg = m.vision.registration()
    if m.boards.current() is None:
        s.state, s.why = "missing", "no board imported"
    elif reg is None:
        s.state, s.why = "missing", "not registered (or the registration is stale)"
    else:
        s.state = "valid" if reg.rms <= 0.1 and reg.max <= 0.2 else "stale"
        s.why = "" if s.state == "valid" else f"residual {reg.rms:.2f} mm RMS, worst {reg.max:.2f} mm: check the features"
        s.info = {"board": board.name, "side": reg.side, "rms": round(reg.rms, 3), "max": round(reg.max, 3)}
    s.actions = [Action("register", "Register", ["side", "top_edge"])]
    steps.append(s)

    # 10 ----------------------------------------------------------- first part
    s = Step("first_part", 10, "First part",
             "Lay a strip of a chip part (R or C) in a slot. Press 'Look at the slot' and say Yes if the picture shows a "
             "part. Choose a board part of that package, press 'Place it'. Look at the before / after photos: OK, or type "
             "how far to move it (dx, dy in mm, dθ in degrees).",
             "Picks the part (checks the pocket is empty after), turns it, places it, and measures where it landed.",
             "You pressed OK. The correction is used by every job.",
             needs=["nozzle_offset", "heights", "survey", "board"])
    s.state, s.why, s.date = _chain_state(m, "first_part")
    s.actions = [Action("look_at_slot", "Look at the slot", ["slot", "package"]),
                 Action("confirm", "Yes, a part", ["yes"]),
                 Action("first_part", "Place it", ["slot", "ref"], danger=True),
                 Action("first_part_answer", "OK / move it", ["ok", "dx", "dy", "dtheta"])]
    s.undo = "first_part"
    if m.setup.candidate:
        s.info["candidate"] = {"slot": m.setup.candidate["slot"], "image": "/api/setup/candidate.png"}
    if m.setup.last_first_part:
        s.info["last"] = m.setup.last_first_part
    steps.append(s)

    # dependencies: a step is blocked while an earlier step it needs is not valid
    by_id = {x.id: x for x in steps}
    for x in steps:
        x.blocked_by = [n for n in x.needs if n in by_id and by_id[n].state not in ("valid", "absent")]
    return steps
