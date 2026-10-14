"""What a profile can't do yet: every TBD (null) field that switches a capability off, in plain words."""
from __future__ import annotations

from dataclasses import asdict, dataclass

from .schema import Profile


@dataclass(frozen=True)
class Missing:
    field: str          # the profile field that is null
    effect: str         # what the machine refuses because of it
    fix: str            # what to do (where)

    def as_dict(self) -> dict:
        return asdict(self)


def missing(p: Profile) -> list[Missing]:
    out: list[Missing] = []
    t = p.motion.travel
    if t.x is None or t.y is None:
        out.append(Missing("motion.travel.x / y", "No go-to and no automatic X / Y moves: only slow jogs",
                           "Measure the travel (Setup, step 2) or write it in the profile"))
    if t.z is None:
        out.append(Missing("motion.travel.z", "Z has no limits: Z moves only by your own jogs",
                           "Measure the Z travel (Setup, step 2)"))
    if p.rotation.type == "servo_pwm":
        s = p.rotation.servo_pwm
        if s.settle_s is None:
            out.append(Missing("rotation.servo_pwm.settle_s", "The nozzle does not turn",
                               "Measure how long the servo needs to stop (Setup, step 6)"))
        if s.nominal_range_deg is None:
            out.append(Missing("rotation.servo_pwm.nominal_range_deg", "No angle for a pulse until the angle table is "
                               "measured", "Write the servo's range from its datasheet, or measure the table"))
        if s.accuracy_deg is None:
            out.append(Missing("rotation.servo_pwm.accuracy_deg", "The planner can't tell which parts need a "
                               "better rotation axis", "Run the repeat test (Setup, step 6)"))
    if p.vacuum.type == "pump_only" and p.vacuum.release_s is None:
        out.append(Missing("vacuum.release_s", "No release wait: parts may stay on the needle",
                           "Set a release wait, or add a valve"))
    if p.vacuum.pick_dwell_s is None:
        out.append(Missing("vacuum.pick_dwell_s", "No picks: the time the pump needs at the part is not known",
                           "Write how long the nozzle stays down for a pick"))
    if p.cameras.head is None:
        out.append(Missing("cameras.head", "No camera: no homing on the HOME label, no vision",
                           "Add the head camera to the profile"))
    elif not p.cameras.head.exposure.auto and p.cameras.head.exposure.value is None:
        out.append(Missing("cameras.head.exposure.value", "The picture brightness changes with the light",
                           "Run Auto exposure (Machine tab)"))
    if p.cameras.head is not None and (p.cameras.head.moves_with_z is None or p.cameras.head.tilted is None):
        out.append(Missing("cameras.head.moves_with_z / tilted", "The camera calibration can't choose its method",
                           "Write how the camera is mounted"))
    if p.homing.method == "camera_marker" and p.cameras.head is None:
        out.append(Missing("homing.method", "Camera homing needs a head camera", "Add cameras.head"))
    if p.motion.z_clearance_mm is None:
        out.append(Missing("motion.z_clearance_mm", "No automatic Z moves: the moving height can't be derived",
                           "Write how far above the pick / place heights the nozzle travels (docs/08)"))
    cur = next(n for n in p.nozzles.items if n.id == p.nozzles.current)
    if cur.press_mm is None:
        out.append(Missing(f"nozzles.items[{cur.id}].press_mm", "No pick, place or ink mark: the press is not known",
                           "Write how far below the touch the tip may go (spring nozzle, docs/08)"))
    if cur.spring_mm is None:
        out.append(Missing(f"nozzles.items[{cur.id}].spring_mm", "The press can't be checked against the spring",
                           "Measure how far the nozzle's spring can be pressed"))
    if any(n.od_mm is None for n in p.nozzles.items):
        out.append(Missing("nozzles.items[].od_mm", "Pick checks can't use the needle size",
                           "Measure the needle's outer diameter"))
    return out
