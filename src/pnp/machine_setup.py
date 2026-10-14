"""The set-up procedures of the calibration wizard (docs/13 §4.6), run on the machine's worker thread:
travel (step 2), nozzle offset (5), rotation table (6), bed survey (8) and the first part (10). Camera (3) and
board registration (9) are in machine_vision.py, Home and Z (4) in machine.py.

Every result is saved through the validity chain (calibration/chain.py) and can be undone.
"""
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from .calibration import chain
from .calibration.nozzle import Mark, NozzleOffset, RotationTable, fit_offset, fit_table, repeat_spread
from .capabilities.base import CapabilityError
from .motion.state import Refusal, Refused
from .vision.marks import find_change

if TYPE_CHECKING:
    from .machine import Machine

TRAVEL_MARGIN = 2.0              # mm: a measured end is kept this far inside (docs/13 §4.6 step 2)
CREEP_STEP = 2.0                 # mm per stall-checked step toward an end
CONFIRM_OVER_MM = 1.0            # an offset change bigger than this needs the operator's yes (bug 5.5)
MARK_SPACING = 2.5               # mm between two marks on the sheet (rows run to the front: they fit between trays)


class SetupOps:
    def __init__(self, m: Machine) -> None:
        self.m = m
        self.ends: dict[str, float] = {}          # travel ends found in this session: "X-", "X+", "Y-", "Y+"
        self.pending: dict[str, dict] = {}        # results waiting for the operator's yes (big changes)
        self.pointer: dict | None = None
        self.candidate: dict | None = None        # the first part of a slot, waiting for yes / no
        self.templates: dict[int, np.ndarray] = {}
        self.last_first_part: dict | None = None

    # ================================================================ step 2: travel
    def travel_now(self) -> dict:
        t = self.m.limits.travel
        return {"x": list(t["X"]) if t["X"] else None, "y": list(t["Y"]) if t["Y"] else None,
                "source": self.m.limits.source, "found": dict(self.ends)}

    def creep(self, axis: str, sign: int) -> None:
        """Toward one end in 2 mm steps; after every step the camera must see the move. The first step it does not
        see is the end: the end is kept 2 mm inside, and the position is not trusted any more (Home again)."""
        m = self.m
        if axis not in ("X", "Y") or sign not in (-1, 1):
            raise ValueError("axis X / Y, sign ±1")
        if not m.safety.homed:
            raise Refused(Refusal("Not homed", "Home first: the ends are measured from HOME"))
        self._z_clear()
        unknown, moved = 0, 0.0
        while True:
            if m.worker.cancel.is_set():
                m.say("Travel: stopped by you", "warn", "setup")
                return
            pos = m.position()
            last_good = pos[axis]
            delta = {axis: sign * CREEP_STEP}
            verdict = m._move(delta, None, m.limits.slow_feed, stall_check=True, on_stall="return")
            if verdict == "stopped":
                return
            if verdict == "stall":
                # the step before may have been partly blocked and still looked like a move: one more step inside
                end = last_good - sign * (CREEP_STEP + TRAVEL_MARGIN)
                self.ends[f"{axis}{'+' if sign > 0 else '-'}"] = round(end, 2)
                m.safety.homed = False
                m._save()
                m.say(f"Travel: the {axis}{'+' if sign > 0 else '-'} end is near {last_good:.1f} mm; kept {end:.1f} mm "
                      "(one step + 2 mm inside). The head lost steps there: jog back and Home again", "warn", "setup")
                return
            moved += CREEP_STEP
            unknown = unknown + 1 if verdict == "unknown" else 0
            if unknown >= 5:
                m.say("Travel: the picture can't show the moves here (long bars along the move?). Stopped: jog "
                      "across the bars a little and go on, or press 'End here' when the head is at the end", "warn",
                      "setup")
                return
            if moved > 500:
                m.say("Travel: 500 mm and no end: stopped", "warn", "setup")
                return

    def end_here(self, axis: str, sign: int) -> None:
        """The operator says: the head is as far as it may go here."""
        if not self.m.safety.homed:
            raise Refused(Refusal("Not homed", "Home first"))
        v = self.m.position()[axis]
        self.ends[f"{axis}{'+' if sign > 0 else '-'}"] = round(v, 2)
        self.m.say(f"Travel: {axis}{'+' if sign > 0 else '-'} end set here ({v:.1f} mm)", kind="setup")

    def save_travel(self) -> None:
        cur = self.travel_now()
        x, y = list(cur["x"] or [None, None]), list(cur["y"] or [None, None])
        for k, v in self.ends.items():
            tgt = x if k[0] == "X" else y
            tgt[0 if k[1] == "-" else 1] = v
        if None in x or None in y or x[0] >= x[1] or y[0] >= y[1]:
            raise Refused(Refusal(f"The travel is not complete: X {x}, Y {y}", "Find all four ends first"))
        old = chain.value(self.m, "travel")
        if old is not None and (x[0] > old["x"][0] + 0.5 or x[1] < old["x"][1] - 0.5 or y[0] > old["y"][0] + 0.5
                                or y[1] < old["y"][1] - 0.5):
            self.m.say("Travel: the new travel is smaller than the old one somewhere: saved, but check the ends "
                       "(Undo puts the old one back)", "warn", "setup")
        chain.put(self.m, "travel", {"x": x, "y": y}, "ends found with the stall-checked creep / by the operator",
                  {"ends": dict(self.ends)})
        self.ends.clear()
        self.m.refresh_limits()
        self.m.say(f"Travel saved: X {x[0]:.1f} … {x[1]:.1f}, Y {y[0]:.1f} … {y[1]:.1f} mm", kind="setup")

    # ================================================================ step 5: nozzle offset
    def offset_estimate(self) -> NozzleOffset | None:
        v = chain.entry(self.m, "nozzle_offset")
        if v is not None:
            return NozzleOffset.from_dict(v["value"])
        noz = next(n for n in self.m.profile.nozzles.items if n.id == self.m.nozzle_current())
        if noz.offset_guess_mm is None:
            return None
        return NozzleOffset(tuple(noz.offset_guess_mm), 0.0, 0.0, 0.0, False)

    def _rules(self):
        from .job.pickplace import height_rules
        return height_rules(self.m)

    def _z_clear(self) -> None:
        """Before any X / Y move of a procedure: the nozzle at the moving height (when Z is known)."""
        m = self.m
        if m.safety.z_ref:
            rules = self._rules()
            if m.position()["Z"] < rules.moving() - 0.05:
                m._move_z(rules.moving())

    def _servo_turn(self, deg: float | None, pulse: int | None = None) -> float | None:
        rot = self.m.caps.rotation
        if rot.kind == "none":
            return None
        if pulse is not None:
            rot.set_pulse(pulse)
            return rot.angle
        rot.turn_to(deg)
        return deg

    def _mark(self, head_xy, sheet_top: float, est: NozzleOffset, servo: float | None, pulse: int | None) -> Mark:
        """One touch on the sheet at head_xy, the mark measured from above it."""
        from .machine_vision import MachineRig
        m = self.m
        model = m.vision.model()
        rules = self._rules()
        rig = MachineRig(m, "home")
        moving = max(rules.moving(), sheet_top + rules.clearance)
        tip_guess = np.asarray(head_xy, float) + est.tip(servo)
        m._move_z(moving)
        rig.move_to(*tip_guess)                                          # the camera over where the mark will be
        before = np.mean(np.asarray(rig.frames(3), np.float32), axis=0)
        rig.move_to(*head_xy)
        if self._over_tray(np.asarray(head_xy, float) + est.tip(servo)):
            raise Refused(Refusal("The needle would touch down on a tray's tape, not on the sheet",
                                  "Lay the sheet away from the trays and start there"))
        m._move_z(rules.mark(sheet_top))
        m.caps.vacuum.dwell()
        m._move_z(moving)
        rig.move_to(*tip_guess)
        after = np.mean(np.asarray(rig.frames(3), np.float32), axis=0)
        w, h = model.size
        k = model.px_per_mm(sheet_top)
        search = 3.0 if est.rms < 0.5 and (est.runout_known or est.runout == 0) and chain.entry(m, "nozzle_offset") \
            else 8.0
        ch = find_change(before, after, (w / 2, h / 2), search * k, darker=True, min_change=30)
        if ch is None:
            raise CapabilityError(f"No new ink mark within {search:g} mm of where the tip should be. Is the sheet under "
                                  "the ring on the picture, and is there ink on the needle?")
        xy = model.px_to_xy([[ch.u, ch.v]], sheet_top)[0] + tip_guess
        return Mark(servo if servo is not None else 0.0, (float(head_xy[0]), float(head_xy[1])),
                    (float(xy[0]), float(xy[1])), pulse)

    def _over_tray(self, xy) -> bool:
        for n, (c, u, v) in self.slot_frames().items():
            t = self.m.feeders.trays.get(n)
            u0, u1 = t.zone_u if t is not None else (17.0, 77.0)
            v0, v1 = t.zone_v if t is not None else (-8.0, 8.0)
            d = np.asarray(xy, float) - c
            if u0 - 3 <= float(d @ u) <= u1 + 3 and v0 - 2 <= float(d @ v) <= v1 + 2:
                return True
        return False

    def _sheet_checks(self, sheet_top: float) -> None:
        m = self.m
        if not (m.safety.homed and m.safety.z_ref):
            raise Refused(Refusal("Not homed or Z has no reference", "Home and set Z first (step 4)"))
        if m.vision.model() is None:
            raise Refused(Refusal("The camera is not calibrated", "Step 3 first"))
        if not 0.0 <= sheet_top <= 10.0:
            raise Refused(Refusal("The sheet height must be 0 … 10 mm", "Type the sheet's height above the labels"))

    def nozzle_marks(self, sheet_top: float, angles: list[float] | None = None) -> None:
        """Marks at several servo angles on a sheet under the tip; the circle gives the axis, runout and phase."""
        m = self.m
        self._sheet_checks(sheet_top)
        est = self.offset_estimate()
        if est is None:
            raise Refused(Refusal("No idea yet where the tip is", "Use 'Pointer by eye' first, or write "
                                  "nozzles.<id>.offset_guess_mm in the profile"))
        rot = m.caps.rotation
        if angles is None:
            if rot.kind == "none" or not rot.availability().ok:
                angles = [None]
            else:
                lo, hi = rot.range_deg
                angles = [a for a in (-90.0, -45.0, 0.0, 45.0, 90.0) if lo <= a <= hi] or [0.0]
        start = np.array([m.position()["X"], m.position()["Y"]])
        marks = []
        for i, a in enumerate(angles):
            if m.worker.cancel.is_set():
                return
            self._z_clear()
            servo = self._servo_turn(a) if a is not None else None
            marks.append(self._mark(start + np.array([0.0, -i * MARK_SPACING]), sheet_top, est, servo, None))
            m.say(f"   mark {i + 1} of {len(angles)}: tip at {marks[-1].offset[0]:+.2f}, {marks[-1].offset[1]:+.2f} mm "
                  f"from the camera" + (f" (servo {a:+.0f}°)" if a is not None else ""), "debug", "setup")
        off = fit_offset(marks)
        self._offer("nozzle_offset", off.to_dict(), "ink marks", {"rms_mm": round(off.rms, 3), "marks": len(marks),
                                                                  "runout_mm": round(off.runout, 3)},
                    f"Nozzle offset: axis {off.axis[0]:+.2f}, {off.axis[1]:+.2f} mm from the camera"
                    + (f", needle {off.runout:.2f} mm off the axis (fit {off.rms:.3f} mm)" if off.runout_known else ""))
        if m.caps.rotation.kind != "none" and m.caps.rotation.availability().ok:
            self._servo_turn(0.0)
        self._z_clear()

    def pointer_tip_here(self) -> None:
        """Pointer by eye, part 1: the operator jogged the tip onto a sharp feature (a label corner)."""
        m = self.m
        if not (m.safety.homed and m.safety.z_ref):
            raise Refused(Refusal("Not homed or Z has no reference", "Step 4 first"))
        pos = m.position()
        self.pointer = {"head": (pos["X"], pos["Y"]), "z": pos["Z"], "servo": self.servo_angle()}
        m.say(f"Pointer: the tip is on the feature (head {pos['X']:.2f}, {pos['Y']:.2f}, tip Z {pos['Z']:.2f}). Now "
              "lift, move the camera over the same feature and click its exact spot on the picture", kind="setup")

    def servo_angle(self) -> float | None:
        rot = self.m.caps.rotation
        return None if rot.kind == "none" else getattr(rot, "angle", None)

    def pointer_click(self, u: float, v: float) -> None:
        """Pointer by eye, part 2: the click on the picture (an explicit target, never the cross, bug 5.5)."""
        m = self.m
        if self.pointer is None:
            raise Refused(Refusal("Say first where the tip touched", "Press 'The tip is on it'"))
        model = m.vision.model()
        w, h = model.size
        pos = m.position()
        xy = model.px_to_xy([[u * w, v * h]], max(0.0, self.pointer["z"]))[0] + np.array([pos["X"], pos["Y"]])
        mk = Mark(self.pointer["servo"] or 0.0, self.pointer["head"], (float(xy[0]), float(xy[1])))
        prev = self.offset_estimate()
        off = fit_offset([mk])
        if prev is not None and prev.runout_known:                    # keep the measured runout: move the axis
            ax = np.asarray(mk.offset) - (prev.tip(self.pointer["servo"]) - np.asarray(prev.axis))
            off = NozzleOffset((float(ax[0]), float(ax[1])), prev.runout, prev.phase_deg, 0.0, True, off.marks)
        self.pointer = None
        self._offer("nozzle_offset", off.to_dict(), "pointer by eye", {"marks": 1},
                    f"Nozzle offset (by eye): {off.axis[0]:+.2f}, {off.axis[1]:+.2f} mm from the camera")

    def _offer(self, key: str, value: dict, method: str, quality: dict, text: str) -> None:
        """Save it, unless it moves the offset by more than 1 mm: then it waits for the operator's yes."""
        old = chain.entry(self.m, key)
        if old is not None and "axis" in value:
            d = float(np.hypot(*(np.asarray(value["axis"]) - np.asarray(old["value"]["axis"]))))
            if d > CONFIRM_OVER_MM:
                self.pending[key] = {"value": value, "method": method, "quality": quality, "text": text,
                                     "change_mm": round(d, 2)}
                self.m.say(f"{text}. That is {d:.2f} mm from the saved one: press Accept to use it, or measure again",
                           "warn", "setup")
                return
        chain.put(self.m, key, value, method, quality)
        self.m.install_calibrated_caps()
        self.m.say(text + " (saved)", kind="setup")

    def accept(self, key: str) -> None:
        p = self.pending.pop(key, None)
        if p is None:
            raise Refused(Refusal("Nothing waits for a yes", ""))
        chain.put(self.m, key, p["value"], p["method"], p["quality"])
        self.m.install_calibrated_caps()
        self.m.say(p["text"] + " (accepted and saved)", kind="setup")

    # ================================================================ step 5b: the bottom camera (Phase G)
    def bottom_calibrate(self) -> None:
        """The bare needle over the bottom camera, at the height where it is sharp (the operator put it there).
        Moves 1 mm in X and in Y (lifting between: X / Y never move with the nozzle low) to measure the picture's
        scale, turn and mirror, then centres the tip; the nozzle offset gives where the camera looks on the bed."""
        from .machine_vision import MachineRig
        from .vision.bottom import BottomCal, find_blob
        m = self.m
        cam = m.cameras.get("camera_bottom")
        if cam is None or cam.kind == "none" or not cam.availability().ok:
            raise Refused(Refusal("No bottom camera", "Add cameras.bottom to the profile"))
        if not (m.safety.homed and m.safety.z_ref):
            raise Refused(Refusal("Not homed or Z has no reference", "Step 4 first"))
        off_v = chain.value(m, "nozzle_offset")
        if off_v is None:
            raise Refused(Refusal("The nozzle offset is not measured", "Step 5 first"))
        off = NozzleOffset.from_dict(off_v)
        pos = m.position()
        tip_z, head0 = float(pos["Z"]), np.array([pos["X"], pos["Y"]])
        servo = self.servo_angle()
        rig = MachineRig(m, "home")
        moving = max(self._rules().moving(), tip_z + 3.0)

        def look():
            img = np.mean(np.asarray(cam.frames_after(time.monotonic() + 0.15, 3), np.float32), axis=0)
            b = find_blob(img)
            if b is None:
                raise CapabilityError("The needle is not seen in the bottom camera: jog it over the camera and down "
                                      "until it is sharp, then Measure again")
            return np.array([b.u, b.v]), img

        def go(h):
            if np.hypot(*(np.asarray(h) - head0)) > 3.0:
                raise Refused(Refusal("The bottom camera step moves only near where you put the needle", ""))
            m._move_z(moving)
            rig.move_to(*h)
            m._move_z(tip_z)                                      # back down to the height you chose (the camera)
        p0, img = look()
        go(head0 + [1.0, 0.0])
        p1, _ = look()
        go(head0 + [0.0, 1.0])
        p2, _ = look()
        A = np.column_stack([p1 - p0, p2 - p0])
        if abs(np.linalg.det(A)) < 1.0:
            raise CapabilityError("The needle hardly moved in the bottom picture: is it the needle you see?")
        h, w = img.shape
        c = np.array([w / 2.0, h / 2.0])
        head, p = head0, p0
        for _ in range(2):                                         # the tip to the picture centre
            head = head + np.linalg.solve(A, c - p)
            go(head)
            p, _ = look()
        cam_xy = head + off.tip(servo) + np.linalg.solve(A, p - c)
        cal = BottomCal(A.tolist(), c.tolist(), [float(v) for v in cam_xy], tip_z, servo)
        m._move_z(moving)
        chain.put(m, "bottom_camera", cal.to_dict(), "the bare needle moved 1 mm in X and Y",
                  {"px_per_mm": round(cal.px_per_mm(), 2), "mirror": bool(np.linalg.det(A) > 0)})
        m.say(f"Bottom camera: {cal.px_per_mm():.1f} px/mm, looks at {cam_xy[0]:.2f}, {cam_xy[1]:.2f} mm from HOME, "
              f"sharp at tip Z {tip_z:.2f}" + (" (mirrored, as a camera looking up should be)" if np.linalg.det(A) > 0
                                               else " (NOT mirrored: is it really looking up?)"), kind="setup")

    # ================================================================ step 6: rotation table
    def rotation_marks(self, sheet_top: float, n_pulses: int = 7, repeats: int = 5) -> None:
        m = self.m
        self._sheet_checks(sheet_top)
        rot = m.caps.rotation
        if rot.kind == "stepper_axis":
            return self.stepper_marks(sheet_top, repeats)
        if rot.kind != "servo_pwm":
            raise Refused(Refusal(f"The angle table is for a servo; this machine has '{rot.kind}'",
                                  "Nothing to do here" if rot.kind == "none" else "Stepper axis: Phase G step"))
        off_v = chain.value(m, "nozzle_offset")
        if off_v is None:
            raise Refused(Refusal("The nozzle offset is not measured", "Step 5 first"))
        off = NozzleOffset.from_dict(off_v)
        c = rot.cfg
        pulses = [int(round(p)) for p in np.linspace(c.min_us + c.approach_us, c.max_us, n_pulses)]
        start = np.array([m.position()["X"], m.position()["Y"]])
        marks: list[Mark] = []
        for i, p in enumerate(pulses):
            if m.worker.cancel.is_set():
                return
            self._z_clear()
            self._servo_turn(None, p)
            marks.append(self._mark(start + np.array([0.0, -i * MARK_SPACING]), sheet_top, off, rot.angle_nominal(p), p))
        table = fit_table(marks, off)
        rep: list[Mark] = []
        centre = c.centre_us
        for j in range(repeats):
            if m.worker.cancel.is_set():
                return
            self._z_clear()
            self._servo_turn(None, c.min_us)                       # far away, then the same approach every time
            self._servo_turn(None, centre)
            rep.append(self._mark(start + np.array([-MARK_SPACING, -j * MARK_SPACING]), sheet_top, off,
                                  rot.angle_nominal(centre), centre))
        table.accuracy_deg = round(repeat_spread(rep, off), 2)
        chain.put(m, "rotation_table", table.to_dict(), "ink marks", {"rms_deg": round(table.rms_deg, 2),
                                                                      "accuracy_deg": table.accuracy_deg})
        m.install_calibrated_caps()
        m.say(f"Rotation table saved: {table.deg_per_us * 1000:.1f}° per 1000 µs (nominal "
              f"{(c.nominal_range_deg or 0) / (c.max_us - c.min_us) * 1000:.1f}), off a straight line by "
              f"{table.rms_deg:.1f}°; the same pulse stops up to ±{table.accuracy_deg:.1f}° apart", kind="setup")
        if c.accuracy_deg is not None and table.accuracy_deg > c.accuracy_deg:
            m.say(f"Rotation: ±{table.accuracy_deg:.1f}° is worse than the profile's tolerance ±{c.accuracy_deg:g}°: the "
                  "job turns parts only where the pads allow (0° / 180°)", "warn", "setup")
        self._servo_turn(0.0)

    def stepper_marks(self, sheet_top: float, repeats: int = 5) -> None:
        """A stepper rotation axis (Phase G): marks at commanded angles give its real scale; 0° from below and from
        above gives the backlash; the same angle 5 times the repeat. Then every turn comes from below."""
        m = self.m
        rot = m.caps.rotation
        off_v = chain.value(m, "nozzle_offset")
        if off_v is None:
            raise Refused(Refusal("The nozzle offset is not measured", "Step 5 first"))
        off = NozzleOffset.from_dict(off_v)
        if not off.runout_known or off.runout < 0.5:
            raise Refused(Refusal(f"The needle is only {off.runout:.2f} mm from the axis: marks can't show the angle",
                                  "Use the first part to check the angle"))
        start = np.array([m.position()["X"], m.position()["Y"]])
        lo, hi = rot.range_deg
        cmds = [float(a) for a in range(-150, 151, 30) if lo + 10 <= a <= hi]   # wide: the scale is read better
        marks, k = [], 0

        def mark(cmd: float, approach: float, row: float) -> Mark:
            nonlocal k
            self._z_clear()
            rot.command(cmd, approach)
            rot.angle = cmd
            mk = self._mark(start + np.array([row, -k * MARK_SPACING]), sheet_top, off, cmd, None)
            k += 1
            return mk
        for c in cmds:
            if m.worker.cancel.is_set():
                return
            marks.append(mark(c, 10.0, 0.0))
        real = np.degrees(np.unwrap(np.radians(np.array([a for a in _angles(marks, off)]))))
        scale, b0 = np.polyfit(cmds, real, 1)
        k = 0                                                   # 3 pairs: one mark reads the angle to ~1°
        pairs = [(mark(0.0, 10.0, -MARK_SPACING), mark(0.0, -10.0, -MARK_SPACING)) for _ in range(3)]
        backlash = abs(float(np.mean([_angles([a], off)[0] - _angles([b], off)[0] for b, a in pairs])))
        k = 0
        rep = [mark(0.0, 10.0, -2 * MARK_SPACING) for _ in range(repeats)]
        acc = round(repeat_spread(rep, off), 2)
        table = {"kind": "stepper", "scale": float(scale), "zero_deg": float(b0), "backlash_deg": round(backlash, 2),
                 "accuracy_deg": acc}
        chain.put(m, "rotation_table", table, "ink marks (stepper)", {"scale": round(float(scale), 4),
                                                                      "backlash_deg": table["backlash_deg"]})
        m.install_calibrated_caps()
        m.say(f"Rotation axis: it turns {scale * 100:.1f} % of the commanded angle, backlash {backlash:.2f}° (every "
              f"turn now comes from below), repeat ±{acc:.2f}°", kind="setup")
        rot.turn_to(0.0)
        self._z_clear()

    # ================================================================ step 6b: the vacuum sensor (Phase G)
    def vacuum_read(self, closed: bool) -> None:
        """The sensor with the pump on: once with the needle open (in the air), once closed (the tip pressed on the
        paper, or a part on it). The threshold is half way; a part is 'held' above it."""
        m = self.m
        vac = m.caps.vacuum
        if m.profile.vacuum.sensor is None:
            raise Refused(Refusal("No vacuum sensor in the profile", "Add vacuum.sensor"))
        if vac.reader is None:
            raise Refused(Refusal("How the controller reports the vacuum sensor is not known yet (TBD)",
                                  "Wire the sensor, then this program needs its reading (docs/builder/upgrades.md)"))
        vac.pump(True)
        if m.profile.vacuum.valve is not None:
            m.ctl.gcode(m.profile.vacuum.valve.start)
        m.ctl.gcode("G4 P0.5")
        vals = []
        for _ in range(5):
            vals.append(vac.read())
            time.sleep(0.05)
        vac.release()
        v = float(np.median(vals))
        self.vacuum_seen = getattr(self, "vacuum_seen", {})
        self.vacuum_seen["closed" if closed else "open"] = v
        m.say(f"Vacuum sensor, needle {'closed' if closed else 'open'}: {v:.1f}", kind="setup")
        seen = self.vacuum_seen
        if "open" in seen and "closed" in seen:
            if seen["closed"] - seen["open"] < 10.0:
                m.say("Vacuum sensor: open and closed read almost the same: is the needle really closed? A leak?",
                      "warn", "setup")
                return
            thr = (seen["open"] + seen["closed"]) / 2
            chain.put(m, "vacuum_sensor", {"open": seen["open"], "closed": seen["closed"], "threshold": thr},
                      "open and closed needle", {"margin": round((seen["closed"] - seen["open"]) / 2, 1)})
            m.install_calibrated_caps()
            m.say(f"Vacuum sensor saved: a part is held above {thr:.1f}", kind="setup")

    # ================================================================ step 2 with switches (Phase G)
    def switch_test(self, times: int = 3) -> None:
        """Home with the switches 3 times. With the HOME label in view (you jogged there first), the camera
        measures it after every Home: the spread is how well the switches repeat. The travel comes from the
        controller's max travel ($130 / $131)."""
        from .machine_vision import MachineRig
        from .vision import markers as vmk
        m = self.m
        if m.profile.homing.method != "switches":
            raise Refused(Refusal("This machine homes with the camera", "Use the travel creep"))
        model = m.vision.model()
        view = None
        if m.safety.homed and model is not None and m.markers is not None:
            pos = m.position()
            view = (pos["X"], pos["Y"])
        seen = []
        for i in range(times):
            if m.worker.cancel.is_set():
                return
            if i and m.safety.homed:                                # away from the switches, then Home again
                m._move({"X": -10.0, "Y": -10.0}, None, m.limits.slow_feed)
            m._home()
            if view is not None:
                rig = MachineRig(m, "home")
                rig.move_to(*view)
                off, why = vmk.home_offset(m.markers, rig.frames(4), model, 0.0)
                if off is not None:
                    seen.append(np.asarray(rig.head(), float) + off)
        st = m.ctl.info.settings if m.ctl else {}
        span = {}
        for a in ("X", "Y"):
            L = m.ctl.axes.letter(a) if m.ctl else a
            v = st.get(f"${130 + 'XYZ'.index(L)}") if L in "XYZ" else None
            span[a] = float(v) if v not in (None, "") else None
        spread = None
        if len(seen) >= 2:
            c = np.mean(seen, axis=0)
            spread = round(float(max(np.hypot(*(p - c)) for p in seen)), 3)
        if None in span.values():
            m.say("Homing switches: Home works, but the controller does not report its max travel ($130/$131): "
                  "find the ends with 'End here'", "warn", "setup")
            return
        travel = {"x": [-span["X"] + 1.0, -1.0], "y": [-span["Y"] + 1.0, -1.0]}
        chain.put(m, "travel", travel, "homing switches + the controller's max travel",
                  {"home_repeat_mm": spread, "homes": times})
        m.refresh_limits()
        m.say(f"Homing switches: {times} homes" + (f", HOME label within ±{spread:.3f} mm" if spread is not None else "")
              + f"; travel X {travel['x'][0]:.1f} … {travel['x'][1]:.1f}, Y {travel['y'][0]:.1f} … {travel['y'][1]:.1f}",
              kind="setup")

    # ================================================================ step 8: bed survey
    def survey(self) -> None:
        """Every label of the bed plan measured at the picture centre: where it is and which way it points."""
        from .machine_vision import MachineRig
        m = self.m
        if not m.safety.homed:
            raise Refused(Refusal("Not homed", "Home first"))
        model = m.vision.model()
        if model is None or m.markers is None:
            raise Refused(Refusal("No camera calibration or no bed file", "Step 3 first"))
        self._z_clear()
        lay = (m.bed or {}).get("layout", {}) or {}
        plan = {t.name: t.plan_xy for t in m.feeders.trays.values() if t.plan_xy is not None}
        if (m.bed.get("pcb_area") or {}).get("corner_labels") and lay.get("pcb_corner_marker_xy"):
            plan.update({f"pcb_{i}": v for i, v in enumerate(lay["pcb_corner_marker_xy"], start=1)})
        rig = MachineRig(m, "home")
        out, notes = {}, []
        for name, xy in plan.items():
            if m.worker.cancel.is_set():
                return
            target = self._inside_travel(np.asarray(xy, float))
            if target is None:
                notes.append(f"{name}: too far outside the travel, not measured")
                continue
            seen = None
            tray = m.feeders.trays.get(int(name[5:])) if name.startswith("slot_") else None
            z = tray.marker_z if tray is not None else 0.0                 # a holder's marker is on its top
            for _ in range(2):
                rig.move_to(*target)
                s = m.markers.measure(rig.frames(4), model, name, z)
                if s is None:
                    break
                seen = s
                xy_m = s.centre_xy + np.array(rig.head())
                nxt = self._inside_travel(xy_m)                     # off-centre when the label is outside the travel
                if nxt is None or np.hypot(*(nxt - target)) < 1.0:
                    break
                target = nxt
            if seen is None:
                notes.append(f"{name}: not found near the plan")
                continue
            head = np.array(rig.head())
            corners = model.px_to_xy(seen.corners_px, z) + head
            u = corners[1] - corners[0] + corners[2] - corners[3]
            u /= np.linalg.norm(u)
            out[name] = {"xy": [round(float(v), 3) for v in corners.mean(axis=0)],
                         "u": [round(float(v), 5) for v in u], "sides_mm": [round(float(v), 3) for v in seen.sides_mm]}
            m.say(f"   {name}: {out[name]['xy'][0]:.2f}, {out[name]['xy'][1]:.2f} "
                  f"({np.hypot(*(np.array(out[name]['xy']) - np.asarray(xy, float))):.2f} mm from the plan)", "debug",
                  "setup")
        chain.put(m, "bed_survey", {"labels": out, "notes": notes}, "camera at every label", {"labels": len(out)})
        m.say(f"Bed survey: {len(out)} labels measured" + (f"; {'; '.join(notes)}" if notes else ""),
              "warn" if notes else "info", "setup")

    def _inside_travel(self, xy, reach_mm: float = 25.0):
        """The nearest head position inside the travel; None when the label would be too far off the picture centre."""
        t = self.m.limits.travel
        if t["X"] is None or t["Y"] is None:
            return None
        q = np.array([min(max(xy[0], t["X"][0]), t["X"][1]), min(max(xy[1], t["Y"][0]), t["Y"][1])])
        return q if np.hypot(*(q - xy)) <= reach_mm else None

    def slot_frames(self) -> dict[int, tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """Slot n -> (marker centre, u along the label, v = u turned −90°), from the survey (else the plan)."""
        v = chain.value(self.m, "bed_survey")
        out = {}
        if v is not None:
            for name, d in v["labels"].items():
                if name.startswith("slot_") and int(name[5:]) in self.m.feeders.trays:
                    u = np.asarray(d["u"], float)
                    out[int(name[5:])] = (np.asarray(d["xy"], float), u, np.array([u[1], -u[0]]))
            return out
        for n, t in self.m.feeders.trays.items():                   # no survey yet: the plan
            if t.plan_xy is not None:
                a = np.asarray(t.plan_dir, float)
                a /= np.linalg.norm(a)
                out[n] = (np.asarray(t.plan_xy, float), a, np.array([a[1], -a[0]]))
        return out

    # ================================================================ step 10: the first part
    def slot_setups(self, slot: int, package: str) -> list:
        """Every row of a tray as a strip-like setup (a strip and a stick: one)."""
        s, pk = self.slot_setup(slot, package)
        tray = self.m.feeders.trays[slot]
        rows = tray.cells()
        if not rows:
            return [s]
        out = []
        used = set(self.m.feeder_state.get(slot).get("used", []))
        for r, us, v in rows:
            from .job import dryrun
            from .vision import parts as vp
            spec = vp.StripSpec(body=s.spec.body, pitch=tray.pitch_u, u_range=(us[0] - tray.pitch_u / 2 - 1.0,
                                                                                  us[-1] + tray.pitch_u / 2 + 1.0),
                                hole_row=-99.0)
            q = dryrun.SlotSetup(slot, s.centre + s.v * v, s.u, s.v, spec, s.z_tape, s.template,
                                 {e - r * 1000 for e in used if e // 1000 == r}, r, us, tray.type == "stick")
            out.append(q)
        return out

    def slot_setup(self, slot: int, package: str):
        from .job import dryrun
        from .vision import parts as vp
        frames = self.slot_frames()
        if slot not in frames:
            raise Refused(Refusal(f"Slot {slot} is not on the bed survey", "Survey the bed (step 8)"))
        lib = self.m.boards.library()
        pk = lib.packages.get(package)
        if pk is None:
            raise Refused(Refusal(f"Package {package!r} is not in the library", "Add it (packages.user.yaml)"))
        c, u, v = frames[slot]
        tray = self.m.feeders.trays[slot]
        spec = vp.StripSpec(body=tuple(pk.body), pitch=float(pk.tape.get("pitch", 4)) if pk.tape else 4.0,
                            u_range=tray.zone_u, hole_row=tray.hole_row)
        s = dryrun.SlotSetup(slot, c, u, v, spec, tray.tape_floor_mm + (pk.height or 0.0))
        s.template = self.templates.get(slot)
        if s.template is None:
            s.template = self.m.feeder_state.template(slot)
        s.used = set(self.m.feeder_state.get(slot).get("used", []))
        return s, pk

    def look_at_slot(self, slot: int, package: str) -> None:
        """Find the first candidate part of a slot and show it: the operator says if it is a part (docs/13 §4.4)."""
        from .job import dryrun
        from .machine_vision import MachineRig
        from .vision import parts as vp
        import cv2
        m = self.m
        s = self.slot_setups(slot, package)[0]                  # a matrix tray: its first row
        self._z_clear()
        rig = MachineRig(m, "home")
        rig.move_to(*(s.centre + s.u * sum(s.spec.u_range) / 2))
        strip, back = dryrun.look(rig, m.vision.model(), s)
        found = vp.find(strip, back, s.spec) if s.cells is None else vp.find_cells(strip, back, s.spec, s.cells)
        for cand in (f for f in found if f.state == "candidate"):
            try:
                tpl = vp.learn_template(strip, cand, s.spec)
            except ValueError:
                continue
            col, row = (cand.u - s.spec.u_range[0]) * vp.RES, (7.0 - cand.v) * vp.RES
            crop = strip[max(0, int(row) - 40):int(row) + 40, max(0, int(col) - 60):int(col) + 60]
            path = m.data / "setup" / f"slot{slot}_candidate.png"
            path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(path), cv2.resize(np.clip(crop, 0, 255).astype(np.uint8), None, fx=3, fy=3,
                                              interpolation=cv2.INTER_NEAREST))
            self.candidate = {"slot": slot, "package": package, "template": tpl, "image": str(path), "u": cand.u}
            m.say(f"Slot {slot}: is this a {package} part? (Setup, step 10: Yes / No)", kind="setup")
            return
        self.candidate = None
        m.say(f"Slot {slot}: no part found in the tape mark. Is the strip in its slot, film off, holes to the right?",
              "warn", "setup")

    def confirm_candidate(self, yes: bool) -> dict:
        c = self.candidate
        if c is None:
            raise Refused(Refusal("No part waits for a yes", "Press 'Look at the slot' first"))
        self.candidate = None
        if yes:
            self.templates[c["slot"]] = c["template"]
            self.m.feeder_state.set_template(c["slot"], c["template"])
            self.m.say(f"Slot {c['slot']}: the part is confirmed; the next parts are matched to it", kind="setup")
        else:
            self.m.say(f"Slot {c['slot']}: not a part. Check the strip and look again", "warn", "setup")
        return {"ok": True}

    def first_part(self, slot: int, ref: str) -> None:
        from .job.pickplace import PickPlace
        m = self.m
        board, reg = m.vision.registration()
        if board is None or reg is None:
            raise Refused(Refusal("The board is not registered", "Step 9 first"))
        c = board.component(ref)
        if c is None:
            raise Refused(Refusal(f"No part {ref} on the board", "Choose a part of the board"))
        if c.side != reg.side:
            raise Refused(Refusal(f"{ref} is on side {c.side}, the board lies with {reg.side} up", "Choose another part"))
        lib = m.boards.library()
        pk = lib.packages.get(c.package)
        if pk is None or pk.pickable != "yes":
            raise Refused(Refusal(f"{ref} ({c.package or 'unknown package'}) is not a machine part", "Choose a chip part"))
        if pk.polar and pk.tape_angle is None:
            raise Refused(Refusal(f"{c.package} is polar and its angle in the tape is not known yet",
                                  "Choose a non-polar part (R, C) for the first part"))
        s, _ = self.slot_setup(slot, c.package)
        if s.template is None:
            raise Refused(Refusal(f"The first part of slot {slot} is not confirmed", "Look at the slot and say Yes"))
        pp = PickPlace(m, board, reg)
        folder = m.data / "setup" / f"first_part_{time.strftime('%Y%m%d-%H%M%S')}"
        target_angle = reg.angle_deg + c.rotation + pp.correction[2]
        picked, s_place = pp.pick(s, pk.height or 1.0, not pk.polar, target_angle, folder)
        if picked.check != "picked":
            m.say(f"First part: the pick {picked.check} (slot {slot}): look at the slot and try again", "warn", "setup")
            return
        res = pp.place(ref, picked, s_place, pk.height or 1.0, folder)
        m.placed.mark(board.name, ref, "machine")                  # a job never places it again
        self.last_first_part = {"ref": ref, "slot": slot, "photos": {**picked.photos, **res.photos},
                                "error": None if res.error is None else [round(float(v), 3) for v in res.error],
                                "angle_error": None if res.angle_error is None else round(float(res.angle_error), 2),
                                "turn": round(res.turn, 1), "why": res.why}
        if res.error is None:
            m.say(f"First part {ref}: placed, but the camera did not see it ({res.why})", "warn", "setup")
        else:
            m.say(f"First part {ref}: placed {np.hypot(*res.error):.2f} mm from its place (X {res.error[0]:+.2f}, "
                  f"Y {res.error[1]:+.2f}), angle {res.angle_error:+.1f}°. Look at the photos: OK, or move it by "
                  "(dx, dy, dθ)", kind="setup")
        pp.lift()

    def first_part_answer(self, ok: bool, dx: float = 0.0, dy: float = 0.0, dtheta: float = 0.0) -> None:
        """OK: the correction stays. Otherwise the operator's (dx, dy, dθ) is added to it for every placement."""
        old = chain.value(self.m, "first_part") or {"dx": 0.0, "dy": 0.0, "dtheta": 0.0}
        new = {"dx": old["dx"] + (0 if ok else dx), "dy": old["dy"] + (0 if ok else dy),
               "dtheta": old["dtheta"] + (0 if ok else dtheta)}
        chain.put(self.m, "first_part", new, "first part: operator" + (" OK" if ok else " moved it"),
                  {"last": self.last_first_part or {}})
        self.m.say("First part: OK, saved" if ok else
                   f"First part: correction now X {new['dx']:+.2f}, Y {new['dy']:+.2f} mm, {new['dtheta']:+.1f}°: place "
                   "another one to check", kind="setup")


def _angles(marks, off) -> list[float]:
    from .calibration.nozzle import mark_angles
    return [float(a) for a in mark_angles(marks, off)]
