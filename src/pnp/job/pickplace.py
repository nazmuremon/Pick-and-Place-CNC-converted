"""Pick one part from a strip and place it (docs/13 §4.6 step 10, §4.7, §4.8). The Phase E job runner uses these.

Rules (docs/13 §4.8, bug 5.6):
- Z goes down by itself only when homed, with a valid Z reference, valid heights (derived) and a valid nozzle
  offset, and only over a known surface: a tray's tape zone or the registered board's outline (or the mark sheet).
- X / Y never move with the nozzle below the moving height.
- The pump goes off only over the tape (a missed pick) or the board (a place), or when the operator says "drop".
- When a pick check can't tell, the part stays on the nozzle and the operator decides.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

import cv2
import numpy as np

from ..calibration import chain
from ..calibration.heights import HeightRules
from ..calibration.nozzle import NozzleOffset, RotationTable, choose_angles
from ..motion.state import Refusal, Refused
from ..vision import parts as vp
from ..vision.placecheck import measure_placed
from . import dryrun

if TYPE_CHECKING:
    from ..machine import Machine


class PartHeld(RuntimeError):
    """The part may still be on the nozzle: the operator must look (the pump stays on)."""


@dataclass
class Picked:
    slot: int
    pocket_u: float
    part_xy: np.ndarray              # where the part was (machine, mm from HOME)
    part_angle: float                # its long side (machine angle, degrees)
    servo_pick: float | None
    check: str                       # picked | missed | unclear
    photos: dict = field(default_factory=dict)
    template: np.ndarray | None = None   # the part's picture in the tray (rows = −v, cols = +u)
    u: np.ndarray | None = None          # the slot frame at the pick
    v: np.ndarray | None = None
    body: tuple[float, float] = (0.0, 0.0)


@dataclass
class PlaceResult:
    ref: str
    target: np.ndarray
    target_angle: float
    measured: np.ndarray | None = None
    measured_angle: float | None = None
    error: np.ndarray | None = None
    angle_error: float | None = None
    turn: float = 0.0
    photos: dict = field(default_factory=dict)
    why: str = ""


def _ang180(a: float) -> float:
    return (a + 90.0) % 180.0 - 90.0


def height_rules(m: Machine, board=None) -> HeightRules:
    """The rules from the profile, bed.yaml, the package library and the board. Refused when a number is TBD."""
    p = m.profile
    noz = next(n for n in p.nozzles.items if n.id == m.nozzle_current())
    if noz.press_mm is None:
        raise Refused(Refusal("The nozzle's press is TBD", f"Write nozzles.{noz.id}.press_mm in the profile (docs/08)"))
    if p.motion.z_clearance_mm is None:
        raise Refused(Refusal("The moving clearance is TBD", "Write motion.z_clearance_mm in the profile (docs/08)"))
    bed = m.bed or {}
    floors = [t.tape_floor_mm for t in m.feeders.trays.values()]          # the highest tray sets the moving height
    tape_floor = max(floors) if floors else float((bed.get("feed_tray") or {}).get("tape_floor_mm") or 0.0)
    board_top = m.vision.board_top_z(board) if board is not None else None
    tallest = 1.0
    if board is not None:                                  # the tallest part the machine itself places
        from ..feeders.packages import annotate
        classes = annotate(board, m.boards.library())
        heights = [c.height for c in board.components if c.height and classes[c.ref]["class"] == "machine"]
        tallest = max(heights) if heights else 1.0
    return HeightRules(noz.press_mm, p.motion.z_clearance_mm, tape_floor, board_top, tallest)


class PickPlace:
    def __init__(self, m: Machine, board=None, reg=None, rules: HeightRules | None = None) -> None:
        self.m, self.board, self.reg = m, board, reg
        self.model = m.vision.model()
        if self.model is None:
            raise Refused(Refusal("The camera is not calibrated", "Calibrate it (Setup, step 3)"))
        v = chain.value(m, "nozzle_offset")
        if v is None:
            raise Refused(Refusal("The nozzle offset is not measured (or stale)", "Measure it (Setup, step 5)"))
        self.off = NozzleOffset.from_dict(v)
        self.rules = rules or height_rules(m, board)
        if not (m.safety.homed and m.safety.z_ref):
            raise Refused(Refusal("Not homed or Z has no reference", "Home and set Z (Setup, step 4)"))
        if m.profile.vacuum.pick_dwell_s is None:
            raise Refused(Refusal("The pick dwell is TBD", "Write vacuum.pick_dwell_s in the profile"))
        from ..machine_vision import MachineRig
        self.rig = MachineRig(m, "home")
        self.moving = self.rules.moving()
        self.correction = np.zeros(3)
        self.bottom_photos: dict = {}
        bv = chain.value(m, "bottom_camera")
        cam_b = m.cameras.get("camera_bottom")
        from ..vision.bottom import BottomCal
        self.bottom = BottomCal.from_dict(bv) if bv is not None and cam_b is not None and cam_b.kind != "none" else None
        cv_ = chain.value(m, "first_part")
        if cv_ is not None:
            self.correction = np.array([cv_.get("dx", 0.0), cv_.get("dy", 0.0), cv_.get("dtheta", 0.0)])

    # ---------------------------------------------------------------- motion helpers
    def servo(self) -> float | None:
        rot = self.m.caps.rotation
        return None if rot.kind == "none" else getattr(rot, "angle", None)

    def turn(self, deg: float | None) -> None:
        rot = self.m.caps.rotation
        if deg is None or rot.kind == "none":
            return
        if getattr(rot, "angle", None) is not None and abs(rot.angle - deg) < 0.05:
            return
        rot.turn_to(deg)

    def lift(self) -> None:
        pos = self.m.position()
        if pos["Z"] is None or abs(pos["Z"] - self.moving) > 0.005:
            self.m._move_z(self.moving)

    def move_head(self, xy) -> None:
        pos = self.m.position()
        if pos["Z"] is None or pos["Z"] < self.moving - 0.05:
            raise Refused(Refusal("X / Y never move with the nozzle low", "Lift first"))
        self.rig.move_to(float(xy[0]), float(xy[1]))
        if self.m.safety.lock is not None:
            raise Refused(Refusal(self.m.safety.lock.title, self.m.safety.lock.todo))

    def camera_over(self, xy) -> None:
        self.lift()
        self.move_head(xy)

    def tip_over(self, xy, servo: float | None) -> None:
        self.lift()
        self.move_head(np.asarray(xy, float) - self.off.tip(servo))

    def can_pick_at(self, xy) -> bool:
        """The camera can look at xy and the tip can get there (at any servo angle: the runout as a margin)."""
        t = self.m.limits.travel
        if t["X"] is None or t["Y"] is None:
            return False
        r = self.off.runout if self.off.runout_known else 0.0
        h = np.asarray(xy, float) - np.asarray(self.off.axis, float)
        inside = lambda p, m: t["X"][0] + m <= p[0] <= t["X"][1] - m and t["Y"][0] + m <= p[1] <= t["Y"][1] - m
        return inside(np.asarray(xy, float), 0.0) and inside(h, r)

    def look(self, n: int = 3) -> np.ndarray:
        return np.mean(np.asarray(self.rig.frames(n), np.float32), axis=0)

    def down(self, z: float, where: str) -> None:
        """Z down for a pick / place / mark. where: what the tip is over, checked here."""
        pos = self.m.position()
        tip = np.array([pos["X"], pos["Y"]]) + self.off.tip(self.servo())
        if where == "board":
            if self.board is None or self.reg is None or not _on_board(self.board, self.reg, tip):
                raise Refused(Refusal("The tip is not over the registered board", "Register the board (Board tab)"))
        elif where == "tape":
            if not self._on_tape(tip):
                raise Refused(Refusal("The tip is not over a tray's tape zone", "Survey the bed (Setup, step 8)"))
        elif where != "sheet":
            raise ValueError(where)
        self.m._move_z(z)

    def _on_tape(self, tip) -> bool:
        for n, (c, u, v) in self.m.setup.slot_frames().items():
            t = self.m.feeders.trays.get(n)
            u0, u1 = t.zone_u if t is not None else (17.0, 77.0)
            v0, v1 = t.zone_v if t is not None else (-8.0, 8.0)
            du, dv = float(np.dot(tip - c, u)), float(np.dot(tip - c, v))
            if u0 - 1.0 <= du <= u1 + 1.0 and v0 <= dv <= v1:
                return True
        return False

    # ---------------------------------------------------------------- pick
    def pick(self, s: dryrun.SlotSetup, height: float, symmetric: bool, target_angle: float | None,
             photo_dir: Path | None = None, on_lifted=None) -> tuple[Picked, float | None]:
        """The next present part of the slot: centre the camera on it, pick it at a servo angle that leaves room
        for the turn to target_angle. Returns the pick and the servo angle to place at."""
        mid = s.centre + s.u * sum(s.spec.u_range) / 2
        self.camera_over(mid)
        strip, back = dryrun.look(self.rig, self.model, s)
        pockets = vp.find(strip, back, s.spec, template=s.template) if s.cells is None else \
            vp.find_cells(strip, back, s.spec, s.cells, s.template)
        found = [f for f in pockets if f.state == "present"
                 and (s.stick or pocket_key(s, f.u) not in s.used) and self.can_pick_at(dryrun.pocket_xy(s, f))]
        if not found:
            raise Refused(Refusal(f"Slot {s.slot} has no part left", "Refill it"))
        f = min(found, key=lambda x: x.u)
        self.camera_over(dryrun.pocket_xy(s, f))
        before = self.look()
        strip, back = dryrun.look(self.rig, self.model, s)
        again = [g for g in (vp.find(strip, back, s.spec, template=s.template) if s.cells is None else
                             vp.find_cells(strip, back, s.spec, s.cells, s.template)) if g.state == "present"]
        if not again:
            raise Refused(Refusal("The part was not seen again from above it", "Look at the slot"))
        g = min(again, key=lambda x: abs(x.u - f.u))
        xy = dryrun.pocket_xy(s, g)
        a = math.radians(g.angle)
        d = math.cos(a) * s.u + math.sin(a) * s.v
        part_angle = math.degrees(math.atan2(d[1], d[0]))
        rot = self.m.caps.rotation
        s_pick, s_place = None, None
        if rot.kind != "none" and target_angle is not None:
            rng = rot.range_deg or (0.0, 0.0)
            s_pick, s_place, turn = choose_angles(target_angle - part_angle, rng, symmetric)
            if abs(turn) > 1.0 and rot.kind == "servo_pwm" and getattr(rot, "table", None) is None:
                raise Refused(Refusal(f"The part must turn {turn:+.0f}°, but the servo's angle table is not measured "
                                      "(which way it turns, how far)", "Measure it (Setup, step 6)"))
        elif rot.kind != "none":                                 # no turn wanted: pick at the servo's angle now
            s_pick = s_place = self.servo() if self.servo() is not None else 0.0
        if rot.kind == "none" and target_angle is not None:
            turn = _ang180(target_angle - part_angle) if symmetric else (target_angle - part_angle)
            if abs(turn) > 5.0:
                raise Refused(Refusal(f"The part must turn {turn:+.0f}° but this nozzle does not turn",
                                      "Place it by hand"))
        self.turn(s_pick)
        self.tip_over(xy, s_pick)
        self.down(s.z_tape - self.rules.press, "tape")             # this tray's part top − press
        self.m.caps.vacuum.pick()
        self.m.caps.vacuum.dwell()
        if on_lifted is not None:                                # saved before anything else can stop: a STOP from
            on_lifted({"tray": s.slot, "xy": [float(v) for v in xy], "angle": part_angle,   # here on knows the part
                       "servo_pick": s_pick, "servo_place": s_place, "pocket_u": g.u})
        self.lift()
        if not s.stick:
            s.used.add(pocket_key(s, g.u))
        self.camera_over(dryrun.pocket_xy(s, g))
        after = self.look()
        # the pick check: is the part still in its pocket (matched to the confirmed template)? The plain picture
        # difference only says 'unclear' when the rest of the picture changed (a hand, the light)
        c_px = self.model.xy_to_px([xy - np.array(self.rig.head())], s.z_tape)[0]
        k = self.model.px_per_mm(s.z_tape)
        half = max(s.spec.body) * 0.4 * k
        chk = vp.pick_check(before, after, c_px, (half, half))
        strip2, back2 = dryrun.look(self.rig, self.model, s)
        again = [f for f in (vp.find(strip2, back2, s.spec, template=s.template) if s.cells is None else
                             vp.find_cells(strip2, back2, s.spec, s.cells, s.template)) if abs(f.u - g.u) < s.spec.pitch / 2]
        # a stick refills its pick point at once: a part there again is the NEXT part, not a miss
        still = bool(again) and again[0].state == "present" and not s.stick
        cam = "unclear" if chk.verdict == "unclear" else ("missed" if still else "picked")
        sensor = self.m.caps.vacuum.held()                 # None without a sensor: the camera decides alone
        if sensor is None or cam == "unclear":
            verdict = cam if sensor is None else ("picked" if sensor else "missed")
        else:
            verdict = cam if (cam == "picked") == sensor else "unclear"   # they disagree: the operator looks
        photos = _save(photo_dir, {"pick_before": before, "pick_after": after})
        p = Picked(s.slot, g.u, xy, part_angle, s_pick, verdict, photos, s.template, s.u, s.v, s.spec.body)
        if verdict == "missed":                                  # the pump goes off over the tape only (bug 5.6),
            self.tip_over(xy, s_pick)                            # just above it: never let a part fall from high
            self.down(s.z_tape + 0.5, "tape")
            self.m.caps.vacuum.release()
            self.lift()
        elif verdict == "unclear":
            raise PartHeld(f"Pick check unclear ({chk.why}): the pump stays on. Look at the nozzle")
        return p, s_place

    # ---------------------------------------------------------------- place
    # ---------------------------------------------------------------- the bottom camera (Phase G)
    def bottom_view(self, height: float, photo_dir: Path | None = None, tag: str = "bottom") -> tuple[np.ndarray, float]:
        """The part on the nozzle seen from below: its centre relative to the head (mm) and its angle (machine)."""
        from ..vision.bottom import find_blob
        cal = self.bottom
        self.lift()
        self.move_head(np.asarray(cal.cam_xy, float) - self.off.tip(self.servo()))
        self.m._move_z(cal.tip_z + height)                        # the part's bottom at the camera's sharp height
        cam = self.m.cameras["camera_bottom"]
        img = np.mean(np.asarray(cam.frames_after(time.monotonic() + 0.15, 3), np.float32), axis=0)
        head = np.asarray(self.rig.head(), float)
        self.lift()
        self.bottom_photos.update(_save(photo_dir, {tag: img}))
        b = find_blob(img)
        if b is None:
            raise PartHeld("Nothing on the nozzle seen from below (the part fell?): the pump stays on. Look at it")
        return cal.to_machine((b.u, b.v)) - head, cal.angle_to_machine(b.angle_px)

    def place(self, ref: str, picked: Picked, servo_place: float | None, height: float,
              photo_dir: Path | None = None, use_bottom: bool = False, symmetric: bool = True) -> PlaceResult:
        c = self.board.component(ref)
        target = self.reg.to_machine(self.board, (c.x, c.y)) + self.correction[:2]
        t_ang = self.reg.angle_deg + c.rotation + self.correction[2]
        res = PlaceResult(ref, target, t_ang)
        z_top = self.m.vision.board_top_z(self.board) + height
        self.camera_over(target)
        before = self.look()
        self.turn(servo_place)
        turned = 0.0 if picked.servo_pick is None or servo_place is None else servo_place - picked.servo_pick
        rel = None
        self.bottom_photos = {}
        if use_bottom and self.bottom is not None:
            # from below: where the part really is on the needle and how it is turned; one turn to correct the angle
            # (the servo's error), then look again. The head then puts the part's real centre on the target
            rel, a = self.bottom_view(height, photo_dir)
            rot = self.m.caps.rotation
            for _ in range(2):
                need = _ang180(t_ang - a) if symmetric else ((t_ang - a + 180) % 360 - 180)
                if rot.kind == "none" or abs(need) < 0.5 or self.servo() is None:
                    break
                lo, hi = rot.range_deg or (0.0, 0.0)
                to = self.servo() + need
                if not lo <= to <= hi and symmetric:
                    to = self.servo() + need - math.copysign(180.0, need)
                if not lo <= to <= hi:
                    break
                self.turn(to)
                rel, a = self.bottom_view(height, photo_dir, "bottom_after_turn")
            if self.servo() is not None and picked.servo_pick is not None:
                turned = self.servo() - picked.servo_pick                 # all the turning commanded so far
            turned = turned + _ang180((a - picked.part_angle) - turned)   # the real one (a chip: known mod 180°)
        res.turn = turned
        if rel is not None:
            self.lift()
            self.move_head(target - rel)
        else:
            self.tip_over(target, servo_place)
        self.down(self.rules.place(height), "board")
        self.m.caps.vacuum.release()
        self.lift()
        self.camera_over(target)
        after = self.look()
        res.photos = _save(photo_dir, {"place_before": before, "place_after": after}) | self.bottom_photos
        if picked.template is None:
            res.why = "no template of the part: the place was not measured"
            return res
        a = math.radians(turned)
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        got = measure_placed(before, after, self.model, np.array(self.rig.head()), z_top, picked.template,
                             R @ picked.u, R @ picked.v, picked.body, target)
        if got is None or got.gain < 0.3:
            res.why = "no part seen at the place (still on the nozzle? fallen?)"
            return res
        res.measured = got.xy
        res.error = got.xy - target
        res.measured_angle = picked.part_angle + turned + got.dtheta
        res.angle_error = _ang180(res.measured_angle - t_ang)
        return res


def pocket_key(s, u: float) -> int:
    """Which pocket: a fixed cell's own index (matrix / stick), else the strip's pocket number along the tape."""
    if s.cells:
        return int(np.argmin([abs(u - c) for c in s.cells]))
    return round(u / s.spec.pitch)


def _on_board(board, reg, xy) -> bool:
    if not board.outline:
        return False
    poly = np.array([reg.to_machine(board, p) for p in board.outline[0]], np.float32).reshape(-1, 1, 2)
    return cv2.pointPolygonTest(poly, (float(xy[0]), float(xy[1])), False) >= 0


def _save(folder: Path | None, pics: dict) -> dict:
    if folder is None:
        return {}
    folder.mkdir(parents=True, exist_ok=True)
    out = {}
    for name, img in pics.items():
        p = folder / f"{name}_{time.strftime('%H%M%S')}.png"
        cv2.imwrite(str(p), np.clip(img, 0, 255).astype(np.uint8))
        out[name] = str(p)
    return out


__all__ = ["PartHeld", "PickPlace", "Picked", "PlaceResult", "RotationTable", "height_rules"]
