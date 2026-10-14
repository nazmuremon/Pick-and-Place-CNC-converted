"""The dry run (camera only, docs/13 §4.7 and §11 Phase C "sim 20-part dry run"): for every part, find the next
full pocket of its slot and centre the camera on it (the pick), then go to its place on the registered board and
check its pads are seen there. Nothing goes down, the pump stays off. Phase E builds the job runner on this.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from ..board.model import Board
from ..vision import parts as vp
from ..vision.camera_model import CameraModel
from ..vision.register import Registration


@dataclass
class SlotSetup:
    slot: int
    centre: np.ndarray              # the slot marker's centre, mm from HOME (bed survey)
    u: np.ndarray                   # slot frame axes (docs/07): u along the label, v = the label's +Y
    v: np.ndarray
    spec: vp.StripSpec
    z_tape: float                   # tape top above the label surface
    template: np.ndarray | None = None
    used: set = field(default_factory=set)
    row: int = 0                    # a matrix tray's row (its used pockets are stored as row * 1000 + k)
    cells: list[float] | None = None    # matrix / stick: the pockets' u (fixed places); None = a strip
    stick: bool = False             # a stick: the pick point refills itself (never 'used')


@dataclass
class DryPart:
    ref: str
    slot: int
    pick_xy: tuple[float, float] | None = None
    pick_score: float = 0.0
    place_xy: tuple[float, float] | None = None
    pads_seen: bool | None = None
    why: str = ""


def mapper(model: CameraModel, head, s: SlotSetup):
    """Slot frame (u, v) -> picture pixels, through the camera model at the tape's height."""
    def f(uv):
        uv = np.asarray(uv, float)
        xy = s.centre + uv[:, :1] * s.u + uv[:, 1:2] * s.v
        return model.xy_to_px(xy - np.asarray(head, float), s.z_tape)
    return f


def look(rig, model: CameraModel, s: SlotSetup):
    img = np.mean(np.asarray(rig.frames(2), np.float32), axis=0)
    head = np.asarray(rig.head(), float)[:2]
    strip, _back = vp.straighten(img, mapper(model, head, s), s.spec)
    return strip, _back


def pocket_xy(s: SlotSetup, f: vp.Found) -> np.ndarray:
    return s.centre + f.u * s.u + f.v * s.v


def run(rig, model: CameraModel, board: Board, reg: Registration, plan: list[tuple[str, int]],
        slots: dict[int, SlotSetup], z_board: float, confirm: Callable[[np.ndarray, vp.Found], bool],
        say=print, reach=None) -> list[DryPart]:
    """reach: (xmin, xmax, ymin, ymax) of the camera centre; pockets and places outside it are skipped."""
    from ..motion.state import Refused, Stopped
    out: list[DryPart] = []
    for ref, slot in plan:
        dp = DryPart(ref, slot)
        out.append(dp)
        try:
            _one(rig, model, board, reg, slots[slot], dp, z_board, confirm, say, reach)
        except Stopped:
            dp.why = "stopped"
            raise
        except Refused as err:
            dp.why = str(err)
        if dp.why:
            say(f"   {ref}: {dp.why}")
    return out


def _inside(reach, xy) -> bool:
    return reach is None or (reach[0] <= xy[0] <= reach[1] and reach[2] <= xy[1] <= reach[3])


def _one(rig, model: CameraModel, board: Board, reg: Registration, s: SlotSetup, dp: DryPart, z_board: float,
         confirm, say, reach) -> None:
    # 1. the strip: a view over the tape mark's middle, then the next full pocket within reach
    mid = s.centre + s.u * sum(s.spec.u_range) / 2
    rig.move_to(*mid)
    strip, back = look(rig, model, s)
    if s.template is None:
        for cand in (f for f in vp.find(strip, back, s.spec) if f.state == "candidate"):
            try:
                tpl = vp.learn_template(strip, cand, s.spec)
            except ValueError:
                continue                                         # at the edge of the view: the next one
            if confirm(strip, cand):
                s.template = tpl
            break
        if s.template is None:
            dp.why = f"slot {s.slot}: no part confirmed (is the strip in its slot?)"
            return
    found = [f for f in vp.find(strip, back, s.spec, template=s.template) if f.state == "present"
             and round(f.u / s.spec.pitch) not in s.used and _inside(reach, pocket_xy(s, f))]
    if not found:
        dp.why = f"slot {s.slot} has no part left within reach: refill it"
        return
    f = min(found, key=lambda x: x.u)
    # 2. centre the camera on it and look again (the picture centre measures best)
    rig.move_to(*pocket_xy(s, f))
    strip, back = look(rig, model, s)
    again = [g for g in vp.find(strip, back, s.spec, template=s.template) if g.state == "present"]
    if not again:
        dp.why = "the part was not seen again from above it"
        return
    g = min(again, key=lambda x: abs(x.u - f.u))
    s.used.add(round(g.u / s.spec.pitch))
    xy = pocket_xy(s, g)
    dp.pick_xy, dp.pick_score = (float(xy[0]), float(xy[1])), g.score
    # 3. the place: over its pads on the registered board, are the pads there?
    c = board.component(dp.ref)
    target = reg.to_machine(board, (c.x, c.y))
    if not _inside(reach, target):
        dp.why = "its place is outside the camera's reach"
        return
    rig.move_to(*target)
    img = np.mean(np.asarray(rig.frames(2), np.float32), axis=0)
    head = np.asarray(rig.head(), float)[:2]
    dp.place_xy = (float(target[0]), float(target[1]))
    dp.pads_seen = pads_seen(img, model, board, reg, c, head, z_board)
    say(f"   {dp.ref}: pick at {xy[0]:.2f}, {xy[1]:.2f} (match {g.score:.2f}); place at {target[0]:.2f}, "
        f"{target[1]:.2f}: pads {'seen' if dp.pads_seen else 'NOT seen'}")


def pads_seen(img: np.ndarray, model: CameraModel, board: Board, reg: Registration, c, head, z: float,
              min_contrast: float = 25.0) -> bool:
    """The component's pads, drawn where the registration puts them, must be brighter than the board around."""
    import cv2
    h, w = img.shape
    pads = np.zeros((h, w), np.uint8)
    for pad in c.pads:
        q = model.xy_to_px([reg.to_machine(board, p) - head for p in pad.polygon], z)
        cv2.fillPoly(pads, [np.round(q).astype(np.int32)], 1)
    if not pads.any():
        return False
    ring = cv2.dilate(pads, np.ones((9, 9), np.uint8)) - cv2.dilate(pads, np.ones((5, 5), np.uint8))
    inner = cv2.erode(pads, np.ones((3, 3), np.uint8))
    a = img[inner > 0] if inner.any() else img[pads > 0]
    b = img[ring > 0]
    return bool(b.size and float(np.median(a)) - float(np.median(b)) > min_contrast)
