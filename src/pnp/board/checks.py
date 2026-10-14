"""Checks after an import (docs/13 §4.3): what the operator must look at before the first place.

Two-pad parts carry the strongest check: in every library we know (KiCad, IPC-7351, LCSC / EasyEDA) a two-pad
part at 0° has its pads along X. So the line through its pads, seen from its side, must lie along its angle.
A wrong rotation rule (a centroid file read with the wrong bottom rule) shows up here as 90° off.

Pin 1 catches the 180° errors: an LCSC / EasyEDA footprint name says where pin 1 is at 0° (-BL bottom-left, -BR,
-TL, -TR), and a polar two-pad part (diode) has pad 1 on the −X side at 0° (KiCad, IPC-7351). Checked on the
HT5023: pin 1 lands where the name says on both sides, which also shows the bottom rule (D26) is right.
"""
from __future__ import annotations

import math
import re
from collections import Counter
from dataclasses import asdict, dataclass

from . import geometry as g
from .model import Board, Component

CENTRE_TOL_MM = 0.3
AXIS_TOL_DEG = 10.0
PIN1_QUADRANT = {"BL": 225.0, "BR": 315.0, "TL": 135.0, "TR": 45.0}
PIN1_NAME = re.compile(r"_L[\d.]+-W[\d.]+.*-(BL|BR|TL|TR)$")      # LCSC style only (not "LNK3206D-TL")


@dataclass
class Issue:
    level: str          # error | warn | info
    ref: str
    text: str

    def as_dict(self) -> dict:
        return asdict(self)


def pads_bbox_centre(c: Component) -> g.Point:
    pts = [p for pad in c.pads for p in pad.polygon]
    x0, y0, x1, y1 = g.bbox(pts)
    return ((x0 + x1) / 2, (y0 + y1) / 2)


def pad_axis_deg(b: Board, c: Component) -> float | None:
    """The angle of the line through a two-pad part's pads, seen from its side (mod 180)."""
    if len(c.pads) != 2:
        return None
    (x1, y1), (x2, y2) = (b.view(c.side, p.centre) for p in c.pads)
    if math.hypot(x2 - x1, y2 - y1) < 1e-6:
        return None
    return math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0


def pin1_expected(c: Component, polar: bool) -> float | None:
    """Where pin 1 should be at 0° (degrees from the part's +X), or None when nothing says so."""
    m = PIN1_NAME.search(c.footprint)
    if m:
        return PIN1_QUADRANT[m.group(1)]
    if polar and len(c.pads) == 2 and {p.number for p in c.pads} == {"1", "2"}:
        return 180.0
    return None


def pin1_angle(b: Board, c: Component) -> float | None:
    """Where pin 1 is, in degrees from the part's +X, seen from its side."""
    p1 = next((p for p in c.pads if p.number == "1"), None)
    if p1 is None:
        return None
    x, y = b.view(c.side, p1.centre)
    cx, cy = b.view(c.side, (c.x, c.y))
    if math.hypot(x - cx, y - cy) < 1e-6:
        return None
    return (math.degrees(math.atan2(y - cy, x - cx)) - c.rotation) % 360.0


def check(b: Board, classes: dict[str, dict] | None = None, polar: set[str] | None = None) -> list[Issue]:
    """polar: the package ids whose orientation matters (from the package library)."""
    out: list[Issue] = []
    for ref, n in Counter(c.ref for c in b.components).items():
        if n > 1:
            out.append(Issue("error", ref, f"{ref} is on the board {n} times: every part needs its own reference"))
    outer = b.outline[0] if b.outline else None
    for c in b.components:
        cls = (classes or {}).get(c.ref, {}).get("class")
        if outer is not None and not g.inside((c.x, c.y), outer):
            out.append(Issue("warn", c.ref, f"{c.ref} is outside the board outline: wrong origin or units?"))
        if len(c.pads) >= 2:
            cx, cy = pads_bbox_centre(c)
            d = math.hypot(cx - c.x, cy - c.y)
            if d > CENTRE_TOL_MM and cls in ("machine", "bottom_camera", None):
                out.append(Issue("warn", c.ref, f"{c.ref}: its centre is {d:.2f} mm from the middle of its pads"))
        axis = pad_axis_deg(b, c)
        if axis is not None and cls in ("machine", "bottom_camera", None):
            off = abs(((c.rotation % 180.0) - axis + 90.0) % 180.0 - 90.0)
            if off > AXIS_TOL_DEG:
                out.append(Issue("warn", c.ref, f"{c.ref}: its pads lie at {axis:.0f}°, its angle is {c.rotation:.0f}°"
                                 f" ({off:.0f}° off): check the rotation rule"))
        want = pin1_expected(c, c.package in (polar or set()))
        got = pin1_angle(b, c) if want is not None else None
        if want is not None and got is not None and cls in ("machine", "bottom_camera", "by_hand", None):
            off = abs((got - want + 180.0) % 360.0 - 180.0)
            if off > 60.0:
                out.append(Issue("warn", c.ref, f"{c.ref}: pin 1 is {off:.0f}° away from where its footprint puts it"
                                 ": the angle may be 180° off (check the rotation rule)"))
        if not c.pads and not c.dnp:
            out.append(Issue("info", c.ref, f"{c.ref}: no pads in the files"))
    if b.thickness_from == "default":
        out.append(Issue("info", "", f"Board thickness not in the files: {b.thickness} mm is used (Board tab)"))
    return out
