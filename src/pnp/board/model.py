"""The Board model (docs/13 §4.3): one format for every PCB tool.

Board frame: mm, X right, Y up, origin = the bottom-left corner of the outline's bounding box, seen from the TOP.
Rotation: degrees, counter-clockwise **seen from the side the part is on**, 0° = the package library's zero.
  For a part on the bottom (B) the view from below is the board turned over left-right (around its Y axis):
  view x = width − x, view y = y (D26). This is D10's rule for KiCad: B rotation = 180° − KiCad's angle.
Pads: polygons in the board frame (seen from the top), on the copper side that matters.
`smd` comes from the pads (no plated or non-plated through-hole pad), never from a tool's "SMD" flag (docs/11).
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

from .geometry import Point, bbox, centroid, norm_deg

MODEL_VERSION = 1


@dataclass
class Pad:
    number: str
    polygon: list[Point]
    through_hole: bool = False
    plated: bool = True
    drill: float | None = None
    net: str = ""
    layer: str = ""                      # "F" / "B" copper side ("" = both, a through-hole pad)

    @property
    def centre(self) -> Point:
        return centroid(self.polygon)


@dataclass
class Component:
    ref: str
    value: str
    footprint: str                       # as the source tool calls it
    side: str                            # "F" top / "B" bottom
    x: float
    y: float
    rotation: float                      # CCW seen from its side, library zero
    package: str = ""                    # the package library's id ("" = not known)
    dnp: bool = False
    pads: list[Pad] = field(default_factory=list)
    height: float | None = None
    mpn: str = ""
    manufacturer: str = ""
    lcsc: str = ""
    source_rotation: float | None = None  # the angle as the source file has it (traceability)
    source_flags: dict = field(default_factory=dict)   # e.g. {"kicad_attr": ["smd"]}
    smd_hint: bool | None = None         # what the source says when there are no pads (X3 CMnt, a CSV)

    @property
    def smd(self) -> bool:
        """No through-hole pads. Without pads: the source's hint (or False: never guess "machine")."""
        if self.pads:
            return not any(p.through_hole for p in self.pads)
        return bool(self.smd_hint)

    @property
    def tht(self) -> bool:
        return any(p.through_hole and p.plated for p in self.pads)


@dataclass
class Hole:
    x: float
    y: float
    diameter: float
    plated: bool = False
    ref: str = ""


@dataclass
class Fiducial:
    side: str
    x: float
    y: float
    diameter: float
    ref: str = ""


@dataclass
class Board:
    name: str
    source_tool: str                     # "kicad", "gerber_x3", "centroid", ...
    source_files: list[str] = field(default_factory=list)
    outline: list[list[Point]] = field(default_factory=list)    # [0] = the outer outline, then cut-outs
    width: float = 0.0
    height: float = 0.0
    thickness: float = 1.6
    thickness_from: str = "default"      # where the thickness came from ("default" = not in the files)
    holes: list[Hole] = field(default_factory=list)
    fiducials: list[Fiducial] = field(default_factory=list)
    components: list[Component] = field(default_factory=list)
    origin_in_source: Point = (0.0, 0.0)  # the board origin in the source's Y-up coordinates
    conventions: dict = field(default_factory=dict)   # how rotations / sides were read (shown in the preview)
    notes: list[str] = field(default_factory=list)    # what the importer wants the operator to know
    version: int = MODEL_VERSION

    # ---------------------------------------------------------------- lookups
    def component(self, ref: str) -> Component | None:
        return next((c for c in self.components if c.ref == ref), None)

    def by_ref(self) -> dict[str, Component]:
        return {c.ref: c for c in self.components}

    def view(self, side: str, p: Point) -> Point:
        """A board point as seen from `side` (B: turned over left-right)."""
        return p if side == "F" else (self.width - p[0], p[1])

    def summary(self) -> dict:
        smd = [c for c in self.components if c.smd and not c.dnp]
        return {"components": len(self.components), "smd": len(smd),
                "smd_top": sum(c.side == "F" for c in smd), "smd_bottom": sum(c.side == "B" for c in smd),
                "tht": sum(c.tht for c in self.components), "dnp": sum(c.dnp for c in self.components),
                "holes": len(self.holes), "fiducials": len(self.fiducials),
                "size": [round(self.width, 3), round(self.height, 3)], "thickness": self.thickness}

    # ---------------------------------------------------------------- set-up helpers for importers
    def set_outline(self, loops: list[list[Point]]) -> None:
        """Loops in the board frame; the biggest one becomes the outer outline."""
        from .geometry import area
        loops = sorted((lp for lp in loops if len(lp) >= 3), key=lambda lp: -abs(area(lp)))
        self.outline = loops
        if loops:
            x0, y0, x1, y1 = bbox(loops[0])
            self.width, self.height = x1 - x0, y1 - y0

    # ---------------------------------------------------------------- files
    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> Board:
        d = dict(d)
        comps = []
        for c in d.pop("components", []):
            c = dict(c)
            pads = [Pad(**{**p, "polygon": [tuple(q) for q in p["polygon"]]}) for p in c.pop("pads", [])]
            comps.append(Component(**c, pads=pads))
        b = cls(**{k: v for k, v in d.items() if k not in ("holes", "fiducials", "outline", "origin_in_source")},
                components=comps)
        b.outline = [[tuple(p) for p in lp] for lp in d.get("outline", [])]
        b.holes = [Hole(**h) for h in d.get("holes", [])]
        b.fiducials = [Fiducial(**f) for f in d.get("fiducials", [])]
        b.origin_in_source = tuple(d.get("origin_in_source", (0.0, 0.0)))
        return b

    def save(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.to_dict(), separators=(",", ":")))
        tmp.replace(path)

    @classmethod
    def load(cls, path: Path) -> Board:
        return cls.from_dict(json.loads(path.read_text()))


def bottom_rotation_from_top_view(angle_top: float) -> float:
    """A bottom part's angle as the source shows it seen from the TOP (KiCad's editor and .pos/.csv) ->
    the model's angle, seen from below (D26: 180° − angle)."""
    return norm_deg(180.0 - angle_top)
