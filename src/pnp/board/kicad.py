"""KiCad .kicad_pcb importer (KiCad 6–10, S-expression; `pcbnew` not needed).

How KiCad stores a footprint (checked on the HT5023, KiCad 10):
- `(at X Y R)`: position in mm with Y DOWN, R = the angle shown in the editor (counter-clockwise on screen).
- pad `(at x y a)`: x, y local to the footprint (not rotated); a = the pad's absolute angle.
- a footprint on B.Cu is stored flipped top-to-bottom: its local Y is mirrored (the "-BL" SOT-23-6 on the bottom
  has pin 1 at local y < 0). So seen from below (turned over left-right) its angle is 180° − R (D10, D26).
"""
from __future__ import annotations

import gzip
import math
from pathlib import Path

from . import geometry as g
from .model import Board, Component, Fiducial, Hole, Pad, bottom_rotation_from_top_view
from .sexpr import Node, parse

FIELDS = {"mpn": ("MPN", "Manufacturer Part Number", "Manufacturer Part"),
          "manufacturer": ("Manufacturer", "Manufacurer"),
          "lcsc": ("LCSC Part", "LCSC", "Supplier Part Number", "Supplier Part", "Supplier Part  Number")}
EDGE = "Edge.Cuts"


def read_text(path: Path) -> str:
    data = path.read_bytes()
    if data[:2] == b"\x1f\x8b":
        data = gzip.decompress(data)
    return data.decode("utf-8", errors="replace")


def _xy(n: Node | None) -> g.Point:
    return (float(n[1]), float(n[2])) if n is not None else (0.0, 0.0)


def _local_to_up(p: g.Point) -> g.Point:
    """KiCad local (Y down) -> Y up."""
    return (p[0], -p[1])


class _Frame:
    """KiCad absolute (Y down) -> source Y-up -> board frame."""

    def __init__(self) -> None:
        self.x0 = self.y0 = 0.0

    def up(self, p: g.Point) -> g.Point:
        return (p[0], -p[1])

    def board(self, p_up: g.Point) -> g.Point:
        return (p_up[0] - self.x0, p_up[1] - self.y0)


# ---------------------------------------------------------------- graphics (outline)
def _graphic_points(n: Node) -> list[list[g.Point]]:
    """Edge.Cuts pieces in KiCad coordinates (Y down) as polylines."""
    kind = n.name.split("_", 1)[1] if "_" in n.name else n.name
    if kind == "line":
        return [[_xy(n.first("start")), _xy(n.first("end"))]]
    if kind == "arc":
        if n.first("mid") is not None:
            return [g.arc_points(_xy(n.first("start")), _xy(n.first("mid")), _xy(n.first("end")))]
        # KiCad 5: (start = centre) (end = arc start) (angle deg)
        c, s, ang = _xy(n.first("start")), _xy(n.first("end")), float(n.value("angle", 1, 0))
        r, a0 = math.hypot(s[0] - c[0], s[1] - c[1]), math.atan2(s[1] - c[1], s[0] - c[0])
        k = max(2, int(abs(ang) / 10) + 1)
        return [[(c[0] + r * math.cos(a0 + math.radians(ang) * i / k), c[1] + r * math.sin(a0 + math.radians(ang) * i / k))
                 for i in range(k + 1)]]
    if kind == "rect":
        (x1, y1), (x2, y2) = _xy(n.first("start")), _xy(n.first("end"))
        return [[(x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, y1)]]
    if kind == "circle":
        c, e = _xy(n.first("center")), _xy(n.first("end"))
        pts = g.circle(c[0], c[1], math.hypot(e[0] - c[0], e[1] - c[1]))
        return [pts + [pts[0]]]
    if kind in ("poly", "curve"):
        pts: list[g.Point] = []
        for item in (n.first("pts") or Node())[1:]:
            if isinstance(item, Node) and item.name == "xy":
                pts.append(_xy(item))
            elif isinstance(item, Node) and item.name == "arc":
                pts += g.arc_points(_xy(item.first("start")), _xy(item.first("mid")), _xy(item.first("end")))
        if kind == "poly" and pts:
            pts.append(pts[0])
        return [pts]
    return []


# ---------------------------------------------------------------- pads
def _pad_shape(p: Node) -> list[g.Point]:
    """The pad's outline in its own frame (Y up, not rotated)."""
    shape = p[3] if len(p) > 3 and isinstance(p[3], str) else "rect"
    size = p.floats("size") or [0.0, 0.0]
    w, h = size[0], size[1] if len(size) > 1 else size[0]
    if shape == "circle":
        poly = g.circle(0, 0, w / 2)
    elif shape == "oval":
        poly = g.oval(w, h)
    elif shape == "roundrect":
        poly = g.roundrect(w, h, float(p.value("roundrect_rratio", 1, 0.25)) * min(w, h))
    elif shape == "custom":
        pts: list[g.Point] = []
        anchor = (p.first("options") or Node()).value("anchor", 1, "circle")
        pts += g.circle(0, 0, w / 2) if anchor == "circle" else g.rect(w, h)
        for prim in (p.first("primitives") or Node())[1:]:
            if not isinstance(prim, Node):
                continue
            width = float(prim.value("width", 1, 0) or 0)
            for line in _graphic_points(prim):
                for q in line:
                    q = _local_to_up(q)
                    pts += [(q[0] + dx, q[1] + dy) for dx, dy in ((0, 0), (width / 2, 0), (-width / 2, 0),
                                                                    (0, width / 2), (0, -width / 2))]
        poly = g.hull(pts)
    else:                                                    # rect, trapezoid, chamfered: the rectangle
        poly = g.rect(w, h)
    off = (p.first("drill") or Node()).first("offset")
    if off is not None:
        ox, oy = _local_to_up(_xy(off))
        poly = [(x + ox, y + oy) for x, y in poly]
    return poly


def _drill(p: Node) -> float | None:
    d = p.first("drill")
    if d is None:
        return None
    nums = [float(v) for v in d[1:] if isinstance(v, str) and v != "oval" and g_is_num(v)]
    return min(nums) if nums else None


def g_is_num(v: str) -> bool:
    try:
        float(v)
        return True
    except ValueError:
        return False


def _pad(p: Node, fp_pos_up: g.Point, fp_angle: float, frame: _Frame, side: str) -> Pad | None:
    kind = p[2] if len(p) > 2 else ""
    layers = p.first("layers")
    names = layers.words() if layers is not None else []
    copper = [n for n in names if n.endswith(".Cu")]
    if kind == "connect" or (not copper and kind != "np_thru_hole"):
        return None                                          # paste / mask aperture only
    at = p.first("at")
    local = _local_to_up(_xy(at))
    pad_angle = float(at[3]) if at is not None and len(at) > 3 else 0.0
    cx, cy = g.rot(local, fp_angle)
    centre = (fp_pos_up[0] + cx, fp_pos_up[1] + cy)
    poly = [frame.board((centre[0] + q[0], centre[1] + q[1])) for q in (g.rot(v, pad_angle) for v in _pad_shape(p))]
    th = kind in ("thru_hole", "np_thru_hole")
    layer = "" if th or "*.Cu" in copper or ("F.Cu" in copper and "B.Cu" in copper) else (
        "B" if "B.Cu" in copper else "F")
    net = p.first("net")
    return Pad(number=str(p[1]), polygon=poly, through_hole=th, plated=kind != "np_thru_hole", drill=_drill(p),
               net=net[2] if net is not None and len(net) > 2 else "", layer=layer)


# ---------------------------------------------------------------- footprints
def _prop(fp: Node, name: str) -> str:
    for pr in fp.all("property"):
        if len(pr) > 2 and pr[1].lower() == name.lower():
            return pr[2].strip()
    for t in fp.all("fp_text"):                              # KiCad 6 / 7
        if len(t) > 2 and t[1] == name.lower():
            return t[2].strip()
    return ""


def _field(fp: Node, names) -> str:
    for n in names:
        v = _prop(fp, n)
        if v and v != "~":
            return v
    return ""


def load(path: Path, name: str | None = None) -> Board:
    root = parse(read_text(path))
    if root.name != "kicad_pcb":
        raise ValueError(f"{path.name} is not a KiCad board (.kicad_pcb)")
    frame = _Frame()
    b = Board(name=name or path.name.split(".")[0], source_tool="kicad", source_files=[path.name])
    b.conventions = {"kicad_version": root.value("version", 1, "?"),
                     "bottom_rotation": "KiCad angle R (seen from the top) -> 180° − R (seen from below, D10/D26)"}
    general = root.first("general")
    if general is not None and general.value("thickness") is not None:
        b.thickness, b.thickness_from = float(general.value("thickness")), "kicad general.thickness"

    # outline: board graphics and footprint graphics on Edge.Cuts
    pieces: list[list[g.Point]] = []
    for n in root[1:]:
        if isinstance(n, Node) and n.name.startswith("gr_") and n.value("layer") == EDGE:
            pieces += _graphic_points(n)
    footprints = root.all("footprint") + root.all("module")
    for fp in footprints:
        pos, ang = _xy(fp.first("at")), _angle(fp)
        for n in fp[1:]:
            if isinstance(n, Node) and n.name.startswith("fp_") and n.value("layer") == EDGE:
                for line in _graphic_points(n):
                    pieces.append([_fp_point(q, pos, ang) for q in line])
    loops_up = [[frame.up(p) for p in loop] for loop in g.chain(pieces)]
    if not loops_up:
        raise ValueError(f"{path.name}: no board outline on Edge.Cuts")
    outer = max(loops_up, key=lambda lp: abs(g.area(lp)))
    frame.x0, frame.y0, _, _ = g.bbox(outer)
    b.origin_in_source = (frame.x0, frame.y0)
    b.set_outline([[frame.board(p) for p in lp] for lp in loops_up])

    for fp in footprints:
        lib_id = fp[1] if len(fp) > 1 and isinstance(fp[1], str) else ""
        fp_name = lib_id.split(":")[-1]
        side = "B" if fp.value("layer") == "B.Cu" else "F"
        attr_node = fp.first("attr")
        attrs = attr_node.words() if attr_node is not None else []
        dnp = "dnp" in attrs or str(fp.value("dnp", 1, "no")).lower() in ("yes", "true")
        ref = _prop(fp, "Reference")
        value = _prop(fp, "Value")
        pos_up = frame.up(_xy(fp.first("at")))
        ang = _angle(fp)
        pads = [pd for pd in (_pad(p, pos_up, ang, frame, side) for p in fp.all("pad")) if pd is not None]
        x, y = frame.board(pos_up)
        if lib_id.split(":")[0] == "MountingHole" or fp_name.lower().startswith("mountinghole"):
            for pd in pads:
                if pd.drill:
                    c = pd.centre
                    b.holes.append(Hole(round(c[0], 4), round(c[1], 4), pd.drill, pd.plated, ref))
        if "fiducial" in fp_name.lower() or ref.upper().startswith("FID"):
            dia = max((g.bbox(pd.polygon)[2] - g.bbox(pd.polygon)[0] for pd in pads), default=1.0)
            b.fiducials.append(Fiducial(side, round(x, 4), round(y, 4), round(dia, 3), ref))
            continue
        if "exclude_from_pos_files" in attrs:
            b.notes.append(f"{ref or fp_name}: not a placed part (KiCad 'exclude from position files')")
            continue
        rot_top = g.norm_deg(ang)
        b.components.append(Component(
            ref=ref, value=value, footprint=fp_name, side=side, x=round(x, 4), y=round(y, 4),
            rotation=rot_top if side == "F" else bottom_rotation_from_top_view(rot_top), dnp=dnp, pads=pads,
            mpn=_field(fp, FIELDS["mpn"]), manufacturer=_field(fp, FIELDS["manufacturer"]),
            lcsc=_field(fp, FIELDS["lcsc"]), source_rotation=rot_top,
            source_flags={"kicad_attr": attrs, "lib": lib_id.split(":")[0]}))
    _kicad_attr_trap(b)
    return b


def _angle(fp: Node) -> float:
    at = fp.first("at")
    return float(at[3]) if at is not None and len(at) > 3 else 0.0


def _fp_point(q: g.Point, pos: g.Point, ang: float) -> g.Point:
    """A footprint-local point (Y down) -> KiCad absolute (Y down)."""
    v = g.rot(_local_to_up(q), ang)
    return (pos[0] + v[0], pos[1] - v[1])



def _kicad_attr_trap(b: Board) -> None:
    """docs/11: footprints that ARE SMD by their pads but not marked "smd" in KiCad (an "only SMD" export
    misses them). The model decides by the pads; the note says how many differ."""
    odd = [c.ref for c in b.components if c.smd and "smd" not in c.source_flags.get("kicad_attr", [])]
    if odd:
        b.notes.append(f"{len(odd)} parts are SMD by their pads but not marked SMD in KiCad (the model uses the pads): "
                       + ", ".join(sorted(odd)[:12]) + (" ..." if len(odd) > 12 else ""))
