"""Centroid / pick-and-place text files (KiCad .pos / .csv, Altium, EasyEDA, JLC-style CSV, Eagle .mnt / .mnb, any
CSV): a column mapper with auto-detection, units, side names, origin and the bottom rotation rule.

Nothing is guessed silently (docs/13 §4.3): `sniff` proposes a CentroidSpec and says what it is not sure about;
`suggest` fits the origin, the Y direction, the bottom X sign and the rotation rules to a reference board (the
same board from KiCad or Gerber X3) and reports how well each choice fits; the operator confirms in the preview.
"""
from __future__ import annotations

import re
import statistics
from dataclasses import asdict, dataclass, field, replace

from . import geometry as g
from .model import Board, Component

UNITS = {"mm": 1.0, "mil": 0.0254, "inch": 25.4, "in": 25.4}
# bottom rotation rules: the model's angle (seen from below, turned over left-right, D26) from the file's angle R
BOTTOM_RULES = {
    "top_view": ("180 − R", "the angle as seen from the TOP (KiCad .pos / .csv, the KiCad editor)"),
    "negated": ("−R", "KiCad's Gerber X3 rule"),
    "as_is": ("R", "already seen from below, board turned over left-right"),
    "plus_180": ("R + 180", "seen from below, board turned over top-to-bottom"),
}
ROLE_ALIASES = {
    "ref": ["designator", "ref", "refdes", "reference", "part", "name", "component", "comp", "partname"],
    "x": ["midx", "centerx", "centrex", "posx", "x", "locationx", "centx", "refx"],
    "y": ["midy", "centery", "centrey", "posy", "y", "locationy", "centy", "refy"],
    "rotation": ["rotation", "rot", "angle", "orientation", "rotate"],
    "side": ["layer", "side", "tb", "placement", "mirror"],
    "value": ["value", "val", "comment", "partvalue", "partnumber"],
    "package": ["footprint", "package", "pattern", "partdecal", "fp", "device"],
}
TOP_NAMES = {"top", "t", "f", "front", "toplayer", "component", "componentside", "1"}
BOTTOM_NAMES = {"bottom", "b", "bot", "back", "bottomlayer", "solder", "solderside", "2"}


@dataclass
class CentroidSpec:
    columns: dict[str, int]                      # role -> column index (ref, x, y required)
    header_row: int = -1                         # index in the table lines (-1 = no header line)
    delimiter: str = ","                         # "," ";" "\t" or "ws" (spaces)
    units: str = "mm"
    y_up: bool = True                            # the file's Y grows to the board's top edge
    bottom_x_negated: bool = False               # bottom X written as −X (a KiCad / Altium option)
    origin: tuple[float, float] = (0.0, 0.0)     # the board origin in the file's coordinates (after units, signs)
    bottom_rule: str = "top_view"
    top_offset: float = 0.0                      # added to every top angle (a tool with another zero)
    tool: str = ""
    side_default: str = "F"                      # when there is no side column (Eagle .mnt = top, .mnb = bottom)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> CentroidSpec:
        d = dict(d)
        d["columns"] = {k: int(v) for k, v in d.get("columns", {}).items() if v is not None and v != ""}
        d["origin"] = tuple(d.get("origin", (0.0, 0.0)))
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class Row:
    ref: str
    x: float
    y: float
    rotation: float
    side: str
    value: str = ""
    package: str = ""


@dataclass
class Sniff:
    spec: CentroidSpec
    header: list[str]
    sample: list[list[str]]
    unsure: list[str] = field(default_factory=list)   # what the operator must check


def decode(data: bytes) -> str:
    if data[:2] in (b"\xff\xfe", b"\xfe\xff"):
        return data.decode("utf-16")
    if data[:3] == b"\xef\xbb\xbf":
        return data[3:].decode("utf-8", errors="replace")
    if len(data) > 4 and data[1:2] == b"\x00" and data[3:4] == b"\x00":
        return data.decode("utf-16-le", errors="replace")
    return data.decode("utf-8", errors="replace")


def _norm(h: str) -> str:
    h = re.sub(r"\(.*?\)|\[.*?\]", "", h.lower())
    return re.sub(r"[^a-z0-9]", "", h)


def _split(line: str, delim: str) -> list[str]:
    if delim == "ws":
        return line.split()
    cells, cur, q = [], "", False
    for ch in line:
        if ch == '"':
            q = not q
        elif ch == delim and not q:
            cells.append(cur.strip())
            cur = ""
        else:
            cur += ch
    cells.append(cur.strip())
    return [c.strip().strip('"') for c in cells]


def _match_columns(header: list[str]) -> dict[str, int]:
    norm = [_norm(h) for h in header]
    cols: dict[str, int] = {}
    for role, aliases in ROLE_ALIASES.items():
        for a in aliases:                                    # the first alias that is there wins (priority order)
            if a in norm and norm.index(a) not in cols.values():
                cols[role] = norm.index(a)
                break
    return cols


def sniff(data: bytes | str, filename: str = "") -> Sniff:
    text = decode(data) if isinstance(data, bytes) else data
    lines = [ln.rstrip("\r") for ln in text.splitlines()]
    unsure: list[str] = []
    tool = ""
    if "Altium" in text[:500]:
        tool = "altium"
    elif "### Footprint positions" in text[:300] or re.match(r"^Ref,Val,Package,PosX,PosY,Rot,Side", text.strip()):
        tool = "kicad"
    best = (-1, -1, ",", {})
    for i, ln in enumerate(lines[:60]):
        if not ln.strip():
            continue
        clean = ln.lstrip("#").strip()
        for d in ("\t", ",", ";", "ws"):
            cells = _split(clean, d)
            if len(cells) < 3:
                continue
            cols = _match_columns(cells)
            if {"ref", "x", "y"} <= set(cols) and len(cols) > best[0]:
                best = (len(cols), i, d, cols)
    if best[1] < 0:                                          # no header: Eagle .mnt style "R1 10.16 20.32 90 10k R0805"
        rows = [ln.split() for ln in lines if ln.strip() and not ln.startswith("#")]
        if rows and all(len(r) >= 4 and _is_num(r[1]) and _is_num(r[2]) and _is_num(r[3]) for r in rows[:5]):
            tool = tool or "eagle_mnt"
            spec = CentroidSpec({"ref": 0, "x": 1, "y": 2, "rotation": 3, "value": 4, "package": 5}, -1, "ws",
                                tool=tool, side_default="B" if filename.lower().endswith(".mnb") else "F")
            unsure.append("No header line: columns read as name, x, y, angle, value, package (Eagle .mnt). Check them")
            unsure.append("Units: Eagle writes mm by default. Check the preview")
            return Sniff(spec, [], rows[:8], unsure)
        raise ValueError(f"{filename or 'The file'}: no header line with a reference, X and Y column")
    _, hi, delim, cols = best
    header = _split(lines[hi].lstrip("#").strip(), delim)
    norm = [_norm(h) for h in header]
    if {"midx", "refx", "padx"} <= set(norm):
        tool = tool or "easyeda"
    elif "midx" in norm and tool == "":
        tool = "jlc_csv"
    rows = [_split(ln.strip(), delim) for ln in lines[hi + 1:] if ln.strip() and not ln.lstrip().startswith("#")]
    units = _units(text, header, cols, rows)
    if units is None:
        units = "mm"
        unsure.append("Units not written in the file: read as mm")
    spec = CentroidSpec(cols, hi, delim, units, tool=tool)
    if "side" not in cols:
        unsure.append("No side / layer column: every part is read as top. Choose the side")
    if "rotation" not in cols:
        unsure.append("No rotation column: every angle is 0")
    if tool != "kicad":
        unsure.append(f"Bottom rotation rule for {tool or 'this tool'} is not checked: confirm it in the preview "
                      "(or let it be fitted to a reference board)")
    return Sniff(spec, header, rows[:8], unsure)


def _is_num(v: str) -> bool:
    try:
        float(re.sub(r"(mm|mil|in|inch)$", "", v.strip().lower()))
        return True
    except ValueError:
        return False


def _units(text: str, header: list[str], cols: dict, rows: list[list[str]]) -> str | None:
    hx = header[cols["x"]].lower() if "x" in cols else ""
    for u in ("mm", "mil", "inch", "in"):
        if f"({u})" in hx or f"[{u}]" in hx:
            return "inch" if u == "in" else u
    m = re.search(r"(?:Unit\s*=|Units used:)\s*(mm|mil|in|inch)", text[:2000], re.I)
    if m:
        u = m.group(1).lower()
        return "inch" if u == "in" else u
    for r in rows[:5]:
        if "x" in cols and cols["x"] < len(r):
            m = re.search(r"(mm|mil|inch|in)$", r[cols["x"]].strip().lower())
            if m:
                return "inch" if m.group(1) == "in" else m.group(1)
    return None


def _num(v: str) -> float:
    return float(re.sub(r"(mm|mil|inch|in)$", "", v.strip().lower()) or 0)


def rows(data: bytes | str, spec: CentroidSpec) -> list[Row]:
    text = decode(data) if isinstance(data, bytes) else data
    lines = [ln.rstrip("\r") for ln in text.splitlines()]
    body = lines[spec.header_row + 1:] if spec.header_row >= 0 else lines
    out = []
    k = UNITS[spec.units]
    c = spec.columns
    for ln in body:
        if not ln.strip() or ln.lstrip().startswith("#"):
            continue
        cells = _split(ln.strip(), spec.delimiter)
        try:
            ref = cells[c["ref"]]
            x, y = _num(cells[c["x"]]) * k, _num(cells[c["y"]]) * k
        except (IndexError, ValueError):
            continue
        side = spec.side_default
        if "side" in c and c["side"] < len(cells):
            s = _norm(cells[c["side"]])
            side = "B" if s in BOTTOM_NAMES else ("F" if s in TOP_NAMES else spec.side_default)
        rot = _num(cells[c["rotation"]]) if "rotation" in c and c["rotation"] < len(cells) and cells[c["rotation"]] else 0.0
        out.append(Row(ref, x, y, rot, side, cells[c["value"]] if "value" in c and c["value"] < len(cells) else "",
                       cells[c["package"]] if "package" in c and c["package"] < len(cells) else ""))
    return out


def model_rotation(r: float, side: str, spec: CentroidSpec) -> float:
    if side == "F":
        return g.norm_deg(r + spec.top_offset)
    return g.norm_deg({"top_view": 180.0 - r, "negated": -r, "as_is": r, "plus_180": r + 180.0}[spec.bottom_rule])


def source_xy(row: Row, spec: CentroidSpec) -> g.Point:
    """The row's position in the file's coordinates with Y up and bottom X not negated (before the origin)."""
    x = -row.x if (row.side == "B" and spec.bottom_x_negated) else row.x
    y = row.y if spec.y_up else -row.y
    return (x, y)


def to_board(rs: list[Row], spec: CentroidSpec, name: str, reference: Board | None = None,
             filename: str = "") -> Board:
    """The placements of the file; the outline, pads and holes come from the reference board when given."""
    b = Board(name=name, source_tool=f"centroid:{spec.tool or 'csv'}", source_files=[filename] if filename else [])
    b.conventions = {"units": spec.units, "y_up": spec.y_up, "bottom_x_negated": spec.bottom_x_negated,
                     "origin": list(spec.origin), "bottom_rotation": f"{spec.bottom_rule}: model = "
                     f"{BOTTOM_RULES[spec.bottom_rule][0]}", "top_offset": spec.top_offset}
    ref = reference.by_ref() if reference is not None else {}
    for r in rs:
        sx, sy = source_xy(r, spec)
        x, y = sx - spec.origin[0], sy - spec.origin[1]
        c = Component(ref=r.ref, value=r.value, footprint=r.package, side=r.side, x=round(x, 4), y=round(y, 4),
                      rotation=model_rotation(r.rotation, r.side, spec), source_rotation=r.rotation)
        other = ref.get(r.ref)
        if other is not None:
            c.pads = [replace(p) for p in other.pads]
            c.dnp, c.mpn, c.manufacturer, c.lcsc, c.height = other.dnp, other.mpn, other.manufacturer, other.lcsc, \
                other.height
        b.components.append(c)
    if reference is not None:
        b.outline, b.width, b.height = reference.outline, reference.width, reference.height
        b.holes, b.fiducials = list(reference.holes), list(reference.fiducials)
        b.thickness, b.thickness_from = reference.thickness, f"reference ({reference.source_tool})"
        b.origin_in_source = spec.origin
        missing = sorted(set(ref) - {r.ref for r in rs})
        extra = sorted({r.ref for r in rs} - set(ref))
        if missing:
            b.notes.append(f"{len(missing)} parts of the reference are not in the file: {', '.join(missing[:10])}")
        if extra:
            b.notes.append(f"{len(extra)} parts are not in the reference (no pads, no outline check): "
                           f"{', '.join(extra[:10])}")
    else:
        xs, ys = [c.x for c in b.components], [c.y for c in b.components]
        if xs:
            b.width, b.height = max(xs) - min(xs), max(ys) - min(ys)
        b.notes.append("No reference board: no outline and no pads. The origin is the file's own; add the Gerbers or "
                       "the KiCad file as the reference to check the parts on their pads")
    return b


# ---------------------------------------------------------------- fitting to a reference
@dataclass
class Fit:
    spec: CentroidSpec
    matched: int
    position_error: float            # median distance after the fit, mm
    rotation_agree: float            # share of matched parts whose angle agrees
    report: list[str]


def suggest(rs: list[Row], spec: CentroidSpec, reference: Board) -> Fit:
    """Fit Y direction, bottom X sign, origin and the rotation rules to the reference. Every choice is reported."""
    ref = reference.by_ref()
    pairs = [(r, ref[r.ref]) for r in rs if r.ref in ref]
    if len(pairs) < 2:
        return Fit(spec, len(pairs), float("inf"), 0.0, ["Too few parts with the same reference as the board: "
                                                         "the origin and rules can't be fitted"])
    best: Fit | None = None
    for y_up in (True, False):
        for neg in (False, True):
            s = replace(spec, y_up=y_up, bottom_x_negated=neg)
            dx = [source_xy(r, s)[0] - c.x for r, c in pairs]
            dy = [source_xy(r, s)[1] - c.y for r, c in pairs]
            origin = (statistics.median(dx), statistics.median(dy))
            err = statistics.median(math_hypot(source_xy(r, s)[0] - origin[0] - c.x,
                                               source_xy(r, s)[1] - origin[1] - c.y) for r, c in pairs)
            s = replace(s, origin=(round(origin[0], 4), round(origin[1], 4)))
            if best is None or err < best.position_error - 1e-6:
                best = Fit(s, len(pairs), err, 0.0, [])
    s = best.spec
    same = [(r, c) for r, c in pairs if r.side == c.side or c.source_flags.get("side_unknown")]
    top = [(r, c) for r, c in same if r.side == "F"]
    bot = [(r, c) for r, c in same if r.side == "B"]

    known = [(r, c) for r, c in top + bot if not c.source_flags.get("rotation_unknown")]
    by_axis = not known                                     # the reference has no angles: use the pads' direction

    def agree(rule_spec, group):
        if by_axis:
            group = [(r, c) for r, c in group if len(c.pads) == 2]
        if not group:
            return 1.0
        if by_axis:
            ok = 0
            for r, c in group:
                axis = _axis(reference, c, r.side)
                rot = model_rotation(r.rotation, r.side, rule_spec)
                ok += axis is not None and abs(((rot % 180.0) - axis + 90.0) % 180.0 - 90.0) < 10.0
            return ok / len(group)
        return sum(g.angle_diff(model_rotation(r.rotation, r.side, rule_spec), c.rotation) < 0.5 for r, c in group) / len(group)

    off = max((0.0, 90.0, 180.0, -90.0), key=lambda o: agree(replace(s, top_offset=o), top))
    s = replace(s, top_offset=off)
    rule = max(BOTTOM_RULES, key=lambda k: (agree(replace(s, bottom_rule=k), bot), k == spec.bottom_rule))
    s = replace(s, bottom_rule=rule)
    agree_all = agree(s, top + bot)
    side_bad = sum(r.side != c.side for r, c in pairs if not c.source_flags.get("side_unknown"))
    report = [f"{len(pairs)} of {len(rs)} parts matched the reference by their reference designator",
              f"Y {'up' if s.y_up else 'down'}, bottom X {'negated' if s.bottom_x_negated else 'as written'}, origin "
              f"({s.origin[0]:.3f}, {s.origin[1]:.3f}) in the file: median position error {best.position_error:.3f} mm",
              f"Top angles {'as written' if off == 0 else f'+{off:g}°'}; bottom rule '{rule}' (model = "
              f"{BOTTOM_RULES[rule][0]}): {agree_all * 100:.0f} % of the angles agree with the reference"]
    if side_bad:
        report.append(f"{side_bad} parts are on the other side in the reference: check the side names")
    if by_axis:
        report.append("The reference has no angles (Gerbers without X3): the angles were checked by the direction of "
                      "the pads only, so a 180° error is NOT ruled out. Check the polar parts (diodes, ICs) in the "
                      "preview")
    return Fit(s, len(pairs), best.position_error, agree_all, report)


def _axis(ref: Board, c: Component, side: str) -> float | None:
    import math
    (x1, y1), (x2, y2) = (ref.view(side, p.centre) for p in c.pads)
    if abs(x2 - x1) + abs(y2 - y1) < 1e-6:
        return None
    return math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180.0


def math_hypot(a: float, b: float) -> float:
    return (a * a + b * b) ** 0.5
