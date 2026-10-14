"""Gerber (RS-274X with X2 / X3 attributes) and Excellon readers, just enough for boards:

- attributes: file (TF), aperture (TA) and object (TO) attributes, deleted with TD;
- apertures: C, R, O, P and aperture macros (primitives 1, 4, 5, 20, 21, with $-expressions);
- flashes (D03), draws (D01, linear and circular arcs) and regions (G36 / G37);
- every flashed or region object keeps its polygon (a convex hull for macros: good for pads) and the object
  attributes that were active (TO.P = pad of ref / pin, TO.C... = an X3 component).
Not supported (the reader says so): step-and-repeat (SR), block apertures (AB), image transformations (LM/LR/LS).
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

from . import geometry as g

INCH = 25.4


@dataclass
class Aperture:
    code: str
    kind: str                         # C, R, O, P or a macro name
    params: list[float]
    attrs: dict[str, list[str]]       # the TA attributes when it was defined
    unit: float = 1.0                 # mm per file unit (macro shapes are scaled by it)


@dataclass
class GObject:
    kind: str                         # flash, draw, region
    points: list[g.Point]             # flash: [centre]; draw: the polyline; region: the contour
    aperture: Aperture | None
    attrs: dict[str, list[str]]       # TO attributes
    dark: bool = True

    @property
    def function(self) -> str:
        a = (self.aperture.attrs.get(".AperFunction") if self.aperture else None) or []
        return a[0] if a else ""

    def polygon(self, macros: dict) -> list[g.Point]:
        if self.kind == "region":
            return self.points
        if self.kind == "flash" and self.aperture is not None:
            cx, cy = self.points[0]
            return [(cx + x, cy + y) for x, y in aperture_shape(self.aperture, macros)]
        return self.points


@dataclass
class GerberFile:
    name: str
    file_attrs: dict[str, list[str]] = field(default_factory=dict)
    objects: list[GObject] = field(default_factory=list)
    macros: dict[str, list[str]] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)

    @property
    def function(self) -> list[str]:
        return self.file_attrs.get(".FileFunction", [])

    @property
    def generator(self) -> str:
        return ",".join(self.file_attrs.get(".GenerationSoftware", []))


# ---------------------------------------------------------------- macro arithmetic
_EXPR = re.compile(r"\s*([0-9.]+|\$[0-9]+|[-+xX/()])")


def _eval(expr: str, var: dict[int, float]) -> float:
    toks = _EXPR.findall(expr.replace(" ", ""))
    pos = 0

    def peek():
        return toks[pos] if pos < len(toks) else None

    def take():
        nonlocal pos
        pos += 1
        return toks[pos - 1]

    def factor():
        t = take()
        if t == "-":
            return -factor()
        if t == "+":
            return factor()
        if t == "(":
            v = add()
            take()
            return v
        if t.startswith("$"):
            return var.get(int(t[1:]), 0.0)
        return float(t)

    def mul():
        v = factor()
        while peek() in ("x", "X", "/"):
            op = take()
            v = v * factor() if op in "xX" else v / factor()
        return v

    def add():
        v = mul()
        while peek() in ("+", "-"):
            v = v + mul() if take() == "+" else v - mul()
        return v

    return add() if toks else 0.0


def macro_points(body: list[str], params: list[float]) -> list[g.Point]:
    """All points of the macro's dark primitives (their hull is the aperture's shape)."""
    var = {i + 1: v for i, v in enumerate(params)}
    pts: list[g.Point] = []
    for stmt in body:
        stmt = stmt.strip()
        if not stmt or stmt.startswith("0"):
            continue
        if stmt.startswith("$"):
            k, _, e = stmt.partition("=")
            var[int(k[1:])] = _eval(e, var)
            continue
        f = [_eval(x, var) for x in stmt.split(",")]
        code = int(f[0])
        if len(f) < 2 or f[1] == 0:
            continue                                          # exposure off: a hole in the shape
        if code == 1:                                         # circle: exposure, diameter, x, y[, rot]
            c = (f[3], f[4])
            if len(f) > 5:
                c = g.rot(c, f[5])
            pts += g.circle(c[0], c[1], f[2] / 2, 16)
        elif code == 4:                                       # outline: exposure, n, x0, y0 ... xn, yn, rot
            n = int(f[2])
            rotd = f[5 + 2 * n] if len(f) > 5 + 2 * n else 0.0
            pts += [g.rot((f[3 + 2 * i], f[4 + 2 * i]), rotd) for i in range(n + 1)]
        elif code == 5:                                       # polygon: exposure, n, x, y, diameter, rot
            n, cx, cy, d, rotd = int(f[2]), f[3], f[4], f[5], f[6] if len(f) > 6 else 0.0
            pts += [g.rot((cx + d / 2 * math.cos(2 * math.pi * i / n), cy + d / 2 * math.sin(2 * math.pi * i / n)), rotd)
                    for i in range(n)]
        elif code == 20:                                      # vector line: exposure, width, x1, y1, x2, y2, rot
            w, x1, y1, x2, y2, rotd = f[2], f[3], f[4], f[5], f[6], f[7] if len(f) > 7 else 0.0
            dx, dy = x2 - x1, y2 - y1
            L = math.hypot(dx, dy) or 1.0
            nx, ny = -dy / L * w / 2, dx / L * w / 2
            pts += [g.rot(p, rotd) for p in ((x1 + nx, y1 + ny), (x2 + nx, y2 + ny), (x2 - nx, y2 - ny), (x1 - nx, y1 - ny))]
        elif code == 21:                                      # centre line: exposure, w, h, x, y, rot
            w, h, cx, cy, rotd = f[2], f[3], f[4], f[5], f[6] if len(f) > 6 else 0.0
            pts += [g.rot((cx + x, cy + y), rotd) for x, y in g.rect(w, h)]
    return pts


def aperture_shape(a: Aperture, macros: dict) -> list[g.Point]:
    p = a.params
    if a.kind == "C":
        return g.circle(0, 0, p[0] / 2, 24)
    if a.kind == "R":
        return g.rect(p[0], p[1])
    if a.kind == "O":
        return g.oval(p[0], p[1])
    if a.kind == "P":
        n, rotd = int(p[1]), p[2] if len(p) > 2 else 0.0
        return [g.rot((p[0] / 2 * math.cos(2 * math.pi * i / n), p[0] / 2 * math.sin(2 * math.pi * i / n)), rotd)
                for i in range(n)]
    if a.kind in macros:
        return [(x * a.unit, y * a.unit) for x, y in g.hull(macro_points(macros[a.kind], p))]
    return g.circle(0, 0, 0.05, 8)


# ---------------------------------------------------------------- the reader
_BLOCK = re.compile(r"%([^%]*)%|([^%*]*)\*", re.S)
_WORD = re.compile(r"([XYIJD])([+-]?\d+(?:\.\d*)?)")


def parse(text: str, name: str = "") -> GerberFile:
    gf = GerberFile(name)
    unit = 1.0
    fmt = (4, 6)
    apertures: dict[str, Aperture] = {}
    ta: dict[str, list[str]] = {}
    to: dict[str, list[str]] = {}
    cur: Aperture | None = None
    x = y = 0.0
    interp = "G01"
    in_region = False
    contour: list[g.Point] = []
    contours: list[list[g.Point]] = []
    dark = True
    draw: list[g.Point] = []

    def num(v: str, axis: int) -> float:
        if "." in v:
            return float(v) * unit
        return int(v) / 10 ** fmt[axis] * unit

    def flush_draw():
        nonlocal draw
        if len(draw) >= 2 and not in_region:
            gf.objects.append(GObject("draw", draw, cur, dict(to), dark))
        draw = []

    def close_contour():
        nonlocal contour
        if len(contour) >= 3:
            contours.append(contour)
        contour = []

    for m in _BLOCK.finditer(text):
        if m.group(1) is not None:                            # extended command block
            cmds = [c.strip() for c in m.group(1).split("*") if c.strip()]
            if cmds and cmds[0].startswith("AM"):             # a macro: the rest of this block is its body
                gf.macros[cmds[0][2:]] = cmds[1:]
                continue
            for cmd in cmds:
                if cmd.startswith("FS"):
                    mm = re.search(r"X(\d)(\d)Y(\d)(\d)", cmd)
                    if mm:
                        fmt = (int(mm.group(2)), int(mm.group(4)))
                elif cmd.startswith("MO"):
                    unit = INCH if cmd[2:4] == "IN" else 1.0
                elif cmd.startswith("TF"):
                    k, *v = cmd[2:].split(",")
                    gf.file_attrs[k] = v
                elif cmd.startswith("TA"):
                    k, *v = cmd[2:].split(",")
                    ta[k] = v
                elif cmd.startswith("TO"):
                    k, *v = cmd[2:].split(",")
                    to[k] = v
                elif cmd.startswith("TD"):
                    k = cmd[2:]
                    if k:
                        ta.pop(k, None)
                        to.pop(k, None)
                    else:
                        ta.clear()
                        to.clear()
                elif cmd.startswith("AD"):
                    mm = re.match(r"ADD(\d+)([A-Za-z_.$][\w.$]*)(?:,(.*))?", cmd)
                    if mm:
                        kind = mm.group(2)
                        params = [float(v) for v in (mm.group(3) or "").split("X") if v]
                        if kind in ("C", "R", "O"):
                            params = [v * unit for v in params]
                        elif kind == "P" and params:              # diameter has a unit; vertices, rotation don't
                            params[0] *= unit
                        apertures[mm.group(1)] = Aperture(mm.group(1), kind, params, dict(ta), unit)
                elif cmd.startswith("LP"):
                    dark = cmd[2] == "D"
                elif cmd.startswith("SR") and cmd != "SR":
                    gf.warnings.append(f"{name}: step-and-repeat (SR) is not supported")
                elif cmd.startswith(("AB", "LM", "LR", "LS")):
                    gf.warnings.append(f"{name}: {cmd[:2]} is not supported")
            continue
        word = m.group(2).strip()
        if not word:
            continue
        if word.startswith("G04") or word.startswith("G4 "):
            continue
        for gc in re.findall(r"G0?([0-9]+)", word):
            gc = int(gc)
            if gc in (1, 2, 3):
                interp = f"G0{gc}"
            elif gc == 36:
                in_region, contours, contour = True, [], []
                flush_draw()
            elif gc == 37:
                close_contour()
                for c in contours:
                    gf.objects.append(GObject("region", c, None, dict(to), dark))
                in_region, contours = False, []
        dm = re.search(r"D0?([0-9]+)$", word)
        if dm and int(dm.group(1)) >= 10:
            flush_draw()
            cur = apertures.get(str(int(dm.group(1))))
            continue
        coords = {k: v for k, v in _WORD.findall(word)}
        nx = num(coords["X"], 0) if "X" in coords else x
        ny = num(coords["Y"], 1) if "Y" in coords else y
        d = int(coords.get("D", "0") or 0)
        if d == 2:
            if in_region:
                close_contour()
                contour = [(nx, ny)]
            else:
                flush_draw()
                draw = [(nx, ny)]
        elif d == 1:
            if interp == "G01":
                seg = [(nx, ny)]
            else:
                ci, cj = num(coords.get("I", "0"), 0), num(coords.get("J", "0"), 1)
                seg = g.arc_center((x, y), (x + ci, y + cj), (nx, ny), interp == "G03")[1:]
            if in_region:
                if not contour:
                    contour = [(x, y)]
                contour += seg
            else:
                if not draw:
                    draw = [(x, y)]
                draw += seg
        elif d == 3:
            flush_draw()
            gf.objects.append(GObject("flash", [(nx, ny)], cur, dict(to), dark))
        x, y = nx, ny
    flush_draw()
    return gf


# ---------------------------------------------------------------- Excellon drill files
@dataclass
class Drill:
    x: float
    y: float
    diameter: float
    plated: bool | None               # None = the file does not say
    function: str = ""                # X2 comment: ComponentDrill, ViaDrill, MechanicalDrill...


def parse_excellon(text: str, name: str = "") -> tuple[list[Drill], dict]:
    unit = 1.0
    decimals = (3, 3)
    tools: dict[str, tuple[float, str]] = {}
    pending_fn = ""
    file_fn = ""
    cur: tuple[float, str] | None = None
    out: list[Drill] = []
    x = y = 0.0
    zeros = "TZ"                                  # no header: plain decimal numbers are the usual case
    plated: bool | None = None
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith(";"):
            m = re.search(r"#@!\s*TA\.AperFunction,([^\s]+)", line)
            if m:
                pending_fn = m.group(1)
            m = re.search(r"#@!\s*TF\.FileFunction,([^\s]+)", line)
            if m:
                file_fn = m.group(1)
                plated = False if file_fn.startswith("NonPlated") else (True if file_fn.startswith("Plated") else None)
            continue
        if line.startswith(("METRIC", "INCH")):
            unit = INCH if line.startswith("INCH") else 1.0
            decimals = (2, 4) if unit == INCH else (3, 3)
            zeros = "TZ" if "TZ" in line else "LZ"     # TZ: trailing zeros kept (read from the right)
            m = re.search(r"(0+)\.(0+)", line)
            if m:
                decimals = (len(m.group(1)), len(m.group(2)))
            continue
        m = re.match(r"T(\d+)(?:F\d+)?(?:S\d+)?C([\d.]+)", line)
        if m:
            tools[m.group(1)] = (float(m.group(2)) * unit, pending_fn)
            pending_fn = ""
            continue
        m = re.match(r"T(\d+)$", line)
        if m:
            cur = tools.get(str(int(m.group(1)))) or tools.get(m.group(1))
            continue
        if line.startswith(("X", "Y")) and cur is not None:
            parts = re.findall(r"([XY])([+-]?[\d.]+)", line.split("G85")[0])
            for k, v in parts:
                if "." in v:
                    val = float(v) * unit
                else:
                    s = v.lstrip("+-")
                    neg = v.startswith("-")
                    if zeros == "LZ":                  # leading zeros kept: the digits are read from the left
                        s = s.ljust(sum(decimals), "0")
                    val = int(s) / 10 ** decimals[1] * unit * (-1 if neg else 1)
                if k == "X":
                    x = val
                else:
                    y = val
            if "G85" in line:                                   # a slot: its middle
                end = dict(re.findall(r"([XY])([+-]?[\d.]+)", line.split("G85")[1]))
                ex = float(end.get("X", x)) * unit if "X" in end and "." in end["X"] else x
                ey = float(end.get("Y", y)) * unit if "Y" in end and "." in end["Y"] else y
                out.append(Drill((x + ex) / 2, (y + ey) / 2, cur[0], plated, cur[1]))
            else:
                out.append(Drill(x, y, cur[0], plated, cur[1]))
    return out, {"file_function": file_fn, "name": name}
