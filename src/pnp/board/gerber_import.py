"""Board from a set of Gerber X2 / X3 files + Excellon drill files (+ the .gbrjob): the universal path when
the native PCB file is not at hand (KiCad, Altium, EasyEDA Pro, Eagle 9+ can all export it).

- Component layers (X3, "Component,L1,Top" / "Component,Ln,Bot"): every part's centre (TO.C + the ComponentMain
  flash), rotation (TO.CRot), value, footprint, mount type, MPN, height, pin 1.
- Copper layers (X2 TO.P on pad flashes and regions): the pads of every part.
- Profile (outline), drill files (holes; the non-plated ones that are not in a part with copper are the
  mounting holes), the job file (board thickness).

X3 bottom rotation: KiCad 10 writes CRot = (its editor angle R) + 180° for a bottom part. Checked on all 139 bottom
parts of the HT5023 against its board file (2026-10-12). The model's bottom angle is 180° − R (D26), so
model = −CRot. For other generators the same rule is used and the preview asks the operator to confirm it.
"""
from __future__ import annotations

import io
import json
import re
import zipfile
from dataclasses import dataclass, field
from pathlib import Path

from . import geometry as g
from .gerber import Drill, GerberFile, parse, parse_excellon
from .model import Board, Component, Hole, Pad

GERBER_EXT = (".gbr", ".ger", ".gtl", ".gbl", ".gm1", ".gko", ".gtp", ".gbp", ".gts", ".gbs", ".gto", ".gbo",
              ".g1", ".g2", ".gp1", ".gm", ".art", ".pho")
DRILL_EXT = (".drl", ".xln", ".exc", ".ncd", ".txt", ".drd", ".tap")


@dataclass
class FileSet:
    gerbers: list[GerberFile] = field(default_factory=list)
    drills: list[tuple[str, list[Drill], dict]] = field(default_factory=list)
    job: dict | None = None
    names: list[str] = field(default_factory=list)
    skipped: list[str] = field(default_factory=list)


def _decode(v: str) -> str:
    return re.sub(r"\\u([0-9a-fA-F]{4})", lambda m: chr(int(m.group(1), 16)), v)


def read_files(files: list[tuple[str, bytes]]) -> FileSet:
    """Sort a list of (name, bytes) into Gerber, drill and job files. A .zip is opened."""
    fs = FileSet()
    flat: list[tuple[str, bytes]] = []
    for name, data in files:
        if name.lower().endswith(".zip") or data[:4] == b"PK\x03\x04":
            with zipfile.ZipFile(io.BytesIO(data)) as z:
                flat += [(n, z.read(n)) for n in z.namelist() if not n.endswith("/")]
        else:
            flat.append((name, data))
    for name, data in flat:
        base = Path(name).name
        text = data.decode("utf-8", errors="replace")
        low = base.lower()
        if low.endswith(".gbrjob"):
            try:
                fs.job = json.loads(text)
                fs.names.append(base)
            except json.JSONDecodeError:
                fs.skipped.append(f"{base}: not a valid job file")
            continue
        head = text[:2000]
        if head.lstrip().startswith("M48") or ("METRIC" in head and "M48" in head) or (
                low.endswith(DRILL_EXT) and re.search(r"^T\d+C", text, re.M)):
            drills, info = parse_excellon(text, base)
            fs.drills.append((base, drills, info))
            fs.names.append(base)
        elif "%FS" in head or "%TF" in head or low.endswith(GERBER_EXT):
            gf = parse(text, base)
            fs.gerbers.append(gf)
            fs.names.append(base)
        else:
            fs.skipped.append(f"{base}: not a Gerber, drill or job file")
    return fs


def _side(gf: GerberFile) -> str:
    f = gf.function
    return "B" if len(f) > 2 and f[2].startswith("Bot") else "F"


def _func(gf: GerberFile) -> str:
    return gf.function[0] if gf.function else _guess_function(gf.name)


def _guess_function(name: str) -> str:
    low = name.lower()
    if "edge" in low or low.endswith((".gm1", ".gko", ".gm")) or "profile" in low or "outline" in low:
        return "Profile"
    if low.endswith((".gtl", ".gbl")) or "_cu" in low:
        return "Copper"
    if "pos" in low and low.endswith(".gbr"):
        return "Component"
    return ""


def load_files(files: list[tuple[str, bytes]], name: str | None = None, require_components: bool = True) -> Board:
    """require_components=False: without X3 files the parts are made from the copper pads (position = the middle
    of the pads, angle unknown): good as the reference for a centroid file."""
    fs = read_files(files)
    profiles = [gf for gf in fs.gerbers if _func(gf) == "Profile"]
    comps = [gf for gf in fs.gerbers if _func(gf) == "Component"]
    coppers = [gf for gf in fs.gerbers if _func(gf) == "Copper" and (len(gf.function) < 3 or
                                                                     gf.function[2].startswith(("Top", "Bot")))]
    if not profiles:
        raise ValueError("No board outline: add the Edge_Cuts / Profile Gerber")
    if not comps and require_components:
        raise ValueError("No component (X3) files: export the position files as Gerber (KiCad: Fabrication outputs > "
                         "Component placement, format Gerber), or import a centroid file with these Gerbers as the "
                         "reference")
    b = Board(name=name or (fs.job or {}).get("GeneralSpecs", {}).get("ProjectId", {}).get("Name") or "board",
              source_tool="gerber_x3", source_files=fs.names)
    gen = (comps or fs.gerbers)[0].generator
    b.conventions = {"generator": gen, "bottom_rotation":
                     "X3 CRot on the bottom -> −CRot (KiCad writes R + 180°; the model is 180° − R, D26)"}
    if "kicad" not in gen.lower():
        b.notes.append(f"The X3 files come from {gen or 'an unknown tool'}: the bottom-side rotation rule is checked "
                       "only for KiCad. Check the bottom parts in the orientation preview")
    b.notes += [w for gf in fs.gerbers for w in gf.warnings] + fs.skipped

    # outline
    pieces = [o.points for gf in profiles for o in gf.objects if o.kind == "draw"]
    pieces += [o.points + [o.points[0]] for gf in profiles for o in gf.objects if o.kind == "region"]
    loops = g.chain(pieces)
    outer = max(loops, key=lambda lp: abs(g.area(lp)))
    x0, y0, _, _ = g.bbox(outer)
    b.origin_in_source = (x0, y0)
    B = lambda p: (round(p[0] - x0, 6), round(p[1] - y0, 6))       # noqa: E731 - the source (Y up) -> board
    b.set_outline([[B(p) for p in lp] for lp in loops])

    if fs.job:
        t = fs.job.get("GeneralSpecs", {}).get("BoardThickness")
        if t:
            b.thickness, b.thickness_from = float(t), "gbrjob GeneralSpecs.BoardThickness"

    # components and their outlines (courtyard: to give non-plated holes to their part)
    courtyards: dict[str, list[list[g.Point]]] = {}
    pins: dict[str, list[tuple[str, g.Point]]] = {}
    for gf in comps:
        side = _side(gf)
        for o in gf.objects:
            ref = _decode(o.attrs.get(".C", [""])[0])
            if o.kind == "flash" and o.function == "ComponentMain" and ref:
                a = {k: _decode(",".join(v)) for k, v in o.attrs.items()}
                crot = float(a.get(".CRot", "0") or 0)
                mnt = a.get(".CMnt", "")
                h = a.get(".CHgt")
                x, y = B(o.points[0])
                b.components.append(Component(
                    ref=ref, value=a.get(".CVal", ""), footprint=a.get(".CFtp", ""), side=side, x=x, y=y,
                    rotation=g.norm_deg(crot if side == "F" else -crot), mpn=a.get(".CMPN", ""),
                    manufacturer=a.get(".CMfr", ""), lcsc=_supplier(a.get(".CSup", "")),
                    height=float(h) if h else None, source_rotation=crot,
                    source_flags={"x3_mount": mnt, "x3_package": a.get(".CPgN", ""), "x3_library": a.get(".CLbN", "")},
                    smd_hint=(mnt == "SMD") if mnt in ("SMD", "TH", "Pressfit") else None))
            elif o.kind == "draw" and o.function == "ComponentOutline" and ref:
                courtyards.setdefault(ref, []).append([B(q) for q in o.points])
            elif o.kind == "flash" and o.function == "ComponentPin":
                p = o.attrs.get(".P", [])
                if len(p) >= 2:
                    pins.setdefault(_decode(p[0]), []).append((_decode(p[1]), B(o.points[0])))
    if not comps:
        b.source_tool = "gerber_x2"
        b.conventions["components_from"] = "copper pads (no X3 files): positions from the pads, angles unknown"
        for gf in coppers:
            for o in gf.objects:
                p = o.attrs.get(".P", [])
                ref = _decode(p[0]) if p else ""
                if ref and ref not in {c.ref for c in b.components}:
                    b.components.append(Component(ref=ref, value="", footprint="", side=_side(gf), x=0.0, y=0.0,
                                                  rotation=0.0, source_flags={"rotation_unknown": True}))
    comp = b.by_ref()

    # pads from the copper layers
    seen: set[tuple] = set()
    for gf in coppers:
        layer = _side(gf)
        for o in gf.objects:
            p = o.attrs.get(".P", [])
            if len(p) < 2 or o.kind not in ("flash", "region") or not o.dark:
                continue
            ref, pin = _decode(p[0]), _decode(p[1])
            c = comp.get(ref)
            if c is None:
                continue
            poly = [B(q) for q in o.polygon(gf.macros)]
            th = o.function in ("ComponentPad",) or (o.function == "" and layer != c.side)
            cx, cy = g.centroid(poly)
            key = (ref, pin, round(cx, 2), round(cy, 2))
            if key in seen:
                continue
            if th and layer != c.side and any(k[:2] == (ref, pin) and abs(k[2] - round(cx, 2)) < 0.02 and
                                              abs(k[3] - round(cy, 2)) < 0.02 for k in seen):
                continue
            seen.add(key)
            c.pads.append(Pad(number=pin, polygon=poly, through_hole=th, plated=True, net=_decode(
                o.attrs.get(".N", [""])[0]), layer="" if th else layer))
    # through-hole pads seen on both copper layers: keep the one on the part's side
    for c in b.components:
        c.pads = _dedupe(c.pads, c.side)
        if c.source_flags.get("rotation_unknown") and c.pads:
            x0_, y0_, x1_, y1_ = g.bbox([q for pd in c.pads for q in pd.polygon])
            c.x, c.y = round((x0_ + x1_) / 2, 4), round((y0_ + y1_) / 2, 4)
            if any(pd.through_hole for pd in c.pads):
                c.source_flags["side_unknown"] = True

    # drills: plated ones size the pads; non-plated ones are pegs of a part or mounting holes
    for _, drills, info in fs.drills:
        for d in drills:
            p = B((d.x, d.y))
            if d.plated is False or "NonPlated" in d.function or "NPTH" in d.function:
                owner = _owner(p, courtyards, comp)
                if owner is not None:
                    owner.pads.append(Pad(number="", polygon=g.circle(p[0], p[1], d.diameter / 2), through_hole=True,
                                          plated=False, drill=d.diameter))
                if owner is None or not any(pd.plated for pd in owner.pads):
                    b.holes.append(Hole(round(p[0], 4), round(p[1], 4), round(d.diameter, 4), False,
                                        owner.ref if owner else ""))
            elif "ViaDrill" not in d.function:
                for c in b.components:
                    for pd in c.pads:
                        if pd.through_hole and pd.drill is None and g.inside(p, pd.polygon):
                            pd.drill = round(d.diameter, 4)
    if not coppers:
        b.notes.append("No copper layers: the pads are unknown (SMD / through-hole comes from the X3 mount type). Add "
                       "the copper Gerbers for the pad check and the orientation preview")
        for ref, ps in pins.items():
            c = comp.get(ref)
            if c is not None and not c.pads:
                c.source_flags["pins"] = [[n, list(q)] for n, q in ps]
    return b


def _supplier(v: str) -> str:
    """TO.CSup: 'LCSC,C1234' pairs (supplier, part)."""
    parts = [x.strip() for x in v.split(",")]
    for i in range(0, len(parts) - 1, 2):
        if parts[i].upper() == "LCSC":
            return parts[i + 1]
    return ""


def _dedupe(pads: list[Pad], side: str) -> list[Pad]:
    out: list[Pad] = []
    for pd in pads:
        c = pd.centre
        dup = next((q for q in out if q.number == pd.number and abs(q.centre[0] - c[0]) < 0.02 and
                    abs(q.centre[1] - c[1]) < 0.02), None)
        if dup is None:
            out.append(pd)
    return out


def _owner(p: g.Point, courtyards: dict, comp: dict) -> Component | None:
    for ref, loops in courtyards.items():
        if any(len(lp) >= 3 and g.inside(p, lp) for lp in loops):
            c = comp.get(ref)
            if c is not None:
                return c
    return None


def load_dir(path: Path, name: str | None = None) -> Board:
    files = [(p.name, p.read_bytes()) for p in sorted(path.iterdir()) if p.is_file()]
    return load_files(files, name)
