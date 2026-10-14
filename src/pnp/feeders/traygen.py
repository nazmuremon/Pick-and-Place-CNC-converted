"""3D printed SMD strip holders for the T-slots of the bed. Every holder slides into a bar's T-slot, holds one
cut-tape strip in its own channel (cover film peeled off by hand, no blade) and carries the slot's marker and number
sunk into its top (spray the top black, sand it flat: the code stays black in the recesses, the rest is white PLA).

The set (owner, 2026-10-12):
    holders 1-4  8 mm tape, each in two kinds (print the one the strip needs):
                 `8mm_paper`    0402-1206 resistors / capacitors (paper 1.0 thick; `--paper-t 0.7` for thin tape)
                 `8mm_embossed` SOT-23, SOD-123, SOD-323, small diodes / LEDs (plastic tape)
    holder 5     12 mm embossed tape: SOIC-8, SOP, SMA / SMB, SOT-89, SOT-223, crystals, inductors
    holder 6     16 mm embossed tape, the large parts: SOIC-14 / 16, optocouplers, bigger inductors
Embossed = plastic tape: the pockets hang into a groove up to 6 mm deep (any part height); `--pocket K0` raises the
groove floor to the strip's pocket depth so the pockets sit on it.

In every holder the top of the tape (and of the parts) is at the same height (`tape_top`), and the parts' centre
line is over the bar centre: one pick height for all, and the software finds the parts as on a paper label (marker
centre 7.5 mm from the back end, the 60 mm part zone 17..77 mm from it, holes on the side the top of the number
points to, D14).

    .venv/bin/python hardware/tray/smd_tray.py                       # the whole set
    .venv/bin/python hardware/tray/smd_tray.py --set 2:8:embossed 5:12 --pocket 2.5
    .venv/bin/python hardware/tray/smd_tray.py --set 12:8:paper --marker 0      # tray 12 with ArUco 0

The page's "Add tray" (Feeders tab, docs/13 §4.4.1) uses make_holder_stl() with the next free marker id. Numbers
1-99 (two digits are drawn narrower), tape 8 / 12 / 16 / 24 mm, any marker id of the bed's dictionary.

Units mm. Frame: x along the holder (0 = back end, marker end), y across (+y = sprocket-hole side), z up (0 = top
of the bar). The STL files are turned upside down (top face on the bed) ready to print.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import manifold3d as mf
import numpy as np
import trimesh

HERE = Path(__file__).resolve().parents[3] / "hardware" / "tray"
HOLE_TO_PART = {8: 3.5, 12: 5.5, 16: 7.5, 24: 11.5}     # EIA-481: sprocket hole line -> parts' centre line
DEFAULT_SET = {1: 8, 2: 8, 3: 8, 4: 8, 5: 12, 6: 16}     # holder number: tape width (owner, 2026-10-12)
KINDS = {8: ("paper", "embossed"), 12: ("embossed",), 16: ("embossed",), 24: ("embossed",)}   # kinds per width


@dataclass
class P:
    # ---- T-slot of the bar (measured by the owner 2026-10-12): A opening, B lip thickness, C inside width, D depth
    slot_a: float = 6.0
    slot_b: float = 3.2
    slot_c: float = 9.5
    slot_d: float = 4.0
    fit: float = -0.10         # clearance on each side of the rail (PLA, 0.4 nozzle). 0.25 printed a little loose
                               # (2026-10-12); the owner asked for 0.05, then another 0.3 mm bigger on every side:
                               # negative = the rail is drawn 0.1 mm BIGGER than the measured slot on every side
    # ---- the tape (EIA-481): holes 1.75 from the edge, the parts' centre line HOLE_TO_PART from the hole line
    tape_w: float = 8.0
    kind: str = "paper"        # "paper" (parts in punched paper, flat bottom) or "embossed" (pockets hang below)
    tape_t: float = 1.0        # paper tape thickness (0805: about 0.9-1.1; 0402 / 0603: 0.6-0.8)
    carrier_t: float = 0.3     # embossed tape: thickness of the flat carrier the pockets hang from
    pocket: float | None = None  # embossed: groove depth under the carrier (None = the deepest that fits). Set it to
                               # the strip's pocket depth and the pockets sit on the groove floor (held from below)
    tape_play: float = 0.3     # channel = tape width + this
    lip_gap: float = 0.15      # room between the top of the tape and the lips
    lip_t: float = 1.0         # lips over the tape edges hold the strip down
    lip_over: float = 0.8      # how far the lips reach over each tape edge (holes start 1.0 mm from the edge)
    ledge_far: float = 0.5     # embossed: the far tape edge rests on this much floor beside the groove
    groove_to_holes: float = 1.0  # embossed: the groove ends this far before the hole line (the holes rest on floor)
    # ---- the holder
    tape_top: float = 7.5      # top of the tape above the bar, the same in every holder (one pick height for all)
    base: float = 1.2          # thinnest floor (under the deepest groove)
    length_label: float = 87.0  # the label part (marker, number, 60 mm zone)
    entry: float = 13.0         # in front of it: the strip's end goes in here
    stop_x: float = 23.5        # the strip stops against this wall (the zone starts at 24.5)
    min_width: float = 20.0     # at least as wide as the slot label (marker + white border + number)
    wall: float = 2.4           # walls beside the tape channel
    # ---- marker and number (paint fill)
    marker: float = 10.0
    quiet: float = 2.5
    recess: float = 0.8         # depth of the black cells and the number
    digit_cell: float = 1.1     # 5 x 7 pixel digits

    @property
    def L(self) -> float:
        return self.length_label + self.entry

    @property
    def top(self) -> float:
        return self.tape_top + self.lip_gap + self.lip_t

    @property
    def width(self) -> float:
        he, fe = tape_edges(self.tape_w)
        return max(self.min_width, 2 * (max(he, fe) + self.tape_play / 2 + self.wall))


def tape_edges(w: float) -> tuple[float, float]:
    """(hole-side edge, |far edge|) of a w mm tape whose parts' centre line is on y = 0."""
    hole_line = HOLE_TO_PART[int(w)]
    return hole_line + 1.75, w - 1.75 - hole_line


DIGITS = {  # 5 x 7, top row first
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    "6": ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00010", "01100"],
}


def box(x0, y0, z0, x1, y1, z1) -> mf.Manifold:
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def ccw(poly):
    """CrossSection keeps only counter-clockwise outlines: turn a clockwise one round."""
    pts = [(float(a), float(b)) for a, b in poly]
    area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    return pts if area > 0 else pts[::-1]


def prism_xz(poly_xz, y0, y1) -> mf.Manifold:
    """A shape drawn in the x-z plane, extruded across y from y0 to y1."""
    m = mf.CrossSection([ccw(poly_xz)]).extrude(y1 - y0)                                   # points (x, z, w)
    return m.warp(lambda v: (v[0], y1 - v[2], v[1]))   # a rotation about x (a swap would mirror it inside out)


def under_of(p: P) -> float:
    """Where the tape lies (its bottom, or the embossed carrier's bottom)."""
    return p.tape_top - (p.tape_t if p.kind == "paper" else p.carrier_t)


def strip_channel(p: P, x0: float, x1: float, floor_z: float, top: float) -> mf.Manifold:
    """The solid to cut for one strip from x0 (its back stop) to past x1 (open front): the channel, the window between
    the lips, the groove for embossed pockets (down to floor_z at the deepest) and lead-ins at the front."""
    he, fe = tape_edges(p.tape_w)
    hole_line = HOLE_TO_PART[int(p.tape_w)]
    c_hi, c_lo = he + p.tape_play / 2, -fe - p.tape_play / 2
    under, chan_top = under_of(p), p.tape_top + p.lip_gap
    cut = box(x0, c_lo, under, x1 + 1, c_hi, chan_top)
    open_lo, open_hi = c_lo + p.tape_play / 2 + p.lip_over, c_hi - p.tape_play / 2 - p.lip_over
    cut += box(x0, open_lo, chan_top - 0.01, x1 + 1, open_hi, top + 1)
    if p.kind == "embossed":
        deepest = under - floor_z
        depth = deepest if p.pocket is None else p.pocket
        if depth > deepest + 1e-9:
            raise ValueError(f"pocket {depth} mm does not fit: at most {deepest:.1f} mm")
        cut += box(x0, -fe + p.ledge_far, under - depth, x1 + 1, hole_line - p.groove_to_holes, under + 0.01)
    cut += prism_xz([(x1 - 3, chan_top - 0.01), (x1 + 0.01, chan_top - 0.01), (x1 + 0.01, chan_top + 0.6)], c_lo, c_hi)
    cut += prism_xz([(x1 - 3, under + 0.01), (x1 + 0.01, under - 0.6), (x1 + 0.01, under + 0.01)], c_lo, c_hi)
    return cut


def marker_and_number(p: P, slot: int, marker_id: int, top: float, dictionary: str = "DICT_4X4_50") -> mf.Manifold:
    """The marker's black cells (ArUco, top of the marker towards +y) and the slot number next to it (top of the
    digits towards +y = the hole side, as on the paper label), as one solid to cut out of the top. The first 50
    markers of DICT_4X4_250 / _1000 are the same as DICT_4X4_50 (docs/13 §4.4.1)."""
    if not 1 <= slot <= 99:
        raise ValueError(f"tray numbers are 1-99, not {slot}")
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    if not 0 <= marker_id < d.bytesList.shape[0]:
        raise ValueError(f"marker {marker_id} is not in {dictionary}")
    bits = cv2.aruco.generateImageMarker(d, marker_id, 6, borderBits=1)
    s, cells = p.marker / 6, []
    for r in range(6):
        for c in range(6):
            if bits[r, c] < 128:
                x0, y1 = p.quiet + c * s, p.marker / 2 - r * s
                cells.append(box(x0 - 0.001, y1 - s - 0.001, top - p.recess, x0 + s + 0.001, y1 + 0.001, top + 1))
    text = str(slot)
    dc = min(p.digit_cell, 8.0 / (6 * len(text) - 1))         # two digits: narrower, in the same 8 mm
    x_start = p.quiet + p.marker + 2.0 + (8.0 - (6 * len(text) - 1) * dc) / 2
    for k, ch_ in enumerate(text):
        for r, row in enumerate(DIGITS[ch_]):
            for c, ch in enumerate(row):
                if ch == "1":
                    x0, y1 = x_start + (6 * k + c) * dc, 7 * dc / 2 - r * dc
                    cells.append(box(x0 - 0.001, y1 - dc - 0.001, top - p.recess, x0 + dc + 0.001, y1 + 0.001,
                                     top + 1))
    return mf.Manifold.batch_boolean(cells, mf.OpType.Add)


def holder(p: P, slot: int, marker_id: int, dictionary: str = "DICT_4X4_50") -> mf.Manifold:
    """A holder with its own channel for one tape (p.tape_w, p.kind): marker and number at the back, the strip from
    the stop wall to the open front end, the rail underneath."""
    L, top, half = p.L, p.top, p.width / 2
    body = box(0, -half, 0, L, half, top)
    body -= strip_channel(p, p.stop_x, L, p.base, top)
    body -= marker_and_number(p, slot, marker_id, top, dictionary)
    return body + rail(p, L)


def rail(p: P, L: float) -> mf.Manifold:
    """The T-rail under the tray, from x = 1 to L - 1: a neck through the slot opening and a head inside, `fit`
    smaller than the slot on every side (45 deg shoulders on the head: prints without support)."""
    neck = p.slot_a - 2 * p.fit
    head_w = p.slot_c - 2 * p.fit
    head_h = p.slot_d - 2 * p.fit
    z_neck = -(p.slot_b + p.fit)
    cham = (head_w - neck) / 2
    pts = [(-neck / 2, 0.01), (neck / 2, 0.01), (neck / 2, z_neck), (head_w / 2, z_neck - cham),
           (head_w / 2, z_neck - head_h), (-head_w / 2, z_neck - head_h), (-head_w / 2, z_neck - cham),
           (-neck / 2, z_neck)]
    return mf.CrossSection([ccw(pts)]).extrude(L - 2).warp(lambda v: (v[2] + 1, v[0], v[1]))


def to_stl(m: mf.Manifold, path: Path, upside_down: bool = True) -> None:
    if upside_down:                                            # rotate 180 deg about x: top face on the bed
        m = m.warp(lambda v: (v[0], -v[1], -v[2]))
        bb = m.bounding_box()
        m = m.translate([0, 0, -bb[2]])
    mesh = m.to_mesh()
    tm = trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3], faces=np.asarray(mesh.tri_verts))
    path.parent.mkdir(parents=True, exist_ok=True)
    tm.export(path)




def make_holder_stl(number: int, marker_id: int, tape_w: int, kind: str, out: Path,
                    dictionary: str = "DICT_4X4_50", **extra) -> Path:
    """One holder as a print-ready STL (the page's "Add tray")."""
    if int(tape_w) not in HOLE_TO_PART:
        raise ValueError(f"tape width {tape_w} mm: 8, 12, 16 or 24")
    if kind not in KINDS[int(tape_w)]:
        raise ValueError(f"{tape_w} mm tape comes as {' / '.join(KINDS[int(tape_w)])}, not {kind}")
    p = replace(P(), tape_w=float(tape_w), kind=kind, **extra)
    to_stl(holder(p, number, marker_id, dictionary), out)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", nargs="+", metavar="N:W[:KIND]",
                    help="holder number : tape width [: paper | embossed] (default: the whole set, DEFAULT_SET)")
    ap.add_argument("--pocket", type=float, help="embossed: groove depth = the strip's pocket depth")
    ap.add_argument("--paper-t", type=float, help="paper tape thickness (default 1.0)")
    ap.add_argument("--out", type=Path, default=HERE / "stl")
    ap.add_argument("--marker", type=int, help="the marker id (default: 40 + number - 1, the paper slot labels)")
    ap.add_argument("--dictionary", default="DICT_4X4_50")
    args = ap.parse_args()
    jobs = []
    for item in args.set or [f"{n}:{w}" for n, w in DEFAULT_SET.items()]:
        n, w, *kind = item.split(":")
        jobs += [(int(n), int(w), k) for k in (kind or KINDS[int(w)])]
    for slot, w, kind in jobs:
        extra = {"pocket": args.pocket} if kind == "embossed" and args.pocket is not None else \
            {"tape_t": args.paper_t} if kind == "paper" and args.paper_t is not None else {}
        p = replace(P(), tape_w=w, kind=kind, **extra)
        mid = args.marker if args.marker is not None else 40 + slot - 1   # the paper labels: ArUco 40 + n - 1
        tag = "".join(f"_{k.replace('tape_t', 'paper')}{v:g}" for k, v in extra.items())
        out = args.out / f"holder_{slot}_{w}mm_{kind}{tag}.stl"
        to_stl(holder(p, slot, mid, args.dictionary), out)
        print(f"{out.name}: holder {slot}, {w} mm {kind} tape, {p.width:g} wide, marker ArUco {mid}")


if __name__ == "__main__":
    main()
