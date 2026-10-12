#!/usr/bin/env python3
"""3D printed SMD tray for one T-slot of the bed: slides into the slot, holds one cut-tape strip, peels the cover
film off at a PLA blade while the strip is pushed in, and carries the slot's marker and number sunk into the top
(spray the top black, sand it flat: the code stays black in the recesses, the rest is white PLA).

The top copies the printed slot label (tools/make_labels.py, 87 x 20 mm): marker centre 7.5 mm from the back end,
slot number next to it, the 60 mm part zone 17..77 mm from the marker centre, parts on the centre line, sprocket
holes on the side the top of the number points to (D14). So the software finds the parts as on a paper label.

    .venv/bin/python hardware/tray/smd_tray.py            # STL for trays (slots) 1..6 into hardware/tray/stl/
    .venv/bin/python hardware/tray/smd_tray.py --slots 3  # one tray (slot 3)

Units mm. Tray frame: x along the tray (0 = back end, marker end), y across (+y = sprocket-hole side), z up
(0 = top of the bar). The STL files are turned upside down (top face on the bed) ready to print.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import manifold3d as mf
import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent


@dataclass
class P:
    # ---- T-slot of the bar (measured by the owner 2026-10-12): A opening, B lip thickness, C inside width, D depth
    slot_a: float = 6.0
    slot_b: float = 3.2
    slot_c: float = 9.5
    slot_d: float = 4.0
    fit: float = 0.25          # clearance on each side of the rail (PLA, 0.4 nozzle)
    # ---- the tape: 8 mm paper tape (owner's 100 nF strips). Parts 3.5 mm from the hole line, holes 1.75 from edge
    tape_w: float = 8.0
    tape_t: float = 1.0        # paper tape thickness (0805: about 0.9-1.1). Embossed tape: also set pocket_depth
    pocket_depth: float = 0.0  # embossed (plastic) tape: how far the pockets hang below the tape
    pocket_w: float = 2.2      # embossed: groove width under the pockets
    tape_play: float = 0.3     # channel = tape width + this
    # ---- the tray body
    length_label: float = 87.0  # the label part (marker, number, 60 mm zone)
    entry: float = 13.0         # in front of it: the peel blade and the opening where the strip goes in
    width: float = 20.0
    base: float = 1.6           # floor under the tape
    lip_t: float = 0.8          # lips over the tape edges hold the strip down
    lip_over: float = 0.8       # how far the lips reach over each tape edge (holes start 1.0 mm from the edge)
    # ---- marker and number (paint fill)
    marker: float = 10.0
    quiet: float = 2.5
    recess: float = 0.8         # depth of the black cells and the number
    digit_cell: float = 1.1     # 5 x 7 pixel digits
    # ---- peel blade
    blade_x: float = 89.0       # back edge of the blade (the peel edge), mm from the back end
    blade_len: float = 3.0
    blade_press: float = 0.05    # the blade's underside sits this much lower than the lips (holds the tape down)
    stop_x: float = 23.5        # the strip stops against this wall (the zone starts at 24.5)


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


def tray(p: P, slot: int, marker_id: int) -> mf.Manifold:
    L = p.length_label + p.entry
    half = p.width / 2
    floor_top = p.base + p.pocket_depth                       # where the tape lies
    chan_top = floor_top + p.tape_t + 0.25                    # tape + a little room
    top = chan_top + p.lip_t
    # tape across: parts on y = 0, holes at +3.5, hole-side edge at +5.25, other edge at -2.75 (8 mm tape)
    hole_edge, far_edge = p.tape_w / 2 + 1.25, -(p.tape_w / 2 - 1.25)
    c_hi, c_lo = hole_edge + p.tape_play / 2, far_edge - p.tape_play / 2

    body = box(0, -half, 0, L, half, top)
    # the channel (open from the stop wall to the front end), lips over the tape edges
    body -= box(p.stop_x, c_lo, floor_top, L + 1, c_hi, chan_top)
    open_lo, open_hi = c_lo + p.tape_play / 2 + p.lip_over, c_hi - p.tape_play / 2 - p.lip_over
    body -= box(p.stop_x, open_lo, chan_top - 0.01, p.blade_x, open_hi, top + 1)         # window over the zone
    body -= box(p.blade_x + p.blade_len, c_lo, chan_top - 0.01, L + 1, c_hi, top + 1)    # open entry
    if p.pocket_depth > 0:                                                               # embossed pockets
        body -= box(p.stop_x, -p.pocket_w / 2, p.base, L + 1, p.pocket_w / 2, floor_top + 0.01)
    # peel blade: a PLA bridge over the tape at the entry, flush with the top. Its back edge (towards the
    # parts) is the peel edge: the film turns up over it and runs back over the bridge to the entry. A plain 90 deg
    # edge (a bevel would hang in the air when printed upside down)
    body += box(p.blade_x, c_lo - 0.01, chan_top - p.blade_press, p.blade_x + p.blade_len, c_hi + 0.01, top)
    # a slope at the entry so the strip's end slides in easily
    body -= prism_xz([(L - 4, floor_top), (L + 0.01, floor_top - 0.6), (L + 0.01, floor_top)], c_lo, c_hi)

    # the marker: black cells sunk in (ArUco DICT_4X4_50, 6 x 6 cells with the black border)
    bits = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), marker_id, 6,
                                         borderBits=1)
    s = p.marker / 6
    mx0 = p.quiet                                              # marker left edge (x), top edge at y = +marker/2
    for r in range(6):
        for c in range(6):
            if bits[r, c] < 128:
                x0, y1 = mx0 + c * s, p.marker / 2 - r * s
                body -= box(x0 - 0.001, y1 - s - 0.001, top - p.recess, x0 + s + 0.001, y1 + 0.001, top + 1)
    # the slot number (top of the digit towards +y = the hole side, as on the paper label)
    d = DIGITS[str(slot)]
    dc = p.digit_cell
    dx0 = p.quiet + p.marker + 2.0 + (8.0 - 5 * dc) / 2
    for r, row in enumerate(d):
        for c, ch in enumerate(row):
            if ch == "1":
                x0, y1 = dx0 + c * dc, 7 * dc / 2 - r * dc
                body -= box(x0 - 0.001, y1 - dc - 0.001, top - p.recess, x0 + dc + 0.001, y1 + 0.001, top + 1)

    # the T-rail under the tray: neck through the slot opening, head inside (45 deg on top: prints without support)
    neck = p.slot_a - 2 * p.fit
    head_w = p.slot_c - 2 * p.fit
    head_h = p.slot_d - 2 * p.fit
    z_neck = -(p.slot_b + p.fit)
    cham = (head_w - neck) / 2
    rail = [(-neck / 2, 0.01), (neck / 2, 0.01), (neck / 2, z_neck), (head_w / 2, z_neck - cham),
            (head_w / 2, z_neck - head_h), (-head_w / 2, z_neck - head_h), (-head_w / 2, z_neck - cham),
            (-neck / 2, z_neck)]
    rail_m = mf.CrossSection([ccw(rail)]).extrude(L - 2).warp(lambda v: (v[2] + 1, v[0], v[1]))
    return body + rail_m


def to_stl(m: mf.Manifold, path: Path, upside_down: bool = True) -> None:
    if upside_down:                                            # rotate 180 deg about x: top face on the bed
        m = m.warp(lambda v: (v[0], -v[1], -v[2]))
        bb = m.bounding_box()
        m = m.translate([0, 0, -bb[2]])
    mesh = m.to_mesh()
    tm = trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3], faces=np.asarray(mesh.tri_verts))
    path.parent.mkdir(parents=True, exist_ok=True)
    tm.export(path)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--slots", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--out", type=Path, default=HERE / "stl")
    args = ap.parse_args()
    p = P()
    for slot in args.slots:
        m = tray(p, slot, 40 + slot - 1)                       # slot n = ArUco 40 + n - 1 (config/bed.yaml slot_ids)
        out = args.out / f"smd_tray_slot{slot}.stl"
        to_stl(m, out)
        print(f"{out.name}: {m.volume() / 1000:.1f} cm3, marker ArUco {40 + slot - 1}")


if __name__ == "__main__":
    main()
