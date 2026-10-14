#!/usr/bin/env python3
"""3D printed PCB corner pieces: they slide into the T-slots of the bed bars like the SMD trays (same rail), carry
the PCB 1-4 markers (ArUco 30-33) and number sunk into the top (spray black, sand the top), and mark the board:

    PCB 1 front-left, PCB 2 front-right: an arm whose end is the board's front corner (mark only).
    PCB 3 back-right, PCB 4 back-left:   an L that HOLDS the back corner: the arm is a wall along the back edge,
                                          the leg a wall along the side edge. Push the board back and left into
                                          them: PCB 4 sets X and Y, PCB 3 only Y (its leg has room, side_play).

Board and bars (owner's bed, survey 2026-10-08, board registration from the back corners 2026-10-11): the left
bar (PCB 1, PCB 4, slot 7) at X -147.9 and the right bar (PCB 2, PCB 3) at X +33.2 from HOME; the HT5023 board
(130 x 140) from X -118.6 to +11.2, back edge at Y -94.9. So the board's left edge is 29.3 mm right of the left
bar centre, its right edge 22.0 mm left of the right bar centre.

    .venv/bin/python hardware/tray/pcb_corner.py          # STL for PCB 1..4 and HOME into hardware/tray/stl/

HOME piece: the HOME marker (ArUco 49, config/bed.yaml markers.home_id) on the same rail, for the HOME bar. Its
marker centre is where the paper HOME label's was (layout origin), with an "H" in front of it.

Units mm. Piece frame: x across the bar (+x = right, 0 = bar centre), y along the bar (+y = back, 0 = the board
edge the piece marks: front edge for PCB 1 / 2, back edge for PCB 3 / 4), z up (0 = top of the bar).
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import manifold3d as mf

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import smd_tray as st  # noqa: E402  (the same rail, box, digits and STL export as the trays)


@dataclass
class C:
    left_arm: float = 29.3      # left bar centre -> board's left edge
    right_arm: float = 22.0     # right bar centre -> board's right edge
    side_play: float = 1.5      # PCB 3's leg stands this far off the right edge (the bar spacing is known to ~1 mm)
    edge: float = 8.5           # marker centre -> the board edge line (in front of it)
    top: float = 2.6            # top of every piece above the bar: 1 mm above the board top (board 1.6 thick)
    plate_w: float = 20.0       # on the bar, like the trays
    plate_len: float = 27.0     # along the bar, in front of the edge line
    front_arm_w: float = 6.0    # PCB 1 / 2: the arm in front of the front edge
    back_arm_w: float = 4.0     # PCB 3 / 4: the wall behind the back edge (2 mm left before the slot 7 tray)
    wrap: float = 10.0          # the back wall reaches this far along the board's back edge
    leg_w: float = 4.0          # the side wall
    leg_len: float = 15.0       # the side wall reaches this far along the board's side edge
    marker: float = 10.0
    recess: float = 0.8         # depth of the black cells and the number (paint fill)
    digit_cell: float = 1.1


H_GLYPH = ["10001", "10001", "10001", "11111", "10001", "10001", "10001"]   # 5 x 7, top row first
HOME_ID = 49


@dataclass
class Home:
    top: float = 2.0            # low, close to where the paper HOME label was
    plate_w: float = 20.0
    back: float = 8.5           # plate from the marker centre this far back ...
    front: float = 18.5         # ... and this far forward (room for the "H")
    marker: float = 10.0
    recess: float = 0.8
    letter_cell: float = 1.1


def home_piece(h: Home, p: st.P) -> mf.Manifold:
    """The HOME marker on a plate with the T-rail under it. Frame: marker centre at (0, 0), +y = back, z = 0 =
    top of the bar."""
    hw = h.plate_w / 2
    body = st.box(-hw, -h.front, 0, hw, h.back, h.top)
    bits = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), HOME_ID, 6,
                                         borderBits=1)
    s, cells = h.marker / 6, []
    for r in range(6):
        for col in range(6):
            if bits[r, col] < 128:
                x0, y1 = -h.marker / 2 + col * s, h.marker / 2 - r * s
                cells.append(st.box(x0 - 0.001, y1 - s - 0.001, h.top - h.recess, x0 + s + 0.001, y1 + 0.001,
                                    h.top + 1))
    dc, top_y = h.letter_cell, -h.marker / 2 - 2.5 - 0.5
    for r, row in enumerate(H_GLYPH):
        for col, ch in enumerate(row):
            if ch == "1":
                x0, y1 = -2.5 * dc + col * dc, top_y - r * dc
                cells.append(st.box(x0 - 0.001, y1 - dc - 0.001, h.top - h.recess, x0 + dc + 0.001, y1 + 0.001,
                                    h.top + 1))
    body -= mf.Manifold.batch_boolean(cells, mf.OpType.Add)
    length = h.front + h.back
    rail = st.rail(p, length + 2).warp(lambda v: (-v[1], v[0] - 1 - h.front, v[2]))     # along the bar
    return body + rail


PIECES = {1: ("left", "front"), 2: ("right", "front"), 3: ("right", "back"), 4: ("left", "back")}
MARKER_IDS = {1: 30, 2: 31, 3: 32, 4: 33}               # config/bed.yaml markers.pcb_corner_ids


def piece(c: C, p: st.P, n: int) -> mf.Manifold:
    side, end = PIECES[n]
    s = 1.0 if side == "left" else -1.0                  # the board lies towards +x from the left bar
    arm = c.left_arm if side == "left" else c.right_arm
    hw = c.plate_w / 2
    body = st.box(-hw, -c.plate_len, 0, hw, 0.01, c.top)                        # the plate on the bar
    if end == "front":                                   # the arm ends exactly at the board's front corner
        x0, x1 = sorted((0.0, s * arm))
        body += st.box(x0, -c.front_arm_w, 0, x1, 0.01, c.top)
        rail_y1 = 0.0
    else:                                                # the back wall and the side wall: an L around the corner
        x0, x1 = sorted((0.0, s * (arm + c.wrap)))
        body += st.box(x0, 0, 0, x1, c.back_arm_w, c.top)
        stand = arm - (c.side_play if side == "right" else 0.0)                 # bar centre -> leg's inner face
        lx0, lx1 = sorted((s * (stand - c.leg_w), s * stand))
        body += st.box(lx0, -c.leg_len, 0, lx1, 0.01, c.top)
        rail_y1 = c.back_arm_w
    body -= marker_and_number(c, n)
    rail = st.rail(p, rail_y1 + c.plate_len + 2)                               # along x from 1 to L - 1
    rail = rail.warp(lambda v: (-v[1], v[0] - 1 - c.plate_len, v[2]))          # turned along the bar
    return body + rail


def marker_and_number(c: C, n: int) -> mf.Manifold:
    """The black cells of the marker (top of the marker towards the back) and the number under it (readable from
    the front), as one solid to cut out of the top."""
    bits = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), MARKER_IDS[n],
                                         6, borderBits=1)
    cells = []
    s = c.marker / 6
    for r in range(6):
        for col in range(6):
            if bits[r, col] < 128:
                x0, y1 = -c.marker / 2 + col * s, -c.edge + c.marker / 2 - r * s
                cells.append(st.box(x0 - 0.001, y1 - s - 0.001, c.top - c.recess, x0 + s + 0.001, y1 + 0.001, c.top + 1))
    dc = c.digit_cell
    top_y = -c.edge - c.marker / 2 - 2.5 - 1.0           # under the marker's white border
    for r, row in enumerate(st.DIGITS[str(n)]):
        for col, ch in enumerate(row):
            if ch == "1":
                x0, y1 = -2.5 * dc + col * dc, top_y - r * dc
                cells.append(st.box(x0 - 0.001, y1 - dc - 0.001, c.top - c.recess, x0 + dc + 0.001, y1 + 0.001,
                                    c.top + 1))
    return mf.Manifold.batch_boolean(cells, mf.OpType.Add)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pieces", type=int, nargs="*", default=[1, 2, 3, 4])
    ap.add_argument("--no-home", action="store_true", help="do not make the HOME piece")
    ap.add_argument("--out", type=Path, default=HERE / "stl")
    args = ap.parse_args()
    c, p = C(), st.P()
    for n in args.pieces:
        m = piece(c, p, n)
        out = args.out / f"pcb_corner_{n}.stl"
        st.to_stl(m, out)
        side, end = PIECES[n]
        print(f"{out.name}: PCB {n} ({end}-{side}), marker ArUco {MARKER_IDS[n]}, {m.volume() / 1000:.1f} cm3")
    if not args.no_home:
        m = home_piece(Home(), p)
        out = args.out / "home.stl"
        st.to_stl(m, out)
        print(f"{out.name}: HOME, marker ArUco {HOME_ID}, {m.volume() / 1000:.1f} cm3")


if __name__ == "__main__":
    main()
