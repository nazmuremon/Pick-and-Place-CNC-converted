#!/usr/bin/env python3
"""Picture of the printed parts placed on the pick and place bed, seen from above (back at the top), in mm from the
HOME marker: the bar slots, HOME, the strip holders on their bars, the PCB corner pieces, the board, and how far the
camera centre and the nozzle can reach (config/bench.yaml limits.reach_from_home + nozzle_calibration offset_pick).

    .venv/bin/python hardware/tray/bed_layout.py       # -> hardware/tray/bed_layout.png

Positions: slot markers and bars from config/bed.yaml layout (bed survey), the board's back corners from the board
registration (2026-10-11), the corner pieces as designed in pcb_corner.py.
"""
from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
import pcb_corner as pc  # noqa: E402
import smd_tray as st  # noqa: E402

PX = 4.0                                            # pixels per mm
VIEW = (-185.0, 65.0, -275.0, 30.0)                 # x0, x1, y0, y1 in mm from HOME
BOARD = {"left": -118.6, "back": -94.9, "w": 130.0, "h": 140.0}
LEFT_BAR, RIGHT_BAR = -147.9, 33.2                  # the bars of the PCB corner pieces (pcb_corner.py)


def to_px(xy) -> np.ndarray:
    xy = np.asarray(xy, float)
    return np.stack([(xy[..., 0] - VIEW[0]) * PX, (VIEW[3] - xy[..., 1]) * PX], axis=-1)


def draw_mesh(img, m, place, colour) -> None:
    """Top view of a manifold: faces pointing up, low to high; the 0.8 mm paint recesses black."""
    mesh = m.to_mesh()
    v = np.asarray(mesh.vert_properties)[:, :3]
    f = np.asarray(mesh.tri_verts)
    tri = v[f]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    up = n[:, 2] / (np.linalg.norm(n, axis=1) + 1e-12) > 0.5
    top = v[:, 2].max()
    tri = tri[up]
    z = tri[:, :, 2].mean(1)
    for i in np.argsort(z):
        t = tri[i]
        if abs(z[i] - (top - 0.8)) < 0.02:
            c = (25, 25, 25)                        # painted marker cells / digits
        else:
            k = 0.55 + 0.45 * np.clip((z[i] - (top - 8)) / 8, 0, 1)
            c = tuple(int(ch * k) for ch in colour)
        pts = to_px(place(t[:, :2]))
        cv2.fillConvexPoly(img, np.round(pts * 4).astype(np.int32), c, cv2.LINE_AA, shift=2)


def label(img, text, xy, scale=0.5, colour=(30, 30, 30), bg=(255, 255, 255)) -> None:
    x, y = to_px(xy)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    cv2.rectangle(img, (int(x - tw / 2 - 3), int(y - th - 4)), (int(x + tw / 2 + 3), int(y + 4)), bg, -1)
    cv2.putText(img, text, (int(x - tw / 2), int(y)), cv2.FONT_HERSHEY_SIMPLEX, scale, colour, 1, cv2.LINE_AA)


def dashed_rect(img, x0, x1, y0, y1, colour, text, text_at) -> None:
    y0 = max(y0, VIEW[2])                            # the front part runs off the picture
    pts = to_px([[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]).astype(int)
    for a, b in zip(pts, pts[1:]):
        d = np.linalg.norm(b - a)
        for s in np.arange(0, d, 14):
            p = a + (b - a) * s / d
            q = a + (b - a) * min(s + 8, d) / d
            cv2.line(img, tuple(p.astype(int)), tuple(q.astype(int)), colour, 2, cv2.LINE_AA)
    label(img, text, text_at, 0.5, colour)


def main() -> None:
    bed = yaml.safe_load((ROOT / "config/bed.yaml").read_text())
    bench = yaml.safe_load((ROOT / "config/bench.yaml").read_text())
    cal = yaml.safe_load((ROOT / "config/nozzle_calibration.yaml").read_text())
    slots = {int(k): np.array(v, float) for k, v in bed["layout"]["slot_marker_xy"].items()}
    reach = bench["limits"]["reach_from_home"]
    off = np.array(cal["offset_pick"]["xy"], float)

    W, H = int((VIEW[1] - VIEW[0]) * PX), int((VIEW[3] - VIEW[2]) * PX)
    img = np.full((H + 90, W, 3), 238, np.uint8)
    img[:H] = (215, 215, 215)
    bars = sorted([s[0] for s in slots.values()] + [0.0, RIGHT_BAR])
    for bx in bars:                                  # the slot openings (A = 6 mm) along the bars
        a, b = to_px([[bx - 3, VIEW[3]], [bx + 3, VIEW[2]]])
        cv2.rectangle(img, tuple(a.astype(int)), tuple(b.astype(int)), (150, 150, 150), -1)

    # the board
    bl, bb = BOARD["left"], BOARD["back"]
    a, b = to_px([[bl, bb], [bl + BOARD["w"], bb - BOARD["h"]]])
    cv2.rectangle(img, tuple(a.astype(int)), tuple(b.astype(int)), (70, 120, 40), -1)
    label(img, "HT5023 board, B side up (130 x 140)", [bl + 65, bb - 70], 0.55, (255, 255, 255), (70, 120, 40))
    label(img, "KiCad top edge (front)", [bl + 65, bb - 132], 0.45, (255, 255, 255), (70, 120, 40))

    # the HOME piece (printed, marker centre = the layout origin)
    draw_mesh(img, pc.home_piece(pc.Home(), st.P()), lambda t: t[:, :2], (40, 170, 230))
    label(img, "HOME", [0, 12])

    # strip holders: right to left on the slot bars (marker at the back, sprocket holes to the right)
    order = [x for x in sorted(slots.values(), key=lambda s: -s[0])]
    plan = [(1, 8, "paper"), (2, 8, "paper"), (3, 8, "paper"), (4, 8, "paper"), (5, 12, "embossed")]
    colours = {8: (70, 140, 235), 12: (80, 180, 90), 16: (60, 110, 220)}
    for (n, w, kind), mxy in zip(plan, order):
        m = st.holder(replace(st.P(), tape_w=w, kind=kind), n, 39 + n)
        place = lambda t, mxy=mxy: np.stack([mxy[0] + t[:, 1], mxy[1] - (t[:, 0] - 7.5)], axis=-1)
        draw_mesh(img, m, place, colours[w])
        label(img, f"{n}: {w} mm", [mxy[0], mxy[1] - 84], 0.45)
        label(img, kind, [mxy[0], mxy[1] - 89], 0.4)

    # the PCB corner pieces
    c = pc.C()
    front = bb - BOARD["h"]
    for n, (side, end) in pc.PIECES.items():
        bx = LEFT_BAR if side == "left" else RIGHT_BAR
        ey = bb if end == "back" else front
        m = pc.piece(c, st.P(), n)
        draw_mesh(img, m, lambda t, bx=bx, ey=ey: np.stack([bx + t[:, 0], ey + t[:, 1]], axis=-1), (40, 170, 230))
        label(img, f"PCB {n}", [bx, ey - 31], 0.45)

    # reach: camera centre and nozzle (camera centre + offset)
    rx, ry = reach["x"], reach["y"]
    dashed_rect(img, rx[0], rx[1], ry[0], ry[1], (200, 90, 30), "camera centre reach", [rx[0] + 34, ry[1] - 4])
    dashed_rect(img, rx[0] + off[0], rx[1] + off[0], ry[0] + off[1], ry[1] + off[1], (40, 40, 220),
                f"nozzle reach (camera + {off[0]:.1f}, {off[1]:.1f})", [rx[0] + off[0] + 62, ry[1] + off[1] - 4])

    # legend
    y = H + 30
    cv2.putText(img, "Seen from above, BACK at the top, mm from HOME. Grey stripes: the bar slots. "
                     "Holder 6 (16 mm): no free bar left inside the reach.",
                (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1, cv2.LINE_AA)
    cv2.putText(img, "Orange: 8 mm holders, green: 12 mm, yellow: HOME + PCB corners, black: painted markers. "
                     "Dashed: camera / nozzle reach.",
                (14, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1, cv2.LINE_AA)
    out = HERE / "bed_layout.png"
    cv2.imwrite(str(out), img)
    print(out)


if __name__ == "__main__":
    main()
