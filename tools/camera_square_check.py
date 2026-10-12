#!/usr/bin/env python3
"""Is the head camera looking straight down? A live check while you mount or re-mount it. NOTHING MOVES.

    .venv/bin/python tools/camera_square_check.py

Have a bed label (10 mm marker) near the middle of the picture (inside the circle). A camera that looks
straight down sees the marker square: 4 equal sides, 4 corners of 90 deg. A tilted camera sees a
trapezoid: the edge nearer to the lens looks longer. That edge is drawn RED, with an arrow to it.
Raise the camera on that side (the lens then turns towards it) until the window says STRAIGHT.
Then turn the lens ring for the highest focus number.

Why: the camera calibration (px per mm of head move) refuses a tilted camera. On 2026-10-07, after the
nozzle was mounted, the markers had sides 7-12 % apart and corners of 81-87 deg, and it was refused.

If config/camera_rectify.yaml exists (a tilted camera that can't be re-mounted), the window shows the
straightened picture: then it checks that correction. --raw shows the camera as it is.

Keys (window "Camera square check"): r = turn the picture 180 deg, q / Esc = quit.
Settings: config/bench.yaml camera_square.
"""
from __future__ import annotations

import argparse
import time
from collections import deque

import cv2
import numpy as np

import bed_test_run as t
import camera_rectify as cr
import make_labels as ml
from bench import load_bench, sharpness
from smd_identify import DEFAULT_CAMERA

WINDOW = "Camera square check"
SCALE = 1.5          # picture size on the screen
PRINT_S = 3.0        # a status line on the terminal this often


def marker_shape(corners: np.ndarray) -> dict:
    """The look of one marker (4 corners in the picture, in order around it).
    sides[i] = corner i to corner i+1 (px); spread_pct = longest minus shortest side, % of the mean side;
    angles = the 4 inner corner angles (deg, they add up to 360); corner_err = the worst one minus 90;
    tilt = arrow from the marker centre towards the edge that looks too long (the one nearest to the lens),
    in % of the side; near_edge = the side the arrow points at."""
    c = np.asarray(corners, float).reshape(4, 2)
    nxt, prv = np.roll(c, -1, axis=0), np.roll(c, 1, axis=0)
    sides = np.linalg.norm(nxt - c, axis=1)
    a, b = prv - c, nxt - c
    cos = np.sum(a * b, axis=1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1))
    angles = np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))
    centre = c.mean(axis=0)
    mids = (c + nxt) / 2 - centre
    unit = mids / np.linalg.norm(mids, axis=1)[:, None]
    excess = sides / sides.mean() - 1                 # + = longer than the mean side
    tilt = (excess[:, None] * unit).sum(axis=0) * 100
    return {"sides": sides, "spread_pct": float(np.ptp(sides) / sides.mean() * 100), "angles": angles,
            "corner_err": float(np.abs(angles - 90).max()), "tilt": tilt, "near_edge": int(np.argmax(unit @ tilt)),
            "centre": centre}


def straight(shape: dict, cfg: dict) -> bool:
    return (shape["spread_pct"] <= float(cfg["max_side_spread_pct"])
            and shape["corner_err"] <= float(cfg["max_corner_error_deg"]))


def nearest_to_centre(found: dict, centre: np.ndarray) -> str | None:
    return min(found, key=lambda n: np.hypot(*(found[n].mean(axis=0) - centre))) if found else None


def focus_of(gray: np.ndarray, corners: np.ndarray) -> float:
    """Sharpness around the marker (the same marker gives comparable numbers: higher = sharper)."""
    x0, y0 = np.floor(corners.min(axis=0)).astype(int)
    x1, y1 = np.ceil(corners.max(axis=0)).astype(int)
    m = max(x1 - x0, y1 - y0) // 4
    h, w = gray.shape
    roi = gray[max(0, y0 - m):min(h, y1 + m), max(0, x0 - m):min(w, x1 + m)]
    return sharpness(roi) if roi.size else 0.0


def report(name: str | None, shape: dict | None, cfg: dict, size_mm: float, off_px: float, focus: float,
           best_focus: float) -> list[tuple[str, tuple]]:
    """The lines of the status panel: (text, BGR colour)."""
    keys = ("r: turn the picture 180 deg    q: quit    (nothing moves in this tool)", (170, 170, 170))
    if shape is None:
        return [("No label in view: put a bed label under the camera, near the middle", (0, 220, 255)), keys]
    lo, hi = shape["angles"].min(), shape["angles"].max()
    numbers = f"sides {shape['spread_pct']:.1f} % apart, corners {lo:.1f} to {hi:.1f} deg"
    lines = []
    if off_px > float(cfg["centre_radius_px"]):
        lines.append((f"{name.upper().replace('_', ' ')} is {off_px:.0f} px from the middle: the lens bends the picture "
                      "there. Bring a label inside the circle (tools/bed_test_run.py jogs).", (0, 220, 255)))
    if straight(shape, cfg):
        lines.append((f"STRAIGHT: {numbers}", (0, 255, 0)))
    else:
        lines.append((f"TILTED: {numbers} (want {float(cfg['max_side_spread_pct']):g} % and "
                      f"90 +- {float(cfg['max_corner_error_deg']):g} deg)", (60, 60, 255)))
        lines.append(("RED edge = nearest to the lens. Raise the camera on that side (the lens turns "
                      "towards it) and watch the numbers fall.", (255, 255, 255)))
    lines.append((f"Focus {focus:.0f} (best {best_focus:.0f} on this label): turn the lens ring for the highest. "
                  f"Scale {shape['sides'].mean() / size_mm:.2f} px/mm", (255, 255, 0)))
    return lines + [keys]


def draw(gray: np.ndarray, shape: dict | None, ok: bool, cfg: dict, rot: bool, lines: list) -> np.ndarray:
    h, w = gray.shape
    view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if rot:
        view = cv2.rotate(view, cv2.ROTATE_180)
    view = cv2.resize(view, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_LINEAR)

    def p(pt) -> tuple[int, int]:
        u, v = pt
        if rot:
            u, v = w - 1 - u, h - 1 - v
        return int(round(u * SCALE)), int(round(v * SCALE))

    centre = ((w - 1) / 2, (h - 1) / 2)
    cv2.drawMarker(view, p(centre), (0, 0, 255), cv2.MARKER_CROSS, 40, 1)
    cv2.circle(view, p(centre), int(float(cfg["centre_radius_px"]) * SCALE), (0, 0, 255), 1, cv2.LINE_AA)
    if shape is not None:
        c = shape["corners"]
        for i in range(4):
            red = not ok and i == shape["near_edge"]
            cv2.line(view, p(c[i]), p(c[(i + 1) % 4]), (0, 0, 255) if red else (0, 220, 0), 4 if red else 2,
                     cv2.LINE_AA)
            out = c[i] + (c[i] - shape["centre"]) * 0.45          # angle written just outside its corner
            t.label_on_picture(view, f"{shape['angles'][i]:.1f}", p(out), (255, 255, 255))
        if not ok:
            tip = shape["centre"] + shape["tilt"] / max(np.hypot(*shape["tilt"]), 1e-9) * shape["sides"].mean() * 0.9
            cv2.arrowedLine(view, p(shape["centre"]), p(tip), (0, 0, 255), 3, cv2.LINE_AA, tipLength=0.25)
    return t.status_panel(view, lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default=DEFAULT_CAMERA)
    ap.add_argument("--raw", action="store_true", help="raw pictures (not straightened by config/camera_rectify.yaml)")
    args = ap.parse_args()
    cfg = load_bench()["camera_square"]
    markers = t.Markers(ml.load_bed())
    cam = cr.open_camera(args.camera, raw=args.raw)
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    hist: deque = deque(maxlen=int(cfg["average_frames"]))
    hist_name, best_focus, rot, last_print, last_t = None, 0.0, False, 0.0, 0.0
    print("Camera square check: nothing moves. q in the window quits.", flush=True)
    try:
        while True:
            got = cam.latest()
            if got is None or got[0] == last_t:
                if cv2.waitKeyEx(20) in (ord("q"), 27):
                    break
                continue
            last_t, gray = got
            centre = np.array([(gray.shape[1] - 1) / 2, (gray.shape[0] - 1) / 2])
            found = markers.find(gray)
            name = nearest_to_centre(found, centre)
            if name != hist_name:
                hist.clear()
                hist_name, best_focus = name, 0.0
            shape, ok, focus, off = None, False, 0.0, 0.0
            if name is not None:
                hist.append(found[name])
                corners = np.mean(hist, axis=0)
                shape = marker_shape(corners) | {"corners": corners}
                ok = straight(shape, cfg)
                focus = focus_of(gray, found[name])
                best_focus = max(best_focus, focus)
                off = float(np.hypot(*(shape["centre"] - centre)))
            lines = report(name, shape, cfg, markers.size_mm, off, focus, best_focus)
            cv2.imshow(WINDOW, draw(gray, shape, ok, cfg, rot, lines))
            if time.monotonic() - last_print > PRINT_S:
                last_print = time.monotonic()
                if shape is None:
                    print("no label in view", flush=True)
                else:
                    print(f"{name}: sides {shape['spread_pct']:.1f} % apart, corners "
                          f"{shape['angles'].min():.1f}-{shape['angles'].max():.1f} deg, "
                          f"{shape['sides'].mean() / markers.size_mm:.2f} px/mm, focus {focus:.0f}, "
                          f"{off:.0f} px from the middle -> {'STRAIGHT' if ok else 'TILTED'}", flush=True)
            key = cv2.waitKeyEx(1)
            if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("r"):
                rot = not rot
    finally:
        cam.close()
        cv2.destroyAllWindows()
    print("Bye.", flush=True)


if __name__ == "__main__":
    main()
