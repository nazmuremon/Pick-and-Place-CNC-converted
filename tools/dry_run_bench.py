#!/usr/bin/env python3
"""Dry pick and place on the bench, with the live camera. No nozzle, no vacuum.

The camera stands in for the nozzle (the head has no nozzle yet). In the window you see it:
  1. calibrate on a label and measure HOME. No label in view (head moved by hand, PC restarted)?
     It drives to the BACK, where all labels sit, in short checked steps until one is in view,
  2. find the board's 4 mounting holes and work out where the board lies (board registration, D9),
  3. find the real parts in the slot's tape mark (empty pockets are skipped). Then, for every part
     of one value on the side that faces up: centre the camera on the next part in the slot ("PICK"),
     go to the part's place on the board ("PLACE") and draw its pads there. At every pick and place
     the head goes down 2 mm and back up to Z0, like the nozzle will (config/bench.yaml dry_run).
     When the slot is empty, stop placing, like a real job (refill the slot),
  4. come back near HOME and check the drift.

    .venv/bin/python tools/dry_run_bench.py
    .venv/bin/python tools/dry_run_bench.py --value 100nF --slot 3 --side B --top-edge front

--side / --top-edge: which board side faces up, and where the board's KiCad top edge (the LCD /
HT5023 edge, small KiCad Y) is: at the front or at the back of the machine.
Keys: space = stop after the current move, q = quit. Report + pictures: logs/dry_run/<time>/
"""
from __future__ import annotations

import argparse
import datetime
import itertools
import json
import re
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

import bed_test_run as t
import camera_rectify as cr
import make_labels as ml
from bench import Grbl, GrblError, load_bench, mark_parked, was_parked
from smd_identify import DEFAULT_CAMERA

ROOT = Path(__file__).resolve().parents[1]
BOARD_JS = ROOT.parent / "solder_helper" / "board_data.js"
LOG_DIR = ROOT / "logs" / "dry_run"
WINDOW = "Dry pick and place"
HOLES = ("H1", "H2", "H3", "H4")
HOLE_D_MM = 3.2
DWELL_S = 0.8
SPEED_LEVELS = [(800, 25), (1200, 40), (1600, 60), (2000, 80)]   # mm/min, mm/s2: tried in this order
SPEED_TRIPS = 2
MAX_DRIFT_MM = 0.15          # HOME may move this much after the trips, or that speed lost steps


# ---------------------------------------------------------------- board data

def load_board() -> dict:
    txt = BOARD_JS.read_text()
    return json.loads(txt[txt.index("{"): txt.rindex("}") + 1])


def farads(value: str) -> float | None:
    """'100nF', '0.1uF', '100n', '0.1 µF' -> 1e-7. None if it is not a capacitance."""
    m = re.fullmatch(r"([\d.]+)\s*([pnuµm]?)f?", value.strip().lower().replace("µ", "u"))
    if not m:
        return None
    return float(m.group(1)) * {"p": 1e-12, "n": 1e-9, "u": 1e-6, "m": 1e-3, "": 1.0}[m.group(2)]


def parts_to_place(board: dict, side: str, value: str) -> list[dict]:
    """The SMD parts of one capacitance on one side, front to back. '100nF' also finds '0.1uF'."""
    want = farads(value)
    if want is None:
        raise ValueError(f"--value {value!r}: not a capacitance (for example 100nF)")
    return sorted([p for p in board["parts"] if p["side"] == side and p["smd"] and not p["dnp"]
                   and farads(p["val"]) is not None and abs(farads(p["val"]) - want) < want * 1e-3],
                  key=lambda p: (p["y"], p["x"]))


def orientation(side: str, top_edge: str) -> np.ndarray:
    """2 x 2 matrix: board mm (KiCad x right, y down) -> bed mm (+X right, +Y back), for a board lying
    with `side` up and its KiCad top edge (small y) at the `top_edge` ('front' / 'back') of the bed."""
    if side == "F":
        return np.array([[1.0, 0], [0, -1.0]]) if top_edge == "back" else np.array([[-1.0, 0], [0, 1.0]])
    return np.array([[-1.0, 0], [0, -1.0]]) if top_edge == "back" else np.array([[1.0, 0], [0, 1.0]])


def fit_affine(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """M (2 x 2) and c with M @ src + c ~ dst. Affine, so a small scale error of an axis is absorbed."""
    X = np.c_[src, np.ones(len(src))]
    sol, *_ = np.linalg.lstsq(X, dst, rcond=None)
    return sol[:2].T, sol[2]


def odd_hole_out(q: np.ndarray, P: np.ndarray, tol: float = 0.6) -> int | None:
    """The 4 measured holes P do not fit the pattern q well: is one of them a wrong spot (a pad next to
    a hole that was not seen)? Leave each one out; if the other 3 fit each other (rotation + shift +
    one scale, so 3 points can disagree) within tol mm, that one is the odd hole. None = no single one."""
    best = None
    for j in range(len(q)):
        keep = [i for i in range(len(q)) if i != j]
        X = np.vstack([np.c_[q[keep, 0], -q[keep, 1], np.ones(3), np.zeros(3)],
                       np.c_[q[keep, 1], q[keep, 0], np.zeros(3), np.ones(3)]])
        (a, b, tx, ty), *_ = np.linalg.lstsq(X, np.r_[P[keep, 0], P[keep, 1]], rcond=None)
        fit = q[keep] @ np.array([[a, -b], [b, a]]).T + (tx, ty)
        r = np.hypot(*(fit - P[keep]).T).max()
        if best is None or r < best[0]:
            best = (r, j)
    return best[1] if best[0] < tol else None


def describe(M: np.ndarray) -> dict:
    """Scale of each board axis, rotation and skew (deg) of an affine fit."""
    sx, sy = np.linalg.norm(M[:, 0]), np.linalg.norm(M[:, 1])
    angle = np.degrees(np.arctan2(M[1, 0], M[0, 0]))
    skew = np.degrees(np.arccos(np.clip(M[:, 0] @ M[:, 1] / (sx * sy), -1, 1))) - 90
    return {"scale_x": float(sx), "scale_y": float(sy), "angle_deg": float(angle), "skew_deg": float(skew)}


def best_hole_set(cands: list[np.ndarray], q: np.ndarray, plan_c: np.ndarray, max_shift: float = 45.0,
                  max_res: float = 1.0):
    """One candidate per hole so that the 4 points look like the board's hole pattern q (4 x 2, mm):
    affine fit with scale within 4 %, angle < 6 deg, skew < 2 deg, within max_shift of the plan, and
    every hole within max_res mm of the fit (a set with a pad in it fits worse).
    Returns (indexes, residuals mm, M, c) of the best set, or None."""
    best = None
    for idx in itertools.product(*[range(len(c)) for c in cands]):
        P = np.array([cands[i][k] for i, k in enumerate(idx)])
        if any(abs(np.hypot(*(P[i] - P[j])) - np.hypot(*(q[i] - q[j]))) > 2 + 0.04 * np.hypot(*(q[i] - q[j]))
               for i, j in ((0, 1), (1, 2), (2, 3), (3, 0))):
            continue
        M, c = fit_affine(q, P)
        f = describe(M)
        if not (0.96 < f["scale_x"] < 1.04 and 0.96 < f["scale_y"] < 1.04 and abs(f["angle_deg"]) < 6
                and abs(f["skew_deg"]) < 2 and np.hypot(*(c - plan_c)) < max_shift):
            continue
        res = np.hypot(*(q @ M.T + c - P).T)
        if res.max() > max_res:
            continue
        score = res.max() + 0.01 * np.hypot(*(c - plan_c))
        if best is None or score < best[0]:
            best = (score, idx, res, M, c)
    return None if best is None else best[1:]


def holes_by_shift(cands: list[np.ndarray], guess: np.ndarray, q: np.ndarray, tol: float = 2.0) -> np.ndarray | None:
    """Fallback when a hole is missing from its candidates (run 2026-10-07: pads ranked above it).
    The board is rigid, so its holes all sit about the same shift away from where the plan puts them
    (guess, 4 x 2, machine mm). Every candidate votes for its shift; the shift that at least 3 holes
    agree on (within tol mm) wins, and a hole not found is predicted from the others. Returns the 4
    hole positions (machine mm), or None."""
    best = None
    for k, ck in enumerate(cands):
        for c in ck:
            s = c - guess[k]
            pick, spread = [], 0.0
            for j, cj in enumerate(cands):
                d = np.hypot(*(cj - guess[j] - s).T) if len(cj) else np.zeros(0)
                ok = len(d) and d.min() < tol
                pick.append(cj[int(np.argmin(d))] if ok else None)
                spread += float(d.min()) if ok else 0.0
            n = sum(p is not None for p in pick)
            if n >= 3 and (best is None or (n, -spread) > (best[0], -best[2])):
                best = (n, pick, spread)
    if best is None:
        return None
    ok = [j for j, p in enumerate(best[1]) if p is not None]
    M, c = fit_affine(q[ok], np.array([best[1][j] for j in ok]))
    f = describe(M)
    if not (0.96 < f["scale_x"] < 1.04 and 0.96 < f["scale_y"] < 1.04 and abs(f["angle_deg"]) < 6
            and abs(f["skew_deg"]) < 2):
        return None
    return np.array([best[1][j] if best[1][j] is not None else q[j] @ M.T + c for j in range(len(q))])


def hole_candidates(gray: np.ndarray, r_px: float, keep: int = 12, min_contrast: float | None = None) -> np.ndarray:
    """Round things the size of a 3.2 mm hole, best first: a hole is lighter inside (the bed shows
    through) than the board around it; a round solder pad is the opposite (bright ring, dark centre).
    min_contrast: keep only those that are lighter inside than around by more than this (grey levels)."""
    b = cv2.GaussianBlur(gray, (0, 0), 1.2)
    c = cv2.HoughCircles(b, cv2.HOUGH_GRADIENT, dp=1, minDist=12, param1=60, param2=9,
                         minRadius=max(3, int(r_px * 0.75)), maxRadius=int(r_px * 1.3) + 1)
    if c is None:
        return np.zeros((0, 2))
    h, w = b.shape
    yy, xx = np.ogrid[:h, :w]
    scored = []
    for x, y, _ in c[0]:
        d2 = (xx - x) ** 2 + (yy - y) ** 2
        inside = b[d2 <= (0.6 * r_px) ** 2]
        ring = b[(d2 >= (1.25 * r_px) ** 2) & (d2 <= (1.7 * r_px) ** 2)]
        if inside.size and ring.size:
            scored.append((x, y, float(inside.mean()) - float(ring.mean())))
    if min_contrast is not None:
        scored = [s for s in scored if s[2] > min_contrast]
    scored.sort(key=lambda v: -v[2])
    return np.array([(x, y) for x, y, _ in scored[:keep]]).reshape(-1, 2)


def find_slot_parts(gray: np.ndarray, J: np.ndarray, pos: np.ndarray, centre_px: np.ndarray, slot_xy: np.ndarray,
                    u_dir: np.ndarray, v_dir: np.ndarray, zone_uv: np.ndarray, cfg: dict,
                    margin_mm: float = 1.0) -> list[dict]:
    """The parts in a slot's tape mark, seen in one picture, sorted from the marker end.

    A part is a spot much darker than the tape around it (the black top of a chip resistor); tape
    holes and empty pockets are only a little darker (cfg, config/bench.yaml slot_parts). Only spots
    whose centre lies in the zone (zone_uv: corners in the slot frame, minus margin_mm) count.
    unclear = a reason not to pick it (never guess): across the zone edge, too big, too close to
    another dark spot. A spot cut by the picture edge is left out: another view must see it whole.
    Positions in machine mm: pixel px is at pos + J^-1 (centre_px - px)."""
    img = np.asarray(gray, np.float32)
    h, w = img.shape
    Jinv = np.linalg.inv(J)
    px_mm = float(np.sqrt(abs(np.linalg.det(J))))

    def to_mm(px):
        return pos + (centre_px - np.asarray(px, float)) @ Jinv.T

    def to_uv(xy):
        d = np.asarray(xy, float) - slot_xy
        return np.stack([d @ u_dir, d @ v_dir], axis=-1)

    k = max(3, int(cfg["background_mm"] * px_mm / 4)) | 1
    small = cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    bg = cv2.resize(cv2.medianBlur(np.clip(small, 0, 255).astype(np.uint8), k), (w, h),
                    interpolation=cv2.INTER_LINEAR).astype(np.float32)
    rel = (bg - img) / np.maximum(bg, 1.0)                 # how much darker than the tape, 0..1
    dark = (rel > cfg["dark"]).astype(np.uint8)
    n, lab, st, cen = cv2.connectedComponentsWithStats(dark)
    big = [i for i in range(1, n) if st[i, 4] / px_mm ** 2 >= cfg["min_area_mm2"]]

    # Dark spots closer than min_gap_mm grow into one when each grows by half of it.
    r = max(1, int(round(cfg["min_gap_mm"] * px_mm / 2)))
    grown = cv2.dilate(np.isin(lab, big).astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1,) * 2))
    _, glab = cv2.connectedComponents(grown)
    group = {}
    for i in big:
        ys, xs = np.nonzero(lab == i)
        group[i] = int(glab[ys[0], xs[0]])

    (u0, v0), (u1, v1) = zone_uv.min(axis=0), zone_uv.max(axis=0)
    parts = []
    for i in big:
        u, v = to_uv(to_mm(cen[i]))
        if not (u0 + margin_mm <= u <= u1 - margin_mm and v0 + margin_mm <= v <= v1 - margin_mm):
            continue
        x, y, bw, bh = st[i, :4]
        if x == 0 or y == 0 or x + bw >= w or y + bh >= h:
            continue
        ys, xs = np.nonzero(lab == i)
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(np.c_[xs, ys].astype(np.float32))
        corners = to_mm(cv2.boxPoints(((cx, cy), (rw + 1, rh + 1), ang)))   # +1: pixel centres -> edges
        s1, s2 = corners[1] - corners[0], corners[2] - corners[1]
        length, width = sorted((float(np.hypot(*s1)), float(np.hypot(*s2))), reverse=True)
        # Centre and angle from the second moments in machine mm: steadier than the outline (a 1206 top is
        # only ~16 x 10 px; the outline gave up to 4 deg). Inside the spot every pixel counts fully (the white
        # print on a resistor must not pull), the edge pixels by how dark they are (sub-pixel edges).
        blob = (lab == i).astype(np.uint8)
        outline, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        filled = cv2.drawContours(np.zeros_like(blob), outline, -1, 1, cv2.FILLED)
        inside = cv2.erode(filled, np.ones((3, 3), np.uint8)) > 0
        ref = float(np.median(rel[inside])) if inside.any() else float(cfg["dark"])
        if ref > float(cfg.get("max_dark", 1.0)):
            continue                                  # far darker than any part: an empty pocket over black tape
        yy, xx = np.nonzero(cv2.dilate(filled, np.ones((3, 3), np.uint8)))
        wts = np.where(inside[yy, xx], 1.0, np.clip(rel[yy, xx] / max(ref, 1e-3), 0.0, 1.0))
        pts = to_mm(np.c_[xx, yy])
        centre_mm = (wts[:, None] * pts).sum(axis=0) / wts.sum()
        dev = pts - centre_mm
        long_dir = np.linalg.eigh((dev * wts[:, None]).T @ dev / wts.sum())[1][:, 1]
        angle = float(np.degrees(np.arctan2(long_dir @ v_dir, long_dir @ u_dir)))
        angle = (angle + 90) % 180 - 90
        if length < float(cfg.get("min_length_mm", 0.0)):
            continue                                  # a tape sprocket hole (dark over a dark bed), not a part
        if width > 0 and length / width < float(cfg.get("min_aspect", 0.0)):
            continue                                  # round: a sprocket hole, not a part (0805: 2.0 x 1.25)
        puv = to_uv(to_mm(np.c_[xs, ys]))
        unclear = None
        if length > cfg["max_length_mm"]:
            unclear = "too big for one part"
        elif np.any((puv[:, 0] < u0) | (puv[:, 0] > u1) | (puv[:, 1] < v0) | (puv[:, 1] > v1)):
            unclear = "across the edge of the tape mark"
        elif sum(g == group[i] for g in group.values()) > 1:
            unclear = "touches another dark spot"
        parts.append({"xy": centre_mm, "uv": to_uv(centre_mm), "size_mm": (length, width),
                      "angle_deg": angle, "area_mm2": float(st[i, 4] / px_mm ** 2), "corners_xy": corners,
                      "off_centre_px": float(np.hypot(*(cen[i] - centre_px))), "unclear": unclear})
    return sorted(parts, key=lambda p: p["uv"][0])


def dark_spots(gray: np.ndarray, J: np.ndarray, pos: np.ndarray, centre_px: np.ndarray, cfg: dict,
               max_mm2: float = 20.0) -> list[tuple[np.ndarray, float, np.ndarray]]:
    """Compact spots much darker than what is around them: a part's black top, an ink mark. Same darkness
    test as the slot parts (config/bench.yaml slot_parts). Spots cut by the picture edge are left out.
    Returns (machine xy of the centre, area mm2, centre px) for each."""
    img = np.asarray(gray, np.float32)
    h, w = img.shape
    px_mm = float(np.sqrt(abs(np.linalg.det(J))))
    k = max(3, int(cfg["background_mm"] * px_mm / 4)) | 1
    small = cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    bg = cv2.resize(cv2.medianBlur(np.clip(small, 0, 255).astype(np.uint8), k), (w, h),
                    interpolation=cv2.INTER_LINEAR).astype(np.float32)
    dark = ((bg - img) / np.maximum(bg, 1.0) > cfg["dark"]).astype(np.uint8)
    n, _, st, cen = cv2.connectedComponentsWithStats(dark)
    Jinv = np.linalg.inv(J)
    out = []
    for i in range(1, n):
        area = st[i, 4] / px_mm ** 2
        x, y, bw, bh = st[i, :4]
        if not float(cfg["min_area_mm2"]) <= area <= max_mm2 or x == 0 or y == 0 or x + bw >= w or y + bh >= h:
            continue
        out.append((pos + Jinv @ (centre_px - cen[i]), float(area), cen[i].copy()))
    return out


def part_axis_deg(angle_deg: float, frame) -> float:
    """Direction (deg from machine +X, anticlockwise) of a part's long side, from its angle in the slot frame
    (find_slot_parts: from u towards v). A label's v is turned -90 deg from u (its corners run clockwise),
    so the angle can't just be added to u's direction."""
    _, u, v = frame
    a = np.radians(angle_deg)
    d = np.cos(a) * np.asarray(u, float) + np.sin(a) * np.asarray(v, float)
    return float(np.degrees(np.arctan2(d[1], d[0])))


# ---------------------------------------------------------------- the machine with the dry-run jobs

class DryRun(t.Machine):
    TITLE = "DRY RUN  (camera = nozzle, no vacuum)"
    KEYS_HELP = "space: stop    q: quit"
    def __init__(self, grbl, cam, markers, nominal, out_dir, board, args, bed) -> None:
        super().__init__(grbl, cam, markers, nominal, out_dir)
        self.board, self.args, self.bed = board, args, bed
        self.A = orientation(args.side, args.top_edge)
        parts = {p["ref"]: p for p in board["parts"]}
        self.hole_xy = np.array([[parts[h]["x"], parts[h]["y"]] for h in HOLES])
        self.todo = parts_to_place(board, args.side, args.value)
        self.parts_cfg = load_bench()["slot_parts"]
        self.dry_cfg = load_bench()["dry_run"]
        self.nozzle_mounted = bool(load_bench()["nozzle"].get("mounted", False))
        if not 0 <= float(self.dry_cfg["z_dip_mm"]) <= 4:
            raise ValueError("config/bench.yaml dry_run.z_dip_mm must be 0 ... 4 mm")
        self.z = 0.0                      # mm from the Z where the program started (Z0), down is negative
        self.M = self.c = None            # board registration: machine = M @ (A @ board) + c
        self.overlay: list[tuple] = []    # (polyline in machine mm, BGR colour, text)
        self.moving = False
        self.step_name = "starting"
        self.report: dict = {}
        self.done = False

    # ---- helpers
    def move(self, d) -> None:
        self.moving = True
        try:
            super().move(d)
        finally:
            self.moving = False

    def px_per_mm(self) -> float:
        return float(np.mean(np.linalg.norm(self.J, axis=0)))

    def to_px(self, P: np.ndarray) -> np.ndarray:
        return self.centre_px() - (self.J @ (np.asarray(P, float) - self.pos).T).T

    def board_to_machine(self, xy) -> np.ndarray:
        return (self.M @ (self.A @ np.asarray(xy, float).T)).T + self.c

    def nominal_T(self) -> np.ndarray:
        """Translation that puts the board's outline on the planned PCB area (mm from HOME)."""
        x0, y0, x1, y1 = self.board["bbox"]
        corners = (self.A @ np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]).T).T
        area = np.array(self.bed["layout"]["pcb_area_xy"], float)
        return self.home + area - corners.min(axis=0)

    def frame(self) -> np.ndarray:
        f = self.cam.frames_after(time.monotonic() + t.SETTLE_S, 1)[0]
        self.frame_shape = f.shape
        return f

    def stage(self, name: str) -> None:
        self.step_name = name
        self.say(name)

    # ---- jobs
    def job_dry_run(self) -> None:
        if not self.find_home():
            return
        if not self.register():
            return
        self.pick_and_place()
        self.stage("4/4 back near HOME, drift check")
        self.overlay = []
        self.move_to(self.clip_to_reach(self.home))
        drift = self.home_drift()
        self.report["home_drift_mm"] = None if drift is None else np.round(drift, 3).tolist()
        self.save_report()
        self.say(f"DRY RUN DONE: {len(self.report.get('placements', []))} of {len(self.todo)} placed. "
                 + ("" if drift is None else f"HOME drift X{drift[0]:+.2f} Y{drift[1]:+.2f} mm"))
        self.done = True

    def find_home(self) -> bool:
        self.stage("1/4 calibrate the camera and measure HOME")
        seen = self.visible() or self.search_for_labels()
        ref = self.calibration_label(seen) if seen else None
        if ref is None:
            self.say("No label in view: jog near HOME first (tools/bed_test_run.py), then start again")
            return False
        if not self.calibrate(ref):
            return False
        if ref != "home":
            c = self.look(ref)
            ref_pos = self.pos + np.linalg.solve(self.J, self.centre_px() - c.mean(axis=0))
            self.move_checked(ref_pos + self.nominal["home"] - self.nominal[ref] - np.array([15.0, 0]) - self.pos)
        self.job_home()
        return self.home is not None

    def marker_pos(self, name: str, frames: int = 6) -> np.ndarray | None:
        """Machine position of a marker centre, measured from where the head believes it is."""
        c = self.look(name, frames=frames)
        return None if c is None else self.pos + np.linalg.solve(self.J, self.centre_px() - c.mean(axis=0))

    def job_backlash(self) -> None:
        """Play in each axis: arrive at the same spot from the - side and from the + side (3 times each)
        and measure HOME with the camera. If HOME seems to move between the two, that is the backlash."""
        if not self.find_home():
            return
        base = self.clip_to_reach(self.home + np.array(t.SAFE_NEAR_HOME))
        self.move_to(base)
        out = {}
        for axis, e in (("X", np.array([1.0, 0.0])), ("Y", np.array([0.0, 1.0]))):
            self.stage(f"backlash {axis}: arriving 3 x from each side")
            from_minus, from_plus = [], []
            for _ in range(3):
                self.move_to(self.clip_to_reach(base - 6 * e))
                self.move_to(base)
                from_minus.append(self.marker_pos("home"))
                self.move_to(self.clip_to_reach(base + 6 * e))
                self.move_to(base)
                from_plus.append(self.marker_pos("home"))
            if any(v is None for v in from_minus + from_plus):
                self.say(f"   {axis}: HOME not seen every time")
                continue
            m, p_ = np.array(from_minus) @ e, np.array(from_plus) @ e
            gap = float(p_.mean() - m.mean())
            spread = float(max(np.ptp(m), np.ptp(p_)))
            out[axis] = {"backlash_mm": round(abs(gap), 3), "plus_minus_mm": round(gap, 3),
                         "repeat_spread_mm": round(spread, 3)}
            self.say(f"   {axis}: backlash {abs(gap):.2f} mm (same-side repeat spread {spread:.2f} mm)")
        self.report["backlash"] = out
        self.save_report()
        self.say("BACKLASH DONE: " + ", ".join(f"{a} {v['backlash_mm']:.2f} mm" for a, v in out.items()))
        self.done = True

    def job_speed_test(self) -> None:
        """Find the fastest speed that still comes back to HOME: long trips at each level, then the camera
        measures HOME. If HOME moved, that level lost steps: keep the last good one."""
        if not self.find_home():
            return
        base = self.clip_to_reach(self.home + np.array(t.SAFE_NEAR_HOME))
        trip = [self.clip_to_reach(self.home + np.array(p)) for p in ((-140, -15), (-140, -200), (-20, -200))]
        idx = {a: self.grbl.axes.steps_setting(a)[-1] for a in "XY"}          # '1' / '2' -> $11n, $12n
        start = (float(self.grbl.max_feed), 10.0)     # what is known to be safe: the bench cap, gentle accel
        self.move_to(base)
        good, results = None, []
        try:
            for feed, accel in SPEED_LEVELS:
                for i in idx.values():
                    self.grbl.send(f"$11{i}={feed}")
                    self.grbl.send(f"$12{i}={accel}")
                self.feed = float(feed)
                self.grbl.max_feed = float(feed)      # this test is what sets the cap: lift it for this level
                self.stage(f"speed test: {feed} mm/min, acceleration {accel} mm/s2, {SPEED_TRIPS} trips")
                for _ in range(SPEED_TRIPS):
                    for p in trip + [base]:
                        self.move_to(p)
                drift = self.home_drift()
                ok = drift is not None and np.hypot(*drift) <= MAX_DRIFT_MM
                results.append({"feed_mm_min": feed, "accel_mm_s2": accel, "ok": bool(ok),
                                "home_drift_mm": None if drift is None else np.round(drift, 3).tolist()})
                self.say(f"   {feed} mm/min, {accel} mm/s2: HOME moved "
                         + ("?" if drift is None else f"X{drift[0]:+.2f} Y{drift[1]:+.2f} mm")
                         + (" -> OK" if ok else " -> TOO FAST (lost steps)"))
                if not ok:
                    break
                good = (feed, accel)
        finally:                                      # always leave Grbl at a speed that passed
            # Nothing passed: even the slowest level lost steps. Halve it (the old cap may be faster than
            # what just failed): the mechanics need a look (loose pulley or coupler, driver current).
            slow = (SPEED_LEVELS[0][0] / 2, SPEED_LEVELS[0][1] / 2)
            feed, accel = good or (min(start, slow) if results else start)
            for i in idx.values():
                self.grbl.send(f"$11{i}={feed:g}")
                self.grbl.send(f"$12{i}={accel:g}")
            self.feed = self.grbl.max_feed = float(feed)
        set_bench_feed(feed, accel)
        self.report["speed_test"] = {"results": results, "chosen": {"feed_mm_min": feed, "accel_mm_s2": accel}}
        self.save_report()
        self.say(f"SPEED TEST DONE: using {feed:g} mm/min, {accel:g} mm/s2 (saved in Grbl and config/bench.yaml)"
                 + ("" if good else ". Even the slowest level lost steps: halved. Check the mechanics (pulleys, "
                    "couplers, driver current), then run the test again. Re-home before moving on."))
        self.done = True

    def register(self) -> bool:
        self.stage("2/4 find the 4 mounting holes")
        r_px = HOLE_D_MM / 2 * self.px_per_mm()
        q = (self.A @ self.hole_xy.T).T                     # hole pattern, bed axes, board origin
        guess = q + (self.nominal_T() - self.home)          # mm from HOME, if the board sat on the plan
        cands = []
        for name, g in zip(HOLES, guess):
            self.say(f"   {name}: looking around X{g[0]:+.0f} Y{g[1]:+.0f}")
            self.move_to(self.clip_to_reach(self.home + g))
            f = self.frame()
            c_px = hole_candidates(f, r_px)
            pts = self.pos + np.linalg.solve(self.J, (self.centre_px() - c_px).T).T if len(c_px) else np.zeros((0, 2))
            cands.append(pts)
            self.overlay = [(self._circle(p, HOLE_D_MM / 2), (0, 160, 255), "") for p in pts]
            cv2.imwrite(str(self.out_dir / f"holes_{name}.png"), f)
        found = best_hole_set(cands, q, self.nominal_T())
        if found is not None:
            P = np.array([cands[i][k] for i, k in enumerate(found[0])])
            self.say(f"   holes found, first fit: worst hole {found[1].max():.2f} mm off the pattern")
        else:
            P = holes_by_shift(cands, self.home + guess, q)
            if P is None:
                self.say("Could not find the board's holes. Is the board inside the PCB area, "
                         f"{self.args.side} side up, top edge at the {self.args.top_edge}?")
                return False
            shift = P.mean(axis=0) - (self.home + guess).mean(axis=0)
            self.say(f"   3 or 4 holes found by their common shift (X{shift[0]:+.1f} Y{shift[1]:+.1f} mm from the "
                     "plan); a missing one is predicted")
        self.stage("2/4 measure each hole again near the picture centre")
        refined = []
        for name, p in zip(HOLES, P):
            self.move_to(self.clip_to_reach(p))
            pts = []
            for f in self.cam.frames_after(time.monotonic() + t.SETTLE_S, 3):
                c_px = hole_candidates(f, r_px)
                if len(c_px):
                    m = self.pos + np.linalg.solve(self.J, (self.centre_px() - c_px).T).T
                    near = m[np.argmin(np.hypot(*(m - p).T))]
                    if np.hypot(*(near - p)) < 3.0:
                        pts.append(near)
            refined.append(np.mean(pts, axis=0) if pts else p)
            self.overlay = [(self._circle(refined[-1], HOLE_D_MM / 2), (0, 255, 0), name)]
            time.sleep(0.4)
        P = np.array(refined)
        self.M, self.c = fit_affine(q, P)
        res = np.hypot(*(q @ self.M.T + self.c - P).T)
        j = odd_hole_out(q, P) if res.max() > 0.5 else None
        if j is not None:
            keep = [i for i in range(len(q)) if i != j]
            M3, c3 = fit_affine(q[keep], P[keep])
            P[j] = q[j] @ M3.T + c3
            self.say(f"   {HOLES[j]} does not fit the other 3 holes (a pad?): it is predicted from them")
            self.M, self.c = fit_affine(q, P)
            res = np.hypot(*(q @ self.M.T + self.c - P).T)
        f = describe(self.M)
        shift = self.c - self.nominal_T()
        self.report["registration"] = {
            "holes_mm_from_home": {h: np.round(p - self.home, 3).tolist() for h, p in zip(HOLES, P)},
            "residuals_mm": np.round(res, 3).tolist(),
            "board_angle_deg": round(f["angle_deg"], 3),
            "board_shift_from_plan_mm": np.round(shift, 2).tolist(),
            "scale_x": round(f["scale_x"], 5), "scale_y": round(f["scale_y"], 5),
            "skew_deg": round(f["skew_deg"], 3),
            "note": "scale = machine mm per board mm. Not 1.000 = that axis's steps/mm is off (calibration step 1.5)",
        }
        self.say(f"   board registered: worst hole {res.max():.2f} mm, angle {f['angle_deg']:+.2f} deg, "
                 f"{shift[0]:+.1f} / {shift[1]:+.1f} mm from the plan, axis scale X {f['scale_x']:.4f} "
                 f"Y {f['scale_y']:.4f}")
        self.overlay = [(self._board_outline(), (0, 255, 0), "")]
        if res.max() > 0.5:
            self.say("   the holes do not fit well (> 0.5 mm): check the result pictures before trusting it")
        out = [p["ref"] for p in self.todo if not self.in_reach(self.board_to_machine([p["x"], p["y"]]) - self.home)]
        if out:
            self.say(f"   {len(out)} of the {len(self.todo)} places are outside the reach ({', '.join(out)}): "
                     "they will be skipped. Move the board towards the plan")
        return True

    def slot_frame(self, slot: str) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """The slot marker centre, and the label's u (marker -> tape mark) and v directions, machine mm."""
        centre = self.locate(slot)
        c = None if centre is None else self.look(slot, frames=6)
        if c is None:
            return None
        Jinv = np.linalg.inv(self.J)
        u, v = -(Jinv @ (c[1] - c[0])), -(Jinv @ (c[3] - c[0]))   # the picture moves against the head
        return centre, u / np.linalg.norm(u), v / np.linalg.norm(v)

    def parts_here(self, frame) -> list[dict]:
        centre, u_dir, v_dir = frame
        img = np.mean(self.cam.frames_after(time.monotonic() + t.SETTLE_S, 3), axis=0)
        self.frame_shape = img.shape
        return find_slot_parts(img, self.J, self.pos, self.centre_px(), centre, u_dir, v_dir,
                               ml.slot_zone_mm(self.bed), self.parts_cfg,
                               float(self.bed["feed_tray"]["search_margin_mm"]))

    def search_slot(self, slot: str, frame) -> list[dict]:
        """Look along the tape mark (a few camera stops) and list the parts in it, from the marker end.

        The lens bends the picture edges: a part far from the picture centre is measured up to ~1.5 mm
        off. So each stop keeps only the parts nearest to it (up to half way to the next stop, + 1 mm),
        and a part seen from two stops is merged, keeping the view nearer the picture centre."""
        centre, u_dir, v_dir = frame
        zone = ml.slot_zone_mm(self.bed)
        zone_xy = centre + zone[:, :1] * u_dir + zone[:, 1:] * v_dir
        stops = [float(u) for u in self.parts_cfg["views_u_mm"]]
        found: list[dict] = []
        for i, u in enumerate(stops):
            lo = -np.inf if i == 0 else (stops[i - 1] + u) / 2 - 1.0
            hi = np.inf if i == len(stops) - 1 else (u + stops[i + 1]) / 2 + 1.0
            self.move_to(self.clip_to_reach(centre + u * u_dir))
            for p in self.parts_here(frame):
                if not lo <= p["uv"][0] <= hi:
                    continue                           # another stop sees this one nearer its centre
                dist = [np.hypot(*(q["xy"] - p["xy"])) for q in found]
                k = int(np.argmin(dist)) if dist else -1
                if k < 0 or dist[k] >= self.parts_cfg["same_part_mm"]:
                    found.append(p)
                elif p["off_centre_px"] < found[k]["off_centre_px"]:
                    found[k] = p                       # the same part: keep the view nearer the picture centre
            self.overlay = [(_closed(zone_xy), (255, 200, 0), "")] + \
                           [(_closed(q["corners_xy"]), (0, 140, 255) if q["unclear"] else (0, 255, 0), "")
                            for q in found]
            time.sleep(DWELL_S * 0.6)
            self.snapshot_overlay(f"{slot}_view{i + 1}")
        found.sort(key=lambda q: q["uv"][0])
        return found

    def centre_on_part(self, part: dict, frame) -> dict | None:
        """Bring the camera centre onto the part: find it again, correct up to twice. The result is always
        measured after the last move (so its offset is the real centring error). None = not seen again."""
        self.move_to(self.clip_to_reach(part["xy"]))
        for attempt in range(3):
            near = [p for p in self.parts_here(frame) if not p["unclear"]
                    and np.hypot(*(p["xy"] - self.pos)) < self.parts_cfg["recheck_mm"]]
            if not near:
                return None
            got = min(near, key=lambda p: np.hypot(*(p["xy"] - self.pos)))
            if np.hypot(*(got["xy"] - self.pos)) < 0.05 or attempt == 2:
                return got
            self.move_to(self.clip_to_reach(got["xy"]))
        return None

    def dip(self, what: str) -> None:
        """Head down z_dip_mm and back up, like the nozzle at a pick or a place. Z always comes back to
        where it was, also when a move fails. No pictures down there: the camera is calibrated at Z0."""
        depth, feed = float(self.dry_cfg["z_dip_mm"]), float(self.dry_cfg["z_feed"])
        if getattr(self, "nozzle_mounted", False):
            if not getattr(self, "said_no_dips", False):
                self.say("   no Z dips: a nozzle is on the head (config/bench.yaml nozzle.mounted) and the dry run "
                         "does not know its heights")
                self.said_no_dips = True
            return
        if depth <= 0:
            return
        z0 = self.grbl.status()[1]["Z"]
        self.moving = True
        try:
            self.say(f"   {what}: Z down {depth:g} mm")
            self.grbl.move_axes({"Z": -depth}, feed)
            self.z = self.grbl.status()[1]["Z"]
            time.sleep(DWELL_S * 0.5)
        finally:
            self.grbl.move_axes({"Z": z0 - self.grbl.status()[1]["Z"]}, feed)
            self.z = self.grbl.status()[1]["Z"]
            self.moving = False
        self.say(f"   {what}: Z back up")

    def pick_and_place(self) -> None:
        if not self.prepare_slot():
            return
        while self.place_next():
            pass

    def prepare_slot(self, reset: bool = True) -> bool:
        """Find the slot and list its clear parts (they are picked in this order). False = slot not found.
        reset=False: the slot was refilled during a job: keep the places done so far and go on from there."""
        slot = f"slot_{self.args.slot}"
        self.stage(f"3/4 find {slot}")
        self.move_to(self.clip_to_reach(self.home + self.nominal[slot] - self.nominal["home"]))
        frame = self.slot_frame(slot)
        if frame is None:
            self.say(f"{slot} not found")
            return False
        self.stage(f"3/4 look for parts in {slot}")
        parts = self.search_slot(slot, frame)
        clear = [p for p in parts if not p["unclear"]]
        self.report["slot_parts"] = [
            {"u_mm": round(float(p["uv"][0]), 2), "v_mm": round(float(p["uv"][1]), 2),
             "dark_top_mm": [round(s, 2) for s in p["size_mm"]], "angle_to_slot_deg": round(p["angle_deg"], 1),
             "unclear": p["unclear"]} for p in parts]
        self.say(f"   {slot}: {len(clear)} parts found"
                 + "".join(f"; 1 unclear ({p['unclear']}): skipped" for p in parts if p["unclear"]))
        self.slot_view, self.n_clear = frame, len(clear)
        self.waiting = list(enumerate(clear, start=1))
        self.current_k = None                         # the place being worked on (for an interrupted job)
        if reset or getattr(self, "next_k", None) is None:
            self.next_k = 0
            self.report["picks"], self.report["placements"] = [], []
            self.report.pop("not_placed", None)
        else:                                         # refilled: those places are not "slot empty" any more
            self.report["not_placed"] = [x for x in self.report.get("not_placed", []) if "empty" not in x["why"]]
        return True

    def place_next(self) -> bool:
        """Pick the next part from the slot and place it on its pads. False = nothing more to do (every place
        done, or the slot is empty)."""
        slot, frame = f"slot_{self.args.slot}", self.slot_view
        while self.next_k < len(self.todo):
            k, part = self.next_k, self.todo[self.next_k]
            self.next_k += 1
            rel = self.board_to_machine([part["x"], part["y"]]) - self.home
            if not self.in_reach(rel):
                self.say(f"   {part['ref']} at X{rel[0]:+.1f} Y{rel[1]:+.1f} is outside the reach: a real job "
                         "refuses it (skipped)")
                self.report.setdefault("not_placed", []).append({"ref": part["ref"], "why": "outside the reach"})
                continue
            got = None
            while self.waiting and got is None:
                n, p = self.waiting.pop(0)
                self.stage(f"3/4 [{k + 1}/{len(self.todo)}] PICK part {n} of {self.n_clear} from {slot}")
                self.overlay = [(_closed(p["corners_xy"]), (0, 255, 0), f"PART {n}")]
                got = self.centre_on_part(p, frame)
                if got is None:
                    self.say(f"   part {n} not seen again under the camera: unclear, skipped")
            if got is None:
                left = self.todo[k:]
                self.say(f"{slot} is empty: {len(left)} places left. A real job stops here: refill {slot}, "
                         "then go on.")
                self.report.setdefault("not_placed", []).extend({"ref": q["ref"], "why": f"{slot} empty"} for q in left)
                self.next_k, self.slot_view = k, None     # after a refill the slot is searched again
                return False
            off = got["xy"] - self.pos
            self.overlay = [(_closed(got["corners_xy"]), (0, 255, 0),
                             f"PICK {n}: {got['size_mm'][0]:.1f} x {got['size_mm'][1]:.1f} mm")]
            self.say(f"   part {n}: black top {got['size_mm'][0]:.1f} x {got['size_mm'][1]:.1f} mm, "
                     f"camera centre {np.hypot(*off):.2f} mm from it")
            time.sleep(DWELL_S * 0.6)
            self.snapshot_overlay(f"pick_{part['ref']}")
            self.current_k = k
            picked = self.at_pick(got, n, part)
            if picked == "place":                     # the nozzle can't reach this place: next place, same part
                self.report.setdefault("not_placed", []).append({"ref": part["ref"],
                                                                 "why": "the nozzle cannot reach the place"})
                self.waiting.insert(0, (n, p))
                self.current_k = None
                continue
            if picked == "part":                      # the nozzle can't reach this part: same place, next part
                self.next_k, self.current_k = k, None
                continue
            if picked is not True:                    # missed: the place waits for the next part
                self.next_k, self.current_k = k, None
                return True
            self.report["picks"].append({"part": n, "ref": part["ref"], "u_mm": round(float(got["uv"][0]), 2),
                                         "machine_xy_from_home": np.round(got["xy"] - self.home, 3).tolist(),
                                         "dark_top_mm": [round(s, 2) for s in got["size_mm"]],
                                         "centre_error_mm": round(float(np.hypot(*off)), 3)})
            target = self.board_to_machine([part["x"], part["y"]])
            self.stage(f"3/4 [{k + 1}/{len(self.todo)}] PLACE {part['ref']} {part['val']} {part['fp']}")
            self.overlay = [(self._board_outline(), (0, 160, 0), "")] + \
                           [(self.board_to_machine(np.array(pad["p"][0]).reshape(-1, 2)), (0, 255, 255), "")
                            for pad in part["pads"]] + \
                           [(self._cross(target, 1.5), (0, 0, 255), part["ref"])]
            self.move_to(self.clip_to_reach(target))
            time.sleep(DWELL_S)
            self.snapshot_overlay(f"place_{part['ref']}")
            done = self.at_place(target, part, got)
            self.current_k = None
            if not (isinstance(done, dict) and done.get("recorded")):     # (the real place records at the release)
                self.report["placements"].append({"ref": part["ref"], "value": part["val"], "footprint": part["fp"],
                                                  "board_xy": [part["x"], part["y"]], "kicad_rot": part["rot"],
                                                  "machine_xy_from_home": np.round(target - self.home, 3).tolist(),
                                                  **(done if isinstance(done, dict) else {})})
            return True
        return False

    def at_pick(self, got: dict, n: int, part: dict) -> bool | str:
        """The camera is centred on the part to pick. True = picked; "part" / "place" = the nozzle can't reach
        the part / its place; anything else = the pick failed. Dry run: the head dips like the nozzle will."""
        self.dip(f"pick {n}")
        return True

    def at_place(self, target: np.ndarray, part: dict, got: dict) -> bool | dict:
        """The camera is centred on the place. Dry run: the head dips like the nozzle will."""
        self.dip(f"place {part['ref']}")
        return True

    # ---- drawing
    def _circle(self, c, r, n=24):
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)
        return np.asarray(c) + r * np.c_[np.cos(a), np.sin(a)]

    def _cross(self, c, r):
        c = np.asarray(c, float)
        return np.array([c + (-r, 0), c + (r, 0), c, c + (0, -r), c + (0, r), c])

    def _board_outline(self):
        x0, y0, x1, y1 = self.board["bbox"]
        return self.board_to_machine([[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]])

    def draw(self, frame: np.ndarray, found: dict) -> np.ndarray:
        view = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        scale = 1.5
        view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
        for name, c in found.items():
            cv2.polylines(view, [(c * scale).astype(np.int32)], True, (255, 180, 0), 2)
            t.label_on_picture(view, name.upper().replace("_", " "), c.mean(axis=0) * scale + (0, -45),
                               (0, 255, 0) if name == "home" else (255, 220, 120))
        if self.J is not None and not self.moving:
            for poly, colour, text in self.overlay:
                px = self.to_px(poly) * scale
                cv2.polylines(view, [px.astype(np.int32)], len(poly) > 6, colour, 2, cv2.LINE_AA)
                if text:
                    t.label_on_picture(view, text, px.mean(axis=0) + (0, -30), colour)
        h, w = view.shape[:2]
        cv2.drawMarker(view, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 40, 1)
        with self.lock:
            msg = self.msg
        rel = self.pos - (self.home if self.home is not None else 0)
        try:
            extra = self.status_extra()
        except Exception as e:                        # a bad entry in a calibration file must not kill the window
            extra = f"(status line: {e!r})"
        return t.status_panel(view, [
            (self.TITLE, (0, 255, 255)),
            (f"Step: {self.step_name}", (0, 255, 255)),
            (f"Head X{rel[0]:+.1f}  Y{rel[1]:+.1f}  Z{0.0 if abs(self.z) < 0.05 else self.z:+.1f} mm "
             f"{'from HOME' if self.home is not None else 'from start'}   "
             f"{'MOVING' if self.moving else 'still'}", (255, 255, 255))]
            + ([(extra, (120, 255, 120))] if extra else [])
            + [(msg, (255, 255, 255)), (self.KEYS_HELP, (170, 170, 170))])

    def status_extra(self) -> str | None:
        """One more status line (the nozzle tool shows its heights here)."""
        return None

    def snapshot_overlay(self, name: str) -> None:
        f = self.frame()
        cv2.imwrite(str(self.out_dir / f"{name}.png"), self.draw(f, self.markers.find(f)))

    def save_report(self) -> None:
        self.report.update({"date": datetime.datetime.now().isoformat(timespec="seconds"),
                            "board_side_up": self.args.side, "kicad_top_edge_at": self.args.top_edge,
                            "value": self.args.value, "slot": self.args.slot,
                            "note": "machine mm from the HOME marker centre, +X right, +Y back"})
        (self.out_dir / "report.yaml").write_text(yaml.safe_dump(self.report, sort_keys=False))


def _closed(poly: np.ndarray) -> np.ndarray:
    return np.vstack([poly, poly[:1]])


def set_bench_feed(feed: float, accel: float) -> None:
    path = ROOT / "config" / "bench.yaml"
    text = path.read_text()
    text = re.sub(r"(?m)^  max_feed: .*$",
                  f"  max_feed: {feed:g}     # mm/min, from the camera speed test {datetime.date.today()} "
                  f"(Grbl $111/$112 = {feed:g}, $121/$122 = {accel:g} mm/s2)", text)
    path.write_text(text)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--value", default="100nF")
    ap.add_argument("--slot", type=int, default=3)
    ap.add_argument("--side", choices=["F", "B"], default="B", help="board side that faces up")
    ap.add_argument("--top-edge", choices=["front", "back"], default="front",
                    help="where the board's KiCad top edge (LCD / HT5023 edge) lies on the bed")
    ap.add_argument("--speed-test", action="store_true",
                    help="instead of the dry run: find the fastest speed that does not lose steps")
    ap.add_argument("--backlash", action="store_true",
                    help="instead of the dry run: measure the play (backlash) of X and Y with the camera")
    args = ap.parse_args()

    bed = ml.load_bed()
    labels = ml.make_all(bed)
    nominal = {name: np.array(c, float) for name, c, _ in ml.label_placements(bed, labels)}
    out_dir = LOG_DIR / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    cam = cr.open_camera(DEFAULT_CAMERA)           # straightened if config/camera_rectify.yaml exists
    parked = was_parked()                         # before the port opens (that marks it "not parked")
    grbl = Grbl(cfg=load_bench())
    grbl.check_ready()
    machine = DryRun(grbl, cam, t.Markers(bed), nominal, out_dir, load_board(), args, bed)
    machine.feed = float(load_bench()["limits"]["max_feed"])
    print(f"Dry run: {len(machine.todo)} x {args.value} on side {args.side}, from slot {args.slot}. "
          f"Results: {out_dir}", flush=True)
    if not parked and machine.nozzle_mounted:
        machine.not_parked = ("the last program did not leave the nozzle up (it was stopped, or it moves Z): nothing "
                              "moves by itself. Start tools/nozzle_calibrate.py, lift the nozzle, press h, quit with q")
    machine.start()
    machine.submit(machine.job_speed_test if args.speed_test else
                   machine.job_backlash if args.backlash else machine.job_dry_run)

    cv2.namedWindow(WINDOW)
    last_ts, idle_since = 0.0, time.monotonic()
    try:
        while True:
            item = cam.latest()
            if item is not None and item[0] - last_ts > 0.08:
                last_ts, frame = item
                cv2.imshow(WINDOW, machine.draw(frame, machine.markers.find(frame)))
            key = cv2.waitKeyEx(15)
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                machine.stop_flag.set()
                machine.say("Stopping after the current move...")
            if machine.busy:
                idle_since = time.monotonic()
            elif time.monotonic() - idle_since > (15 if machine.done else 20):
                break                                     # finished (or stopped): close by itself
            try:
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    finally:
        machine.stop_flag.set()
        t0 = time.monotonic()
        while machine.busy and time.monotonic() - t0 < 30:
            time.sleep(0.05)
        grbl.close()
        if not machine.busy:                              # the dips always come back up (finally), none when mounted
            mark_parked(parked, "dry_run_bench")
        cam.close()
        cv2.destroyAllWindows()
        print("Bye.", flush=True)


if __name__ == "__main__":
    main()
