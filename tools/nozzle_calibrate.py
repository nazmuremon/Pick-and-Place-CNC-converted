#!/usr/bin/env python3
"""Nozzle calibration on the bench (docs/08-calibration.md steps 7 and 8, decision D18), with the live camera.

It measures two things:
  1. OFFSET: where the nozzle tip is, seen from the camera (nozzle tip = camera centre + offset), at several
     servo positions. The nozzle presses marks into paper with a thin layer of soft putty or ink, and the camera
     finds each mark. If the needle is not exactly on the servo shaft, its tip goes round a small circle when the
     servo turns: the circle's centre (the shaft) and radius (the runout) are fitted too.
  2. SERVO TABLE: which pulse gives which nozzle angle. It picks a part from the slot (pump on), turns the
     servo, places the part on double-sided tape and measures the part's angle with the camera.
  3. Z, with the camera: the camera's scale (px per mm of a label) depends on its height, and 1 / scale grows
     in a straight line with it. A Z sweep over HOME gives that line: from then on a Z that slid while the
     power was off is measured (~0.05 mm) and the touch heights are corrected. The sweep also shows whether
     Z moves in a straight line, its play and its repeatability. With a block of known height (a spare label
     on top), it also checks the Z steps/mm.

    .venv/bin/python tools/nozzle_calibrate.py
    .venv/bin/python tools/nozzle_calibrate.py --no-servo   # servo not wired yet: 3 marks at one angle

The machine first finds HOME by itself. Then, in the window:
  arrows, 1-4    jog X / Y (1, 5, 10, 30 mm), only with the nozzle up
  h              find HOME again (after jogging until a label is in view)
  [  ]           jog Z down / up by 0.1 mm        {  }   by 1 mm
  p              the paper for the marks is under the camera: remember this spot
  l              the tape for placed parts is under the camera: remember this spot
  t              the nozzle tip just touches the paper: remember this Z
  k              the nozzle tip just touches a part in its slot: remember this Z (PICK height; the pick
                 goes press_mm lower)
  d              the place ("drop") height on the board: remember this Z. Best with a part held (a): jog
                 down until the part just touches the board
  m              the MOVING height: parts are carried at this Z (at least nozzle.move_clearance_mm above
                 the pick and place heights)
  a              pump (vacuum) on / off
  n  (twice)     nozzle offset by eye: 1st n = the camera centres on what is nearest the red cross: a label,
                 a part in the tape (PICK offset) or a board hole (PLACE offset); then jog the NOZZLE tip right
                 above its centre and press n again. With the tilted camera the offset depends on the height,
                 so take the pick offset on a part and the place offset on a board hole
  r              REAL pick and place, ONE part per press: the camera finds the next part in the slot, the
                 nozzle picks it (pump on), the camera checks the pocket is empty, the servo turns the part to
                 its pads, the nozzle places it (pump off) and the camera takes a picture of it (after_*.png).
                 The first press finds the board's holes and the parts in the slot
  c              servo to the centre
  o              run the OFFSET job        v   run the SERVO TABLE job (needs the offset and the pump)
  z              run the Z SWEEP (over HOME; nothing may be under the nozzle there)
  b              Z steps/mm check: a block of known height with a label on top lies next to a slot label,
                 both in view (needs the paper touch height: the nozzle goes one block height down)
  space          stop after the current move    q   quit (the nozzle goes up first)

  0  (twice)     make the camera height of now the remembered one (Z0): every height and offset is set again

Z0 is the REMEMBERED CAMERA HEIGHT (D19). Grbl's Z restarts at 0 in every session and Z slides when the power is off,
so h measures the pick slot's label at the picture centre and drives Z back to that height before anything else.
Every height (t, k, d, m) and offset (n) is kept in mm from Z0. After a session that did not end cleanly (Z not
back up) or a Grbl reset, nothing moves by itself: lift Z by hand (}), then press h. The Z sweep (z) and the block
check (b) only measure Z (straight line, play, steps/mm); they don't change the heights.
Settings: config/bench.yaml `nozzle`. Results: config/nozzle_calibration.yaml and logs/nozzle_cal/<time>/.
"""
from __future__ import annotations

import argparse
import datetime
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

import bed_test_run as t
import camera_rectify as cr
import dry_run_bench as d
import make_labels as ml
from bench import Grbl, GrblError, load_bench, mark_parked, was_parked
from pnp_paths import CONFIG_DIR, LOGS_DIR
from servo_pump_test import firmware_ok
from smd_identify import DEFAULT_CAMERA

ROOT = Path(__file__).resolve().parents[1]
CAL_FILE = CONFIG_DIR / "nozzle_calibration.yaml"
LOG_DIR = LOGS_DIR / "nozzle_cal"
WINDOW = "Nozzle calibration"
ARROWS = {65361: (-1, 0), 65363: (1, 0), 65362: (0, 1), 65364: (0, -1)}   # left, right, up = back, down = front
# Only to tell a part turned by a from one turned by a + 180 deg (its outline looks the same): the 270 deg
# servo over 500-2500 us. The real scale comes from the measurement.
NOMINAL_DEG_PER_US = 270.0 / 2000.0
HEIGHT_KEYS = {"paper": "t", "part": "k", "place": "d", "move": "m"}
HEIGHT_WHAT = {"paper": "touch height on the paper", "part": "pick height (touch on a part)",
               "place": "place height", "move": "moving height"}


# ---------------------------------------------------------------- calibration file and maths

def load_cal() -> dict:
    try:
        return yaml.safe_load(CAL_FILE.read_text()) or {}
    except OSError:
        return {}


def save_cal(cal: dict) -> None:
    CAL_FILE.write_text("# Written by tools/nozzle_calibrate.py. Positions in mm from HOME (+X right, +Y back);\n"
                        "# Z in mm from where the program started (down is negative). See docs/08-calibration.md.\n"
                        + yaml.safe_dump(cal, sort_keys=False))


def height_ok(now: float | None, saved: float | None, tol: float) -> bool:
    """Is the camera (and so the head) at the same height as when a touch height was set?"""
    return now is not None and saved is not None and abs(now / saved - 1) <= tol


def find_mark(gray: np.ndarray, J: np.ndarray, pos: np.ndarray, centre_px: np.ndarray, expect_xy: np.ndarray,
              radius_mm: float, od_mm: float, dark: float, max_area: float = 4.0) -> np.ndarray | None:
    """Centre (machine mm) of the nozzle mark nearest to expect_xy, within radius_mm. A mark is a spot (ink) or
    a ring (putty) darker than the paper, about the needle's size: 0.3 to 4 times its area. A ring is filled
    before its centre is taken. Pixel px is at pos + J^-1 (centre_px - px)."""
    img = np.asarray(gray, np.float32)
    h, w = img.shape
    px_mm = float(np.sqrt(abs(np.linalg.det(J))))
    k = max(3, int(8.0 * px_mm / 4)) | 1
    small = cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
    bg = cv2.resize(cv2.medianBlur(np.clip(small, 0, 255).astype(np.uint8), k), (w, h),
                    interpolation=cv2.INTER_LINEAR).astype(np.float32)
    mask = ((bg - img) / np.maximum(bg, 1.0) > dark).astype(np.uint8)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    nominal = np.pi * (od_mm / 2) ** 2 * px_mm ** 2
    best, best_d = None, radius_mm
    for c in contours:
        m = cv2.moments(c)
        if m["m00"] <= 0 or not 0.3 * nominal <= m["m00"] <= max_area * nominal:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        if x == 0 or y == 0 or x + bw >= w or y + bh >= h:
            continue
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        xy = pos + np.linalg.solve(J, centre_px - np.array([cx, cy]))
        dist = float(np.hypot(*(xy - expect_xy)))
        if dist < best_d:
            best, best_d = xy, dist
    return best


def fit_circle(points) -> tuple[np.ndarray, float]:
    """Least-squares circle through the points: centre and radius."""
    P = np.asarray(points, float)
    sol, *_ = np.linalg.lstsq(np.c_[2 * P, np.ones(len(P))], (P ** 2).sum(axis=1), rcond=None)
    centre = sol[:2]
    return centre, float(np.sqrt(max(0.0, sol[2] + centre @ centre)))


def offset_for(pulse: float, by_pulse: dict) -> np.ndarray:
    """Nozzle tip offset at a servo pulse: straight line between the measured pulses, the nearest outside."""
    ps = sorted(float(p) for p in by_pulse)
    xs = np.array([by_pulse[p] if p in by_pulse else by_pulse[int(p)] for p in ps], float)
    return np.array([np.interp(pulse, ps, xs[:, 0]), np.interp(pulse, ps, xs[:, 1])])


def unwrap_turns(pulses, raw_deg, centre_us: float, sign: float = 1.0) -> list[float]:
    """A part's outline looks the same turned by 180 deg: take the turn nearest the nominal servo scale.
    sign: the servo's direction is not known before the table is measured (-1 = longer pulse turns clockwise)."""
    out = []
    for p, a in zip(pulses, raw_deg):
        nominal = sign * (float(p) - centre_us) * NOMINAL_DEG_PER_US
        out.append(float(a + 180.0 * round((nominal - a) / 180.0)))
    return out


def fit_table(pulses, turns, centre_us: float) -> dict:
    """Straight line turn = deg_per_us * (pulse - centre) + offset_deg, and how far each point is from it."""
    x = np.asarray(pulses, float) - centre_us
    y = np.asarray(turns, float)
    b, a = np.polyfit(x, y, 1)
    res = y - (a + b * x)
    return {"deg_per_us": round(float(b), 5), "offset_deg": round(float(a), 3),
            "worst_deg": round(float(np.abs(res).max()), 3)}


def fit_z_sweep(zs, scales) -> dict:
    """Pinhole camera: 1 / scale = (h0 + z) / f, h = the camera height above the label plane. Fits that line.
    f_mm is in commanded Z mm per unit of 1 / scale (right even when the Z steps/mm is off), h0_mm the camera
    height at Z0, z_seen_mm the Z each scale says, worst_mm how far Z is from a straight line."""
    z = np.asarray(zs, float)
    inv = 1.0 / np.asarray(scales, float)
    b, a = np.polyfit(z, inv, 1)
    seen = inv / b - a / b
    return {"f_mm": float(1.0 / b), "slope": float(b), "h0_mm": float(a / b), "z_seen_mm": seen,
            "worst_mm": float(np.abs(seen - z).max())}


def z_drift(f_mm: float, scale_now: float, scale_then: float) -> float:
    """How far the head is higher (+) or lower (-) now than then, at the same commanded Z (commanded mm)."""
    return f_mm * (1.0 / scale_now - 1.0 / scale_then)


def steps_from_block(s_block: float, s_bed: float, s_bed_down: float, steps_old: float):
    """Z steps/mm from a block of known height H with a label on top, all at the picture centre:
    s_block = block label at Z0, s_bed = bed label at Z0, s_bed_down = bed label after a Z move of -H.
    If 1 commanded mm moved 1 real mm, the camera is then as high above the bed label as it was above the
    block: s_bed_down = s_block. Returns (real mm per commanded mm, the steps/mm that makes them equal)."""
    alpha = 1.0 - (1.0 / s_bed_down - 1.0 / s_block) / (1.0 / s_bed - 1.0 / s_block)
    return float(alpha), float(steps_old / alpha)


def grid(k: int, spacing: float) -> np.ndarray:
    """k-th spot on the paper or tape: 5 per row, rows towards the front."""
    return spacing * np.array([k % 5, -(k // 5)], float)


# ---------------------------------------------------------------- the machine with the calibration jobs

class NozzleCal(d.DryRun):
    TITLE = "NOZZLE CALIBRATION"
    KEYS_HELP = "[ ] { } Z  k pick  d place  m moving  a pump  n offset  r REAL 1 part  t/p/l  c o v z b  q"

    def __init__(self, grbl, cam, markers, nominal, out_dir, board, bed) -> None:
        cfg = load_bench()
        args = argparse.Namespace(side="B", top_edge="front", value="100nF", slot=int(cfg["nozzle"]["pick_slot"]))
        super().__init__(grbl, cam, markers, nominal, out_dir, board, args, bed)
        self.setup(cfg)

    def setup(self, cfg: dict) -> None:
        self.ncfg, self.scfg, self.pcfg, self.zcfg = cfg["nozzle"], cfg["nozzle_servo"], cfg["pump"], cfg["z_cal"]
        self.home_px_mm: float | None = None   # the HOME label's scale at Z0 in this session (camera height)
        self.nozzle_target: tuple[str, np.ndarray] | None = None   # label kept by the first press of n
        self.no_servo = False                  # --no-servo: the servo is not wired: never turn it
        self.max_down = float(cfg["zoom"]["max_down_mm"])
        self.cal = load_cal()
        self.pulse: float | None = None
        self.pump_on = False
        self.real: dict | None = None          # the real pick and place plan (key r), None = dry run hooks
        self.part_on_nozzle = False            # True after an interrupted pick/place with the pump on
        self.pending: dict = {}                # the turn, pulse and nozzle position for the place of the part held
        # Z0 = the remembered camera height. Grbl's Z restarts at 0 in every session and Z slides when the power
        # is off, so at each start the camera height is measured on the pick slot's label and Z is driven back.
        self.z_base = 0.0                      # Grbl's Z at Z0 in this session
        self.z_ok = False                      # the camera height was checked in this session (heights usable)
        self.z_locked: str | None = None       # why nothing may move by itself (unclean end, Grbl reset)
        self.restoring = False
        self.new_ref_armed = False             # first press of 0

    # ---- Z, servo, pump
    def z_now(self) -> float:
        return float(self.grbl.status()[1]["Z"]) - self.z_base

    def z_to(self, z: float, by_hand: bool = False) -> None:
        z = float(z)
        if getattr(self.grbl, "reset_seen", False) and not self.z_locked:
            self.z_locked = "Grbl was reset (power dip?): the position is lost"
            self.z_ok = False
        if self.z_locked and not (by_hand and z >= self.z_now()):
            raise GrblError(f"{self.z_locked}. Lift the nozzle by hand (}} or ]) until it is clear, then press h")
        if z < -self.max_down and not (by_hand and z >= self.z_now()):
            raise GrblError(f"Z {z:+.2f} is deeper than the safety cap zoom.max_down_mm ({self.max_down:g} mm): refused")
        dz = z - self.z_now()
        try:
            if abs(dz) > 1e-4:
                self.moving = True
                self.grbl.move_axes({"Z": dz}, float(self.dry_cfg["z_feed"]))
        finally:
            self.moving = False
            self.z = self.z_now()                 # also after a failed move: never a stale "up"

    def servo(self, pulse: float) -> None:
        if self.z_locked:
            raise GrblError(f"{self.z_locked}: the servo does not turn")
        if self.no_servo:
            self.pulse = float(pulse)
            return
        settle = float(self.scfg["settle_s"])
        self.grbl.send(f"M3 S{pulse:.0f}")
        self.grbl.send(f"G4 P{settle:g}", timeout=settle + 5)
        self.pulse = float(pulse)

    def pump(self, on: bool) -> None:
        self.grbl.send(self.pcfg["start" if on else "stop"])
        self.pump_on = on

    def nozzle_up(self, gap: float | None = None) -> bool:
        """Is the tip at least gap mm (default nozzle.move_clearance_mm) above every surface it knows (pick,
        place, paper heights)? Before any is known: at Z0 or above it."""
        gap = float(self.ncfg["move_clearance_mm"]) if gap is None else gap
        tops = [v for v in (self.touch_z(n, quiet=True) for n in ("paper", "part", "place")) if v is not None]
        if tops:
            return self.z >= max(tops) + gap
        return self.z >= -0.5

    def move(self, d) -> None:
        """Every X / Y move: refused while Z is not trusted, or with the nozzle low (the owner's own jogs need
        nozzle.jog_clearance_mm, every other move nozzle.move_clearance_mm). The camera-height restore moves
        only around the remembered camera height."""
        d = np.asarray(d, float)
        if getattr(self.grbl, "reset_seen", False) and not self.z_locked:
            self.z_locked = "Grbl was reset or lost its position"
            self.z_ok = False
        if np.hypot(*d) > 1e-9:
            if self.z_locked:
                raise GrblError(f"{self.z_locked}. Lift the nozzle by hand (}} or ]) until it is clear, then press h")
            hand = getattr(self, "by_hand", False)
            gap = float(self.ncfg["jog_clearance_mm" if hand else "move_clearance_mm"])
            if not self.restoring and not self.nozzle_up(gap):
                raise GrblError(f"X / Y refused: the nozzle is less than {gap:g} mm above the highest surface it knows. "
                                "Lift it first (] or })")
        super().move(d)

    # ---- checks
    def need(self, *keys: str) -> bool:
        missing = [k for k in keys if self.ncfg.get(k) is None]
        if missing:
            self.say(f"Fill in config/bench.yaml nozzle: {', '.join(missing)} (TBD), then start again")
        return not missing

    def spot(self, name: str, key: str) -> np.ndarray | None:
        xy = self.cal.get(f"{name}_xy_from_home")
        if xy is None:
            self.say(f"Put the {name} under the camera and press {key} first")
            return None
        return self.home + np.array(xy, float)

    def ref_id(self) -> str | None:
        return (self.cal.get("z_reference") or {}).get("id")

    def touch_z(self, name: str, quiet: bool = False) -> float | None:
        """A remembered height (mm from Z0). Only after the camera height was checked in this session, and only
        if it was set at the same remembered camera height."""
        say = (lambda text: None) if quiet else self.say
        tz = self.cal.get(f"touch_{name}")
        key, what = HEIGHT_KEYS[name], HEIGHT_WHAT[name]
        if tz is None:
            say(f"Set the {what} first ({key})")
            return None
        if not self.z_ok:
            say("The camera height is not checked yet in this session: press h")
            return None
        if tz.get("ref") is None or tz.get("ref") != self.ref_id():
            say(f"The {what} was set at another camera height (before 0 was pressed): set it again ({key})")
            return None
        return float(tz["z"])

    # ---- the remembered camera height (Z0)
    def centred_scale(self, label: str) -> float | None:
        """px per mm of a label brought to the picture centre: always the same picture spot, so the tilted
        camera's perspective does not change the reading."""
        if self.locate(label) is None:
            return None
        return self.marker_scale(label)

    def probe_f(self, label: str, s0: float) -> float | None:
        """The camera's factor F (1 / scale = height / F) from a 1 mm move and back: DOWN when h has just
        lifted (there is room below), else UP (Z may be near the top of its travel only after a lift)."""
        z0 = self.z_now()
        dz = -1.0 if getattr(self, "start_lift", 0.0) >= 1.5 else 1.0
        try:
            self.z_to(z0 + dz)
            s1 = self.centred_scale(label)
        finally:
            if dz < 0:
                self.z_to(z0 + 0.5)                   # back, arriving from above
            self.z_to(z0)
        if s1 is None or abs(s1 - s0) < 1e-9:
            return None
        f = dz / (1.0 / s1 - 1.0 / s0)
        return f if f > 0 else None

    def ref_tags(self) -> dict:
        """What the remembered heights depend on: if one changes, they are refused (never converted)."""
        try:
            steps = self.grbl.settings().get(self.grbl.axes.steps_setting("Z"))
        except Exception:
            steps = None
        rect = cr.RECTIFY_FILE
        rect_time = (yaml.safe_load(rect.read_text()) or {}).get("time") if rect.exists() else None
        return {"z_steps_per_mm": steps, "needle_od_mm": self.ncfg.get("od_mm"), "camera_rectify": rect_time}

    def new_reference(self, label: str, s: float) -> None:
        self.cal["z_reference"] = {"label": label, "px_mm": round(float(s), 5), "f_mm": None,
                                   "id": datetime.datetime.now().isoformat(timespec="seconds"), **self.ref_tags(),
                                   "note": "Z0 = the camera height where this label (at the picture centre) has "
                                           "px_mm; every height and offset is tagged with id"}
        save_cal(self.cal)
        self.z_base += self.z_now()
        self.z, self.z_ok = 0.0, True
        self.lifted_at = None
        self.say("This camera height is now the remembered one (Z0). Set the heights (k, d, m, t) and the offsets (n)")

    def restore_camera_height(self) -> bool:
        """Drive Z back to the remembered camera height, measured on the pick slot's label at the picture
        centre (HOME can't be centred: it is outside the reach). Then this height is Z0. Up by itself up to
        z_cal.max_drift_mm (Z slides down); down by itself only up to z_cal.restore_max_down_mm."""
        label = str(self.zcfg["label"])
        ref = self.cal.get("z_reference")
        self.restoring, self.camera_moved_z = True, False
        try:
            if not ref:
                self.say("No camera height is remembered yet. Jog Z (] }) until the nozzle is clear of the board, the "
                         "holder and the strips, then press 0 twice")
                return False
            changed = [k for k, v in self.ref_tags().items() if ref.get(k) != v] + \
                      (["label"] if ref.get("label") != label else [])
            if changed:
                self.say(f"The remembered camera height belongs to another set-up ({', '.join(changed)} changed): "
                         "nothing is used. Jog clear, press 0 twice, then set the heights and offsets again")
                return False
            s = self.centred_scale(label)
            if s is None:
                self.say(f"{label.upper().replace('_', ' ')} not seen: the camera height can't be checked. Jog near "
                         "it, then h")
                return False
            tol = float(self.zcfg["restore_tol"])
            for _ in range(int(self.zcfg["restore_rounds"])):
                if abs(s / float(ref["px_mm"]) - 1.0) <= tol:
                    break
                if not ref.get("f_mm"):
                    f = self.probe_f(label, s)
                    if f is None:
                        self.say("The camera height can't be measured (no scale change for a 1 mm move): check the "
                                 "label and the focus, then h")
                        return False
                    ref["f_mm"] = round(float(f), 3)
                    save_cal(self.cal)
                up = float(ref["f_mm"]) * (1.0 / float(ref["px_mm"]) - 1.0 / s)     # + = the camera is too low
                end = self.z_now() + up               # where Z would end, from the height h was pressed at
                if end < -float(self.zcfg["restore_max_down_mm"]) or end > float(self.zcfg["max_drift_mm"]):
                    self.say(f"The camera is {abs(up):.2f} mm {'HIGHER' if up < 0 else 'LOWER'} than the remembered "
                             "height: too much to move by itself (Z moved by hand, or the camera moved on the head). "
                             f"Jog Z {'down' if up < 0 else 'up'} about {abs(up):.1f} mm, then h. Or make this the new "
                             "camera height (0 twice) and set the heights again")
                    return False
                self.say(f"   camera height: Z {up:+.2f} mm back to the remembered one")
                z = self.z_now() + up
                if up > 0:
                    self.z_to(z + 0.5)                # come down onto it: Z has some play
                self.z_to(z)
                self.camera_moved_z = True
                s = self.centred_scale(label)
                if s is None:
                    self.say(f"{label} lost from view while restoring the camera height")
                    return False
            if abs(s / float(ref["px_mm"]) - 1.0) > tol:
                self.say("The camera height could not be brought back (it does not settle): check Z for lost steps")
                return False
            self.z_base += self.z_now()               # this height is Z0 from now on
            self.z, self.z_ok = 0.0, True
            self.lifted_at = None
            return True
        finally:
            self.restoring = False

    def camera_height_still_right(self) -> bool:
        """Before the first r of a session: the reference label must still have its size (Z did not slip)."""
        ref = self.cal.get("z_reference") or {}
        self.restoring = True
        try:
            s = self.centred_scale(str(self.zcfg["label"]))
        finally:
            self.restoring = False
        if s is None or abs(s / float(ref["px_mm"]) - 1.0) > 3 * float(self.zcfg["restore_tol"]):
            self.z_ok = False
            self.say("Z is not at the remembered camera height any more (lost steps?): press h. Nothing goes down")
            return False
        self.checked_for_r = True
        return True

    def job_new_reference(self) -> None:
        """Key 0, twice: make the camera height of now the remembered one. Every height and offset must then be
        set again (the old ones stay in the file, but are refused)."""
        if not self.new_ref_armed:
            self.new_ref_armed = True
            self.say("Press 0 again to make THIS the camera height (Z0). Then set every height (k, d, m, t) and "
                     "offset (n) again. Any other key: nothing changes")
            return
        self.new_ref_armed = False
        if self.home is None:
            self.say("HOME is not known yet: press h first")
            return
        label = str(self.zcfg["label"])
        old = [v for v in (self.touch_z(n, quiet=True) for n in ("paper", "part", "place")) if v is not None]
        gap = float(self.ncfg["move_clearance_mm"])
        if old and self.z_now() < max(old) + gap:
            self.say(f"Here the nozzle is less than {gap:g} mm above the heights set so far: go higher first")
            return
        self.restoring = True                     # the owner just said this height is clear: X / Y may move here
        try:
            if label not in self.visible() and label in self.nominal:
                self.say(f"Going to {label.upper().replace('_', ' ')} (where the plan puts it) to measure the camera height")
                self.move_to(self.clip_to_reach(self.home + self.nominal[label] - self.nominal["home"]))
            s = self.centred_scale(label)
        finally:
            self.restoring = False
        if s is None:
            self.say(f"{label.upper().replace('_', ' ')} not seen: jog so its black square is near the red cross "
                     "(lift first if the arrows are refused), then 0 twice")
            return
        self.new_reference(label, s)

    def marker_scale(self, name: str) -> float | None:
        """px per mm of a bed label, from its four sides averaged over several pictures (camera height)."""
        c = self.look(name, frames=int(self.zcfg["frames"]))
        if c is None:
            return None
        return float(np.mean(np.linalg.norm(np.roll(c, -1, axis=0) - c, axis=1))) / self.markers.size_mm

    def scale_of_id(self, marker_id: int) -> tuple[float, np.ndarray] | None:
        """px per mm and centre (px) of any label by its ArUco id (also one that is not on the bed plan)."""
        sides, centres = [], []
        n = int(self.zcfg["frames"])
        for f in self.cam.frames_after(time.monotonic() + t.SETTLE_S, n):
            corners, ids, _ = self.markers.detector.detectMarkers(f)
            for c, i in zip(corners or [], [] if ids is None else ids.ravel()):
                if int(i) == marker_id:
                    c = c.reshape(4, 2)
                    sides.append(np.mean(np.linalg.norm(np.roll(c, -1, axis=0) - c, axis=1)))
                    centres.append(c.mean(axis=0))
        if len(sides) < (n + 1) // 2:
            return None
        return float(np.mean(sides)) / self.markers.size_mm, np.mean(centres, axis=0)

    def measure_home_scale(self) -> None:
        self.z_to(0)
        self.move_to(self.clip_to_reach(self.home))
        self.home_px_mm = self.marker_scale("home")

    def servo_build(self) -> bool:
        if firmware_ok(self.grbl, self.scfg):
            return True
        self.say("This Uno does not run the servo Grbl build: see firmware/README.md")
        return False

    # ---- jobs started by keys
    def job_start(self) -> None:
        """h: lift, find HOME, bring the camera back to the remembered height (Z0), and measure again there."""
        if getattr(self.grbl, "reset_seen", False) and not self.z_locked:
            self.lock_after_reset()                   # first the owner lifts by hand, then h again
            return
        reset = bool(getattr(self.grbl, "reset_seen", False)) or "reset" in (self.z_locked or "")
        if reset:
            self.z_ok = False                         # never go back to a Z0 of the old Grbl frame
        if self.z_locked:                         # the owner lifted the nozzle by hand and pressed h
            self.say("Going on from here: the nozzle was lifted by hand")
            self.z_locked = None
        if hasattr(self.grbl, "reset_seen"):
            self.grbl.reset_seen = False
        if reset:                                 # Grbl's X / Y restarted at 0 too: forget the old frame
            self.home, self.extremes = None, None
            st = self.grbl.status()[1]
            self.pos = np.array([st["X"], st["Y"]], float)
        if self.z_ok:
            self.z_to(0)                          # a checked Z0: go back to it
            self.start_lift = 0.0
        else:                                     # not checked: start from here (keeps the owner's own jog)
            grbl_z = float(self.grbl.status()[1]["Z"])
            lifted = getattr(self, "lifted_at", None)
            if lifted is not None and abs(grbl_z - lifted) < 0.1:
                pass                              # h again, no jog since the last lift: don't lift once more
            else:
                self.z_base += self.z_now()
                self.z = 0.0
                self.start_lift = 0.0
                if self.cal.get("z_reference"):   # Z only slides DOWN: lift before any X / Y move, the restore
                    self.start_lift = float(self.zcfg["max_drift_mm"])    # then comes back down onto Z0
                    self.say(f"Lifting {self.start_lift:g} mm before moving (Z may have slid down while the power "
                             "was off)")
                    self.z_to(self.start_lift)
                    self.lifted_at = float(self.grbl.status()[1]["Z"])
        self.z_ok = False
        self.checked_for_r = False
        self.nozzle_target = None                 # a first n of before is not valid any more
        self.M = self.c = None                    # after re-homing, the board and the slot are found again (r)
        self.slot_view = None
        if not self.find_home():
            return
        if not self.restore_camera_height():
            return
        if self.camera_moved_z and not self.find_home():     # J and HOME again, at the remembered height
            return
        self.measure_home_scale()
        self.say("Ready (camera at the remembered height). Heights: k pick, d place, m moving, t paper. Offsets: n. "
                 "Then r")

    def job_jog(self, dist) -> None:
        if self.z_locked or not self.nozzle_up(float(self.ncfg["jog_clearance_mm"])):
            self.say("Lift the nozzle first (] or }): no X / Y moves with the nozzle down"
                     + (f" ({self.z_locked})" if self.z_locked else ""))
            return
        super().job_jog(dist)

    def job_jog_z(self, dz: float) -> None:
        self.z_to(self.z_now() + dz, by_hand=True)
        self.say(f"Z {self.z:+.2f} mm from Z0")

    def job_set_spot(self, name: str) -> None:
        if self.home is None:
            self.say("HOME is not known yet")
            return
        self.cal[f"{name}_xy_from_home"] = np.round(self.pos - self.home, 2).tolist()
        save_cal(self.cal)
        self.say(f"{name} spot remembered: X{self.pos[0] - self.home[0]:+.1f} Y{self.pos[1] - self.home[1]:+.1f}")

    def job_touch(self, name: str) -> None:
        """Remember the Z of now as one of the heights (t paper, k pick, d place, m moving), with the HOME
        label's scale of this session (to notice a Z that slid before the height is used again)."""
        if self.z_locked or getattr(self.grbl, "reset_seen", False):
            self.z_ok = False
            self.say("Grbl was reset or the machine is locked: nothing is kept. Lift the nozzle by hand (}), then h")
            return
        if not self.z_ok:
            self.say("The camera height is not checked yet in this session: press h")
            return
        z = self.z_now()
        gap = float(self.ncfg["move_clearance_mm"])
        if name == "place" and not self.pump_on:
            self.say("The place height is set WITH a part on the nozzle: pump on (a), hold a part, then d")
            return
        if name != "move" and z > -gap:
            self.say(f"This is less than {gap:g} mm below Z0 (the camera height): Z0 is too low for it. Lift the "
                     "head, press 0 twice for a higher camera height, then set the heights again")
            return
        if name == "move":
            low = [v for v in (self.touch_z(n, quiet=True) for n in ("paper", "part", "place")) if v is not None]
            if z > 1e-6:
                self.say("The moving height can't be above Z0 (the camera height): not kept")
                return
            if low and z < max(low) + gap:
                self.say(f"Moving height Z {z:+.2f} is less than {gap:g} mm above the pick / place / paper heights "
                         f"(highest {max(low):+.2f}): not kept. Go higher, then m")
                return
        self.cal[f"touch_{name}"] = {"z": round(z, 3), "ref": self.ref_id(),
                                     "date": datetime.datetime.now().isoformat(timespec="seconds")}
        save_cal(self.cal)
        self.say(f"{HEIGHT_WHAT[name].capitalize()} kept: Z {z:+.2f} mm"
                 + (" (parts are carried at this height)" if name == "move" else ". Lift the nozzle before moving"))


    def job_nozzle_point(self) -> None:
        """Two presses of n. 1st: the camera centres on what is nearest the red cross (a label, a part in the
        tape or a board hole) and keeps where it is. 2nd, after the nozzle tip was jogged right above it:
        offset = that place - head. On a part it is the PICK offset, on a board hole the PLACE offset (the
        tilted camera makes the offset depend on the height of what it looks at)."""
        if self.home is None:
            self.say("HOME is not known yet")
            return
        if self.nozzle_target is None:
            if not self.z_ok:
                self.say("The camera height is not checked yet in this session: press h")
                return
            self.z_to(0)                          # X / Y moves and the camera only at Z0
            if not self.no_servo and self.servo_build():
                self.servo(float(self.scfg["centre_us"]))     # offsets are for the centre pulse
            got = self.target_here()
            if got is None:
                self.say("Nothing to centre on: put a label, a part in the tape or a board hole near the red "
                         "cross, then n")
                return
            self.nozzle_target = got
            kind, name, _ = got
            self.say(f"{name.upper().replace('_', ' ')} kept. Now jog the NOZZLE tip right above its centre "
                     "(arrows; look from the front and from the side), then press n again")
            return
        kind, name, xy = self.nozzle_target
        self.nozzle_target = None
        if self.z_locked or getattr(self.grbl, "reset_seen", False) or not self.z_ok:
            self.say("Grbl was reset or the machine is locked: no offset kept. Lift by hand (}), then h")
            return
        off = xy - self.pos
        self.cal["offset_guess_mm"] = np.round(off, 1).tolist()
        key = {"part": "offset_pick", "hole": "offset_place"}.get(kind)
        if key:
            self.cal[key] = {"xy": np.round(off, 3).tolist(), "by": "eye (n)", "ref": self.ref_id() if self.z_ok else None,
                             "date": datetime.datetime.now().isoformat(timespec="seconds")}
        save_cal(self.cal)
        what = {"part": "PICK offset", "hole": "PLACE offset"}.get(kind, "rough offset")
        self.say(f"{what}: nozzle tip = camera centre + X{off[0]:+.2f} Y{off[1]:+.2f} mm (by eye)."
                 + (" Now do the other one (n on a part / on a board hole)" if key else
                    " The offset job (o) starts from this"))

    def target_here(self) -> tuple[str, str, np.ndarray] | None:
        """What is nearest the picture centre: a label, a dark spot (a part, a mark) or a board hole.
        The camera centres on it. Returns (kind, name, its machine xy) or None."""
        img = self.frame()
        centre = self.centre_px()
        cands = [(float(np.hypot(*(c.mean(axis=0) - centre))), "label", name)
                 for name, c in self.markers.find(img).items()]
        cands += [(float(np.hypot(*(px - centre))), "part", "part") for _, _, px in
                  d.dark_spots(img, self.J, self.pos, centre, self.parts_cfg)]
        cands += [(float(np.hypot(*(px - centre))), "hole", "board hole") for px in
                  d.hole_candidates(img, d.HOLE_D_MM / 2 * self.px_per_mm(), keep=6, min_contrast=0.0)]
        if not cands:
            return None
        _, kind, name = min(cands)
        xy = self.locate(name) if kind == "label" else self.centre_on(kind)
        return None if xy is None else (kind, name, xy)

    def spot_xy(self, kind: str, img: np.ndarray) -> np.ndarray | None:
        """Machine xy of the dark spot / board hole nearest the picture centre in img."""
        centre = self.centre_px()
        if kind == "part":
            pts = [px for _, _, px in d.dark_spots(img, self.J, self.pos, centre, self.parts_cfg)]
        else:
            pts = list(d.hole_candidates(img, d.HOLE_D_MM / 2 * self.px_per_mm(), keep=6, min_contrast=0.0))
        if not pts:
            return None
        px = min(pts, key=lambda q: np.hypot(*(q - centre)))
        return self.pos + np.linalg.solve(self.J, centre - px)

    def centre_on(self, kind: str) -> np.ndarray | None:
        """Bring the nearest dark spot / hole to the picture centre (near the centre the lens and the tilt
        measure best); its xy, measured after the last move."""
        xy = self.spot_xy(kind, self.frame())
        for _ in range(3):
            if xy is None:
                return None
            self.move_to(self.clip_to_reach(xy))
            new = self.spot_xy(kind, self.frame())
            if new is None or np.hypot(*(new - xy)) > 2.0:
                return None
            xy, near = new, np.hypot(*(new - self.pos)) < 0.05
            if near:
                break
        return xy

    def job_centre(self) -> None:
        if self.servo_build():
            self.servo(float(self.scfg["centre_us"]))
            self.say("Servo at the centre")

    def job_offset(self) -> None:
        """Marks at several servo positions -> nozzle tip offset per pulse, servo shaft position, runout."""
        if self.cal.get("offset_guess_mm") is None and not self.need("offset_guess_mm"):
            self.say("... or press n twice to get it with the camera")
            return
        if not (self.need("od_mm") and (self.no_servo or self.servo_build())):
            return
        paper, touch = self.spot("paper", "p"), self.touch_z("paper")
        if paper is None or touch is None:
            return
        nc = self.ncfg
        od, press, spacing = float(nc["od_mm"]), float(nc["press_mm"]), float(nc["spacing_mm"])
        pulses = [float(p) for p in nc["mark_pulses_us"]]
        if self.no_servo:
            pulses = [float(self.scfg["centre_us"])] * 3      # one angle: three marks for the repeat check
        spots = [paper + grid(k, spacing) for k in range(len(pulses))]
        self.z_to(0)
        self.stage("OFFSET: is the paper clean?")
        self.move_to(self.clip_to_reach(np.mean(spots, axis=0)))
        img = self.frame()
        old = [s for s in spots if find_mark(img, self.J, self.pos, self.centre_px(), s, 2.0, od,
                                             float(nc["mark_dark"])) is not None]
        if old:
            self.say("There are marks on the paper already: use a clean piece, or a new paper spot (p)")
            return
        o_est = np.array(self.cal.get("offset_guess_mm") or nc["offset_guess_mm"], float)
        found: list[tuple[float, np.ndarray]] = []
        for k, (pulse, s) in enumerate(zip(pulses, spots)):
            self.stage(f"OFFSET [{k + 1}/{len(pulses)}] servo {pulse:.0f} us: mark")
            head = s - o_est
            if not self.in_reach(head - self.home):
                self.say("The nozzle cannot reach the paper spot: set it nearer the middle (p)")
                return
            self.servo(pulse)
            self.move_to(head)
            self.z_to(touch - press)
            time.sleep(float(nc["dwell_s"]))
            self.z_to(0)
            self.move_to(self.clip_to_reach(s))
            img = self.frame()
            m = find_mark(img, self.J, self.pos, self.centre_px(), s, 6.0 if not found else 1.5, od,
                          float(nc["mark_dark"]))
            if m is None:
                self.say(f"No mark found near spot {k + 1}: check the putty / ink, od_mm and the touch height")
                cv2.imwrite(str(self.out_dir / f"mark_{k + 1}_missing.png"), img)
                return
            o_est = m - head
            found.append((pulse, o_est))
            self.overlay = [(self._circle(m, od / 2), (0, 255, 0), f"{pulse:.0f} us")]
            self.snapshot_overlay(f"mark_{k + 1}")
            self.say(f"   {pulse:.0f} us: nozzle tip = camera + X{o_est[0]:+.3f} Y{o_est[1]:+.3f} mm")
        self.servo(float(self.scfg["centre_us"]))
        by: dict[float, list] = {}
        for p, o in found:
            by.setdefault(p, []).append(o)
        repeat = max((float(np.hypot(*(v[0] - v[-1]))) for v in by.values() if len(v) > 1), default=None)
        by_pulse = {int(p): np.mean(v, axis=0) for p, v in by.items()}
        pts = np.array(list(by_pulse.values()))
        if len(by_pulse) >= 3 and np.ptp(pts, axis=0).max() > 0.05:     # a real circle, not one spot
            axis, runout = fit_circle(list(by_pulse.values()))
        else:
            axis, runout = np.mean(list(by_pulse.values()), axis=0), 0.0
        self.cal["offset"] = {
            "by_pulse_us": {p: np.round(o, 3).tolist() for p, o in sorted(by_pulse.items())},
            "shaft_xy": np.round(axis, 3).tolist(), "runout_mm": round(runout, 3),
            "repeat_mm": None if repeat is None else round(repeat, 3),
            "ref": self.ref_id() if self.z_ok else None,
            "date": datetime.datetime.now().isoformat(timespec="seconds"),
            "note": "nozzle tip = camera centre + offset; shaft = the servo axis; runout = how far the tip is off it"}
        save_cal(self.cal)
        ok = repeat is not None and repeat <= 0.05
        self.say(f"OFFSET DONE: shaft at camera + X{axis[0]:+.2f} Y{axis[1]:+.2f} mm, runout {runout:.2f} mm, "
                 + ("repeat not measured" if repeat is None else f"repeat {repeat:.3f} mm")
                 + (" (PASS)" if ok else " (FAIL: must be <= 0.05 mm, docs/08 step 7)"))
        self.done = True

    def job_servo(self) -> None:
        """Pick, turn, place, measure the angle -> pulse to angle table."""
        if self.no_servo:
            self.say("Started with --no-servo: the servo table needs the servo wired")
            return
        if not (self.need("part_height_mm") and self.servo_build()):
            return
        if "offset" not in self.cal:
            self.say("Run the OFFSET job first (o)")
            return
        place = self.spot("tape", "l")
        t_part, t_paper = self.touch_z("part"), self.touch_z("paper")
        if place is None or t_part is None or t_paper is None:
            return
        nc, by_pulse = self.ncfg, self.cal["offset"]["by_pulse_us"]
        press, centre = float(nc["press_mm"]), float(self.scfg["centre_us"])
        pulses = [float(p) for p in nc["table_pulses_us"]]
        slot = f"slot_{self.args.slot}"
        self.z_to(0)
        self.servo(centre)
        self.stage(f"SERVO: find the parts in {slot}")
        self.move_to(self.clip_to_reach(self.home + self.nominal[slot] - self.nominal["home"]))
        frame = self.slot_frame(slot)
        if frame is None:
            self.say(f"{slot} not found")
            return
        parts = [p for p in self.search_slot(slot, frame) if not p["unclear"]]
        if len(parts) < len(pulses):
            self.say(f"{slot} has {len(parts)} clear parts, the table needs {len(pulses)}: refill it")
            return
        rows = []
        for k, (pulse, part) in enumerate(zip(pulses, parts)):
            self.stage(f"SERVO [{k + 1}/{len(pulses)}] pick")
            self.z_to(0)
            self.servo(centre)
            got = self.centre_on_part(part, frame)
            if got is None:
                self.say("   the part was not seen again: skipped")
                continue
            a_pick = d.part_axis_deg(got["angle_deg"], frame)
            head = got["xy"] - offset_for(centre, by_pulse)
            if not self.in_reach(head - self.home):
                self.say(f"The nozzle cannot reach the parts in {slot} (the camera would have to go outside the "
                         "reach): the nozzle sits too far from the camera (docs risk K9)")
                return
            self.move_to(head)
            self.pump(True)
            self.z_to(t_part - press)
            time.sleep(float(nc["dwell_s"]))
            self.z_to(0)
            self.stage(f"SERVO [{k + 1}/{len(pulses)}] turn to {pulse:.0f} us, place")
            self.servo(pulse)
            target = place + grid(k, float(nc["spacing_mm"]))
            head = target - offset_for(pulse, by_pulse)
            if not self.in_reach(head - self.home):
                self.pump(False)
                self.say("The nozzle cannot reach the tape spot: set it nearer the middle (l)")
                return
            self.move_to(head)
            self.z_to(t_paper + float(nc["part_height_mm"]) - press)
            self.pump(False)
            time.sleep(float(nc["release_s"]))
            self.z_to(0)
            self.move_to(self.clip_to_reach(target))
            img = self.frame()
            zone = np.array([[-4, -4], [4, -4], [4, 4], [-4, 4]], float)
            seen = [p for p in d.find_slot_parts(img, self.J, self.pos, self.centre_px(), target, np.array([1.0, 0.0]),
                                                 np.array([0.0, 1.0]), zone, self.parts_cfg, 0.5) if not p["unclear"]]
            if not seen:
                self.say("   no part found on the tape (was it picked? dropped?): skipped")
                cv2.imwrite(str(self.out_dir / f"place_{k + 1}_missing.png"), img)
                continue
            p_placed = min(seen, key=lambda p: np.hypot(*(p["xy"] - target)))
            rows.append({"pulse_us": pulse, "raw_deg": float(p_placed["angle_deg"] - a_pick),
                         "landing_error_mm": round(float(np.hypot(*(p_placed["xy"] - target))), 3)})
            self.overlay = [(d._closed(p_placed["corners_xy"]), (0, 255, 0), f"{pulse:.0f} us")]
            self.snapshot_overlay(f"place_{k + 1}")
        self.z_to(0)
        self.servo(centre)
        if len(rows) < 3:
            self.say("SERVO: fewer than 3 good placements, no table")
            return
        pulses_done, raw = [r["pulse_us"] for r in rows], [r["raw_deg"] for r in rows]
        tries = [(lambda tu: (fit_table(pulses_done, tu, centre), tu))(unwrap_turns(pulses_done, raw, centre, s))
                 for s in (1.0, -1.0)]
        fit, turns = min(tries, key=lambda ft: ft[0]["worst_deg"])      # the servo's direction: the one that fits
        self.cal["servo"] = {
            "pulse_to_turn_deg": [[int(r["pulse_us"]), round(tu, 2)] for r, tu in zip(rows, turns)],
            "fit": fit, "landing_error_mm": [r["landing_error_mm"] for r in rows],
            "date": datetime.datetime.now().isoformat(timespec="seconds"),
            "note": f"turn of the nozzle from the pick at {centre:.0f} us (the centre), in degrees, + = anticlockwise "
                    "seen from above"}
        save_cal(self.cal)
        self.say(f"SERVO DONE: {fit['deg_per_us'] * 1000:.1f} deg per 1000 us, worst point {fit['worst_deg']:.2f} deg "
                 + ("(PASS)" if fit["worst_deg"] <= 1.0 else "(FAIL: must be <= 1 deg, R1)"))
        self.done = True


    # ---- the real pick and place (key r): one part per press
    def real_offsets(self) -> tuple[np.ndarray, np.ndarray, str] | None:
        """Nozzle tip = camera centre + offset: the PICK offset (n on a part) and the PLACE offset (n on a board
        hole). With the tilted camera an offset is right only at the height it was measured at and at the
        camera height of then, so the offset job's (paper height) and the rough guess are not used here."""
        out = []
        for key, what, where in (("offset_pick", "pick", "a part in the tape"), ("offset_place", "place", "a board hole")):
            entry = self.cal.get(key)
            if not entry:
                self.say(f"No {what} offset: press n twice on {where}")
                return None
            if entry.get("ref") is None or entry.get("ref") != self.ref_id():
                self.say(f"The {what} offset was measured at another camera height: press n twice on {where} again")
                return None
            out.append(np.array(entry["xy"], float))
        return out[0], out[1], "n on a part (pick), n on a board hole (place)"

    def real_plan(self) -> dict | None:
        """Everything the real pick and place needs, checked before anything moves down. None = not ready
        (the message says what is missing)."""
        if self.home is None or not self.z_ok:
            self.say("HOME and the camera height are not checked yet in this session (press h)")
            return None
        if self.pump_on or self.part_on_nozzle:
            self.say("The pump is on or a part may still be on the nozzle: jog over the tape (or anywhere safe), "
                     "press a to let it go, then r")
            return None
        pick_touch, place_z = self.touch_z("part"), self.touch_z("place")
        if pick_touch is None or place_z is None:
            return None
        move_z = self.touch_z("move", quiet=True)
        if move_z is None:
            if self.cal.get("touch_move") is not None:
                self.touch_z("move")                  # set, but Z changed since: say why it is refused
                return None
            move_z = 0.0
        pick_z = pick_touch - float(self.ncfg["press_mm"])
        gap = float(self.ncfg["move_clearance_mm"])
        highest = max(pick_touch, place_z)
        if 0.0 < highest + gap:
            self.say(f"Z0, the camera height (the camera looks and moves there, also with a part), is less than {gap:g} mm "
                     f"above the pick / place heights ({pick_touch:+.2f}, {place_z:+.2f}). Restart the program with "
                     "the head higher, then set k and d again")
            return None
        if move_z < highest + gap:
            self.say(f"The moving height (Z {move_z:+.2f}) must be at least {gap:g} mm above the pick and place heights "
                     f"({pick_touch:+.2f}, {place_z:+.2f}): set it higher (m)")
            return None
        offs = self.real_offsets()
        if offs is None:
            return None
        if not (self.no_servo or self.servo_build()):
            return None
        fit = (self.cal.get("servo") or {}).get("fit")
        if fit and float(fit.get("worst_deg", 99)) > 1.0:
            self.say(f"The servo table's worst point is {fit.get('worst_deg', '?')} deg (must be <= 1): not used, "
                     "turns are nominal. Measure it again (v)")
            fit = None
        return {"pick_z": pick_z, "place_z": place_z, "move_z": move_z, "pick_off": offs[0], "place_off": offs[1],
                "offsets_from": offs[2], "servo_fit": fit}

    def pulse_for_turn(self, turn_deg: float) -> float:
        """Servo pulse that turns the nozzle by turn_deg from the pick at the centre pulse (+ = anticlockwise
        seen from above): the servo table if there is one (v), else the nominal 270 deg over 2000 us."""
        s, fit = self.scfg, self.real["servo_fit"] if self.real else None
        centre = float(s["centre_us"])
        if fit:
            pulse = centre + (turn_deg - float(fit["offset_deg"])) / float(fit["deg_per_us"])
        else:
            pulse = centre + turn_deg / NOMINAL_DEG_PER_US
        return float(np.clip(pulse, float(s["min_us"]), float(s["max_us"])))

    def runout_at(self, pulse: float) -> np.ndarray:
        """How far the tip moves when the servo turns from the centre (needle not on the shaft): from the
        offset job (o), else 0."""
        o = self.cal.get("offset")
        if not o:
            return np.zeros(2)
        return offset_for(pulse, o["by_pulse_us"]) - offset_for(float(self.scfg["centre_us"]), o["by_pulse_us"])

    def needed_turn(self, got: dict, part: dict) -> float | None:
        """Nozzle turn (deg, + anticlockwise) that brings the picked part's long side onto its two pads.
        A two-pad part looks the same turned by 180 deg: the smallest turn is taken. None = not two pads."""
        if len(part["pads"]) != 2:
            return None
        c = [self.board_to_machine(np.array(pad["p"][0]).reshape(-1, 2)).mean(axis=0) for pad in part["pads"]]
        pads_deg = float(np.degrees(np.arctan2(*(c[1] - c[0])[::-1])))
        return float((pads_deg - d.part_axis_deg(got["angle_deg"], self.slot_view) + 90.0) % 180.0 - 90.0)

    def job_real(self) -> None:
        """Key r: pick and place ONE part (the next one). The first press finds the board and the slot's parts."""
        plan = self.real_plan()
        if plan is None:
            return
        self.z_to(0)                                  # every press: X / Y moves and the camera only at Z0
        if not getattr(self, "checked_for_r", False) and not self.camera_height_still_right():
            return
        self.real = plan
        if self.M is None:
            self.say(f"REAL: offsets from {plan['offsets_from']}; heights pick {plan['pick_z']:+.2f}, place "
                     f"{plan['place_z']:+.2f}, moving {plan['move_z']:+.2f}"
                     + ("" if plan["servo_fit"] or self.no_servo else "; servo table not measured (v): turns are "
                        "nominal"))
            if not self.register():
                return
        if getattr(self, "slot_view", None) is None:
            if not self.prepare_slot(reset=getattr(self, "next_k", None) is None):
                return
        try:
            more = self.place_next()
        except BaseException:
            self.interrupted()
            raise
        self.save_report()
        done = len(self.report.get("placements", []))
        self.say(f"REAL: {done} placed so far" + ("" if more else ": nothing more to place (slot empty or all done)")
                 + (". Press r for the next part" if more else ""))

    def interrupted(self) -> None:
        """A stop or an error in the middle of a part: the place is done again with the next part, and a part
        that may still be on the nozzle is kept there (pump on) and lifted, never pushed onto another part."""
        k = getattr(self, "current_k", None)
        if k is not None:
            self.next_k, self.current_k = k, None
            self.report.setdefault("interrupted", []).append(self.todo[k]["ref"])
        if self.pump_on:
            self.part_on_nozzle = True
            try:
                self.z_to(max(self.real["move_z"] if self.real else 0.0, 0.0))
            except Exception:
                pass
            self.say("INTERRUPTED with the pump on: a part may be on the nozzle. Jog over the tape, press a to let "
                     "it go, then r goes on with the same place")

    def job_pump(self) -> None:
        self.pump(not self.pump_on)
        if not self.pump_on:
            self.part_on_nozzle = False
        self.say("Pump ON: the nozzle sucks (it can hold a part)" if self.pump_on else "Pump OFF")

    def at_pick(self, got: dict, n: int, part: dict) -> bool | str:
        if self.real is None:
            return super().at_pick(got, n, part)
        r = self.real
        head = got["xy"] - r["pick_off"]
        turn = self.needed_turn(got, part)
        centre = float(self.scfg["centre_us"])
        pulse = centre if turn is None or self.no_servo else self.pulse_for_turn(turn)
        target = self.board_to_machine([part["x"], part["y"]])
        place_head = target - r["place_off"] - self.runout_at(pulse)
        for what, h in (("part", head), ("place", place_head)):
            if not self.in_reach(h - self.home):
                rel = h - self.home
                self.say(f"   the nozzle cannot reach the {what} (head at X{rel[0]:+.1f} Y{rel[1]:+.1f} from HOME, "
                         "outside the reach): skipped")
                return what
        self.pending = {"turn": turn, "pulse": pulse, "head": place_head}
        self.z_to(0)
        if not self.no_servo:
            self.servo(centre)
        self.move_to(head)
        self.pump(True)
        self.say(f"   pick {n}: nozzle down to Z {r['pick_z']:+.2f}, pump on")
        self.z_from_above(r["pick_z"])
        time.sleep(float(self.ncfg["dwell_s"]))
        self.z_to(0)                                  # the camera height: look into the pocket
        self.move_to(self.clip_to_reach(got["xy"]))
        left = [s for s in d.dark_spots(self.frame(), self.J, self.pos, self.centre_px(), self.parts_cfg)
                if np.hypot(*(s[0] - got["xy"])) < float(self.ncfg["after_check_mm"])]
        if left:
            self.pump(False)
            self.snapshot_overlay(f"missed_{part['ref']}")
            self.say(f"   part {n} is still in its pocket: the pick missed (pick offset or pick height?). Pump off. "
                     "The next r tries this place with the next part")
            return "missed"
        self.z_to(r["move_z"])
        return True

    def at_place(self, target: np.ndarray, part: dict, got: dict) -> bool | dict:
        if self.real is None:
            return super().at_place(target, part, got)
        r, pend = self.real, self.pending
        turn, pulse, head = pend["turn"], pend["pulse"], pend["head"]
        if not self.no_servo:
            self.servo(pulse)
        self.z_to(max(r["move_z"], self.z))
        self.move_to(head)
        self.say(f"   place {part['ref']}: nozzle down to Z {r['place_z']:+.2f}"
                 + ("" if turn is None else f", turned {turn:+.1f} deg ({pulse:.0f} us)") + ", pump off")
        self.z_from_above(r["place_z"])
        self.pump(False)
        self.current_k = None                         # let go: this place is done, whatever happens next
        record = {"ref": part["ref"], "value": part["val"], "footprint": part["fp"],
                  "board_xy": [part["x"], part["y"]], "kicad_rot": part["rot"],
                  "machine_xy_from_home": np.round(target - self.home, 3).tolist(),
                  "turn_deg": None if turn is None else round(turn, 2), "pulse_us": round(pulse),
                  "seen_off_mm": "not checked"}
        self.report["placements"].append(record)
        time.sleep(float(self.ncfg["release_s"]))
        self.z_to(0)
        if not self.no_servo:
            self.servo(float(self.scfg["centre_us"]))
        self.move_to(self.clip_to_reach(target))
        img = self.frame()
        seen = [s for s in d.dark_spots(img, self.J, self.pos, self.centre_px(), self.parts_cfg)
                if np.hypot(*(s[0] - target)) < float(self.ncfg["after_check_mm"])]
        self.snapshot_overlay(f"after_{part['ref']}")
        off = None if not seen else min((s[0] - target for s in seen), key=lambda v: np.hypot(*v))
        self.say(f"   {part['ref']}: " + ("no dark part seen at its place (look at after_" + part["ref"] + ".png)"
                                          if off is None else f"a part is seen {np.hypot(*off):.2f} mm from its place "
                                          "(not corrected for the camera tilt: the part top is higher than the pads)"))
        record["seen_off_mm"] = None if off is None else np.round(off, 3).tolist()
        return {"recorded": True}

    def submit(self, fn, *args) -> bool:
        """Every job: if Grbl was reset while it ran, lock at once (nothing moves until the owner lifted by hand)."""
        def job(*a):
            try:
                fn(*a)
            finally:
                if getattr(self.grbl, "reset_seen", False) and not self.z_locked:
                    self.lock_after_reset()
        return super().submit(job, *args)

    def lock_after_reset(self) -> None:
        self.z_locked = "Grbl was reset (power dip?): the position is lost"
        self.z_ok = False
        self.say("Grbl was reset: the position is lost. Lift the nozzle by hand (} or ]) until it is clear, then h")

    def park(self) -> bool:
        """At quit: back up to the checked Z0 (True = parked: the next start trusts it). Z0 not checked in this
        session: only UP, to where the session started, never down (False). Locked: no Z move (False)."""
        if self.z_locked:
            return False
        if self.z_ok:
            self.z_to(0)
            return True
        start = -self.z_base                          # Grbl Z 0 = the height this session started at
        if self.z_now() < start:
            self.z_to(start)
        return bool(getattr(self, "started_parked", False))   # back where a parked session started: still parked

    def status_extra(self) -> str | None:
        def h(name):
            v = self.touch_z(name, quiet=True)
            return "--" if v is None else f"{v:+.2f}"
        return (f"Z now {self.z:+.2f}   pick {h('part')}  place {h('place')}  moving {h('move')}  paper {h('paper')}"
                f"   pump {'ON' if self.pump_on else 'off'}")

    def job_z(self) -> None:
        """Z sweep over HOME: camera scale at each Z -> straight line, play, repeatability, drift correction."""
        if self.home is None or not self.z_ok:
            self.say("HOME and the camera height are not checked yet in this session (press h)")
            return
        zs = [float(z) for z in self.zcfg["sweep_mm"]]
        touch = self.touch_z("paper", quiet=True)
        if touch is not None and min(zs) < touch + 1.0:
            self.say(f"The sweep goes to Z {min(zs):+.1f}, closer than 1 mm to the paper touch height: "
                     "make config/bench.yaml z_cal.sweep_mm shallower")
            return
        self.z_to(0)
        self.move_to(self.clip_to_reach(self.home))
        self.z_to(0.5)                     # so the first point is also reached going down (Z has some play)
        rows = []
        for k, z in enumerate(zs):
            self.stage(f"Z SWEEP [{k + 1}/{len(zs)}] Z {z:+.1f} mm: measuring the HOME label")
            self.z_to(z)
            scale = self.marker_scale("home")
            if scale is None:
                self.say(f"   HOME not seen at Z {z:+.1f}: the sweep stops here")
                break
            rows.append((z, float(scale)))
            self.say(f"   Z {z:+.1f}: {scale:.4f} px/mm")
        self.z_to(0)
        if len(rows) < 3 or len({z for z, _ in rows}) < 2:
            self.say("Z SWEEP: too few points")
            return
        down = 1                           # the line comes from the first downward run only: no play in it
        while down < len(rows) and rows[down][0] < rows[down - 1][0]:
            down += 1
        line = fit_z_sweep([z for z, _ in rows[:down]], [s for _, s in rows[:down]])
        fit = {**line, "z_seen_mm": 1.0 / np.array([s for _, s in rows]) / line["slope"] - line["h0_mm"]}
        fit["worst_mm"] = float(np.abs(line["z_seen_mm"] - np.array([z for z, _ in rows[:down]])).max())
        seen = fit["z_seen_mm"]
        play, repeat = [], []
        for i, (z, _) in enumerate(rows):
            for j in range(i):
                if abs(rows[j][0] - z) < 1e-6:
                    came_down_i = i == 0 or rows[i - 1][0] > z     # the first point came down from +0.5
                    came_down_j = j == 0 or rows[j - 1][0] > z
                    (play if came_down_i != came_down_j else repeat).append(float(abs(seen[i] - seen[j])))
        self.cal["z"] = {"f_mm": round(fit["f_mm"], 3), "slope": fit["slope"], "camera_height_mm": round(fit["h0_mm"], 2),
                         "straight_within_mm": round(fit["worst_mm"], 3),
                         "play_mm": round(max(play), 3) if play else None,
                         "repeat_mm": round(max(repeat), 3) if repeat else None,
                         "points": [[z, round(s, 5)] for z, s in rows],
                         "date": datetime.datetime.now().isoformat(timespec="seconds"),
                         "note": "1 / (HOME label px per mm) = (camera_height + Z) / f: a scale change -> Z change"}
        save_cal(self.cal)
        self.say(f"Z SWEEP DONE: straight within {fit['worst_mm']:.3f} mm"
                 + ("" if not play else f", play {max(play):.3f} mm") + ("" if not repeat else f", repeat {max(repeat):.3f} mm")
                 + f", camera {fit['h0_mm']:.1f} mm above the labels. Touch heights now follow a Z that slid.")
        self.done = True

    def z_from_above(self, z: float) -> None:
        """Reach z coming down (from 0.5 mm higher): Z has some play, so a height is only repeatable
        when it is always reached from the same side."""
        self.z_to(z + 0.5)
        self.z_to(z)

    def centre_and_scale(self, marker_id: int) -> float | None:
        """Bring a label to the picture centre (same image spot = same lens error), then its px per mm."""
        for _ in range(3):
            got = self.scale_of_id(marker_id)
            if got is None:
                return None
            d_mm = np.linalg.solve(self.J, self.centre_px() - got[1])
            if np.hypot(*d_mm) < 0.1:
                break
            self.move_to(self.clip_to_reach(self.pos + d_mm))
        got = self.scale_of_id(marker_id)
        return None if got is None else float(got[0])

    def job_block(self) -> None:
        """Z steps/mm: a block of known height H with a label on top lies next to a slot label. Block label
        scale at Z0; slot label scale at Z0 and after Z -H. Equal scales = exact steps/mm."""
        height = self.zcfg.get("block_height_mm")
        if height is None:
            self.say("Fill in config/bench.yaml z_cal.block_height_mm (the block height, from calipers), then b")
            return
        height = float(height)
        touch = self.touch_z("paper")
        if touch is None:
            return
        tops = [v for v in (self.touch_z(n, quiet=True) for n in ("part", "place")) if v is not None]
        if touch + 2 * height + 1.0 > 0 or (tops and -height < max(tops) + 1.0):
            self.say(f"Going {height:g} mm down would bring the nozzle within 1 mm of the block (its top is {height:g} mm "
                     f"above the paper touch height {touch:+.1f}) or of the parts / the board: use a thinner block")
            return
        self.z_to(0)
        corners, ids, _ = self.markers.detector.detectMarkers(self.frame())
        ids = [] if ids is None else [int(i) for i in ids.ravel()]
        block = [i for i in ids if i not in self.markers.names]
        bed = [i for i in ids if i in self.markers.names and self.markers.names[i] != "home"]
        if len(block) != 1 or not bed:
            self.say("Put the block (one label on top, not a bed label) next to a slot label, both in view, then b")
            return
        inv = {"block": [], "bed": [], "down": []}
        repeats = int(self.zcfg["block_repeats"])
        for r in range(repeats):
            self.stage(f"Z STEPS [{r + 1}/{repeats}]: block label, then the slot label at Z0 and {height:g} mm lower")
            self.z_from_above(0)
            s_block = self.centre_and_scale(block[0])
            s_bed = self.centre_and_scale(bed[0])
            self.z_from_above(-height)
            got = self.scale_of_id(bed[0])
            self.z_to(0)
            if s_block is None or s_bed is None or got is None:
                self.say("A label was lost from view: nothing measured")
                return
            for k, v in (("block", s_block), ("bed", s_bed), ("down", got[0])):
                inv[k].append(1.0 / float(v))
        setting = self.grbl.axes.steps_setting("Z")
        steps_old = float(self.grbl.settings()[setting])
        alpha, steps_new = steps_from_block(*(1.0 / np.mean(inv[k]) for k in ("block", "bed", "down")), steps_old)
        each = [steps_from_block(1 / b, 1 / e, 1 / w, steps_old)[0] for b, e, w in zip(inv["block"], inv["bed"], inv["down"])]
        spread = float(np.ptp(each) / 2) if len(each) > 1 else None
        self.cal.setdefault("z", {})["steps_check"] = {
            "block_height_mm": height, "real_mm_per_command_mm": round(alpha, 5),
            "spread_between_repeats": None if spread is None else round(spread, 5), "steps_per_mm_now": steps_old,
            "steps_per_mm_exact": round(steps_new, 2), "grbl_setting": setting,
            "date": datetime.datetime.now().isoformat(timespec="seconds")}
        save_cal(self.cal)
        self.say(f"Z STEPS: 1 mm of Z command moves {alpha:.4f} mm"
                 + ("" if spread is None else f" (+-{spread:.4f} between repeats)")
                 + f". {setting} is {steps_old:g}; exact would be "
                 f"{steps_new:.2f}. Not changed: send {setting}={steps_new:.2f} to Grbl to use it")
        self.done = True


# ---------------------------------------------------------------- window

def handle_key(m, key: int, step: float) -> tuple[bool, float]:
    """One key pressed in the window. Returns (quit?, jog step in mm)."""
    if key == -1:
        return False, step
    if key in (ord("q"), 27):
        return True, step
    if key != ord("0"):
        m.new_ref_armed = False                 # 0 must be pressed twice in a row
    jobs = {ord("p"): (m.job_set_spot, "paper"), ord("l"): (m.job_set_spot, "tape"),
            ord("t"): (m.job_touch, "paper"), ord("k"): (m.job_touch, "part"),
            ord("d"): (m.job_touch, "place"), ord("m"): (m.job_touch, "move"),
            ord("["): (m.job_jog_z, -0.1), ord("]"): (m.job_jog_z, 0.1),
            ord("{"): (m.job_jog_z, -1.0), ord("}"): (m.job_jog_z, 1.0),
            ord("c"): (m.job_centre,), ord("a"): (m.job_pump,), ord("0"): (m.job_new_reference,),
            ord("r"): (m.job_real,), ord("n"): (m.job_nozzle_point,), ord("h"): (m.job_start,),
            ord("o"): (m.job_offset,), ord("v"): (m.job_servo,), ord("z"): (m.job_z,), ord("b"): (m.job_block,)}
    if key == ord(" "):
        m.stop_flag.set()
        m.say("Stopping after the current move...")
    elif key in t.STEPS:
        step = t.STEPS[key]
    elif key in ARROWS:                         # arrow keys only: l and k are taken here
        m.submit(m.job_jog, (ARROWS[key][0] * step, ARROWS[key][1] * step))
    elif key in jobs:
        fn, *args = jobs[key]
        m.submit(fn, *args)
    return False, step


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-servo", action="store_true", help="the servo is not wired: marks at one angle, no turning")
    args = ap.parse_args()
    bed = ml.load_bed()
    labels = ml.make_all(bed)
    nominal = {name: np.array(c, float) for name, c, _ in ml.label_placements(bed, labels)}
    out_dir = LOG_DIR / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    cam = cr.open_camera(DEFAULT_CAMERA)           # straightened if config/camera_rectify.yaml exists
    parked = was_parked()                           # before the port opens (that marks it "not parked")
    grbl = Grbl(cfg=load_bench())
    grbl.check_ready()
    m = NozzleCal(grbl, cam, t.Markers(bed), nominal, out_dir, d.load_board(), bed)
    m.feed = float(load_bench()["limits"]["max_feed"])
    m.no_servo = args.no_servo
    m.started_parked = parked
    if not parked:
        m.z_locked = "The last session did not end cleanly: the nozzle may be down"
        m.msg = ("LOCKED: the last session did not end cleanly, the nozzle may be down. Lift it by hand with } "
                 "until it is clear of everything, then press h")
    m.start()
    if not m.z_locked:
        m.submit(m.job_start)
    print(f"Nozzle calibration. Results: {out_dir}", flush=True)
    step, last_ts = 5.0, 0.0
    cv2.namedWindow(WINDOW)
    try:
        while True:
            item = cam.latest()
            if item is not None and item[0] - last_ts > 0.08:
                last_ts, frame = item
                cv2.imshow(WINDOW, m.draw(frame, m.markers.find(frame)))
            stop, step = handle_key(m, cv2.waitKeyEx(15), step)
            if stop:
                break
            try:
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    finally:
        m.stop_flag.set()
        t0 = time.monotonic()
        while m.busy and time.monotonic() - t0 < 60:
            time.sleep(0.05)
        parked = False
        try:
            if m.busy:                      # never talk to Grbl from here while a job may still do it
                print("The machine is still busy: Z NOT lifted. The next start is locked.", flush=True)
            else:
                try:
                    parked = m.park()
                finally:
                    if m.pump_on:
                        print("The pump goes off now: a part on the nozzle drops here", flush=True)
                    try:
                        m.pump(False)
                    except Exception:
                        pass
        finally:
            mark_parked(parked, "nozzle_calibrate")
            grbl.close()
            cam.close()
            cv2.destroyAllWindows()
            print("Bye." if parked else "Bye (not parked: the next start is locked until you lift Z and press h).",
                  flush=True)


if __name__ == "__main__":
    main()
