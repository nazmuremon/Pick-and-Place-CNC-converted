#!/usr/bin/env python3
"""Bed test run: jog to HOME with the live camera, then let the machine find every bed label.

Bench only (Arduino Uno + Grbl, config/bench.yaml: +X = right, +Y = back). There are NO limit
switches: you bring the head to HOME yourself with the keys. Every automatic move is cut into
short steps, and the space key stops it after the current step. Keep a hand near the power switch.

    .venv/bin/python tools/bed_test_run.py
    .venv/bin/python tools/bed_test_run.py --measure-reach   # no reach guard: you drive to every end,
                                                             # the reach is saved on quit

Keys (window "Bed test run"; the picture is turned so it looks as seen from the FRONT):
  arrows      jog: left / right = X, up = to the back (+Y), down = to the front (-Y)
              (or j / l / i / k)
  1 2 3 4     jog step 1 / 5 / 10 / 30 mm
  h           HOME in view: calibrate the camera (small moves to the left and front), measure where
              HOME is in the picture (no move onto it: it may sit just outside the reach), make it X0 Y0
  s           survey: go to every label of config/bed.yaml (planned distances from HOME), centre on
              it and record where it really is. Then back to HOME and check the drift
  g           go back to HOME
  a           automatic test: calibrate on a label in view, find HOME, measure it, survey, come back.
              No label in view (head moved by hand)? It drives to the BACK, where all labels sit,
              in short checked steps until one is in view (config/bench.yaml label_search).
              (or start with --auto: the program then quits by itself at the end)
  space       stop after the current short move
  q / Esc     quit (the head stays where it is)

Results: logs/test_run/<time>/survey.yaml and a picture of every label.
"""
from __future__ import annotations

import argparse
import datetime
import queue
import re
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

import make_labels as ml
from bench import Grabber, Grbl, GrblError, load_bench, mark_parked, was_parked
from smd_identify import DEFAULT_CAMERA

ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "logs" / "test_run"
POSITION_FILE = ROOT / "logs" / "head_position.yaml"   # where the head was last, in mm from HOME
CAMERA_FILE = ROOT / "logs" / "camera_calibration.yaml"   # the last good camera calibration (px per mm)
WINDOW = "Bed test run"
FEED = 300.0                 # mm/min (bench cap)
SETTLE_S = 0.25
CAL_MOVE = 8.0               # mm, the moves that calibrate the camera (longer = more precise)
SAFE_NEAR_HOME = (-15.0, -8.0)   # mm from HOME: well inside the reach, HOME still in view (speed and backlash tests)
CHECK_STEP = 10.0            # mm, the short steps of a camera-checked move (stall check after each)
LEFT, RIGHT, UP, DOWN = {65361, ord("j")}, {65363, ord("l")}, {65362, ord("i")}, {65364, ord("k")}
STEPS = {ord("1"): 1.0, ord("2"): 5.0, ord("3"): 10.0, ord("4"): 30.0}


class Stopped(Exception):
    pass


class Stalled(Exception):
    """The camera saw the head move much less than commanded: end of travel or lost steps."""


class Markers:
    def __init__(self, bed: dict) -> None:
        mk = bed["markers"]
        self.names = {int(mk["home_id"]): "home"}
        if bed["pcb_area"].get("corner_labels", True):
            self.names.update({int(i): f"pcb_{n}" for n, i in enumerate(mk["pcb_corner_ids"], start=1)})
        self.names.update({int(mk["slot_ids"][n - 1]): f"slot_{n}" for n in ml.slot_numbers(bed)})
        self.size_mm = float(mk["size_mm"])
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if mk.get("border_error_rate") is not None:
            params.maxErroneousBitsInBorderRate = float(mk["border_error_rate"])
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, mk["dictionary"]))
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        # uneven light (a lamp from one side, 2026-10-11): local contrast equalisation before the detection
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)) if mk.get("clahe") else None

    def find(self, gray: np.ndarray) -> dict[str, np.ndarray]:
        if self.clahe is not None:
            g = np.asarray(gray)
            g = cv2.cvtColor(g, cv2.COLOR_BGR2GRAY) if g.ndim == 3 else g
            gray = self.clahe.apply(np.clip(g, 0, 255).astype(np.uint8))
        corners, ids, _ = self.detector.detectMarkers(gray)
        out = {}
        if ids is not None:
            for c, i in zip(corners, ids.ravel()):
                if int(i) in self.names:
                    out[self.names[int(i)]] = c.reshape(4, 2)
        return out


class Machine(threading.Thread):
    """Owns the Grbl link. Runs one job at a time: jog, home, survey, go home."""

    def __init__(self, grbl: Grbl, cam: Grabber, markers: Markers, nominal: dict, out_dir: Path) -> None:
        super().__init__(daemon=True)
        self.grbl, self.cam, self.markers, self.nominal, self.out_dir = grbl, cam, markers, nominal, out_dir
        self.max_step = grbl.max_step
        self.feed = FEED
        self.reach = load_bench()["limits"]["reach_from_home"]
        self.reach_stale = bool(load_bench()["limits"].get("reach_stale", False))
        self.by_hand = False                      # True while the owner's own jog runs
        self.extremes: np.ndarray | None = None   # left, right, front, back (mm from HOME) since HOME
        self.auto_done = False
        self.jobs: queue.Queue = queue.Queue()
        self.busy = False
        self.stop_flag = threading.Event()
        self.lock = threading.Lock()
        self.pos = np.zeros(2)            # machine X, Y (mm) since the program started
        self.home: np.ndarray | None = None
        self.J: np.ndarray | None = None  # image px per machine mm (2 x 2)
        self.marker_px_mm: float | None = None   # the 10 mm marker's px per mm at the last calibration (camera height)
        self.J_hint: np.ndarray | None = None      # the saved calibration, corrected for a turned camera
        self.msg = "Jog until the HOME label is in view, then press h"
        self.results: list[dict] = []
        self.frame_shape = (400, 640)

    # ---- plumbing
    def submit(self, fn, *args) -> bool:
        if self.busy:
            return False
        self.busy = True
        self.stop_flag.clear()
        self.jobs.put((fn, args))
        return True

    def run(self) -> None:
        while True:
            fn, args = self.jobs.get()
            try:
                fn(*args)
            except Stopped:
                self.say("Stopped by you.")
            except Stalled as e:
                self.home = None           # the position is not known any more
                self.say(f"STALL: {e}. Position lost: jog until HOME is in view and press h (g and s are locked).")
            except (GrblError, TimeoutError) as e:
                self.say(f"ERROR: {e}")
            except Exception as e:                    # a bug: say it, don't let the worker die silently
                import traceback
                traceback.print_exc()
                self.say(f"ERROR (program bug, see the terminal): {e!r}")
            finally:
                self.busy = False

    def say(self, text: str) -> None:
        with self.lock:
            self.msg = text
        print(text, flush=True)

    # ---- motion
    def move(self, d) -> None:
        d = np.asarray(d, float)
        if not getattr(self, "by_hand", False):
            if getattr(self, "reach_stale", False):
                raise GrblError("the reach was measured before the camera was moved on the head, so the machine does "
                                "not move by itself. Measure it again: tools/bed_test_run.py --measure-reach (arrow "
                                "jogs still work)")
            if getattr(self, "not_parked", None):
                raise GrblError(self.not_parked)
        if self.home is not None and self.reach is not None:
            target = self.pos + d - self.home
            if not self.in_reach(target):
                (x0, x1), (y0, y1) = self.reach["x"], self.reach["y"]
                raise GrblError(f"refused: X{target[0]:+.1f} Y{target[1]:+.1f} from HOME is outside the measured "
                                f"reach (X {x0}..{x1}, Y {y0}..{y1}, config/bench.yaml)")
        n = int(np.ceil(np.max(np.abs(d)) / self.max_step)) if np.max(np.abs(d)) > 1e-4 else 0
        for _ in range(n):
            if self.stop_flag.is_set():
                raise Stopped()
            step = d / n
            self.grbl.move_axes({"X": float(step[0]), "Y": float(step[1])}, self.feed)
            _, p = self.grbl.status()
            with self.lock:
                self.pos = np.array([p["X"], p["Y"]])
            if self.home is not None:
                self.remember_position()
                x, y = self.pos - self.home
                e = self.extremes
                self.extremes = np.array([x, x, y, y]) if e is None else np.array(
                    [min(e[0], x), max(e[1], x), min(e[2], y), max(e[3], y)])

    def move_to(self, target) -> None:
        self.move(np.asarray(target, float) - self.pos)

    # ---- vision
    def look(self, name: str, frames: int = 3) -> np.ndarray | None:
        """The marker's 4 corners (px), averaged over fresh frames taken after the head stopped."""
        found = []
        for f in self.cam.frames_after(time.monotonic() + SETTLE_S, frames):
            self.frame_shape = f.shape
            c = self.markers.find(f).get(name)
            if c is not None:
                found.append(c)
        return np.mean(found, axis=0) if len(found) >= (frames + 1) // 2 else None

    def centre_px(self) -> np.ndarray:
        h, w = self.frame_shape
        return np.array([(w - 1) / 2, (h - 1) / 2])

    def calibrate(self, name: str, tries: int = 2, strict: bool = True) -> bool:
        """Measure the camera (px per mm of head move) on a marker. The marker is first brought near the
        middle of the picture: at the edge the lens bends straight moves (a skewed, wrong calibration).
        strict=False: no check and nothing saved (a rough calibration of the raw, tilted picture, used only
        by tools/camera_rectify.py to plan its moves)."""
        for attempt in range(tries):
            if self._near_centre(name) and self._calibrate_once(name, strict):
                return True
        return False

    def calibration_label(self, seen: dict) -> str | None:
        """The label to calibrate the camera on: a slot label, which can be brought to the picture centre.
        HOME sits just outside the reach: off-centre the lens bends the picture and the calibration was
        rejected (2026-10-07). With only HOME in view, first drive to the nearest slot label (planned layout)."""
        slots = [n for n in seen if n.startswith("slot_")]
        if slots:
            return min(slots, key=lambda n: np.hypot(*(seen[n].mean(axis=0) - self.centre_px())))
        if "home" not in seen:
            return next(iter(seen), None)
        J = next((j for j in (self.J, getattr(self, "J_hint", None), saved_camera_J()) if j is not None), None)
        planned = [n for n in self.nominal if n.startswith("slot_")]
        if J is None or not planned:
            return "home"
        near = min(planned, key=lambda n: np.hypot(*(self.nominal[n] - self.nominal["home"])))
        home_pos = self.pos + np.linalg.solve(J, self.centre_px() - seen["home"].mean(axis=0))
        self.say(f"Only HOME in view: going to {near.upper().replace('_', ' ')} to calibrate the camera there")
        self.move_checked(home_pos + self.nominal[near] - self.nominal["home"] - self.pos)
        seen = self.visible()
        return near if near in seen else ("home" if "home" in seen else None)

    def _near_centre(self, name: str) -> bool:
        """Bring the marker near the picture centre with the last good calibration. Stop a few mm short
        to the right and the back: HOME sits near those ends of the reach. The first move is short: if the
        marker goes the wrong way, the camera was turned since that calibration (re-mounted): allow for it."""
        c = self.look(name)
        if c is None:
            self.say(f"{name} is not in view")
            return False
        J = next((j for j in (self.J, getattr(self, "J_hint", None), saved_camera_J()) if j is not None), None)
        if J is None:
            return True                               # nothing to aim with: calibrate where it is

        def aim(corners):
            d = np.linalg.solve(J, self.centre_px() - corners.mean(axis=0)) - np.array([6.0, 4.0])
            d = np.clip(d, -40, 40)
            return self.clip_to_reach(self.pos + d) - self.pos if self.home is not None else d

        d = aim(c)
        if np.hypot(*d) <= 2:
            return True
        first = d * min(1.0, 3.0 / np.hypot(*d))
        self.move(first)
        c1 = self.look(name)
        if c1 is None:
            self.say(f"{name} left the picture on a 3 mm move: jog until it is in the middle, then try again")
            return False
        expected, seen = J @ first, c1.mean(axis=0) - c.mean(axis=0)
        if seen @ expected < 0.5 * (expected @ expected):
            turn = np.degrees(np.arctan2(seen[1], seen[0]) - np.arctan2(expected[1], expected[0]))
            a = np.radians(turn)
            J = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]) @ J
            self.J_hint = J
            self.say(f"The camera seems turned by {(turn + 180) % 360 - 180:+.0f} deg since its last calibration: "
                     "allowing for it")
        d = aim(c1)
        if np.hypot(*d) > 2:
            self.move(d)
        return self.look(name) is not None

    def _calibrate_once(self, name: str, strict: bool = True) -> bool:
        c0 = self.look(name)
        if c0 is None:
            self.say(f"{name} is not in view")
            return False
        cols = []
        # Move to the LEFT and to the FRONT: HOME sits near the right and back ends of the reach.
        for d in (np.array([-CAL_MOVE, 0.0]), np.array([0.0, -CAL_MOVE])):
            self.move(d)
            c1 = self.look(name)
            self.move(-d)
            if c1 is None:
                self.say(f"lost {name} during the calibration move")
                return False
            cols.append((c1.mean(axis=0) - c0.mean(axis=0)) / -CAL_MOVE)
        J = np.column_stack(cols)
        side = float(np.mean(np.linalg.norm(np.roll(c0, -1, axis=0) - c0, axis=1)))
        sx, sy = np.linalg.norm(J[:, 0]), np.linalg.norm(J[:, 1])
        skew = np.degrees(np.arccos(abs(J[:, 0] @ J[:, 1]) / (sx * sy)))
        marker = side / self.markers.size_mm
        self.say(f"Camera calibrated: {sx:.2f} px/mm along X, {sy:.2f} px/mm along Y "
                 f"(the 10 mm marker says {marker:.2f}); angle X-Y {skew:.1f} deg")
        if not strict:
            self.J = J
            return True
        # A head that hit an end (lost steps) gives a crooked or wrongly scaled result: refuse it.
        chk = load_bench().get("camera_check", {})
        max_deg, max_scale = float(chk.get("max_axis_angle_error_deg", 2.0)), float(chk.get("max_scale_error", 0.05))
        if abs(skew - 90) > max_deg or abs(sx / marker - 1) > max_scale or abs(sy / marker - 1) > max_scale:
            self.say(f"Calibration REJECTED (axes not at 90 +-{max_deg:g} deg, or scale more than {max_scale * 100:g} % "
                     "from the marker). "
                     "Trying again with the marker in the middle... (A tilted camera is always refused: "
                     "straighten its pictures with tools/camera_rectify.py)")
            return False
        self.J = J
        self.marker_px_mm = marker
        CAMERA_FILE.write_text(yaml.safe_dump({"J": J.round(4).tolist(),
                                               "time": datetime.datetime.now().isoformat(timespec="seconds")}))
        return True

    def locate(self, name: str, tol_mm: float = 0.05, tries: int = 6) -> np.ndarray | None:
        """Machine position of a marker centre. Centre on it as far as the reach allows, then measure
        the rest in the picture (a marker may sit just outside the reach, like HOME). None = not seen."""
        c = self.look(name)
        for _ in range(tries):
            if c is None:
                return None
            d = np.linalg.solve(self.J, self.centre_px() - c.mean(axis=0))
            if np.hypot(*d) < tol_mm:
                break
            d = d if np.hypot(*d) < 15 else d * 15 / np.hypot(*d)
            d = self.clip_to_reach(self.pos + d) - self.pos
            if np.hypot(*d) < 0.02:
                break                                 # as close as the reach allows
            self.move(d)
            c_new = self.look(name)
            expected = self.J @ d                     # how far the marker should move in the picture
            if c_new is not None and np.hypot(*expected) > 3:
                seen = c_new.mean(axis=0) - c.mean(axis=0)
                if seen @ expected < 0.5 * (expected @ expected):
                    raise Stalled(f"commanded X{d[0]:+.1f} Y{d[1]:+.1f} mm but {name} moved only "
                                  f"{np.hypot(*seen) / np.hypot(*expected) * 100:.0f} % of that in the picture")
            c = c_new
        c = self.look(name, frames=6)
        return None if c is None else self.pos + np.linalg.solve(self.J, self.centre_px() - c.mean(axis=0))

    def snapshot(self, name: str) -> None:
        f = self.cam.frames_after(time.monotonic(), 1)[0]
        cv2.imwrite(str(self.out_dir / f"{name}.png"), cv2.rotate(f, cv2.ROTATE_180) if self.rotate180() else f)

    def rotate180(self) -> bool:
        # The raw picture is already seen from the front (checked on the slot labels 2026-10-06). After
        # calibration this comes from the measurement: turn it if a marker moves right when the head does.
        return False if self.J is None else bool(self.J[0, 0] > 0)

    # ---- jobs
    def job_jog(self, d) -> None:
        self.by_hand = True                       # the owner moves the head, watching it
        try:
            self.move(d)
        finally:
            self.by_hand = False
        rel = self.pos - (self.home if self.home is not None else 0)
        self.say(f"Jogged to X{rel[0]:+.1f} Y{rel[1]:+.1f}" + (" from HOME" if self.home is not None else " from start"))

    def job_home(self) -> None:
        """Measure where HOME is from the picture. No move onto it: it may sit just outside the reach."""
        if self.J is None and not self.calibrate("home"):
            return
        c = self.look("home", frames=6)
        if c is None:
            self.say("HOME is not in view")
            return
        offset = np.linalg.solve(self.J, self.centre_px() - c.mean(axis=0))
        self.home = self.pos + offset
        self.extremes = None
        self.snapshot("home")
        self.say(f"HOME measured ({offset[0]:+.1f}, {offset[1]:+.1f} mm from the camera centre): it is now X0 Y0. "
                 "Press s to survey all labels.")

    def remember_position(self) -> None:
        rel = self.pos - self.home
        POSITION_FILE.write_text(yaml.safe_dump({
            "head_from_home_mm": [round(float(v), 3) for v in rel],
            "time": datetime.datetime.now().isoformat(timespec="seconds"),
            "note": "written after every bench move. For information only: the tools find HOME with the camera"}))

    def search_for_labels(self) -> dict[str, np.ndarray]:
        """No label in view, and where the head is may be unknown (moved by hand, PC restarted). All the
        labels sit in one row at the BACK of the reach (HOME and the slot markers, across its whole
        width), so drive to the back in short steps until one is in view. After every step the picture
        must have moved as far as the head should: if not, the head is blocked (an end), so stop.
        Settings: config/bench.yaml label_search. Returns the labels in view ({} = none found)."""
        cfg = load_bench()["label_search"]
        J = self.J if self.J is not None else saved_camera_J()
        if J is None:
            self.say("No label in view and no camera calibration yet: jog until a label is in view")
            return {}
        step = np.array([0.0, float(cfg["step_mm"])])
        span = self.reach["y"][1] - self.reach["y"][0] if self.reach is not None else 230.0
        self.say("No label in view: driving to the BACK, where the labels are, in short checked steps...")
        feed, self.feed = self.feed, min(self.feed, float(cfg["feed"]))
        unsure = 0
        try:
            before = self.picture()
            for _ in range(int(np.ceil((span + 10) / step[1]))):
                self.move(step)
                seen = self.visible()
                if seen:
                    self.say(f"Found {', '.join(n.upper().replace('_', ' ') for n in seen)}")
                    return seen
                after = self.picture()
                moved = moved_as_commanded(before, after, J @ step, float(cfg["margin"]))
                if moved is not True:
                    turn = camera_turn(before, after, J @ step)
                    if turn is not None and abs(turn) > 20:   # moved the right distance, but another way:
                        a = np.radians(turn)
                        J = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]) @ J
                        self.say(f"The camera seems turned by {turn:+.0f} deg since its last calibration: the "
                                 "search allows for it (the camera is calibrated again on the first label)")
                        self.J_hint = J
                        moved = True
                if moved is False:
                    raise Stalled("the picture did not move with the head: it is blocked (at the back end?)")
                unsure = unsure + 1 if moved is None else 0
                if unsure >= int(cfg["max_unsure_steps"]):
                    self.say(f"The picture cannot show the movement here ({unsure} steps: a plain area or bars "
                             "along the move). Stopped to be safe: jog until a label is in view")
                    return {}
                before = after
        finally:
            self.feed = feed
        self.say("No label found at the back of the bed: jog until a label is in view")
        return {}

    def in_reach(self, rel: np.ndarray, tol: float = 0.01) -> bool:
        """rel = mm from HOME. The tolerance keeps a head parked on the edge inside: Grbl reports positions to
        0.001 mm (one step is 0.0025 mm), so a head clipped onto the edge reads a hair outside it, and every
        later jog along that edge was refused (2026-10-07)."""
        (x0, x1), (y0, y1) = self.reach["x"], self.reach["y"]
        return x0 - tol <= rel[0] <= x1 + tol and y0 - tol <= rel[1] <= y1 + tol

    def clip_to_reach(self, target: np.ndarray) -> np.ndarray:
        """The closest point to target (machine mm) that is inside the reach. HOME may sit just outside."""
        if self.home is None or self.reach is None:
            return target
        (x0, x1), (y0, y1) = self.reach["x"], self.reach["y"]
        rel = target - self.home
        return self.home + np.array([np.clip(rel[0], x0, x1), np.clip(rel[1], y0, y1)])

    def home_drift(self) -> np.ndarray | None:
        """Where HOME is now, minus where it was measured (mm). None if HOME is not in view."""
        c = self.look("home", frames=6)
        if c is None:
            return None
        return self.pos + np.linalg.solve(self.J, self.centre_px() - c.mean(axis=0)) - self.home

    def job_go_home(self) -> None:
        if self.home is None:
            self.say("No HOME yet: press h with HOME in view")
            return
        self.move_to(self.clip_to_reach(self.home))
        self.say("Back at HOME (as close as the reach allows)")

    def search(self, name: str, target: np.ndarray) -> bool:
        for off in ([0, 15], [0, -15], [20, 0], [-20, 0], [0, 30], [0, -30]):
            spot = target + np.array(off, float)
            if not np.allclose(self.clip_to_reach(spot), spot):
                continue                              # outside the reach: skip, don't stop the survey
            self.move_to(spot)
            if self.look(name) is not None:
                return True
        return False

    def plan_order(self) -> list[str]:
        left = [n for n in self.nominal if n != "home"]
        order, here = [], self.nominal["home"]
        while left:
            nxt = min(left, key=lambda n: np.hypot(*(self.nominal[n] - here)))
            order.append(nxt)
            left.remove(nxt)
            here = self.nominal[nxt]
        return order

    def visible(self) -> dict[str, np.ndarray]:
        return self.markers.find(self.picture())

    def picture(self) -> np.ndarray:
        """A fresh camera picture, taken after the head has settled."""
        f = self.cam.frames_after(time.monotonic() + SETTLE_S, 1)[0]
        self.frame_shape = f.shape
        return f

    def move_checked(self, d) -> None:
        """Move in short steps; after each one, every marker seen before and after must have moved as
        the calibration predicts. Otherwise the head is stuck (end of travel): stop."""
        d = np.asarray(d, float)
        n = int(np.ceil(np.max(np.abs(d)) / CHECK_STEP)) if np.max(np.abs(d)) > 1e-4 else 0
        for _ in range(n):
            before = self.visible()
            self.move(d / n)
            if self.J is None:
                continue                              # no calibration yet: nothing to compare with
            after = self.visible()
            expected = self.J @ (d / n)
            for name in set(before) & set(after):
                seen = after[name].mean(axis=0) - before[name].mean(axis=0)
                if seen @ expected < 0.5 * (expected @ expected):
                    raise Stalled(f"{name} moved only {np.hypot(*seen) / np.hypot(*expected) * 100:.0f} % of the "
                                  "commanded step in the picture")

    def job_auto(self) -> None:
        """The whole test without the keys: calibrate, find HOME, measure it, survey, come back."""
        seen = self.visible() or self.search_for_labels()
        if not seen:
            self.say("AUTO: no label in view. Jog until a slot label or HOME is in the picture, then press a.")
            return
        ref = self.calibration_label(seen)
        if ref is None:
            self.say("AUTO: the labels went out of view. Jog until a slot label is in the picture, then press a.")
            return
        self.say(f"AUTO: calibrating the camera on {ref}")
        if self.J is None and not self.calibrate(ref):
            return
        if ref != "home":
            c = self.look(ref)
            ref_pos = self.pos + np.linalg.solve(self.J, self.centre_px() - c.mean(axis=0))
            home_guess = ref_pos + self.nominal["home"] - self.nominal[ref]
            target = home_guess - np.array([15.0, 0.0])   # stop short: HOME sits just outside the reach
            self.say(f"AUTO: driving to HOME (planned {home_guess[0] - self.pos[0]:+.0f} mm X, "
                     f"{home_guess[1] - self.pos[1]:+.0f} mm Y), checking every step with the camera")
            self.move_checked(target - self.pos)
        self.job_home()
        if self.home is None:
            self.say("AUTO stopped: HOME was not found where the plan says")
            return
        self.job_survey()
        self.say("AUTO DONE")
        self.auto_done = True

    def job_survey(self) -> None:
        if self.home is None:
            self.say("No HOME yet: press h with HOME in view")
            return
        self.results = []
        order = self.plan_order()
        for k, name in enumerate(order, start=1):
            plan = self.nominal[name] - self.nominal["home"]
            self.say(f"[{k}/{len(order)}] {name}: going to the plan position X{plan[0]:+.1f} Y{plan[1]:+.1f}")
            target = self.clip_to_reach(self.home + plan)
            self.move_to(target)
            seen = self.look(name) is not None or self.search(name, target)
            found = self.locate(name) if seen else None
            if found is not None:
                meas = found - self.home
                self.results.append({"label": name, "plan_mm": plan.round(2).tolist(),
                                     "measured_mm": meas.round(2).tolist()})
                self.snapshot(name)
                off = meas - plan
                self.say(f"   {name}: found at X{meas[0]:+.2f} Y{meas[1]:+.2f} (plan off by X{off[0]:+.1f} Y{off[1]:+.1f})")
            else:
                self.results.append({"label": name, "plan_mm": plan.round(2).tolist(), "measured_mm": None})
                self.say(f"   {name}: NOT FOUND")
        self.say("Survey done. Going back to HOME to check the drift...")
        self.move_to(self.clip_to_reach(self.home))
        drift = self.home_drift()
        self.save(drift)
        self.say("Survey saved. " + ("HOME not seen at the end!" if drift is None else
                                     f"HOME drift after the trip: X{drift[0]:+.2f} Y{drift[1]:+.2f} mm"))

    def save(self, drift) -> None:
        found = {r["label"]: np.array(r["measured_mm"]) for r in self.results if r["measured_mm"] is not None}
        summary = {}
        slots = [found.get(f"slot_{i}") for i in range(1, 8)]
        pitches = [float(np.hypot(*(b - a))) for a, b in zip(slots, slots[1:]) if a is not None and b is not None]
        if pitches:
            summary["slot_pitch_mm"] = [round(p, 2) for p in pitches]
        if all(f"pcb_{i}" in found for i in range(1, 5)):
            p = [found[f"pcb_{i}"] for i in range(1, 5)]
            summary["pcb_corner_distances_mm"] = {
                "1-2 (front, left-right)": round(float(np.hypot(*(p[1] - p[0]))), 2),
                "4-3 (back, left-right)": round(float(np.hypot(*(p[2] - p[3]))), 2),
                "1-4 (left, front-back)": round(float(np.hypot(*(p[3] - p[0]))), 2),
                "2-3 (right, front-back)": round(float(np.hypot(*(p[2] - p[1]))), 2),
                "diagonal 1-3": round(float(np.hypot(*(p[2] - p[0]))), 2),
                "diagonal 2-4": round(float(np.hypot(*(p[3] - p[1]))), 2),
            }
        data = {
            "date": datetime.datetime.now().isoformat(timespec="seconds"),
            "note": "Machine X/Y (mm) of every marker centre, measured from HOME. +X = right, +Y = back.",
            "camera_px_per_mm_matrix": self.J.round(3).tolist(),
            "home_drift_after_survey_mm": None if drift is None else np.round(drift, 3).tolist(),
            "labels": self.results,
            "summary": summary,
        }
        (self.out_dir / "survey.yaml").write_text(yaml.safe_dump(data, sort_keys=False))
        print(yaml.safe_dump({"summary": summary}, sort_keys=False), flush=True)


def saved_camera_J() -> np.ndarray | None:
    """The last good camera calibration (px per mm of head move), or None."""
    try:
        return np.array(yaml.safe_load(CAMERA_FILE.read_text())["J"], float)
    except (OSError, KeyError, TypeError, ValueError):
        return None


def camera_turn(a: np.ndarray, b: np.ndarray, expected_px) -> float | None:
    """The picture moved about as far as expected but in another direction: the camera was turned (re-mounted)
    since its calibration. Returns that turn in degrees, or None (it did not move, or the shift is unclear)."""
    a, b = np.float32(a), np.float32(b)
    (dx, dy), sure = cv2.phaseCorrelate(a, b, cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F))
    exp = np.asarray(expected_px, float)
    got = np.array([dx, dy])
    if sure < 0.1 or not 0.6 <= np.hypot(*got) / np.hypot(*exp) <= 1.6:
        return None
    turn = np.degrees(np.arctan2(got[1], got[0]) - np.arctan2(exp[1], exp[0]))
    return float((turn + 180) % 360 - 180)


def moved_as_commanded(a: np.ndarray, b: np.ndarray, expected_px, margin: float) -> bool | None:
    """Picture a before a move, b after it; expected_px = how far the picture should shift (u, v).
    True: b looks like a shifted by that (the head moved). False: b looks like a unshifted (it did
    not move: blocked). None: the picture cannot tell, both fit as well (a plain area, or long bars
    running in the direction of the move). Compared as correlation (-1..1); margin = how much better
    one must fit."""
    a = cv2.GaussianBlur(np.float32(a), (0, 0), 2)
    b = cv2.GaussianBlur(np.float32(b), (0, 0), 2)
    h, w = a.shape
    dx, dy = (int(round(v)) for v in expected_px)
    ys, xs = slice(max(0, -dy), min(h, h - dy)), slice(max(0, -dx), min(w, w - dx))
    ref = a[ys, xs]
    if ref.size < 1000 or ref.std() < 2:
        return None
    shifted = b[ys.start + dy:ys.stop + dy, xs.start + dx:xs.stop + dx]
    still = b[ys, xs]

    def corr(x, y):
        return float(np.corrcoef(x.ravel(), y.ravel())[0, 1]) if y.std() > 1e-6 else 0.0

    c_moved, c_still = corr(ref, shifted), corr(ref, still)
    if c_moved > c_still + margin:
        return True
    if c_still > c_moved + margin:
        return False
    return None


PANEL_FONT, PANEL_LINE, PANEL_ROWS = 0.75, 34, 6


def status_panel(view: np.ndarray, lines: list[tuple[str, tuple]]) -> np.ndarray:
    """A black panel ABOVE the camera picture with large text (it never covers the picture).
    lines = [(text, BGR colour)]; long lines are wrapped to the window width."""
    import textwrap
    w = view.shape[1]
    (tw, _), _ = cv2.getTextSize("abcdefghijklmnopqrstuvwxyz0123456789", cv2.FONT_HERSHEY_SIMPLEX, PANEL_FONT, 2)
    per_line = max(20, int((w - 20) / (tw / 36)))
    rows = [(part, colour) for text, colour in lines for part in (textwrap.wrap(text, per_line) or [""])]
    panel = np.zeros((PANEL_LINE * max(PANEL_ROWS, len(rows)) + 12, w, 3), np.uint8)   # fixed height: no jumping
    for i, (text, colour) in enumerate(rows):
        cv2.putText(panel, text, (10, 28 + PANEL_LINE * i), cv2.FONT_HERSHEY_SIMPLEX, PANEL_FONT, colour, 2, cv2.LINE_AA)
    return np.vstack([panel, view])


def label_on_picture(view: np.ndarray, text: str, xy, colour=(255, 255, 255)) -> None:
    """A name on the camera picture: bright text on a black box, readable on any background."""
    ml._text(view, text, xy, 0.7, colour, 2, centred=True, box=True, bg=(0, 0, 0))


def draw(frame: np.ndarray, found: dict, machine: Machine, step: float) -> np.ndarray:
    rot = machine.rotate180()
    view = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    h, w = frame.shape
    if rot:
        view = cv2.rotate(view, cv2.ROTATE_180)
    scale = 1.5
    view = cv2.resize(view, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)

    def p(pt):
        u, v = pt
        if rot:
            u, v = w - 1 - u, h - 1 - v
        return int(u * scale), int(v * scale)

    for name, c in found.items():
        pts = np.array([p(q) for q in c], np.int32)
        cv2.polylines(view, [pts], True, (0, 220, 0) if name == "home" else (255, 180, 0), 2)
        label_on_picture(view, name.upper().replace("_", " "), pts.mean(axis=0) + (0, -45),
                         (0, 255, 0) if name == "home" else (255, 220, 120))
    cx, cy = p(((w - 1) / 2, (h - 1) / 2))
    cv2.drawMarker(view, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 40, 1)
    with machine.lock:
        msg, pos = machine.msg, machine.pos.copy()
    rel = pos - (machine.home if machine.home is not None else 0)
    where = "from HOME" if machine.home is not None else "from start"
    return status_panel(view, [
        (f"X{rel[0]:+.2f}  Y{rel[1]:+.2f} mm {where}   step {step:g} mm   {'BUSY' if machine.busy else 'ready'}",
         (0, 255, 255)),
        (msg, (255, 255, 255)),
        ("arrows: jog   1-4: step   h: HOME   s: survey   a: auto   g: go HOME   space: stop   q: quit",
         (170, 170, 170))])


def save_reach(extremes: np.ndarray) -> None:
    """Write the measured reach into config/bench.yaml (keeps the comments)."""
    path = ROOT / "config" / "bench.yaml"
    left, right, front, back = (float(round(v, 1)) for v in extremes)
    text = path.read_text()
    text = re.sub(r"(reach_from_home:\n\s+x: )\[[^\]]*\]", rf"\g<1>[{left:g}, {right:g}]", text)
    text = re.sub(r"(reach_from_home:\n\s+x: \[[^\]]*\]\n\s+y: )\[[^\]]*\]", rf"\g<1>[{front:g}, {back:g}]", text)
    stamp = datetime.date.today().isoformat()
    text = text.replace("  # ESTIMATE until the new HOME position is measured.", f"  # Measured with --measure-reach on {stamp}.")
    text = re.sub(r"(?m)^(  reach_stale: )true\b", r"\g<1>false", text)
    path.write_text(text)
    print(f"Saved the reach to config/bench.yaml: X {left:g}..{right:g}, Y {front:g}..{back:g} mm from HOME",
          flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--auto", action="store_true",
                    help="run the whole test by itself (a slot label or HOME must be in view); quit at the end")
    ap.add_argument("--measure-reach", action="store_true",
                    help="no reach guard; after h, drive to every end (stop ~5 mm before it); saved on quit")
    args = ap.parse_args()
    bed = ml.load_bed()
    labels = ml.make_all(bed)
    nominal = {name: np.array(c, float) for name, c, _ in ml.label_placements(bed, labels)}
    markers = Markers(bed)
    out_dir = LOG_DIR / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)

    import camera_rectify as cr                     # (it imports this module)
    cam = cr.open_camera(DEFAULT_CAMERA)           # straightened if config/camera_rectify.yaml exists
    parked = was_parked()                           # before the port opens (that marks it "not parked")
    grbl = Grbl(cfg=load_bench())
    grbl.check_ready()
    machine = Machine(grbl, cam, markers, nominal, out_dir)
    if not parked and load_bench()["nozzle"].get("mounted"):
        machine.not_parked = ("the last program did not leave the nozzle up (it was stopped, or it moves Z): nothing "
                              "moves by itself. Start tools/nozzle_calibrate.py, lift the nozzle, press h, quit with q")
    if args.measure_reach:
        machine.reach = None
        machine.reach_stale = False                 # the owner drives and watches: h may calibrate (small moves)
        machine.msg = "MEASURE REACH (no guard!): press h on HOME, then drive to each end, stop ~5 mm before it"
    machine.start()
    if args.auto:
        machine.submit(machine.job_auto)
    print(f"Bed test run. Results go to {out_dir}", flush=True)

    step, last_ts = 5.0, 0.0
    auto_idle_since = time.monotonic()
    cv2.namedWindow(WINDOW)
    try:
        while True:
            item = cam.latest()
            if item is not None and item[0] - last_ts > 0.08:
                last_ts, frame = item
                cv2.imshow(WINDOW, draw(frame, markers.find(frame), machine, step))
            key = cv2.waitKeyEx(15)
            if key in (ord("q"), 27):
                break
            if key == ord(" "):
                machine.stop_flag.set()
                machine.say("Stopping after the current short move...")
            elif key in STEPS:
                step = STEPS[key]
            elif key in LEFT | RIGHT | UP | DOWN:
                d = {**{k: (-step, 0) for k in LEFT}, **{k: (step, 0) for k in RIGHT},
                     **{k: (0, step) for k in UP}, **{k: (0, -step) for k in DOWN}}[key]
                machine.submit(machine.job_jog, d)
            elif key == ord("h"):
                machine.submit(machine.job_home)
            elif key == ord("s"):
                machine.submit(machine.job_survey)
            elif key == ord("g"):
                machine.submit(machine.job_go_home)
            elif key == ord("a"):
                machine.submit(machine.job_auto)
            if args.auto and not machine.busy and (machine.auto_done or machine.jobs.empty()):
                if machine.auto_done or time.monotonic() - auto_idle_since > 3:
                    break
            elif args.auto:
                auto_idle_since = time.monotonic()
            try:
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    finally:
        machine.stop_flag.set()
        t = time.monotonic()
        while machine.busy and time.monotonic() - t < 30:
            time.sleep(0.05)
        grbl.close()
        if not machine.busy:
            mark_parked(parked, "bed_test_run (no Z moves)")     # Z was not moved: as it was before
        cam.close()
        cv2.destroyAllWindows()
        if args.measure_reach:
            e = machine.extremes
            if e is not None and e[1] - e[0] > 20 and e[3] - e[2] > 20:
                save_reach(e)
            else:
                print("Reach NOT saved: press h on HOME first, then drive to all 4 ends.", flush=True)
        print("Bye.", flush=True)


if __name__ == "__main__":
    main()
