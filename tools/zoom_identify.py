#!/usr/bin/env python3
"""Zoom in with Z and identify SMD parts live.

The camera rides on Z. Going down brings it closer to the sheet: more px/mm and bigger parts
(this lens has fixed focus, so the image may also get sharper or blurrier). The tool steps Z
down while the live window shows every part with its package name. At the end it goes to the
height that identified the most parts and holds there.

Settings: config/bench.yaml (axis map, limits, zoom) and config/smd_identify.yaml (sheet,
packages). The sheet markers give the scale, so zoom.min_markers of them must stay in view.
With fewer, part sizes far from the marker are wrong: the auto zoom stops going down and goes
back to the last height that still saw enough markers.

There are NO limit switches: Z never goes more than zoom.max_down_mm below the start height,
or more than zoom.max_up_mm above it. On quit, Z goes back to the start height.

    .venv/bin/python tools/zoom_identify.py
    .venv/bin/python tools/zoom_identify.py --no-window     (auto zoom, save pictures, go back, exit)

Keys:  space  pause / continue the auto zoom      a  run the auto zoom again
       s / w  Z down / up one step (manual)       p  save a picture
       + / -  detection threshold                 q / Esc  back to the start height and quit
"""
from __future__ import annotations

import argparse
import csv
import datetime
import queue
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import yaml

import smd_identify as si
from bench import Grabber, Grbl, GrblError, load_bench, sharpness
from pnp_paths import LOGS_DIR

ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = LOGS_DIR / "zoom_identify"
STATE_FILE = LOGS_DIR / "bench_state.yaml"
WINDOW = "Zoom identify"
SETTLE_S = 0.3


def load_z_offset() -> float:
    """Z (mm) below/above the start height when the last run ended. 0 after a clean exit."""
    try:
        return float(yaml.safe_load(STATE_FILE.read_text()).get("z_from_start", 0.0))
    except (OSError, AttributeError, TypeError, ValueError):
        return 0.0


def save_z_offset(z: float) -> None:
    STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    STATE_FILE.write_text(f"# Written by tools/zoom_identify.py after every Z move.\nz_from_start: {z:.3f}\n")


class ZWorker(threading.Thread):
    """Owns the Grbl link. Moves Z to targets (mm from the start height), one at a time."""

    def __init__(self, grbl: Grbl, feed: float, low: float, high: float, z: float) -> None:
        super().__init__(daemon=True)
        self.grbl, self.feed, self.low, self.high = grbl, feed, low, high
        self.z = z
        self.target = z
        self.q: queue.Queue[float | None] = queue.Queue()
        self.busy = False
        self.moves_done = 0
        self.error: str | None = None

    def goto(self, z: float) -> bool:
        if not self.low - 1e-6 <= z <= self.high + 1e-6:
            print(f"Refused: Z {z:+.1f} is outside {self.low:+.1f} .. {self.high:+.1f} mm", flush=True)
            return False
        self.busy = True
        self.target = z
        self.q.put(z)
        return True

    def finish(self) -> None:
        self.q.put(None)

    def run(self) -> None:
        while True:
            z = self.q.get()
            if z is None:
                return
            try:
                while abs(z - self.z) > 1e-6 and self.error is None:
                    step = float(np.clip(z - self.z, -self.grbl.max_step, self.grbl.max_step))
                    self.grbl.move_relative("Z", step, self.feed)
                    self.z += step
                    save_z_offset(self.z)
            except (GrblError, TimeoutError) as e:
                self.error = str(e)
                print(f"Z move failed: {e}", flush=True)
            finally:
                if self.q.empty():
                    self.moves_done += 1
                    self.busy = False


@dataclass
class Level:
    z: float
    frames: int = 0
    found: int = 0
    sharp: list[float] = field(default_factory=list)
    px_per_mm: list[float] = field(default_factory=list)
    names: list[str] = field(default_factory=list)
    view: np.ndarray | None = None

    @property
    def found_ratio(self) -> float:
        return self.found / self.frames if self.frames else 0.0

    @property
    def named(self) -> int:
        return sum(n != "unknown" for n in self.names)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default=si.DEFAULT_CAMERA)
    ap.add_argument("--port", help="serial port (default: config/bench.yaml, else the first USB port)")
    ap.add_argument("--no-window", action="store_true", help="auto zoom only, save pictures, go back, exit")
    args = ap.parse_args()

    bench = load_bench()
    if bench.get("nozzle", {}).get("mounted"):
        raise SystemExit("A nozzle is on the head (config/bench.yaml nozzle.mounted): this zoom test goes down to "
                         "zoom.max_down_mm and would drive it into the bed. Not started. (Set mounted: false only "
                         "with the nozzle taken off.)")
    zc = bench["zoom"]
    step, dwell, feed = float(zc["step_mm"]), float(zc["dwell_s"]), float(zc["feed"])
    min_markers, frame_period = int(zc["min_markers"]), 1.0 / float(zc["max_fps"])
    cv2.setNumThreads(2)
    low, high = -float(zc["max_down_mm"]), float(zc["max_up_mm"])
    levels = [-step * i for i in range(int(round(-low / step)) + 1)]

    cfg, packages = si.load_config(si.DEFAULT_CONFIG)
    ident = si.Identifier(cfg, packages, outside_mm=float(zc["outside_mm"]))
    out_dir = LOG_DIR / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)

    z0 = load_z_offset()
    if abs(z0) > 1e-3:
        print(f"NOTE: the last run ended with Z {z0:+.2f} mm from its start height. "
              "Counting from there, so 'back to start' goes to that old start height.", flush=True)

    cam = Grabber(args.camera)
    grbl = Grbl(args.port)
    grbl.check_ready()
    worker = ZWorker(grbl, feed, low, high, z0)
    worker.start()
    print(f"Zoom levels (mm from start): {levels}. Dwell {dwell} s. Output: {out_dir}", flush=True)

    auto = True
    paused = False
    level_i = 0
    stats: list[Level] = []
    current: Level | None = None
    arrived_at: float | None = None
    seen_moves = -1
    start_px: float | None = None
    last_ts = 0.0
    status = ""
    quitting = False
    worker.goto(levels[0])

    def finish_auto(reason: str) -> None:
        nonlocal auto, status
        auto = False
        good = [lv for lv in stats if lv.found_ratio >= 0.5]
        print(f"Auto zoom done ({reason}).", flush=True)
        if good:
            best = max(good, key=lambda lv: (lv.named, np.mean(lv.sharp)))
            print(f"Best height: Z {best.z:+.1f} mm, {best.named} parts named, "
                  f"{np.mean(best.px_per_mm):.1f} px/mm. Going there.", flush=True)
            worker.goto(best.z)
            status = f"HOLD at best height Z {best.z:+.1f}"
        with (out_dir / "levels.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["z_mm", "px_per_mm", "sharpness", "markers_found_pct", "parts"])
            for lv in stats:
                w.writerow([f"{lv.z:+.1f}", f"{np.mean(lv.px_per_mm) if lv.px_per_mm else 0:.2f}",
                            f"{np.mean(lv.sharp):.1f}", f"{100 * lv.found_ratio:.0f}", "; ".join(lv.names)])

    try:
        while True:
            if worker.error:
                print("Stopping: Z worker error.", flush=True)
                break
            item = cam.latest()
            if item is None or item[0] - last_ts < frame_period:
                time.sleep(0.01)
                key = cv2.waitKey(1) & 0xFF if not args.no_window else 255
            else:
                last_ts, gray = item
                if worker.moves_done != seen_moves and not worker.busy:
                    seen_moves = worker.moves_done
                    ident.average = None   # the view changed: don't mix in the old frames
                    arrived_at = time.monotonic()
                res = ident.process(gray)
                sharp = sharpness(gray)
                n_markers = len(res.marker_corners) if res.found else 0
                scale_ok = n_markers >= min_markers
                if scale_ok and start_px is None:
                    start_px = res.camera_px_per_mm
                view = si.draw(gray, res, cfg, 2.0, ident.det["threshold"])

                settled = not worker.busy and arrived_at is not None and time.monotonic() - arrived_at > SETTLE_S
                if auto and not paused and settled:
                    if current is None or current.z != worker.z:
                        current = Level(worker.z)
                    current.frames += 1
                    current.sharp.append(sharp)
                    if scale_ok:
                        current.found += 1
                        current.px_per_mm.append(res.camera_px_per_mm)
                        current.names = [p.match.name if p.match else "unknown" for p in res.parts]
                    current.view = view
                    if time.monotonic() - arrived_at > SETTLE_S + dwell:
                        lv, current = current, None
                        stats.append(lv)
                        px = f"{np.mean(lv.px_per_mm):.2f} px/mm" if lv.px_per_mm else f"< {min_markers} markers"
                        print(f"Z {lv.z:+5.1f} mm: {px}, sharpness {np.mean(lv.sharp):5.1f}, "
                              f"parts: {', '.join(lv.names) or 'none'}", flush=True)
                        cv2.imwrite(str(out_dir / f"z{lv.z:+05.1f}.png"), lv.view)
                        if lv.found_ratio < 0.5:
                            finish_auto(f"fewer than {min_markers} markers in view: the scale is not reliable")
                        elif level_i + 1 >= len(levels):
                            finish_auto("lowest level reached")
                        else:
                            level_i += 1
                            worker.goto(levels[level_i])

                if worker.busy:
                    state = f"MOVING to Z {worker.target:+.1f}"
                elif auto:
                    state = "PAUSED" if paused else f"AUTO ZOOM level {level_i + 1}/{len(levels)}"
                else:
                    state = status or "MANUAL"
                zoom = f"zoom {res.camera_px_per_mm / start_px:.2f}x" if scale_ok and start_px else "zoom ?"
                if not scale_ok:
                    si.put_text(view, f"Only {n_markers} marker(s) in view (need {min_markers}): "
                                "sizes and names are NOT reliable", (10, view.shape[0] - 66), (60, 60, 255), 0.6)
                si.put_text(view, f"Z {worker.z:+.1f} mm from start | {zoom} | markers {n_markers} | "
                            f"sharpness {sharp:.1f} | {state}", (10, view.shape[0] - 40), (0, 255, 255), 0.6)
                si.put_text(view, "space pause  a auto  s/w down/up  p save  +/- threshold  q back+quit",
                            (10, view.shape[0] - 14), (200, 200, 200), 0.5)

                if args.no_window:
                    key = 255
                    if not auto and not worker.busy:
                        break
                else:
                    cv2.imshow(WINDOW, view)
                    key = cv2.waitKey(1) & 0xFF
                    if not si.window_open(WINDOW):
                        break

            if key in (ord("q"), 27):
                break
            if key == ord(" ") and auto:
                paused = not paused
            elif key == ord("a") and not worker.busy:
                auto, paused, level_i, stats, current = True, False, 0, [], None
                worker.goto(levels[0])
            elif key in (ord("s"), ord("w")) and not worker.busy:
                auto = False
                status = "MANUAL"
                worker.goto(round(worker.z + (-step if key == ord("s") else step), 3))
            elif key == ord("p") and item is not None:
                path = out_dir / f"snap_{time.strftime('%H%M%S')}.png"
                cv2.imwrite(str(path), view)
                print(f"Saved {path}", flush=True)
            elif key in (ord("+"), ord("=")):
                ident.det["threshold"] = round(min(0.9, ident.det["threshold"] + 0.01), 2)
            elif key in (ord("-"), ord("_")):
                ident.det["threshold"] = round(max(0.02, ident.det["threshold"] - 0.01), 2)

        quitting = True
        print("Going back to the start height...", flush=True)
        if worker.error is None:
            while worker.busy:
                time.sleep(0.05)
            worker.goto(0.0)
            while worker.busy:
                time.sleep(0.05)
        print(f"Z is {worker.z:+.2f} mm from start.", flush=True)
    except KeyboardInterrupt:
        print("\nCtrl-C: feed hold + reset. Check the Z height by eye before the next run.", flush=True)
        grbl.stop()
    finally:
        worker.finish()
        worker.join(timeout=30)
        grbl.close()
        cam.close()
        cv2.destroyAllWindows()
        if not quitting:
            print(f"Z may not be at the start height. Saved offset: {load_z_offset():+.2f} mm", flush=True)


if __name__ == "__main__":
    main()
