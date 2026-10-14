#!/usr/bin/env python3
"""Move one axis by small steps and measure the real move with the camera.

Test bench: Arduino Uno + CNC shield (Grbl 1.1) and the USB camera looking down at the
printed SMD identify sheet (ArUco markers 20-23, see config/smd_identify.yaml).
Axes are machine axes: config/bench.yaml maps them to the Grbl axes (X/Z swap).

There are NO limit switches. So every move is small and slow, and goes straight back to the
start point. The test stops (after going back) if the axis moves much more or much less than
commanded, or if the camera loses the markers.

How it measures: the sheet markers give a homography (sheet mm <-> camera pixels). The image
centre is turned into sheet mm before and after each move. The difference is the real move of
the camera over the sheet, in mm, and camera tilt does not matter. For Z, the marker size
(px/mm) shows if the camera went up (smaller) or down (bigger).

    .venv/bin/python tools/axis_camera_test.py --axis X
    .venv/bin/python tools/axis_camera_test.py --axis Y --moves 0.5,1,2,5 --feed 150
    .venv/bin/python tools/axis_camera_test.py --axis Z --moves 0.5,1

Results: logs/axis_test/<time>_<axis>.csv and annotated images.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from bench import Grabber, Grbl, GrblError
from pnp_paths import LOGS_DIR
from smd_identify import DEFAULT_CAMERA, DEFAULT_CONFIG, load_config, marker_positions_mm

ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = LOGS_DIR / "axis_test"

SETTLE_S = 0.15         # wait after the axis stops, before using frames (stale frame rule)
FRAMES_PER_MEASURE = 5
RATIO_LOW, RATIO_HIGH = 0.5, 1.5   # measured / commanded must stay inside this


@dataclass
class Measurement:
    xy: np.ndarray          # image centre in sheet mm
    xy_std: float           # spread over the frames, mm
    px_per_mm: float
    markers: int            # fewest markers seen in one frame
    frame: np.ndarray
    corners: list


class SheetMeter:
    def __init__(self, cfg: dict) -> None:
        sheet = cfg["sheet"]
        self.size = float(sheet["marker_size_mm"])
        self.markers_mm = marker_positions_mm(sheet)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, sheet["dictionary"]))
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)

    def _one(self, gray: np.ndarray):
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return None
        src, dst, sides, kept = [], [], [], []
        for c, i in zip(corners, ids.flatten()):
            if int(i) in self.markers_mm:
                pts = c.reshape(4, 2)
                src.append(self.markers_mm[int(i)])
                dst.append(pts)
                sides.append(np.mean(np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)))
                kept.append(c)
        if not src:
            return None
        H, _ = cv2.findHomography(np.concatenate(src), np.concatenate(dst), 0)
        if H is None:
            return None
        h, w = gray.shape
        centre = np.array([[[w / 2, h / 2]]], np.float32)
        xy = cv2.perspectiveTransform(centre, np.linalg.inv(H)).reshape(2)
        return xy, float(np.mean(sides)) / self.size, len(src), kept

    def measure(self, frames: list[np.ndarray]) -> Measurement | None:
        results = [r for r in (self._one(f) for f in frames) if r is not None]
        if len(results) < len(frames) // 2 + 1:
            return None
        xys = np.array([r[0] for r in results])
        mean = xys.mean(axis=0)
        return Measurement(
            xy=mean,
            xy_std=float(np.linalg.norm(xys - mean, axis=1).max()),
            px_per_mm=float(np.mean([r[1] for r in results])),
            markers=min(r[2] for r in results),
            frame=frames[-1],
            corners=results[-1][3],
        )


def annotate(m: Measurement, text: str) -> np.ndarray:
    img = cv2.cvtColor(m.frame, cv2.COLOR_GRAY2BGR)
    for c in m.corners:
        cv2.polylines(img, [c.reshape(4, 2).astype(np.int32)], True, (255, 200, 0), 2)
    h, w = m.frame.shape
    cv2.drawMarker(img, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 30, 1)
    cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return img


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--axis", required=True, choices=["X", "Y", "Z"], help="machine axis")
    ap.add_argument("--moves", default="0.5,1,2,5", help="distances in mm, small first (default 0.5,1,2,5)")
    ap.add_argument("--reverse", action="store_true", help="go the negative way first (axis near its + end)")
    ap.add_argument("--feed", type=float, default=150.0, help="mm/min (capped by config/bench.yaml)")
    ap.add_argument("--port", help="serial port (default: config/bench.yaml, else the first USB port)")
    ap.add_argument("--camera", default=DEFAULT_CAMERA)
    ap.add_argument("--size", default="640x400", help="camera frame size WxH")
    args = ap.parse_args()

    moves = [float(v) for v in args.moves.split(",")]
    if any(d <= 0 for d in moves):
        sys.exit("Moves must be > 0. Use --reverse for the negative way.")
    axis = args.axis
    sign = -1.0 if args.reverse else 1.0
    width, height = (int(v) for v in args.size.lower().split("x"))

    cfg, _ = load_config(DEFAULT_CONFIG)
    meter = SheetMeter(cfg)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

    cam = Grabber(args.camera, width, height)
    grbl = Grbl(args.port)
    rows = []
    try:
        grbl.check_ready()
        state, pos = grbl.status()
        print(f"Grbl state {state}, machine position {pos}")
        steps_key = grbl.axes.steps_setting(axis)
        steps_now = float(grbl.settings()[steps_key])
        print(f"Machine {axis} = Grbl {grbl.axes.to_grbl[axis][0]} (sign {grbl.axes.to_grbl[axis][1]:+.0f}), "
              f"{steps_key} = {steps_now} steps/mm, feed {args.feed:.0f} mm/min")

        def measure(label: str) -> Measurement:
            m = meter.measure(cam.frames_after(time.monotonic() + SETTLE_S, FRAMES_PER_MEASURE))
            if m is None:
                raise RuntimeError(f"markers not found ({label})")
            return m

        time.sleep(SETTLE_S)
        start = measure("start")
        print(f"Start: sheet ({start.xy[0]:.3f}, {start.xy[1]:.3f}) mm, {start.px_per_mm:.2f} px/mm, "
              f"{start.markers} markers, noise {start.xy_std:.3f} mm")
        cv2.imwrite(str(LOG_DIR / f"{stamp}_{axis}_start.png"), annotate(start, f"{axis} start"))

        print(f"\n{'cmd mm':>7} {'meas mm':>8} {'ratio':>6} {'dir deg':>8} {'px/mm':>6} {'back err':>9}")
        for d in moves:
            before = measure("before")
            grbl.move_relative(axis, sign * d, args.feed)
            try:
                out = measure("after move")
            finally:
                grbl.move_relative(axis, -sign * d, args.feed)
            back = measure("after return")

            delta = out.xy - before.xy
            dist = float(np.linalg.norm(delta))
            ratio = dist / d
            direction = float(np.degrees(np.arctan2(delta[1], delta[0])))
            back_err = float(np.linalg.norm(back.xy - before.xy))
            scale_change = (out.px_per_mm / before.px_per_mm - 1) * 100
            print(f"{sign * d:7.3f} {dist:8.3f} {ratio:6.2f} {direction:8.1f} {out.px_per_mm:6.2f} {back_err:9.3f}"
                  + (f"   scale {scale_change:+.2f} %" if axis == "Z" else ""))
            rows.append({
                "axis": axis, "commanded_mm": sign * d, "measured_mm": round(dist, 4), "ratio": round(ratio, 4),
                "dx_sheet_mm": round(float(delta[0]), 4), "dy_sheet_mm": round(float(delta[1]), 4),
                "direction_deg": round(direction, 1), "px_per_mm_before": round(before.px_per_mm, 3),
                "px_per_mm_out": round(out.px_per_mm, 3), "scale_change_pct": round(scale_change, 3),
                "return_error_mm": round(back_err, 4), "markers_min": min(out.markers, back.markers),
                "noise_mm": round(max(before.xy_std, out.xy_std, back.xy_std), 4),
            })
            cv2.imwrite(str(LOG_DIR / f"{stamp}_{axis}_{sign * d:+g}.png"),
                        annotate(out, f"{axis} {sign * d:+g} mm -> measured {dist:.3f} mm"))

            if axis != "Z":
                too_far = ratio > RATIO_HIGH and dist - d > 0.2
                too_short = ratio < RATIO_LOW and d >= 1.0
                if too_far or too_short:
                    print(f"\nSTOP: the axis moved {dist:.3f} mm for {d} mm commanded. "
                          "Check steps/mm, motor power and wiring before bigger moves.")
                    break
            if back_err > 0.3:
                print(f"\nSTOP: did not come back to the start ({back_err:.3f} mm off). "
                      "Lost steps or a loose part? Check before bigger moves.")
                break

        end = measure("end")
        total = float(np.linalg.norm(end.xy - start.xy))
        print(f"\nEnd vs start: {total:.3f} mm")

        good = [r for r in rows if abs(r["commanded_mm"]) >= 1.0 and RATIO_LOW <= r["ratio"] <= RATIO_HIGH]
        if axis != "Z" and good:
            best = max(good, key=lambda r: abs(r["commanded_mm"]))
            suggested = steps_now * best["ratio"] ** -1
            print(f"From the {best['commanded_mm']} mm move: {steps_key} should be about {suggested:.1f} "
                  f"(now {steps_now}). This assumes the printed sheet is exactly to scale "
                  "(check the 50 mm bar with a ruler).")
    except GrblError as e:
        print(f"\nGrbl: {e}")
    except KeyboardInterrupt:
        print("\nCtrl-C: feed hold + reset")
        grbl.stop()
    finally:
        grbl.close()
        cam.close()
        if rows:
            path = LOG_DIR / f"{stamp}_{axis}.csv"
            with path.open("w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
            print(f"Saved {path}")


if __name__ == "__main__":
    main()
