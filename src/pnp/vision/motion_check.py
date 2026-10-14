"""Did the head move as commanded? (docs/13 §4.8: stall detection on every automatic move; from
tools/bed_test_run.py, kept with its tests.) The picture before and after a move is compared with where the camera
model says it should have moved."""
from __future__ import annotations

import cv2
import numpy as np


def moved_as_commanded(a: np.ndarray, b: np.ndarray, expected_px, margin: float = 0.05) -> bool | None:
    """Picture a before a move, b after it; expected_px = how far the picture should shift (u, v).
    True: b looks like a shifted by that (the head moved). False: b looks like a unshifted (it did not move:
    blocked). None: the picture can't tell (a plain area, long bars along the move, or a scene that repeats at that
    shift)."""
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

    # a scene that repeats at this shift (a strip's pockets at its pitch, the bed's bars) can't tell "moved" from
    # "did not move" (dry run 2026-10-12: a false stall moving one pocket along a strip)
    if corr(ref, a[ys.start + dy:ys.stop + dy, xs.start + dx:xs.stop + dx]) > 0.8:
        return None
    c_moved, c_still = corr(ref, shifted), corr(ref, still)
    if c_moved > c_still + margin:
        return True
    if c_still > c_moved + margin:
        return False
    return None


def moved_as_predicted(a: np.ndarray, b: np.ndarray, H: np.ndarray, margin: float = 0.05) -> bool | None:
    """Like moved_as_commanded, but the move's effect on the picture is the homography H (from the camera model:
    a tilted camera sees a head move as a perspective change, not one shift). True = moved, False = did not move,
    None = can't tell (plain, or the scene looks the same after that move: a strip's pitch, the bars)."""
    a = cv2.GaussianBlur(np.float32(a), (0, 0), 2)
    b = cv2.GaussianBlur(np.float32(b), (0, 0), 2)
    h, w = a.shape
    warped = cv2.warpPerspective(a, H, (w, h), flags=cv2.INTER_LINEAR, borderValue=-1.0)
    mask = warped >= 0
    mask[: h // 10] = mask[-h // 10:] = False                    # the nozzle's shadow and the picture edges
    if mask.sum() < max(2000, 0.15 * h * w) or a[mask].std() < 2:
        return None                                              # too little overlap to judge

    def corr(x, y):
        x, y = x[mask], y[mask]
        return float(np.corrcoef(x, y)[0, 1]) if x.std() > 1e-6 and y.std() > 1e-6 else 0.0

    if corr(warped, a) > 0.8:
        return None                                              # the scene repeats at this move
    c_moved, c_still = corr(warped, b), corr(a, b)
    if c_moved > c_still + margin:
        return True
    if c_still > c_moved + margin:
        return False
    return None


def measured_shift(a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, float]:
    """How far the picture really moved (u, v) and how sure (0..1): phase correlation."""
    a, b = np.float32(a), np.float32(b)
    (dx, dy), sure = cv2.phaseCorrelate(a, b, cv2.createHanningWindow(a.shape[::-1], cv2.CV_32F))
    return np.array([dx, dy]), float(sure)


def camera_turn(a: np.ndarray, b: np.ndarray, expected_px) -> float | None:
    """The picture moved about as far as expected but in another direction: the camera was turned since its
    calibration. Returns that turn in degrees, or None."""
    got, sure = measured_shift(a, b)
    exp = np.asarray(expected_px, float)
    if sure < 0.1 or not 0.6 <= np.hypot(*got) / np.hypot(*exp) <= 1.6:
        return None
    turn = np.degrees(np.arctan2(got[1], got[0]) - np.arctan2(exp[1], exp[0]))
    return float((turn + 180) % 360 - 180)
