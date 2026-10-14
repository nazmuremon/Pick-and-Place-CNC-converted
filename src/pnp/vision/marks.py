"""What changed between two pictures of the same view: a new ink mark (nozzle calibration) or a part just put down
(the place check). Both compare a picture before and after, so old marks, pads and labels don't count.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Change:
    u: float                         # centre in the picture (px)
    v: float
    angle: float                     # the long side's direction (degrees, picture frame: v down), mod 180
    length_px: float
    width_px: float
    strength: float                  # the mean brightness change in the blob
    why: str = ""


def _elsewhere(a: np.ndarray, b: np.ndarray, box) -> float:
    m = np.ones(a.shape, bool)
    m[box] = False
    h = a.shape[0]
    m[: h // 10] = m[-h // 10:] = False
    return float(np.median(np.abs(a - b)[m])) if m.any() else 0.0


def find_change(before: np.ndarray, after: np.ndarray, near_px, search_px: float, darker: bool | None = None,
                min_change: float = 25.0, min_area_px: int = 4) -> Change | None:
    """The biggest new blob near `near_px`. darker: True = only darker (an ink mark), False = only brighter,
    None = either (a part on a board: bright body or dark top). None when nothing clear changed, or the whole
    picture changed (the light, the camera moved)."""
    a = cv2.GaussianBlur(np.asarray(before, np.float32), (0, 0), 1.0)
    b = cv2.GaussianBlur(np.asarray(after, np.float32), (0, 0), 1.0)
    h, w = a.shape
    cx, cy = near_px
    r = int(math.ceil(search_px))
    x0, x1 = max(0, int(cx) - r), min(w, int(cx) + r + 1)
    y0, y1 = max(0, int(cy) - r), min(h, int(cy) + r + 1)
    if x1 - x0 < 3 or y1 - y0 < 3:
        return None
    box = (slice(y0, y1), slice(x0, x1))
    if _elsewhere(a, b, box) > 8.0:
        return None
    d = (a - b)[box] if darker else ((b - a)[box] if darker is False else np.abs(a - b)[box])
    mask = (d > min_change).astype(np.uint8)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return None
    best = max(range(1, n), key=lambda i: float(d[lab == i].sum()))
    if stats[best, cv2.CC_STAT_AREA] < min_area_px:
        return None
    ys, xs = np.nonzero(lab == best)
    wts = d[ys, xs].astype(np.float64)
    mx, my = float((wts * xs).sum() / wts.sum()), float((wts * ys).sum() / wts.sum())
    if len(xs) >= 3:
        cov = np.cov(np.vstack([xs - mx, ys - my]), aweights=wts)
        ev, evec = np.linalg.eigh(cov)
        long_dir = evec[:, 1]
        angle = math.degrees(math.atan2(long_dir[1], long_dir[0])) % 180.0
        length, width = 4 * math.sqrt(max(ev[1], 0)), 4 * math.sqrt(max(ev[0], 0))
    else:
        angle, length, width = 0.0, 1.0, 1.0
    return Change(x0 + mx, y0 + my, angle, length, width, float(wts.mean()))
