"""The bottom camera (Phase G, docs/13 §4.14 "PlaceCorrector"): a camera fixed in the bed, looking up at the part on
the nozzle. Its calibration maps a tip / part position in the picture to the machine frame (mm from HOME):

  machine point = cam_xy + A⁻¹ · (px − centre)

A (2 x 2, px per mm) holds the scale, the turn and the mirror (a camera looking up sees the world mirrored); it is
measured by moving the head 1 mm in X and in Y with the bare needle in view. cam_xy = the machine point seen at the
picture centre, from the head position with the tip there and the measured nozzle offset. Everything is valid at
the focus height `tip_z` (where the operator focused the needle); a part is looked at with its bottom there.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import cv2
import numpy as np


@dataclass
class Blob:
    u: float
    v: float
    angle_px: float                  # the long side's direction in the picture (degrees, v down), mod 180
    length_px: float
    width_px: float
    area_px: float


def find_blob(img: np.ndarray, min_area: float = 20.0) -> Blob | None:
    """The biggest bright thing on the dark background (the needle tip, or the part hiding it)."""
    g = cv2.GaussianBlur(np.asarray(img, np.float32), (0, 0), 1.0)
    bg = float(np.percentile(g, 20))
    top = float(np.percentile(g, 99.9))
    if top - bg < 30:
        return None
    thr = bg + 0.35 * (top - bg)
    mask = (g > thr).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[i, cv2.CC_STAT_AREA] < min_area:
        return None
    ys, xs = np.nonzero(lab == i)
    w = (g[ys, xs] - bg).astype(np.float64)
    mx, my = float((w * xs).sum() / w.sum()), float((w * ys).sum() / w.sum())
    # the outline's shape (unweighted: the bright terminals must not pull the long side's direction)
    cov = np.cov(np.vstack([xs - xs.mean(), ys - ys.mean()]))
    ev, evec = np.linalg.eigh(cov)
    d = evec[:, 1]
    return Blob(mx, my, math.degrees(math.atan2(d[1], d[0])) % 180.0, 4 * math.sqrt(max(ev[1], 0)),
                4 * math.sqrt(max(ev[0], 0)), float(len(xs)))


@dataclass
class BottomCal:
    A: list[list[float]]             # px per mm: picture shift = A · head shift
    centre: list[float]              # the picture centre (px)
    cam_xy: list[float]              # the machine point seen at the picture centre (mm from HOME)
    tip_z: float                     # the focus height (tip, bare needle)
    servo: float | None = None       # the servo angle during the calibration

    def to_machine(self, px) -> np.ndarray:
        Ai = np.linalg.inv(np.asarray(self.A, float))
        return np.asarray(self.cam_xy, float) + Ai @ (np.asarray(px, float) - np.asarray(self.centre, float))

    def angle_to_machine(self, angle_px: float) -> float:
        """A direction in the picture -> its direction in the machine frame (degrees)."""
        a = math.radians(angle_px)
        d = np.linalg.inv(np.asarray(self.A, float)) @ np.array([math.cos(a), math.sin(a)])
        return math.degrees(math.atan2(d[1], d[0]))

    def px_per_mm(self) -> float:
        return float(np.sqrt(abs(np.linalg.det(np.asarray(self.A, float)))))

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> BottomCal:
        return cls(d["A"], d["centre"], d["cam_xy"], float(d["tip_z"]), d.get("servo"))
