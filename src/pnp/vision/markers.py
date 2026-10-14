"""The bed labels (ArUco markers of bed.yaml): found in a picture with sub-pixel corners, then measured on the label
plane (z = 0) with the camera model. HOME homing and the HOME drift check use these (docs/13 §4.5)."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .camera_model import CameraModel


@dataclass
class Seen:
    name: str                       # home, slot_3, pcb_1
    corners_px: np.ndarray          # 4 x 2, ArUco order (corner 0 -> 1 = the label's +X)
    centre_xy: np.ndarray | None = None   # head-relative mm on the label plane (with a camera model)
    sides_mm: np.ndarray | None = None
    edge: bool = False              # touches the picture edge (may be cut off: not for measuring)


class Markers:
    def __init__(self, bed: dict, trays: dict[int, str] | None = None, dictionary: str | None = None) -> None:
        """trays: marker id -> name from the feeder bank (docs/13 §4.4.1); without it, bed.yaml's slot_ids.
        A marker id used twice on the bed is an error."""
        mk = bed["markers"]
        self.names = {int(mk["home_id"]): "home"}
        if (bed.get("pcb_area") or {}).get("corner_labels", True):
            self.names.update({int(i): f"pcb_{n}" for n, i in enumerate(mk.get("pcb_corner_ids", []), start=1)})
        if trays is None:
            trays = {int(i): f"slot_{n}" for n, i in enumerate(mk.get("slot_ids", []), start=1)}
        for i, name in trays.items():
            if int(i) in self.names:
                raise ValueError(f"marker {i} is used twice ({self.names[int(i)]} and {name})")
            self.names[int(i)] = name
        self.size_mm = float(mk["size_mm"])
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if mk.get("border_error_rate") is not None:
            params.maxErroneousBitsInBorderRate = float(mk["border_error_rate"])
        self.dictionary = dictionary or mk["dictionary"]
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary)),
                                                params)
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)) if mk.get("clahe") else None

    def find(self, gray: np.ndarray, model: CameraModel | None = None, z: float = 0.0,
             edge_px: float = 3.0) -> dict[str, Seen]:
        g = gray if gray.ndim == 2 else cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
        if self.clahe is not None:
            g = self.clahe.apply(g)
        corners, ids, _ = self.detector.detectMarkers(g)
        out: dict[str, Seen] = {}
        if ids is None:
            return out
        h, w = g.shape[:2]
        for c, i in zip(corners, ids.ravel()):
            name = self.names.get(int(i))
            if name is None:
                continue
            c = c.reshape(4, 2).astype(np.float64)
            s = Seen(name, c, edge=bool(c[:, 0].min() < edge_px or c[:, 1].min() < edge_px or
                                        c[:, 0].max() > w - 1 - edge_px or c[:, 1].max() > h - 1 - edge_px))
            if model is not None:
                xy = model.px_to_xy(c, z)
                s.centre_xy = xy.mean(axis=0)
                s.sides_mm = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)
            out[name] = s
        return out

    def measure(self, frames: list[np.ndarray], model: CameraModel, name: str, z: float = 0.0) -> Seen | None:
        """One label measured on several pictures (corners averaged). None if not seen whole in most of them."""
        cs = []
        for f in frames:
            s = self.find(f, None).get(name)
            if s is not None and not s.edge:
                cs.append(s.corners_px)
        if len(cs) < (len(frames) + 1) // 2:
            return None
        c = np.mean(cs, axis=0)
        xy = model.px_to_xy(c, z)
        return Seen(name, c, xy.mean(axis=0), np.hypot(*(np.roll(xy, -1, axis=0) - xy).T))


def home_offset(markers: Markers, frames: list[np.ndarray], model: CameraModel, z: float = 0.0,
                tol_mm: float = 0.25) -> tuple[np.ndarray | None, str]:
    """Where HOME is from the head (mm), and why not when it can't be measured. The label must measure its true
    size (a wrong camera height or a stale calibration shows here first)."""
    s = markers.measure(frames, model, "home", z)
    if s is None:
        return None, "HOME is not in the picture (or cut off at its edge): jog until it is in view"
    worst = float(np.max(np.abs(s.sides_mm - markers.size_mm)))
    if worst > tol_mm:
        return None, (f"HOME measures {s.sides_mm.mean():.2f} mm, not {markers.size_mm:g} mm: the camera calibration "
                      "does not fit (camera moved? Z changed?). Calibrate the camera again")
    return s.centre_xy, ""
