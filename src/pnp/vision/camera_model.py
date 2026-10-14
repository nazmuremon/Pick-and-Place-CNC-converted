"""The head camera as a real camera (docs/13 §4.5): a pinhole with radial lens bending (k1, k2), looking down at
an angle from a fixed place on the head.

Frames:
- head frame: machine axes (X right, Y back, Z up), origin = the head's reference point (D23): the point on the
  plane Z = 0 that the picture centre sees. Z = height above the Z reference surface (the bed label top).
- a picture point (u, v) seen on a plane at height z is at head-relative (x, y) = px_to_xy(u, v, z); the machine
  position is the head's position + that.

A tilted camera sees a thing h mm higher about h·tan(tilt) mm off (0.47 mm per mm at 25°, docs/13 §3.2): the
model handles every height exactly, so parts, tape and board top each use their own plane.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


def rodrigues(r) -> np.ndarray:
    return cv2.Rodrigues(np.asarray(r, np.float64).reshape(3))[0]


@dataclass
class CameraModel:
    size: tuple[int, int]                    # (width, height) px
    f: float                                 # focal length, px (square pixels)
    c: tuple[float, float]                   # principal point, px
    dist: tuple[float, float]                # k1, k2 (radial)
    rvec: tuple[float, float, float]         # head frame -> camera rotation (Rodrigues)
    tvec: tuple[float, float, float]         # head frame -> camera translation, mm
    moves_with_z: bool = False               # the camera sits on the Z slide
    quality: dict = field(default_factory=dict)

    # ---------------------------------------------------------------- matrices
    @property
    def K(self) -> np.ndarray:
        return np.array([[self.f, 0, self.c[0]], [0, self.f, self.c[1]], [0, 0, 1]], np.float64)

    @property
    def D(self) -> np.ndarray:
        return np.array([self.dist[0], self.dist[1], 0.0, 0.0, 0.0], np.float64)

    @property
    def R(self) -> np.ndarray:
        return rodrigues(self.rvec)

    @property
    def centre_3d(self) -> np.ndarray:
        """The camera's optical centre in the head frame (mm)."""
        return -self.R.T @ np.asarray(self.tvec, np.float64)

    @property
    def tilt_deg(self) -> float:
        """Angle between the optical axis and straight down."""
        axis = self.R.T @ np.array([0.0, 0.0, 1.0])
        return float(np.degrees(np.arccos(np.clip(-axis[2], -1, 1))))

    @property
    def height_mm(self) -> float:
        return float(self.centre_3d[2])

    # ---------------------------------------------------------------- mapping
    def _head_z(self, head_dz: float) -> np.ndarray:
        """A camera on the Z slide moves with Z: everything is head_dz lower relative to it."""
        return np.array([0.0, 0.0, head_dz if self.moves_with_z else 0.0])

    def xy_to_px(self, pts, z: float = 0.0, head_dz: float = 0.0) -> np.ndarray:
        """Head-relative (x, y) on the plane z -> pixels (N x 2)."""
        P = np.asarray(pts, np.float64).reshape(-1, 2)
        X = np.c_[P, np.full(len(P), z)] - self._head_z(head_dz)
        px, _ = cv2.projectPoints(X, np.asarray(self.rvec, np.float64), np.asarray(self.tvec, np.float64),
                                  self.K, self.D)
        return px.reshape(-1, 2)

    def undistort_norm(self, px, iterations: int = 30) -> np.ndarray:
        """Pixels -> undistorted normalized image coordinates (radial k1, k2; fixed-point iteration)."""
        p = np.asarray(px, np.float64).reshape(-1, 2)
        xd = (p - np.asarray(self.c)) / self.f
        x = xd.copy()
        k1, k2 = self.dist
        for _ in range(iterations):
            r2 = (x * x).sum(axis=1, keepdims=True)
            x = xd / (1.0 + k1 * r2 + k2 * r2 * r2)
        return x

    def rays(self, px) -> np.ndarray:
        """Unit ray directions (head frame) through pixels (N x 2 -> N x 3)."""
        n = self.undistort_norm(px)
        d = np.c_[n, np.ones(len(n))] @ self.R            # R.T @ d for each row
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def px_to_xy(self, px, z: float = 0.0, head_dz: float = 0.0) -> np.ndarray:
        """Pixels (N x 2) -> head-relative (x, y) on the plane z."""
        d = self.rays(px)
        C = self.centre_3d + self._head_z(head_dz)
        lam = (z - C[2]) / d[:, 2]
        return C[:2] + lam[:, None] * d[:, :2]

    def px_per_mm(self, z: float = 0.0, at=None) -> float:
        """Picture scale on the plane z around a pixel (default: the picture centre)."""
        u, v = at if at is not None else (self.size[0] / 2, self.size[1] / 2)
        a = self.px_to_xy([[u - 5, v], [u + 5, v], [u, v - 5], [u, v + 5]], z)
        return float(10.0 / ((np.hypot(*(a[1] - a[0])) + np.hypot(*(a[3] - a[2]))) / 2))

    def anchor(self) -> CameraModel:
        """Move the head frame's origin so the picture centre sees (0, 0) on the plane z = 0 (D23)."""
        o = self.px_to_xy([[self.size[0] / 2, self.size[1] / 2]], 0.0)[0]
        t = np.asarray(self.tvec, np.float64) + self.R @ np.array([o[0], o[1], 0.0])
        return CameraModel(self.size, self.f, self.c, self.dist, self.rvec, tuple(t), self.moves_with_z, self.quality)

    def shift_px(self, delta_xy, z: float = 0.0) -> np.ndarray:
        """How far the picture moves (u, v) at its centre when the head moves by delta_xy."""
        centre = self.px_to_xy([[self.size[0] / 2, self.size[1] / 2]], z)[0]
        a, b = self.xy_to_px([centre, centre - np.asarray(delta_xy, float)], z)
        return b - a

    # ---------------------------------------------------------------- files
    def to_dict(self) -> dict:
        return {"size": list(self.size), "f": float(self.f), "c": [float(v) for v in self.c],
                "dist": [float(v) for v in self.dist], "rvec": [float(v) for v in self.rvec],
                "tvec": [float(v) for v in self.tvec], "moves_with_z": self.moves_with_z,
                "quality": self.quality, "derived": {"tilt_deg": round(self.tilt_deg, 3),
                                                     "height_mm": round(self.height_mm, 3),
                                                     "px_per_mm_z0": round(self.px_per_mm(0.0), 4)}}

    @classmethod
    def from_dict(cls, d: dict) -> CameraModel:
        return cls(tuple(d["size"]), float(d["f"]), tuple(d["c"]), tuple(d["dist"]), tuple(d["rvec"]),
                   tuple(d["tvec"]), bool(d.get("moves_with_z", False)), dict(d.get("quality", {})))

    @classmethod
    def looking_down(cls, size: tuple[int, int], f: float, height: float, tilt_deg: float = 0.0,
                     tilt_dir_deg: float = 0.0, dist=(0.0, 0.0), turn_deg: float = 0.0) -> CameraModel:
        """A camera `height` mm above the plane z = 0, turned by tilt_deg away from straight down (towards the
        direction tilt_dir_deg in the plane), and turned about its own axis by turn_deg. Anchored (D23)."""
        down = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], np.float64)   # camera x = +X, y = -Y, z = -Z
        R = rodrigues([0, 0, np.radians(turn_deg)]) @ down
        a = np.radians(tilt_dir_deg)
        axis = np.array([-np.sin(a), np.cos(a), 0.0])
        R = R @ rodrigues(axis * np.radians(tilt_deg)).T
        C = np.array([0.0, 0.0, height])
        t = -R @ C
        m = cls(size, f, (size[0] / 2, size[1] / 2), tuple(dist), tuple(cv2.Rodrigues(R)[0].ravel()), tuple(t))
        return m.anchor()
