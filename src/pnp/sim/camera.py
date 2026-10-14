"""The simulated head camera: every picture pixel follows the TRUE camera model's ray down to each layer of the
world (bed, strips, board, ChArUco) at that layer's height. Lens bending, tilt and parallax are real; the program
does not know this model and must measure it (Phase C calibration).

The head's REAL position is used (not the counted one), so lost steps show in the picture like on the bench.
"""
from __future__ import annotations

import threading
import time

import cv2
import numpy as np

from ..capabilities.base import Availability, Capability
from ..capabilities.camera import FrameBuffer
from ..vision.camera_model import CameraModel
from .controller import SimGrbl
from .world import World


class SimCamera(Capability):
    kind = "sim"

    def __init__(self, role: str, world: World, grbl: SimGrbl, model: CameraModel, exposure: float | None = None,
                 fps: float = 15.0, noise: float = 2.0, seed: int = 0, run: bool = True,
                 light_port: int | None = None) -> None:
        self.role, self.world, self.grbl, self.model = role, world, grbl, model
        self.light_port = light_port         # a head light on this digital output: off = a darker picture
        self.size = model.size
        self.ref_exposure = exposure or 100.0
        self.gain = 1.0
        self.noise = noise
        self.blur = 0.6                      # px: a real lens is never perfectly sharp
        self.rng = np.random.default_rng(seed)
        self.buf = FrameBuffer()
        self.period = 1.0 / fps
        w, h = self.size
        uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
        d = model.rays(np.c_[uu.ravel(), vv.ravel()])
        self._d = d.reshape(h, w, 3)
        self._cache: dict[float, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self.running = run
        self.thread = threading.Thread(target=self._run, name=f"simcam-{role}", daemon=True)
        if run:
            self.thread.start()

    def _offsets(self, dz: float):
        """Per pixel: where the ray meets a plane dz below the camera centre (relative to the centre), and valid."""
        key = round(dz, 6)
        if key not in self._cache:
            dzr = self._d[..., 2]
            ok = dzr < -1e-6
            lam = np.where(ok, dz / np.where(ok, dzr, -1.0), 0.0)
            self._cache[key] = (lam * self._d[..., 0], lam * self._d[..., 1], ok)
            if len(self._cache) > 32:
                self._cache.pop(next(iter(self._cache)))
        return self._cache[key]

    def render(self, head_xyz=None) -> np.ndarray:
        if head_xyz is None:
            with self.grbl.lock:
                head_xyz = (self.grbl.phys["X"], self.grbl.phys["Y"], self.grbl.phys["Z"])
        hx, hy, hz = head_xyz
        C = self.model.centre_3d + np.array([hx, hy, hz if self.model.moves_with_z else 0.0])
        h, w = self.size[1], self.size[0]
        out = np.zeros((h, w), np.float32)
        for L in self.world.layers:                                    # lowest first
            ox, oy, ok = self._offsets(L.z - C[2])
            mx = ((C[0] + ox - L.x0) * L.T - 0.5).astype(np.float32)
            my = ((L.y1 - (C[1] + oy)) * L.T - 0.5).astype(np.float32)
            g = cv2.remap(L.gray, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
            a = cv2.remap(L.alpha, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0) * ok
            out = out * (1 - a) + g * a
        if self.blur:
            out = cv2.GaussianBlur(out, (0, 0), self.blur)
        out *= self.gain
        if self.light_port is not None and not self.grbl.dout.get(self.light_port):
            out *= 0.55
        if self.noise:
            out += self.rng.normal(0, self.noise, out.shape).astype(np.float32)
        return np.clip(out, 0, 255).astype(np.uint8)

    def _run(self) -> None:
        while self.running:
            t0 = time.monotonic()
            self.buf.put(self.render(), t0)
            time.sleep(max(0.0, self.period - (time.monotonic() - t0)))

    def availability(self) -> Availability:
        return Availability.yes()

    def latest(self):
        return self.buf.latest()

    def frames_after(self, t: float, n: int, timeout: float = 5.0):
        if not self.running:                                            # a camera without its thread (tests)
            return [self.render() for _ in range(n)]
        return self.buf.frames_after(t, n, timeout)

    def set_exposure(self, value: float | None) -> bool:
        self.gain = 1.0 if value is None else float(value) / self.ref_exposure
        return True

    def close(self) -> None:
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2)
