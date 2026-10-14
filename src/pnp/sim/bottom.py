"""The simulated bottom camera (docs/13 §4.14, Phase G): fixed in the bed, looking up. It sees the needle tip and
the part held on it from below; only what is near its focus height is sharp. The program does not know where it is,
how it is turned or how big a mm is in its picture: the Setup step measures it."""
from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np

from ..capabilities.base import Availability, Capability
from ..capabilities.camera import FrameBuffer


class SimBottomCamera(Capability):
    kind = "sim"

    def __init__(self, role: str, grbl, cfg, size=(640, 480), fps: float = 15.0, seed: int = 0, run: bool = True,
                 od_mm: float = 0.72) -> None:
        self.role, self.g, self.cfg = role, grbl, cfg
        self.size = size
        self.od = od_mm
        self.gain = 1.0
        self.rng = np.random.default_rng(seed + 7)
        self.buf = FrameBuffer()
        self.period = 1.0 / fps
        self.running = run
        self.thread = threading.Thread(target=self._run, name=f"simcam-{role}", daemon=True)
        if run:
            self.thread.start()

    def _nozzle(self):
        from . import nozzle_of
        return nozzle_of(self.g)

    def to_px(self, xy) -> np.ndarray:
        c = self.cfg
        d = np.asarray(xy, float) - np.asarray(c.xy, float)
        if c.mirror:
            d = np.array([-d[0], d[1]])                    # seen from below: left and right swap
        a = math.radians(c.turn_deg)
        d = np.array([d[0] * math.cos(a) - d[1] * math.sin(a), d[0] * math.sin(a) + d[1] * math.cos(a)])
        w, h = self.size
        return np.array([w / 2 + d[0] * c.px_per_mm, h / 2 - d[1] * c.px_per_mm])

    def render(self) -> np.ndarray:
        w, h = self.size
        img = np.full((h, w), 18.0, np.float32)
        noz = self._nozzle()
        if noz is not None:
            with self.g.lock:
                tip = noz.tip_xy()
                z = self.g.phys["Z"]
                held = noz.held
                axis = noz.axis_xy()
                servo = self.g.servo_angle
            k = self.cfg.px_per_mm
            bottom = z - (held.height if held is not None else 0.0)       # what the camera looks at
            blur = abs(bottom - self.cfg.focus_z) * 1.5
            if blur < 12:
                pts = lambda P: np.round((np.array([self.to_px(p) for p in P]) - 0.5) * 16).astype(np.int32)  # noqa
                c = self.to_px(tip)
                cv2.circle(img, (int(round(c[0] * 16)), int(round(c[1] * 16))), int(round(self.od / 2 * k * 16)),
                           120.0, -1, cv2.LINE_AA, shift=4)
                if held is not None:
                    d = servo - held.servo
                    a = math.radians(d)
                    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
                    pc = axis + R @ held.rel
                    ang = math.radians(held.angle + d)
                    ax, nx = np.array([math.cos(ang), math.sin(ang)]), np.array([-math.sin(ang), math.cos(ang)])
                    L, W = held.body

                    def rect(a0, a1):
                        return [pc + ax * p + nx * q for p, q in ((a0, -W / 2), (a1, -W / 2), (a1, W / 2), (a0, W / 2))]
                    cv2.fillPoly(img, [pts(rect(-L / 2, L / 2))], 150.0, cv2.LINE_AA, shift=4)
                    e = min(0.35, L * 0.2)
                    cv2.fillPoly(img, [pts(rect(-L / 2, -L / 2 + e))], 235.0, cv2.LINE_AA, shift=4)
                    cv2.fillPoly(img, [pts(rect(L / 2 - e, L / 2))], 235.0, cv2.LINE_AA, shift=4)
                if blur > 0.3:
                    img = cv2.GaussianBlur(img, (0, 0), blur)
        img = img * self.gain + self.rng.normal(0, 2.0, img.shape).astype(np.float32)
        return np.clip(img, 0, 255).astype(np.uint8)

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
        if not self.running:
            return [self.render() for _ in range(n)]
        return self.buf.frames_after(t, n, timeout)

    def set_exposure(self, value) -> bool:
        return True

    def close(self) -> None:
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=2)
