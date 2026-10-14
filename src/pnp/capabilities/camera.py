"""Cameras. The V4L2 one keeps reading in a thread, so the driver never hands out old frames (CLAUDE.md:
stale frames), reconnects when the USB drops, and sets the exposure from the profile every time it opens
(docs/13 §3.5: a lamp changed everything when auto exposure was on)."""
from __future__ import annotations

import threading
import time
from collections import deque

import numpy as np

from ..profile.schema import Camera as CameraCfg
from .base import Availability, Capability, NoneCap, NotBuilt


class FrameBuffer:
    """The newest frames with their times. frames_after() skips the first new frame (it may be old)."""

    def __init__(self, keep: int = 30) -> None:
        self.frames: deque[tuple[float, np.ndarray]] = deque(maxlen=keep)
        self.lock = threading.Lock()
        self.cv = threading.Condition(self.lock)

    def put(self, frame: np.ndarray, t: float | None = None) -> None:
        with self.cv:
            self.frames.append((time.monotonic() if t is None else t, frame))
            self.cv.notify_all()

    def latest(self) -> tuple[float, np.ndarray] | None:
        with self.lock:
            return self.frames[-1] if self.frames else None

    def frames_after(self, t: float, n: int, timeout: float = 5.0) -> list[np.ndarray]:
        deadline = time.monotonic() + timeout
        with self.cv:
            while True:
                new = [f for ts, f in self.frames if ts > t]
                if len(new) >= n + 1:
                    return new[1:n + 1]
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError("the camera gives no new frames")
                self.cv.wait(left)


class V4L2Camera(Capability):
    kind = "v4l2"

    def __init__(self, role: str, cfg: CameraCfg) -> None:
        import cv2
        self.cv2 = cv2
        self.role, self.cfg = role, cfg
        self.buf = FrameBuffer()
        self.controls: dict[str, float] = {}
        if not cfg.exposure.auto and cfg.exposure.value is not None:
            self.controls = {"AUTO_EXPOSURE": cfg.exposure.manual_mode, "EXPOSURE": cfg.exposure.value}
        self.cap = None
        self.error = ""
        self.running = True
        self.thread = threading.Thread(target=self._run, name=f"camera-{role}", daemon=True)
        self.thread.start()

    def _open(self):
        cv2 = self.cv2
        dev = int(self.cfg.device) if self.cfg.device.isdigit() else self.cfg.device
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            self.error = f"can't open {self.cfg.device}"
            return None
        if self.cfg.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.cfg.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.resolution[1])
        for name, value in self.controls.items():
            cap.set(getattr(cv2, f"CAP_PROP_{name}"), float(value))
        self.error = ""
        return cap

    def _run(self) -> None:
        fails = 0
        while self.running:
            if self.cap is None:
                self.cap = self._open()
                if self.cap is None:
                    time.sleep(0.5)
                    continue
            ok, f = self.cap.read()
            if ok:
                fails = 0
                self.buf.put(self.cv2.cvtColor(f, self.cv2.COLOR_BGR2GRAY) if f.ndim == 3 else f)
                continue
            fails += 1
            if fails >= 5:                       # gone from the USB: open it again when it is back
                self.cap.release()
                self.cap = None
                self.error = "no frames (USB?): reconnecting"
            time.sleep(0.02)

    def availability(self) -> Availability:
        item = self.buf.latest()
        if item is None or time.monotonic() - item[0] > 2.0:
            return Availability.no(f"No frames from {self.cfg.device}" + (f" ({self.error})" if self.error else ""),
                                   "Check the camera's USB cable and the device in the profile")
        return Availability.yes()

    def latest(self):
        return self.buf.latest()

    def frames_after(self, t: float, n: int, timeout: float = 5.0):
        return self.buf.frames_after(t, n, timeout)

    def set_exposure(self, value: float | None) -> bool:
        """A fixed exposure (None = the camera's auto exposure). Kept for reconnects."""
        if value is None:
            self.controls = {"AUTO_EXPOSURE": 3}
        else:
            self.controls = {"AUTO_EXPOSURE": self.cfg.exposure.manual_mode, "EXPOSURE": float(value)}
        cap = self.cap
        if cap is None:
            return False
        return all(cap.set(getattr(self.cv2, f"CAP_PROP_{k}"), float(v)) for k, v in self.controls.items())

    def close(self) -> None:
        self.running = False
        self.thread.join(timeout=2)
        if self.cap is not None:
            self.cap.release()


class Picamera2Camera(Capability):
    """A camera on the Pi's camera port (D5: the bottom camera), through picamera2 (Raspberry Pi OS:
    `sudo apt install python3-picamera2`). Grey frames in a grab thread, like the USB camera.
    NOT TESTED on a Pi yet (written 2026-10-14 without the hardware)."""
    kind = "picamera2"

    def __init__(self, role: str, cfg: CameraCfg) -> None:
        self.role, self.cfg = role, cfg
        self.buf = FrameBuffer()
        self.cam = None
        self.why = ""
        self.running = False
        try:
            from picamera2 import Picamera2                     # noqa: PLC0415 - only on a Pi
            self.cam = Picamera2(int(cfg.device) if str(cfg.device).isdigit() else 0)
            w, h = cfg.resolution
            self.cam.configure(self.cam.create_video_configuration(main={"size": (w, h), "format": "YUV420"}))
            self.cam.start()
            self.running = True
            self.thread = threading.Thread(target=self._run, name=f"picam-{role}", daemon=True)
            self.thread.start()
        except Exception as err:                                 # noqa: BLE001 - not a Pi, no camera, no library
            self.why = f"picamera2: {err}"

    def _run(self) -> None:
        h = self.cfg.resolution[1]
        while self.running:
            try:
                yuv = self.cam.capture_array("main")
                self.buf.put(np.ascontiguousarray(yuv[:h]))      # the Y plane = the grey picture
            except Exception:                                     # noqa: BLE001
                time.sleep(0.1)

    def availability(self) -> Availability:
        if self.cam is None:
            return Availability.no(self.why or "The camera did not start", "Check the camera cable and picamera2")
        return Availability.yes()

    def latest(self):
        return self.buf.latest()

    def frames_after(self, t: float, n: int, timeout: float = 5.0):
        return self.buf.frames_after(t, n, timeout)

    def set_exposure(self, value) -> bool:
        if self.cam is None:
            return False
        try:
            self.cam.set_controls({"AeEnable": value is None} | ({} if value is None else
                                                                {"ExposureTime": int(value)}))
            return True
        except Exception:                                         # noqa: BLE001
            return False

    def close(self) -> None:
        self.running = False
        if self.cam is not None:
            try:
                self.cam.stop()
            except Exception:                                     # noqa: BLE001
                pass


def build_camera(role: str, cfg: CameraCfg | None) -> Capability:
    if cfg is None:
        return NoneCap(role, f"No {role} camera in this profile")
    if cfg.driver == "v4l2":
        return V4L2Camera(role, cfg)
    if cfg.driver == "picamera2":
        return Picamera2Camera(role, cfg)
    return NotBuilt(role, cfg.driver, "a later phase")
