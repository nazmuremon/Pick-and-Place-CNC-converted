"""Shared helpers for the PC test bench: Arduino Uno + CNC shield (Grbl 1.1) and a USB camera.

Not part of the planned src/pnp app. Settings: config/bench.yaml.
All moves use machine axes (X, Y, Z). The axis map in bench.yaml turns them into Grbl axes,
so a motor plugged into the wrong driver can be fixed in software.
"""
from __future__ import annotations

import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import serial
import yaml
from serial.tools import list_ports

from pnp_paths import CONFIG_DIR, LOGS_DIR

ROOT = Path(__file__).resolve().parents[1]
BENCH_CONFIG = CONFIG_DIR / "bench.yaml"
GRBL_AXES = "XYZ"


def load_bench(path: Path = BENCH_CONFIG) -> dict:
    return yaml.safe_load(path.read_text())


STATE_FILE = LOGS_DIR / "nozzle_state.yaml"


def mark_parked(parked: bool, by: str) -> None:
    """parked = the nozzle was left up at its camera height by a clean quit. Every program that opens the Grbl
    port writes false; only a clean quit with Z back up writes true (D19)."""
    try:
        STATE_FILE.write_text(yaml.safe_dump({"parked": bool(parked), "by": by,
                                              "time": time.strftime("%Y-%m-%dT%H:%M:%S")}))
    except OSError:
        pass


def was_parked() -> bool:
    """Did the last program leave the nozzle up (clean quit)? No file yet: yes. Unreadable: no."""
    try:
        return bool((yaml.safe_load(STATE_FILE.read_text()) or {}).get("parked", False))
    except FileNotFoundError:
        return True
    except (OSError, yaml.YAMLError):
        return False


class GrblError(RuntimeError):
    pass


@dataclass(frozen=True)
class AxisMap:
    """Machine axis -> (Grbl axis, sign)."""
    to_grbl: dict[str, tuple[str, float]]

    @classmethod
    def from_config(cls, cfg: dict) -> AxisMap:
        m = {str(a).upper(): (str(v["grbl"]).upper(), float(v.get("sign", 1))) for a, v in cfg["axis_map"].items()}
        if sorted(m) != list(GRBL_AXES) or sorted(g for g, _ in m.values()) != list(GRBL_AXES):
            raise ValueError(f"axis_map must use each of X, Y, Z once on both sides: {m}")
        if any(s not in (1.0, -1.0) for _, s in m.values()):
            raise ValueError("axis_map sign must be 1 or -1")
        return cls(m)

    def word(self, axis: str, dist: float) -> str:
        """G-code word for a machine-axis distance, e.g. Z +2 -> 'X-2.000'."""
        grbl, sign = self.to_grbl[axis]
        return f"{grbl}{sign * dist:.3f}"

    def steps_setting(self, axis: str) -> str:
        """The Grbl steps/mm setting ($100..$102) of the motor that moves this machine axis."""
        return f"$10{GRBL_AXES.index(self.to_grbl[axis][0])}"

    def machine_pos(self, grbl_pos: np.ndarray) -> dict[str, float]:
        return {a: sign * float(grbl_pos[GRBL_AXES.index(g)]) for a, (g, sign) in self.to_grbl.items()}


class Grbl:
    """Line-by-line Grbl link in machine axes: send, wait for ok, poll status."""

    def __init__(self, port: str | None = None, cfg: dict | None = None) -> None:
        cfg = cfg or load_bench()
        self.axes = AxisMap.from_config(cfg)
        self.max_feed = float(cfg["limits"]["max_feed"])
        self.max_step = float(cfg["limits"]["max_step"])
        port = port or cfg["serial"]["port"] or find_port()
        mark_parked(False, "Grbl port opened")      # until a clean quit says the nozzle is up again
        self.ser = serial.Serial(port, int(cfg["serial"]["baud"]), timeout=0.1)
        time.sleep(2.0)  # opening the port resets the Uno; wait for the Grbl banner
        self.ser.reset_input_buffer()
        self.reset_seen = False           # a Grbl start-up banner later on = the Uno was reset: position lost
        self.armed = False                # set by check_ready (the first banner after the port opened is normal)

    def _readline(self, deadline: float) -> str:
        while time.monotonic() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if line.startswith("Grbl ") and getattr(self, "armed", False):
                self.reset_seen = True
                raise GrblError("Grbl was reset (a power dip or the USB?): the position is lost")
            if line:
                return line
        raise TimeoutError("no reply from Grbl")

    def send(self, line: str, timeout: float = 5.0) -> list[str]:
        self.ser.write(line.encode() + b"\n")
        deadline = time.monotonic() + timeout
        replies = []
        while True:
            r = self._readline(deadline)
            if r == "ok":
                return replies
            if r.startswith("ALARM"):
                self.reset_seen = True            # an alarm stops the motion: the position is not sure any more
            if r.startswith(("error", "ALARM")):
                raise GrblError(f"{line!r} -> {r}")
            replies.append(r)

    def status(self) -> tuple[str, dict[str, float]]:
        """Grbl state and the machine-axis position."""
        self.ser.write(b"?")
        deadline = time.monotonic() + 2.0
        while True:
            r = self._readline(deadline)
            if r.startswith("<"):
                fields = r.strip("<>").split("|")
                mpos = next(f for f in fields if f.startswith("MPos:"))
                return fields[0], self.axes.machine_pos(np.array([float(v) for v in mpos[5:].split(",")]))

    def settings(self) -> dict[str, str]:
        out = {}
        for r in self.send("$$"):
            if r.startswith("$") and "=" in r:
                k, v = r.split("=", 1)
                out[k] = v
        return out

    def check_ready(self) -> None:
        """Refuse to run if Grbl is not Idle or homing is on (there are no limit switches)."""
        state, _ = self.status()
        if state != "Idle":
            raise GrblError(f"Grbl is {state}, not Idle. Fix that first (alarm? use the console).")
        if self.settings().get("$22") != "0":
            raise GrblError("Homing is on ($22=1) but there are no limit switches. Set $22=0 first.")
        self.armed = True

    def move_relative(self, axis: str, dist: float, feed: float) -> None:
        """Relative move of one machine axis. Returns only when the axis has stopped."""
        self.move_axes({axis: dist}, feed)

    def move_axes(self, dists: dict[str, float], feed: float) -> None:
        """Relative move of several machine axes together, e.g. {"X": 2, "Y": -1}. Waits until stopped."""
        dists = {a: d for a, d in dists.items() if abs(d) > 1e-6}
        if not dists:
            return
        for axis, dist in dists.items():
            if abs(dist) > self.max_step:
                raise GrblError(f"{axis} move of {dist} mm is bigger than limits.max_step ({self.max_step})")
        if feed > self.max_feed:
            raise GrblError(f"feed {feed} is above limits.max_feed ({self.max_feed})")
        travel_s = float(np.sqrt(sum(d * d for d in dists.values()))) / (feed / 60.0)
        words = " ".join(self.axes.word(a, d) for a, d in dists.items())
        _, before = self.status()
        self.send(f"G91 G1 {words} F{feed:.0f}")
        self.send("G4 P0.01", timeout=travel_s * 3 + 5)  # G4 answers ok only after the planner is empty
        self.send("G90")
        state, after = self.status()
        if state != "Idle":
            raise GrblError(f"state after move is {state}, expected Idle")
        for a, d in dists.items():                   # a reset in between would put Grbl's position back to 0
            if abs(after[a] - before[a] - d) > 0.01:
                self.reset_seen = True
                raise GrblError(f"{a} moved {after[a] - before[a]:+.3f} mm, not {d:+.3f}: Grbl was reset or lost "
                                "its position")

    def stop(self) -> None:
        """Feed hold, then soft reset (the position is lost after this)."""
        self.ser.write(b"!")
        time.sleep(0.2)
        self.ser.write(b"\x18")

    def close(self) -> None:
        self.ser.close()


class Grabber:
    """Keeps reading the camera in a thread, so the driver never holds old frames. If the camera drops off
    the USB (the head camera did, 2026-10-07), it is opened again as soon as it is back."""

    def __init__(self, device: str, width: int = 640, height: int = 400, rectify=None) -> None:
        """rectify: a function applied to every grey frame (tools/camera_rectify.py straightens the pictures
        of the tilted head camera with it). None = raw pictures."""
        self.device, self.size, self.rectify = device, (width, height), rectify
        self.controls: dict[str, float] = {}             # camera settings (exposure...), set again after a reconnect
        self.cap = self._open()
        if self.cap is None:
            sys.exit(f"Cannot open camera {device}")
        self.frames: deque[tuple[float, np.ndarray]] = deque(maxlen=30)
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        for name, value in getattr(self, "controls", {}).items():
            cap.set(getattr(cv2, f"CAP_PROP_{name}"), float(value))
        return cap

    def set_control(self, name: str, value: float) -> bool:
        """A V4L2 camera setting by its OpenCV name (AUTO_EXPOSURE, EXPOSURE, GAIN, BRIGHTNESS). Kept for reconnects."""
        self.controls[name] = float(value)
        return bool(self.cap is not None and self.cap.set(getattr(cv2, f"CAP_PROP_{name}"), float(value)))

    def get_control(self, name: str) -> float | None:
        return None if self.cap is None else float(self.cap.get(getattr(cv2, f"CAP_PROP_{name}")))

    def _run(self) -> None:
        fails = 0
        while self.running:
            ok, f = self.cap.read() if self.cap is not None else (False, None)
            if ok:
                fails = 0
                gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) if f.ndim == 3 else f
                if self.rectify is not None:
                    gray = self.rectify(gray)
                with self.lock:
                    self.frames.append((time.monotonic(), gray))
                continue
            fails += 1
            if fails < 5:
                time.sleep(0.02)
                continue
            if self.cap is not None:                      # gone: USB dropped. Reopen it when it is back
                self.cap.release()
                self.cap = None
                print("Camera lost (USB connection?): reconnecting...", flush=True)
            time.sleep(0.5)
            self.cap = self._open()
            if self.cap is not None:
                print("Camera back.", flush=True)
                fails = 0

    def latest(self) -> tuple[float, np.ndarray] | None:
        with self.lock:
            return self.frames[-1] if self.frames else None

    def frames_after(self, t: float, n: int, timeout: float = 5.0) -> list[np.ndarray]:
        """n frames that were read after time t. The first new frame is skipped (it may be old)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                new = [f for ts, f in self.frames if ts > t]
            if len(new) >= n + 1:
                return new[1:n + 1]
            time.sleep(0.02)
        raise TimeoutError("camera gives no frames")

    def close(self) -> None:
        self.running = False
        self.thread.join(timeout=2)
        if self.cap is not None:
            self.cap.release()


def find_port() -> str:
    ports = [p.device for p in list_ports.comports() if "USB" in p.device or "ACM" in p.device]
    if not ports:
        sys.exit("No USB serial port found. Is the Arduino plugged in?")
    return ports[0]


def sharpness(gray: np.ndarray) -> float:
    """Variance of the Laplacian: higher = sharper. Only for comparing frames of the same scene."""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())
