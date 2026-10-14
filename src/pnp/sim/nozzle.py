"""The simulated nozzle (docs/13 §4.12): what the needle does to the world.

- The tip is at the head's camera centre + the TRUE nozzle offset (the program must measure it). The needle is off
  the servo's axis (bench 2026-10-11: 1.56 mm), so the tip circles the axis when the servo turns.
- Z contact: the tip can't go below what is under it (bed, mark sheet, tape, a part, the board). It has a spring:
  up to `spring_mm` of press is taken by the spring, beyond that the Z motor loses steps (a crash, logged).
- Ink: every touch on the mark sheet leaves a dot (the nozzle offset and rotation calibration, docs/13 §4.6).
- Pump on with the tip on a part's body: the part is held. It turns with the servo about the servo's axis, and goes
  down where the pump stops: on what is under it (a board, the bed), or it falls when the tip is too high.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import cv2
import numpy as np

from .world import GREY, Layer, World, new_layer, strip_layer


@dataclass
class Placed:
    """A part the nozzle put down: where and how it lies (the truth, for tests and the camera)."""
    x: float
    y: float
    angle: float                     # the body's long side, degrees, CCW from +X
    z: float                         # the surface it lies on
    body: tuple[float, float]
    height: float
    look: str
    source: tuple[int, int]          # (slot, pocket index)
    uid: int = 0                     # its layer is placed_<uid>


@dataclass
class Held:
    rel: np.ndarray                  # part centre - servo axis, when it was picked (mm)
    angle: float                     # part angle when picked
    servo: float                     # servo angle when picked
    body: tuple[float, float]
    height: float
    look: str
    source: tuple[int, int]


@dataclass
class NozzleTruth:
    offset: tuple[float, float]      # the servo axis, from the camera centre (mm)
    runout: float = 0.0              # the tip's distance from the servo axis (mm)
    runout_phase: float = 0.0        # the tip's direction from the axis at servo angle 0 (degrees)
    spring: float = 2.0              # mm of press the spring takes
    log: list[str] = field(default_factory=list)


class SimNozzle:
    def __init__(self, grbl, world: World, truth: NozzleTruth, seed: int = 0, pump_flag: str = "flood",
                 valve_flag: str | None = None) -> None:
        self.g, self.w, self.t = grbl, world, truth
        self.pump_flag = pump_flag              # which controller output the pump is on (flood = M8, mist = M7)
        self.valve_flag = valve_flag            # the valve's output, when there is one: vacuum = pump AND valve
        self.pump_on = False
        self.held: Held | None = None
        self.placed: list[Placed] = []
        self.touching = False
        self.max_press = 0.0
        self.crashes = 0
        self.rng = random.Random(seed)
        self.mark_sheet: Layer | None = None
        self.marks: list[tuple[float, float]] = []
        self.uid = 0
        grbl.z_floor = self.z_floor
        grbl.hooks.append(self._event)

    # ---------------------------------------------------------------- geometry
    def axis_xy(self) -> np.ndarray:
        return np.array([self.g.phys["X"] + self.t.offset[0], self.g.phys["Y"] + self.t.offset[1]])

    def tip_xy(self) -> np.ndarray:
        a = math.radians(self.t.runout_phase + self.g.servo_angle)
        return self.axis_xy() + self.t.runout * np.array([math.cos(a), math.sin(a)])

    def surface_at(self, x: float, y: float) -> tuple[float, str, object]:
        """The highest thing under (x, y): (z, what, detail)."""
        best = (0.0, "bed", None)
        p = np.array([x, y])
        sheet = self.mark_sheet
        if sheet is not None and sheet in self.w.layers:
            h, w_ = sheet.gray.shape
            u, v = sheet.px(x, y)
            if 0 <= u < w_ and 0 <= v < h and sheet.alpha[int(v), int(u)] > 0.5:
                best = max(best, (sheet.z, "sheet", None), key=lambda b: b[0])
        for board, pose, top in list(getattr(self.w, "boards", {}).values()):
            if top in self.w.layers:
                outline = [pose.to_machine(board, q) for q in board.outline[0]]
                if cv2.pointPolygonTest(np.asarray(outline, np.float32).reshape(-1, 1, 2), (float(x), float(y)),
                                        False) >= 0 and pose.z_top > best[0]:
                    best = (pose.z_top, "board", None)
        for pl in self.placed:
            if _in_body(p, (pl.x, pl.y), pl.angle, pl.body) and pl.z + pl.height > best[0]:
                best = (pl.z + pl.height, "placed", pl)
        for s in getattr(self.w, "strips", {}).values():
            c, u, v = self.w.slot_frame(s.slot)
            c = c + v * s.v_off
            du, dv = float(np.dot(p - c, u)), float(np.dot(p - c, v))
            u0, u1 = s.first_u - 3.5, s.first_u + s.pitch * (len(s.pockets) - 0.5) + 1.5
            b0, b1 = s.band
            if not (u0 <= du <= u1 and b0 <= dv <= b1):
                continue
            z, what, det = s.z_top, "tape", None
            k = int(round((du - s.first_u) / s.pitch))
            if 0 <= k < len(s.pockets) and abs(dv) < 1.5 and abs(du - (s.first_u + k * s.pitch)) < 1.2:
                pk = s.pockets[k]
                centre = c + u * (s.first_u + k * s.pitch + pk.du) + v * pk.dv
                ang = _strip_part_angle(u, pk.turn)
                if pk.present and _in_body(p, centre, ang, s.package_body, shrink=0.0):
                    z, what, det = s.z_top, "part", (s, k, centre, ang)
                else:
                    z, what, det = s.z_top - s.part_height, "pocket", (s, k)
            if z > best[0] or what == "part":
                best = (z, what, det)
        return best

    def floor(self) -> float:
        """The lowest tip height before contact: the surface, raised by a held part's height."""
        x, y = self.tip_xy()
        z = self.surface_at(x, y)[0]
        return z + (self.held.height if self.held is not None else 0.0)

    def z_floor(self) -> float:
        """For the virtual Z motor (called with the controller's lock held): below this it loses steps."""
        return self.floor() - self.t.spring

    # ---------------------------------------------------------------- events from the controller
    def _event(self, kind: str, data: dict) -> None:
        if kind == "tick":
            self._contact()
        elif kind == "lost_z":
            self.crashes += 1
            self.t.log.append(f"CRASH: the tip was pressed {self.t.spring + data['mm']:.2f} mm into the surface")
        elif kind == "output":
            on = self._flag(data, self.pump_flag) and (self.valve_flag is None or self._flag(data, self.valve_flag))
            if on and not self.pump_on:
                self.pump_on = True
                self._try_pick()
            elif not on and self.pump_on:
                self.pump_on = False
                self._release()

    @staticmethod
    def _flag(data: dict, flag: str) -> bool:
        if flag.startswith("dout"):
            return bool((data.get("dout") or {}).get(int(flag[4:])))
        return bool(data.get(flag))

    def pressure(self) -> float:
        """The simulated vacuum sensor (kPa below air): open needle about 8, a part on it about 55, nothing on."""
        if not self.pump_on:
            return 0.0
        sealed = self.held is not None or self.touching              # a part, or the tip pressed on something
        return (55.0 if self.held is not None else 50.0 if sealed else 8.0) + self.rng.uniform(-2, 2)

    def _contact(self) -> None:
        fl = self.floor()
        z = self.g.phys["Z"]
        now = z <= fl + 0.02
        if now:
            self.max_press = max(self.max_press, fl - z)
            if z < fl - self.t.spring - 1e-6 and self.crashes < 1000:
                self.crashes += 1                                  # moved sideways into something with the tip low
                self.t.log.append(f"CRASH: the tip was dragged into something {fl - z:.2f} mm higher")
        if now and not self.touching:
            x, y = self.tip_xy()
            what = self.surface_at(x, y)[1]
            if what == "sheet" and self.held is None:
                self._ink(x, y)
            if self.pump_on:
                self._try_pick()
        self.touching = now

    def _ink(self, x: float, y: float) -> None:
        s = self.mark_sheet
        u, v = s.px(x, y)
        c = (int(round((u - 0.5) * 16)), int(round((v - 0.5) * 16)))
        cv2.circle(s.gray, c, int(round(0.3 * s.T * 16)), 25.0, -1, cv2.LINE_AA, shift=4)
        self.marks.append((float(x), float(y)))

    def _try_pick(self) -> None:
        if self.held is not None:
            return
        x, y = self.tip_xy()
        z, what, det = self.surface_at(x, y)
        if what != "part" or self.g.phys["Z"] > z + 0.05:
            return
        s, k, centre, ang = det
        if s.stick and s.stock > 0:
            s.stock -= 1                                          # the next part slides into the pick point
        else:
            s.pockets[k].present = False
        self.w.add(strip_layer(self.w, s))
        self.held = Held(np.asarray(centre, float) - self.axis_xy(), ang, self.g.servo_angle, s.package_body,
                         s.part_height, s.look, (s.slot, k))

    def _release(self) -> None:
        h = self.held
        if h is None:
            return
        self.held = None
        d = self.g.servo_angle - h.servo
        a = math.radians(d)
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        centre = self.axis_xy() + R @ h.rel
        under = self.surface_at(*centre)[0]
        gap = self.g.phys["Z"] - h.height - under
        if gap > 1.0:                                              # dropped from too high: it bounces somewhere
            centre = centre + np.array([self.rng.uniform(-0.8, 0.8), self.rng.uniform(-0.8, 0.8)])
            d += self.rng.uniform(-40, 40)
            self.t.log.append(f"DROPPED from {gap:.1f} mm above the surface")
        self.uid += 1
        self.placed.append(Placed(float(centre[0]), float(centre[1]), h.angle + d, under, h.body, h.height, h.look,
                                  h.source, self.uid))
        self.w.add(placed_layer(self.w, self.placed[-1], self.uid))

    # ---------------------------------------------------------------- props
    def lay_mark_sheet(self, x: float, y: float, thickness: float = 0.1, size=(60.0, 40.0)) -> Layer:
        L = new_layer("mark_sheet", thickness, x - size[0] / 2, x + size[0] / 2, y - size[1] / 2, y + size[1] / 2,
                      238.0, 1.0, self.w.T)
        self.mark_sheet = self.w.add(L)
        self.marks = []
        return L


def _strip_part_angle(u: np.ndarray, turn: float) -> float:
    return math.degrees(math.atan2(u[1], u[0])) + 90.0 + turn


def _in_body(p, centre, angle_deg: float, body, shrink: float = 0.0) -> bool:
    a = math.radians(angle_deg)
    d = np.asarray(p, float) - np.asarray(centre, float)
    along = d[0] * math.cos(a) + d[1] * math.sin(a)
    across = -d[0] * math.sin(a) + d[1] * math.cos(a)
    return abs(along) <= body[0] / 2 - shrink and abs(across) <= body[1] / 2 - shrink


def placed_layer(world: World, pl: Placed, n: int) -> Layer:
    r = max(pl.body) / 2 + 0.5
    L = new_layer(f"placed_{n}", pl.z + pl.height, pl.x - r, pl.x + r, pl.y - r, pl.y + r, 0.0, 0.0, world.T)
    a = math.radians(pl.angle)
    ax, nx = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    c = np.array([pl.x, pl.y])
    bl, bw = pl.body

    def rect(a0, a1):
        return [c + ax * p + nx * q for p, q in ((a0, -bw / 2), (a1, -bw / 2), (a1, bw / 2), (a0, bw / 2))]
    if pl.look == "resistor":
        L.poly(rect(-bl / 2, bl / 2), 210)
        L.poly(rect(-bl / 2 + 0.3, bl / 2 - 0.3), 40)
    else:
        L.poly(rect(-bl / 2, bl / 2), 200)
        L.poly(rect(-bl / 2 + 0.35, bl / 2 - 0.35), 105)
    return L


__all__ = ["GREY", "Held", "NozzleTruth", "Placed", "SimNozzle", "placed_layer"]
