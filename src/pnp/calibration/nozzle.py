"""The nozzle offset, the needle's runout and the rotation table from ink marks (docs/13 §4.6 steps 5 and 6).

Every mark is one touch of the needle on a sheet: where the head was (the camera centre, mm) and where the mark is
(measured by the camera). mark - head = the tip's offset from the camera centre for that servo angle.

The needle is not on the servo's axis (bench: 1.56 mm off), so the tip goes round a circle as the servo turns:
  tip(θ) = axis + r · (cos(φ + θ), sin(φ + θ))
The circle's centre is the axis offset, its radius the runout r, φ the tip's direction at servo angle 0. With r big
enough, the mark's direction from the axis also MEASURES the servo's real angle: the rotation table.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

import numpy as np

from ..vision.features import fit_circle

MIN_RUNOUT_FOR_ANGLES = 0.5      # mm: closer to the axis, a 0.03 mm mark error is more than 3° of angle


@dataclass
class Mark:
    servo_deg: float                 # the angle commanded (nominal or from the table)
    head: tuple[float, float]        # head (camera centre) when the tip touched, mm from HOME
    mark: tuple[float, float]        # the mark, mm from HOME
    pulse: int | None = None

    @property
    def offset(self) -> np.ndarray:
        return np.asarray(self.mark, float) - np.asarray(self.head, float)


@dataclass
class NozzleOffset:
    axis: tuple[float, float]        # the servo axis from the camera centre (mm)
    runout: float                    # the tip's distance from the axis (mm); 0 when it was not measured
    phase_deg: float                 # the tip's direction from the axis at servo angle 0
    rms: float                       # how well the marks fit the circle (mm)
    runout_known: bool
    marks: list[dict] = field(default_factory=list)

    def tip(self, servo_deg: float | None) -> np.ndarray:
        """The tip from the camera centre at this servo angle (the axis when the angle or the runout is unknown)."""
        a = np.asarray(self.axis, float)
        if not self.runout_known or servo_deg is None:
            return a
        t = math.radians(self.phase_deg + servo_deg)
        return a + self.runout * np.array([math.cos(t), math.sin(t)])

    def to_dict(self) -> dict:
        d = asdict(self)
        d["axis"] = [float(v) for v in self.axis]
        return d

    @classmethod
    def from_dict(cls, d: dict) -> NozzleOffset:
        return cls(tuple(d["axis"]), float(d["runout"]), float(d["phase_deg"]), float(d["rms"]),
                   bool(d["runout_known"]), list(d.get("marks", [])))


def fit_offset(marks: list[Mark]) -> NozzleOffset:
    """One mark: the tip offset at that angle (no runout). Three or more at angles ≥ 60° apart: the circle."""
    if not marks:
        raise ValueError("no marks")
    offs = np.array([m.offset for m in marks])
    angles = np.array([m.servo_deg for m in marks], float)
    rec = [{"servo_deg": m.servo_deg, "head": list(m.head), "mark": list(m.mark), "pulse": m.pulse} for m in marks]
    spread = float(np.ptp(angles)) if len(angles) > 1 else 0.0
    if len(marks) < 3 or spread < 60.0:
        c = offs.mean(axis=0)
        rms = float(np.sqrt(np.mean(np.sum((offs - c) ** 2, axis=1))))
        return NozzleOffset((float(c[0]), float(c[1])), 0.0, 0.0, rms, False, rec)
    c, r = fit_circle(offs)
    d = offs - c
    res = np.hypot(d[:, 0], d[:, 1]) - r
    ph = np.degrees(np.arctan2(d[:, 1], d[:, 0])) - angles
    phase = math.degrees(math.atan2(np.mean(np.sin(np.radians(ph))), np.mean(np.cos(np.radians(ph)))))
    return NozzleOffset((float(c[0]), float(c[1])), float(r), float(phase), float(np.sqrt(np.mean(res ** 2))), True,
                        rec)


def mark_angles(marks: list[Mark], off: NozzleOffset) -> np.ndarray:
    """The servo's REAL angle at every mark, from the mark's direction around the axis (degrees)."""
    out = []
    for m in marks:
        d = m.offset - np.asarray(off.axis, float)
        a = math.degrees(math.atan2(d[1], d[0])) - off.phase_deg
        out.append((a + 180.0) % 360.0 - 180.0)
    return np.array(out)


@dataclass
class RotationTable:
    pulses: list[int]
    angles: list[float]              # the real angle at each pulse (degrees, same zero as the offset's phase)
    deg_per_us: float
    rms_deg: float                   # how far the points are from a straight line
    accuracy_deg: float | None = None    # repeat: the same pulse stops up to this far apart (±)

    def pulse_for(self, deg: float) -> int:
        p, a = np.array(self.pulses, float), np.array(self.angles, float)
        order = np.argsort(a)
        if not a.min() - 1e-6 <= deg <= a.max() + 1e-6:
            raise ValueError(f"{deg:+.1f}° is outside the measured range {a.min():+.0f}° … {a.max():+.0f}°")
        return int(round(float(np.interp(deg, a[order], p[order]))))

    def angle_for(self, pulse: int) -> float:
        p, a = np.array(self.pulses, float), np.array(self.angles, float)
        order = np.argsort(p)
        return float(np.interp(pulse, p[order], a[order]))

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> RotationTable:
        return cls([int(v) for v in d["pulses"]], [float(v) for v in d["angles"]], float(d["deg_per_us"]),
                   float(d["rms_deg"]), d.get("accuracy_deg"))


def fit_table(marks: list[Mark], off: NozzleOffset) -> RotationTable:
    """Pulse -> real angle, from marks at several pulses (approached from the same side)."""
    if not off.runout_known or off.runout < MIN_RUNOUT_FOR_ANGLES:
        raise ValueError(f"The needle is only {off.runout:.2f} mm from the servo's axis: the marks can't show the "
                         "angle. Use the nominal range and check with the first part")
    if len(marks) < 3:
        raise ValueError("Need marks at 3 pulses or more")
    pulses = np.array([m.pulse for m in marks], float)
    ang = mark_angles(marks, off)
    order = np.argsort(pulses)
    pulses, ang = pulses[order], np.unwrap(np.radians(ang[order]))
    ang = np.degrees(ang)
    k, b = np.polyfit(pulses, ang, 1)
    res = ang - (k * pulses + b)
    return RotationTable([int(p) for p in pulses], [float(a) for a in ang], float(k), float(np.sqrt(np.mean(res ** 2))))


def repeat_spread(marks: list[Mark], off: NozzleOffset) -> float:
    """The same pulse, several times from the same side: how far apart the angles are (± degrees)."""
    a = np.unwrap(np.radians(mark_angles(marks, off)))
    a = np.degrees(a)
    return float(np.max(np.abs(a - a.mean()))) if len(a) else 0.0


def choose_angles(turn: float, rng: tuple[float, float], symmetric: bool) -> tuple[float, float, float]:
    """The servo angles for a pick and its place, and the turn really made.
    turn: how far the part must turn (degrees). symmetric: a 180° turn is the same part (2-pad chips): then the
    smallest of turn and turn ± 180 is used. The pick angle is free: the turn is centred in the range."""
    t = (turn + 180.0) % 360.0 - 180.0
    options = [t]
    if symmetric:
        options += [t - 180.0, t + 180.0]
    options = sorted((o for o in options if -360 < o < 360), key=abs)
    lo, hi = rng
    for o in options:
        if abs(o) <= hi - lo:
            s0 = max(lo - min(0.0, o), min(hi - max(0.0, o), -o / 2.0))   # centred, both ends inside the range
            return float(s0), float(s0 + o), float(o)
    raise ValueError(f"A {t:+.0f}° turn does not fit the rotation range {lo:+.0f}° … {hi:+.0f}°")
