"""Travel limits and move caps from the profile (docs/13 §4.8). Every move is checked here before it is sent.

Positions are in the travel frame of the profile: 'home' = mm from the HOME marker (X right, Y back), Z = the
nozzle tip above the Z reference surface. The camera centre is the head's reference point; the nozzle reach is
the travel shifted by the nozzle offset (Phase D) and is never stored on its own.
"""
from __future__ import annotations

import math

from ..profile.schema import Profile
from .state import Refusal

EPS = 0.01          # mm: a target on the edge of the travel (rounded) is still inside


class Limits:
    def __init__(self, p: Profile, measured: dict | None = None) -> None:
        """measured: the travel from Setup step 2 ({"x": [lo, hi], "y": [lo, hi]}); it replaces the profile's."""
        m = p.motion
        self.travel = {"X": m.travel.x, "Y": m.travel.y, "Z": m.travel.z}
        self.source = "profile"
        if measured:
            for k in ("x", "y", "z"):
                if measured.get(k):
                    self.travel[k.upper()] = tuple(measured[k])
            self.source = "measured"
        self.max_step = m.max_step_mm
        self.max_feed_xy, self.max_feed_z = m.max_feed_xy, m.max_feed_z
        self.slow_feed = m.jog_feed_not_homed

    @property
    def xy_known(self) -> bool:
        return self.travel["X"] is not None and self.travel["Y"] is not None

    def inside(self, axis: str, value: float) -> bool:
        r = self.travel[axis]
        return r is None or r[0] - EPS <= value <= r[1] + EPS

    def check_step(self, delta: dict[str, float]) -> Refusal | None:
        for a, d in delta.items():
            if abs(d) > self.max_step + 1e-9:
                return Refusal(f"{a} {d:+.2f} mm is longer than the largest single move ({self.max_step:g} mm)",
                               "Use a shorter step")
        return None

    def check_target(self, target: dict[str, float], what: str = "This move") -> Refusal | None:
        """target: positions in the travel frame (only the axes that move)."""
        for a, v in target.items():
            r = self.travel.get(a)
            if r is not None and not self.inside(a, v):
                return Refusal(f"{what} goes to {a} {v:.2f}, outside the travel {r[0]:g} … {r[1]:g} mm",
                               "Stay inside the travel (bed map), or measure the travel again (Setup, step 2)")
        return None

    def check_goto(self, target: dict[str, float]) -> Refusal | None:
        if not self.xy_known:
            return Refusal("The X / Y travel is not known (TBD in the profile)",
                           "Measure the travel (Setup, step 2): until then only jogs")
        return self.check_target(target, "Go-to")

    def feed(self, delta: dict[str, float], wanted: float | None, homed: bool) -> float:
        """The feed for a move: never above the profile's caps; slow before Home."""
        xy = any(a in delta for a in "XY") and any(abs(delta.get(a, 0)) > 0 for a in "XY")
        cap = self.max_feed_xy if xy else self.max_feed_z
        if xy and not homed:
            cap = min(cap, self.slow_feed)
        f = cap if wanted is None else min(float(wanted), cap)
        return max(1.0, f)

    @staticmethod
    def travel_time(delta: dict[str, float], feed: float) -> float:
        return math.sqrt(sum(d * d for d in delta.values())) / (feed / 60.0)
