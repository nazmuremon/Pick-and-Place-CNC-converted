"""Machine axes (X right, Y back, Z up) ↔ controller letters and signs (the profile's axis_map).

On the bench the motors sit on other drivers than their names say: machine X = Grbl −Y, and so on. Everything
above this module uses machine axes only; only the controller driver turns them into letters.
"""
from __future__ import annotations

from dataclasses import dataclass

from ..profile.schema import Profile

MACHINE_AXES = ("X", "Y", "Z")
LETTERS = "XYZABC"


@dataclass(frozen=True)
class AxisMap:
    to_ctrl: dict[str, tuple[str, int]]          # machine axis -> (controller letter, sign)

    @classmethod
    def from_profile(cls, p: Profile) -> AxisMap:
        return cls({a: (t.letter, int(t.sign)) for a, t in p.controller.axis_map.items()})

    def word(self, axis: str, value: float) -> str:
        """G-code word for a machine-axis value (a distance or a controller-frame position): Z +2 -> 'X-2.000'."""
        letter, sign = self.to_ctrl[axis]
        return f"{letter}{sign * value:.3f}"

    def words(self, values: dict[str, float]) -> str:
        return " ".join(self.word(a, v) for a, v in values.items() if a in self.to_ctrl)

    def to_machine(self, ctrl: dict[str, float]) -> dict[str, float]:
        """Controller positions by letter -> machine axes (the controller frame, mapped)."""
        return {a: sign * ctrl[letter] for a, (letter, sign) in self.to_ctrl.items() if letter in ctrl}

    def to_controller(self, machine: dict[str, float]) -> dict[str, float]:
        out = {}
        for a, v in machine.items():
            letter, sign = self.to_ctrl[a]
            out[letter] = sign * v
        return out

    def letter(self, axis: str) -> str:
        return self.to_ctrl[axis][0]
