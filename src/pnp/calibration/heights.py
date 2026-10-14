"""Heights are derived, never typed (docs/13 §4.6 step 7, bug 5.13). Every Z is the nozzle tip's height above the
Z reference surface (the bed label top, D23):

  pick  = the part's top in its tray − press      (part top = tape floor + part height)
  place = board top + part height − press         (board top = the board support + the board's thickness)
  mark  = the mark sheet's top − press
  moving = the highest surface the head passes over + clearance

press and clearance come from the profile (the nozzle's spring, docs/08), the part height from the package library,
the tape floor from bed.yaml (feed_tray.tape_floor_mm: 0 on the paper labels, 6.5 in the printed holders).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class HeightRules:
    press: float                     # mm below the touch
    clearance: float                 # mm above the highest thing passed over
    tape_floor: float                # the tape's floor above the label surface
    board_top: float | None          # the board's top above the label surface (None: no board)
    tallest_part: float              # the tallest part that may be on the nozzle or on the board

    def pick(self, part_height: float) -> float:
        return self.tape_floor + part_height - self.press

    def place(self, part_height: float) -> float:
        if self.board_top is None:
            raise ValueError("No board: import it and set its thickness (Board tab)")
        return self.board_top + part_height - self.press

    def mark(self, sheet_top: float) -> float:
        return sheet_top - self.press

    def moving(self) -> float:
        """High enough to carry the tallest part over the tallest thing in a tray or on the board."""
        tops = [self.tape_floor + self.tallest_part]
        if self.board_top is not None:
            tops.append(self.board_top + self.tallest_part)
        return max(tops) + self.tallest_part + self.clearance

    def as_dict(self, part_height: float | None = None) -> dict:
        d = {"press": self.press, "clearance": self.clearance, "tape_floor": self.tape_floor,
             "board_top": self.board_top, "tallest_part": self.tallest_part, "moving": round(self.moving(), 3)}
        if part_height is not None:
            d["pick"] = round(self.pick(part_height), 3)
            if self.board_top is not None:
                d["place"] = round(self.place(part_height), 3)
        return d
