"""The job plan (docs/13 §4.7): which parts the machine places, in which order, and why every other part is not
placed by it, before Start.

- Only this side's SMD parts the package library calls 'machine' (§4.4); by hand, bottom camera, THT, pad only and
  unknown parts go to the hand list with their reason (§4.15).
- A polar part needs its angle in the tape (tape_angle) or it is skipped ("measure it on the first part").
- A place the nozzle can't reach (the travel shifted by the nozzle offset) is skipped.
- A part already placed (by the machine or by hand, the shared placed state) is skipped.
- The rest is grouped by line (value + package) and loads (feeders/loads.py), and ordered by travel inside a line.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

import numpy as np

from ..feeders.loads import Line, Load, line_key, split


@dataclass
class PlanPart:
    ref: str
    value: str
    package: str
    line: str
    x: float                         # the place, mm from HOME (registered)
    y: float
    angle: float                     # the body's angle on the board, machine frame (degrees)
    height: float
    symmetric: bool                  # a 180° turn gives the same part (non-polar 2-pad)
    load: int = 1
    bottom: bool = False             # looked at from below before the place (Phase G)
    nozzle: str | None = None        # the nozzle it needs (Phase G: nozzle changes)
    copy: str = "1"                  # which board of a panel (Phase G)
    state: str = "planned"           # planned | picking | picked | placing | placed | verified | skipped | failed
    reason: str = ""
    attempts: int = 0
    tray: int | None = None
    pick: dict = field(default_factory=dict)      # where it was picked (for a held part after a stop)
    result: dict = field(default_factory=dict)    # error, angle error, photos

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass
class Plan:
    board: str
    side: str
    parts: list[PlanPart]
    skipped: list[dict]              # {ref, value, package, reason}
    lines: list[Line]
    loads: list[Load]
    estimate_s: float

    def summary(self) -> dict:
        reasons: dict[str, int] = {}
        for s in self.skipped:
            reasons[s["reason"]] = reasons.get(s["reason"], 0) + 1
        return {"board": self.board, "side": self.side, "parts": len(self.parts), "skipped": len(self.skipped),
                "skip_reasons": reasons, "lines": len(self.lines), "loads": len(self.loads),
                "estimate_min": round(self.estimate_s / 60, 1)}


def make_plan(board, classes: dict, lib, reg, tip_reach, placed, trays: list, capacity,
              seconds_per_part: float = 25.0, bottom: str = "none", nozzle_for=None, nozzle_now: str | None = None) -> Plan:
    """reg: one Registration, or a panel's [(copy, Registration)]; placed: a set of refs, or {copy: set}."""
    copies = reg if isinstance(reg, list) else [("1", reg)]
    placed_by = placed if isinstance(placed, dict) else {"1": set(placed)}
    parts_all, skipped_all = [], []
    for copy, rg in copies:
        p = _plan_one(board, classes, lib, rg, tip_reach, placed_by.get(copy, set()), bottom, nozzle_for, copy)
        parts_all += p[0]
        skipped_all += p[1]
    return _finish(board, copies[0][1].side, parts_all, skipped_all, lib, trays, capacity, seconds_per_part, nozzle_now)


def _plan_one(board, classes, lib, reg, tip_reach, placed, bottom, nozzle_for, copy):
    """bottom: "none" (no calibrated bottom camera), "needed" (only parts that need it), "always" (every part).
    With a bottom camera the "bottom_camera" parts (SOIC, SOP, ...) are machine parts too."""
    """tip_reach(xy) -> bool: the nozzle can put its tip there. trays: the reachable trays for the load split.
    capacity(tray, line) -> parts per strip."""
    side = reg.side
    parts, skipped = [], []
    tag = "" if copy == "1" else f" (copy {copy})"
    for c in board.components:
        if c.side != side:
            continue                                            # the other side: another job
        cls = classes.get(c.ref, {})
        why = None
        if c.ref in placed:
            why = "already placed"
        elif cls.get("class") == "bottom_camera" and bottom != "none":
            pk = lib.packages.get(c.package)
            if pk is not None and pk.polar and pk.tape_angle is None:
                why = f"{c.package} is polar and its angle in the tape is not known yet (measure it on the first part)"
            elif not tip_reach(reg.to_machine(board, (c.x, c.y))):
                why = "outside the nozzle's reach"
        elif cls.get("class") != "machine":
            why = cls.get("why") or cls.get("class", "unknown")
        else:
            pk = lib.packages.get(c.package)
            if pk is None:
                why = "package not in the library"
            elif pk.polar and pk.tape_angle is None:
                why = f"{c.package} is polar and its angle in the tape is not known yet (measure it on the first part)"
            else:
                xy = reg.to_machine(board, (c.x, c.y))
                if not tip_reach(xy):
                    why = "outside the nozzle's reach"
                elif nozzle_for is not None and nozzle_for(c.package) is None:
                    why = f"no nozzle of the profile lists {c.package}"
        if why is not None:
            if cls.get("class") not in ("dnp",) or c.ref in placed:
                skipped.append({"ref": c.ref + tag, "value": c.value, "package": c.package, "reason": why,
                                "class": cls.get("class", ""), "copy": copy})
            continue
        pk = lib.packages[c.package]
        xy = reg.to_machine(board, (c.x, c.y))
        needs = cls.get("class") == "bottom_camera"
        parts.append(PlanPart(c.ref, c.value, c.package, line_key(c.value, c.package), float(xy[0]), float(xy[1]),
                              float(reg.angle_deg + c.rotation), float(c.height or pk.height or 1.0),
                              bool(not pk.polar and (pk.pads or 2) == 2),
                              bottom=bottom == "always" or (bottom == "needed" and needs),
                              nozzle=None if nozzle_for is None else nozzle_for(c.package), copy=copy))
    return parts, skipped


def _finish(board, side, parts, skipped, lib, trays, capacity, seconds_per_part, nozzle_now) -> Plan:
    # lines: the same key as a feeder line; sorted by package, then value (similar heights together)
    by_line: dict[str, Line] = {}
    for p in parts:
        pk = lib.packages[p.package]
        ln = by_line.setdefault(p.line, Line(p.line, p.value, p.package, [],
                                             float((pk.tape or {}).get("pitch", 4.0)),
                                             int((pk.tape or {}).get("width", 8))))
        ln.refs.append(p.ref)
    lines = sorted(by_line.values(), key=lambda ln: (ln.package, ln.value))
    loads = split(lines, trays, capacity)
    where = {}
    for ld in loads:
        for tray, key in ld.trays.items():
            where.setdefault(key, ld.index)
    ordered: list[PlanPart] = []
    for ld in loads:
        in_load: list[PlanPart] = []
        for key in ld.lines():
            todo = [p for p in parts if p.line == key]
            for p in todo:
                p.load = ld.index
            in_load += _nearest_first(todo)
        # inside a load: the nozzle on the head first, then one change per other nozzle (stable: travel order kept)
        in_load.sort(key=lambda p: (p.nozzle != nozzle_now, p.nozzle or ""))
        ordered += in_load
    unplanned = [p for p in parts if p.line not in where]
    widths = {ln.key: ln.tape_w for ln in lines}
    for p in unplanned:
        w = widths.get(p.line, 8)
        why = (f"needs a tray for {w} mm tape (none is reachable)" if trays and w > max(int(getattr(t, "tape_w", 8))
                                                                                    for t in trays)
               else "no reachable tray")
        skipped.append({"ref": p.ref, "value": p.value, "package": p.package, "reason": why, "class": "machine"})
    est = seconds_per_part * len(ordered)
    return Plan(board.name, side, ordered, skipped, lines, loads, est)


def _nearest_first(parts: list[PlanPart]) -> list[PlanPart]:
    """Places in travel order: each time the nearest one to the last (from the tray side: the largest Y first)."""
    if not parts:
        return []
    left = sorted(parts, key=lambda p: -p.y)
    out = [left.pop(0)]
    while left:
        last = out[-1]
        i = min(range(len(left)), key=lambda k: math.hypot(left[k].x - last.x, left[k].y - last.y))
        out.append(left.pop(i))
    return out


def tip_reach_fn(travel: dict, tip_offset) -> callable:
    X, Y = travel["X"], travel["Y"]
    off = np.asarray(tip_offset, float)

    def ok(xy) -> bool:
        if X is None or Y is None:
            return False
        p = np.asarray(xy, float)
        h = p - off
        # the tip must get there, and the camera too (the photos before and after, the place check)
        return X[0] <= h[0] <= X[1] and Y[0] <= h[1] <= Y[1] and X[0] <= p[0] <= X[1] and Y[0] <= p[1] <= Y[1]
    return ok
