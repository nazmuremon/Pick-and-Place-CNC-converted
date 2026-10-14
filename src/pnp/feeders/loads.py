"""Loads: which line goes into which tray, when a board has more lines than there are trays (docs/13 §4.4.1).

A board is split into loads of at most "reachable trays" lines ("load 1 of 4: put these 6 lines into trays 1-6"),
so more trays simply mean fewer loads. A line with more parts than one tray holds gets a second (spare) tray when the
load has one left; otherwise the job stops for a refill of that tray.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Line:
    key: str                         # value|package: the same key as a feeder line and a drawer line (§4.15)
    value: str
    package: str
    refs: list[str]
    pitch: float = 4.0
    tape_w: int = 8                  # the tape this line comes on: it needs a tray at least this wide

    @property
    def count(self) -> int:
        return len(self.refs)


@dataclass
class Load:
    index: int
    trays: dict[int, str] = field(default_factory=dict)      # tray number -> line key

    def lines(self) -> list[str]:
        return list(dict.fromkeys(self.trays.values()))


def line_key(value: str, package: str) -> str:
    return f"{value.strip()}|{package.strip()}"


def split(lines: list[Line], trays: list, capacity) -> list[Load]:
    """trays: the reachable Tray objects (in the order to fill). capacity(tray, line) -> parts per strip.
    Lines are kept in the order given (the planner sorts them by package, then value)."""
    if not trays:
        return []
    widest = max(int(getattr(t, "tape_w", 8)) for t in trays)
    lines = [ln for ln in lines if ln.tape_w <= widest]    # no tray for it at all: the planner says so
    loads: list[Load] = []
    todo = list(lines)
    while todo:
        load = Load(len(loads) + 1)
        free = list(trays)
        rest = []
        for i, ln in enumerate(todo):
            fits = sorted((t for t in free if int(getattr(t, "tape_w", 8)) >= ln.tape_w),
                          key=lambda t: int(getattr(t, "tape_w", 8)))        # the narrowest tray that fits
            if not fits:
                rest.append(ln)
                continue
            t = fits[0]
            free.remove(t)
            load.trays[t.number] = ln.key
            need = ln.count - capacity(t, ln)
            waiting = len(todo) - i - 1                    # lines after this one that still need a tray
            while need > 0 and len(free) > waiting:        # a spare tray only when it takes no line's place
                more = [x for x in free if int(getattr(x, "tape_w", 8)) >= ln.tape_w]
                if not more:
                    break
                free.remove(more[0])
                load.trays[more[0].number] = ln.key
                need -= capacity(more[0], ln)
        loads.append(load)
        todo = rest
    return loads
