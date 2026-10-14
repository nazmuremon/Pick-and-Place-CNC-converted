"""BOM lines for the parts store and the hand-assembly view (docs/13 §4.15), from the one Board model.

A drawer line is "value + footprint", as in Solder Helper (so the drawers come out the same as on the old page);
it also carries the feeder line key "value|package" of the machine (feeders/loads.py), so a scanned bag can say
"tray 4" or "drawer 7". SMD / THT is decided by the pads, never by a tool's flag.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

CATS = [("R", "Resistors"), ("C", "Capacitors"), ("L", "Inductors & fuses"), ("D", "Diodes & LEDs"),
        ("Q", "Transistors"), ("X", "Crystals"), ("U", "ICs & modules"), ("J", "Connectors"), ("M", "Switches & misc")]
CAT_INDEX = {k: i for i, (k, _) in enumerate(CATS)}


def cat_of(ref: str) -> str:
    pre = re.sub(r"\d+$", "", ref).upper()
    if pre == "R":
        return "R"
    if pre == "C":
        return "C"
    if pre in ("L", "F", "FB"):
        return "L"
    if pre in ("D", "LED", "IR"):
        return "D"
    if pre == "Q":
        return "Q"
    if pre in ("X", "Y"):
        return "X"
    if pre in ("U", "IC"):
        return "U"
    if pre in ("J", "CN", "P"):
        return "J"
    return "M"


def value_num(v: str) -> float:
    m = re.match(r"^([\d.]+)\s*([pnuµmkKMR]?)", v or "")
    if not m:
        return 0.0
    try:
        x = float(m.group(1))
    except ValueError:
        return 0.0
    return x * {"p": 1e-12, "n": 1e-9, "u": 1e-6, "µ": 1e-6, "m": 1e-3, "k": 1e3, "K": 1e3, "M": 1e6, "R": 1}.get(
        m.group(2), 1)


def nat_key(s: str):
    """Like JavaScript's localeCompare(numeric, base): numbers by value, letters without case."""
    return [(0, int(t), "") if t.isdigit() else (1, 0, t.lower()) for t in re.findall(r"\d+|\D+", s or "")]


@dataclass
class StoreLine:
    key: str                         # value + "\0" + footprint (the drawer line)
    value: str
    footprint: str
    feeder_key: str                  # value|package (the machine's line); "" when it has no package
    cat: str
    th: bool
    refs: list[str] = field(default_factory=list)
    lcsc: set[str] = field(default_factory=set)
    mpn: set[str] = field(default_factory=set)

    def as_dict(self) -> dict:
        return {"key": self.key, "value": self.value, "footprint": self.footprint, "feeder_key": self.feeder_key,
                "cat": self.cat, "th": self.th, "refs": self.refs, "count": len(self.refs),
                "lcsc": sorted(self.lcsc), "mpn": sorted(self.mpn)}


def in_bom(c) -> bool:
    return bool(c.pads) and not c.dnp and "exclude_from_bom" not in c.source_flags.get("kicad_attr", [])


def board_lines(board) -> list[StoreLine]:
    """The board's lines in the old page's order: by kind (R, C, L, ...), SMD before THT, then by value."""
    m: dict[str, StoreLine] = {}
    for c in board.components:
        if not in_bom(c):
            continue
        key = f"{c.value}\0{c.footprint}"
        ln = m.get(key)
        th = any(p.through_hole for p in c.pads)
        if ln is None:
            ln = m[key] = StoreLine(key, c.value, c.footprint, f"{c.value}|{c.package}" if c.package else "",
                                    "M", False)
        ln.refs.append(c.ref)
        ln.th = ln.th or th
        for code in re.split(r"[\s,;]+", c.lcsc or ""):
            if code:
                ln.lcsc.add(code.upper())
        if c.mpn:
            ln.mpn.add(norm_mpn(c.mpn))
    out = list(m.values())
    for ln in out:
        ln.refs.sort(key=nat_key)
        ln.cat = cat_of(ln.refs[0])
    out.sort(key=lambda ln: (CAT_INDEX[ln.cat], ln.th, value_num(ln.value), nat_key(ln.value), nat_key(ln.footprint)))
    return out


def norm_mpn(x: str) -> str:
    return re.sub(r"[^A-Z0-9]", "", (x or "").upper())
