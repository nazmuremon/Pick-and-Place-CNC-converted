"""The package library (docs/13 §4.4): footprint name -> package, and what the machine can do with a part.

Matching is by package, never by the footprint's prefix: a capacitor on a `D_0805_2012Metric` footprint is an
0805 (CLAUDE.md). Order: the user's aliases, the aliases and patterns of the library, the metric chip code, the
imperial chip code, the LCSC / EasyEDA name style.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from ..board.model import Board, Component

METRIC = {"0603": "0201", "1005": "0402", "1608": "0603", "2012": "0805", "3216": "1206", "3225": "1210",
          "4532": "1812", "5025": "2010", "6332": "2512"}
IMPERIAL = set(METRIC.values())
LCSC = re.compile(r"^(?P<base>[A-Za-z0-9-]+?)_L(?P<l>\d+(?:\.\d+)?)-W(?P<w>\d+(?:\.\d+)?)")
PREFIX = re.compile(r"^(?:[A-Za-z_]+:)?(?:D|Q|U|LED|CP|C|R|L|F)_", re.I)

CLASSES = {
    "machine": "placed by the machine",
    "bottom_camera": "needs the bottom camera (its angle on the nozzle): by hand for now",
    "by_hand": "placed by hand",
    "tht": "through-hole: soldered by hand (THT pass)",
    "pad_only": "one pad (a wire or test point): not a part",
    "mechanical": "no copper (a mounting hole)",
    "dnp": "do not populate",
    "unknown": "not in the package library: add it (config/packages.user.yaml)",
}


@dataclass
class Package:
    id: str
    kind: str
    body: tuple[float, float]
    height: float | None
    pads: int | None
    polar: bool
    pickable: str
    tape: dict | None = None
    tape_angle: float | None = None
    nozzle: str | None = None
    aliases: list[str] = field(default_factory=list)
    patterns: list[str] = field(default_factory=list)


def _pickable(v) -> str:
    """YAML reads a bare yes / no as true / false."""
    if v is True:
        return "yes"
    if v is False:
        return "by_hand"
    v = str(v)
    if v not in ("yes", "bottom_camera", "by_hand"):
        raise ValueError(f"pickable must be yes, bottom_camera or by_hand, not {v!r}")
    return v


@dataclass
class Match:
    package: Package | None
    how: str                         # alias, pattern, metric, imperial, lcsc, none


class Library:
    def __init__(self, shipped: dict, user: dict | None = None) -> None:
        self.packages: dict[str, Package] = {}
        base = shipped.get("packages") or {}
        for src in (base, (user or {}).get("packages") or {}):
            for pid, d in src.items():
                d = dict(d or {})
                if src is not base and pid in base:                 # a user entry changes only the fields it names
                    d = {**(base[pid] or {}), **d}
                self.packages[str(pid)] = Package(
                    id=str(pid), kind=d.get("kind", "other"), body=tuple(d.get("body", (0.0, 0.0))),
                    height=d.get("height"), pads=d.get("pads"), polar=bool(d.get("polar", True)),
                    pickable=_pickable(d.get("pickable", "by_hand")), tape=d.get("tape"), tape_angle=d.get("tape_angle"),
                    nozzle=d.get("nozzle"), aliases=[str(a) for a in d.get("aliases", [])],
                    patterns=list(d.get("patterns", [])))
        self.user_aliases = {str(k).lower(): str(v) for k, v in ((user or {}).get("aliases") or {}).items()}
        bad = [v for v in self.user_aliases.values() if v not in self.packages]
        if bad:
            raise ValueError(f"packages.user.yaml: aliases to unknown packages: {', '.join(sorted(set(bad)))}")
        self._alias = {a.lower(): p for p in self.packages.values() for a in p.aliases + [p.id]}
        self._patterns = [(re.compile(rx, re.I), p) for p in self.packages.values() for rx in p.patterns]

    @classmethod
    def load(cls, cfg_dir: Path) -> Library:
        f = cfg_dir / "packages.yaml"
        if not f.is_file():                                   # an installed program: the copy shipped with it
            f = Path(__file__).resolve().parents[1] / "data" / "packages.yaml"
        shipped = yaml.safe_load(f.read_text()) or {}
        user_file = cfg_dir / "packages.user.yaml"
        user = yaml.safe_load(user_file.read_text()) if user_file.is_file() else {}
        return cls(shipped, user)

    def identify(self, footprint: str, hint: str = "") -> Match:
        for name in [n for n in (footprint, hint) if n]:
            base = name.split(":")[-1].strip()
            low = base.lower()
            if low in self.user_aliases:
                return Match(self.packages[self.user_aliases[low]], "alias")
            if low in self._alias:
                return Match(self._alias[low], "alias")
            for rx, p in self._patterns:
                if rx.search(base):
                    return Match(p, "pattern")
            m = re.search(r"(\d{4})Metric", base)
            if m and METRIC.get(m.group(1)) in self.packages:
                return Match(self.packages[METRIC[m.group(1)]], "metric")
            stripped = PREFIX.sub("", base)
            m = re.match(r"^(?:CAPC|RESC|INDC|LEDC|C|R|L)?(\d{4})(?:$|[_-])", stripped, re.I)
            if m and m.group(1) in IMPERIAL and m.group(1) in self.packages:
                return Match(self.packages[m.group(1)], "imperial")
            m = LCSC.match(stripped)
            if m:
                cand = m.group("base").upper()
                for rx, p in self._patterns:
                    if rx.search(cand):
                        return Match(p, "lcsc")
        return Match(None, "none")


def classify(c: Component, m: Match) -> tuple[str, str]:
    """What happens to this part: (class, reason in plain words)."""
    if c.dnp:
        return "dnp", CLASSES["dnp"]
    if c.pads and not any(p.plated for p in c.pads):
        return "mechanical", CLASSES["mechanical"]
    if c.tht or (not c.pads and c.smd_hint is False):
        return "tht", CLASSES["tht"]
    if not c.smd:
        return "unknown", "no pads and no mount type in the files: add the copper layers or the KiCad file"
    if len(c.pads) == 1:
        return "pad_only", CLASSES["pad_only"]
    p = m.package
    if p is None:
        return "unknown", CLASSES["unknown"]
    if p.pads and c.pads and len(c.pads) != p.pads:
        return "by_hand", f"{len(c.pads)} pads, but {p.id} has {p.pads}: check the package (by hand until then)"
    if p.pickable == "yes":
        return "machine", f"{p.id}: {CLASSES['machine']}"
    if p.pickable == "bottom_camera":
        return "bottom_camera", f"{p.id}: {CLASSES['bottom_camera']}"
    return "by_hand", f"{p.id}: {CLASSES['by_hand']}"


def annotate(board: Board, lib: Library) -> dict[str, dict]:
    """Set every component's package and height (when the files have none); return ref -> class and reason."""
    out = {}
    for c in board.components:
        m = lib.identify(c.footprint, c.source_flags.get("x3_package", ""))
        if m.package is not None:
            c.package = m.package.id
            if c.height is None:
                c.height = m.package.height
        cls, why = classify(c, m)
        out[c.ref] = {"class": cls, "why": why, "package": c.package, "how": m.how}
    return out
