"""The feeder bank: the trays on the bed as a LIST of any length (docs/13 §4.4.1, bug 5.21).

`config/feeders.yaml` holds one entry per tray: its number (1-99), marker id, type, tape, zone, the tape floor and
where it was put (the plan; the bed survey measures the real place). The first time, the list is made from
bed.yaml (the paper slot labels: slot n = marker 40 + n − 1). Nothing in the program assumes 5, 6 or 8 trays, or
that tray n has marker 40 + n − 1.

Marker ids: HOME 49, PCB corners 30-33, trays 40-47 as today; new trays take free ids (40-47 first, then 0-19,
24-29, 34-39, 48; 20-23 belong to the smd_identify demo sheet). Past 50 markers the bed switches from DICT_4X4_50 to
DICT_4X4_250, whose first 50 markers are the same, so every label and tray printed so far stays valid.

What is loaded in each tray (the line, the confirmed template, the used pockets) is the feeder STATE, kept in the
data folder (feeder_state.json), not here.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml

HOLE_TO_PART = {8: 3.5, 12: 5.5, 16: 7.5, 24: 11.5}      # EIA-481: sprocket hole line -> parts' centre line
FREE_ORDER = list(range(40, 48)) + list(range(0, 20)) + list(range(24, 30)) + list(range(34, 40)) + [48]
RESERVED = {49, 30, 31, 32, 33, 20, 21, 22, 23}
TYPES = ("paper_label", "holder", "matrix", "stick")   # matrix: a grid of pockets; stick: a tube feeding one pick point


@dataclass
class Tray:
    number: int
    marker_id: int
    type: str = "paper_label"        # paper_label (a printed slot label) | holder (a printed holder, hardware/tray)
    tape_w: int = 8
    kind: str = "paper"              # paper | embossed
    zone_mm: float = 60.0            # how long the part zone is (17 mm from the marker on)
    tape_floor_mm: float = 0.0       # the tape's floor above the label surface (holders: 6.5, bed.yaml)
    bank: str = "back"               # a name for the row it sits in
    plan_xy: list[float] | None = None    # where it was put, mm from HOME (the survey measures it)
    plan_dir: list[float] = field(default_factory=lambda: [0.0, -1.0])   # from the marker toward the zone
    # matrix / stick (Phase G): the pockets by their place from the marker (u along, v across, mm)
    rows: int = 1
    cols: int = 0                    # 0 = a strip (pockets found along the zone at the tape's pitch)
    pitch_u: float = 4.0
    pitch_v: float = 4.0
    first_u: float = 22.0
    first_v: float = 0.0
    top_mm: float | None = None      # matrix / stick: its top (where its marker is) above the label surface

    @property
    def name(self) -> str:
        return f"slot_{self.number}"

    @property
    def marker_z(self) -> float:
        """The marker's height above the label surface: a paper label lies on the bar; a printed holder carries
        its marker on its top, 2.15 mm above the tape floor (traygen: tape top + lip gap + lip)."""
        if self.type in ("matrix", "stick"):
            return float(self.top_mm or 0.0)
        return 0.0 if self.type == "paper_label" else self.tape_floor_mm + 2.15

    @property
    def zone_u(self) -> tuple[float, float]:
        if self.type in ("matrix", "stick"):
            return (self.first_u - self.pitch_u / 2, self.first_u + (max(self.cols, 1) - 0.5) * self.pitch_u)
        return (17.0, 17.0 + self.zone_mm)

    @property
    def zone_v(self) -> tuple[float, float]:
        """Across the tray: the strip's tape (±8 mm), or a matrix tray's rows."""
        if self.type == "matrix":
            return (self.first_v - self.pitch_v / 2 - 1.0, self.first_v + (max(self.rows, 1) - 0.5) * self.pitch_v + 1.0)
        if self.type == "stick":
            return (self.first_v - 3.0, self.first_v + 3.0)
        return (-8.0, 8.0)

    def cells(self) -> list[tuple[int, list[float], float]]:
        """Matrix / stick: (row, the pockets' u, the row's v). Empty for a strip."""
        if self.type not in ("matrix", "stick"):
            return []
        n = 1 if self.type == "stick" else max(self.cols, 1)
        return [(r, [self.first_u + k * self.pitch_u for k in range(n)], self.first_v + r * self.pitch_v)
                for r in range(1 if self.type == "stick" else max(self.rows, 1))]

    @property
    def hole_row(self) -> float:
        return -HOLE_TO_PART[int(self.tape_w)]

    def capacity(self, pitch: float) -> int:
        if self.type == "stick":
            return 10 ** 6                                   # a tube refills its pick point: no load limit
        if self.type == "matrix":
            return max(self.rows, 1) * max(self.cols, 1)
        return int(self.zone_mm // pitch)


class FeederBank:
    def __init__(self, cfg_dir: Path, bed: dict | None) -> None:
        self.path = cfg_dir / "feeders.yaml"
        self.bed = bed or {}
        self.dictionary = (self.bed.get("markers") or {}).get("dictionary", "DICT_4X4_50")
        self.trays: dict[int, Tray] = {}
        self.migrated = False
        self.load()

    # ---------------------------------------------------------------- file
    def load(self) -> None:
        if self.path.is_file():
            d = yaml.safe_load(self.path.read_text()) or {}
            self.dictionary = d.get("dictionary", self.dictionary)
            self.trays = {int(t["number"]): Tray(**t) for t in d.get("trays", [])}
        else:
            self.trays = self._from_bed()
            self.migrated = bool(self.trays)

    def _from_bed(self) -> dict[int, Tray]:
        mk, ft, lay = self.bed.get("markers") or {}, self.bed.get("feed_tray") or {}, self.bed.get("layout") or {}
        ids = list(mk.get("slot_ids") or [])
        floor = float(ft.get("tape_floor_mm") or 0.0)
        out = {}
        for n in ft.get("slot_numbers") or []:
            n = int(n)
            if n - 1 >= len(ids):
                continue
            xy = (lay.get("slot_marker_xy") or {}).get(n)
            out[n] = Tray(n, int(ids[n - 1]), "paper_label" if floor == 0 else "holder", 8, "paper",
                          float(ft.get("zone_length_mm", 60.0)), floor, "back",
                          None if xy is None else [float(xy[0]), float(xy[1])],
                          [float(v) for v in lay.get("slot_direction", [0, -1])])
        return out

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        head = ("# The trays on the bed (docs/13 §4.4.1), written by the program: use the Feeders tab (Add tray,\n"
                "# Remove tray). Positions are the plan; the bed survey measures the real places.\n")
        tmp.write_text(head + yaml.safe_dump({"dictionary": self.dictionary,
                                              "trays": [asdict(t) for t in sorted(self.trays.values(),
                                                                                   key=lambda t: t.number)]},
                                             sort_keys=False, default_flow_style=None))
        os.replace(tmp, self.path)

    # ---------------------------------------------------------------- markers
    def used_ids(self) -> set[int]:
        mk = self.bed.get("markers") or {}
        ids = {int(mk.get("home_id", 49))} | {int(i) for i in mk.get("pcb_corner_ids", [])}
        return ids | {t.marker_id for t in self.trays.values()}

    def next_marker(self, number: int) -> tuple[int, str]:
        """The marker for a new tray: 40 + n − 1 when free (like the paper labels), else the next free id.
        Returns (id, dictionary): past the 50 markers of DICT_4X4_50, DICT_4X4_250."""
        used = self.used_ids() | RESERVED
        if 1 <= number <= 8 and 40 + number - 1 not in used:
            return 40 + number - 1, self.dictionary
        for i in FREE_ORDER:
            if i not in used:
                return i, self.dictionary
        for i in range(50, 250):
            if i not in used:
                return i, "DICT_4X4_250" if self.dictionary == "DICT_4X4_50" else self.dictionary
        raise ValueError("No free marker id left")

    def names(self) -> dict[int, str]:
        """Marker id -> label name, for the marker finder."""
        return {t.marker_id: t.name for t in self.trays.values()}

    # ---------------------------------------------------------------- add / remove
    def add(self, number: int, tape_w: int = 8, kind: str = "paper", zone_mm: float = 60.0, type_: str = "holder",
            plan_xy=None, plan_dir=(0.0, -1.0), bank: str = "back", tape_floor_mm: float | None = None,
            **grid) -> Tray:
        """grid (matrix / stick): rows, cols, pitch_u, pitch_v, first_u, first_v, top_mm."""
        if not 1 <= int(number) <= 99:
            raise ValueError("Tray numbers are 1-99")
        if int(number) in self.trays:
            raise ValueError(f"Tray {number} exists already")
        if int(tape_w) not in HOLE_TO_PART:
            raise ValueError("Tape width: 8, 12, 16 or 24 mm")
        if type_ not in TYPES:
            raise ValueError(f"Tray type: {', '.join(TYPES)}")
        mid, dic = self.next_marker(int(number))
        if dic != self.dictionary:
            self.dictionary = dic
        if type_ in ("matrix", "stick") and (tape_floor_mm is None or grid.get("top_mm") is None):
            raise ValueError("A matrix tray or a stick: measure where the parts sit (tape_floor_mm) and its top, where "
                             "its marker is (top_mm), above the label surface")
        floor = tape_floor_mm if tape_floor_mm is not None else \
            (6.5 if type_ == "holder" else 0.0)                    # bed.yaml: tape top 7.5 above the bar in a holder
        t = Tray(int(number), mid, type_, int(tape_w), kind, float(zone_mm), float(floor), bank,
                 None if plan_xy is None else [float(plan_xy[0]), float(plan_xy[1])],
                 [float(plan_dir[0]), float(plan_dir[1])], **grid)
        if type_ in ("matrix", "stick") and t.cols < 1:
            t.cols = 1
        self.trays[t.number] = t
        self.save()
        return t

    def remove(self, number: int) -> Tray:
        t = self.trays.pop(int(number))
        self.save()
        return t

    # ---------------------------------------------------------------- reach
    def reach(self, tray: Tray, frame, camera_travel: dict, tip_offset) -> str:
        """reachable | partly (some pockets) | camera_only | out_of_reach, by the NOZZLE reach (the travel shifted
        by the tip offset), not the camera's. frame: (marker centre, u, v) from the survey or the plan."""
        c, u, _v = frame
        pts = [np.asarray(c) + np.asarray(u) * a for a in np.linspace(tray.zone_u[0] + 2, tray.zone_u[1] - 2, 9)]
        X, Y = camera_travel["X"], camera_travel["Y"]
        if X is None or Y is None:
            return "out_of_reach"
        off = np.asarray(tip_offset, float)

        def tip_ok(p):
            h = p - off
            return X[0] <= h[0] <= X[1] and Y[0] <= h[1] <= Y[1]

        def cam_ok(p):
            return X[0] - 20 <= p[0] <= X[1] + 20 and Y[0] - 20 <= p[1] <= Y[1] + 20
        n = sum(tip_ok(p) for p in pts)
        if n == len(pts):
            return "reachable"
        if n > 0:
            return "partly"
        return "camera_only" if any(cam_ok(p) for p in pts) else "out_of_reach"


# ---------------------------------------------------------------- the feeder state (what is loaded)
class FeederState:
    """Per tray: the line it holds, the template of its confirmed first part, the pockets used. data folder."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.tpl_dir = path.parent / "templates"
        self.d: dict[str, dict] = {}
        if path.is_file():
            try:
                self.d = json.loads(path.read_text())
            except json.JSONDecodeError:
                self.d = {}

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.d, indent=1))
        os.replace(tmp, self.path)

    def get(self, number: int) -> dict:
        return self.d.get(str(number), {})

    def load_line(self, number: int, line: str, value: str, package: str, scanned: str = "") -> None:
        """A strip of `line` goes into the tray: a new strip, so no template and no used pockets yet."""
        self.d[str(number)] = {"line": line, "value": value, "package": package, "used": [], "template": None,
                               "loaded": time.strftime("%Y-%m-%d %H:%M:%S"), "scanned": scanned, "empty": False}
        self._save()

    def unload(self, number: int) -> None:
        self.d.pop(str(number), None)
        self._save()

    def refill(self, number: int) -> None:
        e = self.d.get(str(number))
        if e is not None:
            e["used"], e["empty"] = [], False
            self._save()

    def set_template(self, number: int, tpl: np.ndarray) -> None:
        self.tpl_dir.mkdir(parents=True, exist_ok=True)
        p = self.tpl_dir / f"tray_{number}.npy"
        np.save(p, np.asarray(tpl, np.float32))
        self.d.setdefault(str(number), {})["template"] = str(p)
        self._save()

    def template(self, number: int) -> np.ndarray | None:
        p = self.get(number).get("template")
        return np.load(p) if p and Path(p).is_file() else None

    def use(self, number: int, pocket: int) -> None:
        e = self.d.setdefault(str(number), {})
        e.setdefault("used", []).append(int(pocket))
        self._save()

    def mark_empty(self, number: int, empty: bool = True) -> None:
        self.d.setdefault(str(number), {})["empty"] = empty
        self._save()

    def trays_for(self, line: str) -> list[int]:
        return sorted(int(k) for k, v in self.d.items() if v.get("line") == line)
