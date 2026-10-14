"""The machine's parts store and hand-assembly operations (Phase E2, docs/13 §4.15): drawers, bag scans (to load a
tray, or to find a part's drawer), links, labels, the hand list after the machine job, and the placed state shared
with the machine (machine_job.PlacedStore)."""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

from .store.hand import passes
from .store.labels import build_printer, drawer_labels, label_image
from .store.lines import board_lines
from .store.store import PartsStore

if TYPE_CHECKING:
    from .machine import Machine


class StoreOps:
    def __init__(self, m: Machine) -> None:
        self.m = m
        self.store = PartsStore(m.data / "store.json")
        self._cache: tuple | None = None

    def lines(self):
        b = self.m.boards.current()
        if b is None:
            return None, []
        key = (b.name, self.m.boards.current_name())
        if self._cache is None or self._cache[0] != key:
            from .feeders.packages import annotate
            annotate(b, self.m.boards.library())                   # packages: the feeder line keys
            self._cache = (key, b, board_lines(b))
        return self._cache[1], self._cache[2]

    # ---------------------------------------------------------------- drawers
    def drawers_view(self) -> dict:
        b, lines = self.lines()
        if b is None:
            return {"ok": False, "why": "No board", "todo": "Import it (Board tab)"}
        dr = self.store.drawers(lines)
        qty = self.store.d["qty"]
        out: dict[int, list] = {}
        for ln in lines:
            out.setdefault(dr[ln.key], []).append(ln.as_dict() | {"drawer": dr[ln.key], "qty": qty.get(ln.key)})
        return {"ok": True, "board": b.name, "count": self.store.d["drawer_count"],
                "drawers": [{"n": n, "lines": out[n]} for n in sorted(out)]}

    def move(self, key: str, drawer: int) -> dict:
        _, lines = self.lines()
        if not 1 <= int(drawer) <= self.store.d["drawer_count"]:
            return {"ok": False, "why": f"Drawers are 1 … {self.store.d['drawer_count']}"}
        self.store.move(key, int(drawer), lines)
        return {"ok": True}

    def set_count(self, n: int) -> dict:
        try:
            self.store.set_count(int(n))
        except ValueError as err:
            return {"ok": False, "why": str(err)}
        return {"ok": True}

    # ---------------------------------------------------------------- bag scans
    def scan(self, text: str, tray: int | None = None) -> dict:
        """Where does this bag go? With `tray`: load it into that tray (the bag's code decides the line, R5)."""
        b, lines = self.lines()
        if b is None:
            return {"ok": False, "why": "No board", "todo": "Import it (Board tab)"}
        r = self.store.scan(text, lines)
        if not r["ok"]:
            return r
        ln = r["line"]
        dr = self.store.drawers(lines)
        st = self.m.feeder_state
        here = [int(t) for t, v in st.d.items() if v.get("line") == ln["feeder_key"] and ln["feeder_key"]]
        machine_line = self._is_machine_line(ln)
        r["drawer"] = dr.get(ln["key"])
        if tray is not None:
            if not machine_line:
                return {"ok": False, "why": f"{ln['value']} {ln['footprint']} is not a machine part: drawer "
                        f"{r['drawer']}, by hand", "line": ln}
            res = self.m.jobs.load_tray(int(tray), ln["feeder_key"], text.strip()[:120])
            r.update(res)
            r["where"] = f"Tray {tray}"
            return r
        if here:
            r["where"] = f"Tray {here[0]}"
        elif machine_line:
            sug = self._suggested_tray(ln["feeder_key"])
            r["where"] = f"Tray {sug} (load sheet)" if sug else f"Drawer {r['drawer']} (not in this load)"
        else:
            r["where"] = f"Drawer {r['drawer']}, by hand"
        need = len(ln["refs"])
        r["need"] = need
        return r

    def _is_machine_line(self, ln: dict) -> bool:
        b, _ = self.lines()
        from .feeders.packages import annotate
        cls = annotate(b, self.m.boards.library())
        return bool(ln["feeder_key"]) and any(cls.get(r, {}).get("class") == "machine" for r in ln["refs"])

    def _suggested_tray(self, feeder_key: str) -> int | None:
        try:
            pv = self.m.jobs.plan_view()
        except Exception:                                          # noqa: BLE001 - no plan yet
            return None
        for ld in pv.get("loads", []):
            for t, line in ld["trays"].items():
                if line == feeder_key:
                    return int(t)
        return None

    def link(self, text: str, key: str) -> dict:
        try:
            self.store.link(text, key)
        except ValueError as err:
            return {"ok": False, "why": str(err)}
        return {"ok": True}

    # ---------------------------------------------------------------- the hand list
    def hand(self) -> dict:
        b, lines = self.lines()
        if b is None:
            return {"ok": False, "why": "No board", "todo": "Import it (Board tab)"}
        placed = self.m.placed.get(b.name)
        reasons = {}
        from .feeders.packages import annotate
        for ref, cl in annotate(b, self.m.boards.library()).items():
            if cl.get("class") != "machine":
                reasons[ref] = cl.get("why", "")
        j = self.m.jobs.job
        if j is not None and j.board == b.name:
            for s in j.skipped:
                reasons[s["ref"]] = s["reason"]
            for p in j.parts:
                if p.state in ("failed", "skipped"):
                    reasons[p.ref] = f"the machine: {p.reason or p.state}"
        return {"ok": True, "board": b.name, "passes": passes(b, lines, self.store.drawers(lines), placed, reasons)}

    def mark(self, refs: list[str], placed: bool) -> dict:
        b, _ = self.lines()
        if b is None:
            return {"ok": False, "why": "No board"}
        for r in refs:
            self.m.placed.mark(b.name, r, "hand", placed)
        return {"ok": True}

    def import_old(self, data: dict) -> dict:
        """Solder Helper's localStorage, once: the placed refs, the drawer count and moves, the bag links."""
        b, lines = self.lines()
        if b is None:
            return {"ok": False, "why": "No board"}
        n = 0
        for ref in data.get("placed", []):
            if ref not in self.m.placed.get(b.name):
                self.m.placed.mark(b.name, ref, "hand")
                n += 1
        dr = data.get("drawers") or {}
        if dr.get("count"):
            self.store.d["drawer_count"] = int(dr["count"])
        for k, v in (dr.get("over") or {}).items():
            self.store.d["moves"][k] = int(v)                       # the same key: value + "\0" + footprint
        for k, v in (data.get("links") or {}).items():
            self.store.d["links"][k] = v
        self.store._save()
        self.m.say(f"Imported from Solder Helper: {n} placed marks, {len(dr.get('over') or {})} drawer moves, "
                   f"{len(data.get('links') or {})} links", kind="store")
        return {"ok": True, "placed": n}

    # ---------------------------------------------------------------- labels
    def print_labels(self, which: str = "drawers", mode: str = "strip") -> dict:
        b, lines = self.lines()
        if b is None:
            return {"ok": False, "why": "No board"}
        out = self.m.data / "labels" / f"{which}_{time.strftime('%Y%m%d-%H%M%S')}.pdf"
        pr = build_printer(self.m.profile.label_printer.type, out)
        st = pr.status()
        h = st.get("tape_px") or 76
        dr = self.store.drawers(lines)
        if which == "drawers":
            groups: dict[int, list[str]] = {}
            for ln in lines:
                groups.setdefault(dr[ln.key], []).append(f"{ln.value} {ln.footprint[:18]}")
            imgs = drawer_labels(groups, h)
        else:                                                       # one label per line: value, package, drawer
            imgs = [label_image(ln.value, [ln.footprint[:24], f"drawer {dr[ln.key]}"], h) for ln in lines]
        r = pr.print(imgs, mode)
        if r.get("pdf"):
            r["file"] = r["pdf"].split("/")[-1]
        return r
