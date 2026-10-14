"""The parts store's state, on the server (docs/13 §4.15): one file shared by every browser on the LAN (Solder
Helper kept it in one browser's localStorage). Drawer count (1-999), the operator's drawer moves, bag links (a code
-> a line) and quantities on hand from the bag scans."""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

from .drawers import auto_assign
from .lines import StoreLine
from .scan import find_line, link_key, parse_code


class PartsStore:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.d = {"drawer_count": 10, "moves": {}, "links": {}, "qty": {}, "log": []}
        if path.is_file():
            try:
                self.d.update(json.loads(path.read_text()))
            except json.JSONDecodeError:
                pass

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(self.d, indent=1, ensure_ascii=False))
        os.replace(tmp, self.path)

    # ---------------------------------------------------------------- drawers
    def set_count(self, n: int) -> None:
        if not 1 <= int(n) <= 999:
            raise ValueError("1 to 999 drawers")
        self.d["drawer_count"] = int(n)
        self._save()

    def drawers(self, lines: list[StoreLine]) -> dict[str, int]:
        auto = auto_assign(lines, self.d["drawer_count"])
        out = {}
        for ln in lines:
            o = self.d["moves"].get(ln.key)
            out[ln.key] = o if o and o <= self.d["drawer_count"] else auto.get(ln.key, 1)
        return out

    def move(self, key: str, drawer: int, lines: list[StoreLine]) -> None:
        auto = auto_assign(lines, self.d["drawer_count"])
        if int(drawer) == auto.get(key):
            self.d["moves"].pop(key, None)
        else:
            self.d["moves"][key] = int(drawer)
        self._save()

    # ---------------------------------------------------------------- bags
    def scan(self, text: str, lines: list[StoreLine]) -> dict:
        code = parse_code(text)
        if code is None:
            return {"ok": False, "why": "Not a bag code or a part number"}
        ln = find_line(code, lines, self.d["links"])
        if ln is None:
            return {"ok": False, "why": f"No line of this board has {code.get('pc') or code.get('pm')}",
                    "todo": "Choose the part, then 'Link' to remember it", "code": code}
        if code.get("qty"):
            self.d["qty"][ln.key] = code["qty"]
            self._save()
        return {"ok": True, "line": ln.as_dict(), "code": code}

    def link(self, text: str, key: str) -> None:
        code = parse_code(text)
        k = link_key(code or {})
        if k is None:
            raise ValueError("Nothing to link")
        self.d["links"][k] = key
        self.d["log"].append(f"{time.strftime('%Y-%m-%d %H:%M')} linked {k} to {key.replace(chr(0), ' ')}")
        self._save()
