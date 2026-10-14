"""Board import sessions, the saved boards, and the preview for the page (docs/13 §4.3).

An import is a folder of uploaded files. `run` picks the importer:
  a .kicad_pcb            -> KiCad
  Gerbers with X3 files   -> Gerber X3 (a centroid file next to them is then not needed)
  a centroid file         -> centroid, with the Gerbers / KiCad file of the same import as the reference, else the
                             current board (same parts), else none
Every result gets the package classes and the checks. Nothing becomes the current board until `save`.
"""
from __future__ import annotations

import json
import re
import shutil
import time
import uuid
from dataclasses import replace
from pathlib import Path

from ..feeders.packages import CLASSES, Library, annotate
from . import centroid as ce
from . import checks, gerber_import, kicad
from .model import Board

MAX_FILE = 60 * 1024 * 1024
CENTROID_EXT = (".csv", ".pos", ".txt", ".mnt", ".mnb", ".xy", ".tsv")
KICAD_EXT = (".kicad_pcb", ".kicad_pcb.gz")


def slug(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("._") or "board"


class BoardService:
    def __init__(self, root: Path, cfg_dir: Path) -> None:
        self.root = root
        self.cfg_dir = cfg_dir
        self.imports = root / "_imports"
        self.imports.mkdir(parents=True, exist_ok=True)

    # ---------------------------------------------------------------- the package library
    def library(self) -> Library:
        try:
            return Library.load(self.cfg_dir)
        except (OSError, ValueError):
            return Library({})

    # ---------------------------------------------------------------- imports
    def new_import(self) -> str:
        iid = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
        (self.imports / iid / "files").mkdir(parents=True)
        self._prune()
        return iid

    def _dir(self, iid: str) -> Path:
        if not re.fullmatch(r"[\w-]+", iid) or not (self.imports / iid).is_dir():
            raise KeyError(f"no import {iid}")
        return self.imports / iid

    def add_file(self, iid: str, name: str, data: bytes) -> list[str]:
        if len(data) > MAX_FILE:
            raise ValueError(f"{name}: bigger than {MAX_FILE // 1024 // 1024} MB")
        d = self._dir(iid) / "files"
        (d / slug(Path(name).name)).write_bytes(data)
        return sorted(p.name for p in d.iterdir())

    def files(self, iid: str) -> list[tuple[str, bytes]]:
        d = self._dir(iid) / "files"
        return [(p.name, p.read_bytes()) for p in sorted(d.iterdir()) if p.is_file()]

    def _prune(self, keep: int = 20) -> None:
        for old in sorted(p for p in self.imports.iterdir() if p.is_dir())[:-keep]:
            shutil.rmtree(old, ignore_errors=True)

    def run(self, iid: str, spec: dict | None = None, fit: bool = True, use_current: bool = True,
            name: str | None = None) -> dict:
        files = self.files(iid)
        if not files:
            raise ValueError("No files in this import: upload the board files first")
        low = [(n.lower(), n, d) for n, d in files]
        kicad_files = [(n, d) for l_, n, d in low if l_.endswith(KICAD_EXT)]
        cent = [(n, d) for l_, n, d in low if l_.endswith(CENTROID_EXT) and not _looks_like_drill(d)]
        gerb = [(n, d) for n, d in files if (n, d) not in kicad_files and (n, d) not in cent]
        meta: dict = {"import": iid, "files": [n for n, _ in files]}
        reference: Board | None = None
        if kicad_files:
            n, d = kicad_files[0]
            tmp = self._dir(iid) / "files" / n
            board = kicad.load(tmp, name or n.split(".")[0])
            meta["kind"] = "kicad"
            if len(kicad_files) > 1:
                board.notes.append(f"More than one KiCad file: used {n}")
        elif gerb and not cent:
            board = gerber_import.load_files(gerb, name)
            meta["kind"] = "gerber_x3"
        else:
            if not cent:
                raise ValueError("No board file found: upload a .kicad_pcb, Gerbers with X3 files, or a centroid file")
            if gerb:
                try:
                    reference = gerber_import.load_files(gerb, name, require_components=False)
                except ValueError as err:
                    meta["reference_error"] = str(err)
            elif use_current:
                reference = self.current()
            board, info = self._centroid(cent, spec, fit, reference, name)
            meta.update(info)
            meta["kind"] = "centroid"
            meta["reference"] = None if reference is None else reference.source_tool
        lib = self.library()
        classes = annotate(board, lib)
        polar = {p.id for p in lib.packages.values() if p.polar}
        issues = checks.check(board, classes, polar)
        board.save(self._dir(iid) / "board.json")
        (self._dir(iid) / "meta.json").write_text(json.dumps({**meta, "classes": classes}, default=str))
        return {**meta, "board": board.summary(), "notes": board.notes, "conventions": board.conventions,
                "issues": [i.as_dict() for i in issues], "classes": _count(classes)}

    def _centroid(self, cent, spec, fit, reference, name) -> tuple[Board, dict]:
        rows: list[ce.Row] = []
        info: dict = {}
        sn = ce.sniff(cent[0][1], cent[0][0])
        s = ce.CentroidSpec.from_dict({**sn.spec.to_dict(), **(spec or {})}) if spec else sn.spec
        for fname, data in cent:
            per = ce.sniff(data, fname).spec if fname != cent[0][0] else s
            if spec:
                per = replace(s, side_default=per.side_default)
            rows += ce.rows(data, per)
        info["sniff"] = {"header": sn.header, "sample": sn.sample, "unsure": sn.unsure, "tool": sn.spec.tool}
        if fit and reference is not None:
            f = ce.suggest(rows, s, reference)
            if f.matched >= 2:
                s = f.spec
            info["fit"] = {"report": f.report, "matched": f.matched, "position_error": f.position_error,
                           "rotation_agree": f.rotation_agree}
        info["spec"] = s.to_dict()
        info["rules"] = {k: {"formula": v[0], "about": v[1]} for k, v in ce.BOTTOM_RULES.items()}
        board = ce.to_board(rows, s, name or Path(cent[0][0]).stem, reference, cent[0][0])
        return board, info

    def pending(self, iid: str) -> Board:
        return Board.load(self._dir(iid) / "board.json")

    def save(self, iid: str, name: str | None = None) -> str:
        b = self.pending(iid)
        if name:
            b.name = name
        target = self.root / slug(b.name)
        if target.exists():
            shutil.rmtree(target)
        (target / "source").mkdir(parents=True)
        for n, d in self.files(iid):
            (target / "source" / n).write_bytes(d)
        b.save(target / "board.json")
        shutil.copy(self._dir(iid) / "meta.json", target / "meta.json")
        (self.root / "current.txt").write_text(target.name)
        return target.name

    # ---------------------------------------------------------------- saved boards
    def names(self) -> list[str]:
        return sorted(p.name for p in self.root.iterdir() if p.is_dir() and (p / "board.json").is_file())

    def current_name(self) -> str | None:
        f = self.root / "current.txt"
        n = f.read_text().strip() if f.is_file() else ""
        return n if n and (self.root / n / "board.json").is_file() else None

    def current(self) -> Board | None:
        n = self.current_name()
        return Board.load(self.root / n / "board.json") if n else None

    def use(self, name: str) -> None:
        if name not in self.names():
            raise KeyError(f"no board {name}")
        (self.root / "current.txt").write_text(name)

    def set_thickness(self, mm: float) -> Board:
        n = self.current_name()
        if n is None:
            raise KeyError("no current board")
        if not 0.2 <= mm <= 5.0:
            raise ValueError("thickness must be 0.2 … 5 mm")
        b = self.current()
        b.thickness, b.thickness_from = float(mm), "set by the operator"
        b.save(self.root / n / "board.json")
        return b

    # ---------------------------------------------------------------- the preview
    def preview(self, b: Board, side: str = "F") -> dict:
        lib = self.library()
        classes = annotate(b, lib)
        polar = {p.id for p in lib.packages.values() if p.polar}
        issues = checks.check(b, classes, polar)
        return build_preview(b, side, classes, issues)


def build_preview(b: Board, side: str, classes: dict, issues: list) -> dict:
    """Everything the page draws, seen from `side` (the bottom view is turned over left-right)."""
    V = lambda p: [round(v, 3) for v in b.view(side, p)]     # noqa: E731
    comps = []
    for c in b.components:
        cl = classes.get(c.ref, {})
        item = {"ref": c.ref, "value": c.value, "package": c.package, "footprint": c.footprint, "side": c.side,
                "cls": cl.get("class", "unknown"), "why": cl.get("why", ""), "xy": V((c.x, c.y)),
                "rot": round(c.rotation, 2), "smd": c.smd}
        if c.side == side or any(pd.through_hole for pd in c.pads):     # through-hole pins show on both sides
            item["pads"] = [[V(q) for q in _thin(pd.polygon)] for pd in c.pads]
            item["th"] = any(pd.through_hole for pd in c.pads)
            p1 = next((pd for pd in c.pads if pd.number == "1"), None)
            if p1 is not None and len(c.pads) > 1:
                item["pin1"] = V(p1.centre)
        comps.append(item)
    by_ref: dict[str, list[str]] = {}
    for i in issues:
        by_ref.setdefault(i.ref, []).append(i.text)
    return {"name": b.name, "side": side, "width": b.width, "height": b.height, "thickness": b.thickness,
            "thickness_from": b.thickness_from, "source": b.source_tool, "files": b.source_files,
            "outline": [[V(p) for p in lp] for lp in b.outline],
            "holes": [{"xy": V((h.x, h.y)), "d": h.diameter, "ref": h.ref} for h in b.holes],
            "fiducials": [{"xy": V((f.x, f.y)), "d": f.diameter, "side": f.side} for f in b.fiducials],
            "components": comps, "issues": [i.as_dict() for i in issues], "issue_refs": by_ref,
            "classes": _count(classes), "class_names": CLASSES, "notes": b.notes, "conventions": b.conventions,
            "summary": b.summary()}


def _thin(poly: list, most: int = 16) -> list:
    if len(poly) <= most:
        return poly
    step = len(poly) / most
    return [poly[int(i * step)] for i in range(most)]


def _count(classes: dict) -> dict:
    out: dict[str, int] = {}
    for v in classes.values():
        out[v["class"]] = out.get(v["class"], 0) + 1
    return out


def _looks_like_drill(data: bytes) -> bool:
    head = data[:400].decode("utf-8", errors="replace")
    return head.lstrip().startswith("M48") or "FMAT" in head
