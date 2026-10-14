"""Calibration results (docs/13 §4.10): one YAML file, every entry with its date, method, the inputs it depends on
and its measured quality. A copy of the file is kept before every write (the config history).

Every entry: value, meta (date, method, inputs, quality) and `previous` (the last few values, for Undo). The validity
chain (calibration/chain.py) gives each entry's inputs; an entry is stale when an input it names changed, and an
input can be another entry's date, so re-measuring one result makes the results built on it stale.
"""
from __future__ import annotations

import os
import shutil
import time
from pathlib import Path

import yaml


class CalibrationStore:
    def __init__(self, path: Path, history: Path | None = None) -> None:
        self.path = path
        self.history = history

    def _read(self) -> dict:
        try:
            st = self.path.stat()
        except FileNotFoundError:
            return {}
        key = (st.st_mtime_ns, st.st_size)
        if getattr(self, "_cache_key", None) != key:
            self._cache = yaml.safe_load(self.path.read_text()) or {}
            self._cache_key = key
        import copy
        return copy.deepcopy(self._cache)

    UNDO_DEPTH = 5

    def get(self, key: str) -> dict | None:
        return self._read().get(key)

    def keys(self) -> list[str]:
        return list(self._read())

    def date(self, key: str) -> str | None:
        e = self.get(key)
        return None if e is None else e.get("meta", {}).get("date")

    def status(self, key: str, inputs: dict | None = None) -> tuple[str, str, str | None]:
        """("valid" | "stale" | "missing", why, date)."""
        e = self.get(key)
        if e is None:
            return "missing", "not measured yet", None
        v, why = self.value(key, inputs)
        return ("valid" if v is not None else "stale"), why, e.get("meta", {}).get("date")

    def value(self, key: str, inputs: dict | None = None) -> tuple[dict | None, str]:
        """The value, if every input it was made with still holds. Else (None, why)."""
        e = self.get(key)
        if e is None:
            return None, "not measured yet"
        for k, v in (inputs or {}).items():
            have = (e.get("meta", {}).get("inputs") or {}).get(k)
            if have != v:
                return None, f"made with {k} = {have}, now {v}: measure it again"
        return e.get("value"), ""

    def put(self, key: str, value: dict, inputs: dict, method: str, quality: dict | None = None) -> None:
        data = self._read()
        if self.path.exists() and self.history is not None:
            self.history.mkdir(parents=True, exist_ok=True)
            shutil.copy(self.path, self.history / f"{self.path.stem}-{time.strftime('%Y%m%d-%H%M%S')}.yaml")
        old = data.get(key)
        prev = [] if old is None else ([{"value": old["value"], "meta": old["meta"]}] + old.get("previous", []))
        data[key] = {"value": value, "meta": {"date": _stamp(data, key), "method": method, "inputs": inputs,
                                              "quality": quality or {}},
                     "previous": prev[: self.UNDO_DEPTH]}
        self._write(data)

    def undo(self, key: str) -> bool:
        """Put the value before the last one back. False when there is none."""
        data = self._read()
        e = data.get(key)
        if e is None:
            return False
        if self.path.exists() and self.history is not None:
            self.history.mkdir(parents=True, exist_ok=True)
            shutil.copy(self.path, self.history / f"{self.path.stem}-{time.strftime('%Y%m%d-%H%M%S')}.yaml")
        prev = e.get("previous") or []
        if not prev:
            del data[key]
        else:
            data[key] = {"value": prev[0]["value"], "meta": prev[0]["meta"], "previous": prev[1:]}
        self._write(data)
        return True

    def remove(self, key: str) -> None:
        data = self._read()
        if key in data:
            del data[key]
            self._write(data)

    def _write(self, data: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        header = ("# Calibration results, written by the program (docs/13 §4.10). Do not edit by hand: run the\n"
                  "# matching Setup step again. A copy is kept before every write (data folder, config_history).\n")
        tmp.write_text(header + yaml.safe_dump(data, sort_keys=False, default_flow_style=None))
        os.replace(tmp, self.path)


def _stamp(data: dict, key: str) -> str:
    """The date, made unique for this key (two saves within a second still differ: dependants must see it)."""
    t = time.strftime("%Y-%m-%d %H:%M:%S")
    old = (data.get(key) or {}).get("meta", {}).get("date")
    n = 1
    out = t
    while out == old:
        n += 1
        out = f"{t}.{n}"
    return out
