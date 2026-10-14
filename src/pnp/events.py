"""Structured events (docs/13 §4.11): one JSON line per event in the data dir, the last 200 in memory for the
page, and listeners (the page's WebSocket)."""
from __future__ import annotations

import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable

LEVELS = ("debug", "info", "warn", "error")


class EventLog:
    def __init__(self, path: Path | None, keep: int = 200, max_bytes: int = 5_000_000) -> None:
        self.path = path
        self.max_bytes = max_bytes
        self.recent: deque[dict] = deque(maxlen=keep)
        self.listeners: list[Callable[[dict], None]] = []
        self.lock = threading.Lock()
        self.seq = 0
        self.context: Callable[[], dict] | None = None      # the machine state written with every event

    def add(self, kind: str, text: str, level: str = "info", **data) -> dict:
        ev = {"t": round(time.time(), 3), "kind": kind, "level": level, "text": text}
        if data:
            ev["data"] = data
        if self.context is not None:
            try:
                ev["state"] = self.context()
            except Exception:
                pass
        with self.lock:
            self.seq += 1
            ev["seq"] = self.seq
            self.recent.append(ev)
            if self.path is not None:
                self._write(ev)
        for fn in list(self.listeners):
            try:
                fn(ev)
            except Exception:
                pass
        return ev

    def _write(self, ev: dict) -> None:
        try:
            if self.path.exists() and self.path.stat().st_size > self.max_bytes:
                self.path.replace(self.path.with_suffix(".1.jsonl"))       # keep one old file
            with self.path.open("a") as f:
                f.write(json.dumps(ev, default=str) + "\n")
        except OSError:
            pass

    def tail(self, n: int = 200) -> list[dict]:
        with self.lock:
            return list(self.recent)[-n:]
