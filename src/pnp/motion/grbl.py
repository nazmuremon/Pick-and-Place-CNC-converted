"""The Grbl / grblHAL link: one command at a time (send → ok), a status poll, realtime bytes, and the events
that matter for safety (reset banner, ALARM, link lost). Same dialect for Grbl 1.1 and grblHAL (docs/13 §4.2).

Threads: a reader (parses every line) and a poller (sends '?' every poll_s, watches for silence). A caller
blocks only in send() / wait_*().
"""
from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

from .axes import LETTERS

BANNER = re.compile(r"^(Grbl|GrblHAL) \S+ \[")
Listener = Callable[[str, dict], None]


class ControllerError(RuntimeError):
    """The controller refused a line or stopped (error, ALARM, reset, no answer)."""


class ResetSeen(ControllerError):
    pass


class AlarmSeen(ControllerError):
    def __init__(self, code: str, line: str = "") -> None:
        self.code = code
        super().__init__(f"ALARM:{code}" + (f" after {line!r}" if line else ""))


class LinkLost(ControllerError):
    pass


@dataclass
class Status:
    state: str                         # Idle, Run, Jog, Hold:0, Alarm, Door:1, Check, Home, Sleep
    mpos: dict[str, float]             # controller letter -> machine position (MPos)
    t: float                           # time.monotonic() when it arrived
    feed: float | None = None
    spindle: float | None = None       # the servo build reports the pulse width here
    pins: str = ""                     # Pn: (limit / probe / e-stop inputs)
    accessories: str = ""              # A: S = spindle (servo pulses), F = flood, M = mist

    @property
    def base(self) -> str:
        return self.state.split(":")[0]


def parse_status(line: str, wco: dict[str, float] | None, now: float) -> tuple[Status, dict[str, float] | None]:
    """'<Idle|MPos:1,2,3|FS:0,0|WCO:...>' → Status. WPos is turned into MPos with the last WCO."""
    fields = line.strip().strip("<>").split("|")
    st = Status(fields[0], {}, now)
    wpos = None
    for f in fields[1:]:
        key, _, val = f.partition(":")
        nums = val.split(",")
        if key == "MPos":
            st.mpos = {LETTERS[i]: float(v) for i, v in enumerate(nums)}
        elif key == "WPos":
            wpos = {LETTERS[i]: float(v) for i, v in enumerate(nums)}
        elif key == "WCO":
            wco = {LETTERS[i]: float(v) for i, v in enumerate(nums)}
        elif key == "FS":
            st.feed, st.spindle = float(nums[0]), float(nums[1])
        elif key == "F":
            st.feed = float(nums[0])
        elif key == "Pn":
            st.pins = val
        elif key == "A":
            st.accessories = val
    if not st.mpos and wpos is not None:
        off = wco or {}
        st.mpos = {k: v + off.get(k, 0.0) for k, v in wpos.items()}
    return st, wco


@dataclass
class _Pending:
    line: str
    done: threading.Event = field(default_factory=threading.Event)
    replies: list[str] = field(default_factory=list)
    result: str | None = None          # "ok", "error:n", "ALARM:n", "reset", "lost"


class GrblLink:
    def __init__(self, port, poll_s: float = 0.2, watchdog_s: float = 2.0) -> None:
        self.port = port                                   # a pyserial-like object (or the simulator's port)
        self.poll_s, self.watchdog_s = poll_s, watchdog_s
        self.listeners: list[Listener] = []
        self.status: Status | None = None
        self.wco: dict[str, float] | None = None
        self.messages: list[str] = []                      # [MSG:...] and other loose lines, newest last
        self._write_lock = threading.Lock()
        self._cmd_lock = threading.Lock()
        self._pending: _Pending | None = None
        self._status_cv = threading.Condition()
        self._closing = threading.Event()
        self._lost = False
        self._buf = b""
        self._reader = threading.Thread(target=self._read_loop, name="grbl-reader", daemon=True)
        self._poller = threading.Thread(target=self._poll_loop, name="grbl-poller", daemon=True)

    # ---------------------------------------------------------------- plumbing
    def start(self) -> GrblLink:
        self._last_status_t = time.monotonic()
        self._reader.start()
        self._poller.start()
        return self

    def close(self) -> None:
        self._closing.set()
        for t in (self._poller, self._reader):
            if t.is_alive() and t is not threading.current_thread():
                t.join(timeout=1.0)
        try:
            self.port.close()
        except Exception:
            pass

    def _emit(self, kind: str, **data) -> None:
        for fn in list(self.listeners):
            try:
                fn(kind, data)
            except Exception:                              # a listener must never stop the reader
                pass

    def _write(self, data: bytes) -> None:
        with self._write_lock:
            self.port.write(data)

    def realtime(self, byte: bytes) -> None:
        """'?' status, '!' feed hold, '~' resume, b'\\x85' jog cancel, b'\\x18' soft reset."""
        self._write(byte)

    # ---------------------------------------------------------------- reading
    def _read_loop(self) -> None:
        while not self._closing.is_set():
            try:
                chunk = self.port.read(256) if hasattr(self.port, "read") else b""
            except Exception as err:                       # USB unplugged
                if not self._closing.is_set():
                    self._link_lost(f"read failed: {err}")
                return
            if not chunk:
                continue
            self._buf += chunk
            while b"\n" in self._buf:
                raw, self._buf = self._buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line:
                    self._line(line)

    def _line(self, line: str) -> None:
        now = time.monotonic()
        if line.startswith("<") and line.endswith(">"):
            st, self.wco = parse_status(line, self.wco, now)
            with self._status_cv:
                self.status = st
                self._last_status_t = now
                if self._lost:
                    self._lost = False
                    self._emit("link_back")
                self._status_cv.notify_all()
            self._emit("status", status=st)
            return
        p = self._pending
        if line == "ok" or line.startswith("error"):
            if p is not None:
                p.result = line
                p.done.set()
            return
        if line.startswith("ALARM"):
            self._emit("alarm", code=line.partition(":")[2], line=line)
            if p is not None:
                p.result = line
                p.done.set()
            return
        if BANNER.match(line):
            self._emit("reset", banner=line)
            if p is not None:
                p.result = "reset"
                p.done.set()
            return
        if line.startswith("[PRB:"):
            self._emit("probe", line=line)
        if line.startswith("[MSG:"):
            self.messages = (self.messages + [line])[-50:]
            self._emit("message", line=line)
        if p is not None:
            p.replies.append(line)

    def _poll_loop(self) -> None:
        while not self._closing.wait(self.poll_s):
            try:
                self._write(b"?")
            except Exception as err:
                self._link_lost(f"write failed: {err}")
                return
            if time.monotonic() - self._last_status_t > self.watchdog_s and not self._lost:
                self._link_lost(f"no status for {self.watchdog_s:g} s")

    def _link_lost(self, why: str) -> None:
        self._lost = True
        p = self._pending
        if p is not None:
            p.result = "lost"
            p.done.set()
        self._emit("link_lost", why=why)

    @property
    def lost(self) -> bool:
        return self._lost

    # ---------------------------------------------------------------- commands
    def send(self, line: str, timeout: float = 5.0) -> list[str]:
        """Send one line and wait for its ok. Returns the lines that came before the ok."""
        with self._cmd_lock:
            p = _Pending(line)
            self._pending = p
            try:
                self._write(line.encode() + b"\n")
                if not p.done.wait(timeout):
                    raise ControllerError(f"no answer to {line!r} within {timeout:g} s")
            finally:
                self._pending = None
        if p.result == "ok":
            return p.replies
        if p.result == "reset":
            raise ResetSeen(f"the controller was reset while {line!r} ran")
        if p.result == "lost":
            raise LinkLost(f"the link was lost while {line!r} ran")
        if p.result and p.result.startswith("ALARM"):
            raise AlarmSeen(p.result.partition(":")[2], line)
        raise ControllerError(f"{line!r} -> {p.result}")

    def wait_status(self, after: float, timeout: float = 2.0) -> Status:
        """The first status that arrived after time `after` (monotonic)."""
        deadline = time.monotonic() + timeout
        with self._status_cv:
            while self.status is None or self.status.t <= after:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise LinkLost(f"no status within {timeout:g} s")
                self._status_cv.wait(left)
            return self.status

    def wait_idle(self, after: float, timeout: float) -> Status:
        """Wait until a status newer than `after` says Idle (or the machine stopped in Alarm / Hold)."""
        deadline = time.monotonic() + timeout
        t = after
        while True:
            st = self.wait_status(t, max(0.1, deadline - time.monotonic()))
            if st.base in ("Idle", "Alarm", "Door") or st.state == "Hold:0":
                return st
            if time.monotonic() > deadline:
                raise ControllerError(f"still {st.state} after {timeout:g} s")
            t = st.t

    def settings(self) -> dict[str, str]:
        out = {}
        for r in self.send("$$"):
            if r.startswith("$") and "=" in r:
                k, v = r.split("=", 1)
                out[k] = v.split(" ")[0]
        return out

    def build_info(self) -> str:
        """$I: '[VER:1.1h.20261007:]' -> '1.1h.20261007' (grblHAL: '[VER:1.1f.20230610:]')."""
        for r in self.send("$I"):
            if r.startswith("[VER:"):
                return r[5:].rstrip("]").split(":")[0]
        return ""

    def parser_state(self) -> dict[str, str]:
        """$G: '[GC:G0 G54 ... T3 F0 S0]' -> {'G': [...], 'T': '3', ...} (first letter -> value text)."""
        out: dict[str, str] = {}
        for r in self.send("$G"):
            if r.startswith("[GC:"):
                for w in r[4:].rstrip("]").split():
                    out[w[0]] = w[1:] if w[0] in "TFS" else out.get(w[0], "") + " " + w
        return out
