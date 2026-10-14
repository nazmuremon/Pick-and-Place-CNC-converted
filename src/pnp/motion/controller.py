"""The controller in machine axes: connect (with the reset check), jog, go to, G-code lines, stop.

Session token (D22): Grbl and grblHAL forget the tool number T at every reset (power, USB, soft reset), but keep
it while they run. At Connect the program reads T with $G. If it is the number this program wrote last time, the
controller was NOT reset: the position, Home and the Z reference are still right, even after a program restart.
Otherwise the old position is gone. Then a new number is written.
"""
from __future__ import annotations

import random
import time
from dataclasses import dataclass, field

from ..profile.schema import Profile
from .axes import AxisMap
from .grbl import ControllerError, GrblLink, Status

JOG_CANCEL = b"\x85"


@dataclass
class ConnectInfo:
    version: str = ""
    settings: dict[str, str] = field(default_factory=dict)
    token_found: int | None = None       # T read at Connect
    token: int | None = None             # T written now (None: the token is off or could not be written)
    state: str = ""
    mismatched: dict[str, tuple[str, str]] = field(default_factory=dict)   # setting -> (expected, actual)
    firmware_ok: bool = True
    notes: list[str] = field(default_factory=list)


class Controller:
    def __init__(self, link: GrblLink, profile: Profile) -> None:
        self.link = link
        self.profile = profile
        self.axes = AxisMap.from_profile(profile)
        self.info = ConnectInfo()

    # ---------------------------------------------------------------- connect
    def wait_alive(self, timeout: float) -> Status:
        """An Uno that resets on open says nothing for ~1.5 s: wait for the first status."""
        return self.link.wait_status(0.0, timeout)

    def connect(self, old_token: int | None) -> ConnectInfo:
        c = self.profile.controller
        st = self.wait_alive(c.open_timeout_s)
        info = ConnectInfo(state=st.state)
        try:
            info.version = self.link.build_info()
            info.settings = self.link.settings()
            gc = self.link.parser_state() if c.session_token else {}
        except ControllerError as err:
            info.notes.append(f"Could not read the controller's info: {err}")
            gc = {}
        if c.firmware and c.firmware not in info.version:
            info.firmware_ok = False
            info.notes.append(f"Firmware {info.version or '?'} is not {c.firmware}: the commands that need it are off")
        for k, want in c.expected_settings.items():
            have = info.settings.get(k)
            if have is not None and _num(have) != _num(want):
                info.mismatched[k] = (want, have)
        if c.session_token and "T" in gc:
            try:
                info.token_found = int(float(gc["T"]))
            except ValueError:
                info.token_found = None
        if c.session_token and st.base != "Alarm":
            if old_token and info.token_found == old_token:
                info.token = old_token                     # not reset since: keep it
            else:
                info.token = self.write_token(old_token)
        self.info = info
        return info

    def write_token(self, old: int | None) -> int | None:
        """Write a new token (1-255, never the old one; 0 is what a reset leaves)."""
        new = random.choice([n for n in range(1, 256) if n != old])
        try:
            self.link.send(f"T{new}")
            return new
        except ControllerError:
            return None

    # ---------------------------------------------------------------- positions
    def raw(self) -> dict[str, float]:
        """The last reported position in machine axes, controller frame (no HOME offset)."""
        st = self.link.status
        return self.axes.to_machine(st.mpos) if st else {}

    def state(self) -> str:
        return self.link.status.state if self.link.status else "?"

    # ---------------------------------------------------------------- motion
    def jog(self, delta: dict[str, float], feed: float) -> float:
        """Relative jog in machine axes. Returns the time the ok came (the jog runs after it)."""
        self.link.send(f"$J=G91 G21 {self.axes.words(delta)} F{feed:.0f}")
        return time.monotonic()

    def jog_to(self, raw_target: dict[str, float], feed: float) -> float:
        """Jog to a controller-frame position (G53: machine coordinates, no work offset)."""
        self.link.send(f"$J=G53 G90 G21 {self.axes.words(raw_target)} F{feed:.0f}")
        return time.monotonic()

    def wait_idle(self, after: float, timeout: float) -> Status:
        return self.link.wait_idle(after, timeout)

    def gcode(self, line: str, timeout: float = 10.0) -> list[str]:
        """One G-code line (M-codes, G4). Grbl answers M3 / M8 / M9 / G4 only after the moves before them."""
        return self.link.send(line, timeout)

    def sync(self, timeout: float = 60.0) -> Status:
        """Wait until every queued move and command is done."""
        self.link.send("G4 P0.01", timeout)
        return self.link.wait_idle(time.monotonic(), 5.0)

    # ---------------------------------------------------------------- stop
    def stop(self) -> None:
        """Software STOP: cancel jogs (no position loss) and hold any other motion."""
        self.link.realtime(JOG_CANCEL)
        st = self.link.status
        if st is not None and st.base in ("Run", "Home"):
            self.link.realtime(b"!")

    def unlock(self) -> None:
        self.link.send("$X")


def _num(v: str) -> str:
    try:
        return f"{float(v):g}"
    except ValueError:
        return v.strip()
