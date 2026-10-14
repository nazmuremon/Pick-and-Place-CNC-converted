"""The machine's safety state: ONE object with one message and one "what to do" (docs/13 §4.8, bug 16).

Modes, from least to most trusted:
  disconnected  no controller link
  locked        something made the position or the nozzle height unknown (reset, ALARM, stall, link lost,
                E-stop). Only Z up is allowed until the operator says the nozzle is clear.
  not_homed     the XY position is unknown: slow jogs only, no go-to, no automatic moves
  homed         XY known (from HOME). The Z reference is a separate flag (z_ref).

Every action asks `refuse(action)`: None = allowed, else a Refusal with the reason and what to do.
"""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass

ACTIONS = ("jog_xy", "jog_z_up", "jog_z_down", "goto_xy", "home", "z_reference", "turn", "pump", "light",
           "auto_z_down", "unlock_alarm", "confirm_clear")

# lock code -> (title, what to do)
LOCKS = {
    "reset": ("The controller was reset",
              "The position is lost (power dip, USB, or a reset). Lift the nozzle clear with Z+ (Machine tab), "
              "press 'Nozzle is clear', then Home and set Z again."),
    "alarm": ("The controller is in ALARM",
              "Look at the machine. Press 'Unlock alarm' (Machine tab), lift the nozzle clear with Z+, press "
              "'Nozzle is clear', then Home and set Z again."),
    "stall": ("A move did not happen as commanded",
              "The head may have hit an end or lost steps. Lift the nozzle clear with Z+, press 'Nozzle is "
              "clear', then Home again."),
    "link_lost": ("The controller stopped answering",
                  "Check the USB cable and the controller power, then press Connect (Machine tab)."),
    "estop": ("E-stop",
              "Release the E-stop when it is safe. Then: Unlock alarm, lift the nozzle clear, 'Nozzle is "
              "clear', Home and set Z again."),
    "unclean": ("The last session did not end cleanly",
                "The nozzle may be down. Lift it clear with Z+ (Machine tab), press 'Nozzle is clear', then Home."),
}


@dataclass
class Lock:
    code: str
    detail: str = ""
    since: float = 0.0

    @property
    def title(self) -> str:
        return LOCKS.get(self.code, (self.code, ""))[0]

    @property
    def todo(self) -> str:
        return LOCKS.get(self.code, ("", "Ask for help: unknown lock"))[1]


@dataclass
class Refusal:
    why: str
    todo: str

    def __str__(self) -> str:
        return f"{self.why}. {self.todo}"


class Refused(RuntimeError):
    def __init__(self, r: Refusal) -> None:
        self.refusal = r
        super().__init__(str(r))


class Stopped(Refused):
    """STOP was pressed: the operation ends at once, nothing more moves (the caller only lifts the nozzle)."""

    def __init__(self) -> None:
        super().__init__(Refusal("Stopped by you", "Choose what to do next"))


@dataclass
class SafetyState:
    connected: bool = False
    lock: Lock | None = None
    homed: bool = False
    z_ref: bool = False

    @property
    def mode(self) -> str:
        if not self.connected:
            return "disconnected"
        if self.lock is not None:
            return "locked"
        return "homed" if self.homed else "not_homed"

    # ---------------------------------------------------------------- events
    def set_lock(self, code: str, detail: str = "") -> None:
        """Any lock forgets the position: after it, nothing is trusted until Home and Z are done again.
        An E-stop or ALARM is never replaced by a milder lock (the operator must see the first cause)."""
        if self.lock is not None and self.lock.code in ("estop", "alarm") and code not in ("estop", "alarm"):
            return
        self.lock = Lock(code, detail, time.time())
        self.homed = False
        self.z_ref = False

    def clear_lock(self) -> None:
        self.lock = None

    # ---------------------------------------------------------------- the rules
    def refuse(self, action: str) -> Refusal | None:
        if action not in ACTIONS:
            raise ValueError(f"unknown action {action}")
        if not self.connected:
            return Refusal("The controller is not connected", "Press Connect (Machine tab)")
        if self.lock is not None:
            if action == "jog_z_up" or action == "pump" or action == "light":
                return None
            if action == "unlock_alarm" and self.lock.code in ("alarm", "estop"):
                return None
            if action == "confirm_clear" and self.lock.code not in ("alarm", "estop", "link_lost"):
                return None
            return Refusal(self.lock.title, self.lock.todo)
        if action in ("unlock_alarm", "confirm_clear"):
            return Refusal("Nothing is locked", "No need to do this now")
        if action == "goto_xy" and not self.homed:
            return Refusal("The position is not known yet", "Press Home first (Machine tab)")
        if action == "auto_z_down" and not (self.homed and self.z_ref):
            return Refusal("Z goes down by itself only after Home and the Z reference",
                           "Home, then set Z (Setup, step 4)")
        return None

    def banner(self) -> dict:
        """What the page shows at the top: one message and one thing to do."""
        if not self.connected:
            return {"level": "off", "title": "Not connected", "todo": "Press Connect (Machine tab)",
                    "link": "#machine"}
        if self.lock is not None:
            return {"level": "locked", "title": self.lock.title, "detail": self.lock.detail, "todo": self.lock.todo,
                    "link": "#machine", "code": self.lock.code}
        if not self.homed:
            return {"level": "warn", "title": "Not homed: slow jogs only",
                    "todo": "Jog until the HOME label is in view, then press Home (Machine tab)", "link": "#machine"}
        if not self.z_ref:
            return {"level": "warn", "title": "Homed. Z has no reference yet",
                    "todo": "Touch the nozzle onto the bed label surface and press 'Set Z here' (Machine tab)",
                    "link": "#machine"}
        return {"level": "ok", "title": "Ready", "todo": "", "link": ""}

    def as_dict(self) -> dict:
        d = asdict(self)
        d["mode"] = self.mode
        return d
