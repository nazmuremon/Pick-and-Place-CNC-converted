"""The capability interfaces (docs/13 §4.14). The job, the wizard and the page talk only to these; the profile
says which implementation is plugged in. Each has a real one, a `none` / manual one and the simulator's.

For the capabilities that work through G-code (homing switches, probe, servo, stepper axis, pump, valve,
controller light, E-stop input), the simulator's implementation IS the real class talking to the virtual
controller (sim/controller.py): the tests then run the real code.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np

from ..motion.controller import Controller


@dataclass(frozen=True)
class Availability:
    ok: bool
    why: str = ""            # when not ok: what is missing
    todo: str = ""           # when not ok: what to do

    @classmethod
    def yes(cls) -> Availability:
        return cls(True)

    @classmethod
    def no(cls, why: str, todo: str = "") -> Availability:
        return cls(False, why, todo)


class CapabilityError(RuntimeError):
    pass


class Capability:
    role: str = ""           # homing, z_reference, rotation, ...
    kind: str = ""           # the implementation's name, as in the profile

    def availability(self) -> Availability:
        return Availability.yes()

    def require(self) -> None:
        a = self.availability()
        if not a.ok:
            raise CapabilityError(f"{a.why}. {a.todo}".strip(". ") + ".")

    state = ""               # "" = real; "later" = built in a later phase; "absent" = not in this machine

    def describe(self) -> dict:
        a = self.availability()
        return {"role": self.role, "kind": self.kind, "ok": a.ok, "why": a.why, "todo": a.todo,
                "state": self.state}


class NotBuilt(Capability):
    """A capability whose code comes in a later phase: it says so instead of failing somewhere deep inside."""

    state = "later"

    def __init__(self, role: str, kind: str, phase: str) -> None:
        self.role, self.kind, self.phase = role, kind, phase

    def availability(self) -> Availability:
        return Availability.no(f"'{self.kind}' is built in {self.phase}", "Use the old bench page for it until then")


class NoneCap(Capability):
    """The profile has none of this hardware."""

    state = "absent"

    def __init__(self, role: str, why: str = "Not in this machine's profile") -> None:
        self.role, self.kind, self._why = role, "none", why

    def availability(self) -> Availability:
        return Availability.no(self._why)


# ---------------------------------------------------------------- the interfaces
@runtime_checkable
class Homing(Protocol):
    role: str
    kind: str
    restores_after_reset: bool        # True: a reset needs only home() again (switches), no camera search

    def availability(self) -> Availability: ...
    def home(self, ctl: Controller) -> dict[str, float]:
        """Find HOME. Returns the controller-frame XY of the travel frame's origin."""
        ...


@runtime_checkable
class ZReference(Protocol):
    role: str
    kind: str

    def availability(self) -> Availability: ...
    def establish(self, ctl: Controller, surface_z: float) -> float:
        """Returns the controller-frame Z where the nozzle tip touches Z = 0 (the reference surface).
        surface_z = the height of what the tip touches now, above the reference surface."""
        ...


@runtime_checkable
class RotationAxis(Protocol):
    role: str
    kind: str
    accuracy_deg: float | None
    range_deg: tuple[float, float] | None

    def availability(self) -> Availability: ...
    def turn_to(self, deg: float) -> None: ...
    def limp(self) -> None: ...


@runtime_checkable
class VacuumSystem(Protocol):
    role: str
    kind: str
    can_sense: bool

    def availability(self) -> Availability: ...
    def pump(self, on: bool) -> None: ...
    def pick(self) -> None: ...
    def release(self) -> None: ...
    def held(self) -> bool | None: ...


@runtime_checkable
class Light(Protocol):
    role: str
    kind: str

    def availability(self) -> Availability: ...
    def set(self, on: bool) -> None: ...


@runtime_checkable
class Camera(Protocol):
    role: str
    kind: str

    def availability(self) -> Availability: ...
    def latest(self) -> tuple[float, np.ndarray] | None: ...
    def frames_after(self, t: float, n: int, timeout: float = 5.0) -> list[np.ndarray]: ...
    def set_exposure(self, value: float | None) -> bool: ...
    def close(self) -> None: ...


@runtime_checkable
class EStop(Protocol):
    role: str
    kind: str

    def availability(self) -> Availability: ...
    def triggered(self, ctl: Controller) -> bool: ...
    def stop(self, ctl: Controller) -> None: ...
