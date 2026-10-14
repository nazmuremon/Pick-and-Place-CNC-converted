"""Interfaces whose real code comes in later phases. Each says where it comes from, so the page can show it."""
from __future__ import annotations

from typing import Protocol, runtime_checkable

from ..profile.schema import Profile
from .base import Availability, Capability, NoneCap, NotBuilt


@runtime_checkable
class PickVerifier(Protocol):
    def verify_pick(self, before, after, part) -> tuple[bool | None, float]: ...   # (picked?, confidence)


@runtime_checkable
class PlaceCorrector(Protocol):
    def correction_for(self, part) -> tuple[float, float, float]: ...               # dx, dy, dθ


@runtime_checkable
class Feeder(Protocol):
    def next_part(self): ...
    remaining: int | None
    def refill(self) -> None: ...


@runtime_checkable
class LabelPrinter(Protocol):
    def print(self, labels: list, mode: str = "strip") -> None: ...
    def status(self) -> dict: ...


@runtime_checkable
class PartScanner(Protocol):
    def scan(self) -> str | None: ...


class NozzleSet(Capability):
    """The needles and how they are changed. Manual change: the program asks the operator."""
    role = "nozzles"

    def __init__(self, p: Profile) -> None:
        self.kind = p.nozzles.change
        self.items = {n.id: n for n in p.nozzles.items}
        self.current = p.nozzles.current

    def select(self, package: str) -> str | None:
        """The nozzle for a package: the current one if it lists the package (or lists nothing yet)."""
        cur = self.items[self.current]
        if not cur.packages or package in cur.packages:
            return cur.id
        return next((n.id for n in self.items.values() if package in n.packages), None)

    def availability(self) -> Availability:
        if self.kind == "rack":
            return Availability.no("A nozzle rack: its positions and change moves are TBD (they depend on the rack "
                                   "you build)", "Change nozzles by hand for now (nozzles.change: manual): the job "
                                   "pauses and asks for each change")
        return Availability.yes()


class Described(Capability):
    """A capability that works through the job and the page (no hardware of its own to drive): listed so the
    Doctor and the page show what this machine can do."""

    def __init__(self, role: str, kind: str, why: str = "") -> None:
        self.role, self.kind, self._why = role, kind, why

    def availability(self) -> Availability:
        return Availability.no(self._why) if self._why else Availability.yes()


def build_later(p: Profile) -> dict[str, Capability]:
    sensor = p.vacuum.sensor is not None
    return {
        "feeders": Described("feeders", "strip / matrix / stick trays (config/feeders.yaml)"),
        "pick_verifier": Described("pick_verifier", "pocket check" + (" + vacuum sensor" if sensor else "")),
        "place_corrector": Described("place_corrector", "bottom_camera") if p.cameras.bottom is not None else
        NoneCap("place_corrector", "No bottom camera: parts are not looked at from below"),
        "label_printer": Described("label_printer", p.label_printer.type),
        "part_scanner": Described("part_scanner", "typed / USB scanner / camera QR (in the page)"),
        "reel_feeder": NotBuilt("reel_feeder", "motorized", "a later phase: it needs the feeder hardware"),
        "nozzles": NozzleSet(p),
    }
