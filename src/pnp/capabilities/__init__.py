"""Build the machine's capabilities from its profile (docs/13 §4.14)."""
from __future__ import annotations

from dataclasses import dataclass, field

from ..motion.controller import Controller
from ..profile.schema import Profile
from .base import Availability, Capability, CapabilityError, NoneCap, NotBuilt
from .head import ControllerVacuum, build_estop, build_light, build_rotation
from .later import build_later
from .references import build_homing, build_z_reference


@dataclass
class Capabilities:
    homing: Capability
    z_reference: Capability
    rotation: Capability
    vacuum: Capability
    estop: Capability
    lights: dict[str, Capability]
    cameras: dict[str, Capability] = field(default_factory=dict)
    later: dict[str, Capability] = field(default_factory=dict)

    def all(self) -> list[Capability]:
        return [self.homing, self.z_reference, self.rotation, self.vacuum, self.estop, *self.lights.values(),
                *self.cameras.values(), *self.later.values()]

    def describe(self) -> list[dict]:
        return [c.describe() for c in self.all()]


def build(p: Profile, ctl: Controller | None, cameras: dict[str, Capability] | None = None) -> Capabilities:
    """Cameras are built by the caller (they run threads and outlive a profile reload)."""
    firmware_ok = ctl is None or ctl.info.firmware_ok
    return Capabilities(
        homing=build_homing(p),
        z_reference=build_z_reference(p),
        rotation=build_rotation(p, ctl, firmware_ok),
        vacuum=ControllerVacuum(p, ctl),
        estop=build_estop(p),
        lights={"light_head": build_light("light_head", p.lights.head, ctl),
                "light_bottom": build_light("light_bottom", p.lights.bottom, ctl)},
        cameras=cameras or {},
        later=build_later(p),
    )


__all__ = ["Availability", "Capabilities", "Capability", "CapabilityError", "NoneCap", "NotBuilt", "build"]
